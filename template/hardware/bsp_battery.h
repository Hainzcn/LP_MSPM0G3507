/**
 * @file    bsp_battery.h
 * @brief   电池电压采样 + 阈值状态机（ADC0 / 通道 5 / PB24）
 *
 * 硬件资源（来自 SysConfig 与 docs/TaskLog/Stage0-PinAllocation.md §4.6）：
 *   ─ ADC：ADC0，单通道 MEM_IDX_0，参考 VDDA = 3.3 V，12-bit
 *   ─ 引脚：PB24（A0_5）
 *   ─ 分压：外部 R1 = 100 kΩ + R2 = 22 kΩ → 比值 K = 22/(100+22) ≈ 0.1803
 *      电池电压 V_bat = ADC_mV / K ≈ ADC_mV × 5.545
 *
 * 接口约定：
 *   ─ `bsp_battery_init()` 只触发首次软件转换；ADC 已在 `SYSCFG_DL_init()` 中
 *      enable conversions（AUTO_NEXT 模式），后续靠 `bsp_battery_update()` 触发
 *      新一轮采样并读取 MEM0；
 *   ─ `bsp_battery_update()` 建议 100 Hz 调用，内部做 EMA 滤波，整数实现，
 *      不动浮点；
 *   ─ 读 `bsp_battery_get_mv()` 拿 EMA 后的电池毫伏值；
 *   ─ 读 `bsp_battery_get_state()` 拿三态状态：NORMAL / LOW_WARN / LOW_STOP，
 *      由业务侧（如 app_safety）决定是否降功率 / 急停。
 *
 * 调用次序：
 *   SYSCFG_DL_init() → bsp_battery_init() → 主循环周期调 bsp_battery_update()
 */

#ifndef BSP_BATTERY_H
#define BSP_BATTERY_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/* 编译期可配宏                                                                 */
/* ========================================================================== */

/** ADC 参考电压（与 SysConfig ADCMEM_0_REF_VOLTAGE_V 强绑定） */
#ifndef BSP_BATTERY_ADC_REF_MV
#define BSP_BATTERY_ADC_REF_MV                   (3300)
#endif

/** ADC 满量程（12-bit ADC0：4095） */
#ifndef BSP_BATTERY_ADC_FULL_SCALE
#define BSP_BATTERY_ADC_FULL_SCALE               (4095)
#endif

/**
 * 分压系数 = R2 / (R1 + R2)，定点表示放大 10000 倍
 *   默认 R1 = 100 k, R2 = 22 k → K = 22 / 122 ≈ 0.18033 → 1803
 *   电池满电 12.6 V 对应 ADC ≈ 12600 × 0.1803 / 3300 × 4095 ≈ 2820（< 4095，安全）
 */
#ifndef BSP_BATTERY_DIVIDER_RATIO_X10000
#define BSP_BATTERY_DIVIDER_RATIO_X10000         (1803)
#endif

/** EMA 滤波系数（α × 256）：value = (α × new + (256-α) × old) >> 8。
 *   默认 32 ⇒ α ≈ 0.125（低通时间常数 ≈ 8 拍 = 80 ms @ 100 Hz）。 */
#ifndef BSP_BATTERY_EMA_ALPHA_256
#define BSP_BATTERY_EMA_ALPHA_256                (32)
#endif

/** 低压告警阈值（毫伏）：建议 PWM 限幅降级触发点 */
#ifndef BSP_BATTERY_WARN_MV
#define BSP_BATTERY_WARN_MV                      (9500)
#endif

/** 安全停车阈值（毫伏）：低于此值业务层应立刻 brake + STBY 关闭 */
#ifndef BSP_BATTERY_STOP_MV
#define BSP_BATTERY_STOP_MV                      (9000)
#endif

/** 状态回滞（毫伏），避免阈值附近反复抖动 */
#ifndef BSP_BATTERY_HYSTERESIS_MV
#define BSP_BATTERY_HYSTERESIS_MV                (200)
#endif

/**
 * 断线 / 未接电池 阈值（毫伏）。
 *   PB24 浮空 / 电池分压电路未焊 / 电池未插的情况下，ADC raw ≈ 0，换算电池
 *   mv 约 0~几百，会被旧版 classify() 判为 `LOW_STOP` 触发 app_safety 急停，
 *   导致开发期"想跑但被电池保护拦下"的体验。
 *
 *   修复策略：当 ema_mv 低于此阈值时，state 保持 `UNKNOWN`（不进 LOW_STOP，
 *   也不触发 PWM 限幅），相当于"电池电路未连接，不参与安全决策"。一旦真正
 *   接上电池（哪怕电池快没电也会 > 1 V），ema_mv 会越过该阈值进入正式分类。
 *
 *   合法电池电压区间：例如 3S 锂电 9.0~12.6 V，远高于 1 V，不会误判。
 */
#ifndef BSP_BATTERY_DISCONNECTED_MV
#define BSP_BATTERY_DISCONNECTED_MV              (1000)
#endif

/* ========================================================================== */
/* 状态枚举                                                                     */
/* ========================================================================== */

typedef enum {
    BSP_BATT_STATE_UNKNOWN  = 0,    /* 上电后还没攒够采样 */
    BSP_BATT_STATE_NORMAL   = 1,    /* mv > WARN_MV */
    BSP_BATT_STATE_LOW_WARN = 2,    /* WARN_MV ≥ mv > STOP_MV */
    BSP_BATT_STATE_LOW_STOP = 3     /* mv ≤ STOP_MV */
} bsp_battery_state_t;

/* ========================================================================== */
/* API                                                                         */
/* ========================================================================== */

/**
 * @brief 初始化：清状态、触发首次软件转换。
 *        前提：`SYSCFG_DL_init()` 已完成（含 `SYSCFG_DL_ADC_BAT_init()`）。
 */
void bsp_battery_init(void);

/**
 * @brief 周期采样任务（建议 100 Hz）：
 *          ① 等上一次转换完成（轮询 MEM0 result loaded 标志，典型 < 5 µs）
 *          ② 读 MEM0 → ADC raw → 换算为电池 mV → EMA 更新
 *          ③ 触发下一次转换（保证下一拍能拿到结果）
 *          ④ 按阈值 + 回滞更新状态枚举
 *
 *        本函数必须从主循环单线程调用，不要在 ISR 内调。
 */
void bsp_battery_update(void);

/** @return 滤波后的电池电压，单位毫伏（mV）。上电首次 update 前返回 0。 */
uint32_t bsp_battery_get_mv(void);

/** @return 最近一次 ADC 原始读数（0~4095），调试用。上电首次 update 前返回 0。 */
uint16_t bsp_battery_get_raw(void);

/** @return 当前阈值状态（见 `bsp_battery_state_t`）。 */
bsp_battery_state_t bsp_battery_get_state(void);

/**
 * @brief 把 ADC raw 转换为电池电压 (mV) 的纯函数（调试 / 单元测试入口）。
 *        计算路径：mv_at_pin = raw × ADC_REF_MV / FULL_SCALE
 *                   v_bat    = mv_at_pin × 10000 / DIVIDER_RATIO_X10000
 *        全程 uint32_t，无浮点，最高输入 raw=4095 → 中间值 ≤ 13.5e6 不会溢出。
 */
uint32_t bsp_battery_raw_to_mv(uint16_t raw);

#ifdef __cplusplus
}
#endif

#endif /* BSP_BATTERY_H */
