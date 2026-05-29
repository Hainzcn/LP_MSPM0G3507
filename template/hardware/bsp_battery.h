/**
 * @file    bsp_battery.h
 * @brief   电池电压采样 + 阈值状态机（ADC0 / 通道 5 / PB24）
 *
 * 硬件资源（来自 SysConfig 与 docs/TaskLog/Stage0-PinAllocation.md §4.6）：
 *   ─ ADC：ADC0，单通道 MEM_IDX_0，参考 VDDA = 3.3 V，12-bit
 *   ─ 引脚：PB24（A0_5）
 *   ─ 分压：外部 R1 = 100 kΩ + R2 = 22 kΩ → 比值 K = 22/(100+22) ≈ 0.1803
 *     （实测校准后 K ≈ 0.1503，见 BSP_BATTERY_DIVIDER_RATIO_X10000 注释）
 *
 * 采样策略——两阶段低频设计：
 *
 *   阶段 A（自检快照，t = BOOT_SAMPLE_MS）：
 *     分压电路旁路电容通过 R_th = R1‖R2 ≈ 18 kΩ 充电，时间常数 τ ≈ 3 s。
 *     在 t = τ ≈ 3 s 处，电容充至约 63 %——对 12 V 电池约 7.6 V，
 *     读值超过 BOOT_OK_MV（7 V）即可判定电池正常，通知 app_safety 提前
 *     结束 BOOT_CHECK（跳过剩余等待，最多节省 2 s）。
 *
 *   阶段 B（业务周期，UPDATE_PERIOD_MS = 3 s，直接读值，无 EMA）：
 *     电池电压在正常使用中变化极缓（分钟级放电曲线）；3 s 采样率足以感知
 *     放电趋势，同时旁路电容本身充当硬件低通（τ ≈ 3 s），电机电流瞬态
 *     在到达 ADC 前已大幅衰减，无需再做软件 EMA 滤波。
 *
 * 接口约定：
 *   ─ `bsp_battery_init()` 清 ADC 标志并设置自检快照触发时刻（不立即转换）；
 *   ─ `bsp_battery_update()` 随主循环调用，内部按时间戳驱动，大多数调用立即返回；
 *   ─ `bsp_battery_is_boot_ok()` 快照完成且读值 > BOOT_OK_MV 时返回 true，
 *      供 app_safety_tick() 在 BOOT_CHECK 期间提前解锁；
 *   ─ `bsp_battery_get_mv()` 返回最近一次有效采样值；
 *   ─ `bsp_battery_get_state()` 返回分类状态：UNKNOWN / NORMAL / LOW_WARN / LOW_STOP。
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
 * 分压系数 = R2 / (R1 + R2)，定点表示放大 10000 倍。
 *
 *   标称值：R1 = 100 kΩ, R2 = 22 kΩ → K = 22/122 ≈ 0.1803 → 1803
 *   实测校准：上电稳定后 ADC 读约 10 V，万用表量同一电池实际约 12 V。
 *     实际 K ≈ 1803/12000 ≈ 0.15025 → RATIO_X10000 = 1503
 *   验证：mv_pin = 1803 mV → v_bat = 1803 × 10000/1503 ≈ 11993 mV ≈ 12 V ✓
 *   阈值校核（以实际电压表示）：
 *     WARN 9500 mV → 实际 ≈ 9.5 V ✓
 *     STOP 9000 mV → 实际 ≈ 9.0 V ✓
 */
#ifndef BSP_BATTERY_DIVIDER_RATIO_X10000
#define BSP_BATTERY_DIVIDER_RATIO_X10000         (1503)
#endif

/** 低压告警阈值（mV）：建议 PWM 限幅降级触发点 */
#ifndef BSP_BATTERY_WARN_MV
#define BSP_BATTERY_WARN_MV                      (9500)
#endif

/** 安全停车阈值（mV）：低于此值业务层应立刻 brake + STBY 关闭 */
#ifndef BSP_BATTERY_STOP_MV
#define BSP_BATTERY_STOP_MV                      (9000)
#endif

/** 状态回滞（mV），避免阈值附近反复抖动 */
#ifndef BSP_BATTERY_HYSTERESIS_MV
#define BSP_BATTERY_HYSTERESIS_MV                (200)
#endif

/**
 * LOW_STOP 连续确认次数。
 *   业务阶段采样周期 3 s，2 次确认 = 6 s 窗口。
 *   旁路电容（τ ≈ 3 s）已作为硬件低通衰减瞬态，采样值代表慢变趋势，
 *   无需大量去抖；2 次足以排除单点突刺，同时保证响应不过于迟钝。
 */
#ifndef BSP_BATTERY_LOW_STOP_DEBOUNCE_SAMPLES
#define BSP_BATTERY_LOW_STOP_DEBOUNCE_SAMPLES    (2u)
#endif

/**
 * 自检快照时刻（ms）：bsp_battery_init() 调用后，经过此时间触发首次 ADC 转换。
 *
 *   选取 τ ≈ 3 s（分压电路旁路电容充电时间常数）：
 *     t = τ 时电容充至 1 − e^{-1} ≈ 63 %，对 12 V 电池读值 ≈ 7.6 V
 *     > BOOT_OK_MV（7 V）——满足提前解锁条件。
 *   若电路无外部电容，可减小至 100 ms；若电容更大（τ 更大），相应增大。
 */
#ifndef BSP_BATTERY_BOOT_SAMPLE_MS
#define BSP_BATTERY_BOOT_SAMPLE_MS               (3000u)
#endif

/**
 * 自检快照"电池正常"判定阈值（mV）。
 *   读值超过此值时 bsp_battery_is_boot_ok() 返回 true，app_safety 可在
 *   BOOT_CHECK 计时器到期前提前完成自检（约节省 2 s）。
 *
 *   7000 mV 对应 t = τ = 3 s 时实际电池 ≈ 7000/(1−e^{-1}) ≈ 11.1 V，
 *   即仅当电池接近满电（≥ 11.1 V 实际）才触发提前解锁。
 */
#ifndef BSP_BATTERY_BOOT_OK_MV
#define BSP_BATTERY_BOOT_OK_MV                   (7000u)
#endif

/**
 * 业务阶段采样周期（ms）：自检快照完成后，每隔此时间重新采样并更新分类状态。
 *   3 s 足以覆盖正常使用中的放电趋势（典型 ≈ 0.1 V/min @ 1 A）；
 *   ADC 占用从 100 Hz 降至 0.33 Hz，与旁路电容 τ ≈ 3 s 相匹配。
 */
#ifndef BSP_BATTERY_UPDATE_PERIOD_MS
#define BSP_BATTERY_UPDATE_PERIOD_MS             (3000u)
#endif

/**
 * 断线 / 未接电池阈值（mV）。
 *   PB24 浮空时 ADC ≈ 0，换算 mv ≈ 0，会被 classify() 误判为 LOW_STOP 引发急停。
 *   低于此阈值视为"电池未连接"，state 保持 UNKNOWN，不参与安全决策。
 *   合法 3S 锂电（9~12.6 V）远高于 1 V，不会误判。
 */
#ifndef BSP_BATTERY_DISCONNECTED_MV
#define BSP_BATTERY_DISCONNECTED_MV              (1000)
#endif

/* ========================================================================== */
/* 状态枚举                                                                     */
/* ========================================================================== */

typedef enum {
    BSP_BATT_STATE_UNKNOWN  = 0,    /* 上电后尚无有效业务采样（自检阶段） */
    BSP_BATT_STATE_NORMAL   = 1,    /* mv > WARN_MV */
    BSP_BATT_STATE_LOW_WARN = 2,    /* WARN_MV ≥ mv > STOP_MV */
    BSP_BATT_STATE_LOW_STOP = 3     /* mv ≤ STOP_MV */
} bsp_battery_state_t;

/* ========================================================================== */
/* API                                                                         */
/* ========================================================================== */

/**
 * @brief 初始化：清状态并设定自检快照触发时刻（不立即触发 ADC 转换）。
 *        前提：SYSCFG_DL_init() 和 bsp_systick_init() 已完成。
 */
void bsp_battery_init(void);

/**
 * @brief 周期采样任务，随主循环调用（调用频率不限，内部按时间戳驱动）：
 *          ─ 自检阶段（前 BOOT_SAMPLE_MS）：定时未到，立即返回；
 *          ─ 快照采样点（t = BOOT_SAMPLE_MS）：触发一次转换，读值，置 boot_ok 标志；
 *          ─ 业务阶段：每 UPDATE_PERIOD_MS 触发转换 → 读值 → 分类状态。
 *        本函数必须从主循环单线程调用，不要在 ISR 内调。
 */
void bsp_battery_update(void);

/**
 * @brief 查询自检快照结果：快照已完成 且 读值 ≥ BOOT_OK_MV 时返回 true。
 *        供 app_safety_tick() 在 BOOT_CHECK 期间检测，用于提前结束自检。
 *        自检快照完成前（t < BOOT_SAMPLE_MS）始终返回 false。
 */
bool bsp_battery_is_boot_ok(void);

/** @return 最近一次有效采样换算的电池电压（mV）；首次采样前返回 0。 */
uint32_t bsp_battery_get_mv(void);

/** @return 最近一次 ADC 原始读数（0~4095），调试用；首次采样前返回 0。 */
uint16_t bsp_battery_get_raw(void);

/** @return 当前阈值状态（见 bsp_battery_state_t）。 */
bsp_battery_state_t bsp_battery_get_state(void);

/**
 * @brief 把 ADC raw 转换为电池电压（mV）的纯函数（调试 / 单元测试入口）。
 *        计算路径：mv_at_pin = raw × ADC_REF_MV / FULL_SCALE
 *                   v_bat    = mv_at_pin × 10000 / DIVIDER_RATIO_X10000
 *        全程 uint32_t，无浮点，最高输入 raw=4095 → 中间值 ≤ 13.5e6 不会溢出。
 */
uint32_t bsp_battery_raw_to_mv(uint16_t raw);

#ifdef __cplusplus
}
#endif

#endif /* BSP_BATTERY_H */
