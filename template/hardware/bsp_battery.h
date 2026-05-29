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
 * 采样策略——预热自适应 + 业务低频两阶段设计：
 *
 *   阶段 A（上电预热，自适应）：
 *     分压电路旁路电容通过 R_th = R1‖R2 充电，τ 实测偏大且受电源缓启动影响，
 *     固定时刻采样会读到严重偏低的半充电压。改为从上电起按 BOOT_POLL_MS 连续
 *     采样，待读值爬升到平台期（跨 BASELINE_MS 窗口的总上升量 < STABLE_DELTA_MV）
 *     判定充满，此刻读值才作有效自检电压并据此置 boot_ok；WARMUP_MAX_MS 兜底。
 *     预热期 is_ready()=false、state=UNKNOWN，安全层不据此急停。
 *
 *   阶段 B（业务周期，UPDATE_PERIOD_MS = 3 s，直接读值，无 EMA）：
 *     电池电压在正常使用中变化极缓（分钟级放电曲线）；3 s 采样率足以感知
 *     放电趋势，同时旁路电容本身充当硬件低通，电机电流瞬态在到达 ADC 前已
 *     大幅衰减，无需再做软件 EMA 滤波。
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

/* ---- 上电预热采样（自适应，替代旧的"固定 3 s 单次快照"） ----
 *
 * 旧设计在固定 t=3 s 采一次，假设旁路电容 τ≈3 s 已充至 63%。实测冷上电时
 * 电容充电远未完成（τ 比预期大，或电源缓启动），t=3 s 读值严重偏低，导致
 * boot_ok 误判 + 心跳电压虚低。新设计不再赌固定时刻，而是从上电起以
 * BOOT_POLL_MS 周期连续采样，待"读值爬升到平台期"（连续若干次样本变化量
 * < STABLE_DELTA_MV）判定电容已充满，此刻的读值才作为有效自检电压；并设
 * WARMUP_MAX_MS 兜底超时，防止电池缺接/异常时永久卡在预热。 */

/** 预热期 ADC 轮询周期（ms）。 */
#ifndef BSP_BATTERY_BOOT_POLL_MS
#define BSP_BATTERY_BOOT_POLL_MS                 (250u)
#endif

/**
 * 平台判据基线窗口（ms）：与"窗口起点"读值比较，而非相邻拍。
 *
 *   仅比相邻两拍（250 ms）会被"缓慢上升"骗过——若电源/电容 τ 较大，相邻差
 *   可能 < 阈值却仍在半电压爬升（曾实测自检读到 ~6.5 V）。改为跨 ≥ 此窗口比较
 *   总上升量：只有整段窗口几乎不再上升才判为充满，可靠拒绝"仍在缓升"。
 */
#ifndef BSP_BATTERY_BOOT_BASELINE_MS
#define BSP_BATTERY_BOOT_BASELINE_MS             (2000u)
#endif

/** 平台判据：基线窗口内总上升量 < 此值（mV）即判定电容/电源已稳、读值有效。 */
#ifndef BSP_BATTERY_BOOT_STABLE_DELTA_MV
#define BSP_BATTERY_BOOT_STABLE_DELTA_MV         (150u)
#endif

/** 预热兜底超时（ms）：即使未检出平台，超过此时长也以最近读值结束预热。 */
#ifndef BSP_BATTERY_BOOT_WARMUP_MAX_MS
#define BSP_BATTERY_BOOT_WARMUP_MAX_MS           (15000u)
#endif

/**
 * 自检"电池正常"判定阈值（mV）。
 *   预热结束（电容充满）后的有效读值 ≥ 此值时 bsp_battery_is_boot_ok()
 *   返回 true，app_safety 可在 BOOT_CHECK 计时器到期前提前完成自检。
 *   现在读值取自充满后的稳定电压，可直接按实际电池阈值设定（11 V 满电附近）。
 */
#ifndef BSP_BATTERY_BOOT_OK_MV
#define BSP_BATTERY_BOOT_OK_MV                   (11000u)
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
 *          ─ 预热阶段：每 BOOT_POLL_MS 采样一次，检测读值爬升到平台（电容充满）；
 *            充满（或 WARMUP_MAX_MS 兜底）后置 boot_ok 并落地首个业务分类；
 *          ─ 业务阶段：每 UPDATE_PERIOD_MS 触发转换 → 读值 → 分类状态。
 *        本函数必须从主循环单线程调用，不要在 ISR 内调。
 */
void bsp_battery_update(void);

/**
 * @brief 查询自检结果：预热已结束（电容充满）且充满读值 ≥ BOOT_OK_MV 时返回 true。
 *        供 app_safety_tick() 在 BOOT_CHECK 期间检测，用于提前结束自检。
 *        预热结束前始终返回 false。
 */
bool bsp_battery_is_boot_ok(void);

/**
 * @brief 预热是否结束（旁路电容已充满、读值已可信）。
 *        预热期间返回 false，此时 get_mv() 仅供观察充电爬升，不应据此做安全决策。
 */
bool bsp_battery_is_ready(void);

/** @return 最近一次采样换算的电池电压（mV）；预热期返回正在爬升的瞬时值，首拍前为 0。 */
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
