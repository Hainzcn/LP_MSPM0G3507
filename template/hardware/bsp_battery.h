/**
 * @file    bsp_battery.h
 * @brief   电池电压采样 + 阈值状态机（ADC0 / 通道 5 / PB24）
 *
 * ============================================================
 * 这个文件是做什么的？（写给初学者）
 * ============================================================
 * 自平衡小车使用锂电池供电。锂电池有一个重要的特性：
 * 电压过低时继续放电会损坏电池（甚至引发安全问题）。
 *
 * 所以我们需要实时监测电池电压：
 *   - 电压正常 → 一切照常
 *   - 电压偏低 → 降功率运行（PWM 限幅），提醒用户充电
 *   - 电压过低 → 立即停车保护电池
 *
 * 由于单片机的 ADC 引脚只能测量 0~3.3V 的电压，
 * 而锂电池电压是 9~12.6V，所以需要用"电阻分压"把电压降低
 * 到 ADC 能测量的范围内。
 *
 * 硬件连接：
 *   电池正极 → R1(100kΩ) → ADC 引脚(PB24) → R2(22kΩ) → GND
 *
 *   分压比 K = R2/(R1+R2) = 22/122 ≈ 0.18
 *   电池 12V → ADC 引脚 = 12 × 0.18 ≈ 2.16V（在 0~3.3V 范围内）
 *
 * 这个文件封装了：
 *   1. ADC 采样与电压换算（原始值 → 毫伏）
 *   2. EMA 低通滤波（消除 ADC 噪声）
 *   3. 阈值状态机（NORMAL → LOW_WARN → LOW_STOP）
 *   4. 断线检测（未接电池时不被误判为"电压过低"）
 *
 * ============================================================
 * 为什么用软件触发 + 轮询，而不是中断？
 * ============================================================
 * ADC 转换本身很快（< 5 µs），每 10 ms 采样一次。
 * 如果使用中断：
 *   - 每次转换完都要进一次 ISR
 *   - ADC0 的中断优先级要与电机/编码器/IMU UART 协调
 *   - 增加代码复杂度
 *
 * 用轮询方式：
 *   - 在 bsp_battery_update() 中等待转换完成（约 5 µs）
 *   - 不占用 NVIC 中断资源
 *   - 代码简单明了
 */

#ifndef BSP_BATTERY_H
#define BSP_BATTERY_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
 * 编译期可配宏 —— 如果换了分压电阻或电池类型，只改这里就行
 * ================================================================ */

/** ADC 参考电压（毫伏）。MSPM0G3507 的 VDDA = 3.3V = 3300 mV。
 *  必须和 SysConfig 中 ADCMEM_0_REF_VOLTAGE_V 的配置一致。 */
#ifndef BSP_BATTERY_ADC_REF_MV
#define BSP_BATTERY_ADC_REF_MV                   (3300)
#endif

/** ADC 满量程。MSPM0G3507 的 ADC 是 12 位的，范围 0~4095。 */
#ifndef BSP_BATTERY_ADC_FULL_SCALE
#define BSP_BATTERY_ADC_FULL_SCALE               (4095)
#endif

/**
 * 分压系数（放大 10000 倍的定点数）。
 *
 * 电阻分压公式：V_adc = V_bat × R2 / (R1 + R2)
 * 通常 R1=100kΩ, R2=22kΩ
 * 分压比 K = 22 / (100 + 22) = 22 / 122 ≈ 0.18033
 *
 * 为了不用浮点数（Cortex-M0+ 软浮点慢），放大 10000 倍：
 *   K × 10000 = 0.18033 × 10000 = 1803
 *
 * 反算电池电压：V_bat = V_adc / K = V_adc × 10000 / 1803
 *
 * 满电 12.6V 对应 ADC 引脚电压 = 12.6 × 0.18 ≈ 2.27V
 * ADC 原始值 = 2.27 / 3.3 × 4095 ≈ 2820（< 4095，安全） */
#ifndef BSP_BATTERY_DIVIDER_RATIO_X10000
#define BSP_BATTERY_DIVIDER_RATIO_X10000         (1803)
#endif

/**
 * EMA 滤波系数（乘以 256 的整数形式）。
 *   EMA 公式：value = (α × new + (256-α) × old) >> 8
 *   默认 α = 32 → α/256 ≈ 0.125
 *
 * 时间常数 ≈ (1/α) × 采样间隔 × 256
 *             = (1/32) × 256 × 10ms = 80ms
 * 即滤波输出跟随输入变化到 63% 需要约 80ms。
 * 这个时间常数对 100Hz 采样率来说，滤波效果足够，
 * 又不会把"电池急速放电"事件给滤掉。
 */
#ifndef BSP_BATTERY_EMA_ALPHA_256
#define BSP_BATTERY_EMA_ALPHA_256                (32)
#endif

/** 低压告警阈值（毫伏）。低于此值 → 进入 LOW_WARN 状态。
 *  业务层应据此限幅 PWM 降功率运行。
 *  默认 9.5V（3S 锂电约 3.17V/节）。 */
#ifndef BSP_BATTERY_WARN_MV
#define BSP_BATTERY_WARN_MV                      (9500)
#endif

/** 安全停车阈值（毫伏）。低于此值 → 进入 LOW_STOP 状态。
 *  业务层应立即刹车 + 关闭 STBY。
 *  默认 9.0V（3S 锂电约 3.0V/节，不能再低了）。 */
#ifndef BSP_BATTERY_STOP_MV
#define BSP_BATTERY_STOP_MV                      (9000)
#endif

/** 状态回滞（毫伏）。防止 ADC 噪声导致状态在阈值附近反复跳变。
 *  例如从 NORMAL 进入 LOW_WARN 后，需要电压回升到
 *  WARN_MV + HYS = 9500 + 200 = 9700 mV 才能回到 NORMAL。
 *  这 200mV 的"缓冲区间"避免了频繁切换。 */
#ifndef BSP_BATTERY_HYSTERESIS_MV
#define BSP_BATTERY_HYSTERESIS_MV                (200)
#endif

/**
 * 断线/未接电池阈值（毫伏）。
 *
 * 如果电池没有连接（或分压电路没有焊接），PB24 引脚浮空，
 * ADC 读数会很低，换算成电池电压可能只有几十到几百毫伏。
 *
 * 没有这个检测的话，classify() 会把这种情况判为 LOW_STOP，
 * 触发紧急停车——但电池根本没接，不应该触发任何保护。
 *
 * 修复后：当 ema_mv < 1000 mV 时，状态保持 UNKNOWN，
 * 不进 LOW_STOP，不触发任何保护措施。
 * 合法电池电压 9~12.6V 远高于 1V，不会误判。
 */
#ifndef BSP_BATTERY_DISCONNECTED_MV
#define BSP_BATTERY_DISCONNECTED_MV              (1000)
#endif

/* ================================================================
 * 状态枚举
 * ================================================================
 * 电池状态分为 4 个等级，从"未知"到"停车保护"。
 * 业务层（如 app_safety.c）根据当前状态决定行为。
 */

typedef enum {
    BSP_BATT_STATE_UNKNOWN  = 0,    /* 上电后还没完成第一次有效采样 */
    BSP_BATT_STATE_NORMAL   = 1,    /* 电压正常（mv > WARN_MV） */
    BSP_BATT_STATE_LOW_WARN = 2,    /* 低压告警（WARN_MV ≥ mv > STOP_MV） */
    BSP_BATT_STATE_LOW_STOP = 3     /* 电压过低（mv ≤ STOP_MV），应立即停车 */
} bsp_battery_state_t;

/* ================================================================
 * API 函数
 * ================================================================ */

/**
 * @brief 初始化电池电压监测。
 *
 * 这个函数在 SYSCFG_DL_init() 之后调用。
 * 它做三件事：
 *   1. 状态清零（状态设为 UNKNOWN，EMA 设为 0）
 *   2. 清除 ADC 中断标志（防止残留标志干扰）
 *   3. 触发第一次 ADC 转换（下调用 update() 时就能读到结果）
 *
 * @note  第一次转换结果会在 bsp_battery_update() 中被读取。
 *        在那之前，bsp_battery_get_mv() 返回 0。
 */
void bsp_battery_init(void);

/**
 * @brief 周期采样任务（⭐ 建议 100 Hz 调用）
 *
 * 这个函数应该在主循环中周期调用（每 10 ms 一次）。
 * 它完成以下工作：
 *   1. 等待上一次 ADC 转换完成（轮询，< 5 µs）
 *   2. 读取 ADC 原始值
 *   3. 换算为电池电压（mV）
 *   4. 更新 EMA 低通滤波值
 *   5. 根据阈值 + 回滞更新状态枚举
 *   6. 触发下一次 ADC 转换
 *
 * @code
 *   // 在主循环中（每 10 ms 调用一次）
 *   if (tick_count % 10 == 0) {
 *       bsp_battery_update();
 *   }
 * @endcode
 *
 * @note  这个函数必须从主循环单线程调用，不要在 ISR 内调。
 */
void bsp_battery_update(void);

/** @brief 返回滤波后的电池电压（毫伏）。
 *  在第一次 update() 之前返回 0。 */
uint32_t bsp_battery_get_mv(void);

/** @brief 返回最近一次 ADC 原始读数（0~4095），调试用。
 *  在第一次 update() 之前返回 0。 */
uint16_t bsp_battery_get_raw(void);

/** @brief 返回当前阈值状态。 */
bsp_battery_state_t bsp_battery_get_state(void);

/**
 * @brief 把 ADC 原始值换算为电池电压（毫伏）——纯函数。
 *
 * 这个函数是"纯函数"——它不依赖任何全局状态，
 * 给定相同的 raw，总是返回相同的结果。
 *
 * 它可以在调试/单元测试中独立调用，不需要初始化 ADC 硬件。
 *
 * 计算路径：
 *   1. ADC 引脚电压 = raw × REF_MV / FULL_SCALE
 *   2. 电池电压 = 引脚电压 × 10000 / 分压系数
 *
 * 全程 uint32_t 整数运算，无浮点。
 *
 * @param raw  ADC 原始值（0~4095）
 * @return 电池电压（毫伏）
 */
uint32_t bsp_battery_raw_to_mv(uint16_t raw);

#ifdef __cplusplus
}
#endif

#endif /* BSP_BATTERY_H */
