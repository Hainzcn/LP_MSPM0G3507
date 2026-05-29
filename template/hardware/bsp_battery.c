/**
 * @file    bsp_battery.c
 * @brief   电池电压采样实现，详见 bsp_battery.h。
 *
 * 设计要点：
 *   ─ 采样模式选择"软件触发 + 轮询完成"而非 IRQ：每 10 ms 一次、单次 < 5 µs，
 *     IRQ 化反而徒增 ISR 复杂度（ADC0 与电机/编码器/IMU UART 共享 NVIC 优先级）；
 *   ─ EMA 用整数 (alpha << 8 / 256) 实现，避免浮点；80 ms 时间常数对 100 Hz
 *     采样率而言抗噪足够，又不会让"急速放电"事件被滤掉；
 *   ─ 状态机有回滞：触发 LOW_WARN 后必须升回 WARN_MV + HYS 才能回 NORMAL，
 *     避免在 9.5 V 阈值附近 ADC 噪声导致 PWM 限幅反复打开 / 关闭。
 */

#include "bsp_battery.h"
#include "bsp_systick.h"
#include "ti_msp_dl_config.h"

#include <stdint.h>

typedef struct {
    uint32_t            ema_mv;               /* 滤波后电池电压（mV） */
    uint32_t            sample_after_ms;      /* 上电预延迟到期绝对时刻（ms），到期前不采样 */
    uint16_t            last_raw;             /* 最近一次 ADC 原始值 */
    uint16_t            startup_grace_left;   /* 上电静默拍数倒计数，0 后开始 classify */
    uint8_t             primed;               /* 0 = 还没攒够第一次有效采样 */
    uint8_t             low_stop_count;       /* 连续低于 STOP 的确认计数 */
    bsp_battery_state_t state;
} batt_state_t;

static batt_state_t s_batt;

/* -------------------------------------------------------------------------- */
/* 内部辅助                                                                    */
/* -------------------------------------------------------------------------- */

/** 触发一次软件转换。SysConfig 配置为 AUTO_NEXT，单次转换写完 MEM0 即停。 */
static void trigger_conversion(void)
{
    DL_ADC12_startConversion(ADC_BAT_INST);
}

/**
 * 阻塞等待上一次转换完成（典型 < 5 µs @ 32 MHz）。
 *   通过 raw interrupt status 而非 NVIC 中断，避免占用 ISR 资源。
 *   设最大等待循环 N = 1000，相当于 ~30 µs 超时；正常路径下 ~50 个循环就退出。
 *   超时返回 false，调用方应该跳过本拍而不是用陈旧数据。
 */
static bool wait_conversion_done(void)
{
    for (uint32_t i = 0u; i < 1000u; i++) {
        if (DL_ADC12_getRawInterruptStatus(ADC_BAT_INST,
                DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED) != 0u) {
            DL_ADC12_clearInterruptStatus(ADC_BAT_INST,
                DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);
            return true;
        }
    }
    return false;
}

/**
 * 阈值 + 回滞状态机：
 *   NORMAL    → LOW_WARN  当 mv ≤ WARN_MV
 *   LOW_WARN  → NORMAL    当 mv ≥ WARN_MV + HYS
 *   LOW_WARN  → LOW_STOP  当 mv ≤ STOP_MV
 *   LOW_STOP  → LOW_WARN  当 mv ≥ STOP_MV + HYS
 *
 *   STOP 不会直接回 NORMAL（必须经过 WARN 中间态），保证人工判断电池后才解除最严级别。
 *
 *   断线护栏：任何状态下若 mv < BSP_BATTERY_DISCONNECTED_MV（默认 1 V），认定
 *   "PB24 浮空 / 电池电路未连"，state 回到 UNKNOWN，避免误触发 LOW_STOP；正常
 *   3S 锂电（9~12.6 V）远高于 1 V，不会被误识。
 */
static bsp_battery_state_t classify(uint32_t mv, bsp_battery_state_t prev)
{
    if (mv < (uint32_t)BSP_BATTERY_DISCONNECTED_MV) {
        return BSP_BATT_STATE_UNKNOWN;
    }

    switch (prev) {
    case BSP_BATT_STATE_LOW_STOP:
        if (mv >= (BSP_BATTERY_STOP_MV + BSP_BATTERY_HYSTERESIS_MV)) {
            return BSP_BATT_STATE_LOW_WARN;
        }
        return BSP_BATT_STATE_LOW_STOP;

    case BSP_BATT_STATE_LOW_WARN:
        if (mv <= BSP_BATTERY_STOP_MV) {
            return BSP_BATT_STATE_LOW_STOP;
        }
        if (mv >= (BSP_BATTERY_WARN_MV + BSP_BATTERY_HYSTERESIS_MV)) {
            return BSP_BATT_STATE_NORMAL;
        }
        return BSP_BATT_STATE_LOW_WARN;

    case BSP_BATT_STATE_NORMAL:
        if (mv <= BSP_BATTERY_STOP_MV) {
            return BSP_BATT_STATE_LOW_STOP;
        }
        if (mv <= BSP_BATTERY_WARN_MV) {
            return BSP_BATT_STATE_LOW_WARN;
        }
        return BSP_BATT_STATE_NORMAL;

    case BSP_BATT_STATE_UNKNOWN:
    default:
        /* 首次跨过 DISCONNECTED 阈值后，按当前实测 mv 进入对应分类 */
        if (mv <= BSP_BATTERY_STOP_MV) return BSP_BATT_STATE_LOW_STOP;
        if (mv <= BSP_BATTERY_WARN_MV) return BSP_BATT_STATE_LOW_WARN;
        return BSP_BATT_STATE_NORMAL;
    }
}

/* -------------------------------------------------------------------------- */
/* 公共 API                                                                    */
/* -------------------------------------------------------------------------- */

void bsp_battery_init(void)
{
    s_batt.ema_mv             = 0u;
    s_batt.last_raw           = 0u;
    s_batt.primed             = 0u;
    s_batt.low_stop_count     = 0u;
    s_batt.startup_grace_left = (uint16_t)BSP_BATTERY_STARTUP_GRACE_TICKS;
    s_batt.state              = BSP_BATT_STATE_UNKNOWN;

    /* 上电预采样延迟：到期前 bsp_battery_update() 直接返回，不启动 ADC 转换。
     * 延迟期间分压电路中的旁路电容有时间通过 R1+R2 充电到接近真实电压，
     * 避免 ADC 首拍读到未充电的低值后 EMA 被"污染"。 */
    s_batt.sample_after_ms = bsp_systick_get_ms() + BSP_BATTERY_PRESAMPLE_DELAY_MS;

    /* 清掉 ADC 可能残留的脏中断标志；首次转换由 update() 在延迟到期后触发。 */
    DL_ADC12_clearInterruptStatus(ADC_BAT_INST,
        DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);
}

uint32_t bsp_battery_raw_to_mv(uint16_t raw)
{
    if (raw > BSP_BATTERY_ADC_FULL_SCALE) {
        raw = BSP_BATTERY_ADC_FULL_SCALE;
    }
    /* 引脚处毫伏值：raw × VREF_MV / FULL_SCALE
     *   raw ≤ 4095, VREF_MV = 3300 → 中间乘积 ≤ 13.5e6 < 2^24，uint32 安全 */
    uint32_t mv_pin = ((uint32_t)raw * (uint32_t)BSP_BATTERY_ADC_REF_MV)
                       / (uint32_t)BSP_BATTERY_ADC_FULL_SCALE;
    /* 反算电池端：v_bat = v_pin / divider_ratio = v_pin × 10000 / RATIO_X10000
     *   mv_pin ≤ 3300 → 乘 10000 ≤ 33e6 < 2^32，uint32 安全 */
    return (mv_pin * 10000u) / (uint32_t)BSP_BATTERY_DIVIDER_RATIO_X10000;
}

void bsp_battery_update(void)
{
    /* 上电预采样延迟：等待分压电容充电，到期前不启动任何 ADC 转换。
     * 使用有符号差值比较，可正确处理 bsp_systick_get_ms() 的 uint32 溢出回绕。 */
    if ((int32_t)(bsp_systick_get_ms() - s_batt.sample_after_ms) < 0) {
        return;
    }

    if (!wait_conversion_done()) {
        /* 上一次转换未完成（或首次进入：尚未触发过转换）：触发一次，等下一拍读取 */
        trigger_conversion();
        return;
    }

    uint16_t raw = (uint16_t)DL_ADC12_getMemResult(ADC_BAT_INST,
                       DL_ADC12_MEM_IDX_0);
    s_batt.last_raw = raw;

    uint32_t mv_now = bsp_battery_raw_to_mv(raw);

    if (s_batt.primed == 0u) {
        /* 首拍直接吃 mv_now，避免 0 → 12000 的慢爬过程被分类成 STOP */
        s_batt.ema_mv = mv_now;
        s_batt.primed = 1u;
    } else {
        /* EMA：new = (α × now + (256 - α) × old) >> 8
         *   α = 32 → 时间常数 ≈ 8 拍 = 80 ms @ 100 Hz
         *   uint32 中间乘积：α × mv ≤ 256 × 16000 = 4.1e6，安全 */
        uint32_t a = (uint32_t)BSP_BATTERY_EMA_ALPHA_256;
        s_batt.ema_mv = ((a * mv_now) + ((256u - a) * s_batt.ema_mv) + 128u) >> 8;
    }

    /* 上电静默：等待 ADC / 分压电容充电稳定。
     *   EMA 已在上方完成更新（读值逐渐趋近真实电压），但 state 强制保持 UNKNOWN，
     *   上层 app_safety_tick() 对 UNKNOWN 不做状态迁移，避免误触发 BAT_WARN。
     *   与 APP_SAFETY_BOOT_CHECK_MS 对齐（默认均为 5 s）。 */
    if (s_batt.startup_grace_left > 0u) {
        s_batt.startup_grace_left--;
        s_batt.state = BSP_BATT_STATE_UNKNOWN;
        trigger_conversion();
        return;
    }

    bsp_battery_state_t next = classify(s_batt.ema_mv, s_batt.state);
    if ((next == BSP_BATT_STATE_LOW_STOP) &&
        (s_batt.state != BSP_BATT_STATE_LOW_STOP)) {
        if (s_batt.low_stop_count < BSP_BATTERY_LOW_STOP_DEBOUNCE_SAMPLES) {
            s_batt.low_stop_count++;
        }
        if (s_batt.low_stop_count < BSP_BATTERY_LOW_STOP_DEBOUNCE_SAMPLES) {
            next = BSP_BATT_STATE_LOW_WARN;
        }
    } else if (next != BSP_BATT_STATE_LOW_STOP) {
        s_batt.low_stop_count = 0u;
    }
    s_batt.state = next;

    trigger_conversion();   /* 为下一拍预备 */
}

uint32_t bsp_battery_get_mv(void)
{
    return s_batt.ema_mv;
}

uint16_t bsp_battery_get_raw(void)
{
    return s_batt.last_raw;
}

bsp_battery_state_t bsp_battery_get_state(void)
{
    return s_batt.state;
}
