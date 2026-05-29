/**
 * @file    bsp_battery.c
 * @brief   电池电压采样实现，详见 bsp_battery.h。
 *
 * 设计要点：
 *   ─ 两阶段采样：上电预热（自适应轮询直到读值平台 = 电容充满）+ 业务周期（UPDATE_PERIOD_MS）；
 *   ─ 采样模式"软件触发 + 轮询完成"，单次 < 5 µs；
 *   ─ 去掉 EMA 滤波：旁路电容（τ ≈ 3 s）本身是硬件低通，3 s 采样点读值已是时间平均，
 *     软件层直接使用原始换算值；
 *   ─ 状态机保留回滞（HYS），LOW_STOP 需 DEBOUNCE_SAMPLES（默认 2）次连续确认（= 6 s）。
 */

#include "bsp_battery.h"
#include "bsp_systick.h"
#include "ti_msp_dl_config.h"

#include <stdint.h>

typedef struct {
    uint32_t            latest_mv;       /* 最近一次采样换算值（mV）；预热期为爬升瞬时值 */
    uint32_t            next_sample_ms;  /* 下次采样触发的绝对时刻（ms） */
    uint32_t            warmup_until_ms; /* 预热兜底超时绝对时刻（ms） */
    uint32_t            ref_mv;          /* 预热平台检测：基线窗口起点读值 */
    uint32_t            ref_ms;          /* 预热平台检测：基线窗口起点时刻 */
    uint16_t            last_raw;        /* 最近一次 ADC 原始值 */
    uint8_t             warmup_done;     /* 0 = 仍在预热（电容充电中） */
    uint8_t             have_ref;        /* 预热平台检测：基线起点是否已记录 */
    uint8_t             boot_ok;         /* 1 = 充满后读值 ≥ BOOT_OK_MV，可提前解锁 */
    uint8_t             low_stop_count;  /* LOW_STOP 连续确认计数 */
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
 * 阻塞等待转换完成（典型 < 5 µs @ 32 MHz）。
 *   通过 raw interrupt status 而非 NVIC，避免占用 ISR 资源。
 *   最大等待 1000 个循环（约 30 µs 超时）；超时返回 false，调用方推迟重试。
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
 *   NORMAL   → LOW_WARN  当 mv ≤ WARN_MV
 *   LOW_WARN → NORMAL    当 mv ≥ WARN_MV + HYS
 *   LOW_WARN → LOW_STOP  当 mv ≤ STOP_MV
 *   LOW_STOP → LOW_WARN  当 mv ≥ STOP_MV + HYS（不直接回 NORMAL）
 *
 *   断线护栏：mv < DISCONNECTED_MV 时 state 回 UNKNOWN，避免浮空引脚误触发急停。
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
        /* 首次进入业务采样：按实测 mv 直接定位分类，不走回滞 */
        if (mv <= BSP_BATTERY_STOP_MV)  return BSP_BATT_STATE_LOW_STOP;
        if (mv <= BSP_BATTERY_WARN_MV)  return BSP_BATT_STATE_LOW_WARN;
        return BSP_BATT_STATE_NORMAL;
    }
}

/* -------------------------------------------------------------------------- */
/* 公共 API                                                                    */
/* -------------------------------------------------------------------------- */

void bsp_battery_init(void)
{
    uint32_t now          = bsp_systick_get_ms();
    s_batt.latest_mv      = 0u;
    s_batt.ref_mv         = 0u;
    s_batt.ref_ms         = now;
    s_batt.last_raw       = 0u;
    s_batt.warmup_done    = 0u;
    s_batt.have_ref       = 0u;
    s_batt.boot_ok        = 0u;
    s_batt.low_stop_count = 0u;
    s_batt.state          = BSP_BATT_STATE_UNKNOWN;

    /* 预热从一个轮询周期后开始（让 ADC/基准上电稳定），随后连续采样直至平台。 */
    s_batt.next_sample_ms = now + BSP_BATTERY_BOOT_POLL_MS;
    s_batt.warmup_until_ms = now + BSP_BATTERY_BOOT_WARMUP_MAX_MS;

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
    /* 反算电池端：v_bat = v_pin × 10000 / RATIO_X10000
     *   mv_pin ≤ 3300 → 乘 10000 ≤ 33e6 < 2^32，uint32 安全 */
    return (mv_pin * 10000u) / (uint32_t)BSP_BATTERY_DIVIDER_RATIO_X10000;
}

void bsp_battery_update(void)
{
    uint32_t now = bsp_systick_get_ms();
    if ((int32_t)(now - s_batt.next_sample_ms) < 0) {
        return;  /* 未到采样时刻，立即返回（大多数调用走此路径） */
    }

    /* 触发转换并阻塞等待（< 5 µs，忽略不计）*/
    DL_ADC12_clearInterruptStatus(ADC_BAT_INST,
        DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);
    trigger_conversion();
    if (!wait_conversion_done()) {
        /* ADC 超时（极罕见）：推迟 10 ms 重试，避免读取陈旧结果 */
        s_batt.next_sample_ms = now + 10u;
        return;
    }

    uint16_t raw      = (uint16_t)DL_ADC12_getMemResult(ADC_BAT_INST, DL_ADC12_MEM_IDX_0);
    s_batt.last_raw   = raw;
    uint32_t mv       = bsp_battery_raw_to_mv(raw);
    s_batt.latest_mv  = mv;   /* 预热期也更新，供心跳观察充电爬升 */

    if (!s_batt.warmup_done) {
        /* 阶段 A：预热 ─ 连续轮询，按"基线窗口总上升量"判定电容/电源是否已稳。
         * state 保持 UNKNOWN，不参与安全决策。 */
        bool charged = false;
        if (!s_batt.have_ref) {
            s_batt.ref_mv   = mv;
            s_batt.ref_ms   = now;
            s_batt.have_ref = 1u;
        } else if ((int32_t)(now - s_batt.ref_ms) >= (int32_t)BSP_BATTERY_BOOT_BASELINE_MS) {
            /* 跨过一个基线窗口：比较整段总上升量 */
            uint32_t rise = (mv > s_batt.ref_mv) ? (mv - s_batt.ref_mv)
                                                 : (s_batt.ref_mv - mv);
            if (rise < (uint32_t)BSP_BATTERY_BOOT_STABLE_DELTA_MV) {
                charged = true;     /* 整窗几乎不再上升 → 已充满 */
            } else {
                s_batt.ref_mv = mv; /* 仍在上升：滑动基线，继续观察 */
                s_batt.ref_ms = now;
            }
        }

        bool timeout = ((int32_t)(now - s_batt.warmup_until_ms) >= 0);

        if (charged || timeout) {
            /* 预热结束：以当前读值定 boot_ok，并落地首个业务分类。
             * 注意：兜底超时若恰逢电源仍在缓升而读到低压，不直接 latch LOW_STOP
             * （会触发急停），先降一档为 LOW_WARN，交由业务期去抖确认后再 STOP。 */
            s_batt.warmup_done = 1u;
            s_batt.boot_ok     = (mv >= (uint32_t)BSP_BATTERY_BOOT_OK_MV) ? 1u : 0u;
            bsp_battery_state_t st = classify(mv, BSP_BATT_STATE_UNKNOWN);
            if (st == BSP_BATT_STATE_LOW_STOP) {
                st = BSP_BATT_STATE_LOW_WARN;
                s_batt.low_stop_count = 1u;   /* 预置一档，业务期再确认一次即 STOP */
            }
            s_batt.state          = st;
            s_batt.next_sample_ms = now + BSP_BATTERY_UPDATE_PERIOD_MS;
        } else {
            s_batt.next_sample_ms = now + BSP_BATTERY_BOOT_POLL_MS;
        }
        return;
    }

    /* 阶段 B：业务周期采样 ─ classify + LOW_STOP 去抖（DEBOUNCE_SAMPLES 次确认）*/
    bsp_battery_state_t next = classify(mv, s_batt.state);
    if ((next == BSP_BATT_STATE_LOW_STOP) &&
        (s_batt.state != BSP_BATT_STATE_LOW_STOP)) {
        s_batt.low_stop_count++;
        if (s_batt.low_stop_count < BSP_BATTERY_LOW_STOP_DEBOUNCE_SAMPLES) {
            next = BSP_BATT_STATE_LOW_WARN;  /* 尚未确认，降级为 WARN */
        }
    } else if (next != BSP_BATT_STATE_LOW_STOP) {
        s_batt.low_stop_count = 0u;
    }
    s_batt.state = next;

    s_batt.next_sample_ms = now + BSP_BATTERY_UPDATE_PERIOD_MS;
}

bool bsp_battery_is_boot_ok(void)
{
    return s_batt.boot_ok != 0u;
}

bool bsp_battery_is_ready(void)
{
    return s_batt.warmup_done != 0u;
}

uint32_t bsp_battery_get_mv(void)
{
    return s_batt.latest_mv;
}

uint16_t bsp_battery_get_raw(void)
{
    return s_batt.last_raw;
}

bsp_battery_state_t bsp_battery_get_state(void)
{
    return s_batt.state;
}
