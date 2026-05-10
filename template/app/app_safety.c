/**
 * @file    app_safety.c
 * @brief   安全状态机实现，详见 app_safety.h。
 *
 * 状态转移图（大写 = 顶层状态；Bat / Pitch 是事件源）：
 *
 *            app_safety_arm()                |pitch|>60
 *  DISARMED ─────────────► ARMED ─────────────────► FALLEN
 *      ▲                    │   │                     │
 *      │                    │   │ Bat.LOW_WARN        │ app_safety_arm()
 *      │ disarm()           │   ▼                     │
 *      │                    │  LOW_BAT_WARN ─────► ARMED  (若 Bat 回 NORMAL)
 *      │                    │   │
 *      │                    │   │ Bat.LOW_STOP
 *      │                    ▼   ▼
 *      └─────────── LOW_BAT_STOP  ◄─── any state, on Bat.LOW_STOP
 *                          │
 *                          │ arm：被拒绝（蜂鸣外部触发，本模块不直接响）
 *                          ▼
 *                       (stay LOW_BAT_STOP until Bat 回 WARN+HYS)
 */

#include "app_safety.h"

#include "bsp_battery.h"
#include "bsp_motor.h"
#include "bsp_systick.h"

#include <stddef.h>

/* -------------------------------------------------------------------------- */
/* 内部                                                                        */
/* -------------------------------------------------------------------------- */

static app_safety_state_t s_state = APP_SAFETY_DISARMED;
static uint32_t s_startup_grace_until_ms = 0u;
static uint8_t s_fall_debounce_count = 0u;

static bool is_startup_grace_active(void)
{
    return ((int32_t)(s_startup_grace_until_ms - bsp_systick_get_ms()) > 0);
}

/** 进入"急停"硬件操作：brake_pulse + enable(false)。可重入（再次跌倒不出问题）。 */
static void hw_emergency(uint32_t brake_ms)
{
    bsp_motor_brake_pulse_ms(brake_ms);
    bsp_motor_enable(false);
}

/** 进入"正常运行"硬件操作：复位 PWM 限幅、enable(true)。 */
static void hw_arm_normal(void)
{
    bsp_motor_set_pwm_limit(1000u);
    bsp_motor_enable(true);
}

/** 进入"低压降功率"硬件操作：限幅 + enable(true)。 */
static void hw_arm_low_warn(void)
{
    bsp_motor_set_pwm_limit(APP_SAFETY_LOW_BAT_PWM_LIMIT);
    bsp_motor_enable(true);
}

/** 状态转移 + 同步硬件动作。仅在状态确实变化时操作硬件，避免冗余写。 */
static void transition(app_safety_state_t next)
{
    if (next == s_state) {
        return;
    }
    s_state = next;

    switch (next) {
    case APP_SAFETY_DISARMED:
    case APP_SAFETY_FALLEN:
        hw_emergency(APP_SAFETY_FALL_BRAKE_MS);
        break;

    case APP_SAFETY_LOW_BAT_STOP:
        hw_emergency(APP_SAFETY_LOW_BAT_BRAKE_MS);
        break;

    case APP_SAFETY_ARMED:
        hw_arm_normal();
        break;

    case APP_SAFETY_LOW_BAT_WARN:
        hw_arm_low_warn();
        break;

    default:
        break;
    }
}

/* -------------------------------------------------------------------------- */
/* 公共 API                                                                    */
/* -------------------------------------------------------------------------- */

void app_safety_init(void)
{
    s_state = APP_SAFETY_DISARMED;
    s_startup_grace_until_ms =
        bsp_systick_get_ms() + APP_SAFETY_STARTUP_FALL_MUTE_MS;
    s_fall_debounce_count = 0u;
    /* 上电默认电机已经被 bsp_motor_init 设为 STBY=0，但保险起见再做一遍 */
    bsp_motor_set_pwm_limit(1000u);
    bsp_motor_brake_pulse_ms(0u);   /* 等价 stop */
    bsp_motor_enable(false);
}

bool app_safety_arm(void)
{
    if (s_state == APP_SAFETY_LOW_BAT_STOP) {
        return false;   /* 电池保护态拒绝重启，调用方自行蜂鸣 / 报警 */
    }
    s_fall_debounce_count = 0u;
    /* 不论之前是 DISARMED / FALLEN / LOW_BAT_WARN，重置到 ARMED；电池侧后续
     * 会自动重新 demote 到 LOW_BAT_WARN（如果还低压告警）。 */
    transition(APP_SAFETY_ARMED);
    return true;
}

void app_safety_disarm(void)
{
    transition(APP_SAFETY_DISARMED);
}

app_safety_state_t app_safety_tick(const app_safety_attitude_t *att)
{
    /* ---- 1) 跌倒检测（仅 ARMED / LOW_BAT_WARN 时有意义） ---- */
    bool fallen = false;
    if (is_startup_grace_active() || (att == NULL) || !att->attitude_valid) {
        s_fall_debounce_count = 0u;
    } else {
        float p = att->pitch_deg;
        if (p < 0.0f) p = -p;
        if (p > APP_SAFETY_FALL_PITCH_DEG) {
            if (s_fall_debounce_count < APP_SAFETY_FALL_DEBOUNCE_TICKS) {
                s_fall_debounce_count++;
            }
            if (s_fall_debounce_count >= APP_SAFETY_FALL_DEBOUNCE_TICKS) {
                fallen = true;
            }
        } else {
            s_fall_debounce_count = 0u;
        }
    }

    /* ---- 2) 电池状态读取 ---- */
    bsp_battery_state_t bs = bsp_battery_get_state();

    /* ---- 3) 按优先级合成新状态：LOW_STOP > FALLEN > LOW_WARN > ARMED ----
     *
     *   注意：DISARMED 是人工状态，电池保护 / 跌倒都不会主动把它升回 ARMED；
     *   只有 app_safety_arm() 能把 DISARMED → ARMED。但电池保护可以把
     *   DISARMED 升级为 LOW_BAT_STOP（提示用户即使没启动电机也得换电池），
     *   跌倒事件在 DISARMED 下被忽略（车在地上谁都知道倒着，无需告警）。 */

    if (bs == BSP_BATT_STATE_LOW_STOP) {
        transition(APP_SAFETY_LOW_BAT_STOP);
    } else if (s_state == APP_SAFETY_LOW_BAT_STOP) {
        /* 上一拍处于 LOW_STOP，但电池现在已升回 LOW_WARN+HYS 或更好；
         * 这里把状态降级到 LOW_BAT_WARN，等待上层再 arm（不自动恢复电机）。
         * 这条策略避免"电池电压在阈值附近抖动→车体反复急停启动"的安全风险。 */
        transition(APP_SAFETY_LOW_BAT_WARN);
        bsp_motor_enable(false);   /* 二次确认 STBY 为低 */
    } else if (s_state == APP_SAFETY_DISARMED) {
        /* DISARMED 下电池告警依然走 LOW_BAT_WARN 提示，但电机仍 STBY=低 */
        if (bs == BSP_BATT_STATE_LOW_WARN) {
            s_state = APP_SAFETY_LOW_BAT_WARN;   /* 不调 transition()，避免 enable(true) */
        }
        /* DISARMED 下跌倒被忽略 */
    } else if (fallen) {
        transition(APP_SAFETY_FALLEN);
    } else if (bs == BSP_BATT_STATE_LOW_WARN) {
        /* 当前电池告警但未跌倒：保持驱动但限速 */
        if (s_state == APP_SAFETY_ARMED || s_state == APP_SAFETY_LOW_BAT_WARN) {
            transition(APP_SAFETY_LOW_BAT_WARN);
        }
        /* 若处于 FALLEN，等上层重新 arm 时再走 ARM 路径 */
    } else if (bs == BSP_BATT_STATE_NORMAL) {
        /* 电池正常 + 没跌倒：把限速恢复（如果之前 LOW_WARN） */
        if ((s_state == APP_SAFETY_LOW_BAT_WARN) && bsp_motor_is_enabled()) {
            transition(APP_SAFETY_ARMED);
        }
        /* ARMED / FALLEN / DISARMED 不主动改 */
    }
    /* bs == UNKNOWN（采样还没攒够首拍）→ 状态不变 */

    return s_state;
}

app_safety_state_t app_safety_get_state(void)
{
    return s_state;
}

bool app_safety_can_drive(void)
{
    return (s_state == APP_SAFETY_ARMED) ||
           (s_state == APP_SAFETY_LOW_BAT_WARN);
}

bool app_safety_is_startup_grace_active(void)
{
    return is_startup_grace_active();
}
