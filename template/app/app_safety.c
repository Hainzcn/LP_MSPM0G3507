/**
 * @file    app_safety.c
 * @brief   安全状态机实现，详见 app_safety.h。
 *
 * 状态转移图（大写 = 顶层状态；Bat / Pitch 是事件源）：
 *
 *   上电（首次 init）
 *       │
 *       ▼  等待 BOOT_CHECK_MS (5 s)
 *  BOOT_CHECK ─(arm pending + Bat≠LOW_STOP)─► ARMED
 *       │    ─(arm pending + Bat==LOW_STOP)─► LOW_BAT_STOP
 *       └────(no pending arm)───────────────► DISARMED
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
#include <stdio.h>

/* -------------------------------------------------------------------------- */
/* 内部                                                                        */
/* -------------------------------------------------------------------------- */

/**
 * Canary 包装：强制 canary_before | state | canary_after 连续布局，
 * 栈溢出写脏时 canary 会先被破坏，get_state() 可检测并 auto-heal。
 */
#define SAFETY_CANARY_VALUE  (0xDEAD5AFEu)

typedef struct {
    uint32_t              canary_before;
    app_safety_state_t    state;
    uint32_t              canary_after;
} app_safety_block_t;

static app_safety_block_t s_blk = {
    .canary_before = SAFETY_CANARY_VALUE,
    .state         = APP_SAFETY_DISARMED,
    .canary_after  = SAFETY_CANARY_VALUE,
};

#define s_state  s_blk.state

static uint32_t s_startup_grace_until_ms = 0u;
static uint8_t  s_fall_debounce_count    = 0u;

/* BOOT_CHECK 相关：
 *   s_boot_check_done  — 静态标志，C 启动清零；首次上电为 false，自检完成后置 true。
 *                        模式切换时 app_safety_init() 见 true 则直接进 DISARMED。
 *   s_pending_arm      — arm() 在 BOOT_CHECK 期间被调用时置 true；自检结束后据此
 *                        决定进入 ARMED 还是 DISARMED。
 *   s_boot_check_until_ms — 自检结束的绝对时间戳（ms）。 */
static bool     s_boot_check_done     = false;   /* C 启动 BSS 初始化为 false */
static bool     s_pending_arm         = false;
static uint32_t s_boot_check_until_ms = 0u;

/** 检测 canary 和 state 合法性；corruption 时 auto-heal 到 DISARMED。 */
static app_safety_state_t sanitize_state(void)
{
    bool corrupt = false;
    if (s_blk.canary_before != SAFETY_CANARY_VALUE) corrupt = true;
    if (s_blk.canary_after  != SAFETY_CANARY_VALUE) corrupt = true;
    if ((uint32_t)s_blk.state > (uint32_t)APP_SAFETY_BOOT_CHECK) corrupt = true;

    if (corrupt) {
        (void)printf("[safety] CORRUPTION detected, auto-heal -> DISARMED\r\n");
        s_blk.canary_before = SAFETY_CANARY_VALUE;
        s_blk.state         = APP_SAFETY_DISARMED;
        s_blk.canary_after  = SAFETY_CANARY_VALUE;
        bsp_motor_brake_pulse_ms(APP_SAFETY_FALL_BRAKE_MS);
        bsp_motor_enable(false);
    }
    return s_blk.state;
}

static bool is_startup_grace_active(void)
{
    return ((int32_t)(s_startup_grace_until_ms - bsp_systick_get_ms()) > 0);
}

static bool is_boot_check_active(void)
{
    return ((int32_t)(s_boot_check_until_ms - bsp_systick_get_ms()) > 0);
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
    s_blk.canary_before = SAFETY_CANARY_VALUE;
    s_blk.canary_after  = SAFETY_CANARY_VALUE;
    s_fall_debounce_count    = 0u;
    s_pending_arm            = false;
    s_startup_grace_until_ms = bsp_systick_get_ms() + APP_SAFETY_STARTUP_FALL_MUTE_MS;

    if (!s_boot_check_done) {
        /* 首次上电（MCU 硬件复位后 BSS 清零）：进入 BOOT_CHECK 静默自检态 */
        s_state               = APP_SAFETY_BOOT_CHECK;
        s_boot_check_until_ms = bsp_systick_get_ms() + APP_SAFETY_BOOT_CHECK_MS;
    } else {
        /* 模式切换后重新 init（非首次上电）：直接 DISARMED，跳过自检 */
        s_state = APP_SAFETY_DISARMED;
    }

    /* 上电默认电机已经被 bsp_motor_init 设为 STBY=0，但保险起见再做一遍 */
    bsp_motor_set_pwm_limit(1000u);
    bsp_motor_brake_pulse_ms(0u);   /* 等价 stop */
    bsp_motor_enable(false);
}

bool app_safety_arm(void)
{
    if (s_state == APP_SAFETY_BOOT_CHECK) {
        /* 自检期间受理 arm 请求；自检结束后在 tick 内自动执行真正的 arm 动作。
         * 返回 true 告知调用方"请求已受理"，不阻塞 main() 的后续流程。 */
        s_pending_arm = true;
        return true;
    }
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
    /* ---- 0) BOOT_CHECK：上电自检静默期，忽略所有外部事件 ---- */
    if (s_state == APP_SAFETY_BOOT_CHECK) {
        /* 解锁条件（满足其一即可）：
         *   ① 5 s 计时器到期（正常流程）
         *   ② bsp_battery 在 t=3 s 自检快照读值 > BOOT_OK_MV（电池满足条件，提前约省 2 s）*/
        if (!is_boot_check_active() || bsp_battery_is_boot_ok()) {
            s_boot_check_done     = true;
            s_fall_debounce_count = 0u;
            bool arm_req          = s_pending_arm;
            s_pending_arm         = false;

            bsp_battery_state_t bs = bsp_battery_get_state();
            if (bs == BSP_BATT_STATE_LOW_STOP) {
                /* 自检期间电池已确认低于急停阈值，优先进 LOW_BAT_STOP */
                transition(APP_SAFETY_LOW_BAT_STOP);
            } else if (arm_req) {
                transition(APP_SAFETY_ARMED);
                /* 若电池为 LOW_WARN，下一拍 tick 会自动降级到 LOW_BAT_WARN */
            } else {
                transition(APP_SAFETY_DISARMED);
            }
        }
        return s_state;
    }

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
    return sanitize_state();
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
