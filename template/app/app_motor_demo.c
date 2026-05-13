/**
 * @file    app_motor_demo.c
 * @brief   电机驱动演示：串口控制电机、校准，并可切入装车模式。
 */

#include "app_motor_demo.h"

#include "bsp_battery.h"
#include "bsp_gpio.h"
#include "bsp_log_uart.h"
#include "bsp_motor.h"
#include "bsp_systick.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define APP_MOTOR_DEMO_MAX_RPM                     (720u)
#define APP_MOTOR_DEMO_DEFAULT_RPM                 (200u)
#define APP_MOTOR_DEMO_BRAKE_MS                    (120u)
#define APP_MOTOR_DEMO_RPM_STEP                    (20u)
#define APP_MOTOR_SYNC_PERIOD_MS                   (50u)
/*
 * BSP 右路基础补偿接管了正转稳态误差（~13 rpm @1000‰），PI 只需处理瞬态与反转残差，
 * 因此 Kp 从 8 降至 4，Ki 从 1 降至 0（纯比例）以避免积分抖动。
 * 若反转仍有残余偏差可将 Ki 恢复为 1。
 */
#define APP_MOTOR_SYNC_KP_PM_PER_RPM               (4)
#define APP_MOTOR_SYNC_KI_PM_PER_RPM_STEP          (0)
#define APP_MOTOR_SYNC_MAX_CORRECTION_PM           (200)

#define APP_MOTOR_LOG_PERIOD_MS                    (100u)
#define APP_MOTOR_HEARTBEAT_PERIOD_MS              (250u)
#define APP_MOTOR_BATT_PERIOD_MS                   (10u)

/* ── 校准扫描可调宏 ─────────────────────────────────────────────────────── */
#ifndef APP_MOTOR_CAL_PWM_START_PM
#define APP_MOTOR_CAL_PWM_START_PM     (0)
#endif
#ifndef APP_MOTOR_CAL_PWM_END_PM
#define APP_MOTOR_CAL_PWM_END_PM       (100)
#endif
#ifndef APP_MOTOR_CAL_PWM_STEP_PM
#define APP_MOTOR_CAL_PWM_STEP_PM      (5)
#endif
#ifndef APP_MOTOR_CAL_DWELL_MS
#define APP_MOTOR_CAL_DWELL_MS         (2000u)
#endif
#ifndef APP_MOTOR_CAL_SAMPLE_PERIOD_MS
#define APP_MOTOR_CAL_SAMPLE_PERIOD_MS (100u)
#endif
#ifndef APP_MOTOR_CAL_SETTLE_MS
#define APP_MOTOR_CAL_SETTLE_MS        (500u)
#endif

/* ── 校准状态机 ─────────────────────────────────────────────────────────── */
typedef enum {
    APP_CAL_IDLE    = 0,
    APP_CAL_FORWARD = 1,
    APP_CAL_REVERSE = 2,
    APP_CAL_DONE    = 3,
} app_cal_phase_t;

static app_cal_phase_t s_cal_phase         = APP_CAL_IDLE;
static int16_t         s_cal_step_idx      = 0;
static int16_t         s_cal_total_steps   = 0;
static uint32_t        s_cal_step_start_ms = 0u;
static int16_t         s_cal_pm_now        = 0;
static uint32_t        s_cal_last_sample_ms = 0u;

/* 校准进入时保存的原始状态，结束/中止后恢复 */
static int16_t         s_cal_saved_pwm_pm        = 0;
static bool            s_cal_saved_sync_enabled  = false;
static bool            s_cal_saved_deadzone_comp = false;
static bool            s_cal_saved_cal_mode      = false;
static bool            s_cal_saved_right_fwd_scale = false;
static bool            s_cal_apply_comp          = true;

static uint16_t s_target_rpm = APP_MOTOR_DEMO_DEFAULT_RPM;
static int16_t  s_target_pwm_pm =
    (int16_t)((APP_MOTOR_DEMO_DEFAULT_RPM * BSP_MOTOR_PWM_MAX_PERMILLE +
        (APP_MOTOR_DEMO_MAX_RPM / 2u)) / APP_MOTOR_DEMO_MAX_RPM);

static app_motor_demo_sync_diag_t s_sync = {
    .enabled = true,
    .kp_pm_per_rpm = APP_MOTOR_SYNC_KP_PM_PER_RPM,
    .ki_pm_per_rpm_step = APP_MOTOR_SYNC_KI_PM_PER_RPM_STEP,
    .correction_pm = 0,
    .left_cmd_pm = 0,
    .right_cmd_pm = 0,
    .rpm_error = 0,
};
static int16_t s_sync_i_pm = 0;

static int16_t rpm_to_pwm_pm(uint16_t rpm)
{
    if (rpm > APP_MOTOR_DEMO_MAX_RPM) {
        rpm = APP_MOTOR_DEMO_MAX_RPM;
    }
    return (int16_t)(((uint32_t)rpm * BSP_MOTOR_PWM_MAX_PERMILLE +
        (APP_MOTOR_DEMO_MAX_RPM / 2u)) / APP_MOTOR_DEMO_MAX_RPM);
}

void app_motor_demo_set_speed_rpm(uint16_t rpm)
{
    if (rpm > APP_MOTOR_DEMO_MAX_RPM) {
        rpm = APP_MOTOR_DEMO_MAX_RPM;
    }
    s_target_rpm = rpm;
    s_target_pwm_pm = rpm_to_pwm_pm(rpm);
}

uint16_t app_motor_demo_get_speed_rpm(void)
{
    return s_target_rpm;
}

void app_motor_demo_set_sync_enabled(bool enabled)
{
    s_sync.enabled = enabled;
    if (!enabled) {
        app_motor_demo_reset_sync();
    }
}

void app_motor_demo_set_sync_gains(int16_t kp_pm_per_rpm,
                                   int16_t ki_pm_per_rpm_step)
{
    s_sync.kp_pm_per_rpm = kp_pm_per_rpm;
    s_sync.ki_pm_per_rpm_step = ki_pm_per_rpm_step;
}

void app_motor_demo_reset_sync(void)
{
    s_sync_i_pm = 0;
    s_sync.correction_pm = 0;
    s_sync.left_cmd_pm = s_target_pwm_pm;
    s_sync.right_cmd_pm = s_target_pwm_pm;
    s_sync.rpm_error = 0;
}

void app_motor_demo_get_sync_diag(app_motor_demo_sync_diag_t *out)
{
    if (out == NULL) {
        return;
    }
    *out = s_sync;
}

static int16_t clamp_pm(int32_t v)
{
    if (v > BSP_MOTOR_PWM_MAX_PERMILLE) {
        return BSP_MOTOR_PWM_MAX_PERMILLE;
    }
    if (v < -BSP_MOTOR_PWM_MAX_PERMILLE) {
        return -BSP_MOTOR_PWM_MAX_PERMILLE;
    }
    return (int16_t)v;
}

static int16_t clamp_sync_correction(int32_t v)
{
    if (v > APP_MOTOR_SYNC_MAX_CORRECTION_PM) {
        return APP_MOTOR_SYNC_MAX_CORRECTION_PM;
    }
    if (v < -APP_MOTOR_SYNC_MAX_CORRECTION_PM) {
        return -APP_MOTOR_SYNC_MAX_CORRECTION_PM;
    }
    return (int16_t)v;
}

static void apply_motor_output(int16_t correction_pm)
{
    int16_t left_pm  = clamp_pm((int32_t)s_target_pwm_pm + correction_pm);
    int16_t right_pm = clamp_pm((int32_t)s_target_pwm_pm - correction_pm);

    s_sync.left_cmd_pm  = left_pm;
    s_sync.right_cmd_pm = right_pm;
    bsp_motor_set_output(left_pm, right_pm);
}

static int16_t rpm_to_i16(float rpm)
{
    if (rpm >= 0.0f) {
        return (int16_t)(rpm + 0.5f);
    }
    return (int16_t)(rpm - 0.5f);
}

static void motor_sync_step(const bsp_motor_feedback_t *feedback, bool running)
{
    if ((feedback == NULL) || !running) {
        return;
    }

    if (!s_sync.enabled) {
        s_sync.rpm_error = 0;
        s_sync.correction_pm = 0;
        apply_motor_output(0);
        return;
    }

    int16_t left_rpm = rpm_to_i16(feedback->left_speed_rpm);
    int16_t right_rpm = rpm_to_i16(feedback->right_speed_rpm);
    int16_t error = (int16_t)(right_rpm - left_rpm);

    int32_t i_term = (int32_t)s_sync_i_pm +
        ((int32_t)error * (int32_t)s_sync.ki_pm_per_rpm_step);
    s_sync_i_pm = clamp_sync_correction(i_term);
    s_sync.rpm_error = error;

    int32_t correction = ((int32_t)error * (int32_t)s_sync.kp_pm_per_rpm) +
        (int32_t)s_sync_i_pm;
    s_sync.correction_pm = clamp_sync_correction(correction);
    apply_motor_output(s_sync.correction_pm);
}

static void apply_run_output_if_needed(bool running)
{
    if (running) {
        bsp_motor_enable(true);
        apply_motor_output(s_sync.enabled ? s_sync.correction_pm : 0);
    }
}

static void brake_now(void)
{
    bsp_motor_brake_pulse_ms(APP_MOTOR_DEMO_BRAKE_MS);
}

static void prepare_load_mode(void)
{
    (void)printf("[ctrl] load mode requested\r\n");
    bsp_motor_stop();
    bsp_motor_enable(false);
}

/* ========================================================================== */
/* 校准扫描 — 日志打印                                                          */
/* ========================================================================== */

static void print_cal_header(int8_t dir)
{
    int16_t total = s_cal_total_steps;
    (void)printf(
        "[cal] start dir=%+d steps=%d pm_start=%d pm_end=%d step=%d "
        "dwell_ms=%u settle_ms=%u apply_comp=%u dz_comp=%u cal_mode=%u rf_scale=%u "
        "dz_lf=%d dz_lr=%d dz_rf=%d dz_rr=%d "
        "rdz_lf=%d rdz_lr=%d rdz_rf=%d rdz_rr=%d\r\n",
        (int)dir,
        (int)total,
        APP_MOTOR_CAL_PWM_START_PM,
        APP_MOTOR_CAL_PWM_END_PM,
        APP_MOTOR_CAL_PWM_STEP_PM,
        (unsigned int)APP_MOTOR_CAL_DWELL_MS,
        (unsigned int)APP_MOTOR_CAL_SETTLE_MS,
        s_cal_apply_comp ? 1u : 0u,
        bsp_motor_get_deadzone_comp_enabled() ? 1u : 0u,
        bsp_motor_get_calibration_mode() ? 1u : 0u,
        bsp_motor_get_right_forward_scale_enabled() ? 1u : 0u,
        BSP_MOTOR_LEFT_FORWARD_DEADZONE_PM,
        BSP_MOTOR_LEFT_REVERSE_DEADZONE_PM,
        BSP_MOTOR_RIGHT_FORWARD_DEADZONE_PM,
        BSP_MOTOR_RIGHT_REVERSE_DEADZONE_PM,
        BSP_MOTOR_LEFT_FORWARD_RUNNING_DEADZONE_PM,
        BSP_MOTOR_LEFT_REVERSE_RUNNING_DEADZONE_PM,
        BSP_MOTOR_RIGHT_FORWARD_RUNNING_DEADZONE_PM,
        BSP_MOTOR_RIGHT_REVERSE_RUNNING_DEADZONE_PM);
}

static void print_cal_step(int8_t dir, int16_t idx, int16_t pm)
{
    (void)printf("[cal] step dir=%+d idx=%d/%d pm=%d\r\n",
        (int)dir, (int)idx, (int)s_cal_total_steps, (int)pm);
}

static void print_cal_sample(uint32_t now_ms,
                              int8_t dir,
                              int16_t idx,
                              int16_t pm,
                              const bsp_motor_feedback_t *fb)
{
    uint32_t vbat = bsp_battery_get_mv();
    (void)printf(
        "[cal] dir=%+d idx=%d/%d pm=%d t=%lu vbat=%lu "
        "rpmL=%ld rpmR=%ld ctL=%ld ctR=%ld\r\n",
        (int)dir,
        (int)idx,
        (int)s_cal_total_steps,
        (int)pm,
        (unsigned long)now_ms,
        (unsigned long)vbat,
        (long)(int32_t)fb->left_speed_rpm,
        (long)(int32_t)fb->right_speed_rpm,
        (long)fb->left_count,
        (long)fb->right_count);
}

static void print_cal_done(int8_t dir, bool next_reverse)
{
    if (next_reverse) {
        (void)printf("[cal] done dir=%+d next=reverse\r\n", (int)dir);
    } else {
        (void)printf("[cal] done dir=%+d\r\n", (int)dir);
    }
}

/* ========================================================================== */
/* 校准扫描 — 状态机                                                            */
/* ========================================================================== */

static int16_t cal_total_steps(void)
{
    return (int16_t)((APP_MOTOR_CAL_PWM_END_PM - APP_MOTOR_CAL_PWM_START_PM) /
                     APP_MOTOR_CAL_PWM_STEP_PM + 1);
}

static int16_t cal_pm_for_step(int16_t idx, int8_t dir)
{
    int16_t abs_pm = (int16_t)(APP_MOTOR_CAL_PWM_START_PM +
                                (int32_t)idx * APP_MOTOR_CAL_PWM_STEP_PM);
    if (abs_pm > APP_MOTOR_CAL_PWM_END_PM) {
        abs_pm = (int16_t)APP_MOTOR_CAL_PWM_END_PM;
    }
    return (dir >= 0) ? abs_pm : (int16_t)(-abs_pm);
}

static int8_t cal_current_dir(void)
{
    return (s_cal_phase == APP_CAL_FORWARD) ? (int8_t)1 : (int8_t)-1;
}

static void cal_apply_step(uint32_t now_ms)
{
    s_cal_pm_now       = cal_pm_for_step(s_cal_step_idx, cal_current_dir());
    s_cal_step_start_ms = now_ms;
    s_cal_last_sample_ms = now_ms;
    s_sync_i_pm        = 0;
    s_sync.correction_pm = 0;
    bsp_motor_set_output(s_cal_pm_now, s_cal_pm_now);
    print_cal_step(cal_current_dir(), s_cal_step_idx, s_cal_pm_now);
}

bool app_motor_demo_cal_start(void)
{
    if (s_cal_phase != APP_CAL_IDLE) {
        (void)printf("[cal] busy\r\n");
        return false;
    }

    s_cal_saved_pwm_pm       = s_target_pwm_pm;
    s_cal_saved_sync_enabled = s_sync.enabled;
    s_cal_saved_deadzone_comp = bsp_motor_get_deadzone_comp_enabled();
    s_cal_saved_cal_mode     = bsp_motor_get_calibration_mode();
    s_cal_saved_right_fwd_scale = bsp_motor_get_right_forward_scale_enabled();

    s_sync.enabled = false;
    s_sync_i_pm    = 0;
    s_sync.correction_pm = 0;
    /* apply_comp=1：扫描时启用当前运行补偿系统（静摩擦起转 → 动摩擦运行）；
     * apply_comp=0：输出原始 PWM，用于重新采集未补偿的起转曲线。 */
    bsp_motor_set_right_forward_scale_enabled(s_cal_apply_comp);
    bsp_motor_set_deadzone_comp_enabled(s_cal_apply_comp);
    bsp_motor_set_calibration_mode(false);

    s_cal_total_steps = cal_total_steps();
    s_cal_step_idx    = 0;
    s_cal_phase       = APP_CAL_FORWARD;

    bsp_motor_enable(true);
    print_cal_header((int8_t)1);

    uint32_t now_ms = bsp_systick_get_ms();
    cal_apply_step(now_ms);

    return true;
}

void app_motor_demo_cal_abort(void)
{
    if (s_cal_phase == APP_CAL_IDLE) {
        return;
    }
    (void)printf("[cal] abort\r\n");
    s_cal_phase = APP_CAL_IDLE;
    bsp_motor_set_calibration_mode(s_cal_saved_cal_mode);
    bsp_motor_set_deadzone_comp_enabled(s_cal_saved_deadzone_comp);
    bsp_motor_set_right_forward_scale_enabled(s_cal_saved_right_fwd_scale);
    brake_now();

    s_target_pwm_pm  = s_cal_saved_pwm_pm;
    s_sync.enabled   = s_cal_saved_sync_enabled;
    s_sync_i_pm      = 0;
    s_sync.correction_pm = 0;
}

bool app_motor_demo_cal_is_active(void)
{
    return (s_cal_phase == APP_CAL_FORWARD) || (s_cal_phase == APP_CAL_REVERSE);
}

/**
 * @brief 每个主循环 tick 调用（1 kHz 路径）。
 * @param now_ms   当前毫秒时间戳
 * @param feedback 已更新的编码器快照（由调用方在 sync tick 时机更新）
 * @return true = 校准进行中，调用方应跳过普通 sync/log tick
 */
static bool handle_calibration_tick(uint32_t now_ms, bsp_motor_feedback_t *feedback)
{
    if (!app_motor_demo_cal_is_active()) {
        return false;
    }

    int8_t dir = cal_current_dir();
    uint32_t elapsed = now_ms - s_cal_step_start_ms;

    /* ── 到期进入下一档 ─────────────────────────────── */
    if (elapsed >= APP_MOTOR_CAL_DWELL_MS) {
        s_cal_step_idx++;
        if (s_cal_step_idx >= s_cal_total_steps) {
            if (s_cal_phase == APP_CAL_FORWARD) {
                print_cal_done(dir, true);
                s_cal_phase    = APP_CAL_REVERSE;
                s_cal_step_idx = 0;
                print_cal_header((int8_t)-1);
                cal_apply_step(now_ms);
            } else {
                /* 反向也完成 */
                print_cal_done(dir, false);
                s_cal_phase = APP_CAL_IDLE;
                s_target_pwm_pm  = s_cal_saved_pwm_pm;
                s_sync.enabled   = s_cal_saved_sync_enabled;
                s_sync_i_pm      = 0;
                s_sync.correction_pm = 0;
                bsp_motor_set_calibration_mode(s_cal_saved_cal_mode);
                bsp_motor_set_deadzone_comp_enabled(s_cal_saved_deadzone_comp);
                bsp_motor_set_right_forward_scale_enabled(s_cal_saved_right_fwd_scale);
                bsp_motor_set_output(s_cal_saved_pwm_pm, s_cal_saved_pwm_pm);
                (void)printf("[cal] calibration complete\r\n");
            }
        } else {
            cal_apply_step(now_ms);
        }
        return true;
    }

    /* ── settle 窗口内不采样 ─────────────────────────── */
    if (elapsed < APP_MOTOR_CAL_SETTLE_MS) {
        return true;
    }

    /* ── 稳态周期采样 ────────────────────────────────── */
    if ((now_ms - s_cal_last_sample_ms) >= APP_MOTOR_CAL_SAMPLE_PERIOD_MS) {
        s_cal_last_sample_ms = now_ms;
        bsp_motor_get_feedback(feedback);
        print_cal_sample(now_ms, dir, s_cal_step_idx, s_cal_pm_now, feedback);
    }

    return true;
}

static void print_ctrl_help(void)
{
    (void)printf("[ctrl] UART commands: '+'/'-' step %urpm, '<rpm><Enter>' set speed, "
                 "'b' brake, 'r' run, 's' sync on/off, 'p' print sync, "
                 "'d' calib comp on/off, "
                 "'c' calib sweep, 'x' abort calib, 'l'/'load' enter load mode\r\n",
        (unsigned int)APP_MOTOR_DEMO_RPM_STEP);
}

static bool process_log_uart_commands(bool *running)
{
    static uint16_t rpm_acc = 0u;
    static bool rpm_pending = false;
    uint8_t ch;

    while (bsp_log_uart_read_byte(&ch)) {
        if (ch == (uint8_t)'l' || ch == (uint8_t)'L') {
            prepare_load_mode();
            return true;
        }

        /* 校准期间：'c' → busy 提示，'x' → abort，其余写目标的命令全部丢弃 */
        if (app_motor_demo_cal_is_active()) {
            if (ch == (uint8_t)'x' || ch == (uint8_t)'X') {
                app_motor_demo_cal_abort();
            } else if (ch == (uint8_t)'c' || ch == (uint8_t)'C') {
                (void)printf("[cal] busy\r\n");
            }
            /* 吞掉所有其他字符（含数字累积），避免污染 rpm_acc */
            rpm_acc = 0u;
            rpm_pending = false;
            continue;
        }

        if (ch >= (uint8_t)'0' && ch <= (uint8_t)'9') {
            uint32_t next = (uint32_t)rpm_acc * 10u + (uint32_t)(ch - (uint8_t)'0');
            rpm_acc = (next > APP_MOTOR_DEMO_MAX_RPM) ?
                APP_MOTOR_DEMO_MAX_RPM : (uint16_t)next;
            rpm_pending = true;
            continue;
        }

        if (ch == (uint8_t)'\r' || ch == (uint8_t)'\n' ||
            ch == (uint8_t)' ') {
            if (rpm_pending) {
                app_motor_demo_set_speed_rpm(rpm_acc);
                app_motor_demo_reset_sync();
                apply_run_output_if_needed(*running);
                (void)printf("[ctrl] set target=%urpm pwm=%d/1000\r\n",
                    (unsigned int)s_target_rpm, (int)s_target_pwm_pm);
                rpm_acc = 0u;
                rpm_pending = false;
            }
            continue;
        }

        rpm_acc = 0u;
        rpm_pending = false;

        switch (ch) {
        case (uint8_t)'+':
            app_motor_demo_set_speed_rpm(
                (uint16_t)(s_target_rpm + APP_MOTOR_DEMO_RPM_STEP));
            app_motor_demo_reset_sync();
            apply_run_output_if_needed(*running);
            (void)printf("[ctrl] target=%urpm pwm=%d/1000\r\n",
                (unsigned int)s_target_rpm, (int)s_target_pwm_pm);
            break;
        case (uint8_t)'-':
            app_motor_demo_set_speed_rpm((s_target_rpm > APP_MOTOR_DEMO_RPM_STEP) ?
                (uint16_t)(s_target_rpm - APP_MOTOR_DEMO_RPM_STEP) : 0u);
            app_motor_demo_reset_sync();
            apply_run_output_if_needed(*running);
            (void)printf("[ctrl] target=%urpm pwm=%d/1000\r\n",
                (unsigned int)s_target_rpm, (int)s_target_pwm_pm);
            break;
        case (uint8_t)'b':
        case (uint8_t)'B':
            *running = false;
            brake_now();
            (void)printf("[ctrl] brake\r\n");
            break;
        case (uint8_t)'r':
        case (uint8_t)'R':
            *running = true;
            app_motor_demo_reset_sync();
            apply_run_output_if_needed(*running);
            (void)printf("[ctrl] run target=%urpm pwm=%d/1000\r\n",
                (unsigned int)s_target_rpm, (int)s_target_pwm_pm);
            break;
        case (uint8_t)'s':
        case (uint8_t)'S':
            app_motor_demo_set_sync_enabled(!s_sync.enabled);
            apply_run_output_if_needed(*running);
            (void)printf("[ctrl] sync=%s kp=%d ki=%d maxCorr=%d\r\n",
                s_sync.enabled ? "on" : "off",
                (int)s_sync.kp_pm_per_rpm,
                (int)s_sync.ki_pm_per_rpm_step,
                APP_MOTOR_SYNC_MAX_CORRECTION_PM);
            break;
        case (uint8_t)'p':
        case (uint8_t)'P':
            (void)printf(
                "[ctrl] sync=%s err=%d corr=%d cmdL=%d cmdR=%d kp=%d ki=%d cal_apply_comp=%u rf_scale=%u\r\n",
                s_sync.enabled ? "on" : "off",
                (int)s_sync.rpm_error,
                (int)s_sync.correction_pm,
                (int)s_sync.left_cmd_pm,
                (int)s_sync.right_cmd_pm,
                (int)s_sync.kp_pm_per_rpm,
                (int)s_sync.ki_pm_per_rpm_step,
                s_cal_apply_comp ? 1u : 0u,
                bsp_motor_get_right_forward_scale_enabled() ? 1u : 0u);
            break;
        case (uint8_t)'d':
        case (uint8_t)'D':
            s_cal_apply_comp = !s_cal_apply_comp;
            (void)printf("[ctrl] cal_apply_comp=%u\r\n",
                s_cal_apply_comp ? 1u : 0u);
            break;
        case (uint8_t)'f':
        case (uint8_t)'F':
            /* 保留提示兼容旧操作；实际右路 5% 基础偏置已下沉到 bsp_motor。 */
            rpm_acc = 0u;
            rpm_pending = false;
            (void)printf("[ctrl] right forward bias is fixed in bsp_motor\r\n");
            break;
        case (uint8_t)'c':
        case (uint8_t)'C':
            (void)app_motor_demo_cal_start();
            break;
        case (uint8_t)'x':
        case (uint8_t)'X':
            (void)printf("[cal] not active\r\n");
            break;
        case (uint8_t)'h':
        case (uint8_t)'H':
        case (uint8_t)'?':
            print_ctrl_help();
            break;
        default:
            break;
        }
    }
    return false;
}

static void print_boot_banner(void)
{
    (void)printf("[boot] stage2 motor demo start\r\n");
    (void)printf("[boot] target=%urpm pwm=%d/1000 max=%urpm\r\n",
        (unsigned int)s_target_rpm, (int)s_target_pwm_pm,
        (unsigned int)APP_MOTOR_DEMO_MAX_RPM);
    (void)printf("[boot] motor sync enabled kp=%d ki=%d maxCorr=%d period=%ums\r\n",
        (int)s_sync.kp_pm_per_rpm,
        (int)s_sync.ki_pm_per_rpm_step,
        APP_MOTOR_SYNC_MAX_CORRECTION_PM,
        (unsigned int)APP_MOTOR_SYNC_PERIOD_MS);
    (void)printf("[boot] send 'l' or 'load' on UART to enter load balance mode\r\n");
    print_ctrl_help();
}

static void handle_sync_tick(uint32_t now_ms,
                             uint32_t *last_sync_ms,
                             bool running,
                             bsp_motor_feedback_t *feedback)
{
    if ((now_ms - *last_sync_ms) < APP_MOTOR_SYNC_PERIOD_MS) {
        return;
    }

    *last_sync_ms = now_ms;
    bsp_motor_get_feedback(feedback);
    motor_sync_step(feedback, running);
}

static void print_feedback_log(uint32_t now_ms,
                               bool running,
                               const bsp_motor_feedback_t *feedback)
{
    int32_t left_cdeg = (int32_t)(feedback->left_angle_deg * 100.0f);
    int32_t right_cdeg = (int32_t)(feedback->right_angle_deg * 100.0f);

    (void)printf(
        "[enc] t=%lums L=%ld(%ld.%02ld deg) R=%ld(%ld.%02ld deg) "
        "rpmL=%ld rpmR=%ld target=%urpm state=%s sync=%u "
        "err=%d corr=%d cmdL=%d cmdR=%d\r\n",
        (unsigned long)now_ms,
        (long)feedback->left_count,
        (long)(left_cdeg / 100),
        (long)(left_cdeg < 0 ? -left_cdeg : left_cdeg) % 100,
        (long)feedback->right_count,
        (long)(right_cdeg / 100),
        (long)(right_cdeg < 0 ? -right_cdeg : right_cdeg) % 100,
        (long)feedback->left_speed_rpm,
        (long)feedback->right_speed_rpm,
        (unsigned int)s_target_rpm,
        running ? "run" : "brake",
        s_sync.enabled ? 1u : 0u,
        (int)s_sync.rpm_error,
        (int)s_sync.correction_pm,
        (int)s_sync.left_cmd_pm,
        (int)s_sync.right_cmd_pm);
}

static void handle_log_tick(uint32_t now_ms,
                            uint32_t *last_log_ms,
                            bool running,
                            bsp_motor_feedback_t *feedback)
{
    if ((now_ms - *last_log_ms) < APP_MOTOR_LOG_PERIOD_MS) {
        return;
    }

    *last_log_ms = now_ms;
    bsp_motor_get_feedback(feedback);
    print_feedback_log(now_ms, running, feedback);
}

static void handle_led_tick(uint32_t now_ms, uint32_t *last_led_ms)
{
    if ((now_ms - *last_led_ms) < APP_MOTOR_HEARTBEAT_PERIOD_MS) {
        return;
    }

    *last_led_ms = now_ms;
    DL_GPIO_togglePins(BSP_LED_G_PORT, BSP_LED_G_PIN);
}

bool app_motor_demo_run(void)
{
    bool running = true;
    uint32_t last_sync_ms = 0u;
    uint32_t last_log_ms  = 0u;
    uint32_t last_led_ms  = 0u;
    uint32_t last_batt_ms = 0u;
    bsp_motor_feedback_t feedback;

    bsp_motor_enable(true);
    app_motor_demo_reset_sync();
    apply_motor_output(0);
    print_boot_banner();

    for (;;) {
        if (!bsp_systick_consume_tick()) {
            __WFI();
            continue;
        }

        bsp_motor_update();
        if (process_log_uart_commands(&running)) {
            return true;
        }

        uint32_t now_ms = bsp_systick_get_ms();

        /* 100 Hz 电池采样（校准期间与普通 demo 均有效，温暖 EMA 滤波器） */
        if ((now_ms - last_batt_ms) >= APP_MOTOR_BATT_PERIOD_MS) {
            last_batt_ms = now_ms;
            bsp_battery_update();
        }

        /* 校准期间：用 cal tick 替代 sync tick 和 enc 日志 */
        if (handle_calibration_tick(now_ms, &feedback)) {
            handle_led_tick(now_ms, &last_led_ms);
            continue;
        }

        handle_sync_tick(now_ms, &last_sync_ms, running, &feedback);

        handle_log_tick(now_ms, &last_log_ms, running, &feedback);
        handle_led_tick(now_ms, &last_led_ms);
    }
}
