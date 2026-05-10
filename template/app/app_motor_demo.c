/**
 * @file    app_motor_demo.c
 * @brief   电机驱动演示：S1 急刹 / 启动，串口输出左右轮转速与角度。
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

#define APP_MOTOR_DEMO_MAX_RPM                     (620u)
#define APP_MOTOR_DEMO_DEFAULT_RPM                 (620u)
#define APP_MOTOR_DEMO_BRAKE_MS                    (120u)
#define APP_MOTOR_DEMO_RPM_STEP                    (20u)
#define APP_MOTOR_SYNC_PERIOD_MS                   (50u)
/*
 * 前馈接管了正转稳态误差（~13 rpm @1000‰），PI 只需处理瞬态与反转残差，
 * 因此 Kp 从 8 降至 4，Ki 从 1 降至 0（纯比例）以避免积分抖动。
 * 若反转仍有残余偏差可将 Ki 恢复为 1。
 */
#define APP_MOTOR_SYNC_KP_PM_PER_RPM               (4)
#define APP_MOTOR_SYNC_KI_PM_PER_RPM_STEP          (0)
#define APP_MOTOR_SYNC_MAX_CORRECTION_PM           (200)

/*
 * 右电机正转前馈系数（× 1000）。
 * 校准值：正转斜率比 = 0.2559 / 0.2432 = 1.0522，
 * 右轮需少输出约 4.96% ≈ 50‰/1000‰。
 * 仅对 target_pm > 0（正转）生效；反转由 PI 处理 ±2 rpm 小残差。
 */
#ifndef APP_MOTOR_SYNC_FF_RIGHT_X1000
#define APP_MOTOR_SYNC_FF_RIGHT_X1000              (50)
#endif
#define APP_MOTOR_LOG_PERIOD_MS                    (100u)
#define APP_MOTOR_HEARTBEAT_PERIOD_MS              (250u)
#define APP_MOTOR_BATT_PERIOD_MS                   (10u)

/* ── 校准扫描可调宏 ─────────────────────────────────────────────────────── */
#ifndef APP_MOTOR_CAL_PWM_START_PM
#define APP_MOTOR_CAL_PWM_START_PM     (100)
#endif
#ifndef APP_MOTOR_CAL_PWM_END_PM
#define APP_MOTOR_CAL_PWM_END_PM       (1000)
#endif
#ifndef APP_MOTOR_CAL_PWM_STEP_PM
#define APP_MOTOR_CAL_PWM_STEP_PM      (50)
#endif
#ifndef APP_MOTOR_CAL_DWELL_MS
#define APP_MOTOR_CAL_DWELL_MS         (1500u)
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
    .ff_pm = 0,
};
static int16_t s_sync_i_pm = 0;
static int16_t s_sync_ff_right_x1000 = APP_MOTOR_SYNC_FF_RIGHT_X1000;

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

void app_motor_demo_set_ff_right(int16_t ff_x1000)
{
    if (ff_x1000 < 0) {
        ff_x1000 = 0;
    }
    if (ff_x1000 > 200) {
        ff_x1000 = 200;
    }
    s_sync_ff_right_x1000 = ff_x1000;
}

int16_t app_motor_demo_get_ff_right(void)
{
    return s_sync_ff_right_x1000;
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
    /*
     * 右电机静态前馈：仅在正转（target_pm > 0）时生效。
     * ff_pm = target_pm × ff_x1000 / 1000，从右轮 PWM 中扣除，
     * 抵消 TB6612 B 通道比 A 通道高约 5.2% 的固有速度差。
     * 反转时 ff_pm = 0，由 PI 处理 ±2 rpm 的小残差。
     */
    int16_t ff_pm = (s_target_pwm_pm > 0)
        ? (int16_t)((int32_t)s_target_pwm_pm * s_sync_ff_right_x1000 / 1000)
        : (int16_t)0;

    int16_t left_pm  = clamp_pm((int32_t)s_target_pwm_pm + correction_pm);
    int16_t right_pm = clamp_pm((int32_t)s_target_pwm_pm - ff_pm - correction_pm);

    s_sync.ff_pm       = ff_pm;
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

/* ========================================================================== */
/* 校准扫描 — 日志打印                                                          */
/* ========================================================================== */

static void print_cal_header(int8_t dir)
{
    int16_t total = s_cal_total_steps;
    (void)printf(
        "[cal] start dir=%+d steps=%d pm_start=%d pm_end=%d step=%d "
        "dwell_ms=%u settle_ms=%u\r\n",
        (int)dir,
        (int)total,
        APP_MOTOR_CAL_PWM_START_PM,
        APP_MOTOR_CAL_PWM_END_PM,
        APP_MOTOR_CAL_PWM_STEP_PM,
        (unsigned int)APP_MOTOR_CAL_DWELL_MS,
        (unsigned int)APP_MOTOR_CAL_SETTLE_MS);
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

    s_sync.enabled = false;
    s_sync_i_pm    = 0;
    s_sync.correction_pm = 0;

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
                bsp_motor_set_output(s_cal_saved_pwm_pm, s_cal_saved_pwm_pm);
                s_target_pwm_pm  = s_cal_saved_pwm_pm;
                s_sync.enabled   = s_cal_saved_sync_enabled;
                s_sync_i_pm      = 0;
                s_sync.correction_pm = 0;
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
                 "'f<val><Enter>' set ff (0~200), "
                 "'c' calib sweep, 'x' abort calib\r\n",
        (unsigned int)APP_MOTOR_DEMO_RPM_STEP);
}

/* UART 命令解析模式：普通转速输入 vs 前馈系数输入（'f' 前缀） */
typedef enum { CMD_MODE_RPM = 0, CMD_MODE_FF = 1 } cmd_input_mode_t;

static void process_log_uart_commands(bool *running)
{
    static uint16_t rpm_acc = 0u;
    static bool rpm_pending = false;
    static cmd_input_mode_t cmd_mode = CMD_MODE_RPM;
    uint8_t ch;

    while (bsp_log_uart_read_byte(&ch)) {
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
            if (cmd_mode == CMD_MODE_FF) {
                rpm_acc = (next > 200u) ? 200u : (uint16_t)next;
            } else {
                rpm_acc = (next > APP_MOTOR_DEMO_MAX_RPM) ?
                    APP_MOTOR_DEMO_MAX_RPM : (uint16_t)next;
            }
            rpm_pending = true;
            continue;
        }

        if (ch == (uint8_t)'\r' || ch == (uint8_t)'\n' ||
            ch == (uint8_t)' ') {
            if (rpm_pending) {
                if (cmd_mode == CMD_MODE_FF) {
                    app_motor_demo_set_ff_right((int16_t)rpm_acc);
                    (void)printf("[ctrl] ff_right=%d/1000 ff_pm=%d\r\n",
                        (int)s_sync_ff_right_x1000, (int)s_sync.ff_pm);
                } else {
                    app_motor_demo_set_speed_rpm(rpm_acc);
                    app_motor_demo_reset_sync();
                    apply_run_output_if_needed(*running);
                    (void)printf("[ctrl] set target=%urpm pwm=%d/1000\r\n",
                        (unsigned int)s_target_rpm, (int)s_target_pwm_pm);
                }
                rpm_acc = 0u;
                rpm_pending = false;
                cmd_mode = CMD_MODE_RPM;
            }
            continue;
        }

        rpm_acc = 0u;
        rpm_pending = false;
        cmd_mode = CMD_MODE_RPM;

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
                "[ctrl] sync=%s err=%d corr=%d ff=%d cmdL=%d cmdR=%d "
                "kp=%d ki=%d ff_x1000=%d\r\n",
                s_sync.enabled ? "on" : "off",
                (int)s_sync.rpm_error,
                (int)s_sync.correction_pm,
                (int)s_sync.ff_pm,
                (int)s_sync.left_cmd_pm,
                (int)s_sync.right_cmd_pm,
                (int)s_sync.kp_pm_per_rpm,
                (int)s_sync.ki_pm_per_rpm_step,
                (int)s_sync_ff_right_x1000);
            break;
        case (uint8_t)'f':
        case (uint8_t)'F':
            /* 'f<value>\n' 设置右电机前馈系数（× 1000，范围 0~200）。
             * 例如 'f50\n' = 5.0%。不带数字时打印当前值。 */
            cmd_mode = CMD_MODE_FF;
            rpm_acc = 0u;
            rpm_pending = false;
            (void)printf("[ctrl] ff input mode: enter value (0~200, current=%d)\r\n",
                (int)s_sync_ff_right_x1000);
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
}

/* S2/SW2 当前硬件冲突：LaunchPad J15 默认把 SW2 接到 PA16，而 PA16 是 AIN2。
 * 若后续把 S2 飞线到空闲 GPIO，可在 bsp_gpio.h 补 BSP_LOAD_BTN_* 后打开这里。 */
#ifndef APP_MOTOR_DEMO_ENABLE_LOAD_BUTTON
#define APP_MOTOR_DEMO_ENABLE_LOAD_BUTTON          (0)
#endif

#if APP_MOTOR_DEMO_ENABLE_LOAD_BUTTON
static bool consume_load_button_request(uint32_t now_ms)
{
    static uint32_t last_press_ms = 0u;
    static bool was_pressed = false;

    bool pressed = ((DL_GPIO_readPins(BSP_LOAD_BTN_PORT, BSP_LOAD_BTN_PIN) &
        BSP_LOAD_BTN_PIN) == 0u);
    bool rising_event = false;

    if (pressed && !was_pressed &&
        ((now_ms - last_press_ms) >= BSP_MOTOR_BTN_DEBOUNCE_MS)) {
        last_press_ms = now_ms;
        rising_event = true;
    }
    was_pressed = pressed;
    return rising_event;
}
#else
static bool consume_load_button_request(uint32_t now_ms)
{
    (void)now_ms;
    return false;
}
#endif

static void print_boot_banner(void)
{
    (void)printf("[boot] stage2 motor demo start\r\n");
    (void)printf("[boot] target=%urpm pwm=%d/1000 max=%urpm\r\n",
        (unsigned int)s_target_rpm, (int)s_target_pwm_pm,
        (unsigned int)APP_MOTOR_DEMO_MAX_RPM);
    (void)printf("[boot] motor sync enabled kp=%d ki=%d maxCorr=%d period=%ums "
                 "ff_right=%d/1000\r\n",
        (int)s_sync.kp_pm_per_rpm,
        (int)s_sync.ki_pm_per_rpm_step,
        APP_MOTOR_SYNC_MAX_CORRECTION_PM,
        (unsigned int)APP_MOTOR_SYNC_PERIOD_MS,
        (int)s_sync_ff_right_x1000);
    (void)printf("[boot] press S1(PA18) to brake/start both motors\r\n");
    (void)printf("[boot] S2 load-mode request is disabled until its GPIO is rerouted from PA16/AIN2\r\n");
    print_ctrl_help();
}

static void handle_start_button(bool *running)
{
    if (!bsp_motor_consume_toggle_request()) {
        return;
    }

    (void)printf("[btn] S1 pressed (irq=%lu poll=%lu raw=%u active=%u)\r\n",
        (unsigned long)bsp_motor_get_button_irq_count(),
        (unsigned long)bsp_motor_get_button_poll_count(),
        bsp_motor_get_start_button_raw_level() ? 1u : 0u,
        bsp_motor_is_start_button_active() ? 1u : 0u);

    /* 校准期间 S1 等价于 abort */
    if (app_motor_demo_cal_is_active()) {
        app_motor_demo_cal_abort();
        *running = false;
        DL_GPIO_togglePins(BSP_LED_B_PORT, BSP_LED_B_PIN);
        return;
    }

    *running = !(*running);
    (void)printf("[btn] S1: %s\r\n", *running ? "start" : "brake");

    if (*running) {
        bsp_motor_enable(true);
        app_motor_demo_reset_sync();
        apply_run_output_if_needed(true);
    } else {
        brake_now();
    }

    (void)printf("[motor] state=%s target=%urpm pwm=%d/1000\r\n",
        *running ? "run" : "brake",
        (unsigned int)s_target_rpm, (int)s_target_pwm_pm);
    DL_GPIO_togglePins(BSP_LED_B_PORT, BSP_LED_B_PIN);
}

static bool handle_load_button(uint32_t now_ms)
{
    if (!consume_load_button_request(now_ms)) {
        return false;
    }

    (void)printf("[btn] S2 pressed: enter load balance mode\r\n");
    bsp_motor_stop();
    bsp_motor_enable(false);
    return true;
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
        "err=%d corr=%d cmdL=%d cmdR=%d "
        "btn_irq=%lu btn_poll=%lu raw=%u active=%u\r\n",
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
        (int)s_sync.right_cmd_pm,
        (unsigned long)bsp_motor_get_button_irq_count(),
        (unsigned long)bsp_motor_get_button_poll_count(),
        bsp_motor_get_start_button_raw_level() ? 1u : 0u,
        bsp_motor_is_start_button_active() ? 1u : 0u);
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
        process_log_uart_commands(&running);
        handle_start_button(&running);

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

        if (handle_load_button(now_ms)) {
            return true;
        }

        handle_log_tick(now_ms, &last_log_ms, running, &feedback);
        handle_led_tick(now_ms, &last_led_ms);
    }
}
