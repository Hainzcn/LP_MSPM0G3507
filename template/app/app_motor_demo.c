/**
 * @file    app_motor_demo.c
 * @brief   电机驱动演示：S1 急刹 / 启动，串口输出左右轮转速与角度。
 */

#include "app_motor_demo.h"

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
#define APP_MOTOR_SYNC_KP_PM_PER_RPM               (8)
#define APP_MOTOR_SYNC_KI_PM_PER_RPM_STEP          (1)
#define APP_MOTOR_SYNC_MAX_CORRECTION_PM           (350)
#define APP_MOTOR_LOG_PERIOD_MS                    (100u)
#define APP_MOTOR_HEARTBEAT_PERIOD_MS              (250u)

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
    int16_t left_pm = clamp_pm((int32_t)s_target_pwm_pm + correction_pm);
    int16_t right_pm = clamp_pm((int32_t)s_target_pwm_pm - correction_pm);

    s_sync.left_cmd_pm = left_pm;
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

static void print_ctrl_help(void)
{
    (void)printf("[ctrl] UART commands: '+'/'-' step %urpm, '<rpm><Enter>' set speed, "
                 "'b' brake, 'r' run, 's' sync on/off, 'p' print sync\r\n",
        (unsigned int)APP_MOTOR_DEMO_RPM_STEP);
}

static void process_log_uart_commands(bool *running)
{
    static uint16_t rpm_acc = 0u;
    static bool rpm_pending = false;
    uint8_t ch;

    while (bsp_log_uart_read_byte(&ch)) {
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
            (void)printf("[ctrl] sync=%s err=%d corr=%d cmdL=%d cmdR=%d kp=%d ki=%d\r\n",
                s_sync.enabled ? "on" : "off",
                (int)s_sync.rpm_error,
                (int)s_sync.correction_pm,
                (int)s_sync.left_cmd_pm,
                (int)s_sync.right_cmd_pm,
                (int)s_sync.kp_pm_per_rpm,
                (int)s_sync.ki_pm_per_rpm_step);
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
    (void)printf("[boot] motor sync enabled kp=%d ki=%d maxCorr=%d period=%ums\r\n",
        (int)s_sync.kp_pm_per_rpm,
        (int)s_sync.ki_pm_per_rpm_step,
        APP_MOTOR_SYNC_MAX_CORRECTION_PM,
        (unsigned int)APP_MOTOR_SYNC_PERIOD_MS);
    (void)printf("[boot] press S1(PA18) to brake/start both motors\r\n");
    (void)printf("[boot] S2 load-mode request is disabled until its GPIO is rerouted from PA16/AIN2\r\n");
    print_ctrl_help();
}

static void handle_start_button(bool *running)
{
    if (!bsp_motor_consume_toggle_request()) {
        return;
    }

    *running = !(*running);
    (void)printf("[btn] S1 pressed: %s (irq=%lu poll=%lu raw=%u active=%u)\r\n",
        *running ? "start" : "brake",
        (unsigned long)bsp_motor_get_button_irq_count(),
        (unsigned long)bsp_motor_get_button_poll_count(),
        bsp_motor_get_start_button_raw_level() ? 1u : 0u,
        bsp_motor_is_start_button_active() ? 1u : 0u);

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
    uint32_t last_log_ms = 0u;
    uint32_t last_led_ms = 0u;
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
        handle_sync_tick(now_ms, &last_sync_ms, running, &feedback);

        if (handle_load_button(now_ms)) {
            return true;
        }

        handle_log_tick(now_ms, &last_log_ms, running, &feedback);
        handle_led_tick(now_ms, &last_led_ms);
    }
}
