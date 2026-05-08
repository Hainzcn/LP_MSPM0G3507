/**
 * @file    app_motor_demo.c
 * @brief   电机驱动演示：S1 切换正反转，串口输出左右轮角度。
 */

#include "app_motor_demo.h"

#include "bsp_gpio.h"
#include "bsp_log_uart.h"
#include "bsp_motor.h"
#include "bsp_systick.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define APP_MOTOR_DEMO_PWM_PERMILLE                (350)
#define APP_MOTOR_LOG_PERIOD_MS                    (100u)
#define APP_MOTOR_HEARTBEAT_PERIOD_MS              (250u)

void app_motor_demo_run(void)
{
    bool forward = true;
    uint32_t last_log_ms = 0u;
    uint32_t last_led_ms = 0u;
    bsp_motor_feedback_t feedback;

    bsp_motor_enable(true);
    bsp_motor_set_output(APP_MOTOR_DEMO_PWM_PERMILLE, APP_MOTOR_DEMO_PWM_PERMILLE);

    (void)printf("[boot] stage2 motor demo start\r\n");
    (void)printf("[boot] press S1(PA18) to toggle motor direction\r\n");

    for (;;) {
        if (!bsp_systick_consume_tick()) {
            __WFI();
            continue;
        }

        bsp_motor_update();

        if (bsp_motor_consume_toggle_request()) {
            forward = !forward;
            if (forward) {
                bsp_motor_set_output(APP_MOTOR_DEMO_PWM_PERMILLE,
                    APP_MOTOR_DEMO_PWM_PERMILLE);
            } else {
                bsp_motor_set_output(-APP_MOTOR_DEMO_PWM_PERMILLE,
                    -APP_MOTOR_DEMO_PWM_PERMILLE);
            }

            (void)printf("[motor] dir=%s pwm=%d/1000\r\n",
                forward ? "forward" : "reverse",
                APP_MOTOR_DEMO_PWM_PERMILLE);
            DL_GPIO_togglePins(BSP_LED_B_PORT, BSP_LED_B_PIN);
        }

        uint32_t now_ms = bsp_systick_get_ms();
        if ((now_ms - last_log_ms) >= APP_MOTOR_LOG_PERIOD_MS) {
            int32_t left_cdeg;
            int32_t right_cdeg;

            last_log_ms = now_ms;
            bsp_motor_get_feedback(&feedback);
            left_cdeg = (int32_t)(feedback.left_angle_deg * 100.0f);
            right_cdeg = (int32_t)(feedback.right_angle_deg * 100.0f);
            (void)printf(
                "[enc] t=%lums L=%ld(%ld.%02ld deg) R=%ld(%ld.%02ld deg)\r\n",
                (unsigned long)now_ms,
                (long)feedback.left_count,
                (long)(left_cdeg / 100), (long)(left_cdeg < 0 ? -left_cdeg : left_cdeg) % 100,
                (long)feedback.right_count,
                (long)(right_cdeg / 100), (long)(right_cdeg < 0 ? -right_cdeg : right_cdeg) % 100);
        }

        if ((now_ms - last_led_ms) >= APP_MOTOR_HEARTBEAT_PERIOD_MS) {
            last_led_ms = now_ms;
            DL_GPIO_togglePins(BSP_LED_G_PORT, BSP_LED_G_PIN);
        }
    }
}
