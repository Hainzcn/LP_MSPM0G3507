/**
 * @file    app_telemetry.c
 * @brief   阶段 1 调度实现，详见 app_telemetry.h。
 */

#include "app_telemetry.h"
#include "ti_msp_dl_config.h"

#include "bsp_gpio.h"
#include "bsp_systick.h"
#include "bsp_bt_uart.h"
#include "bsp_k230_uart.h"
#include "mpu6050.h"
#include "att_filter.h"
#include "vofa.h"

#include <stdio.h>

/* 1 ms tick 周期下的相位计数器：每攒够 N 次 tick 触发一次相应任务 */
#define PHASE_VOFA_TICKS    10u    /* 100 Hz */
#define PHASE_LED_TICKS     200u   /* 5 Hz   */
#define PHASE_LOG_TICKS     1000u  /* 1 Hz   */

void app_telemetry_run(void)
{
    /* 把 vofa 写函数注入为蓝牙 UART3 阻塞写。后续若想换 UART 只动这一行 */
    vofa_set_writer(bsp_bt_uart_write);

    uint32_t tick_count = 0u;
    uint32_t last_total_rx = bsp_k230_uart_total_rx();

    att_state_t att = { 0 };

    for (;;) {
        if (!bsp_systick_consume_tick()) {
            /* 没到下一个 1 ms：低功耗等待，被任意中断唤醒 */
            __WFI();
            continue;
        }
        tick_count++;

        /* ---- 1 kHz：IMU 采样 + 互补滤波 ---- */
        mpu6050_raw_t raw;
        if (mpu6050_read_raw(&raw) == 0) {
            att_filter_update(&raw, 0.001f, &att);
        }
        /* I²C 偶发读失败时不更新 att，保持上一拍输出，让上层观察到丢点 */

        /* ---- 100 Hz：4 通道 VOFA JustFloat ---- */
        if ((tick_count % PHASE_VOFA_TICKS) == 0u) {
            float ch[4] = {
                att.pitch_deg,
                att.pitch_rate_dps,
                att.pitch_acc_deg,
                att.temp_c,
            };
            vofa_send(ch, 4u);
        }

        /* ---- 5 Hz：LED 心跳（绿灯翻转） ----
         * BSP_LED_G_PORT = GPIOB，宏定义见 bsp_gpio.h；
         * GPIO 由 BSP 接管的背景见 docs/TaskLog/Stage1-IMU-BT-Telemetry.md §8.5 */
        if ((tick_count % PHASE_LED_TICKS) == 0u) {
            DL_GPIO_togglePins(BSP_LED_G_PORT, BSP_LED_G_PIN);
        }

        /* ---- 1 Hz：UART0 调试日志 ---- */
        if ((tick_count % PHASE_LOG_TICKS) == 0u) {
            uint32_t total_rx = bsp_k230_uart_total_rx();
            uint32_t delta_rx = total_rx - last_total_rx;
            last_total_rx = total_rx;
            (void)printf("[hb] t=%lus pitch=%.2f rate=%.2f acc=%.2f T=%.1fC k230_rx=%lub/s\n",
                (unsigned long)(tick_count / 1000u),
                (double)att.pitch_deg,
                (double)att.pitch_rate_dps,
                (double)att.pitch_acc_deg,
                (double)att.temp_c,
                (unsigned long)delta_rx);
        }
    }
}
