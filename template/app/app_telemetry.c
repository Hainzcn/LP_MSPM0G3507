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
#include "bsp_imu_uart.h"
#include "ms901m.h"
#include "vofa.h"

#include <stdio.h>

/* 1 ms tick 周期下的相位计数器：每攒够 N 次 tick 触发一次相应任务 */
#define PHASE_VOFA_TICKS    10u    /* 100 Hz */
#define PHASE_LED_TICKS     200u   /* 5 Hz   */
#define PHASE_LOG_TICKS     1000u  /* 1 Hz   */

/* 单拍最多从 UART2 RX 环缓拉走多少字节给 ms901m 解析。
 * MS901M 默认 5 帧 × 200 Hz × ~15 B ≈ 15 kB/s = 15 B/ms，
 * 64 B 单拍裕度 4×，足够吸收偶发抖动。 */
#define IMU_DRAIN_CHUNK     64u

void app_telemetry_run(void)
{
    /* 把 vofa 写函数注入为蓝牙 UART3 阻塞写。后续若想换 UART 只动这一行 */
    vofa_set_writer(bsp_bt_uart_write);

    uint32_t tick_count    = 0u;
    uint32_t last_total_rx = bsp_k230_uart_total_rx();

    ms901m_snapshot_t snap = { 0 };

    for (;;) {
        if (!bsp_systick_consume_tick()) {
            /* 没到下一个 1 ms：低功耗等待，被任意中断唤醒 */
            __WFI();
            continue;
        }
        tick_count++;

        /* ---- 1 kHz：drain UART2 → 喂 ms901m 状态机 → 取最新 snapshot ----
         * UART RX FIFO 半满中断已把字节排入 256 B 环缓，本 tick 只做拷贝 +
         * 解析，I2C 阻塞读已下线，整体 ISR/CPU 负担更轻。 */
        uint8_t buf[IMU_DRAIN_CHUNK];
        size_t  got = bsp_imu_uart_rx_pop_bulk(buf, sizeof(buf));
        if (got > 0u) {
            ms901m_feed_bytes(buf, got);
        }
        ms901m_get_snapshot(&snap);
        /* 注：snap 即使本拍没新字节也会保留上一帧值（has_* 标志已在 main
         * 启动期等到首帧后置位），让下游 100 Hz / 1 Hz 任务始终拿到稳定数据。 */

        /* ---- 100 Hz：4 通道 VOFA JustFloat ----
         * Stage 1.5 通道映射（vs Stage 1 旧版 pitch/rate/acc/temp）：
         *   CH0 = pitch_deg     (0x01 帧；MS901M 板载 EKF 输出，平衡环主用)
         *   CH1 = roll_deg      (0x01 帧；用于装机姿态校验)
         *   CH2 = gy_dps        (0x03 帧；俯仰角速度，平衡环 D 项参考)
         *   CH3 = temp_c        (0x04 帧；监控热漂)            */
        if ((tick_count % PHASE_VOFA_TICKS) == 0u) {
            float ch[4] = {
                snap.pitch_deg,
                snap.roll_deg,
                snap.gy_dps,
                snap.temp_c,
            };
            vofa_send(ch, 4u);
        }

        /* ---- 5 Hz：LED 心跳（绿灯翻转） ----
         * BSP_LED_G_PORT = GPIOB，宏定义见 bsp_gpio.h；
         * GPIO 由 BSP 接管的背景见 docs/TaskLog/Stage1-IMU-BT-Telemetry.md §8.5 */
        if ((tick_count % PHASE_LED_TICKS) == 0u) {
            DL_GPIO_togglePins(BSP_LED_G_PORT, BSP_LED_G_PIN);
        }

        /* ---- 1 Hz：UART0 调试日志 ----
         * 输出 pitch + gy + temp + MS901M 错误统计 + K230 RX 字节速率，
         * 任一字段异常都能在 1 s 内被肉眼捕获。 */
        if ((tick_count % PHASE_LOG_TICKS) == 0u) {
            uint32_t total_rx = bsp_k230_uart_total_rx();
            uint32_t delta_rx = total_rx - last_total_rx;
            last_total_rx = total_rx;
            (void)printf("[hb] t=%lus pitch=%.2f roll=%.2f gy=%.2f T=%.1fC "
                "ms901m_good=%lu bad=%lu over=%lu k230_rx=%lub/s\n",
                (unsigned long)(tick_count / 1000u),
                (double)snap.pitch_deg,
                (double)snap.roll_deg,
                (double)snap.gy_dps,
                (double)snap.temp_c,
                (unsigned long)ms901m_good_frames(),
                (unsigned long)ms901m_bad_frames(),
                (unsigned long)bsp_imu_uart_rx_overrun(),
                (unsigned long)delta_rx);
        }
    }
}
