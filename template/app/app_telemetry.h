/**
 * @file    app_telemetry.h
 * @brief   阶段 1 顶层调度：MS901M UART RX drain + 解析 + VOFA 蓝牙转发
 *          + LED 心跳 + K230 RX 字节计数日志。
 *
 *  调度策略：单任务轮询 + SysTick 节拍标志。无 RTOS。
 *    -  1 kHz：bsp_imu_uart_rx_pop_bulk → ms901m_feed_bytes
 *    - 100 Hz：通过 vofa_send 把 4 路 float 推到蓝牙 UART3
 *    -   5 Hz：翻转 LED_STATUS_G，作为整车 "heartbeat" 灯
 *    -   1 Hz：UART0 printf "[hb] pitch=... bad=... rx=..." 用于 XDS-UART 自测
 *
 *  Stage 1.5 变更：1 kHz 任务由原 mpu6050_read_raw + att_filter_update 改为
 *  「drain UART2 → 喂状态机 → 取最新 snapshot」。VOFA 通道映射也随之改为
 *  pitch / roll / gy(0x03) / temp(0x04)。
 */

#ifndef APP_TELEMETRY_H
#define APP_TELEMETRY_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  主循环入口。该函数永不返回，被 main() 调用。
 *
 *  调用前必须保证：
 *    - SYSCFG_DL_init() 已完成；
 *    - bsp_systick_init / bsp_log_uart_init / bsp_bt_uart_init /
 *      bsp_k230_uart_init / bsp_imu_uart_init / ms901m_init 全部完成；
 *    - main 已经通过 wait_for_ms901m_attitude 验证 0x01 帧在线。
 */
void app_telemetry_run(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_TELEMETRY_H */
