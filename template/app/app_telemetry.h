/**
 * @file    app_telemetry.h
 * @brief   阶段 1 顶层调度：MS901M UART RX drain + 解析 + LED 心跳
 *          + K230 RX 字节计数日志。
 *
 *  调度策略：单任务轮询 + SysTick 节拍标志。无 RTOS。
 *    -  1 kHz：bsp_imu_uart_rx_pop_bulk → ms901m_feed_bytes
 *    -   5 Hz：翻转 LED_STATUS_G，作为整车 "heartbeat" 灯
 *    -   1 Hz：UART0 printf "[hb] pitch=... roll=... gy=... T=... bad=... rx=..."
 *              用于 XDS-UART 自测（同时也是 Stage 1.6 起姿态可视化的主路径）
 *
 *  Stage 1.5 变更：1 kHz 任务由原 mpu6050_read_raw + att_filter_update 改为
 *  「drain UART → 喂状态机 → 取最新 snapshot」。
 *
 *  Stage 1.6 变更：蓝牙 UART3 整体下线（PB12/PB13 让给 IMU），原 100 Hz
 *  VOFA+ JustFloat 推送已移除；姿态可视化改走 1 Hz XDS-UART printf。
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
 *    - bsp_systick_init / bsp_log_uart_init / bsp_k230_uart_init /
 *      bsp_imu_uart_init / ms901m_init 全部完成；
 *    - main 已经通过 wait_for_ms901m_attitude 验证 0x01 帧在线。
 */
void app_telemetry_run(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_TELEMETRY_H */
