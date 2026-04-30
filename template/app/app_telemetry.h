/**
 * @file    app_telemetry.h
 * @brief   阶段 1 顶层调度：IMU 采样 + 互补滤波 + VOFA 蓝牙转发 + LED 心跳
 *          + K230 RX 字节计数日志。
 *
 *  调度策略：单任务轮询 + SysTick 节拍标志。无 RTOS。
 *    -  1 kHz：mpu6050_read_raw → att_filter_update
 *    - 100 Hz：通过 vofa_send 把 4 路 float 推到蓝牙 UART2
 *    -   5 Hz：翻转 LED_STATUS_G，作为整车 "heartbeat" 灯
 *    -   1 Hz：UART0 printf "[hb] pitch=... rx=..." 用于 XDS-UART 自测
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
 *      bsp_k230_uart_init / mpu6050_init / att_filter_init 全部完成。
 */
void app_telemetry_run(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_TELEMETRY_H */
