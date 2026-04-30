/**
 * @file    bsp_log_uart.h
 * @brief   UART0（XDS-UART 桥）调试日志：fputc retarget，让 printf 可用。
 *
 * 走板载 XDS110 USB-COM，波特率 115200 8N1（已在 SysConfig UART_LOG 实例锁定）。
 * 仅用于开发期日志，**不**用于运行期遥测（运行期遥测走蓝牙 UART2）。
 */

#ifndef BSP_LOG_UART_H
#define BSP_LOG_UART_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

void bsp_log_uart_init(void);

/**
 * @brief  阻塞写一段缓冲。
 *         printf 通过 retarget 的 fputc 间接走这条路径。
 */
void bsp_log_uart_write(const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* BSP_LOG_UART_H */
