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
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void bsp_log_uart_init(void);

/**
 * @brief  阻塞写一段缓冲。
 *         printf 通过 retarget 的 fputc 间接走这条路径。
 */
void bsp_log_uart_write(const uint8_t *data, size_t len);

/**
 * @brief 尝试非阻塞写一段缓冲。
 * @return true = 整段数据已进入 TX 队列；false = 队列空间不足，数据未写入。
 */
bool bsp_log_uart_try_write_async(const uint8_t *data, size_t len);

/**
 * @brief 非阻塞读取 XDS-UART RX 单字节。
 * @return true = 读到 1 字节；false = RX FIFO 为空。
 */
bool bsp_log_uart_read_byte(uint8_t *out);

/** @return XDS-UART RX 环形缓冲溢出次数；非 0 表示 PC 命令发送过快或主循环未及时消费。 */
uint32_t bsp_log_uart_rx_overrun(void);

/** @return XDS-UART 非阻塞 TX 入队失败次数；非 0 表示高频报文超过串口带宽。 */
uint32_t bsp_log_uart_tx_drop_count(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_LOG_UART_H */
