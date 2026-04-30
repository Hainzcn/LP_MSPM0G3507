/**
 * @file    bsp_bt_uart.h
 * @brief   蓝牙串口 UART3 (HC-04, PB12 TX / PB13 RX, 115200 8N1)。
 *
 * - TX：阻塞写。VOFA+ JustFloat 帧典型 4 ch × 4 B + 4 B 尾 = 20 B，
 *   115200 bps 下 ≈ 1.74 ms/帧；100 Hz 帧率占空 17 %，主循环阻塞可接受。
 * - RX：开 RX 中断，ISR 内逐字节入 256 B 环形缓冲，应用层 `bt_rx_pop()`
 *   按需读取。本阶段不解析 RX 数据，仅作为后续接受 AT 回显 / 上位机控制
 *   位的扩展点。
 * - 环缓数据结构对应用层是黑盒：消费者不需要关心读写指针。
 *
 * 注：原阶段 0 文档预留 UART2 (PB17/PB16)，后因 SDK 2.10.00.04 在该
 *     multi-pad 引脚组上的 UART2 模板生成 bug 改用 UART3 + PB12/PB13。
 *     源代码层面所有 UART_BT_* 宏由 SysConfig 自动生成，应用代码不感知。
 */

#ifndef BSP_BT_UART_H
#define BSP_BT_UART_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void bsp_bt_uart_init(void);

/**
 * @brief  阻塞写一段缓冲到蓝牙 UART2。
 */
void bsp_bt_uart_write(const uint8_t *data, size_t len);

/**
 * @brief  从 RX 环缓中拉一字节。
 * @param  out  输出字节。
 * @return true = 拉到了；false = 缓冲空。
 */
bool bsp_bt_uart_rx_pop(uint8_t *out);

/**
 * @brief  返回当前 RX 环缓中可读字节数。
 */
size_t bsp_bt_uart_rx_available(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_BT_UART_H */
