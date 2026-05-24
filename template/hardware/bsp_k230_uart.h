/**
 * @file    bsp_k230_uart.h
 * @brief   K230 通讯 UART1（中断 RX + 阻塞 TX）。
 *
 * RX：UART1 RX FIFO 半满中断驱动，ISR 把字节写入 512 B 应用环形缓冲；
 *     上层 pop_bulk 取字节喂 k230_parser 状态机。
 * TX：阻塞逐字节写（bsp_k230_uart_write_blocking），满足 ~240 B/s 帧需求。
 * 帧协议层（k230_protocol.h）由上层 app_balance 调用，本 BSP 只负责字节搬运。
 */

#ifndef BSP_K230_UART_H
#define BSP_K230_UART_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  初始化 UART1 中断 RX 接收，开始持续填充环缓。
 */
void bsp_k230_uart_init(void);

/**
 * @brief  从 RX 应用环缓中批量拉取字节。
 * @param  dst     目标缓冲
 * @param  max_len 单次最多拉取字节数
 * @return 实际拉取字节数（0 = 缓冲空）
 *
 *  推荐主循环每 1~5 ms 调一次，喂给 k230_parser_feed。
 */
size_t bsp_k230_uart_rx_pop_bulk(uint8_t *dst, size_t max_len);

/** 返回当前 RX 应用环缓可读字节数。 */
size_t bsp_k230_uart_rx_available(void);

/** 累计环缓溢出丢弃次数。 */
uint32_t bsp_k230_uart_rx_overrun(void);

/**
 * @brief  返回从初始化至今 RX 中断累计写入字节总数。
 */
uint32_t bsp_k230_uart_total_rx(void);

/**
 * @brief  阻塞写一段缓冲到 UART1。
 */
void bsp_k230_uart_write_blocking(const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* BSP_K230_UART_H */
