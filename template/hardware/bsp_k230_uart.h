/**
 * @file    bsp_k230_uart.h
 * @brief   K230 通讯 UART1（DMA RX 半缓冲双中断 + 阻塞 TX）。
 *
 * Stage 4 重构（IMU TX 一分二方案）：
 *   - RX：DMA 乒乓半缓冲（半满 + 全满双中断），每半块触发 DMA ISR，
 *         ISR 把就绪半块写入 512 B 应用环形缓冲；上层 pop_bulk 取字节
 *         喂 k230_parser 状态机。无丢字节窗口（DMA 在 ISR 处理时已切到另一半）。
 *   - TX：阻塞逐字节写（已有 bsp_k230_uart_write_blocking），满足
 *         ~240 B/s 低频帧需求。TX DMA 已移除。
 *   - 帧协议层（k230_protocol.h）由上层 app_balance 调用，本 BSP 只负责
 *     字节搬运，不感知帧边界。
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
 * @brief  初始化 UART1 + DMA RX 半缓冲接收，开始持续填充。
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

/** 累计 DMA→应用环缓 溢出丢弃字节次数。 */
uint32_t bsp_k230_uart_rx_overrun(void);

/**
 * @brief  返回从初始化至今 DMA 累计写入字节总数。
 */
uint32_t bsp_k230_uart_total_rx(void);

/**
 * @brief  阻塞写一段缓冲到 UART1。
 *         Stage 4 起 MCU→K230 全部走此函数（速度/状态/心跳帧）。
 */
void bsp_k230_uart_write_blocking(const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* BSP_K230_UART_H */
