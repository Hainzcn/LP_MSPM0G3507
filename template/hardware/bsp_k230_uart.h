/**
 * @file    bsp_k230_uart.h
 * @brief   K230 通讯 UART1 接收骨架（DMA + 环形缓冲，本阶段不解析帧）。
 *
 * 阶段 1 范围 C：把链路硬件部分（DMA RX + 环形缓冲）搭通，1 Hz 在日志里
 * 打印过去 1 s 收到的字节数，便于物理回环（PB6 短接 PB7）测试。
 *
 * 帧协议（0xAA55 起始 / CRC16 / 0x55AA 结束）的解析、IMU_TELEM /
 * MOTION_CMD / HEARTBEAT / ERROR 命令、心跳超时降级 留到下一阶段。
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
 * @brief  初始化 UART1 + DMA RX 循环模式，开始持续填充接收缓冲。
 */
void bsp_k230_uart_init(void);

/**
 * @brief  返回从初始化至今 DMA 累计写入字节总数（32-bit，约 4 GB 后溢出）。
 *         应用层用相邻两次差值得"过去 N 秒收到多少字节"。
 */
uint32_t bsp_k230_uart_total_rx(void);

/**
 * @brief  仅给阶段 1 自测用：读出 RX 缓冲中第 idx 个字节（idx ∈ [0,len)）。
 *         用 (total_rx - len) 作为基准，可以打印最近 N 字节的快照。
 */
uint8_t bsp_k230_uart_peek(uint32_t abs_index);

/**
 * @brief  阻塞写一段缓冲到 UART1（不走 DMA，仅自测用）。
 *         本阶段不发任何业务数据，留接口以备 loopback 自激测试。
 */
void bsp_k230_uart_write_blocking(const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* BSP_K230_UART_H */
