/**
 * @file    bsp_k230_uart.h
 * @brief   K230 视觉处理器通讯 UART1 —— DMA RX 骨架（本阶段不解析帧）
 *
 * ============================================================
 * 这个文件是做什么的？（写给初学者）
 * ============================================================
 * 本项目中有两个主控芯片：
 *   1. MSPM0G3507（本文件所在的主控）——负责平衡控制、电机驱动
 *   2. K230（CanMV 开发板）——负责视觉处理（循迹、激光光斑识别）
 *
 * 两个芯片需要互相通信：
 *   MSPM0G3507 → K230：发送 IMU 数据（pitch 角、车速）和状态
 *   K230 → MSPM0G3507：发送运动指令（目标速度 v、角速度 ω）
 *
 * 这个文件就是两个芯片之间的"通讯管道"——UART1 串口。
 *
 * ============================================================
 * 为什么用 DMA 而不用中断？
 * ============================================================
 * K230 发给 MSPM0G3507 的数据是"不定长帧"，帧与帧之间的间隔不固定。
 * 如果用 RX FIFO 中断（像 bsp_imu_uart 那样），每收到一个字节就要进一次 ISR。
 * 如果数据量大，ISR 频率会很高，挤占 CPU 时间。
 *
 * DMA（Direct Memory Access，直接存储器访问）可以解决这个问题：
 *   1. CPU 配置好 DMA："把 UART 收到的数据自动搬到这个缓冲区"
 *   2. DMA 硬件自己完成搬运，不需要 CPU 参与
 *   3. 只有当整个缓冲区填满时，DMA 才触发一次中断
 *   4. CPU 只需要处理那一次中断
 *
 * 这样就把"每字节中断一次"降低为"每缓冲区中断一次"。
 *
 * ============================================================
 * 当前阶段只做什么？
 * ============================================================
 * 本阶段（阶段 1）只做两件事：
 *   1. 让 DMA 转起来——持续把 UART1 收到的数据存入环形缓冲区
 *   2. 用 1 Hz 日志打印"过去 1 秒收到了多少字节"
 *
 * 这样我们可以做物理回环测试（把 TX 和 RX 引脚短接），
 * 验证 DMA 链路是否正常工作。
 *
 * 帧协议解析、心跳超时降级等复杂功能留到下一阶段实现。
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
 * @brief 初始化 UART1 + DMA RX 循环模式。
 *
 * 这个函数配置 UART1 的 DMA 接收通道，让它以 BLOCK 模式运行。
 * 初始化后，DMA 会自动把 UART1 RX 收到的数据搬运到内部环形缓冲区，
 * 不需要 CPU 干预。
 *
 * 每装满一个缓冲区（512 字节），DMA 触发一次中断，
 * 在中断中重新装填 DMA（指向缓冲区开头），继续接收。
 *
 * @note  SysConfig 已经配置了 UART1 和 DMA 通道的硬件参数，
 *        本函数只做"让 DMA 转起来"的最后一步工作。
 */
void bsp_k230_uart_init(void);

/**
 * @brief 返回 DMA 累计写入缓冲区的字节总数。
 *
 * 这个值从 0 开始，每装满一次缓冲区（512 字节）就加 512。
 * 上限是 32 位无符号整数的最大值（约 4 GB），远超过实际需求。
 *
 * 用途：用相邻两次差值得出"过去 N 秒收到了多少字节"。
 * 例如：
 * @code
 *   uint32_t prev = bsp_k230_uart_total_rx();
 *   delay_ms(1000);
 *   uint32_t bytes_in_1s = bsp_k230_uart_total_rx() - prev;
 * @endcode
 *
 * @return 累计接收字节数（0 ~ 0xFFFFFFFF）
 */
uint32_t bsp_k230_uart_total_rx(void);

/**
 * @brief 从接收缓冲区中读取指定索引的字节（阶段 1 自测用）。
 *
 * 这个函数根据绝对索引 abs_index 计算出在环形缓冲区中的位置，
 * 返回该位置的字节值。
 *
 * 典型用法（打印最近接收到的 N 个字节）：
 * @code
 *   uint32_t total = bsp_k230_uart_total_rx();
 *   for (uint32_t i = total - N; i < total; i++) {
 *       printf("%02X ", bsp_k230_uart_peek(i));
 *   }
 * @endcode
 *
 * @param abs_index  绝对字节索引（从 0 开始递增的全局序号）
 * @return 该索引位置的一个字节
 */
uint8_t bsp_k230_uart_peek(uint32_t abs_index);

/**
 * @brief 阻塞式发送数据到 UART1（不走 DMA，仅自测用）。
 *
 * 这个函数通过轮询方式逐字节发送数据到 K230。
 * 不走 DMA 的原因是：本阶段 TX 没有大流量需求，
 * 轮询方式更简单，不需要额外配置 DMA TX 通道。
 *
 * 可用于回环测试：把 UART1 的 TX 和 RX 短接，
 * 发送数据后检查是否能收到自己发的内容。
 *
 * @param data  待发送数据指针
 * @param len   要发送的字节数
 */
void bsp_k230_uart_write_blocking(const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* BSP_K230_UART_H */
