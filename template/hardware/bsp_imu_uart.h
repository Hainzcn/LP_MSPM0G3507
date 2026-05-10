/**
 * @file    bsp_imu_uart.h
 * @brief   ATK-MS901M 姿态传感器串口驱动 —— UART3 (PB12 TX / PB13 RX, 115200 8N1)
 *
 * ============================================================
 * 这个文件是做什么的？（写给初学者）
 * ============================================================
 * 在 ms901m.c 的学习笔记中我们说过，MS901M 姿态传感器通过 UART 串口
 * 不断往外发送姿态数据（pitch、roll、yaw 等）。但是，UART 收到的每个字节
 * 是"一个一个"到来的，而且到来的时刻不确定。
 *
 * 如果每次收到一个字节就马上处理（调用 ms901m_feed_bytes），
 * 那就会在中断服务函数（ISR）中做大量运算，占用 CPU 时间。
 *
 * 更好的做法是：
 *   1. ISR（中断）只做一件事：把收到的字节存到一个"缓冲区"里
 *   2. 主循环定期从这个缓冲区里批量取出字节，交给解析器
 *
 * 这个缓冲区叫"环形缓冲区"（Ring Buffer），也就是本模块的核心。
 *
 * 本模块做的事情可以概括为：
 *   1. 初始化 UART3 硬件 + 中断
 *   2. ISR 把收到的字节放入环形缓冲区
 *   3. 主循环通过 pop / pop_bulk 从缓冲区取走数据
 *
 * 典型数据流：
 *   MS901M → UART3 RX 引脚 → ISR 逐字节入队 → 环形缓冲区(256B) →
 *   主循环 pop_bulk → ms901m_feed_bytes() → 解析出 pitch/rate
 *
 * ============================================================
 * 为什么是 UART3？为什么从 UART2 搬过来？
 * ============================================================
 * Stage 1.5：MS901M 替代 MPU6050，使用 UART2（PA21/PA22）
 * Stage 1.6：发现 PA21 未引出到 BoosterPack 排针，需要焊接
 *            → 迁到 UART3（PB12/PB13），都是开放排针，免焊接
 *            代价：原占 UART3 的蓝牙模块下线
 *
 * 这个调整告诉我们：嵌入式硬件中"引脚是否容易接线"很重要！
 */

#ifndef BSP_IMU_UART_H
#define BSP_IMU_UART_H

#include <stdint.h>     /* 引入 uint8_t、uint32_t 等定宽类型 */
#include <stddef.h>     /* 引入 size_t */
#include <stdbool.h>    /* 引入 bool / true / false */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 IMU UART3 接收。
 *
 * 这个函数做的事情很少——因为 UART 的硬件配置（引脚、波特率、FIFO 模式）
 * 已经由 SysConfig 在 SYSCFG_DL_init() 中完成了。
 *
 * 它主要做三件事：
 *   1. 清除残留的中断标志（防止误触发）
 *   2. 使能 NVIC 中的 UART3 中断（⚠️ SysConfig 不会做这一步！）
 *   3. 复位环形缓冲区的读写指针
 *
 * 调用方式（在 main 初始化阶段）：
 * @code
 *   SYSCFG_DL_init();
 *   bsp_gpio_init();
 *   bsp_systick_init(1000);
 *   bsp_imu_uart_init();   // 打开 IMU 串口接收
 * @endcode
 */
void bsp_imu_uart_init(void);

/**
 * @brief 阻塞式发送数据到 IMU UART（供发送 MS901M 配置命令）。
 *
 * MS901M 上电后默认主动上报数据，不需要单片机发任何指令。
 * 但如果我们想改变 MS901M 的配置（比如改变上报频率、量程等），
 * 就需要通过这个函数发送命令帧。
 *
 * 实现方式和 bsp_log_uart_write() 相同——逐字节阻塞发送。
 *
 * @param data  待发送数据指针
 * @param len   要发送的字节数
 */
void bsp_imu_uart_write(const uint8_t *data, size_t len);

/**
 * @brief 从 RX 环形缓冲区读取一个字节（单字节弹出）。
 *
 * 检查环形缓冲区是否为空。如果不为空，取出一个字节。
 * 这个函数是"非阻塞"的——如果没有数据，立即返回 false。
 *
 * @param out  输出参数：读到字节后写入此变量
 * @return true = 成功读到字节；false = 缓冲区空
 */
bool bsp_imu_uart_rx_pop(uint8_t *out);

/**
 * @brief 批量从 RX 环形缓冲区读取多个字节（⭐ 推荐使用）。
 *
 * 这是主循环中最常用的函数。一次性从环形缓冲区取出尽可能多的字节
 * （最多 max_len 个），直接喂给 ms901m_feed_bytes()。
 *
 * 为什么批量比单字节好？
 *   批量读取是一次函数调用取出多个字节。
 *   单字节读取是多次函数调用，每次取出一个字节——开销更大。
 *
 * 典型用法（在主循环中每 1 ms 调用）：
 * @code
 *   uint8_t buf[64];
 *   size_t n = bsp_imu_uart_rx_pop_bulk(buf, sizeof(buf));
 *   if (n > 0) {
 *       ms901m_feed_bytes(buf, n);
 *   }
 * @endcode
 *
 * @param dst     目标缓冲区，至少 max_len 字节
 * @param max_len 本次最多取多少个字节（建议 64~128）
 * @return 实际取出的字节数（0 = 缓冲区空）
 */
size_t bsp_imu_uart_rx_pop_bulk(uint8_t *dst, size_t max_len);

/**
 * @brief 返回当前 RX 环形缓冲区中可读的字节数。
 *
 * 用于判断缓冲区中是否有数据等待处理。
 * 如果返回值很大（接近 256），说明主循环处理速度跟不上 ISR 接收速度。
 */
size_t bsp_imu_uart_rx_available(void);

/**
 * @brief 返回累计的缓冲区溢出次数。
 *
 * 当 ISR 往缓冲区写数据时，如果缓冲区已满（写指针追上了读指针），
 * 新到的字节会被丢弃，s_rx_overrun 加 1。
 *
 * 如果这个值持续增长，说明：
 *   1. 主循环处理得太慢（控制周期太长）
 *   2. 缓冲区太小（256 字节对 MS901M 的 15 kB/s 来说完全够用）
 *   3. 中断没有正常工作
 *
 * 建议在调试日志中每 1 秒输出一次这个值。
 */
uint32_t bsp_imu_uart_rx_overrun(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_IMU_UART_H */
