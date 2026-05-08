/**
 * @file    bsp_imu_uart.h
 * @brief   ATK-MS901M 姿态传感器串口 UART2 (PA21 TX / PA22 RX, 115200 8N1)。
 *
 * Stage 1.5 替代原 I²C MPU6050 链路（详见 docs/TaskLog/Stage1.5-IMU-Swap-MS901M.md）。
 *
 * - RX：开 RX FIFO 半满 + 超时中断，ISR 内逐字节排入 256 B 环形缓冲，
 *       上层 `pop_bulk` 一次拉走全部待解析字节给 ms901m 状态机。
 * - TX：阻塞写。仅供将来发 ATK 配置/校准命令；MS901M 默认上电即主动上报，
 *       本阶段业务上 TX 不动。
 *
 * 容量估算：MS901M 出厂上报 5 帧 / 200 Hz、单帧最大 ~15 B、≈ 15 kB/s；
 * 1 kHz 主循环每拍 drain ≈ 15 B，远低于 256 B 环缓 → 无溢出风险。
 *
 * 与 bsp_bt_uart 结构镜像（同样 SysConfig 实例 + 单 RX 中断 + 环形缓冲），
 * 二者代码改动相互独立——这里只动 UART2，不影响蓝牙链路。
 */

#ifndef BSP_IMU_UART_H
#define BSP_IMU_UART_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  初始化 IMU UART2 接收。SysConfig 已配好引脚 / FIFO / 波特率 / 中断，
 *         本函数仅清挂起标志并复位环缓游标。
 */
void bsp_imu_uart_init(void);

/**
 * @brief  阻塞写一段缓冲到 IMU UART2（仅供发 MS901M 配置命令使用）。
 */
void bsp_imu_uart_write(const uint8_t *data, size_t len);

/**
 * @brief  从 RX 环缓中拉一字节。
 * @param  out 输出字节。
 * @return true = 拉到了；false = 缓冲空。
 */
bool bsp_imu_uart_rx_pop(uint8_t *out);

/**
 * @brief  批量拉取 RX 环缓字节，复制到 dst。
 * @param  dst 目标缓冲，至少 max_len 字节。
 * @param  max_len 单次最多拉取字节数。
 * @return 实际拉取字节数（0 = 缓冲空）。
 *
 *  推荐主循环每拍调一次 max_len = 64~128，把字节一次性喂给 ms901m_feed_bytes。
 */
size_t bsp_imu_uart_rx_pop_bulk(uint8_t *dst, size_t max_len);

/** 返回当前 RX 环缓中可读字节数。 */
size_t bsp_imu_uart_rx_available(void);

/** 累计 RX 溢出（环缓写满丢弃）次数，1 Hz 自测可读取诊断。 */
uint32_t bsp_imu_uart_rx_overrun(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_IMU_UART_H */
