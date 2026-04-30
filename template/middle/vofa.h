/**
 * @file    vofa.h
 * @brief   VOFA+ JustFloat 二进制协议打包：N 个 float (LE) + 4 字节帧尾。
 *
 * 帧尾 = 0x00 0x00 0x80 0x7F（小端表示的 +inf 的低字节序列变种，
 *       VOFA+ 文档约定的特殊 token，PC 端用作分帧边界）。
 *
 * 上位机：VOFA+ → 串口 → 选 JustFloat → 通道数 = N。
 *
 * 本文件不假定输出走哪条 UART：通过 `vofa_set_writer` 注入，方便切换
 * （阶段 1 注入蓝牙 UART2，未来可注入 K230 UART 或文件存储）。
 */

#ifndef VOFA_H
#define VOFA_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*vofa_writer_fn)(const uint8_t *data, size_t len);

/** 注册底层字节写函数（必须在调用 vofa_send 前完成）。 */
void vofa_set_writer(vofa_writer_fn writer);

/**
 * @brief  打包并发送 n 个 float 到上位机。
 * @param  ch   待发送通道数据指针（IEEE-754 32-bit 浮点小端）。
 * @param  n    通道数，必须 1~32。
 *
 *  注：MSPM0G3507 是 little-endian + IEEE-754 32-bit float，无需字节序调整。
 */
void vofa_send(const float *ch, uint8_t n);

#ifdef __cplusplus
}
#endif

#endif /* VOFA_H */
