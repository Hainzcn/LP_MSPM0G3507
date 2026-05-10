/**
 * @file    vofa.h
 * @brief   VOFA+ JustFloat 协议打包器 —— 把 float 数据发到电脑上"画波形"
 *
 * ============================================================
 * 这个文件是做什么的？（写给初学者）
 * ============================================================
 * 你在调试自平衡小车的时候，肯定想知道：
 *   - 当前的 pitch 角是多少？
 *   - 电机输出了多大的力？
 *   - 这些数据随时间怎么变化？
 *
 * 你当然可以用串口把数据打印出来：
 *   printf("pitch=%.2f, output=%.1f\n", pitch, output);
 * 但一堆滚动的数字很难看出趋势。
 *
 * VOFA+ 就是一个"数据可视化工具"——它运行在你的电脑上，
 * 通过串口接收单片机发来的数据，然后实时绘制成波形曲线。
 * 就像医院的心电图仪一样，让你一眼看出数据的变化趋势。
 *
 * 这个文件（vofa.h/.c）就是"VOFA+ 协议的打包器"——
 * 它把你的 float 数据，按照 VOFA+ 能识别的格式打包好，
 * 然后通过串口发送到电脑上。
 *
 * ============================================================
 * VOFA+ JustFloat 协议原理
 * ============================================================
 * 协议格式非常简单：
 *
 *   [float1] [float2] ... [floatN] [帧尾标记]
 *     ↑        ↑           ↑         ↑
 *   第1通道   第2通道     第N通道    4字节特殊标记
 *
 * 每个 float 占 4 字节（小端 IEEE 754 格式）。
 * 帧尾是固定的 4 字节：0x00 0x00 0x80 0x7F
 *
 * 帧尾的秘密：
 *   这 4 个字节其实是 IEEE 754 浮点数 "+infinity"（正无穷）的
 *   小端字节序表示。+infinity 的二进制是 0x7F800000，
 *   拆成小端字节就是 [0x00, 0x00, 0x80, 0x7F]。
 *
 *   为什么用 +inf 做帧尾？
 *   因为正常的 float 数据不可能等于正无穷，
 *   所以当 VOFA+ 收到这四个字节时，就知道"一帧数据结束了"。
 *   这叫做"带外标识"（sentinel value）——用一个不可能出现的值
 *   来标记边界。
 *
 * ============================================================
 * 本模块设计特点
 * ============================================================
 * 1. 通过"函数注入"方式解耦底层硬件
 *     不直接调用 UART 发送函数，而是通过 vofa_set_writer()
 *     注册一个"写函数"。这样：
 *       - 调试阶段：注入蓝牙 UART → 无线看波形
 *       - 正式阶段：注入不同 UART，甚至写入文件
 *       - 不改 vofa.c 任何代码
 *
 * 2. 单缓冲一次发送
 *     把所有的 float + 帧尾拼成一个连续的大缓冲区，
 *    然后一次调用 writer 发出去。减少 UART 中断次数。
 */

#ifndef VOFA_H                      /* 头文件保护宏 */
#define VOFA_H

#include <stdint.h>                 /* 引入 uint8_t 等定宽类型 */
#include <stddef.h>                 /* 引入 size_t */

#ifdef __cplusplus                   /* 兼容 C++ 编译器 */
extern "C" {
#endif

/**
 * @brief  底层字节写函数的类型定义（函数指针）
 *
 * 这是一个"函数指针类型"的 typedef 定义。
 * 它定义了一类函数的"签名"（signature）——即参数类型和返回值类型。
 *
 * 语法解读：
 *   typedef void (*vofa_writer_fn)(const uint8_t *data, size_t len);
 *      ↑        ↑         ↑              ↑                  ↑
 *   类型定义  返回值   函数指针名       参数1：数据指针    参数2：数据长度
 *
 * 符合这个签名的函数，可以用来注册为 VOFA+ 的底层写函数。
 * 例如下面的两个函数都符合：
 * @code
 *   void uart_send_bytes(const uint8_t *data, size_t len) {
 *       for (size_t i = 0; i < len; i++) {
 *           while (!UART_IS_TRANSMIT_READY());  // 等待发送就绪
 *           UART_SEND_BYTE(data[i]);             // 逐字节发送
 *       }
 *   }
 *
 *   void bluetooth_send_bytes(const uint8_t *data, size_t len) {
 *       // 蓝牙串口发送实现
 *   }
 * @endcode
 *
 * 两者都满足 vofa_writer_fn 类型，都可以注册为 writer。
 *
 * 函数指针的好处：
 *   1. 解耦——vofa.c 不用知道具体用的是哪个 UART
 *   2. 灵活——运行时可以切换 writer（调试用蓝牙，正式用有线）
 *   3. 可测试——可以注入"把数据写到内存"的假 writer
 */
typedef void (*vofa_writer_fn)(const uint8_t *data, size_t len);

/**
 * @brief  注册底层字节写函数（必须在调用 vofa_send 前调用）
 *
 * 这个函数告诉 VOFA+ 模块："当你需要发送数据时，调用这个函数。"
 *
 * 调用方式：
 * @code
 *   // 方式 1：注入 UART 发送函数
 *   vofa_set_writer(uart_send_bytes);
 *
 *   // 方式 2：注入蓝牙发送函数
 *   vofa_set_writer(bluetooth_send_bytes);
 *
 *   // 方式 3：注入"调试用"的假发送函数（把数据存到数组里）
 *   vofa_set_writer(debug_writer);
 * @endcode
 *
 * 为什么需要"注册"这一步？
 *   因为 vofa.c 是一个"通用"模块，它不知道项目里用的是哪个 UART。
 *   通过注入的方式，vofa.c 和具体的 UART 驱动解耦。
 *   想换 UART 时，只需要换一个注入函数，vofa.c 不用改。
 *
 * @param writer  指向底层字节写函数的函数指针
 *                可以传 NULL 来"取消注册"——此时 vofa_send() 会静默返回
 *
 * @note  这个函数必须在第一次调用 vofa_send() 之前调用。
 *        如果忘记注册，vofa_send() 会检测到 s_writer == NULL，
 *        然后直接返回，不发送任何数据。
 */
void vofa_set_writer(vofa_writer_fn writer);

/**
 * @brief  打包并发送 N 个 float 数据到 VOFA+ 上位机（★核心函数）
 *
 * 这是本模块的核心函数——它把 N 个 float 数据按照 JustFloat 协议
 * 打包成二进制流，然后通过注册的 writer 发送出去。
 *
 * 打包格式（JustFloat 协议）：
 *   通道数 = 3 时，发送的内容如下：
 *
 *   偏移 0:   ch[0] 的 4 字节（float1, 小端）
 *   偏移 4:   ch[1] 的 4 字节（float2, 小端）
 *   偏移 8:   ch[2] 的 4 字节（float3, 小端）
 *   偏移 12:  帧尾 4 字节（0x00 0x00 0x80 0x7F）
 *
 *   总共发送：3 × 4 + 4 = 16 字节
 *
 * 调用方式（典型的主循环代码）：
 * @code
 *   // 在主循环中，每 50 ms 调用一次
 *   float channels[] = {
 *       snap.pitch_deg,    // 通道 0：俯仰角
 *       snap.gy_dps,       // 通道 1：俯仰角速度
 *       motor_output       // 通道 2：电机输出
 *   };
 *   vofa_send(channels, 3);
 *
 *   // 电脑上的 VOFA+ 会显示 3 条实时波形曲线
 * @endcode
 *
 * 通道数与 VOFA+ 的对应关系：
 *   如果 n=3，VOFA+ 上会出现 Channel 0 / Channel 1 / Channel 2 三条曲线。
 *   VOFA+ 会根据收到的 float 数量自动推断通道数，不需要额外配置。
 *
 * 限制：
 *   - n 最大为 32（VOFA_MAX_CHANNELS），防止缓冲区溢出
 *   - n 最小为 1（只发一个通道也行）
 *   - 数据量：n=3 → 16 字节。即使 n=32 → 132 字节，一次 UART 发送即可
 *
 * 安全性：
 *   - 如果 s_writer 为 NULL（忘记注册），函数静默返回
 *   - 如果 ch 指针为 NULL，函数静默返回
 *   - 如果 n 为 0 或 > 32，函数静默返回
 *   - 内部使用栈上缓冲区，不动态分配内存（malloc），适合单片机环境
 *
 * @param ch  待发送的浮点数据数组指针
 *            数组中的每个 float 对应 VOFA+ 上的一个通道
 *            注意：数据长度 = n × sizeof(float) = n × 4 字节
 *
 * @param n   通道数（= 要发送的 float 个数）
 *            范围：1 ~ 32（超过 32 会被截断——直接 return）
 *
 * @note  MSPM0G3507 是小端（Little-Endian）+ IEEE 754 32-bit float，
 *        所以 float 的内存表示直接符合 JustFloat 协议要求。
 *        如果换到大端单片机（如某些 ARM 芯片的大端模式），
 *        需要在发送前做字节序转换。
 */
void vofa_send(const float *ch, uint8_t n);

#ifdef __cplusplus
}
#endif

#endif /* VOFA_H */
