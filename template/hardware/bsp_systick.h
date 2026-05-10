/**
 * @file    bsp_systick.h
 * @brief   1 kHz SysTick 系统节拍 —— 提供毫秒级延时和全局时间戳
 *
 * ============================================================
 * 这个文件是做什么的？（写给初学者）
 * ============================================================
 * 自平衡小车需要很精确的时间控制——比如"每 5 毫秒执行一次平衡控制计算"、
 * "等待 100 毫秒再启动"、"统计运行了多长时间"等。
 *
 * 要实现"精确计时"，需要有一个"心跳"（Heartbeat）——一个周期性触发的中断，
 * 每 1 毫秒跳一次，告诉 CPU："又过了 1 毫秒！"
 *
 * SysTick 就是 Cortex-M0+ 内核自带的"心跳发生器"。
 * 它不是芯片外设（如 TIMER/PWM），而是 CPU 内核的一部分——
 * 所以不需要配置复杂的时钟树，用起来非常简单。
 *
 * 这个模块把 SysTick 配置为 1 kHz（每秒 1000 次中断），
 * 并提供了三个核心功能：
 *   1. 获取当前毫秒时间戳（类似"现在几点"）
 *   2. 阻塞延时（"等待 N 毫秒"）
 *   3. 非阻塞 tick 消费（"检查是否又过了 1 毫秒"）
 *
 * ============================================================
 * 为什么要自己做 SysTick 驱动而不是用标准库？
 * ============================================================
 * TI 的 DriverLib 没有封装 SysTick（SysTick 是 CPU 内核外设，不是芯片外设）。
 * CMSIS 标准提供了 SysTick_Config() 函数来配置 SysTick，
 * 但上层的时间管理（毫秒计数、延时、超时检测）需要自己实现。
 * 这个文件就是"在 CMSIS SysTick 基础上搭建的时间服务层"。
 */

#ifndef BSP_SYSTICK_H                /* 头文件保护宏 */
#define BSP_SYSTICK_H

#include <stdint.h>                  /* 引入 uint32_t、int32_t 等定宽类型 */
#include <stdbool.h>                 /* 引入 bool / true / false */

#ifdef __cplusplus                   /* 兼容 C++ 编译器 */
extern "C" {
#endif

/**
 * @brief  初始化 1 kHz SysTick 定时中断
 *
 * 这个函数配置 CPU 内核的 SysTick 定时器，让它以指定的频率触发中断。
 * 本工程固定传入 1000 Hz（每 1 毫秒中断一次）。
 *
 * 工作原理（简单版）：
 *   SysTick 是一个 24 位的"倒数计数器"。
 *   给它一个"初始值"（reload value），然后每个 CPU 时钟周期减 1，
 *   减到 0 时触发一次中断，然后自动重装初始值，继续倒数。
 *
 *   初始值的计算公式：
 *     reload = CPU 时钟频率 ÷ 目标中断频率
 *   例如 CPU = 32 MHz，目标 = 1 kHz：
 *     reload = 32,000,000 ÷ 1000 = 32,000
 *   意味着 SysTick 每数 32000 个数就触发一次中断——正好 1 毫秒。
 *
 * 函数内部的额外操作：
 *   除了调用 SysTick_Config() 外，还把 SysTick 的中断优先级设为最高（0）。
 *   这是为了防止业务中断（如 GPIO、UART）饿死 SysTick——详见 .c 文件注释。
 *
 * @param hz  期望的中断频率（单位 Hz）
 *            本工程固定传入 1000（1 kHz）
 *            如果传 0，函数返回 -1 表示失败
 *
 * @return 0   = 初始化成功
 *         -1  = hz 为 0（无效参数）
 *         其他正数 = SysTick_Config() 返回的错误（可能性很低）
 *
 * 调用方式（在 main 函数初始化阶段调用一次）：
 * @code
 *   int rc = bsp_systick_init(1000);
 *   if (rc != 0) {
 *       // 初始化失败的处理（通常不会发生）
 *   }
 * @endcode
 */
int32_t bsp_systick_init(uint32_t hz);

/**
 * @brief  获取从系统启动至今的毫秒数（全局时间戳）
 *
 * 这个函数返回的是一个不断自增的 32 位无符号整数，
 * 表示从上电到现在经过了多少毫秒。
 *
 * 用途：
 *   1. 计算时间差：记录两个时间点，相减得到间隔
 *   2. 超时检测：当前时间 - 开始时间 > 超时值
 *   3. 定时执行：每隔 N 毫秒执行一次任务
 *
 * 使用示例：
 * @code
 *   uint32_t start = bsp_systick_get_ms();
 *   do_something();
 *   uint32_t elapsed = bsp_systick_get_ms() - start;
 *   // elapsed 就是 do_something() 的执行耗时（毫秒）
 * @endcode
 *
 * 注意事项：
 *   - 返回值是 32 位无符号整数，约 49.7 天溢出回 0
 *   - 计算时间差时，即使溢出也能正确计算（无符号整数减法自动处理回绕）
 *     例：假设 start=0xFFFFFFF0, now=0x00000010
 *         now - start = 0x10 - 0xFFFFFFF0 = 32（正确！利用了整数溢出）
 *   - 精度为 1 毫秒（因为 SysTick 是 1 kHz），不能用于更高精度的测量
 *
 * @return 上电至今的毫秒数（0 ~ 0xFFFFFFFF，约 49.7 天一个循环）
 */
uint32_t bsp_systick_get_ms(void);

/**
 * @brief  阻塞延时（"等待 N 毫秒"）
 *
 * 功能：让程序在此停留指定的毫秒数，然后再继续执行后续代码。
 *
 * 内部实现原理（"忙等待 + 睡眠"模式）：
 *   1. 计算目标时间 = 当前时间 + 要等待的毫秒数
 *   2. 在未达到目标时间之前，不断调用 __WFI() 指令
 *   3. __WFI() 让 CPU 进入睡眠模式，直到下一个中断唤醒
 *   4. 每次 SysTick 中断唤醒后就检查是否超时
 *
 * __WFI()（Wait For Interrupt）：
 *   这是 ARM Cortex-M 内核的低功耗指令。
 *   执行这条指令后，CPU 暂停执行，进入睡眠状态，
 *   直到有中断发生才被唤醒。
 *   这样做的好处是：延时期间 CPU 不空转消耗电能。
 *   但注意：这不是深度睡眠，唤醒非常快（几个时钟周期）。
 *
 * 使用示例：
 * @code
 *   DL_GPIO_setPins(GPIOB, BSP_BUZZER_PIN);   // 蜂鸣器响
 *   bsp_systick_delay_ms(100);                 // 持续 100 ms
 *   DL_GPIO_clearPins(GPIOB, BSP_BUZZER_PIN);  // 蜂鸣器停
 * @endcode
 *
 * 注意事项：
 *   - 这是"阻塞"的——延时期间 CPU 不能做其他事情
 *   - 对实时性要求高的场景，请使用非阻塞的 tick 消费模式
 *   - 延时精度受 SysTick 中断影响，±1 ms 误差
 *   - 延时过程中其他中断仍然可以响应（不会被阻塞）
 *
 * @param ms  要等待的毫秒数
 *            传 0 时函数立即返回（不等待）
 */
void bsp_systick_delay_ms(uint32_t ms);

/**
 * @brief  非阻塞式检查是否又过了一毫秒（"消耗一个 tick"）
 *
 * 功能：检查自从上次调用这个函数以来，是否又发生了一次 SysTick 中断。
 *       如果是，返回 true 并清除标志；否则返回 false。
 *
 * 这是自平衡小车主循环中最常用的"定时"方式——
 * 不需要阻塞等待，主循环可以持续做其他事情。
 *
 * 使用示例（典型的主循环骨架）：
 * @code
 *   for (;;) {
 *       // 检查 1 kHz tick，每毫秒执行一次
 *       if (bsp_systick_consume_tick()) {
 *           tick_count++;
 *
 *           // 每 1 ms：喂数据给姿态解析器
 *           ms901m_feed_bytes(buf, n);
 *
 *           // 每 5 ms：执行平衡控制
 *           if (tick_count % 5 == 0) {
 *               float u = pid_step(&pid, 0, snap.pitch_deg, 0.005f);
 *               motor_set_output(u);
 *           }
 *
 *           // 每 50 ms：发送 VOFA+ 数据
 *           if (tick_count % 50 == 0) {
 *               vofa_send(channels, 3);
 *           }
 *
 *           // 每 1000 ms（1 秒）：输出调试日志
 *           if (tick_count % 1000 == 0) {
 *               printf("pitch=%.1f\n", snap.pitch_deg);
 *           }
 *       }
 *   }
 * @endcode
 *
 * 原子性保证：
 *   这个函数内部使用 PRIMASK（优先级掩码）短暂关闭全局中断，
 *   然后读取并清除 s_tick_pending 标志，再重新开启中断。
 *   这样避免了和 SysTick_Handler（ISR）同时访问 s_tick_pending 的竞争条件。
 *
 * @return true  = 从上一次调用到现在，至少发生过一次 SysTick 中断
 *                 （即又过了一毫秒），标志已被清零
 *         false = 还没到下一个 1 毫秒
 */
bool bsp_systick_consume_tick(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_SYSTICK_H */
