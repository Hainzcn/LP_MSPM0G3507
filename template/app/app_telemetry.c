/**
 * @file    app_telemetry.c
 * @brief   阶段 1 遥测调度实现 —— 最小的能跑主循环
 *
 * ============================================================
 * 这个文件是做什么的？（写给初学者）
 * ============================================================
 * 这是本项目中"第一个能跑的主循环"——在平衡控制还没写好的时候，
 * 这个文件让单片机先跑起来，验证硬件（IMU、串口、LED）是否正常。
 *
 * 它做的事情非常少：
 *   1 kHz：从 UART3 环形缓冲区取字节 → 喂给 MS901M 解析器 → 取快照
 *   5 Hz：翻转绿灯（看到灯闪 = 代码在跑）
 *   1 Hz：printf 打印 pitch/roll/gyro/temp 到电脑串口
 *
 * 这是嵌入式开发的"Hello World"——
 * 不是 printf("hello world")，而是"传感器数据在串口上滚动了"。
 *
 * 等阶段 2 开发平衡控制时，这个文件会被 app_balance.c 替代。
 * 但它的结构（tick_count 周期调度 + __WFI 睡眠）被 app_balance_run()
 * 完整继承。
 *
 * ============================================================
 * 定点格式化宏（与 app_balance.c 共用相同的设计）
 * ============================================================
 * 为什么不用 printf("%f") 打印浮点数？
 *   因为 Keil AC6 的 printf("%f") 会拉入几 KB 的浮点格式化代码，
 *   而且每次调用消耗 200~300 字节的栈空间。
 *   MSPM0G3507 的栈只有 1~4 KB，打几个 %f 就溢出了 → HardFault！
 *
 * 我们的方案：
 *   把浮点数 ×100 变成定点整数，用 printf 的整数格式 %ld 打印。
 *   例如 pitch=2.35° → 拆成 符号' '+整数2+小数点+小数35 → " 2.35"
 *
 * 四个宏的分工：
 *   F2_X100(v)：v × 100 四舍五入 → int32
 *   F2_S(v)   ：取符号字符 ' ' 或 '-'
 *   F2_I(v)   ：取整数部分（绝对值 / 100）
 *   F2_F(v)   ：取小数部分（绝对值 % 100），两位补零
 *
 * 副作用注意：
 *   这些宏会多次求值参数 v（最多 6 次）。
 *   所以只能传"纯值"（如 snap.pitch_deg），不能传带副作用的表达式
 *   （如 ++i 或函数调用返回值）。
 */

#include "app_telemetry.h"
#include "ti_msp_dl_config.h"   /* DL_GPIO_xxx 函数 */

#include "bsp_gpio.h"           /* BSP_LED_G_PORT/PIN */
#include "bsp_systick.h"        /* consume_tick */
#include "bsp_k230_uart.h"      /* K230 接收字节统计 */
#include "bsp_imu_uart.h"       /* IMU UART 环形缓冲区读取 */
#include "ms901m.h"             /* MS901M 姿态解析 */
#include <stdio.h>              /* printf（但不用 %f！） */

/* ================================================================
 * 调度相位（多少个 1 ms tick 执行一次）
 * ================================================================ */
#define PHASE_LED_TICKS     200u   /* 200 ms → 5 Hz */
#define PHASE_LOG_TICKS     1000u  /* 1000 ms → 1 Hz */

/* 1 kHz 任务中，单拍最多从 UART3 环形缓冲区取多少字节。
 * MS901M 默认 5 帧 × 200 Hz × ~15 B/帧 = 15 kB/s = 15 B/ms。
 * 64 字节 = 4× 裕度，即使偶尔积压也能一次清空。 */
#define IMU_DRAIN_CHUNK     64u

/* ================================================================
 * 浮点字段格式化辅助宏（避免 printf("%f")）
 * ================================================================
 *
 * 示例：pitch = 2.35° → " 2.35"
 *   F2_X100(2.35)  = 235（乘 100 + 四舍五入）
 *   F2_S(2.35)     = ' '（正数用空格对齐，负数用 '-'）
 *   F2_I(2.35)     = 2（整数部分，去符号后 / 100）
 *   F2_F(2.35)     = 35（小数部分，去符号后 % 100）
 *
 * 示例：pitch = -2.35° → "-2.35"
 *   F2_X100(-2.35) = -235
 *   F2_S(-2.35)    = '-'
 *   F2_I(-2.35)    = 2
 *   F2_F(-2.35)    = 35
 *
 * printf 格式串："%c%ld.%02lu" → 符号 + 整数 + "." + 两位小数
 *
 * 关键正确性：99.995 → 10000 → " 100.00"（而不是 " 99.100"）
 *   因为先整体 ×100 四舍五入，再拆整数/小数。如果先拆再四舍五入就会出错。
 */

/** 浮点数 v × 100 并四舍五入为带符号的 int32 */
#define F2_X100(v)  ((int32_t)((v) * 100.0f + ((v) >= 0.0f ? 0.5f : -0.5f)))

/** 取符号字符：正数返回空格 ' '（对齐），负数返回 '-' */
#define F2_S(v)     (F2_X100(v) < 0 ? '-' : ' ')

/** 取整数部分：绝对值 / 100 */
#define F2_I(v)     ((int32_t)((F2_X100(v) < 0 ? -F2_X100(v) : F2_X100(v)) / 100))

/** 取小数部分（两位）：绝对值 % 100 */
#define F2_F(v)     ((uint32_t)((F2_X100(v) < 0 ? -F2_X100(v) : F2_X100(v)) % 100))

/* ================================================================
 * app_telemetry_run() —— 阶段 1 主循环入口（永不返回）
 * ================================================================ */

void app_telemetry_run(void)
{
    /* tick_count：SysTick 计数器，每 1 ms + 1。
     * 用它做取模运算实现周期调度：tick_count % N == 0 */
    uint32_t tick_count    = 0u;

    /* last_total_rx：上一次 1 Hz 时的 K230 接收字节总数。
     * 用于计算"过去 1 秒收到了多少字节"（delta = now - last） */
    uint32_t last_total_rx = bsp_k230_uart_total_rx();

    /* snap：MS901M 姿态快照（在 1 kHz 路径中被更新） */
    ms901m_snapshot_t snap = { 0 };

    /* ██ 主循环 ██ 永不退出 */
    for (;;) {

        /* ---- 等待 1 ms SysTick 心跳 ---- */
        /* 如果当前 tick 还没到（SysTick 中断还没触发），
         * 用 __WFI() 让 CPU 进入睡眠模式。
         * 被任意中断唤醒后检查是否需要处理。 */
        if (!bsp_systick_consume_tick()) {
            __WFI();
            continue;
        }
        tick_count++;

        /* ============================================================
         * 1 kHz 任务：读取 MS901M 数据
         * ============================================================
         * 每 1 ms 从 UART3 环形缓冲区取一批字节，喂给 MS901M 解析器。
         *
         * UART RX 半满中断已经把字节排入了 256 字节的环形缓冲区。
         * 本拍只需要 pop_bulk 取出来 → 喂给 ms901m_feed_bytes。
         *
         * 这个操作很快（只是内存拷贝 + 状态机推进），
         * 不会拖累主循环的其他任务。 */
        uint8_t buf[IMU_DRAIN_CHUNK];
        size_t  got = bsp_imu_uart_rx_pop_bulk(buf, sizeof(buf));
        if (got > 0u) {
            ms901m_feed_bytes(buf, got);
        }

        /* 取最新姿态快照。
         * 注意：即使本拍没有新字节（got == 0），snap 仍然保留上一帧的值。
         * 这样 1 Hz 日志始终有数据可打印。
         * has_attitude 标志在 main 的 wait_for_ms901m_attitude 中
         * 已经等到首帧后置位，这里不用再检查。 */
        ms901m_get_snapshot(&snap);

        /* ============================================================
         * 5 Hz 任务：LED 心跳（绿灯翻转）
         * ============================================================
         * 每 200 ms 翻转一次绿灯。
         * 如果绿灯不闪了 → 代码卡住了（SysTick 停了）。
         * 这是一个"心跳灯"（heartbeat LED）——和心电图的滴答声同理。
         *
         * BSP_LED_G_PORT/PIN 在 bsp_gpio.h 中定义。
         * DL_GPIO_togglePins() 自动翻转引脚电平（高→低，低→高）。 */
        if ((tick_count % PHASE_LED_TICKS) == 0u) {
            DL_GPIO_togglePins(BSP_LED_G_PORT, BSP_LED_G_PIN);
        }

        /* ============================================================
         * 1 Hz 任务：串口调试日志
         * ============================================================
         * 每 1 秒通过 XDS-UART 打印一行日志：
         *
         *   [hb] t=12s pitch= 1.23 roll= 0.05 gy= 3.45 T= 25.30C
         *        ms901m_good=1200 bad=0 over=0 k230_rx=0b/s
         *
         * 字段解读：
         *   t           = 启动至今的秒数
         *   pitch       = 俯仰角（°）
         *   roll        = 横滚角（°）
         *   gy          = Y 轴角速度（°/s）
         *   T           = 传感器温度（°C）
         *   ms901m_good = MS901M 累计成功帧数
         *   ms901m_bad  = MS901M 累计失败帧数（如果增长，串口有误码）
         *   over        = UART3 环形缓冲区溢出次数（如果增长，CPU 太慢）
         *   k230_rx     = 过去 1 秒从 K230 收到的字节数
         *
         * 这条日志是阶段 1 最重要的调试工具——
         * 所有关键数据一目了然，任何异常都能在 1 秒内被发现。 */
        if ((tick_count % PHASE_LOG_TICKS) == 0u) {
            /* 计算过去 1 秒 K230 接收字节数 */
            uint32_t total_rx = bsp_k230_uart_total_rx();
            uint32_t delta_rx = total_rx - last_total_rx;
            last_total_rx = total_rx;

            /* 用定点格式化宏打印浮点数，避免 printf("%f") 的栈溢出风险 */
            (void)printf("[hb] t=%lus pitch=%c%ld.%02lu roll=%c%ld.%02lu "
                         "gy=%c%ld.%02lu T=%c%ld.%02luC "
                         "ms901m_good=%lu bad=%lu over=%lu k230_rx=%lub/s\n",
                (unsigned long)(tick_count / 1000u),
                F2_S(snap.pitch_deg), (long)F2_I(snap.pitch_deg), (unsigned long)F2_F(snap.pitch_deg),
                F2_S(snap.roll_deg),  (long)F2_I(snap.roll_deg),  (unsigned long)F2_F(snap.roll_deg),
                F2_S(snap.gy_dps),    (long)F2_I(snap.gy_dps),    (unsigned long)F2_F(snap.gy_dps),
                F2_S(snap.temp_c),    (long)F2_I(snap.temp_c),    (unsigned long)F2_F(snap.temp_c),
                (unsigned long)ms901m_good_frames(),
                (unsigned long)ms901m_bad_frames(),
                (unsigned long)bsp_imu_uart_rx_overrun(),
                (unsigned long)delta_rx);
        }
    }
}
