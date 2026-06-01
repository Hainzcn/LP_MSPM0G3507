/**
 * @file    app_telemetry.c
 * @brief   阶段 1 调度实现，详见 app_telemetry.h。
 */

#include "app_telemetry.h"
#include "ti_msp_dl_config.h"

#include "bsp_gpio.h"
#include "bsp_systick.h"
#include "bsp_k230_uart.h"
#include "bsp_imu_uart.h"
#include "ms901m.h"

#include <stdio.h>

/* 1 ms tick 周期下的相位计数器：每攒够 N 次 tick 触发一次相应任务 */
#define PHASE_LED_TICKS     200u   /* 5 Hz   */
#define PHASE_LOG_TICKS     1000u  /* 1 Hz   */

/* 单拍最多从 UART3 RX 环缓拉走多少字节给 ms901m 解析。
 * MS901M 默认 5 帧 × 200 Hz × ~15 B ≈ 15 kB/s = 15 B/ms，
 * 64 B 单拍裕度 4×，足够吸收偶发抖动。 */
#define IMU_DRAIN_CHUNK     64u

/* 浮点字段格式化辅助宏（避开 Keil AC6 printf("%f") 浮点路径）：
 *   F2_X100(v) → 把 v ×100 四舍五入为 int32_t（带符号，整数运算只做 1 次）
 *   F2_S(v)    → 符号字符 '-' 或 ' '
 *   F2_I(v)    → 整数部分（无符号绝对值）
 *   F2_F(v)    → 两位小数部分（无符号）
 * 配合 printf 格式 "%c%ld.%02lu" 可正确显示 -0.50 / 1.23 / -1.23 / 99.995→" 100.00"。
 *
 * 设计动机：Keil AC6 标准库 printf("%f") 单次浮点格式化栈帧 200~300 B，
 * 在原 256 B 栈上塞 4 个 %f 立即溢出 → HardFault；改成整数 vararg 后
 * 栈使用降至 < 80 B，且二进制不再链接浮点格式化路径，节省 ~3 KB ROM。
 *
 * 关键正确性：先把 v 整体 ×100 四舍五入到一个 int32_t，再用 / 100 与 % 100
 * 拆整数 / 小数。这样 99.995 → 10000 → " 100.00" 不会出 " 99.100" 这种跨
 * 整数边界的格式错；而且 / 与 % 在 C 上对负数走 truncation toward zero、
 * 商和余数同号，对正负数预先取绝对值后再拆即可保证两位 %02lu 不出 " -23"。
 *
 * 副作用注意：v 在 F2_S/F2_I/F2_F 中各被求值一次（且 F2_X100 内 v 被算 2 次），
 * 总共最多 6 次。传 snap 字段（纯字段访问）无副作用 OK；严禁传 `++i` / `f()`
 * 等带副作用的表达式。 */
#define F2_X100(v)  ((int32_t)((v) * 100.0f + ((v) >= 0.0f ? 0.5f : -0.5f)))
#define F2_S(v)     (F2_X100(v) < 0 ? '-' : ' ')
#define F2_I(v)     ((int32_t)((F2_X100(v) < 0 ? -F2_X100(v) : F2_X100(v)) / 100))
#define F2_F(v)     ((uint32_t)((F2_X100(v) < 0 ? -F2_X100(v) : F2_X100(v)) % 100))

void app_telemetry_run(void)
{
    uint32_t tick_count    = 0u;
    uint32_t last_total_rx = bsp_k230_uart_total_rx();

    ms901m_snapshot_t snap = { 0 };

    for (;;) {
        if (!bsp_systick_consume_tick()) {
            /* 没到下一个 1 ms：低功耗等待，被任意中断唤醒 */
            __WFI();
            continue;
        }
        tick_count++;

        /* ---- 1 kHz：drain UART3 → 喂 ms901m 状态机 → 取最新 snapshot ----
         * UART RX FIFO 半满中断已把字节排入 256 B 环缓，本 tick 只做拷贝 +
         * 解析，I2C 阻塞读已下线，整体 ISR/CPU 负担更轻。 */
        uint8_t buf[IMU_DRAIN_CHUNK];
        size_t  got = bsp_imu_uart_rx_pop_bulk(buf, sizeof(buf));
        if (got > 0u) {
            ms901m_feed_bytes(buf, got);
        }
        ms901m_get_snapshot(&snap);
        /* 注：snap 即使本拍没新字节也会保留上一帧值（has_* 标志已在 main
         * 启动期等到首帧后置位），让下游 1 Hz 任务始终拿到稳定数据。 */

        /* ---- 5 Hz：LED 心跳（绿灯翻转） ----
         * BSP_LED_G_PORT = GPIOB，宏定义见 bsp_gpio.h；
         * GPIO 由 BSP 接管的背景见 docs/TaskLog/Stage1-IMU-BT-Telemetry.md §8.5 */
        if ((tick_count % PHASE_LED_TICKS) == 0u) {
            DL_GPIO_togglePins(BSP_LED_G_PORT, BSP_LED_G_PIN);
        }

        /* ---- 1 Hz：UART0 调试日志 ----
         * 输出 pitch + roll + gy + temp + MS901M 错误统计 + K230 RX 字节速率，
         * 任一字段异常都能在 1 s 内被肉眼捕获。Stage 1.6 起本日志同时承担
         * 姿态可视化职责（原 100 Hz VOFA+ 二进制流随蓝牙下线已停）。 */
        if ((tick_count % PHASE_LOG_TICKS) == 0u) {
            uint32_t total_rx = bsp_k230_uart_total_rx();
            uint32_t delta_rx = total_rx - last_total_rx;
            last_total_rx = total_rx;
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
