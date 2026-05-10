/**
 * @file    main.c
 * @brief   阶段 2.2 主入口：MSPM0G3507 自平衡瞄准小车 —— 上车基线固件
 *          （姿态遥测 + 电机 + 编码器 + 电池 + 安全状态机 + 平衡 / 速度环骨架）。
 *
 *  调用链：
 *      SYSCFG_DL_init()      -- 由 SysConfig 自动生成，配置时钟 / peripheral pins
 *      bsp_gpio_init         -- 14 路业务 GPIO 手工 init（绕开 SDK multi-pad bug）
 *      bsp_systick_init      -- 1 kHz 节拍 + ms 计时
 *      bsp_log_uart_init     -- UART0 (XDS-UART) printf retarget
 *      bsp_k230_uart_init    -- UART1 + DMA RX 接收骨架
 *      bsp_imu_uart_init     -- UART3 (MS901M) RX 中断 + 256 B 环缓
 *      ms901m_init           -- 解析器状态机复位 + 量程系数（±4 g / ±2000 dps）
 *      wait_for_ms901m_*     -- 上电后 3000 ms 内等到第一帧 0x01；超时报警
 *      bsp_motor_init        -- TB6612 + QEI + 右编码器中断（X4 解码默认开 PA13 双沿）
 *      bsp_battery_init      -- ADC0/PB24 触发首次软件转换
 *      app_safety_init       -- 安全状态机置 DISARMED + 电机 brake + STBY 关
 *      app_balance_init      -- 两路 PID 安全默认（增益 0 + 输出限幅 + D 滤波）
 *      app_balance_run       -- 默认装车模式；UART `t`/`test` 可切入电机演示
 *
 *  失败处理：MS901M 上电后 3000 ms 仍未收到 0x01 姿态帧 → LED_R 常亮 + 蜂鸣
 *            200 ms，然后死循环；不进入主循环以免上报无效数据。
 *
 *  GPIO 备注：业务 GPIO 全部由 bsp_gpio.[ch] 管理，宏前缀 BSP_*；syscfg 不
 *  生成 GPIO_OUT_*_PORT/PIN 这类宏（详见 EIDE/LP_MSPM0G3507.syscfg 头部注释、
 *  docs/TaskLog/Stage1-IMU-BT-Telemetry.md §8.5）。同 port 的引脚可合并到
 *  一次 DL_GPIO_xxxPins 调用（LED_R/G/B 同属 GPIOB → BSP_LED_*_PORT）。
 *
 *  Stage 1.5 变更（2026-05-07）：原 I²C+MPU6050 链路因开发板 PB2/PB3 缺上拉
 *  电阻被废弃，改走 UART+ATK-MS901M（板载 EKF），姿态直接采纳 0x01 帧。
 *
 *  Stage 1.6 变更（2026-05-08）：IMU 串口从 UART2/PA21/PA22 迁到 UART3/PB12/
 *  PB13（原蓝牙引脚），原因是 PA21 未引到 BoosterPack 需焊接；同期蓝牙
 *  HC-04 模块整体下线，遥测改走 1 Hz XDS-UART printf；MS901M 0x01 姿态帧
 *  由 app_balance 再做一阶低通后进入平衡环。详见
 *  docs/TaskLog/Stage1.5-IMU-Swap-MS901M.md §11 Stage 1.6 重排。
 *
 *  Stage 2.2 变更（2026-05-09）：上车基线固件就绪。原 `app_telemetry_run()`
 *  入口被 `app_balance_run()` 取代，后者吸收了 telemetry 的 IMU drain + 1 Hz
 *  心跳日志，同时新增 100 Hz 控制环（safety + balance step）+ LED 状态指示。
 *  ⚠️ PID 增益默认 0，上电不会自己动；装车整定时通过串口注入即可。
 *  详见 docs/TaskLog/Stage2-MotorDrive-Encoder.md §3.5 / §6.3。
 */

#include "ti_msp_dl_config.h"

#include "bsp_battery.h"
#include "bsp_gpio.h"
#include "bsp_imu_uart.h"
#include "bsp_k230_uart.h"
#include "bsp_log_uart.h"
#include "bsp_motor.h"
#include "bsp_systick.h"
#include "ms901m.h"

#include "app_balance.h"
#include "app_motor_demo.h"
#include "app_safety.h"

#include <stdint.h>
#include <stdio.h>

/* PT_LOAD 段 8 字节对齐尾填充移到独立文件 hardware/bsp_flash_pad.c
 * scatter 用模块级 `bsp_flash_pad.o (+Last)` 选择器（见
 * template/keil/mspm0g3507.sct 与 docs/TaskLog/Stage1-IMU-BT-Telemetry.md §8.6）。 */

/* MS901M 上电后允许多长时间没出第一帧 0x01 姿态。实车电源冷启动时模块
 * 可能比主控慢，给 3 s 窗口，避免初始化期红灯常亮误判为 fatal。 */
#define MS901M_BOOT_TIMEOUT_MS   3000u

/* 等待期间每拍 drain 字节数上限（≥ 单帧最大 17 B + 几帧裕度即可） */
#define MS901M_DRAIN_CHUNK       64u

/* 装车模式默认 PID：上电保持 0 输出，实际整定通过 XDS-UART 注入。 */
#define LOAD_TEST_BALANCE_KP     (0.0f)
#define LOAD_TEST_BALANCE_KI     (0.0f)
#define LOAD_TEST_BALANCE_KD     (0.0f)
#define LOAD_TEST_SPEED_KP       (0.0f)
#define LOAD_TEST_SPEED_KI       (0.0f)
#define LOAD_TEST_SPEED_KD       (0.0f)

static void fatal_imu_init_failure(int32_t rc)
{
    DL_GPIO_setPins(BSP_LED_R_PORT, BSP_LED_R_PIN);
    DL_GPIO_clearPins(BSP_LED_G_PORT, BSP_LED_G_PIN | BSP_LED_B_PIN);
    DL_GPIO_setPins(BSP_BUZZER_PORT, BSP_BUZZER_PIN);
    bsp_systick_delay_ms(200u);
    DL_GPIO_clearPins(BSP_BUZZER_PORT, BSP_BUZZER_PIN);

    (void)printf("[FATAL] ms901m boot timeout rc=%ld\n", (long)rc);
    for (;;) { __WFI(); }
}

/**
 * @brief  上电后等 MS901M 第一帧 0x01 姿态，最长 timeout_ms 毫秒。
 * @return 0 = 拿到了；-1 = 超时。
 *
 *  MS901M 默认主动按帧上报，主控只需被动 drain UART3 RX 环缓 + 喂解析器。
 *  本函数主动 1 ms 轮询而非 SysTick 节拍，避免和 main 主循环 tick 标志竞争。
 */
static int32_t wait_for_ms901m_attitude(uint32_t timeout_ms)
{
    uint32_t start = bsp_systick_get_ms();
    uint8_t  buf[MS901M_DRAIN_CHUNK];

    while ((bsp_systick_get_ms() - start) < timeout_ms) {
        size_t got = bsp_imu_uart_rx_pop_bulk(buf, sizeof(buf));
        if (got > 0u) {
            ms901m_feed_bytes(buf, got);
            if (ms901m_has_attitude()) {
                return 0;
            }
        }
        /* 没有数据 → 轻睡 1 ms 减小空转占空 */
        bsp_systick_delay_ms(1u);
    }
    return -1;
}

int main(void)
{
    SYSCFG_DL_init();
    bsp_gpio_init();

    if (bsp_systick_init(1000u) != 0) {
        for (;;) { __WFI(); }
    }

    bsp_log_uart_init();
    bsp_k230_uart_init();
    bsp_imu_uart_init();

    /* MS901M 出厂默认 ±4 g / ±2000 dps（与 ATK 上位机默认量程一致） */
    ms901m_init(4, 2000);

    (void)printf("\n[boot] MSPM0G3507 stage2.2 balance baseline start (MS901M / TB6612 / safety)\n");

    int32_t rc = wait_for_ms901m_attitude(MS901M_BOOT_TIMEOUT_MS);
    if (rc != 0) {
        fatal_imu_init_failure(rc);
    }

    (void)printf("[boot] MS901M attitude online, %lu good / %lu bad frames\n",
        (unsigned long)ms901m_good_frames(),
        (unsigned long)ms901m_bad_frames());

    /* Stage 2.7：上电默认进入装车模式。仅在装车模式收到 UART `t` / `test`
     * 后切入电机演示；电机演示收到 `l` / `load` 后返回装车模式。
     *
     *   bsp_motor_init  必须在 bsp_gpio_init 之后（依赖 BSP_ENC_R_*_PIN 配置）
     *   bsp_battery_init 必须在 SYSCFG_DL_init 之后（依赖 ADC_BAT_INST 已配）
     *   app_safety_init 必须在 bsp_motor_init 之后（构造期会调 brake/enable）
     *   app_balance_init 任意位置都可以（纯 PID 数据结构初始化）。 */
    bsp_motor_init();
    bsp_battery_init();

    /* 启动正常：红灯灭、绿灯由心跳任务接管，蓝灯留给后续状态指示。
     * 注意：app_safety 状态显示也会写 LED_R（FALLEN / BAT_STOP 常亮、BAT_WARN 闪），
     *       这里清一下当作 "Boot OK" 视觉反馈，进入 run() 后由 5 Hz 任务接管。 */
    DL_GPIO_clearPins(BSP_LED_R_PORT, BSP_LED_R_PIN);

    for (;;) {
        (void)printf("[boot] entering load balance mode; inject PID by UART\r\n");

        app_safety_init();
        app_balance_init();
        app_balance_set_balance_gains(LOAD_TEST_BALANCE_KP,
            LOAD_TEST_BALANCE_KI, LOAD_TEST_BALANCE_KD);
        app_balance_set_speed_gains(LOAD_TEST_SPEED_KP,
            LOAD_TEST_SPEED_KI, LOAD_TEST_SPEED_KD);
        (void)app_safety_arm();

        if (app_balance_run()) {
            (void)printf("[boot] switching to motor test demo; send 'l' or 'load' to return\r\n");
            (void)app_motor_demo_run();
        }
    }

    return 0;
}
