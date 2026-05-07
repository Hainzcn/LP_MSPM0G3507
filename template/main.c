/**
 * @file    main.c
 * @brief   阶段 1 主入口：MSPM0G3507 自平衡瞄准小车
 *          —— 蓝牙姿态遥测固件。
 *
 *  调用链：
 *      SYSCFG_DL_init()   -- 由 SysConfig 自动生成，配置时钟 / peripheral pins
 *      bsp_gpio_init      -- 14 路业务 GPIO 手工 init（绕开 SDK multi-pad bug）
 *      bsp_systick_init   -- 1 kHz 节拍 + ms 计时
 *      bsp_log_uart_init  -- UART0 (XDS-UART) printf retarget
 *      bsp_bt_uart_init   -- UART3 (HC-04) RX 中断 + 阻塞 TX
 *      bsp_k230_uart_init -- UART1 + DMA RX 接收骨架
 *      bsp_imu_i2c_init   -- I2C1 控制器
 *      mpu6050_init       -- 出厂复位 + 量程 / DLPF / SMPLRT 配置
 *      att_filter_init    -- 互补滤波累加器清零
 *      app_telemetry_run  -- 永不返回的主循环
 *
 *  失败处理：MPU6050 init 失败时 LED_R 常亮（bsp_gpio_init 已设 SET）+
 *            蜂鸣器报警 200 ms，然后死循环；不进入主循环以免上报无效数据。
 *
 *  GPIO 备注：业务 GPIO 全部由 bsp_gpio.[ch] 管理，宏前缀 BSP_*；syscfg 不
 *  生成 GPIO_OUT_*_PORT/PIN 这类宏（详见 EIDE/LP_MSPM0G3507.syscfg 头部注释、
 *  docs/TaskLog/Stage1-IMU-BT-Telemetry.md §8.5）。同 port 的引脚可合并到
 *  一次 DL_GPIO_xxxPins 调用（LED_R/G/B 同属 GPIOB → BSP_LED_*_PORT）。
 */

#include "ti_msp_dl_config.h"

#include "bsp_gpio.h"
#include "bsp_systick.h"
#include "bsp_log_uart.h"
#include "bsp_bt_uart.h"
#include "bsp_k230_uart.h"
#include "bsp_imu_i2c.h"
#include "mpu6050.h"
#include "att_filter.h"
#include "app_telemetry.h"

#include <stdint.h>
#include <stdio.h>

/* PT_LOAD 段 8 字节对齐尾填充移到独立文件 hardware/bsp_flash_pad.c
 * scatter 用模块级 `bsp_flash_pad.o (+Last)` 选择器（见
 * template/keil/mspm0g3507.sct 与 docs/TaskLog/Stage1-IMU-BT-Telemetry.md §8.6）。
 * 之前放本文件用 `.ANY3 (.flash_pad, +Last)` 抢，被 `.ANY1 (+RO)` 拦下，
 * 12588 → 12604 仍 mod 8 = 4，未对齐，已记录为反面教材。 */

static void fatal_imu_init_failure(int32_t rc)
{
    DL_GPIO_setPins(BSP_LED_R_PORT, BSP_LED_R_PIN);
    DL_GPIO_clearPins(BSP_LED_G_PORT, BSP_LED_G_PIN | BSP_LED_B_PIN);
    DL_GPIO_setPins(BSP_BUZZER_PORT, BSP_BUZZER_PIN);
    bsp_systick_delay_ms(200u);
    DL_GPIO_clearPins(BSP_BUZZER_PORT, BSP_BUZZER_PIN);

    (void)printf("[FATAL] mpu6050_init failed rc=%ld\n", (long)rc);
    for (;;) { __WFI(); }
}

int main(void)
{
    SYSCFG_DL_init();
    bsp_gpio_init();

    if (bsp_systick_init(1000u) != 0) {
        for (;;) { __WFI(); }
    }

    bsp_log_uart_init();
    bsp_bt_uart_init();
    bsp_k230_uart_init();
    bsp_imu_i2c_init();

    (void)printf("\n[boot] MSPM0G3507 stage1 telemetry start\n");

    int32_t rc = mpu6050_init();
    if (rc != 0) {
        fatal_imu_init_failure(rc);
    }
    att_filter_init();

    /* 启动正常：红灯灭、绿灯由心跳任务接管，蓝灯留给后续状态指示 */
    DL_GPIO_clearPins(BSP_LED_R_PORT, BSP_LED_R_PIN);

    app_telemetry_run();
    return 0;
}
