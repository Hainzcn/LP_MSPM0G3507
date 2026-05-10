/**
 * @file    bsp_gpio.h
 * @brief   GPIO 引脚抽象层 —— 绕开 SDK 2.10 multi-pad codegen bug
 *
 * 设计要点：
 *   · 本工程所有业务 GPIO 都不走 SysConfig（详见 EIDE/LP_MSPM0G3507.syscfg 头部注释、
 *     docs/TaskLog/Stage1-IMU-BT-Telemetry.md §8.5）。
 *   · 引脚分配真值表见 docs/TaskLog/Stage0-PinAllocation.md，本头文件中
 *     12 个 BSP_*_PORT/PIN/IOMUX 宏与该表保持一致；任何引脚改动必须先改文档。
 *   · IOMUX_PINCMxx 与 DL_GPIO_PIN_<bit> 直接来自 ti/devices/msp/peripherals/
 *     hw_iomux.h、ti/driverlib/dl_gpio.h，不依赖 SysConfig 生成的 _PIN/_PORT 宏。
 *   · `bsp_gpio_init()` 由 `main.c` 在 `SYSCFG_DL_init()` 之后立即调用。
 *
 * 注：阶段 1 仅业务上需要 LED / BUZZER / STBY / 电机方向（默认低，安全态），
 *     输入引脚（ENC_R_* / IMU_INT）只做方向与上拉配置，
 *     **不**开 NVIC 中断——中断由阶段 2 各自模块在使用前 enable，避免
 *     当前阶段没有 ISR 时触发默认 fault handler。
 */
#ifndef BSP_GPIO_H
#define BSP_GPIO_H

#include "ti_msp_dl_config.h"

/* ========================================================================== */
/* 输出 —— 状态 LED (PB)                                                       */
/* ========================================================================== */
#define BSP_LED_R_PORT          GPIOB
#define BSP_LED_R_PIN           DL_GPIO_PIN_26
#define BSP_LED_R_IOMUX         IOMUX_PINCM57

#define BSP_LED_G_PORT          GPIOB
#define BSP_LED_G_PIN           DL_GPIO_PIN_27
#define BSP_LED_G_IOMUX         IOMUX_PINCM58

#define BSP_LED_B_PORT          GPIOB
#define BSP_LED_B_PIN           DL_GPIO_PIN_22
#define BSP_LED_B_IOMUX         IOMUX_PINCM50

/* ========================================================================== */
/* 输出 —— 蜂鸣器 / 激光使能 (PA, 单 pad)                                       */
/* ========================================================================== */
#define BSP_BUZZER_PORT         GPIOA
#define BSP_BUZZER_PIN          DL_GPIO_PIN_0
#define BSP_BUZZER_IOMUX        IOMUX_PINCM1

#define BSP_LASER_EN_PORT       GPIOA
#define BSP_LASER_EN_PIN        DL_GPIO_PIN_1
#define BSP_LASER_EN_IOMUX      IOMUX_PINCM2

/* ========================================================================== */
/* 输出 —— TB6612 电机方向 / STBY                                               */
/*   阶段 1 业务上不驱动电机；本组引脚只在 init 时设安全态（全部低 + STBY 低）  */
/*   防止 PWM_MOTOR 上电瞬间电机乱动。STBY 低 → TB6612 进 Standby → 输出 Hi-Z  */
/* ========================================================================== */
#define BSP_STBY_PORT           GPIOB
#define BSP_STBY_PIN            DL_GPIO_PIN_0
#define BSP_STBY_IOMUX          IOMUX_PINCM12

#define BSP_AIN1_PORT           GPIOA
#define BSP_AIN1_PIN            DL_GPIO_PIN_15
#define BSP_AIN1_IOMUX          IOMUX_PINCM37

#define BSP_AIN2_PORT           GPIOA
#define BSP_AIN2_PIN            DL_GPIO_PIN_16
#define BSP_AIN2_IOMUX          IOMUX_PINCM38

#define BSP_BIN1_PORT           GPIOA
#define BSP_BIN1_PIN            DL_GPIO_PIN_26
#define BSP_BIN1_IOMUX          IOMUX_PINCM59

#define BSP_BIN2_PORT           GPIOA
#define BSP_BIN2_PIN            DL_GPIO_PIN_27
#define BSP_BIN2_IOMUX          IOMUX_PINCM60

/* 右编码器 A 相 —— 阶段 2 拟双边沿中断 / capture，阶段 1 只配方向 */
#define BSP_ENC_R_A_PORT        GPIOA
#define BSP_ENC_R_A_PIN         DL_GPIO_PIN_12
#define BSP_ENC_R_A_IOMUX       IOMUX_PINCM34

/* 右编码器 B 相 —— ISR 内读电平判方向，无需中断 */
#define BSP_ENC_R_B_PORT        GPIOA
#define BSP_ENC_R_B_PIN         DL_GPIO_PIN_13
#define BSP_ENC_R_B_IOMUX       IOMUX_PINCM35

/* PB4 已于 Stage 1.5 释放：MPU6050 被 ATK-MS901M 串口姿态传感器替代后，
 * 不再需要 IMU DataReady 中断引脚（MS901M 主动按帧上报）。详见
 * docs/TaskLog/Stage1.5-IMU-Swap-MS901M.md。 */

/* ========================================================================== */
/* API                                                                         */
/* ========================================================================== */
/**
 * @brief 初始化所有业务 GPIO（输出 + 输入），由 main.c 在 SYSCFG_DL_init() 之后
 *        立即调用。函数内部：
 *          ① 10 个输出 pin 全部 initDigitalOutput()；
 *          ② STBY/AIN/BIN/BUZZER/LASER_EN/LED_G/LED_B 初值 CLEARED；
 *          ③ LED_R 初值 SET（开机点亮一颗，便于直观判断 init 完成）；
 *          ④ 2 个输入 pin initDigitalInputFeatures()，均启用上拉；
 *          ⑤ **不**调用 NVIC_EnableIRQ —— 中断在阶段 2 各模块按需开启。
 *
 *  Stage 1.5 后：原 PORTB 输入 IMU_INT(PB4) 已下线，PORTB 不再有业务输入。
 */
void bsp_gpio_init(void);

#endif /* BSP_GPIO_H */
