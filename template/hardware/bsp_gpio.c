/**
 * @file    bsp_gpio.c
 * @brief   GPIO 引脚初始化 —— 手工调用 DriverLib，绕开 SDK 2.10 codegen bug
 *
 * 与 SDK 自动生成的 SYSCFG_DL_GPIO_init() 等价的最小手工实现。
 * 拆分为 3 个静态函数让阅读更清晰：
 *   · init_outputs_porta() —— PORTA 的 6 路输出
 *   · init_outputs_portb() —— PORTB 的 4 路输出
 *   · init_inputs_porta()  —— PORTA 的 3 路输入（START_BTN / ENC_R_A / ENC_R_B）
 *
 * Stage 1.5 后 PORTB 不再有业务输入：IMU_INT(PB4) 随 MPU6050→MS901M 切换下线。
 *
 * GPIO power 启用不在这里做：UART/PWM/QEI 等 peripheral 占用了 PORTA/PORTB
 * 之后，SDK 在 SYSCFG_DL_initPower() 里已经 `DL_GPIO_enablePower(GPIOA/B)`。
 */
#include "bsp_gpio.h"

/* -------------------------------------------------------------------------- */
/* PORTA 输出：BUZZER / LASER_EN / AIN1 / AIN2 / BIN1 / BIN2                  */
/*   全部 CLEARED —— 蜂鸣器静音 / 激光关 / 电机方向位低，等待业务按需驱动     */
/* -------------------------------------------------------------------------- */
static void init_outputs_porta(void)
{
    DL_GPIO_initDigitalOutput(BSP_BUZZER_IOMUX);
    DL_GPIO_initDigitalOutput(BSP_LASER_EN_IOMUX);
    DL_GPIO_initDigitalOutput(BSP_AIN1_IOMUX);
    DL_GPIO_initDigitalOutput(BSP_AIN2_IOMUX);
    DL_GPIO_initDigitalOutput(BSP_BIN1_IOMUX);
    DL_GPIO_initDigitalOutput(BSP_BIN2_IOMUX);

    DL_GPIO_clearPins(GPIOA,
        BSP_BUZZER_PIN | BSP_LASER_EN_PIN |
        BSP_AIN1_PIN | BSP_AIN2_PIN | BSP_BIN1_PIN | BSP_BIN2_PIN);

    DL_GPIO_enableOutput(GPIOA,
        BSP_BUZZER_PIN | BSP_LASER_EN_PIN |
        BSP_AIN1_PIN | BSP_AIN2_PIN | BSP_BIN1_PIN | BSP_BIN2_PIN);
}

/* -------------------------------------------------------------------------- */
/* PORTB 输出：STBY / LED_R / LED_G / LED_B                                   */
/*   STBY/LED_G/LED_B 初值 CLEARED；LED_R 初值 SET（红灯亮，标识 init OK）    */
/* -------------------------------------------------------------------------- */
static void init_outputs_portb(void)
{
    DL_GPIO_initDigitalOutput(BSP_STBY_IOMUX);
    DL_GPIO_initDigitalOutput(BSP_LED_R_IOMUX);
    DL_GPIO_initDigitalOutput(BSP_LED_G_IOMUX);
    DL_GPIO_initDigitalOutput(BSP_LED_B_IOMUX);

    DL_GPIO_clearPins(GPIOB,
        BSP_STBY_PIN | BSP_LED_G_PIN | BSP_LED_B_PIN);
    DL_GPIO_setPins(GPIOB, BSP_LED_R_PIN);

    DL_GPIO_enableOutput(GPIOB,
        BSP_STBY_PIN | BSP_LED_R_PIN | BSP_LED_G_PIN | BSP_LED_B_PIN);
}

/* -------------------------------------------------------------------------- */
/* PORTA 输入：START_BTN / ENC_R_A / ENC_R_B —— 全部启用内部上拉 + 施密特滞回 */
/*                                                                              */
/*   设计要点（Stage 2.4 / 2026-05-09 修复）：                                  */
/*     原版本 ENC_R_A/B 用 RESISTOR_NONE + HYSTERESIS_DISABLE，引脚悬空时（如 */
/*     编码器未上电、连接松动、调试期不接电机）会被 AC 噪声触发**雪崩中断**：  */
/*     PA12/PA13 双沿中断 + 浮空噪声 → 数十 kHz 边沿率 → CPU 100% 占用在      */
/*     `GROUP1_IRQHandler`（GPIO 中断在 MSPM0 上共享 GROUP1 入口，参考         */
/*     bsp_motor.c 的 ISR 注释），主循环 / SysTick / printf 全部饿死。          */
/*                                                                              */
/*   修复策略（双保险）：                                                       */
/*     ① RESISTOR_PULL_UP：引脚悬空 / 高阻态时被内部 ~32 kΩ 上拉到 VDD，电平  */
/*        保持稳定高，不会被噪声拉到阈值附近；编码器主动驱动时（推挽 / 开漏    */
/*        都行）能压过 32 kΩ 上拉，正常输出 0/1。                              */
/*     ② HYSTERESIS_ENABLE：施密特触发器滞回 ~100 mV，进一步压制阈值附近的    */
/*        噪声毛刺，避免边沿事件二次触发。                                     */
/*                                                                              */
/*   不开 polarity / 不 enableInterrupt —— 阶段 2 由 bsp_motor 按需开。       */
/* -------------------------------------------------------------------------- */
static void init_inputs_porta(void)
{
    DL_GPIO_initDigitalInputFeatures(BSP_START_BTN_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(BSP_ENC_R_A_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(BSP_ENC_R_B_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
}

/* PORTB 输入：Stage 1.5 后无业务输入；IMU_INT(PB4) 已随 MS901M 替换下线。
 * 若将来再添加 PORTB 输入，按 init_inputs_porta() 模板新增 init_inputs_portb()。 */

void bsp_gpio_init(void)
{
    init_outputs_porta();
    init_outputs_portb();
    init_inputs_porta();
}
