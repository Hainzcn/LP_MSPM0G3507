/**
 * @file    bsp_gpio.c
 * @brief   GPIO 引脚初始化 —— 手工调用 DriverLib，绕开 SDK 2.10 codegen bug
 *
 * 与 SDK 自动生成的 SYSCFG_DL_GPIO_init() 等价的最小手工实现。
 * 拆分为 4 个静态函数让阅读更清晰：
 *   · init_outputs_porta() —— PORTA 的 6 路输出
 *   · init_outputs_portb() —— PORTB 的 4 路输出
 *   · init_inputs_porta()  —— PORTA 的 3 路输入（START_BTN / ENC_R_A / ENC_R_B）
 *   · init_inputs_portb()  —— PORTB 的 1 路输入（IMU_INT）
 *
 * GPIO power 启用不在这里做：UART/I2C/PWM/QEI 等 peripheral 占用了 PORTA/PORTB
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
/* PORTA 输入：START_BTN（上拉）/ ENC_R_A / ENC_R_B（无上拉）                  */
/*   不开 polarity / 不 enableInterrupt —— 阶段 2 各模块按需开                */
/* -------------------------------------------------------------------------- */
static void init_inputs_porta(void)
{
    DL_GPIO_initDigitalInputFeatures(BSP_START_BTN_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(BSP_ENC_R_A_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(BSP_ENC_R_B_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
}

/* -------------------------------------------------------------------------- */
/* PORTB 输入：IMU_INT（无上拉，外部 MPU6050 推挽输出）                        */
/* -------------------------------------------------------------------------- */
static void init_inputs_portb(void)
{
    DL_GPIO_initDigitalInputFeatures(BSP_IMU_INT_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
}

void bsp_gpio_init(void)
{
    init_outputs_porta();
    init_outputs_portb();
    init_inputs_porta();
    init_inputs_portb();
}
