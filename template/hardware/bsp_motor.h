/**
 * @file    bsp_motor.h
 * @brief   TB6612FNG 双路电机驱动 —— 370 电机 (6 V / 620 RPM 空载)。
 *
 * 硬件映射（与 Stage0-PinAllocation.md / bsp_gpio.h 保持一致）：
 *   左电机  PWMA = PA8  (TIMA0_CCP0)   AIN1 = PA15   AIN2 = PA16
 *   右电机  PWMB = PA9  (TIMA0_CCP1)   BIN1 = PA26   BIN2 = PA27
 *   全局    STBY = PB0  (低 = 待机 Hi-Z，高 = 正常工作)
 *
 * PWM 参数（SysConfig PWM_MOTOR 实例）：
 *   时钟 32 MHz，LOAD = 1599 → 20 kHz，分辨率 ≈ 0.06 %
 *
 * TB6612 真值表：
 *   IN1  IN2  │ 模式
 *   1    0    │ 正转 (CW)
 *   0    1    │ 反转 (CCW)
 *   1    1    │ 短路刹车
 *   0    0    │ 滑行 (Coast)
 *   STBY = 0  │ 待机，输出 Hi-Z
 *
 * API 约定：
 *   - duty_pct 范围 -100.0 ~ +100.0，正 = 正转，负 = 反转
 *   - bsp_motor_init() 仅置零占空比 + 滑行态，不拉高 STBY
 *   - 使用前须调用 bsp_motor_standby(true) 退出待机
 */

#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/* 370 电机参数                                                                */
/* -------------------------------------------------------------------------- */
#define BSP_MOTOR_RATED_VOLTAGE_MV      6000u
#define BSP_MOTOR_NOLOAD_SPEED_RPM      620u

/* PWM LOAD 值，须与 SysConfig PWM_MOTOR.timerCount 一致
 * 频率 = PWM_MOTOR_INST_CLK_FREQ / (LOAD + 1) = 32 MHz / 1600 = 20 kHz */
#define BSP_MOTOR_PWM_LOAD              1599u

/* -------------------------------------------------------------------------- */
/* 电机编号                                                                    */
/* -------------------------------------------------------------------------- */
typedef enum {
    BSP_MOTOR_LEFT  = 0,
    BSP_MOTOR_RIGHT = 1,
    BSP_MOTOR_COUNT = 2
} bsp_motor_id_t;

/* -------------------------------------------------------------------------- */
/* API                                                                         */
/* -------------------------------------------------------------------------- */

/**
 * @brief  电机驱动初始化：占空比清零 + 方向引脚置滑行态。
 *         不修改 STBY（仍为 bsp_gpio_init 设的低电平 = 待机）。
 *         须在 SYSCFG_DL_init() + bsp_gpio_init() 之后调用。
 */
void bsp_motor_init(void);

/**
 * @brief  设置电机占空比。
 * @param  id        电机编号
 * @param  duty_pct  -100.0 ~ +100.0；正 = 正转，负 = 反转，0 = 滑行
 */
void bsp_motor_set_duty(bsp_motor_id_t id, float duty_pct);

/**
 * @brief  短路刹车：IN1 = IN2 = 1，PWM = 0。
 *         电机两端短接，快速制动。
 */
void bsp_motor_brake(bsp_motor_id_t id);

/**
 * @brief  滑行：IN1 = IN2 = 0，PWM = 0。
 *         电机两端开路，自由减速。
 */
void bsp_motor_coast(bsp_motor_id_t id);

/**
 * @brief  STBY 控制。
 * @param  enable  true = 正常工作（STBY 拉高）；false = 待机 Hi-Z（STBY 拉低）
 */
void bsp_motor_standby(bool enable);

/**
 * @brief  读取当前占空比设定值。
 * @return -100.0 ~ +100.0；未初始化或 id 越界返回 0。
 */
float bsp_motor_get_duty(bsp_motor_id_t id);

#ifdef __cplusplus
}
#endif

#endif /* BSP_MOTOR_H */
