/**
 * @file    bsp_motor.c
 * @brief   TB6612FNG 双路电机驱动实现，详见 bsp_motor.h。
 */

#include "bsp_motor.h"
#include "bsp_gpio.h"

static float s_duty[BSP_MOTOR_COUNT] = {0.0f, 0.0f};

/* -------------------------------------------------------------------------- */
/* 方向引脚控制                                                                */
/* -------------------------------------------------------------------------- */

static void set_dir_forward(bsp_motor_id_t id)
{
    if (id == BSP_MOTOR_LEFT) {
        DL_GPIO_setPins(BSP_AIN1_PORT, BSP_AIN1_PIN);
        DL_GPIO_clearPins(BSP_AIN2_PORT, BSP_AIN2_PIN);
    } else {
        DL_GPIO_setPins(BSP_BIN1_PORT, BSP_BIN1_PIN);
        DL_GPIO_clearPins(BSP_BIN2_PORT, BSP_BIN2_PIN);
    }
}

static void set_dir_reverse(bsp_motor_id_t id)
{
    if (id == BSP_MOTOR_LEFT) {
        DL_GPIO_clearPins(BSP_AIN1_PORT, BSP_AIN1_PIN);
        DL_GPIO_setPins(BSP_AIN2_PORT, BSP_AIN2_PIN);
    } else {
        DL_GPIO_clearPins(BSP_BIN1_PORT, BSP_BIN1_PIN);
        DL_GPIO_setPins(BSP_BIN2_PORT, BSP_BIN2_PIN);
    }
}

static void set_dir_coast(bsp_motor_id_t id)
{
    if (id == BSP_MOTOR_LEFT) {
        DL_GPIO_clearPins(BSP_AIN1_PORT, BSP_AIN1_PIN);
        DL_GPIO_clearPins(BSP_AIN2_PORT, BSP_AIN2_PIN);
    } else {
        DL_GPIO_clearPins(BSP_BIN1_PORT, BSP_BIN1_PIN);
        DL_GPIO_clearPins(BSP_BIN2_PORT, BSP_BIN2_PIN);
    }
}

static void set_dir_brake(bsp_motor_id_t id)
{
    if (id == BSP_MOTOR_LEFT) {
        DL_GPIO_setPins(BSP_AIN1_PORT, BSP_AIN1_PIN);
        DL_GPIO_setPins(BSP_AIN2_PORT, BSP_AIN2_PIN);
    } else {
        DL_GPIO_setPins(BSP_BIN1_PORT, BSP_BIN1_PIN);
        DL_GPIO_setPins(BSP_BIN2_PORT, BSP_BIN2_PIN);
    }
}

/* -------------------------------------------------------------------------- */
/* PWM 占空比控制                                                              */
/*   CCR = 0 → 0 %；CCR = LOAD → ≈100 %                                      */
/*   SysConfig 生成的 GPIO_PWM_MOTOR_Cx_IDX 宏直接用作 CC 索引                 */
/* -------------------------------------------------------------------------- */

static void set_pwm_ccr(bsp_motor_id_t id, uint16_t ccr)
{
    if (ccr > BSP_MOTOR_PWM_LOAD) {
        ccr = (uint16_t)BSP_MOTOR_PWM_LOAD;
    }
    if (id == BSP_MOTOR_LEFT) {
        DL_TimerA_setCaptureCompareValue(PWM_MOTOR_INST, (uint32_t)ccr,
            GPIO_PWM_MOTOR_C0_IDX);
    } else {
        DL_TimerA_setCaptureCompareValue(PWM_MOTOR_INST, (uint32_t)ccr,
            GPIO_PWM_MOTOR_C1_IDX);
    }
}

/* -------------------------------------------------------------------------- */
/* 公开 API                                                                    */
/* -------------------------------------------------------------------------- */

void bsp_motor_init(void)
{
    set_pwm_ccr(BSP_MOTOR_LEFT,  0u);
    set_pwm_ccr(BSP_MOTOR_RIGHT, 0u);
    set_dir_coast(BSP_MOTOR_LEFT);
    set_dir_coast(BSP_MOTOR_RIGHT);
    s_duty[BSP_MOTOR_LEFT]  = 0.0f;
    s_duty[BSP_MOTOR_RIGHT] = 0.0f;
}

void bsp_motor_set_duty(bsp_motor_id_t id, float duty_pct)
{
    if ((uint32_t)id >= BSP_MOTOR_COUNT) {
        return;
    }

    float clamped = duty_pct;
    if (clamped > 100.0f)  clamped = 100.0f;
    if (clamped < -100.0f) clamped = -100.0f;

    s_duty[id] = clamped;

    float abs_duty = (clamped >= 0.0f) ? clamped : -clamped;
    uint16_t ccr   = (uint16_t)(abs_duty * (float)BSP_MOTOR_PWM_LOAD / 100.0f);

    if (clamped >= 0.0f) {
        set_dir_forward(id);
    } else {
        set_dir_reverse(id);
    }

    set_pwm_ccr(id, ccr);
}

void bsp_motor_brake(bsp_motor_id_t id)
{
    if ((uint32_t)id >= BSP_MOTOR_COUNT) {
        return;
    }
    set_dir_brake(id);
    set_pwm_ccr(id, 0u);
    s_duty[id] = 0.0f;
}

void bsp_motor_coast(bsp_motor_id_t id)
{
    if ((uint32_t)id >= BSP_MOTOR_COUNT) {
        return;
    }
    set_dir_coast(id);
    set_pwm_ccr(id, 0u);
    s_duty[id] = 0.0f;
}

void bsp_motor_standby(bool enable)
{
    if (enable) {
        DL_GPIO_setPins(BSP_STBY_PORT, BSP_STBY_PIN);
    } else {
        DL_GPIO_clearPins(BSP_STBY_PORT, BSP_STBY_PIN);
    }
}

float bsp_motor_get_duty(bsp_motor_id_t id)
{
    if ((uint32_t)id >= BSP_MOTOR_COUNT) {
        return 0.0f;
    }
    return s_duty[id];
}
