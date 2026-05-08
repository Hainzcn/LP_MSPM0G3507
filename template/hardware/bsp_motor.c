/**
 * @file    bsp_motor.c
 * @brief   阶段 2 电机底层驱动实现。
 */

#include "bsp_motor.h"

#include "bsp_gpio.h"
#include "bsp_systick.h"
#include "ti_msp_dl_config.h"

#include <stddef.h>

#define BSP_MOTOR_BTN_DEBOUNCE_MS                  (80u)
#define BSP_MOTOR_LEFT_SIGN                        (1)
#define BSP_MOTOR_RIGHT_SIGN                       (1)

static volatile int32_t  s_right_count            = 0;
static volatile uint8_t  s_toggle_request         = 0u;
static volatile uint32_t s_last_button_ms         = 0u;

static int32_t  s_left_count                      = 0;
static uint16_t s_left_raw_prev                   = 0u;
static float    s_left_angle_deg                  = 0.0f;
static float    s_right_angle_deg                 = 0.0f;

static uint32_t clamp_abs_permille(int16_t permille)
{
    int32_t duty = (permille >= 0) ? permille : -(int32_t)permille;
    if (duty > BSP_MOTOR_PWM_MAX_PERMILLE) {
        duty = BSP_MOTOR_PWM_MAX_PERMILLE;
    }
    return (uint32_t)duty;
}

static void set_tb6612_dir_left(int16_t permille)
{
    if (permille > 0) {
        DL_GPIO_setPins(BSP_AIN1_PORT, BSP_AIN1_PIN);
        DL_GPIO_clearPins(BSP_AIN2_PORT, BSP_AIN2_PIN);
    } else if (permille < 0) {
        DL_GPIO_clearPins(BSP_AIN1_PORT, BSP_AIN1_PIN);
        DL_GPIO_setPins(BSP_AIN2_PORT, BSP_AIN2_PIN);
    } else {
        DL_GPIO_clearPins(BSP_AIN1_PORT, BSP_AIN1_PIN);
        DL_GPIO_clearPins(BSP_AIN2_PORT, BSP_AIN2_PIN);
    }
}

static void set_tb6612_dir_right(int16_t permille)
{
    if (permille > 0) {
        DL_GPIO_setPins(BSP_BIN1_PORT, BSP_BIN1_PIN);
        DL_GPIO_clearPins(BSP_BIN2_PORT, BSP_BIN2_PIN);
    } else if (permille < 0) {
        DL_GPIO_clearPins(BSP_BIN1_PORT, BSP_BIN1_PIN);
        DL_GPIO_setPins(BSP_BIN2_PORT, BSP_BIN2_PIN);
    } else {
        DL_GPIO_clearPins(BSP_BIN1_PORT, BSP_BIN1_PIN);
        DL_GPIO_clearPins(BSP_BIN2_PORT, BSP_BIN2_PIN);
    }
}

static void set_pwm_compare(uint32_t channel, uint32_t permille)
{
    uint32_t load_value = DL_TimerA_getLoadValue(PWM_MOTOR_INST);
    uint32_t compare = load_value - ((load_value * permille) / BSP_MOTOR_PWM_MAX_PERMILLE);

    if (compare > load_value) {
        compare = load_value;
    }

    DL_TimerA_setCaptureCompareValue(PWM_MOTOR_INST, compare, channel);
}

static void update_angles(void)
{
    s_left_angle_deg =
        ((float)s_left_count * 360.0f) / (float)BSP_MOTOR_LEFT_COUNTS_PER_OUTPUT_REV;
    s_right_angle_deg =
        ((float)s_right_count * 360.0f) / (float)BSP_MOTOR_RIGHT_COUNTS_PER_OUTPUT_REV;
}

static void handle_right_encoder_edge(void)
{
    uint32_t pins = DL_GPIO_readPins(GPIOA, BSP_ENC_R_A_PIN | BSP_ENC_R_B_PIN);
    bool phase_a = ((pins & BSP_ENC_R_A_PIN) != 0u);
    bool phase_b = ((pins & BSP_ENC_R_B_PIN) != 0u);
    int32_t step = (phase_a != phase_b) ? 1 : -1;

    s_right_count += (step * BSP_MOTOR_RIGHT_SIGN);
}

static void handle_toggle_button(void)
{
    uint32_t now_ms = bsp_systick_get_ms();
    if ((now_ms - s_last_button_ms) < BSP_MOTOR_BTN_DEBOUNCE_MS) {
        return;
    }

    s_last_button_ms = now_ms;
    s_toggle_request = 1u;
}

void bsp_motor_init(void)
{
    s_left_raw_prev = (uint16_t)DL_TimerG_getTimerCount(QEI_LEFT_INST);
    s_left_count = 0;
    s_right_count = 0;
    s_toggle_request = 0u;
    s_last_button_ms = 0u;
    update_angles();

    DL_TimerA_setCaptureCompareValue(PWM_MOTOR_INST, DL_TimerA_getLoadValue(PWM_MOTOR_INST),
        GPIO_PWM_MOTOR_C0_IDX);
    DL_TimerA_setCaptureCompareValue(PWM_MOTOR_INST, DL_TimerA_getLoadValue(PWM_MOTOR_INST),
        GPIO_PWM_MOTOR_C1_IDX);

    set_tb6612_dir_left(0);
    set_tb6612_dir_right(0);
    DL_GPIO_clearPins(BSP_STBY_PORT, BSP_STBY_PIN);

    DL_GPIO_setLowerPinsPolarity(GPIOA, DL_GPIO_PIN_12_EDGE_RISE | DL_GPIO_PIN_12_EDGE_FALL);
    DL_GPIO_setUpperPinsPolarity(GPIOA, DL_GPIO_PIN_18_EDGE_FALL);
    DL_GPIO_clearInterruptStatus(GPIOA, BSP_ENC_R_A_PIN | BSP_START_BTN_PIN);
    DL_GPIO_enableInterrupt(GPIOA, BSP_ENC_R_A_PIN | BSP_START_BTN_PIN);
    NVIC_ClearPendingIRQ(GPIOA_INT_IRQn);
    NVIC_EnableIRQ(GPIOA_INT_IRQn);
}

void bsp_motor_enable(bool enable)
{
    if (enable) {
        DL_GPIO_setPins(BSP_STBY_PORT, BSP_STBY_PIN);
    } else {
        DL_GPIO_clearPins(BSP_STBY_PORT, BSP_STBY_PIN);
    }
}

void bsp_motor_stop(void)
{
    set_tb6612_dir_left(0);
    set_tb6612_dir_right(0);
    set_pwm_compare(GPIO_PWM_MOTOR_C0_IDX, 0u);
    set_pwm_compare(GPIO_PWM_MOTOR_C1_IDX, 0u);
}

void bsp_motor_set_output(int16_t left_permille, int16_t right_permille)
{
    set_tb6612_dir_left(left_permille);
    set_tb6612_dir_right(right_permille);
    set_pwm_compare(GPIO_PWM_MOTOR_C0_IDX, clamp_abs_permille(left_permille));
    set_pwm_compare(GPIO_PWM_MOTOR_C1_IDX, clamp_abs_permille(right_permille));
}

void bsp_motor_update(void)
{
    uint16_t raw_now = (uint16_t)DL_TimerG_getTimerCount(QEI_LEFT_INST);
    int16_t delta = (int16_t)((uint16_t)(raw_now - s_left_raw_prev));

    s_left_raw_prev = raw_now;
    s_left_count += ((int32_t)delta * BSP_MOTOR_LEFT_SIGN);
    update_angles();
}

void bsp_motor_get_feedback(bsp_motor_feedback_t *feedback)
{
    if (feedback == NULL) {
        return;
    }

    __disable_irq();
    feedback->left_count = s_left_count;
    feedback->right_count = s_right_count;
    feedback->left_angle_deg = s_left_angle_deg;
    feedback->right_angle_deg = s_right_angle_deg;
    __enable_irq();
}

bool bsp_motor_consume_toggle_request(void)
{
    bool pending;

    __disable_irq();
    pending = (s_toggle_request != 0u);
    s_toggle_request = 0u;
    __enable_irq();

    return pending;
}

void GPIOA_IRQHandler(void)
{
    DL_GPIO_IIDX pending;

    do {
        pending = DL_GPIO_getPendingInterrupt(GPIOA);

        if (pending == DL_GPIO_IIDX_DIO12) {
            DL_GPIO_clearInterruptStatus(GPIOA, BSP_ENC_R_A_PIN);
            handle_right_encoder_edge();
        } else if (pending == DL_GPIO_IIDX_DIO18) {
            DL_GPIO_clearInterruptStatus(GPIOA, BSP_START_BTN_PIN);
            handle_toggle_button();
        }
    } while (pending != DL_GPIO_IIDX_NO_INTR);
}
