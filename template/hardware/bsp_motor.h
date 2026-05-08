/**
 * @file    bsp_motor.h
 * @brief   阶段 2 电机底层驱动：TB6612 + 左右轮编码器反馈。
 *
 * 左轮：TIMG8 硬件 QEI（16-bit 计数，软件扩展为 32-bit）。
 * 右轮：PA12 双边沿中断 + PA13 电平判方向（X2 解码，CPU 负担更低）。
 *
 * 说明：
 *   - 速度命令接口使用 `permille`（千分比），范围 [-1000, 1000]。
 *   - 角度默认按 GB370 常见 11 PPR、30:1 减速比估算；若实物参数不同，
 *     只需改本头文件顶部的编码器宏。
 */

#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BSP_MOTOR_PWM_MAX_PERMILLE                 (1000)
#define BSP_MOTOR_GB370_GEAR_RATIO                 (30)
#define BSP_MOTOR_GB370_HALL_PPR                   (11)
#define BSP_MOTOR_LEFT_COUNTS_PER_OUTPUT_REV \
    (BSP_MOTOR_GB370_GEAR_RATIO * BSP_MOTOR_GB370_HALL_PPR * 4)
#define BSP_MOTOR_RIGHT_COUNTS_PER_OUTPUT_REV \
    (BSP_MOTOR_GB370_GEAR_RATIO * BSP_MOTOR_GB370_HALL_PPR * 2)

typedef struct {
    int32_t left_count;
    int32_t right_count;
    float   left_angle_deg;
    float   right_angle_deg;
} bsp_motor_feedback_t;

void bsp_motor_init(void);
void bsp_motor_enable(bool enable);
void bsp_motor_stop(void);
void bsp_motor_set_output(int16_t left_permille, int16_t right_permille);
void bsp_motor_update(void);
void bsp_motor_get_feedback(bsp_motor_feedback_t *feedback);
bool bsp_motor_consume_toggle_request(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_MOTOR_H */
