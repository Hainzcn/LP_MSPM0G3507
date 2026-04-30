/**
 * @file    att_filter.c
 * @brief   俯仰互补滤波实现，详见 att_filter.h。
 */

#include "att_filter.h"
#include <math.h>
#include <stddef.h>

#ifndef ATT_FILTER_ALPHA
#define ATT_FILTER_ALPHA   0.98f
#endif

#define RAD2DEG            57.2957795131f

static float s_pitch_deg = 0.0f;
static int   s_inited    = 0;

void att_filter_init(void)
{
    s_pitch_deg = 0.0f;
    s_inited    = 0;
}

void att_filter_update(const mpu6050_raw_t *raw, float dt_s,
                       att_state_t *out)
{
    if (raw == NULL || out == NULL) {
        return;
    }

    /* 1) 单位换算：raw → 物理量 */
    float ax = (float)raw->accel_x / MPU6050_ACCEL_LSB_PER_G;
    float ay = (float)raw->accel_y / MPU6050_ACCEL_LSB_PER_G;
    float az = (float)raw->accel_z / MPU6050_ACCEL_LSB_PER_G;
    float gy = (float)raw->gyro_y  / MPU6050_GYRO_LSB_PER_DPS;
    float tc = (float)raw->temp / MPU6050_TEMP_LSB_PER_C
             + MPU6050_TEMP_OFFSET;

    /* 2) 加速度俯仰：车体 X 朝前 → ax 受重力分量 = -sin(pitch)
     *    pitch_acc = atan2(-ax, sqrt(ay^2 + az^2))
     *    若装机方向不同，调整 ax 符号或换轴；先在调试期用 pitch_acc 单独
     *    比对水平 / 抬头 / 低头三种姿态确认极性。 */
    float denom = sqrtf(ay * ay + az * az);
    float pitch_acc = atan2f(-ax, denom) * RAD2DEG;

    /* 3) 首拍直接吃加速度结果，避免冷启动时陀螺积分从 0 慢慢爬过去 */
    if (!s_inited) {
        s_pitch_deg = pitch_acc;
        s_inited    = 1;
    } else {
        /* 4) 互补滤波 */
        float pitch_gyr = s_pitch_deg + gy * dt_s;
        s_pitch_deg = ATT_FILTER_ALPHA * pitch_gyr
                    + (1.0f - ATT_FILTER_ALPHA) * pitch_acc;
    }

    out->pitch_deg      = s_pitch_deg;
    out->pitch_rate_dps = gy;
    out->pitch_acc_deg  = pitch_acc;
    out->temp_c         = tc;
}
