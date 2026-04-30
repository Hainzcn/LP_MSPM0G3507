/**
 * @file    att_filter.h
 * @brief   俯仰角互补滤波（pitch-only）。
 *
 * 假定的车体坐标系（**装机前必须复核**，搞反整车直接朝反方向倒）：
 *   - X 轴：朝车头（前进方向为正）
 *   - Y 轴：朝车体右侧
 *   - Z 轴：朝车顶（与重力反向为正）
 *   - pitch：绕 Y 轴旋转的角度，车头上扬为正。
 *
 * 算法：
 *   pitch_acc = atan2(-ax, sqrt(ay^2 + az^2)) * 180/pi
 *   pitch     = α * (pitch_prev + gy * dt) + (1-α) * pitch_acc
 *
 * 参数：
 *   - 默认 α = 0.98（陀螺主导，加速度只做低频校正），调试期可在文件顶部
 *     宏中调；α 越大对加速度噪声越不敏感，但对陀螺漂移也越敏感。
 *   - dt 由调用方提供，单位秒（典型 1e-3 = 1 kHz）。
 */

#ifndef ATT_FILTER_H
#define ATT_FILTER_H

#include <stdint.h>
#include "mpu6050.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float pitch_deg;        /* 互补滤波后的俯仰角 (°) */
    float pitch_rate_dps;   /* 当前俯仰角速度 (°/s)   */
    float pitch_acc_deg;    /* 仅用加速度算的瞬时俯仰，用作收敛对照 */
    float temp_c;           /* MPU6050 内温度 (°C)，便于诊断热漂 */
} att_state_t;

/** 复位滤波内部状态（pitch 累积清零）。 */
void att_filter_init(void);

/**
 * @brief  以 raw 原始读数推进一拍滤波。
 * @param  raw   一次 mpu6050_read_raw 的结果。
 * @param  dt_s  距离上次调用的时间间隔（秒）。
 * @param  out   输出当前姿态。
 */
void att_filter_update(const mpu6050_raw_t *raw, float dt_s,
                       att_state_t *out);

#ifdef __cplusplus
}
#endif

#endif /* ATT_FILTER_H */
