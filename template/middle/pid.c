/**
 * @file    pid.c
 * @brief   通用 PID 实现，详见 pid.h。
 */

#include "pid.h"

#include <stddef.h>

/* -------------------------------------------------------------------------- */
/* 内部辅助                                                                    */
/* -------------------------------------------------------------------------- */

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* -------------------------------------------------------------------------- */
/* 公共 API                                                                    */
/* -------------------------------------------------------------------------- */

void pid_init(pid_t *pid)
{
    if (pid == NULL) return;
    pid->kp = 0.0f;
    pid->ki = 0.0f;
    pid->kd = 0.0f;
    pid->u_min = -1000.0f;
    pid->u_max =  1000.0f;
    pid->i_max = 0.0f;            /* 0 = 跟 u_max 同（动态判断） */
    pid->d_filter_alpha = 0.0f;   /* 默认禁用 D 滤波 */
    pid->i_term = 0.0f;
    pid->prev_meas = 0.0f;
    pid->prev_d_filt = 0.0f;
    pid->has_prev = false;
}

void pid_set_gains(pid_t *pid, float kp, float ki, float kd)
{
    if (pid == NULL) return;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

void pid_set_output_limit(pid_t *pid, float lo, float hi)
{
    if (pid == NULL) return;
    if (lo >= hi) return;
    pid->u_min = lo;
    pid->u_max = hi;
}

void pid_set_integral_limit(pid_t *pid, float i_abs_max)
{
    if (pid == NULL) return;
    if (i_abs_max < 0.0f) i_abs_max = 0.0f;
    pid->i_max = i_abs_max;
}

void pid_set_d_filter(pid_t *pid, float alpha)
{
    if (pid == NULL) return;
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;
    pid->d_filter_alpha = alpha;
}

void pid_reset(pid_t *pid)
{
    if (pid == NULL) return;
    pid->i_term = 0.0f;
    pid->prev_meas = 0.0f;
    pid->prev_d_filt = 0.0f;
    pid->has_prev = false;
}

float pid_step(pid_t *pid, float target, float measured, float dt_sec)
{
    if (pid == NULL) return 0.0f;
    if (dt_sec <= 0.0f) return 0.0f;

    float error = target - measured;

    /* P --------------------------------------------------------------------- */
    float p_term = pid->kp * error;

    /* I ---------------------------------------------------------------------
     * 先试探性累加，回卷在末尾根据"输出是否被限幅 + 误差方向"决定是否撤回。 */
    float i_inc      = pid->ki * error * dt_sec;
    float i_try      = pid->i_term + i_inc;
    float i_abs_max  = (pid->i_max > 0.0f) ? pid->i_max : pid->u_max;
    float i_abs_min  = -i_abs_max;
    float i_clamped  = clampf(i_try, i_abs_min, i_abs_max);

    /* D（on measurement，避免 setpoint 阶跃产生 D 冲击） --------------------- */
    float d_term = 0.0f;
    if (pid->has_prev) {
        float d_meas = (measured - pid->prev_meas) / dt_sec;
        float a      = pid->d_filter_alpha;
        float d_filt = (a > 0.0f) ? (a * d_meas + (1.0f - a) * pid->prev_d_filt)
                                  : d_meas;
        d_term            = -pid->kd * d_filt;
        pid->prev_d_filt  = d_filt;
    } else {
        pid->has_prev    = true;
        pid->prev_d_filt = 0.0f;
    }
    pid->prev_meas = measured;

    /* 合成 + 限幅 + 抗 windup ------------------------------------------------
     * 若 u_raw 已经处于上 / 下限，且本拍 i_inc 还在"继续推过限"那一侧，
     * 则不让 i_term 累加（撤回回到上一拍值）；否则正常更新。 */
    float u_raw = p_term + i_clamped + d_term;
    float u     = clampf(u_raw, pid->u_min, pid->u_max);

    bool saturated_high = (u >= pid->u_max) && (u_raw >= pid->u_max);
    bool saturated_low  = (u <= pid->u_min) && (u_raw <= pid->u_min);

    if ((saturated_high && i_inc > 0.0f) ||
        (saturated_low  && i_inc < 0.0f)) {
        /* 不更新 i_term（保留上一拍值），等价"积分回卷" */
    } else {
        pid->i_term = i_clamped;
    }

    return u;
}

/* ========================================================================== */
/* pid2_t 实现                                                                 */
/* ========================================================================== */

void pid2_init(pid2_t *p)
{
    if (p == NULL) return;
    p->kp  = 0.0f;
    p->ki  = 0.0f;
    p->kd  = 0.0f;
    p->out_min    = -1000.0f;
    p->out_max    =  1000.0f;
    p->i_min      = -1000.0f;
    p->i_max      =  1000.0f;
    p->out_offset = 0.0f;
    p->target     = 0.0f;
    p->actual     = 0.0f;
    p->prev_actual = 0.0f;
    p->out        = 0.0f;
    p->i_term     = 0.0f;
    p->has_prev   = false;
}

void pid2_reset(pid2_t *p)
{
    if (p == NULL) return;
    p->i_term      = 0.0f;
    p->prev_actual = 0.0f;
    p->out         = 0.0f;
    p->has_prev    = false;
}

void pid2_update(pid2_t *p)
{
    if (p == NULL) return;

    float error = p->target - p->actual;

    /* 积分项：Ki=0 时自动清零，防"调参积分债务" */
    if (p->ki != 0.0f) {
        p->i_term += error;
        p->i_term  = clampf(p->i_term, p->i_min, p->i_max);
    } else {
        p->i_term = 0.0f;
    }

    /* 微分先行：作用于 actual 变化而非 error，避免 setpoint 阶跃冲击 */
    float d = p->has_prev ? (p->actual - p->prev_actual) : 0.0f;
    p->prev_actual = p->actual;
    p->has_prev    = true;

    float u = p->kp * error + p->ki * p->i_term - p->kd * d;

    /* 死区补偿：非零输出时叠加固定偏移，突破 TB6612 静摩擦死区 */
    if (u > 0.0f) {
        u += p->out_offset;
    } else if (u < 0.0f) {
        u -= p->out_offset;
    }

    p->out = clampf(u, p->out_min, p->out_max);
}
