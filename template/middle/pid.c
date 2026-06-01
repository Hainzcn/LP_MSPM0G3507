/**
 * @file    pid.c
 * @brief   平衡车 PID 实现，详见 pid.h。
 */

#include "pid.h"

#include <stddef.h>

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

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
    p->freeze_integral = false;
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

    /* 积分项：Ki=0 时自动清零，防"调参积分债务"。
     * freeze_integral 时保留现值不累加（反馈临时不可信场景），仍参与输出。 */
    if (p->ki != 0.0f) {
        if (!p->freeze_integral) {
            p->i_term += error;
            p->i_term  = clampf(p->i_term, p->i_min, p->i_max);
        }
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
