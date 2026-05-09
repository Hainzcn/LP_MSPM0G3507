/**
 * @file    pid.h
 * @brief   通用浮点 PID 控制器（位置式 + 抗积分饱和 + 微分项独立滤波）
 *
 * 选型理由：
 *   ─ **位置式 PID**：相比增量式，输出值直接对应控制量（如电机 PWM permille），
 *     便于"零命令复位"与"上限钳位"。增量式适合积分输出量（如步进电机角度），
 *     本工程平衡环 / 速度环的命令是绝对量，位置式更直观。
 *   ─ **抗积分饱和（积分回卷）**：当输出已经被钳位时，积分项停止累加，
 *     避免饱和退出后超调。
 *   ─ **微分项独立 dt**：直接传 dt（秒），不假设固定调用周期，让同一个
 *     控制器能在 100 Hz / 200 Hz / 500 Hz 之间灵活切换。
 *   ─ **微分项可选低通**：对 setpoint 阶跃有冲击的场景，提供 D 项 EMA 滤波；
 *     coefficient = 0 时禁用滤波，等价"裸 D"。
 *
 * 使用约定：
 *   pid_t pid;
 *   pid_init(&pid);
 *   pid_set_gains(&pid, 1.5f, 0.05f, 0.10f);
 *   pid_set_output_limit(&pid, -1000.0f, 1000.0f);
 *   pid_set_integral_limit(&pid, 500.0f);
 *   ...
 *   for (;;) {
 *       float u = pid_step(&pid, target, measured, dt_sec);
 *       motor_set_output(u);
 *   }
 *
 *   切换 setpoint 阶跃前请调 `pid_reset(&pid)` 清积分与微分历史，
 *   避免"上一段累积的 i_term 让新阶跃产生超调"。
 */

#ifndef MIDDLE_PID_H
#define MIDDLE_PID_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /* 增益（业务侧通过 pid_set_gains 设置；默认 0 = 控制器输出 0，安全） */
    float kp;
    float ki;
    float kd;

    /* 输出限幅 [u_min, u_max]：默认 ±1000（permille 量纲） */
    float u_min;
    float u_max;

    /* 积分项绝对值上限（抗积分饱和，独立于输出限幅）；默认 0 = 跟 u_max 同 */
    float i_max;

    /* 微分项 EMA 系数 [0, 1]；0 = 禁用滤波；默认 0 */
    float d_filter_alpha;

    /* 内部状态（业务侧不要直接写） */
    float i_term;
    float prev_meas;        /* 用 d on measurement 而不是 d on error，避免 setpoint 阶跃冲击 */
    float prev_d_filt;
    bool  has_prev;
} pid_t;

/**
 * @brief 初始化为安全默认值：增益全 0、输出 ±1000、积分 ±0（即跟 u_max 同）、
 *        D 滤波禁用、内部状态清零。
 *
 *  默认增益 = 0 是有意为之：未整定的 PID 不会输出任何控制量，比夹带魔法数字
 *  更安全；业务侧必须显式 `pid_set_gains` 才有输出。
 */
void pid_init(pid_t *pid);

/** 设置 KP / KI / KD（任何值；负值会被原样接受，调用方自负责）。 */
void pid_set_gains(pid_t *pid, float kp, float ki, float kd);

/** 设置输出限幅 [lo, hi]（lo 必须 < hi，否则被忽略）。 */
void pid_set_output_limit(pid_t *pid, float lo, float hi);

/**
 * @brief 设置积分项独立上限（抗积分饱和）。
 * @param i_abs_max  绝对值上限（≥ 0）；传 0 表示"跟 u_max 同"，自动按比例。
 */
void pid_set_integral_limit(pid_t *pid, float i_abs_max);

/**
 * @brief 设置微分项 EMA 滤波系数。
 * @param alpha  [0, 1]；0 = 禁用，1 = 完全跟随；典型 0.1~0.3 抑制高频噪声。
 */
void pid_set_d_filter(pid_t *pid, float alpha);

/** 复位内部状态（积分项 + 微分历史），不动增益与限幅。 */
void pid_reset(pid_t *pid);

/**
 * @brief 跑一拍 PID 计算并返回输出值（已限幅）。
 *
 * @param pid       控制器实例
 * @param target    目标值（setpoint）
 * @param measured  测量值
 * @param dt_sec    距上一拍的时间间隔（秒）；首次调用 dt 任意（D 项被首拍跳过）
 * @return          控制量输出，落在 [u_min, u_max] 区间
 *
 *  公式（"d on measurement"，避免 setpoint 阶跃冲击）：
 *    error  = target - measured
 *    p_term = kp * error
 *    i_term += ki * error * dt
 *    i_term = clamp(i_term, -i_max, +i_max)
 *    d_meas = (measured - prev_meas) / dt
 *    d_filt = α * d_meas + (1 - α) * prev_d_filt
 *    d_term = -kd * d_filt
 *    u_raw  = p_term + i_term + d_term
 *    u      = clamp(u_raw, u_min, u_max)
 *    若 u_raw 被限幅 → 回退本拍 i_term 增量（抗 windup）
 */
float pid_step(pid_t *pid, float target, float measured, float dt_sec);

#ifdef __cplusplus
}
#endif

#endif /* MIDDLE_PID_H */
