/**
 * @file    pid.h
 * @brief   平衡车专用浮点 PID（对齐 STM32 demo 公式）
 *
 * 特性：
 *   ─ **位置式 PID**：输出直接对应 PWM permille，便于零命令复位与上限钳位。
 *   ─ **Ki=0 自动清零 i_term**：调试时避免"积分债务"。
 *   ─ **out_offset 死区补偿**：非零输出时叠加固定偏移，突破 TB6612 静摩擦死区。
 *   ─ **微分先行（d on measurement）**：Kd 作用于 actual 变化率，避免 setpoint 阶跃冲击。
 *   ─ **i_min/i_max 独立限幅**：防 wind-up，积分范围可与输出限幅分离。
 *
 * 使用约定（固定调用周期，增益已吸收 dt 因子）：
 *   pid2_t pid;
 *   pid2_init(&pid);
 *   pid.kp = 1.5f; pid.ki = 0.05f; pid.kd = 0.10f;
 *   pid.out_min = -1000.0f; pid.out_max = 1000.0f;
 *   ...
 *   for (;;) {
 *       pid.target = setpoint;
 *       pid.actual = measured;
 *       pid2_update(&pid);
 *       motor_set_output(pid.out);
 *   }
 *
 *   切换 setpoint 阶跃前请调 `pid2_reset(&pid)` 清积分与微分历史。
 */

#ifndef MIDDLE_PID_H
#define MIDDLE_PID_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /* 增益（默认 0 = 安全输出 0，必须显式设置才有控制作用） */
    float kp;
    float ki;
    float kd;

    /* 输出限幅 [out_min, out_max] */
    float out_min;
    float out_max;

    /* ErrorInt（i_term）独立限幅，防积分 wind-up */
    float i_min;
    float i_max;

    /* 死区补偿：非零输出时在 |u| 上叠加此偏移，突破电机静摩擦死区。
     * 典型值：TB6612 + N20 电机约 60~100 permille；调试期先从 0 开始逐步加。 */
    float out_offset;

    /* 积分冻结：true 时本拍跳过 i_term 累加（保留现值参与输出），仍计算
     * 比例/微分。用于"反馈临时不可信"场景（如加速度门控）只冻结积分，
     * 而不冻结整条环路，避免释放时输出跳变。默认 false。 */
    bool  freeze_integral;

    /* 内部状态（业务侧不要直接写） */
    float target;
    float actual;
    float prev_actual;
    float out;
    float i_term;
    bool  has_prev;
} pid2_t;

/** 初始化为安全默认：增益全 0，限幅 ±1000，i_min/i_max ±1000，out_offset 0，状态清零。 */
void pid2_init(pid2_t *p);

/** 复位内部状态（i_term / prev_actual / has_prev），不动增益与限幅。 */
void pid2_reset(pid2_t *p);

/**
 * @brief 跑一拍 PID 计算，结果写入 p->out。
 *
 * 调用方在调用前设置 p->target / p->actual，调用后读 p->out。
 *
 * 公式：
 *   error  = target - actual
 *   if (ki != 0): i_term += error; clamp(i_term, i_min, i_max)
 *   else:         i_term = 0
 *   d      = actual - prev_actual   (首拍为 0)
 *   u_raw  = kp*error + ki*i_term - kd*d
 *   if (u_raw > 0): u_raw += out_offset
 *   if (u_raw < 0): u_raw -= out_offset
 *   out    = clamp(u_raw, out_min, out_max)
 */
void pid2_update(pid2_t *p);

#ifdef __cplusplus
}
#endif

#endif /* MIDDLE_PID_H */
