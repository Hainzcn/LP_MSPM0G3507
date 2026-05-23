/**
 * @file    app_circle_demo.h
 * @brief   圆弧运动演示子任务（寄生在 app_balance 主循环 20 Hz 分支）。
 *
 * 功能：以指定线速度绕指定直径圆做整圈运动，一圈自动停止。
 *
 * 串口触发：
 *   `c`  / `circle`              → 默认参数启动（800 mm / 200 mm/s / 顺时针）
 *   `circle <diam_mm> <v_mm_s>`  → 自定义直径与速度
 *   `cx`                          → 中止当前圆运动
 *
 * 完成判据（任一满足即停止）：
 *   ① IMU gz_dps 积分偏航 ≥ 360°（主判据）
 *   ② 编码器弧长 ≥ 圆周长 × 1.2（备份）
 *   ③ 运行时间 ≥ 期望时间 × 3（兜底超时）
 */

#ifndef APP_CIRCLE_DEMO_H
#define APP_CIRCLE_DEMO_H

#include "app_balance.h"
#include "bsp_motor.h"
#include "ms901m.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/* 编译期可配宏                                                                 */
/* ========================================================================== */

#ifndef APP_CIRCLE_DEFAULT_DIAMETER_MM
#define APP_CIRCLE_DEFAULT_DIAMETER_MM      (800)
#endif

#ifndef APP_CIRCLE_DEFAULT_SPEED_MM_S
#define APP_CIRCLE_DEFAULT_SPEED_MM_S       (200)
#endif

#ifndef APP_CIRCLE_DEFAULT_CLOCKWISE
#define APP_CIRCLE_DEFAULT_CLOCKWISE        (1)
#endif

/**
 * 开环转向增益：每 rad/s 角速度需要的 PWM permille 差分 × 100。
 *
 * 物理含义：robot 以 omega rad/s 转弯时，左右轮需
 *   delta_PWM = omega × GAIN_X100 / 100  permille 差分。
 *
 * 初始经验值 12000 = 120 permille / (rad/s)，上车标定后替换：
 *   1. 发 `c` 跑默认圆，观察一圈终止时车体是否回到原朝向。
 *   2. 偏小（不够一圈）→ 加大此值；偏大（超过一圈）→ 减小此值。
 */
#ifndef APP_CIRCLE_OPEN_YAW_PM_PER_RAD_S_X100
#define APP_CIRCLE_OPEN_YAW_PM_PER_RAD_S_X100  (12000)
#endif

/** 弧长备份判据：弧长超过圆周长的此倍率即判停（×100 避浮点，120 = 1.20 倍）。 */
#ifndef APP_CIRCLE_ARC_OVERSHOOT_X100
#define APP_CIRCLE_ARC_OVERSHOOT_X100       (120)
#endif

/** 超时兜底倍率（×100，300 = 3.00 倍期望时间）。 */
#ifndef APP_CIRCLE_TIMEOUT_FACTOR_X100
#define APP_CIRCLE_TIMEOUT_FACTOR_X100      (300)
#endif

/* ========================================================================== */
/* 诊断快照                                                                     */
/* ========================================================================== */

typedef struct {
    bool    active;
    float   yaw_accum_deg;
    int32_t arc_mm;
    uint32_t elapsed_ms;
} app_circle_diag_t;

/* ========================================================================== */
/* API                                                                         */
/* ========================================================================== */

/**
 * @brief 启动圆运动演示。
 * @param diameter_mm  圆直径（mm），例如 800
 * @param v_mm_s       中心线速度（mm/s，正 = 前进），例如 200
 * @param clockwise    true = 顺时针（俯视），false = 逆时针
 */
void app_circle_demo_start(uint16_t diameter_mm, int16_t v_mm_s, bool clockwise);

/** 中止当前圆运动，清空 cmd 输出，回到 IDLE。 */
void app_circle_demo_cancel(void);

/** @return true = 圆运动正在运行中。 */
bool app_circle_demo_is_active(void);

/** 拷贝一份诊断快照（不影响内部状态）。 */
void app_circle_demo_get_diag(app_circle_diag_t *out);

/**
 * @brief 20 Hz 调度入口，由 app_balance_run 主循环在速度环后调用。
 *
 * 激活期间会覆盖 cmd->target_speed_cps 和 cmd->target_yaw_pm。
 * 非激活时不修改 cmd。
 *
 * @param snap  最新 IMU 快照（gz_dps 用于偏航积分）
 * @param fb    最新编码器反馈（left/right_count 用于弧长计算）
 * @param cmd   运动指令结构体（可能被覆盖）
 */
void app_circle_demo_tick_20hz(const ms901m_snapshot_t *snap,
                                const bsp_motor_feedback_t *fb,
                                app_balance_motion_cmd_t *cmd);

#ifdef __cplusplus
}
#endif

#endif /* APP_CIRCLE_DEMO_H */
