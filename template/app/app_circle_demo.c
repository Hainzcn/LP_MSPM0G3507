/**
 * @file    app_circle_demo.c
 * @brief   顺时针绕圆演示实现。
 *
 * 控制环路（20 Hz，由外部在 balance_step_speed 前调用）：
 *
 *   ①  累积旋转量：accumulated_deg += gz_dps × CW_GZ_SIGN × dt_s
 *       顺时针为正，用于退出判断。
 *
 *   ②  角速度 PI 控制器（跟踪目标转率 ω_target = v/R）：
 *         error      = omega_target_dps − gz_dps × CW_GZ_SIGN
 *         i_term    += error × dt_s（积分钳位）
 *         yaw_pm     = KP × error + KI × i_term
 *         yaw_pm     = clamp(yaw_pm, 0, YAW_MAX_CORRECTION_PM)
 *       乘以 APP_CIRCLE_CW_SIGN 得到最终 target_yaw_pm（正值 = 顺时针差速）。
 *
 *   ③  线性减速：
 *         剩余角度 = 360° − accumulated_deg
 *         若剩余角度 < DECEL_MARGIN：
 *           speed_scale = clamp(剩余/DECEL_MARGIN, 0, 1)
 *           target_speed = SPEED_MPS × speed_scale
 *         若剩余角度 < STOP_MARGIN：演示结束，清零 cmd。
 *
 * 注意：本模块不直接控制左右轮 PWM，而是写入 app_balance_motion_cmd_t；
 *       平衡环的速度外环和角度环继续处理姿态控制，保证小车在绕圈过程中
 *       始终保持直立。
 */

#include "app_circle_demo.h"
#include "robot_param.h"

#include <stdio.h>
#include <stdint.h>

/* 浮点格式化辅助（避开 AC6 printf("%f") 浮点路径，
 * 与 app_balance.c 的 BAL_F2_* 同源策略；'.1' 表示保留 1 位小数）。 */
#define CIRC_F1_X10(v)  ((int32_t)((v) * 10.0f + ((v) >= 0.0f ? 0.5f : -0.5f)))
#define CIRC_F1_S(v)    (CIRC_F1_X10(v) < 0 ? '-' : ' ')
#define CIRC_F1_I(v)    ((CIRC_F1_X10(v) < 0 ? -CIRC_F1_X10(v) : CIRC_F1_X10(v)) / 10)
#define CIRC_F1_F(v)    ((uint32_t)((CIRC_F1_X10(v) < 0 ? -CIRC_F1_X10(v) : CIRC_F1_X10(v)) % 10))

/* ========================================================================== */
/* 内部常量                                                                     */
/* ========================================================================== */

/** 期望偏航角速率（°/s）= v / R × (180/π)。编译时计算。 */
#define CIRCLE_OMEGA_TARGET_DPS  \
    (APP_CIRCLE_SPEED_MPS / APP_CIRCLE_RADIUS_M * (180.0f / ROBOT_PI))

/** yaw_pm 上限（使用航向差速最大修正量）。 */
#define CIRCLE_YAW_PM_MAX        ((float)APP_BALANCE_YAW_MAX_CORRECTION_PM)

/** 简单 float clamp。 */
#define F_CLAMP(x, lo, hi)  ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))

/* ========================================================================== */
/* 内部状态                                                                     */
/* ========================================================================== */

typedef struct {
    circle_demo_state_t state;
    float    accumulated_deg;   /* 累积顺时针旋转量（°） */
    float    omega_i_term;      /* PI 控制器积分项（permille）*/
    uint32_t start_ms;          /* 绕圈启动时刻，用于超时保护 */

    /* PENDING 倒计时 */
    uint32_t pending_end_ms;    /* 倒计时到期时刻 */
    bool     needs_launch;      /* 到期后置 true，直到 launch() 被调用 */
} circle_demo_t;

static circle_demo_t s_cd;

/* ========================================================================== */
/* 内部辅助函数                                                                 */
/* ========================================================================== */

/** 重置绕圈运行状态（不改变 state、pending_end_ms、needs_launch）。 */
static void reset_run_state(void)
{
    s_cd.accumulated_deg = 0.0f;
    s_cd.omega_i_term    = 0.0f;
    s_cd.start_ms        = 0u;
}

/** 将 cmd 的速度/转向字段清零。 */
static void zero_cmd(app_balance_motion_cmd_t *cmd)
{
    if (cmd == NULL) { return; }
    cmd->target_speed_cps = 0;
    cmd->target_yaw_pm    = 0;
}

/* ========================================================================== */
/* 公开 API                                                                    */
/* ========================================================================== */

void app_circle_demo_init(void)
{
    s_cd.state           = CIRCLE_DEMO_IDLE;
    s_cd.pending_end_ms  = 0u;
    s_cd.needs_launch    = false;
    reset_run_state();
}

void app_circle_demo_start_pending(uint32_t delay_ms, uint32_t now_ms)
{
    reset_run_state();
    s_cd.pending_end_ms = now_ms + delay_ms;
    s_cd.needs_launch   = false;
    s_cd.state          = CIRCLE_DEMO_PENDING;
    (void)printf("[circle] pending: motors off, moving to open area... "
                 "auto-start in %lu s\r\n", (unsigned long)(delay_ms / 1000u));
}

void app_circle_demo_start(void)
{
    reset_run_state();
    s_cd.needs_launch = false;
    s_cd.state        = CIRCLE_DEMO_RUNNING;
    (void)printf("[circle] start: R=%dmm v=%dcm/s omega_target=%c%ld.%lu dps\r\n",
                 (int)(APP_CIRCLE_RADIUS_M * 1000.0f),
                 (int)(APP_CIRCLE_SPEED_MPS * 100.0f),
                 CIRC_F1_S(CIRCLE_OMEGA_TARGET_DPS),
                 (long)CIRC_F1_I(CIRCLE_OMEGA_TARGET_DPS),
                 (unsigned long)CIRC_F1_F(CIRCLE_OMEGA_TARGET_DPS));
}

void app_circle_demo_stop(app_balance_motion_cmd_t *cmd)
{
    s_cd.state        = CIRCLE_DEMO_IDLE;
    s_cd.needs_launch = false;
    reset_run_state();
    zero_cmd(cmd);
    (void)printf("[circle] stopped\r\n");
}

bool app_circle_demo_is_active(void)
{
    return (s_cd.state == CIRCLE_DEMO_RUNNING) ||
           (s_cd.state == CIRCLE_DEMO_DECELING);
}

circle_demo_state_t app_circle_demo_get_state(void)
{
    return s_cd.state;
}

float app_circle_demo_get_accumulated_deg(void)
{
    return s_cd.accumulated_deg;
}

uint32_t app_circle_demo_get_pending_remaining_ms(uint32_t now_ms)
{
    if (s_cd.state != CIRCLE_DEMO_PENDING) { return 0u; }
    if ((int32_t)(s_cd.pending_end_ms - now_ms) <= 0) { return 0u; }
    return s_cd.pending_end_ms - now_ms;
}

bool app_circle_demo_needs_launch(void)
{
    return s_cd.needs_launch;
}

void app_circle_demo_launch(void)
{
    s_cd.needs_launch = false;
    reset_run_state();
    s_cd.state = CIRCLE_DEMO_RUNNING;
    (void)printf("[circle] launch: R=%dmm v=%dcm/s omega_target=%c%ld.%lu dps\r\n",
                 (int)(APP_CIRCLE_RADIUS_M * 1000.0f),
                 (int)(APP_CIRCLE_SPEED_MPS * 100.0f),
                 CIRC_F1_S(CIRCLE_OMEGA_TARGET_DPS),
                 (long)CIRC_F1_I(CIRCLE_OMEGA_TARGET_DPS),
                 (unsigned long)CIRC_F1_F(CIRCLE_OMEGA_TARGET_DPS));
}

void app_circle_demo_update(app_balance_motion_cmd_t *cmd,
                             float gz_dps, float dt_s, uint32_t now_ms)
{
    /* PENDING：倒计时，不注入命令 */
    if (s_cd.state == CIRCLE_DEMO_PENDING) {
        if ((int32_t)(s_cd.pending_end_ms - now_ms) <= 0) {
            /* 倒计时到期：设置标志，由外部执行重校零后调用 launch() */
            s_cd.needs_launch = true;
            (void)printf("[circle] countdown done, please stand upright...\r\n");
        }
        /* PENDING 期间不修改 cmd，电机由外部（ci 命令处）已物理关闭 */
        (void)gz_dps; (void)dt_s; (void)cmd;
        return;
    }

    if ((s_cd.state == CIRCLE_DEMO_IDLE) || (s_cd.state == CIRCLE_DEMO_DONE)) {
        return;
    }
    if (cmd == NULL) { return; }

    /* 记录启动时刻 */
    if (s_cd.start_ms == 0u) {
        s_cd.start_ms = now_ms;
    }

    /* ── 超时保护 ───────────────────────────────────────────────── */
    if ((APP_CIRCLE_TIMEOUT_MS > 0u) &&
        ((now_ms - s_cd.start_ms) >= APP_CIRCLE_TIMEOUT_MS)) {
        (void)printf("[circle] timeout after %lu ms, acc=%c%ld.%lu deg\r\n",
                     (unsigned long)(now_ms - s_cd.start_ms),
                     CIRC_F1_S(s_cd.accumulated_deg),
                     (long)CIRC_F1_I(s_cd.accumulated_deg),
                     (unsigned long)CIRC_F1_F(s_cd.accumulated_deg));
        s_cd.state = CIRCLE_DEMO_DONE;
        zero_cmd(cmd);
        return;
    }

    /* ── 累积旋转量（顺时针为正）─────────────────────────────────── */
    float gz_cw = gz_dps * (float)APP_CIRCLE_CW_GZ_SIGN;
    s_cd.accumulated_deg += gz_cw * dt_s;

    /* 防止负值（启动瞬间 gz 噪声）拉低累积量 */
    if (s_cd.accumulated_deg < 0.0f) {
        s_cd.accumulated_deg = 0.0f;
    }

    /* ── 退出判断 ────────────────────────────────────────────────── */
    float remaining_deg = APP_CIRCLE_TARGET_DEG - s_cd.accumulated_deg;

    if (remaining_deg <= APP_CIRCLE_STOP_MARGIN_DEG) {
        (void)printf("[circle] done: acc=%c%ld.%lu deg elapsed=%lu ms\r\n",
                     CIRC_F1_S(s_cd.accumulated_deg),
                     (long)CIRC_F1_I(s_cd.accumulated_deg),
                     (unsigned long)CIRC_F1_F(s_cd.accumulated_deg),
                     (unsigned long)(now_ms - s_cd.start_ms));
        s_cd.state = CIRCLE_DEMO_DONE;
        zero_cmd(cmd);
        return;
    }

    /* ── 减速状态切换 ────────────────────────────────────────────── */
    if (remaining_deg <= APP_CIRCLE_DECEL_MARGIN_DEG) {
        s_cd.state = CIRCLE_DEMO_DECELING;
    } else {
        s_cd.state = CIRCLE_DEMO_RUNNING;
    }

    /* ── 角速度 PI 控制器 ────────────────────────────────────────── */
    /*
     * 误差 = 目标角速率 − 实测顺时针角速率
     * 目标：CIRCLE_OMEGA_TARGET_DPS（始终正值）
     * 实测：gz_cw（顺时针为正）
     */
    float omega_error = CIRCLE_OMEGA_TARGET_DPS - gz_cw;

    /* 积分（含抗饱和钳位） */
    s_cd.omega_i_term += omega_error * dt_s;
    s_cd.omega_i_term  = F_CLAMP(s_cd.omega_i_term,
                                 -APP_CIRCLE_OMEGA_I_LIMIT_PM,
                                  APP_CIRCLE_OMEGA_I_LIMIT_PM);

    float yaw_pm_raw = (APP_CIRCLE_OMEGA_KP * omega_error) +
                       (APP_CIRCLE_OMEGA_KI * s_cd.omega_i_term);

    /* 只允许正值（不允许反向修正，避免绕圈方向倒转） */
    yaw_pm_raw = F_CLAMP(yaw_pm_raw, 0.0f, CIRCLE_YAW_PM_MAX);

    /* ── 速度计算（减速阶段线性缩减）───────────────────────────────── */
    float speed_scale = 1.0f;
    if (s_cd.state == CIRCLE_DEMO_DECELING) {
        speed_scale = (remaining_deg - APP_CIRCLE_STOP_MARGIN_DEG) /
                      (APP_CIRCLE_DECEL_MARGIN_DEG - APP_CIRCLE_STOP_MARGIN_DEG);
        speed_scale = F_CLAMP(speed_scale, 0.0f, 1.0f);

        /* 减速时角速度目标也同比降低，避免小车原地继续高速转 */
        yaw_pm_raw *= speed_scale;
    }

    /* ── 写入运动指令 ────────────────────────────────────────────── */
    float target_v_mps = APP_CIRCLE_SPEED_MPS * speed_scale;
    cmd->target_speed_cps = ROBOT_MPS_TO_EQ_CPS(target_v_mps);
    cmd->target_yaw_pm    = (int16_t)((float)APP_CIRCLE_CW_SIGN * yaw_pm_raw);
}
