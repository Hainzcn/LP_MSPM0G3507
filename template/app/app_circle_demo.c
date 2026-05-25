/**
 * @file    app_circle_demo.c
 * @brief   圆弧运动演示实现，详见 app_circle_demo.h。
 */

#include "app_circle_demo.h"

#include "robot_param.h"
#include "bsp_systick.h"

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/* ── 浮点格式化辅助（复用 app_balance 风格，避免 %f） ─────────────────────── */
#define CIR_F2_X100(v) ((int32_t)((v) * 100.0f + ((v) >= 0.0f ? 0.5f : -0.5f)))
#define CIR_F2_S(v)    (CIR_F2_X100(v) < 0 ? '-' : '+')
#define CIR_F2_I(v)    ((int32_t)((CIR_F2_X100(v) < 0 ? -CIR_F2_X100(v) : CIR_F2_X100(v)) / 100))
#define CIR_F2_F(v)    ((uint32_t)((CIR_F2_X100(v) < 0 ? -CIR_F2_X100(v) : CIR_F2_X100(v)) % 100))

/* ── 内部状态 ─────────────────────────────────────────────────────────────── */

typedef enum {
    CIRCLE_IDLE    = 0,
    CIRCLE_RUNNING = 1,
} circle_phase_t;

typedef struct {
    circle_phase_t phase;

    /* 运动参数（start 时计算，运行期间只读） */
    int32_t  target_avg_cps;
    int32_t  target_dif_cps;
    int32_t  circumference_mm;
    uint32_t expected_ms;

    /* 判停累计量 */
    float    yaw_accum_deg;
    int32_t  start_left_count;
    int32_t  start_right_count;
    int32_t  arc_mm;
    uint32_t start_ms;
    uint32_t elapsed_ms;

    /* 方向 */
    int8_t   yaw_sign;          /* +1 顺时针, -1 逆时针 */
} circle_state_t;

static circle_state_t s_cir;

/* ── 实现 ─────────────────────────────────────────────────────────────────── */

void app_circle_demo_start(uint16_t diameter_mm, int16_t v_mm_s, bool clockwise)
{
    if (s_cir.phase != CIRCLE_IDLE) {
        (void)printf("[circle] busy, cancel first\r\n");
        return;
    }

    if (diameter_mm == 0u || v_mm_s == 0) {
        (void)printf("[circle] invalid param diam=%u v=%d\r\n",
            (unsigned int)diameter_mm, (int)v_mm_s);
        return;
    }

    /* 圆周长 = π × diameter */
    int32_t circ_mm = (int32_t)(3.14159265f * (float)diameter_mm);
    s_cir.circumference_mm = circ_mm;

    /* 期望完成时间 = 周长 / 速度 */
    int32_t abs_v = (v_mm_s > 0) ? v_mm_s : -v_mm_s;
    s_cir.expected_ms = (abs_v > 0) ? (uint32_t)((circ_mm * 1000L) / abs_v) : 60000u;

    /* 中心线速度 → avg_cps */
    s_cir.target_avg_cps = robot_v_mm_s_to_avg_cps((int32_t)v_mm_s);

    /* 角速度 omega = 2v / diameter (rad/s)
     * 差速 = omega × wheel_base (mm/s) → 转 cps 再归一化
     * omega_mrad_s = omega × 1000 = 2000 × abs_v / diameter
     */
    int32_t omega_mrad_s = (int32_t)((2000LL * (int64_t)abs_v) / (int64_t)diameter_mm);

    s_cir.yaw_sign = clockwise ? (int8_t)1 : (int8_t)-1;
    int32_t dif_cps_raw = robot_omega_mrad_to_delta_cps(omega_mrad_s);
    int32_t dif_cps_norm = dif_cps_raw / (int32_t)APP_BALANCE_SPEED_CPS_SCALE;

    /* target_dif_cps 约定：正值 = 代数意义的 L > R（前进时左快，倒退时左慢）。
     * 倒退 CW 时外圈轮需要更快倒退（代数更小），对应 L-R < 0，需乘以 v 的符号。
     * 前进时 v_sign=+1 不变；倒退时 v_sign=-1 翻转差速方向。 */
    int8_t v_sign = (v_mm_s >= 0) ? (int8_t)1 : (int8_t)-1;
    s_cir.target_dif_cps = dif_cps_norm * (int32_t)s_cir.yaw_sign
                                        * (int32_t)v_sign;

    /* 记录起始编码器计数 */
    bsp_motor_feedback_t fb;
    bsp_motor_get_feedback(&fb);
    s_cir.start_left_count  = fb.left_count;
    s_cir.start_right_count = fb.right_count;

    s_cir.yaw_accum_deg = 0.0f;
    s_cir.arc_mm        = 0;
    s_cir.start_ms      = bsp_systick_get_ms();
    s_cir.elapsed_ms    = 0u;

    s_cir.phase = CIRCLE_RUNNING;

    (void)printf("[circle] start diam=%umm v=%dmm/s %s circ=%ldmm "
                 "avg_cps=%ld dif_cps=%ld expect=%lums\r\n",
        (unsigned int)diameter_mm,
        (int)v_mm_s,
        clockwise ? "CW" : "CCW",
        (long)circ_mm,
        (long)s_cir.target_avg_cps,
        (long)s_cir.target_dif_cps,
        (unsigned long)s_cir.expected_ms);
}

void app_circle_demo_cancel(void)
{
    if (s_cir.phase == CIRCLE_IDLE) {
        return;
    }
    (void)printf("[circle] cancelled yaw=%c%ld.%02lu arc=%ldmm t=%lums\r\n",
        CIR_F2_S(s_cir.yaw_accum_deg),
        (long)CIR_F2_I(s_cir.yaw_accum_deg),
        (unsigned long)CIR_F2_F(s_cir.yaw_accum_deg),
        (long)s_cir.arc_mm,
        (unsigned long)s_cir.elapsed_ms);
    s_cir.phase = CIRCLE_IDLE;
}

bool app_circle_demo_is_active(void)
{
    return (s_cir.phase == CIRCLE_RUNNING);
}

void app_circle_demo_get_diag(app_circle_diag_t *out)
{
    if (out == (void *)0) return;
    out->active       = app_circle_demo_is_active();
    out->yaw_accum_deg = s_cir.yaw_accum_deg;
    out->arc_mm       = s_cir.arc_mm;
    out->elapsed_ms   = s_cir.elapsed_ms;
}

void app_circle_demo_tick_20hz(const ms901m_snapshot_t *snap,
                                const bsp_motor_feedback_t *fb,
                                app_balance_motion_cmd_t *cmd)
{
    if (s_cir.phase != CIRCLE_RUNNING) {
        return;
    }

    /* dt = 调度周期（50 ms for 20 Hz） */
    static const float dt_sec = (float)APP_BALANCE_SPEED_PERIOD_MS / 1000.0f;

    /* ── 偏航积分（主判据） ────────────────────────────────────── */
    if (snap != (void *)0 && snap->has_gyro_acc) {
        s_cir.yaw_accum_deg += snap->gz_dps * dt_sec;
    }

    /* ── 弧长（备份判据） ──────────────────────────────────────── */
    if (fb != (void *)0) {
        int32_t dl = fb->left_count  - s_cir.start_left_count;
        int32_t dr = fb->right_count - s_cir.start_right_count;
        int32_t avg_cnt = (dl + dr) / 2;
        int32_t arc = robot_arc_mm_from_avg_counts(avg_cnt);
        s_cir.arc_mm = (arc < 0) ? -arc : arc;
    }

    /* ── 时间 ──────────────────────────────────────────────────── */
    s_cir.elapsed_ms = bsp_systick_get_ms() - s_cir.start_ms;

    /* ── 写入运动指令 ──────────────────────────────────────────── */
    cmd->target_speed_cps = s_cir.target_avg_cps;
    cmd->target_dif_cps   = s_cir.target_dif_cps;

    /* ── 判停 ──────────────────────────────────────────────────── */
    float yaw_abs = (s_cir.yaw_accum_deg < 0.0f)
                    ? -s_cir.yaw_accum_deg : s_cir.yaw_accum_deg;

    bool done = false;
    const char *reason = "";

    /* ① IMU 偏航 ≥ 360° */
    if (yaw_abs >= 360.0f) {
        done = true;
        reason = "yaw>=360";
    }

    /* ② 弧长 ≥ 圆周长 × 1.2 */
    if (!done) {
        int32_t arc_limit = (s_cir.circumference_mm *
                             APP_CIRCLE_ARC_OVERSHOOT_X100) / 100;
        if (s_cir.arc_mm >= arc_limit) {
            done = true;
            reason = "arc_overshoot";
        }
    }

    /* ③ 超时兜底 */
    if (!done) {
        uint32_t timeout_ms = (s_cir.expected_ms *
                               APP_CIRCLE_TIMEOUT_FACTOR_X100) / 100u;
        if (timeout_ms < 5000u) timeout_ms = 5000u;
        if (s_cir.elapsed_ms >= timeout_ms) {
            done = true;
            reason = "timeout";
        }
    }

    if (done) {
        cmd->target_speed_cps = 0;
        cmd->target_dif_cps   = 0;
        s_cir.phase = CIRCLE_IDLE;
        (void)printf("[circle] done reason=%s yaw=%c%ld.%02lu arc=%ldmm t=%lums\r\n",
            reason,
            CIR_F2_S(s_cir.yaw_accum_deg),
            (long)CIR_F2_I(s_cir.yaw_accum_deg),
            (unsigned long)CIR_F2_F(s_cir.yaw_accum_deg),
            (long)s_cir.arc_mm,
            (unsigned long)s_cir.elapsed_ms);
    }
}
