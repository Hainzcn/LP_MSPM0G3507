/**
 * @file    app_track.c
 * @brief   赛道模式主控状态机实现，详见 app_track.h。
 */

#include "app_track.h"

#include "app_buzzer.h"
#include "app_safety.h"
#include "bsp_motor.h"
#include "bsp_systick.h"
#include "robot_param.h"

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/* ── 浮点格式化辅助（复用 app_balance / circle_demo 风格，避免 %f） ───────── */
#define TRK_F2_X100(v) ((int32_t)((v) * 100.0f + ((v) >= 0.0f ? 0.5f : -0.5f)))
#define TRK_F2_S(v)    (TRK_F2_X100(v) < 0 ? '-' : '+')
#define TRK_F2_I(v)    ((int32_t)((TRK_F2_X100(v) < 0 ? -TRK_F2_X100(v) : TRK_F2_X100(v)) / 100))
#define TRK_F2_F(v)    ((uint32_t)((TRK_F2_X100(v) < 0 ? -TRK_F2_X100(v) : TRK_F2_X100(v)) % 100))

/* ── 赛道预置 PID 增益（来自 formula.md 实测，x1000 命令值换算后的浮点） ──── */
/*   bp 70000 2000 0 20 → angle kp=70 ki=2 kd=0 offset=20pm                    */
/*   sp -2    1    0 0  → speed kp=-0.002 ki=0.001 kd=0 offset=0               */
/*   dp 4000  3000 0 0  → diff  kp=4 ki=3 kd=0 offset=0                        */
/*   yp 0     0    0 0  → yaw   全 0                                           */
#define TRK_GAIN_ANGLE_KP   (70.0f)
#define TRK_GAIN_ANGLE_KI   (2.0f)
#define TRK_GAIN_ANGLE_KD   (0.0f)
#define TRK_GAIN_ANGLE_OFS  (20.0f)
#define TRK_GAIN_SPEED_KP   (-0.002f)
#define TRK_GAIN_SPEED_KI   (0.001f)
#define TRK_GAIN_SPEED_KD   (0.0f)
#define TRK_GAIN_SPEED_OFS  (0.0f)
#define TRK_GAIN_DIFF_KP    (4.0f)
#define TRK_GAIN_DIFF_KI    (3.0f)
#define TRK_GAIN_DIFF_KD    (0.0f)
#define TRK_GAIN_DIFF_OFS   (0.0f)

/* ── 内部状态 ─────────────────────────────────────────────────────────────── */

typedef struct {
    app_track_phase_t phase;
    uint8_t  lap;                   /* 当前圈号（1 起；0=未开始） */

    uint32_t phase_start_ms;        /* 当前阶段进入时刻 */

    float    rise_pitch0;           /* 自立起始倾角（°） */

    /* 判圈累计量 */
    float    yaw_accum_deg;
    int32_t  start_left_count;
    int32_t  start_right_count;
    int32_t  arc_mm;

    /* 速度包络 */
    int32_t  applied_cps;

    /* 计时器 */
    uint32_t settle_ms;             /* 自立稳定累计 */
    uint32_t stop_ms;               /* 停稳累计 */
} track_state_t;

static track_state_t s_trk;

static const float s_dt_sec = (float)APP_BALANCE_SPEED_PERIOD_MS / 1000.0f;

/* ── 内部辅助 ─────────────────────────────────────────────────────────────── */

static const char *phase_name(app_track_phase_t p)
{
    switch (p) {
    case APP_TRACK_IDLE:        return "IDLE";
    case APP_TRACK_SELF_STAND:  return "SELF_STAND";
    case APP_TRACK_STAND_SETTLE:return "SETTLE";
    case APP_TRACK_TRACE:       return "TRACE";
    case APP_TRACK_BRAKE:       return "BRAKE";
    case APP_TRACK_PAUSE:       return "PAUSE";
    case APP_TRACK_FINAL_BRAKE: return "FINAL_BRAKE";
    case APP_TRACK_DONE:        return "DONE";
    default:                    return "?";
    }
}

static void enter_phase(app_track_phase_t next)
{
    s_trk.phase          = next;
    s_trk.phase_start_ms = bsp_systick_get_ms();
    (void)printf("[track] -> %s (lap=%u)\r\n", phase_name(next), (unsigned int)s_trk.lap);
}

/** 角度环 → 自立专用增益（猛起 kp / ki=0 / kd 阻尼）。 */
static void track_set_angle_rise(void)
{
    app_balance_set_balance_gains(APP_TRACK_RISE_KP, APP_TRACK_RISE_KI,
                                  APP_TRACK_RISE_KD, APP_TRACK_RISE_OFS);
}

/** 角度环 → 运动（循线/平衡）增益。 */
static void track_set_angle_motion(void)
{
    app_balance_set_balance_gains(TRK_GAIN_ANGLE_KP, TRK_GAIN_ANGLE_KI,
                                  TRK_GAIN_ANGLE_KD, TRK_GAIN_ANGLE_OFS);
}

/** 外环（速度/差速/航向）→ 运动增益。速度环全程在线，自立段亦如此（防漂移）。 */
static void track_apply_outer_gains(void)
{
    app_balance_set_speed_gains(TRK_GAIN_SPEED_KP, TRK_GAIN_SPEED_KI,
                                TRK_GAIN_SPEED_KD, TRK_GAIN_SPEED_OFS);
    app_balance_set_diff_gains(TRK_GAIN_DIFF_KP, TRK_GAIN_DIFF_KI,
                               TRK_GAIN_DIFF_KD, TRK_GAIN_DIFF_OFS);
    app_balance_set_yaw_gains(0.0f, 0.0f, 0.0f, 0.0f);
}

/** 速率限制：applied 朝 desired 每拍最多移动 max_step（raw cps）。 */
static int32_t rate_limit(int32_t applied, int32_t desired, int32_t max_step)
{
    int32_t d = desired - applied;
    if (d >  max_step) d =  max_step;
    if (d < -max_step) d = -max_step;
    return applied + d;
}

/** 复位本圈判圈累计量，记录起始编码器计数。 */
static void reset_lap_accum(const bsp_motor_feedback_t *fb)
{
    s_trk.yaw_accum_deg = 0.0f;
    s_trk.arc_mm        = 0;
    if (fb != NULL) {
        s_trk.start_left_count  = fb->left_count;
        s_trk.start_right_count = fb->right_count;
    } else {
        s_trk.start_left_count  = 0;
        s_trk.start_right_count = 0;
    }
}

/** 判圈：偏航 + 里程双条件，超时兜底。 */
static bool lap_complete(uint32_t elapsed_ms)
{
    float yaw_abs = (s_trk.yaw_accum_deg < 0.0f)
                    ? -s_trk.yaw_accum_deg : s_trk.yaw_accum_deg;
    int32_t arc_abs = (s_trk.arc_mm < 0) ? -s_trk.arc_mm : s_trk.arc_mm;
    int32_t arc_min = (APP_TRACK_LAP_LENGTH_MM * APP_TRACK_LAP_ARC_MIN_X100) / 100;

    if ((yaw_abs >= APP_TRACK_YAW_PER_LAP_DEG) && (arc_abs >= arc_min)) {
        return true;
    }
    if (elapsed_ms >= APP_TRACK_LAP_TIMEOUT_MS) {
        (void)printf("[track] lap timeout fallback (yaw=%c%ld arc=%ldmm)\r\n",
            TRK_F2_S(s_trk.yaw_accum_deg), (long)TRK_F2_I(s_trk.yaw_accum_deg),
            (long)arc_abs);
        return true;
    }
    return false;
}

/** 累计本拍偏航与里程。 */
static void update_lap_accum(const ms901m_snapshot_t *snap,
                             const bsp_motor_feedback_t *fb)
{
    if (snap != NULL && snap->has_gyro_acc) {
        s_trk.yaw_accum_deg += snap->gz_dps * s_dt_sec;
    }
    if (fb != NULL) {
        int32_t dl = fb->left_count  - s_trk.start_left_count;
        int32_t dr = fb->right_count - s_trk.start_right_count;
        s_trk.arc_mm = robot_arc_mm_from_avg_counts((dl + dr) / 2);
    }
}

static int32_t avg_cps_abs(const bsp_motor_feedback_t *fb)
{
    if (fb == NULL) return 0;
    int32_t avg = (fb->left_speed_cps + fb->right_speed_cps) / 2;
    return (avg < 0) ? -avg : avg;
}

/* ── 公共 API ─────────────────────────────────────────────────────────────── */

void app_track_init(void)
{
    s_trk.phase          = APP_TRACK_IDLE;
    s_trk.lap            = 0u;
    s_trk.phase_start_ms = 0u;
    s_trk.rise_pitch0    = 0.0f;
    s_trk.yaw_accum_deg  = 0.0f;
    s_trk.start_left_count  = 0;
    s_trk.start_right_count = 0;
    s_trk.arc_mm         = 0;
    s_trk.applied_cps    = 0;
    s_trk.settle_ms      = 0u;
    s_trk.stop_ms        = 0u;
}

void app_track_start(void)
{
    if (s_trk.phase != APP_TRACK_IDLE && s_trk.phase != APP_TRACK_DONE) {
        (void)printf("[track] already active (%s)\r\n", phase_name(s_trk.phase));
        return;
    }
    /* 外环（速度/差速/航向）设运动增益且全程在线（速度环防自立漂移）；
     * 角度环切自立专用增益（猛起+阻尼）。自立靠速度环把 target_speed=0 维持
     * 原地，角度环把车从大倾角摆到直立。 */
    track_apply_outer_gains();
    track_set_angle_rise();
    s_trk.lap          = 0u;
    s_trk.applied_cps  = 0;
    s_trk.settle_ms    = 0u;
    s_trk.stop_ms      = 0u;
    s_trk.rise_pitch0  = app_balance_get_pitch_meas();
    enter_phase(APP_TRACK_SELF_STAND);
    /* 上电自检已通过（ARMED）；赛道模式启动时播放《兰花草》，播完自动静音。 */
    app_buzzer_play_lanhua_cao();
    (void)printf("[track] start: rise from pitch0=%c%ld.%02lu deg (rise kp=%d kd=%d)\r\n",
        TRK_F2_S(s_trk.rise_pitch0),
        (long)TRK_F2_I(s_trk.rise_pitch0),
        (unsigned long)TRK_F2_F(s_trk.rise_pitch0),
        (int)APP_TRACK_RISE_KP, (int)APP_TRACK_RISE_KD);
}

void app_track_cancel(void)
{
    if (s_trk.phase == APP_TRACK_IDLE) return;
    app_buzzer_stop();
    /* 恢复角度环运动增益，避免取消后仍停留在自立增益。 */
    track_set_angle_motion();
    s_trk.applied_cps = 0;
    enter_phase(APP_TRACK_IDLE);
    (void)printf("[track] cancelled\r\n");
}

bool app_track_is_active(void)
{
    return (s_trk.phase != APP_TRACK_IDLE);
}

app_track_phase_t app_track_get_phase(void)
{
    return s_trk.phase;
}

uint8_t app_track_get_lap(void)
{
    return s_trk.lap;
}

void app_track_get_diag(app_track_diag_t *out)
{
    if (out == NULL) return;
    out->phase            = s_trk.phase;
    out->lap              = s_trk.lap;
    out->yaw_accum_deg    = s_trk.yaw_accum_deg;
    out->arc_mm           = s_trk.arc_mm;
    out->applied_cps      = s_trk.applied_cps;
    out->phase_elapsed_ms = bsp_systick_get_ms() - s_trk.phase_start_ms;
}

/* ── 20 Hz 调度 ──────────────────────────────────────────────────────────── */

void app_track_tick_20hz(const ms901m_snapshot_t *snap,
                         const bsp_motor_feedback_t *fb,
                         app_balance_motion_cmd_t *cmd)
{
    if (s_trk.phase == APP_TRACK_IDLE || cmd == NULL) {
        return;
    }

    uint32_t now_ms     = bsp_systick_get_ms();
    uint32_t elapsed_ms = now_ms - s_trk.phase_start_ms;

    /* 活动期间若跌倒 / 失能（非自立态）则中止赛道，回到 IDLE。
     * 自立态本身就工作在大倾角，由 app_balance/safety 的 60° 判据兜底。 */
    if (s_trk.phase != APP_TRACK_SELF_STAND && !app_safety_can_drive()) {
        (void)printf("[track] drive lost in %s -> abort\r\n", phase_name(s_trk.phase));
        app_track_cancel();
        cmd->target_speed_cps = 0;
        cmd->target_dif_cps   = 0;
        return;
    }

    switch (s_trk.phase) {

    /* ── 自立：角度环用 rise 增益（setpoint 由速度环给出，target_speed=0）
     *   猛起摆到直立。速度环全程在线 → 原地不漂移。
     *   首次摆到 |pitch|<RISE_DONE_DEG 即转稳定确认；RISE_MS 超时兜底。 */
    case APP_TRACK_SELF_STAND: {
        cmd->target_speed_cps = 0;
        cmd->target_dif_cps   = 0;

        float pitch_abs = app_balance_get_pitch_meas();
        if (pitch_abs < 0.0f) pitch_abs = -pitch_abs;

        if ((pitch_abs < APP_TRACK_RISE_DONE_DEG) ||
            (elapsed_ms >= APP_TRACK_RISE_MS)) {
            s_trk.settle_ms = 0u;
            enter_phase(APP_TRACK_STAND_SETTLE);
        }
        break;
    }

    /* ── 稳定确认：保持 rise 角度增益 + target_speed=0，等 |pitch|/|gz|
     *   平稳一段时间；确认后切换运动角度增益，交棒给循线。 */
    case APP_TRACK_STAND_SETTLE: {
        cmd->target_speed_cps = 0;
        cmd->target_dif_cps   = 0;

        float pitch_abs = app_balance_get_pitch_meas();
        if (pitch_abs < 0.0f) pitch_abs = -pitch_abs;
        float gz_abs = (snap != NULL) ? snap->gz_dps : 0.0f;
        if (gz_abs < 0.0f) gz_abs = -gz_abs;

        if ((pitch_abs < APP_TRACK_RISE_DONE_DEG) &&
            (gz_abs < APP_TRACK_RISE_DONE_DPS)) {
            s_trk.settle_ms += (uint32_t)APP_BALANCE_SPEED_PERIOD_MS;
        } else {
            s_trk.settle_ms = 0u;
        }

        if (s_trk.settle_ms >= APP_TRACK_SETTLE_MS) {
            /* 交棒：角度环切运动增益，进入第一圈循线 */
            track_set_angle_motion();
            (void)printf("[track] stood up, handover to motion PID\r\n");
            s_trk.lap         = 1u;
            s_trk.applied_cps = 0;
            reset_lap_accum(fb);
            enter_phase(APP_TRACK_TRACE);
        }
        break;
    }

    /* ── 循线：速度透传 K230（与手动 WiFi trace 2000 一致），累计判圈 ──
     * 不在此二次限速：K230 已有 slew，balance 有 speed_target_lpf；
     * 旧方案 ACCEL=400 raw/拍 使 2000 归一化目标需 ~2.5s 才爬满 → 骑线几乎不走。 */
    case APP_TRACK_TRACE: {
        update_lap_accum(snap, fb);

        s_trk.applied_cps = cmd->target_speed_cps;   /* 诊断用，透传 K230 raw cps */
        /* target_dif_cps 透传 K230 循线转向 */

        if (lap_complete(elapsed_ms)) {
            (void)printf("[track] lap %u done (yaw=%c%ld.%02lu arc=%ldmm t=%lums)\r\n",
                (unsigned int)s_trk.lap,
                TRK_F2_S(s_trk.yaw_accum_deg),
                (long)TRK_F2_I(s_trk.yaw_accum_deg),
                (unsigned long)TRK_F2_F(s_trk.yaw_accum_deg),
                (long)s_trk.arc_mm,
                (unsigned long)elapsed_ms);
            s_trk.stop_ms = 0u;
            if (s_trk.lap >= APP_TRACK_N_LAPS) {
                enter_phase(APP_TRACK_FINAL_BRAKE);
            } else {
                enter_phase(APP_TRACK_BRAKE);
            }
        }
        break;
    }

    /* ── 减速刹车（满圈→暂停）：速度斜坡到 0 并停稳 ──────────────── */
    case APP_TRACK_BRAKE: {
        s_trk.applied_cps = rate_limit(s_trk.applied_cps, 0,
                                       APP_TRACK_DECEL_CPS_PER_TICK);
        cmd->target_speed_cps = s_trk.applied_cps;
        cmd->target_dif_cps   = 0;

        if ((s_trk.applied_cps == 0) && (avg_cps_abs(fb) < APP_TRACK_STOP_CPS)) {
            s_trk.stop_ms += (uint32_t)APP_BALANCE_SPEED_PERIOD_MS;
        } else {
            s_trk.stop_ms = 0u;
        }
        if (s_trk.stop_ms >= APP_TRACK_STOP_SETTLE_MS) {
            enter_phase(APP_TRACK_PAUSE);
        }
        break;
    }

    /* ── 暂停：保持直立静止 PAUSE_MS，到期进入下一圈 ───────────────── */
    case APP_TRACK_PAUSE: {
        cmd->target_speed_cps = 0;
        cmd->target_dif_cps   = 0;
        s_trk.applied_cps     = 0;

        if (elapsed_ms >= APP_TRACK_PAUSE_MS) {
            s_trk.lap        = (uint8_t)(s_trk.lap + 1u);
            s_trk.applied_cps = 0;
            reset_lap_accum(fb);
            enter_phase(APP_TRACK_TRACE);
        }
        break;
    }

    /* ── 末圈刹车：速度斜坡到 0 停稳，进入 DONE ───────────────────── */
    case APP_TRACK_FINAL_BRAKE: {
        s_trk.applied_cps = rate_limit(s_trk.applied_cps, 0,
                                       APP_TRACK_DECEL_CPS_PER_TICK);
        cmd->target_speed_cps = s_trk.applied_cps;
        cmd->target_dif_cps   = 0;

        if ((s_trk.applied_cps == 0) && (avg_cps_abs(fb) < APP_TRACK_STOP_CPS)) {
            s_trk.stop_ms += (uint32_t)APP_BALANCE_SPEED_PERIOD_MS;
        } else {
            s_trk.stop_ms = 0u;
        }
        if (s_trk.stop_ms >= APP_TRACK_STOP_SETTLE_MS) {
            enter_phase(APP_TRACK_DONE);
            (void)printf("[track] all %u laps complete, holding upright\r\n",
                (unsigned int)APP_TRACK_N_LAPS);
        }
        break;
    }

    /* ── 完成：保持直立静止 ───────────────────────────────────────── */
    case APP_TRACK_DONE:
    default:
        cmd->target_speed_cps = 0;
        cmd->target_dif_cps   = 0;
        s_trk.applied_cps     = 0;
        break;
    }
}
