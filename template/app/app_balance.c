/**
 * @file    app_balance.c
 * @brief   平衡车四级级联控制实现，详见 app_balance.h。
 *
 *   速度环 (20 Hz) → 角度环 (100 Hz) → 角速度环 (200 Hz) → 电机输出
 *   航向环 (50 Hz) ──────────────────→ 差分叠加 ──┘
 *
 *   所有 PID 增益默认 0，业务层未 set_gains 之前电机不会动。
 */

#include "app_balance.h"

#include "app_safety.h"
#include "bsp_battery.h"
#include "bsp_gpio.h"
#include "bsp_imu_uart.h"
#include "bsp_k230_uart.h"
#include "bsp_log_uart.h"
#include "bsp_motor.h"
#include "bsp_systick.h"
#include "ms901m.h"
#include "pid.h"
#include "ti_msp_dl_config.h"

#include <stdio.h>
#include <stddef.h>

/* -------------------------------------------------------------------------- */
/* 内部状态                                                                    */
/* -------------------------------------------------------------------------- */

typedef struct {
    pid_t   speed_pid;       /* 速度外环 20 Hz：输入 cps 误差，输出目标 tilt deg */
    pid_t   angle_pid;       /* 角度环 100 Hz：输入 tilt 误差 deg，输出目标角速率 °/s */
    pid_t   rate_pid;        /* 角速度内环 200 Hz：输入角速率误差 °/s，输出 PWM permille */
    pid_t   yaw_pid;         /* 航向环 50 Hz：输入 yaw 误差 °，输出差分 PWM permille */
    float   target_rate_dps;    /* 角度环输出：角速度内环设定值 */
    float   target_tilt_deg;    /* 速度外环输出：角度环设定值 */
    int16_t cached_yaw_corr_pm; /* 航向环缓存的差分补偿，供 200 Hz 角速度环使用 */
    float   rate_lpf_dps;       /* 角速度测量 EMA 低通（抑制振动耦合噪声） */
    float   pitch_offset_deg;
    float   pitch_lpf_deg;
    float   speed_lpf_cps;   /* 速度反馈 EMA 低通：抑制编码器量化噪声 */
    int8_t  pitch_sign;      /* +1 正常，-1 传感器前后反装时软件翻转 */
    int8_t  yaw_sign;        /* +1 正常，-1 gz_dps/yaw_deg 符号翻转 */
    int8_t  speed_sign;      /* +1 正常，-1 编码器方向与平衡环正向相反时翻转 */
    bool    pitch_lpf_valid;
    float   yaw_kp;          /* 转向开环系数 */
    float   yaw_target_deg;     /* [源 0] EKF 模式：锁定目标航向（°） */
    bool    yaw_target_valid;   /* [源 0] EKF 模式：首次有效 yaw 才初始化 */
    float   yaw_gz_integrated;  /* [源 1] 陀螺积分模式：累积偏航量（°） */
    bool    lt_stream_enabled;
    app_balance_diag_t diag;
} balance_state_t;

static balance_state_t s_bal;

static const float s_dt_rate_sec  = (float)APP_BALANCE_RATE_PERIOD_MS  / 1000.0f;
static const float s_dt_angle_sec = (float)APP_BALANCE_ANGLE_PERIOD_MS / 1000.0f;
static const float s_dt_yaw_sec   = (float)APP_BALANCE_YAW_PERIOD_MS   / 1000.0f;
static const float s_dt_speed_sec = (float)APP_BALANCE_SPEED_PERIOD_MS / 1000.0f;

#define APP_BAL_CMD_BUF_LEN  48u
#define APP_BAL_PID_SCALE    1000L

/* -------------------------------------------------------------------------- */
/* 内部辅助                                                                    */
/* -------------------------------------------------------------------------- */

static int16_t clamp_pwm_pm(float v)
{
    if (v >  (float)APP_BALANCE_MAX_PWM_PERMILLE) v =  (float)APP_BALANCE_MAX_PWM_PERMILLE;
    if (v < -(float)APP_BALANCE_MAX_PWM_PERMILLE) v = -(float)APP_BALANCE_MAX_PWM_PERMILLE;
    return (int16_t)v;
}

static float apply_pitch_orientation(float raw_pitch_deg)
{
    float centered = raw_pitch_deg - s_bal.pitch_offset_deg;
    return (s_bal.pitch_sign < 0) ? -centered : centered;
}

static void reset_pitch_filter(void)
{
    s_bal.pitch_lpf_deg = 0.0f;
    s_bal.pitch_lpf_valid = false;
}

static float filter_pitch(float pitch_deg)
{
    float alpha = APP_BALANCE_PITCH_LPF_ALPHA;
    if (alpha <= 0.0f) {
        s_bal.pitch_lpf_deg = pitch_deg;
        s_bal.pitch_lpf_valid = true;
        return pitch_deg;
    }
    if (alpha > 1.0f) {
        alpha = 1.0f;
    }

    if (!s_bal.pitch_lpf_valid) {
        s_bal.pitch_lpf_deg = pitch_deg;
        s_bal.pitch_lpf_valid = true;
        return pitch_deg;
    }

    s_bal.pitch_lpf_deg += alpha * (pitch_deg - s_bal.pitch_lpf_deg);
    return s_bal.pitch_lpf_deg;
}

/** 把角度差折叠到 [-180, 180] 区间，处理 yaw 环绕。 */
static float wrap_180(float deg)
{
    while (deg >  180.0f) { deg -= 360.0f; }
    while (deg < -180.0f) { deg += 360.0f; }
    return deg;
}

static int16_t clamp_yaw_correction(float v)
{
    if (v >  (float)APP_BALANCE_YAW_MAX_CORRECTION_PM) {
        v =  (float)APP_BALANCE_YAW_MAX_CORRECTION_PM;
    }
    if (v < -(float)APP_BALANCE_YAW_MAX_CORRECTION_PM) {
        v = -(float)APP_BALANCE_YAW_MAX_CORRECTION_PM;
    }
    return (int16_t)v;
}

static void reset_yaw_state(void)
{
    pid_reset(&s_bal.yaw_pid);
    s_bal.yaw_target_valid   = false;
    s_bal.yaw_gz_integrated  = 0.0f;
    s_bal.diag.yaw_error_deg     = 0.0f;
    s_bal.diag.yaw_correction_pm = 0;
}

/**
 * Yaw 角度环一拍计算。
 *
 * 由编译期 APP_BALANCE_YAW_SOURCE 选择数据源：
 *
 *   0 = EKF 绝对角（yaw_deg）
 *       ─ 锁定首次目标角，后续用 wrap_180 + virtual_measured 跟踪；
 *       ─ 转向时持续更新目标角为当前角，结束后无缝接管。
 *
 *   1 = 陀螺仪积分（gz_dps × dt）
 *       ─ 直行时持续积分，PID 以"归零积分量"为目标；
 *       ─ D 项等效于 gz_dps 角速率阻尼，天然平滑；
 *       ─ 转向时清零积分，转向结束后重新从 0 追踪；
 *       ─ 彻底无 ±180° 跳变，不受磁场干扰。
 *
 * 输出：差分补偿 permille，正值 → left 加 / right 减（即左轮加速 → 修正左偏）。
 */
static int16_t yaw_angle_step(const app_balance_attitude_t *att,
                               const app_balance_motion_cmd_t *cmd)
{
#if !APP_BALANCE_YAW_ENABLED
    (void)att;
    (void)cmd;
    reset_yaw_state();
    return 0;

/* ---- 源 0：EKF 绝对偏航角 ------------------------------------------------ */
#elif APP_BALANCE_YAW_SOURCE == 0
    if ((att == NULL) || (cmd == NULL)) {
        reset_yaw_state();
        return 0;
    }

    float yaw_deg = att->yaw_deg * (float)s_bal.yaw_sign;

    /* 转向期间：暂停闭环，持续刷新目标角，转向结束后无缝接管 */
    if (cmd->target_yaw_pm != 0) {
        pid_reset(&s_bal.yaw_pid);
        s_bal.yaw_target_deg   = yaw_deg;
        s_bal.yaw_target_valid = true;
        s_bal.diag.yaw_error_deg     = 0.0f;
        s_bal.diag.yaw_correction_pm = 0;
        return 0;
    }

    /* 直行：首拍初始化目标角 */
    if (!s_bal.yaw_target_valid) {
        s_bal.yaw_target_deg   = yaw_deg;
        s_bal.yaw_target_valid = true;
        pid_reset(&s_bal.yaw_pid);
        s_bal.diag.yaw_error_deg     = 0.0f;
        s_bal.diag.yaw_correction_pm = 0;
        return 0;
    }

    /* wrap_180 误差，"virtual measured" 技巧避免 D 项在 ±180 跳变 */
    float err          = wrap_180(s_bal.yaw_target_deg - yaw_deg);
    float virtual_meas = s_bal.yaw_target_deg - err;
    float raw_corr     = pid_step(&s_bal.yaw_pid,
                                   s_bal.yaw_target_deg,
                                   virtual_meas,
                                   s_dt_yaw_sec);
    int16_t corr_pm    = clamp_yaw_correction(raw_corr);

    s_bal.diag.yaw_error_deg     = err;
    s_bal.diag.yaw_correction_pm = corr_pm;
    return corr_pm;

/* ---- 源 1：陀螺仪积分（gz_dps × dt） -------------------------------------- */
#else
    if ((att == NULL) || (cmd == NULL)) {
        reset_yaw_state();
        return 0;
    }

    /* 转向期间：暂停 PID，清零积分，转向结束后从 0 重新追踪 */
    if (cmd->target_yaw_pm != 0) {
        pid_reset(&s_bal.yaw_pid);
        s_bal.yaw_gz_integrated      = 0.0f;
        s_bal.diag.yaw_error_deg     = 0.0f;
        s_bal.diag.yaw_correction_pm = 0;
        return 0;
    }

    /* 直行：积分 gz_dps（经 yaw_sign 修正极性），PID 以"归零积分量"为目标。
     * D 项 = -Kd * d(yaw_gz_integrated)/dt ≈ -Kd * gz_dps * yaw_sign，
     * 天然提供角速率阻尼，无需单独引入速率反馈。 */
    s_bal.yaw_gz_integrated += att->gz_dps * (float)s_bal.yaw_sign * s_dt_yaw_sec;

    float raw_corr  = pid_step(&s_bal.yaw_pid,
                                0.0f,
                                s_bal.yaw_gz_integrated,
                                s_dt_yaw_sec);
    int16_t corr_pm = clamp_yaw_correction(raw_corr);

    s_bal.diag.yaw_error_deg     = -s_bal.yaw_gz_integrated; /* error = target - measured */
    s_bal.diag.yaw_correction_pm = corr_pm;
    return corr_pm;
#endif
}

/* -------------------------------------------------------------------------- */
/* 公共 API                                                                    */
/* -------------------------------------------------------------------------- */

void app_balance_init(void)
{
    pid_init(&s_bal.speed_pid);
    pid_init(&s_bal.angle_pid);
    pid_init(&s_bal.rate_pid);
    pid_init(&s_bal.yaw_pid);

    /* 速度外环 20 Hz：输出"目标 tilt deg"，限幅 ±MAX_TILT_DEG */
    pid_set_output_limit(&s_bal.speed_pid,
        -(float)APP_BALANCE_MAX_TILT_DEG, (float)APP_BALANCE_MAX_TILT_DEG);
    pid_set_d_filter(&s_bal.speed_pid, APP_BALANCE_SPEED_D_FILTER_ALPHA);

    /* 角度环 100 Hz：输出"目标角速率 °/s"，限幅 ±MAX_TARGET_RATE_DPS */
    pid_set_output_limit(&s_bal.angle_pid,
        -(float)APP_BALANCE_MAX_TARGET_RATE_DPS,
         (float)APP_BALANCE_MAX_TARGET_RATE_DPS);

    /* 角速度内环 200 Hz：输出 PWM permille，限幅 ±MAX_PWM */
    pid_set_output_limit(&s_bal.rate_pid,
        -(float)APP_BALANCE_MAX_PWM_PERMILLE,
         (float)APP_BALANCE_MAX_PWM_PERMILLE);
    pid_set_d_filter(&s_bal.rate_pid, APP_BALANCE_RATE_D_FILTER_ALPHA);

    /* 航向环 50 Hz：输出差分 PWM permille，限幅 ±YAW_MAX_CORRECTION */
    pid_set_output_limit(&s_bal.yaw_pid,
        -(float)APP_BALANCE_YAW_MAX_CORRECTION_PM,
         (float)APP_BALANCE_YAW_MAX_CORRECTION_PM);
    pid_set_d_filter(&s_bal.yaw_pid, APP_BALANCE_YAW_D_FILTER_ALPHA);

    s_bal.target_rate_dps    = 0.0f;
    s_bal.target_tilt_deg    = 0.0f;
    s_bal.cached_yaw_corr_pm = 0;
    s_bal.rate_lpf_dps       = 0.0f;
    s_bal.pitch_offset_deg   = 0.8f;
    s_bal.speed_lpf_cps      = 0.0f;
    reset_pitch_filter();
    s_bal.pitch_sign =
#if APP_BALANCE_PITCH_INVERT
        -1;
#else
        1;
#endif
    s_bal.yaw_sign =
#if APP_BALANCE_YAW_INVERT
        -1;
#else
        1;
#endif
    s_bal.speed_sign =
#if APP_BALANCE_SPEED_INVERT
        -1;
#else
        1;
#endif
    s_bal.yaw_kp = 1.0f;
    s_bal.lt_stream_enabled = false;
    reset_yaw_state();

    s_bal.diag.target_tilt_deg = 0.0f;
    s_bal.diag.pitch_meas_deg  = 0.0f;
    s_bal.diag.balance_out_pwm = 0.0f;
    s_bal.diag.left_cmd_pm     = 0;
    s_bal.diag.right_cmd_pm    = 0;
    s_bal.diag.speed_meas_cps  = 0;
    s_bal.diag.driving         = false;
}

void app_balance_reset(void)
{
    pid_reset(&s_bal.speed_pid);
    pid_reset(&s_bal.angle_pid);
    pid_reset(&s_bal.rate_pid);
    s_bal.target_rate_dps    = 0.0f;
    s_bal.target_tilt_deg    = 0.0f;
    s_bal.cached_yaw_corr_pm = 0;
    s_bal.rate_lpf_dps       = 0.0f;
    s_bal.speed_lpf_cps      = 0.0f;
    reset_pitch_filter();
    reset_yaw_state();
}

void app_balance_set_pitch_offset(float deg)
{
    s_bal.pitch_offset_deg = deg;
}

void app_balance_set_pitch_inverted(bool inverted)
{
    s_bal.pitch_sign = inverted ? (int8_t)-1 : (int8_t)1;
    app_balance_reset();
}

bool app_balance_get_pitch_inverted(void)
{
    return (s_bal.pitch_sign < 0);
}

void app_balance_set_yaw_inverted(bool inverted)
{
    s_bal.yaw_sign = inverted ? (int8_t)-1 : (int8_t)1;
    reset_yaw_state();
}

bool app_balance_get_yaw_inverted(void)
{
    return (s_bal.yaw_sign < 0);
}

void app_balance_set_speed_inverted(bool inverted)
{
    s_bal.speed_sign = inverted ? (int8_t)-1 : (int8_t)1;
    pid_reset(&s_bal.speed_pid);
    s_bal.speed_lpf_cps   = 0.0f;
    s_bal.target_tilt_deg = 0.0f;
}

bool app_balance_get_speed_inverted(void)
{
    return (s_bal.speed_sign < 0);
}

void app_balance_set_balance_gains(float kp, float ki, float kd)
{
    pid_set_gains(&s_bal.angle_pid, kp, ki, kd);
}

void app_balance_set_rate_gains(float kp, float ki, float kd)
{
    pid_set_gains(&s_bal.rate_pid, kp, ki, kd);
}

void app_balance_set_speed_gains(float kp, float ki, float kd)
{
    pid_set_gains(&s_bal.speed_pid, kp, ki, kd);
}

void app_balance_set_yaw_kp(float kp_yaw)
{
    s_bal.yaw_kp = kp_yaw;
}

void app_balance_set_yaw_gains(float kp, float ki, float kd)
{
    pid_set_gains(&s_bal.yaw_pid, kp, ki, kd);
}

/* -------------------------------------------------------------------------- */
/* 多速率级联子函数                                                             */
/* -------------------------------------------------------------------------- */

/** 速度外环（20 Hz）：输入 cps 误差，输出目标 tilt deg → s_bal.target_tilt_deg */
static void balance_step_speed(const app_balance_motion_cmd_t *cmd)
{
    bsp_motor_feedback_t fb;
    bsp_motor_get_feedback(&fb);
    /* speed_sign：+1 = 编码器正向与平衡环正向一致；-1 = 方向相反（需翻转）。
     * 可用串口 si0/si1 在线切换，或修改 APP_BALANCE_SPEED_INVERT 宏重编译。
     * 诊断方法：不开速度环，看心跳 v= 字段；小车漂移时 v 应与漂移方向同号。
     *
     * 量纲归一化：除以 APP_BALANCE_SPEED_CPS_SCALE（默认 10）。
     * 原始 avg_cps 在典型漂移速度（0.1 rev/s）下约为 5100，若直接用于 PID，
     * sp 1（Kp=0.001）即产生 5.1° 倾角指令，立即触发饱和极限环。
     * 归一化后 → 510，sp 1 对应 0.51°，从安全值开始调试。
     * target_speed_cps 同步归一化，保持误差量纲一致。 */
    int32_t avg_cps_raw = ((fb.left_speed_cps + fb.right_speed_cps) / 2)
                          * (int32_t)s_bal.speed_sign;
    float norm_cps = (float)avg_cps_raw / (float)APP_BALANCE_SPEED_CPS_SCALE;

    float spd_alpha = APP_BALANCE_SPEED_LPF_ALPHA;
    s_bal.speed_lpf_cps += spd_alpha * (norm_cps - s_bal.speed_lpf_cps);

    float target_norm = (float)cmd->target_speed_cps / (float)APP_BALANCE_SPEED_CPS_SCALE;
    s_bal.target_tilt_deg = pid_step(&s_bal.speed_pid,
        target_norm,
        s_bal.speed_lpf_cps,
        s_dt_speed_sec);

    s_bal.diag.target_tilt_deg = s_bal.target_tilt_deg;
    s_bal.diag.speed_meas_cps  = (int32_t)s_bal.speed_lpf_cps;
}

/** 角度环（100 Hz）：输入 tilt 误差 deg，输出目标角速率 °/s → s_bal.target_rate_dps */
static void balance_step_angle(const app_balance_attitude_t *att)
{
    float pitch_meas = filter_pitch(apply_pitch_orientation(att->pitch_deg));

    s_bal.target_rate_dps = pid_step(&s_bal.angle_pid,
        s_bal.target_tilt_deg,
        pitch_meas,
        s_dt_angle_sec);

    s_bal.diag.pitch_meas_deg = pitch_meas;
}

/** 航向环（50 Hz）：输出差分补偿 → s_bal.cached_yaw_corr_pm */
static void balance_step_yaw(const app_balance_attitude_t *att,
                              const app_balance_motion_cmd_t *cmd)
{
    s_bal.cached_yaw_corr_pm = yaw_angle_step(att, cmd);
}

/**
 * 角速度内环（200 Hz）：输入角速率误差 °/s，输出 PWM → bsp_motor_set_output。
 *
 * 同时执行 safety tick + 电机输出。safety 使用最近一次 100 Hz 角度环的 pitch_meas。
 */
static void balance_step_rate(const app_balance_attitude_t *att,
                               const app_balance_motion_cmd_t *cmd)
{
    /* safety tick：使用角度环最新的 pitch_meas */
    app_safety_attitude_t sa = {
        .pitch_deg      = s_bal.diag.pitch_meas_deg,
        .attitude_valid = att->attitude_valid,
    };
    (void)app_safety_tick(&sa);

    if (app_safety_is_startup_grace_active() ||
        !app_safety_can_drive() || !att->attitude_valid) {
        app_balance_reset();
        s_bal.diag.balance_out_pwm   = 0.0f;
        s_bal.diag.yaw_correction_pm = 0;
        s_bal.diag.yaw_error_deg     = 0.0f;
        s_bal.diag.left_cmd_pm       = 0;
        s_bal.diag.right_cmd_pm      = 0;
        s_bal.diag.driving           = false;
        return;
    }

    /* 角速度测量低通滤波：alpha=0.5 EMA，抑制振动耦合噪声（tau≈10ms@200Hz） */
    float raw_rate = att->pitch_rate_dps * (float)s_bal.pitch_sign;
    s_bal.rate_lpf_dps += 0.5f * (raw_rate - s_bal.rate_lpf_dps);
    float measured_rate = s_bal.rate_lpf_dps;

    /* 角速度 PD：输入 = 目标角速率 - 实测角速率，输出 = PWM permille */
    float pwm_out = pid_step(&s_bal.rate_pid,
        s_bal.target_rate_dps,
        measured_rate,
        s_dt_rate_sec);

    /* 转向叠加 + 航向环差分补偿 */
    float yaw_pm     = (float)cmd->target_yaw_pm * s_bal.yaw_kp;
    int16_t yaw_corr = s_bal.cached_yaw_corr_pm;
    int16_t left_pm  = clamp_pwm_pm(pwm_out - yaw_pm + (float)yaw_corr);
    int16_t right_pm = clamp_pwm_pm(pwm_out + yaw_pm - (float)yaw_corr);

#if APP_BALANCE_ZERO_BAND_PM > 0
    if (left_pm  > -APP_BALANCE_ZERO_BAND_PM && left_pm  < APP_BALANCE_ZERO_BAND_PM)
        left_pm  = 0;
    if (right_pm > -APP_BALANCE_ZERO_BAND_PM && right_pm < APP_BALANCE_ZERO_BAND_PM)
        right_pm = 0;
#endif

    bsp_motor_set_output(left_pm, right_pm);

    s_bal.diag.balance_out_pwm = pwm_out;
    s_bal.diag.left_cmd_pm     = left_pm;
    s_bal.diag.right_cmd_pm    = right_pm;
    s_bal.diag.driving         = true;
}

void app_balance_get_diag(app_balance_diag_t *out)
{
    if (out == NULL) return;
    *out = s_bal.diag;
}

/* ========================================================================== */
/* Stage 2.2 上车基线主循环（吸收 Stage 1.6 telemetry 的 IMU drain + 心跳日志）  */
/* ========================================================================== */

/* 单拍 drain UART3 RX 环缓上限，与原 telemetry 一致：
 * MS901M 默认 5 帧 × 200 Hz × ~15 B ≈ 15 kB/s = 15 B/ms，64 B 单拍裕度 4×。 */
#define APP_BAL_IMU_DRAIN_CHUNK     64u

/* 调度相位（1 ms tick 倍数） */
#define APP_BAL_PHASE_RATE_TICKS    APP_BALANCE_RATE_PERIOD_MS     /* 200 Hz */
#define APP_BAL_PHASE_ANGLE_TICKS   APP_BALANCE_ANGLE_PERIOD_MS    /* 100 Hz */
#define APP_BAL_PHASE_YAW_TICKS     APP_BALANCE_YAW_PERIOD_MS      /*  50 Hz */
#define APP_BAL_PHASE_SPEED_TICKS   APP_BALANCE_SPEED_PERIOD_MS    /*  20 Hz */
#define APP_BAL_PHASE_LED_TICKS     200u                            /*   5 Hz */
#define APP_BAL_PHASE_LOG_TICKS     1000u                           /*   1 Hz */

/* 浮点字段格式化辅助（与 app_telemetry.c 同款，避开 AC6 printf("%f") 浮点路径） */
#define BAL_F2_X100(v)  ((int32_t)((v) * 100.0f + ((v) >= 0.0f ? 0.5f : -0.5f)))
#define BAL_F2_S(v)     (BAL_F2_X100(v) < 0 ? '-' : ' ')
#define BAL_F2_I(v)     ((int32_t)((BAL_F2_X100(v) < 0 ? -BAL_F2_X100(v) : BAL_F2_X100(v)) / 100))
#define BAL_F2_F(v)     ((uint32_t)((BAL_F2_X100(v) < 0 ? -BAL_F2_X100(v) : BAL_F2_X100(v)) % 100))

static int32_t cps_to_rpm_x100(int32_t cps, int32_t counts_per_rev)
{
    if (counts_per_rev <= 0) {
        return 0;
    }
    return (int32_t)((cps * 6000L) / counts_per_rev);
}

static char scaled2_sign(int32_t x100)
{
    return (x100 < 0) ? '-' : '+';
}

static int32_t scaled2_int_abs(int32_t x100)
{
    int32_t abs_v = (x100 < 0) ? -x100 : x100;
    return abs_v / 100;
}

static uint32_t scaled2_frac_abs(int32_t x100)
{
    int32_t abs_v = (x100 < 0) ? -x100 : x100;
    return (uint32_t)(abs_v % 100);
}

static bool is_test_command(const char *buf, uint8_t len)
{
    if ((len == 1u) && (buf[0] == 't')) {
        return true;
    }
    return (len == 4u) &&
           (buf[0] == 't') && (buf[1] == 'e') &&
           (buf[2] == 's') && (buf[3] == 't');
}

static bool is_line_delimiter(uint8_t ch)
{
    return (ch == (uint8_t)'\r') || (ch == (uint8_t)'\n');
}

static bool is_field_separator(char ch)
{
    return (ch == ' ') || (ch == '\t') || (ch == ',') ||
           (ch == ';') || (ch == ':');
}

static void skip_separators(const char **p)
{
    while ((*p != NULL) && is_field_separator(**p)) {
        (*p)++;
    }
}

static bool parse_i32_token(const char **p, int32_t *out)
{
    if ((p == NULL) || (*p == NULL) || (out == NULL)) {
        return false;
    }

    skip_separators(p);

    int32_t sign = 1;
    if (**p == '-') {
        sign = -1;
        (*p)++;
    } else if (**p == '+') {
        (*p)++;
    }

    if ((**p < '0') || (**p > '9')) {
        return false;
    }

    int32_t v = 0;
    while ((**p >= '0') && (**p <= '9')) {
        v = (v * 10) + (int32_t)(**p - '0');
        (*p)++;
    }
    *out = v * sign;
    return true;
}

static float scaled_to_float(int32_t x1000)
{
    return (float)x1000 / (float)APP_BAL_PID_SCALE;
}

static int32_t float_to_scaled(float v)
{
    float scaled = v * (float)APP_BAL_PID_SCALE;
    return (int32_t)(scaled + ((scaled >= 0.0f) ? 0.5f : -0.5f));
}

static void print_scaled3(const char *tag, float kp, float ki, float kd)
{
    int32_t kp_i = float_to_scaled(kp);
    int32_t ki_i = float_to_scaled(ki);
    int32_t kd_i = float_to_scaled(kd);
    int32_t kp_abs = (kp_i < 0) ? -kp_i : kp_i;
    int32_t ki_abs = (ki_i < 0) ? -ki_i : ki_i;
    int32_t kd_abs = (kd_i < 0) ? -kd_i : kd_i;
    (void)printf("[pid] %s kp=%c%ld.%03lu ki=%c%ld.%03lu kd=%c%ld.%03lu "
                 "(x1000=%ld,%ld,%ld)\r\n",
        tag,
        (kp_i < 0) ? '-' : '+',
        (long)(kp_abs / APP_BAL_PID_SCALE),
        (unsigned long)(kp_abs % APP_BAL_PID_SCALE),
        (ki_i < 0) ? '-' : '+',
        (long)(ki_abs / APP_BAL_PID_SCALE),
        (unsigned long)(ki_abs % APP_BAL_PID_SCALE),
        (kd_i < 0) ? '-' : '+',
        (long)(kd_abs / APP_BAL_PID_SCALE),
        (unsigned long)(kd_abs % APP_BAL_PID_SCALE),
        (long)kp_i, (long)ki_i, (long)kd_i);
}

static void print_pid_status(void)
{
    print_scaled3("rate",    s_bal.rate_pid.kp,    s_bal.rate_pid.ki,    s_bal.rate_pid.kd);
    print_scaled3("angle",   s_bal.angle_pid.kp,   s_bal.angle_pid.ki,   s_bal.angle_pid.kd);
    print_scaled3("speed",   s_bal.speed_pid.kp,   s_bal.speed_pid.ki,   s_bal.speed_pid.kd);
    print_scaled3("yaw",     s_bal.yaw_pid.kp,     s_bal.yaw_pid.ki,     s_bal.yaw_pid.kd);
    (void)printf("[pid] loops: rate 200Hz, angle 100Hz, yaw 50Hz, speed 20Hz\r\n");
}

static void print_pid_help(void)
{
    (void)printf("[pid] UART commands: rp/bp/sp/yp <kp_x1000> <ki_x1000> <kd_x1000>, "
                 "yi0/yi1, si0/si1, pid?, pid0, lt, lt0, t/test\r\n");
    (void)printf("[pid] rp=rate(200Hz) bp=angle(100Hz) sp=speed(20Hz) yp=yaw(50Hz)\r\n");
    (void)printf("[pid] yi0/yi1=yaw_invert  si0/si1=speed_invert(si?=query+v_meas)\r\n");
    (void)printf("[pid] example: rp 1800 0 0 ; bp 35000 2000 0 ; sp 5 0 0 ; yp 8000 0 0\r\n");
    (void)printf("[pid] sp note: speed fb is normalized by /10 (SCALE=10); "
                 "sp 5 at 0.1rev/s drift -> ~2.5deg tilt cmd\r\n");
}

static void send_lt_header(void)
{
    static const char header[] =
        "lt,t_ms,pitch_deg,left_target_rpm,right_target_rpm,"
        "left_actual_rpm,right_actual_rpm,left_actual_pwm,right_actual_pwm\r\n";
    (void)bsp_log_uart_try_write_async((const uint8_t *)header, sizeof(header) - 1u);
}

static void set_lt_stream_enabled(bool enabled)
{
    s_bal.lt_stream_enabled = enabled;
    (void)printf("[lt] high-rate CSV %s\r\n", enabled ? "on" : "off");
    if (enabled) {
        send_lt_header();
    }
}

static void send_lt_sample(uint32_t now_ms,
                           const app_balance_motion_cmd_t *cmd,
                           const bsp_motor_feedback_t *fb)
{
    if (!s_bal.lt_stream_enabled || (cmd == NULL) || (fb == NULL)) {
        return;
    }

    int32_t left_target_rpm_x100 = cps_to_rpm_x100(
        cmd->target_speed_cps, BSP_MOTOR_LEFT_COUNTS_PER_OUTPUT_REV);
    int32_t right_target_rpm_x100 = cps_to_rpm_x100(
        cmd->target_speed_cps, BSP_MOTOR_RIGHT_COUNTS_PER_OUTPUT_REV);
    int32_t left_actual_rpm_x100 = BAL_F2_X100(fb->left_speed_rpm);
    int32_t right_actual_rpm_x100 = BAL_F2_X100(fb->right_speed_rpm);

    char line[128];
    int n = snprintf(line, sizeof(line),
        "lt,%lu,%c%ld.%02lu,%c%ld.%02lu,%c%ld.%02lu,%c%ld.%02lu,%c%ld.%02lu,%d,%d\r\n",
        (unsigned long)now_ms,
        BAL_F2_S(s_bal.diag.pitch_meas_deg), (long)BAL_F2_I(s_bal.diag.pitch_meas_deg), (unsigned long)BAL_F2_F(s_bal.diag.pitch_meas_deg),
        scaled2_sign(left_target_rpm_x100), (long)scaled2_int_abs(left_target_rpm_x100), (unsigned long)scaled2_frac_abs(left_target_rpm_x100),
        scaled2_sign(right_target_rpm_x100), (long)scaled2_int_abs(right_target_rpm_x100), (unsigned long)scaled2_frac_abs(right_target_rpm_x100),
        scaled2_sign(left_actual_rpm_x100), (long)scaled2_int_abs(left_actual_rpm_x100), (unsigned long)scaled2_frac_abs(left_actual_rpm_x100),
        scaled2_sign(right_actual_rpm_x100), (long)scaled2_int_abs(right_actual_rpm_x100), (unsigned long)scaled2_frac_abs(right_actual_rpm_x100),
        (int)bsp_motor_get_left_actual_pwm(),
        (int)bsp_motor_get_right_actual_pwm());

    if ((n > 0) && ((size_t)n < sizeof(line))) {
        (void)bsp_log_uart_try_write_async((const uint8_t *)line, (size_t)n);
    }
}

static bool parse_pid_triplet(const char *args, float *kp, float *ki, float *kd)
{
    int32_t kp_i;
    int32_t ki_i;
    int32_t kd_i;
    const char *p = args;

    if (!parse_i32_token(&p, &kp_i) ||
        !parse_i32_token(&p, &ki_i) ||
        !parse_i32_token(&p, &kd_i)) {
        return false;
    }

    *kp = scaled_to_float(kp_i);
    *ki = scaled_to_float(ki_i);
    *kd = scaled_to_float(kd_i);
    return true;
}

static bool handle_pid_command(const char *cmd)
{
    if (cmd == NULL) {
        return false;
    }

    if ((cmd[0] == 'p') && (cmd[1] == 'i') && (cmd[2] == 'd') &&
        ((cmd[3] == '?') || (cmd[3] == '\0'))) {
        print_pid_status();
        return true;
    }

    if ((cmd[0] == 'p') && (cmd[1] == 'i') && (cmd[2] == 'd') &&
        (cmd[3] == '0') && (cmd[4] == '\0')) {
        app_balance_set_rate_gains(0.0f, 0.0f, 0.0f);
        app_balance_set_balance_gains(0.0f, 0.0f, 0.0f);
        app_balance_set_speed_gains(0.0f, 0.0f, 0.0f);
        app_balance_set_yaw_gains(0.0f, 0.0f, 0.0f);
        app_balance_reset();
        (void)printf("[pid] all gains cleared\r\n");
        return true;
    }

    if ((cmd[0] == 'l') && (cmd[1] == 't')) {
        if (cmd[2] == '\0') {
            set_lt_stream_enabled(true);
            return true;
        }
        if ((cmd[2] == '0') && (cmd[3] == '\0')) {
            set_lt_stream_enabled(false);
            return true;
        }
        if ((cmd[2] == '1') && (cmd[3] == '\0')) {
            set_lt_stream_enabled(true);
            return true;
        }
    }

    if ((cmd[0] == 'r') && (cmd[1] == 'p') && is_field_separator(cmd[2])) {
        float kp, ki, kd;
        if (!parse_pid_triplet(&cmd[2], &kp, &ki, &kd)) {
            (void)printf("[pid] bad rp command, use: rp 1000 0 0\r\n");
            return true;
        }
        app_balance_set_rate_gains(kp, ki, kd);
        app_balance_reset();
        print_scaled3("rate", kp, ki, kd);
        return true;
    }

    if ((cmd[0] == 'b') && (cmd[1] == 'p') && is_field_separator(cmd[2])) {
        float kp, ki, kd;
        if (!parse_pid_triplet(&cmd[2], &kp, &ki, &kd)) {
            (void)printf("[pid] bad bp command, use: bp 5000 0 0\r\n");
            return true;
        }
        app_balance_set_balance_gains(kp, ki, kd);
        app_balance_reset();
        print_scaled3("angle", kp, ki, kd);
        return true;
    }

    if ((cmd[0] == 's') && (cmd[1] == 'p') && is_field_separator(cmd[2])) {
        float kp, ki, kd;
        if (!parse_pid_triplet(&cmd[2], &kp, &ki, &kd)) {
            (void)printf("[pid] bad sp command, use: sp 2 0 0\r\n");
            return true;
        }
        app_balance_set_speed_gains(kp, ki, kd);
        app_balance_reset();
        print_scaled3("speed", kp, ki, kd);
        return true;
    }

    if ((cmd[0] == 'y') && (cmd[1] == 'p') && is_field_separator(cmd[2])) {
        float kp, ki, kd;
        if (!parse_pid_triplet(&cmd[2], &kp, &ki, &kd)) {
            (void)printf("[pid] bad yp command, use: yp 500 0 100\r\n");
            return true;
        }
        app_balance_set_yaw_gains(kp, ki, kd);
        reset_yaw_state();
        print_scaled3("yaw", kp, ki, kd);
        return true;
    }

    if ((cmd[0] == 'y') && (cmd[1] == 'i') &&
        ((cmd[2] == '\0') || (cmd[2] == '?'))) {
        (void)printf("[yaw] invert=%u\r\n",
                     app_balance_get_yaw_inverted() ? 1u : 0u);
        return true;
    }
    if ((cmd[0] == 'y') && (cmd[1] == 'i') &&
        ((cmd[2] == '0') || (cmd[2] == '1')) && (cmd[3] == '\0')) {
        app_balance_set_yaw_inverted(cmd[2] == '1');
        (void)printf("[yaw] invert=%u\r\n",
                     app_balance_get_yaw_inverted() ? 1u : 0u);
        return true;
    }

    /* si? / si0 / si1：速度反馈极性翻转查询 / 复位 / 启用。
     * 诊断：先发 pid0 把速度环增益清零，使小车漂移后看心跳 v= 值符号：
     *   v= 正值时向前漂移 → 方向正确 (si0)；v= 负值 → 需要翻转 (si1)。 */
    if ((cmd[0] == 's') && (cmd[1] == 'i') &&
        ((cmd[2] == '\0') || (cmd[2] == '?'))) {
        (void)printf("[speed] invert=%u (v_meas=%ldcps)\r\n",
                     app_balance_get_speed_inverted() ? 1u : 0u,
                     (long)s_bal.diag.speed_meas_cps);
        return true;
    }
    if ((cmd[0] == 's') && (cmd[1] == 'i') &&
        ((cmd[2] == '0') || (cmd[2] == '1')) && (cmd[3] == '\0')) {
        app_balance_set_speed_inverted(cmd[2] == '1');
        (void)printf("[speed] invert=%u\r\n",
                     app_balance_get_speed_inverted() ? 1u : 0u);
        return true;
    }

    if ((cmd[0] == 'h') && (cmd[1] == '\0')) {
        print_pid_help();
        return true;
    }

    return false;
}

static void drain_log_uart_command_tail(void)
{
    uint8_t ch;
    while (bsp_log_uart_read_byte(&ch)) {
        if (is_line_delimiter(ch)) {
            break;
        }
    }
}

static bool request_motor_test_mode(void)
{
    (void)printf("[ctrl] motor test mode requested\r\n");
    bsp_motor_stop();
    bsp_motor_set_deadzone_comp_enabled(true);
    bsp_motor_enable(false);
    drain_log_uart_command_tail();
    return true;
}

static bool process_log_uart_commands(void)
{
    static char cmd_buf[APP_BAL_CMD_BUF_LEN];
    static uint8_t cmd_len = 0u;
    uint8_t ch;

    while (bsp_log_uart_read_byte(&ch)) {
        if ((ch >= (uint8_t)'A') && (ch <= (uint8_t)'Z')) {
            ch = (uint8_t)(ch - (uint8_t)'A' + (uint8_t)'a');
        }

        if (is_line_delimiter(ch)) {
            cmd_buf[cmd_len] = '\0';
            bool request_test = is_test_command(cmd_buf, cmd_len);
            cmd_len = 0u;
            if (request_test) {
                return request_motor_test_mode();
            }
            (void)handle_pid_command(cmd_buf);
            continue;
        }

        if ((ch >= (uint8_t)' ') && (ch <= (uint8_t)'~')) {
            if (cmd_len < (sizeof(cmd_buf) - 1u)) {
                cmd_buf[cmd_len++] = (char)ch;
                if (is_test_command(cmd_buf, cmd_len)) {
                    cmd_len = 0u;
                    return request_motor_test_mode();
                }
            } else {
                cmd_len = 0u;
            }
        } else {
            cmd_len = 0u;
        }
    }

    return false;
}

static const char *safety_state_to_str(app_safety_state_t s)
{
    switch (s) {
    case APP_SAFETY_DISARMED:     return "DISARM";
    case APP_SAFETY_ARMED:        return "ARMED";
    case APP_SAFETY_LOW_BAT_WARN: return "BAT_WARN";
    case APP_SAFETY_FALLEN:       return "FALLEN";
    case APP_SAFETY_LOW_BAT_STOP: return "BAT_STOP";
    default:                      return "?";
    }
}

bool app_balance_run(void)
{
    uint32_t tick_count    = 0u;
    uint32_t last_total_rx = bsp_k230_uart_total_rx();
    uint32_t last_enc_irq  = bsp_motor_get_enc_irq_count();
    ms901m_snapshot_t snap = { 0 };

    /* 电机极性修正：TB6612 驱动信号与电机安装方向反相（正 PWM → 物理后退）。
     * 软件 invert 等效于修正接线，使正 PWM = 物理前进 = 编码器增大，
     * 保证三者（PWM / 物理方向 / 编码器）符号一致，速度环可正常工作。
     * 对应地：pitch_sign 改回 +1（见 APP_BALANCE_PITCH_INVERT=0），
     * 数学等价于原 pitch_sign=-1+无 invert，平衡行为不变。 */
    bsp_motor_set_invert(true, true);

    /* 平衡车死区策略：完全禁用死区补偿重映射。
     *
     * running_dz 重映射在零点附近产生符号翻转跳变（±40 permille 瞬变），
     * 在 200Hz 内环下表现为持续颤动。对倒立摆而言此补偿弊大于利：
     *   - 小命令无效 → 倾角增大 → PID 输出增大 → 自然超越物理死区（积分效应）
     *   - 角速度内环加小 Ki 可进一步消除稳态死区导致的残余误差
     * 禁用后电机在极低命令时可能有短暂不响应，但平衡环自修复。 */
    bsp_motor_set_deadzone_comp_enabled(false);
    bsp_motor_set_calibration_mode(false);
    bsp_motor_set_static_dz_enabled(false);
    bsp_motor_set_running_dz_enabled(false);
    bsp_motor_set_dither_dz_enabled(false);

    /* 主循环上电默认无运动指令（K230 通讯接入后由 MOTION_CMD 帧覆盖） */
    app_balance_motion_cmd_t cmd = { .target_speed_cps = 0, .target_yaw_pm = 0 };

    print_pid_help();
    print_pid_status();

    for (;;) {
        if (!bsp_systick_consume_tick()) {
            __WFI();
            continue;
        }
        tick_count++;

        if (process_log_uart_commands()) {
            return true;
        }

        /* ---- 1 kHz：IMU drain + 电机 1 ms 节拍 ----------------------------- */
        uint8_t buf[APP_BAL_IMU_DRAIN_CHUNK];
        size_t  got = bsp_imu_uart_rx_pop_bulk(buf, sizeof(buf));
        if (got > 0u) {
            ms901m_feed_bytes(buf, got);
        }
        bsp_motor_update();

        /* ---- 200 Hz：四级级联控制 + 电机输出 ------------------------------ */
        if ((tick_count % APP_BAL_PHASE_RATE_TICKS) == 0u) {
            ms901m_get_snapshot(&snap);

            app_balance_attitude_t att = {
                .pitch_deg      = snap.pitch_deg,
                .pitch_rate_dps = snap.gy_dps,
                .yaw_deg        = snap.yaw_deg,
                .gz_dps         = snap.gz_dps,
                .attitude_valid = snap.has_attitude,
            };

            /* 20 Hz 速度外环（每 50 ticks）—— 最先跑，更新 target_tilt_deg */
            if ((tick_count % APP_BAL_PHASE_SPEED_TICKS) == 0u) {
                balance_step_speed(&cmd);
            }

            /* 100 Hz 角度环（每 10 ticks）—— 更新 target_rate_dps */
            if ((tick_count % APP_BAL_PHASE_ANGLE_TICKS) == 0u) {
                bsp_battery_update();
                balance_step_angle(&att);
            }

            /* 50 Hz 航向环（每 20 ticks）—— 更新 cached_yaw_corr_pm */
            if ((tick_count % APP_BAL_PHASE_YAW_TICKS) == 0u) {
                balance_step_yaw(&att, &cmd);
            }

            /* 200 Hz 角速度内环（每 5 ticks）—— 每拍都跑，输出 PWM */
            balance_step_rate(&att, &cmd);

            /* 100 Hz LT 采样 */
            if ((tick_count % APP_BAL_PHASE_ANGLE_TICKS) == 0u) {
                bsp_motor_feedback_t fb_lt;
                bsp_motor_get_feedback(&fb_lt);
                send_lt_sample(tick_count, &cmd, &fb_lt);
            }
        }

        /* ---- 5 Hz：LED_G 绿心跳 + LED_R 跌倒 / 低压告警 ------------------- */
        if ((tick_count % APP_BAL_PHASE_LED_TICKS) == 0u) {
            DL_GPIO_togglePins(BSP_LED_G_PORT, BSP_LED_G_PIN);

            app_safety_state_t st = app_safety_get_state();
            if (st == APP_SAFETY_FALLEN ||
                st == APP_SAFETY_LOW_BAT_STOP) {
                DL_GPIO_setPins(BSP_LED_R_PORT, BSP_LED_R_PIN);
            } else if (st == APP_SAFETY_LOW_BAT_WARN) {
                DL_GPIO_togglePins(BSP_LED_R_PORT, BSP_LED_R_PIN);
            } else {
                DL_GPIO_clearPins(BSP_LED_R_PORT, BSP_LED_R_PIN);
            }
        }

        /* ---- 1 Hz：XDS-UART 调试日志 -------------------------------------- */
        if (!s_bal.lt_stream_enabled && ((tick_count % APP_BAL_PHASE_LOG_TICKS) == 0u)) {
            uint32_t total_rx = bsp_k230_uart_total_rx();
            uint32_t delta_rx = total_rx - last_total_rx;
            last_total_rx = total_rx;

            uint32_t total_enc_irq = bsp_motor_get_enc_irq_count();
            uint32_t delta_enc_irq = total_enc_irq - last_enc_irq;
            last_enc_irq = total_enc_irq;
            bool encQuenched = bsp_motor_enc_irq_is_quenched();

            int32_t  left_cnt  = bsp_motor_get_left_count();
            int32_t  right_cnt = bsp_motor_get_right_count();

            app_balance_diag_t  diag;
            app_balance_get_diag(&diag);
            uint32_t batt_mv  = bsp_battery_get_mv();
            uint32_t log_ovr = bsp_log_uart_rx_overrun();

            (void)printf("[hb] t=%lus state=%s pitch=%c%ld.%02lu inv=%u tilt*=%c%ld.%02lu "
                         "pwm=%c%ld.%02lu yawErr=%c%ld.%02lu yawCorr=%d L=%ld R=%ld v=%ldcps "
                         "batt=%lumV ms901m_g=%lu/b=%lu log_ovr=%lu k230_rx=%lub/s "
                         "encL=%ld encR=%ld encISR=%lu/s%s\n",
                (unsigned long)(tick_count / 1000u),
                safety_state_to_str(app_safety_get_state()),
                BAL_F2_S(diag.pitch_meas_deg), (long)BAL_F2_I(diag.pitch_meas_deg), (unsigned long)BAL_F2_F(diag.pitch_meas_deg),
                app_balance_get_pitch_inverted() ? 1u : 0u,
                BAL_F2_S(diag.target_tilt_deg), (long)BAL_F2_I(diag.target_tilt_deg), (unsigned long)BAL_F2_F(diag.target_tilt_deg),
                BAL_F2_S(diag.balance_out_pwm), (long)BAL_F2_I(diag.balance_out_pwm), (unsigned long)BAL_F2_F(diag.balance_out_pwm),
                BAL_F2_S(diag.yaw_error_deg), (long)BAL_F2_I(diag.yaw_error_deg), (unsigned long)BAL_F2_F(diag.yaw_error_deg),
                (int)diag.yaw_correction_pm,
                (long)diag.left_cmd_pm, (long)diag.right_cmd_pm,
                (long)diag.speed_meas_cps,
                (unsigned long)batt_mv,
                (unsigned long)ms901m_good_frames(),
                (unsigned long)ms901m_bad_frames(),
                (unsigned long)log_ovr,
                (unsigned long)delta_rx,
                (long)left_cnt, (long)right_cnt,
                (unsigned long)delta_enc_irq,
                encQuenched ? " [ISR_QUENCH!]" : "");
        }
    }
}
