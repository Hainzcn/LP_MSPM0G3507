/**
 * @file    app_balance.c
 * @brief   平衡车两级级联控制实现，详见 app_balance.h。
 *
 *   速度环 (20 Hz) → 角度环 (100 Hz) → 电机输出
 *   航向角环 (20 Hz) ─────────────────→ 差分叠加 ┘
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
#include "k230_protocol.h"
#include "ms901m.h"
#include "pid.h"
#include "ti_msp_dl_config.h"

#include <stdio.h>
#include <stddef.h>
#include <string.h>

/* -------------------------------------------------------------------------- */
/* 内部状态                                                                    */
/* -------------------------------------------------------------------------- */

typedef struct {
    pid2_t  angle_pid;          /* 角度环 100 Hz：输入 tilt 误差 deg，输出 PWM permille */
    pid2_t  speed_pid;          /* 速度外环 20 Hz：输入 cps 误差，输出目标 tilt deg */
    pid2_t  yaw_pid;            /* 航向角环 20 Hz：输入 yaw 误差，输出差分 PWM permille */
    float   target_tilt_deg;    /* 速度环输出：角度环目标角 */
    float   yaw_corr_pm;        /* 航向角环输出 */
    float   yaw_target_deg;     /* 源 0：锁定的目标航向（°） */
    float   yaw_gz_integrated;  /* 源 1：gz 积分累积偏航（°） */
    float   speed_lpf_cps;      /* 平均速度反馈 EMA 低通（归一化 cps） */
    float   pitch_offset_deg;   /* 直立零点 */
    int8_t  speed_sign;         /* +1 正常，-1 编码器方向反向 */
    int8_t  yaw_sign;           /* +1 正常，-1 yaw/gz 符号翻转 */
    bool    yaw_target_valid;   /* 源 0：是否已捕获直行目标航向 */
    bool    lt_stream_enabled;
    app_balance_diag_t diag;
} balance_state_t;

static const float s_dt_yaw_sec =
    (float)APP_BALANCE_YAW_PERIOD_MS / 1000.0f;

static balance_state_t s_bal;

#define APP_BAL_CMD_BUF_LEN  64u
#define APP_BAL_PID_SCALE    1000L

/* -------------------------------------------------------------------------- */
/* 内部辅助                                                                    */
/* -------------------------------------------------------------------------- */

/* 浮点字段格式化辅助（避开 AC6 printf("%f") 浮点路径；在全文件范围内使用） */
#define BAL_F2_X100(v)  ((int32_t)((v) * 100.0f + ((v) >= 0.0f ? 0.5f : -0.5f)))
#define BAL_F2_S(v)     (BAL_F2_X100(v) < 0 ? '-' : ' ')
#define BAL_F2_I(v)     ((int32_t)((BAL_F2_X100(v) < 0 ? -BAL_F2_X100(v) : BAL_F2_X100(v)) / 100))
#define BAL_F2_F(v)     ((uint32_t)((BAL_F2_X100(v) < 0 ? -BAL_F2_X100(v) : BAL_F2_X100(v)) % 100))

static int16_t clamp_pwm_pm(float v)
{
    if (v >  (float)APP_BALANCE_MAX_PWM_PERMILLE) v =  (float)APP_BALANCE_MAX_PWM_PERMILLE;
    if (v < -(float)APP_BALANCE_MAX_PWM_PERMILLE) v = -(float)APP_BALANCE_MAX_PWM_PERMILLE;
    return (int16_t)v;
}

static int16_t clamp_diff_pm(float v)
{
    if (v >  (float)APP_BALANCE_YAW_MAX_CORRECTION_PM) v =  (float)APP_BALANCE_YAW_MAX_CORRECTION_PM;
    if (v < -(float)APP_BALANCE_YAW_MAX_CORRECTION_PM) v = -(float)APP_BALANCE_YAW_MAX_CORRECTION_PM;
    return (int16_t)v;
}

/** 把角度差折叠到 [-180, 180] 区间，处理 yaw 环绕。 */
static float wrap_180(float deg)
{
    while (deg >  180.0f) { deg -= 360.0f; }
    while (deg < -180.0f) { deg += 360.0f; }
    return deg;
}

static void reset_yaw_state(void)
{
    pid2_reset(&s_bal.yaw_pid);
    s_bal.yaw_target_valid  = false;
    s_bal.yaw_gz_integrated = 0.0f;
    s_bal.yaw_corr_pm       = 0.0f;
    s_bal.diag.yaw_error_deg     = 0.0f;
    s_bal.diag.yaw_correction_pm = 0;
}

/* -------------------------------------------------------------------------- */
/* 公共 API                                                                    */
/* -------------------------------------------------------------------------- */

void app_balance_init(void)
{
    pid2_init(&s_bal.angle_pid);
    pid2_init(&s_bal.speed_pid);
    pid2_init(&s_bal.yaw_pid);

    /* 角度环 100 Hz：输出 PWM permille */
    s_bal.angle_pid.out_min    = -(float)APP_BALANCE_MAX_PWM_PERMILLE;
    s_bal.angle_pid.out_max    =  (float)APP_BALANCE_MAX_PWM_PERMILLE;
    s_bal.angle_pid.i_min      = -600.0f;
    s_bal.angle_pid.i_max      =  600.0f;
    s_bal.angle_pid.out_offset = APP_BALANCE_ANGLE_OUT_OFFSET;

    /* 速度外环 20 Hz：输出目标俯仰角（°） */
    s_bal.speed_pid.out_min    = -(float)APP_BALANCE_MAX_TILT_DEG;
    s_bal.speed_pid.out_max    =  (float)APP_BALANCE_MAX_TILT_DEG;
    s_bal.speed_pid.i_min      = -150.0f;
    s_bal.speed_pid.i_max      =  150.0f;
    s_bal.speed_pid.out_offset = APP_BALANCE_SPEED_OUT_OFFSET;

    /* 航向角环 20 Hz：输出差分 PWM permille */
    s_bal.yaw_pid.out_min    = -(float)APP_BALANCE_YAW_MAX_CORRECTION_PM;
    s_bal.yaw_pid.out_max    =  (float)APP_BALANCE_YAW_MAX_CORRECTION_PM;
    s_bal.yaw_pid.i_min      = -APP_BALANCE_YAW_INTEGRAL_LIMIT_PM;
    s_bal.yaw_pid.i_max      =  APP_BALANCE_YAW_INTEGRAL_LIMIT_PM;
    s_bal.yaw_pid.out_offset = 0.0f;

    s_bal.target_tilt_deg  = 0.0f;
    s_bal.yaw_corr_pm      = 0.0f;
    s_bal.speed_lpf_cps    = 0.0f;
    s_bal.pitch_offset_deg  = 0.8f;
    s_bal.speed_sign =
#if APP_BALANCE_SPEED_INVERT
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
    s_bal.yaw_target_valid  = false;
    s_bal.yaw_gz_integrated = 0.0f;
    s_bal.lt_stream_enabled = false;

    s_bal.diag.target_tilt_deg    = 0.0f;
    s_bal.diag.pitch_meas_deg     = 0.0f;
    s_bal.diag.balance_out_pwm    = 0.0f;
    s_bal.diag.yaw_error_deg      = 0.0f;
    s_bal.diag.yaw_correction_pm  = 0;
    s_bal.diag.left_cmd_pm        = 0;
    s_bal.diag.right_cmd_pm       = 0;
    s_bal.diag.speed_meas_cps     = 0;
    s_bal.diag.driving            = false;
}

void app_balance_reset(void)
{
    pid2_reset(&s_bal.angle_pid);
    pid2_reset(&s_bal.speed_pid);
    reset_yaw_state();
    s_bal.target_tilt_deg = 0.0f;
    s_bal.speed_lpf_cps   = 0.0f;
}

void app_balance_set_pitch_offset(float deg)
{
    s_bal.pitch_offset_deg = deg;
}

void app_balance_set_speed_inverted(bool inverted)
{
    s_bal.speed_sign = inverted ? (int8_t)-1 : (int8_t)1;
    pid2_reset(&s_bal.speed_pid);
    s_bal.speed_lpf_cps   = 0.0f;
    s_bal.target_tilt_deg = 0.0f;
}

bool app_balance_get_speed_inverted(void)
{
    return (s_bal.speed_sign < 0);
}

void app_balance_set_balance_gains(float kp, float ki, float kd, float out_offset)
{
    s_bal.angle_pid.kp         = kp;
    s_bal.angle_pid.ki         = ki;
    s_bal.angle_pid.kd         = kd;
    s_bal.angle_pid.out_offset = out_offset;
}

void app_balance_set_speed_gains(float kp, float ki, float kd, float out_offset)
{
    s_bal.speed_pid.kp         = kp;
    s_bal.speed_pid.ki         = ki;
    s_bal.speed_pid.kd         = kd;
    s_bal.speed_pid.out_offset = out_offset;
}

void app_balance_set_yaw_gains(float kp, float ki, float kd, float out_offset)
{
    s_bal.yaw_pid.kp         = kp;
    s_bal.yaw_pid.ki         = ki;
    s_bal.yaw_pid.kd         = kd;
    s_bal.yaw_pid.out_offset = out_offset;
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

void app_balance_get_diag(app_balance_diag_t *out)
{
    if (out == NULL) return;
    *out = s_bal.diag;
}

/* -------------------------------------------------------------------------- */
/* 多速率级联子函数                                                             */
/* -------------------------------------------------------------------------- */

/**
 * 航向角环一拍：输出差分 PWM（permille），正值 → 左轮加 / 右轮减。
 *
 * APP_BALANCE_YAW_SOURCE：
 *   0 = EKF yaw_deg，锁定首帧目标角 + virtual measured 避免 ±180° 跳变
 *   1 = gz_dps 积分，以归零积分量为目标；D 项等效 gz 阻尼
 *
 * target_yaw_pm != 0（K230 转向指令）时暂停航向闭环并刷新目标角（源 0）
 * 或清零积分（源 1），避免与开环转向对抗。
 */
static float yaw_angle_step(const app_balance_attitude_t *att,
                             const app_balance_motion_cmd_t *cmd)
{
#if !APP_BALANCE_YAW_ENABLED
    (void)att;
    (void)cmd;
    reset_yaw_state();
    return 0.0f;

#elif APP_BALANCE_YAW_SOURCE == 0
    if ((att == NULL) || (cmd == NULL)) {
        reset_yaw_state();
        return 0.0f;
    }

    float yaw_deg = att->yaw_deg * (float)s_bal.yaw_sign;

    if (cmd->target_yaw_pm != 0) {
        pid2_reset(&s_bal.yaw_pid);
        s_bal.yaw_target_deg   = yaw_deg;
        s_bal.yaw_target_valid = true;
        s_bal.diag.yaw_error_deg     = 0.0f;
        s_bal.yaw_corr_pm            = 0.0f;
        return 0.0f;
    }

    if (!s_bal.yaw_target_valid) {
        s_bal.yaw_target_deg   = yaw_deg;
        s_bal.yaw_target_valid = true;
    }

    float err = wrap_180(s_bal.yaw_target_deg - yaw_deg);
    float virtual_meas = s_bal.yaw_target_deg - err;

    s_bal.yaw_pid.target = s_bal.yaw_target_deg;
    s_bal.yaw_pid.actual = virtual_meas;
    pid2_update(&s_bal.yaw_pid);

    s_bal.diag.yaw_error_deg = err;
    s_bal.yaw_corr_pm        = s_bal.yaw_pid.out;
    return s_bal.yaw_corr_pm;

#else /* APP_BALANCE_YAW_SOURCE == 1：陀螺积分 */
    if ((att == NULL) || (cmd == NULL)) {
        reset_yaw_state();
        return 0.0f;
    }

    if (cmd->target_yaw_pm != 0) {
        pid2_reset(&s_bal.yaw_pid);
        s_bal.yaw_gz_integrated      = 0.0f;
        s_bal.diag.yaw_error_deg     = 0.0f;
        s_bal.yaw_corr_pm            = 0.0f;
        return 0.0f;
    }

    float gz = att->gz_dps * (float)s_bal.yaw_sign;
    s_bal.yaw_gz_integrated += gz * s_dt_yaw_sec;

    s_bal.yaw_pid.target = 0.0f;
    s_bal.yaw_pid.actual = s_bal.yaw_gz_integrated;
    pid2_update(&s_bal.yaw_pid);

    s_bal.diag.yaw_error_deg = -s_bal.yaw_gz_integrated;
    s_bal.yaw_corr_pm        = s_bal.yaw_pid.out;
    return s_bal.yaw_corr_pm;
#endif
}

/**
 * 速度/外环（20 Hz）：
 *   - SpeedPID：输入 cps 误差，输出 target_tilt_deg（角度环 setpoint）
 *   - YawPID   ：航向角闭环（见 yaw_angle_step）
 *
 * EKF 加速度耦合门控：若 |a_mag - 1g| > 0.15g（剧烈加速/减速），
 * 本拍跳过 SpeedPID 更新，防 EKF pitch 瞬时耦合污染速度环积分。
 */
static void balance_step_speed(const app_balance_attitude_t *att,
                                const app_balance_motion_cmd_t *cmd)
{
    bsp_motor_feedback_t fb;
    bsp_motor_get_feedback(&fb);

    /* 量纲归一化：除以 SCALE（默认 10），避免大 cps 直通后 PID 发散 */
    int32_t avg_cps_raw = ((fb.left_speed_cps + fb.right_speed_cps) / 2)
                          * (int32_t)s_bal.speed_sign;
    float norm_cps = (float)avg_cps_raw / (float)APP_BALANCE_SPEED_CPS_SCALE;

    s_bal.speed_lpf_cps += APP_BALANCE_SPEED_LPF_ALPHA *
                           (norm_cps - s_bal.speed_lpf_cps);

    /* EKF 加速度耦合门控：剧烈机动时跳过 SpeedPID（保护积分不受污染） */
    float a2 = (att->ax_g * att->ax_g) +
               (att->ay_g * att->ay_g) +
               (att->az_g * att->az_g);
    float a_err = a2 - 1.0f;
    if (a_err < 0.0f) a_err = -a_err;
    bool accel_ok = (a_err < (2.0f * 0.15f));   /* |a_mag - 1g| < 0.15g */

    if (accel_ok) {
        float target_norm = (float)cmd->target_speed_cps / (float)APP_BALANCE_SPEED_CPS_SCALE;
        s_bal.speed_pid.target = target_norm;
        s_bal.speed_pid.actual = s_bal.speed_lpf_cps;
        pid2_update(&s_bal.speed_pid);
        s_bal.target_tilt_deg = s_bal.speed_pid.out;
    }

    (void)yaw_angle_step(att, cmd);

    s_bal.diag.target_tilt_deg   = s_bal.target_tilt_deg;
    s_bal.diag.yaw_correction_pm = clamp_diff_pm(s_bal.yaw_corr_pm);
    s_bal.diag.speed_meas_cps    = (int32_t)s_bal.speed_lpf_cps;
}

/**
 * 角度环（100 Hz）：
 *   1. safety tick（跌倒检测 + 电池检查）
 *   2. 用 pid2_update(AnglePID) 输出 PWM
 *   3. 混合转向差分，分发左右轮命令
 */
static void balance_step_angle(const app_balance_attitude_t *att,
                                const app_balance_motion_cmd_t *cmd)
{
    /* safety tick（以 pitch 误差 deg 为输入，已减零点并按安装方向修正符号） */
    float pitch_meas = att->pitch_deg - s_bal.pitch_offset_deg;
#if APP_BALANCE_PITCH_INVERT
    pitch_meas = -pitch_meas;
#endif
    app_safety_attitude_t sa = {
        .pitch_deg      = pitch_meas,
        .attitude_valid = att->attitude_valid,
    };
    (void)app_safety_tick(&sa);

    s_bal.diag.pitch_meas_deg = pitch_meas;

    /* 跌倒 / 安全态不允许驱动时清空状态机并停输出 */
    if (app_safety_is_startup_grace_active() ||
        !app_safety_can_drive() || !att->attitude_valid) {
        app_balance_reset();
        s_bal.diag.balance_out_pwm   = 0.0f;
        s_bal.diag.left_cmd_pm       = 0;
        s_bal.diag.right_cmd_pm      = 0;
        s_bal.diag.driving           = false;
        (void)cmd;
        return;
    }

    /* 追加 APP_BALANCE_FALL_PITCH_DEG 早期跌倒保护（50°，比 safety 模块的 60° 严格） */
    float pitch_abs = (pitch_meas < 0.0f) ? -pitch_meas : pitch_meas;
    if (pitch_abs > APP_BALANCE_FALL_PITCH_DEG) {
        app_safety_disarm();
        app_balance_reset();
        s_bal.diag.balance_out_pwm = 0.0f;
        s_bal.diag.left_cmd_pm     = 0;
        s_bal.diag.right_cmd_pm    = 0;
        s_bal.diag.driving         = false;
        return;
    }

    /* 角度环：target = speed 外环输出，actual = 实测俯仰偏差 */
    s_bal.angle_pid.target = s_bal.target_tilt_deg;
    s_bal.angle_pid.actual = pitch_meas;
    pid2_update(&s_bal.angle_pid);
    float ave_pwm = s_bal.angle_pid.out;

    /* 航向差分：left = ave + yaw/2, right = ave - yaw/2 */
    float dif_half = s_bal.yaw_corr_pm * 0.5f;
    int16_t left_pm  = clamp_pwm_pm(ave_pwm + dif_half);
    int16_t right_pm = clamp_pwm_pm(ave_pwm - dif_half);

    bsp_motor_set_output(left_pm, right_pm);

    s_bal.diag.balance_out_pwm = ave_pwm;
    s_bal.diag.left_cmd_pm     = left_pm;
    s_bal.diag.right_cmd_pm    = right_pm;
    s_bal.diag.driving         = true;
}

/* ========================================================================== */
/* 上电自校零：pitch_offset_deg                                                 */
/* ========================================================================== */

/**
 * 上电静止采样自校零：阻塞 APP_BALANCE_PITCH_AUTOZERO_MS 毫秒，
 * 仅校准 pitch_offset_deg（新 2 级架构无角速度内环，无需 gyro bias 补偿）。
 *
 * 静止判据：|gy_dps| ≤ DEADBAND_DPS 且 |a_mag² - 1| ≤ 2×ACC_DEVIATION_G
 * 容错：有效样本 < 20 则沿用 init 默认值，日志打印 FAILED。
 */
static void auto_zero_pitch_offset(void)
{
#if APP_BALANCE_PITCH_AUTOZERO_MS == 0
    (void)printf("[autocal] disabled; pitch_offset=%c%ld.%02lu deg\r\n",
                 BAL_F2_S(s_bal.pitch_offset_deg),
                 (long)BAL_F2_I(s_bal.pitch_offset_deg),
                 (unsigned long)BAL_F2_F(s_bal.pitch_offset_deg));
    return;
#else
    const uint32_t end_ms       = bsp_systick_get_ms() + APP_BALANCE_PITCH_AUTOZERO_MS;
    float    sum_pitch           = 0.0f;
    uint32_t n_samples           = 0u;
    uint32_t n_rejected          = 0u;
    uint32_t last_sample_ms      = bsp_systick_get_ms();

    (void)printf("[autocal] sampling pitch for %lums (keep car upright and STILL)\r\n",
                 (unsigned long)APP_BALANCE_PITCH_AUTOZERO_MS);

    while ((int32_t)(end_ms - bsp_systick_get_ms()) > 0) {
        uint8_t imu_buf[64u];
        size_t  got = bsp_imu_uart_rx_pop_bulk(imu_buf, sizeof(imu_buf));
        if (got > 0u) {
            ms901m_feed_bytes(imu_buf, got);
        }

        uint32_t now_ms = bsp_systick_get_ms();
        if ((now_ms - last_sample_ms) >= 10u) {
            last_sample_ms = now_ms;

            ms901m_snapshot_t s;
            ms901m_get_snapshot(&s);
            if (!s.has_attitude || !s.has_gyro_acc) {
                continue;
            }

            float gy_abs = (s.gy_dps < 0.0f) ? -s.gy_dps : s.gy_dps;
            float gz_abs = (s.gz_dps < 0.0f) ? -s.gz_dps : s.gz_dps;
            if ((gy_abs > APP_BALANCE_PITCH_AUTOZERO_RATE_DEADBAND_DPS) ||
                (gz_abs > APP_BALANCE_PITCH_AUTOZERO_RATE_DEADBAND_DPS)) {
                n_rejected++;
                continue;
            }

            float a2     = (s.ax_g * s.ax_g) + (s.ay_g * s.ay_g) + (s.az_g * s.az_g);
            float a2_dev = a2 - 1.0f;
            if (a2_dev < 0.0f) a2_dev = -a2_dev;
            if (a2_dev > (2.0f * APP_BALANCE_PITCH_AUTOZERO_ACC_DEVIATION_G)) {
                n_rejected++;
                continue;
            }

            sum_pitch += s.pitch_deg;
            n_samples++;
        }

        bsp_systick_delay_ms(1u);
    }

    if (n_samples >= 20u) {
        float new_offset = sum_pitch / (float)n_samples;
        app_balance_set_pitch_offset(new_offset);
        (void)printf("[autocal] OK pitch_offset=%c%ld.%02lu deg (n=%lu, rejected=%lu)\r\n",
                     BAL_F2_S(new_offset),
                     (long)BAL_F2_I(new_offset),
                     (unsigned long)BAL_F2_F(new_offset),
                     (unsigned long)n_samples,
                     (unsigned long)n_rejected);
    } else {
        (void)printf("[autocal] FAILED n=%lu rejected=%lu; keep car still & power-cycle. "
                     "Using default pitch_offset=%c%ld.%02lu deg\r\n",
                     (unsigned long)n_samples,
                     (unsigned long)n_rejected,
                     BAL_F2_S(s_bal.pitch_offset_deg),
                     (long)BAL_F2_I(s_bal.pitch_offset_deg),
                     (unsigned long)BAL_F2_F(s_bal.pitch_offset_deg));
    }
#endif
}

/* ========================================================================== */
/* Stage 2.2 上车基线主循环                                                     */
/* ========================================================================== */

#define APP_BAL_IMU_DRAIN_CHUNK  64u
#define APP_BAL_PHASE_ANGLE_TICKS   APP_BALANCE_ANGLE_PERIOD_MS    /* 100 Hz */
#define APP_BAL_PHASE_SPEED_TICKS   APP_BALANCE_SPEED_PERIOD_MS    /*  20 Hz */
#define APP_BAL_PHASE_LED_TICKS     200u                            /*   5 Hz */
#define APP_BAL_PHASE_LOG_TICKS     1000u                           /*   1 Hz */

static int32_t cps_to_rpm_x100(int32_t cps, int32_t counts_per_rev)
{
    if (counts_per_rev <= 0) return 0;
    return (int32_t)((cps * 6000L) / counts_per_rev);
}

static char scaled2_sign(int32_t x100) { return (x100 < 0) ? '-' : '+'; }
static int32_t  scaled2_int_abs(int32_t x100)  { return ((x100 < 0) ? -x100 : x100) / 100; }
static uint32_t scaled2_frac_abs(int32_t x100) { return (uint32_t)(((x100 < 0) ? -x100 : x100) % 100); }

static bool is_test_command(const char *buf, uint8_t len)
{
    if ((len == 1u) && (buf[0] == 't')) return true;
    return (len == 4u) && (buf[0]=='t') && (buf[1]=='e') && (buf[2]=='s') && (buf[3]=='t');
}

static bool is_line_delimiter(uint8_t ch)
{
    return (ch == (uint8_t)'\r') || (ch == (uint8_t)'\n');
}

static bool is_field_separator(char ch)
{
    return (ch == ' ') || (ch == '\t') || (ch == ',') || (ch == ';') || (ch == ':');
}

static void skip_separators(const char **p)
{
    while ((*p != NULL) && is_field_separator(**p)) (*p)++;
}

static bool parse_i32_token(const char **p, int32_t *out)
{
    if ((p == NULL) || (*p == NULL) || (out == NULL)) return false;
    skip_separators(p);
    int32_t sign = 1;
    if (**p == '-') { sign = -1; (*p)++; }
    else if (**p == '+') { (*p)++; }
    if ((**p < '0') || (**p > '9')) return false;
    int32_t v = 0;
    while ((**p >= '0') && (**p <= '9')) { v = v * 10 + (int32_t)(**p - '0'); (*p)++; }
    *out = v * sign;
    return true;
}

static float scaled_to_float(int32_t x1000) { return (float)x1000 / (float)APP_BAL_PID_SCALE; }
static int32_t float_to_scaled(float v)
{
    float s = v * (float)APP_BAL_PID_SCALE;
    return (int32_t)(s + (s >= 0.0f ? 0.5f : -0.5f));
}

static void print_pid2_status(const char *tag, const pid2_t *p)
{
    int32_t kp_i = float_to_scaled(p->kp);
    int32_t ki_i = float_to_scaled(p->ki);
    int32_t kd_i = float_to_scaled(p->kd);
    int32_t of_i = (int32_t)(p->out_offset + 0.5f);
    int32_t kp_abs = (kp_i < 0) ? -kp_i : kp_i;
    int32_t ki_abs = (ki_i < 0) ? -ki_i : ki_i;
    int32_t kd_abs = (kd_i < 0) ? -kd_i : kd_i;
    (void)printf("[pid] %s kp=%c%ld.%03lu ki=%c%ld.%03lu kd=%c%ld.%03lu offset=%ldpm "
                 "(x1000=%ld,%ld,%ld)\r\n",
        tag,
        (kp_i < 0) ? '-' : '+', (long)(kp_abs / APP_BAL_PID_SCALE), (unsigned long)(kp_abs % APP_BAL_PID_SCALE),
        (ki_i < 0) ? '-' : '+', (long)(ki_abs / APP_BAL_PID_SCALE), (unsigned long)(ki_abs % APP_BAL_PID_SCALE),
        (kd_i < 0) ? '-' : '+', (long)(kd_abs / APP_BAL_PID_SCALE), (unsigned long)(kd_abs % APP_BAL_PID_SCALE),
        (long)of_i,
        (long)kp_i, (long)ki_i, (long)kd_i);
}

static void print_pid_status(void)
{
    print_pid2_status("angle", &s_bal.angle_pid);
    print_pid2_status("speed", &s_bal.speed_pid);
    print_pid2_status("yaw",   &s_bal.yaw_pid);
    (void)printf("[pid] loops: angle 100Hz, speed/yaw 20Hz | yaw_src=%d invert=%u\r\n",
                 (int)APP_BALANCE_YAW_SOURCE,
                 app_balance_get_yaw_inverted() ? 1u : 0u);
}

static void print_pid_help(void)
{
    (void)printf("[pid] commands: bp/sp/yp <kp_x1000> <ki_x1000> <kd_x1000> <offset_pm>, "
                 "bo, si0/si1, yi0/yi1, pid?, pid0, lt, lt0, t/test\r\n");
    (void)printf("[pid] bp=angle(100Hz) sp=speed(20Hz) yp=yaw_angle(20Hz) bo=angle_offset_only\r\n");
    (void)printf("[pid] offset_pm: direct permille, NOT x1000 (e.g. bo 80 = 80pm dead-zone comp)\r\n");
    (void)printf("[pid] example: bo 80 ; bp 5000 0 5000 80 ; sp 2000 50 0 0 ; yp 800 0 200 0\r\n");
    (void)printf("[pid] si?/si0/si1=speed_invert  yi?/yi0/yi1=yaw_invert\r\n");
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
    if (enabled) send_lt_header();
}

static void send_lt_sample(uint32_t now_ms,
                           const app_balance_motion_cmd_t *cmd,
                           const bsp_motor_feedback_t *fb)
{
    if (!s_bal.lt_stream_enabled || (cmd == NULL) || (fb == NULL)) return;

    int32_t lt_rpm = cps_to_rpm_x100(cmd->target_speed_cps, BSP_MOTOR_LEFT_COUNTS_PER_OUTPUT_REV);
    int32_t rt_rpm = cps_to_rpm_x100(cmd->target_speed_cps, BSP_MOTOR_RIGHT_COUNTS_PER_OUTPUT_REV);
    int32_t la_rpm = BAL_F2_X100(fb->left_speed_rpm);
    int32_t ra_rpm = BAL_F2_X100(fb->right_speed_rpm);

    char line[128];
    int n = snprintf(line, sizeof(line),
        "lt,%lu,%c%ld.%02lu,%c%ld.%02lu,%c%ld.%02lu,%c%ld.%02lu,%c%ld.%02lu,%d,%d\r\n",
        (unsigned long)now_ms,
        BAL_F2_S(s_bal.diag.pitch_meas_deg),
        (long)BAL_F2_I(s_bal.diag.pitch_meas_deg), (unsigned long)BAL_F2_F(s_bal.diag.pitch_meas_deg),
        scaled2_sign(lt_rpm), (long)scaled2_int_abs(lt_rpm), (unsigned long)scaled2_frac_abs(lt_rpm),
        scaled2_sign(rt_rpm), (long)scaled2_int_abs(rt_rpm), (unsigned long)scaled2_frac_abs(rt_rpm),
        scaled2_sign(la_rpm), (long)scaled2_int_abs(la_rpm), (unsigned long)scaled2_frac_abs(la_rpm),
        scaled2_sign(ra_rpm), (long)scaled2_int_abs(ra_rpm), (unsigned long)scaled2_frac_abs(ra_rpm),
        (int)bsp_motor_get_left_actual_pwm(),
        (int)bsp_motor_get_right_actual_pwm());

    if ((n > 0) && ((size_t)n < sizeof(line))) {
        (void)bsp_log_uart_try_write_async((const uint8_t *)line, (size_t)n);
    }
}

/**
 * 解析 4 参数 PID 指令：kp_x1000 ki_x1000 kd_x1000 offset_direct
 * offset 是直接 permille 值（不乘以 1000）。
 */
static bool parse_pid_quad(const char *args, float *kp, float *ki, float *kd, float *offset)
{
    int32_t kp_i, ki_i, kd_i, off_i;
    const char *p = args;
    if (!parse_i32_token(&p, &kp_i) ||
        !parse_i32_token(&p, &ki_i) ||
        !parse_i32_token(&p, &kd_i)) {
        return false;
    }
    /* offset 是可选的，默认 0 */
    if (!parse_i32_token(&p, &off_i)) off_i = 0;
    *kp     = scaled_to_float(kp_i);
    *ki     = scaled_to_float(ki_i);
    *kd     = scaled_to_float(kd_i);
    *offset = (float)off_i;
    return true;
}

static bool handle_pid_command(const char *cmd)
{
    if (cmd == NULL) return false;

    /* pid? / pid0 */
    if ((cmd[0]=='p') && (cmd[1]=='i') && (cmd[2]=='d')) {
        if ((cmd[3]=='?' || cmd[3]=='\0')) { print_pid_status(); return true; }
        if (cmd[3]=='0' && cmd[4]=='\0') {
            app_balance_set_balance_gains(0.0f, 0.0f, 0.0f, 0.0f);
            app_balance_set_speed_gains(0.0f, 0.0f, 0.0f, 0.0f);
            app_balance_set_yaw_gains(0.0f, 0.0f, 0.0f, 0.0f);
            app_balance_reset();
            (void)printf("[pid] all gains cleared\r\n");
            return true;
        }
    }

    /* lt / lt0 / lt1 */
    if ((cmd[0]=='l') && (cmd[1]=='t')) {
        if (cmd[2]=='\0') { set_lt_stream_enabled(true);  return true; }
        if ((cmd[2]=='0') && (cmd[3]=='\0')) { set_lt_stream_enabled(false); return true; }
        if ((cmd[2]=='1') && (cmd[3]=='\0')) { set_lt_stream_enabled(true);  return true; }
    }

    /* bo <offset>：单独调角度环 OutOffset */
    if ((cmd[0]=='b') && (cmd[1]=='o') && is_field_separator(cmd[2])) {
        int32_t off_i;
        const char *p = &cmd[2];
        if (!parse_i32_token(&p, &off_i)) {
            (void)printf("[pid] bad bo command, use: bo 80\r\n");
            return true;
        }
        s_bal.angle_pid.out_offset = (float)off_i;
        (void)printf("[pid] angle out_offset=%ldpm\r\n", (long)off_i);
        return true;
    }

    /* bp <kp_x1000> <ki_x1000> <kd_x1000> <offset_pm>：角度环 */
    if ((cmd[0]=='b') && (cmd[1]=='p') && is_field_separator(cmd[2])) {
        float kp, ki, kd, offset;
        if (!parse_pid_quad(&cmd[2], &kp, &ki, &kd, &offset)) {
            (void)printf("[pid] bad bp command, use: bp 5000 0 5000 80\r\n");
            return true;
        }
        app_balance_set_balance_gains(kp, ki, kd, offset);
        app_balance_reset();
        print_pid2_status("angle", &s_bal.angle_pid);
        return true;
    }

    /* sp <kp_x1000> <ki_x1000> <kd_x1000> <offset_pm>：速度环 */
    if ((cmd[0]=='s') && (cmd[1]=='p') && is_field_separator(cmd[2])) {
        float kp, ki, kd, offset;
        if (!parse_pid_quad(&cmd[2], &kp, &ki, &kd, &offset)) {
            (void)printf("[pid] bad sp command, use: sp 2000 50 0 0\r\n");
            return true;
        }
        app_balance_set_speed_gains(kp, ki, kd, offset);
        app_balance_reset();
        print_pid2_status("speed", &s_bal.speed_pid);
        return true;
    }

    /* yp：航向角环 */
    if ((cmd[0]=='y') && (cmd[1]=='p') && is_field_separator(cmd[2])) {
        float kp, ki, kd, offset;
        if (!parse_pid_quad(&cmd[2], &kp, &ki, &kd, &offset)) {
            (void)printf("[pid] bad yp command, use: yp 800 0 200 0\r\n");
            return true;
        }
        app_balance_set_yaw_gains(kp, ki, kd, offset);
        reset_yaw_state();
        print_pid2_status("yaw", &s_bal.yaw_pid);
        return true;
    }

    /* yi? / yi0 / yi1：航向反馈极性翻转 */
    if ((cmd[0]=='y') && (cmd[1]=='i')) {
        if (cmd[2]=='\0' || cmd[2]=='?') {
            (void)printf("[yaw] invert=%u yawErr=%c%ld.%02lu\r\n",
                         app_balance_get_yaw_inverted() ? 1u : 0u,
                         BAL_F2_S(s_bal.diag.yaw_error_deg),
                         (long)BAL_F2_I(s_bal.diag.yaw_error_deg),
                         (unsigned long)BAL_F2_F(s_bal.diag.yaw_error_deg));
            return true;
        }
        if ((cmd[2]=='0' || cmd[2]=='1') && cmd[3]=='\0') {
            app_balance_set_yaw_inverted(cmd[2] == '1');
            (void)printf("[yaw] invert=%u\r\n",
                         app_balance_get_yaw_inverted() ? 1u : 0u);
            return true;
        }
    }

    /* si? / si0 / si1：速度反馈极性翻转 */
    if ((cmd[0]=='s') && (cmd[1]=='i')) {
        if (cmd[2]=='\0' || cmd[2]=='?') {
            (void)printf("[speed] invert=%u (v_meas=%ldcps)\r\n",
                         app_balance_get_speed_inverted() ? 1u : 0u,
                         (long)s_bal.diag.speed_meas_cps);
            return true;
        }
        if ((cmd[2]=='0' || cmd[2]=='1') && cmd[3]=='\0') {
            app_balance_set_speed_inverted(cmd[2] == '1');
            (void)printf("[speed] invert=%u\r\n",
                         app_balance_get_speed_inverted() ? 1u : 0u);
            return true;
        }
    }

    /* h：帮助 */
    if (cmd[0]=='h' && cmd[1]=='\0') { print_pid_help(); return true; }

    return false;
}

static void drain_log_uart_command_tail(void)
{
    uint8_t ch;
    while (bsp_log_uart_read_byte(&ch)) {
        if (is_line_delimiter(ch)) break;
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
    static char    cmd_buf[APP_BAL_CMD_BUF_LEN];
    static uint8_t cmd_len = 0u;
    uint8_t ch;

    while (bsp_log_uart_read_byte(&ch)) {
        if ((ch >= (uint8_t)'A') && (ch <= (uint8_t)'Z')) {
            ch = (uint8_t)(ch - (uint8_t)'A' + (uint8_t)'a');
        }
        if (is_line_delimiter(ch)) {
            cmd_buf[cmd_len] = '\0';
            bool req_test = is_test_command(cmd_buf, cmd_len);
            cmd_len = 0u;
            if (req_test) return request_motor_test_mode();
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

/* ========================================================================== */
/* K230 通讯（Stage 4 IMU TX 一分二方案）                                      */
/* ========================================================================== */

#define K230_HEARTBEAT_TIMEOUT_MS   500u
#define K230_RX_DRAIN_CHUNK         64u

static k230_parser_t s_k230_parser;
static uint32_t      s_k230_last_rx_ms = 0u;
static bool          s_k230_online     = false;

static void k230_comm_init(void)
{
    k230_parser_init(&s_k230_parser);
    s_k230_last_rx_ms = bsp_systick_get_ms();
    s_k230_online     = false;
}

static void k230_drain_and_dispatch(app_balance_motion_cmd_t *cmd, uint32_t now_ms)
{
    uint8_t buf[K230_RX_DRAIN_CHUNK];
    size_t got = bsp_k230_uart_rx_pop_bulk(buf, sizeof(buf));
    for (size_t i = 0u; i < got; ++i) {
        if (!k230_parser_feed(&s_k230_parser, buf[i])) continue;
        s_k230_last_rx_ms = now_ms;
        s_k230_online     = true;

        switch (s_k230_parser.cmd) {
        case K230_CMD_MOTION_CMD:
            if (s_k230_parser.len == sizeof(k230_motion_cmd_t)) {
                k230_motion_cmd_t mc;
                memcpy(&mc, s_k230_parser.payload, sizeof(mc));
                cmd->target_speed_cps = (int32_t)mc.target_v;
                cmd->target_yaw_pm    = (int16_t)mc.target_omega;
            }
            break;

        case K230_CMD_PID_INJECT:
            if (s_k230_parser.len == sizeof(k230_pid_inject_t)) {
                k230_pid_inject_t pi;
                memcpy(&pi, s_k230_parser.payload, sizeof(pi));
                switch (pi.pid_id) {
                case 0u: app_balance_set_balance_gains(pi.kp, pi.ki, pi.kd, s_bal.angle_pid.out_offset); break;
                case 2u: app_balance_set_speed_gains(pi.kp, pi.ki, pi.kd, s_bal.speed_pid.out_offset);   break;
                case 3u: app_balance_set_yaw_gains(pi.kp, pi.ki, pi.kd, s_bal.yaw_pid.out_offset);
                         reset_yaw_state();
                         break;
                default: break;
                }
            }
            break;

        case K230_CMD_HEARTBEAT_K230:
            break;

        default:
            break;
        }
    }
}

static void k230_check_timeout(app_balance_motion_cmd_t *cmd, uint32_t now_ms)
{
    if ((now_ms - s_k230_last_rx_ms) > K230_HEARTBEAT_TIMEOUT_MS) {
        if (s_k230_online) {
            s_k230_online = false;
            (void)printf("[k230] heartbeat timeout -> zero cmd\r\n");
        }
        cmd->target_speed_cps = 0;
        cmd->target_yaw_pm    = 0;
    }
}

static void k230_send_vehicle_status(void)
{
    bsp_motor_feedback_t fb;
    bsp_motor_get_feedback(&fb);

    k230_vehicle_status_t vs;
    vs.avg_cps      = (fb.left_speed_cps + fb.right_speed_cps) / 2;
    vs.safety_state = (uint8_t)app_safety_get_state();
    vs.bat_mv       = (uint16_t)bsp_battery_get_mv();

    uint8_t frame[K230_PROTO_MAX_FRAME];
    size_t len = k230_encode_frame(K230_CMD_VEHICLE_STATUS,
        &vs, (uint8_t)sizeof(vs), frame, sizeof(frame));
    if (len > 0u) bsp_k230_uart_write_blocking(frame, len);
}

static void k230_send_heartbeat(uint32_t now_ms)
{
    k230_heartbeat_t hb;
    hb.uptime_ms = now_ms;

    uint8_t frame[K230_PROTO_MAX_FRAME];
    size_t len = k230_encode_frame(K230_CMD_HEARTBEAT_MCU,
        &hb, (uint8_t)sizeof(hb), frame, sizeof(frame));
    if (len > 0u) bsp_k230_uart_write_blocking(frame, len);
}

static void k230_flush_rx_buffer(void)
{
    uint8_t flush[64];
    while (bsp_k230_uart_rx_pop_bulk(flush, sizeof(flush)) > 0u) { /* discard */ }
}

/* ========================================================================== */
/* 主循环                                                                      */
/* ========================================================================== */

bool app_balance_run(void)
{
    uint32_t tick_count    = 0u;
    uint32_t last_total_rx = bsp_k230_uart_total_rx();
    uint32_t last_enc_irq  = bsp_motor_get_enc_irq_count();
    ms901m_snapshot_t snap = { 0 };

    /* 电机极性修正：正 PWM → 物理前进 → 编码器增大（三者符号一致） */
    bsp_motor_set_invert(true, true);

    /* 平衡车死区策略：BSP 死区补偿全部关闭，改由 pid2 out_offset 补偿。
     * BSP 重映射在零点产生符号翻转跳变，对倒立摆弊大于利。 */
    bsp_motor_set_deadzone_comp_enabled(false);
    bsp_motor_set_calibration_mode(false);
    bsp_motor_set_static_dz_enabled(false);
    bsp_motor_set_running_dz_enabled(false);
    bsp_motor_set_dither_dz_enabled(false);

    app_balance_motion_cmd_t cmd = { .target_speed_cps = 0, .target_yaw_pm = 0 };

    print_pid_help();
    print_pid_status();

    /* 上电自校零 pitch_offset：阻塞采样 1.5s，电机 PWM=0 */
    auto_zero_pitch_offset();

    /* 校零后清掉累积 K230 字节，刷新心跳时间戳防误报超时 */
    k230_flush_rx_buffer();
    k230_comm_init();

    for (;;) {
        if (!bsp_systick_consume_tick()) {
            __WFI();
            continue;
        }
        tick_count++;

        if (process_log_uart_commands()) return true;

        /* ---- 1 kHz：IMU drain + K230 drain + 电机 1 ms 节拍 --------------- */
        {
            uint8_t imu_buf[APP_BAL_IMU_DRAIN_CHUNK];
            size_t  got = bsp_imu_uart_rx_pop_bulk(imu_buf, sizeof(imu_buf));
            if (got > 0u) ms901m_feed_bytes(imu_buf, got);
        }
        k230_drain_and_dispatch(&cmd, tick_count);
        k230_check_timeout(&cmd, tick_count);
        bsp_motor_update();

        /* ---- 100 Hz：角度环 + safety ---------------------------------------- */
        if ((tick_count % APP_BAL_PHASE_ANGLE_TICKS) == 0u) {
            ms901m_get_snapshot(&snap);

            app_balance_attitude_t att = {
                .pitch_deg      = snap.pitch_deg,
                .pitch_rate_dps = snap.gy_dps,
                .yaw_deg        = snap.yaw_deg,
                .gz_dps         = snap.gz_dps,
                .ax_g           = snap.ax_g,
                .ay_g           = snap.ay_g,
                .az_g           = snap.az_g,
                .attitude_valid = snap.has_attitude,
            };

            bsp_battery_update();

            /* 20 Hz 速度/转向外环（每 50 ticks）—— 先跑，更新 target_tilt_deg */
            if ((tick_count % APP_BAL_PHASE_SPEED_TICKS) == 0u) {
                balance_step_speed(&att, &cmd);
                k230_send_vehicle_status();
            }

            /* 100 Hz 角度环 —— 消费 target_tilt_deg 和 yaw_corr_pm 输出 PWM */
            balance_step_angle(&att, &cmd);

            /* LT 采样 */
            {
                bsp_motor_feedback_t fb_lt;
                bsp_motor_get_feedback(&fb_lt);
                send_lt_sample(tick_count, &cmd, &fb_lt);
            }
        }

        /* ---- 5 Hz：LED 心跳 ------------------------------------------------- */
        if ((tick_count % APP_BAL_PHASE_LED_TICKS) == 0u) {
            DL_GPIO_togglePins(BSP_LED_G_PORT, BSP_LED_G_PIN);
            app_safety_state_t st = app_safety_get_state();
            if (st == APP_SAFETY_FALLEN || st == APP_SAFETY_LOW_BAT_STOP) {
                DL_GPIO_setPins(BSP_LED_R_PORT, BSP_LED_R_PIN);
            } else if (st == APP_SAFETY_LOW_BAT_WARN) {
                DL_GPIO_togglePins(BSP_LED_R_PORT, BSP_LED_R_PIN);
            } else {
                DL_GPIO_clearPins(BSP_LED_R_PORT, BSP_LED_R_PIN);
            }
        }

        /* ---- 1 Hz：调试日志 + K230 心跳 TX ---------------------------------- */
        if (!s_bal.lt_stream_enabled && ((tick_count % APP_BAL_PHASE_LOG_TICKS) == 0u)) {
            k230_send_heartbeat(tick_count);

            uint32_t total_rx  = bsp_k230_uart_total_rx();
            uint32_t delta_rx  = total_rx - last_total_rx;
            last_total_rx = total_rx;

            uint32_t total_enc = bsp_motor_get_enc_irq_count();
            uint32_t delta_enc = total_enc - last_enc_irq;
            last_enc_irq = total_enc;
            bool encQ = bsp_motor_enc_irq_is_quenched();

            int32_t left_cnt  = bsp_motor_get_left_count();
            int32_t right_cnt = bsp_motor_get_right_count();

            app_balance_diag_t diag;
            app_balance_get_diag(&diag);
            uint32_t batt_mv  = bsp_battery_get_mv();
            uint32_t log_ovr  = bsp_log_uart_rx_overrun();

            (void)printf("[hb] t=%lus state=%s pitch=%c%ld.%02lu tilt*=%c%ld.%02lu "
                         "pwm=%c%ld.%02lu yawErr=%c%ld.%02lu yawCorr=%d L=%ld R=%ld v=%ldcps "
                         "batt=%lumV ms901m_g=%lu/b=%lu log_ovr=%lu k230_rx=%lub/s "
                         "k230_g=%lu/b=%lu k230_%s "
                         "encL=%ld encR=%ld encISR=%lu/s%s\n",
                (unsigned long)(tick_count / 1000u),
                safety_state_to_str(app_safety_get_state()),
                BAL_F2_S(diag.pitch_meas_deg), (long)BAL_F2_I(diag.pitch_meas_deg), (unsigned long)BAL_F2_F(diag.pitch_meas_deg),
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
                (unsigned long)s_k230_parser.good_frames,
                (unsigned long)s_k230_parser.bad_frames,
                s_k230_online ? "ON" : "OFF",
                (long)left_cnt, (long)right_cnt,
                (unsigned long)delta_enc,
                encQ ? " [ISR_QUENCH!]" : "");
        }
    }
}
