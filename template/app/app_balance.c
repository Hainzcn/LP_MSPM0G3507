/**
 * @file    app_balance.c
 * @brief   平衡车双环控制骨架实现，详见 app_balance.h。
 *
 * 本模块是"骨架"：
 *   - 完整的级联 PID 数据流；
 *   - 完整的 safety 集成；
 *   - 完整的诊断输出；
 *   - 但**所有 PID 增益默认 0**，业务层未 set_gains 之前电机不会动。
 *
 * 业务层在准备好上车整定时调一次 `app_balance_set_*_gains()` 即可让车工作。
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
    pid_t   speed_pid;       /* 外环：输入 cps 误差，输出目标 tilt deg */
    pid_t   balance_pid;     /* 内环：仅 P+I，D 项已分离到 balance_kd */
    pid_t   yaw_pid;         /* Yaw 角度环：输入 yaw 误差 °，输出差分 PWM permille */
    float   balance_kd;      /* 平衡内环 D 增益（直接乘陀螺仪角速率，绕开 LPF） */
    float   pitch_offset_deg;
    float   pitch_lpf_deg;
    float   speed_lpf_cps;   /* 速度反馈 EMA 低通：抑制编码器量化噪声 */
    int8_t  pitch_sign;      /* +1 正常，-1 传感器前后反装时软件翻转 */
    int8_t  yaw_sign;        /* +1 正常，-1 gz_dps/yaw_deg 符号翻转 */
    bool    pitch_lpf_valid;
    float   yaw_kp;          /* 转向开环系数 */
    float   yaw_target_deg;     /* [源 0] EKF 模式：锁定目标航向（°） */
    bool    yaw_target_valid;   /* [源 0] EKF 模式：首次有效 yaw 才初始化 */
    float   yaw_gz_integrated;  /* [源 1] 陀螺积分模式：累积偏航量（°） */
    bool    lt_stream_enabled;
    app_balance_diag_t diag;
} balance_state_t;

static balance_state_t s_bal;

static const float s_dt_sec =
    (float)APP_BALANCE_CONTROL_PERIOD_MS / 1000.0f;

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
                                   s_dt_sec);
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
    s_bal.yaw_gz_integrated += att->gz_dps * (float)s_bal.yaw_sign * s_dt_sec;

    float raw_corr  = pid_step(&s_bal.yaw_pid,
                                0.0f,
                                s_bal.yaw_gz_integrated,
                                s_dt_sec);
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
    pid_init(&s_bal.balance_pid);
    pid_init(&s_bal.yaw_pid);

    /* 速度外环：输出是"目标 tilt deg"，限幅 ±MAX_TILT_DEG */
    pid_set_output_limit(&s_bal.speed_pid,
        -(float)APP_BALANCE_MAX_TILT_DEG, (float)APP_BALANCE_MAX_TILT_DEG);
    pid_set_d_filter(&s_bal.speed_pid, APP_BALANCE_SPEED_D_FILTER_ALPHA);

    /* 平衡内环：P+I 输出限幅 ±MAX_PWM，Kd 留在 PID 外部直接用陀螺仪 */
    pid_set_output_limit(&s_bal.balance_pid,
        -(float)APP_BALANCE_MAX_PWM_PERMILLE, (float)APP_BALANCE_MAX_PWM_PERMILLE);
    s_bal.balance_kd = 0.0f;

    /* Yaw 角度环：输出是差分 PWM permille，限幅 ±YAW_MAX_CORRECTION */
    pid_set_output_limit(&s_bal.yaw_pid,
        -(float)APP_BALANCE_YAW_MAX_CORRECTION_PM,
         (float)APP_BALANCE_YAW_MAX_CORRECTION_PM);
    pid_set_d_filter(&s_bal.yaw_pid, APP_BALANCE_YAW_D_FILTER_ALPHA);

    s_bal.pitch_offset_deg = -0.0f;
    s_bal.speed_lpf_cps = 0.0f;
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
    pid_reset(&s_bal.balance_pid);
    s_bal.speed_lpf_cps = 0.0f;
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

void app_balance_set_balance_gains(float kp, float ki, float kd)
{
    pid_set_gains(&s_bal.balance_pid, kp, ki, 0.0f);
    s_bal.balance_kd = kd;
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

void app_balance_step(const app_balance_attitude_t *att,
                      const app_balance_motion_cmd_t *cmd)
{
    if ((att == NULL) || (cmd == NULL)) {
        return;
    }

    float pitch_meas = filter_pitch(apply_pitch_orientation(att->pitch_deg));

    /* ---- 1) safety tick：转交已经按车体坐标修正后的 attitude，拿状态 ---- */
    app_safety_attitude_t sa = {
        .pitch_deg = pitch_meas,
        .attitude_valid = att->attitude_valid,
    };
    (void)app_safety_tick(&sa);

    if (app_safety_is_startup_grace_active() ||
        !app_safety_can_drive() || !att->attitude_valid) {
        /* 不允许驱动：不调 set_output（静默期等待姿态稳定，故障态由 safety 下发 brake/coast）；
         * 同时 reset PID 内部历史，避免下次 ARMED 时 i_term / d 历史跨段污染。 */
        app_balance_reset();
        s_bal.diag.target_tilt_deg   = 0.0f;
        s_bal.diag.pitch_meas_deg    = sa.pitch_deg;
        s_bal.diag.balance_out_pwm   = 0.0f;
        s_bal.diag.yaw_correction_pm = 0;
        s_bal.diag.yaw_error_deg     = 0.0f;
        s_bal.diag.left_cmd_pm       = 0;
        s_bal.diag.right_cmd_pm      = 0;
        s_bal.diag.speed_meas_cps    = 0;
        s_bal.diag.driving           = false;
        return;
    }

    /* ---- 2) 速度外环（100 Hz）：输入 cps 误差，输出目标 tilt deg ---- */
    bsp_motor_feedback_t fb;
    bsp_motor_get_feedback(&fb);
    int32_t avg_cps = (fb.left_speed_cps + fb.right_speed_cps) / 2;

    /* 速度反馈 LPF：编码器 20ms 差分窗口在低速时量化噪声 ±50 cps，
     * 直通会被速度环放大后耦合到平衡内环。此 EMA 将速度外环带宽
     * 压到 ~1 Hz，远低于平衡内环 ~15 Hz，保证级联解耦。 */
    float spd_alpha = APP_BALANCE_SPEED_LPF_ALPHA;
    s_bal.speed_lpf_cps += spd_alpha * ((float)avg_cps - s_bal.speed_lpf_cps);

    float target_tilt_deg = pid_step(&s_bal.speed_pid,
        (float)cmd->target_speed_cps,
        s_bal.speed_lpf_cps,
        s_dt_sec);

    /* ---- 3) 平衡内环（PD 分离架构）----
     * P+I 项：由 PID 库按 LPF 后的 pitch_meas 计算（位置反馈，精度优先）。
     * D  项：直接取陀螺仪 pitch_rate_dps × pitch_sign，绕开 pitch LPF，
     *        延迟仅 ~5ms（MS901M 内部处理），比微分滤波角快 20 倍以上。
     *        公式与 PID 库 "D on measurement" 等价：d = -Kd × d(measured)/dt，
     *        其中 d(measured)/dt ≈ pitch_rate_dps × pitch_sign。 */
    float pi_out = pid_step(&s_bal.balance_pid,
        target_tilt_deg,
        pitch_meas,
        s_dt_sec);
    float gyro_d = -s_bal.balance_kd * att->pitch_rate_dps
                   * (float)s_bal.pitch_sign;
    float pwm_out = pi_out + gyro_d;
    if (pwm_out >  (float)APP_BALANCE_MAX_PWM_PERMILLE)
        pwm_out =  (float)APP_BALANCE_MAX_PWM_PERMILLE;
    if (pwm_out < -(float)APP_BALANCE_MAX_PWM_PERMILLE)
        pwm_out = -(float)APP_BALANCE_MAX_PWM_PERMILLE;

    /* ---- 4) 转向叠加 + Yaw 角度环差分：left += yaw_corr, right -= yaw_corr ---- */
    float yaw_pm     = (float)cmd->target_yaw_pm * s_bal.yaw_kp;
    int16_t yaw_corr = yaw_angle_step(att, cmd);
    int16_t left_pm  = clamp_pwm_pm(pwm_out - yaw_pm + (float)yaw_corr);
    int16_t right_pm = clamp_pwm_pm(pwm_out + yaw_pm - (float)yaw_corr);

#if APP_BALANCE_ZERO_BAND_PM > 0
    if (left_pm  > -APP_BALANCE_ZERO_BAND_PM && left_pm  < APP_BALANCE_ZERO_BAND_PM)
        left_pm  = 0;
    if (right_pm > -APP_BALANCE_ZERO_BAND_PM && right_pm < APP_BALANCE_ZERO_BAND_PM)
        right_pm = 0;
#endif

    bsp_motor_set_output(left_pm, right_pm);

    /* ---- 5) 诊断写回 ---- */
    s_bal.diag.target_tilt_deg   = target_tilt_deg;
    s_bal.diag.pitch_meas_deg    = pitch_meas;
    s_bal.diag.balance_out_pwm   = pwm_out;
    s_bal.diag.left_cmd_pm       = left_pm;
    s_bal.diag.right_cmd_pm      = right_pm;
    s_bal.diag.speed_meas_cps    = (int32_t)s_bal.speed_lpf_cps;
    s_bal.diag.driving           = true;
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
#define APP_BAL_PHASE_CTRL_TICKS    APP_BALANCE_CONTROL_PERIOD_MS  /* 100 Hz */
#define APP_BAL_PHASE_LED_TICKS     200u                            /* 5 Hz */
#define APP_BAL_PHASE_LOG_TICKS     1000u                           /* 1 Hz */

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
    print_scaled3("balance", s_bal.balance_pid.kp, s_bal.balance_pid.ki, s_bal.balance_kd);
    (void)printf("[pid] balance D-src=gyro_rate (bypasses pitch LPF)\r\n");
    print_scaled3("speed",   s_bal.speed_pid.kp,   s_bal.speed_pid.ki,   s_bal.speed_pid.kd);
    print_scaled3("yaw",     s_bal.yaw_pid.kp,     s_bal.yaw_pid.ki,     s_bal.yaw_pid.kd);
}

static void print_pid_help(void)
{
    (void)printf("[pid] UART commands: bp <kp_x1000> <ki_x1000> <kd_x1000>, "
                 "sp <kp_x1000> <ki_x1000> <kd_x1000>, "
                 "yp <kp_x1000> <ki_x1000> <kd_x1000>, "
                 "yi0/yi1, pid?, pid0, lt, lt0, t/test\r\n");
    (void)printf("[pid] example: bp 8000 0 1000 ; sp 2 0 0 ; yp 500 0 100 ; yi1\r\n");
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

    if ((cmd[0] == 'b') && (cmd[1] == 'p') && is_field_separator(cmd[2])) {
        float kp, ki, kd;
        if (!parse_pid_triplet(&cmd[2], &kp, &ki, &kd)) {
            (void)printf("[pid] bad bp command, use: bp 8000 0 1000\r\n");
            return true;
        }
        app_balance_set_balance_gains(kp, ki, kd);
        app_balance_reset();
        print_scaled3("balance", kp, ki, kd);
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

    /* 平衡车死区策略：禁用静摩擦 kick，仅用 running DZ 无条件映射。
     * 倒立摆物理特性保证：电机不响应 → 倾角增大 → PID 输出自然增大 → 超越静摩擦。
     * 无需状态机检测"是否已启动"，避免方向翻转时 kick 脉冲引发机械振颤。 */
    bsp_motor_set_calibration_mode(false);
    bsp_motor_set_static_dz_enabled(false);
    bsp_motor_set_running_dz_enabled(true);

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

        /* ---- 1 kHz：IMU drain + 电机 1 ms 节拍 -----------------------------
         *   IMU UART RX 半满中断已把字节排入 256 B 环缓，本拍只做拷贝 + 解析；
         *   bsp_motor_update() 必须 1 kHz 调（QEI 软扩 + brake_pulse 倒计时）。 */
        uint8_t buf[APP_BAL_IMU_DRAIN_CHUNK];
        size_t  got = bsp_imu_uart_rx_pop_bulk(buf, sizeof(buf));
        if (got > 0u) {
            ms901m_feed_bytes(buf, got);
        }
        bsp_motor_update();

        /* ---- 100 Hz：电池采样 + 控制环（safety + balance step） ------------ */
        if ((tick_count % APP_BAL_PHASE_CTRL_TICKS) == 0u) {
            bsp_battery_update();
            ms901m_get_snapshot(&snap);

            app_balance_attitude_t att = {
                .pitch_deg      = snap.pitch_deg,
                .pitch_rate_dps = snap.gy_dps,
                .yaw_deg        = snap.yaw_deg,
                .gz_dps         = snap.gz_dps,
                .attitude_valid = snap.has_attitude,
            };
            app_balance_step(&att, &cmd);

            bsp_motor_feedback_t fb_lt;
            bsp_motor_get_feedback(&fb_lt);
            send_lt_sample(tick_count, &cmd, &fb_lt);
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
