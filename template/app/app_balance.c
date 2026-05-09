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
    pid_t   balance_pid;     /* 内环：输入 tilt 误差 deg，输出 PWM permille */
    float   pitch_offset_deg;
    float   yaw_kp;          /* 转向开环系数 */
    app_balance_diag_t diag;
} balance_state_t;

static balance_state_t s_bal;

static const float s_dt_sec =
    (float)APP_BALANCE_CONTROL_PERIOD_MS / 1000.0f;

/* -------------------------------------------------------------------------- */
/* 内部辅助                                                                    */
/* -------------------------------------------------------------------------- */

static int16_t clamp_pwm_pm(float v)
{
    if (v >  (float)APP_BALANCE_MAX_PWM_PERMILLE) v =  (float)APP_BALANCE_MAX_PWM_PERMILLE;
    if (v < -(float)APP_BALANCE_MAX_PWM_PERMILLE) v = -(float)APP_BALANCE_MAX_PWM_PERMILLE;
    return (int16_t)v;
}

/* -------------------------------------------------------------------------- */
/* 公共 API                                                                    */
/* -------------------------------------------------------------------------- */

void app_balance_init(void)
{
    pid_init(&s_bal.speed_pid);
    pid_init(&s_bal.balance_pid);

    /* 速度外环：输出是"目标 tilt deg"，限幅 ±MAX_TILT_DEG */
    pid_set_output_limit(&s_bal.speed_pid,
        -(float)APP_BALANCE_MAX_TILT_DEG, (float)APP_BALANCE_MAX_TILT_DEG);
    pid_set_d_filter(&s_bal.speed_pid, APP_BALANCE_SPEED_D_FILTER_ALPHA);

    /* 平衡内环：输出是 PWM permille，限幅 ±MAX_PWM */
    pid_set_output_limit(&s_bal.balance_pid,
        -(float)APP_BALANCE_MAX_PWM_PERMILLE, (float)APP_BALANCE_MAX_PWM_PERMILLE);
    pid_set_d_filter(&s_bal.balance_pid, APP_BALANCE_BALANCE_D_FILTER_ALPHA);

    s_bal.pitch_offset_deg = 0.0f;
    s_bal.yaw_kp = 1.0f;

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
}

void app_balance_set_pitch_offset(float deg)
{
    s_bal.pitch_offset_deg = deg;
}

void app_balance_set_balance_gains(float kp, float ki, float kd)
{
    pid_set_gains(&s_bal.balance_pid, kp, ki, kd);
}

void app_balance_set_speed_gains(float kp, float ki, float kd)
{
    pid_set_gains(&s_bal.speed_pid, kp, ki, kd);
}

void app_balance_set_yaw_kp(float kp_yaw)
{
    s_bal.yaw_kp = kp_yaw;
}

void app_balance_step(const app_balance_attitude_t *att,
                      const app_balance_motion_cmd_t *cmd)
{
    if ((att == NULL) || (cmd == NULL)) {
        return;
    }

    /* ---- 1) safety tick：转交 attitude，拿状态 ---- */
    app_safety_attitude_t sa = {
        .pitch_deg = att->pitch_deg - s_bal.pitch_offset_deg,
        .attitude_valid = att->attitude_valid,
    };
    (void)app_safety_tick(&sa);

    if (!app_safety_can_drive() || !att->attitude_valid) {
        /* 不允许驱动：不调 set_output（safety 已经下发了 brake / coast）；
         * 同时 reset PID 内部历史，避免下次 ARMED 时 i_term / d 历史跨段污染。 */
        app_balance_reset();
        s_bal.diag.target_tilt_deg = 0.0f;
        s_bal.diag.pitch_meas_deg  = sa.pitch_deg;
        s_bal.diag.balance_out_pwm = 0.0f;
        s_bal.diag.left_cmd_pm     = 0;
        s_bal.diag.right_cmd_pm    = 0;
        s_bal.diag.speed_meas_cps  = 0;
        s_bal.diag.driving         = false;
        return;
    }

    /* ---- 2) 速度外环（100 Hz）：输入 cps 误差，输出目标 tilt deg ---- */
    bsp_motor_feedback_t fb;
    bsp_motor_get_feedback(&fb);
    int32_t avg_cps = (fb.left_speed_cps + fb.right_speed_cps) / 2;

    /* 速度环的"测量值"是当前速度，"目标值"是 cmd->target_speed_cps */
    float target_tilt_deg = pid_step(&s_bal.speed_pid,
        (float)cmd->target_speed_cps,
        (float)avg_cps,
        s_dt_sec);

    /* ---- 3) 平衡内环（100 Hz）：输入 tilt 误差 deg，输出 PWM permille ----
     * pitch_meas = att->pitch_deg - offset；目标角是外环输出的 target_tilt_deg
     * 误差 = target_tilt - pitch_meas（即"还需要倾多少度"）
     * 注意：MS901M pitch 与车体倾倒方向的极性需要业务侧验证；如发现"加 Kp 后
     * 车自己倒"，把 Kp 取反或在外面对 att->pitch_deg 取反即可。 */
    float pitch_meas = att->pitch_deg - s_bal.pitch_offset_deg;
    float pwm_out = pid_step(&s_bal.balance_pid,
        target_tilt_deg,
        pitch_meas,
        s_dt_sec);

    /* ---- 4) 转向叠加：left -= yaw, right += yaw ---- */
    float yaw_pm = (float)cmd->target_yaw_pm * s_bal.yaw_kp;
    int16_t left_pm  = clamp_pwm_pm(pwm_out - yaw_pm);
    int16_t right_pm = clamp_pwm_pm(pwm_out + yaw_pm);

    bsp_motor_set_output(left_pm, right_pm);

    /* ---- 5) 诊断写回 ---- */
    s_bal.diag.target_tilt_deg = target_tilt_deg;
    s_bal.diag.pitch_meas_deg  = pitch_meas;
    s_bal.diag.balance_out_pwm = pwm_out;
    s_bal.diag.left_cmd_pm     = left_pm;
    s_bal.diag.right_cmd_pm    = right_pm;
    s_bal.diag.speed_meas_cps  = avg_cps;
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
#define APP_BAL_PHASE_CTRL_TICKS    APP_BALANCE_CONTROL_PERIOD_MS  /* 100 Hz */
#define APP_BAL_PHASE_LED_TICKS     200u                            /* 5 Hz */
#define APP_BAL_PHASE_LOG_TICKS     1000u                           /* 1 Hz */

/* 浮点字段格式化辅助（与 app_telemetry.c 同款，避开 AC6 printf("%f") 浮点路径） */
#define BAL_F2_X100(v)  ((int32_t)((v) * 100.0f + ((v) >= 0.0f ? 0.5f : -0.5f)))
#define BAL_F2_S(v)     (BAL_F2_X100(v) < 0 ? '-' : ' ')
#define BAL_F2_I(v)     ((int32_t)((BAL_F2_X100(v) < 0 ? -BAL_F2_X100(v) : BAL_F2_X100(v)) / 100))
#define BAL_F2_F(v)     ((uint32_t)((BAL_F2_X100(v) < 0 ? -BAL_F2_X100(v) : BAL_F2_X100(v)) % 100))

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

void app_balance_run(void)
{
    uint32_t tick_count    = 0u;
    uint32_t last_total_rx = bsp_k230_uart_total_rx();
    uint32_t last_enc_irq  = bsp_motor_get_enc_irq_count();
    ms901m_snapshot_t snap = { 0 };

    /* 主循环上电默认无运动指令（K230 通讯接入后由 MOTION_CMD 帧覆盖） */
    app_balance_motion_cmd_t cmd = { .target_speed_cps = 0, .target_yaw_pm = 0 };

    for (;;) {
        if (!bsp_systick_consume_tick()) {
            __WFI();
            continue;
        }
        tick_count++;

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
                .pitch_deg     = snap.pitch_deg,
                .pitch_rate_dps = snap.gy_dps,
                .attitude_valid = snap.has_attitude,
            };
            app_balance_step(&att, &cmd);
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
        if ((tick_count % APP_BAL_PHASE_LOG_TICKS) == 0u) {
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

            (void)printf("[hb] t=%lus state=%s pitch=%c%ld.%02lu tilt*=%c%ld.%02lu "
                         "pwm=%c%ld.%02lu L=%ld R=%ld v=%ldcps "
                         "batt=%lumV ms901m_g=%lu/b=%lu k230_rx=%lub/s "
                         "encL=%ld encR=%ld encISR=%lu/s btn=%lu/%lu%s\n",
                (unsigned long)(tick_count / 1000u),
                safety_state_to_str(app_safety_get_state()),
                BAL_F2_S(diag.pitch_meas_deg), (long)BAL_F2_I(diag.pitch_meas_deg), (unsigned long)BAL_F2_F(diag.pitch_meas_deg),
                BAL_F2_S(diag.target_tilt_deg), (long)BAL_F2_I(diag.target_tilt_deg), (unsigned long)BAL_F2_F(diag.target_tilt_deg),
                BAL_F2_S(diag.balance_out_pwm), (long)BAL_F2_I(diag.balance_out_pwm), (unsigned long)BAL_F2_F(diag.balance_out_pwm),
                (long)diag.left_cmd_pm, (long)diag.right_cmd_pm,
                (long)diag.speed_meas_cps,
                (unsigned long)batt_mv,
                (unsigned long)ms901m_good_frames(),
                (unsigned long)ms901m_bad_frames(),
                (unsigned long)delta_rx,
                (long)left_cnt, (long)right_cnt,
                (unsigned long)delta_enc_irq,
                (unsigned long)bsp_motor_get_button_irq_count(),
                (unsigned long)bsp_motor_get_button_poll_count(),
                encQuenched ? " [ISR_QUENCH!]" : "");
        }
    }
}
