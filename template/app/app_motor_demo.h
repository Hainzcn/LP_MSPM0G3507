/**
 * @file    app_motor_demo.h
 * @brief   阶段 2 电机驱动演示任务。
 */

#ifndef APP_MOTOR_DEMO_H
#define APP_MOTOR_DEMO_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 运行电机转动演示。
 * @return true = 收到返回装车模式请求，调用方应切回 app_balance_run()。
 *
 * 串口 `b` / `r` 用作急刹 / 启动切换；`l` 或 `load` 请求切入装车模式。
 */
bool app_motor_demo_run(void);

/**
 * @brief 手动配置 demo 目标转速。
 * @param rpm 目标空载转速，按 GB370 最大 620 rpm 钳位并换算为 PWM permille。
 */
void app_motor_demo_set_speed_rpm(uint16_t rpm);

/** 读取当前 demo 目标转速配置。 */
uint16_t app_motor_demo_get_speed_rpm(void);

typedef struct {
    bool    enabled;
    int16_t kp_pm_per_rpm;
    int16_t ki_pm_per_rpm_step;
    int16_t correction_pm;
    int16_t left_cmd_pm;
    int16_t right_cmd_pm;
    int16_t rpm_error;
} app_motor_demo_sync_diag_t;

/** 开关双轮同步服务；关闭时左右轮都输出目标转速对应的同一 PWM。 */
void app_motor_demo_set_sync_enabled(bool enabled);

/** 配置同步环增益：误差定义为 rpmR - rpmL，输出为左右差分 PWM 补偿。 */
void app_motor_demo_set_sync_gains(int16_t kp_pm_per_rpm,
                                   int16_t ki_pm_per_rpm_step);

/** 清同步环积分与诊断；改变目标转速或重新启动前可调用。 */
void app_motor_demo_reset_sync(void);

/** 读取同步环诊断快照。 */
void app_motor_demo_get_sync_diag(app_motor_demo_sync_diag_t *out);

/* ========================================================================== */
/* 电机校准扫描                                                                 */
/* ========================================================================== */

/**
 * @brief 启动 PWM 校准扫描。
 *
 * 自动关闭同步环，依次从 APP_MOTOR_CAL_PWM_START_PM 到
 * APP_MOTOR_CAL_PWM_END_PM 以 APP_MOTOR_CAL_PWM_STEP_PM 步进遍历，
 * 正向完成后再以负值做反向扫描，结束后恢复原同步配置与目标转速。
 * 默认启用当前运行补偿系统（静摩擦起转、动摩擦运行、右正 scale）；
 * demo 串口 `d` 可切换为原始 PWM 扫描。
 * 每 APP_MOTOR_CAL_SAMPLE_PERIOD_MS 输出一条 [cal] 日志，包含 vbat / rpmL / rpmR。
 *
 * @return true  = 成功启动；false = 已在校准中（busy）。
 */
bool app_motor_demo_cal_start(void);

/**
 * @brief 中止正在进行的校准扫描，立即 brake 并恢复 demo 默认状态。
 *        若当前未在校准中则静默返回。
 */
void app_motor_demo_cal_abort(void);

/** @return true = 校准扫描正在运行中。 */
bool app_motor_demo_cal_is_active(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_MOTOR_DEMO_H */
