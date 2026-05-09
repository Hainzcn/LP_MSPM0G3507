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
 * @return true = 收到装车模式启动请求，调用方应切入 app_balance_run()。
 *
 * S1 用作急刹 / 启动切换；左右轮保持同向输出，方便通过编码器转速判断同步性。
 *
 * 当前工程没有可直接复用的板载 S2 输入：LaunchPad SW2/J15 默认落在 PA16，
 * 但 PA16 已被 TB6612 AIN2 占用。确认新的 S2 引脚后，在 app_motor_demo.c
 * 打开 APP_MOTOR_DEMO_ENABLE_LOAD_BUTTON 并填入对应 BSP_LOAD_BTN_* 宏即可返回。
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

#ifdef __cplusplus
}
#endif

#endif /* APP_MOTOR_DEMO_H */
