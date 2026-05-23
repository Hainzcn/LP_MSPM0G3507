/**
 * @file    app_circle_demo.h
 * @brief   顺时针绕圆演示：沿直径 80 cm 圆顺时针行进一圈后自动停止。
 *
 * ============================================================================
 * 运动学原理
 * ============================================================================
 *  差速驱动绕圆（radius = R，前向速度 v，角速度 ω = v/R）：
 *    v_L = v + ω × L/2  （外侧，顺时针时为左轮）
 *    v_R = v − ω × L/2  （内侧，顺时针时为右轮）
 *
 *  本模块在 app_balance 层之上工作：
 *    ─ target_speed_cps：控制前向速度（速度外环）
 *    ─ target_yaw_pm   ：提供开环差速差量；经修复后的 yaw_angle_step 直接
 *                         写入 yaw_corr_pm → 左右轮差速混合
 *
 * ============================================================================
 * 控制策略
 * ============================================================================
 *  1. 启动后按目标速度（APP_CIRCLE_SPEED_MPS）行驶，同时用角速度 PI 控制器
 *     跟踪期望偏航角速率 ω_target = v/R（以 gz_dps 为反馈）。
 *
 *  2. 用 gz_dps 积分累计实际旋转角度（顺时针方向为正）。
 *
 *  3. 当累计角度 ≥ 360° − APP_CIRCLE_DECEL_MARGIN_DEG 时开始线性减速；
 *     当累计角度 ≥ 360° − APP_CIRCLE_STOP_MARGIN_DEG 时速度归零，
 *     转向同时收敛为零，演示结束。
 *
 * ============================================================================
 * 方向约定与调参
 * ============================================================================
 *  APP_CIRCLE_CW_SIGN：
 *    +1 → target_yaw_pm > 0 时左轮快于右轮 → 小车顺时针旋转（俯视）
 *    若小车实际往逆时针方向绕，将其改为 -1。
 *
 *  APP_CIRCLE_CW_GZ_SIGN：
 *    顺时针旋转时 gz_dps 的符号（+1 或 -1），与 IMU 安装方向有关。
 *    若实际积分方向反了（accumulate 为负值下降），改为 -1。
 *    建议：先用 UART `ci` 启动，观察串口输出的 acc= 字段：
 *      acc 持续增大 → APP_CIRCLE_CW_GZ_SIGN 正确
 *      acc 持续减小 → 改为 -1
 *
 *  APP_CIRCLE_OMEGA_KP / KI：
 *    增大 KP 可加快跟踪，但过大会振荡；
 *    增大 KI 可消除稳态误差（固定 yaw_pm 对应固定转率时等比例）。
 *    建议先调 KP，再微调 KI。
 *
 * ============================================================================
 * 集成说明
 * ============================================================================
 *  1. 在 app_balance.c 的 20 Hz 速度环**之前**调用 app_circle_demo_update()。
 *  2. 活跃时，本模块**覆写** cmd.target_speed_cps 和 cmd.target_yaw_pm，
 *     K230 命令自动被屏蔽。
 *  3. 演示完成后 cmd 字段清零，平衡环自动切回原地驻稳模式。
 */

#ifndef APP_CIRCLE_DEMO_H
#define APP_CIRCLE_DEMO_H

#include <stdbool.h>
#include <stdint.h>
#include "app_balance.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/* §1  编译期配置参数（均可在工程级 -D 覆盖）                                   */
/* ========================================================================== */

/**
 * 绕行圆半径（m）。
 * 题目：圆形黑线内径约 80 cm，小车沿黑线行驶 → 有效半径约 0.40 m。
 */
#ifndef APP_CIRCLE_RADIUS_M
#define APP_CIRCLE_RADIUS_M             (0.40f)
#endif

/**
 * 绕圈前向速度（m/s）。
 * 值越小越容易平衡，推荐先用 0.05~0.08 m/s 调参，再逐步提速。
 */
#ifndef APP_CIRCLE_SPEED_MPS
#define APP_CIRCLE_SPEED_MPS            (0.08f)
#endif

/**
 * 顺时针方向符号。
 * +1 = target_yaw_pm > 0 让左轮快，小车顺时针（俯视）旋转。
 * 若小车反向转，改为 -1。
 */
#ifndef APP_CIRCLE_CW_SIGN
#define APP_CIRCLE_CW_SIGN              (+1)
#endif

/**
 * gz_dps 方向符号：顺时针旋转时 gz_dps 的符号。
 * 与 IMU 安装方向有关；若积累值持续为负请改为 -1。
 */
#ifndef APP_CIRCLE_CW_GZ_SIGN
#define APP_CIRCLE_CW_GZ_SIGN           (+1)
#endif

/**
 * 角速度 PI 控制器比例增益（permille / (°/s)）。
 * 期望角速率 ≈ 0.2 rad/s ≈ 11.5 °/s；
 * KP=8 时初始输出约 92 permille，属合理量程。
 */
#ifndef APP_CIRCLE_OMEGA_KP
#define APP_CIRCLE_OMEGA_KP             (8.0f)
#endif

/**
 * 角速度 PI 控制器积分增益（permille / °）。
 * 抑制稳态误差；置零可退化为纯 P 控制，适合初次调试。
 */
#ifndef APP_CIRCLE_OMEGA_KI
#define APP_CIRCLE_OMEGA_KI             (2.0f)
#endif

/** 积分项钳位（permille）。防止积分饱和。 */
#ifndef APP_CIRCLE_OMEGA_I_LIMIT_PM
#define APP_CIRCLE_OMEGA_I_LIMIT_PM     (60.0f)
#endif

/**
 * 开始线性减速的提前量（°）。
 * 当累积转角 ≥ 360° − DECEL_MARGIN 时，速度按比例从 SPEED_MPS 降至 0。
 */
#ifndef APP_CIRCLE_DECEL_MARGIN_DEG
#define APP_CIRCLE_DECEL_MARGIN_DEG     (30.0f)
#endif

/**
 * 完全停止的提前量（°）。
 * 当累积转角 ≥ 360° − STOP_MARGIN 时速度/转向归零，宣告完成。
 * 应 < DECEL_MARGIN。
 */
#ifndef APP_CIRCLE_STOP_MARGIN_DEG
#define APP_CIRCLE_STOP_MARGIN_DEG      (5.0f)
#endif

/** 期望转过的总角度（°）。一圈 = 360°。 */
#ifndef APP_CIRCLE_TARGET_DEG
#define APP_CIRCLE_TARGET_DEG           (360.0f)
#endif

/** 安全超时（ms）。若超时仍未完成，强制停止。0 = 禁用超时。 */
#ifndef APP_CIRCLE_TIMEOUT_MS
#define APP_CIRCLE_TIMEOUT_MS           (60000u)
#endif

/* ========================================================================== */
/* §2  状态枚举                                                                 */
/* ========================================================================== */

typedef enum {
    CIRCLE_DEMO_IDLE     = 0,   /**< 空闲 / 未启动 */
    CIRCLE_DEMO_PENDING,        /**< 倒计时等待阶段（用户搬运机器人）*/
    CIRCLE_DEMO_RUNNING,        /**< 正在绕圈（全速阶段）*/
    CIRCLE_DEMO_DECELING,       /**< 减速阶段（临近目标角）*/
    CIRCLE_DEMO_DONE            /**< 已完成一圈，等待外部重置 */
} circle_demo_state_t;

/* ========================================================================== */
/* §3  API                                                                     */
/* ========================================================================== */

/**
 * @brief 初始化 / 重置圆圈演示模块（上电后调用一次即可）。
 */
void app_circle_demo_init(void);

/**
 * @brief 延迟启动：停止车轮、开始倒计时，到期后由外部触发正式启动。
 *
 * 调用此函数后状态变为 PENDING；调用方（app_balance_run）应同时：
 *   ① app_safety_disarm()         — 平衡环停止输出
 *   ② bsp_motor_enable(false)     — 物理关闭电机（STBY=0），方便搬运
 *
 * 在 PENDING 期间，update() 不向 cmd 注入任何命令。
 * 当 needs_launch() 返回 true 时，外部应依次调用：
 *   ① bsp_motor_enable(true)
 *   ② app_safety_arm()
 *   ③ auto_zero_pitch_offset()（或等效的自校零）
 *   ④ app_circle_demo_launch()   — 清除 needs_launch 标志，进入 RUNNING
 *
 * @param delay_ms  倒计时毫秒数（典型值 10000）
 * @param now_ms    当前系统时刻（bsp_systick_get_ms()）
 */
void app_circle_demo_start_pending(uint32_t delay_ms, uint32_t now_ms);

/**
 * @brief 立即启动绕圆演示（不经过倒计时）。
 *
 * 演示状态直接变为 RUNNING；下一次 update() 开始注入运动命令。
 * 若演示已在运行，则重置状态并从头开始。
 */
void app_circle_demo_start(void);

/**
 * @brief 立即中止演示，将 cmd 的速度/转向置零，状态回 IDLE。
 *
 * @param cmd  运动指令指针（可为 NULL，此时只重置内部状态）
 */
void app_circle_demo_stop(app_balance_motion_cmd_t *cmd);

/** @return 演示是否正在进行（RUNNING 或 DECELING 均返回 true）。 */
bool app_circle_demo_is_active(void);

/** @return 当前演示状态枚举。 */
circle_demo_state_t app_circle_demo_get_state(void);

/** @return 当前累计旋转角度（°），顺时针为正；IDLE/DONE 时返回最后值。 */
float app_circle_demo_get_accumulated_deg(void);

/**
 * @return PENDING 状态下的剩余等待时间（ms）；非 PENDING 时返回 0。
 */
uint32_t app_circle_demo_get_pending_remaining_ms(uint32_t now_ms);

/**
 * @return true = 倒计时已结束，外部应执行电机使能 + 重校零 + launch()。
 *         读取后标志保持 true，直到 launch() 被调用。
 */
bool app_circle_demo_needs_launch(void);

/**
 * @brief 在电机使能 + 安全重新 ARM + 自校零之后调用，正式进入 RUNNING 状态。
 *        同时清除 needs_launch 标志。
 */
void app_circle_demo_launch(void);

/**
 * @brief 20 Hz 更新函数（在 balance_step_speed 之前调用）。
 *
 * ─ IDLE / DONE：不修改 cmd。
 * ─ RUNNING     ：以 PI 控制角速率，更新 cmd.target_speed_cps 与
 *                  cmd.target_yaw_pm。
 * ─ DECELING    ：同上但线性降低前向速度。
 * ─ 满足退出条件后自动清零 cmd 并置 DONE。
 *
 * @param cmd     运动指令，活跃时被本模块覆写
 * @param gz_dps  本拍陀螺仪偏航角速率（°/s），来自 MS901M snap.gz_dps
 * @param dt_s    本次调用间隔（s），通常 = APP_BALANCE_SPEED_PERIOD_MS/1000
 * @param now_ms  当前系统时刻（ms），用于超时判断
 */
void app_circle_demo_update(app_balance_motion_cmd_t *cmd,
                             float gz_dps, float dt_s, uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* APP_CIRCLE_DEMO_H */
