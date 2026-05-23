/**
 * @file    app_safety.h
 * @brief   安全状态机：跌倒检测 + 电池保护。
 *
 * 这是一个"被动监督"模块：业务循环（如平衡环）每拍调一次 `app_safety_tick()`，
 * 函数内部判定是否触发跌倒 / 低压并主动操作电机 BSP（brake_pulse_ms / enable(false)
 * / set_pwm_limit），然后返回最新状态码给调用方决策。
 *
 *   ─ 跌倒：MS901M `pitch_deg` 绝对值 > APP_SAFETY_FALL_PITCH_DEG (默认 60°)
 *      → 立即 `bsp_motor_brake_pulse_ms(80)` + `bsp_motor_enable(false)`
 *      → 进 FALLEN 态，业务侧 PID 应停止输出
 *   ─ 低压告警：`bsp_battery_get_state() == LOW_WARN`
 *      → `bsp_motor_set_pwm_limit(APP_SAFETY_LOW_BAT_PWM_LIMIT)` (默认 600)
 *      → 进 LOW_BAT_WARN 态；可继续行驶，但限速
 *   ─ 低压急停：`bsp_battery_get_state() == LOW_STOP`
 *      → 立即 `bsp_motor_brake_pulse_ms(120)` + `bsp_motor_enable(false)`
 *      → 进 LOW_BAT_STOP 态，业务侧 PID 应停止
 * 状态优先级（高优先抢占低优先）：
 *   LOW_BAT_STOP  >  FALLEN  >  LOW_BAT_WARN  >  ARMED
 *
 * 与 BSP / IMU 的耦合点：
 *   - `bsp_motor_*`（命令电机）
 *   - `bsp_battery_get_state()`（读电压状态）
 *   - `ms901m_get_snapshot()` 经由调用方传入（避免本模块直接 include ms901m.h；
 *     如此 app_safety 也可以无 IMU 跑单元测试）
 */

#ifndef APP_SAFETY_H
#define APP_SAFETY_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/* 编译期可配宏                                                                 */
/* ========================================================================== */

/** 跌倒判据：|pitch_deg| > 此值即视为车体倾倒（与 Overview §4.1 / Stage 8 对齐） */
#ifndef APP_SAFETY_FALL_PITCH_DEG
#define APP_SAFETY_FALL_PITCH_DEG               (60.0f)
#endif

/** 跌倒急停 brake 脉冲毫秒数（之后转 coast 避免 TB6612 持续注入大电流） */
#ifndef APP_SAFETY_FALL_BRAKE_MS
#define APP_SAFETY_FALL_BRAKE_MS                (80u)
#endif

/** 低压急停 brake 脉冲毫秒数 */
#ifndef APP_SAFETY_LOW_BAT_BRAKE_MS
#define APP_SAFETY_LOW_BAT_BRAKE_MS             (120u)
#endif

/** 低压告警时 PWM 限幅（permille），默认 600 = 60% */
#ifndef APP_SAFETY_LOW_BAT_PWM_LIMIT
#define APP_SAFETY_LOW_BAT_PWM_LIMIT            (600u)
#endif

/**
 * 上电姿态静默窗口：MS901M 刚启动时可能先吐出未稳定姿态，窗口内不做跌倒判定。
 * 平衡层会同步禁止 PID 输出，等待姿态模块稳定后再允许驱动。
 */
#ifndef APP_SAFETY_STARTUP_FALL_MUTE_MS
#define APP_SAFETY_STARTUP_FALL_MUTE_MS         (2500u)
#endif

/**
 * 跌倒确认拍数。app_balance 以 100 Hz 调用 safety，5 拍约 50 ms。
 * 目的不是延迟真正跌倒保护，而是滤掉 MS901M 初始化期 / 串口解析期的单帧毛刺。
 */
#ifndef APP_SAFETY_FALL_DEBOUNCE_TICKS
#define APP_SAFETY_FALL_DEBOUNCE_TICKS          (5u)
#endif

/* ========================================================================== */
/* 状态机                                                                       */
/* ========================================================================== */

typedef enum {
    APP_SAFETY_DISARMED     = 0,    /* 上电默认；业务侧 arm 前主动停在此态 */
    APP_SAFETY_ARMED        = 1,    /* 正常允许电机输出 */
    APP_SAFETY_LOW_BAT_WARN = 2,    /* 电池告警；电机已被限速但仍可走 */
    APP_SAFETY_FALLEN       = 3,    /* 跌倒；电机已急停 */
    APP_SAFETY_LOW_BAT_STOP = 4,    /* 电池保护；电机已急停 */
    /* —— 哨兵值：强制本枚举占用 4 字节、word 对齐，规避 AC6 --short-enums
     *    把枚举压成单字节后被相邻结构体字节越界写入污染的风险。
     *    本值仅作内部存储宽度提示，不应被业务代码使用。 */
    APP_SAFETY_FORCE_INT32_ = 0x7FFFFFFF
} app_safety_state_t;

/** 业务侧每拍传入的"当前姿态"快照（解耦 ms901m.h，方便单元测试） */
typedef struct {
    float pitch_deg;        /* MS901M 0x01 帧；车头上扬为正 */
    bool  attitude_valid;   /* 0x01 帧是否至少收到过；false 时跌倒判据短路返回 ARMED */
} app_safety_attitude_t;

/* ========================================================================== */
/* API                                                                         */
/* ========================================================================== */

/**
 * @brief 初始化：状态置 DISARMED、电机 brake + enable(false)、PWM 上限复位为 1000。
 *        前提：`bsp_motor_init()` 与 `bsp_battery_init()` 已完成。
 */
void app_safety_init(void);

/**
 * @brief 业务侧"准备就绪"指令（如已完成 IMU 校准、对零位）：
 *        若当前不在 LOW_BAT_STOP，把状态切到 ARMED 并 `bsp_motor_enable(true)`。
 *        若当前是 LOW_BAT_STOP，调用被拒绝，返回 false（业务侧应蜂鸣告警）。
 *
 * @return true  = 切到 ARMED 成功
 *         false = 被电池保护态拒绝，仍为 LOW_BAT_STOP
 */
bool app_safety_arm(void);

/**
 * @brief 主动 disarm：状态切到 DISARMED + brake_pulse + enable(false)。
 *        用于人工介入（如调试中按下急停按钮、或上层故障检测）。
 */
void app_safety_disarm(void);

/**
 * @brief 业务循环周期任务（建议 100~1000 Hz 调用）：
 *          ① 读 attitude → 触发跌倒检测
 *          ② 读 bsp_battery_get_state() → 触发低压告警 / 急停
 *          ③ 按状态优先级合成最终输出，更新 PWM 限幅 / brake / enable
 *
 *        本函数会主动调 BSP（不需要业务再去 set_output 时担心权限），
 *        业务层应在进入 ARMED 时才允许 PID 输出（调用方根据返回值判断）。
 *
 * @param att  当前姿态快照；`attitude_valid = false` 时跌倒检测被禁用
 * @return     最新状态枚举
 */
app_safety_state_t app_safety_tick(const app_safety_attitude_t *att);

/** 仅查询当前状态，不做任何处理 */
app_safety_state_t app_safety_get_state(void);

/** 当前状态是否允许业务下发电机命令（仅 ARMED / LOW_BAT_WARN 返回 true） */
bool app_safety_can_drive(void);

/** 上电姿态静默窗口是否仍在生效。true 时业务层应保持电机无输出。 */
bool app_safety_is_startup_grace_active(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_SAFETY_H */
