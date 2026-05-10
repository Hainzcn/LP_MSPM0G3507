/**
 * @file    app_safety.h
 * @brief   安全状态机 —— 跌倒检测 + 电池保护 + S1 重启
 *
 * ============================================================
 * 这个文件是做什么的？（写给初学者）
 * ============================================================
 * 自平衡小车在运行过程中可能遇到各种危险情况：
 *   1. 车身倾斜太大 → 跌倒了，电机还在空转，可能烧坏驱动器
 *   2. 电池电压过低 → 继续行驶会过放，损坏锂电池
 *   3. 用户想手动停止 → 按下 S1 按键
 *
 * 这个文件实现的"安全状态机"就像一个"监护人"——
 * 它在每个控制周期都被调用一次，检查当前是否安全，
 * 并在危险发生时自动采取保护措施：
 *   - 跌倒 → 立刻刹车 + 关闭电机使能
 *   - 低压告警 → 限幅 PWM（降功率行驶）
 *   - 低压急停 → 立刻刹车 + 关闭电机使能
 *   - S1 按键 → 重启/急停
 *
 * 这是一个"被动监督"模块——它不主动控制（那是 app_balance.c 的工作），
 * 但它在后台持续监督，一旦发生意外就强制介入。
 *
 * ============================================================
 * 状态机设计
 * ============================================================
 *
 *                    S1 启动                    |pitch| > 60°
 *   DISARMED ────────────────► ARMED ────────────────────► FALLEN
 *       ▲                         │  │                         │
 *       │                         │  │ 电池 LOW_WARN           │ S1
 *       │ disarm()                │  ▼                         │
 *       │                         │ LOW_BAT_WARN ────► ARMED  (若电池恢复)
 *       │                         │  │
 *       │                         │  │ 电池 LOW_STOP
 *       │                         ▼  ▼
 *       └──────────── LOW_BAT_STOP ◄── 任何状态，电池 LOW_STOP
 *                              │
 *                              │ S1：被拒绝（电压太低，不能重启）
 *                              ▼
 *                         (停留在 LOW_BAT_STOP，等待电池恢复)
 *
 * 5 个状态的含义：
 *   DISARMED（未就绪）     ：上电默认态，或手动 disarm 后。不运行。
 *   ARMED（就绪/运行中）   ：一切正常，可以行驶。
 *   LOW_BAT_WARN（低压告警）：电池偏低，限速行驶。仍然可以走。
 *   FALLEN（跌倒）         ：车身倾角 > 60°，已经急停。
 *   LOW_BAT_STOP（低压停车）：电池电压太低，已经急停。S1 重启被拒绝。
 *
 * 状态优先级（高优先抢占低优先）：
 *   LOW_BAT_STOP > FALLEN > LOW_BAT_WARN > ARMED
 */
#ifndef APP_SAFETY_H
#define APP_SAFETY_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
 * 编译期可配宏
 * ================================================================
 * 如果小车更"敏感"（容易倒）或更"容忍"（允许更大倾角），
 * 可以在这里调整阈值。也可以在编译命令行通过 -D 覆盖。
 */

/** 跌倒判据：|pitch_deg| > 此值即视为车体倾倒。
 *  默认 60°，与 Overview 文档 §4.1 / Stage 8 约定一致。 */
#ifndef APP_SAFETY_FALL_PITCH_DEG
#define APP_SAFETY_FALL_PITCH_DEG               (60.0f)
#endif

/** 跌倒急停刹车脉冲毫秒数。
 *  检测到跌倒后执行 80 ms 的短刹车脉冲，之后自动转 coast（滑行）。
 *  这个时间足够让电机在自由状态下停下来，又不会让 TB6612 过热。 */
#ifndef APP_SAFETY_FALL_BRAKE_MS
#define APP_SAFETY_FALL_BRAKE_MS                (80u)
#endif

/** 低压急停刹车脉冲毫秒数。
 *  电池电压过低时执行 120 ms 刹车。
 *  比跌倒刹车时间长，因为低电压场景下电流可能更弱。 */
#ifndef APP_SAFETY_LOW_BAT_BRAKE_MS
#define APP_SAFETY_LOW_BAT_BRAKE_MS             (120u)
#endif

/** 低压告警时的 PWM 限幅值（千分比）。
 *  默认 600 = 60% 最大出力。
 *  电池偏低时降功率运行，既保护电池又给用户时间开回来更换/充电。 */
#ifndef APP_SAFETY_LOW_BAT_PWM_LIMIT
#define APP_SAFETY_LOW_BAT_PWM_LIMIT            (600u)
#endif

/* ================================================================
 * 状态枚举
 * ================================================================ */

typedef enum {
    APP_SAFETY_DISARMED     = 0,    /* 上电默认态 / 手动 disarm。
                                       S1 一键启动前停在此态。
                                       此态下电机 STBY=低，不响应任何命令。 */
    APP_SAFETY_ARMED        = 1,    /* 运行态：一切正常，可行驶。
                                       允许 PID 输出到电机。 */
    APP_SAFETY_LOW_BAT_WARN = 2,    /* 低压告警态：电池偏低。
                                       电机已被限速（PWM 上限 600），但仍可行驶。 */
    APP_SAFETY_FALLEN       = 3,    /* 跌倒态：|pitch| > 60°。
                                       电机已急停（brake pulse + STBY 关闭）。 */
    APP_SAFETY_LOW_BAT_STOP = 4     /* 低压急停态：电池电压 ≤ 9.0V。
                                       电机已急停。S1 重启被拒绝。 */
} app_safety_state_t;

/* ================================================================
 * 姿态快照 —— 解耦 ms901m.h，方便单元测试
 * ================================================================
 *
 * 为什么要单独定义这个结构体，而不是直接引用 ms901m_snapshot_t？
 *   1. 解耦：app_safety 不依赖 ms901m.h 的内部结构
 *   2. 可测试：单元测试时可以构造假的姿态快照
 *   3. 精简：只传需要的字段（pitch 和 valid 标志），不传无关数据
 */

typedef struct {
    float pitch_deg;        /* 当前俯仰角（°），车头上扬为正。
                               来自 MS901M 的 0x01 姿态帧。
                               这是跌倒检测的核心输入。 */
    bool  attitude_valid;   /* 姿态数据是否有效。
                               false = 还没收到过 0x01 帧。
                               此时跌倒检测被禁用（短路返回 ARMED），
                               避免"没收到数据就误判为跌倒"。 */
} app_safety_attitude_t;

/* ================================================================
 * API 函数
 * ================================================================ */

/**
 * @brief 初始化安全状态机。
 *
 * 操作：
 *   1. 状态设为 DISARMED（未就绪）
 *   2. 确保电机处于安全态：PWM 限幅恢复 1000、电机停止、STBY 关闭
 *
 * 前提：bsp_motor_init() 和 bsp_battery_init() 已完成。
 *
 * 调用方式（在 main 初始化阶段）：
 * @code
 *   bsp_motor_init();
 *   bsp_battery_init();
 *   app_safety_init();
 * @endcode
 */
void app_safety_init(void);

/**
 * @brief 尝试进入 ARMED 状态（"启动"）。
 *
 * 等价于按一次 S1 按键的效果——如果当前状态允许，切换到 ARMED 并打开 STBY。
 *
 * 被拒绝的情况：如果当前是 LOW_BAT_STOP（电池电压太低），
 * 则函数返回 false，调用方应蜂鸣/报警提示用户。
 *
 * 其他状态（DISARMED / FALLEN / LOW_BAT_WARN）都可以切换到 ARMED。
 * 如果切换到 ARMED 后电池仍然是 LOW_WARN，下一步 tick() 会自动降级。
 *
 * @return true  = 切入 ARMED 成功
 *         false = 被电池保护态拒绝（电压太低）
 */
bool app_safety_arm(void);

/**
 * @brief 主动进入 DISARMED 状态（"手动停车"）。
 *
 * 用于：调试中按下急停按钮、上层故障检测触发、或其他人工介入场景。
 * 操作：brake pulse + 关闭 STBY。
 */
void app_safety_disarm(void);

/**
 * @brief 业务循环周期任务（⭐ 核心函数，建议每拍调用一次）
 *
 * 这个函数是安全状态机的"执行步"——每次调用执行一次完整的检查。
 *
 * 检查流程：
 *   1. 读取 S1 按键 toggle 请求 → 切换状态（ARM ↔ DISARMED）
 *   2. 读取姿态 pitch 角 → 检测是否跌倒
 *   3. 读取电池状态 → 触发低压告警或急停
 *   4. 按优先级合成最终状态，更新 PWM 限幅 / brake / enable
 *
 * 调用方式（在主循环中）：
 * @code
 *   app_safety_attitude_t att;
 *   att.pitch_deg = snap.pitch_deg;
 *   att.attitude_valid = snap.has_attitude;
 *
 *   app_safety_state_t sa = app_safety_tick(&att);
 *
 *   // 根据状态决定是否允许 PID 输出
 *   if (app_safety_can_drive()) {
 *       pid_output_to_motor();
 *   }
 * @endcode
 *
 * @param att  当前姿态快照指针。如果为 NULL，跌倒检测被跳过。
 *             attitude_valid = false 时，跌倒检测也被跳过。
 * @return 当前安全状态枚举
 */
app_safety_state_t app_safety_tick(const app_safety_attitude_t *att);

/** @brief 仅查询当前状态，不做任何检查和动作。 */
app_safety_state_t app_safety_get_state(void);

/**
 * @brief 当前状态是否允许业务代码输出电机命令。
 *
 * 只有两个状态返回 true：
 *   - ARMED（正常行驶）
 *   - LOW_BAT_WARN（低压告警但可继续行驶）
 *
 * DISARMED / FALLEN / LOW_BAT_STOP 返回 false。
 *
 * 业务侧（app_balance.c）应该在 PID 输出前检查这个条件：
 * @code
 *   if (app_safety_can_drive()) {
 *       bsp_motor_set_output(left, right);
 *   }
 * @endcode
 */
bool app_safety_can_drive(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_SAFETY_H */
