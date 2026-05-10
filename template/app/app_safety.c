/**
 * @file    app_safety.c
 * @brief   安全状态机实现 —— 跌倒检测 + 电池保护 + S1 重启
 *
 * ============================================================
 * 这个文件是做什么的？（写给初学者）
 * ============================================================
 * 在 app_safety.h 中我们定义了安全状态机的"接口"。
 * 这个 app_safety.c 是"状态机具体怎么判断和切换"的实现。
 *
 * 状态机的运行方式：
 *   每拍调一次 app_safety_tick()，
 *   tick() 内部检查：
 *     1. S1 按键有没有被按下？
 *     2. pitch 角有没有超过 60°？
 *     3. 电池电压有没有低于阈值？
 *   根据检查结果决定当前应该是什么状态，
 *   如果状态变了，就自动执行对应的硬件操作。
 *
 * ============================================================
 * 状态转移图
 * ============================================================
 *
 *                  S1 启动                    |pitch|>60°
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
 *                              │ S1：被拒绝
 *                              ▼
 *                         (stay LOW_BAT_STOP)
 */

/* ============================================================
 * 头文件包含
 * ============================================================ */
#include "app_safety.h"            /* 引入状态枚举、函数声明 */
#include "bsp_battery.h"           /* 引入电池状态查询 */
#include "bsp_motor.h"             /* 引入电机控制（brake/enable/limit） */
#include <stddef.h>                /* NULL */
#include <stdio.h>                 /* printf（调试日志） */

/* ================================================================
 * 模块内部状态
 * ================================================================
 * s_state 是整个安全状态机的"当前状态"。
 * static：只在本文件可见，外部通过 get_state() 查询。
 */

static app_safety_state_t s_state = APP_SAFETY_DISARMED;

/* ================================================================
 * 内部辅助函数 —— 硬件操作封装
 * ================================================================
 * 这些函数把"状态变化时要做什么硬件操作"封装起来。
 * transition() 统一管理状态切换 + 硬件同步。
 */

/**
 * hw_emergency —— 执行紧急停机的硬件操作
 *
 * 紧急停机包括两个操作：
 *   1. brake_pulse_ms(brake_ms)：脉冲式短刹车
 *      让电机在 brake_ms 毫秒内快速停下来
 *   2. enable(false)：关闭 TB6612 的 STBY
 *      之后电机驱动输出 Hi-Z，不再响应任何命令
 *
 * 可重入：即使已经紧急停机了，再调一次也不会出问题。
 * brake_pulse_ms 内部会重新开始计时。
 *
 * @param brake_ms  刹车脉冲的毫秒数（不同的紧急情况用不同的刹车时间）
 */
static void hw_emergency(uint32_t brake_ms)
{
    bsp_motor_brake_pulse_ms(brake_ms);  /* 脉冲式短刹车 */
    bsp_motor_enable(false);              /* 关闭 STBY */
}

/**
 * hw_arm_normal —— 进入"正常运行"的硬件操作
 *
 * 操作：
 *   1. PWM 限幅恢复到最大值 1000（无限制）
 *   2. 使能 STBY（打开电机）
 */
static void hw_arm_normal(void)
{
    bsp_motor_set_pwm_limit(1000u);  /* 恢复满 PWM */
    bsp_motor_enable(true);           /* 使能电机 */
}

/**
 * hw_arm_low_warn —— 进入"低压告警运行"的硬件操作
 *
 * 操作：
 *   1. PWM 限幅设为 600（60% 最大出力，降功率）
 *   2. 使能 STBY（仍然可以走，只是被限速了）
 */
static void hw_arm_low_warn(void)
{
    bsp_motor_set_pwm_limit(APP_SAFETY_LOW_BAT_PWM_LIMIT);  /* 限幅 60% */
    bsp_motor_enable(true);                                   /* 仍可行驶 */
}

/**
 * transition —— 状态转移 + 同步硬件动作
 *
 * 这是整个状态机的"切换中枢"。
 * 每次状态变化时调用它，它会：
 *   1. 检查新状态和旧状态是否相同（相同就不做任何事，避免冗余操作）
 *   2. 更新全局状态 s_state
 *   3. 根据新状态执行对应的硬件操作
 *
 * 为什么需要"相同就不操作"的检查？
 *   如果每拍都执行 hw_emergency()（即使已经紧急停机了），
 *   brake_pulse_ms 的计时器会被反复重置——刹车永远到不了期。
 *   所以"只在状态真正变化时才做硬件操作"。
 *
 * @param next  要切换到的新状态
 */
static void transition(app_safety_state_t next)
{
    /* 如果新状态和当前状态相同，什么都不做 */
    if (next == s_state) {
        return;
    }

    /* 更新状态 */
    s_state = next;

    /* 根据新状态执行硬件操作 */
    switch (next) {

    /* DISARMED 和 FALLEN：都执行紧急刹车
     * DISARMED（手动停车）和 FALLEN（跌倒）都用相同的跌倒刹车脉冲时间 */
    case APP_SAFETY_DISARMED:
    case APP_SAFETY_FALLEN:
        hw_emergency(APP_SAFETY_FALL_BRAKE_MS);
        break;

    /* LOW_BAT_STOP：执行低压停车
     * 用更长的刹车时间（120 ms vs 80 ms），因为电池电压低时电流弱 */
    case APP_SAFETY_LOW_BAT_STOP:
        hw_emergency(APP_SAFETY_LOW_BAT_BRAKE_MS);
        break;

    /* ARMED：正常行驶——恢复满 PWM，使能电机 */
    case APP_SAFETY_ARMED:
        hw_arm_normal();
        break;

    /* LOW_BAT_WARN：低压告警但可行驶——限幅 PWM，使能电机 */
    case APP_SAFETY_LOW_BAT_WARN:
        hw_arm_low_warn();
        break;

    /* 未知状态：不做任何操作（防御性编程） */
    default:
        break;
    }
}

/* ================================================================
 * 公开 API
 * ================================================================ */

/* ----------------------------------------------------------------
 * app_safety_init() —— 初始化
 * ----------------------------------------------------------------
 * 上电时调用一次。确保系统处于安全的初始状态。
 *
 * 操作：
 *   1. 状态 = DISARMED（未就绪）
 *   2. PWM 限幅恢复 1000（默认值）
 *   3. 刹车脉冲 0 ms（即立即停止）
 *   4. STBY = false（电机不使能）
 *
 * 注意：bsp_motor_init() 已经把 STBY 设为低了，
 * 但这里再做一遍是"双保险"——万一 bsp_motor_init 没成功呢？
 */
void app_safety_init(void)
{
    s_state = APP_SAFETY_DISARMED;

    /* 确保电机处于安全态（即使 bsp_motor_init 已经做了，再做一遍保平安） */
    bsp_motor_set_pwm_limit(1000u);    /* 复位限幅 */
    bsp_motor_brake_pulse_ms(0u);      /* 等价于 bsp_motor_stop() */
    bsp_motor_enable(false);            /* 关闭 STBY */
}

/* ----------------------------------------------------------------
 * app_safety_arm() —— 尝试进入 ARMED
 * ----------------------------------------------------------------
 * 功能：把状态切换到 ARMED，并打开电机。
 *
 * 唯一会被拒绝的情况：当前状态是 LOW_BAT_STOP（电池电压太低）。
 * 拒绝时返回 false，调用方应该蜂鸣/报警提示用户。
 *
 * 其他任何状态（DISARMED / FALLEN / LOW_BAT_WARN）都可以切换到 ARMED。
 * 如果切换后电池仍然是 LOW_WARN，下一步 tick() 会自动降级到 LOW_BAT_WARN。
 *
 * @return true = 成功进入 ARMED
 *         false = 被电池保护拒绝
 */
bool app_safety_arm(void)
{
    /* 低电压急停态拒绝启动——电池已经不行了，不能硬开 */
    if (s_state == APP_SAFETY_LOW_BAT_STOP) {
        return false;
    }

    /* 切换到 ARMED。transition() 会自动执行硬件操作。
     * 注意：如果电池此刻还是 LOW_WARN，
     * 下一拍 tick() 会自动把状态降级为 LOW_BAT_WARN。
     * 所以这里"先切到 ARMED"是安全的。 */
    transition(APP_SAFETY_ARMED);
    return true;
}

/* ----------------------------------------------------------------
 * app_safety_disarm() —— 主动进入 DISARMED
 * ----------------------------------------------------------------
 * 用于手动停车。在任何状态下调用都会立即切换到 DISARMED。
 * transition() 会自动执行刹车 + 关闭 STBY。
 */
void app_safety_disarm(void)
{
    transition(APP_SAFETY_DISARMED);
}

/* ----------------------------------------------------------------
 * app_safety_tick() —— 核心周期任务
 * ----------------------------------------------------------------
 * 每次调用执行一次完整的"安全检查 + 状态决策"流程。
 *
 * 步骤：
 *   1. 处理 S1 按键事件 → 切换状态
 *   2. 检查 pitch 角是否 > 60° → 跌倒检测
 *   3. 读取电池电压状态 → 低压告警/急停
 *   4. 按优先级合成最终状态
 *
 * 优先级规则（从高到低）：
 *   LOW_BAT_STOP > FALLEN > LOW_BAT_WARN > ARMED
 *
 * "高优先级抢占"意味着：
 *   即使当前是 ARMED，如果电池突然 LOW_STOP，
 *   状态会直接跳到 LOW_BAT_STOP——不需要经过中间状态。
 *
 * @param att  当前姿态快照指针（可以为 NULL，跌倒检测被跳过）
 * @return 当前状态
 */
app_safety_state_t app_safety_tick(const app_safety_attitude_t *att)
{
    /* ================================================================
     * 步骤 1：处理 S1 按键事件
     * ================================================================
     * bsp_motor_consume_toggle_request() 返回 true 表示 S1 被按下了。
     *
     * S1 按键在不同状态下的行为：
     *   DISARMED → ARMED（启动！）
     *   FALLEN   → ARMED（从跌倒中恢复）
     *   LOW_BAT_WARN → ARMED（如果此时电池已恢复，后续 tick 会升回 ARMED）
     *   ARMED    → DISARMED（手动停车）
     *   LOW_BAT_STOP → 拒绝（电压太低，不能重启）
     *
     * 注意：按键处理在第一步——优先级最高。
     * 即使电池没电了，用户按 S1 是想重启，
     * 但如果电压真的低于 STOP 阈值，第 3 步的电池检查会再把它拉回 STOP。
     */
    if (bsp_motor_consume_toggle_request()) {
        /* 调试日志：S1 被按下了 */
        (void)printf("[btn] S1 pressed: safety state=%d\n", (int)s_state);

        if (s_state == APP_SAFETY_DISARMED ||
            s_state == APP_SAFETY_FALLEN ||
            s_state == APP_SAFETY_LOW_BAT_WARN) {
            /* 可以切换的状态：尝试进入 ARMED */
            bool ok = app_safety_arm();
            (void)printf("[safety] S1 arm request %s, state=%d\n",
                ok ? "accepted" : "rejected", (int)s_state);

        } else if (s_state == APP_SAFETY_ARMED) {
            /* 已在 ARMED 状态，再按 S1 → 主动停车（disarm） */
            transition(APP_SAFETY_DISARMED);
            (void)printf("[safety] S1 disarm request accepted, state=%d\n",
                (int)s_state);

        } else if (s_state == APP_SAFETY_LOW_BAT_STOP) {
            /* 电池电压太低，拒绝重启 */
            (void)app_safety_arm();  /* 返回值 false，但在 LOW_BAT_STOP 下预期如此 */
            (void)printf("[safety] S1 arm request rejected by LOW_BAT_STOP\n");
        }
    }

    /* ================================================================
     * 步骤 2：跌倒检测
     * ================================================================
     * 检查 |pitch| 是否超过 APP_SAFETY_FALL_PITCH_DEG（默认 60°）。
     *
     * 注意：
     *   - 如果 attitude 指针为 NULL，跳过跌倒检测（不判断）
     *   - 如果 attitude_valid = false（还没收到过 IMU 数据），也跳过
     *     避免"IMU 还没工作就误判为跌倒"
     *   - pitch 取绝对值——无论前倾还是后仰，超过 60° 都是危险的
     *
     * fallen 标志先暂存，到步骤 4 再参与状态决策。 */
    bool fallen = false;
    if ((att != NULL) && att->attitude_valid) {
        float p = att->pitch_deg;
        if (p < 0.0f) p = -p;     /* 取绝对值 */
        if (p > APP_SAFETY_FALL_PITCH_DEG) {
            fallen = true;
        }
    }

    /* ================================================================
     * 步骤 3：读取电池状态
     * ================================================================
     * 从 bsp_battery_get_state() 获取当前电池等级。
     * 这个值在 bsp_battery_update() 中被更新（建议 100 Hz 调用）。
     *
     * bs 的可能性：UNKNOWN / NORMAL / LOW_WARN / LOW_STOP */
    bsp_battery_state_t bs = bsp_battery_get_state();

    /* ================================================================
     * 步骤 4：按优先级合成状态
     * ================================================================
     * 这里的 if-else if 链按照优先级排列：
     *   LOW_BAT_STOP > FALLEN > LOW_BAT_WARN > ARMED
     *
     * 特殊考虑：
     *   - DISARMED 是"人工状态"，不会被电池/跌倒事件自动升回 ARMED
     *     （只有 S1 按键可以把 DISARMED → ARMED）
     *   - 但电池 LOW_STOP 仍然可以覆盖 DISARMED
     *     （即使还没启动，也要提示用户电池没电了）
     *   - 跌倒事件在 DISARMED 下被忽略
     *     （车在地上本身就会倒，不需要告警）
     *   - UNKNOWN 状态下（采样还没攒够首拍），不做任何状态变更。
     */

    /* 优先级 1（最高）：电池 LOW_STOP → 立刻停车，不管之前什么状态 */
    if (bs == BSP_BATT_STATE_LOW_STOP) {
        transition(APP_SAFETY_LOW_BAT_STOP);

    } else if (s_state == APP_SAFETY_LOW_BAT_STOP) {
        /* 上一拍是 LOW_STOP，但现在电池已经回升到 STOP_MV + HYS 以上。
         *
         * 不自动恢复 ARMED！只降级到 LOW_BAT_WARN。
         * 需要用户手动按 S1 才能重新启动。
         *
         * 这样做的原因：
         *   防止"电池电压在阈值附近抖动 → 车体反复急停/启动"的危险情况。
         *   一旦进入了 LOW_BAT_STOP，必须人工确认才能重启。
         *
         * 额外做一次 enable(false) 是"二次确认"——确保 STBY 仍然是低。 */
        transition(APP_SAFETY_LOW_BAT_WARN);
        bsp_motor_enable(false);

    } else if (s_state == APP_SAFETY_DISARMED) {
        /* DISARMED 状态下：
         *   - 跌倒事件被忽略（车倒了是正常的）
         *   - 但电池告警仍然要提示（改为 LOW_BAT_WARN 状态）
         *     注意：不用 transition()——因为 transition 会调 hw_arm_low_warn，
         *     它会 enable(true) 把电机打开。但 DISARMED 状态下不应该开电机。
         *     所以只更新状态变量，不做硬件操作。 */
        if (bs == BSP_BATT_STATE_LOW_WARN) {
            s_state = APP_SAFETY_LOW_BAT_WARN;
        }

    } else if (fallen) {
        /* 跌倒检测触发 →
         *   进入 FALLEN 状态，transition() 自动执行紧急刹车 */
        transition(APP_SAFETY_FALLEN);

    } else if (bs == BSP_BATT_STATE_LOW_WARN) {
        /* 电池低电压告警 →
         *   如果当前是 ARMED 或 LOW_BAT_WARN，保持/进入 LOW_BAT_WARN
         *   如果当前是 FALLEN，不做变更（等 S1 重启） */
        if (s_state == APP_SAFETY_ARMED || s_state == APP_SAFETY_LOW_BAT_WARN) {
            transition(APP_SAFETY_LOW_BAT_WARN);
        }

    } else if (bs == BSP_BATT_STATE_NORMAL) {
        /* 电池正常 + 没有跌倒 →
         *   如果之前是 LOW_BAT_WARN（低压告警），现在电池恢复了，
         *   自动升回 ARMED。
         *   其他状态（ARMED / FALLEN / DISARMED）不做变更。 */
        if (s_state == APP_SAFETY_LOW_BAT_WARN) {
            transition(APP_SAFETY_ARMED);
        }
    }

    /* bs == UNKNOWN（采样还没攒够首拍）→ 状态不变，跳过本拍 */

    return s_state;
}

/* ----------------------------------------------------------------
 * 状态查询函数
 * ---------------------------------------------------------------- */

app_safety_state_t app_safety_get_state(void)
{
    return s_state;
}

bool app_safety_can_drive(void)
{
    /* 只有 ARMED 和 LOW_BAT_WARN 两个状态允许输出电机命令。
     * DISARMED / FALLEN / LOW_BAT_STOP 都不允许。
     *
     * 业务侧（app_balance.c）应该在每次 PID 输出前检查这个条件。 */
    return (s_state == APP_SAFETY_ARMED) ||
           (s_state == APP_SAFETY_LOW_BAT_WARN);
}
