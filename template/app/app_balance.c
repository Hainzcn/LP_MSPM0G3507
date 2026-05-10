/**
 * @file    app_balance.c
 * @brief   平衡车双环控制骨架实现 —— 级联 PID + 主循环调度
 *
 * ============================================================
 * 这个文件是做什么的？（写给初学者）
 * ============================================================
 * app_balance.h 定义了"平衡控制有哪些功能和接口"。
 * 这个 app_balance.c 是"具体怎么实现"。
 *
 * 这个文件分为两大部分：
 *
 *   上半部分（app_balance_step 及配套）：
 *     实现"级联 PID"的完整数据流：
 *       速度外环 PID → 目标倾角 → 平衡内环 PID → PWM → 电机
 *     加上 safety 检查、转向叠加、诊断记录。
 *
 *   下半部分（app_balance_run）：
 *     实现"最终跑在单片机上的主循环"。
 *     基于 1 ms SysTick 节拍，调度所有任务：
 *       1 kHz / 100 Hz / 5 Hz / 1 Hz
 *
 * ============================================================
 * 级联 PID 数据流（一次 app_balance_step 调用）
 * ============================================================
 *
 *   cmd->target_speed_cps (目标速度, counts/s)
 *         │
 *         ▼
 *   ┌──────────────────┐
 *   │  速度外环 PID     │  ← 输入：目标速度 - 实际速度(avg_cps)
 *   │  (speed_pid)     │  ← 输出：目标倾角 target_tilt_deg
 *   └────────┬─────────┘
 *            │ target_tilt_deg
 *            ▼
 *   ┌──────────────────┐
 *   │  平衡内环 PID     │  ← 输入：目标倾角 - 实际倾角(pitch_meas)
 *   │  (balance_pid)   │  ← 输出：PWM permille
 *   └────────┬─────────┘
 *            │ pwm_out
 *            ▼
 *   ┌──────────────────┐
 *   │  转向叠加 + 限幅  │  left = pwm - yaw, right = pwm + yaw
 *   └────────┬─────────┘
 *            │ left_pm, right_pm
 *            ▼
 *   ┌──────────────────┐
 *   │  bsp_motor_set_   │  → TB6612 → 电机转动
 *   │  output(l, r)     │
 *   └──────────────────┘
 *
 * 所有的增益初始化为 0——调试时必须手动设置。
 */

/* ============================================================
 * 头文件包含
 * ============================================================ */
#include "app_balance.h"
#include "app_safety.h"         /* 安全状态机 */
#include "bsp_battery.h"        /* 电池电压（1 Hz 日志打印） */
#include "bsp_gpio.h"           /* LED 控制（心跳灯） */
#include "bsp_imu_uart.h"       /* MS901M 串口数据接收 */
#include "bsp_k230_uart.h"      /* K230 通讯统计 */
#include "bsp_motor.h"          /* 电机速度命令 + 编码器反馈 */
#include "bsp_systick.h"        /* consume_tick + get_ms */
#include "ms901m.h"             /* MS901M 姿态解析 */
#include "pid.h"                /* PID 控制器 */
#include "ti_msp_dl_config.h"   /* DL_GPIO_xxx 函数 */
#include <stdio.h>              /* printf（调试日志） */
#include <stddef.h>             /* NULL */

/* ================================================================
 * 模块内部状态
 * ================================================================
 * 所有的控制状态集中在一个结构体中，方便管理。
 *
 * s_bal（全局静态实例）包含：
 *   - speed_pid：速度外环（输入 cps 误差，输出 target_tilt_deg）
 *   - balance_pid：平衡内环（输入 tilt 误差，输出 PWM）
 *   - pitch_offset_deg：静态零点偏移
 *   - yaw_kp：转向开环系数
 *   - diag：本拍诊断快照
 */

typedef struct {
    pid_t   speed_pid;          /* 外环 PID：目标速度 vs 实际速度（cps → °） */
    pid_t   balance_pid;        /* 内环 PID：目标倾角 vs 实际倾角（° → PWM permille） */
    float   pitch_offset_deg;   /* 静态俯仰零点偏移（补偿 IMU 安装偏差） */
    float   yaw_kp;             /* 转向开环缩放系数 */
    app_balance_diag_t diag;    /* 本拍诊断快照 */
} balance_state_t;

/** 全局唯一的平衡控制状态实例 */
static balance_state_t s_bal;

/**
 * s_dt_sec：控制周期（秒）
 *
 * APP_BALANCE_CONTROL_PERIOD_MS = 10 ms = 0.010 秒。
 * 这个值被传入 pid_step() 的 dt_sec 参数。
 * 用宏转换为 float 常量，避免每次调用 pid_step 时重复计算。 */
static const float s_dt_sec =
    (float)APP_BALANCE_CONTROL_PERIOD_MS / 1000.0f;

/* ================================================================
 * 内部辅助函数
 * ================================================================ */

/**
 * clamp_pwm_pm —— 把浮点 PWM 值限幅到 ±MAX_PWM_PERMILLE 并转为 int16。
 *
 * 限幅后转为 int16_t——电机的 set_output 接受 int16_t 类型。
 * 为什么要限幅？即使 PID 本身有限幅，这一步作为"最后一道保险"
 * 确保送到电机的值绝不会超过范围。
 */
static int16_t clamp_pwm_pm(float v)
{
    /* 上限限幅 */
    if (v >  (float)APP_BALANCE_MAX_PWM_PERMILLE) v =  (float)APP_BALANCE_MAX_PWM_PERMILLE;
    /* 下限限幅 */
    if (v < -(float)APP_BALANCE_MAX_PWM_PERMILLE) v = -(float)APP_BALANCE_MAX_PWM_PERMILLE;
    /* 转为 int16 返回 */
    return (int16_t)v;
}

/* ================================================================
 * 公开 API —— 初始化与配置
 * ================================================================ */

/* ----------------------------------------------------------------
 * app_balance_init() —— 初始化（所有增益 = 0，安全！）
 * ----------------------------------------------------------------
 * 初始化两个 PID 控制器为安全默认值。
 *
 * 速度外环：输出限幅 ±10°（防止推到无法恢复的倾角）
 * 平衡内环：输出限幅 ±1000（防止 PWM 超出范围）
 *
 * 增益全为 0——调用方必须手动 set_gains 才有输出。 */
void app_balance_init(void)
{
    /* 初始化外环 PID */
    pid_init(&s_bal.speed_pid);
    /* 初始化内环 PID */
    pid_init(&s_bal.balance_pid);

    /* 速度外环输出限幅：±10°
     * 速度环输出的是"目标倾角"，不能超过 ±10° */
    pid_set_output_limit(&s_bal.speed_pid,
        -(float)APP_BALANCE_MAX_TILT_DEG, (float)APP_BALANCE_MAX_TILT_DEG);
    /* 速度外环 D 项滤波（速度信号噪声大，需要较强滤波） */
    pid_set_d_filter(&s_bal.speed_pid, APP_BALANCE_SPEED_D_FILTER_ALPHA);

    /* 平衡内环输出限幅：±1000（千分比） */
    pid_set_output_limit(&s_bal.balance_pid,
        -(float)APP_BALANCE_MAX_PWM_PERMILLE, (float)APP_BALANCE_MAX_PWM_PERMILLE);
    /* 平衡内环 D 项滤波（角速度噪声小，弱滤波即可） */
    pid_set_d_filter(&s_bal.balance_pid, APP_BALANCE_BALANCE_D_FILTER_ALPHA);

    /* 零点偏移 = 0°（调用方可以后续通过 set_pitch_offset 修改） */
    s_bal.pitch_offset_deg = 0.0f;
    /* 转向系数 = 1.0（默认不缩放） */
    s_bal.yaw_kp = 1.0f;

    /* 清空诊断快照 */
    s_bal.diag.target_tilt_deg = 0.0f;
    s_bal.diag.pitch_meas_deg  = 0.0f;
    s_bal.diag.balance_out_pwm = 0.0f;
    s_bal.diag.left_cmd_pm     = 0;
    s_bal.diag.right_cmd_pm    = 0;
    s_bal.diag.speed_meas_cps  = 0;
    s_bal.diag.driving         = false;
}

/** 复位：两个 PID 内部状态（积分 + 微分历史）清零。 */
void app_balance_reset(void)
{
    pid_reset(&s_bal.speed_pid);
    pid_reset(&s_bal.balance_pid);
}

/** 设置静态俯仰零点偏移 */
void app_balance_set_pitch_offset(float deg)
{
    s_bal.pitch_offset_deg = deg;
}

/** 设置平衡内环增益 */
void app_balance_set_balance_gains(float kp, float ki, float kd)
{
    pid_set_gains(&s_bal.balance_pid, kp, ki, kd);
}

/** 设置速度外环增益 */
void app_balance_set_speed_gains(float kp, float ki, float kd)
{
    pid_set_gains(&s_bal.speed_pid, kp, ki, kd);
}

/** 设置转向开环系数 */
void app_balance_set_yaw_kp(float kp_yaw)
{
    s_bal.yaw_kp = kp_yaw;
}

/* ================================================================
 * 公开 API —— app_balance_step() ⭐ 核心控制函数
 * ================================================================
 * 这是整个平衡控制的大脑！
 * 每 10 ms（100 Hz）调用一次，执行以下流程：
 *
 *   步骤 1：安全检查（safety tick）
 *   步骤 2：速度外环 PID
 *   步骤 3：平衡内环 PID
 *   步骤 4：转向叠加 + 限幅
 *   步骤 5：输出到电机 + 诊断记录
 *
 * 如果 safety 不允许驱动（跌倒/低压/未就绪）：
 *   → 复位 PID（清零历史和积分避免残留）
 *   → 不调 bsp_motor_set_output（让 safety 的刹车指令生效）
 *   → 记录诊断信息（全零）
 *
 * @param att  当前姿态
 * @param cmd  运动指令
 */
void app_balance_step(const app_balance_attitude_t *att,
                      const app_balance_motion_cmd_t *cmd)
{
    /* 防御性编程：att 和 cmd 都不能为 NULL */
    if ((att == NULL) || (cmd == NULL)) {
        return;
    }

    /* ================================================================
     * 步骤 1：安全检查
     * ================================================================
     * 构造 app_safety_attitude_t 传给 app_safety_tick()。
     * tick() 内部会检查：
     *   - S1 按键事件
     *   - pitch 角是否 > 60°（跌倒）
     *   - 电池电压是否正常
     * 并自动执行必要的硬件操作（刹车/限幅/关闭 STBY）。
     *
     * 注意：这里传入的 pitch_deg 已经减去了 pitch_offset_deg。
     * 这样 safety 的跌倒检测也是基于"调整后的倾角"。 */
    app_safety_attitude_t sa = {
        .pitch_deg = att->pitch_deg - s_bal.pitch_offset_deg,
        .attitude_valid = att->attitude_valid,
    };
    (void)app_safety_tick(&sa);

    /* ---- 如果 safety 不允许驱动，提前返回 ----
     * 两种情况：
     *   !can_drive()：safety 状态是 DISARMED / FALLEN / LOW_BAT_STOP
     *   !attitude_valid：还没收到过 IMU 数据（跑平衡但不控制）
     *
     * 此时：
     *   1. reset PID——清零积分历史和微分历史，避免下次 ARMED 时跨段污染
     *   2. 记录诊断信息（全零），表示"本拍没在驱动"
     *   3. 不调 bsp_motor_set_output——让 safety 的刹车指令生效 */
    if (!app_safety_can_drive() || !att->attitude_valid) {
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

    /* ================================================================
     * 步骤 2：速度外环（100 Hz）
     * ================================================================
     * 外环 PID 输入：目标速度 vs 实际速度
     * 外环 PID 输出：目标倾角 target_tilt_deg（°）
     *
     * 实际速度 = 左轮速度 + 右轮速度的平均值（cps）
     * 正 = 前进，负 = 后退。
     *
     * PID 的 dt = s_dt_sec = 0.010 秒（10 ms 控制周期） */
    bsp_motor_feedback_t fb;
    bsp_motor_get_feedback(&fb);
    int32_t avg_cps = (fb.left_speed_cps + fb.right_speed_cps) / 2;

    float target_tilt_deg = pid_step(&s_bal.speed_pid,
        (float)cmd->target_speed_cps,   /* 目标：主控/K230 给的速度命令 */
        (float)avg_cps,                  /* 测量：当前实际平均速度 */
        s_dt_sec);

    /* ================================================================
     * 步骤 3：平衡内环（100 Hz）
     * ================================================================
     * 内环 PID 输入：目标倾角 vs 实际倾角
     * 内环 PID 输出：PWM permille
     *
     * 实际倾角 = 传感器 pitch_deg - 静态零点偏移
     * 目标倾角 = 速度外环的输出 target_tilt_deg
     *
     * 内环 PID 的 D 项使用的是"d on measurement"模式（参见 pid.c），
     * 即微分作用在测量值（pitch_meas）上而不是误差上。
     * 这样目标倾角突变时不会产生微分冲击。
     *
     * 极性说明：
     *   如果发现"加 Kp 后车自己倒了"——可能是 pitch 极性反了。
     *   这时把 Kp 取反，或在外面对 pitch_deg 取反即可。 */
    float pitch_meas = att->pitch_deg - s_bal.pitch_offset_deg;
    float pwm_out = pid_step(&s_bal.balance_pid,
        target_tilt_deg,    /* 目标：速度外环输出的目标倾角 */
        pitch_meas,          /* 测量：当前实际倾角（已减偏移） */
        s_dt_sec);

    /* ================================================================
     * 步骤 4：转向叠加 + 限幅
     * ================================================================
     * 转向 = 目标 yaw × yaw_kp 系数
     * 左轮 = 平衡输出 - 转向（左转时左轮减速）
     * 右轮 = 平衡输出 + 转向（左转时右轮加速）
     *
     * 限幅：确保左右轮命令都在 ±MAX_PWM 范围内。 */
    float yaw_pm = (float)cmd->target_yaw_pm * s_bal.yaw_kp;
    int16_t left_pm  = clamp_pwm_pm(pwm_out - yaw_pm);
    int16_t right_pm = clamp_pwm_pm(pwm_out + yaw_pm);

    /* ================================================================
     * 步骤 5：输出到电机 + 诊断记录
     * ================================================================
     * bsp_motor_set_output() 内部会做：
     *   限幅 → 极性翻转（如果有 invert）→ 方向位 + PWM 占空比
     * 
     * 诊断记录：保存本拍的关键数值，供 1 Hz 日志打印。 */
    bsp_motor_set_output(left_pm, right_pm);

    s_bal.diag.target_tilt_deg = target_tilt_deg;
    s_bal.diag.pitch_meas_deg  = pitch_meas;
    s_bal.diag.balance_out_pwm = pwm_out;
    s_bal.diag.left_cmd_pm     = left_pm;
    s_bal.diag.right_cmd_pm    = right_pm;
    s_bal.diag.speed_meas_cps  = avg_cps;
    s_bal.diag.driving         = true;
}

/* ================================================================
 * 公开 API —— 诊断信息获取
 * ================================================================ */

void app_balance_get_diag(app_balance_diag_t *out)
{
    if (out == NULL) return;
    *out = s_bal.diag;
}

/* ================================================================
 * Stage 2.2 上车基线主循环 —— app_balance_run()
 * ================================================================
 * 这个函数是"最终跑在单片机上的主循环"。
 * 它调用后永不返回（无限循环）。
 *
 * ================================================================
 * 调度策略
 * ================================================================
 * 基于 1 ms SysTick 节拍，用 tick_count 做周期调度：
 *
 *   tick_count 取值：0, 1, 2, 3, 4, 5, ... 999, 1000, ...
 *
 *   1 kHz（每 tick）：
 *     - IMU UART 数据 drain（pop_bulk → ms901m_feed_bytes）
 *     - bsp_motor_update()（QEI 16→32 扩展 + 速度窗口 + brake 倒计时）
 *
 *   100 Hz（tick % 10 == 0）：
 *     - bsp_battery_update()（ADC 采样 + EMA 滤波）
 *     - ms901m_get_snapshot() + app_balance_step()（级联 PID）
 *
 *   5 Hz（tick % 200 == 0）：
 *     - LED_G 绿灯翻转（心跳指示）
 *     - LED_R 按 safety 状态变化（跌倒/低压亮红，其他灭红）
 *
 *   1 Hz（tick % 1000 == 0）：
 *     - printf 调试日志（包含 pitch/速度/safety/电池/PID/编码器等信息）
 */

/* 单拍从 UART3 环形缓冲区取出的字节数上限。
 * MS901M 默认 5 帧 × 200 Hz × ~15 B = 15 kB/s = 15 B/ms。
 * 64 字节的单拍容量给 4 倍裕量，即使偶尔积压也能一次清空。 */
#define APP_BAL_IMU_DRAIN_CHUNK     64u

/* 调度相位（多少个 1 ms tick 执行一次） */
#define APP_BAL_PHASE_CTRL_TICKS    APP_BALANCE_CONTROL_PERIOD_MS  /* = 10 → 100 Hz */
#define APP_BAL_PHASE_LED_TICKS     200u                            /* = 200 → 5 Hz */
#define APP_BAL_PHASE_LOG_TICKS     1000u                           /* = 1000 → 1 Hz */

/* ================================================================
 * 浮点字段格式化辅助宏
 * ================================================================
 *
 * 为什么需要这些宏？因为不用 printf("%f")！
 *
 * AC6（ARM Compiler 6）的 printf 浮点路径会拉入非常大的库代码
 * （几 KB 的浮点格式化代码），而且 Cortex-M0+ 没有硬件 FPU，
 * 浮点格式化非常慢。
 *
 * 所以我们的做法是：
 *   把浮点数乘 100 变成定点数，用整数 printf 打印。
 *
 * 例如 pitch = 2.35°：
 *   BAL_F2_X100(2.35) = 235（乘以 100，四舍五入）
 *   BAL_F2_S(2.35)     = ' '（正数用空格，负数用 '-'）
 *   BAL_F2_I(2.35)     = 2（整数部分）
 *   BAL_F2_F(2.35)     = 35（小数部分，2 位）
 *   最终输出：" 2.35"
 *
 * 对于负数 pitch = -2.35°：
 *   BAL_F2_X100(-2.35) = -235
 *   BAL_F2_S(-2.35)    = '-'（负数用减号）
 *   BAL_F2_I(-2.35)    = 2（取绝对值后的整数）
 *   BAL_F2_F(-2.35)    = 35（取绝对值后的小数）
 *   最终输出："-2.35"
 *
 * 这样避免了 printf("%f") 带来的巨大代码膨胀和性能损失。 */

/** 浮点数 ×100 并四舍五入为 int32 */
#define BAL_F2_X100(v)  ((int32_t)((v) * 100.0f + ((v) >= 0.0f ? 0.5f : -0.5f)))

/** 符号字符：正数返回空格 ' '，负数返回 '-' */
#define BAL_F2_S(v)     (BAL_F2_X100(v) < 0 ? '-' : ' ')

/** 整数部分（取绝对值后 / 100 取整） */
#define BAL_F2_I(v)     ((int32_t)((BAL_F2_X100(v) < 0 ? -BAL_F2_X100(v) : BAL_F2_X100(v)) / 100))

/** 小数部分（取绝对值后 % 100） */
#define BAL_F2_F(v)     ((uint32_t)((BAL_F2_X100(v) < 0 ? -BAL_F2_X100(v) : BAL_F2_X100(v)) % 100))

/* ================================================================
 * 安全状态 → 可打印字符串的映射
 * ================================================================ */

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

/* ================================================================
 * app_balance_run() —— 主循环入口（永不返回）
 * ================================================================ */

void app_balance_run(void)
{
    /* ---- 主循环变量 ----
     * tick_count：SysTick 计数器（每 1 ms + 1）
     * last_total_rx：上次 1 Hz 时的 K230 接收字节数（用于计算差值）
     * last_enc_irq：上次 1 Hz 时的编码器 ISR 计数（用于诊断雪崩） */
    uint32_t tick_count    = 0u;
    uint32_t last_total_rx = bsp_k230_uart_total_rx();
    uint32_t last_enc_irq  = bsp_motor_get_enc_irq_count();
    ms901m_snapshot_t snap = { 0 };

    /* 默认运动指令：原地不动。
     * 后续阶段 K230 通讯接入后，由 MOTION_CMD 帧覆盖。 */
    app_balance_motion_cmd_t cmd = { .target_speed_cps = 0, .target_yaw_pm = 0 };

    /* ██ 主循环 ██ 永不退出！ */
    for (;;) {

        /* ---- 等待 1 ms SysTick 心跳 ---- */
        /* 如果当前 tick 还没到，用 __WFI() 睡眠等待下一个中断唤醒 */
        if (!bsp_systick_consume_tick()) {
            __WFI();
            continue;
        }
        tick_count++;

        /* ============================================================
         * 1 kHz 任务：IMU 数据 drain + 电机 1 ms 节拍
         * ============================================================
         * IMU UART RX 半满中断已经把字节排入 256 字节环形缓冲区。
         * 这里只需要从环缓冲中批量取出来，喂给 ms901m 解析器。
         *
         * bsp_motor_update() 必须每毫秒调用一次！
         *   它负责 QEI 16→32 扩展、速度窗口累计、brake_pulse 倒计时、
         *   ISR 雪崩检测——如果漏调了，这些功能会错乱。 */
        uint8_t buf[APP_BAL_IMU_DRAIN_CHUNK];
        size_t  got = bsp_imu_uart_rx_pop_bulk(buf, sizeof(buf));
        if (got > 0u) {
            ms901m_feed_bytes(buf, got);
        }
        bsp_motor_update();

        /* ============================================================
         * 100 Hz 任务：电池采样 + 控制环
         * ============================================================
         * tick_count % 10 == 0 → 每 10 ms 执行一次。
         *
         * 1. bsp_battery_update()：ADC 采样 + EMA 滤波 + 阈值状态机
         * 2. ms901m_get_snapshot()：获取最新姿态数据
         * 3. app_balance_step()：执行一拍的完整双环控制
         *
         * 注意：电池采样虽然建议 100 Hz，但实际 update 内部会做
         * 自己的时序管理。这里每次 100 Hz 都调一次没问题。 */
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

        /* ============================================================
         * 5 Hz 任务：LED 状态指示
         * ============================================================
         * tick_count % 200 == 0 → 每 200 ms 执行一次。
         *
         * LED_G（绿灯）：
         *   持续翻转（toggle）——表示"系统在运行，心跳正常"
         *   如果绿灯不闪了，说明系统卡住了。
         *
         * LED_R（红灯）：
         *   - 正常 / ARMED → 灭
         *   - LOW_BAT_WARN → 闪烁（toggle）——提醒电池偏低
         *   - FALLEN / LOW_BAT_STOP → 常亮——紧急情况
         *
         * 这样调试时看 LED 就能大致判断状态，不需要一直看串口。 */
        if ((tick_count % APP_BAL_PHASE_LED_TICKS) == 0u) {
            /* 绿灯心跳：每 200 ms 翻转一次 */
            DL_GPIO_togglePins(BSP_LED_G_PORT, BSP_LED_G_PIN);

            app_safety_state_t st = app_safety_get_state();
            if (st == APP_SAFETY_FALLEN ||
                st == APP_SAFETY_LOW_BAT_STOP) {
                /* 紧急状态：红灯常亮 */
                DL_GPIO_setPins(BSP_LED_R_PORT, BSP_LED_R_PIN);
            } else if (st == APP_SAFETY_LOW_BAT_WARN) {
                /* 低压告警：红灯闪烁（随绿灯一起 5 Hz toggle） */
                DL_GPIO_togglePins(BSP_LED_R_PORT, BSP_LED_R_PIN);
            } else {
                /* 正常：红灯灭 */
                DL_GPIO_clearPins(BSP_LED_R_PORT, BSP_LED_R_PIN);
            }
        }

        /* ============================================================
         * 1 Hz 任务：XDS-UART 调试日志
         * ============================================================
         * tick_count % 1000 == 0 → 每 1000 ms 执行一次。
         *
         * 打印内容（一条长长的日志行）：
         *   [hb] t=12s state=ARMED pitch= 1.23 tilt*= 0.00 pwm= 12.34
         *        L=123 R=456 v=789cps batt=11000mV ms901m_g=200/b=0
         *        k230_rx=0b/s encL=12345 encR=-678 encISR=1320/s
         *        btn=1/0
         *
         * 字段解读：
         *   t          = 启动至今的秒数
         *   state      = 当前安全状态
         *   pitch      = 俯仰角（已减偏移）
         *   tilt*      = 速度外环输出的目标倾角
         *   pwm        = 平衡内环输出的 PWM
         *   L/R        = 最终左右轮命令
         *   v          = 实际速度（cps）
         *   batt       = 电池电压（mV）
         *   ms901m_g/b = MS901M 的好帧/坏帧计数
         *   k230_rx    = 过去 1 秒从 K230 收到的字节数
         *   encL/encR  = 左右编码器累计计数
         *   encISR     = 过去 1 秒编码器 ISR 触发次数
         *   btn        = 按键中断次数 / 轮询兜底次数
         *   ISR_QUENCH = 如果编码器 ISR 雪崩抑制正在激活，加这个标记！
         */
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

            /* 用定点格式化宏避免 printf("%f") 的浮点路径
             * 每个浮点数拆成：符号 + 整数部分 + "." + 小数部分 */
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
