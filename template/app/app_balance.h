/**
 * @file    app_balance.h
 * @brief   平衡车双环控制骨架 —— 速度外环 + 平衡内环 + 转向叠加
 *
 * ============================================================
 * 这个文件是做什么的？（写给初学者）
 * ============================================================
 * 这是整个项目最核心的控制模块——你想要小车"站起来并且走"，
 * 就是通过这个文件来实现的。
 *
 * 它实现了"级联 PID 控制"（Cascade PID），包含两个环路：
 *
 *   外环（速度环）：控制"目标速度"→ 输出"目标倾角"
 *     你想让车以多快的速度往前走？
 *     如果实际速度比目标慢 → 让车往前倾更多 → 加速
 *     如果实际速度比目标快 → 让车倾角减小 → 减速
 *
 *   内环（平衡环）：控制"目标倾角"→ 输出"电机 PWM"
 *     你想让车保持多少度的倾角？
 *     如果实际倾角比目标大 → 给电机更多力 → 摆正
 *     如果实际倾角比目标小 → 给电机少一点力
 *
 *   转向叠加：左右电机加一个差值 → 车体旋转
 *
 * 理解"级联"（Cascade）：
 *   就像开车——你先决定"我要开多快"（外环），
 *   然后根据速度差距决定"油门踩多深"（内环）。
 *   两个环节环环相扣，一个的输出是另一个的输入。
 *
 * ============================================================
 * ⚠️ 为什么所有 PID 增益默认是 0？
 * ============================================================
 * 这是"失效安全"（fail-safe）设计：
 *   上电后如果不设增益，电机输出永远为 0——不会乱跑。
 *   等你在调试中通过串口注入合适的增益后，车才会动。
 *
 * 每个小车的物理参数（重量、重心、电机功率）都不同，
 * 不可能有"万能增益值"。调试是你的必经之路。
 */

#ifndef APP_BALANCE_H
#define APP_BALANCE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
 * 编译期可配宏
 * ================================================================ */

/** 控制环周期（毫秒）。默认 10 ms = 100 Hz。
 *  即每 10 毫秒执行一次速度环 + 平衡环的 PID 计算。
 *  100 Hz 对平衡控制来说足够了——太快会过度消耗 CPU，
 *  太慢控制延迟大会导致车不稳。 */
#ifndef APP_BALANCE_CONTROL_PERIOD_MS
#define APP_BALANCE_CONTROL_PERIOD_MS           (10u)
#endif

/**
 * 速度外环输出"目标俯仰角"的绝对值上限（°）。
 * 默认 10°：外环不能把目标倾角推到超过 ±10°。
 *
 * 为什么需要这个限制？
 *   速度外环的 PID 输出是"目标倾角"——它让车体倾斜来加速/减速。
 *   如果外环输出 90°，车体就翻了。10° 是一个合理的上限——
 *   在这个倾角内，平衡内环还能正常工作。 */
#ifndef APP_BALANCE_MAX_TILT_DEG
#define APP_BALANCE_MAX_TILT_DEG                (10.0f)
#endif

/** 平衡内环输出 PWM 的绝对值上限（千分比）。
 *  默认 1000 = 100% 占空比。 */
#ifndef APP_BALANCE_MAX_PWM_PERMILLE
#define APP_BALANCE_MAX_PWM_PERMILLE            (1000)
#endif

/**
 * 速度外环 D 项 EMA 滤波系数。
 * 默认 0.20：速度信号的噪声较大（20 ms 窗口差分产生的抖动），
 * 需要用更强的滤波来压制。
 * 0.20 比平衡环的 0.10 大，因为速度信号本身噪声更大。 */
#ifndef APP_BALANCE_SPEED_D_FILTER_ALPHA
#define APP_BALANCE_SPEED_D_FILTER_ALPHA        (0.20f)
#endif

/**
 * 平衡内环 D 项 EMA 滤波系数。
 * 默认 0.10：MS901M 角速度的噪声较小（传感器内部已做滤波），
 * 用较弱滤波即可。 */
#ifndef APP_BALANCE_BALANCE_D_FILTER_ALPHA
#define APP_BALANCE_BALANCE_D_FILTER_ALPHA      (0.10f)
#endif

/* ================================================================
 * 输入结构体 —— 调用方每拍传入的数据
 * ================================================================
 * 这两个结构体是"外部数据"到"平衡控制"的接口。
 * 调用方在主循环中填充它们，然后传给 app_balance_step()。
 */

/**
 * 当前姿态快照（从 MS901M 获取后填入）。
 *
 * 为什么单独定义而不是直接用 ms901m_snapshot_t？
 *   1. 解耦：app_balance 不依赖 ms901m.h 的全部字段
 *   2. 精简：只传需要的字段（pitch_deg + pitch_rate_dps + 有效性标志）
 *   3. 可测试：可以构造假的姿态数据做单元测试
 */
typedef struct {
    float pitch_deg;        /* 当前俯仰角（°）。车头上扬为正。
                               来自 MS901M 0x01 姿态帧，经过单位换算。
                               这是平衡控制的"测量值"。 */
    float pitch_rate_dps;   /* 当前俯仰角速度（°/s）。
                               来自 MS901M gy_dps 字段。
                               注意：在 ms901m.c 中这个值叫 gy_dps，
                               但在这里我们直接用 PID 的"d on measurement"模式，
                               所以这个值不是给顶层的微分项用的——
                               PID 内部自己用 prev_meas 算微分。 */
    bool  attitude_valid;   /* 姿态数据是否有效。
                               false = 还没收到过 0x01 帧 → 不执行控制。 */
} app_balance_attitude_t;

/**
 * 运动指令（目标速度和方向）。
 *
 * 在阶段 1~2，这个值从 K230 运动指令帧解析得到（或本地默认值）。
 * 外环的目标速度 = 两个轮子的平均速度（前进方向）
 * 转向量 = 左右轮的速度差（旋转方向）
 */
typedef struct {
    int32_t target_speed_cps;   /* 期望前进速度（counts/s）。
                                   正值 = 前进，负值 = 后退。
                                   这个值等于左右轮速度的平均值：(v_left + v_right) / 2。
                                   单位是 cps（counts per second），
                                   可以通过编码器参数换算为 °/s 或 rpm。 */
    int16_t target_yaw_pm;      /* 期望转向量（千分比）。
                                   正值 = 顺时针（俯视），负值 = 逆时针。
                                   这个值直接叠加到左右电机命令上：
                                   left_cmd -= yaw, right_cmd += yaw。
                                   注意：这是"开环"转向——没有 yaw 角度反馈，
                                   因为我们没有 yaw 编码器。 */
} app_balance_motion_cmd_t;

/* ================================================================
 * 诊断信息快照 —— 用于调试日志打印
 * ================================================================
 * 这个结构体不是"控制逻辑需要的"，而是"调试人员需要的"。
 * 通过读取这个结构体，你可以在串口上打印出当前的控制状态。
 */

typedef struct {
    float   target_tilt_deg;    /* 速度外环输出 → 传入平衡内环的目标倾角（°） */
    float   pitch_meas_deg;     /* 实际俯仰角（已减去静态零点偏移） */
    float   balance_out_pwm;    /* 平衡内环输出 PWM（permille） */
    int16_t left_cmd_pm;        /* 最终发给左轮的命令（permille） */
    int16_t right_cmd_pm;       /* 最终发给右轮的命令（permille） */
    int32_t speed_meas_cps;     /* 实际测量速度 = (左+右)/2（counts/s） */
    bool    driving;            /* 本拍是否真的在驱动电机（受 safety 限制） */
} app_balance_diag_t;

/* ================================================================
 * API 函数
 * ================================================================ */

/**
 * @brief 初始化平衡控制器。
 *
 * 初始化两个 PID 控制器（速度外环 + 平衡内环）为安全默认值：
 *   - 所有增益 = 0（电机不会动！必须手动设置增益）
 *   - 速度外环输出限幅 ±10°
 *   - 平衡内环输出限幅 ±1000
 *   - D 项滤波系数预置
 *   - 静态零偏 = 0°、转向系数 = 1.0
 */
void app_balance_init(void);

/**
 * @brief 复位两个 PID 的内部状态（积分和微分历史清零）。
 *
 * 在以下情况调用：
 *   - 切换控制目标（如从前进改为后退）
 *   - 跌倒后重新启动
 *   - 换了一组 PID 增益
 *   - safety 禁止驱动时（自动调用）
 */
void app_balance_reset(void);

/**
 * @brief 设置静态俯仰零点偏移（°）。
 *
 * 小车在"自然直立"时，pitch 角可能不是 0°——
 * 因为 IMU 安装角度、车身结构偏差等原因。
 * 这个函数让你补偿这个偏差：
 *   调整后的 pitch = 原始 pitch - 你设定的 offset
 *
 * 怎么确定这个值？
 *   用支架/积木让车体保持"你认为的直立"，
 *   用串口读 1 秒内的平均 pitch_deg，把这个值设进去。
 */
void app_balance_set_pitch_offset(float deg);

/**
 * @brief 设置平衡内环的三个增益（Kp/Ki/Kd）。
 *
 * 平衡内环：
 *   输入 = 目标倾角 - 实际倾角（°）
 *   输出 = PWM 命令（permille）
 *
 * 调试顺序（先内环后外环）：
 *   Kp 从 0 慢慢加到车体能短暂直立但前后晃
 *   Kd 从 0 慢慢加到晃动被抑制
 *   Ki 暂留 0（内环通常不需要积分）
 */
void app_balance_set_balance_gains(float kp, float ki, float kd);

/**
 * @brief 设置速度外环的三个增益（Kp/Ki/Kd）。
 *
 * 速度外环：
 *   输入 = 目标速度 - 实际速度（counts/s）
 *   输出 = 目标倾角（°）
 *
 * 内环能稳定直立后再调外环。
 * Kp 典型在 0.001~0.01 之间（因为输入 cps 数量级是千到万，输出度是几度）。
 * Ki 用来消除稳态速度误差。
 */
void app_balance_set_speed_gains(float kp, float ki, float kd);

/**
 * @brief 设置转向开环系数。
 *
 * 转向量的计算：left_cmd -= yaw_kp × target_yaw, right_cmd += yaw_kp × target_yaw。
 * 默认 1.0 = K230 给的 target_yaw_pm 直接作为差速量。
 * 调试范围 0.3~1.5。
 */
void app_balance_set_yaw_kp(float kp_yaw);

/**
 * @brief ⭐ 执行一拍完整的双环控制计算（建议 100 Hz 调用）。
 *
 * 这是整个平衡控制模块最核心的函数！
 * 每次调用执行一次完整的"决策 + 执行"流程。
 *
 * 内部流程：
 *   1. 调用 app_safety_tick() 检查安全状态
 *   2. 如果不允许驱动（跌倒/低压/未就绪）→ 复位 PID + 不输出电机命令
 *   3. 如果允许驱动：
 *      a. 读取当前速度反馈（左右轮平均 cps）
 *      b. 速度外环 PID：目标速度 vs 实际速度 → 输出目标倾角
 *      c. 平衡内环 PID：目标倾角 vs 实际倾角 → 输出 PWM
 *      d. 转向叠加：left -= yaw, right += yaw
 *      e. 限幅后调用 bsp_motor_set_output(left, right)
 *   4. 记录诊断信息到内部 diag 结构体
 *
 * @param att  当前姿态快照（从 MS901M 获取）
 * @param cmd  当前运动指令（从 K230 或本地默认获取）
 *
 * @note  这个函数完全幂等——同样的输入 + 同样的内部状态 → 同样的输出。
 *        可以在单元测试中调用，不依赖任何硬件。
 */
void app_balance_step(const app_balance_attitude_t *att,
                      const app_balance_motion_cmd_t *cmd);

/**
 * @brief 获取本拍内部诊断信息快照。
 *
 * 不影响内部状态。主要用于调试日志（1 Hz 打印一次）。
 * 调用方传入一个 app_balance_diag_t 变量的地址，函数把数据填入。
 */
void app_balance_get_diag(app_balance_diag_t *out);

/**
 * @brief Stage 2.2 上车基线主循环入口（永不返回，由 main 调用）。
 *
 * 这个函数是"最终跑在单片机上的主循环"——它包含了所有定时任务。
 * 调用后不会返回（无限循环）。
 *
 * 定时任务调度（基于 1 ms SysTick）：
 *   ┌─────────┬──────────────────────────────────────────┐
 *   │ 频率     │ 任务                                     │
 *   ├─────────┼──────────────────────────────────────────┤
 *   │ 1 kHz   │ IMU 数据读取 + ms901m 解析               │
 *   │         │ bsp_motor_update（QEI 扩展 + 速度窗口）   │
 *   ├─────────┼──────────────────────────────────────────┤
 *   │ 100 Hz  │ bsp_battery_update（电池采样）            │
 *   │         │ ms901m_get_snapshot + app_balance_step    │
 *   ├─────────┼──────────────────────────────────────────┤
 *   │ 5 Hz    │ LED_G 心跳翻转 + 安全状态 LED 指示        │
 *   ├─────────┼──────────────────────────────────────────┤
 *   │ 1 Hz    │ printf 调试日志（pitch/速度/safety/电池）  │
 *   └─────────┴──────────────────────────────────────────┘
 *
 * 调用前提（所有初始化必须完成）：
 *   - SYSCFG_DL_init / bsp_gpio_init / bsp_systick_init
 *   - bsp_log_uart_init / bsp_k230_uart_init / bsp_imu_uart_init
 *   - ms901m_init / bsp_motor_init / bsp_battery_init
 *   - app_safety_init / app_balance_init
 *   - main 已通过 wait_for_ms901m_attitude 验证 IMU 在线
 */
void app_balance_run(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_BALANCE_H */
