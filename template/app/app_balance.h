/**
 * @file    app_balance.h
 * @brief   平衡车双环控制骨架（速度外环 + 平衡内环 + 转向叠加）
 *
 * ============================================================================
 * ⚠️ 整定提示（必读）
 * ============================================================================
 * 本模块**默认所有 PID 增益 = 0**，即上电后输出恒为 0、电机不会自己动。
 * 这样设计的理由：
 *   1) 平衡车 PID 增益强依赖物理参数（轮径、质量、重心、电机转速常数 KV、
 *      电池电压等），任何"硬编码经验值"在本工程实物上几乎都会发散；
 *   2) 调试期需要随时通过串口 / K230 命令注入新增益做"在线整定"，
 *      固定值会让试参流程被反复刷写 Flash 拖慢；
 *   3) 上电默认 0 是失效安全的（fail-safe），即使误启动也不会乱跑。
 *
 * 推荐整定流程（先内环后外环，经典级联 PID 整定）：
 *
 *   ① **测量先验**：
 *      - 轮径 D / 减速比 GR / 编码器 PPR（已在 bsp_motor.h 中）；
 *      - 静态俯仰零点 pitch_offset_deg：让车体自由立直（用积木 / 支架辅助），
 *        读 1 s 平均的 pitch_deg，写入 `app_balance_set_pitch_offset()`；
 *      - 若传感器前后装反，打开 `APP_BALANCE_PITCH_INVERT` 或运行时调用
 *        `app_balance_set_pitch_inverted(true)`，无需改 MS901M 解析层。
 *      - 估算车体高度 h、质心高度 hc、整车质量 m，套倒立摆模型粗估
 *        Kp_balance ≈ m·g·hc / (转矩常数·hc²)，做整定起点。
 *
 *   ② **平衡内环（先调）**：
 *      a) 速度外环增益全 0（已是默认），目标是只跑内环；
 *      b) `app_balance_set_balance_gains(Kp, 0, Kd)`：
 *         - 把 Kp 从小开始（如 30），慢慢加大直到车体能短暂直立但前后晃；
 *         - 加 Kd（典型 Kp 的 1/4 ~ 1/8）抑制晃动；
 *         - Ki 暂留 0；
 *      c) 通过串口 / K230 实时注入并复位（`app_balance_reset()`）查看效果；
 *      d) 出现"低头快速冲刺 / 抬头反向冲刺" → Kp 太大或 Kd 太小；
 *         "低头慢漂" → Kp 太小或方向接反；
 *         "正常一段后小幅振荡" → Kd 太大或采样噪声大。
 *
 *   ③ **速度外环（后调）**：
 *      a) 内环已能直立 ≥ 5 s 后开此环；
 *      b) `app_balance_set_speed_gains(Kp, Ki, 0)`：
 *         - 速度外环输出是"目标俯仰角偏移"（°），让车体主动前 / 后倾来加 / 减速；
 *         - Kp 从小开始（典型 0.001~0.01，因为输入是 cps、输出是 deg）；
 *         - 加少量 Ki 消除稳态速度误差；
 *      c) 速度外环输出会被钳到 ±`APP_BALANCE_MAX_TILT_DEG`（默认 ±10°），
 *         避免外环把目标角推到内环工作区外；
 *
 *   ④ **转向叠加（最后调）**：
 *      `app_balance_set_yaw_kp(Kp_yaw)`：直接的 yaw 速率反馈；K230 给的 ω 命令
 *      乘以一个开环系数即可，本骨架未做内环 yaw（GB370 没装 yaw 编码器）。
 *
 * ============================================================================
 * 调用模型
 * ============================================================================
 *   bsp_motor_init() / bsp_battery_init() / app_safety_init();
 *   app_balance_init();
 *   app_balance_set_balance_gains(Kp, Ki, Kd);
 *   app_balance_set_speed_gains  (Kp, Ki, Kd);
 *   app_balance_set_yaw_kp       (Kp_yaw);
 *
 *   for (;;) {
 *       if (bsp_systick_consume_tick()) {
 *           bsp_motor_update();          // 1 kHz 必跑
 *           bsp_battery_update();        // 100 Hz 调用频率不必每 ms 都触发
 *           // 100 Hz 控制环
 *           if (++ctrl_div >= 10) {
 *               ctrl_div = 0;
 *               ms901m_get_snapshot(&snap);
 *               app_balance_attitude_t att = { .pitch_deg = snap.pitch_deg,
 *                                              .pitch_rate_dps = snap.gy_dps,
 *                                              .attitude_valid = snap.has_attitude };
 *               app_balance_motion_cmd_t cmd = { .target_speed_cps = ...,
 *                                                .target_yaw_pm = ... };
 *               app_balance_step(&att, &cmd);
 *           }
 *       }
 *   }
 *
 *   `app_balance_step()` 内部会：
 *     - 调 `app_safety_tick()` 检查是否允许驱动；
 *     - 不允许 → 停止输出（不调 set_output，避免覆盖 safety 的 brake 命令）；
 *     - 允许 → 跑速度外环 → 平衡内环 → 直行同步补偿 → set_output(left, right)。
 */

#ifndef APP_BALANCE_H
#define APP_BALANCE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/* 编译期可配宏                                                                 */
/* ========================================================================== */

/** 控制环周期（毫秒），与调用方调用 `app_balance_step()` 的周期对齐 */
#ifndef APP_BALANCE_CONTROL_PERIOD_MS
#define APP_BALANCE_CONTROL_PERIOD_MS           (10u)   /* 100 Hz */
#endif

/** 速度外环输出"目标俯仰角"绝对值上限（°），防止外环推到内环工作区外 */
#ifndef APP_BALANCE_MAX_TILT_DEG
#define APP_BALANCE_MAX_TILT_DEG                (10.0f)
#endif

/** 平衡内环输出 PWM 绝对值上限（permille） */
#ifndef APP_BALANCE_MAX_PWM_PERMILLE
#define APP_BALANCE_MAX_PWM_PERMILLE            (1000)
#endif

/** 速度外环 D 项 EMA 滤波系数（0=禁用），速度环 D 项典型噪声大需要滤 */
#ifndef APP_BALANCE_SPEED_D_FILTER_ALPHA
#define APP_BALANCE_SPEED_D_FILTER_ALPHA        (0.20f)
#endif

/** 平衡内环 D 项 EMA 滤波系数（0=禁用） */
#ifndef APP_BALANCE_BALANCE_D_FILTER_ALPHA
#define APP_BALANCE_BALANCE_D_FILTER_ALPHA      (0.10f)
#endif

/**
 * 俯仰角一阶低通滤波系数（0~1）。
 * MS901M 已内置 EKF，但主控侧仍加一层轻量 LPF 抑制串口帧抖动和单帧毛刺。
 * 控制环 100 Hz 下，0.35 约等效 18 ms 时间常数，延迟较小，适合作为平衡初值。
 */
#ifndef APP_BALANCE_PITCH_LPF_ALPHA
#define APP_BALANCE_PITCH_LPF_ALPHA             (0.35f)
#endif

/** 俯仰角软件极性翻转：当前装车 MS901M 前后方向与车体坐标相反，默认启用。 */
#ifndef APP_BALANCE_PITCH_INVERT
#define APP_BALANCE_PITCH_INVERT                (1)
#endif

/** 装车模式双轮同步服务开关：直行时按左右编码器速度差做差分 PWM 补偿。 */
#ifndef APP_BALANCE_SYNC_ENABLED
#define APP_BALANCE_SYNC_ENABLED                (1)
#endif

/** 同步环比例项，单位是 0.01 permille / cps；20 = 0.20 pm/cps ≈ 4.4 pm/rpm。 */
#ifndef APP_BALANCE_SYNC_KP_PM_PER_CPS_X100
#define APP_BALANCE_SYNC_KP_PM_PER_CPS_X100     (20)
#endif

/** 同步环积分项，单位同上；默认 0，先避免与平衡内环互相积分。 */
#ifndef APP_BALANCE_SYNC_KI_PM_PER_CPS_STEP_X100
#define APP_BALANCE_SYNC_KI_PM_PER_CPS_STEP_X100 (0)
#endif

/** 同步差分补偿限幅，避免编码器异常时大幅扰动平衡内环。 */
#ifndef APP_BALANCE_SYNC_MAX_CORRECTION_PM
#define APP_BALANCE_SYNC_MAX_CORRECTION_PM      (200)
#endif

/**
 * 同步环启用的最小平衡驱动力（permille）。
 * PID 输出很小时暂停同步，避免 PID=0 手拨轮触发电机；平衡环真正发力时恢复同步。
 */
#ifndef APP_BALANCE_SYNC_MIN_DRIVE_PM
#define APP_BALANCE_SYNC_MIN_DRIVE_PM          (30)
#endif

/* ========================================================================== */
/* 输入结构体                                                                   */
/* ========================================================================== */

/** 当前姿态（业务侧从 ms901m_get_snapshot 取后填入） */
typedef struct {
    float pitch_deg;        /* 俯仰角；车头上扬为正（与 MS901M 对齐） */
    float pitch_rate_dps;   /* 俯仰角速度，°/s（MS901M gy_dps） */
    bool  attitude_valid;   /* 0x01 帧是否至少收到过 */
} app_balance_attitude_t;

/** 运动指令（业务侧从 K230 命令 / 本地状态机解析后填入） */
typedef struct {
    int32_t target_speed_cps;   /* 期望前进速度（counts/s 平均 = (L+R)/2）；正 = 前进 */
    int16_t target_yaw_pm;      /* 期望转向开环量（permille）；正 = 顺时针俯视 */
} app_balance_motion_cmd_t;

/* ========================================================================== */
/* 调试 / 遥测快照                                                              */
/* ========================================================================== */

/** 调用方读取本拍内部状态用，便于 1 Hz / 10 Hz 串口打印调试 */
typedef struct {
    float   target_tilt_deg;    /* 速度外环输出 = 平衡内环目标角 */
    float   pitch_meas_deg;     /* 实际俯仰（已减零点） */
    float   balance_out_pwm;    /* 平衡内环输出，permille */
    int16_t sync_correction_pm;  /* 双轮同步差分补偿；正值 = 左加右减 */
    int32_t sync_error_cps;      /* right_speed_cps - left_speed_cps */
    int16_t left_cmd_pm;        /* 最终左轮命令 */
    int16_t right_cmd_pm;       /* 最终右轮命令 */
    int32_t speed_meas_cps;     /* 实际平均速度 = (L+R)/2 */
    bool    driving;            /* 本拍是否真在驱动（受 safety 限制） */
} app_balance_diag_t;

/* ========================================================================== */
/* API                                                                         */
/* ========================================================================== */

/** 初始化两路 PID 为安全默认（增益 0 + 输出限幅 + D 滤波系数已写入）。 */
void app_balance_init(void);

/** 复位：清两路 PID 内部状态（i_term + 微分历史）。换控制目标前调一次。 */
void app_balance_reset(void);

/**
 * @brief 设置静态俯仰零点（°）。让车体在地面"标准直立"状态下读 1 s 平均 pitch
 *        填入此处，运行时 `(raw_pitch - offset) * pitch_sign` 会真正反映
 *        "偏离平衡点的角度"。
 */
void app_balance_set_pitch_offset(float deg);

/**
 * @brief 设置俯仰角软件极性翻转。
 *
 * MS901M 前后方向装反时，前倾会被解析成后倾，平衡环会反向输出。
 * 开启后仅在 app_balance 层把 `(raw_pitch - offset)` 取反，不改传感器解析层。
 */
void app_balance_set_pitch_inverted(bool inverted);

/** @return true = 当前已启用俯仰角软件翻转。 */
bool app_balance_get_pitch_inverted(void);

/** 设置平衡内环增益（输入：tilt 误差 deg；输出：PWM permille）。 */
void app_balance_set_balance_gains(float kp, float ki, float kd);

/** 设置速度外环增益（输入：速度误差 cps；输出：目标 tilt deg）。 */
void app_balance_set_speed_gains(float kp, float ki, float kd);

/**
 * @brief 设置转向开环系数：left -= yaw, right += yaw 的乘子。
 *        默认 1.0 = K230 给的 target_yaw_pm 直接当差速量加；可在 0.3 ~ 1.5 间调试。
 */
void app_balance_set_yaw_kp(float kp_yaw);

/**
 * @brief 跑一拍控制环（建议 100 Hz 调用，与 `APP_BALANCE_CONTROL_PERIOD_MS` 对齐）。
 *
 *        内部流程：
 *          ① app_safety_tick(att)：拿到当前安全状态；
 *          ② 不允许驱动 → app_balance_reset() + 不再调 bsp_motor_set_output
 *             （避免覆盖 safety 已经下发的 brake 命令），return；
 *          ③ 允许驱动 → 速度外环 → 限幅得到目标 tilt → 平衡内环 →
 *             转向叠加 + 直行同步补偿 → bsp_motor_set_output(left, right)。
 *
 *        本函数完全幂等：同样的输入 + 同样的内部状态 → 同样的输出。
 */
void app_balance_step(const app_balance_attitude_t *att,
                      const app_balance_motion_cmd_t *cmd);

/** 拷贝一份本拍内部诊断（不影响内部状态）。 */
void app_balance_get_diag(app_balance_diag_t *out);

/**
 * @brief Stage 2.2 上车基线主循环入口。
 *
 *        调度策略（单任务轮询 + SysTick 1 ms 节拍标志）：
 *          1 kHz : drain UART3 → ms901m_feed_bytes + bsp_motor_update（QEI 软扩 / 速度窗）
 *          100 Hz: bsp_battery_update + ms901m_get_snapshot + app_balance_step
 *           5 Hz : LED_G 心跳翻转（同时按 safety 状态点 LED_R / LED_B 提示）
 *           1 Hz : XDS-UART 调试日志（pitch / 速度 / safety / 电池 / PID 输出）
 *
 *        调用前必须保证：
 *          - SYSCFG_DL_init / bsp_gpio_init / bsp_systick_init / bsp_log_uart_init
 *          - bsp_k230_uart_init / bsp_imu_uart_init / ms901m_init (4, 2000)
 *          - bsp_motor_init / bsp_battery_init
 *          - app_safety_init / app_balance_init 全部完成
 *          - main 已通过 wait_for_ms901m_attitude 验证 0x01 帧在线
 *
 *        ⚠️ PID 增益默认 0：上电后即使站立姿态正确电机也不会动。装车整定时
 *           通过串口 / K230 命令注入 `app_balance_set_balance_gains` /
 *           `app_balance_set_speed_gains`；上层调用 `app_safety_arm()` 后才允许驱动。
 *
 * @return true = 收到 UART `t` / `test`，调用方应切入电机演示模式。
 */
bool app_balance_run(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_BALANCE_H */
