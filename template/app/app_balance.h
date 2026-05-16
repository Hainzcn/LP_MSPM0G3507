/**
 * @file    app_balance.h
 * @brief   平衡车四级级联控制（速度环 20Hz + 角度环 100Hz + 角速度环 200Hz + 航向环 50Hz）
 *
 * ============================================================================
 * ⚠️ 整定提示（必读）
 * ============================================================================
 * 本模块**默认所有 PID 增益 = 0**，即上电后输出恒为 0、电机不会自己动。
 *
 * 推荐整定流程（从最内环到最外环）：
 *
 *   ① **测量先验**：
 *      - 静态俯仰零点 pitch_offset_deg → `app_balance_set_pitch_offset()`；
 *      - 若传感器前后装反 → `app_balance_set_pitch_inverted(true)`。
 *
 *   ② **角速度内环（200 Hz，最先调）**：
 *      `app_balance_set_rate_gains(Kp, 0, 0)`：
 *         - Kp 直接乘角速率误差（°/s），输出 PWM permille；
 *         - 串口 `rp 1000 0 0`（Kp=1.0）实时注入。
 *
 *   ③ **角度环（100 Hz）**：
 *      `app_balance_set_balance_gains(Kp, Ki, 0)`：
 *         - 输出"目标角速率"（°/s）给角速度内环；
 *         - 串口 `bp 5000 0 0`（Kp=5.0）实时注入。
 *
 *   ④ **速度外环（20 Hz，内环稳定后再调）**：
 *      `app_balance_set_speed_gains(Kp, Ki, 0)`：
 *         - 输出"目标俯仰角偏移"（°），钳到 ±APP_BALANCE_MAX_TILT_DEG；
 *         - 串口 `sp 5 0 0`（Kp=0.005，速度已归一化 /10）实时注入；
 *           归一化后 0.1 rev/s ≈ 510，sp 5 产生 ~2.5° 倾角，安全可用。
 *
 *   ⑤ **航向环（50 Hz，最后调）**：
 *      `app_balance_set_yaw_gains(Kp, Ki, Kd)`：
 *         - 直行闭环锁定航向，输出差分 PWM；串口 `yp 500 0 100`。
 *
 * ============================================================================
 * 调用模型
 * ============================================================================
 *   bsp_motor_init() / bsp_battery_init() / app_safety_init();
 *   app_balance_init();
 *   app_balance_set_rate_gains    (Kp_rate, 0, 0);
 *   app_balance_set_balance_gains (Kp_angle, Ki_angle, 0);
 *   app_balance_set_speed_gains   (Kp_spd, Ki_spd, 0);
 *   app_balance_set_yaw_kp        (Kp_yaw);
 *   app_balance_run();   // 内部多速率调度：200/100/50/20 Hz
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

/** 角速度内环周期（毫秒），与 IMU 200 Hz 回报率对齐。电机动作同频。 */
#ifndef APP_BALANCE_RATE_PERIOD_MS
#define APP_BALANCE_RATE_PERIOD_MS              (5u)    /* 200 Hz */
#endif

/** 角度环周期（毫秒）。 */
#ifndef APP_BALANCE_ANGLE_PERIOD_MS
#define APP_BALANCE_ANGLE_PERIOD_MS             (10u)   /* 100 Hz */
#endif

/** 航向环周期（毫秒）。 */
#ifndef APP_BALANCE_YAW_PERIOD_MS
#define APP_BALANCE_YAW_PERIOD_MS               (20u)   /*  50 Hz */
#endif

/** 速度外环周期（毫秒）。 */
#ifndef APP_BALANCE_SPEED_PERIOD_MS
#define APP_BALANCE_SPEED_PERIOD_MS             (50u)   /*  20 Hz */
#endif

/** 速度外环输出"目标俯仰角"绝对值上限（°），防止外环推到内环工作区外 */
#ifndef APP_BALANCE_MAX_TILT_DEG
#define APP_BALANCE_MAX_TILT_DEG                (10.0f)
#endif

/** 平衡内环输出 PWM 绝对值上限（permille） */
#ifndef APP_BALANCE_MAX_PWM_PERMILLE
#define APP_BALANCE_MAX_PWM_PERMILLE            (1000)
#endif

/**
 * 输出零区阈值（permille）：|最终轮命令| < 此值时强制 Coast（0 电压）。
 *
 * 作用：消除死区映射在零点的 ±DZ 方波翻转振动。
 * 代价：引入 ±(threshold / balance_Kp)° 的角度不敏感带。
 *   例：threshold=8, Kp=25 → ±0.32° 不敏感带，对平衡车可接受。
 * 设 0 禁用。
 */
#ifndef APP_BALANCE_ZERO_BAND_PM
#define APP_BALANCE_ZERO_BAND_PM                (0)
#endif

/** 速度外环 D 项 EMA 滤波系数（0=禁用），速度环 D 项典型噪声大需要滤 */
#ifndef APP_BALANCE_SPEED_D_FILTER_ALPHA
#define APP_BALANCE_SPEED_D_FILTER_ALPHA        (0.20f)
#endif

/**
 * 速度反馈低通滤波系数（0~1）。
 *
 * 编码器 20ms 差分窗口在低速时量化噪声严重（最小分辨率 50 cps），
 * 直通到速度外环会被放大后注入平衡内环，造成 PWM 抖动加剧。
 * 此 EMA 滤波器平滑速度测量，使速度外环带宽远低于平衡内环：
 *   20 Hz × α=0.15 → 时间常数 ~280ms → 带宽 ~0.6 Hz（内环 ~12 Hz 的 1/20）。
 *   带宽公式（20 Hz 下）：α = dt/(dt+τ)，dt=0.05s；α=0.15 → τ=283ms → BW≈0.56Hz。
 * 调大 α → 外环响应更快但噪声耦合更强；调小 → 更平滑但漂移修正更慢。
 */
#ifndef APP_BALANCE_SPEED_LPF_ALPHA
#define APP_BALANCE_SPEED_LPF_ALPHA             (0.15f)   /* 20 Hz 下 ~0.8 Hz 带宽；原 0.5 带宽 3.2 Hz 过快导致速度环与平衡环耦合振荡 */
#endif

/** 角度环输出"目标角速率"绝对值上限（°/s），防止角度环输出过大 */
#ifndef APP_BALANCE_MAX_TARGET_RATE_DPS
#define APP_BALANCE_MAX_TARGET_RATE_DPS         (500.0f)
#endif

/** 角速度内环 D 项 EMA 滤波系数（0=禁用）。对角加速度信号做低通。 */
#ifndef APP_BALANCE_RATE_D_FILTER_ALPHA
#define APP_BALANCE_RATE_D_FILTER_ALPHA         (0.20f)
#endif

/**
 * 俯仰角一阶低通滤波系数（0~1）。
 * MS901M 已内置 EKF，输出已经足够平滑；主控侧仅加极轻量 LPF 防串口帧毛刺。
 * 平衡内环 D 项已改用陀螺仪角速率直通（不经此 LPF），P 项延迟由此系数决定。
 * 100 Hz 下 α=0.8 约等效 4 ms 时间常数（1/2 个控制周期），几乎无相位滞后。
 * 若 MS901M 输出足够干净，可设 1.0 完全禁用 LPF。
 */
#ifndef APP_BALANCE_PITCH_LPF_ALPHA
#define APP_BALANCE_PITCH_LPF_ALPHA             (1.0f)
#endif

/** 俯仰角软件极性翻转：已通过 bsp_motor_set_invert(true,true) 修正电机极性，
 *  pitch_sign 无需再做补偿翻转，默认 0（不翻转）。
 *  若日后移除 motor invert（改为硬件接线修正），需将此宏改回 1。 */
#ifndef APP_BALANCE_PITCH_INVERT
#define APP_BALANCE_PITCH_INVERT                (0)
#endif

/**
 * Yaw 轴极性翻转（gz_dps / yaw_deg 符号修正）。
 *
 * 标准 IMU 约定：俯视逆时针 → gz_dps > 0。但本系统差速驱动约定
 * "left 加 / right 减 → 修正左偏"，要求车向左偏时 PID 输出为正。
 * 若 IMU 默认符号与差速修正方向不匹配（现象：高 Kp 无限自转），
 * 启用此翻转即可将正反馈改为负反馈。
 *
 * 判别方法：上电后手动将车顺时针转一小角度再松手，
 *   - 若 yaw_corr 先出正值再衰减 → 符号正确，设 0；
 *   - 若 yaw_corr 持续同方向增长 → 正反馈，设 1。
 */
#ifndef APP_BALANCE_YAW_INVERT
#define APP_BALANCE_YAW_INVERT                  (1)
#endif

/**
 * 速度反馈极性翻转：当编码器"前进方向"与平衡环"正PWM方向"相反时置 1。
 *
 * 判别方法（不启用速度环，仅观察心跳 v= 字段）：
 *   - 使小车在地面以正常平衡状态漂移（平衡环正常驱动，轮子向前）；
 *   - 若心跳日志 v= 为负值 → 编码器方向倒置，设 1；
 *   - 若 v= 为正值 → 方向正确，设 0（默认）。
 * 也可运行时串口 `si0` / `si1` 切换，效果等同。
 */
#ifndef APP_BALANCE_SPEED_INVERT
#define APP_BALANCE_SPEED_INVERT                (0)
#endif

/**
 * 速度反馈量纲缩放因子。
 *
 * 本系统编码器分辨率极高（左轮 68000 cnt/rev，右轮 34000 cnt/rev），
 * 典型平衡漂移速度（~0.1 rev/s）即对应 avg_cps ≈ 5100。
 * 若直接将原始 cps 送入速度 PID：
 *   - Kp = 0.001（sp 1）时 tilt = 5.1°，立刻接近 ±10° 饱和上限，
 *     导致极限环振荡——这正是"sp 后立即大幅抖动"的根因。
 *
 * 此因子将 avg_cps 除以该值后再进 PID：
 *   - SCALE=10 → 5100 cps → 510（归一化 cps）
 *   - sp 1（Kp=0.001）× 510 = 0.51°，从极小扰动开始调试，安全
 *   - sp 5（Kp=0.005）× 510 = 2.55°，中等制动
 *   - sp 20（Kp=0.02） × 510 = 10.2°，接近最大倾角（饱和）
 *
 * target_speed_cps（K230 运动指令）单位同步归一化：
 *   K230 发送 target = 500 ≡ 5000 raw cps ≡ 约 0.15 rev/s 向前。
 */
#ifndef APP_BALANCE_SPEED_CPS_SCALE
#define APP_BALANCE_SPEED_CPS_SCALE             (10)
#endif

/** Yaw 角度环开关：直行时闭环锁定航向，抑制原地偏航。 */
#ifndef APP_BALANCE_YAW_ENABLED
#define APP_BALANCE_YAW_ENABLED                 (1)
#endif

/**
 * Yaw 数据源选择（编译期一键切换，上车前按实测效果选定后重新编译）：
 *   0 = MS901M EKF 绝对偏航角 yaw_deg [-180, 180]°
 *       ─ 磁力计 + 陀螺 EKF 融合，长期稳定，不漂移；
 *       ─ 存在 ±180° 跳变风险（已用 wrap_180 + virtual_measured 技巧处理）；
 *       ─ 磁场干扰环境下 yaw_deg 可能出现 180° 跳变，导致短暂反向输出。
 *   1 = 陀螺仪积分 gz_dps × dt（相对偏航累积量）
 *       ─ 零漂约 0.2~0.5°/s，100 Hz 拍上不明显；直行短时间（< 30 s）效果好；
 *       ─ 彻底无 ±180° 跳变，无磁干扰；转向结束后积分自动清零重新锁定；
 *       ─ PID 的 D 项天然等效 gz_dps 角速率阻尼，调参直觉更好。
 */
#ifndef APP_BALANCE_YAW_SOURCE
#define APP_BALANCE_YAW_SOURCE                  (1)   /* 0=EKF yaw_deg  1=gyro_integration */
#endif

/** Yaw PID 微分项 EMA 滤波系数（0 = 禁用）。 */
#ifndef APP_BALANCE_YAW_D_FILTER_ALPHA
#define APP_BALANCE_YAW_D_FILTER_ALPHA          (0.20f)
#endif

/** Yaw 角度环输出差分补偿限幅（permille），防止大角度误差时扰动平衡内环。 */
#ifndef APP_BALANCE_YAW_MAX_CORRECTION_PM
#define APP_BALANCE_YAW_MAX_CORRECTION_PM       (200)
#endif

/* ========================================================================== */
/* 输入结构体                                                                   */
/* ========================================================================== */

/** 当前姿态（业务侧从 ms901m_get_snapshot 取后填入） */
typedef struct {
    float pitch_deg;        /* 俯仰角；车头上扬为正（与 MS901M 对齐） */
    float pitch_rate_dps;   /* 俯仰角速度，°/s（MS901M gy_dps） */
    float yaw_deg;          /* 偏航角 [-180, 180]°（MS901M 0x01 帧 EKF 输出） */
    float gz_dps;           /* 偏航角速度，°/s（MS901M gz_dps，Z 轴陀螺） */
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
    int16_t yaw_correction_pm;  /* Yaw 角度环差分补偿；正值 = 左加右减 */
    float   yaw_error_deg;      /* wrap_180(yaw_target - yaw_measured)，°  */
    int16_t left_cmd_pm;        /* 最终左轮命令 */
    int16_t right_cmd_pm;       /* 最终右轮命令 */
    int32_t speed_meas_cps;     /* 实际平均速度 = (L+R)/2 */
    bool    driving;            /* 本拍是否真在驱动（受 safety 限制） */
} app_balance_diag_t;

/* ========================================================================== */
/* API                                                                         */
/* ========================================================================== */

/** 初始化四路 PID 为安全默认（增益 0 + 输出限幅 + D 滤波系数已写入）。 */
void app_balance_init(void);

/** 复位：清四路 PID 内部状态（i_term + 微分历史 + 级联中间变量）。 */
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

/**
 * @brief 设置 Yaw 轴极性翻转（影响 gz_dps / yaw_deg 在角度环内的符号）。
 *
 * 若启用，gz_dps 在积分前取反、yaw_deg 误差在计算前取反，
 * 使 PID 输出方向与差速修正方向一致。修改后自动 reset_yaw_state。
 */
void app_balance_set_yaw_inverted(bool inverted);

/** @return true = 当前已启用 Yaw 轴软件翻转。 */
bool app_balance_get_yaw_inverted(void);

/**
 * @brief 设置速度反馈极性翻转。
 *
 * 当编码器"前进方向"（avg_cps 增大的方向）与平衡环"正PWM前进方向"相反时，
 * 速度环为正反馈，任何 Kp 都会立即发散。启用后翻转 avg_cps 符号使其变为负反馈。
 * 修改后自动 reset 速度环状态。
 *
 * 等效串口命令：`si0`（正常）/ `si1`（翻转）。
 */
void app_balance_set_speed_inverted(bool inverted);

/** @return true = 当前已启用速度反馈软件翻转。 */
bool app_balance_get_speed_inverted(void);

/** 设置角度环增益（输入：tilt 误差 deg；输出：目标角速率 °/s）。 */
void app_balance_set_balance_gains(float kp, float ki, float kd);

/** 设置角速度内环增益（输入：角速率误差 °/s；输出：PWM permille）。 */
void app_balance_set_rate_gains(float kp, float ki, float kd);

/** 设置速度外环增益（输入：速度误差 cps；输出：目标 tilt deg）。 */
void app_balance_set_speed_gains(float kp, float ki, float kd);

/**
 * @brief 设置转向开环系数：left -= yaw, right += yaw 的乘子。
 *        默认 1.0 = K230 给的 target_yaw_pm 直接当差速量加；可在 0.3 ~ 1.5 间调试。
 */
void app_balance_set_yaw_kp(float kp_yaw);

/**
 * @brief 设置 Yaw 角度环 PID 增益（输入：yaw 误差 °；输出：差分 PWM permille）。
 *        直行时闭环锁定 MS901M yaw_deg；转向时 PID 暂停并更新目标角。
 *        默认增益 0（同其他环，需串口 `yp kp ki kd` 注入）。
 */
void app_balance_set_yaw_gains(float kp, float ki, float kd);

/** 拷贝一份本拍内部诊断（不影响内部状态）。 */
void app_balance_get_diag(app_balance_diag_t *out);

/**
 * @brief 上车基线主循环入口（四级级联多速率调度）。
 *
 *        调度策略（单任务轮询 + SysTick 1 ms 节拍标志）：
 *          1 kHz : drain UART3 → ms901m_feed_bytes + bsp_motor_update
 *          200 Hz: ms901m_get_snapshot + safety + 角速度内环 + bsp_motor_set_output
 *          100 Hz: bsp_battery_update + 角度环
 *           50 Hz: 航向环
 *           20 Hz: 速度外环
 *            5 Hz: LED_G 心跳翻转 + safety 指示
 *            1 Hz: XDS-UART 调试日志
 *
 *        调用前必须保证：
 *          - SYSCFG_DL_init / bsp_gpio_init / bsp_systick_init / bsp_log_uart_init
 *          - bsp_k230_uart_init / bsp_imu_uart_init / ms901m_init (4, 2000)
 *          - bsp_motor_init / bsp_battery_init
 *          - app_safety_init / app_balance_init 全部完成
 *          - main 已通过 wait_for_ms901m_attitude 验证 0x01 帧在线
 *
 *        ⚠️ PID 增益默认 0：上电后即使站立姿态正确电机也不会动。装车整定时
 *           通过串口 / K230 命令注入增益；上层调用 `app_safety_arm()` 后才允许驱动。
 *
 * @return true = 收到 UART `t` / `test`，调用方应切入电机演示模式。
 */
bool app_balance_run(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_BALANCE_H */
