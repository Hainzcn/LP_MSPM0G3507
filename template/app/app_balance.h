/**
 * @file    app_balance.h
 * @brief   平衡车两级级联控制（角度环 100Hz + 速度/转向环 20Hz）
 *
 * ============================================================================
 * ⚠️ 整定提示（必读）
 * ============================================================================
 * 本模块**默认所有 PID 增益 = 0**，即上电后输出恒为 0、电机不会自己动。
 *
 * 推荐整定顺序（与 STM32 demo 对齐）：
 *
 *   ① 串口 `bo 80`        → 粗调 OutOffset 突破 TB6612 死区
 *   ② 串口 `bp 5 0 5 80`  → (kp/ki/kd/offset) 角度环能站稳
 *   ③ 串口 `sp 2 0.05 0 0`→ 速度环消静态漂移
 *   ④ 串口 `yp 800 0 200 0` → 航向角环（锁 yaw，见 APP_BALANCE_YAW_SOURCE）
 *
 * ============================================================================
 * 调用模型
 * ============================================================================
 *   bsp_motor_init() / bsp_battery_init() / app_safety_init();
 *   app_balance_init();
 *   app_balance_set_balance_gains(kp, ki, kd, out_offset);
 *   app_balance_set_speed_gains  (kp, ki, kd, out_offset);
 *   app_balance_set_yaw_gains    (kp, ki, kd, out_offset);  航向角环
 *   app_balance_run();   内部多速率调度 100/20 Hz
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

/** 角度环周期（毫秒）。 */
#ifndef APP_BALANCE_ANGLE_PERIOD_MS
#define APP_BALANCE_ANGLE_PERIOD_MS             (10u)   /* 100 Hz */
#endif

/** 速度/转向外环周期（毫秒）。 */
#ifndef APP_BALANCE_SPEED_PERIOD_MS
#define APP_BALANCE_SPEED_PERIOD_MS             (50u)   /*  20 Hz */
#endif

/** 速度外环输出"目标俯仰角"绝对值上限（°），防止外环推到内环工作区外。 */
#ifndef APP_BALANCE_MAX_TILT_DEG
#define APP_BALANCE_MAX_TILT_DEG                (10.0f)
#endif

/** 平衡角度环输出 PWM 绝对值上限（permille）。 */
#ifndef APP_BALANCE_MAX_PWM_PERMILLE
#define APP_BALANCE_MAX_PWM_PERMILLE            (1000)
#endif

/** 航向环差分 PWM 绝对值上限（permille）。 */
#ifndef APP_BALANCE_YAW_MAX_CORRECTION_PM
#define APP_BALANCE_YAW_MAX_CORRECTION_PM       (200)
#endif

/** 航向角环开关：直行时闭环锁定偏航（抑制原地绕圈）。 */
#ifndef APP_BALANCE_YAW_ENABLED
#define APP_BALANCE_YAW_ENABLED                 (1)
#endif

/**
 * 航向角环数据源（编译期）：
 *   0 = MS901M EKF 绝对偏航角 yaw_deg（virtual measured 避免 ±180° 跳变）
 *   1 = 陀螺积分 gz_dps×dt（无磁干扰，默认）
 */
#ifndef APP_BALANCE_YAW_SOURCE
#define APP_BALANCE_YAW_SOURCE                  (1)
#endif

/** 航向角环周期（毫秒），与速度外环同拍。 */
#ifndef APP_BALANCE_YAW_PERIOD_MS
#define APP_BALANCE_YAW_PERIOD_MS               APP_BALANCE_SPEED_PERIOD_MS
#endif

/**
 * Yaw 轴极性翻转（yaw_deg / gz_dps 符号修正）。
 * 判别：顺时针轻推车体，yawCorr 应先正后衰减 → 正确则 0，持续同向增大则 1。
 * 串口 `yi0` / `yi1` 可运行时切换。
 */
#ifndef APP_BALANCE_YAW_INVERT
#define APP_BALANCE_YAW_INVERT                  (1)
#endif

/** 航向角环积分项独立上限（permille）。 */
#ifndef APP_BALANCE_YAW_INTEGRAL_LIMIT_PM
#define APP_BALANCE_YAW_INTEGRAL_LIMIT_PM       (100.0f)
#endif

/**
 * 跌倒判据：|pitch_meas_deg| 超过此角度则判为已跌倒，切 DISARMED。
 * 与 STM32 demo 对齐（原 60° 偏大，容易在真正倒下时仍持续驱动电机）。
 */
#ifndef APP_BALANCE_FALL_PITCH_DEG
#define APP_BALANCE_FALL_PITCH_DEG              (60.0f)
#endif

/**
 * 角度环 OutOffset（permille）。
 *
 * 关键死区补偿参数：当 PID 算出微弱控制量时，若不加此偏移则电机无法动作，
 * 积分不断累积最终"突然冲出"。需上车实测 TB6612+N20 的实际死区后微调。
 * 典型 TB6612 死区：60~100 permille；先设 80，逐步微调。
 */
#ifndef APP_BALANCE_ANGLE_OUT_OFFSET
#define APP_BALANCE_ANGLE_OUT_OFFSET            (80.0f)
#endif

/** 速度环 OutOffset（permille）。速度环输出为角度°，通常不需要死区补偿。 */
#ifndef APP_BALANCE_SPEED_OUT_OFFSET
#define APP_BALANCE_SPEED_OUT_OFFSET            (0.0f)
#endif

/**
 * 速度反馈低通滤波系数（0~1）。
 *
 * 编码器 20ms 差分窗口在低速时量化噪声严重，此 EMA 平滑速度测量：
 *   20 Hz × α=0.15 → 时间常数 ~280ms → 带宽 ~0.6 Hz
 */
#ifndef APP_BALANCE_SPEED_LPF_ALPHA
#define APP_BALANCE_SPEED_LPF_ALPHA             (0.15f)
#endif

/**
 * 俯仰角软件极性翻转。
 *
 * MS901M 前后方向装反时，前倾会被解析成后倾，平衡环反向输出。
 * 置 1 后 pitch_meas = -(pitch_deg - offset)，仅在 app_balance 层翻转，
 * 不改传感器解析层与自校零逻辑。
 *
 * 判别方法：上电后用手把车向前推，串口心跳 pitch= 应变为正值；若变负 → 置 1。
 */
#ifndef APP_BALANCE_PITCH_INVERT
#define APP_BALANCE_PITCH_INVERT                (0)
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
 * 此因子将 avg_cps 除以该值后再进 PID，使速度值落在合理范围。
 *   SCALE=10 → 5100 cps → 510（归一化 cps）
 */
#ifndef APP_BALANCE_SPEED_CPS_SCALE
#define APP_BALANCE_SPEED_CPS_SCALE             (10)
#endif

/**
 * 上电静止采样时长（毫秒）。用于自动校准 pitch_offset_deg：
 *
 *   app_balance_run() 入口阻塞此段时间，连续对 MS901M pitch_deg 求平均后
 *   写入 pitch_offset_deg。期间电机不输出（PWM 维持 0）。
 *   设 0 跳过自动校零，沿用 init 时的初始值。
 *
 *   小车上电姿态必须为"标准直立 + 静止"，否则会把偏角学进零点。
 */
#ifndef APP_BALANCE_PITCH_AUTOZERO_MS
#define APP_BALANCE_PITCH_AUTOZERO_MS           (1500u)
#endif

/** 静止判据：自校零期间允许的最大 |gy_dps|（°/s）。超过判为非静止，丢样。 */
#ifndef APP_BALANCE_PITCH_AUTOZERO_RATE_DEADBAND_DPS
#define APP_BALANCE_PITCH_AUTOZERO_RATE_DEADBAND_DPS  (2.0f)
#endif

/** 静止判据：自校零期间允许 |a_mag - 1g|（g）超过则丢样（侦测轻微扰动）。 */
#ifndef APP_BALANCE_PITCH_AUTOZERO_ACC_DEVIATION_G
#define APP_BALANCE_PITCH_AUTOZERO_ACC_DEVIATION_G    (0.05f)
#endif

/* ========================================================================== */
/* 输入结构体                                                                   */
/* ========================================================================== */

/** 当前姿态（业务侧从 ms901m_get_snapshot 取后填入） */
typedef struct {
    float pitch_deg;        /* 俯仰角；车头上扬为正（与 MS901M 对齐） */
    float pitch_rate_dps;   /* 俯仰角速度，°/s（MS901M gy_dps，仅自校零用） */
    float yaw_deg;          /* 偏航角 [-180, 180]°（MS901M EKF，航向环源 0 使用） */
    float gz_dps;           /* 偏航角速度 °/s（航向环源 1 积分用） */
    float ax_g;             /* X 轴加速度（g），用于 EKF 耦合门控 */
    float ay_g;             /* Y 轴加速度（g），用于 EKF 耦合门控 */
    float az_g;             /* Z 轴加速度（g），用于 EKF 耦合门控 */
    bool  attitude_valid;   /* 0x01 帧是否至少收到过 */
} app_balance_attitude_t;

/** 运动指令（业务侧从 K230 命令 / 本地状态机解析后填入） */
typedef struct {
    int32_t target_speed_cps;   /* 期望前进速度（counts/s 平均 = (L+R)/2）；正 = 前进 */
    int16_t target_yaw_pm;      /* 开环转向差分量（permille）；正 = 左加右减（顺时针俯视）。
                                 * 非零时同时禁用 yaw 闭环，值真实叠加到左右轮 PWM 差分。 */
} app_balance_motion_cmd_t;

/* ========================================================================== */
/* 调试 / 遥测快照                                                              */
/* ========================================================================== */

/** 调用方读取本拍内部状态用，便于 1 Hz / 10 Hz 串口打印调试 */
typedef struct {
    float   target_tilt_deg;    /* 速度外环输出 = 角度环目标角（°） */
    float   pitch_meas_deg;     /* 实际俯仰（已减零点，°） */
    float   balance_out_pwm;    /* 角度环输出 PWM（permille） */
    float   yaw_error_deg;      /* 航向角环误差（°）；源 1 时为 -gz 积分量 */
    int16_t yaw_correction_pm;  /* 航向环差分补偿（permille）；正值 = 左加右减 */
    int16_t left_cmd_pm;        /* 最终左轮命令（permille） */
    int16_t right_cmd_pm;       /* 最终右轮命令（permille） */
    int32_t speed_meas_cps;     /* 实际平均速度 = (L+R)/2（counts/s） */
    bool    driving;            /* 本拍是否真在驱动（受 safety 限制） */
} app_balance_diag_t;

/* ========================================================================== */
/* API                                                                         */
/* ========================================================================== */

/** 初始化三路 pid2_t 为安全默认（增益 0 + 限幅 + OutOffset）。 */
void app_balance_init(void);

/** 复位：清三路 PID 内部状态（i_term + 微分历史 + 级联中间变量）。 */
void app_balance_reset(void);

/**
 * @brief 设置静态俯仰零点（°）。让车体在地面"标准直立"状态下读 1 s 平均 pitch
 *        填入此处，运行时 `raw_pitch - offset` 会真正反映"偏离平衡点的角度"。
 */
void app_balance_set_pitch_offset(float deg);

/**
 * @brief 设置速度反馈极性翻转。
 *
 * 当编码器"前进方向"（avg_cps 增大的方向）与平衡环"正PWM前进方向"相反时，
 * 速度环为正反馈，任何 Kp 都会立即发散。启用后翻转 avg_cps 符号使其变为负反馈。
 * 修改后自动 reset 速度环状态。等效串口命令：`si0` / `si1`。
 */
void app_balance_set_speed_inverted(bool inverted);

/** @return true = 当前已启用速度反馈软件翻转。 */
bool app_balance_get_speed_inverted(void);

/**
 * @brief 设置角度环增益与死区补偿。
 * @param kp         比例增益
 * @param ki         积分增益（调试阶段建议先设 0）
 * @param kd         微分增益（微分先行，作用于 pitch 变化率）
 * @param out_offset 死区补偿（permille），突破 TB6612 静摩擦；典型 60~100
 */
void app_balance_set_balance_gains(float kp, float ki, float kd, float out_offset);

/**
 * @brief 设置速度外环增益与死区补偿。
 * @param kp         比例增益
 * @param ki         积分增益
 * @param kd         微分增益
 * @param out_offset 死区补偿（°），速度环通常设 0
 */
void app_balance_set_speed_gains(float kp, float ki, float kd, float out_offset);

/** 设置航向角环增益（闭环锁 yaw）。 */
void app_balance_set_yaw_gains(float kp, float ki, float kd, float out_offset);

void app_balance_set_yaw_inverted(bool inverted);
bool app_balance_get_yaw_inverted(void);

/** 拷贝一份本拍内部诊断（不影响内部状态）。 */
void app_balance_get_diag(app_balance_diag_t *out);

/**
 * @brief 上车基线主循环入口（两级级联多速率调度）。
 *
 *        调度策略（单任务轮询 + SysTick 1 ms 节拍标志）：
 *          1 kHz : drain UART3 → ms901m_feed_bytes + bsp_motor_update
 *          100 Hz: ms901m_get_snapshot + safety + 角度环 + bsp_motor_set_output
 *           20 Hz: 速度/转向外环 + bsp_battery_update
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
 *           通过串口命令注入增益；上层调用 `app_safety_arm()` 后才允许驱动。
 *
 * @return true = 收到 UART `t` / `test`，调用方应切入电机演示模式。
 */
bool app_balance_run(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_BALANCE_H */
