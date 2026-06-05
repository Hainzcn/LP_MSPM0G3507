/**
 * @file    app_track.h
 * @brief   赛道模式主控状态机（寄生在 app_balance 主循环 20 Hz 分支）。
 *
 * 工作流程（上电自检通过、安全态 ARMED 后自动启动，亦可串口/远程触发）：
 *
 *   SELF_STAND   自立：rise 角度环；蜂鸣器/激光均关闭。
 *   STAND_SETTLE 进入即切运动角度 PID；稳定累计 SETTLE_MS 后 TRACE；提示音 STOOD_UP。
 *   TRACE        循线 + 激光开；提示音 TRACE_START。
 *   BRAKE        满圈反速后仰刹停 + K230 转向；激光开。
 *   PAUSE        暂停 5 s；提示音 LAP_PAUSE；激光保持开。
 *   FINAL_BRAKE  末圈反速刹停（同上）；激光保持开。
 *   DONE         完成；提示音 ALL_DONE；激光关。
 *
 * 圈数判定（MCU 自主，优先级见 app_track.h 判圈宏）：
 *   ① 偏航 ≥ YAW_PER_LAP 且 arc ≥ arc_min（防原地空转）
 *   ② arc ≥ 周长×ARC_COMPLETE% 且 yaw ≥ YAW_MIN（里程收口，防 gyro 偏小多走）
 *   ③ 单圈超时兜底
 *
 * 与 K230 对齐：MCU 通过 VEHICLE_STATUS.track_phase 上报当前阶段；
 * K230 仅在 TRACE 阶段下发循线驱动指令；激光使能由 MCU PA1（bsp_laser）控制。
 */

#ifndef APP_TRACK_H
#define APP_TRACK_H

#include "app_balance.h"
#include "bsp_motor.h"
#include "ms901m.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/* 阶段枚举（须与 K230 comms/protocol.py 的 TRACK_PHASE_* 保持一致）            */
/* ========================================================================== */

typedef enum {
    APP_TRACK_IDLE        = 0,
    APP_TRACK_SELF_STAND  = 1,
    APP_TRACK_STAND_SETTLE = 2,
    APP_TRACK_TRACE       = 3,
    APP_TRACK_BRAKE       = 4,
    APP_TRACK_PAUSE       = 5,
    APP_TRACK_FINAL_BRAKE = 6,
    APP_TRACK_DONE        = 7,

    APP_TRACK_FORCE_INT32_ = 0x7FFFFFFF
} app_track_phase_t;

/* ========================================================================== */
/* 编译期可配宏                                                                 */
/* ========================================================================== */

/** 上电自检通过后是否自动启动赛道模式（0=仅手动 trk/远程触发）。 */
#ifndef APP_TRACK_AUTOSTART
#define APP_TRACK_AUTOSTART                 (1)
#endif

/**
 * 自动启动前是否等待 K230 在线再起立（1=等待）。
 *
 *   冷上电时 MCU 自检（~5 s）远早于 K230 启动（屏幕亮起约需 10 s）。若在 K230
 *   未就绪时就 ARMED→自立，会遇到：① 编码器引脚噪声雪崩（encISR 数万、ISR_QUENCH）；
 *   ② IMU EKF 尚未收敛、俯仰角不可信；导致电机疯冲。等 K230 在线后再起立可一并
 *   规避（此时 IMU 也已收敛、旁路电容已充满）。等待期间增益保持 0、电机不驱动。
 */
#ifndef APP_TRACK_AUTOSTART_WAIT_K230
#define APP_TRACK_AUTOSTART_WAIT_K230       (1)
#endif

/**
 * 等待 K230 在线的最长时长（ms，从安全态首次进入 ARMED 起算）。
 *   超时仍未等到 K230 则不再等待、直接启动（自立可独立完成，循线需 K230）。
 *   K230 冷启动约 10 s，默认 15 s 留足裕度。
 */
#ifndef APP_TRACK_AUTOSTART_K230_WAIT_MS
#define APP_TRACK_AUTOSTART_K230_WAIT_MS    (15000u)
#endif

/** K230 首次在线后，再等待多久才触发自立（ms）。规避 K230 上电瞬间 EMI/编码器噪声。 */
#ifndef APP_TRACK_K230_ONLINE_DELAY_MS
#define APP_TRACK_K230_ONLINE_DELAY_MS      (1000u)
#endif

/** 完成圈数。 */
#ifndef APP_TRACK_N_LAPS
#define APP_TRACK_N_LAPS                    (2u)
#endif

/* ---- 自立 PID（专用一套，猛起→减速→稳定；与运动 PID 分离） ----
 *
 * 自立无法靠"温和加速"实现：从 30~40° 倾倒摆到直立需要一记"猛起"驱动轮子
 * 把车身甩起来，临近直立时再由阻尼项（kd）减速，最后稳定在 0°。因此自立段
 * 单独走一套角度环增益：
 *   - kp 大：提供起摆冲量（猛起）；
 *   - ki = 0：自立段误差大且持续，积分必然 windup，故关掉；
 *   - kd > 0：核心阻尼项，临近直立减速，抑制过冲/震荡（"减速-稳定"）；
 *   - 全输出权限（不额外限幅），保证猛起力度。
 * 进入 STAND_SETTLE 时切换到运动角度 PID（TRK_GAIN_*）；稳定确认后再进 TRACE 循线。
 *
 * 注意：这几个值需上车实测整定。下面是合理起点，kd 是首要调节对象——
 * 过小→过冲震荡数秒，过大→起摆无力/抖动。命令量换算同 bp：x1000。
 */
/** 自立角度环 kp（起摆冲量）。 */
#ifndef APP_TRACK_RISE_KP
#define APP_TRACK_RISE_KP                   (50.0f)
#endif
/** 自立角度环 ki（固定 0，关积分防 windup）。 */
#ifndef APP_TRACK_RISE_KI
#define APP_TRACK_RISE_KI                   (0.0f)
#endif
/** 自立角度环 kd（阻尼/减速项，首要整定对象）。 */
#ifndef APP_TRACK_RISE_KD
#define APP_TRACK_RISE_KD                   (8.0f)
#endif
/** 自立角度环死区补偿 offset（permille）。 */
#ifndef APP_TRACK_RISE_OFS
#define APP_TRACK_RISE_OFS                  (20.0f)
#endif

/** 自立摆起最大窗口（ms）：超过仍未达直立判据则强制进入稳定确认兜底。 */
#ifndef APP_TRACK_RISE_MS
#define APP_TRACK_RISE_MS                   (1000u)
#endif

/** 自立完成判据：|pitch| 阈值（°）。 */
#ifndef APP_TRACK_RISE_DONE_DEG
#define APP_TRACK_RISE_DONE_DEG             (15.0f)
#endif

/** 自立完成判据：|gz| 阈值（°/s）。 */
#ifndef APP_TRACK_RISE_DONE_DPS
#define APP_TRACK_RISE_DONE_DPS             (40.0f)
#endif

/** 稳定确认持续时长（ms）：|pitch|/|gz| 连续满足 RISE_DONE 判据的累计时间。 */
#ifndef APP_TRACK_SETTLE_MS
#define APP_TRACK_SETTLE_MS                 (5000u)
#endif

/* ---- 反速后仰刹停（raw cps；K230 target_v 已 ×SCALE 还原） ----
 * TRACE 阶段速度透传 K230，不在 MCU 二次加速限速。
 * BRAKE / FINAL_BRAKE：先施加与行驶方向相反的目标速度（后仰制动），
 * 持续 BRAKE_REVERSE_MS 后改 target_speed=0，待实测轮速低于 STOP_CPS 并
 * 稳定 STOP_SETTLE_MS 后进入下一阶段。反速脉冲期间保留 K230 target_dif。 */

/** 反速刹车目标速度幅值（raw cps，符号由进入刹车时的行驶方向决定）。 */
#ifndef APP_TRACK_BRAKE_REVERSE_CPS
#define APP_TRACK_BRAKE_REVERSE_CPS           (20000)
#endif

/** 反速后仰脉冲持续时间（ms）。 */
#ifndef APP_TRACK_BRAKE_REVERSE_MS
#define APP_TRACK_BRAKE_REVERSE_MS            (200u)
#endif

/** 收束停稳判据：|avg_cps| 阈值（raw cps，反速脉冲结束后生效）。 */
#ifndef APP_TRACK_STOP_CPS
#define APP_TRACK_STOP_CPS                  (300)
#endif

/** 收束停稳确认持续时长（ms）：反速后 |avg| 低于 STOP_CPS 的累计时间。 */
#ifndef APP_TRACK_STOP_SETTLE_MS
#define APP_TRACK_STOP_SETTLE_MS            (500u)
#endif

/** 刹车阶段最长停留（ms）：超时强制进入 PAUSE / DONE。 */
#ifndef APP_TRACK_BRAKE_MAX_MS
#define APP_TRACK_BRAKE_MAX_MS              (3000u)
#endif

/* ---- 圈数判定 ----
 *
 * 优先级（与 app_circle_demo 一致：主判据 OR 收口，而非 AND 双门槛）：
 *
 *   ① 偏航主判据：|Σgz·dt| ≥ YAW_PER_LAP_DEG 且 arc ≥ arc_min
 *      arc_min 仅作“防早停”下界，避免原地空转仅靠 yaw 误判满圈。
 *
 *   ② 里程收口：arc ≥ LAP_LENGTH×ARC_COMPLETE% 且 yaw ≥ YAW_MIN_FOR_ARC
 *      当陀螺积分系统性偏小（常见多走 ~30°）时，以编码器弧长先收口，
 *      不必等 yaw 积满 360° 才触发 BRAKE。
 *
 *   ③ LAP_TIMEOUT_MS 超时兜底。
 *
 * 上车标定：LAP_LENGTH_MM 填实测周长；若仍偏早/偏晚，微调 YAW_PER_LAP 或
 * ARC_COMPLETE%。
 */
/** 每圈累计偏航阈值（°）。闭环赛道一圈约 360°，可按 gyro 偏差下调（如 330）。 */
#ifndef APP_TRACK_YAW_PER_LAP_DEG
#define APP_TRACK_YAW_PER_LAP_DEG           (360.0f)
#endif

/** 每圈赛道周长（mm），上车实测后更新。 */
#ifndef APP_TRACK_LAP_LENGTH_MM
#define APP_TRACK_LAP_LENGTH_MM             (2500)
#endif

/** 里程下界系数（×100）：arc ≥ 周长×此值/100 才允许 ① 偏航判圈（抗原地空转）。 */
#ifndef APP_TRACK_LAP_ARC_MIN_X100
#define APP_TRACK_LAP_ARC_MIN_X100          (60)
#endif

/** 里程收口系数（×100）：arc ≥ 周长×此值/100 且 yaw 足够时 ② 直接满圈。默认 92≈330°/360°。 */
#ifndef APP_TRACK_LAP_ARC_COMPLETE_X100
#define APP_TRACK_LAP_ARC_COMPLETE_X100     (90)
#endif

/** ② 里程收口要求的最小累计偏航（°），防止长直道仅靠里程误触发。 */
#ifndef APP_TRACK_YAW_MIN_FOR_ARC_DEG
#define APP_TRACK_YAW_MIN_FOR_ARC_DEG       (300.0f)
#endif

/** 单圈超时兜底（ms）：超过则强制判圈，防止视觉异常时卡死。 */
#ifndef APP_TRACK_LAP_TIMEOUT_MS
#define APP_TRACK_LAP_TIMEOUT_MS            (60000u)
#endif

/* ---- 暂停 ---- */
/** 满一圈后的暂停时长（ms）。 */
#ifndef APP_TRACK_PAUSE_MS
#define APP_TRACK_PAUSE_MS                  (5000u)
#endif

/* ========================================================================== */
/* 诊断快照                                                                     */
/* ========================================================================== */

typedef struct {
    app_track_phase_t phase;
    uint8_t  lap;               /* 当前圈号（1 起；0=未开始） */
    float    yaw_accum_deg;     /* 本圈累计偏航（°） */
    int32_t  arc_mm;            /* 本圈累计里程（mm） */
    int32_t  applied_cps;       /* 刹车段当前 target_speed（raw cps，诊断用） */
    uint32_t phase_elapsed_ms;  /* 当前阶段已用时（ms） */
} app_track_diag_t;

/* ========================================================================== */
/* API                                                                         */
/* ========================================================================== */

/** 初始化（置 IDLE）。 */
void app_track_init(void);

/** 启动赛道模式：进入 SELF_STAND（套用赛道预置 PID 增益）。 */
void app_track_start(void);

/** 取消赛道模式：释放自立覆盖，回到 IDLE。 */
void app_track_cancel(void);

/** @return true = 赛道状态机处于非 IDLE 活动阶段。 */
bool app_track_is_active(void);

/** @return 当前阶段枚举（用于 VEHICLE_STATUS 上报）。 */
app_track_phase_t app_track_get_phase(void);

/** @return 当前圈号（1 起；0=未开始）。 */
uint8_t app_track_get_lap(void);

/** 拷贝一份诊断快照。 */
void app_track_get_diag(app_track_diag_t *out);

/**
 * @brief 20 Hz 调度入口，由 app_balance_run 在速度环之前调用。
 *
 * 活动阶段会覆盖 cmd->target_speed_cps / target_dif_cps，并按阶段切换角度环
 * 增益（自立 rise 增益 ↔ 运动增益）。IDLE 阶段不修改 cmd。
 *
 * @param snap  最新 IMU 快照（gz_dps 用于偏航积分）
 * @param fb    最新编码器反馈（count 用于里程、speed_cps 用于停稳判据）
 * @param cmd   运动指令结构体（可能被覆盖）
 */
void app_track_tick_20hz(const ms901m_snapshot_t *snap,
                         const bsp_motor_feedback_t *fb,
                         app_balance_motion_cmd_t *cmd);

#ifdef __cplusplus
}
#endif

#endif /* APP_TRACK_H */
