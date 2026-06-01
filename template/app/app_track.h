/**
 * @file    app_track.h
 * @brief   赛道模式主控状态机（寄生在 app_balance 主循环 20 Hz 分支）。
 *
 * 工作流程（上电自检通过、安全态 ARMED 后自动启动，亦可串口/远程触发）：
 *
 *   SELF_STAND   自立：从起始倾角（约 30~40°）"猛起"摆到直立。角度环走
 *                专用 rise 增益（kp 大 / ki=0 / kd 阻尼）把车甩起并减速；
 *                速度环全程在线（target_speed=0）维持原地、防止自立漂移。
 *   STAND_SETTLE 稳定确认：|pitch|、|gz| 持续低于阈值一段时间；确认后角度环
 *                切回运动增益（TRK_GAIN_*），交棒给循线。
 *   TRACE        循线：采纳 K230 下发的 target_v / target_omega，
 *                速度经"启动加速包络"平滑爬升；同时累计偏航 + 里程判圈。
 *   BRAKE        减速刹车：满圈后把下发速度斜坡拉到 0，保持平衡停稳。
 *   PAUSE        暂停 5 s：保持直立静止，到期进入下一圈。
 *   FINAL_BRAKE  末圈刹车：第二圈满圈后减速停稳。
 *   DONE         完成：保持直立静止。
 *
 * 圈数判定（MCU 自主）：累计偏航 |Σ gz·dt| ≥ YAW_PER_LAP_DEG 且
 * 累计里程 ≥ LAP_LENGTH_MM × 下限系数（双条件抗误判），并设超时兜底。
 *
 * 与 K230 对齐：MCU 通过 VEHICLE_STATUS.track_phase 上报当前阶段；
 * K230 仅在 TRACE 阶段下发循线驱动指令，其余阶段输出 (0,0)，防止任务冲突。
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
 * 摆稳后（STAND_SETTLE 确认）切换到运动 PID（TRK_GAIN_*）接管循线。
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

/** 稳定确认持续时长（ms）。 */
#ifndef APP_TRACK_SETTLE_MS
#define APP_TRACK_SETTLE_MS                 (400u)
#endif

/* ---- 速度包络（raw cps；K230 target_v 已 ×SCALE 还原为 raw cps） ----
 * TRACE 阶段速度透传 K230，不在 MCU 二次加速限速（与手动 WiFi trace 对齐）。
 * 下列 DECEL 仅用于 BRAKE / FINAL_BRAKE 减速停稳。 */

/** 减速刹车：每 20 Hz 拍允许的最大速度减量（raw cps，较陡）。 */
#ifndef APP_TRACK_DECEL_CPS_PER_TICK
#define APP_TRACK_DECEL_CPS_PER_TICK        (800)
#endif

/** 停稳判据：|avg_cps| 阈值（raw cps）。 */
#ifndef APP_TRACK_STOP_CPS
#define APP_TRACK_STOP_CPS                  (300)
#endif

/** 停稳确认持续时长（ms）。 */
#ifndef APP_TRACK_STOP_SETTLE_MS
#define APP_TRACK_STOP_SETTLE_MS            (500u)
#endif

/* ---- 圈数判定 ---- */
/** 每圈累计偏航阈值（°）。闭环赛道一圈约 360°，可按实际赛道标定。 */
#ifndef APP_TRACK_YAW_PER_LAP_DEG
#define APP_TRACK_YAW_PER_LAP_DEG           (360.0f)
#endif

/** 每圈赛道周长（mm），上车实测后更新。 */
#ifndef APP_TRACK_LAP_LENGTH_MM
#define APP_TRACK_LAP_LENGTH_MM             (2500)
#endif

/** 里程下限系数（×100）：里程 ≥ 周长 × 此系数/100 才允许判圈，抗早停。 */
#ifndef APP_TRACK_LAP_ARC_MIN_X100
#define APP_TRACK_LAP_ARC_MIN_X100          (60)
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
    int32_t  applied_cps;       /* 当前速度包络输出（raw cps） */
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
