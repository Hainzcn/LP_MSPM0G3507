/**
 * @file    robot_param.h
 * @brief   小车物理参数集中定义：尺寸、惯量、单轮速度换算、差速运动学。
 *
 * 设计原则
 * ─────────────────────────────────────────────────────────────────────────
 * 本文件是整车物理参数的唯一真源（Single Source of Truth）：
 *   ─ 尺寸：轮径、轴距、整车质量、重心高度
 *   ─ 传动：减速比、霍尔 PPR、各轮解码倍率 → 每转计数
 *   ─ 换算：每轮 CPS ↔ m/s，物理平均速度
 *   ─ 运动学：差速模型 (v, ω) ↔ (v_L, v_R)
 *
 * 层级关系（无循环依赖）
 *   robot_param.h  ← 自包含，不依赖任何 BSP 头文件
 *   bsp_motor.c    ← 同时包含 bsp_motor.h 与 robot_param.h，填充 *_speed_mps
 *   app_balance.c  ← 包含 robot_param.h，使用物理单位接口
 *
 * ⚠️  编码器参数（齿轮比、PPR、解码倍率）在本文件定义，与 bsp_motor.h 中的
 *     BSP_MOTOR_LEFT/RIGHT_COUNTS_PER_OUTPUT_REV **必须一致**：
 *       ROBOT_LEFT_COUNTS_PER_OUTPUT_REV  应 == BSP_MOTOR_LEFT_COUNTS_PER_OUTPUT_REV
 *       ROBOT_RIGHT_COUNTS_PER_OUTPUT_REV 应 == BSP_MOTOR_RIGHT_COUNTS_PER_OUTPUT_REV
 *     若更换电机，两处同步修改。
 */

#ifndef ROBOT_PARAM_H
#define ROBOT_PARAM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/* §1  物理尺寸参数（实测值，单位 mm / kg）                                     */
/* ========================================================================== */

/**
 * 驱动轮直径（mm）。
 * 测量方法：卡尺测轮胎外径（充气轮胎宜在承重状态下测）。
 */
#ifndef ROBOT_WHEEL_DIAMETER_MM
#define ROBOT_WHEEL_DIAMETER_MM     (35.0f)
#endif

/**
 * 轴距：左右驱动轮接触中心点间距（mm）。
 * 测量方法：两轮同轴内缘距 + 两侧半轮宽之和；或直接量轮中心到轮中心的水平距离。
 */
#ifndef ROBOT_WHEELBASE_MM
#define ROBOT_WHEELBASE_MM          (184.0f)
#endif

/**
 * 整车质量（kg）。
 * TODO: 待台秤实测后替换（含电池、LaunchPad、电机、外壳）。
 */
#ifndef ROBOT_MASS_KG
#define ROBOT_MASS_KG               (1.5f)
#endif

/**
 * 重心高度（mm，从驱动轮接地面量到整车重心位置）。
 * TODO: 待实测或三维模型测量后替换；用于未来模型化平衡控制（LQR 等）。
 * 粗估方法：绕轮轴悬挂小车，记录悬挂点与铅垂线的关系。
 */
#ifndef ROBOT_COM_HEIGHT_MM
#define ROBOT_COM_HEIGHT_MM         (150.0f)
#endif

/* ========================================================================== */
/* §2  传动链参数（与 bsp_motor.h 中 BSP_MOTOR_* 保持一致）                    */
/* ========================================================================== */

/** 减速电机减速比（输出轴转速 = 电机轴转速 / GEAR_RATIO）。 */
#define ROBOT_GEAR_RATIO            (34.0f)

/** 霍尔编码器单相每圈脉冲数（A 相单沿计）。 */
#define ROBOT_HALL_PPR              (500u)

/**
 * 左轮解码倍率（TIMG8 硬件 QEI X4）。
 * 500 PPR × 34:1 × 4 = 68,000 cnt/rev
 */
#define ROBOT_LEFT_DECODE_X         (4u)

/**
 * 右轮解码倍率（GPIO 中断软件 X2）。
 * X4 在 >180 RPM 时边沿率超过 ISR 兜底阈值，故固定使用 X2：
 * 500 PPR × 34:1 × 2 = 34,000 cnt/rev
 */
#define ROBOT_RIGHT_DECODE_X        (2u)

/** 左轮每输出轴一圈的编码器计数（= GEAR_RATIO × HALL_PPR × LEFT_DECODE_X）。 */
#define ROBOT_LEFT_COUNTS_PER_OUTPUT_REV  \
    ((uint32_t)((uint32_t)ROBOT_GEAR_RATIO * ROBOT_HALL_PPR * ROBOT_LEFT_DECODE_X))
/* = 34 × 500 × 4 = 68,000 */

/** 右轮每输出轴一圈的编码器计数（= GEAR_RATIO × HALL_PPR × RIGHT_DECODE_X）。 */
#define ROBOT_RIGHT_COUNTS_PER_OUTPUT_REV \
    ((uint32_t)((uint32_t)ROBOT_GEAR_RATIO * ROBOT_HALL_PPR * ROBOT_RIGHT_DECODE_X))
/* = 34 × 500 × 2 = 34,000 */

/* ========================================================================== */
/* §3  派生几何量                                                               */
/* ========================================================================== */

#ifndef ROBOT_PI
#define ROBOT_PI  (3.14159265f)
#endif

/** 驱动轮周长（m）= π × 直径(m)。 */
#define ROBOT_WHEEL_CIRCUMFERENCE_M \
    (ROBOT_PI * (ROBOT_WHEEL_DIAMETER_MM / 1000.0f))

/** 驱动轮半径（m）。 */
#define ROBOT_WHEEL_RADIUS_M        (ROBOT_WHEEL_DIAMETER_MM / 2000.0f)

/** 轴距（m）。 */
#define ROBOT_WHEELBASE_M           (ROBOT_WHEELBASE_MM / 1000.0f)

/** 重心高度（m）。 */
#define ROBOT_COM_HEIGHT_M          (ROBOT_COM_HEIGHT_MM / 1000.0f)

/* ========================================================================== */
/* §4  单轮速度换算                                                             */
/*                                                                             */
/* 左右轮解码倍率不同（X4 vs X2），相同物理转速下：                              */
/*   left_cps = 2 × right_cps                                                 */
/* 因此 m/s 换算必须各用自己的 counts/rev，不可共用同一系数。                   */
/* ========================================================================== */

/**
 * 左轮 CPS → 线速度换算系数（cps/(m/s)）。
 * v_L(m/s) = left_cps / ROBOT_LEFT_CPS_PER_MPS
 */
#define ROBOT_LEFT_CPS_PER_MPS  \
    ((float)ROBOT_LEFT_COUNTS_PER_OUTPUT_REV / ROBOT_WHEEL_CIRCUMFERENCE_M)
/* ≈ 68000 / (π×0.035) ≈ 618,411 cps·s/m */

/**
 * 右轮 CPS → 线速度换算系数（cps/(m/s)）。
 * v_R(m/s) = right_cps / ROBOT_RIGHT_CPS_PER_MPS
 */
#define ROBOT_RIGHT_CPS_PER_MPS \
    ((float)ROBOT_RIGHT_COUNTS_PER_OUTPUT_REV / ROBOT_WHEEL_CIRCUMFERENCE_M)
/* ≈ 34000 / (π×0.035) ≈ 309,205 cps·s/m */

/** 左轮 CPS 转 m/s。 */
#define ROBOT_LEFT_CPS_TO_MPS(cps)    ((float)(cps) / ROBOT_LEFT_CPS_PER_MPS)

/** 右轮 CPS 转 m/s。 */
#define ROBOT_RIGHT_CPS_TO_MPS(cps)   ((float)(cps) / ROBOT_RIGHT_CPS_PER_MPS)

/** 左轮 m/s 转 CPS（返回 int32_t 截断值）。 */
#define ROBOT_MPS_TO_LEFT_CPS(mps)    ((int32_t)((mps) * ROBOT_LEFT_CPS_PER_MPS))

/** 右轮 m/s 转 CPS（返回 int32_t 截断值）。 */
#define ROBOT_MPS_TO_RIGHT_CPS(mps)   ((int32_t)((mps) * ROBOT_RIGHT_CPS_PER_MPS))

/**
 * 物理平均线速度（m/s）：从左右轮各自的 CPS 值计算整车前向速度。
 *
 * 注意：不能直接对 CPS 求平均后再换算——因为左轮（X4）和右轮（X2）的
 * counts/rev 不同，同一转速下 left_cps = 2 × right_cps。
 * 正确做法是先各自换算为 m/s，再取平均：
 *   v_avg = (left_cps / LEFT_CPR + right_cps / RIGHT_CPR) × CIRC / 2
 *
 * 等效归一化到右轮单位的快速公式（避免两次除法）：
 *   v_avg = (left_cps/2 + right_cps) / 2 / RIGHT_CPR × CIRC
 *         = (left_cps/2 + right_cps) / (2 × RIGHT_CPR) × CIRC
 */
#define ROBOT_AVG_CPS_TO_MPS(l_cps, r_cps) \
    (((float)(l_cps) / ROBOT_LEFT_CPS_PER_MPS + \
      (float)(r_cps) / ROBOT_RIGHT_CPS_PER_MPS) * 0.5f)

/**
 * 整车前向速度（m/s）→ "右轮等效 CPS"（供速度环使用的归一化参考量）。
 *
 * 由于 LEFT_CPR = 2 × RIGHT_CPR，"右轮等效 CPS" 与物理速度呈线性关系：
 *   eq_cps = v_mps × RIGHT_CPS_PER_MPS
 *
 * 这也是修正后的 avg_cps 在直行时的真实含义：
 *   avg_cps_corrected = (left_cps/2 + right_cps) / 2 ≈ right_cps（直行时）
 */
#define ROBOT_MPS_TO_EQ_CPS(mps)  ((int32_t)((mps) * ROBOT_RIGHT_CPS_PER_MPS))

/* ========================================================================== */
/* §5  性能极值（参考用，不参与控制运算）                                        */
/* ========================================================================== */

/** 电机额定最高转速（输出轴，RPM）。 */
#define ROBOT_MAX_WHEEL_RPM         (300.0f)

/** 对应最高线速度（m/s）= MAX_RPM / 60 × 周长。 */
#define ROBOT_MAX_SPEED_MPS         (ROBOT_MAX_WHEEL_RPM / 60.0f * ROBOT_WHEEL_CIRCUMFERENCE_M)
/* ≈ 5 rev/s × 0.10996 m ≈ 0.55 m/s */

/** 最高速度对应的右轮 CPS（作为 APP_BALANCE_SPEED_CPS_SCALE 推算基准）。
 *  right_cps_max = MAX_RPM / 60 × RIGHT_CPR ≈ 170,000 */
#define ROBOT_MAX_RIGHT_CPS         ((uint32_t)(ROBOT_MAX_WHEEL_RPM / 60.0f * ROBOT_RIGHT_COUNTS_PER_OUTPUT_REV))

/* ========================================================================== */
/* §6  差速运动学                                                               */
/*                                                                             */
/* 标准差速驱动（Differential Drive）模型，以轴中心为参考点：                   */
/*   前向速度  v   (m/s)   : 正方向 = 车头方向                                 */
/*   角速度    ω   (rad/s) : 正方向 = 俯视逆时针（右转为负）                    */
/*   左轮线速  v_L (m/s)                                                        */
/*   右轮线速  v_R (m/s)                                                        */
/*                                                                             */
/*   正向推导：v_L = v - ω × L/2    v_R = v + ω × L/2                        */
/*   逆向推导：v   = (v_L + v_R)/2  ω   = (v_R - v_L) / L                    */
/*   其中 L = ROBOT_WHEELBASE_M                                                */
/* ========================================================================== */

/**
 * @brief 差速正向运动学：全局速度 → 左右轮速度（m/s）。
 *
 * @param v_mps       期望前向速度（m/s），正 = 前进
 * @param omega_rad_s 期望偏航角速度（rad/s），正 = 俯视逆时针（左转）
 * @param vl_mps      [out] 左轮线速（m/s）
 * @param vr_mps      [out] 右轮线速（m/s）
 */
static inline void robot_diff_drive_fwd(float v_mps, float omega_rad_s,
                                         float *vl_mps, float *vr_mps)
{
    float half_wb = ROBOT_WHEELBASE_M * 0.5f;
    *vl_mps = v_mps - omega_rad_s * half_wb;
    *vr_mps = v_mps + omega_rad_s * half_wb;
}

/**
 * @brief 差速逆运动学：左右轮速度（m/s）→ 全局速度。
 *
 * @param vl_mps      左轮线速（m/s）
 * @param vr_mps      右轮线速（m/s）
 * @param v_mps       [out] 前向速度（m/s）
 * @param omega_rad_s [out] 偏航角速度（rad/s）
 */
static inline void robot_diff_drive_inv(float vl_mps, float vr_mps,
                                         float *v_mps, float *omega_rad_s)
{
    *v_mps       = (vl_mps + vr_mps) * 0.5f;
    *omega_rad_s = (vr_mps - vl_mps) / ROBOT_WHEELBASE_M;
}

/**
 * @brief 差速正向运动学：全局速度 → 左右轮目标 CPS（整数）。
 *
 * 各轮使用自身 CPS/m/s 系数，正确处理 X4/X2 不对称。
 *
 * @param v_mps       期望前向速度（m/s）
 * @param omega_rad_s 期望偏航角速度（rad/s）
 * @param left_cps    [out] 左轮目标 CPS
 * @param right_cps   [out] 右轮目标 CPS
 */
static inline void robot_diff_drive_fwd_cps(float v_mps, float omega_rad_s,
                                              int32_t *left_cps, int32_t *right_cps)
{
    float vl, vr;
    robot_diff_drive_fwd(v_mps, omega_rad_s, &vl, &vr);
    *left_cps  = ROBOT_MPS_TO_LEFT_CPS(vl);
    *right_cps = ROBOT_MPS_TO_RIGHT_CPS(vr);
}

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_PARAM_H */
