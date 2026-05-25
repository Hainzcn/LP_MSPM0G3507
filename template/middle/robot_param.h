/**
 * @file    robot_param.h
 * @brief   车体几何参数与运动学换算（编译期常量 + inline 辅助）。
 *
 * 所有 mm 级物理量在此集中定义，其余模块通过 #include "robot_param.h" 引用，
 * 避免魔法数字散落各处。宏均带 #ifndef 包裹，可由工程级 -D 覆盖。
 *
 * 依赖：bsp_motor.h（BSP_MOTOR_LEFT/RIGHT_COUNTS_PER_OUTPUT_REV）
 *
 * ─────────────────────────────────────────────────────────────────────────
 * 上车标定步骤（实测后更新本文件对应宏即可固化）：
 *
 *   1. 轮径：用游标卡尺量轮外径（含橡胶），当前实测 35 mm。
 *   2. 轴距：量左右轮胎接地中心点间距，当前实测 185 mm。
 *   3. 编码器定标：清零编码器 → 手转输出轴精确一圈 → 读 count，
 *      确认与 BSP_MOTOR_LEFT/RIGHT_COUNTS_PER_OUTPUT_REV 一致。
 * ─────────────────────────────────────────────────────────────────────────
 */

#ifndef ROBOT_PARAM_H
#define ROBOT_PARAM_H

#include "bsp_motor.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/* 基础几何参数                                                                 */
/* ========================================================================== */

/** 轮胎外径（mm），含橡胶。 */
#ifndef ROBOT_WHEEL_DIAMETER_MM
#define ROBOT_WHEEL_DIAMETER_MM             (65)
#endif

/** 两轮接地中心间距（mm）。 */
#ifndef ROBOT_WHEEL_BASE_MM
#define ROBOT_WHEEL_BASE_MM                 (185)
#endif

/* ========================================================================== */
/* 衍生常量（避免运行时浮点；周长用 x1000 µm 表示）                             */
/* ========================================================================== */

/**
 * 轮周长 × 1000（单位 µm）。
 *   C = π × D  →  C_um_x1000 = 3141593 × D_mm / 1000
 * 35 mm → 109956 µm × 1000  (≈ 109.956 mm × 1000)
 *
 * 用于整数除法 cnt→mm 换算，避免运行时浮点。
 */
#define ROBOT_WHEEL_CIRCUMFERENCE_UM_X1000 \
    ((int32_t)((3141593LL * (int64_t)ROBOT_WHEEL_DIAMETER_MM) / 1000LL))

/** 轮周长浮点值（mm），供 inline 运动学函数使用。 */
#define ROBOT_WHEEL_CIRCUMFERENCE_MM_F \
    (3.14159265f * (float)ROBOT_WHEEL_DIAMETER_MM)

/**
 * 每 mm 行程对应的编码器计数 × 100（整数，避免浮点宏）。
 *   counts_per_mm = counts_per_rev / circumference_mm
 *   ×100 保留两位精度。
 *
 * 左轮：68000 / 109.956 ≈ 618.42 → ×100 = 61842
 * 右轮：34000 / 109.956 ≈ 309.21 → ×100 = 30921
 */
#define ROBOT_LEFT_COUNTS_PER_MM_X100 \
    ((int32_t)(((int64_t)BSP_MOTOR_LEFT_COUNTS_PER_OUTPUT_REV * 100LL * 1000LL) \
               / ROBOT_WHEEL_CIRCUMFERENCE_UM_X1000))

#define ROBOT_RIGHT_COUNTS_PER_MM_X100 \
    ((int32_t)(((int64_t)BSP_MOTOR_RIGHT_COUNTS_PER_OUTPUT_REV * 100LL * 1000LL) \
               / ROBOT_WHEEL_CIRCUMFERENCE_UM_X1000))

/** 左右平均 counts_per_mm × 100。 */
#define ROBOT_AVG_COUNTS_PER_MM_X100 \
    ((ROBOT_LEFT_COUNTS_PER_MM_X100 + ROBOT_RIGHT_COUNTS_PER_MM_X100) / 2)

/* ========================================================================== */
/* 编译期 sanity 检查                                                           */
/* ========================================================================== */

#if ROBOT_WHEEL_DIAMETER_MM <= 0
#error "ROBOT_WHEEL_DIAMETER_MM must be > 0"
#endif

#if ROBOT_WHEEL_BASE_MM <= 0
#error "ROBOT_WHEEL_BASE_MM must be > 0"
#endif

/* ========================================================================== */
/* 运动学 inline 辅助                                                           */
/* ========================================================================== */

/**
 * @brief 线速度 (mm/s) → 左右轮平均 counts/s。
 *
 *   avg_cps = v_mm_s × avg_counts_per_mm
 *           = v_mm_s × (AVG_COUNTS_PER_MM_X100 / 100)
 */
static inline int32_t robot_v_mm_s_to_avg_cps(int32_t v_mm_s)
{
    return (v_mm_s * ROBOT_AVG_COUNTS_PER_MM_X100) / 100;
}

/**
 * @brief 左右轮平均计数差值 → 行驶弧长 (mm)。
 *
 *   arc_mm = avg_counts / avg_counts_per_mm
 *          = avg_counts × 100 / AVG_COUNTS_PER_MM_X100
 */
static inline int32_t robot_arc_mm_from_avg_counts(int32_t avg_counts)
{
    if (ROBOT_AVG_COUNTS_PER_MM_X100 == 0) return 0;
    return (avg_counts * 100) / ROBOT_AVG_COUNTS_PER_MM_X100;
}

/**
 * @brief 给定圆弧半径 (mm)，计算所需角速度 omega (rad/s × 1000) 对应的
 *        左右轮差速 counts/s。
 *
 * 差分运动学：omega = (v_right - v_left) / wheel_base
 *   → delta_v_mm_s = omega × wheel_base
 *   → delta_cps = delta_v_mm_s × avg_counts_per_mm
 *
 * 此函数给定 omega_mrad_s = omega × 1000，返回 delta_cps = right_cps - left_cps。
 */
static inline int32_t robot_omega_mrad_to_delta_cps(int32_t omega_mrad_s)
{
    int32_t delta_v_mm_s_x1000 = omega_mrad_s * ROBOT_WHEEL_BASE_MM;
    return (int32_t)((int64_t)delta_v_mm_s_x1000 * ROBOT_AVG_COUNTS_PER_MM_X100
                     / (100LL * 1000LL));
}

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_PARAM_H */
