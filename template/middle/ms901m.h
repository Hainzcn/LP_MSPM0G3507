/**
 * @file    ms901m.h
 * @brief   ATK-MS901M 串口姿态传感器流式二进制协议解析（C 移植版）
 *
 *  从 docs/chore/Ms901mStreamParser.{cpp,h} 移植而来，关键差异：
 *    - 去掉 Qt（QByteArray / QList / QString），全部静态缓冲 + 标志位；
 *    - 用 float 替代 double（Cortex-M0+ 无 FPU，float 走 soft-float
 *      但比 double 短 2~3×，配合编译器 single-precision math 链路）；
 *    - 解析改为字节级状态机，免除 mid()/append() 缓冲压缩开销；
 *    - 不输出 19 元素 Snapshot 数组，改为字段化 `ms901m_snapshot_t`，
 *      与 app_telemetry / VOFA 通道映射强绑定；
 *    - 校验和算法保持一致：sum(0x55, 0x55, ID, LEN, DATA[*]) & 0xFF
 *      与 ID = 上一字节的下一字节比对。
 *
 *  帧结构：0x55 0x55 <ID> <LEN> <DATA[LEN]> <CHECKSUM>
 *
 *    ID 0x01: 姿态     LEN=6    roll/pitch/yaw       (int16 LE / 32768 * 180°)
 *    ID 0x02: 四元数   LEN=8    q0/q1/q2/q3          (int16 LE / 32768)
 *    ID 0x03: gyro+acc LEN=12   ax/ay/az/gx/gy/gz    (int16 LE 量纲见 .c)
 *    ID 0x04: mag+temp LEN=8    mx/my/mz / temp(/100)
 *    ID 0x05: baro+alt LEN=10   pressure(int32 Pa) / altitude(int32 / 100 m)
 *
 *  本工程业务上仅使用 0x01（pitch 主用）+ 0x03（gy 用作角速度）+ 0x04（温度），
 *  0x02/0x05 仍解析以备扩展，但不进 snapshot。
 */

#ifndef MS901M_H
#define MS901M_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  最近解析快照（字段化）。仅在主循环线程访问，无并发顾虑。
 *
 *  量纲选择（与 VOFA / 平衡环习惯对齐）：
 *    - 角度：度（°）
 *    - 角速度：度/秒（°/s）—— 直接喂 PD 速率项，无需 rad 转换
 *    - 加速度：g（重力加速度倍数）—— 静态时 ax² + ay² + az² ≈ 1
 *    - 温度：°C
 */
typedef struct {
    float pitch_deg;        /* 0x01 帧：俯仰角 (°)，平衡环主用 */
    float roll_deg;         /* 0x01 帧：横滚角 (°) */
    float yaw_deg;          /* 0x01 帧：偏航角 (°)，磁干扰时不可信 */

    float gx_dps;           /* 0x03 帧：陀螺 X (°/s) */
    float gy_dps;           /* 0x03 帧：陀螺 Y (°/s)，平衡环 pitch 速率 */
    float gz_dps;           /* 0x03 帧：陀螺 Z (°/s) */

    float ax_g;             /* 0x03 帧：加速度 X (g) */
    float ay_g;             /* 0x03 帧：加速度 Y (g) */
    float az_g;             /* 0x03 帧：加速度 Z (g) */

    float temp_c;           /* 0x04 帧：内温度 (°C) */

    bool  has_attitude;     /* 0x01 至少收到过一次 */
    bool  has_gyro_acc;     /* 0x03 至少收到过一次 */
    bool  has_mag_temp;     /* 0x04 至少收到过一次 */
} ms901m_snapshot_t;

/**
 * @brief  初始化 / 复位解析器状态机与所有最新帧标志。
 * @param  acc_fsr_g   加速度计满量程 (g)，与 MS901M 寄存器配置一致；典型 4。
 * @param  gyro_fsr_dps 陀螺仪满量程 (°/s)，典型 2000。
 *
 *  注：ATK 出厂默认 ±4 g / ±2000 dps（与 cpp 版默认一致）。如上位机被改过
 *  量程，需在此同步传入对应数值，否则单位换算系数错位。
 */
void ms901m_init(int16_t acc_fsr_g, int16_t gyro_fsr_dps);

/**
 * @brief  把 UART RX 字节流喂给状态机，按需更新内部最新快照。
 *         典型调用：主循环每 1 ms 调一次，单次 ≤ 64 B。
 */
void ms901m_feed_bytes(const uint8_t *p, size_t n);

/**
 * @brief  返回 0x01 帧是否至少收到过一次（即 pitch 是否就绪）。
 *         主控启动期 ms901m_has_attitude() == false 应视为 IMU 未在线。
 */
bool ms901m_has_attitude(void);

/**
 * @brief  把内部最新快照拷贝给上层（深拷贝，调用方持有副本可自由用）。
 */
void ms901m_get_snapshot(ms901m_snapshot_t *out);

/** 返回累计校验失败 / 长度异常的帧数（1 Hz 自测可监控误码率）。 */
uint32_t ms901m_bad_frames(void);

/** 返回累计成功解析的帧数。 */
uint32_t ms901m_good_frames(void);

#ifdef __cplusplus
}
#endif

#endif /* MS901M_H */
