/**
 * @file    k230_protocol.h
 * @brief   MCU <-> K230 定长帧通讯协议（Stage 4 IMU TX 一分二方案）。
 *
 * 帧格式：  0xAA 0x55 | LEN | CMD | PAYLOAD[LEN] | CRC16_LO | CRC16_HI | 0x55 0xAA
 *   - LEN：PAYLOAD 字节数（0~K230_PROTO_MAX_PAYLOAD）
 *   - CMD：帧类型标识
 *   - CRC16：CCITT（初始 0xFFFF），校验范围 = LEN + CMD + PAYLOAD
 *   - 尾部 0x55 0xAA 用于帧边界二次确认
 *
 * 通讯方向与帧类型：
 *
 *   MCU → K230（阻塞 TX，~240 B/s）：
 *     VEHICLE_STATUS  0x01   20 Hz   avg_cps(i32) + safety_state(u8) + bat_mv(u16)
 *     HEARTBEAT_MCU   0x02    1 Hz   uptime_ms(u32)
 *
 *   K230 → MCU（DMA RX）：
 *     MOTION_CMD      0x11  20~50 Hz  target_v(i16) + target_omega(i16) + mode(u8)
 *     HEARTBEAT_K230  0x12    1 Hz    uptime_ms(u32)
 *     PID_INJECT      0x13   按需     pid_id(u8) + kp(f32) + ki(f32) + kd(f32)
 */

#ifndef K230_PROTOCOL_H
#define K230_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ======================================================================== */
/* 帧格式常量                                                                */
/* ======================================================================== */

#define K230_PROTO_SYNC_HEAD_0      0xAAu
#define K230_PROTO_SYNC_HEAD_1      0x55u
#define K230_PROTO_SYNC_TAIL_0      0x55u
#define K230_PROTO_SYNC_TAIL_1      0xAAu

#define K230_PROTO_MAX_PAYLOAD      32u
/** 完整帧最大长度：2(head) + 1(len) + 1(cmd) + payload + 2(crc) + 2(tail) */
#define K230_PROTO_MAX_FRAME        (2u + 1u + 1u + K230_PROTO_MAX_PAYLOAD + 2u + 2u)
/** 帧开销（不含 payload 的固定字节数） */
#define K230_PROTO_OVERHEAD         8u

/* ======================================================================== */
/* CMD 定义                                                                   */
/* ======================================================================== */

/* MCU → K230 */
#define K230_CMD_VEHICLE_STATUS     0x01u
#define K230_CMD_HEARTBEAT_MCU      0x02u

/* K230 → MCU */
#define K230_CMD_MOTION_CMD         0x11u
#define K230_CMD_HEARTBEAT_K230     0x12u
#define K230_CMD_PID_INJECT         0x13u

/* ======================================================================== */
/* 业务 Payload 结构体                                                        */
/* ======================================================================== */

/** MCU→K230：车辆状态帧 (CMD 0x01)，20 Hz */
typedef struct __attribute__((packed)) {
    int32_t  avg_cps;           /* 整车前向速度（右轮等效 CPS）：
                                 *   = (left_cps/2 + right_cps) / 2
                                 * 直行时恒等于 right_cps；与物理线速度换算：
                                 *   v_mps = avg_cps / ROBOT_RIGHT_CPS_PER_MPS
                                 *         ≈ avg_cps / 309205 (轮径 35mm)
                                 * 正 = 前进，负 = 后退。 */
    uint8_t  safety_state;      /* app_safety 状态枚举 */
    uint16_t bat_mv;            /* 电池电压 mV */
} k230_vehicle_status_t;

/** MCU→K230 / K230→MCU：心跳帧 (CMD 0x02 / 0x12)，1 Hz */
typedef struct __attribute__((packed)) {
    uint32_t uptime_ms;
} k230_heartbeat_t;

/** K230→MCU：运动指令帧 (CMD 0x11)，20~50 Hz */
typedef struct __attribute__((packed)) {
    int16_t  target_v;          /* 期望前向速度（右轮等效 CPS）：
                                 *   与 k230_vehicle_status_t.avg_cps 单位相同。
                                 *   app_balance 直接写入 motion_cmd.target_speed_cps；
                                 *   换算为 m/s：v = target_v / ROBOT_RIGHT_CPS_PER_MPS
                                 *   典型范围：[-17000, 17000]（对应 ±0.55 m/s）
                                 * 若需以 m/s 发送：target_v = (int16_t)(v_mps × 309205)
                                 *   但需注意 int16_t 量程（±32767），高速时溢出风险。
                                 * TODO(Stage N): 迁移为 mm/s（int16_t 范围 ±550mm/s 足够）。*/
    int16_t  target_omega;      /* 期望偏航角速度差分量（permille）；
                                 *   正 = 顺时针（俯视），对应右轮快 / 左轮慢。
                                 *   app_balance 直接叠加到差速分配中。 */
    uint8_t  mode;              /* 运动模式：0=停, 1=直行, 2=转弯, ... */
} k230_motion_cmd_t;

/** K230→MCU：远程 PID 注入帧 (CMD 0x13)，按需 */
typedef struct __attribute__((packed)) {
    uint8_t pid_id;             /* PID 环 ID：0=angle, 2=speed, 3=yaw */
    float   kp;
    float   ki;
    float   kd;
} k230_pid_inject_t;

/* ======================================================================== */
/* 帧解析器（字节级状态机）                                                    */
/* ======================================================================== */

typedef enum {
    K230_RX_WAIT_HEAD0 = 0,
    K230_RX_WAIT_HEAD1,
    K230_RX_WAIT_LEN,
    K230_RX_WAIT_CMD,
    K230_RX_WAIT_PAYLOAD,
    K230_RX_WAIT_CRC_LO,
    K230_RX_WAIT_CRC_HI,
    K230_RX_WAIT_TAIL0,
    K230_RX_WAIT_TAIL1
} k230_rx_state_t;

typedef struct {
    k230_rx_state_t state;
    uint8_t  len;               /* 当前帧 PAYLOAD 长度 */
    uint8_t  cmd;               /* 当前帧 CMD */
    uint8_t  payload[K230_PROTO_MAX_PAYLOAD];
    uint8_t  payload_idx;
    uint16_t crc_recv;          /* 接收到的 CRC16 */
    uint16_t crc_calc;          /* 计算中的 CRC16 */
    uint32_t good_frames;
    uint32_t bad_frames;
} k230_parser_t;

/* ======================================================================== */
/* API                                                                       */
/* ======================================================================== */

/** CRC16-CCITT (poly 0x1021, init 0xFFFF)。 */
uint16_t k230_crc16(const uint8_t *data, size_t len);

/**
 * @brief 编码一帧完整数据到 out 缓冲。
 * @param cmd       帧类型
 * @param payload   PAYLOAD 数据指针（可为 NULL 当 payload_len==0）
 * @param payload_len PAYLOAD 长度
 * @param out       输出缓冲（至少 payload_len + K230_PROTO_OVERHEAD 字节）
 * @param out_cap   输出缓冲容量
 * @return 实际帧长度；0 = 失败（容量不足或 payload 过长）
 */
size_t k230_encode_frame(uint8_t cmd,
                         const void *payload, uint8_t payload_len,
                         uint8_t *out, size_t out_cap);

/** 初始化 / 复位解析器。 */
void k230_parser_init(k230_parser_t *p);

/**
 * @brief 喂一个字节给解析器状态机。
 * @return true = 完成一帧（cmd / payload / len 可读）；false = 未完成或校验失败。
 */
bool k230_parser_feed(k230_parser_t *p, uint8_t byte);

#ifdef __cplusplus
}
#endif

#endif /* K230_PROTOCOL_H */
