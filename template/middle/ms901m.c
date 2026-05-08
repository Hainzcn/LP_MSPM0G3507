/**
 * @file    ms901m.c
 * @brief   MS901M 字节级状态机解析器实现，详见 ms901m.h。
 *
 * 状态机：
 *   SYNC1 -> SYNC2 -> ID -> LEN -> DATA(*LEN) -> CHECKSUM -> SYNC1 ...
 *
 * 校验：边收边累加 (sum(已收的 sync/ID/LEN/DATA) & 0xFF) == checksum byte
 *       与 cpp 版 sum(buf[0..total-2]) 等价（buf 区间 = 上述 4+LEN 字节）。
 *
 * 单线程使用：所有 static 状态仅由 ms901m_feed_bytes / ms901m_get_snapshot
 * 在 app_telemetry 主循环线程访问，UART RX ISR 只往环形缓冲写、不调本模块。
 */

#include "ms901m.h"

/* MS901M 各帧 LEN：0x01=6 0x02=8 0x03=12 0x04=8 0x05=10。
 * 设 32 B 上限给将来扩展（cpp 版用 64，这里折半够用）。 */
#define MS901M_DATA_MAX  32u

#define SYNC_BYTE        0x55u

typedef enum {
    ST_SYNC1 = 0,
    ST_SYNC2,
    ST_ID,
    ST_LEN,
    ST_DATA,
    ST_CHECKSUM
} parse_state_t;

/* 解析状态机 */
static parse_state_t s_state    = ST_SYNC1;
static uint8_t       s_id       = 0u;
static uint8_t       s_len      = 0u;
static uint8_t       s_data[MS901M_DATA_MAX];
static uint8_t       s_data_idx = 0u;
static uint8_t       s_chk      = 0u;   /* 累加校验和 */

/* 量纲转换系数 */
static float s_acc_scale  = 4.0f / 32768.0f;     /* default ±4 g */
static float s_gyro_scale = 2000.0f / 32768.0f;  /* default ±2000 dps */

/* 最新快照 + 统计 */
static ms901m_snapshot_t s_snap = { 0 };
static uint32_t          s_bad_frames  = 0u;
static uint32_t          s_good_frames = 0u;

/* ---------- 小工具 ------------------------------------------------------ */

static inline int16_t le16(uint8_t lo, uint8_t hi)
{
    return (int16_t)(((uint16_t)hi << 8) | (uint16_t)lo);
}

static void reset_state_machine(void)
{
    s_state    = ST_SYNC1;
    s_chk      = 0u;
    s_data_idx = 0u;
}

/* ---------- 各帧解析（in：4 字节 sync/id/len 已校验对齐） --------------- */

static void parse_attitude(const uint8_t *d, uint8_t len)
{
    if (len != 6u) { return; }
    /* 12-bit Q15-like：int16 / 32768 * 180° */
    s_snap.roll_deg  = (float)le16(d[0], d[1]) * (180.0f / 32768.0f);
    s_snap.pitch_deg = (float)le16(d[2], d[3]) * (180.0f / 32768.0f);
    s_snap.yaw_deg   = (float)le16(d[4], d[5]) * (180.0f / 32768.0f);
    s_snap.has_attitude = true;
}

static void parse_gyro_acc(const uint8_t *d, uint8_t len)
{
    if (len != 12u) { return; }
    /* 加速度：int16 / 32768 * acc_fsr (g)；这里取 g 量纲方便阅读 */
    s_snap.ax_g = (float)le16(d[0], d[1]) * s_acc_scale;
    s_snap.ay_g = (float)le16(d[2], d[3]) * s_acc_scale;
    s_snap.az_g = (float)le16(d[4], d[5]) * s_acc_scale;
    /* 陀螺：int16 / 32768 * gyro_fsr (°/s) */
    s_snap.gx_dps = (float)le16(d[6],  d[7])  * s_gyro_scale;
    s_snap.gy_dps = (float)le16(d[8],  d[9])  * s_gyro_scale;
    s_snap.gz_dps = (float)le16(d[10], d[11]) * s_gyro_scale;
    s_snap.has_gyro_acc = true;
}

static void parse_mag_temp(const uint8_t *d, uint8_t len)
{
    if (len != 8u) { return; }
    /* 磁力计原始值不进 snapshot（本工程不用磁力计，避免被 PWM 干扰污染输出） */
    /* 温度：int16 / 100 → °C */
    s_snap.temp_c = (float)le16(d[6], d[7]) * 0.01f;
    s_snap.has_mag_temp = true;
}

/* 0x02 四元数 / 0x05 气压 不进 snapshot；保留校验路径以避免被当成 bad_frame */
static void parse_quaternion(const uint8_t *d, uint8_t len)
{
    (void)d;
    if (len != 8u) { return; }
}

static void parse_baro_alt(const uint8_t *d, uint8_t len)
{
    (void)d;
    if (len != 10u) { return; }
}

static void dispatch_frame(uint8_t id, const uint8_t *d, uint8_t len)
{
    switch (id) {
        case 0x01: parse_attitude(d, len);   break;
        case 0x02: parse_quaternion(d, len); break;
        case 0x03: parse_gyro_acc(d, len);   break;
        case 0x04: parse_mag_temp(d, len);   break;
        case 0x05: parse_baro_alt(d, len);   break;
        default: /* 未知 ID 不算 bad_frame，可能是 ATK 后续固件扩展 */ break;
    }
    s_good_frames++;
}

/* ---------- 公开 API ---------------------------------------------------- */

void ms901m_init(int16_t acc_fsr_g, int16_t gyro_fsr_dps)
{
    if (acc_fsr_g  > 0) { s_acc_scale  = (float)acc_fsr_g  / 32768.0f; }
    if (gyro_fsr_dps > 0) { s_gyro_scale = (float)gyro_fsr_dps / 32768.0f; }

    reset_state_machine();

    /* 清快照与计数 */
    s_snap.pitch_deg = 0.0f;
    s_snap.roll_deg  = 0.0f;
    s_snap.yaw_deg   = 0.0f;
    s_snap.gx_dps = s_snap.gy_dps = s_snap.gz_dps = 0.0f;
    s_snap.ax_g   = s_snap.ay_g   = s_snap.az_g   = 0.0f;
    s_snap.temp_c = 0.0f;
    s_snap.has_attitude = false;
    s_snap.has_gyro_acc = false;
    s_snap.has_mag_temp = false;

    s_bad_frames  = 0u;
    s_good_frames = 0u;
}

void ms901m_feed_bytes(const uint8_t *p, size_t n)
{
    if (p == NULL) { return; }

    for (size_t i = 0u; i < n; ++i) {
        uint8_t b = p[i];

        switch (s_state) {

        case ST_SYNC1:
            if (b == SYNC_BYTE) {
                s_chk   = b;
                s_state = ST_SYNC2;
            }
            /* 非同步字节直接丢，状态保持 */
            break;

        case ST_SYNC2:
            if (b == SYNC_BYTE) {
                s_chk  += b;
                s_state = ST_ID;
            } else {
                /* 0x55 之后非 0x55：从这一字节起重新找 sync。
                 * 注意：当前字节本身可能是 0x55（理论上前一个 if 已捕获），
                 * 所以这里保留 ST_SYNC1，让下一字节继续找 sync1 */
                reset_state_machine();
            }
            break;

        case ST_ID:
            s_id    = b;
            s_chk  += b;
            s_state = ST_LEN;
            break;

        case ST_LEN:
            if (b > MS901M_DATA_MAX) {
                /* 长度异常：丢弃，重置；当前字节无法判断是不是 sync，
                 * 简单起见直接回 SYNC1 让下一拍重新对齐 */
                s_bad_frames++;
                reset_state_machine();
                break;
            }
            s_len      = b;
            s_chk     += b;
            s_data_idx = 0u;
            s_state    = (s_len == 0u) ? ST_CHECKSUM : ST_DATA;
            break;

        case ST_DATA:
            s_data[s_data_idx++] = b;
            s_chk += b;
            if (s_data_idx >= s_len) {
                s_state = ST_CHECKSUM;
            }
            break;

        case ST_CHECKSUM:
            if (b == s_chk) {
                dispatch_frame(s_id, s_data, s_len);
            } else {
                s_bad_frames++;
            }
            reset_state_machine();
            break;

        default:
            reset_state_machine();
            break;
        }
    }
}

bool ms901m_has_attitude(void)
{
    return s_snap.has_attitude;
}

void ms901m_get_snapshot(ms901m_snapshot_t *out)
{
    if (out == NULL) { return; }
    *out = s_snap;
}

uint32_t ms901m_bad_frames(void)
{
    return s_bad_frames;
}

uint32_t ms901m_good_frames(void)
{
    return s_good_frames;
}
