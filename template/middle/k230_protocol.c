/**
 * @file    k230_protocol.c
 * @brief   MCU <-> K230 帧协议实现。
 *
 * CRC16-CCITT 按位计算（poly 0x1021, init 0xFFFF），校验范围 = LEN + CMD + PAYLOAD。
 * 编码端：填 head → len → cmd → payload → crc → tail。
 * 解码端：逐字节状态机，完成一帧后由调用方读 cmd/payload/len。
 */

#include "k230_protocol.h"
#include <string.h>

/* ======================================================================== */
/* CRC16-CCITT (poly 0x1021, init 0xFFFF) 按位计算                            */
/*                                                                            */
/* 之前的查表实现存在 50 处错误，导致 HEARTBEAT_MCU 等帧 CRC 与 K230 侧不一致。 */
/* 对于本工程 <1 kB/s 的吞吐量，按位计算在 80 MHz M0+ 上开销可忽略不计。       */
/* ======================================================================== */

uint16_t k230_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFu;
    size_t   i;
    int      b;
    for (i = 0u; i < len; ++i) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8u);
        for (b = 0; b < 8; b++) {
            if (crc & 0x8000u) {
                crc = (uint16_t)((crc << 1u) ^ 0x1021u);
            } else {
                crc = (uint16_t)(crc << 1u);
            }
        }
    }
    return crc;
}

/* ======================================================================== */
/* 编码                                                                       */
/* ======================================================================== */

size_t k230_encode_frame(uint8_t cmd,
                         const void *payload, uint8_t payload_len,
                         uint8_t *out, size_t out_cap)
{
    if (payload_len > K230_PROTO_MAX_PAYLOAD) {
        return 0u;
    }
    size_t total = (size_t)payload_len + K230_PROTO_OVERHEAD;
    if (out == NULL || out_cap < total) {
        return 0u;
    }

    /* head */
    out[0] = K230_PROTO_SYNC_HEAD_0;
    out[1] = K230_PROTO_SYNC_HEAD_1;

    /* len + cmd */
    out[2] = payload_len;
    out[3] = cmd;

    /* payload */
    if (payload_len > 0u && payload != NULL) {
        memcpy(&out[4], payload, payload_len);
    }

    /* CRC16 over (len + cmd + payload) */
    uint16_t crc = k230_crc16(&out[2], (size_t)2u + payload_len);
    out[4u + payload_len]      = (uint8_t)(crc & 0xFFu);
    out[4u + payload_len + 1u] = (uint8_t)(crc >> 8);

    /* tail */
    out[4u + payload_len + 2u] = K230_PROTO_SYNC_TAIL_0;
    out[4u + payload_len + 3u] = K230_PROTO_SYNC_TAIL_1;

    return total;
}

/* ======================================================================== */
/* 解析器                                                                     */
/* ======================================================================== */

void k230_parser_init(k230_parser_t *p)
{
    if (p == NULL) return;
    p->state       = K230_RX_WAIT_HEAD0;
    p->len         = 0u;
    p->cmd         = 0u;
    p->payload_idx = 0u;
    p->crc_recv    = 0u;
    p->crc_calc    = 0u;
    p->good_frames = 0u;
    p->bad_frames  = 0u;
}

bool k230_parser_feed(k230_parser_t *p, uint8_t byte)
{
    if (p == NULL) return false;

    switch (p->state) {
    case K230_RX_WAIT_HEAD0:
        if (byte == K230_PROTO_SYNC_HEAD_0) {
            p->state = K230_RX_WAIT_HEAD1;
        }
        break;

    case K230_RX_WAIT_HEAD1:
        if (byte == K230_PROTO_SYNC_HEAD_1) {
            p->state = K230_RX_WAIT_LEN;
        } else if (byte == K230_PROTO_SYNC_HEAD_0) {
            /* stay: consecutive 0xAA */
        } else {
            p->state = K230_RX_WAIT_HEAD0;
        }
        break;

    case K230_RX_WAIT_LEN:
        if (byte > K230_PROTO_MAX_PAYLOAD) {
            p->bad_frames++;
            p->state = K230_RX_WAIT_HEAD0;
        } else {
            p->len = byte;
            p->payload_idx = 0u;
            p->state = K230_RX_WAIT_CMD;
        }
        break;

    case K230_RX_WAIT_CMD:
        p->cmd = byte;
        if (p->len == 0u) {
            p->state = K230_RX_WAIT_CRC_LO;
        } else {
            p->state = K230_RX_WAIT_PAYLOAD;
        }
        break;

    case K230_RX_WAIT_PAYLOAD:
        p->payload[p->payload_idx++] = byte;
        if (p->payload_idx >= p->len) {
            p->state = K230_RX_WAIT_CRC_LO;
        }
        break;

    case K230_RX_WAIT_CRC_LO:
        p->crc_recv = byte;
        p->state = K230_RX_WAIT_CRC_HI;
        break;

    case K230_RX_WAIT_CRC_HI:
        p->crc_recv |= ((uint16_t)byte << 8);
        p->state = K230_RX_WAIT_TAIL0;
        break;

    case K230_RX_WAIT_TAIL0:
        if (byte == K230_PROTO_SYNC_TAIL_0) {
            p->state = K230_RX_WAIT_TAIL1;
        } else {
            p->bad_frames++;
            p->state = K230_RX_WAIT_HEAD0;
        }
        break;

    case K230_RX_WAIT_TAIL1:
        p->state = K230_RX_WAIT_HEAD0;
        if (byte != K230_PROTO_SYNC_TAIL_1) {
            p->bad_frames++;
            return false;
        }
        /* verify CRC: build temp buffer [len, cmd, payload...] */
        {
            uint8_t crc_buf[2u + K230_PROTO_MAX_PAYLOAD];
            crc_buf[0] = p->len;
            crc_buf[1] = p->cmd;
            if (p->len > 0u) {
                memcpy(&crc_buf[2], p->payload, p->len);
            }
            uint16_t calc = k230_crc16(crc_buf, (size_t)2u + p->len);
            if (calc != p->crc_recv) {
                p->bad_frames++;
                return false;
            }
        }
        p->good_frames++;
        return true;

    default:
        p->state = K230_RX_WAIT_HEAD0;
        break;
    }

    return false;
}
