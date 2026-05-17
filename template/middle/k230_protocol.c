/**
 * @file    k230_protocol.c
 * @brief   MCU <-> K230 帧协议实现。
 *
 * CRC16-CCITT 查表法（256 B ROM），校验范围 = LEN + CMD + PAYLOAD。
 * 编码端：填 head → len → cmd → payload → crc → tail。
 * 解码端：逐字节状态机，完成一帧后由调用方读 cmd/payload/len。
 */

#include "k230_protocol.h"
#include <string.h>

/* ======================================================================== */
/* CRC16-CCITT (0x1021, init 0xFFFF) 查表                                     */
/* ======================================================================== */

static const uint16_t s_crc16_table[256] = {
    0x0000,0x1021,0x2042,0x3063,0x4084,0x50A5,0x6096,0x70B7,
    0x8108,0x9129,0xA14A,0xB16B,0xC18C,0xD1AD,0xE1CE,0xF1EF,
    0x1231,0x0210,0x3273,0x2252,0x52B5,0x4294,0x72F7,0x62D6,
    0x9339,0x8318,0xB37B,0xA35A,0xD3BD,0xC39C,0xF3FF,0xE3DE,
    0x2462,0x3443,0x0420,0x1401,0x64E6,0x74C7,0x44A4,0x54A5,
    0xA54A,0xB56B,0x8508,0x9529,0xE5CE,0xF5EF,0xC58C,0xD5AD,
    0x3653,0x2672,0x1611,0x0630,0x76D7,0x66F6,0x5695,0x46B4,
    0xB75B,0xA77A,0x9719,0x8738,0xF7DF,0xE7FE,0xD79D,0xC7BC,
    0x4864,0x5845,0x6826,0x7807,0x08E0,0x18C1,0x28A2,0x38A3,
    0xC94C,0xD96D,0xE90E,0xF92F,0x89C8,0x99E9,0xA98A,0xB9AB,
    0x5A55,0x4A74,0x7A17,0x6A36,0x1AD1,0x0AF0,0x3A93,0x2AB2,
    0xDB5D,0xCB7C,0xFB1F,0xEB3E,0x9BD9,0x8BF8,0xBB9B,0xAB9A,
    0x6CA6,0x7C87,0x4CE4,0x5CC5,0x2C22,0x3C03,0x0C60,0x1C41,
    0xEDAE,0xFD8F,0xCDEC,0xDDCD,0xAD2A,0xBD0B,0x8D68,0x9D49,
    0x7E97,0x6EB6,0x5ED5,0x4EF4,0x3E13,0x2E32,0x1E51,0x0E70,
    0xFF9F,0xEFBE,0xDFDD,0xCFFC,0xBF1B,0xAF3A,0x9F59,0x8F78,
    0x9188,0x81A9,0xB1CA,0xA1EB,0xD10C,0xC12D,0xF14E,0xE16F,
    0x1080,0x00A1,0x30C2,0x20E3,0x5004,0x4025,0x7046,0x6067,
    0x83B9,0x9398,0xA3FB,0xB3DA,0xC33D,0xD31C,0xE37F,0xF35E,
    0x02B1,0x1290,0x22F3,0x32D2,0x4235,0x5214,0x6277,0x7256,
    0xB5EA,0xA5CB,0x95A8,0x8589,0xF56E,0xE54F,0xD52C,0xC50D,
    0x34E2,0x24C3,0x14A0,0x0481,0x7466,0x6447,0x5424,0x4405,
    0xA7DB,0xB7FA,0x8799,0x97B8,0xE75F,0xF77E,0xC71D,0xD73C,
    0x26D3,0x36F2,0x0691,0x16B0,0x6657,0x7676,0x4615,0x5634,
    0xD94C,0xC96D,0xF90E,0xE92F,0x99C8,0x89E9,0xB98A,0xA9AB,
    0x5844,0x4865,0x7806,0x6827,0x18C0,0x08E1,0x3882,0x28A3,
    0xCB7D,0xDB5C,0xEB3F,0xFB1E,0x8BD9,0x9BF8,0xAB9B,0xBB9A,
    0x4A75,0x5A54,0x6A37,0x7A16,0x0AD1,0x1AF0,0x2A93,0x3AB2,
    0xFD2E,0xED0F,0xDD6C,0xCD4D,0xBDAA,0xAD8B,0x9DE8,0x8DC9,
    0x7C26,0x6C07,0x5C64,0x4C45,0x3CA2,0x2C83,0x1CE0,0x0CC1,
    0xEF1F,0xFF3E,0xCF5D,0xDF7C,0xAF9B,0xBFBA,0x8FD9,0x9FF8,
    0x6E17,0x7E36,0x4E55,0x5E74,0x2E93,0x3EB2,0x0ED1,0x1EF0,
};

uint16_t k230_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFu;
    for (size_t i = 0u; i < len; ++i) {
        crc = (uint16_t)((crc << 8) ^ s_crc16_table[(crc >> 8) ^ data[i]]);
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
