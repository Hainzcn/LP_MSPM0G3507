/**
 * @file    vofa.c
 * @brief   VOFA+ JustFloat 打包实现，详见 vofa.h。
 */

#include "vofa.h"
#include <string.h>

#define VOFA_MAX_CHANNELS  32u

static const uint8_t JUSTFLOAT_TAIL[4] = { 0x00u, 0x00u, 0x80u, 0x7Fu };
static vofa_writer_fn s_writer = NULL;

void vofa_set_writer(vofa_writer_fn writer)
{
    s_writer = writer;
}

void vofa_send(const float *ch, uint8_t n)
{
    if (s_writer == NULL || ch == NULL) { return; }
    if (n == 0u || n > VOFA_MAX_CHANNELS) { return; }

    /* 一次性把 N×4 + 4 个尾字节凑成单缓冲，减少底层写调用次数 */
    uint8_t buf[VOFA_MAX_CHANNELS * 4u + 4u];
    size_t  payload_bytes = (size_t)n * sizeof(float);
    memcpy(buf, ch, payload_bytes);
    memcpy(&buf[payload_bytes], JUSTFLOAT_TAIL, 4u);

    s_writer(buf, payload_bytes + 4u);
}
