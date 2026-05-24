/**
 * @file    bsp_k230_uart.c
 * @brief   UART1 中断 RX + 阻塞 TX 实现（K230 通讯链路）。
 *
 * 原 DMA RX 方案因 SysConfig 通道号/宽度/地址递增配置错误导致 MCU 收不到
 * K230 字节。改为 UART RX FIFO 半满中断驱动，与 bsp_imu_uart.c 相同模式，
 * 简单可靠；K230 链路吞吐仅 ~420 B/s，中断开销可忽略。
 */

#include "bsp_k230_uart.h"
#include "ti_msp_dl_config.h"

/* 应用层环形缓冲，2 的幂 */
#define K230_APP_BUF_SIZE    512u
#define K230_APP_BUF_MASK    (K230_APP_BUF_SIZE - 1u)

static volatile uint8_t  s_app_buf[K230_APP_BUF_SIZE];
static volatile uint16_t s_app_head = 0u;   /* ISR 写 */
static volatile uint16_t s_app_tail = 0u;   /* 主循环读 */
static volatile uint32_t s_total_rx = 0u;
static volatile uint32_t s_overrun  = 0u;

/* ------------------------------------------------------------------ */
/* 初始化                                                              */
/* ------------------------------------------------------------------ */

void bsp_k230_uart_init(void)
{
    s_app_head = 0u;
    s_app_tail = 0u;
    s_total_rx = 0u;
    s_overrun  = 0u;

    /* SysConfig 开启了 DMA Receive Event，这里关闭它避免干扰 */
    DL_UART_Main_disableDMAReceiveEvent(UART_K230_INST, DL_UART_DMA_INTERRUPT_RX);

    /* 启用 UART1 RX FIFO 半满中断 */
    DL_UART_Main_enableInterrupt(UART_K230_INST, DL_UART_MAIN_INTERRUPT_RX);

    /* NVIC 使能 UART1 中断（SysConfig 不会自动做这一步） */
    NVIC_EnableIRQ(UART_K230_INST_INT_IRQN);
}

/* ------------------------------------------------------------------ */
/* UART1 中断服务函数                                                   */
/* ------------------------------------------------------------------ */

void UART1_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_K230_INST)) {
    case DL_UART_MAIN_IIDX_RX: {
        while (DL_UART_Main_isRXFIFOEmpty(UART_K230_INST) == false) {
            uint8_t b = (uint8_t)DL_UART_Main_receiveData(UART_K230_INST);
            uint16_t next = (uint16_t)((s_app_head + 1u) & K230_APP_BUF_MASK);
            if (next == s_app_tail) {
                s_overrun++;
            } else {
                s_app_buf[s_app_head] = b;
                s_app_head = next;
                s_total_rx++;
            }
        }
        break;
    }
    default:
        break;
    }
}

/* ------------------------------------------------------------------ */
/* 应用层 API                                                          */
/* ------------------------------------------------------------------ */

size_t bsp_k230_uart_rx_pop_bulk(uint8_t *dst, size_t max_len)
{
    if (dst == NULL || max_len == 0u) {
        return 0u;
    }
    size_t got = 0u;
    while (got < max_len) {
        uint16_t h = s_app_head;
        uint16_t t = s_app_tail;
        if (h == t) break;
        dst[got++] = s_app_buf[t];
        s_app_tail = (uint16_t)((t + 1u) & K230_APP_BUF_MASK);
    }
    return got;
}

size_t bsp_k230_uart_rx_available(void)
{
    uint16_t h = s_app_head;
    uint16_t t = s_app_tail;
    return (size_t)((h - t) & K230_APP_BUF_MASK);
}

uint32_t bsp_k230_uart_rx_overrun(void)
{
    return s_overrun;
}

uint32_t bsp_k230_uart_total_rx(void)
{
    return s_total_rx;
}

void bsp_k230_uart_write_blocking(const uint8_t *data, size_t len)
{
    if (data == NULL) return;
    for (size_t i = 0u; i < len; ++i) {
        while (DL_UART_Main_isBusy(UART_K230_INST)) {
            ;
        }
        DL_UART_Main_transmitDataBlocking(UART_K230_INST, data[i]);
    }
}
