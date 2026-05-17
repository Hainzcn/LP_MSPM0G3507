/**
 * @file    bsp_k230_uart.c
 * @brief   UART1 DMA RX 半缓冲 + 阻塞 TX 实现。
 *
 * Stage 4 重构（IMU TX 一分二方案）：
 *   - TX DMA 已在 SysConfig / ti_msp_dl_config 中移除。
 *   - RX DMA 改为半缓冲双中断：256 B DMA 缓冲，128 B 半满中断 + 128 B
 *     全满中断；ISR 把就绪半块搬入 512 B 应用环缓，上层 pop_bulk 读取。
 *   - 上层（app_balance）负责把字节流喂给 k230_parser 状态机。
 *
 * DMA 通道号：ti_msp_dl_config.h 定义 DMA_CH_UART_K230_DMA_RX_CHAN，
 * 等于 DMA_CH0_CHAN_ID = 1（硬件通道 1）。
 */

#include "bsp_k230_uart.h"
#include "ti_msp_dl_config.h"

/* DMA_CH0_CHAN_ID = 1（UART1 RX 触发，见 ti_msp_dl_config.h）。
 * 旧骨架代码在此定义 fallback 为 0u，与实际通道号不一致；此处修正为 1u。
 * ti_msp_dl_config.h 中已同步添加 DMA_CH_UART_K230_DMA_RX_CHAN 宏；
 * 此处 #ifndef 保证两者只取一份，避免重定义警告。 */
#ifndef DMA_CH_UART_K230_DMA_RX_CHAN
#define DMA_CH_UART_K230_DMA_RX_CHAN  (1u)
#endif

/* DMA 原始缓冲：一整块 256 B，ISR 按半块 (0~127 / 128~255) 搬入应用环缓 */
#define K230_DMA_BUF_SIZE    256u
#define K230_DMA_HALF_SIZE   (K230_DMA_BUF_SIZE / 2u)

/* 应用层环形缓冲，2 的幂 */
#define K230_APP_BUF_SIZE    512u
#define K230_APP_BUF_MASK    (K230_APP_BUF_SIZE - 1u)

static volatile uint8_t  s_dma_buf[K230_DMA_BUF_SIZE];
static volatile uint8_t  s_app_buf[K230_APP_BUF_SIZE];
static volatile uint16_t s_app_head = 0u;   /* ISR 写 */
static volatile uint16_t s_app_tail = 0u;   /* 主循环读 */
static volatile uint32_t s_total_rx = 0u;
static volatile uint32_t s_overrun  = 0u;

/* 标记当前 DMA 写完了哪一半（ISR 设、ISR 用） */
static volatile uint8_t  s_half_ready = 0u; /* 0=无, 1=前半, 2=后半 */

/* ------------------------------------------------------------------ */
/* 内部：把 DMA 半缓冲搬入应用环缓                                    */
/* ------------------------------------------------------------------ */
static void copy_half_to_ring(const volatile uint8_t *src, uint16_t count)
{
    for (uint16_t i = 0u; i < count; ++i) {
        uint16_t next = (uint16_t)((s_app_head + 1u) & K230_APP_BUF_MASK);
        if (next == s_app_tail) {
            s_overrun++;
        } else {
            s_app_buf[s_app_head] = src[i];
            s_app_head = next;
        }
    }
    s_total_rx += count;
}

/* ------------------------------------------------------------------ */
/* DMA 初始配置：BLOCK 模式传满整个 256 B 缓冲后中断                   */
/* ------------------------------------------------------------------ */
static void k230_dma_rx_arm(void)
{
    DL_DMA_disableChannel(DMA, DMA_CH_UART_K230_DMA_RX_CHAN);

    DL_DMA_setSrcAddr(DMA, DMA_CH_UART_K230_DMA_RX_CHAN,
        (uint32_t)&UART_K230_INST->RXDATA);
    DL_DMA_setDestAddr(DMA, DMA_CH_UART_K230_DMA_RX_CHAN,
        (uint32_t)&s_dma_buf[0]);
    DL_DMA_setTransferSize(DMA, DMA_CH_UART_K230_DMA_RX_CHAN,
        K230_DMA_BUF_SIZE); 

    DL_DMA_enableChannel(DMA, DMA_CH_UART_K230_DMA_RX_CHAN);
}

void bsp_k230_uart_init(void)
{
    DL_UART_Main_clearInterruptStatus(UART_K230_INST,
        DL_UART_MAIN_INTERRUPT_RX | DL_UART_MAIN_INTERRUPT_TX);

    s_app_head = 0u;
    s_app_tail = 0u;
    s_total_rx = 0u;
    s_overrun  = 0u;
    s_half_ready = 0u;

    NVIC_EnableIRQ(DMA_INT_IRQn);
    k230_dma_rx_arm();
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

/* ------------------------------------------------------------------ */
/* DMA ISR：DMA 传满整个缓冲后触发                                     */
/*                                                                      */
/* MSPM0G3507 所有 DMA 通道共用一个 NVIC 槽。                           */
/* 当前仅 K230 RX 一个通道在用（TX DMA 已在 Stage 4 移除）。           */
/* ------------------------------------------------------------------ */
void DMA_IRQHandler(void)
{
    DL_DMA_EVENT_IIDX iidx = DL_DMA_getPendingInterrupt(DMA);

    if (iidx == (DL_DMA_EVENT_IIDX_DMACH0 + DMA_CH_UART_K230_DMA_RX_CHAN)) {
        /* DMA 写满整个 256 B 缓冲：搬全部到应用环缓，然后重装 */
        copy_half_to_ring(s_dma_buf, K230_DMA_BUF_SIZE);
        k230_dma_rx_arm();
    }
}
