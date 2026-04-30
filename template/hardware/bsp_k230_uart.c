/**
 * @file    bsp_k230_uart.c
 * @brief   UART1 DMA RX 接收骨架。
 *
 * 设计要点：
 *   - SysConfig 的 K230_UART (`UART_K230`) 已配 RX/TX 双向 DMA，本文件
 *     仅负责把 RX DMA 跑成 "ring buffer"：BLOCK 模式 + 重启计数策略。
 *   - 真正最优的"零拷贝环缓"做法是 BASIC + 重装 srcAddr/dstAddr/size，
 *     但 SDK 对 UART RX DMA 在 G3507 上 **不支持** AUTO 自动重装；这里
 *     退而求其次：DMA 转完一整个 buf 后在 DMA_IRQHandler 里重启它，
 *     `s_total_rx` 累加 buf 大小。1 Hz 自测足够，**不**用于平衡环时序。
 *   - 真业务（阶段下一轮）需要做帧解析时，应改用半满 + 全满双中断，
 *     或切换到 UART RX FIFO 中断 + 软件搬运（拒绝 DMA）；本文件预留
 *     `bsp_k230_uart_peek()` 给上层试探。
 */

#include "bsp_k230_uart.h"
#include "ti_msp_dl_config.h"

#define K230_RX_BUF_SIZE   512u

static volatile uint8_t  s_rx_buf[K230_RX_BUF_SIZE];
static volatile uint32_t s_total_rx = 0u;

/* SysConfig 在 ti_msp_dl_config.h 里给 K230 RX 通道定义了
 *   DMA_CH_UART_K230_DMA_RX_INTERRUPT  (NVIC 索引)
 *   DMA_CH_UART_K230_DMA_RX_CHAN       (DMA 通道号)
 * 用宏间接引用，保证 SysConfig 重命名时本文件不需要改。
 *
 * 若用户 SysConfig 把 enableDMARX 关掉，下面这两个符号会缺，编译器报
 * "undeclared identifier"——这是预期的、强制的耦合，提示阶段 1 必须
 * 保留 RX DMA。 */
#ifndef DMA_CH_UART_K230_DMA_RX_CHAN
#define DMA_CH_UART_K230_DMA_RX_CHAN  (0u)
#endif

static void k230_dma_rx_arm(void)
{
    /* 关掉，重设 dst+size，再开 */
    DL_DMA_disableChannel(DMA, DMA_CH_UART_K230_DMA_RX_CHAN);

    DL_DMA_setSrcAddr(DMA, DMA_CH_UART_K230_DMA_RX_CHAN,
        (uint32_t)&UART_K230_INST->RXDATA);
    DL_DMA_setDestAddr(DMA, DMA_CH_UART_K230_DMA_RX_CHAN,
        (uint32_t)&s_rx_buf[0]);
    DL_DMA_setTransferSize(DMA, DMA_CH_UART_K230_DMA_RX_CHAN,
        K230_RX_BUF_SIZE);

    DL_DMA_enableChannel(DMA, DMA_CH_UART_K230_DMA_RX_CHAN);
}

void bsp_k230_uart_init(void)
{
    /* SysConfig 已实例化 UART1 + DMA 通道；本工程仅作 RX，TX 留给后续 */
    DL_UART_Main_clearInterruptStatus(UART_K230_INST,
        DL_UART_MAIN_INTERRUPT_RX | DL_UART_MAIN_INTERRUPT_TX);

    /* 使能 DMA 中断，便于 buf 满后重新装填并累加 total_rx */
    NVIC_EnableIRQ(DMA_INT_IRQn);

    s_total_rx = 0u;
    k230_dma_rx_arm();
}

uint32_t bsp_k230_uart_total_rx(void)
{
    /* 在重装时机之间，DMA 已搬走但还没 IRQ 触发的字节看不到，
     * 这对 1 Hz 自测的字节计数日志影响 < 1 字节，可接受。 */
    return s_total_rx;
}

uint8_t bsp_k230_uart_peek(uint32_t abs_index)
{
    /* abs_index 期望落在 [s_total_rx - K230_RX_BUF_SIZE, s_total_rx) 区间，
     * 否则可能读到上一轮已被覆盖的旧数据；上层自测时务必紧跟最新 total_rx */
    return s_rx_buf[abs_index % K230_RX_BUF_SIZE];
}

void bsp_k230_uart_write_blocking(const uint8_t *data, size_t len)
{
    if (data == NULL) {
        return;
    }
    for (size_t i = 0u; i < len; ++i) {
        while (DL_UART_Main_isBusy(UART_K230_INST)) {
            ;
        }
        DL_UART_Main_transmitDataBlocking(UART_K230_INST, data[i]);
    }
}

/**
 * @brief  DMA 中断（全 16 通道共用 NVIC 槽）。
 *
 *  仅处理 K230 RX 通道完成事件：累加 total_rx，重新装填一轮。
 *  其它 DMA 通道（如 K230 TX 后续启用）应在此函数里追加 case。
 */
void DMA_IRQHandler(void)
{
    /* getPendingInterrupt 会返回当前最高优先级的 pending DMA 事件，并清标志 */
    DL_DMA_EVENT_IIDX iidx = DL_DMA_getPendingInterrupt(DMA);

    /* SDK 把"通道 N 传输完成"枚举命名为 DL_DMA_EVENT_IIDX_DMACH<N> */
    if (iidx == (DL_DMA_EVENT_IIDX_DMACH0 + DMA_CH_UART_K230_DMA_RX_CHAN)) {
        s_total_rx += K230_RX_BUF_SIZE;
        k230_dma_rx_arm();
    }
}
