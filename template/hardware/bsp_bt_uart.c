/**
 * @file    bsp_bt_uart.c
 * @brief   蓝牙 UART2 实现，详见 bsp_bt_uart.h。
 */

#include "bsp_bt_uart.h"
#include "ti_msp_dl_config.h"

#define BT_RX_BUF_SIZE   256u   /* 必须为 2 的幂，方便用 (idx & MASK) 折回 */
#define BT_RX_BUF_MASK   (BT_RX_BUF_SIZE - 1u)

static volatile uint8_t  s_rx_buf[BT_RX_BUF_SIZE];
static volatile uint16_t s_rx_head = 0u;   /* ISR 写 */
static volatile uint16_t s_rx_tail = 0u;   /* 应用读 */
static volatile uint32_t s_rx_overrun = 0u;

void bsp_bt_uart_init(void)
{
    /* SysConfig 已配好引脚 / 波特率 / FIFO；这里只清挂起的 IT，避免误触发 */
    DL_UART_Main_clearInterruptStatus(UART_BT_INST,
        DL_UART_MAIN_INTERRUPT_RX | DL_UART_MAIN_INTERRUPT_TX);

    /* SDK 在 SYSCFG_DL_UART_BT_init 里已经使能 RX 中断 + NVIC（见 syscfg
     * 中 enabledInterrupts = ["RX"]），此处不重复使能，避免覆盖 */
    s_rx_head = 0u;
    s_rx_tail = 0u;
    s_rx_overrun = 0u;
}

void bsp_bt_uart_write(const uint8_t *data, size_t len)
{
    if (data == NULL) {
        return;
    }
    for (size_t i = 0u; i < len; ++i) {
        while (DL_UART_Main_isBusy(UART_BT_INST)) {
            ;
        }
        DL_UART_Main_transmitDataBlocking(UART_BT_INST, data[i]);
    }
}

bool bsp_bt_uart_rx_pop(uint8_t *out)
{
    if (out == NULL) {
        return false;
    }
    if (s_rx_head == s_rx_tail) {
        return false;
    }
    *out = s_rx_buf[s_rx_tail];
    s_rx_tail = (uint16_t)((s_rx_tail + 1u) & BT_RX_BUF_MASK);
    return true;
}

size_t bsp_bt_uart_rx_available(void)
{
    /* head/tail 都是 16-bit volatile，访问原子。无需关中断 */
    uint16_t h = s_rx_head;
    uint16_t t = s_rx_tail;
    return (size_t)((h - t) & BT_RX_BUF_MASK);
}

/**
 * @brief  UART3 中断服务函数（蓝牙 UART）。
 * @note   函数名由 startup_mspm0g350x_uvision.s 中的向量表决定，
 *         在 MSPM0G3507 SDK 中是 `UART3_IRQHandler`。
 *         若以后蓝牙 UART 切回 UART2/UART1/UART0，**必须**同步把这里
 *         的函数名改成对应的 `UARTx_IRQHandler`，否则中断不会触发但
 *         编译能过（弱符号回退到 startup 默认死循环）。
 */
void UART3_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_BT_INST)) {
        case DL_UART_MAIN_IIDX_RX: {
            /* RX FIFO 半满或超时触发；把 FIFO 里的字节全搬出 */
            while (DL_UART_Main_isRXFIFOEmpty(UART_BT_INST) == false) {
                uint8_t b = (uint8_t)DL_UART_Main_receiveData(UART_BT_INST);
                uint16_t next = (uint16_t)((s_rx_head + 1u) & BT_RX_BUF_MASK);
                if (next == s_rx_tail) {
                    /* 缓冲满：丢弃新字节并计数；上层 1 Hz 日志能观察到 */
                    s_rx_overrun++;
                } else {
                    s_rx_buf[s_rx_head] = b;
                    s_rx_head = next;
                }
            }
            break;
        }
        default:
            break;
    }
}
