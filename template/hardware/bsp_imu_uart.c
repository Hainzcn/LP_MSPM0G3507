/**
 * @file    bsp_imu_uart.c
 * @brief   ATK-MS901M UART3 接收实现，详见 bsp_imu_uart.h。
 *          Stage 1.6 起占用 UART3 + PB12/PB13（原蓝牙引脚），不再使用 UART2。
 *
 * 实现要点：
 *   · 256 B 环形缓冲（2 的幂便于 (idx & MASK) 折回）
 *   · ISR 写 head，主循环读 tail；head/tail 都是 16-bit volatile，
 *     在 Cortex-M0+ 上单次访问原子，无需关中断
 *   · 满缓冲策略：丢弃新字节并累加 overrun 计数，由 1 Hz 自测日志观察
 *   · pop_bulk 内不关中断（同上），并发只产生"少读 1~2 B 当前 ISR 入队"
 *     这种良性偏差，下一拍 drain 时会补回
 */

#include "bsp_imu_uart.h"
#include "ti_msp_dl_config.h"

#define IMU_RX_BUF_SIZE   256u
#define IMU_RX_BUF_MASK   (IMU_RX_BUF_SIZE - 1u)

static volatile uint8_t  s_rx_buf[IMU_RX_BUF_SIZE];
static volatile uint16_t s_rx_head = 0u;   /* ISR 写 */
static volatile uint16_t s_rx_tail = 0u;   /* 应用读 */
static volatile uint32_t s_rx_overrun = 0u;

void bsp_imu_uart_init(void)
{
    /* SysConfig 已配好引脚 / 波特率 / FIFO + 外设级 RX 中断使能位（IMSC）。
     * 但 SDK 2.10 的 SYSCFG_DL_UART_IMU_init 仅调用 DL_UART_Main_enableInterrupt
     * 设外设 IMSC，**不**会自动 NVIC_EnableIRQ；NVIC 必须由用户代码显式开启，
     * 否则 RX FIFO 半满中断永远不会进入 CPU 调度（症状：MS901M 数据流到达
     * 串口但 UART3_IRQHandler 永不触发，环缓 head 不前进，ms901m_has_attitude
     * 永 false → main 等待窗口超时 fatal）。
     *
     * 参考对照：bsp_k230_uart.c:bsp_k230_uart_init() 同样显式调用
     * NVIC_EnableIRQ(DMA_INT_IRQn) 才能让 K230 RX DMA 完成中断生效。 */
    DL_UART_Main_clearInterruptStatus(UART_IMU_INST,
        DL_UART_MAIN_INTERRUPT_RX | DL_UART_MAIN_INTERRUPT_TX);
    NVIC_EnableIRQ(UART_IMU_INST_INT_IRQN);

    s_rx_head = 0u;
    s_rx_tail = 0u;
    s_rx_overrun = 0u;
}

void bsp_imu_uart_write(const uint8_t *data, size_t len)
{
    if (data == NULL) {
        return;
    }
    for (size_t i = 0u; i < len; ++i) {
        while (DL_UART_Main_isBusy(UART_IMU_INST)) {
            ;
        }
        DL_UART_Main_transmitDataBlocking(UART_IMU_INST, data[i]);
    }
}

bool bsp_imu_uart_rx_pop(uint8_t *out)
{
    if (out == NULL) {
        return false;
    }
    if (s_rx_head == s_rx_tail) {
        return false;
    }
    *out = s_rx_buf[s_rx_tail];
    s_rx_tail = (uint16_t)((s_rx_tail + 1u) & IMU_RX_BUF_MASK);
    return true;
}

size_t bsp_imu_uart_rx_pop_bulk(uint8_t *dst, size_t max_len)
{
    if (dst == NULL || max_len == 0u) {
        return 0u;
    }

    size_t got = 0u;
    while (got < max_len) {
        uint16_t h = s_rx_head;
        uint16_t t = s_rx_tail;
        if (h == t) {
            break;
        }
        dst[got++] = s_rx_buf[t];
        s_rx_tail = (uint16_t)((t + 1u) & IMU_RX_BUF_MASK);
    }
    return got;
}

size_t bsp_imu_uart_rx_available(void)
{
    uint16_t h = s_rx_head;
    uint16_t t = s_rx_tail;
    return (size_t)((h - t) & IMU_RX_BUF_MASK);
}

uint32_t bsp_imu_uart_rx_overrun(void)
{
    return s_rx_overrun;
}

/**
 * @brief  UART3 中断服务函数（IMU UART，Stage 1.6 起）。
 * @note   函数名由 startup_mspm0g350x_uvision.s 中的向量表决定，
 *         在 MSPM0G3507 SDK 中是 `UART3_IRQHandler`。
 *         若以后 IMU UART 切到其它外设号，**必须**同步把这里函数名改成
 *         对应的 `UARTx_IRQHandler`，否则中断不会触发但编译能过（弱符号
 *         回退到 startup 默认死循环）。
 *
 *         Stage 1.6 之前 IMU 占 UART2，本函数原名 `UART2_IRQHandler`；
 *         同期蓝牙模块下线，UART3 向量从 bsp_bt_uart.c 让出给 IMU。
 */
void UART3_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_IMU_INST)) {
        case DL_UART_MAIN_IIDX_RX: {
            /* RX FIFO 半满或超时触发；把 FIFO 里的字节全搬出 */
            while (DL_UART_Main_isRXFIFOEmpty(UART_IMU_INST) == false) {
                uint8_t b = (uint8_t)DL_UART_Main_receiveData(UART_IMU_INST);
                uint16_t next = (uint16_t)((s_rx_head + 1u) & IMU_RX_BUF_MASK);
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
