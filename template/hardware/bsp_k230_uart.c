/**
 * @file    bsp_k230_uart.c
 * @brief   K230 视觉处理器通讯 UART1 —— DMA RX 接收骨架实现
 *
 * ============================================================
 * 这个文件是做什么的？（写给初学者）
 * ============================================================
 * 在 bsp_k230_uart.h 中我们说过：这个模块是 MSPM0G3507 和 K230
 * 两个芯片之间的"通讯管道"，使用 UART1 + DMA。
 *
 * 这个 bsp_k230_uart.c 是"DMA 具体怎么收数据"的实现。
 *
 * 简单说，我们让 DMA 做这样一件事：
 *   "自动把 UART1 收到的每一个字节，搬到 s_rx_buf 这个数组里，
 *   搬满 512 个字节就告诉我一声（中断），然后从头开始继续搬。"
 *
 * ============================================================
 * 什么是 DMA？
 * ============================================================
 * DMA = Direct Memory Access（直接存储器访问）。
 *
 * 没有 DMA 时：CPU 要亲自把 UART 收到的每个字节从寄存器搬到内存。
 *   for (每个字节) {
 *       buf[i] = UART->RXDATA;  // CPU 亲自做
 *   }
 *
 * 有 DMA 时：DMA 控制器硬件自动完成搬运，CPU 可以去做其他事。
 *   配置 DMA → DMA 自己搬 → 搬完了通知 CPU
 *
 * 形象地说，DMA 就像一个"自动搬运工"——
 * 你告诉它"把 A 处的东西搬到 B 处，搬 N 次"，
 * 然后它自己默默地搬，搬完了喊你一声。
 *
 * ============================================================
 * BLOCK 模式 + 重启策略
 * ============================================================
 * 本模块使用 DMA 的 BLOCK（块）传输模式。
 *
 * 工作方式：
 *   1. 配置 DMA 搬运 512 字节（源=UART 数据寄存器，目标=s_rx_buf）
 *   2. DMA 开始自动搬运——每收到一个字节就搬一次
 *   3. 搬完 512 字节 → DMA 触发中断 → ISR 被调用
 *   4. ISR 中：累加总字节数 + 重新装填 DMA（再次指向 buf 开头）
 *   5. DMA 继续从头开始搬，覆盖旧数据
 *
 * 缺点：不是真正的环形缓冲区。DMA 覆盖旧数据时，如果上层还没读完，
 *       旧数据就丢失了。但对于阶段 1 的"只统计字节数"足够了。
 *
 * 真正的"零拷贝环形缓冲区"需要 DMA 支持 AUTO 重装模式，
 * 但 MSPM0G3507 的 SDK 不支持 UART RX DMA 的 AUTO 重装。
 * 所以这里用"中断时手动重装"的折中方案。
 */

/* ============================================================
 * 头文件包含
 * ============================================================ */
#include "bsp_k230_uart.h"
#include "ti_msp_dl_config.h"   /* SysConfig 生成：
                                  *   UART_K230_INST —— UART1 实例
                                  *   DMA —— DMA 控制器实例
                                  *   DMA_CH_UART_K230_DMA_RX_CHAN —— RX 通道号
                                  *   DMA_INT_IRQn —— DMA 中断号 */

/* ================================================================
 * 环形缓冲区定义
 * ================================================================
 * 这里的"环形"指的是：DMA 装满 512 字节后重新指向开头，
 * 覆盖旧数据。对于阶段 1 的"只测字节数"需求足够了。
 *
 * 如果后续需要完整的数据帧解析，应该改为：
 *   1. UART RX FIFO 半满/全满中断（像 bsp_imu_uart 那样）
 *   2. 或使用双缓冲区（ping-pong buffer）
 */

/** 接收缓冲区大小：512 字节。
 *  为什么选 512？2 的幂，方便取模运算。
 *  对于 K230 的数据率足够大，DMA 中断频率不会太高。 */
#define K230_RX_BUF_SIZE   512u

/** 接收缓冲区：DMA 把 UART 收到的数据自动写入这里。
 *  volatile：DMA 硬件修改它（不通过 CPU），编译器不能优化掉对它的读取。 */
static volatile uint8_t  s_rx_buf[K230_RX_BUF_SIZE];

/** DMA 累计接收字节数。
 *  每装满一次缓冲区（512 字节），s_total_rx += 512。
 *  用相邻两次读数的差值可以算出"一段时间内收到了多少字节"。 */
static volatile uint32_t s_total_rx = 0u;

/**
 * DMA 通道号保护宏。
 *
 * SysConfig 在 ti_msp_dl_config.h 中定义了
 * DMA_CH_UART_K230_DMA_RX_CHAN 宏。
 * 如果用户在 SysConfig 中关闭了 UART1 的 RX DMA 使能，
 * 这个宏就不存在——编译器会报错。
 *
 * 这种设计叫"编译期强耦合"——如果配置不对，编译不过，
 * 而不是运行时才暴露问题。
 */
#ifndef DMA_CH_UART_K230_DMA_RX_CHAN
#define DMA_CH_UART_K230_DMA_RX_CHAN  (0u)
#endif

/* ================================================================
 * k230_dma_rx_arm() —— 装填（ARM）DMA RX 通道
 * ================================================================
 * 这个函数配置 DMA 通道，让它准备好搬运数据。
 *
 * 每次 DMA 完成一次传输（装满 512 字节）后，
 * 都必须重新调用这个函数来装填下一次传输。
 *
 * 配置了三样东西：
 *   1. 源地址（Source）：UART1 的接收数据寄存器地址
 *      DMA 从这个地址读取数据（UART 收到的字节）
 *   2. 目的地址（Destination）：s_rx_buf 数组的起始地址
 *      DMA 把数据写入这里
 *   3. 传输大小（Transfer Size）：512 字节
 *      DMA 搬完这么多字节后触发中断
 *
 * 工作流程：
 *   对于 UART1 收到的每一个字节，DMA 硬件自动执行：
 *     1. 从 UART1_RXDATA 寄存器读取一个字节
 *     2. 写入 s_rx_buf[current_position]
 *     3. current_position++（目的地址自增）
 *     4. 剩余字节数减 1
 *   当剩余字节数减到 0 时，触发 DMA 中断。
 */
static void k230_dma_rx_arm(void)
{
    /* ---- 步骤 1：关闭 DMA 通道 ----
     * 在修改 DMA 通道的配置时，必须先禁用该通道。
     * 否则正在进行的传输可能被打乱。 */
    DL_DMA_disableChannel(DMA, DMA_CH_UART_K230_DMA_RX_CHAN);

    /* ---- 步骤 2：设置源地址 ----
     * 源地址是 UART1 的接收数据寄存器（RXDATA）。
     * 注意：这里是 UART 外设的寄存器地址，不是内存地址。
     * DMA 从外设读取数据时，源地址不自增——
     * 因为每次都从同一个寄存器读取。 */
    DL_DMA_setSrcAddr(DMA, DMA_CH_UART_K230_DMA_RX_CHAN,
        (uint32_t)&UART_K230_INST->RXDATA);

    /* ---- 步骤 3：设置目的地址 ----
     * 目的地址是 s_rx_buf 数组的起始地址。
     * DMA 写入内存时，目的地址**自动递增**——
     * 第一个字节写到 buf[0]，第二个写到 buf[1]，以此类推。 */
    DL_DMA_setDestAddr(DMA, DMA_CH_UART_K230_DMA_RX_CHAN,
        (uint32_t)&s_rx_buf[0]);

    /* ---- 步骤 4：设置传输大小 ----
     * 告诉 DMA 要搬多少字节。
     * 搬完这些字节后，DMA 会触发一次完成中断。 */
    DL_DMA_setTransferSize(DMA, DMA_CH_UART_K230_DMA_RX_CHAN,
        K230_RX_BUF_SIZE);

    /* ---- 步骤 5：开启 DMA 通道 ----
     * 使能后，DMA 开始等待 UART1 的数据。
     * 每来一个字节，DMA 自动搬一个字节到缓冲区。
     * 不需要 CPU 参与。 */
    DL_DMA_enableChannel(DMA, DMA_CH_UART_K230_DMA_RX_CHAN);
}

/* ================================================================
 * bsp_k230_uart_init() —— 初始化
 * ================================================================
 * 初始化 UART1 的 DMA 接收功能。
 *
 * SysConfig 已经完成了大部分硬件配置：
 *   - UART1 的引脚、波特率、FIFO 配置
 *   - DMA 通道的触发源（UART1 RX）配置
 *
 * 这里做剩下的工作：
 *   1. 清除 UART 中断标志
 *   2. 使能 DMA 中断（NVIC 级别）
 *   3. 复位累计计数
 *   4. 首次装填 DMA 通道（开始接收）
 */
void bsp_k230_uart_init(void)
{
    /* 清除 UART1 的中断标志
     * 防止 SysConfig 初始化后残留的标志误触发 */
    DL_UART_Main_clearInterruptStatus(UART_K230_INST,
        DL_UART_MAIN_INTERRUPT_RX | DL_UART_MAIN_INTERRUPT_TX);

    /* 使能 DMA 中断
     * DMA 传输完成时会产生中断，需要 NVIC 使能才能响应。
     * 注意：DMA 中断号是 DMA_INT_IRQn，不是 UART 的中断号。
     * DMA 有自己的 NVIC 通道。 */
    NVIC_EnableIRQ(DMA_INT_IRQn);

    /* 复位累计接收字节数 */
    s_total_rx = 0u;

    /* 首次装填 DMA：开始接收！ */
    k230_dma_rx_arm();
}

/* ================================================================
 * bsp_k230_uart_total_rx() —— 查询累计接收字节数
 * ================================================================
 * 返回从初始化至今，DMA 累计搬运的字节总数。
 *
 * 注意：在两次 DMA 中断之间，有部分字节已经被 DMA 搬到了缓冲区，
 * 但还没有触发中断——所以 s_total_rx 可能比实际字节数少
 * 最多 511 个字节。
 *
 * 但对于 1 Hz 的"每秒收到了多少字节"统计来说，
 * 这种偏差最多 1 个字节（512 字节/中断 × 1 秒/次中断），
 * 完全可以接受。
 *
 * @return 累计接收字节数
 */
uint32_t bsp_k230_uart_total_rx(void)
{
    return s_total_rx;
}

/* ================================================================
 * bsp_k230_uart_peek() —— 读取缓冲区中的指定字节
 * ================================================================
 * 根据绝对索引 abs_index 计算在环形缓冲区中的位置并返回该字节。
 *
 * 环形缓冲区的特点：
 *   索引 = abs_index % K230_RX_BUF_SIZE
 *   当索引超过 512 时自动"折回"到开头。
 *
 * 使用限制：
 *   只能访问"最近一次 DMA 装填之后写入的字节"。
 *   如果 abs_index 对应的位置在 DMA 重新装填之前，
 *   可能读到的是上一轮的旧数据。
 *
 *   所以调用方应该紧跟 bsp_k230_uart_total_rx() 的最新值，
 *   确保 abs_index 落在 [total - 512, total) 区间内。
 *
 * @param abs_index  全局绝对索引（从 0 开始递增的序号）
 * @return 该索引位置的一个字节
 */
uint8_t bsp_k230_uart_peek(uint32_t abs_index)
{
    /* 用取模运算实现环形缓冲区的"折回"
     * 例如 abs_index = 520 → 520 % 512 = 8 → 读取 buf[8] */
    return s_rx_buf[abs_index % K230_RX_BUF_SIZE];
}

/* ================================================================
 * bsp_k230_uart_write_blocking() —— 阻塞式发送
 * ================================================================
 * 通过轮询方式逐字节发送数据。
 *
 * 不走 DMA 的原因：
 *   本阶段不需要从 MSPM0G3507 向 K230 发送大量数据，
 *   轮询方式简单可靠，不需要额外配置 DMA TX 通道。
 *
 * 后续阶段如果需要大流量 TX，可以新增 DMA TX 配置。
 */
void bsp_k230_uart_write_blocking(const uint8_t *data, size_t len)
{
    /* 防御性编程 */
    if (data == NULL) {
        return;
    }

    /* 逐字节发送
     * 每个字节发送前等待 UART 发送器空闲 */
    for (size_t i = 0u; i < len; ++i) {
        while (DL_UART_Main_isBusy(UART_K230_INST)) {
            ;
        }
        DL_UART_Main_transmitDataBlocking(UART_K230_INST, data[i]);
    }
}

/* ================================================================
 * DMA_IRQHandler —— DMA 中断服务函数
 * ================================================================
 *
 * 这个函数在所有 DMA 通道传输完成时被调用。
 * 注意：DMA 的所有通道（0~15）共享同一个 NVIC 中断入口！
 *
 * 所以进入 ISR 后，需要通过 DL_DMA_getPendingInterrupt()
 * 判断具体是哪个通道触发了中断。
 *
 * 本模块只关心 K230 RX 通道的完成事件。
 * 如果后续启用其他 DMA 通道（如 K230 TX），
 * 需要在这个函数中追加对应的 case。
 *
 * ISR 中的处理逻辑：
 *   1. 读取中断标识（IIDX），判断是哪个通道
 *   2. 如果是 K230 RX 通道：
 *      a. 累计接收字节数 += 512
 *      b. 重新装填 DMA 通道（从头开始接收）
 *
 * 注意：DL_DMA_getPendingInterrupt() 在读取 IIDX 的同时
 * 会自动清除中断标志，所以不需要手动 clear。
 */
void DMA_IRQHandler(void)
{
    /* 获取当前最高优先级的 pending DMA 事件
     * 返回值是一个枚举值，标识是哪个通道的什么事件。
     * 同时这个函数会自动清除中断标志。 */
    DL_DMA_EVENT_IIDX iidx = DL_DMA_getPendingInterrupt(DMA);

    /* 判断是否是 K230 RX 通道的传输完成事件
     * DL_DMA_EVENT_IIDX_DMACH0 是"通道 0 传输完成"的枚举值。
     * 加上通道号偏移：DMA_CH_UART_K230_DMA_RX_CHAN + DMACH0
     * 得到"指定通道的传输完成"事件标识。 */
    if (iidx == (DL_DMA_EVENT_IIDX_DMACH0 + DMA_CH_UART_K230_DMA_RX_CHAN)) {
        /* 累计接收字节数 */
        s_total_rx += K230_RX_BUF_SIZE;

        /* 重新装填 DMA：开始下一轮接收
         * 重新指向 buf 开头，覆盖旧数据 */
        k230_dma_rx_arm();
    }
}
