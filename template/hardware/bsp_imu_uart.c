/**
 * @file    bsp_imu_uart.c
 * @brief   ATK-MS901M UART3 接收实现 —— 环形缓冲区 + ISR 逐字节入队 + 主循环批量出队
 *
 * ============================================================
 * 这个文件是做什么的？（写给初学者）
 * ============================================================
 * 在 bsp_imu_uart.h 中我们说过，这个模块的核心是"环形缓冲区"。
 * 这个 bsp_imu_uart.c 就是"环形缓冲区具体怎么运作"的实现。
 *
 * 整个模块的工作模式是"生产者-消费者"模式：
 *   生产者（ISR）：UART3_IRQHandler 每收到一个字节就放入环形缓冲区
 *   消费者（主循环）：pop_bulk 批量取出字节，喂给 ms901m_feed_bytes()
 *
 * 环形缓冲区的工作原理：
 *   它就像一个"环形队列"——有头指针（head，ISR 写）和尾指针（tail，主循环读）。
 *   写数据时 head 前进；读数据时 tail 前进。
 *   当头指针追上尾指针时，缓冲区为空；当头指针+1 追上尾指针时，缓冲区满。
 *
 *   用 2 的幂作为缓冲区大小（256 = 2^8），使得取模运算可以用位运算加速：
 *     index = (index + 1) & (size - 1)  // 等价于 index = (index + 1) % 256
 *
 * ============================================================
 * 为什么在中断里只存不处理？
 * ============================================================
 * 中断服务函数（ISR）必须尽可能短小精悍。
 * 在 ISR 中调用 ms901m_feed_bytes()（涉及浮点运算）会严重拖慢中断响应。
 * 所以 ISR 只做"把字节塞进缓冲区"这一件事。
 * 字节的"解析"工作留到主循环中做。
 *
 * 这是嵌入式系统设计的黄金法则：
 *   ISR 中只做"记录事件"（存数据、置标志），
 *   主循环中做"处理事件"（解析数据、执行控制）。
 */

/* ============================================================
 * 头文件包含
 * ============================================================ */
#include "bsp_imu_uart.h"
#include "ti_msp_dl_config.h"   /* SysConfig 生成：
                                  *   UART_IMU_INST —— UART3 实例
                                  *   UART_IMU_INST_INT_IRQN —— UART3 中断号
                                  *   DL_UART_Main_xxx 函数声明 */

/* ================================================================
 * 环形缓冲区定义
 * ================================================================
 * 这里定义了一个 256 字节的环形缓冲区（Ring Buffer）。
 *
 * 为什么用 256？
 *   256 = 2^8，是一个 2 的幂。2 的幂的好处是：
 *   取模操作 (index % 256) 可以用位运算 (index & 255) 代替，更快。
 *
 * MS901M 一秒钟发送约 15 KB 数据（5 帧 × 200 Hz × 15 字节/帧）。
 * 每毫秒约 15 字节。256 字节的缓冲区可以缓存约 17 毫秒的数据——
 * 远高于主循环的 1 ms 处理周期，所以不会溢出。
 */

/** 缓冲区大小：256 字节（2 的幂） */
#define IMU_RX_BUF_SIZE   256u

/** 缓冲区掩码：255 = 0xFF，用于位运算取模 */
#define IMU_RX_BUF_MASK   (IMU_RX_BUF_SIZE - 1u)

/** 环形缓冲区：存储 UART 收到的原始字节 */
static volatile uint8_t  s_rx_buf[IMU_RX_BUF_SIZE];

/**
 * s_rx_head：写指针（ISR 写数据时移动）
 *   指向"下一个要写入的位置"。
 *   每次 ISR 写入一个字节后，head = (head + 1) & MASK
 *   volatile：因为 ISR 修改它，主循环读取它。
 */
static volatile uint16_t s_rx_head = 0u;

/**
 * s_rx_tail：读指针（主循环读数据时移动）
 *   指向"下一个要读取的位置"。
 *   每次 pop/pop_bulk 读取一个字节后，tail = (tail + 1) & MASK
 *   volatile：因为主循环修改它，ISR 也读取它（判断缓冲区是否满）。
 */
static volatile uint16_t s_rx_tail = 0u;

/**
 * s_rx_overrun：累计溢出次数
 *   当 ISR 尝试写入但缓冲区已满（head+1 == tail）时，
 *   新字节被丢弃，overrun 加 1。
 *   这个值可以从主循环中读取，用于监控通信质量。
 */
static volatile uint32_t s_rx_overrun = 0u;

/* ================================================================
 * bsp_imu_uart_init() —— 初始化 IMU 串口
 * ================================================================
 * 这个函数初始化 UART3，让它能接收 MS901M 的数据。
 *
 * 大部分硬件配置（引脚分配、波特率 115200、FIFO 模式）已经由
 * SysConfig 在 SYSCFG_DL_init() 中完成了。
 *
 * 这里做 SysConfig 没做的三件事：
 *   1. 清除可能残留的中断标志
 *   2. ⚠️ 使能 NVIC 中的 UART3 中断（SysConfig 不会做这一步！）
 *   3. 复位环形缓冲区指针和溢出计数
 *
 * 第 2 点特别重要！
 *   SysConfig 只会配置 UART 硬件级的中断使能（IMSC 寄存器），
 *   但不会调用 NVIC_EnableIRQ()。
 *   如果不手动使能 NVIC，RX 数据到达时 UART 硬件会产生中断信号，
 *   但 CPU 不会响应——因为 NVIC 没开这个通道。
 *   结果就是：MS901M 发数据了，UART 收到了，但 ISR 永不触发。
 */
void bsp_imu_uart_init(void)
{
    /* 清除接收和发送的中断标志
     * 防止 SysConfig 初始化后残留的中断标志误触发 ISR */
    DL_UART_Main_clearInterruptStatus(UART_IMU_INST,
        DL_UART_MAIN_INTERRUPT_RX | DL_UART_MAIN_INTERRUPT_TX);

    /* ⭐ 使能 NVIC 中的 UART3 中断通道
     * 这是 SysConfig 不会做的一步！
     * 如果不加这行，ISR 永远不会触发。 */
    NVIC_EnableIRQ(UART_IMU_INST_INT_IRQN);

    /* 复位环形缓冲区指针和溢出计数 */
    s_rx_head = 0u;
    s_rx_tail = 0u;
    s_rx_overrun = 0u;
}

/* ================================================================
 * bsp_imu_uart_write() —— 阻塞式发送数据
 * ================================================================
 * 往 MS901M 发送配置命令（如设置量程、改变上报频率）。
 * 实现方式同 bsp_log_uart_write()——逐字节阻塞发送。
 *
 * 不是所有串口驱动都有"写"功能——
 * 很多传感器只发不收，不需要单片机发送数据。
 * 但 MS901M 支持双向通信，我们可以配置它。
 *
 * @param data  待发送数据指针
 * @param len   要发送的字节数
 */
void bsp_imu_uart_write(const uint8_t *data, size_t len)
{
    /* 防御性编程：data 不能为 NULL */
    if (data == NULL) {
        return;
    }

    /* 逐字节发送 */
    for (size_t i = 0u; i < len; ++i) {
        /* 等待 UART 发送器空闲 */
        while (DL_UART_Main_isBusy(UART_IMU_INST)) {
            ;
        }
        /* 发送当前字节 */
        DL_UART_Main_transmitDataBlocking(UART_IMU_INST, data[i]);
    }
}

/* ================================================================
 * bsp_imu_uart_rx_pop() —— 单字节弹出
 * ================================================================
 * 从环形缓冲区中取出一个字节。
 * 非阻塞：如果没有数据，立即返回 false。
 *
 * 这个函数主要用于"一次只处理一个字节"的场景。
 * 对于批量处理，推荐使用 pop_bulk。
 *
 * @param out  输出参数：成功时写入读取的字节
 * @return true = 有数据；false = 缓冲区空
 */
bool bsp_imu_uart_rx_pop(uint8_t *out)
{
    /* 输出指针不能为 NULL */
    if (out == NULL) {
        return false;
    }

    /* 检查缓冲区是否为空：head == tail 表示空 */
    if (s_rx_head == s_rx_tail) {
        return false;
    }

    /* 取出当前的字节并移动 tail 指针
     * (tail + 1) & IMU_RX_BUF_MASK 等价于 (tail + 1) % 256
     * 当 tail = 255 时：255 + 1 = 256，256 & 255 = 0 → 折回开头 */
    *out = s_rx_buf[s_rx_tail];
    s_rx_tail = (uint16_t)((s_rx_tail + 1u) & IMU_RX_BUF_MASK);
    return true;
}

/* ================================================================
 * bsp_imu_uart_rx_pop_bulk() —— 批量弹出（⭐ 推荐使用）
 * ================================================================
 * 一次性从环形缓冲区取出多个字节（最多 max_len 个）。
 *
 * 这个函数比循环调用 pop() 更高效：
 *   一次函数调用取出多个字节 vs 多次函数调用每次取一个字节。
 *
 * 典型用法（在主循环中每 1 ms 调用）：
 * @code
 *   uint8_t buf[64];
 *   size_t n = bsp_imu_uart_rx_pop_bulk(buf, sizeof(buf));
 *   if (n > 0) {
 *       ms901m_feed_bytes(buf, n);
 *   }
 * @endcode
 *
 * @param dst     目标缓冲区
 * @param max_len 最多取多少个字节
 * @return 实际取出的字节数（可能小于 max_len）
 */
size_t bsp_imu_uart_rx_pop_bulk(uint8_t *dst, size_t max_len)
{
    /* 参数合法性检查 */
    if (dst == NULL || max_len == 0u) {
        return 0u;
    }

    /* 循环取出字节，直到取满 max_len 或缓冲区为空 */
    size_t got = 0u;
    while (got < max_len) {
        /* 读取 head 和 tail。
         * 注意：这里每次循环都重新读 s_rx_head，
         * 因为 ISR 可能在循环过程中修改了它。 */
        uint16_t h = s_rx_head;
        uint16_t t = s_rx_tail;
        if (h == t) {
            break;  /* 缓冲区为空 */
        }

        /* 取出一个字节并移动 tail */
        dst[got++] = s_rx_buf[t];
        s_rx_tail = (uint16_t)((t + 1u) & IMU_RX_BUF_MASK);
    }
    return got;
}

/* ================================================================
 * bsp_imu_uart_rx_available() —— 查询缓冲区可读字节数
 * ================================================================
 * 计算环形缓冲区中当前有多少字节可用。
 *
 * 公式：(head - tail) & MASK
 * 利用无符号整数溢出来处理 head 和 tail 的"折回"（回绕）问题。
 *
 * 例如 head=10, tail=5 → (10-5) & 255 = 5（正确）
 * 例如 head=5, tail=250（tail 折回了，head 落后但实际已绕了一圈）
 *      → (5-250) & 255 = (-245) & 255 = 11（正确！）
 *
 * @return 当前可读字节数（0 ~ 255）
 */
size_t bsp_imu_uart_rx_available(void)
{
    uint16_t h = s_rx_head;
    uint16_t t = s_rx_tail;
    return (size_t)((h - t) & IMU_RX_BUF_MASK);
}

/* ================================================================
 * bsp_imu_uart_rx_overrun() —— 查询溢出计数
 * ================================================================
 * 如果这个值持续增长，说明主循环处理速度跟不上 ISR 接收速度。
 * 对于 MS901M 约 15 KB/s 的数据率，256 字节缓冲区 + 1 kHz 主循环，
 * 正常情况下 overrun 应该永远是 0。
 *
 * @return 累计溢出次数
 */
uint32_t bsp_imu_uart_rx_overrun(void)
{
    return s_rx_overrun;
}

/* ================================================================
 * UART3_IRQHandler —— UART3 中断服务函数
 * ================================================================
 *
 * 这个函数每当中断产生时被 NVIC 调用。
 * 中断来源：UART3 的 RX FIFO 达到半满水平或接收超时。
 *
 * ⚠️ 函数名很重要！
 *   函数名由启动文件（startup_mspm0g350x_uvision.s）的中断向量表决定。
 *   对于 UART3，向量表入口是 UART3_IRQHandler（weak 符号）。
 *   如果我们定义了这个函数，链接器用我们的版本覆盖 weak 版本。
 *
 *   如果函数名写错（如 UART2_IRQHandler），编译链接不报错，
 *   但向量表仍然指向默认的 weak 函数（死循环），中断永不触发。
 *
 *   Stage 1.6 后 IMU 从 UART2 搬到 UART3，函数名也从 UART2_IRQHandler
 *   改为了 UART3_IRQHandler。
 *
 * ISR 中的处理逻辑：
 *   1. 通过 DL_UART_Main_getPendingInterrupt() 读取中断类型（IIDX）
 *   2. 如果是 RX 中断（IIDX_RX），进入循环处理
 *   3. 循环把 RX FIFO 中的所有字节全部搬出
 *   4. 如果缓冲区已满（head+1 == tail），丢弃新字节并累加 overrun
 *   5. 否则写入缓冲区并移动 head
 *
 * 为什么用 while 循环搬空 FIFO，而不是只搬一个字节？
 *   RX FIFO 半满中断意味着 FIFO 中有多个字节等待读取。
 *   如果不一次性搬空，多次进出 ISR 的开销更大。
 *   一次搬空更高效。
 */
void UART3_IRQHandler(void)
{
    /* 读取中断标识（IIDX = Interrupt IDentiﬁer）
     * 判断是什么类型的中断触发了 ISR */
    switch (DL_UART_Main_getPendingInterrupt(UART_IMU_INST)) {

        /* RX 中断：RX FIFO 达到半满水平，或者接收超时 */
        case DL_UART_MAIN_IIDX_RX: {
            /* ---- 把 FIFO 中的所有字节搬出到环形缓冲区 ---- */
            /* while 循环持续读取，直到 FIFO 为空
             * isRXFIFOEmpty() 返回 false = FIFO 中还有数据 */
            while (DL_UART_Main_isRXFIFOEmpty(UART_IMU_INST) == false) {
                /* 从 UART 接收数据寄存器读取一个字节 */
                uint8_t b = (uint8_t)DL_UART_Main_receiveData(UART_IMU_INST);

                /* 计算下一个 head 位置
                 * (head + 1) & MASK —— 用位运算实现取模 256
                 * 当 head = 255 时自动折回到 0 */
                uint16_t next = (uint16_t)((s_rx_head + 1u) & IMU_RX_BUF_MASK);

                /* 检查缓冲区是否已满
                 * 如果 next == tail，说明 head 追上了 tail，缓冲区满了
                 * 此时新字节无法写入，只能丢弃 */
                if (next == s_rx_tail) {
                    /* 缓冲区满：丢弃这个字节，累计溢出次数
                     * overrun 计数可以让主循环在调试日志中观察
                     * 如果 overrun 持续增长，说明环缓冲太小或者主循环太慢 */
                    s_rx_overrun++;
                } else {
                    /* 写入缓冲区并更新 head
                     * s_rx_buf[s_rx_head] = b;
                     * s_rx_head = next; */
                    s_rx_buf[s_rx_head] = b;
                    s_rx_head = next;
                }
            }
            break;
        }

        /* 其他类型的中断（如 TX 完成、错误等）——忽略 */
        default:
            break;
    }
}
