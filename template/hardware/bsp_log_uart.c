/**
 * @file    bsp_log_uart.c
 * @brief   UART0 printf 重定向 —— 让 printf 输出到 XDS110 USB 串口
 *
 * ============================================================
 * 这个文件是做什么的？（写给初学者）
 * ============================================================
 * 在 bsp_log_uart.h 中我们说过："printf 输出需要重定向到 UART"。
 * 这个 bsp_log_uart.c 就是"具体怎么重定向"的实现。
 *
 * 这个文件包含两个主要部分：
 *
 *   上半部分（简单）：三个业务函数
 *     bsp_log_uart_init() —— 初始化
 *     bsp_log_uart_write() —— 发送数据
 *     bsp_log_uart_read_byte() —— 读取一个字节
 *
 *   下半部分（复杂）：printf 重定向的"胶水代码"
 *     实现 fputc() —— printf 的底层输出函数
 *     实现一堆 _sys_xxx() 函数 —— 给 ARM 编译器提供"标准库依赖"
 *
 * 下半部分是整个文件最复杂的地方。简单说就是：
 *   为了让 printf("hello") 能正常工作，
 *   ARM 的 C 库需要一些"底层函数"的支持。
 *   这些函数在桌面电脑上由操作系统提供，
 *   在单片机上需要我们自己写"空壳"（stub）。
 *
 * ============================================================
 * 什么是半主机（Semihosting）？
 * ============================================================
 * 半主机是一种调试技术——单片机通过调试器（如 XDS110）
 * 把 printf 的输出"转发"到电脑的 IDE 控制台。
 * 它的好处是：不需要 UART 引脚，直接通过调试器就能看到输出。
 *
 * 但半主机有严重的缺点：
 *   一旦拔掉调试器（脱离调试状态运行），
 *   printf 会触发 BKPT（断点）指令，导致程序卡死。
 *
 * 所以我们要做的是：
 *   1. 用 __use_no_semihosting 告诉链接器"不要半主机"
 *   2. 自己实现 fputc 把字符送到 UART
 *   3. 补齐 C 库需要的其他底层函数（stub）
 *
 * ============================================================
 * 本文件只在 Arm Compiler 6 (armclang) 下需要这些 stub
 * ============================================================
 * 本工程使用 Arm Compiler 6（armclang）+ 不开 microLIB。
 * 在这个组合下，C 库会要求提供 _sys_open、_sys_write 等函数。
 * 如果使用 GCC 或 IAR 编译器，这些 stub 不需要。
 *
 * 所有的 stub 函数都用 #if defined(__ARMCC_VERSION)
 * 包裹，只在 armclang 下编译，不影响其他工具链。
 */

/* ============================================================
 * 头文件包含
 * ============================================================ */
#include "bsp_log_uart.h"         /* 引入本模块的函数声明 */
#include "ti_msp_dl_config.h"     /* 引入 SysConfig 生成的配置：
                                   *   UART_LOG_INST —— UART0 实例
                                   *   DL_UART_Main_xxx 函数声明 */
#include <stdio.h>                /* 引入 FILE 类型定义、printf 声明 */
#include <time.h>                 /* 引入 clock_t、time_t 类型定义 */

/* ================================================================
 * bsp_log_uart_init() —— 初始化日志串口
 * ================================================================
 * UART 的硬件配置（引脚、波特率、时钟）已经由 SysConfig 在
 * SYSCFG_DL_init() 中完成了。
 *
 * 这里只需要多做一件事：清除中断标志。
 *
 * 为什么需要清除中断标志？
 *   SysConfig 初始化 UART 时可能会设置一些中断标志位。
 *   如果不清除，这些标志位可能立即触发一次无意义的中断，
 *   但此时还没有注册对应的中断服务函数（ISR），
 *   导致程序进入默认的 fault handler。
 *
 * 注意：这个函数只清除了中断标志，并没有使能任何 UART 中断。
 * 本模块的发送方式是"阻塞式轮询"（polling），
 * 不使用中断，所以也不需要配置中断优先级和使能中断。
 */
void bsp_log_uart_init(void)
{
    /* 清除接收（RX）和发送（TX）的中断标志。
     * DL_UART_MAIN_INTERRUPT_RX：接收中断标志
     * DL_UART_MAIN_INTERRUPT_TX：发送中断标志
     * 
     * 这两个标志用 | 合并，一次调用清除两者。
     * 
     * UART_LOG_INST 是 SysConfig 生成的宏，
     * 代表 UART0 的实例（寄存器的基地址）。
     * 在 ti_msp_dl_config.h 中定义。
     * 
     * DL_UART_Main_clearInterruptStatus() 的作用是
     * 向 UART 的中断状态寄存器写 1 来清除标志位。
     * 这符合 TI 芯片的"写 1 清除"（Write-1-to-Clear）习惯。 */
    DL_UART_Main_clearInterruptStatus(UART_LOG_INST,
        DL_UART_MAIN_INTERRUPT_RX | DL_UART_MAIN_INTERRUPT_TX);
}

/* ================================================================
 * bsp_log_uart_write() —— 阻塞式发送数据
 * ================================================================
 * 功能：把 data 指向的 len 个字节逐一发送到 UART0。
 *
 * 实现方式：逐字节阻塞发送
 *   对于 data 中的每一个字节：
 *     1. 等待 UART 发送移位寄存器空闲（isBusy 返回 false）
 *     2. 调用 transmitDataBlocking 写入该字节
 *
 * 为什么是"阻塞"的？
 *   isBusy 循环一直在等待——直到前一个字节发送完成，
 *   CPU 不能做其他事情。
 *   对于调试日志（数据量小、不需要实时性），这是可接受的。
 *
 * 为什么不使用中断或 DMA 发送？
 *   中断和 DMA 适合大量数据的发送，但增加了代码复杂度。
 *   对于每秒几十个字节的调试日志，阻塞轮询足够了。
 *
 * @param data  待发送数据的缓冲区
 * @param len   要发送的字节数
 */
void bsp_log_uart_write(const uint8_t *data, size_t len)
{
    /* 防御性编程：如果 data 是 NULL，直接返回 */
    if (data == NULL) {
        return;
    }

    /* 逐字节发送 */
    for (size_t i = 0u; i < len; ++i) {
        /* 等待 UART 发送就绪
         * DL_UART_Main_isBusy() 检查发送移位寄存器是否为空。
         * 如果还在发送上一个字节，返回 true，循环等待。
         * 这种"原地等待"就是"阻塞"的含义。
         * 
         * 注意：循环体是空语句（;），没有任何操作。
         * 这种写法叫"忙等待"（busy wait）或"自旋等待"（spin wait）。 */
        while (DL_UART_Main_isBusy(UART_LOG_INST)) {
            ;
        }

        /* 发送当前字节
         * DL_UART_Main_transmitDataBlocking() 把 data[i] 写入
         * UART 的发送数据寄存器（TXDATA）。
         * 硬件会自动把这个字节从 TX 引脚发送出去。
         * 
         * "Blocking"（阻塞）的含义：
         *   如果发送 FIFO 已满，这个函数会等待到 FIFO 有空位。
         *   但我们前面已经用 isBusy 确保发送器空闲了，
         *   所以这里的 blocking 通常不会真的"阻塞"。 */
        DL_UART_Main_transmitDataBlocking(UART_LOG_INST, data[i]);
    }
}

/* ================================================================
 * bsp_log_uart_read_byte() —— 非阻塞式读取一个字节
 * ================================================================
 * 功能：从 UART0 的接收 FIFO 中尝试读取一个字节。
 *
 * 非阻塞的含义：
 *   如果接收 FIFO 中有数据，立刻读取并返回 true。
 *   如果接收 FIFO 中没有数据，立刻返回 false（不等待）。
 *
 * 这个函数通常在主循环中"轮询"调用：
 *   if (bsp_log_uart_read_byte(&ch)) {
 *       handle_byte(ch);
 *   }
 *
 * @param out  输出参数：读到字节后写入此变量
 * @return true = 成功读到；false = 无可读数据
 */
bool bsp_log_uart_read_byte(uint8_t *out)
{
    /* 防御性编程：输出指针不能为 NULL */
    if (out == NULL) {
        return false;
    }

    /* 检查接收 FIFO 是否为空
     * DL_UART_Main_isRXFIFOEmpty() 返回 true 表示没有数据。
     * 如果没有数据，直接返回 false（非阻塞）。 */
    if (DL_UART_Main_isRXFIFOEmpty(UART_LOG_INST)) {
        return false;
    }

    /* 从接收 FIFO 中读取一个字节
     * DL_UART_Main_receiveData() 从 UART 的接收数据寄存器（RXDATA）
     * 读取一个字节。因为前面已经检查过 FIFO 不为空，
     * 所以这里一定能读到有效数据。 */
    *out = DL_UART_Main_receiveData(UART_LOG_INST);
    return true;
}

/* ================================================================
 * C 库重定向（Retarget）—— 仅 Arm Compiler 6 (armclang) 需要
 * ================================================================
 *
 * #if defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6000000)
 *
 * 这行条件编译的意思是："只在 Arm Compiler 6 及更高版本下编译下面的代码"。
 * __ARMCC_VERSION 是 ARM 编译器预定义的宏：
 *   - AC5（Arm Compiler 5）：__ARMCC_VERSION 在 5000000~5999999 之间
 *   - AC6（Arm Compiler 6）：__ARMCC_VERSION >= 6000000
 *
 * 所以下面的代码只在 armclang 下编译，GCC/IAR 直接跳过。
 */
#if defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6000000)

/* ================================================================
 * 禁止半主机模式
 * ================================================================
 * __asm(".global __use_no_semihosting\n\t");
 *
 * 这是一条内联汇编指令，告诉 ARM 链接器：
 *   "这个程序不使用半主机（semihosting）功能"
 *
 * 如果不加这个指令，链接器可能会拉入半主机相关的库函数。
 * 这些函数会调用 BKPT（断点）指令，在脱离调试器时导致程序卡死。
 *
 * 原理：
 *   在 ARM C 库中，有一组"半主机"函数，它们通过调试器
 *   把输入输出转发到主机（电脑）。默认情况下，
 *   printf 最终会调用半主机函数。
 *
 *   加上 __use_no_semihosting 后，链接器就不会链接半主机函数，
 *   而是要求我们自己提供 fputc 等底层函数。
 *   这就是"重定向"的本质——自己不实现，链接器报错；
 *   自己实现了，链接器用我们的版本。
 *
 * .global 伪指令：
 *   把 __use_no_semihosting 符号导出为全局符号，
 *   让链接器能够看到它。
 */
__asm(".global __use_no_semihosting\n\t");

/* ================================================================
 * 标准 I/O 文件流实例
 * ================================================================
 * FILE __stdout;
 * FILE __stdin;
 * FILE __stderr;
 *
 * C 语言标准库中，printf 输出到 stdout（标准输出），
 * scanf 从 stdin（标准输入）读取，错误信息输出到 stderr。
 *
 * 在桌面电脑上，这些是操作系统管理的"文件流"。
 * 在单片机上，我们需要为它们分配内存——即使我们不使用 scanf 和 stderr。
 *
 * 为什么需要这三个变量？
 *   因为 ARM C 库中引用了 __stdout、__stdin、__stderr 这三个符号。
 *   如果我们不定义它们，链接器会报"undefined symbol"错误。
 *   定义成全局变量就满足了链接器的需求。
 *
 * 注意：struct __FILE 已经在 Keil 的 <stdio.h> 中定义了，
 * 所以这里不需要再定义 __FILE 结构体，否则会报"redefinition"错误。
 */
FILE __stdout;
FILE __stdin;
FILE __stderr;

/* ================================================================
 * fputc() —— printf 的底层输出函数（核心！）
 * ================================================================
 * 这是整个 printf 重定向的核心函数。
 *
 * C 标准库中的 printf、puts、putchar 等函数，
 * 最终都会调用 fputc(int ch, FILE *f) 来输出每一个字符。
 *
 * 我们的实现：
 *   1. 把 int ch 转成 uint8_t c
 *   2. 如果是换行符 '\n'，在前面加回车 '\r'
 *      （电脑串口助手通常需要 \r\n 才能正确换行，不仅仅是 \n）
 *   3. 调用 bsp_log_uart_write() 发送一个字节
 *   4. 返回 ch（fputc 的返回值规定为写入的字符）
 *
 * 为什么需要 \n → \r\n 转换？
 *   在 Unix/Linux 系统中，换行 = \n（0x0A）
 *   在 Windows 系统中，换行 = \r\n（0x0D 0x0A）
 *   大多数 Windows 串口助手期望 \r\n 才能正确显示换行。
 *   如果只发 \n，串口助手可能只换行不回行（阶梯状输出）。
 *
 * @param ch  要输出的字符（int 类型，取值范围 0~255）
 * @param f   目标文件流（我们忽略——所有输出走 UART）
 * @return    写入的字符（返回 ch 本身表示成功）
 */
int fputc(int ch, FILE *f)
{
    /* (void)f：消除"未使用参数"的编译器警告
     * 因为我们只有一个 UART 输出口，不区分 stdout/stderr。 */
    (void)f;

    /* int → uint8_t 转换
     * ch 虽然是 int，但实际值在 0~255 范围内，强制转换安全 */
    uint8_t c = (uint8_t)ch;

    /* 自动 \n → \r\n 转换
     * 如果当前字符是换行符 \n（0x0A），
     * 先发送一个回车符 \r（0x0D），
     * 再发送 \n 本身。
     * 这样电脑串口助手就能正确换行。 */
    if (c == (uint8_t)'\n') {
        bsp_log_uart_write((const uint8_t *)"\r", 1u);
    }

    /* 发送当前字符
     * bsp_log_uart_write 会阻塞等待，确保字符发送完成。 */
    bsp_log_uart_write(&c, 1u);

    /* 返回 ch（C 标准规定 fputc 返回写入的字符，失败返回 EOF） */
    return ch;
}

/* ================================================================
 * fgetc() —— 从串口读取字符（未实现）
 * ================================================================
 * fgetc 是 fputc 的"反向"操作——从输入流读取一个字符。
 * 对应到我们的场景，就是从 UART 接收一个字符。
 *
 * 但我们暂不实现这个功能，直接返回 -1 表示"不支持读取"。
 * 如果需要读取串口输入，应使用 bsp_log_uart_read_byte()。
 *
 * @param f  目标文件流
 * @return   -1（暂不支持）
 */
int fgetc(FILE *f)
{
    (void)f;
    return -1; /* 暂不支持从日志口读入 */
}

/* ================================================================
 * 以下是一系列"stub"（存根）函数
 * ================================================================
 * 这些函数是 ARM C 库需要的底层系统调用接口（syscall）。
 * 在桌面电脑上，它们由操作系统实现（打开文件、读写文件等）。
 * 在单片机上，我们没有操作系统，也不需要文件操作，
 * 但 C 库仍然会调用它们——所以我们必须提供"空实现"。
 *
 * 所有 stub 函数的共同特点：
 *   1. 用 (void)xxx 消除未使用参数警告
 *   2. 返回一个"表示不支持"的默认值（通常是 -1 或 0）
 *   3. 不做任何实际的文件操作
 */

/**
 * _ttywrch —— 输出一个字符到"控制台"
 * 这是 ARM C 库中的底层函数，fputc 可能会调用它。
 * 我们让它同样走 UART 输出。
 */
void _ttywrch(int ch)
{
    uint8_t c = (uint8_t)ch;
    bsp_log_uart_write(&c, 1u);
}

/**
 * _sys_exit —— 程序退出
 * 当程序调用 exit() 或 main() 返回时，此函数被调用。
 * 在单片机上，程序不应该"退出"——没有操作系统来回收资源。
 * 所以我们让它进入死循环：系统卡在这里。
 * 实际上，main() 不会返回，这个函数不应该被触发。
 */
void _sys_exit(int x)
{
    (void)x;
    for (;;) { ; }  /* 死循环——程序"挂起" */
}

/**
 * _sys_open —— 打开文件
 * 单片机没有文件系统，所以永远返回 -1（打开失败）。
 */
int _sys_open(const char *name, int openmode)
{
    (void)name; (void)openmode;
    return -1;
}

/**
 * _sys_close —— 关闭文件
 * 因为没有文件打开，所以"关闭"只是返回 0（成功）。
 */
int _sys_close(int fh)
{
    (void)fh;
    return 0;
}

/**
 * _sys_write —— 写入文件
 * 当 printf 最终调用到"写文件"层时，会走这个函数。
 * 我们的实现：直接把数据通过 UART 发送。
 * 这样 printf 的整个调用链就是：
 *   printf → fputc → _sys_write → bsp_log_uart_write → UART 发送
 */
int _sys_write(int fh, const unsigned char *buf, unsigned len, int mode)
{
    (void)fh; (void)mode;
    bsp_log_uart_write((const uint8_t *)buf, (size_t)len);
    return 0;
}

/**
 * _sys_read —— 读取文件
 * 不支持从日志口读取，返回 -1。
 */
int _sys_read(int fh, unsigned char *buf, unsigned len, int mode)
{
    (void)fh; (void)buf; (void)len; (void)mode;
    return -1;
}

/**
 * _sys_istty —— 判断文件是否为终端
 * 返回 0 表示"不是终端"（串口不是终端设备）。
 */
int _sys_istty(int fh)
{
    (void)fh;
    return 0;
}

/**
 * _sys_seek —— 文件定位
 * 不支持，返回 -1。
 */
int _sys_seek(int fh, long pos)
{
    (void)fh; (void)pos;
    return -1;
}

/**
 * _sys_flen —— 获取文件长度
 * 不支持，返回 0。
 */
long _sys_flen(int fh)
{
    (void)fh;
    return 0;
}

/**
 * _sys_tmpnam —— 生成临时文件名
 * 不支持，返回 -1。
 */
int _sys_tmpnam(char *name, int sig, unsigned maxlen)
{
    (void)name; (void)sig; (void)maxlen;
    return -1;
}

/**
 * _sys_command_string —— 获取命令行参数
 * 不支持，空实现。
 */
void _sys_command_string(char *cmd, int len)
{
    (void)cmd; (void)len;
}

/**
 * clock() —— 获取 CPU 时钟周期
 * C 标准库中的 clock() 函数返回程序运行的 CPU 时钟周期数。
 * 本工程不使用这个函数，返回 -1 表示"不支持"。
 * （本工程的时间函数使用 bsp_systick_get_ms()）
 */
clock_t clock(void)
{
    return (clock_t)-1;
}

/**
 * time() —— 获取当前时间
 * C 标准库中的 time() 函数返回当前日历时间。
 * 单片机没有实时时钟（RTC），返回 0。
 */
time_t time(time_t *tloc)
{
    if (tloc) { *tloc = (time_t)0; }
    return (time_t)0;
}

/**
 * system() —— 执行系统命令
 * 单片机没有操作系统，不支持执行命令，返回 -1。
 */
int system(const char *cmd)
{
    (void)cmd;
    return -1;
}

/**
 * getenv() —— 获取环境变量
 * 单片机没有环境变量，返回 NULL。
 */
char *getenv(const char *name)
{
    (void)name;
    return (char *)0;
}

#endif /* __ARMCC_VERSION >= 6 */
/* 条件编译结束：以上所有代码只在 Arm Compiler 6 下编译 */
