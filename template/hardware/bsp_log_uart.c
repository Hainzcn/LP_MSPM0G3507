/**
 * @file    bsp_log_uart.c
 * @brief   UART0 printf retarget。
 *
 * 工程使用 Arm Compiler 6 (armclang) 标准 C 库 + 不开 microLIB（见
 * EIDE/.eide/eide.yml `use-microLIB: false`）。在该组合下，调用 printf
 * 默认会拉入半主机 (semihosting)，导致脱离调试器后程序卡死在 BKPT。
 *
 * 处理办法：
 *   1) 用 `__use_no_semihosting` 强制链接器不带半主机；
 *   2) 实现 `fputc` 把字符走 UART0；
 *   3) 提供少量 stub（`_sys_*` / `_ttywrch` / `system` / `time` / 等）满足
 *      armclang ARM C 库的弱符号依赖，避免链接报 undefined。
 *
 * 仅在 AC6 (armclang) 下编译这些 stub；其它工具链（GCC/IAR）走通用 fputc 即可。
 */

#include "bsp_log_uart.h"
#include "ti_msp_dl_config.h"
#include <stdio.h>
#include <time.h>

void bsp_log_uart_init(void)
{
    /* SysConfig 已配好引脚 / 波特率 / 时钟 / FIFO；这里只清一次 IT 防误触发 */
    DL_UART_Main_clearInterruptStatus(UART_LOG_INST,
        DL_UART_MAIN_INTERRUPT_RX | DL_UART_MAIN_INTERRUPT_TX);
}

void bsp_log_uart_write(const uint8_t *data, size_t len)
{
    if (data == NULL) {
        return;
    }
    for (size_t i = 0u; i < len; ++i) {
        while (DL_UART_Main_isBusy(UART_LOG_INST)) {
            ;
        }
        DL_UART_Main_transmitDataBlocking(UART_LOG_INST, data[i]);
    }
}

bool bsp_log_uart_read_byte(uint8_t *out)
{
    if (out == NULL) {
        return false;
    }
    if (DL_UART_Main_isRXFIFOEmpty(UART_LOG_INST)) {
        return false;
    }
    *out = DL_UART_Main_receiveData(UART_LOG_INST);
    return true;
}

/* ---------------------------------------------------------------------------
 *  C 库 retarget —— 仅 AC6 (armclang) 需要这堆 stub
 *
 *  注：AC6 用 `__asm(".global __use_no_semihosting")` 替代 AC5 的
 *      `#pragma import(__use_no_semihosting)`；struct __FILE 已由 Keil
 *      <stdio.h> 提供，不要在此重复定义，否则 AC6 报 redefinition。
 * ------------------------------------------------------------------------- */
#if defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6000000)

__asm(".global __use_no_semihosting\n\t");

FILE __stdout;
FILE __stdin;
FILE __stderr;

int fputc(int ch, FILE *f)
{
    (void)f;
    uint8_t c = (uint8_t)ch;
    /* 自动 \n -> \r\n，方便 PC 端串口调试器换行 */
    if (c == (uint8_t)'\n') {
        bsp_log_uart_write((const uint8_t *)"\r", 1u);
    }
    bsp_log_uart_write(&c, 1u);
    return ch;
}

int fgetc(FILE *f)
{
    (void)f;
    return -1; /* 暂不支持从日志口读入 */
}

void _ttywrch(int ch)
{
    uint8_t c = (uint8_t)ch;
    bsp_log_uart_write(&c, 1u);
}

void _sys_exit(int x)
{
    (void)x;
    for (;;) { ; }
}

int _sys_open(const char *name, int openmode)
{
    (void)name; (void)openmode;
    return -1;
}

int _sys_close(int fh)
{
    (void)fh;
    return 0;
}

int _sys_write(int fh, const unsigned char *buf, unsigned len, int mode)
{
    (void)fh; (void)mode;
    bsp_log_uart_write((const uint8_t *)buf, (size_t)len);
    return 0;
}

int _sys_read(int fh, unsigned char *buf, unsigned len, int mode)
{
    (void)fh; (void)buf; (void)len; (void)mode;
    return -1;
}

int _sys_istty(int fh)
{
    (void)fh;
    return 0;
}

int _sys_seek(int fh, long pos)
{
    (void)fh; (void)pos;
    return -1;
}

long _sys_flen(int fh)
{
    (void)fh;
    return 0;
}

int _sys_tmpnam(char *name, int sig, unsigned maxlen)
{
    (void)name; (void)sig; (void)maxlen;
    return -1;
}

void _sys_command_string(char *cmd, int len)
{
    (void)cmd; (void)len;
}

clock_t clock(void)
{
    return (clock_t)-1;
}

time_t time(time_t *tloc)
{
    if (tloc) { *tloc = (time_t)0; }
    return (time_t)0;
}

int system(const char *cmd)
{
    (void)cmd;
    return -1;
}

char *getenv(const char *name)
{
    (void)name;
    return (char *)0;
}

#endif /* __ARMCC_VERSION >= 6 */
