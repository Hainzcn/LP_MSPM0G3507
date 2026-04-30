/**
 * @file    bsp_systick.c
 * @brief   1 kHz SysTick 实现，详见 bsp_systick.h。
 */

#include "bsp_systick.h"
#include "ti_msp_dl_config.h"

static volatile uint32_t s_ms_count       = 0u;
static volatile uint8_t  s_tick_pending   = 0u;

int32_t bsp_systick_init(uint32_t hz)
{
    if (hz == 0u) {
        return -1;
    }
    /* CPUCLK_FREQ 由 SysConfig 在 ti_msp_dl_config.h 中生成，
     * MSPM0G3507 默认 SYSOSC = 32 MHz；切换到 HFXT 80 MHz 时
     * SysConfig 会自动改写宏值，本文件无需改。
     * 注：MSPM0G3507 (Cortex-M0+) CMSIS 默认未导出 SystemCoreClock 全局变量，
     *     启动文件也没有 SystemInit 初始化它，所以**不能**用 SystemCoreClock。 */
    return (int32_t)SysTick_Config(CPUCLK_FREQ / hz);
}

uint32_t bsp_systick_get_ms(void)
{
    return s_ms_count;
}

void bsp_systick_delay_ms(uint32_t ms)
{
    uint32_t target = s_ms_count + ms;
    while ((int32_t)(target - s_ms_count) > 0) {
        __WFI();
    }
}

bool bsp_systick_consume_tick(void)
{
    bool pending;
    __disable_irq();
    pending = (s_tick_pending != 0u);
    s_tick_pending = 0u;
    __enable_irq();
    return pending;
}

void SysTick_Handler(void)
{
    s_ms_count    += 1u;
    s_tick_pending = 1u;
}
