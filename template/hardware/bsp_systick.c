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
    int32_t rc = (int32_t)SysTick_Config(CPUCLK_FREQ / hz);
    if (rc != 0) {
        return rc;
    }

    /* CMSIS 的 SysTick_Config() 内部会调
     *   NVIC_SetPriority(SysTick_IRQn, (1UL << __NVIC_PRIO_BITS) - 1UL);
     * 把 SysTick **设为最低优先级**（MSPM0G3507 的 __NVIC_PRIO_BITS = 2 → 3）。
     * 这意味着任何 NVIC IRQ（默认优先级 0 = 最高）都能持续抢占 SysTick：
     *   ─ 一旦 GPIOA / UART 等外设 ISR 进入雪崩（如编码器引脚浮空噪声触发
     *     PA12/PA13 高频沿事件），SysTick 永远轮不到执行机会；
     *   ─ s_ms_count 不增、s_tick_pending 不置位 → 主循环 `consume_tick()` 永远
     *     返回 false → 全程 `__WFI()` → 表现为"心跳停摆 + 绿灯不闪"。
     * 这里覆盖为 0（最高），保证系统节拍永不被业务 ISR 饿死。配合
     * `bsp_motor_init()` 把 GPIOA_INT_IRQn 主动降到最低优先级一起使用。 */
    NVIC_SetPriority(SysTick_IRQn, 0u);
    return 0;
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

/* ========================================================================== */
/* HardFault Handler                                                            */
/*                                                                              */
/*   startup_mspm0g350x_uvision.s 里 `HardFault_Handler` 是 weak + `B .` 死循环*/
/*   原始写法：任何总线错误 / 未定义指令 / 错误 vector / 栈溢出 进 HardFault    */
/*   都会让 MCU "假死"，无法看到串口、无法重启、连按键都没用，调试时极度劝退。  */
/*                                                                              */
/*   覆盖为 `NVIC_SystemReset()`，让 fault 直接变 reset：                        */
/*     ─ 若 fault 是偶发硬件干扰 → 复位即可恢复；                                */
/*     ─ 若 fault 持续触发 → 串口能看到 boot log 反复刷屏，可立即定位为 fault   */
/*       而不是"卡死"。比"假死"友好得多。                                       */
/*                                                                              */
/*   留空 fault 现场调试需求（PC / LR / SP 等寄存器 dump）等 Stage 3+ 再补；    */
/*   现阶段先把"看不到症状"变"看得到症状"。                                     */
/* ========================================================================== */

void HardFault_Handler(void)
{
    NVIC_SystemReset();
    while (1) { /* unreachable */ }
}
