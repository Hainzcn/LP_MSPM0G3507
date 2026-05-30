/**
 * @file    bsp_buzzer.c
 * @brief   蜂鸣器 TIMA1 硬件 PWM，详见 bsp_buzzer.h。
 */

#include "bsp_buzzer.h"

#include "bsp_gpio.h"
#include "bsp_systick.h"
#include "ti_msp_dl_config.h"

#define BUZZER_PWM_INST         TIMA1
#define BUZZER_CC_IDX           DL_TIMER_CC_0_INDEX
#define BUZZER_CC_CAPTURE       DL_TIMERA_CAPTURE_COMPARE_0_INDEX
#define BUZZER_IOMUX_FUNC       IOMUX_PINCM17_PF_TIMA1_CCP0

/* TIMA1 为 16-bit 计数器，LOAD/CC 有效位仅低 16 位；低音若不预分频会溢出并静音 */
#define BUZZER_LOAD_MAX         65535u
#define BUZZER_PRESCALE         0u

static bool s_pwm_running;

static DL_Timer_ClockConfig s_clk_cfg = {
    .clockSel    = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_2,
    .prescale    = BUZZER_PRESCALE,
};

static uint32_t buzzer_clk_divisor(DL_TIMER_CLOCK_DIVIDE div)
{
    switch (div) {
    case DL_TIMER_CLOCK_DIVIDE_1: return 1u;
    case DL_TIMER_CLOCK_DIVIDE_2: return 2u;
    case DL_TIMER_CLOCK_DIVIDE_3: return 3u;
    case DL_TIMER_CLOCK_DIVIDE_4: return 4u;
    case DL_TIMER_CLOCK_DIVIDE_5: return 5u;
    case DL_TIMER_CLOCK_DIVIDE_6: return 6u;
    case DL_TIMER_CLOCK_DIVIDE_7: return 7u;
    case DL_TIMER_CLOCK_DIVIDE_8: return 8u;
    default:                    return 8u;
    }
}

static uint32_t buzzer_timer_hz(DL_TIMER_CLOCK_DIVIDE div)
{
    uint32_t ratio = buzzer_clk_divisor(div);

    return CPUCLK_FREQ / (ratio * (uint32_t)(BUZZER_PRESCALE + 1u));
}

/**
 * @brief 为 freq_hz 选择分频，使 LOAD ≤ 65535（16-bit TIMA1）。
 * @return false 表示频率非法或超出可分频范围。
 */
static bool buzzer_calc_pwm(uint16_t freq_hz, DL_TIMER_CLOCK_DIVIDE *div_out,
    uint32_t *load_out, uint32_t *compare_out)
{
    static const DL_TIMER_CLOCK_DIVIDE k_divs[] = {
        DL_TIMER_CLOCK_DIVIDE_1,
        DL_TIMER_CLOCK_DIVIDE_2,
        DL_TIMER_CLOCK_DIVIDE_3,
        DL_TIMER_CLOCK_DIVIDE_4,
        DL_TIMER_CLOCK_DIVIDE_5,
        DL_TIMER_CLOCK_DIVIDE_6,
        DL_TIMER_CLOCK_DIVIDE_7,
        DL_TIMER_CLOCK_DIVIDE_8,
    };
    unsigned i;

    if (freq_hz == 0u) {
        return false;
    }

    for (i = 0u; i < (sizeof(k_divs) / sizeof(k_divs[0])); i++) {
        uint32_t timer_hz = buzzer_timer_hz(k_divs[i]);
        uint32_t period   = timer_hz / (uint32_t)freq_hz;
        uint32_t load;

        if (period < 3u) {
            period = 3u;
        }
        load = period - 1u;
        if (load <= BUZZER_LOAD_MAX) {
            *div_out     = k_divs[i];
            *load_out    = load;
            *compare_out = load / 2u;
            return true;
        }
    }

    return false;
}

static void mux_gpio_high(void)
{
    DL_GPIO_initDigitalOutput(BSP_BUZZER_IOMUX);
    DL_GPIO_setPins(BSP_BUZZER_PORT, BSP_BUZZER_PIN);
    DL_GPIO_enableOutput(BSP_BUZZER_PORT, BSP_BUZZER_PIN);
}

static void mux_pwm_out(void)
{
    DL_GPIO_initPeripheralOutputFunction(BSP_BUZZER_IOMUX, BUZZER_IOMUX_FUNC);
    DL_GPIO_enableOutput(BSP_BUZZER_PORT, BSP_BUZZER_PIN);
}

static void apply_pwm_tone(uint16_t freq_hz)
{
    DL_TIMER_CLOCK_DIVIDE div;
    uint32_t              load;
    uint32_t              compare;

    if (!buzzer_calc_pwm(freq_hz, &div, &load, &compare)) {
        return;
    }

    if (s_clk_cfg.divideRatio != div) {
        s_clk_cfg.divideRatio = div;
        DL_TimerA_setClockConfig(BUZZER_PWM_INST, &s_clk_cfg);
    }

    DL_TimerA_setLoadValue(BUZZER_PWM_INST, load);
    DL_TimerA_setCaptureCompareValue(BUZZER_PWM_INST, compare, BUZZER_CC_IDX);

    if (!s_pwm_running) {
        mux_pwm_out();
        DL_TimerA_startCounter(BUZZER_PWM_INST);
        s_pwm_running = true;
    }
}

void bsp_buzzer_init(void)
{
    static const DL_TimerA_PWMConfig s_pwm = {
        .pwmMode           = DL_TIMER_PWM_MODE_EDGE_ALIGN,
        .period            = 999u,
        .isTimerWithFourCC = false,
        .startTimer        = DL_TIMER_STOP,
    };

    s_pwm_running = false;

    DL_TimerA_reset(BUZZER_PWM_INST);
    DL_TimerA_enablePower(BUZZER_PWM_INST);

    DL_TimerA_setClockConfig(BUZZER_PWM_INST, &s_clk_cfg);
    DL_TimerA_initPWMMode(BUZZER_PWM_INST, (DL_TimerA_PWMConfig *)&s_pwm);

    DL_TimerA_setCounterControl(BUZZER_PWM_INST,
        DL_TIMER_CZC_CCCTL0_ZCOND,
        DL_TIMER_CAC_CCCTL0_ACOND,
        DL_TIMER_CLC_CCCTL0_LCOND);

    /* 低电平触发：反相输出 + 50% 占空 → 引脚在 VDD/GND 间方波驱动无源蜂鸣器 */
    DL_TimerA_setCaptureCompareOutCtl(BUZZER_PWM_INST,
        DL_TIMER_CC_OCTL_INIT_VAL_LOW,
        DL_TIMER_CC_OCTL_INV_OUT_ENABLED,
        DL_TIMER_CC_OCTL_SRC_FUNCVAL,
        BUZZER_CC_CAPTURE);

    DL_TimerA_setCaptCompUpdateMethod(BUZZER_PWM_INST,
        DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, BUZZER_CC_CAPTURE);

    DL_TimerA_setCaptureCompareValue(BUZZER_PWM_INST, 999u, BUZZER_CC_IDX);
    DL_TimerA_enableClock(BUZZER_PWM_INST);
    DL_TimerA_setCCPDirection(BUZZER_PWM_INST, DL_TIMER_CC0_OUTPUT);

    mux_gpio_high();
}

void bsp_buzzer_set_tone_hz(uint16_t freq_hz)
{
    if (freq_hz == 0u) {
        if (s_pwm_running) {
            DL_TimerA_stopCounter(BUZZER_PWM_INST);
            s_pwm_running = false;
            mux_gpio_high();
        }
        return;
    }

    apply_pwm_tone(freq_hz);
}

void bsp_buzzer_beep_ms(uint16_t freq_hz, uint32_t duration_ms)
{
    if (freq_hz == 0u || duration_ms == 0u) {
        bsp_buzzer_set_tone_hz(0u);
        return;
    }

    bsp_buzzer_set_tone_hz(freq_hz);
    bsp_systick_delay_ms(duration_ms);
    bsp_buzzer_set_tone_hz(0u);
}
