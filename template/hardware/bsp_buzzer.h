/**
 * @file    bsp_buzzer.h
 * @brief   无源蜂鸣器硬件 PWM 驱动（TIMA1 CCP0 → PB4，BoosterPack NO.21）。
 *
 * 低电平触发模块：静音时 GPIO 拉高；发声时 TIMA1 输出 50% 占空方波（输出反相），
 * 由硬件定时器产生准确音频频率，避免 1 kHz 软件翻转带来的严重失真。
 *
 * TIMA1 为 16-bit 定时器：低音（32 MHz 不分频时约低于 490 Hz）须自动加大
 * CLKDIV，否则 LOAD 溢出会导致比较值大于周期、PWM 无输出。
 */

#ifndef BSP_BUZZER_H
#define BSP_BUZZER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 初始化 TIMA1 PWM（默认静音）。须在 SYSCFG_DL_init() 之后调用。 */
void bsp_buzzer_init(void);

/**
 * @brief 设置发声频率（Hz）；freq_hz=0 停止并恢复 GPIO 高电平静音。
 */
void bsp_buzzer_set_tone_hz(uint16_t freq_hz);

/**
 * @brief 阻塞鸣叫一次（用于 boot fatal 等），播完自动静音。
 */
void bsp_buzzer_beep_ms(uint16_t freq_hz, uint32_t duration_ms);

#ifdef __cplusplus
}
#endif

#endif /* BSP_BUZZER_H */
