/**
 * @file    app_buzzer.h
 * @brief   蜂鸣器曲谱播放（非阻塞，1 kHz 主循环驱动）。
 *
 * 音高由 TIMA1 硬件 PWM（bsp_buzzer）在 PB4 产生；本模块负责曲谱时序。
 *
 * 赛道模式：app_track_start() 触发起播，播完自动静音；cancel 时立即停止。
 */

#ifndef APP_BUZZER_H
#define APP_BUZZER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 初始化（默认静音）。 */
void app_buzzer_init(void);

/** 开始播放《兰花草》全曲（前奏 + D.S. 反复 + 尾奏；已在播则忽略）。 */
void app_buzzer_play_lanhua_cao(void);

/** 立即停止并静音。 */
void app_buzzer_stop(void);

/** @return true = 正在播放。 */
bool app_buzzer_is_playing(void);

/**
 * @brief 1 kHz 节拍入口，由 app_balance 主循环每 ms 调用一次。
 */
void app_buzzer_tick_1ms(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_BUZZER_H */
