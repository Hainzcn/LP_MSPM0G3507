/**
 * @file    app_buzzer.h
 * @brief   蜂鸣器曲谱/提示音播放（非阻塞，1 kHz 主循环驱动）。
 *
 * 音高由 TIMA1 硬件 PWM（bsp_buzzer）在 PB4 产生；本模块负责时序。
 *
 * 赛道模式提示音（app_track enter_phase）：
 *   STOOD_UP     自立完成 → 进入稳定确认
 *   TRACE_START  稳定 5 s 后进入循迹
 *   LAP_PAUSE    满圈暂停
 *   ALL_DONE     全部圈数完成
 */

#ifndef APP_BUZZER_H
#define APP_BUZZER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 赛道模式短提示音（会打断当前播放）。 */
typedef enum {
    APP_BUZZER_CUE_STOOD_UP = 0,
    APP_BUZZER_CUE_TRACE_START,
    APP_BUZZER_CUE_LAP_PAUSE,
    APP_BUZZER_CUE_ALL_DONE,

    APP_BUZZER_CUE_COUNT,
    APP_BUZZER_FORCE_INT32_ = 0x7FFFFFFF
} app_buzzer_cue_t;

/** 初始化（默认静音）。 */
void app_buzzer_init(void);

/** 播放赛道模式短提示音（已在播则打断并切到新提示）。 */
void app_buzzer_play_cue(app_buzzer_cue_t cue);

/** 开始播放《兰花草》全曲（调试/演示用）。 */
void app_buzzer_play_lanhua_cao(void);

/** 立即停止并静音。 */
void app_buzzer_stop(void);

/** @return true = 正在播放。 */
bool app_buzzer_is_playing(void);

/** 1 kHz 节拍入口，由 app_balance 主循环每 ms 调用一次。 */
void app_buzzer_tick_1ms(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_BUZZER_H */
