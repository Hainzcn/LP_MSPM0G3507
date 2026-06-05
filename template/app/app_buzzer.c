/**
 * @file    app_buzzer.c
 * @brief   蜂鸣器曲谱播放实现，详见 app_buzzer.h。
 *
 * 曲谱：《兰花草》简谱 1=C 4/4 ♩=96（下加点用 NOTE_L* 低八度）。
 * 音高由 bsp_buzzer TIMA1 硬件 PWM 产生；本模块仅负责音符时序调度。
 */

#include "app_buzzer.h"

#include "bsp_buzzer.h"
#include "bsp_gpio.h"
#include "bsp_systick.h"
#include "ti_msp_dl_config.h"

#include <stdio.h>

/* ── 音符频率 (Hz) ───────────────────────────────────────────────────────── */

#define NOTE_REST  0u

#define NOTE_L1    262u
#define NOTE_L2    294u
#define NOTE_L3    330u
#define NOTE_L4    349u
#define NOTE_L4S   370u
#define NOTE_L5    392u
#define NOTE_L6    440u
#define NOTE_L7    494u

#define NOTE_M1    523u
#define NOTE_M2    587u
#define NOTE_M3    659u
#define NOTE_M4    698u
#define NOTE_M4S   740u
#define NOTE_M5    784u
#define NOTE_M6    880u
#define NOTE_M7    988u

#define NOTE_H1    1047u
#define NOTE_H2    1175u
#define NOTE_H3    1319u
#define NOTE_H4    1397u
#define NOTE_H5    1568u
#define NOTE_H6    1760u
#define NOTE_H7    1976u

/* 速度 96 BPM，四分音符 = 625 ms */
#define DUR_QUARTER         625u
#define DUR_EIGHTH          312u
#define DUR_SIXTEENTH       156u
#define DUR_HALF            1250u
#define DUR_DOTTED_EIGHTH   468u

/*
 * 《兰花草》1=C 4/4 ♩=96；无下加点→中音 NOTE_M*，下加点→低音 NOTE_L*。
 * 结构：前奏 M1–4 → 主旋律 M5–12 → D.S. 反复 M5–12 → 尾奏 M13–14。
 * {频率 Hz, 持续 ms}；末项 {NOTE_REST, 0} 为结束标记。
 */
static const uint16_t s_lanhua_cao[][2] = {
    /* ── 前奏 M1–M4 ── */
    /* M1: 3 · 6 6 · 5 · 3· 2 */
    {NOTE_M3, DUR_QUARTER},
    {NOTE_M6, DUR_EIGHTH}, {NOTE_M6, DUR_EIGHTH},
    {NOTE_M5, DUR_QUARTER},
    {NOTE_M3, DUR_DOTTED_EIGHTH}, {NOTE_M2, DUR_SIXTEENTH},
    /* M2: 1· 2 · 1 · 7̲ 6̲ 3̲ */
    {NOTE_M1, DUR_DOTTED_EIGHTH}, {NOTE_M2, DUR_SIXTEENTH},
    {NOTE_M1, DUR_QUARTER},
    {NOTE_L7, DUR_EIGHTH}, {NOTE_L6, DUR_EIGHTH}, {NOTE_L3, DUR_QUARTER},
    /* M3: 3̲· 1 · 1 · 7̲ 6̲ 3̲ */
    {NOTE_L3, DUR_DOTTED_EIGHTH}, {NOTE_M1, DUR_SIXTEENTH},
    {NOTE_M1, DUR_QUARTER},
    {NOTE_L7, DUR_EIGHTH}, {NOTE_L6, DUR_EIGHTH}, {NOTE_L3, DUR_QUARTER},
    /* M4: 2· 1 · 7̲ 5̲ 6̲- */
    {NOTE_M2, DUR_DOTTED_EIGHTH}, {NOTE_M1, DUR_SIXTEENTH},
    {NOTE_L7, DUR_EIGHTH}, {NOTE_L5, DUR_EIGHTH},
    {NOTE_L6, DUR_HALF},

    /* ── 主旋律 M5–M12（𝄋）── */
    /* M5: 6̲ · 3 3 · 3 · 3· 2  我从山中来 */
    {NOTE_L6, DUR_QUARTER},
    {NOTE_M3, DUR_EIGHTH}, {NOTE_M3, DUR_EIGHTH},
    {NOTE_M3, DUR_QUARTER},
    {NOTE_M3, DUR_DOTTED_EIGHTH}, {NOTE_M2, DUR_SIXTEENTH},
    /* M6: 1· 2 · 1 · 7̲ 6̲-  带着兰花草 */
    {NOTE_M1, DUR_DOTTED_EIGHTH}, {NOTE_M2, DUR_SIXTEENTH},
    {NOTE_M1, DUR_QUARTER},
    {NOTE_L7, DUR_EIGHTH}, {NOTE_L6, DUR_HALF},
    /* M7: 6 · 6 6 · 6 · 6· 5  种在小园中 */
    {NOTE_M6, DUR_QUARTER},
    {NOTE_M6, DUR_EIGHTH}, {NOTE_M6, DUR_EIGHTH},
    {NOTE_M6, DUR_QUARTER},
    {NOTE_M6, DUR_DOTTED_EIGHTH}, {NOTE_M5, DUR_SIXTEENTH},
    /* M8: 3 · 5 5 · #4 · 3-  希望花开早 */
    {NOTE_M3, DUR_QUARTER},
    {NOTE_M5, DUR_EIGHTH}, {NOTE_M5, DUR_EIGHTH},
    {NOTE_M4S, DUR_QUARTER},
    {NOTE_M3, DUR_HALF},
    /* M9: 3 · 6 6 · 5 · 3· 2  一日看三回 */
    {NOTE_M3, DUR_QUARTER},
    {NOTE_M6, DUR_EIGHTH}, {NOTE_M6, DUR_EIGHTH},
    {NOTE_M5, DUR_QUARTER},
    {NOTE_M3, DUR_DOTTED_EIGHTH}, {NOTE_M2, DUR_SIXTEENTH},
    /* M10: 1· 2 · 1 · 7̲ 6̲ 3̲  看得花时过 */
    {NOTE_M1, DUR_DOTTED_EIGHTH}, {NOTE_M2, DUR_SIXTEENTH},
    {NOTE_M1, DUR_QUARTER},
    {NOTE_L7, DUR_EIGHTH}, {NOTE_L6, DUR_EIGHTH}, {NOTE_L3, DUR_QUARTER},
    /* M11: 3̲· 1 · 1 · 7̲ 6̲ 3̲  兰花却依然 */
    {NOTE_L3, DUR_DOTTED_EIGHTH}, {NOTE_M1, DUR_SIXTEENTH},
    {NOTE_M1, DUR_QUARTER},
    {NOTE_L7, DUR_EIGHTH}, {NOTE_L6, DUR_EIGHTH}, {NOTE_L3, DUR_QUARTER},
    /* M12: 2· 1 · 7̲ 5̲ 6̲-  苞也无一个 */
    {NOTE_M2, DUR_DOTTED_EIGHTH}, {NOTE_M1, DUR_SIXTEENTH},
    {NOTE_L7, DUR_EIGHTH}, {NOTE_L5, DUR_EIGHTH},
    {NOTE_L6, DUR_HALF},

    /* ── D.S. 反复 M5–M12 ── */
    /* M5: 6̲ · 3 3 · 3 · 3· 2  转眼秋天到 */
    {NOTE_L6, DUR_QUARTER},
    {NOTE_M3, DUR_EIGHTH}, {NOTE_M3, DUR_EIGHTH},
    {NOTE_M3, DUR_QUARTER},
    {NOTE_M3, DUR_DOTTED_EIGHTH}, {NOTE_M2, DUR_SIXTEENTH},
    /* M6: 1· 2 · 1 · 7̲ 6̲-  移兰入暖房 */
    {NOTE_M1, DUR_DOTTED_EIGHTH}, {NOTE_M2, DUR_SIXTEENTH},
    {NOTE_M1, DUR_QUARTER},
    {NOTE_L7, DUR_EIGHTH}, {NOTE_L6, DUR_HALF},
    /* M7: 6 · 6 6 · 6 · 6· 5  朝朝频顾惜 */
    {NOTE_M6, DUR_QUARTER},
    {NOTE_M6, DUR_EIGHTH}, {NOTE_M6, DUR_EIGHTH},
    {NOTE_M6, DUR_QUARTER},
    {NOTE_M6, DUR_DOTTED_EIGHTH}, {NOTE_M5, DUR_SIXTEENTH},
    /* M8: 3 · 5 5 · #4 · 3-  夜夜不能忘 */
    {NOTE_M3, DUR_QUARTER},
    {NOTE_M5, DUR_EIGHTH}, {NOTE_M5, DUR_EIGHTH},
    {NOTE_M4S, DUR_QUARTER},
    {NOTE_M3, DUR_HALF},
    /* M9: 3 · 6 6 · 5 · 3· 2  但愿花开早 */
    {NOTE_M3, DUR_QUARTER},
    {NOTE_M6, DUR_EIGHTH}, {NOTE_M6, DUR_EIGHTH},
    {NOTE_M5, DUR_QUARTER},
    {NOTE_M3, DUR_DOTTED_EIGHTH}, {NOTE_M2, DUR_SIXTEENTH},
    /* M10: 1· 2 · 1 · 7̲ 6̲ 3̲  能将夙愿偿 */
    {NOTE_M1, DUR_DOTTED_EIGHTH}, {NOTE_M2, DUR_SIXTEENTH},
    {NOTE_M1, DUR_QUARTER},
    {NOTE_L7, DUR_EIGHTH}, {NOTE_L6, DUR_EIGHTH}, {NOTE_L3, DUR_QUARTER},
    /* M11: 3̲· 1 · 1 · 7̲ 6̲ 3̲  满庭花簇簇 */
    {NOTE_L3, DUR_DOTTED_EIGHTH}, {NOTE_M1, DUR_SIXTEENTH},
    {NOTE_M1, DUR_QUARTER},
    {NOTE_L7, DUR_EIGHTH}, {NOTE_L6, DUR_EIGHTH}, {NOTE_L3, DUR_QUARTER},
    /* M12: 2· 1 · 7̲ 5̲ 6̲-  添得许多香 */
    {NOTE_M2, DUR_DOTTED_EIGHTH}, {NOTE_M1, DUR_SIXTEENTH},
    {NOTE_L7, DUR_EIGHTH}, {NOTE_L5, DUR_EIGHTH},
    {NOTE_L6, DUR_HALF},

    /* ── 尾奏 M13–M14 ── */
    /* M13: 3̲· 1 · 1 · 7̲ 6̲ 3̲ */
    {NOTE_L3, DUR_DOTTED_EIGHTH}, {NOTE_M1, DUR_SIXTEENTH},
    {NOTE_M1, DUR_QUARTER},
    {NOTE_L7, DUR_EIGHTH}, {NOTE_L6, DUR_EIGHTH}, {NOTE_L3, DUR_QUARTER},
    /* M14: 2· 1 · 7̲ 5̲ 6̲- */
    {NOTE_M2, DUR_DOTTED_EIGHTH}, {NOTE_M1, DUR_SIXTEENTH},
    {NOTE_L7, DUR_EIGHTH}, {NOTE_L5, DUR_EIGHTH},
    {NOTE_L6, DUR_HALF},

    {NOTE_REST, 0u},
};

/* 每个音符末尾保留的静音间隔(ms)，使相邻（同音高）音符边界可辨 */
#define NOTE_GAP_MS  20u

/* ── 赛道模式短提示音 ─────────────────────────────────────────────────────── */

#define CUE_DUR_SHORT   120u
#define CUE_DUR_MED     200u
#define CUE_DUR_LONG    480u
#define CUE_GAP         70u

static const uint16_t s_cue_stood_up[][2] = {
    {NOTE_M4, CUE_DUR_MED},
    {NOTE_M5, CUE_DUR_MED},
    {NOTE_REST, 0u},
};

static const uint16_t s_cue_trace_start[][2] = {
    {NOTE_M6, CUE_DUR_SHORT},
    {NOTE_REST, CUE_GAP},
    {NOTE_M6, CUE_DUR_SHORT},
    {NOTE_REST, CUE_GAP},
    {NOTE_M6, CUE_DUR_MED},
    {NOTE_REST, 0u},
};

static const uint16_t s_cue_lap_pause[][2] = {
    {NOTE_M5, CUE_DUR_LONG},
    {NOTE_REST, 0u},
};

static const uint16_t s_cue_all_done[][2] = {
    {NOTE_M5, CUE_DUR_MED},
    {NOTE_M6, CUE_DUR_MED},
    {NOTE_M7, CUE_DUR_LONG},
    {NOTE_REST, 0u},
};

static const uint16_t (*const s_cue_seqs[APP_BUZZER_CUE_COUNT])[] = {
    [APP_BUZZER_CUE_STOOD_UP]    = s_cue_stood_up,
    [APP_BUZZER_CUE_TRACE_START] = s_cue_trace_start,
    [APP_BUZZER_CUE_LAP_PAUSE]   = s_cue_lap_pause,
    [APP_BUZZER_CUE_ALL_DONE]    = s_cue_all_done,
};

static const char *const s_cue_names[APP_BUZZER_CUE_COUNT] = {
    [APP_BUZZER_CUE_STOOD_UP]    = "stood_up",
    [APP_BUZZER_CUE_TRACE_START] = "trace_start",
    [APP_BUZZER_CUE_LAP_PAUSE]   = "lap_pause",
    [APP_BUZZER_CUE_ALL_DONE]    = "all_done",
};

/* ── 播放状态 ─────────────────────────────────────────────────────────────── */

static bool                    s_playing;
static uint16_t                s_note_idx;
static uint32_t                s_note_start_ms;
static bool                    s_in_gap;
static const uint16_t        (*s_play_seq)[2];
static const char             *s_play_tag;

/** 状态蓝灯（BSP_LED_B）与蜂鸣器同拍：响=亮，静音=灭。 */
static void status_led_tone(bool on)
{
    if (on) {
        DL_GPIO_setPins(BSP_LED_B_PORT, BSP_LED_B_PIN);
    } else {
        DL_GPIO_clearPins(BSP_LED_B_PORT, BSP_LED_B_PIN);
    }
}

static bool seq_is_end(const uint16_t (*seq)[2], uint16_t idx)
{
    if (seq == NULL) {
        return true;
    }
    return (seq[idx][0] == NOTE_REST) && (seq[idx][1] == 0u);
}

static void apply_seq_note(const uint16_t (*seq)[2], uint16_t idx)
{
    if (seq_is_end(seq, idx)) {
        bsp_buzzer_set_tone_hz(0u);
        status_led_tone(false);
        return;
    }
    bool sounding = (seq[idx][0] != NOTE_REST);
    bsp_buzzer_set_tone_hz(sounding ? seq[idx][0] : 0u);
    status_led_tone(sounding);
}

static void finish_playback(void)
{
    if (!s_playing) {
        return;
    }
    s_playing  = false;
    s_play_seq = NULL;
    s_play_tag = NULL;
    bsp_buzzer_set_tone_hz(0u);
    status_led_tone(false);
    (void)printf("[buzzer] done\r\n");
}

static void start_sequence(const uint16_t (*seq)[2], const char *tag)
{
    if (seq == NULL) {
        return;
    }
    s_playing       = true;
    s_play_seq      = seq;
    s_play_tag      = (tag != NULL) ? tag : "seq";
    s_note_idx      = 0u;
    s_note_start_ms = bsp_systick_get_ms();
    s_in_gap        = false;
    apply_seq_note(s_play_seq, 0u);
    (void)printf("[buzzer] %s start\r\n", s_play_tag);
}

/* ── 公共 API ─────────────────────────────────────────────────────────────── */

void app_buzzer_init(void)
{
    bsp_buzzer_init();
    status_led_tone(false);
    s_playing       = false;
    s_note_idx      = 0u;
    s_note_start_ms = 0u;
    s_in_gap        = false;
    s_play_seq      = NULL;
    s_play_tag      = NULL;
}

void app_buzzer_play_cue(app_buzzer_cue_t cue)
{
    if ((unsigned)cue >= (unsigned)APP_BUZZER_CUE_COUNT) {
        return;
    }
    start_sequence(s_cue_seqs[cue], s_cue_names[cue]);
}

void app_buzzer_play_lanhua_cao(void)
{
    start_sequence(s_lanhua_cao, "lanhua_cao");
}

void app_buzzer_stop(void)
{
    if (!s_playing) {
        bsp_buzzer_set_tone_hz(0u);
        status_led_tone(false);
        return;
    }
    s_playing  = false;
    s_play_seq = NULL;
    s_play_tag = NULL;
    bsp_buzzer_set_tone_hz(0u);
    status_led_tone(false);
    (void)printf("[buzzer] stopped\r\n");
}

bool app_buzzer_is_playing(void)
{
    return s_playing;
}

void app_buzzer_tick_1ms(void)
{
    if (!s_playing || (s_play_seq == NULL)) {
        return;
    }

    if (seq_is_end(s_play_seq, s_note_idx)) {
        finish_playback();
        return;
    }

    uint16_t dur_ms  = s_play_seq[s_note_idx][1];
    uint32_t elapsed = bsp_systick_get_ms() - s_note_start_ms;

    if (elapsed >= (uint32_t)dur_ms) {
        s_note_idx++;
        s_note_start_ms = bsp_systick_get_ms();
        s_in_gap        = false;

        if (seq_is_end(s_play_seq, s_note_idx)) {
            finish_playback();
            return;
        }

        apply_seq_note(s_play_seq, s_note_idx);
        return;
    }

    if (!s_in_gap && (elapsed >= (uint32_t)(dur_ms - NOTE_GAP_MS))) {
        s_in_gap = true;
        bsp_buzzer_set_tone_hz(0u);
        status_led_tone(false);
    }
}
