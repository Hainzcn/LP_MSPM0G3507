/**
 * @file    bsp_motor.c
 * @brief   阶段 2 电机底层驱动实现 —— TB6612 + 左右编码器 + S1 切换。
 *
 * 内部状态聚合到 `s_motor` 单例：
 *   ─ ISR 共享字段（counts / toggle 请求 / 上次按键时间戳）放前面，访问
 *     必须用 `MOTOR_LOCK / MOTOR_UNLOCK` 一对宏关 PRIMASK；
 *   ─ 主循环私有字段（speed window / cmd / 极性 / 限幅）放后面，无需关中断。
 *
 * 1 kHz 路径（update）只做整数加减；浮点换算（角度 / dps / rpm）一律放到
 * `bsp_motor_get_feedback()`，由业务层按 10~100 ms 节拍触发即可，
 * 避免 Cortex-M0+ 软浮点拖累 SysTick 周期。
 */

#include "bsp_motor.h"

#include "bsp_gpio.h"
#include "bsp_systick.h"
#include "ti_msp_dl_config.h"

#include <stddef.h>

/* ========================================================================== */
/* 内部常量 / 助记                                                              */
/* ========================================================================== */

#define MOTOR_LOCK()        __disable_irq()
#define MOTOR_UNLOCK()      __enable_irq()

#define LEFT_DEG_PER_COUNT   (360.0f / (float)BSP_MOTOR_LEFT_COUNTS_PER_OUTPUT_REV)
#define RIGHT_DEG_PER_COUNT  (360.0f / (float)BSP_MOTOR_RIGHT_COUNTS_PER_OUTPUT_REV)

/* ========================================================================== */
/* 模块状态                                                                     */
/* ========================================================================== */

typedef struct {
    /* --- ISR 共享字段：访问必须 MOTOR_LOCK ----------------------------------- */
    volatile int32_t  left_count;       /* 由 update() 维护（QEI 软扩） */
    volatile int32_t  right_count;      /* 由 PA12 ISR 维护 */
    volatile uint8_t  toggle_request;   /* S1 按键置位、消费时清零 */
    volatile uint32_t last_button_ms;   /* 用于 80 ms 去抖 */

    /* --- update() 私有：QEI 16-bit 上一拍原值 -------------------------------- */
    uint16_t left_raw_prev;

    /* --- 速度差分窗口（主循环私有） ------------------------------------------ */
    int32_t  left_speed_prev_count;
    int32_t  right_speed_prev_count;
    uint32_t speed_window_acc_ms;
    int32_t  left_speed_cps;
    int32_t  right_speed_cps;

    /* --- 命令与配置 --------------------------------------------------------- */
    int16_t  left_cmd_pm;
    int16_t  right_cmd_pm;
    uint16_t pwm_limit_pm;
    bool     invert_left;
    bool     invert_right;
    bool     enabled;
} motor_state_t;

static motor_state_t s_motor;

/* ========================================================================== */
/* 内部辅助                                                                     */
/* ========================================================================== */

static inline int32_t clip_int(int32_t v, int32_t lo, int32_t hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/** 将 permille 命令限幅到 [-pwm_limit, +pwm_limit] 并返回。 */
static int16_t apply_limit(int16_t permille)
{
    int32_t lim = (int32_t)s_motor.pwm_limit_pm;
    if (lim > BSP_MOTOR_PWM_MAX_PERMILLE) {
        lim = BSP_MOTOR_PWM_MAX_PERMILLE;
    }
    return (int16_t)clip_int((int32_t)permille, -lim, lim);
}

/** 取 permille 的绝对值（已假设输入已经限幅，安全无溢出）。 */
static uint32_t abs_permille(int16_t permille)
{
    return (uint32_t)((permille >= 0) ? permille : -(int32_t)permille);
}

/**
 * 设置 TIMA0 某 CCP 通道的 compare：
 *   compare = period - period * duty / 1000   (INIT_VAL_LOW，故 duty 段在高位)
 *   duty=1000 → compare=0   (一直高 = 100% 占空)
 *   duty=0    → compare=load (一直低 = 0% 占空)
 */
static void set_pwm_duty(uint32_t channel_idx, uint32_t permille)
{
    uint32_t load    = DL_TimerA_getLoadValue(PWM_MOTOR_INST);
    uint32_t compare = load - ((load * permille) / BSP_MOTOR_PWM_MAX_PERMILLE);
    if (compare > load) {
        compare = load;
    }
    DL_TimerA_setCaptureCompareValue(PWM_MOTOR_INST, compare, channel_idx);
}

/* TB6612 真值表（PWMx 提供占空、IN1/IN2 决定方向 / 制动 / 滑行）：
 *   IN1=L IN2=L           → 滑行 (Coast / Hi-Z)
 *   IN1=H IN2=L PWMx=H/L  → 正转
 *   IN1=L IN2=H PWMx=H/L  → 反转
 *   IN1=H IN2=H           → 短刹车 (Brake)
 */
typedef enum {
    DIR_COAST    = 0,
    DIR_FORWARD  = 1,
    DIR_REVERSE  = 2,
    DIR_BRAKE    = 3
} tb6612_dir_t;

static void set_dir_left(tb6612_dir_t d)
{
    switch (d) {
    case DIR_FORWARD:
        DL_GPIO_setPins  (BSP_AIN1_PORT, BSP_AIN1_PIN);
        DL_GPIO_clearPins(BSP_AIN2_PORT, BSP_AIN2_PIN);
        break;
    case DIR_REVERSE:
        DL_GPIO_clearPins(BSP_AIN1_PORT, BSP_AIN1_PIN);
        DL_GPIO_setPins  (BSP_AIN2_PORT, BSP_AIN2_PIN);
        break;
    case DIR_BRAKE:
        DL_GPIO_setPins  (BSP_AIN1_PORT, BSP_AIN1_PIN | BSP_AIN2_PIN);
        break;
    case DIR_COAST:
    default:
        DL_GPIO_clearPins(BSP_AIN1_PORT, BSP_AIN1_PIN | BSP_AIN2_PIN);
        break;
    }
}

static void set_dir_right(tb6612_dir_t d)
{
    switch (d) {
    case DIR_FORWARD:
        DL_GPIO_setPins  (BSP_BIN1_PORT, BSP_BIN1_PIN);
        DL_GPIO_clearPins(BSP_BIN2_PORT, BSP_BIN2_PIN);
        break;
    case DIR_REVERSE:
        DL_GPIO_clearPins(BSP_BIN1_PORT, BSP_BIN1_PIN);
        DL_GPIO_setPins  (BSP_BIN2_PORT, BSP_BIN2_PIN);
        break;
    case DIR_BRAKE:
        DL_GPIO_setPins  (BSP_BIN1_PORT, BSP_BIN1_PIN | BSP_BIN2_PIN);
        break;
    case DIR_COAST:
    default:
        DL_GPIO_clearPins(BSP_BIN1_PORT, BSP_BIN1_PIN | BSP_BIN2_PIN);
        break;
    }
}

/**
 * 把"经过限幅的 permille"应用到一路输出：
 *   ─ 极性翻转 (invert) 在调用方先做完
 *   ─ permille = 0 → 滑行
 *   ─ permille ≠ 0 → 设方向 + 占空
 */
static void apply_one_channel(int16_t permille_after_invert,
                              bool is_left)
{
    tb6612_dir_t dir;
    if (permille_after_invert > 0) {
        dir = DIR_FORWARD;
    } else if (permille_after_invert < 0) {
        dir = DIR_REVERSE;
    } else {
        dir = DIR_COAST;
    }

    if (is_left) {
        set_dir_left(dir);
        set_pwm_duty(GPIO_PWM_MOTOR_C0_IDX, abs_permille(permille_after_invert));
    } else {
        set_dir_right(dir);
        set_pwm_duty(GPIO_PWM_MOTOR_C1_IDX, abs_permille(permille_after_invert));
    }
}

/** 命令 → 物理输出（限幅 + 极性 + 写硬件） */
static void commit_left(int16_t cmd_pm)
{
    int16_t pm = apply_limit(cmd_pm);
    if (s_motor.invert_left) {
        pm = (int16_t)(-(int32_t)pm);
    }
    apply_one_channel(pm, true);
}

static void commit_right(int16_t cmd_pm)
{
    int16_t pm = apply_limit(cmd_pm);
    if (s_motor.invert_right) {
        pm = (int16_t)(-(int32_t)pm);
    }
    apply_one_channel(pm, false);
}

/* ========================================================================== */
/* 中断辅助                                                                     */
/* ========================================================================== */

/**
 * 右编码器 PA12 双沿中断触发。X2 解码：
 *   每次触发都计 1 步，方向由 PA13 与 PA12 当前电平异同判定。
 *   不开 PA13 中断（CPU 负担减半，分辨率 = 1320 / 2 = 660 cnt/rev）。
 */
static void on_right_encoder_edge(void)
{
    uint32_t pins = DL_GPIO_readPins(GPIOA, BSP_ENC_R_A_PIN | BSP_ENC_R_B_PIN);
    bool phase_a = ((pins & BSP_ENC_R_A_PIN) != 0u);
    bool phase_b = ((pins & BSP_ENC_R_B_PIN) != 0u);

    /* A == B → 计 +1，否则 -1（极性可被 invert_right 二次翻转，但在 ISR 里不算
     * 极性 —— 极性是命令侧的事，反馈侧保持原始物理符号，调用方按需自行处理）。 */
    s_motor.right_count += (phase_a != phase_b) ? 1 : -1;
}

static void on_start_button_edge(void)
{
    uint32_t now_ms = bsp_systick_get_ms();
    if ((now_ms - s_motor.last_button_ms) < BSP_MOTOR_BTN_DEBOUNCE_MS) {
        return;
    }
    s_motor.last_button_ms = now_ms;
    s_motor.toggle_request = 1u;
}

/* ========================================================================== */
/* 公共 API：初始化 / 使能                                                      */
/* ========================================================================== */

void bsp_motor_init(void)
{
    /* --- 1) 状态清零 ------------------------------------------------------- */
    s_motor.left_count             = 0;
    s_motor.right_count            = 0;
    s_motor.toggle_request         = 0u;
    s_motor.last_button_ms         = 0u;

    s_motor.left_raw_prev          = (uint16_t)DL_TimerG_getTimerCount(QEI_LEFT_INST);

    s_motor.left_speed_prev_count  = 0;
    s_motor.right_speed_prev_count = 0;
    s_motor.speed_window_acc_ms    = 0u;
    s_motor.left_speed_cps         = 0;
    s_motor.right_speed_cps        = 0;

    s_motor.left_cmd_pm            = 0;
    s_motor.right_cmd_pm           = 0;
    s_motor.pwm_limit_pm           = BSP_MOTOR_PWM_MAX_PERMILLE;
    s_motor.invert_left            = false;
    s_motor.invert_right           = false;
    s_motor.enabled                = false;

    /* --- 2) 安全态：方向位清零 + PWM 占空 0 + STBY 拉低 -------------------- */
    set_dir_left (DIR_COAST);
    set_dir_right(DIR_COAST);
    set_pwm_duty(GPIO_PWM_MOTOR_C0_IDX, 0u);
    set_pwm_duty(GPIO_PWM_MOTOR_C1_IDX, 0u);
    DL_GPIO_clearPins(BSP_STBY_PORT, BSP_STBY_PIN);

    /* --- 3) PA12 双沿 + PA18 下降沿中断（PA13 不开中断，仅 ISR 内读电平） --
     *   双沿：单独用 _EDGE_RISE 与 _EDGE_FALL 按位或；某些 SDK 版本没有
     *   `_EDGE_RISE_FALL` 这个组合宏，按位或写法在所有 SDK 里都成立。 */
    DL_GPIO_setLowerPinsPolarity(GPIOA,
        DL_GPIO_PIN_12_EDGE_RISE | DL_GPIO_PIN_12_EDGE_FALL);
    DL_GPIO_setUpperPinsPolarity(GPIOA,
        DL_GPIO_PIN_18_EDGE_FALL);
    DL_GPIO_clearInterruptStatus(GPIOA, BSP_ENC_R_A_PIN | BSP_START_BTN_PIN);
    DL_GPIO_enableInterrupt    (GPIOA, BSP_ENC_R_A_PIN | BSP_START_BTN_PIN);
    NVIC_ClearPendingIRQ(GPIOA_INT_IRQn);
    NVIC_EnableIRQ      (GPIOA_INT_IRQn);
}

void bsp_motor_enable(bool enable)
{
    s_motor.enabled = enable;
    if (enable) {
        DL_GPIO_setPins(BSP_STBY_PORT, BSP_STBY_PIN);
    } else {
        DL_GPIO_clearPins(BSP_STBY_PORT, BSP_STBY_PIN);
    }
}

bool bsp_motor_is_enabled(void)
{
    return s_motor.enabled;
}

/* ========================================================================== */
/* 公共 API：速度命令                                                           */
/* ========================================================================== */

void bsp_motor_set_output(int16_t left_permille, int16_t right_permille)
{
    s_motor.left_cmd_pm  = left_permille;
    s_motor.right_cmd_pm = right_permille;
    commit_left (left_permille);
    commit_right(right_permille);
}

void bsp_motor_set_left(int16_t left_permille)
{
    s_motor.left_cmd_pm = left_permille;
    commit_left(left_permille);
}

void bsp_motor_set_right(int16_t right_permille)
{
    s_motor.right_cmd_pm = right_permille;
    commit_right(right_permille);
}

void bsp_motor_stop(void)
{
    s_motor.left_cmd_pm  = 0;
    s_motor.right_cmd_pm = 0;
    set_dir_left (DIR_COAST);
    set_dir_right(DIR_COAST);
    set_pwm_duty(GPIO_PWM_MOTOR_C0_IDX, 0u);
    set_pwm_duty(GPIO_PWM_MOTOR_C1_IDX, 0u);
}

void bsp_motor_brake(void)
{
    s_motor.left_cmd_pm  = 0;
    s_motor.right_cmd_pm = 0;
    set_dir_left (DIR_BRAKE);
    set_dir_right(DIR_BRAKE);
    set_pwm_duty(GPIO_PWM_MOTOR_C0_IDX, BSP_MOTOR_PWM_MAX_PERMILLE);
    set_pwm_duty(GPIO_PWM_MOTOR_C1_IDX, BSP_MOTOR_PWM_MAX_PERMILLE);
}

/* ========================================================================== */
/* 公共 API：极性 / 限幅                                                        */
/* ========================================================================== */

void bsp_motor_set_invert(bool invert_left, bool invert_right)
{
    s_motor.invert_left  = invert_left;
    s_motor.invert_right = invert_right;
    /* 立刻让新极性生效，避免"已调 invert 但旧命令还在按老方向跑"的窗口 */
    commit_left (s_motor.left_cmd_pm);
    commit_right(s_motor.right_cmd_pm);
}

void bsp_motor_get_invert(bool *invert_left, bool *invert_right)
{
    if (invert_left  != NULL) *invert_left  = s_motor.invert_left;
    if (invert_right != NULL) *invert_right = s_motor.invert_right;
}

void bsp_motor_set_pwm_limit(uint16_t limit_permille)
{
    if (limit_permille > BSP_MOTOR_PWM_MAX_PERMILLE) {
        limit_permille = BSP_MOTOR_PWM_MAX_PERMILLE;
    }
    s_motor.pwm_limit_pm = limit_permille;
    /* 立刻应用到当前命令上，避免"调小限幅但旧命令仍在跑"的危险窗口 */
    commit_left (s_motor.left_cmd_pm);
    commit_right(s_motor.right_cmd_pm);
}

uint16_t bsp_motor_get_pwm_limit(void)
{
    return s_motor.pwm_limit_pm;
}

/* ========================================================================== */
/* 公共 API：命令查询                                                           */
/* ========================================================================== */

int16_t bsp_motor_get_left_cmd(void)
{
    return s_motor.left_cmd_pm;
}

int16_t bsp_motor_get_right_cmd(void)
{
    return s_motor.right_cmd_pm;
}

/* ========================================================================== */
/* 公共 API：编码器 / update / feedback                                         */
/* ========================================================================== */

void bsp_motor_update(void)
{
    /* --- 1) 左轮 QEI 16-bit → 32-bit 软扩 --------------------------------- */
    uint16_t raw_now = (uint16_t)DL_TimerG_getTimerCount(QEI_LEFT_INST);
    int16_t  delta   = (int16_t)((uint16_t)(raw_now - s_motor.left_raw_prev));
    s_motor.left_raw_prev = raw_now;
    /* left_count 与 right_count 是 ISR 共享变量，但 update 只在主循环里跑、
     * 而 PA12 ISR 只摸 right_count；左路写入这里其实无 race，无需关中断。
     * 为了未来可能新增"在 ISR 里读 left_count"的场景，仍然短锁一下保平安。 */
    MOTOR_LOCK();
    s_motor.left_count += (int32_t)delta;
    MOTOR_UNLOCK();

    /* --- 2) 速度窗口累计；满 N ms 做一次差分，得到 cps --------------------- */
    s_motor.speed_window_acc_ms += 1u;
    if (s_motor.speed_window_acc_ms >= BSP_MOTOR_SPEED_WINDOW_MS) {
        int32_t left_now;
        int32_t right_now;

        MOTOR_LOCK();
        left_now  = s_motor.left_count;
        right_now = s_motor.right_count;
        MOTOR_UNLOCK();

        int32_t dl = left_now  - s_motor.left_speed_prev_count;
        int32_t dr = right_now - s_motor.right_speed_prev_count;

        /* counts/ms × 1000 = counts/s。Window 满量程 20 ms，dl/dr 均不会
         * 超过 ±50 万（实际机械极限远小于此），int32 安全。 */
        s_motor.left_speed_cps  = (dl * 1000) / (int32_t)s_motor.speed_window_acc_ms;
        s_motor.right_speed_cps = (dr * 1000) / (int32_t)s_motor.speed_window_acc_ms;

        s_motor.left_speed_prev_count  = left_now;
        s_motor.right_speed_prev_count = right_now;
        s_motor.speed_window_acc_ms    = 0u;
    }
}

void bsp_motor_get_feedback(bsp_motor_feedback_t *feedback)
{
    if (feedback == NULL) {
        return;
    }

    int32_t left_count;
    int32_t right_count;
    int32_t left_speed_cps;
    int32_t right_speed_cps;

    MOTOR_LOCK();
    left_count       = s_motor.left_count;
    right_count      = s_motor.right_count;
    left_speed_cps   = s_motor.left_speed_cps;
    right_speed_cps  = s_motor.right_speed_cps;
    MOTOR_UNLOCK();

    feedback->left_count        = left_count;
    feedback->right_count       = right_count;
    feedback->left_speed_cps    = left_speed_cps;
    feedback->right_speed_cps   = right_speed_cps;

    /* 浮点换算只在调用方需要时算，1 kHz update 路径里不动浮点 */
    feedback->left_angle_deg    = (float)left_count       * LEFT_DEG_PER_COUNT;
    feedback->right_angle_deg   = (float)right_count      * RIGHT_DEG_PER_COUNT;
    feedback->left_speed_dps    = (float)left_speed_cps   * LEFT_DEG_PER_COUNT;
    feedback->right_speed_dps   = (float)right_speed_cps  * RIGHT_DEG_PER_COUNT;
    feedback->left_speed_rpm    = feedback->left_speed_dps  / 6.0f;   /* dps/360 * 60 */
    feedback->right_speed_rpm   = feedback->right_speed_dps / 6.0f;
}

int32_t bsp_motor_get_left_count(void)
{
    int32_t v;
    MOTOR_LOCK();
    v = s_motor.left_count;
    MOTOR_UNLOCK();
    return v;
}

int32_t bsp_motor_get_right_count(void)
{
    int32_t v;
    MOTOR_LOCK();
    v = s_motor.right_count;
    MOTOR_UNLOCK();
    return v;
}

void bsp_motor_reset_encoders(void)
{
    MOTOR_LOCK();
    s_motor.left_count             = 0;
    s_motor.right_count            = 0;
    s_motor.left_speed_prev_count  = 0;
    s_motor.right_speed_prev_count = 0;
    s_motor.left_speed_cps         = 0;
    s_motor.right_speed_cps        = 0;
    s_motor.speed_window_acc_ms    = 0u;
    /* 让下次 update() 把当前 16-bit raw 当作 0 起点，避免一次性吃进上一段差值 */
    s_motor.left_raw_prev = (uint16_t)DL_TimerG_getTimerCount(QEI_LEFT_INST);
    MOTOR_UNLOCK();
}

/* ========================================================================== */
/* 公共 API：S1 toggle 请求                                                     */
/* ========================================================================== */

bool bsp_motor_consume_toggle_request(void)
{
    bool pending;
    MOTOR_LOCK();
    pending = (s_motor.toggle_request != 0u);
    s_motor.toggle_request = 0u;
    MOTOR_UNLOCK();
    return pending;
}

/* ========================================================================== */
/* GPIOA 中断入口（PA12 = 右编码器 A 双沿；PA18 = S1 下降沿）                    */
/* ========================================================================== */

void GPIOA_IRQHandler(void)
{
    DL_GPIO_IIDX pending;

    do {
        pending = DL_GPIO_getPendingInterrupt(GPIOA);

        if (pending == DL_GPIO_IIDX_DIO12) {
            DL_GPIO_clearInterruptStatus(GPIOA, BSP_ENC_R_A_PIN);
            on_right_encoder_edge();
        } else if (pending == DL_GPIO_IIDX_DIO18) {
            DL_GPIO_clearInterruptStatus(GPIOA, BSP_START_BTN_PIN);
            on_start_button_edge();
        }
    } while (pending != DL_GPIO_IIDX_NO_INTR);
}
