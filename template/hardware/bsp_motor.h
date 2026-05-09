/**
 * @file    bsp_motor.h
 * @brief   阶段 2 电机底层驱动：TB6612 + 左右轮编码器反馈。
 *
 * 硬件资源（来自 SysConfig 与 docs/TaskLog/Stage0-PinAllocation.md）：
 *   ─ PWM：TIMA0 CCP0/CCP1 → PA8 (PWMA) / PA9 (PWMB)，BUSCLK 32 MHz、
 *      period = 1599 → PWM 频率 = 32 MHz / 1600 ≈ 20 kHz（超出人耳范围）
 *   ─ 方向：PA15/PA16 (AIN1/AIN2)、PA26/PA27 (BIN1/BIN2)
 *   ─ STBY：PB0（高有效；上电默认低 → TB6612 待机 / 输出 Hi-Z）
 *   ─ 左轮编码器：TIMG8 硬件 QEI（A=PA29、B=PA30），mode 3 = X4 解码，
 *      16-bit 计数器（LOAD = 65535）由 `bsp_motor_update()` 软件扩为 32-bit
 *   ─ 右轮编码器：PA12 双边沿中断 + PA13 ISR 内电平判方向（X2 解码）
 *   ─ 板载按键 S1：PA18，下降沿中断 + 80 ms 软件去抖
 *
 * 接口约定：
 *   ─ 速度命令使用 permille（千分比），范围 [-1000, 1000]，
 *     正值正转、负值反转、0 = 滑行 (coast，方向位清零、PWM = 0)
 *   ─ 角度按 GB370 常见 11 PPR、30:1 减速比估算；如实物参数不同，
 *     仅需改本文件顶部的编码器宏，不要改业务代码
 *   ─ 速度反馈基于 update() 周期内的滑动窗口差分（默认 20 ms 窗口 → 50 Hz 速度刷新率）
 *
 * 调用次序（参考 main.c）：
 *   SYSCFG_DL_init()                     -- SDK 自动生成
 *   bsp_gpio_init()                      -- 14 路业务 GPIO 手工 init
 *   bsp_systick_init(1000)               -- 1 kHz tick
 *   bsp_motor_init()                     -- 本模块；ISR 注册 / 计数清零 / STBY 保持低
 *   bsp_motor_enable(true)               -- 拉高 STBY，电机才会真正动
 *   while (1) {
 *       if (bsp_systick_consume_tick()) { bsp_motor_update(); ... }
 *       bsp_motor_set_output(...)       -- 业务命令
 *       bsp_motor_get_feedback(&fb)     -- 反馈
 *   }
 */

#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/* 编译期可配宏                                                                 */
/* ========================================================================== */

/** PWM 命令满量程千分比（不要改） */
#define BSP_MOTOR_PWM_MAX_PERMILLE                 (1000)

/** GB370 减速箱减速比（实物为多少改多少） */
#define BSP_MOTOR_GB370_GEAR_RATIO                 (30)

/** GB370 内置霍尔每电机轴转脉冲数（A 相单沿） */
#define BSP_MOTOR_GB370_HALL_PPR                   (11)

/** 左轮 QEI mode 3 = X4 解码，每输出轴一圈的计数 */
#define BSP_MOTOR_LEFT_DECODE_X                    (4)
/** 右轮 PA12 双边沿中断 = X2 解码 */
#define BSP_MOTOR_RIGHT_DECODE_X                   (2)

#define BSP_MOTOR_LEFT_COUNTS_PER_OUTPUT_REV \
    (BSP_MOTOR_GB370_GEAR_RATIO * BSP_MOTOR_GB370_HALL_PPR * BSP_MOTOR_LEFT_DECODE_X)
#define BSP_MOTOR_RIGHT_COUNTS_PER_OUTPUT_REV \
    (BSP_MOTOR_GB370_GEAR_RATIO * BSP_MOTOR_GB370_HALL_PPR * BSP_MOTOR_RIGHT_DECODE_X)

/**
 * 速度差分窗口（毫秒）：update() 每次累计 1 ms，达到该值时做一次 count 差分，
 * 得到 cps（counts per second）；窗口越小响应越快，但低速时分辨率越粗。
 * 默认 20 ms ⇒ 50 Hz 速度刷新率，最低可分辨速度
 *      左轮 = 1000/20 = 50 cps  ≈ 50/1320 * 60 ≈ 2.27 rpm
 *      右轮 = 50 cps             ≈ 50/660  * 60 ≈ 4.55 rpm
 */
#ifndef BSP_MOTOR_SPEED_WINDOW_MS
#define BSP_MOTOR_SPEED_WINDOW_MS                  (20u)
#endif

/** S1 按键去抖窗口（毫秒） */
#ifndef BSP_MOTOR_BTN_DEBOUNCE_MS
#define BSP_MOTOR_BTN_DEBOUNCE_MS                  (80u)
#endif

/* ========================================================================== */
/* 反馈结构体                                                                   */
/*   `*_count`         编码器累计计数（int32，约 ±5×10^5 圈不溢出）              */
/*   `*_angle_deg`     输出轴累计机械角（计数 / 每圈计数 × 360°）                */
/*   `*_speed_cps`     输出轴瞬时角速度，单位 counts/s（差分窗口决定刷新率）     */
/*   `*_speed_dps`     输出轴瞬时角速度，单位 °/s                               */
/*   `*_speed_rpm`     输出轴瞬时转速，单位 rpm                                 */
/* ========================================================================== */
typedef struct {
    int32_t left_count;
    int32_t right_count;

    float   left_angle_deg;
    float   right_angle_deg;

    int32_t left_speed_cps;
    int32_t right_speed_cps;

    float   left_speed_dps;
    float   right_speed_dps;

    float   left_speed_rpm;
    float   right_speed_rpm;
} bsp_motor_feedback_t;

/* ========================================================================== */
/* 初始化 / 使能                                                                */
/* ========================================================================== */

/**
 * @brief 初始化电机驱动：清零计数与命令、STBY 拉低（待机）、注册 PA12/PA18 中断。
 *
 * 前提：`bsp_gpio_init()` 已配好 AIN/BIN/STBY/PA12/PA13/PA18 的方向与上拉，
 *       `SYSCFG_DL_init()` 已配好 PWM (TIMA0) 与左轮 QEI (TIMG8)。
 */
void bsp_motor_init(void);

/**
 * @brief 拉高/拉低 STBY 引脚，控制 TB6612 整体使能。
 *        即使 PWM/方向位已就绪，STBY = 0 时电机也不会动；上电默认 0 = 待机。
 */
void bsp_motor_enable(bool enable);

/** 查询 STBY 当前是否拉高（true = 已使能；不一定有 PWM 输出） */
bool bsp_motor_is_enabled(void);

/* ========================================================================== */
/* 速度命令                                                                     */
/* ========================================================================== */

/**
 * @brief 同时设置左右轮速度命令。
 * @param left_permille  [-1000, 1000]，正 = 正转、负 = 反转、0 = 滑行
 * @param right_permille 同上
 *
 * 软件极性翻转（`bsp_motor_set_invert`）会在写入硬件前应用一次。
 */
void bsp_motor_set_output(int16_t left_permille, int16_t right_permille);

/** 仅更新左轮命令，保持右轮不变 */
void bsp_motor_set_left(int16_t left_permille);

/** 仅更新右轮命令，保持左轮不变 */
void bsp_motor_set_right(int16_t right_permille);

/**
 * @brief Coast (滑行) 停止：方向位清零、PWM 占空比 0、STBY 保持原状态。
 *        电机在反电动势衰减下慢慢停下；适合常规减速。
 */
void bsp_motor_stop(void);

/**
 * @brief Brake (短刹车) 停止：AIN1=AIN2=1 / BIN1=BIN2=1、PWM = 满，
 *        TB6612 内部把电机两端短接 → 反电动势制动，停车比 stop 快但电流冲击大。
 *        STBY 保持原状态。仅在需要快速停车（如跌倒保护）时用。
 */
void bsp_motor_brake(void);

/* ========================================================================== */
/* 极性 / 限幅（运行时可调，省去重新编译）                                       */
/* ========================================================================== */

/**
 * @brief 软件方向反转：true → 后续命令 permille 正负与硬件方向位互换。
 *        装车后若发现某轮方向反了，调用一次即可，不必改动力线或方向位真值表。
 *        函数返回前会立刻按新极性重发当前命令，避免"已调极性但旧命令仍在跑"
 *        的危险窗口。注意：极性只影响命令侧，反馈侧 (count / speed) 仍按
 *        编码器物理方向计数，调用方如需让反馈也跟随业务正向，请自行乘 -1。
 */
void bsp_motor_set_invert(bool invert_left, bool invert_right);

/** 查询当前左 / 右轮极性翻转标志 */
void bsp_motor_get_invert(bool *invert_left, bool *invert_right);

/**
 * @brief 设置 PWM 占空比上限钳位，[0, 1000]。
 *        所有命令在送进硬件前都会被钳到 ±limit 之内，用于安全保护
 *        （如电池低压时降功率、调试时限速等）。
 *        默认值 = BSP_MOTOR_PWM_MAX_PERMILLE。
 */
void bsp_motor_set_pwm_limit(uint16_t limit_permille);

/** 查询当前 PWM 限幅值 */
uint16_t bsp_motor_get_pwm_limit(void);

/* ========================================================================== */
/* 当前命令查询（只读最近一次写入）                                              */
/* ========================================================================== */

/** 当前左轮命令 permille（已应用极性翻转与限幅之前的值） */
int16_t bsp_motor_get_left_cmd(void);

/** 当前右轮命令 permille */
int16_t bsp_motor_get_right_cmd(void);

/* ========================================================================== */
/* 反馈与编码器                                                                 */
/* ========================================================================== */

/**
 * @brief 1 kHz 周期任务：读 QEI 计数 → 软件扩 32-bit、累计速度窗口。
 *        必须在主循环里以 ≈ 1 kHz 频率调用（典型 `bsp_systick_consume_tick()`）；
 *        ISR 内部不调用此函数，所有浮点运算延迟到 `bsp_motor_get_feedback()`。
 */
void bsp_motor_update(void);

/**
 * @brief 一次性快照所有反馈字段（线程安全，对 ISR 共享变量做了关中断保护）。
 *        浮点角度 / dps / rpm 在本函数内现算；调用方按业务节拍（如 100 ms / 10 ms）
 *        触发即可，无需在 1 kHz 路径里调用。
 *
 * @param feedback  输出结构体指针；NULL 时函数静默返回。
 */
void bsp_motor_get_feedback(bsp_motor_feedback_t *feedback);

/** 单独读左轮累计计数（int32，原子读） */
int32_t bsp_motor_get_left_count(void);

/** 单独读右轮累计计数（int32，原子读） */
int32_t bsp_motor_get_right_count(void);

/**
 * @brief 同时清零左右轮累计计数与速度差分窗口。常用于上电校准、调试归零、
 *        或上一次跑车结束后准备下一次试跑。不影响命令与 STBY 状态。
 */
void bsp_motor_reset_encoders(void);

/* ========================================================================== */
/* 板载按键 S1：上一次按下后是否有未消费的请求？                                 */
/* ========================================================================== */

/**
 * @brief 取出 S1(PA18) 按键中断置位的 toggle 请求。读后自动清零。
 *        典型用法：业务层把它当作 "正反转切换 / 模式切换" 的边沿事件源。
 *        80 ms 软件去抖在 ISR 内已做。
 *
 * @return true  = 上一次调用以来 S1 至少按下过一次
 *         false = 没有新事件
 */
bool bsp_motor_consume_toggle_request(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_MOTOR_H */
