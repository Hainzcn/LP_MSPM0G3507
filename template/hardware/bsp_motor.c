/**
 * @file    bsp_motor.c
 * @brief   电机底层驱动实现 —— TB6612 + 左右编码器 + S1 按键
 *
 * ============================================================
 * 这个文件是做什么的？（写给初学者）
 * ============================================================
 * bsp_motor.h 定义了"电机驱动有哪些功能"（接口），
 * 而这个 bsp_motor.c 是"具体怎么实现这些功能"（实现）。
 *
 * 这个文件非常长（近 800 行），因为它做了很多事：
 *   1. 控制 TB6612 驱动芯片（方向引脚 + PWM）
 *   2. 读取左轮编码器（硬件 QEI，16→32 位扩展）
 *   3. 读取右轮编码器（GPIO 中断软件解码）
 *   4. 计算速度（滑动窗口差分法）
 *   5. 检测 S1 启动按键（中断 + 轮询双保险）
 *   6. 中断雪崩保护（防止浮空引脚导致 CPU 锁死）
 *   7. 脉冲刹车（brake N ms 后自动转 coast）
 *
 * 内部状态全部放在 s_motor 这个结构体中，方便管理。
 *
 * ============================================================
 * 文件结构
 * ============================================================
 * 1. 头文件包含 + 内部常量
 * 2. 模块状态结构体 motor_state_t + s_motor 实例
 * 3. 内部辅助函数（clip_int / apply_limit / set_pwm_duty / TB6612 方向控制）
 * 4. 中断辅助函数（右编码器解码 / S1 按键处理）
 * 5. 公开 API 实现（初始化/使能/命令/极性/更新/反馈/按键）
 * 6. GROUP1_IRQHandler（GPIO 中断服务函数）
 */

/* ============================================================
 * 头文件包含
 * ============================================================ */
#include "bsp_motor.h"

#include "bsp_gpio.h"        /* 电机方向引脚（AIN1/BIN1/STBY 等） */
#include "bsp_systick.h"     /* 获取时间戳用于按键去抖 */
#include "ti_msp_dl_config.h"/* SysConfig 配置：QEI_LEFT_INST、PWM_MOTOR_INST 等 */
#include <stddef.h>          /* NULL 宏定义 */

/* ================================================================
 * 内部常量 / 助记宏
 * ================================================================ */

/** MOTOR_LOCK / MOTOR_UNLOCK：关/开全局中断（保护 ISR 共享变量）。
 *  凡是 s_motor 中被 ISR 修改的字段（left_count, right_count 等），
 *  主循环在读写它们时必须关中断，防止 ISR 在读写过程中插进来修改。 */
#define MOTOR_LOCK()        __disable_irq()
#define MOTOR_UNLOCK()      __enable_irq()

/** 每个编码器计数对应的角度（度/计数）。用于把 count 换算为角度。 */
#define LEFT_DEG_PER_COUNT   (360.0f / (float)BSP_MOTOR_LEFT_COUNTS_PER_OUTPUT_REV)
#define RIGHT_DEG_PER_COUNT  (360.0f / (float)BSP_MOTOR_RIGHT_COUNTS_PER_OUTPUT_REV)

/* ================================================================
 * 模块状态结构体 + 全局实例
 * ================================================================
 * 整个模块的所有状态集中在一个结构体 s_motor 中。
 *
 * 结构体中字段的排列顺序有讲究：
 *   - 前面放 ISR 共享字段（需要 MOTOR_LOCK 保护）
 *   - 后面放主循环私有字段（不需要关中断）
 *
 * 这种"把相关的东西放在一起"的做法叫"内聚"——提高了代码的可维护性。
 */

typedef struct {
    /* ================================================================
     * ISR 共享字段 —— 访问时必须用 MOTOR_LOCK/MOTOR_UNLOCK 保护
     * ================================================================
     * volatile 修饰：告诉编译器"这个变量会被 ISR 修改"，
     * 不要做"这个变量在循环中不变"的优化假设。 */

    volatile int32_t  left_count;       /* 左轮编码器累计计数（由 update() 维护，QEI 软扩） */
    volatile int32_t  right_count;      /* 右轮编码器累计计数（由 PA12/PA13 ISR 维护） */
    volatile uint8_t  toggle_request;   /* S1 按键事件标志（ISR 置位，consume 时清零） */
    volatile uint32_t last_button_ms;   /* 上次按键时间戳（用于 80 ms 去抖） */
    volatile uint32_t button_irq_count; /* S1 中断命中次数统计 */
    volatile uint32_t button_poll_count;/* S1 轮询兜底命中次数统计 */
    volatile uint32_t enc_irq_count;    /* 右编码器 ISR 总进入次数（雪崩诊断用） */
    volatile uint32_t enc_irq_window;   /* 当前 1 ms 窗口内 ISR 进入次数（雪崩检测用） */
    volatile uint8_t  enc_irq_quench_remain_ms; /* 雪崩抑制剩余毫秒数（0=未激活） */

    /* ================================================================
     * 主循环私有字段 —— 不需要关中断保护
     * ================================================================
     * 这些字段只在 bsp_motor_update() 中访问（主循环上下文），
     * ISR 不碰它们，所以不需要 MOTOR_LOCK。 */

    uint16_t left_raw_prev;  /* 左轮 QEI 16-bit 上一拍原始值（用于 16→32 扩展） */

    /* 速度差分窗口 */
    int32_t  left_speed_prev_count;   /* 上一拍左轮 count（用于计算差值） */
    int32_t  right_speed_prev_count;  /* 上一拍右轮 count */
    uint32_t speed_window_acc_ms;     /* 当前窗口累计毫秒数 */
    int32_t  left_speed_cps;          /* 左轮速度（counts/s），窗口差分结果 */
    int32_t  right_speed_cps;         /* 右轮速度（counts/s） */

    /* 脉冲刹车定时器 */
    uint32_t brake_pulse_remain_ms;   /* 剩余刹车毫秒数（0 = 无 pending 刹车脉冲） */

    /* 命令与配置 */
    int16_t  left_cmd_pm;             /* 左轮命令（permille，未限幅/未翻转） */
    int16_t  right_cmd_pm;            /* 右轮命令 */
    uint16_t pwm_limit_pm;            /* PWM 限幅值 */
    bool     invert_left;             /* 左轮极性翻转标志 */
    bool     invert_right;            /* 右轮极性翻转标志 */
    bool     button_idle_high;        /* S1 按键空闲电平（上电时判定，按下时相反） */
    bool     button_was_pressed;      /* 上一拍的按键状态（用于检测边沿） */
    bool     enabled;                 /* 是否已使能（STBY 状态） */
} motor_state_t;

/** 全局唯一的电机状态实例 */
static motor_state_t s_motor;

/* ================================================================
 * 内部辅助函数
 * ================================================================
 * 这些函数用 static 修饰，只在 bsp_motor.c 内部使用。
 * 它们不对外暴露，外部代码不能直接调用。
 */

/**
 * clip_int —— 整数限幅
 * 把 v 限制在 [lo, hi] 范围内。
 * 用于限幅命令值、PWM 占空比等。
 */
static inline int16_t clip_int(int32_t v, int32_t lo, int32_t hi)
{
    if (v < lo) return (int16_t)lo;
    if (v > hi) return (int16_t)hi;
    return (int16_t)v;
}

/**
 * apply_limit —— 对 permille 命令应用 PWM 限幅
 * 把传入的 permille 值钳到 [-pwm_limit, +pwm_limit]。
 */
static int16_t apply_limit(int16_t permille)
{
    int32_t lim = (int32_t)s_motor.pwm_limit_pm;
    if (lim > BSP_MOTOR_PWM_MAX_PERMILLE) {
        lim = BSP_MOTOR_PWM_MAX_PERMILLE;
    }
    return (int16_t)clip_int((int32_t)permille, -lim, lim);
}

/**
 * abs_permille —— 取 permille 的绝对值
 * 注意：假设输入已经限幅，不会出现 -32768 这种"负得不能再负"的极端值，
 * 所以 -(int32_t)permille 是安全的。
 */
static uint32_t abs_permille(int16_t permille)
{
    return (uint32_t)((permille >= 0) ? permille : -(int32_t)permille);
}

/**
 * set_pwm_duty —— 设置 TIMA0 某通道的 PWM 占空比
 *
 * TIMA0 定时器工作在"向上计数"模式（从 0 数到 LOAD，然后归零重新开始）。
 * PWM 输出极性是 INIT_VAL_LOW（初始低电平），
 * 比较匹配时翻转为高电平，计到 LOAD 时翻回低。
 *
 * 所以：compare 值越小 → 高电平时间越长 → 占空比越大。
 *
 * 公式推导：
 *   compare = load - (load * duty / 1000)
 *   duty=1000 → compare = load - load = 0 → 一直高 → 100%
 *   duty=0    → compare = load - 0 = load → 一直低 → 0%
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

/* ================================================================
 * TB6612 方向控制
 * ================================================================
 * TB6612 的每个通道有 2 个方向引脚（IN1/IN2），1 个 PWM 引脚。
 * 方向引脚的组合决定了电机的状态：
 *
 *   IN1=L IN2=L     → 滑行 (Coast)：电机两端悬空，自由转动
 *   IN1=H IN2=L     → 正转 (Forward)
 *   IN1=L IN2=H     → 反转 (Reverse)
 *   IN1=H IN2=H     → 刹车 (Brake)：电机两端短接，反电动势制动
 *
 * PWM 引脚的占空比决定速度（本模块由 TIMA0 的 CCP0/CCP1 输出）。
 */

typedef enum {
    DIR_COAST    = 0,   /* 滑行 */
    DIR_FORWARD  = 1,   /* 正转 */
    DIR_REVERSE  = 2,   /* 反转 */
    DIR_BRAKE    = 3    /* 刹车 */
} tb6612_dir_t;

/** 设置左电机（Motor A）的方向 */
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

/** 设置右电机（Motor B）的方向（同 set_dir_left，但用 BSP_BIN1/BSP_BIN2） */
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
 * apply_one_channel —— 把"经过限幅的 permille"应用到一路电机输出
 *
 * 处理流程：
 *   permille > 0 → 正转 + 占空比 = |permille|
 *   permille < 0 → 反转 + 占空比 = |permille|
 *   permille = 0 → 滑行 + 占空比 = 0
 *
 * @param permille_after_invert  已经应用过极性翻转的 permille 值
 * @param is_left                是否是左轮（true=左，false=右）
 */
static void apply_one_channel(int16_t permille_after_invert, bool is_left)
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

/** commit_left —— 左轮命令 → 物理输出（限幅 + 极性翻转 + 写硬件） */
static void commit_left(int16_t cmd_pm)
{
    int16_t pm = apply_limit(cmd_pm);      /* 先限幅 */
    if (s_motor.invert_left) {
        pm = (int16_t)(-(int32_t)pm);      /* 再翻转极性 */
    }
    apply_one_channel(pm, true);           /* 应用到硬件 */
}

/** commit_right —— 右轮命令 → 物理输出 */
static void commit_right(int16_t cmd_pm)
{
    int16_t pm = apply_limit(cmd_pm);
    if (s_motor.invert_right) {
        pm = (int16_t)(-(int32_t)pm);
    }
    apply_one_channel(pm, false);
}

/* ================================================================
 * 中断辅助函数
 * ================================================================
 * 这些函数在 GROUP1_IRQHandler（ISR）中被调用。
 * 它们必须尽可能快——只在 ISR 中做最必要的操作（计数 + 标志），
 * 把耗时的运算（浮点换算）留到主循环。
 */

/**
 * on_right_encoder_edge —— 右编码器边沿中断处理
 *
 * 当右轮编码器的 A 相（PA12）或 B 相（PA13）电平变化时，
 * 这个函数被 ISR 调用。它根据当前 A/B 电平判断旋转方向，
 * 然后 right_count 加 1 或减 1。
 *
 * 解码原理（正交编码器）：
 *   正转时 A 相领先 B 相 90°：
 *     A ╭─╮   ╭─╮   ╭─╮          B ╭─╮   ╭─╮   ╭─╮
 *       ╯ ╰───╯ ╰───╯ ╰──        ╯ ╰───╯ ╰───╯ ╰──
 *     在 A 上升沿时：B=高 → direction = +1
 *
 *   反转时 B 相领先 A 相 90°：
 *     在 A 上升沿时：B=低 → direction = -1
 *
 * 代码中的判断逻辑（X4 通用）：
 *   当 A、B 电平不同时（phase_a != phase_b）→ +1
 *   当 A、B 电平相同时（phase_a == phase_b）→ -1
 *   PA13 触发时取反（因为 B 跳变时的 A/B 异同关系与 A 跳变时相反）
 *
 * @param is_phase_a_edge  true=PA12(A相)触发，false=PA13(B相)触发
 */
static void on_right_encoder_edge(bool is_phase_a_edge)
{
    /* 统计 ISR 进入次数（用于雪崩检测和诊断） */
    s_motor.enc_irq_count++;
    s_motor.enc_irq_window++;

    /* 读取 A 相（PA12）和 B 相（PA13）的当前电平 */
    uint32_t pins = DL_GPIO_readPins(GPIOA, BSP_ENC_R_A_PIN | BSP_ENC_R_B_PIN);
    bool phase_a = ((pins & BSP_ENC_R_A_PIN) != 0u);
    bool phase_b = ((pins & BSP_ENC_R_B_PIN) != 0u);

    /* X4 解码：根据 A/B 电平关系判断方向 */
    int32_t step = (phase_a != phase_b) ? 1 : -1;
    if (!is_phase_a_edge) {
        step = -step;  /* B 相触发时，方向取反 */
    }

    /* 右轮编码器物理安装方向与左轮相反（机械原因）。
     * 这里翻转反馈符号，让左右轮在同向命令下 count 同号，
     * 方便速度环计算。 */
    step = -step;

    /* 更新累计计数 */
    s_motor.right_count += step;
}

/**
 * on_start_button_edge —— S1 按键边沿中断处理
 *
 * S1 按键（PA18）按下/松开时会触发 GPIO 边沿中断。
 * 这个函数判断是否是一次"有效的按下"：
 *   1. 读取当前电平，和上电空闲电平比较
 *   2. 如果是按下（电平 ≠ 空闲电平），做 80 ms 去抖
 *   3. 确认有效后设置 toggle_request = 1
 *
 * 注意：这里用的是"上电空闲电平"判断，而不是直接判断"低电平=按下"。
 * 因为 LaunchPad 的 S1 按键走线经过 J8 跳线，可能影响极性，
 * 所以在上电时先记录一次空闲电平，之后根据"电平是否变化"来判断按下。
 */
static void on_start_button_edge(void)
{
    uint32_t pins = DL_GPIO_readPins(BSP_START_BTN_PORT, BSP_START_BTN_PIN);
    bool level_high = ((pins & BSP_START_BTN_PIN) != 0u);

    /* 按下 = 当前电平与空闲电平相反
     * 例如空闲电平=高（松开时高），则按下时电平=低 */
    bool pressed = (level_high != s_motor.button_idle_high);

    uint32_t now_ms = bsp_systick_get_ms();

    /* 如果是松开事件（不是按下），只记录 was_pressed 状态，不触发 toggle */
    if (!pressed) {
        s_motor.button_was_pressed = false;
        return;
    }

    /* 去抖检查：如果已经按过了，或者距离上次按下的时间不到 80 ms，则忽略 */
    if (s_motor.button_was_pressed ||
        ((now_ms - s_motor.last_button_ms) < BSP_MOTOR_BTN_DEBOUNCE_MS)) {
        s_motor.button_was_pressed = pressed;
        return;
    }

    /* 有效按下：记录时间戳、统计计数、设置 toggle 标志 */
    s_motor.last_button_ms = now_ms;
    s_motor.button_irq_count++;
    s_motor.button_was_pressed = true;
    s_motor.toggle_request = 1u;
}

/**
 * poll_start_button —— S1 按键轮询兜底处理
 *
 * 如果中断路径没有命中（比如中断函数名写错、中断优先级被屏蔽等），
 * 这个轮询函数会作为"备份"来检测按键。
 *
 * 它在 bsp_motor_consume_toggle_request() 中被调用——当没有 pending 的
 * toggle 请求时，主动检查一次按键状态。
 *
 * 这样做的好处：即使中断没工作，按键功能仍然可用。
 * 代价是：每次 consume 都要读一次 GPIO 寄存器（需要几个微秒）。
 */
static void poll_start_button(void)
{
    uint32_t pins = DL_GPIO_readPins(BSP_START_BTN_PORT, BSP_START_BTN_PIN);
    bool level_high = ((pins & BSP_START_BTN_PIN) != 0u);
    bool pressed = (level_high != s_motor.button_idle_high);
    uint32_t now_ms = bsp_systick_get_ms();

    /* 如果检测到有效按下（去抖通过），设置 toggle 并统计轮询命中次数 */
    if (pressed && !s_motor.button_was_pressed &&
        ((now_ms - s_motor.last_button_ms) >= BSP_MOTOR_BTN_DEBOUNCE_MS)) {
        s_motor.last_button_ms = now_ms;
        s_motor.button_poll_count++;  /* 轮询兜底统计（和中断统计区分开） */
        s_motor.toggle_request = 1u;
    }

    /* 记录当前按下状态，供下一拍判断边沿 */
    s_motor.button_was_pressed = pressed;
}

/* ================================================================
 * 公开 API：初始化 / 使能
 * ================================================================ */

void bsp_motor_init(void)
{
    /* --- 步骤 0：配置左轮 QEI 引脚（PB15/PB16）的上拉和施密特触发 ---
     *
     * QEI 引脚由 SysConfig 配置为 TIMG8 的外设输入功能（GPIO_QEI_*），
     * 但 SysConfig 默认不会配置内部上拉和施密特触发。
     * 如果不配，编码器掉电或未连接时 PB15/PB16 浮空，
     * AC 噪声让 QEI 计数器乱跳。
     *
     * 这里二次写 IOMUX 寄存器追加 PULL_UP + HYSTERESIS_ENABLE，
     * 与右轮 PA12/PA13（在 bsp_gpio.c 中已配）保持一致的稳健度。 */
    DL_GPIO_initPeripheralInputFunctionFeatures(
        GPIO_QEI_LEFT_PHA_IOMUX, GPIO_QEI_LEFT_PHA_IOMUX_FUNC,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(
        GPIO_QEI_LEFT_PHB_IOMUX, GPIO_QEI_LEFT_PHB_IOMUX_FUNC,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);

    /* --- 步骤 1：所有状态清零 --- */
    s_motor.left_count             = 0;
    s_motor.right_count            = 0;
    s_motor.toggle_request         = 0u;
    s_motor.last_button_ms         = 0u;
    s_motor.button_irq_count       = 0u;
    s_motor.button_poll_count      = 0u;

    /* 记录当前 QEI 计数值作为"零位"——下次 update 会基于这个值算差值 */
    s_motor.left_raw_prev          = (uint16_t)DL_TimerG_getTimerCount(QEI_LEFT_INST);

    s_motor.left_speed_prev_count  = 0;
    s_motor.right_speed_prev_count = 0;
    s_motor.speed_window_acc_ms    = 0u;
    s_motor.left_speed_cps         = 0;
    s_motor.right_speed_cps        = 0;

    s_motor.brake_pulse_remain_ms  = 0u;

    s_motor.enc_irq_count          = 0u;
    s_motor.enc_irq_window         = 0u;
    s_motor.enc_irq_quench_remain_ms = 0u;

    s_motor.left_cmd_pm            = 0;
    s_motor.right_cmd_pm           = 0;
    s_motor.pwm_limit_pm           = BSP_MOTOR_PWM_MAX_PERMILLE;
    s_motor.invert_left            = false;
    s_motor.invert_right           = false;

    /* 记录 S1 按键上电空闲电平——此时还没有任何人按过它。
     * 之后通过比较当前电平和这个空闲电平来判断是否按下。 */
    s_motor.button_idle_high       =
        ((DL_GPIO_readPins(BSP_START_BTN_PORT, BSP_START_BTN_PIN) &
            BSP_START_BTN_PIN) != 0u);
    s_motor.button_was_pressed     = false;
    s_motor.enabled                = false;

    /* --- 步骤 2：设安全态（电机不会转） ---
     * 方向位 = 滑行，PWM = 0，STBY = 低（待机）
     * 即使此时 PWM 模块已经开始输出，电机也不会转。
     * 这叫"上电安全态"——宁可不动，不能乱动。 */
    set_dir_left (DIR_COAST);
    set_dir_right(DIR_COAST);
    set_pwm_duty(GPIO_PWM_MOTOR_C0_IDX, 0u);
    set_pwm_duty(GPIO_PWM_MOTOR_C1_IDX, 0u);
    DL_GPIO_clearPins(BSP_STBY_PORT, BSP_STBY_PIN);

    /* --- 步骤 3：配置 GPIO 中断 ---
     *
     * 右编码器（PA12 = A 相，PA13 = B 相）：
     *   双边沿中断 —— 上升沿和下降沿都触发 ISR。
     *   X4 模式：A 和 B 都开双沿，分辨率 = 电机轴 PPR × 4
     *   X2 模式：只 A 开双沿，B 只读电平不触发中断
     *
     * S1 按键（PA18）：
     *   双边沿中断 —— 同时检测按下和松开
     *   具体判断逻辑在 on_start_button_edge() 中
     *
     * 优先级设置（关键！）：
     *   GPIOA_INT_IRQn = 3（最低优先级）
     *   SysTick_IRQn = 0（最高优先级，已在 bsp_systick.c 中设置）
     *   这样即使 GPIO ISR 雪崩，SysTick 也能正常执行。 */

    /* 设置极性（哪个边沿触发中断）：双沿 = 上升沿 | 下降沿 */
#if (BSP_MOTOR_RIGHT_DECODE_X == 4)
    DL_GPIO_setLowerPinsPolarity(GPIOA,
        DL_GPIO_PIN_12_EDGE_RISE | DL_GPIO_PIN_12_EDGE_FALL |
        DL_GPIO_PIN_13_EDGE_RISE | DL_GPIO_PIN_13_EDGE_FALL);
    uint32_t enc_pins = BSP_ENC_R_A_PIN | BSP_ENC_R_B_PIN;
#else
    DL_GPIO_setLowerPinsPolarity(GPIOA,
        DL_GPIO_PIN_12_EDGE_RISE | DL_GPIO_PIN_12_EDGE_FALL);
    uint32_t enc_pins = BSP_ENC_R_A_PIN;
#endif
    /* S1 按键同样使用双沿 */
    DL_GPIO_setUpperPinsPolarity(GPIOA,
        DL_GPIO_PIN_18_EDGE_RISE | DL_GPIO_PIN_18_EDGE_FALL);

    /* 清除可能残留的中断标志 + 使能中断 */
    DL_GPIO_clearInterruptStatus(GPIOA, enc_pins | BSP_START_BTN_PIN);
    DL_GPIO_enableInterrupt    (GPIOA, enc_pins | BSP_START_BTN_PIN);

    /* 设置中断优先级 + 使能 NVIC */
    NVIC_ClearPendingIRQ(GPIOA_INT_IRQn);
    NVIC_SetPriority    (GPIOA_INT_IRQn, 3u);  /* 最低优先级（数值 3 = 最低） */
    NVIC_EnableIRQ      (GPIOA_INT_IRQn);
}

void bsp_motor_enable(bool enable)
{
    s_motor.enabled = enable;
    if (enable) {
        DL_GPIO_setPins(BSP_STBY_PORT, BSP_STBY_PIN);   /* STBY=高 → 电机可以工作 */
    } else {
        DL_GPIO_clearPins(BSP_STBY_PORT, BSP_STBY_PIN);  /* STBY=低 → 待机 */
    }
}

bool bsp_motor_is_enabled(void)
{
    return s_motor.enabled;
}

/* ================================================================
 * 公开 API：速度命令
 * ================================================================ */

void bsp_motor_set_output(int16_t left_permille, int16_t right_permille)
{
    s_motor.brake_pulse_remain_ms = 0u;   /* 任何新命令都取消未到期的刹车脉冲 */
    s_motor.left_cmd_pm  = left_permille;
    s_motor.right_cmd_pm = right_permille;
    commit_left (left_permille);
    commit_right(right_permille);
}

void bsp_motor_set_left(int16_t left_permille)
{
    s_motor.brake_pulse_remain_ms = 0u;
    s_motor.left_cmd_pm = left_permille;
    commit_left(left_permille);
}

void bsp_motor_set_right(int16_t right_permille)
{
    s_motor.brake_pulse_remain_ms = 0u;
    s_motor.right_cmd_pm = right_permille;
    commit_right(right_permille);
}

void bsp_motor_stop(void)
{
    s_motor.brake_pulse_remain_ms = 0u;
    s_motor.left_cmd_pm  = 0;
    s_motor.right_cmd_pm = 0;
    /* 直接操作硬件：方向位 → 滑行，PWM → 0 */
    set_dir_left (DIR_COAST);
    set_dir_right(DIR_COAST);
    set_pwm_duty(GPIO_PWM_MOTOR_C0_IDX, 0u);
    set_pwm_duty(GPIO_PWM_MOTOR_C1_IDX, 0u);
}

void bsp_motor_brake(void)
{
    s_motor.brake_pulse_remain_ms = 0u;   /* 持续模式——不会自动转 stop */
    s_motor.left_cmd_pm  = 0;
    s_motor.right_cmd_pm = 0;
    /* 短接电机两端（刹车）：方向位 → 刹车，PWM → 100% */
    set_dir_left (DIR_BRAKE);
    set_dir_right(DIR_BRAKE);
    set_pwm_duty(GPIO_PWM_MOTOR_C0_IDX, BSP_MOTOR_PWM_MAX_PERMILLE);
    set_pwm_duty(GPIO_PWM_MOTOR_C1_IDX, BSP_MOTOR_PWM_MAX_PERMILLE);
}

void bsp_motor_brake_pulse_ms(uint32_t duration_ms)
{
    if (duration_ms == 0u) {
        bsp_motor_stop();  /* 0 ms = 立即转 stop */
        return;
    }
    /* 硬件命令和持续 brake 一样，但设置计时器 */
    s_motor.left_cmd_pm  = 0;
    s_motor.right_cmd_pm = 0;
    set_dir_left (DIR_BRAKE);
    set_dir_right(DIR_BRAKE);
    set_pwm_duty(GPIO_PWM_MOTOR_C0_IDX, BSP_MOTOR_PWM_MAX_PERMILLE);
    set_pwm_duty(GPIO_PWM_MOTOR_C1_IDX, BSP_MOTOR_PWM_MAX_PERMILLE);
    /* 启动计时器，update() 中会倒计时 */
    s_motor.brake_pulse_remain_ms = duration_ms;
}

/* ================================================================
 * 公开 API：极性 / 限幅
 * ================================================================ */

void bsp_motor_set_invert(bool invert_left, bool invert_right)
{
    s_motor.invert_left  = invert_left;
    s_motor.invert_right = invert_right;
    /* 立刻重新应用当前命令——避免"已改极性但旧命令还在跑"的时间窗口 */
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
    /* 立刻重发当前命令应用新限幅 */
    commit_left (s_motor.left_cmd_pm);
    commit_right(s_motor.right_cmd_pm);
}

uint16_t bsp_motor_get_pwm_limit(void)
{
    return s_motor.pwm_limit_pm;
}

/* ================================================================
 * 公开 API：命令查询
 * ================================================================ */

int16_t bsp_motor_get_left_cmd(void) { return s_motor.left_cmd_pm; }
int16_t bsp_motor_get_right_cmd(void) { return s_motor.right_cmd_pm; }

/* ================================================================
 * 公开 API：编码器 / update / feedback
 * ================================================================ */

void bsp_motor_update(void)
{
    /* --- 步骤 0：brake pulse 倒计时（先做，避免本拍 skip） ---
     * 如果有未到期的刹车脉冲，递减剩余毫秒数。
     * 到 0 时自动转 stop（滑行）。 */
    if (s_motor.brake_pulse_remain_ms != 0u) {
        s_motor.brake_pulse_remain_ms--;
        if (s_motor.brake_pulse_remain_ms == 0u) {
            bsp_motor_stop();
        }
    }

    /* --- 步骤 0.5：ISR 雪崩检测 ---
     *
     * 检查上一毫秒内右编码器 ISR 进入了多少次。
     * 如果超过阈值（默认 200），说明编码器引脚可能浮空，
     * 环境噪声触发了雪崩中断。
     *
     * 处理方式：
     *   - 超过阈值 → 暂时关闭 PA12/PA13 中断（持续 N ms）
     *   - 关闭期间编码器边沿丢失，但 CPU 能喘气
     *   - 到期后自动重新打开中断 */
    uint32_t window;
    MOTOR_LOCK();
    window = s_motor.enc_irq_window;
    s_motor.enc_irq_window = 0u;  /* 清空窗口计数 */
    MOTOR_UNLOCK();

    if (s_motor.enc_irq_quench_remain_ms != 0u) {
        /* 正在抑制中：递减剩余毫秒数 */
        s_motor.enc_irq_quench_remain_ms--;
        if (s_motor.enc_irq_quench_remain_ms == 0u) {
            /* 到期：重新使能编码器中断 */
            DL_GPIO_clearInterruptStatus(GPIOA,
                BSP_ENC_R_A_PIN
#if (BSP_MOTOR_RIGHT_DECODE_X == 4)
                | BSP_ENC_R_B_PIN
#endif
            );
            DL_GPIO_enableInterrupt(GPIOA,
                BSP_ENC_R_A_PIN
#if (BSP_MOTOR_RIGHT_DECODE_X == 4)
                | BSP_ENC_R_B_PIN
#endif
            );
        }
    } else if (window > BSP_MOTOR_ENC_IRQ_QUENCH_PER_MS) {
        /* 超过阈值：关闭编码器中断，启动抑制计时 */
        DL_GPIO_disableInterrupt(GPIOA,
            BSP_ENC_R_A_PIN
#if (BSP_MOTOR_RIGHT_DECODE_X == 4)
            | BSP_ENC_R_B_PIN
#endif
        );
        s_motor.enc_irq_quench_remain_ms = BSP_MOTOR_ENC_IRQ_QUENCH_DURATION_MS;
    }

    /* --- 步骤 1：左轮 QEI 16-bit → 32-bit 软件扩位 ---
     *
     * TIMG8 的 QEI 计数器是 16 位的，范围 0~65535。
     * 电机转得快时，几秒钟就会溢出。
     * 解决方法是：每次 update() 读一次 16 位原始值，
     * 和上一拍的值做差，把差值累加到 32 位的 left_count 中。
     *
     * 差值计算（处理了溢出情况）：
     *   delta = (int16_t)(raw_now - raw_prev)
     *   例如 raw_now=10, raw_prev=65530
     *   delta = (int16_t)(10 - 65530) = (int16_t)(-65520) = 16
     *   虽然计数器溢出了，但差值仍然是正确的 16！ */
    uint16_t raw_now = (uint16_t)DL_TimerG_getTimerCount(QEI_LEFT_INST);
    int16_t  delta   = (int16_t)((uint16_t)(raw_now - s_motor.left_raw_prev));
    s_motor.left_raw_prev = raw_now;

    /* left_count 是 ISR 共享变量，但左轮 QEI 只在 update() 中写入，
     * 而右轮 ISR 只碰 right_count，理论上这里没有 race condition。
     * 但为了代码健壮性（将来可能改 ISR），仍然关中断保护。 */
    MOTOR_LOCK();
    s_motor.left_count += (int32_t)delta;
    MOTOR_UNLOCK();

    /* --- 步骤 2：速度窗口差分 ---
     *
     * 每 1 ms 累加一次，累计满 BSP_MOTOR_SPEED_WINDOW_MS 后做一次差分。
     *
     * 差分的计算：
     *   left_speed_cps = (当前 left_count - 上一拍 left_count) × 1000 / 窗口毫秒数
     *   right_speed_cps 同理
     *
     * 这样就得到了"counts per second"（每秒计数），即瞬时速度。
     * 至于把 cps 换算为 rpm 或 dps（°/s），在 get_feedback() 中做。 */
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

    /* 关中断保护：一次性读取所有 ISR 共享字段 */
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

    /* 填充整数计数（不需要浮点运算） */
    feedback->left_count        = left_count;
    feedback->right_count       = right_count;
    feedback->left_speed_cps    = left_speed_cps;
    feedback->right_speed_cps   = right_speed_cps;

    /* 现场换算浮点数值（避免在 1 kHz update 中做浮点运算）
     *   counts → 角度：每 count 对应的角度 × count 数
     *   cps → dps：每 count 对应的角度 × cps
     *   dps → rpm：dps / 360 × 60 = dps / 6 */
    feedback->left_angle_deg    = (float)left_count       * LEFT_DEG_PER_COUNT;
    feedback->right_angle_deg   = (float)right_count      * RIGHT_DEG_PER_COUNT;
    feedback->left_speed_dps    = (float)left_speed_cps   * LEFT_DEG_PER_COUNT;
    feedback->right_speed_dps   = (float)right_speed_cps  * RIGHT_DEG_PER_COUNT;
    feedback->left_speed_rpm    = feedback->left_speed_dps  / 6.0f;
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

uint32_t bsp_motor_get_enc_irq_count(void)
{
    uint32_t v;
    MOTOR_LOCK();
    v = s_motor.enc_irq_count;
    MOTOR_UNLOCK();
    return v;
}

bool bsp_motor_enc_irq_is_quenched(void)
{
    return (s_motor.enc_irq_quench_remain_ms != 0u);
}

uint32_t bsp_motor_get_button_irq_count(void)
{
    uint32_t v;
    MOTOR_LOCK();
    v = s_motor.button_irq_count;
    MOTOR_UNLOCK();
    return v;
}

uint32_t bsp_motor_get_button_poll_count(void)
{
    uint32_t v;
    MOTOR_LOCK();
    v = s_motor.button_poll_count;
    MOTOR_UNLOCK();
    return v;
}

bool bsp_motor_is_start_button_active(void)
{
    bool pressed;
    MOTOR_LOCK();
    uint32_t pins = DL_GPIO_readPins(BSP_START_BTN_PORT, BSP_START_BTN_PIN);
    bool level_high = ((pins & BSP_START_BTN_PIN) != 0u);
    pressed = (level_high != s_motor.button_idle_high);
    MOTOR_UNLOCK();
    return pressed;
}

bool bsp_motor_get_start_button_raw_level(void)
{
    return ((DL_GPIO_readPins(BSP_START_BTN_PORT, BSP_START_BTN_PIN) &
        BSP_START_BTN_PIN) != 0u);
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
    /* 重置 QEI 零位——避免下一拍突然吃进一大段差值 */
    s_motor.left_raw_prev = (uint16_t)DL_TimerG_getTimerCount(QEI_LEFT_INST);
    MOTOR_UNLOCK();
}

/* ================================================================
 * 公开 API：S1 toggle 请求
 * ================================================================ */

bool bsp_motor_consume_toggle_request(void)
{
    bool pending;
    MOTOR_LOCK();
    if (s_motor.toggle_request == 0u) {
        /* 没有 pending 的中断事件 → 轮询兜底检查一次 */
        poll_start_button();
    }
    pending = (s_motor.toggle_request != 0u);
    s_motor.toggle_request = 0u;  /* 消费——清零标志 */
    MOTOR_UNLOCK();
    return pending;
}

/* ================================================================
 * GROUP1_IRQHandler —— GPIO 中断服务函数（极易踩坑！）
 * ================================================================
 *
 * ⚠️⚠️⚠️ 重要！MSPM0G3507 的 GPIO 中断架构 ⚠️⚠️⚠️
 *
 * 在 MSPM0G3507 中，所有 GPIO 中断（GPIOA + GPIOB）+ TRNG + COMP0
 * 共享同一个 NVIC 中断通道：IRQn = 1，名字叫 "GROUP1"。
 *
 * 中断向量表的入口名是 `GROUP1_IRQHandler`，**不是**
 * `GPIOA_IRQHandler` 或 `GPIOB_IRQHandler`。
 *
 * 踩坑历史（Stage 2.4 关键修复 / 2026-05-09）：
 *   本文件早期把 ISR 命名为 `GPIOA_IRQHandler`。
 * 编译链接都不报错（这只是个普通的全局函数符号），
 *   但向量表槽位 17 仍然指向启动文件中的 weak 默认函数
 *   `GROUP1_IRQHandler`（实现为 `B .` 死循环）。
 *
 *   症状：转动右轮时 PA12/PA13 沿事件触发 → NVIC 跳转到
 *   默认的 GROUP1_IRQHandler（死循环）→ MCU 整体卡死，
 *   串口 / SysTick / 业务全停。即使把 SysTick 优先级提到最高、
 *   ENC 上拉 + 雪崩兜底全部启用都救不了——因为 ISR 根本就不是
 *   我们写的那个函数！
 *
 * 正确做法：函数名必须叫 `GROUP1_IRQHandler`。
 *
 * 中断分发逻辑：
 *   进入 ISR 后，通过 DL_GPIO_getEnabledInterruptStatus()
 *   读取 GPIOA 的中断状态位图（MIS 寄存器），
 *   然后逐个处理：PA12 → PA13（X4）→ PA18（S1）。
 *   每个源处理完后单独 clear 中断标志。
 *
 *   为什么不用 IIDX（中断索引号）？
 *   因为 IIDX 一次只能返回一个最高优先级的待处理中断，
 *   如果 PA12 和 PA18 同时触发，IIDX 只会告诉你 PA12，
 *   PA18 的中断会被延迟到 ISR 的下一次运行。
 *   而用 MIS 位图方式可以一次处理所有 pending 的源。
 */

void GROUP1_IRQHandler(void)
{
    /* 读取 GPIOA 的使能中断状态位图（哪个引脚触发了中断） */
    uint32_t gpioa = DL_GPIO_getEnabledInterruptStatus(GPIOA,
        BSP_ENC_R_A_PIN
#if (BSP_MOTOR_RIGHT_DECODE_X == 4)
        | BSP_ENC_R_B_PIN
#endif
        | BSP_START_BTN_PIN);

    /* 处理 PA12（右编码器 A 相）中断 */
    if ((gpioa & BSP_ENC_R_A_PIN) != 0u) {
        on_right_encoder_edge(true);   /* is_phase_a_edge = true */
        DL_GPIO_clearInterruptStatus(GPIOA, BSP_ENC_R_A_PIN);
    }

    /* 处理 PA13（右编码器 B 相）中断——仅在 X4 模式下启用 */
#if (BSP_MOTOR_RIGHT_DECODE_X == 4)
    if ((gpioa & BSP_ENC_R_B_PIN) != 0u) {
        on_right_encoder_edge(false);  /* is_phase_a_edge = false */
        DL_GPIO_clearInterruptStatus(GPIOA, BSP_ENC_R_B_PIN);
    }
#endif

    /* 处理 PA18（S1 按键）中断 */
    if ((gpioa & BSP_START_BTN_PIN) != 0u) {
        on_start_button_edge();
        DL_GPIO_clearInterruptStatus(GPIOA, BSP_START_BTN_PIN);
    }
}
