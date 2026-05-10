/**
 * @file    bsp_motor.h
 * @brief   电机底层驱动 —— TB6612 驱动 + 左右编码器反馈 + S1 启动按键
 *
 * ============================================================
 * 这个文件是做什么的？（写给初学者）
 * ============================================================
 * 这个文件是整个自平衡小车的"腿"——它控制两个轮子怎么转。
 *
 * 它管三件事：
 *   1. 让电机转起来（TB6612 驱动芯片 + PWM 控制速度 + 方向引脚控制方向）
 *   2. 知道轮子转了多少（编码器反馈——左轮用硬件 QEI，右轮用 GPIO 中断）
 *   3. 检测启动按键（S1 按键，用来一键启动小车）
 *
 * 本工程的电机硬件方案：
 *   ─ 电机：GB370 减速直流电机（30:1 减速比，11 PPR 霍尔编码器）
 *   ─ 驱动：TB6612FNG（双路 H 桥，可独立控制左右电机）
 *   ─ 左轮测速：TIMG8 硬件 QEI（正交编码器接口，X4 解码）
 *   ─ 右轮测速：PA12/PA13 GPIO 中断（软件解码，X2 或 X4）
 *   ─ PWM：TIMA0 定时器，20 kHz（超出人耳范围，没有啸叫声）
 *
 * 如果你想了解"自平衡小车怎么动"——从这个文件开始读。
 */

#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
 * 编译期可配宏 —— 可以在编译器命令行通过 -D 覆盖
 * ================================================================
 * 这些宏定义了电机相关的物理参数。
 * 如果换了不同规格的电机，只需要修改这里的宏，不用改代码。
 */

/** PWM 命令满量程（千分比）。1000 = 100% 占空比，满速。
 *  所有速度命令的范围都是 [-1000, +1000]。 */
#define BSP_MOTOR_PWM_MAX_PERMILLE                 (1000)

/** GB370 减速箱减速比。电机轴转 30 圈，输出轴转 1 圈。
 *  我们关心的是"输出轴转了多少"，所以要除以这个减速比。 */
#define BSP_MOTOR_GB370_GEAR_RATIO                 (30)

/** GB370 内置霍尔传感器每**电机轴**转一圈产生的脉冲数（A 相单沿）。
 *  11 PPR（Pulse Per Revolution）= 电机轴每转一圈，A 相产生 11 个脉冲。
 *  实际输出轴转一圈 = 11 × 30 = 330 个脉冲（单沿）。 */
#define BSP_MOTOR_GB370_HALL_PPR                   (11)

/** 左轮 QEI 解码倍率。X4 表示每个脉冲的上升沿和下降沿都计数。
 *  左轮每输出轴一圈的编码器计数 = 30 × 11 × 4 = 1320 */
#define BSP_MOTOR_LEFT_DECODE_X                    (4)

/** 右轮解码倍率。默认 X4 与左轮一致，两边分辨率相同。
 *  如果 CPU 负荷太重，可以编译时传 -DBSP_MOTOR_RIGHT_DECODE_X=2 切到 X2。 */
#ifndef BSP_MOTOR_RIGHT_DECODE_X
#define BSP_MOTOR_RIGHT_DECODE_X                   (4)
#endif

/* 左右轮每输出轴一圈的编码器计数（编译时常量） */
#define BSP_MOTOR_LEFT_COUNTS_PER_OUTPUT_REV \
    (BSP_MOTOR_GB370_GEAR_RATIO * BSP_MOTOR_GB370_HALL_PPR * BSP_MOTOR_LEFT_DECODE_X)   /* = 1320 */
#define BSP_MOTOR_RIGHT_COUNTS_PER_OUTPUT_REV \
    (BSP_MOTOR_GB370_GEAR_RATIO * BSP_MOTOR_GB370_HALL_PPR * BSP_MOTOR_RIGHT_DECODE_X)   /* = 1320 (X4) 或 660 (X2) */

/**
 * 速度差分窗口（毫秒）：
 *   bsp_motor_update() 每次累计 1 ms，累计满这个值后做一次 count 差分，
 *   计算出 cps（counts per second）。
 *
 *   窗口越小 → 速度响应越快，但低速时分辨率越粗。
 *   默认 20 ms → 50 Hz 速度刷新率。
 *
 *   最低可分辨速度（20 ms 窗口只能差 1 count）：
 *     左轮 = (1000/20) / 1320 × 60 ≈ 2.27 rpm
 *     右轮 = (1000/20) / 1320 × 60 ≈ 2.27 rpm（X4）或 4.55 rpm（X2）
 */
#ifndef BSP_MOTOR_SPEED_WINDOW_MS
#define BSP_MOTOR_SPEED_WINDOW_MS                  (20u)
#endif

/** S1 按键软件去抖时间（毫秒）。机械按键按下/松开时会有抖动，
 *  80 ms 的去抖窗口可以滤除大多数按键抖动导致的误触发。 */
#ifndef BSP_MOTOR_BTN_DEBOUNCE_MS
#define BSP_MOTOR_BTN_DEBOUNCE_MS                  (80u)
#endif

/**
 * 右编码器 ISR 雪崩保护参数（Stage 2.4 新增）。
 *
 * 编码器用 GPIO 中断来计数。如果编码器引脚悬空（没接电机或连接松动），
 * 环境噪声会触发数十 kHz 的边沿中断，导致：
 *   CPU 100% 在 ISR 中 → SysTick / 主循环 / printf 全部饿死
 *
 * 这个机制就是在"雪崩"发生时暂时关闭编码器中断，让 CPU 喘口气。
 *
 *   ENC_IRQ_QUENCH_PER_MS：单毫秒边沿率上限。
 *     手动转动编码器最快也就几 kHz/ms，超过 200 一定是引脚浮空。
 *     默认 200 = 200 kHz 边沿率警戒线。
 *
 *   ENC_IRQ_QUENCH_DURATION_MS：触发后关闭中断的持续时间（毫秒）。
 *     期间编码器边沿丢失，但主循环能正常推进。
 *     到期后由 bsp_motor_update() 重新打开中断。
 */
#ifndef BSP_MOTOR_ENC_IRQ_QUENCH_PER_MS
#define BSP_MOTOR_ENC_IRQ_QUENCH_PER_MS            (200u)
#endif

#ifndef BSP_MOTOR_ENC_IRQ_QUENCH_DURATION_MS
#define BSP_MOTOR_ENC_IRQ_QUENCH_DURATION_MS       (50u)
#endif

/* ================================================================
 * 反馈结构体 —— 调用方从这里读到电机的当前状态
 * ================================================================
 * 在 main 循环中调用 bsp_motor_get_feedback(&fb)，
 * 然后看 fb.left_speed_rpm、fb.right_angle_deg 等。
 *
 * 所有字段都在 get_feedback() 中一次性填充完毕（关中断保护）。
 */

typedef struct {
    /* ---- 累计计数 ---- */
    int32_t left_count;      /* 左轮编码器累计计数（int32，约 ±5 亿圈不溢出） */
    int32_t right_count;     /* 右轮编码器累计计数 */

    /* ---- 累计机械角 ---- */
    float   left_angle_deg;  /* 左轮输出轴累计旋转角度（度） */
    float   right_angle_deg; /* 右轮输出轴累计旋转角度（度） */

    /* ---- 瞬时速度（counts/s）---- */
    int32_t left_speed_cps;  /* 左轮瞬时速度（counts per second） */
    int32_t right_speed_cps; /* 右轮瞬时速度（counts per second） */

    /* ---- 瞬时速度（度/秒）---- */
    float   left_speed_dps;  /* 左轮瞬时角速度（degree per second） */
    float   right_speed_dps; /* 右轮瞬时角速度（degree per second） */

    /* ---- 瞬时速度（转/分钟）---- */
    float   left_speed_rpm;  /* 左轮瞬时转速（转/分钟） */
    float   right_speed_rpm; /* 右轮瞬时转速（转/分钟） */
} bsp_motor_feedback_t;

/* ================================================================
 * 初始化 / 使能
 * ================================================================ */

/**
 * @brief 初始化电机驱动。
 *
 * 这个函数做以下事情：
 *   1. 配置左轮 QEI 引脚（PB15/PB16）的内部上拉和施密特触发
 *   2. 清零所有状态（计数、命令、速度窗口等）
 *   3. 设安全态：PWM=0、方向=滑行、STBY=低（电机不转）
 *   4. 注册 PA12/PA13（编码器）和 PA18（按键）的 GPIO 中断
 *   5. GPIOA 中断优先级设为最低（不让 ISR 饿死 SysTick）
 *
 * 调用前提（按顺序）：
 *   SYSCFG_DL_init() → bsp_gpio_init() → bsp_systick_init(1000) → 本函数
 *
 * 调用后电机**不会**立即转动。还需要调用 bsp_motor_enable(true) 拉高 STBY。
 */
void bsp_motor_init(void);

/**
 * @brief 拉高/拉低 STBY 引脚，控制 TB6612 整体使能/待机。
 *
 * STBY 是 TB6612 的"总开关"：
 *   enable=true  → STBY=高 → 电机可以响应 PWM 和方向信号
 *   enable=false → STBY=低 → 电机驱动输出 Hi-Z，即使有 PWM 也不转
 *
 * 上电默认 STBY=低（待机），调用这个函数后才能让电机动。
 * 这是一个重要的安全设计——防止上电瞬间电机乱动。
 */
void bsp_motor_enable(bool enable);

/** 查询 STBY 当前是否拉高（true = 已使能，不一定有 PWM 输出） */
bool bsp_motor_is_enabled(void);

/* ================================================================
 * 速度命令
 * ================================================================
 * 所有速度命令使用"千分比"（permille）量纲，范围 [-1000, +1000]。
 *   +1000 = 正向满速
 *   -1000 = 反向满速
 *   0     = 滑行（coast，PWM=0，方向位清零）
 */

/**
 * @brief 同时设置左右轮速度命令。
 *
 * 这是最主要的控制接口。平衡控制算法（app_balance.c）计算出左右轮目标速度后，
 * 调用这个函数让电机执行。
 *
 * 处理流程：
 *   限幅 → 极性翻转（如果 invert 了）→ 设方向位 → 设 PWM 占空比
 *
 * @param left_permille  左轮命令 [-1000, 1000]，正=正转，负=反转，0=滑行
 * @param right_permille 右轮命令，同上
 */
void bsp_motor_set_output(int16_t left_permille, int16_t right_permille);

/** 仅设置左轮命令（右轮保持不变） */
void bsp_motor_set_left(int16_t left_permille);

/** 仅设置右轮命令（左轮保持不变） */
void bsp_motor_set_right(int16_t right_permille);

/**
 * @brief 滑行（Coast）停止。
 *
 * 操作：方向位清零、PWM 占空比设为 0、STBY 保持原状态。
 * 效果：电机在反电动势衰减下慢慢停下，适合常规减速。
 * 特点：停下较慢，但电流冲击小。
 */
void bsp_motor_stop(void);

/**
 * @brief 短刹车（Brake）停止。
 *
 * 操作：AIN1=AIN2=1（或 BIN1=BIN2=1）、PWM=满占空比。
 * 效果：TB6612 内部把电机两端短接 → 反电动势制动 → 快速停车。
 * 特点：停下快，但电流冲击大，长时间 brake 会发热。
 *
 * 这个函数进入**持续** brake 模式——调用方需要手动切回 stop 或 set_output。
 * 推荐使用 bsp_motor_brake_pulse_ms() 做脉冲式刹车。
 */
void bsp_motor_brake(void);

/**
 * @brief 脉冲式短刹车：刹车 N 毫秒后自动转滑行。
 *
 * 调用后立即进入 brake 状态，同时启动一个计时器。
 * 计时器到期后自动转 stop（滑行），避免长时间 brake 导致 TB6612 发热。
 *
 * 典型用法：跌倒检测 → bsp_motor_brake_pulse_ms(80) → 80 ms 后自动 coast。
 *
 * 这个函数和 bsp_motor_brake()（持续模式）互斥。
 * 调用 set_output / stop / brake 都会取消未到期的脉冲计时。
 *
 * @param duration_ms  brake 持续毫秒数，推荐 50~150。
 *                     传 0 = 立即转 stop（等价于 bsp_motor_stop()）
 */
void bsp_motor_brake_pulse_ms(uint32_t duration_ms);

/* ================================================================
 * 极性 / 限幅（运行时可调，不用重新编译）
 * ================================================================ */

/**
 * @brief 软件方向反转。
 *
 * 装车后发现某个轮子方向反了？调用这个函数翻转极性就行了，
 * 不用改电路（交换电机线）也不用改代码（修改方向真值表）。
 *
 * 函数返回前会立刻按新极性重发当前命令，避免"已调极性但旧命令还在跑"。
 *
 * 注意：极性只影响命令侧。反馈侧（count/speed）仍按编码器物理方向计数，
 * 如果想让反馈也跟随业务正向，调用方自己乘 -1。
 */
void bsp_motor_set_invert(bool invert_left, bool invert_right);

/** 查询当前极性翻转状态 */
void bsp_motor_get_invert(bool *invert_left, bool *invert_right);

/**
 * @brief 设置 PWM 占空比上限，范围 [0, 1000]。
 *
 * 所有命令在送进硬件前都会被钳到 ±limit 之内。
 * 用途：电池低压时降功率、调试时限速等安全保护。
 * 默认值 = 1000（不限幅）。
 */
void bsp_motor_set_pwm_limit(uint16_t limit_permille);

/** 查询当前 PWM 限幅值 */
uint16_t bsp_motor_get_pwm_limit(void);

/* ================================================================
 * 命令查询
 * ================================================================ */

/** 查询当前左轮命令 permille（已应用极性翻转与限幅**之前**的值） */
int16_t bsp_motor_get_left_cmd(void);

/** 查询当前右轮命令 permille */
int16_t bsp_motor_get_right_cmd(void);

/* ================================================================
 * 反馈与编码器 —— 获取电机的实际运行数据
 * ================================================================ */

/**
 * @brief 1 kHz 周期任务（必须在主循环中每毫秒调用一次！）
 *
 * 这个函数是电机驱动的"心脏"——它在每个 SysTick 中断中被触发，
 * 完成以下工作：
 *
 *   1. brake pulse 倒计时（到期自动转 stop）
 *   2. 编码器 ISR 雪崩检测和抑制
 *   3. 左轮 QEI 16-bit → 32-bit 软件扩位
 *   4. 速度窗口累计（满 N ms 做一次差分算出 cps）
 *
 * 必须在主循环中以 ≈ 1 kHz 频率调用：
 * @code
 *   for (;;) {
 *       if (bsp_systick_consume_tick()) {
 *           bsp_motor_update();
 *       }
 *   }
 * @endcode
 */
void bsp_motor_update(void);

/**
 * @brief 一次性获取所有反馈数据（线程安全）。
 *
 * 在调用方触发时（如每 10 ms 或 100 ms 一次），
 * 用关中断保护读取所有编码器状态，并现场计算浮点角度/转速。
 *
 * 浮点运算不放在 1 kHz 的 update() 中，而是放在这里——
 * 避免 Cortex-M0+ 的软浮点运算拖累 SysTick 周期。
 *
 * @param feedback  输出结构体指针；NULL 时函数静默返回。
 */
void bsp_motor_get_feedback(bsp_motor_feedback_t *feedback);

/** 单独读取左轮累计编码器计数（int32，原子读） */
int32_t bsp_motor_get_left_count(void);

/** 单独读取右轮累计编码器计数（int32，原子读） */
int32_t bsp_motor_get_right_count(void);

/**
 * @brief 读取右编码器 ISR 总进入次数（用于诊断编码器噪声）。
 *
 * 正常情况：每秒钟进入次数 ≈ 编码器实际边沿数。
 * 如果值突然飙到几十万/秒，说明引脚可能浮空，被环境噪声触发大量中断。
 *
 * 用法：每 1 秒采样一次，看差值是否合理。
 */
uint32_t bsp_motor_get_enc_irq_count(void);

/** @brief 查询 ISR 雪崩抑制当前是否激活。
 *  true = 编码器中断被暂时关闭（剩余抑制时间 > 0） */
bool bsp_motor_enc_irq_is_quenched(void);

/** @brief 读取 S1 按键中断命中次数（用于诊断按键中断是否工作） */
uint32_t bsp_motor_get_button_irq_count(void);

/** @brief 读取 S1 按键轮询兜底命中次数。
 *  如果中断路径没命中（比如中断函数名写错），轮询兜底可以补上。
 *  非 0 说明按键可读但中断路径可能有问题。 */
uint32_t bsp_motor_get_button_poll_count(void);

/** @brief 按上电空闲电平判定 S1 当前是否处于按下态 */
bool bsp_motor_is_start_button_active(void);

/** @brief 直接读取 S1(PA18) 原始电平：true = 高，false = 低 */
bool bsp_motor_get_start_button_raw_level(void);

/**
 * @brief 同时清零左右轮编码器计数与速度窗口。
 *
 * 常用于：上电校准、调试归零、新一次试跑开始前。
 * 不影响命令与 STBY 状态——不会让电机停止。
 */
void bsp_motor_reset_encoders(void);

/* ================================================================
 * S1 启动按键：Toggle 请求
 * ================================================================ */

/**
 * @brief 取出 S1(PA18) 的 toggle 请求（读后自动清零）。
 *
 * 典型用法：业务层把它当作"启动/急刹/模式切换"的边沿事件。
 *
 * 实现方式：
 *   中断路径（on_start_button_edge）和轮询兜底（poll_start_button）
 *   都会在检测到有效按键按下后设置 toggle_request=1。
 *   两者都做了 80 ms 软件去抖。
 *
 * @return true  = 自从上次调用以来，S1 至少被按下过一次
 *         false = 没有新的按键事件
 */
bool bsp_motor_consume_toggle_request(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_MOTOR_H */
