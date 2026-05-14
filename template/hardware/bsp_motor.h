/**
 * @file    bsp_motor.h
 * @brief   阶段 2 电机底层驱动：TB6612 + 左右轮编码器反馈。
 *
 * 硬件资源（来自 SysConfig 与 docs/TaskLog/Stage0-PinAllocation.md）：
 *   ─ PWM：TIMA0 CCP0/CCP1 → PA8 (PWMA) / PA9 (PWMB)，BUSCLK 32 MHz、
 *      period = 1599 → PWM 频率 = 32 MHz / 1600 ≈ 20 kHz（超出人耳范围）
 *   ─ 方向：PA15/PA16 (AIN1/AIN2)、PA26/PA27 (BIN1/BIN2)
 *   ─ STBY：PB0（高有效；上电默认低 → TB6612 待机 / 输出 Hi-Z）
 *   ─ 左轮编码器：TIMG8 硬件 QEI（A=PB15 / BP J4.34、B=PB16 / BP J4.40，
 *      Stage 2.3 起从 J12 迁来；2-Pin Mode = X4 解码），16-bit 计数器
 *      （LOAD = 65535）由 `bsp_motor_update()` 软件扩为 32-bit
 *   ─ 右轮编码器：PA12 双边沿中断 + PA13 ISR 内电平判方向（X2 解码）
 *
 * 接口约定：
 *   ─ 速度命令使用 permille（千分比），范围 [-1000, 1000]，
 *     正值正转、负值反转、0 = 滑行 (coast，方向位清零、PWM = 0)
 *   ─ 角度按 GB370 实测 11 PPR、9.6:1 减速比计算；如实物参数不同，
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

/** GB370 减速箱减速比（标称 9.6:1，仅供参考；实际定标以下方 COUNTS_PER_OUTPUT_REV 为准） */
#define BSP_MOTOR_GB370_GEAR_RATIO                 (9.6f)

/** GB370 内置霍尔每电机轴转脉冲数（A 相单沿） */
#define BSP_MOTOR_GB370_HALL_PPR                   (11)

/** 左轮 QEI mode 3 = X4 解码，每输出轴一圈的计数 */
#define BSP_MOTOR_LEFT_DECODE_X                    (4)

/**
 * 右轮解码倍率：默认 X4 (PA12 + PA13 都开双沿中断，与左轮分辨率一致 1320 cnt/rev)。
 * 若 CPU 负担过大或高速漏脉冲明显，可切回 X2（仅 PA12 双沿，PA13 ISR 内读电平）。
 *   X4 → 404 cnt/rev（实测定标），与左轮一致，平衡环左右系数可共用
 *   X2 → 约 202 cnt/rev，CPU ISR 减半（推荐转速 > 1500 rpm 出轴侧时切此）
 */
#ifndef BSP_MOTOR_RIGHT_DECODE_X
#define BSP_MOTOR_RIGHT_DECODE_X                   (4)
#endif

/**
 * 右路正转静态补偿：校准显示同一 PWM 下右路正转约快 5.22%，因此在 BSP 层
 * 对右路正向命令统一按 95% 输出。放在 BSP 而非 app 层，避免 demo / balance /
 * K230 等不同业务路径漏补偿或重复造轮子。
 * 可通过 `bsp_motor_set_right_forward_scale_enabled(false)` 临时关闭，用于校准
 * 扫描原始未补偿曲线。
 */
#ifndef BSP_MOTOR_RIGHT_FORWARD_SCALE_X1000
#define BSP_MOTOR_RIGHT_FORWARD_SCALE_X1000        (980)
#endif

/**
 * 电机静/动摩擦双门槛死区补偿（permille）。
 *
 * 物理事实（来自 tools/motor_calib 多轮标定）：
 *   ─ "静摩擦突破"：电机从静止启动所需的最小 PWM，四路各异，左反向最大。
 *   ─ "动摩擦门槛"：电机一旦转起来，维持转动所需的 PWM 远低于静摩擦。
 *
 * 单门槛"一刀切"会导致：非零命令瞬间跳到静摩擦补偿值 → 转速直接 0→10 RPM
 * 阶跃，PID 在零附近永远无法线性可控。因此 BSP 按编码器运动状态拆为两阶段：
 *
 *   ① **未确认运动 / 停转重试**：用 STATIC_DEADZONE_PM 强行突破静摩擦。
 *   ② **已确认运动**：用 RUNNING_DEADZONE_PM 做线性映射，死区减小，小命令
 *      也能产生小输出，0…1000‰ 区间线性可控。
 *
 * 静摩擦标定值（2026-05-11 第二轮 cal_mode 扫描，DZ in data 全部 ≈ 5 ✓）：
 *   ─ 左正 50（旧 44，残留 DZ_data=10 → +6 余量）
 *   ─ 左反 68（沿用，DZ_data=5 已最优）
 *   ─ 右正 50（旧 44，残留 DZ_data=10 → +6 余量，含 RIGHT_FORWARD_SCALE 0.95）
 *   ─ 右反 54（旧 42，残留 DZ_data=15 → +12 余量）
 *
 * 若旧工程只定义 BSP_MOTOR_DEADZONE_COMP_PM，则四路 DEADZONE_PM 沿用该值，
 * RUNNING_DEADZONE 默认取其一半（最少 1‰），保留旧行为入口。
 */
#ifndef BSP_MOTOR_DEADZONE_COMP_PM
#define BSP_MOTOR_DEADZONE_COMP_PM                (0)
#endif

#ifndef BSP_MOTOR_LEFT_FORWARD_DEADZONE_PM
#define BSP_MOTOR_LEFT_FORWARD_DEADZONE_PM        ((BSP_MOTOR_DEADZONE_COMP_PM > 0) ? BSP_MOTOR_DEADZONE_COMP_PM : 60)
#endif

#ifndef BSP_MOTOR_LEFT_REVERSE_DEADZONE_PM
#define BSP_MOTOR_LEFT_REVERSE_DEADZONE_PM        ((BSP_MOTOR_DEADZONE_COMP_PM > 0) ? BSP_MOTOR_DEADZONE_COMP_PM : 60)
#endif

#ifndef BSP_MOTOR_RIGHT_FORWARD_DEADZONE_PM
#define BSP_MOTOR_RIGHT_FORWARD_DEADZONE_PM       ((BSP_MOTOR_DEADZONE_COMP_PM > 0) ? BSP_MOTOR_DEADZONE_COMP_PM : 60)
#endif

#ifndef BSP_MOTOR_RIGHT_REVERSE_DEADZONE_PM
#define BSP_MOTOR_RIGHT_REVERSE_DEADZONE_PM       ((BSP_MOTOR_DEADZONE_COMP_PM > 0) ? BSP_MOTOR_DEADZONE_COMP_PM : 60)
#endif

/**
 * 动摩擦死区（Running Dead Zone）—— 平衡车模式下的核心补偿参数。
 *
 * 功能：非零 PID 命令经线性映射后，最小实际输出被抬升到此门槛值。
 * 设定原则：≥ 实测静摩擦起转 PWM（确保任何非零命令都能让电机响应）。
 *
 * 平衡车模式下（static_dz_enabled = false）此值被无条件应用：
 *   - 不依赖编码器确认运动状态，不引入 kick 脉冲；
 *   - PID 输出 ±1‰ 时电机也能拿到 ≥门槛的实际电压；
 *   - 倒立摆物理特性 + D 项角速率响应保证 10-20ms 内自然突破静摩擦。
 *
 * 标定方法：对每路电机扫描找到"从 0 开始加 PWM，编码器首次产生计数"
 * 的门槛值，取该值 +10~15‰ 余量即可。
 */
#ifndef BSP_MOTOR_LEFT_FORWARD_RUNNING_DEADZONE_PM
#define BSP_MOTOR_LEFT_FORWARD_RUNNING_DEADZONE_PM   (43)
#endif
#ifndef BSP_MOTOR_LEFT_REVERSE_RUNNING_DEADZONE_PM
#define BSP_MOTOR_LEFT_REVERSE_RUNNING_DEADZONE_PM   (65)
#endif
#ifndef BSP_MOTOR_RIGHT_FORWARD_RUNNING_DEADZONE_PM
#define BSP_MOTOR_RIGHT_FORWARD_RUNNING_DEADZONE_PM  (50)
#endif
#ifndef BSP_MOTOR_RIGHT_REVERSE_RUNNING_DEADZONE_PM
#define BSP_MOTOR_RIGHT_REVERSE_RUNNING_DEADZONE_PM  (40)
#endif

/**
 * 已启动状态下连续多少毫秒没有编码器计数变化，就认为电机停转/卡住并切回
 * 静摩擦补偿重试。取值要大于低速下单个编码器计数的间隔，避免极低速抖动切换。
 */
#ifndef BSP_MOTOR_STATIC_RETRY_NO_MOTION_MS
#define BSP_MOTOR_STATIC_RETRY_NO_MOTION_MS          (80u)
#endif

/**
 * 静摩擦补偿幅值：直接复用各方向静摩擦标定值。
 * 如需独立调参（例如突破幅值要超过静摩擦更多），可在工程级 -D 覆盖本处别名。
 */
#define BSP_MOTOR_LEFT_FORWARD_KICK_PM    BSP_MOTOR_LEFT_FORWARD_DEADZONE_PM
#define BSP_MOTOR_LEFT_REVERSE_KICK_PM    BSP_MOTOR_LEFT_REVERSE_DEADZONE_PM
#define BSP_MOTOR_RIGHT_FORWARD_KICK_PM   BSP_MOTOR_RIGHT_FORWARD_DEADZONE_PM
#define BSP_MOTOR_RIGHT_REVERSE_KICK_PM   BSP_MOTOR_RIGHT_REVERSE_DEADZONE_PM

/**
 * 每输出轴一圈的编码器计数（实测定标值，优先于 gear×PPR×X 公式）。
 *
 * 标定方法：清零编码器 → 手转输出轴精确一圈 → 读 bsp_motor_get_left/right_count()。
 * 当前值来自实测：左轮手转一圈 left_count = 404。
 * 右轮尚未单独标定，暂与左轮一致（同型号电机，偏差通常 < 1%）。
 * 改此值后角度/速度换算自动正确，无需修改业务代码。
 */
#define BSP_MOTOR_LEFT_COUNTS_PER_OUTPUT_REV      (404)
#define BSP_MOTOR_RIGHT_COUNTS_PER_OUTPUT_REV     (404)

/**
 * 速度差分窗口（毫秒）：update() 每次累计 1 ms，达到该值时做一次 count 差分，
 * 得到 cps（counts per second）；窗口越小响应越快，但低速时分辨率越粗。
 * 默认 20 ms ⇒ 50 Hz 速度刷新率，最低可分辨速度
 *      左轮 = 1000/20 = 50 cps  ≈ 50/404 * 60 ≈ 7.43 rpm
 *      右轮 = 50 cps             ≈ 50/404 * 60 ≈ 7.43 rpm
 */
#ifndef BSP_MOTOR_SPEED_WINDOW_MS
#define BSP_MOTOR_SPEED_WINDOW_MS                  (10u)
#endif

/**
 * 右编码器 ISR 雪崩保护（Stage 2.4 新增）：
 *   ─ `BSP_MOTOR_ENC_IRQ_QUENCH_PER_MS`：单毫秒边沿率上限。手动转编码器极快也只
 *     会到几 kHz/ms 级别，超过该阈值唯一可能是引脚浮空 + 噪声 / 编码器电源异常
 *     引发的 ISR 雪崩。默认 200 = 200 kHz 边沿率告警线，远高于实际机械极限。
 *   ─ `BSP_MOTOR_ENC_IRQ_QUENCH_DURATION_MS`：触发后关闭 PA12/PA13 中断的毫秒数。
 *     期间编码器边沿丢失，但主循环 / SysTick / printf 能正常推进；到期后由
 *     `bsp_motor_update()` 重新打开。
 */
#ifndef BSP_MOTOR_ENC_IRQ_QUENCH_PER_MS
#define BSP_MOTOR_ENC_IRQ_QUENCH_PER_MS            (200u)
#endif

#ifndef BSP_MOTOR_ENC_IRQ_QUENCH_DURATION_MS
#define BSP_MOTOR_ENC_IRQ_QUENCH_DURATION_MS       (50u)
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
 * @brief 初始化电机驱动：清零计数与命令、STBY 拉低（待机）、注册右编码器中断。
 *
 * 前提：`bsp_gpio_init()` 已配好 AIN/BIN/STBY/PA12/PA13 的方向与上拉，
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
 *        STBY 保持原状态。**持续模式**：进入 brake 后保持，调用方负责按需切回 stop / set_output。
 *        仅在需要快速停车（如跌倒保护）时用；对 TB6612 持续注入大电流可能发热，
 *        100 ms 以内的脉冲式刹车请用 `bsp_motor_brake_pulse_ms()`。
 */
void bsp_motor_brake(void);

/**
 * @brief 脉冲式短刹车：进入 brake 态 N ms 后由 `bsp_motor_update()` 自动转 stop (coast)。
 *        N 由 SysTick 1 kHz 时基计时；调用本函数会立刻设 brake，不阻塞返回。
 *        典型用法：跌倒检测、急停事件触发 → `bsp_motor_brake_pulse_ms(80)` →
 *        80 ms 内电机已基本停下，转 coast 避免 brake 持续回灌大电流伤 TB6612。
 *
 *        本 API 与 `bsp_motor_brake()` (持续) 互斥：再调一次本函数会刷新计时器，
 *        调用 `bsp_motor_brake() / bsp_motor_stop() / bsp_motor_set_output*()` 中
 *        任意一个都会取消未到期的脉冲计时。
 *
 * @param duration_ms  brake 持续毫秒数，推荐 50~150；0 = 立即转 stop（等价 `bsp_motor_stop()`）
 */
void bsp_motor_brake_pulse_ms(uint32_t duration_ms);

/**
 * @brief 死区补偿总开关（master switch）。
 *        false → 跳过全部死区逻辑（静态 + 动态），所有命令只经 apply_limit 后直通硬件。
 *        true  → 由 static_dz / running_dz 两个分开关分别控制各自逻辑。
 *        默认开启。
 */
void bsp_motor_set_deadzone_comp_enabled(bool enabled);

/** 查询死区补偿总开关状态。 */
bool bsp_motor_get_deadzone_comp_enabled(void);

/**
 * @brief 静摩擦死区补偿分开关。
 *        false → 跳过 apply_static_deadzone()，电机未启动时也不做起转 kick，
 *                命令直通限幅值（适合 PID 调试时观察线性小 PWM 响应）。
 *        true  → 保持原有行为：未确认运动 / cal 模式 → 静摩擦门槛映射（默认）。
 *        切换时立即以新参数重发当前命令。总开关 deadzone_comp_enabled 为 false 时
 *        本开关无效。
 */
void bsp_motor_set_static_dz_enabled(bool enabled);

/** 查询静摩擦死区补偿分开关状态。 */
bool bsp_motor_get_static_dz_enabled(void);

/**
 * @brief 动摩擦死区补偿分开关。
 *        false → 跳过 apply_running_deadzone()，编码器确认已启动后命令仍直通限幅值，
 *                不做动摩擦线性重映射（适合分析原始 PWM→速度曲线）。
 *        true  → 保持原有行为：已确认运动 → 动摩擦线性区映射（默认）。
 *        切换时立即以新参数重发当前命令。总开关 deadzone_comp_enabled 为 false 时
 *        本开关无效。
 */
void bsp_motor_set_running_dz_enabled(bool enabled);

/** 查询动摩擦死区补偿分开关状态。 */
bool bsp_motor_get_running_dz_enabled(void);

/**
 * @brief 切换静态死区强制模式（legacy cal-mode）。
 *
 *        默认 false：未确认运动时用 *_DEADZONE_PM；编码器确认已转动后用
 *                    RUNNING_DEADZONE_PM，停转则切回静摩擦重试。
 *        true：commit 始终用 *_DEADZONE_PM 做线性映射，便于复现旧版
 *              static-only 校准扫描。
 *
 *        当前 app_motor_demo 校准扫描默认保持 false，以验证真实运行补偿系统。
 */
void bsp_motor_set_calibration_mode(bool enabled);

/** 查询当前是否处于静态死区强制模式。 */
bool bsp_motor_get_calibration_mode(void);

/**
 * @brief 开关右路正转比例补偿。
 *        默认开启；校准扫描若需要采集原始 PWM→RPM 曲线，可临时关闭并在结束后恢复。
 */
void bsp_motor_set_right_forward_scale_enabled(bool enabled);

/** 查询右路正转比例补偿是否启用。 */
bool bsp_motor_get_right_forward_scale_enabled(void);

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

/** 当前左轮实际 PWM 占空比 permille（已应用限幅、补偿、scale、极性翻转）。 */
int16_t bsp_motor_get_left_actual_pwm(void);

/** 当前右轮实际 PWM 占空比 permille（已应用限幅、补偿、scale、极性翻转）。 */
int16_t bsp_motor_get_right_actual_pwm(void);

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
 * @brief 读取右编码器 ISR 累计进入次数（雪崩 / 噪声诊断）。
 *        正常调用：1 Hz 心跳里 `cur = bsp_motor_get_enc_irq_count(); delta = cur - prev`，
 *        delta 应等于 1 s 内编码器边沿数，远小于 BSP_MOTOR_ENC_IRQ_QUENCH_PER_MS × 1000。
 *        若 delta 突然飙到几十万 / 雪崩兜底触发，说明引脚浮空或编码器电源问题。
 */
uint32_t bsp_motor_get_enc_irq_count(void);

/** @return ISR 雪崩兜底当前是否激活（剩余抑制毫秒数 > 0 即为 true）。 */
bool bsp_motor_enc_irq_is_quenched(void);

/**
 * @brief 同时清零左右轮累计计数与速度差分窗口。常用于上电校准、调试归零、
 *        或上一次跑车结束后准备下一次试跑。不影响命令与 STBY 状态。
 */
void bsp_motor_reset_encoders(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_MOTOR_H */
