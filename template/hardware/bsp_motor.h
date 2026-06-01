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
 *   ─ 右轮编码器：PA12 → TIMG0_CCP0 硬件捕获中断（X1，仅上升沿）+ PA13 ISR
 *      内读电平判方向。独立 TIMG0 中断向量、优先级可控，彻底脱离 GROUP1
 *      共享入口与 GPIO 雪崩关断逻辑（Stage 3.x 迁移，见下方 DECODE_X 注释）
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
#define BSP_MOTOR_GB370_GEAR_RATIO                 (34.0f)

/** GB370 内置霍尔每电机轴转脉冲数（A 相单沿） */
#define BSP_MOTOR_GB370_HALL_PPR                   (500)

/** 左轮 QEI mode 3 = X4 解码，每输出轴一圈的计数 */
#define BSP_MOTOR_LEFT_DECODE_X                    (4)

/**
 * 右轮解码倍率（Stage 3.x：GPIO 双沿中断 → TIMG0 捕获中断迁移）。
 *
 * 旧方案（GPIO 双沿 + GROUP1 共享入口）的致命问题：
 *   X2 在 ≈530 RPM 出轴侧产生 300 边/ms，**恰好命中下方雪崩兜底阈值 300**
 *   → 右编码器中断被强制关闭 50 ms → right_count 冻结 → 右轮速度瞬间读 0。
 *   大外力推车（高倾角→高速纠偏）时这正是"左轮远快于右轮、整车自转一圈"
 *   的根因：差速环看到 L≫R 误判，反而加大左右差速 → 正反馈自转。
 *
 * 新方案：PA12 → TIMG0_CCP0 硬件捕获，X1（仅上升沿），PA13 在 ISR 内读电平
 * 判方向。本器件仅 TIMG8 支持硬件 QEI（左轮已占），无第二路 QEI，故右轮迁
 * 到定时器捕获中断；编码器分辨率足够，X1 即可（17,000 cnt/rev）：
 *   ─ 独立 TIMG0_IRQHandler 向量，不再与 GPIOA/GPIOB 共享 GROUP1；
 *   ─ X1 → 530 RPM 下 150 边/ms（X2 的一半），ISR CPU 占用 ≈ 13%；
 *   ─ 雪崩阈值抬到 1000（见下），正常高速 150 边/ms 有 6× 余量，
 *     仅在引脚浮空噪声（数万边/ms）时才触发，绝不会误关正常高速读数。
 *
 * 注：本宏现仅用于 cnt/rev 与文档表述；解码逻辑在 bsp_motor.c 固定为
 *     "TIMG0 捕获 PA12 上升沿 + 读 PA13"，不再随本宏分支。
 */
#ifndef BSP_MOTOR_RIGHT_DECODE_X
#define BSP_MOTOR_RIGHT_DECODE_X                   (1)
#endif

/**
 * 右路正转静态补偿：校准显示同一 PWM 下右路正转约快 5.22%，因此在 BSP 层
 * 对右路正向命令统一按 95% 输出。放在 BSP 而非 app 层，避免 demo / balance /
 * K230 等不同业务路径漏补偿或重复造轮子。
 * 可通过 `bsp_motor_set_right_forward_scale_enabled(false)` 临时关闭，用于校准
 * 扫描原始未补偿曲线。
 */
#ifndef BSP_MOTOR_RIGHT_FORWARD_SCALE_X1000
#define BSP_MOTOR_RIGHT_FORWARD_SCALE_X1000        (1000)
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
 * 功能：非零 PID 命令经线性映射后，最小实际输出被抬升到此门槛值，
 *       确保任何非零命令均能让电机实际转动，消除正反向死区不对称导致
 *       的"小倾角单侧反转"现象。
 *
 * 设定原则：≥ 实测静摩擦起转 PWM（确保任何非零命令都能让电机响应）。
 *
 * 平衡车模式下（static_dz_enabled = false）此值被无条件应用：
 *   - 不依赖编码器确认运动状态，不引入 kick 脉冲；
 *   - PID 输出 ±1‰ 时电机也能拿到 ≥门槛的实际电压；
 *   - 倒立摆物理特性 + D 项角速率响应保证 10-20ms 内自然突破静摩擦。
 *
 * 标定方法（每次换新电机后必做）：
 *   在 app_motor_demo 标定模式下，对每路电机从 0 缓慢扫描 PWM（步进 1‰，
 *   每步等 50 ms），记录编码器计数首次发生变化时的 PWM 值；
 *   取该值 +10~15‰ 余量填入对应宏。
 *
 * 当前值（2026-05-16 临时占位，500PPR×34:1 新电机待标定）：
 *   参考旧 GB370（9.6:1）标定值约 50-68‰，取保守上界 65‰ 作为初始值。
 *   标定完成后替换为实测值，预计可降至 45~55‰ 区间。
 *
 * ⚠ 若 running_dz 设为 0：小命令直接原路返回 → 低于物理死区 → 电机不响应；
 *   正反向物理死区不对称 → 某方向先突破 → 出现单侧反转。
 */
#ifndef BSP_MOTOR_LEFT_FORWARD_RUNNING_DEADZONE_PM
#define BSP_MOTOR_LEFT_FORWARD_RUNNING_DEADZONE_PM   (40)   /* TODO: 标定后替换 */
#endif
#ifndef BSP_MOTOR_LEFT_REVERSE_RUNNING_DEADZONE_PM
#define BSP_MOTOR_LEFT_REVERSE_RUNNING_DEADZONE_PM   (40)   /* TODO: 标定后替换 */
#endif
#ifndef BSP_MOTOR_RIGHT_FORWARD_RUNNING_DEADZONE_PM
#define BSP_MOTOR_RIGHT_FORWARD_RUNNING_DEADZONE_PM  (40)   /* TODO: 标定后替换 */
#endif
#ifndef BSP_MOTOR_RIGHT_REVERSE_RUNNING_DEADZONE_PM
#define BSP_MOTOR_RIGHT_REVERSE_RUNNING_DEADZONE_PM  (40)   /* TODO: 标定后替换 */
#endif

/**
 * Sigma-delta dither 累加器触发门槛（permille）。
 *   控制脉冲发射频率：累加器达到此值时触发一次脉冲。
 *   值越小 → 脉冲越频繁 → 电机响应越快（但平滑度略降）。
 *   建议与 running_dz 相同或略小。
 */
#ifndef BSP_MOTOR_DITHER_THRESHOLD_PM
#define BSP_MOTOR_DITHER_THRESHOLD_PM                (20)
#endif

/**
 * Sigma-delta dither 脉冲输出幅度（permille）。
 *   控制每次脉冲的 PWM 强度，需 > 死区阈值以确保负载下有效驱动。
 *   与 threshold 独立配置；pulse > threshold 时引入增益因子 = pulse/threshold，
 *   等效于子死区区域输出被放大，PID 增益需相应缩小。
 *   取值建议：刚好能可靠驱动负载的最小值（如 120~200）。
 */
#ifndef BSP_MOTOR_DITHER_PULSE_PM
#define BSP_MOTOR_DITHER_PULSE_PM                    (200)
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
 *
 * 左轮（TIMG8 硬件 QEI，X4）：500 PPR × 34:1 × 4 = 68,000 cnt/rev
 *   最低可分辨速度（10ms 窗口）：1000/10 / 68000 × 60 ≈ 0.09 rpm
 *
 * 右轮（TIMG0 捕获 ISR，X1）：500 PPR × 34:1 × 1 = 17,000 cnt/rev
 *   仅捕获 PA12 上升沿 + 读 PA13 判向，530 RPM 下边沿率 150 边/ms，
 *   ISR CPU 占用约 13%；无第二路硬件 QEI（仅 TIMG8 支持，左轮已占），
 *   故采用 TIMG0 捕获中断方案，详见上方 BSP_MOTOR_RIGHT_DECODE_X 注释。
 *   最低可分辨速度（10ms 窗口）：1000/10 / 17000 × 60 ≈ 0.35 rpm（足够）
 */
#define BSP_MOTOR_LEFT_COUNTS_PER_OUTPUT_REV      (68000)
#define BSP_MOTOR_RIGHT_COUNTS_PER_OUTPUT_REV     (17000)

/**
 * 速度差分窗口（毫秒）：update() 每次累计 1 ms，达到该值时做一次 count 差分，
 * 得到 cps（counts per second）；窗口越小响应越快，但低速时分辨率越粗。
 * 当前 10 ms ⇒ 100 Hz 速度刷新率，最低可分辨速度
 *      左轮 = 1000/10 = 100 cps  ≈ 100/68000 × 60 ≈ 0.09 rpm（硬件 QEI X4）
 *      右轮 = 100 cps             ≈ 100/34000 × 60 ≈ 0.18 rpm（GPIO ISR X2）
 */
#ifndef BSP_MOTOR_SPEED_WINDOW_MS
#define BSP_MOTOR_SPEED_WINDOW_MS                  (10u)
#endif

/**
 * 编码器速度 EMA 低通滤波系数（0~1）。
 *
 * 每次速度窗口差分完成后（100 Hz），对各轮 cps 施加一阶 EMA：
 *   lpf += alpha × (raw - lpf)
 * α=0.3 @ 100 Hz → 时间常数 ~30 ms，衰减编码器量化噪声的同时保持
 * 足够快的响应。差速环反馈 (L-R) 对噪声敏感，此滤波尤为关键。
 */
#ifndef BSP_MOTOR_SPEED_LPF_ALPHA
#define BSP_MOTOR_SPEED_LPF_ALPHA                  (0.3f)
#endif

/**
 * 右编码器 ISR 雪崩保护（Stage 2.4 引入，Stage 3.x 迁 TIMG0 捕获后调整）：
 *   ─ `BSP_MOTOR_ENC_IRQ_QUENCH_PER_MS`：单毫秒捕获中断率上限（次/ms）。
 *     低于此值：正常计数。超过此值：判定为噪声/浮空雪崩，关 TIMG0 CC0 中断保护 CPU。
 *
 *     500 PPR × 34:1 电机在 X1 捕获（仅 PA12 上升沿）下：
 *       180 RPM → 51 次/ms，最高约 530 RPM → 150 次/ms。
 *     ⚠️  旧值 300 在 X2 下恰好等于 530 RPM 的边沿率 → 正常高速即触发雪崩
 *           误关、右轮速度归零（自转根因）。X1 迁移后阈值抬到 1000，对正常
 *           高速 150 次/ms 有 6× 余量，仅在引脚浮空噪声（数万次/ms）时触发。
 *
 *   ─ `BSP_MOTOR_ENC_IRQ_QUENCH_DURATION_MS`：触发后关闭 TIMG0 CC0 中断的毫秒数。
 *     期间编码器边沿丢失，但主循环 / SysTick / printf 能正常推进；到期后由
 *     `bsp_motor_update()` 重新打开。
 */
#ifndef BSP_MOTOR_ENC_IRQ_QUENCH_PER_MS
#define BSP_MOTOR_ENC_IRQ_QUENCH_PER_MS            (1000u)
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
 * @brief 启用/禁用 Sigma-Delta dither 死区模式。
 *
 *        启用后替代 running_dz 逻辑：对低于死区阈值的命令使用累加器产生
 *        时间平均脉冲输出，避免高频正负切换引起机械振颤。
 *        建议同时禁用 running_dz 以避免冲突。
 */
void bsp_motor_set_dither_dz_enabled(bool enabled);

/** 查询 sigma-delta dither 死区模式状态。 */
bool bsp_motor_get_dither_dz_enabled(void);

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
