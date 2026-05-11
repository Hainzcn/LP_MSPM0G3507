/**
 * @file    bsp_motor.c
 * @brief   阶段 2 电机底层驱动实现 —— TB6612 + 左右编码器。
 *
 * 内部状态聚合到 `s_motor` 单例：
 *   ─ ISR 共享字段（counts / 编码器中断诊断）放前面，访问
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
    volatile int32_t  right_count;      /* 由 PA12 (X2 时) 或 PA12+PA13 (X4 时) ISR 维护 */
    volatile uint32_t enc_irq_count;    /* 右编码器 ISR 总进入次数（雪崩诊断） */
    volatile uint32_t enc_irq_window;   /* 1 ms 窗口内 ISR 进入次数 */
    volatile uint8_t  enc_irq_quench_remain_ms;  /* 雪崩兜底：剩余抑制毫秒数 */

    /* --- update() 私有：QEI 16-bit 上一拍原值 -------------------------------- */
    uint16_t left_raw_prev;

    /* --- 速度差分窗口（主循环私有） ------------------------------------------ */
    int32_t  left_speed_prev_count;
    int32_t  right_speed_prev_count;
    uint32_t speed_window_acc_ms;
    int32_t  left_speed_cps;
    int32_t  right_speed_cps;

    /* --- 脉冲刹车定时（主循环私有，update() 内倒计时；0 = 无 pending） -------- */
    uint32_t brake_pulse_remain_ms;

    /* --- 命令与配置 --------------------------------------------------------- */
    int16_t  left_cmd_pm;
    int16_t  right_cmd_pm;
    int16_t  left_actual_pwm_pm;
    int16_t  right_actual_pwm_pm;
    uint16_t pwm_limit_pm;
    bool     invert_left;
    bool     invert_right;
    bool     deadzone_comp_enabled;
    bool     calibration_mode;     /* true: commit 走静态 DZ + 无 kick；详见 .h */
    bool     enabled;

    /* --- Kick 启动脉冲状态（主循环私有：仅 commit_* 与 update() 访问） -------
     *   prev_dir:        +1 / 0 / -1，跟踪上一次写到硬件的物理方向（已应用
     *                    极性翻转之前），用于检测方向跳变；
     *   kick_remain_ms:  剩余 kick 毫秒数，0 = 无 kick。
     *   方向跳变（0→非零 或 +→-/-→+）时由 commit_* 置 BSP_MOTOR_KICK_MS；
     *   update() 1 kHz 递减；到期后 commit_* 自动切回 running deadzone 路径。 */
    int8_t   left_prev_dir;
    int8_t   right_prev_dir;
    uint16_t left_kick_remain_ms;
    uint16_t right_kick_remain_ms;
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

/** 按方向 / 左右取动摩擦"running"死区（稳态线性区基准）。 */
static int32_t get_running_dz_pm(int16_t permille, bool is_left)
{
    if (permille > 0) {
        return is_left ? BSP_MOTOR_LEFT_FORWARD_RUNNING_DEADZONE_PM
                       : BSP_MOTOR_RIGHT_FORWARD_RUNNING_DEADZONE_PM;
    }
    if (permille < 0) {
        return is_left ? BSP_MOTOR_LEFT_REVERSE_RUNNING_DEADZONE_PM
                       : BSP_MOTOR_RIGHT_REVERSE_RUNNING_DEADZONE_PM;
    }
    return 0;
}

/** 按方向 / 左右取静摩擦 kick 幅值（=原 *_DEADZONE_PM 标定值）。 */
static int32_t get_kick_pm(int16_t permille, bool is_left)
{
    if (permille > 0) {
        return is_left ? BSP_MOTOR_LEFT_FORWARD_KICK_PM
                       : BSP_MOTOR_RIGHT_FORWARD_KICK_PM;
    }
    if (permille < 0) {
        return is_left ? BSP_MOTOR_LEFT_REVERSE_KICK_PM
                       : BSP_MOTOR_RIGHT_REVERSE_KICK_PM;
    }
    return 0;
}

/**
 * 死区线性重映射核：[-lim, -dz] ∪ {0} ∪ [dz, +lim]。
 *   0 仍为 coast；非零命令按 dz 抬升到死区门槛以上，同时按当前限幅重映射，
 *   保证逻辑满量程仍对应物理满量程，不会因简单相加而提前饱和。
 *   `dz` 由调用方按 running / static 表选好后传入；本函数不关心 dz 来源。
 */
static int16_t apply_deadzone_mapping(int16_t permille, int32_t dz)
{
    if (permille == 0) {
        return 0;
    }
    if (dz <= 0) {
        return permille;
    }

    int32_t lim = (int32_t)s_motor.pwm_limit_pm;
    if (lim > BSP_MOTOR_PWM_MAX_PERMILLE) {
        lim = BSP_MOTOR_PWM_MAX_PERMILLE;
    }
    if (lim <= 0) {
        return 0;
    }

    if (dz >= lim) {
        return (permille > 0) ? (int16_t)lim : (int16_t)-lim;
    }

    int32_t mag = (permille > 0) ? (int32_t)permille : -(int32_t)permille;
    if (mag > lim) {
        mag = lim;
    }
    mag = dz + ((mag * (lim - dz)) / lim);
    return (permille > 0) ? (int16_t)mag : (int16_t)-mag;
}

/** Running 死区映射（稳态线性区，平衡车 / 默认运行模式）。 */
static int16_t apply_running_deadzone(int16_t permille, bool is_left)
{
    return apply_deadzone_mapping(permille, get_running_dz_pm(permille, is_left));
}

/** Static 死区映射（= 静摩擦门槛，cal 模式 / 旧单门槛行为）。 */
static int16_t apply_static_deadzone(int16_t permille, bool is_left)
{
    return apply_deadzone_mapping(permille, get_kick_pm(permille, is_left));
}

/**
 * Kick "地板"：把已 deadzone 映射的输出抬升到至少静摩擦突破值（受限幅约束）。
 *   ─ 对小命令（如 PID 在零点附近）：等价于"50 ms 内固定输出 KICK_PM"；
 *   ─ 对大命令（如 demo 全速）：抬升不生效，避免产生"先低后高"的扭矩悬崖。
 *   ─ 输入 dz_mapped_pm 已含方向（正/负），输出维持同符号。
 */
static int16_t apply_kick_floor(int16_t dz_mapped_pm, bool is_left)
{
    if (dz_mapped_pm == 0) {
        return 0;
    }

    int32_t kick = get_kick_pm(dz_mapped_pm, is_left);
    if (kick <= 0) {
        return dz_mapped_pm;
    }

    int32_t lim = (int32_t)s_motor.pwm_limit_pm;
    if (lim > BSP_MOTOR_PWM_MAX_PERMILLE) {
        lim = BSP_MOTOR_PWM_MAX_PERMILLE;
    }
    if (kick > lim) {
        kick = lim;
    }

    int32_t mag = (dz_mapped_pm > 0) ? (int32_t)dz_mapped_pm : -(int32_t)dz_mapped_pm;
    if (mag < kick) {
        mag = kick;
    }
    return (dz_mapped_pm > 0) ? (int16_t)mag : (int16_t)-mag;
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

/**
 * 通道公共提交：完整流程  limit → kick state → right_fwd_scale → deadzone(+kick) →
 *                          invert → 写硬件。
 *
 * 两种死区路径（由 s_motor.calibration_mode 决定）：
 *   ─ calibration_mode = false（默认 / 平衡车 / motor demo 运行）：
 *       deadzone = running_dz，方向跳变时叠加 50 ms kick floor（双门槛 + 启动脉冲）。
 *   ─ calibration_mode = true（校准扫描）：
 *       deadzone = static_dz（= KICK_PM），不应用 kick floor。整段扫描走旧
 *       单门槛行为，"DZ in data" 反映真实有效死区，可用于验证补偿是否生效。
 *       cal 模式下 kick 计时器仍维护，但不影响输出 —— 切回非 cal 模式后立即生效。
 *
 * Kick 状态机（始终维护，与 cal_mode 无关）：
 *   ─ new_dir 取自**逻辑 cmd 的符号**（不含极性翻转 / 右路 scale，避免极性翻转
 *     被错认为方向跳变）；
 *   ─ new_dir == 0：清 kick，下次启动会重新触发；
 *   ─ new_dir != 0 且 new_dir != prev_dir：装载 BSP_MOTOR_KICK_MS。
 *   ─ kick 计时在 update() 1 kHz 路径递减，到期后再次 commit 让输出退到 running
 *     deadzone。
 */
static void commit_channel(int16_t cmd_pm, bool is_left)
{
    int8_t   new_dir         = (cmd_pm > 0) ? (int8_t)1
                              : (cmd_pm < 0) ? (int8_t)-1 : (int8_t)0;
    int8_t  *prev_dir_p      = is_left ? &s_motor.left_prev_dir
                                       : &s_motor.right_prev_dir;
    uint16_t *kick_remain_p  = is_left ? &s_motor.left_kick_remain_ms
                                       : &s_motor.right_kick_remain_ms;

    if (new_dir == 0) {
        *kick_remain_p = 0u;
    } else if (new_dir != *prev_dir_p) {
        *kick_remain_p = BSP_MOTOR_KICK_MS;
    }
    *prev_dir_p = new_dir;

    int16_t pm = apply_limit(cmd_pm);
    if (!is_left && pm > 0) {
        pm = (int16_t)((((int32_t)pm * BSP_MOTOR_RIGHT_FORWARD_SCALE_X1000) + 500) / 1000);
    }

    if (s_motor.deadzone_comp_enabled) {
        if (s_motor.calibration_mode) {
            pm = apply_static_deadzone(pm, is_left);
        } else {
            pm = apply_running_deadzone(pm, is_left);
            if (*kick_remain_p > 0u) {
                pm = apply_kick_floor(pm, is_left);
            }
        }
    }

    bool invert = is_left ? s_motor.invert_left : s_motor.invert_right;
    if (invert) {
        pm = (int16_t)(-(int32_t)pm);
    }
    if (is_left) {
        s_motor.left_actual_pwm_pm = pm;
    } else {
        s_motor.right_actual_pwm_pm = pm;
    }
    apply_one_channel(pm, is_left);
}

static void commit_left(int16_t cmd_pm)
{
    commit_channel(cmd_pm, true);
}

static void commit_right(int16_t cmd_pm)
{
    commit_channel(cmd_pm, false);
}

/* ========================================================================== */
/* 中断辅助                                                                     */
/* ========================================================================== */

/**
 * 右编码器边沿中断触发：X2 时仅 PA12 双沿；X4 时 PA12 + PA13 都双沿。
 *
 * 标准正交解码状态机（X4）：
 *   读出 (A, B) 当前电平，根据"哪个相刚刚跳变 + 跳变方向"得到 ±1 步。
 *   实现等价表达式：A == B 则 +1 / != 则 -1（在"PA12 边沿事件"上等价于
 *   X2 经典做法；在 X4 + PA13 边沿时也成立，但符号相反 —— 因为 PA13 跳变
 *   时 A/B 异同关系正好是 PA12 跳变时的反相）。所以本函数把"哪根线触发"
 *   作为参数，PA13 触发时步进取反。这样 X2 / X4 共用同一份解码代码。
 *
 *   X2 (默认 RIGHT_DECODE_X = 2)：only `is_phase_a_edge = true` 路径生效
 *   X4 (RIGHT_DECODE_X = 4)：PA12 与 PA13 各自调用，分别 true / false
 */
static void on_right_encoder_edge(bool is_phase_a_edge)
{
    s_motor.enc_irq_count++;
    s_motor.enc_irq_window++;

    uint32_t pins = DL_GPIO_readPins(GPIOA, BSP_ENC_R_A_PIN | BSP_ENC_R_B_PIN);
    bool phase_a = ((pins & BSP_ENC_R_A_PIN) != 0u);
    bool phase_b = ((pins & BSP_ENC_R_B_PIN) != 0u);

    int32_t step = (phase_a != phase_b) ? 1 : -1;
    if (!is_phase_a_edge) {
        step = -step;
    }
    /* 右轮编码器安装方向与左轮相反：正转命令下物理边沿给出负计数。
     * 这里翻转反馈符号，让左右轮在同向命令下 cps / count 同号，便于速度环共用。 */
    step = -step;
    s_motor.right_count += step;
}

/* ========================================================================== */
/* 公共 API：初始化 / 使能                                                      */
/* ========================================================================== */

void bsp_motor_init(void)
{
    /* --- 0) 给左轮 QEI (PB15/PB16) 补内部上拉 + 施密特滞回 -------------------
     *   SYSCFG_DL_GPIO_init() 已在 main 顶部把 PB15/PB16 mux 到 TIMG8_CCP0/1，
     *   但 SDK 的 `DL_GPIO_initPeripheralInputFunction()` 不会配 pull/hyst，
     *   PINCM32/33 默认 RESISTOR_NONE + HYSTERESIS_DISABLE → 编码器掉电或线
     *   未接时 PB15/PB16 浮空 → AC 噪声让 QEI 计数器乱跳 / 不动；这里二次写
     *   IOMUX 寄存器追加 PULL_UP + HYSTERESIS_ENABLE，与右轮 PA12/PA13（在
     *   bsp_gpio.c 里已配过）保持一致的稳健度。
     *   `DL_GPIO_initPeripheralInputFunctionFeatures` 与原 `*Function` 写同一
     *   PINCM 槽，只是多带了 inversion/pull/hyst 字段。 */
    DL_GPIO_initPeripheralInputFunctionFeatures(
        GPIO_QEI_LEFT_PHA_IOMUX, GPIO_QEI_LEFT_PHA_IOMUX_FUNC,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(
        GPIO_QEI_LEFT_PHB_IOMUX, GPIO_QEI_LEFT_PHB_IOMUX_FUNC,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);

    /* --- 1) 状态清零 ------------------------------------------------------- */
    s_motor.left_count             = 0;
    s_motor.right_count            = 0;

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
    s_motor.left_actual_pwm_pm     = 0;
    s_motor.right_actual_pwm_pm    = 0;
    s_motor.pwm_limit_pm           = BSP_MOTOR_PWM_MAX_PERMILLE;
    s_motor.invert_left            = false;
    s_motor.invert_right           = false;
    s_motor.deadzone_comp_enabled  = true;
    s_motor.calibration_mode       = false;
    s_motor.enabled                = false;
    s_motor.left_prev_dir          = 0;
    s_motor.right_prev_dir         = 0;
    s_motor.left_kick_remain_ms    = 0u;
    s_motor.right_kick_remain_ms   = 0u;

    /* --- 2) 安全态：方向位清零 + PWM 占空 0 + STBY 拉低 -------------------- */
    set_dir_left (DIR_COAST);
    set_dir_right(DIR_COAST);
    set_pwm_duty(GPIO_PWM_MOTOR_C0_IDX, 0u);
    set_pwm_duty(GPIO_PWM_MOTOR_C1_IDX, 0u);
    DL_GPIO_clearPins(BSP_STBY_PORT, BSP_STBY_PIN);

    /* --- 3) PA12 双沿 (+ X4 时 PA13 双沿) 中断 ---------------------------
     *   双沿：单独用 _EDGE_RISE 与 _EDGE_FALL 按位或；某些 SDK 版本没有
     *   `_EDGE_RISE_FALL` 这个组合宏，按位或写法在所有 SDK 里都成立。
     *
     *   X4 模式（默认）：A、B 都开双沿，分辨率 1320 cnt/rev，与左轮一致；
     *   X2 模式（编译期 -DBSP_MOTOR_RIGHT_DECODE_X=2）：只开 PA12 双沿，
     *   分辨率 660 cnt/rev，CPU ISR 减半。 */
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
    DL_GPIO_clearInterruptStatus(GPIOA, enc_pins);
    DL_GPIO_enableInterrupt    (GPIOA, enc_pins);

    /* 把 GPIOA 中断设为最低优先级（MSPM0G3507 __NVIC_PRIO_BITS = 2 → 3 = 最低），
     * 与 `bsp_systick.c` 把 SysTick 提到 0 = 最高 配套使用：
     *   ─ 任何编码器噪声引发的 GPIOA ISR 雪崩都不能阻断 SysTick；
     *   ─ 主循环节拍、电池采样、心跳日志即使在 ISR 高频时也能正常推进。
     * 副作用：编码器边沿与其他外设（UART/ADC）同时到达时，UART/ADC 优先服务，
     *         编码器最多迟几微秒；对 1 kHz 控制环影响可忽略。 */
    NVIC_ClearPendingIRQ(GPIOA_INT_IRQn);
    NVIC_SetPriority    (GPIOA_INT_IRQn, 3u);
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
    s_motor.brake_pulse_remain_ms = 0u;   /* 任何新命令都取消未到期 pulse */
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

/** 清 kick 状态：方向归零、计时归零。stop / brake 后下次启动会重新触发 kick。 */
static void clear_kick_state(void)
{
    s_motor.left_prev_dir         = 0;
    s_motor.right_prev_dir        = 0;
    s_motor.left_kick_remain_ms   = 0u;
    s_motor.right_kick_remain_ms  = 0u;
}

void bsp_motor_stop(void)
{
    s_motor.brake_pulse_remain_ms = 0u;
    s_motor.left_cmd_pm  = 0;
    s_motor.right_cmd_pm = 0;
    s_motor.left_actual_pwm_pm = 0;
    s_motor.right_actual_pwm_pm = 0;
    clear_kick_state();
    set_dir_left (DIR_COAST);
    set_dir_right(DIR_COAST);
    set_pwm_duty(GPIO_PWM_MOTOR_C0_IDX, 0u);
    set_pwm_duty(GPIO_PWM_MOTOR_C1_IDX, 0u);
}

void bsp_motor_brake(void)
{
    s_motor.brake_pulse_remain_ms = 0u;   /* 持续模式，不会自动转 stop */
    s_motor.left_cmd_pm  = 0;
    s_motor.right_cmd_pm = 0;
    s_motor.left_actual_pwm_pm = 0;
    s_motor.right_actual_pwm_pm = 0;
    clear_kick_state();
    set_dir_left (DIR_BRAKE);
    set_dir_right(DIR_BRAKE);
    set_pwm_duty(GPIO_PWM_MOTOR_C0_IDX, BSP_MOTOR_PWM_MAX_PERMILLE);
    set_pwm_duty(GPIO_PWM_MOTOR_C1_IDX, BSP_MOTOR_PWM_MAX_PERMILLE);
}

void bsp_motor_brake_pulse_ms(uint32_t duration_ms)
{
    if (duration_ms == 0u) {
        bsp_motor_stop();
        return;
    }
    /* 与持续 brake 复用硬件命令，但置位计时器；update() 倒计时到 0 后自动转 stop */
    s_motor.left_cmd_pm  = 0;
    s_motor.right_cmd_pm = 0;
    s_motor.left_actual_pwm_pm = 0;
    s_motor.right_actual_pwm_pm = 0;
    clear_kick_state();
    set_dir_left (DIR_BRAKE);
    set_dir_right(DIR_BRAKE);
    set_pwm_duty(GPIO_PWM_MOTOR_C0_IDX, BSP_MOTOR_PWM_MAX_PERMILLE);
    set_pwm_duty(GPIO_PWM_MOTOR_C1_IDX, BSP_MOTOR_PWM_MAX_PERMILLE);
    s_motor.brake_pulse_remain_ms = duration_ms;
}

void bsp_motor_set_deadzone_comp_enabled(bool enabled)
{
    if (s_motor.deadzone_comp_enabled == enabled) {
        return;
    }

    s_motor.deadzone_comp_enabled = enabled;
    commit_left (s_motor.left_cmd_pm);
    commit_right(s_motor.right_cmd_pm);
}

bool bsp_motor_get_deadzone_comp_enabled(void)
{
    return s_motor.deadzone_comp_enabled;
}

void bsp_motor_set_calibration_mode(bool enabled)
{
    if (s_motor.calibration_mode == enabled) {
        return;
    }

    s_motor.calibration_mode = enabled;
    /* 立即让新模式生效，避免"已切 cal 但旧命令仍按运行 dz 跑"的窗口。 */
    commit_left (s_motor.left_cmd_pm);
    commit_right(s_motor.right_cmd_pm);
}

bool bsp_motor_get_calibration_mode(void)
{
    return s_motor.calibration_mode;
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

int16_t bsp_motor_get_left_actual_pwm(void)
{
    return s_motor.left_actual_pwm_pm;
}

int16_t bsp_motor_get_right_actual_pwm(void)
{
    return s_motor.right_actual_pwm_pm;
}

/* ========================================================================== */
/* 公共 API：编码器 / update / feedback                                         */
/* ========================================================================== */

void bsp_motor_update(void)
{
    /* --- 0) brake pulse 倒计时（先做，避免本拍命令到期后还多跑 1 ms 才转 stop） */
    if (s_motor.brake_pulse_remain_ms != 0u) {
        s_motor.brake_pulse_remain_ms--;
        if (s_motor.brake_pulse_remain_ms == 0u) {
            /* 到期：转 coast。直接调 stop 会清 brake_pulse 计数（已是 0，无影响） */
            bsp_motor_stop();
        }
    }

    /* --- 0.2) Kick 启动脉冲倒计时（左右独立）-------------------------------
     *   到期后再次调用 commit_left/right(cmd_pm)：kick_remain 已归零，输出会从
     *   "静摩擦地板 ≥ KICK_PM" 切换到"running deadzone 线性映射"，避免大命令在
     *   首 50 ms 被压在 KICK_PM 上的"扭矩悬崖"。 */
    if (s_motor.left_kick_remain_ms != 0u) {
        s_motor.left_kick_remain_ms--;
        if (s_motor.left_kick_remain_ms == 0u) {
            commit_left(s_motor.left_cmd_pm);
        }
    }
    if (s_motor.right_kick_remain_ms != 0u) {
        s_motor.right_kick_remain_ms--;
        if (s_motor.right_kick_remain_ms == 0u) {
            commit_right(s_motor.right_cmd_pm);
        }
    }

    /* --- 0.5) ISR 雪崩兜底：每毫秒检查窗口内 ISR 进入次数 -------------------
     *   `BSP_MOTOR_ENC_IRQ_QUENCH_PER_MS`（默认 200）= 单毫秒边沿率上限。手动
     *   转编码器极快也只会到几 kHz/ms，超过该阈值唯一可能是引脚浮空 + 噪声、
     *   或编码器电源异常引发的中断雪崩；此时关闭 PA12/PA13 中断
     *   `BSP_MOTOR_ENC_IRQ_QUENCH_DURATION_MS`（默认 50 ms），让 CPU 喘气。
     *   到期后由 update() 重新打开（边沿丢失但不会饿死主循环）。 */
    uint32_t window;
    MOTOR_LOCK();
    window = s_motor.enc_irq_window;
    s_motor.enc_irq_window = 0u;
    MOTOR_UNLOCK();

    if (s_motor.enc_irq_quench_remain_ms != 0u) {
        s_motor.enc_irq_quench_remain_ms--;
        if (s_motor.enc_irq_quench_remain_ms == 0u) {
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
        DL_GPIO_disableInterrupt(GPIOA,
            BSP_ENC_R_A_PIN
#if (BSP_MOTOR_RIGHT_DECODE_X == 4)
            | BSP_ENC_R_B_PIN
#endif
        );
        s_motor.enc_irq_quench_remain_ms = BSP_MOTOR_ENC_IRQ_QUENCH_DURATION_MS;
    }

    /* --- 1) 左轮 QEI 16-bit → 32-bit 软扩 --------------------------------- */
    uint16_t raw_now = (uint16_t)DL_TimerG_getTimerCount(QEI_LEFT_INST);
    int16_t  delta   = (int16_t)((uint16_t)(raw_now - s_motor.left_raw_prev));
    s_motor.left_raw_prev = raw_now;
    /* left_count 与 right_count 是 ISR 共享变量，但 update 只在主循环里跑、
     * 而 PA12/PA13 ISR 只摸 right_count；左路写入这里其实无 race，无需关中断。
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
    /* 一并清 kick 状态：reset_encoders 语义上等价于"准备下次跑车"，下一次命令
     * 应当被视为 0→非零的全新启动，自动触发 kick。motor 当前若仍在转，调用方
     * 通常先 stop / brake 再 reset_encoders，所以此处覆盖到的边缘场景极少。 */
    s_motor.left_prev_dir         = 0;
    s_motor.right_prev_dir        = 0;
    s_motor.left_kick_remain_ms   = 0u;
    s_motor.right_kick_remain_ms  = 0u;
    MOTOR_UNLOCK();
}

/* ========================================================================== */
/* GROUP1 中断入口（GPIOA + GPIOB + TRNG + COMP0 共享）                          */
/*                                                                              */
/*   ⚠️ MSPM0G3507 NVIC 架构（极易踩坑）：                                       */
/*      所有 GPIO 中断（GPIOA / GPIOB）+ TRNG + COMP0 共享 IRQn = 1 = "GROUP1",*/
/*      vector table 入口名是 `GROUP1_IRQHandler`，**不是**                     */
/*      `GPIOA_IRQHandler` / `GPIOB_IRQHandler`。                                */
/*      `NVIC_EnableIRQ(GPIOA_INT_IRQn)` 实际是 `NVIC_EnableIRQ(1)` = 启用      */
/*      GROUP1 整个组。                                                         */
/*                                                                              */
/*   历史踩坑（Stage 2.4 关键修复 / 2026-05-09）：                               */
/*      本文件早期把 ISR 命名为 `void GPIOA_IRQHandler(void)`，编译链接都不报   */
/*      错（这只是个普通的全局函数符号），但 vector 槽位 17 仍然指向 startup.s */
/*      里 weak 默认 `GROUP1_IRQHandler`（`B .` 死循环）。                      */
/*      症状：转动右轮时 PA12/PA13 沿事件触发 → NVIC 跳 GROUP1 → 死循环 →      */
/*            MCU 整体卡死，串口 / SysTick / 业务全停（绿灯也不闪），即使把    */
/*            SysTick 优先级提到最高、ENC 上拉 + 雪崩兜底全部启用都救不了。     */
/*      正确名字 = `GROUP1_IRQHandler`，参考 SDK 例程                          */
/*      `examples/nortos/LP_MSPM0G3507/driverlib/gpio_simultaneous_interrupts`. */
/*                                                                              */
/*   本工程 GROUP1 上启用的中断源（仅 GPIOA，GPIOB 暂未启用任何沿中断）：       */
/*     PA12 = 右编码器 A 双沿（X2/X4 都开）                                     */
/*     PA13 = 右编码器 B 双沿（仅 X4 开）                                       */
/*   将来若新增 GPIOB 沿中断 / TRNG / COMP0 中断，需要在本函数内追加对应模块的 */
/*   `getPendingInterrupt` 分支（按 SDK gpio_simultaneous_interrupts 例程       */
/*   "GPIOA 段 + GPIOB 段并列" 写法）。                                         */
/*                                                                              */
/*   分发策略：按 TI `gpio_simultaneous_interrupts` 示例读取 MIS 位图，分别   */
/*   处理 PA12 / PA13 后逐 pin clear，避免 IIDX 最高优先级选择吞掉低优先事件。 */
/* ========================================================================== */

void GROUP1_IRQHandler(void)
{
    uint32_t gpioa = DL_GPIO_getEnabledInterruptStatus(GPIOA,
        BSP_ENC_R_A_PIN
#if (BSP_MOTOR_RIGHT_DECODE_X == 4)
        | BSP_ENC_R_B_PIN
#endif
    );

    if ((gpioa & BSP_ENC_R_A_PIN) != 0u) {
        on_right_encoder_edge(true);   /* PA12 (A 相) */
        DL_GPIO_clearInterruptStatus(GPIOA, BSP_ENC_R_A_PIN);
    }
#if (BSP_MOTOR_RIGHT_DECODE_X == 4)
    if ((gpioa & BSP_ENC_R_B_PIN) != 0u) {
        on_right_encoder_edge(false);  /* PA13 (B 相) */
        DL_GPIO_clearInterruptStatus(GPIOA, BSP_ENC_R_B_PIN);
    }
#endif
}
