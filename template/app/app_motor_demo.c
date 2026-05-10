/**
 * @file    app_motor_demo.c
 * @brief   阶段 2 电机驱动演示实现 —— 交互式电机测试 + 双轮同步控制
 *
 * ============================================================
 * 这个文件是做什么的？（写给初学者）
 * ============================================================
 * app_motor_demo.h 定义了"电机测试有哪些功能"。
 * 这个 app_motor_demo.c 是"具体怎么实现这些功能"。
 *
 * 这个文件实现了三个核心功能：
 *   1. **开环速度控制**：给定 rpm → 换算 PWM → 驱动电机
 *   2. **双轮同步 PI 控制**：编码器反馈左右轮速度差 → 补偿 PWM
 *   3. **串口交互式命令行**：在电脑上输入 '+'/'-'/'b'/'r' 等命令控制电机
 *
 * 这个文件也是"从轮询调度到事件响应"的过渡——
 * 它没有用 tick_count 做严格的周期调度，而是用"基于时间戳的松散调度"：
 *   handle_sync_tick(now_ms, last_ms)：检查是否到了同步周期
 *   handle_log_tick(now_ms, last_ms)：检查是否到了日志周期
 *   handle_led_tick(now_ms, last_ms)：检查是否到了 LED 周期
 *
 * 这种方式更灵活——每个任务的频率可以独立调整。
 */

/* ============================================================
 * 头文件包含
 * ============================================================ */
#include "app_motor_demo.h"

#include "bsp_gpio.h"        /* BSP_LED_G/B_PORT/PIN */
#include "bsp_log_uart.h"    /* 串口交互命令输入（read_byte） */
#include "bsp_motor.h"       /* 电机驱动（set_output/get_feedback） */
#include "bsp_systick.h"     /* consume_tick + get_ms */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>           /* printf（串口交互输出） */

/* ================================================================
 * 配置宏
 * ================================================================ */

/** 电机最大空载转速（rpm）。GB370 在 12V 下约 620 rpm。
 *  所有 rpm → PWM 的换算都以此为基准。 */
#define APP_MOTOR_DEMO_MAX_RPM                     (620u)

/** 默认目标转速（rpm）。启动时默认设为最大转速，
 *  方便快速验证电机和编码器是否正常。 */
#define APP_MOTOR_DEMO_DEFAULT_RPM                 (620u)

/** S1 刹车脉冲时间（毫秒） */
#define APP_MOTOR_DEMO_BRAKE_MS                    (120u)

/** 每次按 '+/-' 调整的 rpm 步长 */
#define APP_MOTOR_DEMO_RPM_STEP                    (20u)

/** 同步环执行周期（毫秒）。50 ms = 20 Hz，
 *  速度窗口也是 20 ms，所以同步环和速度反馈刷新同步。 */
#define APP_MOTOR_SYNC_PERIOD_MS                   (50u)

/** 同步环比例增益：每 rpm 误差 = 多少 permille 修正量。
 *  例如左轮 600 rpm、右轮 610 rpm，误差 = 10 rpm
 *  修正量 = 10 × 8 = 80 permille（左轮 +80，右轮 -80） */
#define APP_MOTOR_SYNC_KP_PM_PER_RPM               (8)

/** 同步环积分增益：每 rpm 误差每次累积多少 permille。
 *  如果左右轮持续偏差，积分项会慢慢增大修正量直到消除误差。 */
#define APP_MOTOR_SYNC_KI_PM_PER_RPM_STEP          (1)

/** 同步环修正量上限（permille）。防止 PI 输出过大导致一侧电机反转。 */
#define APP_MOTOR_SYNC_MAX_CORRECTION_PM           (350)

/** 编码器日志打印周期（毫秒）。100 ms = 10 Hz。 */
#define APP_MOTOR_LOG_PERIOD_MS                    (100u)

/** LED 心跳灯周期（毫秒）。250 ms = 4 Hz。
 *  比 app_telemetry 的 5 Hz 慢了一点，给不同的视觉区分。 */
#define APP_MOTOR_HEARTBEAT_PERIOD_MS              (250u)

/* ================================================================
 * 模块状态
 * ================================================================ */

/* ---- 目标转速 ---- */
/** s_target_rpm：当前目标转速（用户可通过串口或按键调整） */
static uint16_t s_target_rpm = APP_MOTOR_DEMO_DEFAULT_RPM;

/** s_target_pwm_pm：目标转速对应的 PWM permille 值。
 *  在 set_speed_rpm() 中自动计算并缓存，避免每拍重复换算。 */
static int16_t  s_target_pwm_pm =
    (int16_t)((APP_MOTOR_DEMO_DEFAULT_RPM * BSP_MOTOR_PWM_MAX_PERMILLE +
        (APP_MOTOR_DEMO_MAX_RPM / 2u)) / APP_MOTOR_DEMO_MAX_RPM);

/* ---- 同步环状态 ---- */
/** s_sync：双轮同步控制器的诊断快照（包含增益、修正量、误差等） */
static app_motor_demo_sync_diag_t s_sync = {
    .enabled = true,                               /* 默认开启同步 */
    .kp_pm_per_rpm = APP_MOTOR_SYNC_KP_PM_PER_RPM,
    .ki_pm_per_rpm_step = APP_MOTOR_SYNC_KI_PM_PER_RPM_STEP,
    .correction_pm = 0,
    .left_cmd_pm = 0,
    .right_cmd_pm = 0,
    .rpm_error = 0,
};

/** s_sync_i_pm：同步环积分累积值（独立于 diag 结构体）。
 *  需要独立存储的原因：diag 是输出用的快照（可能在 ISR 外读取），
 *  而 i_pm 是运行时连续累加的变量。 */
static int16_t s_sync_i_pm = 0;

/* ================================================================
 * rpm → PWM permille 换算
 * ================================================================
 * 这是"开环控制"的核心——给定目标 rpm，计算对应的 PWM 占空比。
 *
 * 公式：PWM = rpm × 1000 / 620（线性映射）
 *
 * 为什么是线性映射？
 *   在轻载情况下，直流电机的转速和 PWM 占空比近似成线性关系。
 *   虽然严格来说不是完全线性的（有死区、非线性区），
 *   但对于调试电机和编码器来说，线性近似已经足够。
 *   平衡控制会通过 PID 自动补偿非线性。
 *
 * 加上 MAX_RPM/2 是为了四舍五入：
 *   PWM = (rpm × 1000 + 310) / 620
 *
 * @param rpm  目标转速（0~620）
 * @return PWM permille 值
 */
static int16_t rpm_to_pwm_pm(uint16_t rpm)
{
    /* 输入保护：rpm 不能超过最大值 */
    if (rpm > APP_MOTOR_DEMO_MAX_RPM) {
        rpm = APP_MOTOR_DEMO_MAX_RPM;
    }
    /* 先做 uint32_t 乘法避免溢出，加半除数实现四舍五入 */
    return (int16_t)(((uint32_t)rpm * BSP_MOTOR_PWM_MAX_PERMILLE +
        (APP_MOTOR_DEMO_MAX_RPM / 2u)) / APP_MOTOR_DEMO_MAX_RPM);
}

/* ================================================================
 * 公开 API —— 速度设置与查询
 * ================================================================ */

/* ----------------------------------------------------------------
 * app_motor_demo_set_speed_rpm() —— 设置目标转速
 * ----------------------------------------------------------------
 * 钳位输入 → 更新 s_target_rpm → 重新计算 s_target_pwm_pm
 */
void app_motor_demo_set_speed_rpm(uint16_t rpm)
{
    if (rpm > APP_MOTOR_DEMO_MAX_RPM) {
        rpm = APP_MOTOR_DEMO_MAX_RPM;
    }
    s_target_rpm = rpm;
    s_target_pwm_pm = rpm_to_pwm_pm(rpm);
}

uint16_t app_motor_demo_get_speed_rpm(void)
{
    return s_target_rpm;
}

/* ================================================================
 * 公开 API —— 同步环配置
 * ================================================================ */

void app_motor_demo_set_sync_enabled(bool enabled)
{
    s_sync.enabled = enabled;
    if (!enabled) {
        app_motor_demo_reset_sync();  /* 关闭时清空积分，避免残留 */
    }
}

void app_motor_demo_set_sync_gains(int16_t kp_pm_per_rpm,
                                   int16_t ki_pm_per_rpm_step)
{
    s_sync.kp_pm_per_rpm = kp_pm_per_rpm;
    s_sync.ki_pm_per_rpm_step = ki_pm_per_rpm_step;
}

void app_motor_demo_reset_sync(void)
{
    s_sync_i_pm = 0;
    s_sync.correction_pm = 0;
    s_sync.left_cmd_pm = s_target_pwm_pm;
    s_sync.right_cmd_pm = s_target_pwm_pm;
    s_sync.rpm_error = 0;
}

void app_motor_demo_get_sync_diag(app_motor_demo_sync_diag_t *out)
{
    if (out == NULL) { return; }
    *out = s_sync;
}

/* ================================================================
 * 内部辅助 —— 限幅函数
 * ================================================================ */

/** clamp_pm：把 PWM 值限幅到 ±1000 */
static int16_t clamp_pm(int32_t v)
{
    if (v > BSP_MOTOR_PWM_MAX_PERMILLE)   return BSP_MOTOR_PWM_MAX_PERMILLE;
    if (v < -BSP_MOTOR_PWM_MAX_PERMILLE)  return -BSP_MOTOR_PWM_MAX_PERMILLE;
    return (int16_t)v;
}

/** clamp_sync_correction：把同步修正量限制在 ±MAX_CORRECTION_PM 内。
 *  这是为了防止 PI 控制器输出过大的修正量，
 *  导致一侧电机全速另一侧反转（失去同步控制的意义）。 */
static int16_t clamp_sync_correction(int32_t v)
{
    if (v > APP_MOTOR_SYNC_MAX_CORRECTION_PM)   return APP_MOTOR_SYNC_MAX_CORRECTION_PM;
    if (v < -APP_MOTOR_SYNC_MAX_CORRECTION_PM)  return -APP_MOTOR_SYNC_MAX_CORRECTION_PM;
    return (int16_t)v;
}

/* ================================================================
 * apply_motor_output —— 把修正量应用到左右电机
 * ================================================================
 * 同步控制的"执行"部分：
 *   左轮 = 目标 PWM + 修正量（如果左轮慢，修正量 > 0 → 左轮加速）
 *   右轮 = 目标 PWM - 修正量（如果右轮快，修正量 > 0 → 右轮减速）
 *
 * 注意：左右和右轮是**相反**的修正方向。
 * 因为修正量定义为 (右rpm - 左rpm)，
 * 正修正量 = 右轮快 = 需要左轮加速、右轮减速。
 *
 * @param correction_pm  修正量（permille）
 */
static void apply_motor_output(int16_t correction_pm)
{
    int16_t left_pm  = clamp_pm((int32_t)s_target_pwm_pm + correction_pm);
    int16_t right_pm = clamp_pm((int32_t)s_target_pwm_pm - correction_pm);

    s_sync.left_cmd_pm  = left_pm;
    s_sync.right_cmd_pm = right_pm;
    bsp_motor_set_output(left_pm, right_pm);
}

/* ================================================================
 * rpm_to_i16 —— float rpm → int16 rpm（四舍五入）
 * ================================================================
 * bsp_motor_feedback_t 中的 speed_rpm 是 float，
 * 但同步 PI 控制器用的是整数运算。
 * 这个函数把 float rpm 转为 int16_t，做四舍五入。
 *
 * 用法：rpm_to_i16(feedback->left_speed_rpm)
 *       正数：加 0.5 再截断 → 四舍五入
 *       负数：减 0.5 再截断 → 四舍五入 */
static int16_t rpm_to_i16(float rpm)
{
    if (rpm >= 0.0f) return (int16_t)(rpm + 0.5f);
    else             return (int16_t)(rpm - 0.5f);
}

/* ================================================================
 * motor_sync_step —— 同步环 PI 控制的一拍计算
 * ================================================================
 * 这是双轮同步控制的核心函数！
 * 每个同步周期（默认 50 ms）执行一次。
 *
 * 计算流程：
 *   1. 读取左右轮实际转速（rpm）
 *   2. 误差 = 右轮 rpm - 左轮 rpm
 *   3. 积分累加：i_pm += 误差 × Ki
 *   4. 积分限幅（MAX_CORRECTION_PM）
 *   5. 修正量 = 误差 × Kp + 积分值
 *   6. 修正量限幅
 *   7. 应用到电机输出
 *
 * @param feedback  电机编码器反馈数据
 * @param running   是否正在运行（刹车时不执行同步逻辑）
 */
static void motor_sync_step(const bsp_motor_feedback_t *feedback, bool running)
{
    if ((feedback == NULL) || !running) {
        return;
    }

    /* 如果同步控制关闭了，直接输出目标 PWM（不做补偿） */
    if (!s_sync.enabled) {
        s_sync.rpm_error = 0;
        s_sync.correction_pm = 0;
        apply_motor_output(0);
        return;
    }

    /* 读取左右轮的实际转速（float → int16 四舍五入） */
    int16_t left_rpm = rpm_to_i16(feedback->left_speed_rpm);
    int16_t right_rpm = rpm_to_i16(feedback->right_speed_rpm);

    /* 误差 = 右轮 - 左轮。
     * 正误差 = 右轮更快 → 需要减小右轮/增大左轮 */
    int16_t error = (int16_t)(right_rpm - left_rpm);

    /* ---- 积分更新 ----
     * 累加本拍的积分增量：i += error × Ki
     * 升到 int32 计算避免溢出，结果限幅在 MAX_CORRECTION_PM 内 */
    int32_t i_term = (int32_t)s_sync_i_pm +
        ((int32_t)error * (int32_t)s_sync.ki_pm_per_rpm_step);
    s_sync_i_pm = clamp_sync_correction(i_term);
    s_sync.rpm_error = error;

    /* ---- 计算总修正量 ----
     * 修正量 = P 项（error × Kp）+ I 项（累积积分）
     * 限幅后应用到左右电机 */
    int32_t correction = ((int32_t)error * (int32_t)s_sync.kp_pm_per_rpm) +
        (int32_t)s_sync_i_pm;
    s_sync.correction_pm = clamp_sync_correction(correction);
    apply_motor_output(s_sync.correction_pm);
}

/* ================================================================
 * 内部辅助 —— 运行/刹车控制
 * ================================================================ */

/** apply_run_output_if_needed —— 如果正在运行，重新设置电机输出。
 *  同步环开启时用修正量，关闭时输出相同的目标 PWM。 */
static void apply_run_output_if_needed(bool running)
{
    if (running) {
        bsp_motor_enable(true);
        apply_motor_output(s_sync.enabled ? s_sync.correction_pm : 0);
    }
}

/** brake_now —— 立即执行脉冲刹车 */
static void brake_now(void)
{
    bsp_motor_brake_pulse_ms(APP_MOTOR_DEMO_BRAKE_MS);
}

/* ================================================================
 * 串口交互命令
 * ================================================================
 * 以下是"命令行界面"的实现——通过 UART0（XDS-UART）接收电脑发来的
 * 单个字符命令，根据命令执行不同的操作。
 *
 * 支持的命令一览：
 *   '+'：目标转速 + 20 rpm
 *   '-'：目标转速 - 20 rpm
 *   数字 + 回车：直接设为目标转速（如 "300\n" → 300 rpm）
 *   'b'：刹车（brake）
 *   'r'：启动（run）
 *   's'：开关同步控制（sync on/off）
 *   'p'：打印同步环诊断信息
 *   'h'/'?'：打印帮助信息
 *
 * 数字输入原理（"多字节命令组合"）：
 *   用户输入 "3" "0" "0" "\n" → rpm_acc 逐字符累积：
 *     '3' → rpm_acc = 0×10 + 3 = 3
 *     '0' → rpm_acc = 3×10 + 0 = 30
 *     '0' → rpm_acc = 30×10 + 0 = 300
 *     '\n' → 提交：设目标转速 = 300 rpm
 */

/** print_ctrl_help —— 打印命令帮助 */
static void print_ctrl_help(void)
{
    (void)printf("[ctrl] UART commands: '+'/'-' step %urpm, '<rpm><Enter>' set speed, "
                 "'b' brake, 'r' run, 's' sync on/off, 'p' print sync\r\n",
        (unsigned int)APP_MOTOR_DEMO_RPM_STEP);
}

/**
 * process_log_uart_commands —— 处理来自 UART 的交互命令
 *
 * 每拍被调一次，使用 while 循环清空 UART RX 缓冲区中的所有待处理字节。
 * 这样做的好处：如果用户快速输入多个命令，不会因为每拍只处理一个字节而丢失。
 *
 * @param running  指向运行状态的指针（函数内部会修改它） */
static void process_log_uart_commands(bool *running)
{
    /* rpm_acc：多字节数字累积（用户输入 "350\n" → 依次存入 3→30→350） */
    static uint16_t rpm_acc = 0u;
    /* rpm_pending：是否有数字正在输入（未提交） */
    static bool rpm_pending = false;
    uint8_t ch;

    /* while 循环：一次性处理 UART RX 缓冲区中的所有字节 */
    while (bsp_log_uart_read_byte(&ch)) {

        /* ---- 数字输入 ----- */
        if (ch >= (uint8_t)'0' && ch <= (uint8_t)'9') {
            /* 累积数字：rpm_acc = rpm_acc × 10 + 输入数字
             * 例如：输入'3'→'5'→'0'：0→3→35→350
             * 如果累积值超过 MAX_RPM，钳位到 MAX_RPM */
            uint32_t next = (uint32_t)rpm_acc * 10u + (uint32_t)(ch - (uint8_t)'0');
            rpm_acc = (next > APP_MOTOR_DEMO_MAX_RPM) ?
                APP_MOTOR_DEMO_MAX_RPM : (uint16_t)next;
            rpm_pending = true;
            continue;  /* 数字不执行任何命令，继续等更多输入 */
        }

        /* ---- 提交数字（回车/换行/空格） ---- */
        if (ch == (uint8_t)'\r' || ch == (uint8_t)'\n' ||
            ch == (uint8_t)' ') {
            if (rpm_pending) {
                app_motor_demo_set_speed_rpm(rpm_acc);
                app_motor_demo_reset_sync();
                apply_run_output_if_needed(*running);
                (void)printf("[ctrl] set target=%urpm pwm=%d/1000\r\n",
                    (unsigned int)s_target_rpm, (int)s_target_pwm_pm);
                rpm_acc = 0u;
                rpm_pending = false;
            }
            continue;
        }

        /* ---- 单字符命令 ---- */
        /* 任何非数字字符都会清空当前的数字累积（rpm_acc 清零）。
         * 这是因为用户按了其他键，说明之前的数字输入被取消了。 */
        rpm_acc = 0u;
        rpm_pending = false;

        switch (ch) {
        case (uint8_t)'+':
            /* 加速：在当前目标转速基础上 + 20 rpm */
            app_motor_demo_set_speed_rpm(
                (uint16_t)(s_target_rpm + APP_MOTOR_DEMO_RPM_STEP));
            app_motor_demo_reset_sync();
            apply_run_output_if_needed(*running);
            (void)printf("[ctrl] target=%urpm pwm=%d/1000\r\n",
                (unsigned int)s_target_rpm, (int)s_target_pwm_pm);
            break;

        case (uint8_t)'-':
            /* 减速：在当前目标转速基础上 - 20 rpm（不小于 0） */
            app_motor_demo_set_speed_rpm((s_target_rpm > APP_MOTOR_DEMO_RPM_STEP) ?
                (uint16_t)(s_target_rpm - APP_MOTOR_DEMO_RPM_STEP) : 0u);
            app_motor_demo_reset_sync();
            apply_run_output_if_needed(*running);
            (void)printf("[ctrl] target=%urpm pwm=%d/1000\r\n",
                (unsigned int)s_target_rpm, (int)s_target_pwm_pm);
            break;

        case (uint8_t)'b':
        case (uint8_t)'B':
            /* 刹车：停止电机，进入刹车态 */
            *running = false;
            brake_now();
            (void)printf("[ctrl] brake\r\n");
            break;

        case (uint8_t)'r':
        case (uint8_t)'R':
            /* 启动：重新运行电机 */
            *running = true;
            app_motor_demo_reset_sync();
            apply_run_output_if_needed(*running);
            (void)printf("[ctrl] run target=%urpm pwm=%d/1000\r\n",
                (unsigned int)s_target_rpm, (int)s_target_pwm_pm);
            break;

        case (uint8_t)'s':
        case (uint8_t)'S':
            /* 切换同步控制开关 */
            app_motor_demo_set_sync_enabled(!s_sync.enabled);
            apply_run_output_if_needed(*running);
            (void)printf("[ctrl] sync=%s kp=%d ki=%d maxCorr=%d\r\n",
                s_sync.enabled ? "on" : "off",
                (int)s_sync.kp_pm_per_rpm,
                (int)s_sync.ki_pm_per_rpm_step,
                APP_MOTOR_SYNC_MAX_CORRECTION_PM);
            break;

        case (uint8_t)'p':
        case (uint8_t)'P':
            /* 打印当前同步环诊断信息 */
            (void)printf("[ctrl] sync=%s err=%d corr=%d cmdL=%d cmdR=%d kp=%d ki=%d\r\n",
                s_sync.enabled ? "on" : "off",
                (int)s_sync.rpm_error,
                (int)s_sync.correction_pm,
                (int)s_sync.left_cmd_pm,
                (int)s_sync.right_cmd_pm,
                (int)s_sync.kp_pm_per_rpm,
                (int)s_sync.ki_pm_per_rpm_step);
            break;

        case (uint8_t)'h':
        case (uint8_t)'H':
        case (uint8_t)'?':
            /* 打印帮助信息 */
            print_ctrl_help();
            break;

        default:
            /* 未知命令：忽略 */
            break;
        }
    }
}

/* ================================================================
 * S2 装车按键（当前未启用，预留给将来开发）
 * ================================================================
 * LaunchPad 的 S2/SW2 默认接到 PA16，但 PA16 已被 TB6612 的 AIN2 占用。
 * 等将来把 S2 飞线到空闲 GPIO 后，定义 BSP_LOAD_BTN_* 宏并打开下面的条件编译即可启用。
 * 届时按下 S2 → demo 返回 true → main 切入 app_balance_run()。 */

#ifndef APP_MOTOR_DEMO_ENABLE_LOAD_BUTTON
#define APP_MOTOR_DEMO_ENABLE_LOAD_BUTTON          (0)
#endif

#if APP_MOTOR_DEMO_ENABLE_LOAD_BUTTON
/* S2 按键检测（带 80 ms 去抖）。
 * 使用轮询方式（不用中断），每拍在 handle_load_button 中被调用。
 * 按下 S2 → 返回 true */
static bool consume_load_button_request(uint32_t now_ms)
{
    static uint32_t last_press_ms = 0u;
    static bool was_pressed = false;

    /* 读取 S2 引脚电平（低有效 → 按下 = 低电平） */
    bool pressed = ((DL_GPIO_readPins(BSP_LOAD_BTN_PORT, BSP_LOAD_BTN_PIN) &
        BSP_LOAD_BTN_PIN) == 0u);
    bool rising_event = false;

    /* 检测上升沿（从松开到按下）并做去抖 */
    if (pressed && !was_pressed &&
        ((now_ms - last_press_ms) >= BSP_MOTOR_BTN_DEBOUNCE_MS)) {
        last_press_ms = now_ms;
        rising_event = true;
    }
    was_pressed = pressed;
    return rising_event;
}
#else
/* S2 未启用时的存根函数 */
static bool consume_load_button_request(uint32_t now_ms)
{
    (void)now_ms;
    return false;
}
#endif

/* ================================================================
 * 日志与 UI 辅助
 * ================================================================ */

/** print_boot_banner —— 启动时打印配置信息 */
static void print_boot_banner(void)
{
    (void)printf("[boot] stage2 motor demo start\r\n");
    (void)printf("[boot] target=%urpm pwm=%d/1000 max=%urpm\r\n",
        (unsigned int)s_target_rpm, (int)s_target_pwm_pm,
        (unsigned int)APP_MOTOR_DEMO_MAX_RPM);
    (void)printf("[boot] motor sync enabled kp=%d ki=%d maxCorr=%d period=%ums\r\n",
        (int)s_sync.kp_pm_per_rpm,
        (int)s_sync.ki_pm_per_rpm_step,
        APP_MOTOR_SYNC_MAX_CORRECTION_PM,
        (unsigned int)APP_MOTOR_SYNC_PERIOD_MS);
    (void)printf("[boot] press S1(PA18) to brake/start both motors\r\n");
    (void)printf("[boot] S2 load-mode request is disabled until its GPIO is rerouted from PA16/AIN2\r\n");
    print_ctrl_help();
}

/* ================================================================
 * S1 按键处理 —— 启动/刹车切换
 * ================================================================
 * bsp_motor_consume_toggle_request() 在检测到 S1 有效按下后返回 true。
 * 每次按下切换 running 状态：运行 ↔ 刹车。 */

static void handle_start_button(bool *running)
{
    if (!bsp_motor_consume_toggle_request()) {
        return;
    }

    /* 切换运行状态 */
    *running = !(*running);

    (void)printf("[btn] S1 pressed: %s (irq=%lu poll=%lu raw=%u active=%u)\r\n",
        *running ? "start" : "brake",
        (unsigned long)bsp_motor_get_button_irq_count(),
        (unsigned long)bsp_motor_get_button_poll_count(),
        bsp_motor_get_start_button_raw_level() ? 1u : 0u,
        bsp_motor_is_start_button_active() ? 1u : 0u);

    if (*running) {
        bsp_motor_enable(true);
        app_motor_demo_reset_sync();
        apply_run_output_if_needed(true);
    } else {
        brake_now();
    }

    (void)printf("[motor] state=%s target=%urpm pwm=%d/1000\r\n",
        *running ? "run" : "brake",
        (unsigned int)s_target_rpm, (int)s_target_pwm_pm);

    /* 按一次 S1 翻转一次蓝灯——提供视觉反馈 */
    DL_GPIO_togglePins(BSP_LED_B_PORT, BSP_LED_B_PIN);
}

/* ================================================================
 * S2 装车按键处理
 * ================================================================ */

static bool handle_load_button(uint32_t now_ms)
{
    if (!consume_load_button_request(now_ms)) {
        return false;
    }

    /* S2 按下 → 退出 demo 进入装车平衡模式 */
    (void)printf("[btn] S2 pressed: enter load balance mode\r\n");
    bsp_motor_stop();
    bsp_motor_enable(false);
    return true;
}

/* ================================================================
 * 定时任务（基于时间戳的松散调度）
 * ================================================================
 * 与 app_balance_run() 的 tick_count 取模方式不同，
 * 这里使用"绝对时间戳差"来做周期判断。
 *
 * 好处：每个任务的频率可以独立调整（改宏即可），不需要整体改 tick_count 算术。
 * 坏处：需要记录多个 last_xxx_ms 变量，代码稍微多几行。
 */

/** handle_sync_tick —— 同步环周期任务（默认 50 ms = 20 Hz） */
static void handle_sync_tick(uint32_t now_ms,
                             uint32_t *last_sync_ms,
                             bool running,
                             bsp_motor_feedback_t *feedback)
{
    /* 检查是否距离上次执行已过去 APP_MOTOR_SYNC_PERIOD_MS */
    if ((now_ms - *last_sync_ms) < APP_MOTOR_SYNC_PERIOD_MS) {
        return;
    }

    *last_sync_ms = now_ms;
    bsp_motor_get_feedback(feedback);
    motor_sync_step(feedback, running);
}

/** handle_log_tick —— 编码器日志周期任务（默认 100 ms = 10 Hz） */
static void handle_log_tick(uint32_t now_ms,
                            uint32_t *last_log_ms,
                            bool running,
                            bsp_motor_feedback_t *feedback)
{
    if ((now_ms - *last_log_ms) < APP_MOTOR_LOG_PERIOD_MS) {
        return;
    }

    *last_log_ms = now_ms;
    bsp_motor_get_feedback(feedback);
    print_feedback_log(now_ms, running, feedback);
}

/** handle_led_tick —— LED 心跳灯周期任务（默认 250 ms = 4 Hz） */
static void handle_led_tick(uint32_t now_ms, uint32_t *last_led_ms)
{
    if ((now_ms - *last_led_ms) < APP_MOTOR_HEARTBEAT_PERIOD_MS) {
        return;
    }

    *last_led_ms = now_ms;
    DL_GPIO_togglePins(BSP_LED_G_PORT, BSP_LED_G_PIN);
}

/** print_feedback_log —— 打印编码器反馈日志
 *
 * 格式示例：
 *   [enc] t=1250ms L=1680( 45.75 deg) R=1660( 44.50 deg)
 *   rpmL=600 rpmR=605 target=620rpm state=run sync=1
 *   err=5 corr=40 cmdL=640 cmdR=600
 *
 * 角度使用定点格式化（×100 避免 printf("%f")） */
static void print_feedback_log(uint32_t now_ms,
                               bool running,
                               const bsp_motor_feedback_t *feedback)
{
    /* 角度 × 100 变成厘度（centi-degrees），方便整数打印 */
    int32_t left_cdeg = (int32_t)(feedback->left_angle_deg * 100.0f);
    int32_t right_cdeg = (int32_t)(feedback->right_angle_deg * 100.0f);

    (void)printf(
        "[enc] t=%lums L=%ld(%ld.%02ld deg) R=%ld(%ld.%02ld deg) "
        "rpmL=%ld rpmR=%ld target=%urpm state=%s sync=%u "
        "err=%d corr=%d cmdL=%d cmdR=%d "
        "btn_irq=%lu btn_poll=%lu raw=%u active=%u\r\n",
        (unsigned long)now_ms,
        (long)feedback->left_count,
        (long)(left_cdeg / 100),                    /* 左轮角度整数部分 */
        (long)(left_cdeg < 0 ? -left_cdeg : left_cdeg) % 100,  /* 小数部分 */
        (long)feedback->right_count,
        (long)(right_cdeg / 100),
        (long)(right_cdeg < 0 ? -right_cdeg : right_cdeg) % 100,
        (long)feedback->left_speed_rpm,
        (long)feedback->right_speed_rpm,
        (unsigned int)s_target_rpm,
        running ? "run" : "brake",
        s_sync.enabled ? 1u : 0u,
        (int)s_sync.rpm_error,
        (int)s_sync.correction_pm,
        (int)s_sync.left_cmd_pm,
        (int)s_sync.right_cmd_pm,
        (unsigned long)bsp_motor_get_button_irq_count(),
        (unsigned long)bsp_motor_get_button_poll_count(),
        bsp_motor_get_start_button_raw_level() ? 1u : 0u,
        bsp_motor_is_start_button_active() ? 1u : 0u);
}

/* ================================================================
 * app_motor_demo_run() —— 主循环入口
 * ================================================================
 * 这是电机 demo 的"主循环"，进入后持续运行直到：
 *   - 收到 S2 装车命令 → 返回 true（main 切到平衡控制）
 *   - 调试器停止 / 断电
 *
 * 调度方式：基于时间戳的松散周期调度
 *   1 kHz：bsp_motor_update()（编码器计数 + 速度窗口）
 *   1 kHz：process_log_uart_commands()（串口命令处理）
 *   1 kHz：handle_start_button()（S1 按键处理）
 *   20 Hz：handle_sync_tick()（同步环 PI 控制）
 *   10 Hz：handle_log_tick()（编码器日志打印）
 *   4 Hz：handle_led_tick()（LED 心跳灯）
 *
 * @return true = 收到装车请求，调用方应切入 app_balance_run()
 *         false = 永不返回（还在 demo 中）
 */
bool app_motor_demo_run(void)
{
    bool running = true;     /* 启动时默认运行 */
    uint32_t last_sync_ms = 0u;
    uint32_t last_log_ms = 0u;
    uint32_t last_led_ms = 0u;
    bsp_motor_feedback_t feedback;

    /* 启动：使能电机 + 输出初始命令 + 打印配置信息 */
    bsp_motor_enable(true);
    app_motor_demo_reset_sync();
    apply_motor_output(0);
    print_boot_banner();

    /* ██ 主循环 ██ 持续运行 */
    for (;;) {
        /* ---- 等待 1 ms SysTick 心跳 ---- */
        if (!bsp_systick_consume_tick()) {
            __WFI();
            continue;
        }

        /* ---- 1 kHz：电机 1 ms 节拍（编码器 + brake 计时） ---- */
        bsp_motor_update();

        /* ---- 1 kHz：串口命令处理（键盘输入） ---- */
        process_log_uart_commands(&running);

        /* ---- 1 kHz：S1 按键处理（启动/刹车切换） ---- */
        handle_start_button(&running);

        uint32_t now_ms = bsp_systick_get_ms();

        /* ---- 20 Hz：同步环 PI 控制 ---- */
        handle_sync_tick(now_ms, &last_sync_ms, running, &feedback);

        /* ---- S2 装车按键处理 ---- */
        if (handle_load_button(now_ms)) {
            return true;   /* 退出 demo，进入平衡模式 */
        }

        /* ---- 10 Hz：编码器日志打印 ---- */
        handle_log_tick(now_ms, &last_log_ms, running, &feedback);

        /* ---- 4 Hz：LED 心跳灯 ---- */
        handle_led_tick(now_ms, &last_led_ms);
    }
}
