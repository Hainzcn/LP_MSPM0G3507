/**
 * @file    app_motor_demo.h
 * @brief   阶段 2 电机驱动演示 —— 让电机转起来，验证驱动和编码器是否正常
 *
 * ============================================================
 * 这个文件是做什么的？（写给初学者）
 * ============================================================
 * 在写平衡控制之前，我们需要先确认"电机能不能正常转"和"编码器能不能正常读"。
 * 这个文件就是用来做这件事的——一个交互式的电机测试程序。
 *
 * 它提供了以下功能：
 *   1. 让左右电机以指定转速转动（通过 PWM 开环控制）
 *   2. 双轮同步控制：根据编码器反馈自动补偿左右轮速度差（PI 控制器）
 *   3. 串口交互命令：在电脑上输入 '+'/'-' 调整转速，'b'刹车，'r'启动等
 *   4. S1 按键：一键启动/刹车
 *   5. 100 Hz 编码器日志：实时打印左右轮的角度和转速
 *
 * 这是从"传感器能读到数据"到"电机能转起来"的关键一步——
 * 只有确认电机和编码器都正常，才能开始调平衡控制参数。
 *
 * ============================================================
 * 双轮同步控制原理
 * ============================================================
 * 两个电机即使接同样的 PWM，转速也不完全一样（制造误差、摩擦差异等）。
 * 对于自平衡小车，左右轮速度不一致会导致车体偏离黑线。
 *
 * 这个 demo 中实现了一个简单的"同步 PI 控制器"：
 *   误差 = 右轮 rpm - 左轮 rpm
 *   修正量 = Kp × 误差 + Ki × 累积误差
 *   左轮命令 = 目标 PWM + 修正量（左轮加速）
 *   右轮命令 = 目标 PWM - 修正量（右轮减速）
 *
 * 这样就能让两个轮子尽可能同步旋转。
 */

#ifndef APP_MOTOR_DEMO_H                   /* 头文件保护宏 */
#define APP_MOTOR_DEMO_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 运行电机转动演示主循环（永不返回，除非收到装车模式请求）。
 *
 * 这个函数调用后进入交互式电机测试模式。
 * 在此模式下，你可以通过串口命令实时控制电机，
 * 同时观察编码器反馈来验证硬件是否正常。
 *
 * 控制方式：
 *   - S1 按键：切换 启动/刹车
 *   - 串口命令：'+'/'-' 调整目标转速、'b'刹车、'r'启动等（详见 help）
 *   - 100 Hz 编码器日志：角度和转速输出到串口
 *
 * @return true = 收到"进入装车/平衡模式"的请求，
 *         调用方应退出 demo 循环并切入 app_balance_run()。
 *         false = 永不返回（demo 持续运行）
 */
bool app_motor_demo_run(void);

/**
 * @brief 设置 demo 的目标转速（rpm）。
 *
 * 目标转速会被钳位到 0 ~ 620 rpm 范围内（GB370 电机的最大空载转速）。
 * 设置后会内部换算为对应的 PWM permille 值：
 *   PWM = rpm × 1000 / 620（线性映射）
 *
 * @param rpm  目标转速（0~620）。超出范围自动钳位。
 */
void app_motor_demo_set_speed_rpm(uint16_t rpm);

/** @brief 读取当前 demo 的目标转速配置。 */
uint16_t app_motor_demo_get_speed_rpm(void);

/* ================================================================
 * 同步环诊断结构体
 * ================================================================
 * 这个结构体记录了双轮同步 PI 控制器的运行时状态。
 * 通过 app_motor_demo_get_sync_diag() 获取，
 * 可以用于调试同步控制效果。
 */

typedef struct {
    bool    enabled;              /* 同步控制是否开启 */
    int16_t kp_pm_per_rpm;       /* 同步环比例增益：每 rpm 误差修正多少 permille */
    int16_t ki_pm_per_rpm_step;  /* 同步环积分增益：每 rpm 误差每次累积多少 permille */
    int16_t correction_pm;       /* 当前修正量（permille） */
    int16_t left_cmd_pm;         /* 当前左轮最终命令 */
    int16_t right_cmd_pm;        /* 当前右轮最终命令 */
    int16_t rpm_error;           /* 当前转速误差 = 右 rpm - 左 rpm */
} app_motor_demo_sync_diag_t;

/**
 * @brief 开关双轮同步控制服务。
 *
 * 开启时：左右轮根据编码器反馈自动补偿速度差
 * 关闭时：左右轮输出相同的目标 PWM（不做同步补偿）
 *
 * @param enabled  true=开启同步，false=关闭同步
 */
void app_motor_demo_set_sync_enabled(bool enabled);

/**
 * @brief 配置同步环的 PI 增益。
 *
 * @param kp_pm_per_rpm     比例增益：误差 × Kp = 本拍修正量
 * @param ki_pm_per_rpm_step 积分增益：误差 × Ki = 每次累积的积分增量
 */
void app_motor_demo_set_sync_gains(int16_t kp_pm_per_rpm,
                                   int16_t ki_pm_per_rpm_step);

/** @brief 清空同步环的积分累积和诊断值（切换目标转速或重启前调用）。 */
void app_motor_demo_reset_sync(void);

/**
 * @brief 读取同步环的诊断信息快照。
 * @param out  输出结构体指针。NULL 时函数静默返回。
 */
void app_motor_demo_get_sync_diag(app_motor_demo_sync_diag_t *out);

#ifdef __cplusplus
}
#endif

#endif /* APP_MOTOR_DEMO_H */
