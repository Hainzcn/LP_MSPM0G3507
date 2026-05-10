/**
 * @file    main.c
 * @brief   阶段 2.2 主入口：MSPM0G3507 自平衡瞄准小车 —— 上车基线固件
 *          （姿态遥测 + 电机 + 编码器 + 电池 + 安全状态机 + 平衡 / 速度环骨架）。
 *
 *  调用链：
 *      SYSCFG_DL_init()      -- 由 SysConfig 自动生成，配置时钟 / peripheral pins
 *      bsp_gpio_init         -- 14 路业务 GPIO 手工 init（绕开 SDK multi-pad bug）
 *      bsp_systick_init      -- 1 kHz 节拍 + ms 计时
 *      bsp_log_uart_init     -- UART0 (XDS-UART) printf retarget
 *      bsp_k230_uart_init    -- UART1 + DMA RX 接收骨架
 *      bsp_imu_uart_init     -- UART3 (MS901M) RX 中断 + 256 B 环缓
 *      ms901m_init           -- 解析器状态机复位 + 量程系数（±4 g / ±2000 dps）
 *      wait_for_ms901m_*     -- 上电后 500 ms 内等到第一帧 0x01；超时报警
 *      bsp_motor_init        -- TB6612 + QEI + S1 中断（X4 解码默认开 PA13 双沿）
 *      bsp_battery_init      -- ADC0/PB24 触发首次软件转换
 *      app_safety_init       -- 安全状态机置 DISARMED + 电机 brake + STBY 关
 *      app_balance_init      -- 两路 PID 安全默认（增益 0 + 输出限幅 + D 滤波）
 *      app_balance_run       -- 永不返回的主循环
 *
 *  失败处理：MS901M 上电后 500 ms 仍未收到 0x01 姿态帧 → LED_R 常亮 + 蜂鸣
 *            200 ms，然后死循环；不进入主循环以免上报无效数据。
 *
 *  GPIO 备注：业务 GPIO 全部由 bsp_gpio.[ch] 管理，宏前缀 BSP_*；syscfg 不
 *  生成 GPIO_OUT_*_PORT/PIN 这类宏（详见 EIDE/LP_MSPM0G3507.syscfg 头部注释、
 *  docs/TaskLog/Stage1-IMU-BT-Telemetry.md §8.5）。同 port 的引脚可合并到
 *  一次 DL_GPIO_xxxPins 调用（LED_R/G/B 同属 GPIOB → BSP_LED_*_PORT）。
 *
 *  Stage 1.5 变更（2026-05-07）：原 I²C+MPU6050 链路因开发板 PB2/PB3 缺上拉
 *  电阻被废弃，改走 UART+ATK-MS901M（板载 EKF），姿态直接采纳 0x01 帧。
 *
 *  Stage 1.6 变更（2026-05-08）：IMU 串口从 UART2/PA21/PA22 迁到 UART3/PB12/
 *  PB13（原蓝牙引脚），原因是 PA21 未引到 BoosterPack 需焊接；同期蓝牙
 *  HC-04 模块整体下线，遥测改走 1 Hz XDS-UART printf。详见
 *  docs/TaskLog/Stage1.5-IMU-Swap-MS901M.md §11 Stage 1.6 重排。
 *
 *  Stage 2.2 变更（2026-05-09）：上车基线固件就绪。原 `app_telemetry_run()`
 *  入口被 `app_balance_run()` 取代，后者吸收了 telemetry 的 IMU drain + 1 Hz
 *  心跳日志，同时新增 100 Hz 控制环（safety + balance step）+ LED 状态指示。
 *  ⚠️ PID 增益默认 0，上电不会自己动；装车整定时通过串口注入即可。
 *  详见 docs/TaskLog/Stage2-MotorDrive-Encoder.md §3.5 / §6.3。
 */
 
/*
 * ============================================================
 * 头文件包含区域
 * ============================================================
 * 每个 #include 都引入一个"工具箱"，让 main.c 能调用对应模块的功能。
 * 分层结构：
 *   ti_msp_dl_config.h → 单片机底层硬件配置（由 SysConfig 图形工具生成）
 *   bsp_*.h            → 硬件驱动层（BSP）：控制引脚、串口、定时器等
 *   ms901m.h           → 中间层：姿态传感器数据解析
 *   app_*.h            → 应用层：安全逻辑、平衡控制、电机测试
 */

#include "ti_msp_dl_config.h"   /* 芯片时钟树 + 外设引脚映射（SysConfig 自动生成） */

/* ---- 硬件驱动层 (BSP) ---- */
#include "bsp_battery.h"        /* 电池电压 ADC 采样 */
#include "bsp_gpio.h"           /* 业务 GPIO 手工初始化（LED/蜂鸣/电机方向/编码器） */
#include "bsp_imu_uart.h"       /* UART3：接收 MS901M 姿态传感器数据 */
#include "bsp_k230_uart.h"      /* UART1：DMA 接收 K230 视觉模块指令 */
#include "bsp_log_uart.h"       /* UART0：XDS 调试串口，printf 输出到电脑 */
#include "bsp_motor.h"          /* 电机驱动（TB6612 PWM + 编码器 QEI/中断） */
#include "bsp_systick.h"        /* 1 毫秒系统心跳定时器 */

/* ---- 中间层 (Middleware) ---- */
#include "ms901m.h"             /* MS901M 姿态字节流 → 角度/角速度 解析器 */

/* ---- 应用层 (App) ---- */
#include "app_balance.h"        /* 平衡控制主循环（串级 PID + 调度） */
#include "app_motor_demo.h"     /* 电机独立测试模式（装车调试前使用） */
#include "app_safety.h"         /* 安全状态机（跌倒/低压保护） */

#include <stdint.h>             /* 标准整型定义：uint8_t, int32_t 等 */
#include <stdio.h>              /* 标准输入输出：printf 串口打印 */

/*
 * ============================================================
 * 编译时常量（宏定义）
 * ============================================================
 * 这些 #define 在编译时就会被替换成具体数值，不占用单片机运行内存(RAM)。
 * 用宏而不直接用数字的好处：改一处即可全局生效，且名字表意更清晰。
 */

/* MS901M 上电后允许的最长等待时间（毫秒）。
 * 姿态传感器冷启动通常 100 ms 内出数据，500 ms 是留有充足裕度的安全值。
 * 超过这个时间还没收到数据 → 判定传感器未连接/故障 → 红灯+蜂鸣报警。
 * 出厂默认 200 Hz 主动上报，
 * 上电到首帧典型 < 100 ms；500 ms 给冷启动足够裕度。 */
#define MS901M_BOOT_TIMEOUT_MS   500u

/* 每次从串口缓冲区捞取的最大字节数。
 * MS901M 一帧数据约 11~17 字节，200 Hz 频率 → 约 3.4 kB/s。
 * 64 字节单次捞取能力是其 4 倍，不会出现"一次捞不完"的窘境。
 * 等待期间每拍 drain 字节数上限（≥ 单帧最大 17 B + 几帧裕度即可） */
#define MS901M_DRAIN_CHUNK       64u

/* ---- 装车模式临时测试 PID 参数 ----
 * 这些是给"离地调试/桌面支架测试"用的默认参数。
 * 正式装车上路时，应该通过串口动态注入整定好的参数，而不是改这里。
 * 注意：速度环增益全为 0，意味着当前只做原地平衡，不响应前进/后退指令。
 * 装车模式临时测试 PID：只用于离地 / 支架调试，正式整定前应改为串口注入。 */
#define LOAD_TEST_BALANCE_KP     (30.0f)  /* 平衡环 P 增益：值越大，纠正倾角的力度越大 */
#define LOAD_TEST_BALANCE_KI     (0.0f)   /* 平衡环 I 增益：0 = 不消除静态偏差 */
#define LOAD_TEST_BALANCE_KD     (4.0f)   /* 平衡环 D 增益：值越大，抑制摇晃的"阻尼"越强 */
#define LOAD_TEST_SPEED_KP       (0.0f)   /* 速度环 P 增益：0 = 速度环不工作 */
#define LOAD_TEST_SPEED_KI       (0.0f)   /* 速度环 I 增益 */
#define LOAD_TEST_SPEED_KD       (0.0f)   /* 速度环 D 增益 */

/*
 * ============================================================
 * 辅助函数：IMU 初始化失败处理
 * ============================================================
 * 当 MS901M 姿态传感器在上电后 500 ms 内没反应时调用。
 * 这是"硬故障"，无法恢复，只能报警后死循环。
 *
 * 执行动作：
 *   1. 亮红灯 → 视觉报警
 *   2. 灭绿灯和蓝灯 → 避免混淆
 *   3. 蜂鸣器响 200 ms → 听觉报警
 *   4. 串口打印错误信息 → 方便调试定位
 *   5. 进入死循环（__WFI() = 低功耗等待中断，但不会再做任何事）
 */
static void fatal_imu_init_failure(int32_t rc)
{
    /* 点亮红灯，表示致命错误 */
    DL_GPIO_setPins(BSP_LED_R_PORT, BSP_LED_R_PIN);
    /* 熄灭绿灯和蓝灯（用同一组引脚的 clearPins 批量操作） */
    DL_GPIO_clearPins(BSP_LED_G_PORT, BSP_LED_G_PIN | BSP_LED_B_PIN);
    /* 蜂鸣器鸣叫 */
    DL_GPIO_setPins(BSP_BUZZER_PORT, BSP_BUZZER_PIN);
    bsp_systick_delay_ms(200u);         /* 持续鸣叫 200 毫秒 */
    DL_GPIO_clearPins(BSP_BUZZER_PORT, BSP_BUZZER_PIN); /* 关闭蜂鸣器 */

    /* 打印致命错误信息到调试串口。
     * (void) 前缀表示"我知道 printf 有返回值但故意不用"，消除编译器警告。 */
    (void)printf("[FATAL] ms901m boot timeout rc=%ld\n", (long)rc);
    /* __WFI() = Wait For Interrupt：让 CPU 进入低功耗休眠，不再执行任何代码。
     * 这是一个永不退出的死循环，等于"宣告系统死亡"。 */
    for (;;) { __WFI(); }
}

/**
 * @brief  上电后等 MS901M 第一帧 0x01 姿态，最长 timeout_ms 毫秒。
 * @return 0 = 拿到了；-1 = 超时。
 *
 * 工作原理：
 *   MS901M 传感器上电后会主动按 200 Hz 频率往串口发送姿态数据帧。
 *   单片机这边是被动接收方，只需要：
 *     1. 从 UART3 的环形缓冲区里把字节捞出来
 *     2. 喂给 ms901m 解析器（状态机会自动找帧头、验校验和、解析数据）
 *     3. 检查 ms901m_has_attitude() 是否为真（是否至少解析到一帧 0x01）
 *
 * 为什么不用 SysTick 节拍？
 *   当前这个函数运行在 main() 的初始化阶段，SysTick 的 1 ms 节拍已启动
 *   但主循环还没开始。直接读 ms 时间戳 + 主动轮询更简单可靠。
 *
 *  MS901M 默认主动按帧上报，主控只需被动 drain UART3 RX 环缓 + 喂解析器。
 *  本函数主动 1 ms 轮询而非 SysTick 节拍，避免和 main 主循环 tick 标志竞争。
 */
static int32_t wait_for_ms901m_attitude(uint32_t timeout_ms)
{
    /* 记录起始时刻（毫秒时间戳） */
    uint32_t start = bsp_systick_get_ms();
    /* 每次从环形缓冲区捞数据的临时存放数组 */
    uint8_t  buf[MS901M_DRAIN_CHUNK];

    /* 循环等待：只要没超时就不停尝试 */
    while ((bsp_systick_get_ms() - start) < timeout_ms) {
        /* 从 IMU 串口的环形缓冲区中批量读取字节。
         * 返回值 got = 实际读到了多少个字节（可能为 0，表示暂无新数据） */
        size_t got = bsp_imu_uart_rx_pop_bulk(buf, sizeof(buf));
        if (got > 0u) {
            /* 有数据就交给状态机解析器逐字节处理。
             * 解析器内部会自动：找 0x55 帧头 → 读 ID/LEN → 收 DATA → 验校验和 */
            ms901m_feed_bytes(buf, got);
            /* 检查解析器是否至少成功收到过一帧 0x01 姿态数据 */
            if (ms901m_has_attitude()) {
                return 0;   /* 成功！返回 0 表示拿到了首帧姿态 */
            }
        }
        /* 没有数据 → 轻睡 1 ms 减小空转占空 */
        bsp_systick_delay_ms(1u);
    }
    return -1;  /* 超时：500 ms 内没收到任何有效帧 */
}

int main(void)
{
    SYSCFG_DL_init();
    bsp_gpio_init();

    if (bsp_systick_init(1000u) != 0) {
        for (;;) { __WFI(); }
    }

    bsp_log_uart_init();
    bsp_k230_uart_init();
    bsp_imu_uart_init();

    /* MS901M 出厂默认 ±4 g / ±2000 dps（与 ATK 上位机默认量程一致） */
    ms901m_init(4, 2000);

    (void)printf("\n[boot] MSPM0G3507 stage2.2 balance baseline start (MS901M / TB6612 / safety)\n");

    int32_t rc = wait_for_ms901m_attitude(MS901M_BOOT_TIMEOUT_MS);
    if (rc != 0) {
        fatal_imu_init_failure(rc);
    }

    (void)printf("[boot] MS901M attitude online, %lu good / %lu bad frames\n",
        (unsigned long)ms901m_good_frames(),
        (unsigned long)ms901m_bad_frames());

    /* Stage 2.2：电机 / 电池初始化后先进入电机 demo。安全 / 平衡在装车
     * 请求后再初始化，避免 demo 输出被 safety 的 DISARMED 状态覆盖。
     *
     *   bsp_motor_init  必须在 bsp_gpio_init 之后（依赖 BSP_ENC_R_*_PIN 配置）
     *   bsp_battery_init 必须在 SYSCFG_DL_init 之后（依赖 ADC_BAT_INST 已配）
     *   app_safety_init 必须在 bsp_motor_init 之后（构造期会调 brake/enable）
     *   app_balance_init 任意位置都可以（纯 PID 数据结构初始化）。 */
    bsp_motor_init();
    bsp_battery_init();

    /* 启动正常：红灯灭、绿灯由心跳任务接管，蓝灯留给后续状态指示。
     * 注意：app_safety 状态显示也会写 LED_R（FALLEN / BAT_STOP 常亮、BAT_WARN 闪），
     *       这里清一下当作 "Boot OK" 视觉反馈，进入 run() 后由 5 Hz 任务接管。 */
    DL_GPIO_clearPins(BSP_LED_R_PORT, BSP_LED_R_PIN);

    if (!app_motor_demo_run()) {
        for (;;) { __WFI(); }
    }

    (void)printf("[boot] switching to load balance mode with test PID "
                 "Kp=%ld Ki=%ld Kd=%ld (x100)\n",
        (long)(LOAD_TEST_BALANCE_KP * 100.0f),
        (long)(LOAD_TEST_BALANCE_KI * 100.0f),
        (long)(LOAD_TEST_BALANCE_KD * 100.0f));

    app_safety_init();
    app_balance_init();
    app_balance_set_balance_gains(LOAD_TEST_BALANCE_KP,
        LOAD_TEST_BALANCE_KI, LOAD_TEST_BALANCE_KD);
    app_balance_set_speed_gains(LOAD_TEST_SPEED_KP,
        LOAD_TEST_SPEED_KI, LOAD_TEST_SPEED_KD);
    (void)app_safety_arm();

    app_balance_run();
    return 0;
}
