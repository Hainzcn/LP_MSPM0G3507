/**
 * @file    bsp_gpio.h
 * @brief   GPIO 引脚抽象层 —— 定义所有业务 GPIO 的引脚宏 + 初始化函数声明
 *
 * ============================================================
 * 这个文件是做什么的？（写给初学者）
 * ============================================================
 * 单片机控制外部设备（LED灯、蜂鸣器、电机等）或者读取外部信号（按键、编码器），
 * 都是通过"引脚"（PIN）来完成的。每个引脚都有一个编号（如 PA0、PB26），
 * 但直接写裸编号的话，代码里到处都是魔法数字，看不懂也容易改漏。
 *
 * 这个文件的作用就是：
 *   给每个用到的 GPIO 引脚起一个"有名字的宏"，
 *   比如 BSP_LED_R_PIN 代表"红色LED灯的引脚号"，
 *   这样在代码里写 BSP_LED_R_PIN 比写 DL_GPIO_PIN_26 要清晰得多。
 *
 * 如果将来需要换引脚（比如把红灯从 PB26 换到 PB10），
 * 只需要修改这个头文件中的宏定义，不用去改 c 文件。
 *
 * ============================================================
 * 为什么不用 SysConfig 自动生成 GPIO 代码？
 * ============================================================
 * SysConfig 是 TI 提供的图形化配置工具，可以自动生成初始化代码。
 * 但本工程选择手动写 GPIO 初始化，原因如下：
 *   1. 绕开 SDK 2.10 的 multi-pad codegen bug（多个引脚配置可能生成错误代码）
 *   2. 精细控制初始化顺序（先设输出值，再使能输出——避免引脚瞬间输出不确定电平）
 *   3. 手动代码更透明——每一行都清楚知道在做什么
 *
 * ============================================================
 * 引脚宏的三件套模式
 * ============================================================
 * 每个 BSP 引脚都定义了三个宏：
 *
 *   BSP_xxx_PORT  = 引脚属于哪个 GPIO 端口（GPIOA 或 GPIOB）
 *   BSP_xxx_PIN   = 引脚在该端口内的位号（DL_GPIO_PIN_26 = bit 26）
 *   BSP_xxx_IOMUX = 引脚的 IOMUX 编号（用于引脚功能复用配置）
 *
 * 为什么需要三个？
 *   因为 MSPM0G3507 的 GPIO 操作需要知道三件事：
 *   1. 哪个端口（A 还是 B）—— 类似"第几栋楼"
 *   2. 端口内的哪个位 —— 类似"几楼几号"
 *   3. IOMUX 编号 —— 类似"门牌号的 GPS 坐标"（告诉芯片这个脚是干什么的）
 */

#ifndef BSP_GPIO_H                   /* 头文件保护宏 */
#define BSP_GPIO_H

#include "ti_msp_dl_config.h"        /* TI DriverLib 配置头文件
                                      * 这个文件由 SysConfig 生成，包含了：
                                      *   - GPIOA / GPIOB 的基地址定义
                                      *   - DL_GPIO_PIN_0 ~ DL_GPIO_PIN_31 等位掩码
                                      *   - IOMUX_PINCMxx 引脚复用编号
                                      *   - SYSCFG_DL_init() 函数声明
                                      * 它是所有 DriverLib 调用的基础。
                                      * 注意：用双引号 "" 而不是尖括号 <>，
                                      * 因为它是项目自己的文件（由 SysConfig 生成到项目目录）。 */

/* ================================================================
 * 输出引脚 —— 状态 LED（PORTB）
 * ================================================================
 * LaunchPad 板载了 3 颗 LED：红(R)、绿(G)、蓝(B)。
 * 它们连接到 PORTB 的三根引脚。
 *
 * 用途：显示小车当前状态
 *   - 红灯亮：初始化完成（默认）
 *   - 绿灯亮：平衡模式已启动
 *   - 蓝灯亮：正在循迹
 *   - 闪烁：报警/异常
 *
 * 注意：PB22 / PB26 / PB27 的 IOMUX 编号需要根据芯片数据手册查阅。
 * IOMUX 是"IO MUX"（输入输出复用）的缩写——每个物理引脚可以
 * 被配置为 GPIO、UART、PWM 等不同功能，IOMUX 编号告诉芯片
 * "这个引脚现在用 GPIO 功能"。
 */

/* 红色 LED —— PB26，IOMUX 编号 57
 * 用途：上电自检指示灯（init 完成后点亮，表示初始化成功） */
#define BSP_LED_R_PORT          GPIOB      /* 红色 LED 属于 PORTB */
#define BSP_LED_R_PIN           DL_GPIO_PIN_26  /* PORTB 的第 26 位（即 PB26 引脚）
                                                  * DL_GPIO_PIN_26 展开后是 (1u << 26)，
                                                  * 即一个 32 位整数的 bit26 被置 1。
                                                  * 这种用"位掩码"表示引脚的方式很常见——
                                                  * 可以用一个 32 位整数同时操作多个引脚。 */
#define BSP_LED_R_IOMUX         IOMUX_PINCM57   /* IOMUX 编号 57，对应 PB26 引脚
                                                  * IOMUX_PINCMxx 是 TI DriverLib 中预定义的常量 */

/* 绿色 LED —— PB27，IOMUX 编号 58 */
#define BSP_LED_G_PORT          GPIOB
#define BSP_LED_G_PIN           DL_GPIO_PIN_27
#define BSP_LED_G_IOMUX         IOMUX_PINCM58

/* 蓝色 LED —— PB22，IOMUX 编号 50 */
#define BSP_LED_B_PORT          GPIOB
#define BSP_LED_B_PIN           DL_GPIO_PIN_22
#define BSP_LED_B_IOMUX         IOMUX_PINCM50

/* ================================================================
 * 输出引脚 —— 蜂鸣器 & 激光使能（PORTA）
 * ================================================================
 * 这两个是"单 pad"（单一功能的简单引脚），不需要特殊的外设配置，
 * 直接当作普通 GPIO 推挽输出即可。
 *
 * 蜂鸣器：用于声光提示（到达起点、故障报警等）
 * 激光使能：控制激光发射器的开关（高电平 = 激光开启）
 *   注意：激光发射器有独立供电，这个引脚只是控制信号（开关），
 *   不是给激光器供电的电源引脚。
 */

#define BSP_BUZZER_PORT         GPIOA
#define BSP_BUZZER_PIN          DL_GPIO_PIN_0   /* PA0 */
#define BSP_BUZZER_IOMUX        IOMUX_PINCM1

#define BSP_LASER_EN_PORT       GPIOA
#define BSP_LASER_EN_PIN        DL_GPIO_PIN_1   /* PA1 */
#define BSP_LASER_EN_IOMUX      IOMUX_PINCM2

/* ================================================================
 * 输出引脚 —— TB6612 电机驱动方向 & STBY（Standby）
 * ================================================================
 * TB6612 是一款电机驱动芯片。要控制一个直流电机正反转，
 * 需要两样东西：
 *   1. 方向信号（AIN1/AIN2 或 BIN1/BIN2）：决定电机转动的方向
 *   2. PWM 信号：决定电机转动的速度
 *
 * 本文件只配置方向引脚（AIN1/AIN2/BIN1/BIN2）和待机引脚（STBY）。
 * PWM 引脚由 bsp_motor.c 中的 PWM 模块配置。
 *
 * 安全设计（非常重要！）：
 *   在初始化阶段（main 函数刚开始运行时），
 *   所有方向引脚被设为低电平（CLEARED），
 *   STBY 引脚也被设为低电平。
 *
 *   为什么这是安全的？
 *   因为 TB6612 的 STBY 引脚为低电平时，
 *   芯片进入"待机模式"（Standby Mode），
 *   电机驱动输出为 Hi-Z（高阻态，相当于断开），
 *   此时即使 PWM 有信号，电机也不会转。
 *   这就防止了上电瞬间电机"乱动"的危险情况。
 *
 *   等到业务代码准备就绪后，再把 STBY 拉高，
 *   电机才会响应控制信号。
 */

/* STBY —— TB6612 待机控制引脚（PB0）
 * 低电平 = 待机（电机不转，安全态）
 * 高电平 = 正常工作 */
#define BSP_STBY_PORT           GPIOB
#define BSP_STBY_PIN            DL_GPIO_PIN_0    /* PB0 */
#define BSP_STBY_IOMUX          IOMUX_PINCM12

/* AIN1 / AIN2 —— 左电机（Motor A）方向控制
 *   AIN1=0, AIN2=0 → 刹车（短接）
 *   AIN1=0, AIN2=1 → 正转
 *   AIN1=1, AIN2=0 → 反转
 *   AIN1=1, AIN2=1 → 刹车（短接） */
#define BSP_AIN1_PORT           GPIOA
#define BSP_AIN1_PIN            DL_GPIO_PIN_15   /* PA15 */
#define BSP_AIN1_IOMUX          IOMUX_PINCM37

#define BSP_AIN2_PORT           GPIOA
#define BSP_AIN2_PIN            DL_GPIO_PIN_16   /* PA16 */
#define BSP_AIN2_IOMUX          IOMUX_PINCM38

/* BIN1 / BIN2 —— 右电机（Motor B）方向控制
 * 真值表同 AIN1/AIN2 */
#define BSP_BIN1_PORT           GPIOA
#define BSP_BIN1_PIN            DL_GPIO_PIN_26   /* PA26 */
#define BSP_BIN1_IOMUX          IOMUX_PINCM59

#define BSP_BIN2_PORT           GPIOA
#define BSP_BIN2_PIN            DL_GPIO_PIN_27   /* PA27 */
#define BSP_BIN2_IOMUX          IOMUX_PINCM60

/* ================================================================
 * 输入引脚 —— 阶段 1 只配置方向和内部上拉，不开中断
 * ================================================================
 * 输入引脚与输出引脚的配置不同：
 *   输出：要设置初始电平（高/低）+ 使能输出驱动
 *   输入：要设置是否上拉/下拉 + 是否施密特触发 + 是否唤醒
 *
 * 阶段 1（初期开发阶段）只配置 GPIO 的基本方向（输入/输出），
 * 但**不开中断**。
 *
 * 为什么阶段 1 不开中断？
 *   如果开了中断但没有注册对应的中断服务函数（ISR），
 *   一旦中断触发，CPU 会跳转到默认的 fault handler（错误处理），
 *   导致程序死机。为了安全，中断在阶段 2 各自模块使用时才开启。
 *
 * 内部上拉的作用：
 *   对于按键（START_BTN）：按键按下 = 低电平，松开 = 高电平（上拉保证）
 *   对于编码器（ENC_R_*）：编码器未连接时，上拉保证引脚电平稳定，
 *   不会被环境噪声触发中断。
 */

/* S1 一键启动按键 —— PA18
 * LaunchPad 板载按键，按下时引脚被拉到 GND（低电平），
 * 所以是"低有效"（按下 = 低电平 = true）。
 * 需要内部上拉电阻：按键松开时引脚保持高电平，
 * 不会悬空（floating）导致电平不确定。 */
#define BSP_START_BTN_PORT      GPIOA
#define BSP_START_BTN_PIN       DL_GPIO_PIN_18   /* PA18 */
#define BSP_START_BTN_IOMUX     IOMUX_PINCM40

/* 右编码器 A 相 —— PA12
 * 编码器是测量电机转速和方向的传感器。
 * A 相输出脉冲信号，每转一圈产生固定数量的脉冲。
 * 通过脉冲计数可以算出电机的转速。
 *
 *   ╭─╮   ╭─╮   ╭─╮   ╭─╮
 * A ─╯ ╰───╯ ╰───╯ ╰───╯ ╰───  （脉冲信号）
 * B ───╮   ╭─╮   ╭───╯   ╰─── （相差 90°）
 *      ╰───╯ ╰───╯
 *
 * A 相和 B 相相位差 90°，通过判断哪个相先变化
 * 可以知道电机的旋转方向。
 *
 * 阶段 2 中，A 相将启用双边沿中断（上升沿和下降沿都触发），
 * 每次中断时脉冲计数 +1，同时读取 B 相电平来判断方向。 */
#define BSP_ENC_R_A_PORT        GPIOA
#define BSP_ENC_R_A_PIN         DL_GPIO_PIN_12   /* PA12 */
#define BSP_ENC_R_A_IOMUX       IOMUX_PINCM34

/* 右编码器 B 相 —— PA13
 * B 相的用途：在 A 相中断时读取 B 相的电平，
 * 如果 B 相为高 → 正转；B 相为低 → 反转。
 * B 相自身不触发中断，只在 A 相中断时被查询。 */
#define BSP_ENC_R_B_PORT        GPIOA
#define BSP_ENC_R_B_PIN         DL_GPIO_PIN_13   /* PA13 */
#define BSP_ENC_R_B_IOMUX       IOMUX_PINCM35

/* PB4（原 IMU 中断引脚）已释放说明：
 * 在项目早期（阶段 1），IMU 使用 MPU6050 芯片，
 * MPU6050 通过一个 INT 引脚（PB4）来通知主控"数据已就绪"。
 * 后期切换为 ATK-MS901M 串口姿态传感器后，
 * MS901M 通过 UART 主动上报数据，不再需要 INT 引脚。
 * 所以 PB4 被释放，可以复用于其他功能。
 * 详见 docs/TaskLog/Stage1.5-IMU-Swap-MS901M.md */

/* ================================================================
 * 公开 API
 * ================================================================ */

/**
 * @brief  初始化所有业务 GPIO（输出 + 输入）
 *
 * 这个函数由 main.c 在 SYSCFG_DL_init() 之后立即调用。
 * 它完成所有业务相关的 GPIO 引脚初始化。
 *
 * 函数内部执行了以下操作：
 *
 *   ① 输出引脚初始化（11 个引脚）：
 *      调用 DL_GPIO_initDigitalOutput() 配置为推挽输出模式
 *
 *   ② 输出引脚默认电平设置：
 *      STBY / AIN1 / AIN2 / BIN1 / BIN2 / BUZZER / LASER_EN
 *      / LED_G / LED_B → 全部 CLEARED（低电平，安全态）
 *      LED_R → SET（高电平，点亮红灯表示初始化完成）
 *
 *   ③ 使能输出驱动：
 *      调用 DL_GPIO_enableOutput() 真正让引脚开始输出电平
 *      （注意先设电平再使能——防止引脚瞬间输出不确定值）
 *
 *   ④ 输入引脚初始化（3 个引脚）：
 *      调用 DL_GPIO_initDigitalInputFeatures() 配置为输入模式
 *      START_BTN / ENC_R_A / ENC_R_B 均启用内部上拉 + 施密特触发
 *
 *   ⑤ 不开 NVIC 中断：
 *      中断由阶段 2 各自模块在使用前 enable（如 bsp_motor.c）
 *
 * 调用方式：
 * @code
 *   int main(void) {
 *       SYSCFG_DL_init();     // TI 驱动库初始化（时钟、电源等）
 *       bsp_gpio_init();      // 自定义 GPIO 初始化
 *       // ... 其他初始化 ...
 *   }
 * @endcode
 *
 * @note  这个函数必须在 SYSCFG_DL_init() 之后调用，
 *        因为 SYSCFG_DL_init() 负责使能 GPIO 端口的电源
 *        （DL_GPIO_enablePower(GPIOA/B)）。
 *        在电源未使能时操作 GPIO 寄存器是无效的。
 */
void bsp_gpio_init(void);

#endif /* BSP_GPIO_H */
