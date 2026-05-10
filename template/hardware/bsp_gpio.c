/**
 * @file    bsp_gpio.c
 * @brief   GPIO 引脚初始化 —— 手工调用 DriverLib，代替 SysConfig 自动生成
 *
 * ============================================================
 * 这个文件是做什么的？（写给初学者）
 * ============================================================
 * 单片机上电后，并不是所有引脚都自动配置好功能的。
 * 每个引脚需要明确告诉芯片：这个脚是输入还是输出？
 * 如果是输出，初始应该输出高电平还是低电平？
 * 如果是输入，要不要内部上拉电阻？
 *
 * 这个文件（bsp_gpio.c）就是负责"配置引脚"的。
 * 它调用了 TI DriverLib（驱动库）中的函数，把引脚设为正确的模式。
 *
 * 它做的事情可以概括为：
 *   1. 把 11 个引脚设为"输出模式"，并设好初始电平
 *   2. 把 3 个引脚设为"输入模式"，开启内部上拉
 *
 * ============================================================
 * 为什么用手写代码代替 SysConfig？
 * ============================================================
 * SysConfig 是 TI 的图形化配置工具——你在界面上点一点，它自动生成代码。
 * 但在 SDK 2.10 版本中，SysConfig 生成的 GPIO 初始化代码有 bug，
 * 特别是"多个引脚同时配置"时可能出错。
 *
 * 所以本工程选择手动写 GPIO 初始化，好处是：
 *   1. 绕开 SysConfig 的 bug
 *   2. 精确控制初始化顺序（先设电平，再使能输出——防止引脚瞬间闪一下）
 *   3. 代码完全透明——每一行都清楚知道在做什么
 *
 * ============================================================
 * GPIO 初始化的"三步走"套路
 * ============================================================
 * 每个输出引脚的初始化都遵循相同的三步：
 *
 *   第 1 步：initDigitalOutput(IOMUX)
 *     告诉芯片："这个引脚我要当数字输出用"
 *     参数是 IOMUX 编号（引脚的"身份证号"）
 *
 *   第 2 步：setPins() / clearPins()
 *     设置初始电平：高电平（SET）或低电平（CLEAR）
 *     ⚠️ 这一步必须在使能输出之前做！
 *     否则引脚会在使能瞬间输出不确定的电平。
 *
 *   第 3 步：enableOutput()
 *     真正让引脚开始输出电平。
 *     第 2 步设的电平现在才真正出现在引脚上。
 *
 * 输入引脚的初始化则不同（initDigitalInputFeatures）：
 *   配置内部上拉/下拉电阻 + 施密特触发器 + 唤醒功能
 */

/* ============================================================
 * 头文件包含
 * ============================================================
 * #include "bsp_gpio.h"：引入我们自己的头文件，
 *   其中定义了 BSP_xxx_PORT/PIN/IOMUX 等引脚宏，
 *   以及 bsp_gpio_init() 的函数声明。
 *
 * 注意：这里只需要包含 bsp_gpio.h 就够了。
 * bsp_gpio.h 中已经通过 #include "ti_msp_dl_config.h"
 * 引入了所有 TI DriverLib 的函数声明（如 DL_GPIO_initDigitalOutput、
 * DL_GPIO_setPins、DL_GPIO_enableOutput 等）。
 * 这就是头文件"链式包含"的好处——一个 #include 带来所有需要的东西。
 */
#include "bsp_gpio.h"

/* ================================================================
 * PORTA 输出引脚初始化（6 个引脚）
 * ================================================================
 * 这个函数初始化 PORTA 上的 6 个输出引脚：
 *   BUZZER（蜂鸣器）
 *   LASER_EN（激光使能）
 *   AIN1 / AIN2（左电机方向）
 *   BIN1 / BIN2（右电机方向）
 *
 * 所有这些引脚在初始化时都被设为低电平（CLEARED）：
 *   蜂鸣器低电平 → 不响
 *   激光使能低电平 → 激光关闭
 *   方向引脚低电平 → 电机处于"刹车"状态
 *      （实际上 TB6612 的 STBY 也是低电平，电机根本不工作）
 *
 * 这种"上电全关闭"的设计是嵌入式系统的安全惯例——
 * 宁可让设备什么都不做，也不能让设备乱动。
 */
static void init_outputs_porta(void)
{
    /* ---- 第 1 步：配置为数字输出模式 ---- */
    /* DL_GPIO_initDigitalOutput() 是 TI DriverLib 提供的函数，
     * 它的作用是告诉芯片内部硬件："这个引脚引脚我要当数字输出用"。
     * 
     * 这个函数实际上会配置芯片内部的 IOMUX（引脚功能复用）寄存器，
     * 把引脚从默认的高阻态（Hi-Z）切换到 GPIO 输出模式。
     * 
     * 参数是 IOMUX_PINCMxx 编号——这是每个物理引脚的唯一标识。
     * 例如 IOMUX_PINCM1 对应 PA0（蜂鸣器）。
     * 这些编号在芯片数据手册中可以查到。 */
    DL_GPIO_initDigitalOutput(BSP_BUZZER_IOMUX);    /* PA0 → 蜂鸣器 */
    DL_GPIO_initDigitalOutput(BSP_LASER_EN_IOMUX);  /* PA1 → 激光使能 */
    DL_GPIO_initDigitalOutput(BSP_AIN1_IOMUX);      /* PA15 → 左电机方向 1 */
    DL_GPIO_initDigitalOutput(BSP_AIN2_IOMUX);      /* PA16 → 左电机方向 2 */
    DL_GPIO_initDigitalOutput(BSP_BIN1_IOMUX);      /* PA26 → 右电机方向 1 */
    DL_GPIO_initDigitalOutput(BSP_BIN2_IOMUX);      /* PA27 → 右电机方向 2 */

    /* ---- 第 2 步：设置初始电平为低（CLEARED） ---- */
    /* DL_GPIO_clearPins(GPIOA, 位掩码) 把指定的引脚设为低电平。
     * 
     * 第二个参数是"位掩码"（bitmask）——一个 32 位整数，
     * 每个位代表一个引脚。例如：
     *   BSP_BUZZER_PIN = DL_GPIO_PIN_0  = (1u << 0)  = 0x00000001
     *   多个引脚用 | 合并：0x00000001 | 0x00000002 = 0x00000003
     * 
     * DL_GPIO_clearPins() 遍历位掩码中所有为 1 的位，
     * 将对应的引脚输出寄存器写 0（低电平）。
     * 
     * ⚠️ 这一步必须在 enableOutput() 之前做！
     *   如果在 enableOutput() 之后再做，引脚可能先输出一瞬间的高电平
     *   （默认输出值不确定），导致蜂鸣器"滴"一声或激光闪一下。 */
    DL_GPIO_clearPins(GPIOA,
        BSP_BUZZER_PIN | BSP_LASER_EN_PIN |
        BSP_AIN1_PIN | BSP_AIN2_PIN | BSP_BIN1_PIN | BSP_BIN2_PIN);

    /* ---- 第 3 步：使能输出驱动 ---- */
    /* DL_GPIO_enableOutput() 让引脚真正开始输出电平。
     * 在这之前，引脚处于"输入模式"或"高阻态"，
     * 虽然设置了输出数据寄存器（第 2 步），但电平不会出现在引脚上。
     * 
     * 调用这个函数后，第 2 步设置的电平（全部低）才真正出现在引脚上。
     * 
     * 三步的顺序（config → set → enable）非常重要！
     * 如果 set 和 enable 顺序颠倒，引脚会先输出不确定的电平，
     * 导致外设设备（蜂鸣器、激光、电机）在上电瞬间产生不可预测的行为。 */
    DL_GPIO_enableOutput(GPIOA,
        BSP_BUZZER_PIN | BSP_LASER_EN_PIN |
        BSP_AIN1_PIN | BSP_AIN2_PIN | BSP_BIN1_PIN | BSP_BIN2_PIN);
}

/* ================================================================
 * PORTB 输出引脚初始化（4 个引脚）
 * ================================================================
 * 这个函数初始化 PORTB 上的 4 个输出引脚：
 *   STBY（TB6612 待机控制）
 *   LED_R（红色 LED）
 *   LED_G（绿色 LED）
 *   LED_B（蓝色 LED）
 *
 * 特殊设计：LED_R 初始值为高电平（点亮红灯），
 * 其他引脚初始值为低电平。
 *
 * 为什么红灯要亮？
 *   这是一种"自检指示灯"设计——代码运行到这里如果红灯亮了，
 *   说明 GPIO 初始化成功了。调试时一眼就能看到。
 *   如果红灯不亮，说明初始化有问题或者芯片根本没跑起来。
 *   这就是嵌入式开发中常用的"视觉反馈调试法"。
 */
static void init_outputs_portb(void)
{
    /* ---- 第 1 步：配置为数字输出模式 ---- */
    DL_GPIO_initDigitalOutput(BSP_STBY_IOMUX);   /* PB0 → TB6612 待机控制 */
    DL_GPIO_initDigitalOutput(BSP_LED_R_IOMUX);  /* PB26 → 红色 LED */
    DL_GPIO_initDigitalOutput(BSP_LED_G_IOMUX);  /* PB27 → 绿色 LED */
    DL_GPIO_initDigitalOutput(BSP_LED_B_IOMUX);  /* PB22 → 蓝色 LED */

    /* ---- 第 2 步：设置初始电平 ---- */
    /* STBY = 低电平 → TB6612 进入待机模式，电机驱动输出 Hi-Z（安全！）
     *   如上所述，STBY 低电平防止上电瞬间电机乱转。
     * LED_G = 低电平 → 绿灯灭（等待后续业务代码点亮）
     * LED_B = 低电平 → 蓝灯灭
     * 用 clearPins 一次性设置多个引脚为低。 */
    DL_GPIO_clearPins(GPIOB,
        BSP_STBY_PIN | BSP_LED_G_PIN | BSP_LED_B_PIN);

    /* LED_R = 高电平 → 红灯亮（自检指示灯）
     * 用 setPins 单独点亮红灯。
     * 注意：setPins 和 clearPins 可以混合使用——
     * 先 clear 一批，再 set 另一个。 */
    DL_GPIO_setPins(GPIOB, BSP_LED_R_PIN);

    /* ---- 第 3 步：使能输出驱动 ---- */
    /* 注意：这里一次性使能所有 4 个引脚。
     * 第 2 步已经分别设好了每个引脚的电平，
     * 所以在这一步使能后：
     *   STBY=低、LED_G=低、LED_B=低、LED_R=高
     * 全部同时生效，不会产生中间状态。 */
    DL_GPIO_enableOutput(GPIOB,
        BSP_STBY_PIN | BSP_LED_R_PIN | BSP_LED_G_PIN | BSP_LED_B_PIN);
}

/* ================================================================
 * PORTA 输入引脚初始化（3 个引脚）
 * ================================================================
 * 这个函数初始化 PORTA 上的 3 个输入引脚：
 *   START_BTN（一键启动按键）
 *   ENC_R_A（右编码器 A 相）
 *   ENC_R_B（右编码器 B 相）
 *
 * 输入引脚和输出引脚的配置方式完全不同：
 *   输出：要设置"输出什么电平"
 *   输入：要设置"怎么读取外部信号"
 *
 * 每个输入引脚配置了 4 个参数：
 *   1. INVERSION：是否取反（禁用 = 读取到的值就是引脚电平）
 *   2. RESISTOR：内部上拉/下拉（上拉 = 引脚悬空时保持高电平）
 *   3. HYSTERESIS：施密特触发（抗噪声干扰）
 *   4. WAKEUP：是否允许从休眠中唤醒
 *
 * 这些配置对编码器引脚尤其重要！
 * 详见下面的注释。
 */

/* ---- 关于编码器引脚上拉和施密特触发的重要性 ----
 *
 * 这个函数包含一个重要的"踩坑修复"记录。
 * 在早期版本中（Stage 2.4 之前），ENC_R_A 和 ENC_R_B
 * 配置为 RESISTOR_NONE（不上拉）和 HYSTERESIS_DISABLE（不施密特），
 * 导致了一个严重的问题：
 *
 * 问题场景：
 *   当编码器未上电、连接松动、或者调试时没接电机时，
 *   PA12 和 PA13 引脚处于"悬空"（floating）状态。
 *   悬空的引脚电平不确定，会被环境电磁噪声随意改变。
 *   如果此时 A 相中断已经开启，噪声会触发数十 kHz 的边沿中断，
 *   CPU 100% 的时间都在处理中断，主循环和 SysTick 全部饿死。
 *
 * 这个问题的根源是：
 *   1. 浮空引脚 = 天线——会接收环境电磁噪声
 *   2. 数字电路输入端最怕浮空（会产生"震颤"现象）
 *   3. 没有施密特触发时，噪声毛刺会被当作有效边沿
 *
 * 修复方案（双保险）：
 *   ① 内部上拉（PULL_UP）：
 *      引脚悬空时被内部 ~32kΩ 电阻拉到 VDD（高电平），
 *      电平稳定在高位，噪声拉不动。
 *      编码器主动驱动时（推挽或开漏输出），驱动电流远大于 32kΩ 上拉，
 *      所以编码器仍然可以正常输出 0 和 1。
 *
 *   ② 施密特触发（HYSTERESIS）：
 *      施密特触发器有"滞回"特性——上升阈值和下降阈值不同。
 *      比如上升阈值 = 1.8V，下降阈值 = 1.2V。
 *      这样电平在阈值附近抖动时，不会反复触发边沿事件。
 *      滞回宽度约 100mV，足以压制大多数噪声毛刺。
 */
static void init_inputs_porta(void)
{
    /* ---- START_BTN：一键启动按键 ---- */
    /* DL_GPIO_initDigitalInputFeatures() 配置输入引脚的特性。
     * 
     * 参数 1（IOMUX）：引脚编号
     * 参数 2（INVERSION）：是否取反。
     *    DL_GPIO_INVERSION_DISABLE = 不取反，读取到的值 = 引脚实际电平。
     *    如果使能取反，引脚高电平时读到 0，低电平时读到 1。
     *    这里不需要取反，因为代码中直接判断按键按下 = 低电平即可。
     * 
     * 参数 3（RESISTOR）：内部电阻配置。
     *    DL_GPIO_RESISTOR_PULL_UP = 内部上拉电阻（约 32kΩ 到 VDD）。
     *    LaunchPad 的 S1 按键按下时连接到 GND（低电平），
     *    松开时如果不做任何处理，引脚会浮空。
     *    上拉电阻保证松开时引脚为高电平，按下时为低电平。
     * 
     * 参数 4（HYSTERESIS）：施密特触发。
     *    DL_GPIO_HYSTERESIS_ENABLE = 使能施密特触发。
     *    按键按下/松开时可能有"抖动"（机械触点弹跳），
     *    施密特触发可以抑制抖动导致的误触发。
     * 
     * 参数 5（WAKEUP）：唤醒功能。
     *    DL_GPIO_WAKEUP_DISABLE = 禁止从休眠模式唤醒。
     *    本项目不使用休眠模式，所以禁用。 */
    DL_GPIO_initDigitalInputFeatures(BSP_START_BTN_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);

    /* ---- ENC_R_A：右编码器 A 相 ---- */
    /* 同样使能内部上拉和施密特触发。
     * 上拉保证编码器未连接时引脚电平稳定（高电平），
     * 施密特触发保证编码器信号边沿清晰无毛刺。
     * 这俩组合是"双保险"——共同防止噪声触发雪崩中断。 */
    DL_GPIO_initDigitalInputFeatures(BSP_ENC_R_A_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);

    /* ---- ENC_R_B：右编码器 B 相 ---- */
    /* B 相的配置和 A 相完全一样。
     * B 相不触发中断，只在 A 相中断时被读取电平来判断方向。
     * 但即使不触发中断，上拉和施密特仍然是必要的——
     * 浮空引脚仍然会导致内部 CMOS 电路消耗额外电流。 */
    DL_GPIO_initDigitalInputFeatures(BSP_ENC_R_B_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
}

/* PORTB 输入：Stage 1.5 后无业务输入。
 * 在项目早期，PORTB 的 PB4 引脚曾作为 MPU6050 的中断输入（IMU_INT）。
 * 当传感器切换到 ATK-MS901M（串口姿态传感器）后，MS901M 通过 UART
 * 主动上报数据，不再需要中断引脚，所以 PB4 被释放。
 * 如果将来需要新增 PORTB 的输入引脚，可参考 init_inputs_porta() 的模板
 * 新建一个 init_inputs_portb() 函数。 */

/* ================================================================
 * 公开 API：bsp_gpio_init()
 * ================================================================
 * 这是本模块唯一对外暴露的函数。
 * 它按顺序调用三个内部静态函数，完成所有 GPIO 的初始化。
 *
 * 调用顺序很重要：
 *   1. 先初始化 PORTA 输出（蜂鸣器、激光、电机方向）
 *   2. 再初始化 PORTB 输出（LED、STBY）
 *   3. 最后初始化 PORTA 输入（按键、编码器）
 *
 * 为什么输出在输入之前？
 *   这是一个"安全习惯"——先把输出引脚设到确定的状态（全部关闭），
 *   再配置输入引脚。这样即使输入引脚的电平变化影响了输出逻辑，
 *   输出引脚也已经有了确定的值。
 *
 * 调用方式（在 main.c 中）：
 * @code
 *   int main(void) {
 *       SYSCFG_DL_init();    // 使能 GPIO 端口电源、配置时钟等
 *       bsp_gpio_init();     // 配置所有业务 GPIO 引脚
 *       // ... 其他初始化 ...
 *   }
 * @endcode
 */
void bsp_gpio_init(void)
{
    /* 第 1 步：初始化 PORTA 的 6 个输出引脚
     * （BUZZER、LASER_EN、AIN1、AIN2、BIN1、BIN2）——全部低电平 */
    init_outputs_porta();

    /* 第 2 步：初始化 PORTB 的 4 个输出引脚
     * （STBY、LED_R、LED_G、LED_B）——STBY/绿/蓝=低，红=高 */
    init_outputs_portb();

    /* 第 3 步：初始化 PORTA 的 3 个输入引脚
     * （START_BTN、ENC_R_A、ENC_R_B）——全部内部上拉+施密特 */
    init_inputs_porta();

    /* 初始化完成后：
     *   ✅ 红灯亮 → 初始化成功，肉眼可见
     *   ✅ 蜂鸣器不响、激光关闭、电机不转 → 安全态
     *   ✅ 按键带内部上拉 → 按下=低，松开=高
     *   ✅ 编码器引脚带内部上拉+施密特 → 悬空时不会误触发中断
     * 
     * 注意：此时还没有使能任何 GPIO 中断！
     * 中断在阶段 2 由各功能模块自行开启（如 bsp_motor.c）。
     * 这样做是为了避免阶段 1 没有 ISR 时触发默认 fault handler。 */
}
