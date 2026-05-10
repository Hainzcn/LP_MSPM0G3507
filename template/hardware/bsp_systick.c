/**
 * @file    bsp_systick.c
 * @brief   1 kHz SysTick 系统节拍的实现 —— 毫秒计数 + 延时 + tick 消费 + HardFault 覆盖
 *
 * ============================================================
 * 这个文件是做什么的？（写给初学者）
 * ============================================================
 * 前面说到，SysTick 就像 CPU 的"心跳"——每 1 毫秒跳一次。
 * 这个文件实现了"心跳怎么跳"以及"其他代码怎么用这个心跳"。
 *
 * 具体来说，它做了 4 件事：
 *   1. 配置 SysTick 定时器为 1 kHz（每 1 ms 触发一次中断）
 *   2. 在 SysTick 中断中维护一个毫秒计数器（s_ms_count）
 *   3. 提供了"延时"、"获取时间"、"检查 tick"三个函数
 *   4. 覆盖了 HardFault_Handler——系统崩溃时自动重启
 *
 * ============================================================
 * 文件内容一览
 * ============================================================
 * 1. 静态变量：s_ms_count（毫秒计数器）+ s_tick_pending（tick 待处理标志）
 * 2. bsp_systick_init() —— 配置 SysTick + 设置优先级
 * 3. bsp_systick_get_ms() —— 返回当前毫秒计数
 * 4. bsp_systick_delay_ms() —— 阻塞延时
 * 5. bsp_systick_consume_tick() —— 非阻塞 tick 消费
 * 6. SysTick_Handler() —— SysTick 中断服务函数
 * 7. HardFault_Handler() —— 硬件错误处理（自动重启）
 */

/* ============================================================
 * 头文件包含
 * ============================================================
 * "bsp_systick.h"：引入本模块的函数声明和类型定义。
 *
 * "ti_msp_dl_config.h"：TI DriverLib 的配置头文件。
 *   这个文件由 SysConfig（TI 的图形化配置工具）自动生成，
 *   包含了 CPUCLK_FREQ（CPU 时钟频率）的宏定义。
 *   CPUCLK_FREQ 的值取决于系统时钟配置——
 *   默认使用 SYSOSC（内部振荡器）= 32 MHz，
 *   如果切换到 HFXT（外部晶振）= 80 MHz，SysConfig 会自动
 *   更新 CPUCLK_FREQ 的值，这个文件不需要做任何修改。
 *
 * 注意：这里不需要再包含 CMSIS 的头文件（如 core_cm0plus.h），
 * 因为 ti_msp_dl_config.h 已经间接包含了它。
 * SysTick_Config()、NVIC_SetPriority()、__WFI() 等 CMSIS 函数
 * 都在 core_cm0plus.h 中有声明。
 */
#include "bsp_systick.h"
#include "ti_msp_dl_config.h"

/* ================================================================
 * 静态全局变量
 * ================================================================
 * 这两个变量都用了 static + volatile 修饰。
 *
 * static：只在当前文件中可见，其他 .c 文件不能直接访问。
 *   要访问它们只能通过公开函数（如 bsp_systick_get_ms()）。
 *   这是封装（encapsulation）的体现。
 *
 * volatile（易变的）：告诉编译器"这个变量可能被中断服务函数意外修改"。
 *   如果不加 volatile，编译器可能优化掉对变量的读取——比如：
 *     while (s_tick_pending == 0) { __WFI(); }
 *   编译器看到 s_tick_pending 在循环中没有被修改，
 *   可能把它优化成：
 *     if (s_tick_pending == 0) { for(;;) { __WFI(); } }
 *   进入死循环永远跳不出来了。
 *   加了 volatile 后，编译器每次读取时都从内存重新加载，
 *   不会做这种"优化"。
 *
 * 总结：被中断（ISR）和主循环同时访问的变量，必须加 volatile！
 */

/**
 * s_ms_count：系统毫秒计数器
 * 在 SysTick_Handler() 中每 1 ms 加 1。
 * 范围 0 ~ 0xFFFFFFFF，约 49.7 天溢出一次。
 * 溢出后自动从 0 重新开始。
 */
static volatile uint32_t s_ms_count       = 0u;

/**
 * s_tick_pending：tick 待处理标志
 * 在 SysTick_Handler() 中被设为 1（有 tick 待处理）。
 * 在 bsp_systick_consume_tick() 中被读取并清零。
 * 这是一个"生产者-消费者"模式——ISR 产生事件，主循环消费事件。
 *
 * 为什么不用 s_ms_count 的变化来判断 tick？
 *   因为 s_ms_count 是连续递增的，不好判断"有没有新 tick"。
 *   s_tick_pending 更直接——它是"事件发生"的二进制标志。
 */
static volatile uint8_t  s_tick_pending   = 0u;

/* ================================================================
 * bsp_systick_init() —— 初始化 1 kHz SysTick
 * ================================================================
 * 这个函数配置 Cortex-M0+ 内核的 SysTick 定时器，
 * 让它以 1 kHz 的频率（每 1 ms）触发 SysTick_Handler 中断。
 *
 * SysTick 是 CPU 内核自带的定时器，特点：
 *   - 24 位递减计数器（最大值 16,777,215）
 *   - 减到 0 自动重装（不用手动重新设置）
 *   - 减到 0 触发 SysTick 异常（中断）
 *   - 所有 Cortex-M 芯片都有，代码可移植
 *
 * CPUCLK_FREQ 的来源：
 *   这个宏定义在 SysConfig 生成的 ti_msp_dl_config.h 中。
 *   它表示 CPU 的时钟频率，单位 Hz。
 *   默认 SYSOSC（内部振荡器）= 32 MHz = 32,000,000 Hz
 *   如果使用外部晶振 HFXT = 80 MHz，SysConfig 会自动更新这个宏。
 *
 * SysTick_Config(reload) 函数：
 *   这是 CMSIS 标准库提供的函数，参数是"重装载值"。
 *   工作原理：SysTick 从 reload 开始递减计数，
 *   每个 CPU 时钟周期减 1，减到 0 触发中断，然后自动重装。
 *
 *   reload = CPUCLK_FREQ / hz
 *   例如：32,000,000 / 1000 = 32,000
 *   意味着 SysTick 从 32000 数到 0，正好 1 毫秒。
 *
 *   限制：reload 不能超过 24 位（16,777,215），
 *   对于 32 MHz CPU 和 1 kHz 目标，32,000 远小于限制。
 *
 * @param hz  目标中断频率（本工程固定传 1000）
 * @return 0 成功；-1 hz=0；正数 SysTick_Config 错误
 */
int32_t bsp_systick_init(uint32_t hz)
{
    /* ---- 参数检查 ---- */
    if (hz == 0u) {
        return -1;  /* 频率不能为 0，否则会除以 0 */
    }

    /* ---- 配置 SysTick 定时器 ----
     * SysTick_Config(CPUCLK_FREQ / hz) 内部做了 3 件事：
     *   1. 设置重装载寄存器（LOAD）
     *   2. 设置当前值寄存器（VAL）= 0（立即开始倒数）
     *   3. 使能 SysTick 中断 + 启动定时器
     *
     * 返回 0 = 成功，非 0 = 失败（如 reload 超过 24 位）。
     *
     * CPUCLK_FREQ 由 SysConfig 在 ti_msp_dl_config.h 中生成，
     * 不用手动改文件。
     *
     * ⚠️ 不能使用 SystemCoreClock：
     *   在标准 CMSIS 中，SystemCoreClock 全局变量保存了 CPU 频率。
     *   但 MSPM0G3507 的启动文件没有初始化这个变量，
     *   所以 SystemCoreClock 的值是未定义的（默认 0），
     *   用它计算 reload 会出错（除以 0）。
     *   所以这里直接用 CPUCLK_FREQ 宏（编译时常量）。
     */
    int32_t rc = (int32_t)SysTick_Config(CPUCLK_FREQ / hz);
    if (rc != 0) {
        return rc;  /* SysTick 配置失败（通常不会发生） */
    }

    /* ---- 设置 SysTick 中断优先级为最高（0） ----
     *
     * 背景知识：
     *   CMSIS 的 SysTick_Config() 内部会调用：
     *     NVIC_SetPriority(SysTick_IRQn, (1UL << __NVIC_PRIO_BITS) - 1UL)
     *   对于 MSPM0G3507，__NVIC_PRIO_BITS = 2（2 位优先级，4 级），
     *   所以这条语句设置 SysTick 优先级为 (1<<2)-1 = 3，即最低优先级。
     *
     * 问题：
     *   如果 SysTick 是最低优先级（3），而其他外设中断（如 GPIO、UART）
     *   是默认最高优先级（0），那么：
     *     - 一旦 GPIO 或 UART 中断触发，SysTick 被"压制"
     *     - 如果 GPIO 中断频率很高（如编码器噪声导致数十 kHz 中断），
     *       SysTick 永远轮不到执行
     *     - 表现为：s_ms_count 不增加 → 主循环 tick 消费返回 false →
     *       主循环一直 __WFI() → 系统"心跳停摆"
     *
     * 解决方案：
     *   把 SysTick 优先级设为最高（0），确保它不会被任何其他中断饿死。
     *   这行代码覆盖了 SysTick_Config() 设置的默认优先级。
     *
     * 副作用说明：
     *   SysTick 变为最高优先级后，如果 SysTick_Handler 执行时间过长，
     *   会延迟其他中断的响应。但我们的 SysTick_Handler 非常短——
     *   只做了两个赋值操作（几纳秒），所以没有负面影响。
     */
    NVIC_SetPriority(SysTick_IRQn, 0u);
    return 0;
}

/* ================================================================
 * bsp_systick_get_ms() —— 获取当前毫秒时间戳
 * ================================================================
 * 直接返回 s_ms_count 的值。
 *
 * 注意：s_ms_count 每毫秒在 SysTick_Handler 中被修改一次，
 * 而主循环可能在任何时刻读取它。这里存在"竞争条件"吗？
 *
 * 对于 uint32_t 类型：
 *   Cortex-M0+ 的加载指令（LDR）可以原子地读取一个 32 位值。
 *   即使 ISR 在读取过程中修改了 s_ms_count，也不会有问题——
 *   因为 LDR 要么读取到修改前的值，要么读取到修改后的值，
 *   不会出现"一半旧一半新"的撕裂情况。
 *
 *   所以这个函数不需要关中断保护。
 *
 * @return 当前毫秒计数（0 ~ 0xFFFFFFFF，约 49.7 天循环）
 */
uint32_t bsp_systick_get_ms(void)
{
    return s_ms_count;
}

/* ================================================================
 * bsp_systick_delay_ms() —— 阻塞延时
 * ================================================================
 * 这个函数让程序暂停执行指定的毫秒数。
 *
 * 实现原理（循环等待 + WFI 睡眠）：
 *   1. 计算目标时间 = 当前时间 + 要等待的毫秒数
 *   2. 循环检查条件：(int32_t)(target - s_ms_count) > 0
 *   3. 条件为真 → 调用 __WFI() 睡眠，等待下一次 SysTick 中断唤醒
 *   4. 条件为假 → 时间到了，返回
 *
 * 为什么用 (int32_t) 强制转换？
 *   这是为了处理"溢出"问题：
 *   假设 s_ms_count 当前是 0xFFFFFFF0，要等待 100 ms，
 *   target = 0xFFFFFFF0 + 100 = 0xFFFFFF54（溢出了，实际变成了 0x00000054）
 *   此时 target < s_ms_count（0x54 < 0xFFFFFFF0），
 *   如果用 target > s_ms_count 判断，会直接认为"时间到了"。
 *
 *   但转成 int32_t 后：
 *   (int32_t)(0x54 - 0xFFFFFFF0) = (int32_t)(0x64) = 100
 *   100 > 0 → 继续等待！正确！
 *
 *   这个技巧利用了有符号整数的溢出特性，
 *   是嵌入式系统中处理时间戳溢出的标准做法。
 *
 *   while 循环中的内容：__WFI() + 空语句
 *   循环体只有一个 __WFI();，没有其他语句。
 *   这种"在循环中等待条件变化"的模式叫"忙等待"（busy wait），
 *   但因为加了 __WFI()，CPU 在等待期间进入睡眠模式，
 *   所以实际上是"省电等待"。
 *
 *   __WFI() 的作用：
 *     Wait For Interrupt（等待中断）——ARM 指令。
 *     执行后 CPU 暂停，直到有中断发生才被唤醒。
 *     唤醒后检查时间到了没有，如果没到继续 WFI。
 *     这样 CPU 在等待期间几乎不耗电。
 *
 * @param ms  要延时的毫秒数
 */
void bsp_systick_delay_ms(uint32_t ms)
{
    /* 计算目标时间：当前时间 + 要等待的时间
     * 注意：即使 target 溢出也能正确工作（见上述 int32_t 转换） */
    uint32_t target = s_ms_count + ms;

    /* 循环等待，直到 s_ms_count 达到或超过 target
     * 每次被 SysTick 中断唤醒后检查一次条件 */
    while ((int32_t)(target - s_ms_count) > 0) {
        __WFI();  /* 等待中断——进入睡眠，省电 */
    }
}

/* ================================================================
 * bsp_systick_consume_tick() —— 非阻塞式 tick 消费
 * ================================================================
 * 这是自平衡小车主循环中最关键的时间函数！
 * 它每次返回 true 表示"又过了一毫秒"，返回 false 表示"还没到"。
 *
 * 主循环的典型用法：
 *   for (;;) {
 *       if (bsp_systick_consume_tick()) {
 *           // 每 1 ms 执行一次的任务
 *           feed_ms901m();
 *
 *           if (tick_count % 5 == 0) {
 *               // 每 5 ms 执行一次
 *               balance_control();
 *           }
 *       }
 *       // 不需要定时执行的任务可以放在这里
 *       check_button();
 *   }
 *
 * 原子操作的必要性：
 *   这个函数和 SysTick_Handler（ISR）共享 s_tick_pending 变量。
 *   如果不在关中断的保护下访问 s_tick_pending，可能发生：
 *
 *   主循环                      SysTick_Handler
 *   ├─ if (s_tick_pending)      ├─ s_tick_pending = 1
 *   │   → 读取到 0              │
 *   │                           │（刚才发生了 tick）
 *   └─ return false             └─（标志被 ISR 设了但主循环没看到）
 *
 *   这就是"竞争条件"——主循环在 ISR 设置标志之前读取了它。
 *   结果：tick 丢失了，主循环错过了一次执行机会。
 *
 * 解决方案（PRIMASK 关中断）：
 *   __disable_irq()：设置 PRIMASK 寄存器为 1，关闭所有可屏蔽中断
 *   __enable_irq() ：清除 PRIMASK，重新开启中断
 *
 *   在关中断期间，SysTick_Handler 不能执行，
 *   所以主循环可以安全地读取并清除 s_tick_pending。
 *
 * 为什么不直接用 volatile 防止竞争？
 *   volatile 只能防止编译器优化，不能防止 ISR 和主循环的时序冲突。
 *   原子操作（关中断）才是正确的解决方案。
 *
 * @return true = 有 tick 待处理（又过了一毫秒）
 *         false = 还没有新 tick
 */
bool bsp_systick_consume_tick(void)
{
    bool pending;  /* 临时变量，保存检查结果 */

    /* ---- 关中断（原子操作开始） ---- */
    /* __disable_irq() 是 CMSIS 提供的函数，
     * 它执行 ARM 的 CPSID I 指令，设置 PRIMASK 寄存器。
     * PRIMASK = 1 时，所有可屏蔽中断都被禁止。
     * 注意：SysTick 也是可屏蔽中断（属于异常类型 15），
     * 所以 SysTick_Handler 在关中断期间不会执行。 */
    __disable_irq();

    /* ---- 读取并清除标志 ---- */
    /* 此时 SysTick_Handler 不会执行，所以安全 */
    pending = (s_tick_pending != 0u);  /* 读取 tick 标志 */
    s_tick_pending = 0u;               /* 清除 tick 标志 */

    /* ---- 开中断（原子操作结束） ---- */
    /* __enable_irq() 执行 CPSIE I 指令，清除 PRIMASK。
     * 如果关中断期间有中断触发了（挂起位被置 1），
     * 开中断后 CPU 会立即响应挂起的中断。 */
    __enable_irq();

    return pending;  /* 返回检查结果 */
}

/* ================================================================
 * SysTick_Handler() —— SysTick 中断服务函数
 * ================================================================
 * 这个函数每 1 毫秒被调用一次（由 SysTick 定时器触发）。
 *
 * 它做的事情极少——只有两个操作：
 *   1. s_ms_count += 1（毫秒计数器加 1）
 *   2. s_tick_pending = 1（设置 tick 待处理标志）
 *
 * 这就是嵌入式系统的黄金法则：
 *   **ISR 要尽可能短小精悍！**
 *   中断服务函数中不应该做复杂的事情（如浮点运算、printf），
 *   只应该做最必要的"记录事件"工作，把实际处理留给主循环。
 *
 * 我们的 SysTick_Handler 只执行两个赋值操作，
 * 执行时间大约几十纳秒，几乎不占用 CPU 时间。
 *
 * 函数名的由来：
 *   SysTick_Handler 是 CMSIS 标准中为 SysTick 异常预定义的
 *   中断服务函数名。在启动文件（startup_mspm0g350x_uvision.s）
 *   的中断向量表中，SysTick_IRQn 对应的入口就是这个名字。
 *   所以只要在代码中定义了这个函数，系统就会自动关联。
 */
void SysTick_Handler(void)
{
    s_ms_count    += 1u;  /* 毫秒计数器加 1 */
    s_tick_pending = 1u;  /* 告诉主循环：又过了一毫秒！ */
}

/* ================================================================
 * HardFault_Handler() —— 硬件错误处理（自动重启）
 * ================================================================
 *
 * 问题背景：
 *   TI 的启动文件（startup_mspm0g350x_uvision.s）中，
 *   HardFault_Handler 默认是一个"weak"（弱）函数，
 *   实现为 `B .`（无限循环跳转到自身），即死循环。
 *
 *   这意味着一旦发生硬件错误（如访问非法地址、除以 0、栈溢出），
 *   CPU 直接卡死——串口没输出、按键没反应、无法重启。
 *   在调试时，这种"假死"状态非常劝退初学者。
 *
 * 我们的覆盖实现：
 *   用 `NVIC_SystemReset()` 代替死循环——系统崩溃时自动复位。
 *   复位后系统重新启动，如果故障是偶发的（如电磁干扰），
 *   复位后就能正常工作。
 *   如果故障持续触发，串口可以看到 boot log 反复刷屏，
 *   立即定位到问题，而不是"卡死在某个地方"。
 *
 * 为什么 NVIC_SystemReset() 能复位？
 *   这是 CMSIS 提供的函数，它设置 NVIC 中的应用中断复位寄存器
 *   （AIRCR）的 SYSRESETREQ 位，触发整个系统复位。
 *   复位后所有外设重新初始化，程序从 main() 重新开始执行。
 *
 * 未来扩展（Stage 3+）：
 *   这里可以加更多的故障诊断代码，比如：
 *     1. 在复位前把 CPU 寄存器（PC、LR、SP）保存到备份寄存器
 *     2. 用 RTC 或备份 SRAM 记录错误计数
 *     3. 在启动时检查是否有 HardFault 发生过
 *   但现阶段，先让故障"看得见"比"卡死"要好得多。
 */
void HardFault_Handler(void)
{
    NVIC_SystemReset();  /* 系统复位——从 main() 重新开始 */
    while (1) { /* unreachable */ }  /* 保护：理论上不会执行到这里 */
}
