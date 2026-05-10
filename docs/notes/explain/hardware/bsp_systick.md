# BSP_SYSTICK 系统节拍模块 学习笔记

## 📖 一、整体概述

### 1.1 这个模块在项目中的位置

本项目是一个**两轮自平衡瞄准小车**，主控为 **TI MSPM0G3507（Cortex-M0+）**。

SysTick（系统节拍定时器）是 CPU 内核自带的定时器，不需要配置复杂的芯片外设时钟树。
本模块把它配置为 **1 kHz（每 1 毫秒中断一次）**，为整个项目提供"心跳"。

```
┌──────────────────────────────────────────────────────────────┐
│                     app_balance.c                             │
│  使用 bsp_systick_consume_tick() 实现每 5 ms 的平衡控制       │
└────────────────────┬─────────────────────────────────────────┘
                     │
┌────────────────────▼─────────────────────────────────────────┐
│                    bsp_systick.h / bsp_systick.c              │
│  1 kHz SysTick 系统节拍：毫秒计数 + 延时 + tick 消费         │
└────────────────────┬─────────────────────────────────────────┘
                     │
┌────────────────────▼─────────────────────────────────────────┐
│                   CMSIS（Cortex Microcontroller Software       │
│                   Interface Standard）                         │
│  SysTick_Config() / NVIC_SetPriority() / __WFI() / __disable  │
└──────────────────────────────────────────────────────────────┘
```

### 1.2 什么是 SysTick？

SysTick = System Tick Timer（系统节拍定时器）。

它是一个**24 位递减计数器**，每个 CPU 时钟周期减 1，减到 0 时触发一次中断，然后自动重装初始值继续倒数。

```
SysTick 工作原理（形象比喻）：
┌─────────────────────────────────────────┐
│        闹钟（SysTick）                   │
│                                         │
│  设置：每 32000 下响一次                │
│  计数：32000, 31999, 31998, ... 1, 0    │
│                                            │
│  响铃 → 执行 SysTick_Handler()            │
│     ├─ s_ms_count += 1（已经过了 1 ms）   │
│     └─ s_tick_pending = 1（通知主循环）   │
│                                            │
│  重置：又回到 32000 重新开始倒数           │
└─────────────────────────────────────────┘
    ↑ 每 1 毫秒重复一次
```

**SysTick 和普通 TIMER 的区别**：

| 特性 | SysTick | 芯片 TIMER（如 TIMG0） |
|------|---------|----------------------|
| 所属 | CPU 内核（ARM Cortex-M） | 芯片外设 |
| 配置方式 | CMSIS 标准函数 | TI DriverLib 函数 |
| 位宽 | 24 位 | 16 位或 32 位 |
| 代码可移植 | 所有 Cortex-M 通用 | 不同芯片不同 |
| 主要用途 | 系统节拍 / OS 心跳 | PWM / 输入捕获 / 计时 |

### 1.3 文件结构

| 文件 | 作用 |
|------|------|
| `bsp_systick.h` | 头文件：函数声明 + 设计说明 |
| `bsp_systick.c` | 实现文件：所有函数的实现 + HardFault 覆盖 |

### 1.4 核心功能模块

| 模块 | 对应函数 | 作用 |
|------|---------|------|
| **初始化** | `bsp_systick_init()` | 配置 SysTick 为 1 kHz + 设置最高优先级 |
| **获取时间** | `bsp_systick_get_ms()` | 返回上电至今的毫秒数 |
| **阻塞延时** | `bsp_systick_delay_ms()` | 等待 N 毫秒（WFI 省电模式） |
| **Tick 消费** | `bsp_systick_consume_tick()` ⭐ | 非阻塞检查是否又过了 1 毫秒 |
| **ISR** | `SysTick_Handler()` | 每 1 ms 执行一次：计数器 + 1，设置标志 |
| **故障恢复** | `HardFault_Handler()` | 系统崩溃时自动重启 |

---

## 📋 二、全局变量/函数汇总

### 2.1 静态变量

| 变量名 | 类型 | 修饰 | 初始值 | 作用 | 在哪被修改 |
|--------|------|------|--------|------|-----------|
| `s_ms_count` | `uint32_t` | `static volatile` | `0` | 系统毫秒计数器，每 1 ms 加 1 | `SysTick_Handler()` |
| `s_tick_pending` | `uint8_t` | `static volatile` | `0` | Tick 待处理标志，有中断时置 1 | `SysTick_Handler()` / `consume_tick()` |

### 2.2 函数汇总

| 函数名 | 参数 | 返回值 | 功能 | 调用时机 |
|--------|------|--------|------|---------|
| `bsp_systick_init(hz)` | `hz`: 目标频率 | `int32_t`: 0=成功 | 初始化 SysTick + 设置优先级 | main 初始化一次 |
| `bsp_systick_get_ms()` | 无 | `uint32_t`: 毫秒数 | 获取当前时间戳 | 任意时刻 |
| `bsp_systick_delay_ms(ms)` | `ms`: 等待毫秒数 | 无 | 阻塞延时 | 初始化阶段 / 非实时任务 |
| `bsp_systick_consume_tick()` | 无 | `bool`: 是否有 tick | 非阻塞检查 tick | 主循环中频繁调用 |
| `SysTick_Handler()` | 无 | 无 | ISR：计数器+1 + 置标志 | 每 1 ms 自动触发 |
| `HardFault_Handler()` | 无 | 无 | 硬件错误处理：系统复位 | 硬件错误时自动触发 |

---

## 🔄 三、核心逻辑总结

### 3.1 完整工作流程

```
上电复位
  ↓
main()
  ├── SYSCFG_DL_init()
  ├── bsp_systick_init(1000)    ← SysTick 开始以 1 kHz 运行
  │     ├─ SysTick_Config(CPUCLK_FREQ/1000)
  │     └─ NVIC_SetPriority(SysTick_IRQn, 0)
  ├── 其他初始化...
  └── for (;;) 主循环
        └─ if (bsp_systick_consume_tick())
              ├─ 每 1 ms：喂数据给姿态传感器解析器
              ├─ 每 5 ms：执行平衡控制 PID
              ├─ 每 50 ms：发送 VOFA+ 数据
              └─ 每 1000 ms：输出调试日志

        同时 ─── 每 1 ms ─── SysTick ISR 触发 ─── s_ms_count++
                                                    s_tick_pending = 1
```

### 3.2 生产者-消费者模式

这个模块实现了一个典型的**生产者-消费者**模式：

```
生产者（ISR）                    消费者（主循环）
─────────────                   ─────────────
SysTick_Handler()                bsp_systick_consume_tick()
  ↓                                ↓
s_tick_pending = 1  ────────→   if (s_tick_pending) { ... }
（每 1 ms 生产一个 tick）        s_tick_pending = 0
                                  （消费这个 tick）
```

**为什么不用 s_ms_count 的变化来判断 tick？**
`s_ms_count` 是连续递增的——不能区分"是否又有新的 tick"。
`s_tick_pending` 是一个二进制标志——1 表示"有事件"，0 表示"已处理"。
判断"有没有事件"用标志比用计数器更简单直接。

### 3.3 关键设计：SysTick 优先级设为最高

```
SysTick_Config() 默认行为：
  NVIC_SetPriority(SysTick_IRQn, (1<<2)-1 = 3)
  → SysTick 是"最低优先级"

问题：
  如果 GPIO 中断（默认优先级 0）以数十 kHz 频率触发，
  SysTick（优先级 3）永远被压制，无法执行。
  → s_ms_count 不增加 → 主循环 tick 消费返回 false
  → 主循环一直 __WFI() → 整个系统"心跳停摆"

解决方案：
  NVIC_SetPriority(SysTick_IRQn, 0);
  → SysTick 提升为"最高优先级"
  → 任何中断都不会饿死 SysTick
```

### 3.4 关键设计：原子操作（关中断）

`bsp_systick_consume_tick()` 需要读取并清除 `s_tick_pending`，而 `SysTick_Handler()` 也会设置这个变量。如果不保护，可能发生：

```
时刻      主循环                      SysTick_Handler
────      ──────                      ──────────────
T1        if (s_tick_pending) → 0
T2                                    s_tick_pending = 1（发生了 tick）
T3        return false                 ← tick 丢失了！
```

解决方案：在操作 `s_tick_pending` 前关中断，操作完再开：

```c
__disable_irq();             // 关中断 —— SysTick_Handler 不能执行
pending = s_tick_pending;    // 安全读取
s_tick_pending = 0;          // 安全清除
__enable_irq();              // 开中断
```

### 3.5 关键设计：延时函数的溢出安全

`bsp_systick_delay_ms()` 使用 `(int32_t)(target - s_ms_count) > 0` 判断是否超时。

为什么不直接用 `target > s_ms_count`？考虑这种情况：

```
s_ms_count 当前 = 0xFFFFFFF0（即将溢出）
要等待 100 ms
target = 0xFFFFFFF0 + 100 = 0xFFFFFF54（32 位溢出 → 0x00000054）

此时 s_ms_count (= 0xFFFFFFF0) > target (= 0x54)
如果用 target > s_ms_count 判断 → 条件假 → 直接返回 → 错误！

但转 int32_t 后：
(int32_t)(0x54 - 0xFFFFFFF0) = (int32_t)(0x64) = 100 > 0
→ 正确判断"还没到时间"
```

这个技巧利用了**有符号整数溢出**的特性，是嵌入式系统处理时间戳溢出的标准做法。

---

## 🧩 四、代码逐段详解

### 4.1 头文件包含

```c
#include "bsp_systick.h"
#include "ti_msp_dl_config.h"
```

`ti_msp_dl_config.h` 由 SysConfig 自动生成，包含了 `CPUCLK_FREQ` 宏（CPU 时钟频率）。

**为什么不用 CMSIS 的 SystemCoreClock？**
在标准 CMSIS 中，`SystemCoreClock` 保存了 CPU 频率。但 MSPM0G3507 的启动文件没有初始化这个变量，它的值是未定义的（默认为 0）。如果用 `SystemCoreClock` 计算 reload 值，会得到 `0 / hz = 0`，SysTick_Config(0) 会出错。

### 4.2 static volatile 变量

```c
static volatile uint32_t s_ms_count     = 0u;
static volatile uint8_t  s_tick_pending = 0u;
```

**volatile 为什么必要？**
假设没有 volatile：
```c
// 编译器可能把这段代码：
while (s_tick_pending == 0) { __WFI(); }

// 优化成（危险的！）：
if (s_tick_pending == 0) {
    for (;;) { __WFI(); }  // 死循环，永远跳不出来
}
```

因为编译器看到 `s_tick_pending` 在循环中没有被修改，就"聪明"地认为它永远不会变。但实际上 `SysTick_Handler` 会修改它。

加了 volatile：告诉编译器"这个变量可能被意外修改，每次使用都必须从内存重新加载"。

### 4.3 bsp_systick_init()

```c
int32_t bsp_systick_init(uint32_t hz)
{
    if (hz == 0u) { return -1; }
    int32_t rc = (int32_t)SysTick_Config(CPUCLK_FREQ / hz);
    if (rc != 0) { return rc; }
    NVIC_SetPriority(SysTick_IRQn, 0u);
    return 0;
}
```

**SysTick_Config() 内部做了三件事：**
1. 设置重装载寄存器（LOAD）= CPUCLK_FREQ / hz
2. 设置当前值寄存器（VAL）= 0
3. 使能 SysTick 中断 + 启动定时器

**NVIC_SetPriority(SysTick_IRQn, 0) 覆盖了 SysTick_Config 设置的默认优先级。**
SysTick_Config 把 SysTick 设为最低优先级（3），但我们需要最高优先级（0）。

### 4.4 bsp_systick_get_ms()

```c
uint32_t bsp_systick_get_ms(void)
{
    return s_ms_count;
}
```

一行代码——但为什么不关中断保护？
因为 Cortex-M0+ 读取 32 位值 `s_ms_count` 是**原子操作**——一条 LDR 指令完成读取。即使 ISR 在读取过程中修改了它，结果要么是旧值，要么是新值，不会出现"一半旧一半新"的情况。

### 4.5 bsp_systick_delay_ms()

```c
void bsp_systick_delay_ms(uint32_t ms)
{
    uint32_t target = s_ms_count + ms;
    while ((int32_t)(target - s_ms_count) > 0) {
        __WFI();
    }
}
```

**__WFI() 的作用**：Wait For Interrupt——暂停 CPU 执行，进入睡眠模式，直到有中断发生才被唤醒。

`__WFI()` 和纯空转的区别：
- 纯空转：`while(...) {}` → CPU 100% 运行，发热、费电
- `__WFI()`：CPU 进入睡眠 → 几乎不耗电，醒来后继续

### 4.6 bsp_systick_consume_tick() ⭐

```c
bool bsp_systick_consume_tick(void)
{
    bool pending;
    __disable_irq();
    pending = (s_tick_pending != 0u);
    s_tick_pending = 0u;
    __enable_irq();
    return pending;
}
```

这是主循环中最关键的函数。它的工作模式：

```
第一次调用：s_tick_pending = 1
  → 返回 true，s_tick_pending 被清零

第二次调用（1 ms 内）：s_tick_pending = 0
  → 返回 false

第三次调用（下次 SysTick 发生后）：s_tick_pending = 1
  → 返回 true，s_tick_pending 被清零
  ...
```

**原子操作区间**：
`__disable_irq()` 到 `__enable_irq()` 之间的代码不会被中断打断。
虽然关中断的时间很短（几条指令，几十纳秒），但它能保证 `s_tick_pending` 的读取和清除是"不可分割"的操作。

### 4.7 SysTick_Handler()

```c
void SysTick_Handler(void)
{
    s_ms_count    += 1u;
    s_tick_pending = 1u;
}
```

ISR 中的**黄金法则**：只做最必要的事。
- 不在这里做 PID 计算
- 不在这里做 printf
- 不在这里做任何浮点运算

只做了两件事：计数器 +1，置标志。执行时间几十纳秒。

**为什么函数名叫 SysTick_Handler？**
在 CMSIS 标准中，SysTick 异常对应的中断服务函数名是固定的。
启动文件（startup_mspm0g350x_uvision.s）的中断向量表中，
`SysTick_IRQn` 对应的入口被声明为 `SysTick_Handler`（weak 符号）。
只要我们在 C 代码中定义了这个函数，链接器就会用我们的版本覆盖 weak 版本。

### 4.8 HardFault_Handler()

```c
void HardFault_Handler(void)
{
    NVIC_SystemReset();
    while (1) { /* unreachable */ }
}
```

**TI 默认行为**：`B .` 指令 → 死循环 → 卡死。

**我们的行为**：`NVIC_SystemReset()` → 系统复位 → 从 main 重新开始。

为什么自动重启比卡死好？
- 如果 fault 是电磁干扰导致的偶发故障 → 重启后恢复正常
- 如果 fault 是代码 bug 导致的持续故障 → 串口会反复刷 boot log → 立刻定位问题
- 比"卡死了，不知道发生了什么"好得多

---

## 🔍 五、C 语言关键语法知识点

### 5.1 volatile 关键字

```c
static volatile uint32_t s_ms_count = 0u;
```

volatile 告诉编译器："这个变量可能**在程序流程之外**被修改"。
典型场景：
- 被 ISR 修改的变量
- 被硬件寄存器映射的变量
- 被多线程共享的变量

**没有 volatile 的症状**：
```
现象：延时函数永远等不到超时
原因：编译器优化掉了对 s_tick_pending 的重复读取
解决：加 volatile，强制每次从内存读取
```

### 5.2 原子操作

原子操作 = 不可分割的操作（要么全部执行完，要么完全不执行）。

C 语言中的一条语句（如 `s_tick_pending = 0`）在底层可能是多条汇编指令：
```
LDRB  R0, [R1]    ; 加载 s_tick_pending 到 R0
MOVS  R0, #0      ; R0 = 0
STRB  R0, [R1]    ; 把 R0 写回 s_tick_pending
```

如果在 LDRB 和 STRB 之间 ISR 插进来修改了变量，就会出问题。
所以需要关中断来"模拟"原子性。

### 5.3 CMSIS 核心函数速查

| 函数 | 作用 | 底层指令 |
|------|------|---------|
| `SysTick_Config(n)` | 配置 SysTick，reload = n | 操作 STK_LOAD/STK_VAL/STK_CTRL 寄存器 |
| `NVIC_SetPriority(IRQn, prio)` | 设置中断优先级 | 操作 NVIC_IPR 寄存器 |
| `__disable_irq()` | 关全局中断 | `CPSID I`（设置 PRIMASK） |
| `__enable_irq()` | 开全局中断 | `CPSIE I`（清除 PRIMASK） |
| `__WFI()` | 等待中断（睡眠） | `WFI` |
| `NVIC_SystemReset()` | 系统复位 | 操作 SCB_AIRCR 寄存器 |

### 5.4 ARM Cortex-M 优先级

MSPM0G3507 的 NVIC 有 2 位优先级（`__NVIC_PRIO_BITS = 2`），支持 4 级：

| 优先级数值 | 含义 | 配置方式 |
|-----------|------|---------|
| 0 | **最高**优先级 | `NVIC_SetPriority(IRQn, 0)` |
| 1 | 高优先级 | |
| 2 | 低优先级 | |
| 3 | **最低**优先级 | `SysTick_Config()` 的默认值 |

数值越小，优先级越高。这和直觉相反——所以新手容易搞混。

---

## ⚡ 六、在项目中的实际使用

### 6.1 典型的主循环骨架

```c
#include "bsp_systick.h"

int main(void)
{
    SYSCFG_DL_init();
    bsp_systick_init(1000);  // 启动 1 kHz 心跳
    // ... 其他初始化 ...

    uint32_t tick_count = 0;

    for (;;) {
        if (bsp_systick_consume_tick()) {
            tick_count++;

            // 每 1 ms：喂数据给 MS901M 解析器
            uint8_t buf[64];
            size_t n = uart_read(buf, sizeof(buf));
            if (n > 0) ms901m_feed_bytes(buf, n);

            // 每 5 ms：执行平衡控制
            if (tick_count % 5 == 0) {
                ms901m_snapshot_t snap;
                ms901m_get_snapshot(&snap);
                float u = pid_step(&balance_pid,
                    0.0f, snap.pitch_deg, 0.005f);
                motor_set_output(u);
            }

            // 每 50 ms：VOFA+ 波形显示
            if (tick_count % 50 == 0) {
                float ch[] = { snap.pitch_deg, snap.gy_dps, u };
                vofa_send(ch, 3);
            }

            // 每 1000 ms：调试日志
            if (tick_count % 1000 == 0) {
                printf("pitch=%.1f rate=%.1f\n",
                    snap.pitch_deg, snap.gy_dps);
            }
        }

        // 非定时任务：检查按键
        check_button();
    }
}
```

### 6.2 各种时间获取方式对比

| 方式 | 精度 | 阻塞？ | 典型用途 |
|------|------|--------|---------|
| `bsp_systick_get_ms()` | 1 ms | 否 | 计时、超时检测 |
| `bsp_systick_delay_ms(100)` | ±1 ms | **是** | 等待、初始化延时 |
| `bsp_systick_consume_tick()` | 1 ms | 否 | **主循环定时（推荐）** |

### 6.3 定时调度表

| 任务 | 频率 | 每 N ticks | 优先级 |
|------|------|-----------|--------|
| MS901M 数据读取 | 1000 Hz | 1 | 高 |
| 平衡控制 PD | 200 Hz | 5 | 最高 |
| VOFA+ 数据发送 | 20 Hz | 50 | 低 |
| 调试日志输出 | 1 Hz | 1000 | 最低 |

---

## 🐛 七、常见错误与调试方法

### 7.1 常见问题排查

| 现象 | 可能原因 | 解决方法 |
|------|---------|---------|
| 延时不准（偏慢） | SysTick 优先级太低被抢占 | 确认 `NVIC_SetPriority(SysTick_IRQn, 0)` 被调用 |
| 系统整体"卡死" | HardFault 进入死循环 | 确认 `HardFault_Handler()` 被覆盖为 reset |
| `consume_tick()` 一直返回 false | SysTick 没启动 | 检查 `bsp_systick_init()` 返回值 |
| 偶尔丢一个 tick | 关中断时间过长 | 确保关中断区间只做必要的操作 |
| 时间戳溢出导致错误 | 用 `>` 比较时间戳 | 改用 `(int32_t)(t2 - t1) > 0` |

### 7.2 关于关中断时间的注意事项

`__disable_irq()` 和 `__enable_irq()` 之间的代码执行时间应该尽可能短——
因为关中断期间，系统无法响应任何中断（包括 SysTick、UART、GPIO 等）。

本模块中关中断区间只有 3 条语句：
```c
__disable_irq();
pending = (s_tick_pending != 0u);
s_tick_pending = 0u;
__enable_irq();
```

执行时间约几十纳秒——不会造成任何可察觉的中断延迟。

### 7.3 调试建议

如果在调试时发现系统行为异常，首先检查 SysTick 是否正常运行：

```c
// 在调试日志中输出 SysTick 状态
uint32_t now = bsp_systick_get_ms();
static uint32_t last = 0;
if (now - last >= 1000) {  // 每秒打印一次
    printf("SysTick OK: ms=%lu\n", now);
    last = now;
}
```

如果 `ms` 不增长 → SysTick 没有正常运行 → 检查初始化代码。

---

## 📖 八、扩展阅读

### 8.1 Cortex-M 异常模型

Cortex-M 处理器有一个"异常编号"系统，每种异常有固定的编号：

| 异常编号 | 名称 | 优先级 | 触发方式 |
|---------|------|--------|---------|
| 1 | Reset | -3（最高） | 复位引脚或看门狗 |
| 2 | NMI | -2 | NMI 引脚 |
| 3 | HardFault | -1 | 硬件错误 |
| 11 | SVCall | 可编程 | SVC 指令 |
| 12~13 | DebugMon | 可编程 | 调试事件 |
| 14 | PendSV | 可编程 | 软件触发 |
| **15** | **SysTick** | **可编程** | **SysTick 定时器** |
| 16+ | 外设中断 | 可编程 | 外设事件 |

SysTick 是编号 15 的异常，和外设中断（16+）一样，优先级可编程。

### 8.2 PRIMASK 寄存器

PRIMASK 是 Cortex-M 内核中的**优先级掩码寄存器**，只有 1 位：

| PRIMASK 值 | 效果 |
|-----------|------|
| 0 | 所有中断使能（正常状态） |
| 1 | 禁止所有可屏蔽中断（除了 NMI 和 HardFault） |

`__disable_irq()` 设置 PRIMASK = 1
`__enable_irq()` 清除 PRIMASK = 0

注意：PRIMASK 不能屏蔽 HardFault——所以如果关中断期间发生硬件错误，
HardFault_Handler 仍然能执行。

### 8.3 无符号整数溢出/回绕

32 位无符号整数范围：0 ~ 4,294,967,295

```
最大值 + 1 = 0（回绕）
0 - 1 = 0xFFFFFFFF（下溢）
```

利用溢出计算时间差：
```c
uint32_t start = bsp_systick_get_ms();
// ... 一段时间 ...
uint32_t elapsed = bsp_systick_get_ms() - start;
// elapsed = 经过的毫秒数，即使 start > now 也正确
```

为什么即使 start > now 也正确？
因为无符号整数的减法在底层是按照"模运算"实现的——
`a - b` 当 a < b 时，结果 = `0x100000000 + a - b`，即正确的时间差。

### 8.4 24 位 SysTick 的限制

SysTick 的计数器是 24 位的，最大值 16,777,215（0xFFFFFF）。

reload 值的限制：`reload <= 16,777,215`

| CPU 频率 | 最大 SysTick 周期 | 最小 SysTick 频率 |
|---------|-----------------|-----------------|
| 32 MHz | 16,777,215 / 32M ≈ 0.524 秒 | ~1.9 Hz |
| 80 MHz | 16,777,215 / 80M ≈ 0.210 秒 | ~4.8 Hz |

对于 1 kHz（reload = CPU/1000）：
- 32 MHz：reload = 32,000 ✓
- 80 MHz：reload = 80,000 ✓

都远小于 24 位上限，安全。

---

> 本文档配合 `bsp_systick.h` 和 `bsp_systick.c` 中的详细注释阅读效果最佳。加油！
