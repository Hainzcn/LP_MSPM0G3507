# BSP_LOG_UART 日志串口模块 学习笔记

## 📖 一、整体概述

### 1.1 这个模块在项目中的位置

本项目是一个**两轮自平衡瞄准小车**，主控为 **TI MSPM0G3507**。

`bsp_log_uart` 是硬件驱动层中最"不起眼但最常用"的模块——它让 `printf` 能够输出到电脑串口助手，是嵌入式开发中最重要的调试手段。

```
┌──────────────────────────────────────────────────────────────┐
│                    app_xxx.c / main.c                         │
│  printf("pitch = %.1f\n", snap.pitch_deg);                    │
└────────────────────┬─────────────────────────────────────────┘
                     │ printf → fputc (每字符调用)
                     ▼
┌──────────────────────────────────────────────────────────────┐
│               bsp_log_uart.c（本模块）                        │
│                                                              │
│  上半部分：bsp_log_uart_write() —— 阻塞式逐字节 UART 发送    │
│  下半部分：fputc() + _sys_write() —— printf 重定向           │
│           + 一堆 stub 函数满足 C 库依赖                       │
└────────────────────┬─────────────────────────────────────────┘
                     │ UART 发送
                     ▼
┌──────────────────────────────────────────────────────────────┐
│            UART0 硬件 + XDS110 USB-COM 桥                     │
│  单片机 TX → XDS110 → USB → 电脑虚拟串口 → 串口助手显示      │
└──────────────────────────────────────────────────────────────┘
```

### 1.2 本工程中的三个串口

| 串口模块 | 硬件 | 用途 | 方向 | 速率 |
|---------|------|------|------|------|
| **bsp_log_uart** | UART0 (XDS110 USB) | 调试日志 printf | 主要输出 | 115200 |
| bsp_imu_uart | UART1 | 接收 MS901M 姿态数据 | 输入 | 115200 |
| bsp_k230_uart | UART2 | 与 K230 视觉处理器通讯 | 双向 | 待定 |

### 1.3 文件结构

| 文件 | 作用 |
|------|------|
| `bsp_log_uart.h` | 头文件：3 个公开函数声明 + printf retarget 概念说明 |
| `bsp_log_uart.c` | 实现文件：3 个业务函数 + fputc + 16 个 C 库 stub 函数 |

---

## 📋 二、函数汇总

### 2.1 上半部分：业务函数

| 函数名 | 参数 | 返回值 | 功能 | 阻塞？ |
|--------|------|--------|------|--------|
| `bsp_log_uart_init()` | 无 | 无 | 清除 UART 中断标志 | 否 |
| `bsp_log_uart_write(data, len)` | 数据指针 + 长度 | 无 | 逐字节发送到 UART | **是** |
| `bsp_log_uart_read_byte(out)` | 输出指针 | `bool` | 非阻塞读一个字节 | 否 |

### 2.2 下半部分：C 库 retarget 函数（仅 armclang）

| 函数名 | 功能 | 重要性 |
|--------|------|--------|
| `fputc(ch, f)` ⭐ | printf 的底层单字符输出 | **核心** |
| `fgetc(f)` | 从输入流读字符（未实现） | stub |
| `_ttywrch(ch)` | 控制台单字符输出 | stub |
| `_sys_exit(x)` | 程序退出处理 | stub |
| `_sys_open(...)` | 文件打开（不支持） | stub |
| `_sys_close(fh)` | 文件关闭 | stub |
| `_sys_write(...)` ⭐ | **文件写入（printf 实际路径）** | **重要** |
| `_sys_read(...)` | 文件读取（不支持） | stub |
| `_sys_istty(fh)` | 判断是否为终端 | stub |
| `_sys_seek(...)` | 文件定位 | stub |
| `_sys_flen(fh)` | 文件长度 | stub |
| `_sys_tmpnam(...)` | 临时文件名 | stub |
| `_sys_command_string(...)` | 命令行参数 | stub |
| `clock()` | CPU 时钟周期 | stub |
| `time()` | 日历时间 | stub |
| `system()` | 系统命令执行 | stub |
| `getenv()` | 环境变量 | stub |

### 2.3 其他

| 符号 | 类型 | 作用 |
|------|------|------|
| `__use_no_semihosting` | 链接器指令 | 禁止半主机模式 |
| `__stdout` | `FILE` 变量 | 标准输出文件流 |
| `__stdin` | `FILE` 变量 | 标准输入文件流 |
| `__stderr` | `FILE` 变量 | 标准错误文件流 |

---

## 🔄 三、核心逻辑总结

### 3.1 printf 的完整调用链

```
printf("pitch = %.1f\n", pitch);
  │
  ├─ C 库内部：格式化字符串 "pitch = 12.3\n"
  │
  ├─ 对每个字符调用 fputc()
  │     │
  │     ├─ fputc('p', stdout) → _sys_write(...) → bsp_log_uart_write("p") → UART 发 'p'
  │     ├─ fputc('i', stdout) → _sys_write(...) → bsp_log_uart_write("i") → UART 发 'i'
  │     ├─ ...
  │     └─ fputc('\n', stdout)
  │           ├─ 检测到换行符 → 先发 '\r'
  │           └─ 再发 '\n'
  │
  └─ printf 返回
```

### 3.2 阻塞发送过程

`bsp_log_uart_write()` 的逐字节发送过程：

```
数据缓冲区：H  e  l  l  o  \r  \n
           ↑
           data[0] = 'H'

发送 'H'：
  1. isBusy()？→ No → 发送 'H'
  2. isBusy()？→ Yes → 等待
  3. isBusy()？→ No → 发送 'e'
  4. isBusy()？→ Yes → 等待
  ... 重复直到所有字节发送完
```

每个字节的发送时间：在 115200 bps 下，1 字节 ≈ 87 微秒。
发送 100 字节 ≈ 8.7 毫秒。

### 3.3 \n → \r\n 自动转换

```
程序里写：printf("hello\nworld\n");
串口输出：hello\r\nworld\r\n

为什么？
  Windows 串口助手通常期望 \r\n（回车+换行）来换行。
  只发 \n（换行）可能只换行不回行，产生"阶梯状"输出：
    hello
         world
              next

  发 \r\n 可以正确换行：
    hello
    world
    next
```

### 3.4 半主机 vs 重定向

```
半主机模式（默认，危险）：
  printf("hello")
    → 调用半主机函数
    → 执行 BKPT（断点）指令
    → 调试器捕获断点 → 转发到 IDE 控制台
    → 脱离调试器时 → BKPT 触发 HardFault → 卡死 ❌

重定向模式（我们的实现，安全）：
  printf("hello")
    → 调用 fputc（我们自己写的）
    → 调用 bsp_log_uart_write()
    → UART 发送到 XDS110 USB 串口
    → 电脑串口助手显示
    → 脱离调试器也能正常工作 ✅
```

---

## 🧩 四、代码逐段详解

### 4.1 bsp_log_uart_init()

```c
void bsp_log_uart_init(void)
{
    DL_UART_Main_clearInterruptStatus(UART_LOG_INST,
        DL_UART_MAIN_INTERRUPT_RX | DL_UART_MAIN_INTERRUPT_TX);
}
```

**为什么这么短？**
UART 的硬件初始化（引脚配置、波特率设置、时钟使能）已经在 `SYSCFG_DL_init()` 中完成了。这里只需要"善后"——清除 SysConfig 初始化时可能遗留的中断标志。

**清除中断标志的重要性：**
SysConfig 的初始化函数可能会操作 UART 的中断相关寄存器，导致某些中断标志位被意外置 1。如果不清除，一旦全局中断使能，这些"幽灵中断"就会触发。但此时还没有注册 ISR，程序会跳转到默认的 HardFault handler。

### 4.2 bsp_log_uart_write()

```c
void bsp_log_uart_write(const uint8_t *data, size_t len)
{
    if (data == NULL) { return; }
    for (size_t i = 0u; i < len; ++i) {
        while (DL_UART_Main_isBusy(UART_LOG_INST)) { ; }
        DL_UART_Main_transmitDataBlocking(UART_LOG_INST, data[i]);
    }
}
```

**关键点 1：NULL 检查**
`data == NULL` 的检查是"防御性编程"——如果调用方不小心传了 NULL，直接返回而不是解引用 NULL 指针导致崩溃。

**关键点 2：忙等待（busy wait）**
`while (isBusy()) { ; }` 循环体是空语句 `;`。CPU 在循环中不断读取 UART 的状态寄存器，直到发送器空闲才继续。这期间 CPU 不能做其他事情——这就是"阻塞"的含义。

**关键点 3：逐字节发送**
为什么不用 `transmitDataBlocking` 直接发送整个缓冲区？因为这个函数可能只发送单字节。逐字节发送配合 isBusy 检查可以确保每个字节都发送完成后再发下一个，防止发送 FIFO 溢出。

### 4.3 bsp_log_uart_read_byte()

```c
bool bsp_log_uart_read_byte(uint8_t *out)
{
    if (out == NULL) { return false; }
    if (DL_UART_Main_isRXFIFOEmpty(UART_LOG_INST)) { return false; }
    *out = DL_UART_Main_receiveData(UART_LOG_INST);
    return true;
}
```

**非阻塞设计**：如果接收 FIFO 为空，立即返回 false，不等待。这样主循环可以"顺便"检查一下有没有输入，没有就继续做其他事。

**输出参数用法**：`uint8_t *out` 是"输出参数"——函数通过指针修改调用方的变量。调用方需要先定义一个变量，然后把变量的地址传给函数：
```c
uint8_t ch;
if (bsp_log_uart_read_byte(&ch)) {
    process(ch);
}
```

### 4.4 fputc() —— printf 重定向的核心

```c
int fputc(int ch, FILE *f)
{
    (void)f;
    uint8_t c = (uint8_t)ch;
    if (c == (uint8_t)'\n') {
        bsp_log_uart_write((const uint8_t *)"\r", 1u);
    }
    bsp_log_uart_write(&c, 1u);
    return ch;
}
```

**为什么是 `int ch` 而不是 `char ch`？**
C 标准规定 fputc 的参数是 `int`，因为 `EOF`（End Of File）是 -1，而 `char` 可能是无符号的（0~255），无法表示 -1。

**`\n → \r\n` 转换的目的**：
在类 Unix 系统中，`\n` 本身就代表"换行+回行"。但在 Windows 系统中，`\n` 只是换行，不会回行。大多数串口助手运行在 Windows 上，所以需要在 `\n` 前面加 `\r` 才能正确显示。

### 4.5 条件编译 #if defined(__ARMCC_VERSION)

```c
#if defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6000000)
// ... armclang 特有的 stub 代码 ...
#endif
```

`__ARMCC_VERSION` 是 ARM 编译器预定义的宏：

| 编译器 | `__ARMCC_VERSION` 值 |
|--------|---------------------|
| ARM Compiler 5 (AC5) | 5000000 ~ 5999999 |
| ARM Compiler 6 (AC6, armclang) | ≥ 6000000 |
| GCC | **未定义** |
| IAR | **未定义** |

所以这里只在 Arm Compiler 6 下编译这些 stub 函数。如果用 GCC 编译，这些函数不存在，需要采用不同的 retarget 方式。

### 4.6 `__use_no_semihosting` 指令

```c
__asm(".global __use_no_semihosting\n\t");
```

这是一条内联汇编，做两件事：
1. `.global`：把 `__use_no_semihosting` 声明为全局符号
2. `__use_no_semihosting`：告诉 ARM 链接器"这个程序不使用半主机功能"

**工作原理**：
ARM C 库中有两套底层 I/O 实现：
- **半主机版本**：通过调试器（BKPT 指令）转发到主机
- **非半主机版本**：由用户自己提供（fputc 等）

加上 `__use_no_semihosting` 后，链接器选择非半主机版本。如果用户没有实现 fputc 等函数，链接器会报"undefined reference"错误——这比"运行时卡死"好得多。

### 4.7 FILE __stdout / __stdin / __stderr

```c
FILE __stdout;
FILE __stdin;
FILE __stderr;
```

这三个变量是 C 标准库中的"标准 I/O 文件流"。在桌面电脑上，它们由操作系统管理。在单片机上，我们需要为它们分配内存空间。

**为什么 ARM C 库需要它们存在？**
因为 C 库内部定义了 `extern FILE __stdout;` 等声明，链接器需要在某个目标文件中找到它们的定义。如果找不到，链接器报错。所以我们定义它们——即使我们不使用 `stdin` 和 `stderr`。

**注意不要重复定义 `struct __FILE`**：
Keil 的 `<stdio.h>` 中已经定义了 `struct __FILE`。如果在 C 文件中再定义一次，会报"redefinition"错误。所以我们只需要定义 `FILE __stdout;` 变量，不需要定义结构体本身。

### 4.8 所有 stub 函数一览

| 函数 | 为什么需要 | 我们的实现 |
|------|-----------|-----------|
| `_sys_write` | printf 最终调用它写数据 | 调用 bsp_log_uart_write 发送到 UART |
| `_sys_open` | printf 初始化时尝试打开 stdout | 返回 -1（失败），但不会导致 printf 崩溃 |
| `_sys_close` | 程序"结束"时关闭文件 | 空实现，返回 0 |
| `_sys_read` | 程序尝试从 stdin 读取 | 返回 -1（不支持） |
| `_sys_istty` | 判断 stdout 是否为终端 | 返回 0（不是终端） |
| `_sys_seek` / `_sys_flen` | 文件定位/长度 | 返回 -1（不支持） |
| `_sys_exit` | exit() 调用 | 死循环 |
| `_ttywrch` | 单字符控制台输出 | 调用 bsp_log_uart_write |
| `clock` / `time` | C 库时间函数 | 返回默认值（不支持） |
| `system` / `getenv` | 系统命令/环境变量 | 返回 -1 / NULL |

---

## 🔍 五、C 语言关键语法知识点

### 5.1 条件编译 #if / #endif

```c
#if defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6000000)
// ... armclang 特定代码 ...
#endif
```

**作用**：让同一份源码在不同编译器下编译不同的代码。

**为什么需要条件编译**？
不同编译器的 C 库实现不同，对底层函数的需求不同。Arm Compiler 6 (armclang) 需要这些 `_sys_xxx` 函数，而 GCC 不需要。条件编译让源码能同时支持多种编译器。

### 5.2 (void) 消除未使用参数警告

```c
int fputc(int ch, FILE *f)
{
    (void)f;  // 告诉编译器：我故意不用 f 参数
    // ...
}
```

`(void)f;` 生成一条"读取 f 的值但不使用"的代码。编译器看到 f 被"读取"了，就不会产生"unused parameter"警告。

### 5.3 内联汇编 __asm()

```c
__asm(".global __use_no_semihosting\n\t");
```

`__asm()` 是 ARM 编译器提供的在 C 代码中嵌入汇编指令的功能。

**什么是 .global？**
汇编器指令，把符号导出为全局符号，让链接器能够在其他目标文件中看到它。

**什么是 \n\t？**
汇编器要求每条指令占一行。`\n` 换行，`\t` 缩进——模拟汇编源文件中的格式。

### 5.4 输出参数（Output Parameter）

```c
bool bsp_log_uart_read_byte(uint8_t *out)
{
    // ...
    *out = DL_UART_Main_receiveData(UART_LOG_INST);
    return true;
}
```

函数通过**指针参数**修改调用方的变量，这种方式叫"输出参数"。

**为什么不用返回值直接返回读取的字节？**
因为函数需要两种信息：
1. 是否成功读到数据（用 `bool` 返回值）
2. 读到的数据是什么（通过指针 `out` 修改）

如果用 `uint8_t` 返回值，无法区分"读到了 0x00"和"读取失败"。

### 5.5 忙等待（Busy Wait）

```c
while (DL_UART_Main_isBusy(UART_LOG_INST)) {
    ;
}
```

**忙等待 vs 中断**：

| 方式 | CPU 利用率 | 响应速度 | 复杂度 |
|------|-----------|---------|--------|
| 忙等待（本模块） | 低（空转） | 快 | 低 |
| 中断驱动 | 高 | 快 | 高 |
| DMA | 高 | 最快 | 最高 |

对于调试日志这种低频、非实时操作，忙等待是最简单可靠的方式。

---

## ⚡ 六、在项目中的实际使用

### 6.1 典型的初始化和使用

```c
#include "bsp_log_uart.h"
#include <stdio.h>

int main(void)
{
    SYSCFG_DL_init();          // 系统初始化（含 UART 硬件配置）
    bsp_log_uart_init();       // 清除中断标志
    
    printf("System initialized!\n");
    printf("Pitch: %.2f\n", snap.pitch_deg);
    printf("Rate:  %.2f\n", snap.gy_dps);
    
    for (;;) {
        // 主循环...
        
        // 可以随时用 printf 输出调试信息
        if (error_occurred) {
            printf("ERROR: motor overcurrent!\n");
        }
    }
}
```

### 6.2 把日志串口注册为 VOFA+ 的 writer

```c
// 在 bsp_log_uart.h 中声明（函数已在 bsp_log_uart.c 中实现）
// 或者直接在 vofa_set_writer 中使用
vofa_set_writer(bsp_log_uart_write);
```

这样 VOFA+ 的波形数据就通过日志串口发送到电脑上的 VOFA+ 上位机。

### 6.3 不同调试需求的输出方式

| 场景 | 推荐方式 | 示例 |
|------|---------|------|
| 简单打 log | printf | `printf("pitch=%.1f\n", pitch)` |
| 实时波形 | VOFA+ (JustFloat) | `vofa_send(ch, 3)` |
| 二进制数据 | 直接调 write | `bsp_log_uart_write(data, len)` |
| 交互式调试 | read_byte 轮询 | `if (read_byte(&c)) handle(c)` |

---

## 🐛 七、常见错误与调试方法

### 7.1 常见问题排查

| 现象 | 可能原因 | 解决方法 |
|------|---------|---------|
| printf 没有输出 | 忘记调用 `bsp_log_uart_init()` | 在 main 中添加 |
| printf 卡死 | 半主机模式未关闭 | 确认 `__use_no_semihosting` 生效 |
| 串口显示乱码 | 波特率不匹配 | 检查电脑串口助手波特率 = 115200 |
| 换行显示阶梯状 | 没有 `\r\n` 转换 | 确认 fputc 中 `\n` → `\r\n` 逻辑 |
| 链接报 undefined `_sys_xxx` | stub 函数未包含 | 确认 `#if defined(__ARMCC_VERSION)` 条件满足 |
| 链接报 redefinition of `__FILE` | 重复定义了结构体 | 删除自定义的 `struct __FILE` 定义 |
| printf 输出很慢 | 115200 波特率限制 | 降低打印频率，或提高波特率 |

### 7.2 不要在生产代码中使用 printf

printf 虽然是调试利器，但它有一些严重的缺点：

| 问题 | 原因 | 影响 |
|------|------|------|
| 阻塞 | 每发一个字符都要等 UART 空闲 | 可能影响控制循环的实时性 |
| 慢 | 字符串格式化需要 CPU 时间 | 频繁 printf 会拖慢主循环 |
| 代码体积大 | printf 会拉入大量格式化代码 | 增加 Flash 占用 |

**建议**：在调试阶段使用 printf，在最终版本中注释掉或通过条件编译关闭。

### 7.3 调试建议

如果在串口助手中看不到任何输出，按此顺序排查：

```
1. 硬件连接检查：
   用 USB 线连接 LaunchPad 到电脑
   检查设备管理器中是否出现 "XDS110 Class Application/User UART" COM 口

2. 串口助手配置：
   波特率 = 115200
   数据位 = 8
   停止位 = 1
   校验位 = None
   流控制 = None

3. 代码检查：
   main 中是否有 SYSCFG_DL_init() 和 bsp_log_uart_init()？
   printf 是否在初始化之后调用？

4. 链接检查：
   编译时是否出现关于 _sys_xxx 的未定义错误？
   如果有 → stub 条件编译未生效
```

---

## 📖 八、扩展阅读

### 8.1 什么是 UART？

UART = Universal Asynchronous Receiver/Transmitter（通用异步收发器）。

**异步**：不需要时钟线，发送方和接收方各自用自己的时钟。双方必须约定相同的波特率。

**一帧 UART 数据的格式（8N1）**：

```
起始位(1) | 数据位(8) | 校验位(0) | 停止位(1)
    0        0/1/0/1...             1
```

"115200 8N1" 的含义：
- 115200：波特率（每秒传输的位数）
- 8：8 位数据
- N：无校验（No Parity）
- 1：1 位停止位

每传输 1 字节需要 10 位（1 起始 + 8 数据 + 1 停止），
所以 115200 bps 的理论最大传输速率 ≈ 11520 字节/秒。

### 8.2 什么是 printf 重定向（Retarget）？

printf 的完整调用链：

```
printf("fmt", ...)
  → vsnprintf(buf, ...)    格式化输出到内存缓冲区
  → __putchar(ch)          输出单个字符
    → fputc(ch, stdout)    写入标准输出文件流
      → _sys_write(...)     系统调用：写文件
```

在桌面电脑上，最后一步由操作系统完成（写入控制台窗口）。
在单片机上，最后一步需要我们自己实现（写入 UART 寄存器）。

**重定向的过程就是替换最后几步的实现**，让数据走 UART 而不是控制台。

### 8.3 什么是半主机（Semihosting）？

半主机（Semihosting）是一种让单片机通过调试器与主机（电脑）通信的机制。

**工作原理**：
1. 单片机执行 BKPT（断点）指令
2. CPU 暂停，调试器捕获断点事件
3. 调试器读取 CPU 寄存器和内存，确定要执行什么操作
4. 调试器在主机的 IDE 控制台中执行该操作（如打印字符）
5. CPU 恢复执行

**优点**：不需要 UART 引脚，不需要串口助手，直接在 IDE 中看到输出。

**缺点**：脱离调试器后，BKPT 指令触发 HardFault → 程序卡死。

### 8.4 什么是 stub（存根）函数？

Stub 函数 = 只有声明（签名）没有实际功能的函数。

在嵌入式开发中，stub 函数的典型作用是"告诉编译器/链接器：这个函数存在，虽然它什么都不做"。

**为什么需要 stub？**
因为标准 C 库在编译时假设某些函数（如 `_sys_open`）一定存在。在没有操作系统的单片机上，这些函数不存在，链接器会报错。stub 函数提供了"空实现"来满足链接器的要求。

### 8.5 Arm Compiler 6 (armclang) vs GCC vs IAR

本工程使用 **Arm Compiler 6 (armclang)**，这是 Keil MDK 的默认编译器。

| 特性 | armclang (AC6) | GCC | IAR |
|------|---------------|-----|-----|
| 基于 | LLVM/Clang | GCC | 专有 |
| printf retarget 方式 | fputc + _sys_write | _write (syscall) | __write |
| 是否默认 semihosting | 是（需关闭） | 否 | 否 |
| stub 函数 | `_sys_xxx` 系列 | `_write` 一个函数 | `__write` 一个函数 |

条件编译 `#if defined(__ARMCC_VERSION)` 让我们可以在同一份源码中支持多编译器——armclang 编译 stub 部分，GCC/IAR 跳过。

---

> 本文档配合 `bsp_log_uart.h` 和 `bsp_log_uart.c` 中的详细注释阅读效果最佳。加油！
