# BSP_GPIO GPIO 引脚初始化模块 学习笔记

## 📖 一、整体概述

### 1.1 这个模块在项目中的位置

本项目是一个**两轮自平衡瞄准小车**，主控为 **TI MSPM0G3507**。

`bsp_gpio` 是硬件驱动层（Hardware Abstraction Layer）中最基础的模块——它负责配置所有业务相关的 GPIO 引脚，让它们在上电瞬间就处于正确的输入/输出模式和安全的电平状态。

```
┌──────────────────────────────────────────────────────────────┐
│                     main.c                                    │
│  SYSCFG_DL_init() → bsp_gpio_init() → 其他模块初始化...       │
└────────────────────┬─────────────────────────────────────────┘
                     │ bsp_gpio_init() 调用
                     ▼
┌──────────────────────────────────────────────────────────────┐
│               bsp_gpio.h / bsp_gpio.c（本模块）               │
│                                                              │
│  配置 11 个输出引脚 + 3 个输入引脚的初始状态                   │
│  （引脚分配表见 docs/TaskLog/Stage0-PinAllocation.md）         │
└────────────────────┬─────────────────────────────────────────┘
                     │ 被其他模块调用
                     ▼
┌──────────────────────────────────────────────────────────────┐
│  bsp_motor.c    bsp_battery.c    app_balance.c   app_xxx     │
│  （读编码器）    （读电压）       （控制LED/蜂鸣器）            │
└──────────────────────────────────────────────────────────────┘
```

### 1.2 什么是 GPIO？

GPIO = General Purpose Input/Output，通用输入输出。

简单理解：GPIO 引脚就是芯片伸出来的"腿"——你可以控制每条腿输出高电平（3.3V）或低电平（0V），也可以读取每条腿上的电平是高还是低。

**本模块涉及的 GPIO 引脚功能分类：**

| 功能类别 | 引脚数量 | 具体引脚 |
|---------|---------|---------|
| 状态指示（LED） | 3 个输出 | PB26(红)、PB27(绿)、PB22(蓝) |
| 声光提示 | 1 个输出 + 1 个输出 | PA0(蜂鸣器)、PA1(激光使能) |
| 电机方向控制 | 4 个输出 + 1 个输出 | PA15/PA16(AIN1/2)、PA26/PA27(BIN1/2)、PB0(STBY) |
| 按键输入 | 1 个输入 | PA18(一键启动) |
| 编码器输入 | 2 个输入 | PA12(ENC_R_A)、PA13(ENC_R_B) |

### 1.3 文件结构

| 文件 | 作用 |
|------|------|
| `bsp_gpio.h` | 头文件：定义 14 个 BSP 引脚宏（PORT/PIN/IOMUX 三件套）、声明 `bsp_gpio_init()` |
| `bsp_gpio.c` | 实现文件：3 个内部静态初始化函数 + 1 个公开初始化函数 |

---

## 📋 二、宏定义/变量/类型汇总

### 2.1 BSP 引脚宏定义总表

每个引脚定义了三个宏（PORT / PIN / IOMUX），共 14 个引脚 × 3 = 42 个宏。

| 宏名前缀 | PORT | PIN（位号） | IOMUX | 物理引脚 | 功能 |
|---------|------|------------|-------|---------|------|
| `BSP_LED_R_*` | GPIOB | DL_GPIO_PIN_26 | IOMUX_PINCM57 | PB26 | 红色 LED（自检指示灯） |
| `BSP_LED_G_*` | GPIOB | DL_GPIO_PIN_27 | IOMUX_PINCM58 | PB27 | 绿色 LED |
| `BSP_LED_B_*` | GPIOB | DL_GPIO_PIN_22 | IOMUX_PINCM50 | PB22 | 蓝色 LED |
| `BSP_BUZZER_*` | GPIOA | DL_GPIO_PIN_0 | IOMUX_PINCM1 | PA0 | 蜂鸣器 |
| `BSP_LASER_EN_*` | GPIOA | DL_GPIO_PIN_1 | IOMUX_PINCM2 | PA1 | 激光发射器使能 |
| `BSP_STBY_*` | GPIOB | DL_GPIO_PIN_0 | IOMUX_PINCM12 | PB0 | TB6612 待机控制 |
| `BSP_AIN1_*` | GPIOA | DL_GPIO_PIN_15 | IOMUX_PINCM37 | PA15 | 左电机方向 1 |
| `BSP_AIN2_*` | GPIOA | DL_GPIO_PIN_16 | IOMUX_PINCM38 | PA16 | 左电机方向 2 |
| `BSP_BIN1_*` | GPIOA | DL_GPIO_PIN_26 | IOMUX_PINCM59 | PA26 | 右电机方向 1 |
| `BSP_BIN2_*` | GPIOA | DL_GPIO_PIN_27 | IOMUX_PINCM60 | PA27 | 右电机方向 2 |
| `BSP_START_BTN_*` | GPIOA | DL_GPIO_PIN_18 | IOMUX_PINCM40 | PA18 | 一键启动按键 |
| `BSP_ENC_R_A_*` | GPIOA | DL_GPIO_PIN_12 | IOMUX_PINCM34 | PA12 | 右编码器 A 相 |
| `BSP_ENC_R_B_*` | GPIOA | DL_GPIO_PIN_13 | IOMUX_PINCM35 | PA13 | 右编码器 B 相 |

### 2.2 函数汇总

| 函数名 | 修饰 | 功能 | 调用方 |
|--------|------|------|--------|
| `init_outputs_porta()` | `static` | 初始化 PORTA 的 6 个输出引脚（BUZZER/LASER_EN/AIN1/2/BIN1/2） | `bsp_gpio_init()` |
| `init_outputs_portb()` | `static` | 初始化 PORTB 的 4 个输出引脚（STBY/LED_R/G/B） | `bsp_gpio_init()` |
| `init_inputs_porta()` | `static` | 初始化 PORTA 的 3 个输入引脚（START_BTN/ENC_R_A/B） | `bsp_gpio_init()` |
| `bsp_gpio_init()` | 公开 | 依次调用上述 3 个函数，完成所有 GPIO 初始化 | `main.c` |

---

## 🔄 三、核心逻辑总结

### 3.1 GPIO 输出初始化的"三步走"套路

每个输出引脚都遵循严格的初始化顺序：

```
第 1 步：initDigitalOutput(IOMUX)
    ↓  配置引脚为数字输出模式（配置 IOMUX 寄存器）
第 2 步：setPins() / clearPins()
    ↓  设置输出数据寄存器（确定初始电平，但还未出现在引脚上）
第 3 步：enableOutput()
    ↓  使能输出驱动（第 2 步设的电平真正出现在引脚上）
```

**为什么这个顺序这么重要？**

如果第 2 步和第 3 步颠倒：

```
❌ 错误的顺序：
   enableOutput() → 引脚使能输出，但输出数据寄存器是默认值（通常 = 0，但不保证）
   clearPins()    → 设为低电平
                   → 引脚可能在使能瞬间输出不确定电平（"毛刺"或"闪烁"）

✅ 正确的顺序：
   clearPins()    → 输出数据寄存器设为 0（低电平）
   enableOutput() → 引脚使能，立即输出低电平
                   → 引脚电平从使能那一刻就是确定的低电平，无毛刺
```

### 3.2 安全设计：上电全关闭

**原则**：宁可什么都不做，也不能让设备乱动。

所有输出引脚在初始化时都被设为"关闭/安全"状态：

| 引脚 | 初始电平 | 效果 |
|------|---------|------|
| BUZZER | 低（CLEAR） | 蜂鸣器不响 |
| LASER_EN | 低（CLEAR） | 激光关闭 |
| AIN1/AIN2/BIN1/BIN2 | 低（CLEAR） | 电机方向 = 刹车 |
| STBY | 低（CLEAR） | TB6612 待机，电机驱动 Hi-Z |
| LED_G / LED_B | 低（CLEAR） | 绿灯/蓝灯灭 |
| LED_R | **高（SET）** | **红灯亮（自检指示）** |

**STBY 低电平的双重安全**：即使 STBY 引脚本身已经确保电机不转（TB6612 待机模式），AIN/BIN 方向引脚仍然被设为低电平。这是"多重防护"——即使 STBY 电路意外失效，方向引脚还是安全的。

### 3.3 自检指示灯设计

LED_R（红灯）在初始化后被点亮。这是一个"视觉自检"设计：

```
上电启动 → SYSCFG_DL_init() → bsp_gpio_init() → 红灯亮
                                                     ↓
                                           如果红灯亮了 → 初始化成功
                                           如果红灯没亮 → 代码根本没跑到这里
```

在调试阶段，一眼就能看出系统是否正常启动到了 GPIO 初始化阶段。

### 3.4 输入引脚配置：内部上拉 + 施密特触发

三个输入引脚都配置了"内部上拉 + 施密特触发"的双保险：

**内部上拉（PULL_UP）**：
- 引脚悬空时被内部约 32kΩ 电阻拉到 VDD（高电平）
- 防止浮空引脚被环境噪声干扰
- 编码器未连接时引脚保持稳定高电平

**施密特触发（HYSTERESIS）**：
- 上升阈值和下降阈值不同（有"滞回"）
- 信号在阈值附近抖动时不会反复触发
- 压制机械按键抖动和编码器信号毛刺

### 3.5 为什么阶段 1 不开中断？

`bsp_gpio_init()` 只配置引脚的方向和电气特性，**不使能任何 GPIO 中断**。

中断由功能模块在**阶段 2** 自行开启（如 `bsp_motor.c` 在需要读编码器时才使能中断）。

原因：如果在阶段 1 没有 ISR（中断服务函数）时开启了中断，一旦外部信号触发中断，CPU 会跳转到默认的 fault handler，程序直接死机。

---

## 🧩 四、代码逐段详解

### 4.1 bsp_gpio.h：引脚宏定义

#### 4.1.1 三件套宏

```c
#define BSP_LED_R_PORT   GPIOB
#define BSP_LED_R_PIN    DL_GPIO_PIN_26
#define BSP_LED_R_IOMUX  IOMUX_PINCM57
```

为什么需要三个宏而不是一个？

| 宏 | 数据类型 | 用途 | 类比 |
|---|---------|------|------|
| `PORT` | `GPIO_Regs*` | 指定 GPIO 端口基地址 | "几号楼" |
| `PIN` | `uint32_t` 位掩码 | 指定端口内的哪个位 | "几零几室" |
| `IOMUX` | `uint32_t` | 引脚功能复用编号 | "GPS 坐标" |

`DL_GPIO_PIN_26` 展开为 `(1u << 26)` = `0x04000000`，是一个 32 位的位掩码。`DL_GPIO_setPins(GPIOB, mask)` 用这个掩码只操作 PB26 引脚，不影响 PORTB 的其他引脚。

#### 4.1.2 电机方向控制真值表

AIN1/AIN2（左电机）和 BIN1/BIN2（右电机）的组合：

| AIN1 | AIN2 | 左电机状态 |
|------|------|-----------|
| 0 | 0 | 刹车（线圈短接） |
| 0 | 1 | 正转 |
| 1 | 0 | 反转 |
| 1 | 1 | 刹车（线圈短接） |

初始化为 (0, 0) → 刹车态，安全。

#### 4.1.3 编码器信号解读

编码器 A 相和 B 相是两路相位差 90° 的脉冲信号：

```
正转时：
  A ╭─╮   ╭─╮   ╭─╮   ╭─╮
    ╯ ╰───╯ ╰───╯ ╰───╯ ╰───
  B ──╯ ╰───╯ ╰───╯ ╰───╯ ╰─
     ↑ A 上升沿时 B=高

反转时：
  A ╭─╮   ╭─╮   ╭─╮   ╭─╮
    ╯ ╰───╯ ╰───╯ ╰───╯ ╰───
  B ╭─╯ ╰───╯ ╰───╯ ╰───╯ ╰─
     ↑ A 上升沿时 B=低
```

- A 相用于触发中断（每变化一次计一个脉冲）
- B 相用于判断方向（A 相中断时读 B 相电平）

### 4.2 bsp_gpio.c：初始化实现

#### 4.2.1 init_outputs_porta()

初始化 6 个 PORTA 输出引脚。三个步骤清晰可见：

**第 1 步：配置方向**
```c
DL_GPIO_initDigitalOutput(BSP_BUZZER_IOMUX);   // PA0
DL_GPIO_initDigitalOutput(BSP_LASER_EN_IOMUX);  // PA1
// ... 共 6 个
```
每个 `initDigitalOutput` 调用配置一个引脚。这个函数内部操作 IOMUX 寄存器，把引脚从默认功能切换到 GPIO 输出。

**第 2 步：设初始电平**
```c
DL_GPIO_clearPins(GPIOA,
    BSP_BUZZER_PIN | BSP_LASER_EN_PIN |
    BSP_AIN1_PIN | BSP_AIN2_PIN | BSP_BIN1_PIN | BSP_BIN2_PIN);
```
`clearPins` 接受一个"位掩码"，把多个引脚一次性设为低电平。`|` 运算符把多个 `DL_GPIO_PIN_xx` 合并成一个 32 位掩码。

**第 3 步：使能输出**
```c
DL_GPIO_enableOutput(GPIOA, /* 同上位掩码 */);
```
使能后，第 2 步设的低电平才真正出现在引脚上。

#### 4.2.2 init_outputs_portb()

和第 4.2.1 类似，但有一处不同：

```c
DL_GPIO_clearPins(GPIOB, BSP_STBY_PIN | BSP_LED_G_PIN | BSP_LED_B_PIN);
DL_GPIO_setPins(GPIOB, BSP_LED_R_PIN);
```

先 `clear` 三个（STBY/绿灯/蓝灯 = 低），再 `set` 一个（红灯 = 高）。两个函数可以混合使用来控制不同的初始电平。

#### 4.2.3 init_inputs_porta()

```c
DL_GPIO_initDigitalInputFeatures(BSP_START_BTN_IOMUX,
    DL_GPIO_INVERSION_DISABLE,
    DL_GPIO_RESISTOR_PULL_UP,
    DL_GPIO_HYSTERESIS_ENABLE,
    DL_GPIO_WAKEUP_DISABLE);
```

`initDigitalInputFeatures` 函数有 5 个参数：

| 参数位置 | 参数 | 含义 |
|---------|------|------|
| 1 | IOMUX 编号 | 指定要配置的引脚 |
| 2 | INVERSION | 是否取反（DISABLE = 不取反） |
| 3 | RESISTOR | 内部电阻（PULL_UP = 上拉） |
| 4 | HYSTERESIS | 施密特触发（ENABLE = 使能） |
| 5 | WAKEUP | 唤醒功能（DISABLE = 禁用） |

**编码器引脚踩坑修复**：

早期版本中编码器引脚 `RESISTOR_NONE + HYSTERESIS_DISABLE`，导致未接编码器时引脚浮空。浮空引脚 = 天线，环境噪声触发数十 kHz 的 GPIO 中断，CPU 100% 被中断占用。

修复后改为 `PULL_UP + HYSTERESIS_ENABLE`：
- 上拉：悬空时引脚被拉到高电平，稳定
- 施密特：阈值附近的噪声毛刺被压制

#### 4.2.4 bsp_gpio_init() 

```c
void bsp_gpio_init(void)
{
    init_outputs_porta();   // PORTA 输出：蜂鸣器/激光/电机方向
    init_outputs_portb();   // PORTB 输出：STBY/LED
    init_inputs_porta();    // PORTA 输入：按键/编码器
}
```

三个静态函数按"先输出再输入"的顺序调用。输出在输入之前——先确保对外设的控制是安全的，再配置读取外部信号的引脚。

---

## 🔍 五、C 语言关键语法知识点

### 5.1 位掩码（Bitmask）

```c
DL_GPIO_clearPins(GPIOA,
    BSP_BUZZER_PIN | BSP_LASER_EN_PIN | BSP_AIN1_PIN);
```

这里的 `|` 是**按位或**（bitwise OR）运算符。

`BSP_BUZZER_PIN` = `DL_GPIO_PIN_0` = `(1u << 0)` = `0x00000001`
`BSP_LASER_EN_PIN` = `DL_GPIO_PIN_1` = `(1u << 1)` = `0x00000002`
`BSP_AIN1_PIN` = `DL_GPIO_PIN_15` = `(1u << 15)` = `0x00008000`

合并后：`0x00000001 | 0x00000002 | 0x00008000` = `0x00008003`

`DL_GPIO_clearPins(GPIOA, 0x00008003)` 会同时操作 PA0、PA1、PA15 三个引脚——每个为 1 的位对应一个引脚。

### 5.2 static 函数的模块封装

```c
static void init_outputs_porta(void) { ... }
static void init_outputs_portb(void) { ... }
static void init_inputs_porta(void) { ... }
```

三个初始化函数都用了 `static` 修饰，表示它们只在 `bsp_gpio.c` 内部可见。

这是一种"模块内部分解"——把一个大函数 `bsp_gpio_init()` 拆成三个小函数，提高可读性，但又不想让外部代码看到这些内部细节。

### 5.3 DriverLib 函数命名规律

TI DriverLib 的函数命名：

| 前缀 | 含义 |
|------|------|
| `DL_GPIO_` | GPIO 外设驱动库 |
| `initDigitalOutput` | 初始化为数字输出 |
| `initDigitalInputFeatures` | 初始化为数字输入（带特性配置） |
| `setPins` | 设置引脚为高电平 |
| `clearPins` | 清除引脚（设为低电平） |
| `enableOutput` | 使能输出驱动 |

命名规律：`DL_` + 外设名 + `_` + 动作 + 修饰

### 5.4 IOMUX 概念

IOMUX = Input/Output MUltipleXer（输入输出复用器）。

MSPM0G3507 的每个物理引脚可以承担多种功能。例如 PA0 可以是：
- GPIO 输出
- UART0 TX
- SPI0 CLK
- TIMER0 输入捕获
- ...等

IOMUX 编号（如 `IOMUX_PINCM1`）告诉芯片："这个引脚现在用哪个功能"。

`initDigitalOutput(IOMUX_PINCM1)` 在底层做的就是配置 IOMUX 寄存器，让 PA0 连接到 GPIO 输出功能。

---

## ⚡ 六、在项目中的实际使用

### 6.1 典型调用顺序

```c
// 在 main.c 中
#include "bsp_gpio.h"

int main(void)
{
    SYSCFG_DL_init();    // 第 1 步：系统级初始化（时钟、电源、GPIO 端口电源）
    
    bsp_gpio_init();     // 第 2 步：业务 GPIO 初始化（引脚模式 + 初始电平）
                         // → 红灯亮起，表示初始化成功
    
    // 第 3 步：其他模块初始化
    bsp_motor_init();    // 电机驱动初始化（PWM、编码器中断）
    bsp_imu_uart_init(); // IMU 串口初始化
    // ...
    
    // 第 4 步：进入主循环
    for (;;) {
        // 业务代码...
        
        // 需要时点亮其他 LED
        DL_GPIO_setPins(GPIOB, BSP_LED_G_PIN);   // 亮绿灯
        DL_GPIO_clearPins(GPIOB, BSP_LED_R_PIN);  // 灭红灯
        
        // 使能电机
        DL_GPIO_setPins(GPIOB, BSP_STBY_PIN);     // STBY=高 → 电机可以工作
        
        // 控制蜂鸣器
        DL_GPIO_setPins(GPIOA, BSP_BUZZER_PIN);   // 蜂鸣器响
        delay_ms(100);
        DL_GPIO_clearPins(GPIOA, BSP_BUZZER_PIN);  // 蜂鸣器停
        
        // 读取按键
        uint8_t btn_state = DL_GPIO_readPins(
            BSP_START_BTN_PORT, BSP_START_BTN_PIN);
        if (btn_state == 0) {  // 低电平 = 按下
            // 启动...
        }
    }
}
```

### 6.2 LED 状态指示约定

| 状态 | 红灯 | 绿灯 | 蓝灯 |
|------|------|------|------|
| 初始化完成（默认） | 亮 | 灭 | 灭 |
| 平衡模式已启动 | 灭 | 亮 | 灭 |
| 正在循迹 | 灭 | 灭 | 亮 |
| 报警/异常 | **闪烁** | 灭 | 灭 |

---

## 🔗 七、相关文件参考

| 文件 | 与本模块的关系 |
|------|--------------|
| `main.c` | 调用 `bsp_gpio_init()` 完成 GPIO 初始化 |
| `bsp_motor.c` | 使用编码器引脚（ENC_R_A/B）读取电机转速 |
| `bsp_battery.c` | 使用 ADC 引脚（非 GPIO）读取电池电压 |
| `app_balance.c` | 控制 LED 状态指示和蜂鸣器声光提示 |
| `app_safety.c` | 使用 STBY 引脚控制电机紧急停止 |
| `docs/TaskLog/Stage0-PinAllocation.md` | 引脚分配真值表（所有宏的来源） |

---

## 🧩 八、代码设计模式总结

| 设计模式 | 体现位置 | 说明 |
|---------|---------|------|
| **抽象层** | `.h` 中的 BSP_xxx 宏 | 用有名字的宏代替裸引脚编号，换引脚时只改一处 |
| **三步走初始化** | `init_outputs_*()` 函数 | config → set → enable 的严格顺序，防止输出毛刺 |
| **安全默认值** | 所有输出初始化为低电平 | 上电瞬间设备全关闭，防止意外动作 |
| **视觉自检** | LED_R 初始化为高电平 | 红灯亮表示初始化成功，肉眼可见 |
| **防呆设计** | 编码器引脚上拉+施密特 | 即使引脚悬空也不会产生雪崩中断 |
| **模块内部分解** | 3 个 static 函数 | 把大函数拆成小函数，提高可读性又不暴露内部细节 |
| **阶段化中断开启** | 阶段 1 不开中断 | 没有 ISR 时不开中断，避免 fault handler 触发 |

---

## 📖 九、扩展阅读

### 9.1 什么是推挽输出（Push-Pull Output）？

本模块的所有输出引脚都配置为"推挽输出"模式。

推挽输出的含义：引脚内部有两个晶体管——一个"推"（连接到 VDD，输出高电平），一个"挽"（连接到 GND，输出低电平）。任何时候只有一个导通，所以引脚要么输出高，要么输出低，不会出现"高阻态"。

对比"开漏输出"（Open-Drain）：只能输出低电平或高阻态，需要外部上拉电阻才能输出高电平。

### 9.2 什么是上拉电阻（Pull-Up Resistor）？

上拉电阻是把引脚连接到 VDD（电源正极）的电阻。

**为什么需要上拉？**
如果不接上拉，引脚悬空时电平不确定——可能是高，可能是低，也可能在高低之间来回跳变（被环境噪声驱动）。

**上拉电阻的大小**：
- 内部上拉：约 32kΩ（MSPM0G3507 内部集成）
- 外部上拉：通常 4.7kΩ ~ 10kΩ

电阻越大，越省电，但抗噪声能力越弱。32kΩ 是内部上拉的标准值，适合大多数场景。

### 9.3 什么是施密特触发器（Schmitt Trigger）？

施密特触发器是一种"带滞回"的比较器。

普通比较器：
```
输入超过 1.5V → 输出 1（高电平）
输入低于 1.5V → 输出 0（低电平）
问题：输入在 1.5V 附近抖动 → 输出在 0/1 之间反复跳变
```

施密特触发器：
```
输入超过 1.8V → 输出 1（高电平）
输入低于 1.2V → 输出 0（低电平）
输入在 1.2~1.8V 之间 → 保持上次的输出值
效果：即使输入在阈值附近抖动，输出也不会跳变
```

这个 0.6V 的差值叫"滞回电压"（hysteresis voltage），它能有效压制噪声。

### 9.4 什么是 GPIO 的位操作？

MSPM0G3507 的 GPIO 输出寄存器是 32 位的，每一位对应一个引脚。例如：

```
GPIOA 的 DOUT（数据输出）寄存器：
bit 31 ... bit 2  bit 1  bit 0
 0         0       PA1    PA0
```

- `DL_GPIO_setPins(GPIOA, (1u<<0))`：向 DOUT 寄存器的 bit0 写 1 → PA0 输出高电平
- `DL_GPIO_clearPins(GPIOA, (1u<<0))`：向 DOUT 寄存器的 bit0 写 0 → PA0 输出低电平

这种用"一个 32 位整数同时操作多个引脚"的机制，就是**位操作**。它比逐引脚操作更高效——一条指令就能控制最多 32 个引脚。

### 9.5 什么是 IOMUX？

IOMUX 是 MSPM0 芯片内部的一个"多路开关"矩阵。每个物理引脚都有多个"功能"可选：

```
物理引脚 PA0 的可能功能：
  ┌──────────────┐
  │ GPIO 输出    │──┐
  │ UART0 TX     │──┤
  │ SPI0 CLK     │──┼──→ PA0 物理引脚
  │ TIMER0_IO0   │──┤
  │ ...          │──┘
  └──────────────┘
```

IOMUX 寄存器中的值决定了哪个功能连接到这个引脚。`IOMUX_PINCM1 = 1` 表示"PA0 用 GPIO 功能"。

本模块的头文件直接使用 `IOMUX_PINCMxx` 宏，这些宏的定义来自 TI 的芯片头文件 `hw_iomux.h`。

---

> 本文档配合 `bsp_gpio.h` 和 `bsp_gpio.c` 中的详细注释阅读效果最佳。继续加油！
