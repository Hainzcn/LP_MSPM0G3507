# 阶段 0 ｜ 硬件准备与引脚分配

> 文档定位：自平衡瞄准小车项目的 **引脚分配唯一真源**。任何后续硬件改动 / 外设增减，必须先改本表，再改 `EIDE/LP_MSPM0G3507.syscfg`，再改驱动代码。
>
> 关联文档：
>
> - 项目总览与执行计划：[docs/Overview/Overview.md](../Overview/Overview.md)
> - 芯片 / LaunchPad 跳线物理参考：[docs/Overview/pin.md](../Overview/pin.md)
> - 自动化控制评审注：[docs/chore/temporary.md](../chore/temporary.md)
>
> 主控：MSPM0G3507（LQFP-64 / PM 封装），LP-MSPM0G3507 LaunchPad 开发板。

---

## 1. 任务回顾与决策摘要

阶段 0 共三件事，本文落地前两件：

| # | 阶段 0 任务 | 落地形式 |
|---|-------------|----------|
| 1 | 整理 LaunchPad 跳线现状 | 第 2 节"跳线决策表" |
| 2 | 输出引脚分配表 | 第 3 节"引脚分配总表" + 第 4 节"按外设分组详表" |
| 3 | 通电空载验证 5V / 3V3 与共地 | 第 5 节"上电空载验证清单" |

本阶段已敲定的关键决策（开工前与项目负责人确认）：

| 决策项 | 结论 | 影响 |
|--------|------|------|
| IMU 接口 | I²C（MPU6050 仅支持 I²C） | 占用 I2C1（PB2/PB3）+ 一根 INT |
| 蜂鸣器类型 | 有源（GPIO 高低电平直驱） | 不占 PWM 通道 |
| 蓝牙串口 | **阶段 1 已实例化 UART3 (PB12 TX / PB13 RX, 115200 8N1, RX 中断)** | 用于 MPU6050 姿态数据 VOFA+ 可视化；外设由 UART2 改为 UART3 是为绕开 SDK multi-pad pin 模板生成 bug，详见 [Stage1-IMU-BT-Telemetry.md §8](Stage1-IMU-BT-Telemetry.md) |
| 编码器 Z 相 | PB14 配为 TIMG8 IDX 输入（3 Pin Mode） | 编码器无 Z 相也不冲突，留作扩展 |
| **左/右编码器解码方式** | **左 = 硬件 QEI（TIMG8）；右 = GPIO 双边沿中断（阶段 2 评估升级 CAPTURE）** | MSPM0G3507 仅 TIMG8 支持 SysConfig QEI 模块 |
| TB6612 控制方式 | 2 PWM + 4 方向 + 1 STBY | 共 7 根线 |
| TB6612 PWM 频率 | 20 kHz（建议 15~25 kHz） | 避开人耳与 IMU 通带 |
| K230 串口波特率 | 921600 8N1，TX/RX 双向 DMA | 占 UART1（PB6/PB7）+ 2 个 DMA 通道 |
| XDS-UART0 | 保留为开发期日志 | J21/J22 保持 ON |

> **架构变更说明**：原计划"双路硬件 QEI"经 SDK 源码核对（[QEIMSPM0.syscfg.js](file:///A:/Program%20Files/ti/mspm0_sdk_2_10_00_04/source/ti/driverlib/.meta/qei/QEIMSPM0.syscfg.js) 第 175 行 `TIMG(8|9|10|11)` 过滤器 + [Common.js](file:///A:/Program%20Files/ti/mspm0_sdk_2_10_00_04/source/ti/driverlib/.meta/Common.js) 第 1774 行 `getTimerInstances("QEI")`）确认 **MSPM0G3507 上只有 TIMG8 支持硬件 QEI**（TIMG9/10/11 该器件不存在）。因此：
> - **左编码器**走硬件 QEI（TIMG8 + J12，3 Pin Mode），无丢脉冲。
> - **右编码器**阶段 0 暂以 GPIO 双边沿中断方式接入 PA12/PA13，理论 X4 解码，但高速时 ISR 抖动可能丢脉冲。
> - **阶段 2 决策点**：若实测发现右轮在最高速下丢脉冲超过 1 %，把右编码器升级为 CAPTURE 模式（占用 TIMA1/TIMG6/TIMG7 之一的 1 个捕获通道），并把 PA13 从纯 GPIO 升级为 CCP 输入。

---

## 2. 跳线决策表

来源：[docs/Overview/pin.md](../Overview/pin.md) 第 23~52 行。"动作"列描述本项目相对 LaunchPad 出厂默认要做的改变。

| 跳线 | 关联引脚 / 信号 | 默认 | 本项目动作 | 原因 |
|------|------------------|------|-------------|------|
| J4   | PA0 → LED1（红） | ON  | **OFF** | PA0 让给有源蜂鸣器 |
| J5   | PB22 → RGB-Blue  | ON  | 保留 ON | 复用为状态指示蓝灯（声光提示） |
| J6   | PB26 → RGB-Red   | ON  | 保留 ON | 复用为状态指示红灯 |
| J7   | PB27 → RGB-Green | ON  | 保留 ON | 复用为状态指示绿灯 |
| J8   | PA18 → S1 按键 + BSL | ON | 保留 ON | 板载 S1 直接当一键启动按键 |
| J9   | PB24 → 热敏电阻  | (1)-(2) | **OFF** | PB24 释放给电池分压 ADC 输入 |
| J12  | PA29 / PA30 / PB14 → QEI 接口 | 不适用 | **必须 ON** | 用作左编码器硬件 QEI |
| J13  | 模拟域 3V3 → 热敏 / OPA2365 | ON | OFF（可选） | 不使用片上 OPA，可断电避免漏流 |
| J14  | PB23 → BP J1.3 | (1)-(2) PB23 | 保留默认 | 暂不动 BoosterPack |
| J15  | PA16 → BP J3.29 | (1)-(2) PA16 | 保留默认 | 与 4.3 节 AIN2 复用，BoosterPack 排针上不要再连其他设备 |
| J16  | PA22 → 光传感器 OPA0_OUT | ON | **OFF** | PA22 留给蓝牙 UART 或其它扩展 |
| J17  | PA27 → OPA0_IN0- | ON | **OFF** | PA27 让给 TB6612 BIN2 |
| J18  | PA26 → OPA0_IN0+ | ON | **OFF** | PA26 让给 TB6612 BIN1 |
| J19  | PA0 开漏上拉 → 3V3 | (1)-(2) | **OFF** | 蜂鸣器输出，不需要 3V3 上拉 |
| J20  | PA1 开漏上拉 → 3V3 | (1)-(2) | **OFF** | 激光使能输出，不需要上拉 |
| J21  | PA10 → UART0_TX (XDS) | (1)-(2) XDS | 保留 ON | 开发期日志 UART0 |
| J22  | PA11 → UART0_RX (XDS) | (1)-(2) XDS | 保留 ON | 开发期日志 UART0 |
| J101 | XDS110-ET 隔离块 | ON | 保留 ON | 调试期需 XDS110 |

> 验收标准：装车前按上表逐一核对跳线是否到位，把"OFF"的跳线帽统一存放（用于回退 LaunchPad 出厂状态测试）。

---

## 3. 引脚分配总表（项目唯一真源）

> 排序按"功能模块 → 引脚名"。**所有未在本表出现的引脚一律视为未分配**，禁止在驱动代码里直接拉电平。
>
> 方向：IN（输入）/ OUT（输出）/ I/O（双向，仅 I²C SDA 与 SWDIO）。
> 电平：3.3 V（默认）/ 5V-tolerant 标识为 5VT（MSPM0G3507 数字 IO 默认 5V 容忍，参见数据手册）。

### 3.1 必占（不可变更）

| 信号       | 引脚 | LQFP 引脚号 | 方向 | 外设      | 备注 |
|------------|------|------------|------|-----------|------|
| SWDIO      | PA19 | 12         | I/O  | DEBUGSS   | J101 13:14 ON |
| SWCLK      | PA20 | 13         | OUT  | DEBUGSS   | J101 15:16 ON |
| LOG_TX     | PA10 | 56         | OUT  | UART0_TX  | XDS-UART 桥，J21 ON |
| LOG_RX     | PA11 | 57         | IN   | UART0_RX  | XDS-UART 桥，J22 ON |

### 3.2 业务模块

| 信号           | 引脚 | LQFP 引脚号 | 方向 | 外设                         | 备注 |
|----------------|------|------------|------|------------------------------|------|
| ENC_L_A        | PA29 | 36         | IN   | **TIMG8_CCP0** (QEI PHA)     | J12 ON，左轮硬件 QEI |
| ENC_L_B        | PA30 | 37         | IN   | **TIMG8_CCP1** (QEI PHB)     | J12 ON，左轮硬件 QEI |
| ENC_L_IDX      | PB14 | 2          | IN   | **TIMG8_IDX** (3 Pin Mode)   | J12 ON，预留 Z 相 |
| ENC_R_A        | PA12 | 5          | IN   | **GPIO + 双边沿中断**         | 右轮，软件 X4 解码（阶段 2 拟升级 CAPTURE） |
| ENC_R_B        | PA13 | 6          | IN   | **GPIO（无中断）**            | 右轮 B 相，仅在 ENC_R_A 中断 ISR 内读电平判方向 |
| PWMA           | PA8  | 54         | OUT  | TIMA0_CCP0                   | TB6612 左电机 PWM，20 kHz |
| PWMB           | PA9  | 55         | OUT  | TIMA0_CCP1                   | TB6612 右电机 PWM，20 kHz |
| AIN1           | PA15 | 8          | OUT  | GPIO                         | TB6612 左电机方向 1 |
| AIN2           | PA16 | 9          | OUT  | GPIO                         | TB6612 左电机方向 2，BP J15 默认到此脚需断开 |
| BIN1           | PA26 | 30         | OUT  | GPIO                         | TB6612 右电机方向 1，J18 OFF |
| BIN2           | PA27 | 31         | OUT  | GPIO                         | TB6612 右电机方向 2，J17 OFF |
| STBY           | PB0  | 47         | OUT  | GPIO                         | 上电默认低，初始化完成后拉高 |
| IMU_SCL        | PB2  | 50         | OUT  | I2C1_SCL                     | 400 kHz，外部 4.7 kΩ 上拉到 3V3 |
| IMU_SDA        | PB3  | 51         | I/O  | I2C1_SDA                     | 同上 |
| IMU_INT        | PB4  | 52         | IN   | GPIO + EXTI                  | MPU6050 DataReady，上升沿触发 |
| K230_TX        | PB6  | 58         | OUT  | **UART1_TX**                 | DMA TX 通道 |
| K230_RX        | PB7  | 59         | IN   | **UART1_RX**                 | DMA RX 通道 |
| BT_TX          | PB12 | 64         | OUT  | **UART3_TX**                 | 蓝牙 HC-04，115200 8N1，遥测口（PINCM29，单 pad） |
| BT_RX          | PB13 | 1          | IN   | **UART3_RX**                 | 蓝牙 HC-04，RX 中断进环缓冲（PINCM30，单 pad） |
| BUZZER         | PA0  | 33         | OUT  | GPIO                         | 有源蜂鸣器，J4 OFF, J19 OFF |
| START_BTN      | PA18 | 11         | IN   | GPIO + EXTI                  | 板载 S1，J8 ON，下降沿触发 |
| LASER_EN       | PA1  | 34         | OUT  | GPIO                         | J20 OFF，默认低 |
| BAT_VSENSE     | PB24 | 23         | IN   | ADC0 通道 5 (A0_5)           | J9 OFF，分压 1/4 至 3V3 范围内 |
| LED_STATUS_R   | PB26 | 28         | OUT  | GPIO                         | RGB-R，J6 ON |
| LED_STATUS_G   | PB27 | 29         | OUT  | GPIO                         | RGB-G，J7 ON |
| LED_STATUS_B   | PB22 | 21         | OUT  | GPIO                         | RGB-B，J5 ON |

### 3.3 预留（不进 SysConfig，仅占位）

| 信号        | 引脚 | LQFP 引脚号 | 用途                       | 启用条件 |
|-------------|------|------------|---------------------------|----------|
| —           | —    | —          | 当前无预留项                 | 阶段 1 蓝牙引脚已实例化（见 3.2 BT_TX/BT_RX） |

### 3.4 禁用（硬件不可作通用 IO）

| 引脚 | 原因 |
|------|------|
| NRST (38) | 复位 |
| VDD (40) / VSS (41) / VCORE (32) | 电源 |
| PA2 / ROSC (42) | 内部 ROSC |
| PA3 / LFXIN (43) / PA4 / LFXOUT (44) | LFXT 晶振接口 |
| PA5 / HFXIN (45) / PA6 / HFXOUT (46) | HFXT 晶振接口 |
| PA21 / VREF- (17) | ADC VREF- |
| PA23 / VREF+ (24) | ADC VREF+ |

---

## 4. 按外设分组详表

### 4.1 编码器解码（左：硬件 QEI；右：GPIO 中断软件 X4）

| 项目 | 左轮（硬件 QEI） | 右轮（软件 X4 ） |
|------|------------------|-------------------|
| 实现方式 | TIMG8 QEI（3 Pin Mode） | GPIO 双边沿中断 + 同向电平判方向 |
| 定时器 / 资源 | TIMG8（PD0 域，BUSCLK = ULPCLK） | 不占定时器，占 1 个 EXTI 通道 |
| PHA | PA29 = TIMG8_CCP0 | PA12 GPIO IN，RISE_FALL 中断 |
| PHB | PA30 = TIMG8_CCP1 | PA13 GPIO IN，无中断（ISR 内 `DL_GPIO_readPinsIn` 读电平） |
| INDEX | PB14 = TIMG8_IDX（预留 Z 相，电机无 Z 时软件忽略 LOAD 事件） | 无 |
| 计数 | 硬件 X4，LOAD = 0xFFFF，硬件 16-bit 累加 | 软件 X4，32-bit `volatile int32_t enc_r_count` |
| 取数 | 100~1000 Hz 周期读取并差分 | 同左 |
| 抗丢脉冲 | 硬件保证；MSPM0G3507 数字滤波 ≥ 4 BUSCLK | 受 ISR 抖动影响；高速实测后决定是否升 CAPTURE |

> **架构依据**：[QEIMSPM0.syscfg.js](file:///A:/Program%20Files/ti/mspm0_sdk_2_10_00_04/source/ti/driverlib/.meta/qei/QEIMSPM0.syscfg.js) 第 175 行 QEI 过滤器只允许 `TIMG(8|9|10|11)`，MSPM0G3507 上唯一存在的 QEI 实例就是 TIMG8。**硬件上不存在第二路硬件 QEI 选项**，回退路径只能在"GPIO 中断"和"CAPTURE 模式"之间二选一。
>
> **阶段 2 升级判定**：右轮转速峰值约 = 最高线速度 / 轮周长 × 减速比 × 编码线数 × 4（X4）。以 1.0 m/s、Φ65 mm 轮、30:1 减速、11 PPR 计算：peak ≈ 1.0 / (π·0.065) × 30 × 11 × 4 ≈ 6463 Hz。此频率下 GPIO 中断 ISR 仍可满足（MSPM0G3507 ULPCLK 32 MHz，单次 ISR ≤ 200 cycles ≈ 6 µs，6463 Hz × 6 µs = 3.9 % CPU 占用，可接受）。但 **若实测出现脉冲丢失或 ISR 抖动影响平衡环（500 Hz 周期 = 2 ms，ISR 累计不能超 200 µs）**，则在阶段 2 把右编码器升级为 CAPTURE 模式（占用 TIMG6/TIMG7 一通道），并同步更新本表。

### 4.2 TB6612 电机驱动

| 控制方式 | 2 PWM + 4 方向 + 1 STBY |
|----------|--------------------------|
| 左电机 | PWMA = PA8 (TIMA0_C0)，AIN1 = PA15，AIN2 = PA16 |
| 右电机 | PWMB = PA9 (TIMA0_C1)，BIN1 = PA26，BIN2 = PA27 |
| 全局 | STBY = PB0 |
| PWM 频率 | 20 kHz |
| PWM 分辨率 | 1000（10-bit 等效），由 TIMA0 LOAD = (40 MHz / 20 kHz) - 1 = 1999 决定 |
| 死区 | 不需要（TB6612 内部已处理） |
| 真值表 | AIN1/AIN2 = 10 → 正转；01 → 反转；11 → 短路刹车；00 → 滑行 |

### 4.3 IMU MPU6050

| 项目 | 配置 |
|------|------|
| 接口 | I²C1，主机模式，400 kHz 标准模式 |
| 引脚 | SCL = PB2，SDA = PB3 |
| 上拉 | 外部 4.7 kΩ 至 3V3（板载未提供，需在小车主板/转接板上添加） |
| 中断 | INT = PB4，上升沿触发 + DataReady |
| 软件读取 | DMP 不启用，仅读 raw → 软件互补滤波 / Mahony |
| 期望频率 | 1 kHz IMU 采样、200~500 Hz 平衡环 |

### 4.4 K230 通讯 UART

| 项目 | 配置 |
|------|------|
| 外设 | **UART1**（不是 UART3，UART3 让给蓝牙串口） |
| 引脚 | TX = PB6，RX = PB7 |
| 电平 | 3.3 V（K230 GPIO 也是 3.3 V，可直连，注意共地） |
| 波特率 | 921600 8N1 |
| 流控 | 无（不接 RTS/CTS） |
| DMA | TX/RX 各占 1 个 DMA 通道 |
| 帧格式 | `0xAA 0x55 \| LEN \| CMD \| PAYLOAD \| CRC16 \| 0x55 0xAA`（详见阶段 1 协议） |
| 心跳 | 双向 50 Hz，超时 200 ms 触发降级 |

> **外设选择依据**：MSPM0G3507 LQFP-64 上 PB6/PB7 是 **UART1** 的 TX/RX 引脚（PINCM23/PINCM24，参 [mspm0g350x.h:707/718](file:///A:/Program%20Files/ti/mspm0_sdk_2_10_00_04/source/ti/devices/msp/m0p/mspm0g350x.h)）。

### 4.5 蓝牙串口 UART

| 项目 | 配置 |
|------|------|
| 外设 | **UART3** |
| 引脚 | TX = PB12 (PINCM29)，RX = PB13 (PINCM30) |
| 模块 | HC-04（BC417 经典蓝牙 SPP，需用 USB-TTL AT 配 115200 一次） |
| 电平 | HC-04 Vcc 5V，TXD/RXD 3.3 V LVTTL 可直连 MSPM0 |
| 波特率 | 115200 8N1 |
| FIFO | 启用，深度 4×8 |
| 中断 | RX FIFO 半满中断（用作上位机控制位接收，本阶段进环缓冲不解析） |
| DMA | 不开（VOFA+ JustFloat 帧仅 20 B，阻塞 TX 1.74 ms 可接受） |

> **外设选择依据 + 踩坑记录**：原计划走 UART2 + PB17/PB16（PINCM43/PINCM33），但 SDK 2.10.00.04 在 LQFP-64(PM) 上对该 multi-pad 引脚组生成 `ti_msp_dl_config.h` 时 `getGPIONumberMultiPad → identifyPadIndex` 落到越界索引，崩在 `Common.js:sliceNumber` 上。改用 UART3 + PB12/PB13（单 pad，PINCM29/PINCM30）规避。详细诊断与修复过程见 [Stage1-IMU-BT-Telemetry.md §8](Stage1-IMU-BT-Telemetry.md)。

### 4.6 ADC 电池分压

| 项目 | 配置 |
|------|------|
| 外设 | ADC0 |
| 通道 | 通道 5 → A0_5 → PB24 |
| 分辨率 | 12-bit |
| 参考 | 内部 VREF（2.5 V，可在 SysConfig 切换为外部 VREF+/-） |
| 触发 | 软件触发，平衡环周期内统一采样 |
| 分压 | 外部分压电阻把电池电压（3S Li-ion ≈ 12.6 V）分到 ≤ 3 V 范围（推荐 R1 = 100k，R2 = 22k，比值 ≈ 1/5.5） |
| 软件保护 | 实测 < 9.5 V 触发"低电压告警"，< 9.0 V 触发"安全停车" |

### 4.7 GPIO（汇总）

> **重要**：14 个业务 GPIO 全部由 [`template/hardware/bsp_gpio.{h,c}`](../../template/hardware/bsp_gpio.h) 统一初始化，**不再走 SysConfig**。原因是 SDK 2.10.00.04 GPIO module 在 multi-pad 引脚上有 codegen 体系性 bug（详见 [Stage1-IMU-BT-Telemetry.md §8.5](Stage1-IMU-BT-Telemetry.md)）。改引脚流程：先改本表 → 改 `bsp_gpio.h` 对应宏（`BSP_<NAME>_PORT/PIN/IOMUX`） → 改 `bsp_gpio.c` init 语句。**禁止**在 syscfg GUI 里再 addInstance() GPIO 模块。

| 信号 | 引脚 | 方向 | 上电初值 | 中断 | BSP 宏前缀 | 备注 |
|------|------|------|----------|------|------------|------|
| AIN1 | PA15 | OUT | 0 | — | `BSP_AIN1_*` | TB6612 |
| AIN2 | PA16 | OUT | 0 | — | `BSP_AIN2_*` | TB6612 |
| BIN1 | PA26 | OUT | 0 | — | `BSP_BIN1_*` | TB6612 |
| BIN2 | PA27 | OUT | 0 | — | `BSP_BIN2_*` | TB6612 |
| STBY | PB0  | OUT | 0 | — | `BSP_STBY_*` | 默认禁用电机 |
| BUZZER | PA0 | OUT | 0 | — | `BSP_BUZZER_*` | 默认静音 |
| LASER_EN | PA1 | OUT | 0 | — | `BSP_LASER_EN_*` | 默认关闭激光 |
| LED_STATUS_R | PB26 | OUT | 1（起播红灯，提示未就绪）| — | `BSP_LED_R_*` | bsp_gpio_init 设 SET，IMU init 完成后由 main.c 拉低 |
| LED_STATUS_G | PB27 | OUT | 0 | — | `BSP_LED_G_*` | 5 Hz 心跳由 app_telemetry 翻转 |
| LED_STATUS_B | PB22 | OUT | 0 | — | `BSP_LED_B_*` | 留作业务状态指示 |
| IMU_INT | PB4 | IN  | — | 上升沿（**阶段 1 不开 NVIC**） | `BSP_IMU_INT_*` | DataReady；阶段 1 用 SysTick 1 kHz 轮询 |
| START_BTN | PA18 | IN | — | 下降沿（**阶段 1 不开 NVIC**） | `BSP_START_BTN_*` | S1 按下时拉低，bsp_gpio_init 已配内部上拉 |
| ENC_R_A | PA12 | IN | — | 双边沿（**阶段 1 不开 NVIC**） | `BSP_ENC_R_A_*` | 右编码器 A 相，阶段 2 拟开中断或升 CAPTURE |
| ENC_R_B | PA13 | IN | — | 无中断 | `BSP_ENC_R_B_*` | 右编码器 B 相，ISR 内读电平判方向 |

---

## 5. 上电空载验证清单（阶段 0 第 3 步）

> 装车后**首次上电之前**，逐项打钩。任一项未通过禁止接 MCU 电源。

### 5.1 电源链路独立性

- [ ] 主控 3V3 与 5V 由 LaunchPad XDS110 / 外部 USB 供电，与电机驱动 VM、舵机 VS **物理隔离**（共地、非共电源）。
- [ ] TB6612 VM（电机电源，6~13.5 V）单独经过电池接入，VCC（5V 数字电源）由 MCU 域 5V 提供。
- [ ] 激光器（≤ 5 mW，405 nm）由独立电池供电，使能 GPIO 走光耦或低边 NMOS 隔离，不直接共主控 3V3。
- [ ] K230 由其自身电源供电，与主控仅在 UART3 + GND 上共地。

### 5.2 共地

- [ ] 主控 GND ↔ TB6612 GND ↔ 电池 GND ↔ K230 GND 单点星型汇接（推荐 TB6612 散热片下铜区为汇接点）。
- [ ] 万用表测各 GND 节点之间电阻 < 0.1 Ω。

### 5.3 空载电压

- [ ] 主控 3V3：3.20~3.40 V。
- [ ] 主控 5V：4.85~5.20 V。
- [ ] 电池满电：12.4~12.6 V（3S Li-ion 标称）。
- [ ] BAT_VSENSE 分压点电压：电池电压 × R2/(R1+R2) ≤ 3.0 V，且 ADC 静态读数稳定（抖动 < 5 LSB）。

### 5.4 跳线核对

- [ ] 第 2 节"跳线决策表"逐项核对，所有"OFF"项确认跳线帽已拔。
- [ ] J12 三根线（PA29/PA30/PB14）与编码器 A/B/(Z) 走线一一对应（万用表蜂鸣档点测）。

### 5.5 电机驱动空载

- [ ] STBY 强制拉低时，VM 接通后电机不转，TB6612 静态电流 < 5 mA。
- [ ] STBY 拉高、PWM = 0 时，电机仍不转，TB6612 静态电流 < 10 mA。
- [ ] 缓慢加 PWM（10% 起步），左右轮均能正反转，无异响、无堵转。

### 5.6 IMU 与 K230 空载枚举

- [ ] I²C1 总线上能扫到 MPU6050（地址 0x68 或 0x69）。
- [ ] UART3 物理回环（TX 短接 RX）能收到自发数据，无丢字符。

### 5.7 SWD / XDS

- [ ] J-Link / XDS110 能识别到 MSPM0G3507 并读出器件 ID。
- [ ] UART0 通过 XDS110 能收到 `printf` 输出（波特率 115200）。

### 5.8 跌倒保护与急停

- [ ] 一键启动按键（S1）未按下时，IMU、电机均处于"未使能"状态。
- [ ] 急停（断开主控 5V 或拔 STBY 跳线）能立刻让两轮失力。

---

## 6. 后续维护规则

1. **本表是唯一真源**：任何引脚改动必须先改本表，再按引脚类型走对应路径，最后更新阶段验收文档。
   - **业务 GPIO**（§4.7 表中所有引脚）：改 [`template/hardware/bsp_gpio.h`](../../template/hardware/bsp_gpio.h) 对应 `BSP_<NAME>_PORT/PIN/IOMUX` + 必要时改 [`bsp_gpio.c`](../../template/hardware/bsp_gpio.c) init 语句。**不进 SysConfig**（理由见 §4.7 提示与 [Stage1-IMU-BT-Telemetry.md §8.5](Stage1-IMU-BT-Telemetry.md)）。
   - **Peripheral 引脚**（PWM/QEI/I2C/UART/ADC 的 ccp/sda/scl/tx/rx/adcPin 等）：改 [`EIDE/LP_MSPM0G3507.syscfg`](../../EIDE/LP_MSPM0G3507.syscfg)，触发 `syscfg.bat` 重生 `ti_msp_dl_config.{c,h}`；这条路径走 SDK `getDualBondedPadFunction`，不踩 multi-pad bug。
2. **改动需走 PR / 提交说明**：在 commit message 标 `[pin]` 标签，并在 [docs/Overview/Overview.md](../Overview/Overview.md) 引用。
3. **跳线变化记入本文第 2 节**：不要散落在驱动文件注释里。
4. **预留引脚不允许"借用"**：3.3 节预留的 PB16/PB17 在蓝牙到位前不得被其他模块占用；如确需占用，必须升级本表。
5. **fallback 必须落表**：第 4.1 节提到的 QEI / PWM 备选回退方案，一旦真的执行回退，必须把本表 3.2 节的右轮 / PWMA / PWMB 行同步改写。

---

## 7. 修订历史

| 日期 | 版本 | 修订内容 | 作者 |
|------|------|----------|------|
| 2026-04-29 | v0.1 | 阶段 0 初版，确立全部业务引脚、跳线决策与上电验证清单 | 主控团队 |
| 2026-04-29 | v0.2 | 经 SDK 源码核对，MSPM0G3507 仅 TIMG8 支持 SysConfig 硬件 QEI；左轮 QEI 由 TIMA1 改为 TIMG8（J12 引脚不变），右轮改用 GPIO 双边沿中断软件 X4 解码（PA12/PA13），阶段 2 评估升级为 CAPTURE | 主控团队 |
| 2026-04-29 | v0.3 | 经 SDK 例程核对，PB6/PB7 在 LQFP-64 是 UART1（不是 UART3）；K230 通讯外设由 UART3 改为 UART1，引脚不变 | 主控团队 |
| 2026-04-29 | v0.4 | 阶段 1 实例化 UART2 蓝牙串口（HC-04，PB17/PB16，115200 8N1，RX 中断），PB16/PB17 由「预留」迁入「业务模块」，详见 [Stage1-IMU-BT-Telemetry.md](Stage1-IMU-BT-Telemetry.md) | 主控团队 |
| 2026-04-29 | v0.5 | **撤销 v0.4** 的 UART2/PB17/PB16 决定（命中 SDK 2.10.00.04 multi-pad 引脚 codegen bug），改用 **UART3 / PB12 (TX) / PB13 (RX)**（PINCM29/PINCM30 单 pad），新增 §4.5 蓝牙串口外设详表，K230 §4.4 → §4.4，ADC §4.5 → §4.6，GPIO §4.6 → §4.7 | 主控团队 |
| 2026-04-30 | v0.6 | 第五轮编译彻底确认 SDK 2.10 GPIO module 在所有 multi-pad 引脚上 codegen 不可用（无 syntax workaround；详见 [Stage1-IMU-BT-Telemetry.md §8.5](Stage1-IMU-BT-Telemetry.md) 根因复盘）。**14 个业务 GPIO 由 [`template/hardware/bsp_gpio.{h,c}`](../../template/hardware/bsp_gpio.h) 接管**，syscfg 不再 addInstance() GPIO 模块。§4.7 表头补充 `BSP_<NAME>_*` 宏前缀列与中断说明（阶段 1 输入引脚均不开 NVIC，留给阶段 2）；§6 维护规则把"修引脚"流程拆成"业务 GPIO 走 BSP / Peripheral 引脚走 SysConfig"两条独立路径 | 主控团队 |
