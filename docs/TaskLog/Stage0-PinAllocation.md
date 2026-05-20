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
| **IMU 接口** | **Stage 1.5 起改用 UART**；**Stage 1.6 起占用 UART3 (PB12 TX / PB13 RX, 115200 8N1, FIFO + RX 中断)** 主动上报姿态帧 | 占用 UART3 + PB12/PB13；释放 I2C1 + PB2/PB3/PB4 + UART2 + PA21/PA22。引脚集中化重排原因详见 [Stage1.5-IMU-Swap-MS901M.md §11](Stage1.5-IMU-Swap-MS901M.md) |
| 蜂鸣器类型 | 有源（GPIO 高低电平直驱） | 不占 PWM 通道 |
| 蓝牙串口 | **Stage 1.6 起整体下线** | 原占 UART3 (PB12/PB13)，引脚让给 IMU MS901M。无线遥测/可视化路径暂停（VOFA+ 二进制流），姿态数据改走 1 Hz XDS-UART (UART0) printf 文本日志；后续无线路径回归方案见 [Stage1.5-IMU-Swap-MS901M.md §11](Stage1.5-IMU-Swap-MS901M.md) |
| 编码器 Z 相 | **不接（2 Pin Mode）**——GB370 编码器无 Z 相，PB14 进入预留池 | Stage 2.3 起取消 IDX；不影响 X4 解码精度 |
| **左/右编码器解码方式** | **左 = 硬件 QEI（TIMG8 + PB15/PB16，2-Pin Mode，Stage 2.3 起从 J12 迁到 BP）；右 = GPIO 双边沿中断（Stage 2.2 起升 X4 解码，PA12/PA13 都开中断）** | MSPM0G3507 仅 TIMG8 支持 SysConfig QEI 模块 |
| TB6612 控制方式 | 2 PWM + 4 方向 + 1 STBY | 共 7 根线 |
| TB6612 PWM 频率 | 20 kHz（建议 15~25 kHz） | 避开人耳与 IMU 通带 |
| K230 串口波特率 | 115200 8N1，**仅 RX DMA**（Stage 4 起 TX DMA 移除，MCU→K230 阻塞写） | 占 UART1（PB6/PB7）+ 1 个 DMA 通道 |
| XDS-UART0 | 保留为开发期日志 | J21/J22 保持 ON |

> **架构变更说明**：原计划"双路硬件 QEI"经 SDK 源码核对（[QEIMSPM0.syscfg.js](file:///A:/Program%20Files/ti/mspm0_sdk_2_10_00_04/source/ti/driverlib/.meta/qei/QEIMSPM0.syscfg.js) 第 175 行 `TIMG(8|9|10|11)` 过滤器 + [Common.js](file:///A:/Program%20Files/ti/mspm0_sdk_2_10_00_04/source/ti/driverlib/.meta/Common.js) 第 1774 行 `getTimerInstances("QEI")`）确认 **MSPM0G3507 上只有 TIMG8 支持硬件 QEI**（TIMG9/10/11 该器件不存在）。因此：
> - **左编码器**走硬件 QEI（TIMG8）。**Stage 2.3 起引脚由 J12 (PA29/PA30/PB14) 迁到 BoosterPack (PB15/PB16)，模式由 3-Pin 降为 2-Pin**（GB370 无 Z 相）；硬件 X4 精度 1320 cnt/rev 不变，无丢脉冲；数据手册 PINMUX 表证：`PB15 = TIMG8_C0 [func 5, PINCM32]`、`PB16 = TIMG8_C1 [func 5, PINCM33]`，二者均在 BP J4.34 / J4.40 上空闲。
> - **右编码器**Stage 2.2 起升级为 X4 解码（PA12/PA13 都开 GPIO 双沿中断，1320 cnt/rev，与左轮一致）；硬件 CAPTURE 模式仍预留为高速失稳时的回退选项。

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
| J12  | PA29 / PA30 / PB14 → QEI 接口 | 不适用 | **OFF / 不再使用**（v0.9） | Stage 2.3 起左编码器迁到 BP 上的 PB15/PB16，J12 排针出厂未焊不再需要使用 |
| J13  | 模拟域 3V3 → 热敏 / OPA2365 | ON | OFF（可选） | 不使用片上 OPA，可断电避免漏流 |
| J14  | PB23 / PA9 → BP J1.3 | (1)-(2) PB23 | **(2)-(3) PA9** | Stage 1.6 起把 PWMB (PA9) 接到 BoosterPack J1.3，避免 PA9 焊接 |
| J15  | PA16 → BP J3.29 | (1)-(2) PA16 | 保留默认 | 与 4.2 节 AIN2 复用，BoosterPack 排针上不要再连其他设备 |
| J16  | PA22 → 光传感器 OPA0_OUT | ON | **OFF** | PA22 在 Stage 1.5 给 IMU 用；Stage 1.6 起 IMU 迁出，PA22 释放且不再连光传感器 |
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
> 方向：IN（输入）/ OUT（输出）/ I/O（双向，仅 SWDIO；Stage 1.5 起 I²C SDA 已下线）。
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
| ENC_L_A        | PB15 | 32         | IN   | **TIMG8_CCP0** (QEI PHA, mux f=5, PINCM32) | Stage 2.3 起从 J12/PA29 迁来；BP **J4.34** 直接接出 |
| ENC_L_B        | PB16 | 33         | IN   | **TIMG8_CCP1** (QEI PHB, mux f=5, PINCM33) | Stage 2.3 起从 J12/PA30 迁来；BP **J4.40** 直接接出 |
| ENC_R_A        | PA12 | 5          | IN   | **GPIO + 双边沿中断**         | 右轮，软件 X4 解码（阶段 2 拟升级 CAPTURE） |
| ENC_R_B        | PA13 | 6          | IN   | **GPIO（无中断）**            | 右轮 B 相，仅在 ENC_R_A 中断 ISR 内读电平判方向 |
| PWMA           | PA8  | 54         | OUT  | TIMA0_CCP0                   | TB6612 左电机 PWM，20 kHz |
| PWMB           | PA9  | 55         | OUT  | TIMA0_CCP1                   | TB6612 右电机 PWM，20 kHz |
| AIN1           | PA15 | 8          | OUT  | GPIO                         | TB6612 左电机方向 1 |
| AIN2           | PA16 | 9          | OUT  | GPIO                         | TB6612 左电机方向 2，BP J15 默认到此脚需断开 |
| BIN1           | PA26 | 30         | OUT  | GPIO                         | TB6612 右电机方向 1，J18 OFF |
| BIN2           | PA27 | 31         | OUT  | GPIO                         | TB6612 右电机方向 2，J17 OFF |
| STBY           | PB0  | 47         | OUT  | GPIO                         | 上电默认低，初始化完成后拉高 |
| IMU_TX         | PB12 | 64         | OUT  | **UART3_TX**                 | ATK-MS901M（115200 8N1，单 pad PINCM29）；备用配置 / 校准命令；Stage 1.6 由 PA21 迁来，BP J4.32 直接接出 |
| IMU_RX         | PB13 | 1          | IN   | **UART3_RX**                 | ATK-MS901M 主动上报，FIFO + RX 中断（PINCM30，单 pad）；Stage 1.6 由 PA22 迁来，BP J2.26 直接接出 |
| K230_TX        | PB6  | 58         | OUT  | **UART1_TX**                 | 阻塞写（Stage 4 起 TX DMA 移除） |
| K230_RX        | PB7  | 59         | IN   | **UART1_RX**                 | DMA RX 通道 |
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
| —           | PA21 | 17         | UART2_TX 候选 / VREF- 模式 | Stage 1.6 起释放（IMU 已迁到 UART3）；本工程内部 VREF，可作扩展 GPIO，但需焊接（不在 BoosterPack 上）|
| —           | PA22 | 18         | UART2_RX 候选 / ADC0_7      | Stage 1.6 起释放（IMU 已迁到 UART3）；BP J3.14 开放，可作扩展 |
| —           | PA29 | 36         | TIMG8_CCP0 候选 / J12 编码器 | Stage 2.3 起释放（左编码器 PHA 迁到 PB15）；J12 排针未焊，扩展需焊接 |
| —           | PA30 | 37         | TIMG8_CCP1 候选 / J12 编码器 | Stage 2.3 起释放（左编码器 PHB 迁到 PB16）；J12 排针未焊，扩展需焊接 |
| —           | PB14 | 2          | TIMG8_IDX 候选 / J12 编码器  | Stage 2.3 起释放（GB370 无 Z 相，QEI 已降为 2-Pin Mode）；J12 排针未焊 |
| —           | UART2 整体 | — | 备用 UART 实例 | 若回归且坚持单 pad，可走 PA21/PA22（PA21 焊接）；**PB15/PB16 已被 Stage 2.3 占用为 TIMG8 QEI**，不再作 UART2 备选 |

### 3.4 禁用（硬件不可作通用 IO）

| 引脚 | 原因 |
|------|------|
| NRST (38) | 复位 |
| VDD (40) / VSS (41) / VCORE (32) | 电源 |
| PA2 / ROSC (42) | 内部 ROSC |
| PA3 / LFXIN (43) / PA4 / LFXOUT (44) | LFXT 晶振接口 |
| PA5 / HFXIN (45) / PA6 / HFXOUT (46) | HFXT 晶振接口 |
| PA23 / VREF+ (24) | ADC VREF+（仅外部 VREF 模式占用；本工程内部 VREF 故 PA23 实际可作扩展，但暂未启用） |

> **PA21 / PA22 状态变更（Stage 1.5 → Stage 1.6）**：
> - **Stage 1.5**：PA21（LQFP pin 17，原 VREF-）+ PA22（LQFP pin 18，原 J16 光传感器 OUT）首次启用为 `IMU_TX (UART2_TX) / IMU_RX (UART2_RX)`，单 pad（PINCM46/47），规避 multi-pad codegen bug。
> - **Stage 1.6（2026-05-08）**：因 PA21 不在 BoosterPack 排针上，需要焊到板底 J23-J28 才能接出 → IMU UART 整体迁到 UART3 + PB12/PB13（BP 直接接出）。PA21/PA22 重新进入预留 §3.3，**未来扩展 GPIO 仍需考虑 PA21 焊接代价**。

> **必须焊接清单（Stage 1.6 起，板载固有限制）**：
>
> | 信号 | 引脚 | LQFP | 唯一接出方式 |
> |------|------|------|-------------|
> | BUZZER | PA0 | 33 | 板载 J4 跳线柱 PA0 端飞线 / 板底 J23-J28（J4 OFF + J19 OFF）|
> | LASER_EN | PA1 | 34 | 板载 J20 跳线柱 PA1 端飞线 / 板底 J23-J28（J20 OFF）|
>
> 这两脚 LaunchPad 只引到板载 LED1 跳线柱与开漏上拉跳线柱，**没有引到 BoosterPack 排针**。其余所有业务引脚（PWMA/PWMB、TB6612 方向、IMU、K230、ADC、LOG、START_BTN、左/右编码器）Stage 1.6 / Stage 2.3 重排后**全部从 BoosterPack 排针直接接出，无焊接需求；J12 编码器接头出厂未焊但已不再使用**。

---

## 4. 按外设分组详表

### 4.1 编码器解码（左：硬件 QEI / 2-Pin Mode；右：GPIO 中断软件 X4）

| 项目 | 左轮（硬件 QEI） | 右轮（软件 X4 ） |
|------|------------------|-------------------|
| 实现方式 | TIMG8 QEI（**2 Pin Mode**，Stage 2.3 起） | GPIO 双边沿中断（**PA12 + PA13 都开沿中断**，Stage 2.2 起 X4） |
| 定时器 / 资源 | TIMG8（PD0 域，BUSCLK = ULPCLK） | 不占定时器，占 2 个 GPIOA EXTI 通道（PINCM12 / PINCM13） |
| PHA | **PB15 = TIMG8_CCP0**（mux f=5, PINCM32, BP **J4.34**） | PA12 GPIO IN，RISE_FALL 中断 |
| PHB | **PB16 = TIMG8_CCP1**（mux f=5, PINCM33, BP **J4.40**） | PA13 GPIO IN，**RISE_FALL 中断**（X4，X2 兼容回退把 PA13 中断关掉即可） |
| INDEX | **不接（GB370 编码器无 Z 相）** | 无 |
| 计数 | 硬件 X4，LOAD = 0xFFFF，硬件 16-bit 累加，BSP 软扩 32-bit | 软件 X4，32-bit `volatile int32_t enc_r_count` |
| 取数 | 100~1000 Hz 周期读取并差分 | 同左 |
| 抗丢脉冲 | 硬件保证；MSPM0G3507 数字滤波 ≥ 4 BUSCLK | 受 ISR 抖动影响；6.5 kHz 内可用，高速失稳时升 CAPTURE |

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
| 死区 | 实测电机静摩擦死区约 `50‰`：`±40‰` 不转、`±60‰` 起转；由 `BSP_MOTOR_DEADZONE_COMP_PM` 在 BSP 层补偿 |
| 真值表 | AIN1/AIN2 = 10 → 正转；01 → 反转；11 → 短路刹车；00 → 滑行 |

### 4.3 IMU ATK-MS901M（Stage 1.5 替代原 MPU6050；Stage 1.6 引脚集中化重排）

| 项目 | 配置 |
|------|------|
| 元件 | 正点原子 ATK-MS901M（板载 9 轴 + 气压，内部 15 阶 EKF） |
| 接口 | **UART3**（Stage 1.6 起；Stage 1.5 曾用 UART2），主动上报（MS901M → 主控），单向流为主 |
| 引脚 | TX = **PB12**（PINCM29，主控 → 模块，发配置/校准命令；BP J4.32 直接接出）；RX = **PB13**（PINCM30，模块 → 主控，业务接收；BP J2.26 直接接出） |
| 波特率 | 115200 8N1（出厂默认；上位机可改 230400/460800，需主控同步） |
| FIFO / 中断 | UART3 FIFO 启用 + RX 半满中断；不开 DMA（吞吐 < 20 kB/s，无需） |
| 帧格式 | `0x55 0x55 <ID> <LEN> <DATA[LEN]> <CHECKSUM>`，`CHECKSUM = sum(除最后字节) & 0xFF` |
| 帧组（默认上报） | 0x01 姿态 (RPY) / 0x02 四元数 / 0x03 raw gyro+accel / 0x04 mag+temp / 0x05 baro+alt |
| 量程 | ±4 g / ±2000 dps（与上位机默认一致；与 [ms901m.h](../../template/middle/ms901m.h) 中 `ms901m_init(4, 2000)` 强绑定） |
| 主控用法 | UART RX 中断 → 256 B 环缓 → 主循环 1 kHz drain → `ms901m_feed_bytes` 状态机 → `ms901m_get_snapshot`；解析层保留 0x01 姿态帧（板载 EKF 输出），装车平衡层再做轻量一阶低通 |
| 期望频率 | MS901M 默认 200 Hz 主动上报；主控 1 kHz drain 足够覆盖 |
| 上电检测 | 上电后 3000 ms 内若仍未收到 0x01 → 视为 IMU 未在线，主控进入 fatal handler（LED_R 常亮 + 蜂鸣 200 ms + 死循环）；进入装车模式后另有 2500 ms 姿态静默窗口 |
| 上拉电阻 | UART 不需要外部上拉电阻（区别于原 I²C 方案）——这是切换到 MS901M 的核心动机 |
| 接线（装车）| MS901M VCC → 主控 5V；MS901M GND → 共地；MS901M TX → 主控 PB13（J2.26）；MS901M RX → 主控 PB12（J4.32）；交叉接（模块 TX 接主控 RX）|

> **元件替换原因**：开发板 PB2/PB3 未板载 4.7 kΩ I²C 上拉电阻、I2C1 总线全开路；为避免热风焊台修复风险，改用串口推送的 MS901M。详见 [Stage1.5-IMU-Swap-MS901M.md](Stage1.5-IMU-Swap-MS901M.md)。
>
> **Stage 1.6 重排原因**：原 Stage 1.5 使用 UART2 + PA21/PA22，但 PA21（LQFP pin 17）不在 LaunchPad BoosterPack 排针上（仅板底 J23-J28 引脚扩展接头可达），需要焊接才能接出；同期蓝牙模块下线，UART3 + PB12/PB13 释放给 IMU，二者均为 BoosterPack 开放排针 + 单 pad，无 SDK codegen 风险。详见 [Stage1.5-IMU-Swap-MS901M.md §11](Stage1.5-IMU-Swap-MS901M.md)。

### 4.4 K230 通讯 UART（Stage 4 IMU TX 一分二方案）

| 项目 | 配置 |
|------|------|
| 外设 | **UART1**（不是 UART3，UART3 让给 IMU MS901M） |
| 引脚 | TX = PB6，RX = PB7 |
| 电平 | 3.3 V（K230 GPIO 也是 3.3 V，可直连，注意共地） |
| 波特率 | 115200 8N1 |
| 流控 | 无（不接 RTS/CTS） |
| DMA | **仅 RX 占 1 个 DMA 通道**（Stage 4 起移除 TX DMA；MCU→K230 改为阻塞写，~240 B/s 占用率 < 0.03%） |
| 帧格式 | `0xAA 0x55 \| LEN \| CMD \| PAYLOAD \| CRC16 \| 0x55 0xAA` |
| MCU→K230 帧 | `VEHICLE_STATUS` (0x01, 20 Hz, 速度/状态/电压) + `HEARTBEAT_MCU` (0x02, 1 Hz) |
| K230→MCU 帧 | `MOTION_CMD` (0x11, 20~50 Hz, v/ω/模式) + `HEARTBEAT_K230` (0x12, 1 Hz) + `PID_INJECT` (0x13, 按需) |
| 心跳超时 | 500 ms 无 K230 帧 → 主控运动指令归零 + 原地平衡 |
| IMU 数据 | **不再通过 UART1 转发**：MS901M TX 线 Y 分至 K230 独立 UART RX (115200)，K230 直接解析 200 Hz 6 轴数据 |

> **Stage 4 架构变更**：原计划 MCU 通过 UART1 TX DMA 高频推送 IMU 遥测（pitch/车速/状态），K230 端被动接收。Stage 4 改为"IMU TX 一分二"方案——MS901M TX 硬件 Y 分线同时连接 MCU PB13 和 K230 独立 UART RX，K230 直接获得 200 Hz 原始 6 轴数据。主控 UART1 TX 仅需发送速度反馈 + 状态 + 心跳等低频帧，阻塞写即可，释放一路 DMA 通道。
>
> **外设选择依据**：MSPM0G3507 LQFP-64 上 PB6/PB7 是 **UART1** 的 TX/RX 引脚（PINCM23/PINCM24，参 [mspm0g350x.h:707/718](file:///A:/Program%20Files/ti/mspm0_sdk_2_10_00_04/source/ti/devices/msp/m0p/mspm0g350x.h)）。

### 4.5 蓝牙串口 UART（**Stage 1.6 起整体下线**）

| 项目 | 配置（已下线，仅作历史参考） |
|------|------|
| 外设 | ~~UART3~~ → 引脚 PB12/PB13 已交还给 IMU MS901M |
| 模块 | ~~HC-04~~（已不焊接到主板）|
| 下线原因 | Stage 1.6 引脚集中化重排：IMU 原 UART2/PA21/PA22 因 PA21 不在 BoosterPack 上需要焊接，迁到 UART3/PB12/PB13 后蓝牙必须让位；选择"取消蓝牙"而非"赌 PB15/PB16 multi-pad bug 不复发"是为保留 SDK 可用性 |
| 替代方案 | 调试期：1 Hz XDS-UART (UART0) printf 文本日志（已实现）；运行期可视化：暂无；未来无线遥测可走 K230 BLE 透传或 USB CDC，不再占主控 UART |
| 历史踩坑 | 原计划 UART2 + PB17/PB16（PINCM43/PINCM33）命中 SDK 2.10 multi-pad codegen bug，详见 [Stage1-IMU-BT-Telemetry.md §8](Stage1-IMU-BT-Telemetry.md) |

> **如未来需要恢复无线遥测**：优先评估 K230 端 BLE 模块（K230 已通过 UART1/PB6/PB7 与主控通讯，加 BLE 模组即可在 K230 端转发 VOFA 流），或 USB Type-C HID/CDC（MSPM0G3507 自带 USB FS 控制器但本工程未启用）。**不推荐**重新占用主控 UART：UART2 单 pad 选项 PA21 仍需焊接，PB15/PB16 已在 Stage 2.3 被左编码器 QEI 占用。

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
| ENC_R_A | PA12 | IN | — | 双边沿（**阶段 1 不开 NVIC**） | `BSP_ENC_R_A_*` | 右编码器 A 相，阶段 2 拟开中断或升 CAPTURE |
| ENC_R_B | PA13 | IN | — | 无中断 | `BSP_ENC_R_B_*` | 右编码器 B 相，ISR 内读电平判方向 |

---

## 5. 上电空载验证清单（阶段 0 第 3 步）

> 装车后**首次上电之前**，逐项打钩。任一项未通过禁止接 MCU 电源。

### 5.1 电源链路独立性

- [ ] 主控 3V3 与 5V 由 LaunchPad XDS110 / 外部 USB 供电，与电机驱动 VM、舵机 VS **物理隔离**（共地、非共电源）。
- [ ] TB6612 VM（电机电源，6~13.5 V）单独经过电池接入，VCC（5V 数字电源）由 MCU 域 5V 提供。
- [ ] 激光器（≤ 5 mW，405 nm）由独立电池供电，使能 GPIO 走光耦或低边 NMOS 隔离，不直接共主控 3V3。
- [ ] K230 由其自身电源供电，与主控仅在 **UART1**（PB6/PB7）+ GND 上共地。

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
- [ ] **左编码器 A/B 接 BoosterPack J4.34 (PB15) / J4.40 (PB16)**（Stage 2.3 起）；J12 排针无需焊接、无需短接。
- [ ] 万用表蜂鸣档点测：BP J4.34 ↔ 左编码器 A、BP J4.40 ↔ 左编码器 B；确认 J12 三脚（PA29/PA30/PB14）保持悬空进入预留池。

### 5.5 电机驱动空载

- [ ] STBY 强制拉低时，VM 接通后电机不转，TB6612 静态电流 < 5 mA。
- [ ] STBY 拉高、PWM = 0 时，电机仍不转，TB6612 静态电流 < 10 mA。
- [ ] 缓慢加 PWM（10% 起步），左右轮均能正反转，无异响、无堵转。

### 5.6 IMU 与 K230 空载枚举

- [ ] **MS901M（Stage 1.6）**：上电后用 USB-TTL 监听 PB13 (UART3_RX, BP J2.26) 应在 100~200 ms 内看到 `0x55 0x55 ...` 周期数据流；XDS-UART 1 Hz 心跳日志 `ms901m_good` 字段持续递增、`bad` 字段保持 0。
- [ ] UART1 物理回环（K230 端 TX 短接 RX，或 PB6 短接 PB7）能收到自发数据，无丢字符。

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
   - **Peripheral 引脚**（PWM/QEI/UART/ADC 的 ccp/tx/rx/adcPin 等；Stage 1.5 起本工程不再使用 I²C）：改 [`EIDE/LP_MSPM0G3507.syscfg`](../../EIDE/LP_MSPM0G3507.syscfg)，触发 `syscfg.bat` 重生 `ti_msp_dl_config.{c,h}`；这条路径走 SDK `getDualBondedPadFunction`，不踩 multi-pad bug。
2. **改动需走 PR / 提交说明**：在 commit message 标 `[pin]` 标签，并在 [docs/Overview/Overview.md](../Overview/Overview.md) 引用。
3. **跳线变化记入本文第 2 节**：不要散落在驱动文件注释里。
4. **预留引脚不允许"借用"**：3.3 节预留的所有引脚在新增模块到位前不得被其他模块占用；如确需占用，必须升级本表。
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
| 2026-05-07 | v0.7 | **元件替换：MPU6050 → ATK-MS901M（Stage 1.5）**。开发板 PB2/PB3 未板载 4.7 kΩ I²C 上拉电阻、I2C1 总线全开路；为避免热风焊台修复风险，IMU 链路由 I²C 切换为 UART2 + ATK-MS901M（板载 EKF，主动按帧上报）。引脚 diff：① 释放 `PB2 (IMU_SCL) / PB3 (IMU_SDA) / PB4 (IMU_INT)` 三脚 + I2C1 实例；② 占用 `PA21 (UART2_TX, PINCM46) / PA22 (UART2_RX, PINCM47)`，二者均为单 pad；③ §3.4 禁用表把 PA21 移出（仅外部 VREF 模式占用，本工程内部 VREF），PA23 保留并备注"内部 VREF 实际可作扩展但未启用"；④ §4.3 IMU 详表整段重写为 MS901M（帧格式、量程 ±4 g/±2000 dps、上电 500 ms 检测、115200 8N1、不需上拉）；⑤ §4.7 GPIO 表删 IMU_INT 行（PB4 不再使用）；⑥ §5.6 上电验证清单中"I²C 扫地址 0x68"改为"USB-TTL 监听 PA22 应见 0x55 0x55 周期帧 + 1 Hz 日志 ms901m_good 递增"。详见 [Stage1.5-IMU-Swap-MS901M.md](Stage1.5-IMU-Swap-MS901M.md) | 主控团队 |
| 2026-05-08 | v0.8 | **引脚集中化重排（Stage 1.6）+ 蓝牙整体下线**。复核 LaunchPad User's Guide 图 2-10 BoosterPack 引脚布局后，发现 v0.7 中 `IMU_TX = PA21` 不在 BP 排针上（仅板底 J23-J28 引脚扩展接头可达，需焊接），同时 `PWMB = PA9` 默认通过 J14 接 PB23 也不开放。引脚 diff：① IMU UART 整体由 UART2 迁到 UART3：`IMU_TX PA21 → PB12（BP J4.32，PINCM29 单 pad）`、`IMU_RX PA22 → PB13（BP J2.26，PINCM30 单 pad）`；② 蓝牙 HC-04 模块整体下线，UART_BT 实例从 syscfg 删除，`bsp_bt_uart.{c,h}` git rm，VOFA+ JustFloat 100 Hz 推送暂停（vofa.{c,h} 接口保留待无线路径回归后复用）；③ §1 决策行同步更新（IMU 接口/蓝牙串口）；④ §2 跳线表 J14 由 (1)-(2) PB23 改为 (2)-(3) PA9，让 PWMB 从 BP J1.3 直接接出（避免 PA9 焊接）；J16 备注更新（PA22 释放）；⑤ §3.2 业务表 IMU_TX/IMU_RX 引脚改 PB12/PB13；删 BT_TX/BT_RX 两行；⑥ §3.3 预留表加 PA21/PA22/UART2 备注；⑦ §3.4 禁用表追加"必须焊接清单"小节，明确 PA0(BUZZER)/PA1(LASER_EN) 是 LaunchPad 板载固有限制无法绕过，其余业务引脚 1.6 起全部从 BP/J12 直接接出；⑧ §4.3 IMU 详表更新外设/引脚/装车接线；⑨ §4.5 蓝牙详表整段改写为"已下线"+ 替代方案 + 历史踩坑保留；⑩ §5.6 上电验证清单中"USB-TTL 监听 PA22"改为"USB-TTL 监听 PB13"。详见 [Stage1.5-IMU-Swap-MS901M.md §11](Stage1.5-IMU-Swap-MS901M.md) | 主控团队 |
| 2026-05-17 | v0.10 | **Stage 4 K230 通讯架构变更（IMU TX 一分二方案）**。①决策表：K230 DMA 由"TX/RX 双向"改为"仅 RX"；② §3.2 K230_TX 备注由"DMA TX 通道"改为"阻塞写"；③ §4.4 K230 详表整段改写：TX DMA 移除、帧类型定义（VEHICLE_STATUS / HEARTBEAT_MCU / MOTION_CMD / HEARTBEAT_K230 / PID_INJECT）、心跳超时 500 ms、IMU 数据不走帧协议；④ §5.1 修正笔误 UART3→UART1。详见 [Stage4-K230-Communication.md](Stage4-K230-Communication.md) | 主控团队 |
| 2026-05-09 | v0.9 | **左编码器从 J12 迁到 BoosterPack（Stage 2.3）**。原方案 PHA/PHB/IDX = PA29/PA30/PB14 三脚全走 LaunchPad J12 QEI 接头，但板上 J12 排针**出厂未焊接**，无法直接接线。复核数据手册 PINMUX 表后发现 `PB15 = TIMG8_C0 [func 5, PINCM32]` / `PB16 = TIMG8_C1 [func 5, PINCM33]` 在 BP J4.34 / J4.40 上空闲（蓝牙 1.6 下线后释放进预留池），同时 GB370 编码器无 Z 相，IDX 可省，QEI 由 3-Pin Mode 降为 2-Pin Mode，X4 解码精度（1320 cnt/rev）不变。引脚 diff：① §1 决策行：解码方式行追加 "左 = TIMG8 + PB15/PB16，2-Pin Mode（Stage 2.3 起从 J12 迁到 BP）"；右轮注明 Stage 2.2 起升 X4；编码器 Z 相行整段改写"不接（2-Pin Mode），PB14 进入预留池"；② §2 跳线表：J12 由 "**必须 ON**" 改为 "**OFF / 不再使用**"；③ §3.2 业务表：删 ENC_L_IDX 行；ENC_L_A 改 `PA29/J12 ON` → `PB15/BP J4.34`；ENC_L_B 改 `PA30/J12 ON` → `PB16/BP J4.40`；④ §3.3 预留表新增 PA29/PA30/PB14 三行（从 J12 释放）；UART2 备选行去除 PB15/PB16 候选；⑤ §4.1 编码器表标题加 "/ 2-Pin Mode"；表内"实现方式 / 资源 / PHA / PHB / INDEX / 抗丢脉冲" 全部按 BP+2-Pin+X4 改写；⑥ §5.6 上电验证清单：J12 三线点测项改为 "万用表蜂鸣档点测 BP J4.34↔编码器 A、BP J4.40↔编码器 B"，并提示 J12 三脚保持悬空；⑦ §6 §3.3 引文 "PB16/PB17" 改为通用 "所有预留引脚"。SysConfig：`QEI_LEFT.enableIndexInput = false`，`peripheral.ccp0Pin.$assign = "PB15"`，`peripheral.ccp1Pin.$assign = "PB16"`，删除 `idxPin.$assign`；`ti_msp_dl_config.{c,h}` 由 EIDE build 自动重生（手工同步等价输出已落盘）。详见 [Stage2-MotorDrive-Encoder.md §7.2](Stage2-MotorDrive-Encoder.md) | 主控团队 |
