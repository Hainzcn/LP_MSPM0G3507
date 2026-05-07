# 阶段 1 ｜ 蓝牙串口配置 + MPU6050 姿态转发

> 文档定位：阶段 1 第一刀的实现总结与验收凭证。本轮交付 ① 蓝牙 UART2 实例化 ② MPU6050 驱动 + 互补滤波 ③ VOFA+ JustFloat 蓝牙转发 ④ K230 UART DMA RX 接收骨架。
>
> K230 帧协议解析、IMU_TELEM/MOTION_CMD/HEARTBEAT/ERROR、心跳超时降级 留到下一轮。
>
> 关联文档：
>
> - 项目总览：[../Overview/Overview.md](../Overview/Overview.md)
> - 阶段 0 引脚分配：[Stage0-PinAllocation.md](Stage0-PinAllocation.md)（已升级到 v0.4）
> - SysConfig 真源：[../../EIDE/LP_MSPM0G3507.syscfg](../../EIDE/LP_MSPM0G3507.syscfg)
> - EIDE 工程清单：[../../EIDE/.eide/eide.yml](../../EIDE/.eide/eide.yml)

---

## 1. 任务回顾与决策摘要

阶段 1 在 [Overview.md:79-86](../Overview/Overview.md) 共三件事：


| #   | 阶段 1 任务                                          | 本轮交付                                        |
| --- | ------------------------------------------------ | ------------------------------------------- |
| 1   | MPU6050 驱动 + 互补滤波 + 串口波形可视化                      | **完成**（蓝牙转发 VOFA+ JustFloat）                |
| 2   | MSPM0 ↔ K230 UART 帧协议（0xAA55 / CRC16 / 必备帧 / 心跳） | **仅做硬件骨架**（DMA RX + 字节计数），帧解析下一轮            |
| 3   | K230 端串口收发回环测试                                   | 主控侧已配合：UART1 物理回环时主控能在 1 Hz 日志看到 RX 字节数线性增长 |


本轮关键决策：


| 决策项        | 结论                                                                     | 影响                                                          |
| ---------- | ---------------------------------------------------------------------- | ----------------------------------------------------------- |
| 蓝牙模块       | **HC-04**（BC417 经典蓝牙 SPP），波特率 **115200 8N1**                           | 默认 9600，**装车前**必须先用 USB-TTL AT 配置一次（见 §6.1）                 |
| 蓝牙 UART    | **UART3 (PB12 TX / PB13 RX)**，FIFO + RX 中断                             | 原计划 UART2/PB17/PB16 命中 SDK multi-pad codegen bug，详见 §8 踩坑记录 |
| 可视化协议      | **VOFA+ JustFloat 二进制**（4×float LE + 尾 `00 00 80 7F`）                  | 上位机零解析、零绘图开销；4 通道 100 Hz 占空 17 %                            |
| 4 通道映射     | `pitch_deg` / `pitch_rate_dps` / `pitch_acc_deg`（仅加速度） / `temp_c`      | `pitch_acc` 用于现场对照滤波收敛与轴向极性                                 |
| MPU6050 量程 | 加速度 ±8 g（4096 LSB/g），陀螺 ±500 dps（65.5 LSB/dps），DLPF 44 Hz，SMPLRT 1 kHz | 满足平衡车带宽（< 20 Hz 主导），抑制 PWM 20 kHz 旁瓣                        |
| MPU6050 触发 | **不用 INT (PB4)**，走 SysTick 1 kHz 软轮询                                   | INT 引脚保留，下一阶段平衡环再切；本阶段实现简单                                  |
| 互补滤波       | α = 0.98（陀螺主导）；首拍直接吃 `pitch_acc`，避免冷启动慢爬                               | 调试期可在 `att_filter.c` 顶部宏 `ATT_FILTER_ALPHA` 调               |
| 调度         | 单任务轮询 + SysTick 1 kHz 节拍标志，**不上 RTOS**                                 | 任务相位：1 kHz IMU / 100 Hz VOFA / 5 Hz LED / 1 Hz 日志           |
| K230 RX    | UART1 + DMA BLOCK 模式，512 B 缓冲，DMA 完成 IRQ 重装并累加 `total_rx`              | 仅供 1 Hz 字节计数日志；下一轮做帧解析时改半满 + 全满双 IRQ 或转 FIFO 中断             |
| 安全状态       | TB6612 STBY = 0、LASER_EN = 0 全程保持低                                     | 本阶段不动力、不亮激光，纯遥测                                             |


---

## 2. SysConfig 改动（[EIDE/LP_MSPM0G3507.syscfg](../../EIDE/LP_MSPM0G3507.syscfg)）

仅新增 UART3 实例（其它部分保持阶段 0 不变）：

```javascript
const BT_UART = UART.addInstance();
BT_UART.$name                              = "UART_BT";
BT_UART.targetBaudRate                     = 115200;
BT_UART.enableFIFO                         = true;
BT_UART.enabledInterrupts                  = ["RX"];
BT_UART.peripheral.$assign                 = "UART3";
BT_UART.peripheral.txPin.$assign           = "PB12";
BT_UART.peripheral.rxPin.$assign           = "PB13";
```

注意：

- **不要**把 enableDMA(RX/TX) 打开。蓝牙数据量小，DMA 反而徒增 ISR 复杂度且和 K230 UART 抢 DMA 通道。
- **不要**改回 UART2 + PB17/PB16，会触发 SDK multi-pad codegen bug（见 §8）。

同步更新 [Stage0-PinAllocation.md](Stage0-PinAllocation.md) v0.5：

- 第 1 节决策表：`蓝牙串口` 行写明 UART3/PB12/PB13。
- 第 3.2 节业务模块表：新增 BT_TX (PB12) / BT_RX (PB13) 两行。
- 第 3.3 节预留表：清空蓝牙行（现已业务化）。
- 第 4.5 节：新增「蓝牙串口 UART」详表，含 multi-pad bug 简注。
- 章节顺序：原 §4.5 ADC → §4.6，原 §4.6 GPIO → §4.7。
- 第 7 节修订历史：追加 v0.4 + v0.5。

---

## 3. 软件架构与文件清单

按 [eide.yml](../../EIDE/.eide/eide.yml) 已声明的虚拟文件夹分层：

```
template/
├── main.c                          # 启动 + 全局初始化序列
├── hardware/                       # BSP 层（直接和外设打交道）
│   ├── bsp_systick.{c,h}           # 1 kHz SysTick + ms 计时 + 节拍标志
│   ├── bsp_log_uart.{c,h}          # UART0 printf retarget（含 AC6 _sys_* stub）
│   ├── bsp_bt_uart.{c,h}           # UART2 阻塞 TX + IT RX（256B 环缓）
│   ├── bsp_imu_i2c.{c,h}           # I2C1 控制器读 / 写原语（带超时）
│   └── bsp_k230_uart.{c,h}         # UART1 DMA RX BLOCK 模式 + 字节计数
├── middle/                         # 算法 / 协议
│   ├── mpu6050.{c,h}               # 寄存器图 + init + burst 14B 读
│   ├── att_filter.{c,h}            # 俯仰互补滤波
│   └── vofa.{c,h}                  # JustFloat 二进制打包
└── app/
    └── app_telemetry.{c,h}         # 主循环：1 kHz / 100 Hz / 5 Hz / 1 Hz 任务
```

全部新文件已在 [eide.yml](../../EIDE/.eide/eide.yml) 的 `hardware`/`middle`/`app` 虚拟文件夹下登记，include 路径已经在阶段 0 配好（`incList` 含 `../template/hardware`、`../template/middle`、`../template/app`）。

---

## 4. MPU6050 寄存器配置详表


| 寄存器          | 地址   | 写入值                    | 含义                                               |
| ------------ | ---- | ---------------------- | ------------------------------------------------ |
| PWR_MGMT_1   | 0x6B | 0x80 → 等 100 ms → 0x01 | 软复位；解 sleep；CLKSEL = PLL with X gyro             |
| CONFIG       | 0x1A | 0x03                   | DLPF_CFG = 3 → Accel 44 Hz / Gyro 42 Hz；基准 1 kHz |
| SMPLRT_DIV   | 0x19 | 0x00                   | SMPLRT = 1 kHz / (1+0) = 1 kHz                   |
| GYRO_CONFIG  | 0x1B | 0x08                   | FS_SEL = 1 → ±500 dps，灵敏度 65.5 LSB/dps           |
| ACCEL_CONFIG | 0x1C | 0x10                   | AFS_SEL = 2 → ±8 g，灵敏度 4096 LSB/g                |
| INT_ENABLE   | 0x38 | 0x00                   | 关闭 DataReady INT（本阶段软轮询）                         |
| WHO_AM_I     | 0x75 | （读，期望 0x68）            | 设备校验                                             |


**强绑定提醒**：[mpu6050.h](../../template/middle/mpu6050.h) 中的 `MPU6050_ACCEL_LSB_PER_G` / `MPU6050_GYRO_LSB_PER_DPS` 宏与上表 GYRO/ACCEL_CONFIG 写入值耦合；任一边改了必须同步另一边，否则换算系数错位、滤波输出整体跑偏。

---

## 5. 互补滤波算法

**车体坐标系约定**（**装机前必须复核**）：

- X 轴朝车头（前进方向），Y 轴朝车体右侧，Z 轴朝车顶；
- pitch = 绕 Y 轴旋转角，车头上扬为正。

**核心公式**（[att_filter.c](../../template/middle/att_filter.c)）：

```
pitch_acc = atan2(-ax, sqrt(ay^2 + az^2)) * 180 / pi
pitch     = α * (pitch_prev + gy * dt) + (1 - α) * pitch_acc
```

**首拍策略**：第一次 update 直接 `pitch = pitch_acc`，避免陀螺积分从 0 慢慢爬过去（会有数秒级的稳定时间）。后续才进入互补混合。

**α 调参指引**：

- α = 0.98（默认）：陀螺主导 → 静态噪声小，对加速度高频干扰免疫；缺点是 IMU 长期漂移会通过陀螺积累，加速度只能慢慢拉回。
- α = 0.95：加速度比重略增 → 适合振动相对平稳的台架调试。
- α = 0.99：陀螺更强 → 适合电机已经在跑、加速度被振动严重污染的场景。
- 改 α **必须**在记录一次基线波形后再调，且只动一个参数。

**轴向极性**：若现场观察到「车头明明上扬，pitch 却变成负」，最快的修法是把 `att_filter.c` 中 `atan2f(-ax, ...)` 的 `-ax` 改为 `+ax`，**同时**把 `gy` 改为 `-gy`，二者必须一起翻，否则积分项与加速度项打架。

---

## 6. HC-04 配置 + VOFA+ 上位机接入

### 6.1 HC-04 一次性 AT 配置

> HC-04 出厂默认 9600 8N1，固件不会自动改它。装车前 **必须** 用 USB-TTL（CH340/CP2102 都行）外接 HC-04 完成下面操作，**之后蓝牙模块的设置就永久保存了**。

接线（USB-TTL ↔ HC-04）：

- TTL TXD → HC-04 RXD
- TTL RXD → HC-04 TXD
- TTL 5V  → HC-04 VCC（HC-04 板载 LDO，3.3 V/5 V 通吃）
- TTL GND → HC-04 GND
- **进入 AT 模式**：HC-04 通常在 `EN/KEY` 引脚拉高时进 AT 模式（部分批次为上电按住 `KEY` 按键）；查模块底面丝印为准。

串口调试器（115200 ❌ 用 9600 8N1，结尾 `\r\n`）依次发送：

```
AT
AT+VERSION
AT+NAME=BalanceCar
AT+PIN=1234
AT+UART=115200,0,0
```

期望回复每条都是 `OK`。最后一条把波特率写到 115200 后，**断电重启 HC-04，再用 115200 重新连接**确认 `AT` 仍回 `OK`。

完成后焊到主板：

- **HC-04 TXD → MSPM0 PB13** (UART3_RX, LQFP pin 1)
- **HC-04 RXD → MSPM0 PB12** (UART3_TX, LQFP pin 64)
- HC-04 VCC → 主板 5V
- HC-04 GND → 主板 GND（与主控共地）

### 6.2 VOFA+ 上位机接入

1. 手机或电脑（带蓝牙）→ 配对 HC-04，PIN `1234`。配对成功后操作系统会创建一个虚拟串口（Windows 下查"设备管理器 → 端口"）。
2. 打开 VOFA+，「串口监视器」→ 选择上一步生成的虚拟 COM → 波特率 `115200` → 数据格式选 **JustFloat** → 通道数 `4`。
3. 点开始，应能立即看到 4 路波形：
  - CH0 = `pitch_deg`（互补滤波后）
    - CH1 = `pitch_rate_dps`（陀螺仪 Y 轴）
    - CH2 = `pitch_acc_deg`（仅加速度的瞬时俯仰）
    - CH3 = `temp_c`（IMU 内温度，℃）
4. 通道 0 与通道 2 应该贴近重合，但 CH0 明显更"丝滑"。挥手晃动模块时 CH1 摆幅最大。

---

## 7. 验收清单

> 在硬件焊好 / 蓝牙配置过 / SDK 重新生成 `ti_msp_dl_config.{c,h}` 后逐项打钩。


| 项          | 通过条件                                                                      | 验证方式                      | 状态  |
| ---------- | ------------------------------------------------------------------------- | ------------------------- | --- |
| 工程能编译      | EIDE 构建无 error，allow warning                                              | EIDE 构建按钮                 | [ ] |
| MPU6050 在线 | 上电后串口日志不出现 `[FATAL] mpu6050_init failed`                                  | XDS-UART 看日志              | [ ] |
| 蓝牙连通       | 手机 SPP 助手连上 HC-04 后能收到 20 B/帧 × 100 Hz 数据流                                | 手机端十六进制看尾字节 `00 00 80 7F` | [ ] |
| VOFA+ 4 通道 | 4 路波形流畅、无丢帧、无错位、CH0 与 CH2 趋势一致                                            | VOFA+ JustFloat 4 通道      | [ ] |
| 静态俯仰噪声     | 模块水平静置 30 s，CH0 标准差 < 0.1°                                                | VOFA+ 录波 → 离线算 std        | [ ] |
| 动态滞后       | 手抖 ±20°，CH0 跟随 CH2 滞后 < 10 ms                                             | VOFA+ 同屏对比                | [ ] |
| 轴向极性       | 抬车头 → CH0 增大；低车头 → CH0 减小                                                 | 手动倾斜模块                    | [ ] |
| 1 Hz 日志    | XDS-UART 看到 `[hb] t=...s pitch=... rate=... acc=... T=... k230_rx=...b/s` | XDS-UART                  | [ ] |
| K230 RX 自测 | PB6 短接 PB7（或 K230 端发数据），1 Hz 日志 `k230_rx` 字段非零                            | 杜邦线短接 + 看日志               | [ ] |
| 跌倒安全       | 全程电机 STBY = 0、LASER_EN = 0（万用表测量 PB0 / PA1 = 0 V）                         | 万用表                       | [ ] |


---

## 8. 踩坑记录：UART2 + PB17/PB16 multi-pad SDK bug

### 8.1 现象

阶段 1 v0.1 把蓝牙串口配为 `UART2 / PB17 TX / PB16 RX`（直接采用阶段 0 文档中预留的引脚组）。SysConfig CLI 跑通了 validation，但在 **Generating Code** 阶段抛出：

```
TypeError: Cannot read properties of undefined (reading 'match')
    at sliceNumber               (.../Common.js:1866:21)
    at getGPIONumber             (.../Common.js:1871:9)
    at Object.getGPIONumberMultiPad (.../Common.js:1913:12)
    at printDefine               (.../uart/UART.Board.h.xdt:128:27)
```

`ti_msp_dl_config.{c,h}` 完全没有生成，整个工程链编译断在 `linking syscfg` 步骤。

### 8.2 诊断

沿堆栈翻 [Common.js](file:///A:/Program%20Files/ti/mspm0_sdk_2_10_00_04/source/ti/driverlib/.meta/Common.js)：

```javascript
function getGPIONumberMultiPad(packagePin, inst, pinInterfaceName){
    let pinNames = (system.deviceData.devicePins[packagePin]
                      .mux.muxSetting.find(item => item["mode"] === "1")
                      .peripheralPin.peripheralName).split("/");
    if(pinNames.length == 1){ return getGPIONumber(pinNames[0]) }
    if(hasPinNameInMuxMode(packagePin)){ ... }
    else{
        padIndex = identifyPadIndexUsingInst(packagePin, inst, pinInterfaceName);
    }
    return getGPIONumber(pinNames[padIndex])   // ← pinNames[padIndex] = undefined
}
```

PB16 / PB17 在 MSPM0G3507 LQFP-64(PM) 上是 **multi-pad / dual-bonded** 引脚（同一物理 ball 上绑了多个 GPIO 编号，`peripheralName` 形如 `"PB16/PA15"`），`pinNames.length > 1`。`identifyPadIndex` 通过遍历 `mux.muxSetting` 数组比对 `peripheralName + "." + pinInterfaceName`（例如 `"UART2.TX"`）寻找索引；当目标外设功能不在 SDK 的 mux 索引里出现时，返回值落到了 `pinNames` 数组之外，最终把 `undefined` 传给 `sliceNumber("DL_GPIO_PIN_" + undefined)`，崩在 `.match(/\d+$/)` 上。

可以稳定重现的最小条件：

```javascript
BT_UART.peripheral.$assign       = "UART2";
BT_UART.peripheral.txPin.$assign = "PB17";   // PINCM43 multi-pad
BT_UART.peripheral.rxPin.$assign = "PB16";   // PINCM33 multi-pad
```

### 8.3 修复

把蓝牙 UART 换到 **UART3 + PB12 (TX) / PB13 (RX)**（PINCM29 / PINCM30）。这两个 PINCM 在 [mspm0g350x.h](file:///A:/Program%20Files/ti/mspm0_sdk_2_10_00_04/source/ti/devices/msp/m0p/mspm0g350x.h) 中只挂一组 GPIO 映射（`GPIOB_DIO12` / `GPIOB_DIO13`），是单 pad 引脚，绕开 `getGPIONumberMultiPad` 的越界路径。修改后 SysConfig 一次通过。

源代码层面唯一受影响的符号是 `bsp_bt_uart.c` 的 IRQ 函数名：从 `UART2_IRQHandler` 改为 `UART3_IRQHandler`。其它地方都用 `UART_BT_INST` 宏（SysConfig 自动生成），与外设号解耦。

### 8.4 教训

1. **不要把"datasheet 上写着 X 引脚 = X 外设"等同于"SysConfig 模板能在 X 上 codegen"**：MSPM0 系列的 multi-pad 引脚组（PB14~PB19、PA0/PA15 等组合）有 codegen 死角，阶段 0 仅做 IOMUX 表纸面核对、未做端到端 codegen 验证是隐患。
2. **新增任何外设 / 引脚，先单独跑一次 SysConfig codegen 再写驱动代码**。本轮我把 SysConfig 修改与 5 个新增 .c/.h 一次性提交，导致 codegen 失败时已经有 ~600 行驱动代码无法编译验证，回退成本高。
3. 排查时发现 SDK 的 `system.deviceData.devicePins` 不能用普通 grep 直接搜（pin 名通过 ID 间接索引），但 `mspm0g350x.h` 里的 `IOMUX_PINCM<N>_PF_<FUNC>` 宏是稳定的真源——是判断"某 pin 是否硬件支持某外设"的最快路径。

### 8.5 后续防护

- 阶段 0 引脚分配文档若再有"候选 / 预留"引脚，**评审 + codegen 双确认**后才能挂到任何 syscfg。
- 每次新增外设到 SysConfig 后，单独跑 `syscfg.bat ... --output <tmpdir>`，确认 `ti_msp_dl_config.{c,h}` 生成成功且 `make` 也能链通过，再开始写驱动代码。
- 后续如果还有引脚要走 multi-pad（如某些 PWM/SPI 引脚），优先选 SDK 自带 example 已用过的组合。

### 8.2 ARMCLANG (AC6) + Keil 工具链兼容坑

首轮编译（`build` 阶段）暴露了 **3 类 AC5→AC6 / SDK 命名约定不熟悉**导致的硬错，加上 §8.3 工程结构 / SysConfig 属性两类问题，本节合并落档备查：


| #   | 错误现象                                                                                                                                                | 触发原因                                                                                                                                                                                                                            | 解决方案                                                                                                                                                          |
| --- | --------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 1   | `'#pragma import' is an Arm Compiler 5 extension, and is not supported by Arm Compiler for Embedded 6` `redefinition of '__FILE'`（`bsp_log_uart.c`） | AC5 风格的 `#pragma import(__use_no_semihosting)` + 自定义 `struct __FILE` 在 AC6 下都不合法；AC6 的 Keil `<stdio.h>` 已预定义 `struct __FILE`                                                                                                    | 改用 `__asm(".global __use_no_semihosting\n\t");`（AC6 推荐写法，注释见 `bsp_log_uart.c`），并删除自定义 `struct __FILE`，仅保留 `FILE __stdout/__stdin/__stderr` 占位实例               |
| 2   | `use of undeclared identifier 'SystemCoreClock'`（`bsp_systick.c`）                                                                                   | MSPM0G3507 (Cortex-M0+) 启动文件未实现 `SystemInit`，CMSIS 也没导出 `SystemCoreClock` 全局变量；CMSIS device 头里仅做了声明，链接期找不到符号定义                                                                                                                  | 改用 SysConfig 在 `ti_msp_dl_config.h` 自动生成的 `**CPUCLK_FREQ`** 宏（默认 SYSOSC = 32_000_000；切到 HFXT 80 MHz 时该宏会自动同步）                                                 |
| 3   | `use of undeclared identifier 'GPIO_OUT_PORT'` / `'GPIO_OUT_GPIO_LED_R_PIN'`（`main.c`、`app_telemetry.c`）                                            | 我误以为 SysConfig 会把整个 `GPIO_OUT` 实例聚合成单一 `GPIO_OUT_PORT`；实际上当一个 GPIO 实例**同时含 PA 和 PB 引脚**（我们的 GPIO_OUT 就是：BUZZER/AINx/BINx/LASER_EN 在 PA，LED_R/G/B、STBY 在 PB），SDK 模板会**逐 pin 生成 `<INSTANCE>_<NAME>_PORT`**，不会出现 `<INSTANCE>_PORT` | 把 `GPIO_OUT_PORT` 全部替换为每个 pin 自己的 `GPIO_OUT_GPIO_<NAME>_PORT`；同端口的引脚（LED_R/G/B 同属 GPIOB）仍可合并到一次 `DL_GPIO_xxxPins` 调用，**跨端口操作必须分两次调用**（如 BUZZER/PA 与 LED_R/PB） |
| 4   | `use of undeclared identifier 'NULL'`（`att_filter.c`）                                                                                               | `att_filter.c` 仅 include 了 `<math.h>` 和自家 `att_filter.h`，二者都不带 NULL 定义                                                                                                                                                          | 显式 `#include <stddef.h>`                                                                                                                                      |


**复盘要点**：

- AC5 (`#pragma import`) ↔ AC6 (`__asm(".global ...")`) 的差异在 Keil 老 wiki 上还在，迁移时**全文搜 `#pragma import`** 一次防漏。
- `SystemCoreClock` 在 G 系列上不可用；之后凡是要拿 MCLK 的地方一律走 `CPUCLK_FREQ`，**不要写 magic number 32000000**——后期改 HFXT 时这是隐藏炸点。
- SDK GPIO 实例的 `_PORT` 宏命名规则：**所有 pin 同端口** → `<INSTANCE>_PORT`；**跨端口** → `<INSTANCE>_<NAME>_PORT`。建议直接用每 pin 的 `_PORT` 形式，**不依赖聚合宏**，重构时可零成本拆分。
- 新写的每个 `.c` 上来先检查 `<stdint.h>/<stddef.h>/<string.h>/<stdio.h>` 必备四件套是否齐，编译期能省一半「未声明标识符」类错误。

### 8.3 SysConfig `assignedPort` 静默丢弃 + 工程桩文件长期未刷新

第二轮编译（在 §8.2 修完后）暴露了**两个互相纠缠**的根因问题，是 Stage 0 起就潜伏的隐患，本节单独立档：

#### 现象

```
./../template/app/app_telemetry.c:62: error: use of undeclared identifier 'GPIOC'
.\ti_msp_dl_config.h:246:75: note: expanded from macro 'GPIO_OUT_GPIO_LED_G_PORT'
  246 | #define GPIO_OUT_GPIO_LED_G_PORT  (GPIOC)
./../template/main.c:40: error: use of undeclared identifier 'GPIO_OUT_GPIO_LED_R_PORT'
```

`app_telemetry.c` 和 `main.c` 报的错完全不一样：前者**找得到**宏 `GPIO_OUT_GPIO_LED_G_PORT`、值是不存在的 `GPIOC`；后者**根本找不到**宏 `GPIO_OUT_GPIO_LED_R_PORT`。

#### 根因 1：SysConfig `assignedPort` 是非法属性、被静默丢弃

`LP_MSPM0G3507.syscfg` 自 Stage 0 起对 14 个 GPIO 引脚一直用：

```javascript
GPIO_OUT.associatedPins[7].$name             = "GPIO_LED_R";
GPIO_OUT.associatedPins[7].assignedPin       = "26";    // 仅 bit 序号
GPIO_OUT.associatedPins[7].assignedPort      = "PORTB"; // ← 不是 SysConfig 合法属性
```

SDK 2.10 的 GPIO module schema 里**只有 `assignedPin` 一个字段**（且仅当 instance 级 `port = "PORTA/B"` 时有效）；`assignedPort` 完全不识别，被静默丢弃。我们的 `GPIO_OUT` 实例又**同时含 PA 和 PB 引脚**，无法在 instance 级设 `port`，导致 SysConfig 在 `assignedPin = "26"` 时不知道选哪个端口的 26 号 bit，最终对 PB22/26/27/PA0/PA1 这类 multi-pad 引脚回退到不存在的 `GPIOC` 第二 pad（同一 PINCM 在更小封装上对应 `PC0/PC1/PC2`）。表现就是 LED_R 和 LED_G 各自被算到了 `GPIOC.1`（同一个 pin）、LED_B 算到 `GPIOC.2`，全部 GPIOC.* 全部不存在 → ARMCLANG 报 `use of undeclared identifier 'GPIOC'`。

**修法**：每个 GPIO pin 改用 SDK example 标准的**单字段直接锁定**：

```javascript
GPIO_OUT.associatedPins[7].pin.$assign       = "PB26";  // 一句话锁死，不靠端口推断
```

`pin.$assign` 直接对应物理引脚名，不会被 multi-pad 机制误判。本轮把 GPIO_OUT (10 pin) + GPIO_IN (4 pin) 共 14 处全改了。其它外设（PWM/UART/I²C/ADC）原本就是用 `peripheral.txPin.$assign / rxPin.$assign` 形式，不受影响。

#### 根因 2：`eide.yml` 把 `ti_msp_dl_config.{c,h}` 指到 `template/` 的陈旧桩

工程结构是：

- `EIDE/syscfg.bat` 读 `EIDE/LP_MSPM0G3507.syscfg` → **生成到 `EIDE/ti_msp_dl_config.{c,h}`**
- 但 `EIDE/.eide/eide.yml` 把构建源指向 `**../template/ti_msp_dl_config.{c,h}**` —— 那是**项目模板期一次性放进去的桩文件，从未被 `syscfg.bat` 更新过**

后果是非常隐蔽：

- `template/ti_msp_dl_config.h` 只定义了 `CPUCLK_FREQ`，没有任何 GPIO/UART/I²C 宏 → main.c 因为同目录优先匹配，吃到桩 → 报「`GPIO_OUT_GPIO_LED_R_PORT` 未声明」。
- `template/ti_msp_dl_config.c` 里 `SYSCFG_DL_GPIO_init/SYSCFG_DL_UART_BT_init/...` 全部空实现 → **链接成功也不会真正初始化外设**。Stage 0 之所以"看起来能跑"是因为之前测的 LED/UART0 都用了启动文件的默认状态，并未走 SysConfig 流程；这才是 Stage 0 真正的盲区。
- `app_telemetry.c` 在子目录里，include 搜索路径 `.`（= EIDE/）能命中 `EIDE/ti_msp_dl_config.h` 的真文件 → 看到 `(GPIOC)` 报错；和 main.c 的错不一样的根本原因就在这里。

**修法**：

1. 删 `template/ti_msp_dl_config.{c,h}` 桩，避免 main.c 同目录优先匹配。
2. `eide.yml` 把这两个文件改成 `path: ./ti_msp_dl_config.{c,h}`（相对项目根 EIDE/），同时把 SysConfig 真源也由旧模板路径改回 `./LP_MSPM0G3507.syscfg`（之前 Stage 0 引入的笔误，但因为 syscfg.bat 用绝对路径才一直没暴露）。

#### 复盘要点

- **每次新加 GPIO pin 一律用 `pin.$assign = "PXNN"`，不要用 `assignedPin/assignedPort` 双字段写法**；后者一旦 instance 级没设 `port` 就只能听 SysConfig 自动分配的天命。
- 项目结构里**不要并存两份会同名相互遮蔽的生成文件**。SysConfig 输出目录、eide.yml 注册路径、include 搜索路径三处必须**指向同一个 ti_msp_dl_config**。
- 任何「编译过 + 烧录看似有反应」的 Stage 0 通过条件，都不能省略**用 LED 红灯 / 串口 echo 实际验证 GPIO 已被 SysConfig 初始化**这一步——本轮如果不是要点 LED_G 心跳，桩文件可以再藏一两个月。
- 后续若再换 multi-pad 引脚（如某些 OPA / COMP / ADC 输入），先用 SDK GUI 试拖一遍生成 `pin.$assign` 字段，再 copy 进 syscfg；不要手写 bit + port。

### 8.4 SysConfig 多 pad 引脚 codegen bug ：必须用 instance 级 `port` 才能解开

§8.3 改完后再次构建，新错误冒出来：

```
./../template/app/app_telemetry.c:62:32: error: use of undeclared identifier 'GPIOC'
.\ti_msp_dl_config.h:246:75: note: expanded from macro 'GPIO_OUT_GPIO_LED_G_PORT'
  246 | #define GPIO_OUT_GPIO_LED_G_PORT  (GPIOC)
```

`Generating Code` 行还报 `**Unchanged ti_msp_dl_config.h**`——说明 `pin.$assign = "PB27"` 改完之后，SysConfig 生成的字节跟改之前**完全一样**。

#### 现象详查

把 `EIDE/ti_msp_dl_config.h` 的 GPIO 段全 dump 出来对比 SDK example，规律一目了然：


| pin.$assign  | IOMUX 宏（对）      | PORT 宏（错） | PIN 宏（错）        | 真正应该              |
| ------------ | --------------- | --------- | --------------- | ----------------- |
| PA15 (AIN1)  | `IOMUX_PINCM37` | `GPIOA` ✓ | `DL_GPIO_PIN_1` | `DL_GPIO_PIN_15`  |
| PA16 (AIN2)  | `IOMUX_PINCM38` | `GPIOA` ✓ | `DL_GPIO_PIN_1` | `DL_GPIO_PIN_16`  |
| PA26 (BIN1)  | `IOMUX_PINCM59` | `GPIOA` ✓ | `DL_GPIO_PIN_0` | `DL_GPIO_PIN_26`  |
| PA27 (BIN2)  | `IOMUX_PINCM60` | `GPIOA` ✓ | `DL_GPIO_PIN_0` | `DL_GPIO_PIN_27`  |
| PB22 (LED_B) | `IOMUX_PINCM50` | `GPIOC` ✗ | `DL_GPIO_PIN_2` | `DL_GPIO_PIN_22`  |
| PB26 (LED_R) | `IOMUX_PINCM57` | `GPIOC` ✗ | `DL_GPIO_PIN_1` | `DL_GPIO_PIN_26`  |
| PB27 (LED_G) | `IOMUX_PINCM58` | `GPIOC` ✗ | `DL_GPIO_PIN_1` | `DL_GPIO_PIN_26`  |
| PA0 (BUZZER) | `IOMUX_PINCM1`  | `GPIOA` ✓ | `DL_GPIO_PIN_0` | `DL_GPIO_PIN_0` ✓ |
| PB0 (STBY)   | `IOMUX_PINCM12` | `GPIOB` ✓ | `DL_GPIO_PIN_0` | `DL_GPIO_PIN_0` ✓ |


**所有「错」的引脚（PA15/16/26/27、PB22/26/27）共同点都是：MSPM0G350x 上的 multi-pad bonded pin**——同一 PINCM 内部还接到第二 pad（如 PB26 的第二 pad 是 PC1），SysConfig 在没有 instance 级 `port` 锁定时**默认拿第二 pad 的 (port, bit) 去填 `_PORT/_PIN` 宏**，但 `_IOMUX` 宏却拿第一 pad 的 PINCM——所以 IOMUX 是对的、PORT/PIN 全是错的，三个宏内部互相矛盾。LQFP-64-PM 封装根本没有 PC bank 引脚，宏展开 `GPIOC` 直接编译错。

#### 为什么 §8.3 改成 `pin.$assign = "PB26"` 没救

GUI 文档里 `pin.$assign` 是「明确指定物理引脚名」的官方写法，但**SDK 2.10.00.04 的 GPIO module 在生成 `_PORT/_PIN` 宏时，无论 `assignedPin` 还是 `pin.$assign` 走的都是同一个 `getGPIONumberMultiPad → identifyPadIndex` 函数链**，对 multi-pad 引脚都返回第二 pad 的索引。两种语法殊途同归地命中同一个 codegen bug。

#### 怎样的 syscfg 才会生成正确宏

参照 SDK example `examples/nortos/LP_MSPM0G3507/driverlib/gpio_toggle_output/`：

```javascript
GPIO1.$name = "GPIO_LEDS";
GPIO1.port  = "PORTB";              // ← 关键：instance 级声明 port
GPIO1.associatedPins.create(4);
GPIO1.associatedPins[0].assignedPin = "22";  // PB22 (multi-pad)
GPIO1.associatedPins[3].pin.$assign = "PB16"; // PB16 (也 multi-pad)
```

生成结果（来自 example 自带 `ti_msp_dl_config.h`）：

```c
#define GPIO_LEDS_PORT                (GPIOB)             // ← 正确
#define GPIO_LEDS_USER_LED_1_PIN      (DL_GPIO_PIN_22)    // ← 正确（PB22）
#define GPIO_LEDS_USER_TEST_PIN       (DL_GPIO_PIN_16)    // ← 正确（PB16）
```

关键差别**不止一处**：

1. **instance 级 `GPIO1.port = "PORTB"`** —— 锁定该 instance 所有 pin 都在 PORTB。
2. **每 pin 用 `assignedPin = "22"`（纯数字字符串）** —— 只给 bit 号，端口由 instance.port 推断。

第三轮我们尝试 `port = "PORTB"` + `pin.$assign = "PB26"`（带 P 前缀的全名），**仍然生成 `GPIOC.1`**！原因是 SDK 2.10 GPIO codegen 在 `pin.$assign` 路径下**会重新做一次「按全名查找物理 pad」**，这条独立查找忽略了 instance.port，重新命中多 pad bug；而 `assignedPin = "26"` 路径下，端口直接走 instance.port，不再做 pad 名解析，自然不踩 bug。

总结一句话：**instance 级 `port` + 每 pin `assignedPin = "<bit>"` 是 SDK 2.10 上唯一能正确生成多 pad 引脚宏的组合**。`pin.$assign` 字段在多 pad 引脚上不可用。

#### 修法

我们的 `GPIO_OUT/GPIO_IN` 业务上**同时含 PA + PB 两端口的引脚**，单一 instance 没法设 `port`，唯一干净办法是**按物理 port 拆成两个实例**：


| 旧实例        | 新实例          | port  | 引脚                            |
| ---------- | ------------ | ----- | ----------------------------- |
| `GPIO_OUT` | `GPIO_OUT_A` | PORTA | AIN1/2、BIN1/2、BUZZER、LASER_EN |
| `GPIO_OUT` | `GPIO_OUT_B` | PORTB | STBY、LED_R/G/B                |
| `GPIO_IN`  | `GPIO_IN_A`  | PORTA | START_BTN、ENC_R_A、ENC_R_B     |
| `GPIO_IN`  | `GPIO_IN_B`  | PORTB | IMU_INT                       |


C 端宏命名也跟着升级（保留 SDK 标准约定）：


| 旧宏（错误）                      | 新宏（正确）                         |
| --------------------------- | ------------------------------ |
| `GPIO_OUT_GPIO_LED_R_PORT`  | `GPIO_OUT_B_PORT` （instance 级） |
| `GPIO_OUT_GPIO_LED_R_PIN`   | `GPIO_OUT_B_GPIO_LED_R_PIN`    |
| `GPIO_OUT_GPIO_BUZZER_PORT` | `GPIO_OUT_A_PORT`              |
| `GPIO_OUT_GPIO_BUZZER_PIN`  | `GPIO_OUT_A_GPIO_BUZZER_PIN`   |


`main.c` 与 `app_telemetry.c` 共 4 处引用同步替换。

#### 复盘要点

- **MSPM0G350x 的 multi-pad bonded pin 列表**（实际使用 LQFP-64-PM 时）至少包括：PB16/PB17（§8.1 已记）、PA15/PA16/PA26/PA27、PB22/PB26/PB27。后续做 OPA / COMP / 高速比较器输入时若再选这些引脚，**必须**在 GPIO instance 上显式 `port = "PORTA/B"` 并配 `assignedPin = "<bit>"`。
- **任何 GPIO 实例只要混用 PORTA + PORTB 引脚**就得拆。SysConfig 不像 STM32CubeMX 那样让你"瞎填都能跑"，cross-port 时要么显式拆实例，要么 instance 级 `port` 不能设——后者必然踩多 pad bug。
- **GPIO 引脚 syscfg 写法的最终决议**（按可靠性排序）：
  1. 单 pad 引脚：`pin.$assign = "PXNN"` 或 `assignedPin = "<bit>"` + instance.port，两者都行；
  2. 多 pad 引脚：**只能**用 instance 级 `port` + `assignedPin = "<bit>"`；
  3. 单 pad **被外设占用**的引脚（如 UART TX/RX）：`peripheral.txPin.$assign = "PXNN"` 是外设模块自己的字段，不走 GPIO module 的多 pad 解析路径，直接用即可。
- **重复检查 `Unchanged ti_msp_dl_config.h`**：构建日志里出现这行不一定是好事，syscfg 改了但生成出的字节没变，等价于改动没生效。下次见到这条要立刻 diff `ti_msp_dl_config.h`，确认期望的修改的确反映到了输出里。
- 三个宏 `_PORT / _PIN / _IOMUX` 之间**应当**是自洽的（全指向同一物理引脚）。出现 `_IOMUX = PINCM57` 但 `_PORT = GPIOC, _PIN = PIN_1` 这种内部矛盾，几乎一定是 multi-pad 解析 bug，**别尝试 workaround C 代码，回去改 syscfg**。
- 还要看 **生成的 `ti_msp_dl_config.c` 里 `SYSCFG_DL_GPIO_init()` 的初始化语句**：哪怕 `_PORT/_PIN` 宏看起来对，如果 init 里出现 `DL_GPIO_clearPins(GPIOC, ...)` 这种端口字面量，依然是多 pad bug——`.h` 与 `.c` 的端口字面量是**两条独立的 codegen 路径**，可能一个对一个错。

### 8.5 终极结论：SDK 2.10 GPIO codegen 对 multi-pad 引脚根本不可用 → BSP 接管全部 GPIO

§8.3 / §8.4 用 `pin.$assign`、`assignedPin`、instance 级 `port`、拆 `GPIO_OUT_A/B` 等四种语法**全部尝试过**，结论一致：只要 GPIO 实例里包含 multi-pad 引脚（PA13/15/16/18/26/27、PB22/26/27 等高 PINCM 引脚），SDK 2.10 (sysconfig 1.27.0) 都会在 `ti_msp_dl_config.c` 的 `SYSCFG_DL_GPIO_init()` 里生成 `DL_GPIO_clearPins(GPIOC, ...)` 这类**字面**端口名，编译失败。即使 §8.4 拆完 4 个实例后，`.h` 里 `GPIO_OUT_B_PORT = (GPIOB)` 看起来对了，`.c` 里 init 函数的端口字面量仍然是 `GPIOC`。两条 codegen 路径互相独立。

#### 根因（已读 `Common.js` 实证）

走读 `A:\Program Files\ti\mspm0_sdk_2_10_00_04\source\ti\driverlib\.meta\Common.js` 与 `gpio\GPIO.Board.c.xdt`，bug 链条：

1. `GPIO.Board.c.xdt` 行 139 / 141 在生成每条 init 语句时调用：
  ```javascript
   let port  = Common.getGPIOPortMultiPad(packagePin, pinst, undefined);
   let pinID = Common.getGPIONumberMultiPad(packagePin, pinst, undefined);
  ```
   注意第三个参数 `pinInterfaceName` 传的是 `undefined`——GPIO 没有"哪一根线"的语义（这是 UART RX/TX 那种 peripheral pin 才有的）。
2. `Common.js:getGPIOPortMultiPad()` (行 2129) 对多 pad 引脚走 `else` 分支调 `identifyPadIndexUsingInst()`：
  ```javascript
   else {
       padIndex = identifyPadIndexUsingInst(packagePin, inst, pinInterfaceName);
   }
  ```
3. `identifyPadIndexUsingInst()` 对 GPIO 模块的 inst 把 `peripheralName` 留为 `undefined`，再调 `identifyPadIndex(packagePin, pinInterfaceName, undefined, gpioName)`。
4. `identifyPadIndex()` 行 2245-2247 写错了：
  ```javascript
   else if(gpioName !== undefined) {
       muxId = ((system.deviceData.devicePins[packagePin].mux.muxSetting)
                .map(a => a.peripheralPin.name)).indexOf(peripheralName + "." + pinInterfaceName);
       //                                                ^^^^^^^^^^^^^^   ^^^^^^^^^^^^^^^^
       //                                                undefined        undefined
   }
  ```
   `peripheralName + "." + pinInterfaceName` 在 JavaScript 中拼成字符串 `"undefined.undefined"`，去 `indexOf` 永远是 `-1`，于是 `muxId = -1`，最终 `padIndex = 0`（因 `-1 > modeMiddleIndex` 为 false）。
5. 又因为 `getGPIOPortMultiPad` 回到 `return getGPIOPort(pinNames[padIndex])`，而 `pinNames` 来自 `mux.muxSetting.find(item => item.mode === "1").peripheralPin.peripheralName.split("/")`——MSPM0G350x 的 device data 里**多 pad 引脚 mode "1" 的 peripheralName 顺序是 `"PCx/PBx"`（PC 在前）**，于是 `pinNames[0]` = `"PC1"` → port = `"C"` → `getGPIOPort` 返回 `"GPIOC"`。

整条 bug 的本质是：**SDK 编写者忘了 GPIO 模块没有 `peripheralName.pinInterfaceName` 这种东西**，导致对多 pad 引脚的 pad 选择函数永远返回错误的 alternate pad（PC bank），而 PC bank 在 LQFP-64-PM 上根本没引出，链接期失败。

无论 syscfg 怎么写都触发同一个 bug——bug 在 codegen 路径，不在 syntax。

#### 修法：syscfg 不再 addInstance() GPIO 模块

放弃 SysConfig 管理 GPIO，改在 BSP 层手工初始化：


| 文件                             | 改动                                                                                                                                                                                                      |
| ------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `EIDE/LP_MSPM0G3507.syscfg`    | 删除 `GPIO_OUT_A/B`、`GPIO_IN_A/B` 全部 4 个实例（保留 `GPIO` 模块导入即可，无 instance 不会触发 codegen）。peripheral pins（UART/I2C/PWM/QEI/ADC）走 `getDualBondedPadFunction` 路径，不踩 bug，全部保留。                                    |
| `template/hardware/bsp_gpio.h` | 新增。14 组 `BSP_*_PORT/PIN/IOMUX` 宏，覆盖原 syscfg 全部 GPIO（LED_R/G/B、BUZZER、LASER_EN、STBY、AIN1/2、BIN1/2、START_BTN、ENC_R_A/B、IMU_INT）。                                                                          |
| `template/hardware/bsp_gpio.c` | 新增。`bsp_gpio_init()` 用 `DL_GPIO_initDigitalOutput(IOMUX_PINCMxx)` + `DL_GPIO_PIN_<bit>` 直接初始化，与 SDK 自动生成的 `SYSCFG_DL_GPIO_init` 等价。**输入引脚只配方向 + 上拉，不开 NVIC**——中断由阶段 2 各模块按需开启，避免当前阶段没有 ISR 时触发默认 fault。 |
| `template/main.c`              | 在 `SYSCFG_DL_init()` 之后立刻 `bsp_gpio_init()`；原先 4 处 `GPIO_OUT_*_`* 宏全部改为 `BSP_*_PORT/PIN`。                                                                                                               |
| `template/app/app_telemetry.c` | LED_G 心跳的 `GPIO_OUT_B_GPIO_LED_G_*` 改为 `BSP_LED_G_PORT/PIN`。                                                                                                                                            |
| `EIDE/.eide/eide.yml`          | `hardware` 虚拟目录注册 `bsp_gpio.{h,c}`。                                                                                                                                                                     |


GPIO power 启用不需要操心：UART/I2C/PWM/QEI 占用了 PORTA/PORTB 之后，SDK 在 `SYSCFG_DL_initPower()` 已经 `DL_GPIO_enablePower(GPIOA/B)`。

#### 取舍


| 维度           | SysConfig GPIO（原方案） | BSP GPIO（终方案）                                       |
| ------------ | ------------------- | --------------------------------------------------- |
| 多 pad 引脚正确性  | ❌ codegen bug，无解    | ✅ 用 IOMUX_PINCMxx + DL_GPIO_PIN_bit 字面常量，绕过 codegen |
| 改引脚的工作量      | 改 syscfg 一处         | 改 `bsp_gpio.h` 三行（PORT/PIN/IOMUX）                   |
| 引脚冲突检查       | SysConfig 自动        | 手工对照 `Stage0-PinAllocation.md`                      |
| 与 GUI 配置工具兼容 | ✅                   | ❌ GUI 看不到这些 GPIO（不影响 CLI 构建）                        |
| 改 SDK 版本就能恢复 | 需要等 TI 修 bug        | 与 SDK 修复无关，BSP 层稳定                                  |


**结论**：在 SDK 2.10 时代这是唯一可行做法。如果将来 SDK 修复了多 pad codegen，可以把 `bsp_gpio.{h,c}` 与 syscfg GPIO 并存（只用 SDK 不能正确处理的引脚走 BSP），但这会让"引脚分配"失去单一来源，得不偿失。**保持 BSP 接管 GPIO 不变**，文档以 `Stage0-PinAllocation.md` 与 `bsp_gpio.h` 互为校验。

#### 复盘要点

- **遇到 SDK / 工具链 bug 时，识别"无解 vs 有 workaround"很重要**。§8.3/§8.4 各反复改了一次 syscfg，每次都以为找到 workaround，实际只是把 bug 推到下一条 codegen 路径。建议遇到 SDK bug 先把根因定位到源码（不到 30 分钟读 `Common.js` 与 `.xdt` 模板），再决定 workaround 还是绕开整条路径。
- **TI MSPM0 SDK 2.10 在 multi-pad 引脚处理上有体系性 bug，不仅 GPIO module 受影响**——任何 codegen 路径只要它把 `pinInterfaceName` 留空（即 `undefined`）就会踩。peripheral pin（UART/I2C/PWM/QEI）走 `getDualBondedPadFunction()` 因为有明确的 `txPin`/`rxPin`/`ccp0Pin` 等 interface 名而幸免。本工程将来若用 OPA / COMP / DAC 自定义 pin（也是 GPIO 化的多 pad 引脚），同样要走 BSP 路径。
- **手写 GPIO init 唯一要小心的地方是 `IOMUX_PINCMxx` 与 `DL_GPIO_PIN_<bit>` 的对应**。我们在 `bsp_gpio.h` 注释里把每个引脚的 `(PORT, PIN, IOMUX)` 三元组列齐，与 `Stage0-PinAllocation.md` §3.2 业务模块表一一对应，避免日后改引脚时只改一半。
- **GPIO 引脚改动流程**（取代原 §8.4 复盘的"引脚 syscfg 写法决议表"）：
  1. 改 `Stage0-PinAllocation.md` §3.2 表格（单一真值源）；
  2. 改 `bsp_gpio.h` 对应 `BSP_<NAME>_PORT/PIN/IOMUX`；
  3. 如果是新加引脚，在 `bsp_gpio.c` 对应 `init_outputs_porta/b` 或 `init_inputs_porta/b` 加初始化语句；
  4. 业务代码用 `BSP_<NAME>_PORT/PIN` 引用，禁止再回到 `GPIO_OUT_`* 旧宏（旧宏已不再生成）。
- **不要试图在 GUI 里继续编辑 GPIO**。GUI 可能"恢复"GPIO 实例并触发 codegen。修 GPIO 永远改 BSP 文件。

### 8.6 dslite 烧录报「Length of block is N, but it should be divisible by 8」—— PT_LOAD 段强制 8 字节对齐

#### 现象（两轮）

`bsp_gpio` 接管 GPIO 后编译 / 链接全过，但 `dslite.bat` 烧录两次都失败：

**第一轮（喂 `.hex`）**：
```
error: CORTEX_M0P: Flash Programmer: Error in image size. Length of block is 12588,
       but it should be divisible by 8 since Flash Programmer writes in 64-bits
```

**第二轮（已把 eide.yml 的烧录命令改成 `.axf` —— 误判为 hex 格式问题）**：
```
Loading Program: ...LP_MSPM0G3507.axf
        Preparing ...
        PT_LOAD[0]: 0 of 12588 at 0x0
error: CORTEX_M0P: Flash Programmer: Error in image size. Length of block is 12588,
       but it should be divisible by 8 since Flash Programmer writes in 64-bits
```

`PT_LOAD[0]: 0 of 12588 at 0x0` 这一行解开了误会：**dslite 即使读 ELF，也会把 `p_filesz` 当作"待写 flash 数据块"长度去做 8 字节对齐校验，跟输入文件是 .hex 还是 .axf 无关**。换文件类型没用，`.axf` 只是顺便多吐了一行 `PT_LOAD` 诊断而已。

#### 根因

MSPM0G3507 flash controller 是 64-bit 字宽，每条 PROGRAM 指令一次写 8 字节，**部分写一个字是非法操作**。dslite 在烧录前对每个 PT_LOAD 段（== ROM 上一段连续 flash 内容）做长度 mod 8 == 0 的硬校验，凡不满足直接拒。

我们的 ROM 镜像末尾代码段长度 12588 = 1573 × 8 + 4，不对齐。链接器侧两个属性都没救：
- `ALIGNALL 8` 在 `ER_IROM1` 上只约束**每个 input section 的起始地址**对齐到 8，不约束最末 input section 的**结束位置**；
- `LR_IROM1 0x00000000 0x00020000` 这个 load region 的"max size = 0x20000"是 `.axf` 的 `p_memsz` 上限，跟实际写入 flash 的 `p_filesz` 是两回事，加 `ALIGN 8` 也只对齐起始。

`fromelf --i32` 转 hex 同样是按 PT_LOAD 字节实数输出，不补尾部 padding。SDK 自带 LaunchPad 例程**没踩这个坑纯属代码尺寸恰好落在 8 的倍数上**——示例代码量小且固定，巧合成立；我们项目代码 + driverlib + printf retarget 凑到 12588 字节，碰上偏移 4，立刻翻车。

#### 修法（最终版）：独立 .c 文件 + scatter 用模块级 `<obj>.o (+Last)` 选择器

让链接器在 `ER_IROM1` 末尾**物理写入** 8 字节内容，并保证它自身 8 字节对齐：这样配合 `ALIGNALL 8` 对前面所有 section 起始的对齐，整个 PT_LOAD 段大小一定是 8 的倍数。

**核心决策**：把 padding 变量单独放一个 .c 文件（[`template/hardware/bsp_flash_pad.c`](../../template/hardware/bsp_flash_pad.c)），scatter 用**模块级 + 节名双锁定** `bsp_flash_pad.o (.flash_pad, +Last)` 抢。两个约束缺一不可：

1. **模块级 selector 优先级 > 任何 `.ANY`** —— ARM 文档《Scatter file syntax》"Section selection rules"明确的"most specific first"规则。不会被 `.ANY (+RO)` / `.ANY (+XO)` 通配吃掉。
2. **`+Last` 必须跟"单一 section"或"单一属性"selector** —— `L6234E: Last must follow a single selector` 的约束。`bsp_flash_pad.o (+Last)` 单写会被链接器直接拒（等价"该 .o 所有 section"，是多 section 通配）。

**Scatter**（[`template/keil/mspm0g3507.sct`](../../template/keil/mspm0g3507.sct)）：

```diff
 ER_IROM1 0x00000000 ALIGNALL 8 0x00020000 {
   *.o (RESET, +First)
   *(InRoot$$Sections)
-  .ANY (+RO)
-  .ANY (+XO)
+  bsp_flash_pad.o (.flash_pad, +Last)   ; 8-byte 尾填充：模块+节名双锁定
+  .ANY (+RO)
+  .ANY (+XO)
 }
```

`bsp_flash_pad.o (.flash_pad, +Last)` 同时满足两条约束：模块名 + 节名是 ARMCLANG 选择器里能写的最特异性形式，再加 `+Last` 钉死位置。文本顺序无所谓 —— armlink 按选择器特异性匹配。

**C 侧**（[`template/hardware/bsp_flash_pad.c`](../../template/hardware/bsp_flash_pad.c)）：

```c
#include <stdint.h>

__attribute__((used, section(".flash_pad"), aligned(8)))
const uint64_t _flash_image_pad = 0xFFFFFFFFFFFFFFFFULL;
```

各属性的作用：
- `section(".flash_pad")` —— 把这个变量放到独立 section（实际不被 scatter selector 用到，但保留是为了在 .map 里一眼可识别）；
- `aligned(8)` —— 自身首地址对齐 8 字节（`uint64_t` 自然对齐已是 8，写出来是给读者看意图）；
- `used` —— ARMCLANG `-O2` 下没人引用就会剔除，配合 `extern` 链接（不加 `static`）可双保险；
- 值 `0xFFFFFFFFFFFFFFFF` —— flash 擦除态就是全 0xFF，写 0xFF 等同未写入，避免占用一个真实 word。

**EIDE 注册**：`bsp_flash_pad.c` 也要登记到 [`EIDE/.eide/eide.yml`](../../EIDE/.eide/eide.yml) 的 `hardware` 虚拟目录，否则不会进编译列表，scatter 上的 `bsp_flash_pad.o` 也就不存在 —— armlink 会静默跳过缺失模块的 selector，又回到 12604 状态。

**最终效果**：
- 假设代码 + RO data 自然结束于 offset X（X 可能不对齐）；
- ALIGNALL 8 把 X 向上对齐到 `(X+7) & ~7`，作为 `.flash_pad` 起始；
- `_flash_image_pad` 占 8 字节、本身是 8-byte 段，结束位置仍对齐 8；
- 整个 PT_LOAD 段大小 ∈ `[X+8, X+15]` 中、必为 8 的倍数（多写 0~7 字节填充 + 8 字节 pad，总额最多 15 字节，可忽略）。

#### 关键陷阱：四轮失败 → 模块级 + 节名 + +Last 才是最终解

走到最终方案之前**踩了四轮**：

**第一轮**：`.ANY (.flash_pad, +Last)`：

```
PT_LOAD[0]: 0 of 12604 at 0x0
error: ... Length of block is 12604, but it should be divisible by 8
```

`12604 - 12588 = 16`，但 `12604 mod 8` 仍是 4——多写了 16 字节却没改对齐！原推测：ARMCLANG armlink 的 `.ANYn` 是**优先级匹配**（n=1..9，大者优先；`.ANY` ≡ `.ANY1`），`_flash_image_pad` 自带 RO 属性、被前面的 `.ANY (+RO)` 通配抢走，塞到 RO section 中间，`+Last` 形同虚设。

**第二轮**：把选择器升到 `.ANY3 (.flash_pad, +Last)`：

```
PT_LOAD[0]: 0 of 12604 at 0x0           ← 一字节都没变
```

仍然 12604。说明**`.ANYn` 优先级规则在我们这里没生效**——armlink 实测对 `.ANY3 (.flash_pad, ...)` 与 `.ANY (+RO)` 之间的优先级处理跟文档描述不完全一致，可能是 SDK 给的 `*.o (RESET, +First)` 这类通配模块级 selector 干扰了 .ANY 的特异性计算，也可能是 ARMCLANG 19.x 某次更新破坏了节名级 .ANY 选择器的优先级语义。无论根因如何，**`.ANYn (<section_name>, ...)` 这种"用名字 + 优先级抢 RO 段"的写法在我们这条工具链上不可靠**。

**第三轮**：放弃 .ANY，改走模块级 selector `bsp_flash_pad.o (+Last)`，链接直接报错：

```
"...mspm0g3507.sct", line 37 (column 22): Error: L6234E: Last must follow a single selector.
```

ARM 文档《Image Layout in Scatter Files》"+FIRST and +LAST"小节写得很死：**`+Last` / `+First` 只能跟"单一 section selector"或"单一属性 selector"**——`bsp_flash_pad.o (+Last)` 没有 section 名也没有属性约束，等价"该 .o 所有 section"，是多 section 通配，被链接器直接拒。`*.o (RESET, +First)` 之所以合法是因为 `RESET` 收窄到了单一 section。

**第四轮（最终）**：在模块名后加节名收窄成"单一 section"——`bsp_flash_pad.o (.flash_pad, +Last)`：

```
PT_LOAD[0]: 0 of 12608 at 0x0           ← 12608 = 1576 × 8 ✓
Verifying    ... done
```

模块名 + 节名是 ARMCLANG selector 能写的最特异性形式：模块级特异性 → 优先级高于任何 `.ANY (+RO)`；节名收窄到单一 section → 满足 `+Last` 的语法约束。两条约束同时满足，armlink 必接受。

**法则记住**（两条独立约束，缺一不可）：

1. `.ANY (<section_name>, ...)` 这种"按 section 名 + .ANYn 抢已有 RO 属性的段 + +Last 排序"的写法在 ARMCLANG armlink 上**对带常用属性（+RO/+RW/+ZI/+XO）的 section 极易失效**；想精确控制某个对象的 section 位置，**唯一稳定方法是把它放到独立 .c 文件、用模块级 `<obj>.o (...)` selector 抢**。
2. `+Last` / `+First` 只能跟单一 section 或单一属性 selector；`<obj>.o (+Last)` 这种"该 .o 所有 section"的通配会触发 `L6234E`。**正确写法是 `<obj>.o (<section_name>, +Last)`** —— 模块名 + 节名双锁定，既满足"最特异性"又满足"单一 section"。

#### 备选修法（不推荐）

| 方案 | 评价 |
|------|------|
| `ER_PAD AlignExpr(ImageLimit(ER_IROM1), 8) FILL 0xFFFFFFFF N { }` | scatter 单点修改，但 `FILL` 永远写固定 N 字节、不会按需 0~7；写少了仍可能不对齐，写 8 又跟我们 `.flash_pad` 等价。复杂度无收益。 |
| post-build 跑 Python 脚本补 `.hex` 末尾 | 多一道 build 依赖，团队成员需 Python；且 .axf 仍不被填充 → 调试器加载 ELF 时没保护。 |
| 把 LR_IROM1 大小写死成已知 8 倍数 | 代码每改一行可能都要重算大小，运维代价不可接受。 |

#### 复盘

- **`PT_LOAD[i]: N of M at 0xXX` 这行诊断信息只有喂 `.axf` 才会打印**——如果你只看 `.hex` 报错的 "Length of block is N"，可能误以为是 hex 解析问题；把烧录文件换成 ELF 后，`PT_LOAD` 一行立刻揭示是**段大小**校验失败而非格式校验。**遇 dslite 烧录报错先换 .axf 拿诊断，再决定修法**——这是为什么本工程保留了 `.axf` 配置而不回退 `.hex`。
- **`ALIGNALL N` 和"PT_LOAD 段总长对齐 N"是两件事**。前者只管每个 input section 的**起始**位置，后者要靠链接器在最末添加实物字节强制实现。这条规则在 ARMCLANG / GCC / IAR 三家工具链下都成立。
- **MSPM0G3507 flash 64-bit 写要求是器件硬约束**，不只 dslite 受影响——以后换 OpenOCD / pyocd / J-Link 烧 MSPM0 时，**也都需要镜像 8 字节对齐**，flash loader 内部都会做同样校验。所以"`.flash_pad +Last`"这条 scatter 修补**一次到位、多 debugger 通用**。
- **SDK 例程"无修也能跑"是巧合**，不要照抄它的 scatter 就以为安全。校验规则一旦命中，调试 1~2 小时起步——本次从首次烧录失败到最终 root cause 大约 40 分钟，主要是被 `.hex` vs `.axf` 这条岔路误导。下次见到 `Length of block is N` + `divisible by 8` 直接跳到本节看修法。
- 如果以后扩到 MSPM0 全家族其它产品（如 MSPM0L1306 是 32-bit 写、MSPM0C1104 是 64-bit 写），**校验粒度可能不同**——MSPM0L 系列 32-bit 写时 PT_LOAD 长度需对齐 4，把 `.flash_pad` 改成 `uint32_t`、scatter `ALIGNALL 8` 改成 `ALIGNALL 4` 即可。

---

## 9. 阶段 1 剩余工作（留给下一轮）

按 [Overview.md:82-85](../Overview/Overview.md) 的范围，下一轮主控侧需要补完：

1. **0xAA55 帧协议解析层**：`middle/k230_proto.{c,h}`，CRC16-CCITT，状态机 `WAIT_AA → WAIT_55 → LEN → CMD → PAYLOAD → CRC_LO → CRC_HI → WAIT_55 → WAIT_AA`，校验失败丢弃整帧并 `bad_frames++`。
2. **业务帧定义**：
  - `IMU_TELEM` (主控 → K230)：pitch / pitch_rate / 车速（左右编码器）/ 状态位（电机使能、IMU 健康、电池告警…）。
    - `MOTION_CMD` (K230 → 主控)：v / ω / 模式（直立 / 巡线 / 自起立）。
    - `HEARTBEAT` 双向 50 Hz。
    - `ERROR`：故障码 + 时间戳。
3. **心跳超时降级**：K230 心跳超时 200 ms → 主控强制 `setVelocity(0, 0)`、关电机使能、LED_R 闪烁、蜂鸣告警；恢复后通讯由主控握手再放行。
4. **K230 RX DMA 升级**：把当前 BLOCK 模式改成 BASIC + 半满/全满双中断（或换 FIFO RX 中断），保证不丢任何字节。
5. **TX DMA 启用**：当前 K230_UART 在 SysConfig 里已勾选 TX DMA，但骨架未使用；下一轮 IMU_TELEM 50~100 Hz 推送时启用 DMA TX 队列。

非阶段 1 范围、但本轮决策需要落档以便后续不踩坑：

- 蓝牙的 RX 通道目前**只入环缓冲不读**；如果未来想从手机端调参（动态改 PID 系数等），可在 `app_telemetry.c` 主循环里加 `bsp_bt_uart_rx_pop` 简单文本协议。
- K230 RX BLOCK 模式在每 512 B 重装窗口期会丢 1~2 字节（DMA 重装的若干总线周期内 RX 数据可能溢出）。**1 Hz 字节计数自测**对此不敏感，但帧解析阶段会暴露——下一轮升级 DMA 时一并修。

---

## 10. 修订历史


| 日期         | 版本   | 修订内容                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           | 作者                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| ---------- | ---- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| 2026-04-29 | v0.1 | 阶段 1 初版：UART2 蓝牙实例化 + MPU6050 驱动 + 互补滤波 + VOFA+ JustFloat 转发 + K230 RX 接收骨架；含 HC-04 AT 配置步骤、VOFA+ 接入指引、验收清单与剩余 TODO                                                                                                                                                                                                                                                                                                                                                                                            | 主控团队                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 2026-04-29 | v0.2 | UART2 + PB17/PB16 命中 SDK multi-pad codegen bug，蓝牙串口外设改为 **UART3 + PB12/PB13**（单 pad），同步 [Stage0-PinAllocation.md](Stage0-PinAllocation.md) v0.5；新增 §8 踩坑记录小节，原 §8 / §9 顺延为 §9 / §10                                                                                                                                                                                                                                                                                                                            | 主控团队                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 2026-04-30 | v0.3 | 首轮编译暴露 4 类 AC6 / SDK 命名约定坑：`#pragma import` AC5 only、`SystemCoreClock` 无定义、`GPIO_OUT_PORT` 跨端口被拆成 per-pin `_PORT` 宏、`<stddef.h>` NULL 缺包含。修复 `bsp_log_uart.c`/`bsp_systick.c`/`main.c`/`app_telemetry.c`/`att_filter.c`，编译通过；新增 §8.2 落档                                                                                                                                                                                                                                                                        | 主控团队                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 2026-04-30 | v0.4 | 第二轮编译暴露两个**Stage 0 起就潜伏的盲区**：(1) 14 个 GPIO 引脚用了 SysConfig 不识别的 `assignedPort` 字段，对 multi-pad 引脚（PB0/16/17/22/26/27、PA0/1）回退到不存在的 `GPIOC` → 全改为 `pin.$assign = "PXNN"`；(2) `eide.yml` 把 `ti_msp_dl_config.{c,h}` 指向 `template/` 下的一次性陈旧桩（`SYSCFG_DL_*_init` 全是空函数），改为 `./ti_msp_dl_config.{c,h}`（即 `EIDE/` 下 syscfg.bat 实际生成位置）并删除桩文件。新增 §8.3 落档                                                                                                                                                                | 主控团队                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 2026-04-30 | v0.5 | 第三轮编译暴露 `pin.$assign` **在 multi-pad 引脚上同样不能修复 codegen bug**：SysConfig 仍把 PB22/26/27 解析为不存在的 `GPIOC.x`、把 PA15/16/26/27 的 PIN 号也错配（`_IOMUX/_PORT/_PIN` 三宏内部矛盾）。根因是 SDK 2.10 GPIO module 在没有 instance 级 `port` 时，对 multi-pad 引脚一律回退到第二 pad。修法：把 `GPIO_OUT/GPIO_IN` 拆成 `GPIO_OUT_A/B` + `GPIO_IN_A/B` 共 4 个实例，每个实例显式设 `port = "PORTA/B"`；C 端宏 `GPIO_OUT_GPIO_<NAME>_PORT/PIN` 同步改为 `GPIO_OUT_A/B_PORT` + `GPIO_OUT_A/B_GPIO_<NAME>_PIN`，main.c / app_telemetry.c 共 4 处引用更新。新增 §8.4 落档（含 SDK example 对照与多 pad 引脚清单） | 主控团队                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 2026-04-30 | v0.6 | 第四轮编译暴露：仅靠 instance 级 `port` 还不够——只要 pin 用 `pin.$assign = "PXNN"` 写法，SDK 2.10 codegen 就会**重新按全名查找物理 pad** 并独立命中多 pad bug，导致 `ti_msp_dl_config.c` 的 `SYSCFG_DL_GPIO_init()` 出现 `DL_GPIO_clearPins(GPIOC, ...)`（即使 .h 里 `GPIO_OUT_B_PORT` 已经是 `(GPIOB)`，.h 与 .c 两条 codegen 路径不一致）。修法：把 11 处 `pin.$assign = "PXNN"` 全改成 `assignedPin = "<bit>"`（纯数字字符串，让端口由 instance.port 推断，跳过 pad 名解析）。§8.4 内容订正：明确「instance.port + assignedPin 数字字符串」是 SDK 2.10 上多 pad 引脚**唯一**正确组合，并在「复盘要点」追加 GPIO 引脚 syscfg 写法决议表。           | 主控团队                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 2026-04-30 | v0.7 | 第五轮编译**仍然失败**——把 4 个 GPIO 实例 + `assignedPin = "<bit>"` 全部改对之后，`ti_msp_dl_config.c` 里 `SYSCFG_DL_GPIO_init()` 依然生成 `DL_GPIO_clearPins(GPIOC, GPIO_OUT_B_GPIO_LED_G_PIN                                                                                                                                                                                                                                                                                                                                          | ...)`。读` Common.js`源码定位 bug 在`identifyPadIndex()`行 2245-2247：GPIO codegen 路径`peripheralName + "." + pinInterfaceName`拼成字符串`"undefined.undefined"`，永远 indexOf = -1，padIndex 回退到 0，又因 device data 里多 pad 引脚 mode "1" 的 peripheralName 列出顺序是` "PCx/PBx"`（PC 在前），最终` getGPIOPort`返回`"GPIOC"`。**这是 SDK 2.10 GPIO module codegen 的体系性 bug，无 syntax workaround**。终极修法：syscfg 不再` addInstance()`GPIO 模块，14 个业务 GPIO 全部移到新建的`template/hardware/bsp_gpio.{h,c}`用`IOMUX_PINCMxx + DL_GPIO_PIN_ `字面常量手工初始化；`main.c`在`SYSCFG_DL_init()`后调用`bsp_gpio_init()`；业务代码` GPIO_OUT_*_`*宏全部改为`BSP_*_PORT/PIN`。新增 §8.5 落档（含` Common.js` bug 链条复盘、SysConfig vs BSP 取舍表、GPIO 引脚改动流程）。 |
| 2026-04-30 | v0.8 | 编译 / 链接全过后烧录失败：`dslite` 抛 `Length of block is 12588, but it should be divisible by 8`——MSPM0G3507 flash controller 按 64-bit 字写。**第一轮误判**：以为是 `.hex` 转换不补 padding，把 `EIDE/.eide/eide.yml` 烧录文件由 `${ExecutableName}.hex` 改为 `${ExecutableName}.axf`，期望 dslite 走 ELF segment metadata 自动对齐——结果**烧录依然报同一错**，仅多出 `PT_LOAD[0]: 0 of 12588 at 0x0` 一行诊断信息。dslite 对 .hex 和 .axf 走同一条 8-byte 校验路径。新增 §8.6 落档。 | 主控团队 |
| 2026-04-30 | v0.9 | §8.6 复盘 + 真修法：`PT_LOAD` 那行揭示是**段总长**校验失败而非格式问题。修复需在链接期向 `ER_IROM1` 末尾物理写入 padding 字节。`template/keil/mspm0g3507.sct` 在 `ER_IROM1` 末尾追加 `.ANY (.flash_pad, +Last)`；`template/main.c` 用 `__attribute__((used, section(".flash_pad"), aligned(8)))` 定义 `static const uint64_t _flash_image_pad = 0xFFFFFFFFFFFFFFFFULL`（8 字节定长 + 8 字节对齐 + `used` 防剔除 + 全 0xFF 等价 flash 擦除态）。配合 `ALIGNALL 8` 对 section 起始的对齐，期望整个 PT_LOAD 段大小变成 8 的倍数。**保留 `.axf` 烧录**：`PT_LOAD[i]` 诊断信息更详细。§8.6 重写，含"两轮现象 + 真根因 + scatter/C 双侧改动 + 备选方案对比表 + 多产品延展"完整落档。 | 主控团队 |
| 2026-04-30 | v0.10 | v0.9 修补**仍然失败**：`PT_LOAD[0]: 0 of 12604`，`12604 - 12588 = 16` 但 `12604 mod 8 = 4`。根因：ARMCLANG armlink 的 `.ANYn` 是**优先级匹配**（n=1..9，大者优先；`.ANY` ≡ `.ANY1`）；`_flash_image_pad` 自带 RO 属性，被前面 `.ANY (+RO)` 通配选择器**先**抢走，塞到 RO section 中段——`+Last` 因此匹配不到任何 section，形同虚设。修法：scatter 把 `.ANY (.flash_pad, +Last)` 改为 `.ANY3 (.flash_pad, +Last)`，优先级 3 抢先于 `.ANY1 (+RO)` 拿到 `.flash_pad`，再由 `+Last` 排末尾。同时把 `.ANY (+RO)` 显式写为 `.ANY1 (+RO)`、`.ANY (+XO)` 写为 `.ANY1 (+XO)`，让 priority 一目了然。§8.6 追加「关键陷阱：.ANY vs .ANY3」小节，含 12604 失败现象的逐字节解释与「依赖文本顺序 + +Last 排序在 ARMCLANG 上对带 +RO/+RW 等常用属性 section 一律不奏效」原则说明。 | 主控团队 |
| 2026-04-30 | v0.11 | v0.10 的 `.ANY3` 修补**仍然失败**，烧录依旧报 `PT_LOAD[0]: 0 of 12604`——一字节都没变。说明 ARMCLANG armlink 对 `.ANYn (<section_name>, ...)` 的优先级处理在本工具链上跟文档描述不一致（疑似 SDK 给的 `*.o (RESET, +First)` 通配模块级 selector 干扰了 .ANY 特异性计算，或 ARMCLANG 19.x 某次回归破坏了节名级 .ANY 选择器语义）。改走**模块级 selector**：把 `_flash_image_pad` 单独搬到新建的 [`template/hardware/bsp_flash_pad.c`](../../template/hardware/bsp_flash_pad.c)（独立 .c 文件、`extern` 链接 + `used` 属性双保险），scatter 改为 `bsp_flash_pad.o (+Last)`——**链接报 `L6234E: Last must follow a single selector`**：`+Last` 只能跟单一 section 或单一属性 selector，`<obj>.o (+Last)` 等价"该 .o 所有 section"是多 section 通配，被链接器拒。**最终修法**：scatter 加节名收窄成 `bsp_flash_pad.o (.flash_pad, +Last)`——模块名 + 节名是 ARMCLANG selector 最特异性形式（高于任何 .ANY），节名收窄到单一 section 又满足 `+Last` 语法约束。两条独立约束同时满足。`template/main.c` 同步删除原 `_flash_image_pad` 定义、保留指向独立文件的注释。`EIDE/.eide/eide.yml` 在 `hardware` 虚拟目录登记 `bsp_flash_pad.c`。§8.6 「修法」与「关键陷阱」整段重写：分四轮（`.ANY` → `.ANY3` → `<obj>.o (+Last)` 报 L6234E → `<obj>.o (.flash_pad, +Last)`）记录全过程；并给出两条独立法则——「精确控制 section 位置必须用独立 .c + 模块级 selector」「`+Last` 必须跟单一 section selector，所以模块名后必须跟节名」。烧录通过：`PT_LOAD[0]: 0 of 12608 at 0x0`，12608 = 1576×8 ✓。 | 主控团队 |

---

## 11. 硬件接线对照表（焊接 / 杜邦线施工面板）

> 本节为**装车焊接 / 调试杜邦线**的速查面板，对引脚功能定义的真源仍是 [Stage0-PinAllocation.md §3.2](Stage0-PinAllocation.md)；本表只提供"把哪根线焊到哪个针脚"的施工视图。
>
> 阶段 1 物理接线只涉及 3 个外接模块：**MPU6050**（I²C 姿态）、**HC-04**（蓝牙串口）、**K230**（与上位 SoC 通讯，本阶段仅 RX 骨架，可先不接）。其它模块（TB6612 / 编码器 / 蜂鸣器 / 激光等）属于后续阶段，本阶段不接线、保持悬空，固件已默认拉低 STBY/LASER_EN 防止误动作。
>
> **共地是装机前的硬约束**：所有外接模块的 GND 必须与 LaunchPad GND（J101 任意 GND 针 / 板边 GND 焊盘）单点星型汇接。任何"忘了接 GND，靠 USB 共地"的接线都会在电机上电时炸 I²C 总线。

### 11.1 MPU6050（I²C 姿态传感器）

> **接口**：I²C1 主机 400 kHz；上拉 4.7 kΩ → 3V3 必须**外置**（LaunchPad 默认未提供）。
>
> **本阶段不用 INT 引脚**（走 SysTick 1 kHz 软轮询读 raw），但 INT 走线建议**预留焊接**，下一阶段平衡环时可直接切到中断模式不用拆线。
>
> **电源**：MPU6050 模块 VCC 接 **3V3**（LaunchPad J27 / J28 任意 3V3 针），**不要接 5V**——MPU6050 芯片本身 2.375~3.46 V 供电，市售模块虽然带 LDO 但部分批次直通无 LDO，5V 直供瞬间烧片。

| MPU6050 模块引脚 | 信号    | MSPM0G3507 引脚 | LQFP pin | 外设 / 功能          | 备注                                          |
| ---------- | ----- | ------------- | -------- | ---------------- | ------------------------------------------- |
| VCC        | 3V3   | LaunchPad 3V3 | —        | 电源               | 板载 J27 / J28 任意 3V3，**勿接 5V**               |
| GND        | GND   | LaunchPad GND | —        | 地                | 与主控、HC-04、K230 共地（星型汇接）                     |
| SCL        | I²C 时钟 | **PB2**       | 50       | I2C1_SCL         | **必接 4.7 kΩ 上拉到 3V3**                       |
| SDA        | I²C 数据 | **PB3**       | 51       | I2C1_SDA         | 同上 4.7 kΩ 上拉                                |
| INT        | DataReady | **PB4**       | 52       | GPIO + EXTI 上升沿  | 阶段 1 不用、可悬空；建议焊一根杜邦线占位，下一阶段直接切中断            |
| AD0        | I²C 地址低位 | GND           | —        | —                | 接 GND 时模块地址 = 0x68（默认）；接 3V3 时 = 0x69，需改驱动 |
| XCL / XDA / FSYNC | — | 悬空            | —        | —                | 主从 I²C / 帧同步，本项目不用                          |

### 11.2 HC-04（蓝牙串口模块）

> **接口**：UART3 (PB12 TX / PB13 RX) 115200 8N1；**TX/RX 必须交叉接**（HC-04 的 TXD ↔ MSPM0 的 RX，反之亦然）。
>
> **AT 配置在装车之前完成**（一次性，见 §6.1）；装车后正常上电即遥测，**不需要再进 AT 模式**。
>
> **电源**：HC-04 板载 LDO，VCC 接 **5V**（LaunchPad J27 5V 针），不要接 3V3，否则蓝牙模块蓝灯不亮 / 配对失败。

| HC-04 模块引脚 | 信号        | MSPM0G3507 引脚 | LQFP pin | 外设 / 功能      | 备注                                      |
| --------- | --------- | ------------- | -------- | ------------ | --------------------------------------- |
| VCC       | 5V        | LaunchPad 5V  | —        | 电源           | LaunchPad J27 5V 或 USB 5V               |
| GND       | GND       | LaunchPad GND | —        | 地            | 共地                                      |
| TXD       | HC-04 → 主控  | **PB13**      | 1        | UART3_RX     | **交叉**：模块 TXD → 主控 RX                   |
| RXD       | 主控 → HC-04 | **PB12**      | 64       | UART3_TX     | **交叉**：主控 TX → 模块 RXD（部分模块此脚有 1 kΩ 限流串电阻） |
| EN / KEY  | AT 模式控制   | 悬空            | —        | —            | AT 模式只在初次配置时用 USB-TTL 拉高一次（见 §6.1）；装车后悬空即可 |
| STATE     | 连接状态指示    | 悬空（可选）       | —        | —            | 配对连上时输出高，本阶段固件不读；如需指示 LED 直接外挂          |

### 11.3 K230（上位 SoC 通讯，本阶段可暂不接）

> **接口**：UART1 (PB6 TX / PB7 RX) 921600 8N1，DMA RX 已就绪、DMA TX 阶段 2 启用；本阶段固件**只统计 RX 字节数**做硬件自测，不解析协议。
>
> **本阶段可选接线**：
>
> - **不接 K230**：固件正常运行，1 Hz 日志里 `k230_rx=0 b/s`；正常。
> - **接 K230**：能收到 K230 端发的任意字节流，1 Hz 日志 `k230_rx` 非零线性增长。
> - **自测回环（无 K230 时验证 UART1 通路）**：拿一根杜邦线把 PB6 和 PB7 短接，主控 TX 自发自收（**当前阶段固件未启用 TX**，所以自测需要另外手动触发，详见验收清单 §7"K230 RX 自测"行）。
>
> **电平**：K230 GPIO 也是 3.3 V，可与 MSPM0 直连，**不需要电平转换**；唯一硬要求是共地。

| K230 端引脚 | 信号        | MSPM0G3507 引脚 | LQFP pin | 外设 / 功能      | 备注                                  |
| ------- | --------- | ------------- | -------- | ------------ | ----------------------------------- |
| GND     | GND       | LaunchPad GND | —        | 地            | **唯一硬约束** —— 不接共地 UART1 必收乱码        |
| UART_TX | K230 → 主控  | **PB7**       | 59       | UART1_RX     | 交叉：K230 TX → 主控 RX                  |
| UART_RX | 主控 → K230 | **PB6**       | 58       | UART1_TX     | 交叉：主控 TX → K230 RX（阶段 1 主控暂不发，此线可悬空但建议焊） |
| 5V / 3V3 | 电源       | **不互供**       | —        | —            | K230 自有电源，**不要**从 LaunchPad 引电压过去   |

### 11.4 调试链路（板载 XDS110，无需杜邦线）

> 阶段 1 调试日志走 LaunchPad 自带的 XDS110-ET 桥：USB-CDC 在电脑端虚拟出一个 COM 口，固件 `printf` → UART0_TX (PA10) → XDS110 → USB → 电脑串口监视器。**只要 J21 / J22 跳线 ON，不需要任何外接线**。

| 通路              | LaunchPad 跳线  | 主控引脚                | LQFP pin | 外设         | 备注                                        |
| --------------- | ------------ | ------------------- | -------- | ---------- | ----------------------------------------- |
| LOG_TX (主控 → PC) | **J21 ON**   | **PA10**            | 56       | UART0_TX   | 电脑端 115200 8N1 收 `printf`                 |
| LOG_RX (PC → 主控) | **J22 ON**   | **PA11**            | 57       | UART0_RX   | 阶段 1 固件不读，预留下一阶段串口调参                      |
| SWD             | **J101 ON**  | **PA19 / PA20**     | 12 / 13  | DEBUGSS    | XDS110 → MSPM0 烧录 + 调试，必接                 |

### 11.5 接线总验收（装车前一次性核对）

- [ ] MPU6050 VCC = 3V3（**不是** 5V），万用表测 2.4~3.4 V。
- [ ] MPU6050 SCL/SDA 各**有**一个 4.7 kΩ 上拉到 3V3（断电后用万用表测 SCL ↔ 3V3 阻值 ≈ 4.7 kΩ）。
- [ ] HC-04 VCC = 5V，模块通电后红灯快闪（未配对态）。
- [ ] HC-04 TXD → 主控 PB13、HC-04 RXD → 主控 PB12（**交叉**核对，万用表蜂鸣档点测两端 LQFP 焊盘）。
- [ ] 所有模块 GND 与 LaunchPad GND 单点星型汇接，万用表测各 GND 节点之间阻值 < 0.1 Ω。
- [ ] LaunchPad J21 / J22 / J101 跳线全 ON。
- [ ] 阶段 1 不用的外设（TB6612 / 编码器 / 蜂鸣器 / 激光器 / 按键）相关引脚全部**悬空**，**不要**接任何外部电源或上拉。

> **整线后第一次通电的预期现象**：
>
> 1. LaunchPad XDS110 红灯常亮 → MCU 上电；
> 2. HC-04 红灯快闪（未配对）→ 蓝牙就绪；
> 3. 串口监视器（XDS-UART，115200）每秒打印一行 `[hb] t=Ns pitch=... rate=... acc=... T=... k230_rx=...b/s`；
> 4. 蓝牙手机助手配对 HC-04（PIN 1234）→ 连上后能收到二进制流，约 20 字节/帧、100 帧/秒；
> 5. **板载红色 LED_R（PB26）熄灭**——表示 MPU6050 init 成功（若常亮则 init 失败，去看 XDS-UART 错误码）；
> 6. **板载绿色 LED_G（PB27）以 5 Hz 闪烁**——心跳，证明主循环在跑。


