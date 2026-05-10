# 阶段 1.5 ｜ IMU 元件替换：MPU6050 → ATK-MS901M（含 1.6 引脚重排）

> 文档定位：临时阶段，记录 IMU 链路从 I²C+MPU6050 切换到 UART+ATK-MS901M 的全部变更——硬件原因、引脚迁移、UART 实例化、C 版解析器移植、VOFA 通道重映射、上位机一次性配置流程、验收清单。
>
> **2026-05-08 追加 §11**：Stage 1.6 引脚集中化重排，IMU 串口由 UART2/PA21/PA22 迁到 UART3/PB12/PB13（蓝牙整体下线），原因是 PA21 不在 LaunchPad BoosterPack 排针上需要焊接。本文 §1-§10 中所有 `PA21/PA22/UART2` 字样在 1.6 后实际指 `PB12/PB13/UART3`，阅读时请注意；§5 SysConfig 改动以 §11.2 的最新 diff 为准。
>
> 关联文档：
>
> - 项目总览：[../Overview/Overview.md](../Overview/Overview.md)
> - 引脚分配真源（已升级到 v0.8）：[Stage0-PinAllocation.md](Stage0-PinAllocation.md)
> - 阶段 1（蓝牙整段被本文 §11 下线，K230/SDK bug 复盘部分仍有效）：[Stage1-IMU-BT-Telemetry.md](Stage1-IMU-BT-Telemetry.md)
> - 移植参考（C++ 原版解析器）：[../chore/Ms901mStreamParser.cpp](../chore/Ms901mStreamParser.cpp) / [.h](../chore/Ms901mStreamParser.h)

---

## 1. 任务回顾与决策摘要

### 1.1 触发原因

阶段 1 验收装机时核对 LaunchPad 原理图与实际电路板，发现：

- **`PB2 / PB3` 这对 I²C1 引脚未板载 4.7 kΩ 上拉电阻**——LaunchPad 默认布局把 SDA/SCL 当成普通 GPIO，没有拉到 3V3。原阶段 0 文档第 5.6 节"I²C1 总线上能扫到 MPU6050"自检条目无法通过，I²C 总线**完全开路**。
- 修复方案有两条：① 在小车主板/转接板上自加两颗 4.7 kΩ 0805 上拉电阻；② 改用串口元件。
- 出于"避免热风焊台二次返工损伤 LaunchPad"的工程风险考量，选②；以串口推送的 ATK-MS901M 替代 MPU6050。

### 1.2 关键决策（与项目负责人 2026-05-07 确认）

| 决策项 | 结论 | 理由 |
|--------|------|------|
| IMU 元件 | 正点原子 ATK-MS901M（板载 9 轴 + 气压 + 15 阶 EKF，UART 主动上报） | 串口接口免上拉电阻；板载 EKF 卸载主控算力 |
| 主控 UART 外设 | **UART2** | UART0 已占（XDS log）、UART1 已占（K230）、UART3 已占（HC-04 蓝牙）；UART2 之前因 multi-pad bug 暂未启用 |
| UART2 引脚 | **TX = PA21（PINCM46）/ RX = PA22（PINCM47）**，**单 pad** | 单 pad 引脚不踩 [Stage1 §8.5](Stage1-IMU-BT-Telemetry.md) 的 multi-pad codegen bug；PA22 在 Stage 0 已显式标"留作蓝牙或扩展"，PA21 在 Stage 0 §3.4 因 VREF- 列禁用，但本工程用内部 VREF（2.5 V），PA21 实际可用 |
| 波特率 | **115200 8N1** | 出厂默认；MS901M 5 帧 × 200 Hz × ~15 B ≈ 17 kbps，远低于 115200 链路容量；调试期不动上位机参数 |
| FIFO / 中断 | UART2 FIFO + RX 半满中断；不开 DMA | 吞吐 < 20 kB/s，DMA 复杂度无收益，且 K230 UART1 已占 2 个 DMA 通道 |
| 姿态算法 | **直接采纳 0x01 帧的 pitch/roll/yaw**，删 `att_filter` | MS901M 板载 EKF 输出已足够；主控不再需要在 1 kHz tick 里做加速度+陀螺仪互补滤波，CPU 占用更低 |
| MS901M TX 线 | 焊上（PA21 → MS901M RX） | 保留主控向 MS901M 发配置/校准命令的能力；运行期不发 |
| 量程 | ±4 g / ±2000 dps（出厂默认） | 与 cpp 原版 [parser](../chore/Ms901mStreamParser.cpp) 默认一致；与 [`ms901m.h`](../../template/middle/ms901m.h) 中 `ms901m_init(4, 2000)` 强绑定 |
| 失效检测 | 上电 500 ms 内未收到 0x01 → fatal handler（LED_R + 蜂鸣 + 死循环） | 与原 mpu6050_init 失败处理同形态，保持 main.c 控制流一致 |

---

## 2. 元件对比

| 维度 | MPU6050（旧） | ATK-MS901M（新） |
|------|--------------|------------------|
| 接口 | I²C，需外部 4.7 kΩ 上拉 | UART，无上拉要求 |
| 数据来源 | raw 14-byte burst（accel/gyro/temp） | 5 种主动上报帧：0x01 RPY / 0x02 四元数 / 0x03 raw gyro+accel / 0x04 mag+temp / 0x05 baro+alt |
| EKF 位置 | 主控（互补滤波 / Mahony 自实现） | 模块板载（15 阶 EKF，主控被动接收） |
| 主控 CPU 占用 | 1 kHz I²C burst (~0.4 ms) + 互补滤波 (~5 µs) | 1 kHz drain（~5 µs）+ 状态机解析（~10 µs/帧）|
| 磁干扰风险 | 无（无磁力计） | 有：yaw 受电机 PWM 磁干扰，**本工程不使用 yaw**（balancing 只用 pitch + 视觉用 K230 端的相位角） |
| 失效模式 | I²C NACK / SDA 卡死 / 总线开路（本次触发原因） | UART 断流 / 校验和错累计 |
| 主控引脚 | PB2 SCL / PB3 SDA / PB4 INT，3 脚 | PA21 TX / PA22 RX，2 脚 |

---

## 3. 引脚迁移摘要

详细 diff 见 [Stage0-PinAllocation.md v0.7 §7 修订历史](Stage0-PinAllocation.md)。本文只列净变化：

释放：

- `PB2 (IMU_SCL)` / `PB3 (IMU_SDA)` / `PB4 (IMU_INT)` —— 全部下线，未来如有 I²C 需求再走外部转接板上拉
- `I2C1` 实例 —— 整体从 SysConfig 下线，`bsp_imu_i2c.{c,h}` git rm

新占用：

- `PA21` (LQFP pin 17, PINCM46, 单 pad) → `UART2_TX`，主控 → MS901M（发配置/校准命令）
- `PA22` (LQFP pin 18, PINCM47, 单 pad) → `UART2_RX`，MS901M → 主控（业务接收）
- `UART2` 实例 —— SysConfig 新增 `UART_IMU`（`UART_IMU_INST` 宏在 `ti_msp_dl_config.h` 自动生成）

新增驱动文件：

- [`template/hardware/bsp_imu_uart.{h,c}`](../../template/hardware/bsp_imu_uart.h) —— UART2 RX 256 B 环缓 + 阻塞 TX，结构与 [`bsp_bt_uart.{h,c}`](../../template/hardware/bsp_bt_uart.h) 镜像
- [`template/middle/ms901m.{h,c}`](../../template/middle/ms901m.h) —— 字节级状态机解析器，单线程使用、float 量纲

删除：

- [`template/hardware/bsp_imu_i2c.{c,h}`](../../template/hardware/bsp_imu_i2c.h)（git 历史可回退）
- [`template/middle/mpu6050.{c,h}`](../../template/middle/mpu6050.h)
- [`template/middle/att_filter.{c,h}`](../../template/middle/att_filter.h)

---

## 4. C 版解析器 vs C++ 原版

[`Ms901mStreamParser.cpp`](../chore/Ms901mStreamParser.cpp) 来自其它项目，依赖 Qt（QByteArray / QList / QString）；本工程移植到 C 时做的关键调整：

| 维度 | C++ 原版 | 本工程 C 版 |
|------|---------|-------------|
| 缓冲数据结构 | `QByteArray m_buffer` 动态扩容 + `mid()` 压缩 | `static uint8_t s_data[32]` + 累加校验和 + 状态机游标，**零堆分配** |
| 解析模式 | 缓冲式：累积字节 → `tryExtractFrame` 找 sync 字 → `mid` 提取 payload | 字节级状态机：SYNC1 → SYNC2 → ID → LEN → DATA(*LEN) → CHECKSUM，每字节一拍推进 |
| 校验和算法 | `for (i in [idx, idx+total-1)) sum += buf[i]; sum == buf[idx+total-1]` | 边收边累加 `s_chk += b`，CHECKSUM 状态对比；与 C++ 完全等价（sum(0x55+0x55+ID+LEN+DATA[*]) & 0xFF） |
| 数值类型 | `double` | `float`（Cortex-M0+ 无 FPU；double 走 soft-double 是 float 的 2~3×） |
| 加速度量纲 | m/s² (× fsr × 9.8) | g（× fsr / 32768），便于阅读，平衡环只需相对量 |
| 陀螺量纲 | rad/s (× fsr × π/180) | dps（× fsr / 32768），直接喂 PD 速率项无 rad↔deg 转换 |
| 输出形态 | `Snapshot19 = std::array<double,19>` 19 元素数组 | `ms901m_snapshot_t` 字段化 struct，编译期 layout 与 VOFA 通道映射强绑定 |
| 触发模型 | 0x03 帧触发一次 `buildSnapshot()`，返回 `QList<Snapshot19>` | 每帧到达即更新对应字段；`ms901m_get_snapshot(out)` 读最新，主循环按需轮询 |
| 错误处理 | 校验失败：丢字节并继续；长度 > 64 跳过 | 校验失败：`s_bad_frames++` + 重置状态机；长度 > 32：同样累计 + 重置 |
| 0x02 / 0x05 | 进 Snapshot19 索引 [6..9] / [17..18] | 仅校验通过、不存字段（本工程不用四元数与气压；解析路径保留以免被误算 bad_frame） |
| 调试格式化 | `formatDebug()` 返回 QString | 删除（嵌入式不需要；用 1 Hz `printf("[hb] pitch=...")`） |

---

## 5. SysConfig 改动

[`EIDE/LP_MSPM0G3507.syscfg`](../../EIDE/LP_MSPM0G3507.syscfg) 仅改两处：

```diff
-const I2C    = scripting.addModule("/ti/driverlib/I2C",  {}, false);
 const UART   = scripting.addModule("/ti/driverlib/UART", {}, false);

-const IMU_I2C = I2C.addInstance();
-IMU_I2C.$name                              = "I2C_IMU";
-IMU_I2C.basicEnableController              = true;
-IMU_I2C.basicControllerStandardBusSpeed    = "Fast";
-IMU_I2C.peripheral.$assign                 = "I2C1";
-IMU_I2C.peripheral.sdaPin.$assign          = "PB3";
-IMU_I2C.peripheral.sclPin.$assign          = "PB2";
+const IMU_UART = UART.addInstance();
+IMU_UART.$name                             = "UART_IMU";
+IMU_UART.targetBaudRate                    = 115200;
+IMU_UART.enableFIFO                        = true;
+IMU_UART.enabledInterrupts                 = ["RX"];
+IMU_UART.peripheral.$assign                = "UART2";
+IMU_UART.peripheral.txPin.$assign          = "PA21";
+IMU_UART.peripheral.rxPin.$assign          = "PA22";
```

注意：

- **不要** addInstance 第二个 UART2，会和 `UART_BT` (UART3) / `UART_K230` (UART1) / `UART_LOG` (UART0) 冲突。
- **不要** 把 enableDMARX / enableDMATX 打开。MS901M 默认 200 Hz 上报、单帧 ≤ 17 B，吞吐 17 kbps，DMA 完全过设计；DMA 通道留给 K230 UART1。
- 重新生成 `ti_msp_dl_config.{c,h}` 后，`UART_IMU_INST` / `UART_IMU_IRQN` 宏会自动出现，`bsp_imu_uart.c` 的 `UART2_IRQHandler` 函数名与 `UART_IMU_INST` 一致即可。

---

## 6. 软件架构改动

### 6.1 调用链 diff

```
旧（Stage 1）：
  SysTick 1 kHz → mpu6050_read_raw (I2C burst 14 B, ~0.4 ms)
                → att_filter_update (互补滤波)
                → att_state_t {pitch, rate, acc, temp}
                → vofa_send / printf

新（Stage 1.5）：
  UART2_IRQHandler (FIFO 半满) → 256 B 环缓
  SysTick 1 kHz → bsp_imu_uart_rx_pop_bulk (~5 µs)
                → ms901m_feed_bytes (状态机推进)
                → ms901m_get_snapshot (字段拷贝)
                → ms901m_snapshot_t {pitch, roll, gx/gy/gz, ax/ay/az, temp}
                → vofa_send / printf
```

### 6.2 文件改动清单

| 路径 | 动作 | 说明 |
|------|------|------|
| [`EIDE/LP_MSPM0G3507.syscfg`](../../EIDE/LP_MSPM0G3507.syscfg) | 改 | 删 `IMU_I2C` 实例 + I2C 模块导入；加 `UART_IMU` 实例 |
| [`EIDE/.eide/eide.yml`](../../EIDE/.eide/eide.yml) | 改 | `hardware` 删 `bsp_imu_i2c.{c,h}` 加 `bsp_imu_uart.{c,h}`；`middle` 删 `mpu6050.{c,h}` `att_filter.{c,h}` 加 `ms901m.{c,h}` |
| [`template/hardware/bsp_imu_uart.{h,c}`](../../template/hardware/bsp_imu_uart.h) | 新增 | UART2 RX 256 B 环缓 + 阻塞 TX；UART2_IRQHandler 把 FIFO 字节排入环缓 |
| [`template/middle/ms901m.{h,c}`](../../template/middle/ms901m.h) | 新增 | 字节级状态机解析；公开 `ms901m_init / feed_bytes / has_attitude / get_snapshot / good_frames / bad_frames` |
| [`template/hardware/bsp_imu_i2c.{h,c}`](../../template/hardware/bsp_imu_i2c.h) | **删除** | git rm，历史可回退 |
| [`template/middle/mpu6050.{h,c}`](../../template/middle/mpu6050.h) | **删除** | 同上 |
| [`template/middle/att_filter.{h,c}`](../../template/middle/att_filter.h) | **删除** | 同上；MS901M 板载 EKF 已替代主控互补滤波 |
| [`template/main.c`](../../template/main.c) | 改 | init 序列：`bsp_imu_uart_init` + `ms901m_init(4, 2000)`；fatal 判定改为 500 ms 内 `ms901m_has_attitude()` 仍 false |
| [`template/app/app_telemetry.c`](../../template/app/app_telemetry.c) | 改 | 1 kHz 任务改为 drain UART2 → ms901m_feed_bytes；VOFA 4 通道与 1 Hz 日志重映射 |
| [`template/hardware/bsp_gpio.{h,c}`](../../template/hardware/bsp_gpio.h) | 改 | 删 `BSP_IMU_INT_*` 宏 + `init_inputs_portb()` 全函数（PB4 释放，PORTB 不再有业务输入） |
| [`docs/TaskLog/Stage0-PinAllocation.md`](Stage0-PinAllocation.md) | 改 | 升 v0.7：§1 决策行 / §3.2 业务表 / §3.4 禁用表 / §4.3 IMU 详表 / §4.7 GPIO 表 / §5.6 上电验证 / §7 修订历史 |
| [`docs/TaskLog/Stage1-IMU-BT-Telemetry.md`](Stage1-IMU-BT-Telemetry.md) | 改 | 头部加替代提示，引导读者跳到本文 |

---

## 7. VOFA+ 4 通道重映射

| 通道 | Stage 1（旧）| Stage 1.5（新）| 物理含义 | 来源帧 |
|------|--------------|----------------|----------|--------|
| CH0 | `pitch_deg`（互补滤波后） | `pitch_deg` | 平衡环主用俯仰 | 0x01（MS901M EKF）|
| CH1 | `pitch_rate_dps`（陀螺 Y）| `roll_deg` | 装机姿态校验（横滚理想 ≈ 0）| 0x01 |
| CH2 | `pitch_acc_deg`（仅加速度）| `gy_dps` | 俯仰角速度，平衡环 D 项参考 | 0x03（raw gyro Y）|
| CH3 | `temp_c` | `temp_c` | 内温度，监控热漂 | 0x04 |

> **CH0 / CH2 静态对照**：水平静置 30 s 时，CH0 噪声标准差 < 0.1°（板载 EKF 已滤波）；CH2（gy）应近 0 但有 ±0.5 dps 漂移（陀螺零偏，正常）。
> **抬车头测试**：抬车头 → CH0 缓变增大、CH2 跳变出正脉冲、CH1 几乎不动。
> **轴向极性**：若 CH0 与"车头上扬为正"反向，需在 ATK 上位机里改 MS901M 安装方向（XYZ 翻转），**不要**在主控 C 代码里翻——避免与 0x03 raw 解出来的 gy 极性脱钩。

---

## 8. ATK 上位机一次性配置（装车前必做）

> ATK-MS901M 出厂默认：115200 8N1 + 主动上报 0x01..0x05 五帧 + ±4 g / ±2000 dps + 200 Hz 输出。如果模块是新拆封的，多数情况下**直接焊到主板就能用**。本节只在以下情况需要执行：① 模块被前任使用者改过参数；② 实测 0x01 极性与车体不一致；③ 想把上报频率降到 100 Hz 节省 UART 带宽。

### 8.1 物理接线（USB-TTL 转接，不焊到主板）

- USB-TTL TXD → MS901M RX
- USB-TTL RXD → MS901M TX
- USB-TTL 5V → MS901M VCC（板载 LDO，3.3/5 V 通吃）
- USB-TTL GND → MS901M GND

### 8.2 软件配置

1. 下载正点原子官方 **MiniBalance / MS901M 上位机**（厂家网站；ATK 论坛也有）。
2. 串口选 USB-TTL 对应 COM、波特率 115200，点"打开串口"，左下应立即看到帧率统计。
3. **如需恢复出厂**：菜单"设置"→"恢复出厂设置"→"是"，等回显"配置成功"。
4. **如需对齐车体安装方向**：把 MS901M 按车体最终装机姿态摆好（水平 + 头朝车头），上位机"姿态"页 → "轴方向"按钮逐次切换直到 RPY 三轴显示为 (0, 0, 0)，点"保存到 EEPROM"。
5. **如需改帧率**：菜单"输出设置" → "回传速率" → 选 100 Hz / 200 Hz / 500 Hz；并保留 0x01 + 0x03 + 0x04 三个帧（本工程业务用），0x02/0x05 可关也可留。
6. **如需改波特率**：上位机"系统设置"→"串口配置"，**改完先记录新值**，点保存后**断电重启** USB-TTL 用新波特率重连验证；同步把 `EIDE/LP_MSPM0G3507.syscfg` 的 `IMU_UART.targetBaudRate` 从 115200 改为新值，并重跑 `syscfg.bat`。

### 8.3 装车后接线

- MS901M VCC → 主控 5V（**不要** 3V3——板载 LDO 输入 3V3 时 LDO 不起压）
- MS901M GND → 主控 GND（与 LaunchPad / TB6612 / HC-04 / K230 单点星型共地）
- MS901M TX → 主控 PA22（UART2_RX，LQFP pin 18）
- MS901M RX → 主控 PA21（UART2_TX，LQFP pin 17）
- 接线**交叉**：模块 TX 接主控 RX，模块 RX 接主控 TX

---

## 9. 验收清单

> 在硬件焊好 / SDK 重新生成 `ti_msp_dl_config.{c,h}` 后逐项打钩。

| 项 | 通过条件 | 验证方式 | 状态 |
|---|---------|---------|------|
| 工程能编译 | EIDE 构建无 error，allow warning | EIDE 构建按钮 | [ ] |
| `ti_msp_dl_config.h` 自动生成 `UART_IMU_INST` 宏 | grep 该宏存在且指向 `UART2` | 文本检查 | [ ] |
| MS901M 在线 | 上电后 XDS-UART 不出现 `[FATAL] ms901m boot timeout` | XDS-UART 日志 | [ ] |
| 1 Hz 心跳 | XDS-UART 每秒出现 `[hb] t=...s pitch=... roll=... gy=... T=... ms901m_good=... bad=0 over=0 k230_rx=...b/s` | XDS-UART | [ ] |
| 帧率合理 | `ms901m_good` 字段每秒增量在 200~1000（5 帧 × 200 Hz = 1000）之间 | XDS-UART | [ ] |
| 校验干净 | `ms901m_bad=0` 持续 60 s 不变 | XDS-UART | [ ] |
| 环缓不溢出 | `over=0` 持续 60 s 不变 | XDS-UART | [ ] |
| 蓝牙 VOFA+ 4 通道 | 4 路波形流畅、无丢帧、CH0 与 CH2 趋势一致 | VOFA+ JustFloat 4 通道 | [ ] |
| 静态俯仰噪声 | 模块水平静置 30 s，CH0（pitch）标准差 < 0.1° | VOFA+ 录波 → 离线算 std | [ ] |
| 动态滞后 | 手抖 ±20°，CH0 跟随车体姿态视觉滞后 < 30 ms（MS901M EKF 内部延迟） | VOFA+ 同屏对比 | [ ] |
| 轴向极性 | 抬车头 → CH0 增大；低车头 → CH0 减小；不一致则用 §8.2 步骤 4 在上位机改安装方向 | 手动倾斜模块 | [ ] |
| 跌倒安全 | 全程电机 STBY = 0、LASER_EN = 0（万用表测量 PB0 / PA1 = 0 V） | 万用表 | [ ] |
| K230 RX 自测 | PB6 短接 PB7（或 K230 端发数据），1 Hz 日志 `k230_rx` 字段非零 | 杜邦线短接 + 看日志 | [ ] |

---

## 10. 风险与回退

### 10.1 已知风险

- **MS901M 板载 EKF 不完全是黑盒**：若磁场/振动严重影响 yaw，最坏退化是 0x01 帧的 yaw 漂移；本工程 yaw 不进 VOFA、不参与平衡环（仅 K230 视觉端用相位角 φ），影响可控。
- **上电首帧延迟**：MS901M 内部 EKF 收敛需要 200~500 ms，早期 500 ms 上电检测窗口与之边界重合；当前 [`template/main.c`](../../template/main.c) 已把 `MS901M_BOOT_TIMEOUT_MS` 放宽到 3000 ms，并在装车模式增加 2500 ms 姿态静默窗口。

### 10.2 回退路径（若 MS901M 链路不可用）

如果实测 0x01 pitch 受电机 PWM 干扰严重（超出预期），不需要回退到 MPU6050（PB2/PB3 上拉问题仍在），而是：

1. **方案 A（推荐）**：保持 UART2 链路，但姿态来源切换到 0x03 帧的 raw gyro+accel，在主控重新做互补滤波。从 git 历史拉回 [`att_filter.{c,h}`](../../template/middle/att_filter.h)（commit ID 见本次 Stage 1.5 提交的 parent）；改 [`app_telemetry.c`](../../template/app/app_telemetry.c) 的 1 kHz 任务，把 `snap.gy_dps` 与 `snap.ax_g/ay_g/az_g` 喂给互补滤波，VOFA CH0 改用滤波器输出。
2. **方案 B（最坏情况，仍焊上拉）**：飞线焊两颗 4.7 kΩ 0805 上拉电阻到 PB2/PB3，把 `bsp_imu_i2c.{c,h}` + `mpu6050.{c,h}` + `att_filter.{c,h}` 三套从 git 历史 cherry-pick 回来，UART2 链路保留作"双 IMU 冗余"——但这条路径需热风焊台返工 LaunchPad，按 §1.1 决策放最后。

---

## 11. Stage 1.6 引脚集中化重排（2026-05-08）

### 11.1 触发原因

复核 [LaunchPad User's Guide 图 2-10](../MSPM0G3507%20LaunchPad%20User%27s%20Guide.pdf) BoosterPack 引脚布局后发现：本文 §3 占用的 `IMU_TX = PA21`（LQFP pin 17）**未引到任何一个 BoosterPack 排针**——LaunchPad 把 PA21 仅留给"VREF- 模式"占位（板上 R20 DNC，本工程不用外部 VREF 故空置），可达性仅限板底 J23-J28 引脚扩展接头。如要把 IMU TX 接到外部 MS901M 模块，**必须在 LaunchPad 上焊接**——这与本工程"避免热风焊台修复风险"的 Stage 1.5 立项初衷相违，等同于回到 PB2/PB3 加上拉电阻的同等代价。

同时，`PWMB = PA9`（LQFP pin 55）默认通过 J14 跳线接到 PB23 而非 PA9，需要把 J14 从 (1)-(2) 切到 (2)-(3) 才能把 PA9 接到 BP J1.3。这一项不需焊接、只需切换跳线，但 [docs/Overview/pin.md](../Overview/pin.md) 第 56-69 行的"开放引脚表"未体现 J14 的可切换性，容易让人误以为 PA9 不可用。

### 11.2 决策摘要（与项目负责人 2026-05-08 确认）

| 决策项 | 结论 | 理由 |
|--------|------|------|
| IMU UART 实例 | **UART2 → UART3** | UART3 + PB12/PB13 二者均在 BP（J4.32 + J2.26），且为单 pad（PINCM29/PINCM30），无 SDK multi-pad codegen 风险 |
| 蓝牙模块 | **HC-04 整体下线**（释放 UART3 + PB12/PB13）| UART3 让给 IMU；蓝牙是 Stage 1 引入的"无线姿态可视化"路径，本阶段 K230 视觉链尚未联调，可视化暂时改走 1 Hz XDS-UART printf 不影响整车功能 |
| PWMB 接出 | J14 切到 (2)-(3)，让 PA9 → BP J1.3 | 避免 PA9 焊接，整车装配只剩 PA0/PA1 两个不可避免焊点 |
| VOFA+ 路径 | 100 Hz JustFloat 二进制流暂停；保留 vofa.{c,h} 模块接口 | 二进制流与 printf 文本日志混用同一 XDS-UART 会让 VOFA+ 上位机分帧失败；未来无线路径回归（K230 BLE / USB CDC）后只需重新注入 writer，业务代码无须改动 |
| 焊接清单 | 仅 **PA0 (BUZZER)** + **PA1 (LASER_EN)** 两脚必须焊接 | LaunchPad 把 PA0/PA1 仅引到板载 LED1 跳线柱与开漏上拉跳线柱，没有引到任何 BP 排针；从 J4.1/J20.1 跳线柱直接飞线即可，无需热风焊台 |
| 备选方案保留 | 若蓝牙必须回归，优先 K230 BLE 透传或 USB CDC，不再占主控 UART | 主控 UART2 单 pad 选项 PA21 仍需焊接，PB15/PB16 是否触发 SDK multi-pad bug 未实测验证 |

### 11.3 引脚 diff（vs Stage 1.5）

释放：
- `PA21 (IMU_TX, UART2_TX, PINCM46)` —— 退回 §3.3 预留池，未来扩展 GPIO 仍需考虑焊接代价
- `PA22 (IMU_RX, UART2_RX, PINCM47)` —— 退回 §3.3 预留池（BP J3.14 仍可用作扩展）
- `UART2` 实例 —— 整体从 SysConfig 下线
- `PB12 (BT_TX, UART3_TX)` / `PB13 (BT_RX, UART3_RX)` —— 蓝牙下线后释放，**当 tick 即被 IMU 重新占用**
- `UART3` 蓝牙实例 —— 删 `UART_BT` addInstance；`bsp_bt_uart.{c,h}` git rm

新占用：
- `PB12 (PINCM29, 单 pad)` → `UART3_TX`（IMU），主控 → MS901M（发配置/校准命令）
- `PB13 (PINCM30, 单 pad)` → `UART3_RX`（IMU），MS901M → 主控（业务接收）
- `UART3` 重新 addInstance 为 `UART_IMU`（实例名复用 Stage 1.5 的 `UART_IMU`，仅 peripheral.$assign / txPin / rxPin 三行变化）

跳线变化：
- `J14`：(1)-(2) PB23 → **(2)-(3) PA9**，让 PWMB 从 BP J1.3 直接接出

### 11.4 SysConfig 改动（替代 §5）

[`EIDE/LP_MSPM0G3507.syscfg`](../../EIDE/LP_MSPM0G3507.syscfg) 的最终 diff（vs Stage 1.5 v0.7）：

```diff
 const IMU_UART = UART.addInstance();
 IMU_UART.$name                             = "UART_IMU";
 IMU_UART.targetBaudRate                    = 115200;
 IMU_UART.enableFIFO                        = true;
 IMU_UART.enabledInterrupts                 = ["RX"];
-IMU_UART.peripheral.$assign                = "UART2";
-IMU_UART.peripheral.txPin.$assign          = "PA21";
-IMU_UART.peripheral.rxPin.$assign          = "PA22";
+IMU_UART.peripheral.$assign                = "UART3";
+IMU_UART.peripheral.txPin.$assign          = "PB12";
+IMU_UART.peripheral.rxPin.$assign          = "PB13";

-const BT_UART = UART.addInstance();
-BT_UART.$name                              = "UART_BT";
-BT_UART.targetBaudRate                     = 115200;
-BT_UART.enableFIFO                         = true;
-BT_UART.enabledInterrupts                  = ["RX"];
-BT_UART.peripheral.$assign                 = "UART3";
-BT_UART.peripheral.txPin.$assign           = "PB12";
-BT_UART.peripheral.rxPin.$assign           = "PB13";
```

注意：
- 重新生成 `ti_msp_dl_config.{c,h}` 后，`UART_IMU_INST` 宏指向 `UART3`，`UART_IMU_INST_IRQHandler` 展开为 `UART3_IRQHandler`；[`bsp_imu_uart.c`](../../template/hardware/bsp_imu_uart.c) 已把 IRQ 函数名从 `UART2_IRQHandler` 改为 `UART3_IRQHandler`，与启动文件向量表一致。
- `UART_BT_INST` / `UART_BT_INST_IRQHandler` 等宏在重生后会消失，编译期任何残留 `bsp_bt_uart_*` 调用都会报错；本次改动已把 [`main.c`](../../template/main.c) 的 `bsp_bt_uart_init()` 与 [`app_telemetry.c`](../../template/app/app_telemetry.c) 的 `vofa_set_writer(bsp_bt_uart_write)` 与 100 Hz `vofa_send` 全部删除。

### 11.5 文件改动清单（vs Stage 1.5 v0.1）

| 路径 | 动作 | 说明 |
|------|------|------|
| [`EIDE/LP_MSPM0G3507.syscfg`](../../EIDE/LP_MSPM0G3507.syscfg) | 改 | UART_IMU 改 UART3+PB12/PB13；删 UART_BT 整段（含 §5 历史注释） |
| [`template/hardware/bsp_imu_uart.{h,c}`](../../template/hardware/bsp_imu_uart.h) | 改 | 头部注释 UART2→UART3；IRQ 函数名 `UART2_IRQHandler` → `UART3_IRQHandler` |
| [`template/hardware/bsp_bt_uart.{h,c}`](../../template/hardware/bsp_imu_uart.h) | **删除** | git rm，历史可回退；蓝牙下线 |
| [`template/app/app_telemetry.{h,c}`](../../template/app/app_telemetry.h) | 改 | 删 `bsp_bt_uart.h` 引入、`vofa.h` 引入、`PHASE_VOFA_TICKS` 宏、`vofa_set_writer` 调用、100 Hz `vofa_send` 调用；1 Hz printf 心跳保留并升级为姿态可视化主路径 |
| [`template/main.c`](../../template/main.c) | 改 | 删 `bsp_bt_uart.h` 引入、`bsp_bt_uart_init()` 调用；启动 banner 改 `stage1.6 telemetry start (MS901M / no BT)` |
| [`EIDE/.eide/eide.yml`](../../EIDE/.eide/eide.yml) | 改 | `hardware` 文件列表删 `bsp_bt_uart.{c,h}` |
| [`docs/TaskLog/Stage0-PinAllocation.md`](Stage0-PinAllocation.md) | 改 | 升 v0.8：§1 决策行 / §2 跳线 J14 / §3.2 业务表 IMU 引脚 + 删 BT 行 / §3.3 预留 / §3.4 焊接清单 / §4.3 IMU 详表 / §4.5 蓝牙下线 / §5.6 验证清单 / §7 修订历史 |
| 本文 | 改 | 头部 banner + 关联文档版本号 + 新增 §11 章；§1-§10 中 UART2/PA21/PA22 字样保留作历史，加文档头部说明 |
| [`docs/TaskLog/Stage1-IMU-BT-Telemetry.md`](Stage1-IMU-BT-Telemetry.md) | 改 | 头部 banner 追加 1.6 蓝牙下线提示 |

### 11.6 验收追加项（在 §9 基础上）

| 项 | 通过条件 | 验证方式 | 状态 |
|---|---------|---------|------|
| 工程能编译且无未定义引用 | 删除蓝牙后 `bsp_bt_uart_*` / `UART_BT_INST` 等符号在主控代码中零引用，AC6 链接器无 undefined reference | EIDE 构建按钮 + map 文件 grep | [x] |
| `ti_msp_dl_config.h` 正确生成 UART3 IMU 宏 | grep `UART_IMU_INST` 指向 `UART3`、`UART_IMU_INST_IRQHandler` 展开为 `UART3_IRQHandler` | 文本检查 | [x] |
| MS901M 在 PB12/PB13 在线 | 上电后 USB-TTL 监听 BP J2.26 (PB13) 应见 `0x55 0x55 ...` 周期帧；XDS-UART 不再出现 fatal | 串口工具 + XDS-UART | [x] |
| 1 Hz 心跳承载姿态可视化 | XDS-UART 文本日志中 pitch/roll/gy/temp 数值随手动倾斜模块变化；`ms901m_good` 持续递增、`bad=0`、`over=0` | XDS-UART | [x] |
| 蓝牙模块物理拆除 | HC-04 已从主控板上拔除；BP J4.32 (PB12) 与 J2.26 (PB13) 让位给 MS901M | 装车前目检 | [ ] |
| PWMB 通过 J14 接出 | 万用表蜂鸣档点测：PA9 (LQFP 55) ↔ J1.3 通；PB23 ↔ J1.3 不通 | 万用表 | [ ] |
| 焊接清单仅剩 2 项 | 整车装配后，飞线统计：PA0(BUZZER) + PA1(LASER_EN) 共 2 路，无其他业务信号需焊接 | 装配检查表 | [ ] |

---

## 12. 首次实测踩坑与修复（2026-05-08）

> 本节记录 Stage 1.6 固件首次烧录实测中暴露的两个 bug，及其修复过程与根因分析。
> 属于"测试驱动发现"的经验资产，**比代码改动本身更有长期参考价值**。

### 12.1 Bug A：UART RX 中断到达 FIFO 但 ISR 永不触发 → `[FATAL] ms901m boot timeout`

**现象**：构建通过、下载成功、XDS-UART 打印 banner，随即出现：

```
[boot] MSPM0G3507 stage1.6 telemetry start (MS901M / no BT)
[FATAL] ms901m boot timeout rc=-1
```

**根因**：MSPM0G3507 SDK 2.10 的 `SYSCFG_DL_UART_IMU_init()`（SysConfig 自动生成）仅调用 `DL_UART_Main_enableInterrupt()` 设置**外设级 IMSC 寄存器**（让 UART3 模块在 RX FIFO 半满时声明中断线电平），但**不调用 `NVIC_EnableIRQ()`**（让 Cortex-M0+ 内核 NVIC 监听这条 IRQ 线）。

两者缺一不可：

| 层次 | 寄存器 | 作用 | 谁来设置 |
|------|--------|------|---------|
| UART 外设级 | IMSC | 触发条件 → IRQ 线 | SysConfig 生成的 `SYSCFG_DL_UART_IMU_init` |
| ARM 内核级 | NVIC ISER | IRQ 线 → CPU 调度 ISR | **用户代码必须显式调用** |

原 `bsp_imu_uart_init()` 注释写"SDK 在 SYSCFG_DL_UART_IMU_init 里已经使能 RX 中断 + NVIC，无需再开"——这条假设**错误**。结果是 MS901M 数据帧完整送达 UART3 FIFO，FIFO 半满标志置位，但 `UART3_IRQHandler` 永远得不到 CPU 调度 → 环缓 `s_rx_head` 不前进 → `ms901m_has_attitude()` 永远 false → 上电等待窗口超时触发 fatal。

**修复**：`bsp_imu_uart_init()` 加一行：

```c
NVIC_EnableIRQ(UART_IMU_INST_INT_IRQN);   /* 当前展开为 UART3_INT_IRQn */
```

`UART_IMU_INST_INT_IRQN` 是 SysConfig 生成的宏，外设号变更时自动跟随。参考对照：`bsp_k230_uart.c:bsp_k230_uart_init()` 早就正确调用了 `NVIC_EnableIRQ(DMA_INT_IRQn)`。

**经验规则（后续所有 BSP 驱动必须遵守）**：
> SDK 2.10 SysConfig 生成的外设初始化函数**只设置外设级中断 mask，不开 NVIC**。
> 凡需要 ISR 的外设（UART RX、DMA 完成、定时器 OVF 等），BSP init 函数里**必须**显式调用 `NVIC_EnableIRQ(<INST>_INT_IRQN)`。

---

### 12.2 Bug B：printf 浮点格式化栈溢出 → 进入主循环后无后续输出

**现象**：修复 Bug A 后重新烧录，出现：

```
[boot] MSPM0G3507 stage1.6 telemetry start (MS901M / no BT)
[boot] MS901M attitude online, 4 good / 0 bad frames
（此后永久无输出；绿灯不翻转）
```

MS901M 已经在线（4 good 帧），主循环却像死了一样——绿灯 5 Hz 翻转没出现，1 Hz `[hb]` 心跳也没出现。

**根因**：双因素叠加：

| 因素 | 数值 | 危险点 |
|------|------|--------|
| 启动文件栈大小 | `Stack_Size EQU 0x100`（**256 B**）| Cortex-M0+ 默认值，极保守 |
| 1 Hz 心跳一次 printf 4 个 `%.2f / %.1f` | 每个浮点格式化栈帧 ~200 B | 共计 > 800 B，4 倍于栈上限 |

启动期 banner 只用 `%lu` 整数格式化（栈帧 < 50 B），正常输出。进入主循环后，`tick_count == 1000` 的瞬间调用含 4 个 `%f` 的 printf → 栈从顶向下写穿 → 踩到 NVIC 向量表以下的内存 → HardFault → CPU 跳进默认死循环 → 主循环永不返回。

绿灯翻转发生在 `tick_count == 200`（5 Hz 分支），而 printf 崩溃在 `tick_count == 1000`（1 Hz 分支）——实测中绿灯**出现过 5 次翻转**，正好证明主循环在 0~999 ms 内活着，第 1000 ms 崩。（用户报告"无后续报文"时绿灯状态未记录，后续遇到类似现象可检查 LED 辅助定位崩溃点。）

**修复**：

1. **栈扩到 1 KB**：`startup_mspm0g350x_uvision.s` 改 `Stack_Size EQU 0x00000400`，覆盖未来 snprintf / 浮点数学等潜在栈高峰。
2. **printf 改整数化**：引入 `F2_X100 / F2_S / F2_I / F2_F` 四个宏，把 float 先 ×100 四舍五入为 `int32_t`，再用 `%c%ld.%02lu` 整数格式化输出。不再链接浮点格式化路径，栈占用从 > 800 B 降至 < 80 B，附带节省 ~3 KB ROM。

宏实现关键点：先把 `v` 整体 ×100 四舍五入到一个 `int32_t`，再统一拆整数/小数。这样 `99.995 → 10000 → " 100.00"` 不会出现独立计算时的 `" 99.100"` 跨整数边界错误。

**经验规则（后续所有 printf 调用必须遵守）**：
> Keil AC6 标准库 `printf("%f")` 单次栈帧 200~300 B，Cortex-M0+ 栈空间一般只有几百 B。
> **嵌入式项目中应完全禁用 `%f / %e / %g`**，改用 `×100` 整数化宏或 `snprintf` 分段截断。
> 若必须用浮点格式化，链接选项加 `--float` 并把栈至少开到 2 KB。

---

## 13. 修订历史

| 日期 | 版本 | 修订内容 | 作者 |
|------|------|---------|------|
| 2026-05-07 | v0.1 | 初版：MPU6050 → MS901M 元件替换全程落档；含决策摘要、元件对比、引脚迁移、SysConfig 改动、解析器移植差异、VOFA 通道重映射、ATK 上位机一次性配置、验收清单、风险与回退路径 | 主控团队 |
| 2026-05-08 | v0.2 | **追加 §11 Stage 1.6 引脚集中化重排**：① IMU UART 由 UART2/PA21/PA22 迁到 UART3/PB12/PB13；② 蓝牙 HC-04 整体下线；③ PWMB 通过 J14 切换；④ VOFA+ 暂停改走 1 Hz printf；⑤ 焊接清单收敛到 PA0+PA1 | 主控团队 |
| 2026-05-08 | v0.3 | **追加 §12 首次实测踩坑复盘**：Bug A（UART NVIC 未开→IMU fatal）+ Bug B（printf %f 栈溢出→主循环无输出）；修复：`bsp_imu_uart_init` 加 `NVIC_EnableIRQ`、栈 256 B→1 KB、printf 改整数化宏 `F2_X100/F2_S/F2_I/F2_F`；提炼两条项目级经验规则 | 主控团队 |
