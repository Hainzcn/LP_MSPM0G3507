# 阶段 1.5 ｜ IMU 元件替换：MPU6050 → ATK-MS901M

> 文档定位：临时阶段，记录 IMU 链路从 I²C+MPU6050 切换到 UART+ATK-MS901M 的全部变更——硬件原因、引脚迁移、UART2 实例化、C 版解析器移植、VOFA 通道重映射、上位机一次性配置流程、验收清单。
>
> 关联文档：
>
> - 项目总览：[../Overview/Overview.md](../Overview/Overview.md)
> - 引脚分配真源（已升级到 v0.7）：[Stage0-PinAllocation.md](Stage0-PinAllocation.md)
> - 阶段 1（被本文部分替代，蓝牙/K230 部分仍有效）：[Stage1-IMU-BT-Telemetry.md](Stage1-IMU-BT-Telemetry.md)
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
- **上电首帧延迟**：MS901M 内部 EKF 收敛需要 200~500 ms，500 ms 上电检测窗口与之边界重合；若实测 fatal handler 偶发触发，把 [`template/main.c`](../../template/main.c) 的 `MS901M_BOOT_TIMEOUT_MS` 调到 1000~2000。

### 10.2 回退路径（若 MS901M 链路不可用）

如果实测 0x01 pitch 受电机 PWM 干扰严重（超出预期），不需要回退到 MPU6050（PB2/PB3 上拉问题仍在），而是：

1. **方案 A（推荐）**：保持 UART2 链路，但姿态来源切换到 0x03 帧的 raw gyro+accel，在主控重新做互补滤波。从 git 历史拉回 [`att_filter.{c,h}`](../../template/middle/att_filter.h)（commit ID 见本次 Stage 1.5 提交的 parent）；改 [`app_telemetry.c`](../../template/app/app_telemetry.c) 的 1 kHz 任务，把 `snap.gy_dps` 与 `snap.ax_g/ay_g/az_g` 喂给互补滤波，VOFA CH0 改用滤波器输出。
2. **方案 B（最坏情况，仍焊上拉）**：飞线焊两颗 4.7 kΩ 0805 上拉电阻到 PB2/PB3，把 `bsp_imu_i2c.{c,h}` + `mpu6050.{c,h}` + `att_filter.{c,h}` 三套从 git 历史 cherry-pick 回来，UART2 链路保留作"双 IMU 冗余"——但这条路径需热风焊台返工 LaunchPad，按 §1.1 决策放最后。

---

## 11. 修订历史

| 日期 | 版本 | 修订内容 | 作者 |
|------|------|---------|------|
| 2026-05-07 | v0.1 | 初版：MPU6050 → MS901M 元件替换全程落档；含决策摘要、元件对比、引脚迁移、SysConfig 改动、解析器移植差异、VOFA 通道重映射、ATK 上位机一次性配置、验收清单、风险与回退路径 | 主控团队 |
