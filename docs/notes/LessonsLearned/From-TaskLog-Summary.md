# TaskLog 踩坑经历汇总

本文从 [`docs/TaskLog`](../../TaskLog/) 各阶段文档中提取**已验证的踩坑记录**，按主题归类，并给出项目级防护规则。详细现场描述、代码 diff 与验收清单仍以 TaskLog 原文为准。

---

## 索引（按严重级别）

| 级别 | 主题 | TaskLog 来源 | 专文 |
|------|------|--------------|------|
| 致命 | `printf("%f")` 栈溢出 | Stage1.5 §12.2；Stage3 心跳约定；2026-05 circle demo 复发 | [StackOverflow-Printf-StateCorruption.md](./StackOverflow-Printf-StateCorruption.md) |
| 致命 | ISR 向量名错误 → 假死 | Stage2 §2.4（v0.6） | 下文 §2 |
| 致命 | UART RX 未开 NVIC → IMU 超时 fatal | Stage1.5 §12.1 | 下文 §3 |
| 高 | SDK multi-pad GPIO codegen 不可用 | Stage1 §8、§8.5；Stage0 §4.7 | 下文 §4 |
| 高 | 编码器 X4 + 高 PPR → ISR 雪崩 | Stage3 §6.1 | 下文 §5 |
| 高 | ADC 未配 sampleTime → batt=0 误急停 | Stage2 §2.4（v0.6） | 下文 §6 |
| 中 | LaunchPad 跳线未核对 → 电机/方向异常 | Stage2 §7.5 | 下文 §7 |
| 中 | 速度环极性反了 → PID 立刻发散 | Stage3 §7.1 | 下文 §8 |
| 中 | 高分辨率编码器未量纲归一 → 速度环饱和 | Stage3 §7.2 | 下文 §9 |
| 中 | K230 DMA 通道宏默认值错误 | Stage4 §3.2 | 下文 §10 |
| 规划 | 引脚纸面可行 ≠ 可焊接 / 可 codegen | Stage0 v0.8；Stage1.5 §11 | 下文 §11 |

---

## 1. printf 浮点格式化与栈溢出

### 第一次（2026-05-08，Stage 1.6）

- **现象**：MS901M 已 online，boot banner 正常，进入主循环后无 `[hb]`、绿灯停止翻转。
- **根因**：栈仅 256 B；1 Hz 心跳含 4 个 `%.2f/%.1f`，AC6 `_printf_fp_dec` 单次栈帧约 200 B，合计 >800 B → HardFault / 死循环。
- **修复**：栈 256 B→1 KB；引入 `F2_X100/F2_S/F2_I/F2_F` 整数化宏，心跳改 `%c%ld.%02lu`。
- **来源**：[Stage1.5-IMU-Swap-MS901M.md §12.2](../../TaskLog/Stage1.5-IMU-Swap-MS901M.md)

### 第二次（2026-05，Stage 2.2 + circle demo）

- **现象**：心跳 `state=?`；`k230_rx` 速率异常；`s_state` 被写成 SRAM 指针值。
- **根因**：boot 路径 `print_pid_help()` / `app_circle_demo` 重新引入 `%f`；1 KB 栈底与 `.bss` 零间隙，溢出约 24 B 即写脏 `s_state`、`s_total_rx`。
- **修复**：消除全部运行时 `%f`；栈 1 KB→2 KB；保留 canary + enum 哨兵。
- **专文**：[StackOverflow-Printf-StateCorruption.md](./StackOverflow-Printf-StateCorruption.md)

### 项目规则（强制）

1. **禁止**在 boot / 心跳 / 1 kHz 路径使用 `printf("%f")`、`%.2f`、`%.1f`。
2. 浮点展示统一用整数缩放宏：`BAL_F2_*`（2 位小数）、`CIRC_F1_*`（1 位小数）。
3. 新增 `printf` 前 grep 全仓库 `%f`；看 `.map` 中 STACK 与 `.bss` 边界。
4. 栈大小见 `template/keil/startup_mspm0g350x_uvision.s` 注释链（当前 **2 KB**）。

---

## 2. GPIO 中断向量名错误（假死，非雪崩）

### 现象

手转右轮瞬间 MCU 整体卡死，像宕机。

### 根因

早期 `bsp_motor.c` 将 ISR 命名为 `GPIOA_IRQHandler`。MSPM0G3507 上 **GPIOA + GPIOB 共享 IRQn=1（GROUP1）**，向量表入口必须是 **`GROUP1_IRQHandler`**。错误命名只是普通符号，永不挂到 vector；编码器沿触发后 NVIC 跳进 startup 里 weak 默认 `B .` 死循环。

> 易误判为 ISR 雪崩或浮空噪声——实际根因是**向量名**。

### 修复与附加防护

- ISR 改名为 `GROUP1_IRQHandler`，按 pin index 分发。
- ENC 引脚内部上拉 + Hysteresis；SysTick 优先级高于 GPIO；200 edges/ms 雪崩兜底；`HardFault_Handler` → `NVIC_SystemReset()`。
- **来源**：[Stage2-MotorDrive-Encoder.md](../../TaskLog/Stage2-MotorDrive-Encoder.md)（Stage 2.4 / v0.6 提示框）

### 规则

> 写 ISR 前对照 startup / SDK 例程中的**确切 handler 名**，不能凭 STM32 习惯臆造 `GPIOx_IRQHandler`。

---

## 3. SysConfig 开了外设中断但未开 NVIC

### 现象

`[FATAL] ms901m boot timeout`；UART FIFO 有数据但环缓不前进。

### 根因

SDK 2.10 生成的 `SYSCFG_DL_UART_*_init()` 只设 UART 外设 IMSC，**不调用 `NVIC_EnableIRQ()`**。`bsp_imu_uart_init()` 误以为 SDK 已全开。

### 修复

```c
NVIC_EnableIRQ(UART_IMU_INST_INT_IRQN);
```

对照：`bsp_k230_uart_init()` 对 DMA  IRQ 的做法是正确的。

- **来源**：[Stage1.5 §12.1](../../TaskLog/Stage1.5-IMU-Swap-MS901M.md)

### 规则

> 凡依赖 ISR 的外设（UART RX、DMA 完成、定时器 OVF），BSP `init()` **必须**显式 `NVIC_EnableIRQ`。

---

## 4. SDK 2.10 multi-pad 引脚 codegen

### 现象

- SysConfig 对 UART2 + PB17/PB16 报 codegen 错误（`undefined` → `.match` 崩溃）。
- GPIO 模块对 PB22/PB26 等生成 `GPIOC.*`，LQFP-64 无 GPIOC → 编译失败。

### 根因

MSPM0G350x 上 PB16/PB17、PA15/16/26/27、PB22/26/27 等为 **multi-pad bonded pin**。GPIO codegen 在无 instance 级 `port` 锁定时，`_PORT/_PIN` 与 `_IOMUX` 可能不一致。

### 修复策略

| 引脚类型 | 做法 |
|----------|------|
| 业务 GPIO（LED/S1/STBY 等 14 脚） | **不走 SysConfig**，统一 [`bsp_gpio.{c,h}`](../../../template/hardware/bsp_gpio.h) |
| Peripheral（UART/PWM/QEI/ADC） | 走 syscfg + `peripheral.txPin.$assign`，用单 pad 引脚（PINCM29/30 等） |
| 若必须用 multi-pad GPIO | instance 设 `port = "PORTA/B"` + `assignedPin = "<bit>"` |

- **来源**：[Stage1-IMU-BT-Telemetry.md §8、§8.5](../../TaskLog/Stage1-IMU-BT-Telemetry.md)；[Stage0-PinAllocation.md §4.7](../../TaskLog/Stage0-PinAllocation.md)

### 规则

> 引脚分配 = **datasheet 核对 + SysConfig 试生成** 双确认；候选引脚不得只写进表格未 codegen 验证。

---

## 5. 右轮 X4 解码 + 高 PPR 编码器 → ISR 雪崩

### 现象

换新电机（500 PPR × 34:1）后平衡环崩溃；右轮速度瞬间归零。

### 根因

180 RPM 出轴时 PA12/PA13 双沿 X4 约 **204 edges/ms**，超过雪崩阈值 200 → `bsp_motor` 关中断 50 ms → 右轮无速度反馈。

旧电机（11 PPR × 9.6:1）仅 ~2 edges/ms，X4 完全安全——**同一套 X4 代码换电机后失效**。

### 修复

- `BSP_MOTOR_RIGHT_DECODE_X`：4 → **2**（102 edges/ms @180 RPM）。
- 雪崩阈值 200 → **300**；注释记录 X4 不可用根因。

- **来源**：[Stage3-BalanceControl.md §6.1](../../TaskLog/Stage3-BalanceControl.md)

### 规则

> 改 PPR / 减速比 / 解码倍率后，必须重算 `peak_edge_rate` 与 `BSP_MOTOR_ENC_IRQ_QUENCH_PER_MS` 余量。

---

## 6. ADC 未配采样窗口 → 电池误保护

### 现象

`batt=0mV`，安全态锁在 `BAT_STOP`。

### 根因

`ADC_BAT` 未设 `sampleTime0`，转换永不完成，读数为 0；`classify()` 把 0 mV 当低压急停。

### 修复

- syscfg + `DL_ADC12_setSampleTime0`；
- `BSP_BATTERY_DISCONNECTED_MV`（默认 1000 mV）→ 未接电池返回 `UNKNOWN`，不进 `LOW_STOP`。

- **来源**：[Stage2 §2.4](../../TaskLog/Stage2-MotorDrive-Encoder.md)

---

## 7. LaunchPad 跳线（装车必查）

文档 Stage2 §7.5 列出 15 项跳线；最易踩的两项：

| 跳线 | 必须状态 | 不到位后果 |
|------|----------|------------|
| **J17/J18 OFF** | OFF | BIN1/BIN2 与板载 OPA 抢线，TB6612 方向位错乱 |
| **J14 (2)-(3) PA9** | 接 PWMB 到 BP J1.3 | 默认 PB23 时右电机不转 |

- **来源**：[Stage2 §7.5](../../TaskLog/Stage2-MotorDrive-Encoder.md)；[Stage0 §2 跳线表](../../TaskLog/Stage0-PinAllocation.md)

---

## 8. 速度环反馈极性反了

### 现象

注入任意 `sp` Kp 后小车立刻发散，无法整定。

### 根因

编码器 `avg_cps` 符号与「正 PWM = 前进」不一致 → 速度环变正反馈。

### 修复

`APP_BALANCE_SPEED_INVERT` + 串口 `si0`/`si1`/`si?`；改极性后 reset 速度 PID 与 LPF。

- **来源**：[Stage3 §7.1](../../TaskLog/Stage3-BalanceControl.md)

### 规则

> 速度环整定前先用 `si?` 确认 `v_meas` 与前进方向同号。

---

## 9. 高分辨率编码器 + 速度 PID 量纲

### 现象

`sp 1`（Kp=0.001）即产生约 5° 倾角指令，速度环饱和振荡。

### 根因

500×34 编码器下 0.1 rev/s 漂移 ≈ 5100 cps，未缩放直接进 PID。

### 修复

`APP_BALANCE_SPEED_CPS_SCALE = 10`，`avg_cps / 10` 后再进速度环。

- **来源**：[Stage3 §7.2](../../TaskLog/Stage3-BalanceControl.md)

---

## 10. K230 通讯侧踩坑

| 问题 | 根因 / 修复 | 来源 |
|------|-------------|------|
| DMA RX 通道号不对 | fallback 默认 0，实际通道 1；新增 `DMA_CH_UART_K230_DMA_RX_CHAN` | Stage4 §3.2 |
| 心跳 CRC 不一致 | 表初始数值转录错误（非算法错）；bad 统计保留追踪 | Stage4 联调记录 |
| 921600 发热 | 降为 115200，协议流量 <1 kB/s 足够 | Stage4 v0.5 |
| MCU 离线指令残留 | MCU 500 ms 超时 + K230 侧也应主动发 v=0 | Stage4-K230-Side |

- **来源**：[Stage4-K230-Communication.md](../../TaskLog/Stage4-K230-Communication.md)

---

## 11. 引脚规划：纸面 / 焊接 / codegen 三角

Stage 0→1.6 多次重排总结的元规则：

1. **BoosterPack 排针可达** > 单 pad > 免焊接；PA21 不在 BP 上 → IMU 最终迁 UART3/PB12/PB13。
2. **蓝牙下线**是为给 IMU 让 UART3，而非功能废弃；调试改 1 Hz printf。
3. I²C MPU6050 路线因 **缺上拉、开路线** 整体换成 MS901M UART（Stage 1.5）。
4. 改引脚流程：先 [Stage0-PinAllocation.md](../../TaskLog/Stage0-PinAllocation.md) → `bsp_gpio.h` 或 syscfg → **codegen 试编译**。

---

## 12. 调试方法论（TaskLog 共性）

| 症状 | 优先怀疑 | TaskLog 先例 |
|------|----------|--------------|
| boot 有输出、进循环后静默 | 栈溢出 / HardFault | §12.2；2026-05 state=? |
| 外设 FIFO 有数据、环缓不动 | NVIC 未开 | §12.1 |
| 手转轮子立刻全机假死 | ISR 向量名错 | Stage 2.4 |
| 高速时某一侧速度变 0 | ENC ISR 雪崩 | Stage 3 §6.1 |
| `state=?` / 计数器离谱 | 栈写 `.bss` 或 buffer 溢出 | 2026-05 + map |
| SysConfig 一改就崩 | multi-pad GPIO | Stage 1 §8 |
| PID 怎么调都发散 | 极性 / 量纲 / 正反馈 | Stage 3 §7 |

**推荐工具链**：XDS-UART 日志 → `.map` 内存布局 → LED 5 Hz vs 1 Hz 分支定位崩溃时刻 → grep `%f` / `NVIC_EnableIRQ` / `GROUP1_IRQHandler`。

---

## 13. TaskLog 原文索引

| 文件 | 主要踩坑章节 |
|------|----------------|
| [Stage0-PinAllocation.md](../../TaskLog/Stage0-PinAllocation.md) | §2 跳线；§4.7 GPIO BSP 化；§4.5 蓝牙下线；multi-pad 历史 |
| [Stage1-IMU-BT-Telemetry.md](../../TaskLog/Stage1-IMU-BT-Telemetry.md) | §8 UART multi-pad；§8.5 GPIO codegen；§8.6 PT_LOAD 对齐 |
| [Stage1.5-IMU-Swap-MS901M.md](../../TaskLog/Stage1.5-IMU-Swap-MS901M.md) | **§12 首次实测踩坑（NVIC + printf 栈）**；§11 引脚重排 |
| [Stage2-MotorDrive-Encoder.md](../../TaskLog/Stage2-MotorDrive-Encoder.md) | **§2.4 GROUP1 + ADC**；§7.5 跳线；§2.6 右轮 5% 补偿 |
| [Stage3-BalanceControl.md](../../TaskLog/Stage3-BalanceControl.md) | **§6.1 X4 雪崩**；§7 极性/量纲；§6.4 dither 死区 |
| [Stage4-K230-Communication.md](../../TaskLog/Stage4-K230-Communication.md) | DMA 通道；CRC；波特率 |
| [Stage4-K230-Side.md](../../TaskLog/Stage4-K230-Side.md) | K230 离线归零语义 |

---

## 14. 与本目录其他文档的关系

- **栈溢出专文**（含 map 定案、canary 策略）：[StackOverflow-Printf-StateCorruption.md](./StackOverflow-Printf-StateCorruption.md)
- **架构纵览**（模块职责）：[../StudyNotes/ProjectArchitecture.md](../StudyNotes/ProjectArchitecture.md)
- **PID 整定**（非踩坑，但常与 §8–§9 联动）：[../StudyNotes/PIDTuningGuide.md](../StudyNotes/PIDTuningGuide.md)
