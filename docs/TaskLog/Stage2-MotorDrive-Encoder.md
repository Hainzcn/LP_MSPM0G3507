# 阶段 2 ｜ TB6612 双电机驱动 + 编码器角度日志

> **⚠️ Stage 2.1 BSP 重写提示（2026-05-09）**：[bsp\_motor.{c,h}](../../template/hardware/bsp_motor.h) 在本版次完成**驱动 API 重写**，从原本 7 个最小接口扩展为 8 组共 18 个 API（单轮命令、Coast vs Brake、运行时极性翻转、PWM 上限保护、编码器原始读 / 归零、瞬时速度 cps/dps/rpm 反馈、命令查询、S1 toggle）。**演示层 [app\_motor\_demo.c](../../template/app/app_motor_demo.c) 完全向后兼容**，原有 5 个调用 (`bsp_motor_enable / set_output / consume_toggle_request / update / get_feedback`) 行为不变；`bsp_motor_feedback_t` 仅追加字段不删字段。同期废弃：原顶部宏 `BSP_MOTOR_LEFT_SIGN / RIGHT_SIGN` 不再存在，方向反转改为运行时 `bsp_motor_set_invert()`。详见 §3 全 API 表 与 §9 修订历史 v0.2。
>
> **⚠️ 引脚核对提示（2026-05-09，本文 v0.3）**：本文从分支 A 迁回时引脚信息基于 Stage 0 v0.6 旧版，本分支 Stage 1.6（Stage0 真源 v0.8）已做"引脚集中化重排 + 跳线决策更新"。对照 [Stage0-PinAllocation.md §3.2](Stage0-PinAllocation.md) / [bsp_gpio.h](../../template/hardware/bsp_gpio.h) / [ti_msp_dl_config.h](../../EIDE/ti_msp_dl_config.h) 三处真源逐脚核对结果：**电机 / 编码器 / S1 / LED 等 14 个引脚映射完全一致，无任何错位**；但原文档 §7 接线指导**漏列了 9 个关键 LaunchPad 跳线决策**（J4 / J8 / J12 / J14 / J15 / J17 / J18 / J19 / J20）以及"PA0/PA1 必焊、其余业务脚 BoosterPack 直接接出"的施工指引。本版次 §7 全部表格补 `LQFP 引脚号` 列与 `跳线 / 备注` 列、§7.5 重写为完整跳线核对清单，与真源 Stage 0 §2 / §3.4 一一对应；底层代码 / 业务逻辑无改动。详见 §9 修订历史 v0.3。
>
> **🚀 Stage 2.2 上车准备就绪（2026-05-09，本文 v0.4）**：完成 §8 列出的 5 项后续 TODO，新增 4 个模块共 8 个文件，**已就位但暂未加入 EIDE 构建链**（避免破坏当前 telemetry 基线）：① [bsp\_motor.{c,h}](../../template/hardware/bsp_motor.h) 右轮升 X4 解码（编译期 `-DBSP_MOTOR_RIGHT_DECODE_X=4` 已为默认，左右轮分辨率统一 1320 cnt/rev）；② [bsp\_motor.{c,h}](../../template/hardware/bsp_motor.h) 新增 `bsp_motor_brake_pulse_ms()` 脉冲式短刹车 API（持续 brake 互斥）；③ 新建 [bsp\_battery.{c,h}](../../template/hardware/bsp_battery.h)（ADC0/PB24 周期采样 + EMA 滤波 + 阈值状态机 + 回滞）；④ 新建 [middle/pid.{c,h}](../../template/middle/pid.h) 通用浮点 PID（位置式 + 抗积分饱和 + 微分项独立滤波）；⑤ 新建 [app\_safety.{c,h}](../../template/app/app_safety.h)（跌倒检测 + 电池保护 + 5 态状态机 + S1 重启）；⑥ 新建 [app\_balance.{c,h}](../../template/app/app_balance.h) 速度外环 + 平衡内环骨架（**所有 PID 增益默认 0**，业务侧 `set_*_gains()` 注入；附详细整定流程注释）。⑦ §3.5 / §4.3 列出全部新模块 API，§6.3 增加上车整定回归矩阵。详见 §9 修订历史 v0.4。
>
> **🔧 Stage 2.3 左编码器迁出 J12 / 改走 BoosterPack（2026-05-09，本文 v0.5）**：板上 J12 排针出厂未焊、PA29/PA30/PB14 三脚也都不在 BP 排针上，导致原 QEI 接线必须先焊接 J12。复核 MSPM0G3507 数据手册 PINMUX 表后将左编码器 PHA/PHB 迁到 BP 上空闲且同列相邻的 `PB15 (BP J4.34, TIMG8_C0 mux f=5, PINCM32)` / `PB16 (BP J4.40, TIMG8_C1 mux f=5, PINCM33)`，IDX 因 GB370 编码器无 Z 相同步省略，QEI 由 3-Pin Mode 降为 2-Pin Mode。**硬件 X4 精度 1320 cnt/rev 不变、无丢脉冲、BSP 层零改动**；同步刷新 [EIDE/LP_MSPM0G3507.syscfg](../../EIDE/LP_MSPM0G3507.syscfg) + [ti\_msp\_dl\_config.{c,h}](../../EIDE/ti_msp_dl_config.h)，§7.2 / §7.5.1 / §7.5.2 全部按新走线刷新。装车收益：J12 保持悬空、不需焊接；左/右编码器接线全在 BP 同侧排针上施工。详见 §9 修订历史 v0.5；引脚真源同步至 [Stage0-PinAllocation.md v0.9](Stage0-PinAllocation.md)。
>
> **🛑 Stage 2.4 关键 Bug 修复 ｜ GROUP1_IRQHandler 命名 + ADC 采样窗 + 引脚噪声防护（2026-05-09，本文 v0.6）**：上车首次 boot 时发现两个致命问题：(A) **`batt=0mV` 始终判 `BAT_STOP`**：根因 = SysConfig 没给 `ADC_BAT` 设置 `sampleTime0`，转换永不完成；同步加 `BSP_BATTERY_DISCONNECTED_MV=1000mV` 软兜底，未接电池保持 `UNKNOWN` 不进 `LOW_STOP`。(B) **手转右轮瞬间宕机**：根因 **不是 ISR 雪崩**，而是 [bsp\_motor.c](../../template/hardware/bsp_motor.c) 早期把 ISR 命名为 `void GPIOA_IRQHandler(void)` —— 但 **MSPM0G3507 vector table 里根本没有 `GPIOA_IRQHandler` 这个名字**！整个 GPIO 中断（GPIOA + GPIOB）共享 IRQn=1 = "GROUP1"，入口名叫 **`GROUP1_IRQHandler`**（参考 SDK `gpio_simultaneous_interrupts` 例程）。原命名只是个普通函数符号、永远不会被链接到 vector，PA12/PA13 沿事件触发后 NVIC 跳到 startup.s 里 weak 默认 `B .` 死循环 → MCU 整体卡死。同时本轮还做了多道附加防护：① ENC 输入引脚 PA12/PA13/PB15/PB16 启用内部上拉 + Hysteresis（防浮空噪声）；② SysTick 优先级 0、GPIOA 优先级 3（防 ISR 饿死节拍）；③ `bsp_motor` 加 ISR 雪崩兜底（200 边沿/ms 触发，禁用 50 ms）；④ [bsp\_systick.c](../../template/hardware/bsp_systick.c) 覆盖 `HardFault_Handler` 为 `NVIC_SystemReset()`（fault 直接复位、boot log 反复刷屏，比"假死"友好得多）。详见 §9 修订历史 v0.6。
>
> **🧭 Stage 2.5 电机测速 / 同步 / 校准服务（2026-05-10，本文 v0.7）**：当前 `main.c` 已接入 [app\_motor\_demo.c](../../template/app/app_motor_demo.c) 作为上电默认入口，保留 `app_balance_run()` 装车入口但暂不依赖板载 S2（LaunchPad `SW2/J15` 默认落在 PA16，与 TB6612 `AIN2` 冲突）。本轮把演示固件从“固定 PWM + S1 正反转”升级为“XDS-UART 可调目标转速 + S1 急刹/启动 + 双轮同步闭环 + PWM 扫描校准”：① 目标转速按 GB370 额定 `620 rpm` 钳位并换算 PWM，串口 `+/-` 或数字回车可在线调速；② S1 改为急刹/启动，PA18 采用双沿 + 上电空闲电平判定 + 轮询兜底，并在日志打印 `raw/active/btn_irq/btn_poll` 诊断；③ 右编码器反馈符号翻正，使同向命令下左右计数同号；④ 新增 `app_motor_demo_set_sync_*` / `get_sync_diag` 双轮同步服务，默认 50 ms 一拍，按 `rpmR-rpmL` 做 PI 差分 PWM 补偿；⑤ 新增 `app_motor_demo_cal_*` PWM 正/反向扫描校准，串口 `c` 触发、`x` 中止，输出 `[cal]` 稳态样本；⑥ 新增 [tools/motor\_calib](../../tools/motor_calib/README.md) 离线分析脚本，用日志拟合 PWM→RPM 与绕组/电压塌陷误差模型。详见 §3.6 / §4.4 / §5 / §6.4 / §9 修订历史 v0.7。
>
> **📐 Stage 2.6 右电机基础补偿（2026-05-10，本文 v0.8）**：基于 Stage 2.5 校准扫描数据（正向 PWM→RPM 线性拟合：左 `0.2432 rpm/‰`、右 `0.2559 rpm/‰`，斜率比 **1.0522**），确认 TB6612 B 通道比 A 通道在正转方向固有快 **5.22%**，反转方向两路近乎对称（仅 0.83% 差异）。该偏差属于硬件通道基础特性，应在 `bsp_motor` 统一补偿，而不是分散在 demo / balance 应用层。当前 `commit_right()` 对右路正向命令固定乘 `BSP_MOTOR_RIGHT_FORWARD_SCALE_X1000=950`，所有业务路径共用；同步环只处理剩余动态误差。
>
> **🧭 Stage 2.7 默认装车模式入口调整（2026-05-10，本文 v0.9）**：`main.c` 上电默认进入 `app_balance_run()` 装车模式；仅在装车模式收到 UART `t` / `test` 后切入 `app_motor_demo_run()` 电机演示模式。电机演示中继续使用 UART `l` / `load` 返回装车模式。板载 S1/S2 已退出业务定义。
>
> **🔋 Stage 2.8 电池预热自适应 + 分压实测校准（2026-05-30，本文 v0.10）**：[`bsp_battery.{c,h}`](../../template/hardware/bsp_battery.h) 去掉固定 `t=3s` 单次快照，改为 250 ms 轮询 + 2 s 基线窗口平台检测 + 15 s 兜底；`DIVIDER_RATIO_X10000` 1803→**1503**；`BOOT_OK_MV` 7000→**11000**；新增 `bsp_battery_is_ready()`。解决冷上电半充电压误报与心跳 `batt` 虚低。详见 §9 修订历史 v0.10。
>
> **⚙️ Stage 2.9 右编码器迁 TIMG0 捕获中断（2026-06-01，本文 v0.11）**：装车平衡大外力纠偏时出现"左轮远快于右轮、整车自转一周"。根因 = 旧方案右轮 `PA12/PA13` GPIO 双沿中断（X2，34000 cnt/rev）在 ≈530 RPM 出轴侧边沿率 **300 边/ms 恰好命中** `BSP_MOTOR_ENC_IRQ_QUENCH_PER_MS=300` 雪崩阈值 → 中断被关 50 ms → `right_count` 冻结 → 差速环误判。**修复**：本器件仅 TIMG8 支持硬件 QEI（左轮已占），无第二路 QEI；[`bsp_motor.{c,h}`](../../template/hardware/bsp_motor.h) 将右轮迁到 **`PA12 → TIMG0_CCP0` 硬件捕获 + `TIMG0_IRQHandler` 独立向量**（X1 仅上升沿，PA13 ISR 内读电平判向），`RIGHT_COUNTS_PER_OUTPUT_REV` **34000→17000**，雪崩阈值 **300→1000**；删除 `GROUP1_IRQHandler` 右编码器分支。TIMG0 由 BSP 手动 init（不在 SysConfig），[LP_MSPM0G3507.syscfg](../../EIDE/LP_MSPM0G3507.syscfg) 注释标注 TIMG0 已占用。详见 §2.2 / §4.1 / §6.3 / §7.3 / §9 修订历史 v0.11。
>
> 文档定位：本轮交付 `TB6612 + GB370` 电机驱动演示固件，覆盖 PWM 输出、速度反馈、双轮同步、校准扫描、编码器角度累计，以及 XDS 调试串口日志 / 命令控制。默认入口为装车模式，电机演示仅作为 UART 触发的测试模式。Stage 2.1 起底层 BSP 升级为"平衡环 / 速度环可直接接入"的完备模块；Stage 2.2 起补齐安全 + 电池 + PID + 平衡骨架，业务层一行 `set_balance_gains` 即可启动整定；Stage 2.3 起左编码器从 J12 迁到 BP，全车不再依赖未焊跳线排针。
>
> 关联文件：
>
> - 主入口：[main.c](../../template/main.c)
> - 电机底层：[bsp\_motor.h](../../template/hardware/bsp_motor.h)、[bsp\_motor.c](../../template/hardware/bsp_motor.c)
> - 演示任务：[app\_motor\_demo.h](../../template/app/app_motor_demo.h)、[app\_motor\_demo.c](../../template/app/app_motor_demo.c)
> - **Stage 2.5 新增工具**：[tools/motor\_calib](../../tools/motor_calib/README.md)（校准日志解析 + PWM/RPM 拟合 + 误差模型图表）；[tools/motor\_calib/serial\_capture.py](../../tools/motor_calib/serial_capture.py)（串口录制 + 自动发令）
> - **Stage 2.2 新增**：[bsp\_battery.{h,c}](../../template/hardware/bsp_battery.h) | [middle/pid.{h,c}](../../template/middle/pid.h) | [app\_safety.{h,c}](../../template/app/app_safety.h) | [app\_balance.{h,c}](../../template/app/app_balance.h)
> - 引脚真源：[Stage0-PinAllocation.md](Stage0-PinAllocation.md)

***

## 1. 本轮目标

| # | 需求                                                          | 落地结果                                                                 |
| - | ----------------------------------------------------------- | -------------------------------------------------------------------- |
| 1 | 模式切换与电机演示人工控制                                             | 上电默认装车模式；UART `t` / `test` 进入电机演示，UART `l` / `load` 返回装车模式 |
| 2 | 接收编码器信号并主动更新角度到调试串口                                         | 已完成，左轮 TIMG8 硬件 QEI，右轮 TIMG0 捕获中断（Stage 2.9），100 ms 打印一次          |
| 3 | 形成任务日志文档并给出接线指导                                             | 已完成，见本文 §7                                                           |
| 4 | **【Stage 2.1】驱动 API 重写**：把"够 demo 跑"的最小集升级为平衡环 / 速度环可直接接入 | 已完成，新增 13 个 API（单轮 / brake / invert / pwm\_limit / 编码器原始 / 速度反馈），见 §3 |
| 5 | **【Stage 2.2】上车准备**：补齐右轮 X4 解码 + 脉冲刹车 + 电池保护 + 安全状态机 + 通用 PID + 平衡骨架 | 已完成，新增 4 个模块 8 个文件（`bsp_battery` / `pid` / `app_safety` / `app_balance`），见 §3.5 + §4.3 |
| 6 | **【Stage 2.5】电机测速调试入口**：装车模式下按需切入 demo，可串口调速 / 急刹 / 启动 / 查看同步诊断 / 返回装车模式 | 已完成，装车模式 UART `t/test` 进入 demo；demo 支持 `+/-`、`<rpm><Enter>`、`b/r/s/p/c/x/l/load/h`，见 §5 |
| 7 | **【Stage 2.5】双轮同步 + 校准扫描**：解决左右电机绕组差异 / 电源电压不足导致同 PWM 不同速 | 已完成，新增同步 PI 差分补偿、`[cal]` 扫描日志和 `tools/motor_calib` 离线拟合，见 §3.6 / §6.4 |
| 8 | **【Stage 2.6】右路基础 5% 补偿**：消除 TB6612 B 通道固有 5.22% 正转速度差，避免各应用层重复补偿 | 已完成，`BSP_MOTOR_RIGHT_FORWARD_SCALE_X1000=950` 写入 `bsp_motor`，见 §4.1 |
| 9 | **【Stage 2.9】右编码器迁 TIMG0 捕获**：消除 GPIO 雪崩误关导致的差速环自转 | 已完成，PA12→TIMG0_CCP0 X1 捕获 + PA13 判向，17000 cnt/rev，见 §2.2 / §9 v0.11 |

***

## 2. 实现摘要

### 2.1 驱动结构

- [`bsp_motor.c`](../../template/hardware/bsp_motor.c) 负责底层硬件访问：
  - `TB6612` 的 `AIN1/AIN2/BIN1/BIN2/STBY`，并显式区分 4 态：Coast / Forward / Reverse / **Brake**（IN1=IN2=H）
  - `TIMA0` 双路 PWM 占空比设置（PWM 频率 ≈ 20 kHz，超出人耳）
  - 左轮 `TIMG8 QEI` mode 2 (X4) 16-bit 计数扩展为 32-bit
  - 右轮 `PA12 → TIMG0_CCP0` 硬件捕获中断（X1 上升沿 + PA13 读电平判向），反馈符号按实车安装翻正
  - 速度差分窗口（默认 10 ms ⇒ 100 Hz 速度反馈刷新率）
- [`app_motor_demo.c`](../../template/app/app_motor_demo.c) 负责演示 / 调试逻辑：
  - 上电后默认两轮同向运行，目标转速按 `620 rpm` 满量程换算为 PWM
  - XDS-UART 支持在线调速、急刹 / 启动、同步开关、同步诊断、校准扫描、进入装车模式
  - 双轮同步服务默认开启，按 `rpmR-rpmL` 做左右 PWM 差分补偿
  - 每 `100 ms` 打印累计计数、角度、rpm、同步误差与左右命令
  - 每 `250 ms` 翻转一次绿灯，作为循环心跳

### 2.2 编码器策略

当前实现（Stage 2.9 更新）：

- **左轮**：`TIMG8` 硬件 QEI（PB15/PB16），X4 解码，实测定标 **68000 cnt/rev**（500 PPR × 34:1 × 4）
- **右轮**：`PA12 → TIMG0_CCP0` 硬件捕获中断（Stage 2.9），X1 仅捕获 A 相上升沿，PA13 在 ISR 内读电平判方向，实测定标 **17000 cnt/rev**（500 PPR × 34:1 × 1）；反馈符号在 Stage 2.5 按实车安装翻正

硬件约束与迁移背景：

- MSPM0G3507 **仅 TIMG8 支持硬件 QEI**（左轮已占），无第二路 QEI
- 旧方案 `PA12/PA13` GPIO 双沿 + `GROUP1_IRQHandler`（X2，34000 cnt/rev）在 ≈530 RPM 下边沿率 300 边/ms **恰好命中**雪崩阈值 300 → 中断关 50 ms → 右轮速度读 0 → 差速环误判自转
- 新方案迁到 **TIMG0 独立中断向量**：530 RPM 下仅 150 次/ms，雪崩阈值抬到 1000，正常高速不再误触发

这样做的好处：

- 左轮角度统计稳定，不怕高速漏脉冲
- 右轮高速反馈不再被 GPIO 雪崩整段冻结（根治纠偏自转）
- `rpm` 字段已 CPR 归一化，左右轮可直接比较；`cps` 因分辨率不同仍有 4× 差异，速度/差速环应优先用 `rpm` 或做 CPR 归一化

### 2.3 内部状态聚合 + ISR 共享保护（Stage 2.1）

原先散落在文件作用域的 6 个 `static volatile` 变量统一收编进 `motor_state_t s_motor` 单例，按"是否被 ISR 摸过"分两段：

| 段                | 字段                                                                                  | 访问规则                              |
| ---------------- | ----------------------------------------------------------------------------------- | --------------------------------- |
| **ISR 共享**       | `left_count` / `right_count` / `toggle_request` / `last_button_ms`                  | 所有读写必须包在 `MOTOR_LOCK / UNLOCK` 之间 |
| **主循环私有**        | `left_raw_prev` / `speed_window_*` / `*_cmd_pm` / `pwm_limit_pm` / `invert_*` / `enabled` | 主循环单线程访问，不需关中断                  |

`MOTOR_LOCK / UNLOCK` 在 ARM Cortex-M0+ 上落实为 `__disable_irq() / __enable_irq()` 一对（PRIMASK 短锁），关中断窗口 ≤ 10 个指令周期，对 1 kHz SysTick / 20 kHz PWM / 编码器 ISR 都不会引入可观察的抖动。

### 2.4 1 kHz 路径整数化（Stage 2.1）

Cortex-M0+ 没有 FPU，软浮点单次乘除 ≈ 50~150 cycles。原 `bsp_motor_update()` 在 1 kHz 节拍里直接做 `count * 360.0 / 1320` 浮点除法，单拍约消耗 0.3 ~ 0.6 % CPU，看似无伤但在堆叠 IMU + 平衡环后会成为隐性瓶颈。

新版按"算的人 = 用的人"原则改写：

- **`bsp_motor_update()` (1 kHz 调)**：只做 `int16` QEI 差分扩 32-bit、`int32` 速度窗口累计与差分，**全程无浮点**；
- **`bsp_motor_get_feedback()` (业务侧 10~100 ms 调)**：现算 `angle_deg / dps / rpm` 三组浮点字段，调用方按需触发即可。

效果：1 kHz 调度路径里 `bsp_motor_update()` 的实际开销下降到 ≤ 30 cycles，即 < 0.001 % CPU。

***

## 3. 关键接口说明

### 3.1 完整 API 一览（Stage 2.1）

按职责分 8 组，所有原型见 [bsp\_motor.h](../../template/hardware/bsp_motor.h)：

| 组               | API                                                                                | 调用时机                              | 备注                                  |
| --------------- | ---------------------------------------------------------------------------------- | --------------------------------- | ----------------------------------- |
| **初始化 / 使能**    | `bsp_motor_init()`                                                                 | `SYSCFG_DL_init` + `bsp_gpio_init` 之后 | 计数清零、TIMG0 捕获 init + NVIC 注册、STBY 强制低位（待机）           |
|                 | `bsp_motor_enable(bool)`                                                           | 主循环 / 安全态切换                       | 拉高 / 拉低 STBY；上电默认 false             |
|                 | `bsp_motor_is_enabled()`                                                           | 任意时刻                              | 查询 STBY 当前态                         |
| **速度命令**        | `bsp_motor_set_output(L, R)`                                                       | 业务节拍                              | 同时设左右两轮，permille ∈ [-1000, 1000]    |
|                 | `bsp_motor_set_left(L)` / `bsp_motor_set_right(R)`                                 | 业务节拍                              | 单轮独立更新，平衡 / 差速控制必备                  |
|                 | `bsp_motor_stop()`                                                                 | 常规减速                              | Coast：方向位清零、PWM=0；电机自然滑行            |
|                 | `bsp_motor_brake()`                                                                | 急停 / 跌倒保护                         | Brake：IN1=IN2=H、PWM 满；TB6612 内部短接两端 |
| **极性 / 限幅（运行时）** | `bsp_motor_set_invert(invL, invR)`                                                 | 装车后单次 / 标定时                       | 软件方向反转，函数返回前立刻按新极性重发命令              |
|                 | `bsp_motor_get_invert(*invL, *invR)`                                               | 查询 / 持久化                          | NULL 安全                             |
|                 | `bsp_motor_set_pwm_limit(lim)`                                                     | 安全降功率 / 调试限速                      | 钳到 `[0, 1000]`，立刻应用到当前命令            |
|                 | `bsp_motor_get_pwm_limit()`                                                        | 任意时刻                              | 默认 `BSP_MOTOR_PWM_MAX_PERMILLE = 1000` |
| **当前命令查询**      | `bsp_motor_get_left_cmd()` / `bsp_motor_get_right_cmd()`                           | 日志 / 调试                           | 返回最近一次写入的 permille（限幅 / 极性之前的值）     |
| **1 kHz 周期任务**  | `bsp_motor_update()`                                                               | 1 kHz SysTick 节拍                  | 整数化路径：QEI 软扩 + 速度窗口累计               |
| **反馈快照**        | `bsp_motor_get_feedback(*fb)`                                                      | 业务节拍（10~100 ms 典型）                | 一次性快照 10 个字段，含浮点角度 / dps / rpm      |
| **编码器原始**       | `bsp_motor_get_left_count()` / `bsp_motor_get_right_count()`                       | 速度环 / 里程估算                        | 单字段原子读，省去整结构体                       |
|                 | `bsp_motor_reset_encoders()`                                                       | 上电校准 / 试跑前                        | 同步清零 count + 速度窗口；不影响 STBY 与命令      |

### 3.2 反馈结构体 `bsp_motor_feedback_t`（Stage 2.1 扩展）

| 字段                                        | 类型      | 含义                            |
| ----------------------------------------- | ------- | ----------------------------- |
| `left_count` / `right_count`              | int32   | 编码器累计计数（int32，约 ±5×10^5 圈不溢出） |
| `left_angle_deg` / `right_angle_deg`      | float   | 输出轴累计机械角                      |
| `left_speed_cps` / `right_speed_cps`      | int32   | 输出轴瞬时角速度，单位 counts/s          |
| `left_speed_dps` / `right_speed_dps`      | float   | 输出轴瞬时角速度，单位 °/s               |
| `left_speed_rpm` / `right_speed_rpm`      | float   | 输出轴瞬时转速，单位 rpm                |

**速度刷新率**：由 `BSP_MOTOR_SPEED_WINDOW_MS`（默认 10 ms）决定 → 100 Hz；最低可分辨速度

- 左轮：`1000/10 = 100 cps ≈ 100/68000 × 60 ≈ 0.09 rpm`
- 右轮：`100 cps ≈ 100/17000 × 60 ≈ 0.35 rpm`

如调小窗口（如 10 ms）则响应快、低速分辨率粗；调大窗口（如 50 ms）反之。该宏在 `bsp_motor.h` 顶部，且支持外部 `-D` 覆盖，无需改头文件。

### 3.3 平衡 / 速度环典型用法（参考代码）

```c
bsp_motor_feedback_t fb;
bsp_motor_init();
bsp_motor_set_pwm_limit(700);      /* 调试期限速 70% */
bsp_motor_enable(true);

for (;;) {
    if (bsp_systick_consume_tick()) {
        bsp_motor_update();         /* 1 kHz 整数路径 */
    }
    if ((now_ms - last_ctrl_ms) >= 10u) {  /* 100 Hz 速度环 */
        last_ctrl_ms = now_ms;
        bsp_motor_get_feedback(&fb);
        int16_t out_l = pid_step(&pid_l, target_dps, fb.left_speed_dps);
        int16_t out_r = pid_step(&pid_r, target_dps, fb.right_speed_dps);
        bsp_motor_set_output(out_l, out_r);
    }
    if (fall_detected) {
        bsp_motor_brake();          /* 急停 */
        bsp_motor_enable(false);    /* 顺手降级 STBY */
    }
}
```

### 3.4 极性约定（重要）

`bsp_motor_set_invert()` 只对**命令侧**生效；**反馈侧 (count / speed) 始终按编码器物理方向计数**。

- 装车后若发现"命令正转、轮子反转" → 调 `bsp_motor_set_invert(true, ...)`，命令侧瞬时翻转，编码器读数不变（仍是物理方向）。
- 如果业务层希望"反馈也跟随业务正向"（如 PID 反馈与命令同号），自行在调用方乘 -1 即可，不必改 BSP。

这条约定是有意为之：保留编码器作为"硬件真理"，invert 只是给 TB6612 真值表加一层软重映射，避免 BSP 层做"双向语义翻译"导致调试时分不清"哪个是物理 / 哪个是逻辑"。

### 3.5 Stage 2.2 新增模块 API

> ⚠️ 本节列的 4 个模块文件 **已就位但暂未加入 EIDE 构建** —— 当前 main.c 跑的是 [`app_telemetry_run()`](../../template/app/app_telemetry.c)（Stage 1.6 IMU 遥测基线），尚未切到 motor demo 链路。装车准备整定时按 §6.3 步骤把以下 8 个文件 + `bsp_motor.{c,h}` + `app_motor_demo.{c,h}` 一并加入 [EIDE/.eide/eide.yml](../../EIDE/.eide/eide.yml) 的 `virtualFolder` 即可，include 路径已配好（`hardware/` `middle/` `app/`）。

#### 3.5.1 `bsp_battery.{c,h}` —— 电池电压采样 + 阈值状态机

| API | 用途 | 备注 |
| --- | --- | --- |
| `bsp_battery_init()` | 触发首次软件转换 | 前置：`SYSCFG_DL_init()` 已完成；ADC0/PB24/参考 VDDA=3.3V 已 syscfg 配好 |
| `bsp_battery_update()` | 周期采样（建议 100 Hz） | 轮询 `MEM0_RESULT_LOADED` → EMA → 状态分类 → 触发下一拍；典型耗时 < 10 µs |
| `bsp_battery_get_mv()` | 滤波后电池电压（毫伏） | EMA 时间常数 ≈ 80 ms（α=32/256） |
| `bsp_battery_get_raw()` | 最近一次 ADC 原始读数（0~4095） | 调试用 |
| `bsp_battery_get_state()` | 三态枚举：`UNKNOWN` / `NORMAL` / `LOW_WARN` / `LOW_STOP` | 阈值 + 200 mV 回滞，避免抖动 |
| `bsp_battery_raw_to_mv(raw)` | 纯函数：raw → mV，调试 / 单测入口 | 全程 uint32 无浮点，最高输入 4095 安全 |

**默认阈值**（编译期可改）：`WARN_MV = 9500`、`STOP_MV = 9000`、`HYSTERESIS_MV = 200`、分压系数 = 22/(100+22) ≈ 0.1803（与 [Stage0 §4.6](Stage0-PinAllocation.md) 一致）。

#### 3.5.2 `middle/pid.{c,h}` —— 通用浮点 PID 控制器

| API | 用途 |
| --- | --- |
| `pid_init(*pid)` | 安全默认：增益全 0、输出 ±1000、D 滤波禁用、内部状态清零 |
| `pid_set_gains(*pid, kp, ki, kd)` | 设置 PID 三项增益（任意值） |
| `pid_set_output_limit(*pid, lo, hi)` | 设置输出钳位区间 |
| `pid_set_integral_limit(*pid, i_abs_max)` | 设置积分项独立绝对值上限（抗 windup）；0 = 跟 u_max 同 |
| `pid_set_d_filter(*pid, alpha)` | 设置 D 项 EMA 系数 [0,1]；0 = 禁用 |
| `pid_reset(*pid)` | 清积分 + 微分历史，不动增益 |
| `pid_step(*pid, target, measured, dt_sec)` | 跑一拍并返回输出（已限幅） |

**算法特点**：位置式 + 抗积分饱和（积分回卷）+ "d on measurement"（避免 setpoint 阶跃产生 D 冲击）+ 微分项可选 EMA 滤波。**默认增益 0** = 失效安全：未整定不输出。

#### 3.5.3 `app_safety.{c,h}` —— 安全状态机

| API | 用途 |
| --- | --- |
| `app_safety_init()` | 状态置 DISARMED + 电机 brake + STBY 关 |
| `app_safety_arm()` | 切到 ARMED + 电机 enable；`LOW_BAT_STOP` 态拒绝并返回 false |
| `app_safety_disarm()` | 主动切 DISARMED + brake_pulse + enable(false)（人工急停） |
| `app_safety_tick(*att)` | 周期任务（建议 100 Hz）：跌倒检测 + 电池状态合成；返回最新状态 |
| `app_safety_get_state()` | 仅查询状态枚举 |
| `app_safety_can_drive()` | 当前是否允许业务下发电机命令（`ARMED` 或 `LOW_BAT_WARN`） |

**5 态状态机**：`DISARMED` / `ARMED` / `LOW_BAT_WARN` / `FALLEN` / `LOW_BAT_STOP`，优先级 `LOW_BAT_STOP > FALLEN > LOW_BAT_WARN > ARMED`。
**跌倒判据**：`|pitch_deg| > APP_SAFETY_FALL_PITCH_DEG`（默认 60°，与 Overview §4.1 一致）→ `bsp_motor_brake_pulse_ms(80)` + `bsp_motor_enable(false)`。
**低压急停**：`bsp_battery_get_state() == LOW_STOP` → `bsp_motor_brake_pulse_ms(120)` + `enable(false)`，且**不会自动恢复**：电池升回 LOW_WARN 后状态降到 LOW_BAT_WARN 但保持 STBY 关，必须由上层再次调用 `app_safety_arm()`。
**重启策略**：DISARMED / FALLEN / LOW_BAT_WARN 仅由上层显式 `app_safety_arm()` 恢复；低压未恢复时 arm 会被拒绝。

#### 3.5.4 `app_balance.{c,h}` —— 平衡车控制（Stage 2.2 初版骨架）

> **⚠️ API 已演进**：本节描述 Stage 2.2 双环骨架。**当前装车代码为 Stage 3.7 两级级联**（速度 20 Hz → 角度 100 Hz → PWM + 航向角环 20 Hz），使用 `pid2_t` 与 `set_*_gains(kp, ki, kd, out_offset)`。完整 API 与整定流程以 [`app_balance.h`](../../template/app/app_balance.h) 及 [Stage3-BalanceControl.md §7A](Stage3-BalanceControl.md) 为准。

| API（Stage 2.2 历史） | 用途 |
| --- | --- |
| `app_balance_init()` | 初始化两路 PID（输出限幅 + D 滤波系数已写入；增益默认 0） |
| `app_balance_reset()` | 清两路 PID 内部历史，不动增益 |
| `app_balance_set_pitch_offset(deg)` | 设置静态俯仰零点（让车在地面"标准直立"姿态下读 1 s 平均 pitch 填入） |
| `app_balance_set_pitch_inverted(bool)` | 软件翻转俯仰角极性；MS901M 前后装反时无需改解析层 |
| `app_balance_get_pitch_inverted()` | 查询当前俯仰角是否启用软件翻转 |
| `app_balance_set_balance_gains(kp, ki, kd)` | 平衡内环：输入 tilt 误差 deg，输出 PWM permille |
| `app_balance_set_speed_gains(kp, ki, kd)` | 速度外环：输入 cps 误差，输出目标 tilt deg |
| `app_balance_set_yaw_kp(kp_yaw)` | 转向开环系数（`left -= yaw·k, right += yaw·k`） |
| `app_balance_step(*att, *cmd)` | 跑一拍控制环（建议 100 Hz）：safety tick + 速度外环 + 平衡内环 + 转向叠加 + 直行双轮同步 + `bsp_motor_set_output()` |
| `app_balance_get_diag(*out)` | 拷贝本拍诊断（target_tilt / pitch_meas / pwm_out / sync / left/right_cmd / speed / driving） |

**控制结构**：速度外环（输入 cps，输出目标 tilt deg，限幅 ±10°） → 平衡内环（输入 tilt 误差 deg，输出 PWM permille，限幅 ±1000）→ 转向叠加 + 直行双轮同步 → `bsp_motor_set_output(left, right)`。
**与 safety 集成**：`app_balance_step()` 内部调 `app_safety_tick()`；不允许驱动时**不调** `set_output`（保留 safety 的 brake 命令），同时 reset PID 历史。
**姿态极性与滤波**：`pitch_meas = LPF((raw_pitch - offset) * pitch_sign)`；当前装车传感器前后方向与车体坐标相反，默认 `APP_BALANCE_PITCH_INVERT=1`。MS901M 解析层保留模块 EKF 原始输出，`app_balance` 额外使用 `APP_BALANCE_PITCH_LPF_ALPHA` 做主控侧一阶低通，抑制单帧抖动。
**直行同步补偿**：仅 `target_yaw_pm == 0` 时启用，误差 `sync_error_cps = right_speed_cps - left_speed_cps`，输出 `sync_correction_pm` 并按 `left += sync / right -= sync` 叠加；转向时自动暂停并清积分。
**默认增益 0** = 上电不会自己动；业务侧调 `set_*_gains()` 注入后才工作；详细整定流程见 [`app_balance.h`](../../template/app/app_balance.h) 顶部注释（4 步级联 PID 整定法）。

### 3.6 `app_motor_demo` 调速 / 同步 / 校准服务（Stage 2.5）

Stage 2.5 后，演示层不再只是固定 PWM 验证，而是承担电机空载测速、双轮同步和离线建模数据采集：

| API | 用途 |
| --- | --- |
| `app_motor_demo_set_speed_rpm(rpm)` / `get_speed_rpm()` | 设置 / 查询目标空载转速；按 GB370 最大 `620 rpm` 钳位并换算为 PWM permille |
| `app_motor_demo_set_sync_enabled(bool)` | 开关双轮同步；关闭时左右轮输出同一基准 PWM |
| `app_motor_demo_set_sync_gains(kp, ki)` | 配置同步环增益；误差定义为 `rpmR - rpmL`，输出为左右差分 PWM 补偿 |
| `app_motor_demo_reset_sync()` | 清同步环积分与诊断；改变目标转速或重新启动前调用 |
| `app_motor_demo_get_sync_diag(*out)` | 读取 `enabled / kp / ki / correction / cmdL / cmdR / rpm_error` 快照 |
| `app_motor_demo_cal_start()` | 启动 PWM 正向 + 反向扫描校准；自动关闭同步环并输出 `[cal]` 日志 |
| `app_motor_demo_cal_abort()` | 中止校准，立即 brake 并恢复 demo 状态 |
| `app_motor_demo_cal_is_active()` | 查询校准状态，主循环据此跳过普通同步 / `[enc]` 日志 |
同步环策略（Stage 2.6 更新）：

```
left_pm  = target_pm + correction_pm
right_pm = target_pm − correction_pm
```

- **基础补偿**（Stage 2.6 更新）：右路正转 5% 补偿已下沉到 `bsp_motor` 的 `commit_right()`，demo 同步环不再重复施加静态补偿。
- **PI 项**：50 ms 一拍读取 `left/right_speed_rpm`，误差 = `rpmR - rpmL`。底层基础补偿接管稳态后 PI 仅处理瞬态，默认 `Kp=4 pm/rpm`、`Ki=0`、`corr` 限幅 `±200‰`（均比 Stage 2.5 下调）。

校准扫描策略：串口 `c` 触发后，PWM 从 `100‰` 到 `1000‰` 以 `50‰` 步进，每档驻留 `1500 ms`，跳过前 `500 ms` 瞬态后每 `100 ms` 输出一次 `[cal]` 稳态样本；正向完成后自动做负向扫描。扫描完成后恢复进入校准前的同步开关和目标 PWM。离线脚本见 [tools/motor_calib](../../tools/motor_calib/README.md)。

***

## 4. 默认参数与可调项

### 4.1 编译期宏（[`bsp_motor.h`](../../template/hardware/bsp_motor.h) 顶部）

| 宏                                       | 默认值      | 含义                                                                              |
| --------------------------------------- | -------- | ------------------------------------------------------------------------------- |
| `BSP_MOTOR_PWM_MAX_PERMILLE`            | `1000`   | PWM 命令满量程千分比，**不要改**（其它代码假设这是 1000）                                            |
| `BSP_MOTOR_GB370_GEAR_RATIO`            | `34`     | GB370 减速比（标称 9.6:1，实测定标以 `COUNTS_PER_OUTPUT_REV` 为准）                          |
| `BSP_MOTOR_GB370_HALL_PPR`              | `500`    | 电机霍尔每转脉冲数（A 相单沿）                                                                |
| `BSP_MOTOR_LEFT_DECODE_X`               | `4`      | 左轮 QEI X4 解码                                                                  |
| `BSP_MOTOR_RIGHT_DECODE_X`              | **`1` (Stage 2.9)** | 右轮 TIMG0 捕获 X1（仅 PA12 上升沿）；宏仅作文档/cnt/rev 表述，解码逻辑固定于 `bsp_motor.c` |
| `BSP_MOTOR_RIGHT_FORWARD_SCALE_X1000`   | `1000`   | 右路正向基础补偿（1000=不补偿；Stage 2.6 曾用 950，以当前 .h 为准） |
| `BSP_MOTOR_LEFT_COUNTS_PER_OUTPUT_REV`  | **`68000`** | 实测定标：500 × 34 × 4                                                         |
| `BSP_MOTOR_RIGHT_COUNTS_PER_OUTPUT_REV` | **`17000` (Stage 2.9)** | 实测定标：500 × 34 × 1；TIMG0 捕获 X1，无第二路 QEI              |
| `BSP_MOTOR_SPEED_WINDOW_MS`             | **`10`** | 速度差分窗口；100 Hz 速度刷新率，最低分辨速度 ≈ 0.09 rpm（左）/ 0.35 rpm（右）              |
| `BSP_MOTOR_ENC_IRQ_QUENCH_PER_MS`       | **`1000` (Stage 2.9)** | 右编码器捕获中断雪崩阈值（次/ms）；旧值 300 在 X2@530 RPM 误触发           |
| `BSP_MOTOR_ENC_IRQ_QUENCH_DURATION_MS`  | `50`     | 雪崩触发后关闭 TIMG0 CC0 中断的毫秒数                                               |
| `APP_MOTOR_DEMO_MAX_RPM`                | `620`    | Stage 2.5：GB370 空载最大转速，用于目标 rpm → PWM permille 换算（[`app_motor_demo.c`](../../template/app/app_motor_demo.c) 内宏） |
| `APP_MOTOR_DEMO_DEFAULT_RPM`            | `620`    | Stage 2.5：demo 上电默认目标转速 |

右编码器 init / ISR 路径（Stage 2.9）：`bsp_motor_init()` 内 PA12 mux 到 `TIMG0_CCP0`、手动 `DL_TimerG_initCaptureMode()`、`TIMG0_IRQHandler` 分发 `DL_TIMER_IIDX_CC0_DN`；**不再**使用 `GROUP1_IRQHandler` / PA12 GPIO 双沿中断。TIMG0 不在 SysConfig 内，由 BSP 独占；[LP_MSPM0G3507.syscfg](../../EIDE/LP_MSPM0G3507.syscfg) 注释已标注占用。

### 4.2 运行时可调项（Stage 2.1 新增）

| 调整项     | API                                  | 默认值      | 典型场景                          |
| ------- | ------------------------------------ | -------- | ----------------------------- |
| 左轮极性翻转  | `bsp_motor_set_invert(invL, ...)`    | false    | 装车后发现命令方向与轮子方向相反              |
| 右轮极性翻转  | `bsp_motor_set_invert(..., invR)`    | false    | 同上                            |
| PWM 上限  | `bsp_motor_set_pwm_limit(lim_pm)`    | `1000`   | 电池低压、调试限速、热保护                 |
| 编码器归零   | `bsp_motor_reset_encoders()`         | —        | 上电校准、试跑前清零里程                  |
| STBY 开关 | `bsp_motor_enable(bool)`             | false    | 跌倒保护时拉低、起立后拉高                 |

> **⚠️ 已废弃宏（Stage 2.0 → 2.1）**：原 `bsp_motor.c` 顶部的 `BSP_MOTOR_LEFT_SIGN` / `BSP_MOTOR_RIGHT_SIGN` 编译期符号宏在重写中删除。方向反转改为运行时 `bsp_motor_set_invert()`，避免"调一次方向就要重新编译 / 烧录"的痛点。如旧分支引用了这两个宏，搜索全工程即可发现引用点已为 0。

### 4.3 Stage 2.2 新增模块编译期宏

| 模块 | 宏 | 默认值 | 含义 |
| --- | --- | --- | --- |
| **bsp_battery** | `BSP_BATTERY_ADC_REF_MV` | `3300` | ADC 参考电压（与 SysConfig `ADCMEM_0_REF_VOLTAGE_V` 强绑定） |
| | `BSP_BATTERY_ADC_FULL_SCALE` | `4095` | 12-bit ADC 满量程 |
| | `BSP_BATTERY_DIVIDER_RATIO_X10000` | `1803` | 分压系数 × 10000；R1=100k, R2=22k → 22/(100+22)≈0.1803 |
| | `BSP_BATTERY_EMA_ALPHA_256` | `32` | EMA 系数 × 256；α=32 → 时间常数 ≈ 80 ms @ 100 Hz |
| | `BSP_BATTERY_WARN_MV` | `9500` | 低压告警阈值；触发 PWM 限幅降级 |
| | `BSP_BATTERY_STOP_MV` | `9000` | 安全停车阈值；触发 brake + STBY |
| | `BSP_BATTERY_HYSTERESIS_MV` | `200` | 状态回滞，避免阈值附近抖动 |
| | `BSP_BATTERY_LOW_STOP_DEBOUNCE_SAMPLES` | `20` | LOW_STOP 连续确认拍数；100 Hz 下约 200 ms，滤掉上电 ADC 瞬态 |
| **app_safety** | `APP_SAFETY_FALL_PITCH_DEG` | `60.0f` | 跌倒判据；与 Overview §4.1 一致 |
| | `APP_SAFETY_FALL_BRAKE_MS` | `80` | 跌倒急停 brake 脉冲毫秒 |
| | `APP_SAFETY_LOW_BAT_BRAKE_MS` | `120` | 低压急停 brake 脉冲毫秒 |
| | `APP_SAFETY_LOW_BAT_PWM_LIMIT` | `600` | 低压告警时 PWM 限幅（permille） |
| | `APP_SAFETY_STARTUP_FALL_MUTE_MS` | `2500` | 上电姿态静默窗口；等待 MS901M 输出稳定，期间不判跌倒且平衡层不输出 PID |
| | `APP_SAFETY_FALL_DEBOUNCE_TICKS` | `5` | 跌倒连续确认拍数；100 Hz 下约 50 ms，滤掉单帧姿态毛刺 |
| **app_balance** | `APP_BALANCE_CONTROL_PERIOD_MS` | `10` | 控制环周期；100 Hz |
| | `APP_BALANCE_MAX_TILT_DEG` | `10.0f` | 速度外环输出"目标 tilt"绝对值上限 |
| | `APP_BALANCE_MAX_PWM_PERMILLE` | `1000` | 平衡内环输出 PWM 绝对值上限 |
| | `APP_BALANCE_SPEED_D_FILTER_ALPHA` | `0.20f` | 速度外环 D 项 EMA；速度环噪声大需要滤 |
| | `APP_BALANCE_BALANCE_D_FILTER_ALPHA` | `0.10f` | 平衡内环 D 项 EMA |
| | `APP_BALANCE_PITCH_LPF_ALPHA` | `0.35f` | 俯仰角一阶低通系数；100 Hz 下约 18 ms 时间常数，兼顾去抖与低延迟 |
| | `APP_BALANCE_PITCH_INVERT` | `1` | 俯仰角软件极性翻转；MS901M 前后装反时保持默认，安装方向正确可设 0 |
| | `APP_BALANCE_SYNC_ENABLED` | `1` | 装车模式直行双轮同步服务开关 |
| | `APP_BALANCE_SYNC_KP_PM_PER_CPS_X100` | `20` | 同步比例项，0.01 permille/cps；20≈0.20 pm/cps≈4.4 pm/rpm |
| | `APP_BALANCE_SYNC_KI_PM_PER_CPS_STEP_X100` | `0` | 同步积分项，默认关闭以避免与平衡内环互相积分 |
| | `APP_BALANCE_SYNC_MAX_CORRECTION_PM` | `200` | 同步差分补偿限幅 |
| | `APP_BALANCE_SYNC_MIN_DRIVE_PM` | `30` | 同步环启用的最小平衡输出；PID 输出接近 0 时暂停，平衡环发力时恢复同步以抑制原地打转 |

> **运行时可调（无需重新编译）**：
>
> - `app_balance_set_pitch_offset(deg)` — 静态俯仰零点
> - `app_balance_set_pitch_inverted(bool)` — 运行时切换俯仰角软件翻转
> - `app_balance_set_balance_gains(kp, ki, kd)` — 平衡内环增益
> - `app_balance_set_speed_gains(kp, ki, kd)` — 速度外环增益
> - `app_balance_set_yaw_kp(k)` — 转向开环系数
> - `app_safety_arm()` / `app_safety_disarm()` — 主动 arm / 急停
>
> 所有 PID 增益**默认 0** = 上电不输出，业务侧通过串口 / K230 命令注入即可整定。

### 4.4 Stage 2.5 演示 / 同步 / 校准宏

| 宏 | 默认值 | 含义 |
| --- | --- | --- |
| `APP_MOTOR_DEMO_MAX_RPM` | `620` | GB370 最大空载转速；所有目标 rpm 按此钳位并映射到 `1000‰` PWM |
| `APP_MOTOR_DEMO_DEFAULT_RPM` | `620` | 上电默认目标转速 |
| `APP_MOTOR_DEMO_BRAKE_MS` | `120` | 串口 `b` 急刹脉冲时长 |
| `APP_MOTOR_DEMO_RPM_STEP` | `20` | 串口 `+/-` 每次调速步进 |
| `APP_MOTOR_SYNC_PERIOD_MS` | `50` | 双轮同步闭环周期 |
| `APP_MOTOR_SYNC_KP_PM_PER_RPM` | **`4`**（Stage 2.6 由 8 降低） | 同步环比例项：底层基础补偿接管稳态后 PI 仅需处理瞬态，增益可减半 |
| `APP_MOTOR_SYNC_KI_PM_PER_RPM_STEP` | **`0`**（Stage 2.6 由 1 降低） | 同步环积分项：底层基础补偿已消除主要稳态误差，积分暂关闭 |
| `APP_MOTOR_SYNC_MAX_CORRECTION_PM` | **`200`**（Stage 2.6 由 350 收窄） | 同步差分补偿限幅 |
| `APP_MOTOR_CAL_PWM_START_PM` | `100` | 校准扫描起始 PWM |
| `APP_MOTOR_CAL_PWM_END_PM` | `1000` | 校准扫描终止 PWM |
| `APP_MOTOR_CAL_PWM_STEP_PM` | `50` | 校准扫描步进 |
| `APP_MOTOR_CAL_DWELL_MS` | `1500` | 每档驻留时间 |
| `APP_MOTOR_CAL_SAMPLE_PERIOD_MS` | `100` | 稳态样本输出周期 |
| `APP_MOTOR_CAL_SETTLE_MS` | `500` | 每档进入后的瞬态等待时间 |

***

## 5. 串口日志格式

日志与控制命令均走板载 `UART0(XDS-UART)`，波特率保持 `115200 8N1`。`bsp_log_uart` 使用 UART0 RX 中断环形缓冲接收命令，避免 `printf` 阻塞发送期间丢失短命令。当前主程序上电默认进入装车模式；在装车模式发送 `t` 或 `test` 切入电机演示模式（`t` 单字符立即生效，`test` 兼容回车），演示模式发送 `l` 或 `load` 返回装车模式。

上电进入装车模式后会看到：

```text
[boot] MSPM0G3507 stage2.2 balance baseline start (MS901M / TB6612 / safety)
[boot] MS901M attitude online, ...
[boot] entering load balance mode; inject PID by UART
[pid] UART commands: bp <kp_x1000> <ki_x1000> <kd_x1000>, sp <kp_x1000> <ki_x1000> <kd_x1000>, pid?, pid0, t/test
[pid] example: bp 8000 0 1000 ; sp 2 0 0
[pid] balance kp=+0.000 ki=+0.000 kd=+0.000 (x1000=0,0,0)
[pid] speed kp=+0.000 ki=+0.000 kd=+0.000 (x1000=0,0,0)
[hb] t=1s state=ARMED pitch=... inv=1 tilt*=... pwm=... syncErr=... syncCorr=... log_ovr=0 ...
```

进入电机演示模式后会看到：

```text
[boot] stage2 motor demo start
[boot] target=620rpm pwm=1000/1000 max=620rpm
[boot] motor sync enabled kp=4 ki=0 maxCorr=200 period=50ms
[boot] send 'l' or 'load' on UART to enter load balance mode
[ctrl] UART commands: '+'/'-' step 20rpm, '<rpm><Enter>' set speed, 'b' brake, 'r' run,
       's' sync on/off, 'p' print sync, 'c' calib sweep, 'x' abort calib,
       'l'/'load' enter load mode, 'h' help
```

运行中每 `100 ms` 输出：

```text
[enc] t=1200ms L=158(43.09 deg) R=164(44.73 deg) rpmL=86 rpmR=90 target=620rpm state=run sync=1 err=4 corr=33 cmdL=1000 cmdR=967
```

`p` 命令输出同步快照：

```text
[ctrl] sync=on err=2 corr=8 cmdL=1008 cmdR=992 kp=4 ki=0
```

控制命令：

| 命令 | 行为 |
| --- | --- |
| `+` / `-` | 目标转速按 `APP_MOTOR_DEMO_RPM_STEP` 增 / 减 |
| `<数字><Enter>` | 直接设置目标 rpm，例如 `180` 回车 |
| `b` | 急刹，执行 `bsp_motor_brake_pulse_ms(APP_MOTOR_DEMO_BRAKE_MS)` |
| `r` | 启动，恢复当前目标转速输出 |
| `s` | 开关双轮同步服务 |
| `p` | 打印同步诊断：`err / corr / cmdL / cmdR / kp / ki` |
| `c` | 启动 PWM 校准扫描，输出 `[cal]` 日志 |
| `x` | 中止校准扫描并急刹 |
| `l` / `load` | 停止 demo 输出并返回 `true`，由 `main.c` 切入 `app_balance_run()` 装车模式 |
| `h` / `?` | 打印帮助 |

装车模式命令：

| 命令 | 行为 |
| --- | --- |
| `bp <kp> <ki> <kd>` | 设置平衡内环 PID，参数为 x1000 定点整数；例 `bp 8000 0 1000` = `8.000 / 0 / 1.000` |
| `sp <kp> <ki> <kd>` | 设置速度外环 PID，参数为 x1000 定点整数；例 `sp 2 0 0` = `0.002 / 0 / 0` |
| `pid?` | 打印当前平衡环 / 速度环 PID 参数 |
| `pid0` | 清零两路 PID 增益并 reset 内部状态 |
| `t` / `test` | 停止装车控制输出并返回 `true`，由 `main.c` 切入电机演示模式 |

校准扫描日志示例：

```text
[cal] start dir=+1 steps=19 pm_start=100 pm_end=1000 step=50 dwell_ms=1500 settle_ms=500
[cal] step dir=+1 idx=0/19 pm=100
[cal] dir=+1 idx=0/19 pm=100 t=12345 vbat=11120 rpmL=18 rpmR=21 ctL=... ctR=...
[cal] done dir=+1 next=reverse
[cal] calibration complete
```

说明：

- `L` / `R` 是累计计数，不会自动清零；如需归零请在主循环里调 `bsp_motor_reset_encoders()`
- `deg` 是基于编码器参数换算出的输出轴机械角
- `rpmL` / `rpmR` 来自 `bsp_motor_feedback_t.left/right_speed_rpm`，窗口由 `BSP_MOTOR_SPEED_WINDOW_MS` 决定
- `err = rpmR - rpmL`；`corr` 是同步环输出的差分 PWM 补偿；`cmdL/cmdR` 是最终写入 TB6612 的左右 PWM permille
- 若正反方向和实物相反，**Stage 2.1 起改为运行时调** `bsp_motor_set_invert(invL, invR)`（[`bsp_motor.h`](../../template/hardware/bsp_motor.h)），不必重新编译；该函数返回前会立刻按新极性重发当前命令
- Stage 2.5 右编码器反馈符号已按实物安装翻正，同向命令下左右计数 / rpm 应同号

***

## 6. 验收建议

### 6.1 演示固件基线验证（Stage 2.5）

按下面顺序验证：

1. **空载验方向**
   - 先断开车轮离地
   - 上电后观察两轮是否同向旋转
   - 若某轮方向相反，调用 `bsp_motor_set_invert()` 或调整接线；Stage 2.5 不再用 S1 正反转
2. **验编码器计数**
   - 用手慢慢拨动左轮，日志中的 `L` 应连续变化
   - 用手慢慢拨动右轮，日志中的 `R` 应连续变化
   - 同向运行时 `L/R` 应同号增长；反向拨动时计数应反向变化
3. **验角度换算**
   - 在轮胎上做一个明显标记
   - 手动转约 1 圈，观察日志角度是否接近 `360 deg`（左轮 ≈68000 cnt/rev → 右轮 ≈17000 cnt/rev，右轮角度步进为左轮 1/4，**rpm 仍可直接比较**）
   - 如果明显偏差，优先检查 `COUNTS_PER_OUTPUT_REV` 实测定标是否和实物一致
4. **验串口控制**
   - 发送 `b`，两轮应急刹；发送 `r`，两轮恢复运行
   - 发送 `180` 回车，日志 `target` 应变成 `180rpm`
   - 发送 `s`，日志 `sync` 应在 `0/1` 间切换；发送 `p` 打印同步诊断
5. **验模式切换**
   - 上电默认应进入装车模式；发送 `t` 或 `test` 后应切入电机演示
   - 在电机演示中发送 `l` 或 `load`，应打印 load mode requested，并回到装车模式
6. **验双轮同步**
   - 在 `620rpm` 目标下观察 `rpmL/rpmR`
   - 若右轮更快，应看到 `err > 0`、`corr > 0`、`cmdR < cmdL`
   - 稳态时左右 rpm 差应小于同步关闭时的自然差值

### 6.2 Stage 2.1 新增 API 快速回归（无需上车）

如果只想验证 BSP 重写没有引入回归，可在 `app_motor_demo.c` 主循环里临时插测：

| 测试                     | 调用                                                                            | 预期                                          |
| ---------------------- | ----------------------------------------------------------------------------- | ------------------------------------------- |
| 单轮控制                   | `bsp_motor_set_left(400); bsp_motor_set_right(0);`                            | 仅左轮转，右轮静止；编码器 `L` 增长、`R` ≈ 0               |
| Brake vs Coast         | `bsp_motor_set_output(800, 800)` → 1 秒后 `bsp_motor_brake()`，对比 `bsp_motor_stop()` | brake 停车几乎瞬时；stop 后编码器仍因惯性继续跳几十计数            |
| 运行时极性                  | `bsp_motor_set_output(400, 400)` → 调 `bsp_motor_set_invert(true, false)`      | 左轮立刻反向，右轮方向不变                               |
| PWM 限幅                 | `bsp_motor_set_pwm_limit(200); bsp_motor_set_output(800, 800);`               | 实际转速对应 200 / 1000 而非 800；调 `set_pwm_limit(1000)` 后立刻恢复 |
| 编码器归零                  | 让车自由滑行后 `bsp_motor_reset_encoders()`                                          | 下次 `get_feedback()` 的 `*_count` / `*_speed_*` 全部为 0 |
| 速度反馈                   | 拨动左轮维持 ≈ 60 rpm                                                               | `feedback.left_speed_rpm` ≈ 60 ± 5（窗口 20 ms）|
| `is_enabled` / `get_*_cmd` | 任意时刻读                                                                         | 与最近一次 `enable / set_output` 写入值一致           |

### 6.3 Stage 2.2 上车整定回归矩阵

> 把新模块（`bsp_battery / pid / app_safety / app_balance` + `bsp_motor`）加入 [EIDE/.eide/eide.yml](../../EIDE/.eide/eide.yml) 后，按下面顺序逐项验证。**前 4 项空载（车轮离地或拆下）**，第 5 项后才能落地。

| # | 验证项 | 验证步骤 | 通过判据 |
| --- | --- | --- | --- |
| 1 | **右轮 TIMG0 捕获上线** | 手动拨右轮一圈，对比 `feedback.right_count` 增量 | 一圈应 ≈ **17000** ± 50；同向命令下 `rpmL/rpmR` 同号；大力推车时心跳 `[ISR_QUENCH!]` 不应再误触发 |
| 2 | **bsp_battery 采样链路** | `bsp_battery_init()` + 100 Hz 调 `bsp_battery_update()`，1 Hz 打印 `get_mv()` / `get_state()` | 12V 满电时显示 ≈ 12000~12700 mV，状态 `NORMAL`；用电源拉到 9.4V 后状态变 `LOW_WARN` |
| 3 | **app_safety 跌倒触发** | `app_safety_init()` + `arm()`，手动倾斜 IMU > 60° | 状态变 `FALLEN`，电机 STBY 变低；需由上层再次调用 `app_safety_arm()` 才能恢复 |
| 4 | **app_safety 低压急停** | 把 `BSP_BATTERY_STOP_MV` 临时改 11500（高于现实电压），连续运行 `tick()` | 约 20 个电池采样确认后变 `LOW_BAT_STOP`，低压未恢复时 `app_safety_arm()` 被拒绝 |
| 5 | **brake_pulse_ms 自动转 stop** | `bsp_motor_set_output(800,800)` → 1 s 后 `bsp_motor_brake_pulse_ms(80)` | 80 ms 内电机急停，之后转 coast（手拨能转）；不像持续 brake 一直锁住 |
| 6 | **PID 单元自测**（不上车） | 单跑 `pid_step()` 多个目标，画响应曲线 | 阶跃响应有上升 + 收敛；增益全 0 时输出恒 0；积分饱和后撤回不再继续累加 |
| 7 | **平衡内环单环（车轮离地）** | `app_balance_set_balance_gains(Kp, 0, Kd)`（小 Kp 起步），手摇车体 | 电机出现"反向纠偏"PWM；从弱到强，找出"反应明显但不振"的最大 Kp |
| 8 | **平衡内环（落地短测）** | 用支架辅助起立 → 撤手 ≤ 3 s | 车能短暂直立，前后摆动 ≤ 5 cm；如发散，先记录波形再调小 Kp / 增大 Kd |
| 9 | **速度外环上线** | 内环已可短直立 → `app_balance_set_speed_gains(Kp_s, Ki_s, 0)` | 给 `cmd.target_speed_cps = 0`，撤手后车体能维持原地（非慢漂） |
| 10 | **转向叠加** | `cmd.target_yaw_pm = 200` | 车体原地缓转；K230 给定的 ω 转化为差速可观察 |

### 6.4 Stage 2.5 电机同步 / 校准验证

| 验证项 | 验证步骤 | 通过判据 |
| --- | --- | --- |
| 串口调速 | 发送 `100` / `300` / `620` 回车 | `[enc] target=` 随命令变化，`cmdL/cmdR` 随目标变化 |
| 同步开关 | 发送 `s` 关闭同步，再发送 `s` 打开同步 | `sync=0` 时 `cmdL≈cmdR≈base`；`sync=1` 时出现 `corr/cmdL/cmdR` 差分 |
| 同步效果 | 在 `620rpm` 下对比同步关闭 / 开启后的 `rpmL/rpmR` | 开启后左右 rpm 差缩小；若振荡，降低 `APP_MOTOR_SYNC_KP_PM_PER_RPM` / `KI` |
| 急刹 / 启动 | 串口 `b` / `r` | `b` 后电机急停；`r` 后恢复当前目标 rpm |
| 电机演示入口 | 装车模式下串口发送 `t` 或 `test` | `app_balance_run()` 返回 `true`，`main.c` 进入 demo |
| 装车模式入口 | demo 下串口发送 `l` 或 `load` | demo 停止输出，`app_motor_demo_run()` 返回 `true`，`main.c` 回到装车模式 |
| 校准扫描 | 开启串口日志录制 → 发送 `c` → 等待 `[cal] calibration complete` | 正 / 反向都输出完整 `[cal]` 样本；扫描后恢复原同步配置 |
| 离线分析 | `python tools/motor_calib/analyze_calib.py calib_run.txt` | 生成聚合统计、PWM→RPM 拟合、误差模型结论和 4 张 PNG 图 |
| **右路基础补偿效果**（Stage 2.6） | 在 `620rpm` 下发送 `p`，对比补偿前后的 `err / corr` | BSP 右路 95% 补偿生效后，`err` 应接近 0，`corr` ≈ 0；右路物理 PWM 已在底层缩放 |

***

## 7. 接线指导

### 7.1 TB6612 与 MSPM0G3507

| TB6612 引脚 | MSPM0G3507 引脚 | LQFP pin | BoosterPack 接出       | 跳线 / 备注                                                                            |
| --------- | ------------- | -------- | -------------------- | ---------------------------------------------------------------------------------- |
| `PWMA`    | `PA8`         | 54       | BP J1.1（**或 J3.30**）              | TIMA0_CCP0；Stage 1.6 默认接 BP J1.1 / J3.30，无需焊接                                       |
| `AIN1`    | `PA15`        | 8        | BP J3.27             | GPIO，电机方向 1                                                                       |
| `AIN2`    | `PA16`        | 9        | BP J3.29             | GPIO，电机方向 2；**`J15` 默认接 BP J3.29 共用此脚，BP J3.29 排针上不要再连其他设备**                       |
| `PWMB`    | `PA9`         | 55       | **BP J1.3** (J14)    | TIMA0_CCP1；**Stage 1.6 起 `J14` 须切到 (2)-(3) 才把 PA9 引到 BP J1.3**，默认 (1)-(2) 是 PB23   |
| `BIN1`    | `PA26`        | 30       | BP J4.39             | GPIO，电机方向 1；**`J18` 必须 OFF**（默认接 OPA0_IN0+，会与 BIN1 抢线）                             |
| `BIN2`    | `PA27`        | 31       | BP J4.38             | GPIO，电机方向 2；**`J17` 必须 OFF**（默认接 OPA0_IN0-）                                        |
| `STBY`    | `PB0`         | 47       | BP J2.18             | GPIO；上电默认低 = TB6612 待机；`bsp_motor_enable(true)` 后拉高                                |
| `VCC`     | `5V` 或驱动板逻辑电源 | —        | LaunchPad J27 5V     | 逻辑电源，按板卡要求接                                                                        |
| `VM`      | 电机独立电源        | —        | **不与主控共电**           | 电机供电，独立 6~13.5 V 电池；不可直接拿 MCU 3V3 代替                                                |
| `GND`     | `GND`         | —        | 共地                   | 必须与主控、电池、K230 单点星型汇接                                                                |
| `AO1/AO2` | 左 GB370 两根电机线 | —        | —                    | 左电机功率输出；方向反了交换两线，或调 `bsp_motor_set_invert(true, ...)`                              |
| `BO1/BO2` | 右 GB370 两根电机线 | —        | —                    | 右电机功率输出；同上                                                                         |

### 7.2 左电机编码器

> **Stage 2.3 起从 J12 迁到 BoosterPack（PB15/PB16，2-Pin Mode）**：原方案 PA29/PA30/PB14 三脚走 LaunchPad J12 QEI 接头，但 J12 排针**出厂未焊接**无法直接接线；同时 GB370 编码器无 Z 相，IDX 可省。复核数据手册 PINMUX 表后选用 BP 上空闲的 PB15/PB16，TIMG8 QEI 由 3-Pin Mode 降为 2-Pin Mode，**硬件 X4 解码，实测定标 68000 cnt/rev，无丢脉冲**。

| 编码器引脚     | MSPM0G3507 引脚 | LQFP pin | 接出方式              | 跳线 / 备注                                                                                  |
| --------- | ------------- | -------- | ----------------- | ---------------------------------------------------------------------------------------- |
| `A`       | `PB15`        | 32       | BP **J4.34**      | `TIMG8_CCP0`（QEI PHA, mux f=5, PINCM32）；**`J12` 不再使用**                                   |
| `B`       | `PB16`        | 33       | BP **J4.40**      | `TIMG8_CCP1`（QEI PHB, mux f=5, PINCM33）；**`J12` 不再使用**                                   |
| `Z/Index` | —             | —        | **不接（2-Pin Mode）** | GB370 编码器无 Z 相，PB14 进入预留池；不影响 X4 解码精度                                                    |
| `VCC`     | `3V3`         | —        | LaunchPad J28 3V3 | **先确认编码器模块电压等级**（部分 GB370 编码器模块需 5V，标错会烧 GPIO）                                          |
| `GND`     | `GND`         | —        | 共地                | 与主控共地                                                                                    |

### 7.3 右电机编码器

> **Stage 2.9 起 PA12 改走 TIMG0 硬件捕获（接线物理脚不变）**：A 相仍接 `PA12`（BP J4.40），B 相仍接 `PA13`（BP J4.39 邻位）。`bsp_motor_init()` 将 PA12 mux 到 `TIMG0_CCP0`（PINCM34 PF=4），由 `TIMG0_IRQHandler` 捕获上升沿；PA13 保持普通 GPIO 输入，仅在 ISR 内读电平判方向。**无需改线**，仅固件路径变更。

| 编码器引脚 | MSPM0G3507 引脚 | LQFP pin | 接出方式      | 跳线 / 备注                                                                  |
| ----- | ------------- | -------- | ----------- | ------------------------------------------------------------------------ |
| `A`   | `PA12`        | 5        | BP J4.40    | **Stage 2.9**：`TIMG0_CCP0` 硬件捕获（X1 上升沿）；BSP 宏 `BSP_ENC_R_A_*`；datasheet `TIMG0_C0 [PF=4]` |
| `B`   | `PA13`        | 6        | BP J4.39 邻位 | 普通 GPIO 输入（上拉+滞回）；**不在 TIMG0 上**，ISR 内读电平判方向；17000 cnt/rev（X1） |
| `VCC` | `3V3`         | —        | LaunchPad J28 3V3 | **先确认编码器模块电压等级**（标错烧 GPIO）                                              |
| `GND` | `GND`         | —        | 共地          | 与主控共地                                                                    |

### 7.4 板载资源

| 资源               | 引脚          | LQFP pin | BSP 宏 / 外设                  | 跳线 / 用途                                                                |
| ---------------- | ----------- | -------- | --------------------------- | ---------------------------------------------------------------------- |
| `XDS-UART TX`    | `PA10`      | 56       | UART_LOG (UART0_TX)         | **`J21` 必须 ON**；XDS-UART 桥到电脑虚拟 COM，调试 `printf` 输出                     |
| `XDS-UART RX`    | `PA11`      | 57       | UART_LOG (UART0_RX)         | **`J22` 必须 ON**；电脑 → 主控，用于 `t/test`、`bp/sp/pid?`、`b/r/c/x/l/load` 等串口命令                       |
| `LED_R`          | `PB26`      | 28       | `BSP_LED_R_*`               | **`J6` 保留 ON**（RGB-R）；上电默认亮表示未就绪，main.c 后续按需熄灭                          |
| `LED_G`          | `PB27`      | 29       | `BSP_LED_G_*`               | **`J7` 保留 ON**（RGB-G）；250 ms 翻转作主循环心跳                                  |
| `LED_B`          | `PB22`      | 21       | `BSP_LED_B_*`               | **`J5` 保留 ON**（RGB-B）；预留业务状态提示                                |
| `SWDIO`          | `PA19`      | 12       | DEBUGSS                     | **`J101` 13:14 ON**；XDS110 烧录 / 调试，必接                                   |
| `SWCLK`          | `PA20`      | 13       | DEBUGSS                     | **`J101` 15:16 ON**；同上                                                  |

### 7.5 接线注意事项

#### 7.5.1 LaunchPad 跳线核对清单（电机 / 编码器相关，与 Stage0 §2 一一对应）

> 装车前**逐条核对**；未到位的项任意一条都可能让 BIN1/BIN2 与板载 OPA0 抢线、PWMB 接错排针、或 BUZZER/LASER_EN 漏电流。

| 跳线   | 涉及引脚                       | 必须状态        | 不到位的后果                                            |
| ---- | -------------------------- | ----------- | ------------------------------------------------- |
| `J4` | PA0 → 板载 LED1（红）           | **OFF**     | LED1 抢占 PA0（蜂鸣器输出），bsp 拉高时 LED 一并被点亮，蜂鸣器电流被分走     |
| `J5` | PB22 → RGB-Blue            | **保留 ON**   | LED_B 不亮，无法显示方向切换提示                               |
| `J6` | PB26 → RGB-Red             | **保留 ON**   | LED_R 不亮，bsp_motor_init 后失去"未就绪"红灯提示              |
| `J7` | PB27 → RGB-Green           | **保留 ON**   | LED_G 不亮，主循环心跳指示失效                                |
| `J12` | PA29 / PA30 / PB14 → QEI 接头  | **OFF / 不再使用**（v0.5） | Stage 2.3 起左编码器迁到 BP J4.34 (PB15) / J4.40 (PB16)，J12 排针出厂未焊不再使用；PA29/PA30/PB14 进入预留池 |
| `J14` | PB23 ↔ PA9 ↔ BP J1.3        | **(2)-(3) PA9** | 默认 (1)-(2) 是 PB23，PWMB 信号到不了 BP J1.3，右电机不转       |
| `J15` | PA16 ↔ BP J3.29             | 保留默认 (1)-(2) PA16 | AIN2 共用此排针；J3.29 排针上**不要再外接其他设备**                |
| `J17` | PA27 → OPA0_IN0-           | **OFF**     | BIN2 与 OPA0_IN0- 抢线，TB6612 接收的 PA27 电平被 OPA 负载拉偏 |
| `J18` | PA26 → OPA0_IN0+           | **OFF**     | BIN1 与 OPA0_IN0+ 抢线，同上                            |
| `J19` | PA0 开漏上拉 → 3V3              | **OFF**     | 蜂鸣器输出被 3V3 弱拉高，待机静音失败                             |
| `J20` | PA1 开漏上拉 → 3V3              | **OFF**     | 激光使能被 3V3 弱拉高，待机时激光可能微亮                           |
| `J21` | PA10 → UART0_TX (XDS)       | **保留 ON**   | XDS-UART 看不到 `[enc]` / `[motor]` 调试日志             |
| `J22` | PA11 → UART0_RX (XDS)       | **保留 ON**   | 同上                                                |
| `J101` | XDS110-ET 隔离块               | **保留 ON**   | XDS110 烧录 / 调试整体失效                                |

#### 7.5.2 装车通用注意事项

- 编码器 `VCC` 请先确认是 `3.3 V` 还是 `5 V` 版本，不确定前不要直接接主控 IO。
- TB6612 的 `VM` 与主控逻辑电源分开供电，但 **GND 必须共地**（推荐 TB6612 散热片下铜区为汇接点）。
- 初次上电务必让车轮离地，确认方向正确后再落地测试。
- **本阶段所有电机 / 左右编码器 / LED 引脚都从 LaunchPad BoosterPack 排针直接接出，无需焊接**（左编码器 Stage 2.3 起从 J12 迁到 BP J4.34/J4.40；详见 [Stage0-PinAllocation.md §3.4](Stage0-PinAllocation.md) 必焊清单）。BUZZER (PA0) / LASER_EN (PA1) 因 LaunchPad 板载固有限制需要焊飞线，但本阶段不用，可忽略。
- 若某个电机命令方向与轮子实际方向相反，**Stage 2.1 起首选** `bsp_motor_set_invert(invL, invR)` 运行时翻转（参见 §3 / §4.2）；不要再交换动力线或硬编码改 `AIN/BIN` 真值表，避免出厂调好的极性被下一次重新焊接打乱。
- 若编码器读数方向与轮子实际转向相反（即"轮子正转、count 减少"），同样优先用上一条 invert 的同一调用 —— 它在 BSP 内部对命令侧翻转后，业务层将命令与反馈作差时符号会自动一致；如果调用方需要"反馈也按业务正向"，请在 `bsp_motor_get_feedback()` 之后自行乘 -1，**不要**回到 BSP 层做二次翻转，会破坏"编码器 = 硬件真理"的约定。

***

## 8. 后续建议

Stage 2.1 已完成的（不再列入待办）：

- ~~`bsp_motor` 上层叠加轮速计算（`count delta / dt`），形成速度闭环输入~~ → 已经在 BSP 内置 `feedback.left_speed_dps` 等 6 个速度字段
- ~~跌倒时自动拉低 `STBY` 的保护逻辑~~ → BSP 已提供 `bsp_motor_brake()` + `bsp_motor_enable(false)` 原语，业务层封装 1 行即可

Stage 2.2 已完成的（不再列入待办）：

- ~~**右轮升级为 X4 解码**~~ → Stage 2.2 曾用 GPIO X4；Stage 2.9 已整体迁 **TIMG0 捕获 X1**（17000 cnt/rev），根治 GPIO 雪崩误关导致的差速环自转。历史路径：`GROUP1_IRQHandler` + PA12/PA13 双沿 → 已删除。
- ~~**右轮高速丢计数 → 升级 CAPTURE 模式**~~ → Stage 2.9 已落地：`PA12 → TIMG0_CCP0` + `TIMG0_IRQHandler` 独立向量；TIMG0 由 BSP 手动 init，SysConfig 注释标注占用。
- ~~**接入平衡环 / 速度环骨架**~~ → 已交付 [middle/pid.{c,h}](../../template/middle/pid.h) 通用 PID + [app\_balance.{c,h}](../../template/app/app_balance.h) 速度外环 + 平衡内环 + 转向叠加，`app_balance_step()` 100 Hz 节拍调一次即跑完全链路。**所有 PID 增益默认 0**（失效安全），业务侧需调 `set_balance_gains` / `set_speed_gains` 注入；详细整定流程见 [app\_balance.h](../../template/app/app_balance.h) 顶部注释。
- ~~**电池低压保护与 PWM 自动降功率**~~ → 已交付 [bsp\_battery.{c,h}](../../template/hardware/bsp_battery.h) 周期采样 + 阈值状态机；[app\_safety.{c,h}](../../template/app/app_safety.h) 在 `LOW_BAT_WARN` 自动调 `bsp_motor_set_pwm_limit(600)`，在 `LOW_BAT_STOP` 自动 `brake_pulse_ms(120) + enable(false)`。
- ~~**跌倒检测 → brake**~~ → [app\_safety.{c,h}](../../template/app/app_safety.h) 已实现：`|pitch| > 60°` → `brake_pulse_ms(80) + enable(false)`，恢复由上层显式 `app_safety_arm()` 触发。
- ~~**加 IDLE 节流**~~ → 已交付 `bsp_motor_brake_pulse_ms(N)` API，N ms 后由 `bsp_motor_update()` 自动转 coast，避免 brake 持续注入大电流；持续刹车需求保留旧 `bsp_motor_brake()`（持续模式互斥）。

仍然剩余 / 推荐的下一步（Stage 3+ 范围）：

- **EIDE 构建核对**：当前 `main.c` 已默认进入 `app_balance_run()` 装车模式；构建配置需包含 [`bsp_motor.{c,h}`](../../template/hardware/bsp_motor.h) + [`bsp_battery.{c,h}`](../../template/hardware/bsp_battery.h) + [`middle/pid.{c,h}`](../../template/middle/pid.h) + [`app_safety.{c,h}`](../../template/app/app_safety.h) + [`app_balance.{c,h}`](../../template/app/app_balance.h) + [`app_motor_demo.{c,h}`](../../template/app/app_motor_demo.h)。
- **PID 增益整定 + Flash 持久化**：按 [`app_balance.h`](../../template/app/app_balance.h) §"整定提示"4 步流程，先内环后外环；整定完成后把增益写入 Flash（参考阶段 0 §4.5 标定参数固化策略）。
- **K230 通讯接入 `app_balance_motion_cmd_t`**：`MOTION_CMD` 帧解析后填 `target_speed_cps / target_yaw_pm`，主控的 `app_balance_step()` 负责执行；超时降级时业务侧把 `cmd` 清零并调 `app_safety_disarm()`。
- **声光提示 + 自动起立 + 圈数判定**：完成基础项 1~4 后再上发挥项；自动起立可参考 Overview §阶段 7.2，反向梯形速度曲线甩起 + 接近垂直时切平衡环。

***

## 9. 修订历史

| 日期         | 版本   | 修订内容                                                                                                                                                                                                                                                                                                                                                                                                                | 作者   |
| ---------- | ---- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---- |
| 2026-04-30 | v0.1 | 阶段 2 初版：TB6612 双电机驱动 + 左轮 TIMG8 QEI + 右轮 PA12 双边沿中断 + S1 切换演示固件；含接线表 / 串口日志格式 / 验收建议                                                                                                                                                                                                                                                                                                                              | 主控团队 |
| 2026-05-09 | v0.2 | **Stage 2.1 BSP 重写**：[bsp\_motor.{c,h}](../../template/hardware/bsp_motor.h) 从 7 个最小接口扩展为 8 组共 18 个 API（`set_left/set_right` 单轮控制、`brake` 短刹车区分 `stop` 滑行、`set_invert / get_invert` 运行时极性、`set_pwm_limit / get_pwm_limit` 安全限幅、`get_left_count / get_right_count / reset_encoders` 编码器原始接口、`get_left_cmd / get_right_cmd / is_enabled` 状态查询）。`bsp_motor_feedback_t` 追加 6 个速度字段（cps / dps / rpm × L/R），50 Hz 速度刷新率。1 kHz 路径整数化（浮点延迟到 `get_feedback()`），状态聚合到 `motor_state_t s_motor` 单例并显式划分 ISR 共享段 vs 主循环私有段、用 `MOTOR_LOCK / UNLOCK` PRIMASK 短锁保护。**完全向后兼容**：[app\_motor\_demo.c](../../template/app/app_motor_demo.c) 原 5 个调用 (`enable / set_output / consume_toggle_request / update / get_feedback`) 行为不变，原结构体字段全部保留。**已废弃**：原顶部宏 `BSP_MOTOR_LEFT_SIGN / RIGHT_SIGN` 删除，方向反转改为运行时 `bsp_motor_set_invert()`。文档同步：§1 增第 4 项交付、§2 新增 §2.3 状态聚合 + §2.4 整数化、§3 完全重写为全 API 表 + 反馈字段表 + 平衡环用法骨架 + 极性约定、§4 拆 4.1 编译期宏（增 SPEED\_WINDOW\_MS / DECODE\_X / BTN\_DEBOUNCE\_MS）+ 4.2 运行时可调项 + 已废弃宏提示、§5 把"调宏"改为"调 invert API"、§6 新增 6.2 BSP 回归快速测试矩阵、§7.5 接线注意事项更新极性调整流程、§8 划掉两条已实现的 TODO 并新增 5 条剩余优化项 | 主控团队 |
| 2026-05-09 | v0.3 | **引脚 / 跳线核对（分支迁移后回归）**：本文从分支 A 迁回时引脚信息基于 Stage0 v0.6 旧版，与本分支 Stage 1.6（Stage0 真源 v0.8）"引脚集中化重排"存在信息漂移。对照 [Stage0-PinAllocation.md §3.2](Stage0-PinAllocation.md) v0.8、[bsp\_gpio.h](../../template/hardware/bsp_gpio.h)、[ti\_msp\_dl\_config.h](../../EIDE/ti_msp_dl_config.h) 三处真源逐脚核对：**电机 / 编码器 / S1 / LED 等 14 个引脚映射全部一致，无引脚错位**；但原 §7 接线指导**漏列 9 个关键 LaunchPad 跳线决策**，最严重的两项是 `J17/J18 必须 OFF`（否则 BIN1/BIN2 与板载 OPA0_IN0+/- 抢线，TB6612 接收的方向位电平被拉偏）和 `J14 必须切 (2)-(3) PA9`（默认 (1)-(2) 是 PB23，PWMB 信号到不了 BP J1.3，右电机不转）。本版次改动：① 文档顶部加 v0.3 引脚核对提示框；② §7.1 TB6612 表加 `LQFP pin` 列与 `BoosterPack 接出` 列、`跳线 / 备注` 列，明确 J14/J15/J17/J18 关联；③ §7.2 左编码器表加 LQFP / J12 ON 提示；④ §7.3 右编码器表加 LQFP / BP 排针位 / X4 升级提示；⑤ §7.4 板载资源表扩展，加入 LED_R / SWDIO / SWCLK 行 + 全部 J5/J6/J7/J8/J21/J22/J101 跳线状态；⑥ §7.5 拆为 §7.5.1 跳线核对清单（覆盖 J4/J5/J6/J7/J8/J12/J14/J15/J17/J18/J19/J20/J21/J22/J101 共 15 项，每项含"涉及引脚 / 必须状态 / 不到位的后果"）+ §7.5.2 装车通用注意事项；⑦ §7.5.2 增加"本阶段所有引脚都从 BP / J12 直接接出无需焊接"说明（与 Stage0 §3.4 必焊清单互校）。**底层代码 / 业务逻辑 / 引脚分配本身均无任何改动**，本版次为纯文档同步 | 主控团队 |
| 2026-05-09 | v0.4 | **Stage 2.2 上车准备就绪**：完成 §8 列出的 5 项后续 TODO，新增 4 个模块共 8 个文件 + bsp_motor 2 处升级。**bsp_motor.{c,h}**：① `BSP_MOTOR_RIGHT_DECODE_X` 默认 `4`（X4 解码），`bsp_motor_init()` 内条件编译启用 PA13 双沿中断、`GPIOA_IRQHandler` 内条件分发 `DL_GPIO_IIDX_DIO13`，`on_right_encoder_edge(bool is_phase_a)` 用一份逻辑覆盖 X2/X4，左右轮分辨率统一 1320 cnt/rev；② 新增 `bsp_motor_brake_pulse_ms(N)` 脉冲式短刹车 API，N ms 后由 `bsp_motor_update()` 自动转 coast，与持续 `bsp_motor_brake()` 互斥（任意 set_output / stop / brake / brake_pulse 都会取消未到期 pulse），`motor_state_t` 增加 `brake_pulse_remain_ms` 字段。**bsp_battery.{c,h} (新建)**：ADC0/PB24 周期采样，单发模式 + 轮询 `MEM0_RESULT_LOADED`，6 个可配宏（参考电压 / 满量程 / 分压系数 / EMA α / WARN / STOP / 回滞），状态机 5 态 + 200 mV 回滞，纯 uint32 无浮点。**middle/pid.{c,h} (新建)**：通用浮点 PID = 位置式 + 抗积分饱和（积分回卷）+ "d on measurement" + D 项独立 EMA 滤波；默认增益 0 = 失效安全。**app_safety.{c,h} (新建)**：5 态状态机（DISARMED / ARMED / LOW_BAT_WARN / FALLEN / LOW_BAT_STOP）+ 优先级 LOW_STOP > FALLEN > LOW_WARN > ARMED；跌倒判据 \|pitch\| > 60° 触发 `brake_pulse_ms(80) + enable(false)`；低压急停后不自动恢复（必须 S1 重启），低压告警自动 `set_pwm_limit(600)`；与 `ms901m.h` 解耦（业务侧传 `app_safety_attitude_t`，方便单测）。**app_balance.{c,h} (新建)**：速度外环（cps→tilt deg，限幅 ±10°）+ 平衡内环（tilt deg→PWM permille，限幅 ±1000）+ 转向开环叠加；与 safety 集成（`step()` 内调 `safety_tick()`，不允许驱动时不调 `set_output` 且 reset PID 历史）；**所有 PID 增益默认 0**，`set_*_gains()` 运行时注入；详细 4 步级联整定流程见 .h 顶部注释。文档同步：① 顶部加 v0.4 提示框 + 关联文件加新模块；② §1 增第 5 项交付；③ §3.5 新增 4 模块 API 表（bsp_battery / pid / app_safety / app_balance）；④ §4.1 `RIGHT_DECODE_X` 默认值改 4；⑤ §4.3 新增 17 个新模块编译期宏 + 5 个运行时可调项；⑥ §6.3 新增 10 项上车整定回归矩阵（X4 验证 → 电池 → safety → brake_pulse → PID 单测 → 平衡内环 → 速度外环 → 转向）；⑦ §7.3 右编码器 PA13 注释改 X4；⑧ §8 把 5 项 TODO 全划完成区，重列 Stage 3+ 待办（新模块加入 EIDE 构建 / 增益整定 + Flash / K230 接入 / 声光 / 自动起立）。**EIDE 构建未改动**，当前 main.c 仍跑 telemetry，新模块文件已就位等装车时一次性接入 | 主控团队 |
| 2026-05-09 | v0.5 | **Stage 2.3 左编码器从 J12 迁到 BoosterPack（不再依赖未焊 J12）**。问题背景：板上 J12 排针（PA29 PHA / PA30 PHB / PB14 IDX）出厂未焊接，无法直接接线，且 PA29/PA30/PB14 均不在 BoosterPack 排针上。复核数据手册 PINMUX 表后发现 `PB15 = TIMG8_C0 [func 5, PINCM32]` / `PB16 = TIMG8_C1 [func 5, PINCM33]` 在 BP J4.34 / J4.40 上空闲（蓝牙 1.6 下线后释放进 §3.3 预留池），同时 GB370 编码器无 Z 相，IDX 可省，QEI 由 3-Pin Mode 降为 2-Pin Mode。**SysConfig 改动**（[EIDE/LP_MSPM0G3507.syscfg](../../EIDE/LP_MSPM0G3507.syscfg)）：`QEI_LEFT.enableIndexInput = false`、`peripheral.ccp0Pin.$assign = "PB15"`、`peripheral.ccp1Pin.$assign = "PB16"`，删除 `idxPin.$assign`，并把段顶注释整段重写说明迁移背景。**`ti_msp_dl_config.{c,h}` 同步重生**：`GPIO_QEI_LEFT_PHA_*` 改 GPIOB / DL_GPIO_PIN_15 / IOMUX_PINCM32 / IOMUX_PINCM32_PF_TIMG8_CCP0；`GPIO_QEI_LEFT_PHB_*` 改 GPIOB / DL_GPIO_PIN_16 / IOMUX_PINCM33 / IOMUX_PINCM33_PF_TIMG8_CCP1；删除 `GPIO_QEI_LEFT_IDX_*` 全套宏；`SYSCFG_DL_GPIO_init()` 删除 IDX 的 `DL_GPIO_initPeripheralInputFunction(...)`；`SYSCFG_DL_QEI_LEFT_init()` 中两处 `DL_TIMER_QEI_MODE_3_INPUT` 改 `DL_TIMER_QEI_MODE_2_INPUT`。**硬件 X4 解码精度 1320 cnt/rev 不变，无丢脉冲**；BSP 层 [bsp\_motor.{c,h}](../../template/hardware/bsp_motor.h) 无需任何改动（QEI 计数路径透明）。**文档同步**：① §7.2 左编码器表整段改写（PHA→PB15/BP J4.34、PHB→PB16/BP J4.40、IDX→不接 / 2-Pin Mode），表前加 Stage 2.3 迁移说明段；② §7.5.1 跳线核对清单 J12 行由 "**必须 ON**" 改为 "**OFF / 不再使用**" + 说明 PA29/PA30/PB14 进入预留池；③ §7.5.2 装车通用注意事项中 "BP / J12 直接接出" 改写为 "全部从 BP 直接接出（左编码器 Stage 2.3 起从 J12 迁到 BP J4.34/J4.40）"。**配套真源同步**：[Stage0-PinAllocation.md](Stage0-PinAllocation.md) v0.9 （§1 决策行 / §2 跳线表 J12 / §3.2 业务表 ENC_L / §3.3 预留表 / §4.1 编码器表 / §5.6 验证清单 / §6 维护规则 / §7 修订历史 同步刷新）。**装车收益**：J12 排针保持悬空、不需焊接；左/右编码器接线全在 BP 排针上 + 同列相邻（J4.34 与 J4.40 同侧），施工与维护成本同步降低 | 主控团队 |
| 2026-05-09 | v0.6 | **Stage 2.4 关键 Bug 修复 ｜ 上车首次启动后两个致命问题定位 + 修复**。**Bug A — `batt=0mV` 始终判 `BAT_STOP`**：根因 = SysConfig 的 `ADC_BAT` 模块漏配 `sampleTime0`（默认 0 cycles），ADC SCOMP0 永远完不成转换，`MEM0_RESULT_LOADED` 不置位，软件读到 0；同时 `bsp_battery::classify` 直接把 0 mV 判 `LOW_STOP` 进 safety `BAT_STOP` 死锁。修复：① [EIDE/LP_MSPM0G3507.syscfg](../../EIDE/LP_MSPM0G3507.syscfg) 加 `BAT_ADC.sampleTime0 = "1 us"` + 修正"内部 VREF 2.5V"误注释为 VDDA 3.3V；② [ti_msp_dl_config.c](../../EIDE/ti_msp_dl_config.c) `SYSCFG_DL_ADC_BAT_init()` 加 `DL_ADC12_setSampleTime0(ADC_BAT_INST, 32)`；③ [bsp_battery.h](../../template/hardware/bsp_battery.h) 新增 `BSP_BATTERY_DISCONNECTED_MV` 默认 1000 mV，[bsp_battery.c](../../template/hardware/bsp_battery.c) 的 `classify()` 在 `mv < DISCONNECTED_MV` 时返回 `UNKNOWN`，未接电池时安全保持 DISARM、不进 LOW_STOP。**Bug B — 手转右轮瞬间宕机**：根因 **不是** ISR 雪崩 / 不是浮空噪声 / 不是优先级问题（这些都已防住），而是 [bsp_motor.c](../../template/hardware/bsp_motor.c) 早期把 GPIO ISR 命名为 `void GPIOA_IRQHandler(void)` —— **MSPM0G3507 vector table 里根本没有 `GPIOA_IRQHandler` 这个名字**！整个 GPIO 中断（GPIOA + GPIOB）+ TRNG + COMP0 共享 IRQn=1 = "GROUP1"，入口名叫 **`GROUP1_IRQHandler`**（参考 SDK `examples/.../driverlib/gpio_simultaneous_interrupts/`）。原 `GPIOA_IRQHandler` 只是个普通全局函数符号，编译链接都不报错但永远不会被链接到 vector table；PA12/PA13/PA18 任何沿事件触发后 NVIC 跳到 vector slot 17 = startup.s 里 weak 默认 `GROUP1_IRQHandler` (`B .` 死循环) → MCU 整体卡死，串口/SysTick/绿灯/业务全停。修复：① [bsp_motor.c](../../template/hardware/bsp_motor.c) 把 ISR 重命名为 `void GROUP1_IRQHandler(void)`，函数顶部注释整段改写、明确警示"GPIOA/GPIOB 共享 GROUP1 入口"；② 同期附加 4 道防护已生效但单独不能救命，仍保留：(a) PA12/PA13/PB15/PB16 启用内部上拉 + Hysteresis（[bsp_gpio.c](../../template/hardware/bsp_gpio.c) + `bsp_motor_init`），(b) SysTick 优先级提到 0（[bsp_systick.c](../../template/hardware/bsp_systick.c)），GPIOA NVIC 优先级降到 3（`bsp_motor_init`），(c) `bsp_motor` 加 ISR 雪崩兜底（200 边沿/ms 触发 disable + 50 ms 自动恢复，新增 `bsp_motor_get_enc_irq_count` / `bsp_motor_enc_irq_is_quenched` 诊断 API），(d) [bsp_systick.c](../../template/hardware/bsp_systick.c) 覆盖 `HardFault_Handler` 为 `NVIC_SystemReset()` —— fault 直接复位、boot log 反复刷屏，比"假死"友好得多，便于以后定位类似问题。**经验教训**：MSPM0 NVIC 把 GPIOA / GPIOB / TRNG / COMP0 等多个外设合并到 GROUP0/GROUP1 共享 vector，与传统 STM32 / MSP432 "每外设一个 vector" 的习惯不同；后续新增任何 GPIO 中断业务（如新加 GPIOB 沿中断、TRNG / COMP0 中断），必须在 `GROUP1_IRQHandler` 内追加分支、不要再写独立 `GPIOA_IRQHandler` / `GPIOB_IRQHandler`。同步：[app_balance.c](../../template/app/app_balance.c) 1 Hz 心跳新增 `encL=/encR=/encISR=` + `[ISR_QUENCH!]` 标记字段方便排查 | 主控团队 |
| 2026-05-10 | v0.7 | **Stage 2.5 电机测速 / 同步 / 校准服务**。`main.c` 已切入 `app_motor_demo_run()` 作为上电默认入口；`app_motor_demo` 从固定 `350‰` 正反转 demo 升级为可交互电机测试台：① 目标转速按 GB370 `620 rpm` 满量程换算 PWM，XDS-UART 支持 `+/-`、`<rpm><Enter>`、`b/r`、`s/p`、`c/x`、`h/?`；② S1 改为急刹 / 启动，PA18 使用双沿 + 上电空闲电平判定 + 轮询兜底，日志打印 `raw/active/btn_irq/btn_poll` 诊断；③ 右编码器反馈符号按实车安装翻正，同向命令下左右计数 / rpm 同号；④ 新增 `app_motor_demo_set_sync_enabled` / `set_sync_gains` / `reset_sync` / `get_sync_diag`，默认 50 ms 同步周期，误差 `rpmR-rpmL`，PI 输出差分 PWM 补偿，`corr` 限幅 `±350‰`；⑤ 新增 `app_motor_demo_cal_start` / `cal_abort` / `cal_is_active` PWM 扫描校准，正 / 反向从 `100‰` 到 `1000‰`，步进 `50‰`，每档驻留 `1500 ms`，跳过前 `500 ms` 后每 `100 ms` 输出 `[cal]` 样本；⑥ 新增 [tools/motor_calib](../../tools/motor_calib/README.md) 离线分析工具，解析 `[cal]` 日志并生成 PWM→RPM 拟合、误差 vs PWM / 电池电压、残差图。文档同步：顶部 v0.7 提示、§1 目标表、§2 摘要、§3.6 API、§4.4 参数、§5 串口命令 / 日志、§6.4 验收矩阵。 | 主控团队 |
| 2026-05-10 | v0.8 | **Stage 2.6 右电机基础补偿**。**根因分析**：Stage 2.5 校准扫描数据拟合结果：正转斜率左 `0.2432 rpm/‰`、右 `0.2559 rpm/‰`，斜率比 **1.0522**（右轮固有快 5.22%）；反转方向斜率差仅 0.83%，两路接近对称。问题为 TB6612 两路 H 桥正向导通参数差异（B 通道压降略低 → 相同 PWM 下右轮获得更多有效驱动电压），属硬件固有特性，与绕组和电压无关。**修复方案**：在 `bsp_motor` 的 `commit_right()` 内对右路正向命令统一乘 `BSP_MOTOR_RIGHT_FORWARD_SCALE_X1000=950`，让 demo、装车平衡、K230 控制等所有业务路径共享同一基础补偿。**参数调整**：同步环 Kp `8 → 4`、Ki `1 → 0`、`maxCorr` `350 → 200`，底层基础补偿接管稳态后 PI 专注瞬态不再积分发散。文档同步：顶部 v0.8 提示框、§1 第 8 项、§3.6 同步公式、§4.1 BSP 宏、§4.4 Kp/Ki/maxCorr 默认值更新、§5 Boot Banner / `p` 命令输出 / 控制命令表、§6.4 验收项。 | 主控团队 |
| 2026-05-30 | v0.10 | **Stage 2.8 电池采样预热自适应 + 分压校准**。问题：固定 `t=3 s` 单次快照在冷上电时旁路电容/电源缓启动未完成，读值严重偏低（实测 ~6.5 V 对应 12 V 满电）→ `boot_ok` 误判 / 心跳 `batt` 虚低 / 偶发误急停。**方案**：① 去掉 `BOOT_SAMPLE_MS` 固定时刻，改为 `BOOT_POLL_MS=250` 连续轮询 + **跨 `BOOT_BASELINE_MS=2000` 窗口总上升量** `< STABLE_DELTA_MV=150` 判平台（拒绝"仍在缓升"假稳）；`WARMUP_MAX_MS=15s` 兜底，超时若仍 LOW_STOP 先降档 WARN 再业务去抖；② 新增 `bsp_battery_is_ready()`；③ `DIVIDER_RATIO_X10000` 标称 1803 → **实测 1503**（万用表 12 V 对齐）；④ `BOOT_OK_MV` 7000 → **11000**（充满后读值才作提前解锁）。API 四函数不变，`app_safety` BOOT_CHECK 仍调 `is_boot_ok()`。详见 [Stage3 §7E.5](Stage3-BalanceControl.md) 时序说明。 | 主控团队 |
| 2026-06-01 | v0.11 | **Stage 2.9 右编码器迁 TIMG0 捕获中断 ｜ 修复大外力纠偏自转**。**现象**：装车平衡大外力推车后，高倾角速度环纠偏时出现左轮远快于右轮、整车自转一周。**根因**：旧方案右轮 `PA12/PA13` GPIO 双沿中断（X2，34000 cnt/rev）+ `GROUP1_IRQHandler`；500 PPR × 34:1 电机在 ≈530 RPM 出轴侧边沿率 **300 边/ms 恰好等于** `BSP_MOTOR_ENC_IRQ_QUENCH_PER_MS=300` → 雪崩兜底关中断 50 ms → `right_count` 冻结 → 差速环误判 L≫R → 正反馈自转。**约束**：MSPM0G3507 仅 TIMG8 支持硬件 QEI（左轮已占），无第二路 QEI。**修复**（[bsp\_motor.{c,h}](../../template/hardware/bsp_motor.h)）：① PA12 mux 到 `TIMG0_CCP0`（PINCM34 PF=4），`DL_TimerG_initCaptureMode` X1 上升沿捕获；PA13 保持 GPIO，ISR 内读电平判向；② `GROUP1_IRQHandler` 删除，`TIMG0_IRQHandler` 独立向量（startup slot 17）；③ `RIGHT_COUNTS_PER_OUTPUT_REV` 34000→**17000**，`RIGHT_DECODE_X`→**1**，`ENC_IRQ_QUENCH_PER_MS` 300→**1000**；④ 雪崩兜底改为 `DL_TimerG_enable/disableInterrupt(TIMG0, CC0)`；⑤ [LP_MSPM0G3507.syscfg](../../EIDE/LP_MSPM0G3507.syscfg) QEI 注释段标注 TIMG0 已被 BSP 占用。**接线不变**（PA12/PA13 物理脚位同 §7.3）。**文档同步**：顶部 v0.11 提示框；§1 第 2/9 项；§2.1/§2.2 编码器策略；§3.2 速度分辨率；§4.1 宏表（PPR/减速比/cnt/rev/雪崩阈值）；§6.3 验收第 1 项；§7.2/§7.3 接线表；§8 划掉旧 X4/GPIO 路径、补 CAPTURE 已完成。**待上车验证**：手转一圈 ≈17000；大力推车时 `[ISR_QUENCH!]` 不再误触发、自转消失。 | 主控团队 |

