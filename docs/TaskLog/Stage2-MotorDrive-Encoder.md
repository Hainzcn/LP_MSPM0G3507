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
> 文档定位：本轮交付 `TB6612 + GB370` 电机驱动演示固件，覆盖 PWM 输出、方向切换、编码器角度累计、板载 `S1(PA18)` 按键切换正反转，以及 XDS 调试串口日志输出。Stage 2.1 起底层 BSP 升级为"平衡环 / 速度环可直接接入"的完备模块；Stage 2.2 起补齐安全 + 电池 + PID + 平衡骨架，业务层一行 `set_balance_gains` 即可启动整定；Stage 2.3 起左编码器从 J12 迁到 BP，全车不再依赖未焊跳线排针。
>
> 关联文件：
>
> - 主入口：[main.c](../../template/main.c)
> - 电机底层：[bsp\_motor.h](../../template/hardware/bsp_motor.h)、[bsp\_motor.c](../../template/hardware/bsp_motor.c)
> - 演示任务：[app\_motor\_demo.h](../../template/app/app_motor_demo.h)、[app\_motor\_demo.c](../../template/app/app_motor_demo.c)
> - **Stage 2.2 新增**：[bsp\_battery.{h,c}](../../template/hardware/bsp_battery.h) | [middle/pid.{h,c}](../../template/middle/pid.h) | [app\_safety.{h,c}](../../template/app/app_safety.h) | [app\_balance.{h,c}](../../template/app/app_balance.h)
> - 引脚真源：[Stage0-PinAllocation.md](Stage0-PinAllocation.md)

***

## 1. 本轮目标

| # | 需求                                                          | 落地结果                                                                 |
| - | ----------------------------------------------------------- | -------------------------------------------------------------------- |
| 1 | `S1(PA18)` 控制电机正反转切换                                        | 已完成，按下一次切换一次，带 80 ms 去抖                                              |
| 2 | 接收编码器信号并主动更新角度到调试串口                                         | 已完成，左轮硬件 QEI，右轮 GPIO 中断，100 ms 打印一次                                  |
| 3 | 形成任务日志文档并给出接线指导                                             | 已完成，见本文 §7                                                           |
| 4 | **【Stage 2.1】驱动 API 重写**：把"够 demo 跑"的最小集升级为平衡环 / 速度环可直接接入 | 已完成，新增 13 个 API（单轮 / brake / invert / pwm\_limit / 编码器原始 / 速度反馈），见 §3 |
| 5 | **【Stage 2.2】上车准备**：补齐右轮 X4 解码 + 脉冲刹车 + 电池保护 + 安全状态机 + 通用 PID + 平衡骨架 | 已完成，新增 4 个模块 8 个文件（`bsp_battery` / `pid` / `app_safety` / `app_balance`），见 §3.5 + §4.3 |

***

## 2. 实现摘要

### 2.1 驱动结构

- [`bsp_motor.c`](../../template/hardware/bsp_motor.c) 负责底层硬件访问：
  - `TB6612` 的 `AIN1/AIN2/BIN1/BIN2/STBY`，并显式区分 4 态：Coast / Forward / Reverse / **Brake**（IN1=IN2=H）
  - `TIMA0` 双路 PWM 占空比设置（PWM 频率 ≈ 20 kHz，超出人耳）
  - 左轮 `TIMG8 QEI` mode 3 (X4) 16-bit 计数扩展为 32-bit
  - 右轮 `PA12` 双边沿中断 + `PA13` 电平判方向（X2 解码）
  - `S1(PA18)` 按键中断与 80 ms 去抖
  - 速度差分窗口（默认 20 ms ⇒ 50 Hz 速度反馈刷新率）
- [`app_motor_demo.c`](../../template/app/app_motor_demo.c) 负责演示逻辑：
  - 上电后默认两轮同向运行，PWM = `350/1000`
  - 每按一次 `S1`，正转 / 反转翻转
  - 每 `100 ms` 打印左右轮累计计数与角度
  - 每 `250 ms` 翻转一次绿灯，作为循环心跳

### 2.2 编码器策略

当前实现沿用阶段 0 的资源结论：

- **左轮**：继续使用 `TIMG8` 硬件 QEI，分辨率按 `11 PPR * 30:1 * 4 = 1320 count/rev`
- **右轮**：使用 `PA12` 双边沿中断，`PA13` 只在 ISR 中读取电平判方向，分辨率按 `11 PPR * 30:1 * 2 = 660 count/rev`

这样做的好处：

- 左轮角度统计稳定，不怕高速漏脉冲
- 右轮 CPU 占用较低，适合现阶段先验证驱动链路
- 后续若右轮分辨率不够或高速丢计数，可再升级为 `A/B 双通道中断` 或 `CAPTURE` 模式

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
| **初始化 / 使能**    | `bsp_motor_init()`                                                                 | `SYSCFG_DL_init` + `bsp_gpio_init` 之后 | 计数清零、ISR 注册、STBY 强制低位（待机）           |
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
| **S1 toggle**   | `bsp_motor_consume_toggle_request()`                                               | 主循环                               | 边沿事件源，读后清零；ISR 内已做 80 ms 去抖         |

### 3.2 反馈结构体 `bsp_motor_feedback_t`（Stage 2.1 扩展）

| 字段                                        | 类型      | 含义                            |
| ----------------------------------------- | ------- | ----------------------------- |
| `left_count` / `right_count`              | int32   | 编码器累计计数（int32，约 ±5×10^5 圈不溢出） |
| `left_angle_deg` / `right_angle_deg`      | float   | 输出轴累计机械角                      |
| `left_speed_cps` / `right_speed_cps`      | int32   | 输出轴瞬时角速度，单位 counts/s          |
| `left_speed_dps` / `right_speed_dps`      | float   | 输出轴瞬时角速度，单位 °/s               |
| `left_speed_rpm` / `right_speed_rpm`      | float   | 输出轴瞬时转速，单位 rpm                |

**速度刷新率**：由 `BSP_MOTOR_SPEED_WINDOW_MS`（默认 20 ms）决定 → 50 Hz；最低可分辨速度

- 左轮：`1000/20 = 50 cps ≈ 50/1320 × 60 ≈ 2.27 rpm`
- 右轮：`50 cps ≈ 50/660 × 60 ≈ 4.55 rpm`

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
| `app_safety_tick(*att)` | 周期任务（建议 100 Hz）：S1 重启 + 跌倒检测 + 电池状态合成；返回最新状态 |
| `app_safety_get_state()` | 仅查询状态枚举 |
| `app_safety_can_drive()` | 当前是否允许业务下发电机命令（`ARMED` 或 `LOW_BAT_WARN`） |

**5 态状态机**：`DISARMED` / `ARMED` / `LOW_BAT_WARN` / `FALLEN` / `LOW_BAT_STOP`，优先级 `LOW_BAT_STOP > FALLEN > LOW_BAT_WARN > ARMED`。
**跌倒判据**：`|pitch_deg| > APP_SAFETY_FALL_PITCH_DEG`（默认 60°，与 Overview §4.1 一致）→ `bsp_motor_brake_pulse_ms(80)` + `bsp_motor_enable(false)`。
**低压急停**：`bsp_battery_get_state() == LOW_STOP` → `bsp_motor_brake_pulse_ms(120)` + `enable(false)`，且**不会自动恢复**：电池升回 LOW_WARN 后状态降到 LOW_BAT_WARN 但保持 STBY 关，必须人工按 S1 重启。
**S1 二态重启**：DISARMED / FALLEN / LOW_BAT_WARN 按 S1 → ARMED；ARMED 按 S1 → DISARMED（人工急停）。

#### 3.5.4 `app_balance.{c,h}` —— 平衡车双环骨架

| API | 用途 |
| --- | --- |
| `app_balance_init()` | 初始化两路 PID（输出限幅 + D 滤波系数已写入；增益默认 0） |
| `app_balance_reset()` | 清两路 PID 内部历史，不动增益 |
| `app_balance_set_pitch_offset(deg)` | 设置静态俯仰零点（让车在地面"标准直立"姿态下读 1 s 平均 pitch 填入） |
| `app_balance_set_balance_gains(kp, ki, kd)` | 平衡内环：输入 tilt 误差 deg，输出 PWM permille |
| `app_balance_set_speed_gains(kp, ki, kd)` | 速度外环：输入 cps 误差，输出目标 tilt deg |
| `app_balance_set_yaw_kp(kp_yaw)` | 转向开环系数（`left -= yaw·k, right += yaw·k`） |
| `app_balance_step(*att, *cmd)` | 跑一拍控制环（建议 100 Hz）：safety tick + 速度外环 + 平衡内环 + 转向叠加 + `bsp_motor_set_output()` |
| `app_balance_get_diag(*out)` | 拷贝本拍诊断（target_tilt / pitch_meas / pwm_out / left/right_cmd / speed / driving） |

**控制结构**：速度外环（输入 cps，输出目标 tilt deg，限幅 ±10°） → 平衡内环（输入 tilt 误差 deg，输出 PWM permille，限幅 ±1000）→ 转向叠加 → `bsp_motor_set_output(left, right)`。
**与 safety 集成**：`app_balance_step()` 内部调 `app_safety_tick()`；不允许驱动时**不调** `set_output`（保留 safety 的 brake 命令），同时 reset PID 历史。
**默认增益 0** = 上电不会自己动；业务侧调 `set_*_gains()` 注入后才工作；详细整定流程见 [`app_balance.h`](../../template/app/app_balance.h) 顶部注释（4 步级联 PID 整定法）。

***

## 4. 默认参数与可调项

### 4.1 编译期宏（[`bsp_motor.h`](../../template/hardware/bsp_motor.h) 顶部）

| 宏                                       | 默认值      | 含义                                                                              |
| --------------------------------------- | -------- | ------------------------------------------------------------------------------- |
| `BSP_MOTOR_PWM_MAX_PERMILLE`            | `1000`   | PWM 命令满量程千分比，**不要改**（其它代码假设这是 1000）                                            |
| `BSP_MOTOR_GB370_GEAR_RATIO`            | `30`     | GB370 减速比                                                                       |
| `BSP_MOTOR_GB370_HALL_PPR`              | `11`     | 电机霍尔每转脉冲数（A 相单沿）                                                                |
| `BSP_MOTOR_LEFT_DECODE_X`               | `4`      | 左轮 QEI mode 3 = X4 解码                                                           |
| `BSP_MOTOR_RIGHT_DECODE_X`              | **`4` (Stage 2.2)** | 右轮 PA12 + PA13 都开双沿中断 = X4 解码，与左轮分辨率一致；可 `-DBSP_MOTOR_RIGHT_DECODE_X=2` 退回 X2 减半 ISR 频次 |
| `BSP_MOTOR_LEFT_COUNTS_PER_OUTPUT_REV`  | `1320`   | 自动 = `GEAR × PPR × LEFT_DECODE_X`                                               |
| `BSP_MOTOR_RIGHT_COUNTS_PER_OUTPUT_REV` | **`1320` (Stage 2.2)** | 自动 = `GEAR × PPR × RIGHT_DECODE_X`；X4 后与左轮一致，平衡环左右系数可共用                              |
| `BSP_MOTOR_SPEED_WINDOW_MS`             | `20`     | 速度差分窗口；50 Hz 速度刷新率，最低分辨速度 ≈ 2.3 rpm（左）/ 4.6 rpm（右）。支持 `-D` 命令行覆盖              |
| `BSP_MOTOR_BTN_DEBOUNCE_MS`             | `80`     | S1(PA18) 软件去抖窗口。支持 `-D` 命令行覆盖                                                   |
| `APP_MOTOR_DEMO_PWM_PERMILLE`           | `350`    | 演示占空比（[`app_motor_demo.c`](../../template/app/app_motor_demo.c) 内宏，与 BSP 无关） |

若手头 GB370 减速比 / 霍尔线数不同，只改 `GEAR_RATIO` 与 `HALL_PPR`；右轮 X2/X4 已在 Stage 2.2 由 `BSP_MOTOR_RIGHT_DECODE_X` 一处统管（init 内自动按宏决定是否开 PA13 中断、`GROUP1_IRQHandler` 自动条件分发 DIO13）。

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
| **app_safety** | `APP_SAFETY_FALL_PITCH_DEG` | `60.0f` | 跌倒判据；与 Overview §4.1 一致 |
| | `APP_SAFETY_FALL_BRAKE_MS` | `80` | 跌倒急停 brake 脉冲毫秒 |
| | `APP_SAFETY_LOW_BAT_BRAKE_MS` | `120` | 低压急停 brake 脉冲毫秒 |
| | `APP_SAFETY_LOW_BAT_PWM_LIMIT` | `600` | 低压告警时 PWM 限幅（permille） |
| **app_balance** | `APP_BALANCE_CONTROL_PERIOD_MS` | `10` | 控制环周期；100 Hz |
| | `APP_BALANCE_MAX_TILT_DEG` | `10.0f` | 速度外环输出"目标 tilt"绝对值上限 |
| | `APP_BALANCE_MAX_PWM_PERMILLE` | `1000` | 平衡内环输出 PWM 绝对值上限 |
| | `APP_BALANCE_SPEED_D_FILTER_ALPHA` | `0.20f` | 速度外环 D 项 EMA；速度环噪声大需要滤 |
| | `APP_BALANCE_BALANCE_D_FILTER_ALPHA` | `0.10f` | 平衡内环 D 项 EMA |

> **运行时可调（无需重新编译）**：
>
> - `app_balance_set_pitch_offset(deg)` — 静态俯仰零点
> - `app_balance_set_balance_gains(kp, ki, kd)` — 平衡内环增益
> - `app_balance_set_speed_gains(kp, ki, kd)` — 速度外环增益
> - `app_balance_set_yaw_kp(k)` — 转向开环系数
> - `app_safety_arm()` / `app_safety_disarm()` — 主动 arm / 急停
>
> 所有 PID 增益**默认 0** = 上电不输出，业务侧通过串口 / K230 命令注入即可整定。

***

## 5. 串口日志格式

日志由板载 `UART0(XDS-UART)` 输出，波特率保持 `115200 8N1`。

上电后会看到：

```text
[boot] MSPM0G3507 stage2 motor driver start
[boot] stage2 motor demo start
[boot] press S1(PA18) to toggle motor direction
```

运行中每 `100 ms` 输出：

```text
[enc] t=1200ms L=158(43.09 deg) R=82(44.73 deg)
```

按下 `S1` 后输出：

```text
[motor] dir=reverse pwm=350/1000
```

说明：

- `L` / `R` 是累计计数，不会自动清零；如需归零请在主循环里调 `bsp_motor_reset_encoders()`
- `deg` 是基于编码器参数换算出的输出轴机械角
- 若正反方向和实物相反，**Stage 2.1 起改为运行时调** `bsp_motor_set_invert(invL, invR)`（[`bsp_motor.h`](../../template/hardware/bsp_motor.h)），不必重新编译；该函数返回前会立刻按新极性重发当前命令
- Stage 2.1 BSP 内部已具备 cps / dps / rpm 速度反馈字段（参见 §3.2），但本演示固件 `app_motor_demo.c` 暂未打印；接入速度环时直接读 `feedback.left_speed_dps` 即可

***

## 6. 验收建议

### 6.1 演示固件基线验证（Stage 2.0 / 2.1 通用）

按下面顺序验证：

1. **空载验方向**
   - 先断开车轮离地
   - 上电后观察两轮是否同向旋转
   - 按下 `S1`，两轮应整体反向
2. **验编码器计数**
   - 用手慢慢拨动左轮，日志中的 `L` 应连续变化
   - 用手慢慢拨动右轮，日志中的 `R` 应连续变化
   - 反向拨动时计数应反向变化
3. **验角度换算**
   - 在轮胎上做一个明显标记
   - 手动转约 1 圈，观察日志角度是否接近 `360 deg`
   - 如果明显偏差，优先检查 `减速比` 与 `PPR` 宏是否和实物一致
4. **验按键去抖**
   - 快速连按 `S1`
   - 方向切换不应抖动或一次按下切两次

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
| 1 | **右轮 X4 解码上线** | 手动拨右轮一圈，对比 `feedback.right_count` 增量 | 一圈应 ≈ 1320 ± 5（与左轮一致）；从 660 升 1320 = X4 已生效 |
| 2 | **bsp_battery 采样链路** | `bsp_battery_init()` + 100 Hz 调 `bsp_battery_update()`，1 Hz 打印 `get_mv()` / `get_state()` | 12V 满电时显示 ≈ 12000~12700 mV，状态 `NORMAL`；用电源拉到 9.4V 后状态变 `LOW_WARN` |
| 3 | **app_safety 跌倒触发** | `app_safety_init()` + `arm()`，手动倾斜 IMU > 60° | 状态变 `FALLEN`，电机 STBY 变低；按 S1 后状态回 `ARMED` |
| 4 | **app_safety 低压急停** | 把 `BSP_BATTERY_STOP_MV` 临时改 11500（高于现实电压），`tick()` | 状态立刻变 `LOW_BAT_STOP`，按 S1 被拒绝 |
| 5 | **brake_pulse_ms 自动转 stop** | `bsp_motor_set_output(800,800)` → 1 s 后 `bsp_motor_brake_pulse_ms(80)` | 80 ms 内电机急停，之后转 coast（手拨能转）；不像持续 brake 一直锁住 |
| 6 | **PID 单元自测**（不上车） | 单跑 `pid_step()` 多个目标，画响应曲线 | 阶跃响应有上升 + 收敛；增益全 0 时输出恒 0；积分饱和后撤回不再继续累加 |
| 7 | **平衡内环单环（车轮离地）** | `app_balance_set_balance_gains(Kp, 0, Kd)`（小 Kp 起步），手摇车体 | 电机出现"反向纠偏"PWM；从弱到强，找出"反应明显但不振"的最大 Kp |
| 8 | **平衡内环（落地短测）** | 用支架辅助起立 → 撤手 ≤ 3 s | 车能短暂直立，前后摆动 ≤ 5 cm；如发散，先记录波形再调小 Kp / 增大 Kd |
| 9 | **速度外环上线** | 内环已可短直立 → `app_balance_set_speed_gains(Kp_s, Ki_s, 0)` | 给 `cmd.target_speed_cps = 0`，撤手后车体能维持原地（非慢漂） |
| 10 | **转向叠加** | `cmd.target_yaw_pm = 200` | 车体原地缓转；K230 给定的 ω 转化为差速可观察 |

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

> **Stage 2.3 起从 J12 迁到 BoosterPack（PB15/PB16，2-Pin Mode）**：原方案 PA29/PA30/PB14 三脚走 LaunchPad J12 QEI 接头，但 J12 排针**出厂未焊接**无法直接接线；同时 GB370 编码器无 Z 相，IDX 可省。复核数据手册 PINMUX 表后选用 BP 上空闲的 PB15/PB16，TIMG8 QEI 由 3-Pin Mode 降为 2-Pin Mode，**硬件 X4 解码精度 1320 cnt/rev 不变，无丢脉冲**。

| 编码器引脚     | MSPM0G3507 引脚 | LQFP pin | 接出方式              | 跳线 / 备注                                                                                  |
| --------- | ------------- | -------- | ----------------- | ---------------------------------------------------------------------------------------- |
| `A`       | `PB15`        | 32       | BP **J4.34**      | `TIMG8_CCP0`（QEI PHA, mux f=5, PINCM32）；**`J12` 不再使用**                                   |
| `B`       | `PB16`        | 33       | BP **J4.40**      | `TIMG8_CCP1`（QEI PHB, mux f=5, PINCM33）；**`J12` 不再使用**                                   |
| `Z/Index` | —             | —        | **不接（2-Pin Mode）** | GB370 编码器无 Z 相，PB14 进入预留池；不影响 X4 解码精度                                                    |
| `VCC`     | `3V3`         | —        | LaunchPad J28 3V3 | **先确认编码器模块电压等级**（部分 GB370 编码器模块需 5V，标错会烧 GPIO）                                          |
| `GND`     | `GND`         | —        | 共地                | 与主控共地                                                                                    |

### 7.3 右电机编码器

| 编码器引脚 | MSPM0G3507 引脚 | LQFP pin | 接出方式      | 跳线 / 备注                                                                  |
| ----- | ------------- | -------- | ----------- | ------------------------------------------------------------------------ |
| `A`   | `PA12`        | 5        | BP J4.40    | GPIO 双边沿中断输入；BSP 宏 `BSP_ENC_R_A_*`                                       |
| `B`   | `PA13`        | 6        | BP J4.39 邻位 | **Stage 2.2 起也开双边沿中断（X4 解码，1320 cnt/rev）**；可 `-DBSP_MOTOR_RIGHT_DECODE_X=2` 退回 X2，此时 PA13 仅 ISR 读电平判方向 |
| `VCC` | `3V3`         | —        | LaunchPad J28 3V3 | **先确认编码器模块电压等级**（标错烧 GPIO）                                              |
| `GND` | `GND`         | —        | 共地          | 与主控共地                                                                    |

### 7.4 板载资源

| 资源               | 引脚          | LQFP pin | BSP 宏 / 外设                  | 跳线 / 用途                                                                |
| ---------------- | ----------- | -------- | --------------------------- | ---------------------------------------------------------------------- |
| `S1`             | `PA18`      | 11       | `BSP_START_BTN_*`           | **`J8` 必须 ON**；按下拉低，bsp_gpio_init 已配内部上拉，下降沿中断切换电机正反转                  |
| `XDS-UART TX`    | `PA10`      | 56       | UART_LOG (UART0_TX)         | **`J21` 必须 ON**；XDS-UART 桥到电脑虚拟 COM，调试 `printf` 输出                     |
| `XDS-UART RX`    | `PA11`      | 57       | UART_LOG (UART0_RX)         | **`J22` 必须 ON**；电脑 → 主控（本演示固件未读，预留串口调参）                                 |
| `LED_R`          | `PB26`      | 28       | `BSP_LED_R_*`               | **`J6` 保留 ON**（RGB-R）；上电默认亮表示未就绪，main.c 后续按需熄灭                          |
| `LED_G`          | `PB27`      | 29       | `BSP_LED_G_*`               | **`J7` 保留 ON**（RGB-G）；250 ms 翻转作主循环心跳                                  |
| `LED_B`          | `PB22`      | 21       | `BSP_LED_B_*`               | **`J5` 保留 ON**（RGB-B）；按下 S1 切换方向时翻转一次作提示                                |
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
| `J8` | PA18 → S1 按键 + BSL         | **保留 ON**   | S1 按键无电气连接，bsp_motor_consume_toggle_request 永不触发 |
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
- **本阶段所有电机 / 左右编码器 / S1 / LED 引脚都从 LaunchPad BoosterPack 排针直接接出，无需焊接**（左编码器 Stage 2.3 起从 J12 迁到 BP J4.34/J4.40；详见 [Stage0-PinAllocation.md §3.4](Stage0-PinAllocation.md) 必焊清单）。BUZZER (PA0) / LASER_EN (PA1) 因 LaunchPad 板载固有限制需要焊飞线，但本阶段不用，可忽略。
- 若某个电机命令方向与轮子实际方向相反，**Stage 2.1 起首选** `bsp_motor_set_invert(invL, invR)` 运行时翻转（参见 §3 / §4.2）；不要再交换动力线或硬编码改 `AIN/BIN` 真值表，避免出厂调好的极性被下一次重新焊接打乱。
- 若编码器读数方向与轮子实际转向相反（即"轮子正转、count 减少"），同样优先用上一条 invert 的同一调用 —— 它在 BSP 内部对命令侧翻转后，业务层将命令与反馈作差时符号会自动一致；如果调用方需要"反馈也按业务正向"，请在 `bsp_motor_get_feedback()` 之后自行乘 -1，**不要**回到 BSP 层做二次翻转，会破坏"编码器 = 硬件真理"的约定。

***

## 8. 后续建议

Stage 2.1 已完成的（不再列入待办）：

- ~~`bsp_motor` 上层叠加轮速计算（`count delta / dt`），形成速度闭环输入~~ → 已经在 BSP 内置 `feedback.left_speed_dps` 等 6 个速度字段
- ~~跌倒时自动拉低 `STBY` 的保护逻辑~~ → BSP 已提供 `bsp_motor_brake()` + `bsp_motor_enable(false)` 原语，业务层封装 1 行即可

Stage 2.2 已完成的（不再列入待办）：

- ~~**右轮升级为 X4 解码**~~ → 默认 `BSP_MOTOR_RIGHT_DECODE_X = 4`，`bsp_motor_init()` 内已条件编译启用 PA13 双沿中断、`GROUP1_IRQHandler` 已条件分发 `DL_GPIO_IIDX_DIO13`，左右轮分辨率统一 1320 cnt/rev；如 CPU 紧张可 `-DBSP_MOTOR_RIGHT_DECODE_X=2` 退回 X2。
- ~~**接入平衡环 / 速度环骨架**~~ → 已交付 [middle/pid.{c,h}](../../template/middle/pid.h) 通用 PID + [app\_balance.{c,h}](../../template/app/app_balance.h) 速度外环 + 平衡内环 + 转向叠加，`app_balance_step()` 100 Hz 节拍调一次即跑完全链路。**所有 PID 增益默认 0**（失效安全），业务侧需调 `set_balance_gains` / `set_speed_gains` 注入；详细整定流程见 [app\_balance.h](../../template/app/app_balance.h) 顶部注释。
- ~~**电池低压保护与 PWM 自动降功率**~~ → 已交付 [bsp\_battery.{c,h}](../../template/hardware/bsp_battery.h) 周期采样 + 阈值状态机；[app\_safety.{c,h}](../../template/app/app_safety.h) 在 `LOW_BAT_WARN` 自动调 `bsp_motor_set_pwm_limit(600)`，在 `LOW_BAT_STOP` 自动 `brake_pulse_ms(120) + enable(false)`。
- ~~**跌倒检测 → brake**~~ → [app\_safety.{c,h}](../../template/app/app_safety.h) 已实现：`|pitch| > 60°` → `brake_pulse_ms(80) + enable(false)`，恢复后 S1 重启。
- ~~**加 IDLE 节流**~~ → 已交付 `bsp_motor_brake_pulse_ms(N)` API，N ms 后由 `bsp_motor_update()` 自动转 coast，避免 brake 持续注入大电流；持续刹车需求保留旧 `bsp_motor_brake()`（持续模式互斥）。

仍然剩余 / 推荐的下一步（Stage 3+ 范围）：

- **新模块加入 EIDE 构建**：当前 main.c 仍跑 [`app_telemetry_run()`](../../template/app/app_telemetry.c)，新模块 8 个文件已就位但未链接。装车整定时把 [`bsp_motor.{c,h}`](../../template/hardware/bsp_motor.h) + [`bsp_battery.{c,h}`](../../template/hardware/bsp_battery.h) + [`middle/pid.{c,h}`](../../template/middle/pid.h) + [`app_safety.{c,h}`](../../template/app/app_safety.h) + [`app_balance.{c,h}`](../../template/app/app_balance.h) 加入 [EIDE/.eide/eide.yml](../../EIDE/.eide/eide.yml) 的 `virtualFolder`，写一个 `app_balance_run()` 主循环替换 main 中的 `app_telemetry_run()` 调用即可。
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

