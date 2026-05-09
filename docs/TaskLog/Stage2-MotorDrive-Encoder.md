# 阶段 2 ｜ TB6612 双电机驱动 + 编码器角度日志

> **⚠️ Stage 2.1 BSP 重写提示（2026-05-09）**：[bsp\_motor.{c,h}](../../template/hardware/bsp_motor.h) 在本版次完成**驱动 API 重写**，从原本 7 个最小接口扩展为 8 组共 18 个 API（单轮命令、Coast vs Brake、运行时极性翻转、PWM 上限保护、编码器原始读 / 归零、瞬时速度 cps/dps/rpm 反馈、命令查询、S1 toggle）。**演示层 [app\_motor\_demo.c](../../template/app/app_motor_demo.c) 完全向后兼容**，原有 5 个调用 (`bsp_motor_enable / set_output / consume_toggle_request / update / get_feedback`) 行为不变；`bsp_motor_feedback_t` 仅追加字段不删字段。同期废弃：原顶部宏 `BSP_MOTOR_LEFT_SIGN / RIGHT_SIGN` 不再存在，方向反转改为运行时 `bsp_motor_set_invert()`。详见 §3 全 API 表 与 §9 修订历史 v0.2。
>
> **⚠️ 引脚核对提示（2026-05-09，本文 v0.3）**：本文从分支 A 迁回时引脚信息基于 Stage 0 v0.6 旧版，本分支 Stage 1.6（Stage0 真源 v0.8）已做"引脚集中化重排 + 跳线决策更新"。对照 [Stage0-PinAllocation.md §3.2](Stage0-PinAllocation.md) / [bsp_gpio.h](../../template/hardware/bsp_gpio.h) / [ti_msp_dl_config.h](../../EIDE/ti_msp_dl_config.h) 三处真源逐脚核对结果：**电机 / 编码器 / S1 / LED 等 14 个引脚映射完全一致，无任何错位**；但原文档 §7 接线指导**漏列了 9 个关键 LaunchPad 跳线决策**（J4 / J8 / J12 / J14 / J15 / J17 / J18 / J19 / J20）以及"PA0/PA1 必焊、其余业务脚 BoosterPack 直接接出"的施工指引。本版次 §7 全部表格补 `LQFP 引脚号` 列与 `跳线 / 备注` 列、§7.5 重写为完整跳线核对清单，与真源 Stage 0 §2 / §3.4 一一对应；底层代码 / 业务逻辑无改动。详见 §9 修订历史 v0.3。
>
> 文档定位：本轮交付 `TB6612 + GB370` 电机驱动演示固件，覆盖 PWM 输出、方向切换、编码器角度累计、板载 `S1(PA18)` 按键切换正反转，以及 XDS 调试串口日志输出。Stage 2.1 起底层 BSP 升级为"平衡环 / 速度环可直接接入"的完备模块。
>
> 关联文件：
>
> - 主入口：[main.c](../../template/main.c)
> - 电机底层：[bsp\_motor.h](../../template/hardware/bsp_motor.h)、[bsp\_motor.c](../../template/hardware/bsp_motor.c)
> - 演示任务：[app\_motor\_demo.h](../../template/app/app_motor_demo.h)、[app\_motor\_demo.c](../../template/app/app_motor_demo.c)
> - 引脚真源：[Stage0-PinAllocation.md](Stage0-PinAllocation.md)

***

## 1. 本轮目标

| # | 需求                                                          | 落地结果                                                                 |
| - | ----------------------------------------------------------- | -------------------------------------------------------------------- |
| 1 | `S1(PA18)` 控制电机正反转切换                                        | 已完成，按下一次切换一次，带 80 ms 去抖                                              |
| 2 | 接收编码器信号并主动更新角度到调试串口                                         | 已完成，左轮硬件 QEI，右轮 GPIO 中断，100 ms 打印一次                                  |
| 3 | 形成任务日志文档并给出接线指导                                             | 已完成，见本文 §7                                                           |
| 4 | **【Stage 2.1】驱动 API 重写**：把"够 demo 跑"的最小集升级为平衡环 / 速度环可直接接入 | 已完成，新增 13 个 API（单轮 / brake / invert / pwm\_limit / 编码器原始 / 速度反馈），见 §3 |

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

***

## 4. 默认参数与可调项

### 4.1 编译期宏（[`bsp_motor.h`](../../template/hardware/bsp_motor.h) 顶部）

| 宏                                       | 默认值      | 含义                                                                              |
| --------------------------------------- | -------- | ------------------------------------------------------------------------------- |
| `BSP_MOTOR_PWM_MAX_PERMILLE`            | `1000`   | PWM 命令满量程千分比，**不要改**（其它代码假设这是 1000）                                            |
| `BSP_MOTOR_GB370_GEAR_RATIO`            | `30`     | GB370 减速比                                                                       |
| `BSP_MOTOR_GB370_HALL_PPR`              | `11`     | 电机霍尔每转脉冲数（A 相单沿）                                                                |
| `BSP_MOTOR_LEFT_DECODE_X`               | `4`      | 左轮 QEI mode 3 = X4 解码                                                           |
| `BSP_MOTOR_RIGHT_DECODE_X`              | `2`      | 右轮 PA12 双边沿 = X2 解码（ISR 内 PA13 只读电平判方向）                                         |
| `BSP_MOTOR_LEFT_COUNTS_PER_OUTPUT_REV`  | `1320`   | 自动 = `GEAR × PPR × LEFT_DECODE_X`                                               |
| `BSP_MOTOR_RIGHT_COUNTS_PER_OUTPUT_REV` | `660`    | 自动 = `GEAR × PPR × RIGHT_DECODE_X`                                              |
| `BSP_MOTOR_SPEED_WINDOW_MS`             | `20`     | 速度差分窗口；50 Hz 速度刷新率，最低分辨速度 ≈ 2.3 rpm（左）/ 4.6 rpm（右）。支持 `-D` 命令行覆盖              |
| `BSP_MOTOR_BTN_DEBOUNCE_MS`             | `80`     | S1(PA18) 软件去抖窗口。支持 `-D` 命令行覆盖                                                   |
| `APP_MOTOR_DEMO_PWM_PERMILLE`           | `350`    | 演示占空比（[`app_motor_demo.c`](../../template/app/app_motor_demo.c) 内宏，与 BSP 无关） |

若手头 GB370 减速比 / 霍尔线数不同，只改 `GEAR_RATIO` 与 `HALL_PPR`；如果右轮想升级到 X4 解码（PA13 也开中断），只改 `RIGHT_DECODE_X = 4` 并在 `bsp_motor_init()` 里同步打开 PA13 双沿中断。

### 4.2 运行时可调项（Stage 2.1 新增）

| 调整项     | API                                  | 默认值      | 典型场景                          |
| ------- | ------------------------------------ | -------- | ----------------------------- |
| 左轮极性翻转  | `bsp_motor_set_invert(invL, ...)`    | false    | 装车后发现命令方向与轮子方向相反              |
| 右轮极性翻转  | `bsp_motor_set_invert(..., invR)`    | false    | 同上                            |
| PWM 上限  | `bsp_motor_set_pwm_limit(lim_pm)`    | `1000`   | 电池低压、调试限速、热保护                 |
| 编码器归零   | `bsp_motor_reset_encoders()`         | —        | 上电校准、试跑前清零里程                  |
| STBY 开关 | `bsp_motor_enable(bool)`             | false    | 跌倒保护时拉低、起立后拉高                 |

> **⚠️ 已废弃宏（Stage 2.0 → 2.1）**：原 `bsp_motor.c` 顶部的 `BSP_MOTOR_LEFT_SIGN` / `BSP_MOTOR_RIGHT_SIGN` 编译期符号宏在重写中删除。方向反转改为运行时 `bsp_motor_set_invert()`，避免"调一次方向就要重新编译 / 烧录"的痛点。如旧分支引用了这两个宏，搜索全工程即可发现引用点已为 0。

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

| 编码器引脚     | MSPM0G3507 引脚 | LQFP pin | 接出方式               | 跳线 / 备注                                                                          |
| --------- | ------------- | -------- | ------------------ | -------------------------------------------------------------------------------- |
| `A`       | `PA29`        | 36       | LaunchPad J12 1 脚 | `TIMG8_CCP0`（QEI PHA）；**`J12` 必须 ON**                                            |
| `B`       | `PA30`        | 37       | LaunchPad J12 2 脚 | `TIMG8_CCP1`（QEI PHB）；**`J12` 必须 ON**                                            |
| `Z/Index` | `PB14`        | 2        | LaunchPad J12 3 脚 | `TIMG8_IDX` 3-pin 模式；电机无 Z 相可不接，软件忽略 LOAD 事件；**`J12` 必须 ON**                     |
| `VCC`     | `3V3`         | —        | LaunchPad J28 3V3  | **先确认编码器模块电压等级**（部分 GB370 编码器模块需 5V，标错会烧 GPIO）                                   |
| `GND`     | `GND`         | —        | 共地                 | 与主控共地                                                                            |

### 7.3 右电机编码器

| 编码器引脚 | MSPM0G3507 引脚 | LQFP pin | 接出方式      | 跳线 / 备注                                                                  |
| ----- | ------------- | -------- | ----------- | ------------------------------------------------------------------------ |
| `A`   | `PA12`        | 5        | BP J4.40    | GPIO 双边沿中断输入；BSP 宏 `BSP_ENC_R_A_*`                                       |
| `B`   | `PA13`        | 6        | BP J4.39 邻位 | GPIO，无中断；ISR 内读电平判方向（X2 解码）。如未来升 X4，需在 `bsp_motor_init()` 内追加 PA13 双沿配置 |
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
| `J12` | PA29 / PA30 / PB14 → QEI 接头 | **必须 ON**   | 左轮 QEI 接不到电机编码器，左轮 count 永远为 0                    |
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
- **本阶段所有电机 / 编码器 / S1 / LED 引脚都从 LaunchPad BoosterPack 排针 + J12 编码器接头直接接出，无需焊接**（详见 [Stage0-PinAllocation.md §3.4](Stage0-PinAllocation.md) 必焊清单）。BUZZER (PA0) / LASER_EN (PA1) 因 LaunchPad 板载固有限制需要焊飞线，但本阶段不用，可忽略。
- 若某个电机命令方向与轮子实际方向相反，**Stage 2.1 起首选** `bsp_motor_set_invert(invL, invR)` 运行时翻转（参见 §3 / §4.2）；不要再交换动力线或硬编码改 `AIN/BIN` 真值表，避免出厂调好的极性被下一次重新焊接打乱。
- 若编码器读数方向与轮子实际转向相反（即"轮子正转、count 减少"），同样优先用上一条 invert 的同一调用 —— 它在 BSP 内部对命令侧翻转后，业务层将命令与反馈作差时符号会自动一致；如果调用方需要"反馈也按业务正向"，请在 `bsp_motor_get_feedback()` 之后自行乘 -1，**不要**回到 BSP 层做二次翻转，会破坏"编码器 = 硬件真理"的约定。

***

## 8. 后续建议

Stage 2.1 已完成的（不再列入待办）：

- ~~`bsp_motor` 上层叠加轮速计算（`count delta / dt`），形成速度闭环输入~~ → 已经在 BSP 内置 `feedback.left_speed_dps` 等 6 个速度字段
- ~~跌倒时自动拉低 `STBY` 的保护逻辑~~ → BSP 已提供 `bsp_motor_brake()` + `bsp_motor_enable(false)` 原语，业务层封装 1 行即可

仍然剩余 / 推荐的下一步：

- **右轮升级为 X4 解码**：把 `BSP_MOTOR_RIGHT_DECODE_X` 改 4，并在 `bsp_motor_init()` 内同步打开 PA13 双沿中断（`DL_GPIO_setLowerPinsPolarity(GPIOA, DL_GPIO_PIN_13_EDGE_RISE | DL_GPIO_PIN_13_EDGE_FALL)`）+ 在 `GPIOA_IRQHandler` 中分发 `DL_GPIO_IIDX_DIO13`。这样左右轮分辨率一致（1320 cnt/rev），平衡环里两轮回路系数可以共用一份。
- **接入平衡环 / 速度环**：参考 §3.3 示例骨架，在 100 Hz 节拍调用 `bsp_motor_get_feedback()` + PID + `bsp_motor_set_output()`，1 kHz 节拍只调 `bsp_motor_update()`。
- **电池低压保护与 PWM 自动降功率**：用 `ADC_BAT`（已 syscfg 配好）周期采样 → 阈值触发 `bsp_motor_set_pwm_limit(降级值)`，避免锂电低压拉爆 TB6612。
- **跌倒检测 → brake**：MS901M 姿态环里检测 `|pitch| > 60°` 时调 `bsp_motor_brake()` + `bsp_motor_enable(false)`，状态恢复后由 `bsp_motor_consume_toggle_request()`（S1）手动重启。
- **加 IDLE 节流**：`bsp_motor_brake()` 后建议也设 `bsp_motor_set_pwm_limit(0)` 再过 100 ms 复原，避免 brake 持续注入大电流；现版本 brake 写满 PWM 后保持，TB6612 内部反向二极管路径上仍有微弱续流。

***

## 9. 修订历史

| 日期         | 版本   | 修订内容                                                                                                                                                                                                                                                                                                                                                                                                                | 作者   |
| ---------- | ---- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---- |
| 2026-04-30 | v0.1 | 阶段 2 初版：TB6612 双电机驱动 + 左轮 TIMG8 QEI + 右轮 PA12 双边沿中断 + S1 切换演示固件；含接线表 / 串口日志格式 / 验收建议                                                                                                                                                                                                                                                                                                                              | 主控团队 |
| 2026-05-09 | v0.2 | **Stage 2.1 BSP 重写**：[bsp\_motor.{c,h}](../../template/hardware/bsp_motor.h) 从 7 个最小接口扩展为 8 组共 18 个 API（`set_left/set_right` 单轮控制、`brake` 短刹车区分 `stop` 滑行、`set_invert / get_invert` 运行时极性、`set_pwm_limit / get_pwm_limit` 安全限幅、`get_left_count / get_right_count / reset_encoders` 编码器原始接口、`get_left_cmd / get_right_cmd / is_enabled` 状态查询）。`bsp_motor_feedback_t` 追加 6 个速度字段（cps / dps / rpm × L/R），50 Hz 速度刷新率。1 kHz 路径整数化（浮点延迟到 `get_feedback()`），状态聚合到 `motor_state_t s_motor` 单例并显式划分 ISR 共享段 vs 主循环私有段、用 `MOTOR_LOCK / UNLOCK` PRIMASK 短锁保护。**完全向后兼容**：[app\_motor\_demo.c](../../template/app/app_motor_demo.c) 原 5 个调用 (`enable / set_output / consume_toggle_request / update / get_feedback`) 行为不变，原结构体字段全部保留。**已废弃**：原顶部宏 `BSP_MOTOR_LEFT_SIGN / RIGHT_SIGN` 删除，方向反转改为运行时 `bsp_motor_set_invert()`。文档同步：§1 增第 4 项交付、§2 新增 §2.3 状态聚合 + §2.4 整数化、§3 完全重写为全 API 表 + 反馈字段表 + 平衡环用法骨架 + 极性约定、§4 拆 4.1 编译期宏（增 SPEED\_WINDOW\_MS / DECODE\_X / BTN\_DEBOUNCE\_MS）+ 4.2 运行时可调项 + 已废弃宏提示、§5 把"调宏"改为"调 invert API"、§6 新增 6.2 BSP 回归快速测试矩阵、§7.5 接线注意事项更新极性调整流程、§8 划掉两条已实现的 TODO 并新增 5 条剩余优化项 | 主控团队 |
| 2026-05-09 | v0.3 | **引脚 / 跳线核对（分支迁移后回归）**：本文从分支 A 迁回时引脚信息基于 Stage0 v0.6 旧版，与本分支 Stage 1.6（Stage0 真源 v0.8）"引脚集中化重排"存在信息漂移。对照 [Stage0-PinAllocation.md §3.2](Stage0-PinAllocation.md) v0.8、[bsp\_gpio.h](../../template/hardware/bsp_gpio.h)、[ti\_msp\_dl\_config.h](../../EIDE/ti_msp_dl_config.h) 三处真源逐脚核对：**电机 / 编码器 / S1 / LED 等 14 个引脚映射全部一致，无引脚错位**；但原 §7 接线指导**漏列 9 个关键 LaunchPad 跳线决策**，最严重的两项是 `J17/J18 必须 OFF`（否则 BIN1/BIN2 与板载 OPA0_IN0+/- 抢线，TB6612 接收的方向位电平被拉偏）和 `J14 必须切 (2)-(3) PA9`（默认 (1)-(2) 是 PB23，PWMB 信号到不了 BP J1.3，右电机不转）。本版次改动：① 文档顶部加 v0.3 引脚核对提示框；② §7.1 TB6612 表加 `LQFP pin` 列与 `BoosterPack 接出` 列、`跳线 / 备注` 列，明确 J14/J15/J17/J18 关联；③ §7.2 左编码器表加 LQFP / J12 ON 提示；④ §7.3 右编码器表加 LQFP / BP 排针位 / X4 升级提示；⑤ §7.4 板载资源表扩展，加入 LED_R / SWDIO / SWCLK 行 + 全部 J5/J6/J7/J8/J21/J22/J101 跳线状态；⑥ §7.5 拆为 §7.5.1 跳线核对清单（覆盖 J4/J5/J6/J7/J8/J12/J14/J15/J17/J18/J19/J20/J21/J22/J101 共 15 项，每项含"涉及引脚 / 必须状态 / 不到位的后果"）+ §7.5.2 装车通用注意事项；⑦ §7.5.2 增加"本阶段所有引脚都从 BP / J12 直接接出无需焊接"说明（与 Stage0 §3.4 必焊清单互校）。**底层代码 / 业务逻辑 / 引脚分配本身均无任何改动**，本版次为纯文档同步 | 主控团队 |

