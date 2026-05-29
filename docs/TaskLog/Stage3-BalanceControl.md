# 阶段 3 ｜ 平衡控制 —— 从单环 PD 到四级级联多速率架构

> **🛑 Stage 3.0 整定起点提示（2026-05-12）**：Stage 2 交付的 [app_balance.{c,h}](../../template/app/app_balance.h) 仅含"速度外环 + 平衡内环"双环骨架，PID 增益默认 0、上电不输出。阶段 3 的首要任务是把这套骨架从"能编译"变为"能站稳"。为支撑高频迭代，本轮同步构建了 **串口实时 CSV 数据流（`lt_stream`）+ PyQt6 串口绘图工具（`serial_plot.py`）** 作为调试基础设施。
>
> **🧭 Stage 3.1 电机死区补偿升级（2026-05-12，`e98a98f`）**：在正式整定平衡环前，先解决 BSP 层"单门槛死区导致零速阶跃"的根本缺陷。双门槛（静摩擦 / 动摩擦）+ 四路分散补偿 + 运行时开关，使 PID 在零附近能线性可控。详见 §2。
>
> **🔄 Stage 3.2 Yaw 角度环（2026-05-13，`bc3bef7`）**：为抑制原地偏航漂移，新增 Yaw PID 控制器，支持 EKF 绝对偏航角 / 陀螺积分两种数据源切换；同步优化死区补偿（新增 `static_dz_enabled` 开关）、调整 demo 最大转速至 300 rpm。详见 §3。
>
> **📐 Stage 3.3 D 项分离 + 速度反馈 LPF（2026-05-13，`ce33237`）**：将平衡内环的 D 项从 PID 库分离，直接使用陀螺仪角速率计算（绕过 pitch 角 LPF 的相位延迟）；新增速度反馈低通滤波器以抑制编码器 20 ms 窗口量化噪声。详见 §4。
>
> **🏗️ Stage 3.4 四级级联重构（2026-05-14，`0542060`）**：将原有双环（速度 + 平衡）重构为 **速度外环（20 Hz）→ 角度环（100 Hz）→ 角速度内环（200 Hz）→ 电机输出 + 航向环（50 Hz）差分叠加** 的四级多速率架构，每环独立 PID + 独立周期。详见 §5。
>
> **🔧 Stage 3.5 角速率 LPF + sigma-delta dither（2026-05-16，`79b9348`）**：在角速度内环引入 EMA 低通滤波器抑制电机振动耦合的高频噪声；在 BSP 层新增 sigma-delta dither 机制，用脉冲密度调制替代简单死区映射，使子死区命令能平滑转换为电机运动。详见 §6。
>
> **🔁 Stage 3.6 速度反馈极性翻转 + 文档瘦身（2026-05-17，`b8a5783`）**：新增速度反馈极性翻转支持（`si0/si1` 串口命令 / `APP_BALANCE_SPEED_INVERT` 宏），允许在线切换编码器方向与平衡环的符号关系；引入速度反馈低通 LPF（`APP_BALANCE_SPEED_LPF_ALPHA=0.15`）；引入速度量纲缩放（`APP_BALANCE_SPEED_CPS_SCALE=10`）；清理 `docs/chore/` 下过时参考文件、`tools/motor_calib/` 下旧 PNG 与 log。详见 §7。
>
> **🏗️ Stage 3.7 两级级联重构 + 航向角环收敛（2026-05-21，`adc5344`，`pid2` 分支）**：对照 STM32 demo 将俯仰控制从四级级联（含独立角速度内环 `rp`）**收敛为两级串级**：速度外环（20 Hz）→ 角度环（100 Hz，**直接输出 PWM**）。控制器库切换为 **`pid2_t`**（积分按拍累加、微分先行、OutOffset 死区补偿）；平衡模式 **关闭 BSP 死区重映射**，改由角度环 `bo`/`bp` offset 突破静摩擦。横摆仅保留 **航向角环 `yp`**（EKF / 陀螺积分双源，20 Hz），**移除轮速差转向环 `tp`**。串口整定命令：`bo` / `bp` / `sp` / `yp`（无 `rp`）。详见 **§7A**。
>
> **🔄 Stage 3.8 差速闭环 + EMA 滤波（2026-05-24~26，`03dc3a3` / `412800f` / `f8dba1e`）**：新增 **差速环 `diff_pid`**（20 Hz），航向角环 `yp` 输出不再直接叠加 PWM 差分，而是作为差速环的 `target_dif_cps`；差速环输出再叠加到左右 PWM，形成 **航向角环 → 差速环 → 左右 PWM** 的三级链。同步引入速度/差速**目标 EMA 低通**（防阶跃振荡）、差速环 RPM 量纲化、差速方向校正。详见 **§7B**。
>
> **📐 Stage 3.9 圆弧运动演示 + 车体参数集中管理（2026-05-24，`e3e7e50`）**：新增 `app_circle_demo` 子任务（寄生 20 Hz 分支），串口 `c`/`circle` 触发指定直径+速度的整圆运动，三重判停（IMU 偏航 360° / 弧长 / 超时兜底）。新建 `robot_param.h` 集中管理车体几何参数（轮径 65 mm / 轴距 185 mm）与运动学换算（mm/s ↔ avg_cps ↔ 弧长），消除代码中散落的魔法数字。详见 **§7C**。
>
> **🔧 Stage 3.10 自校零开关 + 栈修复 v2.0 + RPM 量纲化（2026-05-24~26）**：新增 `APP_BALANCE_PITCH_AUTOZERO_ENABLE` 总开关、`PITCH_OFFSET_DEFAULT_DEG` 硬编码回退值。栈从 1 KB 扩至 2 KB + canary 防护（`76d7d30`）。差速环量纲由归一化 cps 切换为 RPM。详见 **§7D**。
>
> **🏁 Stage 3.11 赛道模式 + 自立双 PID（2026-05-30）**：新增 `app_track` 主控状态机（自立→循线→判圈→暂停→停车），MCU 为总指挥；自立段专用 rise PID（猛起→阻尼减速→稳定），摆稳后切运动 PID；`VEHICLE_STATUS` 上报 `track_phase/lap` 与 K230 阶段闸门对齐；自检 `ARMED` 后默认等 K230 在线再起立。详见 **§7E**。
>
> 文档定位：阶段 3 自 2026-05-11 起演进；**当前装车控制架构以 Stage 3.11 为准**（含赛道模式）。Stage 3.1~3.6 记录仍保留供对照，其中 §5 四级级联、`rp` 角速度内环、50 Hz 航向调度等描述**已被 Stage 3.7 取代**。
>
> 关联文档：
>
> - 项目总览与执行计划：[docs/Overview/Overview.md](../Overview/Overview.md)
> - 阶段 2 电机驱动与编码器（平衡骨架起点）：[Stage2-MotorDrive-Encoder.md](Stage2-MotorDrive-Encoder.md)
> - 引脚分配真源：[Stage0-PinAllocation.md](Stage0-PinAllocation.md)
> - PID 调参指南：[docs/notes/PIDTuningGuide.md](../notes/PIDTuningGuide.md)
> - 串口绘图工具：[tools/serial_plot.py](../../tools/serial_plot.py)
> - 核心源文件：[app_balance.{c,h}](../../template/app/app_balance.h) | [bsp_motor.{c,h}](../../template/hardware/bsp_motor.h) | [main.c](../../template/main.c)

---

## 1. 任务回顾与决策摘要

阶段 3 在 [Overview.md:106-109](../Overview/Overview.md) 共三件事：

| # | 阶段 3 任务 | 落地结果 |
|---|-----------|---------|
| 1 | 静止状态先调俯仰平衡环（PD 起步，后期可升级为 LQR） | **完成** —— Stage 3.7 两级串级（速度/角度 + 航向角），`pid2_t` + 串口 `bo/bp/sp/yp` |
| 2 | 叠加速度环（外环 PI 输出俯仰参考角 → 内环平衡环），消除贴地慢漂 | **完成** —— 速度外环 20 Hz，输出目标 tilt deg，含 LPF + 极性翻转 + 量纲归一化 |
| 3 | 验收：原地直立 ≥ 3 s，前后偏移 ≤ 10 cm | **整定中** —— 两级架构已就绪，串口 `bo/bp/sp/yp/dp` 注入增益，配合 `serial_plot.py` 可视化闭环整定 |
| — | **Stage 3.8** 差速闭环 | **完成** —— 航向角环 → `diff_pid` → PWM 三级级联，串口 `dp`，EMA 目标/测量双滤波 |
| — | **Stage 3.9** 圆弧运动演示 | **完成** —— `app_circle_demo` + `robot_param.h`，串口 `c`/`circle`/`cx` |
| — | **Stage 3.10** 自校零开关 + 栈修复 | **完成** —— `AUTOZERO_ENABLE` 默认关闭；栈 2 KB + canary |
| — | **Stage 3.11** 赛道模式 + 自立双 PID | **代码完成，联调中** —— `app_track` 状态机 + rise/motion 双增益 + K230 阶段闸门 |

> **架构演进路线图**（各版本里程碑）：
>
> ```
> Stage 2.2 (v0.4)              双环骨架（速度外环 + 平衡内环）      增益全 0 不输出
> Stage 3.1 (e98a98f)           BSP 双门槛死区                        四路分散补偿
> Stage 3.2 (bc3bef7)           + Yaw 角度环                          双源切换
> Stage 3.3 (ce33237)           D 项分离 + 速度 LPF                   角速率直通
> Stage 3.4 (0542060)           四级级联多速率架构                     20/50/100/200 Hz
> Stage 3.5 (79b9348)           + 角速率 LPF + sigma-delta dither     抗振动 + 子死区平滑
> Stage 3.6 (b8a5783)           + 速度极性翻转 + 量纲归一化             串口 si0/si1
> Stage 3.7 (adc5344)           两级级联 + pid2 + 航向角环             移除 rp/tp
> Stage 3.8 (03dc3a3/412800f)   + 差速闭环 + EMA 目标/测量滤波         航向环→diff_pid 级联
> Stage 3.9 (e3e7e50)           + 圆弧运动演示（circle demo）          三重判停 + robot_param
> Stage 3.10 (f527a3d/f8dba1e)  + 自校零开关 + 差速环 RPM 量纲化
> Stage 3.11 (2026-05-30)       + 赛道模式 app_track + 自立 rise PID      当前架构
> ```

---

## 2. Stage 3.1 ｜ 电机死区补偿升级（双门槛 + 四路分散）

> 提交：`e98a98f`（2026-05-12），变更 11 文件，+1187/-835。

### 2.1 问题背景

Stage 2 的 `bsp_motor` 使用**单门槛死区补偿**（`BSP_MOTOR_DEADZONE_COMP_PM`）：任何非零命令都直接加一个固定偏移量。这在调试期勉强可用，但进入平衡环后暴露两个致命缺陷：

1. **零速阶跃**：PID 输出 ±1‰ 时，BSP 直接输出 ±(1 + DZ) ≈ ±60‰ → 电机从 0 跳到 10 RPM，PID 在零附近无法线性可控。
2. **四路不对称**：左正/左反/右正/右反的物理死区各不相同（50/68/50/54‰），一刀切会导致某方向先突破、出现"小倾角单侧反转"。

### 2.2 双门槛方案

BSP 层按编码器运动状态拆为两阶段：

| 阶段 | 判定条件 | 补偿方式 | 参数宏前缀 |
|------|---------|---------|-----------|
| **静摩擦突破** | 编码器连续 `STATIC_RETRY_NO_MOTION_MS`（80 ms）无计数变化 | 加完整静摩擦偏移（~60‰） | `*_DEADZONE_PM` |
| **动摩擦维持** | 编码器已确认在转 | 线性映射，最小输出抬到动摩擦门槛（~40‰） | `*_RUNNING_DEADZONE_PM` |

切换逻辑：`bsp_motor_update()` 在 1 kHz 节拍内监控编码器计数变化 → 一旦检测到运动（左/右任一编码器变化），立即从静摩擦切到动摩擦模式 → 计数停转超过 80 ms 后切回静摩擦。

### 2.3 四路分散补偿

```c
BSP_MOTOR_LEFT_FORWARD_DEADZONE_PM     = 60   /* 左正转静摩擦 */
BSP_MOTOR_LEFT_REVERSE_DEADZONE_PM     = 60   /* 左反转静摩擦 */
BSP_MOTOR_RIGHT_FORWARD_DEADZONE_PM    = 60   /* 右正转静摩擦 */
BSP_MOTOR_RIGHT_REVERSE_DEADZONE_PM    = 60   /* 右反转静摩擦 */

BSP_MOTOR_LEFT_FORWARD_RUNNING_DEADZONE_PM   = 40  /* 左正转动摩擦 */
BSP_MOTOR_LEFT_REVERSE_RUNNING_DEADZONE_PM   = 40
BSP_MOTOR_RIGHT_FORWARD_RUNNING_DEADZONE_PM  = 40
BSP_MOTOR_RIGHT_REVERSE_RUNNING_DEADZONE_PM  = 40
```

同时新增运行时开关：
- `bsp_motor_set_static_dz_enabled(bool)` —— 关闭静摩擦补偿（平衡车模式，依赖倒立摆物理特性 + D 项角速率响应自然突破静摩擦）
- `bsp_motor_set_running_dz_enabled(bool)` —— 关闭动摩擦补偿（校准扫描用，看原始未补偿曲线）

### 2.4 配套工具更新

- `tools/motor_calib/serial_capture.py`：增强校准扫描脚本，支持双门槛模式下的逐路标定
- `tools/motor_calib/log.txt`：更新为 500 PPR × 34:1 新电机的完整校准数据
- `tools/serial_plot.py`：同步更新以支持新的 CSV 协议格式

---

## 3. Stage 3.2 ｜ Yaw 角度环

> 提交：`bc3bef7`（2026-05-13），变更 10 文件，+1036/-769。

### 3.1 动机

Stage 2.2 的平衡骨架没有航向约束。装车实测发现：即使两轮命令完全对称（`left = right`），车体仍会缓慢原地偏航漂移——根源是左右电机绕组不对称、轮径差异、地面摩擦不均等硬件因素累积。

### 3.2 实现

在 `app_balance` 中新增第四路 PID 控制器 `yaw_pid`，输入偏航角误差（°），输出差分 PWM permille（正值 = 左轮加速 / 右轮减速）。

**数据源双模切换**（编译期 `APP_BALANCE_YAW_SOURCE`）：

| 模式 | 值 | 数据源 | 优点 | 缺点 |
|------|---|--------|------|------|
| EKF 绝对角 | 0 | MS901M `yaw_deg` [-180,180]° | 长期不漂移 | ±180° 跳变风险、受磁场干扰 |
| 陀螺积分 | 1 | `gz_dps × dt` 累积 | 无跳变、无磁干扰、天然 D 项阻尼 | 零漂 ~0.3°/s，长期偏移 |

**EKF 模式（源 0）核心技巧**：用"virtual measured"技巧避免 ±180° 跳变——`virtual_meas = target - wrap_180(target - yaw_deg)`，使输入 PID 的测量值在目标角附近连续变化。

**陀螺积分模式（源 1）核心技巧**：直行时持续积分 `gz_dps × dt`，以"归零积分量"为目标；转向时清零重新追踪；D 项天然等效于 `-Kd × gz_dps` 角速率阻尼。当前默认 `APP_BALANCE_YAW_SOURCE=1`。

### 3.3 电机演示模式适配

`app_motor_demo` 最大转速从 720 rpm 降至 **300 rpm**，为平衡整定腾出安全速度空间；串口调速步长同步缩小。

---

## 4. Stage 3.3 ｜ D 项分离 + 速度反馈 LPF

> 提交：`ce33237`（2026-05-13），变更 10 文件，+838/-717。

### 4.1 平衡内环 D 项分离

原因：`pid_step()` 的 D 项输入是 `(target - measured)` 的差分，而 `measured = pitch_meas` 已经过俯仰角 LPF（`APP_BALANCE_PITCH_LPF_ALPHA`），LFP 引入的相位延迟使 D 项响应滞后。

**改为**：平衡内环 D 项不再经 PID 库，直接用陀螺仪角速率 `pitch_rate_dps` 乘以 `kd` 计算：

```c
// 角速度内环（旧：D 项经 PID 库，输入 pitch 误差）
float pwm_out = pid_step(&rate_pid, target_rate_dps, measured_rate, dt);
// → PID 库内部 D 项 = kd * d(target - measured)/dt

// 角速度内环（新：D 项直出，输入角速率误差）
// rate_pid.kd 始终设 0，D 项单独计算：
float d_term = kd_rate * (target_rate_dps - measured_rate);
// 其中 measured_rate = pitch_rate_dps（未经任何 LPF，零相位延迟）
```

效果：D 项响应零延迟，整定直觉更好（Kd 直接对应角速率阻尼）。

### 4.2 速度反馈低通滤波器

编码器 20 ms 差分窗口在低速时量化噪声严重（最小分辨 50 cps），直通到速度外环会导致 PWM 抖动。

新增 `APP_BALANCE_SPEED_LPF_ALPHA`（初值 0.5），对 `avg_cps` 做一阶 EMA 滤波：

```c
s_bal.speed_lpf_cps += alpha * (norm_cps - s_bal.speed_lpf_cps);
```

### 4.3 串口工具适配

`tools/serial_plot.py` 新增独立 PID 参数设置按钮（bp/sp/rp 三环），支持角度环、速度环、角速度环分别设参。

---

## 5. Stage 3.4 ｜ 四级级联多速率架构

> 提交：`0542060`（2026-05-14），变更 11 文件，+1173/-1016。**本次是阶段 3 最大规模的重构**。

### 5.1 架构设计

将原有双环（速度外环 → 平衡内环）重构为四级级联：

```
速度外环 (20 Hz)  输入: speed 误差 cps    → 输出: target_tilt_deg
    ↓
角度环   (100 Hz) 输入: tilt 误差 deg      → 输出: target_rate_dps
    ↓
角速度环 (200 Hz) 输入: rate 误差 °/s      → 输出: PWM permille
    ↓
电机输出 (200 Hz) bsp_motor_set_output(L, R)

航向环   (50 Hz)  输入: yaw 误差 °         → 输出: 差分 PWM → 叠加到 L/R
```

**设计原则**：
- 越内环频率越高（从 20 Hz 到 200 Hz），对应物理响应带宽
- 每环独立 PID（`speed_pid` / `angle_pid` / `rate_pid` / `yaw_pid`），增益默认 0
- 串口独立设参：`rp`（角速率）/ `bp`（角度）/ `sp`（速度）/ `yp`（航向）

### 5.2 调度策略

单任务轮询 + SysTick 1 ms 节拍标志，不上 RTOS：

| 频率 | 任务 | 内容 |
|------|------|------|
| 1 kHz | IMU drain + motor update | `ms901m_feed_bytes` + `bsp_motor_update` |
| 200 Hz | 角速度内环 + 电机输出 | `balance_step_rate` → `bsp_motor_set_output` |
| 100 Hz | 电池采样 + 角度环 | `bsp_battery_update` + `balance_step_angle` |
| 50 Hz | 航向环 | `balance_step_yaw` |
| 20 Hz | 速度外环 | `balance_step_speed` |
| 5 Hz | LED 心跳 | `DL_GPIO_togglePins(BSP_LED_G_*)` |
| 1 Hz | 调试日志 | `[hb]` 心跳 + PID 诊断 |

### 5.3 app_balance.h API 重建

| API | 用途 |
|-----|------|
| `app_balance_set_balance_gains(kp,ki,kd)` | 角度环（100 Hz）—— 输入 tilt 误差 deg |
| `app_balance_set_rate_gains(kp,ki,kd)` | 角速度内环（200 Hz）—— 输入角速率误差 °/s |
| `app_balance_set_speed_gains(kp,ki,kd)` | 速度外环（20 Hz）—— 输入速度误差 cps |
| `app_balance_set_yaw_gains(kp,ki,kd)` | 航向环（50 Hz）—— 输入 yaw 误差 ° |
| `app_balance_set_yaw_kp(kp_yaw)` | 转向开环系数（K230 `target_yaw_pm * kp_yaw`） |
| `app_balance_run()` | 多速率调度主循环入口 |

### 5.4 main.c 默认装车入口

`main.c` 上电默认进入 `app_balance_run()`，串口 `t`/`test` 切入电机演示、`l`/`load` 返回。板载 S1/S2 不再承担业务角色。

---

## 6. Stage 3.5 ｜ 角速率 LPF + sigma-delta dither + 新电机适配

> 提交：`79b9348`（2026-05-16），变更 11 文件，+1107/-976。
>
> 本轮还隐含一次**电机硬件替换**：旧 GB370（11 PPR × 9.6:1）→ 新 GB370（500 PPR × 34:1），编码器分辨率提升约 170 倍，所有 `bsp_motor.h` 宏同步刷新。

### 6.1 右轮软件解码：X4→X2

#### 6.1.1 硬约束：仅一路硬件 QEI

MSPM0G3507 上 **只有 TIMG8 支持硬件 QEI**（该结论在阶段 0 已通过 SDK 源码 [QEIMSPM0.syscfg.js](file:///A:/Program%20Files/ti/mspm0_sdk_2_10_00_04/source/ti/driverlib/.meta/qei/QEIMSPM0.syscfg.js) 第 175 行 `TIMG(8|9|10|11)` 过滤器 + 器件寄存器核对确认：TIMG9/10/11 该器件不存在）。唯一的硬件 QEI 实例已在阶段 2.3 分配给左轮编码器（TIMG8 + PB15/PB16），右轮只能走 GPIO 中断的软件解码路径。

#### 6.1.2 旧电机时代：X4 安全可用

旧 GB370（11 PPR × 9.6:1 × X4 = 404 cnt/rev），右轮 ISR 边沿率计算：

```
peak_edge_rate = 线速度 / (π·轮径) × 减速比 × PPR × X4
               = 1.0 / (π·0.065) × 9.6 × 11 × 4
               ≈ 2068 edges/s ≈ 2.1 edges/ms
```

远低于雪崩阈值 200 edges/ms，CPU 占用 < 4%。X4 解码无任何问题。

#### 6.1.3 新电机触发 ISR 雪崩

换上 500 PPR × 34:1 新电机后，同样公式给出完全不同的数字：

```
peak_edge_rate = 1.0 / (π·0.065) × 34 × 500 × 4
               ≈ 332,800 edges/s ≈ 333 edges/ms  （180 RPM 出轴侧 ≈ 204 edges/ms）
```

**180 RPM 出轴侧即产生 204 edges/ms，已超过雪崩兜底阈值 200** → `GROUP1_IRQHandler` 判定为噪声雪崩 → `bsp_motor` 强制关闭 PA12/PA13 中断 50 ms → 右轮速度读数瞬间归零 → 平衡环因右侧没有速度反馈而崩溃。

这是该提交（`79b9348`）现场实测发现的关键 bug。

#### 6.1.4 方案：X2 解码

将右轮解码倍率从 X4 降为 X2：

| 项目 | X4（旧） | X2（新） | 备注 |
|------|---------|---------|------|
| 解码方式 | PA12 双沿 + PA13 双沿 | PA12 双沿 + PA13 ISR 内读电平判向 | PA13 仅判方向、不产生中断 |
| 分辨率 | 68,000 cnt/rev | 34,000 cnt/rev | 仍为左轮（68,000）的 1/2 |
| 180 RPM 边沿率 | 204 edges/ms | **102 edges/ms** | 雪崩阈值 300 有 3× 余量 |
| CPU 占用 @180 RPM | 51%（不可用） | **26%** | 32 MHz ULPCLK 下 |
| 最低可分辨速度（10 ms 窗口） | 0.09 rpm | 0.18 rpm | 对平衡控制仍远高于需求 |

源码变更（[bsp_motor.h](file:///c:/Users/wfl27/Desktop/1/MSP/LP_MSPM0G3507/template/hardware/bsp_motor.h)）：

```diff
-#define BSP_MOTOR_RIGHT_DECODE_X                   (4)
+#define BSP_MOTOR_RIGHT_DECODE_X                   (2)
```

同时：
- `BSP_MOTOR_RIGHT_COUNTS_PER_OUTPUT_REV` 更新为 `34000`（500 × 34 × 2）
- `BSP_MOTOR_ENC_IRQ_QUENCH_PER_MS` 雪崩阈值从 200→**300**（双重保险：即使未来 RPM 从 180 提升到 ~530 也不触发）
- 注释全部重写，详细说明 X4 不可用的根因与 X2 的安全性分析

#### 6.1.5 未来优化方向

> 当前 X2 方案对平衡控制完全够用（速度反馈精度 0.18 rpm @ 100 Hz），但长远看右轮 GPI O ISR 在高速工况下仍占 26% CPU。建议后续将右编码器迁移至硬件 QEI——MSPM0G3507 上虽无第二路 QEI 外设，但可用 TIMG6 或 TIMG7 的 Capture 模式模拟 X4 解码（硬件自动计数、零 ISR 开销），这也是阶段 0 预留的回退路径。

### 6.2 新电机其余参数适配

| 参数 | 旧值 (11 PPR × 9.6:1) | 新值 (500 PPR × 34:1) | 影响 |
|------|----------------------|----------------------|------|
| `GB370_HALL_PPR` | 11 | 500 | 编码器基础分辨率 |
| `GB370_GEAR_RATIO` | 9.6 | 34.0 | 减速比 |
| `LEFT_COUNTS_PER_OUTPUT_REV` | 404 | 68,000（X4） | 左轮 硬件 QEI X4 |
| `RIGHT_FORWARD_SCALE_X1000` | 980（95%） | **1000（禁用）** | 新电机左右绕组对称，不需补偿 |
| `SPEED_WINDOW_MS` | 20 | **10** | 速度刷新率 50→100 Hz |

### 6.3 角速度测量低通滤波器

电机 PWM 的 20 kHz 振动会通过车体结构耦合到 IMU，在 `pitch_rate_dps` 上产生高频噪声。直接送入角速度内环会导致 PWM 输出抖动、电机发热。

在 `balance_step_rate()` 中新增 `rate_lpf_dps` EMA 滤波器：

```c
/* 角速度测量低通滤波：alpha=0.5 EMA，抑制振动耦合噪声（tau≈10ms@200Hz） */
float raw_rate = att->pitch_rate_dps * (float)s_bal.pitch_sign;
s_bal.rate_lpf_dps += 0.5f * (raw_rate - s_bal.rate_lpf_dps);
float measured_rate = s_bal.rate_lpf_dps;
```

α 硬编码 **0.5**（非宏可配），200 Hz 下时间常数 τ ≈ 10 ms，对应带宽 ≈ 16 Hz——远高于角度环（~10 Hz），对平衡控制引入的相位滞后可忽略。

配套改动：
- `balance_state_t` 新增 `rate_lpf_dps` 字段
- `app_balance_init()` 初始化为 0.0f
- `app_balance_reset()` 同步清零

### 6.4 sigma-delta dither 死区处理

#### 6.4.1 问题

动摩擦补偿（`running_dz` ≈ 40‰）存在硬门槛：PID 输出 < 40‰ 时 BSP 直接输出 0 → 电机不响应；PID 输出恰好越过门槛时输出跳变到 ±40‰ → 方波振动。这在平衡车静止维持时尤为致命——PID 输出常在 0~30‰ 之间微幅波动，导致电机在"不动"和"突然动一下"之间反复。

#### 6.4.2 方案原理

用 **sigma-delta 脉冲密度调制**：对低于死区的命令，不直接映射输出，而是用一个累加器逐拍积分；累加到阈值时发射一次固定幅度的脉冲，并从累加器减去阈值（非脉冲幅值）。

核心两参数解耦：

| 参数 | 值 | 含义 |
|------|---|------|
| `BSP_MOTOR_DITHER_THRESHOLD_PM` | 20 | 累加器触发门槛——控制脉冲**频率** |
| `BSP_MOTOR_DITHER_PULSE_PM` | 200 | 脉冲输出幅度——控制单次力矩**大小** |

#### 6.4.3 实现细节

新增函数 [`apply_dither_deadzone()`](file:///c:/Users/wfl27/Desktop/1/MSP/LP_MSPM0G3507/template/hardware/bsp_motor.c)：

```c
static int16_t apply_dither_deadzone(int16_t permille, bool is_left, float *accum)
{
    int32_t dz = get_running_dz_pm(permille, is_left);
    if (dz <= 0) return permille;             // 无死区直通

    // 命令幅度 ≥ 死区：正常线性映射，累加器归零
    int32_t mag = abs(permille);
    if (mag >= dz) {
        *accum = 0.0f;
        return apply_deadzone_mapping(permille, dz);
    }

    // 子死区区域：累加命令
    *accum += (float)permille;

    if      (*accum >=  threshold) { *accum -= threshold; return  PULSE_PM; }
    else if (*accum <= -threshold) { *accum += threshold; return -PULSE_PM; }
    return 0;  // 未达门槛，本拍不输出
}
```

关键设计点：

1. **触发减 threshold 非 pulse**：`*accum -= threshold`（不是 `*accum = 0`），余量带入下一拍，保证长期平均输出 = 命令值。
2. **命令 ≥ 死区时累加器归零**：避免子死区残差污染正常驱动。
3. **左右通道独立累加器**：`left_dither_accum` / `right_dither_accum`（`float`），分别跟踪。

#### 6.4.4 与现有死区的协作逻辑

`commit_channel()` 中的决策树（按优先序）：

```
deadzone_comp_enabled?
├── false → 直通，跳过全部 DZ 逻辑
└── true
    ├── calibration_mode? → 强制 static DZ
    ├── 编码器未确认运动 (static)?
    │   └── static_dz_enabled? → apply_static_deadzone()
    ├── dither_dz_enabled? → apply_dither_deadzone()        ← 新增
    ├── running_dz_enabled? → apply_running_deadzone()
    └── 否则 → 直通
```

即 dither 优先级介于 static 和 running 之间。建议平衡车模式配置：`static=off, dither=on, running=off`。

#### 6.4.5 运行时 API

```c
void bsp_motor_set_dither_dz_enabled(bool enabled);
bool bsp_motor_get_dither_dz_enabled(void);
```

启用时自动清零左右累加器并重发当前命令；`bsp_motor_stop()` / `bsp_motor_brake()` 同步清零累加器。

#### 6.4.6 初始化行为

- `bsp_motor_init()`：`dither_dz_enabled = false`（默认关闭，向后兼容旧行为）
- `app_balance_run()`：显式调用 `bsp_motor_set_dither_dz_enabled(false)`（当前装车模式暂不启用 dither，留待整定后按需开启）

#### 6.4.7 等效增益与整定影响

脉冲平均输出公式：

```
avg_output = cmd × (PULSE / THRESHOLD)  （当 |cmd| < running_dz 时）
           = cmd                         （当 |cmd| ≥ running_dz 时，经线性映射）
```

以默认值 `PULSE=200, THRESHOLD=20` 为例：

| PID 输出 | dither 行为 | 等效 avg | 有效增益 |
|----------|------------|----------|---------|
| +10‰ | 每 2 拍一个 200‰ 脉冲 | 100‰ | 10× |
| +5‰ | 每 4 拍一个 200‰ 脉冲 | 50‰ | 10× |
| +2‰ | 每 10 拍一个 200‰ 脉冲 | 20‰ | 10× |
| +0.5‰ | 每 40 拍一个 200‰ 脉冲 | 5‰ | 10× |
| +40‰ | 正常线性映射 | 40‰ | 1× |

**整定注意**：子死区区域（0~40‰）增益放大 10×，意味着 `rp` 的 Kp 需相应缩小约 10 倍。当前以 `rp 1800`（原 ~18000 的 1/10）作为起点。

### 6.5 app_motor_demo 精简

`app_motor_demo` 删除大量不再使用的旧 API（`set_sync_*` / `get_sync_diag` / `cal_*` 扫描函数），仅保留 `run()` 循环的核心调速 / 显示功能，最大转速统一 300 rpm。

---

## 7. Stage 3.6 ｜ 速度反馈极性翻转 + 量纲归一化 + 文档清理

> 提交：`b8a5783`（2026-05-17），变更 13 文件，+136/-1433（净删 1297 行）。

### 7.1 速度反馈极性翻转

**问题**：小车前进时编码器 `avg_cps` 的符号取决于编码器物理安装方向，可能与平衡环"正 PWM = 前进"方向相反。此时速度环为正反馈，任何 Kp 都会立即发散。

**方案**：新增 `APP_BALANCE_SPEED_INVERT` 编译期宏 + 运行时串口命令 `si0`/`si1`：

```c
// balance_step_speed() 中
int32_t avg_cps_raw = ((fb.left_speed_cps + fb.right_speed_cps) / 2)
                      * (int32_t)s_bal.speed_sign;  // speed_sign = ±1
```

串口命令：
- `si?` — 查询当前速度极性状态 + 实时 `v_meas` 值
- `si0` — 设置正常极性（编码器正向 = 前进）
- `si1` — 设置翻转极性（编码器反向）

修改后自动 `pid_reset(&speed_pid)` + 清 `speed_lpf_cps` + 清 `target_tilt_deg`。

### 7.2 速度量纲归一化

**问题**：500 PPR × 34:1 编码器分辨率极高（左轮 68,000 cnt/rev、右轮 34,000 cnt/rev），典型漂移速度 0.1 rev/s 对应 `avg_cps ≈ 5100`。若直接用原始 cps 送入速度 PID，`sp 1`（Kp=0.001）即产生 5.1° 倾角指令，瞬间触发 ±10° 饱和极限环。

**方案**：新增 `APP_BALANCE_SPEED_CPS_SCALE = 10`，将 `avg_cps / 10` 后再进 PID：

| 场景 | raw cps | 归一化 cps | `sp 5` 产生倾角 | 效果 |
|------|---------|-----------|----------------|------|
| 0.1 rev/s 漂移 | ~5100 | 510 | 2.55° | 中等制动 |
| 0.02 rev/s 微漂 | ~1020 | 102 | 0.51° | 微小修正 |
| 静止 | 0 | 0 | 0° | 不修正 |

`target_speed_cps`（K230 运动指令）同步归一化，保持误差量纲一致。

### 7.3 速度反馈 LPF 收紧

`APP_BALANCE_SPEED_LPF_ALPHA` 从 0.5 收紧到 **0.15**（20 Hz 下 τ≈283 ms → 带宽 ~0.56 Hz，为内环 ~12 Hz 的 1/20），彻底解决速度环与平衡环耦合振荡。

### 7.4 文档与资源清理

删除过时文件（净删 1297 行）：
- `docs/chore/Ms901mStreamParser.{cpp,h}` —— C++ 版解析器（已被 `template/middle/ms901m.{c,h}` 替代）
- `docs/chore/temporary.md` —— 早期 AI 评审注（信息已沉淀到各 TaskLog）
- `tools/motor_calib/*.png` / `log.txt` / `result/*.png` —— 旧电机（9.6:1）的过期校准数据

---

## 7A. Stage 3.7 ｜ 两级级联重构（当前架构）

> 提交：`adc5344`（2026-05-21，`pid2` 分支）。对照 `docs/chore/STM32_demo` 与 DengFOC 例程，将俯仰控制从 Stage 3.4 四级级联**收敛为两级串级**，横摆仅保留航向角环。

### 7A.1 控制架构

```
速度外环 (20 Hz)  输入: v_meas 误差 (cps)     → 输出: target_tilt_deg
角度环   (100 Hz) 输入: tilt 误差 deg         → 输出: ave_pwm (permille)
航向角环 (20 Hz)  输入: yaw 误差 deg         → 输出: yaw_corr → left/right 差速叠加
```

- **移除**：独立角速度内环 `rate_pid` / 串口 `rp`；轮速差转向环 `turn_pid` / 串口 `tp`
- **保留**：速度反馈 LPF、极性翻转（`si0/si1`）、量纲缩放（`APP_BALANCE_SPEED_CPS_SCALE`）、安全状态机、`lt_stream`
- **D 项**：角度环 Kd 仍走陀螺 `pitch_rate_dps`（不经角速度 PID 环）

### 7A.2 `pid2_t` 控制器

新增 `template/middle/pid2_t`（`pid.h` / `pid.c`）：

| 特性 | 说明 |
|------|------|
| 积分 | 每拍 `integral += error * dt`（非误差×Ki 再乘 dt 的旧式） |
| 微分 | 微分先行：对测量值微分，减少 setpoint 跳变冲击 |
| OutOffset | 输出叠加恒定偏置，用于突破 TB6612 静摩擦（串口 `bo` / `bp` 第 4 参） |
| 抗饱和 | 输出限幅 + 积分 clamp |

平衡模式调用 `bsp_motor_set_deadzone_bypass(true)`，**关闭 BSP 死区重映射**；静摩擦由角度环 OutOffset 承担。

### 7A.3 航向角环

- 编译开关：`APP_BALANCE_YAW_ENABLED=1`（默认开）
- 数据源：`APP_BALANCE_YAW_SOURCE` — 0=EKF `yaw_deg`，1=陀螺 `gz` 积分（默认，抗磁干扰）
- 调度：与速度外环同拍 20 Hz（`balance_step_yaw`）
- `target_yaw_pm ≠ 0`（K230 运动指令主动转向）时**暂停航向闭环**，仅保留俯仰两级串级

### 7A.4 串口整定命令（现行）

| 命令 | 作用 |
|------|------|
| `bo <offset_pm>` | 全局角度环 OutOffset（先于 `bp` 粗调） |
| `bp <kp> <ki> <kd> <offset>` | 角度环增益 ×1000 定点 + offset |
| `sp <kp> <ki> <kd> <offset>` | 速度外环 |
| `yp <kp> <ki> <kd> <offset>` | 航向角环 |
| `yi0` / `yi1` | 航向积分清零 / 查询 |
| `si0` / `si1` | 速度反馈极性翻转 / 查询 |
| `pid?` | 回显三环增益与 offset |

**已移除**：`rp`（角速度内环）、`tp`（轮速差转向环）。

### 7A.5 K230 `PID_INJECT` 映射

| `pid_id` | 环 | MCU API |
|----------|-----|---------|
| 0 | 角度 | `app_balance_set_balance_gains` |
| 2 | 速度 | `app_balance_set_speed_gains` |
| 3 | 航向 | `app_balance_set_yaw_gains` |

`pid_id=1`（旧 rate 环）**不再支持**。

---

## 7B. Stage 3.8 ｜ 差速闭环 + EMA 目标/测量低通滤波

> 提交：`03dc3a3`（2026-05-24）→ `412800f`（2026-05-26）→ `f8dba1e`（2026-05-26，HEAD），三连提交，变更 3 文件，+120/-45。

### 7B.1 动机

Stage 3.7 中航向角环 `yp` 输出直接以差分 PWM permille 叠加到左右轮：

```
left_pwm  = balance_out + yaw_correction
right_pwm = balance_out - yaw_correction
```

这导致两个问题：
1. **航向环与平衡环量纲不匹配**：航向环输出是 PWM permille，而实际需要控制的是左右轮**速度差**，PWM 到速度差的映射受电池电压/地面摩擦/电机温度影响，时变严重
2. **K230 `target_omega` 直接写 `target_yaw_pm`** 没有经过速度闭环校准，原地旋转角速度不可控

### 7B.2 新架构：三级级联（航向角 → 差速 → PWM）

```
航向角环 (20 Hz)   输入: yaw 误差 deg         → 输出: target_dif（RPM/cps）
    ↓
差速环   (20 Hz)   输入: dif 误差              → 输出: diff_out_pm
    ↓
左右 PWM 叠加:  left  = balance_out + diff_out_pm
                right = balance_out - diff_out_pm
```

- **`app_balance_motion_cmd_t` 新增 `target_dif_cps`**：K230 `MOTION_CMD.target_omega` / 圆弧运动计算后填入此处
- 航向角环输出 `yaw_correction` 改为差速环 **目标**（归一化 cps → RPM）
- 差速环用 `pid2_t`（与角度/速度环统一），串口指令 `dp <kp> <ki> <kd> <offset>`

### 7B.3 关键可配宏

| 宏 | 默认值 | 含义 |
|---|--------|------|
| `APP_BALANCE_SPEED_TARGET_LPF_ALPHA` | 0.10 | 速度目标 EMA（20 Hz * 0.10 → τ ≈ 330 ms，抑制阶跃导致的"微倾启动→后仰回退→再启动"顿挫） |
| `APP_BALANCE_DIFF_TARGET_LPF_ALPHA` | 0.20 | 差速目标 EMA（20 Hz * 0.20 → τ ≈ 250 ms） |
| `APP_BALANCE_SPEED_LPF_ALPHA` | 0.25（原 0.15） | 速度测量 EMA，适度收紧以平衡噪声与延迟 |
| `APP_BALANCE_DIFF_LPF_ALPHA` | 0.30 | 差速测量 EMA |
| `APP_BALANCE_DIFF_MAX_PM` | 300 | 差速环输出绝对值上限（permille），防止紧弧运动饱和 |
| `APP_BALANCE_YAW_MAX_DIF_CPS` | 500 | 航向角环输出上限（归一化 cps），防止航向修正过冲 |
| `APP_BALANCE_DIFF_INVERT` | — | 差速方向校正编译期宏 |

### 7B.4 串口命令更新

| 命令 | 作用 | 新增/变更 |
|------|------|-----------|
| `dp <kp> <ki> <kd> <offset>` | 差速环增益 ×1000 + offset | **新增** |
| `yp ...` | 航向角环增益（输出改为差速环 target 而非直接 PWM） | 行为变更 |
| `yi0` / `yi1` | 航向 yaw/gz 极性翻转 | 保持 |
| `pid?` | 回显四环（bp/sp/yp/dp）增益 | 新增 dp |

### 7B.5 差速环 RPM 量纲化（`f8dba1e`）

`03dc3a3` 初版差速环使用归一化 cps（÷`SPEED_CPS_SCALE`）作为量纲。`f8dba1e` 切换为 **RPM 量纲**：

| 版本 | 差速量纲 | 航向环输出量纲 |
|------|---------|---------------|
| `03dc3a3` | 归一化 cps | 归一化 cps |
| `f8dba1e` | **RPM** | 归一化 cps（经 BSP `cps_to_rpm` 换算后送入差速环） |

RPM 量纲的优势：与 `bsp_motor_feedback_t.left/right_speed_rpm` 直接可比，整定直觉更好（"差速 20 RPM"比"差速 200 cps"更直观）。

### 7B.6 `app_balance_diag_t` 新增字段

```c
typedef struct {
    // ... 原有字段 ...
    int32_t diff_target_cps;    /* 差速环目标（归一化 cps） */
    int32_t diff_meas_cps;      /* 差速环实际（归一化 cps） */
    int16_t diff_out_pm;        /* 差速环输出（permille） */
    float   yaw_error_deg;      /* 航向角环误差（°） */
} app_balance_diag_t;
```

---

## 7C. Stage 3.9 ｜ 圆弧运动演示 + 车体参数集中管理

> 提交：`e3e7e50`（2026-05-24），变更 3 新文件 + 2 已有文件，+540/-3。

### 7C.1 功能概述

新增 `app_circle_demo` 子任务，支持平衡车在直立状态下执行**指定直径 + 速度的整圆轨迹运动**，一圈完成后自动停止。

### 7C.2 新文件

| 文件 | 行数 | 职责 |
|------|------|------|
| `template/app/app_circle_demo.h` | 111 | API 声明 + 编译期宏 + 诊断快照 |
| `template/app/app_circle_demo.c` | 218 | 核心状态机：启动→运行→三重判停 |
| `template/middle/robot_param.h` | 142 | 车体几何常数 + inline 运动学换算 |

### 7C.3 串口触发

| 命令 | 行为 |
|------|------|
| `c` / `circle` | 默认参数启动：直径 500 mm、速度 100 mm/s（倒退）、顺时针（俯视） |
| `circle <diam_mm> <v_mm_s>` | 自定义直径与速度（正=前进，负=倒退） |
| `cx` | 立即中止，运动指令归零 |

### 7C.4 三重判停逻辑

| 判据 | 条件 | 优先级 |
|------|------|--------|
| ① IMU 偏航积分 | `|gz_dps 累计| ≥ 360°` | **主判据** |
| ② 编码器弧长 | `arc_mm ≥ circumference × 1.2`（备防 gz 漂移） | 备份 |
| ③ 超时兜底 | `elapsed ≥ expected × 3`（最小 5 s） | 兜底 |

### 7C.5 调度寄生

`app_circle_demo_tick_20hz()` 在 `app_balance_run` 主循环的 20 Hz 分支中调用（速度环之后），激活时**覆盖** `cmd->target_speed_cps` 与 `cmd->target_dif_cps`。非激活时不做任何写入。

### 7C.6 `robot_param.h` —— 车体参数集中管理

**设计原则**：所有 mm 级物理量集中定义，消除散落各处的魔法数字；带 `#ifndef` 包裹支持工程级 `-D` 覆盖；带编译期 sanity check。

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `ROBOT_WHEEL_DIAMETER_MM` | 65 | 轮胎外径（含橡胶、实测） |
| `ROBOT_WHEEL_BASE_MM` | 185 | 左右轮接地中心间距（实测） |
| `ROBOT_WHEEL_CIRCUMFERENCE_MM_F` | π×D | 轮周长（浮点） |
| `ROBOT_AVG_COUNTS_PER_MM_X100` | — | 每 mm 行程对应编码器计数 ×100（整数，无浮点） |

**inline 运动学函数**：

| 函数 | 输入 | 输出 |
|------|------|------|
| `robot_v_mm_s_to_avg_cps(v)` | 线速度 mm/s | 左右轮平均 counts/s |
| `robot_arc_mm_from_avg_counts(cnt)` | 编码器累计差值 | 行驶弧长 mm |
| `robot_omega_mrad_to_delta_cps(ω)` | 角速度 mrad/s | 左右轮差速 counts/s |

圆弧运动计算链：`v_mm_s + diameter_mm` → `avg_cps` + `omega(mrad/s)` → `delta_cps` → 写入 `cmd.target_speed_cps / target_dif_cps` → 差速环闭环。

### 7C.7 默认编译期宏

```c
#define APP_CIRCLE_DEFAULT_DIAMETER_MM    (500)
#define APP_CIRCLE_DEFAULT_SPEED_MM_S     (-100)    // 负=倒退
#define APP_CIRCLE_DEFAULT_CLOCKWISE      (1)
#define APP_CIRCLE_ARC_OVERSHOOT_X100     (120)     // 弧长 1.2 倍判停
#define APP_CIRCLE_TIMEOUT_FACTOR_X100    (300)     // 超时 3 倍兜底
```

---

## 7D. Stage 3.10 ｜ 自校零总开关 + 栈修复 v2.0 + 差速环 RPM 量纲化

> 涉及提交：`f527a3d`（2026-05-25，自校零开关）、`76d7d30`（2026-05-24，栈修复）、`f8dba1e`（2026-05-26，RPM 量纲化，已在 §7B.5 详述）。

### 7D.1 上电自校零总开关（`f527a3d`）

Stage 3.7 中 `app_balance_run()` 入口强制阻赛 ~1.5 s 采样 pitch 均值做 `pitch_offset`。这在调试期频繁烧录时每次都要等，且如果上电姿态不标准会学错零点。

**新增**：

```c
#define APP_BALANCE_PITCH_AUTOZERO_ENABLE    (0)   // 默认关闭！
#define APP_BALANCE_PITCH_OFFSET_DEFAULT_DEG  (-2.5f) // 硬编码回退值
```

- **`AUTOZERO_ENABLE=1`**：行为与之前一致，上电阻赛采样，失败回退到 `OFFSET_DEFAULT_DEG`
- **`AUTOZERO_ENABLE=0`（默认）**：直接使用 `OFFSET_DEFAULT_DEG`，零等待启动

`OFFSET_DEFAULT_DEG = -2.5°` 根据实车重心位置预先测量填入，可消除大部分静态偏角。

### 7D.2 栈溢出修复 v2.0（`76d7d30`）

> **背景**：Stage 1.6 首次实测曾遇到 printf `%f` 栈溢出 → 栈从 256 B 扩到 1 KB（见 [Stage1.5-IMU-Swap-MS901M.md §12.2](Stage1.5-IMU-Swap-MS901M.md)）。本次是 **第二次栈溢出**，根因不同。

**现象**：在叠加了 K230 TEXT_CMD 处理 + 圆弧运动 + 差速环后的复杂调用链下，`app_safety` 的全局状态变量被随机改写 → 安全状态机异常跳变 → 电机突然断电或无法 arm。

**根因**：`app_safety` 的状态变量（`s_state` / `s_fall_debounce` 等）是文件作用域 `static` 变量，放在 `.bss` 段。栈从高地址向下生长，`.bss` 从低地址向上——当栈峰值超过预留空间时，会**静默覆盖** `.bss` 中紧邻的全局变量，且无任何硬件异常触发。

**修复**：

| 措施 | 内容 |
|------|------|
| 栈扩容 | `startup_mspm0g350x_uvision.s`：`Stack_Size EQU 0x00000800`（**2 KB**） |
| canary 检测 | `app_safety` 结构体首尾插入 `canary_start` / `canary_end` 魔数；`app_safety_get_state()` 每次校验，不一致则 `NVIC_SystemReset()` |
| 结构体包装 | `app_safety` 的全局变量从散落 `static` 收编为单一结构体，确保 canary 覆盖全部敏感字段 |
| 经验文档 | 新建 `docs/notes/LessonsLearned/StackOverflow-Printf-StateCorruption.md` |

### 7D.3 差速环 RPM 量纲化

详见 §7B.5，此处仅记录在修订历史中。

---

## 7E. Stage 3.11 ｜ 赛道模式 + 自立双 PID（2026-05-30）

> **目标**（Overview 发挥项）：上电自检通过后完成自立 → K230 循线 → 满圈暂停 5 s → 第二圈 → 稳定停车；MCU 自主判圈，双端流程对齐防冲突。

### 7E.1 架构：MCU 总指挥 + K230 阶段闸门

```
SELF_STAND → STAND_SETTLE → TRACE(lap1) → BRAKE → PAUSE(5s)
           → TRACE(lap2) → FINAL_BRAKE → DONE
```

| 角色 | 职责 |
|------|------|
| **MCU `app_track`** | 阶段状态机、自立/刹车速度包络、偏航+里程判圈、满圈时序 |
| **MCU `app_balance`** | 100 Hz 角度环 + 20 Hz 速度/差速/航向；`rise_override` 冻结外环 |
| **K230** | 视觉循线控制律照常算；**仅** MCU 上报 `track_phase==TRACE` 时下发 `(v,ω)` |

协议：`k230_vehicle_status_t` 由 7 B 扩至 **9 B**（追加 `track_phase:u8 + lap:u8`），K230 按 payload 长度向后兼容。详见 [Stage4-K230-Communication.md §7](Stage4-K230-Communication.md)、[Stage4-K230-Side.md §4.1](Stage4-K230-Side.md)。

### 7E.2 自立：专用 rise PID（非设定点斜坡）

**问题背景**：formula.md 实测运动 PID（`bp 70/2/0/20` 等）在 30~40° 起始倾角下若直接把 setpoint 设 0°，角度环积分在 <1 s 内 windup 到 `i_max` → PWM ±1000 饱和 → 电机疯转、encISR 雪崩、K230 EMI 掉线。首版"设定点线性斜坡"也无法实现真正的"猛起摆起"，只会撑着支架往前蹭。

**方案**：自立与运动 **两套角度环增益**，通过 `app_balance_set_rise_override()` 切换：

| 阶段 | 角度环 | 外环 | setpoint |
|------|--------|------|----------|
| SELF_STAND / STAND_SETTLE | **rise PID**（`APP_TRACK_RISE_*`） | 速度环在线，`target_speed=0` 防漂移 | 固定 **0°**（靠 PID 动力学摆起） |
| TRACE 及以后 | **motion PID**（`TRK_GAIN_*` = formula 实测） | 正常级联 | 速度环输出 tilt |

rise 增益设计要点（`app_track.h`）：

- `kp` 大 → 起摆冲量（猛起）
- **`ki = 0`** → 自立段持续大误差，必须关积分防 windup
- `kd` → 临近直立阻尼（"减速-稳定"），**首要整定对象**
- 全输出权限，不额外限幅

默认起点：`RISE_KP=50, RISE_KI=0, RISE_KD=8, RISE_OFS=20`；摆起窗口 `RISE_MS=1000`；稳定判据 `|pitch|<10°` 且 `|gz|<40°/s` 持续 `SETTLE_MS=400`。

### 7E.3 循线加速 / 减速刹车（速度包络）

在 `app_track_tick_20hz` 内对 **下发** `target_speed_cps` 做 `rate_limit`：

- **TRACE 进入/恢复**：`ACCEL_CPS_PER_TICK=400`（20 Hz 下约 0.6 s 爬满）
- **BRAKE / FINAL_BRAKE**：`DECEL_CPS_PER_TICK=800` 拉到 0；`|avg_cps|<STOP_CPS` 持续 `STOP_SETTLE_MS` 判停稳
- **不用** `app_safety` 的 `brake_pulse`（会掉平衡）

### 7E.4 判圈（MCU 自主）

复用 `app_circle_demo` 手法 + `robot_param.h`：

- 每拍 `yaw_accum += gz_dps × dt`
- `arc_mm = robot_arc_mm_from_avg_counts(Δavg_count)`
- 满圈：`|yaw|≥360°` **且** `arc≥LAP_LENGTH_MM×60%`（默认 `LAP_LENGTH_MM=2500`）；`LAP_TIMEOUT_MS=60s` 兜底

### 7E.5 冷上电时序：等 K230 再起立

**现象**：MCU 自检 ~5 s 进入 `ARMED` 远早于 K230 启动（~10 s）。过早 `app_track_start()` → 编码器浮空噪声 + IMU 未收敛 → 疯冲。

**修复**（`app_track.h` / `app_balance_run`）：

```c
APP_TRACK_AUTOSTART              = 1   /* 自检 ARMED 后自动启动 */
APP_TRACK_AUTOSTART_WAIT_K230    = 1   /* 默认等 K230 在线 */
APP_TRACK_AUTOSTART_K230_WAIT_MS = 15000u
```

逻辑：首次观测 `APP_SAFETY_ARMED` 起计时；`s_k230_online==true` **或** 超时后才 `app_track_start()`。等待期间 PID 增益仍为 0、电机不驱动。

### 7E.6 新增 / 改动文件

| 文件 | 变更 |
|------|------|
| `template/app/app_track.{c,h}` | **新建** — 状态机 + rise/motion 增益 + 包络 + 判圈 |
| `template/app/app_balance.{c,h}` | `set_rise_override()` / `get_pitch_meas()`；20 Hz 调 `app_track_tick_20hz`；`trk`/`tx` 命令；1 Hz `[hb] track=...` |
| `template/middle/k230_protocol.h` | `k230_vehicle_status_t` +2 字段 |
| K230 侧 | 见 [K230/docs/TaskLog/phase_G_track_mode.md](../../../K230/docs/TaskLog/phase_G_track_mode.md) |

### 7E.7 验收清单（赛道模式）

| 项 | 通过条件 | 状态 |
|---|---------|------|
| 冷上电不自立疯冲 | K230 未亮屏前心跳无 `[track] start`；或 `[track] autostart: K230 online` 后再起 | [ ] |
| 自立摆起 | 30~40° 支架姿态 → 1 s 内摆近直立，无持续 ±1000 PWM 饱和 | [ ] |
| K230 阶段对齐 | 非 TRACE 阶段 K230 OSD `CTRL:idle`；TRACE 时 `CTRL:TRACE` | [ ] |
| 满圈暂停 | lap1 满圈 → 停稳 → 直立 5 s → lap2 | [ ] |
| 第二圈结束 | `track_phase=DONE`，保持直立 | [ ] |
| 远程调试 | TEXT_CMD `trk` / `tx` 可启停 | [ ] |

---

## 8. 调试工具链

### 8.1 串口实时 CSV 数据流（lt_stream）

> 提交：`0ac8351`（2026-05-11），变更 5 文件，+206/-2。

在 `app_balance` 中新增高频率 CSV 数据流功能，用于 `serial_plot.py` 实时可视化：

**协议格式**：
```
lt,<t_ms>,<pitch_deg>,<left_target_rpm>,<right_target_rpm>,<left_actual_rpm>,<right_actual_rpm>,<left_pwm>,<right_pwm>\r\n
```

**API**：
- `set_lt_stream_enabled(bool)` — 串口 `lt`/`lt0` 命令开关
- `send_lt_sample(now_ms, cmd, fb)` — 200 Hz 每拍发送一行（约 80 B/行 × 200 Hz = 16 kB/s，XDS-UART 921600 baud 可承载）

**底层支持**：
- `bsp_log_uart.{c,h}`：新增 `bsp_log_uart_try_write_async()` 非阻塞 TX 原语，避免 CSV 流阻塞平衡环
- `bsp_motor.{c,h}`：新增 `bsp_motor_get_left/right_speed_rpm()` 快速取数

### 8.2 serial_plot.py —— PyQt6 串口实时绘图

> 提交：`9132238`（初始版，2026-05-11）+ `d29950d`（PyQt6 重写，2026-05-11）+ `bd9a9c6`（批处理脚本，2026-05-12）。

**功能**：
- 5 组同步折线图：左电机期望/实际转速、右电机期望/实际转速、俯仰角、左 PWM、右 PWM
- 鼠标/键盘交互：滚轮缩放、Space 暂停、R 重置 Y 轴、F 冻结 Y 轴
- 串口自动检测与配置（`--list` 列出可用串口、`--port COM3 --baud 921600`）

**技术栈**：PyQt6 + QThread 串口通信 + QTimer 定时刷新 + matplotlib QtAgg 后端。

**快捷脚本**：
- `run_serial_capture_com10.bat` —— 一键录制串口 CSV 日志
- `run_serial_plot_com10.bat` —— 一键启动实时绘图

### 8.3 PID 调参指南

[PIDTuningGuide.md](../notes/PIDTuningGuide.md) 已随 Stage 3.7 更新：**整定顺序**（OutOffset → 角度 → 速度 → 航向）、串口命令（`bo` / `bp` / `sp` / `yp`，×1000 定点整数 + offset）、安全性约束（MAX_PWM 限幅）。**无 `rp` 角速度内环**。

---

## 9. 文件改动全景

### 9.1 核心业务文件（贯穿全部提交）

| 文件 | 主要变更 |
|------|---------|
| `template/app/app_balance.h` | Stage 3.7：两级 API + `pid2_t` + 航向角环；编译期可配宏（周期、LPF、极性、量纲缩放） |
| `template/app/app_balance.c` | 状态机 + 三路 `pid2` + 多速率调度 + 串口命令 + lt_stream + K230 帧分发 + **Stage 3.11** `app_track` 集成 / `rise_override` |
| `template/app/app_track.{c,h}` | **Stage 3.11 新建** — 赛道主控状态机 + rise/motion 双增益 + 判圈 + 速度包络 |
| `template/middle/pid.{c,h}` | 保留 `pid_t`；新增 `pid2_t` / `pid2_update()` |
| `template/app/app_safety.{c,h}` | 保持 Stage 2 不变；200 Hz 角速度环内集成为 `app_safety_tick()` |
| `template/hardware/bsp_motor.h` | 双门槛死区（静/动摩擦 8 宏）+ sigma-delta dither + 右路正转补偿 + 运行时开关 API |
| `template/hardware/bsp_motor.c` | 双门槛状态机 + dither 累加发射 + 编码器停转检测 + X2 解码默认 |
| `template/hardware/bsp_log_uart.{c,h}` | 新增 `try_write_async` 非阻塞 TX |
| `template/main.c` | 默认装车入口 `app_balance_run()` + MS901M 200 Hz 配置工具 |

### 9.2 工具文件

| 文件 | 主要变更 |
|------|---------|
| `tools/serial_plot.py` | PyQt6 重写（451 行）+ 独立 PID 设参按钮 + 新 CSV 协议支持 |
| `tools/motor_calib/serial_capture.py` | 双门槛校准模式适配 |
| `run_serial_*_com10.bat` | 新增一键启动脚本 |

### 9.3 文档

| 文件 | 动作 |
|------|------|
| `docs/notes/PIDTuningGuide.md` | Stage 3.7 两级 + 航向角环更新 |
| `docs/chore/Ms901mStreamParser.{cpp,h}` | 删除（过时） |
| `docs/chore/temporary.md` | 删除（过时） |
| `tools/motor_calib/*.png` / `log.txt` / `result/*.png` | 删除（旧电机数据） |

---

## 10. 验收清单

> 在平衡参数整定过程中逐项打钩。**Stage 3.7 现行项**如下；§2~§6 历史中涉及 `rp`/角速度内环的条目已作废。

| 项 | 通过条件 | 验证方式 | 状态 |
|---|---------|---------|------|
| 工程能编译 | EIDE 构建无 error | EIDE 构建按钮 | [ ] |
| 上电 IMU 在线 | 3 s 内收到 0x01，心跳 `[hb]` 正常 | XDS-UART 日志 | [ ] |
| 安全状态机工作 | 倾斜 > 60° → 电机 brake + STBY 关 | 手推翻车观察 | [ ] |
| OutOffset 有效 | `bo 80` 后车轮能克服静摩擦微动 | XDS-UART | [ ] |
| 串口 PID 注入有效 | `bp 5000 0 5000 80` 后 `pid?` 回显一致 | XDS-UART | [ ] |
| 串口 lt_stream 正常 | `lt` → `serial_plot.py` 看到波形 | serial_plot.py | [ ] |
| 角度环闭合 | `bp` 注入后松手 → 车轮短暂修正回中 | 手扶微倾松手 | [ ] |
| 速度环不振荡 | `sp` 注入后静止时 PWM 噪声 < 5‰ RMS | serial_plot.py | [ ] |
| 航向环不漂移 | `yp` 注入后直行 10 s 内 yaw 偏差 < 2°（源 1 模式） | 日志 `yaw_err` | [ ] |
| 速度反馈极性正确 | `si?` 显示 `v_meas` 与前进方向同号 | XDS-UART | [ ] |
| 原地直立 ≥ 3 s | 前后偏移 ≤ 10 cm（Overview 基础项 1） | 卷尺 + 录像 | [ ] |

---

## 11. 已知问题与待办

| # | 问题 | 状态 | 计划 |
|---|------|------|------|
| 1 | 两级串级 PID 增益未完成系统整定，当前仍为手工试凑 | **整定中** | 按 PIDTuningGuide：`bo` → `bp` → `sp` → `yp` |
| 2 | 500 PPR × 34:1 新电机的 running_dz 暂用占位值 40‰ | **待标定** | 用 `app_motor_demo` 标定模式逐路测量后替换 |
| 3 | sigma-delta dither 在平衡模式已 bypass BSP 死区；OutOffset 由角度环承担 | **已适配** | 先用 `bo`/`bp` offset 突破静摩擦 |
| 4 | EKF 模式（源 0）的 ±180° 跳变在磁场干扰时可能触发短暂反向输出 | **已规避** | 默认使用源 1（陀螺积分），编译期可切 |
| 5 | XDS-UART 在 CSV 流 + 1 Hz 日志双开时偶有冲突 | **不阻塞** | CSV 流仅调试期用，装车后关闭 |
| 6 | K230 `MOTION_CMD.target_omega` → `target_yaw_pm`；非 0 时航向环暂停，主动转向协调层待实现 | **部分完成** | 见 Stage 4 TaskLog |
| 7 | 赛道模式自立 `RISE_KD` / 判圈 `LAP_LENGTH_MM` 待上车标定 | **整定中** | 见 §7E.2 / §7E.4 |
| 8 | 冷上电 K230 等待超时（15 s）后仍无 K230 时自立可完成、循线需 K230 | **已知** | `APP_TRACK_AUTOSTART_K230_WAIT_MS` |

---

## 12. 修订历史

| 版本 | 日期 | 变更 |
|------|------|------|
| v0.1 | 2026-05-12 | 初始版本：Stage 3.1 双门槛死区补偿 + 调试工具链 |
| v0.2 | 2026-05-13 | + Stage 3.2 Yaw 角度环 |
| v0.3 | 2026-05-13 | + Stage 3.3 D 项分离 + 速度反馈 LPF |
| v0.4 | 2026-05-14 | + Stage 3.4 四级级联重构（最大规模变更） |
| v0.5 | 2026-05-16 | + Stage 3.5 角速率 LPF + sigma-delta dither |
| v0.6 | 2026-05-17 | + Stage 3.6 速度极性翻转 + 量纲归一化 + 文档清理；首版完整 TaskLog |
| v0.7 | 2026-05-21 | + Stage 3.7 两级级联（`pid2`）+ 航向角环收敛；移除 `rp`/`tp`；更新验收清单与 §8.3 |
| v0.8 | 2026-05-26 | + Stage 3.8 差速闭环（`03dc3a3`/`412800f`/`f8dba1e`）——新增 `diff_pid` + EMA 目标/测量滤波 + RPM 量纲化 + 航向→差速三级级联；+ Stage 3.9 圆弧运动演示（`e3e7e50`）——`app_circle_demo` + `robot_param.h` + 三重判停；+ Stage 3.10 自校零总开关（`f527a3d`）+ 栈修复 v2.0（`76d7d30`）——2 KB + canary + 结构体包装；补写 §7B/§7C/§7D 完整章节 |
| v0.9 | 2026-05-30 | + Stage 3.11 赛道模式——`app_track` 状态机 + rise/motion 双 PID + 速度包络判圈 + K230 在线等待自启动；`VEHICLE_STATUS` 扩展 `track_phase/lap`；§7E 完整章节 |
