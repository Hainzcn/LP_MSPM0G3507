# APP_BALANCE 双环平衡控制模块 学习笔记

## 📖 一、整体概述

### 1.1 这个模块在项目中的位置

本项目是一个**两轮自平衡瞄准小车**，主控为 **TI MSPM0G3507**。

`app_balance` 是应用层最核心的模块——它让小车"站起来并走"。这是整个项目中优先级最高的"大脑"：接收姿态传感器数据，计算电机命令，让小车保持平衡。

```
┌──────────────────────────────────────────────────────────────┐
│                    main.c                                     │
│  app_balance_init() → app_balance_set_*_gains() →            │
│  app_balance_run() （永不返回）                                │
└────────────────────┬─────────────────────────────────────────┘
                     │
┌────────────────────▼─────────────────────────────────────────┐
│                  app_balance.c/h（本模块）                    │
│                                                              │
│  app_balance_step()：级联 PID（速度外环 + 平衡内环）         │
│  app_balance_run()：主循环调度（1 kHz/100 Hz/5 Hz/1 Hz）     │
└──────┬──────────────┬──────────────┬─────────────────────────┘
       │              │              │
       ▼              ▼              ▼
  ms901m pitch    bsp_motor      app_safety
  (测量值)         (执行输出)      (安全监督)
```

### 1.2 级联 PID 控制原理

级联 PID 包含两个环路：

```
                    速度外环                    平衡内环
  target_speed ──► speed_pid ──► target_tilt ──► balance_pid ──► PWM → 电机
       ▲               │               ▲               │
       │               │               │               │
       └── avg_cps ────┘               └── pitch_meas ──┘
       (编码器速度反馈)                   (MS901M 俯仰角反馈)
```

**外环（速度环）**：控制目标速度 → 输出目标倾角
- 你想让车以多快往前走？如果慢了→倾更多→加速；快了→倾少点→减速

**内环（平衡环）**：控制目标倾角 → 输出 PWM
- 你想让车倾多少度？如果倾多了→多出力纠正；倾少了→少出力

**理解级联**：就像开车——你先决定"我要开多快"（外环），然后根据速度决定"油门踩多深"（内环）。

---

## 📋 二、宏/类型/函数汇总

### 2.1 编译期可配宏

| 宏 | 默认值 | 含义 |
|----|--------|------|
| `APP_BALANCE_CONTROL_PERIOD_MS` | 10 | 控制周期 ms（= 100 Hz） |
| `APP_BALANCE_MAX_TILT_DEG` | 10.0f | 速度外环输出的倾角上限（°） |
| `APP_BALANCE_MAX_PWM_PERMILLE` | 1000 | 平衡内环输出的 PWM 上限 |
| `APP_BALANCE_SPEED_D_FILTER_ALPHA` | 0.20f | 速度外环 D 项滤波（噪声大） |
| `APP_BALANCE_BALANCE_D_FILTER_ALPHA` | 0.10f | 平衡内环 D 项滤波（噪声小） |

### 2.2 输入/输出结构体

| 结构体 | 用途 | 关键字段 |
|--------|------|---------|
| `app_balance_attitude_t` | 当前姿态 | pitch_deg, pitch_rate_dps, valid |
| `app_balance_motion_cmd_t` | 运动指令 | target_speed_cps, target_yaw_pm |
| `app_balance_diag_t` | 诊断信息 | target_tilt_deg, pitch_meas_deg, balance_out_pwm, left/right_cmd_pm, speed_meas_cps, driving |

### 2.3 函数汇总

| 函数名 | 功能 | 调用频率 |
|--------|------|---------|
| `app_balance_init()` | 初始化两个 PID（增益=0） | 一次 |
| `app_balance_reset()` | 清零积分和微分历史 | 切换模式时 |
| `app_balance_set_pitch_offset(deg)` | 设定零点偏移 | 校准后 |
| `app_balance_set_balance_gains(Kp,Ki,Kd)` | 设内环增益 | 调参 |
| `app_balance_set_speed_gains(Kp,Ki,Kd)` | 设外环增益 | 调参 |
| `app_balance_set_yaw_kp(Kp)` | 设转向系数 | 调参 |
| `app_balance_step(att, cmd)` ⭐ | 执行一拍双环计算 | **100 Hz** |
| `app_balance_get_diag(out)` | 获取诊断快照 | 1 Hz |
| `app_balance_run()` | 主循环入口（永不返回） | main 调用一次 |

---

## 🔄 三、核心逻辑总结

### 3.1 app_balance_step() 完整数据流

```
输入：att(pitch_deg, pitch_rate_dps, valid) + cmd(target_speed, target_yaw)

步骤 1：安全检查
  app_safety_tick(&att)
  if (!can_drive()) → 复位 PID + return

步骤 2：速度外环 PID（100 Hz）
  反馈 = (left_speed + right_speed) / 2（cps）
  误差 = target_speed_cps - 反馈
  输出 = target_tilt_deg（限幅 ±10°）

步骤 3：平衡内环 PID（100 Hz）
  测量值 = pitch_deg - pitch_offset_deg（已减零点偏移）
  误差 = target_tilt_deg - 测量值
  输出 = pwm_out（限幅 ±1000）

步骤 4：转向叠加 + 限幅
  yaw = target_yaw_pm × yaw_kp
  left  = clamp(pwm_out - yaw)
  right = clamp(pwm_out + yaw)

步骤 5：输出到电机
  bsp_motor_set_output(left, right)

输出：电机 PWM 命令（通过 bsp_motor 执行）
```

### 3.2 主循环调度策略

```
基于 1 ms SysTick，tick_count = 0, 1, 2, 3, ...

1 kHz（每 tick）：IMU drain + bsp_motor_update()
100 Hz（%10==0）：电池采样 + snapshot + balance_step()
5 Hz（%200==0）：LED_G 翻转 + LED_R 状态指示
1 Hz（%1000==0）：printf 调试日志
```

### 3.3 关键设计：增益默认为 0

所有 PID 增益初始化为 0——这是"fail-safe"设计。
如果不设增益，电机输出永远为 0，不会乱跑。
调试时必须手动通过 `app_balance_set_*_gains()` 注入合适的增益值。

### 3.4 关键设计：不允许驱动时复位 PID

```c
if (!app_safety_can_drive() || !att->attitude_valid) {
    app_balance_reset();  // 清零积分 + 微分历史
    return;               // 不调 bsp_motor_set_output
}
```

不清零积分的话：安全模式下积分还在累积，等下一次 ARMED 时积分已饱和→超调。

### 3.5 关键设计：定点格式化避免 printf("%f")

```
printf("%f", 2.35) → 拉入几 KB 浮点格式化代码（Cortex-M0+ 软件浮点很慢）

定点格式化方案：
  2.35° → " 2.35"
  -2.35° → "-2.35"

宏拆分：符号 + 整数 + "." + 小数
  完全用整数 printf 实现，代码量极小
```

---

## 🧩 四、关键代码段详解

### 4.1 级联 PID 的核心三行

```c
float target_tilt_deg = pid_step(&s_bal.speed_pid,
    (float)cmd->target_speed_cps, (float)avg_cps, s_dt_sec);

float pitch_meas = att->pitch_deg - s_bal.pitch_offset_deg;
float pwm_out = pid_step(&s_bal.balance_pid,
    target_tilt_deg, pitch_meas, s_dt_sec);
```

这就是级联的本质：第一个 PID 的输出（target_tilt_deg）直接成为第二个 PID 的目标值。

### 4.2 转向叠加

```c
float yaw_pm = (float)cmd->target_yaw_pm * s_bal.yaw_kp;
int16_t left_pm  = clamp_pwm_pm(pwm_out - yaw_pm);
int16_t right_pm = clamp_pwm_pm(pwm_out + yaw_pm);
```

转向是"差速"实现的：左转 → 左轮减、右轮加；右转→ 左轮加、右轮减。

### 4.3 主循环的 __WFI 睡眠

```c
if (!bsp_systick_consume_tick()) {
    __WFI();      // CPU 暂停，直到下一个中断唤醒
    continue;
}
```

如果当前 tick 还没到，CPU 睡眠等待——节省能源，而不是空转。

---

## 🐛 五、常见踩坑

| 现象 | 可能原因 | 解决方法 |
|------|---------|---------|
| 上电后电机不动 | 增益全是 0 | `app_balance_set_balance_gains(Kp, 0, Kd)` |
| 车往一个方向猛冲 | pitch 极性反了 | Kp 取反或 pitch_deg 取反 |
| 车来会震荡 | Kp 太大或 Kd 太小 | 加 Kd，减 Kp |
| 车速不稳定 | 速度外环没调或增益太小 | 调 speed Kp（典型 0.001~0.01） |
| 绿灯不闪 | SysTick 或主循环卡住了 | 检查编码器 ISR 是否雪崩 |
| 日志中 ISR_QUENCH 出现 | 编码器引脚浮空 | 检查编码器连接和上拉配置 |

---

> 本文档配合 `app_balance.h` 和 `app_balance.c` 中的详细注释阅读效果最佳。加油！
