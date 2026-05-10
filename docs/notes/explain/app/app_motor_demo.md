# APP_MOTOR_DEMO 电机驱动演示模块 学习笔记

## 📖 一、整体概述

### 1.1 这个模块在项目中的位置

本项目是一个**两轮自平衡瞄准小车**，主控为 **TI MSPM0G3507**。

`app_motor_demo` 是阶段 2 的"电机验证程序"——在平衡控制还没写好的时候，先用这个程序让电机转起来，验证电机驱动和编码器反馈是否正常。

```
┌──────────────────────────────────────────────────────────────┐
│                    main.c                                     │
│  app_motor_demo_run() → 交互式电机测试                       │
│  如果收到 S2 装车命令 → 切入 app_balance_run()                │
└────────────────────┬─────────────────────────────────────────┘
                     │
┌────────────────────▼─────────────────────────────────────────┐
│                app_motor_demo.c（本模块）                      │
│                                                              │
│  核心功能：                                                   │
│    1. rpm → PWM 开环速度控制                                  │
│    2. 双轮同步 PI 控制（编码器反馈）                           │
│    3. 串口交互命令（'+'/'-'/'b'/'r'/'s'/'p'/'h'）           │
│    4. S1 按键：启动/刹车切换                                   │
│    5. 编码器日志：实时打印左右轮角度和转速                      │
└──────┬──────────────┬────────────────────────────────────────┘
       │              │
       ▼              ▼
  bsp_motor        bsp_log_uart
  (电机驱动)        (串口交互)
```

### 1.2 三个阶段的主循环对比

| 主循环 | 用途 | 状态 |
|--------|------|------|
| `app_telemetry_run()` | 验证 IMU、串口、LED | 阶段 1 |
| `app_motor_demo_run()` | **验证电机驱动 + 编码器反馈**（本模块） | 阶段 2 |
| `app_balance_run()` | 完整双环平衡控制 | 阶段 2.2+ |

### 1.3 两种调度方式对比

| 调度方式 | 本模块 | app_balance |
|---------|--------|-------------|
| 方法 | 基于绝对时间戳差 | 基于 tick_count 取模 |
| 频率调整 | 改宏即可，互不影响 | 需要整体计算 tick 算术 |
| 变量数 | 多个 last_xxx_ms | 一个 tick_count |
| 适合场景 | 各任务频率独立调整 | 统一节拍严格对齐 |

---

## 📋 二、宏/类型/函数汇总

### 2.1 编译期可配宏

| 宏 | 默认值 | 含义 |
|----|--------|------|
| `APP_MOTOR_DEMO_MAX_RPM` | 620 | 电机最大转速 |
| `APP_MOTOR_DEMO_DEFAULT_RPM` | 620 | 默认目标转速 |
| `APP_MOTOR_DEMO_BRAKE_MS` | 120 | 刹车脉冲时间 |
| `APP_MOTOR_DEMO_RPM_STEP` | 20 | '+/-' 调整步长 |
| `APP_MOTOR_SYNC_PERIOD_MS` | 50 | 同步环执行周期 |
| `APP_MOTOR_SYNC_KP_PM_PER_RPM` | 8 | Kp：每 rpm 误差修正量 |
| `APP_MOTOR_SYNC_KI_PM_PER_RPM_STEP` | 1 | Ki：每 rpm 误差累加量 |
| `APP_MOTOR_SYNC_MAX_CORRECTION_PM` | 350 | 修正量上限 |
| `APP_MOTOR_LOG_PERIOD_MS` | 100 | 日志打印周期 |
| `APP_MOTOR_HEARTBEAT_PERIOD_MS` | 250 | LED 心跳周期 |

### 2.2 函数汇总

| 函数名 | 功能 |
|--------|------|
| `app_motor_demo_run()` | ⭐ 主循环入口 |
| `app_motor_demo_set_speed_rpm(rpm)` | 设置目标转速 |
| `motor_sync_step(feedback, running)` | ⭐ 同步环 PI 计算 |
| `process_log_uart_commands(running)` | 串口命令行处理 |

---

## 🔄 三、核心逻辑总结

### 3.1 主循环调度

```
app_motor_demo_run():
  for (;;) {
      if (!consume_tick()) { __WFI(); continue; }

      bsp_motor_update()          ← 1 kHz
      process_uart_commands()     ← 1 kHz（键盘输入）
      handle_start_button()       ← 1 kHz（S1 切换）
      handle_sync_tick()          ← 20 Hz（同步 PI）
      handle_load_button()        ← 1 kHz（S2 装车，当前未启用）
      handle_log_tick()           ← 10 Hz（编码器日志）
      handle_led_tick()           ← 4 Hz（LED 心跳）
  }
```

### 3.2 rpm → PWM 换算

```
PWM = rpm × 1000 / 620（线性映射）

620 rpm → 1000（满速）
310 rpm → 500（半速）
0 rpm → 0（停止）
```

### 3.3 同步 PI 控制原理

```
误差 = 右轮 rpm - 左轮 rpm
积分 += 误差 × Ki
修正量 = 误差 × Kp + 积分（限幅 ±350）

左轮 = 目标 PWM + 修正量
右轮 = 目标 PWM - 修正量
```

### 3.4 串口命令

| 命令 | 操作 |
|------|------|
| `+` | 转速 + 20 rpm |
| `-` | 转速 - 20 rpm |
| `350` + 回车 | 直接设 350 rpm |
| `b` | 刹车 |
| `r` | 启动 |
| `s` | 开关同步控制 |
| `p` | 打印同步诊断 |
| `h` / `?` | 打印帮助 |

---

## 🧩 四、关键代码段

### 4.1 数字累积输入

```c
// 输入 "3" "5" "0" "\n"：
'3' → rpm_acc = 0×10 + 3 = 3
'5' → rpm_acc = 3×10 + 5 = 35
'0' → rpm_acc = 35×10 + 0 = 350
'\n' → 提交：设目标 = 350 rpm
```

### 4.2 同步环 PI 核心算法

```c
int16_t error = right_rpm - left_rpm;     // 右快→正
int32_t i_term = s_sync_i_pm + error * ki; // 积分累加
s_sync_i_pm = clamp(i_term);
int32_t correction = error * kp + s_sync_i_pm; // P+I
apply_motor_output(clamp(correction));    // 左+修正 右-修正
```

---

## 🐛 五、常见踩坑

| 现象 | 可能原因 | 解决方法 |
|------|---------|---------|
| 电机不转 | STBY 未拉高 | 确认 `bsp_motor_enable(true)` |
| 左右轮速度差很大 | 同步控制未开启 | 串口输入 `s` 切换开关 |
| 串口命令无响应 | XDS-UART 波特率不匹配 | 检查 115200 8N1 |
| S2 无效 | PA16 被 AIN2 占用 | 等飞线后启用 `APP_MOTOR_DEMO_ENABLE_LOAD_BUTTON` |
| 编码器读数为 0 | 编码器连线松动 | 检查 PB15/PB16 和 PA12/PA13 连接 |

---

> 本文档配合 `app_motor_demo.h` 和 `app_motor_demo.c` 中的详细注释阅读效果最佳。加油！
