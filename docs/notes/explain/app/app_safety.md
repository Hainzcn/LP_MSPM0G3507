# APP_SAFETY 安全状态机模块 学习笔记

## 📖 一、整体概述

### 1.1 这个模块在项目中的位置

本项目是一个**两轮自平衡瞄准小车**，主控为 **TI MSPM0G3507**。

`app_safety` 是应用层中的"监护人"——它在每个控制周期都被调用，检查小车是否处于安全状态，并在危险发生时自动介入（刹车、限速、关闭电机）。

```
┌──────────────────────────────────────────────────────────────┐
│                    app_balance.c                              │
│  每拍调用 app_safety_tick() 检查是否安全                      │
│  通过 app_safety_can_drive() 决定是否输出 PID 命令             │
└────────────────────▲─────────────────────────────────────────┘
                     │
┌────────────────────┴─────────────────────────────────────────┐
│                  app_safety.c/h（本模块）                      │
│                                                              │
│  安全状态机：跌倒检测 + 电池保护 + S1 按键                    │
│  被动监督——每拍检查一次，危险时强制介入                        │
└──────┬──────────────┬──────────────────┬─────────────────────┘
       │              │                  │
       ▼              ▼                  ▼
  ms901m pitch    bsp_battery       bsp_motor
  (跌倒检测)       (电压监测)         (刹车/限幅/STBY)
```

### 1.2 状态机设计

```
                    S1 启动                    |pitch| > 60°
   DISARMED ────────────────► ARMED ────────────────────► FALLEN
       ▲                         │  │                         │
       │                         │  │ 电池 LOW_WARN           │ S1
       │ disarm()                │  ▼                         │
       │                         │ LOW_BAT_WARN ────► ARMED  (若电池恢复)
       │                         │  │
       │                         │  │ 电池 LOW_STOP
       │                         ▼  ▼
       └──────────── LOW_BAT_STOP ◄── 任何状态，电池 LOW_STOP
                              │
                              │ S1：被拒绝（电压太低）
                              ▼
                         (stay LOW_BAT_STOP)
```

**5 个状态的含义**：

| 状态 | 值 | 含义 | 电机状态 | can_drive? |
|------|-----|------|---------|-----------|
| `DISARMED` | 0 | 上电默认/手动停车 | STBY=低，刹车 | ❌ |
| `ARMED` | 1 | 正常运行 | STBY=高，满 PWM | ✅ |
| `LOW_BAT_WARN` | 2 | 低压告警 | STBY=高，限幅 60% | ✅ |
| `FALLEN` | 3 | 跌倒 | STBY=低，刹车 | ❌ |
| `LOW_BAT_STOP` | 4 | 低电压急停 | STBY=低，刹车 | ❌ |

**状态优先级**（高优先抢占低优先）：`LOW_BAT_STOP > FALLEN > LOW_BAT_WARN > ARMED`

---

## 📋 二、宏/类型/函数汇总

### 2.1 编译期可配宏

| 宏 | 默认值 | 含义 |
|----|--------|------|
| `APP_SAFETY_FALL_PITCH_DEG` | 60.0f | 跌倒角度阈值 |
| `APP_SAFETY_FALL_BRAKE_MS` | 80 | 跌倒刹车脉冲时间 |
| `APP_SAFETY_LOW_BAT_BRAKE_MS` | 120 | 低压刹车脉冲时间 |
| `APP_SAFETY_LOW_BAT_PWM_LIMIT` | 600 | 低压告警 PWM 限幅值 |

### 2.2 函数汇总

| 函数名 | 功能 | 调用频率 |
|--------|------|---------|
| `app_safety_init()` | 初始化为 DISARMED + 安全硬件操作 | 一次 |
| `app_safety_arm()` | 尝试进入 ARMED | 用户触发 |
| `app_safety_disarm()` | 主动进入 DISARMED | 人工介入 |
| `app_safety_tick()` ⭐ | 安全检查 + 状态决策 | **每拍** |
| `app_safety_get_state()` | 查询当前状态 | 任意 |
| `app_safety_can_drive()` | 是否允许输出电机命令 | 每拍 |

---

## 🔄 三、核心逻辑总结

### 3.1 tick() 的 4 步处理流程

```
app_safety_tick(&att) 每拍调用：
  │
  ├─ 步骤 1：处理 S1 按键
  │   DISARMED/FALLEN/LOW_BAT_WARN → ARMED（启动）
  │   ARMED → DISARMED（手动停车）
  │   LOW_BAT_STOP → 拒绝
  │
  ├─ 步骤 2：跌倒检测
  │   |pitch| > 60° → fallen = true
  │   attitude 为 NULL 或无效 → 跳过
  │
  ├─ 步骤 3：读取电池状态
  │   bs = bsp_battery_get_state()
  │
  └─ 步骤 4：按优先级合成状态
      LOW_STOP → fallback to disarm → fallen → LOW_WARN → NORMAL
```

### 3.2 关键设计：transition() 的"相同不操作"

```c
if (next == s_state) return;  // 状态没变，什么都不做
```

这是状态机设计中一个重要的原则：**只在状态真正变化时才执行过渡动作**。

如果每拍都执行 `hw_emergency()`，`brake_pulse_ms` 的计时器会被反复重置——刹车永远到不了期，电机永远停不了。所以只在状态第一次进入时执行硬件操作。

### 3.3 关键设计：LOW_BAT_STOP 不自动恢复

电池电压回升后，状态从 `LOW_BAT_STOP` → `LOW_BAT_WARN`（不是 → `ARMED`）。

这需要用户**手动按 S1**才能重新启动。目的是防止"电池电压在阈值附近抖动 → 车体反复急停/启动"的危险情况。

### 3.4 关键设计：DISARMED 下的电池告警不调 transition()

```c
if (bs == BSP_BATT_STATE_LOW_WARN) {
    s_state = APP_SAFETY_LOW_BAT_WARN;  // 直接改变量，不走 transition()
}
```

不走 `transition()` 的原因是：`transition(LOW_BAT_WARN)` 会调用 `hw_arm_low_warn()`，其中包含 `bsp_motor_enable(true)`——会把电机打开。但 DISARMED 状态下不应该开电机。所以只更新状态变量，不做硬件操作。

---

## 🧩 四、关键代码段详解

### 4.1 transition() —— 状态切换中枢

```c
static void transition(app_safety_state_t next)
{
    if (next == s_state) return;   // 相同不操作
    s_state = next;
    switch (next) {
        case APP_SAFETY_DISARMED:
        case APP_SAFETY_FALLEN:     hw_emergency(FALL_BRAKE_MS); break;
        case APP_SAFETY_LOW_BAT_STOP: hw_emergency(LOW_BAT_BRAKE_MS); break;
        case APP_SAFETY_ARMED:       hw_arm_normal(); break;
        case APP_SAFETY_LOW_BAT_WARN: hw_arm_low_warn(); break;
    }
}
```

三个硬件操作封装：
- `hw_emergency(ms)`：刹车 pulse + STBY=低
- `hw_arm_normal()`：PWM 满 + STBY=高
- `hw_arm_low_warn()`：PWM 限幅 + STBY=高

### 4.2 tick() 中的 if-else if 链

优先级从高到低排列的 if-else if 链：

```c
if (bs == LOW_STOP)           → LOW_BAT_STOP（最高优先级）
else if (s_state == LOW_STOP) → LOW_BAT_WARN（电池恢复但不自动 ARM）
else if (s_state == DISARMED) → 只更新电池状态，不动硬件
else if (fallen)              → FALLEN
else if (bs == LOW_WARN)      → LOW_BAT_WARN
else if (bs == NORMAL)        → ARMED（如果之前是 LOW_BAT_WARN）
```

### 4.3 S1 按键的"一键两用"

```c
if (ARMED)        → DISARMED（正在跑→停车）
if (DISARMED/...) → ARMED（停着→启动）
```

同一个按键在不同状态下有不同的含义——这是嵌入式设备中常见的"一键多义"设计。

---

## 🐛 五、常见踩坑

| 现象 | 可能原因 | 解决方法 |
|------|---------|---------|
| 上电后电机不转 | 状态是 DISARMED，需要按 S1 | 按 S1 或调用 `app_safety_arm()` |
| S1 按键无效 | 状态是 LOW_BAT_STOP | 充电或更换电池 |
| 跌倒后能重启但马上又停 | pitch 角仍然 > 60° | 把车摆正再重启 |
| 一启动就降功率 | 电池电压低 | 检查电池，充到 > 9.7V |
| 刹车脉冲到期又刹车 | 硬件操作被反复触发 | 确认 `transition()` 有"相同不操作" |

---

> 本文档配合 `app_safety.h` 和 `app_safety.c` 中的详细注释阅读效果最佳。加油！
