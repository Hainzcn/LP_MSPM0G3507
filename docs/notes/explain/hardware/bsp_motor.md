# BSP_MOTOR 电机驱动模块 学习笔记

## 📖 一、整体概述

### 1.1 这个模块在项目中的位置

本项目是一个**两轮自平衡瞄准小车**，主控为 **TI MSPM0G3507**。

`bsp_motor` 是硬件驱动层中最核心、最复杂的模块——它让电机转起来、知道轮子转了多少、检测启动按键。

```
┌──────────────────────────────────────────────────────────────┐
│                    app_balance.c                              │
│  PID 计算出左右轮目标速度                                     │
│  bsp_motor_set_output(left_pm, right_pm)                      │
└────────────────────┬─────────────────────────────────────────┘
                     ▼
┌──────────────────────────────────────────────────────────────┐
│                    bsp_motor.c/h（本模块）                    │
│                                                              │
│  TB6612 驱动：方向引脚 + PWM 占空比                           │
│  左轮编码器：TIMG8 硬件 QEI（16→32 位扩展）                    │
│  右轮编码器：PA12/PA13 GPIO 中断软件解码                      │
│  S1 按键：中断 + 轮询双保险                                   │
│  中断雪崩保护：防止浮空引脚导致 CPU 锁死                       │
└────────────────────┬─────────────────────────────────────────┘
                     ▼
┌──────────────────────────────────────────────────────────────┐
│                    硬件层                                      │
│  TB6612 → 左右 GB370 直流减速电机                              │
│  左轮 QEI → PB15/PB16                                       │
│  右轮编码器 → PA12/PA13                                     │
│  S1 按键 → PA18                                              │
└──────────────────────────────────────────────────────────────┘
```

### 1.2 硬件方案

| 组件 | 型号/规格 | 说明 |
|------|----------|------|
| 电机 | GB370 直流减速电机 | 30:1 减速比，11 PPR 霍尔编码器 |
| 驱动 | TB6612FNG | 双路 H 桥，独立控制左右电机 |
| 左轮测速 | TIMG8 硬件 QEI | X4 解码，16 位计数器→软件扩 32 位 |
| 右轮测速 | PA12/PA13 GPIO 中断 | 软件正交解码，X2 或 X4 可选 |
| PWM | TIMA0 CCP0/CCP1 | 20 kHz，超出人耳范围 |
| 按键 | LaunchPad S1 (PA18) | 双沿中断 + 80 ms 软件去抖 |

### 1.3 文件结构

| 文件 | 行数 | 作用 |
|------|------|------|
| `bsp_motor.h` | 398 | 宏定义、反馈结构体、25 个公开函数声明 |
| `bsp_motor.c` | 991 | 全部实现：状态管理、TB6612 控制、编码器解码、中断处理 |

### 1.4 核心功能模块

| 模块 | 对应函数 | 作用 |
|------|---------|------|
| **初始化** | `bsp_motor_init()` | 配置引脚、清零状态、设安全态、注册中断 |
| **使能** | `bsp_motor_enable()` | 拉高/拉低 STBY |
| **速度命令** | `set_output()/set_left()/set_right()` | 设方向 + PWM |
| **停止** | `stop()/brake()/brake_pulse_ms()` | 滑行/刹车/脉冲刹车 |
| **极性/限幅** | `set_invert()/set_pwm_limit()` | 运行时配置 |
| **周期更新** | `bsp_motor_update()` ⭐ | 1 kHz 心跳：计数扩展 + 速度窗口 + 雪崩检测 |
| **反馈获取** | `bsp_motor_get_feedback()` | 关中断读所有状态 + 浮点换算 |
| **按键** | `consume_toggle_request()` | 中断+轮询双保险检测 |
| **ISR** | `GROUP1_IRQHandler()` | GPIO 中断入口 |

---

## 📋 二、宏定义/类型/函数汇总

### 2.1 编码器计数推导

```
输出轴转一圈的编码器计数 = 减速比 × 霍尔 PPR × 解码倍率

左轮（硬件 QEI，X4）：
  30 × 11 × 4 = 1320 counts/rev

右轮（GPIO 中断，默认 X4）：
  30 × 11 × 4 = 1320 counts/rev

右轮（GPIO 中断，X2 模式）：
  30 × 11 × 2 = 660 counts/rev
```

### 2.2 bsp_motor_feedback_t 结构体

| 字段 | 类型 | 单位 | 含义 |
|------|------|------|------|
| `left_count` | int32 | 计数 | 左轮累计编码器计数 |
| `right_count` | int32 | 计数 | 右轮累计编码器计数 |
| `left_angle_deg` | float | ° | 左轮输出轴累计角度 |
| `right_angle_deg` | float | ° | 右轮输出轴累计角度 |
| `left_speed_cps` | int32 | counts/s | 左轮速度（原始单位） |
| `right_speed_cps` | int32 | counts/s | 右轮速度 |
| `left_speed_dps` | float | °/s | 左轮角速度 |
| `right_speed_dps` | float | °/s | 右轮角速度 |
| `left_speed_rpm` | float | rpm | 左轮转速 |
| `right_speed_rpm` | float | rpm | 右轮转速 |

换算关系：
- `角度(°) = 计数 × (360 / 每圈计数)`
- `转速(rpm) = 角速度(°/s) / 6`

### 2.3 函数汇总

| 函数名 | 参数 | 返回值 | 功能 |
|--------|------|--------|------|
| `bsp_motor_init()` | 无 | 无 | 初始化电机驱动 |
| `bsp_motor_enable(enable)` | bool | 无 | 使能/待机(STBY) |
| `bsp_motor_is_enabled()` | 无 | bool | 查询是否使能 |
| `bsp_motor_set_output(l, r)` | int16, int16 | 无 | 同时设左右轮速度 |
| `bsp_motor_set_left(pm)` | int16 | 无 | 仅设左轮 |
| `bsp_motor_stop()` | 无 | 无 | 滑行停止 |
| `bsp_motor_brake()` | 无 | 无 | 持续刹车 |
| `bsp_motor_brake_pulse_ms(ms)` | uint32 | 无 | 脉冲刹车 N ms |
| `bsp_motor_set_invert(l, r)` | bool, bool | 无 | 软件极性反转 |
| `bsp_motor_get_invert()` | bool*, bool* | 无 | 查询极性 |
| `bsp_motor_set_pwm_limit(lim)` | uint16 | 无 | 设 PWM 限幅 |
| `bsp_motor_update()` | 无 | 无 | ⭐ 1 kHz 周期任务 |
| `bsp_motor_get_feedback(fb)` | 结构体指针 | 无 | 获取反馈数据 |
| `bsp_motor_get_left_count()` | 无 | int32 | 读左轮计数 |
| `bsp_motor_get_right_count()` | 无 | int32 | 读右轮计数 |
| `bsp_motor_get_enc_irq_count()` | 无 | uint32 | 读 ISR 进入次数 |
| `bsp_motor_enc_irq_is_quenched()` | 无 | bool | 查询雪崩抑制状态 |
| `bsp_motor_get_button_irq_count()` | 无 | uint32 | 按键中断统计 |
| `bsp_motor_get_button_poll_count()` | 无 | uint32 | 按键轮询统计 |
| `bsp_motor_is_start_button_active()` | 无 | bool | 查询按键按下 |
| `bsp_motor_get_start_button_raw_level()` | 无 | bool | 读原始电平 |
| `bsp_motor_reset_encoders()` | 无 | 无 | 清空编码器计数 |
| `bsp_motor_consume_toggle_request()` | 无 | bool | 消费按键事件 |

---

## 🔄 三、核心逻辑总结

### 3.1 完整工作流程

```
SYSCFG_DL_init()
  → bsp_gpio_init()
  → bsp_systick_init(1000)
  → bsp_motor_init()         ← 1. 配 QEI 上拉 2. 清零 3. 安全态 4. 注册中断
  → bsp_motor_enable(true)   ← 5. 拉高 STBY，可以动了

  for (;;) {
      if (bsp_systick_consume_tick()) {
          bsp_motor_update();  ← 1 kHz：QEI 扩展 + 速度窗口 + 雪崩检测 + brake 计时

          if (tick % 5 == 0) {
              // 计算新的速度命令
              bsp_motor_set_output(left_pm, right_pm);

              // 获取反馈
              bsp_motor_feedback_t fb;
              bsp_motor_get_feedback(&fb);
          }
      }

      // 检查 S1 按键
      if (bsp_motor_consume_toggle_request()) {
          // 切换模式/启动/急停
      }
  }
```

### 3.2 命令处理流程

```
bsp_motor_set_output(600, -400)
  │
  ├─ 取消 pending brake pulse
  ├─ 保存命令值到 s_motor.left_cmd_pm / right_cmd_pm
  │
  ├─ commit_left(600)
  │     ├─ apply_limit(600) → 600（未超限幅）
  │     ├─ 极性翻转？否
  │     └─ apply_one_channel(600, true)
  │           ├─ dir = DIR_FORWARD
  │           ├─ set_dir_left(DIR_FORWARD) → AIN1=H, AIN2=L
  │           └─ set_pwm_duty(C0, 600) → 60% 占空比
  │
  └─ commit_right(-400)
        ├─ apply_limit(-400) → -400
        ├─ 极性翻转？否
        └─ apply_one_channel(-400, false)
              ├─ dir = DIR_REVERSE
              ├─ set_dir_right(DIR_REVERSE) → BIN1=L, BIN2=H
              └─ set_pwm_duty(C1, 400) → 40% 占空比
```

### 3.3 速度测量原理（滑动窗口差分）

```
每 1 ms（bsp_motor_update）：
  speed_window_acc_ms += 1

累计满 20 ms：
  读 left_count = 123456
  算 dl = 123456 - 上一拍 left_speed_prev_count (= 123000) = 456
  left_speed_cps = 456 × 1000 / 20 = 22800 counts/s
  换算为 rpm：
    22800 / 1320 × 60 = 1036 rpm
  left_speed_prev_count = 123456
  speed_window_acc_ms = 0

优点：滑动窗口平均值，比"瞬时差分"更平滑
缺点：响应有 20 ms 延迟（对于 200 Hz 控制环，可接受）
```

### 3.4 中断安全设计（三重保护）

**保护 1：GPIO 中断优先级设为最低**
```c
NVIC_SetPriority(GPIOA_INT_IRQn, 3);  // 最低
// SysTick 优先级 = 0（最高，在 bsp_systick.c 中设置）
```
即使 GPIO 中断风暴，SysTick 也能正常执行。

**保护 2：内部上拉 + 施密特触发**
```c
DL_GPIO_RESISTOR_PULL_UP | DL_GPIO_HYSTERESIS_ENABLE
```
引脚悬空时电平稳定，不会被环境噪声触发雪崩。

**保护 3：ISR 边沿率监控（雪崩兜底）**
```
每毫秒检查 enc_irq_window
超过 200 → 关闭编码器中断 50 ms → CPU 喘气 → 自动重新打开
```

### 3.5 左轮 QEI 16→32 位扩展

TIMG8 硬件 QEI 计数器只有 16 位（0~65535），电机转得快时几秒就溢出了。

解决方法：每次 update 读一次原始值，和上次的值做差：
```
上一拍 raw_prev = 65530
当前   raw_now  = 10
delta = (int16_t)(10 - 65530) = (int16_t)(−65520) = 16
left_count += 16
```

即使硬件计数器溢出，差值仍然是正确的！因为 `(int16_t)` 强制转换自动处理了 16 位回绕。

### 3.6 S1 按键双保险：中断 + 轮询

```
中断路径（正常）：
  PA18 电平变化 → GROUP1_IRQHandler → on_start_button_edge()
    → 80 ms 去抖 → toggle_request = 1

轮询兜底（备份）：
  bsp_motor_consume_toggle_request() 调用
    → 如果 toggle_request = 0 → poll_start_button()
    → 读当前电平 → 去抖 → toggle_request = 1
```

中断路径是主要的，轮询兜底是备份。如果中断函数名写错或中断被屏蔽，轮询还能兜住。

---

## 🧩 四、关键代码段详解

### 4.1 MOTOR_LOCK / MOTOR_UNLOCK

```c
#define MOTOR_LOCK()   __disable_irq()
#define MOTOR_UNLOCK() __enable_irq()
```

凡是被 ISR（GROUP1_IRQHandler）修改的共享变量（right_count、toggle_request 等），主循环在访问前必须关中断。

### 4.2 PWM 占空比计算

```c
compare = load - (load * permille / 1000)
```
- TIMA0 向上计数，INIT_VAL_LOW 极性
- permille=1000 → compare=0 → 一直高 → 100%
- permille=0 → compare=load → 一直低 → 0%

### 4.3 正交编码器解码

```c
int32_t step = (phase_a != phase_b) ? 1 : -1;
if (!is_phase_a_edge) step = -step;
step = -step;  // 物理安装方向相反
s_motor.right_count += step;
```

解码原理：A/B 电平不同 → 正转；相同 → 反转。PA13 触发时取反。

### 4.4 GROUP1_IRQHandler 坑

```c
// ❌ 错误：函数名不对，虽然编译通过但向量表指向的是默认死循环
void GPIOA_IRQHandler(void) { ... }

// ✅ 正确：MSPM0G3507 所有 GPIO 中断入口都是 GROUP1
void GROUP1_IRQHandler(void) { ... }
```

---

## 🐛 五、常见踩坑

| 现象 | 可能原因 | 解决方法 |
|------|---------|---------|
| 电机不转 | 忘记 bsp_motor_enable(true) | 拉高 STBY |
| 轮子反着转 | 极性设置不对 | 调用 bsp_motor_set_invert(true, ...) |
| 右轮计数异常 | ISR 函数名写错 | 确认是 GROUP1_IRQHandler |
| 系统卡死 | 编码器引脚浮空 | 确认上拉+施密特已配 |
| 速度波动大 | 速度窗口太小 | 增大 BSP_MOTOR_SPEED_WINDOW_MS |
| 按键不触发 | J8 跳线影响极性 | 确认 button_idle_high 检测正确 |
| PWM 有啸叫 | 频率太低了 | 确认 TIMA0 配置为 20 kHz |

---

> 本文档配合 `bsp_motor.h` 和 `bsp_motor.c` 中的详细注释阅读效果最佳。加油！
