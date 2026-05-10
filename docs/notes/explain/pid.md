# PID 控制器模块 学习笔记

## 📖 一、整体概述

### 1.1 这个模块在项目中的位置

本项目是一个**两轮自平衡瞄准小车**，主控为 **TI MSPM0G3507**。

PID 控制器是自平衡小车的"大脑"——它接收姿态传感器（MS901M）测得的**俯仰角 pitch**和**角速度 pitch_rate**，计算出**电机应该输出多大的力**来让小车保持平衡。

```
┌──────────────────────────────────────────────────────────┐
│                    app_balance.c                          │  ← 应用层
│  每 5 ms 调用一次 pid_step() 做一次控制计算                 │
└────────────────────┬─────────────────────────────────────┘
                     │
┌────────────────────▼─────────────────────────────────────┐
│                   pid.h / pid.c                           │  ← 中间层：PID 控制器（本模块）
│               位置式 PID + 抗积分饱和 + D 滤波              │
└────────────────────┬─────────────────────────────────────┘
                     │
┌────────────────────▼─────────────────────────────────────┐
│                   ms901m.c                                │  ← 中间层：姿态数据解析
│               提供 snap.pitch_deg 和 snap.gy_dps          │
└──────────────────────────────────────────────────────────┘
```

### 1.2 文件结构

| 文件 | 作用 |
|------|------|
| `pid.h` | 头文件：定义 `pid_t` 结构体、函数声明、详细注释 |
| `pid.c` | 实现文件：`pid_step()` 核心算法 + 所有配置函数的实现 |

### 1.3 核心功能模块

| 模块 | 对应函数 | 作用 |
|------|---------|------|
| **初始化** | `pid_init()` | 设为安全默认值（增益全 0，输出为 0） |
| **参数配置** | `pid_set_gains()` | 设置 Kp/Ki/Kd 三个增益 |
| **限幅配置** | `pid_set_output_limit()` | 限制输出范围 |
| **抗饱和配置** | `pid_set_integral_limit()` | 限制积分累积上限 |
| **滤波配置** | `pid_set_d_filter()` | 设置微分项低通滤波系数 |
| **状态复位** | `pid_reset()` | 清空积分和微分历史 |
| **核心计算** | `pid_step()` ⭐ | 每拍执行一次 PID 计算，返回控制量 |

---

## 📋 二、全局常量/变量/类型汇总

### 2.1 `pid_t` 结构体字段总表

`pid_t` 是本模块唯一的核心数据结构——它"记住"了一个 PID 控制器的全部状态。

| 字段 | 类型 | 默认值 | 含义 | 设置方式 |
|------|------|--------|------|---------|
| `kp` | `float` | `0.0` | 比例增益 | `pid_set_gains()` |
| `ki` | `float` | `0.0` | 积分增益 | `pid_set_gains()` |
| `kd` | `float` | `0.0` | 微分增益 | `pid_set_gains()` |
| `u_min` | `float` | `-1000.0` | 输出下限 | `pid_set_output_limit()` |
| `u_max` | `float` | `1000.0` | 输出上限 | `pid_set_output_limit()` |
| `i_max` | `float` | `0.0` | 积分限幅（0=跟随 u_max） | `pid_set_integral_limit()` |
| `d_filter_alpha` | `float` | `0.0` | D 项 EMA 滤波系数 | `pid_set_d_filter()` |
| `i_term` | `float` | `0.0` | 积分累积值（内部状态） | 被 `pid_step()` 自动更新 |
| `prev_meas` | `float` | `0.0` | 上一拍测量值（内部状态） | 被 `pid_step()` 自动更新 |
| `prev_d_filt` | `float` | `0.0` | 上一拍滤波值（内部状态） | 被 `pid_step()` 自动更新 |
| `has_prev` | `bool` | `false` | 首拍标志（内部状态） | 被 `pid_step()` 自动更新 |

### 2.2 内部辅助函数

| 函数 | 文件 | 作用 |
|------|------|------|
| `clampf(v, lo, hi)` | `.c` 文件 static | 把浮点数 v 限制在 [lo, hi] 范围内 |

---

## 📊 三、函数汇总

| 函数名 | 功能 | 入参 | 返回值 | 调用时机 |
|--------|------|------|--------|---------|
| `pid_init()` | 初始化为安全默认值 | `pid_t*` | 无 | 上电初始化一次 |
| `pid_set_gains()` | 设置 Kp/Ki/Kd | `pid_t*, kp, ki, kd` | 无 | 参数整定时 |
| `pid_set_output_limit()` | 设置输出限幅 | `pid_t*, lo, hi` | 无 | 初始化或换电机时 |
| `pid_set_integral_limit()` | 设置积分限幅 | `pid_t*, i_abs_max` | 无 | 防饱和调参 |
| `pid_set_d_filter()` | 设置 D 项滤波系数 | `pid_t*, alpha` | 无 | 调试噪声时 |
| `pid_reset()` | 清空内部状态 | `pid_t*` | 无 | 切换目标 / 重启控制 |
| `pid_step()` ⭐ | 执行一拍 PID 计算 | `pid_t*, target, measured, dt_sec` | `float` 控制量 | 控制循环每拍 |

---

## 🔄 四、核心逻辑总结

### 4.1 完整工作流程

```
┌─────────────────────────────────────────────────────────────────────┐
│                        1. 初始化阶段                                  │
│                                                                     │
│  pid_t balance_pid;               // 定义 PID 实例                   │
│  pid_init(&balance_pid);          // 设默认值：Kp=Ki=Kd=0           │
│  pid_set_gains(&balance_pid,      // 设置增益：核心调参！             │
│      1.5f, 0.05f, 0.10f);                                          │
│  pid_set_output_limit(            // 输出限幅：-1000 ~ +1000         │
│      &balance_pid, -1000, 1000);                                    │
│  pid_set_integral_limit(          // 积分限幅：500                   │
│      &balance_pid, 500);                                            │
└─────────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────────┐
│                        2. 控制循环（每 5 ms）                        │
│                                                                     │
│  float target = 0.0f;              // 目标角度：0°（直立）           │
│  float measured = snap.pitch_deg;  // 从 ms901m 获取当前俯仰角       │
│  float dt = 0.005f;                // 控制周期：5 ms                 │
│                                                                     │
│  float u = pid_step(&balance_pid,  // 执行 PID 计算                  │
│                    target, measured, dt);                            │
│                                                                     │
│  motor_set_output(u);              // 输出到电机驱动                │
└─────────────────────────────────────────────────────────────────────┘
```

### 4.2 pid_step() 核心算法详解

这是整个模块最核心的部分。每次调用执行以下 10 个步骤：

```
Step 1: error = target - measured
         计算当前误差。
         例：target=0°, measured=2.5° → error=-2.5°（前倾 2.5°）

Step 2: p_term = Kp × error
         比例项。Kp=1.5 → p_term = 1.5 × (-2.5) = -3.75
         含义：电机应该输出 -3.75 的力（负值=后退方向）来回正

Step 3: i_term += Ki × error × dt
         积分项累加。Ki=0.05, dt=0.005 → i_inc = 0.05 × (-2.5) × 0.005 = -0.000625
         含义：每次累加一点点，最终消除稳态误差

Step 4: i_term = clamp(i_term, -i_max, +i_max)
         抗积分饱和：限制积分项不超过 ±500

Step 5: d_meas = (measured - prev_meas) / dt
         对测量值求导，得到变化速度
         例：上一拍 measured=2.3°，当前=2.5° → d_meas = (2.5-2.3)/0.005 = 40 °/s

Step 6: d_filt = α × d_meas + (1-α) × prev_d_filt
         EMA 低通滤波。α=0.1 → 新值占 10%，历史占 90%

Step 7: d_term = -Kd × d_filt
         微分项。Kd=0.1 → d_term = -0.1 × 40 = -4.0
         ⚠️ 负号！因为测量值增大意味着"正在前倾"，需要反向力来刹车

Step 8: u_raw = p_term + i_term + d_term
         三项合成

Step 9: u = clamp(u_raw, u_min, u_max)
         输出限幅，确保不超出物理限制

Step 10: 如果饱和（u 被限幅）且积分还在同向累加 → 撤回积分增量
         抗积分饱和的核心逻辑
```

### 4.3 三种自平衡场景的 PID 行为

| 场景 | 误差 | P 项 | I 项 | D 项 | 总输出 | 效果 |
|------|------|------|------|------|--------|------|
| **车身直立** | error≈0 | ≈0 | 小 | ≈0 | ≈0 | 电机微调，保持平衡 |
| **突然前倾** | error 负大 | 负大 | 慢慢增负 | 负大（刹车） | 负大 | 电机反转，把车体推回 |
| **持续偏左** | error 小负 | 小负 | 持续累积 | ≈0 | 适中 | 积分消除静差 |

### 4.4 关键设计决策

#### 决策 1：位置式 PID vs 增量式 PID

| 特性 | 位置式（本模块） | 增量式 |
|------|-----------------|--------|
| 输出含义 | 控制量的**绝对值**（如 PWM=500） | 控制量的**变化量**（如 +50） |
| 积分方式 | 显式累积 i_term | 隐含在输出差分中 |
| 抗饱和 | 容易（直接钳位 i_term） | 较复杂 |
| 零命令输出 | 0（电机停转） | 0（保持上一次输出） |
| 本工程选择 | ✅ | ❌ |

**本工程选位置式的理由**：
- 平衡控制需要"绝对出力"——车倒多少度就要出多大力，不是"再加大多少"
- 输出限幅直接——"PWM 最大 1000"就是 u_max = 1000
- 零命令停转——目标角度=0° 且积分清零时，电机彻底停止

#### 决策 2：微分作用于测量值（d on measurement）

传统 PID 的微分项：`d_term = Kd × d(error)/dt`

但有一个严重问题：**当目标值（setpoint）突变时**，误差也会突变，导数产生巨大的尖峰（称为"微分冲击"或"derivative kick"）。

```
例子：目标角度从 0° 突然改为 10°
  error 从 0 突然变为 -10
  d(error)/dt = 无穷大！ → 电机瞬间全速运转，非常危险
```

本模块的解决方案：`d_term = -Kd × d(measured)/dt`

因为目标值突变不影响 measured 的变化率，所以微分项不会产生冲击。
这就是"d on measurement"的核心思想。

#### 决策 3：抗积分饱和（Anti-windup）

**积分饱和问题**：
```
1. 小车被卡住，无法达到目标
2. 积分项持续累加（"越够不着越使劲"）
3. 积分变得非常大（饱和）
4. 障碍移除后，积分还在"使劲"
5. 小车严重超调（冲出目标），积分需要很长时间才能"消化"
```

**本模块的抗饱和策略**：
```
在输出被限幅时，检查：
  - 如果输出在上限（u >= u_max）且积分还在向上累加（i_inc > 0）
  - 或者输出在下限（u <= u_min）且积分还在向下累加（i_inc < 0）
  → 撤回本拍的积分增量（i_term 保持不变）
```

效果：积分"知难而退"——既然输出已经到顶了，再累加也没用，先停下来。

---

## 🧩 五、pid.c 代码逐段详解

### 5.1 clampf —— 浮点数限幅工具

```c
static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}
```

这是一个 **static** 函数，只在 `pid.c` 内部使用。
三路比较：如果 v 低于下限→返回下限；高于上限→返回上限；在范围内→返回原值。

`clamp` 是 PID 控制器中最常用的操作——积分要钳位、输出要钳位。

### 5.2 pid_init —— 安全初始化

```c
void pid_init(pid_t *pid)
{
    if (pid == NULL) return;         // 防御性编程
    pid->kp = 0.0f;                  // 增益全为 0（安全！）
    pid->ki = 0.0f;
    pid->kd = 0.0f;
    pid->u_min = -1000.0f;           // 默认输出范围 ±1000
    pid->u_max =  1000.0f;
    pid->i_max = 0.0f;               // 0 = 跟随 u_max
    pid->d_filter_alpha = 0.0f;      // 禁用 D 滤波
    pid->i_term = 0.0f;              // 内部状态清零
    pid->prev_meas = 0.0f;
    pid->prev_d_filt = 0.0f;
    pid->has_prev = false;
}
```

**NULL 检查**：如果传入了 NULL 指针，直接返回不崩溃。这是嵌入式编程的基本素养。

**为什么 Kp=Ki=Kd=0？**
这不是"忘记设默认值"，而是**刻意的安全设计**。如果增益不为 0 但忘记调参，小车一上电可能就全速运转。而增益为 0 → 永远输出 0 → 小车不动 → 安全。

### 5.3 pid_set_gains —— 设定增益

```c
void pid_set_gains(pid_t *pid, float kp, float ki, float kd)
{
    if (pid == NULL) return;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}
```

这个函数简单到只有赋值——但它的意义不在代码量，而在**封装**。
外部代码通过这个函数设置增益，而不是直接写 `pid->kp = 1.5`。
好处是：未来如果想在设置增益时加额外逻辑（比如校验、日志），只需要修改这个函数。

### 5.4 pid_set_output_limit —— 输出限幅

```c
void pid_set_output_limit(pid_t *pid, float lo, float hi)
{
    if (pid == NULL) return;
    if (lo >= hi) return;            // lo 不能 >= hi，否则忽略
    pid->u_min = lo;
    pid->u_max = hi;
}
```

`if (lo >= hi)` 检查：如果下限 >= 上限，整个设置被忽略。
这是防止调用方传参错误——比如 `pid_set_output_limit(&pid, 100, -100)`（下限 > 上限）会导致混乱的限幅行为。

### 5.5 pid_set_integral_limit —— 积分限幅

```c
void pid_set_integral_limit(pid_t *pid, float i_abs_max)
{
    if (pid == NULL) return;
    if (i_abs_max < 0.0f) i_abs_max = 0.0f;  // 负值强制为 0
    pid->i_max = i_abs_max;
}
```

- `i_abs_max = 0`：积分上限自动跟随 `u_max`
- `i_abs_max > 0`：独立设置，通常设为 `u_max` 的 50%~80%
- `i_abs_max < 0`：强制改为 0（负数没有物理意义）

### 5.6 pid_set_d_filter —— 设置 D 项滤波

```c
void pid_set_d_filter(pid_t *pid, float alpha)
{
    if (pid == NULL) return;
    if (alpha < 0.0f) alpha = 0.0f;   // 钳位到 [0, 1]
    if (alpha > 1.0f) alpha = 1.0f;
    pid->d_filter_alpha = alpha;
}
```

alpha 被钳位到 `[0, 1]` 范围内——超出范围的输入被自动修正。这是"宽容性设计"。

### 5.7 pid_reset —— 复位内部状态

```c
void pid_reset(pid_t *pid)
{
    if (pid == NULL) return;
    pid->i_term = 0.0f;        // 积分清零
    pid->prev_meas = 0.0f;     // 微分历史清零
    pid->prev_d_filt = 0.0f;
    pid->has_prev = false;     // 首拍标志重置
}
```

复位后，Kp/Ki/Kd、限幅值、滤波系数**保持不变**，只有"历史记忆"被清空。

### 5.8 pid_step ⭐ —— 核心 PID 计算

这是整个模块最核心的函数。逐行分析：

#### 第 1 步：安全检查和误差计算

```c
if (pid == NULL) return 0.0f;    // NULL 保护
if (dt_sec <= 0.0f) return 0.0f; // 非法 dt 保护

float error = target - measured;  // 计算误差
```

**dt 检查**：如果 dt <= 0，直接返回 0。因为 dt=0 时微分项 `d_meas / dt` 会除以 0，导致无穷大。

#### 第 2 步：比例项 P

```c
float p_term = pid->kp * error;
```

最简单的部分：误差 × Kp。误差越大，P 项输出越大。

#### 第 3~4 步：积分项 I（含试探 + 钳位）

```c
float i_inc      = pid->ki * error * dt_sec;    // 本拍的积分增量
float i_try      = pid->i_term + i_inc;          // 试探性累加
float i_abs_max  = (pid->i_max > 0.0f) ? pid->i_max : pid->u_max;  // 实际上限
float i_abs_min  = -i_abs_max;                   // 实际下限
float i_clamped  = clampf(i_try, i_abs_min, i_abs_max);  // 钳位
```

关键细节：
- `i_abs_max` 动态决定：如果 `i_max > 0` 就用 i_max，否则用 `u_max`
- 先"试探性累加"，再钳位——后面还要根据输出饱和情况决定是否真正更新

**i_inc 的物理意义**：每 dt 时间内，积分项的增量 = Ki × error × dt。
dt 越小，每拍的增量越小——所以控制频率越高，积分累加越"细腻"。

#### 第 5~7 步：微分项 D（on measurement + EMA 滤波）

```c
float d_term = 0.0f;
if (pid->has_prev) {                            // 如果不是第一拍
    float d_meas = (measured - pid->prev_meas) / dt_sec;  // 对测量值求导
    float a      = pid->d_filter_alpha;
    float d_filt = (a > 0.0f)
        ? (a * d_meas + (1.0f - a) * pid->prev_d_filt)  // EMA 滤波
        : d_meas;                                          // 无滤波
    d_term            = -pid->kd * d_filt;       // 注意负号！
    pid->prev_d_filt  = d_filt;
} else {
    pid->has_prev    = true;                     // 第一拍：只记录，不计算
    pid->prev_d_filt = 0.0f;
}
pid->prev_meas = measured;
```

**微分项的负号为什么要单独写？**
传统 PID 公式中微分项是 `Kd × d(error)/dt`，但由于采用了"d on measurement"，
实际计算的是 `Kd × d(measured)/dt`。当测量值增大（车体前倾）时，
导数为正，但我们需要反向力来"刹车"，所以加负号：`d_term = -Kd × d_filt`。

**第一拍特殊处理**：
第一次调用 pid_step() 时没有"上一拍的测量值"，无法计算微分。
所以 `has_prev=false` 时，只记录当前测量值并设 `has_prev=true`，跳过微分计算。

**EMA 滤波**：
```
d_filt = alpha × d_meas + (1 - alpha) × prev_d_filt
```
- alpha=0.1：新值占 10%，历史占 90% → 强滤波，平滑
- alpha=0.0：完全用历史值（但这里被特殊处理为禁用滤波）
- alpha=1.0：完全用新值，等同无滤波

#### 第 8~9 步：合成 + 限幅

```c
float u_raw = p_term + i_clamped + d_term;       // 三项合成
float u     = clampf(u_raw, pid->u_min, pid->u_max);  // 输出限幅
```

`u_raw` 是限幅前的"原始值"——保存它用于第 10 步的饱和判断。

#### 第 10 步：抗积分饱和（关键！）

```c
bool saturated_high = (u >= pid->u_max) && (u_raw >= pid->u_max);
bool saturated_low  = (u <= pid->u_min) && (u_raw <= pid->u_min);

if ((saturated_high && i_inc > 0.0f) ||
    (saturated_low  && i_inc < 0.0f)) {
    // 不更新 i_term（撤回本拍增量）
} else {
    pid->i_term = i_clamped;  // 正常更新
}
```

**饱和判断逻辑**：
- `saturated_high = true`：输出卡在了上限（如 PWM=1000）且原始值也 >= 上限
- `saturated_low = true`：输出卡在了下限（如 PWM=-1000）且原始值也 <= 下限

**积分撤回条件**：
- 上限饱和 + 积分还在向上累加（i_inc > 0）→ 撤回
- 下限饱和 + 积分还在向下累加（i_inc < 0）→ 撤回

**通俗理解**：
"既然输出已经到顶了，积分就别再往上加了，加了也没用。
等误差方向反转了，积分再开始反向累加。"

---

## 🔍 六、C 语言关键语法知识点

### 6.1 结构体指针和成员访问

```c
void pid_set_gains(pid_t *pid, float kp, float ki, float kd)
{
    pid->kp = kp;  // 通过指针访问结构体成员用 ->
    // 等价于 (*pid).kp = kp
}
```

`pid->kp` 是 `(*pid).kp` 的简写。指针 + 箭头 → 访问结构体成员。

为什么用指针而不是直接传结构体？
- 传指针：只传 4 字节（32 位地址），快
- 传结构体：复制整个结构体（44 字节），慢
- 而且传指针可以修改原结构体，传值只能改副本

### 6.2 NULL 指针检查

```c
if (pid == NULL) return;
```

每个公开函数的第一行都是 NULL 检查。这是嵌入式开发的"黄金法则"：
**永远不要假设调用方会传有效指针**。

### 6.3 float 的 f 后缀

```c
float kp = 1.5f;      // float 类型
float kp2 = 1.5;      // double 类型（隐式转 float）
```

C 语言中，`1.5` 默认是 double 类型，`1.5f` 才是 float 类型。
虽然 `float x = 1.5` 会隐式转换，但写 `1.5f` 更明确，也避免编译器警告。

对于 Cortex-M0+（无 FPU），float 运算比 double 快 2~3 倍。

### 6.4 三元运算符

```c
float i_abs_max = (pid->i_max > 0.0f) ? pid->i_max : pid->u_max;
```

相当于：
```c
float i_abs_max;
if (pid->i_max > 0.0f) {
    i_abs_max = pid->i_max;
} else {
    i_abs_max = pid->u_max;
}
```

三元运算符让简短的条件赋值更紧凑。

### 6.5 static 函数的封装

```c
static float clampf(float v, float lo, float hi)
```

`static` 让 `clampf` 只在 `pid.c` 内部可见，外部无法调用。
这是一种"模块内部工具函数"的封装方式。

---

## ⚡ 七、PID 调参实战指南

### 7.1 自平衡小车的调参顺序

对于自平衡小车，PID 调参建议按以下顺序：

#### 第一步：只加 Kp（比例）

```
Kp=0 → 慢慢增加到小车开始轻微来回摆动
此时：小车能回正，但会来回震荡，停不下来
```

#### 第二步：加 Kd（微分）

```
慢慢增加 Kd 直到震荡消失
此时：小车能稳定直立，但可能有"静差"（偏一点点）
```

#### 第三步：加 Ki（积分）

```
慢慢增加 Ki 消除静差
注意 Ki 很小（通常是 Kp 的 1/10 ~ 1/100）
Ki 太大 → 积分饱和 → 来回震荡
```

### 7.2 参数影响速查表

| 参数 | 太小 | 合适 | 太大 |
|------|------|------|------|
| **Kp** | 响应慢，推不动 | 快速回正，略有超调 | 剧烈震荡，可能失控 |
| **Ki** | 有静差（总偏一点） | 消除静差，定位准确 | 积分饱和，来回震荡 |
| **Kd** | 超调大，稳定慢 | 快速稳定，抑制震荡 | 对噪声敏感，电机抖动 |

### 7.3 实践中常见的调试方法

```
方法一：Ziegler-Nichols 法（经典）
  1. Ki=0, Kd=0
  2. 增大 Kp 直到系统持续等幅震荡（临界震荡）
  3. 记录此时的 Ku（临界增益）和 Tu（震荡周期）
  4. 按公式计算 Kp/Ki/Kd

方法二：试凑法（实践中更常用）
  1. Ki=0, Kd=0
  2. 增大 Kp 直到小车能勉强站住但会晃
  3. 加 Kd 抑制晃动
  4. 如果还有静差，加少量 Ki
  5. 重复 2~4 直到满意
```

---

## 🐛 八、常见错误与调试方法

### 8.1 常见问题排查

| 现象 | 可能原因 | 解决方法 |
|------|---------|---------|
| 电机不动 | 增益全为 0 | 检查是否调用了 `pid_set_gains()` |
| 剧烈震荡 | Kp 太大 或 Kd 太小 | 减小 Kp，增加 Kd |
| 慢悠悠晃动 | Kp 太小 或 Kd 太小 | 增加 Kp，适当加 Kd |
| 总偏一个方向 | Ki 不够 | 增加 Ki |
| 一启动就冲出去 | 积分饱和 或 D 项冲击 | 调用 `pid_reset()` 清状态 |
| 高频抖动 | Kd 太大，噪声放大 | 减小 Kd，开启 D 滤波（alpha=0.1） |
| 电机声音异常 | PWM 频率不合适 | 检查 PWM 频率（建议 15~25 kHz） |

### 8.2 dt 不一致的问题

**问题**：控制循环的 dt 不固定（有时 4 ms，有时 6 ms）

**后果**：积分项增量 `Ki × error × dt` 忽大忽小，控制器行为不稳定

**解决方法**：
- 软件定时器精确控制调用间隔
- 或每次调用时测量实际经过的时间

### 8.3 忘记复位的问题

**问题**：切换控制模式（比如从"原地平衡"切到"循迹行进"）时没有调用 `pid_reset()`

**后果**：原地平衡时的积分累积值会带到新模式中，导致起步瞬间电机异常出力

**解决方法**：切换模式前调用 `pid_reset(&pid)`

---

## 📖 九、扩展阅读

### 9.1 什么是 PID 控制器？

PID 控制器是工业控制中**最广泛使用**的控制算法（没有之一）。

**名字的含义**：
- **P**roportional（比例）：看现在
- **I**ntegral（积分）：看过去
- **D**erivative（微分）：看未来

**现实类比：开车**
- P：看到要偏离车道了→打方向盘（比例于偏离程度）
- I：一直在偏→持续修正（累积修正量）
- D：看到偏离速度很快→提前回正（预判趋势）

### 9.2 位置式 PID vs 增量式 PID

| 特性 | 位置式 | 增量式 |
|------|--------|--------|
| 输出 | 绝对位置（如 PWM=500） | 增量（如 +50） |
| 积分 | 显式累积 i_term | 隐含累积 |
| 抗饱和 | 容易 | 较复杂 |
| 积分清零 | 设置 i_term=0 | 需要累积差分 |
| 适用场景 | 电机PWM、阀门开度 | 步进电机位置、增量调节 |

### 9.3 什么是积分饱和（Integral Windup）？

积分饱和是指积分项累积到非常大的值，导致控制器反应迟钝的现象。

**生活类比**：
你在调空调温度。目标 25°C，当前 30°C。
- 每隔 5 分钟调低 1°C（积分累积）
- 到 25°C 时，你已经累积了"调低 5°C"的积分
- 但实际上 25°C 已经够了，积分残留导致温度继续下降（超调）
- 你需要等积分慢慢"消化"，温度才能稳定

这就是积分饱和——过去累积的"纠正量"在达到目标后还在起作用。

### 9.4 什么是 EMA 滤波？

EMA = Exponential Moving Average，指数移动平均。

公式：`y[n] = α × x[n] + (1 - α) × y[n-1]`

- `x[n]`：当前测量值（新值）
- `y[n-1]`：上一拍的滤波值（历史）
- `α`：滤波系数（alpha）
- `y[n]`：当前滤波结果

**alpha 的影响**：
- α=1.0：y[n] = x[n]（完全跟随，无滤波）
- α=0.1：y[n] = 0.1×x[n] + 0.9×y[n-1]（强滤波）
- α=0.5：y[n] = 0.5×x[n] + 0.5×y[n-1]（中等滤波）

**对微分项的意义**：
微分项对噪声极度敏感——测量值微小抖动就会产生巨大的导数。
EMA 滤波能"压平"这些毛刺，让微分项的输出更平滑。

### 9.5 什么是千分比（permille）量纲？

本模块的输出默认使用千分比（‰）量纲，范围 -1000 ~ +1000。
相对于百分比（%），千分比的精度更高（0.1% 的分辨率）。

1000 ‰ = 100% = 电机满速
500 ‰ = 50% = 电机半速
-1000 ‰ = 反向满速

---

---

## 🧩 十、pid.c 代码逐段详解

### 10.1 头文件包含与 #include 细节

```c
#include "pid.h"
#include <stddef.h>
```

- `#include "pid.h"`：用引号 `""` 包含"自己写的"头文件
- `#include <stddef.h>`：用尖括号 `<>` 包含"编译器的标准库"头文件

**为什么 pid.c 需要 `#include <stddef.h>`？**
`stddef.h` 定义了 `NULL` 宏。虽然 `pid.h` 通过 `<stdbool.h>` 间接引入了一些标准库内容，但 `NULL` 的定义在 `stddef.h` 中（或者 `stdio.h`、`stdlib.h` 中也会定义）。在 Keil MDK 等嵌入式开发工具链中，有些头文件不自动包含 `NULL` 定义，所以显式包含 `stddef.h` 是"安全"的做法。

**引号 vs 尖括号的区别**：
| 写法 | 搜索路径 | 用于 |
|------|---------|------|
| `#include "pid.h"` | 先从"当前目录"找，找不到再去系统目录 | 项目自己的头文件 |
| `#include <stddef.h>` | 直接从"系统目录"找 | 编译器的标准库头文件 |

### 10.2 clampf() —— 浮点数限幅工具

```c
static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}
```

**static 的作用**：这个函数只在 `pid.c` 内部使用，其他 `.c` 文件无法调用。这是"封装"——模块内部的实现细节对外部隐藏。

**三路比较的顺序问题**：先检查下限 `v < lo`，再检查上限 `v > hi`。这个顺序没有严格规定，但先检查下限是一种习惯——因为"下限"通常比"上限"在数值上更小，CPU 比较时更符合直觉。

**为什么不用标准库的 `fminf`/`fmaxf`？**
- 减少依赖：嵌入式标准库可能不完整
- 性能：`clampf` 只有简单的 `if` 比较，编译器会内联优化，几乎没有调用开销
- 可移植：不依赖任何特定平台的数学库

### 10.3 pid_init() —— 安全初始化

```c
void pid_init(pid_t *pid)
{
    if (pid == NULL) return;
    pid->kp = 0.0f;
    pid->ki = 0.0f;
    pid->kd = 0.0f;
    // ... 其他字段清零
}
```

**NULL 检查为什么是第一行？**
嵌入式系统中最常见的崩溃原因之一就是"解引用空指针"。如果调用方写了：
```c
pid_t *p = NULL;  // 忘记分配内存或忘记取地址
pid_init(p);       // 没有 NULL 检查 → pid->kp = 0.0f 会访问地址 0 → HardFault！
```
有 NULL 检查时，程序安全返回，不会崩溃。

**为什么不把全部字段设为一个"表格式"的初始值？**
比如 `pid->u_min = -1000.0f;` 为什么不直接用 `memset(pid, 0, sizeof(*pid))` 再个别赋值？因为 `memset` 把所有字节设为 0，但 `float 0.0` 的二进制表示就是全 0，所以理论上 `memset` 也可以。但逐个赋值更加**明确**——读者一眼就能看到每个字段的默认值是什么。

### 10.4 pid_set_gains() —— 设置增益

```c
void pid_set_gains(pid_t *pid, float kp, float ki, float kd)
{
    if (pid == NULL) return;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}
```

这个函数简单到只有三行赋值——但它体现了一个重要的设计原则：**封装**。

**为什么不让调用方直接写 `pid->kp = 1.5`？**
因为如果将来想在设置增益时增加额外的逻辑（比如输出日志、做合法性检查、触发电机动静），只需要修改 `pid_set_gains()` 这个函数，而不需要修改所有调用方。

### 10.5 pid_set_output_limit() —— 输出限幅

```c
void pid_set_output_limit(pid_t *pid, float lo, float hi)
{
    if (pid == NULL) return;
    if (lo >= hi) return;     // 关键！下限 >= 上限 → 无效设置
    pid->u_min = lo;
    pid->u_max = hi;
}
```

**`if (lo >= hi)` 检查为什么重要？**
假设调用方误传了 `pid_set_output_limit(&pid, 500, -500)`，即下限 500 > 上限 -500。如果不检查，`u_min = 500, u_max = -500`，所有输出都会被限幅到 `[500, -500]`——这是一个空区间！`clampf()` 会错误工作，输出永远不可能是正确的值。

**为什么用 `>=` 而不是 `>`？**
`lo == hi` 也是无效的——输出被钉死在一个固定值，PID 无法调节。所以 `>=` 覆盖了"下限大于上限"和"下限等于上限"两种无效情况。

### 10.6 pid_set_integral_limit() —— 积分限幅

```c
void pid_set_integral_limit(pid_t *pid, float i_abs_max)
{
    if (pid == NULL) return;
    if (i_abs_max < 0.0f) i_abs_max = 0.0f;  // 负值钳位到 0
    pid->i_max = i_abs_max;
}
```

**负数钳位到 0 的逻辑**：`i_abs_max` 是"绝对值上限"，负数没有物理意义。相比于"返回忽略设置"（silently ignore），这里做了"自动修正"（auto-correct），更加用户友好。

这种设计叫**宽容性设计**（tolerant design）：输入不合法时，不是粗暴地拒绝，而是自动修正为最接近的合法值。

### 10.7 pid_set_d_filter() —— 设置 D 项滤波

```c
void pid_set_d_filter(pid_t *pid, float alpha)
{
    if (pid == NULL) return;
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;
    pid->d_filter_alpha = alpha;
}
```

这里的两个 `if` 把 alpha 钳位到 `[0, 1]` 范围内。和 `pid_set_integral_limit` 一样，也是"宽容性设计"。

**为什么不用 `clampf()` 函数来钳位 alpha？**
技术上可以这么写：`alpha = clampf(alpha, 0.0f, 1.0f);` 但作者选择了显式的 if 语句。这不是对错问题，而是风格问题——有些开发者认为明确写出"如果小于 0 就设为 0"比调用一个通用函数更直白。

### 10.8 pid_reset() —— 复位内部状态

```c
void pid_reset(pid_t *pid)
{
    if (pid == NULL) return;
    pid->i_term      = 0.0f;
    pid->prev_meas   = 0.0f;
    pid->prev_d_filt = 0.0f;
    pid->has_prev    = false;
}
```

**为什么复位后 `has_prev = false`？**
`has_prev` 控制着微分计算是否执行。设为 `false` 后，下一拍 `pid_step()` 会：
1. 进入 `else` 分支（`has_prev == false`）
2. 设置 `has_prev = true`
3. 只记录 `prev_meas = measured`
4. 不计算微分项（`d_term = 0.0f`）
5. 再下一拍才开始正常计算微分

这就像一个"暖机"过程——复位后第一拍不计算 D 项，避免微分项的初始冲击。

### 10.9 pid_step() ⭐ —— 逐行深度解析

这是整个 PID 模块最核心的函数。下面逐行解读每一段代码的设计意图。

#### 10.9.1 第 1 步：安全检查

```c
if (pid == NULL) return 0.0f;
if (dt_sec <= 0.0f) return 0.0f;
```

**两个检查，两个不同的保护目的**：
- `pid == NULL`：保护"内存访问安全"——不访问无效指针
- `dt_sec <= 0.0f`：保护"数学运算安全"——避免除以零

**为什么 `dt_sec = 0` 也不行？**
因为代码后面有 `d_meas = (measured - prev_meas) / dt_sec`。如果 `dt_sec = 0`，这是"除以零"操作，结果在 IEEE 754 浮点标准中是 `+Inf`（正无穷大）或 `NaN`（不是一个数）。一旦出现 `NaN`，整个计算链都会被污染——任何包含 `NaN` 的运算结果都是 `NaN`。

#### 10.9.2 第 2 步：计算误差

```c
float error = target - measured;
```

**error 的正负号约定**：`error > 0` 表示"测量值低于目标"，需要正向出力。`error < 0` 表示"测量值高于目标"，需要反向出力。

**为什么是 `target - measured` 而不是 `measured - target`？**
这是控制理论的约定俗成——"误差 = 目标 - 测量值"。当 measured < target（如目标 0°，当前 -5°）时，error > 0，需要正向出力把车体推到 0°。

#### 10.9.3 第 3 步：比例项 P

```c
float p_term = pid->kp * error;
```

**P 项的局限性**：如果只有 P 控制，系统会存在"稳态误差"——即永远差一点点到目标。这是因为 P 项的输出和误差成正比，当误差很小时，P 项输出也很小，可能不足以克服摩擦力等阻力。这就需要用 I 项来消除。

#### 10.9.4 第 4 步：积分试探

```c
float i_inc = pid->ki * error * dt_sec;
float i_try = pid->i_term + i_inc;
```

**为什么积分要乘 dt？**
积分是对时间的累积。如果不乘 dt，同样的 Ki 在不同控制频率（dt 不同）下效果完全不同。比如：
- dt = 0.005s（200 Hz）：每拍 i_inc = Ki × error × 0.005
- dt = 0.010s（100 Hz）：每拍 i_inc = Ki × error × 0.01（两倍！）

如果 Ki 在 200 Hz 下调好了，换到 100 Hz 不改变 Ki 值的话，积分效果会翻倍，系统可能失控。乘 dt 后，Ki 的量纲变为 `1/秒`，积分效果与控制频率无关。

**为什么用 `i_try` 做"试探"而不是直接更新 `i_term`？**
因为后面第 10 步（抗积分饱和）可能会决定撤回这个积分增量。先用 `i_try` 存着，最后再决定要不要写入 `pid->i_term`。

#### 10.9.5 第 5 步：积分钳位

```c
float i_abs_max = (pid->i_max > 0.0f) ? pid->i_max : pid->u_max;
float i_abs_min = -i_abs_max;
float i_clamped = clampf(i_try, i_abs_min, i_abs_max);
```

**`i_max = 0` 的特殊含义**：用户没有显式设置积分上限时，自动使用输出上限 `u_max`。这样设计的理由是："既然输出都不能超过 u_max，那积分也不应该比输出更猛。"

**正负对称**：积分上限是 `+i_abs_max`，下限是 `-i_abs_max`。因为误差可能是正也可能是负，积分可以在两个方向上累积。

#### 10.9.6 第 6~7 步：微分计算与滤波

```c
if (pid->has_prev) {
    float d_meas = (measured - pid->prev_meas) / dt_sec;
    float a = pid->d_filter_alpha;
    float d_filt = (a > 0.0f)
        ? (a * d_meas + (1.0f - a) * pid->prev_d_filt)
        : d_meas;
    d_term = -pid->kd * d_filt;
    pid->prev_d_filt = d_filt;
} else {
    pid->has_prev = true;
    pid->prev_d_filt = 0.0f;
}
pid->prev_meas = measured;
```

**微分项的负号为何如此重要？**
假设车体正在前倾，measured 从 2.0° 变为 2.5°：
- `d_meas = (2.5 - 2.0) / 0.005 = 100 °/s`（正数，表示正在前倾）
- 我们需要**反向力**来刹车，所以 `d_term = -Kd × 100`（负数）
- 如果没有负号：`d_term = +Kd × 100`（正数）→ 推波助澜，加速前倾！

**EMA 滤波的计算逻辑**：
`d_filt = (a > 0.0f) ? (a × d_meas + (1-a) × prev_d_filt) : d_meas`

这是一个"条件表达式"（三元运算符）：当 `a > 0` 时做 EMA 滤波，否则 `d_filt = d_meas`（原值直通）。

**为什么 `a > 0.0f` 而不是 `a >= 0.0f`？**
因为 `a = 0.0f` 时，滤波公式变为 `d_filt = 0 × d_meas + 1 × prev_d_filt = prev_d_filt`——永远只有历史值，新值永远进不来。所以 `a = 0.0` 被当作"禁用滤波"处理。

#### 10.9.7 第 8~9 步：合成 + 限幅

```c
float u_raw = p_term + i_clamped + d_term;
float u     = clampf(u_raw, pid->u_min, pid->u_max);
```

这里用了**两个变量** `u_raw` 和 `u`，而不是直接用 `u`：
- `u_raw` = 限幅前的原始值（用于第 10 步判断是否饱和）
- `u` = 限幅后的最终值（返回给调用方）

**为什么不能省掉 `u_raw`？**
因为限幅后的 `u` 永远在 `[u_min, u_max]` 内，无法判断"本真的输出意愿"是否超过了限幅。`u_raw` 保存了限幅前的值，用于饱和检测。

#### 10.9.8 第 10 步：抗积分饱和

```c
bool saturated_high = (u >= pid->u_max) && (u_raw >= pid->u_max);
bool saturated_low  = (u <= pid->u_min) && (u_raw <= pid->u_min);

if ((saturated_high && i_inc > 0.0f) ||
    (saturated_low  && i_inc < 0.0f)) {
    // 撤回：不更新 i_term
} else {
    pid->i_term = i_clamped;
}
```

**饱和检测的细节**：为什么既要检查 `u` 又要检查 `u_raw`？

```
情况 1：u_raw = 1200, u = clamp(1200, -1000, 1000) = 1000
        → saturated_high = (1000 >= 1000) && (1200 >= 1000) = true ✅ 真的饱和了

情况 2：u_raw = 800, u = clamp(800, -1000, 1000) = 800
        → saturated_high = (800 >= 1000) && (800 >= 1000) = false ✅ 没有饱和（正常范围）
```

如果只检查 `u >= u_max`：情况 1 中 `u=1000 >= 1000=true`，但情况 2 中 `u=800 >= 1000=false`，不会误判。但有些边界情况下只检查 u 会漏判。两个条件同时检查更可靠。

**撤回条件的直观理解**：
- `saturated_high && i_inc > 0`：输出已经顶到天花板了，积分还想往上加 → "别再用力了，没用！"
- `saturated_low && i_inc < 0`：输出已经踩到地板了，积分还想往下加 → "别再踩了，到底了！"

**空的 if 分支（什么都不做）**：
```c
if (...) {
    // 空：什么都不做，就是"撤回"
} else {
    pid->i_term = i_clamped;
}
```
有些初学者可能觉得空分支很奇怪。其实这就是"撤回"的实现——不更新 `i_term`，保留上一拍的值。相当于本拍的积分增量被"撤回"了。

### 10.10 pid.c 的代码设计模式总结

| 设计模式 | 体现位置 | 说明 |
|---------|---------|------|
| **防御性编程** | 所有函数的 NULL 检查 | 永远假设调用方可能传无效参数 |
| **安全默认值** | `pid_init()` 中 Kp=Ki=Kd=0 | 忘记设增益时输出为 0，安全 |
| **宽容性输入** | `pid_set_d_filter()` 的钳位 | 不合法输入自动修正为合法值 |
| **试探-提交** | `i_try` → `i_clamped` → 决策是否提交 | 先试探计算，最后决定是否真正更新 |
| **封装** | static 函数 + 公开 API | 内部细节对外隐藏，只暴露必要接口 |
| **早期返回** | NULL 检查在第 1 行 | 不满足前置条件就立即返回，减少嵌套 |

---

> 本文档配合 `pid.h` 和 `pid.c` 中的详细注释阅读效果最佳。继续加油！