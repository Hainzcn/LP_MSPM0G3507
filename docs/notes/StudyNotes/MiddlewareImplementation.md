# 中间层 (Middleware) 代码实现路径详解

上一篇文档中我们从概念上讲了中间层在做什么。这篇文档我们将深入到具体的代码实现，看看 C 语言是如何把抽象的数学公式和协议解析落地的。

---

## 1. 姿态解析器的底层实现 (`ms901m.c`)

这个文件虽然不涉及单片机的寄存器，但它对 C 语言中"状态机"和"数据类型转换"的应用非常经典。

### 1.1 状态机的高效运转 (`ms901m_feed_bytes`)
当你有一大串乱序的字节流入时，如何保证不漏掉任何一帧？
代码中使用了一个 `switch(s_state)` 语句，配合一个枚举类型 `parse_state_t`。
- **状态转移逻辑**：
  - 在 `ST_SYNC1` 状态下，只认 `0x55`。一旦收到，状态立马变成 `ST_SYNC2`。
  - 在 `ST_SYNC2` 状态下，再收一个 `0x55`，状态变成 `ST_ID`。
  - 如果在中间任何一个环节发现数据不对（比如校验和算出来对不上），状态机会调用 `reset_state_machine()`，把状态强行拨回 `ST_SYNC1`，并且把校验和清零，等待下一个 `0x55`。
- **为什么要这样写？**
  这种写法被称为**"非阻塞式解析"**。每次传入一个字节，函数只做极少量的判断就立刻返回，不会在里面用 `while` 循环死等数据。这对于需要保证 1 毫秒节拍准时执行的单片机系统来说至关重要。

### 1.2 字节拼接与量纲转换 (`parse_attitude` / `parse_gyro_acc`)
传感器发来的角度数据是被拆成两个 8 位字节（高位和低位）发送的。
- **拼接 (Little-Endian)**：
  代码里写了一个小工具函数 `le16(uint8_t lo, uint8_t hi)`，它通过位移操作 `(hi << 8) | lo` 把两个 8 位的碎片拼成了一个 16 位的有符号短整数 (`int16_t`)。
- **量纲转换 (Q15 格式还原)**：
  传感器为了省空间，把 -180° 到 +180° 映射到了 -32768 到 +32767 的整数范围。
  代码中的还原公式：
  ```c
  s_snap.pitch_deg = (float)le16(d[2], d[3]) * (180.0f / 32768.0f);
  ```
  编译器在编译时会提前把 `180.0f / 32768.0f` 算成常数，运行时只需一次浮点乘法。

---

## 2. `pid2_t` —— 当前平衡车使用的新一代控制器 (`pid2_update`)

项目中有两套 PID：当前平衡控制使用 `pid2_t`（对齐 STM32 demo），旧 `pid_t` 保留供参考。下面重点解析 `pid2_update()` 的代码实现。

### 2.1 P、I、D 的代码映射

```c
void pid2_update(pid2_t *p)
{
    float error = p->target - p->actual;

    // 积分：Ki=0 自动清零（关键创新）
    if (p->ki != 0.0f) {
        p->i_term += error;              // 每拍直接累加误差（不乘 dt）
        p->i_term  = clampf(p->i_term, p->i_min, p->i_max);
    } else {
        p->i_term = 0.0f;                // 关掉积分时无残留
    }

    // 微分先行：对 actual 变化率微分，非 error
    float d = p->has_prev ? (p->actual - p->prev_actual) : 0.0f;
    p->prev_actual = p->actual;
    p->has_prev    = true;

    float u = p->kp * error + p->ki * p->i_term - p->kd * d;
    // ... OutOffset + clamp → p->out
}
```

与旧 `pid_t` 的关键差异：
- **不传 `dt_sec`**：假设固定调用周期，增益已吸收时间因子
- **Ki=0 自动清零**：调试阶段关积分不会残留"债务"
- **微分先行**：`d = actual - prev_actual`（对测量值微分），而非对 `error` 微分。避免 setpoint 阶跃产生 D 项尖峰冲击

### 2.2 OutOffset —— 突破电机死区

```c
if (u > 0.0f)      u += p->out_offset;
else if (u < 0.0f) u -= p->out_offset;

p->out = clampf(u, p->out_min, p->out_max);
```

**为什么需要它？** TB6612 电机驱动有静摩擦死区：PWM < 60‰ 时电机完全不转。旧 `pid_t` 的死区补偿是把 < 死区阈值的命令直接映射到阈值，导致"突然无力→又突然冲出"的不连续问题。`out_offset` 在非零输出上叠加恒定偏置——只要 PID 有输出意图（哪怕 1‰），电机就能真正动起来。

### 2.3 与旧 `pid_t` 的对比

| 特性 | 旧 `pid_t` (`pid_step`) | 新 `pid2_t` (`pid2_update`) |
|------|------------------------|---------------------------|
| 时间参数 | 显式传 `dt_sec` | 固定周期，增益隐含 dt |
| 积分行为 | `ki * error * dt` 累加，含 anti-windup 条件逻辑 | `ki * error` 累加，Ki=0 自动清零 |
| D 项 | `(measured - prev) / dt`，带 EMA 低通 | `(actual - prev)`，微分先行 |
| 死区处理 | 依赖 BSP 层 | `out_offset` 内置 |
| 适用场景 | 通用 PID（可变周期、EMA 滤波） | 平衡车专用（固定周期、快速迭代） |

### 2.4 冗余积分限幅 `i_min/i_max`

`pid2_t` 的积分限幅**独立于**输出限幅 `out_min/out_max`。这意味着积分可以超过输出范围（如累积到 2000 但输出钳在 ±1000），给积分足够的"发力空间"，又不会让输出越界。

---

## 3. 车体运动学 (`robot_param.h`)

Stage 3.9 新增的模块，集中管理所有 mm 级物理量。

### 3.1 编译期常量链

```c
#define ROBOT_WHEEL_DIAMETER_MM    (65)
#define ROBOT_WHEEL_BASE_MM        (185)

// 轮周长 μm × 1000，整数乘除无害
#define ROBOT_WHEEL_CIRCUMFERENCE_UM_X1000 \
    ((int32_t)((3141593LL * ROBOT_WHEEL_DIAMETER_MM) / 1000LL))

// 每 mm 行程对应编码器计数 × 100
#define ROBOT_AVG_COUNTS_PER_MM_X100 \
    ((LEFT_COUNTS_PER_REV + RIGHT_COUNTS_PER_REV) / 2 * 100 * 1000 / CIRC_UM_X1000)
```

全链路整数运算，**零浮点开销**，Cortex-M0+ 直接高效执行。

### 3.2 inline 运动学函数

```c
static inline int32_t robot_v_mm_s_to_avg_cps(int32_t v_mm_s) {
    return (v_mm_s * ROBOT_AVG_COUNTS_PER_MM_X100) / 100;
}

static inline int32_t robot_omega_mrad_to_delta_cps(int32_t omega_mrad_s) {
    int32_t dv = omega_mrad_s * ROBOT_WHEEL_BASE_MM;
    return (int32_t)((int64_t)dv * ROBOT_AVG_COUNTS_PER_MM_X100 / (100LL * 1000LL));
}
```

圆弧运动 (`app_circle_demo`) 就是靠这些函数计算出来的：给定直径+速度 → 算出平均速度和差速 → 分别填入 `target_speed_cps`/`target_dif_cps` → 差速环闭环执行。

---

**总结：**
`ms901m.c` 展现了状态机解析的标准写法；`pid2_t` 展现了如何把 PID 理论加上"Ki=0 自动清零""微分先行""OutOffset 死区补偿"三大工业级保护，变成平衡车可直接用的控制器；`robot_param.h` 体现了嵌入式工程中"参数集中 + 编译期计算"的好习惯。
