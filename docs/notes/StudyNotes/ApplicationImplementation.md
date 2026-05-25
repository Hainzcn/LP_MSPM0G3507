# 应用层 (App) 代码实现路径详解

在这个项目的最高层，应用层的代码不再纠结于如何把引脚拉高、或者如何写一个微分公式，而是**做决策**和**做调度**。这部分重点剖析安全状态机 (`app_safety.c`) 和平衡大脑的主控制流 (`app_balance.c`) 的代码落地细节。

---

## 1. 安全管家的优先级决策逻辑 (`app_safety.c`)

在 `app_safety_tick` 函数中，安全管家需要在每一次控制节拍（100Hz）时，综合各方信息决定当前小车处于什么安全级别。

### 1.1 动作拦截与"按需操作硬件" (`transition` 函数)
改变状态不仅是改个变量名，还需要让电机做出物理反应。为了防止每 10 毫秒都重复给电机发一次"急刹"指令，代码里写了一个 `transition(app_safety_state_t next)` 包装函数——只在**状态发生跳变的那一瞬间**执行一次硬件动作。

### 1.2 多事件优先级判定 (if-else 瀑布流)
小车可能同时面临多个情况：解除武装 + 电池没电 + 倒在地板上。听谁的？代码通过精心设计的 `if - else if` 瀑布流解决优先级：**LOW_STOP > FALLEN > LOW_WARN > ARMED**。

### 1.3 栈溢出防护 (canary)
安全管家的状态变量包裹在 `app_safety_block_t` 结构体中，首尾各有 `canary_start` / `canary_end` 魔数。每次读取状态时校验，若 canary 损坏则自动复位到 `DISARMED` 并打印 `[safety] CORRUPTION`。这是 2026-05-24 第二次栈溢出事故后加入的防护。

---

## 2. 平衡大脑的多级 PID 调度 (`app_balance.c`)

这里是单片机算力的集中消耗地，它需要精准的时间控制。

### 2.1 四级 PID 的代码连接

当前实现包含四路 `pid2_t` 控制器，按以下链路组织：

```c
// === 20 Hz：速度外环 → target_tilt_deg ===
s_bal.speed_pid.target = target_norm;         // 归一化速度目标（含 EMA 低通）
s_bal.speed_pid.actual = speed_lpf_cps;       // EMA 滤波后的实测速度
pid2_update(&s_bal.speed_pid);
s_bal.target_tilt_deg = s_bal.speed_pid.out;

// === 20 Hz：航向角环 → yaw_dif_cps（差速环目标） ===
s_bal.yaw_pid.target = 0.0f;                  // 锁 yaw=0
s_bal.yaw_pid.actual = yaw_measured;          // gz积分 或 EKF yaw_deg
pid2_update(&s_bal.yaw_pid);
int32_t yaw_dif_cps = clampf(s_bal.yaw_pid.out, YAW_MAX_DIF_CPS);

// === 20 Hz：圆运动覆盖（若激活） ===
app_circle_demo_tick_20hz(&snap, &fb, cmd);
// 激活时 cmd->target_speed_cps / target_dif_cps 被圆运动覆盖

// === 20 Hz：差速环 → diff_out_pm ===
int32_t dif_target = (cmd->target_dif_cps != 0) ? cmd->target_dif_cps : yaw_dif_cps;
s_bal.diff_pid.target = dif_target * DIFF_NORM_TO_RPM;  // cps → RPM
s_bal.diff_pid.actual = diff_meas_rpm;                   // EMA 滤波后的 RPM
pid2_update(&s_bal.diff_pid);

// === 100 Hz：角度环 → ave_pwm ===
s_bal.angle_pid.target = s_bal.target_tilt_deg;
s_bal.angle_pid.actual = pitch_meas;
pid2_update(&s_bal.angle_pid);
float ave_pwm = s_bal.angle_pid.out;

// === 最终合成 ===
int16_t left_pm  = clamp_pwm_pm(ave_pwm + diff_out_pm);
int16_t right_pm = clamp_pwm_pm(ave_pwm - diff_out_pm);
bsp_motor_set_output(left_pm, right_pm);
```

**关键设计点**：
- 角度环在 100 Hz 消费最新的 `target_tilt_deg`（由 20 Hz 速度环产生），"一次计算、一次消费"
- 航向环输出不再是直接 PWM，而是差速环的**速度目标**——差速环以 RPM 闭环补偿 PWM→速度的时变映射
- 圆运动 (`app_circle_demo_tick`) 寄生在 20 Hz 分支，激活时覆盖 `cmd` 字段
- 差速环目标优先使用 K230/圆运动的 `target_dif_cps`，若无外部指令则使用航向环输出

### 2.2 EMA 目标低通——防止阶跃振荡

速度目标和差速目标在进入 PID 前经过一阶 EMA 低通：

```c
// 速度目标平滑（防阶跃→倾角突变→"微倾启动→后仰回退→再启动"顿挫）
s_bal.speed_target_lpf += SPEED_TARGET_LPF_ALPHA * (raw_target - s_bal.speed_target_lpf);
// α=0.10 @20Hz → τ≈330ms → 约0.6s爬升至90%

// 差速目标平滑
s_bal.diff_target_lpf += DIFF_TARGET_LPF_ALPHA * (raw_target - s_bal.diff_target_lpf);
// α=0.20 @20Hz → τ≈250ms
```

### 2.3 `main.c` 里的"启动保险"
如果在代码刚烧录进去、车还没调好时，开机直接进平衡模式是非常危险的。
所以在 `main.c` 里有这样一段逻辑：
1. 所有的 PID 参数 (`Kp`, `Ki`, `Kd`) 在初始化时**全部被默认设为了 0**。这意味着即使进了平衡循环，车子也是软绵绵的，不会发疯。
2. 只有你在测试完电机之后，通过串口注入增益（如 `bp 5000 0 5000 80`），车子才会真正获得站立的力量。

### 2.4 日志打印的去浮点化技巧
在 1 Hz 日志打印里，不会直接写 `printf("%f", pitch)`。而是用宏把浮点数 ×100 变成整数，再用 `%d.%02d` 拼接。

**为什么不直接用 `%f`？** Cortex-M0+ 没有 FPU，`printf("%f")` 单次栈帧 200~300 B，极易栈溢出（HardFault）。这是嵌入式开发中极为经典的**"去浮点化"**内存优化技巧。栈已扩至 **2 KB**（经历了两次栈溢出事故后）。

---

**总结：**
应用层的代码充满了"工程智慧"。它不仅仅是跑通算法，更多的是在考虑：
- 如何不让 CPU 被日志打印卡死（去浮点化）。
- 如何让物理动作在状态切换时不冗余执行（状态回滞和拦截）。
- 如何处理错综复杂的极端情况（安全状态机瀑布流 + canary 防护）。
- 如何用多级 PID 链补偿电池电压/地面摩擦的时变性（航向→差速→PWM）。
- 如何避免阶跃指令引发顿挫（EMA 目标低通）。

读懂了这层代码，你基本上就掌握了编写一个稳定可靠的小型机器人控制系统的全套"心法"。
