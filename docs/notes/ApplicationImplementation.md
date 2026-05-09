# 应用层 (App) 代码实现路径详解

在这个项目的最高层，应用层的代码不再纠结于如何把引脚拉高、或者如何写一个微分公式，而是**做决策**和**做调度**。这部分重点剖析安全状态机 (`app_safety.c`) 和平衡大脑的主控制流 (`app_balance.c`) 的代码落地细节。

---

## 1. 安全管家的优先级决策逻辑 (`app_safety.c`)

在 `app_safety_tick` 函数中，安全管家需要在每一次控制节拍（100Hz）时，综合各方信息决定当前小车处于什么安全级别。这里的代码写得非常有层次感。

### 1.1 动作拦截与“按需操作硬件” (`transition` 函数)
改变状态不仅是改个变量名，还需要让电机做出物理反应（比如急刹）。
为了防止每 10 毫秒都重复给电机发一次“急刹”指令（这会干扰正常的底层逻辑），代码里写了一个 `transition(app_safety_state_t next)` 包装函数：
```c
if (next == s_state) { return; } // 如果状态没变，直接退出
s_state = next;
switch (next) {
    case APP_SAFETY_FALLEN:
        hw_emergency(APP_SAFETY_FALL_BRAKE_MS); // 物理刹车并断电
        break;
    // ...
}
```
这保证了硬件动作只在**状态发生跳变的那一瞬间**执行一次。

### 1.2 多事件优先级判定 (if-else 瀑布流)
小车可能同时面临多个情况：比如它现在是解除武装状态（DISARMED），同时电池又没电了（LOW_STOP），同时还倒在地板上（FALLEN）。听谁的？
代码通过一个精心设计的 `if - else if` 瀑布流解决了优先级冲突：

1. **最高优先级：电池致命低压 (`LOW_STOP`)**
   只要电压触底，不管小车在干嘛，立刻强制进入 `LOW_BAT_STOP`。
2. **状态回滞保护**
   如果上一拍是 `LOW_STOP`，但电压稍微弹回了一点（到了 `WARN` 级别），不能马上让它去跑（防止电压在临界点抖动导致车子抽搐），而是把它降级到 `LOW_BAT_WARN` 待命，等人工确认安全后再按键启动。
3. **人工待机保护 (`DISARMED`)**
   如果在人工待机时摔倒了，系统会**忽略摔倒事件**（因为车停在地上本来就是倒着的，没必要报警）。
4. **跌倒保护 (`FALLEN`)**
   正常行驶时，一旦计算出的 `pitch`（俯仰角绝对值）超过了 `APP_SAFETY_FALL_PITCH_DEG`（例如 60 度），立刻判定跌倒，切断动力。

---

## 2. 平衡大脑的调度与串级执行 (`app_balance.c`)

这里是单片机算力的集中消耗地，它需要精准的时间控制。

### 2.1 串级 PID 的代码连接 (`app_balance_step`)
我们之前说过“串级 PID”，在代码里它是怎么“串”起来的呢？

```c
// 1. 拿数据：取左右轮的平均速度
int32_t avg_cps = (fb.left_speed_cps + fb.right_speed_cps) / 2;

// 2. 算外环（速度环）：目标速度 vs 真实速度，算出一个“目标倾角”
float target_tilt_deg = pid_step(&s_bal.speed_pid, 
                                 (float)cmd->target_speed_cps, 
                                 (float)avg_cps, 
                                 s_dt_sec);

// 3. 算内环（平衡环）：目标倾角 vs 真实倾角，算出一个“电机总输出(PWM)”
float pitch_meas = att->pitch_deg - s_bal.pitch_offset_deg;
float pwm_out = pid_step(&s_bal.balance_pid, 
                         target_tilt_deg, // <--- 这里就是串接点！
                         pitch_meas, 
                         s_dt_sec);

// 4. 差速转向：左轮减一点，右轮加一点
int16_t left_pm  = clamp_pwm_pm(pwm_out - yaw_pm);
int16_t right_pm = clamp_pwm_pm(pwm_out + yaw_pm);

// 5. 下发物理指令给电机
bsp_motor_set_output(left_pm, right_pm);
```
上面这短短的几行代码，清晰地展示了外环的**输出**（`target_tilt_deg`）直接变成了内环的**目标输入**。这就像接力赛一样，一棒传一棒。

### 2.2 轻量化日志输出（字符串拼接技巧）
在 `app_balance_run` 的 1Hz 日志打印里，有一段看起来很复杂的 `printf`：
```c
BAL_F2_S(diag.pitch_meas_deg), 
(long)BAL_F2_I(diag.pitch_meas_deg), 
(unsigned long)BAL_F2_F(diag.pitch_meas_deg)
```
- **为什么不直接用 `%f` 打印浮点数？**
  在单片机上（特别是 M0+ 内核），C 语言标准库的 `printf("%f")` 会消耗巨大的栈内存（动辄几百字节），极易导致栈溢出（HardFault 崩溃），而且会引入庞大的浮点格式化库代码。
- **怎么解决的？**
  作者在文件顶部写了几个宏（`BAL_F2_X100`, `BAL_F2_I` 等）。原理是：把浮点数先乘以 100 变成整数（比如 `15.34` 变成 `1534`），然后用除法 `/ 100` 取整数部分（`15`），用取余 `% 100` 取小数部分（`34`）。最后在 `printf` 里用 `%d.%02d` 的形式把它们当做普通整数拼起来。
  这属于嵌入式开发里极为经典的**“去浮点化”**内存优化技巧。

### 2.3 `main.c` 里的“启动保险”
如果在代码刚烧录进去、车还没调好时，开机直接进平衡模式是非常危险的。
所以在 `main.c` 里有这样一段逻辑：
1. 所有的 PID 参数 (`Kp`, `Ki`, `Kd`) 在初始化时**全部被默认设为了 0**。这意味着即使进了平衡循环，车子也是软绵绵的，不会发疯。
2. 只有你在测试完电机 (`app_motor_demo_run`) 之后，主动调用了 `app_balance_set_balance_gains(...)` 给定参数，车子才会真正获得站立的力量。

---

**总结：**
应用层的代码充满了“工程智慧”。它不仅仅是跑通算法，更多的是在考虑：
- 如何不让 CPU 被日志打印卡死（去浮点化）。
- 如何让物理动作在状态切换时不冗余执行（状态回滞和拦截）。
- 如何处理错综复杂的极端情况（安全状态机瀑布流）。
读懂了这层代码，你基本上就掌握了编写一个稳定可靠的小型机器人控制系统的全套“心法”。