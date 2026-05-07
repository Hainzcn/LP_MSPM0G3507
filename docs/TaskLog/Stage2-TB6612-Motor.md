# TB6612 电机驱动模块日志

> 日期：2026-05-05
> 关联文档：[Stage0-PinAllocation.md](Stage0-PinAllocation.md) §4.2

---

## 新增文件

### template/hardware/bsp_motor.h

TB6612FNG 双路电机驱动头文件。

- 370 电机参数宏：`BSP_MOTOR_RATED_VOLTAGE_MV` (6000)、`BSP_MOTOR_NOLOAD_SPEED_RPM` (620)
- PWM LOAD 常量：`BSP_MOTOR_PWM_LOAD` (1599)，与 SysConfig `PWM_MOTOR.timerCount` 一致
- 电机编号枚举：`bsp_motor_id_t` (LEFT / RIGHT / COUNT)
- 公开 API：
  - `bsp_motor_init()` — 占空比清零 + 滑行态，不拉高 STBY
  - `bsp_motor_set_duty(id, pct)` — -100.0 ~ +100.0，正转/反转
  - `bsp_motor_brake(id)` — 短路刹车
  - `bsp_motor_coast(id)` — 滑行
  - `bsp_motor_standby(enable)` — STBY 控制
  - `bsp_motor_get_duty(id)` — 读取当前占空比

### template/hardware/bsp_motor.c

TB6612FNG 驱动实现。

- 方向引脚控制：4 个静态函数 `set_dir_forward/reverse/coast/brake`
- PWM 占空比：`set_pwm_ccr()` 调用 `DL_TimerA_setCaptureCompareValue`
- CCR 映射：CCR = 0 → 0%，CCR = LOAD → ≈100%
- 占空比限幅：±100.0% 硬限幅，CCR 上限 BSP_MOTOR_PWM_LOAD
- STBY 默认低（由 bsp_gpio_init 设置），须显式调用 `bsp_motor_standby(true)` 退出待机

---

## 硬件映射

| 信号   | 引脚 | 外设              | SysConfig 宏             |
|--------|------|-------------------|--------------------------|
| PWMA   | PA8  | TIMA0_CCP0        | GPIO_PWM_MOTOR_C0_IDX    |
| PWMB   | PA9  | TIMA0_CCP1        | GPIO_PWM_MOTOR_C1_IDX    |
| AIN1   | PA15 | GPIO              | BSP_AIN1_PORT/PIN        |
| AIN2   | PA16 | GPIO              | BSP_AIN2_PORT/PIN        |
| BIN1   | PA26 | GPIO              | BSP_BIN1_PORT/PIN        |
| BIN2   | PA27 | GPIO              | BSP_BIN2_PORT/PIN        |
| STBY   | PB0  | GPIO              | BSP_STBY_PORT/PIN        |

---

## 集成说明

在 `main.c` 中调用顺序：

```c
SYSCFG_DL_init();      /* PWM_MOTOR 由 SysConfig 初始化，CCR = LOAD */
bsp_gpio_init();       /* STBY = LOW，方向引脚 = LOW */
bsp_motor_init();      /* CCR = 0，方向 = 滑行 */
/* ... 其他初始化 ... */
bsp_motor_standby(true);  /* 准备就绪后退出待机 */
bsp_motor_set_duty(BSP_MOTOR_LEFT, 30.0f);   /* 左电机 30% 正转 */
```

---

## 注意事项

1. **CCR 极性**：当前采用 CCR = 0 → 0%、CCR = LOAD → 100% 映射。若实测电机行为反转，
   需在 `set_pwm_ccr()` 中取反（`ccr = BSP_MOTOR_PWM_LOAD - ccr`）。
2. **方向极性**：正转定义为 AIN1=1/AIN2=1(BIN1=1/BIN2=0)。若某侧电机接线相反，
   可在 `set_dir_forward/reverse` 中交换该侧的 set/clear。
3. **STBY 安全**：`bsp_motor_init()` 不拉高 STBY，防止初始化期间电机意外转动。
   应用层须在所有外设就绪后显式调用 `bsp_motor_standby(true)`。
