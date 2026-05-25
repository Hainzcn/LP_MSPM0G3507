# 平衡车 PID 调参指南（两级串级 + 航向角环 + 差速环）

本文面向当前控制架构：**速度 → 角度 → PWM** 两级串级 + **航向角环 → 差速环 → PWM** 三级横摆链。核心代码在 `template/app/app_balance.c`（`pid2_t`），底层电机在 `template/hardware/bsp_motor.c`。

> 历史说明：Stage 3.4 曾采用四级级联（含独立角速度内环 `rp`），Stage 3.7 收敛为两级串级 + `pid2_t`，Stage 3.8 引入差速闭环将航向环输出级联到差速环。演进记录见 [Stage3-BalanceControl.md](../TaskLog/Stage3-BalanceControl.md)。

## 1. 控制结构概览

装车模式由 `main.c` 上电默认启动。串口 `t` / `test` 进入电机演示，`l` / `load` 返回。

```
速度环 (20 Hz) ─→ 角度环 (100 Hz) ─→ 平均 PWM ─→ 电机
航向角环 (20 Hz) ─→ 差速环 (20 Hz) ─→ 差分 PWM ─→ 叠加左右
```

| 环路 | 频率 | 输入 | 输出 | 串口命令 |
|------|------|------|------|----------|
| 角度环 | 100 Hz | 目标倾角 − 实测 pitch (°) | PWM permille | `bp` |
| 速度环 | 20 Hz | 目标 − 实测速度（归一化 cps） | 目标倾角 (°) | `sp` |
| 航向角环 | 20 Hz | 偏航误差（见 `YAW_SOURCE`） | target_dif（归一化 cps → 差速环目标） | `yp` |
| 差速环 | 20 Hz | target_dif − 实测差速（RPM） | 差分 PWM permille | `dp` |

**PID 库**：四路控制器均使用 `pid2_update()`（对齐 STM32 `PID_Update`）：积分按拍累加（Ki=0 时自动清零）、**微分先行**（D 作用于测量值变化）、**OutOffset** 死区补偿。

**电机死区**：平衡模式内关闭 BSP 死区重映射，改由角度环 `OutOffset`（`bo` / `bp` 第 4 参数）突破 TB6612 静摩擦。默认编译宏 `APP_BALANCE_ANGLE_OUT_OFFSET=80`（‰）。

## 2. 调参前检查

### 硬件方向
1. 串口 `t` 进入 demo，`r` 启动、`b` 急刹、`+`/`-` 调速。
2. 同向运行时 `rpmL` 与 `rpmR` 应同号。

### 姿态方向
- `APP_BALANCE_PITCH_INVERT`：MS901M 前后装反时软件翻转（当前默认 0）。
- `APP_BALANCE_PITCH_OFFSET_DEFAULT_DEG=-2.5`：根据实车重心预先测量的静态偏角。
- `APP_BALANCE_PITCH_AUTOZERO_ENABLE=0`：**默认关闭上电自校零**（调试期直接使用硬编码 offset，零等待启动）。
- 手扶车体前倾时，心跳 `pitch=` 应为正值。方向反了先改宏，不要动 PID。

### 航向极性
- 串口 `yi?` 查询；`yi0` / `yi1` 切换 yaw/gz 符号。
- 顺时针轻推车体，`yawCorr` 应先正后衰减；若持续同向增大 → 设 `yi1`。

### 安全
- `app_safety` 跌倒 / 低压会切断输出。
- 上电静默窗口 2500 ms，期间不输出 PID。
- 初期可将 `APP_BALANCE_MAX_PWM_PERMILLE` 临时降到 300~500。

## 3. 串口命令格式

增益 Kp/Ki/Kd 以 **×1000 定点整数** 输入；**OutOffset 为直接 permille 整数**（不 ×1000）。

```text
bo <offset_pm>                              # 仅改角度环 OutOffset
bp <kp_x1000> <ki_x1000> <kd_x1000> <offset_pm>   # 角度环 (100 Hz)
sp <kp_x1000> <ki_x1000> <kd_x1000> <offset_pm>   # 速度环 (20 Hz)
yp <kp_x1000> <ki_x1000> <kd_x1000> <offset_pm>   # 航向角环 (20 Hz)
dp <kp_x1000> <ki_x1000> <kd_x1000> <offset_pm>   # 差速环 (20 Hz) ★新增
pid?                                        # 查询四路参数
pid0                                        # 清零 + reset 状态
si0 / si1                                   # 速度反馈极性翻转
yi0 / yi1                                   # 航向反馈极性翻转
c / circle [diam_mm] [v_mm_s]              # 圆弧运动 ★新增
cx                                          # 取消圆弧运动 ★新增
h                                           # 帮助
```

示例：

```text
bo 80
bp 5000 0 5000 80      # Kp=5.0 Ki=0 Kd=5.0 offset=80‰
sp 2000 50 0 0         # Kp=2.0 Ki=0.05
yp 800 0 200 0         # Kp=0.8 Kd=0.2
dp 6000 2000 0 0       # Kp=6.0 Ki=2.0（差速环 RPM 量纲）
```

K230 远程注入（CMD 0x13）：`pid_id` = 0 角度 / 2 速度 / 3 航向。

## 4. 推荐调参顺序

**原则：先能站，再消漂，再锁航向，最后调差速。每次只改一个参数。**

### 4.1 第零步：OutOffset (`bo`)

```text
pid0
bo 80
```

手扶倾倒，确认电机有跟随力矩；若无反应逐步加到 60~100‰。

### 4.2 第一步：角度环 (`bp`，100 Hz)

角度环 **直接输出 PWM**，含 Kd 微分先行。

```text
bp 5000 0 5000 80
```

| 参数 | 参考范围 | 起步值 | 说明 |
|------|----------|--------|------|
| Kp | 3 ~ 15 | **5.0** | 倾角误差 (°) → PWM (‰) |
| Ki | 0 ~ 0.5 | **0** | 初期不加；稳态偏可用速度环 I 补偿 |
| Kd | 2 ~ 8 | **5.0** | 对 pitch 变化率阻尼，抑制振荡 |
| offset | 60 ~ 100 | **80** | TB6612 死区补偿 |

判断：手扶能追倾角；松手能站 3~5 s。

### 4.3 第二步：速度外环 (`sp`，20 Hz)

```text
sp 2000 50 0 0
```

速度测量经 `APP_BALANCE_SPEED_LPF_ALPHA=0.25` EMA 低通；**目标**经 `SPEED_TARGET_LPF_ALPHA=0.10` 平滑（防阶跃引发顿挫）；输入/目标均除以 `SPEED_CPS_SCALE=10` 归一化。

| 参数 | 参考范围 | 起步值 | 说明 |
|------|----------|--------|------|
| Kp | 0.5 ~ 5 | **2.0** | 归一化速度误差 → 目标倾角 (°) |
| Ki | 0 ~ 0.2 | **0.05** | 消除慢漂 |
| Kd | 0 | **0** | 编码器噪声大，一般不用 |
| offset | 0 | **0** | 速度环输出为角度，通常 0 |

输出限幅 ±`APP_BALANCE_MAX_TILT_DEG`（默认 10°）。

### 4.4 第三步：航向角环 (`yp`，20 Hz)

```text
yp 800 0 200 0
```

数据源由 `APP_BALANCE_YAW_SOURCE` 选择：

| 值 | 模式 | 适用 |
|----|------|------|
| 1（默认） | 陀螺 `gz` 积分，目标=0 | 无磁干扰、直行锁向 |
| 0 | EKF `yaw_deg` + virtual measured | 需锁绝对航向（注意磁环境） |

**输出**：航向环输出不再是直接 PWM，而是**差速环目标**（归一化 cps），由差速环做速度闭环后再叠加到 PWM。限幅 ±`APP_BALANCE_YAW_MAX_DIF_CPS`（默认 500）。

| 参数 | 参考范围 | 起步值 | 说明 |
|------|----------|--------|------|
| Kp | 0.3 ~ 3.0 | **0.8** | 偏航误差 → 差速目标 (归一化 cps) |
| Ki | 0 ~ 0.5 | **0** | 一般不需要 |
| Kd | 0.05 ~ 0.5 | **0.2** | 偏航角速率阻尼 |
| offset | 0 | **0** | 航向环通常 0 |

### 4.5 第四步：差速环 (`dp`，20 Hz) ★新增

```text
dp 6000 2000 0 0
```

差速环以 **RPM** 为量纲（输入先由归一化 cps × `DIFF_NORM_TO_RPM` 转换）。反馈经 `APP_BALANCE_DIFF_LPF_ALPHA=0.3` EMA 滤波；**目标**经 `DIFF_TARGET_LPF_ALPHA=0.20` 平滑。

| 参数 | 参考范围 | 起步值 | 说明 |
|------|----------|--------|------|
| Kp | 4 ~ 8 | **6.0** | RPM 误差 → 差分 PWM (‰) |
| Ki | 1 ~ 3 | **2.0** | RPM 量纲下需要积分解耦 |
| Kd | 0 | **0** | 角速度已有航向环 D 项 |
| offset | 0 ~ 30 | **0** | 差速方向一般不需 offset |

输出限幅 ±`APP_BALANCE_DIFF_MAX_PWM_PM`（默认 600‰）。

## 5. 参考参数一览

```text
bo 80
bp 5000 0 5000 80
sp 2000 50 0 0
yp 800 0 200 0
dp 6000 2000 0 0
```

| 环路 | Kp | Ki | Kd | offset |
|------|-----|-----|-----|--------|
| 角度 (`bp`) | 5.0 | 0 | 5.0 | 80‰ |
| 速度 (`sp`) | 2.0 | 0.05 | 0 | 0 |
| 航向 (`yp`) | 0.8 | 0 | 0.2 | 0 |
| 差速 (`dp`) | 6.0 | 2.0 | 0 | 0 |

## 6. 常见现象对照

| 现象 | 优先检查 | 处理建议 |
|------|----------|----------|
| 前倾时车后退 | pitch 翻转、电机极性 | 修方向，不调 PID |
| 电机无跟随 / 突然冲出 | OutOffset 过小 | 加大 `bo` |
| 高频抖动 | 角度 Kp/Kd 过大 | 降 `bp` Kp 或 Kd |
| 低频前后晃 | 速度环 Kp 过大 | 降 `sp` Kp |
| 能站但慢漂 | pitch offset、速度 I | 设 `PITCH_OFFSET_DEFAULT_DEG`；微调 `sp` Ki |
| 直行绕圈 | 航向环、`yi` 极性 | 查 `yawErr`/`yawCorr`；开 `yp` |
| 航向环一开就抖 | `yp` Kp 过大、yi 反 | 降 Kp；查极性 |
| 原地旋转不受控 | 差速环未调、dp Ki=0 | 开 `dp`；加 Ki |
| 跌倒后不驱动 | safety 状态 | 重新 arm |
| 圆弧运动不完整 | `dp` Kp 不足 | 增大 `dp` Kp / Ki |

## 7. 圆弧运动演示 (`circle`) ★新增

串口 `c` 或 `circle` 启动整圆运动：

```text
c                                # 默认：500mm 直径、100mm/s 倒退、顺时针
circle 800 200                   # 自定义：800mm 直径、200mm/s 前进
cx                               # 中止
```

三重判停：① IMU 偏航 ≥ 360°（主） ② 编码器弧长 ≥ 周长×1.2（备） ③ 超时兜底。

圆弧运动期间会覆盖 `target_speed_cps`/`target_dif_cps`，差速环需先调好。

## 8. 调参记录模板

```text
日期/电池电压：
Pitch invert：    Yaw invert：    Speed invert：
Yaw source：1（陀螺）/ 0（EKF）
Angle (bp)：Kp=  Ki=  Kd=  offset=
Speed (sp)：Kp=  Ki=  Kd=  offset=
Yaw   (yp)：Kp=  Ki=  Kd=  offset=
Diff  (dp)：Kp=  Ki=  Kd=  offset=
现象：
下一步：
```

## 9. 策略总结

1. `pid0`，demo 模式确认电机与编码器方向。
2. `bo` + `bp`：先站稳（角度 P+D + OutOffset）。
3. `sp`：消前后漂。
4. `yp` + `dp`：锁直行航向 + 差速闭环执行。
5. 心跳字段：`pitch` / `tilt*` / `pwm` / `yawErr` / `yawCorr` / `difTgt` / `difOut` / `v`。

先方向正确、能站住，再追求走得直、站得久。
