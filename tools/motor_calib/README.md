# Motor Calibration Sweep — 使用说明

## 目录

- [功能简介](#功能简介)
- [环境准备](#环境准备)
- [录制串口日志](#录制串口日志)
- [执行校准扫描](#执行校准扫描)
- [运行分析脚本](#运行分析脚本)
- [输出说明](#输出说明)
- [典型结果解读](#典型结果解读)
- [可调参数](#可调参数)

---

## 功能简介

固件在 `app_motor_demo` 阶段新增 PWM 扫描校准模式：

- UART 命令 `c` 触发，自动关闭同步环
- 正向从 PWM=100‰ 到 1000‰ 以步长 50‰ 逐档驻留 1500 ms
- 反向重复相同过程（负值 PWM）
- 每档稳态（跳过前 500 ms 瞬态）每 100 ms 输出一条 `[cal]` 日志，包含：
  - 当前 PWM 档位、电池电压 `vbat`、左右轮转速 `rpmL` / `rpmR`、累计编码器计数
- 扫描完成后自动恢复之前的同步配置与目标转速

离线 Python 脚本 `analyze_calib.py` 从日志文件中：

1. 提取稳态样本并按档位聚合（均值 / 标准差）
2. 对正向做 PWM→RPM 线性/二次拟合（左/右各一组）
3. 比较两个误差模型：
   - **M1**：`err = a·pm + b`（仅绕组不对称）
   - **M2**：`err = a·pm + c·(V_nom − Vbat) + b`（叠加电压塌陷）
4. 输出 4 张 PNG 图表

---

## 环境准备

```bash
cd tools/motor_calib
pip install -r requirements.txt
```

Python 3.9+ 即可。

---

## 录制串口日志

### 方案 A：serial_capture.py（推荐）

本目录内置录制脚本，打开串口后**自动等待 5 秒**（让固件完成启动），再依次发送 `r`（启动电机）和 `c`（开始校准），并在检测到 `[cal] calibration complete` 后自动停止。

```bash
# 安装依赖（仅需一次）
pip install -r requirements.txt

# 列出可用串口
python serial_capture.py --list

# 标准用法：5s 后自动发 r/c，校准完成后自动停止
python serial_capture.py --port COM3

# 指定输出文件名
python serial_capture.py --port COM3 --out my_run.txt

# 调整等待时间（例如改为 8 秒）
python serial_capture.py --port COM3 --cmd-delay 8

# 不自动发命令（手动在串口终端输入 r/c）
python serial_capture.py --port COM3 --no-auto-cmd

# 手动 Ctrl+C 停止（跑普通 demo 日志时用）
python serial_capture.py --port COM3 --no-auto-stop

# 无数据超时（默认 30s，可调整）
python serial_capture.py --port COM3 --idle-timeout 60
```

典型运行输出：
```
[capture] 打开串口 COM3  波特率 115200
[capture] 保存至   C:\...\calib_20260510_182345.txt
[capture] 5s 后自动发送 'r' + 'c'
[capture] 检测到 '[cal] calibration complete' 后自动停止

[boot] stage2 motor demo start
...
[capture] >>> 发送命令: 'r  (run motors)'
[ctrl] run target=620rpm pwm=1000/1000
[capture] >>> 发送命令: 'c  (start calib sweep)'
[cal] start dir=+1 steps=19 pm_start=100 ...
...
[cal] calibration complete

[capture] 检测到结束标志，停止录制。
[capture] 共录制 312 行（含 270 条 [cal] 行），耗时 62.3s
[capture] 文件：C:\...\calib_20260510_182345.txt
```

### 方案 B：MobaXterm / PuTTY

1. 打开串口（COM？），波特率 115200，8N1
2. 在 MobaXterm 中开启 **Log to file**，或 PuTTY → Session → Logging → All session output → 指定文件路径
3. 保持录制直到 `[cal] calibration complete` 出现

### 方案 B：PowerShell（无需额外工具）

```powershell
$port = [System.IO.Ports.SerialPort]::new("COM3", 115200, "None", 8, "One")
$port.Open()
$out  = "C:\logs\calib_run.txt"
while ($true) {
    $line = $port.ReadLine()
    Add-Content -Path $out -Value $line
    Write-Host $line
    if ($line -match "\[cal\] calibration complete") { break }
}
$port.Close()
```

将 `COM3` 改为实际端口号，`$out` 改为保存路径。

### 方案 C：Python pyserial 实时录制

```python
import serial, time, pathlib

LOG = pathlib.Path("calib_run.txt")
with serial.Serial("COM3", 115200, timeout=1) as ser, LOG.open("w") as f:
    while True:
        line = ser.readline().decode("utf-8", errors="replace")
        print(line, end="")
        f.write(line)
        if "[cal] calibration complete" in line:
            break
```

---

## 执行校准扫描

1. 烧录固件并上电，等待 boot log 出现
2. 打开串口终端并**开启日志录制**（见上节）
3. 发送 `r` + Enter → 两轮启动
4. 发送 `c` + Enter → 开始校准扫描（约 57 秒）
5. 等待 `[cal] calibration complete` 出现
6. 关闭日志录制文件

> **中途中止**：发送 `x` 或按 S1 按键 → 立即 brake 并恢复 demo 状态

---

## 运行分析脚本

```bash
python analyze_calib.py path/to/calib_run.txt
```

指定输出目录：

```bash
python analyze_calib.py calib_run.txt --out-dir results/
```

使用非标准额定电压（默认 11100 mV = 3S 锂电标称 3.7V×3）：

```bash
python analyze_calib.py calib_run.txt --v-nominal 11100
```

---

## 输出说明

### 控制台

```
Parsing calib_run.txt ...
  273 stable samples found after settle filtering

── Aggregated per-step statistics (38 steps) ──────────...
 dir  idx     pm    n  rpmL_mean  rpmL_std  rpmR_mean  rpmR_std   vbat_mean   err_mean  err_std
...
── Forward PWM → RPM polynomial fits ──────────────────
  [Left  linear ] linear: +0.2412*pm^1 -5.3210  R²=0.9987
  ...

── Error model fitting ─────────────────────────────────
  M1 (winding only):  err = +0.0123*pm +2.41   RMSE=3.214 rpm
  M2 (winding+vbat):  err = +0.0119*pm +0.0009*(V_nom-vbat) +1.87  RMSE=2.103 rpm
  Conclusion: Voltage drop is a significant factor (M2 improves RMSE by 34.6%)

── Generating charts ────────────────────────────────────
  Saved results/fig1_pwm_rpm.png
  ...
Done.
```

### 图表文件

| 文件 | 内容 |
|---|---|
| `fig1_pwm_rpm.png` | 正向 PWM→RPM 散点 + 线性/二次拟合线（左/右两色） |
| `fig2_err_vs_pwm.png` | RPM 误差 vs PWM 散点，颜色映射电池电压 |
| `fig3_err_vs_vbat.png` | RPM 误差 vs 电池电压散点，颜色映射 PWM |
| `fig4_residuals.png` | M1/M2 残差直方图（评估模型拟合质量） |

---

## 典型结果解读

| 现象 | 可能根因 | 建议 |
|---|---|---|
| M1 RMSE 小，M2 改善 < 5% | 绕组不对称主导 | 在同步环中加固定前馈偏置 |
| M2 改善 > 20% | 电压塌陷主导 | 提高电源容量 / 降低满载 PWM 上限 |
| 两者 RMSE 均 > 15 rpm | 编码器噪声 / 转速抖动本底较高 | 检查编码器信号质量，考虑增大 SPEED_WINDOW_MS |
| fig1 曲线出现死区（RPM ≈ 0 at low PWM） | 电机摩擦死区 | 将 `APP_MOTOR_CAL_PWM_START_PM` 降低至 50 重跑 |
| vbat 全部 < 1000 mV | 未接电池 | 接电池后重录；此时 M2 电压分析无意义 |

---

## 可调参数

在 `template/app/app_motor_demo.c` 顶部可通过宏覆盖默认值（不改源码时也可在编译命令行 `-D` 传入）：

| 宏 | 默认值 | 说明 |
|---|---|---|
| `APP_MOTOR_CAL_PWM_START_PM` | 100 | 起始 PWM（‰） |
| `APP_MOTOR_CAL_PWM_END_PM` | 1000 | 终止 PWM（‰） |
| `APP_MOTOR_CAL_PWM_STEP_PM` | 50 | 步长（‰） |
| `APP_MOTOR_CAL_DWELL_MS` | 1500 | 每档驻留时间（ms） |
| `APP_MOTOR_CAL_SAMPLE_PERIOD_MS` | 100 | 稳态采样周期（ms） |
| `APP_MOTOR_CAL_SETTLE_MS` | 500 | 进档后稳定等待时间（ms） |
