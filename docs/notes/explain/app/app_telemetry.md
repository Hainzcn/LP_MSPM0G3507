# APP_TELEMETRY 遥测调度模块 学习笔记

## 📖 一、整体概述

### 1.1 这个模块在项目中的位置

本项目是一个**两轮自平衡瞄准小车**，主控为 **TI MSPM0G3507**。

`app_telemetry` 是阶段 1 的"验证主循环"——在平衡控制还没写好时，让单片机先跑起来，验证硬件（IMU、串口、LED）是否正常。它是整个项目中"第一个能跑的主循环"，是嵌入式开发的 Hello World。

等阶段 2 平衡控制开发好以后，这个文件被 `app_balance_run()` 替代。但主循环结构（tick_count 周期调度 + __WFI 睡眠）被完整继承。

### 1.2 调度策略

```
基于 1 ms SysTick + tick_count 取模调度：

1 kHz（每 tick）：UART3 读数据 → ms901m_feed_bytes → get_snapshot
5 Hz（%200==0）：LED_G 绿灯翻转（心跳指示）
1 Hz（%1000==0）：printf 调试日志
```

---

## 📋 二、函数汇总

| 函数名 | 功能 | 调用方 |
|--------|------|--------|
| `app_telemetry_run()` | 阶段 1 主循环入口（永不返回） | `main()` |

---

## 🔄 三、核心逻辑总结

### 3.1 主循环流程

```
for (;;) {
    if (!consume_tick()) { __WFI(); continue; }  ← 等 1 ms 心跳
    tick_count++;

    1 kHz：取 UART 数据 → 喂 ms901m → 取快照
    5 Hz：翻转绿灯（心跳）
    1 Hz：printf 打印姿态/帧统计/K230 字节数
}
```

### 3.2 定点格式化宏（避免 printf("%f")）

printf("%f") 在 Keil AC6 下会拉入几 KB 浮点格式化代码，且每次调用消耗 200~300 字节栈。MSPM0G3507 栈只有 1~4 KB，多个 %f 直接溢出 → HardFault。

解决方案：浮点数 ×100 → 拆成符号+整数+小数，用整数 printf 打印。

```
pitch=2.35° → F2_S=' ', F2_I=2, F2_F=35 → printf("%c%ld.%02lu") → " 2.35"
pitch=-2.35° → F2_S='-', F2_I=2, F2_F=35 → printf("%c%ld.%02lu") → "-2.35"
99.995 → F2_X100=10000 → F2_I=100, F2_F=0 → " 100.00" ✅（不是" 99.100"）
```

### 3.3 1 Hz 日志字段解读

```
[hb] t=12s pitch= 1.23 roll= 0.05 gy= 3.45 T= 25.30C
     ms901m_good=1200 bad=0 over=0 k230_rx=0b/s
```

| 字段 | 含义 | 异常时的诊断 |
|------|------|------------|
| `t` | 运行秒数 | 如果不动→SysTick 停了 |
| `pitch` | 俯仰角 | 如果全 0→MS901M 没接好 |
| `roll` | 横滚角 | |
| `gy` | Y 轴角速度 | |
| `T` | 传感器温度 | |
| `ms901m_good` | 成功帧数 | 每秒应为 ~200 |
| `ms901m_bad` | 失败帧数 | 如果增长→串口有误码 |
| `over` | UART 缓冲区溢出 | 如果增长→主循环太慢 |
| `k230_rx` | K230 接收速率 | 物理回环测试验证 |

---

## 🧩 四、与 app_balance_run() 的对比

| 特性 | app_telemetry_run() | app_balance_run() |
|------|-------------------|-------------------|
| 用途 | 验证硬件 | 完整平衡控制 |
| 1 kHz | IMU drain + snapshot | IMU drain + motor_update |
| 100 Hz | ❌ | 平衡 step + 电池采样 |
| 5 Hz | 绿灯心跳 | 绿灯心跳 + 安全状态 LED |
| 1 Hz | 姿态日志 | 姿态+速度+PID+电池+编码器 |
| 电机控制 | ❌ | ✅ 级联 PID |

`app_balance_run()` 的主循环结构就是从 `app_telemetry_run()` 继承的：
同样的 `consume_tick` + `tick_count` + 取模调度。

---

> 本文档配合 `app_telemetry.h` 和 `app_telemetry.c` 中的详细注释阅读效果最佳。加油！
