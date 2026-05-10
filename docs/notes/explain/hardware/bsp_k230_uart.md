# BSP_K230_UART K230 通讯串口模块 学习笔记

## 📖 一、整体概述

### 1.1 这个模块在项目中的位置

本项目是一个**两轮自平衡瞄准小车**，主控为 **TI MSPM0G3507**，配合 **K230 (CanMV) 视觉处理器**。

`bsp_k230_uart` 是两个芯片之间的"通讯管道"——MSPM0G3507 通过 UART1 + DMA 接收 K230 发来的运动指令。

```
┌──────────────────────────────────────────────────────────┐
│                    K230（视觉处理器）                      │
│  发送运动指令(v, ω)、接收 IMU 数据                        │
└────────────────────┬─────────────────────────────────────┘
                     │ UART1 TX → MSPM0 RX
                     ▼
┌──────────────────────────────────────────────────────────┐
│                 bsp_k230_uart.c/h（本模块）               │
│                                                          │
│  DMA 自动搬运 UART1 RX 数据到缓冲区（512 字节块）         │
│  装满一块 → DMA 中断 → 累计计数 + 重新装填               │
└────────────────────┬─────────────────────────────────────┘
                     │ 经平衡控制算法处理后
                     ▼
┌──────────────────────────────────────────────────────────┐
│                    app_balance.c                          │
│  根据 K230 的指令控制电机                                 │
└──────────────────────────────────────────────────────────┘
```

### 1.2 本工程中的三个 UART

| 模块 | 硬件 | 用途 | 接收方式 |
|------|------|------|---------|
| **bsp_k230_uart** | UART1 | 与 K230 视觉处理器通讯 | **DMA** |
| bsp_imu_uart | UART3 | 接收 MS901M 姿态数据 | RX FIFO 中断 |
| bsp_log_uart | UART0 (XDS110) | printf 调试日志 | 轮询 |

### 1.3 DMA vs 中断对比

| 特性 | DMA（本模块） | RX FIFO 中断（bsp_imu_uart） |
|------|-------------|---------------------------|
| CPU 参与度 | 配置后零参与 | 每字节都要进一次 ISR |
| 中断频率 | 每 512 字节一次 | 每 1 字节一次（或半满） |
| 数据搬运 | DMA 硬件自动 | CPU 在 ISR 中手动搬运 |
| 适合场景 | 大流量、持续接收 | 小流量、需要实时解析 |
| 复杂度 | 较高（DMA 配置复杂） | 较低（FIFO 中断简单） |

### 1.4 文件结构

| 文件 | 行数 | 作用 |
|------|------|------|
| `bsp_k230_uart.h` | 129 | 4 个函数声明 + DMA 概念说明 |
| `bsp_k230_uart.c` | 316 | DMA 配置 + 中断处理 + 环形缓冲实现 |

---

## 📋 二、函数/变量汇总

### 2.1 静态变量

| 变量名 | 类型 | 修饰 | 初始值 | 含义 | 谁修改 |
|--------|------|------|--------|------|--------|
| `s_rx_buf[512]` | `uint8_t[]` | `volatile` | 全 0 | DMA 接收缓冲区 | DMA 硬件 |
| `s_total_rx` | `uint32_t` | `volatile` | 0 | DMA 累计搬运字节数 | ISR |

### 2.2 函数汇总

| 函数名 | 功能 | 何时调用 |
|--------|------|---------|
| `bsp_k230_uart_init()` | 初始化 UART1 + DMA RX | main 初始化一次 |
| `bsp_k230_uart_total_rx()` | 返回累计接收字节数 | 1 Hz 统计 |
| `bsp_k230_uart_peek(idx)` | 读取缓冲区指定字节 | 调试/自测 |
| `bsp_k230_uart_write_blocking()` | 阻塞式发送到 K230 | 回环测试 |
| `k230_dma_rx_arm()` | 装填 DMA 通道（内部函数） | init + 每次中断后 |
| `DMA_IRQHandler()` | DMA 中断服务函数 | 每装满 512 字节 |

---

## 🔄 三、核心逻辑总结

### 3.1 DMA 工作流程

```
bsp_k230_uart_init()
  └─ k230_dma_rx_arm()
       ├─ disableChannel()       ← 关 DMA 通道以配置
       ├─ setSrcAddr(RXDATA)     ← 源：UART 寄存器（不自增）
       ├─ setDestAddr(buf)       ← 目标：内存数组（自增）
       ├─ setTransferSize(512)   ← 搬 512 个字节
       └─ enableChannel()        ← 开启！DMA 开始自动工作

DMA 自动运行（CPU 不参与）：
  ┌─ 收到字节 1 → 从 RXDATA 搬到 buf[0]
  ├─ 收到字节 2 → 从 RXDATA 搬到 buf[1]
  ├─ ...
  └─ 收到字节 512 → 触发 DMA_IRQHandler

DMA_IRQHandler：
  ├─ s_total_rx += 512        ← 累计字节数
  ├─ k230_dma_rx_arm()        ← 重新装填（从头开始覆盖）
  └─ 返回主循环

1 Hz 日志：
  total_now = bsp_k230_uart_total_rx()
  bytes_in_1s = total_now - total_prev
  printf("K230 RX: %lu B/s\n", bytes_in_1s)
```

### 3.2 中断标识（IIDX）分发

```
DMA 的 16 个通道共享同一个 NVIC 入口：
  DMA_IRQHandler
    ├─ 读 IIDX → 判断是哪个通道
    ├─ 通道 N：处理通道 N 的事件
    └─ 通道 M：处理通道 M 的事件

本模块只用了 K230 RX 一个通道。
如果后续启用 TX 或其他 DMA 通道，
需要在 ISR 中追加对应的 if 分支。
```

### 3.3 累计计数 vs 实际字节数的偏差

```
DMA 配置：传输 512 字节后触发中断

时间线：
  T0: 装填 DMA，开始接收
  T1: 收到 100 字节 → s_total_rx = 0（还没到 512，没触发中断）
  T2: 收到 512 字节 → 触发中断 → s_total_rx = 512
  T3: 收到 600 字节 → 触发中断 → s_total_rx = 1024（512+512）

在 T1 时刻，实际收到 100 字节，但 s_total_rx = 0。
偏差最大 511 字节。对于 1 Hz 统计，偏差 < 1 字节/秒。
```

---

## 🧩 四、关键代码段详解

### 4.1 k230_dma_rx_arm() —— DMA 装填

```c
DL_DMA_setSrcAddr(DMA, chan, (uint32_t)&UART_K230_INST->RXDATA);
DL_DMA_setDestAddr(DMA, chan, (uint32_t)&s_rx_buf[0]);
DL_DMA_setTransferSize(DMA, chan, K230_RX_BUF_SIZE);
```

**源地址不递增**：因为每次都是从同一个 UART 数据寄存器读取。
**目的地址递增**：因为要写入 buf[0]、buf[1]…buf[511]。
**传输大小 512**：搬完触发中断，然后重新装填覆盖旧数据。

### 4.2 DMA_IRQHandler() —— 中断分发

```c
DL_DMA_EVENT_IIDX iidx = DL_DMA_getPendingInterrupt(DMA);
if (iidx == (DL_DMA_EVENT_IIDX_DMACH0 + DMA_CH_UART_K230_DMA_RX_CHAN)) {
    s_total_rx += K230_RX_BUF_SIZE;
    k230_dma_rx_arm();
}
```

`DL_DMA_getPendingInterrupt()` 同时做两件事：
1. 返回当前优先级最高的待处理中断事件
2. 自动清除该事件的中断标志

`DL_DMA_EVENT_IIDX_DMACH0` 是"通道 0 传输完成"的枚举值（数值 = 0）。
加上通道号（如通道 1）就得到"通道 1 传输完成"（数值 = 1）。

### 4.3 bsp_k230_uart_peek() —— 环形缓冲区访问

```c
return s_rx_buf[abs_index % K230_RX_BUF_SIZE];
```

`% 512` 实现环回：索引 0~511 映射到 buf[0]~buf[511]，索引 512 映射到 buf[0]（折回）。

**使用限制**：只能访问最近一轮写入的 512 个字节。如果 DMA 已经重新装填并覆盖了旧数据，再读旧索引就会得到新数据。

### 4.4 为什么 TX 不走 DMA？

本阶段 TX 只需要发送少量数据（每秒几十字节），轮询方式足够。后续阶段如果需要向 K230 发送大量 IMU 数据，可以新增 DMA TX 通道。

---

## 🐛 五、常见踩坑

| 现象 | 可能原因 | 解决方法 |
|------|---------|---------|
| s_total_rx 一直为 0 | DMA 中断没有使能 | 确认 `NVIC_EnableIRQ(DMA_INT_IRQn)` 已调用 |
| 编译报错 DMA_CH_xx 未定义 | SysConfig 中关闭了 UART1 RX DMA | 检查 SysConfig，确认 RX DMA 已勾选 |
| peek() 读到乱码 | 索引落在上一轮旧数据区域 | 确保索引在 [total-512, total) 区间 |
| write_blocking() 卡死 | 对端 K230 没上电，TX 发不出去 | 检查硬件连接和 K230 电源 |
| DMA 中断过于频繁 | 缓冲区太小 | 增大 K230_RX_BUF_SIZE（如 1024） |

---

> 本文档配合 `bsp_k230_uart.h` 和 `bsp_k230_uart.c` 中的详细注释阅读效果最佳。加油！
