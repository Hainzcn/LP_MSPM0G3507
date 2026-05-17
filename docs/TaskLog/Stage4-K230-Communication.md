# 阶段 4 ｜ K230 通讯（IMU TX 一分二方案）

> 文档定位：Stage 4 第一步——MCU-K230 通讯链路实现记录。
>
> 关联文档：
> - 项目总览与执行计划：[Overview.md](../Overview/Overview.md)
> - 引脚分配唯一真源：[Stage0-PinAllocation.md](Stage0-PinAllocation.md)
> - 帧协议实现：[`template/middle/k230_protocol.{c,h}`](../../template/middle/k230_protocol.h)
> - BSP 驱动层：[`template/hardware/bsp_k230_uart.{c,h}`](../../template/hardware/bsp_k230_uart.h)
> - 集成入口：[`template/app/app_balance.c`](../../template/app/app_balance.c)

---

## 1. 架构决策：IMU TX 一分二

### 1.1 原方案

```
MS901M TX ─→ MCU UART3 RX (PB13)
                    │
                    ↓ (MCU 帧编码)
            MCU UART1 TX (PB6) ──DMA──→ K230 UART RX
```

MCU 需要：
- 在 100 Hz 控制环中构建 `IMU_TELEM` 帧（pitch、pitch_rate、车速、状态）
- 通过 UART1 TX DMA 推送给 K230
- 占用 2 个 DMA 通道（TX + RX）

### 1.2 新方案（Stage 4 采纳）

```
MS901M TX ──┬──→ MCU UART3 RX (PB13)    [已有, 不变]
            └──→ K230 独立 UART RX        [新增 Y 分线]

MCU UART1 TX (PB6) ──阻塞写──→ K230 命令 UART RX    [速度/状态/心跳, ~240 B/s]
MCU UART1 RX (PB7) ←──DMA────  K230 命令 UART TX    [运动指令, DMA 保留]
```

### 1.3 可行性分析

**电气**：
- MS901M TX 是标准 CMOS 推挽输出，驱动两个高阻 UART RX 仅增加门极电容负载
- 115200 baud 位周期 ~8.7 µs，寄生电容影响可忽略
- 三端均为 3.3 V 电平，直连兼容

**协议**：
- MS901M 是单向广播（200 Hz 主动上报 0x01/0x02/0x03 三帧），两端独立解析互不干扰
- K230 直接获得原始 6 轴数据，延迟低于主控转发，云台前馈精度更高

**资源**：
- 释放 UART1 TX DMA 通道（原 DMA_CH1, 硬件通道 0）
- MCU→K230 数据量降至 ~240 B/s，阻塞写无压力

### 1.4 接线方式

| 信号线 | 起点 | 终点 |
|--------|------|------|
| MS901M TX | MS901M 模块 TX 引脚 | Y 分：MCU PB13 (BP J2.26) + K230 UART RX |
| MS901M RX | MCU PB12 (BP J4.32) | MS901M 模块 RX 引脚（配置/校准命令，不分线） |
| K230 CMD TX | K230 UART TX | MCU PB7 (UART1 RX) |
| K230 CMD RX | MCU PB6 (UART1 TX) | K230 UART RX (命令通道) |
| GND | 全部共地 | 单点星型汇接 |

---

## 2. 帧协议设计

### 2.1 帧格式

```
字节偏移   内容              说明
0          0xAA              帧头高字节
1          0x55              帧头低字节
2          LEN               PAYLOAD 字节数 (0~32)
3          CMD               帧类型标识
4..4+LEN-1 PAYLOAD[LEN]      业务数据
4+LEN      CRC16_LO          CRC16-CCITT 低 8 位
5+LEN      CRC16_HI          CRC16-CCITT 高 8 位
6+LEN      0x55              帧尾低字节
7+LEN      0xAA              帧尾高字节
```

- CRC16-CCITT：多项式 0x1021，初始值 0xFFFF，校验范围 = `LEN + CMD + PAYLOAD`
- 帧尾用于二次确认帧边界

### 2.2 MCU → K230 帧

| CMD | 名称 | 频率 | PAYLOAD 布局 | 大小 |
|-----|------|------|-------------|------|
| 0x01 | VEHICLE_STATUS | 20 Hz | `avg_cps:i32 + safety_state:u8 + bat_mv:u16` | 7 B |
| 0x02 | HEARTBEAT_MCU | 1 Hz | `uptime_ms:u32` | 4 B |

### 2.3 K230 → MCU 帧

| CMD | 名称 | 频率 | PAYLOAD 布局 | 大小 |
|-----|------|------|-------------|------|
| 0x11 | MOTION_CMD | 20~50 Hz | `target_v:i16 + target_omega:i16 + mode:u8` | 5 B |
| 0x12 | HEARTBEAT_K230 | 1 Hz | `uptime_ms:u32` | 4 B |
| 0x13 | PID_INJECT | 按需 | `pid_id:u8 + kp:f32 + ki:f32 + kd:f32` | 13 B |

### 2.4 带宽估算

MCU→K230（20 Hz × 15 B + 1 Hz × 12 B = 312 B/s）：
- 921600 baud 有效吞吐 ~92 kB/s，占用率 < 0.4%
- 阻塞写每字节 ~10.9 µs，15 B 帧 ~164 µs，对 50 ms 速度环周期占比 0.3%

K230→MCU（50 Hz × 13 B + 1 Hz × 12 B = 662 B/s）：
- DMA 接收，不占 CPU

---

## 3. SysConfig 与生成文件变更

### 3.1 LP_MSPM0G3507.syscfg

移除 K230 UART 的 TX DMA 配置：
```diff
- K230_UART.enabledDMATXTriggers = "DL_UART_DMA_INTERRUPT_TX";
- K230_UART.enableDMATX          = true;
```

### 3.2 ti_msp_dl_config.h

- 移除 `DMA_CH1` 相关定义（TX DMA 通道）
- 新增 `DMA_CH_UART_K230_DMA_RX_CHAN` 映射宏，解决之前 fallback 默认值 0 与实际通道 1 不一致的 bug

### 3.3 ti_msp_dl_config.c

- 移除 `DL_UART_Main_enableDMATransmitEvent` 调用
- 移除 `gDMA_CH1Config` 和 `SYSCFG_DL_DMA_CH1_init`
- 修正 `gDMA_CH0Config`：改为 `DL_DMA_WIDTH_BYTE` + `DL_DMA_ADDR_INCREMENT`（dest），匹配 UART RX 字节传输语义

---

## 4. 软件实现

### 4.1 k230_protocol.c/.h（新增）

- CRC16-CCITT 查表法（256 B ROM）
- `k230_encode_frame`：帧编码（head + len + cmd + payload + crc + tail）
- `k230_parser_t` + `k230_parser_feed`：逐字节状态机解析器
- 所有 payload 结构体 `__attribute__((packed))` 保证 MCU/K230 间二进制兼容

### 4.2 bsp_k230_uart.c（重构）

- RX：DMA BLOCK 模式（256 B 缓冲传满中断 → 搬入 512 B 应用环缓 → 重装）
- TX：保留 `bsp_k230_uart_write_blocking`，TX DMA 相关代码全部移除
- 新增 `bsp_k230_uart_rx_pop_bulk` 供上层批量读取
- 修正 DMA 通道号：使用 `DMA_CH_UART_K230_DMA_RX_CHAN`（= DMA_CH0_CHAN_ID = 1）

### 4.3 app_balance.c（集成）

主循环中新增 K230 通讯调度：

| 频率 | 操作 |
|------|------|
| 1 kHz | `k230_drain_and_dispatch`：从环缓取字节 → 喂 parser → 分发已完成帧 |
| 1 kHz | `k230_check_timeout`：500 ms 无帧 → 运动指令归零 |
| 20 Hz | `k230_send_vehicle_status`：编码 + 阻塞写 VEHICLE_STATUS |
| 1 Hz | `k230_send_heartbeat`：编码 + 阻塞写 HEARTBEAT_MCU |

帧分发逻辑：
- `MOTION_CMD` (0x11)：解包后直接覆盖 `cmd.target_speed_cps` / `cmd.target_yaw_pm`
- `PID_INJECT` (0x13)：按 `pid_id` 调用对应 `set_*_gains` API
- `HEARTBEAT_K230` (0x12)：仅刷新最后收帧时间戳

1 Hz 心跳日志新增字段：`k230_g=<good>/b=<bad> k230_<ON|OFF>`

---

## 5. K230 侧准备事项

K230 端不在本工程范围内，但以下是对接所需：

1. **IMU 解析**：移植 `ms901m.c/.h` 到 K230（C 或 MicroPython），接收 115200 UART 数据
2. **命令帧编码**：实现 `0xAA 0x55` 帧格式编码（发送 MOTION_CMD / HEARTBEAT_K230 / PID_INJECT）
3. **状态帧解码**：实现帧解析状态机（接收 VEHICLE_STATUS / HEARTBEAT_MCU）
4. **硬件接线**：K230 需要两路 UART——一路 115200 接 MS901M TX Y 分线，一路 921600 接 MCU UART1

---

## 6. 验证清单

- [ ] SysConfig 重新生成后编译通过（或手工修改 ti_msp_dl_config 已等效）
- [ ] 上电后 1 Hz 心跳日志出现 `k230_g=0/b=0 k230_OFF`（K230 未连接时预期行为）
- [ ] PB6 用 USB-TTL 监听可见 20 Hz VEHICLE_STATUS + 1 Hz HEARTBEAT_MCU 帧
- [ ] PB7 用 USB-TTL 发送构造的 MOTION_CMD 帧 → 车辆响应 v/ω 指令
- [ ] 发送后断开 → 500 ms 后自动归零 + 日志显示 `k230_OFF`
- [ ] PB6/PB7 回环短接 → 帧计数递增（验证编解码一致性）
- [ ] MS901M TX Y 分线后两端同时收到 200 Hz 数据（万用表 / 逻辑分析仪验证）

---

## 7. 变更日志

| 日期 | 版本 | 内容 | 执行方 |
|------|------|------|--------|
| 2026-05-17 | v0.1 | Stage 4 第一步：IMU TX 一分二方案决策 + MCU 侧帧协议全量实现 | 主控团队 |
