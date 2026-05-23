# 阶段 4 ｜ K230 通讯 — MCU 侧实现记录

> **文档定位**：MCU 团队 TaskLog，记录 SysConfig 变更、软件集成与联调历史。
>
> **协议与 K230 协同真源**（帧格式、Python 参考实现、远程调试）：[Stage4-K230-Side.md](Stage4-K230-Side.md)
>
> **关联文档**：
> - 引脚分配：[Stage0-PinAllocation.md](Stage0-PinAllocation.md)
> - 帧协议 C 实现：[`template/middle/k230_protocol.{c,h}`](../../template/middle/k230_protocol.h)
> - BSP 驱动：[`template/hardware/bsp_k230_uart.{c,h}`](../../template/hardware/bsp_k230_uart.h)
> - 集成入口：[`template/app/app_balance.c`](../../template/app/app_balance.c)

---

## 1. 架构决策：IMU TX 一分二

### 1.1 原方案 vs 新方案

原方案 MCU 转发 IMU 帧给 K230（占用 TX DMA）；Stage 4 采纳 **IMU TX Y 分线**：

```
MS901M TX ──┬──→ MCU UART3 RX (PB13)
            └──→ K230 UART RX

MCU UART1 TX (PB6) ──阻塞写──→ K230 CMD RX    [~240 B/s + TEXT_RESP]
MCU UART1 RX (PB7) ←──DMA────  K230 CMD TX    [运动/PID/TEXT_CMD]
```

**收益**：释放 UART1 TX DMA；K230 获 200 Hz 原始 6 轴；MCU→K230 降至 ~240 B/s，阻塞写无压力。

### 1.2 接线

| 信号线 | 起点 | 终点 |
|--------|------|------|
| MS901M TX | MS901M | Y 分：MCU PB13 + K230 UART RX |
| MS901M RX | MCU PB12 | MS901M（不分线） |
| K230 CMD TX | K230 | MCU PB7 |
| K230 CMD RX | MCU PB6 | K230 |
| GND | 星型共地 | — |

---

## 2. SysConfig 与生成文件变更

### 2.1 LP_MSPM0G3507.syscfg

移除 K230 UART TX DMA：

```diff
- K230_UART.enabledDMATXTriggers = "DL_UART_DMA_INTERRUPT_TX";
- K230_UART.enableDMATX          = true;
```

### 2.2 ti_msp_dl_config

- 移除 `DMA_CH1`（TX DMA）
- `gDMA_CH0Config` 改为 `DL_DMA_WIDTH_BYTE` + dest increment
- UART1 @ PB6/PB7，115200 8N1

---

## 3. 软件实现（MCU 侧）

| 模块 | 文件 | 职责 |
|------|------|------|
| 协议层 | `k230_protocol.c/.h` | CRC16 按位计算、帧编解码、状态机解析 |
| BSP | `bsp_k230_uart.c/.h` | DMA RX 256 B → 512 B 环缓；阻塞 TX |
| 应用层 | `app_balance.c` | drain/dispatch、超时、TX 调度、TEXT_CMD 处理 |

### 3.1 主循环调度

| 频率 | 操作 |
|------|------|
| 1 kHz | `k230_drain_and_dispatch` + `k230_check_timeout`（500 ms 无帧归零） |
| 20 Hz | `k230_send_vehicle_status` |
| 1 Hz | `k230_send_heartbeat`；K230 在线时 `k230_send_text_resp` 精简心跳 |

### 3.2 帧分发（`k230_drain_and_dispatch`）

| CMD | 处理 |
|-----|------|
| 0x11 MOTION_CMD | 覆盖 `cmd.target_speed_cps` / `cmd.target_yaw_pm` |
| 0x13 PID_INJECT | `pid_id` 0/2/3 → `set_*_gains` |
| 0x12 HEARTBEAT_K230 | 刷新在线时间戳 |
| 0x21 TEXT_CMD | `k230_handle_text_cmd` → `handle_pid_command` + TEXT_RESP 回传 |

### 3.3 远程调试（v0.7 新增）

- K230 通过 WiFi TCP 收到文本 → 封装 TEXT_CMD(0x21) → MCU
- MCU 执行后回 TEXT_RESP(0x22)；1 Hz 镜像 `[hb] ...` 文本
- 详见 [Stage4-K230-Side.md §5](Stage4-K230-Side.md)

---

## 4. CRC 查找表错误与修复（2026-05-18）

原 CRC16 查表共 **50 处转录错误**，导致 `HEARTBEAT_MCU (0x02)` CRC 持续不一致（`VEHICLE_STATUS` 因 payload 未命中错误表项而正常）。

**修复**：移除查表，改用按位计算（`k230_protocol.c`），与 K230 Python 实现等价。修复后 `mcu_bad=crc` 停止增长。

---

## 5. 验证清单

- [ ] 编译通过，上电心跳 `k230_OFF`（K230 未连接）
- [ ] PB6 监听：20 Hz VEHICLE_STATUS + 1 Hz HEARTBEAT_MCU
- [ ] PB7 发送 MOTION_CMD → 车辆响应
- [ ] 断开 500 ms → 归零 + `k230_OFF`
- [ ] TEXT_CMD `bp 5000 0 5000 80` → TEXT_RESP `OK bp kp=...`
- [ ] MS901M Y 分线两端均收 200 Hz 数据

---

## 6. 变更日志

| 日期 | 版本 | 内容 | 执行方 |
|------|------|------|--------|
| 2026-05-17 | v0.1 | IMU TX 一分二 + MCU 帧协议全量实现 | 主控团队 |
| 2026-05-18 | v0.2~v0.4 | K230 联调日志判读；CRC 查表修复 | 主控/K230 |
| 2026-05-20 | v0.5 | 波特率 921600→115200 | 主控团队 |
| 2026-05-21 | v0.6 | 对齐 Stage 3.7 pid_id 0/2/3 | 主控团队 |
| 2026-05-24 | v0.7 | TEXT_CMD/TEXT_RESP 远程调试；MAX_PAYLOAD 128；协议真源合并至 Side.md | 主控团队 |
