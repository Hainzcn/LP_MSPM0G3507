# K230 端协同工作手册（Stage 4）

> **文档定位**：K230 ↔ MCU 通讯的**唯一协议真源**。面向 K230（CanMV）侧开发者，涵盖硬件接线、帧协议、远程调试、Python 参考实现与集成逻辑。
>
> **平台假设**：K230 运行 CanMV MicroPython（固件 ≥ 2.x），代码示例均为 MicroPython；如使用 C SDK，数据结构与协议完全一致，可直接对照 [`k230_protocol.h`](../../template/middle/k230_protocol.h) 移植。
>
> **MCU 侧实现记录**（SysConfig、CRC 修复历史、验证清单）：[Stage4-K230-Communication.md](Stage4-K230-Communication.md)
>
> **关联文档**：
> - 引脚分配（MCU 侧）：[Stage0-PinAllocation.md](Stage0-PinAllocation.md)
> - 项目总览：[Overview.md](../Overview/Overview.md)

---

## 0. 系统架构概览

```
MS901M TX ──┬──→ MCU UART3 RX (PB13)    [MCU 姿态环]
            └──→ K230 UART(1) RX         [K230 云台/视觉，200 Hz 原始 6 轴]

MCU UART1 TX (PB6) ──115200──→ K230 UART(2) RX   [MCU→K230：状态/心跳/文本响应]
MCU UART1 RX (PB7) ←──115200──  K230 UART(2) TX   [K230→MCU：运动/PID/文本命令]
GND — 三端共地（星型汇接）
```

- IMU 数据**不走** MCU↔K230 帧协议，由 MS901M TX Y 分线直达 K230
- MCU↔K230 命令链路 115200 8N1，定长 CRC 帧 + 文本透传帧（远程调试）
- MCU 500 ms 无 K230 帧 → 运动指令自动归零

---

## 1. 硬件接线

K230 需要占用 **两路独立 UART**：

| UART 用途 | K230 引脚 | 连接目标 | 波特率 |
|-----------|----------|---------|--------|
| IMU 直通 | 某路 UART RX（见下） | MS901M TX 线 Y 分出来的支路 | **115200** |
| MCU 命令 | 某路 UART TX + RX | MCU PB6 (TX→K230 RX) / MCU PB7 (RX←K230 TX) | **115200** |

> CanMV K230 板载多路 UART，推荐分配：
> - `UART(1)`：115200，接 MS901M TX Y 分线（**仅 RX 使用**）
> - `UART(2)`：115200，接 MCU UART1（TX+RX 双向）

**电平**：MS901M / MCU / K230 全部 3.3 V，直连兼容，**严格共 GND**（一根 GND 线从 MCU GND 接到 K230 GND）。

**不要**把 K230 的 5 V 电源接到 MCU 或 MS901M 的任何引脚。

---

## 2. MS901M 帧协议（IMU 直通解析）

MS901M 以 115200 8N1 主动推送二进制帧，格式如下：

```
0x55 0x55  ID  LEN  DATA[LEN]  CHECKSUM
```

- **CHECKSUM** = `(0x55 + 0x55 + ID + LEN + DATA[0] + ... + DATA[LEN-1]) & 0xFF`
- 本工程仅需解析以下三个帧 ID（MCU 配置为 200 Hz 上报）：

| ID | 名称 | LEN | DATA 布局 | 量纲 |
|----|------|-----|-----------|------|
| 0x01 | 姿态 | 6 | roll, pitch, yaw（各 int16 LE） | `val / 32768 * 180` → 度 |
| 0x02 | 四元数 | 8 | q0, q1, q2, q3（各 int16 LE） | `val / 32768` |
| 0x03 | 陀螺+加速 | 12 | ax, ay, az, gx, gy, gz（各 int16 LE） | accel: `val / 32768 * 4 g`；gyro: `val / 32768 * 2000 dps` |

> 量程 ±4 g / ±2000 dps 是 MCU 出厂默认配置写入 MS901M Flash 的值，不要更改。

### 2.1 MicroPython 解析器

```python
import struct

class MS901MParser:
    """ATK-MS901M 二进制帧逐字节解析器（仅解析 ID 0x01/0x02/0x03）。"""

    GYRO_FSR_DPS  = 2000.0
    ACCEL_FSR_G   = 4.0

    def __init__(self):
        self._buf = bytearray()
        self._state = 0   # 0=等待0x55_1, 1=等待0x55_2, 2=等待ID,
                          # 3=等待LEN, 4=读DATA, 5=等待CHECKSUM
        self._id   = 0
        self._len  = 0
        self._data = bytearray(16)
        self._didx = 0
        self.roll_deg  = 0.0
        self.pitch_deg = 0.0
        self.yaw_deg   = 0.0
        self.ax_g = self.ay_g = self.az_g = 0.0
        self.gx_dps = self.gy_dps = self.gz_dps = 0.0
        self.q0 = self.q1 = self.q2 = self.q3 = 0.0
        self.has_attitude = False
        self.has_gyro_acc = False
        self.good_frames  = 0
        self.bad_frames   = 0

    def feed(self, data: bytes):
        """喂入任意长度字节，内部逐字节处理，自动更新 pitch_deg 等字段。"""
        for b in data:
            self._process(b)

    def _process(self, b: int):
        s = self._state
        if s == 0:
            if b == 0x55:
                self._state = 1
        elif s == 1:
            self._state = 2 if b == 0x55 else (1 if b == 0x55 else 0)
        elif s == 2:
            self._id  = b
            self._state = 3
        elif s == 3:
            self._len  = b
            self._didx = 0
            self._state = 4 if b > 0 else 5
        elif s == 4:
            self._data[self._didx] = b
            self._didx += 1
            if self._didx >= self._len:
                self._state = 5
        elif s == 5:
            # 校验：sum(0x55,0x55,id,len,data...) & 0xFF
            chk = (0x55 + 0x55 + self._id + self._len) & 0xFF
            for i in range(self._len):
                chk = (chk + self._data[i]) & 0xFF
            if chk == b:
                self._dispatch()
                self.good_frames += 1
            else:
                self.bad_frames += 1
            self._state = 0

    def _dispatch(self):
        d = self._data
        if self._id == 0x01 and self._len == 6:
            r, p, y = struct.unpack_from('<hhh', d, 0)
            k = 180.0 / 32768.0
            self.roll_deg  = r * k
            self.pitch_deg = p * k
            self.yaw_deg   = y * k
            self.has_attitude = True
        elif self._id == 0x02 and self._len == 8:
            q0, q1, q2, q3 = struct.unpack_from('<hhhh', d, 0)
            k = 1.0 / 32768.0
            self.q0, self.q1 = q0 * k, q1 * k
            self.q2, self.q3 = q2 * k, q3 * k
        elif self._id == 0x03 and self._len == 12:
            ax, ay, az, gx, gy, gz = struct.unpack_from('<hhhhhh', d, 0)
            ka = self.ACCEL_FSR_G  / 32768.0
            kg = self.GYRO_FSR_DPS / 32768.0
            self.ax_g, self.ay_g, self.az_g = ax*ka, ay*ka, az*ka
            self.gx_dps, self.gy_dps, self.gz_dps = gx*kg, gy*kg, gz*kg
            self.has_gyro_acc = True
```

---

## 3. MCU 命令帧协议

### 3.1 帧格式

```
[0xAA][0x55][LEN][CMD][PAYLOAD × LEN][CRC16_LO][CRC16_HI][0x55][0xAA]
```

- **CRC16-CCITT**：多项式 `0x1021`，初始值 `0xFFFF`
- 校验范围：`LEN + CMD + PAYLOAD`（不含帧头尾）
- 最大 PAYLOAD：**128 字节**（v0.3 自 32 扩展，支持文本透传帧）

### 3.2 帧类型总表

| 方向 | CMD | 名称 | 频率 | PAYLOAD |
|------|-----|------|------|---------|
| MCU→K230 | 0x01 | VEHICLE_STATUS | 20 Hz | `avg_cps:i32 + safety_state:u8 + bat_mv:u16` (7 B) **或** + `track_phase:u8 + lap:u8` (9 B，Stage 3.11+) |
| MCU→K230 | 0x02 | HEARTBEAT_MCU | 1 Hz | `uptime_ms:u32` (4 B) |
| MCU→K230 | 0x22 | TEXT_RESP | 按需 + 1 Hz | ASCII 文本，≤128 B |
| K230→MCU | 0x11 | MOTION_CMD | 20~50 Hz | `target_v:i16 + target_omega:i16 + mode:u8` (5 B) |
| K230→MCU | 0x12 | HEARTBEAT_K230 | 1 Hz | `uptime_ms:u32` (4 B) |
| K230→MCU | 0x13 | PID_INJECT | 按需 | `pid_id:u8 + kp/ki/kd:f32` (13 B) |
| K230→MCU | 0x21 | TEXT_CMD | 按需 | ASCII 文本命令，≤120 B，**不含** `\r\n` |

> **`pid_id`**：0=角度，2=速度，3=航向；1（rate）/4（turn）已废弃。
>
> **`target_omega`**：映射为 MCU `target_yaw_pm`（permille）；非 0 时航向角闭环暂停。

### 3.3 CRC16-CCITT 实现

```python
def crc16_ccitt(data: bytes | bytearray) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
        crc &= 0xFFFF
    return crc
```

### 3.4 帧编码（K230 → MCU）

```python
def encode_frame(cmd: int, payload: bytes | bytearray = b'') -> bytes:
    """构造一帧完整数据（含帧头、CRC、帧尾）。"""
    length = len(payload)
    assert length <= 128, "payload too long"
    crc_data = bytes([length, cmd]) + bytes(payload)
    crc = crc16_ccitt(crc_data)
    return (bytes([0xAA, 0x55, length, cmd])
            + bytes(payload)
            + bytes([crc & 0xFF, crc >> 8])
            + bytes([0x55, 0xAA]))
```

### 3.5 帧解析器（MCU → K230）

```python
class MCUFrameParser:
    """逐字节解析 MCU → K230 帧。"""

    MAX_PAYLOAD = 128

    def __init__(self):
        self._state   = 0
        self._len     = 0
        self._cmd     = 0
        self._payload = bytearray(self.MAX_PAYLOAD)
        self._pidx    = 0
        self._crc_rx  = 0
        self.good = 0
        self.bad  = 0
        # 最近一帧
        self.last_cmd     = 0
        self.last_payload = bytes()

    def feed(self, data: bytes) -> list[tuple[int, bytes]]:
        """喂入字节流，返回本次解析出的所有完整帧列表 [(cmd, payload), ...]。"""
        frames = []
        for b in data:
            result = self._step(b)
            if result is not None:
                frames.append(result)
        return frames

    def _step(self, b: int):
        s = self._state
        if s == 0:
            if b == 0xAA: self._state = 1
        elif s == 1:
            self._state = 2 if b == 0x55 else (1 if b == 0xAA else 0)
        elif s == 2:
            if b > self.MAX_PAYLOAD:
                self.bad += 1; self._state = 0
            else:
                self._len = b; self._state = 3
        elif s == 3:
            self._cmd = b
            self._pidx = 0
            self._state = 4 if self._len > 0 else 5
        elif s == 4:
            self._payload[self._pidx] = b
            self._pidx += 1
            if self._pidx >= self._len: self._state = 5
        elif s == 5:
            self._crc_rx = b; self._state = 6
        elif s == 6:
            self._crc_rx |= b << 8; self._state = 7
        elif s == 7:
            self._state = 8 if b == 0x55 else 0
        elif s == 8:
            self._state = 0
            if b != 0xAA:
                self.bad += 1; return None
            crc_data = bytes([self._len, self._cmd]) + bytes(self._payload[:self._len])
            if crc16_ccitt(crc_data) != self._crc_rx:
                self.bad += 1; return None
            self.good += 1
            self.last_cmd     = self._cmd
            self.last_payload = bytes(self._payload[:self._len])
            return (self._cmd, self.last_payload)
        return None
```

---

## 4. 业务帧定义

### 4.1 MCU → K230 帧（接收）

#### VEHICLE_STATUS (CMD=0x01, 20 Hz)

**旧固件 7 B**（`'<iBH'`）：

```python
avg_cps, safety_state, bat_mv = struct.unpack_from('<iBH', payload, 0)
```

**新固件 9 B**（`'<iBHBB'`，Stage 3.11 赛道模式，向后兼容）：

```python
def parse_vehicle_status(payload: bytes):
    """
    returns: (avg_cps, safety_state, bat_mv, track_phase, lap)
      track_phase -- 见 TRACK_PHASE_*（仅 TRACE=3 时 K230 应下发循线驱动）
      lap         -- 当前圈号（1 起；0=未开始）
    """
    if len(payload) >= 9:
        return struct.unpack_from('<iBHBB', payload, 0)
    avg_cps, safety_state, bat_mv = struct.unpack_from('<iBH', payload, 0)
    return avg_cps, safety_state, bat_mv, 0, 0   # TRACK_PHASE_IDLE
```

**K230 阶段闸门**（`config.TRACK_FOLLOW_MCU_PHASE=True` 默认）：主循环每帧 `controller.follow_mcu_phase(vehicle_track_phase == TRACK_PHASE_TRACE)`；非 TRACE 阶段控制律仍可算，但 `mode=idle` → MOTION_CMD `(0,0)`。详见 [K230 phase_G_track_mode.md](../../../K230/docs/TaskLog/phase_G_track_mode.md)。

#### HEARTBEAT_MCU (CMD=0x02, 1 Hz)

```python
def parse_heartbeat_mcu(payload: bytes) -> int:
    """returns: uptime_ms"""
    return struct.unpack_from('<I', payload, 0)[0]
```

### 4.2 K230 → MCU 帧（发送）

#### MOTION_CMD (CMD=0x11, 20~50 Hz)

```python
def make_motion_cmd(target_v: int, target_omega: int, mode: int = 1) -> bytes:
    """
    target_v     -- 纵向速度（counts/s 已除以 SCALE=10，正=前进）
                    例：target_v=500 ≡ 5000 raw cps ≡ ~0.15 rev/s
    target_omega -- 映射为 MCU `target_yaw_pm`（permille）；非 0 时航向角闭环暂停（Stage 3.7）
    mode         -- 0=停止, 1=正常行驶
    """
    payload = struct.pack('<hhB', target_v, target_omega, mode)
    return encode_frame(0x11, payload)
```

#### HEARTBEAT_K230 (CMD=0x12, 1 Hz)

```python
def make_heartbeat_k230(uptime_ms: int) -> bytes:
    return encode_frame(0x12, struct.pack('<I', uptime_ms))
```

#### PID_INJECT (CMD=0x13, 按需)

```python
def make_pid_inject(pid_id: int, kp: float, ki: float, kd: float) -> bytes:
    """
    pid_id: 0=angle环, 2=speed环, 3=yaw航向角环
    （1=旧rate环、4=旧turn环 已移除，MCU 侧忽略）
    """
    payload = struct.pack('<Bfff', pid_id, kp, ki, kd)
    return encode_frame(0x13, payload)
```

#### TEXT_CMD (CMD=0x21, 按需) — 远程调试透传

将 UART0 文本命令封装为帧发送给 MCU。payload 为纯 ASCII，**不含** `\r\n`，建议 ≤120 字节。

```python
def make_text_cmd(text: str) -> bytes:
    """例：make_text_cmd('bp 5000 0 5000 80')"""
    payload = text.encode('ascii')
    assert len(payload) <= 120
    return encode_frame(0x21, payload)
```

**支持的命令**（与 MCU UART0 本地调试一致）：

| 命令 | 说明 |
|------|------|
| `bp <kp> <ki> <kd> [ofs]` | 角度环 PID（kp/ki/kd ×1000，ofs 直接 permille） |
| `sp ...` / `yp ...` | 速度环 / 航向环 |
| `bo <ofs>` | 仅设角度环 out_offset |
| `pid` / `pid?` | 查询三组 PID |
| `c` / `circle` | 启动圆运动（默认 800mm/200mm/s CW） |
| `circle <diam> <v>` | 自定义圆运动 |
| `cx` | 取消圆运动 |
| `h` | 简短帮助 |

### 4.3 MCU → K230 文本响应（接收）

#### TEXT_RESP (CMD=0x22)

MCU 执行 TEXT_CMD 后回传执行结果；K230 在线时 MCU 还以 1 Hz 推送精简心跳文本。

```python
def parse_text_resp(payload: bytes) -> str:
    """返回 ASCII 文本（已截断至 payload 长度）。"""
    return payload.decode('ascii', errors='replace')
```

**典型响应格式**：

| 触发命令 | TEXT_RESP 示例 |
|----------|----------------|
| `bp 5000 0 5000 80` | `OK bp kp=5000 ki=0 kd=5000 ofs=80\r\n` |
| `pid?` | `OK bp kp=... \| sp kp=... \| yp kp=...\r\n` |
| `c` | `OK circle start\r\n` |
| `cx` | `OK circle cancel\r\n` |
| 未识别 | `ERR unknown: xxx\r\n` |
| （1 Hz 自动） | `[hb] t=5s ARMED pit=+0.12 spd=0cps L=0 R=0 bat=7800mV` |

> kp/ki/kd 值为 ×1000 整数，与 UART0 本地命令格式一致。

---

## 5. 远程调试（WiFi AP + TCP 透传）

K230 侧通过 WiFi AP 模式提供 TCP 服务，将 PC/手机发来的文本命令转为 TEXT_CMD 帧，并将 TEXT_RESP 回传给 TCP 客户端。

```
PC/手机 ──TCP──→ K230 WiFi AP ──TEXT_CMD(0x21)──→ MCU UART1
PC/手机 ←─TCP──  K230 WiFi AP ←─TEXT_RESP(0x22)──  MCU UART1
```

**K230 侧桥接职责**（本仓库不含可运行脚本，需在 CanMV 上实现）：

1. 启动 WiFi AP（SSID/密码自行配置）
2. 监听 TCP 端口（建议 8888，裸 TCP，PuTTY / netcat 可直接连接）
3. 收到一行 ASCII（以 `\n` 结尾）→ 去掉 `\r\n` → `make_text_cmd()` → 写 UART(2)
4. 从 UART(2) 解析 `CMD=0x22` 帧 → 解码 payload → 追加 `\n` 写回 TCP

**桥接伪代码**：

```python
import socket
from machine import UART

uart_mcu = UART(2, baudrate=115200, bits=8, parity=None, stop=1)
mcu_parser = MCUFrameParser()
tcp_clients = []   # 已连接的远程调试客户端

def on_mcu_rx(data: bytes):
    for cmd, payload in mcu_parser.feed(data):
        if cmd == 0x22:   # TEXT_RESP
            text = parse_text_resp(payload)
            for cli in tcp_clients:
                try:
                    cli.send(text.encode('ascii'))
                except OSError:
                    pass

def on_tcp_line(line: str):
    line = line.strip()
    if line:
        uart_mcu.write(make_text_cmd(line))
```

**使用示例**（PC 连接 K230 AP 后）：

```text
> bp 5000 0 5000 80
OK bp kp=5000 ki=0 kd=5000 ofs=80

> c
OK circle start

> pid?
OK bp kp=5000 ki=0 kd=5000 ofs=80 | sp kp=2000 ki=50 kd=0 ofs=0 | yp kp=800 ki=0 kd=200 ofs=0

[hb] t=12s ARMED pit=+1.23 spd=150cps L=120 R=115 bat=7800mV
```

---

## 6. 完整集成示例

以下是一个完整的 K230 主控脚本骨架，演示两路 UART 并发工作的典型结构：

```python
"""
K230 平衡车控制端 —— Stage 4 通讯集成骨架
运行于 CanMV K230 MicroPython 环境
"""

import time
import struct
from machine import UART

# ---- 硬件初始化 ----
uart_imu = UART(1, baudrate=115200, bits=8, parity=None, stop=1)   # MS901M Y分线
uart_mcu = UART(2, baudrate=115200, bits=8, parity=None, stop=1)   # MCU UART1

imu = MS901MParser()
mcu_parser = MCUFrameParser()

# ---- 状态变量 ----
vehicle_avg_cps   = 0
vehicle_safety    = 0
vehicle_bat_mv    = 0
mcu_last_hb_ms    = time.ticks_ms()
MCU_TIMEOUT_MS    = 1000   # MCU 心跳超时阈值（K230 侧比 MCU 侧宽松）

uptime_ms = 0

def is_mcu_online() -> bool:
    return time.ticks_diff(time.ticks_ms(), mcu_last_hb_ms) < MCU_TIMEOUT_MS

# ---- 运动决策（占位，Stage 5 由循迹/视觉算法填充） ----
def compute_motion() -> tuple[int, int]:
    """
    返回 (target_v, target_omega)：
      target_v     整数，单位同 VEHICLE_STATUS avg_cps / SCALE
      target_omega permille，正=顺时针
    如果 MCU 离线则返回 (0, 0)
    """
    if not is_mcu_online():
        return (0, 0)
    # TODO: Stage 5 填入循迹、光斑算法
    return (0, 0)

# ---- 主循环 ----
last_cmd_ms  = time.ticks_ms()
last_hb_ms   = time.ticks_ms()
CMD_PERIOD   = 25    # 40 Hz 发送运动指令
HB_PERIOD    = 1000  # 1 Hz 心跳

while True:
    now = time.ticks_ms()
    uptime_ms = now

    # 1. drain IMU UART
    imu_data = uart_imu.read(64)
    if imu_data:
        imu.feed(imu_data)

    # 2. drain MCU UART → 解析帧
    mcu_data = uart_mcu.read(128)
    if mcu_data:
        frames = mcu_parser.feed(mcu_data)
        for cmd, payload in frames:
            if cmd == 0x01 and len(payload) == 7:   # VEHICLE_STATUS
                vehicle_avg_cps, vehicle_safety, vehicle_bat_mv = parse_vehicle_status(payload)
            elif cmd == 0x02:                        # HEARTBEAT_MCU
                mcu_last_hb_ms = now
            elif cmd == 0x22:                        # TEXT_RESP（远程调试回传）
                text = parse_text_resp(payload)
                for cli in tcp_clients:             # 转发给 WiFi TCP 客户端
                    cli.send(text.encode('ascii'))

    # 3. 20~50 Hz：发送运动指令
    if time.ticks_diff(now, last_cmd_ms) >= CMD_PERIOD:
        last_cmd_ms = now
        v, omega = compute_motion()
        uart_mcu.write(make_motion_cmd(v, omega, mode=1 if is_mcu_online() else 0))

    # 4. 1 Hz：心跳 + 状态打印
    if time.ticks_diff(now, last_hb_ms) >= HB_PERIOD:
        last_hb_ms = now
        uart_mcu.write(make_heartbeat_k230(uptime_ms))
        print(f"[hb] pitch={imu.pitch_deg:.2f} gy={imu.gy_dps:.1f} "
              f"avg_cps={vehicle_avg_cps} bat={vehicle_bat_mv}mV "
              f"mcu={'ON' if is_mcu_online() else 'OFF'} "
              f"imu_g={imu.good_frames}/b={imu.bad_frames} "
              f"mcu_g={mcu_parser.good}/b={mcu_parser.bad}")

    # 5. 云台控制（Stage 5 实现，此处占位）
    if imu.has_attitude:
        pitch_for_gimbal = imu.pitch_deg   # 直接用，无需 MCU 转发
        # gimbal.set_pitch_feedforward(-pitch_for_gimbal)
        pass

    time.sleep_ms(1)
```

---

## 7. 安全规则

| 规则 | 说明 |
|------|------|
| MCU 离线时发 `(v=0, omega=0, mode=0)` | MCU 侧有 500 ms 超时归零，但 K230 侧也应主动归零避免存留指令 |
| 不得发 `mode=1` 但 `safety_state >= 3` | FALLEN / BAT_STOP 时 MCU 会拒绝运动指令，K230 应等 `safety_state` 恢复后再发 |
| `bat_mv < 9500` 时减速 | 对应 MCU 的 LOW_BAT_WARN 提前量，减少 MCU 紧急停车风险 |
| 心跳必须每秒发送 | MCU 500 ms 无帧即归零，K230 侧心跳间隔建议 ≤ 400 ms |

---

## 8. 调试方法

### 8.1 单步验证 IMU 链路

```python
uart_imu = UART(1, baudrate=115200, bits=8, parity=None, stop=1)
imu = MS901MParser()
while True:
    d = uart_imu.read(64)
    if d:
        imu.feed(d)
        if imu.has_attitude:
            print(f"pitch={imu.pitch_deg:.2f} roll={imu.roll_deg:.2f}")
    time.sleep_ms(5)
```

正常情况下 pitch 以 200 Hz 刷新；若无数据，检查 MS901M TX → K230 RX 分线焊点和共地。

### 8.2 单步验证 MCU 命令链路（无 K230 侧主逻辑）

在 PC 上用任意串口工具连接 MCU PB6（115200 8N1），应能看到：
- 每 50 ms 一帧 `AA 55 07 01 ...` (VEHICLE_STATUS)
- 每 1000 ms 一帧 `AA 55 04 02 ...` (HEARTBEAT_MCU)

发送以下字节触发 MOTION_CMD（v=100, omega=0, mode=1）：

```python
# 在 PC Python 端生成测试帧
payload = struct.pack('<hhB', 100, 0, 1)
frame = encode_frame(0x11, payload)
print(frame.hex())
```

收到后 MCU 日志会显示 `k230_ON`，车辆以 v=100 (×10 = 1000 raw cps ≈ 低速前行)  运动。

### 8.3 回环自测（不接 MCU）

将 K230 命令 UART 的 TX 短接 RX，发送任意帧后解析器应立即收到同一帧，`good` 计数递增。

### 8.4 远程调试 TEXT_CMD 自测

在 K230 侧直接发送 TEXT_CMD 帧（无需 WiFi）：

```python
# 设置角度环 PID
uart_mcu.write(make_text_cmd('bp 5000 0 5000 80'))
time.sleep_ms(50)
d = uart_mcu.read(256)
for cmd, payload in mcu_parser.feed(d):
    if cmd == 0x22:
        print(parse_text_resp(payload))
# 预期：OK bp kp=5000 ki=0 kd=5000 ofs=80
```

---

## 9. MCU 上行坏帧判读（联调参考）

K230 侧可统计 MCU→K230 坏帧，日志格式：

```text
mcu_bad=lenX/t1Y/t2Z/crcW last=reason:Lxx:Cyy:rx/calc
```

| 字段 | 含义 |
|------|------|
| `lenX` | `LEN > 128` 的坏帧数 |
| `t1Y` | 帧尾第 1 字节不是 `0x55` |
| `t2Z` | 帧尾第 2 字节不是 `0xAA` |
| `crcW` | CRC16 不匹配 |
| `last=crc:L4:C02:rx/calc` | 最近坏帧：LEN=4、CMD=0x02、收到 CRC vs 本地计算 |

**在线判定**：收到任意合法 MCU 上行帧（`VEHICLE_STATUS` 或 `HEARTBEAT_MCU` 或 `TEXT_RESP`）均应刷新在线时间戳，避免单一帧类型 CRC 问题导致 `MCU:ON/OFF` 抖动。

**CRC 参数**（两端必须一致）：

- poly = `0x1021`，init = `0xFFFF`，不反射
- 校验范围 = `LEN + CMD + PAYLOAD`（不含帧头尾）
- CRC 字节序：低字节在前

---

## 10. 变更日志

| 日期 | 版本 | 内容 |
|------|------|------|
| 2026-05-17 | v0.1 | 初版：IMU 解析、命令帧协议、集成示例与调试方法 |
| 2026-05-21 | v0.2 | 对齐 Stage 3.7：`pid_id` 0/2/3；`target_omega` → `target_yaw_pm` |
| 2026-05-24 | v0.3 | 合并为协议唯一真源；新增 TEXT_CMD(0x21)/TEXT_RESP(0x22)；MAX_PAYLOAD 32→128；远程调试 WiFi AP+TCP 桥接说明；移植联调坏帧判读 |
| 2026-05-30 | v0.4 | **赛道模式流程对齐**：`VEHICLE_STATUS` 9 B 扩展 + `TRACK_PHASE_*` 常量；`McuLink.vehicle_track_phase/lap`；`TrackingController.follow_mcu_phase()`；`TRACK_FOLLOW_MCU_PHASE` 配置项。详见 [phase_G_track_mode.md](../../../K230/docs/TaskLog/phase_G_track_mode.md) |
