"""
serial_plot.py — 串口CSV实时数据图表
=====================================
接收串口CSV协议数据并实时绘制折线图表。

CSV格式（每行7个逗号分隔的浮点数）：
    俯仰角,左期望转速,右期望转速,左实际转速,右实际转速,左PWM占空比,右PWM占空比

图表布局：
  ┌──────────────────────────────┐
  │ 图1: 左电机 期望/实际转速(rpm)  │  蓝色实线=期望  红色虚线=实际
  │ 图2: 右电机 期望/实际转速(rpm)  │  蓝色实线=期望  红色虚线=实际
  │ 图3: 俯仰角 (°)               │  绿色实线
  │ 图4: 左PWM占空比 (‰)          │  品红实线
  │ 图5: 右PWM占空比 (‰)          │  品红实线
  └──────────────────────────────┘
  所有图表时间轴上下对齐，共享x轴。

操作说明：
  鼠标滚轮    — 缩放时间窗口
  键盘 +/-    — 放大/缩小时间窗口
  键盘 Home   — 恢复默认10s窗口
  键盘 空格   — 暂停/恢复自动滚动
  键盘 r      — 重置y轴自动范围

用法:
    python serial_plot.py --port COM3
    python serial_plot.py --port COM3 --baud 115200 --window 10
    python serial_plot.py --list

依赖:
    pip install pyserial matplotlib numpy
"""

import argparse
import collections
import sys
import threading
import time

import numpy as np

import serial
import serial.tools.list_ports

import matplotlib

_backend = None
for _try in ("Qt5Agg", "QtAgg", "TkAgg"):
    try:
        matplotlib.use(_try, force=True)
        _backend = _try
        break
    except Exception:
        continue
if _backend is None:
    matplotlib.use("TkAgg")

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.ticker import MaxNLocator

HELP_TEXT = (
    "鼠标滚轮:缩放窗口 | +/-:调窗口 | Home:默认10s | "
    "空格:暂停滚动 | r:重置Y轴范围 | q:退出"
)


class RingBuffer:
    def __init__(self, maxlen=5000):
        self._lock = threading.Lock()
        self.t = collections.deque(maxlen=maxlen)
        self.pitch = collections.deque(maxlen=maxlen)
        self.left_target = collections.deque(maxlen=maxlen)
        self.right_target = collections.deque(maxlen=maxlen)
        self.left_actual = collections.deque(maxlen=maxlen)
        self.right_actual = collections.deque(maxlen=maxlen)
        self.left_pwm = collections.deque(maxlen=maxlen)
        self.right_pwm = collections.deque(maxlen=maxlen)

    def append(self, timestamp, pitch, lt, rt, la, ra, lp, rp):
        with self._lock:
            self.t.append(timestamp)
            self.pitch.append(pitch)
            self.left_target.append(lt)
            self.right_target.append(rt)
            self.left_actual.append(la)
            self.right_actual.append(ra)
            self.left_pwm.append(lp)
            self.right_pwm.append(rp)

    def snapshot(self):
        with self._lock:
            return (
                list(self.t),
                list(self.pitch),
                list(self.left_target),
                list(self.right_target),
                list(self.left_actual),
                list(self.right_actual),
                list(self.left_pwm),
                list(self.right_pwm),
            )

    @property
    def count(self):
        with self._lock:
            return len(self.t)


def list_ports():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("未发现可用串口。")
    else:
        print(f"{'端口':<12} {'描述':<40} {'硬件ID'}")
        print("-" * 80)
        for p in sorted(ports):
            print(f"{p.device:<12} {p.description:<40} {p.hwid}")
    sys.exit(0)


def serial_reader(port, baud, buffer, stop_event):
    try:
        ser = serial.Serial(port, baud, timeout=0.3)
    except serial.SerialException as e:
        print(f"[错误] 无法打开串口 {port}: {e}", file=sys.stderr)
        stop_event.set()
        return

    t0 = time.monotonic()
    line_count = 0
    print(f"[串口] {port} @ {baud} bps 已打开，等待数据...")

    while not stop_event.is_set():
        try:
            raw = ser.readline()
        except serial.SerialException:
            break

        if not raw:
            continue

        try:
            line = raw.decode("ascii", errors="ignore").strip()
        except Exception:
            continue

        if not line:
            continue

        parts = line.split(",")
        if len(parts) != 7:
            continue

        try:
            vals = [float(p.strip()) for p in parts]
        except ValueError:
            continue

        elapsed = time.monotonic() - t0
        buffer.append(elapsed, vals[0], vals[1], vals[2],
                      vals[3], vals[4], vals[5], vals[6])
        line_count += 1

    ser.close()
    print(f"\n[串口] 已关闭，共接收 {line_count} 行")


class RealTimePlot:
    def __init__(self, buffer, window_sec=10.0):
        self.buffer = buffer
        self.window_sec = window_sec
        self.auto_scroll = True
        self._frame_idx = 0
        self._ylim_frozen = False  # 用户手动缩放Y轴后暂停自动调整

        plt.rcParams["toolbar"] = "toolbar2"
        self.fig = plt.figure(figsize=(14, 10))
        self.fig.canvas.manager.set_window_title(
            f"串口实时监控 [窗口:{window_sec:.0f}s] {HELP_TEXT}")

        gs = self.fig.add_gridspec(5, 1, hspace=0.12, left=0.08, right=0.985,
                                   top=0.97, bottom=0.05)

        # ── 图1: 左电机 期望转速 + 实际转速 叠加 ──
        self.ax1 = self.fig.add_subplot(gs[0])
        self.ax1.set_ylabel("左电机\n(rpm)", fontsize=9, linespacing=1.4)
        self.ax1.grid(True, alpha=0.3, linestyle="--")
        self.line_lt, = self.ax1.plot([], [], "#2196F3", linewidth=1.0, label="期望")
        self.line_la, = self.ax1.plot([], [], "#FF5722", linewidth=1.0,
                                      linestyle="--", dashes=(4, 2), label="实际")
        self.ax1.legend(loc="upper right", fontsize=8, ncol=2,
                        framealpha=0.6, edgecolor="none")
        self.ax1.yaxis.set_major_locator(MaxNLocator(5))

        # ── 图2: 右电机 期望转速 + 实际转速 叠加 ──
        self.ax2 = self.fig.add_subplot(gs[1], sharex=self.ax1)
        self.ax2.set_ylabel("右电机\n(rpm)", fontsize=9, linespacing=1.4)
        self.ax2.grid(True, alpha=0.3, linestyle="--")
        self.line_rt, = self.ax2.plot([], [], "#2196F3", linewidth=1.0, label="期望")
        self.line_ra, = self.ax2.plot([], [], "#FF5722", linewidth=1.0,
                                      linestyle="--", dashes=(4, 2), label="实际")
        self.ax2.legend(loc="upper right", fontsize=8, ncol=2,
                        framealpha=0.6, edgecolor="none")
        self.ax2.yaxis.set_major_locator(MaxNLocator(5))

        # ── 图3: 俯仰角 ──
        self.ax3 = self.fig.add_subplot(gs[2], sharex=self.ax1)
        self.ax3.set_ylabel("俯仰角\n(°)", fontsize=9, linespacing=1.4)
        self.ax3.grid(True, alpha=0.3, linestyle="--")
        self.line_pitch, = self.ax3.plot([], [], "#4CAF50", linewidth=1.0)
        self.ax3.axhline(y=0, color="#4CAF50", alpha=0.25, linestyle=":", linewidth=0.8)
        self.ax3.yaxis.set_major_locator(MaxNLocator(5))

        # ── 图4: 左PWM占空比 ──
        self.ax4 = self.fig.add_subplot(gs[3], sharex=self.ax1)
        self.ax4.set_ylabel("左PWM\n(‰)", fontsize=9, linespacing=1.4)
        self.ax4.grid(True, alpha=0.3, linestyle="--")
        self.ax4.set_ylim(-50, 1050)
        self.line_lp, = self.ax4.plot([], [], "#9C27B0", linewidth=1.0)
        self.ax4.yaxis.set_major_locator(MaxNLocator(5))

        # ── 图5: 右PWM占空比 ──
        self.ax5 = self.fig.add_subplot(gs[4], sharex=self.ax1)
        self.ax5.set_ylabel("右PWM\n(‰)", fontsize=9, linespacing=1.4)
        self.ax5.set_xlabel("时间 (s)")
        self.ax5.grid(True, alpha=0.3, linestyle="--")
        self.ax5.set_ylim(-50, 1050)
        self.line_rp, = self.ax5.plot([], [], "#9C27B0", linewidth=1.0)
        self.ax5.yaxis.set_major_locator(MaxNLocator(5))

        # 隐藏上方子图的x轴刻度标签
        for ax in [self.ax1, self.ax2, self.ax3, self.ax4]:
            plt.setp(ax.get_xticklabels(), visible=False)

        self.all_lines = [
            self.line_lt, self.line_la, self.line_rt, self.line_ra,
            self.line_pitch, self.line_lp, self.line_rp,
        ]

        self.all_axes = [self.ax1, self.ax2, self.ax3, self.ax4, self.ax5]

        self.fig.canvas.mpl_connect("scroll_event", self._on_scroll)
        self.fig.canvas.mpl_connect("key_press_event", self._on_key)

    def _update_title(self):
        status = "自动" if self.auto_scroll else "暂停"
        self.fig.canvas.manager.set_window_title(
            f"串口实时监控 [窗口:{self.window_sec:.1f}s|{status}] {HELP_TEXT}")

    def _on_scroll(self, event):
        if event.inaxes not in self.all_axes:
            return
        factor = 0.8 if event.button == "up" else 1.25
        self.window_sec = max(1.0, min(300.0, self.window_sec * factor))
        self.window_sec = round(self.window_sec, 1)
        self._update_title()

    def _on_key(self, event):
        if event.key == "+" or event.key == "=":
            self.window_sec = min(300.0, self.window_sec * 1.25)
        elif event.key == "-":
            self.window_sec = max(1.0, self.window_sec * 0.8)
        elif event.key == "home":
            self.window_sec = 10.0
        elif event.key == " ":
            self.auto_scroll = not self.auto_scroll
        elif event.key == "r":
            self._ylim_frozen = False
        elif event.key == "f":
            self._ylim_frozen = not self._ylim_frozen
        else:
            return
        self.window_sec = round(self.window_sec, 1)
        self._update_title()

    @staticmethod
    def _smart_ylim(*series, margin_frac=0.08, abs_margin=1.0):
        merged = []
        for s in series:
            if s:
                merged.extend(s)
        if not merged:
            return -abs_margin, abs_margin
        lo, hi = min(merged), max(merged)
        if lo == hi:
            return lo - abs_margin, hi + abs_margin
        r = hi - lo
        pad = max(r * margin_frac, abs_margin)
        return lo - pad, hi + pad

    def _update(self, frame):
        ts, pitch, lt, rt, la, ra, lp, rp = self.buffer.snapshot()

        self.line_lt.set_data(ts, lt)
        self.line_la.set_data(ts, la)
        self.line_rt.set_data(ts, rt)
        self.line_ra.set_data(ts, ra)
        self.line_pitch.set_data(ts, pitch)
        self.line_lp.set_data(ts, lp)
        self.line_rp.set_data(ts, rp)

        if ts and self.auto_scroll:
            t_max = ts[-1]
            t_min = max(0.0, t_max - self.window_sec)
            self.ax1.set_xlim(t_min, t_max)

        self._frame_idx += 1
        if self._frame_idx % 15 == 0 and not self._ylim_frozen and ts:
            self.ax1.set_ylim(self._smart_ylim(lt, la, margin_frac=0.10))
            self.ax2.set_ylim(self._smart_ylim(rt, ra, margin_frac=0.10))
            self.ax3.set_ylim(self._smart_ylim(pitch, margin_frac=0.10))
            # PWM y轴保持固定范围，仅当数据超出时才扩展
            if any(v > 1050 for v in lp) or any(v < -50 for v in lp):
                self.ax4.set_ylim(self._smart_ylim(lp, abs_margin=20))
            if any(v > 1050 for v in rp) or any(v < -50 for v in rp):
                self.ax5.set_ylim(self._smart_ylim(rp, abs_margin=20))

        return self.all_lines

    def run(self):
        self.ani = FuncAnimation(
            self.fig, self._update,
            interval=50,
            blit=False,
            cache_frame_data=False,
        )
        plt.show()


def main():
    parser = argparse.ArgumentParser(
        description="串口CSV实时数据图表监控 — 接收7路CSV并绘制5幅同步折线图")
    parser.add_argument("--port", "-p",
                        help="串口名称，如 COM3")
    parser.add_argument("--baud", "-b", type=int, default=115200,
                        help="波特率（默认115200）")
    parser.add_argument("--window", "-w", type=float, default=10.0,
                        help="初始时间窗口秒数（默认10s）")
    parser.add_argument("--max-points", type=int, default=5000,
                        help="最大缓存点数（默认5000）")
    parser.add_argument("--list", "-l", action="store_true",
                        help="列出系统可用串口并退出")
    args = parser.parse_args()

    if args.list:
        list_ports()

    if not args.port:
        parser.error("请指定串口 --port COM?，或用 --list 查看可用串口")

    buffer = RingBuffer(maxlen=args.max_points)
    stop_event = threading.Event()

    reader = threading.Thread(
        target=serial_reader,
        args=(args.port, args.baud, buffer, stop_event),
        daemon=True,
    )
    reader.start()

    plot = RealTimePlot(buffer, window_sec=args.window)
    try:
        plot.run()
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        reader.join(timeout=2.0)
        print("程序已退出。")


if __name__ == "__main__":
    main()
