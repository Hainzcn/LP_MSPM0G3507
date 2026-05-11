"""
serial_plot.py — 串口CSV实时数据图表 (PyQt6)
=============================================
接收串口CSV协议数据并实时绘制折线图表。

CSV格式（每行9个逗号分隔字段，前2列为帧头）：
    lt,<时间戳ms>,俯仰角,左期望转速,右期望转速,左实际转速,右实际转速,左PWM,右PWM

图表布局：
  ┌──────────────────────────────┐
  │ 图1: 左电机 期望/实际转速(rpm)  │  蓝实线=期望  红虚线=实际
  │ 图2: 右电机 期望/实际转速(rpm)  │  蓝实线=期望  红虚线=实际
  │ 图3: 俯仰角 (°)               │  绿实线
  │ 图4: 左PWM占空比 (‰)          │  品红实线
  │ 图5: 右PWM占空比 (‰)          │  品红实线
  └──────────────────────────────┘
  所有图表时间轴上下对齐，共享x轴。

操作说明：
  鼠标滚轮    — 缩放时间窗口
  键盘 +/-    — 放大/缩小时间窗口
  键盘 Home   — 恢复默认10s窗口
  键盘 Space  — 暂停/恢复自动滚动
  键盘 R      — 重置y轴自动范围
  键盘 F      — 冻结/解冻y轴

用法:
    python serial_plot.py --port COM3
    python serial_plot.py --port COM3 --baud 115200 --window 10
    python serial_plot.py --list

依赖:
    pip install pyserial matplotlib numpy PyQt6
"""

import argparse
import collections
import sys
import time
from collections import deque

import numpy as np
import serial
import serial.tools.list_ports

import matplotlib
matplotlib.use("QtAgg")

from matplotlib.backends.backend_qtagg import (
    FigureCanvasQTAgg,
    NavigationToolbar2QT,
)
from matplotlib.figure import Figure
from matplotlib.ticker import MaxNLocator

from PyQt6 import QtCore, QtGui, QtWidgets

HELP_TEXT = (
    "滚轮:缩放窗口 | +/-:调窗口 | Home:默认10s | "
    "Space:暂停滚动 | R:重置Y轴 | F:冻结Y轴"
)


class RingBuffer:
    def __init__(self, maxlen=5000):
        self._lock = QtCore.QMutex()
        self.t = deque(maxlen=maxlen)
        self.pitch = deque(maxlen=maxlen)
        self.left_target = deque(maxlen=maxlen)
        self.right_target = deque(maxlen=maxlen)
        self.left_actual = deque(maxlen=maxlen)
        self.right_actual = deque(maxlen=maxlen)
        self.left_pwm = deque(maxlen=maxlen)
        self.right_pwm = deque(maxlen=maxlen)
        self._count = 0

    def append(self, timestamp, pitch, lt, rt, la, ra, lp, rp):
        locker = QtCore.QMutexLocker(self._lock)
        self.t.append(timestamp)
        self.pitch.append(pitch)
        self.left_target.append(lt)
        self.right_target.append(rt)
        self.left_actual.append(la)
        self.right_actual.append(ra)
        self.left_pwm.append(lp)
        self.right_pwm.append(rp)
        self._count += 1

    def snapshot(self):
        locker = QtCore.QMutexLocker(self._lock)
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
        locker = QtCore.QMutexLocker(self._lock)
        return self._count


class SerialWorker(QtCore.QThread):
    data_ready = QtCore.pyqtSignal(float, float, float, float, float, float, float, float)
    error_occurred = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal(int)

    def __init__(self, port, baud):
        super().__init__()
        self._port = port
        self._baud = baud
        self._stop_flag = False

    def stop(self):
        self._stop_flag = True

    def run(self):
        try:
            ser = serial.Serial(self._port, self._baud, timeout=0.3)
        except serial.SerialException as e:
            self.error_occurred.emit(f"无法打开串口 {self._port}: {e}")
            return

        t0 = time.monotonic()
        line_count = 0

        while not self._stop_flag:
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
            if len(parts) < 9 or parts[0].strip() != "lt":
                continue

            try:
                vals = [float(p.strip()) for p in parts[2:9]]
            except ValueError:
                continue

            elapsed = time.monotonic() - t0
            self.data_ready.emit(
                elapsed,
                vals[0], vals[1], vals[2],
                vals[3], vals[4], vals[5], vals[6],
            )
            line_count += 1

        ser.close()
        self.finished.emit(line_count)


class RealTimeCanvas(FigureCanvasQTAgg):
    scroll_changed = QtCore.pyqtSignal()

    def __init__(self, parent=None):
        self.fig = Figure(figsize=(14, 10))
        self.fig.set_tight_layout({"rect": [0.08, 0.05, 0.985, 0.97]})
        super().__init__(self.fig)
        self.setParent(parent)
        self.setFocusPolicy(QtCore.Qt.FocusPolicy.StrongFocus)

        self._setup_axes()
        self.setMinimumSize(900, 600)

    def _setup_axes(self):
        gs = self.fig.add_gridspec(5, 1, hspace=0.12, left=0.09, right=0.985,
                                   top=0.97, bottom=0.06)

        self.ax1 = self.fig.add_subplot(gs[0])
        self.ax1.set_ylabel("左电机\n(rpm)", fontsize=9, linespacing=1.4)
        self.ax1.grid(True, alpha=0.3, linestyle="--")
        self.line_lt, = self.ax1.plot([], [], "#2196F3", linewidth=1.0, label="期望")
        self.line_la, = self.ax1.plot([], [], "#FF5722", linewidth=1.0,
                                      linestyle="--", dashes=(4, 2), label="实际")
        self.ax1.legend(loc="upper right", fontsize=8, ncol=2,
                        framealpha=0.6, edgecolor="none")
        self.ax1.yaxis.set_major_locator(MaxNLocator(5))

        self.ax2 = self.fig.add_subplot(gs[1], sharex=self.ax1)
        self.ax2.set_ylabel("右电机\n(rpm)", fontsize=9, linespacing=1.4)
        self.ax2.grid(True, alpha=0.3, linestyle="--")
        self.line_rt, = self.ax2.plot([], [], "#2196F3", linewidth=1.0, label="期望")
        self.line_ra, = self.ax2.plot([], [], "#FF5722", linewidth=1.0,
                                      linestyle="--", dashes=(4, 2), label="实际")
        self.ax2.legend(loc="upper right", fontsize=8, ncol=2,
                        framealpha=0.6, edgecolor="none")
        self.ax2.yaxis.set_major_locator(MaxNLocator(5))

        self.ax3 = self.fig.add_subplot(gs[2], sharex=self.ax1)
        self.ax3.set_ylabel("俯仰角\n(°)", fontsize=9, linespacing=1.4)
        self.ax3.grid(True, alpha=0.3, linestyle="--")
        self.line_pitch, = self.ax3.plot([], [], "#4CAF50", linewidth=1.0)
        self.ax3.axhline(y=0, color="#4CAF50", alpha=0.25, linestyle=":", linewidth=0.8)
        self.ax3.yaxis.set_major_locator(MaxNLocator(5))

        self.ax4 = self.fig.add_subplot(gs[3], sharex=self.ax1)
        self.ax4.set_ylabel("左PWM\n(‰)", fontsize=9, linespacing=1.4)
        self.ax4.grid(True, alpha=0.3, linestyle="--")
        self.ax4.set_ylim(-50, 1050)
        self.line_lp, = self.ax4.plot([], [], "#9C27B0", linewidth=1.0)
        self.ax4.yaxis.set_major_locator(MaxNLocator(5))

        self.ax5 = self.fig.add_subplot(gs[4], sharex=self.ax1)
        self.ax5.set_ylabel("右PWM\n(‰)", fontsize=9, linespacing=1.4)
        self.ax5.set_xlabel("时间 (s)")
        self.ax5.grid(True, alpha=0.3, linestyle="--")
        self.ax5.set_ylim(-50, 1050)
        self.line_rp, = self.ax5.plot([], [], "#9C27B0", linewidth=1.0)
        self.ax5.yaxis.set_major_locator(MaxNLocator(5))

        for ax in [self.ax1, self.ax2, self.ax3, self.ax4]:
            ax.label_outer()

        self.all_lines = [
            self.line_lt, self.line_la, self.line_rt, self.line_ra,
            self.line_pitch, self.line_lp, self.line_rp,
        ]
        self.all_axes = [self.ax1, self.ax2, self.ax3, self.ax4, self.ax5]

    def wheelEvent(self, event):
        delta = event.angleDelta().y()
        factor = 0.8 if delta > 0 else 1.25
        self.scroll_changed.emit()
        self._apply_factor(factor)

    def _apply_factor(self, factor):
        pass

    def keyPressEvent(self, event):
        key = event.key()
        if key == QtCore.Qt.Key.Key_Plus or key == QtCore.Qt.Key.Key_Equal:
            factor = 1.25
        elif key == QtCore.Qt.Key.Key_Minus:
            factor = 0.8
        elif key == QtCore.Qt.Key.Key_Home:
            factor = -1.0
        elif key == QtCore.Qt.Key.Key_Space:
            factor = -2.0
        elif key == QtCore.Qt.Key.Key_R:
            factor = -3.0
        elif key == QtCore.Qt.Key.Key_F:
            factor = -4.0
        else:
            super().keyPressEvent(event)
            return
        self.scroll_changed.emit()
        self._apply_factor(factor)


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, buffer, window_sec=10.0):
        super().__init__()
        self.buffer = buffer
        self.window_sec = window_sec
        self.auto_scroll = True
        self._frame_idx = 0
        self._ylim_frozen = False

        self.setWindowTitle("串口实时监控")
        self.setMinimumSize(1000, 650)
        self.resize(1400, 950)

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        self.canvas = RealTimeCanvas()
        self.toolbar = NavigationToolbar2QT(self.canvas, self)
        self.toolbar.setMaximumHeight(36)
        layout.addWidget(self.toolbar)
        layout.addWidget(self.canvas)

        self.status_label = QtWidgets.QLabel()
        self.status_label.setStyleSheet(
            "QLabel { background:#2b2b2b; color:#ccc; padding:3px 8px; font-size:12px; }"
        )
        self.status_bar = self.statusBar()
        self.status_bar.addPermanentWidget(self.status_label)

        self._update_title()

        self.canvas.scroll_changed.connect(self._on_interaction)
        self.canvas._apply_factor = self._apply_factor

        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self._refresh)
        self._timer.start(50)

    def _update_title(self):
        status = "自动" if self.auto_scroll else "暂停"
        self.setWindowTitle(
            f"串口实时监控 [窗口:{self.window_sec:.1f}s|{status}] {HELP_TEXT}"
        )
        self.status_label.setText(
            f"窗口:{self.window_sec:.1f}s | {status}滚动 | "
            f"数据:{self.buffer.count}行"
        )

    def _apply_factor(self, factor):
        if factor == -1.0:
            self.window_sec = 10.0
        elif factor == -2.0:
            self.auto_scroll = not self.auto_scroll
        elif factor == -3.0:
            self._ylim_frozen = False
        elif factor == -4.0:
            self._ylim_frozen = not self._ylim_frozen
        else:
            self.window_sec = max(1.0, min(300.0, self.window_sec * factor))
        self.window_sec = round(self.window_sec, 1)
        self._update_title()

    def _on_interaction(self):
        pass

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

    def _refresh(self):
        ts, pitch, lt, rt, la, ra, lp, rp = self.buffer.snapshot()

        self.canvas.line_lt.set_data(ts, lt)
        self.canvas.line_la.set_data(ts, la)
        self.canvas.line_rt.set_data(ts, rt)
        self.canvas.line_ra.set_data(ts, ra)
        self.canvas.line_pitch.set_data(ts, pitch)
        self.canvas.line_lp.set_data(ts, lp)
        self.canvas.line_rp.set_data(ts, rp)

        if ts and self.auto_scroll:
            t_max = ts[-1]
            t_min = max(0.0, t_max - self.window_sec)
            self.canvas.ax1.set_xlim(t_min, t_max)

        self._frame_idx += 1
        if self._frame_idx % 15 == 0:
            self.status_label.setText(
                f"窗口:{self.window_sec:.1f}s | "
                f"{'自动' if self.auto_scroll else '暂停'}滚动 | "
                f"数据:{self.buffer.count}行"
            )

        if self._frame_idx % 15 == 0 and not self._ylim_frozen and ts:
            self.canvas.ax1.set_ylim(self._smart_ylim(lt, la, margin_frac=0.10))
            self.canvas.ax2.set_ylim(self._smart_ylim(rt, ra, margin_frac=0.10))
            self.canvas.ax3.set_ylim(self._smart_ylim(pitch, margin_frac=0.10))
            if any(v > 1050 for v in lp) or any(v < -50 for v in lp):
                self.canvas.ax4.set_ylim(self._smart_ylim(lp, abs_margin=20))
            if any(v > 1050 for v in rp) or any(v < -50 for v in rp):
                self.canvas.ax5.set_ylim(self._smart_ylim(rp, abs_margin=20))

        self.canvas.fig.canvas.draw_idle()

    def closeEvent(self, event):
        self._timer.stop()
        super().closeEvent(event)


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


def main():
    parser = argparse.ArgumentParser(
        description="串口CSV实时数据图表监控 — PyQt6 + matplotlib")
    parser.add_argument("--port", "-p", help="串口名称，如 COM3")
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

    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion")

    buffer = RingBuffer(maxlen=args.max_points)

    worker = SerialWorker(args.port, args.baud)
    worker.data_ready.connect(buffer.append)

    def on_error(msg):
        QtWidgets.QMessageBox.critical(None, "串口错误", msg)

    def on_finished(count):
        print(f"\n[串口] 已关闭，共接收 {count} 行")

    worker.error_occurred.connect(on_error)
    worker.finished.connect(on_finished)
    worker.start()

    window = MainWindow(buffer, window_sec=args.window)
    window.show()

    try:
        ret = app.exec()
    finally:
        worker.stop()
        worker.wait(2000)
        print("程序已退出。")
    sys.exit(ret)


if __name__ == "__main__":
    main()
