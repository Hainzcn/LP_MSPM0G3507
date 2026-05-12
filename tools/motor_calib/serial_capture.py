"""
serial_capture.py — 串口日志录制工具
=====================================
录制 XDS-UART 串口输出并保存到文件，检测到校准结束标志后可自动停止。
脚本打开串口后等待 --cmd-delay 秒（默认 5s），依次发送 't'、'b'、'c' 启动流程。

用法:
    python serial_capture.py --port COM3
    python serial_capture.py --port COM3 --baud 115200 --out calib_run.txt
    python serial_capture.py --port COM3 --cmd-delay 5     # 等 5s 后自动发 t/b/c
    python serial_capture.py --port COM3 --no-auto-cmd     # 不自动发命令
    python serial_capture.py --port COM3 --no-auto-stop    # 手动 Ctrl+C 停止
    python serial_capture.py --list                        # 列出可用串口

依赖:
    pip install pyserial
"""

import argparse
import datetime
import os
import subprocess
import sys
import threading
import time

# ── 自动停止标志（固件在校准完成时输出此行） ──────────────────────────────────
AUTO_STOP_MARKER = "[cal] calibration complete"


def list_ports():
    try:
        import serial.tools.list_ports
        ports = list(serial.tools.list_ports.comports())
        if not ports:
            print("未发现可用串口。")
        else:
            print(f"{'端口':<12} {'描述':<40} {'硬件ID'}")
            print("-" * 80)
            for p in sorted(ports):
                print(f"{p.device:<12} {p.description:<40} {p.hwid}")
    except ImportError:
        print("需要安装 pyserial：pip install pyserial")
    sys.exit(0)


def make_default_filename():
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    return f"calib_{ts}.txt"


def _send_cmd(ser, io_lock: threading.Lock, cmd: str, label: str,
              retry: int = 3, retry_gap_s: float = 0.25) -> bool:
    """向串口发送单个命令，失败时重试并给出明确错误。"""
    payload = (cmd + "\r\n").encode("ascii", errors="ignore")
    for attempt in range(1, retry + 1):
        try:
            if not ser.is_open:
                print(f"[capture] !!! 串口未打开，无法发送: {label!r}")
                return False
            with io_lock:
                ser.write(payload)
                ser.flush()
            print(f"[capture] >>> 发送命令: {label!r}")
            return True
        except Exception as e:
            print(
                f"[capture] !!! 发送失败({attempt}/{retry}) {label!r}: {e}",
                file=sys.stderr,
            )
            if attempt < retry:
                time.sleep(retry_gap_s)
    return False


def _auto_cmd_thread(ser, io_lock: threading.Lock, delay_s: float,
                     stop_event: threading.Event):
    """
    后台线程：等待 delay_s 后依次发送 't'、'b'、'c'。
    每条命令之间间隔 1s，给固件处理时间。
    stop_event 置位时线程提前退出（用于 Ctrl+C 中断）。
    """
    if stop_event.wait(timeout=delay_s):
        return  # 主循环已退出，不再发命令
    if not _send_cmd(ser, io_lock, "t", "t"):
        return
    if stop_event.wait(timeout=1.0):
        return
    if not _send_cmd(ser, io_lock, "b", "b"):
        return
    if stop_event.wait(timeout=1.0):
        return
    _send_cmd(ser, io_lock, "c", "c")


def _run_analyze_script(log_path: str):
    analyzer = os.path.join(os.path.dirname(__file__), "analyze_calib.py")
    if not os.path.isfile(analyzer):
        print(f"[capture] 未找到分析脚本，跳过：{analyzer}")
        return

    print(f"[capture] 调用分析脚本: {analyzer}")
    try:
        ret = subprocess.run([sys.executable, analyzer, log_path], check=False)
    except Exception as e:
        print(f"[capture] 调用分析脚本失败：{e}", file=sys.stderr)
        return

    if ret.returncode != 0:
        print(f"[capture] 分析脚本退出码: {ret.returncode}", file=sys.stderr)


def capture(port: str, baud: int, out_path: str,
            auto_stop: bool, timeout_s: float,
            auto_cmd: bool, cmd_delay: float):
    try:
        import serial
    except ImportError:
        print("错误：未安装 pyserial，请先执行：pip install pyserial", file=sys.stderr)
        sys.exit(1)

    print(f"[capture] 打开串口 {port}  波特率 {baud}")
    print(f"[capture] 保存至   {os.path.abspath(out_path)}")
    if auto_cmd:
        print(f"[capture] {cmd_delay:.0f}s 后自动发送 't' + 'b' + 'c'")
    if auto_stop:
        print(f"[capture] 检测到 '{AUTO_STOP_MARKER}' 后自动停止")
    else:
        print("[capture] 按 Ctrl+C 手动停止")
    print()

    try:
        ser = serial.Serial(port, baud, timeout=1.0, write_timeout=1.0)
    except serial.SerialException as e:
        print(f"错误：无法打开串口 {port}：{e}", file=sys.stderr)
        sys.exit(1)

    io_lock = threading.Lock()
    stop_event = threading.Event()
    cmd_thread = None
    if auto_cmd:
        cmd_thread = threading.Thread(
            target=_auto_cmd_thread,
            args=(ser, io_lock, cmd_delay, stop_event),
            daemon=True,
        )
        cmd_thread.start()

    line_count = 0
    cal_sample_count = 0
    start_time = time.monotonic()
    last_activity = start_time
    ended_by_marker = False

    try:
        with open(out_path, "w", encoding="utf-8") as fh:
            while True:
                try:
                    with io_lock:
                        raw = ser.readline()
                except serial.SerialException as e:
                    print(f"\n[capture] 串口读取失败：{e}", file=sys.stderr)
                    break
                if not raw:
                    # readline 超时（1s），检查空闲超时
                    if timeout_s > 0 and (time.monotonic() - last_activity) > timeout_s:
                        print(f"\n[capture] 超过 {timeout_s:.0f}s 无数据，自动停止。")
                        break
                    continue

                line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
                fh.write(line + "\n")
                fh.flush()
                line_count += 1
                last_activity = time.monotonic()

                print(line)

                if "[cal]" in line:
                    cal_sample_count += 1

                if auto_stop and AUTO_STOP_MARKER in line:
                    print("\n[capture] 检测到结束标志，1s 后发送 'b'。")
                    time.sleep(1.0)
                    _send_cmd(ser, io_lock, "b", "b  (post-calib stop)")
                    ended_by_marker = True
                    print("[capture] 停止录制。")
                    break

    except KeyboardInterrupt:
        print("\n[capture] 用户中断。")
    finally:
        stop_event.set()   # 通知后台线程退出
        ser.close()
        if cmd_thread:
            cmd_thread.join(timeout=2.0)

    elapsed = time.monotonic() - start_time
    print(f"\n[capture] 共录制 {line_count} 行（含 {cal_sample_count} 条 [cal] 行），"
          f"耗时 {elapsed:.1f}s")
    print(f"[capture] 文件：{os.path.abspath(out_path)}")
    if ended_by_marker:
        _run_analyze_script(os.path.abspath(out_path))


def main():
    parser = argparse.ArgumentParser(
        description="串口日志录制 — 配合 MSPM0G3507 电机校准固件使用"
    )
    parser.add_argument("--port", "-p", help="串口号，例如 COM3 或 /dev/ttyACM0")
    parser.add_argument("--baud", "-b", type=int, default=115200, help="波特率（默认 115200）")
    parser.add_argument(
        "--out", "-o", default=None,
        help="输出文件路径（默认：当前目录下 calib_YYYYMMDD_HHMMSS.txt）"
    )
    parser.add_argument(
        "--cmd-delay", type=float, default=5.0,
        help="打开串口后等待多少秒再自动发送 t/b/c（默认 5s）"
    )
    parser.add_argument(
        "--no-auto-cmd", action="store_true",
        help="禁止自动发送 t/b/c 命令（手动在串口终端操作）"
    )
    parser.add_argument(
        "--no-auto-stop", action="store_true",
        help="禁用自动停止（默认检测到校准完成标志后停止）"
    )
    parser.add_argument(
        "--idle-timeout", type=float, default=30.0,
        help="无数据超时秒数（默认 30s，设 0 禁用）"
    )
    parser.add_argument("--list", "-l", action="store_true", help="列出系统可用串口并退出")
    args = parser.parse_args()

    if args.list:
        list_ports()

    if not args.port:
        parser.error("请指定串口号 --port COM?，或用 --list 查看可用串口")

    out_path = args.out or make_default_filename()
    capture(
        port=args.port,
        baud=args.baud,
        out_path=out_path,
        auto_stop=not args.no_auto_stop,
        timeout_s=args.idle_timeout,
        auto_cmd=not args.no_auto_cmd,
        cmd_delay=args.cmd_delay,
    )


if __name__ == "__main__":
    main()
