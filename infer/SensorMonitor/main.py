#!/usr/bin/env python3
"""KWR57 acquisition, software zero and live six-axis display."""

import argparse
from collections import deque
import fcntl
import os
from pathlib import Path
import select
import signal
import sys
import termios
import threading
import time

if __package__:
    from .kwr57 import Kwr57Parser
    from .recording import SessionRecorder
else:
    from kwr57 import Kwr57Parser
    from recording import SessionRecorder


def open_sensor(device: str, baud: int) -> int:
    fd = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK | os.O_CLOEXEC)
    try:
        fcntl.ioctl(fd, termios.TIOCEXCL)
        attributes = termios.tcgetattr(fd)
        attributes[:4] = [0, 0, termios.CLOCAL | termios.CREAD | termios.CS8, 0]
        attributes[4] = attributes[5] = getattr(termios, f"B{baud}")
        attributes[6][termios.VMIN] = attributes[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, attributes)
        termios.tcflush(fd, termios.TCIFLUSH)
        return fd
    except Exception:
        os.close(fd)
        raise


def write_command(fd: int, command: bytes) -> None:
    deadline = time.monotonic() + 0.2
    while command:
        if time.monotonic() >= deadline:
            raise TimeoutError("传感器命令发送超时")
        if not select.select([], [fd], [], 0.01)[1]:
            continue
        try:
            count = os.write(fd, command)
        except (BlockingIOError, InterruptedError):
            continue
        if count == 0:
            raise OSError("串口写入失败")
        command = command[count:]


class Acquisition(threading.Thread):
    def __init__(self, device, baud, recorder):
        super().__init__(name="kwr57-acquisition")
        self.device, self.baud, self.recorder = device, baud, recorder
        self.stop_event = threading.Event()
        self.samples = deque(maxlen=1000)  # Bounded display history; CSV retains every decoded frame.
        self.lock = threading.Lock()
        self.error = ""
        self.status = "等待数据"

    def run(self):
        fd = None
        try:
            fd = open_sensor(self.device, self.baud)
            parser = Kwr57Parser()
            write_command(fd, b"\x48\xaa\r\n")
            last_frame = time.monotonic()
            next_display = 0.0
            while not self.stop_event.is_set():
                if time.monotonic() - last_frame > 1:
                    raise TimeoutError("超过 1 秒未收到有效数据，采集已停止")
                if not select.select([fd], [], [], 0.05)[0]:
                    continue
                try:
                    data = os.read(fd, 4096)
                except (BlockingIOError, InterruptedError):
                    continue
                if not data:
                    raise OSError("传感器连接已断开")
                frames = parser.feed(data)
                if not frames:
                    continue
                now_ns = time.monotonic_ns()
                last_frame = now_ns / 1e9
                elapsed, values, zero = self.recorder.record_batch(frames, now_ns)
                self.status = f"{zero['status']} | 已记录 {self.recorder.rows_written} 帧"
                if last_frame >= next_display:
                    with self.lock:
                        # Break curves during zero collection and between tare revisions.
                        self.samples.append((elapsed, values if zero["valid"] else [float("nan")] * 6,
                                             zero["revision"]))
                    next_display = last_frame + 0.02
        except Exception as error:
            self.error = str(error)
        finally:
            if fd is not None:
                try:
                    write_command(fd, b"\x43\xaa\r\n")
                except (OSError, TimeoutError) as error:
                    self.error = self.error or f"停流失败：{error}"
                finally:
                    os.close(fd)
            try:
                self.recorder.close()
            except OSError as error:
                self.error = self.error or f"保存失败：{error}"


def run_gui(reader):
    import numpy as np
    import pyqtgraph as pg
    from pyqtgraph.Qt import QtCore, QtWidgets

    class Window(QtWidgets.QMainWindow):
        def __init__(self):
            super().__init__()
            self.setWindowTitle("KWR57 六轴力传感器")
            self.resize(1100, 650)
            graphs = pg.GraphicsLayoutWidget()
            self.setCentralWidget(graphs)
            self.curves = []
            for row, (title, unit, prefix) in enumerate((("清零后力", "N", "F"),
                                                       ("清零后力矩", "N·m", "M"))):
                plot = graphs.addPlot(row=row, col=0, title=title)
                plot.setLabel("left", unit)
                plot.setLabel("bottom", "Time", units="s")
                plot.showGrid(x=True, y=True, alpha=0.25)
                plot.addLegend()
                self.curves.extend(plot.plot(pen=pg.mkPen(color, width=2), name=prefix + axis)
                                   for axis, color in zip("xyz", ("r", "g", "b")))
            toolbar = self.addToolBar("采集")
            toolbar.addAction("重新清零", self.zero)
            self.finish_action = toolbar.addAction("结束采集并出图", self.finish)
            self.timer = QtCore.QTimer(self)
            self.timer.timeout.connect(self.redraw)
            self.timer.start(100)
            reader.start()

        def zero(self):
            if reader.is_alive():
                reader.recorder.zeroing.reset()
                with reader.lock:
                    reader.samples.clear()

        def redraw(self):
            with reader.lock:
                samples = list(reader.samples)
            if samples:
                revision = samples[-1][2]
                samples = [s for s in samples if s[2] == revision and samples[-1][0] - s[0] <= 20]
                times = np.array([s[0] for s in samples])
                values = np.array([s[1] for s in samples])
                for index, curve in enumerate(self.curves):
                    curve.setData(times - times[-1], values[:, index], connect="finite")
            self.statusBar().showMessage(reader.error or reader.status)

        def stop(self):
            reader.stop_event.set()
            reader.join(timeout=3)
            if reader.is_alive():
                self.statusBar().showMessage("正在等待采集保存，请稍后重试")
                return False
            return True

        def finish(self):
            if not self.stop():
                return
            self.finish_action.setEnabled(False)
            self.statusBar().showMessage("正在生成图片…")
            self.process = QtCore.QProcess(self)
            self.process.setProcessChannelMode(QtCore.QProcess.MergedChannels)
            self.process.finished.connect(self.finished)
            self.process.errorOccurred.connect(self.failed)
            self.process.start(sys.executable, [str(Path(__file__).with_name("analyze.py")),
                                                str(reader.recorder.directory)])

        def failed(self, error):
            self.finish_action.setEnabled(True)
            QtWidgets.QMessageBox.warning(self, "出图失败", self.process.errorString())

        def finished(self, code, status):
            self.finish_action.setEnabled(True)
            output = bytes(self.process.readAllStandardOutput()).decode("utf-8", errors="replace")
            if code or status != QtCore.QProcess.NormalExit:
                QtWidgets.QMessageBox.warning(self, "出图失败", output[-2000:])
            else:
                QtWidgets.QMessageBox.information(self, "图片已保存", output.strip())

        def closeEvent(self, event):
            if hasattr(self, "process") and self.process.state() != QtCore.QProcess.NotRunning:
                self.statusBar().showMessage("正在生成图片，请稍后关闭")
                event.ignore()
            elif not self.stop():
                event.ignore()
            else:
                self.timer.stop()
                event.accept()

    pg.setConfigOptions(background="w", foreground="k")
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv[:1])
    window = Window()
    signal.signal(signal.SIGINT, lambda *_: window.close())
    signal.signal(signal.SIGTERM, lambda *_: window.close())
    window.show()
    return app.exec()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="/dev/ttyUSB0")
    parser.add_argument("--baud", type=int, choices=(115200, 460800), default=460800)
    parser.add_argument("--headless", action="store_true", help="无界面记录，Ctrl+C 停止后出图")
    args = parser.parse_args(argv)
    recorder = SessionRecorder()
    reader = Acquisition(args.device, args.baud, recorder)
    print(f"记录目录：{recorder.directory}", flush=True)
    if not args.headless:
        try:
            result = run_gui(reader)
        finally:
            reader.stop_event.set()
            if reader.ident is not None:
                reader.join()
            else:
                recorder.close()
        return result or int(bool(reader.error))
    signal.signal(signal.SIGINT, lambda *_: reader.stop_event.set())
    signal.signal(signal.SIGTERM, lambda *_: reader.stop_event.set())
    reader.start()
    while reader.is_alive():
        reader.join(timeout=0.2)
    if reader.error:
        print(reader.error, file=sys.stderr)
        return 1
    if __package__:
        from .analyze import analyze
    else:
        from analyze import analyze
    try:
        print(analyze(recorder.directory))
    except (ValueError, OSError) as error:
        print(f"出图失败：{error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
