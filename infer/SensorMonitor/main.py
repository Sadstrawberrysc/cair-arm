#!/usr/bin/env python3
"""Read-only standalone/Redis monitor for the RM75 force sensor."""

from __future__ import annotations

import argparse
import errno
import fcntl
import json
import math
import os
import select
import struct
import sys
import termios
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Deque, Dict, Iterable, Tuple

import numpy as np
import pyqtgraph as pg
import redis
from pyqtgraph.Qt import QtCore, QtWidgets


Vector3 = Tuple[float, float, float]
NAN_VECTOR: Vector3 = (math.nan, math.nan, math.nan)
AXES = ("x", "y", "z")
AXIS_COLORS = {
    "x": (220, 50, 47),
    "y": (38, 139, 70),
    "z": (30, 100, 210),
}

HAPTRON_FUNCTION = 0x04
HAPTRON_START_REGISTER = 0x0038
HAPTRON_REGISTER_COUNT = 0x000C
HAPTRON_PAYLOAD_SIZE = 24
HAPTRON_RESPONSE_SIZE = 29


def _modbus_crc16(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc & 0xFFFF


def _build_haptron_request(slave: int) -> bytes:
    request = bytes(
        (
            slave,
            HAPTRON_FUNCTION,
            (HAPTRON_START_REGISTER >> 8) & 0xFF,
            HAPTRON_START_REGISTER & 0xFF,
            (HAPTRON_REGISTER_COUNT >> 8) & 0xFF,
            HAPTRON_REGISTER_COUNT & 0xFF,
        )
    )
    crc = _modbus_crc16(request)
    return request + bytes((crc & 0xFF, (crc >> 8) & 0xFF))


def _extract_haptron_response(
    buffer: bytearray, slave: int
) -> Tuple[float, ...] | None:
    """Consume noise/echo and return one valid six-axis Modbus response."""
    while buffer:
        try:
            start = buffer.index(slave)
        except ValueError:
            buffer.clear()
            return None
        if start:
            del buffer[:start]
        if len(buffer) < 2:
            return None
        function = buffer[1]
        if function == (HAPTRON_FUNCTION | 0x80):
            if len(buffer) < 5:
                return None
            frame = bytes(buffer[:5])
            expected = _modbus_crc16(frame[:-2])
            if frame[-2:] != bytes((expected & 0xFF, (expected >> 8) & 0xFF)):
                del buffer[0]
                continue
            del buffer[:5]
            raise RuntimeError(f"Haptron Modbus exception code={frame[2]}")
        if function != HAPTRON_FUNCTION:
            del buffer[0]
            continue
        if len(buffer) < 3:
            return None
        if buffer[2] != HAPTRON_PAYLOAD_SIZE:
            # This also rejects a locally echoed request (third byte is zero).
            del buffer[0]
            continue
        if len(buffer) < HAPTRON_RESPONSE_SIZE:
            return None
        frame = bytes(buffer[:HAPTRON_RESPONSE_SIZE])
        expected = _modbus_crc16(frame[:-2])
        if frame[-2:] != bytes((expected & 0xFF, (expected >> 8) & 0xFF)):
            del buffer[0]
            continue
        values = struct.unpack(">6f", frame[3 : 3 + HAPTRON_PAYLOAD_SIZE])
        del buffer[:HAPTRON_RESPONSE_SIZE]
        if not all(math.isfinite(value) for value in values):
            raise ValueError("Haptron response contains a non-finite value")
        return tuple(float(value) for value in values)


def _open_direct_sensor(device: str, baud: int) -> int:
    if baud != 115200:
        raise ValueError("standalone monitor currently supports only 115200 baud")
    fd = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK | os.O_CLOEXEC)
    try:
        fcntl.ioctl(fd, termios.TIOCEXCL)
        attributes = termios.tcgetattr(fd)
        attributes[0] = 0
        attributes[1] = 0
        attributes[2] = (
            termios.CLOCAL | termios.CREAD | termios.CS8
        )
        attributes[3] = 0
        attributes[4] = termios.B115200
        attributes[5] = termios.B115200
        attributes[6][termios.VMIN] = 0
        attributes[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, attributes)
        termios.tcflush(fd, termios.TCIFLUSH)
        return fd
    except Exception:
        os.close(fd)
        raise


@dataclass(frozen=True)
class SensorSample:
    sequence: int
    timestamp_s: float
    raw_force_n: Vector3
    raw_torque_nm: Vector3
    compensated_force_n: Vector3
    compensated_torque_nm: Vector3
    contact_point_mm: Vector3
    wrench_valid: bool
    contact_valid: bool
    checksum_valid: bool
    sensor_stale: bool
    io_status: str
    io_error: int
    control_state: str
    fault: str


def _finite_vector(value: object, field: str) -> Vector3:
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError(f"{field} must be a three-element array")
    result = tuple(float(item) for item in value)
    if not all(math.isfinite(item) for item in result):
        raise ValueError(f"{field} contains a non-finite value")
    return result  # type: ignore[return-value]


def parse_sensor_payload(payload: str, fallback_timestamp_s: float) -> SensorSample:
    """Parse one robot:sensor:v1 payload without mutating Redis state."""
    try:
        message = json.loads(payload)
    except json.JSONDecodeError as error:
        raise ValueError(f"invalid JSON: {error.msg}") from error
    if not isinstance(message, dict):
        raise ValueError("payload must be a JSON object")
    if message.get("version") != 1:
        raise ValueError("unsupported sensor protocol version")

    try:
        sequence = int(message["sequence"])
        timestamp_ns = int(message.get("timestamp_monotonic_ns", 0))
        raw = message["raw_wrench_sensor"]
        compensated = message["compensated_wrench_tool"]
        contact = message["contact"]
        sensor = message["sensor"]
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f"missing or invalid required field: {error}") from error

    if sequence < 0:
        raise ValueError("sequence must be non-negative")
    if not all(isinstance(item, dict) for item in (raw, compensated, contact, sensor)):
        raise ValueError("wrench, contact, and sensor fields must be objects")

    raw_force = _finite_vector(raw.get("force_n"), "raw_wrench_sensor.force_n")
    raw_torque = _finite_vector(raw.get("torque_nm"), "raw_wrench_sensor.torque_nm")
    compensated_force = _finite_vector(
        compensated.get("force_n"), "compensated_wrench_tool.force_n"
    )
    compensated_torque = _finite_vector(
        compensated.get("torque_nm"), "compensated_wrench_tool.torque_nm"
    )
    contact_point_m = _finite_vector(contact.get("point_probe_m"), "contact.point_probe_m")

    checksum_valid = bool(sensor.get("checksum_valid", False))
    sensor_stale = bool(sensor.get("stale", True))
    try:
        io_error = int(sensor.get("io_error", 0))
    except (TypeError, ValueError) as error:
        raise ValueError("sensor.io_error must be an integer") from error
    wrench_valid = (
        bool(message.get("valid", False))
        and checksum_valid
        and not sensor_stale
        and io_error == 0
    )
    contact_valid = wrench_valid and bool(contact.get("valid", False))

    timestamp_s = timestamp_ns / 1_000_000_000.0 if timestamp_ns > 0 else fallback_timestamp_s
    return SensorSample(
        sequence=sequence,
        timestamp_s=timestamp_s,
        raw_force_n=raw_force if wrench_valid else NAN_VECTOR,
        raw_torque_nm=raw_torque if wrench_valid else NAN_VECTOR,
        compensated_force_n=compensated_force if wrench_valid else NAN_VECTOR,
        compensated_torque_nm=compensated_torque if wrench_valid else NAN_VECTOR,
        contact_point_mm=(
            tuple(item * 1000.0 for item in contact_point_m)
            if contact_valid
            else NAN_VECTOR
        ),  # type: ignore[arg-type]
        wrench_valid=wrench_valid,
        contact_valid=contact_valid,
        checksum_valid=checksum_valid,
        sensor_stale=sensor_stale,
        io_status=str(sensor.get("io_status", "unknown")),
        io_error=io_error,
        control_state=str(message.get("control_state", "unknown")),
        fault="" if message.get("fault") is None else str(message.get("fault")),
    )


class SampleBuffer:
    """Time-windowed telemetry buffer owned by the GUI thread."""

    def __init__(self, window_s: float) -> None:
        self.window_s = window_s
        self.samples: Deque[SensorSample] = deque()

    def append(self, sample: SensorSample) -> bool:
        reset = bool(
            self.samples
            and (
                sample.sequence <= self.samples[-1].sequence
                or sample.timestamp_s <= self.samples[-1].timestamp_s
            )
        )
        if reset:
            self.samples.clear()
        self.samples.append(sample)
        cutoff = sample.timestamp_s - self.window_s
        while self.samples and self.samples[0].timestamp_s < cutoff:
            self.samples.popleft()
        return reset

    def plot_arrays(self) -> Dict[str, np.ndarray]:
        if not self.samples:
            return {}
        latest_timestamp = self.samples[-1].timestamp_s
        output: Dict[str, np.ndarray] = {
            "time": np.asarray(
                [sample.timestamp_s - latest_timestamp for sample in self.samples],
                dtype=float,
            )
        }
        groups: Iterable[Tuple[str, str]] = (
            ("raw_force", "raw_force_n"),
            ("raw_torque", "raw_torque_nm"),
            ("comp_force", "compensated_force_n"),
            ("comp_torque", "compensated_torque_nm"),
        )
        for prefix, attribute in groups:
            values = np.asarray(
                [getattr(sample, attribute) for sample in self.samples], dtype=float
            )
            for index, axis in enumerate(AXES):
                output[f"{prefix}_{axis}"] = values[:, index]
        return output


class RedisSubscriber(QtCore.QThread):
    sample_received = QtCore.pyqtSignal(object)
    connection_changed = QtCore.pyqtSignal(bool, str)
    payload_error = QtCore.pyqtSignal(str)

    def __init__(self, host: str, port: int, channel: str) -> None:
        super().__init__()
        self.host = host
        self.port = port
        self.channel = channel
        self._stop_event = threading.Event()

    def stop(self) -> None:
        self._stop_event.set()

    def run(self) -> None:
        while not self._stop_event.is_set():
            client = None
            pubsub = None
            try:
                client = redis.Redis(
                    host=self.host,
                    port=self.port,
                    decode_responses=True,
                    socket_connect_timeout=1.0,
                    socket_timeout=1.0,
                    health_check_interval=10,
                )
                client.ping()
                pubsub = client.pubsub(ignore_subscribe_messages=True)
                pubsub.subscribe(self.channel)
                self.connection_changed.emit(
                    True, f"connected to {self.host}:{self.port}/{self.channel}"
                )
                while not self._stop_event.is_set():
                    message = pubsub.get_message(timeout=0.25)
                    if message is None or message.get("type") != "message":
                        continue
                    try:
                        sample = parse_sensor_payload(
                            str(message.get("data", "")), time.monotonic()
                        )
                    except ValueError as error:
                        self.payload_error.emit(str(error))
                        continue
                    self.sample_received.emit(sample)
            except (redis.RedisError, OSError) as error:
                self.connection_changed.emit(False, f"Redis disconnected: {error}")
            finally:
                if pubsub is not None:
                    try:
                        pubsub.close()
                    except redis.RedisError:
                        pass
                if client is not None:
                    try:
                        client.close()
                    except redis.RedisError:
                        pass
            self._stop_event.wait(1.0)
        self.connection_changed.emit(False, "monitor stopped")


class DirectSensorReader(QtCore.QThread):
    """Own `/dev/ttyUSB*` and issue read-only Haptron Modbus queries."""

    sample_received = QtCore.pyqtSignal(object)
    connection_changed = QtCore.pyqtSignal(bool, str)
    payload_error = QtCore.pyqtSignal(str)

    def __init__(
        self,
        device: str,
        baud: int,
        slave: int,
        query_period_ms: int,
        response_timeout_ms: int,
    ) -> None:
        super().__init__()
        self.device = device
        self.baud = baud
        self.slave = slave
        self.query_period_s = query_period_ms / 1000.0
        self.response_timeout_s = response_timeout_ms / 1000.0
        self._stop_event = threading.Event()

    def stop(self) -> None:
        self._stop_event.set()

    @staticmethod
    def _write_request(fd: int, request: bytes, deadline: float) -> None:
        offset = 0
        while offset < len(request):
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise TimeoutError("Haptron Modbus request write timeout")
            _, writable, _ = select.select([], [fd], [], min(remaining, 0.01))
            if not writable:
                continue
            try:
                offset += os.write(fd, request[offset:])
            except BlockingIOError:
                continue

    def _read_response(self, fd: int, deadline: float) -> Tuple[float, ...] | None:
        buffer = bytearray()
        while not self._stop_event.is_set():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return None
            readable, _, _ = select.select([fd], [], [], min(remaining, 0.01))
            if not readable:
                continue
            try:
                chunk = os.read(fd, 512)
            except BlockingIOError:
                continue
            if not chunk:
                continue
            buffer.extend(chunk)
            values = _extract_haptron_response(buffer, self.slave)
            if values is not None:
                return values
        return None

    def run(self) -> None:
        request = _build_haptron_request(self.slave)
        sequence = 1
        while not self._stop_event.is_set():
            fd = -1
            try:
                fd = _open_direct_sensor(self.device, self.baud)
                self.connection_changed.emit(
                    True,
                    f"direct {self.device} at {self.baud} baud "
                    "(exclusive read-only queries)",
                )
                while not self._stop_event.is_set():
                    cycle_started = time.monotonic()
                    deadline = cycle_started + self.response_timeout_s
                    self._write_request(fd, request, deadline)
                    values = self._read_response(fd, deadline)
                    if values is None:
                        termios.tcflush(fd, termios.TCIFLUSH)
                        self.payload_error.emit("Haptron response timeout")
                    else:
                        force = tuple(values[:3])
                        torque = tuple(values[3:])
                        sample = SensorSample(
                            sequence=sequence,
                            timestamp_s=time.monotonic(),
                            raw_force_n=force,  # type: ignore[arg-type]
                            raw_torque_nm=torque,  # type: ignore[arg-type]
                            compensated_force_n=NAN_VECTOR,
                            compensated_torque_nm=NAN_VECTOR,
                            contact_point_mm=NAN_VECTOR,
                            wrench_valid=True,
                            contact_valid=False,
                            checksum_valid=True,
                            sensor_stale=False,
                            io_status="streaming-direct",
                            io_error=0,
                            control_state="standalone",
                            fault="",
                        )
                        sequence += 1
                        self.sample_received.emit(sample)
                        self.payload_error.emit("")
                    remaining = self.query_period_s - (
                        time.monotonic() - cycle_started
                    )
                    if remaining > 0.0:
                        self._stop_event.wait(remaining)
            except (OSError, RuntimeError, TimeoutError, ValueError) as error:
                error_number = error.errno if isinstance(error, OSError) else 0
                if error_number == errno.EBUSY:
                    detail = (
                        f"{self.device} is busy; stop main_rm75 or another "
                        "serial reader first"
                    )
                else:
                    detail = str(error)
                self.connection_changed.emit(False, f"direct sensor error: {detail}")
            finally:
                if fd >= 0:
                    os.close(fd)
            self._stop_event.wait(1.0)
        self.connection_changed.emit(False, "monitor stopped")


class SensorMonitorWindow(QtWidgets.QMainWindow):
    def __init__(self, arguments: argparse.Namespace) -> None:
        super().__init__()
        self.arguments = arguments
        self.buffer = SampleBuffer(arguments.window_sec)
        self.latest_sample: SensorSample | None = None
        self.latest_received_at = 0.0
        self.connection_text = "connecting"
        self.payload_error_text = ""

        direct = arguments.source == "direct"
        self.setWindowTitle(
            "RM75 Direct Force Sensor Monitor (read-only)"
            if direct
            else "RM75 Redis Sensor Monitor (read-only)"
        )
        self.resize(1400, 720)
        self.graphs = pg.GraphicsLayoutWidget()
        self.setCentralWidget(self.graphs)
        force_title = (
            "Raw force: Sensor frame"
            if direct
            else "Force: raw Sensor frame (solid) / compensated Tool frame (dashed)"
        )
        torque_title = (
            "Raw torque: Sensor frame"
            if direct
            else "Torque: raw Sensor frame (solid) / compensated Tool frame (dashed)"
        )
        self.force_plot = self._new_plot(0, force_title, "N")
        self.torque_plot = self._new_plot(1, torque_title, "N·m")
        self.raw_force_curves = self._axis_curves(
            self.force_plot, "Raw sensor F", QtCore.Qt.SolidLine
        )
        self.raw_torque_curves = self._axis_curves(
            self.torque_plot, "Raw sensor T", QtCore.Qt.SolidLine
        )
        self.comp_force_curves = (
            {}
            if direct
            else self._axis_curves(
                self.force_plot, "Comp tool F", QtCore.Qt.DashLine
            )
        )
        self.comp_torque_curves = (
            {}
            if direct
            else self._axis_curves(
                self.torque_plot, "Comp tool T", QtCore.Qt.DashLine
            )
        )

        self.subscriber = (
            DirectSensorReader(
                arguments.device,
                arguments.baud,
                arguments.slave,
                arguments.query_period_ms,
                arguments.response_timeout_ms,
            )
            if direct
            else RedisSubscriber(arguments.host, arguments.port, arguments.channel)
        )
        self.subscriber.sample_received.connect(self._on_sample)
        self.subscriber.connection_changed.connect(self._on_connection)
        self.subscriber.payload_error.connect(self._on_payload_error)
        self.subscriber.start()

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._redraw)
        self.timer.start(round(1000.0 / arguments.refresh_hz))
        self.statusBar().showMessage(
            f"opening {arguments.device} directly"
            if direct
            else "connecting to Redis"
        )

    def _new_plot(self, row: int, title: str, unit: str) -> pg.PlotItem:
        plot = self.graphs.addPlot(row=row, col=0, title=title)
        plot.setLabel("left", unit)
        plot.setLabel("bottom", "Time", units="s")
        plot.getAxis("left").enableAutoSIPrefix(False)
        plot.setXRange(-self.arguments.window_sec, 0.0, padding=0.0)
        plot.showGrid(x=True, y=True, alpha=0.25)
        plot.addLegend(offset=(10, 10))
        plot.enableAutoRange(axis="y", enable=True)
        return plot

    @staticmethod
    def _axis_curves(
        plot: pg.PlotItem, label_prefix: str, line_style: object
    ) -> Dict[str, pg.PlotDataItem]:
        curves: Dict[str, pg.PlotDataItem] = {}
        for axis in AXES:
            color = AXIS_COLORS[axis]
            curves[axis] = plot.plot(
                pen=pg.mkPen(color, width=2.2, style=line_style),
                name=f"{label_prefix}{axis}",
            )
        return curves

    @QtCore.pyqtSlot(object)
    def _on_sample(self, sample: SensorSample) -> None:
        reset = self.buffer.append(sample)
        self.latest_sample = sample
        self.latest_received_at = time.monotonic()
        self.payload_error_text = "stream reset" if reset else ""

    @QtCore.pyqtSlot(bool, str)
    def _on_connection(self, connected: bool, message: str) -> None:
        self.connection_text = ("connected: " if connected else "disconnected: ") + message

    @QtCore.pyqtSlot(str)
    def _on_payload_error(self, message: str) -> None:
        if not message:
            self.payload_error_text = ""
        elif self.arguments.source == "direct":
            self.payload_error_text = f"sensor: {message}"
        else:
            self.payload_error_text = f"payload ignored: {message}"

    def _redraw(self) -> None:
        arrays = self.buffer.plot_arrays()
        if arrays:
            x_values = arrays["time"]
            for axis in AXES:
                self.raw_force_curves[axis].setData(
                    x_values, arrays[f"raw_force_{axis}"], connect="finite"
                )
                self.raw_torque_curves[axis].setData(
                    x_values, arrays[f"raw_torque_{axis}"], connect="finite"
                )
                if self.comp_force_curves:
                    self.comp_force_curves[axis].setData(
                        x_values, arrays[f"comp_force_{axis}"], connect="finite"
                    )
                    self.comp_torque_curves[axis].setData(
                        x_values, arrays[f"comp_torque_{axis}"], connect="finite"
                    )
        self._update_status()

    def _update_status(self) -> None:
        if self.latest_sample is None:
            detail = (
                "waiting for direct Haptron samples"
                if self.arguments.source == "direct"
                else "waiting for robot:sensor:v1"
            )
        else:
            age_s = max(0.0, time.monotonic() - self.latest_received_at)
            sample = self.latest_sample
            detail = (
                f"seq={sample.sequence} age={age_s:.2f}s "
                f"wrench_valid={int(sample.wrench_valid)} "
                f"contact_valid={int(sample.contact_valid)} "
                f"sensor={sample.io_status}/{sample.io_error} "
                f"state={sample.control_state} fault={sample.fault or '-'}"
            )
        suffix = f" | {self.payload_error_text}" if self.payload_error_text else ""
        self.statusBar().showMessage(f"{self.connection_text} | {detail}{suffix}")

    def closeEvent(self, event: object) -> None:
        self.timer.stop()
        self.subscriber.stop()
        if not self.subscriber.wait(2500):
            self.statusBar().showMessage("waiting for sensor monitor worker to stop")
            self.subscriber.wait(1500)
        super().closeEvent(event)  # type: ignore[arg-type]


def _bounded_float(name: str, minimum: float, maximum: float):
    def parse(value: str) -> float:
        try:
            result = float(value)
        except ValueError as error:
            raise argparse.ArgumentTypeError(f"{name} must be numeric") from error
        if not minimum <= result <= maximum:
            raise argparse.ArgumentTypeError(
                f"{name} must be between {minimum:g} and {maximum:g}"
            )
        return result

    return parse


def _redis_sensor_stream_available(
    host: str, port: int, channel: str, timeout_s: float = 0.75
) -> bool:
    """Briefly probe live telemetry; Pub/Sub has no retained last sample."""
    client = None
    pubsub = None
    try:
        client = redis.Redis(
            host=host,
            port=port,
            decode_responses=True,
            socket_connect_timeout=min(timeout_s, 0.5),
            socket_timeout=min(timeout_s, 0.5),
        )
        client.ping()
        pubsub = client.pubsub(ignore_subscribe_messages=True)
        pubsub.subscribe(channel)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            message = pubsub.get_message(timeout=0.1)
            if message is None or message.get("type") != "message":
                continue
            try:
                parse_sensor_payload(str(message.get("data", "")), time.monotonic())
            except ValueError:
                continue
            return True
    except (redis.RedisError, OSError):
        return False
    finally:
        if pubsub is not None:
            try:
                pubsub.close()
            except redis.RedisError:
                pass
        if client is not None:
            try:
                client.close()
            except redis.RedisError:
                pass
    return False


def resolve_source(arguments: argparse.Namespace) -> str:
    """Select Redis or direct without opening a port already owned by RM75."""
    if _redis_sensor_stream_available(
        arguments.host, arguments.port, arguments.channel
    ):
        return "redis"

    # During main_rm75 startup telemetry may not be published yet, but its
    # exclusive serial ownership is already observable. Probe ownership only;
    # do not issue a Modbus request here.
    fd = -1
    try:
        fd = _open_direct_sensor(arguments.device, arguments.baud)
    except OSError as error:
        if error.errno == errno.EBUSY:
            return "redis"
        return "direct"
    except ValueError:
        return "direct"
    finally:
        if fd >= 0:
            os.close(fd)
    return "direct"


def parse_arguments(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Read-only PyQtGraph monitor; uses RM75 Redis telemetry when "
            "available, otherwise it reads the Haptron sensor directly"
        )
    )
    parser.add_argument("--device", default="/dev/ttyUSB0")
    parser.add_argument("--baud", type=int, choices=(115200,), default=115200)
    parser.add_argument("--slave", type=int, default=1)
    parser.add_argument("--query-period-ms", type=int, default=20)
    parser.add_argument("--response-timeout-ms", type=int, default=50)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=7777)
    parser.add_argument("--channel", default="robot:sensor:v1")
    parser.add_argument(
        "--window-sec",
        type=_bounded_float("window-sec", 10.0, 30.0),
        default=20.0,
    )
    parser.add_argument(
        "--refresh-hz",
        type=_bounded_float("refresh-hz", 10.0, 20.0),
        default=15.0,
    )
    arguments = parser.parse_args(argv)
    if not 1 <= arguments.port <= 65535:
        parser.error("--port must be between 1 and 65535")
    if not arguments.channel:
        parser.error("--channel cannot be empty")
    if not 1 <= arguments.slave <= 247:
        parser.error("--slave must be between 1 and 247")
    if not 10 <= arguments.query_period_ms <= 1000:
        parser.error("--query-period-ms must be between 10 and 1000")
    if not 10 <= arguments.response_timeout_ms <= 1000:
        parser.error("--response-timeout-ms must be between 10 and 1000")
    return arguments


def main() -> int:
    arguments = parse_arguments()
    arguments.source = resolve_source(arguments)
    print(f"[SensorMonitor] automatically selected source={arguments.source}")
    pg.setConfigOptions(antialias=False, background="w", foreground="k")
    application = QtWidgets.QApplication(sys.argv)
    application.setApplicationName("RM75 Sensor Monitor")
    window = SensorMonitorWindow(arguments)
    window.show()
    return application.exec()


if __name__ == "__main__":
    raise SystemExit(main())
