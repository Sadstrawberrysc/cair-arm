#!/usr/bin/env python3
"""Read-only Redis monitor for RM75 force, torque, and contact telemetry."""

from __future__ import annotations

import argparse
import json
import math
import sys
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


class SensorMonitorWindow(QtWidgets.QMainWindow):
    def __init__(self, arguments: argparse.Namespace) -> None:
        super().__init__()
        self.arguments = arguments
        self.buffer = SampleBuffer(arguments.window_sec)
        self.latest_sample: SensorSample | None = None
        self.latest_received_at = 0.0
        self.connection_text = "connecting"
        self.payload_error_text = ""

        self.setWindowTitle("RM75 Redis Sensor Monitor (read-only)")
        self.resize(1400, 720)
        self.graphs = pg.GraphicsLayoutWidget()
        self.setCentralWidget(self.graphs)
        self.force_plot = self._new_plot(0, "Compensated force: Tool frame", "N")
        self.torque_plot = self._new_plot(1, "Compensated torque: Tool frame", "N·m")
        self.force_curves = self._axis_curves(self.force_plot, "F")
        self.torque_curves = self._axis_curves(self.torque_plot, "T")

        self.subscriber = RedisSubscriber(
            arguments.host, arguments.port, arguments.channel
        )
        self.subscriber.sample_received.connect(self._on_sample)
        self.subscriber.connection_changed.connect(self._on_connection)
        self.subscriber.payload_error.connect(self._on_payload_error)
        self.subscriber.start()

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._redraw)
        self.timer.start(round(1000.0 / arguments.refresh_hz))
        self.statusBar().showMessage("connecting to Redis")

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
        plot: pg.PlotItem, symbol: str
    ) -> Dict[str, pg.PlotDataItem]:
        curves: Dict[str, pg.PlotDataItem] = {}
        for axis in AXES:
            color = AXIS_COLORS[axis]
            curves[axis] = plot.plot(
                pen=pg.mkPen(color, width=2.2),
                name=f"{symbol}{axis}",
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
        self.payload_error_text = f"payload ignored: {message}"

    def _redraw(self) -> None:
        arrays = self.buffer.plot_arrays()
        if arrays:
            x_values = arrays["time"]
            for axis in AXES:
                self.force_curves[axis].setData(
                    x_values, arrays[f"comp_force_{axis}"], connect="finite"
                )
                self.torque_curves[axis].setData(
                    x_values, arrays[f"comp_torque_{axis}"], connect="finite"
                )
        self._update_status()

    def _update_status(self) -> None:
        if self.latest_sample is None:
            detail = "waiting for robot:sensor:v1"
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
            self.statusBar().showMessage("waiting for Redis subscriber to stop")
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


def parse_arguments(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Read-only PyQtGraph monitor for robot:sensor:v1"
    )
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
    return arguments


def main() -> int:
    arguments = parse_arguments()
    pg.setConfigOptions(antialias=False, background="w", foreground="k")
    application = QtWidgets.QApplication(sys.argv)
    application.setApplicationName("RM75 Sensor Monitor")
    window = SensorMonitorWindow(arguments)
    window.show()
    return application.exec()


if __name__ == "__main__":
    raise SystemExit(main())
