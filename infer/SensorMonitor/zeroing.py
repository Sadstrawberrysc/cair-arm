"""Fixed software tare from a continuous stable window; no tracking of load/drift."""

from __future__ import annotations

from collections import deque
import threading

import numpy as np


class SoftwareZero:
    WINDOW_S = 3.0
    MAX_GAP_S = 0.15
    MAX_STD = np.array([0.5] * 3 + [0.01] * 3)
    MAX_HALF_CHANGE = np.array([0.1] * 3 + [0.005] * 3)

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._window = deque(maxlen=10000)
        self._valid = False
        self._offset = np.zeros(6)
        self._zero_id = 0
        self._revision = 0
        self._last_time = None
        self._last_stream = None
        self._last_sequence = None
        self._evaluate_at = 0
        self._reason = "等待连续稳定数据清零"

    def reset(self, reason: str = "手动重新清零") -> None:
        with self._lock:
            self._window.clear()
            self._valid = False
            self._offset = np.zeros(6)
            self._revision += 1
            self._last_time = self._last_stream = self._last_sequence = None
            self._evaluate_at = 0
            self._reason = reason

    def snapshot(self) -> dict:
        with self._lock:
            duration = ((self._window[-1][0] - self._window[0][0]) / 1e9
                        if len(self._window) > 1 else 0.0)
            return dict(valid=self._valid, offset_si=self._offset.tolist(),
                        zero_id=self._zero_id, revision=self._revision,
                        window_s=self.WINDOW_S, collected_s=min(duration, self.WINDOW_S),
                        status=self._reason)

    def observe(self, values, monotonic_ns: int, stream_id: int, sequence: int):
        """Observe an entire receive batch. Return a new tare event, if stable."""
        array = np.asarray(values, dtype=float)
        if array.ndim != 2 or array.shape[1] != 6 or not len(array):
            return None
        if not np.isfinite(array).all():
            return None
        with self._lock:
            interrupted = (self._last_time is not None and (
                monotonic_ns < self._last_time or stream_id != self._last_stream
                or sequence <= self._last_sequence))
            if interrupted:
                self._window.clear()
                self._valid = False
                self._revision += 1
                self._reason = "数据流已重置，重新采集零偏"
            if self._last_time is not None and monotonic_ns - self._last_time > self.MAX_GAP_S * 1e9:
                # A gap invalidates the candidate window, not a previously fixed tare.
                self._window.clear()
            self._last_time = monotonic_ns
            self._last_stream = stream_id
            self._last_sequence = sequence + len(array) - 1
            if self._valid:
                return None
            self._window.append((monotonic_ns, len(array), array.sum(axis=0),
                                 (array * array).sum(axis=0)))
            cutoff = monotonic_ns - self.WINDOW_S * 1e9
            while len(self._window) > 1 and self._window[1][0] <= cutoff:
                self._window.popleft()
            if (monotonic_ns - self._window[0][0] < self.WINDOW_S * 1e9
                    or monotonic_ns < self._evaluate_at):
                return None
            self._evaluate_at = monotonic_ns + 200_000_000
            count = sum(item[1] for item in self._window)
            if count < 30:
                return None
            total = sum((item[2] for item in self._window), np.zeros(6))
            squared = sum((item[3] for item in self._window), np.zeros(6))
            mean = total / count
            std = np.sqrt(np.maximum(0, (squared - total * mean) / (count - 1)))
            half = (self._window[0][0] + monotonic_ns) / 2
            early = [item for item in self._window if item[0] < half]
            late = [item for item in self._window if item[0] >= half]
            change = np.abs(
                sum((item[2] for item in early), np.zeros(6)) / sum(item[1] for item in early)
                - sum((item[2] for item in late), np.zeros(6)) / sum(item[1] for item in late))
            if np.any(std > self.MAX_STD) or np.any(change > self.MAX_HALF_CHANGE):
                self._reason = "读数仍在变化，等待稳定后清零"
                return None
            self._offset = mean
            self._valid = True
            self._zero_id += 1
            self._revision += 1
            self._reason = "已清零（固定零偏）"
            self._window.clear()
            return dict(offset_si=mean.tolist(), std_si=std.tolist(), zero_id=self._zero_id,
                        stream_id=stream_id, first_sequence=sequence, samples=count,
                        window_s=self.WINDOW_S)
