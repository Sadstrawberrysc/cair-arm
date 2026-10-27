"""Record raw and fixed-offset six-axis readings in one CSV."""

import csv
from datetime import datetime
from pathlib import Path
import time
from zoneinfo import ZoneInfo

if __package__:
    from .zeroing import SoftwareZero
else:
    from zeroing import SoftwareZero

RECORDS_ROOT = Path(__file__).resolve().parent / "records"
AXIS_FIELDS = ("fx_n", "fy_n", "fz_n", "mx_nm", "my_nm", "mz_nm")
CSV_FIELDS = ("elapsed_s", "zero_id", *AXIS_FIELDS,
              *(f"zero_{axis}" for axis in AXIS_FIELDS))


class SessionRecorder:
    """Owned by the acquisition thread; GUI only reads the locked zero snapshot."""

    def __init__(self, root: Path = RECORDS_ROOT) -> None:
        stamp = datetime.now(ZoneInfo("Asia/Shanghai")).strftime("%Y%m%d_%H%M%S_%f")
        self.directory = root / stamp
        self.directory.mkdir(parents=True, exist_ok=False)
        self.zeroing = SoftwareZero()
        self.rows_written = 0
        self._origin = None
        self._output = (self.directory / "samples.csv").open("w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._output)
        self._writer.writerow(CSV_FIELDS)
        self._last_flush = time.monotonic()

    def record_batch(self, frames, monotonic_ns: int):
        if self._origin is None:
            self._origin = monotonic_ns
        self.zeroing.observe(frames, monotonic_ns, 0, self.rows_written + 1)
        zero = self.zeroing.snapshot()
        elapsed = (monotonic_ns - self._origin) / 1e9
        for values in frames:
            corrected = ([value - offset for value, offset in zip(values, zero["offset_si"])]
                         if zero["valid"] else [""] * 6)
            self._writer.writerow((f"{elapsed:.9f}", zero["zero_id"] if zero["valid"] else "",
                                   *values, *corrected))
        self.rows_written += len(frames)
        if time.monotonic() - self._last_flush >= 1:
            self._output.flush()
            self._last_flush = time.monotonic()
        return elapsed, corrected, zero

    def close(self) -> None:
        self._output.close()
