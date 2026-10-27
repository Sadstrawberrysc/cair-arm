#!/usr/bin/env python3
"""Plot zeroed six-axis means with ±1 sample standard deviation per time bin."""

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np

if __package__:
    from .recording import AXIS_FIELDS, RECORDS_ROOT
else:
    from recording import AXIS_FIELDS, RECORDS_ROOT


class Moments:
    def __init__(self):
        self.count = 0
        self.mean = np.zeros(6)
        self.m2 = np.zeros(6)

    def add(self, values):
        self.count += 1
        delta = values - self.mean
        self.mean += delta / self.count
        self.m2 += delta * (values - self.mean)

    def std(self):
        # One observation cannot estimate sample standard deviation.
        return np.sqrt(np.maximum(0, self.m2) / (self.count - 1)) if self.count > 1 else np.full(6, np.nan)


def zeroed_rows(directory):
    """Read current CSV, or prior recordings with explicit software-zero events."""
    current = directory / "samples.csv"
    if current.exists():
        with current.open(newline="", encoding="utf-8") as stream:
            for row in csv.DictReader(stream):
                if not row["zero_id"]:
                    continue
                yield (float(row["elapsed_s"]), int(row["zero_id"]),
                       np.array([float(row[f"zero_{axis}"]) for axis in AXIS_FIELDS]))
        return
    # Preserve the ability to plot existing records; never invent a historical tare.
    events = []
    event_path = directory / "events.jsonl"
    if event_path.exists():
        for line in event_path.read_text(encoding="utf-8").splitlines():
            event = json.loads(line)
            if event["kind"] in ("zero_set", "zero_reset"):
                events.append(event)
    events.sort(key=lambda event: int(event["received_monotonic_ns"]))
    index, zero, origin = 0, None, None
    for path in sorted(directory.glob("samples_*.csv")):
        with path.open(newline="", encoding="utf-8") as stream:
            for row in csv.DictReader(stream):
                timestamp = int(row["received_monotonic_ns"])
                if origin is None:
                    origin = timestamp
                while index < len(events) and int(events[index]["received_monotonic_ns"]) <= timestamp:
                    event = events[index]
                    zero = event["detail"] if event["kind"] == "zero_set" else None
                    index += 1
                if (zero is None or row["valid"] != "1" or row["stale"] != "0"
                        or (row["checksum_available"] == "1" and row["checksum_valid"] != "1")):
                    continue
                yield ((timestamp - origin) / 1e9, int(zero["zero_id"]),
                       np.array([float(row[axis]) for axis in AXIS_FIELDS]) - np.array(zero["offset_si"]))


def analyze(directory: Path, bin_seconds: float = 1.0) -> Path:
    if not math.isfinite(bin_seconds) or bin_seconds <= 0:
        raise ValueError("分箱秒数必须为正有限值")
    bins = {}
    for seconds, zero_id, values in zeroed_rows(directory):
        if seconds < 0 or not math.isfinite(seconds) or not np.isfinite(values).all():
            raise ValueError("记录包含无效时间或六轴数值")
        key = (zero_id, int(seconds // bin_seconds))
        if key not in bins:
            bins[key] = Moments()
        bins[key].add(values)
    if not bins:
        raise ValueError("没有清零后的数据；请保持无外部接触、静止稳定至少 3 秒后继续采集。旧记录需含清零事件。")

    import matplotlib
    matplotlib.use("Agg")
    from matplotlib import pyplot as plt

    figure, axes = plt.subplots(3, 2, figsize=(13, 9), sharex=True)
    zero_ids = sorted({key[0] for key in bins})
    for zero_index, zero_id in enumerate(zero_ids):
        keys = sorted(key[1] for key in bins if key[0] == zero_id)
        times, means, deviations = [], [], []
        previous = None
        for key in keys:
            if previous is not None and key > previous + 1:
                times.append((previous + 1.5) * bin_seconds)
                means.append(np.full(6, np.nan))
                deviations.append(np.full(6, np.nan))
            moment = bins[(zero_id, key)]
            times.append((key + 0.5) * bin_seconds)
            means.append(moment.mean)
            deviations.append(moment.std())
            previous = key
        times, means, deviations = np.array(times), np.array(means), np.array(deviations)
        for axis_index, name in enumerate(("Fx", "Fy", "Fz", "Mx", "My", "Mz")):
            plot = axes[axis_index % 3, axis_index // 3]
            label = "Mean" if len(zero_ids) == 1 else f"Mean (zero {zero_id})"
            bars = plot.errorbar(times, means[:, axis_index], yerr=deviations[:, axis_index],
                                 fmt=".-", ms=3, capsize=2, label=label)
            line = bars.lines[0]
            plot.fill_between(times, means[:, axis_index] - deviations[:, axis_index],
                              means[:, axis_index] + deviations[:, axis_index],
                              color=line.get_color(), alpha=0.22,
                              label="±1 SD" if zero_index == 0 else None)
            plot.set_ylabel(f"{name} ({'N' if axis_index < 3 else 'N m'})")
            plot.grid(alpha=0.3)
            plot.legend(fontsize=8)
    for plot in axes[-1]:
        plot.set_xlabel("Time since recording start (s)")
    figure.suptitle(f"Zeroed readings: mean ±1 sample SD ({bin_seconds:g} s bins)")
    figure.tight_layout()
    output = directory / "analysis" / "mean_std.png"
    output.parent.mkdir(exist_ok=True)
    try:
        figure.savefig(output, dpi=160, bbox_inches="tight")
    finally:
        plt.close(figure)
    return output


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", nargs="?", type=Path, help="默认使用最近一次记录")
    parser.add_argument("--bin-sec", type=float, default=1.0, help="统计窗口秒数，默认 1")
    args = parser.parse_args(argv)
    if args.directory is None:
        sessions = sorted(path for path in RECORDS_ROOT.glob("*") if path.is_dir())
        if not sessions:
            parser.error("没有记录，请先运行 main.py 采集")
        args.directory = sessions[-1]
    try:
        print(analyze(args.directory, args.bin_sec))
    except (OSError, ValueError, KeyError, TypeError) as error:
        parser.exit(1, f"出图失败：{error}\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
