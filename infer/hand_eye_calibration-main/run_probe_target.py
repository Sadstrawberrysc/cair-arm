#!/usr/bin/env python3
"""Capture a digital-human snapshot, convert its first point and invoke Robot."""
import argparse
import io
import os
from pathlib import Path
import subprocess
import sys

import numpy as np

from transform_point import DEFAULT_CALIBRATION, load_transform, transform_point

ROOT = Path(__file__).resolve().parents[2]


def signature(path):
    try:
        stat = path.stat()
        return (stat.st_dev, stat.st_ino, stat.st_size, stat.st_mtime_ns, stat.st_ctime_ns)
    except FileNotFoundError:
        return None


def first_point(path, previous):
    current = signature(path)
    if current is None or current == previous:
        raise ValueError("本次没有保存新采样点；请在数字人窗口按 s 保存后再按 q 退出")
    content = path.read_bytes()
    if signature(path) != current:
        raise ValueError("读取期间采样文件发生变化，请重新采集")
    rows = np.loadtxt(io.BytesIO(content), ndmin=2)
    if rows.ndim != 2 or rows.shape[1] != 3 or rows.shape[0] < 2:
        raise ValueError("采样文件必须包含一行法向量和至少一行 xyz")
    if not np.isfinite(rows).all() or np.linalg.norm(rows[0]) < 1e-12:
        raise ValueError("采样文件包含无效数值或零法向量")
    # The first line is the normal, not a target point.
    return rows[1]


def run(args, root=ROOT, runner=subprocess.run):
    camera_dir = root / "infer/Camera_RT"
    snapshot = camera_dir / "artery_path.txt"
    robot = root / "infer/Robot/build/arm_probe_pose"
    tool = root / "infer/Robot/build/rm75_force_calibration.json"
    transform = load_transform(args.calibration)
    if not robot.is_file() or not os.access(str(robot), os.X_OK):
        raise ValueError("请先构建 infer/Robot/build/arm_probe_pose")
    if not tool.is_file():
        raise ValueError("缺少 Robot/build/rm75_force_calibration.json")
    previous = signature(snapshot)
    print("模式：只规划" if args.dry_run else
          "模式：真机 MoveJ，速度 {}；保存并退出数字人后自动执行。".format(args.velocity),
          flush=True)
    print("数字人窗口：按 s 保存本次采样点，按 q 退出后继续。", flush=True)
    result = runner([sys.executable, str(camera_dir / "cliff_demo.py")], cwd=str(camera_dir))
    if result.returncode != 0:
        raise ValueError("数字人程序异常退出，流程终止")
    camera_point = first_point(snapshot, previous)
    base_point = transform_point(camera_point, transform)
    if not np.isfinite(base_point).all():
        raise ValueError("转换后的 Base 位置无效")
    target = ",".join("{:.9f}".format(v) for v in base_point)
    print("first_camera_point_m:", camera_point.tolist(), flush=True)
    print("target_position_base_m:", target, flush=True)
    print("目标姿态使用连接时的当前 Probe TCP 姿态；不添加位置偏置。", flush=True)
    command = [str(robot), "--target-position-base-m", target,
               "--tool-calibration", str(tool), "--velocity", str(args.velocity)]
    if not args.dry_run:
        command += ["--execute", "--confirm-single-movej"]
    return runner(command, cwd=str(root)).returncode


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__ + " 默认速度1，自动执行单次真实 MoveJ。")
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument("--velocity", type=int, choices=range(1, 6), default=1)
    parser.add_argument("--dry-run", action="store_true", help="只读取状态并规划，不发送运动命令")
    return parser


def main():
    args = build_parser().parse_args()
    try:
        return run(args)
    except KeyboardInterrupt:
        print("流程已中断", file=sys.stderr)
        return 130
    except (OSError, ValueError, TypeError) as error:
        print("BLOCKED:", error, file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
