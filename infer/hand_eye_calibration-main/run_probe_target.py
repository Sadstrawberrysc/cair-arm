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
INITIAL_STANDOFF_M = 0.050


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
    if rows.ndim != 2 or rows.shape[1] != 3 or rows.shape[0] < 3:
        raise ValueError("短轴姿态需要一行法向量和至少两个采样点")
    if not np.isfinite(rows).all() or np.linalg.norm(rows[0]) < 1e-12:
        raise ValueError("采样文件包含无效数值或零法向量")
    # The first line is the normal, not a target point.
    for point in rows[2:]:
        tangent = point - rows[1]
        length = np.linalg.norm(tangent)
        if not np.isfinite(length):
            raise ValueError("血管切向包含无效数值")
        if length > 1e-6:
            return rows[1], rows[0] / np.linalg.norm(rows[0]), tangent / length
    raise ValueError("采样点重复，无法确定血管走向")


def short_axis_rotation(inward, tangent):
    """Tool-YZ cuts across the projected vessel tangent; -X follows it."""
    if not np.isfinite(inward).all() or not np.isfinite(tangent).all():
        raise ValueError("法向或切向无效")
    nz, nt = np.linalg.norm(inward), np.linalg.norm(tangent)
    if not np.isfinite([nz, nt]).all() or min(nz, nt) < 1e-12:
        raise ValueError("法向或切向为零或无效")
    z = inward / nz
    t = tangent / nt
    x = -(t - np.dot(t, z) * z)
    if np.linalg.norm(x) < 1e-3:
        raise ValueError("血管切向近似平行于表面法向，无法确定短轴姿态")
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    y /= np.linalg.norm(y)
    x = np.cross(y, z)
    return np.column_stack((x, y, z))


def controller_euler(rotation):
    """Return rx, ry, rz with R = Rz(rz) Ry(ry) Rx(rx)."""
    ry = np.arcsin(np.clip(-rotation[2, 0], -1.0, 1.0))
    if abs(np.cos(ry)) > 1e-9:
        rx = np.arctan2(rotation[2, 1], rotation[2, 2])
        rz = np.arctan2(rotation[1, 0], rotation[0, 0])
    else:
        rx = 0.0
        rz = np.arctan2(-rotation[0, 1], rotation[1, 1])
    return np.array([rx, ry, rz])


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
    print("模式：初始位姿预览，不检查控制器逆解或可达性" if args.dry_run else
          "模式：初始位姿 MoveJ_P，速度 {}；保存并退出数字人后自动执行。".format(args.velocity),
          flush=True)
    print("数字人窗口：按 s 保存本次采样点，按 q 退出后继续。", flush=True)
    result = runner([sys.executable, str(camera_dir / "cliff_demo.py")], cwd=str(camera_dir))
    if result.returncode != 0:
        raise ValueError("数字人程序异常退出，流程终止")
    camera_point, outward_camera, tangent_camera = first_point(snapshot, previous)
    surface_point_base = transform_point(camera_point, transform)
    # cliff_demo saves the camera-facing outward normal (camera Z <= 0).
    # Directions use rotation only; +Tool-Z must point into the body.
    inward_base = -(transform[:3, :3] @ outward_camera)
    inward_base /= np.linalg.norm(inward_base)
    # Offset the Probe TCP outward from the sampled surface, before ArmTip conversion.
    base_point = surface_point_base - INITIAL_STANDOFF_M * inward_base
    tangent_base = transform[:3, :3] @ tangent_camera
    rotation = short_axis_rotation(inward_base, tangent_base)
    if not np.isfinite(base_point).all():
        raise ValueError("转换后的 Base 位置无效")
    target = ",".join("{:.9f}".format(v) for v in base_point)
    print("first_camera_point_m:", camera_point.tolist(), flush=True)
    print("surface_point_base_m:", surface_point_base.tolist(), flush=True)
    print("initial_standoff_mm:", INITIAL_STANDOFF_M * 1000.0, flush=True)
    print("target_position_base_m:", target, flush=True)
    normal = ",".join("{:.9f}".format(v) for v in inward_base)
    print("target_inward_normal_base:", normal, flush=True)
    print("target_tool_y_base:", rotation[:, 1].tolist(), flush=True)
    print("target_imaging_view: short_axis", flush=True)
    print("Tool-YZ 为短轴成像平面；-X 沿血管投影切向，+Z 朝内；目标沿外法向后退50 mm。", flush=True)
    pose = ",".join("{:.9f}".format(v) for v in
                    np.concatenate((base_point, controller_euler(rotation))))
    print("target_probe_tcp_m_rad:", pose, flush=True)
    command = [str(robot), "--target-pose-m-rad", pose,
               "--tool-calibration", str(tool), "--velocity", str(args.velocity),
               "--controller-movej-p"]
    if not args.dry_run:
        command += ["--execute", "--confirm-single-movej"]
    return runner(command, cwd=str(root)).returncode


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__ + " 默认速度3，自动执行单次初始位姿 MoveJ_P。")
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument("--velocity", type=int, choices=range(1, 6), default=3)
    parser.add_argument("--dry-run", action="store_true", help="只读取状态并预览目标，不检查可达性、不发送运动命令")
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
