#!/usr/bin/env python3
"""Convert one D455 optical-frame point into the RM75 Base frame."""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Optional, Sequence

import numpy as np


class TransformError(ValueError):
    pass


DEFAULT_CALIBRATION = Path(__file__).with_name("d455_to_rm75_base.json")


def load_transform(path: Path) -> np.ndarray:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise TransformError(f"cannot load calibration {path}: {error}") from error
    if value.get("transform") != "d455_color_optical_to_rm75_base":
        raise TransformError("calibration direction must be D455 optical to RM75 Base")
    if value.get("translation_unit") != "m":
        raise TransformError("calibration translation unit must be metres")
    if not value.get("user_confirmed", False):
        raise TransformError("calibration has not been explicitly confirmed")
    transform = np.asarray(value.get("T_base_camera"), dtype=np.float64)
    if transform.shape != (4, 4) or not np.isfinite(transform).all():
        raise TransformError("T_base_camera must be one finite 4x4 matrix")
    if not np.allclose(transform[3], [0.0, 0.0, 0.0, 1.0], atol=1e-9):
        raise TransformError("T_base_camera has an invalid homogeneous last row")
    rotation = transform[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6):
        raise TransformError("T_base_camera rotation is not orthonormal")
    if not math.isclose(float(np.linalg.det(rotation)), 1.0, abs_tol=1e-6):
        raise TransformError("T_base_camera rotation determinant is not +1")
    return transform


def transform_point(point_camera_m: Sequence[float], transform: np.ndarray) -> np.ndarray:
    point = np.asarray(point_camera_m, dtype=np.float64)
    if point.shape != (3,) or not np.isfinite(point).all():
        raise TransformError("camera point must contain exactly three finite values")
    return transform[:3, :3] @ point + transform[:3, 3]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Convert one D455 optical xyz point to RM75 Base xyz"
    )
    parser.add_argument(
        "--camera-point-m",
        type=float,
        nargs=3,
        metavar=("X", "Y", "Z"),
        required=True,
    )
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        transform = load_transform(args.calibration)
        point_base = transform_point(args.camera_point_m, transform)
    except TransformError as error:
        print(f"BLOCKED: {error}", file=sys.stderr)
        return 2

    formatted = ",".join(f"{value:.6f}" for value in point_base)
    print("source_frame: d455_color_optical")
    print("target_frame: rm75_base")
    print("translation_unit: m")
    print(f'camera_point_m: "{",".join(f"{value:.6f}" for value in args.camera_point_m)}"')
    print(f'target_position_base_m: "{formatted}"')
    print(f'arm_probe_pose_argument: --target-position-base-m "{formatted}"')
    print("independent_validation_recorded: false")
    return 0


if __name__ == "__main__":
    sys.exit(main())
