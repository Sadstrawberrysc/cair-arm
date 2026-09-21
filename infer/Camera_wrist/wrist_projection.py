"""Pure, fail-closed geometry and v1 handoff validation (no hardware imports)."""
import hashlib
import json
from pathlib import Path
import time
import uuid

import numpy as np

SEED_KEY = "robot:wrist:seed:v1"
STATE_CHANNEL = "robot:wrist:state:v1"
FRAME = "gemini305_color_optical"


def boot_id():
    return Path("/proc/sys/kernel/random/boot_id").read_text().strip()


def rigid(value):
    matrix = np.asarray(value, dtype=float)
    if (matrix.shape != (4, 4) or not np.isfinite(matrix).all()
            or not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-8)
            or not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=2e-6)
            or abs(np.linalg.det(matrix[:3, :3]) - 1) > 2e-6):
        raise ValueError("invalid rigid transform")
    return matrix


def calibration(path, key, name):
    raw = Path(path).read_bytes()
    data = json.loads(raw)
    if (data.get("schema_version") != 1 or data.get("translation_unit") != "m"
            or data.get("transform") != name
            or data.get("transform_convention") != "T_A_B maps coordinates from frame B to frame A"):
        raise ValueError("calibration schema/frame/unit mismatch: " + str(path))
    return rigid(data[key]), hashlib.sha256(raw).hexdigest()


def make_seed(point, rotation, hashes, sample_unix_ns, serial):
    point = np.asarray(point, dtype=float)
    transform = np.eye(4)
    transform[:3, :3] = rotation
    rigid(transform)
    if point.shape != (3,) or not np.isfinite(point).all():
        raise ValueError("invalid surface point")
    now = time.monotonic_ns()
    return dict(version=1, session_id=uuid.uuid4().hex, target_id=uuid.uuid4().hex,
                sequence=1, timestamp_monotonic_ns=now, expires_monotonic_ns=now + 60_000_000_000,
                boot_id=boot_id(), sample_unix_ns=int(sample_unix_ns),
                sample_time_basis="snapshot_file_mtime_not_exposure",
                surface_point_base_m=point.tolist(), initial_tool_rotation_base=np.asarray(rotation).tolist(),
                calibration_sha256=hashes, camera_serial=serial, frame="rm75_base",
                length_unit="m", coarse_positioning_succeeded=True,
                independently_validated=False, scope="projection_preview_only")


def validate_seed(seed, hashes, serial, now_ns=None):
    now_ns = time.monotonic_ns() if now_ns is None else now_ns
    if (seed.get("version") != 1 or seed.get("frame") != "rm75_base"
            or seed.get("length_unit") != "m" or seed.get("boot_id") != boot_id()
            or seed.get("camera_serial") != serial or seed.get("calibration_sha256") != hashes
            or seed.get("coarse_positioning_succeeded") is not True
            or seed.get("scope") != "projection_preview_only" or seed.get("sequence") != 1
            or not seed.get("session_id") or not seed.get("target_id")
            or not 0 <= now_ns - seed.get("timestamp_monotonic_ns", 0) < 60_000_000_000
            or not now_ns < seed.get("expires_monotonic_ns", 0)):
        raise ValueError("seed expired or identity/calibration mismatch")
    point = np.asarray(seed["surface_point_base_m"], dtype=float)
    if point.shape != (3,) or not np.isfinite(point).all():
        raise ValueError("invalid seed point")
    return point


def interpolate_pose(states, capture_ns, seed, hashes):
    """Use bracketing feedback only, never reception time or extrapolation."""
    from scipy.spatial.transform import Rotation, Slerp
    def matches(state):
        if (state.get("version") != 1 or state.get("session_id") != seed["session_id"]
                or state.get("target_id") != seed["target_id"] or state.get("boot_id") != boot_id()
                or state.get("calibration_sha256") != hashes or state.get("valid") is not True
                or state.get("frame") != FRAME or state.get("length_unit") != "m"):
            return False
        return True
    values = list(states)
    for a, b in zip(values, values[1:]):
        if not matches(a) or not matches(b):
            continue
        ta, tb = a["timestamp_monotonic_ns"], b["timestamp_monotonic_ns"]
        if (ta <= capture_ns <= tb and 0 < tb-ta <= 50_000_000
                and a["runtime_session_id"] == b["runtime_session_id"]
                and a["sequence"] < b["sequence"]):
            first, second = rigid(a["T_base_camera"]), rigid(b["T_base_camera"])
            alpha = (capture_ns-ta)/(tb-ta)
            result = np.eye(4)
            result[:3, :3] = Slerp([0, 1], Rotation.from_matrix(
                np.stack([first[:3, :3], second[:3, :3]])))(alpha).as_matrix()
            result[:3, 3] = (1-alpha)*first[:3, 3]+alpha*second[:3, 3]
            return result, (tb-ta)/1e6
    raise ValueError("no matching bracketing robot poses within 50 ms")


def project(point_base, T_base_camera, intrinsic, distortion, width, height):
    import cv2
    transform = rigid(T_base_camera)
    point = transform[:3, :3].T @ (np.asarray(point_base)-transform[:3, 3])
    if not np.isfinite(point).all() or point[2] <= 0:
        raise ValueError("target behind camera")
    pixel = cv2.projectPoints(point.reshape(1, 3), np.zeros(3), np.zeros(3),
                             np.asarray(intrinsic, dtype=float), np.asarray(distortion, dtype=float))[0].reshape(2)
    if not np.isfinite(pixel).all() or not (0 <= pixel[0] < width and 0 <= pixel[1] < height):
        raise ValueError("projection outside image; no clamping")
    return pixel, point


def depth_at(depth_m, pixel):
    # Flooring is deliberate: a valid subpixel projection at width-0.1 must
    # not be rounded to the nonexistent width column or silently clamped.
    x, y = np.floor(pixel).astype(int)
    if not (0 <= y < depth_m.shape[0] and 0 <= x < depth_m.shape[1]):
        raise ValueError("depth pixel outside image")
    depth = float(depth_m[y, x])
    if not np.isfinite(depth) or depth <= 0:
        raise ValueError("invalid target depth")
    return depth
