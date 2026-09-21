# coding=utf-8
"""离线眼在手上求解：输出 Camera -> ArmTip 的旋转和平移（米）。"""

import argparse
from pathlib import Path
import sys

import cv2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation

from save_poses import pose_to_homogeneous_matrix

ROOT = Path(__file__).resolve().parent


def select_data_dir(data_dir=None):
    if data_dir is not None:
        path = Path(data_dir).expanduser().resolve()
    else:
        # 优先选择新版采集目录；不跳过损坏的新一轮数据去静默求解旧数据。
        candidates = sorted(p for p in (ROOT / "data").glob("gemini_*") if p.is_dir())
        if not candidates:
            candidates = sorted(p for p in (ROOT / "eye_hand_data").glob("data*") if p.is_dir())
        if not candidates:
            raise ValueError("未找到采样目录，请使用 --data-dir 指定含图片和 poses.txt 的目录")
        path = candidates[-1]
    if not path.is_dir():
        raise ValueError(f"采样目录不存在：{path}")
    return path


def load_samples(path):
    images = list(path.glob("*.jpg"))
    if not images:
        raise ValueError(f"目录中没有 JPG 图像：{path}；请检查是否选中了旧的空目录")
    if any(not f.stem.isdigit() or int(f.stem) < 1 for f in images):
        raise ValueError("图像必须以 1.jpg、2.jpg 等正整数编号，编号对应 poses.txt 行号")
    pose_path = path / "poses.txt"
    if not pose_path.is_file():
        raise ValueError(f"缺少机器人位姿文件：{pose_path}")
    poses = np.loadtxt(pose_path, delimiter=",", ndmin=2)
    if poses.shape[1] != 6 or not np.isfinite(poses).all():
        raise ValueError("poses.txt 每行必须包含六个有限数值 x,y,z,rx,ry,rz（m、rad）")
    images.sort(key=lambda f: int(f.stem))
    ids = [int(f.stem) for f in images]
    if ids != list(range(1, len(poses) + 1)):
        raise ValueError("图片编号与位姿行数不一致；需要从 1 开始连续编号且一图一行，不能猜测配对")
    return images, poses


def load_board(config_path):
    with open(config_path, encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict) or not isinstance(config.get("checkerboard_args"), dict):
        raise ValueError("配置缺少 checkerboard_args")
    board = config["checkerboard_args"]
    xx, yy, length = board.get("XX"), board.get("YY"), board.get("L")
    if type(xx) is not int or type(yy) is not int or xx < 2 or yy < 2:
        raise ValueError("XX、YY 必须是大于等于 2 的内部角点数")
    if isinstance(length, bool) or not isinstance(length, (int, float)) or not np.isfinite(length) or length <= 0:
        raise ValueError("L 必须为正的有限格长，单位米")
    return xx, yy, float(length)


def func(data_dir=None, config_path=None):
    path = select_data_dir(data_dir)
    print(f"采样目录：{path}", flush=True)
    images, poses = load_samples(path)
    xx, yy, length = load_board(config_path or ROOT / "config.yaml")
    print(f"样本数：{len(images)}；内部角点：{xx}x{yy}；格长：{length:g} m", flush=True)
    criteria = (cv2.TERM_CRITERIA_MAX_ITER | cv2.TERM_CRITERIA_EPS, 30, 0.001)
    objp = np.zeros((xx * yy, 3), np.float32)
    objp[:, :2] = np.mgrid[0:xx, 0:yy].T.reshape(-1, 2)
    objp *= length
    obj_points, img_points, transforms, accepted, rejected = [], [], [], [], []
    size = None
    for image in images:
        sample_id = int(image.stem)
        img = cv2.imread(str(image))
        if img is None:
            raise ValueError(f"无法解码图像：{image}")
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        image_size = gray.shape[::-1]
        if size is not None and image_size != size:
            raise ValueError(f"图像尺寸不一致：{image.name} 为 {image_size}，预期 {size}")
        size = image_size
        found, corners = cv2.findChessboardCorners(gray, (xx, yy), None)
        if not found:
            rejected.append(sample_id)
            print(f"跳过 {image.name} 及第 {sample_id} 行位姿：棋盘角点检测失败", flush=True)
            continue
        refined = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)
        obj_points.append(objp.copy())
        img_points.append(refined)
        # 保留真实编号对应的位姿，绝不使用“前 N 行”替代成功图片的配对。
        transforms.append(pose_to_homogeneous_matrix(poses[sample_id - 1]))
        accepted.append(sample_id)
    print(f"有效样本 ID：{accepted}；未检测到棋盘的 ID：{rejected}", flush=True)
    if len(accepted) < 3:
        raise ValueError(f"有效棋盘样本不足 3 组（当前 {len(accepted)}）；请检查 XX/YY、图像清晰度和数据目录")
    relative_rotations = np.array([
        Rotation.from_matrix(transforms[0][:3, :3].T @ t[:3, :3]).as_rotvec()
        for t in transforms[1:]
    ])
    if np.linalg.matrix_rank(relative_rotations, tol=1e-6) < 2:
        raise ValueError("有效样本缺少非平行旋转轴的运动，无法可靠求解手眼变换")
    rms, intrinsics, distortion, rvecs, tvecs = cv2.calibrateCamera(
        obj_points, img_points, size, None, None
    )
    if not all(np.isfinite(v).all() for v in (rms, intrinsics, distortion, np.array(rvecs), np.array(tvecs))):
        raise ValueError("相机标定返回非有限数值")
    rotation, translation = cv2.calibrateHandEye(
        [t[:3, :3] for t in transforms],
        [t[:3, 3] for t in transforms],
        rvecs, tvecs,
        method=cv2.CALIB_HAND_EYE_TSAI,
    )
    if not np.isfinite(rotation).all() or not np.isfinite(translation).all():
        raise ValueError("手眼求解返回非有限数值，请检查运动多样性和配对")
    print(f"相机重投影 RMS：{rms:.6f} px", flush=True)
    print("当前为求解结果，尚未经过独立验证，不代表标定验收通过。", flush=True)
    return rotation, translation


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-dir", type=Path, help="本轮采样目录；默认最新 Gemini 目录，无则使用旧 eye_hand_data")
    parser.add_argument("--config", type=Path, default=ROOT / "config.yaml", help="棋盘格配置文件")
    args = parser.parse_args()
    np.set_printoptions(precision=8, suppress=True)
    rotation, translation = func(args.data_dir, args.config)
    print(f"R_armtip_camera：\n{rotation}")
    print(f"t_armtip_camera_m：\n{translation}")
    print(f"四元数 xyzw：\n{Rotation.from_matrix(rotation).as_quat()}")


if __name__ == "__main__":
    try:
        main()
    except (ValueError, OSError, cv2.error) as exc:
        print(f"标定失败：{exc}", file=sys.stderr)
        sys.exit(1)
