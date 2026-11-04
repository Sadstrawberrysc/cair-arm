# coding=utf-8
"""内部标定求解模块：RealSense Camera → Base；Gemini Camera → ArmTip。"""
from datetime import datetime
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from calibration_samples import load_samples, load_board, pose_to_homogeneous_matrix

ROOT = Path(__file__).resolve().parent
METHODS = {'park': cv2.CALIB_HAND_EYE_PARK, 'tsai': cv2.CALIB_HAND_EYE_TSAI,
           'horaud': cv2.CALIB_HAND_EYE_HORAUD}
DEFAULT_METHOD = 'park' if hasattr(cv2, 'calibrateHandEye') else 'linear'
METHODS['linear'] = None


def inverse(t):
    result = np.eye(4)
    result[:3, :3] = t[:3, :3].T
    result[:3, 3] = -result[:3, :3] @ t[:3, 3]
    return result


def solve_transforms(base_arm, camera_board, method=DEFAULT_METHOD):
    """X = T_Base_Camera, Y = T_ArmTip_Board; T_Base_ArmTip Y = X T_Camera_Board."""
    if len(base_arm) != len(camera_board) or len(base_arm) < 6:
        raise ValueError('眼在手外求解至少需要六组配对位姿')
    relative = np.array([Rotation.from_matrix(base_arm[0][:3, :3].T @ t[:3, :3]).as_rotvec()
                         for t in base_arm[1:]])
    singular = np.linalg.svd(relative, compute_uv=False)
    if singular[1] < 1e-3:
        raise ValueError('缺少非平行旋转轴运动，无法可靠标定')
    if method == 'linear':
        # All pairwise A X = X B, with A=E_j inv(E_i), B=C_j inv(C_i).
        # log(R_A)=R_X log(R_B): solve rotation by proper SVD alignment,
        # then (R_A-I)t_X=R_X t_B-t_A by linear least squares.
        pairs = [(base_arm[j] @ inverse(base_arm[i]), camera_board[j] @ inverse(camera_board[i]))
                 for i in range(len(base_arm)) for j in range(i+1, len(base_arm))]
        a_log = Rotation.from_matrix(np.array([a[:3, :3] for a, b in pairs])).as_rotvec()
        b_log = Rotation.from_matrix(np.array([b[:3, :3] for a, b in pairs])).as_rotvec()
        u, _, vt = np.linalg.svd(a_log.T @ b_log)
        r = u @ np.diag([1., 1., np.linalg.det(u @ vt)]) @ vt
        lhs = np.vstack([a[:3, :3]-np.eye(3) for a, b in pairs])
        rhs = np.concatenate([r @ b[:3, 3]-a[:3, 3] for a, b in pairs])
        t, _, rank, _ = np.linalg.lstsq(lhs, rhs, rcond=None)
        if rank < 3:
            raise ValueError('平移求解秩不足，请补充不同旋转轴的姿态')
    else:
        if not hasattr(cv2, 'calibrateHandEye'):
            raise ValueError('当前 OpenCV 缺少 calibrateHandEye；可使用 --method linear')
        arm_base = [inverse(t) for t in base_arm]
        r, t = cv2.calibrateHandEye([x[:3, :3] for x in arm_base], [x[:3, 3] for x in arm_base],
                                   [x[:3, :3] for x in camera_board], [x[:3, 3] for x in camera_board],
                                   method=METHODS[method])
    if (not np.isfinite(r).all() or not np.isfinite(t).all()
            or np.linalg.norm(r.T @ r-np.eye(3)) > 1e-6 or abs(np.linalg.det(r)-1) > 1e-6):
        raise ValueError(f'{method} 求解未返回有效旋转/平移')
    x = np.eye(4); x[:3, :3] = r; x[:3, 3] = t.ravel()
    boards = [inverse(arm) @ x @ board for arm, board in zip(base_arm, camera_board)]
    center = np.mean([v[:3, 3] for v in boards], axis=0)
    orientation = Rotation.from_matrix(np.array([v[:3, :3] for v in boards])).mean()
    errors_m = np.linalg.norm(np.array([v[:3, 3] for v in boards])-center, axis=1)
    errors_deg = np.degrees((orientation.inv()*Rotation.from_matrix(np.array([v[:3, :3] for v in boards]))).magnitude())
    y = np.eye(4); y[:3, :3] = orientation.as_matrix(); y[:3, 3] = center
    return x, y, {'board_position_rms_mm': float(np.sqrt(np.mean(errors_m**2))*1000),
                  'board_position_max_mm': float(np.max(errors_m)*1000),
                  'board_rotation_rms_deg': float(np.sqrt(np.mean(errors_deg**2))),
                  'board_rotation_max_deg': float(np.max(errors_deg)),
                  'position_error_per_sample_mm': (errors_m*1000).tolist(),
                  'rotation_error_per_sample_deg': errors_deg.tolist(),
                  'motion_rotation_singular_values_rad': singular.tolist()}


def compute(data_dir, config=None, method=DEFAULT_METHOD, *, eye_in_hand=False):
    path = Path(data_dir).expanduser().resolve()
    images, poses = load_samples(path)
    session_path = path/'session.json'
    session = json.loads(session_path.read_text()) if session_path.exists() else None
    if session is not None:
        if not session.get('complete') or session.get('saved_samples') != len(images):
            raise ValueError('采集会话未完成或 saved_samples 与文件数量不符')
        if session.get('target_samples') != len(images):
            raise ValueError('图像数量与目标采集数不符')
    config = Path(config) if config else path/'config.yaml'
    xx, yy, length = load_board(config)
    if session is not None and session.get('checkerboard_args') != {'XX': xx, 'YY': yy, 'L': length}:
        raise ValueError('棋盘参数与本轮采集元数据不一致')
    object_points = np.zeros((xx*yy, 3), np.float32)
    object_points[:, :2] = np.mgrid[0:xx, 0:yy].T.reshape(-1, 2)*length
    objects, corners, arms, accepted, rejected = [], [], [], [], []
    size = None
    hashes = {}
    for image_path in images:
        image = cv2.imread(str(image_path))
        if image is None:
            raise ValueError(f'无法解码图片：{image_path}')
        image_size = image.shape[1], image.shape[0]
        if size is not None and image_size != size:
            raise ValueError('图像尺寸不一致')
        size = image_size
        if session is not None and size != (session['camera']['width'], session['camera']['height']):
            raise ValueError('图像尺寸与相机会话 profile 不一致')
        sample_id = int(image_path.stem)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        found, points = cv2.findChessboardCorners(gray, (xx, yy), None)
        hashes[image_path.name] = hashlib.sha256(image_path.read_bytes()).hexdigest()
        if not found:
            rejected.append(sample_id)
            continue
        refined = cv2.cornerSubPix(gray, points, (5, 5), (-1, -1),
                                  (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, .001))
        objects.append(object_points.copy()); corners.append(refined)
        arms.append(pose_to_homogeneous_matrix(poses[sample_id-1])); accepted.append(sample_id)
    if len(accepted) < 6:
        raise ValueError(f'有效棋盘不足六组：{len(accepted)}，失败编号：{rejected}')
    print(f'配对 {len(images)} 组，角点有效 {len(accepted)} 组；失败编号：{rejected}', flush=True)
    print(f'使用本轮棋盘：{xx}x{yy}，格长 {length} m；图像 {size}', flush=True)
    # Re-estimate standard OpenCV intrinsics from this session. RealSense inverse
    # Brown-Conrady coefficients cannot be copied as forward OpenCV distortion.
    rms, k, distortion, rvecs, tvecs = cv2.calibrateCamera(objects, corners, size, None, None)
    if not all(np.isfinite(v).all() for v in (rms, k, distortion, np.array(rvecs), np.array(tvecs))):
        raise ValueError('相机标定返回非有限值')
    boards, reprojection = [], []
    for obj, pts, rv, tv in zip(objects, corners, rvecs, tvecs):
        board = np.eye(4); board[:3, :3] = cv2.Rodrigues(rv)[0]; board[:3, 3] = tv.ravel()
        boards.append(board)
        projected = cv2.projectPoints(obj, rv, tv, k, distortion)[0]
        reprojection.append(float(np.sqrt(np.mean(np.sum((projected-pts)**2, axis=2)))))
    # 腕部方程：inv(T_base_armtip) T_base_board = T_armtip_camera T_camera_board。
    if eye_in_hand:
        arms = [inverse(arm) for arm in arms]
    transform_key = 'T_armtip_camera' if eye_in_hand else 'T_base_camera'
    board_key = 'T_base_board' if eye_in_hand else 'T_armtip_board'
    x, y, quality = solve_transforms(arms, boards, method)
    comparisons = {}
    for other in METHODS:
        try:
            other_x, _, q = solve_transforms(arms, boards, other)
            comparisons[other] = {transform_key: other_x.tolist(),
                                  'board_position_rms_mm': q['board_position_rms_mm'],
                                  'board_rotation_rms_deg': q['board_rotation_rms_deg']}
        except (ValueError, cv2.error) as exc:
            comparisons[other] = {'error': str(exc)}
    for p in [path/'poses.txt', config, session_path]:
        if p.exists():
            hashes[p.name] = hashlib.sha256(p.read_bytes()).hexdigest()
    return {'schema_version': 1, 'transform': ('wrist_color_optical_to_rm75_armtip' if eye_in_hand
                                                else 'd455_color_optical_to_rm75_base'),
            'transform_convention': 'T_A_B maps coordinates from frame B to frame A',
            'translation_unit': 'm', 'source': 'calibration_solver.py; no empirical correction',
            'computed_at': datetime.now().astimezone().isoformat(),
            'user_confirmed': False, 'independent_validation_recorded': False,
            transform_key: x.tolist(), board_key: y.tolist(),
            'data_dir': str(path), 'input_sha256': hashes,
            'checkerboard_args': {'XX': xx, 'YY': yy, 'L': length},
            'accepted_sample_ids': accepted, 'rejected_sample_ids': rejected,
            'camera': session.get('camera') if session else None,
            'camera_fit': {'model': 'opencv_brown_conrady', 'intrinsics': k.tolist(),
                           'distortion': distortion.ravel().tolist(),
                           'reprojection_rms_px': float(rms), 'per_sample_rms_px': reprojection},
            'hand_eye_method': method, 'fit_consistency': quality, 'method_comparison': comparisons,
            'validation_note': 'Fit residuals use the calibration samples, not independent validation. '
                               'Controller Base/ArmTip reference and board dimensions require physical verification.'}
