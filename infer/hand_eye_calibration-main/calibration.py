"""SDK 采集、线性手眼求解及外参部署；两个相机入口共用。"""
import argparse
from datetime import datetime
from itertools import combinations
import json
from pathlib import Path
import socket
import time

import cv2
import numpy as np
from scipy.spatial.transform import Rotation
import yaml

ROOT = Path(__file__).resolve().parent
OUTPUTS = {
    'realsense': ROOT / 'd455_to_rm75_base.json',
    'gemini': ROOT.parent / 'Camera_wrist/gemini305_to_rm75_armtip.json',
}
CONFIGS = {'realsense': ROOT / 'config.yaml', 'gemini': ROOT / 'config_gemini305.yaml'}


def load_board(path):
    return yaml.safe_load(Path(path).read_text())['checkerboard_args']


def inverse(transform):
    result = np.eye(4)
    result[:3, :3] = transform[:3, :3].T
    result[:3, 3] = -result[:3, :3] @ transform[:3, 3]
    return result


def pose_matrix(pose):
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_euler('xyz', pose[3:]).as_matrix()
    transform[:3, 3] = pose[:3]
    return transform


def solve_hand_eye(arms, boards):
    """E_i Y = X C_i；由所有姿态对的 A X = X B 求解 X。"""
    pairs = [(arms[j] @ inverse(arms[i]), boards[j] @ inverse(boards[i]))
             for i, j in combinations(range(len(arms)), 2)]
    a_log = Rotation.from_matrix(np.array([a[:3, :3] for a, _ in pairs])).as_rotvec()
    b_log = Rotation.from_matrix(np.array([b[:3, :3] for _, b in pairs])).as_rotvec()
    u, _, vt = np.linalg.svd(a_log.T @ b_log)
    rotation = u @ np.diag([1., 1., np.linalg.det(u @ vt)]) @ vt
    lhs = np.vstack([a[:3, :3] - np.eye(3) for a, _ in pairs])
    rhs = np.concatenate([rotation @ b[:3, 3] - a[:3, 3] for a, b in pairs])
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = np.linalg.lstsq(lhs, rhs, rcond=None)[0]
    fitted = [inverse(arm) @ transform @ board for arm, board in zip(arms, boards)]
    positions = np.array([t[:3, 3] for t in fitted])
    errors = np.linalg.norm(positions - positions.mean(axis=0), axis=1) * 1000
    rotations = Rotation.from_matrix(np.array([t[:3, :3] for t in fitted]))
    angles = np.degrees((rotations.mean().inv() * rotations).magnitude())
    return transform, {
        'board_position_rms_mm': float(np.sqrt(np.mean(errors**2))),
        'board_position_max_mm': float(errors.max()),
        'board_rotation_rms_deg': float(np.sqrt(np.mean(angles**2))),
    }


def compute(directory, camera, config=None):
    directory = Path(directory)
    board = load_board(config or directory / 'config.yaml')
    pattern = board['XX'], board['YY']
    obj = np.zeros((pattern[0] * pattern[1], 3), np.float32)
    obj[:, :2] = np.mgrid[:pattern[0], :pattern[1]].T.reshape(-1, 2) * board['L']
    poses = np.loadtxt(directory / 'poses.txt', delimiter=',', ndmin=2)
    objects, corners, arms, sample_ids = [], [], [], []
    for path in sorted(directory.glob('*.jpg'), key=lambda p: int(p.stem)):
        gray = cv2.cvtColor(cv2.imread(str(path)), cv2.COLOR_BGR2GRAY)
        found, points = cv2.findChessboardCorners(gray, pattern, None)
        if found:
            points = cv2.cornerSubPix(gray, points, (5, 5), (-1, -1), (3, 30, .001))
            objects.append(obj.copy())
            corners.append(points)
            arms.append(pose_matrix(poses[int(path.stem) - 1]))
            sample_ids.append(int(path.stem))
    rms, intrinsics, distortion, rvecs, tvecs = cv2.calibrateCamera(
        objects, corners, gray.shape[::-1], None, None)
    boards, per_sample = [], []
    for points, rv, tv in zip(corners, rvecs, tvecs):
        transform = np.eye(4)
        transform[:3, :3] = cv2.Rodrigues(rv)[0]
        transform[:3, 3] = tv.ravel()
        boards.append(transform)
        projected = cv2.projectPoints(obj, rv, tv, intrinsics, distortion)[0]
        residual = projected.reshape(-1, 2) - points.reshape(-1, 2)
        per_sample.append(float(np.sqrt(np.mean(np.sum(residual**2, axis=1)))))
    wrist = camera == 'gemini'
    if wrist:
        arms = [inverse(arm) for arm in arms]
    transform, quality = solve_hand_eye(arms, boards)
    session_path = directory / 'session.json'
    session = json.loads(session_path.read_text()) if session_path.exists() else {}
    return {
        'schema_version': 1,
        'transform': ('gemini305_color_optical_to_rm75_armtip' if wrist
                      else 'd455_color_optical_to_rm75_base'),
        'transform_convention': 'T_A_B maps coordinates from frame B to frame A',
        'translation_unit': 'm',
        'source': 'calibration.py; no empirical correction',
        'computed_at': datetime.now().astimezone().isoformat(),
        'user_confirmed': True,
        'independent_validation_recorded': False,
        'T_armtip_camera' if wrist else 'T_base_camera': transform.tolist(),
        'data_dir': str(directory.resolve()),
        'camera': session.get('camera', {'backend': camera}),
        'checkerboard_args': board,
        'accepted_sample_ids': sample_ids,
        'hand_eye_method': 'linear',
        'camera_fit': {'intrinsics': intrinsics.tolist(), 'distortion': distortion.ravel().tolist(),
                       'reprojection_rms_px': float(rms), 'per_sample_rms_px': per_sample},
        'fit_consistency': quality,
    }


class RealSenseCamera:
    def __init__(self, serial=None):
        import pyrealsense2 as rs
        self.rs = rs
        self.pipeline = rs.pipeline()
        config = rs.config()
        if serial:
            config.enable_device(serial)
        config.enable_stream(rs.stream.color)
        profile = self.pipeline.start(config)
        stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
        intr = stream.get_intrinsics()
        self.metadata = {'backend': 'realsense', 'width': intr.width, 'height': intr.height,
                         'serial': profile.get_device().get_info(rs.camera_info.serial_number)}

    def read(self):
        frame = self.pipeline.wait_for_frames(5000).get_color_frame()
        image = np.asanyarray(frame.get_data())
        if frame.profile.format() == self.rs.format.rgb8:
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        return image.copy()

    def close(self):
        self.pipeline.stop()


class GeminiCamera:
    def __init__(self, serial=None):
        import pyorbbecsdk as ob
        self.context = ob.Context()
        devices = self.context.query_devices()
        device = (devices.get_device_by_serial_number(serial) if serial else
                  next(devices.get_device_by_index(i) for i in range(devices.get_count())
                       if 'Gemini 305' in devices.get_device_name_by_index(i)))
        self.pipeline = ob.Pipeline(device)
        config = ob.Config()
        # 本机 Gemini 单开彩色流无帧，须同时开启深度；标定只用彩色原图。
        for sensor, fmt in ((ob.OBSensorType.COLOR_SENSOR, ob.OBFormat.MJPG),
                            (ob.OBSensorType.DEPTH_SENSOR, ob.OBFormat.Y16)):
            profile = self.pipeline.get_stream_profile_list(sensor).get_video_stream_profile(
                640, 480, fmt, 30)
            config.enable_stream(profile)
        self.pipeline.start(config)
        self.metadata = {'backend': 'gemini', 'transport': 'orbbec_sdk_rgbd',
                         'serial': device.get_device_info().get_serial_number(),
                         'width': 640, 'height': 480, 'fps': 30, 'color_format': 'MJPG'}

    def read(self):
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            frames = self.pipeline.wait_for_frames(1000)
            color = frames.get_color_frame() if frames else None
            if color:
                image = cv2.imdecode(np.frombuffer(color.get_data(), np.uint8), cv2.IMREAD_COLOR)
                if image is None:
                    raise RuntimeError('Gemini 彩色图像解码失败')
                return image
        raise TimeoutError('Gemini 10 秒内未返回彩色帧')

    def close(self):
        self.pipeline.stop()


def read_pose(connection, stream):
    connection.sendall(b'{"command":"get_current_arm_state"}\r\n')
    while True:
        reply = json.loads(stream.readline())
        if reply.get('state') == 'current_arm_state':
            return np.asarray(reply['arm_state']['pose']) / [1e6, 1e6, 1e6, 1e3, 1e3, 1e3]


def capture(camera_type, directory, board, args):
    camera = (GeminiCamera if camera_type == 'gemini' else RealSenseCamera)(args.camera_serial)
    poses = []
    session = {'camera': camera.metadata, 'checkerboard_args': board}
    pattern = board['XX'], board['YY']
    print(f'相机：{camera.metadata}\n棋盘：{pattern}，格长 {board["L"]} m', flush=True)
    print('s 保存当前图像和机器人位姿；q/Esc 结束。', flush=True)
    try:
        with socket.create_connection((args.robot_ip, args.robot_port), timeout=3) as connection:
            with connection.makefile('rb') as stream:
                while len(poses) < args.samples:
                    image = camera.read()
                    display = image.copy()
                    found, corners = cv2.findChessboardCorners(
                        cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), pattern, None)
                    cv2.drawChessboardCorners(display, pattern, corners, found)
                    label = f'{len(poses)}/{args.samples} {"FOUND" if found else "NOT FOUND"} | s:save q:quit'
                    cv2.putText(display, label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                                .55, (0, 200, 0) if found else (0, 0, 255), 1)
                    cv2.imshow('Hand-eye capture', display)
                    key = cv2.waitKey(20) & 0xff
                    if key in (ord('q'), 27):
                        return False
                    if key == ord('s'):
                        image = camera.read()
                        pose = read_pose(connection, stream)
                        index = len(poses) + 1
                        if not cv2.imwrite(str(directory / f'{index}.jpg'), image,
                                           [cv2.IMWRITE_JPEG_QUALITY, 100]):
                            raise OSError('图像保存失败')
                        poses.append(pose)
                        np.savetxt(directory / 'poses.txt', poses, delimiter=',')
                        print(f'已保存 {index}/{args.samples}', flush=True)
        return True
    finally:
        session['saved_samples'] = len(poses)
        (directory / 'session.json').write_text(json.dumps(session, indent=2) + '\n')
        camera.close()
        cv2.destroyAllWindows()


def run(camera, argv=None):
    parser = argparse.ArgumentParser(description='采集后自动求解并部署相机外参')
    parser.add_argument('--samples', type=int, default=50)
    parser.add_argument('--robot-ip', default='192.168.50.254')
    parser.add_argument('--robot-port', type=int, default=8080)
    parser.add_argument('--camera-serial')
    parser.add_argument('--config', type=Path)
    parser.add_argument('--data-dir', type=Path, help='离线重算并部署指定批次')
    args = parser.parse_args(argv)
    directory = args.data_dir
    if directory is None:
        directory = ROOT / 'data' / datetime.now().strftime(camera + '_%Y%m%d_%H%M%S_%f')
        directory.mkdir(parents=True)
        board = load_board(args.config or CONFIGS[camera])
        (directory / 'config.yaml').write_text(yaml.safe_dump({'checkerboard_args': board}))
        if not capture(camera, directory, board, args):
            print(f'采集结束：{directory}')
            return
    result = compute(directory, camera, args.config)
    name = 'eye_in_hand_result.json' if camera == 'gemini' else 'eye_to_hand_result.json'
    payload = json.dumps(result, indent=2, ensure_ascii=False) + '\n'
    (directory / name).write_text(payload)
    OUTPUTS[camera].write_text(payload)
    print(f'重投影 RMS：{result["camera_fit"]["reprojection_rms_px"]:.4f} px')
    print(f'已部署：{OUTPUTS[camera]}\n数据：{directory}')
