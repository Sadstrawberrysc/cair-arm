# coding=utf-8
"""手眼标定采集：默认 RealSense，全程由操作员移动机器人，s 保存，50 组后结束。"""
import argparse
from datetime import datetime
import json
import math
import os
from pathlib import Path
import socket
import sys
import time

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parent
WINDOW = "Hand-eye capture"


def load_board(path):
    with open(path, encoding="utf-8") as stream:
        board = yaml.safe_load(stream)["checkerboard_args"]
    xx, yy, length = board["XX"], board["YY"], board["L"]
    if (type(xx) is not int or type(yy) is not int or min(xx, yy) < 2
            or not isinstance(length, (float, int)) or isinstance(length, bool)
            or not math.isfinite(length) or length <= 0):
        raise ValueError("棋盘参数需为 XX/YY >= 2 的整数及正格长 L（m）")
    return {"XX": xx, "YY": yy, "L": float(length)}


class RobotReader:
    """仅请求状态，不改变工具/工作坐标系，不发送运动命令。"""
    def __init__(self, connection):
        self.connection = connection
        self.buffer = b""

    def read_pose(self):
        self.connection.sendall(b'{"command":"get_current_arm_state"}\r\n')
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            while b"\n" in self.buffer:
                line, self.buffer = self.buffer.split(b"\n", 1)
                if not line.strip():
                    continue
                reply = json.loads(line)
                if reply.get("state") != "current_arm_state":
                    continue
                arm = reply["arm_state"]
                for name in ("arm_err", "sys_err", "err"):
                    if arm.get(name, 0) not in (0, [0]):
                        raise ValueError(f"机器人 {name}: {arm[name]}")
                raw = np.asarray(arm["pose"], dtype=float)
                if raw.shape != (6,) or not np.isfinite(raw).all():
                    raise ValueError("机器人位姿必须为六个有限数值")
                return raw / np.array([1e6, 1e6, 1e6, 1e3, 1e3, 1e3])
            if len(self.buffer) > 65536:
                raise RuntimeError("机器人响应超过长度上限")
            self.connection.settimeout(max(.001, deadline - time.monotonic()))
            chunk = self.connection.recv(4096)
            if not chunk:
                raise ConnectionError("机器人连接已断开")
            self.buffer += chunk
        raise TimeoutError("机器人状态请求超时")


def rotation_from_pose(pose):
    x, y, z = pose[3:]
    cx, sx, cy, sy, cz, sz = math.cos(x), math.sin(x), math.cos(y), math.sin(y), math.cos(z), math.sin(z)
    return np.array([[cz*cy, cz*sy*sx-sz*cx, cz*sy*cx+sz*sx],
                     [sz*cy, sz*sy*sx+cz*cx, sz*sy*cx-cz*sx],
                     [-sy, cy*sx, cy*cx]])


def pose_is_still(before, after):
    relative = rotation_from_pose(before).T @ rotation_from_pose(after)
    angle = math.acos(float(np.clip((np.trace(relative)-1)/2, -1, 1)))
    return np.linalg.norm(before[:3]-after[:3]) <= .0002 and angle <= math.radians(.2)


class RealSenseCamera:
    def __init__(self, serial=None):
        import pyrealsense2 as rs
        self.rs = rs
        devices = list(rs.context().query_devices())
        if serial:
            devices = [d for d in devices if d.get_info(rs.camera_info.serial_number) == serial]
        if len(devices) != 1:
            raise RuntimeError("需恰好一台 RealSense；多台时用 --camera-serial 选择全局相机")
        device = devices[0]
        serial = device.get_info(rs.camera_info.serial_number)
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device(serial)
        # 与 Camera_RT 一样由 SDK 选择彩色默认 profile；保存实际内参和分辨率。
        config.enable_stream(rs.stream.color)
        profile = self.pipeline.start(config)
        try:
            stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
            intr = stream.get_intrinsics()
            self.metadata = {"backend": "realsense", "serial": serial,
                             "name": device.get_info(rs.camera_info.name),
                             "width": intr.width, "height": intr.height, "fps": stream.fps(),
                             "format": str(stream.format()),
                             "intrinsics": {"fx": intr.fx, "fy": intr.fy, "ppx": intr.ppx,
                                            "ppy": intr.ppy, "model": str(intr.model),
                                            "coeffs": list(intr.coeffs)}}
            for _ in range(15):
                self.read()
        except BaseException:
            self.close()
            raise

    def read(self):
        frame = self.pipeline.wait_for_frames(5000).get_color_frame()
        if not frame:
            raise RuntimeError("RealSense 未返回彩色图像")
        image = np.asanyarray(frame.get_data())
        fmt = frame.profile.format()
        if fmt == self.rs.format.rgb8:
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        elif fmt != self.rs.format.bgr8:
            raise RuntimeError(f"不支持的 RealSense 彩色格式：{fmt}")
        return image.copy()

    def close(self):
        self.pipeline.stop()


class GeminiCamera:
    def __init__(self, device):
        node = Path(device).resolve()
        name = Path('/sys/class/video4linux') / node.name / 'name'
        if not name.exists() or 'Gemini 305' not in name.read_text():
            raise RuntimeError(f"所选节点未识别为 Gemini 305：{device}")
        self.cap = cv2.VideoCapture(str(node), cv2.CAP_V4L2)
        try:
            if not self.cap.isOpened():
                raise RuntimeError(f"无法打开 Gemini：{device}")
            for prop, value in ((cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG')),
                                (cv2.CAP_PROP_FRAME_WIDTH, 640), (cv2.CAP_PROP_FRAME_HEIGHT, 480),
                                (cv2.CAP_PROP_FPS, 30), (cv2.CAP_PROP_CONVERT_RGB, 1)):
                if not self.cap.set(prop, value):
                    raise RuntimeError(f"Gemini 不支持设置：{prop}={value}")
            self.metadata = {"backend": "gemini", "device": str(node), "width": 640, "height": 480,
                             "fps": self.cap.get(cv2.CAP_PROP_FPS)}
        except BaseException:
            self.close()
            raise

    def read(self):
        ok, image = self.cap.read()
        if not ok or image.shape != (480, 640, 3):
            raise RuntimeError("Gemini 彩色帧读取失败或尺寸不符")
        return image

    def close(self):
        self.cap.release()


class CaptureSession:
    def __init__(self, directory, target, board, camera, robot_ip):
        self.directory = directory
        directory.mkdir(parents=True, exist_ok=False)
        self.poses = []
        self.target = target
        self.board = board
        self.metadata = {"schema_version": 1, "created_at": datetime.now().astimezone().isoformat(),
                         "target_samples": target, "saved_samples": 0, "complete": False,
                         "camera": camera, "checkerboard_args": board, "robot_ip": robot_ip,
                         "pose_frame": "Base_from_ArmTip (operator must confirm controller reference)",
                         "pose_units": ["m", "m", "m", "rad", "rad", "rad"],
                         "pose_euler_convention": "Rz*Ry*Rx", "config_file": "config.yaml",
                         "pairing": "fresh color between two stationary poses; not hardware synchronized"}
        (directory / 'config.yaml').write_text(yaml.safe_dump({"checkerboard_args": board}), encoding='utf-8')
        self.write_metadata()

    def write_metadata(self):
        temp = self.directory / 'session.json.partial'
        temp.write_text(json.dumps(self.metadata, indent=2, ensure_ascii=False)+'\n', encoding='utf-8')
        os.replace(temp, self.directory / 'session.json')

    def save(self, image, pose):
        if len(self.poses) >= self.target:
            raise ValueError("已达到目标样本数")
        if np.asarray(pose).shape != (6,) or not np.isfinite(pose).all():
            raise ValueError("机器人位姿无效")
        size = (self.metadata['camera']['height'], self.metadata['camera']['width'], 3)
        if image.dtype != np.uint8 or image.shape != size:
            raise ValueError(f"彩色图像需为 uint8 BGR {size}，实际为 {image.shape}")
        found, _ = cv2.findChessboardCorners(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY),
                                           (self.board['XX'], self.board['YY']), None)
        if not found:
            print("未保存：未检测到完整棋盘角点，请调整清晰度、倾角或检查 XX/YY。", flush=True)
            return False
        index = len(self.poses)+1
        image_path = self.directory / f'{index}.jpg'
        temp_image = self.directory / f'{index}.partial.jpg'
        temp_poses = self.directory / 'poses.txt.partial'
        if image_path.exists():
            raise FileExistsError(f"拒绝覆盖样本：{image_path}")
        try:
            if not cv2.imwrite(str(temp_image), image, [cv2.IMWRITE_JPEG_QUALITY, 100]):
                raise RuntimeError("图像保存失败")
            poses = self.poses + [np.asarray(pose).copy()]
            temp_poses.write_text(''.join(','.join(format(v, '.17g') for v in p)+'\n' for p in poses), encoding='utf-8')
            os.replace(temp_image, image_path)
            os.replace(temp_poses, self.directory / 'poses.txt')
        except BaseException:
            image_path.unlink(missing_ok=True)
            temp_image.unlink(missing_ok=True)
            temp_poses.unlink(missing_ok=True)
            raise
        self.poses = poses
        self.metadata['saved_samples'] = index
        self.write_metadata()
        print(f"已保存 {index}/{self.target}：{image_path}", flush=True)
        return True

    def finish(self):
        images = sorted(self.directory.glob('*.jpg'))
        pose_path = self.directory / 'poses.txt'
        rows = pose_path.read_text().splitlines() if pose_path.exists() else []
        expected = {f'{i}.jpg' for i in range(1, len(self.poses)+1)}
        if {p.name for p in images} != expected or len(rows) != len(self.poses):
            raise RuntimeError("采集目录图像/位姿配对不完整，不能作为正式数据")
        self.metadata['complete'] = len(self.poses) == self.target
        self.metadata['ended_at'] = datetime.now().astimezone().isoformat()
        self.write_metadata()
        print(f"{'采集完成' if self.metadata['complete'] else '提前结束，未采满'}：{len(self.poses)}/{self.target}，{self.directory}", flush=True)


def corner_preview(image, board, progress):
    """在副本上绘制角点与状态，原始采集图不叠字或画点。"""
    display = image.copy()
    pattern = (board['XX'], board['YY'])
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCorners(gray, pattern, None)
    if found:
        corners = cv2.cornerSubPix(
            gray, corners, (5, 5), (-1, -1),
            (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, .001))
    if corners is not None:
        cv2.drawChessboardCorners(display, pattern, corners, found)
    detected = len(corners) if corners is not None else 0
    status = 'FOUND' if found else 'NOT FOUND'
    color = (0, 200, 0) if found else (0, 0, 255)
    cv2.putText(display, f'{progress} | s: save (stationary) | q/Esc: quit', (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 200, 0), 1, cv2.LINE_AA)
    cv2.putText(display, f'Corners: {detected}/{pattern[0]*pattern[1]} '
                f'({pattern[0]}x{pattern[1]}) {status}', (10, 50),
                cv2.FONT_HERSHEY_SIMPLEX, .55, color, 1, cv2.LINE_AA)
    return display


def capture(camera, reader, session, max_frames=0, board=None):
    board = session.board if session else board
    if board is None:
        raise ValueError("预览需要棋盘参数")
    frames = 0
    while True:
        image = camera.read()
        progress = f"{len(session.poses)}/{session.target}" if session else 'PREVIEW'
        cv2.imshow(WINDOW, corner_preview(image, board, progress))
        key = cv2.waitKey(20) & 0xff
        frames += 1
        if key in (ord('q'), ord('Q'), 27) or (max_frames and frames >= max_frames):
            return
        if key != ord('s') or session is None:
            continue
        try:
            before = reader.read_pose()
            # 在两次状态读取之间取新图，避免保存按键前的旧预览帧。
            image = camera.read()
            after = reader.read_pose()
        except ValueError as exc:
            print(f'未保存：{exc}', flush=True)
            continue
        if not pose_is_still(before, after):
            print('未保存：采样期间位移超过 0.2 mm 或转角超过 0.2°，请保持静止。', flush=True)
            continue
        if session.save(image, after) and len(session.poses) == session.target:
            return


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--camera', choices=('realsense', 'gemini'), default='realsense')
    parser.add_argument('--camera-serial', help='RealSense 序列号，多台设备时必须指定')
    parser.add_argument('--camera-device', default='/dev/video16', help='Gemini MJPG 彩色节点')
    parser.add_argument('--samples', type=int, default=50, help='成功保存的目标组数，默认 50')
    parser.add_argument('--config', type=Path, default=ROOT / 'config.yaml')
    parser.add_argument('--preview-only', action='store_true', help='不连接机器人或保存数据')
    parser.add_argument('--max-frames', type=int, default=0, help='仅预览时限制帧数')
    parser.add_argument('--robot-ip', default='192.168.50.254')
    parser.add_argument('--robot-port', type=int, default=8080)
    parser.add_argument('--output-dir', type=Path, help='新建目录，拒绝覆盖已有目录')
    args = parser.parse_args(argv)
    if args.samples <= 0 or args.max_frames < 0 or (args.max_frames and not args.preview_only):
        parser.error('--samples 必须为正；--max-frames 非负且仅用于预览')
    if not 1 <= args.robot_port <= 65535:
        parser.error('--robot-port 超出范围')
    if args.camera == 'gemini' and args.camera_serial:
        parser.error('--camera-serial 仅用于 RealSense')
    board = load_board(args.config)
    print(f"棋盘内部角点 {board['XX']}x{board['YY']}，格长 {board['L']} m", flush=True)
    camera = RealSenseCamera(args.camera_serial) if args.camera == 'realsense' else GeminiCamera(args.camera_device)
    session = None
    try:
        print(f'相机：{camera.metadata}', flush=True)
        if args.preview_only:
            capture(camera, None, None, args.max_frames, board=board)
            return
        print('仅请求机器人状态，不发送运动或修改坐标系。请现场确认反馈为 Base 下 ArmTip。\n'
              '人工移动到不同位置/倾角，完全静止后按 s；q/Esc 提前结束。', flush=True)
        with socket.create_connection((args.robot_ip, args.robot_port), timeout=3) as connection:
            reader = RobotReader(connection)
            reader.read_pose()  # 启动先验证连接与状态，再创建数据目录。
            directory = args.output_dir or (ROOT / 'data' / datetime.now().strftime(args.camera+'_%Y%m%d_%H%M%S_%f'))
            session = CaptureSession(directory, args.samples, board, camera.metadata, args.robot_ip)
            try:
                capture(camera, reader, session)
            except KeyboardInterrupt:
                session.finish()
                raise
            session.finish()
    finally:
        camera.close()
        cv2.destroyAllWindows()
