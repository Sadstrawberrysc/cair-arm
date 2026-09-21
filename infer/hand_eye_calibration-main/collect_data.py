# coding=utf-8
import argparse
from datetime import datetime
from pathlib import Path
import json
import logging,os
import socket
import time
import sys
import numpy as np
import cv2
# import pyrealsense2 as rs  # 原 RealSense 依赖，Gemini 彩色采集不需要

from libs.log_setting import CommonLog
# from libs.auxiliary import create_folder_with_date, get_ip, popup_message  # 原依赖；仅旧 IP 探测时延迟加载

cam0_origin_path = "/home/cair-jacen/uspilot_ctrl-main/infer/hand_eye_calibration-main/data" # 提前建立好的存储照片文件的目录


logger_ = logging.getLogger(__name__)
logger_ = CommonLog(logger_)

# 原采集回调（保留参考）：
# def callback(frame):
#
#     scaling_factor = 2.0
#     global count
#
#     cv_img = cv2.resize(frame, None, fx=scaling_factor, fy=scaling_factor, interpolation=cv2.INTER_AREA)
#     cv2.imshow("Capture_Video", cv_img)  # 窗口显示，显示名为 Capture_Video
#
#     k = cv2.waitKey(30) & 0xFF  # 每帧数据延时 1ms，延时不能为 0，否则读取的结果会是静态帧
#
#     if k == ord('s'):  # 若检测到按键 ‘s’，打印字符串
#
#         socket_command = '{"command": "get_current_arm_state"}'
#         state,pose = send_cmd(client,socket_command)
#         logger_.info(f'获取状态：{"成功" if state else "失败"}，{f"当前位姿为{pose}" if state else None}')
#         if state:
#
#             filename = os.path.join(cam0_origin_path,"poses.txt")
#
#             with open(filename, 'a+') as f:
#                 # 将列表中的元素用空格连接成一行
#                 pose_ = [str(i) for i in pose]
#                 new_line = f'{",".join(pose_)}\n'
#                 # 将新行附加到文件的末尾
#                 f.write(new_line)
#
#             image_path = os.path.join(cam0_origin_path,f"{str(count)}.jpg")
#             cv2.imwrite(image_path , cv_img)
#             logger_.info(f"===采集第{count}次数据！")
#
#         count += 1
#
#     else:
#         pass
#

def callback(frame, preview_only=False):
    """只放大显示；保存原始 BGR 图像，样本成功后才递增编号。"""
    global count
    cv2.imshow("Capture_Video", cv2.resize(frame, None, fx=2, fy=2))
    key = cv2.waitKey(30) & 0xFF
    if key in (ord('q'), ord('Q'), 27):
        return False
    if key != ord('s') or preview_only:
        return True

    state, pose = send_cmd(client, '{"command": "get_current_arm_state"}')
    if not state:
        logger_.error_(f"未保存样本：机器人状态读取失败：{pose}")
        return True
    if len(pose) != 6 or not np.isfinite(pose).all():
        raise ValueError("机器人位姿必须为六个有限数值")
    image_path = Path(cam0_origin_path) / f"{count}.jpg"
    if image_path.exists():
        raise FileExistsError(f"拒绝覆盖样本：{image_path}")
    if not cv2.imwrite(str(image_path), frame):
        raise RuntimeError(f"图像保存失败：{image_path}")
    # 写入失败终止本轮，不继续使用可能不完整的样本目录。
    with open(Path(cam0_origin_path) / "poses.txt", "a", encoding="utf-8") as stream:
        stream.write(",".join(str(value) for value in pose) + "\n")
    logger_.info(f"采集第 {count} 组：{image_path}，原始尺寸 {frame.shape[1]}x{frame.shape[0]}")
    count += 1
    return True


def send_cmd(client, cmd, get_pose=True):
    """
    发送命令到机械臂并可选择性地获取姿态(pose)数据

    参数:
    client: socket客户端连接
    cmd: 要发送的命令字符串或JSON字符串
    get_pose: 是否需要获取pose数据

    返回:
    如果get_pose为True，返回tuple (状态, pose或错误信息)
    如果get_pose为False，返回布尔值表示命令是否成功发送
    """
    client.send(cmd.encode('utf-8'))

    if not get_pose:
        response = client.recv(1024).decode('utf-8')
        logger_.info(f"response:{response}")
        return True

    time.sleep(0.1)
    response = client.recv(4096).decode('utf-8')  # 增大接收缓冲区
    logger_.info(f'response:{response}')

    try:
        decoder = json.JSONDecoder()
        data_list = []
        index = 0
        # 分割并解析所有可能的JSON对象
        while index < len(response):
            try:
                # 跳过空白字符
                while index < len(response) and response[index].isspace():
                    index += 1
                if index >= len(response):
                    break
                obj, idx = decoder.raw_decode(response[index:])
                data_list.append(obj)
                index += idx
            except json.JSONDecodeError as e:
                logger_.error(f"JSON解析错误：{str(e)}")
                break

        # 寻找最后一个包含目标状态的响应
        target_data = None
        for data in reversed(data_list):
            if isinstance(data, dict) and data.get("state") == "current_arm_state":
                target_data = data
                break

        if not target_data:
            return False, "未找到有效的机械臂状态响应"

        # ------------------- 核心修改位置 -------------------
        arm_state = target_data.get("arm_state", {})

        # 兼容读取错误码字段（优先使用 arm_err，若无则尝试 err，默认值为 0）
        err_code = arm_state.get("arm_err", arm_state.get("err", 0))

        # 兼容校验：允许整型 0 或列表 [0]
        if err_code != 0 and err_code != [0]:
            return False, f"机械臂报错: {err_code}"

        # 转换单位
        pose_raw = arm_state["pose"]
        pose_converted = [
            pose_raw[0] / 1000000,  # x: 0.001mm → m
            pose_raw[1] / 1000000,  # y: 0.001mm → m
            pose_raw[2] / 1000000,  # z: 0.001mm → m
            pose_raw[3] / 1000,     # rx: 0.001rad → rad
            pose_raw[4] / 1000,     # ry: 0.001rad → rad
            pose_raw[5] / 1000      # rz: 0.001rad → rad
        ]

        return True, pose_converted

    except json.JSONDecodeError:
        return False, "JSON解析错误"
    except KeyError as e:
        return False, f"响应缺少关键字段: {str(e)}"
    except Exception as e:
        return False, f"处理响应时发生错误: {str(e)}"
#
# 原 RealSense 采集及启动入口（保留参考，不执行）：
# def displayD435():
#
#     pipeline = rs.pipeline()
#     config = rs.config()
#     config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
#
#     try:
#         pipeline.start(config)
#     except Exception as e:
#         logger_.error_(f"相机连接异常：{e}")
#         popup_message("提醒", "相机连接异常")
#
#         sys.exit(1)
#
#     global count
#     count = 1
#
#     logger_.info(f"开始手眼标定程序，当前程序版号V1.0.0")
#
#     try:
#         while True:
#             frames = pipeline.wait_for_frames()
#             color_frame = frames.get_color_frame()
#             if not color_frame:
#                 continue
#
#             color_image = np.asanyarray(color_frame.get_data())
#             callback(color_image)
#
#     finally:
#
#         pipeline.stop()
#         cv2.destroyAllWindows()
#
#
# if __name__ == '__main__':
#
#     robot_ip = get_ip()
#
#
#
#     logger_.info(f'robot_ip:{robot_ip}')
#
#     if robot_ip:
#
#         client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#         client.connect((robot_ip, 8080))
#         socket_command = '{"command":"set_change_work_frame","frame_name":"Base"}'
#         send_cmd(client,socket_command,get_pose = False)
#
#     else:
#
#         popup_message("提醒", "机械臂ip没有ping通")
#         sys.exit(1)
#
#     displayD435()


def displayGemini305(device, preview_only=False, max_frames=0):
    """通过 Gemini UVC 彩色接口采集 MJPG 640x480@30，输出 BGR。"""
    node = Path(device).resolve()
    name_file = Path('/sys/class/video4linux') / node.name / 'name'
    if not name_file.exists() or 'Gemini 305' not in name_file.read_text():
        raise RuntimeError(f"所选节点未识别为 Gemini 305：{device}")
    cap = cv2.VideoCapture(str(node), cv2.CAP_V4L2)
    try:
        if not cap.isOpened():
            raise RuntimeError(f"无法打开 {device}，请确认节点及是否被预览程序占用")
        for prop, value in (
            (cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG')),
            (cv2.CAP_PROP_FRAME_WIDTH, 640),
            (cv2.CAP_PROP_FRAME_HEIGHT, 480),
            (cv2.CAP_PROP_FPS, 30),
            (cv2.CAP_PROP_CONVERT_RGB, 1),
        ):
            if not cap.set(prop, value):
                raise RuntimeError(f"相机不支持请求的采集设置：property={prop}, value={value}")
        frames = 0
        while True:
            ok, frame = cap.read()
            if not ok:
                raise RuntimeError("Gemini 彩色帧读取失败")
            if frame.dtype != np.uint8 or frame.shape != (480, 640, 3):
                raise RuntimeError(f"彩色帧格式不符合预期：{frame.shape}, {frame.dtype}")
            frames += 1
            if frames == 1:
                print(f"Gemini 彩色采集已启动：{device}, BGR 640x480，配置帧率 {cap.get(cv2.CAP_PROP_FPS):g}", flush=True)
            if not callback(frame, preview_only):
                break
            if max_frames and frames >= max_frames:
                break
        print(f"采集结束，共读取 {frames} 帧", flush=True)
    finally:
        cap.release()
        cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description="Gemini 305 手眼标定彩色采集；s 保存，q/Esc 退出")
    parser.add_argument('--camera-device', default='/dev/video16', help='Gemini MJPG 彩色节点或 /dev/v4l/by-id 路径')
    parser.add_argument('--preview-only', action='store_true', help='仅预览，不连接机器人或保存样本')
    parser.add_argument('--max-frames', type=int, default=0, help='仅预览模式读取指定帧数后退出，0 表示不限')
    parser.add_argument('--robot-ip', help='显式指定机器人 IP；省略时沿用原 get_ip 探测')
    parser.add_argument('--output-dir', type=Path, help='新建样本目录，拒绝使用已有目录')
    args = parser.parse_args()
    if args.max_frames < 0 or (args.max_frames and not args.preview_only):
        parser.error('--max-frames 必须非负，且只能在 --preview-only 模式使用')
    global count, client, cam0_origin_path
    count = 1
    if args.preview_only:
        displayGemini305(args.camera_device, True, args.max_frames)
        return

    cam0_origin_path = str(args.output_dir or (
        Path(__file__).resolve().parent / 'data' / datetime.now().strftime('gemini_%Y%m%d_%H%M%S_%f')
    ))
    Path(cam0_origin_path).mkdir(parents=True, exist_ok=False)
    logger_.info(f"本次样本目录：{cam0_origin_path}")
    logger_.warning("请确认机器人位姿参考点和 Base 坐标语义；每次完全静止后再按 s。")
    # 保留原机器人初始化与状态读取流程；仅预览模式不会执行此分支。
    robot_ip = args.robot_ip
    if not robot_ip:
        from libs.auxiliary import get_ip
        robot_ip = get_ip()
    if not robot_ip:
        raise RuntimeError("无法连接原配置中的机器人 IP")
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as connection:
        client = connection
        client.settimeout(3.0)
        client.connect((robot_ip, 8080))
        send_cmd(client, '{"command":"set_change_work_frame","frame_name":"Base"}', get_pose=False)
        displayGemini305(args.camera_device)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("采集已中断")
    except Exception as exc:
        print(f"采集失败：{exc}", file=sys.stderr)
        sys.exit(1)
