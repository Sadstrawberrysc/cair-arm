"""两部相机共用的采集、会话检查和离线求解编排。"""
import argparse
from datetime import datetime
import json
from pathlib import Path
import sys

import cv2

import collect_data
from calibration_solver import compute, DEFAULT_METHOD, METHODS

ROOT = Path(__file__).resolve().parent


def main(camera, argv=None):
    wrist = camera == 'gemini'
    parser = argparse.ArgumentParser(description=(
        '腕部 Gemini 眼在手上标定' if wrist else '全局 RealSense 眼在手外标定'
    ) + '：默认采集 50 组后自动求解，不发送机器人运动指令、不覆盖部署外参')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--preview-only', action='store_true', help='只预览，不连接机器人或保存数据')
    mode.add_argument('--data-dir', type=Path, help='只求解指定的完整采集目录，不打开硬件')
    parser.add_argument('--output-dir', type=Path, help='采集输出的新目录，拒绝覆盖')
    parser.add_argument('--samples', type=int, default=50)
    parser.add_argument('--robot-ip', default='192.168.50.254')
    parser.add_argument('--robot-port', type=int, default=8080)
    parser.add_argument('--config', type=Path, help='棋盘配置；腕部默认 config_gemini305.yaml，全局默认 config.yaml；离线默认本轮快照')
    parser.add_argument('--method', choices=METHODS, default=DEFAULT_METHOD)
    if wrist:
        parser.add_argument('--camera-device', default='/dev/video16', help='Gemini 彩色 UVC 节点，可用 /dev/v4l/by-id/ 路径')
    else:
        parser.add_argument('--camera-serial', help='全局 RealSense 序列号')
    args = parser.parse_args(argv)
    if args.samples < 6:
        parser.error('--samples 至少为 6；建议 50 组')
    if args.output_dir and (args.data_dir or args.preview_only):
        parser.error('--output-dir 仅用于新采集')
    if args.method != 'linear' and not hasattr(cv2, 'calibrateHandEye'):
        parser.error('当前 OpenCV 缺少 calibrateHandEye，请使用 --method linear')
    if args.data_dir:
        directory = args.data_dir.expanduser().resolve()
    else:
        directory = (args.output_dir or ROOT / 'data' / datetime.now().strftime(
            camera + '_%Y%m%d_%H%M%S_%f')).expanduser().resolve()
        capture_args = ['--camera', camera, '--samples', str(args.samples),
                        '--robot-ip', args.robot_ip, '--robot-port', str(args.robot_port),
                        '--config', str(args.config or ROOT / (
                            'config_gemini305.yaml' if wrist else 'config.yaml'))]
        if wrist:
            capture_args += ['--camera-device', args.camera_device]
        elif args.camera_serial:
            capture_args += ['--camera-serial', args.camera_serial]
        capture_args += ['--preview-only'] if args.preview_only else ['--output-dir', str(directory)]
        print('固定标定板在 Base；移动腕部相机。' if wrist else
              '固定全局相机；标定板牢固安装在 ArmTip 上。', flush=True)
        collect_data.main(capture_args)
        if args.preview_only:
            return
    session = json.loads((directory / 'session.json').read_text(encoding='utf-8'))
    if session.get('camera', {}).get('backend') != camera:
        raise ValueError('采集相机类型与当前标定入口不符')
    if not session.get('complete'):
        raise ValueError(f'采集提前结束或未完成，不执行求解：{directory}')
    result = compute(directory, args.config, args.method, eye_in_hand=wrist)
    result['calibration_type'] = 'eye_in_hand' if wrist else 'eye_to_hand'
    output = directory / ('eye_in_hand_result.json' if wrist else 'eye_to_hand_result.json')
    output.write_text(json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False) + '\n', encoding='utf-8')
    key = 'T_armtip_camera' if wrist else 'T_base_camera'
    print(f'{key}（平移单位 m）：{result[key]}', flush=True)
    print(f"重投影 RMS：{result['camera_fit']['reprojection_rms_px']:.4f} px", flush=True)
    print(f'待独立验证结果：{output}；未更新部署外参。', flush=True)


def run(camera):
    try:
        main(camera)
    except KeyboardInterrupt:
        print('标定已中断，本轮不自动求解。', file=sys.stderr)
        sys.exit(130)
    except Exception as exc:
        print(f'标定失败：{exc}', file=sys.stderr)
        sys.exit(1)
