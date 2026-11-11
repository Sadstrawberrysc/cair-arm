#!/usr/bin/env python3
"""1000 姿态自动静态采集，随后拟合并写回项目力标定。"""
import argparse
import json
import math
from pathlib import Path
import subprocess
import sys

ROBOT = Path(__file__).resolve().parents[1]
BUILD = ROBOT / 'build'


def rpy_deg(rotation):
    roll = math.atan2(rotation[7], rotation[8])
    pitch = math.atan2(-rotation[6], math.hypot(rotation[7], rotation[8]))
    yaw = math.atan2(rotation[3], rotation[0])
    if math.hypot(rotation[7], rotation[8]) < 1e-10:
        roll, yaw = 0., math.atan2(-rotation[1], rotation[4])
    return ','.join(format(math.degrees(v), '.15g') for v in (roll, pitch, yaw))


def vector(values):
    return ','.join(format(value, '.15g') for value in values)


def checkpoint_sample_count(path):
    # The C++ loader validates rows and metadata; here distinguish header-only
    # attempts so they cannot hide an earlier checkpoint with accepted samples.
    with path.open() as source:
        return sum(1 for line in source if line.strip()
                   and not line.lstrip().startswith('#') and not line.startswith('r00,'))


def resolve_resume(args, parser):
    """Select only checkpoints belonging to this output; never silently restart."""
    if args.resume:
        if args.resume == 'latest':
            output = (args.output or BUILD / 'calibration_samples.csv').resolve()
            if output.is_file() and '# capture_complete=true' in output.read_text():
                parser.error(f'该输出已完成采集，无需恢复：{output}')
            checkpoints = [p for p in output.parent.glob(output.name + '.partial.*')
                           if p.is_file() and not p.name.endswith('.json')
                           and Path(str(p) + '.path.json').is_file()]
            if not checkpoints:
                parser.error(f'未找到该输出的断点及路径快照：{output}')
            populated = [p for p in checkpoints if checkpoint_sample_count(p) > 0]
            if not populated:
                parser.error('该输出的断点均未采到样本；请去掉 --resume 启动采集')
            args.resume = max(populated, key=lambda p: p.stat().st_mtime_ns)
            skipped = sum(p.stat().st_mtime_ns > args.resume.stat().st_mtime_ns
                          for p in checkpoints if p not in populated)
            if skipped:
                print(f'跳过 {skipped} 个较新的空断点，保留已有采样数据。', flush=True)
        else:
            args.resume = Path(args.resume).resolve()
        if not args.resume.is_file():
            parser.error(f'断点文件不存在：{args.resume}')
        try:
            snapshot = json.loads(Path(str(args.resume) + '.path.json').read_text())
            poses = int(snapshot['pose_count'])
            mode = 'local' if snapshot['mode'] in (
                'local_seven_joint_sweep', 'static_seven_joint_sweep', 'static_wrist_rotation_grid') else 'workspace'
        except (OSError, ValueError, KeyError, TypeError) as error:
            parser.error(f'无法读取断点路径快照：{error}')
        if args.poses is not None and args.poses != poses:
            parser.error(f'断点总姿态数为 {poses}，不能改为 {args.poses}')
        args.poses = poses
        if snapshot.get('self_collision_guard') and args.resume_workspace:
            parser.error('已筛选的自碰撞路径不能用 --resume-workspace 重排；请重新运行离线筛选')
        if args.path_mode is not None and args.path_mode != mode and not args.resume_workspace:
            parser.error('普通续采不能改变原路径模式')
        args.path_mode = 'workspace' if args.resume_workspace else mode
        if args.minimum_end_effector_z_m is not None:
            saved_height = snapshot.get('workspace_constraints', {}).get('minimum_body_z_m')
            if saved_height is None or not math.isclose(args.minimum_end_effector_z_m, saved_height, abs_tol=1e-9):
                parser.error('续采高度必须与原快照一致；不能修改原路径高度约束')
            args.minimum_end_effector_z_m = None  # Recover saved envelopes; do not regenerate CAD geometry.
        if args.workspace_config and not args.resume_workspace:
            parser.error('普通续采沿用原快照，不能另设 --workspace-config')
        if args.output is None:
            name, separator, _ = str(args.resume).rpartition('.partial.')
            if not separator:
                parser.error('无法从断点文件名推导输出路径，请提供 --output')
            args.output = Path(name)
        accepted = checkpoint_sample_count(args.resume)
        print(f'恢复断点：{args.resume}；已采 {accepted}/{poses}，下一姿态 {accepted + 1}。', flush=True)
    args.output = args.output or BUILD / 'calibration_samples.csv'
    args.poses = args.poses if args.poses is not None else 1000
    args.path_mode = args.path_mode or 'workspace'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot-ip', default='192.168.50.254')
    parser.add_argument('--robot-port', type=int, default=8080)
    parser.add_argument('--sensor-device', default='/dev/ttyUSB0')
    parser.add_argument('--sensor-id', default='KWR57')
    parser.add_argument('--poses', type=int, help='新任务默认1000；续采沿用快照')
    parser.add_argument('--path-mode', choices=['workspace', 'local'], help='新任务默认workspace；续采沿用快照')
    parser.add_argument('--workspace-config', type=Path, help='多区域采样和禁入区配置，Base 坐标单位米')
    parser.add_argument('--minimum-end-effector-z-m', type=float,
                        help='末端组件包络最低 Base Z（米）；续采填写时须与快照一致')
    parser.add_argument('--joint-half-ranges-deg', help='local 模式的七轴半幅')
    parser.add_argument('--wrist-half-ranges-deg', help='仅覆盖 J5/J6/J7 半幅')
    parser.add_argument('--velocity', type=int, default=5)
    parser.add_argument('--capture-timeout-sec', type=int, default=30)
    parser.add_argument('--resume', nargs='?', const='latest', metavar='PARTIAL',
                        help='沿原路径续采；省略文件时选择 --output 对应的最新断点')
    parser.add_argument('--resume-workspace', action='store_true', help='保留已采样本，只规划剩余工作空间目标')
    parser.add_argument('--check-resume', action='store_true', help='仅离线校验断点，不连接机械臂或传感器')
    parser.add_argument('--samples-per-pose', type=int, default=20)
    parser.add_argument('--output', type=Path, help='新任务默认build/calibration_samples.csv；显式断点自动推导')
    parser.add_argument('--calibration', type=Path, default=BUILD / 'rm75_force_calibration.json')
    parser.add_argument('--plan-only', action='store_true', help='仅生成路径，不运动或打开传感器')
    parser.add_argument('--start-joints-deg', help='仅离线规划用的七轴起点；省略则读取当前位置')
    args = parser.parse_args()
    if args.start_joints_deg and not args.plan_only:
        parser.error('--start-joints-deg 仅用于 --plan-only，实际采集始终从当前位置规划')
    if args.resume_workspace and not args.resume:
        parser.error('--resume-workspace 需要 --resume 文件')
    if args.check_resume and (not args.resume or args.plan_only or args.resume_workspace):
        parser.error('--check-resume 需要 --resume，且不能与规划选项同时使用')
    resolve_resume(args, parser)
    if args.minimum_end_effector_z_m is not None:
        if not math.isfinite(args.minimum_end_effector_z_m) or args.path_mode != 'workspace' or args.resume:
            parser.error('末端高度约束需有限高度、workspace 模式的新任务；续采沿用保存的配置')

    geometry = json.loads(args.calibration.read_text())
    sensor_rpy = rpy_deg(geometry['sensor_to_tool']['rotation_row_major'])
    probe_rpy = rpy_deg(geometry.get('probe_to_sensor', {}).get(
        'rotation_row_major', [1., 0., 0., 0., 1., 0., 0., 0., 1.]))
    rotation = geometry['sensor_to_tool']['rotation_row_major']
    translation = geometry['sensor_to_tool']['translation_sensor_to_tool_m']
    offset = [p - t for p, t in zip(geometry['probe_tcp_sensor_m'], translation)]
    tcp_armtip = [sum(rotation[3 * i + j] * offset[j] for j in range(3)) for i in range(3)]
    if args.minimum_end_effector_z_m is not None:
        from force_calibration_geometry import body_spheres
        config = json.loads(args.workspace_config.read_text()) if args.workspace_config else {
            'schema_version': 1, 'frame': 'rm75_base', 'allowed_regions': [],
            'forbidden_boxes': [], 'forbidden_spheres': []}
        config['minimum_body_z_m'] = args.minimum_end_effector_z_m
        config['body_spheres'] = [body for body in body_spheres(ROBOT) if body['joint_index'] == 6]
        config['body_spheres'].append(dict(name='calibrated_probe_tcp', joint_index=6,
                                           center_m=tcp_armtip, radius_m=0.002))
        config['note'] = '末端组件CAD保守球包络，含2mm膨胀；运动段另留5mm高度余量。附件安装沿用URDF预览模型，非实测碰撞认证。'
        config_path = Path(str(args.output) + '.workspace.json')
        config_path.parent.mkdir(parents=True, exist_ok=True)
        config_path.write_text(json.dumps(config, ensure_ascii=False, indent=2) + '\n')
        args.workspace_config = config_path
    capture = [str(BUILD / 'force_sensor_calibration_capture'),
               '--check-resume' if args.check_resume else '--plan-only' if args.plan_only else '--automatic',
               '--robot-ip', args.robot_ip, '--robot-port', str(args.robot_port),
               '--sensor-device', args.sensor_device,
               '--sensor-protocol', 'kwr57-rs422', '--sensor-baud', '460800',
               '--sensor-to-tool-rpy-deg', sensor_rpy,
               '--probe-to-sensor-rpy-deg', probe_rpy,
               '--poses', str(args.poses), '--samples-per-pose', str(args.samples_per_pose),
               '--path-mode', args.path_mode, '--probe-tcp-armtip-m', vector(tcp_armtip),
               '--velocity', str(args.velocity),
               '--capture-timeout-sec', str(args.capture_timeout_sec),
               '--output', str(args.output), '--overwrite']
    if args.start_joints_deg:
        capture += ['--start-joints-deg', args.start_joints_deg]
    if args.resume:
        capture += ['--resume', str(args.resume)]
    if args.resume_workspace:
        capture += ['--resume-workspace']
    if args.workspace_config:
        capture += ['--workspace-config', str(args.workspace_config)]
    if args.wrist_half_ranges_deg:
        capture += ['--wrist-half-ranges-deg', args.wrist_half_ranges_deg]
    if args.joint_half_ranges_deg:
        capture += ['--joint-half-ranges-deg', args.joint_half_ranges_deg]
    print('仅离线校验断点。' if args.check_resume else '仅生成路径。' if args.plan_only else
          f'自动执行 {args.poses} 个静态姿态，采集完成后拟合并部署。', flush=True)
    subprocess.run(capture, check=True)
    if args.plan_only or args.check_resume:
        return
    fit = [str(BUILD / 'force_sensor_calibrate'),
           '--input', str(args.output), '--output', str(args.calibration),
           '--sensor-id', args.sensor_id,
           '--probe-model', geometry['probe_model'],
           '--probe-file', str(ROBOT / 'model' / geometry['probe_model']),
           '--sensor-to-tool-rpy-deg', sensor_rpy,
           '--probe-to-sensor-rpy-deg', probe_rpy,
           '--sensor-to-tool-translation-m', vector(geometry['sensor_to_tool']['translation_sensor_to_tool_m']),
           '--probe-tcp-sensor-m', vector(geometry['probe_tcp_sensor_m'])]
    if geometry.get('tool_chain_verified'):
        fit.append('--tool-chain-verified')
    subprocess.run(fit, check=True)
    print(f'力标定已写入：{args.calibration}', flush=True)


if __name__ == '__main__':
    try:
        main()
    except subprocess.CalledProcessError as error:
        sys.exit(error.returncode)
    except KeyboardInterrupt:
        sys.exit(130)
