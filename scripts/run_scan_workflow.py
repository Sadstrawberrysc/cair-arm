#!/usr/bin/env python3
"""Run the interactive global-camera → wrist → ultrasound workflow."""
import argparse
import fcntl
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
CONDA = Path('/home/cair-jacen/anaconda3')


class WorkflowError(RuntimeError):
    pass


class Workflow:
    def __init__(self, root=ROOT, stop_timeout=30):
        self.root = root
        self.stop_timeout = stop_timeout
        self.children = []

    def start(self, name, command):
        print(f'\n[流程] {name}', flush=True)
        process = subprocess.Popen(command, cwd=self.root, stdin=subprocess.DEVNULL,
                                   start_new_session=True)
        self.children.append((name, process))
        return process

    def wait(self, name, process, companion=None):
        while True:
            # A controller exiting on its own is never a successful handoff.
            if companion is not None and companion.poll() is not None:
                raise WorkflowError(f'{name}期间 Robot 意外退出（{companion.returncode}），流程停止')
            status = process.poll()
            if status is not None:
                if status != 0:
                    raise WorkflowError(f'{name}失败（退出码 {status}），流程停止')
                return
            time.sleep(.1)

    def stop(self, name, process):
        if process.poll() is not None:
            raise WorkflowError(f'{name}已意外退出（{process.returncode}），拒绝切换下一阶段')
        print(f'[流程] 请求停止 {name}，等待退出…', flush=True)
        os.killpg(process.pid, signal.SIGINT)
        try:
            status = process.wait(timeout=self.stop_timeout)
        except subprocess.TimeoutExpired:
            raise WorkflowError(f'{name}未在 {self.stop_timeout} 秒内退出；不会启动下一阶段。'
                                f'请检查设备与进程组 {process.pid}')
        if status != 0:
            raise WorkflowError(f'{name}停止返回异常（{status}），拒绝切换下一阶段')

    def cleanup(self):
        # Signal all owned groups first, so one slow process cannot delay the rest.
        live = [(name, proc) for name, proc in self.children if proc.poll() is None]
        for _, proc in live:
            try:
                os.killpg(proc.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        deadline = time.monotonic() + self.stop_timeout
        for name, proc in live:
            try:
                proc.wait(timeout=max(0, deadline - time.monotonic()))
            except subprocess.TimeoutExpired:
                print(f'[流程] {name}仍未退出，请检查设备与进程组 {proc.pid}；'
                      '进程退出不代表已完成物理静止验收。', file=sys.stderr)

    def run(self, commands):
        try:
            global_camera = self.start('全局定位：窗口按 s 保存、q 退出，或 Esc 保存并退出',
                                       commands['global'])
            self.wait('全局定位', global_camera)
            camera = self.start('腕部相机：定位完成后按 q，随后自动切换超声', commands['wrist_camera'])
            robot = self.start('腕部 Robot：就绪后相机自动选点并跟随', commands['wrist_robot'])
            self.wait('腕部跟随', camera, robot)
            self.stop('腕部 Robot', robot)
            robot = self.start('超声 Robot：等待悬空 tare 与预检完成', commands['robot'])
            ultrasound = self.start('超声窗口：Robot 就绪后按 b 接近、m 扫描；完成后按 q',
                                    commands['ultrasound'])
            self.wait('超声扫描', ultrasound, robot)
            self.stop('超声 Robot', robot)
            print('[流程] 超声程序与 Robot 均已退出，本次流程结束。', flush=True)
        finally:
            self.cleanup()


def commands():
    # exec keeps the Python process as the session leader, including MoveJ children.
    def python_in(env, script, directory=ROOT):
        return ['bash', '-c', 'source "$1" && conda activate "$2" && cd "$4" && exec python -u "$3"',
                'scan-workflow', str(CONDA / 'etc/profile.d/conda.sh'), env,
                str(ROOT / script), str(directory)]
    return {
        'global': python_in('camera', 'infer/hand_eye_calibration-main/run_probe_target.py'),
        'wrist_camera': ['bash', str(ROOT / 'infer/Camera_wrist/start_camera.sh')],
        'wrist_robot': ['bash', str(ROOT / 'infer/Camera_wrist/start_robot.sh')],
        'robot': ['bash', '-c', 'cd "$1" && exec ./main_rm75', 'scan-workflow',
                  str(ROOT / 'infer/Robot/build')],
        'ultrasound': python_in('carotid', 'intergrate_infer/main_redis_seg_newphase_recovery_mode.py',
                                ROOT / 'intergrate_infer'),
    }


def preflight():
    files = [CONDA / 'etc/profile.d/conda.sh',
             CONDA / 'envs/camera/bin/python', CONDA / 'envs/carotid/bin/python']
    files += [ROOT / name for name in (
        'infer/Robot/build/main_rm75', 'infer/Robot/build/arm_probe_pose',
        'infer/Robot/build/rm75_force_calibration.json',
        'infer/Camera_wrist/start_camera.sh', 'infer/Camera_wrist/start_robot.sh',
        'infer/Camera_wrist/gemini305_to_rm75_armtip.json',
        'infer/hand_eye_calibration-main/run_probe_target.py',
        'infer/hand_eye_calibration-main/d455_to_rm75_base.json',
        'intergrate_infer/main_redis_seg_newphase_recovery_mode.py')]
    for path in files:
        if not path.is_file():
            raise WorkflowError(f'缺少文件：{path}')
    for name in ('main_rm75', 'arm_probe_pose'):
        if not os.access(ROOT / 'infer/Robot/build' / name, os.X_OK):
            raise WorkflowError(f'Robot 入口不可执行：{name}')
    for name in ('bash', 'pgrep', 'redis-cli', 'redis-server'):
        if shutil.which(name) is None:
            raise WorkflowError(f'找不到命令：{name}')
    for pattern in (r'^(/[^ ]*/|\./)?main_rm75( |$)',
                    r'^(/[^ ]*/|\./)?arm_probe_pose( |$)',
                    r'^(\S*/)?python(3)?( -u)? (\S*/)?(click_follow|cliff_demo|run_probe_target|main_redis_seg_newphase_recovery_mode)\.py( |$)'):
        result = subprocess.run(['pgrep', '-af', pattern], text=True, capture_output=True)
        if result.returncode == 0:
            raise WorkflowError('请先停止已有控制或相机进程：\n' + result.stdout.strip())
        if result.returncode != 1:
            raise WorkflowError('无法检查已有进程：' + result.stderr.strip())


def ensure_redis():
    ping = ['redis-cli', '-h', '127.0.0.1', '-p', '7777', 'ping']
    result = subprocess.run(ping, capture_output=True, text=True, timeout=5)
    if result.returncode == 0 and result.stdout.strip() == 'PONG':
        return
    subprocess.run(['redis-server', '--bind', '127.0.0.1', '--port', '7777',
                    '--daemonize', 'yes'], check=True, timeout=10)
    result = subprocess.run(ping, capture_output=True, text=True, timeout=5)
    if result.returncode != 0 or result.stdout.strip() != 'PONG':
        raise WorkflowError('Redis 7777 启动失败')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', help='只检查本机入口与已有进程，不启动设备或 Redis')
    args = parser.parse_args()
    # SIGTERM follows the same cleanup path as terminal Ctrl+C.
    def interrupted(_signum, _frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    try:
        with open(Path(tempfile.gettempdir()) / f'uspilot-scan-{os.getuid()}.lock', 'w') as lock:
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                raise WorkflowError('已有一键扫描流程正在运行')
            preflight()
            if args.check:
                print('本机入口与进程检查通过；未启动 Redis、相机或机械臂，未验证设备与标定精度。')
                return 0
            print('真实运动流程：全局采样 → 腕部跟随 → 超声扫描。Ctrl+C 中止整条流程。', flush=True)
            ensure_redis()
            Workflow().run(commands())
            return 0
    except KeyboardInterrupt:
        print('\n[流程] 已请求中止，未继续下一阶段。', file=sys.stderr)
        return 130
    except (WorkflowError, OSError, subprocess.SubprocessError) as error:
        print(f'[流程] {error}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
