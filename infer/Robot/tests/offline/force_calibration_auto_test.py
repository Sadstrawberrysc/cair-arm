"""Exercise automatic capture and fitting against loopback TCP and a KWR57 PTY."""
import argparse
import csv
import io
import json
import math
import os
from pathlib import Path
import pty
import select
import socket
import struct
import subprocess
import tempfile
import threading
import time


INITIAL_JOINTS = [27215, 17843, 82979, 110588, -28429, -7429, 1023]  # millidegrees
GRAVITY = [.2, -.3, -5.0]
FORCE_BIAS = [.2, -.1, .3]
TORQUE_BIAS = [.01, -.02, .015]
CENTER_OF_MASS = [.01, -.02, .08]


def rotation(rx, ry, rz):
    cx, sx, cy, sy, cz, sz = (math.cos(rx), math.sin(rx), math.cos(ry),
                             math.sin(ry), math.cos(rz), math.sin(rz))
    return [[cz*cy, cz*sy*sx-sz*cx, cz*sy*cx+sz*sx],
            [sz*cy, sz*sy*sx+cz*cx, sz*sy*cx-cz*sx],
            [-sy, cy*sx, cy*cx]]


def multiply(a, b):
    return [[sum(a[i][k]*b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def wrench_from_rotation(base_sensor):
    gravity = [sum(base_sensor[j][i]*GRAVITY[j] for j in range(3)) for i in range(3)]
    x, y, z = CENTER_OF_MASS
    gx, gy, gz = gravity
    torque = [y*gz-z*gy, z*gx-x*gz, x*gy-y*gx]
    return [a+b for a, b in zip(FORCE_BIAS, gravity)] + [a+b for a, b in zip(TORQUE_BIAS, torque)]


def wrench(pose):
    return wrench_from_rotation(multiply(rotation(*[v/1000 for v in pose[3:]]),
                                         rotation(0, 0, math.pi/6)))


class SimulatedHardware:
    def __init__(self, quantized_feedback=False, settling_seconds=0, static_wrench=None):
        self.static_wrench = static_wrench
        self.settling_seconds = settling_seconds
        self.settling_until = None
        self.quantized_feedback = quantized_feedback
        self.feedback_count = 0
        self.lock = threading.Lock()
        self.stopped = threading.Event()
        self.joints = INITIAL_JOINTS.copy()
        self.pose = self.pose_from_joints(self.joints)
        self.moving = False
        self.moves = []
        self.linear_moves = []
        self.stop_commands = 0
        self.frames = 0
        self.errors = []
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.bind(('127.0.0.1', 0))
        self.server.listen(1)
        self.server.settimeout(.1)
        self.port = self.server.getsockname()[1]
        self.master, slave = pty.openpty()
        self.device = os.ttyname(slave)
        os.close(slave)
        os.set_blocking(self.master, False)
        self.threads = [threading.Thread(target=self.serve, daemon=True),
                        threading.Thread(target=self.stream_sensor, daemon=True)]
        for thread in self.threads:
            thread.start()

    @staticmethod
    def pose_from_joints(joints):
        # Quantize exactly as the controller's milliradian pose response.
        return [100000, 200000, 500000] + [round(v*math.pi/180) for v in joints[4:]]

    def serve(self):
        try:
            while not self.stopped.is_set():
                try:
                    client, _ = self.server.accept()
                    break
                except socket.timeout:
                    continue
            else:
                return
            with client, client.makefile('rb') as incoming:
                for line in incoming:
                    request = json.loads(line)
                    command = request['command']
                    if command == 'get_current_arm_state':
                        with self.lock:
                            assert not self.moving, 'state query overlapped blocking MoveJ'
                            pose = self.pose.copy()
                            if self.settling_until is not None and time.monotonic() < self.settling_until:
                                pose[0] += 500 if self.feedback_count % 2 else -500
                            if self.quantized_feedback:
                                self.feedback_count += 1
                                pose[3] += self.feedback_count % 2
                                pose[4] += self.feedback_count % 2
                            state = dict(joint=self.joints.copy(), pose=pose,
                                         arm_err=0, sys_err=0)
                        reply = dict(state='current_arm_state', arm_state=state)
                    elif command == 'movej':
                        with self.lock:
                            self.moving = True
                        client.sendall(b'{"receive_state":true}\r\n')
                        time.sleep(.02)
                        with self.lock:
                            self.joints = request['joint']
                            self.pose = self.pose_from_joints(self.joints)
                            self.moves.append(self.joints.copy())
                            if self.settling_until is None and self.settling_seconds:
                                self.settling_until = time.monotonic() + self.settling_seconds
                            self.moving = False
                        reply = dict(trajectory_state=True)
                    elif command == 'movel':
                        with self.lock:
                            self.pose = request['pose']
                            self.linear_moves.append(self.pose.copy())
                        reply = dict(trajectory_state=True)
                    elif command == 'set_arm_stop':
                        self.stop_commands += 1
                        reply = dict(command=command, arm_stop=True)
                    else:
                        raise AssertionError('unexpected robot command: ' + command)
                    client.sendall((json.dumps(reply)+'\r\n').encode())
        except (OSError, AssertionError, ValueError) as error:
            if not self.stopped.is_set():
                self.errors.append(str(error))

    def stream_sensor(self):
        enabled = False
        commands = b''
        while not self.stopped.is_set():
            try:
                if select.select([self.master], [], [], .005)[0]:
                    commands += os.read(self.master, 4096)
                    if b'\x48\xaa\r\n' in commands:
                        enabled = True
                        commands = b''
                    if b'\x43\xaa\r\n' in commands:
                        enabled = False
                        commands = b''
                if enabled:
                    with self.lock:
                        # Deliberately different moving values prove that
                        # accepted static averages exclude movement samples.
                        values = ([40., 40., 40., 1., 1., 1.] if self.moving else
                                  self.static_wrench if self.static_wrench is not None else wrench(self.pose))
                    packet = b'\x48\xaa' + struct.pack('<6f', *[v/9.80665 for v in values]) + b'\r\n'
                    os.write(self.master, packet)
                    self.frames += 1
            except (BlockingIOError, OSError):
                time.sleep(.005)  # The child has not opened the slave yet.

    def close(self):
        self.stopped.set()
        self.server.close()
        for thread in self.threads:
            thread.join(timeout=2)
        os.close(self.master)


def run_checked(command):
    process = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                             text=True, timeout=45)
    if process.returncode:
        raise AssertionError('command failed (%s):\n%s' % (process.returncode, process.stdout))
    return process.stdout


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture', type=Path, required=True)
    parser.add_argument('--fit', type=Path, required=True)
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix='uspilot-force-auto-') as temporary:
        output = Path(temporary)/'samples.csv'
        hardware = SimulatedHardware()
        try:
            run_checked([str(args.capture.resolve()), '--automatic', '--path-mode', 'local', '--poses', '27',
                '--samples-per-pose', '5', '--min-window-duration-ms', '100',
                '--robot-ip', '127.0.0.1', '--robot-port', str(hardware.port),
                '--sensor-device', hardware.device, '--output', str(output),
                '--sensor-to-tool-rpy-deg', '0,0,30', '--capture-timeout-sec', '5', '--overwrite'])
            assert not hardware.errors, hardware.errors
            assert hardware.frames > 27 * 5
            assert len(hardware.moves) >= 28, len(hardware.moves)
            assert hardware.moves[-1] == INITIAL_JOINTS, hardware.moves[-1]
            assert hardware.stop_commands == 1, hardware.stop_commands
        finally:
            hardware.close()
        text = output.read_text()
        assert '# capture_complete=true' in text
        assert '# automatic=true; robot_motion=MoveJ' in text
        rows = list(csv.DictReader(io.StringIO('\n'.join(line for line in text.splitlines()
                                                         if line and not line.startswith('#')))))
        assert len(rows) == 27, len(rows)
        for row in rows:
            assert len(row) == 15
            base_sensor = [[float(row['r%d%d' % (i, j)]) for j in range(3)] for i in range(3)]
            measured = [float(row[key]) for key in ['fx', 'fy', 'fz', 'tx', 'ty', 'tz']]
            expected = wrench_from_rotation(base_sensor)
            assert max(abs(a-b) for a, b in zip(measured, expected)) < 1e-5, row
        result_path = Path(temporary)/'calibration.json'
        run_checked([str(args.fit.resolve()), '--input', str(output), '--output', str(result_path),
            '--sensor-id', 'KWR57-simulated', '--probe-model', 'simulated-probe',
            '--sensor-to-tool-rpy-deg', '0,0,30'])
        result = json.loads(result_path.read_text())
        for key, expected in [('gravity_base_n', GRAVITY), ('force_bias_n', FORCE_BIAS),
                              ('torque_bias_nm', TORQUE_BIAS), ('center_of_mass_sensor_m', CENTER_OF_MASS)]:
            actual = result['gravity_model'][key]
            assert max(abs(a-b) for a, b in zip(actual, expected)) < 1e-5, (key, actual, expected)
        assert result['quality']['residuals_verified']
        interrupted = Path(temporary)/'stopped.partial'
        interrupted.write_text(text.replace('# capture_complete=true\n',''))
        explicit_output = Path(temporary)/'interrupted_fit.json'
        explicit_fit = [str(args.fit.resolve()), '--input', str(interrupted), '--output', str(explicit_output),
            '--sensor-id', 'KWR57-simulated', '--probe-model', 'simulated-probe',
            '--sensor-to-tool-rpy-deg', '0,0,30']
        rejected = subprocess.run(explicit_fit, capture_output=True, text=True)
        assert rejected.returncode == 3 and not explicit_output.exists()
        original_interrupted = interrupted.read_bytes()
        run_checked(explicit_fit + ['--allow-incomplete'])
        assert interrupted.read_bytes() == original_interrupted
        assert json.loads(explicit_output.read_text())['quality']['residuals_verified']

        # Resume the exact saved grid; preserve five already accepted averages.
        partial = Path(temporary)/'samples.partial'
        partial.write_text(text.split('# pose=6;')[0])
        Path(str(partial)+'.path.json').write_bytes(Path(str(output)+'.path.json').read_bytes())
        # The shared planning filename may have been overwritten by another plan.
        Path(str(output)+'.path.json').write_text('{"invalid_shared_path":true}')
        hardware = SimulatedHardware(quantized_feedback=True, settling_seconds=1.5)
        try:
            log = run_checked([str(args.capture.resolve()), '--resume', str(partial), '--poses', '27',
                '--samples-per-pose', '5', '--min-window-duration-ms', '100',
                '--robot-ip', '127.0.0.1', '--robot-port', str(hardware.port),
                '--sensor-device', hardware.device, '--output', str(output),
                '--sensor-to-tool-rpy-deg', '0,0,30', '--capture-timeout-sec', '5', '--overwrite'])
            assert 'preserved 5 poses; next pose 6/27' in log, log
            assert 'Waiting for a stationary window' not in log, log
            assert 'Stable window found' not in log, log
            assert 'Accepted pose 1:' not in log
            assert not hardware.errors, hardware.errors
            assert hardware.moves[-1] == INITIAL_JOINTS
        finally:
            hardware.close()
        resumed = output.read_text()
        assert resumed.startswith(partial.read_text())
        assert '# capture_complete=true' in resumed
        resumed_rows = list(csv.DictReader(io.StringIO('\n'.join(
            line for line in resumed.splitlines() if line and not line.startswith('#')))))
        assert len(resumed_rows) == 27
        assert resumed_rows[:5] == rows[:5]
        spans = [float(line.split('orientation_span_deg=')[1]) for line in resumed.splitlines()
                 if line.startswith('# pose=')]
        assert max(spans[5:]) > .08, spans
        # Feedback drift is allowed: no positional/joint stationarity gate.
        hardware = SimulatedHardware(quantized_feedback=True, settling_seconds=100)
        try:
            failed = subprocess.run([str(args.capture.resolve()), '--resume', str(partial), '--poses', '27',
                '--samples-per-pose', '5', '--min-window-duration-ms', '100',
                '--robot-ip', '127.0.0.1', '--robot-port', str(hardware.port),
                '--sensor-device', hardware.device, '--output', str(output),
                '--sensor-to-tool-rpy-deg', '0,0,30', '--capture-timeout-sec', '1', '--overwrite'],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=45)
            assert failed.returncode == 0, failed.stdout
            assert 'Accepted pose 27:' in failed.stdout, failed.stdout
            assert 'window_motion' not in failed.stdout, failed.stdout
            drift_text = output.read_text()
            drift_spans = [float(line.split('position_span_mm=')[1].split(';')[0])
                           for line in drift_text.splitlines() if line.startswith('# pose=')]
            assert max(drift_spans[5:]) > .1, drift_spans
            assert '# capture_complete=true' in drift_text
            assert hardware.moves[-1] == INITIAL_JOINTS
            assert not hardware.errors, hardware.errors
        finally:
            hardware.close()
        print('Pose drift accepted without dwell or stationary-window waiting.')
        print('Resume and quantized Euler feedback passed; original averages preserved.')
        workspace_output = Path(temporary)/'workspace.csv'
        config = Path(temporary)/'regions.json'
        config.write_text(json.dumps({'schema_version': 1, 'frame': 'rm75_base',
            'allowed_regions': [
                {'min_m':[-.9,-.9,.2], 'max_m':[-.05,.9,1.1]},
                {'min_m':[.05,-.9,.2], 'max_m':[.9,.9,1.1]}],
            'forbidden_spheres':[{'center_m':[.2,0,.4], 'radius_m':.15}]}))
        run_checked([str(args.capture.resolve()), '--plan-only', '--path-mode', 'workspace',
            '--poses', '6', '--workspace-config', str(config),
            '--start-joints-deg', ','.join(str(x/1000) for x in INITIAL_JOINTS),
            '--output', str(workspace_output), '--sensor-to-tool-rpy-deg', '0,0,30', '--overwrite'])
        plan = json.loads(Path(str(workspace_output)+'.path.json').read_text())
        assert len(plan['points']) == 6 and not workspace_output.exists()
        for point in plan['points']:
            x,y,z = point['tcp_base_m']
            assert -.9 <= x <= .9 and abs(x)>=.05 and -.9 <= y <= .9 and .2<=z<=1.1
            assert (x-.2)**2+y*y+(z-.4)**2 > .15**2
        before = partial.read_bytes()
        mixed = run_checked([str(args.capture.resolve()), '--resume', str(partial), '--resume-workspace',
            '--plan-only', '--poses', '27', '--workspace-config', str(config),
            '--output', str(output), '--sensor-to-tool-rpy-deg', '0,0,30', '--overwrite'])
        assert 'replanned only 22 remaining targets' in mixed
        assert partial.read_bytes() == before
        derived = json.loads(Path(str(partial)+'.workspace.path.json').read_text())
        assert derived['preserved_prefix_count'] == 5
        # A measured keep-out sphere alone must filter the whole kinematic domain.
        config.write_text(json.dumps({'schema_version': 1, 'frame': 'rm75_base',
            'forbidden_spheres':[{'center_m':[.2,0,.4], 'radius_m':.2}]}))
        run_checked([str(args.capture.resolve()), '--plan-only', '--path-mode', 'workspace',
            '--poses', '6', '--workspace-config', str(config),
            '--start-joints-deg', ','.join(str(x/1000) for x in INITIAL_JOINTS),
            '--output', str(workspace_output), '--sensor-to-tool-rpy-deg', '0,0,30', '--overwrite'])
        plan = json.loads(Path(str(workspace_output)+'.path.json').read_text())
        assert plan['motion_allowed'] and not plan['collision_checked']
        assert not plan['return_to_center']
        assert plan['workspace_constraints']['forbidden_spheres'][0]['radius_m'] == .2
        for point in plan['points']:
            x,y,z = point['tcp_base_m']
            assert (x-.2)**2+y*y+(z-.4)**2 > .2**2
        hardware = SimulatedHardware()
        try:
            r = rotation(*[v/1000 for v in hardware.pose[3:]])
            tcp = [hardware.pose[i]/1e6 + .2005*r[i][2] for i in range(3)]
            config.write_text(json.dumps({'schema_version':1, 'frame':'rm75_base',
                'forbidden_spheres':[{'center_m':tcp,'radius_m':.2}],
                'exit_direction_base':[0,0,1]}))
            resumed = run_checked([str(args.capture.resolve()), '--resume', str(partial),
                '--resume-workspace', '--poses', '27', '--workspace-config', str(config),
                '--samples-per-pose', '5', '--min-window-duration-ms', '100',
                '--robot-ip', '127.0.0.1', '--robot-port', str(hardware.port),
                '--sensor-device', hardware.device, '--output', str(workspace_output),
                '--sensor-to-tool-rpy-deg', '0,0,30', '--capture-timeout-sec', '5', '--overwrite'])
            assert 'Accepted pose 6:' in resumed and 'Accepted pose 27:' in resumed, resumed
            assert len(hardware.linear_moves) == 1
            assert hardware.linear_moves[0][2] == 750000
            assert hardware.moves[-1] != INITIAL_JOINTS
            assert hardware.stop_commands == 1 and not hardware.errors, hardware.errors
            assert workspace_output.read_bytes().startswith(before)
            assert partial.read_bytes() == before
        finally:
            hardware.close()
        print('Workspace resume, +Z exit and no return to the old center passed.')
        # A height envelope is sufficient workspace configuration. Resume must
        # recover it from the immutable path snapshot without a config flag.
        config.write_text(json.dumps({'schema_version':1, 'frame':'rm75_base',
            'minimum_body_z_m':.3, 'body_spheres':[
                {'joint_index':6, 'name':'end_effector', 'center_m':[0,0,.1], 'radius_m':.1}]}))
        height_output = Path(temporary)/'height.csv'
        height_partial = Path(temporary)/'height.partial'
        for resume_height in (False, True):
            hardware = SimulatedHardware()
            with hardware.lock:
                hardware.joints = [0]*7
                hardware.pose = hardware.pose_from_joints(hardware.joints)
            try:
                command = [str(args.capture.resolve()), '--automatic', '--poses', '6',
                    '--samples-per-pose', '5', '--min-window-duration-ms', '100',
                    '--robot-ip', '127.0.0.1', '--robot-port', str(hardware.port),
                    '--sensor-device', hardware.device, '--output', str(height_output),
                    '--sensor-to-tool-rpy-deg', '0,0,30', '--capture-timeout-sec', '5', '--overwrite']
                command += ['--resume',str(height_partial)] if resume_height else ['--workspace-config',str(config)]
                log = run_checked(command)
                assert 'Accepted pose 6:' in log and not hardware.errors, log
            finally:
                hardware.close()
            if not resume_height:
                snapshot = Path(str(height_output)+'.path.json').read_bytes()
                height_plan = json.loads(snapshot)
                assert height_plan['height_segment_checked']
                assert height_plan['workspace_constraints']['minimum_body_z_m'] == .3
                height_partial.write_text(height_output.read_text().split('# pose=4;')[0])
                Path(str(height_partial)+'.path.json').write_bytes(snapshot)
        print('Height-constrained capture and snapshot-only resume passed.')
        print('Automatic capture -> 27 static CSV rows -> fitting passed; returned to start; no moving samples.')


if __name__ == '__main__':
    main()
