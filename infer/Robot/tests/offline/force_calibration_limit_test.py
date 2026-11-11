"""Inject excessive raw wrench into simulated devices; never use real hardware."""
import argparse
from pathlib import Path
import subprocess
import tempfile

from force_calibration_auto_test import SimulatedHardware


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--capture',type=Path,required=True)
    args = parser.parse_args()
    with tempfile.TemporaryDirectory() as temporary:
        for index,axis,unit,limit in ((2,'Fz','N','50'),(4,'Ty','N*m','2')):
            values = [0.0]*6
            values[index] = -51.0 if index==2 else 2.1
            hardware = SimulatedHardware(static_wrench=values)
            output = Path(temporary)/f'{axis}.csv'
            try:
                result = subprocess.run([str(args.capture.resolve()),'--automatic','--path-mode','local',
                    '--poses','27','--samples-per-pose','5','--min-window-duration-ms','100',
                    '--robot-ip','127.0.0.1','--robot-port',str(hardware.port),
                    '--sensor-device',hardware.device,'--output',str(output),
                    '--sensor-to-tool-rpy-deg','0,0,30','--capture-timeout-sec','5'],
                    capture_output=True,text=True,timeout=15)
                log = result.stdout+result.stderr
                assert result.returncode==5,log
                assert 'Capture rejected [wrench_limit_exceeded]' in log,log
                assert f'axis={axis}; value=' in log and f'abs_limit={limit} {unit}' in log,log
                assert 'sensor_sequence=' in log and 'raw_si=[' in log,log
                assert 'Accepted pose' not in log and not output.exists(),log
                assert hardware.stop_commands>=1 and not hardware.errors,(log,hardware.errors)
                partials=[p for p in output.parent.glob(output.name+'.partial.*') if not p.name.endswith('.json')]
                assert len(partials)==1 and '# pose=' not in partials[0].read_text()
            finally:
                hardware.close()
    print('Force/torque limit diagnostics, rejection and simulated stop passed.')


if __name__ == '__main__':
    main()
