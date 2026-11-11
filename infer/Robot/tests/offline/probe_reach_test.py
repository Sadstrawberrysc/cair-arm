"""Check reach rejection without opening a robot connection."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile


binary, fixture = sys.argv[1:]
with tempfile.TemporaryDirectory() as directory:
    # Reproduce the deployed geometry, independent of local calibration files.
    calibration = json.loads(Path(fixture).read_text())
    calibration['probe_tcp_sensor_m'] = [0, 0, .188]
    calibration['sensor_to_tool']['translation_sensor_to_tool_m'] = [0, 0, -.0125]
    calibration['sensor_to_tool']['rotation_row_major'] = [
        .8660254037844387, -.5, 0, .5, .8660254037844387, 0, 0, 0, 1]
    path = Path(directory) / 'tool.json'
    path.write_text(json.dumps(calibration))
    for pose, expected, flags in [
        # Field failure: wrist radius 664.472 mm, arm-length bound 466 mm.
        ('0.284218563,-0.355368257,0.538987920,-2.976832174,0.529715423,3.009829134',
         4, ['--execute', '--confirm-single-movej']),
        ('0.284218563,-0.355368257,0.538987920,-2.976832174,0.529715423,3.009829134',
         4, ['--inspect-transform-only']),
        # Exact straight-arm boundary with tool/calibration offsets included.
        ('0,0,1.051,0,0,0', 0, ['--inspect-transform-only']),
        # TCP within the tip's spherical bound, but orientation makes wrist unreachable.
        ('0,0,.7,3.141592653589793,0,0', 4, ['--inspect-transform-only']),
        ('0,0,.6,0,0,0', 0, ['--inspect-transform-only']),
    ]:
        result = subprocess.run(
            [binary, '--tool-calibration', str(path), '--target-pose-m-rad', pose,
             '--controller-movej-p'] + flags,
            capture_output=True, text=True, timeout=5)
        output = result.stdout + result.stderr
        assert result.returncode == expected, output
        assert 'Connecting to Realman' not in output, output
        assert '"command":"movej_p"' not in output, output
        assert 'maximum_wrist_radius_mm: 466.000000' in output, output
        if expected == 4:
            assert 'Target is geometrically unreachable' in output, output
print('5 offline wrist-reach cases passed')
