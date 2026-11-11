"""Check the Python recovery entrypoint without any robot or sensor access."""
import contextlib
import io
import json
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'tools'))
import calibrate_force_static as cli


class RecoveryCLI(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.output = self.root / 'samples.csv'
        self.geometry = self.root / 'calibration.json'
        self.geometry.write_text(json.dumps({
            'sensor_to_tool': {'rotation_row_major': [1,0,0,0,1,0,0,0,1],
                               'translation_sensor_to_tool_m': [0,0,0]},
            'probe_tcp_sensor_m': [0,0,.2], 'probe_model': 'test.stl'}))

    def checkpoint(self, output=None, suffix='1', height=.3):
        partial = Path(str(output or self.output) + '.partial.' + suffix)
        partial.write_text('# saved samples must remain unchanged\n1,0,0,0,1,0,0,0,1,0,0,0,0,0,0\n')
        Path(str(partial)+'.path.json').write_text(json.dumps({
            'pose_count': 31, 'mode': 'workspace_coverage',
            'workspace_constraints': {'minimum_body_z_m': height, 'body_spheres': []}}))
        return partial

    def invoke(self, *arguments):
        with patch.object(sys, 'argv', ['calibrate_force_static.py', '--calibration', str(self.geometry), *arguments]), \
             patch.object(cli.subprocess, 'run') as run, contextlib.redirect_stdout(io.StringIO()), \
             contextlib.redirect_stderr(io.StringIO()):
            cli.main()
            return run.call_args_list

    def test_explicit_checkpoint_restores_count_output_and_height(self):
        partial = self.checkpoint()
        original = partial.read_bytes()
        calls = self.invoke('--resume', str(partial), '--minimum-end-effector-z-m', '.30')
        capture = calls[0].args[0]
        self.assertEqual(capture[capture.index('--poses')+1], '31')
        self.assertEqual(capture[capture.index('--output')+1], str(self.output))
        self.assertEqual(capture[capture.index('--resume')+1], str(partial))
        self.assertNotIn('--workspace-config', capture)
        self.assertFalse(Path(str(self.output)+'.workspace.json').exists())
        self.assertEqual(partial.read_bytes(), original)
        self.assertEqual(calls[1].args[0][2], str(self.output))  # fit uses resumed output

    def test_latest_is_scoped_to_output_and_requires_snapshot(self):
        old = self.checkpoint(suffix='old')
        new = self.checkpoint(suffix='new')
        os.utime(old, ns=(1,1))
        os.utime(new, ns=(2,2))
        self.checkpoint(self.root/'different.csv')
        Path(str(self.output)+'.partial.no-snapshot').write_text('incomplete')
        capture = self.invoke('--resume','--output',str(self.output))[0].args[0]
        self.assertEqual(capture[capture.index('--resume')+1], str(new))

    def test_conflicts_fail_before_subprocess(self):
        partial = self.checkpoint()
        for flags in (['--poses','1000'], ['--minimum-end-effector-z-m','.4'],
                      ['--workspace-config','other.json'], ['--path-mode','local']):
            with self.subTest(flags=flags), self.assertRaises(SystemExit) as error:
                self.invoke('--resume',str(partial),*flags)
            self.assertEqual(error.exception.code,2)

    def test_new_empty_checkpoint_does_not_hide_saved_samples(self):
        saved = self.checkpoint(suffix='21-samples')
        saved.write_text(saved.read_text().splitlines()[0]+'\n'+
                         '1,0,0,0,1,0,0,0,1,0,0,0,0,0,0\n'*21)
        empty = self.checkpoint(suffix='empty')
        empty.write_text('# automatic=true; robot_motion=MoveJ\nr00,r01\n')
        os.utime(saved, ns=(1,1))
        os.utime(empty, ns=(2,2))
        original = saved.read_bytes()
        capture = self.invoke('--resume','--output',str(self.output))[0].args[0]
        self.assertEqual(capture[capture.index('--resume')+1], str(saved))
        self.assertEqual(saved.read_bytes(), original)

    def test_all_empty_checkpoints_fail_before_capture(self):
        self.checkpoint().write_text('# no samples\n')
        with self.assertRaises(SystemExit):
            self.invoke('--resume','--output',str(self.output))

    def test_check_resume_only_validates_and_does_not_fit(self):
        partial = self.checkpoint()
        calls = self.invoke('--resume',str(partial),'--check-resume')
        self.assertEqual(len(calls),1)
        self.assertIn('--check-resume',calls[0].args[0])
        self.assertNotIn('--automatic',calls[0].args[0])

    def test_missing_checkpoint_and_completed_output_do_not_restart(self):
        with self.assertRaises(SystemExit):
            self.invoke('--resume','--output',str(self.output))
        self.checkpoint()
        self.output.write_text('# capture_complete=true\n')
        with self.assertRaises(SystemExit):
            self.invoke('--resume','--output',str(self.output))


if __name__ == '__main__':
    unittest.main()
