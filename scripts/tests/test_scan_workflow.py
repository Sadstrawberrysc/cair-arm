"""Offline lifecycle tests: fake child processes, no Redis or hardware."""
import importlib.util
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import textwrap
import unittest

SCRIPT = Path(__file__).resolve().parents[1] / 'run_scan_workflow.py'
spec = importlib.util.spec_from_file_location('scan_workflow', SCRIPT)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

FAKE = '''
import pathlib, signal, sys, time
name, mode, root = sys.argv[1:]
root = pathlib.Path(root)
def event(value):
    with (root / 'events').open('a') as f:
        f.write(name + ':' + value + '\\n')
def stop(*_):
    event('stop')
    sys.exit(0)
signal.signal(signal.SIGINT, stop)
if mode == 'ignore':
    signal.signal(signal.SIGINT, signal.SIG_IGN)
event('start')
if name == 'robot':
    assert 'wrist_robot:stop' in (root / 'events').read_text()
if mode == 'fail':
    time.sleep(.1)
    sys.exit(6)
if mode == 'exit':
    time.sleep(.1)
    sys.exit(0)
if mode == 'ui':
    if name != 'global':
        companion = 'wrist_robot' if name == 'wrist_camera' else 'robot'
        for _ in range(200):
            if companion + ':start' in (root / 'events').read_text():
                break
            time.sleep(.01)
        time.sleep(.2)
    event('done')
else:
    while True:
        time.sleep(.1)
'''


class WorkflowTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.fake = self.root / 'fake.py'
        self.fake.write_text(textwrap.dedent(FAKE))
        self.workflow = module.Workflow(self.root, stop_timeout=2)

    def commands(self, failures=()):
        return {name: [sys.executable, str(self.fake), name,
                       'fail' if name in failures else
                       ('ui' if name in ('global', 'wrist_camera', 'ultrasound') else 'robot'),
                       str(self.root)]
                for name in ('global', 'wrist_camera', 'wrist_robot', 'robot', 'ultrasound')}

    def events(self):
        return (self.root / 'events').read_text().splitlines()

    def assert_stopped(self):
        self.assertTrue(all(proc.poll() is not None for _, proc in self.workflow.children))

    def test_complete_workflow_stops_wrist_before_ultrasound(self):
        self.workflow.run(self.commands())
        events = self.events()
        self.assertLess(events.index('global:done'), events.index('wrist_camera:start'))
        self.assertLess(events.index('wrist_camera:done'), events.index('wrist_robot:stop'))
        self.assertLess(events.index('wrist_robot:stop'), events.index('robot:start'))
        self.assertLess(events.index('ultrasound:done'), events.index('robot:stop'))
        self.assert_stopped()

    def test_global_failure_does_not_launch_robot(self):
        with self.assertRaises(module.WorkflowError):
            self.workflow.run(self.commands(('global',)))
        self.assertEqual(self.events(), ['global:start'])
        self.assert_stopped()

    def test_wrist_fault_cleans_camera_without_handoff(self):
        with self.assertRaises(module.WorkflowError):
            self.workflow.run(self.commands(('wrist_robot',)))
        self.assertNotIn('robot:start', self.events())
        self.assert_stopped()

    def test_ultrasound_failure_stops_controller(self):
        with self.assertRaises(module.WorkflowError):
            self.workflow.run(self.commands(('ultrasound',)))
        self.assertIn('robot:stop', self.events())
        self.assert_stopped()

    def test_controller_exiting_zero_is_not_a_handoff(self):
        commands = self.commands()
        commands['wrist_robot'][3] = 'exit'
        with self.assertRaises(module.WorkflowError):
            self.workflow.run(commands)
        self.assertNotIn('robot:start', self.events())
        self.assert_stopped()

    def test_wrist_camera_failure_does_not_launch_ultrasound(self):
        with self.assertRaises(module.WorkflowError):
            self.workflow.run(self.commands(('wrist_camera',)))
        self.assertNotIn('robot:start', self.events())
        self.assert_stopped()

    def test_stop_timeout_prevents_handoff(self):
        commands = self.commands()
        commands['wrist_robot'][3] = 'ignore'
        self.workflow.stop_timeout = .1
        try:
            with self.assertRaises(module.WorkflowError):
                self.workflow.run(commands)
            self.assertNotIn('robot:start', self.events())
        finally:
            # Only fake processes are forcibly cleaned by the test harness.
            for _, proc in self.workflow.children:
                if proc.poll() is None:
                    proc.terminate()
                    proc.wait(timeout=2)
        self.assert_stopped()

    def test_sigterm_stops_owned_process_group(self):
        # Run the actual run/cleanup path in a child orchestrator, then cancel it.
        driver = self.root / 'driver.py'
        driver.write_text(textwrap.dedent(f'''
            import importlib.util, signal
            spec = importlib.util.spec_from_file_location('workflow', {str(SCRIPT)!r})
            m = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(m)
            def cancel(*_):
                raise KeyboardInterrupt
            signal.signal(signal.SIGTERM, cancel)
            try:
                m.Workflow(root=m.Path({str(self.root)!r}), stop_timeout=2).run(
                    {{'global': {[sys.executable, str(self.fake), 'global', 'robot', str(self.root)]!r}}})
            except KeyboardInterrupt:
                pass
        '''))
        proc = subprocess.Popen([sys.executable, str(driver)])
        try:
            import time
            for _ in range(200):
                if (self.root / 'events').exists():
                    break
                time.sleep(.01)
            self.assertTrue((self.root / 'events').exists())
            proc.send_signal(signal.SIGTERM)
            self.assertEqual(proc.wait(timeout=5), 0)
            self.assertIn('global:stop', self.events())
        finally:
            if proc.poll() is None:
                proc.kill()
                proc.wait()


if __name__ == '__main__':
    unittest.main()
