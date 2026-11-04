"""无相机/机器人连接的采集回归测试。"""
import contextlib
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import cv2
import numpy as np

SCRIPT = Path(__file__).resolve().parents[1] / 'collect_data.py'
spec = importlib.util.spec_from_file_location('collect_data', SCRIPT)
collect = importlib.util.module_from_spec(spec)
spec.loader.exec_module(collect)


class CaptureTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.directory = Path(self.temp.name) / 'capture'
        self.board = {'XX': 9, 'YY': 6, 'L': .025}
        self.image = np.full((360, 480, 3), 255, np.uint8)
        for y in range(7):
            for x in range(10):
                if (x+y) % 2 == 0:
                    self.image[40+y*40:80+y*40, 40+x*40:80+x*40] = 0
        self.camera_metadata = {'width': 480, 'height': 360, 'backend': 'test'}

    def session(self, target=50):
        return collect.CaptureSession(self.directory, target, self.board, self.camera_metadata, 'offline')

    def test_fifty_samples_stop_and_pair(self):
        session = self.session()
        class Camera:
            def read(inner):
                return self.image.copy()
        class Reader:
            calls = 0
            def read_pose(inner):
                # Each two reads represent one static sample, with a new position for the next.
                pose = np.array([.001*(inner.calls//2), 0, .3, .1, .2, .3])
                inner.calls += 1
                return pose
        reader = Reader()
        with patch.object(cv2, 'imshow'), patch.object(cv2, 'waitKey', return_value=ord('s')), contextlib.redirect_stdout(io.StringIO()):
            collect.capture(Camera(), reader, session)
            session.finish()
        self.assertEqual(reader.calls, 100)
        self.assertEqual({p.name for p in self.directory.glob('*.jpg')}, {f'{i}.jpg' for i in range(1, 51)})
        poses = np.loadtxt(self.directory/'poses.txt', delimiter=',')
        self.assertEqual(poses.shape, (50, 6))
        self.assertAlmostEqual(poses[-1, 0], .049)
        self.assertTrue(json.loads((self.directory/'session.json').read_text())['complete'])
        with self.assertRaises(ValueError):
            session.save(self.image, np.zeros(6))

    def test_missing_board_does_not_count(self):
        session = self.session()
        with contextlib.redirect_stdout(io.StringIO()):
            saved = session.save(np.full_like(self.image, 255), np.zeros(6))
            session.finish()
        self.assertFalse(saved)
        self.assertEqual(len(session.poses), 0)
        self.assertFalse(json.loads((self.directory/'session.json').read_text())['complete'])

    def test_pair_write_failure_rolls_back(self):
        session = self.session()
        replace = collect.os.replace
        def fail_pose(source, target):
            if Path(target).name == 'poses.txt':
                raise OSError('injected write failure')
            return replace(source, target)
        with patch.object(collect.os, 'replace', side_effect=fail_pose), self.assertRaises(OSError):
            session.save(self.image, np.zeros(6))
        self.assertFalse(list(self.directory.glob('*.jpg')))
        self.assertEqual(session.poses, [])
        self.assertFalse((self.directory/'poses.txt').exists())

    def test_existing_directory_not_overwritten(self):
        self.directory.mkdir()
        with self.assertRaises(FileExistsError):
            self.session()

    def test_motion_rejected_and_angle_wrap_allowed(self):
        before = np.zeros(6)
        after = before.copy(); after[2] = .001
        self.assertFalse(collect.pose_is_still(before, after))
        after = before.copy(); after[3] = .01
        self.assertFalse(collect.pose_is_still(before, after))
        before[5] = np.pi - .0001; after = before.copy(); after[5] = -np.pi + .0001
        self.assertTrue(collect.pose_is_still(before, after))

    def test_fragmented_robot_reply_and_units(self):
        message = json.dumps({'state': 'current_arm_state', 'arm_state': {
            'pose': [100000, -200000, 300000, 100, -200, 300], 'arm_err': [0], 'sys_err': 0}}).encode()+b'\r\n'
        class Connection:
            def __init__(inner):
                inner.commands = []
                inner.chunks = iter([b'{"unrelated":true}\r\n'+message[:13], message[13:]])
            def sendall(inner, command):
                inner.commands.append(json.loads(command))
            def recv(inner, size):
                return next(inner.chunks)
            def settimeout(inner, timeout):
                pass
        connection = Connection()
        np.testing.assert_allclose(collect.RobotReader(connection).read_pose(), [.1, -.2, .3, .1, -.2, .3])
        self.assertEqual(connection.commands, [{'command': 'get_current_arm_state'}])

    def test_preview_never_connects_robot_or_creates_session(self):
        class Camera:
            metadata = {}
            def close(inner):
                pass
        with patch.object(collect, 'RealSenseCamera', return_value=Camera()), \
                patch.object(collect, 'capture') as capture, \
                patch.object(collect.socket, 'create_connection') as connection, \
                patch.object(cv2, 'destroyAllWindows'), contextlib.redirect_stdout(io.StringIO()):
            collect.main(['--preview-only', '--max-frames', '2', '--output-dir', str(self.directory)])
        capture.assert_called_once()
        self.assertIsNone(capture.call_args.args[1])
        connection.assert_not_called()
        self.assertFalse(self.directory.exists())


if __name__ == '__main__':
    unittest.main()
