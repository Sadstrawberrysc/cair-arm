"""标定编排的无硬件验证，以及腕部变换方向回归。"""
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import calibration_entry as entry
import calibration_solver as solver


class CalibrationEntryTests(unittest.TestCase):
    def test_preview_never_solves_or_connects_robot(self):
        for camera in ('realsense', 'gemini'):
            with self.subTest(camera=camera), patch.object(entry.collect_data, 'main') as capture, patch.object(entry, 'compute') as compute:
                entry.main(camera, ['--preview-only'])
                self.assertIn('--preview-only', capture.call_args.args[0])
                self.assertNotIn('--output-dir', capture.call_args.args[0])
                compute.assert_not_called()

    def test_incomplete_or_wrong_camera_never_solves(self):
        with tempfile.TemporaryDirectory() as tmp:
            session = Path(tmp) / 'session.json'
            for metadata in ({'complete': False, 'camera': {'backend': 'gemini'}},
                             {'complete': True, 'camera': {'backend': 'realsense'}}):
                session.write_text(json.dumps(metadata))
                with patch.object(entry, 'compute') as compute, self.assertRaises(ValueError):
                    entry.main('gemini', ['--data-dir', tmp])
                compute.assert_not_called()

    def test_capture_then_solve_exact_new_session(self):
        for camera, wrist in (('realsense', False), ('gemini', True)):
            with self.subTest(camera=camera), tempfile.TemporaryDirectory() as tmp:
                directory = Path(tmp) / 'new'
                def capture(argv):
                    self.assertEqual(Path(argv[argv.index('--output-dir') + 1]), directory)
                    directory.mkdir()
                    (directory / 'session.json').write_text(json.dumps(
                        {'complete': True, 'camera': {'backend': camera}}))
                key = 'T_armtip_camera' if wrist else 'T_base_camera'
                result = {key: np.eye(4).tolist(), 'camera_fit': {'reprojection_rms_px': 0.1}}
                with patch.object(entry.collect_data, 'main', side_effect=capture), patch.object(entry, 'compute', return_value=result) as compute:
                    entry.main(camera, ['--output-dir', str(directory)])
                    compute.assert_called_once_with(directory, None, entry.DEFAULT_METHOD, eye_in_hand=wrist)
                output = directory / ('eye_in_hand_result.json' if wrist else 'eye_to_hand_result.json')
                self.assertEqual(json.loads(output.read_text())[key], np.eye(4).tolist())

    def test_wrist_compute_recovers_camera_to_armtip(self):
        def transform(angles, translation):
            t = np.eye(4)
            t[:3, :3] = Rotation.from_euler('xyz', angles).as_matrix()
            t[:3, 3] = translation
            return t
        x = transform([0.2, -0.3, 0.7], [.02, -.03, .1])
        y = transform([.3, .4, -.5], [.4, .1, .6])
        rng = np.random.default_rng(27)
        poses = rng.uniform(-1, 1, (10, 6))
        arms = [transform(p[3:], p[:3]) for p in poses]
        boards = [solver.inverse(x) @ solver.inverse(arm) @ y for arm in arms]
        rvecs = [Rotation.from_matrix(b[:3, :3]).as_rotvec().reshape(3, 1) for b in boards]
        tvecs = [b[:3, 3].reshape(3, 1) for b in boards]
        with tempfile.TemporaryDirectory() as tmp:
            images = [Path(tmp) / f'{i+1}.jpg' for i in range(len(poses))]
            for image in images:
                image.write_bytes(b'fake image; decoded by mock')
            points = np.zeros((54, 1, 2), np.float32)
            with patch.object(solver, 'load_samples', return_value=(images, poses)), patch.object(solver, 'load_board', return_value=(9, 6, .00746)), patch.object(solver.cv2, 'imread', return_value=np.zeros((480, 640, 3), np.uint8)), patch.object(solver.cv2, 'findChessboardCorners', return_value=(True, points)), patch.object(solver.cv2, 'cornerSubPix', return_value=points), patch.object(solver.cv2, 'calibrateCamera', return_value=(0.1, np.eye(3), np.zeros(5), rvecs, tvecs)), patch.object(solver.cv2, 'projectPoints', return_value=(points, None)):
                result = solver.compute(tmp, method='linear', eye_in_hand=True)
        np.testing.assert_allclose(result['T_armtip_camera'], x, atol=1e-12)
        np.testing.assert_allclose(result['T_base_board'], y, atol=1e-12)
        self.assertNotIn('T_base_camera', result)
        self.assertFalse(result['independent_validation_recorded'])


if __name__ == '__main__':
    unittest.main()
