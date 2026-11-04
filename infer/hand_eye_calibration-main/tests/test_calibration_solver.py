"""眼在手外坐标方向与退化输入的合成验证。"""
import sys
from pathlib import Path
import unittest
import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from calibration_solver import inverse, solve_transforms


def transform(angles, translation):
    t = np.eye(4)
    t[:3, :3] = Rotation.from_euler('xyz', angles).as_matrix()
    t[:3, 3] = translation
    return t


class EyeToHandTests(unittest.TestCase):
    def test_camera_to_base_and_unknown_board_mount_recovered(self):
        x = transform([.4, -.6, 1.2], [.3, -.4, .8])
        y = transform([-.7, .1, .8], [.02, -.03, .13])
        rng = np.random.default_rng(42)
        arms = [transform(rng.uniform(-1.5, 1.5, 3), rng.uniform(-.3, .3, 3)) for _ in range(20)]
        boards = [inverse(x) @ e @ y for e in arms]
        actual_x, actual_y, quality = solve_transforms(arms, boards, 'linear')
        np.testing.assert_allclose(actual_x, x, atol=1e-12)
        np.testing.assert_allclose(actual_y, y, atol=1e-12)
        self.assertLess(quality['board_position_max_mm'], 1e-8)
        self.assertLess(quality['board_rotation_max_deg'], 1e-8)

    def test_single_axis_rotations_rejected(self):
        arms = [transform([0, 0, .1*i], [.01*i, .1, .2]) for i in range(10)]
        with self.assertRaisesRegex(ValueError, '非平行'):
            solve_transforms(arms, arms, 'linear')

    def test_pair_count_mismatch_rejected(self):
        with self.assertRaises(ValueError):
            solve_transforms([np.eye(4)]*6, [np.eye(4)]*5, 'linear')


if __name__ == '__main__':
    unittest.main()
