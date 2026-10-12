"""Regression tests for depth backprojection without importing camera drivers."""
import ast
from pathlib import Path
from types import SimpleNamespace
import unittest

import numpy as np


source = Path(__file__).resolve().parents[1] / "cliff_demo.py"
tree = ast.parse(source.read_text(encoding="utf-8"))
function = next(node for node in tree.body
                if isinstance(node, ast.FunctionDef) and node.name == "kpt_2d_to_3d")
scope = {"np": np}
exec(compile(ast.Module(body=[function], type_ignores=[]), str(source), "exec"), scope)
kpt_2d_to_3d = scope["kpt_2d_to_3d"]


class DepthProjectionTests(unittest.TestCase):
    def setUp(self):
        self.intrinsics = SimpleNamespace(fx=100., fy=200., ppx=1., ppy=1.)

    def project(self, pixels, depth):
        return kpt_2d_to_3d(pixels, depth, self.intrinsics, 0.001)

    def test_valid_z16_depth_preserves_fractional_pixel_ray(self):
        depth = np.full((3, 4), 2000, dtype=np.uint16)
        np.testing.assert_allclose(self.project([[1., 1.], [2.5, 2.]], depth),
                                   [[0., 0., 2.], [0.03, 0.01, 2.]])

    def test_zero_depth_uses_nearest_depth_on_original_ray(self):
        depth = np.zeros((3, 5), dtype=np.uint16)
        depth[1, 2] = 2000
        depth[2, 4] = 4000
        np.testing.assert_allclose(self.project([[1., 1.], [4., 1.]], depth),
                                   [[0., 0., 2.], [0.12, 0., 4.]])

    def test_fallback_accepts_uint8_and_uint16_without_opencv(self):
        for dtype in (np.uint8, np.uint16):
            with self.subTest(dtype=dtype):
                depth = np.zeros((3, 4), dtype=dtype)
                depth[0, 3] = 100
                np.testing.assert_allclose(self.project([[1., 1.]], depth),
                                           [[0., 0., 0.1]])

    def test_all_zero_depth_refuses_target(self):
        with self.assertRaisesRegex(ValueError, "没有有效深度"):
            self.project([[1., 1.]], np.zeros((3, 4), dtype=np.uint16))

    def test_depth_lookup_clamps_lower_and_upper_image_bounds(self):
        depth = np.full((3, 4), 9000, dtype=np.uint16)
        depth[0, 0] = 1000
        depth[2, 3] = 3000
        np.testing.assert_allclose(self.project([[-1., -1.], [4., 3.]], depth),
                                   [[-0.02, -0.01, 1.], [0.09, 0.03, 3.]])

    def test_invalid_pixels_and_depth_rejected(self):
        depth = np.ones((3, 4), dtype=np.uint16)
        for pixels in ([[1., float("nan")]], [[1., 2., 3.]], [1., 2.]):
            with self.subTest(pixels=pixels), self.assertRaises(ValueError):
                self.project(pixels, depth)
        for depth in (np.zeros((0, 0)), np.zeros(3)):
            with self.subTest(shape=depth.shape), self.assertRaises(ValueError):
                self.project([[1., 1.]], depth)


if __name__ == "__main__":
    unittest.main()
