"""Exercise the production ordering function without importing camera/GPU drivers."""
import ast
import io
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

import numpy as np


source = Path(__file__).resolve().parents[1] / "cliff_demo.py"
tree = ast.parse(source.read_text(encoding="utf-8"))
function = next(node for node in tree.body
                if isinstance(node, ast.FunctionDef) and node.name == "top_first_pixels")
scope = {"np": np}
exec(compile(ast.Module(body=[function], type_ignores=[]), str(source), "exec"), scope)
top_first_pixels = scope["top_first_pixels"]


class SampleOrderTests(unittest.TestCase):
    def test_top_is_minimum_image_y_with_pairs_preserved(self):
        pixels = np.array([[80., 30.], [10., 250.], [50., 100.]])
        np.testing.assert_array_equal(top_first_pixels(pixels),
                                      [[80., 30.], [50., 100.], [10., 250.]])
        np.testing.assert_array_equal(pixels[0], [80., 30.])

    def test_equal_rows_keep_original_order(self):
        np.testing.assert_array_equal(top_first_pixels([[9., 20.], [1., 20.]]),
                                      [[9., 20.], [1., 20.]])

    def test_invalid_pixels_rejected(self):
        for pixels in ([], [[1, 2, 3]], [[1, float("nan")]]):
            with self.assertRaises(ValueError):
                top_first_pixels(pixels)

    def test_robot_reads_top_pixel_from_saved_camera_snapshot(self):
        projection = next(node for node in tree.body
                          if isinstance(node, ast.FunctionDef) and node.name == "kpt_2d_to_3d")
        robot_source = source.parent.parent / "hand_eye_calibration-main/run_probe_target.py"
        robot_tree = ast.parse(robot_source.read_text(encoding="utf-8"))
        readers = [node for node in robot_tree.body
                   if isinstance(node, ast.FunctionDef) and node.name in ("signature", "first_point")]
        scope = {"np": np, "io": io}
        exec(compile(ast.Module(body=[projection] + readers, type_ignores=[]),
                     str(source), "exec"), scope)

        # The top image pixel has a larger camera-space Y than the others:
        # selecting by 3D Y or taking the old bottom-first order would fail.
        pixels = top_first_pixels([[10., 250.], [80., 30.], [50., 100.]])
        depth = np.zeros((300, 100), dtype=np.uint16)
        depth[30, 80], depth[100, 50], depth[250, 10] = 2000, 100, 100
        intrinsics = SimpleNamespace(fx=100., fy=100., ppx=0., ppy=0.)
        points = scope["kpt_2d_to_3d"](pixels, depth, intrinsics, 0.001)
        with tempfile.TemporaryDirectory() as directory:
            snapshot = Path(directory) / "artery_path.txt"
            np.savetxt(snapshot, np.vstack(([0., 0., -1.], points)), fmt="%.6f")
            point, normal, tangent = scope["first_point"](snapshot, previous=None)
        np.testing.assert_allclose(point, [1.6, 0.6, 2.])
        np.testing.assert_allclose(normal, [0., 0., -1.])
        expected_tangent = np.array([-1.55, -0.5, -1.9])
        np.testing.assert_allclose(tangent, expected_tangent / np.linalg.norm(expected_tangent))
