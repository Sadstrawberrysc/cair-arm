"""Exercise the production ordering function without importing camera/GPU drivers."""
import ast
from pathlib import Path
import unittest

import numpy as np


source = Path(__file__).resolve().parents[1] / "cliff_demo.py"
tree = ast.parse(source.read_text(encoding="utf-8"))
function = next(node for node in tree.body
                if isinstance(node, ast.FunctionDef) and node.name == "bottom_first_pixels")
scope = {"np": np}
exec(compile(ast.Module(body=[function], type_ignores=[]), str(source), "exec"), scope)
bottom_first_pixels = scope["bottom_first_pixels"]


class SampleOrderTests(unittest.TestCase):
    def test_bottom_is_maximum_image_y_with_pairs_preserved(self):
        pixels = np.array([[80., 30.], [10., 250.], [50., 100.]])
        np.testing.assert_array_equal(bottom_first_pixels(pixels),
                                      [[10., 250.], [50., 100.], [80., 30.]])
        np.testing.assert_array_equal(pixels[0], [80., 30.])

    def test_equal_rows_keep_original_order(self):
        np.testing.assert_array_equal(bottom_first_pixels([[9., 20.], [1., 20.]]),
                                      [[9., 20.], [1., 20.]])

    def test_invalid_pixels_rejected(self):
        for pixels in ([], [[1, 2, 3]], [[1, float("nan")]]):
            with self.assertRaises(ValueError):
                bottom_first_pixels(pixels)
