import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from transform_point import TransformError, load_transform, transform_point


class TransformPointTests(unittest.TestCase):
    def test_transform_point(self):
        transform = np.eye(4)
        transform[:3, 3] = [1.0, 2.0, 3.0]
        result = transform_point([0.1, 0.2, 0.3], transform)
        self.assertTrue(np.allclose(result, [1.1, 2.2, 3.3]))

    def test_unconfirmed_calibration_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "calibration.json"
            path.write_text(
                json.dumps(
                    {
                        "transform": "d455_color_optical_to_rm75_base",
                        "translation_unit": "m",
                        "user_confirmed": False,
                        "T_base_camera": np.eye(4).tolist(),
                    }
                ),
                encoding="utf-8",
            )
            with self.assertRaises(TransformError):
                load_transform(path)


if __name__ == "__main__":
    unittest.main()
