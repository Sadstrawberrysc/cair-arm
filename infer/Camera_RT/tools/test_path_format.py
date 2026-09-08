#!/usr/bin/env python3
"""Format characterization tests for legacy Camera_RT path fixtures."""

import math
from pathlib import Path
import unittest


FIXTURES = Path(__file__).resolve().parent / "fixtures"


def load_path(path: Path):
    rows = []
    for line_number, raw_line in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
        fields = raw_line.split()
        if len(fields) != 3:
            raise ValueError("{}:{} must contain three fields".format(path.name, line_number))
        values = tuple(float(field) for field in fields)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("{}:{} contains a non-finite value".format(path.name, line_number))
        rows.append(values)
    return rows


class LegacyPathFormatTest(unittest.TestCase):
    def test_legacy_fixtures_have_normal_and_points(self):
        for path in sorted(FIXTURES.glob("legacy_*_path.txt")):
            with self.subTest(path=path.name):
                rows = load_path(path)
                self.assertGreaterEqual(len(rows), 3)
                normal_length = math.sqrt(sum(value * value for value in rows[0]))
                self.assertAlmostEqual(normal_length, 1.0, places=3)
                self.assertTrue(all(len(point) == 3 for point in rows[1:]))

    def test_both_legacy_formats_are_represented(self):
        names = {path.name for path in FIXTURES.glob("legacy_*_path.txt")}
        self.assertEqual(
            names,
            {
                "legacy_old_machine_camera_path.txt",
                "legacy_old_machine_rbt_named_path.txt",
            },
        )


if __name__ == "__main__":
    unittest.main()
