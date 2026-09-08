import argparse
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from run_probe_target import build_parser, run


class WorkflowTests(unittest.TestCase):
    def simulate(self, mode):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            camera = root / "infer/Camera_RT"
            build = root / "infer/Robot/build"
            camera.mkdir(parents=True)
            build.mkdir(parents=True)
            robot = build / "arm_probe_pose"
            robot.touch()
            robot.chmod(0o700)
            (build / "rm75_force_calibration.json").touch()
            snapshot = camera / "artery_path.txt"
            snapshot.write_text("0 0 -1\n8 8 8\n")
            calls = []

            def runner(command, cwd):
                calls.append(command)
                if len(calls) == 1:
                    if mode != "unsaved":
                        snapshot.write_text("0 0 -1\n0.1 0.2 0.3\n0.4 0.5 0.6\n")
                    return argparse.Namespace(returncode=1 if mode == "failed" else 0)
                return argparse.Namespace(returncode=0)

            args = build_parser().parse_args(["--dry-run"] if mode == "dry" else [])
            transform = np.eye(4)
            transform[:3, 3] = [1, 2, 3]
            with patch("run_probe_target.load_transform", return_value=transform):
                if mode in ("unsaved", "failed"):
                    with self.assertRaises(ValueError):
                        run(args, root, runner)
                    self.assertEqual(len(calls), 1)
                    return
                self.assertEqual(run(args, root, runner), 0)
            self.assertEqual(calls[1][2], "1.100000000,2.200000000,3.300000000")
            self.assertEqual(calls[1][calls[1].index("--velocity") + 1], "1")
            self.assertEqual("--execute" in calls[1], mode == "execute")
            self.assertEqual("--confirm-single-movej" in calls[1], mode == "execute")

    def test_dry_run_first_point(self):
        self.simulate("dry")

    def test_execute_forwarding(self):
        self.simulate("execute")

    def test_old_snapshot_rejected(self):
        self.simulate("unsaved")

    def test_failed_camera_rejected(self):
        self.simulate("failed")
