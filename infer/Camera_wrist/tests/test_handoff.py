"""Exercise the coarse launcher with fake camera/robot/Redis, never hardware."""
import argparse
import contextlib
import io
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

import numpy as np

CALIBRATION_DIR = Path(__file__).resolve().parents[2]/"hand_eye_calibration-main"
sys.path.insert(0, str(CALIBRATION_DIR))
import run_probe_target as launcher


class HandoffTests(unittest.TestCase):
    def exercise(self, code=0, dry_run=False, fail_camera=False, mutate=False):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            camera = root/"infer/Camera_RT"
            robot = root/"infer/Robot/build"
            camera.mkdir(parents=True)
            robot.mkdir(parents=True)
            binary = robot/"arm_probe_pose"
            binary.touch(); binary.chmod(0o700)
            (robot/"rm75_force_calibration.json").write_text("{}")
            global_path = root/"global.json"
            wrist_path = root/"wrist.json"
            for path, name, key in [(global_path,"d455_color_optical_to_rm75_base","T_base_camera"),
                                     (wrist_path,"gemini305_color_optical_to_rm75_armtip","T_armtip_camera")]:
                path.write_text(json.dumps(dict(schema_version=1, transform=name, user_confirmed=True,
                    transform_convention="T_A_B maps coordinates from frame B to frame A",
                    translation_unit="m", **{key:np.eye(4).tolist()})))
            args = argparse.Namespace(wrist_handoff=True, calibration=global_path,
                wrist_calibration=wrist_path, wrist_serial="serial", dry_run=dry_run, velocity=3)
            calls=[]
            class Client:
                def delete(self, key): calls.append(("delete", key))
                def set(self, key, value, ex): calls.append(("set", key, json.loads(value), ex))
            client=Client()
            commands=[]
            def runner(command, **kwargs):
                commands.append(command)
                if len(commands)==1:
                    (camera/"artery_path.txt").write_text("0 0 -1\n0.1 0.2 1\n0.2 0.2 1\n")
                    return types.SimpleNamespace(returncode=1 if fail_camera else 0)
                if mutate:
                    global_path.write_text(global_path.read_text()+"\n")
                return types.SimpleNamespace(returncode=code)
            with patch.dict(sys.modules, {"redis":types.SimpleNamespace(Redis=lambda **kwargs:client)}):
                error=None
                try:
                    with contextlib.redirect_stdout(io.StringIO()):
                        result=launcher.run(args, root=root, runner=runner)
                except ValueError as exception:
                    error=exception; result=None
            return calls, commands, result, error

    def test_success_publishes_surface_after_robot_exit(self):
        calls,commands,result,error=self.exercise()
        self.assertIsNone(error)
        self.assertEqual(result,0)
        self.assertEqual(calls[0][0],"delete")
        seed=calls[1][2]
        np.testing.assert_allclose(seed["surface_point_base_m"],[.1,.2,1])
        self.assertEqual(calls[1][3],60)
        pose=np.fromstring(commands[1][commands[1].index("--target-pose-m-rad")+1],sep=",")
        np.testing.assert_allclose(pose[:3],[.1,.2,.95])

    def test_dry_run_never_publishes(self):
        calls,commands,result,error=self.exercise(dry_run=True)
        self.assertEqual([c[0] for c in calls],["delete"])
        self.assertNotIn("--execute",commands[1])

    def test_failed_motion_never_publishes(self):
        calls,commands,result,error=self.exercise(code=3)
        self.assertEqual(result,3)
        self.assertEqual([c[0] for c in calls],["delete"])

    def test_failed_snapshot_invalidates_old_seed(self):
        calls,commands,result,error=self.exercise(fail_camera=True)
        self.assertIsNotNone(error)
        self.assertEqual(len(commands),1)
        self.assertEqual([c[0] for c in calls],["delete"])

    def test_changed_calibration_never_publishes(self):
        calls,commands,result,error=self.exercise(mutate=True)
        self.assertIsNotNone(error)
        self.assertEqual([c[0] for c in calls],["delete"])
        self.assertEqual(len(commands),2)


if __name__ == "__main__":
    unittest.main()
