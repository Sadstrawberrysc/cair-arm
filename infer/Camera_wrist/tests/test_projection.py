import copy
import json
from pathlib import Path
import sys
import tempfile
import time
import unittest
from unittest.mock import patch, Mock

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from wrist_projection import (make_seed, validate_seed, project, depth_at,
                              interpolate_pose, rigid, calibration, FRAME)
from projection_preview import CaptureClock, aligned_rgbd


class ProjectionTests(unittest.TestCase):
    def test_alignment_warmup_missing_output_then_complete(self):
        align = Mock()
        color, depth = object(), object()
        complete = Mock()
        complete.as_frame_set.return_value.get_color_frame.return_value = color
        complete.as_frame_set.return_value.get_depth_frame.return_value = depth
        incomplete = Mock()
        incomplete.as_frame_set.return_value.get_depth_frame.return_value = None
        empty = Mock()
        empty.as_frame_set.return_value = None
        align.process.side_effect = [None, empty, incomplete, complete]
        self.assertEqual(aligned_rgbd(None, align), (None, None, "NO FRAMES"))
        align.process.assert_not_called()
        for _ in range(3):
            c, d, reason = aligned_rgbd(object(), align)
            self.assertIsNone(c)
            self.assertIsNone(d)
            self.assertIsNotNone(reason)
        self.assertEqual(aligned_rgbd(object(), align), (color, depth, None))

    def setUp(self):
        self.hashes = {"global": "a"*64, "wrist": "b"*64}
        self.seed = make_seed([.1, .2, 1], np.eye(3), self.hashes, 1, "serial")
        self.now = self.seed["timestamp_monotonic_ns"]
        self.k = np.array([[100., 0, 320], [0, 100, 240], [0, 0, 1]])

    def state(self, sequence, offset):
        return dict(version=1, session_id=self.seed["session_id"], target_id=self.seed["target_id"],
                    boot_id=self.seed["boot_id"], calibration_sha256=self.hashes, valid=True,
                    runtime_session_id="run", sequence=sequence, timestamp_monotonic_ns=self.now+offset,
                    T_base_camera=np.eye(4).tolist(), frame=FRAME, length_unit="m")

    def test_seed_is_surface_not_offset(self):
        np.testing.assert_equal(validate_seed(self.seed, self.hashes, "serial", self.now), [.1, .2, 1])

    def test_seed_expiry_serial_and_digest(self):
        for hashes, serial, now in [(self.hashes, "bad", self.now), ({}, "serial", self.now),
                                    (self.hashes, "serial", self.now+60_000_000_000),
                                    (self.hashes, "serial", self.now-1)]:
            with self.assertRaises(ValueError):
                validate_seed(self.seed, hashes, serial, now)

    def test_projection_inverse_and_rejections(self):
        t=np.eye(4)
        t[:3,3]=[.1,.2,.3]
        pixel, point=project([.1,.2,1.3], t, self.k, np.zeros(5), 640, 480)
        np.testing.assert_allclose(pixel, [320,240])
        np.testing.assert_allclose(point, [0,0,1])
        for value in ([0,0,-1], [4,0,1], [float("nan"),0,1]):
            with self.assertRaises(ValueError):
                project(value, np.eye(4), self.k, np.zeros(5),640,480)

    def test_invalid_depth_no_neighbour_fallback(self):
        depth=np.ones((480,640))
        depth[10,10]=0
        with self.assertRaises(ValueError):
            depth_at(depth,[10.1,10.1])
        self.assertEqual(depth_at(depth,[639.9,479.9]),1)

    def test_pose_interpolation_and_timing_gates(self):
        a,b=self.state(1,0),self.state(2,20_000_000)
        b["T_base_camera"][0][3]=.02
        pose,span=interpolate_pose([a,b], self.now+10_000_000,self.seed,self.hashes)
        self.assertAlmostEqual(pose[0,3],.01)
        self.assertEqual(span,20)
        for key,value in [("runtime_session_id","restart"),("valid",False),("sequence",1),
                          ("timestamp_monotonic_ns",self.now+51_000_000), ("target_id","other")]:
            altered=copy.deepcopy(b)
            altered[key]=value
            with self.assertRaises(ValueError):
                interpolate_pose([a,altered], self.now+10_000_000,self.seed,self.hashes)
        invalid=self.state(2,10_000_000)
        invalid["valid"]=False
        with self.assertRaises(ValueError):
            interpolate_pose([a,invalid,b],self.now+5_000_000,self.seed,self.hashes)
        with self.assertRaises(ValueError):
            interpolate_pose([a,b],self.now+30_000_000,self.seed,self.hashes)

    def test_non_rigid_transform(self):
        t=np.eye(4); t[0,0]=-1
        with self.assertRaises(ValueError):
            rigid(t)

    def test_capture_clock_never_uses_receipt_for_old_frames(self):
        clock=CaptureClock()
        wall=1_700_000_000_000_000_000
        mono=100_000_000_000
        for index in range(31):
            with patch("projection_preview.time.time_ns",return_value=wall+index*30_000_000), \
                 patch("projection_preview.time.monotonic_ns",return_value=mono+index*30_000_000):
                if index<29:
                    with self.assertRaises(ValueError):
                        clock.convert((wall+index*30_000_000-10_000_000)//1000)
                else:
                    self.assertEqual(clock.convert((wall+index*30_000_000-10_000_000)//1000),
                                     mono+index*30_000_000-10_000_000)
        with patch("projection_preview.time.time_ns",return_value=wall+2_000_000_000), \
             patch("projection_preview.time.monotonic_ns",return_value=mono+2_000_000_000):
            with self.assertRaises(ValueError):
                clock.convert((wall+30*30_000_000-10_000_000)//1000)


if __name__ == "__main__":
    unittest.main()
