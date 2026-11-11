"""Image-space Tool-Y regression tests; no camera, model weights or robot."""
from pathlib import Path
import ast
import json
import sys
import threading
import time
import uuid
from types import SimpleNamespace
import unittest
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from visual_y_alignment import VisualYAlignment, gate_alignment_command, fresh_visual_command


class AlignmentTests(unittest.TestCase):
    def test_actual_publisher_heartbeat_preserves_observation_age(self):
        source = Path(__file__).resolve().parents[1] / 'main_redis_seg_newphase_recovery_mode.py'
        tree = ast.parse(source.read_text())
        node = next(n for n in tree.body if isinstance(n, ast.ClassDef)
                    and n.name == 'VisionCommandPublisher')
        namespace = dict(threading=threading, time=time, uuid=uuid, json=json,
                         COMMAND_CHANNEL='test', fresh_visual_command=fresh_visual_command)
        exec(compile(ast.Module(body=[node], type_ignores=[]), str(source), 'exec'), namespace)
        class Client:
            def __init__(self): self.messages = []
            def publish(self, channel, payload): self.messages.append(json.loads(payload))
        client = Client()
        publisher = namespace['VisionCommandPublisher'](client)
        observed = time.monotonic_ns() - 1_000_000_000
        publisher.publish(.004, 5., False, True, phase_idx=2, robot_action_state='moving',
                          observation_monotonic_ns=observed)
        publisher.start_heartbeat()
        try:
            deadline = time.monotonic() + 2
            while len(client.messages) < 3 and time.monotonic() < deadline:
                time.sleep(.01)
        finally:
            publisher.stop_heartbeat()
        self.assertGreaterEqual(len(client.messages), 3)
        self.assertEqual(publisher._last_observation_monotonic_ns, observed)
        for held in client.messages[1:]:
            self.assertEqual(held['parameters'], dict(y=0., rz=0.))
            self.assertEqual(held['phase_idx'], -1)
            self.assertTrue(held['action_state'])

    def test_heartbeat_cannot_replay_stale_visual_motion(self):
        command = dict(parameters=dict(y=.004, rz=5.), phase_idx=2,
                       phase_confidence=.9, action_state=True, terminate=False,
                       recovery_mode=True, mask_lr_majority=2)
        self.assertIs(fresh_visual_command(command, 1_000_000_000, 1_100_000_000), command)
        held = fresh_visual_command(command, 1_000_000_000, 1_400_000_000)
        self.assertEqual(held['parameters'], dict(y=0., rz=0.))
        self.assertEqual(held['phase_idx'], -1)
        self.assertFalse(held['recovery_mode'])
        self.assertTrue(held['action_state'])  # Keep force controller enabled.
        self.assertEqual(command['parameters']['y'], .004)  # Does not mutate caller.
        command['terminate'] = True
        self.assertIs(fresh_visual_command(command, 1_000_000_000, 9_000_000_000), command)

    def setUp(self):
        self.control = VisualYAlignment(37)
        self.now = 1_000_000_000

    def update(self, cx=None, *, detection=None, advance_ns=100_000_000, regions=None):
        self.now += advance_ns
        if regions is None:
            regions = [] if cx is None else [dict(cx=cx, cy=275)]
        return self.control.update((640, 557, 3), regions, detection, self.now, self.now)

    def test_recorded_off_center_vessel_is_not_treated_as_point_six_mm(self):
        first = self.update(425.9)
        second = self.update(425.9)
        self.assertAlmostEqual(second['error_m'], -.00855046875)
        self.assertEqual(first['command_m'], 0)
        self.assertEqual(second['command_m'], -.004)
        self.assertGreater(-second['command_m'], 0)  # Robot direction=-1 -> +Tool-Y.

    def test_left_and_right_have_opposite_correct_directions(self):
        self.update(160)
        self.assertGreater(self.update(160)['command_m'], 0)
        self.control.reset()
        self.update(400)
        self.assertLess(self.update(400)['command_m'], 0)

    def test_center_noise_is_zero_and_reengage_has_hysteresis(self):
        for cx in [278, 282, 274, 285, 271, 288, 268] * 3:
            self.assertEqual(self.update(cx)['command_m'], 0)
        self.update(295)
        self.assertLess(self.update(295)['command_m'], 0)
        self.assertEqual(self.update(278)['command_m'], 0)

    def test_reversal_needs_two_frames(self):
        self.update(300)
        self.assertLess(self.update(300)['command_m'], 0)
        self.assertEqual(self.update(255)['command_m'], 0)
        self.assertGreater(self.update(255)['command_m'], 0)

    def test_missing_ambiguous_jumped_and_stale_targets_hold(self):
        self.assertFalse(self.update()['valid'])
        self.assertFalse(self.update(regions=[dict(cx=150, cy=200), dict(cx=400, cy=200)])['valid'])
        self.update(300)
        self.assertFalse(self.update(400)['valid'])
        self.assertTrue(self.update(400, advance_ns=600_000_000)['valid'])
        stale = self.control.update((640, 557, 3), [dict(cx=400, cy=200)], None,
                                    self.now - 400_000_000, self.now)
        self.assertFalse(stale['valid'])
        self.assertEqual(stale['command_m'], 0)

    def test_detector_associates_the_matching_segmentation(self):
        detection = dict(cx=399, cy=280, x1=370, x2=430, y1=250, y2=310)
        result = self.update(detection=detection,
                             regions=[dict(cx=160, cy=270), dict(cx=400, cy=275)])
        self.assertEqual(result['source'], 'segmentation+yolo')
        self.assertEqual(result['cx'], 400)
        result = self.update(detection=detection)
        self.assertEqual(result['source'], 'yolo')
        self.assertLess(result['command_m'], 0)  # Source change does not lose direction confirmation.

    def test_no_target_or_unconfirmed_direction_cannot_start_x_rz(self):
        command = (0., 5., 0, .9, False, 2)
        for result in [self.update(), self.update(420)]:
            self.assertEqual(gate_alignment_command(command, result), (0., 0., -1, 0., False, 0))
        recovered = (0., 0., 2, .9, True, 2)
        self.assertEqual(gate_alignment_command(recovered, self.update()), recovered)
        far = self.update(420, advance_ns=600_000_000)
        far = self.update(420)
        gated = gate_alignment_command((far['command_m'], 5., 1, .9, False, 2), far)
        self.assertEqual(gated, (far['command_m'], 0., -1, 0., False, 0))

    def test_yolo_uses_shared_depth_range_instead_of_old_200_350_band(self):
        source = Path(__file__).resolve().parents[1] / 'main_redis_seg_newphase_recovery_mode.py'
        tree = ast.parse(source.read_text())
        function = next(n for n in tree.body if isinstance(n, ast.FunctionDef)
                        and n.name == 'run_yolo_on_frame')
        namespace = {}
        exec(compile(ast.Module(body=[function], type_ignores=[]), str(source), 'exec'), namespace)
        box = SimpleNamespace(cls=np.array([0]), conf=np.array([.9]),
                              xyxy=np.array([[370, 70, 430, 130]]))
        class Detector:
            def predict(self, **kwargs):
                return [SimpleNamespace(boxes=[box])]
        detection, _ = namespace['run_yolo_on_frame'](
            np.zeros((640, 557, 3)), Detector(), valid_center_y_min=20, valid_center_y_max=500)
        self.assertEqual(detection['cy'], 100)
        detection, _ = namespace['run_yolo_on_frame'](
            np.zeros((640, 557, 3)), Detector(), valid_center_y_min=200, valid_center_y_max=350)
        self.assertIsNone(detection)

    def test_delayed_closed_loop_converges_from_both_sides_without_chatter(self):
        for start in (-.0085, .0085):
            self.control.reset()
            error = start
            previous_command = 0.
            signs = []
            for _ in range(200):
                # One frame of actuation delay. Probe motion shifts image oppositely.
                error -= previous_command * .1
                cx = 278 - error / (.037 / 640)
                result = self.update(cx)
                previous_command = result['command_m']
                if previous_command:
                    signs.append(1 if previous_command > 0 else -1)
                self.assertLessEqual(abs(previous_command), .004)
            self.assertLessEqual(abs(error), .0008)
            self.assertEqual(len(set(signs)), 1)
            self.assertEqual(previous_command, 0)


if __name__ == '__main__':
    unittest.main()
