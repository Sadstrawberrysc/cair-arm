"""Yellow display flow is independent of valid metric observations and Robot control."""
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from local_tracker import ImagePointTracker, LocalTracker
from model_selection import observation_is_fresh
from tracking_display import point_display


class TrackingDisplayTests(unittest.TestCase):
    def setUp(self):
        rng = np.random.default_rng(7)
        gray = cv2.GaussianBlur(rng.integers(0, 256, (240, 320), dtype=np.uint8), (3, 3), 0)
        self.image = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        self.depth = np.full((240, 320), .2)
        self.k = np.array([[400., 0, 160], [0, 400., 120], [0, 0, 1.]])
        self.tracker, self.visual = LocalTracker(), ImagePointTracker()
        self.observation = self.tracker.initialize(self.image, self.depth, [160, 120],
                                                   self.k, np.zeros(5), 1_000_000_000)
        self.assertTrue(self.visual.adopt(self.tracker, self.observation))
        self.session = SimpleNamespace(following=True, target='target', producer='camera',
            last_request_sequence=4, state=dict(timestamp_monotonic_ns=1_000_000_000,
                follow_state='following', target_id='target', request_producer_id='camera',
                request_sequence=4, control_state='following'))

    def marker(self, now=1_033_000_000):
        return point_display(self.visual.result_2d(), self.observation, self.session, now)

    def test_green_requires_fresh_3d_and_matching_robot_follow_ack(self):
        self.assertEqual(self.marker()['color'], (0, 255, 0))
        for key, value in (('follow_state', 'hold'), ('target_id', 'foreign'),
                           ('request_producer_id', 'other-camera'), ('request_sequence', 3),
                           ('control_state', 'fault'), ('timestamp_monotonic_ns', 1)):
            with self.subTest(key=key):
                original = self.session.state[key]
                self.session.state[key] = value
                self.assertEqual(self.marker()['color'], (0, 255, 255))
                self.session.state[key] = original
        self.session.following = False
        self.session.target = ''
        self.assertEqual(self.marker()['color'], (0, 255, 255))
        self.assertTrue(self.tracker.active)

    def test_depth_loss_continues_checked_2d_flow_without_valid_robot_target(self):
        moved = cv2.warpAffine(self.image, np.array([[1., 0, 3], [0, 1., 2]]), (320, 240))
        self.observation = self.tracker.update(moved, np.zeros_like(self.depth), self.k,
                                               np.zeros(5), 1_033_000_000)
        self.assertFalse(self.observation['valid'])
        self.assertFalse(self.tracker.active)
        result = self.visual.update(moved, 1_033_000_000)
        self.assertTrue(result['image_valid'])
        np.testing.assert_allclose(result['pixel'], [163, 122], atol=.3)
        self.assertNotIn('valid', result)
        self.assertNotIn('point_camera_m', result)
        self.assertFalse(observation_is_fresh(result, 1_034_000_000))
        self.assertEqual(self.marker()['color'], (0, 255, 255))

    def test_stale_3d_and_current_2d_is_yellow_but_stale_2d_is_hidden(self):
        self.visual.update(self.image, 1_100_000_000)
        self.assertEqual(self.marker(now=1_150_000_000)['color'], (0, 255, 255))
        self.assertIsNone(self.marker(now=1_400_000_000))

    def test_real_flow_loss_or_missing_image_does_not_leave_a_frozen_point(self):
        result = self.visual.update(np.zeros_like(self.image), 1_033_000_000)
        self.assertFalse(result['image_valid'])
        self.assertIsNone(self.marker())
        self.assertFalse(self.visual.adopt(self.tracker, dict(valid=False)))

    def test_display_tracking_never_changes_robot_tracker_reference(self):
        original = self.tracker.pixel.copy()
        self.visual.update(self.image, 1_033_000_000)
        self.visual.lose('display lost')
        np.testing.assert_array_equal(self.tracker.pixel, original)
        self.assertTrue(self.tracker.active)


if __name__ == '__main__':
    unittest.main()
