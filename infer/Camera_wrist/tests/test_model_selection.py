import sys
from pathlib import Path
import unittest
import time

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from local_tracker import LocalTracker
from click_follow import ClickSession, draw_detection_boxes
from wrist_projection import boot_id, FRAME
from model_selection import (DetectionCenterFusion, fuse_model_observation,
                             replay_selection, observation_is_fresh)


class ModelSelectionTests(unittest.TestCase):
    def setUp(self):
        self.k = np.array([[400., 0, 160], [0, 400., 120], [0, 0, 1.]])
        self.distortion = np.zeros(5)
        rng = np.random.default_rng(7)
        gray = cv2.GaussianBlur(rng.integers(0, 256, (240, 320), dtype=np.uint8), (3, 3), 0)
        self.image = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        self.depth = np.full((240, 320), .2)
        self.prediction = {'scan_start': [155, 117]}

    def frame(self, dx, dy, timestamp):
        image = cv2.warpAffine(self.image, np.array([[1., 0, dx], [0, 1., dy]]), (320, 240))
        return image, self.depth, timestamp

    def test_preview_draws_recent_box_but_not_stale_box(self):
        response = dict(capture_monotonic_ns=1_000_000_000,
                        image_size=[320, 240],
                        detections=[dict(x=160, y=120, width=40, height=30,
                                         confidence=.8, **{'class': 'carotid'})],
                        valid=False, reason='more than one candidate')
        recent = np.zeros((240, 320, 3), np.uint8)
        self.assertTrue(draw_detection_boxes(recent, response, 1_200_000_000))
        self.assertEqual(tuple(recent[105, 140]), (0, 165, 255))
        stale = np.zeros_like(recent)
        self.assertFalse(draw_detection_boxes(stale, response, 1_600_000_000))
        self.assertFalse(stale.any())

    def test_replay_tracks_model_point_to_fresh_frame(self):
        tracker = LocalTracker()
        initial = self.frame(0, 0, 1)
        frames = [self.frame(1, 1, 33_000_001), self.frame(3, 2, 66_000_001)]
        result = replay_selection(tracker, initial, frames, self.prediction,
                                  self.k, self.distortion)
        self.assertTrue(result['valid'])
        self.assertEqual(result['capture_monotonic_ns'], 66_000_001)
        np.testing.assert_allclose(result['pixel'], [158, 119], atol=.4)

    def test_model_point_reaches_existing_robot_observation_contract(self):
        stamp = time.monotonic_ns()
        tracker = LocalTracker()
        result = replay_selection(tracker, self.frame(0, 0, stamp-66_000_000),
                                  [self.frame(1, 1, stamp-33_000_000)],
                                  self.prediction, self.k, self.distortion)
        self.assertTrue(observation_is_fresh(result, time.monotonic_ns()))
        session = ClickSession('hash')
        state = dict(version=1, scope='click_follow', boot_id=boot_id(),
                     calibration_sha256='hash', frame=FRAME, length_unit='m',
                     valid=True, timestamp_monotonic_ns=time.monotonic_ns(),
                     runtime_session_id='run', sequence=1, follow_state='hold')
        session.accept_state(state, time.monotonic_ns())
        session.select()
        packet = session.envelope()
        packet.update(result)
        self.assertEqual(packet['target_id'], session.target)
        self.assertEqual(packet['capture_monotonic_ns'], result['capture_monotonic_ns'])
        self.assertGreaterEqual(packet['quality']['feature_inliers'], 12)
        self.assertFalse(session.following)
        self.assertEqual(session.request('begin', True)['action'], 'begin')

    def test_detection_center_fusion_aligns_source_frame_before_correcting(self):
        tracker = LocalTracker()
        source = self.frame(0, 0, 1_000_000_000)
        current = self.frame(3, 2, 1_033_000_000)
        source_result = tracker.initialize(*source[:2], [155, 117], self.k,
                                           self.distortion, source[2])
        self.assertTrue(source_result['valid'])
        fusion = DetectionCenterFusion()
        fusion.remember(source_result, 'target-a')
        current_result = tracker.update(*current[:2], self.k, self.distortion, current[2])
        self.assertTrue(current_result['valid'])
        response = dict(valid=True, capture_monotonic_ns=source[2],
                        prediction={'scan_start': [157, 116]})
        aligned, reason = fusion.aligned_center(response, 'target-a', current_result,
                                                current[2]+80_000_000)
        self.assertIsNone(reason)
        np.testing.assert_allclose(aligned,
                                   np.asarray(current_result['pixel'])+[2, -1], atol=.001)
        corrected, reason = tracker.correct_from_detection(
            aligned, current[1], self.k, self.distortion, current[2])
        self.assertIsNone(reason)
        self.assertTrue(corrected['valid'])
        np.testing.assert_allclose(corrected['pixel'],
                                   np.asarray(current_result['pixel'])+[.5, -.25], atol=.001)
        self.assertEqual(corrected['capture_monotonic_ns'], current[2])
        fusion.remember(corrected, 'target-a')
        self.assertEqual(fusion.history[-1][1], tuple(corrected['pixel']))
        next_result = tracker.update(self.frame(4, 2, 1_066_000_000)[0],
                                     self.depth, self.k, self.distortion, 1_066_000_000)
        self.assertTrue(next_result['valid'], next_result)

    def test_fusion_aligns_large_offset_but_rejects_stale_and_previous_target(self):
        fusion = DetectionCenterFusion()
        current = dict(valid=True, capture_monotonic_ns=1_033_000_000,
                       pixel=[158, 119])
        fusion.remember(dict(valid=True, capture_monotonic_ns=1_000_000_000,
                             pixel=[155, 117]), 'target-a')
        jump = dict(valid=True, capture_monotonic_ns=1_000_000_000,
                    prediction={'scan_start': [180, 117]})
        aligned, reason = fusion.aligned_center(jump, 'target-a', current,
                                                1_100_000_000)
        self.assertIsNone(reason)
        np.testing.assert_allclose(aligned, [183, 119])
        normal = dict(jump, prediction={'scan_start': [157, 116]})
        self.assertIn('stale', fusion.aligned_center(normal, 'target-a', current,
                                                     1_600_000_000)[1])
        self.assertIn('active target', fusion.aligned_center(normal, 'target-b', current,
                                                             1_100_000_000)[1])
        fusion.remember(dict(valid=True, capture_monotonic_ns=1_100_000_000,
                             pixel=[160, 120]), 'target-b')
        self.assertIn('unavailable', fusion.aligned_center(normal, 'target-b', current,
                                                            1_100_000_000)[1])

    def test_large_model_offset_keeps_track_and_limits_correction(self):
        stamp = 1_000_000_000
        source = self.frame(0, 0, stamp)
        response = dict(valid=True, capture_monotonic_ns=stamp,
                        prediction={'scan_start': [190, 117]})
        tracker = LocalTracker()
        current = tracker.initialize(source[0], source[1], [155, 117],
                                     self.k, self.distortion, stamp)
        self.assertTrue(current['valid'])
        fusion = DetectionCenterFusion()
        fusion.remember(current, 'target-a')
        output, applied, reason, confirmed = fuse_model_observation(
            tracker, fusion, response, stamp, source, current,
            'target-a', self.k, self.distortion, stamp+50_000_000)
        self.assertTrue(output['valid'])
        self.assertTrue(tracker.active)
        self.assertTrue(applied)
        self.assertTrue(confirmed)
        self.assertIsNone(reason)
        np.testing.assert_allclose(output['pixel'], [157, 117], atol=.001)

    def test_fusion_rejects_response_from_different_capture(self):
        stamp = 1_000_000_000
        frame = self.frame(0, 0, stamp)
        tracker = LocalTracker()
        current = tracker.initialize(frame[0], frame[1], [155, 117],
                                     self.k, self.distortion, stamp)
        fusion = DetectionCenterFusion()
        fusion.remember(current, 'target-a')
        response = dict(valid=True, capture_monotonic_ns=stamp+1,
                        prediction={'scan_start': [155, 117]})
        output, applied, reason, confirmed = fuse_model_observation(
            tracker, fusion, response, stamp, frame, current, 'target-a',
            self.k, self.distortion, stamp+50_000_000)
        self.assertIs(output, current)
        self.assertFalse(applied or confirmed)
        self.assertIn('identity mismatch', reason)
        self.assertTrue(tracker.active)

    def test_gap_and_stale_result_fail_closed(self):
        tracker = LocalTracker()
        with self.assertRaisesRegex(ValueError, 'capture gap'):
            replay_selection(tracker, self.frame(0, 0, 1),
                             [self.frame(0, 0, 300_000_001)], self.prediction,
                             self.k, self.distortion)
        self.assertFalse(tracker.active)
        result = replay_selection(tracker, self.frame(0, 0, 1),
                                  [self.frame(0, 0, 33_000_001)], self.prediction,
                                  self.k, self.distortion)
        self.assertTrue(tracker.active)
        self.assertFalse(observation_is_fresh(result, 250_000_000))
        self.assertTrue(observation_is_fresh(result, 100_000_000))


if __name__ == '__main__':
    unittest.main()
