"""Synthetic RGB-D checks distinguish holes, discontinuities and image boundaries."""
import json
from pathlib import Path
import sys
import tempfile
import unittest
import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from depth_diagnostics import DepthNeighborhoodError, depth_cross
from local_tracker import LocalTracker, surface
from model_selection import replay_selection
from tracking_diagnostics import TrackingDiagnostics


class DepthDiagnosticTests(unittest.TestCase):
    def setUp(self):
        self.k = np.array([[400., 0, 160], [0, 400., 120], [0, 0, 1.]])
        self.distortion = np.zeros(5)
        self.depth = np.full((240, 320), .2)
        rng = np.random.default_rng(7)
        gray = cv2.GaussianBlur(rng.integers(0, 256, (240, 320), dtype=np.uint8), (3, 3), 0)
        self.image = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

    def test_four_directions_match_the_exact_pixels_checked(self):
        for name, x, y in (('left', 159, 120), ('right', 161, 120),
                           ('up', 160, 119), ('down', 160, 121)):
            with self.subTest(direction=name):
                depth = self.depth.copy()
                depth[y, x] = .237
                surface(depth, [160.9, 120.8], self.k, self.distortion)
                data = depth_cross(depth, [160.9, 120.8], .030)
                self.assertEqual(data['target']['pixel'], [160, 120])
                self.assertEqual(data['neighbors'][name]['pixel'], [x, y])
                self.assertEqual(data['neighbor_issues'], ['neighbor_depth_jump:'+name])
                self.assertTrue(data['continuity_passed'])
                self.assertAlmostEqual(data['neighbors'][name]['absolute_delta_m'], .037)
                self.assertEqual(data['depth_jump_limit_m'], .030)

    def test_holes_and_nonfinite_depths_are_distinguished_and_json_safe(self):
        for value, kind in ((0., 'zero'), (-.1, 'negative'),
                            (float('nan'), 'nonfinite'), (float('inf'), 'nonfinite')):
            with self.subTest(value=value):
                depth = self.depth.copy()
                depth[120, 159] = value
                depth[119, 160] = depth[121, 160] = 0
                with self.assertRaises(DepthNeighborhoodError) as caught:
                    surface(depth, [160, 120], self.k, self.distortion)
                data = caught.exception.depth_neighborhood
                self.assertIn('neighbor_invalid_depth:left', data['failure_causes'])
                self.assertEqual(data['continuous_neighbor_count'], 1)
                self.assertEqual(data['neighbors']['left']['invalid_kind'], kind)
                json.dumps(data, allow_nan=False)

    def test_invalid_target_and_border_have_their_own_causes(self):
        self.depth[120, 160] = 0
        with self.assertRaises(DepthNeighborhoodError) as caught:
            surface(self.depth, [160, 120], self.k, self.distortion)
        data = caught.exception.depth_neighborhood
        self.assertIn('target_invalid_depth', data['failure_causes'])
        self.assertEqual(data['target']['depth_m'], 0.)
        border = depth_cross(self.depth, [0, 120], .030)
        self.assertIn('neighbor_invalid_depth:left', border['neighbor_issues'])
        self.assertTrue(border['target_on_image_border'])
        self.assertTrue(border['continuity_passed'])
        self.assertIsNone(border['neighbors']['left']['depth_m'])

    def test_jump_threshold_uses_same_strict_comparison(self):
        depth = np.full_like(self.depth, .125)
        depth[120, 161] = .25
        depth[119, 160] = depth[120, 159] = 0
        data = depth_cross(depth, [160, 120], .125)
        self.assertFalse(data['neighbors']['right']['continuity_passed'])
        self.assertIn('neighbor_depth_jump:right', data['failure_causes'])

    def test_twenty_nine_mm_neighbor_is_accepted_but_thirty_one_is_rejected(self):
        for relaxed in (False, True):
            with self.subTest(relaxed=relaxed):
                depth = self.depth.copy()
                depth[119, 160] = depth[120, 159] = 0
                depth[120, 161] = .229
                point, _, _ = surface(depth, [160, 120], self.k, self.distortion, relaxed=relaxed)
                self.assertAlmostEqual(point[2], .2)
                depth[120, 161] = .231
                with self.assertRaises(DepthNeighborhoodError) as caught:
                    surface(depth, [160, 120], self.k, self.distortion, relaxed=relaxed)
                data = caught.exception.depth_neighborhood
                self.assertEqual(data['depth_jump_limit_m'], .030)
                self.assertIn('neighbor_depth_jump:right', data['failure_causes'])
                self.assertEqual(data['continuous_neighbor_count'], 1)

    def test_loss_log_and_evidence_keep_the_failing_frame_depths(self):
        tracker = LocalTracker()
        initial = tracker.initialize(self.image, self.depth, [160, 120], self.k, self.distortion, 1)
        depth = self.depth.copy()
        depth[120, 159] = 0
        depth[120, 161] = .237
        depth[119, 160] = 0
        with tempfile.TemporaryDirectory() as directory:
            rows = []
            writer = TrackingDiagnostics(Path(directory)/'run.jsonl', rows.append)
            writer.emit(initial, tracker)
            failed = tracker.update(self.image, depth, self.k, self.distortion, 33_000_001)
            writer.emit(failed, tracker)
            self.assertEqual(rows[-1]['event'], 'lost')
            data = rows[-1]['depth_neighborhood']
            self.assertEqual(data['capture_monotonic_ns'], 33_000_001)
            self.assertEqual(data['failure_causes'],
                             ['insufficient_continuous_neighbors', 'neighbor_invalid_depth:left',
                              'neighbor_depth_jump:right', 'neighbor_invalid_depth:up'])
            saved = json.loads((writer.directory/'event_001/details.json').read_text())
            self.assertEqual(saved['diagnostics']['depth_neighborhood'], data)
            json.dumps(rows, allow_nan=False)

    def test_two_good_neighbors_are_enough_but_one_is_not(self):
        positions = [(120, 159), (120, 161), (119, 160), (121, 160)]
        for relaxed in (False, True):
            for invalid_count in range(5):
                with self.subTest(relaxed=relaxed, invalid_count=invalid_count):
                    depth = self.depth.copy()
                    for y, x in positions[:invalid_count]:
                        depth[y, x] = 0
                    diagnostic = depth_cross(depth, [160, 120], .030)
                    self.assertEqual(diagnostic['continuous_neighbor_count'], 4-invalid_count)
                    self.assertEqual(diagnostic['minimum_continuous_neighbors'], 2)
                    if invalid_count <= 2:
                        surface(depth, [160, 120], self.k, self.distortion, relaxed=relaxed)
                        self.assertTrue(diagnostic['continuity_passed'])
                        self.assertEqual(diagnostic['failure_causes'], [])
                    else:
                        with self.assertRaises(DepthNeighborhoodError):
                            surface(depth, [160, 120], self.k, self.distortion, relaxed=relaxed)
                        self.assertIn('insufficient_continuous_neighbors', diagnostic['failure_causes'])

    def test_image_corner_counts_only_real_neighbors_without_wraparound(self):
        surface(self.depth, [0, 0], self.k, self.distortion)
        self.depth[0, 1] = 0
        with self.assertRaises(DepthNeighborhoodError) as caught:
            surface(self.depth, [0, 0], self.k, self.distortion)
        self.assertEqual(caught.exception.depth_neighborhood['continuous_neighbor_count'], 1)

    def test_model_replay_preserves_initialization_and_later_frame_failure_details(self):
        bad = self.depth.copy()
        bad[120, 159] = bad[119, 160] = bad[121, 160] = 0
        for source, replay in ((bad, self.depth), (self.depth, bad)):
            with self.subTest(initialization_fails=source is bad):
                tracker = LocalTracker()
                with self.assertRaises(DepthNeighborhoodError) as caught:
                    replay_selection(tracker, (self.image, source, 1),
                                     [(self.image, replay, 33_000_001)],
                                     dict(scan_start=[160, 120]), self.k, self.distortion)
                data = caught.exception.depth_neighborhood
                self.assertIn('neighbor_invalid_depth:left', data['failure_causes'])
                self.assertEqual(data['capture_monotonic_ns'], 1 if source is bad else 33_000_001)


if __name__ == '__main__':
    unittest.main()
