import sys
from pathlib import Path
import unittest
import cv2
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from local_tracker import DisplayPointSmoother, LocalTracker, surface

class TrackerTests(unittest.TestCase):
    def setUp(self):
        self.k=np.array([[400.,0,160],[0,400.,120],[0,0,1.]])
        self.d=np.zeros(5)
        rng=np.random.default_rng(7)
        gray=cv2.GaussianBlur(rng.integers(0,256,(240,320),dtype=np.uint8),(3,3),0)
        self.image=cv2.cvtColor(gray,cv2.COLOR_GRAY2BGR)
        self.depth=np.full((240,320),.2)

    def test_display_smoother_reduces_jitter_and_resets_on_new_target(self):
        rng = np.random.default_rng(9)
        smoother = DisplayPointSmoother()
        raw_x, smooth_x = [], []
        for index in range(30):
            x = 160 + rng.normal(0, 1.5)
            result = dict(valid=True, pixel=[x, 120.],
                          point_camera_m=[x/4000., 0., .2],
                          capture_monotonic_ns=1_000_000_000+index*33_000_000)
            pixel, point = smoother.update(result, 'target-a')
            raw_x.append(x)
            smooth_x.append(pixel[0])
            self.assertAlmostEqual(point[0], pixel[0]/4000.)
        self.assertLess(np.std(smooth_x[10:]), np.std(raw_x[10:])*.8)
        step = dict(valid=True, pixel=[175., 120.],
                    point_camera_m=[175/4000., 0., .2],
                    capture_monotonic_ns=1_000_000_000+30*33_000_000)
        self.assertGreater(smoother.update(step, 'target-a')[0][0], 169.)
        self.assertEqual(smoother.update(step, 'target-b')[0], [175., 120.])

    def test_display_smoother_does_not_hold_stale_point(self):
        smoother = DisplayPointSmoother()
        initial = dict(valid=True, pixel=[100., 100.],
                       point_camera_m=[0., 0., .2], capture_monotonic_ns=1)
        self.assertIsNotNone(smoother.update(initial, 'target-a'))
        self.assertIsNone(smoother.update(dict(valid=False), 'target-a'))
        later = dict(valid=True, pixel=[110., 100.],
                     point_camera_m=[.005, 0., .2],
                     capture_monotonic_ns=300_000_001)
        self.assertEqual(smoother.update(later, 'target-a')[0], [110., 100.])

    def test_plane_normal_and_metric_depth(self):
        point,normal,q=surface(self.depth,[160,120],self.k,self.d)
        np.testing.assert_allclose(point,[0,0,.2],atol=1e-6)
        np.testing.assert_allclose(normal,[0,0,-1],atol=1e-6)
        self.assertGreaterEqual(q['surface_points'],50)

    def test_translation_tracks_selected_point_not_feature_mean(self):
        tracker=LocalTracker()
        self.assertTrue(tracker.initialize(self.image,self.depth,[155,117],self.k,self.d,1)['valid'])
        moved=cv2.warpAffine(self.image,np.array([[1.,0,3],[0,1.,2]]),(320,240))
        result=tracker.update(moved,self.depth,self.k,self.d,33_000_001)
        self.assertTrue(result['valid'],result)
        np.testing.assert_allclose(result['pixel'],[158,119],atol=.3)

    def test_detection_correction_is_capped_at_two_pixels(self):
        tracker = LocalTracker()
        self.assertTrue(tracker.initialize(self.image, self.depth, [155, 117],
                                           self.k, self.d, 1)['valid'])
        corrected, reason = tracker.correct_from_detection(
            [165, 117], self.depth, self.k, self.d, 1)
        self.assertIsNone(reason)
        self.assertTrue(corrected['valid'])
        np.testing.assert_allclose(corrected['pixel'], [157, 117], atol=1e-6)
        self.assertAlmostEqual(corrected['point_camera_m'][0],
                               (157-160)*.2/400, places=6)

    def test_rejected_detection_correction_keeps_optical_flow_state(self):
        tracker = LocalTracker()
        self.assertTrue(tracker.initialize(self.image, self.depth, [155, 117],
                                           self.k, self.d, 1)['valid'])
        original_pixel = tracker.pixel.copy()
        original_position = tracker.position.copy()
        bad_depth = self.depth.copy()
        bad_depth[117, 155] = 0
        corrected, reason = tracker.correct_from_detection(
            [157, 117], bad_depth, self.k, self.d, 1)
        self.assertIsNone(corrected)
        self.assertIn('depth', reason)
        self.assertTrue(tracker.active)
        np.testing.assert_array_equal(tracker.pixel, original_pixel)
        np.testing.assert_array_equal(tracker.position, original_position)
        corrected, reason = tracker.correct_from_detection(
            [157, 117], self.depth, self.k, self.d, 2)
        self.assertIsNone(corrected)
        self.assertIn('current frame', reason)

    def test_edge_pixel_uses_clipped_texture_roi(self):
        tracker=LocalTracker()
        result=tracker.initialize(self.image,self.depth,[20,120],self.k,self.d,1)
        self.assertTrue(result['valid'],result)
        moved=cv2.warpAffine(self.image,np.array([[1.,0,2],[0,1.,1]]),(320,240))
        result=tracker.update(moved,self.depth,self.k,self.d,33_000_001)
        self.assertTrue(result['valid'],result)
        np.testing.assert_allclose(result['pixel'],[22,121],atol=.4)

    def test_depth_failure_latches_until_explicit_reinitialization(self):
        tracker=LocalTracker()
        tracker.initialize(self.image,self.depth,[160,120],self.k,self.d,1)
        self.assertFalse(tracker.update(self.image,np.zeros_like(self.depth),self.k,self.d,33_000_001)['valid'])
        self.assertFalse(tracker.update(self.image,self.depth,self.k,self.d,66_000_001)['valid'])
        self.assertTrue(tracker.initialize(self.image,self.depth,[160,120],self.k,self.d,99_000_001)['valid'])

    def test_weak_texture_and_repeat_frame_rejected(self):
        tracker=LocalTracker()
        self.assertFalse(tracker.initialize(np.zeros_like(self.image),self.depth,[160,120],self.k,self.d,1)['valid'])
        tracker.initialize(self.image,self.depth,[160,120],self.k,self.d,1)
        self.assertFalse(tracker.update(self.image,self.depth,self.k,self.d,1)['valid'])

    def test_relaxed_depth_step_keeps_invalid_depth_rejected(self):
        depth = self.depth.copy()
        depth[120, 161] = .206
        standard = LocalTracker().initialize(self.image, depth, [160, 120],
                                             self.k, self.d, 1)
        relaxed = LocalTracker(relaxed=True).initialize(self.image, depth, [160, 120],
                                                        self.k, self.d, 1)
        self.assertFalse(standard['valid'])
        self.assertTrue(relaxed['valid'], relaxed)
        depth[120, 160] = 0
        invalid = LocalTracker(relaxed=True).initialize(self.image, depth, [160, 120],
                                                        self.k, self.d, 1)
        self.assertFalse(invalid['valid'])

    def test_depth_edge_rejected(self):
        self.depth[:,160:]=.25
        with self.assertRaises(ValueError): surface(self.depth,[160,120],self.k,self.d)

    def test_occlusion_latches(self):
        tracker=LocalTracker()
        tracker.initialize(self.image,self.depth,[160,120],self.k,self.d,1)
        result=tracker.update(np.zeros_like(self.image),self.depth,self.k,self.d,33_000_001)
        self.assertFalse(result['valid'])
        self.assertFalse(tracker.active)

    def test_tilted_plane_and_outliers(self):
        yy,xx=np.mgrid[:240,:320]
        self.depth=.2/(1+.2*(xx-160)/400)
        self.depth[::9,::9]+=.008
        _,normal,q=surface(self.depth,[160,120],self.k,self.d)
        expected=-np.array([.2,0,1])/np.sqrt(1.04)
        np.testing.assert_allclose(normal,expected,atol=.01)
        self.assertLess(q['plane_rms_m'],.002)

if __name__=='__main__': unittest.main()
