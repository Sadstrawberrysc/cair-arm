import sys
from pathlib import Path
import unittest
import cv2
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from local_tracker import LocalTracker, surface

class TrackerTests(unittest.TestCase):
    def setUp(self):
        self.k=np.array([[400.,0,160],[0,400.,120],[0,0,1.]])
        self.d=np.zeros(5)
        rng=np.random.default_rng(7)
        gray=cv2.GaussianBlur(rng.integers(0,256,(240,320),dtype=np.uint8),(3,3),0)
        self.image=cv2.cvtColor(gray,cv2.COLOR_GRAY2BGR)
        self.depth=np.full((240,320),.2)

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
