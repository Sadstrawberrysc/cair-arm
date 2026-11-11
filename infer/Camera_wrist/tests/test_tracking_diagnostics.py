import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
import cv2
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from local_tracker import LocalTracker
from tracking_diagnostics import TrackingDiagnostics

class DiagnosticTests(unittest.TestCase):
    def test_residual_measurements_are_logged_without_triggering_loss(self):
        rng = np.random.default_rng(7)
        gray = cv2.GaussianBlur(rng.integers(0, 256, (240, 320), dtype=np.uint8), (3, 3), 0)
        image = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        depth = np.full((240, 320), .2)
        k = np.array([[400., 0, 160], [0, 400., 120], [0, 0, 1.]])
        tracker = LocalTracker()
        initial = tracker.initialize(image, depth, [160, 120], k, np.zeros(5), 1)
        self.assertTrue(initial['valid'])
        depth[117:124, 157:164] = .2036
        with tempfile.TemporaryDirectory() as directory:
            rows = []
            writer = TrackingDiagnostics(Path(directory)/'run.jsonl', rows.append)
            writer.emit(initial, tracker)
            result = tracker.update(image, depth, k, np.zeros(5), 33_000_001)
            writer.emit(result, tracker)
            self.assertEqual(rows[-1]['event'], 'sample')
            metrics = rows[-1]['quality']
            self.assertEqual(metrics['reference_exceeded'], ['target_plane_distance'])
            self.assertEqual(metrics['failed_checks'], [])
            self.assertFalse(metrics['residual_gates_enabled'])
            self.assertIsNone(metrics['plane_rms_limit_m'])
            self.assertIsNone(metrics['target_plane_distance_limit_m'])
            self.assertGreater(metrics['target_plane_distance_m'], .003)
            self.assertFalse(writer.directory.exists())
            json.dumps(rows, allow_nan=False)

    def test_first_failure_cause_latched(self):
        tracker = LocalTracker()
        tracker.active = True
        tracker.lose('local affine consensus failed')
        tracker.lose('no matching bracketing robot poses within 50 ms')
        self.assertEqual(tracker.reason, 'local affine consensus failed')
        tracker.lose('operator paused', force=True)
        self.assertEqual(tracker.reason, 'operator paused')

    def test_evidence_once_per_loss_and_reinitialization(self):
        with tempfile.TemporaryDirectory() as directory:
            rows=[]
            writer=TrackingDiagnostics(Path(directory)/'run.jsonl',rows.append)
            tracker=SimpleNamespace(debug=dict(crop_xyxy=[np.int64(0),0,10,10],
                old_points=[[2.,3.]],new_points=[[3.,3.]],inlier_mask=[1],
                affine_inliers=1,fb_median_px=float('nan')),
                debug_images=(np.zeros((10,10),np.uint8),np.zeros((10,10,3),np.uint8)))
            writer.emit(dict(valid=False,reason='not initialized'),tracker)
            self.assertFalse(writer.directory.exists())
            writer.emit(dict(valid=True,reason='ok'),tracker,kind='tracking_initialization')
            writer.emit(dict(valid=False,reason='local affine consensus failed'),tracker)
            for _ in range(10):writer.emit(dict(valid=False,reason='local affine consensus failed'),tracker)
            self.assertEqual(sum(r['event']=='lost' for r in rows),1)
            folder=writer.directory/'event_001'
            self.assertEqual(len(list(folder.glob('*.png'))),3)
            data=json.loads((folder/'details.json').read_text())
            self.assertIsNone(data['diagnostics']['fb_median_px'])
            writer.emit(dict(valid=True,reason='ok'),tracker,kind='tracking_initialization')
            writer.emit(dict(valid=False,reason='local affine consensus failed'),tracker)
            self.assertTrue((writer.directory/'event_002/details.json').exists())
            json.dumps(rows,allow_nan=False)

    def test_pause_does_not_save_failure_images(self):
        with tempfile.TemporaryDirectory() as directory:
            rows=[];writer=TrackingDiagnostics(Path(directory)/'run.jsonl',rows.append)
            tracker=SimpleNamespace(debug={},debug_images=None)
            writer.emit(dict(valid=True,reason='ok'),tracker,kind='tracking_initialization')
            writer.emit(dict(valid=False,reason='operator paused'),tracker)
            self.assertEqual(rows[-1]['event'],'paused')
            self.assertFalse(writer.directory.exists())

if __name__=='__main__':unittest.main()
