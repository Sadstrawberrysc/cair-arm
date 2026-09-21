import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from local_tracker import LocalTracker
from tracking_diagnostics import TrackingDiagnostics

class DiagnosticTests(unittest.TestCase):
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
