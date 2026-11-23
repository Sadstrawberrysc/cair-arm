import tempfile
import unittest
from types import SimpleNamespace

import numpy as np

from infer.Camera_wrist.detector.config import ModelConfig
from infer.Camera_wrist.detector.gpu_model import LocalRfDetr, load_cuda_model
from infer.Camera_wrist.detector.pipeline import DetectionPipeline


class FakeOnnxSession:
    def get_inputs(self):
        return [SimpleNamespace(name='input', type='tensor(float)', shape=[1, 3, 2, 2])]

    def get_outputs(self):
        return [SimpleNamespace(name=name, type='tensor(float)', shape=shape)
                for name, shape in [('dets', [1, 3, 4]), ('labels', [1, 3, 2])]]

    def run(self, outputs, inputs):
        self.tensor = inputs['input']
        boxes = [[[.5, .5, .5, .5], [.1, .8, .4, .4], [.25, .5, .3, .4]]]
        return np.array(boxes, dtype=np.float32), self.logits


class LocalDetectorTests(unittest.TestCase):
    def setUp(self):
        self.session = FakeOnnxSession()
        config = dict(network_input=dict(color_mode='rgb', resize_mode='stretch',
                                         input_channels=3, scaling_factor=255,
                                         dynamic_spatial_size_supported=False,
                                         training_input_size=dict(width=2, height=2),
                                         normalization=[[.1, .2, .3], [.5, .5, .5]]),
                      class_names_operations=[dict(type='class_name_removal',
                                                   class_name='background')])
        self.pipeline = DetectionPipeline(LocalRfDetr(self.session, config, ['background', 'carotid']),
                                          ModelConfig('test/model'))
        self.image = np.full((64, 100, 3), [51, 102, 204], dtype=np.uint8)

    def test_filters_background_then_selects_one_clipped_box_in_original_pixels(self):
        self.session.logits = np.array([[[10, -10], [-10, 2], [-10, 1]]], dtype=np.float32)
        result = self.pipeline.process(self.image, 42)
        self.assertTrue(result['valid'], result)
        self.assertEqual(result['capture_monotonic_ns'], 42)
        self.assertEqual(result['image_size'], [100, 64])
        self.assertEqual(len(result['detections']), 1)
        self.assertEqual(result['prediction']['box_xyxy'], [0, 38, 30, 64])
        self.assertEqual(result['prediction']['scan_start'], [15, 51])
        np.testing.assert_allclose(self.session.tensor[0, :, 0, 0], [1.4, .4, -.2], atol=1e-6)

    def test_low_confidence_produces_no_box(self):
        self.session.logits = np.full((1, 3, 2), -2, dtype=np.float32)
        result = self.pipeline.process(self.image, 42)
        self.assertFalse(result['valid'])
        self.assertEqual(result['detections'], [])

    def test_missing_or_outside_model_is_rejected_without_a_loader(self):
        with tempfile.TemporaryDirectory() as root:
            with self.assertRaisesRegex(ValueError, 'invalid local model bundle'):
                load_cuda_model('missing/model', root)
            with self.assertRaisesRegex(ValueError, 'under the local model root'):
                load_cuda_model('../outside', root)
