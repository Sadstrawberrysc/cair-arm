"""Live preview's local transport and result gates; no camera or model download."""
import socket
import time
from types import SimpleNamespace
import unittest

import numpy as np

from infer.Carotid_begin.detection import response_to_boxes, scan_start_from_boxes
from infer.Carotid_begin.live_protocol import pack_frame, unpack_frame
from infer.Carotid_begin.live_preview import draw_preview, evaluate
from infer.Carotid_begin.live_window import key_reply


class FakeModel:
    def __init__(self, count=1):
        self.count = count
        self.received = None

    def infer(self, image):
        self.received = image
        box = SimpleNamespace(x=64, y=64, width=20, height=20,
                              confidence=.9, class_name="carotid")
        return [SimpleNamespace(image=SimpleNamespace(width=128, height=128),
                                predictions=[box] * self.count)]


class LivePreviewTests(unittest.TestCase):
    def setUp(self):
        self.image = np.random.default_rng(7).integers(0, 256, (128, 128, 3), dtype=np.uint8)
        self.depth = np.full((128, 128), .15, dtype=np.float32)
        self.meta = dict(capture_monotonic_ns=time.monotonic_ns(),
                         intrinsic=[[300., 0., 64.], [0., 300., 64.], [0., 0., 1.]],
                         distortion=[0.] * 8)

    def test_private_transport_roundtrip(self):
        a, b = socket.socketpair()
        try:
            b.sendall(pack_frame(self.image, self.depth, self.meta))
            image, depth, meta = unpack_frame(a)
        finally:
            a.close()
            b.close()
        np.testing.assert_array_equal(image, self.image)
        np.testing.assert_array_equal(depth, self.depth)
        self.assertEqual(meta, self.meta)

    def test_window_quit_keys(self):
        self.assertEqual(key_reply(ord("q"), False), b"Q")
        self.assertEqual(key_reply(27, False), b"Q")
        self.assertEqual(key_reply(0, True), b"Q")
        self.assertEqual(key_reply(0, False), b"R")

    def test_model_box_center_near_image_edge(self):
        response = FakeModel()
        response.infer = lambda image: [SimpleNamespace(
            image=SimpleNamespace(width=128, height=128),
            predictions=[SimpleNamespace(x=7, y=64, width=10, height=20,
                                         confidence=.9, class_name="carotid")])]
        raw = response_to_boxes(response.infer(self.image))
        point = scan_start_from_boxes(raw, 128, 128, class_name="carotid")
        self.assertEqual(point["scan_start"], [7., 64.])

    def test_response_dimensions_must_match_camera_frame(self):
        raw = response_to_boxes(FakeModel().infer(self.image))
        with self.assertRaisesRegex(ValueError, "dimensions differ"):
            scan_start_from_boxes(raw, 640, 480, class_name="carotid")

    def test_relaxed_inference_and_depth_gate(self):
        class ThresholdModel(FakeModel):
            def infer(self, image, *, confidence):
                self.threshold = confidence
                return super().infer(image)

        self.depth[64, 65] = .156
        standard = evaluate(self.image, self.depth, self.meta, FakeModel())
        self.assertEqual(standard["status"], "rejected")
        model = ThresholdModel()
        relaxed = evaluate(self.image, self.depth, self.meta, model,
                           min_confidence=.25, inference_confidence=.4,
                           relaxed_vision=True)
        self.assertEqual(model.threshold, .4)
        self.assertEqual(relaxed["vision_profile"], "relaxed")
        self.assertEqual(relaxed["status"], "candidate")
        self.depth[64, 64] = 0
        self.assertEqual(evaluate(self.image, self.depth, self.meta, model,
                                  min_confidence=.25, inference_confidence=.4,
                                  relaxed_vision=True)["status"], "rejected")

    def test_rejected_low_confidence_box_still_drawn(self):
        result = dict(status="rejected", reason="below threshold", inference_ms=5.,
                      age_ms=10., detections=[dict(x=64, y=64, width=20, height=20,
                                                    confidence=.2, **{"class": "carotid"})])
        frame = draw_preview(np.zeros((128, 128, 3), dtype=np.uint8), result)
        self.assertEqual(tuple(frame[64, 54]), (180, 180, 180))

    def test_candidate_ambiguous_and_stale(self):
        model = FakeModel()
        result = evaluate(self.image, self.depth, self.meta, model)
        self.assertIs(model.received, self.image)
        self.assertEqual(result["status"], "candidate")
        self.assertEqual(result["prediction"]["scan_start"], [64., 64.])
        self.assertTrue(result["geometry"]["tracking_initialization_possible"])
        self.assertEqual(draw_preview(self.image, result).shape, self.image.shape)
        ambiguous = evaluate(self.image, self.depth, self.meta, FakeModel(2))
        self.assertEqual(ambiguous["status"], "rejected")
        self.assertIn("exactly one", ambiguous["reason"])
        self.meta["capture_monotonic_ns"] -= 300_000_000
        stale = evaluate(self.image, self.depth, self.meta, FakeModel())
        self.assertEqual(stale["status"], "stale")
        self.assertFalse(stale["age_valid"])


if __name__ == "__main__":
    unittest.main()
