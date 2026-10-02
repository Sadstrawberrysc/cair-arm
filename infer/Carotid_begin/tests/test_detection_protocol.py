"""Current model center selection and private RGB-D transport; no hardware."""
import socket
from types import SimpleNamespace
import unittest

import numpy as np

from infer.Carotid_begin.detection import response_to_boxes, scan_start_from_boxes
from infer.Carotid_begin.live_protocol import pack_frame, unpack_frame


class DetectionProtocolTests(unittest.TestCase):
    def setUp(self):
        self.image = np.random.default_rng(7).integers(0, 256, (128, 128, 3), dtype=np.uint8)
        self.depth = np.full((128, 128), .15, dtype=np.float32)
        self.meta = {'capture_monotonic_ns': 123456789}

    def raw(self, predictions):
        response = [SimpleNamespace(image=SimpleNamespace(width=128, height=128),
                                    predictions=predictions)]
        return response_to_boxes(response)

    def box(self, x=64, y=64, confidence=.9):
        return SimpleNamespace(x=x, y=y, width=10, height=20,
                               confidence=confidence, class_name='carotid')

    def test_rgbd_private_socket_roundtrip(self):
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

    def test_box_center_is_selected_even_near_image_edge(self):
        point = scan_start_from_boxes(self.raw([self.box(x=7)]), 128, 128,
                                      class_name='carotid', min_confidence=.5)
        self.assertEqual(point['scan_start'], [7., 64.])

    def test_size_ambiguity_and_confidence_are_checked(self):
        raw = self.raw([self.box()])
        with self.assertRaisesRegex(ValueError, 'dimensions differ'):
            scan_start_from_boxes(raw, 640, 480, class_name='carotid')
        with self.assertRaisesRegex(ValueError, 'exactly one'):
            scan_start_from_boxes(self.raw([self.box(), self.box(x=80)]),
                                  128, 128, class_name='carotid')
        with self.assertRaisesRegex(ValueError, 'exactly one'):
            scan_start_from_boxes(self.raw([self.box(confidence=.4)]),
                                  128, 128, class_name='carotid', min_confidence=.5)


if __name__ == '__main__':
    unittest.main()
