"""Exercise the production pipeline and socket loop without model downloads."""
import contextlib
import io
import json
import socket
import struct
from concurrent.futures import ThreadPoolExecutor
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import numpy as np

from infer.Carotid_begin.config import ModelConfig
from infer.Carotid_begin.live_protocol import (
    MAX_PAYLOAD, pack_frame, recv_exact, send_json, unpack_frame,
)
from infer.Carotid_begin.model_worker import parse_args, serve
from infer.Carotid_begin.pipeline import DetectionPipeline


def native_response(count=1, x=16):
    return [SimpleNamespace(
        image=SimpleNamespace(width=32, height=32),
        predictions=[SimpleNamespace(x=x, y=16, width=8, height=10,
                                     confidence=.9, class_name='carotid')
                     for _ in range(count)])]


def read_json(sock):
    length = struct.unpack('!I', recv_exact(sock, 4))[0]
    return json.loads(recv_exact(sock, length))


class ModelWorkerTests(unittest.TestCase):
    def setUp(self):
        self.image = np.zeros((32, 32, 3), dtype=np.uint8)
        self.depth = np.full((32, 32), .15, dtype=np.float32)
        self.config = ModelConfig('test/model')
        self.model = Mock()
        self.pipeline = DetectionPipeline(self.model, self.config)

    def test_cli_defaults_and_invalid_settings(self):
        fd, config = parse_args(['--socket-fd', '4', '--model-id', 'test/model'])
        self.assertEqual((fd, config), (4, self.config))
        for option, value in [('--min-confidence', 'nan'), ('--margin', '-1'),
                              ('--inference-confidence', '1.1'), ('--socket-fd', '-1')]:
            with self.subTest(option=option), contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit):
                    parse_args(['--socket-fd', '4', '--model-id', 'test/model', option, value])

    def test_pipeline_retains_preview_when_selection_is_ambiguous(self):
        self.model.infer.return_value = native_response(2)
        result = self.pipeline.process(self.image, 123)
        self.assertFalse(result['valid'])
        self.assertIn('exactly one', result['reason'])
        self.assertEqual(len(result['detections']), 2)
        self.assertEqual(result['capture_monotonic_ns'], 123)
        self.assertEqual(result['image_size'], [32, 32])
        self.model.infer.assert_called_once_with(self.image, confidence=.4)

    def test_nonfinite_or_malformed_model_output_is_serializable_rejection(self):
        for output in [native_response(x=float('nan')), object(), [None]]:
            with self.subTest(output=output):
                self.model.infer.return_value = output
                result = self.pipeline.process(self.image, 123)
                self.assertFalse(result['valid'])
                self.assertNotIn('detections', result)
                json.dumps(result, allow_nan=False)

    def test_fatal_model_failure_propagates(self):
        self.model.infer.side_effect = RuntimeError('GPU failed')
        with self.assertRaisesRegex(RuntimeError, 'GPU failed'):
            self.pipeline.process(self.image, 123)

    def test_socket_session_handles_multiple_frames_and_clean_eof(self):
        self.model.infer.side_effect = [native_response(0), native_response()]
        client, worker = socket.socketpair()
        client.settimeout(3)
        worker.settimeout(3)
        with client, worker, ThreadPoolExecutor(max_workers=1) as pool:
            task = pool.submit(serve, worker, self.pipeline)
            try:
                self.assertEqual(read_json(client), {'ready': True, 'model_id': 'test/model'})
                for stamp, valid in [(123, False), (456, True)]:
                    client.sendall(pack_frame(self.image, self.depth,
                                              {'capture_monotonic_ns': stamp}))
                    result = read_json(client)
                    self.assertEqual(result['capture_monotonic_ns'], stamp)
                    self.assertEqual(result['valid'], valid)
                    if valid:
                        self.assertEqual(result['prediction']['scan_start'], [16., 16.])
            finally:
                client.shutdown(socket.SHUT_WR)
            task.result(timeout=3)

    def test_metadata_is_checked_on_send_and_receive(self):
        for meta in [{}, [], {'capture_monotonic_ns': True}, {'capture_monotonic_ns': -1}]:
            with self.subTest(meta=meta):
                with self.assertRaises(ValueError):
                    pack_frame(self.image, self.depth, meta)
                # Bypass sender validation to exercise the receiver boundary.
                buffer = io.BytesIO()
                np.savez(buffer, image=self.image, depth=self.depth, meta=json.dumps(meta))
                payload = buffer.getvalue()
                left, right = socket.socketpair()
                with left, right:
                    left.sendall(struct.pack('!I', len(payload)) + payload)
                    with self.assertRaises(ValueError):
                        unpack_frame(right)

    def test_invalid_lengths_and_truncated_frames(self):
        for payload in [struct.pack('!I', 0), struct.pack('!I', MAX_PAYLOAD + 1),
                        b'\x00\x00', struct.pack('!I', 10), struct.pack('!I', 10) + b'x']:
            with self.subTest(payload=payload):
                left, right = socket.socketpair()
                with left, right:
                    left.sendall(payload)
                    left.shutdown(socket.SHUT_WR)
                    with self.assertRaises(ValueError):
                        unpack_frame(right)

    def test_response_size_limit_is_enforced_before_writing(self):
        sock = Mock()
        with self.assertRaisesRegex(ValueError, 'too large'):
            send_json(sock, {'reason': 'x' * 65536})
        sock.sendall.assert_not_called()


if __name__ == '__main__':
    unittest.main()
