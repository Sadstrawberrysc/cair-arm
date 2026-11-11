"""No SDK: simulate a producer outrunning the image-processing loop."""
from pathlib import Path
import queue
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from live_frames import LatestFrameReader


class FakePipeline:
    def __init__(self):
        self.frames = queue.Queue()

    def wait_for_frames(self, timeout_ms):
        try:
            value = self.frames.get(timeout=timeout_ms/1000)
        except queue.Empty:
            return None
        if isinstance(value, Exception):
            raise value
        return value


class LatestFrameTests(unittest.TestCase):
    def source(self):
        pipeline = FakePipeline()
        source = LatestFrameReader(pipeline)
        source.start()
        self.addCleanup(source.stop)
        return pipeline, source

    def test_slow_consumer_gets_latest_frame_and_skips_backlog(self):
        pipeline, source = self.source()
        for index in range(20):
            pipeline.frames.put(index)
        with source.condition:
            self.assertTrue(source.condition.wait_for(
                lambda: source.slot is not None and source.slot[0] == 19, timeout=1))
        frame, received_ns, dropped = source.take()
        self.assertEqual(frame, 19)
        self.assertGreater(received_ns, 0)
        self.assertEqual(dropped, 19)
        # No reuse of the previous frame or its receipt timestamp.
        self.assertEqual(source.take(timeout=.01), (None, None, 19))
        pipeline.frames.put(20)
        self.assertEqual(source.take(timeout=1)[0], 20)

    def test_capture_error_reaches_consumer_instead_of_freezing_on_old_frame(self):
        pipeline, source = self.source()
        pipeline.frames.put(RuntimeError('camera disconnected'))
        with self.assertRaisesRegex(RuntimeError, 'camera disconnected'):
            source.take(timeout=1)

    def test_worker_stops_without_device(self):
        _, source = self.source()
        self.assertTrue(source.stop())
        self.assertEqual(source.take(timeout=.01)[:2], (None, None))


if __name__ == '__main__':
    unittest.main()
