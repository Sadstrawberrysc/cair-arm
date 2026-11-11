"""Offline tests with real AVI encoding/decoding and an in-memory telemetry queue."""
import csv
import json
from pathlib import Path
import queue
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from scan_recording import CurveHistory, ScanRecording, curve_image, parse_wrench


def message(**updates):
    result = dict(version=1, timestamp_monotonic_ns=time.monotonic_ns(),
                  runtime_tare_applied=True, valid=True,
                  units=dict(force='N', torque='N*m'),
                  compensated_wrench_tool=dict(force_n=[1, 2, 3], torque_nm=[.01, .02, .03]))
    result.update(updates)
    return result


class Subscription:
    def __init__(self):
        self.items = queue.Queue()
        self.closed = False

    def pubsub(self):
        return self

    def subscribe(self, channel):
        assert channel == 'robot:sensor:v1'

    def get_message(self, timeout=0):
        try:
            return self.items.get_nowait()
        except queue.Empty:
            return None

    def close(self):
        self.closed = True


class RecordingTests(unittest.TestCase):
    def test_full_run_keeps_early_extrema_and_gaps_with_bounded_bins(self):
        origin = 1_000_000_000
        history = CurveHistory(origin)
        history.append((origin + 10_000_000, np.full(6, 7.0)))
        history.append((origin + 20_000_000, np.full(6, np.nan)))
        history.append((origin + 30_000_000, np.full(6, -9.0)))
        for second in range(1, 3601):
            history.append((origin + second * 1_000_000_000, np.zeros(6)))
        self.assertLessEqual(len(history.bins), 512)
        earliest = history.bins[0]
        self.assertEqual(earliest['start'], origin + 10_000_000)
        np.testing.assert_allclose(earliest['high'], 7)
        np.testing.assert_allclose(earliest['low'], -9)
        self.assertTrue(earliest['gap'])

    def test_video_chart_still_draws_start_after_twenty_seconds(self):
        origin = 1_000_000_000
        history = CurveHistory(origin)
        history.append((origin + 1_000_000_000, np.ones(6)))
        history.append((origin + 20_000_000_000, np.zeros(6)))
        image = curve_image(history, origin + 20_000_000_000, origin, 'LIVE')
        # First second remains visible near the left edge of the first chart.
        region = image[110:271, 85:150].astype(int)
        self.assertTrue(np.any((region[:, :, 1] > region[:, :, 0] + 30)
                               & (region[:, :, 1] > region[:, :, 2] + 30)))

    def test_only_valid_fresh_post_tare_data_are_plotted(self):
        item = message()
        stamp, values, reason = parse_wrench(item, time.monotonic_ns())
        np.testing.assert_allclose(values, [1, 2, 3, .01, .02, .03])
        self.assertEqual(reason, '')
        for updates in (dict(runtime_tare_applied=False), dict(valid=False),
                        dict(timestamp_monotonic_ns=stamp - 1_000_000_000)):
            _, values, reason = parse_wrench(message(**updates), time.monotonic_ns())
            self.assertTrue(np.isnan(values).all())
            self.assertTrue(reason)
        item.pop('runtime_tare_applied')
        self.assertEqual(parse_wrench(item, time.monotonic_ns())[2], 'WAITING FOR TARE')

    def test_bad_payload_rejected(self):
        for updates in (dict(timestamp_monotonic_ns=0), dict(units={}),
                        dict(compensated_wrench_tool=dict(force_n=[0, 0], torque_nm=[0, 0, 0])),
                        dict(compensated_wrench_tool=dict(force_n=[float('nan'), 0, 0], torque_nm=[0, 0, 0]))):
            with self.assertRaises(ValueError):
                parse_wrench(message(**updates), time.monotonic_ns())

    def test_curve_without_samples_does_not_invent_measurements(self):
        now = time.monotonic_ns()
        image = curve_image([], now, now, 'WAITING FOR POST-TARE DATA')
        self.assertEqual(image.shape, (800, 1200, 3))
        # Curve green is distinct from grid/text colors; there must be no trace.
        self.assertFalse(np.any(np.all(image == [90, 220, 90], axis=2)))

    def test_paired_files_and_stale_gap(self):
        with tempfile.TemporaryDirectory() as directory:
            subscription = Subscription()
            recorder = ScanRecording(subscription, directory, (557, 640), fps=15)
            try:
                recorder.submit(np.full((640, 557, 3), 100, np.uint8))
                item = message()
                subscription.items.put(dict(type='message', data=json.dumps(item)))
                subscription.items.put(dict(type='message', data='bad json'))
                deadline = time.monotonic() + 3
                while recorder.frame_count < 12 and not recorder.error and time.monotonic() < deadline:
                    time.sleep(.01)
            finally:
                recorder.close()
            self.assertTrue(subscription.closed)
            self.assertGreaterEqual(recorder.frame_count, 12)
            metadata = json.loads((recorder.directory / 'recording.json').read_text())
            self.assertIsNone(metadata['error'])
            self.assertEqual(metadata['rejected_messages'], 1)
            with (recorder.directory / 'frames.csv').open() as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), metadata['frames'])
            self.assertIn('LIVE', [row['force_status'] for row in rows])
            self.assertEqual(rows[-1]['force_status'], 'STALE FORCE DATA')
            with (recorder.directory / 'wrench_tare.csv').open() as stream:
                force = list(csv.DictReader(stream))
            self.assertEqual(len(force), 1)
            self.assertEqual(float(force[0]['Fz']), 3)
            self.assertEqual(float(force[0]['Tz']), .03)
            for name, size in [('ultrasound.avi', (558, 640)), ('wrench_tare.avi', (1200, 800))]:
                video = cv2.VideoCapture(str(recorder.directory / name))
                try:
                    self.assertEqual(int(video.get(cv2.CAP_PROP_FRAME_COUNT)), len(rows))
                    count = 0
                    while True:
                        ok, frame = video.read()
                        if not ok:
                            break
                        self.assertEqual(frame.shape[:2], size[::-1])
                        count += 1
                    self.assertEqual(count, len(rows))
                finally:
                    video.release()

    def test_encoder_failure_is_reported(self):
        with tempfile.TemporaryDirectory() as directory:
            class BrokenWriter:
                def isOpened(self): return False
                def release(self): pass
            with patch('scan_recording.cv2.VideoWriter', return_value=BrokenWriter()):
                recorder = ScanRecording(Subscription(), directory, (557, 640))
                with self.assertRaisesRegex(RuntimeError, 'cannot open video writer'):
                    recorder.thread.join(timeout=3)
                    recorder.close()


if __name__ == '__main__':
    unittest.main()
