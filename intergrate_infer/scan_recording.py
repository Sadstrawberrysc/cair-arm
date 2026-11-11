"""Paired ultrasound / post-tare Tool wrench videos, without control commands."""
import csv
from datetime import datetime
import json
from pathlib import Path
import threading
import time
import uuid

import cv2
import numpy as np

CHANNEL = 'robot:sensor:v1'
AXES = ('Fx', 'Fy', 'Fz', 'Tx', 'Ty', 'Tz')
MAX_AGE_NS = 500_000_000


def parse_wrench(message, now_ns):
    """Never label missing tare, invalid or stale telemetry as measured zero."""
    stamp = message.get('timestamp_monotonic_ns', 0)
    if message.get('version') != 1:
        raise ValueError('unsupported sensor version')
    if not isinstance(stamp, int) or not 0 < stamp <= now_ns:
        raise ValueError('invalid sensor timestamp')
    reason = ''
    if message.get('runtime_tare_applied') is not True:
        reason = 'WAITING FOR TARE'
    elif message.get('valid') is not True:
        reason = 'INVALID FORCE DATA'
    elif now_ns - stamp > MAX_AGE_NS:
        reason = 'STALE FORCE DATA'
    units = message.get('units', {})
    if units.get('force') != 'N' or units.get('torque') != 'N*m':
        raise ValueError('unexpected wrench units')
    wrench = message['compensated_wrench_tool']
    force = np.asarray(wrench['force_n'], dtype=float)
    torque = np.asarray(wrench['torque_nm'], dtype=float)
    if force.shape != (3,) or torque.shape != (3,):
        raise ValueError('wrench must have six components')
    values = np.concatenate((force, torque))
    if not np.isfinite(values).all():
        raise ValueError('non-finite wrench')
    return stamp, values if not reason else np.full(6, np.nan), reason


class CurveHistory:
    """Full-run min/max envelope with bounded display memory; CSV keeps samples.

    Time bins widen by powers of two as the run grows. Extrema and unavailable
    intervals survive compaction; plotting never discards the start of the run.
    """
    def __init__(self, origin_ns, max_bins=512):
        self.origin_ns = origin_ns
        self.max_bins = max_bins
        self.bin_ns = 20_000_000
        self.bins = {}

    @staticmethod
    def merge(a, b):
        return dict(start=a['start'], end=b['end'],
                    low=np.minimum(a['low'], b['low']),
                    high=np.maximum(a['high'], b['high']),
                    first=a['first'], last=b['last'],
                    gap=a['gap'] or b['gap'] or b['start'] - a['end'] > MAX_AGE_NS)

    def append(self, sample):
        stamp, values = sample
        if stamp < self.origin_ns:
            return
        while (stamp - self.origin_ns) // self.bin_ns >= self.max_bins:
            merged = {}
            for index, bucket in sorted(self.bins.items()):
                key = index // 2
                merged[key] = self.merge(merged[key], bucket) if key in merged else bucket
            self.bins = merged
            self.bin_ns *= 2
        index = (stamp - self.origin_ns) // self.bin_ns
        valid = np.isfinite(values)
        bucket = dict(start=stamp, end=stamp,
                      low=np.where(valid, values, np.inf),
                      high=np.where(valid, values, -np.inf),
                      first=values.copy(), last=values.copy(), gap=not valid.all())
        self.bins[index] = self.merge(self.bins[index], bucket) if index in self.bins else bucket


def curve_image(history, now_ns, origin_ns, status):
    image = np.full((800, 1200, 3), 24, np.uint8)
    def text(label, xy, color=(220, 220, 220), scale=.55):
        cv2.putText(image, label, xy, cv2.FONT_HERSHEY_SIMPLEX,
                    scale, color, 1, cv2.LINE_AA)
    elapsed = (now_ns - origin_ns) / 1e9
    text('POST-TARE TOOL WRENCH | FULL RUN: 0 - %.3f s' % elapsed, (20, 28))
    text(status, (20, 53), (60, 210, 60) if status == 'LIVE' else (0, 170, 255))
    if not isinstance(history, CurveHistory):
        samples = history
        history = CurveHistory(origin_ns)
        for sample in samples:
            history.append(sample)
    points = [bucket for _, bucket in sorted(history.bins.items()) if bucket['end'] <= now_ns]
    span_ns = max(1, now_ns - origin_ns)
    for axis, name in enumerate(AXES):
        column, row = axis // 3, axis % 3
        x0, y0 = 85 + column * 600, 110 + row * 225
        width, height = 490, 160
        finite = [b[key][axis] for b in points for key in ('low', 'high')
                  if np.isfinite(b[key][axis])]
        bound = max(1.0 if axis < 3 else .05,
                    max((abs(v) for v in finite), default=0) * 1.15)
        unit = 'N' if axis < 3 else 'N*m'
        text('%s (%s)' % (name, unit), (x0, y0 - 12))
        for fraction in (0, .5, 1):
            y = y0 + round(fraction * height)
            cv2.line(image, (x0, y), (x0 + width, y), (65, 65, 65), 1)
            text('%.3g' % (bound * (1 - 2 * fraction)), (x0 - 70, y + 5), scale=.43)
        previous = None
        def xy(stamp, value):
            return (x0 + round((stamp - origin_ns) / span_ns * width),
                    y0 + round((1 - value / bound) * height / 2))
        for bucket in points:
            if not np.isfinite(bucket['low'][axis]):
                previous = None
                continue
            stamp = (bucket['start'] + bucket['end']) // 2
            cv2.line(image, xy(stamp, bucket['low'][axis]), xy(stamp, bucket['high'][axis]),
                     (90, 220, 90), 1, cv2.LINE_AA)
            if bucket['low'][axis] == bucket['high'][axis]:
                cv2.circle(image, xy(stamp, bucket['low'][axis]), 1, (90, 220, 90), -1)
            if (previous is not None and not bucket['gap']
                    and np.isfinite(bucket['first'][axis])
                    and bucket['start'] - previous[0] <= MAX_AGE_NS):
                cv2.line(image, previous[1], xy(bucket['start'], bucket['first'][axis]),
                         (90, 220, 90), 1, cv2.LINE_AA)
            previous = ((bucket['end'], xy(bucket['end'], bucket['last'][axis]))
                        if not bucket['gap'] and np.isfinite(bucket['last'][axis]) else None)
        text('0.0 s', (x0, y0 + height + 20), scale=.43)
        text('%.1f s' % elapsed, (x0 + width - 55, y0 + height + 20), scale=.43)
    text('Tool axes; calibrated gravity/CoM compensation + runtime tare. Gaps = unavailable data.', (20, 788), scale=.48)
    return image


class ScanRecording:
    """One worker owns Redis subscription, CSV files and both MJPEG writers.

    Frames are resampled onto the same fixed-rate clock. Slow inference repeats
    the latest received ROI, with its age visible; no historical image is called
    a newly captured frame. This records the inference input, not a second camera.
    """
    def __init__(self, redis_client, output_dir, frame_size, fps=30):
        if fps <= 0:
            raise ValueError('recording fps must be positive')
        name = datetime.now().strftime('scan_%Y%m%d_%H%M%S_') + uuid.uuid4().hex[:8]
        self.directory = Path(output_dir) / name
        self.directory.mkdir(parents=True)
        self.client = redis_client
        self.width, self.height = frame_size
        self.video_size = (self.width + self.width % 2, self.height + self.height % 2)
        self.fps = fps
        self.origin_ns = time.monotonic_ns()
        self.started_unix_ns = time.time_ns()
        self.lock = threading.Lock()
        self.latest = None
        self.stop = threading.Event()
        self.error = None
        self.frame_count = 0
        self.history = CurveHistory(self.origin_ns)
        self.last_sensor_ns = 0
        self.status = 'WAITING FOR POST-TARE DATA'
        self.rejected_messages = 0
        self.thread = threading.Thread(target=self._run, name='scan-recording', daemon=True)
        self.thread.start()

    def submit(self, frame, received_ns=None):
        if self.error:
            raise RuntimeError('scan recording failed: ' + str(self.error)) from self.error
        if frame.shape != (self.height, self.width, 3) or frame.dtype != np.uint8:
            raise ValueError('recording frame shape/dtype mismatch')
        with self.lock:
            self.latest = (frame.copy(), received_ns or time.monotonic_ns())

    def _run(self):
        writers = []
        subscription = None
        try:
            subscription = self.client.pubsub()
            subscription.subscribe(CHANNEL)
            for name, size in [('ultrasound.avi', self.video_size),
                               ('wrench_tare.avi', (1200, 800))]:
                writer = cv2.VideoWriter(str(self.directory / name),
                                         cv2.VideoWriter_fourcc(*'MJPG'), self.fps, size)
                writers.append(writer)
                if not writer.isOpened():
                    raise RuntimeError('cannot open video writer: ' + name)
            with (self.directory / 'wrench_tare.csv').open('w', newline='') as force_file, \
                 (self.directory / 'frames.csv').open('w', newline='') as frame_file:
                force_csv = csv.writer(force_file)
                force_csv.writerow(['sensor_monotonic_ns', 'received_monotonic_ns',
                                    'elapsed_s', 'valid', 'reason', *AXES])
                frame_csv = csv.writer(frame_file)
                frame_csv.writerow(['frame_index', 'video_time_s', 'render_monotonic_ns',
                                    'ultrasound_received_monotonic_ns', 'force_sensor_monotonic_ns',
                                    'force_status'])
                next_frame_ns = self.origin_ns
                while not self.stop.is_set():
                    # Bound receive work so an accumulated stream cannot starve recording.
                    for _ in range(256):
                        try:
                            item = subscription.get_message(timeout=0)
                        except Exception as error:
                            self.status = 'REDIS DISCONNECTED'
                            break
                        if item is None:
                            break
                        if item['type'] != 'message':
                            continue
                        now = time.monotonic_ns()
                        try:
                            stamp, values, reason = parse_wrench(json.loads(item['data']), now)
                            if stamp <= self.last_sensor_ns:
                                continue
                            self.last_sensor_ns = stamp
                            self.status = reason or 'LIVE'
                            self.history.append((stamp, values))
                            force_csv.writerow([stamp, now, (stamp - self.origin_ns) / 1e9,
                                                int(not reason), reason, *values])
                        except (ValueError, KeyError, TypeError, OverflowError):
                            self.rejected_messages += 1
                    now = time.monotonic_ns()
                    if now >= next_frame_ns:
                        status = self.status
                        if self.last_sensor_ns and now - self.last_sensor_ns > MAX_AGE_NS:
                            status = 'STALE FORCE DATA'
                        with self.lock:
                            latest = self.latest
                        ultrasound = np.zeros((self.video_size[1], self.video_size[0], 3), np.uint8)
                        received = 0
                        if latest is not None:
                            frame, received = latest
                            ultrasound[:self.height, :self.width] = frame
                        age = 'NO IMAGE' if not received else 'ROI age %.0f ms' % ((now-received)/1e6)
                        cv2.rectangle(ultrasound, (0, 0), (self.width, 28), (0, 0, 0), -1)
                        cv2.putText(ultrasound, 't=%.3fs | %s' % ((now-self.origin_ns)/1e9, age),
                                    (8, 19), cv2.FONT_HERSHEY_SIMPLEX, .45, (0, 255, 255), 1)
                        writers[0].write(ultrasound)
                        writers[1].write(curve_image(self.history, now, self.origin_ns, status))
                        frame_csv.writerow([self.frame_count, self.frame_count / self.fps, now,
                                            received, self.last_sensor_ns, status])
                        self.frame_count += 1
                        next_frame_ns = self.origin_ns + round(self.frame_count * 1e9 / self.fps)
                        if self.frame_count % self.fps == 0:
                            force_file.flush()
                            frame_file.flush()
                    self.stop.wait(max(0, min(.005, (next_frame_ns-time.monotonic_ns())/1e9)))
        except Exception as error:
            self.error = error
            print('[Recording] ERROR: %s' % error, flush=True)
        finally:
            for writer in writers:
                writer.release()
            if subscription is not None:
                subscription.close()
            metadata = dict(schema_version=1, fps=self.fps, frames=self.frame_count,
                            origin_monotonic_ns=self.origin_ns, started_unix_ns=self.started_unix_ns,
                            channel=CHANNEL, wrench_frame='calibrated Tool',
                            tare_required=True, force_unit='N', torque_unit='N*m',
                            curve_time_range='recording start to current time',
                            curve_summary='adaptive time-bin min/max envelope; full samples in CSV',
                            ultrasound_size=list(self.video_size),
                            ultrasound_source='inference ROI; repeated frames retain source receipt timestamp',
                            rejected_messages=self.rejected_messages,
                            error=str(self.error) if self.error else None)
            (self.directory / 'recording.json').write_text(json.dumps(metadata, indent=2))

    def close(self):
        self.stop.set()
        self.thread.join(timeout=10)
        if self.thread.is_alive():
            raise RuntimeError('scan recording worker did not finish within 10 seconds')
        if self.error:
            raise RuntimeError('scan recording failed: ' + str(self.error)) from self.error
        print('[Recording] Saved %d paired frames: %s' % (self.frame_count, self.directory), flush=True)
