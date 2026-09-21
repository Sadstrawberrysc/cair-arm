"""Bounded failure evidence, stored only on active -> stopped transitions."""
import json
from pathlib import Path
import time
import cv2
import numpy as np


def json_safe(value):
    if isinstance(value, dict):
        return {k: json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(v) for v in value]
    if isinstance(value, np.generic):
        return json_safe(value.item())
    if isinstance(value, float) and not np.isfinite(value):
        return None
    return value


class TrackingDiagnostics:
    def __init__(self, log_path, record):
        self.directory = Path(str(log_path) + '.tracking')
        self.record = record
        self.active = False
        self.event_id = 0

    def emit(self, result, tracker, kind='tracking'):
        value = dict(result)
        now_active = bool(value.get('valid'))
        loss = self.active and not now_active
        event = 'sample' if now_active else 'latched_or_waiting'
        if kind == 'tracking_initialization':
            event = 'initialized' if now_active else 'initialization_rejected'
        elif loss:
            event = 'paused' if value.get('reason', '').startswith('operator paused') else 'lost'
        self.active = now_active
        value.update(type=kind, event=event, receive_monotonic_ns=time.monotonic_ns())
        if now_active and kind != 'tracking_initialization':
            value['flow_diagnostics'] = {k: tracker.debug[k] for k in (
                'feature_count', 'forward_matches', 'backward_matches', 'fb_matches',
                'fb_median_px', 'affine_inliers', 'affine_inlier_ratio', 'affine_scales') if k in tracker.debug}
        if loss:
            self.event_id += 1
            if event == 'lost':
                value['failure_diagnostics'] = tracker.debug
                # At most 32 events/run, 3 small PNGs/event. No full video.
                if self.event_id <= 32 and tracker.debug_images is not None:
                    try:
                        folder = self.directory / ('event_%03d' % self.event_id)
                        folder.mkdir(parents=True, exist_ok=False)
                        before, failed = tracker.debug_images
                        if before.ndim == 2:
                            before = cv2.cvtColor(before, cv2.COLOR_GRAY2BGR)
                        for name, image in [('before.png', before), ('current.png', failed)]:
                            if not cv2.imwrite(str(folder / name), image):
                                raise OSError('cannot write ' + name)
                        matches = np.concatenate((before, failed), axis=1)
                        x0, y0, _, _ = tracker.debug['crop_xyxy']
                        mask = tracker.debug.get('inlier_mask')
                        for i, (a, b) in enumerate(zip(tracker.debug.get('old_points', []), tracker.debug.get('new_points', []))):
                            if not np.isfinite(a+b).all():
                                continue
                            start = tuple(np.rint(np.asarray(a)-[x0, y0]).astype(int))
                            end = tuple(np.rint(np.asarray(b)-[x0, y0]+[before.shape[1], 0]).astype(int))
                            color = (0, 255, 0) if mask is not None and mask[i] else (0, 0, 255)
                            cv2.line(matches, start, end, color, 1)
                        if not cv2.imwrite(str(folder / 'matches.png'), matches):
                            raise OSError('cannot write matches')
                        (folder / 'details.json').write_text(json.dumps(json_safe(dict(
                            reason=value['reason'], diagnostics=tracker.debug,
                            note='before/current are the most recent attempted LK pair; external frame loss may occur later')),
                            indent=2, allow_nan=False))
                        value['evidence_directory'] = str(folder)
                    except (OSError, ValueError, cv2.error) as error:
                        value['evidence_error'] = str(error)
                elif self.event_id > 32:
                    value['evidence_skipped'] = '32 event limit'
        value['event_id'] = self.event_id
        self.record(json_safe(value))
