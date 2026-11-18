"""Read RGB/aligned depth owned by the C++ Collision camera process."""
import json
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np


class SharedFrames:
    def __init__(self, directory):
        self.directory = Path(directory)
        self.sequence = None
        self.metadata = None
        self.signature = None

    def next(self):
        deadline = time.monotonic() + .1
        while time.monotonic() < deadline:
            path = self.directory / 'frame.bin'
            try:
                stat = path.stat()
                signature = (stat.st_ino, stat.st_mtime_ns, stat.st_size)
                if signature == self.signature:
                    time.sleep(.01)
                    continue
                if stat.st_size > 20 * 1024 * 1024:
                    raise ValueError('shared camera frame too large')
                raw = path.read_bytes()
            except FileNotFoundError:
                time.sleep(.01)
                continue
            line, payload = raw.split(b'\n', 1)
            meta = json.loads(line)
            self.signature = signature
            if meta['schema_version'] != 1 or meta['session_id'] != self.directory.name:
                raise ValueError('shared camera session mismatch')
            w, h = meta['width'], meta['height']
            if not (0 < w <= 4096 and 0 < h <= 2160 and len(payload) == w * h * 5):
                raise ValueError('shared RGB/depth shape mismatch')
            if meta['color_format'] != 'rgb8' or meta['depth_format'] != 'z16_le':
                raise ValueError('unsupported shared frame format')
            if meta['sequence'] == self.sequence:
                time.sleep(.01)
                continue
            self.metadata = meta
            self.sequence = meta['sequence']
            image = np.frombuffer(payload, np.uint8, count=w*h*3).reshape(h, w, 3).copy()
            depth = np.frombuffer(payload, '<u2', offset=w*h*3).reshape(h, w).copy()
            intr = SimpleNamespace(**{k: meta[k] for k in ('width', 'height', 'fx', 'fy', 'ppx', 'ppy')})
            return image, depth, intr, float(meta['depth_scale'])
        return None  # Planning may pause frame publication; keep the UI alive.

    def publish(self, points=None, normal=None):
        meta = self.metadata
        data = {k: meta[k] for k in ('session_id', 'sequence', 'captured_at_ns',
                                     'capture_time_trusted', 'camera_serial', 'calibration_id')}
        data['schema_version'] = 1
        data['valid'] = points is not None and normal is not None
        if data['valid']:
            points = np.asarray(points, dtype=float)
            normal = np.asarray(normal, dtype=float)
            if points.ndim != 2 or points.shape[1] != 3 or len(points) < 2 or not np.isfinite(points).all():
                raise ValueError('invalid live carotid points')
            tangent = next((p-points[0] for p in points[1:] if np.linalg.norm(p-points[0]) > 1e-6), None)
            if tangent is None or normal.shape != (3,) or not np.isfinite(normal).all() or np.linalg.norm(normal) < 1e-9:
                raise ValueError('invalid live carotid normal/tangent')
            data.update(point_color_m=points[0].tolist(), outward_normal_color=(normal/np.linalg.norm(normal)).tolist(),
                        tangent_color=(tangent/np.linalg.norm(tangent)).tolist())
        temporary = self.directory / 'target.tmp'
        temporary.write_text(json.dumps(data, allow_nan=False), encoding='utf-8')
        temporary.replace(self.directory / 'target.json')
