"""One-shot model selection, followed by frame-exact local tracker catch-up.

The detector runs in a separate Python environment. Nothing from this module
publishes a Robot command or relaxes the wrist tracker quality gates.
"""
from collections import deque
import json
import math
import os
from pathlib import Path
import queue
import socket
import struct
import subprocess
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from infer.Carotid_begin.live_protocol import pack_frame, recv_exact

DEFAULT_DETECTOR_PYTHON = Path('/home/cair-jacen/anaconda3/envs/ultralytics/bin/python')
DEFAULT_MKL_CORE = Path('/home/cair-jacen/anaconda3/pkgs/mkl-2023.1.0-h213fc3f_46344/lib/libmkl_core.so.2')


class ModelSelector:
    def __init__(self, python, model_id, class_name, confidence, margin, mkl_core=None,
                 inference_confidence=None):
        env = os.environ.copy()
        env['CONDA_PREFIX'] = str(Path(python).parent.parent)
        env['PATH'] = str(Path(python).parent) + os.pathsep + env.get('PATH', '')
        core = Path(mkl_core) if mkl_core else DEFAULT_MKL_CORE
        if core.is_file():
            env['LD_PRELOAD'] = str(core)
        runtime_lib = Path(python).parent.parent / 'lib'
        torch_libraries = sorted({path.resolve() for path in
                                  runtime_lib.glob('python*/site-packages/torch/lib')})
        if len(torch_libraries) != 1:
            raise RuntimeError('detector CUDA/cuDNN library directory not found')
        torch_lib = torch_libraries[0]
        env['LD_LIBRARY_PATH'] = os.pathsep.join(
            [str(torch_lib), str(runtime_lib), env.get('LD_LIBRARY_PATH', '')]).rstrip(os.pathsep)
        parent, child = socket.socketpair()
        self.socket = parent
        self.socket.settimeout(15)
        command = [str(python), '-m', 'infer.Carotid_begin.model_worker',
                   '--socket-fd', str(child.fileno()), '--model-id', model_id,
                   '--class-name', class_name, '--min-confidence', str(confidence),
                   '--margin', str(margin)]
        if inference_confidence is not None:
            command += ['--inference-confidence', str(inference_confidence)]
        try:
            self.process = subprocess.Popen(command, cwd=str(ROOT), env=env,
                                            pass_fds=(child.fileno(),))
        finally:
            child.close()
        try:
            self.socket.settimeout(60)
            length = struct.unpack('!I', recv_exact(self.socket, 4))[0]
            if not 0 < length <= 4096:
                raise ValueError('invalid model worker readiness response')
            ready = json.loads(recv_exact(self.socket, length))
            if ready != {'ready': True, 'model_id': model_id}:
                raise ValueError('model worker readiness mismatch')
        except Exception as error:
            self.socket.close()
            self.process.terminate()
            self.process.wait(timeout=5)
            raise RuntimeError('model worker failed to load: %s' % error) from error
        self.socket.settimeout(15)
        self.pending = False
        self.frames = deque(maxlen=45)
        self.overflow = False
        self.results = queue.Queue()

    def request(self, frame, meta, *, collect_frames=True):
        if self.pending:
            raise ValueError('model detection already running')
        if self.process.poll() is not None:
            raise ValueError('model worker exited: %s' % self.process.returncode)
        image, depth, timestamp = frame
        if not 0 <= time.monotonic_ns() - timestamp <= 200_000_000:
            raise ValueError('detection request frame is stale')
        payload = pack_frame(image, depth.astype('float32'), meta)
        self.frames.clear()
        self.overflow = False
        self.collect_frames = collect_frames
        self.pending = True
        def exchange():
            try:
                self.socket.sendall(payload)
                length = struct.unpack('!I', recv_exact(self.socket, 4))[0]
                if not 0 < length <= 65536:
                    raise ValueError('invalid model response size')
                response = json.loads(recv_exact(self.socket, length))
                self.results.put((frame, response, None))
            except Exception as error:
                self.results.put((frame, None, str(error)))
        threading.Thread(target=exchange, daemon=True).start()

    def add_frame(self, frame):
        if self.pending and self.collect_frames:
            if len(self.frames) == self.frames.maxlen:
                self.overflow = True
            self.frames.append(frame)

    def poll(self):
        try:
            initial, response, error = self.results.get_nowait()
        except queue.Empty:
            return None
        frames = list(self.frames)
        overflow = self.overflow
        self.pending = False
        self.frames.clear()
        return initial, frames, response, error or ('detection frame buffer overflow' if overflow else None)

    def close(self):
        self.socket.close()
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()


def replay_selection(tracker, initial, frames, prediction, k, distortion):
    """Validate model pixel on its source frame; replay every subsequent RGB-D frame."""
    if not isinstance(prediction, dict) or 'scan_start' not in prediction:
        raise ValueError('model returned no valid P0')
    if not frames:
        raise ValueError('no newer RGB-D frame after inference')
    image, depth, timestamp = initial
    result = tracker.initialize(image, depth, prediction['scan_start'], k, distortion, timestamp)
    if not result.get('valid'):
        raise ValueError(result['reason'])
    for image, depth, timestamp in frames:
        result = tracker.update(image, depth, k, distortion, timestamp)
        if not result.get('valid'):
            raise ValueError(result['reason'])
    return result


def observation_is_fresh(result, now_ns):
    if not result.get('valid'):
        return False
    capture = result.get('capture_monotonic_ns')
    return isinstance(capture, int) and 0 <= now_ns - capture <= 200_000_000


class DetectionCenterFusion:
    """Align an asynchronous detection with optical flow from the same source frame."""

    def __init__(self, *, max_age_ns=500_000_000):
        self.max_age_ns = max_age_ns
        self.history = deque(maxlen=60)
        self.target_id = None

    def reset(self):
        self.history.clear()
        self.target_id = None

    def remember(self, result, target_id):
        if not target_id or not result.get('valid'):
            return
        if target_id != self.target_id:
            self.reset()
            self.target_id = target_id
        stamp = result['capture_monotonic_ns']
        point = tuple(result['pixel'])
        if self.history and self.history[-1][0] == stamp:
            self.history[-1] = (stamp, point)
        elif not self.history or stamp > self.history[-1][0]:
            self.history.append((stamp, point))

    def aligned_center(self, response, target_id, current_result, now_ns):
        if not target_id or target_id != self.target_id or not current_result.get('valid'):
            return None, 'no active target for fusion'
        if not isinstance(response, dict) or response.get('valid') is not True:
            return None, 'detection is not a unique valid target'
        stamp = response.get('capture_monotonic_ns')
        if not isinstance(stamp, int) or not 0 <= now_ns-stamp <= self.max_age_ns:
            return None, 'detection is stale or from the future'
        current_stamp = current_result.get('capture_monotonic_ns')
        if not isinstance(current_stamp, int) or current_stamp < stamp:
            return None, 'tracking result precedes detection'
        prediction = response.get('prediction')
        if not isinstance(prediction, dict):
            return None, 'detection has no selected center'
        try:
            center = tuple(float(value) for value in prediction['scan_start'])
        except (KeyError, TypeError, ValueError):
            return None, 'invalid detection center'
        if len(center) != 2 or not all(math.isfinite(value) for value in center):
            return None, 'invalid detection center'
        source = next((point for time_ns, point in reversed(self.history)
                       if time_ns == stamp), None)
        if source is None:
            return None, 'matching tracked source frame unavailable'
        current = current_result.get('pixel')
        if (not isinstance(current, (list, tuple)) or len(current) != 2 or
                not all(isinstance(value, (int, float)) and math.isfinite(value)
                        for value in current)):
            return None, 'invalid current tracking point'
        return [current[0]+center[0]-source[0], current[1]+center[1]-source[1]], None


def fuse_model_observation(tracker, fusion, response, source_timestamp, frame,
                           current_result, target_id, k, distortion, now_ns):
    """Return (observation, applied, reason, confirmed) for one model preview."""
    if response.get('capture_monotonic_ns') != source_timestamp:
        return current_result, False, 'model response frame identity mismatch', False
    aligned, reason = fusion.aligned_center(response, target_id, current_result, now_ns)
    if aligned is None:
        return current_result, False, reason, False
    if not observation_is_fresh(current_result, now_ns):
        return current_result, False, 'current tracking observation is stale', False
    corrected, reason = tracker.correct_from_detection(
        aligned, frame[1], k, distortion, frame[2])
    if corrected is not None:
        return corrected, True, None, True
    return current_result, False, reason, reason == 'already aligned'
