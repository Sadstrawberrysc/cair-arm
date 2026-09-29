"""Gemini capture only. Does not import a robot, Redis or target projection module."""
import importlib.util
from pathlib import Path
import time

import cv2
import numpy as np



class CaptureClock:
    """The wrist acquisition clock policy, isolated from any target/handoff code."""
    def __init__(self):
        self.offset = None
        self.last = 0
        self.count = 0

    def convert(self, global_us):
        before = time.monotonic_ns()
        wall = time.time_ns()
        after = time.monotonic_ns()
        offset = (before + after)//2 - wall
        if after-before > 1_000_000 or (self.offset is not None and abs(offset-self.offset) > 2_000_000):
            self.count = 0
            self.offset = offset
            raise ValueError("capture clock offset changed")
        self.offset = offset
        capture = int(global_us)*1000 + offset
        if global_us <= 0 or capture <= self.last or not 0 <= after-capture <= 200_000_000:
            self.count = 0
            raise ValueError("invalid/stale/repeated capture clock")
        self.last = capture
        self.count += 1
        if self.count < 30:
            raise ValueError("capture clock warmup %d/30" % self.count)
        return capture


def wrist_config():
    # This module contains only acquisition settings. No target data or model is loaded.
    path = Path(__file__).resolve().parents[1] / "Camera_wrist/gemini_config.py"
    spec = importlib.util.spec_from_file_location("carotid_begin_camera_config", str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def decode(frames, align, ob):
    if frames is None:
        raise ValueError("no frames")
    aligned = align.process(frames)
    frames = None if aligned is None else aligned.as_frame_set()
    if frames is None:
        raise ValueError("no aligned frames")
    color, depth = frames.get_color_frame(), frames.get_depth_frame()
    if color is None or depth is None:
        raise ValueError("incomplete aligned RGB-D")
    if color.get_format() != ob.OBFormat.MJPG:
        raise ValueError("expected MJPG color")
    image = cv2.imdecode(np.frombuffer(color.get_data(), np.uint8), cv2.IMREAD_COLOR)
    if image is None or image.shape[:2] != (color.get_height(), color.get_width()):
        raise ValueError("color decode/size mismatch")
    scale = float(depth.get_depth_scale()) / 1000.
    if not np.isfinite(scale) or scale <= 0:
        raise ValueError("invalid depth scale")
    z = np.frombuffer(depth.get_data(), np.uint16).reshape(
        depth.get_height(), depth.get_width()).astype(np.float32)*scale
    if z.shape != image.shape[:2]:
        raise ValueError("aligned depth dimensions differ from RGB")
    return image, z, int(color.get_global_timestamp_us()), int(depth.get_global_timestamp_us()), scale
