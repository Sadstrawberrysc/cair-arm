"""Image-space Tool-Y alignment, independent of models, Redis and hardware."""
import math
import numpy as np


def fresh_visual_command(command, observed_ns, now_ns):
    """A live Redis heartbeat must not keep an old image driving Y/X/RZ."""
    if command.get('terminate') or 0 <= now_ns - observed_ns <= 300_000_000:
        return command
    held = dict(command, phase_idx=-1, phase_confidence=0.0,
                recovery_mode=False, mask_lr_majority=0)
    held['parameters'] = dict(command['parameters'], y=0.0, rz=0.0)
    return held


def gate_alignment_command(command, alignment):
    """Zero/pending centering is not evidence of alignment; preserve recovery."""
    y, rz, phase, confidence, recovery, side = command
    if not recovery and not alignment['valid']:
        return 0.0, 0.0, -1, 0.0, False, 0
    if not recovery and abs(alignment['error_m']) > .0008:
        # Velocity limiting/confirmation can make the command smaller than the
        # measured error. Do not use that smaller value to authorize X or RZ.
        return y, 0.0, -1, 0.0, False, 0
    return command


class VisualYConditioner:
    """将逐帧视觉横向误差变成稳定的 Tool-Y 命令输入。"""

    def __init__(self, deadband_m, lowpass_alpha, confirmation_frames,
                 reengage_m=None, maximum_error_m=None):
        if deadband_m < 0.0:
            raise ValueError("visual Y deadband must be non-negative")
        if not 0.0 < lowpass_alpha <= 1.0:
            raise ValueError("visual Y low-pass alpha must be in (0, 1]")
        if confirmation_frames < 1:
            raise ValueError("visual Y confirmation frames must be positive")
        self.deadband_m = float(deadband_m)
        self.lowpass_alpha = float(lowpass_alpha)
        self.confirmation_frames = int(confirmation_frames)
        self.reengage_m = max(self.deadband_m, reengage_m or self.deadband_m)
        self.maximum_error_m = maximum_error_m
        self.reset()

    def reset(self):
        self._filtered_y_m = 0.0
        self._confirmed_sign = 0
        self._candidate_sign = 0
        self._candidate_count = 0

    def update(self, raw_y_m):
        """死区清零；换向时先连续确认，再对同向误差做 EMA 低通。"""
        raw_y_m = float(raw_y_m)
        threshold = self.reengage_m if self._confirmed_sign == 0 else self.deadband_m
        if not np.isfinite(raw_y_m) or abs(raw_y_m) <= threshold:
            self.reset()
            return 0.0

        sign = 1 if raw_y_m > 0.0 else -1
        if sign != self._confirmed_sign:
            if sign == self._candidate_sign:
                self._candidate_count += 1
            else:
                self._candidate_sign = sign
                self._candidate_count = 1

            # 尚未确认方向时发布零，防止单帧噪声让机械臂反向。
            if self._candidate_count < self.confirmation_frames:
                self._filtered_y_m = 0.0
                return 0.0

            self._confirmed_sign = sign
            self._candidate_sign = sign
            self._candidate_count = self.confirmation_frames
            self._filtered_y_m = raw_y_m
        else:
            self._candidate_sign = sign
            self._candidate_count = self.confirmation_frames
            self._filtered_y_m = (
                self.lowpass_alpha * raw_y_m
                + (1.0 - self.lowpass_alpha) * self._filtered_y_m
            )

        value = self._filtered_y_m if abs(self._filtered_y_m) > self.deadband_m else 0.0
        return float(np.clip(value, -self.maximum_error_m, self.maximum_error_m)
                     if self.maximum_error_m is not None else value)


class VisualYAlignment:
    """Follow a visible vessel center, rather than an unobserved pose regression.

    Image-right requires +Tool-Y, consistent with the existing recovery mapping.
    Robot applies model_y_direction=-1, so the published error is center - cx.
    Pixel scale uses the configured ultrasound height and square image pixels.
    """
    def __init__(self, image_height_mm=37.0):
        if not math.isfinite(image_height_mm) or image_height_mm <= 0:
            raise ValueError('ultrasound image height must be positive')
        self.image_height_mm = image_height_mm
        self.conditioner = VisualYConditioner(.0005, .35, 2,
                                              reengage_m=.0008, maximum_error_m=.004)
        self.reset()

    def reset(self):
        self.conditioner.reset()
        self.last_center = None
        self.last_seen_ns = 0
        self.last_source = ''

    def update(self, shape, regions, detection, frame_ns, now_ns):
        height, width = shape[:2]
        result = dict(valid=False, source='none', reason='', cx=None, cy=None,
                      error_m=0.0, command_m=0.0, frame_monotonic_ns=frame_ns)
        def reject(reason):
            self.conditioner.reset()
            result['reason'] = reason
            return result
        if not 0 <= now_ns - frame_ns <= 300_000_000:
            return reject('ultrasound observation older than 300 ms')
        if now_ns - self.last_seen_ns > 500_000_000:
            self.last_center = None
        candidates = [(float(r['cx']), float(r['cy'])) for r in regions
                      if math.isfinite(r['cx']) and math.isfinite(r['cy'])
                      and 0 <= r['cx'] < width and 0 <= r['cy'] < height]
        center, source = None, ''
        if detection is not None:
            dx, dy = float(detection['cx']), float(detection['cy'])
            if math.isfinite(dx) and math.isfinite(dy) and 0 <= dx < width and 0 <= dy < height:
                matched = [p for p in candidates
                           if detection['x1'] <= p[0] <= detection['x2']
                           and detection['y1'] <= p[1] <= detection['y2']]
                center = min(matched, key=lambda p: math.hypot(p[0]-dx, p[1]-dy)) if matched else (dx, dy)
                source = 'segmentation+yolo' if matched else 'yolo'
        if center is None and len(candidates) == 1:
            center, source = candidates[0], 'segmentation'
        elif center is None and candidates and self.last_center is not None:
            center = min(candidates, key=lambda p: math.dist(p, self.last_center))
            source = 'segmentation'
        if center is None:
            return reject('ambiguous vessels' if candidates else 'no visible vessel')
        if self.last_center is not None and math.dist(center, self.last_center) > 60:
            return reject('vessel center jumped more than 60 pixels')
        self.last_source = source
        self.last_center, self.last_seen_ns = center, now_ns
        # width includes only the cropped ROI; video encoding padding is excluded.
        error = ((width - 1) / 2 - center[0]) * self.image_height_mm / height / 1000
        result.update(valid=True, source=source, cx=center[0], cy=center[1], error_m=error,
                      command_m=self.conditioner.update(error))
        return result
