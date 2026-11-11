"""SDK-only spatial smoothing and hole filling on current aligned depth frames."""
from dataclasses import dataclass
import math
import time
import numpy as np

from depth_diagnostics import depth_cross


@dataclass
class ProcessedDepth:
    raw_m: np.ndarray
    filtered_m: np.ndarray
    stats: dict

    def annotate(self, result, capture_ns, depth_jump_limit_m):
        """Pair raw/processed samples only with the exact observation frame."""
        neighborhood = result.get('depth_neighborhood', {})
        stamp = result.get('capture_monotonic_ns', neighborhood.get('capture_monotonic_ns'))
        pixel = result.get('pixel', neighborhood.get('target_pixel_subpixel'))
        if stamp != capture_ns or pixel is None:
            return result
        raw = depth_cross(self.raw_m, pixel, depth_jump_limit_m)
        filtered = depth_cross(self.filtered_m, pixel, depth_jump_limit_m)
        original, processed = raw['target'], filtered['target']
        source = ('sdk_filled' if processed['valid'] and not original['valid']
                  else 'sdk_spatial' if processed['valid'] else 'invalid')
        detail = dict(capture_monotonic_ns=capture_ns, depth_source=source,
                      raw_target_depth_m=original['depth_m'],
                      filtered_target_depth_m=processed['depth_m'],
                      target_was_hole=not original['valid'])
        # Full crosses are useful for holes and failed observations; keep ordinary
        # valid per-frame logging compact.
        if not original['valid'] or not result.get('valid'):
            detail.update(raw_neighborhood=raw, filtered_neighborhood=filtered)
        return dict(result, depth_processing=detail)


class SdkDepthProcessor:
    """Construct after opening the Orbbec device (SDK filter library activation).

    Use the public factory: this SDK's typed constructors can segfault before
    activation, whereas the factory raises an OBError with a useful message.
    """
    def __init__(self, sdk):
        self.sdk = sdk
        self.spatial = sdk.FilterFactory.create_filter('SpatialAdvancedFilter')
        self.holes = sdk.FilterFactory.create_filter('HoleFillingFilter')
        for name, value in (('magnitude', 1), ('alpha', .5), ('radius', 1)):
            self.spatial.set_config_value(name, value)
        # Keep the device SDK's depth-gradient default for edge-preserving smoothing.
        self.holes.set_config_value('hole_filling_mode', sdk.OBHoleFillingMode.FURTHEST.value)
        self.spatial.enable(True)
        self.holes.enable(True)
        self.configuration = dict(
            type='depth_filter_configuration',
            order=['SpatialAdvancedFilter', 'HoleFillingFilter'],
            spatial={name: self.spatial.get_config_value(name)
                     for name in ('magnitude', 'alpha', 'radius', 'disp_diff')},
            hole_filling_mode='FURTHEST', hole_area_limit=None,
            temporal_filter=False, preserves_capture_timestamp=True)

    def _checked_depth(self, frame, original):
        if frame is None:
            raise ValueError('SDK depth filter returned no frame')
        depth = frame.as_depth_frame()
        if depth is None:
            raise ValueError('SDK depth filter returned a non-depth frame')
        width, height, fmt, capture_us, scale = original
        if ((depth.get_width(), depth.get_height(), depth.get_format()) != (width, height, fmt)):
            raise ValueError('SDK depth filter changed dimensions or format')
        if depth.get_global_timestamp_us() != capture_us:
            raise ValueError('SDK depth filter changed capture timestamp')
        if depth.get_depth_scale() != scale:
            raise ValueError('SDK depth filter changed depth scale')
        return depth

    def process(self, depth):
        started = time.monotonic_ns()
        if depth.get_format() != self.sdk.OBFormat.Y16:
            raise ValueError('SDK depth processing requires Y16')
        scale_m = float(depth.get_depth_scale())/1000
        if not math.isfinite(scale_m) or scale_m <= 0:
            raise ValueError('invalid SDK depth scale')
        shape = (depth.get_height(), depth.get_width())
        metadata = (depth.get_width(), depth.get_height(), depth.get_format(),
                    depth.get_global_timestamp_us(), depth.get_depth_scale())
        # Copy before invoking SDK filters, even if a filter mutates its input.
        raw = np.frombuffer(depth.get_data(), np.uint16).reshape(shape).copy()
        spatial = self._checked_depth(self.spatial.process(depth), metadata)
        spatial_finished = time.monotonic_ns()
        filled = self._checked_depth(self.holes.process(spatial), metadata)
        final = np.frombuffer(filled.get_data(), np.uint16).reshape(shape).copy()
        finished = time.monotonic_ns()
        return ProcessedDepth(raw*scale_m, final*scale_m, dict(
            raw_invalid_pixels=int(np.count_nonzero(raw == 0)),
            filtered_invalid_pixels=int(np.count_nonzero(final == 0)),
            filled_pixels=int(np.count_nonzero((raw == 0) & (final > 0))),
            spatial_filter_ms=(spatial_finished-started)/1e6,
            hole_filling_ms=(finished-spatial_finished)/1e6,
            depth_filter_ms=(finished-started)/1e6))
