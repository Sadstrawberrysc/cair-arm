"""Exact depth cross used by the target continuity check; no filtering or filling."""
import math
import numpy as np

MIN_CONTINUOUS_NEIGHBORS = 2


class DepthNeighborhoodError(ValueError):
    def __init__(self, reason, neighborhood):
        super().__init__(reason)
        self.depth_neighborhood = neighborhood


def failure_details(error):
    value = getattr(error, 'depth_neighborhood', None)
    return dict(depth_neighborhood=value) if value is not None else {}


def depth_cross(depth, pixel, limit_m):
    point = np.asarray(pixel, dtype=float)
    if point.shape != (2,) or not np.isfinite(point).all():
        raise ValueError('invalid tracking pixel')
    x, y = np.floor(point).astype(int)
    height, width = depth.shape

    def sample(px, py):
        value = dict(pixel=[int(px), int(py)], depth_m=None, valid=False)
        if not (0 <= px < width and 0 <= py < height):
            value['invalid_kind'] = 'out_of_bounds'
            return value
        raw = float(depth[py, px])
        if not math.isfinite(raw):
            value.update(invalid_kind='nonfinite', raw_value=str(raw))
        else:
            value['depth_m'] = raw
            value['valid'] = raw > 0
            if raw <= 0:
                value['invalid_kind'] = 'zero' if raw == 0 else 'negative'
        return value

    target = sample(x, y)
    neighbors = {name: sample(x+dx, y+dy) for name, dx, dy in (
        ('left', -1, 0), ('right', 1, 0), ('up', 0, -1), ('down', 0, 1))}
    causes, issues = [], []
    if not target['valid']:
        causes.append('target_invalid_depth')
    for name, value in neighbors.items():
        value['delta_from_target_m'] = None
        value['absolute_delta_m'] = None
        value['continuity_passed'] = False
        if target['depth_m'] is not None and value['depth_m'] is not None:
            delta = value['depth_m']-target['depth_m']
            value.update(delta_from_target_m=delta, absolute_delta_m=abs(delta))
        if not value['valid']:
            issues.append('neighbor_invalid_depth:'+name)
        elif target['valid']:
            value['continuity_passed'] = value['absolute_delta_m'] < limit_m
            if not value['continuity_passed']:
                issues.append('neighbor_depth_jump:'+name)
    border = bool(0 <= x < width and 0 <= y < height
                  and (x in (0, width-1) or y in (0, height-1)))
    valid_count = sum(value['valid'] for value in neighbors.values())
    continuous_count = sum(value['continuity_passed'] for value in neighbors.values())
    if target['valid'] and continuous_count < MIN_CONTINUOUS_NEIGHBORS:
        causes.append('insufficient_continuous_neighbors')
        causes.extend(issues)
    return dict(target_pixel_subpixel=point.tolist(), image_size=[width, height],
                target=target, neighbors=neighbors, depth_jump_limit_m=limit_m,
                comparison='abs(neighbor-target) < limit; both depths finite and positive',
                valid_neighbor_count=valid_count, continuous_neighbor_count=continuous_count,
                minimum_continuous_neighbors=MIN_CONTINUOUS_NEIGHBORS,
                continuity_passed=bool(target['valid'] and continuous_count >= MIN_CONTINUOUS_NEIGHBORS),
                neighbor_issues=issues,
                target_on_image_border=border, failure_causes=causes)
