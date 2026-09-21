"""Experimental local RGB-D tracker. No robot or Redis I/O."""
import cv2
import numpy as np
from wrist_projection import depth_at


def surface(depth, pixel, k, distortion):
    """Metric 15 mm neighbourhood; robust plane, oriented toward camera."""
    z = depth_at(depth, pixel)
    ray = cv2.undistortPoints(np.asarray(pixel, np.float64).reshape(1, 1, 2), k, distortion).reshape(2)
    target = np.r_[ray, 1.] * z
    # Unproject actual pixel centres using the color distortion model.
    radius = min(100, int(np.ceil(max(k[0, 0], k[1, 1]) * .015 / z * 1.5)) + 2)
    x, y = np.floor(pixel).astype(int)
    y0, y1 = max(0, y-radius), min(depth.shape[0], y+radius+1)
    x0, x1 = max(0, x-radius), min(depth.shape[1], x+radius+1)
    yy, xx = np.mgrid[y0:y1, x0:x1]
    values = depth[y0:y1, x0:x1]
    valid = np.isfinite(values) & (values > 0) & (np.abs(values-z) < .015)
    # Discard pixels bordering invalid depth or a >5 mm discontinuity.
    safe = valid.astype(np.uint8)
    for dy, dx in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
        neighbour = np.roll(values, (dy, dx), (0, 1))
        safe &= (np.isfinite(neighbour) & (neighbour > 0) & (np.abs(values-neighbour) < .005)).astype(np.uint8)
    safe[[0, -1], :] = 0
    safe[:, [0, -1]] = 0
    if not safe[y-y0, x-x0]:
        raise ValueError('target on invalid depth or depth edge')
    pixels = np.column_stack((xx[safe > 0], yy[safe > 0])).astype(np.float64)
    rays = cv2.undistortPoints(pixels.reshape(-1, 1, 2), k, distortion).reshape(-1, 2)
    cloud = np.column_stack((rays, np.ones(len(rays)))) * values[safe > 0, None]
    cloud = cloud[np.linalg.norm(cloud-target, axis=1) <= .015]
    if len(cloud) < 50:
        raise ValueError('fewer than 50 surface points')
    if len(cloud) > 2000:
        cloud = cloud[np.linspace(0, len(cloud)-1, 2000).astype(int)]
    rng = np.random.default_rng(42)
    best = np.zeros(len(cloud), dtype=bool)
    for _ in range(80):
        a, b, c = cloud[rng.choice(len(cloud), 3, replace=False)]
        normal = np.cross(b-a, c-a)
        length = np.linalg.norm(normal)
        if length < 1e-10:
            continue
        inliers = np.abs((cloud-a) @ (normal/length)) <= .003
        if inliers.sum() > best.sum():
            best = inliers
    if best.sum() < 50 or best.mean() < .7:
        raise ValueError('surface plane lacks consensus')
    points = cloud[best]
    centre = points.mean(axis=0)
    _, singular, vh = np.linalg.svd(points-centre, full_matrices=False)
    if singular[1] / np.sqrt(len(points)) < .002:
        raise ValueError('surface support nearly collinear')
    normal = vh[-1]
    rms = np.sqrt(np.mean(((points-centre) @ normal)**2))
    if rms > .002 or abs((target-centre) @ normal) > .003:
        raise ValueError('surface plane/target residual too large')
    if normal @ target > 0:
        normal = -normal
    return target, normal, dict(surface_points=int(len(points)), plane_rms_m=float(rms),
                                plane_inlier_ratio=float(best.mean()))


class LocalTracker:
    """Loss is latched; only explicit initialize can restore tracking."""
    def __init__(self):
        self.debug = {}
        self.debug_images = None
        self.lose('not initialized', force=True)

    def lose(self, reason, force=False):
        # Preserve the first loss cause until explicit operator action.
        if not force and not self.active:
            return
        self.active = False
        self.reason = str(reason)
        self.gray = self.features = self.pixel = self.position = None
        self.timestamp = None

    def _features(self, gray, pixel):
        x, y = np.floor(pixel).astype(int)
        if x-40 < 0 or y-40 < 0 or x+40 > gray.shape[1] or y+40 > gray.shape[0]:
            raise ValueError('80x80 tracking ROI outside image')
        mask = np.zeros_like(gray)
        mask[y-40:y+40, x-40:x+40] = 255
        points = cv2.goodFeaturesToTrack(gray, 150, .01, 4, mask=mask, blockSize=5)
        if points is None or len(points) < 12:
            raise ValueError('fewer than 12 texture features')
        return points

    def initialize(self, image, depth, pixel, k, distortion, timestamp):
        self.lose('initialization rejected', force=True)
        self.debug = {}
        self.debug_images = None
        try:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            features = self._features(gray, pixel)
            point, normal, quality = surface(depth, pixel, k, distortion)
            quality['feature_inliers'] = int(len(features))
            self.gray, self.features = gray, features
            self.pixel, self.position = np.asarray(pixel, float), point
            self.timestamp, self.active, self.reason = timestamp, True, ''
            return self.result(point, normal, quality)
        except (ValueError, cv2.error) as error:
            self.lose(error, force=True)
            return dict(valid=False, reason=self.reason)

    def result(self, point, normal, quality):
        return dict(valid=True, reason='experimental_local_tracking', pixel=self.pixel.tolist(),
                    point_camera_m=point.tolist(), normal_out_camera=normal.tolist(),
                    capture_monotonic_ns=self.timestamp, quality=quality)

    def update(self, image, depth, k, distortion, timestamp):
        if not self.active:
            return dict(valid=False, reason=self.reason)
        self.debug = dict(capture_monotonic_ns=int(timestamp),
                          previous_capture_monotonic_ns=int(self.timestamp),
                          target_before_px=self.pixel.tolist(), feature_count=int(len(self.features)), stage='capture_time')
        x, y = np.floor(self.pixel).astype(int)
        bounds = (max(0, x-60), max(0, y-60), min(image.shape[1], x+60), min(image.shape[0], y+60))
        x0, y0, x1, y1 = bounds
        self.debug['crop_xyxy'] = list(bounds)
        self.debug_images = (self.gray[y0:y1, x0:x1].copy(), image[y0:y1, x0:x1].copy())
        try:
            if not 0 < timestamp-self.timestamp <= 200_000_000:
                raise ValueError('tracking capture gap or repeated frame')
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            self.debug['stage'] = 'forward_lk'
            nxt, status, _ = cv2.calcOpticalFlowPyrLK(self.gray, gray, self.features, None,
                                                   winSize=(21, 21), maxLevel=3)
            if nxt is None or status is None:
                raise ValueError('forward optical flow failed')
            self.debug['stage'] = 'backward_lk'
            back, reverse, _ = cv2.calcOpticalFlowPyrLK(gray, self.gray, nxt, None,
                                                     winSize=(21, 21), maxLevel=3)
            if back is None or reverse is None:
                raise ValueError('backward optical flow failed')
            valid = (status.ravel() > 0) & (reverse.ravel() > 0)
            fb = np.linalg.norm(back-self.features, axis=2).ravel()
            self.debug.update(forward_matches=int(status.sum()), backward_matches=int(reverse.sum()),
                              fb_median_px=float(np.median(fb[np.isfinite(fb)])) if np.isfinite(fb).any() else None)
            valid &= fb <= 1.
            old, new = self.features[valid].reshape(-1, 2), nxt[valid].reshape(-1, 2)
            self.debug.update(fb_matches=int(len(old)), old_points=old.tolist(), new_points=new.tolist())
            self.debug['stage'] = 'forward_backward_check'
            if len(old) < 12:
                raise ValueError('fewer than 12 forward-backward features')
            self.debug['stage'] = 'affine_consensus'
            affine, mask = cv2.estimateAffine2D(old, new, method=cv2.RANSAC,
                                                      ransacReprojThreshold=1., maxIters=1000)
            self.debug.update(affine=None if affine is None else affine.tolist(),
                              inlier_mask=None if mask is None else mask.ravel().tolist(),
                              affine_inliers=0 if mask is None else int(mask.sum()),
                              affine_inlier_ratio=0. if mask is None else float(mask.mean()))
            if affine is None or mask is None or mask.sum() < 12 or mask.mean() < .6:
                raise ValueError('local affine consensus failed')
            determinant = np.linalg.det(affine[:, :2])
            scales = np.linalg.svd(affine[:, :2], compute_uv=False)
            self.debug['affine_scales'] = scales.tolist()
            self.debug['stage'] = 'affine_scale'
            if not .81 <= determinant <= 1.21 or scales.min() < .9 or scales.max() > 1.1:
                raise ValueError('abnormal affine scale')
            pixel = affine @ np.r_[self.pixel, 1.]
            self.debug['stage'] = 'pixel_jump'
            if np.linalg.norm(pixel-self.pixel) > 15:
                raise ValueError('pixel jump exceeds 15 px/frame')
            self.debug['stage'] = 'feature_replenishment'
            features = self._features(gray, pixel)
            self.debug['stage'] = 'surface_plane'
            point, normal, quality = surface(depth, pixel, k, distortion)
            self.debug['stage'] = 'position_jump'
            if np.linalg.norm(point-self.position) > .01:
                raise ValueError('3D jump exceeds 10 mm/frame')
            quality.update(feature_inliers=int(mask.sum()), forward_backward_limit_px=1.)
            self.gray, self.features, self.pixel = gray, features, pixel
            self.position, self.timestamp = point, timestamp
            self.debug['stage'] = 'accepted'
            return self.result(point, normal, quality)
        except (ValueError, cv2.error) as error:
            self.lose(error)
            return dict(valid=False, reason=self.reason)
