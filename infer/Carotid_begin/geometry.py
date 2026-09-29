"""Offline camera-frame geometry; no hardware, robot, Redis or target priors."""
import cv2
import numpy as np

STANDARD = dict(depth_jump=.005, min_points=50, plane_band=.003, inlier_ratio=.7,
                spread=.002, rms=.002, target_residual=.003, features=12)
RELAXED = dict(depth_jump=.008, min_points=35, plane_band=.004, inlier_ratio=.55,
               spread=.0015, rms=.0035, target_residual=.0045, features=8)


def surface(depth, pixel, k, distortion, radius_m=0.015, *, relaxed=False):
    """Robust local plane in a 15 mm neighborhood with selected quality bounds."""
    limits = RELAXED if relaxed else STANDARD
    pixel = np.asarray(pixel, dtype=float)
    if pixel.shape != (2,) or not np.isfinite(pixel).all():
        raise ValueError("invalid pixel")
    x, y = np.floor(pixel).astype(int)
    if not (0 <= x < depth.shape[1] and 0 <= y < depth.shape[0]):
        raise ValueError("pixel outside image")
    z = float(depth[y, x])
    if not np.isfinite(z) or z <= 0:
        raise ValueError("invalid target depth")
    ray = cv2.undistortPoints(pixel.reshape(1, 1, 2), k, distortion).reshape(2)
    target = np.r_[ray, 1.] * z
    radius = min(100, int(np.ceil(max(k[0, 0], k[1, 1]) * radius_m / z * 1.5)) + 2)
    y0, y1 = max(0, y-radius), min(depth.shape[0], y+radius+1)
    x0, x1 = max(0, x-radius), min(depth.shape[1], x+radius+1)
    values = depth[y0:y1, x0:x1]
    valid = np.isfinite(values) & (values > 0) & (np.abs(values-z) < radius_m)
    safe = valid.copy()
    for dy, dx in ((0, 1), (0, -1), (1, 0), (-1, 0)):
        neighbor = np.roll(values, (dy, dx), (0, 1))
        safe &= np.isfinite(neighbor) & (neighbor > 0) & (np.abs(values-neighbor) < limits["depth_jump"])
    safe[[0, -1], :] = False
    safe[:, [0, -1]] = False
    if not safe[y-y0, x-x0]:
        raise ValueError("target at invalid depth edge/discontinuity")
    yy, xx = np.mgrid[y0:y1, x0:x1]
    pixels = np.column_stack((xx[safe], yy[safe])).astype(float)
    rays = cv2.undistortPoints(pixels.reshape(-1, 1, 2), k, distortion).reshape(-1, 2)
    cloud = np.column_stack((rays, np.ones(len(rays)))) * values[safe, None]
    cloud = cloud[np.linalg.norm(cloud-target, axis=1) <= radius_m]
    if len(cloud) < limits["min_points"]:
        raise ValueError("too few surface points")
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
        inliers = np.abs((cloud-a) @ (normal/length)) <= limits["plane_band"]
        if inliers.sum() > best.sum():
            best = inliers
    if best.sum() < limits["min_points"] or best.mean() < limits["inlier_ratio"]:
        raise ValueError("plane consensus below selected visual threshold")
    points = cloud[best]
    center = points.mean(axis=0)
    _, singular, vh = np.linalg.svd(points-center, full_matrices=False)
    if singular[1] / np.sqrt(len(points)) < limits["spread"]:
        raise ValueError("surface support nearly collinear")
    normal = vh[-1]
    rms = np.sqrt(np.mean(((points-center) @ normal)**2))
    if rms > limits["rms"] or abs((target-center) @ normal) > limits["target_residual"]:
        raise ValueError("plane RMS/target residual exceeds bound")
    if normal @ target > 0:
        normal = -normal
    return target, normal, dict(surface_points=int(len(points)),
                                plane_rms_m=float(rms), plane_inlier_ratio=float(best.mean()))


def texture_check(image, pixel, *, relaxed=False):
    point = np.asarray(pixel, dtype=float)
    if point.shape != (2,) or not np.isfinite(point).all():
        return dict(valid=False, reason="invalid tracking pixel", feature_count=0)
    x, y = np.floor(point).astype(int)
    if not (0 <= x < image.shape[1] and 0 <= y < image.shape[0]):
        return dict(valid=False, reason="tracking pixel outside image", feature_count=0)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    mask = np.zeros(gray.shape, dtype=np.uint8)
    mask[max(0, y-40):min(gray.shape[0], y+40),
         max(0, x-40):min(gray.shape[1], x+40)] = 255
    points = cv2.goodFeaturesToTrack(gray, 150, .01, 4, mask=mask, blockSize=5)
    count = 0 if points is None else len(points)
    minimum = (RELAXED if relaxed else STANDARD)["features"]
    return dict(valid=count >= minimum, feature_count=count,
                reason="" if count >= minimum else "too few texture features")


def check_point(image, depth, meta, pixel, *, relaxed=False):
    k, d = np.asarray(meta["intrinsic"], dtype=float), np.asarray(meta["distortion"], dtype=float)
    result = dict(geometry_valid=False, tracking_initialization_possible=False)
    try:
        point, normal, quality = surface(depth, pixel, k, d, relaxed=relaxed)
        result.update(geometry_valid=True, point_camera_m=point.tolist(),
                      normal_out_camera=normal.tolist(), quality=quality)
    except (ValueError, cv2.error) as error:
        result["geometry_reason"] = str(error)
    # This counts initialization features; it is NOT LK inliers or a Robot observation.
    texture = texture_check(image, pixel, relaxed=relaxed)
    result["texture"] = texture
    result["tracking_initialization_possible"] = result["geometry_valid"] and texture["valid"]
    return result
