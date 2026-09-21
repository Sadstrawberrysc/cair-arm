#!/usr/bin/env python3
"""Gemini projection commissioning UI; publishes NO observations or commands."""
import argparse
from collections import deque
import json
from pathlib import Path
import time

import cv2
import numpy as np
from gemini_config import WIDTH, HEIGHT, configure_close_range, stream_profiles

from wrist_projection import (SEED_KEY, STATE_CHANNEL, calibration, validate_seed,
                              interpolate_pose, project, depth_at)


class CaptureClock:
    """SDK global capture clock -> local CLOCK_MONOTONIC, bounded wall steps.

    SDK documents global timestamp as DEVICE CAPTURE converted to host clock.
    System timestamp and Redis receipt are never substituted. These checks do
    not measure camera exposure error or robot feedback transport latency.
    """
    def __init__(self):
        self.offset = None
        self.count = 0
        self.last = 0

    def convert(self, global_us):
        before = time.monotonic_ns()
        wall = time.time_ns()
        after = time.monotonic_ns()
        offset = (before+after)//2-wall
        if after-before > 1_000_000 or (self.offset is not None and abs(offset-self.offset)>2_000_000):
            self.count = 0
            self.offset = offset
            raise ValueError("clock sampling/step exceeds bound")
        self.offset = offset
        capture = int(global_us)*1000+offset
        if global_us <= 0 or capture <= self.last or not 0 <= after-capture <= 200_000_000:
            self.count = 0
            raise ValueError("global capture clock invalid/stale/repeated; no receipt-time fallback")
        self.last = capture
        self.count += 1
        if self.count < 30:
            raise ValueError("warming up global clock: %d/30" % self.count)
        return capture


def color_bgr(frame, ob):
    array = np.frombuffer(frame.get_data(), dtype=np.uint8)
    h, w = frame.get_height(), frame.get_width()
    fmt = frame.get_format()
    if fmt == ob.OBFormat.MJPG:
        image = cv2.imdecode(array, cv2.IMREAD_COLOR)
    elif fmt == ob.OBFormat.RGB:
        image = cv2.cvtColor(array.reshape(h, w, 3), cv2.COLOR_RGB2BGR)
    elif fmt == ob.OBFormat.BGR:
        image = array.reshape(h, w, 3).copy()
    else:
        raise ValueError("unsupported color format: " + str(fmt))
    if image is None or image.shape[:2] != (h, w):
        raise ValueError("color decode failed")
    return image


def aligned_rgbd(frame, align):
    """SDK filters can yield no output while waiting for matching frames."""
    if frame is None:
        return None, None, "NO FRAMES"
    result = align.process(frame)
    if result is None:
        return None, None, "WAITING FOR RGB-D ALIGNMENT"
    frames = result.as_frame_set()
    if frames is None:
        return None, None, "NO ALIGNED FRAMESET"
    color, depth = frames.get_color_frame(), frames.get_depth_frame()
    if color is None or depth is None:
        return None, None, "INCOMPLETE ALIGNED RGB-D"
    return color, depth, None


def run(args):
    import redis
    import pyorbbecsdk as ob
    sdk_log = Path(__file__).resolve().parent / "log" / "sdk"
    sdk_log.mkdir(parents=True, exist_ok=True)
    args.log.parent.mkdir(parents=True, exist_ok=True)
    ob.Context.set_logger_to_file(ob.OBLogLevel.ERROR, str(sdk_log))
    _, gh = calibration(args.global_calibration, "T_base_camera", "d455_color_optical_to_rm75_base")
    _, wh = calibration(args.wrist_calibration, "T_armtip_camera", "gemini305_color_optical_to_rm75_armtip")
    hashes = {"global": gh, "wrist": wh}
    client = redis.Redis(host="127.0.0.1", port=7777, socket_timeout=1, socket_connect_timeout=1)
    raw_seed = client.get(SEED_KEY)
    if raw_seed is None:
        raise ValueError("no fresh seed; complete successful coarse handoff first")
    seed = json.loads(raw_seed)
    point = validate_seed(seed, hashes, args.serial)
    subscriber = client.pubsub(ignore_subscribe_messages=True)
    subscriber.subscribe(STATE_CHANNEL)
    states = deque(maxlen=100)
    context = ob.Context()
    device = context.query_devices().get_device_by_serial_number(args.serial)
    if device is None or not device.is_global_timestamp_supported():
        raise ValueError("serial unavailable or SDK global capture clock unsupported")
    camera_settings = configure_close_range(device, ob)
    device.enable_global_timestamp(True)
    pipeline = ob.Pipeline(device)
    config = ob.Config()
    color_profile, depth_profile = stream_profiles(pipeline, ob)
    config.enable_stream(color_profile)
    config.enable_stream(depth_profile)
    align = ob.AlignFilter(align_to_stream=ob.OBStreamType.COLOR_STREAM)
    intr = color_profile.get_intrinsic()
    dist = color_profile.get_distortion()
    # OpenCV implements forward Brown/rational distortion, not inverse Brown.
    if dist.model not in (ob.OBCameraDistortionModel.BROWN_CONRADY,
                          ob.OBCameraDistortionModel.BROWN_CONRADY_K6,
                          ob.OBCameraDistortionModel.NONE):
        raise ValueError("unverified SDK distortion model: " + str(dist.model))
    k = np.array([[intr.fx, 0, intr.cx], [0, intr.fy, intr.cy], [0, 0, 1.]])
    d = np.array([dist.k1, dist.k2, dist.p1, dist.p2, dist.k3, dist.k4, dist.k5, dist.k6])
    if dist.model == ob.OBCameraDistortionModel.NONE:
        d[:] = 0
    clock = CaptureClock()
    tracker = None
    if getattr(args, "track", False):
        from local_tracker import LocalTracker
        tracker = LocalTracker()
    previous = None
    runtime = None
    last_sequence = -1
    log_path = args.log
    print("Projection only; no tracking or motion. Log:", log_path, flush=True)
    started = False
    try:
        with log_path.open("x") as log:
            def record(item):
                log.write(json.dumps(item, allow_nan=False)+"\n")
                log.flush()
            from tracking_diagnostics import TrackingDiagnostics
            record(camera_settings)
            diagnostics = TrackingDiagnostics(log_path, record)
            record(dict(type="configuration", seed=seed, intrinsic=k.tolist(), distortion=d.tolist(),
                        serial=args.serial, sdk=str(ob.__version__),
                        optical_frame_verified=False, exposure_mapping_independently_verified=False))
            pipeline.start(config)
            started = True
            pipeline.enable_frame_sync()
            last_complete_rgbd = time.monotonic()
            while True:
                frame = pipeline.wait_for_frames(100)
                color, depth, missing_reason = aligned_rgbd(frame, align)
                if missing_reason is not None:
                    previous = None
                    if tracker is not None:
                        tracker.lose(missing_reason)
                        diagnostics.emit(dict(valid=False, reason=tracker.reason), tracker)
                    image = np.zeros((HEIGHT, WIDTH, 3), np.uint8)
                    cv2.putText(image, missing_reason, (10, 40), 0, .6, (0, 0, 255), 2)
                    record(dict(type="projection", valid=False, reason=missing_reason,
                                receive_monotonic_ns=time.monotonic_ns()))
                    cv2.imshow("Gemini global-point projection ONLY", image)
                    if cv2.waitKey(1) & 255 == ord('q'):
                        break
                    if time.monotonic() - last_complete_rgbd > 5:
                        raise ValueError("no complete aligned RGB-D for 5 seconds: " + missing_reason)
                    continue
                last_complete_rgbd = time.monotonic()
                image = color_bgr(color, ob)
                current = None
                reason = "waiting for bracketed state"
                try:
                    capture = clock.convert(color.get_global_timestamp_us())
                    depth_capture = int(depth.get_global_timestamp_us())*1000+clock.offset
                    if abs(capture-depth_capture)>20_000_000:
                        raise ValueError("RGB-depth capture skew exceeds 20 ms")
                    depth_m = np.frombuffer(depth.get_data(), np.uint16).reshape(
                        depth.get_height(), depth.get_width()).astype(float)*depth.get_depth_scale()/1000
                    if depth_m.shape != image.shape[:2]:
                        raise ValueError("aligned depth dimensions differ from color")
                    current = (capture, image.copy(), depth_m)
                except ValueError as error:
                    reason = str(error)
                # Bounded draining; protocol state carries original source time.
                for _ in range(200):
                    message = subscriber.get_message(timeout=0)
                    if message is None:
                        break
                    if message["type"] != "message":
                        continue
                    try:
                        state = json.loads(message["data"])
                        if (state["session_id"] != seed["session_id"] or state["target_id"] != seed["target_id"]
                                or state["calibration_sha256"] != hashes):
                            continue
                        if runtime is None:
                            runtime = state["runtime_session_id"]
                        if state["runtime_session_id"] != runtime:
                            raise RuntimeError("Robot restarted; restart preview with a fresh handoff")
                        if state["sequence"] <= last_sequence:
                            continue
                        last_sequence = state["sequence"]
                        states.append(state)
                    except (KeyError, TypeError, ValueError):
                        states.clear()
                entry = dict(type="projection", valid=False, reason=reason)
                tracking_frame = None
                if previous is not None and current is not None:
                    capture, image, depth_m = previous
                    entry["capture_monotonic_ns"] = capture
                    try:
                        age_ms = (time.monotonic_ns()-capture)/1e6
                        if not 0 <= age_ms <= 200:
                            raise ValueError("capture age exceeds 200 ms")
                        pose, span_ms = interpolate_pose(states, capture, seed, hashes)
                        tracking_frame = (capture, image.copy(), depth_m)
                        pixel, camera_point = project(point, pose, k, d, image.shape[1], image.shape[0])
                        depth_value = depth_at(depth_m, pixel)
                        entry.update(valid=True, reason="candidate_requires_visual_and_clock_validation",
                                     pixel=pixel.tolist(), point_camera_m=camera_point.tolist(),
                                     measured_depth_m=depth_value, depth_residual_m=depth_value-camera_point[2],
                                     pose_span_ms=span_ms, capture_age_ms=age_ms, T_base_camera=pose.tolist())
                        cv2.circle(image, tuple(np.floor(pixel).astype(int)), 8, (0, 0, 255), 2)
                        cv2.putText(image, "z %.3f depth %.3f m" % (camera_point[2], depth_value),
                                    (10, 70), 0, .55, (0, 255, 255), 1)
                    except (ValueError, KeyError, TypeError) as error:
                        entry["reason"] = str(error)
                previous = current
                tracking = None
                if tracker is not None:
                    if tracking_frame is None:
                        tracker.lose(entry["reason"])
                    elif tracker.active:
                        ts, raw_image, raw_depth = tracking_frame
                        tracking = tracker.update(raw_image, raw_depth, k, d, ts)
                    if tracking is None:
                        tracking = dict(valid=False, reason=tracker.reason)
                    diagnostics.emit(tracking, tracker)
                    if tracking["valid"]:
                        cv2.circle(image, tuple(np.floor(tracking["pixel"]).astype(int)), 6, (0, 255, 0), 2)
                    cv2.putText(image, "TRACK: " + tracking["reason"][:65], (10, 95), 0, .45, (0, 255, 255), 1)
                cv2.putText(image, "PREVIEW ONLY / NOT VALIDATED", (10, 22), 0, .55, (0, 255, 255), 2)
                cv2.putText(image, entry["reason"][:82], (10, 45), 0, .4, (0, 0, 255), 1)
                help_text = "q: quit  c: check" + ("  i: confirm/reinitialize  p: pause" if tracker is not None else " (no motion)")
                cv2.putText(image, help_text, (10, 465), 0, .45, (255, 255, 255), 1)
                cv2.imshow("Gemini global-point projection ONLY", image)
                key = cv2.waitKey(1) & 255
                record(entry)
                if tracker is not None and key == ord('p'):
                    tracker.lose('operator paused; confirm initialization with i', force=True)
                    diagnostics.emit(dict(valid=False, reason=tracker.reason), tracker)
                if tracker is not None and key == ord('i') and entry["valid"] and tracking_frame is not None:
                    ts, raw_image, raw_depth = tracking_frame
                    result = tracker.initialize(raw_image, raw_depth, entry["pixel"], k, d, ts)
                    diagnostics.emit(result, tracker, kind="tracking_initialization")
                if key == ord('c') and entry["valid"]:
                    record(dict(type="operator_visual_check", projection=entry,
                                note="skin location checked; not independent spatial/clock acceptance"))
                if key == ord('q'):
                    break
    finally:
        if started:
            pipeline.stop()
        subscriber.close()
        client.close()
        cv2.destroyAllWindows()


def main():
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--serial", default="CV2L360000HZ")
    parser.add_argument("--track", action="store_true", help="experimental local RGB-D tracking; i confirms initialization, p pauses; no Redis commands")
    parser.add_argument("--global-calibration", type=Path, default=root/"infer/hand_eye_calibration-main/d455_to_rm75_base.json")
    parser.add_argument("--wrist-calibration", type=Path, default=Path(__file__).with_name("gemini305_to_rm75_armtip.json"))
    parser.add_argument("--log", type=Path, default=Path(__file__).resolve().parent / "log" / ("wrist_projection_%d.jsonl" % time.time_ns()))
    args = parser.parse_args()
    try:
        run(args)
    except KeyboardInterrupt:
        return 130
    except Exception as error:
        print("BLOCKED:", error)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
