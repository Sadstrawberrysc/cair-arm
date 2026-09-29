"""Camera-only worker: capture latest aligned Gemini RGB-D for a private socketpair."""
import argparse
import socket
import threading
import time
from pathlib import Path

import numpy as np

from .camera import CaptureClock, decode, wrist_config
from .live_protocol import pack_frame


def serve(sock, latest, condition, stop):
    last_sent = 0
    try:
        while not stop.is_set():
            if sock.recv(1) != b"N":
                break
            with condition:
                while not stop.is_set() and latest.get("meta", {}).get("capture_monotonic_ns", 0) <= last_sent:
                    condition.wait(timeout=1)
                if stop.is_set():
                    break
                image, depth, meta = latest["image"], latest["depth"], latest["meta"]
            last_sent = meta["capture_monotonic_ns"]
            sock.sendall(pack_frame(image, depth, meta))
    except (BrokenPipeError, ConnectionResetError, OSError) as error:
        print("Camera socket error:", type(error).__name__, str(error), flush=True)
    finally:
        stop.set()
        sock.close()


def run(serial, sock):
    import pyorbbecsdk as ob
    config_api = wrist_config()
    logs = Path(__file__).parent / "log"
    logs.mkdir(parents=True, exist_ok=True)
    ob.Context.set_logger_to_file(ob.OBLogLevel.ERROR, str(logs.resolve()))
    context = ob.Context()
    device = context.query_devices().get_device_by_serial_number(serial)
    if device is None or not device.is_global_timestamp_supported():
        raise ValueError("camera serial unavailable or global capture clock unsupported")
    config_api.configure_close_range(device, ob)
    device.enable_global_timestamp(True)
    pipeline = ob.Pipeline(device)
    config = ob.Config()
    color_profile, depth_profile = config_api.stream_profiles(pipeline, ob)
    config.enable_stream(color_profile)
    config.enable_stream(depth_profile)
    intr, dist = color_profile.get_intrinsic(), color_profile.get_distortion()
    if dist.model not in (ob.OBCameraDistortionModel.NONE, ob.OBCameraDistortionModel.BROWN_CONRADY,
                          ob.OBCameraDistortionModel.BROWN_CONRADY_K6):
        raise ValueError("unsupported distortion model")
    k = np.array([[intr.fx, 0., intr.cx], [0., intr.fy, intr.cy], [0., 0., 1.]])
    d = np.array([dist.k1, dist.k2, dist.p1, dist.p2, dist.k3, dist.k4, dist.k5, dist.k6])
    if dist.model == ob.OBCameraDistortionModel.NONE:
        d[:] = 0
    align = ob.AlignFilter(align_to_stream=ob.OBStreamType.COLOR_STREAM)
    condition = threading.Condition()
    latest = {}
    stop = threading.Event()
    server = threading.Thread(target=serve, args=(sock, latest, condition, stop), daemon=True)
    started = False
    try:
        pipeline.start(config)
        started = True
        pipeline.enable_frame_sync()
        server.start()
        clock = CaptureClock()
        last_valid = time.monotonic()
        last_notice = ""
        while not stop.is_set():
            try:
                image, depth, color_us, depth_us, scale = decode(pipeline.wait_for_frames(100), align, ob)
                capture = clock.convert(color_us)
                if abs(color_us - depth_us) > 20_000:
                    raise ValueError("RGB/depth capture skew exceeds 20 ms")
                meta = dict(intrinsic=k.tolist(), distortion=d.tolist(),
                            capture_monotonic_ns=capture,
                            depth_capture_monotonic_ns=depth_us*1000+clock.offset,
                            receive_monotonic_ns=time.monotonic_ns(),
                            timestamp_basis="sdk_global_capture_to_host_monotonic",
                            color_capture_global_us=color_us, depth_capture_global_us=depth_us,
                            depth_scale_m_per_unit=scale, camera_serial=serial)
                with condition:
                    latest.update(image=image, depth=depth, meta=meta)
                    condition.notify_all()
                last_valid = time.monotonic()
            except ValueError as error:
                notice = str(error)
                display_notice = "capture clock warmup (30 frames)" if notice.startswith("capture clock warmup") else notice
                if display_notice != last_notice:
                    print("Camera waiting:", display_notice, flush=True)
                    last_notice = display_notice
                if time.monotonic() - last_valid > 15:
                    raise RuntimeError("no valid synchronized RGB-D for 15 seconds: " + notice)
    finally:
        stop.set()
        with condition:
            condition.notify_all()
        if started:
            pipeline.stop()
        sock.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--serial", default="CV2L360000HZ")
    parser.add_argument("--socket-fd", type=int, required=True)
    args = parser.parse_args(argv)
    with socket.socket(fileno=args.socket_fd) as sock:
        run(args.serial, sock)


if __name__ == "__main__":
    main()
