"""Read-only live Gemini RGB-D model preview; never imports Redis or Robot."""
import argparse
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import time

import cv2

from .credentials import get_api_key
from .detection import response_to_boxes, scan_start_from_boxes
from .geometry import check_point
from .gpu_model import load_cuda_model
from .live_protocol import unpack_frame
from .live_window import WindowSender

CAMERA_PYTHON = Path("/home/cair-jacen/anaconda3/envs/camera/bin/python")


def evaluate(image, depth, meta, model, *, class_name="carotid", min_confidence=0.5, margin=0,
             inference_confidence=None, relaxed_vision=False):
    """Evaluate the exact captured frame; candidate is display-only."""
    started = time.monotonic_ns()
    arrival_age_ms = (started - meta["capture_monotonic_ns"]) / 1e6
    response = (model.infer(image) if inference_confidence is None else
                model.infer(image, confidence=inference_confidence))
    raw = response_to_boxes(response)
    inference_ms = (time.monotonic_ns() - started) / 1e6
    result = dict(capture_monotonic_ns=meta["capture_monotonic_ns"],
                  image_size=[image.shape[1], image.shape[0]],
                  detections=raw["predictions"], inference_ms=inference_ms,
                  arrival_age_ms=arrival_age_ms,
                  vision_profile="relaxed" if relaxed_vision else "standard")
    try:
        point = scan_start_from_boxes(raw, image.shape[1], image.shape[0],
                                      class_name=class_name, min_confidence=min_confidence,
                                      margin=margin)
    except ValueError as error:
        result.update(status="rejected", reason=str(error))
    else:
        result["prediction"] = point
        geometry = check_point(image, depth, meta, point["scan_start"], relaxed=relaxed_vision)
        result["geometry"] = geometry
        if not geometry["tracking_initialization_possible"]:
            result.update(status="rejected", reason=geometry.get("geometry_reason") or
                          geometry["texture"]["reason"])
        else:
            result.update(status="candidate", reason="")
    result["age_ms"] = (time.monotonic_ns() - meta["capture_monotonic_ns"]) / 1e6
    result["age_valid"] = 0 <= result["age_ms"] <= 200
    if not result["age_valid"] and result["status"] == "candidate":
        result.update(status="stale", reason="capture-to-preview age exceeds 200 ms")
    return result


def draw_preview(image, result, *, class_name="carotid", min_confidence=0.5):
    """Overlay all boxes, but only mark P0 when one usable candidate survives."""
    canvas = image.copy()
    accepted = result["status"] == "candidate"
    selected = result.get("prediction", {}).get("box_xyxy") if accepted else None
    for box in result["detections"]:
        x, y, w, h = (float(box[k]) for k in ("x", "y", "width", "height"))
        confidence = float(box["confidence"])
        xyxy = [x-w/2, y-h/2, x+w/2, y+h/2]
        eligible = box.get("class") == class_name and confidence >= min_confidence
        is_selected = eligible and selected == xyxy
        color = (0, 190, 0) if is_selected else ((0, 165, 255) if eligible else (180, 180, 180))
        x0, y0 = max(0, round(xyxy[0])), max(0, round(xyxy[1]))
        x1, y1 = min(canvas.shape[1]-1, round(xyxy[2])), min(canvas.shape[0]-1, round(xyxy[3]))
        if x0 <= x1 and y0 <= y1:
            cv2.rectangle(canvas, (x0, y0), (x1, y1), color, 2)
            label = "%s %.2f" % (box.get("class", "?"), confidence)
            cv2.putText(canvas, label, (x0, max(60, y0-4)),
                        cv2.FONT_HERSHEY_SIMPLEX, .45, color, 1)
    if accepted:
        x, y = result["prediction"]["scan_start"]
        cv2.drawMarker(canvas, (round(x), round(y)), (0, 255, 0),
                       cv2.MARKER_CROSS, 22, 2)
    status = result["status"].upper()
    line = "%s %s | model %.0f ms | age %.0f ms" % (
        status, result.get("vision_profile", "standard"), result["inference_ms"], result["age_ms"])
    cv2.rectangle(canvas, (0, 0), (canvas.shape[1], 48), (0, 0, 0), -1)
    cv2.putText(canvas, line, (8, 18), cv2.FONT_HERSHEY_SIMPLEX,
                .48, (0, 255, 0) if accepted else (0, 180, 255), 1)
    cv2.putText(canvas, result.get("reason", "")[:85], (8, 40),
                cv2.FONT_HERSHEY_SIMPLEX, .38, (0, 180, 255), 1)
    cv2.rectangle(canvas, (0, canvas.shape[0]-25), (canvas.shape[1], canvas.shape[0]),
                  (0, 0, 0), -1)
    cv2.putText(canvas, "q/Esc: quit",
                (8, canvas.shape[0]-7), cv2.FONT_HERSHEY_SIMPLEX, .42, (255, 255, 255), 1)
    return canvas


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model-id", required=True, help="Exact latest Roboflow Deploy model ID")
    parser.add_argument("--class-name", default="carotid")
    parser.add_argument("--serial", default="CV2L360000HZ")
    parser.add_argument("--min-confidence", type=float, help="Selected-box threshold; default 0.25")
    parser.add_argument("--standard-vision", action="store_true", help="Use the original stricter visual thresholds")
    parser.add_argument("--margin", type=int, default=0)
    parser.add_argument("--camera-python", type=Path, default=CAMERA_PYTHON)
    parser.add_argument("--headless", action="store_true", help="No window; requires --max-frames")
    parser.add_argument("--max-frames", type=int, default=0, help="0: until q; positive: stop after N frames")
    parser.add_argument("--log", type=Path, default=Path(__file__).parent / "reports" /
                        ("live_preview_%d.jsonl" % time.time_ns()))
    args = parser.parse_args(argv)
    if args.max_frames < 0 or (args.headless and args.max_frames == 0):
        parser.error("--headless requires --max-frames > 0")
    relaxed_vision = not args.standard_vision
    min_confidence = args.min_confidence if args.min_confidence is not None else (.25 if relaxed_vision else .5)
    if not 0 <= min_confidence <= 1:
        parser.error("--min-confidence must be in [0, 1]")
    if not args.camera_python.is_file():
        parser.error("camera Python environment not found")
    try:
        key = get_api_key()
    except ValueError as error:
        parser.error(str(error))
    if not key:
        parser.error("Roboflow API Key missing; run python -m infer.Carotid_begin.credentials save")
    cache = Path(os.environ.setdefault("MODEL_CACHE_DIR", str(Path(__file__).parent / "data/model_cache")))
    cache.mkdir(parents=True, exist_ok=True)
    print("Loading model...", flush=True)
    model = load_cuda_model(args.model_id, key)
    parent, child = socket.socketpair()
    parent.settimeout(20)
    command = [str(args.camera_python), "-m", "infer.Carotid_begin.camera_stream",
               "--serial", args.serial, "--socket-fd", str(child.fileno())]
    camera_env = os.environ.copy()
    camera_env.pop("LD_PRELOAD", None)
    camera_env.pop("LD_LIBRARY_PATH", None)
    camera_env.pop("PYTHONHOME", None)
    camera_env["CONDA_PREFIX"] = str(args.camera_python.parent.parent)
    camera_env["PATH"] = str(args.camera_python.parent) + os.pathsep + camera_env.get("PATH", "")
    process = subprocess.Popen(command, pass_fds=(child.fileno(),),
                               cwd=Path(__file__).resolve().parents[2], env=camera_env)
    child.close()
    window = None
    counts = dict(candidate=0, rejected=0, stale=0)
    try:
        if not args.headless:
            window = WindowSender(sys.executable)
            print("Direct preview window opened; press q or Esc to quit.", flush=True)
        args.log.parent.mkdir(parents=True, exist_ok=True)
        with args.log.open("x", encoding="utf-8") as log:
            while True:
                parent.sendall(b"N")
                image, depth, meta = unpack_frame(parent)
                result = evaluate(image, depth, meta, model, class_name=args.class_name,
                                  min_confidence=min_confidence, margin=args.margin,
                                  inference_confidence=.4,
                                  relaxed_vision=relaxed_vision)
                counts[result["status"]] += 1
                log.write(json.dumps(result, ensure_ascii=False, allow_nan=False) + "\n")
                log.flush()
                if window is not None:
                    preview = draw_preview(image, result, class_name=args.class_name,
                                           min_confidence=min_confidence)
                    try:
                        action = window.show(preview)
                    except (EOFError, OSError) as error:
                        raise RuntimeError("preview window process stopped unexpectedly") from error
                    if action == "quit":
                        break
                if args.max_frames and sum(counts.values()) >= args.max_frames:
                    break
    except KeyboardInterrupt:
        print("Stopped by Ctrl+C.", flush=True)
    except (EOFError, socket.timeout, ConnectionError) as error:
        try:
            worker_code = process.wait(timeout=1)
        except subprocess.TimeoutExpired:
            worker_code = "running"
        raise RuntimeError("camera stream unavailable (worker exit=%s); close other Gemini programs and check camera worker output" % worker_code) from error
    finally:
        parent.close()
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            process.terminate()
            try:
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        if window is not None:
            window.close()
    print(json.dumps({"frames": sum(counts.values()), "counts": counts,
                      "log": str(args.log)},
                     ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
