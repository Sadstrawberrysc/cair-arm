"""Private one-shot RGB-D model worker for Camera_wrist model selection."""
import argparse
import json
import os
from pathlib import Path
import socket
import struct
import time

from .credentials import get_api_key
from .detection import response_to_boxes, scan_start_from_boxes
from .gpu_model import load_cuda_model
from .live_protocol import unpack_frame


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--socket-fd', type=int, required=True)
    parser.add_argument('--model-id', required=True)
    parser.add_argument('--class-name', default='carotid')
    parser.add_argument('--min-confidence', type=float, default=.5)
    parser.add_argument('--inference-confidence', type=float, default=.4)
    parser.add_argument('--margin', type=int, default=0)
    args = parser.parse_args(argv)
    if args.inference_confidence is not None and not 0 <= args.inference_confidence <= 1:
        parser.error("--inference-confidence must be in [0, 1]")
    key = get_api_key()
    if not key:
        raise ValueError('Roboflow API Key missing; save it with credentials.py')
    os.environ.setdefault('MODEL_CACHE_DIR', str(Path(__file__).parent / 'data/model_cache'))
    model = load_cuda_model(args.model_id, key)
    with socket.socket(fileno=args.socket_fd) as sock:
        ready = json.dumps({'ready': True, 'model_id': args.model_id}).encode('utf-8')
        sock.sendall(struct.pack('!I', len(ready)) + ready)
        while True:
            try:
                image, _depth, meta = unpack_frame(sock)
            except EOFError:
                break
            raw = None
            started_ns = time.monotonic_ns()
            try:
                result = model.infer(image, confidence=args.inference_confidence)
                inference_ms = (time.monotonic_ns()-started_ns)/1e6
                raw = response_to_boxes(result)
                point = scan_start_from_boxes(raw, image.shape[1], image.shape[0],
                                              class_name=args.class_name,
                                              min_confidence=args.min_confidence,
                                              margin=args.margin)
                response = dict(valid=True, prediction=point,
                                capture_monotonic_ns=meta['capture_monotonic_ns'])
            except (ValueError, TypeError, KeyError) as error:
                response = dict(valid=False, reason=str(error),
                                capture_monotonic_ns=meta['capture_monotonic_ns'])
            if raw is not None:
                response['inference_ms'] = inference_ms
                response['detections'] = raw['predictions']
                response['image_size'] = [image.shape[1], image.shape[0]]
            payload = json.dumps(response, allow_nan=False).encode('utf-8')
            sock.sendall(struct.pack('!I', len(payload)) + payload)


if __name__ == '__main__':
    main()
