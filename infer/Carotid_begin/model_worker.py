"""Persistent private-socket model worker for Camera_wrist model selection."""
import argparse
import os
from pathlib import Path
import socket

from .config import ModelConfig
from .credentials import get_api_key
from .gpu_model import load_cuda_model
from .live_protocol import MAX_READY, send_json, unpack_frame
from .pipeline import DetectionPipeline


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--socket-fd', type=int, required=True)
    parser.add_argument('--model-id', required=True)
    parser.add_argument('--class-name', default='carotid')
    parser.add_argument('--min-confidence', type=float, default=.4)
    parser.add_argument('--inference-confidence', type=float, default=.4)
    parser.add_argument('--margin', type=int, default=0)
    args = parser.parse_args(argv)
    try:
        if args.socket_fd < 0:
            raise ValueError('socket-fd must be nonnegative')
        config = ModelConfig(args.model_id, args.class_name, args.min_confidence,
                             args.inference_confidence, args.margin)
    except ValueError as error:
        parser.error(str(error))
    return args.socket_fd, config


def serve(sock, pipeline):
    """Send readiness once, then reply to each frame until the peer closes.

    The caller owns the socket. Truncated/invalid frames and model runtime errors
    terminate the session; a rejected detection returns valid=False and continues.
    """
    send_json(sock, {'ready': True, 'model_id': pipeline.config.model_id}, limit=MAX_READY)
    while True:
        try:
            image, _depth, meta = unpack_frame(sock)
        except EOFError:
            return
        response = pipeline.process(image, meta['capture_monotonic_ns'])
        send_json(sock, response)


def main(argv=None):
    socket_fd, config = parse_args(argv)
    with socket.socket(fileno=socket_fd) as sock:
        key = get_api_key()
        if not key:
            raise ValueError('Roboflow API Key missing; save it with credentials.py')
        os.environ.setdefault('MODEL_CACHE_DIR', str(Path(__file__).parent / 'data/model_cache'))
        model = load_cuda_model(config.model_id, key)
        serve(sock, DetectionPipeline(model, config))


if __name__ == '__main__':
    main()
