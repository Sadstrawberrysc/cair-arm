"""Bounded binary transport for one aligned RGB-D frame over a private socketpair."""
import io
import json
import socket
import struct

import numpy as np

MAX_PAYLOAD = 16 * 1024 * 1024


def recv_exact(sock, length):
    chunks = []
    while length:
        block = sock.recv(length)
        if not block:
            raise EOFError("camera process closed the frame socket")
        chunks.append(block)
        length -= len(block)
    return b"".join(chunks)


def pack_frame(image, depth, meta):
    if image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3:
        raise ValueError("invalid BGR frame")
    if depth.dtype != np.float32 or depth.shape != image.shape[:2]:
        raise ValueError("invalid aligned depth")
    out = io.BytesIO()
    np.savez(out, image=image, depth=depth,
                        meta=json.dumps(meta, allow_nan=False))
    payload = out.getvalue()
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("frame payload too large")
    return struct.pack("!I", len(payload)) + payload


def unpack_frame(sock):
    length = struct.unpack("!I", recv_exact(sock, 4))[0]
    if not 0 < length <= MAX_PAYLOAD:
        raise ValueError("invalid frame payload length")
    with np.load(io.BytesIO(recv_exact(sock, length)), allow_pickle=False) as frame:
        image = frame["image"]
        depth = frame["depth"]
        meta = json.loads(str(frame["meta"].item()))
    if image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3:
        raise ValueError("invalid transported BGR frame")
    if depth.dtype != np.float32 or depth.shape != image.shape[:2]:
        raise ValueError("invalid transported aligned depth")
    return image, depth, meta
