"""Bounded binary transport for one aligned RGB-D frame over a private socketpair."""
import io
import json
import struct

import numpy as np

MAX_PAYLOAD = 16 * 1024 * 1024
MAX_READY = 4096
MAX_RESPONSE = 65536


def send_json(sock, response, *, limit=MAX_RESPONSE):
    """Write the existing uint32 network-order length + UTF-8 JSON contract."""
    payload = json.dumps(response, allow_nan=False).encode('utf-8')
    if not 0 < len(payload) <= limit:
        raise ValueError('model response payload too large')
    sock.sendall(struct.pack('!I', len(payload)) + payload)


def validate_frame(image, depth, meta):
    """Apply identical RGB-D and source-identity checks on both sides."""
    if (not isinstance(image, np.ndarray) or image.dtype != np.uint8 or
            image.ndim != 3 or image.shape[2] != 3 or 0 in image.shape):
        raise ValueError('invalid BGR frame')
    if (not isinstance(depth, np.ndarray) or depth.dtype != np.float32 or
            depth.shape != image.shape[:2]):
        raise ValueError('invalid aligned depth')
    if not isinstance(meta, dict):
        raise ValueError('frame metadata must be an object')
    stamp = meta.get('capture_monotonic_ns')
    if type(stamp) is not int or stamp < 0:
        raise ValueError('invalid capture_monotonic_ns')


def recv_exact(sock, length):
    if length < 0:
        raise ValueError('invalid receive length')
    chunks = []
    while length:
        block = sock.recv(length)
        if not block:
            if chunks:
                raise ValueError('truncated socket payload')
            raise EOFError("camera process closed the frame socket")
        chunks.append(block)
        length -= len(block)
    return b"".join(chunks)


def pack_frame(image, depth, meta):
    validate_frame(image, depth, meta)
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
    try:
        payload = recv_exact(sock, length)
    except EOFError as error:
        raise ValueError('missing frame payload') from error
    with np.load(io.BytesIO(payload), allow_pickle=False) as frame:
        image = frame["image"]
        depth = frame["depth"]
        meta = json.loads(str(frame["meta"].item()))
    validate_frame(image, depth, meta)
    return image, depth, meta
