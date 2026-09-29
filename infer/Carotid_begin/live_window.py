"""Dedicated OpenCV window process for read-only P0 preview frames."""
import argparse
import os
from pathlib import Path
import socket
import struct
import subprocess

import cv2
import numpy as np

from .live_protocol import MAX_PAYLOAD, recv_exact

WINDOW = "Carotid P0 live preview (no Robot)"


def key_reply(key, closed):
    if key in (27, ord("q")) or closed:
        return b"Q"
    return b"R"


def show_frames(sock):
    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    try:
        while True:
            try:
                length = struct.unpack("!I", recv_exact(sock, 4))[0]
            except EOFError:
                break
            if not 0 < length <= MAX_PAYLOAD:
                raise ValueError("invalid preview frame length")
            image = cv2.imdecode(np.frombuffer(recv_exact(sock, length), np.uint8), cv2.IMREAD_COLOR)
            if image is None:
                raise ValueError("cannot decode preview frame")
            cv2.imshow(WINDOW, image)
            key = cv2.waitKey(1) & 255
            closed = cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1
            reply = key_reply(key, closed)
            sock.sendall(reply)
            if reply == b"Q":
                break
    finally:
        cv2.destroyAllWindows()


class WindowSender:
    """Send rendered frames to the GUI process and wait for its draw/quit reply."""
    def __init__(self, python):
        parent, child = socket.socketpair()
        parent.settimeout(4)
        env = os.environ.copy()
        env.pop("LD_PRELOAD", None)
        env.pop("PYTHONHOME", None)
        env["CONDA_PREFIX"] = str(Path(python).parent.parent)
        env["PATH"] = str(Path(python).parent) + os.pathsep + env.get("PATH", "")
        command = [str(python), "-m", "infer.Carotid_begin.live_window",
                   "--socket-fd", str(child.fileno())]
        try:
            self.process = subprocess.Popen(command, pass_fds=(child.fileno(),),
                                            cwd=Path(__file__).resolve().parents[2], env=env)
        finally:
            child.close()
        self.sock = parent

    def show(self, image):
        ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 90])
        if not ok:
            raise ValueError("preview JPEG encode failed")
        payload = encoded.tobytes()
        if len(payload) > MAX_PAYLOAD:
            raise ValueError("preview frame too large")
        self.sock.sendall(struct.pack("!I", len(payload)) + payload)
        reply = recv_exact(self.sock, 1)
        if reply not in (b"R", b"Q"):
            raise ValueError("invalid window response")
        return {b"R": "continue", b"Q": "quit"}[reply]

    def close(self):
        self.sock.close()
        try:
            self.process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self.process.terminate()
            try:
                self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--socket-fd", type=int, required=True)
    args = parser.parse_args(argv)
    with socket.socket(fileno=args.socket_fd) as sock:
        show_frames(sock)


if __name__ == "__main__":
    main()
