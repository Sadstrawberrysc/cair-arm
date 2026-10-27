"""KWR57 fixed-length RS422 framing and kgf/kgf*m to SI conversion."""

import math
import struct


class Kwr57Parser:
    """Keep at most 28 bytes; CR/LF inside float payloads is ordinary data."""

    def __init__(self) -> None:
        self.buffer = bytearray()
        self.statistics = dict(bytes_received=0, valid_frames=0, framing_errors=0,
                               numeric_errors=0, noise_bytes_discarded=0)

    def feed(self, data: bytes) -> list[tuple[float, ...]]:
        frames = []
        self.statistics["bytes_received"] += len(data)
        for value in data:
            self.buffer.append(value)
            while self.buffer:
                if self.buffer[0] not in (0x48, 0x49):
                    del self.buffer[0]
                    self.statistics["noise_bytes_discarded"] += 1
                    continue
                if len(self.buffer) < 2:
                    break
                if self.buffer[1] != 0xAA:
                    del self.buffer[0]
                    self.statistics["noise_bytes_discarded"] += 1
                    continue
                if len(self.buffer) < 28:
                    break
                if self.buffer[26:28] != b"\r\n":
                    del self.buffer[0]
                    self.statistics["framing_errors"] += 1
                    self.statistics["noise_bytes_discarded"] += 1
                    continue
                values = struct.unpack("<6f", self.buffer[2:26])
                self.buffer.clear()
                if all(math.isfinite(item) for item in values):
                    frames.append(tuple(item * 9.80665 for item in values))
                    self.statistics["valid_frames"] += 1
                else:
                    self.statistics["numeric_errors"] += 1
        return frames
