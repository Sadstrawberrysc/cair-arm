"""One SDK reader, one replaceable frame slot; no receipt-time capture fallback."""
import threading
import time


class LatestFrameReader:
    def __init__(self, pipeline):
        self.pipeline = pipeline
        self.condition = threading.Condition()
        self.stopping = threading.Event()
        self.slot = None
        self.error = None
        self.dropped = 0
        self.thread = threading.Thread(target=self._run, name='wrist-latest-frame', daemon=True)

    def start(self):
        self.thread.start()

    def _run(self):
        try:
            while not self.stopping.is_set():
                frames = self.pipeline.wait_for_frames(100)
                received_ns = time.monotonic_ns()
                if frames is None:
                    continue
                with self.condition:
                    if self.slot is not None:
                        self.dropped += 1
                    self.slot = (frames, received_ns)
                    self.condition.notify_all()
        except Exception as error:
            with self.condition:
                self.error = error
                self.condition.notify_all()

    def take(self, timeout=.1):
        with self.condition:
            self.condition.wait_for(
                lambda: self.slot is not None or self.error is not None or self.stopping.is_set(), timeout)
            if self.error is not None:
                raise RuntimeError('RGB-D capture worker failed: %s' % self.error) from self.error
            value, self.slot = self.slot, None
            if value is None:
                return None, None, self.dropped
            return value[0], value[1], self.dropped

    def stop(self):
        self.stopping.set()
        with self.condition:
            self.condition.notify_all()
        self.thread.join(timeout=1)
        return not self.thread.is_alive()
