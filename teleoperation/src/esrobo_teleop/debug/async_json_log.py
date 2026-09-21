"""Bounded, non-blocking producer for diagnostic JSONL files."""

import json
import queue
import threading

import numpy as np


def numpy_json_default(value):
    """Preserve numeric/boolean types in both file and UDP diagnostics."""
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(f"Unsupported diagnostic type: {type(value).__name__}")


class AsyncJsonLog:
    def __init__(self, path, capacity=256):
        self.path = path
        self.queue = queue.Queue(maxsize=capacity)
        self.dropped = 0
        self.error = None
        self.serialization_errors = 0
        self.last_record_error = None
        self._closed = threading.Event()
        self._thread = threading.Thread(target=self._run, name="teleop-diagnostic-log", daemon=True)
        self._thread.start()

    def submit(self, record):
        if self._closed.is_set() or self.error is not None:
            self.dropped += 1
            return False
        try:
            self.queue.put_nowait(record)
            return True
        except queue.Full:
            self.dropped += 1
            return False

    def _run(self):
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            with self.path.open("a", encoding="utf-8") as stream:
                while not self._closed.is_set() or not self.queue.empty():
                    try:
                        item = self.queue.get(timeout=.05)
                    except queue.Empty:
                        continue
                    try:
                        try:
                            line = json.dumps(item, allow_nan=False, default=numpy_json_default)
                        except (TypeError, ValueError) as exc:
                            # One malformed record must not remove all subsequent
                            # fault evidence. I/O errors still stop this writer.
                            self.serialization_errors += 1
                            self.dropped += 1
                            self.last_record_error = str(exc)
                            if self.serialization_errors == 1:
                                print(f"[teleop] diagnostic record dropped: {exc}", flush=True)
                            continue
                        stream.write(line + "\n")
                        stream.flush()
                    finally:
                        self.queue.task_done()
        except (OSError, TypeError, ValueError) as exc:
            self.error = str(exc)
            print(f"[teleop] diagnostic log unavailable: {exc}", flush=True)

    def close(self, timeout=1.0):
        self._closed.set()
        self._thread.join(timeout=timeout)
