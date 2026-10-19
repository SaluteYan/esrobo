"""Bounded passive motion evidence. No device reads and no producer-side I/O."""
import dataclasses
import hashlib
import json
import os
from pathlib import Path
import queue
import subprocess
import itertools
import math
import sys
import threading
import time

import numpy as np

PRIVATE = {'session', 'lease', 'key', 'password', 'secret', 'hmac', 'token',
           'entry_token', 'entry_plan_token', 'signature'}
_SCALARS = (float, int, str, bool, type(None))


def session_tag(value):
    return hashlib.sha256(str(value).encode()).hexdigest()[:20] if value else None


def owned(value):
    kind = type(value)
    if kind in _SCALARS:
        return value
    if kind in (list, tuple):
        if all(type(item) in _SCALARS for item in value):
            return list(value)
        return [owned(item) for item in value]
    if isinstance(value, np.ndarray):
        return value.copy()
    if isinstance(value, dict):
        return {k: owned(v) for k, v in value.items() if k not in PRIVATE}
    return value


def json_value(value):
    if isinstance(value, np.ndarray):
        return json_value(value.tolist())
    if isinstance(value, np.generic):
        return json_value(value.item())
    if isinstance(value, float) and not math.isfinite(value):
        return None  # Invalid raw observations remain identifiable by validity flags.
    if isinstance(value, dict):
        return {k: json_value(v) for k, v in value.items() if k not in PRIVATE}
    if isinstance(value, (list, tuple)):
        return [json_value(v) for v in value]
    if isinstance(value, Path):
        return str(value)
    return value


def metadata(root, **fields):
    """Startup-only provenance; never called from a control cycle."""
    root = Path(root)
    try:
        head = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=root,
                                       stderr=subprocess.DEVNULL, timeout=2).decode().strip()
    except (OSError, subprocess.SubprocessError):
        head = None
    hashes = {}
    unavailable = []
    for folder in ('laptop_teleop/src', 'robot_link/esrobo_link', 'teleoperation/src',
                   'laptop_teleop/config', 'teleoperation/config', 'teleoperation/urdf'):
        for path in sorted((root/folder).rglob('*')):
            if path.suffix in ('.py', '.yaml', '.urdf') and path.is_file():
                try:
                    hashes[str(path.relative_to(root))] = hashlib.sha256(path.read_bytes()).hexdigest()
                except OSError:
                    unavailable.append(str(path.relative_to(root)))
    try:
        boot = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    except OSError:
        boot = None
    return dict(schema_version=1, git_head=head, source_sha256=hashes, unavailable_files=unavailable, boot_id=boot,
                clocks='monotonic clocks are host-local; join hosts by session_tag and command_seq',
                **{k: dataclasses.asdict(v) if dataclasses.is_dataclass(v) else v for k,v in fields.items()})


class MotionTrace:
    def __init__(self, path, *, info=None, capacity=2048, interval=.2, enabled=None):
        self.path = Path(path)
        self.enabled = (os.environ.get('ESROBO_MOTION_TRACE', '1') != '0' if enabled is None else enabled)
        self.queue = queue.Queue(maxsize=capacity)
        self.interval = interval
        self.closed = threading.Event()
        self.attempted = self.enqueued = self.written = self.dropped = self.peak = 0
        self.bytes_written = self.serialization_errors = 0
        self.error = None
        self._sequences = itertools.count(1)
        self.shutdown_incomplete = False
        self.thread = None
        if self.enabled:
            self.thread = threading.Thread(target=self._run, name='motion-trace', daemon=True)
            self.thread.start()
            self.emit('metadata', **(info or {}))

    def emit(self, event, **fields):
        if not self.enabled:
            return False
        seq = next(self._sequences)
        self.attempted = max(self.attempted, seq)
        if self.closed.is_set() or self.error:
            self.dropped += 1
            return False
        try:
            row = dict(event=event, trace_seq=seq, monotonic_ns=time.monotonic_ns(),
                       unix_ns=time.time_ns(), **owned(fields))
            self.queue.put_nowait(row)
            self.enqueued += 1
            self.peak = max(self.peak, self.queue.qsize())
            return True
        except Exception:
            self.dropped += 1
            return False

    def stats(self):
        return dict(enabled=self.enabled, path=str(self.path), attempted=self.attempted,
                    enqueued=self.enqueued, written=self.written, dropped=self.dropped,
                    pending=self.queue.qsize(), queue_peak=self.peak, bytes_written=self.bytes_written,
                    serialization_errors=self.serialization_errors, error=self.error,
                    shutdown_incomplete=self.shutdown_incomplete)

    @staticmethod
    def _default(value):
        if isinstance(value, np.ndarray):
            return value.tolist()
        if isinstance(value, np.generic):
            return value.item()
        if isinstance(value, Path):
            return str(value)
        raise TypeError(f'unsupported diagnostic type: {type(value).__name__}')

    def _encode(self, row):
        try:
            try:
                # Common finite path stays in the C encoder. Only invalid raw
                # observations require the slower recursive normalization.
                return json.dumps(row, default=self._default, allow_nan=False, separators=(',', ':'))+'\n'
            except ValueError:
                return json.dumps(json_value(row), allow_nan=False, separators=(',', ':'))+'\n'
        except Exception as exc:
            self.serialization_errors += 1
            self.dropped += 1
            return json.dumps(dict(event='serialization_error', trace_seq=row.get('trace_seq'),
                                   error=str(exc)[:200]))+'\n'

    def _run(self):
        in_flight = 0
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            with self.path.open('x', encoding='utf-8') as stream:
                while True:
                    self.closed.wait(self.interval)
                    # A bounded batch also bounds temporary serialization memory.
                    batch = []
                    for _ in range(self.queue.maxsize):
                        try:batch.append(self.queue.get_nowait())
                        except queue.Empty:break
                    if batch:
                        in_flight = len(batch)
                        pieces = []
                        for index, row in enumerate(batch):
                            pieces.append(self._encode(row))
                            if index % 8 == 7:
                                # Limit consecutive GIL ownership while encoding
                                # a burst. Disk output remains one batched write.
                                time.sleep(0)
                        encoded = ''.join(pieces)
                        stream.write(encoded)
                        stream.flush()
                        self.bytes_written += len(encoded.encode())
                        self.written += len(batch)
                        in_flight = 0
                    if self.closed.is_set() and self.queue.empty():
                        stream.write(self._encode(dict(event='trace_closed', **self.stats())))
                        stream.flush()
                        break
        except Exception as exc:
            self.error = f'{type(exc).__name__}: {exc}'[:240]
            self.dropped += in_flight + self.queue.qsize()
            print(f"[motion-trace] write failed: {self.error}; diagnostics dropped", file=sys.stderr, flush=True)
            while True:
                try:self.queue.get_nowait()
                except queue.Empty:break

    def close(self, timeout=1.):
        self.closed.set()
        if self.thread:
            self.thread.join(max(0., min(1., timeout)))
            self.shutdown_incomplete = self.thread.is_alive()
        return not self.shutdown_incomplete and self.error is None
