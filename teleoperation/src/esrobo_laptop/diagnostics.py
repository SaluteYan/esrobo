"""Bounded diagnostics; only background workers serialize or perform output I/O."""
from collections import deque
import copy
import json
from pathlib import Path
import sys
import threading
import time

import numpy as np


def _json_default(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(f'Unsupported diagnostic type: {type(value).__name__}')


def _redact(value):
    private = {'session', 'lease', 'key', 'secret', 'password', 'hmac', 'token', 'signature',
               'entry_token', 'entry_plan_token'}
    if isinstance(value, dict):
        return {k: _redact(v) for k, v in value.items() if k not in private}
    if isinstance(value, (list, tuple)):
        return [_redact(v) for v in value]
    return value


class _Mailbox:
    """One coalescing periodic slot, 64 events, and a reserved terminal fault."""
    def __init__(self):
        self.condition = threading.Condition()
        self.events = deque()
        self.latest = self.fault = None
        self.closed = False
        self.coalesced = self.dropped = 0

    def put(self, item, event=False, fault=False):
        with self.condition:
            if self.closed:
                self.dropped += 1
                return False
            if fault:
                self.fault = item
            elif event:
                if len(self.events) == 64:
                    self.events.popleft()
                    self.dropped += 1
                self.events.append(item)
            else:
                if self.latest is not None:
                    self.coalesced += 1
                self.latest = item
            self.condition.notify()
            return True

    def get(self):
        with self.condition:
            while not (self.events or self.latest is not None or self.fault is not None or self.closed):
                self.condition.wait()
            if self.closed and self.fault is not None:
                item, self.fault = self.fault, None
                return item
            candidates = list(self.events)[:1] + [x for x in (self.latest, self.fault) if x is not None]
            if not candidates:
                return None
            item = min(candidates, key=lambda x: x['seq'])
            if self.events and item is self.events[0]:
                self.events.popleft()
            elif item is self.latest:
                self.latest = None
            else:
                self.fault = None
            return item

    def stats(self):
        with self.condition:
            return dict(pending=len(self.events)+int(self.latest is not None)+int(self.fault is not None),
                        coalesced=self.coalesced, dropped=self.dropped)

    def close(self):
        with self.condition:
            self.closed = True
            self.condition.notify_all()


class RuntimeDiagnostics:
    def __init__(self, log_path, status_path=None, *, stdout=None, stderr=None):
        self.path = Path(log_path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        # Failure is reported before a control loop starts; never overwrite an old session.
        self._stream = self.path.open('x', encoding='utf-8')
        self.status_path = Path(status_path) if status_path else None
        try:
            if self.status_path:
                self.status_path.parent.mkdir(parents=True, exist_ok=True)
        except OSError:
            self._stream.close()
            raise
        self.stdout, self.stderr = stdout or sys.stdout, stderr or sys.stderr
        self.files, self.console = _Mailbox(), _Mailbox()
        self._sequence_lock = threading.Lock()
        self._sequence = 0
        self._status_sequence = 0
        self.errors = dict(journal=None, status=None, reference=None, console=None)
        self.serialization_errors = 0
        self._threads = [threading.Thread(target=self._write_files, name='laptop-log', daemon=True),
                         threading.Thread(target=self._write_console, name='laptop-console', daemon=True)]
        for thread in self._threads:
            thread.start()

    def _seq(self):
        with self._sequence_lock:
            self._sequence += 1
            return self._sequence

    def submit(self, record, *, event=False, fault=False):
        # Copy producer-owned arrays/containers. No live controller or solver crosses threads.
        record = copy.deepcopy(record)
        seq = self._seq()
        record['diagnostic_seq'] = seq
        return self.files.put(dict(seq=seq, kind='snapshot', record=record), event, fault)

    def reference(self, path, record):
        seq = self._seq()
        record = copy.deepcopy(record)
        record['diagnostic_seq'] = seq
        return self.files.put(dict(seq=seq, kind='reference', path=Path(path),
                                   record=record), event=True)

    def message(self, *values, sep=' ', end='\n', flush=False, file=None, event=True, fault=False):
        seq = self._seq()
        return self.console.put(dict(seq=seq, text=sep.join(map(str, values))+end,
                                     stderr=file is sys.stderr), event, fault)

    def stats(self):
        return dict(files=self.files.stats(), console=self.console.stats(), errors=dict(self.errors),
                    serialization_errors=self.serialization_errors)

    def _write_files(self):
        try:
            while (item := self.files.get()) is not None:
                record = item['record']
                try:
                    if item['kind'] == 'snapshot':
                        record['diagnostics'] = self.stats()
                    encoded = json.dumps(_redact(record), allow_nan=False, default=_json_default)
                except (TypeError, ValueError) as exc:
                    self.serialization_errors += 1
                    self.errors['serialization'] = str(exc)
                    continue
                if item['kind'] == 'reference':
                    if self.errors['reference'] is None:
                        try:
                            item['path'].parent.mkdir(parents=True, exist_ok=True)
                            with item['path'].open('a', encoding='utf-8') as stream:
                                stream.write(encoded+'\n')
                                stream.flush()
                        except OSError as exc:
                            self.errors['reference'] = str(exc)
                else:
                    if self.errors['journal'] is None:
                        try:
                            self._stream.write(encoded+'\n')
                            self._stream.flush()
                        except OSError as exc:
                            self.errors['journal'] = str(exc)
                    if (self.status_path and 'record_type' not in record
                            and item['seq'] > self._status_sequence and self.errors['status'] is None):
                        try:
                            temporary = self.status_path.with_suffix('.tmp')
                            temporary.write_text(encoded, encoding='utf-8')
                            temporary.replace(self.status_path)
                            self._status_sequence = item['seq']
                        except OSError as exc:
                            self.errors['status'] = str(exc)
        finally:
            try:
                self._stream.close()
            except OSError as exc:
                self.errors['journal'] = str(exc)

    def _write_console(self):
        while (item := self.console.get()) is not None:
            if self.errors['console'] is None:
                try:
                    stream = self.stderr if item['stderr'] else self.stdout
                    stream.write(item['text'])
                    stream.flush()
                except (OSError, ValueError) as exc:
                    self.errors['console'] = str(exc)

    def close(self, timeout=1.):
        self.files.close()
        self.console.close()
        deadline = time.monotonic()+min(1., max(0., timeout))
        for thread in self._threads:
            thread.join(max(0., deadline-time.monotonic()))
        return not any(t.is_alive() for t in self._threads)
