"""Initial approach only: deadline cadence and bounded background diagnostics."""
from collections import deque
from pathlib import Path
import json
import queue
import threading
import time



class EntryCadence:
    period = .02

    def __init__(self, cancel, clock=None):
        self.cancel, self.clock = cancel, clock or time.monotonic
        self.next_tick = self.clock()
        self.deadline = self.next_tick + self.period
        self._first = True
        self._overrun = False
        self._send_slot_reached = False
        self.overruns = 0
        self.skipped_ticks = 0

    def finish(self):
        # Work that did not reach a send slot still observes the same deadline.
        if not self._first and not self._send_slot_reached and self.clock() >= self.deadline:
            self._note_overrun()

    def _note_overrun(self):
        if not self._overrun:
            self.overruns += 1
            self._overrun = True

    def wait(self):
        now = self.clock()
        if self._first:
            self.next_tick = now
            self._first = False
        elif now > self.next_tick + 1e-9:
            import math
            # Skip a deadline missed by work; tolerate ordinary wake/send jitter
            # at the slot just completed without inserting another whole period.
            missed = (math.ceil((now-self.next_tick)/self.period-1e-9) if self._overrun
                      else int((now-self.next_tick)/self.period))
            self.next_tick += missed * self.period
            self.skipped_ticks += missed
        if self.cancel.wait(max(0., self.next_tick - self.clock())):
            return False
        now = self.clock()
        missed = max(0, int((now-self.next_tick)/self.period))
        self.skipped_ticks += missed
        if missed and not self._overrun:
            self.overruns += 1
        self.next_tick += missed * self.period
        self.deadline = self.next_tick + self.period
        self.next_tick = self.deadline
        self._overrun = False
        self._send_slot_reached = False
        return not self.cancel.is_set()

    def can_send(self):
        # Compute inside the period, send at its fixed deadline. Thus varying
        # 2/8/15 ms computation does not become varying CAN send intervals.
        remaining = self.deadline - self.clock()
        if remaining <= 0:
            self._note_overrun()
            return False
        if self.cancel.wait(remaining):
            return False
        if self.clock() >= self.deadline + self.period:
            self._note_overrun()  # Scheduler stall: discard, never replay.
            return False
        self._send_slot_reached = True
        return not self.cancel.is_set()


class EntryDiagnostics:
    """Execution thread copies finite samples; two workers own all I/O.

    File worker batches at 5 Hz. No worker touches FK, trajectory or geometry.
    Blocking disk/terminal has bounded queues and cannot delay motion cleanup.
    """
    def __init__(self, side, path=None):
        self.path = Path(path) if path else (Path(__file__).resolve().parents[3] / 'log'
            / f'entry_execution_{side}_{time.time_ns()}.jsonl')
        self._lock = threading.Lock()
        self._samples = deque(maxlen=256)
        self._console = queue.Queue(maxsize=64)
        self._closed = threading.Event()
        self._summary = dict(active=True, sample_count=0, dropped=0, console_dropped=0,
                             log_path=str(self.path), error=None)
        self._dropped = 0
        self._console_dropped = 0
        self._seq = 0
        self._file_thread = threading.Thread(target=self._write, daemon=True, name='entry-file')
        self._console_thread = threading.Thread(target=self._print, daemon=True, name='entry-console')
        self._file_thread.start()
        self._console_thread.start()

    def submit(self, sample):
        # Caller provides independent scalar/list fields only.
        with self._lock:
            self._seq += 1
            if len(self._samples) == self._samples.maxlen:
                self._dropped += 1
            self._samples.append(dict({k: list(v) if isinstance(v, list) else v
                                       for k, v in sample.items()}, diagnostic_seq=self._seq))

    def message(self, *args, **kwargs):
        try:
            self._console.put_nowait(' '.join(str(x) for x in args))
        except queue.Full:
            self._console_dropped += 1

    def snapshot(self):
        with self._lock:
            return dict(self._summary, active=not self._closed.is_set(),
                        pending=len(self._samples), dropped=self._dropped,
                        console_dropped=self._console_dropped)

    def close(self):
        self._closed.set()  # No join or file operations on the executor thread.

    def _print(self):
        while not self._closed.is_set() or not self._console.empty():
            try:
                text = self._console.get(timeout=.1)
            except queue.Empty:
                continue
            try:
                print(text, flush=True)
            except (OSError, ValueError) as exc:
                with self._lock:
                    self._summary['console_error'] = str(exc)

    def _write(self):
        history = deque(maxlen=200)
        count = 0
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            with self.path.open('x', encoding='utf-8') as stream:
                while True:
                    self._closed.wait(.2)
                    with self._lock:
                        samples = list(self._samples)
                        self._samples.clear()
                    history.extend(samples)
                    count += len(samples)
                    sends = [s for s in history if s.get('sent')]
                    gaps = [s['send_gap_ms'] for s in sends if s.get('send_gap_ms') is not None]
                    summary = dict(active=not self._closed.is_set(), sample_count=count,
                        generated_monotonic_s=(history[-1]['monotonic_s'] if history else None),
                        send_hz=(1000 * len(gaps) / sum(gaps) if gaps and sum(gaps)>0 else None),
                        cache_hits=sum(s.get('cache_hits', 0) for s in history),
                        overruns=(history[-1].get('overruns', 0) if history else 0),
                        skipped_ticks=(history[-1].get('skipped_ticks', 0) if history else 0))
                    with self._lock:
                        self._summary.update(summary)
                    if samples or self._closed.is_set():
                        stream.write(json.dumps(dict(summary=self.snapshot(), samples=samples),
                                                allow_nan=False) + '\n')
                        stream.flush()
                    if self._closed.is_set():
                        break
        except (OSError, TypeError, ValueError) as exc:
            with self._lock:
                self._summary.update(error=str(exc), active=False)
            # Continue bounded retention, no synchronous fallback.
