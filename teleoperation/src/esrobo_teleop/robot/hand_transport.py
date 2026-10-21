"""Left L10 latest-target transport. No ROS, CAN initialization, or motion on import."""
from collections import deque
from pathlib import Path
import json
import math
import threading
import time

VERSION = 1
PREFIX = 'esrobo-hand-v1:'


def make_log(stage):
    from esrobo_teleop.debug.async_json_log import AsyncJsonLog
    root = Path(__file__).resolve().parents[4]
    return AsyncJsonLog(root / 'robot_link/log' / f'hand_{stage}_{time.time_ns()}.jsonl', capacity=2048)


def emit(log, event, **fields):
    if log is not None:
        try:
            log.submit(dict(event=event, monotonic_s=time.monotonic(), **fields))
        except Exception:
            pass  # Diagnostic failure must not alter the hardware decision.


def validate(meta, now):
    if not isinstance(meta, dict) or meta.get('version') != VERSION:
        raise ValueError('hand transport version required')
    if meta.get('action') not in ('target', 'stop', 'finish'):
        raise ValueError('invalid hand transport action')
    if not isinstance(meta.get('session'), str) or not 1 <= len(meta['session']) <= 64:
        raise ValueError('invalid hand transport session')
    if type(meta.get('seq')) is not int or meta['seq'] < 0:
        raise ValueError('invalid hand transport sequence')
    for key in ('issued_s', 'expires_s'):
        if type(meta.get(key)) not in (float, int) or not math.isfinite(meta[key]):
            raise ValueError('invalid hand transport clock')
    if not meta['issued_s'] <= now < meta['expires_s'] <= meta['issued_s'] + .201:
        raise ValueError('expired or future hand command')
    return meta


class LatestHandExecutor:
    """One CAN owner; callbacks replace one slot, never perform hardware I/O.

    send(position, guard) must check guard before *each* CAN position frame and
    raise on partial/failed output. snapshot returns an atomic two-frame sample.
    """
    def __init__(self, send, poll, snapshot, speed, *, log=None, clock=time.monotonic):
        self.send, self.poll, self.snapshot, self.speed = send, poll, snapshot, speed
        self.log, self.clock = log, clock
        self.lock = threading.RLock()
        self.pending = None
        self.settings = deque(maxlen=32)
        self.fault = None
        self.last_issued = -math.inf
        self.last_sequence = -1
        self.session = None
        self.last_command = None
        self.last_sent = None
        self.next_tick = 0.
        self.next_speed = 0.
        self.last_activity = -math.inf
        self.generation = 0
        self.closed = threading.Event()
        self.thread = None
        self.counts = dict(accepted=0, replaced=0, rejected=0, sent=0, expired=0)
        self.last_sent_id = None
        self.last_feedback_ids = None
        self.sending = False
        self.finish_requested = None
        self.finished_id = None
        self.speed_sample = None
        self.speed_stamp = None

    def accept(self, values, meta):
        now = self.clock()
        try:
            validate(meta, now)
            if meta['action'] == 'target':
                if len(values) != 10 or any(type(v) not in (int, float) or not math.isfinite(v)
                                          or not 0 <= v <= 255 for v in values):
                    raise ValueError('ten finite hand positions required')
                values = tuple(round(v) for v in values)
                for key, maximum in (('velocity', 900.), ('step', 18.)):
                    value = meta.get(key)
                    if type(value) not in (int, float) or not math.isfinite(value) or not 0 < value <= maximum:
                        raise ValueError('invalid hand motion limits')
            with self.lock:
                if self.closed.is_set() or self.fault:
                    raise ValueError('hand executor unavailable')
                if meta['issued_s'] < self.last_issued or (meta['session'] == self.session and meta['seq'] <= self.last_sequence):
                    raise ValueError('duplicate/out-of-order hand command')
                self.last_issued, self.session, self.last_sequence = meta['issued_s'], meta['session'], meta['seq']
                self.counts['accepted'] += 1
                self.last_activity = now
                if meta['action'] == 'stop':
                    self.generation += 1
                    self.pending = None
                    self.finish_requested = None
                    self.last_command = None
                    self.last_sent = None
                elif meta['action'] == 'finish':
                    # Normal completion clears queued goals but lets the
                    # already-started pair finish under its ORIGINAL deadline.
                    # Explicit stop/fault/close still cancels each frame.
                    self.pending = None
                    self.finish_requested = dict(session=meta['session'], seq=meta['seq'])
                    if not self.sending:
                        self.finished_id = self.finish_requested
                        self.finish_requested = None
                else:
                    self.counts['replaced'] += self.pending is not None
                    self.pending = (values, dict(meta), now, self.generation)
            emit(self.log, 'sdk_receive', meta=meta, received_s=now)
            return True
        except (ValueError, TypeError, KeyError) as exc:
            self.counts['rejected'] += 1
            emit(self.log, 'command_rejected', reason=str(exc))
            return False

    def setting(self, callback):
        with self.lock:
            if self.closed.is_set() or self.fault or len(self.settings) == self.settings.maxlen:
                return False
            self.settings.append(callback)
            return True

    def fail(self, exc):
        with self.lock:
            if self.fault is None:
                self.fault = str(exc)[:240]
            self.pending = None
            self.finish_requested = None
            self.sending = False
            self.settings.clear()
            self.generation += 1
        emit(self.log, 'executor_fault', reason=str(exc))

    def tick(self):
        now = self.clock()
        if self.closed.is_set() or now < self.next_tick:
            return
        # No catch-up iterations, including after a slow I/O operation.
        self.next_tick = now + .020
        with self.lock:
            pending = self.pending
        try:
            sample = self.snapshot()
            values, stamps, ids = sample['position'], sample['stamps'], sample['sequences']
            fresh = (len(values) == 10 and all(math.isfinite(v) and 0 <= v <= 255 for v in values)
                     and len(stamps) == 2 and all(v is not None and 0 <= now-v < .15 for v in stamps))
            if ids != self.last_feedback_ids:
                emit(self.log, 'can_feedback', position=values, frame_stamps=stamps, frame_sequences=ids)
                self.last_feedback_ids = list(ids)
            if not self.fault and pending:
                goal, meta, received, generation = pending
                if now >= meta['expires_s']:
                    with self.lock:
                        if self.pending is pending:
                            self.pending = None
                    self.counts['expired'] += 1
                    emit(self.log, 'target_expired', seq=meta['seq'], session=meta['session'])
                else:
                    if not fresh:
                        raise RuntimeError('physical hand feedback expired before CAN send')
                    if self.last_command is None:
                        self.last_command = tuple(round(v) for v in values)
                        self.last_sent = now
                    dt = min(.020, max(0., now-self.last_sent))
                    budget = min(int(meta['step']), math.floor(meta['velocity']*dt+1e-7))
                    candidate = tuple(a+max(-budget, min(budget, b-a)) for a,b in zip(self.last_command, goal))
                    def guard():
                        return (not self.closed.is_set() and not self.fault and
                                generation == self.generation and self.clock() < meta['expires_s'])
                    # A newly accepted, fresh target must reach the device even
                    # when its position is unchanged. Completion clears this
                    # slot, so no cached goal is replayed without new input.
                    if budget:
                        with self.lock:
                            # A finish received before this claim must prevent
                            # any output, including a locally cached candidate.
                            if self.pending is not pending or not guard():
                                return
                            self.sending = True
                        same_position = candidate == self.last_command
                        started = self.clock()
                        try:
                            self.send(candidate, guard)
                        except Exception as exc:
                            if (getattr(exc, 'completed_frames', None) == 0 and
                                    (generation != self.generation or self.closed.is_set())):
                                with self.lock:
                                    self.sending = False
                                emit(self.log, 'cancelled_before_can', meta=meta)
                                return
                            raise
                        completed = self.clock()
                        with self.lock:
                            if generation != self.generation or self.closed.is_set():
                                self.sending = False
                                emit(self.log, 'completed_before_stop', meta=meta, position=candidate)
                                return
                            if not guard():
                                raise RuntimeError('hand command cancelled/expired during CAN send')
                            self.last_command, self.last_sent = candidate, completed
                            self.last_sent_id = dict(session=meta['session'], seq=meta['seq'])
                            self.sending = False
                            if self.finish_requested is not None:
                                self.finished_id = self.finish_requested
                                self.finish_requested = None
                        self.counts['sent'] += 1
                        emit(self.log, 'can_command_sent', meta=meta, sdk_received_s=received,
                             started_s=started, completed_s=completed, position=candidate,
                             limited=candidate != goal, budget=budget,
                             same_position=same_position)
                        if candidate == goal:
                            with self.lock:
                                if self.pending is pending:
                                    self.pending = None
            # Feedback polling continues after faults; never resume position output.
            self.poll()
            if not self.fault and self.clock() >= self.next_speed:
                self.speed_sample = self.speed()
                # Query completion is not a measurement timestamp. A cached
                # register value remains as old as its actual CAN responses.
                speed_stamps = self.snapshot().get('speed_stamps', [None, None])
                self.speed_stamp = (min(speed_stamps) if len(speed_stamps) == 2
                                    and all(t is not None for t in speed_stamps) else None)
                self.next_speed = self.clock() + 1.
            setting = None
            with self.lock:
                if not self.fault and self.pending is None and now-self.last_activity >= .2 and self.settings:
                    setting = self.settings.popleft()
            if setting is not None:
                setting()
        except Exception as exc:
            self.fail(exc)

    def status(self):
        sample = self.snapshot()
        with self.lock:
            return dict(version=VERSION, side='left', position=sample['position'],
                        frame_stamps=sample['stamps'], frame_sequences=sample['sequences'],
                        fault=self.fault, sent=self.last_sent_id, counts=dict(self.counts),
                        finish_protocol=1, finished=self.finished_id,
                        speed=self.speed_sample, speed_stamp_s=self.speed_stamp)

    def start(self):
        def run():
            while not self.closed.is_set():
                self.tick()
                self.closed.wait(max(.001, self.next_tick-self.clock()))
        self.thread = threading.Thread(target=run, name='left-hand-can-owner', daemon=True)
        self.thread.start()

    def close(self):
        self.closed.set()
        with self.lock:
            self.pending = None
            self.settings.clear()
            self.generation += 1
        if self.thread:
            self.thread.join(timeout=1.)
        if self.log:
            self.log.close(timeout=1.)
