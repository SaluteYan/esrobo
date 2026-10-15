"""Passive torque evidence. Uses protection samples; never reads SDK or sends CAN."""
from collections import deque
import copy
import math
import threading
import time


def finite(value):
    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, dict):
        return {k: finite(v) for k, v in value.items()}
    if hasattr(value, 'tolist'):
        return finite(value.tolist())
    if isinstance(value, (list, tuple)):
        return [finite(v) for v in value]
    return float(value) if math.isfinite(float(value)) else None


class TorqueRecorder:
    def __init__(self, session, side, writer, mode):
        self.session, self.side, self.writer, self.mode = session, side, writer, mode
        self.lock = threading.Lock()
        self.sequence = self.baseline_id = 0
        self.latest = self.fault = None
        self.history = deque(maxlen=32)
        self.previous_feedback = None
        self.error = None

    def begin_baseline(self):
        with self.lock:
            self.baseline_id += 1
            self.history.clear()
            self.previous_feedback = None

    def record(self, *, current, baseline, limits, counts, metadata, motion,
               phase=None, kind='sample', verdict=None, fault=False):
        now, wall = time.monotonic(), time.time()
        metadata = metadata if isinstance(metadata, dict) else {}
        stamps = metadata.get('axis_stamp_s') or [None]*7
        motion = finite(motion)
        with self.lock:
            self.sequence += 1
            q, stamp = motion.get('feedback_physical_rad'), motion.get('feedback_stamp_s')
            velocity = None
            prior = self.previous_feedback
            if q is not None and stamp is not None:
                if prior is not None and 0 < stamp-prior[1] <= .1:
                    velocity = [(a-b)/(stamp-prior[1]) for a,b in zip(q,prior[0])]
                if prior is None or stamp > prior[1]:
                    self.previous_feedback = (list(q), stamp)
            motion.update(feedback_velocity_rad_s=velocity,
                          feedback_velocity_stamp_s=stamp if velocity is not None else None)
            command = motion.get('command_physical_rad')
            motion['command_feedback_error_rad'] = (
                [a-b for a,b in zip(command,q)] if command is not None and q is not None else None)
            values = finite(current)
            base = finite(baseline)
            row = finite(dict(record_type=kind, session=self.session, side=self.side,
                sequence=self.sequence, baseline_id=self.baseline_id,
                unix_ns=time.time_ns(), monotonic_s=now, gateway_mode=self.mode(),
                phase=phase or self.mode(), valid=current is not None,
                read_status=metadata.get('status','metadata_unavailable'),
                axis_stamp_s=stamps, axis_age_s=[wall-s if s is not None else None for s in stamps],
                torque_nm=values, baseline_nm=base,
                deviation_nm=([abs(a-b) for a,b in zip(values,base)]
                              if values is not None and base is not None else None),
                limits_nm=limits, trip_counts=counts, protection_passed=verdict,
                logging_dropped=self.writer.dropped, motion=motion))
            self.latest = row
            if current is not None and kind == 'sample':
                self.history.append(row)
            if fault and self.fault is None:
                self.fault = dict(trigger=copy.deepcopy(row), samples=copy.deepcopy(list(self.history)))
            # An immutable copy is queued; serialization and disk I/O run elsewhere.
            self.writer.submit(row)

    def health(self):
        return dict(path=str(self.writer.path), dropped=self.writer.dropped,
                    error=self.error or self.writer.error,
                    serialization_errors=self.writer.serialization_errors)

    def snapshot(self):
        with self.lock:
            latest = copy.deepcopy(self.latest)
        return dict(session=self.session, side=self.side, latest=latest,
                    sample_age_s=None if latest is None else max(0.,time.monotonic()-latest['monotonic_s']),
                    logging=self.health())

    def fault_snapshot(self):
        with self.lock:
            return copy.deepcopy(self.fault)
