"""Independent network watchdog and serialized robot I/O worker."""
import argparse
import json
import socket
import secrets
import math
import copy
import threading
import time

from .backends import MockBackend, MockDualBackend
from .protocol import MAX_PACKET, ProtocolError, PacketTooLarge, InputHoldTransition, HoldReturnTransition, key_from_file, pack, unpack
from .session import SessionGate
from .state_summary import compact_state, return_failure, short_text


class Gateway:
    def __init__(self, backend, key, bind="127.0.0.1", port=16000, max_age=0.2, journal=None):
        if not .05 <= max_age <= .5:
            raise ValueError("max target age must be between 0.05 and 0.5 s")
        self.backend, self.key = backend, key
        self.gate = SessionGate(backend.contract, max_age)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((bind, port))
        self.socket.settimeout(.005)
        self.address = self.socket.getsockname()
        self.lock = threading.RLock()
        self.cancelled = threading.Event()
        self.shutdown = threading.Event()
        self.network_done = threading.Event()
        self.mode = "IDLE"
        self.reason = "local e required; no automatic enable"
        self.first_fault_reason = None
        self.first_fault_summary = None
        self._fault_event_pending = None
        self.pending = None
        self.stop_pending = False
        self.stop_send_returned = None
        self.disable_verified = None
        self.feedback = {}
        self.feedback_time = None
        self.rejected_packets = 0
        self.last_rejection = ""
        self.last_return = None
        self.journal = journal
        self.motion_trace = None
        self._trace_target_context = None
        self._diagnostic_events = []
        self.journal_error = None
        self.diagnostic_events_dropped = 0
        self._state_encoding_status = None
        self._last_record = None
        self._record_time = 0.0
        self.entry_target = None
        self.entry_token = ""
        self.entry_plan_token = ""
        self.entry_candidate_since = None
        self.entry_ready_deadline = None
        self.operator_result = dict(seq=0, action="", accepted=False, message="")
        self._last_state_send_at = None
        self.state_send_gap_s = None
        self.state_send_gap_max_s = 0.0
        self.state_send_gaps_over_80ms = 0

    def diagnostic_event(self, kind, **details):
        if isinstance(details.get('state'), dict):
            details['state'] = {k:v for k,v in details['state'].items()
                                if k not in ('lease', 'session', 'client_nonce')}
        with self.lock:
            # Diagnostic failure must not turn into unbounded memory growth.
            if len(self._diagnostic_events) >= 256:
                del self._diagnostic_events[0]
                self.diagnostic_events_dropped += 1
            self._diagnostic_events.append(dict(record_type=kind, unix_ns=time.time_ns(),
                                                monotonic_s=time.monotonic(), **copy.deepcopy(details)))
        print(f"[gateway] {kind}: {short_text(details.get('reason', details.get('stage', 'recorded')))}", flush=True)

    def record_return(self, result):
        self.diagnostic_event('return_result', result=result,
                              diagnostics=getattr(self.backend, 'return_diagnostics', lambda: None)(),
                              stage=result.get('stage'), reason=result.get('reason', ''))

    def encode_state(self, message):
        try:
            packet = pack(message, self.key)
            self._state_encoding_status = None
            return packet
        except PacketTooLarge:
            # Reuse the original state sequence and lease on every attempt.
            reduced = compact_state(message)
            try:
                packet = pack(reduced, self.key)
            except PacketTooLarge as exc:
                if self._state_encoding_status != 'core_oversize':
                    self.diagnostic_event('network_state_error', reason=str(exc),
                                          action='core_state_rejected', state=message)
                    self._state_encoding_status = 'core_oversize'
                    self.input_fault('required state exceeds packet bound; telemetry unavailable')
                return None
            if self._state_encoding_status != 'compacted':
                self.diagnostic_event('network_state_error', reason='optional diagnostics exceeded packet bound',
                                      action='diagnostics_compacted', state=message)
            self._state_encoding_status = 'compacted'
            return packet

    def record_operator_result(self, action, accepted, message=""):
        with self.lock:
            self.operator_result = dict(seq=self.operator_result["seq"] + 1,
                                        action=action, accepted=bool(accepted),
                                        message=str(message)[:240])

    @staticmethod
    def fault_summary(diagnostics):
        """Small, persistent status view of the exact frame kept in the journal."""
        if not isinstance(diagnostics, dict):
            return None
        if "fault_context" not in diagnostics:
            for side in ("left", "right"):
                summary = Gateway.fault_summary(diagnostics.get(side))
                if summary:
                    return dict(side=side, **summary)
            return None
        context = diagnostics["fault_context"]
        if not isinstance(context, dict):
            return None
        joints = context.get("failed_joints") or []
        summary = dict(fault=diagnostics.get("fault"),
                       failed_joints=joints, feedback_dt_s=context.get("feedback_dt_s"))
        if joints and isinstance(joints[0], int):
            index = joints[0] - 1
            for name in ("target_physical_rad", "previous_command_physical_rad",
                         "feedback_physical_rad", "previous_velocity_physical_rad_s",
                         "feasible_velocity_lower_rad_s", "feasible_velocity_upper_rad_s",
                         "lead_rad", "lead_limit_rad"):
                values = context.get(name)
                if isinstance(values, list) and 0 <= index < len(values):
                    summary[name] = values[index]
        return summary

    def entry_execution_diagnostics(self):
        getter = getattr(self.backend, 'entry_execution_diagnostics', None)
        return getter() if callable(getter) else None

    def record(self):
        """5 Hz diagnostic snapshots plus state changes; never log key/lease."""
        if self.journal is None:
            return
        now = time.monotonic()
        with self.lock:
            identity = (self.mode, self.reason, self.stop_send_returned,
                        self.disable_verified, self.operator_result["seq"], self.first_fault_reason)
            fault_event = self._fault_event_pending
            events = list(self._diagnostic_events)
            if not events and fault_event is None and identity == self._last_record and now - self._record_time < .2:
                return
            entry = dict(unix_ns=time.time_ns(), monotonic_s=now, mode=self.mode,
                         reason=self.reason, contract_id=self.backend.contract["id"],
                         journal_path=getattr(self.journal, "name", None),
                         first_fault_reason=self.first_fault_reason,
                         first_fault_summary=self.first_fault_summary,
                         accepted_seq=self.gate.sequence, target=self.gate.target,
                         entry_plan_target=self.entry_target,
                         target_remaining_s=max(0.0, self.gate.deadline-now),
                         feedback=self.feedback, stop_send_returned=self.stop_send_returned,
                         disable_verified=self.disable_verified, recovery_supported=True,
                         return_protocol=self.backend.contract.get("return_protocol"),
                         hold_return_protocol=self.backend.contract.get("hold_return_protocol"),
                         feedback_cache_age_s=None if self.feedback_time is None else now-self.feedback_time,
                         diagnostics_truncated=self._state_encoding_status == "compacted",
                         state_send_gap_s=self.state_send_gap_s,
                         state_send_gap_max_s=self.state_send_gap_max_s,
                         state_send_gaps_over_80ms=self.state_send_gaps_over_80ms,
                         state_send_idle_s=(None if self._last_state_send_at is None
                                            else now-self._last_state_send_at),
                         rejected_packets=self.rejected_packets, last_rejection=self.last_rejection,
                         last_return=self.last_return, entry_token=self.entry_token,
                         entry_protocol=self.backend.contract.get("entry_protocol"),
                         input_hold_protocol=self.backend.contract.get("input_hold_protocol"),
                         input_hold_age_s=(None if self.gate.hold_started_at is None else
                                           now-self.gate.hold_started_at),
                         hold_reason=(self.gate.target.get("hold_reason", "input")
                                      if self.gate.target and self.gate.target.get("hold") else None),
                         position_hold_age_s=(None if self.gate.position_hold_started_at is None else
                                              now-self.gate.position_hold_started_at),
                         operator_result=self.operator_result,
                         entry_ready=(self.mode == "CALIBRATING" and self.entry_candidate_since is not None
                                      and now-self.entry_candidate_since >= .5 and self.gate.fresh(now)
                                      and self.entry_matches(self.gate.target, distance=.01, joint_limit_deg=.5)))
            entry.update(self.entry_liveness(now))
            entry.update(self.recovery_status())
            entry["entry_execution"] = self.entry_execution_diagnostics()
            if self.motion_trace is not None:
                entry["motion_trace"] = self.motion_trace.stats()
        for event in events:
            self.journal.write(json.dumps(event, allow_nan=False) + "\n")
        if fault_event is not None:
            self.journal.write(json.dumps(fault_event, allow_nan=False) + "\n")
        self.journal.write(json.dumps(entry, allow_nan=False) + "\n")
        self.journal.flush()
        with self.lock:
            written = {id(event) for event in events}
            self._diagnostic_events[:] = [event for event in self._diagnostic_events
                                          if id(event) not in written]
            if self._fault_event_pending is fault_event:
                self._fault_event_pending = None
            self._last_record, self._record_time = identity, now

    def trip(self, reason, *, verified_disabled=False):
        with self.lock:
            self.entry_plan_token = ""
            self.cancelled.set()
            intentional_stop = reason.startswith(("operator requested ", "laptop requested stop"))
            if self.mode != "FAULT":
                self.stop_pending = self.stop_pending or (not verified_disabled and self.mode in (
                    "ACTIVE", "HOLD_LATCHED", "ARMING", "RETURNING", "APPROACHING", "WAITING_FOR_ACTIVE_TARGET"))
                self.stop_send_returned = None
                self.disable_verified = None
            if not intentional_stop and self.first_fault_reason is None:
                try:
                    diagnostics = copy.deepcopy(getattr(self.backend, "fault_diagnostics", lambda: None)())
                except Exception as exc:
                    diagnostics = {"capture_error": f"{type(exc).__name__}: {exc}"}
                self.first_fault_reason = reason
                self.first_fault_summary = self.fault_summary(diagnostics)
                self._fault_event_pending = dict(
                    record_type="first_fault", unix_ns=time.time_ns(),
                    monotonic_s=time.monotonic(), mode_before=self.mode,
                    reason=reason, contract_id=self.backend.contract["id"],
                    accepted_seq=self.gate.sequence,
                    target=copy.deepcopy(self.gate.target),
                    feedback=copy.deepcopy(self.feedback),
                    diagnostics=diagnostics)
            if self.mode != "FAULT" or (not intentional_stop and self.first_fault_reason == reason):
                self.reason = reason
            self.mode = "FAULT"
            if self.pending not in ("d", "dl", "dr", "dh"):
                self.pending = None
        self.backend.cancel()  # Only sets an Event; never performs network/CAN I/O.

    def input_fault(self, reason):
        """Latch robot-owned braking without cancelling its trajectory worker.

        Only commissioned endpoints may enter this state. All hardware errors
        still go through trip(), including any failure while maintaining hold.
        """
        with self.lock:
            if self.mode == 'HOLD_LATCHED':
                return
            if (self.mode not in ('ACTIVE', 'WAITING_FOR_ACTIVE_TARGET')
                    or self.backend.contract.get('input_recovery_enabled') is not True):
                self.trip(reason)
                return
            self.mode, self.reason = 'HOLD_LATCHED', reason
            self.entry_plan_token = ''
            self.entry_ready_deadline = None
            self.pending = None

    def accept_checked_hold(self, result):
        """Only accept an explicit backend proof, never infer hold from text."""
        if (result.get('holding') is not True or self.cancelled.is_set()
                or self.backend.contract.get('input_recovery_enabled') is not True):
            return False
        self.mode = 'HOLD_LATCHED'
        self.reason = return_failure('已停止并保持；受检运动未完成，需显式恢复', result)
        self.entry_plan_token = ''
        self.entry_ready_deadline = None
        self.pending = None
        return True

    def recovery_status(self):
        started = self.gate.recovery_started_at
        phase = ('HOLD_LATCHED' if self.mode == 'HOLD_LATCHED' else
                 'REACQUIRING' if started is not None else
                 'INPUT_WAIT' if self.gate.hold_started_at is not None else self.mode)
        return dict(input_recovery_protocol=self.backend.contract.get('input_recovery_protocol'),
                    journal_error=self.journal_error,
                    diagnostic_events_dropped=self.diagnostic_events_dropped,
                    input_recovery_enabled=self.backend.contract.get('input_recovery_enabled', False),
                    controlled_stop_protocol=1,
                    controlled_stop_enabled=self.backend.contract.get('input_recovery_enabled') is True,
                    recovery_phase=phase,
                    reacquisition_age_s=None if started is None else time.monotonic()-started)

    def watchdog(self):
        with self.lock:
            if (self.mode == "ACTIVE" and self.gate.hold_started_at is not None
                    and time.monotonic()-self.gate.hold_started_at > .5):
                self.input_fault("input hold exceeded 500 ms; explicit local recovery required")
            if (self.mode == 'ACTIVE' and self.gate.recovery_started_at is not None
                    and time.monotonic()-self.gate.recovery_started_at >= 3.):
                self.input_fault('input reacquisition exceeded 3 s; explicit local recovery required')
            if (self.mode == "CALIBRATING" and self.backend.contract.get("entry_protocol") in (1, 2, 3, 4, 5, 6, 7)
                    and (not self.gate.fresh(time.monotonic())
                         or self.gate.target.get("entry", {}).get("phase") != "candidate")):
                if self.backend.contract.get("entry_protocol") == 1:
                    self.entry_target = None
                self.entry_candidate_since = None
            if self.mode in ("ACTIVE", "ARMING", "CHECKING_ENTRY", "APPROACHING", "WAITING_FOR_ACTIVE_TARGET") and not self.gate.fresh(time.monotonic()):
                self.input_fault("joint target lease expired; explicit local recovery required")
            if self.mode in ("CHECKING_ENTRY", "APPROACHING", "WAITING_FOR_ACTIVE_TARGET"):
                if not self.entry_matches(self.gate.target):
                    self.input_fault("handover target/reference changed or input expired")
                elif (self.entry_ready_deadline is not None
                      and time.monotonic() > self.entry_ready_deadline):
                    self.input_fault("fresh ACTIVE target acknowledgment timed out")

    def entry_matches(self, target, *, distance=.02, joint_limit_deg=2.):
        reference = self.entry_target
        if reference is None or target is None:
            return False
        entry = target.get("entry", {})
        if entry.get("phase") not in ("candidate", "ready"):
            return False
        source = entry.get("source")
        if not isinstance(source, list) or len(source) != 9:
            return False
        protocol = self.backend.contract.get("entry_protocol")
        baseline = (reference["entry"]["reference_source"]
                    if protocol in (2, 3, 4, 5, 6, 7)
                    else reference["entry"]["source"])
        if protocol in (2, 3, 4, 5, 6, 7):
            supplied = entry.get("reference_source")
            if supplied is None or max(abs(a-b) for a,b in zip(supplied, baseline)) > 1.0e-6:
                return False
        if protocol not in (3, 4, 5, 6, 7):
            for offset in (0, 3, 6):
                if math.dist(source[offset:offset+3], baseline[offset:offset+3]) > distance:
                    return False
        if max(abs(a-b) for a,b in zip(reference["arm_urdf_rad"], target["arm_urdf_rad"])) > math.radians(joint_limit_deg):
            return False
        if reference.get("hand_unit") is not None:
            if max(abs(a-b) for a,b in zip(reference["hand_unit"], target["hand_unit"])) > 5:
                return False
        return True

    def start_approach(self):
        with self.lock:
            if self.motion_cancelled():
                raise RuntimeError("handover cancelled after planning")
            self.mode = "APPROACHING"
            self.reason = "完整路径已认证；正在慢速移动至标定目标，保持 PICO 追踪，抵达后再跟随实时姿态"

    def request(self, action):
        """Operator commands; h latches commissioned hold, s/x remain E-stop."""
        if action in ("x", "s", "q"):
            self.trip("operator requested stop")
            # Even idle x must send a stop (controller may have external state).
            with self.lock:
                self.stop_pending = True
            if action == "q":
                self.shutdown.set()
            return
        with self.lock:
            if action == 'h':
                if (self.backend.contract.get('input_recovery_enabled') is not True
                        or self.mode not in ('ACTIVE', 'HOLD_LATCHED')
                        or self.pending or self.stop_pending or self.cancelled.is_set()):
                    raise RuntimeError('controlled stop requires commissioned ACTIVE/holding gateway; no reset or enable sent')
                self.input_fault('operator controlled stop; position hold latched; explicit recovery required')
                return
            held_return = action in ('r', 'z') and self.mode == 'HOLD_LATCHED' and not getattr(self.backend, 'hand_only', False)
            if held_return:
                fb = self.feedback
                age = fb.get('arm_feedback_age_s')
                if (self.backend.contract.get('hold_return_protocol') != 1
                        or fb.get('enable_states') != [True]*7
                        or fb.get('controller_fault') or fb.get('driver_fault')
                        or not isinstance(age, (int, float)) or not 0 <= age <= .1
                        or self.feedback_time is None
                        or not 0 <= time.monotonic()-self.feedback_time <= .1):
                    raise RuntimeError('healthy fresh enabled feedback and hold-return protocol required; latched hold retained')
            if action == "d":
                self.trip("operator requested disable")
                self.disable_verified = None
                self.pending = "d"
            elif action in ("dl", "dr", "dh"):
                label = {"dl": "left arm", "dr": "right arm", "dh": "hands"}[action]
                self.trip(f"operator requested {label} disable")
                self.pending = action
            elif action == "e":
                if not getattr(self.backend, "supports_prepare", True):
                    raise RuntimeError("automatic session preparation is unavailable without an inter-arm return checker")
                if self.mode != "IDLE" or self.pending or not self.gate.fresh(time.monotonic()):
                    raise RuntimeError("e requires IDLE and a fresh input/feedback stream")
                self.cancelled.clear()
                self.entry_target, self.entry_candidate_since = None, None
                self.entry_token, self.entry_ready_deadline = "", None
                self.entry_plan_token = ""
                self.gate.last_entry_keepalive_at = None
                self.pending, self.mode = "p", "RETURNING"
                self.reason = "verifying hand open zero before checked arm return"
            elif action == "g":
                if (self.backend.contract.get("entry_protocol") not in (1, 2, 3, 4, 5, 6, 7)
                        or self.mode != "CALIBRATING" or self.pending
                        or not self.gate.fresh(time.monotonic())
                        or not self.entry_matches(self.gate.target, distance=.01, joint_limit_deg=.5)
                        or self.entry_candidate_since is None
                        or time.monotonic()-self.entry_candidate_since < .5):
                    raise RuntimeError("检查并接管需要标定完成、持续新鲜且稳定至少 0.5 秒的完整目标")
                if self.backend.contract.get("entry_protocol") not in (3, 4, 5, 6, 7):
                    self.entry_target = copy.deepcopy(self.gate.target)
                self.pending, self.mode = "h", "CHECKING_ENTRY"
                self.entry_plan_token = secrets.token_hex(16)
                self.gate.last_entry_keepalive_at = None
                self.entry_token = ""
                self.reason = "正在检查标定目标并规划外展路径；机器人仍失能"
            elif action == "z":
                if not getattr(self.backend, "supports_return", True):
                    raise RuntimeError("z unavailable in hand-only mode or without an inter-arm return checker; no motion sent")
                if self.mode not in ("IDLE", "FAULT", "HOLD_LATCHED") or self.pending or self.stop_pending:
                    raise RuntimeError("stop first and wait for stop handling before z")
                self.cancelled.clear()
                self.pending, self.mode = ("z_hold" if held_return else "z"), "RETURNING"
            elif action == "r":
                inherited_fault = (self.mode == "IDLE" and
                                   bool(self.feedback.get("controller_fault") or
                                        self.feedback.get("driver_fault")))
                if ((self.mode not in ("FAULT", "HOLD_LATCHED") and not inherited_fault)
                        or self.pending or self.stop_pending or self.stop_send_returned is False):
                    raise RuntimeError("recovery requires a stopped FAULT gateway")
                if getattr(self.backend, "hand_only", False):
                    self.cancelled.clear()
                    self.pending, self.mode = "r", "RECOVERING"
                    self.reason = "verifying disabled arm and hand feedback without motion"
                elif getattr(self.backend, "supports_return", True):
                    self.cancelled.clear()
                    self.pending, self.mode = ("z_hold" if held_return else "z"), "RETURNING"
                    self.reason = "explicit single-arm recovery; verified return to zero"
                else:
                    raise RuntimeError("dual-arm recovery requires the supervised inter-arm procedure; no motion sent")
            else:
                raise ValueError("keys: e prepare, g check+approach, h hold, s/x E-stop, d all, dl left, dr right, dh hands, r recover, z return, q exit")

    def accept_command(self, message, peer, now):
        """Called under lock; a delayed keepalive cannot cross into ACTIVE."""
        if self.mode == 'HOLD_LATCHED' and message.get('type') not in ('stop', 'close', 'input_fault'):
            raise ProtocolError('latched hold requires explicit local recovery; target rejected')
        if message.get("type") == "hold" and self.mode != "ACTIVE":
            raise ProtocolError("hold requires an active arm session")
        token = (self.entry_plan_token if self.mode in
                 ("CHECKING_ENTRY", "APPROACHING", "WAITING_FOR_ACTIVE_TARGET") else None)
        kind = self.gate.accept(message, peer, now, entry_plan_token=token)
        if self.motion_trace is not None:
            from .motion_trace import command_context
            context = command_context(message, now)
            if kind in ('target', 'hold'):
                self._trace_target_context = context
            self.motion_trace.emit('network_accept', context=context)
        return kind

    def entry_liveness(self, now):
        last = self.gate.last_entry_keepalive_at
        return dict(entry_keepalive_protocol=self.backend.contract.get("entry_keepalive_protocol"),
                    entry_plan_token=self.entry_plan_token,
                    entry_keepalive_age_s=None if last is None else max(0., now-last))

    def network_loop(self):
        next_state = 0.0
        try:
            while not self.network_done.is_set():
                self.watchdog()
                try:
                    packet, peer = self.socket.recvfrom(MAX_PACKET + 1)
                    message = unpack(packet, self.key)
                    with self.lock:
                        self.watchdog()  # A late packet cannot revive an expired stream.
                        now = time.monotonic()
                        if message.get("type") == "hello":
                            self.gate.hello(message, peer, now,
                                            self.mode in ("ACTIVE", "HOLD_LATCHED", "ARMING", "RETURNING", "RECOVERING", "CALIBRATING", "CHECKING_ENTRY", "APPROACHING", "WAITING_FOR_ACTIVE_TARGET"))
                        else:
                            kind = self.accept_command(message, peer, now)
                            if (kind == "target" and self.backend.contract.get("entry_protocol") in (1, 2, 3, 4, 5, 6, 7)
                                    and self.mode == "ACTIVE"):
                                entry = self.gate.target.get("entry", {})
                                if entry.get("phase") not in ("ready", "active") or entry.get("token") != self.entry_token:
                                    self.input_fault("invalid ACTIVE handover token/phase")
                            if kind == "stop":
                                self.trip("laptop requested stop")
                                self.stop_pending = True
                            elif kind in ('close', 'input_fault'):
                                self.input_fault('laptop input fault/session closed; explicit local recovery required')
                except socket.timeout:
                    pass
                except (ProtocolError, ValueError, TypeError) as exc:
                    with self.lock:
                        self.rejected_packets += 1
                        self.last_rejection = str(exc)[:180]
                self.watchdog()
                now = time.monotonic()
                if now >= next_state:
                    # Keep the 50 Hz state stream on its original clock. Reset
                    # only after a full missed period; rebasing on every send
                    # accumulates socket/scheduler latency and slows the laptop
                    # control loop, which waits for each fresh state packet.
                    next_state += .02
                    if next_state <= now:
                        next_state = now + .02
                    with self.lock:
                        if self.gate.peer is None:
                            continue
                        status = dict(mode=self.mode, reason=self.reason,
                                      feedback=self.feedback,
                                      feedback_cache_age_s=None if self.feedback_time is None else now - self.feedback_time,
                                      state_send_gap_s=self.state_send_gap_s,
                                      state_send_gap_max_s=self.state_send_gap_max_s,
                                      stop_send_returned=self.stop_send_returned,
                                      disable_verified=self.disable_verified, recovery_supported=True,
                                      return_protocol=self.backend.contract.get("return_protocol"),
                         hold_return_protocol=self.backend.contract.get("hold_return_protocol"),
                                      rejected_packets=self.rejected_packets, last_rejection=self.last_rejection,
                                      last_return=self.last_return, entry_token=self.entry_token,
                                      entry_protocol=self.backend.contract.get("entry_protocol"),
                                      input_hold_protocol=self.backend.contract.get("input_hold_protocol"),
                                      input_hold_age_s=(None if self.gate.hold_started_at is None else
                                                        now-self.gate.hold_started_at),
                                      hold_reason=(self.gate.target.get("hold_reason", "input")
                                                   if self.gate.target and self.gate.target.get("hold") else None),
                                      position_hold_age_s=(None if self.gate.position_hold_started_at is None else
                                                           now-self.gate.position_hold_started_at),
                                      entry_ready=(self.mode == "CALIBRATING" and self.entry_candidate_since is not None
                                                   and now-self.entry_candidate_since >= .5 and self.gate.fresh(now)
                                                   and self.entry_matches(self.gate.target, distance=.01, joint_limit_deg=.5)))
                        status.update(self.entry_liveness(now))
                        status.update(self.recovery_status())
                        status["entry_execution"] = self.entry_execution_diagnostics()
                        packet = self.encode_state(self.gate.state(now, status))
                        peer = self.gate.peer
                    if packet is None:
                        continue
                    try:
                        self.socket.sendto(packet, peer)
                        sent_at = time.monotonic()
                        with self.lock:
                            if self._last_state_send_at is not None:
                                gap = sent_at - self._last_state_send_at
                                self.state_send_gap_s = gap
                                self.state_send_gap_max_s = max(self.state_send_gap_max_s, gap)
                                self.state_send_gaps_over_80ms += gap > .08
                            self._last_state_send_at = sent_at
                    except OSError:
                        self.input_fault("feedback send failed; explicit local recovery required")
        except OSError as exc:
            # The local executor owns braking even if its UDP worker exits.
            # Do not tear it down merely because remote telemetry is lost.
            self.input_fault(f"network I/O failed: {exc}; explicit local recovery required")
        except Exception as exc:
            self.diagnostic_event("network_state_error", reason=f"network worker failed: {exc}")
            self.trip(f"network worker failed: {exc}")
            self.shutdown.set()

    def motion_cancelled(self):
        self.watchdog()
        if self.mode == 'HOLD_LATCHED':
            raise InputHoldTransition('input stop transferred to robot-owned hold')
        return self.cancelled.is_set() or self.shutdown.is_set()

    def hold_cancelled(self):
        """Hand the executor to checked return without treating it as a fault."""
        with self.lock:
            if self.cancelled.is_set() or self.shutdown.is_set() or self.stop_pending:
                return True
            if self.mode == 'RETURNING' and self.pending == 'z_hold':
                # request() can run between the arm and hand hold commands.
                # Leave the accepted operation queued for the next tick; a
                # boolean cancellation here would become a hardware E-stop.
                raise HoldReturnTransition('latched hold transferred to checked return')
            return self.mode != 'HOLD_LATCHED'

    def _execute_traced(self, target, cancelled, context):
        hand = getattr(self.backend, 'hand', None)
        if not hasattr(hand, '_transport_context'):
            hand = None
        if hand is not None:
            hand._transport_context = context if not target.get('hold') else None
        try:
            return self._execute_traced_inner(target, cancelled, context)
        finally:
            if hand is not None:
                hand._transport_context = None

    def _execute_traced_inner(self, target, cancelled, context):
        trace = self.motion_trace
        if trace is None or not trace.enabled:
            return self.backend.step(target, cancelled)
        from .motion_trace import cached_feedback, clear_cycle_feedback
        clear_cycle_feedback(self.backend)
        self.backend._motion_trace_context = context
        started = time.monotonic_ns()
        error = None
        try:
            return self.backend.step(target, cancelled)
        except Exception as exc:
            error = str(exc)
            raise
        finally:
            try:
                trace.emit('execution_cycle', context=context, started_ns=started,
                           mode=self.mode, completed=error is None, error=error,
                           hold=bool(target and target.get('hold')), feedback=cached_feedback(self.backend))
            except Exception:
                # Observability must never change execution or mask its exception.
                trace.dropped += 1
            self.backend._motion_trace_context = None
            clear_cycle_feedback(self.backend)

    def tick(self):
        """Single executor: all hardware operations are serialized here."""
        trace_context = None
        self.watchdog()
        with self.lock:
            if (self.mode == "CALIBRATING" and self.pending is None
                    and self.gate.target is not None and self.gate.fresh(time.monotonic())):
                if self.backend.contract.get("entry_protocol") in (1, 2, 3, 4, 5, 6, 7):
                    if self.gate.target.get("entry", {}).get("phase") == "candidate":
                        if self.backend.contract.get("entry_protocol") in (2, 3, 4, 5, 6, 7) and self.entry_target is None:
                            self.entry_target = copy.deepcopy(self.gate.target)
                            self.entry_candidate_since = None
                        if not self.entry_matches(self.gate.target, distance=.01, joint_limit_deg=.5):
                            if self.backend.contract.get("entry_protocol") == 1:
                                self.entry_target = copy.deepcopy(self.gate.target)
                                self.entry_candidate_since = time.monotonic()
                            else:
                                self.entry_candidate_since = None
                            self.reason = ("标定目标或参考字段不一致；接管暂缓"
                                           if self.backend.contract.get("entry_protocol") in (3, 4, 5, 6, 7) else
                                           "PICO 已离开标定姿态或追踪波动；请回到标定姿态并保持稳定")
                        elif self.entry_candidate_since is None:
                            self.entry_candidate_since = time.monotonic()
                            self.reason = "标定目标已锁定；等待新鲜目标连续 0.5 秒"
                        elif time.monotonic() - self.entry_candidate_since >= .5:
                            self.reason = ("标定目标已锁定；正在等待控制台自动提交受检接管"
                                           if self.backend.contract.get("entry_protocol") in (2, 3, 4, 5, 6, 7) else
                                           "接管姿态已稳定；点击检查并接管，先规划后运动")
                        else:
                            self.reason = "等待接管目标连续稳定 0.5 秒"
                else:
                    self.pending, self.mode = "e", "ARMING"
                    self.reason = "PICO preparation verified; enabling from matching zero target"
            if (self.mode == "WAITING_FOR_ACTIVE_TARGET" and not self.pending
                    and self.gate.fresh(time.monotonic())
                    and (self.backend.contract.get("entry_keepalive_protocol") != 1
                         or time.monotonic() < self.gate.target_deadline)
                    and self.gate.target.get("entry", {}).get("phase") == "ready"
                    and self.gate.target["entry"].get("token") == self.entry_token):
                self.pending = "a"
            stop, self.stop_pending = self.stop_pending, False
            operation, self.pending = self.pending, None
            target = self.gate.target
        if stop:
            try:
                self.backend.stop()
                self.stop_send_returned = True
            except Exception as exc:
                self.stop_send_returned = False
                self.reason = f"stop send failed: {exc}; hardware state unverified"
        if self.shutdown.is_set():
            return
        try:
            if operation == "d":
                disabled = self.backend.disable()
                with self.lock:
                    self.disable_verified = bool(disabled)
                    self.reason = f"disable_verified={disabled}; z required before following"
            elif operation in ("dl", "dr"):
                side = "left" if operation == "dl" else "right"
                disabled = self.backend.disable_side(side)
                with self.lock:
                    self.reason = f"{side}_disable_verified={disabled}; restart session before following"
            elif operation == "dh":
                disabled = self.backend.disable_hands()
                with self.lock:
                    self.reason = f"hands_disable_verified={disabled}; restart session before following"
            elif operation == "p":
                result = self.backend.prepare_session(self.motion_cancelled)
                with self.lock:
                    self.last_return = result
                    self.record_return(result)
                    if (not self.motion_cancelled() and result.get("returned")
                            and result.get("disabled")
                            and result.get("hands_open")):
                        self.mode = "CALIBRATING"
                        self.reason = ("robot_zero_verified=True; hand_open_verified=True; "
                                       "waiting for complete PICO calibration pose")
                        self.gate.target, self.gate.deadline = None, 0.0
                    elif self.accept_checked_hold(result):
                        pass
                    else:
                        raise RuntimeError(return_failure("session preparation failed/cancelled", result))
            elif operation == "h":
                result = self.backend.approach_entry(self.entry_target, self.motion_cancelled,
                                                     self.start_approach)
                feedback = self.backend.snapshot()
                with self.lock:
                    if self.motion_cancelled():
                        raise RuntimeError("handover interrupted before arrival")
                    self.last_return = result
                    self.record_return(result)
                    self.feedback, self.feedback_time = feedback, time.monotonic()
                    if self.accept_checked_hold(result):
                        return  # Next executor tick maintains the verified hold.
                    self.entry_ready_deadline = time.monotonic() + 2.
                    self.entry_token = secrets.token_hex(16)
                    self.mode = "WAITING_FOR_ACTIVE_TARGET"
                    self.reason = "接管姿态已到位；等待新鲜目标连续性核验"
            elif operation == "a":
                self.check_ready_validity()
                self.backend.activate_entry(target, self.motion_cancelled)
                with self.lock:
                    self.check_ready_validity()
                    if self.motion_cancelled():
                        raise RuntimeError("activation interrupted")
                    self.entry_ready_deadline = None
                    self.entry_plan_token = ""
                    self.mode, self.reason = "ACTIVE", "safe handover complete; following fresh targets"
            elif operation == "e":
                self.backend.enable(target, self.motion_cancelled)
                with self.lock:
                    if self.motion_cancelled():
                        raise RuntimeError("enable interrupted")
                    self.mode, self.reason = "ACTIVE", "following fresh laptop targets"
            elif operation in ("z", "z_hold"):
                result = (self.backend.return_zero(self.cancelled.is_set, from_hold=True)
                          if operation == "z_hold" else self.backend.return_zero(self.cancelled.is_set))
                with self.lock:
                    self.last_return = result
                    self.record_return(result)
                    if not self.cancelled.is_set() and result["returned"] and result["disabled"]:
                        self.mode = "IDLE"
                        self.reason = ("zero/hand-open/disable verified; local e required"
                                       if result.get('hands_open') else
                                       "zero/disable verified; local e required")
                        self.first_fault_reason, self.first_fault_summary = None, None
                        self.gate.target, self.gate.deadline = None, 0.0
                    elif self.accept_checked_hold(result):
                        pass
                    else:
                        # A failed offline preflight/plan or hand-open check
                        # must remain FAULT, but need not re-latch E-stop on an
                        # already healthy, freshly verified disabled arm. Never
                        # suppress a pending explicit stop or uncertain state.
                        verified_disabled = False
                        if (result.get('disabled') and result.get('stage') in
                                ('preflight', 'planning', 'hand_open',
                                 'hand_open_before_arm', 'hand_open_verify')):
                            fresh = self.backend.snapshot()
                            age = fresh.get('arm_feedback_age_s')
                            q = fresh.get('arm_urdf_rad')
                            verified_disabled = (
                                fresh.get('enable_states') == [False]*7
                                and not fresh.get('controller_fault') and not fresh.get('driver_fault')
                                and isinstance(age, (int, float)) and 0 <= age <= .1
                                and isinstance(q, list) and len(q) == 7
                                and all(isinstance(v, (int, float)) and math.isfinite(v) for v in q))
                            self.feedback, self.feedback_time = fresh, time.monotonic()
                        self.trip(return_failure("return failed/cancelled", result), verified_disabled=verified_disabled)
                        return
            elif operation == "r":
                result = self.backend.recover_idle()
                with self.lock:
                    self.last_return = result
                    self.record_return(result)
                    if not self.cancelled.is_set() and result.get("disabled") and result.get("hands_ready"):
                        self.mode, self.reason = "IDLE", "hand-only disable/feedback verified; local e required"
                        self.first_fault_reason, self.first_fault_summary = None, None
                        self.gate.target, self.gate.deadline = None, 0.0
                    else:
                        raise RuntimeError(return_failure("hand-only recovery failed/cancelled", result))
            with self.lock:
                active, target = self.mode == "ACTIVE", self.gate.target
                trace_context = dict(self._trace_target_context or {})
                trace_context['deadline_s'] = self.gate.target_deadline
            if self.mode == "WAITING_FOR_ACTIVE_TARGET":
                self.backend.monitor_hold()
            if active:
                if self.motion_cancelled():
                    return
                self._execute_traced(target, self.motion_cancelled, trace_context)
            if self.mode == 'HOLD_LATCHED':
                self._execute_traced({'hold': True}, self.hold_cancelled, None)
            feedback = self.backend.snapshot()
            with self.lock:
                self.feedback, self.feedback_time = feedback, time.monotonic()
        except InputHoldTransition:
            return  # Next executor cycle brakes; do not convert an input stop to E-stop.
        except HoldReturnTransition:
            return  # Next cycle runs z_hold with its existing braking and checks.
        except Exception as exc:
            self.trip(f"executor: {type(exc).__name__}: {exc}")
            # enable can fail after partially enabling hardware; stop even if
            # an asynchronous watchdog has already changed the state to FAULT.
            if operation in ("e", "p", "a"):
                with self.lock:
                    self.stop_pending = True

    def check_ready_validity(self):
        if (self.backend.contract.get("entry_keepalive_protocol") == 1
                and time.monotonic() >= self.gate.target_deadline):
            raise RuntimeError("fresh ready acknowledgment expired during activation")

    def safe_record(self):
        try:
            self.record()
        except Exception as exc:
            # Losing observability is reported separately from motion safety.
            # Do not let a full disk disable a load-bearing arm, nor recursively
            # attempt to write the error through the failed journal.
            with self.lock:
                self.journal_error = short_text(f'{type(exc).__name__}: {exc}')
                self.journal = None
            try:
                print(f'[gateway] journal disabled: {self.journal_error}', flush=True)
            except OSError:
                pass

    def diagnostic_loop(self):
        # Planning can block the I/O worker for seconds. Keep dashboard phases
        # visible without putting filesystem writes on the network watchdog.
        while not self.network_done.wait(.05):
            self.safe_record()
        self.safe_record()

    def run(self):
        logger = threading.Thread(target=self.diagnostic_loop, name="robot-link-journal", daemon=True)
        logger.start()
        thread = threading.Thread(target=self.network_loop, name="robot-link-network", daemon=True)
        thread.start()
        try:
            while True:
                started = time.monotonic()
                self.tick()
                if self.shutdown.is_set():
                    break
                time.sleep(max(0.0, .02 - (time.monotonic() - started)))
        finally:
            # Covers exceptions/KeyboardInterrupt outside the normal operator path.
            try:
                self.backend.stop()
            finally:
                self.network_done.set()
                thread.join(timeout=1)
                logger.join()
                self.safe_record()  # A failed final log write must not skip resource cleanup.
                self.socket.close()
                self.backend.close()


def main():
    ap = argparse.ArgumentParser(description="Robot-side joint executor; mock by default")
    ap.add_argument("--bind", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=16000)
    ap.add_argument("--key-file", required=True)
    ap.add_argument("--side", choices=("left", "right", "both"), default="left",
                    help="both selects both arms AND both hands")
    ap.add_argument("--with-hand", action="store_true")
    ap.add_argument("--hand-only", action="store_true",
                    help="single-side LinkerHand control while the arm remains disabled")
    ap.add_argument("--hardware", action="store_true")
    ap.add_argument("--config")
    ap.add_argument("--log-file", help="JSONL diagnostics; default robot_link/log/gateway_<time>.jsonl")
    args = ap.parse_args()
    if args.hand_only and (not args.with_hand or args.side == "both"):
        ap.error("--hand-only requires --with-hand and a single side")
    key = key_from_file(args.key_file)
    # Whole robot execution is exclusive; prevent two endpoints or an old local
    # teleop from silently sharing CAN. Old programs do not honor this lock, so
    # operator must also stop them (documented in deployment checklist).
    import fcntl
    import os
    lock_file = None
    if args.hardware:
        if not args.config:
            ap.error("--hardware requires --config")
        lock_file = open(f"/tmp/esrobo-robot-link-{os.getuid()}.lock", "w")
        fcntl.flock(lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
        from .backends import HardwareBackend, DualHardwareBackend
        backend = (DualHardwareBackend(args.config) if args.side == "both" else
                   HardwareBackend(args.config, args.side, args.with_hand, hand_only=args.hand_only))
    else:
        backend = (MockDualBackend() if args.side == "both" else
                   MockBackend(args.side, args.with_hand, args.hand_only))
    try:
        gateway = Gateway(backend, key, args.bind, args.port)
    except BaseException:
        backend.close()
        raise
    print(json.dumps(dict(listening=gateway.address, hardware=args.hardware,
                          contract=backend.contract), ensure_ascii=False), flush=True)
    print("Robot terminal: e zero+PICO prepare, g check+approach, h controlled hold, s/x E-stop, d all disable, dl/dr arm disable, dh hands disable, r recover, z return, q exit (Enter required)", flush=True)
    from pathlib import Path
    log_path = Path(args.log_file) if args.log_file else Path(__file__).resolve().parents[1] / "log" / f"gateway_{time.time_ns()}.jsonl"
    try:
        log_path.parent.mkdir(parents=True, exist_ok=True)
        journal = log_path.open("x", encoding="utf-8")
        gateway.journal = journal
    except BaseException:
        gateway.socket.close()
        backend.close()
        raise
    from esrobo_teleop.debug.motion_trace import MotionTrace, metadata
    from .motion_trace import attach
    motion_trace = MotionTrace(log_path.with_name('execution_' + log_path.name.removeprefix('gateway_')),
        info=metadata(Path(__file__).resolve().parents[2], role='gateway', config=getattr(backend, 'cfg', None), contract=backend.contract))
    gateway.motion_trace = motion_trace
    if motion_trace.enabled:
        attach(backend, motion_trace)
    torque_log = None
    if args.hardware and hasattr(backend, 'attach_torque_recorder'):
        from esrobo_teleop.debug.async_json_log import AsyncJsonLog
        torque_path = log_path.with_name('torque_' + log_path.name.removeprefix('gateway_'))
        torque_log = AsyncJsonLog(torque_path, capacity=2048)
        backend.attach_torque_recorder(torque_log, log_path.stem, lambda: gateway.mode)
        print(f"Torque diagnostics: {torque_path}", flush=True)
    print(f"Diagnostics: {log_path}", flush=True)

    def console():
        import sys
        for line in sys.stdin:
            action = line.strip()
            try:
                gateway.request(action)
            except (ValueError, RuntimeError) as exc:
                if action in ("g", "h"):
                    gateway.record_operator_result(action, False, str(exc))
                print(f"REFUSED: {exc}", flush=True)
            else:
                if action in ("g", "h"):
                    gateway.record_operator_result(action, True,
                        "接管请求已接受；正在检查目标和路径" if action == 'g' else
                        "受控停止已接受；机器人端减速并锁存位置保持")
        gateway.request("q")  # Losing the operator terminal never leaves following active.

    import signal
    signal.signal(signal.SIGTERM, lambda *_: gateway.request("q"))
    signal.signal(signal.SIGINT, lambda *_: gateway.request("q"))
    threading.Thread(target=console, name="robot-link-console", daemon=True).start()
    retention_stopped = threading.Event()
    if args.hardware:
        from esrobo_teleop.debug.log_retention import start_log_maintenance
        start_log_maintenance(Path(__file__).resolve().parents[2], retention_stopped)
    try:
        gateway.run()
    finally:
        retention_stopped.set()
        try:
            motion_trace.close()
            if gateway.journal is not None:
                try:
                    journal.write(json.dumps(dict(record_type='motion_trace_closed', **motion_trace.stats()))+'\n')
                    journal.flush()
                except (OSError, ValueError):
                    pass  # A final disk error must not skip closing resources.
        finally:
            try:
                journal.close()
            except OSError:
                pass
            try:
                if torque_log is not None:
                    torque_log.close()
            finally:
                if lock_file:
                    lock_file.close()


if __name__ == "__main__":
    main()
