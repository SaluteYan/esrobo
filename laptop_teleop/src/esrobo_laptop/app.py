"""Receive feedback, compute fresh targets, and send protocol-v1 commands."""
import argparse
from collections import deque
import copy
import json
from pathlib import Path
import select
import signal
import sys
import time

import numpy as np

from esrobo_link.client import RobotClient
from esrobo_link.protocol import ProtocolError, canonical

from .config import ROOT, read_robot_config, read_settings
from .contract import verify_contract
from .inputs import InputUnavailable, SourceExpired
from .pipeline import Pipeline
from .hand_handover import HandHandover
from .wrist_handover import WristHandover
from .diagnostics import RuntimeDiagnostics
from .input_recovery import InputRecovery, RecoveryRejected


def recent_rate(samples, now, window_s=2.0):
    """Stream rate over a bounded recent window, excluding old entry pauses."""
    while samples and now-samples[0] > window_s:
        samples.popleft()
    return ((len(samples)-1)/(samples[-1]-samples[0])
            if len(samples) > 1 and samples[-1] > samples[0] else 0.)


class ControlRateMonitor:
    """Fail a live session before the robot's 200 ms command lease expires."""
    def __init__(self, minimum_hz, gap_timeout_s, rate_tolerance_hz=None, rate_window_s=1.0):
        self.minimum_hz = float(minimum_hz)
        # A one-second floating window can report 39.9 Hz for a nominal 40 Hz
        # stream because of scheduler jitter. The independent gap watchdog
        # still catches every interruption longer than gap_timeout_s.
        self.rate_tolerance_hz = (max(0.5, self.minimum_hz * 0.01)
                                  if rate_tolerance_hz is None else float(rate_tolerance_hz))
        self.rate_window_s = float(rate_window_s)
        if self.rate_window_s <= 0:
            raise ValueError("control rate window must be positive")
        self.gap_timeout_s = float(gap_timeout_s)
        self.started = self.last_target = None
        self.targets = deque(maxlen=200)

    def observe(self, now, mode, has_target):
        if mode not in ("ARMING", "ACTIVE"):
            self.started = None
            self.targets.clear()
            # Entry phases also send complete targets. Preserve their last
            # send time across WAITING_FOR_ACTIVE_TARGET -> ACTIVE so a single
            # source frame missing on the transition gets the actual 80 ms
            # budget, rather than an immediate false timeout.
            if mode in ("CALIBRATING", "CHECKING_ENTRY", "APPROACHING",
                        "WAITING_FOR_ACTIVE_TARGET"):
                if has_target:
                    self.last_target = now
            else:
                self.last_target = None
            return
        if self.started is None:
            self.started = now
        if has_target:
            self.last_target = now
            self.targets.append(now)
        if self.last_target is None or now-self.last_target > self.gap_timeout_s:
            raise RuntimeError(f"control target gap exceeded {self.gap_timeout_s*1000:.0f} ms")
        while self.targets and now-self.targets[0] > self.rate_window_s:
            self.targets.popleft()
        if now-self.started >= self.rate_window_s:
            hz = ((len(self.targets)-1)/(self.targets[-1]-self.targets[0])
                  if len(self.targets) > 1 else 0.)
            if hz < self.minimum_hz - self.rate_tolerance_hz:
                raise RuntimeError(f"control rate {hz:.1f} Hz below required {self.minimum_hz:.1f} Hz "
                                   f"(tolerance {self.rate_tolerance_hz:.1f} Hz; stop below "
                                   f"{self.minimum_hz-self.rate_tolerance_hz:.1f} Hz over "
                                   f"{self.rate_window_s:.0f} s)")


def robot_state_receive_timeout(state, settings):
    """Allow brief planner scheduling jitter only while every arm is disabled."""
    if state.get("mode") != "CHECKING_ENTRY":
        return .08
    feedback = state.get("feedback") or {}
    members = feedback.get("sides", {}) if settings.side == "both" else {settings.side: feedback}
    if all((members.get(side) or {}).get("enable_states") == [False] * 7
           for side in settings.sides):
        # Stay below the robot's 200 ms target lease; enabled phases keep 80 ms.
        return .14
    return .08


def feedback_for(state, settings, received_at, now=None):
    now = time.monotonic() if now is None else now
    feedback = state.get("feedback", {})
    members = feedback.get("sides", {}) if settings.side == "both" else {settings.side: feedback}
    cache = state.get("feedback_cache_age_s")
    if cache is None or not np.isfinite(cache) or cache < 0 or now < received_at:
        raise InputUnavailable("invalid feedback cache age")
    # The peer's sampling + cache age, local computation time, and a configured
    # transport budget are distinct. This is not a clock-synchronized latency measurement.
    lag = cache + now-received_at + settings.network_margin_s
    full = np.zeros(14)  # Non-controlled side is model-only in single-side mode.
    for side in settings.sides:
        item = members.get(side, {})
        ages = []
        if settings.hand_only:
            if item.get("enable_states") != [False] * 7:
                raise InputUnavailable(f"{side} arm must remain verified disabled in hand-only mode")
            if item.get("controller_fault") or item.get("driver_fault"):
                raise InputUnavailable(f"{side} arm safety fault in hand-only mode")
        else:
            q = np.asarray(item.get("arm_urdf_rad"), dtype=float)
            if q.shape != (7,) or not np.all(np.isfinite(q)):
                raise InputUnavailable(f"{side} seven-joint feedback missing")
            ages.append((item.get("arm_feedback_age_s"), settings.feedback_max_age_s))
            start = 0 if side == "left" else 7
            full[start:start+7] = q
        if settings.with_hand:
            hand = item.get("hand") or {}
            value = np.asarray(hand.get("position_unit"), dtype=float)
            if value.shape != (10,) or not np.all(np.isfinite(value)) or np.any((value < 0) | (value > 255)):
                raise InputUnavailable(f"{side} hand feedback missing/invalid")
            ages.append((hand.get("age_s"), settings.hand_feedback_max_age_s))
        if state["mode"] not in ("ARMING", "CHECKING_ENTRY", "APPROACHING"):
            if any(a is None or not np.isfinite(a) or a < 0 or a+lag > maximum
                   for a, maximum in ages):
                raise InputUnavailable(f"{side} robot feedback stale")
    return full, members


def measured_targets(members, settings):
    return {side: dict(arm_urdf_rad=(None if settings.hand_only else list(members[side]["arm_urdf_rad"])),
                       hand_unit=([round(v) for v in members[side]["hand"]["position_unit"]]
                                  if settings.with_hand else None)) for side in settings.sides}


def send_targets(client, targets, settings, *, entry=None):
    if settings.hand_only:
        client.send_hand_target(targets[settings.side]["hand_unit"])
    elif settings.side == "both":
        client.send_dual_target(
            left_arm_urdf_rad=targets["left"]["arm_urdf_rad"],
            right_arm_urdf_rad=targets["right"]["arm_urdf_rad"],
            left_hand_unit=targets["left"]["hand_unit"],
            right_hand_unit=targets["right"]["hand_unit"])
    else:
        target = targets[settings.side]
        if target.get("hold"):
            client.send_hold(reason="position")
        else:
            client.send_target(arm_urdf_rad=target["arm_urdf_rad"],
                               hand_unit=target["hand_unit"], **({"entry": entry} if entry is not None else {}))


class Controller:
    """One session; faults terminate following and require an explicit restart."""
    def __init__(self, client, pipeline, settings, contract, preview=False):
        self.client, self.pipeline, self.settings = client, pipeline, settings
        self.contract = canonical(contract)
        self.mode = "IDLE"
        self.preview = preview
        self.sent = 0
        self.computed = 0
        self.preparing = False
        self.safe_entry = contract.get("entry_protocol") in (3, 4, 5, 6, 7) and not preview
        self.entry_seed = None
        self.entry_token = ""
        self.entry_target = None
        self.entry_reference_source = None
        self.entry_ready_sent_at = None
        self.entry_active_pending = False
        self.entry_hand_source = None
        self.last_hand_source = None
        self.hand_handover = None
        self.wrist_handover = None
        self.last_wrist_source = None
        self.first_wrist_source = None
        self.first_resume_source = None
        self.last_arm_source = {}
        self.last_active_target = None
        self.last_active_target_at = None
        self.last_complete_input_at = None
        self.input_hold_started_at = None
        self.input_resume_count = 0
        self.input_resume_diagnostics = None
        self.input_hold_status = ""
        self.recovery_enabled = contract.get('input_recovery_enabled') is True
        self.recovery = None
        self.recovery_hands = {}
        self.recovery_diagnostics = None
        self.recovery_completion = None
        self.holds_sent = 0
        self.entry_keepalives_sent = 0
        self.entry_keepalive_sent_at = None
        if preview:
            self.pipeline.restart_preparation()

    def _send_input_hold(self, state, status):
        now = time.monotonic()
        if self.preview or state["mode"] != "ACTIVE" or self.last_active_target is None:
            return False
        if self.input_hold_started_at is None:
            self.input_hold_started_at = now
            self.input_resume_count = 0
            self.input_resume_diagnostics = None
            if (self.recovery_enabled and not self.entry_active_pending
                    and (self.wrist_handover is None or self.wrist_handover.done)
                    and (self.hand_handover is None or self.hand_handover.done)):
                if self.recovery is None:
                    self.recovery = InputRecovery(self.settings)
                    self.recovery.begin(self.last_active_target)
                self.recovery.pause()
        if self.recovery is not None:
            self.recovery.check_timeout(now)
        if now - self.input_hold_started_at > self.settings.input_hold_max_s:
            raise self._input_hold_timeout()
        _, members = feedback_for(state, self.settings, self.client.received_at, now)
        for side in self.settings.sides:
            member = members[side]
            if not self.settings.hand_only and (member.get("enable_states") != [True] * 7
                                                or member.get("controller_fault") or member.get("driver_fault")):
                raise InputUnavailable(f"{side} arm cannot maintain input hold")
        self.client.send_hold()
        for side, limiter in self.recovery_hands.items():
            limiter.hold(members[side]['hand']['position_unit'], now)
        if self.wrist_handover is not None and not self.wrist_handover.done:
            self.wrist_handover.pause(now)
        if self.hand_handover is not None and not self.hand_handover.done:
            self.hand_handover.hold(members[self.settings.side]["hand"]["position_unit"], now)
        self.holds_sent += 1
        self.input_hold_status = status
        return True

    def _input_hold_timeout(self):
        detail = (self.input_resume_diagnostics or {}).get("reason", "no fresh matching input")
        return RecoveryRejected(f"PICO input hold exceeded {self.settings.input_hold_max_s*1000:.0f} ms; "
                            f"last recovery check: {detail} "
                            f"({self.input_resume_count}/{self.settings.input_resume_frames}); explicit recovery required")

    def hold_missing_input(self, state, reason):
        if self.last_active_target_at is None:
            return False
        # An unreachable target can still come from a complete, fresh frame.
        # Its braking/hold command must not freeze the input-gap clock together
        # with the last reachable source anchor.
        last_input = (self.last_complete_input_at if self.last_complete_input_at is not None
                      else self.last_active_target_at)
        if (not self.entry_active_pending and self.input_hold_started_at is None
                and time.monotonic()-last_input < self.settings.input_hold_after_s):
            return False
        recovery = self.input_resume_diagnostics
        suffix = ""
        if recovery:
            suffix = (f"; last recovery check: {recovery['reason']} "
                      f"({self.input_resume_count}/{self.settings.input_resume_frames} matching frames)")
        phase = 'WAITING_FOR_FIRST_ACTIVE_INPUT' if self.entry_active_pending else 'PAUSED_INPUT'
        return self._send_input_hold(state, f"{phase}: {reason}; waiting for fresh PICO frames{suffix}")

    def _resume_continuous(self, targets):
        diagnostic = dict(evaluated_monotonic_s=time.monotonic(), allowed=False,
                          phase=('first_active_input' if self.entry_active_pending else 'input_recovery'),
                          reason="target is a position hold")
        self.input_resume_diagnostics = diagnostic
        first_wide = self.recovery_enabled and self.entry_active_pending
        diagnostic['candidate'] = InputRecovery.sources(targets)
        diagnostic['baseline'] = InputRecovery.sources(self.last_active_target)
        for side, source in self.last_arm_source.items():
            diagnostic['baseline'][side]['arm'] = list(source)
        for side in self.settings.sides:
            diagnostic["side"] = side
            previous, current = self.last_active_target[side], targets[side]
            if current.get("hold"):
                # Fresh PICO frames may still request a deliberate position
                # pause. Clearing a short input gap must not require resuming
                # movement: the resulting packet continues braking/holding.
                if self.settings.side in ("left", "right") and self.settings.with_hand and not self.settings.hand_only:
                    continue
                return False
            if not self.settings.hand_only:
                source = current.get('source_arm_urdf_rad', current['arm_urdf_rad'])
                baseline = np.asarray(self.last_arm_source.get(side, previous["arm_urdf_rad"]), dtype=float).copy()
                if self.wrist_handover is not None and not self.wrist_handover.done:
                    # A held entry wrist is not the human's last tracking target.
                    # First startup waits compare fresh raw wrist targets with
                    # one fixed first-frame anchor. Later gaps compare with the
                    # last successfully sent unfiltered source. Neither palm
                    # filtering nor joint handover progress is human movement.
                    wrist_source = (self.last_wrist_source if self.last_wrist_source is not None
                                    else self.first_wrist_source)
                    baseline[5:] = (source[5:] if wrist_source is None
                                    else wrist_source)
                delta = np.abs(np.asarray(source)-baseline)
                limits = np.asarray(([10.]*4+[15.,5.,5.] if first_wide else
                                     [self.settings.input_resume_arm_delta_deg]*7))
                joint = int(np.argmax(delta/np.deg2rad(limits)))
                if delta[joint] > np.deg2rad(limits[joint]):
                    diagnostic.update(joint=joint+1, arm_delta_deg=float(np.degrees(delta[joint])),
                                      arm_limit_deg=float(limits[joint]),
                                      reason=f"{side} J{joint+1} target delta {np.degrees(delta[joint]):.2f}deg "
                                             f"> {limits[joint]:g}deg; return to previous pose")
                    return False
        if (self.wrist_handover is not None and not self.wrist_handover.done
                and self.last_wrist_source is None and self.first_wrist_source is None
                and not targets[self.settings.side].get('hold')):
            current = targets[self.settings.side]
            self.first_wrist_source = list(current.get('source_arm_urdf_rad', current['arm_urdf_rad'])[5:])
        if first_wide:
            current_source = InputRecovery.sources(targets)
            adjacent = (self.first_resume_source is None or
                        InputRecovery(self.settings).within(current_source, self.first_resume_source)[0])
            self.first_resume_source = current_source
            if not adjacent:
                diagnostic.update(reason='first active input adjacent source jump')
                return False
        diagnostic.update(allowed=True, reason="fresh target matches previous pose")
        return True

    def _step_fixed_entry(self, state, measured):
        """Execute a frozen plan independently of current PICO delivery."""
        mode = state["mode"]
        if self.entry_target is None or self.entry_reference_source is None:
            raise InputUnavailable("缺少本次标定锁定的首次目标；请停止并重新准备")
        token = state.get("entry_plan_token")
        if not isinstance(token, str) or len(token) != 32:
            raise ProtocolError("missing current entry plan token")
        self.client.send_entry_keepalive(token)
        self.entry_keepalives_sent += 1
        self.entry_keepalive_sent_at = time.monotonic()
        self.mode = mode
        if mode != "WAITING_FOR_ACTIVE_TARGET":
            self.pipeline.preparation_status = (
                "正在检查锁定目标与路径；PICO 数据间隙不取消检查" if mode == "CHECKING_ENTRY" else
                "正在执行锁定目标；接近保活正常，到位后核验新鲜 PICO 输入")
            return None

        self.pipeline.preparation_status = "已到位并保持；等待完整新鲜 PICO 输入，最长 2 秒"
        # Only input preparation failures may wait. Robot feedback/continuity
        # failures below must still propagate to the existing stop handling.
        try:
            frame = self.pipeline.capture(full_target=True)
            if frame is None:
                return None
            source = self.pipeline.entry_source(frame)
            hand_source = (self.pipeline.map_hand_target(self.settings.side, frame[2],
                           {self.settings.side: state["feedback"]}) if self.settings.with_hand else None)
            self.pipeline.check_ticket(frame[0])
        except InputUnavailable:
            return None
        feedback_for(state, self.settings, self.client.received_at)
        side = self.settings.side
        offset = 0 if side == "left" else 7
        target = self.entry_target[side]
        if np.max(np.abs(np.asarray(target["arm_urdf_rad"])-measured[offset:offset+7])) > np.deg2rad(1.5):
            raise InputUnavailable("标定目标尚未到位，已拒绝进入实时跟随")
        self.entry_token = state["entry_token"]
        self._send_targets(self.entry_target, self.settings,
                     entry=dict(phase="ready", token=self.entry_token, source=source,
                                reference_source=self.entry_reference_source))
        self.entry_ready_sent_at = time.monotonic()
        self.entry_hand_source = copy.deepcopy(hand_source)
        self.sent += 1
        self.pipeline.body.consume(frame[0])
        self.pipeline.preparation_status = "到位与新鲜输入核验通过；等待网关进入实时跟随"
        return copy.deepcopy(self.entry_target)

    def _send_targets(self, targets, settings, **kwargs):
        return send_targets(self.client, targets, settings, **kwargs)

    def step(self, state):
        return self._step(state)

    def _step(self, state):
        if canonical(state["contract"]) != self.contract:
            raise ProtocolError("gateway configuration changed during session")
        mode = state["mode"]
        if mode not in ("IDLE", "RETURNING", "CALIBRATING", "ARMING", "CHECKING_ENTRY", "APPROACHING", "WAITING_FOR_ACTIVE_TARGET", "ACTIVE"):
            raise RuntimeError(f"robot {mode}: {state.get('reason', '')}; restart after local recovery")
        if mode == "RETURNING":
            # Reference collection starts only after the robot has completed
            # its return and reported CALIBRATING.  This preserves the user
            # visible order: robot zero first, then operator pose capture.
            self.preparing = False
            self.pipeline.preparation_status = "机器人正在回零（受检）；灵巧手保持当前位置，请等待完成"
            self.mode = mode
            return None
        if mode == "CALIBRATING" and not self.preparing:
            # A fast/mock return can complete between two 50 Hz state packets,
            # so CALIBRATING itself must also start a fresh reference cycle.
            self.pipeline.restart_preparation()
            self.preparing = True
            self.entry_seed = None
            self.entry_target = None
            self.entry_reference_source = None
            self.entry_ready_sent_at = None
            self.entry_active_pending = False
            self.entry_hand_source = None
            self.last_hand_source = None
            self.hand_handover = None
            self.wrist_handover = None
            self.last_wrist_source = None
            self.first_wrist_source = None
            self.first_resume_source = None
            self.last_arm_source = {}
            self.last_active_target = None
            self.last_active_target_at = None
            self.last_complete_input_at = None
            self.input_hold_started_at = None
            self.input_resume_count = 0
            self.input_resume_diagnostics = None
            self.input_hold_status = ""
            self.recovery = None
            self.recovery_hands = {}
            self.recovery_completion = None
        measured, feedback = feedback_for(state, self.settings, self.client.received_at)
        if (self.safe_entry and state["contract"].get("entry_keepalive_protocol") == 1
                and mode in ("CHECKING_ENTRY", "APPROACHING", "WAITING_FOR_ACTIVE_TARGET")):
            return self._step_fixed_entry(state, measured)
        if (self.safe_entry and state["contract"].get("entry_keepalive_protocol") == 1
                and mode == "ACTIVE" and self.mode == "WAITING_FOR_ACTIVE_TARGET"
                and self.entry_ready_sent_at is not None and self.entry_target is not None
                and self.entry_token == state.get("entry_token")):
            # ACTIVE acknowledges the checked, arrived ready target. Establish
            # its hold/recovery baseline before asking for another sensor frame:
            # that frame may already be missing or expired at this boundary.
            # This does not send a target or renew a PICO sample timestamp.
            self.last_active_target = copy.deepcopy(self.entry_target)
            self.last_active_target_at = self.entry_ready_sent_at
            self.entry_active_pending = True
            # Legacy replay only: protocol 7 finishes the calibrated wrist
            # stage on the gateway before ACTIVE, so it needs no live chase.
            if (state['contract'].get('entry_protocol') == 6
                    and self.settings.side in ('left', 'right') and not self.settings.hand_only):
                self.wrist_handover = WristHandover(
                    feedback[self.settings.side]['arm_urdf_rad'], self.pipeline.cfg.robot,
                    self.settings.side, time.monotonic(), self.settings.rate_hz)
            if self.settings.with_hand:
                if self.entry_hand_source is None:
                    raise InputUnavailable("missing fresh hand source at arrival acknowledgment")
                self.last_hand_source = copy.deepcopy(self.entry_hand_source)
                self.hand_handover = HandHandover(
                    feedback[self.settings.side]["hand"]["position_unit"],
                    self.pipeline.cfg.hand, time.monotonic(), self.settings.rate_hz)
            self.pipeline.reset_command_start(measured)
            self.mode = mode
        if mode == 'ACTIVE' and self.wrist_handover is not None:
            self.wrist_handover.check_timeout(time.monotonic())
        with self.pipeline.body.input_lock:
            if mode == "CALIBRATING" and not self.settings.hand_only and not self.pipeline.body.is_ready():
                self.pipeline.preparation_status = (
                    "请保持完整 PICO 追踪，等待有效初始位姿"
                )
            # Before e, only prove fresh source streams and send the measured
            # robot pose back as a no-jump target.  The real session reference
            # is deliberately collected after the robot has returned to zero.
            require_reference = self.preview or mode != "IDLE" or self.preparing
            ticket = self.pipeline.body.ticket(
                self.settings.input_max_age_s, require_reference=require_reference)
            if ticket is None:
                return None
            idle_frame = ((ticket, None, self.pipeline.body.hand_joints())
                          if not require_reference else None)
        # Each input API owns its short snapshot lock. An outer RLock here
        # would keep the receiver blocked throughout capture's retargeting.
        if mode == "CALIBRATING" and not self.pipeline.preparation_ready():
            return None
        if require_reference and (not self.pipeline.initialized
                                  or (not self.safe_entry and mode == "ACTIVE" and self.mode != "ACTIVE")):
            self.pipeline.initialize(measured)
            self.entry_seed = measured.copy()
        if self.safe_entry and mode == "ACTIVE" and self.mode != "ACTIVE":
            self.pipeline.reset_command_start(measured)
        entering = self.safe_entry and mode in (
            "CALIBRATING", "CHECKING_ENTRY", "APPROACHING", "WAITING_FOR_ACTIVE_TARGET")
        first_calibrated_goal = entering and mode == "CALIBRATING" and self.entry_target is None
        frame = (self.pipeline.capture_calibrated_entry() if first_calibrated_goal else
                 self.pipeline.capture(full_target=True) if entering else
                 self.pipeline.capture() if require_reference else idle_frame)
        if frame is None:
            return None
        if entering and not first_calibrated_goal:
            if self.entry_target is None or self.entry_reference_source is None:
                raise InputUnavailable("缺少本次标定锁定的首次目标；请停止并重新准备")
            targets = copy.deepcopy(self.entry_target)
        elif entering or mode in ("CALIBRATING", "ACTIVE") or self.preview:
            targets = self.pipeline.solve(
                frame, self.entry_seed if entering else measured, feedback,
                allow_limit_hold=(mode == "ACTIVE" and not self.preview),
                hold_entry_hand=entering,
            )
        else:
            targets = measured_targets(feedback, self.settings)
        # Recheck original sample/feedback ages AFTER potentially slow IK. Never
        # stamp old sensor input with the time at which computation completed.
        self.pipeline.check_ticket(frame[0])
        feedback_for(state, self.settings, self.client.received_at)
        resume_ready = False
        recovery_candidate = None
        recovery_complete = False
        raw_recovery_targets = None
        if mode == 'ACTIVE' and self.recovery is not None:
            if any(target.get('hold') for target in targets.values()):
                # An unreachable IK target carries measured joints for display,
                # not a new human source pose. Keep braking without qualifying
                # that placeholder as a recovery source.
                now = time.monotonic()
                reason = 'arm target unreachable; waiting for a valid recovery source'
                self.recovery.pause()
                self.recovery.diagnostic = dict(
                    at=now, anchor=copy.deepcopy(self.recovery.anchor), candidate=None,
                    source_timestamps=dict(frame[0]), allowed=False,
                    adjacent_allowed=False, reason=reason)
                self.recovery_diagnostics = self.recovery.diagnostics()
                self.input_resume_count = 0
                self.input_resume_diagnostics = dict(
                    evaluated_monotonic_s=now, allowed=False, reason=reason)
                self.pipeline.body.consume(frame[0])
                self._send_input_hold(state, 'PAUSED_INPUT: '+reason)
                return None
            recovery_candidate = copy.deepcopy(self.recovery)
            try:
                recovery_ready = recovery_candidate.evaluate(targets, time.monotonic())
            finally:
                if recovery_candidate.diagnostic is not None:
                    recovery_candidate.diagnostic['source_timestamps'] = dict(frame[0])
                self.recovery_diagnostics = recovery_candidate.diagnostics()
            if (self.recovery.started is not None
                    and not recovery_candidate.diagnostic['adjacent_allowed']):
                raise RecoveryRejected('input reacquisition adjacent source jump')
        if mode == "ACTIVE" and self.input_hold_started_at is not None:
            if time.monotonic()-self.input_hold_started_at > self.settings.input_hold_max_s:
                raise self._input_hold_timeout()
            self.input_resume_count = self.input_resume_count + 1 if self._resume_continuous(targets) else 0
            wide_ready = recovery_candidate is not None and recovery_ready
            if self.input_resume_count < self.settings.input_resume_frames and not wide_ready:
                if recovery_candidate is not None:
                    self.recovery = recovery_candidate
                self.pipeline.body.consume(frame[0])
                reason = (f"fresh matching frames {self.input_resume_count}/{self.settings.input_resume_frames}"
                          if self.input_resume_count else self.input_resume_diagnostics["reason"])
                phase = 'WAITING_FOR_FIRST_ACTIVE_INPUT' if self.entry_active_pending else 'PAUSED_INPUT'
                self._send_input_hold(state, phase+": " + reason)
                return None
            # Keep the original hold deadline until this candidate is actually
            # sent. Expiry after proposal or send failure is not a recovery.
            resume_ready = True
            if (recovery_candidate is not None and self.recovery.started is None
                    and self.input_resume_count >= self.settings.input_resume_frames
                    and not self.settings.with_hand):
                # Arm-only narrow recovery needs no new trajectory phase.
                # Hands always retain their measured-to-latest-target ramp:
                # their source excursion is no longer restricted by a gate.
                recovery_candidate = None
        if recovery_candidate is not None:
            raw_recovery_targets = copy.deepcopy(targets)
            recovery_complete = recovery_candidate.arrival(
                targets, feedback, list(self.pipeline.cfg.hand.enabled_physical_joints)
                if self.settings.with_hand else []) if self.recovery.started is not None else False
        if first_calibrated_goal:
            self.entry_reference_source = self.pipeline.calibrated_entry_source()
            self.entry_target = copy.deepcopy(targets)
        entry = None
        if self.safe_entry:
            phase = "idle"
            source = None
            if entering:
                source = self.pipeline.entry_source(frame)
                phase = "candidate"
                if mode == "WAITING_FOR_ACTIVE_TARGET":
                    side = self.settings.side
                    offset = 0 if side == "left" else 7
                    if np.max(np.abs(np.asarray(targets[side]["arm_urdf_rad"])-measured[offset:offset+7])) > np.deg2rad(1.5):
                        raise InputUnavailable("标定目标尚未到位，已拒绝进入实时跟随")
                    self.entry_token = state["entry_token"]
                    phase = "ready"
            elif mode == "ACTIVE":
                phase = "active"
                self.pipeline.preparation_status = "受检接管完成，正在跟随"
            entry = dict(phase=phase, token=self.entry_token, source=source,
                         reference_source=self.entry_reference_source if entering else None)
            if entering:
                prompts = {
                    "CALIBRATING": "有效初始位姿已锁定为首次目标；正在自动检查接管路径",
                    "CHECKING_ENTRY": "正在检查标定目标并规划外展路径",
                    "APPROACHING": "正在沿受检路径慢速接近标定目标；到位后跟随实时 PICO 姿态",
                    "WAITING_FOR_ACTIVE_TARGET": "已到达标定目标，正在核验实测姿态并切入实时跟随",
                }
                self.pipeline.preparation_status = prompts[mode]
        if not self.preview:
            hand_source = None
            hand_candidate = None
            hand_hold = None
            wrist_candidate = None
            wrist_source = None
            if mode == 'ACTIVE' and self.wrist_handover is not None and not self.wrist_handover.done:
                side = self.settings.side
                if targets[side].get('hold'):
                    self.wrist_handover.pause(time.monotonic())
                else:
                    wrist_source = list(targets[side].get('source_arm_urdf_rad', targets[side]['arm_urdf_rad'])[5:])
                    wrist_candidate = self.wrist_handover.propose(
                        targets[side]['arm_urdf_rad'], feedback[side]['arm_urdf_rad'], time.monotonic())
                    targets = copy.deepcopy(targets)
                    targets[side]['arm_urdf_rad'][5:] = wrist_candidate.position.tolist()
            if (mode == "ACTIVE" and self.hand_handover is not None and not self.hand_handover.done):
                side = self.settings.side
                if targets[side].get("hold"):
                    hand_hold = (list(feedback[side]["hand"]["position_unit"]), time.monotonic())
                else:
                    hand_source = list(targets[side]["hand_unit"])
                    targets = copy.deepcopy(targets)
                    hand_candidate = self.hand_handover.propose(
                        hand_source, feedback[side]["hand"]["position_unit"], time.monotonic())
                    targets[side]["hand_unit"] = list(hand_candidate.position)
                self.pipeline.check_ticket(frame[0])
                feedback_for(state, self.settings, self.client.received_at)
            if wrist_candidate is not None:
                self.pipeline.check_ticket(frame[0])
                feedback_for(state, self.settings, self.client.received_at)
            recovery_hand_candidates = {}
            if recovery_candidate is not None:
                for side in self.settings.sides:
                    if self.settings.with_hand:
                        if side not in self.recovery_hands:
                            self.recovery_hands[side] = HandHandover(
                                feedback[side]['hand']['position_unit'], self.pipeline.cfg.hand,
                                time.monotonic()-1/self.settings.rate_hz, self.settings.rate_hz,
                                live_goal=True)
                        proposal = self.recovery_hands[side].propose(
                            targets[side]['hand_unit'], feedback[side]['hand']['position_unit'], time.monotonic())
                        recovery_hand_candidates[side] = proposal
                        targets = copy.deepcopy(targets)
                        targets[side]['hand_unit'] = list(proposal.position)
                self.pipeline.check_ticket(frame[0])
                feedback_for(state, self.settings, self.client.received_at)
            self.client.recovery_metadata = (recovery_candidate.metadata(recovery_complete)
                                             if recovery_candidate is not None else self.recovery_completion)
            self._send_targets(targets, self.settings, entry=entry)
            if recovery_candidate is not None:
                recovery_candidate.commit(time.monotonic())
                for side, proposal in recovery_hand_candidates.items():
                    self.recovery_hands[side].commit(proposal)
                self.recovery = None if recovery_complete else recovery_candidate
                self.recovery_diagnostics = recovery_candidate.diagnostics()
                if recovery_complete:
                    self.recovery_completion = recovery_candidate.metadata(True)
                    self.recovery_hands = {}
                    self.recovery_diagnostics['phase'] = 'ACTIVE'
            elif resume_ready:
                self.recovery = None
                self.recovery_hands = {}
            if hand_candidate is not None:
                self.hand_handover.commit(hand_candidate)
            elif hand_hold is not None:
                self.hand_handover.hold(*hand_hold)
            if wrist_candidate is not None:
                self.wrist_handover.commit(wrist_candidate)
                self.last_wrist_source = wrist_source
                self.first_wrist_source = None
            self.sent += 1
            if mode == "ACTIVE":
                # Only a fresh, fully checked and successfully handled frame
                # advances this clock; no cached/replayed input or keepalive.
                self.last_complete_input_at = time.monotonic()
            if mode == "ACTIVE" and not any(target.get("hold") for target in targets.values()):
                self.last_active_target = copy.deepcopy(targets)
                if raw_recovery_targets is not None:
                    for side in self.settings.sides:
                        self.last_active_target[side]['hand_unit'] = raw_recovery_targets[side]['hand_unit']
                self.last_arm_source = {
                    side: list(target.get('source_arm_urdf_rad', target['arm_urdf_rad']))
                    for side,target in targets.items() if target['arm_urdf_rad'] is not None}
                self.last_active_target_at = time.monotonic()
                self.entry_active_pending = False
                if hand_source is not None:
                    self.last_hand_source = hand_source
        self.pipeline.commit_frame(frame, targets)
        if resume_ready:
            self.input_hold_started_at = None
            self.input_resume_count = 0
            self.input_hold_status = ""
            self.input_resume_diagnostics = None
        self.pipeline.publish_diagnostics(
            frame, targets, measured, preview=self.preview, robot_mode=mode)
        self.pipeline.body.consume(frame[0])
        self.computed += 1
        self.mode = mode
        if mode == "ACTIVE":
            self.preparing = False
        return targets


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("command", choices=("run", "inspect", "check"))
    p.add_argument("--config", default=str(ROOT / "config/laptop.yaml"))
    p.add_argument("--host")
    p.add_argument("--port", type=int)
    p.add_argument("--side", choices=("left", "right", "both"))
    p.add_argument("--arm-only", action="store_true")
    p.add_argument("--hand-only", action="store_true")
    p.add_argument("--key-file")
    p.add_argument("--input-port", type=int)
    p.add_argument("--mock", action="store_true", help="accept only explicit mock contracts on loopback")
    p.add_argument("--preview", action="store_true", help="compute targets but send no commands")
    p.add_argument("--seconds", type=float, default=0)
    p.add_argument("--log-file", help="new JSONL file; default laptop_teleop/log/session_<time>.jsonl")
    p.add_argument("--status-file", help="atomic, token-free live status for the local dashboard")
    return p


def main(argv=None):
    argument_parser = parser()
    args = argument_parser.parse_args(argv)
    settings = read_settings(args.config)
    for arg, field in (("host", "robot_host"), ("port", "robot_port"), ("side", "side"),
                       ("key_file", "key_file"), ("input_port", "input_port")):
        value = getattr(args, arg)
        if value is not None:
            setattr(settings, field, value)
    if args.arm_only:
        settings.with_hand = False
    if args.hand_only:
        if args.arm_only:
            argument_parser.error("--arm-only and --hand-only cannot be combined")
        settings.with_hand, settings.hand_only = True, True
    settings.validate()
    if args.mock and settings.robot_host not in ("127.0.0.1", "localhost"):
        raise ValueError("--mock is restricted to the local mock gateway")
    cfg = read_robot_config(settings)
    if args.command == "check":
        import importlib.util
        if settings.hand_only:
            model_dof = None
        else:
            from esrobo_teleop.ik.solver import IkSolver
            model_dof = IkSolver(cfg.ik)._model.nq
        required_sources = [
            f"{side}_{kind}"
            for side in settings.sides
            for kind in (("hand",) if settings.hand_only else
                         ("arm", "hand", "wrist") if settings.with_hand else ("arm",))
        ]
        print(json.dumps(dict(urdf=(None if settings.hand_only else cfg.ik.urdf_path), model_dof=model_dof,
                              robot=f"{settings.robot_host}:{settings.robot_port}",
                              side=settings.side, with_hand=settings.with_hand,
                              hand_only=settings.hand_only,
                              required_input_sources=required_sources,
                              required_robot_services=(
                                  ["gateway", "hands", "hand_bridge"]
                                  if settings.with_hand else ["gateway"]
                              ),
                              xrobotoolkit_sdk=importlib.util.find_spec("xrobotoolkit_sdk") is not None), indent=2))
        return 0
    client = RobotClient(settings.robot_host, settings.robot_port, Path(settings.key_file).expanduser())
    pipeline = None
    journal = None
    controlling = False
    service_terminated = False
    old_sigterm = signal.getsignal(signal.SIGTERM)
    def terminate(*_):
        nonlocal service_terminated
        service_terminated = True
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, terminate)
    try:
        state = client.connect()
        contracts = verify_contract(state["contract"], cfg, settings, mock=args.mock)
        client.expected_contract_id = state["contract"]["id"]
        print(f"Verified {settings.side} gateway {settings.robot_host}:{settings.robot_port}; contract={client.expected_contract_id}")
        if args.command == "inspect":
            # Omit session/lease tokens from user-visible diagnostics.
            print(json.dumps({k: state.get(k) for k in ("contract", "mode", "reason", "feedback", "feedback_cache_age_s")}, indent=2))
            return 0
        if state["mode"] != "IDLE":
            raise RuntimeError("start the laptop session only with an IDLE gateway")
        pipeline = Pipeline(cfg, settings, contracts)
        controller = Controller(client, pipeline, settings, state["contract"], args.preview)
        log_path = Path(args.log_file) if args.log_file else ROOT / "log" / f"session_{time.time_ns()}.jsonl"
        log_path.parent.mkdir(parents=True, exist_ok=True)
        status_path = Path(args.status_file) if args.status_file else None
        journal = RuntimeDiagnostics(log_path, status_path)
        pipeline.set_diagnostics(journal)
        journal.message(f"Listening for laptop adapters on 127.0.0.1:{settings.input_port}; log={log_path}")
        if settings.hand_only:
            pose_instruction = f"naturally open the selected {settings.side} hand in the headset view"
        elif settings.with_hand:
            pose_instruction = "keep complete PICO arm/waist tracking and hold your chosen arm pose steady; the robot hand stays still until arrival"
        else:
            pose_instruction = "keep complete PICO arm/waist tracking and hold your chosen arm pose steady"
        journal.message(f"Click Enable once. The robot returns to zero first; then {pose_instruction}.")
        if not settings.hand_only:
            journal.message("PICO calibration locks all seven joints: approach J1-J5 first, then J6-J7; fingers stay held. Live following starts after both stages arrive. Laptop s/x/q or Ctrl+C stops and exits.")
        else:
            journal.message("Hand-only mode keeps the arm disabled. Laptop s/x/q or Ctrl+C stops and exits.")
        started = time.monotonic()
        next_print, next_record = 0., 0.
        loop_times, compute_times, publish_times = deque(maxlen=200), deque(maxlen=200), deque(maxlen=200)
        # Count authenticated hold packets as control traffic, while the
        # separate input-hold timer prevents stale PICO data from persisting.
        control_monitor = ControlRateMonitor(settings.minimum_control_rate_hz,
                                             settings.control_gap_timeout_s,
                                             rate_tolerance_hz=(settings.hand_only_rate_tolerance_hz
                                                                if settings.hand_only else settings.arm_rate_tolerance_hz),
                                             rate_window_s=2.0)
        period, next_cycle, deadline_misses = 1/settings.rate_hz, time.monotonic(), 0
        last_mode = None
        last_hold_active = False
        last_recovery_check = None
        previous_state_received_at = client.received_at
        while not args.seconds or time.monotonic()-started < args.seconds:
            if sys.stdin.isatty() and select.select([sys.stdin], [], [], 0)[0]:
                if sys.stdin.readline().strip().lower() in ("s", "x", "q"):
                    client.stop()  # Explicit operator STOP always overrides latched hold.
                    break
            targets = None
            held = False
            status = "waiting"
            state_rx_gap_s = None
            entry_keepalives_before = controller.entry_keepalives_sent
            try:
                state = client.receive(timeout=robot_state_receive_timeout(state, settings))
                state_rx_gap_s = client.received_at - previous_state_received_at
                previous_state_received_at = client.received_at
                controlling = not args.preview
                holds_before = controller.holds_sent
                entry_keepalives_before = controller.entry_keepalives_sent
                targets = controller.step(state)
                if targets is not None:
                    compute_times.append(time.monotonic())
                if targets is None:
                    held = controller.holds_sent > holds_before or controller.hold_missing_input(
                        state, "waiting for next complete source frame")
                    status = controller.input_hold_status if held else "waiting for next complete source frame"
                    if controller.entry_keepalives_sent > entry_keepalives_before:
                        status = pipeline.preparation_status
                elif any(target.get("hold") for target in targets.values()):
                    status = "ACTIVE: arm target unreachable; arm/hand braking/holding, move PICO arm back"
                elif any(target.get("limit_approach") for target in targets.values()):
                    status = "ACTIVE: finishing last valid joint target at limit; move PICO arm back"
                else:
                    status = ('REACQUIRING' if controller.recovery is not None
                              and controller.recovery.started is not None else state["mode"])
            except (InputUnavailable, TimeoutError) as exc:
                status = str(exc)
                if isinstance(exc, SourceExpired) and state["mode"] == "ACTIVE":
                    held = controller.hold_missing_input(state, status)
                    if held:
                        status = controller.input_hold_status
                if isinstance(exc, InputUnavailable) and "_hand" in status:
                    with pipeline.body.input_lock:
                        details = []
                        for side in settings.sides:
                            key = f"{side}_hand"
                            if key not in status:
                                continue
                            stamp = pipeline.body.stamps.get(key)
                            age = "never" if stamp is None else f"{(time.monotonic()-stamp)*1000:.0f} ms"
                            sdk = pipeline.body.pico_hand_status.get(side, "unknown")
                            sdk_at = pipeline.body.pico_hand_status_at
                            sdk_age = "unknown" if sdk_at is None else f"{(time.monotonic()-sdk_at)*1000:.0f} ms old"
                            details.append(f"{side} sample age={age}, PICO SDK={sdk} ({sdk_age})")
                        if details:
                            status += "; " + "; ".join(details)
                if (not held and state["mode"] in
                        ("ACTIVE", "ARMING", "CHECKING_ENTRY", "APPROACHING", "WAITING_FOR_ACTIVE_TARGET")):
                    raise RuntimeError(f"following stopped: {status}") from exc
            now = time.monotonic()
            loop_times.append(now)
            if not args.preview:
                control_monitor.observe(now, state["mode"], targets is not None or held or
                                        controller.entry_keepalives_sent > entry_keepalives_before)
            hold_active = controller.input_hold_started_at is not None
            recovery_check = ((controller.recovery_diagnostics or {}).get('check') or {}).get('at')
            if (now >= next_record or state["mode"] != last_mode or hold_active != last_hold_active
                    or recovery_check != last_recovery_check):
                last_recovery_check = recovery_check
                publish_times.append(now)
                # Assemble expensive diagnostic snapshots only when publishing,
                # after the control and safety checks for this cycle.
                record = dict(unix_ns=time.time_ns(), monotonic_s=now, mode=state["mode"], status=status,
                              sent=controller.sent, preview=args.preview, targets=targets,
                              input_hold_active=controller.input_hold_started_at is not None,
                              input_recovery=controller.recovery_diagnostics,
                              input_recovery_enabled=controller.recovery_enabled,
                              input_hold_age_s=(None if controller.input_hold_started_at is None else
                                                now-controller.input_hold_started_at),
                              input_hold_count=controller.holds_sent,
                              input_resume_count=controller.input_resume_count,
                              input_resume_diagnostics=controller.input_resume_diagnostics,
                              wrist_handover=(None if controller.wrist_handover is None else dict(
                                  controller.wrist_handover.diagnostics(),
                                  phase=('waiting_first_input' if controller.entry_active_pending else
                                         'complete' if controller.wrist_handover.done else 'transition'),
                                  source_target_rad=controller.last_wrist_source)),
                              hand_handover=(None if controller.hand_handover is None else dict(
                                  active=not controller.hand_handover.done,
                                  source_target=controller.last_hand_source,
                                  reference_target=controller.hand_handover.reference_target,
                                  phase=('initial_transition' if not controller.hand_handover.done else 'live_following'),
                                  command=controller.hand_handover.command.tolist(),
                                  reversal_wait_physical=list(controller.hand_handover.reversal_wait),
                                  age_s=now-controller.hand_handover.started,
                                  matched_frames=controller.hand_handover.matched)),
                              state_seq=state.get("state_seq"), state_rx_gap_s=state_rx_gap_s,
                              state_send_gap_s=state.get("state_send_gap_s"),
                              state_send_gap_max_s=state.get("state_send_gap_max_s"),
                              position_limit_hold=bool(targets and any(
                                  target.get("hold") for target in targets.values())),
                              feedback=state.get("feedback"), feedback_cache_age_s=state.get("feedback_cache_age_s"),
                              input_rejection=pipeline.body.last_rejection)
                if state["mode"] in ("RETURNING", "CALIBRATING", "CHECKING_ENTRY", "APPROACHING", "WAITING_FOR_ACTIVE_TARGET"):
                    record["preparation_status"] = pipeline.preparation_status
                record["entry_keepalive_count"] = controller.entry_keepalives_sent
                record["entry_keepalive_age_s"] = (None if controller.entry_keepalive_sent_at is None else
                                                    now-controller.entry_keepalive_sent_at)
                record["gateway_entry_keepalive_age_s"] = state.get("entry_keepalive_age_s")
                with pipeline.body.input_lock:
                    record["sources"] = {k: max(0., now-v) for k, v in pipeline.body.stamps.items()}
                    record["source_hz"] = pipeline.body.source_rates(now)
                    record["input_receive_counts"] = dict(pipeline.body.receive_counts)
                    record["required_sources"] = pipeline.body.required
                    record["input_source"] = "pico"
                    record["pico_hand_status"] = dict(pipeline.body.pico_hand_status)
                    record["pico_hand_status_age_s"] = (None if pipeline.body.pico_hand_status_at is None else
                                                        now-pipeline.body.pico_hand_status_at)
                    record["pico_wrist_input"] = copy.deepcopy(pipeline.body.wrist_diagnostics())
                    record["wrist_mapping"] = {
                        side: copy.deepcopy(wrist.last_diagnostics)
                        for side, wrist in pipeline.wrists.items()
                    }
                    body_reference_ready = settings.hand_only or pipeline.body.is_ready()
                    hand_reference_ready = (not settings.with_hand
                                            or all(side in pipeline.hand_references
                                                   for side in settings.sides))
                    record["body_reference_ready"] = body_reference_ready
                    record["hand_reference_ready"] = hand_reference_ready
                    record["reference_ready"] = body_reference_ready and hand_reference_ready
                record["initialized"] = pipeline.initialized
                record["computed"] = controller.computed
                visualization = getattr(pipeline, '_visualization', None)
                if visualization is not None:
                    record['visualization'] = visualization.stats()
                record["rates"] = dict(loop_hz=recent_rate(loop_times, now),
                                       compute_hz=recent_rate(compute_times, now),
                                       status_hz=recent_rate(publish_times, now),
                                       control_hz=(recent_rate(control_monitor.targets, now,
                                                   control_monitor.rate_window_s)
                                                   if state['mode'] in ('ACTIVE', 'ARMING')
                                                   and not args.preview else None),
                                       display_window_s=2., configured_hz=settings.rate_hz,
                                       minimum_hz=settings.minimum_control_rate_hz,
                                       rate_tolerance_hz=control_monitor.rate_tolerance_hz,
                                       rate_window_s=control_monitor.rate_window_s,
                                       stop_below_hz=(control_monitor.minimum_hz-
                                                      control_monitor.rate_tolerance_hz),
                                       deadline_misses=deadline_misses)

                with pipeline.body.input_lock:
                    record["pico_hand_input_rad"] = {
                        side: angles.tolist() for side, angles in pipeline.body.pico_hand_targets.items()
                        if side in settings.sides
                    }
                    record["pico_hand_reference_rad"] = {
                        side: np.asarray(reference, dtype=float).tolist()
                        for side, reference in pipeline.hand_references.items()
                        if side in settings.sides
                    }
                journal.submit(record, event=(state['mode'] != last_mode or hold_active != last_hold_active))
                next_record = now+.2
            if now >= next_print or state["mode"] != last_mode or hold_active != last_hold_active:
                journal.message(f"[laptop] {status}; targets sent={controller.sent}",
                                event=(state['mode'] != last_mode or hold_active != last_hold_active))
                next_print = now+2
            last_mode = state["mode"]
            last_hold_active = hold_active
            next_cycle += period
            finished = time.monotonic()
            if finished > next_cycle:
                deadline_misses += 1
                if finished-next_cycle >= period:
                    next_cycle = finished
            time.sleep(max(0., next_cycle-time.monotonic()))
        return 0
    except KeyboardInterrupt:
        if controlling and not service_terminated:
            try:
                client.stop()
            except (OSError, TimeoutError):
                pass
        if journal:
            journal.submit(dict(unix_ns=time.time_ns(), monotonic_s=time.monotonic(),
                                record_type='exit', status='operator interrupted',
                                sent=getattr(locals().get('controller'), 'sent', 0),
                                mode=locals().get('state', {}).get('mode', 'unknown')), event=True)
        return 0
    except (OSError, ValueError, RuntimeError, TimeoutError) as exc:
        if isinstance(exc, RecoveryRejected) and controlling and controller.recovery_enabled:
            try:
                client.input_fault()
            except (OSError, TimeoutError):
                pass  # Gateway hold/lease watchdog remains authoritative.
        if journal:
            journal.submit(dict(unix_ns=time.time_ns(), monotonic_s=time.monotonic(),
                                record_type='terminal_fault', status=str(exc),
                                feedback=locals().get('state', {}).get('feedback'),
                                feedback_cache_age_s=locals().get('state', {}).get('feedback_cache_age_s'),
                                state_seq=locals().get('state', {}).get('state_seq'),
                                input_recovery=getattr(locals().get('controller'), 'recovery_diagnostics', None),
                                input_resume_diagnostics=getattr(locals().get('controller'), 'input_resume_diagnostics', None),
                                sent=getattr(locals().get('controller'), 'sent', 0),
                                mode=locals().get('state', {}).get('mode', 'unknown')), fault=True)
            journal.message(f"[laptop] {exc}", file=sys.stderr, fault=True)
        else:
            print(f"[laptop] {exc}", file=sys.stderr, flush=True)
        return 1
    finally:
        signal.signal(signal.SIGTERM, old_sigterm)
        if controlling:
            client.close()  # Best-effort authenticated STOP, otherwise gateway lease expires.
        else:
            client.socket.close()
        if pipeline:
            pipeline.close()
        if journal:
            journal.submit(dict(unix_ns=time.time_ns(), monotonic_s=time.monotonic(),
                                record_type='exit', status='control resources closed',
                                sent=getattr(locals().get('controller'), 'sent', 0),
                                mode=locals().get('state', {}).get('mode', 'unknown')), event=True)
            journal.close(timeout=1.)


if __name__ == "__main__":
    raise SystemExit(main())
