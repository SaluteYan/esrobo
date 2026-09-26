"""Dual NERO arm driver on top of the ``pyAgxArm`` SDK.

Drives the two physical NERO seven-DOF arms (left on ``can_piper1``, right on
``can_piper2``) and maps the full-URDF IK joint output to physical joint angles.

Joint mapping is explicit per side: ``physical = direction * URDF + offset``.
The default J2 offset is +pi/2, which maps the URDF J2 limits onto the NERO SDK
physical limits. Per-side direction vectors account for the installed right
arm wiring and encoder conventions.
"""

from __future__ import annotations

import time
import threading
from dataclasses import dataclass, replace

import numpy as np

from ..config import RobotConfig
from .command_trajectory import CommandTrajectory, TrajectoryFault
from .command_backpressure import CommandBackpressure, MoveJWaypointGate, joint_wire_key

# Number of physical joints on a NERO arm.
NERO_NUM_JOINTS = 7


@dataclass(frozen=True)
class JointFeedbackSnapshot:
    position: np.ndarray
    stamp: float
    received_monotonic: float
    source: str

    def __post_init__(self):
        position = np.asarray(self.position, dtype=float).copy()
        position.setflags(write=False)
        object.__setattr__(self, "position", position)

    def diagnostics(self):
        return dict(source=self.source, stamp_s=self.stamp,
                    age_ms=1000 * (time.monotonic() - self.received_monotonic),
                    physical_rad=self.position.tolist())


class NeroArm:
    """Wrapper around a single pyAgxArm NERO arm instance."""

    def __init__(self, cfg: RobotConfig, channel: str, side: str):
        from pyAgxArm import AgxArmFactory, ArmModel, create_agx_arm_config

        self._cfg = cfg
        self._side = side
        self._cfg_dict = create_agx_arm_config(
            robot=ArmModel.NERO,
            firmeware_version=cfg.firmware_enum_for(side),
            channel=channel,
            interface=cfg.can_interface,
            bitrate=cfg.can_bitrate,
        )
        self._robot = AgxArmFactory.create_arm(self._cfg_dict)
        # Keep the V1.12 left controller on the vendor SDK's native MOVE_J
        # sequence: one 0x151 J-mode frame immediately before every position
        # batch.  Repeated field runs showed that boundary-only mode selection
        # left this controller enabled and reporting J mode while it ignored
        # every streamed target.  The legacy right controller is commissioned
        # with boundary-only selection and keeps that traffic pattern.
        firmware = str(cfg.firmware_enum_for(side)).lower()
        self._repeat_motion_mode_per_target = side == "left" and firmware == "v112"
        self._tracking_control_mode = str(
            getattr(cfg, f"{side}_tracking_control_mode", "j")
        ).lower()
        if self._tracking_control_mode not in ("j", "cpv"):
            raise ValueError(f"{side}_tracking_control_mode must be 'j' or 'cpv'")
        if self._tracking_control_mode == "cpv" and not (
            side == "left" and firmware == "v112"
        ):
            raise ValueError("CPV tracking is commissioned only for the left V112 controller")
        self._robot.set_auto_set_motion_mode_enabled(
            self._repeat_motion_mode_per_target
        )
        self._cpv_tracking_ready = False
        self._selected_motion_mode = None
        self._connected = False
        self._last_feedback_timestamp: float | None = None
        self._last_disable_states: list[bool] | None = None
        self._last_torque_timestamp: float | None = None

    def collision_protection_enabled(self) -> bool:
        override = getattr(self._cfg, f"{self._side}_collision_protection_enabled", None)
        return bool(
            self._cfg.collision_protection_enabled if override is None else override
        )

    def _restore_sdk_auto_motion_mode(self) -> None:
        """Restore the side's MOVE_J auto-mode policy without sending CAN."""
        setter = getattr(self._robot, "set_auto_set_motion_mode_enabled", None)
        if callable(setter):
            setter(bool(getattr(self, "_repeat_motion_mode_per_target", False)))

    def configure_collision_protection(self) -> bool:
        if not self.collision_protection_enabled():
            return True
        getter = getattr(self._robot, "get_crash_protection_rating", None)
        setter = getattr(self._robot, "set_crash_protection_rating", None)
        if not callable(getter) or not callable(setter):
            return False
        ratings = np.asarray(self._cfg.collision_protection_rating, dtype=int).reshape(-1)
        if ratings.shape != (NERO_NUM_JOINTS,) or np.any((ratings < 1) | (ratings > 8)):
            raise ValueError("collision_protection_rating must contain seven values in [1, 8]")
        # Reading is sufficient when the controller already carries the
        # commissioned values. Rewriting all seven parameters immediately
        # before enable left the installed V1.121 controller in a state where
        # it reported NORMAL/CAN/MOVE_J but rejected the first trajectory.
        # A controller reset made the same target executable without changing
        # the stored ratings, so avoid the disruptive write-only startup path.
        current = getter()
        if current is not None and list(current.msg) == ratings.tolist():
            return True
        for joint_index, rating in enumerate(ratings, start=1):
            if not setter(joint_index=joint_index, rating=int(rating)):
                return False
        feedback = getter()
        if feedback is None:
            return not bool(self._cfg.require_collision_protection_readback)
        return list(feedback.msg) == ratings.tolist()

    def wait_for_disabled_state(self, timeout: float = 1.0) -> bool:
        deadline = time.monotonic() + max(0.0, float(timeout))
        while True:
            states = self.get_joint_enable_states()
            if states is not None:
                return len(states) == NERO_NUM_JOINTS and not any(states)
            if time.monotonic() >= deadline:
                return False
            time.sleep(0.02)

    def get_joint_torques(self) -> np.ndarray | None:
        values = []
        timestamps = []
        for joint_index in range(1, NERO_NUM_JOINTS + 1):
            feedback = self._robot.get_motor_states(joint_index)
            if feedback is None:
                return None
            values.append(float(feedback.msg.torque))
            timestamps.append(float(feedback.timestamp))
        torques = np.asarray(values, dtype=np.float64)
        stamp = max(timestamps)
        if (
            not np.all(np.isfinite(torques))
            or stamp - min(timestamps) > 0.2
            or self._last_torque_timestamp is not None
            and stamp <= self._last_torque_timestamp
        ):
            return None
        self._last_torque_timestamp = stamp
        return torques

    def connect(self) -> None:
        self._selected_motion_mode = None
        self._cpv_tracking_ready = False
        self._robot.connect()
        from ..debug.can_send_monitor import CanSendMonitor
        comm = self._robot._ctx.get_comm()
        if getattr(self, "_monitored_comm", None) is not comm:
            self._send_monitor = CanSendMonitor(
                comm, send_timeout_s=self._cfg.can_send_timeout_s
            )
            self._monitored_comm = comm
        else:
            self._send_monitor.bind_bus()
        self._connected = True

    def enable(self, timeout: float = 8.0, *, cancelled=None, send_lock=None) -> bool:
        # A mode sent while disabled may have been ignored by the controller.
        # Socket-send success must not suppress the first post-enable mode
        # frame (which also carries the configured speed). No position is sent
        # here; the caller still verifies enable feedback before its hold.
        self._selected_motion_mode = None
        self._cpv_tracking_ready = False
        self._restore_sdk_auto_motion_mode()
        start = time.monotonic()
        while time.monotonic() - start < timeout:
            if cancelled is not None and cancelled():
                return False
            if send_lock is None:
                enabled = self._robot.enable()
            else:
                with send_lock:
                    if cancelled is not None and cancelled():
                        return False
                    enabled = self._robot.enable()
            if enabled:
                self._selected_motion_mode = None
                return True
            time.sleep(0.05)
        return False

    def disable(self, timeout: float = 3.0) -> bool:
        self._selected_motion_mode = None
        self._cpv_tracking_ready = False
        self._restore_sdk_auto_motion_mode()
        start = time.monotonic()
        # A powered NERO may already be disabled while it is still publishing
        # fresh low-speed feedback. Do not send another disable command in that
        # state: some controllers stop CAN push as soon as they receive it,
        # which turns a verified-safe state into unverifiable missing feedback.
        precheck_deadline = start + min(max(timeout, 0.0), 0.5)
        while time.monotonic() < precheck_deadline:
            states = self.get_joint_enable_states()
            self._last_disable_states = states
            if states is not None:
                if not any(states):
                    return True
                break
            time.sleep(0.02)

        while time.monotonic() - start < timeout:
            # Use the SDK broadcast first, then explicitly disable each joint
            # that is still enabled. Some NERO controller modes ignore the
            # broadcast command while accepting the per-joint command.
            self._robot.disable()
            states = self.get_joint_enable_states()
            self._last_disable_states = states
            if states is not None and not any(states):
                return True
            if states is not None:
                for joint_index, enabled in enumerate(states, start=1):
                    if enabled:
                        self._robot.disable(joint_index)
                retry_delay = 0.05
            else:
                retry_delay = 0.25
            # With no feedback, do not multiply one uncertain broadcast into
            # seven per-joint writes. Missing feedback never counts as a
            # verified disable, but a bounded broadcast retry is sufficient.
            time.sleep(retry_delay)
        return False

    def best_effort_disable_once(self) -> None:
        """Send broadcast and per-joint disable frames without trusting feedback.

        Some NERO controllers intermittently ignore the all-joint selector while
        waking from their power-on silent state.  Explicit per-joint disables
        make the failure path deterministic without issuing any motion target.
        """
        self._selected_motion_mode = None
        self._cpv_tracking_ready = False
        self._restore_sdk_auto_motion_mode()
        self._robot.disable()
        for joint_index in range(1, NERO_NUM_JOINTS + 1):
            self._robot.disable(joint_index)

    @property
    def last_disable_states(self) -> list[bool] | None:
        """Most recent seven-axis enable-state snapshot from disable()."""
        states = getattr(self, "_last_disable_states", None)
        return None if states is None else list(states)

    def get_joint_enable_states(self) -> list[bool] | None:
        """Return seven verified enable bits, or None when feedback is unavailable."""
        states: list[bool] = []
        for joint_index in range(1, NERO_NUM_JOINTS + 1):
            feedback = self._robot.get_driver_states(joint_index)
            if feedback is None:
                return None
            states.append(bool(feedback.msg.foc_status.driver_enable_status))
        return states

    def controller_fault(self, *, allow_disabled=False, allow_estop=False):
        """Read-only, timestamp-checked controller and seven-driver snapshot."""
        timeout = self._cfg.controller_status_timeout_s
        status = self._robot.get_arm_status()
        def fresh(message):
            return (message is not None and np.isfinite(message.timestamp)
                    and 0 <= time.time() - message.timestamp <= timeout)
        if not fresh(status):
            return {"category": "controller_feedback_stale"}
        fields = {k: int(getattr(status.msg, k)) for k in ("arm_status", "err_code", "ctrl_mode")}
        allowed_statuses = ((0, 6) if allow_disabled else (0,))
        if allow_estop:
            allowed_statuses = (*allowed_statuses, 1)
        if (fields["arm_status"] not in allowed_statuses or fields["err_code"]
                or (not allow_disabled and fields["ctrl_mode"] != 1)):
            fault = {"category": "controller_fault", "stamp_s": status.timestamp, **fields}
            if fields["arm_status"] == 1:
                fault["status_name"] = "EMERGENCY_STOP"
            return fault
        selected_mode = getattr(self, "_selected_motion_mode", None)
        expected_feedback = {"j": 1, "cpv": 5}.get(selected_mode)
        if (not allow_disabled and expected_feedback is not None
                and getattr(status.msg, "mode_feedback", None) != expected_feedback):
            return {"category": "motion_mode_mismatch", "expected": selected_mode,
                    "mode_feedback": getattr(status.msg, "mode_feedback", None),
                    "stamp_s": status.timestamp}
        for i in range(1, 8):
            state = self._robot.get_driver_states(i)
            if not fresh(state):
                return {"category": "driver_feedback_stale", "joint": i}
            flags = state.msg.foc_status
            active = [k for k in ("voltage_too_low", "motor_overheating", "driver_overcurrent",
                      "driver_overheating", "collision_status", "driver_error_status", "stall_status")
                      if getattr(flags, k)]
            if active or (not allow_disabled and not flags.driver_enable_status):
                return {"category": "driver_fault", "joint": i, "flags": active,
                        "enabled": bool(flags.driver_enable_status), "stamp_s": state.timestamp}
        return None

    def set_motion_mode(self, mode: str = "js") -> None:
        self._selected_motion_mode = None
        self._robot.set_motion_mode(getattr(self._robot.OPTIONS.MOTION_MODE, mode.upper()))
        self._selected_motion_mode = mode.lower()

    def _ensure_motion_mode(self, mode: str) -> None:
        if getattr(self, "_selected_motion_mode", None) != mode:
            self.set_motion_mode(mode)

    def set_normal_mode(self) -> None:
        self._selected_motion_mode = None
        self._cpv_tracking_ready = False
        self._robot.set_normal_mode()

    def request_can_feedback_push(self) -> bool:
        """Request periodic CAN feedback without sending a position target.

        Nero v1.12 implements ``set_normal_mode()`` as a no-op, but its mode
        message still carries the CAN-push field.  Use that field directly and
        restore the cached message immediately after transmission.
        """
        mode = getattr(self._robot, "_msg_mode", None)
        send_mode = getattr(self._robot, "_set_mode", None)
        if mode is None or not callable(send_mode) or not hasattr(mode, "enable_can_push"):
            return False
        previous_push = mode.enable_can_push
        previous_move_mode = mode.move_mode
        try:
            mode.enable_can_push = 0x01
            mode.move_mode = 0xFF
            send_mode()
        finally:
            mode.enable_can_push = previous_push
            mode.move_mode = previous_move_mode
        return True

    def enable_at_known_target(
        self,
        target: np.ndarray,
        *,
        speed_percent: int,
        timeout: float = 8.0,
    ) -> tuple[np.ndarray | None, list[bool] | None]:
        """Bootstrap a silent controller at an operator-confirmed joint target.

        The same target is sent immediately before and after the single
        all-joint enable request. No teleoperation delta is accepted here.
        """
        values = np.asarray(target, dtype=np.float64).reshape(-1)
        if values.shape != (NERO_NUM_JOINTS,) or not np.all(np.isfinite(values)):
            raise ValueError("known enable target must contain seven finite joints")

        safe_speed = min(max(int(speed_percent), 1), 10)
        self.reset()
        time.sleep(0.1)
        # Match the proven SenseGlove wrist arming sequence before entering the
        # silent-controller handshake: normal/motion mode, low speed, measured
        # hold target, then repeated SDK enable attempts.
        self.set_normal_mode()
        self.set_motion_mode(self._cfg.command_mode)
        self.set_speed_percent(safe_speed)
        self.move_j(values.tolist())

        deadline = time.monotonic() + max(0.5, float(timeout))
        next_configuration = 0.0
        next_status = 0.0
        attempts = 0
        side_label = getattr(self, "_side", "nero")
        while time.monotonic() < deadline:
            # The SDK's established arming path retries enable until driver
            # state feedback confirms it. A V1.12 controller may be completely
            # silent before one of these attempts opens feedback.
            self._selected_motion_mode = None
            self._robot.enable()
            attempts += 1
            now = time.monotonic()
            if now >= next_configuration:
                self.request_can_feedback_push()
                self.set_motion_mode(self._cfg.command_mode)
                self.set_speed_percent(safe_speed)
                self.move_j(values.tolist())
                next_configuration = now + 0.25
            joints = self.get_joint_angles(timeout=0.1)
            states = self.get_joint_enable_states()
            if joints is not None and states is not None and len(states) == NERO_NUM_JOINTS:
                self._last_disable_states = states
                print(
                    f"[{side_label}-arm] Enable handshake received complete joint feedback "
                    f"after {attempts} attempt(s).",
                    flush=True,
                )
                # The last mode/configuration attempt may predate the actual
                # enable transition. The next known-zero hold must reapply it.
                self._selected_motion_mode = None
                return joints, states
            if now >= next_status:
                remaining = max(0.0, deadline - now)
                print(
                    f"[{side_label}-arm] Waiting for seven-joint feedback after enable; "
                    f"attempts={attempts}, remaining={remaining:.1f}s.",
                    flush=True,
                )
                next_status = now + 1.0
            time.sleep(0.05)
        return None, None

    def set_speed_percent(self, percent: int = 100) -> None:
        # The SDK serializes a speed-only update as 0x151 with move_mode=0xFF.
        # Do not let our explicit-mode cache treat the older J selection as a
        # sufficient configuration after that frame.  The next allowed motion
        # command must send a complete mode+speed frame before its position
        # payload.  This is particularly important for the left V1.12 path:
        # field logs show fresh feedback and seven enabled drives, but no
        # execution after the last 0x151 was the speed-only 01ff... frame.
        self._selected_motion_mode = None
        try:
            self._robot.set_speed_percent(percent)
        except Exception:
            self._selected_motion_mode = None
            raise

    def get_joint_angles(self, timeout: float = 0.3) -> np.ndarray | None:
        """Read two fresh, consistent feedback samples without changing arm mode."""
        deadline = time.monotonic() + max(0.0, timeout)
        previous_by_source = {}
        while True:
            source = getattr(self, "_feedback_source", None)
            sources = (source,) if source else ("get_joint_angles", "get_leader_joint_angles")
            for source in sources:
                latest = getattr(self._robot, source)()
                if latest is None:
                    continue
                values = np.asarray(latest.msg, dtype=np.float64).reshape(-1)
                stamp = float(latest.timestamp)
                if values.shape != (7,) or not np.all(np.isfinite(values)) or not np.isfinite(stamp):
                    continue
                previous, previous_stamp = previous_by_source.get(source, (None, None))
                if (
                    values.shape == (NERO_NUM_JOINTS,)
                    and np.all(np.isfinite(values))
                    and previous is not None
                    and previous_stamp is not None
                    and stamp > previous_stamp
                    and np.max(np.abs(values - previous)) <= 0.05
                ):
                    self._last_feedback_timestamp = stamp
                    self._feedback_source = source
                    self._runtime_feedback = JointFeedbackSnapshot(values.copy(), stamp, time.monotonic(), source)
                    self._runtime_feedback_recovery_candidate = None
                    self.runtime_feedback_recovered = False
                    return values.copy()
                previous_by_source[source] = (values.copy(), stamp)
            if time.monotonic() >= deadline:
                return None
            time.sleep(0.01)

    def runtime_feedback(self, max_age_s=0.1, recovery_timeout_s=0.3):
        """Read the pinned source, requiring two fresh frames after a short gap.

        A gap never returns stale feedback and therefore cannot produce a new
        command.  It remains transient only for ``recovery_timeout_s``; malformed
        data, reversed timestamps and excessive position jumps fail immediately.
        """
        previous = getattr(self, "_runtime_feedback", None)
        self.runtime_feedback_failure = None
        self.runtime_feedback_recovered = False
        max_age_s = float(max_age_s)
        recovery_timeout_s = float(recovery_timeout_s)
        if not (np.isfinite(max_age_s) and 0 < max_age_s <= 0.1):
            raise ValueError("runtime feedback max age must be in (0, 0.1]")
        if not (np.isfinite(recovery_timeout_s) and recovery_timeout_s >= max_age_s):
            raise ValueError("runtime feedback recovery timeout must be at least max age")

        def reject(reason, *, transient=False, **details):
            self.runtime_feedback_failure = dict(
                reason=reason, previous=None if previous is None else previous.diagnostics(),
                transient=bool(transient),
                **details)
            return None
        if previous is None:
            return reject("no verified startup snapshot")
        now = time.monotonic()
        accepted_age = now - previous.received_monotonic
        try:
            message = getattr(self._robot, previous.source)()
        except Exception as exc:
            return reject("SDK feedback read failed", error=str(exc))
        if message is None:
            transient = accepted_age <= recovery_timeout_s
            return reject(
                "verified SDK feedback source returned None",
                transient=transient,
                feedback_gap_s=accepted_age,
            )
        try:
            values = np.asarray(message.msg, dtype=float)
            stamp = float(message.timestamp)
        except (TypeError, ValueError, AttributeError):
            return reject("malformed SDK feedback")
        if values.shape != (7,) or not np.all(np.isfinite(values)) or not np.isfinite(stamp):
            return reject("non-finite or incomplete SDK feedback")
        dt = stamp - previous.stamp
        if dt < 0:
            return reject("SDK feedback timestamp moved backwards", feedback_dt_s=dt)

        gap_detected = accepted_age > max_age_s or dt > max_age_s
        if not gap_detected and dt == 0:
            # Nero's getter assembles four CAN groups, using J7's timestamp.
            # J1..J6 may already differ while J7 is still the previous frame.
            # Ignore this partial update; never refresh age or advance commands.
            return previous

        if gap_detected:
            if accepted_age > recovery_timeout_s:
                return reject(
                    f"no accepted complete feedback for over {recovery_timeout_s * 1000:.0f} ms",
                    feedback_gap_s=accepted_age,
                    feedback_dt_s=dt,
                )
            candidate = getattr(self, "_runtime_feedback_recovery_candidate", None)
            if stamp <= previous.stamp:
                return reject(
                    "waiting for first new complete feedback after gap",
                    transient=True,
                    feedback_gap_s=accepted_age,
                    feedback_dt_s=dt,
                    recovery_samples=0,
                )
            if (
                candidate is None
                or stamp <= candidate.stamp
                or stamp - candidate.stamp > max_age_s
            ):
                delta = np.abs(values - previous.position)
                if np.max(delta) > .05:
                    return reject(
                        "joint feedback jump after gap exceeds 0.05 rad",
                        feedback_dt_s=dt,
                        candidate_physical_rad=values.tolist(),
                        failed_joints=(np.flatnonzero(delta > .05) + 1).tolist(),
                        joint_delta_rad=delta.tolist(),
                    )
                self._runtime_feedback_recovery_candidate = JointFeedbackSnapshot(
                    values.copy(), stamp, now, previous.source
                )
                return reject(
                    "validating feedback stream after gap (1/2)",
                    transient=True,
                    feedback_gap_s=accepted_age,
                    feedback_dt_s=dt,
                    recovery_samples=1,
                )
            candidate_dt = stamp - candidate.stamp
            candidate_delta = np.abs(values - candidate.position)
            if np.max(candidate_delta) > .05:
                return reject(
                    "joint feedback recovery jump exceeds 0.05 rad",
                    feedback_dt_s=candidate_dt,
                    candidate_physical_rad=values.tolist(),
                    failed_joints=(np.flatnonzero(candidate_delta > .05) + 1).tolist(),
                    joint_delta_rad=candidate_delta.tolist(),
                )
            result = JointFeedbackSnapshot(values.copy(), stamp, now, previous.source)
            self._runtime_feedback = result
            self._last_feedback_timestamp = stamp
            self._runtime_feedback_recovery_candidate = None
            self.runtime_feedback_recovered = True
            return result

        delta = np.abs(values - previous.position)
        if np.max(delta) > .05:
            return reject("joint feedback jump exceeds 0.05 rad",
                          feedback_dt_s=dt, candidate_physical_rad=values.tolist(),
                          failed_joints=(np.flatnonzero(delta > .05) + 1).tolist(),
                          joint_delta_rad=delta.tolist())
        result = JointFeedbackSnapshot(values.copy(), stamp, now, previous.source)
        self._runtime_feedback = result
        self._last_feedback_timestamp = stamp
        self._runtime_feedback_recovery_candidate = None
        return result

    @property
    def last_feedback_timestamp(self) -> float | None:
        """Timestamp in seconds from the most recently accepted SDK feedback."""
        return getattr(self, "_last_feedback_timestamp", None)

    def prepare_tracking_control(
        self,
        current: np.ndarray,
        velocity_limits: np.ndarray,
        acceleration_limits: np.ndarray,
        *,
        timeout: float = 1.0,
    ) -> bool:
        """Prepare the commissioned live-following mode without changing pose.

        V112 CPV has per-joint position commands intended for a continuously
        refreshed target. Prime every CPV position with measured feedback,
        then use the vendor's mode-before-parameter sequence to configure and
        acknowledge its rate limits. Finally hold the measured pose and
        verify CPV feedback before accepting a changing human target.
        """
        if self._tracking_control_mode == "j":
            self._cpv_tracking_ready = False
            self._restore_sdk_auto_motion_mode()
            return True

        position = np.asarray(current, dtype=np.float64).reshape(-1)
        vmax = np.asarray(velocity_limits, dtype=np.float64).reshape(-1)
        amax = np.asarray(acceleration_limits, dtype=np.float64).reshape(-1)
        if any(values.shape != (NERO_NUM_JOINTS,) for values in (position, vmax, amax)):
            raise ValueError("CPV preparation requires seven position, velocity and acceleration values")
        if (not np.all(np.isfinite(position)) or not np.all(np.isfinite(vmax))
                or not np.all(np.isfinite(amax)) or np.any(vmax <= 0) or np.any(amax <= 0)):
            raise ValueError("CPV preparation values must be finite with positive rate limits")

        required = ("set_cpv_cv", "set_cpv_acc", "set_cpv_dcc", "move_cpv_pos")
        missing = [name for name in required if not callable(getattr(self._robot, name, None))]
        if missing:
            raise RuntimeError(f"V112 CPV API unavailable: {missing}")

        self._cpv_tracking_ready = False
        self._robot.set_auto_set_motion_mode_enabled(False)
        switched = False
        try:
            # Prime the controller's per-joint CPV targets before selecting
            # CPV, then repeat the same hold immediately after the mode frame.
            # This bounds the mode boundary even if a previous session left a
            # stale CPV target in the controller.
            self._send_cpv_positions(position, auto_motion_mode=False)

            self.set_motion_mode("cpv")
            switched = True
            # Restore the vendor V112 CPV command sequence at the transition:
            # every per-joint CPV call first repeats the 0x151 CPV mode frame.
            # A single standalone mode frame was ignored by the installed
            # controller in session 1789976865948792213.
            self._send_cpv_positions(position, auto_motion_mode=True)

            # The vendor V112 setters emit CPV mode immediately before every
            # parameter frame.  Do not require mode_feedback=5 before these
            # calls: hardware session 1789978430614674814 proved that position
            # frames alone can leave status in MOVE_J, while the official API
            # defines the mode+parameter request/ACK as one operation.
            for joint_index in range(1, NERO_NUM_JOINTS + 1):
                index = joint_index - 1
                if not self._robot.set_cpv_cv(joint_index, float(vmax[index]), timeout=timeout):
                    raise RuntimeError(f"CPV velocity acknowledgement missing for J{joint_index}")
                if not self._robot.set_cpv_acc(joint_index, float(amax[index]), timeout=timeout):
                    raise RuntimeError(f"CPV acceleration acknowledgement missing for J{joint_index}")
                if not self._robot.set_cpv_dcc(joint_index, float(amax[index]), timeout=timeout):
                    raise RuntimeError(f"CPV deceleration acknowledgement missing for J{joint_index}")

            # Reassert the non-moving target before the final mode/pose check.
            self._send_cpv_positions(position, auto_motion_mode=True)
            deadline = time.monotonic() + max(0.1, float(timeout))
            next_hold = time.monotonic() + 0.1
            hold_batches = 1
            while time.monotonic() < deadline:
                status = self._robot.get_arm_status()
                if (status is not None and np.isfinite(status.timestamp)
                        and 0 <= time.time() - status.timestamp <= self._cfg.controller_status_timeout_s
                        and int(getattr(status.msg, "ctrl_mode", -1)) == 1
                        and int(getattr(status.msg, "arm_status", -1)) == 0
                        and int(getattr(status.msg, "mode_feedback", -1)) == 5):
                    break
                now = time.monotonic()
                if now >= next_hold:
                    self._send_cpv_positions(position, auto_motion_mode=True)
                    hold_batches += 1
                    next_hold = now + 0.1
                time.sleep(0.01)
            else:
                raise RuntimeError(
                    "controller did not confirm CAN/CPV mode after "
                    f"{hold_batches} vendor-sequence hold batches"
                )

            measured = self.get_joint_angles(timeout=max(0.1, float(timeout)))
            if measured is None:
                raise RuntimeError("joint feedback unavailable after CPV mode transition")
            measured = np.asarray(measured, dtype=np.float64)
            mode_entry_error = np.abs(measured - position)
            max_mode_entry_error = float(np.deg2rad(1.0))
            if np.any(mode_entry_error > max_mode_entry_error):
                joints = (np.flatnonzero(mode_entry_error > max_mode_entry_error) + 1).tolist()
                raise RuntimeError(
                    f"measured pose moved during CPV mode transition on joints {joints}"
                )

            self._cpv_tracking_ready = True
            return True
        except Exception as exc:
            self._cpv_tracking_ready = False
            self._selected_motion_mode = None
            self._restore_sdk_auto_motion_mode()
            if switched:
                try:
                    self.set_motion_mode("j")
                except Exception as restore_exc:
                    raise RuntimeError(
                        f"CPV preparation failed ({exc}); MOVE_J restore also failed ({restore_exc})"
                    ) from exc
            raise

    def _send_cpv_positions(
        self, values: np.ndarray, *, auto_motion_mode: bool
    ) -> None:
        """Send one seven-joint CPV batch without overflowing SocketCAN TX.

        The V112 SDK can emit three wire frames per call (mode plus a doubled
        first position).  The installed gs_usb interface has a ten-frame TX
        queue, so an unpaced seven-joint Python loop can fail at J7 even when
        the controller and bus are healthy.
        """
        position = np.asarray(values, dtype=np.float64).reshape(-1)
        if position.shape != (NERO_NUM_JOINTS,) or not np.all(np.isfinite(position)):
            raise ValueError("CPV position batch requires seven finite joints")
        interval = max(0.0, float(self._cfg.cpv_inter_joint_interval_s))
        self._robot.set_auto_set_motion_mode_enabled(bool(auto_motion_mode))
        for joint_index, value in enumerate(position, start=1):
            self._robot.move_cpv_pos(joint_index, float(value))
            if interval > 0.0 and joint_index < NERO_NUM_JOINTS:
                time.sleep(interval)

    def move_tracking_positions(self, joints: list[float]) -> None:
        """Send one live-following target using the commissioned transport."""
        values = np.asarray(joints, dtype=np.float64).reshape(-1)
        if values.shape != (NERO_NUM_JOINTS,) or not np.all(np.isfinite(values)):
            raise ValueError("tracking target must contain seven finite joints")
        if self._tracking_control_mode == "j":
            return self.move_j(values.tolist())
        if not self._cpv_tracking_ready or self._selected_motion_mode != "cpv":
            raise RuntimeError("CPV tracking was not prepared and verified")
        # Keep the installed V112 controller on the SDK's native sequence:
        # every per-joint CPV position call repeats the CPV mode selection.
        # Do not collapse this to one boundary-only mode frame until hardware
        # logs prove that the controller retains CPV across the whole batch.
        self._send_cpv_positions(values, auto_motion_mode=True)

    def move_j(self, joints: list[float]) -> None:
        # Startup holds and safe returns deliberately leave CPV and restore the
        # vendor's V112 mode-before-target MOVE_J sequence.
        if getattr(self, "_cpv_tracking_ready", False):
            self._cpv_tracking_ready = False
            self._selected_motion_mode = None
            self._restore_sdk_auto_motion_mode()
        if getattr(self, "_repeat_motion_mode_per_target", False):
            # The SDK emits mode first and position second.  Cache success only
            # after the complete call; a failed mode/position write must leave
            # the next attempt requiring a fresh mode frame.
            self._selected_motion_mode = None
            try:
                result = self._robot.move_j(list(joints))
            except Exception:
                self._selected_motion_mode = None
                raise
            self._selected_motion_mode = "j"
            return result
        self._ensure_motion_mode("j")
        return self._robot.move_j(list(joints))

    def move_js(self, joints: list[float]) -> None:
        if getattr(self, "_repeat_motion_mode_per_target", False):
            self._selected_motion_mode = None
            try:
                result = self._robot.move_js(list(joints))
            except Exception:
                self._selected_motion_mode = None
                raise
            self._selected_motion_mode = "js"
            return result
        self._ensure_motion_mode("js")
        return self._robot.move_js(list(joints))

    def emergency_stop(self) -> None:
        self._selected_motion_mode = None
        self._cpv_tracking_ready = False
        self._restore_sdk_auto_motion_mode()
        self._robot.electronic_emergency_stop()

    def reset(self) -> None:
        """Clear a controller-latched electronic stop without enabling motors."""
        self._selected_motion_mode = None
        self._cpv_tracking_ready = False
        self._restore_sdk_auto_motion_mode()
        self._robot.reset()

    def is_connected(self) -> bool:
        return self._connected and self._robot.is_connected()

    def disconnect(self) -> None:
        self._selected_motion_mode = None
        self._cpv_tracking_ready = False
        try:
            self._robot.disconnect()
        except Exception:  # noqa: BLE001
            pass
        self._connected = False


class NeroDualArmDriver:
    """Manages both NERO arms and the URDF -> physical joint mapping."""

    LEFT_JOINT2_IDX = 1
    RIGHT_JOINT2_IDX = 8

    def __init__(self, cfg: RobotConfig):
        if cfg.command_mode not in ("j", "js"):
            raise ValueError("robot.command_mode must be 'j' or 'js'")
        if cfg.command_mode == "js" and not cfg.allow_unsafe_js:
            raise ValueError(
                "NERO move_js is unsmoothed high-risk passthrough; "
                "use command_mode='j' or explicitly set allow_unsafe_js=true"
            )
        self._cfg = cfg
        self._left = NeroArm(cfg, cfg.left_can_channel, "left")
        self._right = NeroArm(cfg, cfg.right_can_channel, "right")
        self._session_start: dict[str, np.ndarray] | None = None
        self._last_command_feedback_timestamp: dict[str, float | None] = {
            "left": None,
            "right": None,
        }
        self._command_velocity = {
            "left": np.zeros(NERO_NUM_JOINTS, dtype=np.float64),
            "right": np.zeros(NERO_NUM_JOINTS, dtype=np.float64),
        }
        self._torque_baseline: dict[str, np.ndarray] = {}
        self._torque_trip_counts = {"left": np.zeros(7, dtype=int), "right": np.zeros(7, dtype=int)}
        self._safety_fault_reason: str | None = None
        self._last_valid_torque_time = {"left": None, "right": None}

    @property
    def safety_fault_reason(self) -> str | None:
        return self._safety_fault_reason

    def _collision_enabled_for(self, side: str) -> bool:
        override = getattr(self._cfg, f"{side}_collision_protection_enabled", None)
        return bool(
            self._cfg.collision_protection_enabled if override is None else override
        )

    def _configure_arm_collision_safety(
        self, arm: NeroArm, side: str, *, disabled_verified: bool = False
    ) -> bool:
        enabled = self._collision_enabled_for(side)
        if enabled and not disabled_verified and not arm.wait_for_disabled_state():
            self._safety_fault_reason = (
                f"{side} collision protection requires complete all-joint disabled feedback"
            )
            return False
        if not arm.configure_collision_protection():
            self._safety_fault_reason = f"{side} collision protection setting/readback failed"
            return False
        if enabled:
            print(
                f"[teleop SAFETY] {side} controller collision ratings verified: "
                f"{list(self._cfg.collision_protection_rating)}.",
                flush=True,
            )
        return True

    def _capture_arm_torque_baseline(self, arm: NeroArm, side: str, *, cancelled=None) -> bool:
        if not bool(self._cfg.torque_monitor_enabled):
            return True
        settle_s = max(0.0, float(self._cfg.torque_baseline_settle_s))
        if settle_s > 0.0:
            print(
                f"[teleop SAFETY] {side} arm holding zero for {settle_s:.1f}s "
                "before torque baseline capture; PICO following has not started.",
                flush=True,
            )
            if cancelled is None:
                time.sleep(settle_s)
            else:
                settle_deadline = time.monotonic() + settle_s
                while time.monotonic() < settle_deadline:
                    if cancelled():
                        return False
                    time.sleep(min(0.02, max(0.0, settle_deadline - time.monotonic())))
        required = max(3, int(self._cfg.torque_baseline_samples))
        deadline = time.monotonic() + max(0.5, float(self._cfg.torque_baseline_timeout_s))
        samples = []
        while len(samples) < required and time.monotonic() < deadline:
            if cancelled is not None and cancelled():
                return False
            value = arm.get_joint_torques()
            if value is not None:
                samples.append(value)
            time.sleep(0.02)
        if len(samples) < required:
            self._safety_fault_reason = f"{side} torque baseline unavailable"
            return False
        self._torque_baseline[side] = np.median(np.asarray(samples), axis=0)
        self._torque_trip_counts[side] = np.zeros(7, dtype=int)
        self._last_valid_torque_time[side] = time.monotonic()
        print(
            f"[teleop SAFETY] {side} stationary torque baseline(Nm)="
            f"{np.round(self._torque_baseline[side], 2).tolist()}, limits="
            f"{self._torque_limits_for(side).tolist()}; monitoring active.",
            flush=True,
        )
        return True

    def _torque_limits_for(self, side: str) -> np.ndarray:
        override = getattr(self._cfg, f"{side}_torque_deviation_limits_nm", None)
        value = self._cfg.torque_deviation_limits_nm if override is None else override
        name = (
            "torque_deviation_limits_nm"
            if override is None
            else f"{side}_torque_deviation_limits_nm"
        )
        return self._joint_limit_vector(value, name)

    def _torque_is_safe(self, arm: NeroArm, side: str) -> bool:
        if not bool(self._cfg.torque_monitor_enabled):
            return True
        baseline = self._torque_baseline.get(side)
        current = arm.get_joint_torques()
        if baseline is None:
            self._safety_fault_reason = f"{side} torque baseline unavailable"
            return False
        if current is None:
            last_valid = self._last_valid_torque_time.get(side)
            if (
                last_valid is not None
                and time.monotonic() - last_valid
                <= max(0.05, float(self._cfg.torque_feedback_timeout_s))
            ):
                return True
            self._safety_fault_reason = f"{side} torque feedback unavailable"
            return False
        self._last_valid_torque_time[side] = time.monotonic()
        limits = self._torque_limits_for(side)
        deviation = np.abs(current - baseline)
        exceeded = deviation > limits
        counts = np.where(exceeded, self._torque_trip_counts[side] + 1, 0)
        self._torque_trip_counts[side] = counts
        if np.any(counts >= max(1, int(self._cfg.torque_trip_consecutive_samples))):
            indices = np.flatnonzero(
                counts >= int(self._cfg.torque_trip_consecutive_samples)
            )
            details = "; ".join(
                f"J{i + 1}: current={current[i]:+.2f}Nm, baseline={baseline[i]:+.2f}Nm, "
                f"delta={deviation[i]:.2f}Nm, limit={limits[i]:.2f}Nm"
                for i in indices
            )
            self._safety_fault_reason = (
                f"{side} torque deviation exceeded ({details}); "
                "following frozen, arm remains enabled"
            )
            return False
        return True

    def connect(self) -> None:
        self._left.connect()
        self._right.connect()

    def enable(self) -> bool:
        self._safety_fault_reason = None
        if not self._configure_arm_collision_safety(
            self._left, "left"
        ) or not self._configure_arm_collision_safety(self._right, "right"):
            return False
        ok_left = self._left.enable()
        ok_right = self._right.enable()
        if ok_left:
            self._left.set_motion_mode(self._cfg.command_mode)
        if ok_right:
            self._right.set_motion_mode(self._cfg.command_mode)
        if not (ok_left and ok_right):
            return False
        prepared = self._capture_arm_torque_baseline(
            self._left, "left"
        ) and self._capture_arm_torque_baseline(
            self._right, "right"
        )
        if prepared:
            return True
        self._left.disable()
        self._right.disable()
        return False

    def set_speed(self, percent: int = 100) -> None:
        self._left.set_speed_percent(percent)
        self._right.set_speed_percent(percent)

    def read_physical_joints(self) -> dict[str, np.ndarray]:
        """Read physical NERO joint angles for both arms."""
        return {
            "left": self._left.get_joint_angles(),
            "right": self._right.get_joint_angles(),
        }

    def read_full_urdf_joints(self) -> np.ndarray:
        """Return the 14 full-URDF arm joint angles (IK input order)."""
        left = self._left.get_joint_angles()
        right = self._right.get_joint_angles()
        if left is None or right is None:
            return None
        return self._physical_to_full_urdf(left, right)

    def capture_session_start(self) -> bool:
        joints = self.read_physical_joints()
        if not self._valid_joints(joints["left"]) or not self._valid_joints(joints["right"]):
            return False
        if not self._within_hard_limits(joints["left"]) or not self._within_hard_limits(
            joints["right"]
        ):
            return False
        self._session_start = {
            "left": np.asarray(joints["left"], dtype=np.float64).copy(),
            "right": np.asarray(joints["right"], dtype=np.float64).copy(),
        }
        timestamps = {
            "left": self._left.last_feedback_timestamp,
            "right": self._right.last_feedback_timestamp,
        }
        if any(
            stamp is None or not np.isfinite(stamp) for stamp in timestamps.values()
        ):
            self._session_start = None
            return False
        self._last_command_feedback_timestamp = timestamps
        self._command_velocity = {
            "left": np.zeros(NERO_NUM_JOINTS, dtype=np.float64),
            "right": np.zeros(NERO_NUM_JOINTS, dtype=np.float64),
        }
        return True

    @staticmethod
    def _valid_joints(joints: np.ndarray | None) -> bool:
        if joints is None:
            return False
        values = np.asarray(joints, dtype=np.float64).reshape(-1)
        return values.shape == (NERO_NUM_JOINTS,) and bool(np.all(np.isfinite(values)))

    def _within_hard_limits(self, joints: np.ndarray) -> bool:
        values = np.asarray(joints, dtype=np.float64)
        lower = np.asarray(self._cfg.joint_lower_limits, dtype=np.float64)
        upper = np.asarray(self._cfg.joint_upper_limits, dtype=np.float64)
        return bool(np.all((values >= lower) & (values <= upper)))

    def _mapping(self, side: str) -> tuple[np.ndarray, np.ndarray]:
        directions = np.asarray(
            self._cfg.left_joint_directions if side == "left" else self._cfg.right_joint_directions,
            dtype=np.float64,
        ).reshape(-1)
        raw_offsets = (
            self._cfg.left_joint_offsets if side == "left" else self._cfg.right_joint_offsets
        )
        offsets = np.asarray(raw_offsets, dtype=np.float64).reshape(-1)
        if directions.shape != (NERO_NUM_JOINTS,) or offsets.shape != (NERO_NUM_JOINTS,):
            raise ValueError(f"{side} joint mapping must contain exactly 7 values")
        if not np.all(np.isin(directions, (-1.0, 1.0))):
            raise ValueError(f"{side} joint directions must contain only -1 or +1")
        return directions, offsets

    def _full_to_physical(self, full_urdf: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Convert 14 full-URDF joint angles to physical left/right NERO joints."""
        full_urdf = np.asarray(full_urdf, dtype=np.float64).reshape(-1)
        if full_urdf.shape != (14,) or not np.all(np.isfinite(full_urdf)):
            raise ValueError("dual-arm target must contain 14 finite joint angles")
        left_dir, left_off = self._mapping("left")
        right_dir, right_off = self._mapping("right")
        left = left_dir * full_urdf[0:7] + left_off
        right = right_dir * full_urdf[7:14] + right_off
        return left, right

    def _physical_to_full_urdf(self, left: np.ndarray, right: np.ndarray) -> np.ndarray:
        if not self._valid_joints(left) or not self._valid_joints(right):
            raise ValueError("NERO feedback must contain 7 finite angles per arm")
        left_dir, left_off = self._mapping("left")
        right_dir, right_off = self._mapping("right")
        left = (np.asarray(left, dtype=np.float64) - left_off) / left_dir
        right = (np.asarray(right, dtype=np.float64) - right_off) / right_dir
        return np.concatenate([left, right]).astype(np.float64)

    def _soft_limits_physical(self, side: str) -> tuple[np.ndarray, np.ndarray]:
        """Return the configured URDF commissioning envelope in SDK coordinates."""
        if side not in ("left", "right"):
            raise ValueError("joint soft-limit side must be left or right")
        lower_override = getattr(
            self._cfg, f"{side}_joint_soft_lower_limits_urdf", None
        )
        upper_override = getattr(
            self._cfg, f"{side}_joint_soft_upper_limits_urdf", None
        )
        lower_urdf = np.asarray(
            self._cfg.joint_soft_lower_limits_urdf
            if lower_override is None else lower_override,
            dtype=np.float64,
        ).reshape(-1)
        upper_urdf = np.asarray(
            self._cfg.joint_soft_upper_limits_urdf
            if upper_override is None else upper_override,
            dtype=np.float64,
        ).reshape(-1)
        if lower_urdf.shape != (NERO_NUM_JOINTS,) or upper_urdf.shape != (
            NERO_NUM_JOINTS,
        ):
            raise ValueError("robot joint soft-limit lists must contain exactly 7 values")
        if (
            not np.all(np.isfinite(lower_urdf))
            or not np.all(np.isfinite(upper_urdf))
            or np.any(lower_urdf >= upper_urdf)
        ):
            raise ValueError("robot URDF joint soft limits must be finite and lower < upper")

        directions, offsets = self._mapping(side)
        endpoint_a = directions * lower_urdf + offsets
        endpoint_b = directions * upper_urdf + offsets
        return np.minimum(endpoint_a, endpoint_b), np.maximum(endpoint_a, endpoint_b)

    @staticmethod
    def _joint_limit_vector(value, name: str) -> np.ndarray:
        values = np.asarray(value, dtype=np.float64).reshape(-1)
        if values.shape == (1,):
            values = np.full(NERO_NUM_JOINTS, values[0], dtype=np.float64)
        if values.shape != (NERO_NUM_JOINTS,) or not np.all(np.isfinite(values)):
            raise ValueError(f"robot.{name} must be one value or exactly 7 finite values")
        if np.any(values < 0.0):
            raise ValueError(f"robot.{name} values must be non-negative")
        return values

    def _joint_rate_limit_vector(self, side: str, name: str) -> np.ndarray:
        if side not in ("left", "right"):
            raise ValueError("joint rate-limit side must be left or right")
        override = getattr(self._cfg, f"{side}_{name}", None)
        value = getattr(self._cfg, name) if override is None else override
        return self._joint_limit_vector(value, f"{side}_{name}" if override is not None else name)

    def _feedback_command_dts(self) -> dict[str, float] | None:
        current = {
            "left": self._left.last_feedback_timestamp,
            "right": self._right.last_feedback_timestamp,
        }
        previous = self._last_command_feedback_timestamp
        dts: dict[str, float] = {}
        for side in ("left", "right"):
            now = current[side]
            before = previous.get(side)
            if (
                now is None
                or before is None
                or not np.isfinite(now)
                or not np.isfinite(before)
                or now <= before
            ):
                return None
            dts[side] = float(now - before)
        self._last_command_feedback_timestamp = current
        return dts

    def _effective_position_limits(self, side):
        lower = np.asarray(self._cfg.joint_lower_limits, dtype=np.float64)
        upper = np.asarray(self._cfg.joint_upper_limits, dtype=np.float64)
        margin = max(0.0, float(self._cfg.joint_limit_margin))
        soft_lower, soft_upper = self._soft_limits_physical(side)
        effective_lower = np.maximum(lower + margin, soft_lower)
        effective_upper = np.minimum(upper - margin, soft_upper)
        if self._session_start is not None and self._cfg.max_joint_deviation_from_start > 0.0:
            start = self._session_start[side]
            deviation = self._cfg.max_joint_deviation_from_start
            effective_lower = np.maximum(effective_lower, start - deviation)
            effective_upper = np.minimum(effective_upper, start + deviation)
        if np.any(effective_lower > effective_upper):
            raise ValueError(f"{side} robot joint safety-limit intervals do not overlap")
        return effective_lower, effective_upper

    def _clamp(self, joints: np.ndarray, current: np.ndarray, side: str, dt: float) -> np.ndarray:
        joints = np.asarray(joints, dtype=np.float64)
        effective_lower, effective_upper = self._effective_position_limits(side)
        joints = np.clip(joints, effective_lower, effective_upper)

        if not np.isfinite(dt) or dt <= 0.0:
            raise ValueError("joint rate limiting requires a positive feedback dt")
        max_step = self._joint_rate_limit_vector(side, "max_joint_step")
        max_velocity = self._joint_rate_limit_vector(side, "max_joint_velocity")
        max_acceleration = self._joint_rate_limit_vector(side, "max_joint_acceleration")

        error = joints - current
        desired_velocity = error / dt
        velocity_bounds = np.where(max_velocity > 0.0, max_velocity, np.inf)
        desired_velocity = np.clip(desired_velocity, -velocity_bounds, velocity_bounds)
        velocity_state = getattr(self, "_command_velocity", {}).get(
            side, np.zeros(NERO_NUM_JOINTS, dtype=np.float64)
        )
        acceleration_delta = np.where(max_acceleration > 0.0, max_acceleration * dt, np.inf)
        step_bounds = np.where(max_step > 0.0, max_step, np.inf)
        if not hasattr(self, "_last_rate_limit_diagnostics"):
            self._last_rate_limit_diagnostics = {}
        self._last_rate_limit_diagnostics[side] = {
            "feedback_dt_s": float(dt),
            "velocity_bound_active": bool(abs(error[3] / dt) > velocity_bounds[3]),
            "acceleration_bound_active": bool(abs(desired_velocity[3] - velocity_state[3]) > acceleration_delta[3]),
            "velocity_limit_deg_s": float(np.degrees(max_velocity[3])),
            "acceleration_limit_deg_s2": float(np.degrees(max_acceleration[3])),
            "step_limit_deg": float(np.degrees(max_step[3])),
        }

        if bool(self._cfg.synchronize_joint_motion):
            deadband = np.deg2rad(
                max(0.0, float(self._cfg.joint_sync_error_deadband_deg))
            )
            active = np.abs(error) > deadband
            inactive_can_stop = np.all(
                np.abs(velocity_state[~active]) <= acceleration_delta[~active]
            )
            progress_lower = 0.0
            progress_upper = 1.0
            for index in np.flatnonzero(active):
                velocity_lower = max(
                    -velocity_bounds[index],
                    velocity_state[index] - acceleration_delta[index],
                )
                velocity_upper = min(
                    velocity_bounds[index],
                    velocity_state[index] + acceleration_delta[index],
                )
                if np.isfinite(step_bounds[index]):
                    step_velocity = step_bounds[index] / dt
                    velocity_lower = max(velocity_lower, -step_velocity)
                    velocity_upper = min(velocity_upper, step_velocity)
                coefficient = error[index] / dt
                if coefficient > 0.0:
                    progress_lower = max(progress_lower, velocity_lower / coefficient)
                    progress_upper = min(progress_upper, velocity_upper / coefficient)
                else:
                    progress_lower = max(progress_lower, velocity_upper / coefficient)
                    progress_upper = min(progress_upper, velocity_lower / coefficient)
            feasible_progress = (
                inactive_can_stop
                and progress_upper >= 0.0
                and progress_upper + 1.0e-12 >= max(0.0, progress_lower)
            )
            if feasible_progress:
                progress = float(np.clip(progress_upper, 0.0, 1.0))
                delta = np.zeros(NERO_NUM_JOINTS, dtype=np.float64)
                delta[active] = error[active] * progress
                if not hasattr(self, "_command_velocity"):
                    self._command_velocity = {}
                self._command_velocity[side] = delta / dt
                return current + delta

        velocity = velocity_state + np.clip(
            desired_velocity - velocity_state, -acceleration_delta, acceleration_delta
        )
        delta = velocity * dt
        delta = np.clip(delta, -step_bounds, step_bounds)
        reaches_target = (delta * error > 0.0) & (np.abs(delta) > np.abs(error))
        delta[reaches_target] = error[reaches_target]
        if not hasattr(self, "_command_velocity"):
            self._command_velocity = {}
        self._command_velocity[side] = delta / dt
        return current + delta

    def _command(self, left_phys: np.ndarray, right_phys: np.ndarray) -> None:
        if self._cfg.command_mode == "js":
            self._left.move_js(left_phys.tolist())
            self._right.move_js(right_phys.tolist())
        else:
            self._left.move_j(left_phys.tolist())
            self._right.move_j(right_phys.tolist())

    def command_full_urdf(
        self, full_urdf_targets: np.ndarray, _legacy_loop_dt: float | None = None
    ) -> bool:
        """Command both arms from 14 URDF targets, or reject unsafe feedback/targets."""
        if not self._torque_is_safe(self._left, "left") or not self._torque_is_safe(
            self._right, "right"
        ):
            return False
        left_phys, right_phys = self._full_to_physical(full_urdf_targets)
        current_left = self._left.get_joint_angles()
        current_right = self._right.get_joint_angles()
        if not self._valid_joints(current_left) or not self._valid_joints(current_right):
            return False
        if not self._within_hard_limits(current_left) or not self._within_hard_limits(current_right):
            return False
        feedback_dts = self._feedback_command_dts()
        if feedback_dts is None:
            return False
        left_phys = self._clamp(left_phys, current_left, "left", feedback_dts["left"])
        right_phys = self._clamp(right_phys, current_right, "right", feedback_dts["right"])
        self._command(left_phys, right_phys)
        return True

    def emergency_stop(self) -> None:
        self._left.emergency_stop()
        self._right.emergency_stop()

    def disconnect(self) -> None:
        self._left.disconnect()
        self._right.disconnect()


class NeroSingleArmDriver(NeroDualArmDriver):
    """Drive one complete NERO arm while leaving the other CAN bus untouched."""

    def __init__(self, cfg: RobotConfig, side: str):
        if cfg.command_mode != "j":
            raise ValueError("single-arm commissioning requires smoothed command_mode='j'")
        if side not in ("left", "right"):
            raise ValueError("side must be 'left' or 'right'")
        self._cfg = cfg
        self._side = side
        self._trajectory_lock = threading.RLock()
        self._return_lock = threading.Lock()
        self._return_cancel = threading.Event()
        self._return_active = False
        self._return_pending = False
        self._return_request_id = 0
        self.last_return_result = None
        self._trajectory_fk = None
        self._trajectory_endpoint_speed = None
        self._trajectory = None
        self._backpressure = (
            CommandBackpressure(
                timeout=getattr(cfg, f"{side}_command_backpressure_timeout_s"),
                pause_after=getattr(cfg, f"{side}_command_backpressure_pause_s"),
                stationary_velocity=np.deg2rad(
                    cfg.return_stationary_velocity_deg_s
                ),
            )
            if getattr(cfg, f"{side}_command_backpressure_enabled", False) else None)
        self._move_j_waypoint_gate = (
            MoveJWaypointGate(
                minimum_step=np.deg2rad(cfg.left_move_j_waypoint_deg),
                timeout=cfg.left_command_backpressure_timeout_s,
            )
            if (side == "left"
                and str(cfg.left_tracking_control_mode).lower() == "j"
                and cfg.left_command_backpressure_enabled)
            else None
        )
        self._last_follow_wire_key = None
        self._last_follow_send_monotonic = None
        self.trajectory_diagnostics = None
        self.last_command_skipped = False
        if self.command_trajectory_enabled:
            self._trajectory = CommandTrajectory(
                self._joint_rate_limit_vector(side, "max_joint_velocity"),
                self._joint_rate_limit_vector(side, "max_joint_acceleration"),
                self._joint_rate_limit_vector(side, "max_joint_step"),
                np.deg2rad(cfg.command_trajectory_lead_deg),
                max_dt=cfg.command_trajectory_max_dt_s,
                lead_timeout=cfg.command_trajectory_lead_timeout_s,
            )
        channel = cfg.left_can_channel if side == "left" else cfg.right_can_channel
        self._arm = NeroArm(cfg, channel, side)
        self._session_start: dict[str, np.ndarray] | None = None
        self._last_command_feedback_timestamp: float | None = None
        self._command_velocity = {side: np.zeros(NERO_NUM_JOINTS, dtype=np.float64)}
        self._last_commanded_full_urdf: np.ndarray | None = None
        self._enabled = False
        self._silent_disabled_startup = False
        self._torque_baseline = {}
        self._torque_trip_counts = {side: np.zeros(7, dtype=int)}
        self._safety_fault_reason = None
        # Set only when this process issued a damped E-stop for a stalled
        # command stream. Operator E-stop and hardware faults never set it.
        self._software_estop_recovery_pending = False
        # A later process may observe the same electronic stop without its
        # in-memory origin marker. It becomes recoverable only after fresh
        # controller, seven-driver, pose and collision checks prove a fully
        # disabled arm; e never consumes this marker, explicit z/q recovery does.
        self._inherited_estop_recovery_pending = False
        self._last_valid_torque_time = {side: None}

    @property
    def command_trajectory_enabled(self):
        return bool(getattr(self._cfg, f"{self._side}_command_trajectory_enabled", False))

    def configure_command_trajectory(self, fk, endpoint_speed):
        if not callable(fk) or not np.isfinite(endpoint_speed) or endpoint_speed <= 0:
            raise ValueError("trajectory requires FK and a positive endpoint speed")
        self._trajectory_fk = fk
        self._trajectory_endpoint_speed = float(endpoint_speed)

    def configure_collision_guard(self, guard):
        if not callable(guard):
            raise ValueError("collision guard must be callable")
        self._collision_guard = guard
        if hasattr(guard, "hand_motion_horizon_s"):
            guard.hand_motion_horizon_s = (self._cfg.command_trajectory_max_dt_s + float(np.max(
                self._joint_rate_limit_vector(self._side, "max_joint_velocity") /
                self._joint_rate_limit_vector(self._side, "max_joint_acceleration"))))

    def _controller_is_safe(self, *, allow_disabled=False):
        if not self._cfg.controller_status_monitor_enabled:
            return True
        try:
            fault = self._arm.controller_fault(allow_disabled=allow_disabled)
        except Exception as exc:
            fault = {"category": "controller_read_failed", "detail": str(exc)}
        if fault is None:
            return True
        self._return_inhibited = True
        if not self._safety_fault_reason:
            self._safety_fault_reason = f"{self._side} controller safety: {fault}"
            if fault.get("category") == "controller_fault" and fault.get("arm_status") == 1:
                self._safety_fault_reason += (
                    "; EMERGENCY_STOP remains active. DISABLED confirms motor state, "
                    "not cleared E-stop. Restarting this program does not reset the controller. "
                    "Do not retry e; support the arm, resolve the stop cause and use the "
                    "manufacturer recovery procedure, then verify controller and motor state. "
                    "An E-stop inherited from another process is not automatically reset"
                )
        self._emit_probe(dict(event="controller_fault", fault_context=fault))
        return False

    def _verify_disabled_estop_recovery_candidate(self, current) -> tuple[bool, str]:
        """Recognize a lone inherited electronic stop without changing hardware."""
        values = np.asarray(current, dtype=np.float64).reshape(-1)
        if values.shape != (7,) or not np.all(np.isfinite(values)):
            return False, "missing complete seven-joint pose"
        if not self._within_hard_limits(values):
            return False, "pose outside commissioned joint envelope"
        states = self._arm.get_joint_enable_states()
        if states is None or len(states) != 7 or any(states):
            return False, "seven-joint disabled state not confirmed"
        try:
            fault = self._arm.controller_fault(allow_disabled=True)
        except Exception as exc:
            return False, f"controller state read failed: {exc}"
        if not (isinstance(fault, dict)
                and fault.get("category") == "controller_fault"
                and fault.get("arm_status") == 1
                and fault.get("err_code") == 0
                and fault.get("ctrl_mode") == 1):
            return False, "controller fault is not an isolated CAN-mode emergency stop"
        try:
            residual = self._arm.controller_fault(allow_disabled=True, allow_estop=True)
        except Exception as exc:
            return False, f"driver-state verification failed: {exc}"
        if residual is not None:
            return False, f"another controller/driver fault remains: {residual}"
        if not self._check_physical_path(values, values):
            return False, "current pose failed collision check"
        return True, ""

    def _recognize_inherited_disabled_estop(self, current) -> bool:
        verified, reason = self._verify_disabled_estop_recovery_candidate(current)
        if not verified:
            self._emit_probe(dict(event="inherited_estop_not_recoverable", reason=reason))
            return False
        self._inherited_estop_recovery_pending = True
        self._emit_probe(dict(
            event="inherited_estop_recovery_available",
            feedback_physical_rad=np.asarray(current, dtype=float).tolist(),
            disabled_verified=True,
        ))
        return True

    def _check_physical_path(self, a, b):
        if not self._cfg.torso_collision_enabled:
            return True
        guard = getattr(self, "_collision_guard", None)
        if guard is None:
            return False
        directions, offsets = self._mapping(self._side)
        start = 0 if self._side == "left" else 7
        qa, qb = np.zeros(14), np.zeros(14)
        qa[start:start+7], qb[start:start+7] = (a-offsets)/directions, (b-offsets)/directions
        return bool(guard(qa, qb))

    def _send_return_position(self, position):
        with self._trajectory_lock:
            if self._return_cancel.is_set():
                return False
            received = getattr(self, "_return_feedback_received", None)
            if received is None or not 0 <= time.monotonic() - received <= self._cfg.command_trajectory_max_dt_s:
                self._emit_probe(dict(event="return_fault", fault="feedback expired before send"))
                return False
            return self._send_return_position_locked(position)

    def _send_return_position_locked(self, position):
        try:
            if self._arm.move_j(position.tolist()) is False:
                raise ValueError("SDK rejected return command")
            return True
        except Exception as exc:
            self._return_inhibited = True
            if not self._safety_fault_reason:
                self._safety_fault_reason = f"{self._side} return command failed: {exc}"
            self._emit_probe(dict(event="return_fault", fault=self._safety_fault_reason))
            return False

    def invalidate_command_trajectory(self):
        self._cycle_feedback = None
        self._first_follow_hold = False
        if getattr(self, "_backpressure", None) is not None:
            self._backpressure.reset()
        if getattr(self, "_move_j_waypoint_gate", None) is not None:
            self._move_j_waypoint_gate.reset()
        self._last_follow_wire_key = None
        self._last_follow_send_monotonic = None
        if getattr(self, "_trajectory", None) is not None:
            with self._trajectory_lock:
                self._trajectory.invalidate()

    def _trajectory_fault(self, reason, diagnostics=None):
        first_fault = not self._safety_fault_reason
        diagnostics = dict(diagnostics or {})
        if "collision" in reason or "torso" in reason:
            self._return_inhibited = True
        if not self._safety_fault_reason:
            self._safety_fault_reason = f"{self._side} command trajectory: {reason}"
        self.invalidate_command_trajectory()
        if (first_fault and self._enabled
                and diagnostics.get("fault_code") == "persistent_feedback_lead"):
            # Logged V112 runs execute the pending target AFTER we stop sending.
            # Stopping the producer alone is not a controller stop. Do not send
            # another position, reset, or infer motor disable from this call.
            self._return_inhibited = True
            stop = {"attempted": True, "send_returned": False,
                    "disable_verified": False}
            try:
                self._arm.emergency_stop()
                stop["send_returned"] = True
                self._software_estop_recovery_pending = True
            except Exception as exc:
                stop["error"] = str(exc)
            diagnostics["controller_stop"] = stop
        self.trajectory_diagnostics = {"fault": self._safety_fault_reason}
        if diagnostics:
            self.trajectory_diagnostics["fault_context"] = diagnostics
        self._emit_probe(dict(event="trajectory_fault", **self.trajectory_diagnostics))
        # Probe capture is optional and periodic diagnostics may have dropped
        # frames. Persist the exact fault on the independent event stream.
        try:
            self.record_startup_event("trajectory_fault")
        except Exception as exc:
            self._emit_probe(dict(event="diagnostic_error", detail=str(exc)))
        return False

    def _emit_probe(self, event):
        sink = getattr(self, "probe_sink", None)
        if sink is not None:
            try:
                sink(event)
            except Exception:
                # Diagnostic failures must never alter command acceptance.
                pass

    def _trajectory_preflight(self):
        if not self.command_trajectory_enabled:
            return True
        if not callable(getattr(self, "_trajectory_fk", None)):
            return self._trajectory_fault("FK is unavailable; enable was not sent")
        if self._cfg.torso_collision_enabled and not callable(getattr(self, "_collision_guard", None)):
            return self._trajectory_fault("torso collision geometry unavailable; enable was not sent")
        try:
            points = np.asarray(self._trajectory_fk(np.zeros(14)), dtype=float)
            if points.shape != (2, 3) or not np.all(np.isfinite(points)):
                raise ValueError("invalid FK output")
        except Exception as exc:
            return self._trajectory_fault(f"FK preflight failed: {exc}")
        return True

    def initialize_command_trajectory(self, snapshot=None):
        """Call after enable/torque settling and before allowing any following."""
        if not self.command_trajectory_enabled:
            return True
        with self._trajectory_lock:
            if not self._trajectory_preflight() or self._safety_fault_reason:
                return False
            joints = self.read_joints() if snapshot is None else snapshot.position.copy()
            stamp = self._arm.last_feedback_timestamp if snapshot is None else snapshot.stamp
            if snapshot is not None and time.monotonic() - snapshot.received_monotonic > .1:
                return self._trajectory_fault("stale initialization feedback")
            if joints is None or not self._within_hard_limits(joints) or stamp is None:
                return self._trajectory_fault("missing initialization feedback")
            if not self._controller_is_safe() or not self._check_physical_path(joints, joints):
                return self._trajectory_fault("startup controller/collision check rejected")
            lower, upper = self._effective_position_limits(self._side)
            if np.any(joints < lower) or np.any(joints > upper):
                return self._trajectory_fault("initial feedback outside position envelope")
            try:
                self._trajectory.reset(joints, stamp)
                if self._backpressure is not None:
                    self._backpressure.reset()
                if self._move_j_waypoint_gate is not None:
                    self._move_j_waypoint_gate.reset()
                self._last_follow_wire_key = None
                self._last_follow_send_monotonic = None
                # Validate the FK before accepting a live session.
                directions, offsets = self._mapping(self._side)
                full = np.zeros(14)
                start = 0 if self._side == "left" else 7
                full[start:start+7] = (joints - offsets) / directions
                points = np.asarray(self._trajectory_fk(full), dtype=float)
                if points.shape != (2, 3) or not np.all(np.isfinite(points)):
                    raise ValueError("invalid FK output")
                tracking_mode = str(getattr(
                    self._cfg, f"{self._side}_tracking_control_mode", "j"
                )).lower()
                if tracking_mode == "cpv":
                    prepared = self._arm.prepare_tracking_control(
                        joints,
                        self._joint_rate_limit_vector(self._side, "max_joint_velocity"),
                        self._joint_rate_limit_vector(self._side, "max_joint_acceleration"),
                    )
                    if not prepared:
                        raise RuntimeError("CPV tracking preparation was rejected")
                    self.record_startup_event("tracking_control_ready", joints)
                    print(
                        f"[{self._side}-arm] Live tracking control: V112 CPV ready; "
                        "current pose preloaded and controller mode feedback verified.",
                        flush=True,
                    )
            except Exception as exc:
                return self._trajectory_fault(f"tracking control preparation failed: {exc}")
            self._last_command_feedback_timestamp = float(stamp)
            self._last_commanded_full_urdf = None
            self.elbow_command_diagnostics = None
            self.trajectory_diagnostics = None
            self._first_follow_hold = True
            return True

    def read_control_cycle_joints(self):
        # The strict runtime reader intentionally refuses to operate until
        # get_joint_angles() has accepted two consistent startup frames.  A
        # robot-link backend connects while disabled and has no earlier
        # calibration read to establish that snapshot, so seed it here using
        # the same read-only validation before entering the runtime path.
        if getattr(self._arm, "_runtime_feedback", None) is None:
            if self.read_joints() is None:
                self.feedback_failure_diagnostics = {
                    "reason": "no verified startup joint feedback",
                    "previous": None,
                    "transient": True,
                }
                self.record_startup_event("runtime_feedback_rejected")
                return None
        self._cycle_feedback = self._arm.runtime_feedback(
            max_age_s=self._cfg.command_trajectory_max_dt_s,
            recovery_timeout_s=self._cfg.runtime_feedback_recovery_timeout_s,
        )
        if self._cycle_feedback is None:
            failure = getattr(self._arm, "runtime_feedback_failure", None)
            self.feedback_failure_diagnostics = failure
            self.record_startup_event("runtime_feedback_rejected")
            return None
        self.feedback_failure_diagnostics = None
        if getattr(self._arm, "runtime_feedback_recovered", False) is True:
            if not self._recover_command_trajectory_after_feedback_gap(self._cycle_feedback):
                return None
        directions, offsets = self._mapping(self._side)
        full = np.zeros(14)
        start = 0 if self._side == "left" else 7
        full[start:start+7] = (self._cycle_feedback.position - offsets) / directions
        return full

    def _recover_command_trajectory_after_feedback_gap(self, snapshot):
        """Re-stamp the unsent controller target after two verified fresh frames."""
        if not self.command_trajectory_enabled or not self._enabled:
            return True
        with self._trajectory_lock:
            state = None if self._trajectory is None else self._trajectory.state
            if state is None:
                self.feedback_failure_diagnostics = {
                    "reason": "feedback returned but command trajectory is not initialized",
                    "transient": False,
                }
                return False
            current = snapshot.position
            lower, upper = self._effective_position_limits(self._side)
            lead = np.abs(state.position - current)
            if (
                not self._within_hard_limits(current)
                or np.any(current < lower)
                or np.any(current > upper)
                or np.any(lead > self._trajectory.lead + 1e-8)
                or not self._controller_is_safe()
                or not self._torque_is_safe(self._arm, self._side)
                or (self._cfg.teleop_torso_collision_enabled
                    and not self._check_physical_path(current, state.position))
            ):
                details = {
                    "reason": "feedback recovery safety recheck rejected",
                    "transient": False,
                    "lead_rad": (state.position - current).tolist(),
                    "lead_limit_rad": self._trajectory.lead.tolist(),
                    "feedback": snapshot.diagnostics(),
                }
                self.feedback_failure_diagnostics = details
                self._trajectory_fault(details["reason"], details)
                return False
            # Keep the exact last controller target.  Only its timestamp and
            # velocity state are reset; no hardware command is sent here.
            held_target = state.position.copy()
            self._trajectory.reset(held_target, snapshot.stamp)
            self._first_follow_hold = True
            self._last_command_feedback_timestamp = float(snapshot.stamp)
            self.trajectory_diagnostics = {
                "feedback_gap_recovered": True,
                "held_command_physical_rad": held_target.tolist(),
                "feedback_physical_rad": current.tolist(),
                "lead_rad": (held_target - current).tolist(),
                "feedback_snapshot": snapshot.diagnostics(),
            }
            self.record_startup_event("runtime_feedback_recovered", held_target)
            self._emit_probe(dict(event="runtime_feedback_recovered", **self.trajectory_diagnostics))
            return True

    def record_startup_event(self, phase, target=None):
        snapshot = getattr(self._arm, "_runtime_feedback", None)
        event = dict(phase=phase, timestamp=time.time(), monotonic_s=time.monotonic(),
                     feedback=snapshot.diagnostics() if isinstance(snapshot, JointFeedbackSnapshot) else None,
                     target_physical_rad=None if target is None else np.asarray(target).tolist())
        if phase == "runtime_feedback_rejected":
            event["failure"] = getattr(self, "feedback_failure_diagnostics", None)
        elif phase == "trajectory_fault":
            event.update(self.trajectory_diagnostics)
        elif phase == "startup_refused":
            event["failure"] = dict(self.last_startup_failure or {})
        elif phase.startswith("command_backpressure_") and self._backpressure is not None:
            event["command_backpressure"] = dict(self._backpressure.diagnostics)
        if phase in ("enable_confirmed", "reference_checks_passed", "tracking_control_ready",
                     "trajectory_fault", "startup_refused"):
            # Read parser caches only: no SDK queries, CAN writes or mode
            # changes. A lead timeout alone cannot distinguish stalled motion
            # from a controller/protocol/feedback-source issue.
            try:
                from ..debug.arm_probe import controller_snapshot
                event["controller_snapshot"] = controller_snapshot(self)
                event["configured_firmware"] = (
                    self._cfg.right_nero_firmware or self._cfg.nero_firmware
                    if self._side == "right" else self._cfg.nero_firmware)
                event["sdk_driver_class"] = (
                    type(self._arm._robot).__module__ + "." + type(self._arm._robot).__name__)
            except Exception as exc:
                event["controller_snapshot_error"] = str(exc)
        self.startup_events = (getattr(self, "startup_events", []) + [event])[-32:]
        sink = getattr(self, "startup_event_sink", None)
        if sink is not None:
            sink(event)

    def connect(self) -> None:
        self._arm.connect()
        disable_on_connect = (
            self._cfg.left_arm_disable_on_connect
            if self._side == "left"
            else self._cfg.right_arm_disable_on_connect
        )
        # Both controllers may boot with active CAN reporting disabled. Asking
        # for telemetry does not enable motors or send a position target.
        self._arm.request_can_feedback_push()
        feedback_deadline = time.monotonic() + 0.75
        states = self._arm.get_joint_enable_states()
        while states is None and time.monotonic() < feedback_deadline:
            time.sleep(0.05)
            states = self._arm.get_joint_enable_states()
        allow_silent = (
            self._side == "left"
            and bool(self._cfg.left_arm_allow_silent_disabled_startup)
        )
        if states is None and allow_silent:
            self._silent_disabled_startup = True
            self._enabled = False
            return
        if disable_on_connect and not self._arm.disable():
            raise RuntimeError(f"{self._side} NERO did not confirm all-joint disable")
        self._silent_disabled_startup = False

    @property
    def silent_disabled_startup(self) -> bool:
        return bool(self._silent_disabled_startup)

    def _refuse_silent_startup(self, stage, reason, *, wake_attempted=False,
                               disable_attempted=False, geometry=None,
                               power_cycle_required=False,
                               transport_unavailable=False):
        self.last_startup_failure = dict(stage=stage, reason=str(reason),
            wake_attempted=bool(wake_attempted), disable_attempted=bool(disable_attempted),
            geometry=dict(geometry or {}),
            power_cycle_required=bool(power_cycle_required),
            transport_unavailable=bool(transport_unavailable))
        self._emit_probe(dict(event="startup_refused", **self.last_startup_failure))
        self.record_startup_event("startup_refused")
        return False

    def enable_from_natural_down(self) -> bool:
        """Enable a silent left arm at URDF zero, then verify fresh feedback."""
        self.last_startup_failure = None
        self.invalidate_command_trajectory()
        if not self._trajectory_preflight() or not self._silent_disabled_startup:
            return self._refuse_silent_startup("preflight",
                self._safety_fault_reason or "silent startup preconditions not met")
        _, zero_physical = self._mapping(self._side)
        if not self._check_physical_path(zero_physical, zero_physical):
            context = dict(getattr(getattr(self, "_collision_guard", None), "last_diagnostics", {}) or {})
            self._trajectory_fault("startup zero safety check rejected", context)
            return self._refuse_silent_startup("geometry", "startup zero safety check rejected",
                                              geometry=context)
        try:
            joints, states = self._arm.enable_at_known_target(
                zero_physical,
                speed_percent=self._cfg.speed_percent,
            )
        except RuntimeError as exc:
            if "CAN socket send failed" not in str(exc):
                raise
            self._enabled = False
            return self._refuse_silent_startup(
                "can_transport",
                "CAN transmit failed during the bounded wake; no controller ACK/feedback "
                f"was available ({exc})",
                wake_attempted=True,
                power_cycle_required=True,
                transport_unavailable=True,
            )
        tolerance = np.deg2rad(
            max(0.1, float(self._cfg.full_arm_startup_zero_tolerance_deg))
        )
        valid = (
            self._valid_joints(joints)
            and states is not None
            and len(states) == NERO_NUM_JOINTS
            and all(states)
            and self._within_hard_limits(np.asarray(joints, dtype=np.float64))
            and np.max(np.abs(np.asarray(joints, dtype=np.float64) - zero_physical)) <= tolerance
        )
        if not valid:
            feedback_complete = (
                self._valid_joints(joints)
                and states is not None
                and len(states) == NERO_NUM_JOINTS
            )
            self._arm.best_effort_disable_once()
            self._enabled = False
            return self._refuse_silent_startup("wake_feedback",
                "wake feedback incomplete, motors not all enabled, or zero/limits not verified",
                wake_attempted=True, disable_attempted=True,
                power_cycle_required=not feedback_complete)
        self._enabled = True

        # V1.12 cannot provide the disabled-state readback needed to configure
        # collision protection until one bounded zero-target wake has opened
        # CAN feedback. Verify that wake, disable again, configure/read back the
        # same protection as the right arm, then re-arm from the same zero.
        if self._collision_enabled_for(self._side):
            if not self._arm.disable():
                self._arm.best_effort_disable_once()
                self._enabled = False
                return self._refuse_silent_startup("disable_verification",
                    "disable after first wake not confirmed", wake_attempted=True, disable_attempted=True)
            self._enabled = False
            self._arm.request_can_feedback_push()
            if not self._configure_arm_collision_safety(
                self._arm, self._side, disabled_verified=True
            ):
                self._arm.best_effort_disable_once()
                return self._refuse_silent_startup("collision_configuration",
                    "controller collision protection configuration/readback failed",
                    wake_attempted=True, disable_attempted=True)
            try:
                joints, states = self._arm.enable_at_known_target(
                    zero_physical,
                    speed_percent=self._cfg.speed_percent,
                )
            except RuntimeError as exc:
                if "CAN socket send failed" not in str(exc):
                    raise
                self._enabled = False
                return self._refuse_silent_startup(
                    "can_transport",
                    "CAN transmit failed during the verified re-arm; controller state "
                    f"is no longer observable ({exc})",
                    wake_attempted=True,
                    power_cycle_required=True,
                    transport_unavailable=True,
                )
            valid = (
                self._valid_joints(joints)
                and states is not None
                and len(states) == NERO_NUM_JOINTS
                and all(states)
                and self._within_hard_limits(np.asarray(joints, dtype=np.float64))
                and np.max(np.abs(np.asarray(joints, dtype=np.float64) - zero_physical))
                <= tolerance
            )
            if not valid:
                feedback_complete = (
                    self._valid_joints(joints)
                    and states is not None
                    and len(states) == NERO_NUM_JOINTS
                )
                self._arm.best_effort_disable_once()
                return self._refuse_silent_startup("rearm_feedback",
                    "second wake feedback or zero verification failed",
                    wake_attempted=True, disable_attempted=True,
                    power_cycle_required=not feedback_complete)
            self._enabled = True

        self._silent_disabled_startup = False
        # The wake-up handshake is deliberately capped at 10%. Restore the
        # configured tracking speed only after zero feedback is verified.
        self._arm.set_speed_percent(self._cfg.speed_percent)
        self._arm.move_j(zero_physical.tolist())
        if self._capture_arm_torque_baseline(self._arm, self._side):
            return True
        disabled = self._arm.disable()
        self._enabled = not disabled
        return self._refuse_silent_startup("torque_baseline",
            self._safety_fault_reason or "startup torque baseline not verified",
            wake_attempted=True, disable_attempted=True)

    def read_joints(self) -> np.ndarray | None:
        joints = self._arm.get_joint_angles()
        if not self._valid_joints(joints):
            return None
        self._return_feedback_received = time.monotonic()
        return np.asarray(joints, dtype=np.float64)

    def read_full_urdf_joints(self) -> np.ndarray | None:
        physical = self.read_joints()
        if physical is None:
            return None
        directions, offsets = self._mapping(self._side)
        side_urdf = (physical - offsets) / directions
        full = np.zeros(2 * NERO_NUM_JOINTS, dtype=np.float64)
        full[:NERO_NUM_JOINTS] = side_urdf if self._side == "left" else 0.0
        full[NERO_NUM_JOINTS:] = side_urdf if self._side == "right" else 0.0
        return full

    def startup_zero_error(self) -> np.ndarray | None:
        physical = self.read_joints()
        if physical is None:
            return None
        _, offsets = self._mapping(self._side)
        return physical - offsets

    def capture_session_start(self) -> bool:
        self.invalidate_command_trajectory()
        joints = self.read_joints()
        if joints is None or not self._within_hard_limits(joints):
            return False
        stamp = self._arm.last_feedback_timestamp
        if stamp is None or not np.isfinite(stamp):
            return False
        self._session_start = {self._side: joints.copy()}
        self._last_command_feedback_timestamp = float(stamp)
        self._command_velocity[self._side] = np.zeros(NERO_NUM_JOINTS, dtype=np.float64)
        return True

    def enable(self, *, cancelled=None) -> bool:
        self.last_startup_failure = None
        if not self._return_active:
            self._return_cancel.clear()
        self.invalidate_command_trajectory()
        if not self._trajectory_preflight():
            return False
        joints = self.read_joints()
        if joints is None:
            return False
        if not self._controller_is_safe(allow_disabled=True):
            if self._recognize_inherited_disabled_estop(joints):
                self._safety_fault_reason += (
                    "; fresh feedback, collision geometry and all seven disabled states "
                    "were verified. Press z once for checked controller recovery and "
                    "collision-planned zero return; e will not clear the stop"
                )
            return self._refuse_silent_startup("controller_preflight", self._safety_fault_reason)
        if not self._check_physical_path(joints, joints):
            context = dict(getattr(getattr(self, "_collision_guard", None), "last_diagnostics", {}) or {})
            self._trajectory_fault("startup measured-pose safety check rejected", context)
            return self._refuse_silent_startup("geometry", "startup measured-pose safety check rejected",
                                              geometry=context)
        self.record_startup_event("before_enable", joints)
        self._safety_fault_reason = None
        if not self._configure_arm_collision_safety(self._arm, self._side):
            return False
        if cancelled is not None and cancelled():
            return False
        # Retain the measured-pose preload, but disabled-time commands may be
        # ignored. Confirm enable and reapply mode/hold below; the preload alone
        # is not proof that the controller replaced a previous session's target.
        self._arm.set_normal_mode()
        self._arm.set_motion_mode(self._cfg.command_mode)
        if cancelled is not None and cancelled():
            return False
        self._arm.move_j(joints.tolist())
        self.record_startup_event("position_preloaded", joints)
        enabled = self._arm.enable() if cancelled is None else self._arm.enable(cancelled=cancelled)
        if not enabled:
            return False
        if cancelled is not None and cancelled():
            return False
        states = self._arm.get_joint_enable_states()
        if states is None or len(states) != NERO_NUM_JOINTS or not all(states):
            self._arm.disable()
            return False
        self._enabled = True
        self.record_startup_event("enable_confirmed", joints)
        if cancelled is not None and cancelled():
            return False
        self._arm.move_j(joints.tolist())
        baseline_ok = (
            self._capture_arm_torque_baseline(self._arm, self._side)
            if cancelled is None else
            self._capture_arm_torque_baseline(self._arm, self._side, cancelled=cancelled)
        )
        if cancelled is not None and cancelled():
            # The gateway owns the stop policy; cancellation must not silently
            # remove motor support or start a return.
            return False
        if baseline_ok:
            self.record_startup_event("torque_settled", joints)
            return True
        disabled = self._arm.disable()
        self._enabled = not disabled
        return False

    def align_startup_zero_and_disable(self) -> tuple[bool, bool]:
        """Same preflight and planned recovery as an explicit operator return."""
        result = self.safe_return(recover=True)
        return result.returned, result.disabled

    def set_speed(self, percent: int = 100) -> None:
        self._arm.set_speed_percent(percent)

    def command_full_urdf(
        self, full_urdf_targets: np.ndarray, _legacy_loop_dt: float | None = None
    ) -> bool:
        if self.command_trajectory_enabled:
            with self._trajectory_lock:
                if self._return_active or self._return_pending or self._return_cancel.is_set():
                    self.last_command_skipped = True
                    return False
                return self._command_trajectory_full_urdf(full_urdf_targets)
        if not self._torque_is_safe(self._arm, self._side):
            return False
        targets = np.asarray(full_urdf_targets, dtype=np.float64).reshape(-1)
        if targets.shape != (2 * NERO_NUM_JOINTS,) or not np.all(np.isfinite(targets)):
            return False
        side_slice = slice(0, NERO_NUM_JOINTS) if self._side == "left" else slice(NERO_NUM_JOINTS, 14)
        directions, offsets = self._mapping(self._side)
        target_physical = directions * targets[side_slice] + offsets
        current = self.read_joints()
        stamp = self._arm.last_feedback_timestamp
        previous = self._last_command_feedback_timestamp
        if (
            current is None
            or not self._within_hard_limits(current)
            or stamp is None
            or previous is None
            or not np.isfinite(stamp)
            or stamp <= previous
        ):
            return False
        self._last_command_feedback_timestamp = float(stamp)
        limited = self._clamp(target_physical, current, self._side, float(stamp - previous))
        self._arm.move_j(limited.tolist())
        full = np.zeros(2 * NERO_NUM_JOINTS, dtype=np.float64)
        side_urdf = (limited - offsets) / directions
        full[side_slice] = side_urdf
        self._last_commanded_full_urdf = full
        self.elbow_command_diagnostics = {
            **getattr(self, "_last_rate_limit_diagnostics", {}).get(self._side, {}),
            "sent_monotonic_s": time.monotonic(),
            "feedback_j4_deg_at_send": float(np.degrees((current[3] - offsets[3]) / directions[3])),
            "command_step_deg": float(np.degrees((limited[3] - current[3]) / directions[3])),
            "requested_j4_deg": float(np.degrees(targets[side_slice][3])),
            "sent_j4_deg": float(np.degrees(side_urdf[3])),
            "driver_limit_active": bool(abs(side_urdf[3] - targets[side_slice][3]) > 1e-6),
        }
        return True

    def hold_command_trajectory(self) -> bool:
        """Brake to a controlled stop and retain the last commanded position."""
        if not self.command_trajectory_enabled:
            return False
        with self._trajectory_lock:
            if self._return_active or self._return_pending or self._return_cancel.is_set():
                self.last_command_skipped = True
                return False
            held = self.last_commanded_full_urdf()
            if held is None:
                return self._trajectory_fault("position-limit hold has no prior command")
            return self._command_trajectory_full_urdf(held, brake_only=True)

    def _command_trajectory_full_urdf(self, targets, *, brake_only=False):
        self.last_command_skipped = False
        self.last_command_skip_reason = None
        if self._safety_fault_reason or not self._enabled:
            return self._trajectory_fault(self._safety_fault_reason or "arm is not enabled")
        try:
            if not self._controller_is_safe():
                self.invalidate_command_trajectory()
                return False
            if not self._torque_is_safe(self._arm, self._side):
                self._return_inhibited = True
                self._emit_probe(dict(event="torque_fault", fault=self._safety_fault_reason))
                self.invalidate_command_trajectory()
                return False
            targets = np.asarray(targets, dtype=float)
            if targets.shape != (14,) or not np.all(np.isfinite(targets)):
                raise TrajectoryFault("invalid IK target")
            snapshot = getattr(self, "_cycle_feedback", None)
            self._cycle_feedback = None
            if snapshot is not None:
                if time.monotonic() - snapshot.received_monotonic > .1:
                    raise TrajectoryFault("control-cycle feedback expired before send")
                current, stamp = snapshot.position, snapshot.stamp
            else:
                current = self.read_joints()
                stamp = self._arm.last_feedback_timestamp
            if current is None or not self._within_hard_limits(current) or stamp is None:
                raise TrajectoryFault("missing or invalid feedback")
            directions, offsets = self._mapping(self._side)
            start = 0 if self._side == "left" else 7
            side_slice = slice(start, start+7)
            def fk(physical):
                full = np.zeros(14)
                full[side_slice] = (physical - offsets) / directions
                return self._trajectory_fk(full)
            lower, upper = self._effective_position_limits(self._side)
            physical_target = directions * targets[side_slice] + offsets
            if np.any(physical_target < lower - 1e-8) or np.any(physical_target > upper + 1e-8):
                raise TrajectoryFault("IK geometry outside commissioned joint envelope; refusing independent joint clipping")
            first_hold = getattr(self, "_first_follow_hold", False)
            if first_hold:
                physical_target = self._trajectory.state.position.copy()
            if brake_only:
                # Decelerate the host trajectory from its actual velocity; a
                # silent send gap would leave MOVE_J executing the old target
                # and would also make the next feedback dt stale.
                physical_target = self._trajectory.state.position.copy()
            brake_for_feedback = bool(brake_only)
            tracking_mode = str(getattr(
                self._cfg, f"{self._side}_tracking_control_mode", "j"
            )).lower()
            if self._move_j_waypoint_gate is not None and not brake_only:
                # MOVE_J is a stop-and-wait point-to-point interface on the
                # commissioned left controller.  While its last transmitted
                # waypoint is pending, keep the host trajectory at that exact
                # waypoint.  Advancing a private, unsent braking trajectory
                # here made the next transmitted waypoint depend on hidden
                # state; when the body target changed it could reverse several
                # joints and produce the observed back-and-forth motion.
                pending = self._move_j_waypoint_gate.pending
                if pending is not None:
                    waiting = self._move_j_waypoint_gate.evaluate(current, stamp)
                    held = (self._move_j_waypoint_gate.pending.copy()
                            if waiting else current.copy())
                    self._trajectory.reset(held, stamp)
                    self._last_command_feedback_timestamp = float(stamp)
                    full = np.zeros(14)
                    full[side_slice] = (held - offsets) / directions
                    self._last_commanded_full_urdf = full
                    self._command_velocity[self._side] = np.zeros(7)
                    self.last_command_skipped = True
                    self.last_command_skip_reason = (
                        "move_j_waypoint_pending" if waiting
                        else "move_j_waypoint_settled"
                    )
                    self.trajectory_diagnostics = dict(
                        torso_collision_check_active=bool(
                            self._cfg.torso_collision_enabled
                            and self._cfg.teleop_torso_collision_enabled
                        ),
                        ik_urdf_rad=targets[side_slice].tolist(),
                        trajectory_urdf_rad=full[side_slice].tolist(),
                        command_urdf_rad=full[side_slice].tolist(),
                        velocity_urdf_rad_s=[0.0] * 7,
                        feedback_urdf_rad=((current-offsets)/directions).tolist(),
                        sent_monotonic_s=self._last_follow_send_monotonic,
                        position_frame_sent=False,
                        command_backpressure=None,
                        move_j_waypoint_gate=dict(
                            self._move_j_waypoint_gate.diagnostics
                        ),
                        first_follow_hold=False,
                        feedback_snapshot=(None if snapshot is None
                                           else snapshot.diagnostics()),
                        move_j_host_state=("waiting_at_transmitted_waypoint"
                                           if waiting
                                           else "reanchored_after_arrival"),
                    )
                    self._emit_probe(dict(event="command_held",
                                         **self.trajectory_diagnostics))
                    return True
            elif self._backpressure is not None:
                was_paused = self._backpressure.paused
                feedback_brake = self._backpressure.evaluate(self._trajectory.state, current, stamp)
                brake_for_feedback = brake_for_feedback or feedback_brake
                if was_paused != feedback_brake:
                    event = "command_backpressure_wait" if feedback_brake else "command_backpressure_resumed"
                    self._emit_probe(dict(event=event, **self._backpressure.diagnostics))
                    try:
                        self.record_startup_event(event)
                    except Exception as exc:
                        self._emit_probe(dict(event="diagnostic_error", detail=str(exc)))
                    print(f"[{self._side}-arm] Command feedback pacing: " + (
                        "braking target growth, then waiting for measured progress."
                        if brake_for_feedback else "feedback caught up and settled; resuming latest target."), flush=True)
            proposed = self._trajectory.propose(
                physical_target, current, stamp,
                lower, upper, fk, self._trajectory_endpoint_speed,
                path_check=(self._check_physical_path if self._cfg.teleop_torso_collision_enabled else None),
                coordinate_elbow=True,
                brake_only=brake_for_feedback,
            )
            if proposed is None:
                self.last_command_skipped = True
                self.last_command_skip_reason = "duplicate_feedback"
                return True  # Duplicate feedback is a no-op, not a new command.
            if snapshot is not None and time.monotonic() - snapshot.received_monotonic > .1:
                raise TrajectoryFault("control-cycle feedback expired during trajectory/collision checks")
            # Commit nothing until the SDK call has returned successfully.
            wire_key = joint_wire_key(proposed.position)
            send_position = (self._backpressure is None or first_hold
                             or wire_key != self._last_follow_wire_key)
            if self._move_j_waypoint_gate is not None:
                send_position = self._move_j_waypoint_gate.should_send(
                    proposed.position, current, force=first_hold
                )
            if send_position:
                result = (
                    self._arm.move_tracking_positions(proposed.position.tolist())
                    if tracking_mode == "cpv"
                    else self._arm.move_j(proposed.position.tolist())
                )
                if result is False:
                    raise TrajectoryFault("SDK rejected position command")
                self._last_follow_wire_key = wire_key
                self._last_follow_send_monotonic = time.monotonic()
                if self._move_j_waypoint_gate is not None:
                    self._move_j_waypoint_gate.mark_sent(proposed.position, current, stamp)
            else:
                # Only time/diagnostics progress; the controller already holds
                # exactly these quantized position fields. Do not restart MOVE J.
                self.last_command_skipped = True
                self.last_command_skip_reason = "duplicate_position"
            self._trajectory.commit(proposed)
            self._first_follow_hold = False
            if first_hold:
                self.record_startup_event("first_follow_hold", proposed.position)
            self._last_command_feedback_timestamp = float(stamp)
            full = np.zeros(14)
            full[side_slice] = (proposed.position - offsets) / directions
            self._last_commanded_full_urdf = full
            self._command_velocity[self._side] = proposed.velocity.copy()
            self.trajectory_diagnostics = dict(proposed.diagnostics)
            self.trajectory_diagnostics.update(
                torso_collision_check_active=bool(self._cfg.torso_collision_enabled
                                                 and self._cfg.teleop_torso_collision_enabled),
                ik_urdf_rad=targets[side_slice].tolist(),
                trajectory_urdf_rad=full[side_slice].tolist(),
                command_urdf_rad=full[side_slice].tolist(),
                velocity_urdf_rad_s=(proposed.velocity / directions).tolist(),
                feedback_urdf_rad=((current-offsets)/directions).tolist(),
                sent_monotonic_s=self._last_follow_send_monotonic,
                position_frame_sent=send_position,
                command_backpressure=None if self._backpressure is None else dict(self._backpressure.diagnostics),
                move_j_waypoint_gate=(None if self._move_j_waypoint_gate is None
                                      else dict(self._move_j_waypoint_gate.diagnostics)),
                first_follow_hold=first_hold,
                position_limit_hold=bool(brake_only),
                feedback_snapshot=None if snapshot is None else snapshot.diagnostics(),
            )
            self._emit_probe(dict(event="command_sent" if send_position else "command_held",
                                 **self.trajectory_diagnostics))
            self.elbow_command_diagnostics = {
                "feedback_dt_s": proposed.diagnostics["feedback_dt_s"],
                "velocity_bound_active": proposed.diagnostics["velocity_bound_active"][3],
                "acceleration_bound_active": proposed.diagnostics["acceleration_bound_active"][3],
                "velocity_limit_deg_s": float(np.degrees(self._trajectory.vmax[3])),
                "acceleration_limit_deg_s2": float(np.degrees(self._trajectory.amax[3])),
                "step_limit_deg": float(np.degrees(self._trajectory.step[3])),
                "sent_monotonic_s": self.trajectory_diagnostics["sent_monotonic_s"],
                "feedback_j4_deg_at_send": float(np.degrees((current[3] - offsets[3]) / directions[3])),
                "command_step_deg": float(np.degrees((proposed.position[3] - current[3]) / directions[3])),
                "requested_j4_deg": float(np.degrees(targets[start+3])),
                "sent_j4_deg": float(np.degrees(full[start+3])),
                "driver_limit_active": bool(abs(targets[start+3] - full[start+3]) > 1e-6),
            }
            return True
        except Exception as exc:
            details = dict(getattr(exc, "diagnostics", {}) or {})
            guard = getattr(self, "_collision_guard", None)
            if (self._cfg.teleop_torso_collision_enabled
                    and guard is not None and hasattr(guard, "last_diagnostics")):
                details["collision"] = guard.last_diagnostics
            return self._trajectory_fault(str(exc), details)

    def last_commanded_full_urdf(self) -> np.ndarray | None:
        value = self._last_commanded_full_urdf
        return None if value is None else value.copy()

    def latch_return_fault(self, reason):
        self._return_inhibited = True
        if not self._safety_fault_reason:
            self._safety_fault_reason = str(reason)
        self.invalidate_command_trajectory()

    def prepare_return(self):
        with self._trajectory_lock:
            if self._return_active or self._return_pending:
                return None
            self._return_request_id += 1
            self._return_pending = True
            self._return_cancel.clear()
            return self._return_request_id

    def cancel_return(self):
        # Set before taking the send lock; no queued command may pass E-stop.
        self._return_cancel.set()

    def _enable_for_return(self):
        self._last_return_enable_failure = None

        def refuse(reason, **details):
            self._last_return_enable_failure = str(reason)
            self._emit_probe(dict(
                event="return_enable_failed",
                reason=self._last_return_enable_failure,
                cancelled=self._return_cancel.is_set(),
                **details,
            ))
            return False

        current = self.read_joints()
        if current is None:
            return refuse("missing seven-joint feedback before recovery enable")
        if not self._within_hard_limits(current):
            return refuse("measured pose outside commissioned joint envelope")
        if not self._controller_is_safe(allow_disabled=True):
            return refuse("controller precheck rejected recovery enable")
        if not self._check_physical_path(current, current):
            return refuse("measured pose failed collision check before recovery enable")
        if not self._configure_arm_collision_safety(self._arm, self._side):
            return refuse("controller collision-rating configuration failed")

        # Collision-rating configuration may take longer than the 100 ms
        # command-feedback horizon.  The old sequence reused the sample from
        # before that configuration, so a safe stationary arm was rejected by
        # _send_return_position() before any enable request was sent.  Acquire
        # a new sample and verify that the arm did not move before preloading
        # the measured hold target.
        configured = self.read_joints()
        if configured is None:
            return refuse("feedback missing after collision-rating configuration")
        if not self._within_hard_limits(configured):
            return refuse("post-configuration pose outside commissioned joint envelope")
        movement = float(np.max(np.abs(configured - current)))
        if movement > np.deg2rad(.2):
            return refuse(
                "arm moved during recovery-enable configuration",
                movement_deg=float(np.degrees(movement)),
            )
        if (not self._controller_is_safe(allow_disabled=True)
                or not self._check_physical_path(configured, configured)):
            return refuse("post-configuration controller or collision check failed")
        current = configured
        with self._trajectory_lock:
            if self._return_cancel.is_set():
                return refuse("cancelled before measured-pose preload")
            self._arm.set_normal_mode()
            self._arm.set_motion_mode(self._cfg.command_mode)
            if not self._send_return_position(current):
                return refuse("fresh measured-pose preload was rejected")
        enabled = self._arm.enable(cancelled=self._return_cancel.is_set,
                                   send_lock=self._trajectory_lock)
        # Do not preload again after E-stop, or infer enabled solely from SDK.
        if self._return_cancel.is_set():
            return refuse("cancelled during controller enable")
        states = self._arm.get_joint_enable_states()
        self._enabled = states is not None and len(states) == 7 and all(states)
        if not enabled:
            return refuse("controller enable timed out or SDK rejected enable",
                          enable_states=states)
        if not self._enabled:
            return refuse("seven-joint enabled state was not confirmed",
                          enable_states=states)
        with self._trajectory_lock:
            if self._return_cancel.is_set():
                return refuse("cancelled before post-enable mode configuration")
            # Disabled-time configuration is not proof of an applied mode.
            # Reapply once after enable, before checking the recovery mode;
            # waypoint positions remain behind the existing path checks.
            self._arm.set_motion_mode(self._cfg.command_mode)
        if not self._controller_is_safe():
            return refuse("controller safety check failed after enable",
                          enable_states=states)
        if not self._capture_arm_torque_baseline(self._arm, self._side):
            return refuse("stationary torque baseline was not verified",
                          enable_states=states)
        self._emit_probe(dict(event="return_enable_verified", enable_states=states))
        return True

    def _return_stationary(self, timeout=2.0, allow_disabled=False):
        deadline = time.monotonic() + timeout
        previous = None
        count = 0
        threshold = np.deg2rad(self._cfg.return_stationary_velocity_deg_s)
        while time.monotonic() < deadline and not self._return_cancel.is_set():
            if not self._controller_is_safe(allow_disabled=allow_disabled):
                return None
            q = self.read_joints()
            stamp = self._arm.last_feedback_timestamp
            if q is None or stamp is None or not np.isfinite(stamp):
                return None
            if previous is not None and stamp > previous[1]:
                velocity = np.abs(q-previous[0])/(stamp-previous[1])
                count = count+1 if (stamp-previous[1] <= self._cfg.command_trajectory_max_dt_s
                    and np.max(velocity) <= threshold) else 0
                if count >= max(3, self._cfg.return_verify_samples):
                    return q.copy()
            if previous is None or stamp > previous[1]:
                previous = (q.copy(), stamp)
            time.sleep(.01)
        return None

    def return_to_zero_and_disable(self) -> tuple[bool, bool]:
        """Compatibility entry point: planned normal return; never clears faults."""
        result = self.safe_return()
        return result.returned, result.disabled

    def safe_return(self, *, recover=False, request_id=None):
        from .return_planner import ReturnPlanner, ReturnResult
        if not self._return_lock.acquire(blocking=False):
            return ReturnResult(False, False, "busy", "return already running")
        self._return_execution_failure = None
        if request_id is None:
            request_id = self.prepare_return()
        stage = "preflight"
        disabled_confirmed = False
        def finish(returned=False, reason="", disabled=None):
            if disabled is None:
                disabled = disabled_confirmed
            result = ReturnResult(returned, disabled, stage, reason)
            self.last_return_result = result
            if not returned:
                self._return_inhibited = True
            self._emit_probe(dict(event="return_result", **result.__dict__,
                collision=getattr(getattr(self, "_collision_guard", None), "last_diagnostics", None)))
            print(f"[{self._side}-arm] RETURN {stage}: {reason or 'zero verified'}; "
                  f"returned={returned}, disabled={disabled}", flush=True)
            if not returned:
                details = getattr(getattr(self, "_collision_guard", None), "last_diagnostics", None)
                if details:
                    print(f"[{self._side}-arm] Last geometry check: {details}", flush=True)
            return result
        try:
            with self._trajectory_lock:
                if (request_id is None or request_id != self._return_request_id
                        or self._return_cancel.is_set()):
                    return finish(reason="cancelled before start")
                self._return_pending = False
                self._return_active = True
                self._return_start_state = None if self._trajectory is None else self._trajectory.state
                self.invalidate_command_trajectory()
            if not self._cfg.torso_collision_enabled or not self._cfg.controller_status_monitor_enabled:
                return finish(reason="return requires collision and controller monitoring enabled")
            if not 0 < self._cfg.return_speed_scale <= 1:
                return finish(reason="invalid return speed scale")
            if not callable(self._trajectory_fk) or not callable(getattr(self, "_collision_guard", None)):
                return finish(reason="return requires FK and collision geometry")
            if (getattr(self, "_return_inhibited", False) or self._safety_fault_reason) and not recover:
                return finish(reason="fault latched; clear physical fault then use z")
            software_stop = getattr(self, "_software_estop_recovery_pending", False)
            inherited_stop = getattr(self, "_inherited_estop_recovery_pending", False)
            if recover and (software_stop or inherited_stop):
                # A same-process guard stop carries an origin marker. A stop
                # inherited across restart must pass the stricter read-only
                # recognition checks before explicit z/q may clear it.
                stage = ("recovering software stop" if software_stop
                         else "recovering verified inherited stop")
                states = self._arm.get_joint_enable_states()
                current = self.read_joints()
                if states is None or len(states) != 7 or current is None:
                    return finish(reason="E-stop recovery lacks complete joint/enable feedback")
                if any(states) and not all(states):
                    return finish(reason="E-stop recovery found mixed motor enable state")
                disabled_confirmed = not any(states)
                if not self._check_physical_path(current, current):
                    return finish(reason="current pose unsafe/unverified; manually remove contact")
                if inherited_stop:
                    verified, candidate_reason = self._verify_disabled_estop_recovery_candidate(current)
                    if not verified:
                        return finish(reason=(
                            "inherited emergency stop no longer satisfies recovery checks: "
                            + candidate_reason
                        ))
                if any(states) and not self._arm.disable():
                    return finish(reason="could not verify seven-joint disable before controller reset")
                states = self._arm.get_joint_enable_states()
                if states is None or len(states) != 7 or any(states):
                    return finish(reason="seven-joint disable not confirmed before controller reset")
                disabled_confirmed = True
                self._enabled = False
                before_reset = current.copy()
                if self._return_cancel.is_set():
                    return finish(reason="cancelled before controller reset")
                self._arm.reset()
                deadline = time.monotonic() + 2.0
                recovered = False
                while time.monotonic() < deadline and not self._return_cancel.is_set():
                    controller_fault = self._arm.controller_fault(allow_disabled=True)
                    states = self._arm.get_joint_enable_states()
                    if (controller_fault is None and states is not None
                            and len(states) == 7 and not any(states)):
                        recovered = True
                        break
                    time.sleep(0.02)
                if not recovered:
                    return finish(reason="controller did not return to a verified disabled normal state after reset")
                after_reset = self.read_joints()
                if (after_reset is None
                        or np.max(np.abs(after_reset-before_reset)) > np.deg2rad(.2)
                        or not self._check_physical_path(after_reset, after_reset)):
                    return finish(reason="pose moved or became unsafe while clearing E-stop")
                self._software_estop_recovery_pending = False
                self._inherited_estop_recovery_pending = False
                self._emit_probe(dict(
                    event=("software_stop_recovered_for_return" if software_stop
                           else "inherited_stop_recovered_for_return"),
                    feedback_physical_rad=after_reset.tolist(),
                    disabled_verified=True,
                ))
            if not self._controller_is_safe(allow_disabled=recover):
                return finish(reason="controller fault remains active")
            states = self._arm.get_joint_enable_states()
            if states is None or len(states) != 7 or (any(states) and not all(states)):
                return finish(reason="unknown or mixed motor enable state")
            self._enabled = all(states)
            disabled_confirmed = not any(states)
            if not self._enabled and not recover:
                return finish(reason="arm disabled; explicit recovery required")
            current = self.read_joints()
            if current is None or not self._check_physical_path(current, current):
                return finish(reason="current pose unsafe/unverified; manually remove contact")
            # A disabled startup-recovery arm has no meaningful enabled-state
            # torque baseline yet. Plan while disabled, then _enable_for_return
            # captures a stationary baseline before the first return motion.
            # Enabled returns must already have a valid baseline.
            if self._enabled and not self._torque_is_safe(self._arm, self._side):
                return finish(reason="torque fault remains active")
            directions, zero = self._mapping(self._side)
            indices = np.arange(7)
            sl = slice(0, 7) if self._side == "left" else slice(7, 14)
            tolerance = np.deg2rad(self._cfg.full_arm_return_tolerance_deg)
            timeout = self._cfg.full_arm_return_timeout_s
            stage = "braking"
            self._emit_probe(dict(event="return_stage", stage=stage))
            initial = self._return_start_state
            if self._enabled and initial is not None and not recover:
                target = np.zeros(14)
                target[sl] = (initial.position-zero)/directions
                if not self._return_group_to_zero(target, indices, initial.position,
                        tolerance, timeout, "braking", brake_only=True):
                    return finish(reason="controlled braking failed")
            elif self._enabled and not recover and self._last_commanded_full_urdf is not None:
                return finish(reason="outstanding command state unavailable; use explicit recovery")
            current = self._return_stationary(allow_disabled=not self._enabled)
            if current is None:
                return finish(reason="fresh stationary feedback not confirmed")
            stage = "planning"
            self._emit_probe(dict(event="return_stage", stage=stage))
            lower, upper = self._effective_position_limits(self._side)
            # A disabled arm may settle slightly under support or gravity while
            # a bounded RRT search runs.  Never execute a path from its stale
            # start, but automatically replan once from newly verified,
            # stationary feedback instead of forcing the operator through a
            # complete calibration/arming cycle again.
            plan = None
            for plan_attempt in range(2):
                planner = ReturnPlanner(lower, upper, self._check_physical_path,
                    timeout=self._cfg.return_plan_timeout_s,
                    max_nodes=self._cfg.return_plan_max_nodes,
                    seed=self._cfg.return_plan_seed,
                    cancelled=self._return_cancel.is_set)
                plan = planner.plan(current, zero)
                self._emit_probe(dict(event="return_plan", attempt=plan_attempt + 1,
                    seed=plan.seed, nodes=plan.nodes, reason=plan.reason,
                    start_physical_rad=current.tolist(),
                    path_physical_rad=[q.tolist() for q in plan.path],
                    collision=getattr(self._collision_guard, "last_diagnostics", None)))
                if not plan.path:
                    return finish(reason=plan.reason)
                actual = self._return_stationary(allow_disabled=not self._enabled)
                if actual is None:
                    return finish(reason="fresh stationary feedback lost after planning")
                drift = float(np.max(np.abs(actual-current)))
                if drift <= np.deg2rad(.2):
                    break
                if self._enabled or plan_attempt == 1:
                    return finish(reason=(
                        "start kept moving during planning; keep supporting the arm "
                        "and request a new return"
                    ))
                if not self._check_physical_path(actual, actual):
                    return finish(reason="pose became unsafe while replanning; manually remove contact")
                self._emit_probe(dict(event="return_replan_stationary_start",
                    attempt=plan_attempt + 2, drift_deg=float(np.degrees(drift)),
                    feedback_physical_rad=actual.tolist()))
                current = actual
            if self._return_cancel.is_set():
                return finish(reason="cancelled")
            if recover:
                # Explicit recovery clears software latches only after all
                # hardware, geometry and stationary checks have succeeded.
                self._return_inhibited = False
                self._safety_fault_reason = None
                if not self._enabled:
                    disabled_confirmed = False
                    stage = "enabling"
                    self._emit_probe(dict(event="return_stage", stage=stage))
                    if self._return_cancel.is_set() or not self._enable_for_return():
                        reason = getattr(self, "_last_return_enable_failure", None)
                        # The attempt may have failed before enable was sent,
                        # after all joints enabled, or in a mixed state.  Do a
                        # read-only final check instead of reporting the
                        # conservative pre-attempt placeholder as hardware
                        # state.
                        states = self._arm.get_joint_enable_states()
                        disabled_confirmed = (
                            states is not None and len(states) == 7
                            and not any(states)
                        )
                        self._enabled = (
                            states is not None and len(states) == 7
                            and all(states)
                        )
                        return finish(
                            reason=reason or "recovery enable failed/cancelled",
                            disabled=disabled_confirmed,
                        )
            self._return_start_state = None  # Stationary after bounded search.
            stage = "executing"
            self._emit_probe(dict(event="return_stage", stage=stage))
            for number, waypoint in enumerate(plan.path[1:], 1):
                if self._return_cancel.is_set():
                    return finish(reason="cancelled")
                actual = self.read_joints()
                self._emit_probe(dict(event="return_waypoint", index=number,
                    target_physical_rad=waypoint.tolist(),
                    feedback_stamp=self._arm.last_feedback_timestamp))
                if actual is None or not self._check_physical_path(actual, waypoint):
                    return finish(reason="planned edge no longer safe")
                if self._return_start_state is not None:
                    # Planning/whole-edge certification may take >100 ms.
                    # Only a verified stationary boundary may refresh its clock;
                    # preserve the last command position (never snap to feedback).
                    actual = self._return_stationary()
                    initial = self._return_start_state
                    if (actual is None or np.max(np.abs(initial.velocity)) > 1e-8
                            or np.max(np.abs(actual-initial.position)) > tolerance):
                        return finish(reason="waypoint boundary no longer stationary")
                    self._return_start_state = replace(initial, stamp=float(self._arm.last_feedback_timestamp))
                target = np.zeros(14)
                target[sl] = (waypoint-zero)/directions
                self._return_execution_failure = None
                if not self._return_group_to_zero(target, indices, waypoint, tolerance,
                        timeout, f"waypoint {number}/{len(plan.path)-1}"):
                    return finish(reason=(self._return_execution_failure
                                          or "waypoint execution failed/cancelled"))
            stage = "disabling"
            if self._return_cancel.is_set():
                return finish(reason="cancelled before disable")
            disabled = bool(self._arm.disable())
            self._enabled = not disabled
            self._return_inhibited = not disabled
            return finish(True, "" if disabled else "zero verified; disable unconfirmed", disabled)
        except Exception as exc:
            return finish(reason=f"{type(exc).__name__}: {exc}")
        finally:
            with self._trajectory_lock:
                self._return_active = False
                self._return_pending = False
            self._return_lock.release()

    def _return_group_to_zero(
        self,
        full_target: np.ndarray,
        indices: np.ndarray,
        zero_physical: np.ndarray,
        tolerance: float,
        timeout: float,
        phase_label: str = "",
        brake_only: bool = False,
    ) -> bool:
        """Send a fixed target until fresh feedback repeatedly verifies zero."""
        target = np.asarray(full_target, dtype=np.float64).reshape(-1)
        if target.shape != (2 * NERO_NUM_JOINTS,) or not np.all(np.isfinite(target)):
            return False
        side_slice = (
            slice(0, NERO_NUM_JOINTS)
            if self._side == "left"
            else slice(NERO_NUM_JOINTS, 2 * NERO_NUM_JOINTS)
        )
        directions, offsets = self._mapping(self._side)
        target_physical = directions * target[side_slice] + offsets
        return_trajectory = CommandTrajectory(
            self._joint_rate_limit_vector(self._side, "max_joint_velocity") * (1.0 if brake_only else self._cfg.return_speed_scale),
            self._joint_rate_limit_vector(self._side, "max_joint_acceleration"),
            self._joint_rate_limit_vector(self._side, "max_joint_step"),
            np.deg2rad(self._cfg.command_trajectory_lead_deg),
            max_dt=self._cfg.command_trajectory_max_dt_s,
            lead_timeout=self._cfg.command_trajectory_lead_timeout_s)
        lower, upper = self._effective_position_limits(self._side)
        def return_fk(physical):
            full = np.zeros(14)
            full[side_slice] = (physical-offsets)/directions
            return self._trajectory_fk(full)
        deadline = time.monotonic() + timeout
        feedback_gap_started: float | None = None
        lead_hold_started: float | None = None
        lead_hold_target: np.ndarray | None = None
        lead_hold_reason: str | None = None
        previous_stamp: float | None = None
        previous_feedback = None
        verified_samples = 0
        required_samples = max(1, int(self._cfg.return_verify_samples))
        feedback_grace = min(self._cfg.command_trajectory_max_dt_s,
                             max(0.0, float(self._cfg.return_feedback_grace_s)))
        next_status = time.monotonic() + 1.0
        last_error_deg: np.ndarray | None = None
        while time.monotonic() < deadline and not self._return_cancel.is_set():
            if not self._controller_is_safe() or not self._torque_is_safe(self._arm, self._side):
                self._return_inhibited = True
                return False
            current = self.read_joints()
            stamp = self._arm.last_feedback_timestamp
            now = time.monotonic()
            if (
                current is None
                or stamp is None
                or not np.isfinite(stamp)
                or (previous_stamp is not None and stamp <= previous_stamp)
            ):
                feedback_gap_started = feedback_gap_started or now
                verified_samples = 0
                if now - feedback_gap_started >= feedback_grace:
                    print(
                        f"[{self._side}-arm] Return phase {phase_label} lost fresh "
                        f"joint feedback for {feedback_grace:.1f}s.",
                        flush=True,
                    )
                    return False
                time.sleep(0.01)
                continue
            feedback_gap_started = None
            initial = getattr(self, "_return_start_state", None) if previous_stamp is None else None
            if initial is not None:
                if float(stamp) == initial.stamp:
                    time.sleep(0.01)
                    continue
                if not 0 < float(stamp) - initial.stamp <= return_trajectory.max_dt:
                    self._return_inhibited = True
                    self._emit_probe(dict(event="return_fault", fault="return command state expired"))
                    return False
                # A normal operator stop must brake the outstanding command,
                # not snap back to feedback and discard its velocity.
                return_trajectory.state = initial
                previous_stamp = initial.stamp
                self._return_start_state = None
            stationary = (
                previous_feedback is not None
                and stamp > previous_feedback[1]
                and np.max(
                    np.abs(current - previous_feedback[0])
                    / (stamp - previous_feedback[1])
                )
                <= np.deg2rad(self._cfg.return_stationary_velocity_deg_s)
            )
            previous_feedback = (current.copy(), float(stamp))
            if lead_hold_target is not None:
                lead_error = lead_hold_target - current
                if not self._check_physical_path(current, lead_hold_target):
                    self._return_inhibited = True
                    self._emit_probe(dict(
                        event="return_fault",
                        fault="held return command path became unsafe",
                        phase=phase_label,
                        held_command_physical_rad=lead_hold_target.tolist(),
                        feedback_physical_rad=current.tolist(),
                    ))
                    print(
                        f"[{self._side}-arm] Return phase {phase_label} stopped: "
                        "the outstanding certified command is no longer collision-safe.",
                        flush=True,
                    )
                    return False
                hold_elapsed = now - lead_hold_started
                catch_tolerance = np.minimum(
                    0.5 * return_trajectory.lead,
                    np.full(NERO_NUM_JOINTS, tolerance, dtype=np.float64),
                )
                if hold_elapsed >= return_trajectory.lead_timeout:
                    self._return_inhibited = True
                    lagging = np.flatnonzero(np.abs(lead_error) > catch_tolerance)
                    lag_summary = ", ".join(
                        f"J{i+1} {np.degrees(lead_error[i]):+.2f}deg"
                        for i in lagging
                    ) or "all joints inside catch tolerance but not stationary"
                    self._return_execution_failure = (
                        f"feedback did not catch held command within "
                        f"{return_trajectory.lead_timeout:.1f}s; "
                        f"remaining command-feedback error: {lag_summary}; "
                        "no newer command sent"
                    )
                    self._emit_probe(dict(
                        event="return_fault",
                        fault="return feedback did not catch the held command",
                        phase=phase_label,
                        trigger=lead_hold_reason,
                        hold_elapsed_s=hold_elapsed,
                        lagging_joints=(lagging + 1).tolist(),
                        held_command_physical_rad=lead_hold_target.tolist(),
                        feedback_physical_rad=current.tolist(),
                        lead_rad=lead_error.tolist(),
                        lead_limit_rad=return_trajectory.lead.tolist(),
                    ))
                    print(
                        f"[{self._side}-arm] Return phase {phase_label} feedback "
                        f"did not catch the last certified command within "
                        f"{return_trajectory.lead_timeout:.1f}s; no newer command was sent.",
                        flush=True,
                    )
                    return False
                if stationary and np.all(np.abs(lead_error) <= catch_tolerance):
                    return_trajectory.reset(lead_hold_target, stamp)
                    previous_stamp = float(stamp)
                    self._last_command_feedback_timestamp = float(stamp)
                    self._emit_probe(dict(
                        event="return_lead_resumed",
                        phase=phase_label,
                        trigger=lead_hold_reason,
                        hold_elapsed_s=hold_elapsed,
                        held_command_physical_rad=lead_hold_target.tolist(),
                        feedback_physical_rad=current.tolist(),
                        lead_rad=lead_error.tolist(),
                    ))
                    print(
                        f"[{self._side}-arm] Return phase {phase_label} feedback "
                        f"caught the held command after {hold_elapsed:.2f}s; continuing.",
                        flush=True,
                    )
                    lead_hold_started = None
                    lead_hold_target = None
                    lead_hold_reason = None
                time.sleep(0.01)
                continue
            if previous_stamp is None:
                # Start each return phase from measured feedback. Do not inherit
                # a timestamp or velocity state from teleoperation/the prior phase.
                previous_stamp = float(stamp)
                self._last_command_feedback_timestamp = float(stamp)
                self._command_velocity[self._side] = np.zeros(
                    NERO_NUM_JOINTS, dtype=np.float64
                )
                if not self._check_physical_path(current, current):
                    self._return_inhibited = True
                    return False
                if not self._send_return_position(current):
                    return False
                return_trajectory.reset(current, stamp)
            else:
                dt = float(stamp - previous_stamp)
                previous_stamp = float(stamp)
                try:
                    proposal = return_trajectory.propose(
                        target_physical, current, stamp, lower, upper, return_fk,
                        self._trajectory_endpoint_speed * (1.0 if brake_only else self._cfg.return_speed_scale),
                        path_check=self._check_physical_path, brake_only=brake_only)
                    limited = proposal.position
                except TrajectoryFault as exc:
                    fault_code = exc.diagnostics.get("fault_code")
                    if fault_code in {
                        "last_command_outside_feedback_lead",
                        "persistent_feedback_lead",
                    }:
                        held = return_trajectory.state.position.copy()
                        if not self._check_physical_path(current, held):
                            self._return_inhibited = True
                            self._emit_probe(dict(
                                event="return_fault",
                                fault="outstanding return command path is unsafe",
                                phase=phase_label,
                                trigger=fault_code,
                                trajectory_diagnostics=exc.diagnostics,
                            ))
                            return False
                        lead_hold_started = now
                        lead_hold_target = held
                        lead_hold_reason = fault_code
                        self._emit_probe(dict(
                            event="return_lead_hold",
                            phase=phase_label,
                            trigger=fault_code,
                            timeout_s=return_trajectory.lead_timeout,
                            held_command_physical_rad=held.tolist(),
                            feedback_physical_rad=current.tolist(),
                            lead_rad=(held - current).tolist(),
                            lead_limit_rad=return_trajectory.lead.tolist(),
                            trajectory_diagnostics=exc.diagnostics,
                        ))
                        print(
                            f"[{self._side}-arm] Return phase {phase_label} is "
                            "holding the last certified command while feedback catches up; "
                            "no newer command will be sent.",
                            flush=True,
                        )
                        time.sleep(0.01)
                        continue
                    self._return_inhibited = True
                    self._emit_probe(dict(
                        event="return_fault", fault=str(exc),
                        trajectory_diagnostics=exc.diagnostics))
                    print(
                        f"[{self._side}-arm] Return phase {phase_label} rejected "
                        f"an unsafe return trajectory: {exc}.",
                        flush=True,
                    )
                    return False
                except (TypeError, ValueError) as exc:
                    self._return_inhibited = True
                    self._emit_probe(dict(event="return_fault", fault=str(exc)))
                    print(
                        f"[{self._side}-arm] Return phase {phase_label} rejected "
                        f"an unsafe return trajectory: {exc}.",
                        flush=True,
                    )
                    return False
                if not self._check_physical_path(current, limited):
                    self._return_inhibited = True
                    return False
                if not self._send_return_position(limited):
                    return False
                return_trajectory.commit(proposal)
                self._last_command_feedback_timestamp = float(stamp)
            if brake_only:
                zero_physical = return_trajectory.state.position
            last_error_deg = np.rad2deg(current - zero_physical)
            if now >= next_status:
                selected = ", ".join(
                    f"J{int(index) + 1}={last_error_deg[index]:+.2f}deg"
                    for index in indices
                )
                print(
                    f"[{self._side}-arm] Return phase {phase_label}: {selected}; "
                    f"verified={verified_samples}/{required_samples}.",
                    flush=True,
                )
                next_status = now + 1.0
            if (stationary and np.max(np.abs(return_trajectory.state.velocity)) <= 1e-8
                    and np.max(np.abs(current[indices] - zero_physical[indices])) <= tolerance
                    and np.max(np.abs(return_trajectory.state.position[indices] - zero_physical[indices])) <= tolerance):
                verified_samples += 1
                if verified_samples >= required_samples:
                    # Feedback tolerance is the acceptance criterion. Never
                    # bypass trajectory/collision checks with a final snap.
                    self._return_start_state = return_trajectory.state
                    return True
            else:
                verified_samples = 0
            time.sleep(0.01)
        error_text = "feedback unavailable"
        if last_error_deg is not None:
            error_text = ", ".join(
                f"J{int(index) + 1}={last_error_deg[index]:+.2f}deg"
                for index in indices
            )
        print(
            f"[{self._side}-arm] Return phase {phase_label} timed out after "
            f"{timeout:.1f}s: {error_text}.",
            flush=True,
        )
        return False

    def disable(self) -> bool:
        self.cancel_return()
        with self._trajectory_lock:
            self.invalidate_command_trajectory()
        return self._disable_locked()

    def _disable_locked(self) -> bool:
        disabled = self._arm.disable()
        self._enabled = not disabled
        if disabled:
            self._return_inhibited = False
        return disabled

    def emergency_stop(self) -> None:
        self.cancel_return()
        with self._trajectory_lock:
            self.invalidate_command_trajectory()
            self._return_inhibited = True
            self._safety_fault_reason = "operator emergency stop; explicit recovery required"
            self._software_estop_recovery_pending = False
            self._inherited_estop_recovery_pending = False
            self._arm.emergency_stop()
            self._enabled = False

    def disconnect(self) -> None:
        self._arm.disconnect()


class NeroWristDriver:
    """Single-arm pilot that moves only configured terminal joints from IMU deltas."""

    def __init__(self, cfg: RobotConfig, side: str):
        if cfg.command_mode != "j":
            raise ValueError("wrist IMU mode requires smoothed command_mode='j'")
        if side not in ("left", "right"):
            raise ValueError("side must be 'left' or 'right'")
        self._cfg = cfg
        self._side = side
        channel = cfg.left_can_channel if side == "left" else cfg.right_can_channel
        self._arm = NeroArm(cfg, channel, side)
        self._session_start: np.ndarray | None = None
        self._wrist_command_velocity = np.zeros(
            len(cfg.right_wrist_joint_indices), dtype=np.float64
        )
        self._enabled = False
        self._emergency_stopped = False

    def connect(self) -> None:
        self._arm.connect()
        disable_on_connect = (
            self._cfg.left_arm_disable_on_connect
            if self._side == "left"
            else self._cfg.right_arm_disable_on_connect
        )
        if disable_on_connect and not self._arm.disable():
            raise RuntimeError(f"{self._side} NERO did not confirm all-joint disable")
        self._enabled = False

    def read_joints(self) -> np.ndarray | None:
        joints = self._arm.get_joint_angles()
        if not NeroDualArmDriver._valid_joints(joints):
            return None
        return np.asarray(joints, dtype=np.float64)

    def capture_session_start(self) -> bool:
        joints = self.read_joints()
        if joints is None or not self._within_hard_limits(joints):
            return False
        self._session_start = joints.copy()
        self._wrist_command_velocity = np.zeros(
            len(self._cfg.right_wrist_joint_indices), dtype=np.float64
        )
        return True

    def physical_to_urdf(self, joints: np.ndarray) -> np.ndarray:
        """Convert this arm's SDK feedback to the URDF joint convention."""
        values = np.asarray(joints, dtype=np.float64).reshape(-1)
        directions = np.asarray(
            self._cfg.left_joint_directions
            if self._side == "left"
            else self._cfg.right_joint_directions,
            dtype=np.float64,
        )
        offsets = np.asarray(
            self._cfg.left_joint_offsets
            if self._side == "left"
            else self._cfg.right_joint_offsets,
            dtype=np.float64,
        )
        if values.shape != (NERO_NUM_JOINTS,) or directions.shape != values.shape:
            raise ValueError("arm mapping must contain seven joints")
        if offsets.shape != values.shape or not np.all(np.isin(directions, (-1.0, 1.0))):
            raise ValueError("invalid arm direction/offset mapping")
        return (values - offsets) / directions

    def wrist_physical_directions(self) -> np.ndarray:
        indices = np.asarray(self._cfg.right_wrist_joint_indices, dtype=int)
        directions = np.asarray(
            self._cfg.left_joint_directions
            if self._side == "left"
            else self._cfg.right_joint_directions,
            dtype=np.float64,
        )
        return directions[indices]

    def wrist_neutral_positions(self) -> np.ndarray:
        """Return fixed SDK joint positions corresponding to calibrated IMU zero."""
        indices = np.asarray(self._cfg.right_wrist_joint_indices, dtype=int).reshape(-1)
        neutral = np.asarray(
            self._cfg.wrist_neutral_joint_positions, dtype=np.float64
        ).reshape(-1)
        if neutral.shape != indices.shape or not np.all(np.isfinite(neutral)):
            raise ValueError("wrist neutral positions must match configured wrist indices")
        return neutral.copy()

    def enable(self) -> bool:
        if self._session_start is None:
            return False
        # A previous electronic E-stop remains latched in the arm controller
        # across process restarts. Clear it only after the operator explicitly
        # presses 'e', while all motors are still disabled.
        self._arm.reset()
        time.sleep(0.1)
        # Leave leader linkage and preload the measured pose while all motors
        # are still disabled. The controller may ignore this preload; the
        # verified post-enable hold below is still required.
        self._arm.set_normal_mode()
        self._arm.set_motion_mode("j")
        self._arm.set_speed_percent(min(int(self._cfg.speed_percent), 10))
        self._arm.move_j(self._session_start.tolist())
        if not self._enabled and not self._arm.enable():
            return False
        enable_states = self._arm.get_joint_enable_states()
        if enable_states is None or len(enable_states) != NERO_NUM_JOINTS or not all(enable_states):
            self._arm.disable()
            return False
        self._enabled = True
        self._wrist_command_velocity = np.zeros(
            len(self._cfg.right_wrist_joint_indices), dtype=np.float64
        )
        self._arm.move_j(self._session_start.tolist())
        return True

    def disable(self) -> bool:
        """Stop wrist control and return the complete arm to disabled state."""
        self._session_start = None
        self._wrist_command_velocity = np.zeros(
            len(self._cfg.right_wrist_joint_indices), dtype=np.float64
        )
        disabled = self._arm.disable()
        self._enabled = False
        return disabled

    def return_wrist_to_neutral(self) -> bool:
        """Rate-limit J5/J6/J7 to fixed zero and verify feedback before disable."""
        if not self._enabled or self._session_start is None:
            return False
        indices = np.asarray(self._cfg.right_wrist_joint_indices, dtype=int).reshape(-1)
        neutral = self.wrist_neutral_positions()
        tolerance = np.deg2rad(
            max(0.1, float(self._cfg.wrist_return_tolerance_deg))
        )
        deadline = time.monotonic() + max(0.1, float(self._cfg.wrist_return_timeout_s))
        last_time = time.monotonic()
        feedback_gap_started: float | None = None
        verified_samples = 0
        required_samples = max(1, int(self._cfg.return_verify_samples))
        feedback_grace = max(0.0, float(self._cfg.return_feedback_grace_s))
        while time.monotonic() < deadline:
            now = time.monotonic()
            dt = max(now - last_time, 0.005)
            last_time = now
            if not self.command_offsets(np.zeros(indices.shape[0], dtype=np.float64), dt):
                feedback_gap_started = feedback_gap_started or now
                verified_samples = 0
                if now - feedback_gap_started >= feedback_grace:
                    return False
                time.sleep(0.02)
                continue
            current = self.read_joints()
            if current is None:
                feedback_gap_started = feedback_gap_started or now
                verified_samples = 0
                if now - feedback_gap_started >= feedback_grace:
                    return False
                time.sleep(0.02)
                continue
            feedback_gap_started = None
            if np.max(np.abs(current[indices] - neutral)) <= tolerance:
                verified_samples += 1
                if verified_samples >= required_samples:
                    # Send the exact verified terminal zero once; other joints
                    # keep their latest feedback positions.
                    target = current.copy()
                    target[indices] = neutral
                    self._arm.move_j(target.tolist())
                    return True
            else:
                verified_samples = 0
            time.sleep(0.02)
        return False

    def _within_hard_limits(self, joints: np.ndarray) -> bool:
        lower = np.asarray(self._cfg.joint_lower_limits, dtype=np.float64)
        upper = np.asarray(self._cfg.joint_upper_limits, dtype=np.float64)
        return bool(np.all((joints >= lower) & (joints <= upper)))

    def command_offsets(self, offsets: np.ndarray, dt: float) -> bool:
        """Command bounded terminal-joint offsets while other joints hold feedback."""
        if self._session_start is None:
            return False
        current = self.read_joints()
        values = np.asarray(offsets, dtype=np.float64).reshape(-1)
        indices = np.asarray(self._cfg.right_wrist_joint_indices, dtype=int).reshape(-1)
        if (
            current is None
            or values.shape != indices.shape
            or indices.shape[0] not in (2, 3)
            or len(set(indices.tolist())) != indices.shape[0]
            or np.any(indices < 0)
            or np.any(indices >= NERO_NUM_JOINTS)
            or not np.all(np.isfinite(values))
            or not self._within_hard_limits(current)
        ):
            return False

        max_angle = np.deg2rad(max(0.0, float(self._cfg.right_wrist_max_angle_deg)))
        values = np.clip(values, -max_angle, max_angle)
        target = self.wrist_neutral_positions() + values

        lower = np.asarray(self._cfg.joint_lower_limits, dtype=np.float64)
        upper = np.asarray(self._cfg.joint_upper_limits, dtype=np.float64)
        margin = max(0.0, float(self._cfg.joint_limit_margin))
        target = np.clip(target, lower[indices] + margin, upper[indices] - margin)

        desired = current.copy()
        if dt <= 0.0:
            return False
        max_velocity = float("inf")
        if self._cfg.right_wrist_max_velocity > 0.0:
            max_velocity = min(max_velocity, float(self._cfg.right_wrist_max_velocity))
        if self._cfg.right_wrist_max_step > 0.0:
            max_velocity = min(max_velocity, float(self._cfg.right_wrist_max_step) / dt)
        target_velocity = (target - current[indices]) / dt
        if np.isfinite(max_velocity):
            target_velocity = np.clip(target_velocity, -max_velocity, max_velocity)

        previous_velocity = getattr(
            self, "_wrist_command_velocity", np.zeros(indices.shape[0], dtype=np.float64)
        )
        max_acceleration = max(0.0, float(self._cfg.right_wrist_max_acceleration))
        if max_acceleration > 0.0:
            velocity = previous_velocity + np.clip(
                target_velocity - previous_velocity,
                -max_acceleration * dt,
                max_acceleration * dt,
            )
        else:
            velocity = target_velocity
        delta = velocity * dt
        error = target - current[indices]
        reached = np.abs(delta) > np.abs(error)
        delta[reached] = error[reached]
        self._wrist_command_velocity = delta / dt
        desired[indices] = current[indices] + delta

        # Non-wrist joints are copied from fresh feedback, so this mode cannot
        # generate a target displacement for the shoulder or elbow.
        self._arm.move_j(desired.tolist())
        return True

    def emergency_stop(self) -> None:
        self._arm.emergency_stop()
        self._emergency_stopped = True
        self._enabled = False

    def disconnect(self) -> None:
        if not self._emergency_stopped:
            try:
                self.disable()
            except Exception:  # noqa: BLE001
                pass
        self._arm.disconnect()


class NeroRightWristDriver(NeroWristDriver):
    """Backward-compatible right-arm wrist pilot."""

    def __init__(self, cfg: RobotConfig):
        super().__init__(cfg, "right")


def imu_rotation_to_wrist_offsets(
    rotation: np.ndarray,
    cfg: RobotConfig,
    wrist_rotation_world: np.ndarray,
    wrist_local_axes_per_physical_rad: np.ndarray,
) -> np.ndarray:
    """Project a world-frame IMU delta onto configured terminal arm joints."""
    from ..math_utils import rotation_matrix_to_rotvec

    world_delta = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    wrist_rotation = np.asarray(wrist_rotation_world, dtype=np.float64).reshape(3, 3)
    axes = np.asarray(wrist_local_axes_per_physical_rad, dtype=np.float64)
    if axes.ndim != 2 or axes.shape[0] != 3 or axes.shape[1] not in (2, 3):
        raise ValueError("wrist axes must have shape 3x2 or 3x3")
    if not all(np.all(np.isfinite(value)) for value in (world_delta, wrist_rotation, axes)):
        raise ValueError("wrist IMU mapping inputs must be finite")
    if np.linalg.matrix_rank(axes) < axes.shape[1]:
        raise ValueError("wrist joint orientation axes are singular")
    local_delta = wrist_rotation.T @ world_delta @ wrist_rotation
    local_rotvec = rotation_matrix_to_rotvec(local_delta).astype(np.float64)
    offsets = np.linalg.lstsq(axes, local_rotvec, rcond=None)[0]
    max_angle = np.deg2rad(max(0.0, float(cfg.right_wrist_max_angle_deg)))
    norm = float(np.linalg.norm(offsets))
    if max_angle > 0.0 and norm > max_angle:
        offsets *= max_angle / norm
    return np.clip(offsets, -max_angle, max_angle)


def imu_local_rotation_to_wrist_offsets(
    local_rotation: np.ndarray,
    cfg: RobotConfig,
    wrist_local_axes_per_physical_rad: np.ndarray,
    *,
    limit_total_angle: bool = True,
) -> np.ndarray:
    """Project a robot-palm-local IMU delta onto physical terminal joints."""
    from ..math_utils import rotation_matrix_to_rotvec

    rotation = np.asarray(local_rotation, dtype=np.float64).reshape(3, 3)
    axes = np.asarray(wrist_local_axes_per_physical_rad, dtype=np.float64)
    if axes.ndim != 2 or axes.shape[0] != 3 or axes.shape[1] not in (2, 3):
        raise ValueError("wrist axes must have shape 3x2 or 3x3")
    if not np.all(np.isfinite(rotation)) or not np.all(np.isfinite(axes)):
        raise ValueError("wrist IMU mapping inputs must be finite")
    if np.linalg.matrix_rank(axes) < axes.shape[1]:
        raise ValueError("wrist joint orientation axes are singular")
    offsets = np.linalg.lstsq(axes, rotation_matrix_to_rotvec(rotation), rcond=None)[0]
    max_angle = np.deg2rad(max(0.0, float(cfg.right_wrist_max_angle_deg)))
    norm = float(np.linalg.norm(offsets))
    if limit_total_angle and max_angle > 0.0 and norm > max_angle:
        offsets *= max_angle / norm
    return np.clip(offsets, -max_angle, max_angle)


def right_imu_rotation_to_wrist_offsets(
    rotation: np.ndarray,
    cfg: RobotConfig,
    wrist_rotation_world: np.ndarray,
    wrist_local_axes_per_physical_rad: np.ndarray,
) -> np.ndarray:
    """Backward-compatible wrapper for the side-independent wrist projection."""
    return imu_rotation_to_wrist_offsets(
        rotation, cfg, wrist_rotation_world, wrist_local_axes_per_physical_rad
    )


__all__ = [
    "NeroArm", "NeroDualArmDriver", "NeroWristDriver", "NeroRightWristDriver",
    "imu_local_rotation_to_wrist_offsets", "imu_rotation_to_wrist_offsets",
    "right_imu_rotation_to_wrist_offsets",
]
