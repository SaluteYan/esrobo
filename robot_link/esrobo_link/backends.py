"""Mock and real robot executors. Hardware imports are lazy and explicit."""
import dataclasses
import hashlib
from pathlib import Path
import time

from .protocol import canonical, InputHoldTransition, HoldReturnTransition


class MockBackend:
    """No CAN, ROS, PICO, glove, numpy, or graphical dependencies."""
    def __init__(self, side="left", with_hand=False, hand_only=False):
        if hand_only and not with_hand:
            raise ValueError("hand-only mock requires a hand")
        self.contract = dict(id=f"mock-v1-{side}-{with_hand}-{bool(hand_only)}", side=side, with_hand=with_hand,
                             hand_only=bool(hand_only),
                             input_hold_protocol=2,
                             input_recovery_protocol=1, input_recovery_enabled=False,
                             return_protocol=3, hold_return_protocol=1,
                             arm_unit="URDF radians", hand_unit="L10 physical 0..255",
                             lower_rad=[-1.0] * 7, upper_rad=[1.0] * 7)
        self.q = [0.0] * 7
        self.hand = [128] * 10 if with_hand else None
        self.enabled = False
        self.hand_enabled = False
        self.hand_only = bool(hand_only)
        self.supports_return = not self.hand_only
        self.supports_prepare = True
        self.stops = 0

    def snapshot(self):
        return dict(arm_urdf_rad=list(self.q), arm_feedback_age_s=0.0,
                    enable_states=[self.enabled] * 7,
                    hand=None if self.hand is None else dict(position_unit=list(self.hand), age_s=0.0))

    def fault_diagnostics(self):
        return None

    def preflight_enable(self, target, *, entry=False):
        pass

    def enable(self, target, cancelled, *, activate_hand=True):
        if cancelled():
            raise RuntimeError("enable cancelled")
        if self.hand_only:
            self.hand_enabled = True
        else:
            self.enabled = True

    def step(self, target, cancelled):
        if cancelled() or (self.hand_only and not self.hand_enabled) or (not self.hand_only and not self.enabled):
            raise RuntimeError("not enabled/cancelled")
        if target.get("hold"):
            return
        if not self.hand_only:
            self.q = [q + max(-.005, min(.005, t - q)) for q, t in zip(self.q, target["arm_urdf_rad"])]
        if self.hand is not None:
            self.hand = list(target["hand_unit"])

    def prepare_step(self, cancelled):
        if cancelled():
            raise RuntimeError("cancelled")

    def send_arm(self, target, cancelled):
        self.step(target, cancelled)

    def monitor_hold(self):
        if self.hand_only:
            if self.enabled:
                raise RuntimeError("mock arm enabled in hand-only mode")
            return
        if not self.enabled:
            raise RuntimeError("mock held arm disabled")

    def hold_arm(self):
        if not self.enabled:
            raise RuntimeError("mock arm cannot hold while disabled")

    def stop(self):
        self.stops += 1
        self.enabled = False
        self.hand_enabled = False

    def cancel(self):
        pass

    def disable(self):
        self.enabled = False
        return True

    def recover_idle(self):
        if not self.hand_only:
            raise RuntimeError("stationary recovery is only available in hand-only mode")
        if not self.disable() or not self.disable_hands():
            raise RuntimeError("mock disable not verified")
        return {"disabled": True, "hands_ready": True, "stage": "hand_only_recovery"}

    def disable_side(self, side):
        if side != self.contract["side"]:
            raise RuntimeError(f"gateway does not own {side} arm")
        return self.disable()

    def disable_hands(self):
        if self.hand is None:
            raise RuntimeError("gateway has no hand")
        self.hand_enabled = False
        return bool(self.hand is not None)

    def return_zero(self, cancelled, *, from_hold=False):
        if cancelled():
            raise RuntimeError("return cancelled")
        self.q = [0.0] * 7
        self.enabled = False
        return {"returned": True, "disabled": True, "stage": "mock", "reason": ""}

    def prepare_session(self, cancelled):
        if cancelled():
            raise RuntimeError("preparation cancelled")
        if not self.hand_only:
            self.q = [0.0] * 7
            self.enabled = False
        if self.hand is not None:
            self.hand = [128] * 10
            self.hand_enabled = False
        return {"returned": True, "disabled": True,
                "hands_open": self.hand is None or self.hand == [128] * 10,
                "stage": "mock", "reason": ""}

    def close(self):
        pass


class ForwardModel:
    """Only FK/model data for the robot's existing safety checks; no IK solver."""
    def __init__(self, cfg):
        import numpy as np
        import pinocchio as pin
        self._cfg = cfg
        self._pin = pin
        self._model = pin.buildModelFromUrdf(cfg.urdf_path)
        ids = [self._model.getJointId(n) for n in cfg.left_arm_joints + cfg.right_arm_joints]
        if any(i == 0 or i >= self._model.njoints for i in ids):
            raise ValueError("URDF arm joint missing")
        self._arm_q_indices = np.array([self._model.idx_qs[i] for i in ids])

    def fk(self, side):
        import numpy as np
        import threading
        data, lock = self._model.createData(), threading.Lock()
        ids = [self._model.getFrameId(getattr(self._cfg, f"{side}_{name}"))
               for name in ("elbow_frame", "hand_frame")]
        if any(i >= self._model.nframes for i in ids):
            raise ValueError("URDF task frame missing")

        def evaluate(full):
            q = np.zeros(self._model.nq)
            q[self._arm_q_indices] = full
            with lock:
                self._pin.forwardKinematics(self._model, data, q)
                self._pin.updateFramePlacements(self._model, data)
                return np.array([data.oMf[i].translation.copy() for i in ids])
        return evaluate


class HardwareBackend:
    """One arm, optionally its hand. Does not start acquisition or visualization."""
    # Also keeps unit-test instances built with ``__new__`` on the normal arm
    # path; production instances overwrite this in ``__init__``.
    hand_only = False

    def entry_execution_diagnostics(self):
        return self.arm.entry_execution_diagnostics()

    def __init__(self, config, side, with_hand, *, hand_driver=None, hand_only=False):
        import numpy as np
        from esrobo_teleop.config import load_config, retarget_filter_fingerprint
        from esrobo_teleop.robot.trajectory_version import TRAJECTORY_ALGORITHM_VERSION
        from esrobo_teleop.robot.nero_driver import NeroSingleArmDriver
        from esrobo_teleop.robot.linker_hand_driver import LinkerHandDriver, L10_PHYSICAL_JOINT_NAMES
        from esrobo_teleop.robot.torso_collision import TorsoCollisionGuard

        config_path = Path(config).resolve()
        cfg = load_config(str(config_path))
        model_path = Path(cfg.ik.urdf_path)
        if not model_path.is_absolute():
            model_path = config_path.parent.parent / model_path
        cfg.ik.urdf_path = str(model_path.resolve())
        if not (cfg.robot.torso_collision_enabled and cfg.robot.controller_status_monitor_enabled
                and cfg.robot.torque_monitor_enabled
                and getattr(cfg.robot, f"{side}_command_trajectory_enabled")):
            raise ValueError("robot link requires collision preflight, controller/torque monitoring and trajectory")
        if with_hand and (cfg.hand.model.upper() != "L10" or cfg.hand.mode != "udp"
                          or not cfg.hand.require_feedback_on_enable):
            raise ValueError("robot link hand requires L10/loopback UDP and feedback gating")
        if hand_only and not with_hand:
            raise ValueError("hand-only backend requires a hand")
        self.cfg, self.side, self.np = cfg, side, np
        self.hand_only = bool(hand_only)
        self.supports_return = not self.hand_only
        self.supports_prepare = True
        self.arm = NeroSingleArmDriver(cfg.robot, side)
        self.hand = None
        self._owns_hand = hand_driver is None
        self._full_slice = slice(0, 7) if side == "left" else slice(7, 14)
        model = ForwardModel(cfg.ik)
        self.arm.configure_command_trajectory(model.fk(side))
        guard = TorsoCollisionGuard(model, side, cfg.robot.collision_package_dirs,
                                   cfg.robot.torso_collision_margin_m,
                                   cfg.robot.shoulder_collision_margin_m, include_fingers=with_hand)
        self.arm.configure_collision_guard(guard)
        try:
            if with_hand:
                self.hand = hand_driver if hand_driver is not None else LinkerHandDriver(cfg.hand, udp_host="127.0.0.1",
                                             udp_port=cfg.hand.udp_port, active_sides=(side,))
                guard.hand_state_provider = lambda horizon: self.hand.geometry_state(
                    side, horizon, cover_valid_age=(guard._entry_certificates is not None
                                                   and not guard._entry_coverage_only))
            lo, hi = self.arm._effective_position_limits(side)
            direction, offset = self.arm._mapping(side)
            a, b = (lo - offset) / direction, (hi - offset) / direction
            fingerprint = dict(robot=dataclasses.asdict(cfg.robot), hand=dataclasses.asdict(cfg.hand),
                               ik={k:v for k,v in dataclasses.asdict(cfg.ik).items() if k != "urdf_path"},
                               trajectory_algorithm=TRAJECTORY_ALGORITHM_VERSION,
                               retarget_filter=retarget_filter_fingerprint(cfg.retarget),
                               model_sha256=hashlib.sha256(model_path.read_bytes()).hexdigest(),
                               side=side, with_hand=with_hand, hand_only=self.hand_only)
            self.contract = dict(id=hashlib.sha256(canonical(fingerprint)).hexdigest(),
                                 side=side, with_hand=with_hand, arm_unit="URDF radians",
                                 hand_only=self.hand_only,
                                 input_hold_protocol=2,
                                 input_recovery_protocol=1,
                                 input_recovery_enabled=getattr(cfg.robot, side+'_input_recovery_verified') is True,
                                 return_protocol=3, hold_return_protocol=1,
                                 hand_unit="L10 physical 0..255", hand_order=L10_PHYSICAL_JOINT_NAMES,
                                 arm_order=list(getattr(cfg.ik, f"{side}_arm_joints")),
                                 lower_rad=np.minimum(a, b).tolist(), upper_rad=np.maximum(a, b).tolist(),
                                 physical_direction=direction.tolist(), physical_offset_rad=offset.tolist(),
                                 velocity_rad_s=self.arm._joint_rate_limit_vector(side, "max_joint_velocity").tolist(),
                                 acceleration_rad_s2=self.arm._joint_rate_limit_vector(side, "max_joint_acceleration").tolist())
            if not self.hand_only:
                self.contract["entry_protocol"] = 7
                self.contract["entry_keepalive_protocol"] = 1
            self.arm.connect()
        except BaseException:
            self.close()
            raise

    def attach_torque_recorder(self, writer, session, mode):
        from esrobo_teleop.debug.torque_diagnostics import TorqueRecorder
        self.arm._torque_recorder = TorqueRecorder(session, self.side, writer, mode)

    def fault_diagnostics(self):
        # Preserve the exact rejected trajectory and separately frozen torque
        # evidence, without another CAN read or replacing existing fields.
        diagnostics = self.arm.trajectory_diagnostics
        recorder = self.arm.__dict__.get('_torque_recorder')
        torque = None if recorder is None else recorder.fault_snapshot()
        return dict(diagnostics or {}, torque=torque) if torque is not None else diagnostics

    def snapshot(self):
        full = self.arm.read_control_cycle_joints()
        snap = getattr(self.arm, "_cycle_feedback", None)
        feedback_diagnostic = (getattr(self.arm, "feedback_failure_diagnostics", None)
                               if full is None else None)
        states = []
        for index in range(1, 8):
            state = self.arm._arm._robot.get_driver_states(index)
            if (state is None or not self.np.isfinite(state.timestamp)
                    or not 0 <= time.time() - state.timestamp <= self.cfg.robot.controller_status_timeout_s):
                states = None
                break
            states.append(bool(state.msg.foc_status.driver_enable_status))
        return dict(arm_urdf_rad=None if full is None else full[self._full_slice].tolist(),
                    motion_diagnostics=getattr(self.arm, 'motion_limit_diagnostics', None),
                    torque_diagnostics=(self.arm._torque_recorder.snapshot()
                                        if self.arm.__dict__.get('_torque_recorder') is not None else None),
                    arm_feedback_age_s=None if snap is None else max(0.0, time.monotonic() - snap.received_monotonic),
                    feedback_diagnostic=feedback_diagnostic,
                    feedback_request_diagnostic=(getattr(self.arm._arm, "feedback_request_diagnostic", None)
                                                 if full is None else None),
                    enable_states=states,
                    controller_fault=self.arm._arm.controller_fault(allow_disabled=True),
                    driver_fault=self.arm.safety_fault_reason,
                    hand=None if self.hand is None else self.hand.feedback_snapshot(self.side))

    def preflight_enable(self, target, *, entry=False):
        # No silent-controller wake, blind enable or implicit homing over network.
        state = self.snapshot()
        if state["enable_states"] != [False] * 7:
            raise RuntimeError("fresh seven-joint feedback and verified disable required; use local commissioning if silent")
        if state["controller_fault"]:
            raise RuntimeError(f"{self.side} controller preflight: {state['controller_fault']}")
        if not self.hand_only:
            if state["arm_urdf_rad"] is None:
                raise RuntimeError("fresh seven-joint feedback required")
            if self.np.max(self.np.abs(self.np.array(target["arm_urdf_rad"]) - state["arm_urdf_rad"])) > self.np.deg2rad(1.5):
                raise RuntimeError("first target must match measured arm pose within 1.5 degrees")
        if self.hand is not None:
            if not entry:
                if not self.hand.open_feedback_calibrated(self.side):
                    raise RuntimeError(f"{self.side} hand natural-open feedback zero is not calibrated")
                hand_open, hand_errors = self.hand.open_pose_status()
                if not hand_open:
                    detail = self.hand.describe_open_pose_errors(hand_errors)
                    raise RuntimeError("hand is not at natural-open zero: " +
                                       (detail or "fresh hand feedback unavailable"))
            current = self.hand.feedback_snapshot(self.side)["position_unit"]
            tolerance = 5 if entry else self.cfg.hand.startup_open_tolerance
            if (current is None or not self.hand.feedback_ready()
                    or max(abs(a-b) for a,b in zip(current, target["hand_unit"])) > tolerance):
                raise RuntimeError("entry hand target must match fresh held feedback" if entry
                                   else "first hand target must match measured open pose")

    def enable(self, target, cancelled, *, activate_hand=True):
        self.preflight_enable(target)
        if self.hand_only:
            if cancelled() or self.hand is None or not self.hand.set_enabled(True):
                raise RuntimeError("hand-only enable failed/cancelled")
            return
        if cancelled() or not self.arm.capture_session_start():
            raise RuntimeError("enable cancelled or missing feedback")
        if not self.arm.enable(cancelled=cancelled):
            raise RuntimeError(self.arm.safety_fault_reason or "arm enable failed/cancelled")
        if cancelled():
            raise RuntimeError("enable cancelled")
        self.arm.set_speed(self.cfg.robot.speed_percent)
        if not self.arm.initialize_command_trajectory():
            raise RuntimeError(self.arm.safety_fault_reason or "trajectory initialization failed")
        if activate_hand and self.hand and (cancelled() or not self.hand.set_enabled(True)):
            raise RuntimeError("hand enable failed/cancelled")

    def approach_entry(self, target, cancelled, on_approach):
        state = self.snapshot()
        self.preflight_enable(dict(target, arm_urdf_rad=state["arm_urdf_rad"]), entry=True)
        if state.get("driver_fault") or cancelled():
            raise RuntimeError("handover requires healthy fresh feedback/input")
        held_hand = None if self.hand is None else list(state["hand"]["position_unit"])
        def hand_stationary():
            if self.hand is None:
                return True
            sample = self.hand.feedback_snapshot(self.side)
            position = sample.get("position_unit")
            return bool(position is not None and sample.get("age_s") is not None
                        and sample["age_s"] <= self.cfg.hand.max_stale_time_s
                        and max(abs(a-b) for a,b in zip(position, held_hand)) <= 5)
        request_id = self.arm.prepare_return()
        if cancelled():
            self.arm.cancel_return()
        result = self.arm.safe_return(recover=True, request_id=request_id,
            handover_goal=target["arm_urdf_rad"], on_approach=on_approach,
            arm_only_collision=True, stationary_hand_check=hand_stationary)
        if result.returned and not hand_stationary():
            raise RuntimeError("held hand moved or feedback expired during handover")
        if not result.returned or cancelled():
            if (not cancelled() and getattr(self.arm, 'return_hold_preserved', False) is True
                    and self.contract.get('input_recovery_enabled') is True):
                holding = self.hand is None or (self.hand.feedback_ready() and self.hand.set_enabled(True))
                if holding:
                    return dict(dataclasses.asdict(result), holding=True)
            diagnostic = getattr(self.arm._collision_guard, "last_diagnostics", {})
            geometry = {key: diagnostic.get(key) for key in
                        ("category", "pair", "clearance_m", "required_m")}
            raise RuntimeError(f"handover refused: {dataclasses.asdict(result)}; last_geometry={geometry}")
        # safe_return left a verified stationary nonzero pose enabled. Do not
        # re-enable/reset the controller or human mapping at this boundary.
        if not self.arm.capture_session_start():
            raise RuntimeError('handover session capture failed: complete finite feedback inside hardware limits required')
        if not self.arm.initialize_command_trajectory(after_handover=True):
            reason = self.arm.safety_fault_reason or 'initialization rejected without driver detail'
            raise RuntimeError(f'handover trajectory initialization failed: {reason}')
        # The full hand-motion envelope can take longer than a feedback tick.
        # Finish it under APPROACHING supervision, before publishing readiness
        # and starting the fresh ready-ack deadline. No finger target is sent.
        self._prepare_entry_activation(target, cancelled)
        return dataclasses.asdict(result)

    def _hand_return_feedback(self, cancelled, *, anchor=None, wait=False, holding=False):
        """Read only: monitor feedback, never reset or enable.

        Controller recovery can require two independent CAN frames. Waiting
        is allowed only before certification/hand motion (or
        after hand output has stopped), not inside its active safety checks.
        """
        deadline = time.monotonic() + (self.cfg.robot.controller_status_timeout_s if wait else 0.)
        while True:
            if cancelled():
                return {}, 'cancelled during hand return feedback verification'
            state = self.snapshot()
            states, fault = state.get('enable_states'), state.get('controller_fault')
            q, age = state.get('arm_urdf_rad'), state.get('arm_feedback_age_s')
            transient = False
            if states is not None and states != ([True]*7 if holding else [False]*7):
                reason = ('arm enable lost during hand return' if holding else
                          'arm disable lost during hand return')
            elif state.get('driver_fault'):
                reason = 'arm driver fault during hand return: '+str(state['driver_fault'])
            elif fault and fault.get('category') not in ('controller_feedback_stale', 'driver_feedback_stale'):
                reason = 'arm controller fault during hand return: '+str(fault)
            elif states is None or fault or q is None:
                reason, transient = 'arm feedback incomplete during hand return', True
            elif self.np.asarray(q).shape != (7,) or not self.np.all(self.np.isfinite(q)):
                reason = 'invalid seven-joint values during hand return'
            elif (not isinstance(age, (int, float)) or not self.np.isfinite(age)
                    or not 0 <= age <= self.cfg.robot.command_trajectory_max_dt_s):
                reason, transient = 'arm feedback expired during hand return', True
            elif anchor is not None and self.np.max(self.np.abs(self.np.asarray(q)-anchor)) > self.np.deg2rad(.2):
                delta = self.np.degrees(self.np.asarray(q)-anchor)
                joint = int(self.np.argmax(self.np.abs(delta)))
                reason = f'arm moved during hand return: J{joint+1} delta={delta[joint]:.3f}deg exceeds 0.2deg'
            elif holding and (not self.arm._controller_is_safe()
                              or not self.arm._torque_is_safe(self.arm._arm, self.side)):
                reason = 'arm safety check failed during held hand return'
            else:
                return state, ''
            if not transient or time.monotonic() >= deadline:
                return state, reason
            time.sleep(min(.01, max(0., deadline-time.monotonic())))

    def _certify_return_hand(self, lower, upper, envelope, near_zero, monitor):
        from .hand_return_proof import certify
        proof = certify(self.cfg, self.side, lower, upper, envelope, near_zero, monitor, self.arm._collision_guard)
        self.arm._collision_guard.last_diagnostics = proof['diagnostics']
        self._last_hand_return_diagnostics = dict(proof)
        return proof['certified']

    def return_diagnostics(self):
        details = dict(getattr(self, '_last_hand_return_diagnostics', None) or {})
        for key, attr in (('first_motion_rejection', '_return_first_rejection'),
                          ('failure_hold', '_return_failure_hold_diagnostics')):
            value = getattr(self.arm, attr, None)
            if isinstance(value, dict):
                details[key] = value
        return details or None

    def _restore_hand_before_return(self, result, cancelled, *, holding=False):
        """Verify hand zero at a disabled or verified stationary held arm pose."""
        result = dict(result, hands_open=False, stage='hand_open_before_arm')
        reason = ''
        last_state = None
        self._last_hand_return_diagnostics = None
        try:
            if cancelled():
                raise RuntimeError('cancelled before hand open')
            # Controller recovery can invalidate the cycle snapshot. Reacquire
            # fresh CAN feedback within the existing bounded preflight wait,
            # before certifying geometry or sending any finger command.
            state, reason = self._hand_return_feedback(cancelled, wait=True, holding=holding)
            last_state = state
            if reason:
                if holding:
                    self.arm.cancel_return()
                raise RuntimeError(reason)
            actual = state.get('arm_urdf_rad')
            anchor = self.np.asarray(actual, dtype=float)
            if anchor.shape != (7,) or not self.np.all(self.np.isfinite(anchor)):
                raise RuntimeError('invalid arm feedback before hand open')
            zero_tolerance = self.np.deg2rad(self.cfg.robot.full_arm_return_tolerance_deg)
            near_zero = bool(self.np.max(self.np.abs(anchor)) <= zero_tolerance)
            result['near_zero_finger_exemption'] = near_zero
            already_open, _ = self.hand.open_pose_status()
            if already_open:
                # Fresh measured arrival needs no new hand command or swept
                # motion certificate. Do not move a hand already at its zero.
                self.hand.finish_output(cancelled, safety_check=lambda: not
                    self._hand_return_feedback(cancelled, anchor=anchor, holding=holding)[1])
                return dict(result, hands_open=True, disabled=not holding, reason='')
            envelope = None
            if near_zero:
                if not self.hand.feedback_ready():
                    if holding:
                        self.arm.cancel_return()
                    raise RuntimeError('fresh hand feedback required before hand open')
            else:
                envelope = self.hand.open_return_geometry_state(self.side)
                if not envelope.valid:
                    raise RuntimeError('hand open geometry unavailable: '+envelope.reason)
            full = self.np.zeros(14)
            full[self._full_slice] = anchor
            drift = self.np.zeros(14)
            drift[self._full_slice] = self.np.deg2rad(.2)
            def check():
                nonlocal reason, last_state
                last_state, reason = self._hand_return_feedback(cancelled, anchor=anchor, holding=holding)
                if reason:
                    if holding:
                        self.arm.cancel_return()  # A real safety loss cannot resume hold.
                    return False
                if near_zero:
                    if self.np.max(self.np.abs(last_state['arm_urdf_rad'])) > zero_tolerance:
                        reason = 'arm left near-zero region during hand return'
                        return False
                    if not self.hand.feedback_ready():
                        reason = 'hand feedback stale during hand return'
                        return False
                    return True
                fresh = self.hand.geometry_state(self.side, 0.)
                if (not fresh.valid or set(fresh.intervals) != set(envelope.intervals)
                        or any(lo < envelope.intervals[key][0] or hi > envelope.intervals[key][1]
                               for key, (lo, hi) in fresh.intervals.items())):
                    reason = 'hand feedback left certified open sweep'
                    return False
                return True

            def monitor():
                ok = check()
                if not ok and holding:
                    self.arm.cancel_return()
                return ok

            if not self._certify_return_hand(full-drift, full+drift, envelope, near_zero, monitor):
                diagnostic = self.arm._collision_guard.last_diagnostics
                pair = diagnostic.get('pair')
                clearance, required = diagnostic.get('clearance_m'), diagnostic.get('required_m')
                detail = ''
                if isinstance(pair, (list, tuple)) and len(pair) == 2:
                    detail = ': '+str(pair[0])+' / '+str(pair[1])
                if isinstance(clearance, (int, float)) and isinstance(required, (int, float)):
                    detail += f'; clearance={clearance*1000:.1f}mm required={required*1000:.1f}mm'
                raise RuntimeError('hand open sweep not certified safe'+detail)
            # A certificate is useful only while the original feedback, pose
            # and hand envelope still match. No command precedes this check.
            if not monitor():
                raise RuntimeError(reason or 'hand return monitoring failed')

            opened, errors = self.hand.align_and_verify_open_pose(
                cancelled, safety_check=monitor, force=True)
            if not opened or not monitor():
                raise RuntimeError(reason or ('hand open feedback not verified: '+
                                             self.hand.describe_open_pose_errors(errors)))
            # A verified pose can arrive while the final redundant command is
            # between its two CAN frames. Finish that pair before normal stop;
            # aborts still take the immediate-stop finally path below.
            self.hand.finish_output(cancelled, safety_check=monitor)
            opened, errors = self.hand.open_pose_status()
            if not opened or not monitor():
                raise RuntimeError(reason or ('hand zero lost before arm return (after completion): '+
                                             self.hand.describe_open_pose_errors(errors)))
            return dict(result, hands_open=True, disabled=not holding, reason='')
        except Exception as exc:
            # A rejected hand phase must not start the arm or advertise IDLE.
            self.hand.set_enabled(False)
            try:
                final, _ = self._hand_return_feedback(cancelled, wait=True, holding=holding)
                disabled = final.get('enable_states') == [False]*7
            except Exception:
                disabled = False
            from .state_summary import feedback_summary, short_text
            self._last_hand_return_diagnostics = dict(self._last_hand_return_diagnostics or {},
                reason=reason or str(exc), feedback=last_state,
                geometry=dict(getattr(self.arm._collision_guard, 'last_diagnostics', {})))
            return dict(result, returned=False, disabled=disabled, reason=short_text(reason or str(exc)),
                        hand_return_feedback=feedback_summary(last_state))
        finally:
            self.hand.set_enabled(False)

    def _entry_activation_positions(self, target):
        self.monitor_hold()
        if self.hand is not None:
            sample = self.hand.feedback_snapshot(self.side)
            position = sample.get("position_unit")
            if (position is None or sample.get("age_s") is None
                    or sample["age_s"] > self.cfg.hand.max_stale_time_s
                    or max(abs(a-b) for a,b in zip(position, target["hand_unit"])) > 5):
                raise RuntimeError("held hand moved or feedback expired before following")
        state = self.snapshot()
        actual = state.get("arm_urdf_rad")
        if actual is None or self.np.max(self.np.abs(self.np.array(target["arm_urdf_rad"])-actual)) > self.np.deg2rad(1.5):
            raise RuntimeError("fresh ACTIVE target differs from reached handover pose")
        direction, offset = self.arm._mapping(self.side)
        current = direction * self.np.asarray(actual) + offset
        candidate = direction * self.np.asarray(target["arm_urdf_rad"]) + offset
        return current, candidate

    def _prepare_entry_activation(self, target, cancelled):
        current, candidate = self._entry_activation_positions(target)
        if cancelled():
            raise RuntimeError("handover cancelled before activation")
        if self.hand:
            if not self.hand.set_enabled(True):
                raise RuntimeError("hand enable failed at handover")
            try:
                if not self.arm.prepare_tracking_activation(current, candidate):
                    raise RuntimeError("hand motion envelope is unsafe at arrival; following not started")
            except BaseException:
                self.hand.set_enabled(False)
                raise
        elif not self.arm.prepare_tracking_activation(current, candidate):
            raise RuntimeError("stationary ACTIVE transition verification failed")
        if cancelled():
            if self.hand:
                self.hand.set_enabled(False)
            raise RuntimeError("handover cancelled after activation preparation")

    def activate_entry(self, target, cancelled):
        try:
            current, candidate = self._entry_activation_positions(target)
            if cancelled() or not self.arm.verify_tracking_activation(current, candidate):
                raise RuntimeError("prepared ACTIVE transition feedback/coverage verification failed")
        except BaseException:
            if self.hand:
                self.hand.set_enabled(False)
            raise

    def prepare_step(self, cancelled):
        if self.hand_only:
            state = self.snapshot()
            if state["enable_states"] != [False] * 7:
                # The mode never enables the arm. If external state changed,
                # assert the driver's existing stop path before latching fault.
                self.arm.emergency_stop()
                raise RuntimeError("arm enable detected during hand-only control")
            if state["controller_fault"] or state["driver_fault"]:
                raise RuntimeError("arm safety fault during hand-only control")
            if cancelled():
                raise RuntimeError("target expired before execution")
            return
        if self.arm.read_control_cycle_joints() is None:
            detail = getattr(self.arm, 'feedback_failure_diagnostics', None)
            raise RuntimeError(f"fresh seven-joint feedback required before execution; {detail}")
        if cancelled():
            raise RuntimeError("target expired before execution")

    def monitor_hold(self):
        self.prepare_step(lambda: False)
        if self.hand_only:
            return
        if (not self.arm._controller_is_safe()
                or not self.arm._torque_is_safe(self.arm._arm, self.side)):
            raise RuntimeError(self.arm.safety_fault_reason or f"{self.side} held arm fault")
        # Arrival -> ACTIVE may wait for multiple independent source frames.
        # Keep committing bounded hold steps while waiting; read-only polling
        # leaves the initialized command timestamp behind fresh CAN feedback.
        self.hold_arm()

    def send_arm(self, target, cancelled):
        if self.hand_only:
            raise RuntimeError("arm command forbidden in hand-only mode")
        if cancelled():
            raise RuntimeError("target expired before arm send")
        full = self.np.zeros(14)
        full[self._full_slice] = target["arm_urdf_rad"]
        ok = self.arm.command_full_urdf(full)
        if not ok and not self.arm.last_command_skipped:
            raise RuntimeError(self.arm.safety_fault_reason or "arm target rejected")

    def hold_arm(self):
        if not self.arm.hold_command_trajectory():
            raise RuntimeError(self.arm.safety_fault_reason or "arm hold rejected")

    def measured_hand_hold_target(self):
        if self.hand is None:
            return None
        position = self.hand.feedback_snapshot(self.side)["position_unit"]
        if position is None or not self.hand.feedback_ready():
            raise RuntimeError(f"{self.side} hand feedback missing during hold")
        return position

    def step(self, target, cancelled):
        self.prepare_step(cancelled)
        if target.get("hold"):
            hand_position = self.measured_hand_hold_target()
            if not self.hand_only:
                self.hold_arm()
            if cancelled():
                raise RuntimeError("hold expired before hand command")
            if hand_position is not None and not self.hand.command_physical(self.side, hand_position):
                raise RuntimeError(f"{self.side} hand hold rejected")
            return
        if self.hand_only:
            if self.hand is None or not self.hand.command_physical(self.side, target["hand_unit"]):
                raise RuntimeError("hand command rejected/feedback expired")
            return
        self.send_arm(target, cancelled)
        if self.hand:
            if cancelled() or not self.hand.command_physical(self.side, target["hand_unit"]):
                raise RuntimeError("hand command rejected/feedback expired")

    def cancel(self):
        self.arm.cancel_return()

    def stop(self):
        try:
            if self.hand:
                self.hand.set_enabled(False)
        finally:
            if not self.hand_only:
                self.arm.emergency_stop()  # NOT proof of physical stop/disable.

    def disable(self):
        if self.hand:
            self.hand.set_enabled(False)
        arm_disabled = bool(self.arm.disable())
        return arm_disabled and (self.hand is None or not self.hand.is_enabled())

    def recover_idle(self):
        """Verify a stopped hand-only session without arm return or hand motion."""
        if not self.hand_only or self.hand is None:
            raise RuntimeError("stationary recovery is only available in hand-only mode")
        if not self.disable():
            raise RuntimeError("arm/hand disable was not verified")
        state = self.snapshot()
        if state["enable_states"] != [False] * 7:
            raise RuntimeError("seven-joint arm disable feedback is unavailable or enabled")
        if state["controller_fault"] or state["driver_fault"]:
            raise RuntimeError("arm controller/driver fault requires local recovery")
        hand = state.get("hand") or {}
        if (self.hand.is_enabled() or not self.hand.feedback_ready()
                or hand.get("age_s") is None or hand["age_s"] > .15):
            raise RuntimeError("fresh disabled hand feedback is required")
        return {"disabled": True, "hands_ready": True, "stage": "hand_only_recovery"}

    def disable_side(self, side):
        if side != self.side:
            raise RuntimeError(f"gateway does not own {side} arm")
        return self.disable()

    def disable_hands(self):
        if self.hand is None:
            raise RuntimeError("gateway has no hand")
        self.hand.set_enabled(False)
        return not self.hand.is_enabled()

    def return_zero(self, cancelled, *, arm_only_collision=False,
                    stationary_hand_check=None, explicit_recovery=True, from_hold=False):
        request_id = self.arm.prepare_return()
        if request_id is None:
            return dict(returned=False, disabled=False, arm_returned=False,
                        hands_open=False, stage='busy', reason='return already running')
        hand_first = self.hand is not None
        result = dict(returned=False, disabled=False, arm_returned=False, hands_open=False)
        try:
            if self.hand:
                self.hand.set_enabled(False)
            if cancelled():
                self.arm.cancel_return()
                return dict(result, stage='preflight', reason='cancelled before return')
            if hand_first and not from_hold:
                if explicit_recovery:
                    try:
                        # Never implicitly disable a load-bearing arm.
                        result['controller_recovery_before_hand'] = self.arm.recover_disabled_controller(cancelled)
                    except Exception as exc:
                        disabled = self.arm._arm.get_joint_enable_states() == [False]*7
                        return dict(result, disabled=disabled,
                                    stage='controller_recovery_before_hand', reason=str(exc))
                    result['disabled'] = True
                result = self._restore_hand_before_return(result, cancelled)
                if not result['hands_open']:
                    return result
            if cancelled():
                self.arm.cancel_return()
                return dict(result, returned=False, stage='preflight', reason='cancelled before arm return')
            if hand_first and not from_hold:
                # Arrival may be lost between the hand worker's final check
                # and this phase boundary. Never start seven-joint motion on
                # a cached hands_open flag alone.
                result['hands_open'], errors = self.hand.open_pose_status()
                if not result['hands_open']:
                    return dict(result, returned=False, stage='hand_open_before_arm',
                                reason='hand zero lost before arm return: '+
                                self.hand.describe_open_pose_errors(errors))
            def open_held_hand():
                nonlocal result
                # Called after braking and stationary enabled feedback checks.
                result = self._restore_hand_before_return(result, cancelled, holding=True)
                if not result['hands_open']:
                    return result['reason']
                result['hands_open'], errors = self.hand.open_pose_status()
                if not result['hands_open']:
                    return 'hand zero lost before arm return: '+self.hand.describe_open_pose_errors(errors)
                return ''

            hand_phase = ({'before_arm_return': open_held_hand}
                          if hand_first and from_hold else {})
            arm_result = self.arm.safe_return(recover=not from_hold, request_id=request_id,
                preserve_hold=from_hold,
                arm_only_collision=arm_only_collision,
                near_zero_finger_exemption=hand_first,
                stationary_hand_check=(stationary_hand_check if stationary_hand_check is not None
                                       else (lambda: self.hand.open_pose_status()[0]) if hand_first else None),
                recognize_inherited_stop=explicit_recovery and not from_hold,
                **hand_phase)
            result.update(dataclasses.asdict(arm_result), arm_returned=arm_result.returned)
            if from_hold or getattr(self.arm, 'return_hold_preserved', False) is True:
                result['holding'] = getattr(self.arm, 'return_hold_preserved', False) is True
                if result['holding'] and self.hand is not None:
                    # Restore only the measured-position software command gate;
                    # this does not send a hand target or resume live following.
                    result['holding'] = bool(not cancelled() and self.hand.feedback_ready()
                                             and self.hand.set_enabled(True))
            if explicit_recovery and result['returned'] and result['disabled']:
                try:
                    result['controller_recovery'] = self.arm.recover_disabled_controller(cancelled)
                except Exception as exc:
                    disabled = self.arm._arm.get_joint_enable_states() == [False]*7
                    return dict(result, returned=False, disabled=disabled,
                                stage='controller_recovery', reason=str(exc))
            if hand_first and result['returned'] and result['disabled']:
                result['hands_open'], errors = self.hand.open_pose_status()
                if not result['hands_open']:
                    return dict(result, returned=False, stage='hand_open_verify',
                                reason='hand zero lost during arm return: '+
                                self.hand.describe_open_pose_errors(errors))
            return result
        finally:
            # Hand preflight may fail before safe_return consumes the request.
            # Do not leave a pending reservation that blocks all later retries.
            self.arm.release_return_request(request_id)

    def prepare_session(self, cancelled):
        """Verify hand zero before arm zero; automatic preparation never resets faults."""
        if not self.hand_only:
            result = self.return_zero(cancelled, explicit_recovery=False)
            if self.hand is None:
                result['hands_open'] = True
            return dict(result, hands_stationary=False)
        # Hand-only preparation keeps its existing behavior; no arm return.
        self.hand.set_enabled(False)
        if not self.hand.open_feedback_calibrated(self.side):
            raise RuntimeError(f"{self.side} hand natural-open feedback zero is not calibrated")
        hands_open, errors = self.hand.align_and_verify_open_pose(cancelled)
        if not hands_open:
            if cancelled():
                return {"returned": False, "disabled": True, "hands_open": False,
                        "stage": "hand_open", "reason": "cancelled"}
            detail = self.hand.describe_open_pose_errors(errors)
            raise RuntimeError("hand open return was not verified: " +
                               (detail or "fresh hand feedback unavailable"))
        state = self.snapshot()
        if state["enable_states"] != [False] * 7:
            raise RuntimeError("arm must remain verified disabled during hand-only preparation")
        return {"returned": True, "disabled": True, "hands_open": True,
                "hands_stationary": False, "stage": "hand_only", "reason": ""}

    def close(self):
        if self.hand and self._owns_hand:
            self.hand.close()
        self.arm.disconnect()


class DualBackend:
    """One command owner and fault domain for both arms and both hands.

    Arm writes are serialized, not hardware-synchronous. A shared hand driver
    owns the one feedback port and emits both hands in one UDP message.
    """
    supports_return = False
    supports_prepare = False

    def __init__(self, members, hand=None):
        if set(members) != {"left", "right"}:
            raise ValueError("both sides required")
        self.members, self.hand = members, hand
        # Single-arm entry capability is not a joint dual-arm planner.
        contracts = {side: {k: v for k, v in members[side].contract.items()
                            if k != "entry_protocol"} for side in ("left", "right")}
        self.contract = dict(id=hashlib.sha256(canonical(contracts)).hexdigest(),
                             side="both", with_hand=True, sides=contracts,
                             input_hold_protocol=2,
                             input_recovery_protocol=1,
                             input_recovery_enabled=all(c.get('input_recovery_enabled') is True for c in contracts.values()),
                             return_supported=False)
        self.operation_results = {}

    def attach_torque_recorder(self, writer, session, mode):
        for member in self.members.values():
            attach = getattr(member, 'attach_torque_recorder', None)
            if attach is not None:
                attach(writer, session, mode)

    def fault_diagnostics(self):
        return {side: details for side, member in self.members.items()
                if (details := member.fault_diagnostics()) is not None} or None

    def _all(self, operation):
        """Always attempt BOTH sides, even when the first CAN send fails."""
        results, errors = {}, []
        for side, member in self.members.items():
            try:
                results[side] = getattr(member, operation)()
            except Exception as exc:
                results[side] = {"error": str(exc)}
                errors.append(f"{side}: {exc}")
        self.operation_results = dict(operation=operation, sides=results)
        if errors:
            raise RuntimeError(f"dual {operation}: " + "; ".join(errors))
        return results

    def snapshot(self):
        states, sampled = {}, {}
        for side, member in self.members.items():
            states[side] = member.snapshot()
            sampled[side] = time.monotonic()
        # Snapshot acquisition of the other side can block: include that time.
        now = time.monotonic()
        for side, state in states.items():
            lag = now-sampled[side]
            if state.get("arm_feedback_age_s") is not None:
                state["arm_feedback_age_s"] += lag
            if state.get("hand") and state["hand"].get("age_s") is not None:
                state["hand"]["age_s"] += lag
        return dict(sides=states, last_operation=self.operation_results)

    def _abort(self, exc):
        if isinstance(exc, (InputHoldTransition, HoldReturnTransition)):
            raise exc
        self.cancel()
        try:
            self.stop()
        except Exception as stop_error:
            raise RuntimeError(f"{exc}; {stop_error}") from exc
        raise exc

    def enable(self, target, cancelled):
        try:
            # Check all feedback/first targets before enabling either controller.
            for side, member in self.members.items():
                member.preflight_enable(target["targets"][side])
            enabled = []

            def check_cancelled():
                if cancelled():
                    return True
                # The first arm is already holding while the second controller
                # enables/captures torque baseline; keep monitoring it as well.
                for active in enabled:
                    active.monitor_hold()
                return False

            for side, member in self.members.items():
                if check_cancelled():
                    raise RuntimeError("dual enable cancelled")
                member.enable(target["targets"][side], check_cancelled, activate_hand=False)
                enabled.append(member)
            if check_cancelled() or (self.hand is not None and not self.hand.set_enabled(True)):
                raise RuntimeError("dual hand enable failed/cancelled")
        except Exception as exc:
            self._abort(exc)

    def step(self, target, cancelled):
        try:
            for member in self.members.values():
                member.prepare_step(cancelled)
            if self.hand is not None and not self.hand.feedback_ready():
                raise RuntimeError("dual hand feedback expired")
            if target.get("hold"):
                positions = ({side: self.hand.feedback_snapshot(side)["position_unit"]
                              for side in self.members} if self.hand is not None else None)
                if positions is not None and any(value is None for value in positions.values()):
                    raise RuntimeError("dual hand feedback missing during hold")
                for member in self.members.values():
                    if cancelled():
                        raise RuntimeError("dual hold expired before arm command")
                    member.hold_arm()
                if self.hand is not None:
                    if cancelled() or not self.hand.command_physical_batch(positions):
                        raise RuntimeError("dual hand feedback missing during hold")
                return
            for side, member in self.members.items():
                member.send_arm(target["targets"][side], cancelled)
            if cancelled():
                raise RuntimeError("dual target expired before hands")
            if self.hand is not None and not self.hand.command_physical_batch({
                    side: target["targets"][side]["hand_unit"] for side in self.members}):
                raise RuntimeError("dual hand command rejected")
        except Exception as exc:
            self._abort(exc)

    def cancel(self):
        # Members' cancellation only sets Events and cannot block on hardware.
        for member in self.members.values():
            member.cancel()

    def stop(self):
        try:
            if self.hand is not None:
                self.hand.set_enabled(False)
        finally:
            self._all("stop")

    def disable(self):
        try:
            if self.hand is not None:
                self.hand.set_enabled(False)
        finally:
            results = self._all("disable")
        return all(results[side] is True for side in self.members)

    def disable_side(self, side):
        if side not in self.members:
            raise RuntimeError(f"gateway does not own {side} arm")
        result = self.members[side].disable()
        self.operation_results = dict(operation=f"disable_{side}", sides={side: result})
        return result is True

    def disable_hands(self):
        if self.hand is None:
            results = {side: member.disable_hands() for side, member in self.members.items()}
            result = all(results.values())
        else:
            # LinkerHandDriver.set_enabled(False) returns the new enabled state,
            # so False means the requested disabled state was applied.
            self.hand.set_enabled(False)
            result = not self.hand.is_enabled()
        self.operation_results = dict(operation="disable_hands", hands=result)
        return result

    def return_zero(self, cancelled):
        raise RuntimeError("dual automatic return unavailable: inter-arm swept-path checking required; no return sent")

    def prepare_session(self, cancelled):
        raise RuntimeError("dual automatic zero return requires an inter-arm swept-path checker; no motion sent")

    def close(self):
        try:
            self._all("close")
        finally:
            if self.hand is not None:
                self.hand.close()


class MockDualBackend(DualBackend):
    supports_prepare = True

    def __init__(self):
        super().__init__({side: MockBackend(side, True) for side in ("left", "right")})

    def prepare_session(self, cancelled):
        if cancelled():
            raise RuntimeError("dual mock preparation cancelled")
        for member in self.members.values():
            member.q = [0.0] * 7
            member.enabled = False
            member.hand = [128] * 10
            member.hand_enabled = False
        return {"returned": True, "disabled": True, "hands_open": True,
                "stage": "dual_mock", "reason": ""}


class DualHardwareBackend(DualBackend):
    def __init__(self, config):
        from esrobo_teleop.config import load_config
        from esrobo_teleop.robot.linker_hand_driver import LinkerHandDriver
        cfg = load_config(str(Path(config).resolve()))
        if cfg.robot.left_can_channel == cfg.robot.right_can_channel:
            raise ValueError("dual arms require distinct CAN channels")
        if (cfg.hand.model.upper() != "L10" or cfg.hand.mode != "udp"
                or not cfg.hand.require_feedback_on_enable):
            raise ValueError("dual mode requires L10 loopback feedback-gated hands")
        members, hand = {}, None
        try:
            hand = LinkerHandDriver(cfg.hand, udp_host="127.0.0.1",
                                    udp_port=cfg.hand.udp_port, active_sides=("left", "right"))
            for side in ("left", "right"):
                members[side] = HardwareBackend(config, side, True, hand_driver=hand)
            super().__init__(members, hand)
        except BaseException:
            for member in members.values():
                try:
                    member.close()
                except Exception:
                    pass
            if hand is not None:
                hand.close()
            raise
