"""Mock and real robot executors. Hardware imports are lazy and explicit."""
import dataclasses
import hashlib
from pathlib import Path
import time

from .protocol import canonical


class MockBackend:
    """No CAN, ROS, PICO, glove, numpy, or graphical dependencies."""
    def __init__(self, side="left", with_hand=False):
        self.contract = dict(id=f"mock-v1-{side}-{with_hand}", side=side, with_hand=with_hand,
                             arm_unit="URDF radians", hand_unit="L10 physical 0..255",
                             lower_rad=[-1.0] * 7, upper_rad=[1.0] * 7)
        self.q = [0.0] * 7
        self.hand = [128] * 10 if with_hand else None
        self.enabled = False
        self.stops = 0

    def snapshot(self):
        return dict(arm_urdf_rad=list(self.q), arm_feedback_age_s=0.0,
                    enable_states=[self.enabled] * 7,
                    hand=None if self.hand is None else dict(position_unit=list(self.hand), age_s=0.0))

    def preflight_enable(self, target):
        pass

    def enable(self, target, cancelled, *, activate_hand=True):
        if cancelled():
            raise RuntimeError("enable cancelled")
        self.enabled = True

    def step(self, target, cancelled):
        if cancelled() or not self.enabled:
            raise RuntimeError("not enabled/cancelled")
        self.q = [q + max(-.005, min(.005, t - q)) for q, t in zip(self.q, target["arm_urdf_rad"])]
        if self.hand is not None:
            self.hand = list(target["hand_unit"])

    def prepare_step(self, cancelled):
        if cancelled():
            raise RuntimeError("cancelled")

    def send_arm(self, target, cancelled):
        self.step(target, cancelled)

    def monitor_hold(self):
        if not self.enabled:
            raise RuntimeError("mock held arm disabled")

    def stop(self):
        self.stops += 1
        self.enabled = False

    def cancel(self):
        pass

    def disable(self):
        self.enabled = False
        return True

    def return_zero(self, cancelled):
        if cancelled():
            raise RuntimeError("return cancelled")
        self.q = [0.0] * 7
        self.enabled = False
        return {"returned": True, "disabled": True, "stage": "mock", "reason": ""}

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
    def __init__(self, config, side, with_hand, *, hand_driver=None):
        import numpy as np
        from esrobo_teleop.config import load_config
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
        self.cfg, self.side, self.np = cfg, side, np
        self.arm = NeroSingleArmDriver(cfg.robot, side)
        self.hand = None
        self._owns_hand = hand_driver is None
        self._full_slice = slice(0, 7) if side == "left" else slice(7, 14)
        model = ForwardModel(cfg.ik)
        self.arm.configure_command_trajectory(model.fk(side), cfg.retarget.max_endpoint_translation_velocity_m_s)
        guard = TorsoCollisionGuard(model, side, cfg.robot.collision_package_dirs,
                                   cfg.robot.torso_collision_margin_m,
                                   cfg.robot.shoulder_collision_margin_m, include_fingers=with_hand)
        self.arm.configure_collision_guard(guard)
        try:
            if with_hand:
                self.hand = hand_driver if hand_driver is not None else LinkerHandDriver(cfg.hand, udp_host="127.0.0.1",
                                             udp_port=cfg.hand.udp_port, active_sides=(side,))
                guard.hand_state_provider = lambda horizon: self.hand.geometry_state(side, horizon)
            lo, hi = self.arm._effective_position_limits(side)
            direction, offset = self.arm._mapping(side)
            a, b = (lo - offset) / direction, (hi - offset) / direction
            fingerprint = dict(robot=dataclasses.asdict(cfg.robot), hand=dataclasses.asdict(cfg.hand),
                               ik={k:v for k,v in dataclasses.asdict(cfg.ik).items() if k != "urdf_path"},
                               endpoint_speed=cfg.retarget.max_endpoint_translation_velocity_m_s,
                               model_sha256=hashlib.sha256(model_path.read_bytes()).hexdigest(),
                               side=side, with_hand=with_hand)
            self.contract = dict(id=hashlib.sha256(canonical(fingerprint)).hexdigest(),
                                 side=side, with_hand=with_hand, arm_unit="URDF radians",
                                 hand_unit="L10 physical 0..255", hand_order=L10_PHYSICAL_JOINT_NAMES,
                                 arm_order=list(getattr(cfg.ik, f"{side}_arm_joints")),
                                 lower_rad=np.minimum(a, b).tolist(), upper_rad=np.maximum(a, b).tolist(),
                                 physical_direction=direction.tolist(), physical_offset_rad=offset.tolist(),
                                 velocity_rad_s=self.arm._joint_rate_limit_vector(side, "max_joint_velocity").tolist(),
                                 acceleration_rad_s2=self.arm._joint_rate_limit_vector(side, "max_joint_acceleration").tolist())
            self.arm.connect()
        except BaseException:
            self.close()
            raise

    def snapshot(self):
        full = self.arm.read_control_cycle_joints()
        snap = getattr(self.arm, "_cycle_feedback", None)
        states = []
        for index in range(1, 8):
            state = self.arm._arm._robot.get_driver_states(index)
            if (state is None or not self.np.isfinite(state.timestamp)
                    or not 0 <= time.time() - state.timestamp <= self.cfg.robot.controller_status_timeout_s):
                states = None
                break
            states.append(bool(state.msg.foc_status.driver_enable_status))
        return dict(arm_urdf_rad=None if full is None else full[self._full_slice].tolist(),
                    arm_feedback_age_s=None if snap is None else max(0.0, time.monotonic() - snap.received_monotonic),
                    enable_states=states,
                    controller_fault=self.arm._arm.controller_fault(allow_disabled=True),
                    driver_fault=self.arm.safety_fault_reason,
                    hand=None if self.hand is None else self.hand.feedback_snapshot(self.side))

    def preflight_enable(self, target):
        # No silent-controller wake, blind enable or implicit homing over network.
        state = self.snapshot()
        if state["arm_urdf_rad"] is None or state["enable_states"] != [False] * 7:
            raise RuntimeError("fresh seven-joint feedback and verified disable required; use local commissioning if silent")
        if state["controller_fault"]:
            raise RuntimeError(f"{self.side} controller preflight: {state['controller_fault']}")
        if self.np.max(self.np.abs(self.np.array(target["arm_urdf_rad"]) - state["arm_urdf_rad"])) > self.np.deg2rad(1.5):
            raise RuntimeError("first target must match measured arm pose within 1.5 degrees")
        if self.hand is not None:
            if not self.hand.open_feedback_calibrated(self.side) or not self.hand.open_pose_status()[0]:
                raise RuntimeError("hand must already be at verified natural-open feedback zero")
            current = self.hand.feedback_snapshot(self.side)["position_unit"]
            if current is None or max(abs(a-b) for a,b in zip(current, target["hand_unit"])) > self.cfg.hand.startup_open_tolerance:
                raise RuntimeError("first hand target must match measured open pose")

    def enable(self, target, cancelled, *, activate_hand=True):
        self.preflight_enable(target)
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

    def prepare_step(self, cancelled):
        if self.arm.read_control_cycle_joints() is None:
            raise RuntimeError("fresh seven-joint feedback required before execution")
        if cancelled():
            raise RuntimeError("target expired before execution")

    def monitor_hold(self):
        self.prepare_step(lambda: False)
        if (not self.arm._controller_is_safe()
                or not self.arm._torque_is_safe(self.arm._arm, self.side)):
            raise RuntimeError(self.arm.safety_fault_reason or f"{self.side} held arm fault")

    def send_arm(self, target, cancelled):
        if cancelled():
            raise RuntimeError("target expired before arm send")
        full = self.np.zeros(14)
        full[self._full_slice] = target["arm_urdf_rad"]
        ok = self.arm.command_full_urdf(full)
        if not ok and not self.arm.last_command_skipped:
            raise RuntimeError(self.arm.safety_fault_reason or "arm target rejected")

    def step(self, target, cancelled):
        self.prepare_step(cancelled)
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
            self.arm.emergency_stop()  # NOT proof of physical stop/disable.

    def disable(self):
        if self.hand:
            self.hand.set_enabled(False)
        return bool(self.arm.disable())

    def return_zero(self, cancelled):
        if self.hand:
            self.hand.set_enabled(False)
        request_id = self.arm.prepare_return()
        if cancelled():
            self.arm.cancel_return()
        result = self.arm.safe_return(recover=True, request_id=request_id)
        return dataclasses.asdict(result)

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

    def __init__(self, members, hand=None):
        if set(members) != {"left", "right"}:
            raise ValueError("both sides required")
        self.members, self.hand = members, hand
        contracts = {side: members[side].contract for side in ("left", "right")}
        self.contract = dict(id=hashlib.sha256(canonical(contracts)).hexdigest(),
                             side="both", with_hand=True, sides=contracts,
                             return_supported=False)
        self.operation_results = {}

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

    def return_zero(self, cancelled):
        raise RuntimeError("dual automatic return unavailable: inter-arm swept-path checking required; no return sent")

    def close(self):
        try:
            self._all("close")
        finally:
            if self.hand is not None:
                self.hand.close()


class MockDualBackend(DualBackend):
    def __init__(self):
        super().__init__({side: MockBackend(side, True) for side in ("left", "right")})


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
