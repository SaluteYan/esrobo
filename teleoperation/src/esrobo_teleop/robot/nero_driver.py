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

import numpy as np

from ..config import RobotConfig

# Number of physical joints on a NERO arm.
NERO_NUM_JOINTS = 7


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
        self._connected = False
        self._last_feedback_timestamp: float | None = None
        self._last_disable_states: list[bool] | None = None

    def connect(self) -> None:
        self._robot.connect()
        self._connected = True

    def enable(self, timeout: float = 8.0) -> bool:
        start = time.monotonic()
        while time.monotonic() - start < timeout:
            if self._robot.enable():
                return True
            time.sleep(0.05)
        return False

    def disable(self, timeout: float = 3.0) -> bool:
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

    def set_motion_mode(self, mode: str = "js") -> None:
        self._robot.set_motion_mode(getattr(self._robot.OPTIONS.MOTION_MODE, mode.upper()))

    def set_normal_mode(self) -> None:
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
        self._robot.reset()
        time.sleep(0.1)
        # Match the proven SenseGlove wrist arming sequence before entering the
        # silent-controller handshake: normal/motion mode, low speed, measured
        # hold target, then repeated SDK enable attempts.
        self._robot.set_normal_mode()
        self._robot.set_motion_mode(self._cfg.command_mode)
        self._robot.set_speed_percent(safe_speed)
        self._robot.move_j(values.tolist())

        deadline = time.monotonic() + max(0.5, float(timeout))
        next_configuration = 0.0
        next_status = 0.0
        attempts = 0
        side_label = getattr(self, "_side", "nero")
        while time.monotonic() < deadline:
            # The SDK's established arming path retries enable until driver
            # state feedback confirms it. A V1.12 controller may be completely
            # silent before one of these attempts opens feedback.
            self._robot.enable()
            attempts += 1
            now = time.monotonic()
            if now >= next_configuration:
                self.request_can_feedback_push()
                self._robot.set_motion_mode(self._cfg.command_mode)
                self._robot.set_speed_percent(safe_speed)
                self._robot.move_j(values.tolist())
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
        self._robot.set_speed_percent(percent)

    def get_joint_angles(self, timeout: float = 0.3) -> np.ndarray | None:
        """Read two fresh, consistent feedback samples without changing arm mode."""
        deadline = time.monotonic() + max(0.0, timeout)
        previous: np.ndarray | None = None
        previous_stamp: float | None = None
        while True:
            messages = (
                self._robot.get_joint_angles(),
                self._robot.get_leader_joint_angles(),
            )
            available = [msg for msg in messages if msg is not None]
            if available:
                latest = max(available, key=lambda msg: float(msg.timestamp))
                values = np.asarray(latest.msg, dtype=np.float64).reshape(-1)
                stamp = float(latest.timestamp)
                if (
                    values.shape == (NERO_NUM_JOINTS,)
                    and np.all(np.isfinite(values))
                    and previous is not None
                    and previous_stamp is not None
                    and stamp > previous_stamp
                    and np.max(np.abs(values - previous)) <= 0.05
                ):
                    self._last_feedback_timestamp = stamp
                    return values.copy()
                previous = values.copy()
                previous_stamp = stamp
            if time.monotonic() >= deadline:
                return None
            time.sleep(0.01)

    @property
    def last_feedback_timestamp(self) -> float | None:
        """Timestamp in seconds from the most recently accepted SDK feedback."""
        return getattr(self, "_last_feedback_timestamp", None)

    def move_j(self, joints: list[float]) -> None:
        self._robot.move_j(list(joints))

    def move_js(self, joints: list[float]) -> None:
        self._robot.move_js(list(joints))

    def emergency_stop(self) -> None:
        self._robot.electronic_emergency_stop()

    def reset(self) -> None:
        """Clear a controller-latched electronic stop without enabling motors."""
        self._robot.reset()

    def is_connected(self) -> bool:
        return self._connected and self._robot.is_connected()

    def disconnect(self) -> None:
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

    def connect(self) -> None:
        self._left.connect()
        self._right.connect()

    def enable(self) -> bool:
        ok_left = self._left.enable()
        ok_right = self._right.enable()
        if ok_left:
            self._left.set_motion_mode(self._cfg.command_mode)
        if ok_right:
            self._right.set_motion_mode(self._cfg.command_mode)
        return ok_left and ok_right

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
        lower_urdf = np.asarray(
            self._cfg.joint_soft_lower_limits_urdf, dtype=np.float64
        ).reshape(-1)
        upper_urdf = np.asarray(
            self._cfg.joint_soft_upper_limits_urdf, dtype=np.float64
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

    def _clamp(self, joints: np.ndarray, current: np.ndarray, side: str, dt: float) -> np.ndarray:
        joints = np.asarray(joints, dtype=np.float64)
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
        channel = cfg.left_can_channel if side == "left" else cfg.right_can_channel
        self._arm = NeroArm(cfg, channel, side)
        self._session_start: dict[str, np.ndarray] | None = None
        self._last_command_feedback_timestamp: float | None = None
        self._command_velocity = {side: np.zeros(NERO_NUM_JOINTS, dtype=np.float64)}
        self._last_commanded_full_urdf: np.ndarray | None = None
        self._enabled = False
        self._silent_disabled_startup = False

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

    def enable_from_natural_down(self) -> bool:
        """Enable a silent left arm at URDF zero, then verify fresh feedback."""
        if not self._silent_disabled_startup:
            return False
        _, zero_physical = self._mapping(self._side)
        joints, states = self._arm.enable_at_known_target(
            zero_physical,
            speed_percent=self._cfg.speed_percent,
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
            self._arm.best_effort_disable_once()
            self._enabled = False
            return False
        self._silent_disabled_startup = False
        self._enabled = True
        # The wake-up handshake is deliberately capped at 10%. Restore the
        # configured tracking speed only after zero feedback is verified.
        self._arm.set_speed_percent(self._cfg.speed_percent)
        self._arm.move_j(zero_physical.tolist())
        return True

    def read_joints(self) -> np.ndarray | None:
        joints = self._arm.get_joint_angles()
        if not self._valid_joints(joints):
            return None
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

    def enable(self) -> bool:
        joints = self.read_joints()
        if joints is None:
            return False
        # Preload the measured pose while disabled so enable cannot recall an
        # old target from a previous controller session.
        self._arm.set_normal_mode()
        self._arm.set_motion_mode(self._cfg.command_mode)
        self._arm.move_j(joints.tolist())
        if not self._arm.enable():
            return False
        states = self._arm.get_joint_enable_states()
        if states is None or len(states) != NERO_NUM_JOINTS or not all(states):
            self._arm.disable()
            return False
        self._enabled = True
        self._arm.move_j(joints.tolist())
        return True

    def align_startup_zero_and_disable(self) -> tuple[bool, bool]:
        """Enable at measured feedback, rate-limit to URDF zero, then disable."""
        if not self.capture_session_start():
            self._arm.best_effort_disable_once()
            return False, False
        if not self.enable():
            disabled = self._arm.disable()
            self._enabled = not disabled
            return False, disabled
        return self.return_to_zero_and_disable()

    def set_speed(self, percent: int = 100) -> None:
        self._arm.set_speed_percent(percent)

    def command_full_urdf(
        self, full_urdf_targets: np.ndarray, _legacy_loop_dt: float | None = None
    ) -> bool:
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
        return True

    def last_commanded_full_urdf(self) -> np.ndarray | None:
        value = self._last_commanded_full_urdf
        return None if value is None else value.copy()

    def return_to_zero_and_disable(self) -> tuple[bool, bool]:
        """Return distal then proximal joints to URDF zero before disabling."""
        returned = False
        if self._enabled:
            directions, zero_physical = self._mapping(self._side)
            tolerance = np.deg2rad(max(0.1, float(self._cfg.full_arm_return_tolerance_deg)))
            timeout = max(0.5, float(self._cfg.full_arm_return_timeout_s))
            current = self.read_joints()
            if current is not None:
                side_slice = (
                    slice(0, NERO_NUM_JOINTS)
                    if self._side == "left"
                    else slice(NERO_NUM_JOINTS, 2 * NERO_NUM_JOINTS)
                )
                held_urdf = (current - zero_physical) / directions
                distal_target = np.zeros(2 * NERO_NUM_JOINTS, dtype=np.float64)
                distal_target[side_slice] = held_urdf
                distal_target[side_slice][3:7] = 0.0

                # Remove residual velocity from active teleoperation before
                # holding J1-J3 and retracting J4-J7.
                self._command_velocity[self._side] = np.zeros(
                    NERO_NUM_JOINTS, dtype=np.float64
                )
                print(
                    f"[{self._side}-arm] Return phase 1/2: holding J1-J3; "
                    "slowly returning J4-J7 to natural-down zero.",
                    flush=True,
                )
                distal_returned = self._return_phase_with_retries(
                    distal_target,
                    np.arange(3, 7),
                    zero_physical,
                    tolerance,
                    timeout,
                    "1/2 J4-J7",
                )
                if distal_returned:
                    self._command_velocity[self._side] = np.zeros(
                        NERO_NUM_JOINTS, dtype=np.float64
                    )
                    print(
                        f"[{self._side}-arm] Return phase 2/2: J4-J7 verified; "
                        "slowly returning J1-J3 and rechecking J1-J7.",
                        flush=True,
                    )
                    returned = self._return_phase_with_retries(
                        np.zeros(2 * NERO_NUM_JOINTS, dtype=np.float64),
                        np.arange(0, NERO_NUM_JOINTS),
                        zero_physical,
                        tolerance,
                        timeout,
                        "2/2 J1-J7",
                    )
                else:
                    print(
                        f"[{self._side}-arm] Return phase 1/2 was not verified; "
                        "J1-J3 return was not started.",
                        flush=True,
                    )
        disabled = False
        if returned or bool(self._cfg.disable_on_unconfirmed_return):
            disabled = self._arm.disable()
        elif self._enabled:
            print(
                f"[{self._side}-arm] ZERO RETURN NOT CONFIRMED; automatic disable "
                "is inhibited. Following is stopped and the arm remains enabled at "
                "the controller's last target. Operator action is required.",
                flush=True,
            )
        self._enabled = not disabled
        return returned, disabled

    def _return_phase_with_retries(
        self,
        full_target: np.ndarray,
        indices: np.ndarray,
        zero_physical: np.ndarray,
        tolerance: float,
        timeout: float,
        phase_label: str,
    ) -> bool:
        """Retry a bounded return phase before falling back to disable."""
        attempts = max(1, int(self._cfg.return_phase_attempts))
        for attempt in range(1, attempts + 1):
            if self._return_group_to_zero(
                full_target,
                indices,
                zero_physical,
                tolerance,
                timeout,
                phase_label=phase_label,
            ):
                return True
            if attempt < attempts:
                self._command_velocity[self._side] = np.zeros(
                    NERO_NUM_JOINTS, dtype=np.float64
                )
                self._arm.request_can_feedback_push()
                print(
                    f"[{self._side}-arm] Return phase {phase_label} was not "
                    f"verified; retrying bounded return ({attempt + 1}/{attempts}).",
                    flush=True,
                )
        return False

    def _return_group_to_zero(
        self,
        full_target: np.ndarray,
        indices: np.ndarray,
        zero_physical: np.ndarray,
        tolerance: float,
        timeout: float,
        phase_label: str = "",
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
        deadline = time.monotonic() + timeout
        feedback_gap_started: float | None = None
        previous_stamp: float | None = None
        verified_samples = 0
        required_samples = max(1, int(self._cfg.return_verify_samples))
        feedback_grace = max(0.0, float(self._cfg.return_feedback_grace_s))
        next_status = time.monotonic() + 1.0
        last_error_deg: np.ndarray | None = None
        while time.monotonic() < deadline:
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
            if previous_stamp is None:
                # Start each return phase from measured feedback. Do not inherit
                # a timestamp or velocity state from teleoperation/the prior phase.
                previous_stamp = float(stamp)
                self._last_command_feedback_timestamp = float(stamp)
                self._command_velocity[self._side] = np.zeros(
                    NERO_NUM_JOINTS, dtype=np.float64
                )
                self._arm.move_j(current.tolist())
            else:
                dt = float(stamp - previous_stamp)
                previous_stamp = float(stamp)
                try:
                    limited = self._clamp(target_physical, current, self._side, dt)
                except (TypeError, ValueError):
                    print(
                        f"[{self._side}-arm] Return phase {phase_label} rejected "
                        "an invalid feedback interval.",
                        flush=True,
                    )
                    return False
                self._arm.move_j(limited.tolist())
                self._last_command_feedback_timestamp = float(stamp)
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
            if np.max(np.abs(current[indices] - zero_physical[indices])) <= tolerance:
                verified_samples += 1
                if verified_samples >= required_samples:
                    final_target = current.copy()
                    final_target[indices] = zero_physical[indices]
                    self._arm.move_j(final_target.tolist())
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
        disabled = self._arm.disable()
        self._enabled = not disabled
        return disabled

    def emergency_stop(self) -> None:
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
        # are still disabled. This prevents enable from restoring an old target.
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
