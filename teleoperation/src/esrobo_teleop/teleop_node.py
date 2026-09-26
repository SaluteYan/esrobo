"""Real-machine dual-arm + LinkerHand teleoperation node for the ESROBO robot.

Wiring:  PICO full-body UDP (BodyDevice) -> arm_vector retargeting -> Pink IK
(IkSolver) -> NERO arms (NeroDualArmDriver).  SenseGlove hand joints from the
same UDP stream -> LinkerHand (LinkerHandDriver).

Safety: the robot arms are only commanded while teleoperation is armed
(default off).  Press 'e' to enable/arm, 's' to stop/hold, 'q' to quit.
"""

from __future__ import annotations

import argparse
from contextlib import nullcontext
import json
import os
import signal
import select
import socket
import sys
import threading
import time
from pathlib import Path

import numpy as np
from .debug.async_json_log import AsyncJsonLog, numpy_json_default
from .debug.arm_probe import ArmProbe

from . import math_utils as mu
from .config import TeleopConfig, build_config, load_config
from .device.body_device import BodyDevice
from .ik.solver import IkSolver
from .robot.linker_hand_driver import LinkerHandDriver
from .robot.nero_driver import (
    NeroDualArmDriver,
    NeroSingleArmDriver,
    NeroWristDriver,
    imu_local_rotation_to_wrist_offsets,
)


class TeleopNode:
    def __init__(self, cfg: TeleopConfig, enable_robot: bool = True, hand_mode: str = "udp",
                 hand_only: bool = False, hand_side: str = "both",
                 right_wrist_imu: bool = False, left_wrist_imu: bool = False,
                 arm_only: bool = False, arm_side: str = "left",
                 arm_with_hand: bool = False):
        self._cfg = cfg
        self._enable_robot = enable_robot
        self._hand_only = hand_only
        self._arm_only = arm_only
        self._arm_with_hand = bool(arm_only and arm_with_hand)
        self._arm_side = arm_side
        if right_wrist_imu and left_wrist_imu:
            raise ValueError("only one wrist IMU side can be active")
        self._wrist_imu_side = "right" if right_wrist_imu else "left" if left_wrist_imu else None
        self._arm_armed = False
        self._hand_enabled = False
        self._return_hand_open_on_close = False
        self._stop = False
        self._manual_intervention_required = False
        self._return_thread = None
        self._startup_alignment = False
        self._startup_alignment_required = False
        self._arming_thread = None
        self._arming_cancel = threading.Event()
        self._quit_after_arming_cancel = False
        self._quit_after_return = False
        self._startup_power_cycle_required = False
        # The keyboard/arming worker and the real-time loop both touch the
        # stateful retarget filters and IK seed.  Keep startup rebasing and
        # session initialization atomic with respect to preview/control solves.
        self._arm_control_lock = threading.RLock()

        self._body = BodyDevice(cfg.retarget, active_arm_side=arm_side if arm_only else None)
        cfg.ik.urdf_path = cfg.ik.urdf_path or default_urdf_path()
        self._ik = None if hand_only else IkSolver(cfg.ik)

        self._driver = None
        if enable_robot and not hand_only:
            self._driver = (
                NeroWristDriver(cfg.robot, self._wrist_imu_side)
                if self._wrist_imu_side
                else NeroSingleArmDriver(cfg.robot, arm_side)
                if arm_only
                else NeroDualArmDriver(cfg.robot)
            )
            if arm_only and not self._wrist_imu_side and self._driver.command_trajectory_enabled:
                physical_lower, physical_upper = self._driver._effective_position_limits(arm_side)
                directions, offsets = self._driver._mapping(arm_side)
                a, b = (physical_lower - offsets) / directions, (physical_upper - offsets) / directions
                self._ik.configure_position_envelope(arm_side, np.minimum(a, b), np.maximum(a, b))
                self._driver.configure_command_trajectory(
                    self._ik.make_command_fk(arm_side),
                    cfg.retarget.max_endpoint_translation_velocity_m_s,
                )
                if cfg.robot.torso_collision_enabled:
                    from .robot.torso_collision import TorsoCollisionGuard
                    # Fail closed before connect/enable if geometry is missing.
                    self._driver.configure_collision_guard(TorsoCollisionGuard(
                        self._ik, arm_side, cfg.robot.collision_package_dirs,
                        cfg.robot.torso_collision_margin_for(arm_side),
                        cfg.robot.shoulder_collision_margin_m,
                        include_fingers=self._arm_with_hand))

        active_sides = (arm_side,) if arm_only else (("left", "right") if hand_side == "both" else (hand_side,))
        self._hand = None if arm_only and not self._arm_with_hand else LinkerHandDriver(
            cfg.hand, udp_port=cfg.hand.udp_port, active_sides=active_sides
        )

        if arm_only and self._arm_with_hand and self._driver is not None:
            guard = getattr(self._driver, "_collision_guard", None)
            if guard is not None:
                guard.hand_state_provider = lambda horizon: self._hand.geometry_state(arm_side, horizon)

        self._current_arm_joints: np.ndarray | None = None
        self._motors_enabled = False
        self._wrist_local_axes: np.ndarray | None = None
        self._full_arm_wrist_local_axes: np.ndarray | None = None
        imu_map_side = self._wrist_imu_side or (arm_side if arm_only else "left")
        self._glove_imu_to_palm = np.asarray(
            cfg.retarget.hand_imu_local_rotvec_map_for(imu_map_side),
            dtype=np.float64,
        ).reshape(3, 3)
        self._last_wrist_diagnostic_time = 0.0
        self._wrist_diagnostics_enabled = False
        self._command_rejected = False
        self._position_limit_hold_active = False
        self._position_limit_recovery_count = 0
        self._arm_diagnostics_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if arm_only else None
        self._last_arm_diagnostics_time = 0.0
        self._elbow_log_path = Path(__file__).resolve().parents[2] / "log" / f"pico_elbow_{time.time_ns()}.jsonl"
        self._elbow_log_failed = False
        self._diagnostic_writer = None
        self._startup_writer = None
        self._arm_probe = None
        if arm_only and self._driver is not None:
            self._driver.startup_event_sink = self._record_startup_event
        self._key_thread = None
        if self._wrist_imu_side:
            self._configure_wrist_mapping(np.zeros(7, dtype=np.float64))

    def _configure_full_arm_wrist_mapping(self, full_urdf_joints: np.ndarray) -> None:
        """Linearize the active URDF wrist at its physical neutral position."""
        if not self._arm_only or self._ik is None:
            raise ValueError("full-arm wrist mapping requires single-arm mode")
        side = self._arm_side
        values = np.asarray(full_urdf_joints, dtype=np.float64).reshape(14).copy()
        indices = np.asarray(self._cfg.robot.right_wrist_joint_indices, dtype=int)
        directions = np.asarray(
            self._cfg.robot.left_joint_directions
            if side == "left"
            else self._cfg.robot.right_joint_directions,
            dtype=np.float64,
        )
        offsets = np.asarray(
            self._cfg.robot.left_joint_offsets
            if side == "left"
            else self._cfg.robot.right_joint_offsets,
            dtype=np.float64,
        )
        neutral = np.asarray(
            self._cfg.robot.wrist_neutral_joint_positions, dtype=np.float64
        ).reshape(-1)
        if indices.shape != (3,) or neutral.shape != (3,):
            raise ValueError("full-arm wrist mapping requires exactly J5/J6/J7")
        side_offset = 0 if side == "left" else 7
        values[side_offset + indices] = (neutral - offsets[indices]) / directions[indices]
        _, urdf_axes = self._ik.wrist_orientation_linearization(
            side, values, tuple(indices.tolist())
        )
        self._full_arm_wrist_local_axes = urdf_axes / directions[indices][np.newaxis, :]

    def _full_arm_terminal_targets(self, imu_delta: np.ndarray) -> np.ndarray:
        """Convert calibrated palm-local IMU rotation to URDF J5/J6/J7 targets."""
        if self._full_arm_wrist_local_axes is None:
            raise ValueError("full-arm wrist coordinate mapping is unavailable")
        side = self._arm_side
        indices = np.asarray(self._cfg.robot.right_wrist_joint_indices, dtype=int)
        directions = np.asarray(
            self._cfg.robot.left_joint_directions
            if side == "left"
            else self._cfg.robot.right_joint_directions,
            dtype=np.float64,
        )
        offsets = np.asarray(
            self._cfg.robot.left_joint_offsets
            if side == "left"
            else self._cfg.robot.right_joint_offsets,
            dtype=np.float64,
        )
        neutral = np.asarray(
            self._cfg.robot.wrist_neutral_joint_positions, dtype=np.float64
        )
        physical_delta = imu_local_rotation_to_wrist_offsets(
            imu_delta,
            self._cfg.robot,
            self._full_arm_wrist_local_axes,
            limit_total_angle=side != "right",
        )
        return (neutral + physical_delta - offsets[indices]) / directions[indices]

    def _compensated_full_arm_terminal_targets(
        self, desired_wrist_pose: np.ndarray, arm_position_targets: np.ndarray
    ) -> np.ndarray:
        """Remove J1-J4 palm rotation before mapping glove IMU to J5-J7."""
        if self._ik is None:
            raise ValueError("full-arm IMU compensation requires the IK model")
        side = self._arm_side
        indices = np.asarray(self._cfg.robot.right_wrist_joint_indices, dtype=int)
        directions = np.asarray(
            self._cfg.robot.left_joint_directions
            if side == "left"
            else self._cfg.robot.right_joint_directions,
            dtype=np.float64,
        )
        offsets = np.asarray(
            self._cfg.robot.left_joint_offsets
            if side == "left"
            else self._cfg.robot.right_joint_offsets,
            dtype=np.float64,
        )
        neutral = np.asarray(
            self._cfg.robot.wrist_neutral_joint_positions, dtype=np.float64
        )
        side_offset = 0 if side == "left" else 7
        neutral_arm = np.asarray(arm_position_targets, dtype=np.float64).reshape(14).copy()
        neutral_arm[side_offset + indices] = (
            neutral - offsets[indices]
        ) / directions[indices]
        baseline_pose = self._ik.current_task_frame_poses(neutral_arm)[f"{side}_wrist"]
        baseline_world = mu.quat_wxyz_to_matrix(baseline_pose[3:7]).astype(np.float64)
        desired = np.asarray(desired_wrist_pose, dtype=np.float64).reshape(7)
        desired_world = mu.quat_wxyz_to_matrix(desired[3:7]).astype(np.float64)
        residual_local = baseline_world.T @ desired_world
        return self._full_arm_terminal_targets(residual_local)

    def _configure_wrist_mapping(self, physical_joints: np.ndarray) -> None:
        """Linearize terminal joints at their fixed zero in robot-palm coordinates."""
        side = self._wrist_imu_side
        if side not in ("left", "right") or self._ik is None:
            raise ValueError("wrist mapping requires an active side and IK solver")
        values = np.asarray(physical_joints, dtype=np.float64).reshape(7).copy()
        indices = np.asarray(self._cfg.robot.right_wrist_joint_indices, dtype=int)
        neutral = np.asarray(
            self._cfg.robot.wrist_neutral_joint_positions, dtype=np.float64
        ).reshape(-1)
        directions = np.asarray(
            self._cfg.robot.left_joint_directions
            if side == "left"
            else self._cfg.robot.right_joint_directions,
            dtype=np.float64,
        )
        offsets = np.asarray(
            self._cfg.robot.left_joint_offsets
            if side == "left"
            else self._cfg.robot.right_joint_offsets,
            dtype=np.float64,
        )
        if neutral.shape != indices.shape or not np.all(np.isin(directions, (-1.0, 1.0))):
            raise ValueError("invalid wrist neutral/direction configuration")
        values[indices] = neutral
        side_urdf = (values - offsets) / directions
        full_urdf = np.zeros(14, dtype=np.float64)
        if side == "left":
            full_urdf[:7] = side_urdf
        else:
            full_urdf[7:] = side_urdf
        _, urdf_axes = self._ik.wrist_orientation_linearization(
            side, full_urdf, tuple(indices.tolist())
        )
        self._wrist_local_axes = urdf_axes / directions[indices][np.newaxis, :]

    def _wrist_offsets(self, imu_local_delta: np.ndarray) -> np.ndarray:
        if self._wrist_local_axes is None:
            raise ValueError("wrist coordinate mapping is unavailable")
        return imu_local_rotation_to_wrist_offsets(
            imu_local_delta,
            self._cfg.robot,
            self._wrist_local_axes,
            limit_total_angle=self._wrist_imu_side != "right",
        )

    def _print_wrist_diagnostics(
        self, imu_local_delta: np.ndarray, offsets: np.ndarray, now: float
    ) -> None:
        if not self._wrist_diagnostics_enabled:
            return
        if now - self._last_wrist_diagnostic_time < 0.5:
            return
        self._last_wrist_diagnostic_time = now
        palm_rotvec = mu.rotation_matrix_to_rotvec(imu_local_delta)
        glove_components = None
        component_reader = getattr(self._body, "hand_orientation_components", None)
        if callable(component_reader) and self._wrist_imu_side:
            try:
                candidate = np.asarray(
                    component_reader(self._wrist_imu_side), dtype=np.float64
                ).reshape(-1)
                if candidate.shape == (3,) and np.all(np.isfinite(candidate)):
                    glove_components = candidate
            except (TypeError, ValueError):
                pass
        if glove_components is None:
            glove_components = np.linalg.solve(self._glove_imu_to_palm, palm_rotvec)
        glove_deg = np.degrees(glove_components)
        joint_deg = np.degrees(offsets)
        state = "ARMED" if self._arm_armed and self._driver is not None else "NO MOTION"
        print(
            "[teleop IMU] glove raw anatomical XYZ(deg)="
            f"{np.array2string(glove_deg, precision=1)} "
            f"=> J5/J6/J7(deg)={np.array2string(joint_deg, precision=1)} [{state}]",
            flush=True,
        )

    def _return_wrist_to_zero_and_disable(
        self, reason: str, *, allow_return: bool = True
    ) -> tuple[bool, bool]:
        """Stop following, safely return the terminal joints, then disable."""
        self._arm_armed = False
        side = self._wrist_imu_side.upper() if self._wrist_imu_side else "WRIST"
        if self._driver is None or not self._motors_enabled:
            print(f"[teleop] {side} WRIST STOPPED: {reason}; arm is already disabled.", flush=True)
            return False, True

        returned = False
        if allow_return:
            print(
                f"[teleop] {side} WRIST STOPPED: {reason}; slowly returning "
                "J5/J6/J7 to zero before disable.",
                flush=True,
            )
            try:
                returned = bool(self._driver.return_wrist_to_neutral())
            except Exception as exc:  # noqa: BLE001
                print(f"[teleop] {side} WRIST ZERO RETURN FAILED: {exc}", flush=True)
        else:
            print(
                f"[teleop] {side} WRIST STOPPED: {reason}; zero return skipped "
                "because valid feedback/control is unavailable.",
                flush=True,
            )

        disabled = False
        try:
            disabled = bool(self._driver.disable())
        except Exception as exc:  # noqa: BLE001
            print(f"[teleop] {side} ARM DISABLE FAILED: {exc}", flush=True)
        # Keep the uncertain state set so close()/disconnect retries disable.
        self._motors_enabled = not disabled
        return_status = (
            "RETURNED TO J5/J6/J7 ZERO"
            if returned
            else "ZERO RETURN NOT CONFIRMED" if allow_return else "ZERO RETURN SKIPPED"
        )
        disable_status = "DISABLED" if disabled else "DISABLE NOT CONFIRMED"
        print(f"[teleop] {side} WRIST {return_status}; ARM {disable_status}.", flush=True)
        return returned, disabled

    def _stop_wrist_for_fault(self, reason: str, *, allow_return: bool = True) -> None:
        self._return_wrist_to_zero_and_disable(reason, allow_return=allow_return)

    def start(self) -> None:
        if self._driver is not None:
            self._driver.connect()
            if self._arm_only and os.environ.get("ESROBO_ARM_DIAGNOSTICS", "0") == "1":
                path = self._elbow_log_path.with_name(self._elbow_log_path.name.replace("pico_elbow", "pico_arm_probe"))
                self._arm_probe = ArmProbe(path, self._driver, lambda: dict(
                    armed=self._arm_armed, motors_enabled=self._motors_enabled,
                    manual_intervention=bool(getattr(self, "_manual_intervention_required", False))))
                self._driver.probe_sink = self._arm_probe.submit
                print(f"[teleop] passive arm diagnostics: {path}; raw CAN: {self._arm_probe.raw_path}", flush=True)
            if self._arm_only:
                label = self._arm_side.capitalize()
                self._current_arm_joints = self._driver.read_full_urdf_joints()
                physical = self._driver.read_joints()
                if self._driver.silent_disabled_startup:
                    print(
                        f"[teleop] {label} NERO CAN is silent in its configured power-on "
                        "disabled state; no enable or position command has been sent. "
                        "Feedback wake and zero verification are deferred until 'e'.",
                        flush=True,
                    )
                else:
                    print(
                        f"[teleop] {label} NERO connected over CAN; all seven joints are DISABLED.",
                        flush=True,
                    )
                if physical is not None:
                    print(
                        f"[teleop] {label} NERO physical feedback(rad): "
                        + np.array2string(physical, precision=4),
                        flush=True,
                    )
            elif self._wrist_imu_side:
                label = self._wrist_imu_side.capitalize()
                print(f"[teleop] {label} NERO connected over CAN and forced DISABLED.", flush=True)
                self._current_arm_joints = self._driver.read_joints()
                if self._current_arm_joints is not None:
                    print(
                        f"[teleop] {label} NERO feedback(rad): "
                        + np.array2string(self._current_arm_joints, precision=4),
                        flush=True,
                    )
                else:
                    print(
                        f"[teleop] {label} NERO feedback unavailable while disabled; "
                        "pressing 'e' will be refused until feedback is valid.",
                        flush=True,
                    )
            else:
                print("[teleop] NERO arms connected.", flush=True)
                self._current_arm_joints = self._driver.read_full_urdf_joints()
        self._ensure_startup_zero_pose()
        if getattr(self, "_key_thread", None) is None:
            self._key_thread = threading.Thread(target=self._key_loop, daemon=True)
            self._key_thread.start()

    def _ensure_startup_zero_pose(self) -> None:
        """Inspect startup zero; defer full-arm motion to the single-key arm gate."""
        if getattr(self, "_arm_only", False):
            if self._driver is None:
                return
            error = self._driver.startup_zero_error()
            if error is None:
                if self._driver.silent_disabled_startup:
                    print(
                        f"[teleop ZERO] {self._arm_side.upper()} zero verification deferred: "
                        "the power-on-disabled controller is not publishing CAN feedback. "
                        "Keep the arm at natural-down zero; press 'e' only while supporting it.",
                        flush=True,
                    )
                    if getattr(self, "_hand", None) is None:
                        return
                else:
                    raise RuntimeError(
                        f"startup zero check failed: invalid {self._arm_side}-arm feedback"
                    )
            else:
                error_deg = np.degrees(error)
                tolerance = max(
                    0.1, float(self._cfg.robot.full_arm_startup_zero_tolerance_deg)
                )
                print(
                    f"[teleop ZERO] {self._arm_side} arm URDF J1..J7 error(deg)="
                    f"{np.array2string(error_deg, precision=2)}; limit={tolerance:.1f}deg.",
                    flush=True,
                )
                if np.max(np.abs(error_deg)) > tolerance:
                    if not sys.stdin.isatty():
                        raise RuntimeError(
                            "startup full-arm zero alignment needs an interactive terminal "
                            "for safety confirmation"
                        )
                    self._startup_alignment_required = True
                    print(
                        f"[等待操作] {self._arm_side} 机械臂当前处于失能非零位。"
                        "请人工托稳机械臂并清空周围夹点；完成 PICO 标定后按一次 e。"
                        "程序将先检查 PICO 姿态，再从当前实测位置限速避碰回零并确认失能，"
                        "随后复检 PICO 姿态并进入遥操作；不需要按 Enter。",
                        flush=True,
                    )
                else:
                    self._startup_alignment_required = False
                    print(
                        f"[teleop ZERO] {self._arm_side.upper()} FULL-ARM NATURAL-DOWN ZERO "
                        "VERIFIED; motors remain DISABLED.",
                        flush=True,
                    )
            if not getattr(self, "_arm_with_hand", False) or getattr(self, "_hand", None) is None:
                return

        if self._hand is None:
            return
        hand_ok = False
        hand_errors: dict[str, np.ndarray] = {}
        deadline = time.monotonic() + max(
            0.5, float(self._cfg.hand.startup_open_verify_timeout_s)
        )
        while time.monotonic() < deadline:
            hand_ok, hand_errors = self._hand.open_pose_status()
            if hand_errors:
                break
            time.sleep(0.02)
        if not hand_errors:
            raise RuntimeError(
                "startup zero check failed: fresh LinkerHand joint feedback is unavailable"
            )

        linked_uncalibrated: list[str] = []
        for side, error in hand_errors.items():
            command_target = (
                self._cfg.hand.left_open if side == "left" else self._cfg.hand.right_open
            )
            feedback_target = (
                self._cfg.hand.left_open_feedback
                if side == "left"
                else self._cfg.hand.right_open_feedback
            )
            target = np.asarray(
                feedback_target if len(feedback_target) == len(error) else command_target,
                dtype=np.float64,
            )
            current = target + error
            calibrated = (
                not getattr(self, "_arm_with_hand", False)
                or self._hand.open_feedback_calibrated(side)
            )
            if calibrated:
                print(
                    f"[teleop ZERO] {side} hand current={np.rint(current).astype(int).tolist()} "
                    f"target(open-feedback-zero)={np.rint(target).astype(int).tolist()} "
                    f"max_error={float(np.max(np.abs(error))):.1f}/255.",
                    flush=True,
                )
            else:
                linked_uncalibrated.append(side)
                print(
                    f"[teleop ZERO] {side} hand current={np.rint(current).astype(int).tolist()} "
                    "target(open-feedback-zero)=UNCONFIGURED; read-only measurement.",
                    flush=True,
                )

        if getattr(self, "_arm_with_hand", False):
            if linked_uncalibrated:
                measured = {
                    side: np.rint(
                        self._hand.open_feedback_target(side) + hand_errors[side]
                    ).astype(int).tolist()
                    for side in linked_uncalibrated
                }
                raise RuntimeError(
                    "linked startup refused before motion: independent natural-open "
                    f"feedback zero is missing for {linked_uncalibrated}; measured={measured}. "
                    "Record repeated supported natural-open feedback and configure the "
                    "matching *_open_feedback value; command endpoints are not feedback "
                    "calibration. No hand alignment, arm enable, or position command was sent."
                )

        arm_ok = True
        arm_current: np.ndarray | None = None
        arm_error = np.zeros(3, dtype=np.float64)
        if self._wrist_imu_side and self._driver is not None:
            arm_current = self._driver.read_joints()
            if arm_current is None:
                raise RuntimeError(
                    f"startup zero check failed: invalid {self._wrist_imu_side}-arm feedback"
                )
            indices = np.asarray(self._cfg.robot.right_wrist_joint_indices, dtype=int)
            neutral = self._driver.wrist_neutral_positions()
            arm_error = arm_current[indices] - neutral
            tolerance = np.deg2rad(
                max(0.1, float(self._cfg.robot.wrist_return_tolerance_deg))
            )
            arm_ok = bool(np.max(np.abs(arm_error)) <= tolerance)
            print(
                f"[teleop ZERO] {self._wrist_imu_side} arm J5/J6/J7(deg)="
                f"{np.array2string(np.degrees(arm_current[indices]), precision=2)} "
                "target=[0. 0. 0] "
                f"max_error={float(np.max(np.abs(np.degrees(arm_error)))):.2f}deg.",
                flush=True,
            )

        if hand_ok and arm_ok:
            print(
                "[teleop ZERO] VERIFIED: arm terminal joints and every physical joint "
                "of the active hand match the configured startup pose.",
                flush=True,
            )
            return

        if not sys.stdin.isatty():
            raise RuntimeError(
                "startup zero alignment needs an interactive terminal for safety confirmation"
            )
        scopes = []
        if not arm_ok:
            scopes.append("机械臂 J5/J6/J7")
        if not hand_ok:
            scopes.append("灵巧手全部关节")
        input(
            "[等待操作] 检测到 " + "、".join(scopes)
            + " 未处于初始零位。请托稳机械臂、让手远离夹点，按 Enter 限速对齐；"
              "按 Ctrl-C 取消："
        )

        if not arm_ok:
            if not self._driver.capture_session_start():
                raise RuntimeError("startup wrist zero alignment refused: invalid arm feedback")
            self._motors_enabled = self._driver.enable()
            if not self._motors_enabled:
                raise RuntimeError("startup wrist zero alignment refused: motor enable failed")
            returned = False
            try:
                returned = self._driver.return_wrist_to_neutral()
            finally:
                disabled = self._driver.disable()
                self._motors_enabled = not disabled
            if not returned or not disabled:
                raise RuntimeError(
                    "startup wrist zero alignment was not verified; arm disable was retried"
                )
            # return_wrist_to_neutral() verifies zero while the joints are still
            # enabled.  Once all joints are disabled the supported arm may move
            # under gravity, so post-disable position is diagnostic only.
            post_disable = self._driver.read_joints()
            indices = np.asarray(self._cfg.robot.right_wrist_joint_indices, dtype=int)
            neutral = self._driver.wrist_neutral_positions()
            tolerance = np.deg2rad(
                max(0.1, float(self._cfg.robot.wrist_return_tolerance_deg))
            )
            print(
                f"[teleop ZERO] {self._wrist_imu_side.upper()} WRIST ZERO VERIFIED; "
                "all arm joints are DISABLED.",
                flush=True,
            )
            if post_disable is None:
                print(
                    f"[teleop ZERO] {self._wrist_imu_side.upper()} post-disable "
                    "feedback unavailable; zero was verified before disable.",
                    flush=True,
                )
            else:
                post_error = post_disable[indices] - neutral
                if np.max(np.abs(post_error)) > tolerance:
                    print(
                        f"[teleop ZERO] {self._wrist_imu_side.upper()} post-disable "
                        "gravity drift(deg)="
                        f"{np.array2string(np.degrees(post_error), precision=2)}; "
                        "this does not invalidate the pre-disable zero check.",
                        flush=True,
                    )

        if not hand_ok:
            aligned, final_errors = self._hand.align_and_verify_open_pose()
            if not aligned:
                detail = (
                    self._hand.describe_open_pose_errors(final_errors)
                    or "fresh feedback unavailable"
                )
                raise RuntimeError(f"startup hand open-zero alignment failed: {detail}")
            self._hand_enabled = False
            print(
                "[teleop ZERO] HAND OPEN-ZERO VERIFIED; hand following remains DISABLED.",
                flush=True,
            )

        print(
            "[teleop ZERO] STARTUP POSE VERIFIED. Keyboard control is now available.",
            flush=True,
        )

    def _key_loop(self) -> None:
        if not sys.stdin.isatty():
            # Non-interactive run: arms stay disarmed unless --enable was passed.
            return
        try:
            import os
            import select
            import tty
            import termios
        except ImportError:
            return
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while not self._stop:
                readable, _, _ = select.select([fd], [], [], 0.1)
                if not readable:
                    continue
                raw = os.read(fd, 1)
                if not raw:
                    break
                ch = raw.decode("utf-8", errors="ignore")
                try:
                    self._handle_key(ch.lower())
                except Exception as exc:  # noqa: BLE001
                    print(
                        f"[teleop] key {ch!r} failed: {type(exc).__name__}: {exc}",
                        flush=True,
                    )
        except Exception as exc:  # noqa: BLE001
            print(f"[teleop] keyboard listener stopped: {exc}", flush=True)
        finally:
            try:
                termios.tcsetattr(fd, termios.TCSADRAIN, old)
            except Exception as exc:  # noqa: BLE001
                print(f"[teleop] failed to restore terminal settings: {exc}", flush=True)

    def _handle_key(self, ch: str) -> None:
        probe = getattr(self, "_arm_probe", None)
        if probe is not None and ch in ("e", "s", "d", "x", "q", "r", "z"):
            probe.submit(dict(event="operator_key", key=ch))
        if getattr(self, "_arm_only", False):
            if getattr(self, "_startup_power_cycle_required", False):
                if ch == "q":
                    self._exit_for_startup_power_cycle("operator pressed 'q'")
                    return
                if ch in ("e", "s", "z"):
                    print(
                        f"[teleop] {ch.upper()} REFUSED: {self._arm_side} controller "
                        "requires a power-cycle after incomplete startup feedback. No "
                        "enable, return, or position command was sent; press q to E-stop "
                        "and exit.",
                        flush=True,
                    )
                    return
            worker = getattr(self, "_return_thread", None)
            arming_worker = getattr(self, "_arming_thread", None)
            arming_active = arming_worker is not None and arming_worker.is_alive()
            returning = ((worker is not None and worker.is_alive())
                         or arming_active
                         or getattr(self, "_startup_alignment", False))
            if returning:
                if arming_active and ch in ("s", "q", "x", "d"):
                    cancel = getattr(self, "_arming_cancel", None)
                    if cancel is not None:
                        cancel.set()
                    if ch == "q":
                        self._quit_after_arming_cancel = True
                if ch == "s":
                    self._quit_after_arming_cancel = False
                    self._quit_after_return = False
                    self._driver.cancel_return()
                    print("[teleop] Return cancelled; following remains locked. Use d/x if needed.", flush=True)
                    return
                if ch == "q":
                    self._quit_after_return = True
                    if getattr(self, "_arm_with_hand", False):
                        self._open_hand_after_return = True
                    print(
                        "[teleop] Safe return or arming cancellation is still in progress; "
                        "q keeps the pending exit active. Use s to cancel, d to disable, "
                        "or x for E-stop.",
                        flush=True,
                    )
                    return
                if ch not in ("x", "d"):
                    print("[teleop] Return in progress; s cancels, x emergency-stops, d disables.", flush=True)
                    return
            fault_latched = self._full_arm_fault_latched()
            if ch == "s" and getattr(self, "_motors_enabled", False) and fault_latched:
                print(
                    "[teleop] STOP ALREADY ACTIVE: following is locked by the arm fault. "
                    "Press q to request checked recovery return and exit, or z to return "
                    "without exiting. Physical contact, torque/controller faults, stale "
                    "feedback, or unsafe geometry will still refuse motion; d/x remain available.",
                    flush=True,
                )
                return
            if ch in ("s", "q", "z"):
                if ch == "q" and not self._motors_enabled and not fault_latched:
                    if getattr(self, "_arm_with_hand", False):
                        self._return_linked_hand_to_open()
                    self._stop = True
                    return
                recover = ch == "z" or (ch == "q" and fault_latched)
                if ch == "q" and fault_latched:
                    print(
                        "[teleop] EXIT REQUEST: fault is latched; starting the same checked "
                        "recovery return used by z. Exit occurs only after zero and seven-joint "
                        "disable are both verified.",
                        flush=True,
                    )
                self._start_full_arm_return(
                    f"operator pressed '{ch}'",
                    recover=recover,
                    quit_after=ch == "q",
                    open_hand_after=(
                        getattr(self, "_arm_with_hand", False)
                        and ch in ("s", "q")
                    ),
                )
                return
            if ch == "h" and not getattr(self, "_arm_with_hand", False):
                print("[teleop] Hand feedback is read-only in arm-only mode.", flush=True)
                return
        if ch == "r":
            if not getattr(self, "_arm_only", False):
                print("[teleop] PICO RECALIBRATION REFUSED: only available in single-arm PICO modes.", flush=True)
                return
            if (self._motors_enabled or self._arm_armed or self._hand_enabled
                    or getattr(self, "_manual_intervention_required", False)
                    or getattr(self._driver, "safety_fault_reason", None)):
                print("[teleop] PICO RECALIBRATION REFUSED: motors/following or safety lock active; no hardware command sent.", flush=True)
                return
            self._body.restart_arm_reference()
            self._ik.reset_j3_observability_baseline(self._arm_side)
            print(
                "[PICO 标定] 已清除旧 PICO 参考。请放松手臂、允许自然微屈，保持在捕捉范围内；"
                "新数据到达后开始准备倒计时。手套标定保持不变，完成后仍需按 e。",
                flush=True,
            )
        elif ch == "e":
            if (getattr(self, "_arm_only", False)
                    and not getattr(self, "_arm_with_hand", False)
                    and not getattr(self, "_wrist_imu_side", None)):
                self._start_pending_startup_arming()
                return
            if getattr(self, "_arm_with_hand", False):
                side = self._arm_side
                if self._body.hand_joints() is None:
                    print(
                        f"[teleop] LINKED ENABLE REFUSED: fresh {side} SenseGlove "
                        "finger targets are unavailable.",
                        flush=True,
                    )
                    self._arm_armed = False
                    self._hand_enabled = False
                    return
                if self._hand is None or not self._hand.feedback_ready():
                    print(
                        f"[teleop] LINKED ENABLE REFUSED: fresh {side}-hand feedback "
                        "is unavailable.",
                        flush=True,
                    )
                    self._arm_armed = False
                    self._hand_enabled = False
                    return
                self._arm_armed = self._arm_robot()
                if not self._arm_armed:
                    self._hand_enabled = False
                    print(
                        "[teleop] LINKED ENABLE CANCELLED: arm checks failed; hand following remains disabled.",
                        flush=True,
                    )
                else:
                    self._hand_enabled = self._hand.set_enabled(True)
                    if not self._hand_enabled:
                        self._stop_full_arm_for_fault(
                            f"{side}-hand feedback disappeared during linked enable")
                        print("[teleop] LINKED ENABLE CANCELLED: hand feedback disappeared; "
                              "following locked. Clear the fault before z recovery.", flush=True)
                    else:
                        print(
                            f"[teleop] {side.upper()} ARM + {side.upper()} HAND "
                            "LINKED FOLLOWING ENABLED.",
                            flush=True,
                        )
            else:
                self._arm_armed = self._arm_robot()

        elif ch == "s":
            self._arm_armed = False
            if self._wrist_imu_side and self._driver is not None:
                self._return_wrist_to_zero_and_disable("operator pressed 's'")
            else:
                print("[teleop] ARM FOLLOWING STOPPED - controller holding position.", flush=True)
        elif ch == "d":
            self._arm_armed = False
            self._manual_intervention_required = False
            if getattr(self, "_arm_with_hand", False) and self._hand is not None:
                self._hand.set_enabled(False)
                self._hand_enabled = False
            if self._driver is None or not self._motors_enabled:
                print("[teleop] OPERATOR DISABLE: arm is already disabled.", flush=True)
            else:
                disabled = bool(self._driver.disable())
                self._motors_enabled = not disabled
                print(
                    f"[teleop] OPERATOR DISABLE: "
                    f"{'DISABLED' if disabled else 'DISABLE NOT CONFIRMED'}.",
                    flush=True,
                )
        elif ch == "x":
            self._arm_armed = False
            self._manual_intervention_required = True
            if getattr(self, "_arm_with_hand", False) and self._hand is not None:
                self._hand.set_enabled(False)
                self._hand_enabled = False
            if self._driver is not None:
                self._driver.emergency_stop()
                self._motors_enabled = False
            scope = (
                f"{self._wrist_imu_side} arm"
                if self._wrist_imu_side
                else f"{self._arm_side} arm"
                if getattr(self, "_arm_only", False)
                else "both arms"
            )
            print(f"[teleop] ELECTRONIC E-STOP sent to {scope}.", flush=True)

        elif ch == "h":
            if self._hand is None:
                print("[teleop] hand control is unavailable in arm-only mode.", flush=True)
                return
            requested = not self._hand_enabled
            self._hand_enabled = self._hand.set_enabled(requested)
            if requested and not self._hand_enabled:
                print("[teleop] HAND ENABLE REFUSED: waiting for fresh state feedback.", flush=True)
            else:
                print(f"[teleop] HANDS {'ENABLED' if self._hand_enabled else 'DISABLED'}.", flush=True)
        elif ch == "i":
            if not self._wrist_imu_side:
                print("[teleop] IMU diagnostics unavailable outside wrist-IMU mode.", flush=True)
                return
            self._wrist_diagnostics_enabled = not self._wrist_diagnostics_enabled
            state = "ENABLED" if self._wrist_diagnostics_enabled else "DISABLED"
            print(f"[teleop] IMU diagnostics {state}.", flush=True)
        elif ch == "q":
            self._arm_armed = False
            self._return_hand_open_on_close = self._hand is not None
            self._stop = True
            print("[teleop] quitting: returning wrist J5/J6/J7 to zero, disabling arm, "
                  "then returning hand to open pose.", flush=True)

    def _enabled_fault_requires_explicit_disable(self) -> bool:
        return bool(
            getattr(self, "_arm_only", False)
            and getattr(self, "_driver", None) is not None
            and getattr(self, "_motors_enabled", False)
            and (
                getattr(self, "_manual_intervention_required", False)
                or getattr(self._driver, "_return_inhibited", False)
            )
        )

    def _full_arm_fault_latched(self) -> bool:
        driver = getattr(self, "_driver", None)
        reason = None if driver is None else getattr(driver, "safety_fault_reason", None)
        return bool(
            getattr(self, "_manual_intervention_required", False)
            or (isinstance(reason, str) and bool(reason))
            or (
                driver is not None
                and getattr(driver, "_return_inhibited", False) is True
            )
        )

    def _exit_for_startup_power_cycle(self, source: str) -> bool:
        """Stop an unobservable startup without attempting a position return."""
        if not getattr(self, "_startup_power_cycle_required", False):
            return False
        self._arm_armed = False
        self._manual_intervention_required = True
        if getattr(self, "_arm_with_hand", False) and self._hand is not None:
            self._hand.set_enabled(False)
            self._hand_enabled = False
        try:
            self._driver.emergency_stop()
        except Exception as exc:  # noqa: BLE001
            if getattr(self, "_startup_transport_unavailable", False):
                self._stop = True
                print(
                    f"[teleop] POWER-CYCLE EXIT: electronic E-stop could not be delivered "
                    f"because CAN is unavailable ({type(exc).__name__}: {exc}). No return "
                    "position was commanded; motor state remains unverified. Use the "
                    "physical emergency stop and power-cycle the arm controller before "
                    "reconnecting.",
                    flush=True,
                )
                return True
            print(
                f"[teleop] POWER-CYCLE EXIT REFUSED: electronic E-stop send failed "
                f"({type(exc).__name__}: {exc}). Keep supporting the arm, use the "
                "physical emergency stop, then press Ctrl-C again.",
                flush=True,
            )
            return False
        self._motors_enabled = False
        self._stop = True
        print(
            f"[teleop] POWER-CYCLE EXIT: {source}; electronic E-stop sent, no return "
            "position was commanded. Seven-joint disable is unverified because feedback "
            f"is absent. Keep supporting the arm, power-cycle the {self._arm_side} "
            "controller, and "
            "verify feedback before the next enable.",
            flush=True,
        )
        return True

    @staticmethod
    def _report_enabled_fault_exit_refusal(source: str) -> None:
        print(
            f"[teleop] {source} REFUSED: the arm fault is latched and seven-joint "
            "disable is not confirmed. Keep supporting the arm. Press 'q' to request "
            "a checked recovery return and exit; if return remains unsafe or hardware "
            "faults remain active, press 'd' once and wait for "
            "'OPERATOR DISABLE: DISABLED' before exiting.",
            flush=True,
        )

    def _start_pending_startup_arming(self) -> None:
        """Run single-arm alignment/arming without racing the control-loop IK state."""
        worker = getattr(self, "_arming_thread", None)
        if worker is not None and worker.is_alive():
            print("[teleop] Startup alignment/arming is already in progress.", flush=True)
            return
        cancel = getattr(self, "_arming_cancel", None)
        if cancel is None:
            cancel = threading.Event()
            self._arming_cancel = cancel
        cancel.clear()

        def run():
            try:
                lock = getattr(self, "_arm_control_lock", None)
                if lock is None:  # Compatibility for small isolated test doubles.
                    armed = self._arm_robot()
                else:
                    with lock:
                        armed = self._arm_robot()
                if armed and cancel.is_set():
                    disabled = bool(self._driver.disable())
                    self._motors_enabled = not disabled
                    print(
                        f"[teleop] Startup arming was cancelled after enable; "
                        f"{self._arm_side} arm "
                        f"{'DISABLED' if disabled else 'DISABLE NOT CONFIRMED'}.",
                        flush=True,
                    )
                    armed = False
                self._arm_armed = bool(armed and not cancel.is_set())
            except Exception as exc:  # noqa: BLE001
                self._arm_armed = False
                self._manual_intervention_required = True
                print(
                    f"[teleop] STARTUP ARMING FAILED: {type(exc).__name__}: {exc}; "
                    "following remains locked.",
                    flush=True,
                )
            finally:
                if getattr(self, "_quit_after_arming_cancel", False):
                    self._quit_after_arming_cancel = False
                    if getattr(self, "_startup_power_cycle_required", False):
                        self._exit_for_startup_power_cycle(
                            "operator pressed 'q' during startup arming"
                        )
                    else:
                        fault_latched = self._full_arm_fault_latched()
                        if not getattr(self, "_motors_enabled", False) and not fault_latched:
                            self._stop = True
                        else:
                            self._start_full_arm_return(
                                "operator pressed 'q' during startup arming",
                                recover=fault_latched,
                                quit_after=True,
                            )

        self._arming_thread = threading.Thread(
            target=run, name="startup-arm-enable", daemon=True
        )
        self._arming_thread.start()

    def _arm_startup_active(self) -> bool:
        worker = getattr(self, "_arming_thread", None)
        return bool(worker is not None and worker.is_alive())

    def _align_pending_startup_zero(self) -> bool:
        if not getattr(self, "_startup_alignment_required", False):
            return True
        error = self._driver.startup_zero_error()
        tolerance = np.deg2rad(max(
            0.1, float(self._cfg.robot.full_arm_startup_zero_tolerance_deg)
        ))
        if error is not None and np.max(np.abs(error)) <= tolerance:
            self._startup_alignment_required = False
            return True
        print(
            f"[teleop ZERO] {self._arm_side.upper()} startup alignment: holding fresh "
            "measured feedback as the first target, then slowly returning along a "
            "collision-checked path to natural-down zero.",
            flush=True,
        )
        self._startup_alignment = True
        try:
            returned, disabled = self._driver.align_startup_zero_and_disable()
        finally:
            self._startup_alignment = False
        self._motors_enabled = not disabled
        if not returned or not disabled:
            self._manual_intervention_required = True
            print(
                f"[teleop] ARMING REFUSED: {self._arm_side} startup natural-down "
                f"alignment failed: zero_verified={returned}, "
                f"disable_verified={disabled}.",
                flush=True,
            )
            return False
        self._startup_alignment_required = False
        print(
            f"[teleop ZERO] {self._arm_side.upper()} FULL-ARM NATURAL-DOWN ZERO "
            "VERIFIED; all joints are DISABLED. Rechecking PICO reference before enable.",
            flush=True,
        )
        return True

    def _return_linked_hand_to_open(self) -> bool:
        """Open the linked hand only after arm disable has been verified."""
        if not getattr(self, "_arm_with_hand", False) or self._hand is None:
            return True
        self._hand.set_enabled(False)
        self._hand_enabled = False
        try:
            opened = bool(self._hand.return_to_open())
        except Exception as exc:  # noqa: BLE001
            print(
                f"[teleop] LINKED HAND OPEN FAILED: {type(exc).__name__}: {exc}; "
                "hand following remains disabled.",
                flush=True,
            )
            return False
        print(
            "[teleop] LINKED HAND RETURNED TO NATURAL OPEN POSE."
            if opened
            else "[teleop] LINKED HAND OPEN SKIPPED: fresh hand feedback unavailable; "
                 "hand following remains disabled.",
            flush=True,
        )
        return opened

    def _start_full_arm_return(
        self,
        reason,
        *,
        recover=False,
        quit_after=False,
        open_hand_after=False,
    ):
        """Keep the keyboard responsive while planning/executing a return."""
        if self._driver is None:
            return
        if getattr(self, "_arm_with_hand", False) and self._hand is not None:
            # Freeze the fingers before the arm starts planning. Opening is
            # deferred until arm zero and seven-joint disable are both verified.
            self._hand.set_enabled(False)
            self._hand_enabled = False
        if not recover and getattr(self, "_manual_intervention_required", False):
            print("[teleop] Return inhibited by fault; clear the fault before explicit z recovery.", flush=True)
            return
        if not recover and not getattr(self, "_motors_enabled", True):
            print("[teleop] Arm already disabled; no return motion requested.", flush=True)
            if open_hand_after:
                self._return_linked_hand_to_open()
            if quit_after:
                self._stop = True
            return
        worker = getattr(self, "_return_thread", None)
        if worker is not None and worker.is_alive():
            if quit_after:
                self._quit_after_return = True
            if open_hand_after:
                self._open_hand_after_return = True
            return
        if quit_after:
            self._quit_after_return = True
        if open_hand_after:
            self._open_hand_after_return = True
        self._arm_armed = False
        self._manual_intervention_required = True
        self._hand_enabled = False
        self._return_hand_open_on_close = False
        if self._hand is not None:
            self._hand.set_enabled(False)
        request_id = self._driver.prepare_return()
        if request_id is None:
            return
        def run():
            try:
                result = self._driver.safe_return(recover=recover, request_id=request_id)
                self._motors_enabled = not result.disabled
                self._manual_intervention_required = not (result.returned and result.disabled)
                pending_exit = bool(
                    quit_after or getattr(self, "_quit_after_return", False)
                )
                completed = bool(result.returned and result.disabled)
                if completed and getattr(self, "_open_hand_after_return", False):
                    self._return_linked_hand_to_open()
                if pending_exit and completed:
                    self._stop = True
                print(f"[teleop] {reason}: {result.stage}: {result.reason or 'zero verified'}", flush=True)
            except Exception as exc:
                self._manual_intervention_required = True
                print(f"[teleop] Return failed: {exc}; following locked.", flush=True)
            finally:
                self._quit_after_return = False
                self._open_hand_after_return = False
        self._return_thread = threading.Thread(target=run, name="safe-arm-return", daemon=True)
        self._return_thread.start()

    def _return_full_arm_to_zero_and_disable(self, reason: str) -> tuple[bool, bool]:
        self._arm_armed = False
        if self._driver is None or not self._motors_enabled:
            print(
                f"[teleop] {self._arm_side.upper()} ARM STOPPED: {reason}; arm is already disabled.",
                flush=True,
            )
            return False, True
        print(
            f"[teleop] {self._arm_side.upper()} ARM STOPPED: {reason}; slowly returning "
            "along a collision-checked path to natural-down zero before disable.",
            flush=True,
        )
        returned, disabled = self._driver.return_to_zero_and_disable()
        self._motors_enabled = not disabled
        self._manual_intervention_required = not returned and not disabled
        if returned:
            status = "ZERO VERIFIED; DISABLED" if disabled else "ZERO VERIFIED; DISABLE NOT CONFIRMED"
        elif disabled:
            status = "ZERO RETURN NOT CONFIRMED; ARM DISABLED"
        else:
            status = (
                "ZERO RETURN NOT CONFIRMED; FOLLOWING LOCKED, ARM REMAINS ENABLED. "
                "After clearing the fault, press 'z' for recovery; 'd' disables, 'x' E-stops"
            )
        print(f"[teleop] {self._arm_side.upper()} ARM {status}.", flush=True)
        return returned, disabled

    def _trajectory_active(self):
        return (getattr(self, "_arm_only", False)
                and bool(getattr(self._cfg.robot, f"{self._arm_side}_command_trajectory_enabled", False)))

    def _stop_full_arm_for_fault(self, reason):
        if (getattr(self._driver, "_return_active", False)
                or getattr(self._driver, "_return_pending", False)):
            return False, False
        if not self._trajectory_active():
            return self._return_full_arm_to_zero_and_disable(reason)
        self._arm_armed = False
        self._manual_intervention_required = True
        if self._driver is not None:
            self._driver.latch_return_fault(reason)
        if getattr(self, "_arm_with_hand", False) and self._hand is not None:
            self._hand.set_enabled(False)
            self._hand_enabled = False
        print(f"[teleop] {self._arm_side.upper()} ARM SAFETY STOP: {reason}; following locked, "
              "last controller target retained; no zero/disable command sent. Clear the fault before z; d/x remain available.", flush=True)
        return False, False

    def _report_silent_startup_failure(self):
        failure = getattr(self._driver, "last_startup_failure", None)
        self._manual_intervention_required = True
        if not isinstance(failure, dict):
            print("[teleop] ARMING REFUSED: startup failed without stage diagnostics; "
                  "verify controller state and inspect logs before retrying.", flush=True)
            return
        print(f"[teleop] ARMING REFUSED: {self._arm_side} startup stage="
              f"{failure['stage']}: {failure['reason']}.", flush=True)
        if failure.get("wake_attempted"):
            # A failed wake does not establish that the arm is disabled.
            self._motors_enabled = True
            print("[teleop] A controller wake was attempted; current motor state requires verification.", flush=True)
        else:
            print("[teleop] Rejected before controller wake; this attempt issued no wake, "
                  "enable, position or disable commands.", flush=True)
        if failure.get("disable_attempted"):
            print("[teleop] Disable commands were attempted; this alone does not confirm all motors disabled.", flush=True)
        if failure.get("power_cycle_required"):
            self._startup_power_cycle_required = True
            self._startup_transport_unavailable = bool(
                failure.get("transport_unavailable")
            )
            if self._startup_transport_unavailable:
                print(
                    f"[teleop] CAN TRANSPORT UNAVAILABLE: {self._arm_side} controller "
                    "did not acknowledge frames and motor state cannot be verified. "
                    "Do not retry e/z. Use the physical emergency stop, check controller "
                    "power and the arm CAN cable, then exit and power-cycle before retrying.",
                    flush=True,
                )
            print(
                f"[teleop] POWER-CYCLE REQUIRED: {self._arm_side} controller returned no "
                "complete seven-joint state after its bounded wake. Further e/z/return "
                "attempts are locked. Keep supporting the arm; q sends electronic E-stop "
                "and exits without a position command, then power-cycle the controller "
                "and verify feedback before retrying.",
                flush=True,
            )
        geometry = failure.get("geometry") or {}
        if geometry:
            print(f"[teleop] Startup geometry diagnostics: {geometry}", flush=True)
        if failure['stage'] == "geometry":
            if (getattr(self, "_arm_with_hand", False)
                    and geometry.get("calibrated_hand_joints") == 0):
                print(
                    "[teleop] LINKED ARMING BLOCKED: no verified hand-feedback-to-URDF "
                    "calibration is configured, so collision checking used the complete "
                    "finger motion envelope. Collect and verify the active hand calibration; "
                    "do not substitute command endpoints or bypass the collision guard. "
                    "Power-cycling does not resolve this preflight rejection.",
                    flush=True,
                )
            else:
                print("[teleop] Check fresh hand-state feedback and verified feedback-to-URDF calibration; "
                      "do not bypass the collision guard. Power-cycling does not resolve this preflight rejection.", flush=True)

    def _arm_robot(self) -> bool:
        if getattr(self, "_startup_power_cycle_required", False):
            print(
                f"[teleop] ARMING REFUSED: {self._arm_side} controller still requires "
                "a power-cycle after incomplete startup feedback. No enable or position "
                "command was sent; press q to E-stop and exit first.",
                flush=True,
            )
            return False
        if getattr(self, "_manual_intervention_required", False):
            print(
                "[teleop] ARMING REFUSED: manual intervention is required after an "
                "unconfirmed zero return. Press 's' to retry, 'd' to disable, or "
                "'x' for E-stop.",
                flush=True,
            )
            return False
        if (
            getattr(self, "_arm_only", False)
            and self._cfg.retarget.require_hand_imu_for_active_arm
            and self._body.hand_orientation_delta(self._arm_side) is None
        ):
            print(
                f"[teleop] ARMING REFUSED: {self._arm_side} SenseGlove IMU is "
                "missing/stale; PICO wrist orientation will not be used as fallback.",
                flush=True,
            )
            return False
        if self._driver is None:
            if self._wrist_imu_side:
                imu_delta = self._body.hand_orientation_delta(self._wrist_imu_side)
                if imu_delta is None:
                    print(
                        f"[teleop] DRY-RUN ARMING REFUSED: "
                        f"{self._wrist_imu_side} glove IMU is missing/stale.",
                        flush=True,
                    )
                    return False
            print("[teleop] dry-run armed (IMU diagnostics only; no robot commands).", flush=True)
            return True
        if self._wrist_imu_side:
            side = self._wrist_imu_side
            print(
                f"[teleop] Arming {side} terminal orientation: checking fresh IMU "
                "and arm feedback...",
                flush=True,
            )
            imu_delta = self._body.hand_orientation_delta(side)
            if imu_delta is None:
                print(f"[teleop] ARMING REFUSED: {side} glove IMU is missing/stale.", flush=True)
                return False
            if not self._driver.capture_session_start():
                print(f"[teleop] ARMING REFUSED: invalid {side}-arm joint feedback.", flush=True)
                return False
            self._current_arm_joints = self._driver.read_joints()
            if self._current_arm_joints is None:
                print(f"[teleop] ARMING REFUSED: {side}-arm feedback disappeared.", flush=True)
                return False
            try:
                self._configure_wrist_mapping(self._current_arm_joints)
            except (TypeError, ValueError, np.linalg.LinAlgError) as exc:
                self._wrist_local_axes = None
                print(f"[teleop] ARMING REFUSED: wrist coordinate mapping failed: {exc}", flush=True)
                return False
            if not self._motors_enabled:
                print(
                    f"[teleop] Fixed zero mapping ready; enabling {side} arm at the "
                    "measured pose before rate-limited convergence...",
                    flush=True,
                )
                self._motors_enabled = self._driver.enable()
                if not self._motors_enabled:
                    print(f"[teleop] ARMING REFUSED: {side}-arm motor enable failed.", flush=True)
                    return False
            print(
                f"[teleop] Aligning {side} J5/J6/J7 to physical zero with rate and "
                "acceleration limits before applying the current glove angle...",
                flush=True,
            )
            if not self._driver.return_wrist_to_neutral():
                self._return_wrist_to_zero_and_disable(
                    "arming zero alignment failed", allow_return=False
                )
                print(
                    f"[teleop] ARMING REFUSED: {side} wrist zero was not verified.",
                    flush=True,
                )
                return False
            # The glove may be anywhere within the configured IMU range. Its
            # calibrated delta remains the first tracking target, but tracking
            # is not armed until the physical wrist has first reached zero.
            if self._body.hand_orientation_delta(side) is None:
                self._return_wrist_to_zero_and_disable(
                    "glove IMU became stale during arming"
                )
                print(
                    f"[teleop] ARMING REFUSED: {side} glove IMU became stale "
                    "during wrist zero alignment.",
                    flush=True,
                )
                return False
            self._current_arm_joints = self._driver.read_joints()
            if self._current_arm_joints is None:
                self._return_wrist_to_zero_and_disable(
                    "arm feedback disappeared after zero alignment", allow_return=False
                )
                print(
                    f"[teleop] ARMING REFUSED: {side}-arm zero feedback disappeared.",
                    flush=True,
                )
                return False
            print(
                f"[teleop] {side.upper()} WRIST ZERO VERIFIED AND ARMED: terminal "
                "joints started at [0, 0, 0]; applying the current calibrated IMU delta; "
                "only terminal joints 5, 6 and 7 may move. "
                "Press 's' to stop or 'x' for E-stop.",
                flush=True,
            )
            return True

        if getattr(self, "_arm_only", False):
            if getattr(self, "_arm_armed", False):
                return True
            if not self._wait_for_arm_prepare():
                return False
            cancel = getattr(self, "_arming_cancel", None)
            if cancel is not None and cancel.is_set():
                print("[teleop] Startup arming cancelled before hardware checks.", flush=True)
                return False
        if not self._body.is_ready():
            print("[teleop] ARMING REFUSED: body tracking/reference is not ready.", flush=True)
            return False
        if getattr(self, "_arm_only", False):
            snapshot = self._body.refresh_arm_reference_for_enable(self._arm_side)
            if snapshot is None:
                print(
                    "[teleop] ARMING REFUSED: recent PICO arm data is unavailable or "
                    "unstable; no motor-enable or position command was sent.",
                    flush=True,
                )
                return False
            target_ready, upper_delta, forearm_delta, max_std, sample_count = snapshot
            if not target_ready:
                print(
                    "[teleop] ARMING REFUSED: the recent human-arm target is invalid; "
                    "no motor-enable or position command was sent.",
                    flush=True,
                )
                return False
            print(
                f"[teleop] PICO latest target accepted from {sample_count} stable frames "
                f"(position_std={max_std:.3f}m, offset_from_calibration="
                f"{upper_delta:.1f}/{forearm_delta:.1f}deg); robot has not moved.",
                flush=True,
            )
            if getattr(self, "_startup_alignment_required", False):
                if not self._align_pending_startup_zero():
                    return False
                cancel = getattr(self, "_arming_cancel", None)
                if cancel is not None and cancel.is_set():
                    print("[teleop] Startup arming cancelled after zero alignment.", flush=True)
                    return False
                snapshot = self._body.refresh_arm_reference_for_enable(self._arm_side)
                if snapshot is None or not snapshot[0]:
                    detail = "unstable/unavailable" if snapshot is None else (
                        f"upper={snapshot[1]:.1f}deg, forearm={snapshot[2]:.1f}deg"
                    )
                    print(
                        f"[teleop] ARMING REFUSED: the latest PICO target became unavailable "
                        f"or unstable during startup alignment ({detail}); the arm is verified "
                        "at zero and disabled. Hold the desired pose steadily and press e again.",
                        flush=True,
                    )
                    return False
                print(
                    f"[teleop] PICO latest target reverified after startup alignment "
                    f"(position_std={snapshot[3]:.3f}m, offset_from_calibration="
                    f"{snapshot[1]:.1f}/{snapshot[2]:.1f}deg).",
                    flush=True,
                )
        if not self._driver.capture_session_start():
            if not (
                getattr(self, "_arm_only", False)
                and self._driver.silent_disabled_startup
            ):
                print("[teleop] ARMING REFUSED: missing/invalid joint feedback.", flush=True)
                return False
            print(
                f"[teleop] {self._arm_side.upper()} ARM: starting one bounded feedback wake "
                "at the configured natural-down zero. Keep supporting the arm.",
                flush=True,
            )
            if not self._driver.enable_from_natural_down():
                self._report_silent_startup_failure()
                return False
            self._motors_enabled = True
            if not self._driver.capture_session_start():
                self._driver.disable()
                self._motors_enabled = False
                print(
                    "[teleop] ARMING REFUSED: feedback disappeared immediately after wake-up.",
                    flush=True,
                )
                return False
        self._current_arm_joints = self._driver.read_full_urdf_joints()
        if self._current_arm_joints is None:
            if getattr(self, "_arm_only", False) and self._motors_enabled:
                disabled = bool(self._driver.disable())
                self._motors_enabled = not disabled
                print(
                    f"[teleop] {self._arm_side.upper()} ARM SAFETY STOP: feedback "
                    f"disappeared after enable; "
                    f"{'DISABLED' if disabled else 'DISABLE NOT CONFIRMED'}.",
                    flush=True,
                )
            print("[teleop] ARMING REFUSED: joint feedback disappeared.", flush=True)
            return False
        if self._arm_only and self._cfg.ik.partition_terminal_wrist_ik:
            try:
                self._configure_full_arm_wrist_mapping(self._current_arm_joints)
            except (TypeError, ValueError, np.linalg.LinAlgError) as exc:
                print(
                    f"[teleop] ARMING REFUSED: full-arm wrist mapping failed: {exc}",
                    flush=True,
                )
                return False
        self._body.rebase_robot_reference(
            self._ik.current_task_frame_poses(self._current_arm_joints)
        )
        if not self._motors_enabled:
            cancel = getattr(self, "_arming_cancel", None)
            if cancel is not None and cancel.is_set():
                print("[teleop] Startup arming cancelled before motor enable.", flush=True)
                return False
            self._motors_enabled = self._driver.enable()
            if not self._motors_enabled:
                if isinstance(getattr(self._driver, "last_startup_failure", None), dict):
                    self._report_silent_startup_failure()
                else:
                    reason = getattr(self._driver, "safety_fault_reason", None)
                    print(f"[teleop] ARMING REFUSED: {reason or 'motor enable failed'}.", flush=True)
                return False
            self._driver.set_speed(self._cfg.robot.speed_percent)
        if self._arm_only and not self._body.is_ready():
            disabled = bool(self._driver.disable())
            self._motors_enabled = not disabled
            print(
                f"[teleop] ARMING REFUSED: PICO data became stale during enable; "
                f"{self._arm_side} arm "
                f"{'DISABLED' if disabled else 'DISABLE NOT CONFIRMED'}.",
                flush=True,
            )
            return False
        if getattr(self, "_arm_only", False):
            snapshot = self._body.refresh_arm_reference_for_enable(self._arm_side)
            if snapshot is None or not snapshot[0]:
                detail = "unstable/unavailable" if snapshot is None else (
                    f"upper={snapshot[1]:.1f}deg, forearm={snapshot[2]:.1f}deg"
                )
                print(
                    f"[teleop] ARMING REFUSED: latest PICO target check failed after "
                    f"enable/torque settling ({detail}). Following remains locked; "
                    "the arm remains enabled holding its verified startup target. Hold the "
                    "desired human-arm pose steadily and press 'e' again.",
                    flush=True,
                )
                return False
        if getattr(self, "_arm_only", False):
            try:
                startup_snapshot = None
                if self._trajectory_active():
                    # One post-enable measurement anchors FK, IK and trajectory.
                    measured = self._driver.read_full_urdf_joints()
                    if measured is None:
                        raise ValueError("post-enable joint feedback unavailable")
                    startup_snapshot = self._driver._arm._runtime_feedback
                    self._current_arm_joints = measured
                    self._body.rebase_robot_reference(self._ik.current_task_frame_poses(measured))
                    if self._cfg.ik.partition_terminal_wrist_ik:
                        self._configure_full_arm_wrist_mapping(measured)
                    self._driver.record_startup_event("reference_checks_passed")
                self._ik.initialize_arm_session(
                    self._arm_side, self._current_arm_joints,
                    self._body.arm_reference_flexion_rad(self._arm_side),
                    relative_elbow_reference=(
                        self._cfg.retarget.arm_vector_position_mode == "segment_direction_relative"
                    ),
                )
                self._body.seed_arm_reference_filters(self._arm_side)
                if self._trajectory_active() and not self._driver.initialize_command_trajectory(startup_snapshot):
                    self._stop_full_arm_for_fault(
                        self._driver.safety_fault_reason or "command trajectory initialization failed")
                    return False
            except (ValueError, TypeError) as exc:
                print(f"[teleop] ARMING REFUSED: invalid calibrated arm reference: {exc}", flush=True)
                return False
        scope = (
            f"{self._arm_side.upper()} ARM"
            if getattr(self, "_arm_only", False)
            else "ARMS"
        )
        print(
            f"[teleop] {scope} ARMED from verified natural-down feedback; "
            "press 's' for controlled zero return or 'x' for E-stop.",
            flush=True,
        )
        return True

    def _wait_for_arm_prepare(self) -> bool:
        seconds = self._cfg.retarget.arm_reference_enable_prepare_s
        deadline = time.monotonic() + seconds
        last = None
        while not getattr(self, "_stop", False):
            # The keyboard thread is busy in this countdown: keep stop keys live.
            if sys.stdin.isatty() and select.select([sys.stdin], [], [], 0)[0]:
                key = os.read(sys.stdin.fileno(), 1).decode("utf-8", errors="ignore").lower()
                if key in ("q", "s", "x"):
                    self._handle_key(key)
                    return False
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return True
            shown = int(np.ceil(remaining))
            if shown != last:
                print(
                    f"[PICO 使能准备] 请放下按键的手，并稳定保持希望机械臂缓慢跟随的当前姿态，"
                    f"{shown} 秒后检查。Ctrl-C 取消。",
                    flush=True,
                )
                last = shown
            time.sleep(min(.1, remaining))
        return False

    def _read_current_joints(self) -> np.ndarray | None:
        if self._driver is None:
            if self._current_arm_joints is None:
                self._current_arm_joints = np.zeros(14, dtype=np.float64)
            return self._current_arm_joints.copy()
        if getattr(self, "_arm_armed", False) and self._trajectory_active():
            joints = self._driver.read_control_cycle_joints()
            if joints is None:
                return None  # Never substitute the last valid pose during following.
            self._current_arm_joints = joints
            return joints.copy()
        joints = (
            self._driver.read_joints()
            if self._wrist_imu_side
            else self._driver.read_full_urdf_joints()
        )
        if joints is not None:
            self._current_arm_joints = joints
        return self._current_arm_joints.copy() if self._current_arm_joints is not None else None

    def run(self) -> None:
        loop_dt = max(self._cfg.ik.dt, 0.005)
        last_hand = time.monotonic()
        hand_interval = 1.0 / max(self._cfg.hand.publish_hz, 1.0)
        if self._hand_only:
            print("[teleop] hand-only mode", flush=True)
        elif getattr(self, "_arm_only", False):
            live_collision = (self._cfg.robot.torso_collision_enabled
                              and self._cfg.robot.teleop_torso_collision_enabled)
            print(
                f"[teleop] Live-following torso geometry checks: {'ON' if live_collision else 'OFF'}; "
                "slow returns retain collision preflight, planning and execution checks.",
                flush=True,
            )
            if getattr(self, "_arm_with_hand", False):
                print(
                    f"[teleop] PICO {self._arm_side} arm + SenseGlove/LinkerHand linked mode; "
                    "the opposite arm and hand are not connected.",
                    flush=True,
                )
            else:
                print(
                    f"[teleop] PICO {self._arm_side}-arm-only mode; the other arm and both "
                    "hands are not connected.",
                    flush=True,
                )
            print(
                "[teleop] keep the human arm naturally down until 'Calibration locked'; "
                "after calibration, hold the current desired pose steadily and press 'e' to "
                "approach it gradually; 's' returns/disables, 'z' requests recovery return, 'd' performs operator-confirmed "
                "disable, 'x' for E-stop, or 'q' to quit.",
                flush=True,
            )
        elif self._wrist_imu_side:
            print(
                f"[teleop] {self._wrist_imu_side}-hand + three-DOF terminal IMU mode; arm is DISARMED.",
                flush=True,
            )
            print(
                f"[teleop] press 'h' for hand, 'e' for {self._wrist_imu_side} wrist, "
                "'s' to stop, 'i' to toggle IMU diagnostics, 'q' to quit.",
                flush=True,
            )
        else:
            print("[teleop] waiting for body calibration...", flush=True)
        if self._hand_only:
            print("[teleop] press 'h' to enable hands, 'q' to quit.", flush=True)

        while not self._stop:
            frame_start = time.monotonic()
            previous_start = getattr(self, "_previous_cycle_start", frame_start)
            self._cycle_timings = {"cycle_period_ms": (frame_start - previous_start) * 1000,
                                   "previous_cycle_work_ms": getattr(self, "_previous_cycle_work_ms", None)}
            self._previous_cycle_start = frame_start

            hand_joints = self._body.hand_joints() if self._hand is not None else None
            if (
                getattr(self, "_arm_with_hand", False)
                and self._arm_armed
                and hand_joints is None
            ):
                self._hand.set_enabled(False)
                self._hand_enabled = False
                self._stop_full_arm_for_fault(
                    f"{self._arm_side} SenseGlove finger targets became stale"
                )
                print(
                    "[teleop] LINKED SAFETY STOP: hand following disabled.",
                    flush=True,
                )
            if (
                self._hand is not None
                and hand_joints is not None
                and time.monotonic() - last_hand >= hand_interval
            ):
                self._hand.command(hand_joints)
                last_hand = time.monotonic()
                if self._hand_enabled and not self._hand.is_enabled():
                    self._hand_enabled = False
                    if getattr(self, "_arm_with_hand", False) and self._arm_armed:
                        self._stop_full_arm_for_fault(
                            f"{self._arm_side} LinkerHand feedback became stale"
                        )
                        print(
                            "[teleop] LINKED SAFETY STOP: hand following disabled.",
                            flush=True,
                        )
            if self._hand is not None and self._cfg.hand.mode == "ros2":
                self._hand.spin_once()

            if self._hand_only:
                elapsed = time.monotonic() - frame_start
                if elapsed < loop_dt:
                    time.sleep(loop_dt - elapsed)
                continue

            if self._wrist_imu_side:
                side = self._wrist_imu_side
                imu_delta = self._body.hand_orientation_delta(side)
                offsets = None
                if imu_delta is not None and self._wrist_local_axes is not None:
                    try:
                        offsets = self._wrist_offsets(imu_delta)
                        self._print_wrist_diagnostics(imu_delta, offsets, frame_start)
                    except (TypeError, ValueError, np.linalg.LinAlgError):
                        offsets = None
                if self._arm_armed:
                    if imu_delta is None:
                        self._stop_wrist_for_fault("glove IMU became stale")
                    elif self._driver is None:
                        # Keep checking the calibrated IMU stream in --no-robot mode.
                        pass
                    elif offsets is None:
                        self._stop_wrist_for_fault("kinematic mapping is unavailable")
                    else:
                        commanded = self._driver.command_offsets(offsets, loop_dt)
                        if not commanded:
                            self._stop_wrist_for_fault(
                                "invalid/missing arm feedback", allow_return=False
                            )
                elapsed = time.monotonic() - frame_start
                if elapsed < loop_dt:
                    time.sleep(loop_dt - elapsed)
                continue

            # Single-arm arming runs in a worker so stop/disable keys stay
            # responsive.  Do not let the preview loop mutate the same body
            # filters or IK seed while that worker rebases them to fresh
            # measured feedback.  The lock also closes the small race where a
            # control cycle began immediately before the worker became alive.
            if self._arm_only and self._arm_startup_active():
                elapsed = time.monotonic() - frame_start
                if elapsed < loop_dt:
                    time.sleep(loop_dt - elapsed)
                continue
            # A preview cycle must finish before startup can rebase the robot.
            # Locking advance/solve separately leaves old local targets and
            # feedback alive across the entire enable worker. Keep acquisition,
            # solving, fault decisions, sending and FK diagnostics in one epoch.
            cycle_lock = getattr(self, "_arm_control_lock", None)
            with cycle_lock if cycle_lock is not None else nullcontext():
                if self._arm_only and self._arm_startup_active():
                    continue
                stage_start = time.monotonic()
                targets = self._body.advance()
                self._cycle_timings["retarget_ms"] = (time.monotonic() - stage_start) * 1000
                if targets is None:
                    if (
                        getattr(self, "_arm_only", False)
                        and self._arm_armed
                        and not self._body.is_ready()
                    ):
                        imu_missing = (
                            self._cfg.retarget.require_hand_imu_for_active_arm
                            and self._body.hand_orientation_delta(self._arm_side) is None
                        )
                        reason = (
                            f"{self._arm_side} SenseGlove IMU became stale"
                            if imu_missing
                            else "PICO body data became stale"
                        )
                        if getattr(self, "_arm_with_hand", False) and self._hand is not None:
                            self._hand.set_enabled(False)
                            self._hand_enabled = False
                        self._stop_full_arm_for_fault(reason)
                    time.sleep(0.005)
                    continue

                stage_start = time.monotonic()
                current = self._read_current_joints()
                self._cycle_timings["feedback_read_ms"] = (time.monotonic() - stage_start) * 1000
                if current is None:
                    if self._arm_armed and self._trajectory_active():
                        failure = getattr(self._driver, "feedback_failure_diagnostics", None)
                        detail = failure.get("reason") if isinstance(failure, dict) else None
                        transient = bool(failure.get("transient")) if isinstance(failure, dict) else False
                        if transient:
                            if not getattr(self, "_feedback_gap_notice_active", False):
                                print(
                                    "[teleop SAFETY] brief joint-feedback gap: holding the last "
                                    "controller target; no new position command is being sent.",
                                    flush=True,
                                )
                            self._feedback_gap_notice_active = True
                        else:
                            self._stop_full_arm_for_fault("joint feedback unavailable" + (f": {detail}" if detail else ""))
                    time.sleep(0.005)
                    continue
                if getattr(self, "_feedback_gap_notice_active", False):
                    print(
                        "[teleop SAFETY] joint feedback recovered and passed the trajectory "
                        "safety recheck; bounded following resumed.",
                        flush=True,
                    )
                    self._feedback_gap_notice_active = False

                partitioned_side = None
                terminal_targets = None
                compensate_full_arm_imu = False
                if self._arm_only and self._cfg.ik.partition_terminal_wrist_ik:
                    imu_delta = self._body.hand_orientation_delta(self._arm_side)
                    try:
                        if imu_delta is None:
                            # PICO-only commissioning: do not infer terminal
                            # orientation from the tracker. Preserve measured URDF
                            # J5/J6/J7 while J1..J4 solve the arm-segment positions.
                            side_offset = 0 if self._arm_side == "left" else 7
                            terminal_targets = current[side_offset + 4:side_offset + 7].copy()
                        else:
                            # First solve shoulder/elbow geometry at neutral wrist.
                            # The second solve below removes this arm-generated
                            # world rotation from the glove's desired palm pose.
                            terminal_targets = self._full_arm_terminal_targets(np.eye(3))
                            compensate_full_arm_imu = True
                        partitioned_side = self._arm_side
                    except (TypeError, ValueError, np.linalg.LinAlgError) as exc:
                        if self._arm_armed:
                            self._stop_full_arm_for_fault(
                                f"SenseGlove wrist mapping failed: {exc}"
                            )
                        continue

                stage_start = time.monotonic()
                # Let a newly queued startup worker acquire the lock without
                # spending another solve on a preview that will be discarded.
                if self._arm_only and self._arm_startup_active():
                    continue
                arm_targets = self._ik.solve(
                    left_wrist_pose=targets["left_wrist"],
                    right_wrist_pose=targets["right_wrist"],
                    current_arm_joint_pos=current,
                    left_elbow_pose=targets["left_elbow"] if self._cfg.ik.enable_elbow_tasks else None,
                    right_elbow_pose=targets["right_elbow"] if self._cfg.ik.enable_elbow_tasks else None,
                    partitioned_side=partitioned_side,
                    terminal_joint_targets=terminal_targets,
                )

                if compensate_full_arm_imu:
                    try:
                        terminal_targets = self._compensated_full_arm_terminal_targets(
                            targets[f"{self._arm_side}_wrist"], arm_targets
                        )
                        arm_targets = self._ik.solve(
                            left_wrist_pose=targets["left_wrist"],
                            right_wrist_pose=targets["right_wrist"],
                            current_arm_joint_pos=current,
                            left_elbow_pose=(
                                targets["left_elbow"]
                                if self._cfg.ik.enable_elbow_tasks else None
                            ),
                            right_elbow_pose=(
                                targets["right_elbow"]
                                if self._cfg.ik.enable_elbow_tasks else None
                            ),
                            partitioned_side=partitioned_side,
                            terminal_joint_targets=terminal_targets,
                        )
                    except (KeyError, TypeError, ValueError, np.linalg.LinAlgError) as exc:
                        if self._arm_armed:
                            if getattr(self, "_arm_with_hand", False) and self._hand is not None:
                                self._hand.set_enabled(False)
                                self._hand_enabled = False
                            self._stop_full_arm_for_fault(
                                f"full-arm IMU compensation failed: {exc}"
                            )
                        continue

                self._cycle_timings["ik_and_wrist_compensation_ms"] = (time.monotonic() - stage_start) * 1000
                position_limit_hold = False
                if not self._ik.last_solution_valid:
                    position_limit_hold = bool(
                        self._arm_armed
                        and self._ik.solution_diagnostics.get(
                            "recoverable_position_limit", False
                        )
                    )
                    if position_limit_hold:
                        self._position_limit_recovery_count = 0
                        self._report_position_limit_hold(True)
                    elif self._arm_armed:
                        self._record_startup_event({
                            "phase": "ik_geometry_rejected",
                            "timestamp": time.time(),
                            "monotonic_s": time.monotonic(),
                            "diagnostics": dict(self._ik.solution_diagnostics),
                            "current_urdf_rad": np.asarray(current, dtype=float).tolist(),
                            "retarget_positions_m": {
                                name: np.asarray(targets[name][:3], dtype=float).tolist()
                                for name in (f"{self._arm_side}_elbow", f"{self._arm_side}_wrist")
                            },
                        })
                        self._stop_full_arm_for_fault(
                            f"IK geometry rejected: {self._ik.solution_diagnostics}")
                        continue
                    else:
                        continue
                else:
                    position_limit_hold = self._position_limit_recovery_pending(
                        self._ik.solution_diagnostics
                    )
                    if not position_limit_hold:
                        self._position_limit_recovery_count = 0
                        self._report_position_limit_hold(False)
                        self._report_workspace_limit()
                stage_start = time.monotonic()
                if self._driver is not None and self._arm_armed:
                    commanded = (
                        self._driver.hold_command_trajectory()
                        if position_limit_hold
                        else self._driver.command_full_urdf(arm_targets)
                    )
                    if position_limit_hold and commanded:
                        held = self._driver.last_commanded_full_urdf()
                        if held is not None:
                            arm_targets = held
                    if not commanded and not self._command_rejected:
                        reason = getattr(self._driver, "safety_fault_reason", None)
                        print(
                            f"[teleop] command rejected: {reason or 'invalid/missing arm feedback'}.",
                            flush=True,
                        )
                    if not commanded and self._arm_only:
                        self._arm_armed = False
                        self._manual_intervention_required = True
                        if self._trajectory_active() and getattr(self, "_arm_with_hand", False) and self._hand is not None:
                            self._hand.set_enabled(False)
                            self._hand_enabled = False
                        fault_context = (getattr(self._driver, "trajectory_diagnostics", None) or {}).get("fault_context", {})
                        controller_stop = fault_context.get("controller_stop")
                        if controller_stop:
                            stop_text = (
                                "Electronic stop send returned; controller stop and motor disable are not yet verified. "
                                if controller_stop.get("send_returned") else
                                "ELECTRONIC STOP SEND FAILED; use the physical emergency stop. "
                            )
                        else:
                            stop_text = (
                                "No zero target or disable command was sent. The last controller target "
                                "is retained and may still execute; stopping target updates does not prove standstill. "
                            )
                        print(
                            f"[teleop] {self._arm_side.upper()} ARM SAFETY STOP: "
                            f"{getattr(self._driver, 'safety_fault_reason', None) or 'feedback invalid'}; "
                            f"Following is locked. {stop_text}"
                            "Keep supporting the arm; 'd' requests disable, 'x' requests E-stop. "
                            "Clear the fault before requesting recovery with 'z'.",
                            flush=True,
                        )
                    self._command_rejected = not commanded
                self._cycle_timings["driver_command_ms"] = (time.monotonic() - stage_start) * 1000
                self._cycle_timings["control_before_diagnostics_ms"] = (time.monotonic() - frame_start) * 1000
                packet_time = self._body._last_body_frame_packet_time_monotonic
                self._cycle_timings["body_packet_age_ms"] = None if packet_time is None else (time.monotonic() - packet_time) * 1000
                self._publish_arm_diagnostics(targets, arm_targets, current)

            elapsed = time.monotonic() - frame_start
            self._previous_cycle_work_ms = elapsed * 1000
            if elapsed < loop_dt:
                time.sleep(loop_dt - elapsed)

        self.close()

    def _position_limit_recovery_pending(self, diagnostics: dict) -> bool:
        """Require a short geometric hysteresis before leaving a limit hold."""
        if not getattr(self, "_position_limit_hold_active", False):
            return False
        geometry_error = max(
            float(diagnostics.get("elbow_error_m", float("inf"))),
            float(diagnostics.get("wrist_error_m", float("inf"))),
        )
        if geometry_error <= self._cfg.ik.position_limit_recovery_error_m:
            self._position_limit_recovery_count = (
                getattr(self, "_position_limit_recovery_count", 0) + 1
            )
        else:
            self._position_limit_recovery_count = 0
        return (
            self._position_limit_recovery_count
            < self._cfg.ik.position_limit_recovery_frames
        )

    def _report_position_limit_hold(self, active: bool):
        """Report a recoverable pause at a measured commissioned joint bound."""
        active = bool(active)
        if active == getattr(self, "_position_limit_hold_active", False):
            return
        self._position_limit_hold_active = active
        diagnostics = dict(getattr(self._ik, "solution_diagnostics", {}) or {})
        if active:
            bounds = diagnostics.get("active_position_bounds", [])
            summary = ", ".join(
                f"J{item['joint']} {item['bound']} "
                f"({np.degrees(item['limit_rad']):.1f}deg)"
                for item in bounds
            ) or "commissioned joint bound"
            print(
                f"[teleop] {self._arm_side.upper()} TARGET PAUSED at {summary}: "
                "controlled braking/hold is active; move the PICO arm back into "
                "the reachable workspace to resume automatically.",
                flush=True,
            )
        else:
            print(
                f"[teleop] {self._arm_side.upper()} target returned inside the "
                "commissioned joint workspace; following resumed.",
                flush=True,
            )
        self._record_startup_event(dict(
            phase=("ik_position_limit_hold" if active
                   else "ik_position_limit_recovered"),
            timestamp=time.time(), monotonic_s=time.monotonic(),
            diagnostics=diagnostics,
        ))

    def _report_workspace_limit(self):
        """Report input saturation separately from IK failure, once per transition."""
        if not self._arm_armed or not getattr(self, "_arm_only", False):
            return
        diagnostics = getattr(self._ik, "solution_diagnostics", {})
        if not isinstance(diagnostics, dict):
            return
        limited = bool(diagnostics.get("workspace_limited", False))
        if limited == getattr(self, "_workspace_limit_active", False):
            return
        self._workspace_limit_active = limited
        if limited:
            print(
                f"[teleop] {self._arm_side.upper()} TARGET LIMITED: requested bend "
                f"{diagnostics['requested_bend_deg']:.1f}deg exceeds the wrist-dependent "
                f"reachable bend {diagnostics['bend_limit_deg']:.1f}deg; "
                f"J4 stays within {diagnostics['j4_upper_limit_deg']:.1f}deg. "
                "Following continues with the bounded target.", flush=True,
            )
        else:
            print(f"[teleop] {self._arm_side.upper()} target returned inside the elbow workspace.", flush=True)
        self._record_startup_event(dict(
            phase="ik_workspace_limited" if limited else "ik_workspace_recovered",
            timestamp=time.time(), monotonic_s=time.monotonic(), diagnostics=dict(diagnostics)))

    def _record_startup_event(self, event):
        probe = getattr(self, "_arm_probe", None)
        if probe is not None:
            probe.submit(dict(event, event="startup_phase"))
        if self._startup_writer is None:
            path = self._elbow_log_path.with_name(self._elbow_log_path.name.replace("pico_elbow", "pico_startup"))
            self._startup_writer = AsyncJsonLog(path)
            print(f"[teleop] startup diagnostic log: {path}", flush=True)
        self._startup_writer.submit(dict(event, side=self._arm_side))

    def _publish_arm_diagnostics(
        self, targets: dict[str, np.ndarray], ik_targets: np.ndarray, feedback: np.ndarray
    ) -> None:
        if self._arm_diagnostics_socket is None or self._ik is None:
            return
        now = time.monotonic()
        if now - self._last_arm_diagnostics_time < 0.1:
            return
        self._last_arm_diagnostics_time = now
        try:
            ik_poses = self._ik.current_task_frame_poses(ik_targets)
            feedback_poses = self._ik.current_task_frame_poses(feedback)
            side = self._arm_side
            commanded = (
                self._driver.last_commanded_full_urdf()
                if self._driver is not None and hasattr(self._driver, "last_commanded_full_urdf")
                else None
            )
            commanded_poses = (
                self._ik.current_task_frame_poses(commanded) if commanded is not None else None
            )

            def frames_from(poses):
                if poses is None:
                    return None
                return {
                    name: {"pos": np.asarray(pose[:3], dtype=float).tolist()}
                    for name, pose in poses.items()
                    if name.startswith(f"{side}_")
                }

            target_frames = {
                f"{side}_shoulder": {"pos": feedback_poses[f"{side}_shoulder"][:3].tolist()},
                f"{side}_elbow": {"pos": targets[f"{side}_elbow"][:3].tolist()},
                f"{side}_wrist": {"pos": targets[f"{side}_wrist"][:3].tolist()},
            }

            def position_errors(reference, actual):
                if reference is None or actual is None:
                    return None
                shoulder = np.asarray(reference[f"{side}_shoulder"][:3], dtype=float)
                ref_elbow = np.asarray(reference[f"{side}_elbow"][:3], dtype=float)
                ref_wrist = np.asarray(reference[f"{side}_wrist"][:3], dtype=float)
                act_elbow = np.asarray(actual[f"{side}_elbow"][:3], dtype=float)
                act_wrist = np.asarray(actual[f"{side}_wrist"][:3], dtype=float)

                def direction_error(a, b):
                    a_norm = np.linalg.norm(a)
                    b_norm = np.linalg.norm(b)
                    if a_norm <= 1.0e-9 or b_norm <= 1.0e-9:
                        return None
                    cosine = np.clip(np.dot(a, b) / (a_norm * b_norm), -1.0, 1.0)
                    return float(np.degrees(np.arccos(cosine)))

                return {
                    "elbow_cm": float(100.0 * np.linalg.norm(act_elbow - ref_elbow)),
                    "wrist_cm": float(100.0 * np.linalg.norm(act_wrist - ref_wrist)),
                    "upper_deg": direction_error(ref_elbow - shoulder, act_elbow - shoulder),
                    "forearm_deg": direction_error(
                        ref_wrist - ref_elbow, act_wrist - act_elbow
                    ),
                }

            reference_poses = {
                f"{side}_shoulder": feedback_poses[f"{side}_shoulder"],
                f"{side}_elbow": targets[f"{side}_elbow"],
                f"{side}_wrist": targets[f"{side}_wrist"],
            }
            offset = 0 if side == "left" else 7
            elbow_diagnostic = self._body.elbow_diagnostics(side)
            if elbow_diagnostic is not None:
                elbow_diagnostic = dict(elbow_diagnostic)
                points = {name: reference_poses[f"{side}_{name}"][:3]
                          for name in ("shoulder", "elbow", "wrist")}
                elbow_diagnostic.update(
                    retarget_deg=float(np.degrees(BodyDevice._points_flexion(points))),
                    ik_j4_deg=float(np.degrees(ik_targets[offset + 3])),
                    last_sent_j4_deg=None if commanded is None else float(np.degrees(commanded[offset + 3])),
                    feedback_j4_deg=float(np.degrees(feedback[offset + 3])),
                    solver=getattr(self._ik, "_last_elbow_solve_diagnostics", {}).get(side),
                    driver=getattr(self._driver, "elbow_command_diagnostics", None),
                )
            retarget_pipeline = None
            retarget_reader = getattr(self._body, "arm_retarget_diagnostics", None)
            if callable(retarget_reader):
                candidate_pipeline = retarget_reader(side)
                if isinstance(candidate_pipeline, dict):
                    retarget_pipeline = candidate_pipeline
            message = {
                "timestamp": time.time(),
                "side": side,
                "armed": bool(self._arm_armed),
                "command_accepted": not self._command_rejected,
                "command_skipped_duplicate_feedback": getattr(self._driver, "last_command_skip_reason", None) == "duplicate_feedback",
                "command_skip_reason": getattr(self._driver, "last_command_skip_reason", None),
                "elbow": elbow_diagnostic,
                "retarget_pipeline": retarget_pipeline,
                "ik_solver": dict(getattr(self._ik, "solution_diagnostics", {})),
                "trajectory": getattr(self._driver, "trajectory_diagnostics", None),
                # Full startup snapshots are already persisted once in the
                # startup log.  Repeating them in every ~10 Hz arm diagnostic
                # row made a short fault-latched session grow by tens of MB.
                "startup_events": [
                    {
                        key: event[key]
                        for key in ("timestamp", "phase", "event", "reason")
                        if key in event
                    }
                    for event in getattr(self._driver, "startup_events", [])[-8:]
                    if isinstance(event, dict)
                ],
                "timings": dict(getattr(self, "_cycle_timings", {})),
                "frames": {
                    "retarget": target_frames,
                    "ik": frames_from(ik_poses),
                    "command": frames_from(commanded_poses),
                    "feedback": frames_from(feedback_poses),
                },
                "joints_deg": {
                    "ik": np.degrees(ik_targets[offset:offset + 7]).tolist(),
                    "command": None if commanded is None else np.degrees(commanded[offset:offset + 7]).tolist(),
                    "feedback": np.degrees(feedback[offset:offset + 7]).tolist(),
                },
                "errors": {
                    "ik_vs_retarget": position_errors(reference_poses, ik_poses),
                    "command_vs_retarget": position_errors(reference_poses, commanded_poses),
                    "feedback_vs_retarget": position_errors(reference_poses, feedback_poses),
                },
            }
            log_path = getattr(self, "_elbow_log_path", None)
            if log_path is not None:
                if getattr(self, "_diagnostic_writer", None) is None:
                    self._diagnostic_writer = AsyncJsonLog(log_path)
                    print(f"[teleop] arm diagnostic log: {log_path}", flush=True)
                message["log_dropped"] = self._diagnostic_writer.dropped
                self._diagnostic_writer.submit(message)
            payload = json.dumps(message, separators=(",", ":"), allow_nan=False,
                                 default=numpy_json_default).encode("utf-8")
            self._arm_diagnostics_socket.sendto(payload, ("127.0.0.1", 15060))
        except (KeyError, TypeError, ValueError, OSError):
            return

    def close(self) -> None:
        self._stop = True
        worker = getattr(self, "_return_thread", None)
        if worker is not None and worker.is_alive():
            self._driver.cancel_return()
            worker.join()  # Cancellation is polled by bounded planner/control loops.
            self._manual_intervention_required = True
        startup_writer = getattr(self, "_startup_writer", None)
        if startup_writer is not None:
            startup_writer.close()
        writer = getattr(self, "_diagnostic_writer", None)
        if writer is not None:
            writer.close()
        if self._wrist_imu_side and self._driver is not None and self._motors_enabled:
            self._return_wrist_to_zero_and_disable("program exit")
        elif (
            getattr(self, "_arm_only", False)
            and self._driver is not None
            and self._motors_enabled
            and not getattr(self, "_manual_intervention_required", False)
            and not getattr(self._driver, "_return_inhibited", False)
        ):
            if sys.stdin.isatty():
                old_key = getattr(self, "_key_thread", None)
                if old_key is not None:
                    old_key.join(timeout=.2)
                self._stop = False
                if old_key is None or not old_key.is_alive():
                    self._key_thread = threading.Thread(target=self._key_loop, daemon=True)
                    self._key_thread.start()
                self._start_full_arm_return("program exit")
                worker = getattr(self, "_return_thread", None)
                if worker is not None:
                    worker.join()
                self._stop = True
            else:
                print("[teleop] Exit return skipped without interactive stop control; following locked.", flush=True)
        if self._return_hand_open_on_close and not getattr(self, "_arm_only", False):
            self._return_hand_open_on_close = False
            try:
                opened = self._hand is not None and self._hand.return_to_open()
                print(
                    "[teleop] HAND RETURNED TO NATURAL OPEN POSE."
                    if opened
                    else "[teleop] HAND OPEN SKIPPED: fresh hand feedback unavailable.",
                    flush=True,
                )
            except Exception as exc:  # noqa: BLE001
                print(f"[teleop] hand open during quit failed: {exc}", flush=True)
        try:
            self._body.close()
        except Exception:  # noqa: BLE001
            pass
        diagnostics_socket = getattr(self, "_arm_diagnostics_socket", None)
        if diagnostics_socket is not None:
            diagnostics_socket.close()
            self._arm_diagnostics_socket = None
        try:
            if self._hand is not None:
                self._hand.close()
        except Exception:  # noqa: BLE001
            pass
        if self._driver is not None:
            probe = getattr(self, "_arm_probe", None)
            if probe is not None:
                self._driver.probe_sink = None
                probe.close()
                self._arm_probe = None
            try:
                self._driver.disconnect()
            except Exception:  # noqa: BLE001
                pass


def default_urdf_path() -> str:
    return os.path.join(project_root(), "urdf", "esrobo_waist_with_head.urdf")


def project_root() -> str:
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.dirname(os.path.dirname(here))


def _run_until_safe_exit(node: TeleopNode) -> None:
    """Keep Ctrl-C from disconnecting a fault-latched, still-enabled arm."""
    while True:
        try:
            node.run()
        except KeyboardInterrupt:
            if getattr(node, "_startup_power_cycle_required", False) is True:
                if node._exit_for_startup_power_cycle("Ctrl-C after startup feedback failure"):
                    return
                node._stop = False
                continue
            if node._enabled_fault_requires_explicit_disable():
                node._stop = False
                node._report_enabled_fault_exit_refusal("INTERRUPT EXIT")
                continue
            return
        if node._enabled_fault_requires_explicit_disable():
            node._stop = False
            node._report_enabled_fault_exit_refusal("EXIT")
            continue
        return


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="ESROBO real-machine teleoperation")
    parser.add_argument("--no-robot", action="store_true", help="run IK/hand without commanding arms")
    parser.add_argument("--hand-only", action="store_true", help="only do SenseGlove hand teleop (no arms/IK)")
    parser.add_argument("--hand-mode", choices=["udp", "ros2"], default=None)
    parser.add_argument("--hand-side", choices=["both", "left", "right"], default="both")
    parser.add_argument(
        "--arm-only",
        action="store_true",
        help="single-side PICO full-arm teleoperation (optionally with --with-hand)",
    )
    parser.add_argument(
        "--arm-side",
        choices=["left", "right"],
        default="left",
        help="physical NERO side used with --arm-only",
    )
    parser.add_argument(
        "--with-hand",
        action="store_true",
        help="with --arm-only, also initialize the matching single LinkerHand",
    )
    parser.add_argument(
        "--require-hand-imu",
        action="store_true",
        help="in --arm-only mode, require fresh SenseGlove IMU for J5/J6/J7",
    )
    parser.add_argument(
        "--left-wrist-imu",
        action="store_true",
        help="left glove + hand, with only left NERO physical joints 5/6/7 following IMU",
    )
    parser.add_argument(
        "--right-wrist-imu",
        action="store_true",
        help="right glove + hand, with only right NERO physical joints 5/6/7 following IMU",
    )
    parser.add_argument("--config", default="", help="path to teleop_config.yaml")
    parser.add_argument("--urdf", default="", help="path to the full ESROBO URDF")
    parser.add_argument("--enable", action="store_true", help="arm robot immediately on start")
    args = parser.parse_args(argv)

    cfg = build_config()
    if args.config:
        cfg = load_config(args.config)
    if args.urdf:
        cfg.ik.urdf_path = args.urdf
    if not os.path.isabs(cfg.ik.urdf_path):
        cfg.ik.urdf_path = os.path.join(project_root(), cfg.ik.urdf_path)
    if args.hand_mode:
        cfg.hand.mode = args.hand_mode
    if args.require_hand_imu:
        cfg.retarget.require_hand_imu_for_active_arm = True

    if args.left_wrist_imu and args.right_wrist_imu:
        parser.error("--left-wrist-imu and --right-wrist-imu cannot be used together")
    if args.hand_only and (args.left_wrist_imu or args.right_wrist_imu):
        parser.error("--hand-only and --*-wrist-imu cannot be used together")
    if args.arm_only and (args.hand_only or args.left_wrist_imu or args.right_wrist_imu):
        parser.error("--arm-only cannot be combined with hand-only or wrist-IMU modes")
    if args.with_hand and not args.arm_only:
        parser.error("--with-hand requires --arm-only")
    if args.arm_only and args.with_hand and args.hand_side != args.arm_side:
        parser.error("--with-hand requires --hand-side to match --arm-side")
    if args.arm_only and not args.with_hand and args.hand_side != "both":
        parser.error("--hand-side is only used with --arm-only when --with-hand is set")
    if args.require_hand_imu and not args.arm_only:
        parser.error("--require-hand-imu requires --arm-only")
    if args.left_wrist_imu and args.hand_side != "left":
        parser.error("--left-wrist-imu requires --hand-side left")
    if args.right_wrist_imu and args.hand_side != "right":
        parser.error("--right-wrist-imu requires --hand-side right")
    hand_only = args.hand_only
    node = TeleopNode(
        cfg,
        enable_robot=not (args.no_robot or hand_only),
        hand_mode=cfg.hand.mode,
        hand_only=hand_only,
        hand_side=args.hand_side,
        right_wrist_imu=args.right_wrist_imu,
        left_wrist_imu=args.left_wrist_imu,
        arm_only=args.arm_only,
        arm_side=args.arm_side,
        arm_with_hand=args.with_hand,
    )
    exit_code = 0
    try:
        node.start()
        if args.enable:
            if node._hand is not None:
                node._hand_enabled = node._hand.set_enabled(True)
            node._arm_armed = node._arm_robot()
        _run_until_safe_exit(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:  # noqa: BLE001
        print(f"[teleop] FATAL: {exc}", flush=True)
        exit_code = 1
    finally:
        node.close()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
