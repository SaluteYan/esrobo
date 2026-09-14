"""Real-machine dual-arm + LinkerHand teleoperation node for the ESROBO robot.

Wiring:  PICO full-body UDP (BodyDevice) -> arm_vector retargeting -> Pink IK
(IkSolver) -> NERO arms (NeroDualArmDriver).  SenseGlove hand joints from the
same UDP stream -> LinkerHand (LinkerHandDriver).

Safety: the robot arms are only commanded while teleoperation is armed
(default off).  Press 'e' to enable/arm, 's' to stop/hold, 'q' to quit.
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import socket
import sys
import threading
import time

import numpy as np

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

        active_sides = ("left", "right") if hand_side == "both" else (hand_side,)
        self._hand = None if arm_only and not self._arm_with_hand else LinkerHandDriver(
            cfg.hand, udp_port=cfg.hand.udp_port, active_sides=active_sides
        )

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
        self._arm_diagnostics_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if arm_only else None
        self._last_arm_diagnostics_time = 0.0
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
        self._key_thread = threading.Thread(target=self._key_loop, daemon=True)
        self._key_thread.start()

    def _ensure_startup_zero_pose(self) -> None:
        """Align robot zero after glove calibration, then gate keyboard control."""
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
                    input(
                        f"[等待操作] {self._arm_side} 机械臂当前处于失能非零位。"
                        "请人工托稳机械臂并清空周围夹点，按 Enter 后将从当前实测位置使能，"
                        "再按该侧低速限制缓慢回到自然下垂零位并失能；按 Ctrl-C 取消："
                    )
                    print(
                        f"[teleop ZERO] {self._arm_side.upper()} startup alignment: "
                        "holding measured feedback as the first target, then slowly returning "
                        "J4..J7 first and J1..J3 second to natural-down zero.",
                        flush=True,
                    )
                    returned, disabled = self._driver.align_startup_zero_and_disable()
                    self._motors_enabled = not disabled
                    if not returned or not disabled:
                        raise RuntimeError(
                            f"{self._arm_side} startup natural-down alignment failed: "
                            f"zero_verified={returned}, disable_verified={disabled}"
                        )
                    print(
                        f"[teleop ZERO] {self._arm_side.upper()} FULL-ARM NATURAL-DOWN ZERO "
                        "VERIFIED; all joints are DISABLED. PICO calibration may continue.",
                        flush=True,
                    )
                else:
                    print(
                        f"[teleop ZERO] {self._arm_side.upper()} FULL-ARM NATURAL-DOWN ZERO "
                        "VERIFIED; motors remain DISABLED.",
                        flush=True,
                    )
            if getattr(self, "_hand", None) is None:
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
            print(
                f"[teleop ZERO] {side} hand current={np.rint(current).astype(int).tolist()} "
                f"target(open-feedback-zero)={np.rint(target).astype(int).tolist()} "
                f"max_error={float(np.max(np.abs(error))):.1f}/255.",
                flush=True,
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
        if ch == "e":
            if getattr(self, "_arm_with_hand", False):
                if self._body.hand_joints() is None:
                    print(
                        "[teleop] LINKED ENABLE REFUSED: fresh left SenseGlove finger "
                        "targets are unavailable.",
                        flush=True,
                    )
                    self._arm_armed = False
                    self._hand_enabled = False
                    return
                if self._hand is None or not self._hand.feedback_ready():
                    print(
                        "[teleop] LINKED ENABLE REFUSED: fresh left-hand feedback is unavailable.",
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
                        _, disabled = self._return_full_arm_to_zero_and_disable(
                            "left-hand feedback disappeared during linked enable"
                        )
                        print(
                            "[teleop] LINKED ENABLE CANCELLED: hand feedback disappeared; "
                            + (
                                "left arm was returned to zero and disabled."
                                if disabled
                                else "left arm remains enabled in manual-intervention hold."
                            ),
                            flush=True,
                        )
                    else:
                        print(
                            "[teleop] LEFT ARM + LEFT HAND LINKED FOLLOWING ENABLED.",
                            flush=True,
                        )
            else:
                self._arm_armed = self._arm_robot()
        elif ch == "s":
            self._arm_armed = False
            linked_hand = getattr(self, "_arm_with_hand", False) and self._hand is not None
            if linked_hand:
                self._hand.set_enabled(False)
                self._hand_enabled = False
            if self._wrist_imu_side and self._driver is not None:
                self._return_wrist_to_zero_and_disable("operator pressed 's'")
            elif getattr(self, "_arm_only", False) and self._driver is not None:
                self._return_full_arm_to_zero_and_disable("operator pressed 's'")
            else:
                print(
                    "[teleop] ARM FOLLOWING STOPPED - controller holding position.",
                    flush=True,
                )
            if linked_hand:
                opened = self._hand.return_to_open()
                print(
                    "[teleop] LEFT HAND RETURNED TO NATURAL OPEN POSE."
                    if opened else
                    "[teleop] LEFT HAND OPEN SKIPPED: fresh feedback unavailable.",
                    flush=True,
                )
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
            self._manual_intervention_required = False
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
            if getattr(self, "_arm_only", False):
                if self._motors_enabled:
                    print(
                        f"[teleop] quitting: slowly returning {self._arm_side} J4..J7, "
                        "then J1..J3 to natural-down zero before disabling the arm.",
                        flush=True,
                    )
                    _, disabled = self._return_full_arm_to_zero_and_disable(
                        "operator pressed 'q'"
                    )
                    if not disabled:
                        self._stop = False
                        print(
                            f"[teleop] QUIT CANCELLED: {self._arm_side} arm remains "
                            "enabled with following locked. Keep supporting it; press "
                            "'s' to retry zero return, 'd' to disable, or 'x' for E-stop.",
                            flush=True,
                        )
                        return
                else:
                    print(
                        f"[teleop] quitting: {self._arm_side} arm is already disabled; "
                        "no return motion will be sent.",
                        flush=True,
                    )
                self._stop = True
            else:
                self._stop = True
                print(
                    "[teleop] quitting: returning wrist J5/J6/J7 to zero, disabling arm, "
                    "then returning hand to open pose.",
                    flush=True,
                )

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
            "J4..J7 first, then J1..J3 to natural-down zero before disable.",
            flush=True,
        )
        returned, disabled = self._driver.return_to_zero_and_disable()
        self._motors_enabled = not disabled
        self._manual_intervention_required = not returned and not disabled
        if returned:
            status = "ZERO VERIFIED; DISABLED" if disabled else "ZERO VERIFIED; DISABLE NOT CONFIRMED"
        elif disabled:
            status = "ZERO RETURN NOT CONFIRMED; DISABLED BY CONFIGURED FALLBACK"
        else:
            status = (
                "ZERO RETURN NOT CONFIRMED; FOLLOWING LOCKED, ARM REMAINS ENABLED. "
                "Press 's' to retry, 'd' to disable, or 'x' for E-stop"
            )
        print(f"[teleop] {self._arm_side.upper()} ARM {status}.", flush=True)
        return returned, disabled

    def _arm_robot(self) -> bool:
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

        if not self._body.is_ready():
            print("[teleop] ARMING REFUSED: body tracking/reference is not ready.", flush=True)
            return False
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
                print(
                    f"[teleop] ARMING REFUSED: {self._arm_side} arm returned no complete "
                    "seven-joint feedback at natural-down zero. Broadcast and J1..J7 "
                    "disable commands were sent. Do not repeatedly press 'e'; power-cycle "
                    "the arm controller, then restart this program.",
                    flush=True,
                )
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
            self._motors_enabled = self._driver.enable()
            if not self._motors_enabled:
                print("[teleop] ARMING REFUSED: motor enable failed.", flush=True)
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

    def _read_current_joints(self) -> np.ndarray | None:
        if self._driver is None:
            if self._current_arm_joints is None:
                self._current_arm_joints = np.zeros(14, dtype=np.float64)
            return self._current_arm_joints.copy()
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
                "then press 'e' to enable, 's' to return/disable, 'd' for operator-confirmed "
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

            hand_joints = self._body.hand_joints() if self._hand is not None else None
            if (
                getattr(self, "_arm_with_hand", False)
                and self._arm_armed
                and hand_joints is None
            ):
                self._hand.set_enabled(False)
                self._hand_enabled = False
                self._return_full_arm_to_zero_and_disable(
                    "left SenseGlove finger targets became stale"
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
                        self._return_full_arm_to_zero_and_disable(
                            "left LinkerHand feedback became stale"
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

            targets = self._body.advance()
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
                    self._return_full_arm_to_zero_and_disable(reason)
                time.sleep(0.005)
                continue

            current = self._read_current_joints()
            if current is None:
                time.sleep(0.005)
                continue

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
                        self._return_full_arm_to_zero_and_disable(
                            f"SenseGlove wrist mapping failed: {exc}"
                        )
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
                        self._return_full_arm_to_zero_and_disable(
                            f"full-arm IMU compensation failed: {exc}"
                        )
                    continue

            if self._driver is not None and self._arm_armed:
                commanded = self._driver.command_full_urdf(arm_targets)
                if not commanded and not self._command_rejected:
                    print("[teleop] command rejected: invalid/missing arm feedback.", flush=True)
                if not commanded and self._arm_only:
                    self._arm_armed = False
                    self._manual_intervention_required = True
                    print(
                        f"[teleop] {self._arm_side.upper()} ARM SAFETY STOP: feedback "
                        "invalid; no zero target or disable command was sent. Following "
                        "is locked and the arm remains enabled at its last controller "
                        "target. Keep supporting it; press 's', 'd', or 'x'.",
                        flush=True,
                    )
                self._command_rejected = not commanded

            self._publish_arm_diagnostics(targets, arm_targets, current)

            elapsed = time.monotonic() - frame_start
            if elapsed < loop_dt:
                time.sleep(loop_dt - elapsed)

        self.close()

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
            offset = 0 if side == "left" else 7
            message = {
                "timestamp": time.time(),
                "side": side,
                "armed": bool(self._arm_armed),
                "command_accepted": not self._command_rejected,
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
            }
            payload = json.dumps(message, separators=(",", ":")).encode("utf-8")
            self._arm_diagnostics_socket.sendto(payload, ("127.0.0.1", 15060))
        except (KeyError, TypeError, ValueError, OSError):
            return

    def close(self) -> None:
        self._stop = True
        if self._wrist_imu_side and self._driver is not None and self._motors_enabled:
            self._return_wrist_to_zero_and_disable("program exit")
        elif (
            getattr(self, "_arm_only", False)
            and self._driver is not None
            and self._motors_enabled
        ):
            self._return_full_arm_to_zero_and_disable("program exit")
        if self._return_hand_open_on_close:
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
            try:
                self._driver.disconnect()
            except Exception:  # noqa: BLE001
                pass


def default_urdf_path() -> str:
    return os.path.join(project_root(), "urdf", "esrobo_waist_with_head.urdf")


def project_root() -> str:
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.dirname(os.path.dirname(here))


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="ESROBO real-machine teleoperation")
    parser.add_argument("--no-robot", action="store_true", help="run IK/hand without commanding arms")
    parser.add_argument("--hand-only", action="store_true", help="only do SenseGlove hand teleop (no arms/IK)")
    parser.add_argument("--hand-mode", choices=["udp", "ros2"], default=None)
    parser.add_argument("--hand-side", choices=["both", "left", "right"], default="both")
    parser.add_argument(
        "--arm-only",
        action="store_true",
        help="PICO full-arm teleoperation without initializing either LinkerHand",
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
        node.run()
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
