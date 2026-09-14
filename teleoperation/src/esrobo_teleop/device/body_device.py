"""Self-contained UDP body-tracking device with PICO ``arm_vector`` retargeting.

Pure-NumPy port of the core behaviour in the reference
``udp_bimanual_body_device.py`` (PICO full-body -> ESROBO end-effector targets),
without any IsaacLab / PyTorch dependencies.  The device receives JSON UDP
packets (body frames and/or SenseGlove ``hand_joints`` / ``hand_orientation_deltas``)
on one socket, runs arm-vector retargeting and exposes left/right elbow+wrist
target poses for the IK solver.
"""

from __future__ import annotations

import json
import socket
import threading
import time
from typing import Any, Optional

import numpy as np

from .. import math_utils as mu
from ..config import RetargetConfig

BODY_FRAME_ALIASES = {
    "pelvis": "waist",
    "waist": "waist",
    "spine1": "waist",
    "left_shoulder": "left_shoulder",
    "right_shoulder": "right_shoulder",
    "left_elbow": "left_elbow",
    "right_elbow": "right_elbow",
    "left_wrist": "left_wrist",
    "right_wrist": "right_wrist",
    "left_hand": "left_hand",
    "right_hand": "right_hand",
    "neck": "neck",
    "head": "head",
    "left_ankle": "left_ankle",
    "right_ankle": "right_ankle",
    "left_foot": "left_foot",
    "right_foot": "right_foot",
}

ARM_POINT_KEYS = ("shoulder", "elbow", "wrist")


class BodyDevice:
    """Receives PICO + SenseGlove UDP and produces robot end-effector targets."""

    def __init__(self, cfg: RetargetConfig, active_arm_side: str | None = None):
        if active_arm_side not in (None, "left", "right"):
            raise ValueError("active_arm_side must be None, 'left', or 'right'")
        self._cfg = cfg
        self._active_arm_side = active_arm_side
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((cfg.host, cfg.port))
        self._sock.settimeout(0.05)

        self._rotation = np.asarray(cfg.source_to_robot_rotation, dtype=np.float32).reshape(3, 3)
        self._signs = np.asarray(cfg.position_delta_signs, dtype=np.float32).reshape(3)
        self._hand_imu_rotation = (
            self._rotation
            if cfg.hand_imu_source_to_robot_rotation is None
            else np.asarray(cfg.hand_imu_source_to_robot_rotation, dtype=np.float32).reshape(3, 3)
        )
        self._hand_imu_local_rotvec_maps = {
            side: np.asarray(
                cfg.hand_imu_local_rotvec_map_for(side), dtype=np.float32
            ).reshape(3, 3)
            for side in ("left", "right")
        }

        self._frames: dict[str, np.ndarray] = {}
        self._hand_joint_targets: Optional[np.ndarray] = None
        self._hand_orientation_delta_matrices: dict[str, np.ndarray] = {}
        self._hand_imu_uses_calibrated_zero: dict[str, bool] = {
            "left": False,
            "right": False,
        }
        self._body_frame_history: list[tuple[float, dict[str, np.ndarray]]] = []
        self._last_packet_time_monotonic: Optional[float] = None
        self._last_body_frame_packet_time_monotonic: Optional[float] = None
        self._last_hand_joint_packet_time_monotonic: Optional[float] = None
        self._last_hand_orientation_packet_time_monotonic: Optional[float] = None
        self._printed_stale = False
        self._printed_hand_imu_frame_mismatch = False
        self._hand_imu_last_source_delta_matrices: dict[str, np.ndarray] = {}
        self._hand_imu_integrated_components: dict[str, np.ndarray] = {}

        self._initial_left_pose_matrix = mu.pose_array_to_matrix(
            np.asarray(cfg.initial_left_wrist_pose, dtype=np.float32)
        )
        self._initial_right_pose_matrix = mu.pose_array_to_matrix(
            np.asarray(cfg.initial_right_wrist_pose, dtype=np.float32)
        )
        self._initial_left_elbow_pose_matrix = mu.pose_array_to_matrix(
            np.asarray(cfg.initial_left_elbow_pose, dtype=np.float32)
        )
        self._initial_right_elbow_pose_matrix = mu.pose_array_to_matrix(
            np.asarray(cfg.initial_right_elbow_pose, dtype=np.float32)
        )
        self._robot_left_shoulder = np.asarray(cfg.robot_left_shoulder_position, dtype=np.float32)
        self._robot_right_shoulder = np.asarray(cfg.robot_right_shoulder_position, dtype=np.float32)
        self._robot_left_elbow = np.asarray(cfg.robot_left_elbow_position, dtype=np.float32)
        self._robot_right_elbow = np.asarray(cfg.robot_right_elbow_position, dtype=np.float32)
        self._robot_left_reference_rotation = np.asarray(
            cfg.robot_left_reference_rotation, dtype=np.float32
        ).reshape(3, 3)
        self._robot_right_reference_rotation = np.asarray(
            cfg.robot_right_reference_rotation, dtype=np.float32
        ).reshape(3, 3)

        # Auto-start reference (stable natural-hold pose used as the origin).
        self._reference_samples_left: list[dict[str, np.ndarray]] = []
        self._reference_samples_right: list[dict[str, np.ndarray]] = []
        self._reference_left_points: Optional[dict[str, np.ndarray]] = None
        self._reference_right_points: Optional[dict[str, np.ndarray]] = None
        self._reference_wrist_rotations: dict[str, np.ndarray] = {}
        self._reference_waist_seen = False
        self._reference_locked = False
        self._reference_start_time: Optional[float] = None
        self._reference_prompt_key: tuple[str, int] | None = None

        self._held_segment_directions: dict[str, dict[str, np.ndarray]] = {
            "left": {},
            "right": {},
        }
        # Simple IMU-sync references for hand-relative orientation.
        self._hand_imu_reference_delta_matrices: dict[str, np.ndarray] = {}

        self._previous_left_pose = np.asarray(cfg.initial_left_wrist_pose, dtype=np.float32)
        self._previous_right_pose = np.asarray(cfg.initial_right_wrist_pose, dtype=np.float32)
        self._previous_left_elbow_pose = np.asarray(cfg.initial_left_elbow_pose, dtype=np.float32)
        self._previous_right_elbow_pose = np.asarray(cfg.initial_right_elbow_pose, dtype=np.float32)
        self._last_target_update_time_monotonic = time.monotonic()

        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._thread.start()

    # ------------------------------------------------------------------ UDP
    def _receive_loop(self) -> None:
        while not self._stop.is_set():
            try:
                payload, _addr = self._sock.recvfrom(self._cfg.max_packet_bytes)
            except socket.timeout:
                continue
            except OSError:
                break
            if not payload:
                continue
            try:
                self._handle_packet(payload)
            except Exception as exc:  # noqa: BLE001 - keep the loop alive
                print(f"[bodytracking_udp] packet error: {exc}", flush=True)

    def _handle_packet(self, payload: bytes) -> None:
        try:
            message = json.loads(payload.decode("utf-8"))
        except (ValueError, UnicodeDecodeError):
            return
        if not isinstance(message, dict):
            return
        now = time.monotonic()
        self._last_packet_time_monotonic = now

        frames: dict[str, np.ndarray] = {}
        if "joint_names" in message and "joint_positions" in message:
            frames.update(self._parse_full_body_arrays(message))
        frame_payload = message.get("frames")
        if isinstance(frame_payload, dict):
            for raw_name, raw_pose in frame_payload.items():
                frame_name = BODY_FRAME_ALIASES.get(str(raw_name).lower())
                if frame_name is None:
                    continue
                pose = self._parse_pose(raw_pose)
                if pose is not None:
                    frames[frame_name] = self._transform_source_pose_to_robot_frame(pose)
        if frames:
            self._frames = frames
            self._last_body_frame_packet_time_monotonic = now
            self._body_frame_history.append((now, frames))
            if len(self._body_frame_history) > 200:
                self._body_frame_history.pop(0)
            self._update_auto_reference(now)

        hand_joints = self._parse_hand_joints(message)
        if hand_joints is not None:
            self._hand_joint_targets = hand_joints
            self._last_hand_joint_packet_time_monotonic = now

        orientation_deltas = self._parse_hand_orientation_deltas(message)
        if orientation_deltas:
            self._hand_orientation_delta_matrices.update(orientation_deltas)
            self._last_hand_orientation_packet_time_monotonic = now

    # ------------------------------------------------------------- parsing
    def _parse_pose(self, raw_pose: Any) -> Optional[np.ndarray]:
        if isinstance(raw_pose, dict) and not raw_pose.get("valid", True):
            return None
        if isinstance(raw_pose, dict):
            position = raw_pose.get("pos", raw_pose.get("position"))
            if "quat_wxyz" in raw_pose:
                quat = raw_pose["quat_wxyz"]
            elif "quat_xyzw" in raw_pose:
                q = raw_pose["quat_xyzw"]
                quat = [q[3], q[0], q[1], q[2]]
            elif "quat" in raw_pose:
                quat = raw_pose["quat"]
                if self._cfg.packet_quaternion_order == "xyzw":
                    quat = [quat[3], quat[0], quat[1], quat[2]]
            elif "orientation" in raw_pose:
                quat = raw_pose["orientation"]
                if self._cfg.packet_quaternion_order == "xyzw":
                    quat = [quat[3], quat[0], quat[1], quat[2]]
            else:
                quat = [1.0, 0.0, 0.0, 0.0]
            if position is None:
                return None
            pose = np.concatenate(
                [np.asarray(position, dtype=np.float32), np.asarray(quat, dtype=np.float32)]
            )
        else:
            pose = np.asarray(raw_pose, dtype=np.float32)
            if pose.shape[0] != 7:
                return None
            if self._cfg.packet_quaternion_order == "xyzw":
                pose = np.asarray(
                    [pose[0], pose[1], pose[2], pose[6], pose[3], pose[4], pose[5]],
                    dtype=np.float32,
                )
        if pose.shape[0] != 7 or not np.all(np.isfinite(pose)):
            return None
        return pose.astype(np.float32)

    def _parse_full_body_arrays(self, message: dict[str, Any]) -> dict[str, np.ndarray]:
        frames: dict[str, np.ndarray] = {}
        joint_names = message["joint_names"]
        joint_positions = np.asarray(message["joint_positions"], dtype=np.float32)
        joint_orientations = np.asarray(message.get("joint_orientations", []), dtype=np.float32)
        joint_valid = np.asarray(
            message.get("joint_valid", np.ones(len(joint_names))), dtype=bool
        )
        for index, raw_name in enumerate(joint_names):
            if index >= len(joint_positions) or index >= len(joint_valid) or not joint_valid[index]:
                continue
            frame_name = BODY_FRAME_ALIASES.get(str(raw_name).lower())
            if frame_name is None:
                continue
            if joint_orientations.shape[0] > index and joint_orientations.shape[1] >= 4:
                q = joint_orientations[index, :4]
                quat_wxyz = np.asarray([q[3], q[0], q[1], q[2]], dtype=np.float32)
            else:
                quat_wxyz = np.asarray([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
            pose = np.concatenate([joint_positions[index, :3], quat_wxyz]).astype(np.float32)
            frames[frame_name] = self._transform_source_pose_to_robot_frame(pose)
        return frames

    def _parse_hand_joints(self, message: dict[str, Any]) -> Optional[np.ndarray]:
        raw = message.get("hand_joints")
        if raw is None:
            raw = message.get("hand_joints_robot_order")
        if isinstance(raw, dict):
            left = raw.get("left")
            right = raw.get("right")
            if left is None or right is None:
                return None
            raw = list(left) + list(right)
        elif raw is None:
            left = message.get("left_hand_joints")
            right = message.get("right_hand_joints")
            if left is None or right is None:
                return None
            raw = list(left) + list(right)
        targets = np.asarray(raw, dtype=np.float32).reshape(-1)
        if targets.shape[0] != self._cfg.hand_joint_count or not np.all(np.isfinite(targets)):
            return None
        return targets.astype(np.float32)

    def _parse_hand_orientation_deltas(self, message: dict[str, Any]) -> dict[str, np.ndarray]:
        if not self._cfg.use_hand_imu_orientation:
            return {}
        input_frame = str(message.get("hand_orientation_source_frame", "")).strip()
        expected_frame = str(self._cfg.hand_imu_input_frame).strip()
        if input_frame and expected_frame and input_frame != expected_frame:
            if not self._printed_hand_imu_frame_mismatch:
                print(
                    "[bodytracking_udp] Ignoring hand IMU frame "
                    f"{input_frame!r}; expected {expected_frame!r}.",
                    flush=True,
                )
                self._printed_hand_imu_frame_mismatch = True
            return {}
        raw_deltas = message.get("hand_orientation_deltas")
        if raw_deltas is None:
            raw_deltas = message.get("hand_orientation_delta")
        sides: dict[str, Any] = {}
        if isinstance(raw_deltas, dict):
            for side in ("left", "right"):
                if side in raw_deltas:
                    sides[side] = raw_deltas[side]
        elif raw_deltas is not None:
            arr = np.asarray(raw_deltas, dtype=np.float32).reshape(-1)
            if arr.shape[0] == 8:
                sides["left"] = arr[:4]
                sides["right"] = arr[4:]
            else:
                return {}
        else:
            for side in ("left", "right"):
                value = message.get(f"{side}_hand_orientation_delta")
                if value is not None:
                    sides[side] = value

        out: dict[str, np.ndarray] = {}
        for side, raw_quat in sides.items():
            quat = np.asarray(raw_quat, dtype=np.float32).reshape(-1)
            if quat.shape[0] != 4 or not np.all(np.isfinite(quat)):
                continue
            if str(message.get("hand_orientation_delta_order", "wxyz")).lower() == "xyzw":
                quat = np.asarray([quat[3], quat[0], quat[1], quat[2]], dtype=np.float32)
            quat = mu.normalize_quat_wxyz(quat)
            source_delta = mu.quat_wxyz_to_matrix(quat)
            if not hasattr(self, "_hand_imu_uses_calibrated_zero"):
                self._hand_imu_uses_calibrated_zero = {"left": False, "right": False}
            self._hand_imu_uses_calibrated_zero[side] = (
                input_frame == "senseglove_zeroed_anatomical_axes"
            )
            if input_frame == "senseglove_zeroed_anatomical_axes":
                independent_anatomical_axes = (
                    self._cfg.left_hand_imu_xyz_decomposition
                    if side == "left"
                    else self._cfg.right_hand_imu_xyz_decomposition
                )
                if independent_anatomical_axes:
                    source_components = self._imu_anatomical_components(side, source_delta)
                else:
                    source_components = mu.rotation_matrix_to_rotvec(source_delta)
                robot_rotvec = self._hand_imu_local_rotvec_maps[side] @ source_components
                if independent_anatomical_axes:
                    # A spherical rotvec limit scales every target whenever one
                    # channel grows.  Clamp the fixed anatomical channels
                    # independently so one wrist DOF cannot move another target.
                    max_axis_angle = np.deg2rad(
                        max(0.0, self._cfg.hand_imu_max_angle_deg)
                    )
                    if max_axis_angle > 0.0:
                        robot_rotvec = np.clip(
                            robot_rotvec, -max_axis_angle, max_axis_angle
                        )
                robot_delta = mu.rotvec_to_rotation_matrix(robot_rotvec)
            else:
                independent_anatomical_axes = False
                # Backward compatibility for world-frame producers.
                robot_delta = self._hand_imu_rotation @ source_delta @ self._hand_imu_rotation.T
            robot_delta = self._limit_imu_rotation(
                side,
                robot_delta,
                limit_total_angle=not independent_anatomical_axes,
            )
            out[side] = robot_delta.astype(np.float32)
        return out

    def _imu_anatomical_components(
        self, side: str, source_delta: np.ndarray
    ) -> np.ndarray:
        """Integrate either hand's body-local anatomical XYZ increments."""
        previous_sources = getattr(
            self, "_hand_imu_last_source_delta_matrices", {}
        )
        integrated = getattr(self, "_hand_imu_integrated_components", {})
        previous = previous_sources.get(side)
        if previous is None or side not in integrated:
            components = mu.rotation_matrix_to_xyz_angles(source_delta)
        else:
            local_increment = mu.rotation_matrix_to_rotvec(previous.T @ source_delta)
            components = integrated[side] + local_increment
        previous_sources[side] = source_delta.copy()
        integrated[side] = np.asarray(components, dtype=np.float32)
        self._hand_imu_last_source_delta_matrices = previous_sources
        self._hand_imu_integrated_components = integrated
        return integrated[side].copy()

    def _limit_imu_rotation(
        self,
        side: str,
        rotation: np.ndarray,
        *,
        limit_total_angle: bool = True,
    ) -> np.ndarray:
        rotvec = mu.rotation_matrix_to_rotvec(rotation)
        angle = float(np.linalg.norm(rotvec))
        max_angle = np.deg2rad(max(0.0, self._cfg.hand_imu_max_angle_deg))
        if limit_total_angle and max_angle > 0.0 and angle > max_angle:
            rotvec *= max_angle / angle
            rotation = mu.rotvec_to_rotation_matrix(rotvec)

        previous = self._hand_orientation_delta_matrices.get(side)
        max_step = np.deg2rad(max(0.0, self._cfg.hand_imu_max_step_deg))
        if previous is not None and max_step > 0.0:
            relative = previous.T @ rotation
            step_rotvec = mu.rotation_matrix_to_rotvec(relative)
            step_angle = float(np.linalg.norm(step_rotvec))
            if step_angle > max_step:
                rotation = previous @ mu.rotvec_to_rotation_matrix(
                    step_rotvec * (max_step / step_angle)
                )
        return rotation.astype(np.float32)

    # ------------------------------------------------------------ transform
    def _transform_source_pose_to_robot_frame(self, pose: np.ndarray) -> np.ndarray:
        position = self._rotation @ pose[:3]
        rotation = self._rotation @ mu.quat_wxyz_to_matrix(pose[3:]) @ self._rotation.T
        quat = mu.matrix_to_quat_wxyz(rotation)
        return np.concatenate([position, quat]).astype(np.float32)

    def _has_fresh_packet(self) -> bool:
        if self._last_body_frame_packet_time_monotonic is None:
            return False
        age = time.monotonic() - self._last_body_frame_packet_time_monotonic
        if age > self._cfg.max_stale_time_s:
            if not self._printed_stale:
                print(
                    f"[bodytracking_udp] Packet stream stale for {age:.2f}s; holding target.",
                    flush=True,
                )
                self._printed_stale = True
            return False
        return True

    def _current_frame_matrix(self, frame_name: str) -> Optional[np.ndarray]:
        pose = self._frames.get(frame_name)
        if pose is None:
            return None
        matrix = mu.pose_array_to_matrix(pose)
        if self._cfg.auto_start_reference_require_waist and "waist" in self._frames:
            waist_matrix = mu.pose_array_to_matrix(self._frames["waist"])
            matrix = mu.invert_pose_matrix(waist_matrix) @ matrix
        return matrix

    def _current_wrist_matrix(self, side: str):
        frame_name = f"{side}_wrist"
        matrix = self._current_frame_matrix(frame_name)
        if matrix is None:
            frame_name = f"{side}_hand"
            matrix = self._current_frame_matrix(frame_name)
        return matrix, frame_name

    def _current_arm_points(self, side: str) -> Optional[dict[str, np.ndarray]]:
        shoulder = self._current_frame_matrix(f"{side}_shoulder")
        elbow = self._current_frame_matrix(f"{side}_elbow")
        wrist, _ = self._current_wrist_matrix(side)
        if shoulder is None or elbow is None or wrist is None:
            return None
        return {
            "shoulder": shoulder[:3, 3].copy(),
            "elbow": elbow[:3, 3].copy(),
            "wrist": wrist[:3, 3].copy(),
        }

    def _predicted_arm_points(self, side: str) -> Optional[dict[str, np.ndarray]]:
        current = self._current_arm_points(side)
        if current is None or self._cfg.arm_vector_prediction_horizon_s <= 0.0:
            return current
        if len(self._body_frame_history) < 2:
            return current
        current_time_s = self._body_frame_history[-1][0]
        candidates = [
            s for s in self._body_frame_history if 0.008 <= current_time_s - s[0] <= 0.10
        ]
        if not candidates:
            return current
        previous_time_s, previous_frames = min(
            candidates,
            key=lambda s: abs((current_time_s - s[0]) - self._cfg.arm_vector_prediction_lookback_s),
        )
        sample_dt = current_time_s - previous_time_s
        previous_points = self._arm_points_from_frames(previous_frames, side)
        if previous_points is None or sample_dt <= 1.0e-6:
            return current
        predicted = {}
        for key in ARM_POINT_KEYS:
            velocity = (current[key] - previous_points[key]) / sample_dt
            vn = float(np.linalg.norm(velocity))
            if self._cfg.arm_vector_prediction_max_velocity_m_s <= 0.0:
                velocity = np.zeros(3, dtype=np.float32)
            elif vn > self._cfg.arm_vector_prediction_max_velocity_m_s:
                velocity *= self._cfg.arm_vector_prediction_max_velocity_m_s / vn
            displacement = velocity * self._cfg.arm_vector_prediction_horizon_s
            dn = float(np.linalg.norm(displacement))
            if self._cfg.arm_vector_prediction_max_displacement_m <= 0.0:
                displacement = np.zeros(3, dtype=np.float32)
            elif dn > self._cfg.arm_vector_prediction_max_displacement_m:
                displacement *= self._cfg.arm_vector_prediction_max_displacement_m / dn
            predicted[key] = (current[key] + displacement).astype(np.float32)
        return predicted

    def _arm_points_from_frames(
        self, frames: dict[str, np.ndarray], side: str
    ) -> Optional[dict[str, np.ndarray]]:
        waist_inverse = None
        if self._cfg.auto_start_reference_require_waist and "waist" in frames:
            waist_inverse = mu.invert_pose_matrix(mu.pose_array_to_matrix(frames["waist"]))
        points: dict[str, np.ndarray] = {}
        for key in ARM_POINT_KEYS:
            pose = frames.get(f"{side}_{key}")
            if pose is None and key == "wrist":
                pose = frames.get(f"{side}_hand")
            if pose is None:
                return None
            matrix = mu.pose_array_to_matrix(pose)
            if waist_inverse is not None:
                matrix = waist_inverse @ matrix
            points[key] = matrix[:3, 3].copy()
        return points

    # ----------------------------------------------------------- calibration
    def _print_reference_prompt(self, phase: str, remaining_s: float, message: str) -> None:
        remaining = max(0, int(np.ceil(remaining_s)))
        key = (phase, remaining)
        if key == self._reference_prompt_key:
            return
        self._reference_prompt_key = key
        print(f"[PICO 标定] {message}", flush=True)

    def _update_auto_reference(self, now: float) -> None:
        if self._reference_locked:
            return
        active_arm_side = getattr(self, "_active_arm_side", None)
        active_sides = (
            (active_arm_side,)
            if active_arm_side in ("left", "right")
            else ("left", "right")
        )
        points = {side: self._current_arm_points(side) for side in active_sides}
        if any(value is None for value in points.values()):
            return
        if self._reference_start_time is None:
            self._reference_start_time = now
            self._reference_prompt_key = None
            side_label = "左臂" if active_sides == ("left",) else "右臂" if active_sides == ("right",) else "双臂"
            print(
                "\n============================================================\n"
                " PICO 手臂参考姿态标定\n"
                f" 动作：站直，{side_label}在身体侧面自然下垂，肘部自然伸直，手腕放松。\n"
                " 要求：保持肩、肘、手腕不动；此阶段机械臂不会运动。\n"
                "============================================================",
                flush=True,
            )
        elapsed = now - self._reference_start_time
        prepare_s = max(0.0, float(self._cfg.auto_start_reference_prepare_s))
        if elapsed < prepare_s:
            self._print_reference_prompt(
                "prepare",
                prepare_s - elapsed,
                f"请摆好自然下垂姿态，{int(np.ceil(prepare_s - elapsed))} 秒后进入稳定缓冲。",
            )
            return

        phase_elapsed = elapsed - prepare_s
        sample_start_s = max(0.0, float(self._cfg.auto_start_reference_sample_start_s))
        calibration_s = max(sample_start_s, float(self._cfg.auto_start_reference_delay_s))
        if phase_elapsed < sample_start_s:
            self._print_reference_prompt(
                "settle",
                sample_start_s - phase_elapsed,
                f"姿态准备完成，请保持不动；{int(np.ceil(sample_start_s - phase_elapsed))} 秒后开始采集。",
            )
            return

        if phase_elapsed >= sample_start_s:
            if "left" in points:
                self._reference_samples_left.append(points["left"])
            if "right" in points:
                self._reference_samples_right.append(points["right"])
            self._print_reference_prompt(
                "sample",
                calibration_s - phase_elapsed,
                f"正在采集参考姿态，请保持不动，还剩 {max(0, int(np.ceil(calibration_s - phase_elapsed)))} 秒。",
            )
        sample_lists = [
            self._reference_samples_left if side == "left" else self._reference_samples_right
            for side in active_sides
        ]
        if any(len(samples) < self._cfg.auto_start_reference_min_samples for samples in sample_lists):
            return
        max_std = max(mu_arm_points_max_std(samples) for samples in sample_lists)
        if phase_elapsed < calibration_s:
            return
        if max_std > self._cfg.auto_start_reference_max_position_std_m:
            self._reference_samples_left.clear()
            self._reference_samples_right.clear()
            self._reference_start_time = now
            self._reference_prompt_key = None
            print(
                f"[PICO 标定] 本次姿态晃动过大（最大标准差 {max_std:.3f} m），"
                "未保存参考零点；请重新摆好自然下垂姿态，倒计时将重新开始。",
                flush=True,
            )
            return
        if self._reference_samples_left:
            self._reference_left_points = mu.average_arm_points(self._reference_samples_left)
        if self._reference_samples_right:
            self._reference_right_points = mu.average_arm_points(self._reference_samples_right)
        for side in active_sides:
            wrist_matrix, _ = self._current_wrist_matrix(side)
            if wrist_matrix is not None:
                self._reference_wrist_rotations[side] = wrist_matrix[:3, :3].copy()
        self._reference_locked = True
        print(
            "[PICO 标定] 完成：自然下垂参考零点已锁定。机械臂仍未运动；"
            "请继续托稳机械臂，确认安全后按 e 开始遥操作。",
            flush=True,
        )

    # --------------------------------------------------------------- retarget
    def _source_side_for_target(self, target_side: str) -> str:
        if self._cfg.swap_left_right_targets:
            return "right" if target_side == "left" else "left"
        return target_side

    def _robot_arm_reference(self, target_side: str):
        if target_side == "left":
            return (
                self._robot_left_shoulder,
                self._robot_left_elbow,
                self._initial_left_pose_matrix[:3, 3],
                self._robot_left_reference_rotation,
            )
        return (
            self._robot_right_shoulder,
            self._robot_right_elbow,
            self._initial_right_pose_matrix[:3, 3],
            self._robot_right_reference_rotation,
        )

    def _robot_arm_segment_lengths(self, target_side: str):
        if target_side == "left":
            return (
                float(np.linalg.norm(self._robot_left_elbow - self._robot_left_shoulder)),
                float(np.linalg.norm(self._initial_left_pose_matrix[:3, 3] - self._robot_left_elbow)),
            )
        return (
            float(np.linalg.norm(self._robot_right_elbow - self._robot_right_shoulder)),
            float(np.linalg.norm(self._initial_right_pose_matrix[:3, 3] - self._robot_right_elbow)),
        )

    def _apply_segment_direction_deadband(
        self, target_side: str, segment_name: str, vector: np.ndarray, deadband_deg: float
    ) -> np.ndarray:
        direction = mu.normalize_vector(vector)
        if direction is None:
            return vector
        previous = self._held_segment_directions[target_side].get(segment_name)
        if previous is not None and deadband_deg > 0.0:
            cosine = float(np.clip(np.dot(previous, direction), -1.0, 1.0))
            if abs(float(np.degrees(np.arccos(cosine)))) < deadband_deg:
                return previous
        self._held_segment_directions[target_side][segment_name] = direction.copy()
        return direction

    def _retarget_arm_segment_direction_positions(
        self, current_points: dict[str, np.ndarray], target_side: str
    ):
        robot_shoulder, robot_elbow, robot_wrist, _ = self._robot_arm_reference(
            target_side
        )
        upper_length, forearm_length = self._robot_arm_segment_lengths(target_side)

        signed = mu.arm_points_relative_to_shoulder(current_points, self._signs)
        current_upper = signed["elbow"] - signed["shoulder"]
        current_forearm = signed["wrist"] - signed["elbow"]

        alignment_rotation = np.eye(3, dtype=np.float32)
        if self._cfg.arm_vector_position_mode == "segment_direction_relative":
            reference_points = (
                self._reference_left_points
                if target_side == "left"
                else self._reference_right_points
            )
            if reference_points is not None:
                signed_reference = mu.arm_points_relative_to_shoulder(reference_points, self._signs)
                # A natural-down arm is nearly straight, so shoulder/elbow/wrist
                # cannot determine rotation about the arm. Align only the upper
                # arm and preserve the fixed PICO-to-robot H1/H2 axes.
                reference_upper = signed_reference["elbow"] - signed_reference["shoulder"]
                robot_upper = robot_elbow - robot_shoulder
                reference_alignment = mu.minimal_vector_alignment_rotation(
                    reference_upper, robot_upper
                )
                if reference_alignment is not None:
                    alignment_rotation = reference_alignment

        mapped_upper = alignment_rotation @ current_upper
        mapped_forearm = alignment_rotation @ current_forearm
        mapped_upper = self._apply_segment_direction_deadband(
            target_side, "upper_arm", mapped_upper, self._cfg.upper_arm_angular_deadband_deg
        )
        mapped_forearm = self._apply_segment_direction_deadband(
            target_side, "forearm", mapped_forearm, self._cfg.forearm_angular_deadband_deg
        )
        upper_vector = mu.scale_vector_to_length(
            mapped_upper, upper_length, robot_elbow - robot_shoulder
        )
        forearm_vector = mu.scale_vector_to_length(
            mapped_forearm, forearm_length, robot_wrist - robot_elbow
        )
        if upper_vector is None or forearm_vector is None:
            return None
        target_elbow_position = robot_shoulder + upper_vector
        target_wrist_position = target_elbow_position + forearm_vector
        return target_elbow_position.astype(np.float32), target_wrist_position.astype(np.float32)

    def _hand_imu_delta_matrix_for_target(self, target_side: str) -> Optional[np.ndarray]:
        if not self._cfg.use_hand_imu_orientation:
            return None
        last_time = getattr(self, "_last_hand_orientation_packet_time_monotonic", None)
        if last_time is None:
            return None
        age = time.monotonic() - last_time
        if age > self._cfg.max_stale_time_s:
            return None
        return self._hand_orientation_delta_matrices.get(target_side)

    def _hand_imu_relative_rotation_local(self, target_side: str) -> Optional[np.ndarray]:
        rotation_delta = self._hand_imu_delta_matrix_for_target(target_side)
        if rotation_delta is None:
            return None
        if getattr(self, "_hand_imu_uses_calibrated_zero", {}).get(target_side, False):
            # The SenseGlove bridge already removed the six-step calibration
            # neutral. PICO/robot rebasing must not define a second IMU zero.
            return rotation_delta.copy().astype(np.float32)
        imu_reference = self._hand_imu_reference_delta_matrices.get(target_side)
        if imu_reference is None:
            self._hand_imu_reference_delta_matrices[target_side] = rotation_delta.copy()
            return np.eye(3, dtype=np.float32)
        hand_delta_local = imu_reference.T @ rotation_delta
        return hand_delta_local.astype(np.float32)

    def _forearm_points_to_rotation(self, points: dict[str, np.ndarray]) -> Optional[np.ndarray]:
        forearm_axis = mu.normalize_vector(points["wrist"] - points["elbow"])
        upper_arm_axis = mu.normalize_vector(points["elbow"] - points["shoulder"])
        if forearm_axis is None:
            return None
        plane_normal = None
        if upper_arm_axis is not None:
            plane_normal = mu.normalize_vector(np.cross(upper_arm_axis, forearm_axis))
        if plane_normal is None:
            for reference in (
                np.asarray([0.0, 0.0, 1.0], dtype=np.float32),
                np.asarray([0.0, 1.0, 0.0], dtype=np.float32),
                np.asarray([1.0, 0.0, 0.0], dtype=np.float32),
            ):
                plane_normal = mu.normalize_vector(np.cross(reference, forearm_axis))
                if plane_normal is not None:
                    break
        if plane_normal is None:
            return None
        bend_axis = mu.normalize_vector(np.cross(forearm_axis, plane_normal))
        if bend_axis is None:
            return None
        return np.stack([forearm_axis, bend_axis, plane_normal], axis=1).astype(np.float32)

    def _compose_wrist_rotation(
        self, target_side: str, target_arm_points: Optional[dict[str, np.ndarray]]
    ) -> np.ndarray:
        if target_side == "left":
            initial_rotation = self._initial_left_pose_matrix[:3, :3]
        else:
            initial_rotation = self._initial_right_pose_matrix[:3, :3]
        current_arm_rotation = (
            self._forearm_points_to_rotation(target_arm_points)
            if target_arm_points is not None
            else None
        )
        relative = self._hand_imu_relative_rotation_local(target_side)
        if relative is not None:
            return (initial_rotation @ relative).astype(np.float32)
        if (
            getattr(self, "_active_arm_side", None) == target_side
            and not self._cfg.allow_pico_wrist_orientation_fallback
        ):
            # Single-arm PICO mode reserves terminal orientation for the glove.
            # advance() rejects this frame, but keep this helper deterministic
            # for diagnostics and unit tests.
            return initial_rotation.copy().astype(np.float32)
        # Legacy opt-in path: retain the tracker's wrist quaternion so
        # forearm twist is observable. The natural-down sample removes the
        # OpenXR world-reference offset without changing its axis ordering.
        if getattr(self, "_active_arm_side", None) == target_side:
            wrist_matrix, _ = self._current_wrist_matrix(
                self._source_side_for_target(target_side)
            )
            reference_wrist = self._reference_wrist_rotations.get(target_side)
            if wrist_matrix is not None and reference_wrist is not None:
                world_delta = wrist_matrix[:3, :3] @ reference_wrist.T
                return (world_delta @ initial_rotation).astype(np.float32)
        if current_arm_rotation is None:
            return initial_rotation.copy().astype(np.float32)
        # Fall back to the forearm frame inheriting the initial wrist orientation.
        reference_arm = self._forearm_points_to_rotation(
            {
                "shoulder": self._robot_left_shoulder if target_side == "left" else self._robot_right_shoulder,
                "elbow": self._robot_left_elbow if target_side == "left" else self._robot_right_elbow,
                "wrist": self._initial_left_pose_matrix[:3, 3]
                if target_side == "left"
                else self._initial_right_pose_matrix[:3, 3],
            }
        )
        if reference_arm is not None:
            return (current_arm_rotation @ reference_arm.T @ initial_rotation).astype(np.float32)
        return initial_rotation.copy().astype(np.float32)

    @staticmethod
    def _limit_pose_translation(
        previous_pose: np.ndarray, target_pose: np.ndarray, max_velocity: float, dt: float
    ) -> np.ndarray:
        """Rate-limit a pose position while preserving its target orientation."""
        previous = np.asarray(previous_pose, dtype=np.float32).reshape(7)
        target = np.asarray(target_pose, dtype=np.float32).reshape(7).copy()
        if max_velocity <= 0.0:
            return target
        if not np.isfinite(dt) or dt <= 0.0:
            target[:3] = previous[:3]
            return target
        delta = target[:3] - previous[:3]
        distance = float(np.linalg.norm(delta))
        max_distance = float(max_velocity) * dt
        if distance > max_distance and distance > 0.0:
            target[:3] = previous[:3] + delta * (max_distance / distance)
        return target

    @staticmethod
    def _limit_arm_segment_translations(
        previous_elbow_pose: np.ndarray,
        previous_wrist_pose: np.ndarray,
        target_elbow_pose: np.ndarray,
        target_wrist_pose: np.ndarray,
        shoulder_position: np.ndarray,
        max_velocity: float,
        dt: float,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Synchronously limit a two-link arm while preserving both link lengths."""
        previous_elbow = np.asarray(previous_elbow_pose, dtype=np.float32).reshape(7)
        previous_wrist = np.asarray(previous_wrist_pose, dtype=np.float32).reshape(7)
        target_elbow = np.asarray(target_elbow_pose, dtype=np.float32).reshape(7).copy()
        target_wrist = np.asarray(target_wrist_pose, dtype=np.float32).reshape(7).copy()
        shoulder = np.asarray(shoulder_position, dtype=np.float32).reshape(3)
        if max_velocity <= 0.0:
            return target_elbow, target_wrist
        if not np.isfinite(dt) or dt <= 0.0:
            target_elbow[:3] = previous_elbow[:3]
            target_wrist[:3] = previous_wrist[:3]
            return target_elbow, target_wrist

        upper_length = float(np.linalg.norm(target_elbow[:3] - shoulder))
        forearm_length = float(np.linalg.norm(target_wrist[:3] - target_elbow[:3]))
        previous_upper = mu.normalize_vector(previous_elbow[:3] - shoulder)
        target_upper = mu.normalize_vector(target_elbow[:3] - shoulder)
        previous_forearm = mu.normalize_vector(previous_wrist[:3] - previous_elbow[:3])
        target_forearm = mu.normalize_vector(target_wrist[:3] - target_elbow[:3])
        if (
            upper_length <= 0.0
            or forearm_length <= 0.0
            or previous_upper is None
            or target_upper is None
            or previous_forearm is None
            or target_forearm is None
        ):
            target_elbow[:3] = previous_elbow[:3]
            target_wrist[:3] = previous_wrist[:3]
            return target_elbow, target_wrist

        def candidate(progress: float) -> tuple[np.ndarray, np.ndarray]:
            upper_direction = mu.normalize_vector(
                (1.0 - progress) * previous_upper + progress * target_upper
            )
            forearm_direction = mu.normalize_vector(
                (1.0 - progress) * previous_forearm + progress * target_forearm
            )
            if upper_direction is None or forearm_direction is None:
                return previous_elbow[:3].copy(), previous_wrist[:3].copy()
            elbow_position = shoulder + upper_direction * upper_length
            wrist_position = elbow_position + forearm_direction * forearm_length
            return elbow_position, wrist_position

        max_distance = float(max_velocity) * dt
        full_elbow, full_wrist = candidate(1.0)
        full_step = max(
            float(np.linalg.norm(full_elbow - previous_elbow[:3])),
            float(np.linalg.norm(full_wrist - previous_wrist[:3])),
        )
        progress = 1.0
        if full_step > max_distance:
            low, high = 0.0, 1.0
            for _ in range(24):
                middle = 0.5 * (low + high)
                elbow_position, wrist_position = candidate(middle)
                step = max(
                    float(np.linalg.norm(elbow_position - previous_elbow[:3])),
                    float(np.linalg.norm(wrist_position - previous_wrist[:3])),
                )
                if step <= max_distance:
                    low = middle
                else:
                    high = middle
            progress = low
        target_elbow[:3], target_wrist[:3] = candidate(progress)
        return target_elbow, target_wrist

    def _retarget_arm_vector_pair(
        self, source_side: str, target_side: str, target_dt: float
    ):
        current_points = self._predicted_arm_points(source_side)
        if current_points is None:
            return None
        wrist_matrix, _ = self._current_wrist_matrix(source_side)
        if wrist_matrix is None:
            return None

        if target_side == "left":
            initial_wrist = self._initial_left_pose_matrix
            initial_elbow = self._initial_left_elbow_pose_matrix
        else:
            initial_wrist = self._initial_right_pose_matrix
            initial_elbow = self._initial_right_elbow_pose_matrix

        target_wrist = initial_wrist.copy()
        target_elbow = initial_elbow.copy()

        if self._cfg.arm_vector_position_mode in (
            "segment_direction_absolute",
            "segment_direction_relative",
        ):
            segment = self._retarget_arm_segment_direction_positions(current_points, target_side)
            if segment is None:
                return None
            target_elbow[:3, 3], target_wrist[:3, 3] = segment
        else:
            # wrist_delta fallback relative to the locked reference.
            reference_points = (
                self._reference_left_points
                if target_side == "left"
                else self._reference_right_points
            )
            if reference_points is None:
                return None
            current_reach = current_points["wrist"] - current_points["shoulder"]
            ref_reach = reference_points["wrist"] - reference_points["shoulder"]
            position_delta = (current_reach - ref_reach) * self._signs
            target_wrist[:3, 3] = initial_wrist[:3, 3] + position_delta * self._cfg.position_scale

        robot_shoulder = (
            self._robot_left_shoulder if target_side == "left" else self._robot_right_shoulder
        )
        target_wrist[:3, :3] = self._compose_wrist_rotation(
            target_side,
            {
                "shoulder": robot_shoulder,
                "elbow": target_elbow[:3, 3],
                "wrist": target_wrist[:3, 3],
            },
        )
        elbow_pose = mu.pose_matrix_to_array(target_elbow)
        wrist_pose = mu.pose_matrix_to_array(target_wrist)
        if target_side == "left":
            elbow_pose, wrist_pose = self._limit_arm_segment_translations(
                self._previous_left_elbow_pose,
                self._previous_left_pose,
                elbow_pose,
                wrist_pose,
                self._robot_left_shoulder,
                self._cfg.max_endpoint_translation_velocity_m_s,
                target_dt,
            )
            self._previous_left_elbow_pose = elbow_pose
            self._previous_left_pose = wrist_pose
        else:
            elbow_pose, wrist_pose = self._limit_arm_segment_translations(
                self._previous_right_elbow_pose,
                self._previous_right_pose,
                elbow_pose,
                wrist_pose,
                self._robot_right_shoulder,
                self._cfg.max_endpoint_translation_velocity_m_s,
                target_dt,
            )
            self._previous_right_elbow_pose = elbow_pose
            self._previous_right_pose = wrist_pose
        return elbow_pose, wrist_pose

    # ---------------------------------------------------------------- public
    def is_ready(self) -> bool:
        if not self._reference_locked or not self._has_fresh_packet():
            return False
        active_side = getattr(self, "_active_arm_side", None)
        if self._cfg.require_hand_imu_for_active_arm and active_side in ("left", "right"):
            return self._hand_imu_delta_matrix_for_target(active_side) is not None
        return True

    def rebase_robot_reference(self, poses: dict[str, np.ndarray]) -> None:
        """Use measured arm FK as this session's zero-motion robot reference."""
        required = (
            "left_shoulder", "left_elbow", "left_wrist",
            "right_shoulder", "right_elbow", "right_wrist",
        )
        if any(key not in poses for key in required):
            raise ValueError("startup FK is missing an elbow or wrist pose")
        self._initial_left_elbow_pose_matrix = mu.pose_array_to_matrix(poses["left_elbow"])
        self._initial_left_pose_matrix = mu.pose_array_to_matrix(poses["left_wrist"])
        self._initial_right_elbow_pose_matrix = mu.pose_array_to_matrix(poses["right_elbow"])
        self._initial_right_pose_matrix = mu.pose_array_to_matrix(poses["right_wrist"])
        self._robot_left_shoulder = poses["left_shoulder"][:3].copy()
        self._robot_right_shoulder = poses["right_shoulder"][:3].copy()
        self._robot_left_elbow = poses["left_elbow"][:3].copy()
        self._robot_right_elbow = poses["right_elbow"][:3].copy()
        self._previous_left_elbow_pose = poses["left_elbow"].copy()
        self._previous_left_pose = poses["left_wrist"].copy()
        self._previous_right_elbow_pose = poses["right_elbow"].copy()
        self._previous_right_pose = poses["right_wrist"].copy()
        for side in ("left", "right"):
            shoulder = self._robot_left_shoulder if side == "left" else self._robot_right_shoulder
            elbow = poses[f"{side}_elbow"][:3]
            wrist = poses[f"{side}_wrist"][:3]
            rotation = mu.arm_points_to_rotation(
                {"shoulder": shoulder, "elbow": elbow, "wrist": wrist}
            )
            if rotation is not None:
                if side == "left":
                    self._robot_left_reference_rotation = rotation
                else:
                    self._robot_right_reference_rotation = rotation
        self._held_segment_directions = {"left": {}, "right": {}}
        self._hand_imu_reference_delta_matrices.clear()
        self._last_target_update_time_monotonic = time.monotonic()

    def hand_joints(self) -> Optional[np.ndarray]:
        if self._last_hand_joint_packet_time_monotonic is None:
            return None
        if time.monotonic() - self._last_hand_joint_packet_time_monotonic > self._cfg.max_stale_time_s:
            return None
        return self._hand_joint_targets.copy() if self._hand_joint_targets is not None else None

    def hand_orientation_delta(self, side: str) -> Optional[np.ndarray]:
        """Return a fresh calibrated glove IMU delta, or ``None`` when stale."""
        if side not in ("left", "right"):
            raise ValueError("side must be 'left' or 'right'")
        rotation = self._hand_imu_delta_matrix_for_target(side)
        return rotation.copy() if rotation is not None else None

    def hand_orientation_components(self, side: str) -> Optional[np.ndarray]:
        """Return fresh pre-limit anatomical XYZ channels for diagnostics."""
        if self._hand_imu_delta_matrix_for_target(side) is None:
            return None
        components = getattr(self, "_hand_imu_integrated_components", {}).get(side)
        return components.copy() if components is not None else None

    def advance(self) -> Optional[dict[str, np.ndarray]]:
        """Return left/right elbow+wrist target poses or None if not ready."""
        now = time.monotonic()
        previous_time = getattr(self, "_last_target_update_time_monotonic", now)
        self._last_target_update_time_monotonic = now
        target_dt = max(0.0, now - previous_time)
        if not self._reference_locked or not self._has_fresh_packet():
            return None
        active_arm_side = getattr(self, "_active_arm_side", None)
        if (
            self._cfg.require_hand_imu_for_active_arm
            and active_arm_side in ("left", "right")
            and self._hand_imu_delta_matrix_for_target(active_arm_side) is None
        ):
            return None
        if active_arm_side == "left":
            left = self._retarget_arm_vector_pair(
                self._source_side_for_target("left"), "left", target_dt
            )
            right = (
                self._previous_right_elbow_pose.copy(),
                self._previous_right_pose.copy(),
            )
        elif active_arm_side == "right":
            left = (
                self._previous_left_elbow_pose.copy(),
                self._previous_left_pose.copy(),
            )
            right = self._retarget_arm_vector_pair(
                self._source_side_for_target("right"), "right", target_dt
            )
        else:
            left = self._retarget_arm_vector_pair(
                self._source_side_for_target("left"), "left", target_dt
            )
            right = self._retarget_arm_vector_pair(
                self._source_side_for_target("right"), "right", target_dt
            )
        if left is None or right is None:
            return None
        return {
            "left_elbow": left[0],
            "left_wrist": left[1],
            "right_elbow": right[0],
            "right_wrist": right[1],
        }

    def close(self) -> None:
        self._stop.set()
        try:
            self._sock.close()
        except OSError:
            pass


def mu_arm_points_max_std(samples: list[dict[str, np.ndarray]]) -> float:
    if len(samples) < 2:
        return 0.0
    stacked = np.stack(
        [
            np.concatenate([sample[key] for key in ARM_POINT_KEYS])
            for sample in samples
        ],
        axis=0,
    )
    return float(np.max(np.std(stacked, axis=0)))


__all__ = ["BodyDevice", "BODY_FRAME_ALIASES"]
