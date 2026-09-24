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
from contextlib import nullcontext
from pathlib import Path
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
        values = [cfg.elbow_absolute_start_delta_deg, cfg.elbow_absolute_full_delta_deg,
                  cfg.bend_plane_observability_start_delta_deg,
                  cfg.bend_plane_observability_full_delta_deg,
                  cfg.arm_reference_enable_prepare_s, cfg.arm_reference_enable_max_flexion_delta_deg]
        if (cfg.elbow_angle_mapping not in (
                "direct_absolute", "responsive_absolute", "smooth_absolute", "relative")
                or not np.all(np.isfinite(values)) or min(values) < 0
                or values[1] <= values[0] or values[3] <= values[2]):
            raise ValueError("invalid elbow mapping or arming-reference parameters")
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
        self._reference_sample_times: list[float] = []
        self._reference_left_points: Optional[dict[str, np.ndarray]] = None
        self._reference_right_points: Optional[dict[str, np.ndarray]] = None
        self._reference_wrist_rotations: dict[str, np.ndarray] = {}
        self._reference_waist_seen = False
        self._reference_locked = False
        self._reference_lock = threading.RLock()
        self._reference_diagnostic_path = (
            Path(__file__).resolve().parents[3] / "log"
            / f"pico_reference_{time.time_ns()}.jsonl"
        )
        self._reference_start_time: Optional[float] = None
        self._reference_prompt_key: tuple[str, int] | None = None

        self._held_segment_directions: dict[str, dict[str, np.ndarray]] = {
            "left": {},
            "right": {},
        }
        self._filtered_segment_directions: dict[str, dict[str, np.ndarray]] = {
            "left": {},
            "right": {},
        }
        self._filtered_segment_timestamps: dict[str, dict[str, float]] = {
            "left": {},
            "right": {},
        }
        self._arm_retarget_diagnostics: dict[str, dict[str, Any]] = {
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
            with self._reference_lock:
                previous_time = self._last_body_frame_packet_time_monotonic
                if (not self._reference_locked and previous_time is not None
                        and now - previous_time > self._cfg.max_stale_time_s):
                    self._reference_samples_left.clear()
                    self._reference_samples_right.clear()
                    self._reference_sample_times.clear()
                    self._reference_start_time = None
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
        return self._arm_points_from_frames(self._frames, side)

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
            prediction_gain = self._prediction_velocity_gain(vn)
            velocity *= prediction_gain
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

    def _prediction_velocity_gain(self, speed_m_s: float) -> float:
        """Smoothly suppress prediction caused by low-speed tracker jitter."""
        start = max(0.0, float(self._cfg.arm_vector_prediction_start_velocity_m_s))
        full = max(start, float(self._cfg.arm_vector_prediction_full_velocity_m_s))
        if speed_m_s <= start:
            return 0.0
        if full <= start or speed_m_s >= full:
            return 1.0
        ratio = (float(speed_m_s) - start) / (full - start)
        return ratio * ratio * (3.0 - 2.0 * ratio)

    def _arm_points_from_frames(
        self, frames: dict[str, np.ndarray], side: str
    ) -> Optional[dict[str, np.ndarray]]:
        waist_inverse = None
        if self._cfg.auto_start_reference_require_waist:
            if "waist" not in frames or not np.all(np.isfinite(frames["waist"])):
                return None
            waist_inverse = mu.invert_pose_matrix(mu.pose_array_to_matrix(frames["waist"]))
        points: dict[str, np.ndarray] = {}
        for key in ARM_POINT_KEYS:
            pose = frames.get(f"{side}_{key}")
            if pose is None and key == "wrist":
                pose = frames.get(f"{side}_hand")
            if pose is None or not np.all(np.isfinite(pose)):
                return None
            matrix = mu.pose_array_to_matrix(pose)
            if waist_inverse is not None:
                matrix = waist_inverse @ matrix
            points[key] = matrix[:3, 3].copy()
        for start, end in (("shoulder", "elbow"), ("elbow", "wrist")):
            if np.linalg.norm(points[end] - points[start]) < 1.0e-6:
                return None
        return points

    # ----------------------------------------------------------- calibration
    def _print_reference_prompt(self, phase: str, remaining_s: float, message: str) -> None:
        remaining = max(0, int(np.ceil(remaining_s)))
        key = (phase, remaining)
        if key == self._reference_prompt_key:
            return
        self._reference_prompt_key = key
        print(f"[PICO 标定] {message}", flush=True)

    def _reference_pose_rejection(self, points_by_side) -> Optional[str]:
        """Reject stable poses that keep the hands outside PICO's useful view."""
        minimum_raise = float(getattr(self._cfg, "reference_min_upper_raise_deg", 0.0))
        minimum_bend = float(getattr(self._cfg, "reference_min_elbow_flexion_deg", 0.0))
        maximum_bend = float(getattr(self._cfg, "reference_max_elbow_flexion_deg", 180.0))
        down = np.asarray([0.0, 0.0, -1.0])
        for side, points in points_by_side.items():
            upper = mu.normalize_vector(points["elbow"] - points["shoulder"])
            if upper is None:
                return f"{side} 上臂数据无效"
            raise_deg = float(np.degrees(np.arccos(np.clip(np.dot(upper, down), -1.0, 1.0))))
            bend_deg = float(np.degrees(self._points_flexion(points)))
            if raise_deg < minimum_raise:
                return f"{side} 手臂仍接近身体侧面（抬臂 {raise_deg:.0f}°，需至少 {minimum_raise:.0f}°）"
            if not minimum_bend <= bend_deg <= maximum_bend:
                return (f"{side} 肘部弯曲 {bend_deg:.0f}°，需保持在 "
                        f"{minimum_bend:.0f}°–{maximum_bend:.0f}°")
        return None

    def _update_auto_reference(self, now: float) -> None:
        if self._reference_locked:
            return
        active_arm_side = getattr(self, "_active_arm_side", None)
        active_sides = (
            (active_arm_side,)
            if active_arm_side in ("left", "right")
            else ("left", "right")
        )
        points = {
            side: self._current_arm_points(self._source_side_for_target(side))
            for side in active_sides
        }
        if any(value is None for value in points.values()):
            self._reference_samples_left.clear()
            self._reference_samples_right.clear()
            self._reference_start_time = None
            self._reference_sample_times = []
            return
        pose_rejection = self._reference_pose_rejection(points)
        if pose_rejection:
            self._reference_samples_left.clear()
            self._reference_samples_right.clear()
            self._reference_sample_times.clear()
            self._reference_start_time = None
            self._print_reference_prompt(
                "pose", 0,
                f"{pose_rejection}；请把前臂抬到胸前、肘部自然弯曲，让双手持续处于头显视野内。",
            )
            return
        if self._reference_start_time is None:
            self._reference_start_time = now
            self._reference_prompt_key = None
            side_label = "左臂" if active_sides == ("left",) else "右臂" if active_sides == ("right",) else "双臂"
            print(
                "\n============================================================\n"
                " PICO 手臂参考姿态标定\n"
                f" 动作：站直，将{side_label}抬到胸前，肘部自然弯曲，手腕放松，双手保持在头显视野内。\n"
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
                f"请保持胸前抬臂准备姿态，{int(np.ceil(prepare_s - elapsed))} 秒后进入稳定缓冲。",
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
            if not hasattr(self, "_reference_sample_times"):
                self._reference_sample_times = []
            self._reference_sample_times.append(now)
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
            self._reference_sample_times.clear()
            self._reference_start_time = now
            self._reference_prompt_key = None
            print(
                f"[PICO 标定] 本次姿态晃动过大（最大标准差 {max_std:.3f} m），"
                "未保存参考零点；请重新摆好胸前抬臂姿态，倒计时将重新开始。",
                flush=True,
            )
            return
        if self._reference_samples_left:
            self._reference_left_points = mu.average_arm_points(self._reference_samples_left)
        if self._reference_samples_right:
            self._reference_right_points = mu.average_arm_points(self._reference_samples_right)
        for side in active_sides:
            reference = self._reference_left_points if side == "left" else self._reference_right_points
            samples = self._reference_samples_left if side == "left" else self._reference_samples_right
            self._record_reference_diagnostic(
                "calibrated", side, reference, reference, len(samples), max_std,
                self._reference_sample_times[0], now,
            )
            wrist_matrix, _ = self._current_wrist_matrix(self._source_side_for_target(side))
            if wrist_matrix is not None:
                self._reference_wrist_rotations[side] = wrist_matrix[:3, :3].copy()
        self._reference_locked = True
        print(
            "[PICO 标定] 完成：胸前抬臂参考已锁定。请保持双手自然张开；"
            "系统将在机器人零位与人手准备姿态均验证后自动开始跟随。",
            flush=True,
        )

    def restart_arm_reference(self) -> None:
        """Discard only PICO calibration; caller must prohibit active motors/following."""
        with self._reference_lock:
            self._reference_locked = False
            self._reference_start_time = None
            self._reference_prompt_key = None
            self._reference_samples_left.clear()
            self._reference_samples_right.clear()
            self._reference_sample_times.clear()
            self._reference_left_points = None
            self._reference_right_points = None
            self._reference_wrist_rotations.clear()
            self._body_frame_history.clear()
            self._frames = {}
            self._last_body_frame_packet_time_monotonic = None
            for side in ("left", "right"):
                self._held_segment_directions[side].clear()
                self._filtered_segment_directions[side].clear()
                self._filtered_segment_timestamps[side].clear()

    def _record_reference_diagnostic(
        self, event, side, reference, candidate, count, std, start, end,
        deviations=None,
    ) -> None:
        path = getattr(self, "_reference_diagnostic_path", None)
        if path is None:
            return

        def describe(points):
            if points is None:
                return None
            vectors = [points["elbow"] - points["shoulder"], points["wrist"] - points["elbow"]]
            lengths = [float(np.linalg.norm(v)) for v in vectors]
            directions = [v / length for v, length in zip(vectors, lengths)]
            return {
                "points_m": {k: v.tolist() for k, v in points.items()},
                "lengths_m": lengths,
                "directions": [v.tolist() for v in directions],
                "flexion_deg": float(np.degrees(np.arccos(np.clip(
                    np.dot(*directions), -1.0, 1.0)))),
            }

        record = {
            "event": event, "wall_time_s": time.time(), "target_side": side,
            "source_side": self._source_side_for_target(side),
            "frame": "waist_local" if self._cfg.auto_start_reference_require_waist else "robot_world",
            "sample_start_monotonic_s": start, "sample_end_monotonic_s": end,
            "sample_count": count, "max_position_std_m": std,
            "reference": describe(reference), "candidate": describe(candidate),
            "deviation_deg": deviations,
        }
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            with path.open("a", encoding="utf-8") as stream:
                stream.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + "\n")
            std_text = "unavailable" if std is None else f"{std:.3f}m"
            print(f"[PICO 参考] {event}: {side}, samples={count}, std={std_text}; log={path}", flush=True)
        except (OSError, ValueError) as exc:
            print(f"[PICO 参考] diagnostic log failed: {exc}", flush=True)

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
        direction = mu.normalize_vector(np.asarray(vector, dtype=np.float32))
        if direction is None:
            return vector
        previous = self._held_segment_directions[target_side].get(segment_name)
        if previous is not None and deadband_deg > 0.0:
            cosine = float(np.clip(np.dot(previous, direction), -1.0, 1.0))
            if abs(float(np.degrees(np.arccos(cosine)))) < deadband_deg:
                return previous
        self._held_segment_directions[target_side][segment_name] = direction.copy()
        return direction

    def _filter_segment_direction(
        self, target_side: str, segment_name: str, vector: np.ndarray
    ) -> np.ndarray:
        """Smooth PICO segment direction once per source frame with adaptive lag."""
        direction = mu.normalize_vector(np.asarray(vector, dtype=np.float32))
        if direction is None:
            return vector
        frame_time = self._last_body_frame_packet_time_monotonic
        previous = self._filtered_segment_directions[target_side].get(segment_name)
        previous_time = self._filtered_segment_timestamps[target_side].get(segment_name)
        if previous is None or previous_time is None or frame_time is None:
            filtered = direction
        else:
            dt = float(frame_time - previous_time)
            if dt <= 1.0e-6:
                return previous
            cosine = float(np.clip(np.dot(previous, direction), -1.0, 1.0))
            angular_speed = float(np.arccos(cosine)) / dt
            min_cutoff = max(0.0, self._cfg.arm_vector_filter_min_cutoff_hz)
            max_cutoff = max(min_cutoff, self._cfg.arm_vector_filter_max_cutoff_hz)
            cutoff = min(
                max_cutoff,
                min_cutoff
                + max(0.0, self._cfg.arm_vector_filter_speed_coefficient) * angular_speed,
            )
            alpha = 1.0 - np.exp(-2.0 * np.pi * cutoff * dt)
            filtered = mu.normalize_vector((1.0 - alpha) * previous + alpha * direction)
            if filtered is None:
                filtered = previous
        self._filtered_segment_directions[target_side][segment_name] = filtered.copy()
        if frame_time is not None:
            self._filtered_segment_timestamps[target_side][segment_name] = frame_time
        return filtered

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
        source_upper_direction = mu.normalize_vector(current_upper)
        source_forearm_direction = mu.normalize_vector(current_forearm)

        alignment_rotation = np.eye(3, dtype=np.float32)
        if self._cfg.arm_vector_position_mode == "segment_direction_relative":
            reference_points = (
                self._reference_left_points
                if target_side == "left"
                else self._reference_right_points
            )
            if reference_points is not None:
                signed_reference = mu.arm_points_relative_to_shoulder(reference_points, self._signs)
                reference_upper = signed_reference["elbow"] - signed_reference["shoulder"]
                robot_upper = robot_elbow - robot_shoulder
                reference_alignment = mu.minimal_vector_alignment_rotation(
                    reference_upper, robot_upper
                )
                if reference_alignment is not None:
                    alignment_rotation = reference_alignment
        mapped_upper = alignment_rotation @ current_upper
        mapped_forearm = alignment_rotation @ current_forearm
        if self._cfg.arm_vector_position_mode == "segment_direction_relative" and reference_points is not None:
            mapped_forearm = self._neutral_relative_forearm(
                mapped_upper, mapped_forearm, self.arm_reference_flexion_rad(target_side),
                robot_elbow - robot_shoulder, robot_wrist - robot_elbow,
                mode=self._cfg.elbow_angle_mapping,
                start_deg=self._cfg.elbow_absolute_start_delta_deg,
                full_deg=self._cfg.elbow_absolute_full_delta_deg,
                plane_start_deg=self._cfg.bend_plane_observability_start_delta_deg,
                plane_full_deg=self._cfg.bend_plane_observability_full_delta_deg,
            )
        mapped_before_filter_upper = mu.normalize_vector(mapped_upper)
        mapped_before_filter_forearm = mu.normalize_vector(mapped_forearm)
        mapped_before_filter_deg = float(np.degrees(np.arccos(np.clip(
            np.dot(mapped_before_filter_upper, mapped_before_filter_forearm),
            -1.0, 1.0,
        ))))
        mapped_upper = self._filter_segment_direction(
            target_side, "upper_arm", mapped_upper
        )
        mapped_forearm = self._filter_segment_direction(
            target_side, "forearm", mapped_forearm
        )
        mapped_upper = self._apply_segment_direction_deadband(
            target_side, "upper_arm", mapped_upper, self._cfg.upper_arm_angular_deadband_deg
        )
        mapped_forearm = self._apply_segment_direction_deadband(
            target_side, "forearm", mapped_forearm, self._cfg.forearm_angular_deadband_deg
        )
        filtered_before_plane_upper = mu.normalize_vector(mapped_upper)
        filtered_before_plane_forearm = mu.normalize_vector(mapped_forearm)
        filtered_before_plane_gate_deg = float(np.degrees(np.arccos(np.clip(
            np.dot(filtered_before_plane_upper, filtered_before_plane_forearm),
            -1.0, 1.0,
        ))))
        if self._cfg.arm_vector_position_mode == "segment_direction_relative":
            # Filtering the two segments independently can reduce the actual
            # bend back into the near-straight region after the raw PICO bend
            # has already exposed its noisy plane. Reapply the observability
            # gate to the directions that will really become the target.
            mapped_forearm = self._stabilize_forearm_plane(
                mapped_upper,
                mapped_forearm,
                robot_elbow - robot_shoulder,
                robot_wrist - robot_elbow,
                plane_start_deg=self._cfg.bend_plane_observability_start_delta_deg,
                plane_full_deg=self._cfg.bend_plane_observability_full_delta_deg,
            )
        desired_after_plane_upper = mu.normalize_vector(mapped_upper)
        desired_after_plane_forearm = mu.normalize_vector(mapped_forearm)
        desired_after_plane_gate_deg = float(np.degrees(np.arccos(np.clip(
            np.dot(desired_after_plane_upper, desired_after_plane_forearm),
            -1.0, 1.0,
        ))))
        stage_diagnostics = getattr(self, "_arm_retarget_diagnostics", None)
        if stage_diagnostics is not None:
            stage_diagnostics[target_side] = {
                "source_predicted_upper_direction": source_upper_direction.tolist(),
                "source_predicted_forearm_direction": source_forearm_direction.tolist(),
                "mapped_before_filter_deg": mapped_before_filter_deg,
                "mapped_before_filter_upper_direction": mapped_before_filter_upper.tolist(),
                "mapped_before_filter_forearm_direction": mapped_before_filter_forearm.tolist(),
                "filtered_before_plane_gate_deg": filtered_before_plane_gate_deg,
                "filtered_before_plane_upper_direction": filtered_before_plane_upper.tolist(),
                "filtered_before_plane_forearm_direction": filtered_before_plane_forearm.tolist(),
                "desired_after_plane_gate_deg": desired_after_plane_gate_deg,
                "desired_after_plane_upper_direction": desired_after_plane_upper.tolist(),
                "desired_after_plane_forearm_direction": desired_after_plane_forearm.tolist(),
            }
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

    @staticmethod
    def _neutral_relative_forearm(upper, forearm, reference_bend, robot_upper, robot_forearm,
                                  *, mode="relative", start_deg=5.0, full_deg=30.0,
                                  plane_start_deg=4.0, plane_full_deg=10.0):
        """Map elbow bend while suppressing its unobservable near-straight plane."""
        u = mu.normalize_vector(upper)
        f = mu.normalize_vector(forearm)
        ru = mu.normalize_vector(robot_upper)
        rf = mu.normalize_vector(robot_forearm)
        if any(v is None for v in (u, f, ru, rf)):
            raise ValueError("invalid arm segment for neutral-relative retargeting")
        carry = mu.minimal_vector_alignment_rotation(ru, u)
        if carry is None:
            raise ValueError("cannot carry robot forearm with upper arm")
        base = carry @ rf
        bend = float(np.arccos(np.clip(np.dot(u, f), -1, 1)))
        robot_bend = float(np.arccos(np.clip(np.dot(ru, rf), -1, 1)))
        # Natural-down cannot extend through the elbow's straight configuration.
        target_bend, _ = BodyDevice._mapped_elbow_angle(
            bend, reference_bend, robot_bend, mode, start_deg, full_deg)
        if mode != "direct_absolute" and abs(target_bend - robot_bend) < 1.0e-6:
            return base

        live_axis = mu.normalize_vector(np.cross(u, f))
        if live_axis is None:
            live_axis = mu.normalize_vector(np.cross(u, base))
        if live_axis is None:
            live_axis = mu.normalize_vector(np.cross(
                u, np.eye(3)[np.argmin(np.abs(u))]
            ))
        mapped_live = mu.rotvec_to_rotation_matrix(live_axis * target_bend) @ u
        return BodyDevice._stabilize_forearm_plane(
            u,
            mapped_live,
            ru,
            rf,
            target_bend_rad=target_bend,
            plane_start_deg=plane_start_deg,
            plane_full_deg=plane_full_deg,
        )

    @staticmethod
    def _stabilize_forearm_plane(upper, forearm, robot_upper, robot_forearm,
                                 *, target_bend_rad=None,
                                 plane_start_deg=4.0, plane_full_deg=10.0):
        """Keep the bend plane calibrated until the actual bend is observable."""
        u = mu.normalize_vector(upper)
        f = mu.normalize_vector(forearm)
        ru = mu.normalize_vector(robot_upper)
        rf = mu.normalize_vector(robot_forearm)
        if any(v is None for v in (u, f, ru, rf)):
            raise ValueError("invalid arm segment for bend-plane stabilization")
        carry = mu.minimal_vector_alignment_rotation(ru, u)
        if carry is None:
            raise ValueError("cannot carry robot forearm with upper arm")
        base = carry @ rf
        target_bend = (
            float(np.arccos(np.clip(np.dot(u, f), -1, 1)))
            if target_bend_rad is None else float(target_bend_rad)
        )
        robot_bend = float(np.arccos(np.clip(np.dot(ru, rf), -1, 1)))
        if not np.isfinite(target_bend):
            raise ValueError("invalid target elbow bend")
        normalized_base = mu.normalize_vector(base)
        if normalized_base is not None and np.linalg.norm(f - normalized_base) < 1.0e-5:
            # Preserve the exact already-calibrated direction. Rebuilding it
            # through another float32 rotation can otherwise create a small
            # startup wrist translation despite an unchanged human pose.
            return f
        plane_start, plane_full = np.deg2rad([plane_start_deg, plane_full_deg])
        if (not np.all(np.isfinite([plane_start, plane_full]))
                or plane_start < 0 or plane_full <= plane_start):
            raise ValueError("invalid elbow bend-plane transition")

        # `base` carries the calibrated robot bend plane with the live upper
        # arm. The PICO plane is noisy only near actual elbow extension because
        # cross(u, f) is ill-conditioned there. Gate on the target bend itself,
        # rather than its departure from the startup robot pose: a bent arm is
        # observable even when the robot happened to start at the same bend.
        base_axis = mu.normalize_vector(np.cross(u, base))
        live_axis = mu.normalize_vector(np.cross(u, f))
        if base_axis is None:
            base_axis = live_axis
        if base_axis is None:
            base_axis = mu.normalize_vector(np.cross(
                u, np.eye(3)[np.argmin(np.abs(u))]
            ))
        if live_axis is None:
            live_axis = base_axis

        observable_bend = max(0.0, target_bend)
        if observable_bend <= plane_start:
            visibility = 0.0
        elif observable_bend >= plane_full:
            visibility = 1.0
        else:
            ratio = (observable_bend - plane_start) / (plane_full - plane_start)
            visibility = ratio * ratio * (3.0 - 2.0 * ratio)

        stable = mu.rotvec_to_rotation_matrix(base_axis * target_bend) @ u
        observed = mu.rotvec_to_rotation_matrix(live_axis * target_bend) @ u
        blended = mu.normalize_vector((1.0 - visibility) * stable + visibility * observed)
        if blended is None:
            blended = stable if visibility < 0.5 else observed
        blended_axis = mu.normalize_vector(np.cross(u, blended))
        if blended_axis is None:
            return blended
        # Reproject after blending so the requested elbow angle is preserved.
        return mu.rotvec_to_rotation_matrix(blended_axis * target_bend) @ u

    @staticmethod
    def _mapped_elbow_angle(bend, reference, robot_bend, mode, start_deg, full_deg):
        if mode == "direct_absolute":
            return float(np.clip(bend, 0, np.pi)), 1.0
        if mode == "relative":
            return float(np.clip(robot_bend + bend - reference, 0, np.pi)), 0.0
        if mode not in ("smooth_absolute", "responsive_absolute"):
            raise ValueError("unknown elbow angle mapping")
        start, full = np.deg2rad([start_deg, full_deg])
        if not np.all(np.isfinite([start, full])) or start < 0 or full <= start:
            raise ValueError("invalid elbow angle transition")
        t = float(np.clip((bend - reference - start) / (full - start), 0, 1))
        weight = t * t * (3 - 2 * t)
        base = robot_bend
        if mode == "responsive_absolute":
            # Add a smoothly engaged relative response before absolute blending
            # dominates. Keep both the neutral band and full-angle endpoint.
            departure = max(0.0, bend - reference - start)
            onset = min(np.deg2rad(5.0), full - start)
            u = float(np.clip(departure / onset, 0, 1))
            base += departure * u * u * (3 - 2 * u)
        return (1 - weight) * base + weight * bend, weight

    @staticmethod
    def _points_flexion(points):
        upper = mu.normalize_vector(points["elbow"] - points["shoulder"])
        forearm = mu.normalize_vector(points["wrist"] - points["elbow"])
        if upper is None or forearm is None:
            raise ValueError("invalid arm segments")
        return float(np.arccos(np.clip(np.dot(upper, forearm), -1, 1)))

    def elbow_diagnostics(self, side):
        points = self._current_arm_points(self._source_side_for_target(side))
        if points is None or not self._reference_locked:
            return None
        bend = self._points_flexion(points)
        reference = self.arm_reference_flexion_rad(side)
        shoulder, elbow, wrist, _ = self._robot_arm_reference(side)
        robot = self._points_flexion(dict(shoulder=shoulder, elbow=elbow, wrist=wrist))
        angle, weight = self._mapped_elbow_angle(
            bend, reference, robot, self._cfg.elbow_angle_mapping,
            self._cfg.elbow_absolute_start_delta_deg, self._cfg.elbow_absolute_full_delta_deg)
        if self._cfg.arm_vector_position_mode != "segment_direction_relative":
            angle, weight = bend, None
        return dict(reference_deg=float(np.degrees(reference)), human_deg=float(np.degrees(bend)),
                    blend_weight=weight, raw_mapped_deg=float(np.degrees(angle)))

    def arm_retarget_diagnostics(self, side: str) -> dict[str, Any]:
        """Return the latest stage-by-stage arm-vector target diagnostics."""
        if side not in ("left", "right"):
            raise ValueError("side must be left or right")
        return dict(self._arm_retarget_diagnostics.get(side, {}))

    def arm_reference_deviation_deg(self, target_side: str) -> Optional[tuple[float, float]]:
        """Return current upper/forearm angular distance from the PICO reference."""
        if target_side not in ("left", "right"):
            raise ValueError("target_side must be 'left' or 'right'")
        source_side = self._source_side_for_target(target_side)
        current = self._current_arm_points(source_side)
        reference = (
            self._reference_left_points if target_side == "left" else self._reference_right_points
        )
        if current is None or reference is None:
            return None
        current_signed = mu.arm_points_relative_to_shoulder(current, self._signs)
        reference_signed = mu.arm_points_relative_to_shoulder(reference, self._signs)

        def segment_angle(start: str, end: str) -> float:
            current_direction = mu.normalize_vector(current_signed[end] - current_signed[start])
            reference_direction = mu.normalize_vector(
                reference_signed[end] - reference_signed[start]
            )
            if current_direction is None or reference_direction is None:
                return float("inf")
            cosine = float(np.clip(np.dot(current_direction, reference_direction), -1.0, 1.0))
            return float(np.degrees(np.arccos(cosine)))

        return segment_angle("shoulder", "elbow"), segment_angle("elbow", "wrist")

    def arm_reference_flexion_rad(self, side: str) -> float:
        """Return the elbow bend in the accepted multi-frame human reference."""
        if side not in ("left", "right"):
            raise ValueError("side must be left or right")
        points = self._reference_left_points if side == "left" else self._reference_right_points
        if points is None:
            raise ValueError("human arm reference is unavailable")
        upper = mu.normalize_vector(points["elbow"] - points["shoulder"])
        forearm = mu.normalize_vector(points["wrist"] - points["elbow"])
        if upper is None or forearm is None:
            raise ValueError("human arm reference contains a zero-length segment")
        angle = float(np.arccos(np.clip(np.dot(upper, forearm), -1.0, 1.0)))
        if not np.isfinite(angle):
            raise ValueError("human arm reference angle is not finite")
        return angle

    def seed_arm_reference_filters(self, side: str) -> None:
        """Start directional filtering at the accepted pose, before live motion."""
        self.arm_reference_flexion_rad(side)
        if self._cfg.elbow_angle_mapping == "direct_absolute":
            # The formal calibration pose may sit at the edge of PICO tracking
            # and can have a very different elbow bend from the stable target
            # accepted at enable time.  Do not inject that stale pose into the
            # filter.  The first live target is still rate-limited from measured
            # robot feedback by _limit_arm_segment_translations().
            for name in ("upper_arm", "forearm"):
                self._filtered_segment_directions[side].pop(name, None)
                self._held_segment_directions[side].pop(name, None)
                self._filtered_segment_timestamps[side].pop(name, None)
            return
        points = self._reference_left_points if side == "left" else self._reference_right_points
        signed = mu.arm_points_relative_to_shoulder(points, self._signs)
        upper = signed["elbow"] - signed["shoulder"]
        forearm = signed["wrist"] - signed["elbow"]
        rotation = np.eye(3)
        if self._cfg.arm_vector_position_mode == "segment_direction_relative":
            shoulder, elbow, wrist, _ = self._robot_arm_reference(side)
            rotation = mu.minimal_vector_alignment_rotation(upper, elbow - shoulder)
            if rotation is None:
                raise ValueError("cannot align human arm reference")
        mapped = {"upper_arm": rotation @ upper, "forearm": rotation @ forearm}
        if self._cfg.arm_vector_position_mode == "segment_direction_relative":
            mapped["forearm"] = self._neutral_relative_forearm(
                mapped["upper_arm"], mapped["forearm"], self.arm_reference_flexion_rad(side),
                elbow - shoulder, wrist - elbow,
                mode=self._cfg.elbow_angle_mapping,
                start_deg=self._cfg.elbow_absolute_start_delta_deg,
                full_deg=self._cfg.elbow_absolute_full_delta_deg,
                plane_start_deg=self._cfg.bend_plane_observability_start_delta_deg,
                plane_full_deg=self._cfg.bend_plane_observability_full_delta_deg,
            )
        for name, vector in mapped.items():
            direction = mu.normalize_vector(vector)
            self._filtered_segment_directions[side][name] = direction.copy()
            self._held_segment_directions[side][name] = direction.copy()
            self._filtered_segment_timestamps[side][name] = self._last_body_frame_packet_time_monotonic

    def refresh_arm_reference_for_enable(
        self, target_side: str
    ) -> Optional[tuple[bool, float, float, float, int]]:
        """Capture a stable recent tracking target without changing calibration."""
        with getattr(self, "_reference_lock", nullcontext()):
            result = self._refresh_arm_reference_for_enable(target_side)
            if result is None:
                reference = self._reference_left_points if target_side == "left" else self._reference_right_points
                self._record_reference_diagnostic(
                    "enable_unavailable_or_unstable", target_side, reference,
                    None, 0, None, None, time.monotonic(),
                )
            return result

    def _refresh_arm_reference_for_enable(self, target_side):
        if target_side not in ("left", "right"):
            raise ValueError("target_side must be 'left' or 'right'")
        history = list(self._body_frame_history)
        if not history:
            return None
        source_side = self._source_side_for_target(target_side)
        if self._arm_points_from_frames(history[-1][1], source_side) is None:
            return None
        latest_time = history[-1][0]
        if time.monotonic() - latest_time > self._cfg.max_stale_time_s:
            return None
        window_s = max(0.1, float(self._cfg.arm_reference_enable_window_s))
        samples = []
        sample_times = []
        for timestamp, frames in history:
            if latest_time - timestamp > window_s:
                continue
            points = self._arm_points_from_frames(frames, source_side)
            if points is not None:
                samples.append(points)
                sample_times.append(timestamp)
        minimum = max(5, int(self._cfg.auto_start_reference_min_samples))
        if len(samples) < minimum:
            return None
        max_std = mu_arm_points_max_std(samples)
        if max_std > self._cfg.auto_start_reference_max_position_std_m:
            return None
        candidate = mu.average_arm_points(samples)
        reference = (
            self._reference_left_points if target_side == "left" else self._reference_right_points
        )
        if candidate is None or reference is None:
            return None

        def angle(start: str, end: str) -> float:
            candidate_direction = mu.normalize_vector(candidate[end] - candidate[start])
            reference_direction = mu.normalize_vector(reference[end] - reference[start])
            if candidate_direction is None or reference_direction is None:
                return float("inf")
            cosine = float(
                np.clip(np.dot(candidate_direction, reference_direction), -1.0, 1.0)
            )
            return float(np.degrees(np.arccos(cosine)))

        upper_delta = angle("shoulder", "elbow")
        forearm_delta = angle("elbow", "wrist")
        reference_bend = float(np.degrees(self._points_flexion(reference)))
        candidate_bend = float(np.degrees(self._points_flexion(candidate)))
        self._record_reference_diagnostic(
            "enable_target_ready", target_side,
            reference, candidate, len(samples), max_std,
            sample_times[0], sample_times[-1],
            [upper_delta, forearm_delta],
        )
        print(
            f"[PICO 参考] latest stable target ready: elbow calibration="
            f"{reference_bend:.1f}deg, current={candidate_bend:.1f}deg; "
            f"upper/forearm offset={upper_delta:.1f}/{forearm_delta:.1f}deg. "
            "Calibration remains fixed; the robot will approach this target through "
            "the configured rate and trajectory limits.",
            flush=True,
        )
        return True, upper_delta, forearm_delta, max_std, len(samples)

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
    def _slerp_direction(start: np.ndarray, end: np.ndarray, progress: float) -> np.ndarray:
        """Interpolate two unit directions along a continuous shortest arc."""
        first = mu.normalize_vector(start)
        second = mu.normalize_vector(end)
        if first is None or second is None:
            raise ValueError("cannot interpolate a zero-length direction")
        amount = float(np.clip(progress, 0.0, 1.0))
        cosine = float(np.clip(np.dot(first, second), -1.0, 1.0))
        if cosine > 1.0 - 1.0e-7:
            result = mu.normalize_vector((1.0 - amount) * first + amount * second)
            return first.copy() if result is None else result
        if cosine < -1.0 + 1.0e-7:
            basis = np.eye(3)[int(np.argmin(np.abs(first)))]
            axis = mu.normalize_vector(np.cross(first, basis))
            if axis is None:
                return first.copy()
            return (mu.rotvec_to_rotation_matrix(axis * np.pi * amount) @ first).astype(
                np.float32
            )
        angle = float(np.arccos(cosine))
        sine = float(np.sin(angle))
        result = (
            np.sin((1.0 - amount) * angle) / sine * first
            + np.sin(amount * angle) / sine * second
        )
        normalized = mu.normalize_vector(result)
        return first.copy() if normalized is None else normalized

    @staticmethod
    def _limit_arm_segment_translations(
        previous_elbow_pose: np.ndarray,
        previous_wrist_pose: np.ndarray,
        target_elbow_pose: np.ndarray,
        target_wrist_pose: np.ndarray,
        shoulder_position: np.ndarray,
        max_velocity: float,
        dt: float,
        *,
        forearm_projector=None,
        diagnostics: dict | None = None,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Advance a two-link arm through bounded geometric state increments.

        The old implementation bisected a scalar interpolation progress while
        applying the bend-plane projector inside every probe.  The projector is
        state dependent around the 4--10 degree observability transition, so
        endpoint displacement is not monotonic in that progress.  This version
        computes the fully stabilized destination once, advances upper-arm
        direction, elbow bend and bend-plane direction continuously, then only
        backtracks toward the known-safe previous state if the reconstructed
        endpoint step exceeds the Cartesian bound.
        """
        previous_elbow = np.asarray(previous_elbow_pose, dtype=np.float32).reshape(7)
        previous_wrist = np.asarray(previous_wrist_pose, dtype=np.float32).reshape(7)
        target_elbow = np.asarray(target_elbow_pose, dtype=np.float32).reshape(7).copy()
        target_wrist = np.asarray(target_wrist_pose, dtype=np.float32).reshape(7).copy()
        shoulder = np.asarray(shoulder_position, dtype=np.float32).reshape(3)
        if diagnostics is not None:
            diagnostics.clear()
            diagnostics["method"] = "arm_state_arc_backtrack"
        if not np.isfinite(dt) or dt <= 0.0:
            target_elbow[:3] = previous_elbow[:3]
            target_wrist[:3] = previous_wrist[:3]
            if diagnostics is not None:
                diagnostics.update(progress=0.0, reason="invalid_dt")
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
            if diagnostics is not None:
                diagnostics.update(progress=0.0, reason="invalid_segment")
            return target_elbow, target_wrist

        # Stabilize the complete destination once.  It becomes the fixed goal
        # for this cycle instead of changing independently at each search probe.
        if forearm_projector is not None:
            projected = mu.normalize_vector(forearm_projector(target_upper, target_forearm))
            if projected is None:
                target_elbow[:3] = previous_elbow[:3]
                target_wrist[:3] = previous_wrist[:3]
                if diagnostics is not None:
                    diagnostics.update(progress=0.0, reason="invalid_projected_target")
                return target_elbow, target_wrist
            target_forearm = projected
            target_wrist[:3] = target_elbow[:3] + target_forearm * forearm_length

        previous_bend = float(
            np.arccos(np.clip(np.dot(previous_upper, previous_forearm), -1.0, 1.0))
        )
        target_bend = float(
            np.arccos(np.clip(np.dot(target_upper, target_forearm), -1.0, 1.0))
        )
        previous_axis = mu.normalize_vector(np.cross(previous_upper, previous_forearm))
        target_axis = mu.normalize_vector(np.cross(target_upper, target_forearm))
        if previous_axis is None:
            previous_axis = target_axis
        if target_axis is None:
            target_axis = previous_axis
        if previous_axis is None:
            basis = np.eye(3)[int(np.argmin(np.abs(previous_upper)))]
            previous_axis = mu.normalize_vector(np.cross(previous_upper, basis))
            target_axis = previous_axis

        def candidate(progress: float) -> tuple[np.ndarray, np.ndarray]:
            if progress <= 0.0:
                return previous_elbow[:3].copy(), previous_wrist[:3].copy()
            if progress >= 1.0:
                return target_elbow[:3].copy(), target_wrist[:3].copy()
            upper_direction = BodyDevice._slerp_direction(
                previous_upper, target_upper, progress
            )
            plane_axis = BodyDevice._slerp_direction(
                previous_axis, target_axis, progress
            )
            plane_axis = mu.normalize_vector(
                plane_axis - upper_direction * np.dot(plane_axis, upper_direction)
            )
            if plane_axis is None:
                provisional = BodyDevice._slerp_direction(
                    previous_forearm, target_forearm, progress
                )
                plane_axis = mu.normalize_vector(np.cross(upper_direction, provisional))
            if plane_axis is None:
                return previous_elbow[:3].copy(), previous_wrist[:3].copy()
            bend = (1.0 - progress) * previous_bend + progress * target_bend
            forearm_direction = mu.normalize_vector(
                mu.rotvec_to_rotation_matrix(plane_axis * bend) @ upper_direction
            )
            if forearm_direction is None:
                return previous_elbow[:3].copy(), previous_wrist[:3].copy()
            # Reapply the near-straight observability gate only to the selected
            # state.  Backtracking always approaches the exact previous state;
            # it does not binary-search a globally non-monotonic projection.
            if forearm_projector is not None:
                forearm_direction = mu.normalize_vector(
                    forearm_projector(upper_direction, forearm_direction)
                )
                if forearm_direction is None:
                    return previous_elbow[:3].copy(), previous_wrist[:3].copy()
            elbow_position = shoulder + upper_direction * upper_length
            wrist_position = elbow_position + forearm_direction * forearm_length
            return elbow_position, wrist_position

        max_distance = float(max_velocity) * dt
        full_elbow, full_wrist = candidate(1.0)
        full_elbow_step = float(np.linalg.norm(full_elbow - previous_elbow[:3]))
        full_wrist_step = float(np.linalg.norm(full_wrist - previous_wrist[:3]))
        if max_velocity <= 0.0:
            target_elbow[:3], target_wrist[:3] = full_elbow, full_wrist
            if diagnostics is not None:
                diagnostics.update(
                    progress=1.0,
                    backtracks=0,
                    max_distance_m=None,
                    elbow_step_m=full_elbow_step,
                    wrist_step_m=full_wrist_step,
                )
            return target_elbow, target_wrist

        full_step = max(full_elbow_step, full_wrist_step)
        progress = 1.0
        if full_step > max_distance:
            upper_angle = float(
                np.arccos(np.clip(np.dot(previous_upper, target_upper), -1.0, 1.0))
            )
            forearm_angle = float(
                np.arccos(np.clip(np.dot(previous_forearm, target_forearm), -1.0, 1.0))
            )
            arc_bound = max(
                upper_length * upper_angle,
                upper_length * upper_angle + forearm_length * forearm_angle,
            )
            progress = min(1.0, max_distance / arc_bound) if arc_bound > 0.0 else 0.0

        backtracks = 0
        elbow_position, wrist_position = candidate(progress)
        elbow_step = float(np.linalg.norm(elbow_position - previous_elbow[:3]))
        wrist_step = float(np.linalg.norm(wrist_position - previous_wrist[:3]))
        while max(elbow_step, wrist_step) > max_distance + 1.0e-9 and progress > 1.0e-7:
            progress *= 0.5
            backtracks += 1
            elbow_position, wrist_position = candidate(progress)
            elbow_step = float(np.linalg.norm(elbow_position - previous_elbow[:3]))
            wrist_step = float(np.linalg.norm(wrist_position - previous_wrist[:3]))
        if max(elbow_step, wrist_step) > max_distance + 1.0e-9:
            progress = 0.0
            elbow_position, wrist_position = candidate(0.0)
            elbow_step = wrist_step = 0.0

        target_elbow[:3], target_wrist[:3] = elbow_position, wrist_position
        if diagnostics is not None:
            limited_upper = mu.normalize_vector(elbow_position - shoulder)
            limited_forearm = mu.normalize_vector(wrist_position - elbow_position)
            limited_bend = previous_bend
            if limited_upper is not None and limited_forearm is not None:
                limited_bend = float(np.arccos(np.clip(
                    np.dot(limited_upper, limited_forearm), -1.0, 1.0
                )))
            diagnostics.update(
                progress=float(progress),
                backtracks=int(backtracks),
                max_distance_m=float(max_distance),
                elbow_step_m=elbow_step,
                wrist_step_m=wrist_step,
                previous_bend_deg=float(np.degrees(previous_bend)),
                desired_bend_deg=float(np.degrees(target_bend)),
                limited_bend_deg=float(np.degrees(limited_bend)),
            )
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
        desired_before_limit_deg = float(np.degrees(self._points_flexion({
            "shoulder": robot_shoulder,
            "elbow": elbow_pose[:3],
            "wrist": wrist_pose[:3],
        })))
        forearm_projector = None
        if self._cfg.arm_vector_position_mode == "segment_direction_relative":
            _, robot_elbow, robot_wrist, _ = self._robot_arm_reference(target_side)
            robot_upper = robot_elbow - robot_shoulder
            robot_forearm = robot_wrist - robot_elbow

            def forearm_projector(upper_direction, forearm_direction):
                return self._stabilize_forearm_plane(
                    upper_direction,
                    forearm_direction,
                    robot_upper,
                    robot_forearm,
                    plane_start_deg=self._cfg.bend_plane_observability_start_delta_deg,
                    plane_full_deg=self._cfg.bend_plane_observability_full_delta_deg,
                )

        limiter_diagnostics: dict[str, float | int | str] = {}
        if target_side == "left":
            elbow_pose, wrist_pose = self._limit_arm_segment_translations(
                self._previous_left_elbow_pose,
                self._previous_left_pose,
                elbow_pose,
                wrist_pose,
                self._robot_left_shoulder,
                self._cfg.max_endpoint_translation_velocity_m_s,
                target_dt,
                forearm_projector=forearm_projector,
                diagnostics=limiter_diagnostics,
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
                forearm_projector=forearm_projector,
                diagnostics=limiter_diagnostics,
            )
            self._previous_right_elbow_pose = elbow_pose
            self._previous_right_pose = wrist_pose
        limited_deg = float(np.degrees(self._points_flexion({
            "shoulder": robot_shoulder,
            "elbow": elbow_pose[:3],
            "wrist": wrist_pose[:3],
        })))
        limited_upper_direction = mu.normalize_vector(elbow_pose[:3] - robot_shoulder)
        limited_forearm_direction = mu.normalize_vector(wrist_pose[:3] - elbow_pose[:3])
        stage_diagnostics = getattr(self, "_arm_retarget_diagnostics", None)
        if stage_diagnostics is not None:
            stage_diagnostics.setdefault(target_side, {}).update(
                desired_before_limit_deg=desired_before_limit_deg,
                limited_deg=limited_deg,
                limited_upper_direction=limited_upper_direction.tolist(),
                limited_forearm_direction=limited_forearm_direction.tolist(),
                target_dt_s=float(target_dt),
                limiter=limiter_diagnostics,
            )
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
        self._filtered_segment_directions = {"left": {}, "right": {}}
        self._filtered_segment_timestamps = {"left": {}, "right": {}}
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
