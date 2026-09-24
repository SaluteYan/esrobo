"""LinkerHand dexterous-hand driver for the ESROBO robot.

Control method (verified from the esrobo ``linker_hand_ros2_sdk``): the ROS2
node ``linker_hand.py`` subscribes to ``/cb_{left|right}_hand_control_cmd``
(``sensor_msgs/JointState``) and calls ``LinkerHandApi.finger_move(pose=...)``,
which expects **0..255 integer servo positions** -- NOT radians.  The number of
values depends on the hand model: ``L10`` -> 10, ``L20`` -> 20.

The installed L20Lite hand is exposed by its SDK as the 10-actuator ``L10``
interface. The teleoperation pipeline produces those 10 active hand-joint
angles; the full URDF expands their passive joints through mimic relations.
This driver maps the active values to 0..255, then applies safety clamps before
sending to the LinkerHand node over loopback UDP (forwarded by
``bridges/hand_ros_bridge.py``) or directly over ROS2.
"""

from __future__ import annotations

import json
import socket
import time
import threading
from typing import Optional

import numpy as np

from ..config import HandConfig

# ESROBO active hand joints per hand, in the teleop order (radians).
ACTIVE_HAND_JOINTS = [
    "thumb_cmc_roll", "thumb_cmc_yaw", "thumb_cmc_pitch",
    "index_mcp_roll", "index_mcp_pitch",
    "middle_mcp_pitch",
    "ring_mcp_roll", "ring_mcp_pitch",
    "pinky_mcp_roll", "pinky_mcp_pitch",
]

# URDF position limits (rad) for the 10 active joints -- used for the default
# rad -> 0..255 linear map (value = round(rad / upper * 255)).
ACTIVE_JOINT_LIMITS_RAD = {
    "thumb_cmc_roll": (0.0, 1.03),
    "thumb_cmc_yaw": (0.0, 1.40),
    "thumb_cmc_pitch": (0.0, 0.52),
    "index_mcp_roll": (0.0, 0.19),
    "index_mcp_pitch": (0.0, 1.36),
    "middle_mcp_pitch": (0.0, 1.36),
    "ring_mcp_roll": (0.0, 0.20),
    "ring_mcp_pitch": (0.0, 1.36),
    "pinky_mcp_roll": (0.0, 0.30),
    "pinky_mcp_pitch": (0.0, 1.36),
}

# Active -> passive relations in the full L20Lite URDF representation.
MIMIC_JOINTS = {
    "thumb_mcp": ("thumb_cmc_pitch", 1.38),
    "thumb_ip": ("thumb_cmc_pitch", 1.49),
    "index_pip": ("index_mcp_pitch", 1.30),
    "index_dip": ("index_mcp_pitch", 0.46),
    "middle_pip": ("middle_mcp_pitch", 1.30),
    "middle_dip": ("middle_mcp_pitch", 0.46),
    "ring_pip": ("ring_mcp_pitch", 1.30),
    "ring_dip": ("ring_mcp_pitch", 0.46),
    "pinky_pip": ("pinky_mcp_pitch", 1.30),
    "pinky_dip": ("pinky_mcp_pitch", 0.46),
}

# URDF position limits (rad) for the 10 passive/mimic L20Lite joints.
MIMIC_JOINT_LIMITS_RAD = {
    "thumb_mcp": (0.0, 0.52 * 1.38),
    "thumb_ip": (0.0, 0.52 * 1.49),
    "index_pip": (0.0, 1.36 * 1.30),
    "index_dip": (0.0, 1.36 * 0.46),
    "middle_pip": (0.0, 1.36 * 1.30),
    "middle_dip": (0.0, 1.36 * 0.46),
    "ring_pip": (0.0, 1.36 * 1.30),
    "ring_dip": (0.0, 1.36 * 0.46),
    "pinky_pip": (0.0, 1.36 * 1.30),
    "pinky_dip": (0.0, 1.36 * 0.46),
}

PHYSICAL_JOINT_ORDER = ACTIVE_HAND_JOINTS + list(MIMIC_JOINTS.keys())

# LinkerHand L10 SDK/CAN order.  Keep this next to the permutation below so
# startup diagnostics identify the physical actuator that did not respond.
L10_PHYSICAL_JOINT_NAMES = [
    "thumb_cmc_pitch", "thumb_cmc_yaw", "index_mcp_pitch",
    "middle_mcp_pitch", "ring_mcp_pitch", "pinky_mcp_pitch",
    "index_mcp_roll", "ring_mcp_roll", "pinky_mcp_roll", "thumb_cmc_roll",
]

# LinkerHand L10 SDK order -> ACTIVE_HAND_JOINTS index. Verified against the
# SDK Topic-Reference.md. This is not the ESROBO URDF/teleop order.
L10_ACTIVE_INDEX_BY_PHYSICAL = np.asarray([2, 1, 4, 5, 7, 9, 3, 6, 8, 0], dtype=np.int64)


def _rad_to_servo_map(names: list[str], limits: dict) -> tuple[np.ndarray, np.ndarray]:
    scale = np.zeros(len(names), dtype=np.float64)
    offset = np.zeros(len(names), dtype=np.float64)
    for i, name in enumerate(names):
        lo, hi = limits[name]
        span = max(hi - lo, 1.0e-6)
        scale[i] = 255.0 / span
        offset[i] = -lo * scale[i]
    return scale, offset


class LinkerHandDriver:
    """Maps teleop hand joints to 0..255 and commands the physical hands safely."""

    def __init__(self, cfg: HandConfig, udp_host: str = "127.0.0.1", udp_port: int = 15051,
                 active_sides: tuple[str, ...] = ("left", "right")):
        self._feedback_lock = threading.RLock()
        self._geometry_feedback_time = {"left": None, "right": None}
        self._cfg = cfg
        self._mode = cfg.mode
        self._model = (cfg.model or "L10").upper()
        self._active_sides = tuple(side for side in active_sides if side in ("left", "right"))
        if not self._active_sides:
            raise ValueError("active_sides must contain 'left' and/or 'right'")
        self._enabled = False

        if self._model == "L10":
            self._physical_count = 10
        elif self._model == "L20":
            self._physical_count = 20
        else:
            raise ValueError(f"unsupported hand model: {self._model!r} (L10|L20)")

        if cfg.home and len(cfg.home) == self._physical_count:
            self._home = np.asarray(cfg.home, dtype=np.float64)
        else:
            self._home = np.full(self._physical_count, 0.0, dtype=np.float64)

        self._last_cmd: dict[str, np.ndarray] = {
            "left": np.zeros(self._physical_count, dtype=np.float64),
            "right": np.zeros(self._physical_count, dtype=np.float64),
        }
        self._last_sent_time = time.monotonic()
        self._feedback: dict[str, np.ndarray | None] = {"left": None, "right": None}
        self._feedback_time: dict[str, float | None] = {"left": None, "right": None}

        if cfg.rad_scale and cfg.rad_offset:
            # User-provided per-physical-joint scale/offset.
            scale = np.asarray(cfg.rad_scale, dtype=np.float64).reshape(-1)
            offset = np.asarray(cfg.rad_offset, dtype=np.float64).reshape(-1)
            if scale.shape[0] != self._physical_count:
                raise ValueError(
                    f"rad_scale length {scale.shape[0]} != physical count {self._physical_count}"
                )
            self._scale, self._offset = scale, offset
        else:
            if self._model == "L20":
                limits = dict(ACTIVE_JOINT_LIMITS_RAD)
                limits.update(MIMIC_JOINT_LIMITS_RAD)
                self._scale, self._offset = _rad_to_servo_map(PHYSICAL_JOINT_ORDER, limits)
            else:
                self._scale, self._offset = _rad_to_servo_map(ACTIVE_HAND_JOINTS, ACTIVE_JOINT_LIMITS_RAD)

        self._node = None
        self._publishers = {}
        self._sock = None
        if self._mode == "udp":
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._sock.settimeout(0.1)
            self._udp_addr = (udp_host, udp_port)
            self._feedback_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._feedback_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._feedback_sock.bind((cfg.feedback_udp_host, cfg.feedback_udp_port))
            self._feedback_sock.setblocking(False)
        elif self._mode == "ros2":
            self._init_ros2()

    def _init_ros2(self) -> None:
        try:
            import rclpy
            from sensor_msgs.msg import JointState
        except ImportError as exc:
            raise RuntimeError(
                "rclpy not available in this environment; use mode='udp' or run under system ROS2."
            ) from exc
        rclpy.init()
        self._node = rclpy.create_node("esrobo_hand_driver")
        self._msg_type = JointState
        self._publishers["left"] = self._node.create_publisher(
            JointState, self._cfg.left_topic, 10
        )
        self._publishers["right"] = self._node.create_publisher(
            JointState, self._cfg.right_topic, 10
        )

    def _poll_feedback(self) -> None:
        with self._feedback_lock:
            self._poll_feedback_locked()

    def _poll_feedback_locked(self) -> None:
        if self._mode != "udp":
            return
        while True:
            try:
                payload, _addr = self._feedback_sock.recvfrom(4096)
            except BlockingIOError:
                break
            try:
                message = json.loads(payload.decode("utf-8"))
            except (ValueError, UnicodeDecodeError):
                continue
            if not isinstance(message, dict):
                continue
            side = message.get("side")
            try:
                values = np.asarray(message.get("position", []), dtype=np.float64).reshape(-1)
            except (TypeError, ValueError):
                continue
            valid_range = np.all(
                (values >= self._cfg.out_min) & (values <= self._cfg.out_max)
            )
            if (
                side in self._feedback
                and values.shape == (self._physical_count,)
                and np.all(np.isfinite(values))
                and valid_range
            ):
                source_stamp = message.get("sample_monotonic_s")
                self._geometry_feedback_time[side] = (float(source_stamp)
                    if isinstance(source_stamp, (int, float)) else None)
                self._feedback[side] = values
                self._feedback_time[side] = time.monotonic()

    def geometry_state(self, side: str, horizon: float):
        """Receive only; this never enables or commands the hand."""
        from .hand_geometry import geometry_state
        with self._feedback_lock:
            self._poll_feedback_locked()
            names = L10_PHYSICAL_JOINT_NAMES if self._model == "L10" else PHYSICAL_JOINT_ORDER
            # When software hand output is disabled no hand target can change,
            # so only feedback age and calibration error enlarge the measured
            # pose.  Active following retains the full future-motion horizon.
            effective_horizon = horizon if self._enabled else 0.0
            return geometry_state(self._feedback.get(side), self._geometry_feedback_time.get(side),
                side, self._cfg.geometry_feedback_calibration, names, effective_horizon,
                self._cfg.geometry_feedback_timeout_s)

    def geometry_calibration_status(self, side: str) -> dict:
        """Describe whether collision geometry can use every physical actuator.

        Natural-open feedback is a startup pose check.  It is deliberately not
        counted here: moving an arm while its fingers can move requires the
        independently verified feedback-to-URDF mapping for all ten axes.
        """
        if side not in self._active_sides:
            raise ValueError(f"{side!r} is not an active LinkerHand side")
        names = L10_PHYSICAL_JOINT_NAMES if self._model == "L10" else PHYSICAL_JOINT_ORDER
        root = self._cfg.geometry_feedback_calibration
        records = root.get(side, {}) if isinstance(root, dict) else {}
        calibrated, invalid = [], []
        for name in names:
            item = records.get(name) if isinstance(records, dict) else None
            try:
                raw0, raw1 = item["raw"]
                rad0, rad1 = item["rad"]
                values = [raw0, raw1, rad0, rad1,
                          item["error_rad"], item["max_velocity_rad_s"]]
                valid = (item.get("verified") is True and np.all(np.isfinite(values))
                         and raw0 != raw1 and item["error_rad"] > 0
                         and item["max_velocity_rad_s"] > 0)
            except (KeyError, TypeError, ValueError):
                valid = False
            if valid:
                calibrated.append(name)
            elif item is not None:
                invalid.append(name)
        missing = [name for name in names if name not in calibrated]
        return {
            "ready": not missing,
            "calibrated_joints": len(calibrated),
            "total_joints": len(names),
            "missing_joints": missing,
            "invalid_joints": invalid,
        }

    def _feedback_is_fresh(self) -> bool:
        self._poll_feedback()
        now = time.monotonic()
        return all(
            self._feedback[side] is not None
            and self._feedback_time[side] is not None
            and now - self._feedback_time[side] <= self._cfg.max_stale_time_s
            and np.all(self._feedback[side] >= self._cfg.out_min)
            and np.all(self._feedback[side] <= self._cfg.out_max)
            for side in self._active_sides
        )

    def set_enabled(self, enabled: bool) -> bool:
        with self._feedback_lock:
            return self._set_enabled_locked(enabled)

    def _set_enabled_locked(self, enabled: bool) -> bool:
        if not enabled:
            self._enabled = False
            return False
        if self._cfg.require_feedback_on_enable and not self._feedback_is_fresh():
            self._enabled = False
            return False
        for side in self._active_sides:
            if self._feedback[side] is not None:
                self._last_cmd[side] = self._feedback[side].copy()
        self._enabled = True
        return True

    def is_enabled(self) -> bool:
        return self._enabled

    def feedback_ready(self) -> bool:
        """Return whether every active hand has fresh, valid hardware feedback."""
        return self._feedback_is_fresh()

    def home(self) -> None:
        """Command the safe home/open pose."""
        left = np.asarray(self._cfg.left_open, dtype=np.float64)
        right = np.asarray(self._cfg.right_open, dtype=np.float64)
        self._send_physical(left, right)

    def return_to_open(self, cancelled=None) -> bool:
        """Move active hands from fresh feedback to their natural open poses."""
        cancelled = cancelled or (lambda: False)
        if not self._cfg.return_open_on_quit:
            self._enabled = False
            return True
        if cancelled() or not self._feedback_is_fresh():
            self._enabled = False
            return False

        starts = {
            side: np.asarray(self._feedback[side], dtype=np.float64).copy()
            for side in self._active_sides
        }
        targets = {
            "left": np.asarray(self._cfg.left_open, dtype=np.float64),
            "right": np.asarray(self._cfg.right_open, dtype=np.float64),
        }
        max_distance = max(
            float(np.max(np.abs(targets[side] - starts[side])))
            for side in self._active_sides
        )
        duration = max(0.1, float(self._cfg.return_open_duration_s))
        if self._cfg.max_velocity > 0:
            # Cubic smoothstep has peak speed 1.5 times its average speed.
            duration = max(duration, 1.5 * max_distance / float(self._cfg.max_velocity))
        interval = 1.0 / max(float(self._cfg.publish_hz), 1.0)
        steps = max(1, int(np.ceil(duration / interval)))

        left = self._last_cmd["left"].copy()
        right = self._last_cmd["right"].copy()
        for step in range(1, steps + 1):
            if cancelled() or not self._feedback_is_fresh():
                self._enabled = False
                return False
            phase = step / steps
            blend = phase * phase * (3.0 - 2.0 * phase)
            if "left" in starts:
                left = starts["left"] + blend * (targets["left"] - starts["left"])
            if "right" in starts:
                right = starts["right"] + blend * (targets["right"] - starts["right"])
            self._send_physical(left, right)
            if step < steps:
                time.sleep(interval)
        self._enabled = False
        return True

    def open_pose_status(self) -> tuple[bool, dict[str, np.ndarray]]:
        """Return whether every active physical joint is near its configured open zero."""
        if not self._feedback_is_fresh():
            return False, {}
        errors = {
            side: np.asarray(self._feedback[side], dtype=np.float64)
            - self.open_feedback_target(side)
            for side in self._active_sides
        }
        tolerance = max(0.0, float(self._cfg.startup_open_tolerance))
        return all(np.max(np.abs(error)) <= tolerance for error in errors.values()), errors

    def open_feedback_target(self, side: str) -> np.ndarray:
        """Return measured feedback expected at the physical natural-open pose."""
        if side not in self._active_sides:
            raise ValueError(f"{side!r} is not an active LinkerHand side")
        command_target = self._cfg.left_open if side == "left" else self._cfg.right_open
        feedback_target = (
            self._cfg.left_open_feedback
            if side == "left"
            else self._cfg.right_open_feedback
        )
        values = feedback_target if len(feedback_target) == self._physical_count else command_target
        return np.asarray(values, dtype=np.float64)

    def open_feedback_calibrated(self, side: str) -> bool:
        """Whether *side* has an independent, complete open-feedback zero."""
        if side not in self._active_sides:
            raise ValueError(f"{side!r} is not an active LinkerHand side")
        values = (
            self._cfg.left_open_feedback
            if side == "left"
            else self._cfg.right_open_feedback
        )
        try:
            target = np.asarray(values, dtype=np.float64).reshape(-1)
        except (TypeError, ValueError):
            return False
        return bool(
            target.shape == (self._physical_count,)
            and np.all(np.isfinite(target))
            and np.all(target >= self._cfg.out_min)
            and np.all(target <= self._cfg.out_max)
        )

    def align_and_verify_open_pose(self, cancelled=None) -> tuple[bool, dict[str, np.ndarray]]:
        """Smoothly command the configured open zero, then verify fresh feedback."""
        cancelled = cancelled or (lambda: False)
        if not self.return_to_open(cancelled):
            return False, {}
        deadline = time.monotonic() + max(
            0.1, float(self._cfg.startup_open_verify_timeout_s)
        )
        status: tuple[bool, dict[str, np.ndarray]] = (False, {})
        while time.monotonic() < deadline:
            if cancelled():
                return False, status[1]
            status = self.open_pose_status()
            if status[0]:
                return status
            time.sleep(0.02)
        return status

    def describe_open_pose_errors(self, errors: dict[str, np.ndarray]) -> str:
        """Describe only physical joints outside the configured zero tolerance."""
        tolerance = max(0.0, float(self._cfg.startup_open_tolerance))
        names = (
            L10_PHYSICAL_JOINT_NAMES
            if self._model == "L10"
            else PHYSICAL_JOINT_ORDER[:self._physical_count]
        )
        details = []
        for side, error in errors.items():
            target = self.open_feedback_target(side)
            current = target + np.asarray(error, dtype=np.float64)
            for index in np.flatnonzero(np.abs(error) > tolerance):
                details.append(
                    f"{side} {names[int(index)]}[{int(index)}] "
                    f"position={current[index]:.0f} target={target[index]:.0f} "
                    f"error={error[index]:+.0f}/255"
                )
        return "; ".join(details)

    # ------------------------------------------------------------------ map
    def _rad_to_servo(self, active_rad: np.ndarray, side: str) -> np.ndarray:
        active_rad = np.asarray(active_rad, dtype=np.float64).reshape(-1)
        if active_rad.shape[0] != 10:
            raise ValueError(f"expected 10 active joints per hand, got {active_rad.shape[0]}")
        if self._model == "L20":
            values = np.zeros(20, dtype=np.float64)
            for i, name in enumerate(ACTIVE_HAND_JOINTS):
                values[i] = active_rad[i]
            for follower, (source, mult) in MIMIC_JOINTS.items():
                src = ACTIVE_HAND_JOINTS.index(source)
                values[PHYSICAL_JOINT_ORDER.index(follower)] = active_rad[src] * mult
            servo = np.clip(values * self._scale + self._offset, 0.0, 255.0)
        else:
            upper = np.asarray(
                [ACTIVE_JOINT_LIMITS_RAD[name][1] for name in ACTIVE_HAND_JOINTS],
                dtype=np.float64,
            )
            normalized = np.clip(active_rad / upper, 0.0, 1.0)
            physical_normalized = normalized[L10_ACTIVE_INDEX_BY_PHYSICAL]
            open_pose = np.asarray(
                self._cfg.left_open if side == "left" else self._cfg.right_open,
                dtype=np.float64,
            )
            default_closed = self._cfg.left_closed if side == "left" else self._cfg.right_closed
            teleop_closed = (
                self._cfg.left_teleop_closed
                if side == "left"
                else self._cfg.right_teleop_closed
            )
            closed_pose = np.asarray(
                teleop_closed
                if len(teleop_closed) == self._physical_count
                else default_closed,
                dtype=np.float64,
            )
            servo = open_pose + physical_normalized * (closed_pose - open_pose)

            # Preserve unverified axes at measured positions.
            feedback = self._feedback.get(side)
            enabled = set(int(index) for index in self._cfg.enabled_physical_joints)
            if feedback is not None:
                for index in range(self._physical_count):
                    if index not in enabled:
                        servo[index] = feedback[index]
        return np.rint(servo).astype(np.int64)

    # -------------------------------------------------------------- safety
    def _apply_safety(self, side: str, servo: np.ndarray) -> np.ndarray:
        servo = np.clip(servo, self._cfg.out_min, self._cfg.out_max)
        last = self._last_cmd[side]
        if np.all(np.isnan(last)):
            # First command after enable: cap the jump from the home pose.
            base = self._home
        else:
            base = last
        delta = servo - base
        step = int(self._cfg.max_step)
        if step > 0:
            delta = np.clip(delta, -step, step)
        # Velocity limit (per publish interval).
        interval = 1.0 / max(float(self._cfg.publish_hz), 1.0)
        max_delta = int(max(self._cfg.max_velocity, 0) * interval)
        if max_delta > 0:
            delta = np.clip(delta, -max_delta, max_delta)
        out = base + delta
        out = np.clip(out, self._cfg.out_min, self._cfg.out_max)
        return np.rint(out).astype(np.int64)

    # ------------------------------------------------------------- output
    def _send_physical(self, left: np.ndarray, right: np.ndarray) -> None:
        left = np.clip(left, self._cfg.out_min, self._cfg.out_max).astype(np.int64).tolist()
        right = np.clip(right, self._cfg.out_min, self._cfg.out_max).astype(np.int64).tolist()
        self._last_cmd["left"] = np.asarray(left, dtype=np.float64)
        self._last_cmd["right"] = np.asarray(right, dtype=np.float64)
        if self._mode == "udp":
            commands = {"left": left, "right": right}
            payload = json.dumps(
                {side: commands[side] for side in self._active_sides}
            ).encode("utf-8")
            self._sock.sendto(payload, self._udp_addr)
        elif self._mode == "ros2":
            commands = {"left": left, "right": right}
            for side in self._active_sides:
                values = commands[side]
                msg = self._msg_type()
                msg.name = [f"{side}_{i}" for i in range(len(values))]
                msg.position = [float(v) for v in values]
                self._publishers[side].publish(msg)

    def command(self, hand_joints: np.ndarray) -> None:
        with self._feedback_lock:
            self._command_locked(hand_joints)

    def feedback_snapshot(self, side: str) -> dict:
        """Read-only copy for network telemetry; no invented feedback values."""
        if side not in self._active_sides:
            raise ValueError("inactive hand side")
        with self._feedback_lock:
            self._poll_feedback_locked()
            value, stamp = self._feedback[side], self._feedback_time[side]
            return {
                "position_unit": None if value is None else value.tolist(),
                "age_s": None if stamp is None else max(0.0, time.monotonic() - stamp),
                "geometry": self.geometry_calibration_status(side),
            }

    def command_physical(self, side: str, values) -> bool:
        """Already mapped L10 targets, still subject to local limits/feedback.

        Used by the robot link: retargeting and radians-to-servo mapping live
        on the laptop, while uncommissioned axes and rate limits remain local.
        """
        return self._command_physical_targets({side: values})

    def command_physical_batch(self, targets: dict) -> bool:
        """Validate and publish both hands once under the shared rate limiter."""
        if set(targets) != set(self._active_sides):
            raise ValueError("batch must contain every active hand")
        return self._command_physical_targets(targets)

    def _command_physical_targets(self, targets: dict) -> bool:
        checked = {}
        for side, values in targets.items():
            if side not in self._active_sides or self._physical_count != 10:
                raise ValueError("network physical command requires an active L10 hand")
            target = np.asarray(values, dtype=float)
            if (target.shape != (10,) or not np.all(np.isfinite(target))
                    or np.any(target < self._cfg.out_min) or np.any(target > self._cfg.out_max)):
                raise ValueError("invalid physical hand target")
            checked[side] = target.copy()
        with self._feedback_lock:
            if not self._enabled or not self._feedback_is_fresh():
                self._enabled = False
                return False
            now = time.monotonic()
            if now - self._last_sent_time < 1.0 / max(1.0, self._cfg.publish_hz):
                return True
            allowed = set(self._cfg.enabled_physical_joints)
            commands = {key: value.copy() for key, value in self._last_cmd.items()}
            for side, target in checked.items():
                for index in range(10):
                    if index not in allowed:
                        target[index] = self._feedback[side][index]
                commands[side] = self._apply_safety(side, target)
            self._send_physical(commands["left"], commands["right"])
            self._last_sent_time = now
            return True

    def _command_locked(self, hand_joints: np.ndarray) -> None:
        """Command both hands from the 20-value teleop packet (radians)."""
        if not self._enabled:
            return
        if self._cfg.require_feedback_on_enable and not self._feedback_is_fresh():
            self._enabled = False
            return
        hand_joints = np.asarray(hand_joints, dtype=np.float64).reshape(-1)
        if hand_joints.shape[0] != self._cfg.hand_joint_count:
            return
        left_active = hand_joints[0:10]
        right_active = hand_joints[10:20]
        if not np.all(np.isfinite(hand_joints)):
            return
        left_safe = self._last_cmd["left"]
        right_safe = self._last_cmd["right"]
        if "left" in self._active_sides:
            left_servo = self._rad_to_servo(left_active, "left")
            left_safe = self._apply_safety("left", left_servo)
        if "right" in self._active_sides:
            right_servo = self._rad_to_servo(right_active, "right")
            right_safe = self._apply_safety("right", right_servo)
        self._send_physical(left_safe, right_safe)

    def spin_once(self) -> None:
        if self._mode == "ros2" and self._node is not None:
            import rclpy
            rclpy.spin_once(self._node, timeout_sec=0.001)

    def close(self) -> None:
        if self._sock is not None:
            self._sock.close()
        if getattr(self, "_feedback_sock", None) is not None:
            self._feedback_sock.close()
        if self._node is not None:
            self._node.destroy_node()


__all__ = ["LinkerHandDriver", "ACTIVE_HAND_JOINTS", "ACTIVE_JOINT_LIMITS_RAD",
           "MIMIC_JOINTS", "PHYSICAL_JOINT_ORDER"]
