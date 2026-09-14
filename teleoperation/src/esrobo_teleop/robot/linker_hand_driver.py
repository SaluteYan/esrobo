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
            side = message.get("side")
            values = np.asarray(message.get("position", []), dtype=np.float64).reshape(-1)
            valid_range = np.all(
                (values >= self._cfg.out_min) & (values <= self._cfg.out_max)
            )
            if (
                side in self._feedback
                and values.shape == (self._physical_count,)
                and np.all(np.isfinite(values))
                and valid_range
            ):
                self._feedback[side] = values
                self._feedback_time[side] = time.monotonic()

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

    def return_to_open(self) -> bool:
        """Move active hands from fresh feedback to their natural open poses."""
        if not self._cfg.return_open_on_quit:
            self._enabled = False
            return True
        if not self._feedback_is_fresh():
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
        command_target = self._cfg.left_open if side == "left" else self._cfg.right_open
        feedback_target = (
            self._cfg.left_open_feedback
            if side == "left"
            else self._cfg.right_open_feedback
        )
        values = feedback_target if len(feedback_target) == self._physical_count else command_target
        return np.asarray(values, dtype=np.float64)

    def align_and_verify_open_pose(self) -> tuple[bool, dict[str, np.ndarray]]:
        """Smoothly command the configured open zero, then verify fresh feedback."""
        if not self.return_to_open():
            return False, {}
        deadline = time.monotonic() + max(
            0.1, float(self._cfg.startup_open_verify_timeout_s)
        )
        status: tuple[bool, dict[str, np.ndarray]] = (False, {})
        while time.monotonic() < deadline:
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
                    f"current={current[index]:.0f} target={target[index]:.0f} "
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
