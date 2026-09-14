#!/usr/bin/env python3
"""Bridge official SenseGlove ROS 2 state topics to ESROBO IsaacLab hand joints."""

from __future__ import annotations

import argparse
import copy
import json
import math
import os
import socket
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np


FINGERS = ("thumb", "index", "middle", "ring", "pinky")
PINCH_FINGERS = ("index",)
# Nova 2 exposes a five-finger SGCore hand model, but its pinky estimate follows
# the ring-finger observation. Do not independently normalize model-fit noise.
FINGER_INPUT_SOURCE = {"pinky": "ring"}
FLEX_SUFFIXES = ("mcp", "pip", "dip")
DEFAULT_LEFT_SERIAL = "01001"
DEFAULT_RIGHT_SERIAL = "01002"
DEFAULT_CALIBRATION_PATH = Path("config/senseglove_esrobo_calibration.json")

ESROBO_HAND_JOINT_ORDER = (
    "left_thumb_cmc_roll",
    "left_thumb_cmc_yaw",
    "left_thumb_cmc_pitch",
    "left_index_mcp_roll",
    "left_index_mcp_pitch",
    "left_middle_mcp_pitch",
    "left_ring_mcp_roll",
    "left_ring_mcp_pitch",
    "left_pinky_mcp_roll",
    "left_pinky_mcp_pitch",
    "right_thumb_cmc_roll",
    "right_thumb_cmc_yaw",
    "right_thumb_cmc_pitch",
    "right_index_mcp_roll",
    "right_index_mcp_pitch",
    "right_middle_mcp_pitch",
    "right_ring_mcp_roll",
    "right_ring_mcp_pitch",
    "right_pinky_mcp_roll",
    "right_pinky_mcp_pitch",
)

ESROBO_HAND_JOINT_LIMITS = {
    "left_thumb_cmc_roll": (0.0, 1.03),
    "left_thumb_cmc_yaw": (0.0, 1.40),
    "left_thumb_cmc_pitch": (0.0, 0.52),
    "left_index_mcp_roll": (0.0, 0.19),
    "left_index_mcp_pitch": (0.0, 1.36),
    "left_middle_mcp_pitch": (0.0, 1.36),
    "left_ring_mcp_roll": (0.0, 0.20),
    "left_ring_mcp_pitch": (0.0, 1.36),
    "left_pinky_mcp_roll": (0.0, 0.30),
    "left_pinky_mcp_pitch": (0.0, 1.36),
    "right_thumb_cmc_roll": (0.0, 1.03),
    "right_thumb_cmc_yaw": (0.0, 1.40),
    "right_thumb_cmc_pitch": (0.0, 0.52),
    "right_index_mcp_roll": (0.0, 0.19),
    "right_index_mcp_pitch": (0.0, 1.36),
    "right_middle_mcp_pitch": (0.0, 1.36),
    "right_ring_mcp_roll": (0.0, 0.20),
    "right_ring_mcp_pitch": (0.0, 1.36),
    "right_pinky_mcp_roll": (0.0, 0.30),
    "right_pinky_mcp_pitch": (0.0, 1.36),
}

@dataclass
class SideState:
    topic: str
    msg: Any | None = None
    stamp_monotonic: float | None = None
    features: dict[str, float] = field(default_factory=dict)
    imu_quat_wxyz: list[float] | None = None
    imu_raw_norm: float | None = None
    imu_w_repaired: bool = False
    imu_sample_count: int = 0
    imu_w_repaired_count: int = 0
    hand_positions_mm: list[list[float]] = field(default_factory=list)
    callback_sequence: int = 0
    connected: bool | None = None
    packets_per_second_received: int | None = None
    packets_per_second_sent: int | None = None
    battery_level: float | None = None
    charging: bool | None = None

    def has_live_sensor_payload(self) -> bool:
        """Reject the initialized all-zero frame published after an SDK read failure."""
        if self.connected is False:
            return False
        return len(self.hand_positions_mm) == 20 and any(
            abs(coordinate) > 1e-3
            for point in self.hand_positions_mm
            for coordinate in point
        )


class SenseGloveUdpBridge:
    """ROS subscriber state and SenseGlove-to-ESROBO hand retargeting."""

    def __init__(
        self, node: Any, msg_type: Any, args: argparse.Namespace, qos_profile: Any
    ):
        self.args = args
        self.left = SideState(args.left_topic)
        self.right = SideState(args.right_topic)
        self._sent_count = 0
        self._last_targets = [0.0] * len(ESROBO_HAND_JOINT_ORDER)

        if args.single_side:
            source = self.right if args.single_side == "right" else self.left
            mirror = self.left if args.single_side == "right" else self.right
            source_callback = self._make_callback(source)
            mirror_callback = self._make_callback(mirror)

            def single_side_callback(msg: Any) -> None:
                source_callback(msg)
                mirror_callback(msg)

            node.create_subscription(
                msg_type, source.topic, single_side_callback, qos_profile
            )
        else:
            node.create_subscription(
                msg_type, args.left_topic, self._make_callback(self.left), qos_profile
            )
            node.create_subscription(
                msg_type, args.right_topic, self._make_callback(self.right), qos_profile
            )

    def _make_callback(self, state: SideState):
        def callback(msg: Any) -> None:
            state.msg = msg
            state.stamp_monotonic = time.monotonic()
            state.features = extract_features(msg)
            imu_diagnostics: dict[str, Any] = {}
            state.imu_quat_wxyz = extract_imu_quat_wxyz(
                msg,
                self.args.imu_correction,
                self.args.imu_w_repair,
                imu_diagnostics,
                previous_quat_wxyz=state.imu_quat_wxyz,
            )
            state.imu_raw_norm = imu_diagnostics.get("raw_norm")
            state.imu_w_repaired = bool(imu_diagnostics.get("w_repaired", False))
            if state.imu_quat_wxyz is not None:
                state.imu_sample_count += 1
                if state.imu_w_repaired:
                    state.imu_w_repaired_count += 1
            state.hand_positions_mm = extract_hand_positions_mm(msg)
            state.connected = (
                bool(msg.connected) if hasattr(msg, "connected") else None
            )
            state.packets_per_second_received = (
                int(msg.packets_per_second_received)
                if hasattr(msg, "packets_per_second_received")
                else None
            )
            state.packets_per_second_sent = (
                int(msg.packets_per_second_sent)
                if hasattr(msg, "packets_per_second_sent")
                else None
            )
            state.battery_level = (
                float(msg.battery_level)
                if getattr(msg, "battery_level_valid", False)
                else None
            )
            state.charging = bool(msg.charging) if hasattr(msg, "charging") else None
            state.features.update(extract_hand_vector_features(state.hand_positions_mm))
            state.callback_sequence += 1

        return callback

    def has_both_hands(self, max_age_s: float) -> bool:
        return self._is_fresh(self.left, max_age_s) and self._is_fresh(
            self.right, max_age_s
        )

    def _is_fresh(self, state: SideState, max_age_s: float) -> bool:
        if state.msg is None or state.stamp_monotonic is None:
            return False
        if time.monotonic() - state.stamp_monotonic > max_age_s:
            return False
        return state.has_live_sensor_payload()

    def topic_status(self) -> str:
        return (
            f"left={self._age_text(self.left)}/points={len(self.left.hand_positions_mm)}"
            f"/payload={self._payload_status(self.left)}"
            f"/{self._device_status(self.left)}"
            f"/imu={self._imu_status(self.left)}({self.left.topic}) "
            f"right={self._age_text(self.right)}/points={len(self.right.hand_positions_mm)}"
            f"/payload={self._payload_status(self.right)}"
            f"/{self._device_status(self.right)}"
            f"/imu={self._imu_status(self.right)}({self.right.topic})"
        )

    def active_states(self) -> tuple[SideState, ...]:
        if self.args.single_side == "left":
            return (self.left,)
        if self.args.single_side == "right":
            return (self.right,)
        return (self.left, self.right)

    def has_explicit_disconnect(self) -> bool:
        return any(state.connected is False for state in self.active_states())

    @staticmethod
    def _device_status(state: SideState) -> str:
        if state.connected is None:
            return "connection=legacy"
        battery = (
            "unknown" if state.battery_level is None else f"{state.battery_level:.0f}%"
        )
        return (
            f"connected={state.connected}/rx={state.packets_per_second_received}pps"
            f"/tx={state.packets_per_second_sent}pps/battery={battery}"
            f"/charging={state.charging}"
        )

    @staticmethod
    def _payload_status(state: SideState) -> str:
        if state.msg is None:
            return "none"
        return "live" if state.has_live_sensor_payload() else "all-zero/invalid"

    def _age_text(self, state: SideState) -> str:
        if state.stamp_monotonic is None:
            return "none"
        return f"{time.monotonic() - state.stamp_monotonic:.3f}s"

    @staticmethod
    def _imu_status(state: SideState) -> str:
        if state.imu_raw_norm is None:
            return "none"
        suffix = "/w-repaired" if state.imu_w_repaired else ""
        return f"norm={state.imu_raw_norm:.3f}{suffix}"

    def build_targets(
        self, calibration: dict[str, Any]
    ) -> tuple[list[float], dict[str, dict[str, float]]]:
        source_left = self.right if self.args.swap_left_right_targets else self.left
        source_right = self.left if self.args.swap_left_right_targets else self.right
        left_targets, left_curls = retarget_side(
            source_left.features,
            calibration["open"][
                "right" if self.args.swap_left_right_targets else "left"
            ],
            calibration["closed"][
                "right" if self.args.swap_left_right_targets else "left"
            ],
            pinch_references_for_side(
                calibration,
                "right" if self.args.swap_left_right_targets else "left",
            ),
            "left",
            self.args,
        )
        right_targets, right_curls = retarget_side(
            source_right.features,
            calibration["open"][
                "left" if self.args.swap_left_right_targets else "right"
            ],
            calibration["closed"][
                "left" if self.args.swap_left_right_targets else "right"
            ],
            pinch_references_for_side(
                calibration,
                "left" if self.args.swap_left_right_targets else "right",
            ),
            "right",
            self.args,
        )
        targets = left_targets + right_targets
        if self.args.smoothing_alpha < 1.0:
            alpha = max(0.0, self.args.smoothing_alpha)
            targets = [
                alpha * current + (1.0 - alpha) * previous
                for current, previous in zip(targets, self._last_targets)
            ]
        self._last_targets = targets
        return targets, {"left": left_curls, "right": right_curls}

    def build_orientation_deltas(
        self, calibration: dict[str, Any]
    ) -> dict[str, list[float]]:
        if self.args.disable_imu_orientation:
            return {}

        imu_neutral = calibration.get("imu_neutral")
        if not isinstance(imu_neutral, dict):
            return {}

        source_left = self.right if self.args.swap_left_right_targets else self.left
        source_right = self.left if self.args.swap_left_right_targets else self.right
        left_key = "right" if self.args.swap_left_right_targets else "left"
        right_key = "left" if self.args.swap_left_right_targets else "right"

        frame_corrections = calibration.get("imu_frame_correction", {})
        if not isinstance(frame_corrections, dict):
            frame_corrections = {}
        axis_decoupling = calibration.get("imu_axis_decoupling", {})
        if not isinstance(axis_decoupling, dict):
            axis_decoupling = {}
        reconstructed_w_branches = calibration.get("imu_reconstructed_w_opposite", {})
        if not isinstance(reconstructed_w_branches, dict):
            reconstructed_w_branches = {}

        left_current = source_left.imu_quat_wxyz
        right_current = source_right.imu_quat_wxyz
        left_neutral = imu_neutral.get(left_key)
        right_neutral = imu_neutral.get(right_key)
        if reconstructed_w_branches.get(left_key) is True:
            left_current = opposite_reconstructed_w_branch_wxyz(
                left_current, self.args.imu_correction
            )
            left_neutral = opposite_reconstructed_w_branch_wxyz(
                left_neutral, self.args.imu_correction
            )
        if reconstructed_w_branches.get(right_key) is True:
            right_current = opposite_reconstructed_w_branch_wxyz(
                right_current, self.args.imu_correction
            )
            right_neutral = opposite_reconstructed_w_branch_wxyz(
                right_neutral, self.args.imu_correction
            )

        calibration_mode = calibration.get("imu_calibration_mode")
        left_delta = orientation_delta_wxyz(
            left_current,
            left_neutral,
            frame_corrections.get(left_key),
            runtime_axis_decoupling(
                left_key, axis_decoupling.get(left_key), calibration_mode
            ),
        )
        right_delta = orientation_delta_wxyz(
            right_current,
            right_neutral,
            frame_corrections.get(right_key),
            runtime_axis_decoupling(
                right_key, axis_decoupling.get(right_key), calibration_mode
            ),
        )
        if left_delta is None or right_delta is None:
            return {}
        return {"left": left_delta, "right": right_delta}

    def seed_imu_continuity_from_calibration(self, calibration: dict[str, Any]) -> None:
        """Use the persisted neutral to select the reconstructed-w branch after restart."""
        imu_neutral = calibration.get("imu_neutral")
        if not isinstance(imu_neutral, dict):
            return
        for side, state in (("left", self.left), ("right", self.right)):
            neutral = imu_neutral.get(side)
            if not isinstance(neutral, list) or len(neutral) != 4:
                continue
            state.imu_quat_wxyz = normalize_quat_wxyz(neutral)

    def send(
        self,
        sock: socket.socket,
        calibration: dict[str, Any],
    ) -> tuple[
        list[float],
        dict[str, dict[str, float]],
        dict[str, list[float]],
        dict[str, Any],
    ]:
        adapt_flexion_calibration_envelope(self, calibration)
        targets, curls = self.build_targets(calibration)
        orientation_deltas = self.build_orientation_deltas(calibration)
        packet = {
            "timestamp": time.time(),
            "source": "senseglove_ros",
            "type": "esrobo_hand_joints",
            "hand_joint_order": list(ESROBO_HAND_JOINT_ORDER),
            "hand_joints": targets,
            "retargeting": {
                "method": (
                    "sgcore_joint_decomposition_and_hand_vectors_to_esrobo_10_active_joints_per_hand"
                    if self.args.retargeting_mode == "vector"
                    else "calibrated_finger_curl_to_esrobo_active_hand_joints"
                ),
                "robot_left_source": (
                    "right_glove" if self.args.swap_left_right_targets else "left_glove"
                ),
                "robot_right_source": (
                    "left_glove" if self.args.swap_left_right_targets else "right_glove"
                ),
            },
            "curls": curls,
        }
        source_left = self.right if self.args.swap_left_right_targets else self.left
        source_right = self.left if self.args.swap_left_right_targets else self.right
        if (
            len(source_left.hand_positions_mm) == 20
            and len(source_right.hand_positions_mm) == 20
        ):
            packet["hand_skeleton_positions"] = {
                "left": canonicalize_hand_points_for_robot(
                    source_left.hand_positions_mm, "left"
                ),
                "right": canonicalize_hand_points_for_robot(
                    source_right.hand_positions_mm, "right"
                ),
            }
            packet["hand_skeleton_layout"] = (
                "finger_major_5x4_thumb_to_pinky_proximal_to_distal"
            )
            packet["hand_skeleton_unit"] = "mm"
            packet["hand_skeleton_frame"] = "robot_hand_local"
            packet["hand_skeleton_source"] = (
                "senseglove_ros/SenseGloveState.hand_position"
            )
        if orientation_deltas:
            packet["hand_orientation_deltas"] = orientation_deltas
            packet["hand_orientation_delta_order"] = "wxyz"
            packet["hand_orientation_source_frame"] = "senseglove_zeroed_anatomical_axes"
            packet["hand_orientation_retargeting"] = {
                "method": "multi_pose_frame_corrected_imu_delta",
                "neutral": "calibration_imu_neutral",
                "frame_correction": "calibration_imu_frame_correction",
                "robot_left_source": (
                    "right_glove" if self.args.swap_left_right_targets else "left_glove"
                ),
                "robot_right_source": (
                    "left_glove" if self.args.swap_left_right_targets else "right_glove"
                ),
            }
        encoded = json.dumps(packet, separators=(",", ":")).encode("utf-8")
        sock.sendto(encoded, (self.args.host, self.args.port))
        self._sent_count += 1
        return targets, curls, orientation_deltas, packet

    def raw_recording_snapshot(self) -> dict[str, Any]:
        """Capture both physical glove messages that produced the current UDP target."""
        return {
            "left": side_recording_snapshot(self.left),
            "right": side_recording_snapshot(self.right),
        }


class HandRecordingWriter:
    """Write replay-compatible hand packets plus same-frame raw SenseGlove data."""

    def __init__(
        self,
        path: Path,
        args: argparse.Namespace,
        calibration: dict[str, Any],
    ):
        self.path = path.expanduser()
        if self.path.exists() and not args.record_overwrite:
            raise FileExistsError(
                f"recording already exists: {self.path}; use --record-overwrite to replace it"
            )
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.stream = self.path.open("w", encoding="utf-8", buffering=1)
        self.start_delay_s = args.record_start_delay_s
        self.duration_s = args.record_duration_s
        self.armed_monotonic: float | None = None
        self.started_monotonic: float | None = None
        self.sequence = 0
        self.complete = False
        self._last_countdown: int | None = None
        header = {
            "type": "esrobo_hand_recording_header",
            "format_version": 2,
            "created_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "source": "real_senseglove_ros",
            "left_serial": args.left_serial,
            "right_serial": args.right_serial,
            "left_topic": args.left_topic,
            "right_topic": args.right_topic,
            "nominal_rate_hz": args.rate_hz,
            "record_start_delay_s": self.start_delay_s,
            "record_duration_s": self.duration_s,
            "imu_orientation_included": not args.disable_imu_orientation,
            "imu_correction": args.imu_correction,
            "imu_w_repair": args.imu_w_repair,
            "retargeting_mode": args.retargeting_mode,
            "hand_joint_order": list(ESROBO_HAND_JOINT_ORDER),
            "raw_fields": {
                "joint_position": "SenseGloveState.position as published",
                "joint_absolute_velocity": "SenseGloveState.absolute_velocity as published",
                "hand_position": "millimeters, physical glove wrist-local frame",
                "finger_tip_position": "millimeters, physical glove wrist-local frame",
                "imu_orientation_raw": "xyzw, exactly as published",
                "imu_orientation_corrected": "wxyz, repaired/normalized and coordinate-corrected",
            },
            "calibration": calibration,
        }
        self.stream.write(json.dumps(header, separators=(",", ":")) + "\n")

    def update(
        self,
        now: float,
        packet: dict[str, Any],
        raw_snapshot: dict[str, Any],
    ) -> str:
        if self.complete:
            return "complete"
        if self.armed_monotonic is None:
            self.armed_monotonic = now
            print(
                "[senseglove_bridge] RECORDING ARMED: keep both hands still for "
                f"{self.start_delay_s:.1f}s; teleoperation is already active.",
                flush=True,
            )

        delay_elapsed = now - self.armed_monotonic
        if delay_elapsed < self.start_delay_s:
            remaining = max(self.start_delay_s - delay_elapsed, 0.0)
            countdown = max(1, math.ceil(remaining))
            if countdown != self._last_countdown:
                print(
                    f"[senseglove_bridge] Recording starts in {countdown}s; hold still.",
                    flush=True,
                )
                self._last_countdown = countdown
            return "countdown"

        if self.started_monotonic is None:
            self.started_monotonic = now
            print(
                f"[senseglove_bridge] RECORDING ACTIVE: move now; saving to {self.path}.",
                flush=True,
            )

        t_rel_s = now - self.started_monotonic
        if self.duration_s > 0.0 and t_rel_s > self.duration_s:
            self.complete = True
            self.stream.flush()
            print(
                "[senseglove_bridge] RECORDING COMPLETE: "
                f"saved={self.path} frames={self.sequence} duration={self.duration_s:.3f}s.",
                flush=True,
            )
            return "complete"

        item = {
            "type": "esrobo_hand_packet",
            "record_sequence": self.sequence,
            "t_rel_s": t_rel_s,
            "record_phase": "motion",
            "packet": packet,
            "senseglove_raw": raw_snapshot,
        }
        self.stream.write(json.dumps(item, separators=(",", ":")) + "\n")
        self.sequence += 1
        return "recording"

    def close(self) -> None:
        if not self.stream.closed:
            self.stream.flush()
            self.stream.close()


def parse_args(argv: list[str]) -> argparse.Namespace:
    env_swap = _env_bool(
        "ESROBO_HAND_SWAP_LEFT_RIGHT_TARGETS",
        _env_bool("ESROBO_BODY_SWAP_LEFT_RIGHT_TARGETS", False),
    )
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--host", default=os.environ.get("ESROBO_BODY_UDP_HOST", "127.0.0.1")
    )
    parser.add_argument(
        "--port", type=int, default=int(os.environ.get("ESROBO_BODY_UDP_PORT", "15050"))
    )
    parser.add_argument(
        "--left-serial",
        default=os.environ.get("SENSEGLOVE_LEFT_SERIAL", DEFAULT_LEFT_SERIAL),
    )
    parser.add_argument(
        "--right-serial",
        default=os.environ.get("SENSEGLOVE_RIGHT_SERIAL", DEFAULT_RIGHT_SERIAL),
    )
    parser.add_argument("--left-topic", default=None)
    parser.add_argument("--right-topic", default=None)
    parser.add_argument("--single-side", choices=("left", "right"), default=None)
    parser.add_argument(
        "--rate-hz",
        type=float,
        default=float(os.environ.get("SENSEGLOVE_BRIDGE_RATE_HZ", "60")),
    )
    parser.add_argument("--qos-depth", type=int, default=5)
    parser.add_argument("--max-state-age", type=float, default=0.5)
    parser.add_argument("--print-interval", type=float, default=1.0)
    parser.add_argument("--print-raw", action="store_true")
    parser.add_argument(
        "--record-path",
        type=Path,
        default=None,
        help="Write replay-compatible JSONL with raw finger, skeleton, and IMU samples.",
    )
    parser.add_argument(
        "--record-start-delay-s",
        type=float,
        default=3.0,
        help="Keep streaming but wait this long after fresh data before recording.",
    )
    parser.add_argument(
        "--record-duration-s",
        type=float,
        default=0.0,
        help="Stop recording after this duration; 0 records until Ctrl-C.",
    )
    parser.add_argument(
        "--record-overwrite",
        action="store_true",
        help="Replace an existing --record-path instead of refusing to start.",
    )
    parser.add_argument(
        "--calibration-file", type=Path, default=DEFAULT_CALIBRATION_PATH
    )
    parser.add_argument("--recalibrate", action="store_true")
    parser.add_argument("--calibrate-only", action="store_true",
                        help="collect finger endpoints and multi-pose IMU frame calibration, save it, then exit "
                             "(used by the one-command launcher).")
    parser.add_argument("--calibration-duration", type=float, default=10.0)
    parser.add_argument("--calibration-sample-start", type=float, default=6.0)
    parser.add_argument(
        "--calibration-method", choices=("last", "average"), default="average"
    )
    parser.add_argument(
        "--imu-calibration-mode",
        choices=("static-orthogonal", "static-linear"),
        default="static-orthogonal",
        help=(
            "static-orthogonal fits one rotation from held poses (default); "
            "static-linear also applies the preserved linear-decoupling matrix"
        ),
    )
    parser.add_argument(
        "--no-prompt",
        action="store_true",
        help="Start each calibration phase without waiting for ENTER.",
    )
    parser.add_argument(
        "--disable-imu-orientation",
        action="store_true",
        help="Send only finger joints; do not send calibrated SenseGlove IMU hand orientation.",
    )
    parser.add_argument(
        "--imu-correction",
        choices=("unity_to_ros", "raw"),
        default=os.environ.get("SENSEGLOVE_IMU_CORRECTION", "unity_to_ros"),
        help="Coordinate correction for SenseGlove imu_orientation before calibration.",
    )
    parser.add_argument(
        "--imu-w-repair",
        choices=("auto", "off", "reconstruct"),
        default=os.environ.get("SENSEGLOVE_IMU_W_REPAIR", "auto"),
        help=(
            "Repair the observed Nova 2/SGCore fault where quaternion w duplicates z. "
            "Auto repairs every duplicated sample and keeps the reconstructed w branch continuous."
        ),
    )
    parser.add_argument(
        "--smoothing-alpha",
        type=float,
        default=1.0,
        help="1.0 disables smoothing; lower values smooth more.",
    )
    parser.add_argument(
        "--retargeting-mode",
        choices=("vector", "curl"),
        default=os.environ.get("ESROBO_HAND_RETARGETING_MODE", "vector"),
        help="Use 5x4 hand-segment vectors (default) or the legacy averaged joint-curl mapping.",
    )
    parser.add_argument(
        "--spread-scale",
        type=float,
        default=None,
        help="Scale finger-vector spread into ESROBO roll joints; defaults to 1 for vector mode and 0 for curl mode.",
    )
    parser.add_argument("--vector-spread-range-deg", type=float, default=18.0)
    # Keep side-specific switches for hardware tuning, but use the same default
    # response on both hands so calibration has identical semantics.
    parser.add_argument("--thumb-roll-gain", type=float, default=0.85)
    parser.add_argument("--thumb-yaw-gain", type=float, default=0.45)
    parser.add_argument("--thumb-flexion-gain", type=float, default=1.00)
    parser.add_argument("--thumb-flexion-exponent", type=float, default=1.00)
    parser.add_argument("--finger-flexion-gain", type=float, default=1.00)
    parser.add_argument("--finger-flexion-exponent", type=float, default=1.00)
    parser.add_argument("--right-thumb-roll-gain", type=float, default=0.85)
    parser.add_argument("--right-thumb-yaw-gain", type=float, default=0.45)
    parser.add_argument("--right-thumb-flexion-gain", type=float, default=1.00)
    parser.add_argument("--right-thumb-flexion-exponent", type=float, default=1.00)
    parser.add_argument("--right-finger-flexion-gain", type=float, default=1.00)
    parser.add_argument("--right-finger-flexion-exponent", type=float, default=1.00)
    parser.add_argument(
        "--visualize-right-hand",
        action="store_true",
        help="Serve a live browser view of the right skeleton and thumb channels.",
    )
    parser.add_argument("--visualizer-rate-hz", type=float, default=15.0)
    parser.add_argument("--visualizer-host", default="0.0.0.0")
    parser.add_argument("--visualizer-port", type=int, default=8766)
    parser.add_argument(
        "--swap-left-right-targets",
        dest="swap_left_right_targets",
        action="store_true",
        default=env_swap,
    )
    parser.add_argument(
        "--no-swap-left-right-targets",
        dest="swap_left_right_targets",
        action="store_false",
    )
    args = parser.parse_args(argv)
    args.left_topic = (
        args.left_topic or f"/senseglove/glove{args.left_serial}/lh/senseglove_states"
    )
    args.right_topic = (
        args.right_topic or f"/senseglove/glove{args.right_serial}/rh/senseglove_states"
    )
    args.rate_hz = max(args.rate_hz, 1.0)
    args.calibration_duration = max(args.calibration_duration, 0.1)
    args.calibration_sample_start = min(
        max(args.calibration_sample_start, 0.0), args.calibration_duration
    )
    args.smoothing_alpha = min(max(args.smoothing_alpha, 0.0), 1.0)
    args.spread_scale = (
        (1.0 if args.retargeting_mode == "vector" else 0.0)
        if args.spread_scale is None
        else args.spread_scale
    )
    args.vector_spread_range_deg = max(args.vector_spread_range_deg, 1.0)
    args.thumb_flexion_exponent = min(max(args.thumb_flexion_exponent, 0.35), 1.5)
    args.thumb_flexion_gain = min(max(args.thumb_flexion_gain, 0.5), 2.0)
    args.thumb_roll_gain = min(max(args.thumb_roll_gain, 0.0), 2.0)
    args.thumb_yaw_gain = min(max(args.thumb_yaw_gain, 0.0), 2.0)
    args.finger_flexion_gain = min(max(args.finger_flexion_gain, 0.5), 2.0)
    args.finger_flexion_exponent = min(max(args.finger_flexion_exponent, 0.35), 1.5)
    args.right_thumb_roll_gain = min(max(args.right_thumb_roll_gain, 0.0), 2.0)
    args.right_thumb_yaw_gain = min(max(args.right_thumb_yaw_gain, 0.0), 2.0)
    args.right_thumb_flexion_gain = min(max(args.right_thumb_flexion_gain, 0.5), 2.0)
    args.right_thumb_flexion_exponent = min(
        max(args.right_thumb_flexion_exponent, 0.35), 1.5
    )
    args.right_finger_flexion_gain = min(
        max(args.right_finger_flexion_gain, 0.5), 2.0
    )
    args.right_finger_flexion_exponent = min(
        max(args.right_finger_flexion_exponent, 0.35), 1.5
    )
    args.visualizer_rate_hz = min(max(args.visualizer_rate_hz, 1.0), 30.0)
    args.visualizer_port = min(max(args.visualizer_port, 1), 65535)
    args.record_start_delay_s = max(args.record_start_delay_s, 0.0)
    args.record_duration_s = max(args.record_duration_s, 0.0)
    return args


def _env_bool(name: str, default: bool) -> bool:
    raw = os.environ.get(name)
    if raw is None:
        return default
    return raw.strip().lower() not in ("", "0", "false", "no", "off")


def extract_features(msg: Any) -> dict[str, float]:
    names = list(getattr(msg, "joint_names", []))
    positions = list(getattr(msg, "position", []))
    joints: dict[str, float] = {}
    for name, position in zip(names, positions):
        try:
            value = float(position)
        except (TypeError, ValueError):
            continue
        if math.isfinite(value):
            joints[_normalize_senseglove_joint_name(name)] = value

    features: dict[str, float] = {}
    for finger in FINGERS:
        flex_values = [
            joints[f"{finger}_{suffix}"]
            for suffix in FLEX_SUFFIXES
            if f"{finger}_{suffix}" in joints
        ]
        features[finger] = _mean(flex_values) if flex_values else 0.0
        for suffix in FLEX_SUFFIXES:
            joint_name = f"{finger}_{suffix}"
            if joint_name in joints:
                features[joint_name] = joints[joint_name]
        brake_name = f"{finger}_brake"
        features[brake_name] = joints.get(brake_name, 0.0)
    return features


def side_recording_snapshot(state: SideState) -> dict[str, Any]:
    """Serialize the original ROS message and the corrected values used by retargeting."""
    msg = state.msg
    if msg is None:
        return {}
    orientation = getattr(msg, "imu_orientation", None)
    raw_imu_xyzw = (
        [
            _finite_float(getattr(orientation, axis, None))
            for axis in ("x", "y", "z", "w")
        ]
        if orientation is not None
        else None
    )
    return {
        "topic": state.topic,
        "callback_sequence": state.callback_sequence,
        "received_monotonic_ns": (
            int(state.stamp_monotonic * 1.0e9)
            if state.stamp_monotonic is not None
            else None
        ),
        "ros_stamp": _ros_stamp_dict(msg),
        "joint_names": [str(value) for value in getattr(msg, "joint_names", [])],
        "position": _finite_float_list(getattr(msg, "position", [])),
        "absolute_velocity": _finite_float_list(getattr(msg, "absolute_velocity", [])),
        "hand_position_mm_raw": _point_list(getattr(msg, "hand_position", [])),
        "finger_tip_position_mm_raw": _point_list(
            getattr(msg, "finger_tip_position", [])
        ),
        "hand_position_mm_used": [list(point) for point in state.hand_positions_mm],
        "imu_orientation_raw_xyzw": raw_imu_xyzw,
        "imu_raw_norm": state.imu_raw_norm,
        "imu_w_repaired": state.imu_w_repaired,
        "imu_orientation_corrected_wxyz": (
            list(state.imu_quat_wxyz) if state.imu_quat_wxyz is not None else None
        ),
        "retargeting_features": dict(state.features),
    }


def _finite_float(value: Any) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _finite_float_list(values: Any) -> list[float | None]:
    return [_finite_float(value) for value in list(values)]


def _point_list(points: Any) -> list[list[float | None]]:
    return [
        [_finite_float(getattr(point, axis, None)) for axis in ("x", "y", "z")]
        for point in list(points)
    ]


def _ros_stamp_dict(msg: Any) -> dict[str, int] | None:
    header = getattr(msg, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return None
    try:
        return {"sec": int(stamp.sec), "nanosec": int(stamp.nanosec)}
    except (AttributeError, TypeError, ValueError):
        return None


def extract_imu_quat_wxyz(
    msg: Any,
    correction: str,
    imu_w_repair: str = "auto",
    diagnostics: dict[str, Any] | None = None,
    previous_quat_wxyz: list[float] | None = None,
) -> list[float] | None:
    orientation = getattr(msg, "imu_orientation", None)
    if orientation is None:
        return None
    raw_xyzw = [
        getattr(orientation, "x", 0.0),
        getattr(orientation, "y", 0.0),
        getattr(orientation, "z", 0.0),
        getattr(orientation, "w", 1.0),
    ]
    raw_xyzw, raw_norm, w_repaired = repair_senseglove_imu_w(raw_xyzw, imu_w_repair)
    if diagnostics is not None:
        diagnostics["raw_norm"] = raw_norm
        diagnostics["w_repaired"] = w_repaired
    raw_candidates = [raw_xyzw]
    if w_repaired and abs(raw_xyzw[3]) > 1.0e-8:
        opposite_w = list(raw_xyzw)
        opposite_w[3] = -opposite_w[3]
        raw_candidates.append(opposite_w)

    candidates: list[list[float]] = []
    for candidate in raw_candidates:
        corrected_xyzw = correct_imu_quat_xyzw(candidate, correction)
        if corrected_xyzw is None:
            continue
        x, y, z, w = corrected_xyzw
        candidates.append(normalize_quat_wxyz([w, x, y, z]))
    if not candidates:
        return None
    if previous_quat_wxyz is None or len(candidates) == 1:
        return candidates[0]

    previous = normalize_quat_wxyz(previous_quat_wxyz)
    return max(candidates, key=lambda candidate: abs(quat_dot_wxyz(candidate, previous)))


def repair_senseglove_imu_w(
    raw_xyzw: list[float], mode: str
) -> tuple[list[float], float | None, bool]:
    """Repair Nova 2 samples where SGCore returns the z component as w."""
    try:
        values = [float(value) for value in raw_xyzw]
    except (TypeError, ValueError):
        return raw_xyzw, None, False
    if len(values) != 4 or not all(math.isfinite(value) for value in values):
        return values, None, False

    x, y, z, w = values
    raw_norm = math.sqrt(sum(value * value for value in values))
    xyz_norm_sq = x * x + y * y + z * z
    duplicated_w = math.isclose(w, z, rel_tol=1.0e-5, abs_tol=1.0e-5)
    should_repair = mode == "reconstruct" or (mode == "auto" and duplicated_w)
    if not should_repair or xyz_norm_sq > 1.02:
        return values, raw_norm, False

    # The broken field duplicates z, so its sign contains no information about
    # the missing scalar component.  Start in the canonical positive-w
    # hemisphere; extract_imu_quat_wxyz() uses frame continuity to select the
    # opposite branch later if the real motion crosses that hemisphere.
    repaired_w = math.sqrt(max(0.0, 1.0 - min(xyz_norm_sq, 1.0)))
    return [x, y, z, repaired_w], raw_norm, True


def extract_hand_positions_mm(msg: Any) -> list[list[float]]:
    """Extract SGCore's 5x4 hand joints, which are millimeters relative to the wrist."""
    positions: list[list[float]] = []
    for point in list(getattr(msg, "hand_position", [])):
        xyz = [float(getattr(point, axis, float("nan"))) for axis in ("x", "y", "z")]
        if not all(math.isfinite(value) for value in xyz):
            return []
        positions.append(xyz)
    if len(positions) != 20:
        return []

    # Prefer the separately published SGCore fingertips when they are geometrically
    # plausible. This also repairs recordings made with the older 19-point guard,
    # where the fourth pinky point was left at zero.
    fingertips = list(getattr(msg, "finger_tip_position", []))
    if len(fingertips) == 5:
        for finger_index, point in enumerate(fingertips):
            xyz = [
                float(getattr(point, axis, float("nan"))) for axis in ("x", "y", "z")
            ]
            distal = positions[finger_index * 4 + 2]
            distal_to_tip = math.dist(distal, xyz)
            if (
                all(math.isfinite(value) for value in xyz)
                and 1.0 <= distal_to_tip <= 120.0
            ):
                positions[finger_index * 4 + 3] = xyz
    return positions


def extract_hand_vector_features(
    hand_positions_mm: list[list[float]],
) -> dict[str, float]:
    """Describe a hand using palm-relative segment directions, independent of hand size."""
    if len(hand_positions_mm) != 20:
        return {}
    chains = {
        finger: [
            [float(value) for value in point]
            for point in hand_positions_mm[index * 4 : index * 4 + 4]
        ]
        for index, finger in enumerate(FINGERS)
    }
    lateral = _vector_normalize(
        _vector_subtract(chains["index"][0], chains["pinky"][0])
    )
    forward = _vector_normalize(chains["middle"][0])
    if lateral is None or forward is None:
        return {}
    normal = _vector_normalize(_vector_cross(lateral, forward))
    if normal is None:
        return {}
    forward = _vector_normalize(_vector_cross(normal, lateral))
    if forward is None:
        return {}

    features: dict[str, float] = {}
    for finger, points in chains.items():
        segments = [
            _vector_normalize(_vector_subtract(points[index + 1], points[index]))
            for index in range(3)
        ]
        if any(segment is None for segment in segments):
            return {}
        first, second, third = segments
        base_flex = math.atan2(_vector_dot(first, normal), _vector_dot(first, forward))
        pip_flex = _unsigned_vector_angle(first, second)
        dip_flex = _unsigned_vector_angle(second, third)
        features[f"vector_{finger}_curl"] = base_flex + pip_flex + dip_flex
        features[f"vector_{finger}_spread"] = math.atan2(
            _vector_dot(first, lateral),
            _vector_dot(first, forward),
        )
        features[f"vector_{finger}_elevation"] = math.atan2(
            _vector_dot(first, normal),
            math.hypot(_vector_dot(first, forward), _vector_dot(first, lateral)),
        )

    # The thumb CMC is upstream of the four SGCore points. Its motion is visible
    # in the wrist-to-tip direction but is missed by angles between local segments.
    thumb_points = chains["thumb"]
    thumb_tip = _vector_normalize(thumb_points[-1])
    if thumb_tip is None:
        return {}
    thumb_segments = [
        _vector_normalize(
            _vector_subtract(thumb_points[index + 1], thumb_points[index])
        )
        for index in range(3)
    ]
    if any(segment is None for segment in thumb_segments):
        return {}
    features["vector_thumb_internal_curl"] = sum(
        _unsigned_vector_angle(thumb_segments[index], thumb_segments[index + 1])
        for index in range(2)
    )
    features["vector_thumb_tip_spread"] = math.atan2(
        _vector_dot(thumb_tip, lateral), _vector_dot(thumb_tip, forward)
    )
    features["vector_thumb_tip_elevation"] = math.atan2(
        _vector_dot(thumb_tip, normal),
        math.hypot(_vector_dot(thumb_tip, forward), _vector_dot(thumb_tip, lateral)),
    )

    # These are the same Euclidean fingertip distances published by the official
    # senseglove_interaction finger_tip_distance_node. Keep them in the bridge so
    # thumb diagnostics remain frame-synchronous with the retargeting features.
    thumb_tip_point = thumb_points[-1]
    for finger in ("index", "middle", "ring", "pinky"):
        features[f"thumb_{finger}_tip_distance_mm"] = math.dist(
            thumb_tip_point, chains[finger][-1]
        )
    return features


def canonicalize_hand_points_for_robot(
    hand_positions_mm: list[list[float]], side: str
) -> list[list[float]]:
    """Express SGCore points in the mirrored ESROBO hand-link axis convention."""
    if len(hand_positions_mm) != 20 or side not in ("left", "right"):
        return []
    index_base = [float(value) for value in hand_positions_mm[4]]
    middle_base = [float(value) for value in hand_positions_mm[8]]
    pinky_base = [float(value) for value in hand_positions_mm[16]]
    lateral = _vector_normalize(_vector_subtract(index_base, pinky_base))
    forward = _vector_normalize(middle_base)
    if lateral is None or forward is None:
        return []
    normal = _vector_normalize(_vector_cross(lateral, forward))
    if normal is None:
        return []
    forward = _vector_normalize(_vector_cross(normal, lateral))
    if forward is None:
        return []
    robot_y_sign = -1.0 if side == "left" else 1.0
    return [
        [
            _vector_dot(point, normal),
            robot_y_sign * _vector_dot(point, lateral),
            _vector_dot(point, forward),
        ]
        for point in hand_positions_mm
    ]


def _vector_subtract(left: list[float], right: list[float]) -> list[float]:
    return [left[index] - right[index] for index in range(3)]


def _vector_dot(left: list[float], right: list[float]) -> float:
    return sum(left[index] * right[index] for index in range(3))


def _vector_cross(left: list[float], right: list[float]) -> list[float]:
    return [
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    ]


def _vector_normalize(vector: list[float]) -> list[float] | None:
    norm = math.sqrt(_vector_dot(vector, vector))
    if norm < 1.0e-6 or not math.isfinite(norm):
        return None
    return [value / norm for value in vector]


def _unsigned_vector_angle(first: list[float], second: list[float]) -> float:
    return math.acos(min(max(_vector_dot(first, second), -1.0), 1.0))


def correct_imu_quat_xyzw(raw_xyzw: list[float], correction: str) -> list[float] | None:
    q_in = normalize_quat_xyzw(raw_xyzw)
    if q_in is None:
        return None
    if correction == "raw":
        return q_in

    # Matches the official senseglove_interaction/common/imu_tf_broadcaster.py conversion.
    x, y, z, w = q_in
    unity_to_ros_q = normalize_quat_xyzw([z, -y, x, w])
    if unity_to_ros_q is None:
        return None
    q_corr = [0.0, 0.0, math.sin(math.pi / 4.0), math.cos(math.pi / 4.0)]
    return normalize_quat_xyzw(quat_multiply_xyzw(q_corr, unity_to_ros_q))


def opposite_reconstructed_w_branch_wxyz(
    corrected_wxyz: Any, correction: str
) -> list[float] | None:
    """Flip only the missing raw scalar component, then reapply coordinate correction."""
    if corrected_wxyz is None:
        return None
    try:
        w, x, y, z = normalize_quat_wxyz(corrected_wxyz)
    except (TypeError, ValueError):
        return None
    corrected_xyzw = [x, y, z, w]
    if correction == "raw":
        raw_xyzw = corrected_xyzw
    else:
        q_corr_inverse = [
            0.0,
            0.0,
            -math.sin(math.pi / 4.0),
            math.cos(math.pi / 4.0),
        ]
        unity_xyzw = quat_multiply_xyzw(q_corr_inverse, corrected_xyzw)
        ux, uy, uz, uw = unity_xyzw
        raw_xyzw = [uz, -uy, ux, uw]
    raw_xyzw[3] = -raw_xyzw[3]
    opposite = correct_imu_quat_xyzw(raw_xyzw, correction)
    if opposite is None:
        return None
    ox, oy, oz, ow = opposite
    return normalize_quat_wxyz([ow, ox, oy, oz])


def _normalize_senseglove_joint_name(name: Any) -> str:
    text = str(name).strip().lower()
    if len(text) > 2 and text[1] == "_" and text[0] in ("l", "r"):
        text = text[2:]
    return text


def _mean(values: list[float]) -> float:
    return sum(values) / len(values)


def load_or_collect_calibration(
    bridge: SenseGloveUdpBridge,
    node: Any,
    args: argparse.Namespace,
) -> dict[str, Any]:
    calibration_path = args.calibration_file.expanduser()
    if calibration_path.exists() and not args.recalibrate:
        with calibration_path.open("r", encoding="utf-8") as stream:
            calibration = json.load(stream)
        required_vector_features = [
            *(f"vector_{finger}_curl" for finger in FINGERS),
            "thumb",
            "thumb_brake",
            "vector_thumb_internal_curl",
            "vector_thumb_elevation",
            "vector_thumb_tip_spread",
            "vector_thumb_tip_elevation",
        ]
        has_required_vectors = args.retargeting_mode != "vector" or (
            calibration.get("feature_schema") == 6
            and all(
                key in calibration.get(pose, {}).get(side, {})
                for pose in ("open", "closed")
                for side in ("left", "right")
                for key in required_vector_features
            )
            and all(
                key in calibration.get("pinches", {}).get(finger, {}).get(side, {})
                for finger in PINCH_FINGERS
                for side in ("left", "right")
                for key in ("thumb_brake", "vector_thumb_elevation", f"thumb_{finger}_tip_distance_mm")
            )
        )
        imu_neutral = calibration.get("imu_neutral")
        imu_frame_correction = calibration.get("imu_frame_correction")
        imu_axis_decoupling = calibration.get("imu_axis_decoupling")
        imu_w_branches = calibration.get("imu_reconstructed_w_opposite")
        has_required_imu = args.disable_imu_orientation or (
            isinstance(imu_neutral, dict)
            and isinstance(imu_frame_correction, dict)
            and isinstance(imu_axis_decoupling, dict)
            and isinstance(imu_w_branches, dict)
            and all(
                isinstance(imu_neutral.get(side), list) and len(imu_neutral[side]) == 4
                and is_rotation_matrix(imu_frame_correction.get(side))
                and is_axis_decoupling_matrix(imu_axis_decoupling.get(side))
                and isinstance(imu_w_branches.get(side), bool)
                for side in ("left", "right")
            )
        )
        repair_mode_matches = args.disable_imu_orientation or (
            calibration.get("imu_w_repair") == args.imu_w_repair
        )
        if has_required_vectors and has_required_imu and repair_mode_matches:
            validate_calibration_motion(
                calibration.get("open", {}),
                calibration.get("pinches", {}),
                calibration.get("closed", {}),
                args,
            )
            print(
                f"[senseglove_bridge] Loaded calibration from {calibration_path}.",
                flush=True,
            )
            print_flexion_calibration_summary(calibration, args)
            return calibration
        if not has_required_vectors:
            print(
                f"[senseglove_bridge] Loaded {calibration_path}, but it has no complete "
                "hand-vector and index-pinch reference; starting the required recalibration.",
                flush=True,
            )
            args.recalibrate = True
            return load_or_collect_calibration(bridge, node, args)
        if not repair_mode_matches:
            print(
                f"[senseglove_bridge] Loaded {calibration_path}, but its IMU w-repair mode "
                f"does not match {args.imu_w_repair!r}; starting recalibration.",
                flush=True,
            )
            args.recalibrate = True
            return load_or_collect_calibration(bridge, node, args)
        print(
            f"[senseglove_bridge] Loaded {calibration_path}, but it has no complete "
            "per-power-cycle IMU frame calibration; starting recalibration.",
            flush=True,
        )

    if args.recalibrate:
        print(
            "[senseglove_bridge] Recalibration requested; existing file will be replaced.",
            flush=True,
        )
    elif calibration_path.exists():
        print(
            f"[senseglove_bridge] Existing calibration at {calibration_path} will be replaced.",
            flush=True,
        )
    else:
        print(
            f"[senseglove_bridge] No calibration file at {calibration_path}; starting calibration.",
            flush=True,
        )

    wait_for_both_hands(bridge, node, args)
    hand_instruction = (
        "右手"
        if args.single_side == "right"
        else "左手" if args.single_side == "left" else "双手"
    )
    total_steps = 3 if args.disable_imu_orientation else 6
    open_features, imu_neutral, _neutral_samples = collect_calibration_pose(
        bridge,
        node,
        args,
        label=f"第 1/{total_steps} 步：张手和 IMU 中立姿态",
        instruction=(
            f"让{hand_instruction}和手臂在身体两侧自然下垂，手腕保持平直；手指张开并"
            "伸直，中指指向地面，掌心朝向自己的身体（朝腿的一侧）；"
            "此姿态定义手套 IMU 零点，并对应当前侧灵巧手和机械臂末端的默认零点方向"
        ),
        require_imu=not args.disable_imu_orientation,
    )
    imu_axis_poses: dict[str, dict[str, list[float]]] = {}
    imu_frame_correction: dict[str, list[list[float]]] = {}
    imu_axis_decoupling: dict[str, list[list[float]]] = {}
    imu_reconstructed_w_opposite: dict[str, bool] = {}
    imu_fit_diagnostics: dict[str, dict[str, Any]] = {}
    if not args.disable_imu_orientation:
        axis_instructions = (
            (
                "x",
                "先恢复到第 1 步的自然下垂姿态，中指保持指向地面；像转动门把手一样"
                "转动前臂和整只手，让掌心从朝向身体开始，转向身体正前方约 25-35 度；"
                "不要弯曲手腕，也不要前后摆动指尖",
            ),
            (
                "y",
                "先恢复到第 1 步的自然下垂姿态；保持前臂完全不转，只活动手腕，把伸直的"
                "指尖向身体正前方抬约 25-35 度。手掌会随手腕自然倾斜，不要为了让掌心"
                "继续朝向身体而扭转前臂；也不要把指尖向腿部弯曲",
            ),
            (
                "z",
                "先恢复到第 1 步的自然下垂姿态，并让手离腿部留出一点安全距离；保持"
                "前臂和手肘不动，只弯曲手腕，让伸直的指尖向身体内侧靠近约 25-35 度；"
                "不要转动掌心，也不要向身体前后摆动指尖",
            ),
        )
        while True:
            imu_axis_poses = {}
            for step, (axis, instruction) in enumerate(axis_instructions, start=2):
                _features, imu_pose, _captured_samples = collect_calibration_pose(
                    bridge,
                    node,
                    args,
                    label=f"第 {step}/{total_steps} 步：IMU {axis.upper()} 轴参考姿态",
                    instruction=f"{hand_instruction}{instruction}",
                    require_imu=True,
                )
                imu_axis_poses[axis] = imu_pose

            repaired_imu_sides = {
                side
                for side, state in (("left", bridge.left), ("right", bridge.right))
                if state.imu_w_repaired
            }
            candidate_branches: dict[str, bool] = {}
            try:
                imu_frame_correction = estimate_imu_frame_corrections(
                    imu_neutral,
                    imu_axis_poses,
                    single_side=args.single_side,
                    imu_correction=args.imu_correction,
                    repaired_imu_sides=repaired_imu_sides,
                    selected_w_branches=candidate_branches,
                    fit_diagnostics=imu_fit_diagnostics,
                )
            except RuntimeError as exc:
                print(
                    f"[calibration IMU] {exc}",
                    flush=True,
                )
                persistent_repairs = persistent_imu_w_repair_sides(
                    bridge, args.single_side
                )
                if persistent_repairs:
                    repair_summary = ", ".join(
                        f"{side}={repaired}/{total} ({100.0 * repaired / total:.1f}%)"
                        for side, repaired, total in persistent_repairs
                    )
                    raise RuntimeError(
                        "IMU axis calibration cannot be trusted because the source "
                        "quaternion w component is persistently missing/duplicated and "
                        f"had to be reconstructed ({repair_summary}). The rejected axis "
                        "fit will not be sent to the robot. For finger-only testing, use "
                        "--left-only or --right-only without --*-wrist-imu; wrist IMU "
                        "teleoperation remains blocked until SenseCom/SGCore supplies a "
                        "valid quaternion."
                    ) from exc
                print(
                    "[calibration IMU] 第 1 步零点已经保留；只需重新采集第 2-4 步。"
                    "每一步都先回到手臂自然下垂、掌心朝身体的零点姿态。"
                    "程序现在返回第 2 步；按该步骤的提示操作，按 Ctrl-C 可取消。",
                    flush=True,
                )
                continue
            imu_reconstructed_w_opposite = candidate_branches
            if args.imu_calibration_mode == "static-orthogonal":
                imu_axis_decoupling = identity_side_matrices(args.single_side)
            else:
                imu_axis_decoupling = estimate_imu_axis_decoupling(
                    imu_neutral,
                    imu_axis_poses,
                    imu_frame_correction,
                    imu_reconstructed_w_opposite,
                    single_side=args.single_side,
                    imu_correction=args.imu_correction,
                )
            break

        # The static estimator is shared by two runtime modes. Record which
        # one was selected so loading preserves whether D is actually applied.
        for diagnostics in imu_fit_diagnostics.values():
            diagnostics["mode"] = args.imu_calibration_mode

    pinch_start = 2 if args.disable_imu_orientation else 5
    finger_names = {"index": "食指", "middle": "中指", "ring": "无名指", "pinky": "小指"}
    pinch_features: dict[str, dict[str, dict[str, float]]] = {}
    for offset, finger in enumerate(PINCH_FINGERS):
        finger_name = finger_names[finger]
        pinch_features[finger], _pinch_imu, _pinch_samples = collect_calibration_pose(
            bridge,
            node,
            args,
            label=f"第 {pinch_start + offset}/{total_steps} 步：拇指与{finger_name}对指姿态",
            instruction=(
                f"{hand_instruction}先张开手，再缓慢移动拇指朝{finger_name}方向；允许{finger_name}"
                f"自然弯曲，直到拇指指腹与{finger_name}指腹轻轻接触，然后保持不动。"
                "其余手指放松并尽量伸直，不要用力挤压，也不要握拳"
            ),
            require_imu=False,
        )

    closed_features, _closed_imu, _closed_samples = collect_calibration_pose(
        bridge,
        node,
        args,
        label=f"第 {total_steps}/{total_steps} 步：握拳姿态",
        instruction=(
            f"{hand_instruction}自然握拳，四指完全屈曲；拇指主动弯曲并收向掌心，然后保持不动"
        ),
        require_imu=False,
    )
    validate_calibration_motion(open_features, pinch_features, closed_features, args)
    calibration = {
        "feature_schema": 6,
        "created_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "source": "senseglove_ros/SenseGloveState",
        "method": "open_index_pinch_closed_with_multi_pose_imu_frame_calibration",
        "calibration_method": args.calibration_method,
        "imu_calibration_mode": args.imu_calibration_mode,
        "valid_window": {
            "sample_start_s": args.calibration_sample_start,
            "duration_s": args.calibration_duration,
        },
        "open": open_features,
        "pinch": pinch_features["index"],
        "pinches": pinch_features,
        "closed": closed_features,
        "imu_neutral": imu_neutral,
        "imu_axis_poses": imu_axis_poses,
        "imu_frame_correction": imu_frame_correction,
        "imu_axis_decoupling": imu_axis_decoupling,
        "imu_reconstructed_w_opposite": imu_reconstructed_w_opposite,
        "imu_fit_diagnostics": imu_fit_diagnostics,
        "imu_correction": args.imu_correction,
        "imu_w_repair": args.imu_w_repair,
        "hand_joint_order": list(ESROBO_HAND_JOINT_ORDER),
    }
    calibration_path.parent.mkdir(parents=True, exist_ok=True)
    with calibration_path.open("w", encoding="utf-8") as stream:
        json.dump(calibration, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(f"[senseglove_bridge] Saved calibration to {calibration_path}.", flush=True)
    print_flexion_calibration_summary(calibration, args)
    return calibration


def persistent_imu_w_repair_sides(
    bridge: SenseGloveUdpBridge,
    single_side: str | None,
    *,
    min_samples: int = 30,
    ratio_threshold: float = 0.90,
) -> list[tuple[str, int, int]]:
    """Report sides whose IMU scalar component was reconstructed almost always."""
    sides = (single_side,) if single_side in ("left", "right") else ("left", "right")
    persistent: list[tuple[str, int, int]] = []
    for side in sides:
        state = bridge.left if side == "left" else bridge.right
        total = state.imu_sample_count
        repaired = state.imu_w_repaired_count
        if total >= min_samples and repaired / total >= ratio_threshold:
            persistent.append((side, repaired, total))
    return persistent


def pinch_references_for_side(
    calibration: dict[str, Any], side: str
) -> dict[str, dict[str, float]]:
    """Return per-finger pinch features, accepting the old index-only layout."""
    pinches = calibration.get("pinches", {})
    result = {
        finger: pinches.get(finger, {}).get(side, {})
        for finger in PINCH_FINGERS
        if isinstance(pinches.get(finger, {}).get(side), dict)
    }
    if "index" not in result:
        legacy = calibration.get("pinch", {}).get(side, {})
        if isinstance(legacy, dict) and legacy:
            result["index"] = legacy
    return result


def validate_calibration_motion(
    open_features: dict[str, dict[str, float]],
    pinch_features: dict[str, Any],
    closed_features: dict[str, dict[str, float]],
    args: argparse.Namespace,
) -> None:
    """Reject calibration poses that do not contain each requested contact."""
    sides = (args.single_side,) if args.single_side else ("left", "right")
    for side in sides:
        opened = open_features.get(side, {})
        closed = closed_features.get(side, {})
        side_pinches = pinch_references_for_side({"pinches": pinch_features}, side)
        required_pinches = PINCH_FINGERS
        if not side_pinches and isinstance(pinch_features.get(side), dict):
            side_pinches = {"index": pinch_features[side]}
            required_pinches = ("index",)
        for finger in required_pinches:
            pinched = side_pinches.get(finger, {})
            key = f"thumb_{finger}_tip_distance_mm"
            open_distance = float(opened.get(key, float("nan")))
            pinch_distance = float(pinched.get(key, float("nan")))
            if (
                not math.isfinite(open_distance)
                or not math.isfinite(pinch_distance)
            ):
                raise RuntimeError(
                    f"{side} thumb-{finger} pinch calibration has no finite fingertip "
                    "distance; reconnect the glove and repeat this pose"
                )
        pinched = side_pinches.get("index", {})
        pinch_direction_spans = {
            key: abs(
                float(pinched.get(key, opened.get(key, 0.0)))
                - float(opened.get(key, 0.0))
            )
            for key in ("thumb_brake", "vector_thumb_elevation")
        }
        if any(span < 0.08 for span in pinch_direction_spans.values()):
            raise RuntimeError(
                f"{side} thumb-index pinch did not exercise both opposition axes "
                f"(brake={pinch_direction_spans['thumb_brake']:.3f}, "
                f"elevation={pinch_direction_spans['vector_thumb_elevation']:.3f}rad); "
                "repeat the OK gesture with the thumb pad facing the index fingertip"
            )
        internal_span = abs(
            float(closed.get("vector_thumb_internal_curl", 0.0))
            - float(opened.get("vector_thumb_internal_curl", 0.0))
        )
        joint_span = abs(float(closed.get("thumb", 0.0)) - float(opened.get("thumb", 0.0)))
        if internal_span < 0.12 and joint_span < 0.08:
            raise RuntimeError(
                f"{side} thumb calibration span is too small "
                f"(internal={internal_span:.3f}, joints={joint_span:.3f}); "
                "repeat calibration and actively fold the thumb toward the palm"
            )
        for finger in ("index", "middle", "ring"):
            candidates = finger_flexion_candidates(opened, closed, finger)
            if not candidates or candidates[0][0] < 0.15:
                raise RuntimeError(
                    f"{side} {finger} calibration has no reliable flexion signal; "
                    "repeat calibration with fully straight fingers, then make a complete fist"
                )


def print_flexion_calibration_summary(
    calibration: dict[str, Any], args: argparse.Namespace
) -> None:
    """Show which calibrated channel will drive each non-thumb finger."""
    sides = (args.single_side,) if args.single_side else ("left", "right")
    for side in sides:
        opened = calibration.get("open", {}).get(side, {})
        closed = calibration.get("closed", {}).get(side, {})
        selections = []
        for finger in ("index", "middle", "ring", "pinky"):
            candidates = finger_flexion_candidates(opened, closed, finger)
            if candidates:
                span, key = candidates[0]
                selections.append(f"{finger}={key}(span={span:.3f})")
            else:
                selections.append(f"{finger}=invalid")
        print(
            f"[calibration] {side} flexion sources: " + ", ".join(selections),
            flush=True,
        )


def wait_for_both_hands(
    bridge: SenseGloveUdpBridge, node: Any, args: argparse.Namespace
) -> None:
    next_print = 0.0
    invalid_since: float | None = None
    while True:
        spin_some(node, timeout_sec=0.05)
        if bridge.has_both_hands(args.max_state_age):
            scope = f"{args.single_side} glove" if args.single_side else "both gloves"
            print(
                f"[SenseGlove] 已收到 {scope} 实时数据：{bridge.topic_status()}。",
                flush=True,
            )
            return
        now = time.monotonic()
        states = (
            (bridge.right,)
            if args.single_side == "right"
            else (bridge.left,)
            if args.single_side == "left"
            else (bridge.left, bridge.right)
        )
        has_fresh_invalid_frame = all(
            state.msg is not None
            and state.stamp_monotonic is not None
            and now - state.stamp_monotonic <= args.max_state_age
            and not state.has_live_sensor_payload()
            for state in states
        )
        if has_fresh_invalid_frame:
            invalid_since = invalid_since or now
            if now - invalid_since >= 5.0:
                raise RuntimeError(
                    "SenseGlove ROS topic is active but contains only all-zero placeholder "
                    "data. The glove pose is not reaching ROS through SenseCom/SGCore; "
                    "restart SenseCom and reconnect the glove before calibration. "
                    f"Status: {bridge.topic_status()}"
                )
        else:
            invalid_since = None
        if now >= next_print:
            print(
                f"[SenseGlove] 正在等待{'单只' if args.single_side else '左右'}手套数据，"
                f"请确认手套保持开机：{bridge.topic_status()}。",
                flush=True,
            )
            next_print = now + 1.0


def collect_calibration_pose(
    bridge: SenseGloveUdpBridge,
    node: Any,
    args: argparse.Namespace,
    label: str,
    instruction: str,
    require_imu: bool = False,
) -> tuple[
    dict[str, dict[str, float]],
    dict[str, list[float]],
    dict[str, Any],
]:
    settle_s = min(args.calibration_sample_start, args.calibration_duration)
    record_s = max(0.0, args.calibration_duration - settle_s)
    print("", flush=True)
    print("============================================================", flush=True)
    print(f" {label}", flush=True)
    print(f" 动作要求：{instruction}。", flush=True)
    print(
        f" 按 Enter 后开始：前 {settle_s:.1f} 秒用于稳定姿态，随后采集约 {record_s:.1f} 秒。",
        flush=True,
    )
    print(" 采集完成前请保持姿态，不要切换动作。", flush=True)
    print("============================================================", flush=True)
    if not args.no_prompt:
        input(f"[等待操作] {label}准备好后，请按 Enter 开始采集：")
    else:
        print(f"[SenseGlove] 自动开始 {label}。", flush=True)

    # The prompt can remain open for an arbitrary time while ROS messages build
    # up. Drain that backlog and require fresh data again before timing starts.
    wait_for_both_hands(bridge, node, args)
    print(f"[SenseGlove] 已开始 {label}，请保持当前姿态。", flush=True)

    left_samples: list[dict[str, float]] = []
    right_samples: list[dict[str, float]] = []
    left_imu_samples: list[list[float]] = []
    right_imu_samples: list[list[float]] = []
    start = time.monotonic()
    next_print = 0.0
    last_sample_sequences = (-1, -1)
    while True:
        spin_some(node, timeout_sec=0.01)
        now = time.monotonic()
        elapsed = now - start
        current_sequences = (
            bridge.left.callback_sequence,
            bridge.right.callback_sequence,
        )
        if bridge.has_both_hands(args.max_state_age) and current_sequences != last_sample_sequences:
            if elapsed >= settle_s:
                left_samples.append(dict(bridge.left.features))
                right_samples.append(dict(bridge.right.features))
                if (
                    bridge.left.imu_quat_wxyz is not None
                    and bridge.right.imu_quat_wxyz is not None
                ):
                    left_imu_samples.append(list(bridge.left.imu_quat_wxyz))
                    right_imu_samples.append(list(bridge.right.imu_quat_wxyz))
            last_sample_sequences = current_sequences
        if now >= next_print:
            remaining = max(0.0, args.calibration_duration - elapsed)
            if elapsed < settle_s:
                until_record = math.ceil(settle_s - elapsed)
                print(
                    f"[SenseGlove] {label}：保持姿态，{until_record} 秒后开始记录，"
                    f"本步骤还剩 {math.ceil(remaining)} 秒。",
                    flush=True,
                )
            else:
                print(
                    f"[SenseGlove] {label}：正在记录，"
                    f"还剩 {math.ceil(remaining)} 秒，"
                    f"手指样本={len(left_samples)}，IMU 样本={len(left_imu_samples)}，"
                    f"{bridge.topic_status()}。",
                    flush=True,
                )
            next_print = now + 1.0
        if elapsed >= args.calibration_duration:
            break

    if not left_samples or not right_samples:
        raise RuntimeError(
            f"no valid SenseGlove samples captured for calibration {label}"
        )
    if require_imu and (not left_imu_samples or not right_imu_samples):
        raise RuntimeError(
            f"no valid SenseGlove IMU samples captured for calibration {label}"
        )

    left_features = reduce_samples(left_samples, args.calibration_method)
    right_features = reduce_samples(right_samples, args.calibration_method)
    imu_neutral: dict[str, list[float]] = {}
    if left_imu_samples and right_imu_samples:
        imu_neutral = {
            "left": reduce_quat_samples(left_imu_samples, args.calibration_method),
            "right": reduce_quat_samples(right_imu_samples, args.calibration_method),
        }
    print(
        f"[SenseGlove] {label}采集完成："
        f"left={format_feature_preview(left_features)} right={format_feature_preview(right_features)} "
        f"imu_left={format_quat_preview(imu_neutral.get('left'))} "
        f"imu_right={format_quat_preview(imu_neutral.get('right'))}.",
        flush=True,
    )
    return (
        {"left": left_features, "right": right_features},
        imu_neutral,
        {"left": left_imu_samples, "right": right_imu_samples},
    )


def spin_once(node: Any, timeout_sec: float) -> None:
    import rclpy

    rclpy.spin_once(node, timeout_sec=timeout_sec)


def spin_some(node: Any, timeout_sec: float, max_callbacks: int = 8) -> None:
    """Process enough callbacks to keep up with both 60 Hz glove topics."""
    spin_once(node, timeout_sec=timeout_sec)
    for _ in range(max_callbacks - 1):
        spin_once(node, timeout_sec=0.0)


def reduce_samples(samples: list[dict[str, float]], method: str) -> dict[str, float]:
    if method == "last":
        return dict(samples[-1])
    keys = sorted({key for sample in samples for key in sample.keys()})
    return {
        key: _mean([sample[key] for sample in samples if key in sample]) for key in keys
    }


def reduce_quat_samples(samples: list[list[float]], method: str) -> list[float]:
    if method == "last":
        return normalize_quat_wxyz(samples[-1])

    reference = normalize_quat_wxyz(samples[0])
    aligned: list[list[float]] = []
    for sample in samples:
        quat = normalize_quat_wxyz(sample)
        if quat_dot_wxyz(reference, quat) < 0.0:
            quat = [-value for value in quat]
        aligned.append(quat)
    averaged = [
        sum(quat[index] for quat in aligned) / len(aligned) for index in range(4)
    ]
    return normalize_quat_wxyz(averaged)


def format_feature_preview(features: dict[str, float]) -> str:
    return ",".join(f"{finger}={features.get(finger, 0.0):.3f}" for finger in FINGERS)


def format_quat_preview(quat: list[float] | None) -> str:
    if quat is None:
        return "none"
    return "[" + ",".join(f"{value:.3f}" for value in quat) + "]"


def normalize_pinch_references(
    pinch_features: dict[str, Any] | None,
) -> dict[str, dict[str, float]]:
    if not isinstance(pinch_features, dict):
        return {}
    references = {
        finger: pinch_features[finger]
        for finger in PINCH_FINGERS
        if isinstance(pinch_features.get(finger), dict)
    }
    if references:
        return references
    return {"index": pinch_features} if pinch_features else {}


def retarget_side(
    features: dict[str, float],
    open_features: dict[str, float],
    closed_features: dict[str, float],
    pinch_features: dict[str, Any] | None,
    robot_side: str,
    args: argparse.Namespace,
) -> tuple[list[float], dict[str, float]]:
    if args.retargeting_mode == "vector":
        return retarget_side_vectors(
            features, open_features, closed_features, robot_side, args, pinch_features
        )

    curls = {
        finger: normalize_feature(features, open_features, closed_features, finger)
        for finger in FINGERS
    }
    spreads = {
        finger: normalize_feature(
            features, open_features, closed_features, f"{finger}_brake"
        )
        for finger in FINGERS
    }
    curls["pinky"] = curls["ring"]
    spreads["pinky"] = spreads["ring"]
    joint_values = {
        f"{robot_side}_thumb_cmc_roll": limit_lerp(
            f"{robot_side}_thumb_cmc_roll", args.thumb_roll_gain * curls["thumb"]
        ),
        f"{robot_side}_thumb_cmc_yaw": limit_lerp(
            f"{robot_side}_thumb_cmc_yaw", args.thumb_yaw_gain * curls["thumb"]
        ),
        f"{robot_side}_thumb_cmc_pitch": limit_lerp(
            f"{robot_side}_thumb_cmc_pitch", curls["thumb"]
        ),
        f"{robot_side}_index_mcp_roll": limit_lerp(
            f"{robot_side}_index_mcp_roll", args.spread_scale * spreads["index"]
        ),
        f"{robot_side}_index_mcp_pitch": limit_lerp(
            f"{robot_side}_index_mcp_pitch", curls["index"]
        ),
        f"{robot_side}_middle_mcp_pitch": limit_lerp(
            f"{robot_side}_middle_mcp_pitch", curls["middle"]
        ),
        f"{robot_side}_ring_mcp_roll": limit_lerp(
            f"{robot_side}_ring_mcp_roll", args.spread_scale * spreads["ring"]
        ),
        f"{robot_side}_ring_mcp_pitch": limit_lerp(
            f"{robot_side}_ring_mcp_pitch", curls["ring"]
        ),
        f"{robot_side}_pinky_mcp_roll": limit_lerp(
            f"{robot_side}_pinky_mcp_roll", args.spread_scale * spreads["pinky"]
        ),
        f"{robot_side}_pinky_mcp_pitch": limit_lerp(
            f"{robot_side}_pinky_mcp_pitch", curls["pinky"]
        ),
    }
    order = [
        joint for joint in ESROBO_HAND_JOINT_ORDER if joint.startswith(f"{robot_side}_")
    ]
    return [joint_values[joint] for joint in order], curls


def retarget_side_vectors(
    features: dict[str, float],
    open_features: dict[str, float],
    closed_features: dict[str, float],
    robot_side: str,
    args: argparse.Namespace,
    pinch_features: dict[str, Any] | None = None,
) -> tuple[list[float], dict[str, float]]:
    """Map palm-relative human finger vectors onto one hand's ten active robot joints."""
    is_right = robot_side == "right"
    finger_gain = (
        args.right_finger_flexion_gain if is_right else args.finger_flexion_gain
    )
    finger_exponent = (
        args.right_finger_flexion_exponent if is_right else args.finger_flexion_exponent
    )
    thumb_flexion_gain = (
        args.right_thumb_flexion_gain if is_right else args.thumb_flexion_gain
    )
    thumb_flexion_exponent = (
        args.right_thumb_flexion_exponent if is_right else args.thumb_flexion_exponent
    )
    thumb_roll_gain = args.right_thumb_roll_gain if is_right else args.thumb_roll_gain
    thumb_yaw_gain = args.right_thumb_yaw_gain if is_right else args.thumb_yaw_gain
    pinch_references = normalize_pinch_references(pinch_features)

    curls = {}
    for finger in ("index", "middle", "ring", "pinky"):
        source_finger = FINGER_INPUT_SOURCE.get(finger, finger)
        candidates = finger_flexion_candidates(
            open_features, closed_features, source_finger
        )
        key = candidates[0][1] if candidates else source_finger
        normalized = normalize_feature(features, open_features, closed_features, key)
        curls[finger] = clamp01(
            finger_gain * normalized ** finger_exponent
        )
    curls["thumb"] = normalize_feature(
        features, open_features, closed_features, "vector_thumb_curl"
    )

    # Nova 2's reported thumb MCP/PIP/DIP angles are the most direct flexion
    # signal. Blend in only a small amount of skeleton-internal curl so CMC
    # translation cannot cancel the bend, then expand the low end for useful
    # LinkerHand pitch travel.
    thumb_joint_curl = normalize_feature(
        features, open_features, closed_features, "thumb"
    )
    thumb_internal_curl = normalize_feature_with_fallback(
        features,
        open_features,
        closed_features,
        "vector_thumb_internal_curl",
        thumb_joint_curl,
        min_span=0.12,
    )
    thumb_pitch = clamp01(
        thumb_flexion_gain
        * (0.80 * thumb_joint_curl + 0.20 * thumb_internal_curl)
        ** thumb_flexion_exponent
    )

    # SGCore already decomposes Nova 2's hand pose into thumb flexion (MCP/PIP/DIP,
    # HandAngles Y) and abduction/adduction (thumb_brake, -HandAngles Z). Use that
    # direct abduction channel for robot yaw. The previous wrist-to-tip azimuth
    # changed strongly whenever the distal joints bent and therefore converted
    # ordinary thumb flexion into a large side swing.
    direction_reference = (
        pinch_references["index"]
        if "index" in pinch_references
        else closed_features
    )
    thumb_yaw_normalized = normalize_feature_unclamped(
        features, open_features, direction_reference, "thumb_brake"
    )

    # Nova 2 has no independent axial thumb-rotation sensor. Approximate robot
    # CMC roll/opposition from the proximal segment elevation; unlike fingertip
    # elevation, this does not include PIP/DIP tip travel. Keep both direction
    # channels linear so small sensor changes are not enlarged by the <1 flexion
    # exponent used to improve low-angle bending response.
    thumb_roll_normalized = normalize_feature_unclamped(
        features, open_features, direction_reference, "vector_thumb_elevation"
    )
    thumb_roll = clamp01(thumb_roll_gain * thumb_roll_normalized)
    thumb_yaw = clamp01(thumb_yaw_gain * thumb_yaw_normalized)
    curls["thumb"] = thumb_pitch

    spread_range = math.radians(args.vector_spread_range_deg)
    outward_sign = {"index": 1.0, "ring": -1.0, "pinky": -1.0}
    spreads: dict[str, float] = {}
    for finger, sign in outward_sign.items():
        source_finger = FINGER_INPUT_SOURCE.get(finger, finger)
        key = f"vector_{source_finger}_spread"
        delta = float(features.get(key, open_features.get(key, 0.0))) - float(
            open_features.get(key, 0.0)
        )
        spreads[finger] = clamp01(sign * delta / spread_range) * args.spread_scale

    joint_values = {
        f"{robot_side}_thumb_cmc_roll": limit_lerp(
            f"{robot_side}_thumb_cmc_roll", thumb_roll
        ),
        f"{robot_side}_thumb_cmc_yaw": limit_lerp(
            f"{robot_side}_thumb_cmc_yaw", thumb_yaw
        ),
        f"{robot_side}_thumb_cmc_pitch": limit_lerp(
            f"{robot_side}_thumb_cmc_pitch", thumb_pitch
        ),
        f"{robot_side}_index_mcp_roll": limit_lerp(
            f"{robot_side}_index_mcp_roll", spreads["index"]
        ),
        f"{robot_side}_index_mcp_pitch": limit_lerp(
            f"{robot_side}_index_mcp_pitch", curls["index"]
        ),
        f"{robot_side}_middle_mcp_pitch": limit_lerp(
            f"{robot_side}_middle_mcp_pitch", curls["middle"]
        ),
        f"{robot_side}_ring_mcp_roll": limit_lerp(
            f"{robot_side}_ring_mcp_roll", spreads["ring"]
        ),
        f"{robot_side}_ring_mcp_pitch": limit_lerp(
            f"{robot_side}_ring_mcp_pitch", curls["ring"]
        ),
        f"{robot_side}_pinky_mcp_roll": limit_lerp(
            f"{robot_side}_pinky_mcp_roll", spreads["pinky"]
        ),
        f"{robot_side}_pinky_mcp_pitch": limit_lerp(
            f"{robot_side}_pinky_mcp_pitch", curls["pinky"]
        ),
    }
    order = [
        joint for joint in ESROBO_HAND_JOINT_ORDER if joint.startswith(f"{robot_side}_")
    ]
    diagnostics = dict(curls)
    diagnostics.update({f"{finger}_spread": value for finger, value in spreads.items()})
    diagnostics.update(
        {
            "thumb_joint_curl": thumb_joint_curl,
            "thumb_internal_curl": thumb_internal_curl,
            "thumb_roll": clamp01(thumb_roll),
            "thumb_yaw": clamp01(thumb_yaw),
            "thumb_pitch": thumb_pitch,
            "pinch_contact_confidence": 0.0,
            **{
                key: float(features.get(key, 0.0))
                for key in (
                    "thumb_index_tip_distance_mm",
                    "thumb_middle_tip_distance_mm",
                    "thumb_ring_tip_distance_mm",
                    "thumb_pinky_tip_distance_mm",
                )
            },
        }
    )
    return [joint_values[joint] for joint in order], diagnostics


def finger_flexion_candidates(
    open_features: dict[str, float], closed_features: dict[str, float], finger: str
) -> list[tuple[float, str]]:
    """Rank flexion channels by positive calibrated open-to-closed travel."""
    candidates = []
    for key in (f"vector_{finger}_curl", finger):
        if key not in open_features or key not in closed_features:
            continue
        span = float(closed_features[key]) - float(open_features[key])
        if math.isfinite(span) and span > 0.0:
            candidates.append((span, key))
    return sorted(candidates, reverse=True)


def adapt_flexion_calibration_envelope(
    bridge: SenseGloveUdpBridge, calibration: dict[str, Any]
) -> None:
    """Expand calibration endpoints when live motion exceeds the sampled poses."""
    source_by_target = {
        "left": bridge.right if bridge.args.swap_left_right_targets else bridge.left,
        "right": bridge.left if bridge.args.swap_left_right_targets else bridge.right,
    }
    calibration_side = {
        "left": "right" if bridge.args.swap_left_right_targets else "left",
        "right": "left" if bridge.args.swap_left_right_targets else "right",
    }
    for target_side, source in source_by_target.items():
        side = calibration_side[target_side]
        opened = calibration.get("open", {}).get(side, {})
        closed = calibration.get("closed", {}).get(side, {})
        for finger in ("index", "middle", "ring", "pinky"):
            for _span, key in finger_flexion_candidates(opened, closed, finger):
                value = source.features.get(key)
                if value is None or not math.isfinite(float(value)):
                    continue
                opened[key] = min(float(opened[key]), float(value))
                closed[key] = max(float(closed[key]), float(value))


def normalize_feature_with_fallback(
    features: dict[str, float],
    open_features: dict[str, float],
    closed_features: dict[str, float],
    key: str,
    fallback: float,
    min_span: float = 1.0e-4,
) -> float:
    open_value = float(open_features.get(key, 0.0))
    closed_value = float(closed_features.get(key, open_value))
    if abs(closed_value - open_value) < min_span:
        return clamp01(fallback)
    return normalize_feature(features, open_features, closed_features, key)


def normalize_feature(
    features: dict[str, float],
    open_features: dict[str, float],
    closed_features: dict[str, float],
    key: str,
) -> float:
    open_value = float(open_features.get(key, 0.0))
    closed_value = float(closed_features.get(key, open_value))
    denominator = closed_value - open_value
    if abs(denominator) < 1.0e-6:
        return 0.0
    value = float(features.get(key, open_value))
    return clamp01((value - open_value) / denominator)


def normalize_feature_unclamped(
    features: dict[str, float],
    open_features: dict[str, float],
    endpoint_features: dict[str, float],
    key: str,
) -> float:
    """Normalize to a reference pose while preserving travel beyond that pose."""
    open_value = float(open_features.get(key, 0.0))
    endpoint_value = float(endpoint_features.get(key, open_value))
    denominator = endpoint_value - open_value
    if abs(denominator) < 1.0e-6:
        return 0.0
    value = float(features.get(key, open_value))
    return max(0.0, (value - open_value) / denominator)


def limit_lerp(joint_name: str, normalized_value: float) -> float:
    lower, upper = ESROBO_HAND_JOINT_LIMITS[joint_name]
    return lower + clamp01(normalized_value) * (upper - lower)


def clamp01(value: float) -> float:
    return min(max(float(value), 0.0), 1.0)


def orientation_delta_wxyz(
    current: list[float] | None,
    neutral: Any,
    frame_correction: Any = None,
    axis_decoupling: Any = None,
) -> list[float] | None:
    if current is None or neutral is None:
        return None
    try:
        neutral_quat = [float(value) for value in neutral]
    except (TypeError, ValueError):
        return None
    current_quat = normalize_quat_wxyz(current)
    neutral_quat = normalize_quat_wxyz(neutral_quat)
    # Remove the neutral offset first. The optional conjugation then expresses
    # this boot's measured delta in the fixed anatomical basis.
    raw_delta = normalize_quat_wxyz(
        quat_multiply_wxyz(quat_conjugate_wxyz(neutral_quat), current_quat)
    )
    if frame_correction is None:
        return raw_delta
    if not is_rotation_matrix(frame_correction):
        return None
    correction = np.asarray(frame_correction, dtype=np.float64).reshape(3, 3)
    raw_rotation = quat_wxyz_to_matrix(raw_delta)
    corrected_rotation = correction @ raw_rotation @ correction.T
    corrected_quat = matrix_to_quat_wxyz(corrected_rotation)
    if axis_decoupling is None:
        return corrected_quat
    if not is_axis_decoupling_matrix(axis_decoupling):
        return None
    decoupling = np.asarray(axis_decoupling, dtype=np.float64).reshape(3, 3)
    return rotation_vector_to_quat_wxyz(decoupling @ rotation_vector_wxyz(corrected_quat))


def runtime_axis_decoupling(
    side: str, axis_decoupling: Any, calibration_mode: Any = None
) -> Any:
    """Select the runtime correction while keeping old calibration files compatible."""
    if calibration_mode == "static-orthogonal":
        return None
    if calibration_mode == "static-linear":
        return axis_decoupling
    if side == "left":
        # A non-orthogonal calibration action must not become a permanent shear:
        # the latest left fit would inject Z flexion into both X and Y channels.
        return None
    return axis_decoupling


def is_rotation_matrix(value: Any, atol: float = 2.0e-3) -> bool:
    try:
        matrix = np.asarray(value, dtype=np.float64).reshape(3, 3)
    except (TypeError, ValueError):
        return False
    if not np.all(np.isfinite(matrix)):
        return False
    return bool(
        np.allclose(matrix.T @ matrix, np.eye(3), atol=atol)
        and np.linalg.det(matrix) > 1.0 - atol
    )


def is_axis_decoupling_matrix(value: Any, max_condition: float = 5.0) -> bool:
    try:
        matrix = np.asarray(value, dtype=np.float64).reshape(3, 3)
    except (TypeError, ValueError):
        return False
    if not np.all(np.isfinite(matrix)):
        return False
    condition = float(np.linalg.cond(matrix))
    return math.isfinite(condition) and condition <= max_condition


def quat_wxyz_to_matrix(quat: list[float]) -> np.ndarray:
    w, x, y, z = normalize_quat_wxyz(quat)
    return np.asarray(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def matrix_to_quat_wxyz(matrix: Any) -> list[float]:
    rotation = np.asarray(matrix, dtype=np.float64).reshape(3, 3)
    candidates = np.asarray(
        [
            1.0 + rotation[0, 0] + rotation[1, 1] + rotation[2, 2],
            1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2],
            1.0 - rotation[0, 0] + rotation[1, 1] - rotation[2, 2],
            1.0 - rotation[0, 0] - rotation[1, 1] + rotation[2, 2],
        ]
    )
    largest = int(np.argmax(candidates))
    root = math.sqrt(max(0.0, float(candidates[largest]))) * 0.5
    denominator = max(4.0 * root, 1.0e-8)
    if largest == 0:
        quat = [
            root,
            (rotation[2, 1] - rotation[1, 2]) / denominator,
            (rotation[0, 2] - rotation[2, 0]) / denominator,
            (rotation[1, 0] - rotation[0, 1]) / denominator,
        ]
    elif largest == 1:
        quat = [
            (rotation[2, 1] - rotation[1, 2]) / denominator,
            root,
            (rotation[0, 1] + rotation[1, 0]) / denominator,
            (rotation[0, 2] + rotation[2, 0]) / denominator,
        ]
    elif largest == 2:
        quat = [
            (rotation[0, 2] - rotation[2, 0]) / denominator,
            (rotation[0, 1] + rotation[1, 0]) / denominator,
            root,
            (rotation[1, 2] + rotation[2, 1]) / denominator,
        ]
    else:
        quat = [
            (rotation[1, 0] - rotation[0, 1]) / denominator,
            (rotation[0, 2] + rotation[2, 0]) / denominator,
            (rotation[1, 2] + rotation[2, 1]) / denominator,
            root,
        ]
    return normalize_quat_wxyz(quat)


def rotation_vector_wxyz(quat: list[float]) -> np.ndarray:
    normalized = np.asarray(normalize_quat_wxyz(quat), dtype=np.float64)
    if normalized[0] < 0.0:
        normalized = -normalized
    vector = normalized[1:]
    vector_norm = float(np.linalg.norm(vector))
    if vector_norm < 1.0e-8:
        return 2.0 * vector
    angle = 2.0 * math.atan2(vector_norm, float(normalized[0]))
    return vector * (angle / vector_norm)


def rotation_vector_to_quat_wxyz(rotvec: Any) -> list[float]:
    vector = np.asarray(rotvec, dtype=np.float64).reshape(3)
    angle = float(np.linalg.norm(vector))
    if angle < 1.0e-8:
        return normalize_quat_wxyz([1.0, *(0.5 * vector).tolist()])
    scale = math.sin(0.5 * angle) / angle
    return normalize_quat_wxyz([math.cos(0.5 * angle), *(vector * scale).tolist()])


def estimate_imu_frame_corrections(
    imu_neutral: dict[str, list[float]],
    imu_axis_poses: dict[str, dict[str, list[float]]],
    single_side: str | None = None,
    imu_correction: str = "unity_to_ros",
    repaired_imu_sides: set[str] | None = None,
    selected_w_branches: dict[str, bool] | None = None,
    fit_diagnostics: dict[str, dict[str, Any]] | None = None,
) -> dict[str, list[list[float]]]:
    """Fit the rotation from this power cycle's IMU basis to anatomical XYZ."""
    corrections: dict[str, list[list[float]]] = {}
    calibration_sides = (
        (single_side,) if single_side in ("left", "right") else ("left", "right")
    )
    for side in calibration_sides:
        # These signs describe the fixed physical prompts, not a per-run axis
        # permutation. The right Y prompt is the negative anatomical Y action.
        expected_axes = np.diag(
            [-1.0, 1.0, 1.0] if side == "left" else [1.0, -1.0, 1.0]
        )
        branch_options = [False]
        if repaired_imu_sides and side in repaired_imu_sides:
            branch_options.append(True)
        candidates = []
        for opposite_branch in branch_options:
            neutral = imu_neutral.get(side)
            poses = [
                imu_axis_poses.get(axis, {}).get(side) for axis in ("x", "y", "z")
            ]
            if opposite_branch:
                neutral = opposite_reconstructed_w_branch_wxyz(neutral, imu_correction)
                poses = [
                    opposite_reconstructed_w_branch_wxyz(pose, imu_correction)
                    for pose in poses
                ]
            measured_axes: list[np.ndarray] = []
            spans_deg: list[float] = []
            for axis, pose in zip(("x", "y", "z"), poses):
                delta = orientation_delta_wxyz(pose, neutral)
                if delta is None:
                    raise RuntimeError(f"{side} IMU {axis.upper()} reference pose is missing")
                rotvec = rotation_vector_wxyz(delta)
                span = float(np.linalg.norm(rotvec))
                spans_deg.append(math.degrees(span))
                if span < math.radians(10.0):
                    raise RuntimeError(
                        f"{side} IMU {axis.upper()} calibration span is too small "
                        f"({math.degrees(span):.1f}deg); repeat that single-axis action at 25-35deg"
                    )
                measured_axes.append(rotvec / span)

            measured = np.stack(measured_axes, axis=1)
            pairwise_separations = [
                math.degrees(
                    math.acos(
                        float(
                            np.clip(
                                abs(np.dot(measured[:, first], measured[:, second])),
                                0.0,
                                1.0,
                            )
                        )
                    )
                )
                for first, second in ((0, 1), (0, 2), (1, 2))
            ]
            u, singular_values, vt = np.linalg.svd(expected_axes @ measured.T)
            handedness = np.eye(3, dtype=np.float64)
            handedness[2, 2] = np.linalg.det(u @ vt)
            correction = u @ handedness @ vt
            corrected_axes = correction @ measured
            residuals = [
                math.degrees(
                    math.acos(
                        float(
                            np.clip(
                                np.dot(corrected_axes[:, index], expected_axes[:, index]),
                                -1.0,
                                1.0,
                            )
                        )
                    )
                )
                for index in range(3)
            ]
            candidates.append(
                (
                    max(residuals),
                    opposite_branch,
                    correction,
                    singular_values,
                    residuals,
                    spans_deg,
                    pairwise_separations,
                )
            )
        (
            _,
            opposite_branch,
            correction,
            singular_values,
            residuals,
            spans_deg,
            pairwise_separations,
        ) = min(candidates, key=lambda candidate: candidate[0])
        if singular_values[-1] < 0.20 or max(residuals) > 25.0:
            raise RuntimeError(
                f"{side} IMU axis calibration is ambiguous: "
                f"spans={np.round(spans_deg, 1).tolist()}deg, "
                f"axis residuals={np.round(residuals, 1).tolist()}deg, "
                "axis separations XY/XZ/YZ="
                f"{np.round(pairwise_separations, 1).tolist()}deg; return to neutral "
                "before each pose and move only the requested axis"
            )
        corrections[side] = correction.tolist()
        if fit_diagnostics is not None:
            fit_diagnostics[side] = {
                "mode": "static-linear",
                "spans_deg": spans_deg,
                "axis_residuals_deg": residuals,
                "axis_separations_deg": pairwise_separations,
            }
        if selected_w_branches is not None:
            selected_w_branches[side] = opposite_branch
        print(
            f"[calibration IMU] {side} frame correction verified: "
            f"spans={np.round(spans_deg, 1).tolist()}deg, "
            f"axis residuals={np.round(residuals, 1).tolist()}deg, "
            "axis separations XY/XZ/YZ="
            f"{np.round(pairwise_separations, 1).tolist()}deg, "
            f"reconstructed-w branch={'opposite' if opposite_branch else 'primary'}.",
            flush=True,
        )
    if single_side in ("left", "right"):
        mirror_side = "left" if single_side == "right" else "right"
        corrections[mirror_side] = copy.deepcopy(corrections[single_side])
        if selected_w_branches is not None:
            selected_w_branches[mirror_side] = selected_w_branches.get(
                single_side, False
            )
        if fit_diagnostics is not None:
            fit_diagnostics[mirror_side] = copy.deepcopy(
                fit_diagnostics.get(single_side, {})
            )
    return corrections


def identity_side_matrices(single_side: str | None = None) -> dict[str, list[list[float]]]:
    matrix = np.eye(3, dtype=np.float64).tolist()
    if single_side in ("left", "right"):
        return {single_side: copy.deepcopy(matrix),
                "left" if single_side == "right" else "right": copy.deepcopy(matrix)}
    return {"left": copy.deepcopy(matrix), "right": copy.deepcopy(matrix)}


def estimate_imu_axis_decoupling(
    imu_neutral: dict[str, list[float]],
    imu_axis_poses: dict[str, dict[str, list[float]]],
    frame_corrections: dict[str, list[list[float]]],
    reconstructed_w_opposite: dict[str, bool],
    single_side: str | None = None,
    imu_correction: str = "unity_to_ros",
) -> dict[str, list[list[float]]]:
    """Fit a bounded linear correction for residual non-orthogonal IMU axes."""
    result: dict[str, list[list[float]]] = {}
    sides = (single_side,) if single_side in ("left", "right") else ("left", "right")
    for side in sides:
        expected = np.diag(
            [-1.0, 1.0, 1.0] if side == "left" else [1.0, -1.0, 1.0]
        )
        neutral = imu_neutral.get(side)
        poses = [imu_axis_poses.get(axis, {}).get(side) for axis in ("x", "y", "z")]
        if reconstructed_w_opposite.get(side) is True:
            neutral = opposite_reconstructed_w_branch_wxyz(neutral, imu_correction)
            poses = [
                opposite_reconstructed_w_branch_wxyz(pose, imu_correction)
                for pose in poses
            ]
        correction = frame_corrections.get(side)
        measured_axes = []
        for axis, pose in zip(("x", "y", "z"), poses):
            delta = orientation_delta_wxyz(pose, neutral, correction)
            if delta is None:
                raise RuntimeError(f"{side} IMU {axis.upper()} decoupling pose is missing")
            rotvec = rotation_vector_wxyz(delta)
            span = float(np.linalg.norm(rotvec))
            if span < math.radians(10.0):
                raise RuntimeError(f"{side} IMU {axis.upper()} decoupling span is too small")
            measured_axes.append(rotvec / span)
        measured = np.stack(measured_axes, axis=1)
        condition = float(np.linalg.cond(measured))
        if not math.isfinite(condition) or condition > 5.0:
            raise RuntimeError(
                f"{side} IMU axis decoupling is ill-conditioned ({condition:.2f}); "
                "repeat the three isolated-axis poses"
            )
        decoupling = expected @ np.linalg.inv(measured)
        if not is_axis_decoupling_matrix(decoupling):
            raise RuntimeError(f"{side} IMU axis decoupling matrix is invalid")
        result[side] = decoupling.tolist()
        print(
            f"[calibration IMU] {side} residual axis decoupling verified: "
            f"condition={condition:.2f}.",
            flush=True,
        )
    if single_side in ("left", "right"):
        mirror_side = "left" if single_side == "right" else "right"
        result[mirror_side] = copy.deepcopy(result[single_side])
    return result


def normalize_quat_wxyz(raw: list[float]) -> list[float]:
    values = [float(value) for value in raw]
    norm = math.sqrt(sum(value * value for value in values))
    if norm < 1.0e-8 or not math.isfinite(norm):
        return [1.0, 0.0, 0.0, 0.0]
    return [value / norm for value in values]


def normalize_quat_xyzw(raw: list[float]) -> list[float] | None:
    try:
        values = [float(value) for value in raw]
    except (TypeError, ValueError):
        return None
    norm = math.sqrt(sum(value * value for value in values))
    if norm < 1.0e-8 or not math.isfinite(norm):
        return None
    return [value / norm for value in values]


def quat_dot_wxyz(left: list[float], right: list[float]) -> float:
    return sum(a * b for a, b in zip(left, right))


def quat_conjugate_wxyz(quat: list[float]) -> list[float]:
    w, x, y, z = quat
    return [w, -x, -y, -z]


def quat_multiply_wxyz(left: list[float], right: list[float]) -> list[float]:
    w1, x1, y1, z1 = left
    w2, x2, y2, z2 = right
    return [
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ]


def quat_multiply_xyzw(left: list[float], right: list[float]) -> list[float]:
    x1, y1, z1, w1 = left
    x2, y2, z2, w2 = right
    return [
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ]


class RightHandSkeletonViewer:
    """Serve a live browser dashboard that works through an SSH port forward."""

    HTML = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Right SenseGlove diagnostics</title>
<style>
*{box-sizing:border-box}body{margin:0;background:#f4f5f7;color:#20242a;font-family:Arial,sans-serif}
header{height:52px;padding:14px 20px;background:#20242a;color:#fff;font-size:18px;font-weight:600}
main{display:grid;grid-template-columns:minmax(0,1fr) 280px;height:calc(100vh - 52px)}
#view{width:100%;height:100%;background:#fff;cursor:grab}aside{border-left:1px solid #d8dce2;padding:20px;background:#f8f9fa}
h2{font-size:14px;margin:0 0 16px}.metric{margin:13px 0}.row{display:flex;justify-content:space-between;font-size:13px;margin-bottom:5px}
.track{height:10px;background:#dfe3e8}.fill{height:100%;width:0}.roll{background:#e0a526}.yaw{background:#347fc4}.pitch{background:#d65f5f}
#raw{font-family:monospace;font-size:12px;line-height:1.65;margin-top:22px;white-space:pre-line}.ok{color:#16834b}.stale{color:#bd3333}
@media(max-width:760px){main{grid-template-columns:1fr;grid-template-rows:minmax(360px,1fr) auto;height:auto}#view{height:60vh}aside{border-left:0;border-top:1px solid #d8dce2}}
</style></head><body><header>Right SenseGlove skeleton diagnostics</header><main>
<canvas id="view"></canvas><aside><h2>Robot thumb output</h2>
<div class="metric"><div class="row"><span>roll</span><span id="rollv">0.000</span></div><div class="track"><div id="roll" class="fill roll"></div></div></div>
<div class="metric"><div class="row"><span>yaw</span><span id="yawv">0.000</span></div><div class="track"><div id="yaw" class="fill yaw"></div></div></div>
<div class="metric"><div class="row"><span>pitch / flexion</span><span id="pitchv">0.000</span></div><div class="track"><div id="pitch" class="fill pitch"></div></div></div>
<div id="raw">Waiting for right glove data...</div></aside></main>
<script>
const c=document.getElementById('view'),ctx=c.getContext('2d');let data=null,yaw=-.55,pitch=.35,drag=false,last=[0,0];
const colors=['#e0a526','#347fc4','#299768','#d65f5f','#8a63b8'];
function resize(){const d=devicePixelRatio||1;c.width=c.clientWidth*d;c.height=c.clientHeight*d;ctx.setTransform(d,0,0,d,0,0)}addEventListener('resize',resize);resize();
c.onpointerdown=e=>{drag=true;last=[e.clientX,e.clientY];c.setPointerCapture(e.pointerId)};c.onpointerup=()=>drag=false;
c.onpointermove=e=>{if(!drag)return;yaw+=(e.clientX-last[0])*.008;pitch=Math.max(-1.2,Math.min(1.2,pitch+(e.clientY-last[1])*.008));last=[e.clientX,e.clientY]};
function project(p,scale,cx,cy){let x=p[1],y=p[2],z=p[0];let x1=x*Math.cos(yaw)-y*Math.sin(yaw),y1=x*Math.sin(yaw)+y*Math.cos(yaw);let y2=y1*Math.cos(pitch)-z*Math.sin(pitch),z2=y1*Math.sin(pitch)+z*Math.cos(pitch);return[cx+x1*scale,cy-y2*scale,z2]}
function draw(){requestAnimationFrame(draw);const w=c.clientWidth,h=c.clientHeight;ctx.clearRect(0,0,w,h);if(!data||data.points.length!==20){ctx.fillStyle='#6d737c';ctx.font='15px Arial';ctx.fillText('Waiting for live skeleton...',24,34);return}
let extent=Math.max(80,...data.points.flat().map(Math.abs)),scale=Math.min(w*.72,h*.78)/(extent*2),cx=w*.5,cy=h*.55;
ctx.strokeStyle='#dfe3e8';ctx.lineWidth=1;ctx.beginPath();ctx.moveTo(24,cy);ctx.lineTo(w-24,cy);ctx.stroke();
const origin=[0,0,0];for(let f=0;f<5;f++){let chain=[origin,...data.points.slice(f*4,f*4+4)].map(p=>project(p,scale,cx,cy));ctx.strokeStyle=colors[f];ctx.fillStyle=colors[f];ctx.lineWidth=f===0?4:3;ctx.beginPath();chain.forEach((p,i)=>i?ctx.lineTo(p[0],p[1]):ctx.moveTo(p[0],p[1]));ctx.stroke();chain.slice(1).forEach((p,i)=>{ctx.beginPath();ctx.arc(p[0],p[1],f===0?5:4,0,Math.PI*2);ctx.fill();if(f===0){ctx.fillStyle='#594200';ctx.font='11px Arial';ctx.fillText(String(i),p[0]+7,p[1]-6);ctx.fillStyle=colors[f]}})}
let palm=[0,4,8,12,16].map(i=>project(data.points[i],scale,cx,cy));ctx.strokeStyle='#555';ctx.lineWidth=2;ctx.beginPath();palm.forEach((p,i)=>i?ctx.lineTo(p[0],p[1]):ctx.moveTo(p[0],p[1]));ctx.stroke();
ctx.fillStyle='#555';ctx.font='12px Arial';ctx.fillText('Drag to rotate | yellow: thumb',18,h-18)}draw();
function value(id,v){v=Math.max(0,Math.min(1,v||0));document.getElementById(id).style.width=(v*100)+'%';document.getElementById(id+'v').textContent=v.toFixed(3)}
async function poll(){try{const r=await fetch('/data',{cache:'no-store'});data=await r.json();value('roll',data.channels.roll);value('yaw',data.channels.yaw);value('pitch',data.channels.pitch);let age=(Date.now()/1000-data.timestamp),live=age<1;document.getElementById('raw').innerHTML=`<span class="${live?'ok':'stale'}">${live?'LIVE':'STALE'} ${age.toFixed(2)} s</span>\nSDK flex (pitch): ${data.features.sdk_flex.toFixed(3)} rad\nSDK abduction (yaw): ${data.features.sdk_abduction.toFixed(3)} rad\nProximal elevation (roll): ${data.features.proximal_elevation.toFixed(3)} rad\nInternal curl: ${data.features.internal_curl.toFixed(3)} rad\nTip spread (diagnostic only): ${data.features.tip_spread.toFixed(3)} rad\nTip elevation (diagnostic only): ${data.features.tip_elevation.toFixed(3)} rad\n\nThumb-tip distance (mm):\nindex=${data.features.tip_distance_index_mm.toFixed(1)} middle=${data.features.tip_distance_middle_mm.toFixed(1)}\nring=${data.features.tip_distance_ring_mm.toFixed(1)} pinky=${data.features.tip_distance_pinky_mm.toFixed(1)}\n\nInput span (3 s):\nSDK flex: ${data.spans.sdk_flex.toFixed(4)} rad\nInternal curl: ${data.spans.internal_curl.toFixed(4)} rad\n\nTarget rad:\n[${data.targets.map(v=>v.toFixed(3)).join(', ')}]`}catch(e){}setTimeout(poll,50)}poll();
</script></body></html>"""

    def __init__(self, rate_hz: float, host: str = "0.0.0.0", port: int = 8766):
        import http.server
        import threading

        self.period = 1.0 / rate_hz
        self.next_update = 0.0
        self._lock = threading.Lock()
        self._payload: dict[str, Any] = {
            "timestamp": 0.0, "points": [], "channels": {}, "features": {},
            "spans": {"sdk_flex": 0.0, "internal_curl": 0.0}, "targets": []
        }
        self._thumb_history: list[tuple[float, float, float]] = []
        owner = self

        class Handler(http.server.BaseHTTPRequestHandler):
            def do_GET(self) -> None:
                if self.path == "/data":
                    with owner._lock:
                        body = json.dumps(owner._payload, separators=(",", ":")).encode()
                    content_type = "application/json"
                elif self.path in ("/", "/index.html"):
                    body = owner.HTML.encode()
                    content_type = "text/html; charset=utf-8"
                else:
                    self.send_error(404)
                    return
                self.send_response(200)
                self.send_header("Content-Type", content_type)
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def log_message(self, _format: str, *_args: Any) -> None:
                return

        bind_error: OSError | None = None
        for candidate_port in range(port, min(port + 10, 65536)):
            try:
                self.server = http.server.ThreadingHTTPServer(
                    (host, candidate_port), Handler
                )
                break
            except OSError as exc:
                bind_error = exc
        else:
            raise bind_error or OSError("no visualizer port available")
        display_host = host
        if host == "0.0.0.0":
            ssh_parts = os.environ.get("SSH_CONNECTION", "").split()
            display_host = ssh_parts[2] if len(ssh_parts) >= 3 else "127.0.0.1"
        self.url = f"http://{display_host}:{self.server.server_address[1]}"
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def update(
        self,
        hand_positions_mm: list[list[float]],
        features: dict[str, float],
        diagnostics: dict[str, float],
        targets_rad: list[float],
        now: float,
    ) -> None:
        if now < self.next_update or len(hand_positions_mm) != 20:
            return
        self.next_update = now + self.period
        points = canonicalize_hand_points_for_robot(hand_positions_mm, "right")
        if len(points) != 20:
            return
        sdk_flex = float(features.get("thumb", 0.0))
        internal_curl = float(features.get("vector_thumb_internal_curl", 0.0))
        wall_now = time.time()
        self._thumb_history.append((wall_now, sdk_flex, internal_curl))
        self._thumb_history = [
            sample for sample in self._thumb_history if wall_now - sample[0] <= 3.0
        ]
        sdk_values = [sample[1] for sample in self._thumb_history]
        internal_values = [sample[2] for sample in self._thumb_history]
        payload = {
            "timestamp": wall_now,
            "points": points,
            "channels": {
                "roll": clamp01(diagnostics.get("thumb_roll", 0.0)),
                "yaw": clamp01(diagnostics.get("thumb_yaw", 0.0)),
                "pitch": clamp01(diagnostics.get("thumb_pitch", 0.0)),
            },
            "features": {
                "sdk_flex": sdk_flex,
                "internal_curl": internal_curl,
                "sdk_abduction": features.get("thumb_brake", 0.0),
                "proximal_elevation": features.get("vector_thumb_elevation", 0.0),
                "tip_spread": features.get("vector_thumb_tip_spread", 0.0),
                "tip_elevation": features.get("vector_thumb_tip_elevation", 0.0),
                "tip_distance_index_mm": features.get(
                    "thumb_index_tip_distance_mm", 0.0
                ),
                "tip_distance_middle_mm": features.get(
                    "thumb_middle_tip_distance_mm", 0.0
                ),
                "tip_distance_ring_mm": features.get(
                    "thumb_ring_tip_distance_mm", 0.0
                ),
                "tip_distance_pinky_mm": features.get(
                    "thumb_pinky_tip_distance_mm", 0.0
                ),
            },
            "spans": {
                "sdk_flex": max(sdk_values) - min(sdk_values),
                "internal_curl": max(internal_values) - min(internal_values),
            },
            "targets": [float(value) for value in targets_rad],
        }
        with self._lock:
            self._payload = payload

    def close(self) -> None:
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=1.0)


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    try:
        import rclpy
        from rclpy.executors import ExternalShutdownException
        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
        from senseglove_msgs.msg import SenseGloveState
    except ImportError as exc:
        print(
            f"[senseglove_bridge] Missing ROS 2 Python module: {exc}",
            file=sys.stderr,
            flush=True,
        )
        print(
            "[senseglove_bridge] Source /opt/ros/humble/setup.bash and the built senseglove_ros install/setup.bash.",
            file=sys.stderr,
            flush=True,
        )
        return 2

    rclpy.init(args=None)
    node = rclpy.create_node("senseglove_to_esrobo_hand_bridge")
    qos_profile = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=args.qos_depth,
        reliability=ReliabilityPolicy.BEST_EFFORT,
    )
    bridge = SenseGloveUdpBridge(node, SenseGloveState, args, qos_profile)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print(f"[senseglove_bridge] Subscribing left topic: {args.left_topic}", flush=True)
    print(
        f"[senseglove_bridge] Subscribing right topic: {args.right_topic}", flush=True
    )
    print(
        f"[senseglove_bridge] Subscription QoS: best_effort keep_last depth={args.qos_depth}.",
        flush=True,
    )
    print(
        f"[senseglove_bridge] Sending ESROBO hand joints to UDP {args.host}:{args.port}.",
        flush=True,
    )
    print(
        "[senseglove_bridge] Hand retargeting: "
        f"mode={args.retargeting_mode}, robot_active_joints=10/hand, "
        f"vector_spread_range={args.vector_spread_range_deg:.1f}deg, "
        f"spread_scale={args.spread_scale:.2f}, flexion_gain={args.finger_flexion_gain:.2f}, "
        f"flexion_exponent={args.finger_flexion_exponent:.2f}.",
        flush=True,
    )
    if args.single_side == "right":
        print(
            "[senseglove_bridge] Right-hand response: "
            f"finger_gain={args.right_finger_flexion_gain:.2f}, "
            f"finger_exponent={args.right_finger_flexion_exponent:.2f}, "
            f"thumb_flexion_gain={args.right_thumb_flexion_gain:.2f}, "
            f"thumb_flexion_exponent={args.right_thumb_flexion_exponent:.2f}, "
            f"thumb_roll/yaw_gain={args.right_thumb_roll_gain:.2f}/"
            f"{args.right_thumb_yaw_gain:.2f}.",
            flush=True,
        )
    elif args.single_side == "left":
        print(
            "[senseglove_bridge] Left-hand response: "
            f"finger_gain={args.finger_flexion_gain:.2f}, "
            f"finger_exponent={args.finger_flexion_exponent:.2f}, "
            f"thumb_flexion_gain={args.thumb_flexion_gain:.2f}, "
            f"thumb_flexion_exponent={args.thumb_flexion_exponent:.2f}, "
            f"thumb_roll/yaw_gain={args.thumb_roll_gain:.2f}/"
            f"{args.thumb_yaw_gain:.2f}.",
            flush=True,
        )
    print(
        "[senseglove_bridge] Hand IMU orientation: "
        f"{'disabled' if args.disable_imu_orientation else 'enabled'} "
        f"(correction={args.imu_correction}, w_repair={args.imu_w_repair}).",
        flush=True,
    )
    print(
        "[senseglove_bridge] Robot hand mapping: "
        f"left<={'right_glove' if args.swap_left_right_targets else 'left_glove'} "
        f"right<={'left_glove' if args.swap_left_right_targets else 'right_glove'}.",
        flush=True,
    )
    if args.single_side:
        print(
            f"[senseglove_bridge] Single-glove mode: {args.single_side}; the other side is mirrored internally only.",
            flush=True,
        )
    if args.record_path is not None:
        print(
            "[senseglove_bridge] Hand recording requested: "
            f"path={args.record_path.expanduser()} start_delay={args.record_start_delay_s:.1f}s "
            f"duration={'until Ctrl-C' if args.record_duration_s <= 0.0 else f'{args.record_duration_s:.1f}s'}.",
            flush=True,
        )

    record_writer: HandRecordingWriter | None = None
    viewer: RightHandSkeletonViewer | None = None
    try:
        calibration = load_or_collect_calibration(bridge, node, args)
        if args.calibrate_only:
            print("[senseglove_bridge] Calibration-only mode: done. Exiting.", flush=True)
            return 0
        bridge.seed_imu_continuity_from_calibration(calibration)
        if args.record_path is not None:
            record_writer = HandRecordingWriter(args.record_path, args, calibration)
        if args.visualize_right_hand:
            try:
                viewer = RightHandSkeletonViewer(
                    args.visualizer_rate_hz, args.visualizer_host, args.visualizer_port
                )
                print(
                    f"[senseglove_bridge] Right-hand skeleton dashboard: {viewer.url}",
                    flush=True,
                )
                print(
                    "[senseglove_bridge] Open that URL from a browser on the robot LAN; "
                    "no nested SSH command is required.",
                    flush=True,
                )
            except Exception as exc:
                print(
                    f"[senseglove_bridge] WARNING: cannot open skeleton window: {exc}",
                    file=sys.stderr,
                    flush=True,
                )
        # Runtime endpoint expansion must not rewrite the persisted calibration.
        calibration = copy.deepcopy(calibration)
        period = 1.0 / args.rate_hz
        next_send = time.monotonic()
        next_print = 0.0
        sent_count = 0
        last_print_sent = 0
        last_print_time = time.monotonic()
        disconnected_since: float | None = None
        while rclpy.ok():
            now = time.monotonic()
            spin_timeout = min(max(next_send - now, 0.0), 0.01)
            spin_once(node, timeout_sec=spin_timeout)
            now = time.monotonic()
            if now < next_send:
                continue
            next_send += period
            if next_send < now - period:
                next_send = now + period

            if bridge.has_explicit_disconnect():
                if disconnected_since is None:
                    disconnected_since = now
                    print(
                        "[senseglove_bridge] SAFETY STOP: glove connection lost; "
                        "stale hand/IMU targets will not be sent.",
                        file=sys.stderr,
                        flush=True,
                    )
                if now - disconnected_since >= 1.0:
                    raise RuntimeError(
                        "SenseGlove remained disconnected for 1.0 s; automatic resume is "
                        "disabled. Power on and reconnect the glove, then restart this "
                        "launcher to recalibrate and manually re-enable teleoperation"
                    )
                continue
            disconnected_since = None

            if not bridge.has_both_hands(args.max_state_age):
                if args.print_interval > 0.0 and now >= next_print:
                    print(
                        f"[senseglove_bridge] Waiting for fresh glove data: {bridge.topic_status()}.",
                        flush=True,
                    )
                    next_print = now + args.print_interval
                continue

            targets, curls, orientation_deltas, packet = bridge.send(sock, calibration)
            sent_count += 1
            if viewer is not None:
                viewer.update(
                    bridge.right.hand_positions_mm,
                    bridge.right.features,
                    curls["right"],
                    targets[10:13],
                    now,
                )
            recording_status = "off"
            if record_writer is not None:
                recording_status = record_writer.update(
                    now, packet, bridge.raw_recording_snapshot()
                )

            if args.print_interval > 0.0 and now >= next_print:
                elapsed = max(now - last_print_time, 1.0e-6)
                hz = (sent_count - last_print_sent) / elapsed
                print(
                    "[senseglove_bridge] "
                    f"sent={sent_count} hz={hz:.1f} "
                    f"imu={'on' if orientation_deltas else 'off'} "
                    f"imu_w_repair=left:{bridge.left.imu_w_repaired}/right:{bridge.right.imu_w_repaired} "
                    f"skeleton={'sgcore_5x4' if len(bridge.left.hand_positions_mm) == 20 and len(bridge.right.hand_positions_mm) == 20 else 'unavailable'} "
                    f"recording={recording_status} "
                    f"left_curl={format_feature_preview(curls['left'])} "
                    f"right_curl={format_feature_preview(curls['right'])} "
                    f"thumb_targets=left:{[round(value, 3) for value in targets[:3]]}"
                    f"/right:{[round(value, 3) for value in targets[10:13]]}",
                    flush=True,
                )
                if args.print_raw:
                    print(
                        "[senseglove_bridge raw] "
                        f"left={format_feature_preview(bridge.left.features)} "
                        f"right={format_feature_preview(bridge.right.features)} "
                        f"targets0={[round(value, 4) for value in targets[:6]]} "
                        f"left_imu_delta={format_quat_preview(orientation_deltas.get('left'))} "
                        f"right_imu_delta={format_quat_preview(orientation_deltas.get('right'))}",
                        flush=True,
                    )
                last_print_time = now
                last_print_sent = sent_count
                next_print = now + args.print_interval
    except KeyboardInterrupt:
        print("\n[senseglove_bridge] stopped.", flush=True)
        return 130 if args.calibrate_only else 0
    except ExternalShutdownException:
        print("\n[senseglove_bridge] stopped.", flush=True)
    except RuntimeError as exc:
        if "Unable to convert call argument to Python object" not in str(exc):
            raise
        print("\n[senseglove_bridge] stopped during ROS 2 shutdown.", flush=True)
    except OSError as exc:
        print(f"[senseglove_bridge] ERROR: {exc}", file=sys.stderr, flush=True)
        return 2
    finally:
        if viewer is not None:
            viewer.close()
        if record_writer is not None:
            record_writer.close()
            if record_writer.sequence > 0 and not record_writer.complete:
                duration = (
                    time.monotonic() - record_writer.started_monotonic
                    if record_writer.started_monotonic is not None
                    else 0.0
                )
                print(
                    "[senseglove_bridge] RECORDING SAVED: "
                    f"path={record_writer.path} frames={record_writer.sequence} duration={duration:.3f}s.",
                    flush=True,
                )
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        sock.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
