#!/usr/bin/env python3
"""Capture raw SenseGlove ROS IMU samples for a vendor support report."""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from senseglove_msgs.msg import SenseGloveState


class ImuCapture(Node):
    def __init__(self, topic: str, output: Path, duration_s: float) -> None:
        super().__init__("senseglove_raw_imu_capture")
        self.output = output
        self.duration_s = duration_s
        self.started = time.monotonic()
        self.samples = 0
        self.exact_z_eq_w = 0
        self.near_z_eq_w = 0
        self.nonfinite = 0
        self.norm_min = math.inf
        self.norm_max = -math.inf
        self.first_ros_stamp_ns: int | None = None
        self.last_ros_stamp_ns: int | None = None
        self.stream = output.open("w", encoding="utf-8", buffering=1)
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(SenseGloveState, topic, self.callback, qos)

    def callback(self, msg: SenseGloveState) -> None:
        q = msg.imu_orientation
        values = [float(q.x), float(q.y), float(q.z), float(q.w)]
        finite = all(math.isfinite(value) for value in values)
        exact = finite and values[2] == values[3]
        near = finite and math.isclose(values[2], values[3], rel_tol=1e-6, abs_tol=1e-6)
        norm = math.sqrt(sum(value * value for value in values)) if finite else None
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(
            msg.header.stamp.nanosec
        )
        item = {
            "sequence": self.samples,
            "received_unix_ns": time.time_ns(),
            "ros_stamp_ns": stamp_ns,
            "imu_orientation_raw_xyzw": values,
            "quaternion_norm_raw": norm,
            "z_equals_w_exact": exact,
            "z_equals_w_near_1e-6": near,
            "connected": bool(msg.connected),
            "packets_per_second_received": int(msg.packets_per_second_received),
            "firmware_version": int(msg.firmware_version),
            "sub_firmware_version": int(msg.sub_firmware_version),
            "connection_type": int(msg.connection_type),
        }
        self.stream.write(json.dumps(item, separators=(",", ":")) + "\n")
        self.samples += 1
        self.exact_z_eq_w += int(exact)
        self.near_z_eq_w += int(near)
        self.nonfinite += int(not finite)
        if norm is not None:
            self.norm_min = min(self.norm_min, norm)
            self.norm_max = max(self.norm_max, norm)
        if self.first_ros_stamp_ns is None:
            self.first_ros_stamp_ns = stamp_ns
        self.last_ros_stamp_ns = stamp_ns

    def finished(self) -> bool:
        return time.monotonic() - self.started >= self.duration_s

    def summary(self, topic: str) -> dict[str, object]:
        elapsed = time.monotonic() - self.started
        ros_elapsed = (
            (self.last_ros_stamp_ns - self.first_ros_stamp_ns) / 1e9
            if self.first_ros_stamp_ns is not None and self.last_ros_stamp_ns is not None
            else 0.0
        )
        return {
            "topic": topic,
            "capture_wall_duration_s": elapsed,
            "capture_ros_duration_s": ros_elapsed,
            "sample_count": self.samples,
            "observed_rate_hz": self.samples / elapsed if elapsed > 0.0 else 0.0,
            "z_equals_w_exact_count": self.exact_z_eq_w,
            "z_equals_w_exact_fraction": (
                self.exact_z_eq_w / self.samples if self.samples else 0.0
            ),
            "z_equals_w_near_1e-6_count": self.near_z_eq_w,
            "z_equals_w_near_1e-6_fraction": (
                self.near_z_eq_w / self.samples if self.samples else 0.0
            ),
            "nonfinite_count": self.nonfinite,
            "raw_quaternion_norm_min": (
                self.norm_min if math.isfinite(self.norm_min) else None
            ),
            "raw_quaternion_norm_max": (
                self.norm_max if math.isfinite(self.norm_max) else None
            ),
            "note": "Values are copied directly from SenseGloveState. No quaternion repair or coordinate conversion was applied.",
        }

    def close(self) -> None:
        self.stream.flush()
        self.stream.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=20.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    samples_path = args.output_dir / "raw_imu_samples.jsonl"
    summary_path = args.output_dir / "imu_summary.json"
    rclpy.init()
    capture = ImuCapture(args.topic, samples_path, max(args.duration, 1.0))
    try:
        while rclpy.ok() and not capture.finished():
            rclpy.spin_once(capture, timeout_sec=0.1)
        summary = capture.summary(args.topic)
        summary_path.write_text(
            json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        print(json.dumps(summary, indent=2, sort_keys=True))
        return 0 if capture.samples > 0 else 2
    finally:
        capture.close()
        capture.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
