#!/usr/bin/env python3
"""Read-only capture of a LinkerHand natural-open feedback zero."""

from __future__ import annotations

import argparse
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class OpenFeedbackCapture(Node):
    def __init__(self, topic: str, expected_joints: int) -> None:
        super().__init__("esrobo_hand_open_feedback_capture")
        self.expected_joints = expected_joints
        self.samples: list[np.ndarray] = []
        self.invalid_messages = 0
        self.create_subscription(JointState, topic, self._on_state, 10)

    def _on_state(self, msg: JointState) -> None:
        try:
            values = np.asarray(msg.position, dtype=np.float64).reshape(-1)
        except (TypeError, ValueError):
            self.invalid_messages += 1
            return
        if (
            values.shape != (self.expected_joints,)
            or not np.all(np.isfinite(values))
            or np.any(values < 0.0)
            or np.any(values > 255.0)
        ):
            self.invalid_messages += 1
            return
        self.samples.append(values.copy())


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="Read-only LinkerHand natural-open feedback capture"
    )
    parser.add_argument("--side", choices=("left", "right"), required=True)
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--min-samples", type=int, default=50)
    parser.add_argument("--max-span", type=float, default=4.0)
    parser.add_argument("--joints", type=int, default=10)
    args = parser.parse_args(argv)
    if args.duration <= 0.0 or args.min_samples < 2 or args.joints <= 0:
        parser.error("duration and joints must be positive; min-samples must be at least 2")
    if args.max_span < 0.0:
        parser.error("max-span must be non-negative")

    topic = f"/cb_{args.side}_hand_state"
    print(
        f"[hand_open_capture] READ ONLY: listening to {topic} for {args.duration:.1f}s. "
        "Place the physical hand in a comfortable natural-open pose and keep it still. "
        "No command topic is created.",
        flush=True,
    )
    rclpy.init(args=None)
    node = OpenFeedbackCapture(topic, args.joints)
    deadline = time.monotonic() + args.duration
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        print("[hand_open_capture] cancelled; no configuration was changed.", flush=True)
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if len(node.samples) < args.min_samples:
        print(
            f"[hand_open_capture] REFUSED: received {len(node.samples)} valid samples "
            f"(need {args.min_samples}); invalid messages={node.invalid_messages}. "
            "Check the LinkerHand driver and feedback topic.",
            flush=True,
        )
        return 2

    samples = np.stack(node.samples)
    median = np.median(samples, axis=0)
    span = np.max(samples, axis=0) - np.min(samples, axis=0)
    rounded = np.rint(median).astype(int)
    print(f"[hand_open_capture] valid_samples={len(samples)} invalid={node.invalid_messages}")
    print(f"[hand_open_capture] median={rounded.tolist()}")
    print(f"[hand_open_capture] channel_span={np.round(span, 2).tolist()}")
    if float(np.max(span)) > args.max_span:
        unstable = np.flatnonzero(span > args.max_span).tolist()
        print(
            f"[hand_open_capture] REFUSED: hand feedback was not stable; channels={unstable}, "
            f"allowed_span={args.max_span:.1f}/255. Repeat without moving the hand. "
            "No configuration was changed.",
            flush=True,
        )
        return 3

    print(
        f"[hand_open_capture] STABLE CANDIDATE (review before editing config):\n"
        f"  {args.side}_open_feedback: {rounded.tolist()}\n"
        "This value only defines the measured natural-open zero. It does not provide "
        "feedback-to-URDF collision geometry calibration.",
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
