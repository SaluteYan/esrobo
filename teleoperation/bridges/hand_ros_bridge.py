#!/usr/bin/env python
"""Forward LinkerHand joint commands from the teleop node to ROS2 topics.

The teleop core runs in the conda env (no ``rclpy``).  When the hand driver is
in ``udp`` mode it sends 20-value left/right joint arrays over loopback UDP
(port 15051 by default).  This bridge runs under system ROS2 (Humble) and
publishes them to the LinkerHand node topics:

    /cb_left_hand_control_cmd   /cb_right_hand_control_cmd   (sensor_msgs/JointState)

Usage (system ROS2, after sourcing your workspace):
    source /opt/ros/humble/setup.bash
    ros2 launch linker_hand_ros2_sdk linker_hand_double.launch.py
    python3 bridges/hand_ros_bridge.py --port 15051
"""

from __future__ import annotations

import argparse
import json
import math
import socket
import time
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from esrobo_teleop.robot.hand_transport import PREFIX, validate, emit, make_log
from std_msgs.msg import String

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class HandBridge(Node):
    def __init__(self, udp_host: str, udp_port: int, feedback_host: str, feedback_port: int,
                 left_topic: str, right_topic: str, left_state_topic: str,
                 right_state_topic: str, side: str, feedback_only: bool = False):
        super().__init__("esrobo_hand_ros_bridge")
        self._feedback_only = feedback_only
        self._transport_log = make_log("bridge")
        if not feedback_only:
            self._pub_left = self.create_publisher(JointState, left_topic, 1)
            self._pub_right = self.create_publisher(JointState, right_topic, 10)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        if not feedback_only:
            self._sock.bind((udp_host, udp_port))
        # Never block ROS feedback handling while waiting for an optional hand
        # command.  The old 50 ms recv timeout meant one ROS callback followed
        # by 50 ms asleep; with left/right callbacks alternating, each hand's
        # nominal 60 Hz state stream reached the gateway at only about 10 Hz.
        self._sock.setblocking(False)
        self._feedback_addr = (feedback_host, feedback_port)
        self._active_sides = ("left", "right") if side == "both" else (side,)
        self._feedback_counts = {side: 0 for side in self._active_sides}
        self._feedback_rate_started = time.monotonic()
        if "left" in self._active_sides:
            self.create_subscription(
                String, '/esrobo_left_hand_transport', self._send_left_feedback, 1
            )
        if "right" in self._active_sides:
            self.create_subscription(
                JointState, right_state_topic, lambda msg: self._send_feedback("right", msg), 1
            )
        self.get_logger().info(f"listening on {udp_host}:{udp_port}")

    def spin_once(self) -> None:
        # A short ROS wait prevents busy-spinning yet services both 60 Hz state
        # topics with ample headroom.  UDP command receive below is nonblocking.
        rclpy.spin_once(self, timeout_sec=0.002)
        if self._feedback_only:
            return
        latest = None
        # Bound receive work even under a flood, while servicing ROS each pass.
        for _ in range(64):
            try:
                payload, _addr = self._sock.recvfrom(65535)
            except OSError:
                break
            try:
                msg = json.loads(payload.decode('utf-8'))
                if not isinstance(msg, dict): continue
                meta = msg.get('_hand_transport')
                if 'left' in self._active_sides and (meta is not None or 'left' in msg):
                    try:
                        validate(meta, time.monotonic())
                        values = msg.get('left', [])
                        if meta['action'] == 'target' and (
                                not isinstance(values, list) or len(values) != 10 or
                                any(type(v) not in (int, float) or not math.isfinite(v)
                                    or not 0 <= v <= 255 for v in values)):
                            raise ValueError('invalid left hand positions')
                        if latest is None or meta['issued_s'] >= latest[0]['issued_s']:
                            if latest is not None:
                                emit(self._transport_log, 'bridge_superseded', meta=latest[0],
                                     replacement_seq=meta['seq'])
                            latest = (meta, values if meta['action'] == 'target' else [], time.monotonic())
                    except ValueError as exc:
                        emit(self._transport_log, 'bridge_rejected', reason=str(exc))
                if 'right' in self._active_sides:
                    values = msg.get('right')
                    if isinstance(values, list) and values:
                        js = JointState()
                        js.position = [float(v) for v in values]
                        self._pub_right.publish(js)
            except (ValueError, TypeError, UnicodeDecodeError):
                continue
        if latest is not None:
            meta, values, received = latest
            js = JointState()
            js.header.frame_id = PREFIX + json.dumps(meta, separators=(',', ':'))
            js.position = [float(v) for v in values]
            self._pub_left.publish(js)
            emit(self._transport_log, 'bridge_publish', meta=meta, received_s=received,
                 published_s=time.monotonic())

    def _send_left_feedback(self, msg):
        try:
            state = json.loads(msg.data)
            if state.get('version') != 1 or state.get('side') != 'left': return
            stamps = state['frame_stamps']
            valid = len(stamps) == 2 and all(isinstance(t, (float, int)) for t in stamps)
            # Fault-only packets must still reach the gateway without feedback.
            payload = dict(side='left', transport_version=1, position=state['position'],
                           sample_monotonic_s=min(stamps) if valid else None,
                           frame_stamps=stamps, frame_sequences=state['frame_sequences'],
                           fault=state.get('fault'), sent=state.get('sent'),
                           finish_protocol=state.get('finish_protocol'), finished=state.get('finished'))
            self._sock.sendto(json.dumps(payload, allow_nan=False).encode(), self._feedback_addr)
        except (KeyError, ValueError, TypeError):
            return

    def _send_feedback(self, side: str, msg: JointState) -> None:
        now = time.monotonic()
        payload = json.dumps({"side": side, "position": list(msg.position), "sample_monotonic_s": now}).encode("utf-8")
        self._sock.sendto(payload, self._feedback_addr)
        self._feedback_counts[side] += 1
        elapsed = now - self._feedback_rate_started
        if elapsed >= 5.0:
            rates = ", ".join(
                f"{name}={self._feedback_counts[name] / elapsed:.1f} Hz"
                for name in self._active_sides
            )
            self.get_logger().info(f"feedback forward rate: {rates}")
            self._feedback_counts = {name: 0 for name in self._active_sides}
            self._feedback_rate_started = now

    def close(self) -> None:
        self._sock.close()
        self._transport_log.close(timeout=1.)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=15051)
    ap.add_argument("--feedback-host", default="127.0.0.1")
    ap.add_argument("--feedback-port", type=int, default=15052)
    ap.add_argument("--left-topic", default="/cb_left_hand_control_cmd")
    ap.add_argument("--right-topic", default="/cb_right_hand_control_cmd")
    ap.add_argument("--left-state-topic", default="/cb_left_hand_state")
    ap.add_argument("--right-state-topic", default="/cb_right_hand_state")
    ap.add_argument("--side", choices=("both", "left", "right"), default="both")
    ap.add_argument("--feedback-only", action="store_true", help="Subscribe only; no command publisher/socket")
    args = ap.parse_args(argv)

    rclpy.init()
    node = HandBridge(
        args.host, args.port, args.feedback_host, args.feedback_port,
        args.left_topic, args.right_topic, args.left_state_topic, args.right_state_topic,
        args.side, args.feedback_only,
    )
    try:
        while rclpy.ok():
            node.spin_once()
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        # ROS signal handling may already have shut the default context down.
        # Avoid an RCLError during otherwise clean launcher cleanup.
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
