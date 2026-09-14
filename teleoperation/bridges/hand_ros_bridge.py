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
import socket

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class HandBridge(Node):
    def __init__(self, udp_host: str, udp_port: int, feedback_host: str, feedback_port: int,
                 left_topic: str, right_topic: str, left_state_topic: str,
                 right_state_topic: str, side: str):
        super().__init__("esrobo_hand_ros_bridge")
        self._pub_left = self.create_publisher(JointState, left_topic, 10)
        self._pub_right = self.create_publisher(JointState, right_topic, 10)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((udp_host, udp_port))
        self._sock.settimeout(0.05)
        self._feedback_addr = (feedback_host, feedback_port)
        self._active_sides = ("left", "right") if side == "both" else (side,)
        if "left" in self._active_sides:
            self.create_subscription(
                JointState, left_state_topic, lambda msg: self._send_feedback("left", msg), 10
            )
        if "right" in self._active_sides:
            self.create_subscription(
                JointState, right_state_topic, lambda msg: self._send_feedback("right", msg), 10
            )
        self.get_logger().info(f"listening on {udp_host}:{udp_port}")

    def spin_once(self) -> None:
        rclpy.spin_once(self, timeout_sec=0.0)
        try:
            payload, _addr = self._sock.recvfrom(65535)
        except socket.timeout:
            return
        except OSError:
            return
        try:
            msg = json.loads(payload.decode("utf-8"))
        except (ValueError, UnicodeDecodeError):
            return
        for side in self._active_sides:
            values = msg.get(side)
            if not isinstance(values, list) or len(values) == 0:
                continue
            js = JointState()
            js.name = [f"{side}_joint{i}" for i in range(len(values))]
            js.position = [float(v) for v in values]
            (self._pub_left if side == "left" else self._pub_right).publish(js)

    def _send_feedback(self, side: str, msg: JointState) -> None:
        payload = json.dumps({"side": side, "position": list(msg.position)}).encode("utf-8")
        self._sock.sendto(payload, self._feedback_addr)

    def close(self) -> None:
        self._sock.close()


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
    args = ap.parse_args(argv)

    rclpy.init()
    node = HandBridge(
        args.host, args.port, args.feedback_host, args.feedback_port,
        args.left_topic, args.right_topic, args.left_state_topic, args.right_state_topic,
        args.side,
    )
    try:
        while rclpy.ok():
            node.spin_once()
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
