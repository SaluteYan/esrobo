#!/usr/bin/env python3
"""Subscribe to head camera images; never start drivers or send motion commands."""

import argparse
import datetime
import json
from pathlib import Path
import time

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--namespace", default="/camera")
    parser.add_argument("--seconds", type=float, default=15.0)
    parser.add_argument("--wait-seconds", type=float, default=20.0)
    parser.add_argument("--qos", choices=("reliable", "best-effort"), default="reliable")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if not all(np.isfinite(v) and v > 0 for v in (args.seconds, args.wait_seconds)):
        parser.error("durations must be finite and positive")
    output = args.output or Path(__file__).resolve().parents[1] / "log" / (
        "head_camera_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f"))
    output.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    node = rclpy.create_node("head_camera_readonly_check")
    bridge = CvBridge()
    qos = QoSProfile(depth=5) if args.qos == "reliable" else qos_profile_sensor_data
    streams, infos, subscriptions = {}, {}, []
    first_arrival = None

    def receive(message, key):
        nonlocal first_arrival
        now = time.monotonic()
        first_arrival = now if first_arrival is None else first_arrival
        entry = streams.setdefault(key, {"arrivals": [], "stamps": [], "first": message})
        entry["arrivals"].append(now)
        entry["stamps"].append(message.header.stamp.sec + message.header.stamp.nanosec * 1e-9)
        entry["last"] = message

    def info(message, key):
        infos[key] = dict(width=message.width, height=message.height, k=list(message.k),
                          d=list(message.d), frame_id=message.header.frame_id)

    for key in ("color", "depth"):
        topic = args.namespace.rstrip("/") + "/" + key
        subscriptions.append(node.create_subscription(
            Image, topic + "/image_raw", lambda m, k=key: receive(m, k), qos))
        subscriptions.append(node.create_subscription(
            CameraInfo, topic + "/camera_info", lambda m, k=key: info(m, k), qos))
    print(f"Read-only camera check: {args.namespace}; output={output}", flush=True)
    wait_deadline = time.monotonic() + args.wait_seconds
    try:
        while time.monotonic() < (wait_deadline if first_arrival is None else first_arrival + args.seconds):
            rclpy.spin_once(node, timeout_sec=.1)
        report = {"namespace": args.namespace, "qos": args.qos, "requested_seconds": args.seconds,
                  "camera_info": infos, "streams": {}}
        for key, entry in streams.items():
            message = entry["last"]
            array = bridge.imgmsg_to_cv2(message, desired_encoding="passthrough")
            first = bridge.imgmsg_to_cv2(entry["first"], desired_encoding="passthrough")
            intervals = np.diff(entry["arrivals"])
            stamp_intervals = np.diff(entry["stamps"])
            stats = dict(frames=len(entry["arrivals"]), width=message.width, height=message.height,
                         encoding=message.encoding, frame_id=message.header.frame_id,
                         arrival_fps=float(1 / np.mean(intervals)) if len(intervals) else None,
                         max_arrival_gap_s=float(max(intervals)) if len(intervals) else None,
                         nonincreasing_stamps=int(np.sum(stamp_intervals <= 0)),
                         last_arrival_age_s=time.monotonic() - entry["arrivals"][-1],
                         mean=float(array.mean()), std=float(array.std()),
                         first_last_mean_abs_difference=float(np.abs(array.astype(float)-first).mean())
                         if array.shape == first.shape else None,
                         arrival_monotonic_s=entry["arrivals"], header_stamp_s=entry["stamps"])
            if key == "color":
                image = bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
                path = output / "color.png"
                if not cv2.imwrite(str(path), image):
                    raise RuntimeError(f"cannot write {path}")
            else:
                valid = array[array > 0]
                stats["valid_fraction"] = float(np.mean(array > 0))
                stats["raw_depth_p5_p50_p95"] = np.percentile(valid, [5, 50, 95]).tolist() if valid.size else []
                if not cv2.imwrite(str(output / "depth_raw.png"), array):
                    raise RuntimeError("cannot write depth_raw.png")
                preview = cv2.applyColorMap(cv2.convertScaleAbs(array, alpha=255/3000), cv2.COLORMAP_TURBO)
                preview[array == 0] = 0
                if not cv2.imwrite(str(output / "depth_preview.png"), preview):
                    raise RuntimeError("cannot write depth_preview.png")
            report["streams"][key] = stats
        report["received_both_streams"] = all(
            key in streams and len(streams[key]["arrivals"]) >= 2 for key in ("color", "depth"))
        (output / "report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
        summary = {key: {k: v for k, v in stats.items() if k not in ("arrival_monotonic_s", "header_stamp_s")}
                   for key, stats in report["streams"].items()}
        print(json.dumps(summary, indent=2), flush=True)
        print(f"Report: {output / 'report.json'}", flush=True)
        return 0 if report["received_both_streams"] else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
