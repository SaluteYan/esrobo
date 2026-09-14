#!/usr/bin/env python3
"""Read-only validation of XRoboToolkit PC Service and PICO body tracking."""

from __future__ import annotations

import argparse
import math
import sys
import time


REQUIRED_JOINTS_BY_SIDE = {
    "left": {16: "left_shoulder", 18: "left_elbow", 20: "left_wrist"},
    "right": {17: "right_shoulder", 19: "right_elbow", 21: "right_wrist"},
}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--wait-seconds", type=float, default=20.0)
    parser.add_argument("--side", choices=("left", "right"), default="left")
    args = parser.parse_args()
    required_joints = REQUIRED_JOINTS_BY_SIDE[args.side]

    try:
        import xrobotoolkit_sdk as xrt
    except ImportError as exc:
        print(f"ERROR: xrobotoolkit_sdk is unavailable: {exc}", file=sys.stderr)
        return 2

    deadline = time.monotonic() + max(0.0, args.wait_seconds)
    previous_timestamp = None
    xrt.init()
    try:
        while time.monotonic() <= deadline:
            if xrt.is_body_data_available():
                poses = list(xrt.get_body_joints_pose() or [])
                invalid = []
                for index, name in required_joints.items():
                    if index >= len(poses):
                        invalid.append(name)
                        continue
                    values = list(poses[index])
                    if len(values) < 7 or not all(math.isfinite(float(v)) for v in values[:7]):
                        invalid.append(name)
                        continue
                    quaternion_norm = math.sqrt(sum(float(v) ** 2 for v in values[3:7]))
                    if quaternion_norm < 0.5:
                        invalid.append(name)
                body_timestamp = int(xrt.get_body_timestamp_ns())
                xr_timestamp = int(xrt.get_time_stamp_ns())
                timestamp = body_timestamp if body_timestamp > 0 else xr_timestamp
                timestamp_source = "body" if body_timestamp > 0 else "xr-fallback"
                positions_nonzero = any(
                    abs(float(value)) > 1.0e-4
                    for index in required_joints
                    if index < len(poses)
                    for value in poses[index][:3]
                )
                complete = len(poses) >= 24 and not invalid and positions_nonzero and timestamp > 0
                timestamp_advanced = (
                    complete
                    and previous_timestamp is not None
                    and timestamp > previous_timestamp
                )
                if timestamp_advanced:
                    print(
                        "PICO BODY DATA READY: joints=24, "
                        f"{args.side}_shoulder/elbow/wrist=valid, "
                        f"timestamp_ns={timestamp} ({timestamp_source})",
                        flush=True,
                    )
                    return 0
                if complete:
                    previous_timestamp = timestamp
                print(
                    f"Waiting for complete {args.side}-arm data: joints={len(poses)}, "
                    f"invalid={invalid}, positions_nonzero={positions_nonzero}, "
                    f"body_timestamp_ns={body_timestamp}, xr_timestamp_ns={xr_timestamp}",
                    flush=True,
                )
            else:
                print("Waiting for PICO full-body data...", flush=True)
            time.sleep(1.0)
    finally:
        xrt.close()

    print(
        f"ERROR: no fresh PICO {args.side} shoulder/elbow/wrist data. Check the headset app, "
        "full-body tracking, trackers, and network. Body timestamp 0 is accepted only "
        "when the XR fallback timestamp is positive and advancing.",
        file=sys.stderr,
    )
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
