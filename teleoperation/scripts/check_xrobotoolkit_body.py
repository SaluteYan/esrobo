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
    previous_sequence = None
    if not hasattr(xrt, "get_body_sample"):
        print("ERROR: rebuild PICO SDK with laptop_teleop/scripts/install_pico.sh", file=sys.stderr)
        return 2
    xrt.init()
    try:
        while time.monotonic() <= deadline:
            sample = xrt.get_body_sample()
            age_s = float(sample["age_s"])
            sequence = int(sample["sequence"])
            if sample["valid"] and math.isfinite(age_s) and 0 <= age_s <= .1:
                poses = list(sample["poses"] or [])
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
                positions_nonzero = any(
                    abs(float(value)) > 1.0e-4
                    for index in required_joints
                    if index < len(poses)
                    for value in poses[index][:3]
                )
                complete = len(poses) >= 24 and not invalid and positions_nonzero
                body_frame_advanced = complete and previous_sequence is not None and sequence > previous_sequence
                if body_frame_advanced:
                    print(
                        "PICO BODY DATA READY: joints=24, "
                        f"{args.side}_shoulder/elbow/wrist=valid, "
                        f"receipt_sequence={sequence}, packet_timestamp_ns={sample['packet_timestamp_ns']}, "
                        f"age_ms={age_s*1000:.1f}",
                        flush=True,
                    )
                    return 0
                if complete:
                    previous_sequence = sequence
                print(
                    f"Waiting for complete {args.side}-arm data: joints={len(poses)}, "
                    f"invalid={invalid}, positions_nonzero={positions_nonzero}, "
                    f"receipt_sequence={sequence}, packet_timestamp_ns={sample['packet_timestamp_ns']}, "
                    f"age_ms={age_s*1000:.1f}",
                    flush=True,
                )
            else:
                print("Waiting for a complete PICO Body component received within 100 ms...", flush=True)
            time.sleep(1.0)
    finally:
        xrt.close()

    print(
        f"ERROR: no fresh PICO {args.side} shoulder/elbow/wrist data. Check the headset app, "
        "full-body tracking, trackers, and network. Only complete advancing Body component "
        "packets renew freshness; stationary poses are valid new frames.",
        file=sys.stderr,
    )
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
