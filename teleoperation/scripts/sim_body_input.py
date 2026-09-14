#!/usr/bin/env python
"""Send synthetic PICO full-body + SenseGlove hand UDP packets for offline tests.

Usage (run in the conda env):
    python scripts/sim_body_input.py --host 127.0.0.1 --port 15050

Hold the sender in a natural pose for ~2.5 s so the receiver locks its
auto-start reference, then move the wrist targets to exercise the IK.
"""

from __future__ import annotations

import argparse
import json
import math
import socket
import time

import numpy as np


def _pose(x, y, z, rpy=(0.0, 0.0, 0.0)):
    r, p, yaw = rpy
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(p * 0.5), math.sin(p * 0.5)
    cr, sr = math.cos(r * 0.5), math.sin(r * 0.5)
    w = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return [x, y, z, w, qx, qy, qz]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=15050)
    ap.add_argument("--hz", type=float, default=60.0)
    ap.add_argument("--swing", action="store_true", help="sinusoidal wrist motion")
    args = ap.parse_args(argv)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    period = 1.0 / args.hz
    t0 = time.monotonic()
    hand = [0.3, 0.3, 0.5] * 3 + [0.3, 0.5] * 3 + [0.0] * 2 + [0.3, 0.5] * 2
    hand = (np.asarray(hand, dtype=np.float32)[:20].tolist())
    print("sending to", args.host, args.port, flush=True)
    try:
        while True:
            t = time.monotonic() - t0
            swing = 0.08 * math.sin(t * 0.8)
            frames = {
                "LEFT_SHOULDER": _pose(-0.3, 0.25, 1.5),
                "LEFT_ELBOW": _pose(-0.3 + swing, 0.3, 1.2),
                "LEFT_WRIST": _pose(-0.25 + swing, 0.35, 0.95),
                "RIGHT_SHOULDER": _pose(-0.3, -0.25, 1.5),
                "RIGHT_ELBOW": _pose(-0.3 - swing, -0.3, 1.2),
                "RIGHT_WRIST": _pose(-0.25 - swing, -0.35, 0.95),
                "WAIST": _pose(0.0, 0.0, 0.85),
            }
            msg = {"type": "esrobo_full_body", "frames": frames, "hand_joints": hand}
            sock.sendto(json.dumps(msg).encode("utf-8"), (args.host, args.port))
            time.sleep(period)
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
