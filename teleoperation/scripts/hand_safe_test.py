#!/usr/bin/env python
"""Safe per-joint LinkerHand bring-up / calibration test.

Verifies the physical hand's joint order, direction and the rad->0..255 mapping
BEFORE running teleoperation.  It drives the LinkerHand through the same
``/cb_{left|right}_hand_control_cmd`` path used by teleop, one joint at a time,
so you can confirm which 0..255 index maps to which finger and which direction
is "open" vs "closed".

Usage (system ROS2 + LinkerHand driver running):
    python3 scripts/hand_safe_test.py --side left --joint 3 --confirm-motion
    python3 scripts/hand_safe_test.py --side left --sweep --confirm-motion

Safety:
    * requires fresh physical-hand feedback and explicit motion confirmation
    * moves relative to the measured pose, never from an assumed home pose
    * always keeps the hand from exceeding 0..255
    * relative delta is restricted to 1..15 servo units
"""

from __future__ import annotations

import argparse
import time

import numpy as np

from esrobo_teleop.config import build_config, load_config
from esrobo_teleop.robot.linker_hand_driver import LinkerHandDriver


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--side", choices=["left", "right"], default="left")
    ap.add_argument("--config", default="")
    ap.add_argument("--joint", type=int, default=None, help="single physical joint index to test")
    ap.add_argument("--sweep", action="store_true", help="sweep all joints one at a time")
    ap.add_argument("--delta", type=int, default=8, help="relative servo units to move (default 8)")
    ap.add_argument("--period", type=float, default=1.5, help="seconds to hold each joint")
    ap.add_argument(
        "--confirm-motion",
        action="store_true",
        help="required acknowledgement that this command moves real hardware",
    )
    args = ap.parse_args(argv)

    cfg = build_config()
    if args.config:
        cfg = load_config(args.config)
    if not args.confirm_motion:
        ap.error("refusing real motion without --confirm-motion")
    if not 1 <= args.delta <= 15:
        ap.error("--delta must be between 1 and 15 servo units")
    cfg.hand.enable_on_start = False

    print(f"[hand_test] model={cfg.hand.model} side={args.side} "
          f"physical joints/hand={10 if cfg.hand.model.upper()=='L10' else 20}")
    driver = LinkerHandDriver(cfg.hand, udp_port=cfg.hand.udp_port)
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline and not driver.set_enabled(True):
        time.sleep(0.05)
    if not driver.is_enabled():
        print("[hand_test] REFUSED: fresh left/right hand feedback is unavailable", flush=True)
        driver.close()
        return 2
    base_left = driver._feedback["left"].copy()
    base_right = driver._feedback["right"].copy()

    def move(index: int) -> None:
        cmd_left = base_left.copy()
        cmd_right = base_right.copy()
        target = cmd_left if args.side == "left" else cmd_right
        direction = -1.0 if target[index] >= 128.0 else 1.0
        target[index] = float(np.clip(target[index] + direction * args.delta, 0, 255))
        left = cmd_left
        right = cmd_right
        left_safe = driver._apply_safety("left", left)
        right_safe = driver._apply_safety("right", right)
        driver._send_physical(left_safe, right_safe)
        print(f"[hand_test] joint[{index}] relative delta={direction * args.delta:+.0f} "
              f"(holding {args.period}s)", flush=True)
        time.sleep(args.period)
        left_safe = driver._apply_safety("left", base_left)
        right_safe = driver._apply_safety("right", base_right)
        driver._send_physical(left_safe, right_safe)
        time.sleep(0.5)

    if args.joint is not None:
        move(args.joint)
    elif args.sweep:
        n = 10 if cfg.hand.model.upper() == "L10" else 20
        for i in range(n):
            move(i)
    else:
        ap.error("provide --joint N or --sweep")

    left_safe = driver._apply_safety("left", base_left)
    right_safe = driver._apply_safety("right", base_right)
    driver._send_physical(left_safe, right_safe)
    print("[hand_test] done. Record which joint index maps to which finger and the direction.", flush=True)
    driver.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
