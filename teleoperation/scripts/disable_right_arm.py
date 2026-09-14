#!/usr/bin/env python3
"""Connect to one NERO over CAN, disable every joint, then disconnect."""

from __future__ import annotations

import argparse
import sys
import time

from esrobo_teleop.config import load_config
from esrobo_teleop.robot.nero_driver import NeroArm


def main() -> int:
    parser = argparse.ArgumentParser(description="Force one NERO arm into disabled state")
    parser.add_argument("--config", required=True)
    parser.add_argument("--side", choices=("left", "right"), default="right")
    parser.add_argument(
        "--release-leader-mode",
        action="store_true",
        help=(
            "switch from leader/follower to normal mode before disabling; "
            "the operator must support the arm first"
        ),
    )
    args = parser.parse_args()

    cfg = load_config(args.config).robot
    channel = cfg.left_can_channel if args.side == "left" else cfg.right_can_channel
    arm = NeroArm(cfg, channel, args.side)
    try:
        arm.connect()
        time.sleep(0.3)
        initial_states = arm.get_joint_enable_states()
        if initial_states is not None and not any(initial_states):
            print(
                f"[{args.side}-arm] all seven joints are already DISABLED; "
                "feedback remains active and no mode/disable command was sent.",
                flush=True,
            )
            return 0
        if args.release_leader_mode:
            print(
                f"[{args.side}-arm] Releasing leader/follower mode; "
                "no position target will be sent.",
                flush=True,
            )
            arm.set_normal_mode()
            time.sleep(0.25)
        if not arm.disable(timeout=5.0):
            states = arm.last_disable_states
            if states is None:
                detail = "no complete seven-joint enable-state feedback"
            else:
                enabled = [str(index) for index, value in enumerate(states, start=1) if value]
                detail = (
                    "still-enabled joints=" + ",".join(enabled)
                    if enabled
                    else "state feedback changed before final verification"
                )
            print(
                f"[{args.side}-arm] ERROR: all-joint disable was not verified; "
                f"{detail}. Keep supporting the arm and do not continue teleoperation.",
                file=sys.stderr,
            )
            return 1
        print(
            f"[{args.side}-arm] {args.side.capitalize()} NERO is DISABLED; "
            "no motion target was sent.",
            flush=True,
        )
        return 0
    except Exception as exc:  # noqa: BLE001
        print(f"[{args.side}-arm] ERROR: failed to disable NERO: {exc}", file=sys.stderr)
        return 1
    finally:
        arm.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
