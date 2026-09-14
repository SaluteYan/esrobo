#!/usr/bin/env python
"""Stand-alone arm bring-up / calibration tool for the two NERO arms.

This talks directly to the physical arms via ``pyAgxArm`` and prints the raw
joint angles / firmware.  It does NOT require the UDP stream or IK.

Commands:
    probe        connect, read firmware + joint angles, then disconnect
    enable       enable motors and set speed
    validate     read physical/URDF angles and verify mapping + SDK soft limits

Always confirm the arm is free to move and keep your hand on an E-stop.
"""

from __future__ import annotations

import argparse
import numpy as np

from esrobo_teleop.config import build_config, load_config
from esrobo_teleop.robot.nero_driver import NeroDualArmDriver


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("command", choices=["probe", "validate"])
    ap.add_argument("--config", default="")
    args = ap.parse_args(argv)

    cfg = build_config()
    if args.config:
        cfg = load_config(args.config)

    driver = NeroDualArmDriver(cfg.robot)
    print("[cal] connecting arms...", flush=True)
    driver.connect()

    if args.command == "probe":
        feedback_ok = True
        for name, arm in (("left", driver._left), ("right", driver._right)):
            fw = arm._robot.get_firmware()
            ja = arm.get_joint_angles()
            print(f"[cal] {name}: connected={arm.is_connected()} "
                  f"firmware={fw} joints={None if ja is None else [round(float(v), 4) for v in ja]}", flush=True)
            feedback_ok = feedback_ok and fw is not None and driver._valid_joints(ja)
        if not feedback_ok:
            print("[cal] FAIL: arm feedback unavailable (no motion command was sent)", flush=True)
            driver.disconnect()
            return 2
    elif args.command == "validate":
        physical = driver.read_physical_joints()
        if not driver._valid_joints(physical["left"]) or not driver._valid_joints(physical["right"]):
            print("[cal] FAIL: incomplete joint feedback", flush=True)
            driver.disconnect()
            return 2
        full = driver._physical_to_full_urdf(physical["left"], physical["right"])
        roundtrip = driver._full_to_physical(full)
        lower = np.asarray(cfg.robot.joint_lower_limits) + cfg.robot.joint_limit_margin
        upper = np.asarray(cfg.robot.joint_upper_limits) - cfg.robot.joint_limit_margin
        print(f"[cal] physical left : {np.round(physical['left'], 4).tolist()}")
        print(f"[cal] physical right: {np.round(physical['right'], 4).tolist()}")
        print(f"[cal] URDF left     : {np.round(full[:7], 4).tolist()}")
        print(f"[cal] URDF right    : {np.round(full[7:], 4).tolist()}")
        mapping_ok = np.allclose(roundtrip[0], physical["left"]) and np.allclose(
            roundtrip[1], physical["right"]
        )
        limits_ok = all(np.all((values >= lower) & (values <= upper)) for values in physical.values())
        print(f"[cal] mapping round-trip: {'OK' if mapping_ok else 'FAIL'}")
        print(f"[cal] SDK soft limits   : {'OK' if limits_ok else 'FAIL'}")
        if not mapping_ok or not limits_ok:
            driver.disconnect()
            return 2

    driver.disconnect()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
