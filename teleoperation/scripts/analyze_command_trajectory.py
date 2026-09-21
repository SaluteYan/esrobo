#!/usr/bin/env python3
"""Read-only log statistics and deterministic synthetic tracking comparison.

No SDK instance or CAN socket is created. The simulation is not a robot model.
"""
import argparse
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from esrobo_teleop.config import build_config
from esrobo_teleop.robot.command_trajectory import CommandTrajectory
from esrobo_teleop.robot.nero_driver import NeroDualArmDriver


def simulate(mode, profile="step"):
    cfg = build_config().robot
    legacy = NeroDualArmDriver.__new__(NeroDualArmDriver)
    legacy._cfg = cfg
    legacy._session_start = None
    legacy._command_velocity = {"right": np.zeros(7)}
    directions, offsets = legacy._mapping("right")
    trajectory = CommandTrajectory(cfg.right_max_joint_velocity, cfg.right_max_joint_acceleration,
                                   cfg.right_max_joint_step, np.deg2rad(cfg.command_trajectory_lead_deg))
    trajectory.reset(offsets.copy(), 0.)
    lower, upper = legacy._effective_position_limits("right")
    def fk(physical):
        q = (physical-offsets) / directions
        return np.array([[q[0]*.3, q[1]*.3, q[2]*.3], [q[3]*.3, q[4]*.3, q[5]*.3]])
    measured = offsets.copy()
    t = 0.
    history = []
    for i in range(450):
        dt = [.02, .015, .025][i % 3]
        t += dt
        angle = (.5 if profile == "step" else min(.5, .18*t)
                 if profile == "ramp" else .5 if t < 3 else .1)
        target = offsets.copy()
        target[3] += angle * directions[3]
        if mode == "trajectory":
            step = trajectory.propose(target, measured, t, lower, upper, fk, .22)
            command = step.position
            trajectory.commit(step)
        else:
            command = legacy._clamp(target, measured, "right", dt)
        lead = float(np.max(np.abs(command-measured)))
        # Assumed first-order response, tau=120 ms, with no hardware claims.
        measured += dt/(.12+dt) * (command-measured)
        error = abs(target[3]-measured[3])
        history.append([t, error, lead])
    values = np.asarray(history)
    after = 3 if profile == "reverse" else .0
    errors = values[values[:, 0] >= after]
    outside = errors[:, 1] > np.deg2rad(1)
    last_outside = errors[outside, 0]
    return {
        "profile": profile, "mode": mode,
        "assumption": "synthetic 7-axis linear FK, first-order feedback tau=0.12s; NOT hardware replay",
        "mean_error_deg": float(np.degrees(np.mean(values[:, 1]))),
        "p90_error_deg": float(np.degrees(np.percentile(values[:, 1], 90))),
        "settling_s_after_last_target_change": None if profile == "ramp" else float((last_outside[-1] if len(last_outside) else after)-after),
        "max_lead_deg": float(np.degrees(np.max(values[:, 2]))),
    }


def analyze(path):
    rows = [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
    rows = [r for r in rows if r.get("armed") and isinstance(r.get("elbow"), dict)
            and r["elbow"].get("last_sent_j4_deg") is not None]
    if not rows:
        return {"path": str(path), "samples": 0}
    keys = ("raw_mapped_deg", "retarget_deg", "ik_j4_deg", "last_sent_j4_deg", "feedback_j4_deg")
    columns = {key: np.array([r["elbow"][key] for r in rows]) for key in keys}
    timing_keys = sorted({key for row in rows for key in row.get("timings", {})})
    def timing_summary(selected):
        result = {}
        for key in timing_keys:
            values = [r["timings"][key] for r in selected
                      if r.get("timings", {}).get(key) is not None]
            if values:
                result[key] = np.percentile(values, [50, 95, 100]).tolist()
        return result
    joint_errors = {}
    for a, b in (("ik", "command"), ("command", "feedback")):
        pairs = [r["joints_deg"] for r in rows if
                 r.get("joints_deg", {}).get(a) is not None and
                 r.get("joints_deg", {}).get(b) is not None]
        if pairs:
            joint_errors[f"{a}->{b}"] = np.percentile(
                np.abs(np.array([p[a] for p in pairs])-np.array([p[b] for p in pairs])),
                [50, 90], axis=0).tolist()
    return {
        "path": str(path), "samples": len(rows),
        "duration_s": rows[-1]["timestamp"] - rows[0]["timestamp"],
        "stage_abs_error_median_p90_deg": {
            f"{a}->{b}": np.percentile(np.abs(columns[a]-columns[b]), [50, 90]).tolist()
            for a, b in zip(keys[:-1], keys[1:])},
        "timing_median_p95_max_ms": timing_summary(rows),
        "first_3s_timing_median_p95_max_ms": timing_summary(
            [r for r in rows if r["timestamp"]-rows[0]["timestamp"] < 3]),
        "joint_abs_error_median_p90_deg": joint_errors,
        "limitation": "Sampled logs, not full hardware replay. Angle errors are not time delay; local packet age is not synchronized PICO-to-motor latency.",
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log", type=Path)
    args = parser.parse_args()
    if args.log:
        print(json.dumps(analyze(args.log), indent=2))
    for profile in ("step", "ramp", "reverse"):
        for mode in ("legacy", "trajectory"):
            print(json.dumps(simulate(mode, profile)))
