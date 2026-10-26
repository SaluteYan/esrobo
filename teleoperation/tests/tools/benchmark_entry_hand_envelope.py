"""Test-only real-mesh certificate comparison; no SDK, CAN or motion interface.

Compare live full-hand checks against arm-only entry on identical recorded poses.
There is no fixed entry hand envelope. This does not execute robot commands.
"""
import argparse
import json
from pathlib import Path
import time

import numpy as np

from test_thumb_collision_envelope import RecordedThumbTests
from esrobo_teleop.robot.hand_geometry import geometry_state
from esrobo_teleop.robot.linker_hand_driver import L10_PHYSICAL_JOINT_NAMES


def compare():
    RecordedThumbTests.setUpClass()
    case = RecordedThumbTests()
    guard, pose = case.guard, case.recorded_arm()
    raw = [143, 127, 254, 254, 254, 254, 0, 0, 0, 112]
    ages = [.002, .005, .010, .015, .020, .030, .080, .099] * 5
    age = [.002]
    guard.hand_state_provider = lambda horizon: case.recorded_state(age[0])

    results = {}
    for name, check in [('live_hand_geometry', guard), ('entry_arm_only', guard.check_arm_only)]:
        age[0] = .002
        guard.begin_entry_certificates(arm_geometry_only=name == 'entry_arm_only')
        try:
            lo, hi = pose.copy(), pose.copy()
            lo[7:] -= np.deg2rad(.2)
            hi[7:] += np.deg2rad(.2)
            assert check(lo, hi), guard.last_diagnostics
            samples, hits = [], 0
            for value in ages:
                age[0] = value
                measured = pose.copy()
                measured[11] += np.deg2rad(.004)
                start = time.perf_counter()
                accepted = check(measured, measured)
                samples.append((time.perf_counter()-start)*1000)
                assert accepted, guard.last_diagnostics
                hits += int(guard.last_diagnostics['entry_cache_hit'])
            results[name] = dict(requests=len(samples), cache_hits=hits,
                median_ms=float(np.median(samples)), p95_ms=float(np.percentile(samples, 95)),
                maximum_ms=float(max(samples)))
        finally:
            guard.end_entry_certificates()
    return dict(test_only=True, physical_motion=False, geometry='real meshes',
                scope='recorded near-zero pose; not a full approach execution',
                feedback_ages_s=ages, results=results)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    report = compare()
    Path(args.output).write_text(json.dumps(report, indent=2))
    print(json.dumps(report['results'], indent=2))
