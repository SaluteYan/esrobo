"""Offline real-mesh comparison; never imported by the teleoperation entrypoint.

Run with PYTHONPATH=teleoperation:teleoperation/src:robot_link:teleoperation/tests.
This measures coverage queries, not robot motion or a complete approach route.
"""
import argparse
import json
from pathlib import Path
import time

import numpy as np

from test_thumb_collision_envelope import RecordedThumbTests


def compare():
    RecordedThumbTests.setUpClass()
    case = RecordedThumbTests()
    guard, pose = case.guard, case.recorded_arm()
    guard.hand_state_provider = lambda _: case.recorded_state(.002)
    lower, upper = pose.copy(), pose.copy()
    lower[7:] -= np.deg2rad(.2)
    upper[7:] += np.deg2rad(.2)
    original = guard._clearance
    counts = [0]
    def measured(*args, **kwargs):
        counts[0] += 1
        return original(*args, **kwargs)
    guard._clearance = measured
    results = {}
    for name, frozen in [('previous_cache_with_fallback', False), ('frozen_coverage_only', True)]:
        guard.begin_entry_certificates()
        try:
            assert guard(lower, upper), guard.last_diagnostics
            if frozen:
                assert guard.freeze_entry_certificates()
            counts[0] = 0
            durations = []
            for i in range(200):
                q = pose.copy(); q[11] += np.deg2rad(.05)*np.sin(i)
                start = time.perf_counter()
                accepted = guard(q, q)
                durations.append((time.perf_counter()-start)*1000)
                assert accepted, guard.last_diagnostics
            covered_checks = counts[0]
            outside = pose.copy(); outside[11] = upper[11]+1e-6
            start = time.perf_counter()
            accepted = guard(outside, outside)
            missed_ms = (time.perf_counter()-start)*1000
            results[name] = dict(covered_requests=len(durations),
                median_ms=float(np.median(durations)), p95_ms=float(np.percentile(durations,95)),
                max_ms=float(max(durations)), covered_geometry_checks=covered_checks,
                outside_request=dict(accepted=bool(accepted), elapsed_ms=missed_ms,
                    geometry_checks=counts[0]-covered_checks,
                    category=guard.last_diagnostics['category']))
            if frozen:
                assert not accepted and counts[0] == 0
        finally:
            guard.end_entry_certificates()
    return dict(test_only=True, physical_motion=False, geometry='real meshes',
                scope='identical recorded pose; 200 covered queries and one outside query; not motion validation',
                results=results)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    report = compare()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report['results'], indent=2))
