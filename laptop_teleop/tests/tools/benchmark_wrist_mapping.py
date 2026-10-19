"""Replay captured wrist requests offline; never imports a physical backend."""
import argparse
import importlib.util
import json
from pathlib import Path
import time

import numpy as np
from esrobo_laptop.mapping import WristMapping


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cases', required=True)
    parser.add_argument('--baseline', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    spec = importlib.util.spec_from_file_location('esrobo_laptop.mapping_baseline', args.baseline)
    old = importlib.util.module_from_spec(spec); spec.loader.exec_module(old)
    cases = [r for line in Path(args.cases).read_text().splitlines()
             if (r := json.loads(line)).get('stage') == 'wrist_case']
    if not cases:
        raise RuntimeError('No actual wrist IK calls captured')
    durations = {'before': [], 'after': []}
    changes, error_changes = [], []
    # Warm imports and both numerical paths before timed comparisons.
    for repetition in range(-1, 3):
        for i, case in enumerate(cases[:1] if repetition < 0 else cases):
            result, errors = {}, {}
            # Alternate order to reduce thermal/scheduling bias.
            order = [('before', old.WristMapping), ('after', WristMapping)]
            if (i+repetition) % 2: order.reverse()
            for name, cls in order:
                wrist = cls.__new__(cls)
                wrist.axes = np.asarray(case['axes'])
                wrist.neutral_urdf = np.asarray(case['neutral'])
                wrist.terminal_lower = np.asarray(case['lower'])
                wrist.terminal_upper = np.asarray(case['upper'])
                wrist._last_terminal_target = (None if case['previous'] is None
                                                else np.asarray(case['previous']))
                started = time.perf_counter()
                result[name] = wrist.terminal(np.asarray(case['desired']))
                elapsed=1000*(time.perf_counter()-started)
                if repetition >= 0: durations[name].append(elapsed)
                errors[name] = wrist.last_diagnostics['mapped_orientation_error_deg']
                assert np.all(result[name] >= wrist.terminal_lower)
                assert np.all(result[name] <= wrist.terminal_upper)
            if repetition >= 0:
                changes.append(float(np.max(np.abs(np.degrees(result['after']-result['before'])))))
                error_changes.append(errors['after']-errors['before'])
    report = dict(test_only=True, cases=len(cases), repetitions=3,
                  max_joint_change_deg=max(changes), max_error_increase_deg=max(error_changes),
                  durations_ms={k:dict(median=float(np.median(v)), p95=float(np.percentile(v,95)),
                                      maximum=max(v)) for k,v in durations.items()})
    Path(args.output).write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__': main()
