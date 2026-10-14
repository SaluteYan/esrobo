"""Summarize recorded J5-J7 limits. Reads JSONL only; no robot connection."""
import argparse
import json
from pathlib import Path

import numpy as np


def summarize(path):
    sides = {}
    for line in Path(path).read_text().splitlines():
        row = json.loads(line)
        feedback = row.get('feedback') or {}
        members = feedback.get('sides') or {row.get('side', 'selected'): feedback}
        for side, member in members.items():
            diag = member.get('motion_diagnostics')
            if not diag:
                continue
            value = sides.setdefault(side, dict(samples=0, latest=None, errors=[], speed=[]))
            value['samples'] += 1
            value['latest'] = diag
            value['errors'].append(np.degrees(diag['target_command_error_rad'])[4:])
            value['speed'].append(np.degrees(diag['feedback_velocity_rad_s'])[4:])
    result = {}
    for side, value in sides.items():
        latest = value['latest']
        result[side] = dict(
            recorded_snapshots=value['samples'], trajectory_samples=latest['samples'],
            limiter_ratio_j5_j7={k:v[4:] for k,v in latest['ratios'].items()},
            endpoint_ratio=latest['endpoint_ratio'],
            target_command_error_abs_p95_deg=np.percentile(np.abs(value['errors']),95,axis=0).tolist(),
            measured_speed_abs_p95_deg_s=np.percentile(np.abs(value['speed']),95,axis=0).tolist())
    return dict(sides=result, note='Snapshot percentiles are descriptive, not per-cycle maxima. '
                'Limiter ratios are cumulative since trajectory initialization; simultaneous limits overlap. '
                'No recorded diagnostics means no supported speed recommendation.')


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('journal')
    args=parser.parse_args()
    print(json.dumps(summarize(args.journal),ensure_ascii=False,indent=2))
