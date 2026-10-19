"""Offline torque evidence summary; no interpolation, hardware access or tuning.
Usage: python laptop_teleop/tests/tools/summarize_torque.py torque_<session>.jsonl
"""
import argparse
from collections import defaultdict
import json
import math
from pathlib import Path
import numpy as np


def summarize(rows):
    groups = defaultdict(list)
    for row in rows:
        if row.get('record_type') == 'sample':
            groups[(row['session'], row['side'], row['phase'], row['baseline_id'])].append(row)
    result = []
    for key, records in groups.items():
        axes = []
        for j in range(7):
            accepted, excluded = [], 0
            previous_sequence = previous_stamp = run_start = None
            run_count = max_count = 0
            max_duration = 0.
            exceeded_count = 0
            for row in records:
                stamps, ages = row.get('axis_stamp_s'), row.get('axis_age_s')
                deviation, limits = row.get('deviation_nm'), row.get('limits_nm')
                stamp = stamps[j] if stamps else None
                age = ages[j] if ages else None
                usable = (row.get('valid') and deviation and limits and stamp is not None
                          and age is not None and 0 <= age <= .3 and deviation[j] is not None
                          and limits[j] is not None and limits[j] > 0
                          and math.isfinite(stamp) and math.isfinite(deviation[j]))
                if not usable or previous_stamp is not None and stamp <= previous_stamp:
                    excluded += 1
                    run_start = None; run_count = 0
                    previous_sequence = None
                    continue
                contiguous = (previous_sequence is not None and row['sequence'] == previous_sequence+1
                              and 0 < stamp-previous_stamp <= .3)
                if not contiguous:
                    run_start = None; run_count = 0
                if deviation[j] > limits[j]:
                    exceeded_count += 1
                    if run_start is None:run_start = stamp
                    run_count += 1
                    max_count = max(max_count, run_count)
                    max_duration = max(max_duration, stamp-run_start)
                else:
                    run_start = None; run_count = 0
                accepted.append(row)
                previous_sequence, previous_stamp = row['sequence'], stamp
            if not accepted:
                axes.append(dict(joint=j+1, valid_samples=0, excluded_samples=excluded))
                continue
            values = [r['deviation_nm'][j] for r in accepted]
            ratios = [r['deviation_nm'][j]/r['limits_nm'][j] for r in accepted]
            peak = accepted[int(np.argmax(values))]
            axes.append(dict(joint=j+1,valid_samples=len(values),excluded_samples=excluded,
                peak_deviation_nm=max(values),p95_deviation_nm=float(np.percentile(values,95)),
                p99_deviation_nm=float(np.percentile(values,99)),peak_limit_ratio=max(ratios),
                p95_limit_ratio=float(np.percentile(ratios,95)),exceeded_fraction=exceeded_count/len(values),
                max_observed_consecutive_exceeded=max_count,max_observed_exceeded_duration_s=max_duration,
                max_protection_trip_count=max(r['trip_counts'][j] for r in accepted),
                peak=dict(sequence=peak['sequence'],unix_ns=peak['unix_ns'],torque_nm=peak['torque_nm'][j],
                          baseline_nm=peak['baseline_nm'][j],limit_nm=peak['limits_nm'][j],motion=peak['motion'])))
        result.append(dict(session=key[0],side=key[1],phase=key[2],baseline_id=key[3],joints=axes))
    return dict(note='Observed samples only; missing/repeated/stale data excluded; no interpolation or automatic limit recommendation. Durations run from first to last observed exceeded sample.',groups=result)


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log',type=Path)
    args=parser.parse_args()
    with args.log.open() as stream:
        print(json.dumps(summarize(json.loads(line) for line in stream if line.strip()),ensure_ascii=False,indent=2,allow_nan=False))
