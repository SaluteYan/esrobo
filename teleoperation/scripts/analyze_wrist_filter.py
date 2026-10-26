#!/usr/bin/env python3
"""Offline snapshot audit and explicitly synthetic wrist-filter comparison.

PYTHONPATH=teleoperation/src python ... --output report.json LOG...
Never imports drivers, sends commands, or treats 5 Hz snapshots as raw 50 Hz input.
"""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from esrobo_laptop.orientation import AbsoluteHandOrientation, ForearmFrame


def audit(path):
    rows = [json.loads(line) for line in path.open()]
    active = [r for r in rows if r.get('mode') == 'ACTIVE' and 'rates' in r]
    if not active:
        return dict(file=str(path), active_snapshots=0)
    steady = [r for r in active if r['monotonic_s'] > active[0]['monotonic_s']+3
              and not r.get('input_hold_active') and not r.get('position_limit_hold')
              and (r.get('targets') or {}).get('left')
              and (r.get('feedback') or {}).get('motion_diagnostics')]
    measured = [r['feedback']['motion_diagnostics'] for r in active
                if (r.get('feedback') or {}).get('motion_diagnostics')]
    wrists = [r['pico_wrist_input']['left'] for r in active
              if r.get('pico_wrist_input', {}).get('left')]
    return dict(file=str(path), active_snapshots=len(active), steady_snapshots=len(steady),
                median_snapshot_interval_s=float(np.median(np.diff([r['monotonic_s'] for r in active]))),
                absolute_error_p95_deg={key: np.percentile([
                    np.abs(np.degrees(r['feedback']['motion_diagnostics'][key]))
                    for r in steady], 95, axis=0).tolist() if steady else None
                    for key in ('target_command_error_rad', 'command_feedback_error_rad')},
                final_cumulative_limiter_ratio=measured[-1]['ratios'] if measured else {},
                fixed_step_limited_snapshots=sum(w.get('step_limited', False) for w in wrists),
                wrist_snapshots=len(wrists),
                input_hold_entries=sum(bool(r.get('input_hold_active')) and
                    not active[i-1].get('input_hold_active', False) for i,r in enumerate(active) if i))


def simulate(values, hz, *, cutoff=3., deadband=.25, legacy=False):
    points = dict(shoulder=np.zeros(3), elbow=np.array([0.,0.,-.3]),
                  wrist=np.array([.3,.1,-.4]))
    carrier = ForearmFrame().update(points)
    tracker = AbsoluteHandOrientation(points,carrier,palm_rotation=carrier)
    output, old = [], 0.
    for i, value in enumerate(values):
        if legacy:
            old += np.clip(value-old,-2.,2.)
            output.append(old)
        else:
            result=tracker.update(points,carrier,
                palm_rotation=carrier@Rotation.from_euler('x',value,degrees=True).as_matrix(),
                sample_time=i/hz,min_cutoff_hz=cutoff,deadband_deg=deadband)
            output.append(np.degrees(Rotation.from_matrix(result).as_rotvec()[0]))
    return np.asarray(output)


def synthetic():
    hz=50
    t=np.arange(0.,4.,1/hz)
    noise=.3*np.sin(2*np.pi*8*t)  # known, synthetic 8 Hz/0.3 degree noise
    step=np.where(t>=1.,30.,0.)
    ramp=20*t
    candidates=[('legacy_2deg',dict(legacy=True)),
                ('2Hz_0.25deg',dict(cutoff=2.)),
                ('3Hz_0.15deg',dict(deadband=.15)),
                ('3Hz_0.25deg',{}),
                ('3Hz_0.30deg',dict(deadband=.30)),
                ('5Hz_0.25deg',dict(cutoff=5.))]
    rows=[]
    for name,kwargs in candidates:
        n=simulate(noise,hz,**kwargs)
        s=simulate(step,hz,**kwargs)
        r=simulate(ramp,hz,**kwargs)
        reached=np.flatnonzero((t>=1)&(s>=27))
        rows.append(dict(name=name, noise_output_rms_deg=float(np.sqrt(np.mean(n[t>=1]**2))),
                         noise_total_variation_deg=float(np.abs(np.diff(n)).sum()),
                         step_90percent_time_s=float(t[reached[0]]-1) if len(reached) else None,
                         ramp_20deg_s_median_lag_deg=float(np.median((ramp-r)[t>=1])),
                         step_max_target_increment_deg=float(np.abs(np.diff(s)).max())))
    return dict(hz=hz, input_noise_amplitude_deg=.3, input_noise_hz=8, candidates=rows,
                caveat='Synthetic one-axis local orientation only; not measured human noise or robot motion.')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('logs',type=Path,nargs='+')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    result=dict(caveats=[
        'Snapshots are about 5 Hz, not full-rate raw shoulder/elbow/palm samples.',
        'target_command_error is effective trajectory goal minus command; not physical tracking error.',
        'Limiter ratios are cumulative and overlap; they do not prove measured oscillation.',
        'Synthetic response demonstrates tradeoffs, not empirical optimal parameters.'
    ],logs=[audit(path) for path in args.logs],synthetic=synthetic())
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(args.output)


if __name__=='__main__':
    main()
