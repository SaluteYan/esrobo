#!/usr/bin/env python3
"""Offline recorded mapping + seven-axis trajectory validation; never opens hardware."""
import argparse
import hashlib
import json
import time
from pathlib import Path

import numpy as np
from scipy.signal import butter, sosfiltfilt

import replay_jitter_trace as replay
from esrobo_laptop.wrist_joint_filter import WristJointFilter
from esrobo_teleop.config import retarget_filter_fingerprint


def metric(t, q, window):
    lo, hi = window
    mask = (t >= lo) & (t < hi)
    ts, qs = t[mask], np.degrees(q[mask])
    if len(ts) < 100:
        raise ValueError('insufficient execution samples')
    grid = np.arange(ts[0], ts[-1], .02)
    regular = np.array([np.interp(grid, ts, qs[:, j]) for j in range(7)]).T
    high = sosfiltfilt(butter(3, 2, fs=50, btype='highpass', output='sos'), regular, axis=0)
    return dict(span_deg=np.ptp(qs, axis=0).tolist(),
                travel_deg=np.abs(np.diff(qs, axis=0)).sum(axis=0).tolist(),
                high_frequency_rms_deg=np.sqrt(np.mean(high**2, axis=0)).tolist())


def trajectory(targets, *, recorded=False, tau=None):
    """Include actual hold/braking cycles, not just target-bearing commands."""
    from esrobo_teleop.robot.command_trajectory import CommandTrajectory, TrajectoryStep
    cfg = replay.cfg.robot
    c = CommandTrajectory(cfg.left_max_joint_velocity, cfg.left_max_joint_acceleration,
        cfg.left_max_joint_step, np.deg2rad(cfg.tracking_command_trajectory_lead_deg),
        max_dt=cfg.command_trajectory_max_dt_s, lead_timeout=cfg.command_trajectory_lead_timeout_s)
    first = replay.D[0]
    stamp = first['feedback_snapshot']['stamp_s']; dt = first['feedback_dt_s']
    prev = [r for r in replay.E if r['event']=='execution_cycle'
        and r['monotonic_ns'] < first['monotonic_ns']
        and r.get('feedback',{}).get('left',{}).get('motion')][-1]['feedback']['left']['motion']
    c.state = TrajectoryStep(np.array(prev['command_rad']), np.array(prev['velocity_rad_s']),
        stamp-dt, np.full(7,np.nan), dict(feedback_physical_rad=first['feedback_urdf_rad'],
        feedback_motion_stamp_s=stamp-dt, feedback_velocity_rad_s=[0.]*7))
    fk14 = replay.s0.make_command_fk('left')
    fk = lambda q: fk14(np.r_[q,np.zeros(7)])
    fb = np.array(first['feedback_urdf_rad']); commands=[]; times=[]; faults=[]
    for j,row in enumerate(replay.D):
        stamp = row['feedback_snapshot']['stamp_s']; dt = stamp-c.state.stamp
        if dt <= 0: continue
        if tau is None: fb = np.array(row['feedback_urdf_rad'])
        else: fb = fb+(1-np.exp(-dt/tau))*(c.state.position-fb) if tau else c.state.position.copy()
        seq = row['context']['command_seq']; held = row.get('position_limit_hold',False)
        target = np.array(row['ik_urdf_rad']) if recorded else (
            c.state.position if held else targets[replay.source_id[seq]])
        try:
            step = c.propose(target, fb, stamp, np.array(cfg.left_joint_soft_lower_limits_urdf),
                np.array(cfg.left_joint_soft_upper_limits_urdf), fk,
                replay.cfg.retarget.max_endpoint_translation_velocity_m_s,
                coordinate_elbow=True, brake_only=held)
        except Exception as exc:
            faults.append((j,str(exc))); break
        c.commit(step); commands.append(step.position)
        times.append(stamp-first['feedback_snapshot']['stamp_s']+
                     replay.T[replay.source_id[first['context']['command_seq']]])
    return dict(t=np.array(times), command=np.array(commands), fault=faults)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for key in ('motion', 'execution', 'reference', 'output'):
        p.add_argument('--'+key, type=Path, required=True)
    p.add_argument('--settings', type=Path, default=replay.ROOT/'config/laptop.yaml')
    args = p.parse_args()
    args.start, args.end = 0., float('inf')
    replay.load(args)
    # Hold packets carry a command sequence but no new mapping target. Keep
    # those execution cycles so a position-limit pause remains controlled braking.
    original = replay.D
    session = replay.S[0]['session_tag']
    replay.D = [r for r in replay.E if r.get('driver_event')=='command_sent'
        and (r.get('context') or {}).get('session_tag')==session
        and original[0]['monotonic_ns'] <= r['monotonic_ns'] <= original[-1]['monotonic_ns']]
    base = trajectory(replay.observed, recorded=True)
    if base['fault']:
        raise RuntimeError(('recorded baseline trajectory failed', base['fault']))
    actual = np.array([r['command_urdf_rad'] for r in replay.D])
    reproduction = float(np.degrees(abs(base['command']-actual)).max())
    print('Trajectory baseline max error deg:', reproduction, flush=True)
    if reproduction > .001:
        raise AssertionError('recorded trajectory not reproduced')
    # Replay original URDF IK and wrist mapping from the already recorded arm
    # translation limiter output; no new endpoint assumptions or hardware reads.
    reconstructed = replay.solve_series(
        [s['arm']['left']['limited_upper_direction'] for s in replay.stages],
        [s['arm']['left']['limited_forearm_direction'] for s in replay.stages],
        [s['wrist']['left']['absolute_local_orientation'] for s in replay.stages], limited=True)
    ik_error = float(np.degrees(abs(reconstructed-replay.observed)).max())
    print('Mapping baseline max error deg:', ik_error, flush=True)
    if ik_error > .001:
        raise AssertionError('recorded mapping not reproduced')
    cfg = replay.cfg
    filt = WristJointFilter(replay.measured[0][5:7], cfg.retarget,
        cfg.robot.left_joint_soft_lower_limits_urdf[5:7], cfg.robot.left_joint_soft_upper_limits_urdf[5:7])
    targets = replay.observed.copy()
    costs = []
    for i, stage in enumerate(replay.stages):
        start = time.perf_counter_ns()
        c = filt.propose(targets[i, 5:7], replay.S[i]['context']['source_times']['left_palm'])
        filt.commit(c)  # S contains only actual successful target sends.
        targets[i, 5:7] = c.output
        costs.append((time.perf_counter_ns()-start)/1e6)
    candidate = trajectory(targets)
    windows = [(234., 239.), (291., 296.)]
    report = dict(source_files={str(f):hashlib.sha256(f.read_bytes()).hexdigest()
                  for f in (args.motion, args.execution, args.reference)},
                  config=retarget_filter_fingerprint(cfg.retarget),
                  session_tag=replay.S[0]['session_tag'], target_count=len(targets),
                  execution_count=len(replay.D), mapping_reproduction_error_deg=ik_error,
                  trajectory_reproduction_error_deg=reproduction,
                  filter_cost_ms=dict(zip(('p50','p95','p99','max'),
                                         [*np.percentile(costs,[50,95,99]),max(costs)])),
                  faults=candidate['fault'], comparisons=[], sensitivity=[],
                  limitations=['Recorded-feedback counterfactual is not a prediction of new actuator dynamics.',
                    'Selected windows lack independent stationary ground truth; right arm has only synthetic validation.'])
    if not candidate['fault']:
        for w in windows:
            b = metric(base['t'], base['command'], w)
            c = metric(candidate['t'], candidate['command'], w)
            reduction = 1-np.array(c['travel_deg'])[5:7]/np.array(b['travel_deg'])[5:7]
            hf = np.array(c['high_frequency_rms_deg'])[5:7] / np.array(b['high_frequency_rms_deg'])[5:7]
            report['comparisons'].append(dict(window=w, baseline=b, candidate=c,
                travel_reduction=reduction.tolist(), high_frequency_ratio=hf.tolist(),
                passed=bool(np.all(reduction >= .4) and np.all(hf <= 1.))))
    for tau in (0., .03, .06, .1):
        s = trajectory(targets, tau=tau)
        report['sensitivity'].append(dict(tau_s=tau, faults=s['fault']))
    report['passed'] = bool(not report['faults'] and report['comparisons']
        and all(r['passed'] for r in report['comparisons'])
        and all(not r['faults'] for r in report['sensitivity']) and report['filter_cost_ms']['p99'] <= .5)
    args.output.mkdir(parents=True,exist_ok=True)
    (args.output/'report.json').write_text(json.dumps(report,indent=2))
    np.savez_compressed(args.output/'series.npz', target_t=replay.T, baseline_target=replay.observed,
        candidate_target=targets, baseline_t=base['t'], baseline_command=base['command'],
        candidate_t=candidate['t'], candidate_command=candidate['command'])
    print(json.dumps(report,indent=2))


if __name__ == '__main__':
    main()
