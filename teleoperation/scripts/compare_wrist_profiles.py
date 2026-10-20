"""Offline wrist trajectory comparison; no hardware, sockets or configuration writes.

Recorded targets drive a first-order actuator model, not recorded robot feedback.
J1-J4 are fixed and FK is intentionally inert to isolate joint limits; full FK
and collision/return behavior require the separate driver regression tests.
"""
import argparse
import json
from pathlib import Path

import numpy as np

from esrobo_teleop.config import load_config
from esrobo_teleop.robot.command_trajectory import CommandTrajectory, TrajectoryFault


def trajectory(cfg, side, baseline=False):
    velocity = np.array(getattr(cfg, side + '_max_joint_velocity') or cfg.max_joint_velocity, float)
    acceleration = np.array(getattr(cfg, side + '_max_joint_acceleration') or cfg.max_joint_acceleration, float)
    step = np.array(getattr(cfg, side + '_max_joint_step') or cfg.max_joint_step, float)
    lead = np.deg2rad(cfg.command_trajectory_lead_deg)
    if baseline:
        velocity[4:] = np.deg2rad(24.)
        lead[4:] = np.deg2rad(1.5)
    return CommandTrajectory(velocity, acceleration, step, lead,
                             max_dt=cfg.command_trajectory_max_dt_s,
                             lead_timeout=cfg.command_trajectory_lead_timeout_s)


def recorded_targets(path):
    rows = [json.loads(line) for line in Path(path).read_text().splitlines()]
    rows = [r for r in rows if r.get('mode') == 'ACTIVE'
            and (r.get('targets') or {}).get('left', {}).get('hold') is False
            and not r.get('input_hold_active')
            and not (r.get('wrist_handover') or {}).get('active', True)
            and (r.get('feedback') or {}).get('motion_diagnostics')]
    if len(rows) < 2:
        raise ValueError('At least two normal left-arm following snapshots required')
    times = np.array([r['monotonic_s'] for r in rows]); times -= times[0]
    goals = np.array([r['targets']['left']['arm_urdf_rad'] for r in rows])
    initial = np.array(rows[0]['feedback']['arm_urdf_rad'])
    return times, goals, initial


def simulate(cfg, side, baseline, tau, scenario, recording=None):
    tr = trajectory(cfg, side, baseline)
    q = np.zeros(7) if recording is None else recording[2].copy()
    tr.reset(q, 0.)
    duration = 8. if recording is None else float(recording[0][-1])
    now = 0.; samples = []; counts = {}; settled = None; fault = None
    stop_anchor = None; stop_travel = None
    for i in range(int(duration / .02)):
        dt = (.015, .020, .025)[i % 3]; now += dt
        if scenario != 'stalled' or now < .5:
            q += (tr.state.position - q) * (1 - np.exp(-dt / tau))
        target = q.copy()
        target[4:] = np.deg2rad(30. if side == 'left' else -30.)
        if scenario == 'stop':
            target[4:] *= 2  # Stop while at full speed, before goal braking starts.
        if scenario == 'reverse':
            target[4:] *= 1 if int(now / 1.2) % 2 == 0 else -1
        if recording is not None:
            target[4:] = [np.interp(now, recording[0], recording[1][:, j]) for j in range(4, 7)]
        brake = ((scenario == 'stop' and now >= 1.) or
                 (scenario == 'input_gap' and (1. <= now < 1.2 or 2. <= now < 2.5)))
        if brake and stop_anchor is None:
            stop_anchor = tr.state.position.copy()
        before = tr.state
        try:
            proposal = tr.propose(target, q, now, np.full(7, -3.), np.full(7, 3.), brake_only=brake)
        except TrajectoryFault as exc:
            assert tr.state is before  # A rejected proposal never advances output.
            fault = dict(time_s=now, reason=str(exc))
            break
        assert np.all(np.abs(proposal.velocity) <= tr.vmax + 1e-9)
        assert np.all(np.abs(proposal.velocity-before.velocity) <= tr.amax*dt + 1e-9)
        assert np.all(np.abs(proposal.position-q) <= tr.lead + 1e-9)
        assert np.allclose(proposal.position-before.position, proposal.velocity*dt, atol=1e-12)
        tr.commit(proposal)
        for key in ('configured_speed_bound_active', 'acceleration_bound_active',
                    'directional_lead_braking_active', 'feedback_motion_braking_active'):
            counts[key] = counts.get(key, np.zeros(7)) + np.array(proposal.diagnostics[key])
        samples.append(np.degrees(np.array([np.abs(target-proposal.position),
                      np.abs(proposal.position-q), np.abs(proposal.velocity)]))[:, 4:])
        if scenario == 'step' and settled is None and np.max(abs(target[4:]-q[4:])) <= np.deg2rad(1.5):
            settled = now
        if brake and stop_travel is None and np.max(abs(proposal.velocity)) < 1e-8:
            stop_travel = np.degrees(abs(proposal.position-stop_anchor))[4:].tolist()
    samples = np.asarray(samples)
    return dict(side=side, profile='baseline_24_1p5' if baseline else 'candidate_36_10',
                response_s=tau, scenario=scenario, samples=len(samples), fault=fault,
                settling_s=settled, command_stop_travel_deg=stop_travel,
                target_command_error_p95_deg=np.percentile(samples[:, 0], 95, axis=0).tolist(),
                command_feedback_error_p95_deg=np.percentile(samples[:, 1], 95, axis=0).tolist(),
                max_command_feedback_error_deg=samples[:, 1].max(axis=0).tolist(),
                command_speed_p95_deg_s=np.percentile(samples[:, 2], 95, axis=0).tolist(),
                limiter_ratios={k:(v[4:]/len(samples)).tolist() for k,v in counts.items()})


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('journal')
    parser.add_argument('--config', default='teleoperation/config/teleop_config.yaml')
    args = parser.parse_args()
    cfg = load_config(args.config).robot
    recording = recorded_targets(args.journal)
    results = [simulate(cfg, side, old, tau, case)
               for side in ('left', 'right') for old in (True, False)
               for tau in (.03, .05, .08)
               for case in ('step', 'reverse', 'input_gap', 'stop', 'stalled')]
    results += [simulate(cfg, 'left', old, tau, 'recorded_targets', recording)
                for old in (True, False) for tau in (.03, .05, .08)]
    print(json.dumps(dict(note=__doc__, journal=args.journal, results=results), indent=2))
