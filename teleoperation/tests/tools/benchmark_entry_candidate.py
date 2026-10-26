"""Offline recorded entry candidates with real FK/geometry; never creates a driver/SDK.

Timing is confined to this test tool. The locked final target substitutes for
unlogged intermediate lookahead vertices; results are a bounded replay, not a
claim to reconstruct the exact live candidate or OS scheduling.
"""
import argparse
import importlib.util
import json
from pathlib import Path
import time
import numpy as np
from esrobo_link.backends import ForwardModel
from esrobo_teleop.config import load_config
from esrobo_teleop.robot.command_trajectory import CommandTrajectory, TrajectoryStep
from esrobo_teleop.robot.nero_driver import NeroSingleArmDriver
from esrobo_teleop.robot.torso_collision import TorsoCollisionGuard
from esrobo_teleop.robot.hand_geometry import geometry_state
from esrobo_teleop.robot.linker_hand_driver import L10_PHYSICAL_JOINT_NAMES


def run(args):
    cfg = load_config(str(args.root / 'teleoperation/config/teleop_config.yaml'))
    cfg.ik.urdf_path = str(args.root / 'teleoperation/urdf/esrobo_waist_with_head.urdf')
    model = ForwardModel(cfg.ik)
    guard_type = TorsoCollisionGuard
    if args.geometry:
        spec = importlib.util.spec_from_file_location('entry_geometry_before', args.geometry)
        module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        # An archived candidate may live in /tmp. Resolve its model resources
        # against the selected checkout, without replacing installed code.
        module.__file__ = str(args.root / 'teleoperation/src/esrobo_teleop/robot/torso_collision.py')
        guard_type = module.TorsoCollisionGuard
    guard = guard_type(model, 'right', cfg.robot.collision_package_dirs,
                       cfg.robot.torso_collision_margin_m, cfg.robot.shoulder_collision_margin_m)
    fixture = json.loads(args.fixture.read_text())
    raw = fixture['hand_unit']
    age_s = .01
    def hand():
        now = time.monotonic()
        return geometry_state(raw, now-age_s, 'right', cfg.hand.geometry_feedback_calibration,
                              L10_PHYSICAL_JOINT_NAMES, 0., cfg.hand.geometry_feedback_timeout_s,
                              now=now)
    guard.hand_state_provider = lambda horizon: hand()
    # Only mapping/configuration helpers: no constructor or physical interface.
    driver = NeroSingleArmDriver.__new__(NeroSingleArmDriver)
    driver._cfg, driver._side = cfg.robot, 'right'
    driver._session_start = None
    direction, offset = driver._mapping('right')
    lower, upper = driver._effective_position_limits('right')
    def full(q):
        result = np.zeros(14);result[7:] = (q-offset)/direction;return result
    goal = direction*np.asarray(fixture['target_urdf_rad']) + offset
    rows = []
    for recorded in fixture['candidates']:
        seed = recorded['previous_sent'];sample = recorded['sample']
        repeats = []
        for repeat in range(args.repeats):
            age_s = args.feedback_ages_ms[repeat % len(args.feedback_ages_ms)] / 1000
            trajectory = CommandTrajectory(driver._joint_rate_limit_vector('right','max_joint_velocity'),
                driver._joint_rate_limit_vector('right','max_joint_acceleration'),
                driver._joint_rate_limit_vector('right','max_joint_step'),np.deg2rad(cfg.robot.command_trajectory_lead_deg),
                max_dt=cfg.robot.return_command_trajectory_max_dt_s)
            trajectory.state = TrajectoryStep(np.array(seed['command_physical_rad']),
                np.array(seed['command_velocity_rad_s']),seed['feedback_stamp_s'],np.full(7,np.nan),
                {'feedback_physical_rad':seed['feedback_physical_rad'],
                 'feedback_motion_stamp_s':seed['feedback_stamp_s'],'feedback_velocity_rad_s':[0.]*7})
            current=np.array(sample['feedback_physical_rad']);stamp=sample['feedback_stamp_s']
            if not args.reuse_entry or repeat == 0:
                guard.begin_entry_certificates()
                # Previous certified motion normally covers the outstanding command.
                assert guard(full(current),full(trajectory.state.position)),guard.last_diagnostics
            calls=[];finger_calls=[]
            build_fingers=guard._build_fingers
            def measured_build(intervals):
                started=time.perf_counter()
                try:return build_fingers(intervals)
                finally:finger_calls.append((time.perf_counter()-started)*1000)
            guard._build_fingers=measured_build
            def check(a,b):
                t=time.perf_counter();ok=guard(full(a),full(b));elapsed=(time.perf_counter()-t)*1000
                calls.append(dict(ms=elapsed,ok=ok,boxes=guard.last_diagnostics.get('checked_boxes'),
                                  cached=guard.last_diagnostics.get('entry_cache_hit'),
                                  category=guard.last_diagnostics.get('category'),
                                  lower_urdf_rad=np.minimum(full(a),full(b)).tolist(),
                                  upper_urdf_rad=np.maximum(full(a),full(b)).tolist()))
                return ok
            t=time.perf_counter();error=None
            try:
                candidate=trajectory.propose(goal,current,stamp,lower,upper,path_check=check)
                accepted=check(current,candidate.position)
            except ValueError as exc:
                error=str(exc);accepted=False
            elapsed=(time.perf_counter()-t)*1000
            repeats.append(dict(total_ms=elapsed,geometry_ms=sum(c['ms'] for c in calls),
                finger_build_ms=finger_calls,
                feedback_age_ms=age_s*1000,checks=calls,accepted=accepted,error=error))
            guard._build_fingers=build_fingers
            if not args.reuse_entry:
                guard.end_entry_certificates()
        guard.end_entry_certificates()
        rows.append(dict(seq=sample['diagnostic_seq'],feedback_dt_ms=(stamp-seed['feedback_stamp_s'])*1000,
            median_ms=float(np.median([r['total_ms'] for r in repeats])),
            p95_ms=float(np.percentile([r['total_ms'] for r in repeats],95)),repeats=repeats))
    return dict(test_only=True,physical_motion=False,limitation=__doc__,
                reuse_entry=args.reuse_entry, feedback_ages_ms=args.feedback_ages_ms,cases=rows)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root',type=Path,default=Path.cwd())
    p.add_argument('--fixture',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--geometry',type=Path);p.add_argument('--repeats',type=int,default=5)
    p.add_argument('--reuse-entry',action='store_true',help='Keep geometry cache between repeated candidate computations; not a full trajectory replay')
    p.add_argument('--feedback-ages-ms',type=float,nargs='+',default=[10.])
    args=p.parse_args();report=run(args);args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps([{k:v for k,v in row.items() if k!='repeats'} for row in report['cases']],indent=2))
