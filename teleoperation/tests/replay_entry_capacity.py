"""Offline recorded entry replay with real meshes; no SDK or motion interface."""
import argparse
import json
from pathlib import Path
import threading
import time

import numpy as np

from esrobo_link.backends import ForwardModel
from esrobo_teleop.config import load_config
from esrobo_teleop.robot.hand_geometry import geometry_state
from esrobo_teleop.robot.handover_planner import HandoverPlanner
from esrobo_teleop.robot.linker_hand_driver import L10_PHYSICAL_JOINT_NAMES
from esrobo_teleop.robot.nero_driver import NeroSingleArmDriver
from esrobo_teleop.robot.torso_collision import TorsoCollisionGuard


def replay(fixture, hand_ages=(.005,), search_without_fingers=True):
    root = Path(__file__).resolve().parents[2]
    cfg = load_config(str(root/'teleoperation/config/teleop_config.yaml'))
    cfg.ik.urdf_path = str(root/'teleoperation/urdf/esrobo_waist_with_head.urdf')
    model, side = ForwardModel(cfg.ik), fixture['side']
    guard = TorsoCollisionGuard(model, side, cfg.robot.collision_package_dirs,
        cfg.robot.torso_collision_margin_m, cfg.robot.shoulder_collision_margin_m)
    hand_reads = [0]
    def hand_state(horizon):
        now = time.monotonic()
        age = hand_ages[hand_reads[0] % len(hand_ages)]
        hand_reads[0] += 1
        return geometry_state(fixture['hand_unit'], now-age, side,
            cfg.hand.geometry_feedback_calibration, L10_PHYSICAL_JOINT_NAMES,
            0., cfg.hand.geometry_feedback_timeout_s, now=now)
    guard.hand_state_provider = hand_state
    # __new__ deliberately avoids SDK construction and any physical interface.
    driver = NeroSingleArmDriver.__new__(NeroSingleArmDriver)
    driver._cfg, driver._side, driver._collision_guard = cfg.robot, side, guard
    driver._session_start = None
    driver._entry_arm_only_collision = True
    driver._return_cancel = threading.Event()
    direction, offset = driver._mapping(side)
    lower, upper = driver._effective_position_limits(side)
    sl = slice(0, 7) if side == 'left' else slice(7, 14)
    evaluate = model.fk(side)
    def fk(physical):
        full = np.zeros(14)
        full[sl] = (physical-offset)/direction
        return evaluate(full)
    start = np.array(fixture['start_urdf_rad'])*direction+offset
    goal = np.array(fixture['goal_urdf_rad'])*direction+offset
    goal[5:] = start[5:]
    guard.begin_entry_certificates(arm_geometry_only=True)
    try:
        check = (lambda a,b: driver._check_physical_path(a,b,entry_search_only=True)
                 ) if search_without_fingers else driver._check_physical_path
        plan = HandoverPlanner(lower, upper, check, fk, direction,
            timeout=20., max_nodes=cfg.robot.return_plan_max_nodes,
            seed=cfg.robot.return_plan_seed).plan(start, goal)
        report = dict(physical_motion=False, real_meshes=True,
            simulated_hand_ages_s=list(hand_ages), search_without_fingers=True,
            collision_profile='entry_arm',
            scope='replanned from recorded post-rejection feedback; original path not logged',
            waypoint_count=len(plan.path), planner_reason=plan.reason,
            held_wrist_urdf_rad=fixture['start_urdf_rad'][5:],
            hand_geometry_reads=hand_reads[0])
        if not plan.path:
            return report
        report['wrist_held_at_every_waypoint'] = all(
            np.array_equal(q[5:], start[5:]) for q in plan.path)
        assert report['wrist_held_at_every_waypoint']
        original = np.array(plan.path).copy()
        checks = []
        original_check = driver._check_physical_path
        def recorded_check(a, b, **kwargs):
            before = len(guard._entry_certificates)
            context = guard._entry_certificate_context
            accepted = original_check(a, b, **kwargs)
            checks.append(dict(accepted=accepted, before=before,
                after=len(guard._entry_certificates),
                context_changed=context != guard._entry_certificate_context,
                category=guard.last_diagnostics.get('category'),
                lower=np.minimum(a,b).tolist(), upper=np.maximum(a,b).tolist()))
            return accepted
        driver._check_physical_path = recorded_check
        reason = driver._prepare_entry_coverage(plan.path, lower, upper, guard)
        report.update(certification_reason=reason,
            certificate_count=len(guard._entry_certificates),
            path_unchanged=bool(np.array_equal(original, plan.path)),
            previous_capacity_check_would_reject=len(plan.path)>33)
        report['preparation_checks'] = checks
        if reason is None:
            def unexpected(*args, **kwargs):
                raise AssertionError('geometry computation after certificates frozen')
            guard._clearance = unexpected
            report['all_edges_covered'] = all(driver._check_physical_path(a, b,
                entry_coverage_only=True) for a, b in zip(plan.path, plan.path[1:]))
        report['last_geometry'] = {k:guard.last_diagnostics.get(k) for k in
            ('category', 'pair', 'clearance_m', 'required_m')}
        return report
    finally:
        guard.end_entry_certificates()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fixture', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--hand-ages-ms', type=float, nargs='+', default=[5.])
    parser.add_argument('--search-without-fingers', action='store_true')
    args = parser.parse_args()
    result = replay(json.loads(args.fixture.read_text()),
                    tuple(age/1000 for age in args.hand_ages_ms), args.search_without_fingers)
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k != 'preparation_checks'}, indent=2))
