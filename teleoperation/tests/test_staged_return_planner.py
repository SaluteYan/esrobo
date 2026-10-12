"""Offline tests for the certified shoulder-first recovery route."""

from pathlib import Path

import numpy as np

from esrobo_teleop.config import load_config
from esrobo_teleop.ik.solver import IkSolver
from esrobo_teleop.robot.staged_return_planner import StagedReturnPlanner
from esrobo_teleop.robot.torso_collision import TorsoCollisionGuard


def test_staged_return_keeps_elbow_outside_until_shoulders_are_clear():
    def fk(q):
        return np.array([[0, .2-q[1], .5], [0, .2-q[1], .2]])

    def check(a, b):
        lo, hi = np.minimum(a, b), np.maximum(a, b)
        # The elbow may straighten only after J2 has moved outward; shoulder
        # may return only after J3 and J4 have reached the target.
        if lo[1] > -.1 and (hi[2] > .1 or hi[3] > .1) and (
                lo[2] < .1 or lo[3] < .1):
            return False
        return True

    start = np.array([0., 0., .5, .5, 0., 0., 0.])
    goal = np.zeros(7)
    planner = StagedReturnPlanner(-np.ones(7), np.ones(7), check, fk,
                                  np.ones(7), timeout=2.)
    result = planner.plan(start, goal)
    assert result.path, result.reason
    assert all(check(a, b) for a, b in zip(result.path, result.path[1:]))
    np.testing.assert_allclose(result.path[0], start)
    np.testing.assert_allclose(result.path[-1], goal)


def test_rejects_unsafe_goal_and_cancel_without_a_path():
    fk = lambda q: np.array([[0, q[1], 0], [0, q[1], 0]])
    start, goal = np.zeros(7), np.ones(7)*.2
    unsafe = StagedReturnPlanner(-np.ones(7), np.ones(7),
                                lambda a, b: not np.allclose(b, goal), fk,
                                np.ones(7))
    assert not unsafe.plan(start, goal).path
    cancelled = StagedReturnPlanner(-np.ones(7), np.ones(7),
                                    lambda a, b: True, fk, np.ones(7),
                                    cancelled=lambda: True)
    assert not cancelled.plan(start, goal).path


def test_reported_right_arm_pose_has_certified_staged_return():
    cfg = load_config(str(Path(__file__).resolve().parents[1]
                          / "config/teleop_config.yaml"))
    cfg.ik.urdf_path = str(Path(__file__).resolve().parents[1]
                            / "urdf/esrobo_waist_with_head.urdf")
    model = IkSolver(cfg.ik)
    rc = cfg.robot
    guard = TorsoCollisionGuard(model, "right", rc.collision_package_dirs,
                                rc.torso_collision_margin_m,
                                rc.shoulder_collision_margin_m,
                                include_fingers=False)
    direction = np.asarray(rc.right_joint_directions)
    zero = np.asarray(rc.right_joint_offsets)
    lower = np.maximum(np.asarray(rc.joint_lower_limits)+rc.joint_limit_margin,
                       direction*np.asarray(rc.right_joint_soft_lower_limits_urdf)+zero)
    upper = np.minimum(np.asarray(rc.joint_upper_limits)-rc.joint_limit_margin,
                       direction*np.asarray(rc.right_joint_soft_upper_limits_urdf)+zero)
    def full(q):
        joints = np.zeros(14)
        joints[7:] = (q-zero)/direction
        return joints
    def check(a, b):
        return guard(full(a), full(b))
    fk = model.make_command_fk("right")
    planner = StagedReturnPlanner(lower, upper, check, lambda q: fk(full(q)),
                                  direction, timeout=5.,
                                  diagnostics=lambda: guard.last_diagnostics)
    start = np.deg2rad([-15.137, 79.795, 96.122, 91.624,
                        .003, .005, -.15])
    result = planner.plan(start, zero)
    assert result.path, result.reason
    assert len(planner.certificates) == len(result.path)-1
    assert all(c["certified"] for c in planner.certificates)
    first_elbow = next(i for i, q in enumerate(result.path)
                       if abs(q[2]-start[2]) > 1e-6 or abs(q[3]-start[3]) > 1e-6)
    assert first_elbow > 1
    assert all(np.allclose(q[2:], start[2:]) for q in result.path[:first_elbow])
    assert abs(result.path[first_elbow][3]-start[3]) > 1e-6
    assert abs(result.path[first_elbow][2]-start[2]) < 1e-6
    assert all(check(a, b) for a, b in zip(result.path, result.path[1:]))
    np.testing.assert_allclose(result.path[0], start)
    np.testing.assert_allclose(result.path[-1], zero)


def test_20260930_right_idle_pose_has_certified_arm_only_return():
    cfg = load_config(str(Path(__file__).resolve().parents[1]
                          / "config/teleop_config.yaml"))
    cfg.ik.urdf_path = str(Path(__file__).resolve().parents[1]
                            / "urdf/esrobo_waist_with_head.urdf")
    model = IkSolver(cfg.ik)
    rc = cfg.robot
    guard = TorsoCollisionGuard(model, "right", rc.collision_package_dirs,
                                rc.torso_collision_margin_m,
                                rc.shoulder_collision_margin_m,
                                include_fingers=False)
    direction = np.asarray(rc.right_joint_directions)
    zero = np.asarray(rc.right_joint_offsets)
    lower = np.maximum(np.asarray(rc.joint_lower_limits) + rc.joint_limit_margin,
                       direction * np.asarray(rc.right_joint_soft_lower_limits_urdf) + zero)
    upper = np.minimum(np.asarray(rc.joint_upper_limits) - rc.joint_limit_margin,
                       direction * np.asarray(rc.right_joint_soft_upper_limits_urdf) + zero)
    measured_urdf = np.asarray([
        0.01663298777150596, -0.011972958668680977,
        0.4935965657565164, -0.022794000031045945,
        -0.2183406894244906, -0.05373868766890541,
        -0.03529055747532535,
    ])
    start = direction * measured_urdf + zero

    def full(physical):
        joints = np.zeros(14)
        joints[7:] = (physical - zero) / direction
        return joints

    fk = model.make_command_fk("right")
    planner = StagedReturnPlanner(
        lower, upper, lambda a, b: guard.check_arm_only(full(a), full(b)),
        lambda physical: fk(full(physical)), direction,
        timeout=rc.return_plan_timeout_s, max_nodes=rc.return_plan_max_nodes,
        seed=rc.return_plan_seed, diagnostics=lambda: guard.last_diagnostics,
    )
    result = planner.plan(start, zero)
    assert result.path, (result.reason, guard.last_diagnostics)
    assert len(planner.certificates) == len(result.path) - 1
    assert all(certificate["certified"] for certificate in planner.certificates)
    np.testing.assert_allclose(result.path[0], start)
    np.testing.assert_allclose(result.path[-1], zero)
