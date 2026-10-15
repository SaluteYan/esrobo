"""Offline tests for the certified shoulder-first recovery route."""

from pathlib import Path

import numpy as np
import pytest

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


@pytest.mark.parametrize('pose', [
    [.041887902047863905, -.195110357080446, -.47713811091020986,
     .3974987371417086, 1.0254332954242285, -.3155206221755349, -.028012534494508988],
    [-.006457718232379019, -.18664551020827358, -.5424832381048775,
     .3863460832214648, 1.5376699775920444, -.2645046481397407, -.131737451940532],
])
def test_20261014_left_return_with_fresh_hand_age_jitter_certifies_execution(pose):
    """Recorded dropped/disabled pose, real meshes; no CAN or robot commands."""
    import time
    from unittest.mock import Mock, patch
    from esrobo_link.backends import ForwardModel
    from esrobo_teleop.robot.nero_driver import NeroSingleArmDriver
    from esrobo_teleop.robot.hand_geometry import geometry_state
    from esrobo_teleop.robot.linker_hand_driver import L10_PHYSICAL_JOINT_NAMES
    root = Path(__file__).resolve().parents[1]
    cfg = load_config(str(root/'config/teleop_config.yaml'))
    cfg.ik.urdf_path = str(root/'urdf/esrobo_waist_with_head.urdf')
    model = ForwardModel(cfg.ik)
    with patch('esrobo_teleop.robot.nero_driver.NeroArm'):
        driver = NeroSingleArmDriver(cfg.robot, 'left')
    guard = TorsoCollisionGuard(model, 'left', cfg.robot.collision_package_dirs,
        cfg.robot.torso_collision_margin_m, cfg.robot.shoulder_collision_margin_m)
    calls = [0]
    def hand(_):
        calls[0] += 1
        age = [.001, .016, .003, .019][calls[0] % 4]
        now = time.monotonic()
        return geometry_state([254]*6+[127,128,127,127], now-age, 'left',
            cfg.hand.geometry_feedback_calibration, L10_PHYSICAL_JOINT_NAMES,
            0., cfg.hand.geometry_feedback_timeout_s, now=now,
            cover_valid_age=not guard._entry_coverage_only)
    guard.hand_state_provider = hand
    guard._build_fingers = Mock(wraps=guard._build_fingers)
    driver._collision_guard = guard
    driver._entry_arm_only_collision = False
    direction, zero = driver._mapping('left')
    lower, upper = driver._effective_position_limits('left')
    start = zero+direction*np.array(pose)
    def full(q):
        out = np.zeros(14); out[:7] = (q-zero)/direction
        return out
    fk = model.fk('left')
    driver.configure_command_trajectory(fk, cfg.retarget.max_endpoint_translation_velocity_m_s)
    guard.begin_entry_certificates()
    planner = StagedReturnPlanner(lower, upper, driver._check_physical_path,
        lambda q: fk(full(q)), direction, timeout=20.,
        diagnostics=lambda: guard.last_diagnostics)
    result = planner.plan(start, zero)
    assert result.path, result.reason
    assert not driver._certify_entry_corridors(result.path, lower, upper, guard)
    assert guard._build_fingers.call_count <= 3
    guard._clearance = Mock(side_effect=AssertionError('geometry after freezing'))
    for a, b in zip(driver._certified_corridor_path, driver._certified_corridor_path[1:]):
        assert driver._check_physical_path(a, b, entry_coverage_only=True)
    # Every point on the original route must remain covered after subdivision;
    # runtime requires each actual braking request to fit a single proof.
    for a, b in zip(result.path, result.path[1:]):
        for fraction in np.linspace(0, 1, 101):
            q = a+(b-a)*fraction
            assert driver._check_physical_path(q, q, entry_coverage_only=True)
    driver._arm.move_j.assert_not_called()
    # Execute against a fake actuator using the frozen REAL mesh proofs. No
    # geometry fallback or controller command can reach physical hardware.
    from test_safe_return import make_driver
    executor, measured, _ = make_driver('left', initial=direction*np.array(pose))
    executor._collision_guard = guard
    executor.configure_command_trajectory(fk, cfg.retarget.max_endpoint_translation_velocity_m_s)
    executor._return_start_state = None
    with patch('esrobo_teleop.robot.nero_driver.time.sleep'):
        assert executor._execute_return_group(
            np.zeros(14), np.arange(7), zero, np.deg2rad(1.5), 20.,
            handover_path=driver._certified_corridor_path,
            require_zero_command_velocity=True), executor._return_execution_failure
    assert np.max(np.abs(measured-zero)) <= np.deg2rad(1.5)
