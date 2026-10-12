"""Two-layer arm/hand kinematics; no robot connection or command publication."""
import copy
import time

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from esrobo_laptop.config import ROOT, Settings, read_robot_config, read_settings
from esrobo_laptop.mapping import WristMapping
from esrobo_laptop.orientation import ForearmFrame, RelativeHandOrientation
from esrobo_laptop.pipeline import Pipeline
from esrobo_teleop import math_utils as mu
from esrobo_teleop.ik.solver import IkSolver


def arm_points(bend=70.):
    angle = np.deg2rad(bend)
    return dict(shoulder=np.zeros(3), elbow=np.array([0., 0., -.3]),
                wrist=np.array([.35*np.sin(angle), 0., -.3-.35*np.cos(angle)]))


@pytest.mark.parametrize("axis", np.eye(3).tolist())
@pytest.mark.parametrize("arm_degrees", [0., 35., 120.])
def test_whole_arm_rotation_does_not_contaminate_independent_hand_rotation(axis, arm_degrees):
    points = arm_points()
    carrier = ForearmFrame().update(points)
    hand = Rotation.from_euler("xyz", [23., -17., 9.], degrees=True).as_matrix()
    tracker = RelativeHandOrientation(points, hand)
    whole = Rotation.from_rotvec(np.array([.3, -.4, .5]) / np.sqrt(.5)
                                 * np.deg2rad(arm_degrees)).as_matrix()
    moved = {name: whole @ value + np.array([.3, .1, -.2]) for name, value in points.items()}
    result = tracker.update(moved, whole @ hand, 60., 2., full_target=True)
    np.testing.assert_allclose(result, np.eye(3), atol=1.e-7)
    assert not tracker.diagnostics["source_angle_capped"]
    delta = Rotation.from_rotvec(np.array(axis) * np.deg2rad(25.)).as_matrix()
    result = tracker.update(moved, whole @ carrier @ delta @ carrier.T @ hand,
                            60., 2., full_target=True)
    np.testing.assert_allclose(result, delta, atol=1.e-7)


def test_elbow_flexion_and_straight_crossing_keep_a_continuous_right_handed_carrier():
    initial = arm_points(25.)
    hand = Rotation.from_euler("y", -25., degrees=True).as_matrix()
    tracker = RelativeHandOrientation(initial, hand)
    for bend in np.r_[np.linspace(25., -3., 100), np.linspace(-3., 85., 150)]:
        result = tracker.update(arm_points(bend), Rotation.from_euler("y", -bend, degrees=True).as_matrix(),
                                60., 2.)
        np.testing.assert_allclose(result, np.eye(3), atol=1.e-6)
        assert np.linalg.det(tracker.forearm.rotation) == pytest.approx(1., abs=1.e-7)


def test_entry_observation_cannot_preload_a_large_first_active_wrist_step():
    points = arm_points()
    tracker = RelativeHandOrientation(points, np.eye(3))
    carrier = ForearmFrame().update(points)
    hand = carrier @ Rotation.from_euler("x", 80., degrees=True).as_matrix() @ carrier.T
    for _ in range(5):
        full = tracker.update(points, hand, 60., 2., full_target=True)
    assert np.degrees(Rotation.from_matrix(full).magnitude()) == pytest.approx(60.)
    actual = tracker.update(points, hand, 60., 2.)
    assert np.degrees(Rotation.from_matrix(actual).magnitude()) == pytest.approx(2.)
    tracker.reset_filter()
    again = tracker.update(points, hand, 60., 2.)
    np.testing.assert_allclose(again, actual, atol=1.e-7)


def test_roll_diagnostics_separate_raw_cap_and_step_without_changing_filter():
    from esrobo_laptop.orientation import forearm_twist_deg
    points = arm_points()
    carrier = ForearmFrame().update(points)
    tracker = RelativeHandOrientation(points, np.eye(3))
    hand = carrier @ Rotation.from_euler("x", 80., degrees=True).as_matrix() @ carrier.T
    result = tracker.update(points, hand, 60., 2.)
    assert tracker.diagnostics["requested_roll_deg"] == pytest.approx(80.)
    assert tracker.diagnostics["limited_roll_deg"] == pytest.approx(60.)
    assert tracker.diagnostics["filtered_roll_deg"] == pytest.approx(2.)
    assert forearm_twist_deg(result) == pytest.approx(2.)
    swing = Rotation.from_euler("y", 35., degrees=True).as_matrix()
    twist = Rotation.from_euler("x", -40., degrees=True).as_matrix()
    assert forearm_twist_deg(swing @ twist) == pytest.approx(-40.)
    assert forearm_twist_deg(Rotation.from_euler("y", 180., degrees=True).as_matrix()) is None


@pytest.fixture
def config():
    return read_robot_config(read_settings(ROOT / "config/laptop.yaml"))


def test_combined_wrist_angles_are_not_rescaled_after_bounded_ik(config):
    wrist = WristMapping(IkSolver(config.ik), config.robot, "right", np.zeros(14), max_angle_deg=60.)
    desired = Rotation.identity()
    for axis, degrees in zip(wrist.axes.T, [55., 25., -15.]):
        desired = desired * Rotation.from_rotvec(axis*np.deg2rad(degrees))
    target = wrist.terminal(desired.as_matrix())
    diagnostics = wrist.last_diagnostics
    assert not diagnostics["joint_norm_capped"]
    np.testing.assert_allclose(diagnostics["unscaled_target_joint_deg"], [55., 25., -15.], atol=.001)
    assert diagnostics["joint_norm_scale"] == 1.
    assert diagnostics["j5_norm_reduction_deg"] == pytest.approx(55.-np.degrees(target[0]), abs=.001)
    np.testing.assert_allclose(np.degrees(target), [55., 25., -15.], atol=.001)


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("clock", [0., 45., 90., -90., 180.])
@pytest.mark.parametrize("motion,joint", [("flexion", 2), ("deviation", 1), ("roll", 0)])
def test_anatomical_palm_clock_keeps_flexion_deviation_and_roll_channels(config, side, clock, motion, joint):
    solver = IkSolver(config.ik)
    solver.configure_position_envelope(side, getattr(config.robot, f"{side}_joint_soft_lower_limits_urdf"),
                                      getattr(config.robot, f"{side}_joint_soft_upper_limits_urdf"))
    q = np.zeros(14)
    poses = solver.current_task_frame_poses(q)
    points = {k: poses[f"{side}_{k}"][:3] for k in ("shoulder", "elbow", "wrist")}
    carrier = ForearmFrame().update(points)
    robot_palm = mu.quat_wxyz_to_matrix(poses[f"{side}_wrist"][3:])
    if side == "left":
        robot_palm = robot_palm @ np.diag([-1., -1., 1.])
    human_palm = carrier @ Rotation.from_euler("x", clock, degrees=True).as_matrix() @ carrier.T @ robot_palm
    # SDK quaternion's constant local axes deliberately differ from landmarks.
    body_hand = human_palm @ Rotation.from_euler("xyz", [29., -13., 41.], degrees=True).as_matrix()
    reference = RelativeHandOrientation(points, body_hand, palm_rotation=human_palm)
    wrist = WristMapping(solver, config.robot, side, q, max_angle_deg=60.,
                         source_palm_clock=reference.palm_reference_clock)
    axis = {"flexion": human_palm[:, 1], "deviation": human_palm[:, 0], "roll": carrier[:, 0]}[motion]
    for degrees in (-10., 10.):
        changed = Rotation.from_rotvec(axis*np.deg2rad(degrees)).as_matrix() @ body_hand
        delta = reference.update(points, changed, 60., 2., full_target=True)
        target = np.degrees(wrist.relative_terminal(delta, q))
        assert np.argmax(np.abs(target)) == joint
        assert abs(target[joint]) == pytest.approx(10., abs=.15)
        assert np.max(np.abs(np.delete(target, joint))) < .15
        assert wrist.last_diagnostics["anatomical_alignment"]
        # Straight-arm fixture: use observable transport without an axial
        # twist (three collinear points cannot identify whole-arm roll).
        whole = Rotation.from_rotvec(carrier[:, 1]*np.deg2rad(35.)).as_matrix()
        moved = {k: whole @ v + [.2, -.1, .3] for k, v in points.items()}
        moved_delta = reference.update(moved, whole @ changed, 60., 2., full_target=True)
        np.testing.assert_allclose(moved_delta, delta, atol=1.e-7)
        np.testing.assert_allclose(wrist.relative_terminal(moved_delta, q), np.deg2rad(target), atol=1.e-5)
        clock_before = reference.palm_reference_clock
        reference.reset_filter()
        assert reference.palm_reference_clock == clock_before
        reference.update(points, body_hand, 60., 2., full_target=True)


def test_palm_clock_does_not_tilt_roll_axis_for_bent_calibration_wrist():
    from esrobo_laptop.orientation import palm_clock
    carrier = ForearmFrame().update(arm_points())
    palm = carrier @ Rotation.from_euler("z", 35., degrees=True).as_matrix()
    tracker = RelativeHandOrientation(arm_points(), palm, palm_rotation=palm)
    clock = tracker.palm_reference_clock
    assert np.isfinite(clock)
    alignment = Rotation.from_euler("x", -clock).as_matrix()
    np.testing.assert_allclose(alignment @ np.array([1., 0., 0.]), [1., 0., 0.], atol=1e-7)
    with pytest.raises(ValueError, match="parallel"):
        palm_clock(carrier, carrier[:, 0])


def make_pipeline(config, side, reference):
    p = Pipeline.__new__(Pipeline)
    p.cfg, p.settings = config, Settings(side=side, with_hand=True)
    solver = IkSolver(config.ik)
    solver.configure_position_envelope(side, getattr(config.robot, f"{side}_joint_soft_lower_limits_urdf"),
                                       getattr(config.robot, f"{side}_joint_soft_upper_limits_urdf"))
    p.solvers = {side: solver}
    p.wrists = {side: WristMapping(solver, config.robot, side, reference, absolute_pose=True)}
    p.position_limit_holds = {side: False}
    p.position_limit_approaching = {side: False}
    p.position_limit_recovery = {side: 0}
    p.hand_references = {}
    return p


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("axis", np.eye(3).tolist())
def test_independent_hand_motion_keeps_arm_solution_and_moves_physical_palm(config, side, axis):
    offset = 0 if side == "left" else 7
    measured = np.zeros(14)
    measured[offset:offset+4] = [.05, -.36, .96, 1.25]
    p = make_pipeline(config, side, measured)
    solver, wrist = p.solvers[side], p.wrists[side]
    poses = solver.current_task_frame_poses(measured)
    delta = Rotation.from_rotvec(np.array(axis) * np.deg2rad(20.)).as_matrix()
    poses["hand_local_orientations"] = {side: delta @ wrist.absolute_local_from_robot(measured)}
    output = p.solve(({f"{side}_arm": time.monotonic()}, poses, np.zeros(20)), measured,
                     {side: {"hand": {"position_unit": [128]*10}}})
    joints = np.array(output[side]["arm_urdf_rad"])
    np.testing.assert_allclose(joints[:4], measured[offset:offset+4], atol=2.e-5)
    assert np.linalg.norm(joints[4:]) > .1
    assert solver.last_solution_valid
    assert solver.solution_diagnostics["wrist_error_m"] < 1.e-5
    actual = measured.copy()
    actual[offset:offset+7] = joints
    world = solver.current_task_frame_poses(actual)[f"{side}_wrist"]
    desired = wrist.last_diagnostics["desired_world_quat_wxyz"]
    error = Rotation.from_matrix(mu.quat_wxyz_to_matrix(world[3:]).T
                                 @ mu.quat_wxyz_to_matrix(desired)).magnitude()
    assert np.degrees(error) < .1
    diagnostic = p.arm_diagnostics(({f'{side}_arm': time.monotonic()}, poses, None), output, measured,
                                   preview=True, robot_mode='IDLE')
    np.testing.assert_allclose(diagnostic['frames']['arm_position_reference'][f'{side}_wrist']['pos'],
                               poses[f'{side}_wrist'][:3], atol=1.e-5)
    np.testing.assert_allclose(diagnostic['frames']['retarget'][f'{side}_wrist']['pos'],
                               world[:3], atol=1.e-5)


@pytest.mark.parametrize("side", ["left", "right"])
def test_hand_world_attitude_is_carried_by_arm_with_constant_local_wrist(config, side):
    offset = 0 if side == "left" else 7
    q = np.zeros(14)
    q[offset:offset+4] = [.05, -.4, .3, .9]
    p = make_pipeline(config, side, q)
    wrist = p.wrists[side]
    from esrobo_laptop.orientation import palm_clock
    poses = p.solvers[side].current_task_frame_poses(q)
    carrier = ForearmFrame().update({k: poses[f"{side}_{k}"][:3] for k in ("shoulder", "elbow", "wrist")})
    normal = mu.quat_wxyz_to_matrix(poses[f"{side}_wrist"][3:])[:, 0] * (1. if side == "right" else -1.)
    wrist.source_palm_clock = palm_clock(carrier, normal)
    local = Rotation.from_euler("xyz", [10., 8., -12.], degrees=True).as_matrix()
    first_terminal = wrist.relative_terminal(local, q)
    first_world = np.array(wrist.last_diagnostics["desired_world_quat_wxyz"])
    q[offset:offset+3] += [.15, -.1, .2]
    second_terminal = wrist.relative_terminal(local, q)
    np.testing.assert_allclose(second_terminal, first_terminal, atol=2.e-4)
    second_world = np.array(wrist.last_diagnostics["desired_world_quat_wxyz"])
    assert np.degrees(Rotation.from_matrix(mu.quat_wxyz_to_matrix(first_world).T
                       @ mu.quat_wxyz_to_matrix(second_world)).magnitude()) > 5.


def test_september_30_entry_pose_no_longer_generates_60_degree_wrist_compensation(config):
    # Recorded 1790740651813995216 handover; the old first ACTIVE candidate
    # was J5..J7 = [-16.97, 34.59, -45.99] deg and failed at 34.43 mm.
    q = np.zeros(14)
    q[7:] = [0.0571246264, -0.3675663405, 0.9660921009, 1.2453447812,
             -0.0009075712, -0.0001047198, -0.0069638637]
    reference = np.zeros(14)
    reference[11:] = q[11:]
    p = make_pipeline(config, "right", reference)
    solver = p.solvers["right"]
    poses = solver.current_task_frame_poses(q)
    poses["right_elbow"][:3] = [-.1302839369, -.3463668525, .9036742449]
    poses["right_wrist"][:3] = [.1917256266, -.1918123066, .7247045636]
    for axis in np.eye(3):
        frame_poses = copy.deepcopy(poses)
        frame_poses["hand_local_orientations"] = {
            "right": Rotation.from_rotvec(axis * np.deg2rad(3.)).as_matrix() @ p.wrists["right"].absolute_local_from_robot(q)}
        out = p.solve(({"right_arm": time.monotonic()}, frame_poses, np.zeros(20)), q,
                      {"right": {"hand": {"position_unit": [128]*10}}})
        candidate = np.array(out["right"]["arm_urdf_rad"])
        assert solver.last_solution_valid
        assert solver.solution_diagnostics["wrist_error_m"] < .001
        # Absolute position mapping uses a neutral terminal carrier rather
        # than the recorded -0.4-degree J7 reference; this changes J4 ~0.13 deg.
        assert np.max(np.abs(candidate[:4]-q[7:11])) < np.deg2rad(.2)
        assert np.linalg.norm(candidate[4:]-reference[11:]) < np.deg2rad(5.)
