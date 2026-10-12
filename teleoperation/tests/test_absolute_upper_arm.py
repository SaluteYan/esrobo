"""Absolute upper direction with retained elbow/plane stabilization; no hardware."""
from pathlib import Path
from unittest import mock
import numpy as np
import pytest
import test_pico_reference as pico
from esrobo_teleop import math_utils as mu
from esrobo_teleop.config import build_config, load_config
from esrobo_teleop.ik.solver import IkSolver

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope='module')
def solver():
    cfg = build_config().ik
    cfg.urdf_path = str(ROOT / 'urdf/esrobo_waist_with_head.urdf')
    return IkSolver(cfg)


def make_device(solver, side, bend=60):
    harness = pico.PicoReferenceTests()
    device = harness.device(side)
    device._cfg.upper_arm_direction_mapping = 'absolute'
    harness.calibrate(device, harness.frames(bend=bend))
    device.rebase_robot_reference(solver.current_task_frame_poses(np.zeros(14)))
    return device


def unit(v):
    return np.asarray(v) / np.linalg.norm(v)


@pytest.mark.parametrize('side', ['left', 'right'])
def test_absolute_direction_is_independent_of_calibration_pose(solver, side):
    device = make_device(solver, side)
    sign = 1 if side == 'left' else -1
    points = dict(shoulder=np.array([0, sign*.2, .5]),
                  elbow=np.array([.2, sign*.3, .3]),
                  wrist=np.array([.4, sign*.4, .4]))
    expected = unit(points['elbow']-points['shoulder'])
    shoulder = device._robot_arm_reference(side)[0]
    results = []
    for yaw in (0, 80):
        rotation = mu.rotvec_to_rotation_matrix([0, 0, np.deg2rad(yaw)])
        reference = {key: rotation @ value for key,value in points.items()}
        setattr(device, f'_reference_{side}_points', reference)
        device.seed_arm_reference_filters(side)
        elbow, wrist = device._retarget_arm_segment_direction_positions(points, side)
        np.testing.assert_allclose(unit(elbow-shoulder), expected, atol=1e-6)
        np.testing.assert_allclose(unit(wrist-elbow), unit(points['wrist']-points['elbow']), atol=2e-6)
        results.append((elbow,wrist))
    np.testing.assert_allclose(results[0],results[1],atol=2e-6)
    assert device.arm_retarget_diagnostics(side)['upper_arm_direction_mapping'] == 'absolute'


@pytest.mark.parametrize('side', ['left', 'right'])
def test_legacy_absolute_upper_keeps_near_straight_forearm_stabilization(solver, side):
    device = make_device(solver, side)
    device._cfg.arm_vector_position_mode = "segment_direction_relative"
    upper = unit([.3, .2, -.6])
    transverse = unit(np.cross(upper,[1.,0.,0.]))
    forearm = np.cos(np.deg2rad(2))*upper + np.sin(np.deg2rad(2))*transverse
    points = dict(shoulder=np.zeros(3), elbow=.3*upper, wrist=.3*upper+.25*forearm)
    with mock.patch.object(device,'_stabilize_forearm_plane',wraps=device._stabilize_forearm_plane) as stabilize:
        elbow,wrist = device._retarget_arm_segment_direction_positions(points,side)
    assert stabilize.called
    shoulder = device._robot_arm_reference(side)[0]
    np.testing.assert_allclose(unit(elbow-shoulder),upper,atol=1e-6)
    bend = np.degrees(np.arccos(np.clip(unit(elbow-shoulder) @ unit(wrist-elbow),-1,1)))
    assert bend == pytest.approx(2.,abs=.002)


def test_non_direct_elbow_filter_seed_also_uses_absolute_upper(solver):
    device = make_device(solver,'right')
    device._cfg.elbow_angle_mapping = 'relative'
    points = dict(shoulder=np.zeros(3),elbow=np.array([.2,-.1,-.2]),wrist=np.array([.4,-.1,-.1]))
    device._reference_right_points = points
    device.seed_arm_reference_filters('right')
    np.testing.assert_allclose(device._filtered_segment_directions['right']['upper_arm'],
                               unit(points['elbow']),atol=1e-6)


def test_configured_source_axes_and_waist_rotation_preserve_absolute_direction(solver):
    device = make_device(solver,'right')
    cfg = load_config(str(ROOT/'config/teleop_config.yaml')).retarget
    assert cfg.upper_arm_direction_mapping == 'absolute'
    device._rotation = np.asarray(cfg.source_to_robot_rotation).reshape(3,3)
    # PICO -Z -> robot forward +X, +Y -> robot up +Z, +X -> robot right -Y.
    np.testing.assert_array_equal(device._rotation @ [0,0,-1], [1,0,0])
    np.testing.assert_array_equal(device._rotation @ [0,1,0], [0,0,1])
    np.testing.assert_array_equal(device._rotation @ [1,0,0], [0,-1,0])
    directions = []
    for yaw,translation in ((0,(0,0,0)), (65,(1,2,3))):
        frames = pico.PicoReferenceTests.frames(bend=50,yaw=yaw,translation=translation)
        device._frames = {name: device._transform_source_pose_to_robot_frame(pose)
                          for name,pose in frames.items()}
        points = device._current_arm_points('right')
        device.seed_arm_reference_filters('right')
        elbow,_ = device._retarget_arm_segment_direction_positions(points,'right')
        shoulder = device._robot_arm_reference('right')[0]
        directions.append(unit(elbow-shoulder))
    np.testing.assert_allclose(directions,[[1,0,0],[1,0,0]],atol=2e-6)


def test_raised_recorded_target_is_complete_and_mapping_survives_handover(solver):
    device = make_device(solver,'right')
    points = {'shoulder': np.array([.15116125,-.19750375,.38538972]),
              'elbow': np.array([.31894404,-.25146750,.21676782]),
              'wrist': np.array([.63175166,-.17048858,.33899271])}
    device._reference_right_points = points
    device.seed_arm_reference_filters('right')
    initial = np.zeros(14)
    poses = solver.current_task_frame_poses(initial)
    elbow,wrist = device._retarget_arm_segment_direction_positions(points,'right')
    assert np.linalg.norm(elbow-poses['right_elbow'][:3]) > .1
    poses['right_elbow'][:3],poses['right_wrist'][:3] = elbow,wrist
    solver.initialize_arm_session('right',initial,device.arm_reference_flexion_rad('right'),
                                  relative_elbow_reference=True)
    q = solver.solve(poses['left_wrist'],poses['right_wrist'],initial,
                     poses['left_elbow'],poses['right_elbow'],'right',np.zeros(3))
    assert solver.last_solution_valid, solver.solution_diagnostics
    actual = solver.current_task_frame_poses(q)
    np.testing.assert_allclose(actual['right_elbow'][:3],elbow,atol=.005)
    np.testing.assert_allclose(actual['right_wrist'][:3],wrist,atol=.005)
    device.reset_command_reference(actual)
    solver.reset_command_seed(q)
    again = device._retarget_arm_segment_direction_positions(points,'right')
    np.testing.assert_allclose(again,(elbow,wrist),atol=1e-6)

    # Certify the mapped nonzero endpoint through the actual startup planner.
    from esrobo_teleop.robot.torso_collision import TorsoCollisionGuard
    from esrobo_teleop.robot.handover_planner import HandoverPlanner
    cfg = build_config()
    guard = TorsoCollisionGuard(solver, 'right', include_fingers=False)
    def full(arm):
        value = np.zeros(14); value[7:] = arm
        return value
    def check(a,b):
        return guard(full(a),full(b))
    fk = solver.make_command_fk('right')
    planner = HandoverPlanner(cfg.robot.right_joint_soft_lower_limits_urdf,
        cfg.robot.right_joint_soft_upper_limits_urdf,check,lambda arm:fk(full(arm)),np.ones(7))
    path = planner.plan(np.zeros(7),q[7:])
    assert path.path, path.reason
    assert all(check(a,b) for a,b in zip(path.path,path.path[1:]))


def test_first_entry_uses_saved_calibration_points_not_later_live_frame(solver):
    device = make_device(solver, 'right')
    reference = device.calibrated_entry_source('right')
    frozen = device.calibrated_entry_poses('right')
    for key, pose in device._frames.items():
        moved = pose.copy()
        moved[:3] += np.array([.2, 0, 0])
        device._frames[key] = moved
    assert device.calibrated_entry_source('right') == reference
    again = device.calibrated_entry_poses('right')
    np.testing.assert_allclose(again['right_elbow'], frozen['right_elbow'], atol=1e-6)
    np.testing.assert_allclose(again['right_wrist'], frozen['right_wrist'], atol=1e-6)
