"""Absolute anatomical targets, full wrist range and entry continuity; offline."""
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from esrobo_laptop.config import ROOT, read_robot_config, read_settings
from esrobo_laptop.mapping import WristMapping, hand_units
from esrobo_laptop.orientation import AbsoluteHandOrientation, ForearmFrame, ANATOMICAL_NEUTRAL
from esrobo_teleop import math_utils as mu
from esrobo_teleop.ik.solver import IkSolver


@pytest.fixture
def config():
    return read_robot_config(read_settings(ROOT / 'config/laptop.yaml'))


def points():
    return dict(shoulder=np.zeros(3), elbow=np.array([0., 0., -.3]),
                wrist=np.array([.3, .1, -.4]))


@pytest.mark.parametrize('initial_roll', [-125., 0., 60.])
def test_same_absolute_pose_is_independent_of_calibration_and_sdk_wrist_axes(initial_roll):
    arm=points(); carrier=ForearmFrame().update(arm)
    first=Rotation.from_euler('x',initial_roll,degrees=True).as_matrix() @ ANATOMICAL_NEUTRAL
    sdk_basis=Rotation.from_euler('xyz',[31.,-17.,45.],degrees=True).as_matrix()
    tracker=AbsoluteHandOrientation(arm,carrier @ first @ sdk_basis,palm_rotation=carrier @ first)
    np.testing.assert_allclose(tracker.initial_local,first,atol=1e-7)
    desired=Rotation.from_euler('xyz',[110.,12.,-9.],degrees=True).as_matrix() @ ANATOMICAL_NEUTRAL
    # A different Body estimate must not overwrite the actual Hand palm.
    result=tracker.update(arm,carrier @ first @ sdk_basis,palm_rotation=carrier @ desired,
                          full_target=True)
    np.testing.assert_allclose(result,desired,atol=1e-7)
    assert not tracker.diagnostics['source_angle_capped']
    assert tracker.diagnostics['requested_roll_deg'] > 100.


def test_whole_arm_carries_absolute_palm_and_entry_observation_does_not_advance_filter():
    arm=points();carrier=ForearmFrame().update(arm)
    initial=Rotation.from_euler('x',-125.,degrees=True).as_matrix() @ ANATOMICAL_NEUTRAL
    tracker=AbsoluteHandOrientation(arm,carrier @ initial,palm_rotation=carrier @ initial)
    turn=Rotation.from_euler('xyz',[35.,-50.,70.],degrees=True).as_matrix()
    moved={k:turn @ value+[.2,-.1,.3] for k,value in arm.items()}
    target=Rotation.from_euler('xyz',[-20.,15.,-10.],degrees=True).as_matrix() @ ANATOMICAL_NEUTRAL
    result=tracker.update(moved,turn @ carrier @ initial,
                          palm_rotation=turn @ carrier @ target,full_target=True)
    np.testing.assert_allclose(result,target,atol=1e-7)
    np.testing.assert_allclose(tracker.filtered,initial,atol=1e-7)
    arrived=Rotation.from_euler('x',-25.,degrees=True).as_matrix() @ ANATOMICAL_NEUTRAL
    tracker.reset_filter(arrived)
    result=tracker.update(moved,turn @ carrier @ initial,
                          palm_rotation=turn @ carrier @ target)
    assert Rotation.from_matrix(result @ arrived.T).magnitude() <= np.deg2rad(2.001)


@pytest.mark.parametrize('side',['left','right'])
@pytest.mark.parametrize('terminal',[[-145.,-36.,-78.],[-125.,25.,-15.],
                                     [110.,20.,30.],[145.,49.,78.]])
def test_absolute_bounded_ik_recovers_large_joint_angles_with_actual_fk(config,side,terminal):
    solver=IkSolver(config.ik)
    solver.configure_position_envelope(side,getattr(config.robot,f'{side}_joint_soft_lower_limits_urdf'),
                                       getattr(config.robot,f'{side}_joint_soft_upper_limits_urdf'))
    q=np.zeros(14);offset=0 if side=='left' else 7
    q[offset:offset+4]=[.1,-.4,.2,1.]
    reference=q.copy();reference[offset+4:offset+7]=np.deg2rad([40.,10.,-10.])
    wrist=WristMapping(solver,config.robot,side,reference,absolute_pose=True)
    q[offset+4:offset+7]=np.deg2rad(terminal)
    local=wrist.absolute_local_from_robot(q)
    result=wrist.absolute_terminal(local,q)
    np.testing.assert_allclose(np.rad2deg(result),terminal,atol=.05)
    actual=q.copy();actual[offset+4:offset+7]=result
    robot=mu.quat_wxyz_to_matrix(solver.current_task_frame_poses(actual)[f'{side}_wrist'][3:])
    desired=mu.quat_wxyz_to_matrix(wrist.last_diagnostics['desired_world_quat_wxyz'])
    assert np.rad2deg(Rotation.from_matrix(robot.T @ desired).magnitude()) < .05
    assert not wrist.last_diagnostics['joint_norm_capped']


@pytest.mark.parametrize('side',['left','right'])
def test_absolute_wrist_local_pose_is_carried_by_arm_without_zero_rebase(config,side):
    solver=IkSolver(config.ik);offset=0 if side=='left' else 7
    q=np.zeros(14);q[offset:offset+4]=[.1,-.4,.2,1.];q[offset+4:offset+7]=np.deg2rad([-120.,20.,-15.])
    wrist=WristMapping(solver,config.robot,side,np.zeros(14),absolute_pose=True)
    local=wrist.absolute_local_from_robot(q)
    first=wrist.absolute_terminal(local,q)
    first_world=wrist.last_diagnostics['desired_world_quat_wxyz']
    q[offset:offset+3]+=[.2,-.1,.3]
    second=wrist.absolute_terminal(local,q)
    np.testing.assert_allclose(first,second,atol=2e-4)
    a=mu.quat_wxyz_to_matrix(first_world);b=mu.quat_wxyz_to_matrix(wrist.last_diagnostics['desired_world_quat_wxyz'])
    assert np.rad2deg(Rotation.from_matrix(a.T @ b).magnitude()) > 5.


def test_unreachable_wrist_is_projected_inside_each_joint_range_and_recovers(config):
    solver=IkSolver(config.ik)
    solver.configure_position_envelope('right',config.robot.right_joint_soft_lower_limits_urdf,
                                       config.robot.right_joint_soft_upper_limits_urdf)
    wrist=WristMapping(solver,config.robot,'right',np.zeros(14),absolute_pose=True)
    def orientation(degrees):
        result=Rotation.identity()
        for axis,angle in zip(wrist.axes.T,degrees):
            result=result * Rotation.from_rotvec(axis*np.deg2rad(angle))
        return result.as_matrix()
    result=wrist.terminal(orientation([165.,-50.,90.]))
    assert np.all(result >= wrist.terminal_lower-1e-8)
    assert np.all(result <= wrist.terminal_upper+1e-8)
    assert wrist.last_diagnostics['orientation_projected']
    result=wrist.terminal(orientation([-120.,20.,-15.]))
    np.testing.assert_allclose(np.rad2deg(result),[-120.,20.,-15.],atol=.05)


@pytest.mark.parametrize('side',['left','right'])
def test_absolute_finger_angles_do_not_subtract_sampled_open_pose(config,side):
    feedback=np.asarray(getattr(config.hand,f'{side}_open_feedback'))
    angles=np.full(10,.3)
    first=hand_units(angles,feedback,config.hand,side,np.full(10,.05),absolute_angles=True)
    second=hand_units(angles,feedback,config.hand,side,np.full(10,.15),absolute_angles=True)
    assert first==second
    assert first==hand_units(angles,feedback,config.hand,side,absolute_angles=True)


def test_absolute_entry_uses_saved_palm_pose_and_keeps_fingers_stationary(config):
    from esrobo_laptop.pipeline import Pipeline
    from esrobo_laptop.config import Settings
    import time
    solver=IkSolver(config.ik)
    solver.configure_position_envelope('right',config.robot.right_joint_soft_lower_limits_urdf,
                                       config.robot.right_joint_soft_upper_limits_urdf)
    measured=np.zeros(14);measured[7:11]=[.1,-.4,.2,1.]
    wrist=WristMapping(solver,config.robot,'right',measured,absolute_pose=True)
    goal=measured.copy();goal[11:]=np.deg2rad([-125.,20.,-15.])
    poses=solver.current_task_frame_poses(measured)
    poses['hand_local_orientations']={'right':wrist.absolute_local_from_robot(goal)}
    p=Pipeline.__new__(Pipeline);p.cfg=config;p.settings=Settings(side='right',with_hand=True)
    margin=np.deg2rad(config.ik.position_limit_hold_feedback_margin_deg)
    lo,hi=solver.position_envelope('right');p._entry_limits={'right':(lo+margin,hi-margin)}
    p.solvers={'right':solver};p.wrists={'right':wrist};p.hand_references={}
    p.position_limit_holds={'right':False};p.position_limit_approaching={'right':False};p.position_limit_recovery={'right':0}
    hand=[128]*10
    result=p.solve(({'right_arm':time.monotonic()},poses,np.zeros(20)),measured,
                   {'right':{'hand':{'position_unit':hand}}},hold_entry_wrist_and_hand=True)['right']
    np.testing.assert_allclose(np.rad2deg(result['arm_urdf_rad'][4:]),[-125.,0.,0.],atol=.05)
    assert result['hand_unit']==hand
    assert wrist.last_diagnostics['entry_wrist_held']
    live=p.solve(({'right_arm':time.monotonic()},poses,np.zeros(20)),measured,
                 {'right':{'hand':{'position_unit':hand}}})['right']
    np.testing.assert_allclose(np.rad2deg(live['arm_urdf_rad'][4:]),[-125.,20.,-15.],atol=.05)
    assert not wrist.last_diagnostics['entry_wrist_held']


@pytest.mark.parametrize('side',['left','right'])
def test_raw_source_mapping_is_separate_from_filtered_command_and_history(config,side):
    import copy
    solver=IkSolver(config.ik)
    q=np.zeros(14);offset=0 if side=='left' else 7
    q[offset:offset+4]=[.1,-.4,.2,1.]
    wrist=WristMapping(solver,config.robot,side,q,absolute_pose=True)
    filtered=q.copy();filtered[offset+4:offset+7]=np.deg2rad([-60.,2.,-2.])
    source=q.copy();source[offset+4:offset+7]=np.deg2rad([-60.,18.,-30.])
    local=wrist.absolute_local_from_robot(filtered)
    raw=wrist.absolute_local_from_robot(source)
    command=wrist.absolute_terminal(local,q)
    previous=wrist._last_terminal_target.copy()
    diagnostics=copy.deepcopy(wrist.last_diagnostics)
    forearm=wrist.forearm_frame
    target=wrist.source_terminal(raw,q)
    np.testing.assert_allclose(np.degrees(target),[-60.,18.,-30.],atol=.05)
    np.testing.assert_allclose(np.degrees(command),[-60.,2.,-2.],atol=.05)
    np.testing.assert_array_equal(wrist._last_terminal_target,previous)
    assert wrist.last_diagnostics==diagnostics
    assert wrist.forearm_frame is forearm
