import numpy as np
import pytest
from dataclasses import replace
from unittest import mock
from esrobo_teleop.robot.handover_planner import HandoverPlanner
from esrobo_teleop.robot.return_planner import ReturnPlan
from esrobo_teleop.robot.command_trajectory import CommandTrajectory
from test_safe_return import make_driver as make_return_driver
from test_entry_execution import guard as certificate_guard


def make_driver(*args, **kwargs):
    driver, feedback, zero = make_return_driver(*args, **kwargs)
    driver.configure_collision_guard(certificate_guard())
    return driver, feedback, zero


def fk(q):
    return np.array([[0, .2+q[1], .5], [0, .2+q[1], .2]])


def planner(check=lambda a,b: True, **kwargs):
    return HandoverPlanner(-np.ones(7), np.ones(7), check, fk, np.ones(7), **kwargs)


def test_two_legs_move_shoulder_outward_before_elbow():
    start, goal = np.zeros(7), np.array([0, .2, 0, .5, 0, 0, 0])
    plan = planner().plan(start, goal)
    assert plan.path, plan.reason
    bend_index = next(i for i,q in enumerate(plan.path) if q[3] != 0)
    assert bend_index > 1
    assert plan.path[bend_index-1][1] >= np.deg2rad(8)
    for q in plan.path[:bend_index]:
        np.testing.assert_array_equal(q[2:], start[2:])
    np.testing.assert_allclose(plan.path[-1], goal)


def test_entry_path_holds_measured_j6_j7_exactly():
    start = np.array([0., 0., 0., 0., 0., .17, -.13])
    goal = start.copy()
    goal[1], goal[3] = .2, .5
    plan = planner().plan(start, goal)
    assert plan.path, plan.reason
    assert all(np.array_equal(q[5:], start[5:]) for q in plan.path)
    changed = goal.copy()
    changed[5] += .01
    assert not planner().plan(start, changed).path


def test_entry_moves_j5_only_after_outward_shoulder_leg_and_holds_j6_j7():
    start=np.array([0.,0.,0.,0.,0.,.17,-.13])
    goal=np.array([0.,.2,0.,.5,.4,.17,-.13])
    checks=[]
    def check(a,b):
        checks.append((a.copy(),b.copy()))
        # Shoulder clearance must precede wrist or elbow movement.
        return not (max(np.max(np.abs(a[3:5])),np.max(np.abs(b[3:5]))) > .1
                    and min(a[1],b[1]) < .1)
    result=planner(check).plan(start,goal)
    assert result.path,result.reason
    np.testing.assert_allclose(result.path[-1],goal)
    assert all(check(a,b) for a,b in zip(result.path,result.path[1:]))
    first_wrist=next(i for i,q in enumerate(result.path) if abs(q[4]) > .01)
    assert result.path[first_wrist-1][1] >= np.deg2rad(8.)


def test_entry_executor_rejects_old_full_wrist_goal_before_enable():
    d,feedback,zero=make_driver('right')
    guard=certificate_guard()
    d.configure_collision_guard(guard)
    d._arm.get_joint_enable_states.return_value=[False]*7
    def enable():
        d._enabled=True
        d._arm.get_joint_enable_states.return_value=[True]*7
        return True
    d._enable_for_return=mock.Mock(side_effect=enable)
    target=np.array([.04,-.08,0.,.05,.2,.15,-.2])
    direction,_=d._mapping('right')
    physical=zero+direction*target
    result=d.safe_return(recover=True,handover_goal=target,
                         stationary_hand_check=lambda:True)
    assert not result.returned and 'hold measured J6/J7' in result.reason
    d._enable_for_return.assert_not_called()
    d._arm.move_j.assert_not_called()


def test_safe_endpoints_do_not_justify_unsafe_swept_edge():
    # Any box containing positive elbow bend and insufficient outward motion
    # is rejected, even though its diagonal might escape the obstacle.
    def check(a,b):
        return not (max(a[3],b[3]) > .1 and min(a[1],b[1]) < .1)
    goal = np.array([0,.2,0,.5,0,0,0])
    plan = planner(check).plan(np.zeros(7), goal)
    assert plan.path
    assert all(check(a,b) for a,b in zip(plan.path,plan.path[1:]))


def test_colliding_goal_and_cancel_never_return_path():
    goal = np.array([0,0,0,.5,0,0,0])
    assert not planner(lambda a,b: b[3] < .4).plan(np.zeros(7), goal).path
    assert not planner(cancelled=lambda: True).plan(np.zeros(7), goal).path


def test_nonzero_executor_preserves_enable_and_verifies_waypoints():
    d, feedback, zero = make_driver('right')
    d._arm.get_joint_enable_states.return_value = [False]*7
    goal = np.array([0,-.1,.1,.1,0,0,0])
    directions, _ = d._mapping('right')
    physical = zero+directions*goal
    def enabled():
        d._enabled = True
        d._arm.get_joint_enable_states.return_value = [True]*7
        return True
    d._enable_for_return = mock.Mock(side_effect=enabled)
    callback = mock.Mock()
    path = ReturnPlan((zero.copy(),physical),0,2)
    with mock.patch('esrobo_teleop.robot.handover_planner.HandoverPlanner.plan', return_value=path), \
         mock.patch('esrobo_teleop.robot.nero_driver.time.sleep'):
        result = d.safe_return(recover=True, handover_goal=goal, on_approach=callback)
    assert result.returned and not result.disabled, result
    assert result.stage == 'handover_arrived'
    assert np.max(abs(feedback-physical)) <= np.deg2rad(1.5)
    d._arm.disable.assert_not_called()
    callback.assert_called_once()
    d._enable_for_return.assert_called_once()


@pytest.mark.parametrize('side', ['left', 'right'])
def test_entry_executor_freezes_wrist_and_uses_arm_only_checker(side):
    initial = np.array([0., 0., 0., 0., 0., .17, -.13])
    d, feedback, zero = make_driver(side, initial=initial)
    guard = certificate_guard()
    guard.check_arm_only = mock.Mock(wraps=guard.check_arm_only)
    original_clearance = guard._clearance.side_effect
    def clearance(*args, **kwargs):
        assert kwargs['include_fingers'] is False
        return original_clearance(*args, **kwargs)
    guard._clearance.side_effect = clearance
    d.configure_collision_guard(guard)
    d._arm.get_joint_enable_states.return_value = [False] * 7
    def enable():
        d._enabled = True
        d._arm.get_joint_enable_states.return_value = [True] * 7
        return True
    d._enable_for_return = mock.Mock(side_effect=enable)
    def plan(start, goal):
        np.testing.assert_array_equal(goal[5:], start[5:])
        return ReturnPlan((start.copy(), goal.copy()), 0, 2)
    target = np.array([.04, -.08, 0., .05, 0., .17, -.13])
    with mock.patch('esrobo_teleop.robot.handover_planner.HandoverPlanner.plan', side_effect=plan), \
         mock.patch('esrobo_teleop.robot.nero_driver.time.sleep'):
        result = d.safe_return(recover=True, handover_goal=target,
                               arm_only_collision=True,
                               stationary_hand_check=lambda: True)
    assert result.returned, result
    assert d._arm.move_j.call_count
    for call in d._arm.move_j.call_args_list:
        np.testing.assert_allclose(np.asarray(call.args[0])[5:], zero[5:] + initial[5:])
    guard.check_arm_only.assert_called()
    assert not d._entry_arm_only_collision


def test_nonzero_executor_never_enables_if_plan_rejected():
    d, _, _ = make_driver('right')
    d._arm.get_joint_enable_states.return_value = [False]*7
    d._enable_for_return = mock.Mock()
    with mock.patch('esrobo_teleop.robot.handover_planner.HandoverPlanner.plan',
                    return_value=ReturnPlan((),0,0,'goal unsafe')), \
         mock.patch('esrobo_teleop.robot.nero_driver.time.sleep'):
        result = d.safe_return(recover=True, handover_goal=np.ones(7)*.1)
    assert not result.returned
    d._enable_for_return.assert_not_called()
    d._arm.move_j.assert_not_called()


def test_handover_looks_ahead_without_stopping_at_each_certified_vertex():
    d, feedback, zero = make_driver('right')
    first = zero.copy(); first[0] += .12
    goal = first.copy(); goal[1] += .12
    path = ReturnPlan((zero.copy(), first, goal), 0, 3)
    events = []
    d._emit_probe = lambda event: events.append(event)
    check = d._check_physical_path
    d._check_physical_path = lambda a, b, **kw: not (
        np.allclose(b, goal) and a[0] - zero[0] < .05
    ) and check(a, b, **kw)
    d._arm.get_joint_enable_states.return_value = [False] * 7

    def enable():
        d._enabled = True
        d._arm.get_joint_enable_states.return_value = [True] * 7
        return True

    d._enable_for_return = mock.Mock(side_effect=enable)
    with mock.patch('esrobo_teleop.robot.handover_planner.HandoverPlanner.plan', return_value=path), \
         mock.patch('esrobo_teleop.robot.nero_driver.time.sleep'):
        result = d.safe_return(recover=True, handover_goal=goal-zero,
                               on_approach=lambda: None)
    assert result.returned, result
    assert any(event.get('event') == 'handover_lookahead' for event in events)
    assert not any(np.allclose(call.args[0], first) for call in d._arm.move_j.call_args_list)
    np.testing.assert_allclose(feedback, goal, atol=np.deg2rad(1.5))


def test_staged_return_looks_ahead_and_stops_only_at_zero():
    initial = np.array([.12, 0, 0, 0, 0, 0, 0])
    d, feedback, zero = make_driver('right', initial=initial)
    start = zero + initial
    outward = start.copy(); outward[1] -= .12
    path = ReturnPlan((start, outward, zero.copy()), 0, 3)
    events = []
    d._emit_probe = lambda event: events.append(event)
    d._check_physical_path = lambda a, b, **kw: True
    with mock.patch('esrobo_teleop.robot.staged_return_planner.StagedReturnPlanner.plan',
                    return_value=path), \
         mock.patch('esrobo_teleop.robot.nero_driver.time.sleep'):
        result = d.safe_return()
    assert result.returned and result.disabled, result
    assert len([e for e in events if e.get('event') == 'return_waypoint']) == 1
    assert any(e.get('event') == 'handover_lookahead' for e in events)
    np.testing.assert_allclose(feedback, zero, atol=np.deg2rad(1.5))


def test_staged_return_rejection_sends_no_motion():
    d, _, _ = make_driver('right', initial=np.array([.12, 0, 0, 0, 0, 0, 0]))
    with mock.patch('esrobo_teleop.robot.staged_return_planner.StagedReturnPlanner.plan',
                    return_value=ReturnPlan((), 0, 0, 'no certified path')):
        result = d.safe_return(recover=True)
    assert not result.returned and result.stage == 'planning'
    d._arm.move_j.assert_not_called()
    d._arm.disable.assert_not_called()


def test_handover_accepts_stationary_feedback_with_subthreshold_command_dither():
    d, feedback, zero = make_driver('right')
    d._cfg.full_arm_return_timeout_s = 2.0
    d._arm.get_joint_enable_states.return_value = [False] * 7
    goal = zero.copy(); goal[0] += .01
    path = ReturnPlan((zero.copy(), goal), 0, 2)

    def enable():
        d._enabled = True
        d._arm.get_joint_enable_states.return_value = [True] * 7
        return True

    d._enable_for_return = mock.Mock(side_effect=enable)
    original_commit = CommandTrajectory.commit

    def quantized_commit(trajectory, step):
        original_commit(trajectory, step)
        trajectory.state = replace(trajectory.state, velocity=np.full(7, 1e-5))

    with mock.patch('esrobo_teleop.robot.handover_planner.HandoverPlanner.plan', return_value=path), \
         mock.patch.object(CommandTrajectory, 'commit', quantized_commit), \
         mock.patch('esrobo_teleop.robot.nero_driver.time.sleep'):
        result = d.safe_return(recover=True, handover_goal=goal-zero,
                               on_approach=lambda: None)
    assert result.returned, result
    np.testing.assert_allclose(feedback, goal, atol=np.deg2rad(1.5))


def test_complete_target_bypasses_cartesian_step_and_reset_preserves_mapping():
    import test_pico_reference as pico
    from esrobo_teleop.config import build_config
    from esrobo_teleop.ik.solver import IkSolver
    from pathlib import Path
    harness = pico.PicoReferenceTests()
    device = harness.device('right')
    cfg = build_config()
    cfg.ik.urdf_path = str(Path(__file__).resolve().parents[1]/'urdf/esrobo_waist_with_head.urdf')
    solver = IkSolver(cfg.ik)
    harness.calibrate(device, harness.frames(bend=50))
    initial = solver.current_task_frame_poses(np.zeros(14))
    device.rebase_robot_reference(initial)
    device.seed_arm_reference_filters('right')
    limited = device._retarget_arm_vector_pair('right','right', .02)
    device.reset_command_reference(initial)
    complete = device._retarget_arm_vector_pair('right','right', .02, full_target=True)
    assert np.linalg.norm(limited[1][:3]-initial['right_wrist'][:3]) <= .22*.02+1e-6
    assert np.linalg.norm(complete[1][:3]-initial['right_wrist'][:3]) > .1
    # A nonzero measured seed must not redefine the human directional mapping.
    measured = np.zeros(14); measured[7:11] = np.deg2rad([30,-15,40,45])
    device.reset_command_reference(solver.current_task_frame_poses(measured))
    again = device._retarget_arm_vector_pair('right','right', .02, full_target=True)
    np.testing.assert_allclose(again[0][:3], complete[0][:3], atol=1e-6)
    np.testing.assert_allclose(again[1][:3], complete[1][:3], atol=1e-6)
    solver.initialize_arm_session('right',np.zeros(14),1.,relative_elbow_reference=True)
    references = (dict(solver._j4_flexion_reference),dict(solver._segment_bend_baseline))
    solver.reset_command_seed(measured)
    assert references == (solver._j4_flexion_reference,solver._segment_bend_baseline)
    np.testing.assert_array_equal(solver._prev_targets,measured)


def test_real_urdf_rejects_elbow_only_start_and_certifies_outward_path():
    from pathlib import Path
    from esrobo_teleop.config import build_config
    from esrobo_teleop.ik.solver import IkSolver
    from esrobo_teleop.robot.torso_collision import TorsoCollisionGuard
    cfg = build_config()
    cfg.ik.urdf_path = str(Path(__file__).resolve().parents[1]/'urdf/esrobo_waist_with_head.urdf')
    solver = IkSolver(cfg.ik)
    # This historical outward-route fixture was certified with 30 mm clearance.
    guard = TorsoCollisionGuard(solver,'right',margin=.03,include_fingers=False)
    def full(q):
        v=np.zeros(14);v[7:]=q;return v
    def check(a,b):
        return guard(full(a),full(b))
    fk=solver.make_command_fk('right')
    p=HandoverPlanner(cfg.robot.right_joint_soft_lower_limits_urdf,
        cfg.robot.right_joint_soft_upper_limits_urdf,check,lambda q: fk(full(q)),np.ones(7),
        diagnostics=lambda:guard.last_diagnostics)
    unsafe=np.deg2rad([0,0,0,60,0,0,0])
    assert not p.plan(np.zeros(7),unsafe).path
    assert guard.last_diagnostics['category'] == 'geometry_rejected'
    assert guard.last_diagnostics['required_m'] == .03
    safe=np.deg2rad([30,-15,40,45,0,0,0])
    result=p.plan(np.zeros(7),safe)
    assert result.path,result.reason
    assert len(p.certificates) == len(result.path)-1
    assert all(c['certified'] and c['clearance_m'] > c['required_m'] for c in p.certificates)
    assert result.path[1][1] < 0
    assert result.path[1][3] == 0
    assert all(check(a,b) for a,b in zip(result.path,result.path[1:]))
