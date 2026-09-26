"""Safe return tests; no CAN or ROS hardware construction."""
import threading
import time
from unittest import mock
import numpy as np
import pytest
from esrobo_teleop.robot.return_planner import ReturnPlanner, ReturnResult
from esrobo_teleop.robot.hand_geometry import geometry_state
from esrobo_teleop.robot.linker_hand_driver import ACTIVE_HAND_JOINTS
import test_command_trajectory as trajectory_tests


def planner(check, **kw):
    return ReturnPlanner(-np.ones(7), np.ones(7), check, **kw)


def test_direct_path_and_invalid_goal():
    calls = []
    def check(a, b):
        calls.append((a.copy(), b.copy()))
        return True
    plan = planner(check).plan(np.ones(7)*.2, np.zeros(7))
    assert len(plan.path) == 2 and len(calls) == 3
    assert not planner(lambda a,b: not np.allclose(b,0)).plan(np.ones(7)*.2,np.zeros(7)).path


@pytest.mark.parametrize('side', [-1, 1])
def test_rrt_detours_around_obstacle_with_independent_axis_edges(side):
    def check(a, b):
        lo, hi = np.minimum(a,b), np.maximum(a,b)
        return not (lo[0] <= .2 and hi[0] >= -.2 and lo[1] <= .25 and hi[1] >= -.25)
    start, goal = np.zeros(7), np.zeros(7)
    start[0], goal[0] = -.8*side, .8*side
    plan = planner(check, seed=7, timeout=2).plan(start, goal)
    assert plan.path, plan.reason
    assert len(plan.path) > 2
    assert all(check(a,b) for a,b in zip(plan.path, plan.path[1:]))
    np.testing.assert_allclose(plan.path[0], start)
    np.testing.assert_allclose(plan.path[-1], goal)


def test_search_cancel_and_budget():
    cancel = threading.Event()
    def check(a,b):
        cancel.set()
        return True
    assert not planner(check, cancelled=cancel.is_set).plan(np.zeros(7),np.ones(7)*.2).path
    result = planner(lambda a,b: np.array_equal(a,b), max_nodes=3, timeout=.02).plan(np.zeros(7),np.ones(7)*.2)
    assert not result.path


def calibration():
    return {'left': {'thumb_cmc_roll': dict(raw=[10,210], rad=[0,1], error_rad=.01,
                max_velocity_rad_s=.2, verified=True)}}


def test_hand_independent_calibration_horizon_and_missing_joints():
    state = geometry_state([110]*10,9.95,'left',calibration(),ACTIVE_HAND_JOINTS,.2,.1,now=10.)
    assert state.valid
    np.testing.assert_allclose(state.intervals['left_thumb_cmc_roll'],[.44,.56])
    assert len(state.intervals) == 1
    for stamp in (9.,None,11.):
        assert not geometry_state([110]*10,stamp,'left',calibration(),ACTIVE_HAND_JOINTS,.2,.1,now=10.).valid
    c = calibration(); c['left']['thumb_cmc_roll']['verified'] = False
    assert not geometry_state([110]*10,10.,'left',c,ACTIVE_HAND_JOINTS,.2,.1,now=10.).valid


def make_driver(side='left', initial=None):
    d = trajectory_tests.DriverTrajectoryTests().driver(side)
    _, zero = d._mapping(side)
    feedback = zero.copy() if initial is None else zero+initial
    clock = [10.]
    d._trajectory.invalidate()
    d._last_commanded_full_urdf = None
    d._arm.get_joint_enable_states.return_value = [True]*7
    d._arm.disable.return_value = True
    def read():
        clock[0] += .02
        d._arm.last_feedback_timestamp = clock[0]
        d._return_feedback_received = time.monotonic()
        return feedback.copy()
    def send(q):
        feedback[:] = q
    d.read_joints = mock.Mock(side_effect=read)
    d._arm.move_j.side_effect = send
    return d, feedback, zero


@pytest.mark.parametrize('side', ['left','right'])
def test_complete_simulated_return_verifies_stationary_before_disable(side):
    d, feedback, zero = make_driver(side,np.array([.1,-.15,.2,.3,-.1,.1,-.1]))
    with mock.patch('esrobo_teleop.robot.nero_driver.time.sleep'):
        result = d.safe_return()
    assert result == ReturnResult(True,True,'disabling')
    assert np.max(abs(feedback-zero)) <= np.deg2rad(1.5)
    assert np.max(abs(d._return_start_state.velocity)) < 1e-8
    d._arm.disable.assert_called_once()
    assert d._arm.move_j.call_count > 10


@pytest.mark.parametrize('side', ['left', 'right'])
def test_live_collision_disabled_does_not_bypass_return_preflight(side):
    d, _, _ = make_driver(side)
    assert not d._cfg.teleop_torso_collision_enabled
    d.configure_collision_guard(mock.Mock(return_value=False))
    with mock.patch('esrobo_teleop.robot.nero_driver.time.sleep'):
        result = d.safe_return()
    assert not result.returned
    d._collision_guard.assert_called()
    d._arm.move_j.assert_not_called()


def test_fault_requires_explicit_recovery():
    d, _, _ = make_driver()
    d._return_inhibited = True
    assert not d.safe_return().returned
    d._arm.move_j.assert_not_called()
    with mock.patch('esrobo_teleop.robot.nero_driver.time.sleep'):
        assert d.safe_return(recover=True).returned


def test_checked_q_recovery_can_clear_only_process_issued_stall_estop():
    d, _, _ = make_driver(initial=np.ones(7) * .03)
    enabled = {"value": True}
    d._return_inhibited = True
    d._safety_fault_reason = "left command trajectory: persistent feedback lead"
    d._software_estop_recovery_pending = True
    d._arm.get_joint_enable_states.side_effect = lambda: [enabled["value"]] * 7
    d._configure_arm_collision_safety = mock.Mock(return_value=True)
    d._capture_arm_torque_baseline = mock.Mock(return_value=True)

    def disable():
        enabled["value"] = False
        return True

    def enable(**_kwargs):
        enabled["value"] = True
        return True

    d._arm.disable.side_effect = disable
    d._arm.enable.side_effect = enable

    with mock.patch('esrobo_teleop.robot.nero_driver.time.sleep'):
        result = d.safe_return(recover=True)

    assert result.returned and result.disabled
    assert not d._software_estop_recovery_pending
    d._arm.reset.assert_called_once_with()
    # First disable makes reset safe; the second occurs after verified zero.
    assert d._arm.disable.call_count == 2


def test_software_stop_recovery_checks_current_pose_before_disable_or_reset():
    d, _, _ = make_driver(initial=np.ones(7) * .03)
    d._return_inhibited = True
    d._software_estop_recovery_pending = True
    d._check_physical_path = mock.Mock(return_value=False)

    result = d.safe_return(recover=True)

    assert not result.returned
    assert result.stage == "recovering software stop"
    d._arm.disable.assert_not_called()
    d._arm.reset.assert_not_called()
    d._arm.move_j.assert_not_called()


def test_software_stop_recovery_never_resets_without_verified_disable():
    d, _, _ = make_driver(initial=np.ones(7) * .03)
    d._return_inhibited = True
    d._software_estop_recovery_pending = True
    d._arm.get_joint_enable_states.return_value = [True] * 7
    d._arm.disable.return_value = True

    result = d.safe_return(recover=True)

    assert not result.returned and not result.disabled
    assert "disable not confirmed" in result.reason
    d._arm.reset.assert_not_called()
    d._arm.move_j.assert_not_called()


def test_operator_estop_is_never_auto_reset_by_recovery_return():
    d, _, _ = make_driver(initial=np.ones(7) * .03)
    d._software_estop_recovery_pending = True
    d.emergency_stop()
    d._arm.controller_fault.return_value = {"category": "controller_fault"}

    result = d.safe_return(recover=True)

    assert not result.returned
    assert not d._software_estop_recovery_pending
    d._arm.reset.assert_not_called()
    d._arm.move_j.assert_not_called()


def test_explicit_recovery_clears_verified_inherited_estop_and_returns():
    d, feedback, zero = make_driver(initial=np.array([.01, -.02, .01, .02, 0, 0, 0]))
    enabled = {'value': False}
    reset = {'value': False}
    d._enabled = False
    d._return_inhibited = True
    d._safety_fault_reason = 'inherited emergency stop'
    d._inherited_estop_recovery_pending = True
    d._arm.get_joint_enable_states.side_effect = lambda: [enabled['value']] * 7

    def controller_fault(*, allow_disabled=False, allow_estop=False):
        if reset['value']:
            return None
        if allow_estop and allow_disabled:
            return None
        return {'category': 'controller_fault', 'arm_status': 1,
                'err_code': 0, 'ctrl_mode': 1, 'status_name': 'EMERGENCY_STOP'}

    d._arm.controller_fault.side_effect = controller_fault
    d._arm.reset.side_effect = lambda: reset.update(value=True)
    d._arm.enable.side_effect = lambda **_kwargs: enabled.update(value=True) or True
    d._arm.disable.side_effect = lambda: enabled.update(value=False) or True
    d._configure_arm_collision_safety = mock.Mock(return_value=True)
    d._capture_arm_torque_baseline = mock.Mock(return_value=True)

    with mock.patch('esrobo_teleop.robot.nero_driver.time.sleep'):
        result = d.safe_return(recover=True)

    assert result.returned and result.disabled
    d._arm.reset.assert_called_once_with()
    assert not d._inherited_estop_recovery_pending
    assert not d._software_estop_recovery_pending
    assert np.max(np.abs(feedback-zero)) <= np.deg2rad(1.5)


def test_inherited_estop_recovery_rechecks_driver_fault_before_reset():
    d, _, _ = make_driver(initial=np.ones(7) * .01)
    d._enabled = False
    d._return_inhibited = True
    d._safety_fault_reason = 'inherited emergency stop'
    d._inherited_estop_recovery_pending = True
    d._arm.get_joint_enable_states.return_value = [False] * 7

    def controller_fault(*, allow_disabled=False, allow_estop=False):
        if allow_estop:
            return {'category': 'driver_fault', 'joint': 2, 'flags': ['stall_status']}
        return {'category': 'controller_fault', 'arm_status': 1,
                'err_code': 0, 'ctrl_mode': 1, 'status_name': 'EMERGENCY_STOP'}

    d._arm.controller_fault.side_effect = controller_fault
    result = d.safe_return(recover=True)
    assert not result.returned and result.disabled
    assert result.stage == 'recovering verified inherited stop'
    assert 'another controller/driver fault remains' in result.reason
    d._arm.reset.assert_not_called()
    d._arm.enable.assert_not_called()
    d._arm.move_j.assert_not_called()


def test_current_contact_and_goal_obstruction_refuse_without_moving():
    for mode in ('current','goal'):
        d, _, zero = make_driver(initial=np.ones(7)*.05)
        d._check_physical_path = lambda a,b: False if mode == 'current' else not np.allclose(b,zero)
        with mock.patch('esrobo_teleop.robot.nero_driver.time.sleep'):
            assert not d.safe_return(recover=True).returned
        d._arm.move_j.assert_not_called()
        d._arm.disable.assert_not_called()


def test_cancel_queued_worker_cannot_reset_estop():
    d, _, _ = make_driver()
    request = d.prepare_return()
    d.emergency_stop()
    assert not d.safe_return(recover=True,request_id=request).returned
    d._arm.move_j.assert_not_called()
    d._arm.enable.assert_not_called()


def test_estop_during_return_prevents_subsequent_sends():
    d, _, _ = make_driver(initial=np.ones(7)*.05)
    sent = []
    def send(q):
        sent.append(q)
        d.emergency_stop()
    d._arm.move_j.side_effect = send
    with mock.patch('esrobo_teleop.robot.nero_driver.time.sleep'):
        assert not d.safe_return().returned
    assert len(sent) == 1
    d._arm.emergency_stop.assert_called_once()
    d._arm.disable.assert_not_called()


def test_execution_never_retries_or_disables_after_failure():
    d, _, _ = make_driver()
    d._return_group_to_zero = mock.Mock(return_value=False)
    with mock.patch('esrobo_teleop.robot.nero_driver.time.sleep'):
        assert not d.safe_return().returned
    assert d._return_group_to_zero.call_count == 1
    d._arm.disable.assert_not_called()


def test_return_exposes_specific_waypoint_failure_without_retry_or_disable():
    d, _, _ = make_driver()
    def fail(*_args, **_kwargs):
        d._return_execution_failure = 'feedback did not catch held command within 2.0s; J7 +1.60deg'
        return False
    d._return_group_to_zero = mock.Mock(side_effect=fail)
    with mock.patch('esrobo_teleop.robot.nero_driver.time.sleep'):
        result = d.safe_return()
    assert result.stage == 'executing'
    assert 'J7 +1.60deg' in result.reason
    d._return_group_to_zero.assert_called_once()
    d._arm.disable.assert_not_called()


def test_disabled_return_replans_once_from_new_stationary_start():
    d, feedback, zero = make_driver(initial=np.ones(7) * .03)
    enabled = {'value': False}
    d._enabled = False
    d._arm.get_joint_enable_states.side_effect = lambda: [enabled['value']] * 7
    d._configure_arm_collision_safety = mock.Mock(return_value=True)
    d._capture_arm_torque_baseline = mock.Mock(return_value=True)
    d._arm.enable.side_effect = lambda **_kwargs: enabled.update(value=True) or True
    d._arm.disable.side_effect = lambda: enabled.update(value=False) or True
    stationary_calls = 0
    original_stationary = d._return_stationary

    def stationary(*args, **kwargs):
        nonlocal stationary_calls
        stationary_calls += 1
        value = original_stationary(*args, **kwargs)
        # First call is the pre-plan start; second call validates plan 1.
        if stationary_calls == 2:
            feedback[0] += np.deg2rad(.3)
            value = feedback.copy()
        return value

    d._return_stationary = mock.Mock(side_effect=stationary)
    events = []
    d.probe_sink = events.append
    with mock.patch('esrobo_teleop.robot.nero_driver.time.sleep'):
        result = d.safe_return(recover=True)

    assert result.returned and result.disabled
    plans = [event for event in events if event.get('event') == 'return_plan']
    assert len(plans) == 2
    assert plans[1]['start_physical_rad'][0] == pytest.approx(
        plans[0]['start_physical_rad'][0] + np.deg2rad(.3)
    )
    assert any(event.get('event') == 'return_replan_stationary_start'
               for event in events)


def test_repeated_return_and_estop_while_planning():
    d, _, _ = make_driver()
    entered, released = threading.Event(), threading.Event()
    def check(a,b):
        entered.set()
        released.wait(1.)
        return False
    d._check_physical_path = check
    worker = threading.Thread(target=d.safe_return)
    worker.start()
    assert entered.wait(1.)
    assert d.safe_return().stage == 'busy'
    d.emergency_stop()
    released.set()
    worker.join(1.)
    assert not worker.is_alive()
    d._arm.move_j.assert_not_called()


@pytest.fixture(scope='module')
def model_guards():
    from pathlib import Path
    from esrobo_teleop.config import build_config
    from esrobo_teleop.ik.solver import IkSolver
    from esrobo_teleop.robot.torso_collision import TorsoCollisionGuard
    from esrobo_teleop.robot.hand_geometry import HandGeometryState
    cfg = build_config()
    cfg.ik.urdf_path = str(Path(__file__).resolve().parents[1]/'urdf/esrobo_waist_with_head.urdf')
    solver = IkSolver(cfg.ik)
    guards = {}
    for side in ('left','right'):
        g = TorsoCollisionGuard(solver,side)
        # Synthetic bounded finger pose; not a real hardware calibration.
        g.hand_state_provider = lambda horizon, side=side: HandGeometryState(time.monotonic(),
            {side+'_'+n:(0,.01) for n in ACTIVE_HAND_JOINTS},tuple(ACTIVE_HAND_JOINTS),True)
        guards[side] = g
    return cfg, solver, guards


# Synthetic model poses: safe starts whose distal-first intermediate is unsafe.
STARTS = {
    'left': [.2376655624,.1112910916,.1547638953,.0608903675,-.4676832194,-.0501252390,-.3895147912],
    'right': [-.0363873984,.0350519469,-.2241579660,.0414826534,.3617757344,.0353491246,-.1861129616],
}


@pytest.mark.parametrize('side',['left','right'])
def test_real_model_detour_avoids_unsafe_old_intermediate(model_guards,side):
    cfg, _, guards = model_guards
    guard = guards[side]
    offset = 0 if side == 'left' else 7
    def check(a,b):
        qa,qb = np.zeros(14),np.zeros(14)
        qa[offset:offset+7],qb[offset:offset+7] = a,b
        return guard(qa,qb)
    start, zero = np.array(STARTS[side]), np.zeros(7)
    assert check(start,start)
    assert check(zero,zero)
    old_mid = start.copy(); old_mid[3:] = 0
    assert not check(old_mid,old_mid)
    result = ReturnPlanner(getattr(cfg.robot,side+'_joint_soft_lower_limits_urdf'),
        getattr(cfg.robot,side+'_joint_soft_upper_limits_urdf'),check,timeout=10.).plan(start,zero)
    assert result.path, result.reason
    assert all(check(a,b) for a,b in zip(result.path,result.path[1:]))
    np.testing.assert_allclose(result.path[-1],zero)


def test_real_hand_missing_calibration_fallback_and_stale_rejection(model_guards):
    from esrobo_teleop.robot.hand_geometry import HandGeometryState
    _, _, guards = model_guards
    guard = guards['left']; provider = guard.hand_state_provider
    try:
        assert guard(np.zeros(14),np.zeros(14))
        guard.hand_state_provider = lambda h: HandGeometryState(time.monotonic(),{},(),True)
        assert not guard(np.zeros(14),np.zeros(14))
        guard.hand_state_provider = lambda h: HandGeometryState(None,{},(),False,'stale hand feedback')
        assert not guard(np.zeros(14),np.zeros(14))
        assert guard.last_diagnostics['reason'] == 'stale hand feedback'
    finally:
        guard.hand_state_provider = provider


def test_l10_geometry_uses_hardware_order_not_teleop_order():
    from esrobo_teleop.robot.linker_hand_driver import LinkerHandDriver
    from esrobo_teleop.config import build_config
    cfg = build_config().hand
    cfg.feedback_udp_port = 0
    cfg.geometry_feedback_calibration = calibration()
    hand = LinkerHandDriver(cfg,active_sides=('left',))
    try:
        hand._feedback['left'] = np.array([10.]*9+[110.])
        hand._geometry_feedback_time['left'] = time.monotonic()
        state = hand.geometry_state('left',0.)
        lo,hi = state.intervals['left_thumb_cmc_roll']
        assert lo < .5 < hi
        assert hi-lo < .025
    finally:
        hand.close()


def test_sdk_rejection_stops_without_retry_or_disable():
    d, _, _ = make_driver(initial=np.ones(7)*.05)
    d._arm.move_j.side_effect = None
    d._arm.move_j.return_value = False
    with mock.patch('esrobo_teleop.robot.nero_driver.time.sleep'):
        assert not d.safe_return().returned
    assert d._arm.move_j.call_count == 1
    d._arm.disable.assert_not_called()


def test_recovery_enable_does_not_reset_hardware_and_is_cancellable():
    d, _, _ = make_driver()
    d._arm.get_joint_enable_states.return_value = [False]*7
    d._configure_arm_collision_safety = mock.Mock(return_value=True)
    def enable(**kwargs):
        d.emergency_stop()
        assert kwargs['cancelled']()
        return False
    d._arm.enable.side_effect = enable
    with mock.patch('esrobo_teleop.robot.nero_driver.time.sleep'):
        result = d.safe_return(recover=True)
    assert not result.returned
    assert result.disabled  # Final seven-axis feedback, not a placeholder.
    assert result.stage == 'enabling'
    assert d._arm.move_j.call_count == 1  # Only measured preload before E-stop.
    d._arm.reset.assert_not_called()


def test_disabled_startup_return_captures_baseline_before_torque_monitoring():
    d, _, _ = make_driver(initial=np.ones(7)*.03)
    enabled = {"value": False}
    d._enabled = False
    d._arm.get_joint_enable_states.side_effect = lambda: [enabled["value"]]*7
    d._configure_arm_collision_safety = mock.Mock(return_value=True)

    def enable(**_kwargs):
        enabled["value"] = True
        return True

    d._arm.enable.side_effect = enable
    d._capture_arm_torque_baseline = mock.Mock(return_value=True)

    def torque_is_safe(*_args):
        # The old bug called this while disabled and failed because no baseline
        # could exist. Every runtime torque check must happen after enable and
        # baseline capture.
        assert enabled["value"]
        assert d._capture_arm_torque_baseline.called
        return True

    d._torque_is_safe = mock.Mock(side_effect=torque_is_safe)
    with mock.patch('esrobo_teleop.robot.nero_driver.time.sleep'):
        result = d.safe_return(recover=True)
    assert result.returned and result.disabled
    d._capture_arm_torque_baseline.assert_called_once()
    assert d._torque_is_safe.call_count > 0


def test_recovery_enable_refreshes_feedback_after_collision_configuration():
    d, _, _ = make_driver(initial=np.ones(7)*.03)
    enabled = {"value": False}
    d._enabled = False
    d._arm.get_joint_enable_states.side_effect = lambda: [enabled["value"]]*7

    def configure(*_args):
        # Reproduce the right-arm log: configuration finished after the sample
        # used to enter _enable_for_return had exceeded the command horizon.
        d._return_feedback_received = (
            time.monotonic() - d._cfg.command_trajectory_max_dt_s - .01
        )
        return True

    d._configure_arm_collision_safety = mock.Mock(side_effect=configure)

    def enable(**_kwargs):
        enabled["value"] = True
        return True

    d._arm.enable.side_effect = enable
    d._capture_arm_torque_baseline = mock.Mock(return_value=True)

    configured_while_enabled = []
    d._arm.set_motion_mode.side_effect = lambda *_: configured_while_enabled.append(enabled['value'])
    original_check = d._controller_is_safe

    def check_after_mode(*args, **kwargs):
        if enabled['value']:
            assert configured_while_enabled[-1] is True
        return original_check(*args, **kwargs)

    d._controller_is_safe = check_after_mode
    assert d._enable_for_return()
    assert configured_while_enabled == [False, True]
    assert d.read_joints.call_count == 2
    d._arm.move_j.assert_called_once()
    assert d._last_return_enable_failure is None


@pytest.mark.parametrize('side',['left','right'])
def test_real_model_full_return_with_lag_and_unsynchronized_axes(model_guards,side):
    _, solver, guards = model_guards
    d, feedback, zero = make_driver(side,np.array(STARTS[side]))
    d.configure_collision_guard(guards[side])
    d.configure_command_trajectory(solver.make_command_fk(side),.22)
    target = feedback.copy()
    stamp = [10.]
    tau = np.array([.08,.09,.1,.12,.08,.13,.11])
    def read():
        feedback[:] += .02/tau*(target-feedback)
        stamp[0] += .02
        d._arm.last_feedback_timestamp = stamp[0]
        d._return_feedback_received = time.monotonic()
        return feedback.copy()
    def send(q):
        target[:] = q
    d.read_joints = mock.Mock(side_effect=read)
    d._arm.move_j.side_effect = send
    with mock.patch('esrobo_teleop.robot.nero_driver.time.sleep'):
        result = d.safe_return()
    assert result.returned, result
    assert result.disabled
    assert np.max(abs(feedback-zero)) <= np.deg2rad(1.5)
    d._arm.disable.assert_called_once()


def test_missing_feedback_before_send_aborts():
    d, _, _ = make_driver()
    d._return_feedback_received = time.monotonic()-1.
    assert not d._send_return_position(np.zeros(7))
    d._arm.move_j.assert_not_called()


def test_node_fault_latch_blocks_normal_s_and_q_before_worker():
    from esrobo_teleop.teleop_node import TeleopNode
    node = TeleopNode.__new__(TeleopNode)
    node._driver = mock.Mock()
    node._manual_intervention_required = True
    node._start_full_arm_return('operator s')
    node._driver.prepare_return.assert_not_called()
    node._driver.safe_return.assert_not_called()


def test_unknown_enable_state_never_reports_disabled():
    d, _, _ = make_driver()
    d._enabled = False
    d._arm.get_joint_enable_states.return_value = None
    result = d.safe_return(recover=True)
    assert not result.returned and not result.disabled
    d._arm.move_j.assert_not_called()
