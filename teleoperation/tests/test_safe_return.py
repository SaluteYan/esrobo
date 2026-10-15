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


def test_locked_wrist_stays_fixed_through_rrt_detour():
    start = np.array([-.5, 0., 0., 0., 0., .17, -.13])
    goal = start.copy()
    goal[0] = .5
    def check(a, b):
        lo, hi = np.minimum(a, b), np.maximum(a, b)
        return not (lo[0] <= .1 and hi[0] >= -.1 and lo[1] <= .1 and hi[1] >= -.1)
    plan = planner(check, locked_joints=(5, 6), seed=7, timeout=2).plan(start, goal)
    assert plan.path, plan.reason
    assert all(np.array_equal(q[5:], start[5:]) for q in plan.path)


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
    from esrobo_teleop.robot.nero_driver import JointFeedbackSnapshot
    def entry_snapshot(**kwargs):
        values = d.read_joints()
        if values is None:
            return None
        return JointFeedbackSnapshot(values, d._arm.last_feedback_timestamp,
                                     d._return_feedback_received, 'verified-test-source')
    d._arm.runtime_feedback.side_effect = entry_snapshot
    d.read_joints = mock.Mock(side_effect=read)
    d._arm.move_j.side_effect = send
    return d, feedback, zero


def test_release_unstarted_return_preserves_cancel_and_allows_new_request():
    d, _, _ = make_driver()
    request_id = d.prepare_return()
    assert request_id is not None and d.prepare_return() is None
    d.cancel_return()
    d.release_return_request(request_id)
    assert not d._return_pending and d._return_cancel.is_set()
    following = d.prepare_return()
    assert following == request_id + 1 and not d._return_cancel.is_set()
    d.release_return_request(request_id)
    assert d._return_pending


@pytest.mark.parametrize('side', ['left', 'right'])
def test_explicit_recovery_at_disabled_zero_never_enables_or_sends_position(side):
    d, _, _ = make_driver(side, initial=np.full(7, .0005))
    d._enabled = False
    d._arm.get_joint_enable_states.return_value = [False]*7
    with mock.patch('esrobo_teleop.robot.nero_driver.time.sleep'):
        result = d.safe_return(recover=True, recognize_inherited_stop=True)
    assert result.returned and result.disabled
    assert result.stage == 'verifying_disabled_zero'
    d._arm.enable.assert_not_called()
    d._arm.move_j.assert_not_called()
    d._arm.disable.assert_not_called()


def test_disabled_zero_recovery_cannot_report_success_if_final_feedback_missing():
    d, _, zero = make_driver()
    d._enabled = False
    d._arm.get_joint_enable_states.return_value = [False]*7
    d._return_stationary = mock.Mock(side_effect=[zero.copy(), None])
    result = d.safe_return(recover=True, recognize_inherited_stop=True)
    assert not result.returned and result.stage == 'verifying_disabled_zero'
    d._arm.enable.assert_not_called()
    d._arm.move_j.assert_not_called()


def test_release_reservation_cannot_end_running_return():
    d, _, _ = make_driver()
    request_id = d.prepare_return()
    d._return_active = True
    d.release_return_request(request_id)
    assert d._return_active and d._return_pending


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
def test_return_execution_uses_verified_snapshot_without_two_sample_wait(side):
    from esrobo_teleop.robot.nero_driver import JointFeedbackSnapshot
    d, feedback, zero = make_driver(side, np.ones(7)*.04)
    clock = [10.]
    def snapshot(**kwargs):
        clock[0] += .02
        d._arm.last_feedback_timestamp = clock[0]
        return JointFeedbackSnapshot(feedback.copy(), clock[0], time.monotonic(), 'verified')
    d._arm.runtime_feedback.side_effect = snapshot
    d.read_joints.side_effect = AssertionError('blocking startup reader used during execution')
    with mock.patch('esrobo_teleop.robot.nero_driver.time.sleep'):
        assert d._execute_return_group(np.zeros(14), np.arange(7), zero,
                                      np.deg2rad(1.5), 2.)
    d.read_joints.assert_not_called()
    assert d._arm.runtime_feedback.call_count > 10
    assert np.max(abs(feedback-zero)) <= np.deg2rad(1.5)


@pytest.mark.parametrize('failure', [None, 'cancel', 'exception'])
def test_return_full_geometry_cache_lifetime_and_context(failure):
    from test_entry_execution import guard
    g = guard()
    d, _, _ = make_driver(initial=np.ones(7)*.04)
    d.configure_collision_guard(g)
    original = d._return_group_to_zero
    def execute(*args, **kwargs):
        assert g._entry_certificates is not None
        assert not g._entry_arm_geometry_only
        assert g._entry_coverage_only
        if failure == 'exception':
            raise RuntimeError('test cancellation cleanup')
        if failure == 'cancel':
            d._return_cancel.set()
            return False
        return original(*args, **kwargs)
    d._return_group_to_zero = execute
    with mock.patch('esrobo_teleop.robot.nero_driver.time.sleep'):
        result = d.safe_return()
        assert result.returned == (failure is None)
        if failure == 'exception':
            assert 'test cancellation cleanup' in result.reason
    assert g._entry_certificates is None
    assert not g._entry_coverage_only


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


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('estop', [True, False])
@pytest.mark.parametrize('geometry_ok', [True, False])
def test_disabled_controller_recovery_is_stationary_and_clears_only_after_verification(side, estop, geometry_ok):
    d, _, _ = make_driver(side)
    d._arm.get_joint_enable_states.return_value = [False]*7
    d._safety_fault_reason = 'operator emergency stop; explicit recovery required'
    pending = [estop]
    def fault(**kw):
        return ({'category': 'controller_fault', 'arm_status': 1, 'err_code': 0, 'ctrl_mode': 1}
                if pending[0] and not kw.get('allow_estop') else None)
    d._arm.controller_fault.side_effect = fault
    d._arm.reset.side_effect = lambda: pending.__setitem__(0, False)
    d._check_physical_path = mock.Mock(return_value=geometry_ok)
    if estop and not geometry_ok:
        assert d._verify_disabled_estop_recovery_candidate(d.read_joints()) == (
            False, 'current pose failed collision check')
    d._check_physical_path.reset_mock()
    with mock.patch('esrobo_teleop.robot.nero_driver.time.sleep'):
        r = d.recover_disabled_controller()
    assert r == dict(disabled=True, reset_sent=estop, verified_samples=3)
    assert d._safety_fault_reason is None and not d._enabled
    assert d._arm.reset.call_count == int(estop)
    d._arm.enable.assert_not_called()
    d._arm.move_j.assert_not_called()
    d._check_physical_path.assert_not_called()


@pytest.mark.parametrize('failure', ['enabled', 'mixed', 'limits', 'missing', 'driver', 'cancel', 'drift', 'duplicate'])
def test_disabled_controller_recovery_rejects_unverified_state(failure):
    d, feedback, _ = make_driver()
    d._arm.get_joint_enable_states.return_value = [False]*7
    d._safety_fault_reason = 'latched'
    d._arm.controller_fault.return_value = None
    if failure == 'enabled': d._arm.get_joint_enable_states.return_value = [True]*7
    if failure == 'mixed': d._arm.get_joint_enable_states.return_value = [False]*6+[True]
    if failure == 'limits': d._within_hard_limits = mock.Mock(return_value=False)
    if failure == 'missing': d.read_joints.return_value = None; d.read_joints.side_effect = None
    if failure == 'driver': d._arm.controller_fault.return_value = {'category': 'driver_fault'}
    if failure == 'drift':
        d.read_joints.side_effect = [feedback.copy(), feedback.copy()+.01]
    if failure == 'duplicate':
        d.read_joints.side_effect = None
        d.read_joints.return_value = feedback.copy()
        d._arm.last_feedback_timestamp = 1.
    with mock.patch('esrobo_teleop.robot.nero_driver.time.sleep'), \
            mock.patch('esrobo_teleop.robot.nero_driver.time.monotonic', side_effect=np.arange(0., 20., .1)):
        with pytest.raises(RuntimeError):
            d.recover_disabled_controller(lambda: failure == 'cancel')
    assert d._safety_fault_reason == 'latched'
    d._arm.reset.assert_not_called()
    d._arm.enable.assert_not_called()
    d._arm.move_j.assert_not_called()


@pytest.mark.parametrize('failure', ['enabled_after_reset', 'driver_after_reset', 'cancel_after_reset'])
def test_disabled_reset_never_commits_failed_hardware_recovery(failure):
    d, _, _ = make_driver()
    d._safety_fault_reason = 'latched'
    reset = [False]
    d._arm.get_joint_enable_states.side_effect = lambda: [reset[0] and failure == 'enabled_after_reset']*7
    def fault(**kw):
        if reset[0]:
            return {'category': 'driver_fault'} if failure == 'driver_after_reset' else None
        return (None if kw.get('allow_estop') else
                {'category': 'controller_fault', 'arm_status': 1, 'err_code': 0, 'ctrl_mode': 1})
    d._arm.controller_fault.side_effect = fault
    d._arm.reset.side_effect = lambda: reset.__setitem__(0, True)
    with pytest.raises(RuntimeError):
        d.recover_disabled_controller(lambda: reset[0] and failure == 'cancel_after_reset')
    assert d._safety_fault_reason == 'latched'
    assert d._arm.disable.call_count == int(failure == 'enabled_after_reset')
    d._arm.enable.assert_not_called()
    d._arm.move_j.assert_not_called()


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('category', ['controller_feedback_stale', 'driver_feedback_stale'])
def test_explicit_disabled_recovery_restores_telemetry_without_blind_wake(side, category):
    d, _, _ = make_driver(side)
    d._safety_fault_reason = 'feedback stale'
    d._arm.get_joint_enable_states.return_value = [False]*7
    requested = [False]
    d._arm.controller_fault.side_effect = lambda **kw: None if requested[0] else {'category': category}
    def request():
        requested[0] = True
        return True
    d._arm.request_can_feedback_push.side_effect = request
    with mock.patch('esrobo_teleop.robot.nero_driver.time.sleep'):
        result = d.recover_disabled_controller()
    assert result == dict(disabled=True, reset_sent=False, verified_samples=3)
    assert d._safety_fault_reason is None
    d._arm.request_can_feedback_push.assert_called_once_with()
    d._arm.reset.assert_not_called()
    d._arm.enable.assert_not_called()
    d._arm.move_j.assert_not_called()


@pytest.mark.parametrize('failure', ['timeout', 'unavailable', 'cancel', 'enabled', 'driver_fault'])
def test_telemetry_request_cannot_clear_unknown_or_faulted_state(failure):
    d, _, _ = make_driver()
    d._safety_fault_reason = 'latched'
    requested = [False]
    def request():
        requested[0] = True
        return failure != 'unavailable'
    d._arm.request_can_feedback_push.side_effect = request
    def fault(**kw):
        if not requested[0] or failure in ('timeout', 'unavailable', 'cancel'):
            return {'category': 'controller_feedback_stale'}
        return {'category': 'driver_fault'} if failure == 'driver_fault' else None
    d._arm.controller_fault.side_effect = fault
    d._arm.get_joint_enable_states.return_value = [failure == 'enabled']*7
    with mock.patch('esrobo_teleop.robot.nero_driver.time.monotonic', side_effect=np.arange(0., 30., .1)):
        with pytest.raises(RuntimeError):
            d.recover_disabled_controller(lambda: requested[0] and failure == 'cancel')
    assert d._safety_fault_reason == 'latched'
    d._arm.reset.assert_not_called()
    d._arm.enable.assert_not_called()
    d._arm.move_j.assert_not_called()


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('marker_known', [True, False])
def test_explicit_recovery_clears_verified_inherited_estop_and_returns(side, marker_known):
    d, feedback, zero = make_driver(side, initial=np.array([.01, -.02, .01, .02, 0, 0, 0]))
    enabled = {'value': False}
    reset = {'value': False}
    d._enabled = False
    d._return_inhibited = True
    d._safety_fault_reason = 'inherited emergency stop'
    d._inherited_estop_recovery_pending = marker_known
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
        result = d.safe_return(recover=True, recognize_inherited_stop=not marker_known)

    assert result.returned and result.disabled
    d._arm.reset.assert_called_once_with()
    assert not d._inherited_estop_recovery_pending
    assert not d._software_estop_recovery_pending
    assert np.max(np.abs(feedback-zero)) <= np.deg2rad(1.5)


@pytest.mark.parametrize('failure', ['automatic', 'enabled', 'mixed', 'other_fault', 'geometry', 'missing_pose'])
def test_new_process_estop_recognition_never_bypasses_prechecks(failure):
    d, _, _ = make_driver()
    d._inherited_estop_recovery_pending = False
    d._software_estop_recovery_pending = False
    d._arm.get_joint_enable_states.return_value = (
        [True] * 7 if failure == 'enabled' else
        [False] * 6 + [True] if failure == 'mixed' else [False] * 7)
    def fault(*, allow_disabled=False, allow_estop=False):
        if allow_estop:
            return {'category': 'driver_fault', 'joint': 2} if failure == 'other_fault' else None
        return {'category': 'controller_fault', 'arm_status': 1, 'err_code': 0, 'ctrl_mode': 1}
    d._arm.controller_fault.side_effect = fault
    if failure == 'geometry':
        d._check_physical_path = lambda a, b: False
    if failure == 'missing_pose':
        d.read_joints = mock.Mock(return_value=None)
    result = d.safe_return(recover=True, recognize_inherited_stop=failure != 'automatic')
    assert not result.returned
    d._arm.reset.assert_not_called()
    d._arm.enable.assert_not_called()
    d._arm.move_j.assert_not_called()


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
        # These recorded detour fixtures were certified at the former 30 mm
        # margin; do not silently change their meaning with the runtime default.
        g = TorsoCollisionGuard(solver, side, margin=.03)
        # Synthetic bounded finger pose; not a real hardware calibration.
        g.hand_state_provider = lambda horizon, side=side: HandGeometryState(time.monotonic(),
            {side+'_'+n:(0,.01) for n in ACTIVE_HAND_JOINTS},tuple(ACTIVE_HAND_JOINTS),True)
        guards[side] = g
    return cfg, solver, guards


def test_entry_arm_only_profile_keeps_arm_clearance_without_hand_feedback(model_guards):
    from esrobo_teleop.robot.hand_geometry import HandGeometryState
    guard = model_guards[2]['right']
    original = guard.hand_state_provider
    try:
        guard.hand_state_provider = lambda horizon: HandGeometryState(
            None, {}, (), False, "stale hand geometry")
        q = np.zeros(14)
        assert not guard(q, q)
        assert guard.last_diagnostics['category'] == 'hand_geometry_unavailable'
        assert guard.check_arm_only(q, q)
        assert guard.last_diagnostics['collision_profile'] == 'arm_wrist_palm'
        assert guard.last_diagnostics['clearance_m'] >= guard.last_diagnostics['required_m']
    finally:
        guard.hand_state_provider = original


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
    assert d.read_joints.call_count == 3
    d._arm.move_j.assert_called_once()
    assert d._last_return_enable_failure is None


@pytest.mark.parametrize('side', ['left', 'right'])
def test_recovery_preload_refreshes_after_expensive_geometry_and_mode(side):
    d, feedback, _ = make_driver(side, initial=np.ones(7)*.03)
    enabled = {'value': False}
    d._enabled = False
    d._arm.get_joint_enable_states.side_effect = lambda: [enabled['value']]*7
    d._configure_arm_collision_safety = mock.Mock(return_value=True)
    d._capture_arm_torque_baseline = mock.Mock(return_value=True)
    boxes = []

    def check(a, b, **_):
        if np.any(a != b):
            boxes.append((a.copy(), b.copy()))
            d._return_feedback_received = time.monotonic() - .2
        return True

    def mode(*_):
        if not enabled['value']:
            d._return_feedback_received = time.monotonic() - .2
            feedback[0] += np.deg2rad(.05)

    def enable(**_):
        enabled['value'] = True
        return True

    d._check_physical_path = mock.Mock(side_effect=check)
    d._arm.set_motion_mode.side_effect = mode
    d._arm.enable.side_effect = enable
    assert d._enable_for_return()
    sent = np.asarray(d._arm.move_j.call_args.args[0])
    assert len(boxes) == 1
    assert np.all(sent >= boxes[0][0]) and np.all(sent <= boxes[0][1])
    np.testing.assert_array_equal(sent, feedback)


@pytest.mark.parametrize('failure', ['drift', 'missing', 'geometry', 'expired', 'cancelled'])
def test_recovery_preload_rejects_changed_boundary_without_enable(failure):
    d, feedback, _ = make_driver(initial=np.ones(7)*.03)
    d._enabled = False
    d._arm.get_joint_enable_states.return_value = [False]*7
    d._configure_arm_collision_safety = mock.Mock(return_value=True)
    mode_done = {'value': False}

    def mode(*_):
        mode_done['value'] = True
        if failure == 'drift':
            feedback[0] += np.deg2rad(.3)
        elif failure == 'missing':
            d.read_joints.side_effect = lambda: None
        elif failure == 'cancelled':
            d.cancel_return()

    def check(*_, **__):
        if mode_done['value'] and failure == 'expired':
            d._return_feedback_received = time.monotonic() - .2
        return not (mode_done['value'] and failure == 'geometry')

    d._arm.set_motion_mode.side_effect = mode
    d._check_physical_path = mock.Mock(side_effect=check)
    assert not d._enable_for_return()
    assert d._last_return_enable_failure
    d._arm.move_j.assert_not_called()
    d._arm.enable.assert_not_called()


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('fault', [
    {'category': 'controller_feedback_stale'},
    {'category': 'motion_mode_mismatch', 'expected': 'j', 'mode_feedback': 0},
    {'category': 'driver_fault', 'joint': 3, 'flags': ['stall_status'], 'enabled': True},
])
def test_post_enable_rejection_survives_gateway_emergency_stop(side, fault):
    d, _, _ = make_driver(side)
    enabled = {'value': False}
    d._enabled = False
    d._arm.get_joint_enable_states.side_effect = lambda: [enabled['value']] * 7
    d._arm.controller_fault.side_effect = lambda **_: fault if enabled['value'] else None
    d._configure_arm_collision_safety = mock.Mock(return_value=True)
    d._capture_arm_torque_baseline = mock.Mock(return_value=True)

    def enable(**_kwargs):
        enabled['value'] = True
        return True

    d._arm.enable.side_effect = enable
    with mock.patch('esrobo_teleop.robot.nero_driver.time.sleep'):
        result = d.safe_return(recover=True)
    assert result.stage == 'enabling' and not result.returned and not result.disabled
    assert 'controller safety check failed after enable' in result.reason
    assert str(fault) in result.reason
    original = d.safety_fault_reason
    d.emergency_stop()
    assert d.safety_fault_reason == original
    assert str(fault) in d.last_return_result.reason
    d._arm.emergency_stop.assert_called_once()
    d._arm.move_j.assert_called_once()  # Only measured-pose preload.
    d._capture_arm_torque_baseline.assert_not_called()
    d._arm.disable.assert_not_called()


def test_explicit_emergency_stop_without_prior_fault_keeps_operator_reason():
    d, _, _ = make_driver()
    d.emergency_stop()
    assert d.safety_fault_reason == 'operator emergency stop; explicit recovery required'
    d._arm.emergency_stop.assert_called_once()


def brake_pending():
    return dict(category='controller_fault', arm_status=6, err_code=0, ctrl_mode=1)


@pytest.mark.parametrize('side', ['left', 'right'])
def test_enable_waits_for_brake_release_without_sending_targets(side):
    d, _, _ = make_driver(side)
    faults = iter([brake_pending(), None, None])
    d._arm.controller_fault.side_effect = lambda **_: next(faults)
    with mock.patch.object(d._return_cancel, 'wait', return_value=False) as wait:
        assert d._wait_return_controller_ready()
    wait.assert_called_once_with(.005)
    assert d.safety_fault_reason is None
    d._arm.move_j.assert_not_called()
    d._arm.enable.assert_not_called()
    d._arm.reset.assert_not_called()


@pytest.mark.parametrize('fault', [
    dict(category='controller_fault', arm_status=1, err_code=0, ctrl_mode=1),
    dict(category='controller_fault', arm_status=6, err_code=1, ctrl_mode=1),
    dict(category='controller_fault', arm_status=6, err_code=0, ctrl_mode=0),
    dict(category='controller_feedback_stale'),
    dict(category='motion_mode_mismatch', expected='j', mode_feedback=0),
])
def test_enable_transition_rejects_other_faults_immediately(fault):
    d, _, _ = make_driver()
    d._arm.controller_fault.return_value = fault
    with mock.patch.object(d._return_cancel, 'wait') as wait:
        assert not d._wait_return_controller_ready()
    wait.assert_not_called()
    assert str(fault) in d.safety_fault_reason


def test_enable_brake_wait_checks_driver_faults_and_enable_bits():
    for residual, states in [
        (dict(category='driver_fault', joint=3, flags=['collision_status']), [True]*7),
        (None, [True]*6+[False]),
        (dict(category='driver_feedback_stale', joint=1), [True]*7),
    ]:
        d, _, _ = make_driver()
        d._arm.controller_fault.side_effect = [brake_pending(), residual]
        d._arm.get_joint_enable_states.return_value = states
        with mock.patch.object(d._return_cancel, 'wait') as wait:
            assert not d._wait_return_controller_ready()
        wait.assert_not_called()


@pytest.mark.parametrize('outcome', ['timeout', 'cancel', 'scheduler_late'])
def test_enable_brake_wait_is_bounded_and_cancellable(outcome):
    d, _, _ = make_driver()
    clock = [0.]
    d._arm.controller_fault.side_effect = lambda **kw: None if kw.get('allow_disabled') else brake_pending()
    def wait(delay):
        clock[0] += delay
        if outcome == 'cancel':
            d._return_cancel.set()
        elif outcome == 'scheduler_late':
            clock[0] += 1.
            d._arm.controller_fault.side_effect = None
            d._arm.controller_fault.return_value = None
    with mock.patch('esrobo_teleop.robot.nero_driver.time.monotonic', side_effect=lambda: clock[0]), \
            mock.patch.object(d._return_cancel, 'wait', side_effect=wait):
        assert not d._wait_return_controller_ready()
    if outcome != 'cancel':
        assert 'brake_release_timeout' in d.safety_fault_reason
    d._arm.move_j.assert_not_called()
    d._arm.enable.assert_not_called()


def test_runtime_status_six_still_rejects_without_transition_wait():
    d, _, _ = make_driver()
    d._arm.controller_fault.return_value = brake_pending()
    with mock.patch.object(d._return_cancel, 'wait') as wait:
        assert not d._controller_is_safe()
    wait.assert_not_called()


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

@pytest.mark.parametrize('side', ['left', 'right'])
def test_enabled_hold_return_disables_only_after_planned_arrival(side):
    d, feedback, zero = make_driver(side, np.array([.1,-.15,.2,.3,-.1,.1,-.1]))
    d._trajectory.reset(feedback, d._arm.last_feedback_timestamp)
    d._arm.disable.side_effect = lambda: bool(np.max(abs(feedback-zero)) <= np.deg2rad(1.5))
    with mock.patch('esrobo_teleop.robot.nero_driver.time.sleep'):
        result = d.safe_return(preserve_hold=True)
    assert result.returned and result.disabled
    assert not d.return_hold_preserved
    d._arm.disable.assert_called_once()
    d._arm.reset.assert_not_called()


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('failure', ['plan', 'cancel', 'feedback', 'torque'])
def test_refused_plan_retains_only_verified_stationary_hold(side, failure):
    from esrobo_teleop.robot.return_planner import ReturnPlan
    d, feedback, zero = make_driver(side, np.ones(7)*.04)
    d._trajectory.reset(feedback, d._arm.last_feedback_timestamp)
    def refuse(*args):
        if failure == 'cancel': d._return_cancel.set()
        if failure == 'feedback': d.read_joints.side_effect = lambda: None
        if failure == 'torque': d._torque_is_safe.return_value = False
        return ReturnPlan((), 0, 0, 'test blocked route')
    with mock.patch('esrobo_teleop.robot.nero_driver.time.sleep'), mock.patch(
            'esrobo_teleop.robot.staged_return_planner.StagedReturnPlanner.plan', side_effect=refuse):
        result = d.safe_return(preserve_hold=True)
    assert not result.returned
    assert d.return_hold_preserved == (failure == 'plan')
    d._arm.disable.assert_not_called()
    if d.return_hold_preserved:
        assert not d._return_inhibited
        np.testing.assert_allclose(d._trajectory.state.position, feedback)
        np.testing.assert_allclose(d._trajectory.state.velocity, 0)
        assert d.hold_command_trajectory()


def test_preserve_hold_cannot_clear_an_existing_driver_latch():
    d, feedback, _ = make_driver(initial=np.ones(7)*.04)
    d._trajectory.reset(feedback, d._arm.last_feedback_timestamp)
    d._return_inhibited = True
    result = d.safe_return(preserve_hold=True)
    assert not result.returned and not d.return_hold_preserved
    assert d._return_inhibited
    d._arm.disable.assert_not_called()


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('moving', [False, True])
def test_hold_return_brakes_live_state_before_any_full_geometry(side, moving):
    from dataclasses import replace
    d, feedback, zero = make_driver(side, initial=np.ones(7)*.04)
    d._trajectory.reset(feedback, d._arm.last_feedback_timestamp)
    velocity = np.zeros(7)
    if moving:
        velocity[4] = .05
    d._trajectory.state = replace(d._trajectory.state, velocity=velocity)
    d._first_follow_hold = False
    braking_done = []
    original_brake = d._brake_held_trajectory_for_return
    original_check = d._check_physical_path
    def brake(timeout):
        # No invalidation or velocity reset before the first brake proposal.
        np.testing.assert_array_equal(d._trajectory.state.velocity, velocity)
        original_commit = d._trajectory.commit
        def commit(proposal):
            before = d._trajectory.state
            dt = proposal.stamp-before.stamp
            assert np.all(np.abs(proposal.velocity) <= d._trajectory.vmax+1e-8)
            assert np.all(np.abs(proposal.velocity-before.velocity) <= d._trajectory.amax*dt+1e-8)
            assert np.all(np.abs(proposal.position-feedback) <= d._trajectory.lead+1e-8)
            original_commit(proposal)
        with mock.patch.object(d._trajectory, 'reset', wraps=d._trajectory.reset) as reset, \
                mock.patch.object(d._trajectory, 'commit', side_effect=commit):
            result = original_brake(timeout)
            reset.assert_not_called()
        assert result
        np.testing.assert_array_equal(d._trajectory.state.velocity, np.zeros(7))
        braking_done.append(True)
        return result
    def geometry(a, b, **kw):
        assert braking_done, 'offline geometry must not block a live brake'
        return original_check(a, b, **kw)
    d._brake_held_trajectory_for_return = brake
    d._check_physical_path = geometry
    d._arm.disable.side_effect = lambda: bool(np.max(abs(feedback-zero)) <= np.deg2rad(1.5))
    with mock.patch('esrobo_teleop.robot.nero_driver.time.sleep'):
        result = d.safe_return(preserve_hold=True)
    assert result.returned and result.disabled, result
    d._arm.disable.assert_called_once()
    d._arm.reset.assert_not_called()


@pytest.mark.parametrize('failure', ['feedback', 'cancel', 'enable', 'torque'])
def test_hold_return_brake_failure_never_starts_geometry_or_claims_hold(failure):
    d, feedback, _ = make_driver(initial=np.ones(7)*.04)
    d._trajectory.reset(feedback, d._arm.last_feedback_timestamp)
    if failure == 'feedback':
        d.read_control_cycle_joints = mock.Mock(return_value=None)
    elif failure == 'cancel':
        d._return_cancel.set()
    elif failure == 'enable':
        d._arm.get_joint_enable_states.return_value = [True]*6+[False]
    else:
        d._torque_is_safe.return_value = False
    with mock.patch.object(d, '_check_physical_path') as geometry:
        assert not d._brake_held_trajectory_for_return(.1)
        geometry.assert_not_called()
    d._arm.move_j.assert_not_called()
    d._arm.disable.assert_not_called()


@pytest.mark.parametrize('outcome', ['fresh', 'timeout', 'hardware_fault', 'cancel'])
def test_disabled_return_enable_waits_only_for_missing_status_without_writes(outcome):
    d, _, _ = make_driver('left', initial=np.ones(7)*.03)
    d._enabled = False
    d._cfg.controller_status_timeout_s = .02
    d._configure_arm_collision_safety = mock.Mock(return_value=False)
    stale = {'category': 'driver_feedback_stale', 'joint': 1}
    values = [stale, None] if outcome == 'fresh' else [stale]
    if outcome == 'hardware_fault':
        values = [{'category': 'driver_fault', 'joint': 1, 'flags': ['driver_overcurrent']}]
    def status(**_):
        d._arm.enable.assert_not_called()
        d._arm.move_j.assert_not_called()
        return values.pop(0) if values else (None if outcome == 'fresh' else stale)
    d._arm.controller_fault.side_effect = status
    if outcome == 'cancel':
        d._return_cancel.wait = mock.Mock(side_effect=lambda _: d._return_cancel.set())
    # Fresh status reaches configuration; stop there so the test never enables.
    assert not d._enable_for_return()
    assert d._configure_arm_collision_safety.call_count == int(outcome == 'fresh')
    d._arm.enable.assert_not_called()
    d._arm.move_j.assert_not_called()
    if outcome == 'hardware_fault':
        assert 'driver_overcurrent' in d._last_return_enable_failure
