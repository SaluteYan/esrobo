"""Explicit checked return includes a certified, verified hand-open phase."""
from types import SimpleNamespace
from unittest.mock import Mock
import numpy as np
import pytest
from esrobo_link.backends import HardwareBackend
from esrobo_teleop.robot.return_planner import ReturnResult
from esrobo_teleop.robot.hand_geometry import HandGeometryState


def backend(side='left'):
    b = HardwareBackend.__new__(HardwareBackend)
    b.side, b.np = side, np
    b._full_slice = slice(0, 7) if side == 'left' else slice(7, 14)
    b.cfg = SimpleNamespace(robot=SimpleNamespace(command_trajectory_max_dt_s=.1,
                                                  full_arm_return_tolerance_deg=1.5))
    b.arm, b.hand = Mock(), Mock()
    b.arm.safe_return.return_value = ReturnResult(True, True, 'disabling')
    b.arm.recover_disabled_controller.return_value = dict(disabled=True, reset_sent=False, verified_samples=3)
    b.snapshot = Mock(return_value=dict(enable_states=[False]*7, arm_urdf_rad=[0.]*7,
                                       controller_fault=None, driver_fault=None, arm_feedback_age_s=.01))
    envelope = HandGeometryState(1., {'finger': (-1., 1.)}, ('finger',), True)
    b.hand.open_return_geometry_state.return_value = envelope
    b.hand.open_pose_status.return_value = (False, {})
    b.hand.geometry_state.return_value = envelope
    b.arm._collision_guard.check_hand_return.return_value = True
    def open_hand(cancelled, **kw):
        ok = kw['safety_check']()
        b.hand.open_pose_status.return_value = (ok, {})
        return ok, {}
    b.hand.align_and_verify_open_pose.side_effect = open_hand
    return b


@pytest.mark.parametrize('side', ['left', 'right'])
def test_explicit_return_verifies_hand_before_seven_joint_return(side):
    b = backend(side)
    events = []
    b.arm.safe_return.side_effect = lambda **kw: events.append('arm_return') or ReturnResult(True, True, 'disabling')
    b.arm.recover_disabled_controller.side_effect = lambda *args: events.append('controller_recovery') or dict(disabled=True)
    b.arm._collision_guard.check_hand_return.side_effect = lambda *args: events.append('geometry') or True
    def open_hand(cancelled, **kw):
        assert kw['force'] and kw['safety_check']()
        events.append('open')
        b.hand.open_pose_status.return_value = (True, {})
        return True, {}
    b.hand.align_and_verify_open_pose.side_effect = open_hand
    r = b.return_zero(lambda: False)
    assert r['returned'] and r['disabled'] and r['hands_open'] and r['arm_returned']
    assert events == ['controller_recovery', 'geometry', 'open', 'arm_return', 'controller_recovery']
    b.hand.set_enabled.assert_called_with(False)
    b.arm.enable.assert_not_called()


@pytest.mark.parametrize('returned,disabled', [(False, True), (True, False), (False, False)])
def test_arm_failure_preserves_completed_hand_phase(returned, disabled):
    b = backend()
    b.arm.safe_return.return_value = ReturnResult(returned, disabled, 'planning', 'failure')
    r = b.return_zero(lambda: False)
    assert r['hands_open'] and r['arm_returned'] == returned
    assert r['disabled'] == disabled
    b.hand.align_and_verify_open_pose.assert_called_once()
    b.arm.recover_disabled_controller.assert_called_once()


def test_automatic_preparation_keeps_hand_stationary():
    b = backend()
    b.return_zero(lambda: False, explicit_recovery=False)
    b.hand.align_and_verify_open_pose.assert_not_called()
    b.arm.recover_disabled_controller.assert_not_called()


@pytest.mark.parametrize('disabled', [True, False])
def test_failed_controller_recovery_blocks_hand_and_preserves_actual_disable(disabled):
    b = backend()
    b.arm.recover_disabled_controller.side_effect = RuntimeError('recovery rejected')
    b.arm._arm.get_joint_enable_states.return_value = [not disabled]*7
    r = b.return_zero(lambda: False)
    assert not r['returned'] and not r['arm_returned'] and r['disabled'] == disabled
    assert r['stage'] == 'controller_recovery_before_hand'
    b.arm.safe_return.assert_not_called()
    b.hand.align_and_verify_open_pose.assert_not_called()


@pytest.mark.parametrize('stage', ['hand_open_before_arm', 'hand_open_verify'])
@pytest.mark.parametrize('failure', ['none', 'enabled', 'stale', 'missing', 'controller', 'driver', 'explicit_stop'])
def test_hand_open_refusal_does_not_restop_only_verified_disabled_arm(failure, stage):
    from esrobo_link.backends import MockBackend
    from esrobo_link.gateway import Gateway
    b = MockBackend('left', True)
    g = Gateway(b, b'test-disabled-return'*4, port=0)
    state = b.snapshot()
    state.update(enable_states=[False]*7, controller_fault=None, driver_fault=None,
                 arm_feedback_age_s=.01)
    if failure == 'enabled': state['enable_states'] = [True]*7
    if failure == 'stale': state['arm_feedback_age_s'] = 1.
    if failure == 'missing': state['arm_urdf_rad'] = None
    if failure == 'controller': state['controller_fault'] = {'arm_status': 1}
    if failure == 'driver': state['driver_fault'] = 'fault'
    b.snapshot = Mock(return_value=state)
    def returned(cancelled):
        if failure == 'explicit_stop': g.request('s')
        return dict(returned=False, disabled=True, arm_returned=stage == 'hand_open_verify', stage=stage, reason='geometry refused')
    b.return_zero = returned
    try:
        g.mode = 'FAULT'
        g.request('z')
        g.tick()
        assert g.mode == 'FAULT'
        assert g.stop_pending == (failure != 'none')
        g.tick()
        assert b.stops == (0 if failure == 'none' else 1)
    finally:
        g.socket.close()


def test_already_open_measured_hand_needs_no_motion():
    b = backend()
    b.hand.open_pose_status.return_value = (True, {})
    r = b.return_zero(lambda: False)
    assert r['returned'] and r['disabled'] and r['hands_open']
    b.hand.align_and_verify_open_pose.assert_not_called()
    b.arm._collision_guard.check_hand_return.assert_not_called()
    b.hand.set_enabled.assert_called_with(False)


@pytest.mark.parametrize('fault', ['collision', 'stale_arm', 'enabled', 'nonfinite', 'hand_invalid', 'cancel'])
def test_preflight_refusal_never_sends_hand(fault):
    b = backend()
    s = b.snapshot.return_value
    if fault == 'collision': b.arm._collision_guard.check_hand_return.return_value = False
    if fault == 'stale_arm': s['arm_feedback_age_s'] = 1.
    if fault == 'enabled': s['enable_states'][0] = True
    if fault == 'nonfinite': s['arm_urdf_rad'][0] = float('nan')
    if fault == 'hand_invalid': b.hand.open_return_geometry_state.return_value = HandGeometryState(None, {}, (), False, 'stale')
    r = b.return_zero(lambda: fault == 'cancel')
    assert not r['returned'] and not r['hands_open']
    if fault == 'enabled': assert not r['disabled']
    b.hand.align_and_verify_open_pose.assert_not_called()
    b.arm.safe_return.assert_not_called()


@pytest.mark.parametrize('fault', ['arm_drift', 'enable_change', 'hand_escape', 'stale_hand', 'no_arrival'])
def test_hand_phase_failure_cannot_report_idle_success(fault):
    b = backend()
    def open_hand(cancelled, **kw):
        if fault == 'arm_drift': b.snapshot.return_value['arm_urdf_rad'][0] = .02
        if fault == 'enable_change': b.snapshot.return_value['enable_states'] = [True]*7
        if fault == 'hand_escape': b.hand.geometry_state.return_value = HandGeometryState(1., {'finger': (-2., 1.)}, ('finger',), True)
        if fault == 'stale_hand': b.hand.geometry_state.return_value = HandGeometryState(None, {}, (), False, 'stale')
        if fault == 'no_arrival': return False, {}
        assert not kw['safety_check']()
        return False, {}
    b.hand.align_and_verify_open_pose.side_effect = open_hand
    b.hand.describe_open_pose_errors.return_value = 'not reached'
    r = b.return_zero(lambda: False)
    assert not r['returned'] and not r['hands_open']
    if fault == 'enable_change': assert not r['disabled']
    b.hand.set_enabled.assert_called_with(False)
    b.arm.safe_return.assert_not_called()


@pytest.mark.parametrize('side', ['left', 'right'])
def test_hand_is_certified_at_actual_nonzero_arm_pose(side):
    b = backend(side)
    q = [.2, -.3, .1, 0., -.1, .4, -.35]
    b.snapshot.return_value['arm_urdf_rad'] = q
    r = b.return_zero(lambda: False)
    assert r['returned'] and r['hands_open']
    lo, hi, _ = b.arm._collision_guard.check_hand_return.call_args.args
    np.testing.assert_allclose((lo+hi)[b._full_slice]/2, q)
    np.testing.assert_allclose((hi-lo)[b._full_slice], np.deg2rad(.4))


def test_busy_return_cannot_start_hand_or_release_other_request():
    b = backend()
    b.arm.prepare_return.return_value = None
    r = b.return_zero(lambda: False)
    assert r['stage'] == 'busy'
    b.hand.align_and_verify_open_pose.assert_not_called()
    b.arm.safe_return.assert_not_called()
    b.arm.release_return_request.assert_not_called()


def test_hand_refusal_releases_reservation_for_retry():
    b = backend()
    b.arm.prepare_return.return_value = 7
    b.arm._collision_guard.check_hand_return.return_value = False
    r = b.return_zero(lambda: False)
    assert not r['returned'] and not r['arm_returned']
    b.arm.safe_return.assert_not_called()
    b.arm.release_return_request.assert_called_once_with(7)
    b.arm._collision_guard.check_hand_return.return_value = True
    assert b.return_zero(lambda: False)['returned']


def test_cancellation_after_hand_arrival_never_starts_arm():
    b = backend()
    cancelled = [False]
    def open_hand(cancel, **kw):
        cancelled[0] = True
        return True, {}
    b.hand.align_and_verify_open_pose.side_effect = open_hand
    assert not b.return_zero(lambda: cancelled[0])['returned']
    b.arm.safe_return.assert_not_called()
    b.arm.release_return_request.assert_called_once()


def test_final_hand_feedback_loss_refuses_idle_without_reopening():
    b = backend()
    def arm_return(**kw):
        b.hand.open_pose_status.return_value = False, {}
        return ReturnResult(True, True, 'disabling')
    b.arm.safe_return.side_effect = arm_return
    b.hand.describe_open_pose_errors.return_value = 'stale'
    r = b.return_zero(lambda: False)
    assert not r['returned'] and r['arm_returned'] and r['disabled']
    assert not r['hands_open'] and r['stage'] == 'hand_open_verify'
    b.hand.align_and_verify_open_pose.assert_called_once()


def test_post_arm_controller_failure_does_not_erase_hand_result():
    b = backend()
    b.arm.recover_disabled_controller.side_effect = [dict(disabled=True), RuntimeError('fault')]
    b.arm._arm.get_joint_enable_states.return_value = [False]*7
    r = b.return_zero(lambda: False)
    assert not r['returned'] and r['arm_returned'] and r['hands_open']
    assert r['stage'] == 'controller_recovery'


def test_arm_only_return_keeps_original_sequence():
    b = backend()
    b.hand = None
    events = []
    b.arm.safe_return.side_effect = lambda **kw: events.append('arm') or ReturnResult(True, True, 'disabling')
    b.arm.recover_disabled_controller.side_effect = lambda *args: events.append('recover') or dict(disabled=True)
    assert b.return_zero(lambda: False)['returned']
    assert events == ['arm', 'recover']
