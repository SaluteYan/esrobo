"""No hardware: entry gates, fresh source supervision and activation barrier."""
import copy
import time
from unittest.mock import Mock
import pytest
from esrobo_link.backends import MockBackend
from esrobo_link.gateway import Gateway
from esrobo_link.protocol import validate_target, ProtocolError


class EntryBackend(MockBackend):
    def __init__(self):
        super().__init__('right', False)
        self.contract['entry_protocol'] = 1
        self.approach_entry = Mock(side_effect=self.approach)
        self.activate_entry = Mock()

    def approach(self, target, cancelled, on_approach):
        assert not self.enabled
        on_approach()
        if cancelled():
            raise RuntimeError('cancelled before enable')
        self.enabled = True
        self.q = list(target['arm_urdf_rad'])
        return dict(returned=True, disabled=False, stage='handover_arrived')


def candidate():
    return dict(arm_urdf_rad=[.1]*7, hand_unit=None,
                entry=dict(phase='candidate', token='', source=[0.]*9))


def frozen_candidate(source=None):
    value = candidate()
    value['entry']['source'] = list(source if source is not None else [0.]*9)
    value['entry']['reference_source'] = [0.]*9
    return value


def test_calibrated_goal_is_frozen_and_raw_pose_must_match_before_handover():
    backend = EntryBackend()
    backend.contract['entry_protocol'] = 2
    g = Gateway(backend, b'test-frozen-entry-only'*4, port=0)
    try:
        g.mode = 'CALIBRATING'
        g.gate.target, g.gate.deadline = frozen_candidate(), time.monotonic()+10
        g.tick()
        assert g.entry_target['entry']['reference_source'] == [0.]*9
        assert not backend.enabled
        g.gate.target = frozen_candidate([.03]+[0.]*8)
        g.tick()
        assert g.entry_candidate_since is None
        assert g.entry_target['entry']['source'] == [0.]*9
        with pytest.raises(RuntimeError, match='稳定'):
            g.request('g')
        g.gate.target = frozen_candidate()
        g.tick()
        assert g.entry_candidate_since is not None
        g.entry_candidate_since -= .6
        g.request('g')
        g.tick()
        assert g.mode == 'WAITING_FOR_ACTIVE_TARGET'
    finally:
        g.socket.close()


def test_protocol_two_requires_frozen_calibration_source():
    backend = EntryBackend()
    backend.contract['entry_protocol'] = 2
    target = dict(candidate(), side='right', contract_id=backend.contract['id'])
    with pytest.raises(ProtocolError, match='calibrated entry source'):
        validate_target(target, backend.contract)


@pytest.mark.parametrize('protocol', [3, 4, 5, 6])
def test_protocol_three_accepts_human_arm_motion_but_keeps_calibration_goal(protocol):
    backend = EntryBackend()
    backend.contract['entry_protocol'] = protocol
    g = Gateway(backend, b'test-moving-human-entry'*4, port=0)
    try:
        g.mode = 'CALIBRATING'
        g.gate.target, g.gate.deadline = frozen_candidate(), time.monotonic()+10
        g.tick()
        frozen = copy.deepcopy(g.entry_target)
        # The operator may move after calibration. The planned arm target must
        # remain the original accepted calibration target.
        g.gate.target = frozen_candidate([.3]*9)
        g.tick()
        assert g.entry_target == frozen
        assert not backend.enabled
        assert g.entry_candidate_since is not None
        g.entry_candidate_since -= .6
        def plan(target, cancelled, callback):
            assert target == frozen
            g.gate.target = frozen_candidate([.4]*9)
            g.watchdog()
            assert not cancelled()
            callback()
            backend.enabled = True
            backend.q = list(target['arm_urdf_rad'])
            return dict(returned=True, disabled=False, stage='handover_arrived')
        backend.approach_entry.side_effect = plan
        g.request('g')
        g.tick()
        assert g.mode == 'WAITING_FOR_ACTIVE_TARGET'
        g.gate.target = frozen_candidate([.5]*9)
        g.gate.target['entry'].update(phase='ready', token=g.entry_token)
        g.tick()
        assert g.mode == 'ACTIVE'
        backend.activate_entry.assert_called_once()
    finally:
        g.socket.close()


@pytest.mark.parametrize('failure', ['stale', 'changed_reference', 'changed_goal'])
def test_protocol_three_still_rejects_lost_input_or_changed_goal(failure):
    backend = EntryBackend()
    backend.contract['entry_protocol'] = 3
    g = Gateway(backend, b'test-frozen-goal-guard'*4, port=0)
    try:
        g.mode = 'CHECKING_ENTRY'
        g.gate.target, g.gate.deadline = frozen_candidate(), time.monotonic()+10
        g.entry_target = copy.deepcopy(g.gate.target)
        if failure == 'stale':
            g.gate.deadline = time.monotonic()-1
        elif failure == 'changed_reference':
            g.gate.target['entry']['reference_source'][0] = .01
        else:
            g.gate.target['arm_urdf_rad'][0] += .1
        g.watchdog()
        assert g.mode == 'FAULT' and g.cancelled.is_set()
    finally:
        g.socket.close()


def test_protocol_three_requires_frozen_reference_field():
    backend = EntryBackend()
    backend.contract['entry_protocol'] = 3
    target = dict(candidate(), side='right', contract_id=backend.contract['id'])
    with pytest.raises(ProtocolError, match='calibrated entry source'):
        validate_target(target, backend.contract)


@pytest.fixture
def gateway():
    g = Gateway(EntryBackend(), b'test-entry-only'*4, port=0)
    g.mode = 'CALIBRATING'
    g.gate.target, g.gate.deadline = candidate(), time.monotonic()+10
    yield g
    g.socket.close()


def approach(g):
    g.tick()
    assert g.mode == 'CALIBRATING'
    assert not g.backend.enabled
    g.entry_candidate_since -= .6
    g.request("g")
    g.tick()
    assert g.mode == 'WAITING_FOR_ACTIVE_TARGET'
    assert g.backend.enabled


def test_stable_full_target_planned_then_explicit_fresh_ready_required(gateway):
    g = gateway
    approach(g)
    g.tick()
    assert g.mode == 'WAITING_FOR_ACTIVE_TARGET'
    g.backend.activate_entry.assert_not_called()
    g.gate.target['entry'].update(phase='ready', token='wrong')
    g.tick()
    assert g.mode == 'WAITING_FOR_ACTIVE_TARGET'
    g.gate.target['entry']['token'] = g.entry_token
    g.tick()
    assert g.mode == 'ACTIVE'
    g.backend.activate_entry.assert_called_once()
    assert g.backend.approach_entry.call_args.args[0]['arm_urdf_rad'] == [.1]*7


def test_unsafe_goal_never_enables_or_falls_back_to_live(gateway):
    g = gateway
    g.backend.approach_entry.side_effect = RuntimeError('handover goal not certified safe')
    g.tick(); g.entry_candidate_since -= .6; g.request("g"); g.tick()
    assert g.mode == 'FAULT'
    assert not g.backend.enabled
    assert 'not certified safe' in g.reason
    g.backend.activate_entry.assert_not_called()


@pytest.mark.parametrize('mode', ['CHECKING_ENTRY', 'APPROACHING', 'WAITING_FOR_ACTIVE_TARGET'])
@pytest.mark.parametrize('failure', ['expired', 'human_moved', 'joint_changed'])
def test_no_cached_target_can_hide_input_loss_or_human_motion(gateway, mode, failure):
    g = gateway
    g.entry_target = copy.deepcopy(g.gate.target)
    g.mode = mode
    if failure == 'expired':
        g.gate.deadline = time.monotonic()-1
    elif failure == 'human_moved':
        g.gate.target['entry']['source'][3] += .03
    else:
        g.gate.target['arm_urdf_rad'][4] += .1
    g.watchdog()
    assert g.mode == 'FAULT' and g.cancelled.is_set()
    if mode != 'CHECKING_ENTRY':
        assert g.stop_pending


def test_arrival_has_bounded_wait_even_with_fresh_candidate(gateway):
    approach(gateway)
    gateway.entry_ready_deadline = time.monotonic()-1
    gateway.watchdog()
    assert gateway.mode == 'FAULT'


def test_source_motion_while_planning_cancels_before_enable(gateway):
    g = gateway
    def plan(target, cancelled, callback):
        g.gate.target['entry']['source'][0] = .1
        callback()
        pytest.fail('must not get past approach callback')
    g.backend.approach_entry.side_effect = plan
    g.tick(); g.entry_candidate_since -= .6; g.request("g"); g.tick()
    assert g.mode == 'FAULT' and not g.backend.enabled


def test_old_client_missing_metadata_rejected(gateway):
    contract = gateway.backend.contract
    message = dict(candidate(), side='right', contract_id=contract['id'])
    assert validate_target(message, contract)['entry']['phase'] == 'candidate'
    del message['entry']
    with pytest.raises(ProtocolError, match='metadata'):
        validate_target(message, contract)


def test_handover_requires_explicit_operator_selection(gateway):
    g=gateway
    g.tick();g.entry_candidate_since-=1.;g.tick()
    assert g.mode=='CALIBRATING' and not g.backend.enabled
    g.backend.approach_entry.assert_not_called()


def test_ready_prompt_clears_when_human_target_moves(gateway):
    g = gateway
    g.tick()
    g.entry_candidate_since -= 1.
    g.tick()
    assert '已稳定' in g.reason
    g.gate.target['entry']['source'][3] += .02
    g.tick()
    assert '追踪波动' in g.reason
    assert g.entry_candidate_since is not None
    with pytest.raises(RuntimeError, match='稳定至少 0.5 秒'):
        g.request('g')
    assert not g.backend.enabled


def test_ready_sent_before_arrival_cannot_activate(gateway):
    g=gateway
    normal=g.backend.approach
    def early(target,cancelled,callback):
        g.gate.target['entry'].update(phase='ready',token=g.entry_token)
        return normal(target,cancelled,callback)
    g.backend.approach_entry.side_effect=early
    approach(g)
    assert g.entry_token != g.gate.target['entry']['token']
    g.tick()
    assert g.mode=='WAITING_FOR_ACTIVE_TARGET'
    g.backend.activate_entry.assert_not_called()


def test_source_gap_discards_old_stability_before_operator_selection(gateway):
    g=gateway
    g.tick();g.entry_candidate_since-=1.
    g.gate.deadline=time.monotonic()-1
    g.watchdog()
    g.gate.deadline=time.monotonic()+10
    g.gate.target=candidate()
    with pytest.raises(RuntimeError,match='稳定'):
        g.request('g')
    g.tick()
    assert g.entry_candidate_since is not None
    g.backend.approach_entry.assert_not_called()


def test_dual_contract_does_not_advertise_single_arm_handover():
    from esrobo_link.backends import DualBackend
    members={side:MockBackend(side,True) for side in ('left','right')}
    for member in members.values():
        member.contract['entry_protocol']=1
    dual=DualBackend(members)
    assert not dual.supports_prepare
    assert all('entry_protocol' not in c for c in dual.contract['sides'].values())


def test_robot_rejects_missing_prepared_proof_without_new_geometry():
    import numpy as np
    from esrobo_link.backends import HardwareBackend
    backend=HardwareBackend.__new__(HardwareBackend)
    backend.np,backend.side=np,'right'
    backend.monitor_hold=Mock()
    backend.snapshot=Mock(return_value=dict(arm_urdf_rad=[0.]*7))
    backend.arm,backend.hand=Mock(),Mock()
    backend.cfg=Mock()
    backend.cfg.hand.max_stale_time_s=.1
    backend.hand.feedback_snapshot.return_value=dict(position_unit=[100]*10,age_s=.01)
    backend.arm._mapping.return_value=(np.ones(7),np.zeros(7))
    backend.arm.verify_tracking_activation.return_value=False
    with pytest.raises(RuntimeError,match='feedback/coverage'):
        backend.activate_entry(dict(arm_urdf_rad=[.01]*7,hand_unit=[100]*10),lambda:False)
    backend.arm._check_physical_path.assert_not_called()
    backend.hand.set_enabled.assert_called_once_with(False)


def test_arm_preparation_holds_hand_without_sending_open_command():
    from esrobo_link.backends import HardwareBackend
    from esrobo_teleop.robot.return_planner import ReturnResult
    backend = HardwareBackend.__new__(HardwareBackend)
    backend.side, backend.hand_only = 'right', False
    backend.hand, backend.arm, backend.cfg = Mock(), Mock(), Mock()
    backend.cfg.hand.max_stale_time_s = .1
    backend.hand.feedback_snapshot.return_value = dict(
        position_unit=[142, 127, 254, 254, 254, 254, 110, 109, 120, 112], age_s=.01)
    backend.arm.safe_return.return_value = ReturnResult(True, True, 'disabling')
    result = backend.prepare_session(lambda: False)
    assert result['returned'] and result['disabled'] and result['hands_stationary']
    assert not result['hands_open']
    backend.hand.align_and_verify_open_pose.assert_not_called()
    assert backend.arm.safe_return.call_args.kwargs['arm_only_collision'] is True
    assert backend.arm.safe_return.call_args.kwargs['recognize_inherited_stop'] is False
    held_check = backend.arm.safe_return.call_args.kwargs['stationary_hand_check']
    assert held_check()
    backend.hand.feedback_snapshot.return_value['position_unit'][0] = 149
    assert not held_check()


def test_entry_refuses_following_when_enabled_hand_envelope_is_unsafe():
    import numpy as np
    from esrobo_link.backends import HardwareBackend
    backend = HardwareBackend.__new__(HardwareBackend)
    backend.np, backend.side = np, 'right'
    backend.monitor_hold = Mock()
    backend.cfg = Mock()
    backend.cfg.hand.max_stale_time_s = .1
    backend.snapshot = Mock(return_value=dict(arm_urdf_rad=[0.]*7))
    backend.arm, backend.hand = Mock(), Mock()
    backend.hand.feedback_snapshot.return_value = dict(position_unit=[142]*10, age_s=.01)
    backend.arm._mapping.return_value = (np.ones(7), np.zeros(7))
    backend.arm._check_physical_path.return_value = True
    backend.arm.prepare_tracking_activation.return_value = False
    with pytest.raises(RuntimeError, match='hand motion envelope'):
        backend._prepare_entry_activation(dict(arm_urdf_rad=[0.]*7, hand_unit=[142]*10), lambda: False)
    assert [c.args[0] for c in backend.hand.set_enabled.call_args_list] == [True, False]


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('with_hand', [False, True])
def test_entry_excludes_fingers_and_holds_wrist_at_backend_boundary(side, with_hand):
    import numpy as np
    from esrobo_link.backends import HardwareBackend
    from esrobo_teleop.robot.return_planner import ReturnResult
    backend = HardwareBackend.__new__(HardwareBackend)
    backend.np, backend.side = np, side
    backend.hand, backend.arm, backend.cfg = (Mock() if with_hand else None), Mock(), Mock()
    backend.cfg.hand.max_stale_time_s = .1
    actual = [0.]*5+[.17,-.13]
    held = [128]*10
    backend.snapshot = Mock(return_value=dict(arm_urdf_rad=actual, driver_fault=None,
                                               hand=dict(position_unit=held)))
    backend.preflight_enable = Mock()
    if with_hand:
        backend.hand.feedback_snapshot.return_value = dict(position_unit=held, age_s=.01)
    backend.arm.safe_return.return_value = ReturnResult(True,False,'handover_arrived')
    target = dict(arm_urdf_rad=[.1]*5+actual[5:], hand_unit=held)
    backend._prepare_entry_activation = Mock()
    backend.approach_entry(target, lambda: False, lambda: None)
    backend.arm.initialize_command_trajectory.assert_called_once_with(after_handover=True)
    backend._prepare_entry_activation.assert_called_once()
    backend.arm._check_physical_path.assert_not_called()  # no pre-planning finger point check
    assert backend.arm.safe_return.call_args.kwargs['arm_only_collision'] is True
    assert backend.arm.safe_return.call_args.kwargs['stationary_hand_check']()
    backend.arm.safe_return.reset_mock()
    target['arm_urdf_rad'][5] += .01
    with pytest.raises(RuntimeError, match='hold measured J6/J7'):
        backend.approach_entry(target, lambda: False, lambda: None)
    backend.arm.safe_return.assert_not_called()


def test_hardware_entry_preflight_accepts_held_hand_without_open_zero():
    import numpy as np
    from esrobo_link.backends import HardwareBackend
    backend = HardwareBackend.__new__(HardwareBackend)
    backend.np, backend.side = np, 'right'
    backend.hand, backend.arm = Mock(), Mock()
    backend.snapshot = Mock(return_value=dict(
        enable_states=[False] * 7, controller_fault=None, arm_urdf_rad=[0.] * 7))
    backend.hand.feedback_snapshot.return_value = dict(position_unit=[142] * 10)
    backend.hand.feedback_ready.return_value = True
    backend.hand.open_feedback_calibrated.return_value = False
    target = dict(arm_urdf_rad=[0.] * 7, hand_unit=[142] * 10)

    backend.preflight_enable(target, entry=True)
    backend.hand.open_pose_status.assert_not_called()
    with pytest.raises(RuntimeError, match='natural-open feedback zero'):
        backend.preflight_enable(target)
    with pytest.raises(RuntimeError, match='held feedback'):
        backend.preflight_enable(dict(target, hand_unit=[150] * 10), entry=True)


def test_slow_hardware_activation_proof_finishes_before_ready_deadline(gateway):
    import numpy as np
    from esrobo_link.backends import HardwareBackend
    from esrobo_teleop.robot.return_planner import ReturnResult
    g = gateway
    hardware = HardwareBackend.__new__(HardwareBackend)
    hardware.side, hardware.np, hardware.hand = 'right', np, None
    hardware.arm = Mock()
    hardware.cfg = Mock()
    hardware.snapshot = Mock(return_value=dict(arm_urdf_rad=[0.]*7, driver_fault=None))
    hardware.preflight_enable = Mock()
    hardware.monitor_hold = Mock()
    hardware.arm._mapping.return_value = (np.ones(7), np.zeros(7))
    def returned(**kwargs):
        kwargs['on_approach']()
        g.backend.enabled = True
        q = kwargs['handover_goal']
        hardware.snapshot.return_value['arm_urdf_rad'] = q
        g.backend.q = q
        return ReturnResult(True, False, 'handover_arrived')
    hardware.arm.safe_return.side_effect = returned
    def geometry(*_):
        assert g.mode == 'APPROACHING'
        assert g.entry_ready_deadline is None
        time.sleep(.15)  # Longer than the laptop's 100 ms feedback freshness.
        assert g.mode == 'APPROACHING'
        return True
    hardware.arm.prepare_tracking_activation.side_effect = geometry
    hardware.arm.verify_tracking_activation.return_value = True
    hardware.arm._check_physical_path.side_effect = AssertionError('redundant geometry')
    g.backend.approach_entry.side_effect = hardware.approach_entry
    g.backend.activate_entry.side_effect = hardware.activate_entry
    g.gate.target['arm_urdf_rad'][5:] = [0., 0.]
    approach(g)
    assert g.entry_ready_deadline-time.monotonic() > 1.9
    assert time.monotonic()-g.feedback_time < .1
    hardware.arm.prepare_tracking_activation.assert_called_once()
    g.gate.target['entry'].update(phase='ready', token=g.entry_token)
    g.tick()
    assert g.mode == 'ACTIVE'
    hardware.arm.prepare_tracking_activation.assert_called_once()
    hardware.arm.verify_tracking_activation.assert_called_once()
    hardware.arm._check_physical_path.assert_not_called()
