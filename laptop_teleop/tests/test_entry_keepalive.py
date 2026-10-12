"""Fixed-plan liveness is independent of sensor freshness; no hardware."""
import copy
from dataclasses import replace
from types import SimpleNamespace
import threading
import time
from unittest.mock import Mock

import pytest

from esrobo_link.backends import MockBackend
from esrobo_link.gateway import Gateway
from esrobo_link.protocol import ProtocolError, canonical, pack, unpack
from esrobo_laptop.inputs import InputUnavailable, SourceExpired
from esrobo_laptop.pipeline import Pipeline
from test_entry_pipeline import controller_fixture


def fixed_controller(mode):
    c, p, client, state = controller_fixture()
    state['contract']['entry_keepalive_protocol'] = 1
    c.contract = canonical(state['contract'])
    c.entry_target = {'right': dict(arm_urdf_rad=[.2]*7, hand_unit=None)}
    c.entry_reference_source = [0.]*9
    state.update(mode=mode, entry_plan_token='p'*32, entry_token='arrival-token')
    state['feedback']['arm_urdf_rad'] = [.2]*7
    return c, p, client, state


@pytest.mark.parametrize('mode', ['CHECKING_ENTRY', 'APPROACHING'])
@pytest.mark.parametrize('age', [.12, .3, 1., 60.])
def test_fixed_plan_never_reads_or_restamps_expired_pico(mode, age):
    c, p, client, state = fixed_controller(mode)
    p.body.stamps = {'right_arm': time.monotonic()-age}
    original = dict(p.body.stamps)
    p.body.ticket.side_effect = SourceExpired('expired')
    assert c.step(state) is None
    client.send_entry_keepalive.assert_called_once_with('p'*32)
    client.send_target.assert_not_called()
    p.body.ticket.assert_not_called()
    p.capture.assert_not_called()
    p.solve.assert_not_called()
    p.check_ticket.assert_not_called()
    p.body.consume.assert_not_called()
    assert p.body.stamps == original
    assert c.computed == 0 and c.entry_keepalives_sent == 1


@pytest.mark.parametrize('failure', ['expired', 'duplicate', 'missing', 'compute_expired'])
def test_arrival_waits_without_fabricating_ready(failure):
    c, p, client, state = fixed_controller('WAITING_FOR_ACTIVE_TARGET')
    if failure == 'duplicate':
        p.capture.return_value = None
    elif failure == 'compute_expired':
        p.check_ticket.side_effect = InputUnavailable('input expired during computation')
    else:
        p.capture.side_effect = SourceExpired('expired') if failure == 'expired' else InputUnavailable('missing')
    assert c.step(state) is None
    client.send_entry_keepalive.assert_called_once()
    client.send_target.assert_not_called()
    p.body.consume.assert_not_called()


def test_arrival_requires_feedback_and_fresh_input_then_uses_existing_active_logic():
    c, p, client, state = fixed_controller('WAITING_FOR_ACTIVE_TARGET')
    c.step(state)
    assert client.send_target.call_args.kwargs['entry']['phase'] == 'ready'
    p.body.consume.assert_called_once()
    state['mode'] = 'ACTIVE'
    c.step(state)
    p.reset_command_start.assert_called_once()
    assert client.send_entry_keepalive.call_count == 1
    assert client.send_target.call_args.kwargs['entry']['phase'] == 'active'
    p.body.ticket.side_effect = SourceExpired('expired')
    with pytest.raises(SourceExpired):
        c.step(state)


def test_arrival_does_not_swallow_robot_feedback_errors():
    c, p, client, state = fixed_controller('WAITING_FOR_ACTIVE_TARGET')
    state['feedback']['arm_feedback_age_s'] = 1.
    with pytest.raises(InputUnavailable, match='feedback stale'):
        c.step(state)
    client.send_entry_keepalive.assert_not_called()


@pytest.mark.parametrize('with_hand', [False, True])
@pytest.mark.parametrize('missing', ['duplicate', 'expired', 'expired_after_solve'])
def test_first_active_input_gap_holds_arrived_target_then_requires_three_fresh_frames(
        monkeypatch, with_hand, missing):
    clock = [100.]
    monkeypatch.setattr(time, 'monotonic', lambda: clock[0])
    c, p, client, state = fixed_controller('WAITING_FOR_ACTIVE_TARGET')
    c.settings = replace(c.settings, with_hand=with_hand)
    if with_hand:
        c.entry_target['right']['hand_unit'] = [128]*10
        state['feedback']['hand'] = dict(position_unit=[128]*10, age_s=0.)
        p.map_hand_target.return_value = [128]*10
        p.cfg = SimpleNamespace(robot=p.cfg.robot, hand=SimpleNamespace(
            max_velocity=400, max_step=18, publish_hz=50, enabled_physical_joints=list(range(10))))
    frozen = copy.deepcopy(c.entry_target)
    p.solve.return_value = copy.deepcopy(frozen)
    c.step(state)
    assert c.last_active_target is None  # ready alone cannot authorize ACTIVE hold
    assert not c.hold_missing_input(state, 'missing while awaiting activation')
    client.send_hold.assert_not_called()
    state['mode'] = 'ACTIVE'
    state['feedback']['enable_states'] = [True]*7
    clock[0] += .02  # inside the usual 40 ms gap: first ACTIVE still needs hold
    client.received_at = clock[0]
    original_ticket = copy.deepcopy(p.body.ticket.return_value)
    if missing == 'duplicate':
        p.body.ticket.return_value = None
        assert c.step(state) is None
    else:
        failing = p.body.ticket if missing == 'expired' else p.check_ticket
        failing.side_effect = (SourceExpired('PICO input expired: 112 ms') if missing == 'expired'
                               else lambda ticket: Pipeline.check_ticket(
                                   c, {'right_arm': clock[0]-.112}))
        with pytest.raises(SourceExpired):
            c.step(state)
        failing.side_effect = None
    assert c.hold_missing_input(state, 'waiting for fresh PICO frames')
    assert c.last_active_target == frozen
    assert c.last_active_target_at == 100.
    assert c.input_hold_started_at == pytest.approx(100.02)
    assert client.send_target.call_count == 1  # only the checked ready target
    assert client.send_entry_keepalive.call_count == 1  # none in ACTIVE
    assert client.send_hold.call_count == 1
    p.body.ticket.return_value = original_ticket
    for index in range(3):
        clock[0] += .02
        client.received_at = clock[0]
        result = c.step(state)
        assert (result is not None) == (index == 2)
    assert not c.entry_active_pending
    assert c.input_hold_started_at is None
    p.reset_command_start.assert_called_once()
    assert client.send_target.call_args.kwargs['entry']['phase'] == 'active'


def test_first_active_hold_keeps_timeout_and_robot_feedback_checks(monkeypatch):
    clock = [100.]
    monkeypatch.setattr(time, 'monotonic', lambda: clock[0])
    c, p, client, state = fixed_controller('WAITING_FOR_ACTIVE_TARGET')
    c.step(state)
    state['mode'] = 'ACTIVE'
    state['feedback']['enable_states'] = [True]*7
    p.body.ticket.return_value = None
    assert c.step(state) is None
    assert c.hold_missing_input(state, 'no first ACTIVE frame')
    state['feedback']['arm_feedback_age_s'] = 1.
    with pytest.raises(InputUnavailable, match='feedback stale'):
        c.hold_missing_input(state, 'no first ACTIVE frame')
    state['feedback']['arm_feedback_age_s'] = 0.
    clock[0] += .501
    client.received_at = clock[0]
    with pytest.raises(RuntimeError, match='hold exceeded 500 ms'):
        c.hold_missing_input(state, 'no first ACTIVE frame')
    assert client.send_target.call_count == 1


def test_mismatched_arrival_token_does_not_establish_active_hold():
    c, p, client, state = fixed_controller('WAITING_FOR_ACTIVE_TARGET')
    c.step(state)
    state.update(mode='ACTIVE', entry_token='different-arrival')
    p.body.ticket.return_value = None
    assert c.step(state) is None
    assert not c.hold_missing_input(state, 'missing')
    assert c.last_active_target is None
    client.send_hold.assert_not_called()


@pytest.fixture
def fixed_gateway(monkeypatch):
    clock = [100.]
    monkeypatch.setattr(time, 'monotonic', lambda: clock[0])
    backend = MockBackend('right', False)
    backend.contract.update(entry_protocol=6, entry_keepalive_protocol=1)
    backend.activate_entry = Mock()
    g = Gateway(backend, b'entry-keepalive-tests-only-key-12345', port=0)
    peer = ('127.0.0.1', 19000)
    g.gate.hello({'client_nonce': 'n'*32}, peer, clock[0], False)
    lease = g.gate.state(clock[0], {})['lease']
    target = dict(type='target', side='right', contract_id=backend.contract['id'],
                  session=g.gate.session, seq=0, lease=lease, arm_urdf_rad=[.2]*7, hand_unit=None,
                  entry=dict(phase='candidate', token='', source=[0.]*9, reference_source=[0.]*9))
    g.gate.accept(target, peer, clock[0])
    g.mode = 'CALIBRATING'
    g.entry_target = copy.deepcopy(g.gate.target)
    g.entry_candidate_since = clock[0]-.6
    g.request('g')
    def heartbeat(**overrides):
        lease = g.gate.state(clock[0], {})['lease']
        msg = dict(type='entry_keepalive', side='right', contract_id=backend.contract['id'],
                   session=g.gate.session, seq=g.gate.sequence+1, lease=lease,
                   plan_token=g.entry_plan_token)
        msg.update(overrides)
        # Exercise actual HMAC pack/unpack fields too.
        g.accept_command(unpack(pack(msg, g.key), g.key), peer, clock[0])
        return msg
    try:
        yield g, backend, clock, peer, heartbeat
    finally:
        g.socket.close()


@pytest.mark.parametrize('gap', [.12, .3, 1., 3.])
def test_execution_continues_without_pico_and_arrival_timeout_remains_two_seconds(fixed_gateway, gap):
    g, backend, clock, peer, heartbeat = fixed_gateway
    frozen = copy.deepcopy(g.entry_target)
    def approach(target, cancelled, on_approach):
        assert target == frozen
        on_approach()
        backend.enabled = True
        backend.q = [.2]*7
        for _ in range(round(gap/.02)):
            clock[0] += .02
            heartbeat()
            assert not cancelled()
            assert g.gate.target == frozen
        return dict(returned=True, disabled=False)
    backend.approach_entry = Mock(side_effect=approach)
    g.tick()
    assert g.mode == 'WAITING_FOR_ACTIVE_TARGET', g.reason
    deadline = g.entry_ready_deadline
    for _ in range(101):
        clock[0] += .02
        heartbeat()
        g.watchdog()
    assert g.mode == 'FAULT'
    assert g.first_fault_reason == 'fresh ACTIVE target acknowledgment timed out'
    assert g.entry_ready_deadline == deadline
    assert g.entry_plan_token == ''
    backend.activate_entry.assert_not_called()


def test_keepalive_does_not_renew_old_ready_acknowledgment(fixed_gateway):
    g, backend, clock, peer, heartbeat = fixed_gateway
    g.mode, g.pending = 'WAITING_FOR_ACTIVE_TARGET', None
    backend.enabled = True
    backend.q = [.2]*7
    g.entry_token = 'arrival-token'
    g.entry_ready_deadline = clock[0]+2
    ready = copy.deepcopy(g.gate.target)
    ready['entry'].update(phase='ready', token=g.entry_token)
    msg = dict(ready, type='target', side='right', contract_id=backend.contract['id'],
               session=g.gate.session, seq=1, lease=g.gate.state(clock[0], {})['lease'])
    g.accept_command(msg, peer, clock[0])
    for _ in range(15):
        clock[0] += .02
        heartbeat()
    assert g.gate.fresh(clock[0])
    g.tick()
    backend.activate_entry.assert_not_called()
    msg.update(seq=g.gate.sequence+1, lease=g.gate.state(clock[0], {})['lease'])
    g.accept_command(msg, peer, clock[0])
    g.tick()
    backend.activate_entry.assert_called_once()
    assert g.mode == 'ACTIVE' and not g.entry_plan_token
    with pytest.raises(ProtocolError):
        heartbeat(plan_token='x'*32)


@pytest.mark.parametrize('mode', ['IDLE', 'CALIBRATING', 'ACTIVE', 'FAULT', 'RETURNING'])
def test_keepalive_rejected_outside_entry(fixed_gateway, mode):
    g, backend, clock, peer, heartbeat = fixed_gateway
    g.mode = mode
    before = (g.gate.sequence, g.gate.deadline)
    with pytest.raises(ProtocolError):
        heartbeat()
    assert before == (g.gate.sequence, g.gate.deadline)


def test_invalid_replay_and_disconnection_do_not_renew_plan(fixed_gateway):
    g, backend, clock, peer, heartbeat = fixed_gateway
    for overrides in ({'plan_token': 'wrong'*7}, {'session': 'old-session'}, {'plan_token': 'x'*32},
                      {'lease': 'old'}, {'arm_urdf_rad': [0]*7}, {'seq': 0}):
        before = (g.gate.sequence, g.gate.deadline)
        with pytest.raises(ProtocolError):
            heartbeat(**overrides)
        assert before == (g.gate.sequence, g.gate.deadline)
    msg = heartbeat()
    with pytest.raises(ProtocolError):
        g.accept_command(msg, peer, clock[0])
    clock[0] += .201
    with pytest.raises(ProtocolError, match='expired plan'):
        heartbeat()
    g.watchdog()
    assert g.mode == 'FAULT'
    assert 'lease expired' in g.first_fault_reason
    assert g.entry_plan_token == ''


def test_ready_cannot_expire_during_slow_activation_while_heartbeats_continue(fixed_gateway):
    g, backend, clock, peer, heartbeat = fixed_gateway
    g.mode, g.pending = 'WAITING_FOR_ACTIVE_TARGET', None
    backend.enabled = True
    g.entry_token = 'arrival-token'
    g.entry_ready_deadline = clock[0]+2
    ready = copy.deepcopy(g.gate.target)
    ready['entry'].update(phase='ready', token=g.entry_token)
    g.accept_command(dict(ready, type='target', side='right', contract_id=backend.contract['id'],
                          session=g.gate.session, seq=1, lease=g.gate.state(clock[0], {})['lease']),
                     peer, clock[0])
    def slow_activation(*args):
        for _ in range(15):
            clock[0] += .02
            heartbeat()
    backend.activate_entry.side_effect = slow_activation
    g.tick()
    assert g.mode == 'FAULT'
    assert 'ready acknowledgment expired' in g.first_fault_reason


def test_operator_stop_invalidates_plan_immediately(fixed_gateway):
    g, backend, clock, peer, heartbeat = fixed_gateway
    old = g.entry_plan_token
    g.request('s')
    assert g.mode == 'FAULT' and not g.entry_plan_token
    with pytest.raises(ProtocolError):
        heartbeat(plan_token=old)


def test_contract_requires_explicit_keepalive_support_for_real_single_arm():
    from esrobo_laptop.config import Settings
    from esrobo_laptop.contract import verify_contract
    backend = MockBackend('right', False)
    backend.contract['entry_protocol'] = 6
    with pytest.raises(ProtocolError, match='entry_keepalive_protocol'):
        verify_contract(backend.contract, None, Settings(side='right', with_hand=False))
