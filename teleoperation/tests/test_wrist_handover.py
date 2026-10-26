"""Current recovery plus explicitly scoped protocol-6 compatibility; no hardware."""
import copy
from dataclasses import replace
import time

import numpy as np
import pytest

from esrobo_laptop.inputs import SourceExpired
from esrobo_laptop.wrist_handover import WristHandover
from esrobo_link.protocol import canonical
from esrobo_teleop.config import RobotConfig, HandConfig
from test_entry_keepalive import fixed_controller


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('with_hand', [False, True])
@pytest.mark.parametrize('gap', [.12, .238, .374])
def test_recorded_network_gaps_do_not_turn_filter_catchup_into_source_motion(side,with_hand,gap):
    from tools.audit_wrist_filter_recovery import replay
    result=replay(side,with_hand,gap_s=gap)
    assert result['resumed_after_frames']==3
    assert result['timeout'] is None
    # Long gaps reseed clocks only; waiting candidates must not advance state.
    assert [r['committed_filter_deg'] for r in result['rows'][:2]] == pytest.approx(
        [result['before_gap_filter_deg']]*2)
    assert [r['filtered_j7_deg'] for r in result['rows'][:3]] == pytest.approx(
        [result['rows'][0]['filtered_j7_deg']]*3)
    assert result['committed_filter_deg'] > result['before_gap_filter_deg']


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('with_hand', [False, True])
def test_source_change_remains_visible_and_qualifies_current_recovery(side,with_hand):
    from tools.audit_wrist_filter_recovery import replay
    result=replay(side,with_hand,source_jump=8.)
    assert result['resumed_after_frames'] == 3
    assert result['timeout'] is None
    assert result['last_sent_source_deg'] == pytest.approx(result['rows'][-1]['source_palm_deg'])
    assert abs(result['rows'][0]['source_palm_deg']-result['rows'][0]['filtered_j7_deg']) > 5


@pytest.mark.parametrize('side', ['left', 'right'])
def test_proposal_is_bounded_and_not_committed_until_sent(side):
    h = WristHandover(np.zeros(7), RobotConfig(), side, 100., 50.)
    goal = np.deg2rad([0, 0, 0, 0, 0, 12, -21])
    first = h.propose(goal, np.zeros(7), 100.5)
    assert np.all(np.abs(first.position) <= h.velocity*.02+1e-12)
    assert np.all(np.abs(first.position) <= h.step)
    np.testing.assert_array_equal(h.command, [0., 0.])
    assert h.last_at == 100.
    h.commit(first)
    np.testing.assert_array_equal(h.command, first.position)


def test_feedback_lead_no_arrival_without_feedback_and_timeout():
    h = WristHandover(np.zeros(7), RobotConfig(), 'left', 0., 50.)
    goal = np.deg2rad([0, 0, 0, 0, 0, 12, -21])
    for i in range(1, 450):
        h.commit(h.propose(goal, np.zeros(7), i*.02))
        assert np.all(np.abs(h.command) <= h.lead+1e-12)
        assert not h.done
    with pytest.raises(RuntimeError, match='initial wrist handover exceeded'):
        h.check_timeout(10.01)


def test_pause_uses_fresh_pose_and_cannot_bank_elapsed_motion_credit():
    h = WristHandover(np.zeros(7), RobotConfig(), 'left', 0., 50.)
    h.pause(.1)
    actual = np.deg2rad([0, 0, 0, 0, 0, .2, -.3])
    goal = np.deg2rad([0, 0, 0, 0, 0, 12, -21])
    p = h.propose(goal, actual, .4)
    assert np.all(np.abs(p.position-actual[5:]) <= h.velocity*.02+1e-12)
    with pytest.raises(ValueError, match='backwards'):
        h.propose(goal, actual, .09)


def setup(monkeypatch, side='left', with_hand=False, *, entry_protocol=7):
    clock = [100.]
    monkeypatch.setattr(time, 'monotonic', lambda: clock[0])
    c, p, client, state = fixed_controller('WAITING_FOR_ACTIVE_TARGET')
    c.settings = replace(c.settings, side=side, with_hand=with_hand)
    state['contract'].update(side=side, with_hand=with_hand, entry_protocol=entry_protocol)
    c.contract = canonical(state['contract'])
    baseline = [0.]*7
    baseline[6] = np.deg2rad(-.092)
    state['feedback']['arm_urdf_rad'] = baseline.copy()
    state['feedback']['enable_states'] = [True]*7
    c.entry_target = {side: dict(arm_urdf_rad=baseline.copy(), hand_unit=None)}
    if with_hand:
        p.cfg.hand = HandConfig()
        c.entry_target[side]['hand_unit'] = [128]*10
        state['feedback']['hand'] = dict(position_unit=[128]*10, age_s=0.)
        p.map_hand_target.return_value = [128]*10
    p.solve.return_value = copy.deepcopy(c.entry_target)
    c.step(state)
    state['mode'] = 'ACTIVE'
    p.solve.return_value[side]['arm_urdf_rad'][5:] = np.deg2rad([12., -21.]).tolist()
    def tick(dt=.02):
        clock[0] += dt
        client.received_at = clock[0]
        return c.step(state)
    return c, p, client, state, clock, tick


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('with_hand', [False, True])
def test_protocol7_keeps_gateway_arrival_and_accepts_latest_input(monkeypatch, side, with_hand):
    c,p,client,state,clock,tick = setup(monkeypatch, side, with_hand)
    c.recovery_enabled = True
    arrived = copy.deepcopy(c.entry_target)
    ticket = p.body.ticket.return_value
    p.body.ticket.return_value = None
    assert tick() is None
    assert c.wrist_handover is None  # Protocol 7 arrived on the gateway.
    assert c.last_active_target == arrived
    assert c.hold_missing_input(state, 'first ACTIVE input missing')
    sends = client.send_target.call_count
    p.body.ticket.return_value = ticket
    # Source movement during approach must not invalidate fresh input.
    for i in range(3):
        result = tick()
        assert (result is not None) == (i == 2)
        if i < 2:
            assert c.input_resume_count == i + 1
            assert client.send_target.call_count == sends
    assert c.wrist_handover is None and c.input_hold_started_at is None
    assert result == p.solve.return_value
    p.reset_command_start.assert_called_once()


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('with_hand', [False, True])
@pytest.mark.parametrize('first_gap', [False, True])
def test_recorded_gap_enters_bounded_transition_and_finishes(monkeypatch, side, with_hand, first_gap):
    c, p, client, state, clock, tick = setup(monkeypatch, side, with_hand, entry_protocol=6)
    before = np.array(state['feedback']['arm_urdf_rad'])
    if first_gap:
        ticket = p.body.ticket.return_value
        p.body.ticket.return_value = None
        assert tick() is None
        assert c.hold_missing_input(state, 'waiting for next complete source frame')
        assert c.input_hold_status.startswith('WAITING_FOR_FIRST_ACTIVE_INPUT')
        p.body.ticket.return_value = ticket
        assert tick() is None
        assert tick() is None
    target = tick()
    assert target is not None
    assert c.input_hold_started_at is None
    assert not c.entry_active_pending
    first = np.array(target[side]['arm_urdf_rad'])
    assert np.all(abs(first[5:]-before[5:]) <= c.wrist_handover.velocity*.02+1e-12)
    # Compare with the original failure's old baseline: it would reject 20.9°.
    assert abs(p.solve.return_value[side]['arm_urdf_rad'][6]-before[6]) > np.deg2rad(5.)
    for _ in range(150):
        state['feedback']['arm_urdf_rad'] = list(target[side]['arm_urdf_rad'])
        previous = np.array(target[side]['arm_urdf_rad'])
        target = tick()
        now = np.array(target[side]['arm_urdf_rad'])
        assert np.all(abs(now[5:]-previous[5:]) <= c.wrist_handover.velocity*.02+1e-12)
        np.testing.assert_array_equal(now[:5], previous[:5])
        if c.wrist_handover.done:
            break
    assert c.wrist_handover.done and c.wrist_handover.matched == 3
    np.testing.assert_allclose(target[side]['arm_urdf_rad'], p.solve.return_value[side]['arm_urdf_rad'])


def test_legacy_first_input_accepts_moving_source_without_resetting_handover(monkeypatch):
    c, p, client, state, clock, tick = setup(monkeypatch, entry_protocol=6)
    ticket = p.body.ticket.return_value
    p.body.ticket.return_value = None
    tick(); c.hold_missing_input(state, 'first gap')
    p.body.ticket.return_value = ticket
    for i in range(3):
        p.solve.return_value['left']['arm_urdf_rad'][6] += np.deg2rad(8.)
        p.solve.return_value['left']['arm_urdf_rad'][0] = np.deg2rad(8. * (i+1))
        result = tick()
        assert (result is not None) == (i == 2)
    assert not c.entry_active_pending
    assert abs(result['left']['arm_urdf_rad'][6]-state['feedback']['arm_urdf_rad'][6]) <= c.wrist_handover.velocity[1]*.02 + 1e-12
    p.reset_command_start.assert_called_once()


@pytest.mark.parametrize('jump', [4., 8.])
def test_recovery_accepts_valid_source_changes_after_three_frames(monkeypatch, jump):
    c, p, client, state, clock, tick = setup(monkeypatch)
    target = tick()
    state['feedback']['arm_urdf_rad'] = target['left']['arm_urdf_rad']
    p.body.ticket.side_effect = SourceExpired('stale')
    with pytest.raises(SourceExpired):tick(.06)
    assert c.hold_missing_input(state, 'stale')
    p.body.ticket.side_effect = None
    p.solve.return_value['left']['arm_urdf_rad'][6] += np.deg2rad(jump)
    for i in range(3):
        result = tick()
        assert (result is not None) == (i == 2)
    assert c.input_hold_started_at is None
    assert result['left']['arm_urdf_rad'][6] == pytest.approx(p.solve.return_value['left']['arm_urdf_rad'][6])


@pytest.mark.parametrize('failure', ['send', 'stale_after_proposal'])
def test_no_commit_on_failed_send_or_stale_input(monkeypatch, failure):
    c, p, client, state, clock, tick = setup(monkeypatch, entry_protocol=6)
    tick()
    old = c.wrist_handover.command.copy()
    stamp = c.wrist_handover.last_at
    if failure == 'send':
        client.send_target.side_effect = OSError('send failed')
        error = OSError
    else:
        p.check_ticket.side_effect = [None, SourceExpired('expired after proposal')]
        error = SourceExpired
    with pytest.raises(error):tick()
    assert p.commit_frame.call_count == 1
    np.testing.assert_array_equal(c.wrist_handover.command, old)
    assert c.wrist_handover.last_at == stamp


def test_duplicate_frames_do_not_advance_transition(monkeypatch):
    c, p, client, state, clock, tick = setup(monkeypatch, entry_protocol=6)
    tick()
    old, stamp = c.wrist_handover.command.copy(), c.wrist_handover.last_at
    sends = client.send_target.call_count
    p.capture.return_value = None
    assert tick() is None
    np.testing.assert_array_equal(c.wrist_handover.command, old)
    assert c.wrist_handover.last_at == stamp
    assert client.send_target.call_count == sends


def test_failed_recovery_send_keeps_original_hold_deadline_and_source(monkeypatch):
    c,p,client,state,clock,tick=setup(monkeypatch)
    target=tick();state['feedback']['arm_urdf_rad']=target['left']['arm_urdf_rad']
    clock[0]+=.12;client.received_at=clock[0]
    assert c.hold_missing_input(state,'source gap')
    deadline_start=c.input_hold_started_at
    original_source=copy.deepcopy(c.last_arm_source)
    commits=p.commit_frame.call_count
    assert tick() is None
    assert tick() is None
    client.send_target.side_effect=OSError('send failed')
    with pytest.raises(OSError):tick()
    assert c.input_hold_started_at==deadline_start
    assert c.last_arm_source==original_source
    assert p.commit_frame.call_count==commits
    with pytest.raises(RuntimeError,match='hold exceeded 500 ms'):tick(.501)


@pytest.mark.parametrize('side', ['left', 'right'])
def test_live_goal_within_arrival_tolerance_does_not_require_exact_equality(side):
    h = WristHandover(np.zeros(7), RobotConfig(), side, 0., 50.)
    for n in range(1, 4):
        actual = np.zeros(7)
        actual[5:] = h.command
        target = actual.copy()
        target[5:] += min(h.tolerance*.8, .02)
        proposed = h.propose(target, actual, n*.02)
        assert np.any(np.abs(proposed.position-target[5:]) > 1e-10)
        assert not h.done
        h.commit(proposed)
    assert h.done


def test_live_goal_outside_feedback_tolerance_cannot_complete_handover():
    h = WristHandover(np.zeros(7), RobotConfig(), 'left', 0., 50.)
    for n in range(1, 10):
        target = np.zeros(7)
        target[5:] = h.tolerance*1.1
        h.commit(h.propose(target, np.zeros(7), n*.02))
        assert not h.done and h.matched == 0


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('axis', range(7))
@pytest.mark.parametrize('recovery_enabled', [False, True])
def test_first_entry_all_axes_accept_large_moving_source(monkeypatch, side, axis, recovery_enabled):
    c, p, client, state, clock, tick = setup(monkeypatch, side)
    c.recovery_enabled = recovery_enabled
    ticket = p.body.ticket.return_value
    p.body.ticket.return_value = None
    tick(); c.hold_missing_input(state, 'first gap')
    p.body.ticket.return_value = ticket
    baseline = copy.deepcopy(c.entry_target[side]['arm_urdf_rad'])
    for i in range(3):
        source = np.array(baseline)
        source[axis] += np.deg2rad(17. + 8. * i)
        p.solve.return_value[side]['arm_urdf_rad'] = source.tolist()
        result = tick()
        assert (result is not None) == (i == 2)
        if i < 2:
            assert c.input_resume_count == i+1
            assert c.input_resume_diagnostics['limits'][side]['arm_limit_deg'] is None
    assert not c.entry_active_pending
    assert c.last_arm_source[side] == source.tolist()
    p.reset_command_start.assert_called_once()
    assert c.entry_target[side]['arm_urdf_rad'] == baseline


@pytest.mark.parametrize('side', ['left', 'right', 'both'])
@pytest.mark.parametrize('first', [False, True])
def test_first_and_live_resume_share_source_validation(monkeypatch, side, first):
    from esrobo_laptop.input_recovery import RecoveryRejected
    c,p,client,state,clock,tick = setup(monkeypatch)
    c.settings = replace(c.settings, side=side, with_hand=True)
    c.entry_active_pending = first
    c.last_active_target = {s: dict(arm_urdf_rad=[0.]*7, hand_unit=[128]*10) for s in c.settings.sides}
    goal = {s: dict(arm_urdf_rad=[.5]*7, hand_unit=[200]*10) for s in c.settings.sides}
    assert c._resume_valid(goal)
    for s in c.settings.sides:
        assert c.input_resume_diagnostics['limits'][s]['arm_limit_deg'] is None
    for bad in ([0.]*6, [float('nan')]*7, [float('inf')]*7):
        invalid = copy.deepcopy(goal)
        invalid[c.settings.sides[-1]]['source_arm_urdf_rad'] = bad
        with pytest.raises(RecoveryRejected, match='finite arm'):
            c._resume_valid(invalid)
    invalid = copy.deepcopy(goal)
    invalid[c.settings.sides[-1]]['hand_unit'] = [256]*10
    with pytest.raises(RecoveryRejected, match='valid hand'):
        c._resume_valid(invalid)
