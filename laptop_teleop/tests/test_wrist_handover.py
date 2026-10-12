"""Offline first-wrist transition and recorded 21-degree boundary regression."""
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


def setup(monkeypatch, side='left', with_hand=False):
    clock = [100.]
    monkeypatch.setattr(time, 'monotonic', lambda: clock[0])
    c, p, client, state = fixed_controller('WAITING_FOR_ACTIVE_TARGET')
    c.settings = replace(c.settings, side=side, with_hand=with_hand)
    state['contract'].update(side=side, with_hand=with_hand)
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
@pytest.mark.parametrize('first_gap', [False, True])
def test_recorded_gap_enters_bounded_transition_and_finishes(monkeypatch, side, with_hand, first_gap):
    c, p, client, state, clock, tick = setup(monkeypatch, side, with_hand)
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


def test_first_frame_anchor_is_fixed_and_other_arm_joints_still_checked(monkeypatch):
    c, p, client, state, clock, tick = setup(monkeypatch)
    ticket = p.body.ticket.return_value
    p.body.ticket.return_value = None
    tick(); c.hold_missing_input(state, 'first gap')
    p.body.ticket.return_value = ticket
    assert tick() is None
    anchor = list(c.first_wrist_source)
    p.solve.return_value['left']['arm_urdf_rad'][6] += np.deg2rad(8.)
    assert tick() is None
    assert c.input_resume_count == 0 and c.first_wrist_source == anchor
    p.solve.return_value['left']['arm_urdf_rad'][6] = anchor[1]
    p.solve.return_value['left']['arm_urdf_rad'][0] = np.deg2rad(8.)
    assert tick() is None
    assert c.input_resume_diagnostics['joint'] == 1


@pytest.mark.parametrize('jump', [4., 8.])
def test_mid_transition_recovery_compares_raw_sources_not_rate_limited_commands(monkeypatch, jump):
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
        assert (result is not None) == (jump == 4. and i == 2)
    if jump == 8.:
        assert c.input_resume_count == 0
        assert c.input_resume_diagnostics['arm_delta_deg'] == pytest.approx(8.)
        with pytest.raises(RuntimeError, match='hold exceeded 500 ms'):tick(.501)


@pytest.mark.parametrize('failure', ['send', 'stale_after_proposal'])
def test_no_commit_on_failed_send_or_stale_input(monkeypatch, failure):
    c, p, client, state, clock, tick = setup(monkeypatch)
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
    np.testing.assert_array_equal(c.wrist_handover.command, old)
    assert c.wrist_handover.last_at == stamp


def test_duplicate_frames_do_not_advance_transition(monkeypatch):
    c, p, client, state, clock, tick = setup(monkeypatch)
    tick()
    old, stamp = c.wrist_handover.command.copy(), c.wrist_handover.last_at
    sends = client.send_target.call_count
    p.capture.return_value = None
    assert tick() is None
    np.testing.assert_array_equal(c.wrist_handover.command, old)
    assert c.wrist_handover.last_at == stamp
    assert client.send_target.call_count == sends


def test_completed_transition_retains_normal_recovery_delta_gate(monkeypatch):
    c, p, client, state, clock, tick = setup(monkeypatch)
    target = tick()
    for _ in range(150):
        state['feedback']['arm_urdf_rad'] = target['left']['arm_urdf_rad']
        target = tick()
        if c.wrist_handover.done:break
    assert c.wrist_handover.done
    p.body.ticket.side_effect = SourceExpired('stale')
    with pytest.raises(SourceExpired):tick(.06)
    c.hold_missing_input(state, 'stale')
    p.body.ticket.side_effect = None
    p.solve.return_value['left']['arm_urdf_rad'][6] += np.deg2rad(8.)
    assert tick() is None
    assert c.input_resume_count == 0
    assert c.input_resume_diagnostics['arm_delta_deg'] == pytest.approx(8.)
