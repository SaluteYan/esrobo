"""First finger handover, including the recorded 142 -> 239 mismatch."""
import copy
from dataclasses import replace
from types import SimpleNamespace
import time

import numpy as np
import pytest

from esrobo_laptop.hand_handover import HandHandover
from esrobo_laptop.inputs import SourceExpired, InputUnavailable
from test_entry_keepalive import fixed_controller


def config():
    return SimpleNamespace(max_velocity=400, max_step=18, publish_hz=50,
                           enabled_physical_joints=[0, 1, 2, 3, 4, 5, 9])


def sent_step(handover, desired, measured, now):
    candidate = handover.propose(desired, measured, now)
    handover.commit(candidate)
    return list(candidate.position)


def setup(monkeypatch, side='right'):
    clock = [100.]
    monkeypatch.setattr(time, 'monotonic', lambda: clock[0])
    c, p, client, state = fixed_controller('WAITING_FOR_ACTIVE_TARGET')
    c.settings = replace(c.settings, side=side, with_hand=True)
    if side == 'left':
        c.entry_target['left'] = c.entry_target.pop('right')
    held = [142, 180, 226, 221, 226, 236, 0, 0, 0, 37]
    source = [238, 169, 213, 179, 192, 208, 0, 0, 0, 41]
    c.entry_target[side]['hand_unit'] = held.copy()
    state['feedback']['hand'] = dict(position_unit=held.copy(), age_s=0.)
    p.cfg = SimpleNamespace(robot=p.cfg.robot, hand=config())
    p.map_hand_target.return_value = source.copy()
    p.solve.return_value = {side: dict(arm_urdf_rad=[.2]*7, hand_unit=source.copy(), hold=False)}
    c.step(state)
    assert client.send_target.call_args.kwargs['hand_unit'] == held
    assert c.hand_handover is None
    state['mode'] = 'ACTIVE'
    state['feedback']['enable_states'] = [True]*7
    def tick(dt=.02):
        clock[0] += dt
        client.received_at = clock[0]
        return c.step(state)
    return c, p, client, state, clock, tick


def test_recorded_first_active_gap_recovers_and_reaches_absolute_hand_target(monkeypatch):
    c, p, client, state, clock, tick = setup(monkeypatch)
    p.body.ticket.side_effect = SourceExpired('143 ms old')
    with pytest.raises(SourceExpired):
        tick()
    assert c.hold_missing_input(state, '143 ms old')
    assert c.last_hand_source[0] == 238
    assert c.last_active_target['right']['hand_unit'][0] == 142
    p.body.ticket.side_effect = None
    p.solve.return_value['right']['hand_unit'][0] = 239
    for i in range(3):
        target = tick()
        assert (target is not None) == (i == 2)
    assert c.input_hold_started_at is None
    assert target['right']['hand_unit'][0] == 150  # bounded first step, not 239
    for _ in range(45):
        state['feedback']['hand']['position_unit'] = target['right']['hand_unit'].copy()
        previous = np.array(target['right']['hand_unit'])
        target = tick()
        assert np.max(np.abs(np.array(target['right']['hand_unit'])-previous)) <= 8
        if c.hand_handover.done:
            break
    assert c.hand_handover.done
    assert target['right']['hand_unit'] == p.solve.return_value['right']['hand_unit']
    assert c.entry_target['right']['hand_unit'][0] == 142
    assert client.send_entry_keepalive.call_count == 1


@pytest.mark.parametrize('kind', ['arm_jump', 'no_input'])
def test_transition_still_rejects_discontinuity_and_expires_at_500ms(monkeypatch, kind):
    c, p, client, state, clock, tick = setup(monkeypatch)
    p.body.ticket.return_value = None
    assert tick() is None
    assert c.hold_missing_input(state, 'missing')
    if kind != 'no_input':
        p.body.ticket.return_value = {'right_arm': clock[0]}
        p.solve.return_value['right']['arm_urdf_rad'][0] += np.deg2rad(6)
        for _ in range(3):
            assert tick() is None
        assert not c.input_resume_diagnostics['allowed']
        assert c.input_resume_count == 0
    clock[0] = c.input_hold_started_at + .501
    client.received_at = clock[0]
    with pytest.raises(RuntimeError, match='hold exceeded 500 ms'):
        c.hold_missing_input(state, 'missing')
    assert client.send_target.call_count == 1
    assert c.last_hand_source[0] == 238


def test_gap_during_ramp_rebases_robot_command_but_retains_source_baseline(monkeypatch):
    c, p, client, state, clock, tick = setup(monkeypatch)
    target = tick()
    target = tick()
    state['feedback']['hand']['position_unit'] = target['right']['hand_unit'].copy()
    desired = c.last_hand_source.copy()
    p.body.ticket.return_value = None
    assert tick(.06) is None
    assert c.hold_missing_input(state, 'gap during hand transition')
    actual = state['feedback']['hand']['position_unit'].copy()
    assert c.hand_handover.command.tolist() == actual
    assert c.last_hand_source == desired
    p.body.ticket.return_value = {'right_arm': clock[0]}
    for _ in range(3):
        target = tick()
    assert target is not None
    assert np.max(np.abs(np.array(target['right']['hand_unit'])-actual)) <= 8


def test_transition_rejects_feedback_fault_and_planner_failure(monkeypatch):
    c, p, client, state, clock, tick = setup(monkeypatch)
    tick()
    count = client.send_target.call_count
    state['feedback']['hand']['age_s'] = 1.
    with pytest.raises(InputUnavailable, match='feedback stale'):
        tick()
    assert client.send_target.call_count == count
    state['feedback']['hand']['age_s'] = 0.
    p.solve.side_effect = InputUnavailable('IK/collision rejected')
    with pytest.raises(InputUnavailable, match='IK/collision'):
        tick()
    assert client.send_target.call_count == count


def test_stalled_feedback_cannot_accumulate_command_lead_and_times_out():
    actual = [142]*10
    h = HandHandover(actual, config(), 100., 50)
    for i in range(1, 100):
        out = sent_step(h, [239]*10, actual, 100.+i*.02)
        assert max(out) <= 157
        assert out[6:9] == actual[6:9]
    assert not h.done
    with pytest.raises(RuntimeError, match=r'exceeded 10 s.*physical\[0\] goal=239 command=157 feedback=142'):
        h.propose([239]*10, actual, 110.01)


def test_pause_does_not_accumulate_velocity_credit_and_completion_needs_feedback():
    h = HandHandover([180]*10, config(), 100., 50)
    assert max(sent_step(h, [239]*10, [180]*10, 100.02)) <= 188
    h.hold([181]*10, 100.4)
    assert max(sent_step(h, [239]*10, [181]*10, 100.42)) <= 189
    h = HandHandover([238]*10, config(), 100., 50)
    for i in range(1, 4):
        sent_step(h, [239]*10, [238]*10, 100.+i*.02)
        assert h.done == (i == 3)


@pytest.mark.parametrize('bad', [[1]*9, [float('nan')]*10, [-1]*10])
def test_invalid_feedback_is_rejected(bad):
    with pytest.raises(ValueError):
        HandHandover(bad, config(), 100., 50)


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('failure', ['send', 'expired_after_proposal', 'hold_send'])
def test_unsent_hand_candidate_never_changes_committed_state(monkeypatch, side, failure):
    c, p, client, state, clock, tick = setup(monkeypatch, side)
    tick()
    before = copy.deepcopy(vars(c.hand_handover))
    source_before = c.last_hand_source.copy()
    if failure == 'expired_after_proposal':
        p.check_ticket.side_effect = [None, SourceExpired('expired after proposal')]
        error = SourceExpired
    else:
        client.send_target.side_effect = OSError('send failed')
        if failure == 'hold_send':
            p.solve.return_value[side]['hold'] = True
            client.send_hold.side_effect = OSError('send failed')
        error = OSError
    with pytest.raises(error):
        tick()
    for name, value in before.items():
        np.testing.assert_equal(getattr(c.hand_handover, name), value)
    assert c.last_hand_source == source_before


@pytest.mark.parametrize('sample_every,delay,reverse_at', [(3, 4, 8), (4, 4, 12)])
@pytest.mark.parametrize('start,goal', [(240, 0), (0, 240)])
def test_moving_source_cannot_reverse_initial_reference(sample_every, delay, reverse_at, start, goal):
    # Delayed, batched feedback and an operator reversing direction must still
    # complete the first transition, without chasing the new live target.
    h = HandHandover([start]*10, config(), 0., 50)
    commands = [[start]*10]
    actual = commands[0]
    for i in range(1, 451):
        if i % sample_every == 0:
            actual = commands[max(0, len(commands)-delay)]
        out = sent_step(h, [goal if i <= reverse_at else start]*10, actual, i*.02)
        assert max(abs(out[j]-commands[-1][j]) for j in h.enabled) <= 8
        assert max(abs(out[j]-actual[j]) for j in h.enabled) <= 15
        assert out[6:9] == actual[6:9]
        commands.append(out)
        if h.done:
            break
    assert h.done
    assert all(commands[-1][j] == goal for j in h.enabled)
    assert h.reference_target == (goal,)*10


def test_fixed_reference_does_not_hide_abnormal_feedback_or_stall():
    h = HandHandover([200]*10, config(), 0., 50)
    sent_step(h, [0]*10, [200]*10, .02)
    sent_step(h, [0]*10, [200]*10, .04)
    for i in range(3, 100):
        out = sent_step(h, [240]*10, [200]*10, i*.02)
        assert out[3] == 185  # wait for actual, no reversal into old motion
        assert h.reference_target == (0,)*10
    before = h.command.copy()
    with pytest.raises(RuntimeError, match=r'joint 4.*physical\[3\].*command=185 feedback=240 budget=8 lead=15'):
        h.propose([240]*10, [200]*3+[240]+[200]*6, 2.)
    np.testing.assert_array_equal(h.command, before)
    with pytest.raises(RuntimeError, match='exceeded 10 s'):
        h.propose([240]*10, [200]*10, 10.01)


def test_discarded_proposal_does_not_consume_direction_or_motion_credit():
    h = HandHandover([200]*10, config(), 0., 50)
    discarded = h.propose([0]*10, [200]*10, .02)
    assert discarded.position[3] == 192
    np.testing.assert_array_equal(h.command, [200]*10)
    assert h.last_at == 0. and not h.direction.any()
    assert h.reference_target is None
    out = sent_step(h, [240]*10, [200]*10, .08)
    assert out[3] == 208 and not h.reversal_wait
    with pytest.raises(ValueError, match='clock'):
        h.propose([240]*10, [200]*10, .07)


@pytest.mark.parametrize('side', ['left', 'right'])
def test_controller_both_sides_complete_delayed_reversing_handover(monkeypatch, side):
    c, p, client, state, clock, tick = setup(monkeypatch, side)
    state['feedback']['hand']['position_unit'] = [240]*10
    history = [[240]*10]
    for i in range(1, 451):
        if i % 3 == 0:
            state['feedback']['hand']['position_unit'] = history[max(0, len(history)-4)].copy()
        p.solve.return_value[side]['hand_unit'] = [0 if i <= 8 else 240]*10
        out = tick()[side]['hand_unit']
        assert client.send_target.call_args.kwargs['hand_unit'] == out
        assert max(abs(out[j]-history[-1][j]) for j in config().enabled_physical_joints) <= 8
        history.append(out.copy())
        if c.hand_handover.done:
            break
    assert c.hand_handover.done
    assert all(history[-1][j] == 0 for j in config().enabled_physical_joints)
    assert c.last_hand_source == [240]*10
    # The next frame supplies the current target to the gateway's continuous
    # elapsed-time limiter; it never restarts the initial 10 s transition.
    assert tick()[side]['hand_unit'] == [240]*10
    client.send_hold.assert_not_called()
