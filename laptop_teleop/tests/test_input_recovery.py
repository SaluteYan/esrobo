import copy
from dataclasses import replace

import numpy as np
import pytest

from esrobo_laptop.config import Settings
from esrobo_laptop.input_recovery import InputRecovery, RecoveryRejected
from esrobo_laptop.inputs import SourceExpired
from test_wrist_handover import setup


def targets(settings, wrist=0., hand=128, proximal=0.):
    return {side: dict(arm_urdf_rad=None if settings.hand_only else
                      np.deg2rad([proximal]*4+[0., 0., wrist]).tolist(),
                      hand_unit=[hand]*10 if settings.with_hand else None)
            for side in settings.sides}


@pytest.mark.parametrize('side,with_hand,hand_only', [
    ('left', False, False), ('right', False, False), ('left', True, False),
    ('right', True, False), ('left', True, True), ('right', True, True), ('both', True, False)])
@pytest.mark.parametrize('gap', [.065, .1, .2])
def test_recorded_motion_after_gap_and_fixed_deadline(side, with_hand, hand_only, gap):
    s = Settings(side=side, with_hand=with_hand, hand_only=hand_only)
    r = InputRecovery(s)
    r.begin(targets(s))
    for i in range(3):
        assert r.evaluate(targets(s, wrist=13.33, hand=144), gap+i*.02) == (i == 2)
    r.commit(gap+.04)
    anchor = copy.deepcopy(r.anchor)
    r.pause()
    assert not r.evaluate(targets(s, wrist=14., hand=145), gap+.2)
    assert r.started == gap+.04 and r.anchor == anchor
    with pytest.raises(RecoveryRejected, match='3 s'):
        r.check_timeout(gap+3.05)


def test_dual_requires_both_members_and_no_moving_anchor():
    s = Settings()
    r = InputRecovery(s); r.begin(targets(s))
    value = targets(s, wrist=14)
    value['right']['arm_urdf_rad'][6] = np.deg2rad(16)
    with pytest.raises(RecoveryRejected, match='fixed recovery envelope'):
        r.evaluate(value, 1.)
    assert r.anchor == InputRecovery.sources(targets(s))


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('with_hand', [False, True])
def test_fresh_unreachable_frames_do_not_become_an_input_gap(monkeypatch, side, with_hand):
    c,p,client,state,clock,tick = setup(monkeypatch, side, with_hand)
    c.recovery_enabled = True
    p.solve.return_value = copy.deepcopy(c.entry_target)
    for _ in range(5):
        result = tick()
        state['feedback']['arm_urdf_rad'] = result[side]['arm_urdf_rad']
    anchor = copy.deepcopy(c.last_active_target)
    last_reachable_at = c.last_active_target_at
    p.solve.return_value[side]['hold'] = True
    # For 1.8 seconds the robot brakes, but PICO continues sending full frames.
    for _ in range(90):
        assert tick()[side]['hold']
    assert c.last_active_target == anchor
    assert c.last_active_target_at == last_reachable_at
    assert c.last_complete_input_at == clock[0]
    p.body.ticket.return_value = None
    assert tick(.021) is None
    assert not c.hold_missing_input(state, 'waiting for next complete source frame')
    assert c.recovery is None and c.input_hold_started_at is None
    # A real subsequent gap still starts recovery from the fixed reachable source.
    assert tick(.065) is None
    assert c.hold_missing_input(state, 'real gap')
    assert c.recovery.anchor == InputRecovery.sources(anchor)


@pytest.mark.parametrize('component', ['arm', 'hand', 'both'])
def test_recorded_j6_and_finger_excursion_still_rejected(component):
    s = Settings(side='left', with_hand=True)
    anchor = targets(s)
    anchor['left']['arm_urdf_rad'][5] = np.deg2rad(24.26008)
    anchor['left']['hand_unit'] = [240,205,219,219,225,229,26,122,121,128]
    candidate = copy.deepcopy(anchor)
    if component in ('arm', 'both'):
        candidate['left']['arm_urdf_rad'][5] = np.deg2rad(-6.744776)
    if component in ('hand', 'both'):
        candidate['left']['hand_unit'] = [225,134,147,132,136,137,22,122,121,128]
    r = InputRecovery(s)
    r.begin(anchor)
    with pytest.raises(RecoveryRejected) as exc:
        r.evaluate(candidate, 1.)
    if component in ('arm', 'both'):
        assert 'left J6 delta 31.00deg > 15deg' in str(exc.value)
    if component in ('hand', 'both'):
        assert 'left hand[5] delta 92 > 30' in str(exc.value)
    assert r.started is None and r.anchor == InputRecovery.sources(anchor)


def test_adjacent_large_jump_restarts_three_frame_qualification():
    s = Settings(side='left', with_hand=False)
    r = InputRecovery(s); r.begin(targets(s))
    assert not r.evaluate(targets(s, wrist=6), 1.)
    assert not r.evaluate(targets(s, wrist=14), 1.02)
    assert r.count == 1 and not r.diagnostic['adjacent_allowed']
    assert not r.evaluate(targets(s, wrist=14), 1.04)
    assert r.evaluate(targets(s, wrist=14), 1.06)


def active_fixture(monkeypatch, side='left', with_hand=False):
    c,p,client,state,clock,tick = setup(monkeypatch, side, with_hand)
    c.recovery_enabled = True
    # Complete first entry with matching measured targets, then create a gap.
    p.solve.return_value = copy.deepcopy(c.entry_target)
    for _ in range(5):
        result = tick()
        state['feedback']['arm_urdf_rad'] = result[side]['arm_urdf_rad']
    assert c.wrist_handover.done
    if c.hand_handover:
        assert c.hand_handover.done
    clock[0] += .065; client.received_at = clock[0]
    assert c.hold_missing_input(state, 'recorded gap')
    p.solve.return_value[side]['arm_urdf_rad'][6] += np.deg2rad(13.33)
    if with_hand:
        p.solve.return_value[side]['hand_unit'][1] += 16
    return c,p,client,state,clock,tick


@pytest.mark.parametrize('side', ['left','right'])
@pytest.mark.parametrize('with_hand', [False,True])
def test_controller_qualifies_then_reacquires_and_preserves_anchor(monkeypatch, side, with_hand):
    c,p,client,state,clock,tick = active_fixture(monkeypatch,side,with_hand)
    assert tick() is None
    assert tick() is None
    result = tick()
    assert result is not None and c.recovery.started is not None
    assert client.recovery_metadata['phase'] == 'reacquiring'
    anchor = copy.deepcopy(c.recovery.anchor)
    if with_hand:
        assert result[side]['hand_unit'][1] < 144
    state['feedback']['arm_urdf_rad'] = p.solve.return_value[side]['arm_urdf_rad']
    if with_hand:
        # Simulate progression, not a feedback teleport outside the lead band.
        for _ in range(4):
            state['feedback']['hand']['position_unit'] = result[side]['hand_unit']
            result = tick()
    for _ in range(3):
        result = tick()
    assert c.recovery is None
    assert anchor != InputRecovery.sources(p.solve.return_value)


def test_expired_third_frame_cannot_start_reacquisition(monkeypatch):
    c,p,client,state,clock,tick = active_fixture(monkeypatch)
    assert tick() is None; assert tick() is None
    p.check_ticket.side_effect = SourceExpired('expired during computation')
    with pytest.raises(SourceExpired): tick()
    assert c.recovery.started is None
    assert c.input_hold_started_at is not None


def test_failed_send_cannot_commit_recovery_timer(monkeypatch):
    c,p,client,state,clock,tick = active_fixture(monkeypatch)
    assert tick() is None; assert tick() is None
    client.send_target.side_effect = OSError('failed transport')
    with pytest.raises(OSError): tick()
    assert c.recovery.started is None
    assert c.input_hold_started_at is not None


def test_disabled_capability_keeps_legacy_gate(monkeypatch):
    c,p,client,state,clock,tick = active_fixture(monkeypatch)
    c.recovery_enabled = False; c.recovery = None
    for _ in range(3): assert tick() is None
    assert c.input_resume_count == 0


def test_first_input_accepts_moderate_planning_time_motion_without_three_second_timer(monkeypatch):
    c,p,client,state,clock,tick=setup(monkeypatch)
    c.recovery_enabled=True
    ticket=p.body.ticket.return_value;p.body.ticket.return_value=None
    tick();c.hold_missing_input(state,'first input gap')
    p.body.ticket.return_value=ticket
    p.solve.return_value['left']['arm_urdf_rad'][0]=np.deg2rad(8)
    p.solve.return_value['left']['arm_urdf_rad'][4]=np.deg2rad(12)
    assert tick() is None;assert tick() is None
    target=tick()
    assert target is not None and c.recovery is None
    assert c.wrist_handover is not None and not c.wrist_handover.done
    assert client.recovery_metadata is None
    assert abs(target['left']['arm_urdf_rad'][6])<np.deg2rad(1)


@pytest.mark.parametrize('side', ['left', 'right'])
def test_real_robot_trajectory_brakes_then_reacquires_without_velocity_reset(side):
    from esrobo_teleop.config import load_config
    from esrobo_teleop.robot.command_trajectory import CommandTrajectory
    from esrobo_laptop.config import WORKSPACE
    cfg=load_config(WORKSPACE/'teleoperation/config/teleop_config.yaml').robot
    trajectory=CommandTrajectory(getattr(cfg,side+'_max_joint_velocity'),
        getattr(cfg,side+'_max_joint_acceleration'),getattr(cfg,side+'_max_joint_step'),
        np.deg2rad(cfg.command_trajectory_lead_deg))
    measured=np.zeros(7);trajectory.reset(measured,10.)
    stamp=10.
    for i in range(200):
        stamp+=.02
        previous=trajectory.state
        brake=20<=i<30
        goal=previous.position if brake else np.deg2rad([0]*4+[10,10,13.33])
        candidate=trajectory.propose(goal,measured,stamp,-np.ones(7),np.ones(7),
            lambda q:np.array([q[:3]*.1,q[3:6]*.1]),.22,brake_only=brake)
        assert np.all(np.abs(candidate.velocity)<=trajectory.vmax+1e-9)
        assert np.all(np.abs(candidate.velocity-previous.velocity)<=trajectory.amax*.02+1e-9)
        assert np.all(np.abs(candidate.position-measured)<=trajectory.lead+1e-9)
        assert trajectory.state is previous
        trajectory.commit(candidate)
        measured=candidate.position.copy()
    np.testing.assert_allclose(np.degrees(measured[4:]),[10,10,13.33],atol=.05)

@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('started', [False, True])
def test_unreachable_hold_is_not_a_recovery_source_jump(monkeypatch, side, started):
    c,p,client,state,clock,tick = active_fixture(monkeypatch, side, True)
    valid = copy.deepcopy(p.solve.return_value)
    if started:
        assert tick() is None
        assert tick() is None
        assert tick() is not None
    anchor, deadline_start = copy.deepcopy(c.recovery.anchor), c.recovery.started
    commands = client.send_target.call_count
    holds = client.send_hold.call_count
    # Recorded limit-hold placeholders were 58.24 / 24.16 degrees from
    # the human-source anchor. They must never become source candidates.
    p.solve.return_value[side]['hold'] = True
    p.solve.return_value[side]['arm_urdf_rad'][4:6] = np.deg2rad([58.24, 24.16]).tolist()
    assert tick() is None
    assert client.send_target.call_count == commands
    assert client.send_hold.call_count == holds+1
    assert c.recovery.anchor == anchor and c.recovery.started == deadline_start
    assert c.recovery.count == 0 and c.input_resume_count == 0
    assert c.recovery_diagnostics['check']['candidate'] is None
    assert 'source_timestamps' in c.recovery_diagnostics['check']
    p.solve.return_value = valid
    assert tick() is None
    assert tick() is None
    assert tick() is not None


@pytest.mark.parametrize('started', [False, True])
def test_unreachable_frames_cannot_extend_recovery_deadlines(monkeypatch, started):
    c,p,client,state,clock,tick = active_fixture(monkeypatch)
    if started:
        for _ in range(3): tick()
    original_start = c.recovery.started
    original_anchor = copy.deepcopy(c.recovery.anchor)
    p.solve.return_value['left']['hold'] = True
    assert tick() is None
    hold_start = c.input_hold_started_at
    assert tick() is None
    assert c.input_hold_started_at == hold_start
    assert c.recovery.anchor == original_anchor
    assert c.recovery.started == original_start
    clock[0] = (original_start+3.01 if started else hold_start+.501)
    with pytest.raises(RecoveryRejected, match='3 s' if started else '500 ms'):
        tick()
