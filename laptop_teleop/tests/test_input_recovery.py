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


def test_dual_accepts_large_targets_and_adjacent_changes():
    s = Settings()
    r = InputRecovery(s); r.begin(targets(s))
    value = targets(s, wrist=40, proximal=30)
    for i in range(3):
        assert r.evaluate(value, 1.+i*.02) == (i == 2)
    value['right']['arm_urdf_rad'][6] += np.deg2rad(6)
    assert r.evaluate(value, 1.06)
    assert r.diagnostic['allowed'] and r.count == 4
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
def test_recorded_excursion_is_diagnostic_only(component):
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
    for i in range(3):
        assert r.evaluate(candidate, 1.+i*.02) == (i == 2)
    assert r.diagnostic['limits']['left']['arm_limit_deg'] is None
    if component in ('arm', 'both'):
        assert r.diagnostic['limits']['left']['arm_delta_deg'][5] == pytest.approx(31.004856)
    assert r.started is None and r.anchor == InputRecovery.sources(anchor)


def test_adjacent_large_jump_counts_as_a_valid_frame():
    s = Settings(side='left', with_hand=False)
    r = InputRecovery(s); r.begin(targets(s))
    assert not r.evaluate(targets(s, wrist=6), 1.)
    assert not r.evaluate(targets(s, wrist=14), 1.02)
    assert r.count == 2 and r.diagnostic['allowed']
    assert r.evaluate(targets(s, wrist=25), 1.04)
    assert r.evaluate(targets(s, wrist=14), 1.06)


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('axis', range(7))
@pytest.mark.parametrize('with_hand', [False, True])
def test_all_axes_accept_large_recovery_changes_before_and_after_start(monkeypatch, side, axis, with_hand):
    c,p,client,state,clock,tick = active_fixture(monkeypatch, side, with_hand)
    for i in range(6):
        source = np.asarray(c.last_arm_source[side]).copy()
        source[axis] += np.deg2rad(8)
        p.solve.return_value[side]['source_arm_urdf_rad'] = source.tolist()
        result = tick()
        assert (result is None) == (i < 2)
    assert c.recovery is None


def test_recorded_j5_jump_qualifies_and_finishes_recovery(monkeypatch):
    c,p,client,state,clock,tick = active_fixture(monkeypatch, 'left', True)
    # Actual source sequence from session_1792153871933010326; last delta
    # 5.884 degrees previously faulted after two successful recovery checks.
    for i,angle in enumerate([33.333,34.846,35.061,36.196,35.950,41.834]):
        source = np.asarray(c.last_arm_source['left']).copy()
        source[4] = np.deg2rad(angle)
        p.solve.return_value['left']['source_arm_urdf_rad'] = source.tolist()
        result = tick(.01934)
        assert (result is None) == (i < 2)
    assert c.recovery is None
    assert c.recovery_completion['phase'] == 'complete'


def test_duplicate_input_cannot_qualify_recovery(monkeypatch):
    c,p,client,state,clock,tick = active_fixture(monkeypatch)
    assert tick() is None
    count = c.recovery.count
    sends = client.send_target.call_count
    p.body.ticket.return_value = None
    for _ in range(3): assert tick() is None
    assert c.recovery.count == count
    assert client.send_target.call_count == sends


def test_removed_recovery_angle_parameter_is_not_a_setting():
    assert 'input_resume_arm_delta_deg' not in Settings.__dataclass_fields__


@pytest.mark.parametrize('side', ['left', 'right', 'both'])
@pytest.mark.parametrize('with_hand', [False, True])
def test_large_stable_arm_excursion_qualifies_without_anchor_limit(side, with_hand):
    s = Settings(side=side, with_hand=with_hand)
    r = InputRecovery(s); r.begin(targets(s))
    value = targets(s, wrist=45, proximal=30)
    for i in range(3):
        assert r.evaluate(value, 1.+i*.02) == (i == 2)
    assert all(item['arm_limit_deg'] is None for item in r.diagnostic['limits'].values())
    r.commit(1.04)
    # Moving gradually farther from the anchor is allowed; pauses must not
    # reset the time bound even when displacement no longer has a bound.
    assert r.evaluate(targets(s, wrist=49, proximal=34), 1.06)
    r.pause()
    with pytest.raises(RecoveryRejected, match='3 s'):
        r.evaluate(value, 4.05)


@pytest.mark.parametrize('bad', [[0]*6, [0]*8, [float('nan')]*7, [float('inf')]*7])
def test_arm_recovery_rejects_invalid_sources_without_anchor_gate(bad):
    s = Settings(side='left', with_hand=False)
    r = InputRecovery(s); r.begin(targets(s))
    value = targets(s)
    # A valid filtered command cannot conceal an invalid unfiltered source.
    value['left']['source_arm_urdf_rad'] = bad
    with pytest.raises(RecoveryRejected, match='seven finite arm positions'):
        r.evaluate(value, 1.)
    assert r.count == 0 and r.started is None


def active_fixture(monkeypatch, side='left', with_hand=False):
    c,p,client,state,clock,tick = setup(monkeypatch, side, with_hand)
    c.recovery_enabled = True
    # Complete first entry with matching measured targets, then create a gap.
    p.solve.return_value = copy.deepcopy(c.entry_target)
    for _ in range(5):
        result = tick()
        state['feedback']['arm_urdf_rad'] = result[side]['arm_urdf_rad']
    # Protocol 7 completes the wrist during APPROACHING, before ACTIVE.
    assert c.wrist_handover is None or c.wrist_handover.done
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


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('with_hand', [False, True])
def test_controller_large_recovery_accepts_jumps_and_keeps_timeout(monkeypatch, side, with_hand):
    c,p,client,state,clock,tick = active_fixture(monkeypatch, side, with_hand)
    source = np.asarray(c.recovery.anchor[side]['arm']) + np.deg2rad([30]*4+[40]*3)
    p.solve.return_value[side]['source_arm_urdf_rad'] = source.tolist()
    assert tick() is None; assert tick() is None
    assert tick() is not None
    assert client.recovery_metadata['phase'] == 'reacquiring'
    commands = client.send_target.call_count
    source[2] += np.deg2rad(6)
    p.solve.return_value[side]['source_arm_urdf_rad'] = source.tolist()
    assert tick() is not None
    assert client.send_target.call_count == commands+1
    commands += 1
    clock[0] = c.recovery.started + 3.01
    with pytest.raises(RecoveryRejected, match='3 s'):
        tick()
    assert client.send_target.call_count == commands


def test_disabled_capability_has_no_angle_gate(monkeypatch):
    c,p,client,state,clock,tick = active_fixture(monkeypatch)
    c.recovery_enabled = False; c.recovery = None
    assert tick() is None; assert tick() is None
    assert tick() is not None
    assert c.recovery is None


def test_first_input_accepts_moderate_planning_time_motion_without_three_second_timer(monkeypatch):
    c,p,client,state,clock,tick=setup(monkeypatch)
    c.recovery_enabled=True
    ticket=p.body.ticket.return_value;p.body.ticket.return_value=None
    tick();c.hold_missing_input(state,'first input gap')
    p.body.ticket.return_value=ticket
    p.solve.return_value = copy.deepcopy(c.entry_target)
    p.solve.return_value['left']['arm_urdf_rad'][0]=np.deg2rad(8)
    p.solve.return_value['left']['arm_urdf_rad'][4]=np.deg2rad(12)
    assert tick() is None;assert tick() is None
    target=tick()
    assert target is not None and c.recovery is None
    assert c.wrist_handover is None  # protocol 7 wrist already arrived
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
        goal=previous.position if brake else np.deg2rad([30]*4+[40]*3)
        candidate=trajectory.propose(goal,measured,stamp,-np.ones(7),np.ones(7),
            lambda q:np.array([q[:3]*.1,q[3:6]*.1]),.22,brake_only=brake)
        assert np.all(np.abs(candidate.velocity)<=trajectory.vmax+1e-9)
        assert np.all(np.abs(candidate.velocity-previous.velocity)<=trajectory.amax*.02+1e-9)
        assert np.all(np.abs(candidate.position-measured)<=trajectory.lead+1e-9)
        assert trajectory.state is previous
        trajectory.commit(candidate)
        measured=candidate.position.copy()
    np.testing.assert_allclose(np.degrees(measured),[30]*4+[40]*3,atol=.05)

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


@pytest.mark.parametrize('side,hand_only', [('left', False), ('right', False), ('both', False), ('left', True), ('right', True)])
def test_hand_recovery_accepts_full_range_and_adjacent_changes(side, hand_only):
    s = Settings(side=side, with_hand=True, hand_only=hand_only)
    r = InputRecovery(s); r.begin(targets(s, hand=0))
    for i, hand in enumerate([255, 0, 255]):
        assert r.evaluate(targets(s, hand=hand), 1.+i*.02) == (i == 2)
    assert r.diagnostic['allowed']
    assert all(v['hand_limit_unit'] is None for v in r.diagnostic['limits'].values())


@pytest.mark.parametrize('bad', [[0]*9, [float('nan')]*10, [float('inf')]*10, [-1]*10, [256]*10])
def test_hand_recovery_still_rejects_invalid_positions(bad):
    s = Settings(side='left', with_hand=True)
    r = InputRecovery(s); r.begin(targets(s))
    candidate = targets(s); candidate['left']['hand_unit'] = bad
    with pytest.raises(RecoveryRejected, match='ten valid hand positions'):
        r.evaluate(candidate, 1.)


@pytest.mark.parametrize('side', ['left', 'right'])
def test_large_finger_recovery_uses_latest_goal_and_bounded_commands(monkeypatch, side):
    c,p,client,state,clock,tick = active_fixture(monkeypatch, side, True)
    # No arm excursion: even ordinary arm recovery must keep the hand ramp.
    p.solve.return_value[side]['arm_urdf_rad'] = copy.deepcopy(c.recovery.anchor[side]['arm'])
    state['feedback']['arm_urdf_rad'] = copy.deepcopy(p.solve.return_value[side]['arm_urdf_rad'])
    goal = p.solve.return_value[side]['hand_unit']
    goal[2:6] = [249,255,255,255]
    assert tick() is None; assert tick() is None
    result = tick()
    assert c.recovery is not None and c.recovery.started is not None
    limiter = c.recovery_hands[side]
    enabled = limiter.enabled
    actual = np.asarray(state['feedback']['hand']['position_unit'])
    assert limiter.lead is None
    assert np.max(np.abs(np.asarray(result[side]['hand_unit'])[enabled]-actual[enabled])) <= 8
    assert result[side]['hand_unit'][2:6] != goal[2:6]
    for i in range(100):
        previous = np.asarray(result[side]['hand_unit'])
        state['feedback']['hand']['position_unit'] = previous.tolist()
        if i == 2: goal[2:6] = [0,0,0,0]  # new qualified target replaces old goal
        result = tick()
        command = np.asarray(result[side]['hand_unit'])
        assert np.max(np.abs(command[enabled]-previous[enabled])) <= min(limiter.step, limiter.velocity*.02)+1e-7
        # Input recovery can finish before the mechanical hand; its limiter persists.
        assert c.recovery_hands[side] is limiter
        if c.recovery is None and np.max(np.abs(command[2:6]-goal[2:6])) <= 2: break
    assert c.recovery is None
    np.testing.assert_allclose(command[2:6], goal[2:6], atol=2)


def run_gap_main(monkeypatch, tmp_path, events, *, side='left', with_hand=False):
    """Run the real CLI loop/exception handler with only I/O and IK simulated."""
    import io
    import time
    from unittest.mock import Mock
    import esrobo_laptop.app as app
    import esrobo_teleop.debug.motion_trace as traces
    c,p,client,state,clock,tick = setup(monkeypatch, side, with_hand)
    c.recovery_enabled = True
    tick()  # A successfully sent live target, before the recorded input gap.
    start = clock[0]
    before_sent = c.sent
    client.send_target.reset_mock()
    p.reset_command_start.reset_mock()
    p.body.stamps = {}
    p.body.hand_sequences = {}
    p.body.receive_counts = {}
    p.body.pico_hand_status = {}
    p.body.pico_hand_status_at = None
    p.body.pico_hand_targets = {}
    p.body.hand_retarget_diagnostics = {}
    p.body.last_rejection = ''
    p.body.required = ()
    p.body.source_rates.return_value = {}
    p.body.wrist_diagnostics.return_value = {}
    p.wrists = {}; p.hand_references = {}
    journal = Mock()
    trace = Mock(); trace.stats.return_value = {}
    monkeypatch.setattr(app, 'RuntimeDiagnostics', lambda *a, **kw: journal)
    monkeypatch.setattr(traces, 'MotionTrace', lambda *a, **kw: trace)
    monkeypatch.setattr(traces, 'metadata', lambda *a, **kw: {})
    monkeypatch.setattr(app, 'read_settings', lambda *a: c.settings)
    monkeypatch.setattr(app, 'read_robot_config', lambda *a: p.cfg)
    monkeypatch.setattr(app, 'verify_contract', lambda *a, **kw: {})
    monkeypatch.setattr(app, 'RobotClient', lambda *a, **kw: client)
    monkeypatch.setattr(app, 'Pipeline', lambda *a, **kw: p)
    monkeypatch.setattr(app, 'Controller', lambda *a, **kw: c)
    monkeypatch.setattr(app.sys, 'stdin', io.StringIO())
    client.connect.return_value = dict(state, mode="IDLE")
    remaining = [(0., "valid")] + list(events)
    before_sent += 1  # Seed the actual main-loop control-rate monitor as well.
    def receive(**kwargs):
        if len(remaining) == len(events):
            client.send_target.reset_mock()
        elapsed, event = remaining.pop(0)
        clock[0] = start + elapsed
        client.received_at = clock[0]
        p.capture.side_effect = None
        p.body.ticket.return_value = {side+'_arm': clock[0]}
        if event == 'pair':
            p.capture.side_effect = SourceExpired(f'{side} Body/Hand 绝对姿态采样不同步（要求 ≤40 ms）')
        elif event == 'missing':
            p.body.ticket.return_value = None
        elif event == 'invalid':
            p.capture.side_effect = app.InputUnavailable('invalid rotation')
        elif event == 'network':
            raise TimeoutError('injected receive timeout')
        elif event == 'stale_feedback':
            state['feedback']['arm_feedback_age_s'] = 1.
        elif event == 'controller_fault':
            state['feedback']['controller_fault'] = {'fault': 'injected'}
            p.capture.side_effect = SourceExpired('pair missing')
        elif event == 'disabled':
            state['feedback']['enable_states'] = [False]*7
            p.capture.side_effect = SourceExpired('pair missing')
        return state
    client.receive.side_effect = receive
    # Advancing only at receive makes the recorded intervals deterministic.
    duration = events[-1][0] + .1
    monkeypatch.setattr(time, 'sleep', lambda _: clock.__setitem__(0, start+duration+.01) if not remaining else None)
    key = tmp_path/'key';key.write_bytes(b'offline-gap-main-test-only-key-123456')
    result = app.main(['run', '--mock', '--host', '127.0.0.1', '--key-file', str(key),
                       '--seconds', str(duration), '--log-file', str(tmp_path/'session.jsonl')])
    cycles = [call.kwargs for call in trace.emit.call_args_list if call.args[0] == 'cycle']
    faults = [call.args[0] for call in journal.submit.call_args_list
              if call.args[0].get('record_type') == 'terminal_fault']
    return result,c,p,client,cycles[1:],faults,before_sent


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('with_hand', [False, True])
def test_main_recorded_pair_gap_waits_then_accepts_next_frame(monkeypatch, tmp_path, side, with_hand):
    result,c,p,client,cycles,faults,before = run_gap_main(monkeypatch,tmp_path,
        [(.0356,'pair'),(.0556,'valid')],side=side,with_hand=with_hand)
    assert result == 0 and not faults, faults
    assert len(cycles) == 2 and cycles[0]['status'].startswith('WAITING_INPUT:')
    assert not cycles[0]['held'] and not cycles[0]['input_hold']
    assert c.sent == before+1
    client.send_hold.assert_not_called()
    assert client.send_target.call_count == 1
    p.reset_command_start.assert_not_called()


def test_main_pair_gap_crosses_grace_and_resumes_after_three_valid_frames(monkeypatch,tmp_path):
    result,c,p,client,cycles,faults,before = run_gap_main(monkeypatch,tmp_path,
        [(.0356,'pair'),(.0556,'pair'),(.0756,'valid'),(.0956,'valid'),(.1156,'valid')])
    assert result == 0 and not faults, faults
    assert not cycles[0]['held']
    assert [x['held'] for x in cycles[1:]] == [True,True,True,False]
    assert c.sent == before+1 and client.send_target.call_count == 1
    assert client.send_hold.call_count == 3
    assert c.input_hold_started_at is None
    p.reset_command_start.assert_not_called()


def test_main_repeated_pair_failures_keep_original_hold_deadline(monkeypatch,tmp_path):
    result,c,p,client,cycles,faults,before = run_gap_main(monkeypatch,tmp_path,
        [(.0356,'pair'),(.0556,'pair'),(.2556,'pair'),(.4556,'pair'),(.5567,'pair')])
    assert result == 1
    assert 'hold exceeded 500 ms' in faults[-1]['status']
    assert c.sent == before and client.send_target.call_count == 0
    client.input_fault.assert_called_once()


@pytest.mark.parametrize('event', ['invalid','network','stale_feedback','controller_fault','disabled'])
def test_main_grace_cannot_hide_other_failures(monkeypatch,tmp_path,event):
    result,c,p,client,cycles,faults,before = run_gap_main(monkeypatch,tmp_path,[(.0356,event)])
    assert result == 1 and faults
    assert not cycles
    assert c.sent == before
    client.send_target.assert_not_called()
