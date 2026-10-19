"""Offline reproduction of 2026-10-16 moving recovery and blocked fingers."""
import copy
from types import SimpleNamespace
from pathlib import Path
import json

import numpy as np
import pytest

from esrobo_laptop.hand_handover import HandHandover
from esrobo_laptop.input_recovery import InputRecovery, RecoveryRejected
from esrobo_laptop.config import Settings
from esrobo_laptop.inputs import InputUnavailable
from test_input_recovery import active_fixture


def cfg():
    return SimpleNamespace(max_velocity=400, max_step=18, publish_hz=50,
                           enabled_physical_joints=[0,1,2,3,4,5,9], live_feedback_lead=30)


def test_recorded_24_unit_gap_is_bounded_and_release_is_possible():
    h=HandHandover([166]*10,cfg(),0.,50,live_goal=True)
    candidate=h.propose([182]*10,[190]*10,.02)
    assert candidate.position[3]==174
    h.commit(candidate)
    assert not h.reversal_wait
    for i in range(2,10):
        # Closing direction decreases physical units; keep the finger blocked.
        h.commit(h.propose([0]*10,[190]*10,i*.02))
    assert h.command[3]==160
    # Opening toward the actual position must not wait for an impossible grasp arrival.
    h.commit(h.propose([255]*10,[190]*10,.2))
    assert h.command[3]==168
    assert not h.reversal_wait


def test_backdrive_backoff_keeps_rate_and_reduces_error():
    h=HandHandover([160]*10,cfg(),0.,50,live_goal=True)
    p=h.propose([0]*10,[205]*10,.02)
    assert p.position[3]==168 and abs(p.position[3]-205)<abs(160-205)
    h.commit(p)
    before=h.command.copy()
    with pytest.raises(RuntimeError,match='feedback diverged'):
        h.propose([0]*10,[240]*10,.04)
    np.testing.assert_array_equal(h.command,before)


def test_blocked_finger_long_session_does_not_accumulate_pressure_target():
    h=HandHandover([200]*10,cfg(),0.,50,live_goal=True)
    for i in range(1,3001):
        p=h.propose([0]*10,[200]*10,i*.02)
        assert max(abs(np.array(p.position)[h.enabled]-h.command[h.enabled]))<=8
        assert max(abs(np.array(p.position)[h.enabled]-200))<=30
        h.commit(p)
    assert h.command[3]==170
    assert h.command[6:9].tolist()==[200]*3
    h.hold([200]*10,70.)
    p=h.propose([0]*10,[200]*10,70.001)
    assert p.position[3]==200  # no stored motion credit


@pytest.mark.parametrize('side',['left','right'])
@pytest.mark.parametrize('with_hand',[False,True])
def test_controller_recovery_completes_while_human_moves_and_hand_is_blocked(monkeypatch,side,with_hand):
    c,p,client,state,clock,tick=active_fixture(monkeypatch,side,with_hand)
    for _ in range(8):
        p.solve.return_value[side]['arm_urdf_rad'][0]+=np.deg2rad(.2)
        if with_hand:p.solve.return_value[side]['hand_unit']=[0]*10
        result=tick()
    assert c.recovery is None and client.recovery_metadata['phase']=='complete'
    # Robot feedback deliberately never follows the human; no arrival gate.
    for _ in range(550):
        result=tick()
    assert c.recovery is None
    if with_hand:
        h=c.recovery_hands[side]
        actual=state['feedback']['hand']['position_unit']
        assert h.lead is None
        assert result[side]['hand_unit'][3] == 0
        p.solve.return_value[side]['hand_unit']=[255]*10
        before=h.command.copy();tick()
        assert h.command[3]>before[3] and h.command[3]-before[3]<=8


@pytest.mark.parametrize('fault',['controller_fault','driver_fault','enable_states'])
def test_recovery_completion_still_requires_health(monkeypatch,fault):
    c,p,client,state,clock,tick=active_fixture(monkeypatch,'left',True)
    tick();tick();tick()
    state['feedback'][fault]=[False]*7 if fault=='enable_states' else {'fault':'injected'}
    count=client.send_target.call_count
    with pytest.raises(RecoveryRejected,match='healthy'):
        tick()
    assert client.send_target.call_count==count


def test_live_limiter_proposal_is_not_committed_on_send_failure(monkeypatch):
    c,p,client,state,clock,tick=active_fixture(monkeypatch,'left',True)
    for _ in range(8):tick()
    h=c.recovery_hands['left'];before=copy.deepcopy(vars(h))
    p.solve.return_value['left']['hand_unit']=[0]*10
    client.send_target.side_effect=OSError('offline send failure')
    with pytest.raises(OSError):tick()
    for name,value in before.items():np.testing.assert_equal(getattr(h,name),value)


@pytest.mark.parametrize('component',['arm','hand'])
def test_stale_feedback_rejects_even_after_recovery_completion(monkeypatch,component):
    c,p,client,state,clock,tick=active_fixture(monkeypatch,'left',True)
    for _ in range(8):tick()
    if component=='arm':state['feedback']['arm_feedback_age_s']=1.
    else:state['feedback']['hand']['age_s']=1.
    count=client.send_target.call_count
    with pytest.raises(InputUnavailable,match='stale'):tick()
    assert client.send_target.call_count==count


@pytest.mark.parametrize('suffix',['6019269569381','6220133234353','6515921626074'])
def test_recorded_moving_recovery_completes_before_original_deadline(suffix):
    root=Path(__file__).parent/'fixtures/moving_recovery'
    records=[json.loads(l) for l in (root/f'session_179213{suffix}.jsonl').open()]
    records=[d for d in records if d.get('status')=='REACQUIRING']
    s=Settings(side='left',with_hand=True);r=InputRecovery(s)
    first=records[0];r.begin(first['targets']);ended=None
    for d in records:
        source=d['input_recovery']['check']['candidate']['left']
        target={'left':dict(arm_urdf_rad=source['arm'],source_arm_urdf_rad=source['arm'],hand_unit=source['hand'])}
        if r.evaluate(target,d['monotonic_s']):
            if r.started is not None and r.confirm_tracking(target,{'left':d['feedback']},cfg().enabled_physical_joints):
                ended=d['monotonic_s'];break
            r.commit(d['monotonic_s'])
    assert ended is not None and ended-first['monotonic_s']<.5


@pytest.mark.parametrize('actual,goal', [(255,0),(0,255),(190,182)])
def test_default_live_mode_has_no_position_error_limit(actual,goal):
    from esrobo_teleop.config import HandConfig
    h=HandHandover([actual]*10,HandConfig(),0.,50,live_goal=True)
    assert h.lead is None
    for i in range(1,601):
        previous=h.command.copy()
        h.commit(h.propose([goal]*10,[actual]*10,i*.02))
        assert max(abs(h.command[h.enabled]-previous[h.enabled]))<=8
    assert all(h.command[i]==goal for i in h.enabled)
    # Full 255-unit mismatch and feedback rebound do not reject a live command.
    h.commit(h.propose([255-goal]*10,[255-actual]*10,12.02))
    assert max(abs(h.command[h.enabled]-goal))<=8
    assert not h.reversal_wait


def test_null_live_lead_preserves_initial_arrival_and_divergence_guards():
    from esrobo_teleop.config import HandConfig
    h=HandHandover([200]*10,HandConfig(),0.,50)
    assert h.lead==15
    for i in range(1,20):h.commit(h.propose([0]*10,[200]*10,i*.02))
    assert h.command[3]==185 and not h.done
    with pytest.raises(RuntimeError,match='feedback diverged'):
        h.propose([0]*10,[255]*10,.4)
    with pytest.raises(RuntimeError,match='exceeded 10 s'):
        h.propose([0]*10,[200]*10,10.01)
