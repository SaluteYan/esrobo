"""J6/J7 stability: source clocks, bounded motion and actual send/commit boundaries."""
import copy
from dataclasses import replace
import time
from unittest.mock import Mock

import numpy as np
import pytest

from esrobo_laptop.wrist_joint_filter import WristJointFilter
from esrobo_teleop.config import RetargetConfig, HandConfig
from esrobo_laptop.inputs import SourceExpired, InputUnavailable


def make(**kwargs):
    return WristJointFilter([0., 0.], replace(RetargetConfig(), **kwargs),
                            np.deg2rad([-38, -80]), np.deg2rad([50, 80]))


def advance(f, goal, stamp):
    p=f.propose(np.deg2rad(goal), stamp);f.commit(p)
    return np.degrees(p.output)


def test_formula_and_candidate_has_no_side_effects():
    f=make();advance(f,[0,0],0)
    p=f.propose(np.deg2rad([10,-10]),.02)
    fc=min(4,1+.2*np.deg2rad(10)/.02)
    expected=10*(-np.expm1(-2*np.pi*fc*.02))-1
    assert np.degrees(p.output)==pytest.approx([expected,-expected])
    assert f.stamp==0 and np.array_equal(f.output,[0,0])
    assert p==f.propose(np.deg2rad([10,-10]),.02)


@pytest.mark.parametrize('hz',[25,50,90])
def test_constant_cutoff_uses_time_not_frame_count(hz):
    f=make(wrist_joint_filter_speed_coefficient=0,wrist_joint_filter_deadband_deg=0)
    advance(f,[0,0],0)
    for i in range(1,hz+1):out=advance(f,[10,-10],i/hz)
    assert out==pytest.approx(np.array([10,-10])*(1-np.exp(-2*np.pi)))


def test_quiet_input_constant_output_and_slow_ramp_remains_live():
    f=make();advance(f,[0,0],0)
    for i in range(1,101):assert advance(f,[.8*(-1)**i,.4],i*.02)==pytest.approx([0,0])
    outputs=[]
    for i in range(1,501):outputs.append(advance(f,[2*i*.02,-2*i*.02],2+i*.02))
    out=np.array(outputs);lag=(20-out[-1,0])/2
    assert .60<lag<.75
    assert np.all(np.diff(out[:,0])>=-1e-12) and out[-1,0]>18.5
    # Reversal must eventually move, without a deadband-sized output jump.
    reverse=[]
    for i in range(1,251):reverse.append(advance(f,[20-2*i*.02,-20+2*i*.02],12+i*.02))
    assert reverse[-1][0]<12 and np.max(abs(np.diff(reverse,axis=0)))<.1


def test_threshold_crossing_is_continuous_and_no_constant_target_creep():
    f=make();advance(f,[0,0],0)
    results=[]
    for i in range(1,501):results.append(advance(f,[i*.005,0],i*.02)[0])
    assert 1<results[-1]<1.5
    assert max(np.diff(results))<.006
    for i in range(501,1001):advance(f,[2.5,0],i*.02)
    frozen=f.output.copy()
    for i in range(1001,1051):advance(f,[2.5,0],i*.02)
    np.testing.assert_allclose(f.output,frozen,atol=1e-10)


@pytest.mark.parametrize('stamp',[10.,9.9])
def test_duplicate_and_backwards_do_not_advance(stamp):
    f=make();advance(f,[0,0],10.)
    p=f.propose(np.deg2rad([30,60]),stamp);f.commit(p)
    assert f.stamp==10 and np.array_equal(f.output,[0,0])
    assert np.array_equal(f.smoothed,[0,0])


def test_gap_only_reseeds_clock_no_bank_and_bounds_do_not_wrap():
    f=make();advance(f,[0,0],10.)
    assert advance(f,[50,-80],11.)==pytest.approx([0,0])
    assert np.all(np.abs(advance(f,[50,-80],11.02))<[50,80])
    for i in range(1,501):out=advance(f,[-38,80],11.02+i*.02)
    assert out==pytest.approx([-37,79],abs=1e-7)
    with pytest.raises(ValueError):f.propose(np.deg2rad([360,0]),22.)
    with pytest.raises(ValueError):f.propose([0,0],float('nan'))


@pytest.mark.parametrize('field,value', [('wrist_joint_filter_min_cutoff_hz',0),
    ('wrist_joint_filter_max_cutoff_hz',.5),('wrist_joint_filter_speed_coefficient',-1),
    ('wrist_joint_filter_deadband_deg',float('nan'))])
def test_bad_parameters_rejected(field,value):
    with pytest.raises(ValueError):make(**{field:value})


def fixture(monkeypatch,side='right',preview=False):
    from test_entry_keepalive import fixed_controller
    from esrobo_link.protocol import canonical
    c,p,client,state=fixed_controller('ACTIVE')
    clock=[100.];monkeypatch.setattr(time,'monotonic',lambda:clock[0])
    c.settings=replace(c.settings,side=side)
    state['contract']['side']=side;c.contract=canonical(state['contract'])
    c.mode='ACTIVE';c.preview=preview
    p.cfg.retarget=RetargetConfig();p.cfg.hand=HandConfig()
    p.trace_context={'capture_id':1};p.motion_trace=Mock()
    baseline=dict(arm_urdf_rad=[.2]*7,hand_unit=None,hold=False)
    c.last_active_target={side:copy.deepcopy(baseline)}
    p.solve.return_value={side:copy.deepcopy(baseline)}
    def tick(dt=.02):
        clock[0]+=dt;client.received_at=clock[0]
        frame=({side+'_arm':clock[0],side+'_palm':clock[0]},
               {'hand_local_orientations':{side:np.eye(3)}},None)
        p.capture.return_value=frame
        return c.step(state)
    return c,p,client,state,clock,tick


@pytest.mark.parametrize('side',['left','right'])
def test_live_filter_preserves_other_joints_hand_and_raw_source(monkeypatch,side):
    c,p,client,state,clock,tick=fixture(monkeypatch,side)
    tick();old=p.solve.return_value[side]
    old['arm_urdf_rad']=[.1,.2,.3,.4,.5,.4,-.4]
    old['source_arm_urdf_rad']=[.15]*7
    before=copy.deepcopy(old);out=tick()[side]
    assert out['arm_urdf_rad'][:5]==before['arm_urdf_rad'][:5]
    assert out['hand_unit']==before['hand_unit']
    assert out['source_arm_urdf_rad']==before['source_arm_urdf_rad']
    assert p.solve.return_value[side]==before
    assert out['arm_urdf_rad'][5:]!=before['arm_urdf_rad'][5:]
    assert all(type(x) is float for x in out['arm_urdf_rad'])
    assert c.wrist_joint_filters[side].stamp==clock[0]


@pytest.mark.parametrize('failure',['send','expired','feedback','hold'])
def test_failed_candidates_never_commit(monkeypatch,failure):
    c,p,client,state,clock,tick=fixture(monkeypatch);tick()
    f=c.wrist_joint_filters['right'];before=copy.deepcopy(f)
    p.solve.return_value['right']['arm_urdf_rad'][5:]=[.5,-.5]
    if failure=='send':client.send_target.side_effect=OSError('transport')
    elif failure=='expired':p.check_ticket.side_effect=SourceExpired('expired')
    elif failure=='feedback':state['feedback']['arm_feedback_age_s']=1
    else:p.solve.return_value['right']['hold']=True
    if failure=='hold':tick()
    else:
        with pytest.raises((OSError,InputUnavailable)):tick()
    assert f.stamp==before.stamp
    np.testing.assert_array_equal(f.output,before.output)
    np.testing.assert_array_equal(f.smoothed,before.smoothed)


def test_preparation_return_and_non_pico_paths_bypass(monkeypatch):
    c,p,client,state,clock,tick=fixture(monkeypatch);tick()
    state['mode']='RETURNING';assert tick() is None
    assert not c.wrist_joint_filters
    targets=copy.deepcopy(p.solve.return_value)
    out,proposals=c._propose_wrist_joint_filters(({}, {},None),targets,{})
    assert out==targets and not proposals
    state['mode']='CALIBRATING';c.preparing=True;c.entry_target=copy.deepcopy(targets)
    c.entry_reference_source=[0]*9
    assert tick()==targets and not c.wrist_joint_filters


def test_missing_palm_clock_rejected_and_preview_separate(monkeypatch):
    c,p,client,state,clock,tick=fixture(monkeypatch,preview=True)
    tick();assert c.wrist_joint_filters['right'].stamp==clock[0]
    client.send_target.assert_not_called()
    with pytest.raises(InputUnavailable,match='paired PICO'):
        c._propose_wrist_joint_filters(({}, {'hand_local_orientations':{'right':np.eye(3)}},None),
            p.solve.return_value,{'right':state['feedback']})


def test_both_sides_commit_independently_and_hold_does_not_commit(monkeypatch):
    c,p,client,state,clock,tick=fixture(monkeypatch);c.settings=replace(c.settings,side='both',with_hand=True)
    targets={s:dict(arm_urdf_rad=[.1]*7,hand_unit=[128]*10,hold=False) for s in c.settings.sides}
    feedback=copy.deepcopy(targets);frame=({s+'_palm':100. for s in targets},
        {'hand_local_orientations':{s:np.eye(3) for s in targets}},None)
    out,candidates=c._propose_wrist_joint_filters(frame,targets,feedback)
    assert not c.wrist_joint_filters
    c._commit_wrist_joint_filters(candidates,out)
    assert set(c.wrist_joint_filters)=={'left','right'}
    frame[0]['left_palm']=100.02;frame[0]['right_palm']=100.04
    targets['right']['hold']=True;targets['left']['arm_urdf_rad'][5:]=[.4,.4]
    out,candidates=c._propose_wrist_joint_filters(frame,targets,feedback)
    c._commit_wrist_joint_filters(candidates,out)
    assert c.wrist_joint_filters['left'].stamp==100.02
    assert c.wrist_joint_filters['right'].stamp==100.


@pytest.mark.parametrize('side',['left','right'])
@pytest.mark.parametrize('field',['wrist_joint_filter_min_cutoff_hz','wrist_joint_filter_max_cutoff_hz',
                                 'wrist_joint_filter_speed_coefficient','wrist_joint_filter_deadband_deg'])
def test_fingerprint_includes_new_settings(side,field):
    from esrobo_laptop.config import ROOT,read_settings,read_robot_config
    from esrobo_laptop.contract import local_id
    cfg=read_robot_config(read_settings(ROOT/'config/laptop.yaml'));old=local_id(cfg,side,True)
    setattr(cfg.retarget,field,getattr(cfg.retarget,field)+.1)
    assert local_id(cfg,side,True)!=old


def test_input_recovery_keeps_filter_until_successful_third_frame(monkeypatch):
    c,p,client,state,clock,tick=fixture(monkeypatch)
    c.recovery_enabled=True;state['feedback']['enable_states']=[True]*7
    tick();before=copy.deepcopy(c.wrist_joint_filters['right'])
    clock[0]+=.15;client.received_at=clock[0]
    assert c._send_input_hold(state,'synthetic gap')
    p.solve.return_value['right']['arm_urdf_rad'][5:]=[.5,-.5]
    for _ in range(2):
        assert tick() is None
        f=c.wrist_joint_filters['right']
        assert f.stamp==before.stamp
        np.testing.assert_array_equal(f.output,before.output)
    result=tick()
    assert result is not None
    f=c.wrist_joint_filters['right']
    assert f.stamp==clock[0]
    np.testing.assert_array_equal(f.output,before.output)  # long gap clocks only
    assert tick()['right']['arm_urdf_rad'][5:]!=list(before.output)
    assert c.last_arm_source['right'][5:]==[.5,-.5]


def test_first_active_uses_arrived_target_and_legacy_transition_is_preserved(monkeypatch):
    c,p,client,state,clock,tick=fixture(monkeypatch)
    p.solve.return_value['right']['arm_urdf_rad'][5:]=[.6,-.6]
    out=tick()
    assert out['right']['arm_urdf_rad'][5:]==[.2,.2]
    from esrobo_laptop.wrist_handover import WristHandover
    c.wrist_joint_filters.clear()
    c.wrist_handover=WristHandover([.2]*7,p.cfg.robot,'right',clock[0],50)
    out=tick()
    assert not c.wrist_joint_filters
    assert max(abs(np.asarray(out['right']['arm_urdf_rad'][5:])-.2))<.02


def test_only_original_palm_clock_advances_joint_filter(monkeypatch):
    c,p,client,state,clock,tick=fixture(monkeypatch);tick()
    f=c.wrist_joint_filters['right'];stamp=f.stamp
    goal=copy.deepcopy(p.solve.return_value);goal['right']['arm_urdf_rad'][5:]=[.5,-.5]
    # New arm/receive clock, same paired palm: no extra smoothing step.
    frame=({'right_arm':stamp+.04,'right_palm':stamp},
           {'hand_local_orientations':{'right':np.eye(3)}},None)
    out,candidates=c._propose_wrist_joint_filters(frame,goal,{'right':state['feedback']})
    c._commit_wrist_joint_filters(candidates,out)
    assert f.stamp==stamp
    np.testing.assert_array_equal(f.output,[.2,.2])


def test_dual_trace_batches_sides_and_keeps_capture_join(monkeypatch):
    c,p,client,state,clock,tick=fixture(monkeypatch);c.settings=replace(c.settings,side='both',with_hand=True)
    targets={s:dict(arm_urdf_rad=[.1]*7,hand_unit=[128]*10,hold=False) for s in c.settings.sides}
    frame=({s+'_palm':100. for s in targets},
           {'hand_local_orientations':{s:np.eye(3) for s in targets}},None)
    result,candidates=c._propose_wrist_joint_filters(frame,targets,targets)
    c._commit_wrist_joint_filters(candidates,result)
    calls=p.motion_trace.emit.call_args_list
    assert len(calls)==2
    assert calls[0].kwargs['context']=={'capture_id':1}
    assert set(calls[0].kwargs['sides'])=={'left','right'}
    assert not calls[0].kwargs['committed'] and not calls[0].kwargs['sent']
    assert calls[1].kwargs['committed'] and calls[1].kwargs['sent']
    assert calls[1].kwargs['sides']['right']['final_target_rad']==[.2,.2]


@pytest.mark.parametrize('send_failure', [False, True])
def test_first_gap_large_source_keeps_filter_transaction_and_arrived_seed(monkeypatch, send_failure):
    c,p,client,state,clock,tick = fixture(monkeypatch)
    c.recovery_enabled = True
    c.mode = 'WAITING_FOR_ACTIVE_TARGET'
    c.entry_ready_sent_at = clock[0]
    c.entry_token = state['entry_token']
    state['feedback']['enable_states'] = [True]*7
    ticket = p.body.ticket.return_value
    p.body.ticket.return_value = None
    assert tick() is None
    assert c.hold_missing_input(state, 'first gap')
    original_deadline = c.input_hold_started_at
    p.body.ticket.return_value = ticket
    p.solve.return_value['right']['arm_urdf_rad'][5:] = [.6, -.6]
    for _ in range(2):
        assert tick() is None
        assert not c.wrist_joint_filters
    if send_failure:
        client.send_target.side_effect = OSError('injected send failure')
        with pytest.raises(OSError, match='send failure'):
            tick()
        assert c.entry_active_pending
        assert c.input_hold_started_at == original_deadline
        assert not c.wrist_joint_filters
        p.reset_command_start.assert_called_once()
        return
    result = tick()
    assert result['right']['arm_urdf_rad'][5:] == [.2, .2]
    assert c.last_arm_source['right'][5:] == [.6, -.6]
    assert not c.entry_active_pending
    assert c.input_hold_started_at is None
    for _ in range(10):
        result = tick()
    assert result['right']['arm_urdf_rad'][5] > .2
    assert result['right']['arm_urdf_rad'][6] < .2
    p.reset_command_start.assert_called_once()
