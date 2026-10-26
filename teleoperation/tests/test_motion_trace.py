import json
from pathlib import Path
import threading
import time
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest
from esrobo_teleop.debug.motion_trace import MotionTrace, session_tag
from esrobo_link.motion_trace import command_context


def rows(path):return [json.loads(l) for l in path.read_text().splitlines()]


def test_owned_redacted_ordered_and_finite(tmp_path):
    p=tmp_path/'trace.jsonl';t=MotionTrace(p,interval=.01,enabled=True)
    x=np.arange(7,dtype=float);t.emit('sample', q=x, session='secret', nested={'token':'secret'}, invalid=float('nan'))
    x[:]=99
    assert t.close()
    data=rows(p);r=next(r for r in data if r['event']=='sample')
    assert r['q']==list(range(7)) and r['invalid'] is None
    assert 'secret' not in p.read_text()
    assert data[-1]['event']=='trace_closed'


def test_bad_payload_does_not_lose_later_samples(tmp_path):
    t=MotionTrace(tmp_path/'t',enabled=True);t.emit('bad',x=object());t.emit('good',q=[1]*7)
    assert t.close();assert t.serialization_errors==1 and t.dropped==1
    assert any(r['event']=='good' for r in rows(t.path))


def test_full_queue_never_waits_and_close_is_bounded(tmp_path,monkeypatch):
    blocked=threading.Event()
    monkeypatch.setattr(MotionTrace,'_run',lambda self:blocked.wait(2))
    t=MotionTrace(tmp_path/'t',capacity=2,enabled=True)
    try:
        start=time.monotonic()
        for i in range(1000):t.emit('sample',q=[i]*7)
        assert time.monotonic()-start<.1 and t.queue.qsize()==2 and t.dropped==999
        assert not t.close(.01) and t.shutdown_incomplete
    finally:blocked.set();t.thread.join()


def test_disk_failure_does_not_escape_producer(tmp_path):
    t=MotionTrace(tmp_path,enabled=True)
    t.thread.join(1)
    assert t.error and not t.emit('after_error') and not t.close()


def test_disabled_has_no_file(tmp_path):
    t=MotionTrace(tmp_path/'t',enabled=False)
    assert not t.emit('x');assert t.close();assert not t.path.exists()


@pytest.mark.parametrize('side',['left','right','both'])
def test_gateway_execution_context_cannot_follow_later_network_target(tmp_path,side):
    from esrobo_link.gateway import Gateway
    from esrobo_link.backends import MockBackend, MockDualBackend
    b=MockDualBackend() if side=='both' else MockBackend(side)
    g=Gateway(b,b'x'*32,port=0);g.motion_trace=MotionTrace(tmp_path/'t',enabled=True)
    consumed=command_context(dict(seq=7,session='private-session',type='target'),1.)
    g._trace_target_context=consumed
    def step(target,cancel):g._trace_target_context=command_context(dict(seq=8,session='other',type='target'),2.)
    b.step=step
    try:g._execute_traced({'hold':False},lambda:False,consumed)
    finally:g.motion_trace.close();g.socket.close()
    record=next(r for r in rows(g.motion_trace.path) if r['event']=='execution_cycle')
    assert record['context']['command_seq']==7
    assert record['context']['session_tag']==session_tag('private-session')


def test_failed_send_is_not_success_and_keeps_original_exception(tmp_path):
    from esrobo_link.client import RobotClient
    c=RobotClient.__new__(RobotClient);c.latest={'session':'private','lease':'secret','lease_ttl_s':1}
    c.received_at=time.monotonic();c.seq=0;c.recovery_metadata=None;c.trace_context=lambda:{'capture_id':12}
    c.key=b'x'*32;c.socket=Mock();c.socket.send.side_effect=OSError('simulated send')
    c.motion_trace=MotionTrace(tmp_path/'t',enabled=True)
    with pytest.raises(OSError,match='simulated send'):c._command('target',arm_urdf_rad=[0]*7)
    c.motion_trace.close();r=next(r for r in rows(c.motion_trace.path) if r['event']=='network_send')
    assert not r['sent'] and r['command_seq']==1 and r['context']['capture_id']==12


def test_pico_raw_dedup_invalid_and_receipts_incremental(tmp_path):
    from test_pico_transport import FakeSDK,forwarder,Mapper
    sdk=FakeSDK();t=MotionTrace(tmp_path/'t',enabled=True);f=forwarder(sdk,mapper=Mapper(),trace=t)
    sdk.emit(1);f._trace_samples(sdk.latest,123);f._trace_samples(sdk.latest,124)
    sdk.emit(3);sdk.latest['left']['flags'][0]=0;f._trace_samples(sdk.latest,125)
    f.snapshot();f.snapshot();t.close();r=rows(t.path)
    raw=[x for x in r if x['event']=='raw_sample'];assert len(raw)==6
    assert len(raw[1]['poses'])==26 and raw[3]['sequence_gap']==1
    assert raw[4]['flags'][0]==0
    assert len([x for x in r if x['event']=='sdk_receipts'])==3
    f.close()


@pytest.mark.parametrize('side', ['left', 'right'])
def test_observing_filter_does_not_change_outputs(side):
    from esrobo_teleop.device.body_device import BodyDevice
    from esrobo_teleop.config import RetargetConfig
    def device(enabled):
        d=BodyDevice.__new__(BodyDevice);d._cfg=RetargetConfig()
        d.motion_trace_enabled=enabled
        d._filtered_segment_directions={side:{}}
        d._filtered_segment_timestamps={side:{}}
        d._held_segment_directions={side:{}}
        return d
    off,on=device(False),device(True)
    for stamp,angle in [(1.,0.),(1.02,.002),(1.04,.2),(1.04,.3),(1.03,.1),(1.2,.01)]:
        output=[]
        for d in (off,on):
            d._last_body_frame_packet_time_monotonic=stamp
            value=d._filter_segment_direction(side,'upper_arm',np.array([np.cos(angle),np.sin(angle),0]))
            output.append(d._apply_segment_direction_deadband(side,'upper_arm',value,.7))
        np.testing.assert_array_equal(*output)
        np.testing.assert_array_equal(off._filtered_segment_directions[side]['upper_arm'],on._filtered_segment_directions[side]['upper_arm'])


def test_bad_trace_context_does_not_change_send_result(tmp_path):
    from esrobo_link.client import RobotClient
    c=RobotClient.__new__(RobotClient);c.latest={'session':'private','lease':'secret','lease_ttl_s':1}
    c.received_at=time.monotonic();c.seq=0;c.recovery_metadata=None
    c.trace_context=lambda:1/0;c.key=b'x'*32;c.socket=Mock()
    c.motion_trace=MotionTrace(tmp_path/'t',enabled=True)
    c._command('target',arm_urdf_rad=[0]*7)
    c.socket.send.assert_called_once();assert c.motion_trace.dropped==1
    c.motion_trace.close()


def test_driver_adapter_uses_cache_only_and_copies_command(tmp_path):
    from esrobo_link.motion_trace import attach,cached_feedback
    from esrobo_teleop.robot.nero_driver import JointFeedbackSnapshot
    class Arm:
        def get_joint_positions(self):raise AssertionError('extra hardware read')
    arm=Arm();arm._cycle_feedback=JointFeedbackSnapshot(np.arange(7),11.,time.monotonic(),'test-cache')
    backend=SimpleNamespace(side='left',arm=arm,_motion_trace_context={'command_seq':19})
    t=MotionTrace(tmp_path/'t',enabled=True);attach(backend,t)
    q=[1]*7;arm.probe_sink(dict(event='command_sent',command_urdf_rad=q,position_frame_sent=True))
    q[:]=[0]*7
    feedback=cached_feedback(backend)
    assert feedback['left']['feedback']['stamp_s']==11.
    t.close();r=next(r for r in rows(t.path) if r['event']=='driver')
    assert r['command_urdf_rad']==[1]*7 and r['context']['command_seq']==19


def test_write_failure_counts_in_flight_samples(tmp_path, monkeypatch):
    original=Path.open
    class Broken:
        def __enter__(self):return self
        def __exit__(self,*args):pass
        def write(self,*args):raise OSError('disk full')
    def open_file(path,*args,**kwargs):
        return Broken() if path==tmp_path/'t' else original(path,*args,**kwargs)
    monkeypatch.setattr(Path,'open',open_file)
    t=MotionTrace(tmp_path/'t',enabled=True);t.emit('sample',q=[1]*7)
    assert not t.close();assert t.error and t.dropped==t.enqueued==2


@pytest.mark.parametrize('failure', [False, True])
def test_consumed_feedback_survives_skip_or_failed_send_without_stale_reuse(tmp_path, failure):
    from esrobo_link.gateway import Gateway
    from esrobo_link.backends import MockBackend
    from esrobo_teleop.robot.nero_driver import JointFeedbackSnapshot
    b=MockBackend('left');b.side='left'
    snap=JointFeedbackSnapshot(np.arange(7),12.,time.monotonic(),'consumed-cache')
    b.arm=SimpleNamespace(_cycle_feedback=snap,_motion_trace_feedback=None)
    g=Gateway(b,b'x'*32,port=0);g.motion_trace=MotionTrace(tmp_path/'t',enabled=True)
    def step(target,cancel):
        b.arm._motion_trace_feedback=b.arm._cycle_feedback
        b.arm._cycle_feedback=None
        b.arm.last_command_skipped=True
        b.arm.last_command_skip_reason='duplicate_feedback'
        if failure:raise RuntimeError('send failed')
    b.step=step
    try:
        if failure:
            with pytest.raises(RuntimeError,match='send failed'):g._execute_traced({},lambda:False,{'command_seq':4})
        else:g._execute_traced({},lambda:False,{'command_seq':4})
        assert b.arm._motion_trace_feedback is None
        b.step=lambda *args:None
        g._execute_traced({},lambda:False,{'command_seq':5})
    finally:g.motion_trace.close();g.socket.close()
    records=[r for r in rows(g.motion_trace.path) if r['event']=='execution_cycle']
    assert records[0]['feedback']['left']['feedback']['stamp_s']==12.
    assert records[0]['completed'] is not failure
    assert records[1]['feedback']['left']['feedback'] is None
