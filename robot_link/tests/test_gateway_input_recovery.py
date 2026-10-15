import time

import pytest

from esrobo_link.backends import MockBackend, MockDualBackend
from esrobo_link.gateway import Gateway
from esrobo_link.protocol import ProtocolError, InputHoldTransition, pack
from esrobo_link.session import SessionGate

KEY = b'recovery-tests-only-not-a-deployment-key'


def connected(monkeypatch):
    now=[10.]
    monkeypatch.setattr(time,'monotonic', lambda:now[0])
    b=MockBackend(with_hand=True); b.contract['input_recovery_enabled']=True
    g=Gateway(b,KEY,port=0)
    peer=('127.0.0.1',10000)
    g.gate.hello(dict(client_nonce='a'*32,input_recovery_protocol=1),peer,now[0],False)
    def send(kind='target',recovery=None):
        state=g.gate.state(now[0],{})
        msg=dict(type=kind,session=g.gate.session,seq=g.gate.sequence+1,lease=state['lease'],
                 side='left',contract_id=b.contract['id'])
        if kind=='target':msg.update(arm_urdf_rad=[0.]*7,hand_unit=[128]*10)
        if recovery:msg['recovery']=recovery
        return g.accept_command(msg,peer,now[0])
    send(); g.mode='ACTIVE'; b.enabled=True; b.hand_enabled=True
    return g,b,now,send


def test_hold_survives_client_teardown_and_requires_local_recovery(monkeypatch):
    g,b,now,send=connected(monkeypatch)
    try:
        send('hold'); now[0]+=.501
        g.watchdog(); assert g.mode=='HOLD_LATCHED'
        g.tick(); assert b.enabled and b.stops==0
        send('close');g.input_fault('client close')
        for _ in range(10):now[0]+=.02;g.tick()
        assert b.enabled and b.stops==0
        with pytest.raises(ProtocolError,match='local recovery'):send()
        g.request('x');g.tick()
        assert g.mode=='FAULT' and b.stops==1
    finally:g.socket.close()


def test_operator_controlled_stop_keeps_hold_across_close_and_repeated_requests(monkeypatch):
    g,b,now,send=connected(monkeypatch)
    try:
        assert g.recovery_status()['controlled_stop_protocol'] == 1
        g.request('h')
        assert g.mode == 'HOLD_LATCHED' and not g.cancelled.is_set()
        assert not g.stop_pending
        g.tick()
        send('close'); g.input_fault('client exited')
        g.request('h')
        for _ in range(30): now[0]+=.02; g.tick()
        assert b.enabled and b.stops == 0
        with pytest.raises(ProtocolError): send()
        g.request('x'); g.tick()
        assert b.stops == 1 and g.mode == 'FAULT'
    finally:g.socket.close()


@pytest.mark.parametrize('mode,verified', [('ACTIVE',False),('FAULT',True),('APPROACHING',True),('RETURNING',True)])
def test_controlled_stop_never_silently_falls_back_to_estop(monkeypatch,mode,verified):
    g,b,now,send=connected(monkeypatch)
    try:
        g.mode=mode; b.contract['input_recovery_enabled']=verified
        with pytest.raises(RuntimeError): g.request('h')
        assert g.mode == mode and not g.stop_pending and b.stops == 0
    finally:g.socket.close()


def test_hold_feedback_failure_still_trips_hardware_stop(monkeypatch):
    g,b,now,send=connected(monkeypatch)
    try:
        g.input_fault('input timeout')
        def broken(*args):raise RuntimeError('feedback expired')
        b.step=broken;g.tick();g.tick()
        assert g.mode=='FAULT' and b.stops==1
    finally:g.socket.close()


def test_watchdog_interrupts_target_without_overwriting_hold(monkeypatch):
    g,b,now,send=connected(monkeypatch)
    try:
        g.input_fault('input timeout')
        with pytest.raises(InputHoldTransition):g.motion_cancelled()
        g.tick();assert b.stops==0
    finally:g.socket.close()


def test_robot_recovery_deadline_cannot_be_reset_by_holds_or_new_epoch(monkeypatch):
    g,b,now,send=connected(monkeypatch)
    try:
        send('hold');now[0]+=.06
        meta=dict(phase='reacquiring',epoch='a'*16);send(recovery=meta)
        started=g.gate.recovery_started_at
        now[0]+=.1;send('hold');now[0]+=.04;send(recovery=meta)
        assert g.gate.recovery_started_at==started
        with pytest.raises(ProtocolError,match='original epoch'):
            send(recovery=dict(meta,epoch='b'*16))
        now[0]=started+3.01;g.watchdog();g.tick()
        assert g.mode=='HOLD_LATCHED' and b.stops==0
    finally:g.socket.close()


def test_uncommissioned_and_old_client_rejected(monkeypatch):
    g,b,now,send=connected(monkeypatch)
    try:
        other=SessionGate(b.contract)
        with pytest.raises(ProtocolError,match='lacks input recovery'):
            other.hello(dict(client_nonce='x'*32),('127.0.0.1',20),10.,False)
        b.contract['input_recovery_enabled']=False
        send('hold')
        with pytest.raises(ProtocolError,match='commissioned'):
            send(recovery=dict(phase='reacquiring',epoch='a'*16))
        g.input_fault('input expired');g.tick()
        assert g.mode=='FAULT' and b.stops==1
    finally:g.socket.close()


def test_completion_repeat_is_idempotent_but_epoch_reuse_is_refused(monkeypatch):
    g,b,now,send=connected(monkeypatch)
    try:
        send('hold');send(recovery=dict(phase='reacquiring',epoch='a'*16))
        for _ in range(2):send(recovery=dict(phase='complete',epoch='a'*16))
        send('hold')
        with pytest.raises(ProtocolError,match='unused epoch'):
            send(recovery=dict(phase='reacquiring',epoch='a'*16))
    finally:g.socket.close()


def test_dual_hold_requires_all_sides_healthy(monkeypatch):
    b=MockDualBackend()
    b.contract['input_recovery_enabled']=True
    g=Gateway(b,KEY,port=0)
    try:
        for m in b.members.values():m.enabled=True
        g.mode='ACTIVE';g.input_fault('input failure');g.tick()
        assert all(m.stops==0 for m in b.members.values())
        b.members['right'].enabled=False
        g.tick();g.tick()
        assert g.mode=='FAULT'
        assert all(m.stops>=1 for m in b.members.values())
    finally:g.socket.close()


@pytest.mark.parametrize('gap', [.065, .1, .2])
def test_udp_active_reacquisition_then_client_close_latches(tmp_path, gap):
    """Real authenticated UDP and worker threads; exclusively loopback mock I/O."""
    import threading
    from esrobo_link.client import RobotClient
    key=tmp_path/'key';key.write_bytes(KEY)
    b=MockBackend(with_hand=True);b.contract['input_recovery_enabled']=True
    g=Gateway(b,KEY,port=0)
    worker=threading.Thread(target=g.run);worker.start()
    client=RobotClient('127.0.0.1',g.address[1],str(key))
    try:
        client.connect();client.send_target([0.]*7,[128]*10)
        client.receive(.2)
        g.request('e')
        deadline=time.monotonic()+2
        while g.mode!='ACTIVE' and time.monotonic()<deadline:
            client.receive(.2);client.send_target([0.]*7,[128]*10)
        assert g.mode=='ACTIVE'
        end=time.monotonic()+gap
        while time.monotonic()<end:
            client.receive(.2);client.send_hold()
        client.recovery_metadata=dict(phase='reacquiring',epoch='e'*16)
        q=[0.]*6+[.23265]
        for _ in range(55):
            client.receive(.2);client.send_target(q,[144]*10)
        assert g.mode=='ACTIVE' and g.gate.recovery_started_at is not None
        assert b.q[-1]==pytest.approx(q[-1])
        client.recovery_metadata=dict(phase='complete',epoch='e'*16)
        for _ in range(3):client.receive(.2);client.send_target(q,[144]*10)
        client.receive(.2)
        assert g.gate.recovery_started_at is None
        client.close()
        deadline=time.monotonic()+.5
        while g.mode!='HOLD_LATCHED' and time.monotonic()<deadline:time.sleep(.01)
        assert g.mode=='HOLD_LATCHED' and b.enabled and b.stops==0
    finally:
        client.socket.close();g.request('q');worker.join(2)
        assert not worker.is_alive()


@pytest.mark.parametrize('key', ['r', 'z'])
def test_legacy_hold_return_preserves_latched_hold(monkeypatch, key):
    g,b,now,send=connected(monkeypatch)
    try:
        g.input_fault('input expired');g.tick()
        b.contract.pop('hold_return_protocol')
        with pytest.raises(RuntimeError, match='latched hold retained'):
            g.request(key)
        assert g.mode == 'HOLD_LATCHED' and g.pending is None
        now[0] += .02; g.tick()
        assert b.enabled and b.stops == 0
    finally:
        g.socket.close()


@pytest.mark.parametrize('key', ['r', 'z'])
@pytest.mark.parametrize('refused', [False, True])
def test_healthy_hold_return_has_no_stop_or_predisable(monkeypatch, key, refused):
    g,b,now,send=connected(monkeypatch)
    try:
        g.input_fault('input expired');g.tick()
        if refused:
            def reject(cancelled, *, from_hold):
                assert from_hold
                return dict(returned=False, disabled=False, holding=True, stage='planning', reason='no route')
            b.return_zero = reject
        g.request(key)
        assert g.pending == 'z_hold' and b.enabled and b.stops == 0
        g.tick()
        assert g.mode == ('HOLD_LATCHED' if refused else 'IDLE')
        assert b.enabled == refused and b.stops == 0
    finally:
        g.socket.close()
