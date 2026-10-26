"""Timing/ownership tests with controlled notification input, no real subscription."""
import copy
import json
import threading
import time
from types import SimpleNamespace

import pytest

from esrobo_laptop.acquisition import load_bridge
from esrobo_laptop.pico_transport import (PicoForwarder, LatestHands, TransportMetrics,
                                         require_notifications, summary_times, AsyncRecording)
from esrobo_laptop.pico_hand import PicoHands
from esrobo_laptop.diagnostics import RuntimeDiagnostics


class FakeSDK:
    def __init__(self, hz=74):
        self.hz = hz
        self.condition = threading.Condition()
        self.latest = dict(notification_id=0, body=self.sample(0), left=self.hand(0), right=self.hand(0))
        self.receipts = []
        self.end = threading.Event()
        self.thread = None

    @staticmethod
    def sample(seq, *, age=0.):
        return dict(valid=seq > 0, active=1, sequence=seq, packet_timestamp_ns=seq,
                    received_monotonic_ns=time.monotonic_ns()-int(age*1e9), age_s=age,
                    poses=[[1., 2., 3., 0., 0., 0., 1.]]*24)

    @classmethod
    def hand(cls, seq, **kw):
        return dict(cls.sample(seq, **kw), poses=[[0., 0., 0., 0., 0., 0., 1.]]*26,
                    flags=[15]*26, timestamp_ns=0, freshness_source='component-packet')

    def get_tracking_updates_version(self):
        return 1

    def close(self):
        self.end.set()
        if self.thread:
            self.thread.join()

    def get_tracking_diagnostics(self):
        with self.condition:
            return {s: dict(committed=len(self.receipts), rejected=0, invalid=0, receipts_ns=self.receipts[-200:]) for s in ('body', 'left', 'right')}

    def emit(self, seq, *, age=0., body=None):
        with self.condition:
            self.latest = dict(notification_id=seq, body=self.sample(seq, age=age) if body is None else body,
                               left=self.hand(seq, age=age), right=self.hand(seq, age=age))
            self.receipts.append(self.latest['body']['received_monotonic_ns'])
            self.condition.notify_all()

    def wait_tracking_updates(self, after_id, timeout_ms=50):
        with self.condition:
            self.condition.wait_for(lambda: self.latest['notification_id'] > after_id, timeout_ms/1000.)
            return copy.deepcopy(self.latest)

    def start(self):
        def produce():
            seq = 0
            due = time.monotonic()
            while not self.end.is_set():
                seq += 1
                self.emit(seq)
                due += 1/self.hz
                self.end.wait(max(0., due-time.monotonic()))
        self.thread = threading.Thread(target=produce)
        self.thread.start()


class Socket:
    def __init__(self):
        self.packets = []
    def setblocking(self, value):
        pass
    def sendto(self, data, destination):
        self.packets.append((time.monotonic_ns(), json.loads(data)))
    def close(self):
        pass


class Mapper:
    def __init__(self, delay=0.):
        self.delay = delay
    def map_sample(self, side, sample):
        time.sleep(self.delay)
        return dict(joints=[0.]*10, sequence=sample['sequence'],
                    sample_monotonic=sample['received_monotonic_ns']/1e9, palm_basis=[[1,0,0],[0,1,0],[0,0,1]])


def test_body_carries_fresh_palm_even_when_finger_computation_fails():
    from test_pico_hand import skeleton
    class ValidSDK(FakeSDK):
        @classmethod
        def hand(cls, seq, **kw):
            return dict(super().hand(seq, **kw), poses=skeleton(.2).tolist())
    class FailedMapper:
        def map_sample(self, side, sample):
            raise ValueError('simulated hand solver timeout')
    sdk = ValidSDK(hz=90)
    f = forwarder(sdk, mapper=FailedMapper())
    f.args.samples = 20
    sdk.start()
    try:
        f.run()
    finally:
        sdk.close(); f.close()
    assert not f.hand_socket.packets
    assert len(f.body_socket.packets) == 20
    for _, packet in f.body_socket.packets:
        assert packet['palm_publication_version'] == 1
        for side in ('left', 'right'):
            palm = packet['pico_palms'][side]
            assert palm['sequence'] > 0
            assert palm['sample_monotonic'] > 0


def forwarder(sdk, **kwargs):
    bridge = load_bridge('xrobotoolkit_body_udp_bridge')
    args = bridge.parse_args(['--samples', '65', '--print-interval', '2'])
    body, hand = Socket(), Socket()
    return PicoForwarder(sdk, bridge, args, body_socket=body, hand_socket=hand, **kwargs)


def run_profiled(f):
    from tools.profile_runtime import Profile
    import tempfile
    with tempfile.TemporaryDirectory() as temp:
        prof = Profile(temp+'/profile.jsonl')
        prof.wrap(f.sdk, 'wait_tracking_updates', 'sdk_wait', prof.tracking)
        prof.wrap(f, '_send', 'udp_send', lambda a,k,r:prof.packet(a[1],'udp_send') if r else None)
        try:
            f.run()
        finally:
            f.test_profile = prof.close()
    return f.test_profile


def test_transport_rate_expires_without_refreshing_sample_age():
    now = 20_000_000_000
    receipts = [now-10_000_000*i for i in range(199, -1, -1)]
    assert summary_times(receipts, now)['hz'] == pytest.approx(100.)
    stale = summary_times(receipts, now+800_000_000_000)
    assert stale == dict(hz=0., age_ms=800_000.)
    assert summary_times([], now) == dict(hz=0., age_ms=None)


def test_hand_diagnostics_expire_when_receipts_or_mapping_stop(monkeypatch):
    now = time.monotonic_ns()
    sdk = FakeSDK()
    sdk.receipts = [now-200_000_000, now-190_000_000]
    f = forwarder(sdk, mapper=Mapper())
    for side in ('left', 'right'):
        f.metrics.status(side, 'tracking')
        f.metrics.mark(side+'_map', now-190_000_000)
    monkeypatch.setattr('esrobo_laptop.pico_transport.time.monotonic_ns', lambda: now)
    assert f.snapshot()['hand_status'] == dict(left='stale', right='stale')
    # Fresh raw data cannot make an old mapped control sample fresh.
    sdk.receipts = [now-10_000_000, now]
    assert f.snapshot()['hand_status']['left'] == 'stale'
    f.metrics.mark('left_map', now)
    assert f.snapshot()['hand_status']['left'] == 'tracking'
    assert f.metrics.status_snapshot()['right'] == 'tracking'  # derived display only


@pytest.mark.parametrize('hz', [60, 74, 90])
@pytest.mark.performance
def test_uniform_input_forwards_95_percent_with_low_notification_latency(hz):
    sdk = FakeSDK(hz)
    f = forwarder(sdk, mapper=Mapper())
    sdk.start()
    try:
        run_profiled(f)
    finally:
        sdk.close()
    summary = f.snapshot()
    sequences = [p['body_sample_sequence'] for _, p in f.body_socket.packets]
    assert len(sequences)/max(sequences) >= .95
    assert f.test_profile['stages']['notification_to_send']['p95_ms'] <= 10.
    assert sequences == sorted(set(sequences))
    for _, packet in f.body_socket.packets:
        assert packet['laptop_input']['sample_monotonic']['right'] == packet['body_sample_monotonic']


@pytest.mark.parametrize('delay', [.02, .1, .5])
@pytest.mark.performance
def test_body_does_not_wait_for_hand_computation(delay):
    sdk = FakeSDK()
    f = forwarder(sdk, mapper=Mapper(delay))
    sdk.start()
    try:
        run_profiled(f)
    finally:
        sdk.close()
    times = [t for t, _ in f.body_socket.packets]
    assert max(b-a for a,b in zip(times,times[1:]))/1e6 < 50.
    assert len(f.hands.slots) <= 2
    if delay >= .1:
        assert not f.hand_socket.packets  # Expired during mapping: no fresh-control lie.
    assert f.metrics.counts['left_coalesced']+f.metrics.counts['right_coalesced'] > 0


def test_burst_latest_only_stale_and_duplicate_never_forward():
    sdk = FakeSDK()
    sdk.emit(100)
    f = forwarder(sdk, mapper=Mapper())
    f.args.samples = 0
    thread = threading.Thread(target=f.run)
    thread.start()
    time.sleep(.04)
    sdk.emit(101, age=.12)
    time.sleep(.04)
    sdk.emit(102, body=dict(FakeSDK.sample(102), packet_timestamp_ns=100))
    time.sleep(.04)
    f.stop.set()
    thread.join(1.)
    assert not thread.is_alive()
    assert [p['body_sample_sequence'] for _,p in f.body_socket.packets] == [100]
    assert f.metrics.counts['body_coalesced'] == 99
    assert f.metrics.counts['body_stale'] == 1
    assert f.metrics.counts['body_duplicate'] == 1


def test_latest_slots_bounded_and_owned_by_side():
    metrics = TransportMetrics()
    slots = LatestHands(metrics)
    for i in range(1, 1001):
        slots.offer('right', {'sequence': i})
    assert slots.slots == {'right': {'sequence': 1000}}
    assert metrics.counts['right_coalesced'] == 999
    assert slots.take() == ('right', {'sequence': 1000})
    slots.offer('right', {'sequence': 1})
    assert not slots.slots
    slots.close()
    assert slots.take() is None


def test_old_sdk_fails_before_subscribing():
    with pytest.raises(RuntimeError, match='rebuild'):
        require_notifications(SimpleNamespace(init=lambda: pytest.fail('must not subscribe')))


def test_native_receipt_survives_packet_build_and_stale_cached_age():
    bridge = load_bridge('xrobotoolkit_body_udp_bridge')
    sample = FakeSDK.sample(1, age=.03)
    received = sample['received_monotonic_ns']/1e9
    sample['age_s'] = 0.  # Getter age can never hide time spent waiting/mapping.
    packet, count = bridge._packet_from_sample(sample, 0, False)
    assert count == 24
    assert packet['body_sample_monotonic'] == received
    assert packet['body_sample_age_s'] >= .03
    sample['received_monotonic_ns'] -= 100_000_000
    assert bridge._packet_from_sample(sample, 0, False) == (None, 0)


def test_hand_expiration_invalid_flags_and_side_independence():
    mapper = PicoHands()
    with pytest.raises(ValueError, match='stale'):
        mapper.map_sample('right', FakeSDK.hand(1, age=.101))
    hand = FakeSDK.hand(2)
    hand['flags'][3] = 0
    with pytest.raises(ValueError, match='invalid/occluded'):
        mapper.map_sample('right', hand)
    assert not mapper.last.get('left')


@pytest.mark.performance
def test_blocked_terminal_does_not_block_body(tmp_path):
    release = threading.Event()
    class Blocked:
        def write(self, value):
            release.wait(5.)
        def flush(self):
            pass
    diagnostics = RuntimeDiagnostics(tmp_path/'pico.jsonl', stdout=Blocked())
    diagnostics.message('block terminal')
    sdk = FakeSDK()
    f = forwarder(sdk, diagnostics=diagnostics, mapper=Mapper(.1))
    sdk.start()
    try:
        run_profiled(f)
        assert len(f.body_socket.packets) == 65
        assert f.test_profile['stages']['notification_to_send']['p95_ms'] < 10.
    finally:
        sdk.close()
        release.set()
        diagnostics.close()


def test_recording_backpressure_bounded_and_timestamps_preserved(tmp_path):
    release = threading.Event()
    saved = []
    writer = SimpleNamespace(warmup_discard_s=0.,
        write_packet=lambda p,n,**kw: (release.wait(2.), saved.append((p, kw))) and (0., 'neutral_start'),
        write_event=lambda *a,**kw: None, close=lambda: None)
    diagnostics = RuntimeDiagnostics(tmp_path/'diag.jsonl')
    metrics = TransportMetrics()
    recording = AsyncRecording(writer, metrics, diagnostics)
    packet = {'frames': {'right_hand': [1]}}
    for _ in range(150):
        recording.submit(packet, 1)
    packet['frames']['right_hand'][0] = 9
    assert len(recording.queue) <= 64
    assert metrics.counts['record_dropped'] > 0
    release.set()
    assert recording.close()
    diagnostics.close()
    assert all(p['frames']['right_hand'] == [1] for p,kw in saved)
    assert all(kw['record_monotonic_ns'] <= time.monotonic_ns() for p,kw in saved)


def test_owner_initializes_once_per_start_and_workers_never_subscribe(monkeypatch, tmp_path, isolated_run_pico):
    import sys
    import esrobo_laptop.pico_transport as module
    sdk = FakeSDK()
    counts = {'init':0, 'deinit':0}
    original_close = sdk.close
    def init():
        counts['init'] += 1
        sdk.end.clear()
        sdk.start()
    def deinit():
        counts['deinit'] += 1
        original_close()
    sdk.init, sdk.close = init, deinit
    monkeypatch.setitem(sys.modules, 'xrobotoolkit_sdk', sdk)
    monkeypatch.setattr(module, 'WORKSPACE', tmp_path)
    bridge = load_bridge('xrobotoolkit_body_udp_bridge')
    args = bridge.parse_args(['--samples','3'])
    assert module.run_pico(bridge,args) == 0
    assert counts == {'init':1, 'deinit':1}
    sdk.latest['notification_id'] = 0
    sdk.latest['body'] = sdk.sample(0)
    assert module.run_pico(bridge,args) == 0
    assert counts == {'init':2, 'deinit':2}
    assert len(isolated_run_pico) == 4  # Body and Hand sockets on each start.
    assert all(s.closed for s in isolated_run_pico)
    assert sum(len(s.packets) for s in isolated_run_pico) > 0


@pytest.mark.parametrize('delay', [.02, .1, .5])
@pytest.mark.performance
def test_slow_recording_does_not_hold_forwarder(tmp_path, delay):
    diagnostics = RuntimeDiagnostics(tmp_path/'diagnostic.jsonl')
    sdk = FakeSDK()
    f = forwarder(sdk, mapper=Mapper())
    release = threading.Event()
    class Writer:
        warmup_discard_s = 0.
        def write_packet(self, *a, **kw):
            release.wait(delay)
            return (0.,'neutral_start')
        def write_event(self,*a,**kw):
            pass
        def close(self):
            pass
    recording = AsyncRecording(Writer(),f.metrics,diagnostics)
    f.recording = recording
    sdk.start()
    try:
        run_profiled(f)
        assert len(f.body_socket.packets) == 65
        assert len(recording.queue) <= 64
        assert f.test_profile['stages']['notification_to_send']['p95_ms'] < 10.
    finally:
        sdk.close()
        release.set()
        recording.close()
        diagnostics.close()


def test_rate_cap_coalesces_fast_source_without_replaying_history():
    sdk = FakeSDK(300)
    f = forwarder(sdk, mapper=Mapper())
    sdk.start()
    try:
        run_profiled(f)
    finally:
        sdk.close()
    packets = f.body_socket.packets
    elapsed = (packets[-1][0]-packets[0][0])/1e9
    assert (len(packets)-1)/elapsed <= 92.
    sequences = [p['body_sample_sequence'] for _,p in packets]
    assert all(b>a for a,b in zip(sequences,sequences[1:]))
    assert f.metrics.counts['body_coalesced'] > 100
    assert len(f.hands.slots) <= 2


def test_recording_failure_is_reported_without_claiming_persistence(tmp_path):
    metrics = TransportMetrics()
    diagnostics = RuntimeDiagnostics(tmp_path/'errors.jsonl')
    def fail(*a,**kw):
        raise OSError('disk full')
    writer = SimpleNamespace(warmup_discard_s=0.,write_packet=fail,close=lambda:None)
    recording = AsyncRecording(writer,metrics,diagnostics)
    recording.submit({'frames':{}},10)
    assert not recording.close()
    assert metrics.counts['record_errors'] == 1
    assert metrics.error_snapshot()['recording'] == 'disk full'
    diagnostics.close()


def test_sdk_has_only_vendor_close_name_and_cleanup_failure_does_not_skip_logs(monkeypatch, tmp_path, isolated_run_pico):
    import sys
    import esrobo_laptop.pico_transport as module
    sdk = FakeSDK()
    native_close = sdk.close
    calls = []
    def close():
        calls.append('close')
        native_close()
        raise OSError('simulated close failure')
    sdk.init, sdk.close = sdk.start, close
    assert not hasattr(sdk,'deinit')
    monkeypatch.setitem(sys.modules,'xrobotoolkit_sdk',sdk)
    monkeypatch.setattr(module,'WORKSPACE',tmp_path)
    bridge = load_bridge('xrobotoolkit_body_udp_bridge')
    with pytest.raises(RuntimeError,match='cleanup incomplete'):
        module.run_pico(bridge,bridge.parse_args(['--samples','2']))
    assert calls == ['close']
    assert len(isolated_run_pico) == 2 and all(s.closed for s in isolated_run_pico)
    assert list((tmp_path/'teleoperation/log').glob('pico_transport_*.jsonl'))
    assert list((tmp_path/'teleoperation/log').glob('pico_motion_*.jsonl'))
    assert not sdk.thread.is_alive()
    assert not any(t.name in ('laptop-log','laptop-console') and t.is_alive() for t in threading.enumerate())


@pytest.fixture
def isolated_run_pico(monkeypatch):
    """Test SDK ownership with a mapper stub and no real UDP delivery."""
    import esrobo_laptop.pico_transport as module
    sockets = []
    class OwnedSocket(Socket):
        closed = False
        def close(self):
            self.closed = True
    def create_socket(*args):
        sock = OwnedSocket()
        sockets.append(sock)
        return sock
    monkeypatch.setattr(module, 'default_hand_mapper', lambda: Mapper())
    # Replace this module's socket facade, not the shared socket module.
    monkeypatch.setattr(module, 'socket', SimpleNamespace(
        AF_INET=module.socket.AF_INET, SOCK_DGRAM=module.socket.SOCK_DGRAM,
        socket=create_socket))
    return sockets


def test_default_hand_mapper_loads_real_ten_axis_config(monkeypatch):
    from pathlib import Path
    import esrobo_laptop.pico_transport as module
    import esrobo_laptop.hand_worker as worker
    configs = []
    mapper = Mapper()
    def create(cfg):
        configs.append(cfg)
        return mapper
    monkeypatch.setattr(worker, 'ProcessHandMapper', create)
    assert module.default_hand_mapper() is mapper
    assert len(configs) == 1
    cfg = configs[0]
    assert cfg.hand.left_retarget_mode == 'urdf_10'
    assert cfg.hand.left_enabled_physical_joints == list(range(10))
    assert Path(cfg.ik.urdf_path).is_absolute() and Path(cfg.ik.urdf_path).is_file()


@pytest.mark.parametrize('send_ok,stale', [(True, False), (False, False), (True, True)])
def test_hand_state_commit_requires_fresh_successful_publication(send_ok, stale):
    class AcknowledgedMapper(Mapper):
        def __init__(self):
            super().__init__(); self.acks = []
        def map_sample(self, side, sample):
            result = super().map_sample(side, sample)
            if stale:
                result['sample_monotonic'] -= .2
            return result
        def finish(self, side, sequence, accepted):
            self.acks.append((side, sequence, accepted))
    mapper = AcknowledgedMapper(); f = forwarder(FakeSDK(), mapper=mapper)
    calls = iter([('left', FakeSDK.hand(1)), None])
    f.hands.take = lambda: next(calls)
    f._send = lambda *args: send_ok
    try:
        f._hands()
        assert mapper.acks == [('left', 1, send_ok and not stale)]
    finally:
        f.close()


@pytest.mark.parametrize('solver_mode', ['v13', 'wrist5'])
def test_worker_publishes_only_matching_acknowledgement(monkeypatch, solver_mode):
    from esrobo_laptop import hand_worker
    from esrobo_laptop import hand_solver
    import numpy as np
    solvers = []
    class Solver:
        def __init__(self, cfg):
            self.previous = np.zeros(10); self.model = SimpleNamespace(
                forward=lambda q: (np.zeros((26, 3)), None), digest='test')
            self.pending = None; self.committed = []; self.serial = 0
            solvers.append(self)
        def solve(self, poses, **kw):
            self.serial += 1; self.pending = self.serial
            return dict(joints=[0.]*10, candidate_id=self.serial)
        def discard(self): self.pending = None
        def commit(self, candidate):
            assert candidate == self.pending
            self.committed.append(candidate); self.pending = None
    class Hands:
        def __init__(self, fit): self.fit = fit
        def map_sample(self, side, sample): return self.fit(side, [])
    class Connection:
        def __init__(self): self.sent = []
        def send(self, value): self.sent.append(value)
        def recv(self): return next(messages)
        def close(self): pass
    monkeypatch.setattr(hand_solver, 'create_solver', Solver)
    monkeypatch.setattr(hand_solver, 'contract', lambda cfg, digest=None: ('test-version', 'test'))
    monkeypatch.setattr('esrobo_laptop.pico_hand.PicoHands', Hands)
    messages = iter([
        ('sample', 1, dict(received_monotonic_ns=1)),
        ('publish', 1, False),
        ('sample', 2, dict(received_monotonic_ns=2)),
        ('publish', 1, True),  # Late success from rejected earlier request.
        ('sample', 3, dict(received_monotonic_ns=3)),
        ('publish', 2, True),  # Superseded candidate cannot commit either.
        ('publish', 3, True), None])
    cfg = SimpleNamespace(hand=SimpleNamespace(left_retarget_mode='urdf_10', left_retarget_solver=solver_mode))
    connection = Connection(); hand_worker._run(connection, cfg)
    assert solvers[0].committed == [4]  # Warm-up=1, then samples=2,3,4.
    assert all('candidate_id' not in r.get('result', {}) for r in connection.sent)


@pytest.mark.parametrize('cancelled', [False, True])
def test_contact_details_stay_in_log_and_cancelled_result_is_not_committed(cancelled):
    records = []
    class DetailedMapper(Mapper):
        def __init__(self): super().__init__(); self.acks = []
        def map_sample(self, side, sample):
            r = super().map_sample(side, sample)
            r.update(candidate_id=9, target_points=[[1., 2., 3.]]*26,
                diagnostics=dict(solve_ms=3., tip_error_mm=[1.]*5, pinch_error_mm=[1.]*4,
                    limited_axes=[], contact_active=[True]*4, correction_deg=[0.]*10))
            if cancelled: f.stop.set()
            return r
        def finish(self, side, sequence, accepted): self.acks.append(accepted)
    mapper = DetailedMapper(); f = forwarder(FakeSDK(), mapper=mapper)
    f.diagnostics = SimpleNamespace(reference=lambda path,row: records.append(copy.deepcopy(row)))
    calls = iter([('left', FakeSDK.hand(1)), None]); f.hands.take = lambda:next(calls)
    try:
        f._hands()
        assert records[0]['result']['diagnostics']['contact_active'] == [True]*4
        assert mapper.acks == [not cancelled]
        if cancelled:
            assert f.hand_socket.packets == []
        else:
            packet = f.hand_socket.packets[0][1]['pico_hands']['left']
            assert 'candidate_id' not in packet and 'target_points' not in packet
            assert 'contact_active' not in packet['diagnostics']
            assert packet['diagnostics']['solve_ms'] == 3.
    finally:
        f.close()
