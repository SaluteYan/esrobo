"""Slow or unavailable diagnostic sinks must not backpressure control."""
import io
import json
from pathlib import Path
import threading
import time
from unittest.mock import Mock

import numpy as np
import pytest

from esrobo_laptop.diagnostics import RuntimeDiagnostics, _Mailbox


class SlowStream:
    def __init__(self, stream, delay=0., release=None, failure=None):
        self.stream, self.delay, self.release, self.failure = stream, delay, release, failure
        self.entered = threading.Event()

    def write(self, text):
        self.entered.set()
        if self.release:
            assert self.release.wait(3)
        if self.delay:
            time.sleep(self.delay)
        if self.failure:
            raise self.failure
        return self.stream.write(text)

    def flush(self):
        self.stream.flush()

    def close(self):
        self.stream.close()


@pytest.mark.parametrize('delay', [.02, .1, .5])
def test_slow_disk_does_not_block_control_submissions(tmp_path, delay):
    writer = RuntimeDiagnostics(tmp_path/'run.jsonl', tmp_path/'status.json', stdout=io.StringIO())
    writer._stream = slow = SlowStream(writer._stream, delay)
    writer.submit({'monotonic_s': 1., 'value': 0})
    assert slow.entered.wait(1)
    finished = threading.Event()
    def control():
        for i in range(200):
            writer.submit({'monotonic_s': float(i+2), 'value': i})
        finished.set()
    producer = threading.Thread(target=control)
    producer.start()
    assert finished.wait(.1), 'producer waited for slow disk'
    producer.join()
    assert writer.stats()['files']['pending'] <= 1
    assert writer.stats()['files']['coalesced'] > 0
    started = time.monotonic()
    writer.close()
    assert time.monotonic()-started < 1.2
    assert writer.close()  # A 500 ms in-flight write plus one queued write can exceed the first drain budget.
    assert json.loads((tmp_path/'status.json').read_text())['value'] == 199


def test_blocked_terminal_does_not_block_files_or_control(tmp_path):
    release = threading.Event()
    terminal = SlowStream(io.StringIO(), release=release)
    writer = RuntimeDiagnostics(tmp_path/'run.jsonl', stdout=terminal)
    writer.message('blocked')
    assert terminal.entered.wait(1)
    for i in range(100):
        writer.message(str(i))
        writer.submit({'value': i})
    assert writer.console.stats()['pending'] == 64
    assert writer.console.stats()['dropped'] == 36
    assert not writer.close(timeout=.05)
    release.set()
    assert writer.close()
    assert (tmp_path/'run.jsonl').read_text()


def test_fault_slot_and_status_sequence_survive_saturation(tmp_path):
    release = threading.Event()
    writer = RuntimeDiagnostics(tmp_path/'run.jsonl', tmp_path/'status.json', stdout=io.StringIO())
    writer._stream = slow = SlowStream(writer._stream, release=release)
    writer.submit({'monotonic_s': 10., 'value': 'first'})
    assert slow.entered.wait(1)
    for i in range(100):
        writer.submit({'monotonic_s': float(i+11), 'value': i}, event=True)
    writer.submit({'monotonic_s': 111., 'value': 'latest'})
    writer.submit({'monotonic_s': 112., 'record_type': 'terminal_fault', 'status': 'original fault'}, fault=True)
    assert writer.files.stats() == dict(pending=66, coalesced=0, dropped=36)
    # Close flushes the reserved fault first; older queued snapshots must never
    # replace a newer status file even with priority processing on shutdown.
    writer.files.close()
    release.set()
    assert writer.close()
    rows = [json.loads(x) for x in (tmp_path/'run.jsonl').read_text().splitlines()]
    assert rows[1]['status'] == 'original fault'
    status = json.loads((tmp_path/'status.json').read_text())
    assert status['value'] == 'latest' and status['monotonic_s'] == 111.
    assert not (tmp_path/'status.tmp').exists()


def test_snapshots_are_owned_and_aggregated_off_thread(tmp_path):
    release = threading.Event()
    writer = RuntimeDiagnostics(tmp_path/'run.jsonl', tmp_path/'status.json', stdout=io.StringIO())
    writer._stream = slow = SlowStream(writer._stream, release=release)
    writer.submit({'value': 'first'})
    assert slow.entered.wait(1)
    record = {'monotonic_s': 123., 'targets': {'right': np.array([1., 2.])},
              'feedback': {'session': 'secret'}, 'lease': 'private'}
    writer.submit(record)
    record['targets']['right'][0] = 999
    release.set()
    assert writer.close()
    status = json.loads((tmp_path/'status.json').read_text())
    assert status['targets']['right'] == [1., 2.]
    assert status['monotonic_s'] == 123.
    assert 'timings' not in status
    assert 'write_ms_previous' not in status['diagnostics']
    assert 'private' not in (tmp_path/'run.jsonl').read_text()
    assert 'secret' not in (tmp_path/'run.jsonl').read_text()


def test_write_and_serialization_errors_are_nonfatal(tmp_path):
    writer = RuntimeDiagnostics(tmp_path/'run.jsonl', tmp_path/'status.json', stdout=io.StringIO())
    writer._stream = SlowStream(writer._stream, failure=OSError('disk unavailable'))
    writer.submit({'bad': object()}, event=True)
    writer.submit({'bad': float('nan')}, event=True)
    writer.submit({'monotonic_s': 1., 'value': 'after bad records'}, event=True)
    assert writer.close()
    assert writer.errors['journal'] == 'disk unavailable'
    assert writer.serialization_errors == 2
    status = json.loads((tmp_path/'status.json').read_text())
    assert status['value'] == 'after bad records'


def test_existing_log_is_not_overwritten(tmp_path):
    path = tmp_path/'run.jsonl'
    path.write_text('old evidence')
    with pytest.raises(FileExistsError):
        RuntimeDiagnostics(path)
    assert path.read_text() == 'old evidence'


def test_status_and_reference_errors_preserve_journal(tmp_path):
    status = tmp_path/'status.json'
    writer = RuntimeDiagnostics(tmp_path/'run.jsonl', status, stdout=io.StringIO())
    status.mkdir()  # Atomic replacement must report the error without blocking control.
    unavailable = tmp_path/'not-a-directory'
    unavailable.write_text('occupied')
    writer.reference(unavailable/'reference.jsonl', {'event': 'calibrated'})
    writer.submit({'monotonic_s': 1., 'value': 'first'}, event=True)
    writer.submit({'monotonic_s': 2., 'value': 'second'}, event=True)
    assert writer.close()
    rows = [json.loads(x) for x in (tmp_path/'run.jsonl').read_text().splitlines()]
    assert [r['value'] for r in rows] == ['first', 'second']
    assert writer.errors['status'] and writer.errors['reference']
    assert rows[-1]['diagnostics']['errors']['status']
    assert writer.errors['journal'] is None


def test_reference_output_leaves_input_lock(tmp_path, monkeypatch):
    from esrobo_teleop.device.body_device import BodyDevice
    device = BodyDevice.__new__(BodyDevice)
    writer = RuntimeDiagnostics(tmp_path/'run.jsonl', stdout=io.StringIO())
    device.diagnostic_record_sink = writer.reference
    record = {'event': 'calibrated', 'sample_monotonic_s': 10.}
    main_thread = threading.get_ident()
    original_open = Path.open
    def check_open(path, *args, **kwargs):
        assert threading.get_ident() != main_thread
        return original_open(path, *args, **kwargs)
    monkeypatch.setattr(Path, 'open', check_open)
    with threading.RLock():
        device._write_reference_record(tmp_path/'reference.jsonl', record)
    assert writer.close()
    monkeypatch.setattr(Path, 'open', original_open)
    assert json.loads((tmp_path/'reference.jsonl').read_text())['sample_monotonic_s'] == 10.


def test_closed_mailbox_is_bounded_and_does_not_accept_output():
    box = _Mailbox()
    box.close()
    assert box.put({'seq': 1}) is False
    assert box.get() is None


def test_control_fault_stops_before_waiting_for_log_close(tmp_path, monkeypatch):
    from esrobo_laptop import app
    from esrobo_laptop.config import Settings
    order = []
    client = Mock(received_at=time.monotonic())
    state = dict(mode='IDLE', contract={'id': 'test'}, feedback_cache_age_s=0.)
    client.connect.return_value = client.receive.return_value = state
    client.close.side_effect = lambda: order.append('STOP')
    class Pipeline:
        def set_diagnostics(self, writer):
            self.writer = writer
        def close(self):
            order.append('control resources closed')
    class FaultController:
        entry_keepalives_sent = holds_sent = 0
        def step(self, state):
            raise RuntimeError('original IK failure')
    class Writer(RuntimeDiagnostics):
        def __init__(self, *args):
            super().__init__(*args, stdout=io.StringIO(), stderr=io.StringIO())
        def close(self, timeout=1.):
            order.append('log close')
            return super().close(timeout)
    monkeypatch.setattr(app, 'read_settings', lambda _: Settings(robot_host='127.0.0.1', side='right', with_hand=False))
    monkeypatch.setattr(app, 'read_robot_config', lambda _: Mock())
    monkeypatch.setattr(app, 'verify_contract', lambda *a, **kw: {})
    monkeypatch.setattr(app, 'RobotClient', lambda *a: client)
    monkeypatch.setattr(app, 'Pipeline', lambda *a: Pipeline())
    monkeypatch.setattr(app, 'Controller', lambda *a: FaultController())
    monkeypatch.setattr(app, 'RuntimeDiagnostics', Writer)
    path = tmp_path/'run.jsonl'
    assert app.main(['run', '--mock', '--log-file', str(path)]) == 1
    assert order == ['STOP', 'control resources closed', 'log close']
    rows = [json.loads(x) for x in path.read_text().splitlines()]
    fault = next(x for x in rows if x.get('record_type') == 'terminal_fault')
    assert fault['status'] == 'original IK failure'
    assert 'timings' not in fault
    assert rows[-1]['record_type'] == 'exit'
