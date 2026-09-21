import threading
import time

import pytest

from esrobo_teleop.debug.head_control import HeadControl


class Adapter:
    def __init__(self):
        self.positions = [1500, 3450]
        self.enabled = False
        self.age = 0
        self.sequence = 0
        self.writes = []
        self.freeze = False
        self.failure = False

    def state(self):
        self.sequence += 1
        return dict(sequence=self.sequence, age_s=self.age, axes={str(i + 1): dict(
            valid='true', position_ticks=str(p), torque_enabled='1', pending='false',
            allow_motion='true', adjustment_enabled=str(self.enabled).lower(), arrival_tolerance_ticks='5'
        ) for i, p in enumerate(self.positions)})

    def gate(self, enabled):
        self.writes.append(('gate', enabled))
        self.enabled = enabled

    def jog(self, sid, delta, speed):
        self.writes.append(('jog', sid, delta, speed))
        if self.failure:
            raise RuntimeError('lost acknowledgement')
        if not self.freeze:
            self.positions[sid - 1] += delta
        time.sleep(.01)


def wait(control):
    control.worker.join(timeout=3)
    assert not control.busy


def test_continuous_targets_do_not_auto_lock():
    adapter = Adapter()
    control = HeadControl(adapter)
    assert adapter.writes == []
    control.submit('enable', 'operator1')
    wait(control)
    for target in (1530, 1900, 2700, 1000, 1500):
        control.submit('move', 'operator1', 1, target)
        wait(control)
        assert abs(adapter.positions[0] - target) <= 5
        assert control.owner == 'operator1'
    assert all(abs(w[2]) <= 10 and w[3] == 5 for w in adapter.writes if w[0] == 'jog')
    control.close()
    assert adapter.enabled is False


def test_stale_feedback_and_send_failure_lock():
    adapter = Adapter()
    control = HeadControl(adapter)
    control.submit('enable', 'operator1')
    wait(control)
    adapter.age = 1
    control.submit('move', 'operator1', 1, 1530)
    wait(control)
    assert control.owner is None
    assert not any(w[0] == 'jog' for w in adapter.writes)
    adapter.age = 0
    control.submit('enable', 'operator1')
    wait(control)
    adapter.failure = True
    control.submit('move', 'operator1', 1, 1530)
    wait(control)
    assert control.owner is None and not adapter.enabled


def test_lease_expiry_and_other_operator():
    adapter = Adapter()
    control = HeadControl(adapter)
    control.submit('enable', 'operator1')
    wait(control)
    with pytest.raises(RuntimeError):
        control.submit('move', 'operator2', 1, 1520)
    control.heartbeat_at -= 4
    control.expire()
    assert control.owner is None and not adapter.enabled


def test_limits_busy_and_explicit_lock_cancel():
    adapter = Adapter()
    control = HeadControl(adapter)
    control.submit('enable', 'operator1')
    wait(control)
    for sid, target in ((0, 1500), (1, 999), (2, 5001), (1, 1500.5)):
        with pytest.raises(ValueError):
            control.submit('move', 'operator1', sid, target)
    control.submit('move', 'operator1', 1, 1800)
    wait(control)
    assert control.owner == 'operator1' and adapter.positions[0] == 1800
    adapter.freeze = True
    control.submit('move', 'operator1', 1, 1530)
    with pytest.raises(RuntimeError):
        control.submit('move', 'operator1', 1, 1520)
    control.lock()
    wait(control)
    count = len(adapter.writes)
    time.sleep(.05)
    assert len(adapter.writes) == count and not adapter.enabled


def test_lock_during_enable_does_not_leave_enabled():
    adapter = Adapter()
    original = adapter.gate
    started = threading.Event()
    def delayed(value):
        if value:
            started.set()
            time.sleep(.1)
        original(value)
    adapter.gate = delayed
    control = HeadControl(adapter)
    control.submit('enable', 'operator1')
    assert started.wait(1)
    control.lock()
    wait(control)
    assert not adapter.enabled and control.owner is None


def test_enable_waits_for_new_enabled_feedback():
    adapter = Adapter()
    original = adapter.state
    remaining = [5]
    def delayed_state():
        data = original()
        if adapter.enabled and remaining[0] > 0:
            remaining[0] -= 1
            for axis in data['axes'].values():
                axis['adjustment_enabled'] = 'false'
                axis['torque_enabled'] = '0'
        return data
    adapter.state = delayed_state
    control = HeadControl(adapter)
    control.submit('enable', 'operator1')
    wait(control)
    assert remaining[0] == 0 and control.owner == 'operator1'
    assert adapter.enabled and control.message == '调整已开启'
    control.close()
