"""Offline limits and feedback checks for the independent waist buttons."""
import json
import pytest

from esrobo_link.waist_control import (
    JOG_ACCEL_DEG_S2, MAX_JOG_SPEED_DEG_S, TARGET_ACCEL_DEG_S2,
    TARGET_SPEED_DEG_S, checked_absolute_target, checked_jog_target, parse_feedback,
)
from esrobo_link import waist_control


def sample():
    return [31, 120, 0, 0, 25, 32, 260, 0, 0, 25, 33, 170, 0, 0, 25]


def test_three_axis_feedback_and_small_jog():
    axes = parse_feedback(sample(), [31, 0, 32, 0, 33, 0])
    assert checked_jog_target(axes, 33, 1) == 171
    assert checked_jog_target(axes, 31, -1) == 119


def test_feedback_and_jog_fail_closed():
    with pytest.raises(ValueError):
        parse_feedback(sample()[:-5], [31, 0, 32, 0, 33, 0])
    with pytest.raises(ValueError):
        parse_feedback(sample(), [31, 0, 31, 0, 33, 0])
    axes = parse_feedback(sample(), [31, 0, 32, 0, 33, 0])
    axes[31]['position_deg'] = 359.5
    with pytest.raises(RuntimeError, match='单圈范围'):
        checked_jog_target(axes, 31, 1)
    axes[31]['position_deg'] = 120
    axes[32]['error'] = 0x0C26
    assert checked_jog_target(axes, 33, -1) == 169
    axes[32]['error'] = 0x1234
    with pytest.raises(RuntimeError, match='0x1234'):
        checked_jog_target(axes, 33, -1)


def test_full_error_code_is_not_truncated():
    axes = parse_feedback(sample(), [31, 0x0C26, 32, 0, 33, 0])
    assert axes[31]['error'] == 3110
    with pytest.raises(ValueError, match='错误码无效'):
        parse_feedback(sample(), [31, -1, 32, 0, 33, 0])


def test_direct_target_is_slower_and_respects_each_axis_limit():
    assert TARGET_SPEED_DEG_S == 4.0
    assert TARGET_ACCEL_DEG_S2 == 10.0
    assert MAX_JOG_SPEED_DEG_S == 5.0
    assert JOG_ACCEL_DEG_S2 == 12.0
    assert TARGET_SPEED_DEG_S < MAX_JOG_SPEED_DEG_S
    assert TARGET_ACCEL_DEG_S2 < JOG_ACCEL_DEG_S2
    axes = parse_feedback(sample(), [31, 0, 32, 0, 33, 0])
    assert checked_absolute_target(axes, 31, 124.5) == 124.5
    assert checked_absolute_target(axes, 32, 255.0) == 255.0
    with pytest.raises(RuntimeError, match='超出当前反馈'):
        checked_absolute_target(axes, 33, 175.1)
    axes[32]['position_deg'] = 255.0
    assert checked_absolute_target(axes, 32, 250.0) == 250.0
    with pytest.raises(RuntimeError, match='超出当前反馈'):
        checked_absolute_target(axes, 32, 249.9)
    axes[32]['position_deg'] = 250.0
    assert checked_absolute_target(axes, 32, 245.0) == 245.0
    axes[32]['position_deg'] = 254.9966
    assert checked_absolute_target(axes, 32, 260.0) == 260.0
    with pytest.raises(RuntimeError, match='超出当前反馈'):
        checked_absolute_target(axes, 32, 260.01)
    with pytest.raises(RuntimeError, match='单圈范围'):
        checked_absolute_target(axes, 32, -1.0)
    with pytest.raises(ValueError, match='目标角度无效'):
        checked_absolute_target(axes, 31, float('nan'))
    with pytest.raises(ValueError, match='目标角度无效'):
        checked_absolute_target(axes, 31, True)
    axes[32]['error'] = 0x0C26
    assert checked_absolute_target(axes, 31, 124.5) == 124.5
    axes[32]['error'] = 0x1234
    with pytest.raises(RuntimeError, match='0x1234'):
        checked_absolute_target(axes, 31, 124.5)


@pytest.mark.parametrize('sent', [False, True])
def test_command_error_reports_whether_publish_may_have_started(monkeypatch, capsys, sent):
    def fail(_action, _axis, _direction, _target_deg, command_state):
        command_state['sent'] = sent
        raise RuntimeError('simulated failure')
    monkeypatch.setattr(waist_control, 'run', fail)
    assert waist_control.main(['target', '--axis', '33', '--target-deg', '161.97']) == 1
    result = json.loads(capsys.readouterr().out)
    assert result == dict(ok=False, error='simulated failure', command_sent=sent)
