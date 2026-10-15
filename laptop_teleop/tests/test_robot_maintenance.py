"""Supervised SDK commissioning with mocked hardware."""
import numpy as np
import pytest
from unittest.mock import Mock


@pytest.mark.parametrize('case', ['success', 'missing', 'wrong_pose', 'fault', 'exception', 'disable_missing'])
def test_supervised_sdk_wake_never_skips_feedback_or_cleanup(case):
    from esrobo_laptop.robot_maintenance import wake_confirmed_left_zero
    zero = np.array([0, np.pi/2, 0, 0, 0, 0, 0])
    arm = Mock()
    arm.get_joint_angles.return_value = zero+.1 if case == 'wrong_pose' else None
    arm.get_joint_enable_states.side_effect = [None, [False]*7]
    arm.controller_fault.return_value = {'category':'driver_fault'} if case == 'fault' else None
    arm.enable_at_known_target.return_value = (None, None) if case == 'missing' else (zero, [True]*7)
    if case == 'exception': arm.enable_at_known_target.side_effect = RuntimeError('send failed')
    arm.disable.return_value = case != 'disable_missing'
    result = wake_confirmed_left_zero(arm, zero, .02)
    if case in ('wrong_pose', 'fault'):
        arm.enable_at_known_target.assert_not_called()
        arm.disable.assert_not_called()
    else:
        arm.enable_at_known_target.assert_called_once()
        np.testing.assert_array_equal(arm.enable_at_known_target.call_args.args[0], zero)
        assert arm.enable_at_known_target.call_args.kwargs == dict(speed_percent=5, timeout=3.)
        arm.disable.assert_called_once_with(timeout=3.)
    assert result['zero_verified'] == (case in ('success', 'disable_missing'))
    if case == 'disable_missing':
        arm.best_effort_disable_once.assert_called_once()
        assert not result['disabled']


def test_wake_script_is_fixed_and_uses_exclusive_robot_lock():
    from esrobo_laptop.robot_maintenance import wake_left_zero_script
    script = wake_left_zero_script('/home/esrobo/Projects/esrobo')
    compile(script, '<wake-script>', 'exec')
    assert 'LOCK_EX|fcntl.LOCK_NB' in script
    assert '"can_piper1", "left"' in script
    assert 'cfg.left_joint_offsets' in script
