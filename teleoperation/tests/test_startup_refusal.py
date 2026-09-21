"""Startup refusals must describe the stage and actual attempted actions."""
from unittest import mock
from types import SimpleNamespace
import numpy as np
import pytest
import time
import test_command_trajectory as trajectory_tests
from esrobo_teleop.config import build_config
from esrobo_teleop.robot.nero_driver import NeroArm
from esrobo_teleop.teleop_node import TeleopNode, _run_until_safe_exit


@pytest.mark.parametrize('allow_disabled', [False, True])
def test_fresh_estop_remains_fault_even_when_disabled_is_allowed(allow_disabled):
    arm = NeroArm.__new__(NeroArm)
    arm._cfg = build_config().robot
    arm._robot = mock.Mock()
    arm._robot.get_arm_status.return_value = SimpleNamespace(
        timestamp=time.time(), msg=SimpleNamespace(arm_status=1, err_code=0, ctrl_mode=1))
    fault = arm.controller_fault(allow_disabled=allow_disabled)
    assert fault['category'] == 'controller_fault'
    assert fault['status_name'] == 'EMERGENCY_STOP'
    arm._robot.reset.assert_not_called()
    arm._robot.enable.assert_not_called()


def test_inherited_estop_refuses_before_commands_and_records_startup(capsys):
    driver = trajectory_tests.DriverTrajectoryTests().driver('left')
    driver._enabled = False
    driver._trajectory_preflight = mock.Mock(return_value=True)
    driver._arm.controller_fault.return_value = {
        'category': 'controller_fault', 'arm_status': 1, 'err_code': 0, 'ctrl_mode': 1}
    driver.read_joints = mock.Mock(return_value=np.zeros(7))
    records = []
    driver.startup_event_sink = records.append
    assert not driver.enable()
    assert driver.last_startup_failure['stage'] == 'controller_preflight'
    assert records[-1]['phase'] == 'startup_refused'
    assert records[-1]['failure'] == driver.last_startup_failure
    assert 'EMERGENCY_STOP' in driver.safety_fault_reason
    assert not driver._software_estop_recovery_pending
    for name in ('enable', 'reset', 'disable', 'move_j', 'set_motion_mode', 'set_normal_mode'):
        getattr(driver._arm, name).assert_not_called()
    node = TeleopNode.__new__(TeleopNode)
    node._driver, node._arm_side, node._motors_enabled = driver, 'left', False
    node._report_silent_startup_failure()
    output = capsys.readouterr().out
    assert 'not cleared E-stop' in output
    assert 'no wake, enable, position or disable commands' in output
    assert not node._motors_enabled


def test_isolated_disabled_estop_is_recognized_but_e_never_resets_it():
    driver = trajectory_tests.DriverTrajectoryTests().driver('left')
    driver._enabled = False
    driver._trajectory_preflight = mock.Mock(return_value=True)
    _, zero = driver._mapping('left')
    driver.read_joints = mock.Mock(return_value=zero.copy())
    driver._arm.get_joint_enable_states.return_value = [False] * 7

    def controller_fault(*, allow_disabled=False, allow_estop=False):
        if allow_disabled and allow_estop:
            return None
        return {'category': 'controller_fault', 'arm_status': 1,
                'err_code': 0, 'ctrl_mode': 1, 'status_name': 'EMERGENCY_STOP'}

    driver._arm.controller_fault.side_effect = controller_fault
    assert not driver.enable()
    assert driver._inherited_estop_recovery_pending
    assert driver.last_startup_failure['stage'] == 'controller_preflight'
    assert 'Press z once' in driver.last_startup_failure['reason']
    driver._arm.reset.assert_not_called()
    driver._arm.enable.assert_not_called()
    driver._arm.move_j.assert_not_called()


@pytest.mark.parametrize('context', [
    {'reason':'missing/stale hand feedback','hand_stamp':None},
    {'pair':['left_thumb_finger_envelope','waist_link1'],'clearance_m':-.01,'required_m':.03},
])
def test_geometry_refusal_does_not_wake_or_disable_and_keeps_detail(context,capsys):
    driver = trajectory_tests.DriverTrajectoryTests().driver('left')
    driver._silent_disabled_startup = True
    driver._enabled = False
    driver._collision_guard = SimpleNamespace(last_diagnostics=context)
    driver._check_physical_path = mock.Mock(return_value=False)
    driver._trajectory_preflight = mock.Mock(return_value=True)
    events=[];driver.probe_sink=events.append
    assert not driver.enable_from_natural_down()
    driver._arm.enable_at_known_target.assert_not_called()
    driver._arm.best_effort_disable_once.assert_not_called()
    driver._arm.disable.assert_not_called()
    driver._arm.move_j.assert_not_called()
    assert driver.last_startup_failure['stage'] == 'geometry'
    assert driver.last_startup_failure['geometry'] == context
    assert driver.trajectory_diagnostics['fault_context'] == context
    assert events[-1]['event'] == 'startup_refused'
    node = TeleopNode.__new__(TeleopNode)
    node._driver, node._arm_side, node._motors_enabled = driver,'left',False
    node._report_silent_startup_failure()
    output=capsys.readouterr().out
    assert 'Rejected before controller wake' in output
    assert 'disable commands were sent' not in output
    assert 'no complete seven-joint feedback' not in output
    assert node._manual_intervention_required and not node._motors_enabled


def test_actual_wake_failure_reports_attempted_disable(capsys):
    driver = trajectory_tests.DriverTrajectoryTests().driver('left')
    driver._silent_disabled_startup = True
    driver._arm.enable_at_known_target.return_value = (None,None)
    assert not driver.enable_from_natural_down()
    assert driver.last_startup_failure['stage'] == 'wake_feedback'
    assert driver.last_startup_failure['wake_attempted']
    assert driver.last_startup_failure['disable_attempted']
    assert driver.last_startup_failure['power_cycle_required']
    driver._arm.best_effort_disable_once.assert_called_once()
    node = TeleopNode.__new__(TeleopNode)
    node._driver,node._arm_side,node._motors_enabled=driver,'left',False
    node._report_silent_startup_failure()
    output=capsys.readouterr().out
    assert 'Disable commands were attempted' in output
    assert 'POWER-CYCLE REQUIRED' in output
    assert 'Rejected before controller wake' not in output
    assert node._motors_enabled  # Unknown is conservatively treated as possibly enabled.
    assert node._startup_power_cycle_required


def test_silent_wake_can_enobufs_is_structured_transport_failure(capsys):
    driver = trajectory_tests.DriverTrajectoryTests().driver('left')
    driver._silent_disabled_startup = True
    driver._arm.enable_at_known_target.side_effect = RuntimeError(
        "CAN socket send failed: No buffer space available [Error Code 105]"
    )

    assert not driver.enable_from_natural_down()
    failure = driver.last_startup_failure
    assert failure['stage'] == 'can_transport'
    assert failure['wake_attempted']
    assert failure['power_cycle_required']
    assert failure['transport_unavailable']
    driver._arm.best_effort_disable_once.assert_not_called()

    node = TeleopNode.__new__(TeleopNode)
    node._driver, node._arm_side, node._motors_enabled = driver, 'left', False
    node._report_silent_startup_failure()
    output = capsys.readouterr().out
    assert 'CAN TRANSPORT UNAVAILABLE' in output
    assert node._startup_power_cycle_required
    assert node._startup_transport_unavailable
    assert node._motors_enabled  # An attempted wake leaves hardware state unknown.


def test_transport_unavailable_q_exits_when_estop_cannot_be_delivered(capsys):
    node = TeleopNode.__new__(TeleopNode)
    node._arm_only = True
    node._arm_side = 'left'
    node._arm_armed = False
    node._motors_enabled = True
    node._manual_intervention_required = True
    node._startup_power_cycle_required = True
    node._startup_transport_unavailable = True
    node._stop = False
    node._driver = mock.Mock()
    node._driver.emergency_stop.side_effect = RuntimeError('CAN unavailable')

    node._handle_key('q')

    assert node._stop
    node._driver.prepare_return.assert_not_called()
    assert 'motor state remains unverified' in capsys.readouterr().out


def test_complete_but_invalid_wake_state_does_not_claim_power_cycle_required():
    driver = trajectory_tests.DriverTrajectoryTests().driver('left')
    driver._silent_disabled_startup = True
    _, zero = driver._mapping('left')
    driver._arm.enable_at_known_target.return_value = (zero.copy(), [False] * 7)

    assert not driver.enable_from_natural_down()
    assert not driver.last_startup_failure['power_cycle_required']


def test_power_cycle_required_q_estops_and_exits_without_return(capsys):
    node = TeleopNode.__new__(TeleopNode)
    node._arm_only = True
    node._arm_side = 'left'
    node._arm_armed = False
    node._motors_enabled = True
    node._manual_intervention_required = True
    node._startup_power_cycle_required = True
    node._stop = False
    node._driver = mock.Mock()

    node._handle_key('q')

    assert node._stop
    assert not node._motors_enabled
    node._driver.emergency_stop.assert_called_once_with()
    node._driver.prepare_return.assert_not_called()
    assert 'no return position was commanded' in capsys.readouterr().out


def test_power_cycle_required_blocks_repeated_enable(capsys):
    node = TeleopNode.__new__(TeleopNode)
    node._arm_side = 'left'
    node._startup_power_cycle_required = True

    assert not node._arm_robot()
    assert 'still requires a power-cycle' in capsys.readouterr().out


@pytest.mark.parametrize('key', ['e', 's', 'z'])
def test_power_cycle_required_keys_never_start_enable_or_return(key, capsys):
    node = TeleopNode.__new__(TeleopNode)
    node._arm_only = True
    node._arm_side = 'left'
    node._startup_power_cycle_required = True
    node._driver = mock.Mock()

    node._handle_key(key)

    node._driver.prepare_return.assert_not_called()
    node._driver.safe_return.assert_not_called()
    node._driver.enable.assert_not_called()
    assert 'requires a power-cycle' in capsys.readouterr().out


def test_ctrl_c_uses_power_cycle_exit_path():
    node = mock.Mock()
    node.run.side_effect = KeyboardInterrupt()
    node._startup_power_cycle_required = True
    node._exit_for_startup_power_cycle.return_value = True

    _run_until_safe_exit(node)

    node._exit_for_startup_power_cycle.assert_called_once_with(
        'Ctrl-C after startup feedback failure'
    )
    node._enabled_fault_requires_explicit_disable.assert_not_called()


def test_right_measured_startup_reports_missing_hand_before_any_hardware_write(capsys):
    driver = trajectory_tests.DriverTrajectoryTests().driver('right')
    context = {'reason':'missing/stale hand feedback','hand_stamp':None}
    driver._trajectory_preflight = mock.Mock(return_value=True)
    driver._collision_guard = SimpleNamespace(last_diagnostics=context)
    driver._check_physical_path = mock.Mock(return_value=False)
    assert not driver.enable()
    driver._arm.enable.assert_not_called()
    driver._arm.move_j.assert_not_called()
    driver._arm.disable.assert_not_called()
    driver._arm.set_normal_mode.assert_not_called()
    assert driver.last_startup_failure['geometry'] == context
    node = TeleopNode.__new__(TeleopNode)
    node._driver,node._arm_side,node._motors_enabled=driver,'right',False
    node._report_silent_startup_failure()
    output=capsys.readouterr().out
    assert 'missing/stale hand feedback' in output
    assert 'Rejected before controller wake' in output
