"""Phase-specific following limits; all hardware and feedback are simulated."""
from dataclasses import replace
import time
from unittest import mock

import numpy as np
import pytest

from esrobo_teleop.config import build_config, load_config
from esrobo_teleop.robot.command_trajectory import CommandTrajectory, TrajectoryFault
from esrobo_teleop.robot.nero_driver import JointFeedbackSnapshot, NeroSingleArmDriver
from test_safe_return import make_driver


def activate(d, feedback):
    d._trajectory.reset(feedback, d._arm.last_feedback_timestamp)
    d._activation_candidate = feedback.copy()
    def read():
        d._cycle_feedback = JointFeedbackSnapshot(feedback.copy(),
            d._arm.last_feedback_timestamp, time.monotonic(), 'test')
        return np.zeros(14)
    with mock.patch.object(d, 'read_control_cycle_joints', side_effect=read), \
            mock.patch.object(d, '_check_physical_path', return_value=True):
        state = d._trajectory.state
        assert d.verify_tracking_activation(feedback.copy(), feedback.copy())
        assert d._trajectory.state is state
    d._first_follow_hold = False


@pytest.mark.parametrize('side', ['left', 'right'])
def test_activation_widens_only_after_verification_and_reentry_restores_limit(side):
    d, feedback, _ = make_driver(side)
    np.testing.assert_allclose(np.degrees(d._trajectory.lead), 10)
    activate(d, feedback)
    np.testing.assert_allclose(np.degrees(d._trajectory.lead), 15)
    assert d._trajectory.lead_timeout == 2
    # A completed session can initialize a new stationary entry, still at 10.
    with mock.patch.object(d, '_certify_tracking_boundary', return_value=(feedback.copy(), 11.)):
        assert d.initialize_command_trajectory()
    np.testing.assert_allclose(np.degrees(d._trajectory.lead), 10)


@pytest.mark.parametrize('side', ['left', 'right'])
def test_failed_takeover_does_not_enable_wider_limit(side):
    d, feedback, _ = make_driver(side)
    d._trajectory.reset(feedback, 10.)
    d._activation_candidate = feedback.copy()
    sample = JointFeedbackSnapshot(feedback.copy(), 10., time.monotonic()-.3, 'test')
    def read():
        d._cycle_feedback = sample
        return np.zeros(14)
    with mock.patch.object(d, 'read_control_cycle_joints', side_effect=read), \
            mock.patch.object(d, '_check_physical_path', return_value=True):
        assert not d.verify_tracking_activation(feedback, feedback)
    np.testing.assert_allclose(np.degrees(d._trajectory.lead), 10)


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('action', ['hold', 'recover', 'return'])
def test_twelve_degree_live_error_can_brake_recover_or_return_without_shrinking(side, action):
    d, feedback, zero = make_driver(side)
    activate(d, feedback)
    position = feedback.copy()
    position[4] += np.deg2rad(12)
    d._trajectory.reset(position, d._arm.last_feedback_timestamp, feedback=feedback)
    direction, offset = d._mapping(side)
    full = np.zeros(14)
    start = 0 if side == 'left' else 7
    full[start:start+7] = (position-offset)/direction
    d._last_commanded_full_urdf = full
    if action == 'recover':
        sample = JointFeedbackSnapshot(feedback.copy(), 10.2, time.monotonic(), 'test')
        assert d._recover_command_trajectory_after_feedback_gap(sample)
        np.testing.assert_array_equal(d._trajectory.state.position, position)
        d._arm.move_j.assert_not_called()
    else:
        velocity = np.zeros(7)
        velocity[4] = .05
        d._trajectory.state = replace(d._trajectory.state, velocity=velocity)
        original_commit = d._trajectory.commit
        def commit(step):
            before = d._trajectory.state
            np.testing.assert_allclose(np.degrees(d._trajectory.lead), 15)
            assert np.all(abs(step.velocity-before.velocity) <= d._trajectory.amax*(step.stamp-before.stamp)+1e-8)
            assert np.all(abs(step.position-feedback) <= d._trajectory.lead+1e-8)
            original_commit(step)
        with mock.patch.object(d._trajectory, 'reset', wraps=d._trajectory.reset) as reset, \
                mock.patch.object(d._trajectory, 'commit', side_effect=commit), \
                mock.patch('esrobo_teleop.robot.nero_driver.time.sleep'):
            if action == 'hold':
                d.read_joints()
                # Model the new control-cycle sample, not the cached entry
                # sample (same timestamp as reset, correctly skipped by driver).
                d._cycle_feedback = JointFeedbackSnapshot(feedback.copy(),
                    d._arm.last_feedback_timestamp, time.monotonic(), 'test')
                assert d.hold_command_trajectory()
                assert d._trajectory.commit.call_count == 1
                np.testing.assert_allclose(d.motion_limit_diagnostics['limits']['lead_deg'], 15)
                reset.assert_not_called()
            else:
                # The live brake retains 15; the independently constructed
                # return trajectory must use 10 for every return segment.
                with mock.patch('esrobo_teleop.robot.nero_driver.CommandTrajectory', wraps=CommandTrajectory) as factory:
                    result = d.safe_return(preserve_hold=True)
                assert result.returned and result.disabled, result
                assert factory.call_count
                for call in factory.call_args_list:
                    np.testing.assert_allclose(np.degrees(call.args[3]), 10)
    np.testing.assert_allclose(np.degrees(d._trajectory.lead), 15)


@pytest.mark.parametrize('side', ['left', 'right'])
def test_live_stall_is_bounded_by_fifteen_degrees_and_original_timeout(side):
    d, feedback, _ = make_driver(side)
    activate(d, feedback)
    p = d._trajectory
    target = feedback + np.deg2rad(40)
    largest = 0.
    with pytest.raises(TrajectoryFault, match='persistent feedback lead'):
        for _ in range(300):
            step = p.propose(target, feedback, p.state.stamp+.02,
                np.full(7, -4.), np.full(7, 4.))
            largest = max(largest, np.max(abs(step.position-feedback)))
            assert np.all(abs(step.position-feedback) <= np.deg2rad(15)+1e-8)
            p.commit(step)
    assert largest > np.deg2rad(10)
    assert p.lead_timeout == 2.


@pytest.mark.parametrize('bad', [[15]*6, [15]*6+[0], [15]*6+[float('nan')]])
def test_invalid_live_limit_rejected_before_hardware_construction(bad):
    cfg = build_config().robot
    cfg.tracking_command_trajectory_lead_deg = bad
    with mock.patch('esrobo_teleop.robot.nero_driver.NeroArm') as arm:
        with pytest.raises(ValueError):
            NeroSingleArmDriver(cfg, 'left')
        arm.assert_not_called()


def test_live_limit_changes_contract_and_yaml_agrees_with_defaults():
    from pathlib import Path
    from esrobo_laptop.contract import local_id
    cfg = load_config(str(Path(__file__).parents[1]/'config/teleop_config.yaml'))
    np.testing.assert_array_equal(cfg.robot.command_trajectory_lead_deg, [10]*7)
    np.testing.assert_array_equal(cfg.robot.tracking_command_trajectory_lead_deg,
                                 build_config().robot.tracking_command_trajectory_lead_deg)
    cfg.ik.urdf_path = str(Path(__file__).parents[1]/'urdf/esrobo_waist_with_head.urdf')
    before = local_id(cfg, 'left', True)
    cfg.robot.tracking_command_trajectory_lead_deg = [10]*7
    assert local_id(cfg, 'left', True) != before
