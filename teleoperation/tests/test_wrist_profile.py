"""Unified wrist limits: trajectory, return construction, and contract checks."""
import dataclasses
from pathlib import Path
from unittest import mock

import numpy as np
import pytest

from esrobo_teleop.config import build_config, load_config
from esrobo_teleop.robot.command_trajectory import CommandTrajectory
from esrobo_teleop.robot.nero_driver import NeroSingleArmDriver
from scripts.compare_wrist_profiles import simulate
from test_safe_return import make_driver

ROOT = Path(__file__).resolve().parents[2]


@pytest.mark.parametrize('side', ['left', 'right'])
def test_config_default_and_yaml_match_all_phase_limits(side):
    for cfg in (build_config().robot, load_config(ROOT/'teleoperation/config/teleop_config.yaml').robot):
        with mock.patch('esrobo_teleop.robot.nero_driver.NeroArm'):
            driver = NeroSingleArmDriver(cfg, side)
        np.testing.assert_allclose(np.degrees(driver._trajectory.vmax[4:]), 36., atol=.0001)
        np.testing.assert_allclose(np.degrees(driver._trajectory.amax[4:]), 100., atol=.0001)
        np.testing.assert_allclose(np.degrees(driver._trajectory.lead[4:]), 10.)
        np.testing.assert_allclose(driver._trajectory.vmax, getattr(cfg, side+'_max_joint_velocity'))
        assert cfg.speed_percent == 30
        assert 'return_speed_scale' not in dataclasses.asdict(cfg)


@pytest.mark.parametrize('side', ['left', 'right'])
def test_actual_return_uses_same_limits_and_endpoint_speed(side):
    driver, feedback, zero = make_driver(side, np.full(7, .04))
    constructed = []
    speeds = []
    def construct(*args, **kwargs):
        result = CommandTrajectory(*args, **kwargs)
        constructed.append(result)
        propose = result.propose
        def record(*a, **kw):
            speeds.append(a[6])
            return propose(*a, **kw)
        result.propose = record
        return result
    with mock.patch('esrobo_teleop.robot.nero_driver.CommandTrajectory', side_effect=construct), \
            mock.patch('esrobo_teleop.robot.nero_driver.time.sleep'):
        result = driver.safe_return()
    assert result.returned and result.disabled, result
    assert constructed and speeds
    for tr in constructed:
        np.testing.assert_array_equal(tr.vmax, driver._trajectory.vmax)
        np.testing.assert_array_equal(tr.amax, driver._trajectory.amax)
        np.testing.assert_array_equal(tr.lead, driver._trajectory.lead)
    assert all(speed == driver._trajectory_endpoint_speed for speed in speeds)


@pytest.mark.parametrize('side', ['left', 'right'])
def test_full_speed_hold_brakes_without_reset_before_return(side):
    driver, feedback, zero = make_driver(side, np.full(7, .04))
    driver._trajectory.reset(feedback, driver._arm.last_feedback_timestamp)
    velocity = np.zeros(7); velocity[4] = driver._trajectory.vmax[4]
    driver._trajectory.state = dataclasses.replace(driver._trajectory.state, velocity=velocity)
    driver._first_follow_hold = False
    original_brake = driver._brake_held_trajectory_for_return
    captured = []
    def brake(timeout):
        np.testing.assert_array_equal(driver._trajectory.state.velocity, velocity)
        original_commit = driver._trajectory.commit
        def commit(proposal):
            before = driver._trajectory.state
            dt = proposal.stamp-before.stamp
            assert np.all(abs(proposal.velocity-before.velocity) <= driver._trajectory.amax*dt+1e-8)
            assert np.all(abs(proposal.position-feedback) <= driver._trajectory.lead+1e-8)
            captured.append(proposal)
            original_commit(proposal)
        with mock.patch.object(driver._trajectory, 'reset', wraps=driver._trajectory.reset) as reset, \
                mock.patch.object(driver._trajectory, 'commit', side_effect=commit):
            ok = original_brake(timeout)
            reset.assert_not_called()
        assert ok
        assert np.max(abs(driver._trajectory.state.velocity)) <= 1e-8
        return ok
    driver._brake_held_trajectory_for_return = brake
    with mock.patch('esrobo_teleop.robot.nero_driver.time.sleep'):
        result = driver.safe_return(preserve_hold=True)
    assert captured and result.returned and result.disabled, result


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('tau', [.03, .05, .08])
@pytest.mark.parametrize('case', ['step', 'reverse', 'input_gap', 'stop', 'stalled'])
def test_candidate_and_baseline_motion_invariants(side, tau, case):
    cfg = build_config().robot
    old = simulate(cfg, side, True, tau, case)
    new = simulate(cfg, side, False, tau, case)
    for result in (old, new):
        if case == 'stalled':
            assert result['fault'] and 'persistent feedback lead' in result['fault']['reason']
        else:
            assert result['fault'] is None, result
        if case == 'stop':
            assert result['command_stop_travel_deg'] is not None
    if case == 'step':
        assert new['settling_s'] < old['settling_s'] * .8


def test_wrist_handover_and_fingerprint_use_unified_limits():
    from esrobo_laptop.wrist_handover import WristHandover
    from esrobo_laptop.contract import local_id
    cfg = load_config(ROOT/'teleoperation/config/teleop_config.yaml')
    cfg.ik.urdf_path = str(ROOT/'teleoperation'/cfg.ik.urdf_path)
    for side in ('left', 'right'):
        handover = WristHandover(np.zeros(7), cfg.robot, side, 0., 50)
        np.testing.assert_allclose(np.degrees(handover.velocity), 36., atol=.0001)
        np.testing.assert_allclose(np.degrees(handover.lead), 10.)
        current = local_id(cfg, side, True)
        old = dataclasses.replace(cfg, robot=dataclasses.replace(cfg.robot,
            command_trajectory_lead_deg=(3,3,3,3,1.5,1.5,1.5)))
        assert current != local_id(old, side, True)
