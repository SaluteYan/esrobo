import json
from pathlib import Path
from unittest import mock

import numpy as np
import pytest

from esrobo_teleop.robot.command_backpressure import (
    CommandBackpressure,
    MoveJWaypointGate,
    joint_wire_key,
)
from esrobo_teleop.robot.command_trajectory import TrajectoryFault, TrajectoryStep
from esrobo_teleop.config import build_config
from esrobo_teleop.ik.solver import IkSolver
import test_command_trajectory as trajectory_tests


def state(q=.5, velocity=0.):
    return TrajectoryStep(np.full(7, np.deg2rad(q)), np.full(7, velocity), 0.,
                          np.full(7, np.nan), {})


def test_pause_and_timeout_use_progress_per_joint_not_packet_arrival():
    gate = CommandBackpressure()
    failure = None
    for i in range(130):
        stamp = 10. + i * .02
        feedback = np.zeros(7)
        # J2 moves toward its target; it cannot hide stationary J1/J3..J7.
        feedback[1] = np.deg2rad(min(.5, i * .01))
        try:
            paused = gate.evaluate(state(), feedback, stamp)
            assert paused == (i >= 8)
        except TrajectoryFault as error:
            failure = error
            break
    assert failure is not None
    assert failure.diagnostics['failed_joints'] == [1, 3, 4, 5, 6, 7]
    assert failure.diagnostics['fault_code'] == 'persistent_feedback_lead'
    # The 2 s window starts when pausing begins, not at the first tiny lead.
    assert i >= 108


def test_stall_timeout_starts_after_braking_reaches_last_wire_position():
    gate = CommandBackpressure(timeout=2.)
    feedback = np.zeros(7)
    # Reproduce the latest run: the command continues braking for about
    # 0.45 s after lead first appears, then the controller begins moving
    # 1.70 s after the final distinct payload.
    for i in range(24):
        q = min(2., i * .1)
        gate.evaluate(state(q=q), feedback, 10. + i * .02)
    assert gate.paused
    stable_stamp = 10.46
    for i in range(1, 86):
        assert gate.evaluate(state(q=2.), feedback, stable_stamp + i * .02)
    feedback[:] = np.deg2rad(.03)
    assert gate.evaluate(state(q=2.), feedback, stable_stamp + 1.72)
    assert max(gate.diagnostics['no_progress_s']) == pytest.approx(0.)


def test_left_runtime_uses_same_continuous_move_j_submission_as_right():
    harness = trajectory_tests.DriverTrajectoryTests()
    driver = harness.driver('left')
    assert driver._move_j_waypoint_gate is None
    assert driver._backpressure is None
    target = np.zeros(14)
    target[3] = np.deg2rad(30)
    for i in range(1, 41):
        driver._arm.last_feedback_timestamp = 10. + i * .02
        assert driver.command_full_urdf(target), driver.safety_fault_reason
    assert driver._arm.move_j.call_count == 40
    assert driver.trajectory_diagnostics['position_frame_sent'] is True
    driver._arm.emergency_stop.assert_not_called()


def test_left_continuous_move_j_reverses_through_acceleration_limited_trajectory():
    harness = trajectory_tests.DriverTrajectoryTests()
    driver = harness.driver('left')
    target = np.zeros(14)
    target[0] = np.deg2rad(20)
    for i in range(1, 30):
        driver._arm.last_feedback_timestamp = 10. + i * .02
        assert driver.command_full_urdf(target), driver.safety_fault_reason
    velocity_before = driver._trajectory.state.velocity[0]
    assert velocity_before > 0

    reverse = np.zeros(14)
    reverse[0] = np.deg2rad(-20)
    previous = driver._trajectory.state
    for i in range(30, 45):
        driver._arm.last_feedback_timestamp = 10. + i * .02
        assert driver.command_full_urdf(reverse), driver.safety_fault_reason
        current = driver._trajectory.state
        assert abs(current.velocity[0] - previous.velocity[0]) <= (
            driver._trajectory.amax[0] * .02 + 1e-7
        )
        previous = current
    assert driver._arm.move_j.call_count == 44
    assert driver._trajectory.state.velocity[0] < velocity_before


def test_resume_needs_position_velocity_and_three_distinct_settled_samples():
    gate = CommandBackpressure()
    gate.evaluate(state(), np.zeros(7), 10.)
    assert gate.evaluate(state(), np.zeros(7), 10.2)
    target = state().position
    assert gate.evaluate(state(velocity=.1), target, 10.22)
    assert gate.evaluate(state(velocity=.1), target, 10.24)
    assert gate.settled == 0
    assert gate.evaluate(state(), target, 10.26)
    assert gate.evaluate(state(), target, 10.26)  # Duplicate cannot settle.
    assert gate.settled == 1
    assert gate.evaluate(state(), target, 10.28)
    assert not gate.evaluate(state(), target, 10.30)


def test_progress_avoids_false_pause_and_reset_clears_old_session():
    gate = CommandBackpressure()
    for i in range(50):
        feedback = np.full(7, np.deg2rad(i * .025))
        q = state(q=i * .025 + .2)
        assert not gate.evaluate(q, feedback, 10. + i * .02)
    gate.reset()
    assert not gate.evaluate(state(), np.zeros(7), 1.)


def test_wire_key_uses_sdk_rounding():
    q = np.deg2rad([.00049, -.00049, .00151, -.00151, 90., 0., 120.])
    assert joint_wire_key(q) == (0, 0, 2, -2, 90000, 0, 120000)


def test_latest_stall_replay_continuous_stream_keeps_feedback_lead_fault():
    fixture = json.loads((Path(__file__).parent / 'fixtures/left_stall_without_mode_frames_20260920.json').read_text())
    harness = trajectory_tests.DriverTrajectoryTests()
    d = harness.driver('left')
    ik_config = build_config().ik
    ik_config.urdf_path = str(Path(__file__).resolve().parents[1] / 'urdf/esrobo_waist_with_head.urdf')
    solver = IkSolver(ik_config)
    d.configure_command_trajectory(solver.make_command_fk('left'), .22)
    first = fixture['samples'][0]
    origin = first['stamp']
    harness.current[:] = first['feedback']
    d._arm.last_feedback_timestamp = 10.
    assert d.initialize_command_trajectory()
    d.startup_event_sink = mock.Mock()
    events = []
    d.probe_sink = events.append
    last_stamp = None
    last_target = None
    for sample in fixture['samples']:
        harness.current[:] = sample['feedback']
        stamp = 10.02 + sample['stamp'] - origin
        last_stamp = stamp
        d._arm.last_feedback_timestamp = stamp
        target = np.zeros(14); target[:7] = sample['target']
        last_target = target.copy()
        previous = d._trajectory.state
        if not d.command_full_urdf(target):
            break
        current = d._trajectory.state
        dt = current.stamp - previous.stamp
        assert np.all(np.abs(current.velocity - previous.velocity)
                      <= d._trajectory.amax * dt + 1e-7)
        assert np.all(np.abs(current.position - previous.position)
                      <= d._trajectory.vmax * dt + 1e-7)
        assert np.all(np.abs(current.position - harness.current) <= d._trajectory.lead + 1e-7)
        if current.diagnostics:
            assert max(current.diagnostics['endpoint_command_speed_m_s']) <= .22 + 1e-7
    # The captured run ends before two full seconds have elapsed from the last
    # distinct braked payload. Extend only its stationary tail to prove the
    # corrected clock still trips a truly unresponsive controller.
    for _ in range(150):
        if d.safety_fault_reason:
            break
        last_stamp += .02
        d._arm.last_feedback_timestamp = last_stamp
        if not d.command_full_urdf(last_target):
            break
    assert 'persistent feedback lead' in d.safety_fault_reason
    assert d.trajectory_diagnostics['fault_context']['fault_code'] == 'persistent_feedback_lead'
    assert d._arm.move_j.call_count >= len(fixture['samples'])
    assert any(e['event'] == 'command_sent' for e in events)
    d._arm.emergency_stop.assert_called_once()
    d._arm.disable.assert_not_called()
    assert not d.command_full_urdf(np.zeros(14))


def test_both_streams_send_each_accepted_feedback_cycle_and_first_hold():
    for side in ('left', 'right'):
        harness = trajectory_tests.DriverTrajectoryTests()
        d = harness.driver(side)
        d._arm.last_feedback_timestamp = 10.02
        assert d.command_full_urdf(np.zeros(14))
        d._arm.move_j.assert_called_once()
        d._arm.last_feedback_timestamp = 10.04
        assert d.command_full_urdf(np.zeros(14))
        assert d._arm.move_j.call_count == 2
        # Explicitly initialized next session must preload again.
        assert d.initialize_command_trajectory()
        before = d._arm.move_j.call_count
        d._arm.last_feedback_timestamp = 10.06
        assert d.command_full_urdf(np.zeros(14))
        assert d._arm.move_j.call_count == before + 1


def test_simulated_continuous_controller_catches_up_with_changing_target():
    harness = trajectory_tests.DriverTrajectoryTests()
    d = harness.driver('left')
    pending = harness.current.copy()
    now = 10.
    events = []
    d.probe_sink = events.append

    def send(q):
        nonlocal pending
        pending = np.array(q)

    d._arm.move_j.side_effect = send
    final_target = np.zeros(14)
    final_target[0] = -.08
    for i in range(1, 601):
        now = 10. + .02 * i
        harness.current[:] += np.clip(pending - harness.current,
                                      -np.deg2rad(.15), np.deg2rad(.15))
        d._arm.last_feedback_timestamp = now
        target = final_target.copy()
        if i < 30:
            target[0] = .12  # Change intent while waiting on an older target.
        old = d._trajectory.state
        assert d.command_full_urdf(target), d.safety_fault_reason
        new = d._trajectory.state
        assert np.all(np.abs(new.velocity - old.velocity)
                      <= d._trajectory.amax * .02 + 1e-7)
        assert np.all(np.abs(new.velocity) <= d._trajectory.vmax + 1e-7)
        assert np.all(np.abs(new.position - harness.current) <= d._trajectory.lead + 1e-7)
    assert abs(harness.current[0] - final_target[0]) < np.deg2rad(.1)
    assert all(e.get('move_j_waypoint_gate') is None for e in events)
    assert d._arm.move_j.call_count == 600
    d._arm.emergency_stop.assert_not_called()


def test_move_j_waypoint_gate_requires_arrival_before_next_send():
    gate = MoveJWaypointGate(minimum_step=np.deg2rad(.5), timeout=1.)
    feedback = np.zeros(7)
    tiny = np.zeros(7); tiny[0] = np.deg2rad(.2)
    waypoint = np.zeros(7); waypoint[0] = np.deg2rad(.6)
    assert not gate.should_send(tiny, feedback)
    assert gate.should_send(waypoint, feedback)
    gate.mark_sent(waypoint, feedback, 10.)
    assert gate.evaluate(feedback, 10.02)
    assert not gate.should_send(waypoint * 2, feedback)
    for i in range(3):
        assert gate.evaluate(waypoint, 10.04 + i * .02) == (i < 2)
    assert gate.should_send(waypoint * 2, waypoint)


def test_failed_send_is_not_deduplicated_as_success_and_does_not_advance_state():
    harness = trajectory_tests.DriverTrajectoryTests()
    d = harness.driver('left')
    d._arm.last_feedback_timestamp = 10.02
    d._arm.move_j.side_effect = RuntimeError('send failed')
    assert not d.command_full_urdf(np.zeros(14))
    assert d._trajectory.state is None
    assert d._last_follow_wire_key is None
    d._arm.move_j.assert_called_once()


@pytest.mark.parametrize('failure', ['stale', 'torque', 'controller'])
def test_continuous_stream_does_not_skip_feedback_or_hardware_safety_checks(failure):
    harness = trajectory_tests.DriverTrajectoryTests()
    d = harness.driver('left')
    d._arm.last_feedback_timestamp = 10.02
    assert d.command_full_urdf(np.zeros(14))
    d._arm.last_feedback_timestamp = 10.04
    assert d.command_full_urdf(np.zeros(14))
    assert d.last_command_skip_reason is None
    if failure == 'stale':
        d._arm.last_feedback_timestamp = 11.
    elif failure == 'torque':
        d._torque_is_safe.return_value = False
    else:
        d._arm.controller_fault.return_value = {'category': 'controller_fault'}
    assert not d.command_full_urdf(np.zeros(14))
    assert d._trajectory.state is None
    assert d._arm.move_j.call_count == 2
