"""Offline wire-level SDK regressions: never connect to a CAN interface."""
import itertools
import json
import math
import struct
import time
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import numpy as np
import pytest

from esrobo_teleop.config import RobotConfig
from esrobo_teleop.robot.nero_driver import NeroArm
from esrobo_teleop.robot.command_trajectory import TrajectoryFault
import test_command_trajectory as trajectory_tests


FIXTURE = Path(__file__).parent / "fixtures/left_deferred_execution_20260920.json"
CHANNELS = itertools.count()


def offline_arm(side, cfg=None):
    pytest.importorskip("pyAgxArm")
    arm = NeroArm(cfg or RobotConfig(), f"offline_{next(CHANNELS)}", side)
    frames = []

    def capture(message):
        packed = arm._robot._parser.pack(message)
        frames.append((packed.arbitration_id, bytes(packed.data).hex()))

    # Intercept before the transport; SDK serialization is real, hardware is not.
    arm._robot._send_msg = mock.Mock(side_effect=capture)
    arm._robot.connect = mock.Mock(side_effect=AssertionError("offline test connected"))
    return arm, frames


def test_matching_collision_ratings_are_verified_without_rewriting_controller():
    arm, _ = offline_arm("left")
    expected = list(arm._cfg.collision_protection_rating)
    arm._robot.get_crash_protection_rating = mock.Mock(
        return_value=SimpleNamespace(msg=expected)
    )
    arm._robot.set_crash_protection_rating = mock.Mock()
    assert arm.configure_collision_protection()
    arm._robot.set_crash_protection_rating.assert_not_called()


def test_mismatched_collision_ratings_are_written_then_verified():
    arm, _ = offline_arm("left")
    expected = list(arm._cfg.collision_protection_rating)
    arm._robot.get_crash_protection_rating = mock.Mock(side_effect=[
        SimpleNamespace(msg=[1] * 7),
        SimpleNamespace(msg=expected),
    ])
    arm._robot.set_crash_protection_rating = mock.Mock(return_value=True)
    assert arm.configure_collision_protection()
    assert arm._robot.set_crash_protection_rating.call_count == 7


def test_left_v112_cpv_tracking_preloads_current_pose_and_streams_one_frame_per_joint():
    cfg = RobotConfig(left_tracking_control_mode="cpv", cpv_inter_joint_interval_s=0.)
    arm, frames = offline_arm("left", cfg)
    current = np.array([-.01, 1.52, .02, .03, 0., 0., -.01])
    vmax = np.array(cfg.left_max_joint_velocity)
    amax = np.array(cfg.left_max_joint_acceleration)

    parameter_modes = []

    def acknowledge_in_cpv(*_args, **_kwargs):
        parameter_modes.append(arm._selected_motion_mode)
        return arm._selected_motion_mode == "cpv"

    arm._robot.set_cpv_cv = mock.Mock(side_effect=acknowledge_in_cpv)
    arm._robot.set_cpv_acc = mock.Mock(side_effect=acknowledge_in_cpv)
    arm._robot.set_cpv_dcc = mock.Mock(side_effect=acknowledge_in_cpv)
    preload = arm._robot.move_cpv_pos
    arm._robot.move_cpv_pos = mock.Mock(side_effect=preload)
    arm.get_joint_angles = mock.Mock(return_value=current.copy())
    arm._robot.get_arm_status = mock.Mock(return_value=SimpleNamespace(
        timestamp=time.time(), msg=SimpleNamespace(
            ctrl_mode=1, arm_status=0, mode_feedback=5)))

    assert arm.prepare_tracking_control(current, vmax, amax, timeout=.1)
    assert arm._selected_motion_mode == "cpv"
    assert arm._cpv_tracking_ready
    assert arm._robot.get_auto_set_motion_mode_enabled()
    assert parameter_modes == ["cpv"] * 21
    # The SDK deliberately duplicates each joint's first CPV position write.
    # All seven current-pose preloads still precede the CPV mode frame, and
    # parameter setters below observe the selected CPV state.
    assert [can_id for can_id, _ in frames[:14]] == [
        can_id for can_id in range(0x181, 0x188) for _ in range(2)
    ]
    assert frames[14][0] == 0x151
    assert bytes.fromhex(frames[14][1])[1] == 5
    assert [can_id for can_id, _ in frames[15:29]] == [
        item
        for joint_id in range(0x181, 0x188)
        for item in (0x151, joint_id)
    ]
    assert all(can_id == 0x151 or 0x181 <= can_id <= 0x187
               for can_id, _ in frames[29:])
    for joint in range(1, 8):
        arm._robot.set_cpv_cv.assert_any_call(joint, float(vmax[joint-1]), timeout=.1)
        arm._robot.set_cpv_acc.assert_any_call(joint, float(amax[joint-1]), timeout=.1)
        arm._robot.set_cpv_dcc.assert_any_call(joint, float(amax[joint-1]), timeout=.1)
        assert arm._robot.move_cpv_pos.call_args_list.count(
            mock.call(joint, float(current[joint-1]))
        ) == 3

    # Preparation wrote the current pose before selecting CPV. Once prepared,
    # live tracking preserves the vendor V112 mode-before-each-joint sequence.
    frames.clear()
    arm._robot.move_cpv_pos.reset_mock(side_effect=True)
    arm._robot.move_cpv_pos.side_effect = preload
    target = current + np.arange(7) * .001
    arm.move_tracking_positions(target.tolist())
    assert [can_id for can_id, _ in frames] == [
        item
        for joint_id in range(0x181, 0x188)
        for item in (0x151, joint_id)
    ]
    assert all(bytes.fromhex(data)[1] == 5 for can_id, data in frames if can_id == 0x151)
    assert not any(can_id in (0x155, 0x156, 0x157, 0x170) for can_id, _ in frames)


def test_cpv_batch_paces_six_inter_joint_boundaries():
    cfg = RobotConfig(left_tracking_control_mode="cpv", cpv_inter_joint_interval_s=.002)
    arm, _ = offline_arm("left", cfg)
    arm._robot.move_cpv_pos = mock.Mock()
    with mock.patch("esrobo_teleop.robot.nero_driver.time.sleep") as sleep:
        arm._send_cpv_positions(np.arange(7, dtype=float) * .01, auto_motion_mode=True)
    assert arm._robot.get_auto_set_motion_mode_enabled()
    assert arm._robot.move_cpv_pos.call_count == 7
    assert sleep.call_args_list == [mock.call(.002)] * 6


def test_cpv_parameter_failure_restores_move_j_before_raising():
    cfg = RobotConfig(left_tracking_control_mode="cpv", cpv_inter_joint_interval_s=0.)
    arm, frames = offline_arm("left", cfg)
    arm._robot.set_cpv_cv = mock.Mock(return_value=False)
    arm._robot.set_cpv_acc = mock.Mock(return_value=True)
    arm._robot.set_cpv_dcc = mock.Mock(return_value=True)
    arm._robot.move_cpv_pos = mock.Mock()
    arm._robot.get_arm_status = mock.Mock(return_value=SimpleNamespace(
        timestamp=time.time(), msg=SimpleNamespace(
            ctrl_mode=1, arm_status=0, mode_feedback=5)))
    arm.get_joint_angles = mock.Mock(return_value=np.zeros(7))
    with pytest.raises(RuntimeError, match="velocity acknowledgement missing for J1"):
        arm.prepare_tracking_control(
            np.zeros(7), np.ones(7) * .2, np.ones(7) * .4, timeout=.1
        )
    assert not arm._cpv_tracking_ready
    assert arm._selected_motion_mode == "j"
    assert arm._robot.get_auto_set_motion_mode_enabled()
    assert arm._robot.move_cpv_pos.call_count == 14
    assert [can_id for can_id, _ in frames] == [0x151, 0x151]
    assert bytes.fromhex(frames[0][1])[1] == 5
    assert bytes.fromhex(frames[1][1])[1] == 1


def test_cpv_mode_transition_pose_drift_rejects_after_acknowledged_parameters():
    cfg = RobotConfig(left_tracking_control_mode="cpv", cpv_inter_joint_interval_s=0.)
    arm, frames = offline_arm("left", cfg)
    current = np.zeros(7)
    arm._robot.set_cpv_cv = mock.Mock(return_value=True)
    arm._robot.set_cpv_acc = mock.Mock(return_value=True)
    arm._robot.set_cpv_dcc = mock.Mock(return_value=True)
    arm._robot.move_cpv_pos = mock.Mock()
    arm._robot.get_arm_status = mock.Mock(return_value=SimpleNamespace(
        timestamp=time.time(), msg=SimpleNamespace(
            ctrl_mode=1, arm_status=0, mode_feedback=5)))
    drifted = current.copy()
    drifted[0] = math.radians(1.1)
    arm.get_joint_angles = mock.Mock(return_value=drifted)

    with pytest.raises(RuntimeError, match=r"moved during CPV mode transition on joints \[1\]"):
        arm.prepare_tracking_control(
            current, np.ones(7) * .2, np.ones(7) * .4, timeout=.1
        )

    assert arm._robot.set_cpv_cv.call_count == 7
    assert arm._robot.set_cpv_acc.call_count == 7
    assert arm._robot.set_cpv_dcc.call_count == 7
    assert not arm._cpv_tracking_ready
    assert arm._selected_motion_mode == "j"
    assert arm._robot.get_auto_set_motion_mode_enabled()
    assert [can_id for can_id, _ in frames] == [0x151, 0x151]


def recorded_targets(record):
    frames = record["native_sends"]
    for i in range(0, len(frames), 5):
        batch = frames[i:i+5]
        assert [f["can_id"] for f in batch] == [0x151, 0x155, 0x156, 0x157, 0x170]
        values = [v for f in batch[1:] for v in struct.unpack(">ii", bytes.fromhex(f["data_hex"]))][:7]
        yield [math.radians(v / 1000) for v in values], [(f["can_id"], f["data_hex"]) for f in batch[1:]]


@pytest.mark.parametrize("side", ["left", "right"])
def test_sdk_stream_keeps_exact_recorded_targets_without_reselecting_mode(side):
    record = json.loads(FIXTURE.read_text())
    arm, frames = offline_arm(side)
    assert arm._robot.get_auto_set_motion_mode_enabled() == (side == "left")
    arm.set_motion_mode("j")
    arm.set_speed_percent(30)
    frames.clear()
    expected = []
    for index, (target, payloads) in enumerate(recorded_targets(record)):
        arm.move_j(target)
        if side == "left" or index == 0:
            expected.append((0x151, "01011e0000000000"))
        expected.extend(payloads)
    assert frames == expected
    expected_mode_frames = 225 if side == "left" else 1
    assert len(frames) == 225 * 4 + expected_mode_frames
    assert sum(can_id == 0x151 for can_id, _ in frames) == expected_mode_frames
    # Reproduce the previous SDK default on the SAME serialized targets.
    arm._robot.set_auto_set_motion_mode_enabled(True)
    frames.clear()
    for target, _ in recorded_targets(record):
        arm._robot.move_j(target)
    assert sum(can_id == 0x151 for can_id, _ in frames) == 225
    assert {data for can_id, data in frames if can_id == 0x151} == {"01011e0000000000"}
    arm._robot.connect.assert_not_called()


@pytest.mark.parametrize("side", ["left", "right"])
def test_speed_only_frame_forces_one_complete_mode_before_next_position(side):
    arm, frames = offline_arm(side)
    arm.set_motion_mode("j")
    arm.move_j([0.] * 7)
    frames.clear()

    arm.set_speed_percent(30)
    assert frames == [(0x151, "01ff1e0000000000")]
    assert arm._selected_motion_mode is None

    frames.clear()
    arm.move_j([0.] * 7)
    assert [can_id for can_id, _ in frames] == [0x151, 0x155, 0x156, 0x157, 0x170]
    assert frames[0][1] == "01011e0000000000"
    assert arm._selected_motion_mode == "j"

    frames.clear()
    arm.move_j([0.] * 7)
    expected = ([0x151] if side == "left" else []) + [0x155, 0x156, 0x157, 0x170]
    assert [can_id for can_id, _ in frames] == expected


def test_mode_failure_does_not_cache_success_and_reset_requires_reselection():
    arm, frames = offline_arm("left")
    capture = arm._robot._send_msg.side_effect
    arm._robot._send_msg.side_effect = RuntimeError("mode send failed")
    with pytest.raises(RuntimeError, match="mode send failed"):
        arm.move_j([0.] * 7)
    assert arm._selected_motion_mode is None
    assert not frames
    arm._robot._send_msg.side_effect = capture
    arm.move_j([0.] * 7)
    assert [f[0] for f in frames] == [0x151, 0x155, 0x156, 0x157, 0x170]
    arm.reset()
    frames.clear()
    arm.move_j([0.] * 7)
    assert frames[0][0] == 0x151
    # Existing explicit JS callers still select JS; move_j must restore J.
    arm.move_js([0.] * 7)
    frames.clear()
    arm.move_j([0.] * 7)
    assert frames[0][0] == 0x151
    assert bytes.fromhex(frames[0][1])[3] == 0


@pytest.mark.parametrize("side", ["left", "right"])
def test_disabled_mode_cache_cannot_suppress_post_enable_configuration(side):
    arm, frames = offline_arm(side)
    # Model the vendor-documented rule: configuration/targets sent while
    # disabled are ignored. The previous controller speed is unknown; use 0
    # to demonstrate why merely seeing CAN/J status proves no applied speed.
    device = dict(enabled=False, mode=1, speed=0, executed=[])
    capture = arm._robot._send_msg.side_effect

    def receive(message):
        capture(message)
        cid, raw = frames[-1]
        payload = bytes.fromhex(raw)
        if cid == 0x471:
            device['enabled'] = payload[1] == 2
        elif cid == 0x151 and device['enabled']:
            device['mode'], device['speed'] = payload[1], payload[2]
        elif cid == 0x170 and device['enabled'] and device['mode'] == 1 and device['speed'] > 0:
            device['executed'].append(len(frames))

    arm._robot._send_msg.side_effect = receive
    arm._robot.get_joints_enable_status_list = mock.Mock(
        side_effect=lambda: [device['enabled']] * 7)
    arm.get_joint_enable_states = mock.Mock(side_effect=lambda: [device['enabled']] * 7)
    hold = [0., math.pi/2, 0., 0., 0., 0., 0.]
    for _ in range(2):  # Covers disable/re-enable in the same process too.
        arm.set_speed_percent(30)
        arm.set_motion_mode('j')
        arm.move_j(hold)
        before = len(frames)
        assert arm.enable(timeout=.1)
        assert not any(cid in (0x155, 0x156, 0x157, 0x170) for cid, _ in frames[before:])
        enabled_at = len(frames)
        arm.move_j(hold)
        assert [cid for cid, _ in frames[enabled_at:]] == [0x151, 0x155, 0x156, 0x157, 0x170]
        assert bytes.fromhex(frames[enabled_at][1])[:4] == bytes([1, 1, 30, 0])
        assert device['executed'][-1] == len(frames)
        # V112 left keeps the vendor mode-before-target sequence; the legacy
        # right path keeps the commissioned boundary-only mode selection.
        frames.clear()
        arm.move_j(hold)
        expected = ([0x151] if side == "left" else []) + [0x155, 0x156, 0x157, 0x170]
        assert [cid for cid, _ in frames] == expected
        assert arm.disable(timeout=.1)
        assert arm._selected_motion_mode is None
        device['speed'] = 0
    arm._robot.connect.assert_not_called()


def test_post_enable_mode_send_failure_prevents_hold_position():
    arm, frames = offline_arm('left')
    arm.set_motion_mode('j')
    arm._robot.enable = mock.Mock(return_value=True)
    assert arm.enable(timeout=.1)
    frames.clear()
    arm._robot._send_msg.side_effect = RuntimeError('mode failed after enable')
    with pytest.raises(RuntimeError, match='mode failed after enable'):
        arm.move_j([0.] * 7)
    assert not frames
    assert arm._selected_motion_mode is None


def test_recorded_feedback_moves_only_after_stream_stops():
    record = json.loads(FIXTURE.read_text())
    t = record["fault_timestamp"]
    before = [r["physical_rad"][:4] for r in record["states"] if r["t"] <= t]
    np.testing.assert_allclose(np.ptp(before, axis=0), 0, atol=1e-9)
    assert record["post_fault_send_count_5s"] == 0
    last = record["fault_context"]["previous_command_physical_rad"]
    after = record["states"][-1]["physical_rad"]
    assert max(abs(a-b) for a, b in zip(last, after)) < math.radians(.02)


def test_external_mode_change_is_rejected_instead_of_reasserted_in_follow_loop():
    arm, frames = offline_arm("left")
    arm.set_motion_mode("j")
    frames.clear()
    arm._robot.get_arm_status = mock.Mock(return_value=SimpleNamespace(
        timestamp=time.time(), msg=SimpleNamespace(
            arm_status=0, err_code=0, ctrl_mode=1, mode_feedback=0)))
    assert arm.controller_fault()["category"] == "motion_mode_mismatch"
    assert not frames


@pytest.mark.parametrize("send_error", [None, RuntimeError("CAN unavailable")])
def test_stalled_feedback_requests_stop_once_without_new_target_or_false_disable(send_error):
    record = json.loads(FIXTURE.read_text())
    driver = trajectory_tests.DriverTrajectoryTests().driver("left")
    driver.startup_event_sink = mock.Mock()
    driver._arm.last_feedback_timestamp = 10.02
    driver._arm.emergency_stop.side_effect = send_error
    with mock.patch.object(driver._trajectory, "propose", side_effect=TrajectoryFault(
            record["fault"], record["fault_context"])):
        assert not driver.command_full_urdf(np.zeros(14))
    driver._arm.emergency_stop.assert_called_once_with()
    driver._arm.move_j.assert_not_called()
    driver._arm.disable.assert_not_called()
    driver._arm.reset.assert_not_called()
    assert driver._enabled  # Remains unknown until actual disable feedback.
    assert driver._return_inhibited
    assert driver._trajectory.state is None
    stop = driver.trajectory_diagnostics["fault_context"]["controller_stop"]
    assert stop["send_returned"] == (send_error is None)
    assert not stop["disable_verified"]
    assert ("error" in stop) == (send_error is not None)
    event = driver.startup_event_sink.call_args.args[0]
    assert event["fault_context"]["controller_stop"] == stop
    assert not driver.command_full_urdf(np.zeros(14))
    driver._arm.emergency_stop.assert_called_once_with()
