import contextlib
import io
import math
import os
import time
import unittest
from unittest import mock

import numpy as np

from esrobo_teleop import math_utils as mu
from esrobo_teleop.config import build_config
from esrobo_teleop.device.body_device import BodyDevice
from esrobo_teleop.ik.solver import IkSolver
from esrobo_teleop.robot.linker_hand_driver import LinkerHandDriver
from esrobo_teleop.robot.nero_driver import (
    NeroArm,
    NeroDualArmDriver,
    NeroSingleArmDriver,
    NeroRightWristDriver,
    NeroWristDriver,
    imu_local_rotation_to_wrist_offsets,
    imu_rotation_to_wrist_offsets,
    right_imu_rotation_to_wrist_offsets,
)
from esrobo_teleop.teleop_node import TeleopNode


class NeroMappingTests(unittest.TestCase):
    def setUp(self):
        self.driver = NeroDualArmDriver.__new__(NeroDualArmDriver)
        self.driver._cfg = build_config().robot
        self.driver._session_start = None
        self.driver._command_velocity = {
            "left": np.zeros(7),
            "right": np.zeros(7),
        }

    def test_joint2_mapping_matches_sdk_and_urdf_limits(self):
        full_lower = np.asarray([-3.3107963])
        full_upper = np.asarray([0.1692037])
        self.assertAlmostEqual(full_lower[0] + math.pi / 2, -1.74, places=2)
        self.assertAlmostEqual(full_upper[0] + math.pi / 2, 1.74, places=2)

    def test_mapping_round_trip(self):
        full = np.linspace(-0.3, 0.3, 14)
        left, right = self.driver._full_to_physical(full)
        np.testing.assert_allclose(self.driver._physical_to_full_urdf(left, right), full)

    def test_per_joint_acceleration_limits_first_command(self):
        _, current = self.driver._mapping("left")
        target = current + 0.1
        output = self.driver._clamp(target, current, "left", 0.02)
        acceleration = self.driver._joint_rate_limit_vector("left", "max_joint_acceleration")
        np.testing.assert_allclose(output - current, acceleration * 0.02**2)

    def test_joint_synchronization_preserves_target_error_ratios(self):
        self.driver._cfg.synchronize_joint_motion = True
        current = np.zeros(7)
        current[1] = math.pi / 2
        error = np.asarray([0.4, -0.2, 0.3, 0.5, -0.1, 0.25, -0.35])
        output = self.driver._clamp(current + error, current, "right", 0.02)
        progress = (output - current) / error

        np.testing.assert_allclose(progress, np.full(7, progress[0]), atol=1.0e-12)
        self.assertGreater(progress[0], 0.0)
        self.assertLess(progress[0], 1.0)

    def test_direction_reversal_decelerates_before_reversing(self):
        self.driver._command_velocity["left"] = np.full(7, 0.1)
        current = np.zeros(7)
        current[1] = math.pi / 2
        target = current - 0.5
        output = self.driver._clamp(target, current, "left", 0.02)
        expected_velocity = 0.1 - self.driver._joint_rate_limit_vector(
            "left", "max_joint_acceleration"
        ) * 0.02
        expected_delta = np.minimum(
            expected_velocity * 0.02,
            self.driver._joint_rate_limit_vector("left", "max_joint_step"),
        )
        np.testing.assert_allclose(output - current, expected_delta)
        self.assertTrue(np.all(output - current > 0.0))

    def test_full_arm_commissioning_soft_limits_are_applied_per_joint(self):
        self.driver._cfg.max_joint_step = 0.0
        self.driver._cfg.max_joint_velocity = 0.0
        self.driver._cfg.max_joint_acceleration = 0.0
        self.driver._cfg.left_max_joint_step = None
        self.driver._cfg.left_max_joint_velocity = None
        self.driver._cfg.left_max_joint_acceleration = None
        target_urdf = np.asarray([2.0, -2.0, 2.0, 2.0, 2.0, -2.0, 2.0])
        target_physical, _ = self.driver._full_to_physical(
            np.concatenate([target_urdf, target_urdf])
        )
        output = self.driver._clamp(target_physical, target_physical, "left", 0.02)
        expected_urdf = np.asarray(
            [1.047198, -1.308997, 1.047198, 1.483530, 1.047198, -0.523599, 0.959931]
        )
        expected_physical, _ = self.driver._full_to_physical(
            np.concatenate([expected_urdf, expected_urdf])
        )
        np.testing.assert_allclose(output, expected_physical)

    def test_full_arm_commissioning_rate_limits_are_per_joint(self):
        cfg = build_config().robot

        self.assertFalse(cfg.synchronize_joint_motion)
        np.testing.assert_allclose(
            np.degrees(cfg.max_joint_velocity), [18.0, 18.0, 18.0, 20.0, 20.0, 20.0, 20.0],
            atol=1.0e-4,
        )
        np.testing.assert_allclose(
            np.degrees(cfg.max_joint_acceleration),
            [45.0, 45.0, 50.0, 60.0, 60.0, 60.0, 60.0],
            atol=1.0e-4,
        )
        np.testing.assert_allclose(
            np.degrees(cfg.max_joint_step), [2.2, 2.2, 2.2, 2.5, 2.5, 2.5, 2.5],
            atol=1.0e-4,
        )
        np.testing.assert_allclose(
            np.degrees(cfg.left_max_joint_velocity),
            [18.0, 18.0, 18.0, 20.0, 20.0, 20.0, 20.0],
            atol=1.0e-4,
        )
        np.testing.assert_allclose(
            np.degrees(cfg.left_max_joint_acceleration),
            [45.0, 45.0, 50.0, 60.0, 60.0, 60.0, 60.0],
            atol=1.0e-4,
        )
        np.testing.assert_allclose(
            np.degrees(cfg.left_max_joint_step),
            [2.2, 2.2, 2.2, 2.5, 2.5, 2.5, 2.5],
            atol=1.0e-4,
        )
        np.testing.assert_allclose(
            self.driver._joint_rate_limit_vector("right", "max_joint_velocity"),
            cfg.right_max_joint_velocity,
        )
        np.testing.assert_allclose(
            cfg.right_max_joint_velocity,
            cfg.left_max_joint_velocity,
        )

    def test_joint2_soft_limits_include_sdk_physical_offset(self):
        lower, upper = self.driver._soft_limits_physical("right")
        self.assertAlmostEqual(lower[1], math.radians(15.0), places=5)
        self.assertAlmostEqual(upper[1], math.radians(97.0), places=5)

    def test_right_arm_has_independent_legacy_firmware_parser(self):
        cfg = build_config().robot
        self.assertEqual(cfg.nero_firmware, "V112")
        self.assertEqual(cfg.right_nero_firmware, "DEFAULT")

    def test_right_joint3_hardware_direction_matches_urdf(self):
        cfg = build_config().robot
        self.assertEqual(tuple(cfg.right_joint_directions), (1, 1, 1, 1, 1, 1, 1))
        urdf = np.asarray([0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.0])
        directions, offsets = self.driver._mapping("right")
        physical = directions * urdf + offsets
        self.assertAlmostEqual(physical[2], 0.2)

    def test_feedback_dt_is_computed_independently_for_each_arm(self):
        self.driver._left = mock.Mock(last_feedback_timestamp=10.04)
        self.driver._right = mock.Mock(last_feedback_timestamp=20.06)
        self.driver._last_command_feedback_timestamp = {
            "left": 10.00,
            "right": 20.00,
        }
        dts = self.driver._feedback_command_dts()
        self.assertIsNotNone(dts)
        self.assertAlmostEqual(dts["left"], 0.04)
        self.assertAlmostEqual(dts["right"], 0.06)

    def test_feedback_dt_rejects_reused_timestamp(self):
        self.driver._left = mock.Mock(last_feedback_timestamp=10.00)
        self.driver._right = mock.Mock(last_feedback_timestamp=20.02)
        self.driver._last_command_feedback_timestamp = {
            "left": 10.00,
            "right": 20.00,
        }
        self.assertIsNone(self.driver._feedback_command_dts())


class EndpointTranslationLimitTests(unittest.TestCase):
    def test_wrist_target_is_limited_to_eight_centimeters_per_second(self):
        previous = np.asarray([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
        target = np.asarray([1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0])
        limited = BodyDevice._limit_pose_translation(previous, target, 0.08, 0.25)
        np.testing.assert_allclose(limited[:3], [0.02, 0.0, 0.0])
        np.testing.assert_allclose(limited[3:], target[3:])

    def test_wrist_target_inside_velocity_bound_is_unchanged(self):
        previous = np.asarray([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
        target = np.asarray([0.001, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
        limited = BodyDevice._limit_pose_translation(previous, target, 0.08, 0.02)
        np.testing.assert_allclose(limited, target)

    def test_arm_segment_limiter_moves_elbow_and_wrist_together(self):
        shoulder = np.asarray([0.0, 0.0, 0.0])
        previous_elbow = np.asarray([0.0, 0.0, -0.4, 1.0, 0.0, 0.0, 0.0])
        previous_wrist = np.asarray([0.0, 0.0, -0.7, 1.0, 0.0, 0.0, 0.0])
        target_elbow = np.asarray([0.4, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
        target_wrist = np.asarray([0.7, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])

        elbow, wrist = BodyDevice._limit_arm_segment_translations(
            previous_elbow,
            previous_wrist,
            target_elbow,
            target_wrist,
            shoulder,
            0.08,
            0.25,
        )

        self.assertGreater(np.linalg.norm(elbow[:3] - previous_elbow[:3]), 0.0)
        self.assertGreater(np.linalg.norm(wrist[:3] - previous_wrist[:3]), 0.0)
        self.assertLessEqual(np.linalg.norm(elbow[:3] - previous_elbow[:3]), 0.020001)
        self.assertLessEqual(np.linalg.norm(wrist[:3] - previous_wrist[:3]), 0.020001)
        self.assertAlmostEqual(np.linalg.norm(elbow[:3] - shoulder), 0.4, places=6)
        self.assertAlmostEqual(np.linalg.norm(wrist[:3] - elbow[:3]), 0.3, places=6)

    def test_arm_segment_limiter_holds_both_points_when_dt_is_invalid(self):
        previous_elbow = np.asarray([0.0, 0.0, -0.4, 1.0, 0.0, 0.0, 0.0])
        previous_wrist = np.asarray([0.0, 0.0, -0.7, 1.0, 0.0, 0.0, 0.0])
        target_elbow = np.asarray([0.4, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
        target_wrist = np.asarray([0.7, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])

        elbow, wrist = BodyDevice._limit_arm_segment_translations(
            previous_elbow,
            previous_wrist,
            target_elbow,
            target_wrist,
            np.zeros(3),
            0.08,
            0.0,
        )

        np.testing.assert_allclose(elbow[:3], previous_elbow[:3])
        np.testing.assert_allclose(wrist[:3], previous_wrist[:3])


class ArmVectorCoordinateTests(unittest.TestCase):
    def test_calibrated_senseglove_delta_is_not_rezeroed_by_pico_rebase(self):
        device = BodyDevice.__new__(BodyDevice)
        device._cfg = build_config().retarget
        device._last_hand_orientation_packet_time_monotonic = time.monotonic()
        calibrated_delta = mu.rotvec_to_rotation_matrix([0.12, -0.08, 0.04])
        device._hand_orientation_delta_matrices = {"left": calibrated_delta}
        device._hand_imu_uses_calibrated_zero = {"left": True, "right": False}
        device._hand_imu_reference_delta_matrices = {}

        result = device._hand_imu_relative_rotation_local("left")

        np.testing.assert_allclose(result, calibrated_delta, atol=1.0e-7)
        self.assertEqual(device._hand_imu_reference_delta_matrices, {})

    def test_matching_down_axis_does_not_rotate_h1_h2(self):
        alignment = mu.minimal_vector_alignment_rotation(
            np.asarray([0.0, 0.0, -1.0]),
            np.asarray([0.0, 0.0, -1.0]),
        )

        np.testing.assert_allclose(alignment, np.eye(3), atol=1.0e-7)
        np.testing.assert_allclose(alignment @ [1.0, 0.0, 0.0], [1.0, 0.0, 0.0])
        np.testing.assert_allclose(alignment @ [0.0, 1.0, 0.0], [0.0, 1.0, 0.0])

    def test_tilt_correction_does_not_add_quarter_turn_yaw(self):
        source_down = np.asarray([-0.25, 0.0, -1.0])
        target_down = np.asarray([0.0, 0.0, -1.0])
        alignment = mu.minimal_vector_alignment_rotation(source_down, target_down)

        mapped_down = alignment @ (source_down / np.linalg.norm(source_down))
        np.testing.assert_allclose(mapped_down, target_down, atol=1.0e-6)
        np.testing.assert_allclose(
            alignment @ [0.0, 1.0, 0.0], [0.0, 1.0, 0.0], atol=1.0e-6
        )

    def test_single_arm_readiness_allows_pico_only_when_glove_is_optional(self):
        device = BodyDevice.__new__(BodyDevice)
        device._cfg = build_config().retarget
        device._active_arm_side = "left"
        device._reference_locked = True
        device._has_fresh_packet = mock.Mock(return_value=True)
        device._hand_imu_delta_matrix_for_target = mock.Mock(return_value=None)
        self.assertTrue(device.is_ready())

        device._cfg.require_hand_imu_for_active_arm = True
        self.assertFalse(device.is_ready())

    def test_pico_only_mode_does_not_use_tracker_wrist_orientation(self):
        device = BodyDevice.__new__(BodyDevice)
        device._cfg = build_config().retarget
        device._active_arm_side = "left"
        device._initial_left_pose_matrix = np.eye(4, dtype=np.float32)
        device._initial_right_pose_matrix = np.eye(4, dtype=np.float32)
        device._hand_imu_relative_rotation_local = mock.Mock(return_value=None)
        device._current_wrist_matrix = mock.Mock()

        rotation = device._compose_wrist_rotation("left", None)

        np.testing.assert_allclose(rotation, np.eye(3))
        device._current_wrist_matrix.assert_not_called()


class BodyReferencePromptTests(unittest.TestCase):
    @staticmethod
    def _device() -> BodyDevice:
        device = BodyDevice.__new__(BodyDevice)
        device._cfg = build_config().retarget
        device._cfg.auto_start_reference_prepare_s = 2.0
        device._cfg.auto_start_reference_sample_start_s = 1.0
        device._cfg.auto_start_reference_delay_s = 2.0
        device._cfg.auto_start_reference_min_samples = 2
        device._active_arm_side = "left"
        device._reference_samples_left = []
        device._reference_samples_right = []
        device._reference_left_points = None
        device._reference_right_points = None
        device._reference_wrist_rotations = {}
        device._reference_locked = False
        device._reference_start_time = None
        device._reference_prompt_key = None
        points = {
            "shoulder": np.asarray([0.0, 0.0, 0.0]),
            "elbow": np.asarray([0.0, 0.0, -0.3]),
            "wrist": np.asarray([0.0, 0.0, -0.6]),
        }
        device._current_arm_points = mock.Mock(return_value=points)
        device._current_wrist_matrix = mock.Mock(return_value=(np.eye(4), "left_wrist"))
        return device

    def test_reference_waits_for_prepare_and_settle_before_sampling(self):
        device = self._device()
        output = io.StringIO()

        with contextlib.redirect_stdout(output):
            device._update_auto_reference(10.0)
            device._update_auto_reference(11.9)
            self.assertEqual(device._reference_samples_left, [])
            device._update_auto_reference(12.5)
            self.assertEqual(device._reference_samples_left, [])
            device._update_auto_reference(13.0)
            device._update_auto_reference(13.5)
            device._update_auto_reference(14.0)

        self.assertTrue(device._reference_locked)
        text = output.getvalue()
        self.assertIn("PICO 手臂参考姿态标定", text)
        self.assertIn("秒后进入稳定缓冲", text)
        self.assertIn("正在采集参考姿态", text)
        self.assertIn("自然下垂参考零点已锁定", text)

    def test_unstable_reference_restarts_full_countdown(self):
        device = self._device()
        device._cfg.auto_start_reference_max_position_std_m = 0.01
        device._reference_start_time = 10.0
        device._reference_samples_left = [
            {
                "shoulder": np.asarray([0.0, 0.0, 0.0]),
                "elbow": np.asarray([0.0, 0.0, -0.3]),
                "wrist": np.asarray([0.0, 0.0, -0.6]),
            },
            {
                "shoulder": np.asarray([0.1, 0.0, 0.0]),
                "elbow": np.asarray([0.1, 0.0, -0.3]),
                "wrist": np.asarray([0.1, 0.0, -0.6]),
            },
        ]
        output = io.StringIO()

        with contextlib.redirect_stdout(output):
            device._update_auto_reference(14.0)

        self.assertFalse(device._reference_locked)
        self.assertEqual(device._reference_start_time, 14.0)
        self.assertEqual(device._reference_samples_left, [])
        self.assertIn("晃动过大", output.getvalue())


class NeroSingleArmTests(unittest.TestCase):
    def setUp(self):
        self.driver = NeroSingleArmDriver.__new__(NeroSingleArmDriver)
        self.driver._cfg = build_config().robot
        self.driver._side = "left"
        self.driver._session_start = None
        self.driver._last_command_feedback_timestamp = 10.0
        self.driver._command_velocity = {"left": np.zeros(7)}
        self.driver._arm = mock.Mock(last_feedback_timestamp=10.02)

    def test_urdf_zero_matches_natural_down_sdk_feedback(self):
        _, offsets = self.driver._mapping("left")
        self.driver.read_joints = mock.Mock(return_value=offsets.copy())
        np.testing.assert_allclose(self.driver.startup_zero_error(), np.zeros(7))
        full = self.driver.read_full_urdf_joints()
        np.testing.assert_allclose(full, np.zeros(14))

    def test_startup_alignment_enables_at_feedback_then_returns_and_disables(self):
        self.driver.capture_session_start = mock.Mock(return_value=True)
        self.driver.enable = mock.Mock(return_value=True)
        self.driver.return_to_zero_and_disable = mock.Mock(return_value=(True, True))

        self.assertEqual(self.driver.align_startup_zero_and_disable(), (True, True))

        self.driver.capture_session_start.assert_called_once_with()
        self.driver.enable.assert_called_once_with()
        self.driver.return_to_zero_and_disable.assert_called_once_with()

    def test_return_to_zero_retracts_j4_to_j7_before_j1_to_j3(self):
        directions, zero = self.driver._mapping("left")
        self.driver._cfg.return_verify_samples = 1
        initial_urdf = np.asarray([0.1, -0.2, 0.3, 0.4, -0.3, 0.2, -0.1])
        distal_zero = initial_urdf.copy()
        distal_zero[3:7] = 0.0
        self.driver._enabled = True
        self.driver.read_joints = mock.Mock(
            side_effect=[
                directions * initial_urdf + zero,
                directions * distal_zero + zero,
                zero.copy(),
            ]
        )
        self.driver._arm.disable.return_value = True

        self.assertEqual(self.driver.return_to_zero_and_disable(), (True, True))

        physical_commands = [
            np.asarray(call.args[0]) for call in self.driver._arm.move_j.call_args_list
        ]
        np.testing.assert_allclose(physical_commands[0], directions * distal_zero + zero)
        np.testing.assert_allclose(physical_commands[-1], zero)
        self.driver._arm.disable.assert_called_once_with()

    def test_failed_distal_return_holds_enabled_without_automatic_disable(self):
        _, zero = self.driver._mapping("left")
        self.driver._cfg.return_feedback_grace_s = 0.0
        self.driver._cfg.return_phase_attempts = 1
        self.driver._enabled = True
        self.driver.read_joints = mock.Mock(side_effect=[zero.copy(), None])
        self.driver._arm.disable.return_value = True

        self.assertEqual(self.driver.return_to_zero_and_disable(), (False, False))

        self.driver._arm.move_j.assert_not_called()
        self.driver._arm.disable.assert_not_called()

    def test_return_group_tolerates_one_missing_feedback_cycle(self):
        _, zero = self.driver._mapping("left")
        self.driver._cfg.return_feedback_grace_s = 0.2
        self.driver._cfg.return_verify_samples = 2
        samples = iter([(None, None), (zero.copy(), 10.04), (zero.copy(), 10.06)])

        def read_sample():
            joints, stamp = next(samples)
            self.driver._arm.last_feedback_timestamp = stamp
            return joints

        self.driver.read_joints = mock.Mock(side_effect=read_sample)

        returned = self.driver._return_group_to_zero(
            np.zeros(14), np.arange(7), zero, math.radians(1.0), 1.0
        )

        self.assertTrue(returned)
        self.assertEqual(self.driver.read_joints.call_count, 3)

    def test_proximal_stage_rechecks_all_seven_joints(self):
        directions, zero = self.driver._mapping("left")
        self.driver._enabled = True
        self.driver.read_joints = mock.Mock(return_value=directions * 0.1 + zero)
        self.driver._return_group_to_zero = mock.Mock(side_effect=[True, True])
        self.driver._arm.disable.return_value = True

        self.assertEqual(self.driver.return_to_zero_and_disable(), (True, True))

        second_indices = self.driver._return_group_to_zero.call_args_list[1].args[1]
        np.testing.assert_array_equal(second_indices, np.arange(7))

    def test_return_retries_distal_phase_before_disabling(self):
        _, zero = self.driver._mapping("right")
        self.driver._side = "right"
        self.driver._command_velocity = {"right": np.zeros(7)}
        self.driver._enabled = True
        self.driver._cfg.return_phase_attempts = 2
        self.driver.read_joints = mock.Mock(return_value=zero.copy())
        self.driver._return_group_to_zero = mock.Mock(
            side_effect=[False, True, True]
        )
        self.driver._arm.disable.return_value = True

        self.assertEqual(self.driver.return_to_zero_and_disable(), (True, True))

        self.assertEqual(self.driver._return_group_to_zero.call_count, 3)
        first_indices = self.driver._return_group_to_zero.call_args_list[0].args[1]
        retry_indices = self.driver._return_group_to_zero.call_args_list[1].args[1]
        final_indices = self.driver._return_group_to_zero.call_args_list[2].args[1]
        np.testing.assert_array_equal(first_indices, np.arange(3, 7))
        np.testing.assert_array_equal(retry_indices, np.arange(3, 7))
        np.testing.assert_array_equal(final_indices, np.arange(7))
        self.driver._arm.request_can_feedback_push.assert_called_once_with()

    def test_return_phase_uses_its_own_feedback_clock(self):
        directions, zero = self.driver._mapping("left")
        self.driver._cfg.return_verify_samples = 1
        self.driver._last_command_feedback_timestamp = 999.0
        samples = iter(
            [
                (directions * 0.05 + zero, 10.00),
                (directions * 0.04 + zero, 10.02),
                (zero.copy(), 10.04),
            ]
        )

        def read_sample():
            joints, stamp = next(samples)
            self.driver._arm.last_feedback_timestamp = stamp
            return joints

        self.driver.read_joints = mock.Mock(side_effect=read_sample)

        returned = self.driver._return_group_to_zero(
            np.zeros(14), np.arange(3), zero, math.radians(1.0), 1.0
        )

        self.assertTrue(returned)
        commands = [
            np.asarray(call.args[0]) for call in self.driver._arm.move_j.call_args_list
        ]
        np.testing.assert_allclose(commands[0], directions * 0.05 + zero)
        self.assertTrue(np.all(np.abs(commands[1] - zero) < 0.04))

    def test_silent_left_startup_defers_commands_until_operator_arms(self):
        self.driver._arm.get_joint_enable_states.return_value = None
        self.driver._cfg.left_arm_allow_silent_disabled_startup = True

        self.driver.connect()

        self.assertTrue(self.driver.silent_disabled_startup)
        self.driver._arm.connect.assert_called_once_with()
        self.driver._arm.request_can_feedback_push.assert_called_once_with()
        self.driver._arm.disable.assert_not_called()
        self.driver._arm.enable_at_known_target.assert_not_called()

    def test_right_startup_requests_feedback_before_verified_disable(self):
        self.driver._side = "right"
        self.driver._command_velocity = {"right": np.zeros(7)}
        self.driver._arm.get_joint_enable_states.return_value = [True] * 7
        self.driver._arm.disable.return_value = True

        self.driver.connect()

        self.driver._arm.request_can_feedback_push.assert_called_once_with()
        self.driver._arm.disable.assert_called_once_with()
        self.assertFalse(self.driver.silent_disabled_startup)

    def test_silent_left_enable_uses_physical_natural_down_zero(self):
        _, zero = self.driver._mapping("left")
        self.driver._silent_disabled_startup = True
        self.driver._arm.enable_at_known_target.return_value = (zero.copy(), [True] * 7)

        self.assertTrue(self.driver.enable_from_natural_down())

        target = self.driver._arm.enable_at_known_target.call_args.args[0]
        np.testing.assert_allclose(target, zero)
        self.driver._arm.best_effort_disable_once.assert_not_called()
        self.driver._arm.set_speed_percent.assert_called_once_with(
            self.driver._cfg.speed_percent
        )
        self.assertFalse(self.driver.silent_disabled_startup)

    def test_silent_left_enable_rejects_missing_feedback_and_disables_once(self):
        self.driver._silent_disabled_startup = True
        self.driver._arm.enable_at_known_target.return_value = (None, None)

        self.assertFalse(self.driver.enable_from_natural_down())

        self.driver._arm.best_effort_disable_once.assert_called_once_with()
        self.assertTrue(self.driver.silent_disabled_startup)

    def test_command_sends_only_the_configured_arm(self):
        _, offsets = self.driver._mapping("left")
        self.driver.read_joints = mock.Mock(return_value=offsets.copy())
        target = np.zeros(14)
        target[:7] = 0.2

        self.assertTrue(self.driver.command_full_urdf(target))

        self.driver._arm.move_j.assert_called_once()
        command = np.asarray(self.driver._arm.move_j.call_args.args[0])
        self.assertEqual(command.shape, (7,))
        self.assertTrue(np.all(command - offsets >= 0.0))
        self.assertTrue(np.all(command - offsets <= np.asarray(self.driver._cfg.max_joint_step)))


class HandMappingTests(unittest.TestCase):
    def test_hand_can_be_disabled_and_reenabled(self):
        cfg = build_config().hand
        cfg.require_feedback_on_enable = False
        driver = LinkerHandDriver(cfg, active_sides=("left",))
        try:
            self.assertTrue(driver.set_enabled(True))
            self.assertTrue(driver.is_enabled())

            self.assertFalse(driver.set_enabled(False))
            self.assertFalse(driver.is_enabled())

            self.assertTrue(driver.set_enabled(True))
            self.assertTrue(driver.is_enabled())
        finally:
            driver.close()

    def setUp(self):
        self.socket_patch = mock.patch(
            "esrobo_teleop.robot.linker_hand_driver.socket.socket"
        )
        socket_factory = self.socket_patch.start()
        socket_factory.return_value.recvfrom.side_effect = BlockingIOError
        cfg = build_config().hand
        cfg.feedback_udp_port = 0
        self.driver = LinkerHandDriver(cfg, udp_port=9)
        self.driver._feedback = {
            "left": np.full(10, 120.0),
            "right": np.full(10, 121.0),
        }

    def tearDown(self):
        self.driver.close()
        self.socket_patch.stop()

    def test_l10_flexion_order_and_direction(self):
        active = np.asarray(
            [limits[1] for limits in self._active_limits()], dtype=np.float64
        )
        left = self.driver._rad_to_servo(active, "left")
        np.testing.assert_array_equal(left[[0, 2, 3, 4, 5]], [0, 0, 0, 0, 0])
        np.testing.assert_array_equal(left[[6, 7, 8]], [120] * 3)

    def test_right_thumb_three_axes_follow_calibrated_targets(self):
        active = np.zeros(10, dtype=np.float64)
        limits = dict(zip(
            [
                "thumb_cmc_roll", "thumb_cmc_yaw", "thumb_cmc_pitch",
                "index_mcp_roll", "index_mcp_pitch", "middle_mcp_pitch",
                "ring_mcp_roll", "ring_mcp_pitch", "pinky_mcp_roll", "pinky_mcp_pitch",
            ],
            self._active_limits(),
        ))
        active[0] = limits["thumb_cmc_roll"][1]
        active[1] = limits["thumb_cmc_yaw"][1]
        active[2] = limits["thumb_cmc_pitch"][1]
        right = self.driver._rad_to_servo(active, "right")
        np.testing.assert_array_equal(right[[0, 1, 9]], [40, 40, 100])

    def test_right_thumb_teleop_endpoints_do_not_change_safe_open_pose(self):
        active = np.zeros(10, dtype=np.float64)
        right = self.driver._rad_to_servo(active, "right")
        enabled = np.asarray(self.driver._cfg.enabled_physical_joints, dtype=np.int64)
        np.testing.assert_array_equal(
            right[enabled], np.asarray(self.driver._cfg.right_open)[enabled]
        )

    def test_left_thumb_side_swing_has_full_calibrated_travel(self):
        active = np.zeros(10, dtype=np.float64)
        active[1] = self._active_limits()[1][1]
        tucked = self.driver._rad_to_servo(active, "left")
        active[1] = 0.0
        extended = self.driver._rad_to_servo(active, "left")
        self.assertEqual(tucked[1], 0)
        self.assertEqual(extended[1], 255)

    def test_right_only_feedback_gate_does_not_require_left_hand(self):
        cfg = build_config().hand
        cfg.feedback_udp_port = 0
        driver = LinkerHandDriver(cfg, udp_port=9, active_sides=("right",))
        try:
            driver._feedback["right"] = np.full(10, 121.0)
            driver._feedback_time["right"] = time.monotonic()
            self.assertTrue(driver.set_enabled(True))
            self.assertTrue(driver.is_enabled())
        finally:
            driver.close()

    def test_right_only_rejects_sdk_invalid_position_sentinel(self):
        cfg = build_config().hand
        cfg.feedback_udp_port = 0
        driver = LinkerHandDriver(cfg, udp_port=9, active_sides=("right",))
        try:
            driver._feedback["right"] = np.full(10, -1.0)
            driver._feedback_time["right"] = time.monotonic()
            self.assertFalse(driver.set_enabled(True))
            self.assertFalse(driver.is_enabled())
        finally:
            driver.close()

    def test_quit_return_uses_fresh_feedback_and_reaches_open_pose(self):
        now = time.monotonic()
        self.driver._feedback_time = {"left": now, "right": now}
        with mock.patch(
            "esrobo_teleop.robot.linker_hand_driver.time.sleep"
        ):
            self.assertTrue(self.driver.return_to_open())
        np.testing.assert_array_equal(
            self.driver._last_cmd["left"], self.driver._cfg.left_open
        )
        np.testing.assert_array_equal(
            self.driver._last_cmd["right"], self.driver._cfg.right_open
        )
        self.assertFalse(self.driver.is_enabled())

    def test_quit_return_is_refused_without_fresh_feedback(self):
        self.driver._feedback_time = {"left": None, "right": None}
        self.assertFalse(self.driver.return_to_open())

    def test_open_pose_status_checks_every_physical_joint(self):
        now = time.monotonic()
        self.driver._feedback_time = {"left": now, "right": now}
        self.driver._feedback = {
            "left": np.asarray(self.driver._cfg.left_open, dtype=np.float64),
            "right": np.asarray(self.driver._cfg.right_open, dtype=np.float64),
        }
        aligned, errors = self.driver.open_pose_status()
        self.assertTrue(aligned)
        self.assertEqual(set(errors), {"left", "right"})

        self.driver._feedback["right"][8] += self.driver._cfg.startup_open_tolerance + 1
        aligned, errors = self.driver.open_pose_status()
        self.assertFalse(aligned)
        self.assertGreater(abs(errors["right"][8]), self.driver._cfg.startup_open_tolerance)

    def test_open_pose_error_names_unresponsive_l10_joint(self):
        errors = {"right": np.zeros(10, dtype=np.float64)}
        errors["right"][0] = -114.0
        detail = self.driver.describe_open_pose_errors(errors)
        self.assertIn("right thumb_cmc_pitch[0]", detail)
        self.assertIn("current=141", detail)
        self.assertIn("target=255", detail)
        self.assertNotIn("thumb_cmc_yaw", detail)

    def test_open_pose_status_supports_distinct_feedback_zero(self):
        self.driver._cfg.right_open_feedback = [
            141, 210, 254, 254, 254, 254, 109, 109, 119, 33,
        ]
        self.driver._active_sides = ("right",)
        self.driver._feedback["right"] = np.asarray(
            self.driver._cfg.right_open_feedback, dtype=np.float64
        )
        self.driver._feedback_time["right"] = time.monotonic()
        aligned, errors = self.driver.open_pose_status()
        self.assertTrue(aligned)
        np.testing.assert_array_equal(errors["right"], np.zeros(10))
        self.assertEqual(self.driver._cfg.right_open[0], 255)

    @staticmethod
    def _active_limits():
        from esrobo_teleop.robot.linker_hand_driver import (
            ACTIVE_HAND_JOINTS,
            ACTIVE_JOINT_LIMITS_RAD,
        )

        return [ACTIVE_JOINT_LIMITS_RAD[name] for name in ACTIVE_HAND_JOINTS]


class ImuSafetyTests(unittest.TestCase):
    def test_imu_delta_is_angle_limited(self):
        device = BodyDevice.__new__(BodyDevice)
        device._cfg = build_config().retarget
        device._hand_orientation_delta_matrices = {}
        rotation_90_z = np.asarray([[0, -1, 0], [1, 0, 0], [0, 0, 1]], dtype=np.float32)
        limited = device._limit_imu_rotation("left", rotation_90_z)
        trace = float(np.trace(limited))
        angle = math.acos(np.clip((trace - 1.0) / 2.0, -1.0, 1.0))
        self.assertAlmostEqual(angle, math.radians(45.0), places=5)

    def test_right_wrist_mapping_projects_local_axis_and_caps_angle(self):
        cfg = build_config().robot
        wrist_rotation = np.eye(3)
        wrist_axes = np.asarray([[1, 0], [0, 1], [0, 0]], dtype=np.float64)
        rotation = np.asarray([[1, 0, 0], [0, 0, -1], [0, 1, 0]], dtype=np.float64)
        offsets = right_imu_rotation_to_wrist_offsets(
            rotation, cfg, wrist_rotation, wrist_axes
        )
        self.assertAlmostEqual(
            offsets[0], math.radians(cfg.right_wrist_max_angle_deg), places=6
        )
        self.assertAlmostEqual(offsets[1], 0.0, places=6)

    def test_right_wrist_mapping_uses_current_world_orientation(self):
        cfg = build_config().robot
        wrist_rotation = np.asarray([[0, -1, 0], [1, 0, 0], [0, 0, 1]], dtype=float)
        wrist_axes = np.asarray([[1, 0], [0, 1], [0, 0]], dtype=float)
        world_delta = wrist_rotation @ np.asarray(
            [[1, 0, 0], [0, math.cos(0.1), -math.sin(0.1)],
             [0, math.sin(0.1), math.cos(0.1)]], dtype=float
        ) @ wrist_rotation.T
        offsets = right_imu_rotation_to_wrist_offsets(
            world_delta, cfg, wrist_rotation, wrist_axes
        )
        np.testing.assert_allclose(offsets, [0.1, 0.0], atol=1e-6)

    def test_three_axis_wrist_offset_has_total_angle_limit(self):
        cfg = build_config().robot
        rotation = mu.rotvec_to_rotation_matrix(np.asarray([0.5, 0.5, 0.5]))
        offsets = imu_rotation_to_wrist_offsets(
            rotation, cfg, np.eye(3), np.eye(3)
        )
        self.assertAlmostEqual(
            float(np.linalg.norm(offsets)),
            math.radians(cfg.right_wrist_max_angle_deg),
            places=6,
        )

    def test_local_palm_axes_project_directly_to_terminal_joints(self):
        cfg = build_config().robot
        # At terminal zero: J5=local Z, J6=local X, J7=local Y.
        axes = np.asarray([[0, 1, 0], [0, 0, 1], [1, 0, 0]], dtype=float)
        local_rotation = mu.rotvec_to_rotation_matrix(np.asarray([0.1, -0.2, 0.3]))
        offsets = imu_local_rotation_to_wrist_offsets(local_rotation, cfg, axes)
        expected = np.asarray([0.3, 0.1, -0.2])
        max_angle = math.radians(cfg.right_wrist_max_angle_deg)
        if np.linalg.norm(expected) > max_angle:
            expected *= max_angle / np.linalg.norm(expected)
        np.testing.assert_allclose(offsets, expected, atol=1e-6)

    def test_right_independent_limit_does_not_scale_other_joint_offsets(self):
        cfg = build_config().robot
        axes = np.asarray([[0, 1, 0], [0, 0, 1], [1, 0, 0]], dtype=float)
        local_rotation = mu.rotvec_to_rotation_matrix(
            np.deg2rad(np.asarray([20.0, -15.0, 60.0]))
        )
        offsets = imu_local_rotation_to_wrist_offsets(
            local_rotation,
            cfg,
            axes,
            limit_total_angle=False,
        )
        np.testing.assert_allclose(
            np.rad2deg(offsets), [40.0, 20.0, -15.0], atol=1e-5
        )

    def test_left_wrist_mapping_uses_urdf_axes_at_current_pose(self):
        cfg = build_config()
        cfg.ik.urdf_path = os.path.join(
            os.path.dirname(os.path.dirname(__file__)),
            "urdf",
            "esrobo_waist_with_head.urdf",
        )
        solver = IkSolver(cfg.ik)
        arm_joints = np.zeros(14, dtype=np.float64)
        arm_joints[1] = -math.pi / 2.0
        wrist_rotation, axes = solver.wrist_orientation_linearization(
            "left", arm_joints, (4, 5, 6)
        )
        rotation = wrist_rotation @ mu.rotvec_to_rotation_matrix(
            axes[:, 0] * 0.05
        ) @ wrist_rotation.T
        offsets = imu_rotation_to_wrist_offsets(
            rotation, cfg.robot, wrist_rotation, axes
        )
        np.testing.assert_allclose(offsets, [0.05, 0.0, 0.0], atol=1e-5)

    def test_both_urdf_wrists_follow_declared_glove_axis_directions(self):
        cfg = build_config()
        cfg.ik.urdf_path = os.path.join(
            os.path.dirname(os.path.dirname(__file__)),
            "urdf",
            "esrobo_waist_with_head.urdf",
        )
        solver = IkSolver(cfg.ik)
        expected_by_side = {
            "left": (
                np.asarray([0.1, 0.0, 0.0]),
                np.asarray([0.0, 0.1, 0.0]),
                np.asarray([0.0, 0.0, 0.1]),
            ),
            # Current configured correction: X -> J5, Y -> J6, Z -> +J7.
            "right": (
                np.asarray([0.1, 0.0, 0.0]),
                np.asarray([0.0, 0.1, 0.0]),
                np.asarray([0.0, 0.0, 0.1]),
            ),
        }
        for side in ("left", "right"):
            glove_to_palm = np.asarray(
                cfg.retarget.hand_imu_local_rotvec_map_for(side), dtype=float
            ).reshape(3, 3)
            directions = np.asarray(getattr(cfg.robot, f"{side}_joint_directions"))
            physical_offsets = np.asarray(getattr(cfg.robot, f"{side}_joint_offsets"))
            side_urdf = -physical_offsets / directions
            full_urdf = np.zeros(14, dtype=np.float64)
            full_urdf[:7] = side_urdf if side == "left" else 0.0
            full_urdf[7:] = side_urdf if side == "right" else 0.0
            _, axes = solver.wrist_orientation_linearization(side, full_urdf, (4, 5, 6))
            axes /= directions[[4, 5, 6]][np.newaxis, :]
            for glove_axis, expected_offsets in zip(np.eye(3), expected_by_side[side]):
                palm_rotvec = glove_to_palm @ (glove_axis * 0.1)
                offsets = imu_local_rotation_to_wrist_offsets(
                    mu.rotvec_to_rotation_matrix(palm_rotvec), cfg.robot, axes
                )
                np.testing.assert_allclose(offsets, expected_offsets, atol=2e-4)


class RightWristDriverTests(unittest.TestCase):
    class FakeArm:
        def __init__(self, joints):
            self.joints = np.asarray(joints, dtype=np.float64)
            self.command = None
            self.normal_mode = False
            self.motion_mode = None
            self.connected = False
            self.disabled = False
            self.reset_called = False

        def connect(self):
            self.connected = True

        def enable(self):
            return True

        def reset(self):
            self.reset_called = True

        def get_joint_enable_states(self):
            return [True] * 7

        def set_normal_mode(self):
            self.normal_mode = True

        def set_motion_mode(self, mode):
            self.motion_mode = mode

        def set_speed_percent(self, _percent):
            pass

        def disable(self):
            self.disabled = True
            return True

        def get_joint_angles(self):
            return self.joints.copy()

        def move_j(self, joints):
            self.command = np.asarray(joints, dtype=np.float64)

    def test_only_terminal_joints_5_6_and_7_receive_target_displacement(self):
        driver = NeroRightWristDriver.__new__(NeroRightWristDriver)
        driver._cfg = build_config().robot
        driver._side = "right"
        driver._arm = self.FakeArm(np.zeros(7))
        driver._session_start = np.zeros(7)
        self.assertTrue(driver.command_offsets(np.asarray([0.2, -0.2, 0.1]), 0.02))
        np.testing.assert_allclose(driver._arm.command[:4], np.zeros(4))
        first_step = driver._cfg.right_wrist_max_acceleration * 0.02**2
        np.testing.assert_allclose(
            driver._arm.command[4:],
            [first_step, -first_step, first_step],
            atol=1e-9,
        )

    def test_wrist_command_velocity_ramps_with_acceleration_limit(self):
        driver = NeroRightWristDriver.__new__(NeroRightWristDriver)
        driver._cfg = build_config().robot
        driver._side = "right"
        driver._arm = self.FakeArm(np.zeros(7))
        driver._session_start = np.zeros(7)

        self.assertTrue(driver.command_offsets(np.asarray([0.2, 0.0, 0.0]), 0.02))
        first = driver._arm.command[4]
        self.assertTrue(driver.command_offsets(np.asarray([0.2, 0.0, 0.0]), 0.02))
        second = driver._arm.command[4]

        self.assertAlmostEqual(
            first,
            driver._cfg.right_wrist_max_acceleration * 0.02**2,
            places=9,
        )
        self.assertAlmostEqual(second, 2.0 * first, places=9)

    def test_wrist_zero_return_requires_consecutive_feedback_and_tolerates_gap(self):
        driver = NeroRightWristDriver.__new__(NeroRightWristDriver)
        driver._cfg = build_config().robot
        driver._cfg.return_feedback_grace_s = 0.2
        driver._cfg.return_verify_samples = 2
        driver._side = "right"
        driver._arm = self.FakeArm(np.zeros(7))
        driver._session_start = np.zeros(7)
        driver._enabled = True
        driver.command_offsets = mock.Mock(side_effect=[False, True, True])
        driver.read_joints = mock.Mock(
            side_effect=[np.zeros(7), np.zeros(7)]
        )

        self.assertTrue(driver.return_wrist_to_neutral())

        self.assertEqual(driver.command_offsets.call_count, 3)
        np.testing.assert_allclose(driver._arm.command, np.zeros(7))

    def test_calibrated_imu_zero_targets_fixed_terminal_joint_zero(self):
        driver = NeroRightWristDriver.__new__(NeroRightWristDriver)
        driver._cfg = build_config().robot
        driver._side = "right"
        current = np.asarray([0.1, 0.2, 0.3, 0.4, 0.25, -0.20, 0.15])
        driver._arm = self.FakeArm(current)
        driver._session_start = current.copy()
        driver._wrist_command_velocity = np.zeros(3)

        self.assertTrue(driver.command_offsets(np.zeros(3), 0.02))

        np.testing.assert_allclose(driver._arm.command[:4], current[:4])
        self.assertLess(driver._arm.command[4], current[4])
        self.assertGreater(driver._arm.command[5], current[5])
        self.assertLess(driver._arm.command[6], current[6])
        np.testing.assert_array_equal(driver.wrist_neutral_positions(), np.zeros(3))

    def test_enable_clears_latched_stop_and_verifies_all_joints(self):
        driver = NeroRightWristDriver.__new__(NeroRightWristDriver)
        driver._cfg = build_config().robot
        driver._side = "right"
        driver._arm = self.FakeArm(np.zeros(7))
        driver._session_start = np.zeros(7)
        driver._enabled = False

        self.assertTrue(driver.enable())
        self.assertTrue(driver._arm.reset_called)
        self.assertTrue(driver._enabled)

        driver._enabled = False
        driver._arm.disabled = False
        driver._arm.get_joint_enable_states = lambda: [True] * 6 + [False]
        self.assertFalse(driver.enable())
        self.assertTrue(driver._arm.disabled)
        self.assertFalse(driver._enabled)

    def test_connect_disables_without_sending_a_position_target(self):
        driver = NeroRightWristDriver.__new__(NeroRightWristDriver)
        driver._cfg = build_config().robot
        driver._side = "right"
        current = np.linspace(-0.3, 0.3, 7)
        driver._arm = self.FakeArm(current)
        driver._session_start = None
        driver._enabled = False
        driver._emergency_stopped = False

        driver.connect()
        self.assertTrue(driver._arm.connected)
        self.assertTrue(driver._arm.disabled)
        self.assertIsNone(driver._arm.command)

    def test_left_connect_uses_same_disable_without_position_target(self):
        driver = NeroWristDriver.__new__(NeroWristDriver)
        driver._cfg = build_config().robot
        driver._side = "left"
        driver._arm = self.FakeArm(np.zeros(7))
        driver._session_start = None
        driver._enabled = False
        driver._emergency_stopped = False

        driver.connect()
        self.assertTrue(driver._arm.connected)
        self.assertTrue(driver._arm.disabled)
        self.assertIsNone(driver._arm.command)

    def test_can_control_holds_current_as_its_first_target(self):
        driver = NeroRightWristDriver.__new__(NeroRightWristDriver)
        driver._cfg = build_config().robot
        driver._side = "right"
        current = np.linspace(-0.3, 0.3, 7)
        driver._arm = self.FakeArm(current)
        driver._session_start = current.copy()
        driver._enabled = False
        driver._emergency_stopped = False
        self.assertTrue(driver.enable())
        self.assertTrue(driver._arm.normal_mode)
        self.assertEqual(driver._arm.motion_mode, "j")
        np.testing.assert_allclose(driver._arm.command, current)


class TeleopQuitTests(unittest.TestCase):
    def test_linked_enable_requires_fresh_finger_targets_before_arm(self):
        node = TeleopNode.__new__(TeleopNode)
        node._arm_with_hand = True
        node._arm_armed = False
        node._hand_enabled = False
        node._body = mock.Mock()
        node._body.hand_joints.return_value = None
        node._hand = mock.Mock()
        node._arm_robot = mock.Mock(return_value=True)

        node._handle_key("e")

        node._hand.set_enabled.assert_not_called()
        node._arm_robot.assert_not_called()
        self.assertFalse(node._arm_armed)
        self.assertFalse(node._hand_enabled)

    def test_linked_enable_does_not_start_hand_when_arm_refuses(self):
        node = TeleopNode.__new__(TeleopNode)
        node._arm_with_hand = True
        node._arm_armed = False
        node._hand_enabled = False
        node._body = mock.Mock()
        node._body.hand_joints.return_value = np.zeros(20)
        node._hand = mock.Mock()
        node._hand.feedback_ready.return_value = True
        node._arm_robot = mock.Mock(return_value=False)

        node._handle_key("e")

        node._hand.set_enabled.assert_not_called()
        self.assertFalse(node._arm_armed)
        self.assertFalse(node._hand_enabled)

    def test_linked_enable_returns_arm_to_zero_if_hand_feedback_disappears(self):
        node = TeleopNode.__new__(TeleopNode)
        node._arm_with_hand = True
        node._arm_armed = False
        node._hand_enabled = False
        node._body = mock.Mock()
        node._body.hand_joints.return_value = np.zeros(20)
        node._hand = mock.Mock()
        node._hand.feedback_ready.return_value = True
        node._hand.set_enabled.return_value = False
        node._arm_robot = mock.Mock(return_value=True)
        node._return_full_arm_to_zero_and_disable = mock.Mock(return_value=(True, True))

        node._handle_key("e")

        node._return_full_arm_to_zero_and_disable.assert_called_once()
        self.assertFalse(node._hand_enabled)

    def test_full_arm_imu_compensation_removes_shoulder_rotation(self):
        node = TeleopNode.__new__(TeleopNode)
        node._cfg = build_config()
        node._arm_side = "left"
        node._ik = mock.Mock()
        baseline_world = mu.rotvec_to_rotation_matrix(np.asarray([0.0, 0.0, 0.4]))
        wrist_local = mu.rotvec_to_rotation_matrix(np.asarray([0.2, 0.0, 0.0]))
        baseline_pose = np.concatenate(
            (np.zeros(3), mu.matrix_to_quat_wxyz(baseline_world))
        )
        desired_pose = np.concatenate(
            (np.zeros(3), mu.matrix_to_quat_wxyz(baseline_world @ wrist_local))
        )
        node._ik.current_task_frame_poses.return_value = {"left_wrist": baseline_pose}
        node._full_arm_terminal_targets = mock.Mock(
            side_effect=lambda rotation: mu.rotation_matrix_to_rotvec(rotation)
        )

        targets = node._compensated_full_arm_terminal_targets(
            desired_pose, np.zeros(14)
        )

        np.testing.assert_allclose(targets, [0.2, 0.0, 0.0], atol=1.0e-6)

    def test_arm_only_startup_aligns_nonzero_disabled_arm_before_teleop(self):
        node = TeleopNode.__new__(TeleopNode)
        node._cfg = build_config()
        node._wrist_imu_side = None
        node._arm_only = True
        node._arm_side = "left"
        node._motors_enabled = False
        node._driver = mock.Mock()
        node._driver.startup_zero_error.return_value = np.deg2rad(
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -10.0]
        )
        node._driver.align_startup_zero_and_disable.return_value = (True, True)

        with mock.patch("builtins.input", return_value=""), mock.patch(
            "sys.stdin.isatty", return_value=True
        ):
            node._ensure_startup_zero_pose()

        node._driver.align_startup_zero_and_disable.assert_called_once_with()
        self.assertFalse(node._motors_enabled)

    def test_arm_only_startup_rejects_unverified_disable_after_alignment(self):
        node = TeleopNode.__new__(TeleopNode)
        node._cfg = build_config()
        node._wrist_imu_side = None
        node._arm_only = True
        node._arm_side = "left"
        node._motors_enabled = False
        node._driver = mock.Mock()
        node._driver.startup_zero_error.return_value = np.deg2rad(
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -10.0]
        )
        node._driver.align_startup_zero_and_disable.return_value = (True, False)

        with mock.patch("builtins.input", return_value=""), mock.patch(
            "sys.stdin.isatty", return_value=True
        ), self.assertRaisesRegex(RuntimeError, "disable_verified=False"):
            node._ensure_startup_zero_pose()

        self.assertTrue(node._motors_enabled)

    def test_arm_only_arming_bootstraps_silent_left_controller_on_e(self):
        node = TeleopNode.__new__(TeleopNode)
        node._cfg = build_config()
        node._wrist_imu_side = None
        node._arm_only = True
        node._arm_side = "left"
        node._motors_enabled = False
        node._driver = mock.Mock()
        node._driver.capture_session_start.side_effect = [False, True]
        node._driver.silent_disabled_startup = True
        node._driver.enable_from_natural_down.return_value = True
        node._driver.read_full_urdf_joints.return_value = np.zeros(14)
        node._body = mock.Mock()
        node._body.is_ready.return_value = True
        node._body.hand_orientation_delta.return_value = np.eye(3)
        node._ik = mock.Mock()
        node._ik.current_task_frame_poses.return_value = {}
        node._ik.wrist_orientation_linearization.return_value = (
            np.eye(3), np.eye(3)
        )
        node._full_arm_wrist_local_axes = None

        self.assertTrue(node._arm_robot())

        node._driver.enable_from_natural_down.assert_called_once_with()
        node._driver.enable.assert_not_called()
        self.assertTrue(node._motors_enabled)

    def test_arm_only_bootstrap_disables_if_feedback_immediately_disappears(self):
        node = TeleopNode.__new__(TeleopNode)
        node._cfg = build_config()
        node._wrist_imu_side = None
        node._arm_only = True
        node._arm_side = "left"
        node._motors_enabled = False
        node._driver = mock.Mock()
        node._driver.capture_session_start.side_effect = [False, True]
        node._driver.silent_disabled_startup = True
        node._driver.enable_from_natural_down.return_value = True
        node._driver.read_full_urdf_joints.return_value = None
        node._driver.disable.return_value = True
        node._body = mock.Mock()
        node._body.is_ready.return_value = True
        node._ik = mock.Mock()

        self.assertFalse(node._arm_robot())

        node._driver.disable.assert_called_once_with()
        self.assertFalse(node._motors_enabled)

    def test_startup_zero_gate_sends_nothing_when_feedback_is_aligned(self):
        node = TeleopNode.__new__(TeleopNode)
        node._cfg = build_config()
        node._wrist_imu_side = "right"
        node._hand = mock.Mock()
        node._hand.open_pose_status.return_value = (True, {"right": np.zeros(10)})
        node._driver = mock.Mock()
        node._driver.read_joints.return_value = np.zeros(7)
        node._driver.wrist_neutral_positions.return_value = np.zeros(3)

        node._ensure_startup_zero_pose()

        node._driver.enable.assert_not_called()
        node._driver.return_wrist_to_neutral.assert_not_called()
        node._hand.align_and_verify_open_pose.assert_not_called()

    def test_startup_zero_gate_aligns_verifies_and_disables(self):
        node = TeleopNode.__new__(TeleopNode)
        node._cfg = build_config()
        node._wrist_imu_side = "right"
        node._motors_enabled = False
        node._hand_enabled = False
        node._hand = mock.Mock()
        node._hand.open_pose_status.return_value = (
            False, {"right": np.full(10, 20.0)}
        )
        node._hand.align_and_verify_open_pose.return_value = (
            True, {"right": np.zeros(10)}
        )
        node._driver = mock.Mock()
        node._driver.read_joints.side_effect = [
            np.asarray([0.1, 0.2, 0.3, 0.4, 0.2, -0.1, 0.1]),
            np.asarray([0.1, 0.2, 0.3, 0.4, 0.0, 0.0, 0.0]),
        ]
        node._driver.wrist_neutral_positions.return_value = np.zeros(3)
        node._driver.capture_session_start.return_value = True
        node._driver.enable.return_value = True
        node._driver.return_wrist_to_neutral.return_value = True
        node._driver.disable.return_value = True

        with mock.patch("builtins.input", return_value=""), mock.patch(
            "sys.stdin.isatty", return_value=True
        ):
            node._ensure_startup_zero_pose()

        node._driver.enable.assert_called_once_with()
        node._driver.return_wrist_to_neutral.assert_called_once_with()
        node._driver.disable.assert_called_once_with()
        node._hand.align_and_verify_open_pose.assert_called_once_with()
        self.assertFalse(node._motors_enabled)

    def test_startup_zero_gate_accepts_gravity_drift_after_verified_disable(self):
        node = TeleopNode.__new__(TeleopNode)
        node._cfg = build_config()
        node._wrist_imu_side = "right"
        node._motors_enabled = False
        node._hand_enabled = False
        node._hand = mock.Mock()
        node._hand.open_pose_status.return_value = (True, {"right": np.zeros(10)})
        node._driver = mock.Mock()
        node._driver.read_joints.side_effect = [
            np.asarray([0.1, 0.2, 0.3, 0.4, 0.2, -0.1, 0.1]),
            np.asarray([0.1, 0.2, 0.3, 0.4, 0.0, 0.0, -0.08]),
        ]
        node._driver.wrist_neutral_positions.return_value = np.zeros(3)
        node._driver.capture_session_start.return_value = True
        node._driver.enable.return_value = True
        node._driver.return_wrist_to_neutral.return_value = True
        node._driver.disable.return_value = True

        with mock.patch("builtins.input", return_value=""), mock.patch(
            "sys.stdin.isatty", return_value=True
        ):
            node._ensure_startup_zero_pose()

        node._driver.return_wrist_to_neutral.assert_called_once_with()
        node._driver.disable.assert_called_once_with()
        self.assertFalse(node._motors_enabled)

    def test_q_disables_wrist_before_returning_hand_to_open_pose(self):
        events = []
        node = TeleopNode.__new__(TeleopNode)
        node._stop = False
        node._arm_armed = True
        node._motors_enabled = True
        node._wrist_imu_side = "right"
        node._return_hand_open_on_close = False
        node._driver = mock.Mock()
        node._driver.return_wrist_to_neutral.side_effect = (
            lambda: events.append("wrist_zero") or True
        )
        node._driver.disable.side_effect = lambda: events.append("arm_disable") or True
        node._driver.disconnect.side_effect = lambda: events.append("arm_disconnect")
        node._hand = mock.Mock()
        node._hand.return_to_open.side_effect = lambda: events.append("hand_open") or True
        node._hand.close.side_effect = lambda: events.append("hand_close")
        node._body = mock.Mock()

        node._handle_key("q")
        node.close()

        self.assertFalse(node._arm_armed)
        self.assertEqual(events[:3], ["wrist_zero", "arm_disable", "hand_open"])

    def test_full_arm_q_is_cancelled_when_zero_return_does_not_disable(self):
        node = TeleopNode.__new__(TeleopNode)
        node._stop = False
        node._arm_armed = True
        node._motors_enabled = True
        node._arm_only = True
        node._arm_side = "right"
        node._wrist_imu_side = None
        node._return_hand_open_on_close = False
        node._hand = None
        node._return_full_arm_to_zero_and_disable = mock.Mock(
            return_value=(False, False)
        )

        node._handle_key("q")

        self.assertFalse(node._stop)
        self.assertFalse(node._arm_armed)
        node._return_full_arm_to_zero_and_disable.assert_called_once_with(
            "operator pressed 'q'"
        )

    def test_operator_disable_key_is_explicit_and_does_not_return_to_zero(self):
        node = TeleopNode.__new__(TeleopNode)
        node._arm_armed = True
        node._motors_enabled = True
        node._manual_intervention_required = True
        node._arm_with_hand = False
        node._driver = mock.Mock()
        node._driver.disable.return_value = True

        node._handle_key("d")

        self.assertFalse(node._arm_armed)
        self.assertFalse(node._motors_enabled)
        self.assertFalse(node._manual_intervention_required)
        node._driver.disable.assert_called_once_with()
        node._driver.return_to_zero_and_disable.assert_not_called()

    def test_s_returns_wrist_to_zero_before_disabling(self):
        events = []
        node = TeleopNode.__new__(TeleopNode)
        node._arm_armed = True
        node._motors_enabled = True
        node._wrist_imu_side = "right"
        node._driver = mock.Mock()
        node._driver.return_wrist_to_neutral.side_effect = (
            lambda: events.append("wrist_zero") or True
        )
        node._driver.disable.side_effect = lambda: events.append("arm_disable") or True

        node._handle_key("s")

        self.assertFalse(node._arm_armed)
        self.assertFalse(node._motors_enabled)
        self.assertEqual(events, ["wrist_zero", "arm_disable"])

    def test_wrist_range_is_enlarged_but_total_angle_remains_bounded(self):
        cfg = build_config().robot
        self.assertEqual(cfg.right_wrist_max_angle_deg, 40.0)
        axes = np.eye(3)
        offsets = imu_local_rotation_to_wrist_offsets(
            mu.rotvec_to_rotation_matrix(np.asarray([1.0, 1.0, 1.0])), cfg, axes
        )
        self.assertLessEqual(
            np.linalg.norm(offsets), math.radians(40.0) + 1.0e-9
        )

    def test_wrist_arming_builds_mapping_at_fixed_joint_zero(self):
        node = TeleopNode.__new__(TeleopNode)
        node._wrist_imu_side = "left"
        node._body = mock.Mock()
        node._body.hand_orientation_delta.return_value = np.eye(3)
        node._driver = mock.Mock()
        current = np.asarray([0.1, 0.2, 0.3, 0.4, 0.5, -0.4, 0.3])
        node._driver.capture_session_start.return_value = True
        node._driver.read_joints.return_value = current.copy()
        node._driver.wrist_neutral_positions.return_value = np.zeros(3)
        node._driver.physical_to_urdf.side_effect = lambda joints: joints.copy()
        node._driver.wrist_physical_directions.return_value = np.ones(3)
        node._driver.enable.return_value = True
        node._driver.return_wrist_to_neutral.return_value = True
        node._ik = mock.Mock()
        node._ik.wrist_orientation_linearization.return_value = (
            np.eye(3), np.eye(3)
        )
        node._cfg = build_config()
        node._motors_enabled = False
        node._current_arm_joints = None
        node._wrist_local_axes = None
        node._last_wrist_diagnostic_time = 0.0

        self.assertTrue(node._arm_robot())

        mapped_full = node._ik.wrist_orientation_linearization.call_args.args[1]
        left_offsets = np.asarray(node._cfg.robot.left_joint_offsets)
        np.testing.assert_allclose(mapped_full[:4], current[:4] - left_offsets[:4])
        np.testing.assert_allclose(mapped_full[4:7], np.zeros(3))
        node._driver.enable.assert_called_once_with()
        node._driver.return_wrist_to_neutral.assert_called_once_with()

    def test_wrist_arming_accepts_glove_away_from_calibrated_neutral(self):
        node = TeleopNode.__new__(TeleopNode)
        node._wrist_imu_side = "right"
        node._body = mock.Mock()
        node._body.hand_orientation_delta.return_value = mu.rotvec_to_rotation_matrix(
            np.asarray([0.0, 0.0, math.radians(8.0)])
        )
        node._driver = mock.Mock()
        current = np.asarray([0.1, 0.2, 0.3, 0.4, 0.1, -0.1, 0.1])
        node._driver.capture_session_start.return_value = True
        node._driver.read_joints.return_value = current
        node._driver.enable.return_value = True
        node._driver.return_wrist_to_neutral.return_value = True
        node._ik = mock.Mock()
        node._ik.wrist_orientation_linearization.return_value = (
            np.eye(3), np.eye(3)
        )
        node._cfg = build_config()
        node._motors_enabled = False
        node._current_arm_joints = None
        node._wrist_local_axes = None

        self.assertTrue(node._arm_robot())

        node._driver.capture_session_start.assert_called_once_with()
        node._driver.enable.assert_called_once_with()
        node._driver.return_wrist_to_neutral.assert_called_once_with()

    def test_wrist_arming_disables_when_zero_alignment_fails(self):
        node = TeleopNode.__new__(TeleopNode)
        node._wrist_imu_side = "right"
        node._body = mock.Mock()
        node._body.hand_orientation_delta.return_value = np.eye(3)
        node._driver = mock.Mock()
        current = np.asarray([0.1, 0.2, 0.3, 0.4, 0.1, -0.1, 0.1])
        node._driver.capture_session_start.return_value = True
        node._driver.read_joints.return_value = current
        node._driver.enable.return_value = True
        node._driver.return_wrist_to_neutral.return_value = False
        node._driver.disable.return_value = True
        node._ik = mock.Mock()
        node._ik.wrist_orientation_linearization.return_value = (
            np.eye(3), np.eye(3)
        )
        node._cfg = build_config()
        node._motors_enabled = False
        node._current_arm_joints = None
        node._wrist_local_axes = None

        self.assertFalse(node._arm_robot())

        node._driver.return_wrist_to_neutral.assert_called_once_with()
        node._driver.disable.assert_called_once_with()
        self.assertFalse(node._motors_enabled)

    def test_wrist_fault_disables_enabled_arm(self):
        node = TeleopNode.__new__(TeleopNode)
        node._arm_armed = True
        node._motors_enabled = True
        node._wrist_imu_side = "right"
        node._driver = mock.Mock()
        node._driver.disable.return_value = True

        node._stop_wrist_for_fault("glove IMU became stale")

        self.assertFalse(node._arm_armed)
        self.assertFalse(node._motors_enabled)
        node._driver.return_wrist_to_neutral.assert_called_once_with()
        node._driver.disable.assert_called_once_with()

    def test_invalid_feedback_fault_skips_zero_return_before_disabling(self):
        node = TeleopNode.__new__(TeleopNode)
        node._arm_armed = True
        node._motors_enabled = True
        node._wrist_imu_side = "right"
        node._driver = mock.Mock()
        node._driver.disable.return_value = True

        node._stop_wrist_for_fault(
            "invalid/missing arm feedback", allow_return=False
        )

        self.assertFalse(node._arm_armed)
        self.assertFalse(node._motors_enabled)
        node._driver.return_wrist_to_neutral.assert_not_called()
        node._driver.disable.assert_called_once_with()

    def test_imu_diagnostic_key_only_toggles_logging(self):
        node = TeleopNode.__new__(TeleopNode)
        node._wrist_imu_side = "right"
        node._wrist_diagnostics_enabled = False
        node._arm_armed = False

        node._handle_key("i")
        self.assertTrue(node._wrist_diagnostics_enabled)
        self.assertFalse(node._arm_armed)

        node._handle_key("i")
        self.assertFalse(node._wrist_diagnostics_enabled)
        self.assertFalse(node._arm_armed)

    def test_unconfirmed_fault_disable_is_retried_during_close(self):
        node = TeleopNode.__new__(TeleopNode)
        node._stop = False
        node._arm_armed = True
        node._motors_enabled = True
        node._wrist_imu_side = "right"
        node._return_hand_open_on_close = False
        node._driver = mock.Mock()
        node._driver.disable.side_effect = [False, True]
        node._body = mock.Mock()
        node._hand = mock.Mock()

        node._stop_wrist_for_fault("glove IMU became stale")
        self.assertTrue(node._motors_enabled)
        node.close()

        self.assertEqual(node._driver.disable.call_count, 2)


class NeroArmFeedbackTests(unittest.TestCase):
    class Feedback:
        def __init__(self, values, stamp):
            self.msg = values
            self.timestamp = stamp

    class DriverState:
        def __init__(self, enabled):
            self.msg = mock.Mock()
            self.msg.foc_status.driver_enable_status = enabled

    def test_motion_mode_uses_sdk_enum_attribute(self):
        sentinel = object()

        class MotionMode:
            J = sentinel

        robot = mock.Mock()
        robot.OPTIONS.MOTION_MODE = MotionMode
        arm = NeroArm.__new__(NeroArm)
        arm._robot = robot

        arm.set_motion_mode("j")
        robot.set_motion_mode.assert_called_once_with(sentinel)

    def test_can_feedback_push_uses_mode_field_without_position_command(self):
        robot = mock.Mock()
        robot._msg_mode.enable_can_push = 0
        robot._msg_mode.move_mode = 1
        transmitted = []
        robot._set_mode.side_effect = lambda: transmitted.append(
            (robot._msg_mode.enable_can_push, robot._msg_mode.move_mode)
        )
        arm = NeroArm.__new__(NeroArm)
        arm._robot = robot

        self.assertTrue(arm.request_can_feedback_push())

        self.assertEqual(transmitted, [(0x01, 0xFF)])
        self.assertEqual(robot._msg_mode.enable_can_push, 0)
        self.assertEqual(robot._msg_mode.move_mode, 1)
        robot.move_j.assert_not_called()

    def test_known_target_enable_preloads_and_repeats_zero_around_enable(self):
        cfg = build_config().robot
        arm = NeroArm.__new__(NeroArm)
        arm._cfg = cfg
        arm._robot = mock.Mock()
        arm._robot.OPTIONS.MOTION_MODE.J = "j-mode"
        arm._last_disable_states = None
        arm.request_can_feedback_push = mock.Mock(return_value=True)
        arm.get_joint_angles = mock.Mock(return_value=np.asarray(cfg.left_joint_offsets))
        arm.get_joint_enable_states = mock.Mock(return_value=[True] * 7)

        joints, states = arm.enable_at_known_target(
            np.asarray(cfg.left_joint_offsets), speed_percent=10, timeout=0.5
        )

        np.testing.assert_allclose(joints, cfg.left_joint_offsets)
        self.assertEqual(states, [True] * 7)
        self.assertEqual(arm._robot.move_j.call_count, 2)
        arm._robot.enable.assert_called_once_with()
        arm.request_can_feedback_push.assert_called_once_with()

    def test_known_target_enable_retries_sdk_handshake_until_feedback_arrives(self):
        cfg = build_config().robot
        zero = np.asarray(cfg.left_joint_offsets)
        arm = NeroArm.__new__(NeroArm)
        arm._cfg = cfg
        arm._robot = mock.Mock()
        arm._robot.OPTIONS.MOTION_MODE.J = "j-mode"
        arm._last_disable_states = None
        arm.request_can_feedback_push = mock.Mock(return_value=True)
        arm.get_joint_angles = mock.Mock(side_effect=[None, zero.copy()])
        arm.get_joint_enable_states = mock.Mock(side_effect=[None, [True] * 7])

        joints, states = arm.enable_at_known_target(zero, speed_percent=10, timeout=0.5)

        np.testing.assert_allclose(joints, zero)
        self.assertEqual(states, [True] * 7)
        self.assertEqual(arm._robot.enable.call_count, 2)

    def test_legacy_leader_feedback_is_accepted_without_mode_command(self):
        arm = NeroArm.__new__(NeroArm)
        robot = mock.Mock()
        robot.get_joint_angles.return_value = None
        robot.get_leader_joint_angles.side_effect = [
            self.Feedback([0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 0.0], 1.0),
            self.Feedback([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7], 2.0),
            self.Feedback([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7], 3.0),
        ]
        arm._robot = robot

        joints = arm.get_joint_angles(timeout=0.1)

        np.testing.assert_allclose(joints, [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7])
        self.assertEqual(arm.last_feedback_timestamp, 3.0)
        self.assertFalse(any("mode" in str(call) for call in robot.mock_calls))

    def test_disable_does_not_treat_missing_status_as_disabled(self):
        arm = NeroArm.__new__(NeroArm)
        robot = mock.Mock()
        robot.disable.return_value = True
        robot.get_driver_states.return_value = None
        arm._robot = robot

        self.assertFalse(arm.disable(timeout=0.02))

    def test_best_effort_disable_covers_broadcast_and_every_joint(self):
        arm = NeroArm.__new__(NeroArm)
        arm._robot = mock.Mock()

        arm.best_effort_disable_once()

        self.assertEqual(
            arm._robot.disable.call_args_list,
            [mock.call()] + [mock.call(index) for index in range(1, 8)],
        )

    def test_disable_requires_all_seven_verified_disabled_bits(self):
        arm = NeroArm.__new__(NeroArm)
        robot = mock.Mock()
        robot.disable.return_value = True
        robot.get_driver_states.side_effect = [self.DriverState(False) for _ in range(7)]
        arm._robot = robot

        self.assertTrue(arm.disable(timeout=0.1))

    def test_disable_preserves_feedback_when_all_joints_are_already_disabled(self):
        arm = NeroArm.__new__(NeroArm)
        robot = mock.Mock()
        robot.get_driver_states.side_effect = [self.DriverState(False) for _ in range(7)]
        arm._robot = robot

        self.assertTrue(arm.disable(timeout=0.1))
        robot.disable.assert_not_called()
        self.assertEqual(arm.last_disable_states, [False] * 7)

    def test_disable_retries_a_still_enabled_joint_individually(self):
        arm = NeroArm.__new__(NeroArm)
        robot = mock.Mock()
        robot.disable.return_value = True
        first = [False, False, True, False, False, False, False]
        second = [False] * 7
        robot.get_driver_states.side_effect = [
            self.DriverState(value) for value in first + first + second
        ]
        arm._robot = robot

        self.assertTrue(arm.disable(timeout=0.2))
        self.assertIn(mock.call(3), robot.disable.call_args_list)
        self.assertEqual(arm.last_disable_states, second)


class StartupRebaseTests(unittest.TestCase):
    def test_ik_reuses_previous_target_and_stays_within_feedback_lead(self):
        cfg = build_config()
        cfg.ik.urdf_path = os.path.abspath(os.path.join(
            os.path.dirname(__file__), "..", "urdf", "esrobo_waist_with_head.urdf"
        ))
        solver = IkSolver(cfg.ik)
        current = np.zeros(14)
        desired = current.copy()
        desired[:4] = np.asarray([0.25, -0.18, 0.20, 0.40])
        target_poses = solver.current_task_frame_poses(desired)
        current_poses = solver.current_task_frame_poses(current)

        def solve_once():
            result = solver.solve(
                target_poses["left_wrist"],
                current_poses["right_wrist"],
                current,
                left_elbow_pose=target_poses["left_elbow"],
                right_elbow_pose=current_poses["right_elbow"],
                partitioned_side="left",
                terminal_joint_targets=np.zeros(3),
            )
            poses = solver.current_task_frame_poses(result)
            error = (
                np.linalg.norm(poses["left_elbow"][:3] - target_poses["left_elbow"][:3])
                + np.linalg.norm(poses["left_wrist"][:3] - target_poses["left_wrist"][:3])
            )
            return result, error

        first, first_error = solve_once()
        second, second_error = solve_once()

        self.assertLess(second_error, first_error * 0.1)
        self.assertLessEqual(
            np.max(np.abs(second - current)), cfg.ik.max_command_lead + 1.0e-12
        )

    def test_partitioned_right_ik_maps_segment_bend_directly_to_j4(self):
        cfg = build_config()
        cfg.ik.urdf_path = os.path.abspath(os.path.join(
            os.path.dirname(__file__), "..", "urdf", "esrobo_waist_with_head.urdf"
        ))
        cfg.ik.max_joint_position_delta = 0.0
        solver = IkSolver(cfg.ik)
        current = np.zeros(14)
        current[10] = math.radians(10.0)
        current[11:14] = np.asarray([0.12, -0.08, 0.06])
        poses = solver.current_task_frame_poses(current)
        flexed = current.copy()
        flexed[10] = math.radians(30.0)
        flexed_poses = solver.current_task_frame_poses(flexed)

        targets = solver.solve(
            poses["left_wrist"],
            flexed_poses["right_wrist"],
            current,
            left_elbow_pose=poses["left_elbow"],
            right_elbow_pose=flexed_poses["right_elbow"],
            partitioned_side="right",
            terminal_joint_targets=current[11:14],
        )

        self.assertAlmostEqual(math.degrees(targets[10]), 30.0, delta=0.1)
        np.testing.assert_allclose(targets[11:14], current[11:14], atol=1.0e-9)

    def test_partitioned_left_ik_reserves_terminal_joints_for_glove(self):
        cfg = build_config()
        cfg.ik.urdf_path = os.path.abspath(os.path.join(
            os.path.dirname(__file__), "..", "urdf", "esrobo_waist_with_head.urdf"
        ))
        solver = IkSolver(cfg.ik)
        current = np.zeros(14)
        poses = solver.current_task_frame_poses(current)
        elbow = poses["left_elbow"].copy()
        wrist = poses["left_wrist"].copy()
        elbow[0] += 0.01
        wrist[0] += 0.02
        terminal = np.asarray([0.10, -0.20, 0.30])

        targets = solver.solve(
            wrist,
            poses["right_wrist"],
            current,
            left_elbow_pose=elbow,
            right_elbow_pose=poses["right_elbow"],
            partitioned_side="left",
            terminal_joint_targets=terminal,
        )

        np.testing.assert_allclose(targets[4:7], terminal, atol=1.0e-9)
        np.testing.assert_allclose(targets[7:14], 0.0, atol=1.0e-7)
        self.assertGreater(np.linalg.norm(targets[:4]), 1.0e-4)

    def test_fk_rebase_uses_measured_joint_configuration(self):
        cfg = build_config()
        cfg.ik.urdf_path = os.path.abspath(os.path.join(
            os.path.dirname(__file__), "..", "urdf", "esrobo_waist_with_head.urdf"
        ))
        solver = IkSolver(cfg.ik)
        physical_zero_as_urdf = np.zeros(14)
        physical_zero_as_urdf[[1, 8]] = -math.pi / 2
        poses = solver.current_task_frame_poses(physical_zero_as_urdf)
        self.assertEqual(set(poses), {
            "left_shoulder", "left_elbow", "left_wrist",
            "right_shoulder", "right_elbow", "right_wrist",
        })
        for pose in poses.values():
            self.assertEqual(pose.shape, (7,))
            self.assertTrue(np.all(np.isfinite(pose)))

        self.assertAlmostEqual(
            np.linalg.norm(poses["left_elbow"][:3] - poses["left_shoulder"][:3]),
            0.31,
            places=3,
        )

        rotation, axes = solver.wrist_orientation_linearization(
            "right", physical_zero_as_urdf, (5, 6)
        )
        self.assertEqual(rotation.shape, (3, 3))
        self.assertEqual(axes.shape, (3, 2))
        self.assertEqual(np.linalg.matrix_rank(axes), 2)
        np.testing.assert_allclose(np.linalg.norm(axes, axis=0), [1.0, 1.0], atol=2e-3)


if __name__ == "__main__":
    unittest.main()
