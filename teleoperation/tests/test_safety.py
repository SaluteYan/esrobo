import contextlib
import io
import math
import os
import threading
import time
import unittest
from unittest import mock

import numpy as np

from esrobo_teleop import math_utils as mu
from esrobo_teleop.config import build_config
from esrobo_teleop.device.body_device import BodyDevice
from esrobo_teleop.ik.solver import IkSolver
from esrobo_teleop.robot.linker_hand_driver import LinkerHandDriver
from esrobo_teleop.robot.return_planner import ReturnResult
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
from esrobo_teleop.teleop_node import TeleopNode, _run_until_safe_exit


class NeroMappingTests(unittest.TestCase):
    def setUp(self):
        self.driver = NeroDualArmDriver.__new__(NeroDualArmDriver)
        self.driver._cfg = build_config().robot
        self.driver._session_start = None
        self.driver._command_velocity = {
            "left": np.zeros(7),
            "right": np.zeros(7),
        }

    def test_right_j4_speed_increase_preserves_feedback_dt_acceleration_bound(self):
        _, current = self.driver._mapping("right")
        target = current.copy()
        target[3] += 1.0
        previous_velocity = np.zeros(7)
        for dt in [.02, .025, .015] * 18:
            following = self.driver._clamp(target, current, "right", dt)
            velocity = (following - current) / dt
            self.assertLessEqual(abs(velocity[3]), math.radians(30) + 1e-6)
            self.assertLessEqual(abs(velocity[3] - previous_velocity[3]), math.radians(50) * dt + 1e-6)
            self.assertLessEqual(abs(following[3] - current[3]), math.radians(2.5) + 1e-6)
            current, previous_velocity = following, velocity
        self.assertGreater(velocity[3], math.radians(29))
        diagnostics = self.driver._last_rate_limit_diagnostics["right"]
        self.assertAlmostEqual(diagnostics["feedback_dt_s"], .015)
        self.assertAlmostEqual(diagnostics["velocity_limit_deg_s"], 30, places=4)

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
        target_urdf = np.asarray([2.0, -2.0, 4.0, 2.4, 2.0, -2.0, 2.0])
        target_physical, _ = self.driver._full_to_physical(
            np.concatenate([target_urdf, target_urdf])
        )
        output = self.driver._clamp(target_physical, target_physical, "left", 0.02)
        expected_urdf = np.asarray(
            [1.047198, -1.308997, 1.570796, 2.094395, 1.047198, -0.523599, 0.959931]
        )
        expected_physical, _ = self.driver._full_to_physical(
            np.concatenate([expected_urdf, expected_urdf])
        )
        np.testing.assert_allclose(output, expected_physical)

    def test_joint3_soft_limits_match_commissioned_side_specific_envelopes(self):
        left_lower, left_upper = self.driver._soft_limits_physical("left")
        right_lower, right_upper = self.driver._soft_limits_physical("right")

        self.assertAlmostEqual(math.degrees(left_lower[2]), -150.0, places=4)
        self.assertAlmostEqual(math.degrees(left_upper[2]), 90.0, places=4)
        self.assertAlmostEqual(math.degrees(right_lower[2]), -90.0, places=4)
        self.assertAlmostEqual(math.degrees(right_upper[2]), 150.0, places=4)

    def test_full_arm_commissioning_rate_limits_are_per_joint(self):
        cfg = build_config().robot

        self.assertFalse(cfg.synchronize_joint_motion)
        np.testing.assert_allclose(
            np.degrees(cfg.max_joint_velocity), [26.0, 26.0, 24.0, 24.0, 24.0, 24.0, 24.0],
            atol=1.0e-4,
        )
        np.testing.assert_allclose(
            np.degrees(cfg.max_joint_acceleration),
            [40.0, 40.0, 45.0, 50.0, 50.0, 50.0, 50.0],
            atol=1.0e-4,
        )
        np.testing.assert_allclose(
            np.degrees(cfg.max_joint_step), [2.8, 2.8, 2.5, 2.5, 2.5, 2.5, 2.5],
            atol=1.0e-4,
        )
        np.testing.assert_allclose(
            np.degrees(cfg.left_max_joint_velocity),
            [26.0, 26.0, 24.0, 30.0, 24.0, 24.0, 24.0],
            atol=1.0e-4,
        )
        np.testing.assert_allclose(
            np.degrees(cfg.left_max_joint_acceleration),
            [40.0, 40.0, 45.0, 50.0, 50.0, 50.0, 50.0],
            atol=1.0e-4,
        )
        np.testing.assert_allclose(
            np.degrees(cfg.left_max_joint_step),
            [2.8, 2.8, 2.5, 2.5, 2.5, 2.5, 2.5],
            atol=1.0e-4,
        )
        np.testing.assert_allclose(
            self.driver._joint_rate_limit_vector("right", "max_joint_velocity"),
            cfg.right_max_joint_velocity,
        )
        np.testing.assert_allclose(
            np.degrees(cfg.right_max_joint_velocity),
            [26, 26, 24, 30, 24, 24, 24], atol=1e-4,
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

    def test_full_arm_commissioning_settings_match_between_sides(self):
        cfg = build_config().ik
        self.assertEqual(cfg.left_j3_retarget_gain, 1.0)
        self.assertEqual(cfg.right_j3_retarget_gain, 1.0)
        self.assertEqual(cfg.iterations_per_cycle, 3)
        self.assertLess(
            cfg.j3_observability_start_flexion_deg,
            cfg.j3_observability_full_flexion_deg,
        )
        retarget = build_config().retarget
        self.assertEqual(
            retarget.bend_plane_observability_start_delta_deg,
            cfg.j3_observability_start_flexion_deg,
        )
        self.assertEqual(
            retarget.bend_plane_observability_full_delta_deg,
            cfg.j3_observability_full_flexion_deg,
        )
        robot = build_config().robot
        self.assertTrue(robot.left_collision_protection_enabled)
        self.assertTrue(robot.right_collision_protection_enabled)
        self.assertTrue(robot.left_command_trajectory_enabled)
        self.assertTrue(robot.right_command_trajectory_enabled)
        np.testing.assert_allclose(robot.left_torque_deviation_limits_nm[:2], [25., 25.])
        np.testing.assert_allclose(robot.right_torque_deviation_limits_nm[:2], [21., 21.])
        np.testing.assert_allclose(
            robot.left_torque_deviation_limits_nm[2:],
            robot.right_torque_deviation_limits_nm[2:],
        )
        np.testing.assert_allclose(robot.left_max_joint_step, robot.right_max_joint_step)
        np.testing.assert_allclose(
            robot.left_max_joint_velocity, robot.right_max_joint_velocity
        )
        self.assertAlmostEqual(np.degrees(robot.right_max_joint_velocity[3]), 30, places=4)
        np.testing.assert_allclose(
            robot.left_max_joint_acceleration, robot.right_max_joint_acceleration
        )

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

    def test_arm_segment_limiter_reapplies_plane_gate_after_bend_reduction(self):
        shoulder = np.zeros(3)
        upper = np.array([0.0, 0.0, -1.0])
        robot_bend = np.deg2rad(2.0)
        robot_forearm = np.array([
            np.sin(robot_bend), 0.0, -np.cos(robot_bend)
        ])
        live_bend = np.deg2rad(20.0)
        live_forearm = np.array([
            0.0, np.sin(live_bend), -np.cos(live_bend)
        ])
        previous_elbow = np.r_[upper * 0.4, [1.0, 0.0, 0.0, 0.0]]
        previous_wrist = np.r_[
            previous_elbow[:3] + robot_forearm * 0.3,
            [1.0, 0.0, 0.0, 0.0],
        ]
        target_elbow = previous_elbow.copy()
        target_wrist = np.r_[
            target_elbow[:3] + live_forearm * 0.3,
            [1.0, 0.0, 0.0, 0.0],
        ]

        def project(actual_upper, actual_forearm):
            return BodyDevice._stabilize_forearm_plane(
                actual_upper, actual_forearm, upper, robot_forearm,
                plane_start_deg=4.0, plane_full_deg=10.0,
            )

        elbow, wrist = BodyDevice._limit_arm_segment_translations(
            previous_elbow,
            previous_wrist,
            target_elbow,
            target_wrist,
            shoulder,
            0.02,
            0.1,
            forearm_projector=project,
        )

        actual_upper = mu.normalize_vector(elbow[:3] - shoulder)
        actual_forearm = mu.normalize_vector(wrist[:3] - elbow[:3])
        actual_bend = np.arccos(np.clip(np.dot(actual_upper, actual_forearm), -1, 1))
        actual_axis = mu.normalize_vector(np.cross(actual_upper, actual_forearm))
        calibrated_axis = mu.normalize_vector(np.cross(upper, robot_forearm))
        self.assertLess(np.degrees(actual_bend - robot_bend), 4.0)
        self.assertGreater(np.dot(actual_axis, calibrated_axis), 0.999)
        self.assertLessEqual(np.linalg.norm(wrist[:3] - previous_wrist[:3]), 0.002001)

    def test_arm_segment_limiter_does_not_project_inside_a_binary_search(self):
        shoulder = np.zeros(3)
        upper = np.array([0.0, 0.0, -1.0])
        robot_bend = np.deg2rad(2.0)
        robot_forearm = np.array([
            np.sin(robot_bend), 0.0, -np.cos(robot_bend)
        ])
        target_bend = np.deg2rad(73.0)
        target_forearm = np.array([
            0.0, np.sin(target_bend), -np.cos(target_bend)
        ])
        previous_elbow = np.r_[upper * 0.31, [1.0, 0.0, 0.0, 0.0]]
        previous_wrist = np.r_[
            previous_elbow[:3] + robot_forearm * 0.399,
            [1.0, 0.0, 0.0, 0.0],
        ]
        target_elbow = previous_elbow.copy()
        target_wrist = np.r_[
            target_elbow[:3] + target_forearm * 0.399,
            [1.0, 0.0, 0.0, 0.0],
        ]

        def stabilize(actual_upper, actual_forearm):
            return BodyDevice._stabilize_forearm_plane(
                actual_upper, actual_forearm, upper, robot_forearm,
                plane_start_deg=4.0, plane_full_deg=10.0,
            )

        projector = mock.Mock(side_effect=stabilize)
        diagnostics = {}
        elbow, wrist = BodyDevice._limit_arm_segment_translations(
            previous_elbow, previous_wrist, target_elbow, target_wrist,
            shoulder, 0.22, 0.02,
            forearm_projector=projector,
            diagnostics=diagnostics,
        )

        self.assertLessEqual(projector.call_count, 2)
        self.assertEqual(diagnostics["method"], "arm_state_arc_backtrack")
        self.assertGreater(diagnostics["progress"], 0.0)
        self.assertLessEqual(np.linalg.norm(elbow[:3] - previous_elbow[:3]), 0.004401)
        self.assertLessEqual(np.linalg.norm(wrist[:3] - previous_wrist[:3]), 0.004401)

    def test_arm_segment_limiter_sustained_deep_bend_converges(self):
        shoulder = np.zeros(3)
        upper = np.array([0.0, 0.0, -1.0])
        robot_bend = np.deg2rad(2.0)
        robot_forearm = np.array([
            np.sin(robot_bend), 0.0, -np.cos(robot_bend)
        ])
        target_bend = np.deg2rad(73.0)
        target_forearm = np.array([
            0.0, np.sin(target_bend), -np.cos(target_bend)
        ])
        elbow = np.r_[upper * 0.31, [1.0, 0.0, 0.0, 0.0]]
        wrist = np.r_[
            elbow[:3] + robot_forearm * 0.399,
            [1.0, 0.0, 0.0, 0.0],
        ]
        target_elbow = elbow.copy()
        target_wrist = np.r_[
            target_elbow[:3] + target_forearm * 0.399,
            [1.0, 0.0, 0.0, 0.0],
        ]

        def stabilize(actual_upper, actual_forearm):
            return BodyDevice._stabilize_forearm_plane(
                actual_upper, actual_forearm, upper, robot_forearm,
                plane_start_deg=4.0, plane_full_deg=10.0,
            )

        bends = []
        for _ in range(220):
            previous_elbow = elbow.copy()
            previous_wrist = wrist.copy()
            elbow, wrist = BodyDevice._limit_arm_segment_translations(
                elbow, wrist, target_elbow, target_wrist,
                shoulder, 0.22, 0.02,
                forearm_projector=stabilize,
            )
            self.assertLessEqual(
                np.linalg.norm(elbow[:3] - previous_elbow[:3]), 0.004401
            )
            self.assertLessEqual(
                np.linalg.norm(wrist[:3] - previous_wrist[:3]), 0.004401
            )
            actual_upper = mu.normalize_vector(elbow[:3] - shoulder)
            actual_forearm = mu.normalize_vector(wrist[:3] - elbow[:3])
            bends.append(float(np.degrees(np.arccos(np.clip(
                np.dot(actual_upper, actual_forearm), -1.0, 1.0
            )))))
            self.assertAlmostEqual(np.linalg.norm(elbow[:3] - shoulder), 0.31, places=6)
            self.assertAlmostEqual(np.linalg.norm(wrist[:3] - elbow[:3]), 0.399, places=6)

        self.assertGreaterEqual(min(np.diff(bends)), -0.02)
        self.assertAlmostEqual(bends[-1], 73.0, places=3)


class ArmVectorCoordinateTests(unittest.TestCase):
    def test_microbend_reference_seeds_filters_at_robot_neutral(self):
        for side in ("left", "right"):
            device = self._direction_filter_device()
            device._cfg.elbow_angle_mapping = "responsive_absolute"
            device._signs = np.ones(3)
            device._held_segment_directions = {"left": {}, "right": {}}
            device._last_body_frame_packet_time_monotonic = 10.0
            rotation = mu.rotvec_to_rotation_matrix([0.2, 0.0, 0.0])
            upper = rotation @ np.array([0.0, 0.0, -0.3])
            forearm = rotation @ np.array([0.3 * np.sin(np.deg2rad(18)), 0.0,
                                            -0.3 * np.cos(np.deg2rad(18))])
            points = dict(shoulder=np.zeros(3), elbow=upper, wrist=upper + forearm)
            device._reference_left_points = points if side == "left" else None
            device._reference_right_points = points if side == "right" else None
            device._robot_arm_reference = mock.Mock(return_value=(
                np.zeros(3), np.array([0, 0, -0.31]), np.array([0, 0, -0.7]), np.eye(3)
            ))
            self.assertAlmostEqual(np.degrees(device.arm_reference_flexion_rad(side)), 18, places=3)
            device.seed_arm_reference_filters(side)
            filtered = device._filtered_segment_directions[side]
            self.assertAlmostEqual(np.degrees(np.arccos(np.dot(
                filtered["upper_arm"], filtered["forearm"]
            ))), 0, places=3)
            device._last_body_frame_packet_time_monotonic = 10.02
            result = device._filter_segment_direction(side, "forearm", [0, 1, 0])
            self.assertGreater(np.linalg.norm(result - [0, 1, 0]), 0.1)

    @staticmethod
    def _direction_filter_device() -> BodyDevice:
        device = BodyDevice.__new__(BodyDevice)
        device._cfg = build_config().retarget
        device._filtered_segment_directions = {"left": {}, "right": {}}
        device._filtered_segment_timestamps = {"left": {}, "right": {}}
        return device

    def test_segment_filter_updates_only_once_per_pico_frame(self):
        device = self._direction_filter_device()
        device._last_body_frame_packet_time_monotonic = 10.0
        first = device._filter_segment_direction("right", "forearm", [1.0, 0.0, 0.0])
        device._last_body_frame_packet_time_monotonic = 10.02
        second = device._filter_segment_direction("right", "forearm", [0.0, 1.0, 0.0])
        repeated = device._filter_segment_direction("right", "forearm", [0.0, 0.0, 1.0])

        np.testing.assert_allclose(first, [1.0, 0.0, 0.0])
        self.assertGreater(second[0], 0.0)
        self.assertGreater(second[1], 0.0)
        np.testing.assert_allclose(repeated, second)

    def test_segment_filter_opens_bandwidth_for_deliberate_motion(self):
        fixed = self._direction_filter_device()
        adaptive = self._direction_filter_device()
        fixed._cfg.arm_vector_filter_speed_coefficient = 0.0
        angle = np.deg2rad(20.0)
        moved = np.asarray([np.cos(angle), np.sin(angle), 0.0])
        for device in (fixed, adaptive):
            device._last_body_frame_packet_time_monotonic = 20.0
            device._filter_segment_direction("left", "upper_arm", [1.0, 0.0, 0.0])
            device._last_body_frame_packet_time_monotonic = 20.02

        fixed_result = fixed._filter_segment_direction("left", "upper_arm", moved)
        adaptive_result = adaptive._filter_segment_direction("left", "upper_arm", moved)

        self.assertGreater(np.arctan2(adaptive_result[1], adaptive_result[0]),
                           np.arctan2(fixed_result[1], fixed_result[0]))

    def test_prediction_soft_gate_rejects_jitter_and_preserves_motion(self):
        device = self._direction_filter_device()

        self.assertEqual(device._prediction_velocity_gain(0.04), 0.0)
        self.assertEqual(device._prediction_velocity_gain(0.30), 1.0)
        middle = device._prediction_velocity_gain(0.155)
        self.assertGreater(middle, 0.0)
        self.assertLess(middle, 1.0)

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

    def test_reference_deviation_uses_each_current_segment(self):
        device = BodyDevice.__new__(BodyDevice)
        device._cfg = build_config().retarget
        device._signs = np.ones(3, dtype=np.float32)
        device._reference_left_points = {
            "shoulder": np.asarray([0.0, 0.0, 0.0]),
            "elbow": np.asarray([0.0, 0.0, -0.3]),
            "wrist": np.asarray([0.0, 0.0, -0.6]),
        }
        device._reference_right_points = None
        upper_rotation = mu.rotvec_to_rotation_matrix([np.deg2rad(5.0), 0.0, 0.0])
        forearm_rotation = mu.rotvec_to_rotation_matrix([np.deg2rad(12.0), 0.0, 0.0])
        upper = upper_rotation @ np.asarray([0.0, 0.0, -0.3])
        forearm = forearm_rotation @ np.asarray([0.0, 0.0, -0.3])
        device._current_arm_points = mock.Mock(
            return_value={
                "shoulder": np.zeros(3),
                "elbow": upper,
                "wrist": upper + forearm,
            }
        )

        upper_delta, forearm_delta = device.arm_reference_deviation_deg("left")

        self.assertAlmostEqual(upper_delta, 5.0, places=3)
        self.assertAlmostEqual(forearm_delta, 12.0, places=3)

    def test_stable_recent_frames_validate_without_replacing_reference(self):
        device = BodyDevice.__new__(BodyDevice)
        device._cfg = build_config().retarget
        device._cfg.auto_start_reference_require_waist = False
        device._reference_left_points = {
            "shoulder": np.asarray([0.0, 0.0, 0.0]),
            "elbow": np.asarray([0.0, 0.0, -0.3]),
            "wrist": np.asarray([0.0, 0.0, -0.6]),
        }
        device._reference_right_points = None
        device._held_segment_directions = {"left": {}, "right": {}}
        device._filtered_segment_directions = {"left": {}, "right": {}}
        device._filtered_segment_timestamps = {"left": {}, "right": {}}
        rotation = mu.rotvec_to_rotation_matrix([np.deg2rad(14.0), 0.0, 0.0])
        upper = rotation @ np.asarray([0.0, 0.0, -0.3])
        forearm = rotation @ np.asarray([0.0, 0.0, -0.3])

        def pose(position):
            return np.concatenate((position, [1.0, 0.0, 0.0, 0.0])).astype(np.float32)

        frames = {
            "left_shoulder": pose(np.zeros(3)),
            "left_elbow": pose(upper),
            "left_wrist": pose(upper + forearm),
        }
        device._body_frame_history = [
            (10.0 + index * 0.02, {key: value.copy() for key, value in frames.items()})
            for index in range(20)
        ]

        with mock.patch("esrobo_teleop.device.body_device.time.monotonic", return_value=10.4):
            result = device.refresh_arm_reference_for_enable("left")

        self.assertTrue(result[0])
        self.assertAlmostEqual(result[1], 14.0, places=3)
        self.assertAlmostEqual(result[2], 14.0, places=3)
        np.testing.assert_allclose(device._reference_left_points["elbow"], [0, 0, -.3], atol=1.0e-6)
        self.assertEqual(len(device._body_frame_history), 20)

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
        device._signs = np.ones(3, dtype=np.float32)
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
        with mock.patch("esrobo_teleop.robot.nero_driver.NeroArm"):
            self.driver = NeroSingleArmDriver(build_config().robot, "left")
        self.driver.configure_command_trajectory(lambda q: np.zeros((2, 3)), .22)
        self.driver._side = "left"
        self.driver._session_start = None
        self.driver._last_command_feedback_timestamp = 10.0
        self.driver._command_velocity = {"left": np.zeros(7)}
        self.driver._arm = mock.Mock(last_feedback_timestamp=10.02)
        self.driver._arm.controller_fault.return_value = None
        self.driver._arm.get_joint_enable_states.return_value = [True]*7
        self.driver.configure_collision_guard(lambda a, b: True)
        self.driver._torque_baseline = {}
        self.driver._torque_trip_counts = {"left": np.zeros(7, dtype=int)}
        self.driver._last_valid_torque_time = {"left": None}
        self.driver._safety_fault_reason = None

    def test_torque_monitor_trips_only_after_consecutive_excess(self):
        self.driver._cfg.torque_monitor_enabled = True
        self.driver._cfg.torque_deviation_limits_nm = [1.0] * 7
        self.driver._cfg.left_torque_deviation_limits_nm = [1.0] * 7
        self.driver._cfg.torque_trip_consecutive_samples = 3
        self.driver._torque_baseline["left"] = np.zeros(7)
        self.driver._last_valid_torque_time["left"] = time.monotonic()
        self.driver._arm.get_joint_torques.side_effect = [
            np.asarray([2.0, 0, 0, 0, 0, 0, 0]),
            np.asarray([2.0, 0, 0, 0, 0, 0, 0]),
            np.asarray([2.0, 0, 0, 0, 0, 0, 0]),
        ]

        self.assertTrue(self.driver._torque_is_safe(self.driver._arm, "left"))
        self.assertTrue(self.driver._torque_is_safe(self.driver._arm, "left"))
        self.assertFalse(self.driver._torque_is_safe(self.driver._arm, "left"))
        self.assertIn("J1: current=", self.driver.safety_fault_reason)
        self.assertIn("delta=", self.driver.safety_fault_reason)

    def test_torque_monitor_tolerates_short_feedback_gap(self):
        self.driver._cfg.torque_monitor_enabled = True
        self.driver._cfg.torque_feedback_timeout_s = 0.3
        self.driver._torque_baseline["left"] = np.zeros(7)
        self.driver._last_valid_torque_time["left"] = time.monotonic()
        self.driver._arm.get_joint_torques.return_value = None

        self.assertTrue(self.driver._torque_is_safe(self.driver._arm, "left"))

    def test_torque_baseline_waits_for_enabled_feedback_to_settle(self):
        self.driver._cfg.torque_monitor_enabled = True
        self.driver._cfg.torque_baseline_settle_s = 0.25
        self.driver._cfg.torque_baseline_samples = 3
        events = []
        self.driver._arm.get_joint_torques.side_effect = lambda: (
            events.append("read") or np.arange(7, dtype=float)
        )

        with mock.patch(
            "esrobo_teleop.robot.nero_driver.time.sleep",
            side_effect=lambda duration: events.append(("sleep", duration)),
        ):
            captured = self.driver._capture_arm_torque_baseline(self.driver._arm, "left")

        self.assertTrue(captured)
        self.assertEqual(events[0], ("sleep", 0.25))
        self.assertEqual(events.count("read"), 3)
        np.testing.assert_allclose(self.driver._torque_baseline["left"], np.arange(7))

    def test_urdf_zero_matches_natural_down_sdk_feedback(self):
        _, offsets = self.driver._mapping("left")
        self.driver.read_joints = mock.Mock(return_value=offsets.copy())
        np.testing.assert_allclose(self.driver.startup_zero_error(), np.zeros(7))
        full = self.driver.read_full_urdf_joints()
        np.testing.assert_allclose(full, np.zeros(14))

    def test_startup_alignment_uses_explicit_safe_recovery(self):
        self.driver.safe_return = mock.Mock(return_value=ReturnResult(True, True, "disabling"))
        self.assertEqual(self.driver.align_startup_zero_and_disable(), (True, True))
        self.driver.safe_return.assert_called_once_with(recover=True)

    def test_return_to_zero_uses_coordinated_checked_path(self):
        directions, zero = self.driver._mapping("left")
        self.driver._cfg.return_verify_samples = 1
        initial_urdf = np.asarray([0.1, -0.2, 0.3, 0.4, -0.3, 0.2, -0.1])
        self.driver._enabled = True
        feedback = directions * initial_urdf + zero
        stamp = 10.0
        def read():
            nonlocal stamp
            stamp += .02
            self.driver._arm.last_feedback_timestamp = stamp
            self.driver._return_feedback_received = time.monotonic()
            return feedback.copy()
        def send(q):
            nonlocal feedback
            feedback = np.asarray(q).copy()
        self.driver.read_joints = mock.Mock(side_effect=read)
        self.driver._arm.move_j.side_effect = send
        self.driver._arm.disable.return_value = True
        with mock.patch("esrobo_teleop.robot.nero_driver.time.sleep"):
            self.assertEqual(self.driver.return_to_zero_and_disable(), (True, True))
        commands = np.array([c.args[0] for c in self.driver._arm.move_j.call_args_list])
        urdf = (commands-zero)/directions
        tolerance = math.radians(self.driver._cfg.full_arm_return_tolerance_deg)
        proximal_moved = np.flatnonzero(np.max(abs(urdf[:, :3]-initial_urdf[:3]), axis=1) > 1e-8)
        self.assertTrue(len(proximal_moved))
        self.assertGreater(np.max(abs(urdf[proximal_moved[0], 3:])), tolerance)
        self.assertLessEqual(np.max(abs(urdf[-1])), tolerance)
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
        samples = iter([(None, None), (zero.copy(), 10.04), (zero.copy(), 10.06), (zero.copy(), 10.08)])

        def read_sample():
            joints, stamp = next(samples)
            self.driver._arm.last_feedback_timestamp = stamp
            self.driver._return_feedback_received = time.monotonic()
            return joints

        self.driver.read_joints = mock.Mock(side_effect=read_sample)

        returned = self.driver._return_group_to_zero(
            np.zeros(14), np.arange(7), zero, math.radians(1.0), 1.0
        )

        self.assertTrue(returned)
        self.assertEqual(self.driver.read_joints.call_count, 4)

    def test_every_waypoint_verifies_all_seven_joints(self):
        _, zero = self.driver._mapping("left")
        self.driver._enabled = True
        self.driver.read_joints = mock.Mock(return_value=zero.copy())
        self.driver._return_stationary = mock.Mock(return_value=zero.copy())
        self.driver._return_group_to_zero = mock.Mock(return_value=True)
        self.driver._arm.disable.return_value = True
        self.assertEqual(self.driver.return_to_zero_and_disable(), (True, True))
        np.testing.assert_array_equal(self.driver._return_group_to_zero.call_args.args[1], np.arange(7))

    def test_return_never_retries_failed_waypoint(self):
        _, zero = self.driver._mapping("left")
        self.driver._enabled = True
        self.driver.read_joints = mock.Mock(return_value=zero.copy())
        self.driver._return_stationary = mock.Mock(return_value=zero.copy())
        self.driver._return_group_to_zero = mock.Mock(side_effect=[False, True])
        self.assertEqual(self.driver.return_to_zero_and_disable(), (False, False))
        self.assertEqual(self.driver._return_group_to_zero.call_count, 1)
        self.driver._arm.disable.assert_not_called()

    def test_return_phase_uses_its_own_feedback_clock(self):
        directions, zero = self.driver._mapping("left")
        self.driver._cfg.return_verify_samples = 1
        self.driver._last_command_feedback_timestamp = 999.0
        feedback = directions * 0.05 + zero
        stamp = 9.98

        def read_sample():
            nonlocal stamp
            stamp += 0.02
            self.driver._arm.last_feedback_timestamp = stamp
            self.driver._return_feedback_received = time.monotonic()
            return feedback.copy()

        def accept_command(values):
            nonlocal feedback
            feedback = np.asarray(values).copy()

        self.driver.read_joints = mock.Mock(side_effect=read_sample)
        self.driver._arm.move_j.side_effect = accept_command
        returned = self.driver._return_group_to_zero(
            np.zeros(14), np.arange(7), zero, math.radians(1.0), 2.0
        )
        self.assertTrue(returned)
        commands = [np.asarray(call.args[0]) for call in self.driver._arm.move_j.call_args_list]
        np.testing.assert_allclose(commands[0], directions * 0.05 + zero)
        # The return starts from its own zero-velocity state at dt=20 ms;
        # neither stale session time nor measured-pose reanchoring may jump it.
        acceleration = self.driver._joint_rate_limit_vector("left", "max_joint_acceleration")
        self.assertTrue(np.all(abs(commands[1]-commands[0]) <= acceleration*.02**2 + 1e-9))
        self.assertLess(np.max(abs(commands[-1]-zero)), math.radians(1.0))

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
        self.driver._arm.enable_at_known_target.side_effect = [
            (zero.copy(), [True] * 7),
            (zero.copy(), [True] * 7),
        ]
        self.driver._arm.disable.return_value = True
        self.driver._arm.configure_collision_protection.return_value = True
        self.driver._capture_arm_torque_baseline = mock.Mock(return_value=True)

        self.assertTrue(self.driver.enable_from_natural_down())

        self.assertEqual(self.driver._arm.enable_at_known_target.call_count, 2)
        for call in self.driver._arm.enable_at_known_target.call_args_list:
            np.testing.assert_allclose(call.args[0], zero)
        self.driver._arm.disable.assert_called_once_with()
        self.driver._arm.request_can_feedback_push.assert_called_once_with()
        self.driver._arm.configure_collision_protection.assert_called_once_with()
        self.driver._arm.best_effort_disable_once.assert_not_called()
        self.driver._arm.set_speed_percent.assert_called_once_with(
            self.driver._cfg.speed_percent
        )
        self.assertFalse(self.driver.silent_disabled_startup)

    def test_silent_left_collision_configuration_failure_stays_disabled(self):
        _, zero = self.driver._mapping("left")
        self.driver._silent_disabled_startup = True
        self.driver._arm.enable_at_known_target.return_value = (zero.copy(), [True] * 7)
        self.driver._arm.disable.return_value = True
        self.driver._arm.configure_collision_protection.return_value = False

        self.assertFalse(self.driver.enable_from_natural_down())

        self.driver._arm.best_effort_disable_once.assert_called_once_with()
        self.assertTrue(self.driver.silent_disabled_startup)
        self.assertFalse(self.driver._enabled)

    def test_silent_left_enable_rejects_missing_feedback_and_disables_once(self):
        self.driver._silent_disabled_startup = True
        self.driver._arm.enable_at_known_target.return_value = (None, None)

        self.assertFalse(self.driver.enable_from_natural_down())

        self.driver._arm.best_effort_disable_once.assert_called_once_with()
        self.assertTrue(self.driver.silent_disabled_startup)

    def test_command_sends_only_the_configured_arm(self):
        # Legacy fallback remains covered; both continuous paths have real-FK tests.
        self.driver._cfg.left_command_trajectory_enabled = False
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
    def setUp(self):
        self.prepare = mock.patch.object(TeleopNode, "_wait_for_arm_prepare", return_value=True)
        self.prepare.start()
        self.addCleanup(self.prepare.stop)

    def test_linked_enable_requires_fresh_finger_targets_before_arm(self):
        node = TeleopNode.__new__(TeleopNode)
        node._arm_with_hand = True
        node._arm_side = "left"
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
        node._arm_side = "left"
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

    def test_linked_enable_latches_fault_if_hand_feedback_disappears(self):
        node = TeleopNode.__new__(TeleopNode)
        node._arm_with_hand = True
        node._arm_side = "right"
        node._arm_armed = False
        node._hand_enabled = False
        node._body = mock.Mock()
        node._body.hand_joints.return_value = np.zeros(20)
        node._hand = mock.Mock()
        node._hand.feedback_ready.return_value = True
        node._hand.set_enabled.return_value = False
        node._arm_robot = mock.Mock(return_value=True)
        node._stop_full_arm_for_fault = mock.Mock(return_value=(True, True))

        node._handle_key("e")

        node._stop_full_arm_for_fault.assert_called_once_with(
            "right-hand feedback disappeared during linked enable"
        )
        self.assertFalse(node._hand_enabled)

    def test_right_linked_enable_reports_the_active_side(self):
        node = TeleopNode.__new__(TeleopNode)
        node._arm_with_hand = True
        node._arm_side = "right"
        node._arm_armed = False
        node._hand_enabled = False
        node._body = mock.Mock()
        node._body.hand_joints.return_value = np.zeros(20)
        node._hand = mock.Mock()
        node._hand.feedback_ready.return_value = True
        node._hand.set_enabled.return_value = True
        node._arm_robot = mock.Mock(return_value=True)
        output = io.StringIO()

        with contextlib.redirect_stdout(output):
            node._handle_key("e")

        self.assertTrue(node._arm_armed)
        self.assertTrue(node._hand_enabled)
        self.assertIn("RIGHT ARM + RIGHT HAND LINKED FOLLOWING ENABLED", output.getvalue())

    def test_right_linked_stop_pauses_hand_without_open_motion(self):
        node = TeleopNode.__new__(TeleopNode)
        node._arm_with_hand = True
        node._arm_only = True
        node._arm_side = "right"
        node._wrist_imu_side = None
        node._arm_armed = True
        node._hand_enabled = True
        node._driver = mock.Mock()
        node._driver.safe_return.return_value = ReturnResult(True, True, "disabling")
        node._hand = mock.Mock()
        node._handle_key("s")
        node._return_thread.join(1.)
        node._hand.set_enabled.assert_called_once_with(False)
        node._driver.safe_return.assert_called_once()
        node._hand.return_to_open.assert_not_called()
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

    def test_arm_only_startup_defers_nonzero_alignment_to_e_without_input(self):
        node = TeleopNode.__new__(TeleopNode)
        node._cfg = build_config()
        node._wrist_imu_side = None
        node._arm_only = True
        node._arm_with_hand = False
        node._arm_side = "left"
        node._motors_enabled = False
        node._startup_alignment_required = False
        node._driver = mock.Mock()
        node._driver.startup_zero_error.return_value = np.deg2rad(
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -10.0]
        )
        node._key_thread = mock.Mock()
        node._driver.align_startup_zero_and_disable.return_value = (True, True)

        with mock.patch("builtins.input") as prompt, mock.patch(
                "sys.stdin.isatty", return_value=True):
            node._ensure_startup_zero_pose()

        prompt.assert_not_called()
        node._driver.align_startup_zero_and_disable.assert_not_called()
        self.assertTrue(node._startup_alignment_required)
        self.assertFalse(node._motors_enabled)

    def test_pending_startup_alignment_rejects_unverified_disable(self):
        node = TeleopNode.__new__(TeleopNode)
        node._cfg = build_config()
        node._arm_side = "left"
        node._motors_enabled = False
        node._manual_intervention_required = False
        node._startup_alignment = False
        node._startup_alignment_required = True
        node._driver = mock.Mock()
        node._driver.startup_zero_error.return_value = np.deg2rad(
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -10.0]
        )
        node._driver.align_startup_zero_and_disable.return_value = (True, False)

        self.assertFalse(node._align_pending_startup_zero())

        self.assertTrue(node._motors_enabled)
        self.assertTrue(node._manual_intervention_required)
        self.assertTrue(node._startup_alignment_required)

    def test_single_e_starts_pending_alignment_worker_without_enter(self):
        node = TeleopNode.__new__(TeleopNode)
        node._arm_only = True
        node._arm_with_hand = False
        node._startup_alignment_required = True
        node._startup_alignment = False
        node._return_thread = None
        node._arming_thread = None
        node._arming_cancel = threading.Event()
        node._arm_probe = None
        node._arm_armed = False
        node._arm_robot = mock.Mock(return_value=True)

        with mock.patch("builtins.input") as prompt:
            node._handle_key("e")
            node._arming_thread.join(1.0)

        prompt.assert_not_called()
        node._arm_robot.assert_called_once_with()
        self.assertTrue(node._arm_armed)

    def test_single_arm_e_always_uses_worker_even_when_zero_alignment_is_not_needed(self):
        node = TeleopNode.__new__(TeleopNode)
        node._arm_only = True
        node._arm_with_hand = False
        node._wrist_imu_side = None
        node._startup_alignment_required = False
        node._startup_alignment = False
        node._return_thread = None
        node._arming_thread = None
        node._arming_cancel = threading.Event()
        node._arm_control_lock = threading.RLock()
        node._arm_probe = None
        node._arm_armed = False
        caller_threads = []

        def arm():
            caller_threads.append(threading.current_thread().name)
            return True

        node._arm_robot = mock.Mock(side_effect=arm)
        node._handle_key("e")
        node._arming_thread.join(1.0)

        self.assertEqual(caller_threads, ["startup-arm-enable"])
        self.assertTrue(node._arm_armed)

    def test_startup_worker_waits_for_control_state_lock_before_reinitializing_ik(self):
        node = TeleopNode.__new__(TeleopNode)
        node._arm_only = True
        node._arm_with_hand = False
        node._wrist_imu_side = None
        node._startup_alignment_required = False
        node._startup_alignment = False
        node._return_thread = None
        node._arming_thread = None
        node._arming_cancel = threading.Event()
        node._arm_control_lock = threading.RLock()
        node._arm_probe = None
        node._arm_armed = False
        entered = threading.Event()

        def arm():
            entered.set()
            return True

        node._arm_robot = mock.Mock(side_effect=arm)
        with node._arm_control_lock:
            node._handle_key("e")
            self.assertFalse(entered.wait(0.05))
        self.assertTrue(entered.wait(1.0))
        node._arming_thread.join(1.0)

        self.assertTrue(node._arm_armed)

    def test_control_loop_does_not_advance_retarget_or_ik_while_arming(self):
        node = TeleopNode.__new__(TeleopNode)
        node._cfg = build_config()
        node._stop = False
        node._arm_only = True
        node._arm_with_hand = False
        node._arm_side = "right"
        node._arm_armed = False
        node._hand_only = False
        node._hand = None
        node._hand_enabled = False
        node._return_hand_open_on_close = False
        node._wrist_imu_side = None
        node._motors_enabled = False
        node._driver = mock.Mock()
        node._body = mock.Mock()
        node._ik = mock.Mock()
        worker = mock.Mock()

        def active_once():
            node._stop = True
            return True

        worker.is_alive.side_effect = active_once
        node._arming_thread = worker

        with contextlib.redirect_stdout(io.StringIO()):
            node.run()

        node._body.advance.assert_not_called()
        node._ik.solve.assert_not_called()

    def test_startup_cannot_rebase_between_preview_acquisition_and_cycle_end(self):
        # The 1789913277583056193 incident used pre-enable target/feedback
        # after the startup worker had installed a new measured reference.
        # Exercise actual thread interleaving, not just an already-live worker.
        for trigger in ("feedback", "diagnostics"):
            with self.subTest(trigger=trigger):
                node = TeleopNode.__new__(TeleopNode)
                node._cfg = build_config()
                node._stop = False
                node._arm_only = True
                node._arm_with_hand = False
                node._arm_side = "right"
                node._arm_armed = False
                node._hand_only = False
                node._hand = None
                node._wrist_imu_side = None
                node._motors_enabled = False
                node._arm_control_lock = threading.RLock()
                node._arming_thread = None
                node._arming_cancel = threading.Event()
                node._driver = mock.Mock()
                node._body = mock.Mock()
                node._body.hand_orientation_delta.return_value = None
                node._body._last_body_frame_packet_time_monotonic = None
                poses = {name: np.array([0., 0., 0., 1., 0., 0., 0.])
                         for name in ("left_elbow", "left_wrist", "right_elbow", "right_wrist")}
                node._body.advance.return_value = poses
                node._ik = mock.Mock(last_solution_valid=True)
                node._ik.solve.return_value = np.zeros(14)
                node.close = mock.Mock()
                node._stop_full_arm_for_fault = mock.Mock()
                rebased = threading.Event()

                def initialize():
                    rebased.set()
                    return True

                node._arm_robot = mock.Mock(side_effect=initialize)

                def start_worker():
                    node._start_pending_startup_arming()
                    # Without the whole-cycle lock the worker completes here,
                    # then the old local targets can enter the newly armed IK.
                    self.assertFalse(rebased.wait(0.1))

                def read_feedback():
                    if trigger == "feedback":
                        start_worker()
                        node._stop = True
                    return np.zeros(14)

                def publish(*_args):
                    if trigger == "diagnostics":
                        start_worker()
                    node._stop = True

                node._read_current_joints = mock.Mock(side_effect=read_feedback)
                node._publish_arm_diagnostics = mock.Mock(side_effect=publish)
                try:
                    with contextlib.redirect_stdout(io.StringIO()):
                        node.run()
                finally:
                    if node._arming_thread is not None:
                        node._arming_thread.join(1.0)
                self.assertTrue(rebased.is_set())
                node._driver.command_full_urdf.assert_not_called()
                node._stop_full_arm_for_fault.assert_not_called()

    def test_pending_alignment_is_between_pico_checks_and_normal_enable(self):
        node = TeleopNode.__new__(TeleopNode)
        node._cfg = build_config()
        node._wrist_imu_side = None
        node._arm_only = True
        node._arm_side = "right"
        node._arm_armed = False
        node._motors_enabled = False
        node._startup_alignment_required = True
        node._driver = mock.Mock()
        node._driver.capture_session_start.return_value = True
        node._driver.read_full_urdf_joints.return_value = np.zeros(14)
        node._driver.enable.return_value = True
        node._body = mock.Mock()
        node._body.is_ready.return_value = True
        node._body.refresh_arm_reference_for_enable.side_effect = [
            (True, 0.0, 0.0, 0.002, 20),
            (True, 0.2, 0.3, 0.002, 20),
            (True, 0.3, 0.4, 0.003, 20),
        ]
        node._body.hand_orientation_delta.return_value = None
        node._ik = mock.Mock()
        node._ik.current_task_frame_poses.return_value = {}
        node._ik.wrist_orientation_linearization.return_value = (
            np.eye(3), np.eye(3)
        )
        node._full_arm_wrist_local_axes = None

        def align():
            node._startup_alignment_required = False
            return True

        node._align_pending_startup_zero = mock.Mock(side_effect=align)

        self.assertTrue(node._arm_robot())

        node._align_pending_startup_zero.assert_called_once_with()
        self.assertEqual(node._body.refresh_arm_reference_for_enable.call_count, 3)
        node._driver.enable.assert_called_once_with()
        self.assertTrue(node._motors_enabled)

    def test_s_cancels_pending_startup_worker_and_disables_enable_race(self):
        node = TeleopNode.__new__(TeleopNode)
        node._arm_only = True
        node._arm_with_hand = False
        node._arm_side = "right"
        node._startup_alignment_required = True
        node._startup_alignment = False
        node._return_thread = None
        node._arming_thread = None
        node._arming_cancel = threading.Event()
        node._arm_probe = None
        node._arm_armed = False
        node._motors_enabled = False
        node._driver = mock.Mock()
        node._driver.disable.return_value = True
        entered = threading.Event()
        release = threading.Event()

        def arm():
            entered.set()
            release.wait(1.0)
            return True

        node._arm_robot = mock.Mock(side_effect=arm)
        node._handle_key("e")
        self.assertTrue(entered.wait(1.0))
        node._handle_key("s")
        release.set()
        node._arming_thread.join(1.0)

        node._driver.cancel_return.assert_called_once_with()
        node._driver.disable.assert_called_once_with()
        self.assertFalse(node._arm_armed)
        self.assertFalse(node._motors_enabled)

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
        node._body.refresh_arm_reference_for_enable.side_effect = [
            (True, 0.0, 0.0, 0.002, 20),
            (True, 0.5, 0.8, 0.003, 20),
        ]
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
        node._body.refresh_arm_reference_for_enable.return_value = (
            True, 0.0, 0.0, 0.002, 20
        )
        node._ik = mock.Mock()

        self.assertFalse(node._arm_robot())

        node._driver.disable.assert_called_once_with()
        self.assertFalse(node._motors_enabled)

    def test_arm_only_arming_refuses_invalid_latest_target_before_driver(self):
        node = TeleopNode.__new__(TeleopNode)
        node._cfg = build_config()
        node._wrist_imu_side = None
        node._arm_only = True
        node._arm_side = "right"
        node._motors_enabled = False
        node._driver = mock.Mock()
        node._body = mock.Mock()
        node._body.is_ready.return_value = True
        node._body.refresh_arm_reference_for_enable.return_value = (
            False, 4.0, 34.0, 0.006, 20
        )
        output = io.StringIO()

        with contextlib.redirect_stdout(output):
            armed = node._arm_robot()

        self.assertFalse(armed)
        node._driver.capture_session_start.assert_not_called()
        node._driver.enable.assert_not_called()
        node._driver.enable_from_natural_down.assert_not_called()
        self.assertIn("no motor-enable or position command was sent", output.getvalue())

    def test_arm_only_arming_rechecks_pose_after_torque_settling(self):
        node = TeleopNode.__new__(TeleopNode)
        node._cfg = build_config()
        node._wrist_imu_side = None
        node._arm_only = True
        node._arm_side = "right"
        node._motors_enabled = False
        node._driver = mock.Mock()
        node._driver.capture_session_start.return_value = True
        node._driver.read_full_urdf_joints.return_value = np.zeros(14)
        node._driver.enable.return_value = True
        node._body = mock.Mock()
        node._body.is_ready.return_value = True
        node._body.refresh_arm_reference_for_enable.side_effect = [
            (True, 0.0, 0.0, 0.002, 20),
            (False, 2.0, 38.0, 0.006, 20),
        ]
        node._body.hand_orientation_delta.return_value = None
        node._ik = mock.Mock()
        node._ik.current_task_frame_poses.return_value = {}
        node._ik.wrist_orientation_linearization.return_value = (
            np.eye(3), np.eye(3)
        )
        node._full_arm_wrist_local_axes = None
        output = io.StringIO()

        with contextlib.redirect_stdout(output):
            armed = node._arm_robot()

        self.assertFalse(armed)
        self.assertTrue(node._motors_enabled)
        node._driver.enable.assert_called_once_with()
        node._body.rebase_robot_reference.assert_called_once_with({})
        self.assertIn("Following remains locked", output.getvalue())

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
        node._hand = None
        node._driver = mock.Mock()
        node._driver.safe_return.return_value = ReturnResult(False, False, "planning", "no path")
        node._handle_key("q")
        node._return_thread.join(1.)
        self.assertFalse(node._stop)
        self.assertFalse(node._arm_armed)
        self.assertTrue(node._manual_intervention_required)
        node._driver.safe_return.assert_called_once()

    def test_fault_latched_q_runs_checked_recovery_then_exits(self):
        node = TeleopNode.__new__(TeleopNode)
        node._stop = False
        node._arm_armed = False
        node._motors_enabled = True
        # The driver latch is authoritative even if the UI-side mirror has not
        # been updated yet by the control loop.
        node._manual_intervention_required = False
        node._arm_only = True
        node._arm_side = "right"
        node._arm_with_hand = False
        node._hand = None
        node._driver = mock.Mock(safety_fault_reason="IK geometry rejected")
        node._driver.prepare_return.return_value = 41
        node._driver.safe_return.return_value = ReturnResult(
            True, True, "disabling"
        )

        with mock.patch("builtins.print") as output:
            node._handle_key("q")
            node._return_thread.join(1.)

        self.assertTrue(node._stop)
        self.assertFalse(node._motors_enabled)
        self.assertFalse(node._manual_intervention_required)
        node._driver.safe_return.assert_called_once_with(recover=True, request_id=41)
        self.assertTrue(any(
            "EXIT REQUEST" in call.args[0] for call in output.call_args_list
        ))

    def test_q_during_active_safe_return_promotes_it_to_exit(self):
        entered = threading.Event()
        release = threading.Event()
        node = TeleopNode.__new__(TeleopNode)
        node._stop = False
        node._arm_armed = True
        node._motors_enabled = True
        node._manual_intervention_required = False
        node._arm_only = True
        node._arm_side = "right"
        node._arm_with_hand = False
        node._startup_alignment = False
        node._arming_thread = None
        node._hand = None
        node._driver = mock.Mock(safety_fault_reason=None)
        node._driver.prepare_return.return_value = 42

        def safe_return(**_kwargs):
            entered.set()
            self.assertTrue(release.wait(1.))
            return ReturnResult(True, True, "disabling")

        node._driver.safe_return.side_effect = safe_return
        node._start_full_arm_return("operator pressed 's'")
        self.assertTrue(entered.wait(1.))

        with mock.patch("builtins.print") as output:
            node._handle_key("q")
        node._driver.cancel_return.assert_not_called()
        self.assertTrue(node._quit_after_return)
        self.assertTrue(any(
            "pending exit" in call.args[0] for call in output.call_args_list
        ))

        release.set()
        node._return_thread.join(1.)
        self.assertFalse(node._return_thread.is_alive())
        self.assertTrue(node._stop)

    def test_fault_latched_s_reports_following_already_stopped(self):
        node = TeleopNode.__new__(TeleopNode)
        node._stop = False
        node._arm_armed = False
        node._motors_enabled = True
        node._manual_intervention_required = True
        node._arm_only = True
        node._arm_side = "right"
        node._arm_with_hand = False
        node._driver = mock.Mock(safety_fault_reason="IK geometry rejected")

        with mock.patch("builtins.print") as output:
            node._handle_key("s")

        node._driver.prepare_return.assert_not_called()
        self.assertIn("STOP ALREADY ACTIVE", output.call_args.args[0])

    def test_ctrl_c_keeps_fault_latched_enabled_arm_connected_until_disable(self):
        node = mock.Mock()
        node.run.side_effect = [KeyboardInterrupt(), None]
        node._enabled_fault_requires_explicit_disable.side_effect = [True, False]
        node._stop = True

        _run_until_safe_exit(node)

        self.assertEqual(node.run.call_count, 2)
        self.assertFalse(node._stop)
        node._report_enabled_fault_exit_refusal.assert_called_once_with(
            "INTERRUPT EXIT"
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
        self.assertIsNone(arm._selected_motion_mode)
        arm._robot.set_motion_mode.reset_mock()
        arm.move_j(list(cfg.left_joint_offsets))
        arm._robot.set_motion_mode.assert_called_once_with("j-mode")

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
        self.assertIsNone(arm._selected_motion_mode)

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
    def test_calibrated_microbend_survives_preview_and_rate_limited_start(self):
        cfg = build_config()
        cfg.ik.urdf_path = os.path.abspath(os.path.join(
            os.path.dirname(__file__), "..", "urdf", "esrobo_waist_with_head.urdf"
        ))
        for side, offset in (("left", 0), ("right", 7)):
            with self.subTest(side=side):
                solver = IkSolver(cfg.ik)
                current = np.zeros(14)
                solver._prev_targets = np.full(14, 0.7)
                solver.initialize_arm_session(side, current, np.deg2rad(18))
                baseline = solver._j3_flexion_baseline[side]
                self.assertAlmostEqual(np.rad2deg(baseline), 18, delta=0.1)
                np.testing.assert_array_equal(solver._prev_targets[offset:offset + 7], 0)
                other = 7 if offset == 0 else 0
                np.testing.assert_array_equal(solver._prev_targets[other:other + 7], 0.7)
                for bend in (2, 18, 20):
                    desired = current.copy()
                    desired[offset + 2] = -0.5
                    desired[offset + 3] = np.deg2rad(bend)
                    poses = solver.current_task_frame_poses(desired)
                    result = solver.solve(
                        poses["left_wrist"], poses["right_wrist"], current,
                        poses["left_elbow"], poses["right_elbow"],
                        partitioned_side=side, terminal_joint_targets=np.zeros(3),
                    )
                    if bend <= cfg.ik.j3_observability_start_flexion_deg:
                        self.assertAlmostEqual(result[offset + 2], 0, places=6)
                    else:
                        self.assertAlmostEqual(result[offset + 2], -0.5, places=5)
                    self.assertEqual(solver._j3_flexion_baseline[side], baseline)
                desired[offset + 3] = np.deg2rad(40)
                poses = solver.current_task_frame_poses(desired)
                result = solver.solve(
                    poses["left_wrist"], poses["right_wrist"], current,
                    poses["left_elbow"], poses["right_elbow"],
                    partitioned_side=side, terminal_joint_targets=np.zeros(3),
                )
                self.assertLess(result[offset + 2], -0.1)
                solver.initialize_arm_session(side, current, np.deg2rad(12))
                self.assertAlmostEqual(np.rad2deg(solver._j3_flexion_baseline[side]), 12, delta=0.1)

    def test_ik_solve_failure_logging_reports_only_state_changes(self):
        solver = IkSolver.__new__(IkSolver)
        solver._solve_failure_active = False

        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            solver._report_solve_failure()
            solver._report_solve_failure()
            solver._report_solve_recovery()
            solver._report_solve_recovery()

        lines = output.getvalue().splitlines()
        self.assertEqual(len(lines), 2)
        self.assertIn("solve unavailable", lines[0])
        self.assertEqual(lines[1], "[ik] solve recovered.")

    def test_bounded_ik_iterations_converge_near_right_j2_upper_limit(self):
        cfg = build_config()
        cfg.ik.urdf_path = os.path.abspath(os.path.join(
            os.path.dirname(__file__), "..", "urdf", "esrobo_waist_with_head.urdf"
        ))
        cfg.ik.max_joint_position_delta = 0.0
        cfg.ik.max_command_lead = 0.0
        solver = IkSolver(cfg.ik)
        current = np.zeros(14)
        desired = current.copy()
        desired[7:11] = np.deg2rad([-34.8938, 0.6723, -54.4303, 63.2025])
        current_poses = solver.current_task_frame_poses(current)
        desired_poses = solver.current_task_frame_poses(desired)

        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            result = solver.solve(
                current_poses["left_wrist"],
                desired_poses["right_wrist"],
                current,
                left_elbow_pose=current_poses["left_elbow"],
                right_elbow_pose=desired_poses["right_elbow"],
                partitioned_side="right",
                terminal_joint_targets=np.zeros(3),
            )

        self.assertNotIn("solve unavailable", output.getvalue())
        np.testing.assert_allclose(result[7:11], desired[7:11], atol=np.deg2rad(0.2))

    def test_j3_gain_fades_in_with_elbow_bend_observability(self):
        raw = math.radians(20.0)
        anchor = math.radians(4.0)

        straight = IkSolver._stabilized_j3_target(raw, anchor, 0.0, 1.12, 8.0, 20.0)
        bent = IkSolver._stabilized_j3_target(
            raw, anchor, math.radians(30.0), 1.12, 8.0, 20.0
        )
        transition = IkSolver._stabilized_j3_target(
            raw, anchor, math.radians(14.0), 1.12, 8.0, 20.0
        )

        self.assertAlmostEqual(straight, anchor)
        self.assertAlmostEqual(bent, raw * 1.12)
        self.assertGreater(transition, anchor)
        self.assertLess(transition, raw * 1.12)

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

        self.assertLess(first_error, 5.0e-4)
        self.assertLess(second_error, 5.0e-4)
        np.testing.assert_allclose(first[:4], desired[:4], atol=np.deg2rad(0.05))
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
