import contextlib
import io
import json
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import numpy as np

from esrobo_teleop import math_utils as mu
from esrobo_teleop.config import build_config
from esrobo_teleop.device.body_device import BodyDevice
from esrobo_teleop.ik.solver import IkSolver
from esrobo_teleop.teleop_node import TeleopNode


class PicoReferenceTests(unittest.TestCase):
    def test_recorded_startup_old_cycle_is_rejected_but_fresh_cycle_is_solvable(self):
        fixture = json.loads((Path(__file__).parent / "fixtures" /
                              "right_startup_stale_cycle_20260920.json").read_text())
        cfg = build_config()
        cfg.ik.urdf_path = str(Path(__file__).resolve().parents[1] /
                               "urdf/esrobo_waist_with_head.urdf")
        solver = IkSolver(cfg.ik)
        startup = np.zeros(14)
        startup[7:] = (np.array(fixture["startup"]["feedback"]["physical_rad"])
                       - np.array(cfg.robot.right_joint_offsets))
        reference = np.deg2rad(fixture["human_reference_flexion_deg"])
        poses = solver.current_task_frame_poses(startup)
        stale = {key: value.copy() for key, value in poses.items()}
        for key, value in fixture["rejection"]["retarget_positions_m"].items():
            stale[key][:3] = value
        self.assertGreater(np.linalg.norm(stale["right_elbow"][:3] -
                                          poses["right_elbow"][:3]), .36)
        self.assertGreater(np.linalg.norm(stale["right_wrist"][:3] -
                                          poses["right_wrist"][:3]), .50)
        solver.initialize_arm_session("right", startup, reference,
                                      relative_elbow_reference=True)
        old_feedback = np.array(fixture["rejection"]["current_urdf_rad"])
        with contextlib.redirect_stdout(io.StringIO()):
            solver.solve(stale["left_wrist"], stale["right_wrist"], old_feedback,
                         stale["left_elbow"], stale["right_elbow"],
                         "right", old_feedback[11:])
        self.assertFalse(solver.last_solution_valid)
        self.assertGreater(solver.solution_diagnostics["elbow_error_m"], .03)

        solver.initialize_arm_session("right", startup, reference,
                                      relative_elbow_reference=True)
        for key, value in fixture["first_fresh_target"].items():
            poses[key][:3] = value
        with contextlib.redirect_stdout(io.StringIO()):
            solver.solve(poses["left_wrist"], poses["right_wrist"], startup,
                         poses["left_elbow"], poses["right_elbow"],
                         "right", startup[11:])
        self.assertTrue(solver.last_solution_valid, solver.solution_diagnostics)
        self.assertLess(solver.solution_diagnostics["elbow_error_m"], 1e-5)
        self.assertLess(solver.solution_diagnostics["wrist_error_m"], 1e-5)

    def test_recorded_bent_neutral_does_not_command_startup_motion(self):
        # Final accepted right-arm reference from the 2026-09-17 incident.
        recorded = {
            "shoulder": np.array([-.04502213, -.19498470, .41248012]),
            "elbow": np.array([-.07412843, -.23703083, .17411853]),
            "wrist": np.array([.03023469, -.26889294, -.04631182]),
        }
        cfg = build_config()
        cfg.ik.urdf_path = str(Path(__file__).resolve().parents[1] / "urdf/esrobo_waist_with_head.urdf")
        for side, offset in (("left", 0), ("right", 7)):
            with self.subTest(side=side):
                solver = IkSolver(cfg.ik)
                device = self.device(side)
                # This regression covers the retained legacy mapping mode.
                device._cfg.elbow_angle_mapping = "responsive_absolute"
                points = {k: v.copy() for k, v in recorded.items()}
                if side == "left":
                    for value in points.values():
                        value[1] *= -1
                setattr(device, f"_reference_{side}_points", points)
                current = np.zeros(14)
                current[offset:offset + 7] = np.deg2rad([1.17, -.28, .01, .01, -.06, 0, -.76])
                initial = current.copy()
                poses = solver.current_task_frame_poses(current)
                device.rebase_robot_reference(poses)
                device._last_body_frame_packet_time_monotonic = 10
                device.seed_arm_reference_filters(side)
                solver.initialize_arm_session(side, current, device.arm_reference_flexion_rad(side),
                                              relative_elbow_reference=True)
                elbow, wrist = device._retarget_arm_segment_direction_positions(points, side)
                np.testing.assert_allclose(elbow, poses[f"{side}_elbow"][:3], atol=2e-6)
                np.testing.assert_allclose(wrist, poses[f"{side}_wrist"][:3], atol=2e-6)
                targets = {k: v.copy() for k, v in poses.items()}
                targets[f"{side}_elbow"][:3] = elbow
                targets[f"{side}_wrist"][:3] = wrist
                for _ in range(30):
                    current = solver.solve(
                        targets["left_wrist"], targets["right_wrist"], current,
                        targets["left_elbow"], targets["right_elbow"],
                        partitioned_side=side, terminal_joint_targets=initial[offset + 4:offset + 7],
                    )
                self.assertLess(np.max(np.abs(np.degrees(current - initial))), .2)

    def test_elbow_mapping_removes_only_reference_bend(self):
        upper = np.array([0., 0., -1.])
        for azimuth in (0, .8, -1.2):
            for bend, expected in ((32, 0), (52, 20), (22, 0), (112, 80)):
                angle = np.deg2rad(bend)
                forearm = np.array([np.sin(angle) * np.cos(azimuth),
                                    np.sin(angle) * np.sin(azimuth), -np.cos(angle)])
                result = BodyDevice._neutral_relative_forearm(
                    upper, forearm, np.deg2rad(32), upper, upper)
                result_angle = np.degrees(np.arccos(np.clip(np.dot(upper, result), -1, 1)))
                self.assertAlmostEqual(result_angle, expected, places=3)
                if expected:
                    self.assertAlmostEqual(np.arctan2(result[1], result[0]), azimuth, places=5)

    def test_low_bend_blends_calibrated_and_live_planes_until_fully_observable(self):
        upper = np.array([0.0, 0.0, -1.0])
        robot_bend = np.deg2rad(5.0)
        robot_forearm = np.array([np.sin(robot_bend), 0.0, -np.cos(robot_bend)])
        reference = np.deg2rad(30.0)

        # A small mapped bend lies inside the 4..10 degree transition. Rotate
        # the live plane by 90 degrees; the output should blend smoothly from
        # the calibrated robot X/Z plane toward the observed Y/Z plane while
        # retaining the mapped bend angle.
        human_bend = reference + np.deg2rad(7.5)
        live_forearm = np.array([0.0, np.sin(human_bend), -np.cos(human_bend)])
        mapped, _ = BodyDevice._mapped_elbow_angle(
            human_bend, reference, robot_bend, "responsive_absolute", 5.0, 30.0
        )
        result = BodyDevice._neutral_relative_forearm(
            upper, live_forearm, reference, upper, robot_forearm,
            mode="responsive_absolute", start_deg=5.0, full_deg=30.0,
            plane_start_deg=4.0, plane_full_deg=10.0,
        )
        self.assertGreater(result[1], 0.0)
        self.assertGreater(result[0], 0.0)
        self.assertAlmostEqual(
            np.arccos(np.clip(np.dot(upper, result), -1.0, 1.0)), mapped, places=6
        )

        # Once additional mapped flexion exceeds 10 degrees, the live plane is
        # observable and should be followed completely.
        human_bend = reference + np.deg2rad(30.0)
        live_forearm = np.array([0.0, np.sin(human_bend), -np.cos(human_bend)])
        result = BodyDevice._neutral_relative_forearm(
            upper, live_forearm, reference, upper, robot_forearm,
            mode="responsive_absolute", start_deg=5.0, full_deg=30.0,
            plane_start_deg=4.0, plane_full_deg=10.0,
        )
        self.assertAlmostEqual(result[0], 0.0, places=7)
        self.assertGreater(result[1], 0.0)

    def test_latest_right_arm_low_bend_plane_replay_tracks_observable_j3(self):
        # Values from pico_arm_probe/pico_elbow_1789786601139793072 just
        # before the 32.2 mm wrist-geometry rejection.
        cfg = build_config()
        cfg.ik.urdf_path = str(
            Path(__file__).resolve().parents[1] / "urdf/esrobo_waist_with_head.urdf"
        )
        physical = np.array([
            0.02764601535159018, 1.4994472669733685, 0.01579522973054868,
            0.11445869234578814, -0.023282692221604357,
            0.014259339988793673, -0.08494517469456402,
        ])
        measured = np.zeros(14)
        measured[7:] = (
            physical - np.asarray(cfg.robot.right_joint_offsets)
        ) / np.asarray(cfg.robot.right_joint_directions)
        solver = IkSolver(cfg.ik)
        poses = solver.current_task_frame_poses(measured)
        shoulder = poses["right_shoulder"][:3]
        robot_upper = poses["right_elbow"][:3] - shoulder
        robot_forearm = poses["right_wrist"][:3] - poses["right_elbow"][:3]

        logged_elbow = np.array([-0.149520, -0.2755, 0.8852])
        logged_wrist = np.array([-0.1338, -0.2995, 0.4870])
        upper = mu.normalize_vector(logged_elbow - shoulder)
        logged_forearm = mu.normalize_vector(logged_wrist - logged_elbow)
        live_axis = mu.normalize_vector(np.cross(upper, logged_forearm))
        human_bend = np.deg2rad(38.38)
        live_forearm = mu.rotvec_to_rotation_matrix(live_axis * human_bend) @ upper
        mapped_forearm = BodyDevice._neutral_relative_forearm(
            upper, live_forearm, np.deg2rad(30.724781), robot_upper, robot_forearm,
            mode="responsive_absolute", start_deg=5.0, full_deg=30.0,
            plane_start_deg=4.0, plane_full_deg=10.0,
        )

        targets = {name: pose.copy() for name, pose in poses.items()}
        targets["right_elbow"][:3] = (
            shoulder + upper * np.linalg.norm(robot_upper)
        )
        targets["right_wrist"][:3] = (
            targets["right_elbow"][:3]
            + mapped_forearm * np.linalg.norm(robot_forearm)
        )
        solver.initialize_arm_session(
            "right", measured, np.deg2rad(30.724781),
            relative_elbow_reference=True,
        )
        result = solver.solve(
            targets["left_wrist"], targets["right_wrist"], measured,
            targets["left_elbow"], targets["right_elbow"], "right", measured[11:14],
        )

        self.assertTrue(solver.last_solution_valid, solver.solution_diagnostics)
        self.assertLess(solver.solution_diagnostics["wrist_error_m"], .001)
        self.assertFalse(solver.solution_diagnostics["j3_frozen"])
        self.assertGreater(abs(result[9] - measured[9]), np.deg2rad(1.0))

    def test_latest_rate_limited_right_arm_frame_reapplies_bend_plane_gate(self):
        # pico_elbow_1789904455791351896 at 1789904488.663: the raw mapped
        # bend had exposed the live plane, then endpoint limiting reduced the
        # actual target bend to 4.81 degrees while J3 remained frozen.
        cfg = build_config()
        cfg.ik.urdf_path = str(
            Path(__file__).resolve().parents[1] / "urdf/esrobo_waist_with_head.urdf"
        )
        solver = IkSolver(cfg.ik)
        startup_physical = np.array([
            -0.0020245819323134223, 1.5089244048116977,
            0.015917402778188285, 0.10756464180041053,
            -0.022916173078685546, 0.012042771838760876,
            -0.09625490824748727,
        ])
        startup = np.zeros(14)
        startup[7:] = (
            startup_physical - np.asarray(cfg.robot.right_joint_offsets)
        ) / np.asarray(cfg.robot.right_joint_directions)
        startup_poses = solver.current_task_frame_poses(startup)
        shoulder = np.array([
            -0.14680342376232147, -0.2350153774023056, 1.192513108253479,
        ])
        elbow = np.array([
            -0.1225259006023407, -0.32733431458473206, 0.8975762128829956,
        ])
        old_wrist = np.array([
            -0.06650044023990631, -0.4239063560962677, 0.5143983364105225,
        ])
        upper = mu.normalize_vector(elbow - shoulder)
        old_forearm = mu.normalize_vector(old_wrist - elbow)
        robot_upper = (
            startup_poses["right_elbow"][:3] - startup_poses["right_shoulder"][:3]
        )
        robot_forearm = (
            startup_poses["right_wrist"][:3] - startup_poses["right_elbow"][:3]
        )
        stable_forearm = BodyDevice._stabilize_forearm_plane(
            upper, old_forearm, robot_upper, robot_forearm,
            plane_start_deg=4.0, plane_full_deg=10.0,
        )
        corrected_wrist = elbow + stable_forearm * np.linalg.norm(robot_forearm)

        feedback = np.zeros(14)
        feedback[7:] = np.deg2rad([
            4.345, -17.333, 0.912, 8.108, -1.27, 0.682, -5.407,
        ])
        targets = {
            name: pose.copy()
            for name, pose in solver.current_task_frame_poses(feedback).items()
        }
        targets["right_elbow"][:3] = elbow
        targets["right_wrist"][:3] = corrected_wrist
        solver.initialize_arm_session(
            "right", startup, np.deg2rad(33.16810211906643),
            relative_elbow_reference=True,
        )
        result = solver.solve(
            targets["left_wrist"], targets["right_wrist"], feedback,
            targets["left_elbow"], targets["right_elbow"],
            "right", feedback[11:14],
        )

        self.assertGreater(np.linalg.norm(corrected_wrist - old_wrist), .026)
        self.assertTrue(solver.last_solution_valid, solver.solution_diagnostics)
        self.assertLess(solver.solution_diagnostics["wrist_error_m"], .001)
        self.assertFalse(solver.solution_diagnostics["j3_frozen"])
        self.assertGreater(abs(result[9] - feedback[9]), np.deg2rad(1.0))

    def device(self, side="left", swap=False):
        cfg = build_config().retarget
        cfg.swap_left_right_targets = swap
        cfg.auto_start_reference_prepare_s = 0.0
        cfg.auto_start_reference_sample_start_s = 0.0
        cfg.auto_start_reference_delay_s = 0.2
        cfg.auto_start_reference_min_samples = 5
        cfg.source_to_robot_rotation = np.eye(3).tolist()
        with mock.patch("esrobo_teleop.device.body_device.socket.socket"), mock.patch(
            "esrobo_teleop.device.body_device.threading.Thread"
        ):
            device = BodyDevice(cfg, active_arm_side=side)
        device._reference_diagnostic_path = None
        return device

    @staticmethod
    def frames(bend=18.0, yaw=0.0, translation=(0, 0, 0)):
        rotation = mu.rotvec_to_rotation_matrix([0, 0, np.deg2rad(yaw)])
        translation = np.asarray(translation)
        quat = mu.matrix_to_quat_wxyz(rotation)

        def pose(point):
            return np.concatenate((rotation @ np.asarray(point) + translation, quat))

        frames = {"waist": pose([0, 0, 0])}
        for side, sign in (("left", 1), ("right", -1)):
            shoulder = np.asarray([0, sign * .2, .5])
            elbow = shoulder + [0, 0, -.3]
            angle = np.deg2rad(bend if side == "left" else bend + 10)
            wrist = elbow + [.3 * np.sin(angle), 0, -.3 * np.cos(angle)]
            for name, point in zip(("shoulder", "elbow", "wrist"), (shoulder, elbow, wrist)):
                frames[f"{side}_{name}"] = pose(point)
        return frames

    def calibrate(self, device, frames, start=10.0):
        for index in range(8):
            now = start + index * .04
            device._frames = frames
            device._last_body_frame_packet_time_monotonic = now
            device._body_frame_history.append((now, frames))
            device._update_auto_reference(now)
        self.assertTrue(device._reference_locked)

    def test_same_pose_with_waist_motion_and_swapped_source(self):
        for side in ("left", "right"):
            for swap in (False, True):
                with self.subTest(side=side, swap=swap):
                    device = self.device(side, swap)
                    self.calibrate(device, self.frames())
                    expected = 18 if device._source_side_for_target(side) == "left" else 28
                    self.assertAlmostEqual(np.degrees(device.arm_reference_flexion_rad(side)), expected, places=3)
                    frames = self.frames(yaw=63, translation=(1, 2, 3))
                    device._body_frame_history = [(11 + i * .02, frames) for i in range(20)]
                    with mock.patch("esrobo_teleop.device.body_device.time.monotonic", return_value=11.4):
                        result = device.refresh_arm_reference_for_enable(side)
                    self.assertTrue(result[0])
                    self.assertLess(max(result[1:3]), .1)

    def test_large_stable_target_is_accepted_without_replacing_calibration(self):
        device = self.device()
        with tempfile.TemporaryDirectory() as directory:
            device._reference_diagnostic_path = Path(directory) / "reference.jsonl"
            self.calibrate(device, self.frames())
            reference = device._reference_left_points["wrist"].copy()
            device._body_frame_history = [(11 + i * .02, self.frames(bend=62)) for i in range(20)]
            with mock.patch("esrobo_teleop.device.body_device.time.monotonic", return_value=11.4):
                result = device.refresh_arm_reference_for_enable("left")
            self.assertTrue(result[0])
            self.assertAlmostEqual(result[2], 44, places=3)
            np.testing.assert_array_equal(reference, device._reference_left_points["wrist"])
            records = [json.loads(line) for line in device._reference_diagnostic_path.read_text().splitlines()]
            self.assertEqual([r["event"] for r in records], ["calibrated", "enable_target_ready"])
            self.assertEqual(records[-1]["source_side"], "left")
            self.assertAlmostEqual(records[-1]["candidate"]["flexion_deg"], 62, places=3)

    def test_post_calibration_current_pose_becomes_a_rate_limited_target(self):
        cfg = build_config()
        cfg.ik.urdf_path = str(
            Path(__file__).resolve().parents[1] / "urdf/esrobo_waist_with_head.urdf"
        )
        solver = IkSolver(cfg.ik)
        device = self.device("right")
        self.calibrate(device, self.frames(bend=18))
        robot = np.zeros(14)
        poses = solver.current_task_frame_poses(robot)
        device.rebase_robot_reference(poses)
        device._last_body_frame_packet_time_monotonic = 10.0
        device.seed_arm_reference_filters("right")

        device._frames = self.frames(bend=62)
        device._last_body_frame_packet_time_monotonic = 10.02
        device._last_target_update_time_monotonic = 10.0
        with mock.patch(
            "esrobo_teleop.device.body_device.time.monotonic", return_value=10.02
        ):
            targets = device.advance()

        self.assertIsNotNone(targets)
        step = np.linalg.norm(
            targets["right_wrist"][:3] - poses["right_wrist"][:3]
        )
        self.assertGreater(step, 0.0)
        self.assertLessEqual(
            step,
            cfg.retarget.max_endpoint_translation_velocity_m_s * 0.02 + 1.0e-8,
        )

    def test_sustained_right_bend_reaches_retarget_through_full_pipeline(self):
        cfg = build_config()
        cfg.ik.urdf_path = str(
            Path(__file__).resolve().parents[1] / "urdf/esrobo_waist_with_head.urdf"
        )
        solver = IkSolver(cfg.ik)
        device = self.device("right")
        self.calibrate(device, self.frames(bend=18))  # right human reference is 28 deg
        poses = solver.current_task_frame_poses(np.zeros(14))
        device.rebase_robot_reference(poses)
        device._last_body_frame_packet_time_monotonic = 10.0
        device.seed_arm_reference_filters("right")

        target_frames = self.frames(bend=63)  # right human target is 73 deg
        bends = []
        previous_wrist = poses["right_wrist"][:3].copy()
        for index in range(260):
            now = 10.02 + index * 0.02
            device._frames = target_frames
            device._last_body_frame_packet_time_monotonic = now
            device._body_frame_history.append((now, target_frames))
            device._body_frame_history = device._body_frame_history[-50:]
            device._last_target_update_time_monotonic = now - 0.02
            with mock.patch(
                "esrobo_teleop.device.body_device.time.monotonic", return_value=now
            ):
                targets = device.advance()
            self.assertIsNotNone(targets)
            wrist = targets["right_wrist"][:3]
            self.assertLessEqual(
                np.linalg.norm(wrist - previous_wrist),
                cfg.retarget.max_endpoint_translation_velocity_m_s * 0.02 + 1.0e-8,
            )
            previous_wrist = wrist.copy()
            bends.append(float(np.degrees(BodyDevice._points_flexion({
                "shoulder": poses["right_shoulder"][:3],
                "elbow": targets["right_elbow"][:3],
                "wrist": wrist,
            }))))

        self.assertGreater(bends[100], 45.0)
        self.assertAlmostEqual(bends[-1], 73.0, delta=0.1)
        diagnostics = device.arm_retarget_diagnostics("right")
        self.assertAlmostEqual(diagnostics["mapped_before_filter_deg"], 73.0, delta=0.1)
        self.assertAlmostEqual(diagnostics["limited_deg"], 73.0, delta=0.1)
        self.assertEqual(len(diagnostics["source_predicted_upper_direction"]), 3)
        self.assertEqual(len(diagnostics["desired_after_plane_forearm_direction"]), 3)
        self.assertEqual(len(diagnostics["limited_forearm_direction"]), 3)
        self.assertEqual(diagnostics["limiter"]["method"], "arm_state_arc_backtrack")

    def test_restart_drops_old_data_but_preserves_glove_calibration(self):
        device = self.device()
        self.calibrate(device, self.frames())
        imu = {"left": np.eye(3)}
        device._hand_imu_reference_delta_matrices = imu
        device.restart_arm_reference()
        self.assertFalse(device.is_ready())
        self.assertIsNone(device._reference_left_points)
        self.assertEqual(device._body_frame_history, [])
        self.assertIs(device._hand_imu_reference_delta_matrices, imu)
        self.assertIsNone(device.refresh_arm_reference_for_enable("left"))
        self.calibrate(device, self.frames(bend=25), start=20)
        self.assertAlmostEqual(np.degrees(device.arm_reference_flexion_rad("left")), 25, places=3)

    def test_invalid_and_stale_frames_cannot_be_accepted(self):
        device = self.device()
        self.calibrate(device, self.frames())
        with mock.patch("esrobo_teleop.device.body_device.time.monotonic", return_value=30):
            self.assertIsNone(device.refresh_arm_reference_for_enable("left"))
        for defect in ("waist", "left_wrist", "nan", "zero"):
            frames = self.frames()
            if defect == "nan":
                frames["left_wrist"][0] = np.nan
            elif defect == "zero":
                frames["left_wrist"] = frames["left_elbow"].copy()
            else:
                del frames[defect]
            self.assertIsNone(device._arm_points_from_frames(frames, "left"))
        device.restart_arm_reference()
        device._frames = frames
        device._update_auto_reference(30)
        self.assertFalse(device._reference_locked)

    def test_recalibration_key_is_hardware_free_and_guarded(self):
        for side in ("left", "right"):
            node = TeleopNode.__new__(TeleopNode)
            node._arm_only = True
            node._arm_side = side
            node._motors_enabled = node._arm_armed = node._hand_enabled = False
            node._manual_intervention_required = False
            node._driver = mock.Mock(safety_fault_reason=None)
            node._body = mock.Mock()
            node._ik = mock.Mock()
            for flag in ("_motors_enabled", "_arm_armed", "_hand_enabled", "_manual_intervention_required"):
                setattr(node, flag, True)
                node._handle_key("r")
                node._body.restart_arm_reference.assert_not_called()
                setattr(node, flag, False)
            node._driver.safety_fault_reason = "torque fault"
            node._handle_key("r")
            node._body.restart_arm_reference.assert_not_called()
            node._driver.safety_fault_reason = None
            node._handle_key("r")
            node._body.restart_arm_reference.assert_called_once_with()
            node._ik.reset_j3_observability_baseline.assert_called_once_with(side)
            self.assertFalse(node._arm_armed)
            self.assertEqual(node._driver.mock_calls, [])

    def test_e_during_recalibration_without_locked_reference_never_enables(self):
        node = TeleopNode.__new__(TeleopNode)
        node._cfg = build_config()
        node._cfg.retarget.arm_reference_enable_prepare_s = 0
        node._wrist_imu_side = None
        node._arm_only = True
        node._arm_side = "right"
        node._motors_enabled = False
        node._driver = mock.Mock()
        node._body = mock.Mock()
        node._body.is_ready.return_value = False
        with contextlib.redirect_stdout(io.StringIO()):
            node._handle_key("e")
        self.assertFalse(node._arm_armed)
        self.assertEqual(node._driver.mock_calls, [])

    def test_absolute_transition_is_continuous_monotone_and_reversible(self):
        reference = np.deg2rad(29)
        inputs = np.linspace(0, np.pi, 2001)
        outputs = [BodyDevice._mapped_elbow_angle(b, reference, 0, "smooth_absolute", 5, 30)[0]
                   for b in inputs]
        self.assertGreaterEqual(min(np.diff(outputs)), -1e-10)
        self.assertAlmostEqual(BodyDevice._mapped_elbow_angle(np.deg2rad(34), reference, 0,
                                                           "smooth_absolute", 5, 30)[0], 0)
        for b in (59, 90, 120):
            mapped, weight = BodyDevice._mapped_elbow_angle(np.deg2rad(b), reference, 0,
                                                          "smooth_absolute", 5, 30)
            self.assertAlmostEqual(np.degrees(mapped), b, places=5)
            self.assertAlmostEqual(weight, 1)
        backward = [BodyDevice._mapped_elbow_angle(b, reference, 0, "smooth_absolute", 5, 30)[0]
                    for b in inputs[::-1]]
        np.testing.assert_allclose(outputs, backward[::-1])
        for boundary in (34, 59):
            a = BodyDevice._mapped_elbow_angle(np.deg2rad(boundary-1e-5), reference, 0, "smooth_absolute", 5, 30)[0]
            b = BodyDevice._mapped_elbow_angle(np.deg2rad(boundary+1e-5), reference, 0, "smooth_absolute", 5, 30)[0]
            self.assertLess(abs(a-b), 1e-5)
        folded = BodyDevice._neutral_relative_forearm(
            np.array([0., 0., -1.]), np.array([0., 0., 1.]), reference,
            np.array([0., 0., -1.]), np.array([0., 0., -1.]), mode="smooth_absolute")
        np.testing.assert_allclose(folded, [0, 0, 1], atol=1e-6)

    def test_responsive_mapping_improves_departure_without_neutral_jump(self):
        for reference_deg in (0, 25, 49):
            reference = np.deg2rad(reference_deg)
            angles = np.linspace(0, np.pi, 4001)
            def mapped(b, mode="responsive_absolute"):
                return BodyDevice._mapped_elbow_angle(b, reference, 0, mode, 5, 30)[0]
            values = np.array([mapped(b) for b in angles])
            self.assertGreaterEqual(np.min(np.diff(values)), -1e-9)
            np.testing.assert_allclose(values, [mapped(b) for b in angles[::-1]][::-1])
            self.assertAlmostEqual(mapped(reference + np.deg2rad(5)), 0, places=9)
            for departure in (7, 10, 15, 20):
                angle = reference + np.deg2rad(departure)
                self.assertGreater(mapped(angle), mapped(angle, "smooth_absolute"))
                self.assertLessEqual(mapped(angle), angle + 1e-9)
            self.assertAlmostEqual(mapped(reference + np.deg2rad(30)), reference + np.deg2rad(30))
            for boundary in (5, 10, 30):
                x = reference + np.deg2rad(boundary)
                self.assertLess(abs(mapped(x+1e-7)-mapped(x-1e-7)), 1e-5)

    def test_direct_absolute_mapping_preserves_live_pico_elbow_angle(self):
        # Latest left-arm run: calibration was 66.4 deg while the live PICO
        # bend was 54.67 deg. Relative-to-calibration mapping collapsed this to
        # the 0.43 deg robot startup bend, making the forearm visibly wrong.
        live = np.deg2rad(54.67450768129151)
        mapped, weight = BodyDevice._mapped_elbow_angle(
            live, np.deg2rad(66.40470886230469), np.deg2rad(0.4279584586620331),
            "direct_absolute", 5.0, 30.0,
        )
        self.assertAlmostEqual(mapped, live)
        self.assertEqual(weight, 1.0)

    def test_bent_forearm_plane_is_observable_without_startup_departure(self):
        upper = np.array([0.0, 0.0, -1.0])
        bend = np.deg2rad(30.0)
        robot_forearm = np.array([np.sin(bend), 0.0, -np.cos(bend)])
        live_forearm = np.array([0.0, np.sin(bend), -np.cos(bend)])
        result = BodyDevice._neutral_relative_forearm(
            upper, live_forearm, bend, upper, robot_forearm,
            mode="direct_absolute",
            plane_start_deg=4.0, plane_full_deg=10.0,
        )
        np.testing.assert_allclose(result, live_forearm, atol=1e-7)

    def test_incident_pose_changes_are_targets_but_cannot_rewrite_calibration(self):
        device = self.device("right")
        self.calibrate(device, self.frames(bend=18.9411))  # right side adds 10 deg
        original = device._reference_right_points["wrist"].copy()
        for bend in (60.172, 48.469):
            device._body_frame_history = [(11+i*.02, self.frames(bend=bend-10)) for i in range(20)]
            with mock.patch("esrobo_teleop.device.body_device.time.monotonic", return_value=11.4):
                result = device.refresh_arm_reference_for_enable("right")
            self.assertTrue(result[0])
            np.testing.assert_array_equal(device._reference_right_points["wrist"], original)
            self.assertAlmostEqual(np.degrees(device.arm_reference_flexion_rad("right")), 28.9411, places=3)

    def test_prepare_countdown_has_no_hardware_calls_and_can_cancel(self):
        node = TeleopNode.__new__(TeleopNode)
        node._cfg = build_config()
        node._stop = False
        node._driver = mock.Mock()
        with mock.patch("esrobo_teleop.teleop_node.time.monotonic", side_effect=[0, 0, 1, 2, 3, 4, 5]), mock.patch(
            "esrobo_teleop.teleop_node.time.sleep"
        ), contextlib.redirect_stdout(io.StringIO()) as output:
            self.assertTrue(node._wait_for_arm_prepare())
        self.assertIn("5 秒", output.getvalue())
        node._stop = True
        self.assertFalse(node._wait_for_arm_prepare())
        self.assertEqual(node._driver.mock_calls, [])

    def test_invalid_mapping_config_rejected_before_socket_creation(self):
        for key, value in (("elbow_angle_mapping", "bad"), ("elbow_absolute_start_delta_deg", 30),
                           ("elbow_absolute_full_delta_deg", np.nan),
                           ("bend_plane_observability_full_delta_deg", 4),
                           ("arm_reference_enable_prepare_s", -1)):
            cfg = build_config().retarget
            setattr(cfg, key, value)
            with mock.patch("esrobo_teleop.device.body_device.socket.socket") as socket:
                with self.assertRaises(ValueError):
                    BodyDevice(cfg)
                socket.assert_not_called()

    def test_ninety_degree_human_bend_reaches_ik_without_reference_subtraction(self):
        cfg = build_config()
        cfg.ik.urdf_path = str(Path(__file__).resolve().parents[1] / "urdf/esrobo_waist_with_head.urdf")
        for side in ("left", "right"):
            device = self.device(side)
            device._cfg.upper_arm_angular_deadband_deg = 0
            device._cfg.forearm_angular_deadband_deg = 0
            device._filter_segment_direction = lambda _side, _name, vector: vector
            solver = IkSolver(cfg.ik)
            joints = np.zeros(14)
            poses = solver.current_task_frame_poses(joints)
            device.rebase_robot_reference(poses)
            reference = device._arm_points_from_frames(self.frames(bend=29 if side == "left" else 19), side)
            setattr(device, f"_reference_{side}_points", reference)
            points = device._arm_points_from_frames(self.frames(bend=90 if side == "left" else 80), side)
            elbow, wrist = device._retarget_arm_segment_direction_positions(points, side)
            target_angle = BodyDevice._points_flexion(dict(
                shoulder=poses[f"{side}_shoulder"][:3], elbow=elbow, wrist=wrist))
            self.assertAlmostEqual(np.degrees(target_angle), 90, delta=.1)
            flexion = solver._segment_elbow_flexion(side, joints, elbow, wrist)
            self.assertAlmostEqual(np.degrees(flexion), 90, delta=.1)

    def test_right_j4_target_does_not_chase_feedback_near_straight_hand_offset(self):
        """Regression for pico_elbow_1789912398059388828.

        With held J7 near -5 degrees, the shoulder/elbow/hand-base angle crosses
        an absolute-value branch near extension.  The old live-feedback delta
        changed a fixed roughly -0.16 degree J4 target into +2.26 degrees as
        the arm followed, eventually tripping the 30 mm IK geometry guard.
        """
        cfg = build_config()
        cfg.ik.urdf_path = str(
            Path(__file__).resolve().parents[1] / "urdf/esrobo_waist_with_head.urdf"
        )
        solver = IkSolver(cfg.ik)
        startup = np.zeros(14)
        startup[7:] = np.deg2rad([
            0.005, 0.157, 0.009, 0.069, -0.053, 0.0, -5.061,
        ])
        solver.initialize_arm_session(
            "right", startup, np.deg2rad(22.320469103785758),
            relative_elbow_reference=True,
        )
        target_elbow = np.array([-0.1555, -0.2524, 0.8831])
        target_wrist = np.array([-0.1668, -0.2840, 0.4854])

        requested = []
        for feedback_j4 in (0.859, 1.273, 1.712, 2.481):
            feedback = startup.copy()
            feedback[10] = np.deg2rad(feedback_j4)
            requested.append(np.degrees(solver._segment_elbow_flexion(
                "right", feedback, target_elbow, target_wrist
            )))

        self.assertLess(np.ptp(requested), 1.0e-6)
        self.assertAlmostEqual(requested[0], -0.16, delta=0.03)

        feedback = startup.copy()
        feedback[10] = np.deg2rad(1.273)
        poses = solver.current_task_frame_poses(feedback)
        poses["right_elbow"][:3] = target_elbow
        poses["right_wrist"][:3] = target_wrist
        result = solver.solve(
            poses["left_wrist"], poses["right_wrist"], feedback,
            poses["left_elbow"], poses["right_elbow"],
            partitioned_side="right", terminal_joint_targets=feedback[11:14],
        )
        self.assertTrue(solver.last_solution_valid, solver.solution_diagnostics)
        self.assertLess(solver.solution_diagnostics["wrist_error_m"], 0.003)
        # The unsigned angle approximation is only a seed now. Verify the
        # actual hand-base endpoint, and invariance to delayed J4 feedback.
        self.assertLess(solver.solution_diagnostics["wrist_error_m"], .0002)
        solved = result.copy()
        for feedback_j4 in (0.859, 1.273, 1.712, 2.481):
            feedback[10] = np.deg2rad(feedback_j4)
            result = solver.solve(
                poses["left_wrist"], poses["right_wrist"], feedback,
                poses["left_elbow"], poses["right_elbow"],
                partitioned_side="right", terminal_joint_targets=feedback[11:14],
            )
            self.assertTrue(solver.last_solution_valid)
            np.testing.assert_allclose(result[7:11], solved[7:11], atol=1e-5)

    def test_diagnostics_distinguishes_ik_from_actual_sent_command(self):
        from types import SimpleNamespace
        cfg = build_config()
        cfg.ik.urdf_path = str(Path(__file__).resolve().parents[1] / "urdf/esrobo_waist_with_head.urdf")
        node = TeleopNode.__new__(TeleopNode)
        node._ik = IkSolver(cfg.ik)
        node._arm_side = "right"
        node._arm_armed = True
        node._command_rejected = False
        node._arm_diagnostics_socket = mock.Mock()
        node._body = mock.Mock()
        node._body.elbow_diagnostics.return_value = {"reference_deg": 29, "human_deg": 90, "blend_weight": 1}
        node._body.arm_retarget_diagnostics.return_value = {
            "mapped_before_filter_deg": 90.0,
            "desired_before_limit_deg": 90.0,
            "limited_deg": 12.0,
            "limiter": {"progress": 0.1, "backtracks": 0},
        }
        node._elbow_log_failed = False
        feedback = np.zeros(14)
        target = np.zeros(14)
        target[10] = np.deg2rad(90)
        sent = target.copy()
        sent[10] = np.deg2rad(30)
        poses = node._ik.current_task_frame_poses(target)
        with tempfile.TemporaryDirectory() as directory:
            node._elbow_log_path = Path(directory) / "elbow.jsonl"
            for command in (None, sent):
                node._last_arm_diagnostics_time = 0
                node._driver = SimpleNamespace(last_commanded_full_urdf=lambda: command,
                                               elbow_command_diagnostics=None,
                                               trajectory_diagnostics={'braking_recovery': np.bool_(True)},
                                               startup_events=[{
                                                   "timestamp": 1.0,
                                                   "event": "armed",
                                                   "phase": "startup",
                                                   "controller_snapshot": {"large": "x" * 10000},
                                               }])
                node._publish_arm_diagnostics(poses, target, feedback)
                payload = json.loads(node._arm_diagnostics_socket.sendto.call_args[0][0])
                self.assertIs(payload['trajectory']['braking_recovery'], True)
                self.assertAlmostEqual(payload["elbow"]["ik_j4_deg"], 90)
                self.assertEqual(payload["elbow"]["feedback_j4_deg"], 0)
                self.assertEqual(payload["retarget_pipeline"]["limited_deg"], 12.0)
                self.assertEqual(payload["retarget_pipeline"]["limiter"]["progress"], 0.1)
                self.assertEqual(payload["startup_events"], [{
                    "timestamp": 1.0, "phase": "startup", "event": "armed"
                }])
                if command is None:
                    self.assertIsNone(payload["elbow"]["last_sent_j4_deg"])
                else:
                    self.assertAlmostEqual(payload["elbow"]["last_sent_j4_deg"], 30)
            node._diagnostic_writer.close()
            self.assertEqual(len(node._elbow_log_path.read_text().splitlines()), 2)


if __name__ == "__main__":
    unittest.main()
