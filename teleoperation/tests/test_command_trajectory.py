import json
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest import mock

import numpy as np

from esrobo_teleop.config import build_config, load_config
from esrobo_teleop.debug.async_json_log import AsyncJsonLog
from esrobo_teleop.ik.solver import IkSolver
from esrobo_teleop.robot.command_trajectory import CommandTrajectory, TrajectoryFault, TrajectoryStep
from esrobo_teleop.robot.nero_driver import NeroSingleArmDriver, NeroArm, JointFeedbackSnapshot
from esrobo_teleop.teleop_node import TeleopNode
from scripts.analyze_command_trajectory import simulate


def linear_fk(q):
    return np.array([q[:3] * .3, [q[3] * .3, q[4] * .3, q[5] * .3]])


def planner():
    cfg = build_config().robot
    result = CommandTrajectory(cfg.right_max_joint_velocity, cfg.right_max_joint_acceleration,
                               cfg.right_max_joint_step, np.deg2rad(cfg.command_trajectory_lead_deg))
    result.reset(np.zeros(7), 10.0)
    return result


class TrajectoryTests(unittest.TestCase):
    def test_latest_left_feedback_stall_still_rejects_after_parameter_sync(self):
        path = Path(__file__).parent / 'fixtures/left_feedback_stall_20260920.json'
        context = json.loads(path.read_text())['fault']['fault_context']
        p = planner()
        stamp, dt = context['feedback_stamp_s'], context['feedback_dt_s']
        since = np.full(7, np.nan)
        since[0] = stamp-context['lead_elapsed_s'][0]
        p.state = TrajectoryStep(np.array(context['previous_command_physical_rad']), np.zeros(7),
                                 stamp-dt, since, {'lead_watch_feedback_rad': context['lead_watch_feedback_rad']})
        with self.assertRaisesRegex(TrajectoryFault, 'persistent feedback lead on joints') as caught:
            p.propose(context['target_physical_rad'], context['feedback_physical_rad'], stamp,
                      -np.ones(7)*3, np.ones(7)*3, linear_fk, .22)
        self.assertEqual(caught.exception.diagnostics['failed_joints'], [1])
        self.assertEqual(caught.exception.diagnostics['lead_progress_from_anchor_rad'], [0.]*7)

    def test_single_arm_motion_parameters_match_without_copying_installation_or_firmware(self):
        cfg = load_config(str(Path(__file__).resolve().parents[1]/'config/teleop_config.yaml')).robot
        self.assertAlmostEqual(np.degrees(cfg.left_joint_soft_upper_limits_urdf[3]), 120., places=4)
        self.assertEqual(cfg.left_joint_soft_upper_limits_urdf[3], cfg.right_joint_soft_upper_limits_urdf[3])
        for name in ('max_joint_step', 'max_joint_velocity', 'max_joint_acceleration',
                     'command_trajectory_enabled'):
            self.assertEqual(getattr(cfg, 'left_'+name), getattr(cfg, 'right_'+name))
        self.assertEqual(cfg.left_torque_deviation_limits_nm[:2], [25., 25.])
        self.assertEqual(cfg.right_torque_deviation_limits_nm[:2], [21., 21.])
        self.assertEqual(
            cfg.left_torque_deviation_limits_nm[2:],
            cfg.right_torque_deviation_limits_nm[2:],
        )
        self.assertNotEqual(cfg.left_joint_soft_lower_limits_urdf[2], cfg.right_joint_soft_lower_limits_urdf[2])
        self.assertEqual(cfg.nero_firmware, 'V112')
        self.assertEqual(cfg.right_nero_firmware, 'DEFAULT')
        self.assertFalse(cfg.left_command_backpressure_enabled)
        self.assertFalse(cfg.right_command_backpressure_enabled)
        self.assertEqual(cfg.left_tracking_control_mode, 'j')
        self.assertEqual(cfg.right_tracking_control_mode, 'j')

    def test_asynchronous_feedback_and_new_command_require_one_collision_box(self):
        p = planner()
        feedback = np.zeros(7)
        feedback[0] = -.04
        target = np.zeros(7)
        target[1] = .3

        def safe_box(a, b):
            lo, hi = np.minimum(a, b), np.maximum(a, b)
            # One lagging axis plus one advancing axis enters this obstacle.
            return not (lo[0] < -.02 and hi[1] > .0001)

        old_candidate = np.zeros(7)
        old_candidate[1] = p.amax[1] * .02**2
        self.assertTrue(safe_box(feedback, p.state.position))
        self.assertTrue(safe_box(p.state.position, old_candidate))
        self.assertTrue(safe_box(old_candidate, old_candidate * 10))
        self.assertFalse(safe_box(feedback, old_candidate))
        step = p.propose(target, feedback, 10.02, -np.ones(7), np.ones(7),
                         linear_fk, .22, path_check=safe_box)
        self.assertGreater(step.diagnostics['collision_rejections'], 0)
        stop = step.position + step.velocity*p.max_dt + step.velocity*abs(step.velocity)/(2*p.amax)
        box = np.array([feedback, p.state.position, step.position, stop])
        self.assertTrue(safe_box(box.min(axis=0), box.max(axis=0)))
        self.assert_limits(p, p.state, step, feedback, .02)

    def test_unsafe_outstanding_path_rejects_and_captures_exact_state(self):
        p = planner()
        before = p.state
        feedback = np.ones(7)*.001
        with self.assertRaises(TrajectoryFault) as caught:
            p.propose(np.ones(7)*.2, feedback, 10.02, -np.ones(7), np.ones(7),
                      linear_fk, .22, path_check=lambda a, b: False)
        self.assertIs(p.state, before)
        context = caught.exception.diagnostics
        self.assertEqual(context['collision_stage'], 'outstanding_motion')
        np.testing.assert_array_equal(context['feedback_physical_rad'], feedback)
        np.testing.assert_array_equal(context['previous_command_physical_rad'], before.position)

    def test_braking_recovery_diagnostics_are_plain_json(self):
        from dataclasses import replace
        p = planner()
        p.state = replace(p.state, velocity=np.ones(7)*.02,
                          diagnostics={'braking_recovery': True})
        step = p.propose(np.zeros(7), np.zeros(7), 10.02, -np.ones(7), np.ones(7), linear_fk, .22)
        self.assertIs(type(step.diagnostics['braking_recovery']), bool)
        json.dumps(step.diagnostics, allow_nan=False)

    def test_per_side_hardware_and_trajectory_defaults_match_runtime_yaml(self):
        default = build_config().robot
        runtime = load_config(str(Path(__file__).resolve().parents[1] / "config/teleop_config.yaml")).robot
        for side in ("left", "right"):
            for suffix in ("command_trajectory_enabled", "max_joint_velocity", "max_joint_acceleration",
                           "max_joint_step", "joint_soft_lower_limits_urdf", "joint_soft_upper_limits_urdf",
                           "joint_directions", "joint_offsets", "torque_deviation_limits_nm"):
                field = f"{side}_{suffix}"
                np.testing.assert_allclose(getattr(default, field), getattr(runtime, field), err_msg=field)
            self.assertEqual(getattr(default, f"{side}_can_channel"), getattr(runtime, f"{side}_can_channel"))
        self.assertEqual(default.nero_firmware, "V112")
        self.assertEqual(default.right_nero_firmware, "DEFAULT")

    def test_same_synthetic_targets_improve_without_increasing_limits(self):
        for profile in ("step", "ramp", "reverse"):
            old = simulate("legacy", profile)
            new = simulate("trajectory", profile)
            self.assertLess(new["mean_error_deg"], old["mean_error_deg"])
            self.assertLessEqual(new["max_lead_deg"], 3)
            if profile != "ramp":
                self.assertLess(new["settling_s_after_last_target_change"],
                                old["settling_s_after_last_target_change"])

    def assert_limits(self, trajectory, before, proposal, feedback, dt, fk=linear_fk):
        self.assertTrue(np.all(np.abs(proposal.velocity) <= trajectory.vmax + 1e-8))
        self.assertTrue(np.all(np.abs(proposal.velocity - before.velocity) <= trajectory.amax * dt + 1e-8))
        self.assertTrue(np.all(np.abs(proposal.position - before.position) <= trajectory.step + 1e-8))
        self.assertTrue(np.all(np.abs(proposal.position - feedback) <= trajectory.lead + 1e-8))
        self.assertTrue(np.all(np.linalg.norm(fk(proposal.position)-fk(before.position), axis=1) <= .22*dt + 1e-8))

    def test_step_ramp_stop_reverse_with_lag_and_irregular_dt(self):
        for profile in ("step", "ramp", "reverse"):
            with self.subTest(profile=profile):
                trajectory = planner()
                feedback = np.zeros(7)
                stamp = 10.0
                for i in range(600):
                    dt = [.02, .015, .025][i % 3]
                    stamp += dt
                    target = np.zeros(7)
                    target[3] = (.35 if profile == "step" else min(.35, i*.0015)
                                 if profile == "ramp" else .3 if i < 180 else -.2)
                    before = trajectory.state
                    step = trajectory.propose(target, feedback, stamp, -np.ones(7), np.ones(7), linear_fk, .22)
                    self.assertIs(trajectory.state, before)  # Proposing never commits.
                    self.assert_limits(trajectory, before, step, feedback, dt)
                    trajectory.commit(step)
                    feedback += dt / (.1+dt) * (step.position-feedback)
                self.assertLess(abs(feedback[3]-target[3]), .005)

    def test_stuck_feedback_lead_timeout_and_no_unbounded_motion(self):
        trajectory = planner()
        target = np.ones(7) * .5
        with self.assertRaisesRegex(TrajectoryFault, "persistent feedback lead"):
            for i in range(250):
                step = trajectory.propose(target, np.zeros(7), 10+(i+1)*.02,
                                          -np.ones(7), np.ones(7), linear_fk, .22)
                self.assertTrue(np.all(np.abs(step.position) <= trajectory.lead+1e-8))
                trajectory.commit(step)

    def test_delayed_controller_progress_restarts_no_progress_timeout(self):
        trajectory = planner()
        target = np.zeros(7)
        target[:2] = .5
        feedback = np.zeros(7)
        reset_seen = False
        # The controller stays still for 1.9 s after enable, then begins to
        # catch the held command.  This reproduces the latest right-arm log.
        for i in range(150):
            stamp = 10 + (i + 1) * .02
            if i >= 95:
                feedback += np.sign(trajectory.state.position - feedback) * np.minimum(
                    np.abs(trajectory.state.position - feedback), np.deg2rad(.04)
                )
            step = trajectory.propose(
                target, feedback, stamp,
                -np.ones(7), np.ones(7), linear_fk, .22,
            )
            self.assertTrue(np.all(np.abs(step.position - feedback) <= trajectory.lead + 1e-8))
            reset_seen = reset_seen or any(step.diagnostics["lead_progress_reset"])
            trajectory.commit(step)
        self.assertTrue(reset_seen)
        self.assertLess(np.max(np.abs(trajectory.state.position - feedback)), trajectory.lead[0])

    def test_duplicate_reversed_and_long_gap_timestamps(self):
        trajectory = planner()
        args = (np.zeros(7), np.zeros(7))
        rest = (-np.ones(7), np.ones(7), linear_fk, .22)
        self.assertIsNone(trajectory.propose(*args, 10, *rest))
        for stamp in (9.99, 10.1001, np.nan):
            with self.assertRaises(TrajectoryFault):
                trajectory.propose(*args, stamp, *rest)
        trajectory.invalidate()
        with self.assertRaisesRegex(TrajectoryFault, "initialization"):
            trajectory.propose(*args, 10.02, *rest)

    def test_shrinking_feedback_envelope_records_infeasible_joint_without_committing(self):
        trajectory = planner()
        target = np.array([.3, 0, 0, 0, 0, 0, 0])
        for i in range(10):
            previous = trajectory.state
            step = trajectory.propose(target, previous.position, 10+(i+1)*.02,
                                      -np.ones(7), np.ones(7), linear_fk, .22)
            trajectory.commit(step)
        before = trajectory.state
        feedback = before.position.copy()
        feedback[0] -= trajectory.lead[0] - .0001
        with self.assertRaises(TrajectoryFault) as caught:
            trajectory.propose(target, feedback, before.stamp+.02,
                               -np.ones(7), np.ones(7), linear_fk, .22)
        context = caught.exception.diagnostics
        self.assertEqual(context['failed_joints'], [1])
        self.assertGreater(context['feasible_velocity_lower_rad_s'][0],
                           context['feasible_velocity_upper_rad_s'][0])
        self.assertIn('J1:', str(caught.exception))
        self.assertIs(trajectory.state, before)
        json.dumps(context, allow_nan=False)

    def test_cartesian_shrinking_and_infeasible_braking_candidate(self):
        trajectory = planner()
        target = np.ones(7)*.5
        def fast_fk(q):
            return linear_fk(q) * 100
        step = trajectory.propose(target, np.zeros(7), 10.02, -np.ones(7), np.ones(7), fast_fk, .22)
        self.assertLess(step.diagnostics["cartesian_progress"], 1)
        self.assert_limits(trajectory, trajectory.state, step, np.zeros(7), .02, fast_fk)
        trajectory.commit(step)
        with self.assertRaisesRegex(TrajectoryFault, "speed candidate"):
            trajectory.propose(target, step.position, 10.021, -np.ones(7), np.ones(7),
                               lambda q: fast_fk(q)*1e6, .22)

    def test_soft_limits_and_abrupt_feedback_jump_rejected(self):
        trajectory = planner()
        lower, upper = -np.ones(7)*.1, np.ones(7)*.1
        for i in range(150):
            feedback = trajectory.state.position.copy()
            step = trajectory.propose(np.ones(7), feedback, 10+(i+1)*.02, lower, upper, linear_fk, .22)
            self.assertTrue(np.all(step.position <= upper+1e-8))
            self.assert_limits(trajectory, trajectory.state, step, feedback, .02)
            trajectory.commit(step)
        with self.assertRaisesRegex(TrajectoryFault, "envelope"):
            trajectory.propose(np.ones(7), -np.ones(7), 13.02, lower, upper, linear_fk, .22)

    def test_independent_urdf_fk_both_sides(self):
        cfg = build_config().ik
        cfg.urdf_path = str(Path(__file__).resolve().parents[1] / "urdf/esrobo_waist_with_head.urdf")
        solver = IkSolver(cfg)
        for side in ("left", "right"):
            fk = solver.make_command_fk(side)
            joints = np.linspace(-.02, .02, 14)
            expected = solver.current_task_frame_poses(joints)
            np.testing.assert_allclose(fk(joints), [expected[f"{side}_elbow"][:3], expected[f"{side}_wrist"][:3]], atol=1e-6)
            trajectory = planner()
            def active_fk(q):
                full = np.zeros(14)
                start = 0 if side == "left" else 7
                full[start:start+7] = q
                return fk(full)
            for i in range(100):
                before = trajectory.state
                target = np.array([.15, -.1, .2, .3, 0, 0, 0])
                step = trajectory.propose(target, before.position, 10+(i+1)*.02,
                                          -np.ones(7), np.ones(7), active_fk, .22)
                self.assert_limits(trajectory, before, step, before.position, .02, active_fk)
                trajectory.commit(step)


class DriverTrajectoryTests(unittest.TestCase):
    def test_position_limit_hold_uses_brake_only_trajectory_updates(self):
        driver = NeroSingleArmDriver.__new__(NeroSingleArmDriver)
        driver._cfg = build_config().robot
        driver._side = "right"
        driver._trajectory_lock = threading.RLock()
        driver._return_active = False
        driver._return_pending = False
        driver._return_cancel = threading.Event()
        driver._last_commanded_full_urdf = np.zeros(14)
        driver._command_trajectory_full_urdf = mock.Mock(return_value=True)

        self.assertTrue(driver.hold_command_trajectory())
        np.testing.assert_array_equal(
            driver._command_trajectory_full_urdf.call_args.args[0], np.zeros(14)
        )
        self.assertTrue(driver._command_trajectory_full_urdf.call_args.kwargs["brake_only"])

    def test_live_collision_policy_skips_follow_and_gap_recovery_only(self):
        for side in ('left', 'right'):
            with self.subTest(side=side):
                driver = self.driver(side)
                self.assertFalse(driver._cfg.teleop_torso_collision_enabled)
                guard = mock.Mock(return_value=False)
                driver.configure_collision_guard(guard)
                driver._arm.last_feedback_timestamp = 10.02
                self.assertTrue(driver.command_full_urdf(np.zeros(14)))
                guard.assert_not_called()
                self.assertFalse(driver.trajectory_diagnostics['torso_collision_check_active'])
                snapshot = JointFeedbackSnapshot(self.current.copy(), 10.2, time.monotonic(), 'get_joint_angles')
                self.assertTrue(driver._recover_command_trajectory_after_feedback_gap(snapshot))
                guard.assert_not_called()
                # One-time arming preflight still rejects the same pose.
                driver._arm.last_feedback_timestamp = 10.22
                self.assertFalse(driver.initialize_command_trajectory())
                guard.assert_called()

    def test_explicit_live_collision_opt_in_still_rejects_without_send(self):
        driver = self.driver()
        driver._cfg.teleop_torso_collision_enabled = True
        driver.configure_collision_guard(mock.Mock(return_value=False))
        driver.startup_event_sink = mock.Mock()
        driver._arm.last_feedback_timestamp = 10.02
        self.assertFalse(driver.command_full_urdf(np.zeros(14)))
        driver._arm.move_j.assert_not_called()
        event = driver.startup_event_sink.call_args.args[0]
        self.assertEqual(event['phase'], 'trajectory_fault')
        self.assertEqual(event['fault_context']['collision_stage'], 'outstanding_motion')

    def test_fault_event_sink_failure_does_not_break_fault_latch(self):
        driver = self.driver()
        driver.startup_event_sink = mock.Mock(side_effect=OSError('log unavailable'))
        self.assertFalse(driver._trajectory_fault('test fault'))
        self.assertIn('test fault', driver.safety_fault_reason)
        self.assertIsNone(driver._trajectory.state)
        driver._arm.move_j.assert_not_called()

    def driver(self, side="right", trajectory_enabled=None):
        cfg = build_config().robot
        if trajectory_enabled is not None:
            setattr(cfg, f"{side}_command_trajectory_enabled", trajectory_enabled)
        with mock.patch("esrobo_teleop.robot.nero_driver.NeroArm"):
            driver = NeroSingleArmDriver(cfg, side)
        self.current = np.array(cfg.right_joint_offsets if side == "right" else cfg.left_joint_offsets, dtype=float)
        driver._arm.get_joint_angles.side_effect = lambda: self.current.copy()
        driver._arm.last_feedback_timestamp = 10.0
        driver._arm.controller_fault.return_value = None
        driver.configure_collision_guard(lambda a, b: True)
        driver._arm.move_j.return_value = None
        driver._enabled = True
        driver._torque_is_safe = mock.Mock(return_value=True)
        start = 0 if side == "left" else 7
        driver.configure_command_trajectory(lambda q: linear_fk(q[start:start+7]), .22)
        driver.capture_session_start()
        self.assertTrue(driver.initialize_command_trajectory())
        return driver

    def test_duplicate_no_send_then_success_commits(self):
        driver = self.driver()
        target = np.zeros(14)
        target[10] = .3
        self.assertTrue(driver.command_full_urdf(target))
        self.assertTrue(driver.last_command_skipped)
        driver._arm.move_j.assert_not_called()
        driver._arm.last_feedback_timestamp = 10.02
        self.assertTrue(driver.command_full_urdf(target))
        self.assertEqual(driver._trajectory.state.stamp, 10.02)
        np.testing.assert_allclose(driver.last_commanded_full_urdf(), np.zeros(14))
        np.testing.assert_allclose(driver._trajectory.state.velocity, np.zeros(7))
        self.assertTrue(driver.trajectory_diagnostics["first_follow_hold"])
        driver._arm.last_feedback_timestamp = 10.04
        self.assertTrue(driver.command_full_urdf(target))
        self.assertGreater(driver.last_commanded_full_urdf()[10], 0)
        for field in ("ik_urdf_rad", "trajectory_urdf_rad", "command_urdf_rad",
                      "feedback_urdf_rad", "velocity_urdf_rad_s", "lead_rad"):
            self.assertEqual(len(driver.trajectory_diagnostics[field]), 7)
        self.assertIn("feedback_j4_deg_at_send", driver.elbow_command_diagnostics)

    def test_shared_cycle_snapshot_does_not_reread_sdk_both_sides(self):
        for side in ("left", "right"):
            driver = self.driver(side)
            snapshot = JointFeedbackSnapshot(self.current.copy(), 10.02, time.monotonic(), "get_joint_angles")
            driver._arm.runtime_feedback.return_value = snapshot
            driver._arm.get_joint_angles.reset_mock()
            measured = driver.read_control_cycle_joints()
            self.assertTrue(driver.command_full_urdf(measured))
            driver._arm.get_joint_angles.assert_not_called()
            self.assertEqual(driver.trajectory_diagnostics["feedback_snapshot"]["stamp_s"], 10.02)
            self.assertIsNone(driver._cycle_feedback)

    def test_first_robot_link_cycle_seeds_verified_startup_snapshot(self):
        driver = self.driver()
        snapshot = JointFeedbackSnapshot(
            self.current.copy(), 10.02, time.monotonic(), "get_joint_angles"
        )
        driver._arm._runtime_feedback = None

        def seed_startup():
            driver._arm._runtime_feedback = snapshot
            return self.current.copy()

        driver._arm.get_joint_angles.side_effect = seed_startup
        driver._arm.runtime_feedback.return_value = snapshot
        driver._arm.get_joint_angles.reset_mock()
        driver._arm.runtime_feedback.reset_mock()
        measured = driver.read_control_cycle_joints()
        self.assertIsNotNone(measured)
        driver._arm.get_joint_angles.assert_called_once()
        driver._arm.runtime_feedback.assert_called_once()

    def test_long_gap_revalidation_is_limited_to_verified_disabled_arm(self):
        driver = self.driver()
        snapshot = JointFeedbackSnapshot(
            self.current.copy(), 10.02, time.monotonic(), "get_joint_angles"
        )
        driver._arm.runtime_feedback.return_value = snapshot
        driver._arm.get_joint_enable_states.return_value = [False] * 7
        driver._enabled = False
        self.assertIsNotNone(driver.read_control_cycle_joints())
        self.assertTrue(driver._arm.runtime_feedback.call_args.kwargs["allow_long_gap_recovery"])
        driver._arm.move_j.assert_not_called()
        driver._arm.runtime_feedback.reset_mock()
        driver._enabled = True
        self.assertIsNotNone(driver.read_control_cycle_joints())
        self.assertFalse(driver._arm.runtime_feedback.call_args.kwargs["allow_long_gap_recovery"])

    def test_first_robot_link_cycle_keeps_missing_feedback_closed(self):
        driver = self.driver()
        driver._arm._runtime_feedback = None
        driver._arm.get_joint_angles.return_value = None
        driver._arm.get_joint_angles.side_effect = None
        driver._arm.runtime_feedback.reset_mock()
        driver.startup_event_sink = mock.Mock()
        self.assertIsNone(driver.read_control_cycle_joints())
        driver._arm.runtime_feedback.assert_not_called()
        self.assertEqual(
            driver.feedback_failure_diagnostics["reason"],
            "no verified startup joint feedback",
        )
        self.assertEqual(
            driver.startup_event_sink.call_args.args[0]["phase"],
            "runtime_feedback_rejected",
        )

    def test_expired_cycle_snapshot_locks_without_send(self):
        driver = self.driver()
        driver._cycle_feedback = JointFeedbackSnapshot(self.current.copy(), 10.02, time.monotonic() - .2, "get_joint_angles")
        self.assertFalse(driver.command_full_urdf(np.zeros(14)))
        driver._arm.move_j.assert_not_called()
        self.assertIsNone(driver._trajectory.state)

    def test_feedback_failure_is_logged_before_control_loop_skips_diagnostics(self):
        driver = self.driver()
        driver._arm.runtime_feedback.return_value = None
        failure = {
            "reason": "verified SDK feedback source returned None",
            "previous": None,
            "transient": True,
        }
        driver._arm.runtime_feedback_failure = failure
        driver.startup_event_sink = mock.Mock()
        self.assertIsNone(driver.read_control_cycle_joints())
        event = driver.startup_event_sink.call_args.args[0]
        self.assertEqual(event["phase"], "runtime_feedback_rejected")
        self.assertEqual(event["failure"], failure)
        driver._arm.move_j.assert_not_called()
        driver._arm.disable.assert_not_called()

    def test_two_frame_feedback_recovery_restamps_held_target_without_send(self):
        driver = self.driver()
        held = driver._trajectory.state.position.copy()
        snapshot = JointFeedbackSnapshot(self.current.copy(), 10.15, time.monotonic(), "get_joint_angles")
        driver._arm.runtime_feedback.return_value = snapshot
        driver._arm.runtime_feedback_recovered = True
        driver.startup_event_sink = mock.Mock()
        measured = driver.read_control_cycle_joints()
        self.assertIsNotNone(measured)
        driver._arm.move_j.assert_not_called()
        np.testing.assert_allclose(driver._trajectory.state.position, held)
        np.testing.assert_allclose(driver._trajectory.state.velocity, 0)
        self.assertEqual(driver._trajectory.state.stamp, 10.15)
        self.assertTrue(driver._first_follow_hold)
        self.assertTrue(driver.trajectory_diagnostics["feedback_gap_recovered"])
        self.assertEqual(
            driver.startup_event_sink.call_args.args[0]["phase"],
            "runtime_feedback_recovered",
        )

    def test_feedback_recovery_rejects_held_target_outside_lead_without_send(self):
        driver = self.driver()
        driver._trajectory.state.position[0] += driver._trajectory.lead[0] + .01
        snapshot = JointFeedbackSnapshot(self.current.copy(), 10.15, time.monotonic(), "get_joint_angles")
        driver._arm.runtime_feedback.return_value = snapshot
        driver._arm.runtime_feedback_recovered = True
        self.assertIsNone(driver.read_control_cycle_joints())
        driver._arm.move_j.assert_not_called()
        self.assertIsNone(driver._trajectory.state)
        self.assertFalse(driver.feedback_failure_diagnostics["transient"])

    def test_probe_failure_cannot_reject_successful_command(self):
        driver = self.driver()
        driver.probe_sink = mock.Mock(side_effect=RuntimeError("diagnostic unavailable"))
        driver._arm.last_feedback_timestamp = 10.02
        self.assertTrue(driver.command_full_urdf(np.zeros(14)))
        self.assertIsNone(driver.safety_fault_reason)
        driver._arm.move_j.assert_called_once()
        driver.probe_sink.assert_called_once()

    def test_probe_captures_fault_before_next_periodic_sample(self):
        driver = self.driver()
        driver.probe_sink = mock.Mock()
        driver.startup_event_sink = mock.Mock()
        driver._trajectory_fault("test", {"failed_joints": [1]})
        event = driver.probe_sink.call_args.args[0]
        self.assertEqual(event["event"], "trajectory_fault")
        self.assertEqual(event["fault_context"]["failed_joints"], [1])
        exact = driver.startup_event_sink.call_args.args[0]
        self.assertEqual(exact['phase'], 'trajectory_fault')
        self.assertEqual(exact['fault_context']['failed_joints'], [1])
        driver._arm.move_j.assert_not_called()

    def test_fault_event_captures_passive_controller_cache_without_hardware_queries(self):
        driver = self.driver('left')
        driver.startup_event_sink = mock.Mock()
        driver._arm.reset_mock()
        snapshot = {'controller': {'fields': {'ctrl_mode': 1, 'mode_feedback': 1}}, 'motors': []}
        with mock.patch('esrobo_teleop.debug.arm_probe.controller_snapshot', return_value=snapshot) as reader:
            self.assertFalse(driver._trajectory_fault('persistent feedback lead on joints [1]'))
        reader.assert_called_once_with(driver)
        event = driver.startup_event_sink.call_args.args[0]
        self.assertEqual(event['controller_snapshot'], snapshot)
        self.assertEqual(event['configured_firmware'], 'V112')
        driver._arm.move_j.assert_not_called()
        driver._arm.get_joint_angles.assert_not_called()
        driver._arm.controller_fault.assert_not_called()

    def test_initialization_uses_explicit_snapshot_without_read(self):
        driver = self.driver()
        snapshot = JointFeedbackSnapshot(self.current.copy(), 12., time.monotonic(), "get_joint_angles")
        driver._arm.get_joint_angles.reset_mock()
        self.assertTrue(driver.initialize_command_trajectory(snapshot))
        driver._arm.get_joint_angles.assert_not_called()
        self.assertEqual(driver._trajectory.state.stamp, 12.)
        np.testing.assert_allclose(driver._trajectory.state.velocity, 0)

    def test_left_cpv_is_prepared_from_measured_pose_and_used_only_for_following(self):
        cfg = build_config().robot
        cfg.left_tracking_control_mode = "cpv"
        with mock.patch("esrobo_teleop.robot.nero_driver.NeroArm"):
            driver = NeroSingleArmDriver(cfg, "left")
        current = np.asarray(cfg.left_joint_offsets, dtype=float)
        driver._arm.get_joint_angles.side_effect = lambda: current.copy()
        driver._arm.last_feedback_timestamp = 10.0
        driver._arm.controller_fault.return_value = None
        driver._arm.prepare_tracking_control.return_value = True
        driver._arm.move_tracking_positions.return_value = None
        driver.configure_collision_guard(lambda a, b: True)
        driver._enabled = True
        driver._torque_is_safe = mock.Mock(return_value=True)
        driver.configure_command_trajectory(lambda q: linear_fk(q[:7]), .22)
        driver.capture_session_start()
        snapshot = JointFeedbackSnapshot(current, 10.0, time.monotonic(), "get_joint_angles")
        self.assertTrue(driver.initialize_command_trajectory(snapshot))
        args = driver._arm.prepare_tracking_control.call_args.args
        np.testing.assert_array_equal(args[0], current)
        np.testing.assert_array_equal(args[1], cfg.left_max_joint_velocity)
        np.testing.assert_array_equal(args[2], cfg.left_max_joint_acceleration)

        # First fresh feedback sends the measured hold through CPV. MOVE_J is
        # reserved for startup/return code paths outside live following.
        driver._arm.last_feedback_timestamp = 10.02
        self.assertTrue(driver.command_full_urdf(np.zeros(14)))
        driver._arm.move_tracking_positions.assert_called_once()
        driver._arm.move_j.assert_not_called()

    def test_left_cpv_preparation_failure_refuses_following_without_position_send(self):
        cfg = build_config().robot
        cfg.left_tracking_control_mode = "cpv"
        with mock.patch("esrobo_teleop.robot.nero_driver.NeroArm"):
            driver = NeroSingleArmDriver(cfg, "left")
        current = np.asarray(cfg.left_joint_offsets, dtype=float)
        driver._arm.get_joint_angles.side_effect = lambda: current.copy()
        driver._arm.last_feedback_timestamp = 10.0
        driver._arm.controller_fault.return_value = None
        driver._arm.prepare_tracking_control.side_effect = RuntimeError("missing CPV ack")
        driver.configure_collision_guard(lambda a, b: True)
        driver._enabled = True
        driver._torque_is_safe = mock.Mock(return_value=True)
        driver.configure_command_trajectory(lambda q: linear_fk(q[:7]), .22)
        snapshot = JointFeedbackSnapshot(current, 10.0, time.monotonic(), "get_joint_angles")
        self.assertFalse(driver.initialize_command_trajectory(snapshot))
        self.assertIn("tracking control preparation failed", driver.safety_fault_reason)
        driver._arm.move_tracking_positions.assert_not_called()
        driver._arm.move_j.assert_not_called()

    def test_node_missing_runtime_feedback_does_not_reuse_previous_pose(self):
        node = TeleopNode.__new__(TeleopNode)
        node._driver = self.driver()
        node._cfg = build_config()
        node._arm_only = True
        node._arm_side = "right"
        node._arm_armed = True
        node._current_arm_joints = np.zeros(14)
        node._driver._arm.runtime_feedback.return_value = None
        self.assertIsNone(node._read_current_joints())

    def test_node_post_enable_snapshot_initializes_all_stages_both_sides(self):
        for side in ("left", "right"):
            node = TeleopNode.__new__(TeleopNode)
            node._cfg = build_config()
            node._cfg.ik.partition_terminal_wrist_ik = False
            node._wrist_imu_side = None
            node._arm_only = True
            node._arm_side = side
            node._motors_enabled = False
            node._wait_for_arm_prepare = mock.Mock(return_value=True)
            node._body = mock.Mock()
            node._body.refresh_arm_reference_for_enable.return_value = (True, 0., 0., .001, 20)
            node._body.arm_reference_flexion_rad.return_value = .4
            node._driver = mock.Mock()
            node._driver.capture_session_start.return_value = True
            before = np.zeros(14)
            after = np.ones(14) * .001
            node._driver.read_full_urdf_joints.side_effect = [before, after]
            snapshot = JointFeedbackSnapshot(np.zeros(7), 10., time.monotonic(), "get_joint_angles")
            node._driver._arm._runtime_feedback = snapshot
            node._ik = mock.Mock()
            self.assertTrue(node._arm_robot())
            np.testing.assert_array_equal(node._ik.initialize_arm_session.call_args.args[1], after)
            np.testing.assert_array_equal(node._ik.current_task_frame_poses.call_args.args[0], after)
            node._driver.initialize_command_trajectory.assert_called_once_with(snapshot)
            node._body.seed_arm_reference_filters.assert_called_once_with(side)
            node._body.arm_reference_flexion_rad.assert_called_once_with(side)

    def test_send_failure_locks_without_commit_or_disable(self):
        for failure in (False, RuntimeError("CAN send failed")):
            driver = self.driver()
            driver._arm.last_feedback_timestamp = 10.02
            if isinstance(failure, Exception):
                driver._arm.move_j.side_effect = failure
            else:
                driver._arm.move_j.return_value = failure
            self.assertFalse(driver.command_full_urdf(np.zeros(14)))
            self.assertIsNone(driver._trajectory.state)
            self.assertIsNone(driver.last_commanded_full_urdf())
            self.assertEqual(driver._last_command_feedback_timestamp, 10.0)
            driver._arm.disable.assert_not_called()
            self.assertFalse(driver.command_full_urdf(np.zeros(14)))
            self.assertEqual(driver._arm.move_j.call_count, 1)

    def test_infeasible_fault_context_survives_driver_invalidation(self):
        driver = self.driver()
        driver._arm.last_feedback_timestamp = 10.02
        context = {'failed_joints': [3], 'feedback_dt_s': .02}
        with mock.patch.object(driver._trajectory, 'propose',
                               side_effect=TrajectoryFault('infeasible interval', context)):
            self.assertFalse(driver.command_full_urdf(np.zeros(14)))
        self.assertIsNone(driver._trajectory.state)
        self.assertEqual(driver.trajectory_diagnostics['fault_context'], context)
        driver._arm.move_j.assert_not_called()
        driver._arm.disable.assert_not_called()

    def test_missing_feedback_and_torque_fault_invalidate(self):
        for fault in ("feedback", "torque", "gap"):
            driver = self.driver()
            driver._arm.last_feedback_timestamp = 10.02
            if fault == "feedback":
                driver._arm.get_joint_angles.side_effect = lambda: None
            elif fault == "torque":
                driver._torque_is_safe.return_value = False
            else:
                driver._arm.last_feedback_timestamp = 10.2
            self.assertFalse(driver.command_full_urdf(np.zeros(14)))
            self.assertIsNone(driver._trajectory.state)
            driver._arm.move_j.assert_not_called()
            driver._arm.disable.assert_not_called()

    def test_missing_and_bad_fk_refuse_enable_before_hardware_commands(self):
        for fk in (None, lambda q: np.full((2, 3), np.nan)):
            driver = self.driver()
            driver._trajectory_fk = fk
            self.assertFalse(driver.enable())
            driver._arm.enable.assert_not_called()
            driver._arm.set_normal_mode.assert_not_called()
            driver._arm.move_j.assert_not_called()

    def test_disable_estop_return_invalidate_command_state(self):
        for operation in ("disable", "emergency_stop", "return_to_zero_and_disable"):
            driver = self.driver()
            getattr(driver, operation)()
            self.assertIsNone(driver._trajectory.state)
            if operation == "return_to_zero_and_disable":
                driver._arm.move_j.assert_not_called()

    def test_reenable_initialization_drops_old_velocity_and_timestamp(self):
        driver = self.driver()
        driver._arm.last_feedback_timestamp = 10.02
        driver.command_full_urdf(np.ones(14)*.1)
        driver._arm.last_feedback_timestamp = 30.0
        self.assertTrue(driver.initialize_command_trajectory())
        np.testing.assert_array_equal(driver._trajectory.state.velocity, 0)
        np.testing.assert_array_equal(driver._trajectory.state.position, self.current)
        self.assertEqual(driver._trajectory.state.stamp, 30)

    def test_left_disabled_mode_uses_old_clamp(self):
        driver = self.driver("left", trajectory_enabled=False)
        self.assertFalse(driver.command_trajectory_enabled)
        driver._clamp = mock.Mock(return_value=self.current.copy())
        driver._arm.last_feedback_timestamp = 10.02
        self.assertTrue(driver.command_full_urdf(np.zeros(14)))
        driver._clamp.assert_called_once()

    def test_both_sides_track_with_own_mapping_limits_and_real_fk(self):
        cfg = build_config()
        cfg.ik.urdf_path = str(Path(__file__).resolve().parents[1] / "urdf/esrobo_waist_with_head.urdf")
        solver = IkSolver(cfg.ik)
        for side, start in (("left", 0), ("right", 7)):
            with self.subTest(side=side):
                driver = self.driver(side)
                # Exercise the continuous trajectory math independently of
                # the left controller's point-to-point submission gate.
                driver._move_j_waypoint_gate = None
                self.assertTrue(driver.command_trajectory_enabled)
                fk = solver.make_command_fk(side)
                driver.configure_command_trajectory(fk, .22)
                directions, offsets = driver._mapping(side)
                lower, upper = driver._effective_position_limits(side)
                for i in range(240):
                    before = driver._trajectory.state
                    dt = (.015, .02, .025)[i % 3]
                    driver._arm.last_feedback_timestamp = before.stamp + dt
                    target = np.zeros(14)
                    target[start:start+7] = [0.05, -0.04, .12 if side == "left" else -.12,
                                             .25 if i < 100 else .05, 0, 0, 0]
                    feedback = self.current.copy()
                    self.assertTrue(driver.command_full_urdf(target), driver.safety_fault_reason)
                    step = driver._trajectory.state
                    def physical_fk(q):
                        full = np.zeros(14)
                        full[start:start+7] = (q-offsets)/directions
                        return fk(full)
                    TrajectoryTests.assert_limits(self, driver._trajectory, before, step, feedback, dt, physical_fk)
                    self.assertTrue(np.all(step.position >= lower-1e-8))
                    self.assertTrue(np.all(step.position <= upper+1e-8))
                    full = driver.last_commanded_full_urdf()
                    np.testing.assert_allclose(full[start:start+7], (step.position-offsets)/directions)
                    np.testing.assert_array_equal(full[7:] if side == "left" else full[:7], 0)
                    self.current += dt/(.1+dt)*(step.position-self.current)
                self.assertLess(abs(driver.last_commanded_full_urdf()[start+3]-.05), .005)

    def test_both_sides_fault_and_reinitialize_without_old_targets(self):
        for side in ("left", "right"):
            for fault in ("send", "stale", "torque"):
                with self.subTest(side=side, fault=fault):
                    driver = self.driver(side)
                    # This test isolates generic send/stale/torque fault
                    # handling. Left stop-and-wait submission is covered in
                    # test_command_backpressure.py.
                    driver._move_j_waypoint_gate = None
                    driver._arm.last_feedback_timestamp = 10.02
                    self.assertTrue(driver.command_full_urdf(np.zeros(14)))
                    last = driver.last_commanded_full_urdf()
                    driver._arm.last_feedback_timestamp = 10.04
                    next_target = np.zeros(14)
                    if fault == "send":
                        driver._arm.move_j.side_effect = RuntimeError("test send failure")
                        # A held, identical wire target now needs no send.
                        next_target[0 if side == "left" else 7] = .05
                    elif fault == "stale":
                        driver._arm.last_feedback_timestamp = 11.0
                    else:
                        driver._torque_is_safe.return_value = False
                    self.assertFalse(driver.command_full_urdf(next_target))
                    self.assertIsNone(driver._trajectory.state)
                    np.testing.assert_array_equal(driver.last_commanded_full_urdf(), last)
                    driver._arm.disable.assert_not_called()
                    # Simulate successful explicit recovery checks, without real enable.
                    driver._safety_fault_reason = None
                    driver._arm.move_j.side_effect = None
                    driver._torque_is_safe.return_value = True
                    driver._arm.last_feedback_timestamp = 20.0
                    self.assertTrue(driver.initialize_command_trajectory())
                    np.testing.assert_array_equal(driver._trajectory.state.velocity, 0)
                    np.testing.assert_array_equal(driver._trajectory.state.position, self.current)

    def test_node_fault_locks_and_repeat_e_cannot_resume(self):
        node = TeleopNode.__new__(TeleopNode)
        node._cfg = build_config()
        node._arm_only = True
        node._arm_side = "right"
        node._arm_armed = True
        node._driver = self.driver()
        node._stop_full_arm_for_fault("body data stale")
        self.assertFalse(node._arm_armed)
        self.assertTrue(node._manual_intervention_required)
        self.assertFalse(node._arm_robot())
        node._driver._arm.move_j.assert_not_called()
        node._driver._arm.disable.assert_not_called()

    def test_repeat_e_when_armed_does_not_reinitialize_trajectory(self):
        node = TeleopNode.__new__(TeleopNode)
        node._cfg = build_config()
        node._cfg.retarget.require_hand_imu_for_active_arm = False
        node._arm_only = True
        node._arm_side = "right"
        node._wrist_imu_side = None
        node._arm_armed = True
        node._driver = self.driver()
        before = node._driver._trajectory.state
        self.assertTrue(node._arm_robot())
        self.assertIs(node._driver._trajectory.state, before)
        node._driver._arm.enable.assert_not_called()


class AsyncLogTests(unittest.TestCase):
    def test_numpy_fields_keep_types_and_malformed_record_does_not_stop_writer(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'log.jsonl'
            writer = AsyncJsonLog(path)
            writer.submit({'recovery': np.bool_(True), 'nested': {'count': np.int64(2)},
                           'values': np.array([1., 2.], dtype=np.float32)})
            writer.submit({'invalid': object()})
            writer.submit({'invalid': np.float64(float('nan'))})
            writer.submit({'phase': 'trajectory_fault', 'clearance_m': np.float32(.03)})
            writer.close()
            rows = [json.loads(line) for line in path.read_text().splitlines()]
            self.assertEqual(rows[0], {'recovery': True, 'nested': {'count': 2}, 'values': [1., 2.]})
            self.assertEqual(rows[1]['phase'], 'trajectory_fault')
            self.assertEqual(writer.serialization_errors, 2)
            self.assertEqual(writer.dropped, 2)
            self.assertIsNone(writer.error)

    def test_queue_is_bounded_and_submit_does_not_wait_for_writer(self):
        release = threading.Event()
        entered = threading.Event()
        with tempfile.TemporaryDirectory() as directory:
            def blocked_run(writer):
                entered.set()
                release.wait(2)
            with mock.patch.object(AsyncJsonLog, "_run", blocked_run):
                writer = AsyncJsonLog(Path(directory)/"log.jsonl", capacity=2)
                self.assertTrue(entered.wait(1))
                self.assertTrue(writer.submit({"i": 1}))
                self.assertTrue(writer.submit({"i": 2}))
                self.assertFalse(writer.submit({"i": 3}))
                self.assertEqual(writer.dropped, 1)
                self.assertEqual(writer.queue.qsize(), 2)
                release.set()
                writer.close()

    def test_flush_on_close_and_write_error_is_nonfatal(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"log.jsonl"
            writer = AsyncJsonLog(path)
            writer.submit({"i": 1})
            writer.close()
            self.assertEqual(json.loads(path.read_text()), {"i": 1})
            writer = AsyncJsonLog(Path(directory))
            writer.close()
            self.assertIsNotNone(writer.error)


if __name__ == "__main__":
    unittest.main()
