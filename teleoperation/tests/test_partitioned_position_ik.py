"""Continuous URDF and recorded-session regressions for partitioned arm IK."""
import contextlib
import io
import json
from pathlib import Path
import unittest
from types import SimpleNamespace
from unittest import mock

import numpy as np

from esrobo_teleop.config import build_config
from esrobo_teleop.ik.solver import IkSolver
from esrobo_teleop.robot.command_trajectory import CommandTrajectory
from esrobo_teleop.robot.torso_collision import TorsoCollisionGuard
from esrobo_teleop.teleop_node import TeleopNode


ROOT = Path(__file__).resolve().parents[1]


class PartitionedPositionTests(unittest.TestCase):
    def test_latest_right_j2_boundary_rejection_is_a_recoverable_hold(self):
        cfg, solver, _, _ = self.solver("right")
        feedback = np.array([
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            1.0114881147007937, 0.12201596800692371,
            0.9923069462213762, 1.0462550734005207,
            0.004886921905584123, -0.024434609527920616,
            -0.10971139678036357,
        ])
        poses = solver.current_task_frame_poses(feedback)
        poses["right_elbow"][:3] = [
            0.10652600228786469, -0.16696295142173767, 1.0273070335388184,
        ]
        poses["right_wrist"][:3] = [
            0.35931146144866943, 0.1045408844947815, 1.174302577972412,
        ]
        solver.initialize_arm_session("right", feedback, .4,
                                      relative_elbow_reference=True)
        with contextlib.redirect_stdout(io.StringIO()):
            held = solver.solve(
                poses["left_wrist"], poses["right_wrist"], feedback,
                poses["left_elbow"], poses["right_elbow"], "right", feedback[11:],
            )
        self.assertFalse(solver.last_solution_valid)
        self.assertTrue(solver.solution_diagnostics["recoverable_position_limit"])
        self.assertEqual(solver.solution_diagnostics["active_position_bounds"][0]["joint"], 2)
        self.assertEqual(solver.solution_diagnostics["active_position_bounds"][0]["bound"], "upper")
        self.assertTrue(solver.solution_diagnostics["active_position_bounds"][0]["feedback_at_bound"])
        np.testing.assert_array_equal(held, feedback)

        # The same unreachable target is not silently treated as a hold before
        # measured feedback has actually reached the limiting joint boundary.
        away = feedback.copy()
        away[8] = np.deg2rad(5.0)
        solver.initialize_arm_session("right", away, .4,
                                      relative_elbow_reference=True)
        with contextlib.redirect_stdout(io.StringIO()):
            solver.solve(
                poses["left_wrist"], poses["right_wrist"], away,
                poses["left_elbow"], poses["right_elbow"], "right", away[11:],
            )
        self.assertFalse(solver.last_solution_valid)
        self.assertFalse(solver.solution_diagnostics["recoverable_position_limit"])

    def test_latest_left_session_uses_shared_ik_without_geometry_failure(self):
        session = json.loads((ROOT / 'tests/fixtures/left_feedback_stall_20260920.json').read_text())
        cfg, solver, lo, hi = self.solver('left')
        startup = np.zeros(14)
        startup[:7] = np.array(session['startup_physical_rad']) - cfg.robot.left_joint_offsets
        solver.initialize_arm_session('left', startup, np.deg2rad(session['human_reference_deg']),
                                      relative_elbow_reference=True)
        first_feedback = np.array(session['frames'][0]['feedback_deg'])
        for frame in session['frames']:
            np.testing.assert_array_equal(frame['feedback_deg'], first_feedback)
            q = np.zeros(14)
            q[:7] = np.deg2rad(frame['feedback_deg'])
            poses = solver.current_task_frame_poses(q)
            for key, value in frame['target'].items():
                poses[key][:3] = value
            result = self.solve(solver, 'left', q, poses, q[4:7])
            self.assertLess(solver.solution_diagnostics['wrist_error_m'], .002)
            self.assertTrue(np.all(result[:7] >= lo-1e-8))
            self.assertTrue(np.all(result[:7] <= hi+1e-8))

    def test_latest_limit_frame_with_old_and_expanded_envelopes(self):
        session = json.loads((ROOT / "tests/fixtures/right_ik_sequences_20260920.json").read_text())[-1]
        event = session['rejection']
        q = np.array(event['current_urdf_rad'])
        for upper in (85., 120.):
            with self.subTest(upper=upper):
                _, solver, lo, hi = self.solver('right')
                hi[3] = np.deg2rad(upper)
                solver.configure_position_envelope('right', lo, hi)
                poses = solver.current_task_frame_poses(q)
                for key, value in event['retarget_positions_m'].items():
                    poses[key][:3] = value
                result = self.solve(solver, 'right', q, poses, q[11:])
                self.assertEqual(solver.solution_diagnostics['workspace_limited'], upper == 85.)
                self.assertLess(solver.solution_diagnostics['wrist_error_m'], .001)
                self.assertLessEqual(result[10], np.deg2rad(upper))
                if upper == 120.:
                    self.assertAlmostEqual(np.degrees(result[10]), 90.57, places=1)

    def test_out_of_range_bend_saturates_then_recovers_with_wrist_offsets(self):
        for side in ('left', 'right'):
            offset = 0 if side == 'left' else 7
            for wrist in ((1.5, .7, -4.1), (-5., -8., 15.)):
                with self.subTest(side=side, wrist=wrist):
                    _, solver, lo, hi = self.solver(side)
                    q = np.zeros(14)
                    q[offset:offset+7] = np.deg2rad([10, -20, 20, 60, *wrist])
                    solver.initialize_arm_session(side, q, .4, relative_elbow_reference=True)
                    poses = solver.current_task_frame_poses(q)
                    elbow = poses[f'{side}_elbow'][:3].astype(float)
                    upper = elbow - poses[f'{side}_shoulder'][:3]
                    upper /= np.linalg.norm(upper)
                    forearm = poses[f'{side}_wrist'][:3].astype(float) - elbow
                    plane = forearm - upper*np.dot(upper, forearm)
                    plane /= np.linalg.norm(plane)
                    for bend in list(np.linspace(65., 145., 35)) + [145.]*15 + list(np.linspace(145., 65., 35)):
                        poses[f'{side}_wrist'][:3] = elbow + np.linalg.norm(forearm)*(
                            upper*np.cos(np.deg2rad(bend)) + plane*np.sin(np.deg2rad(bend)))
                        result = self.solve(solver, side, q, poses, q[offset+4:offset+7])
                        self.assertLess(solver.solution_diagnostics['wrist_error_m'], .003)
                        self.assertTrue(np.all(result[offset:offset+7] >= lo-1e-8))
                        self.assertTrue(np.all(result[offset:offset+7] <= hi+1e-8))
                        if bend == 145.:
                            self.assertTrue(solver.solution_diagnostics['workspace_limited'])
                            self.assertLess(abs(result[offset+3]-hi[3]), np.deg2rad(.1))
                        q += .15*(result-q)
                    self.assertFalse(solver.solution_diagnostics['workspace_limited'])

    def test_workspace_notice_is_not_a_fault_and_only_reports_transitions(self):
        node = TeleopNode.__new__(TeleopNode)
        node._arm_armed = node._arm_only = True
        node._arm_side = 'right'
        node._ik = SimpleNamespace(solution_diagnostics=dict(workspace_limited=True,
            requested_bend_deg=145., bend_limit_deg=118., j4_upper_limit_deg=120.))
        node._record_startup_event = mock.Mock()
        node._stop_full_arm_for_fault = mock.Mock()
        with contextlib.redirect_stdout(io.StringIO()):
            node._report_workspace_limit()
            node._report_workspace_limit()
            node._ik.solution_diagnostics['workspace_limited'] = False
            node._report_workspace_limit()
        self.assertEqual([c.args[0]['phase'] for c in node._record_startup_event.call_args_list],
                         ['ik_workspace_limited', 'ik_workspace_recovered'])
        node._stop_full_arm_for_fault.assert_not_called()

    def test_position_limit_hold_reports_only_pause_and_recovery_transitions(self):
        node = TeleopNode.__new__(TeleopNode)
        node._arm_side = "right"
        node._position_limit_hold_active = False
        node._ik = SimpleNamespace(solution_diagnostics={
            "recoverable_position_limit": True,
            "active_position_bounds": [{
                "joint": 2, "bound": "upper", "limit_rad": np.deg2rad(7.0),
            }],
        })
        node._record_startup_event = mock.Mock()
        with contextlib.redirect_stdout(io.StringIO()) as output:
            node._report_position_limit_hold(True)
            node._report_position_limit_hold(True)
            node._ik.solution_diagnostics = {"recoverable_position_limit": False}
            node._report_position_limit_hold(False)
        self.assertIn("TARGET PAUSED", output.getvalue())
        self.assertIn("following resumed", output.getvalue())
        self.assertEqual(
            [call.args[0]["phase"] for call in node._record_startup_event.call_args_list],
            ["ik_position_limit_hold", "ik_position_limit_recovered"],
        )

    def test_position_limit_recovery_requires_three_frames_inside_hysteresis(self):
        node = TeleopNode.__new__(TeleopNode)
        node._cfg = build_config()
        node._position_limit_hold_active = True
        node._position_limit_recovery_count = 0
        inside = {"elbow_error_m": .024, "wrist_error_m": .018}
        outside = {"elbow_error_m": .026, "wrist_error_m": .018}

        self.assertTrue(node._position_limit_recovery_pending(inside))
        self.assertTrue(node._position_limit_recovery_pending(outside))
        self.assertEqual(node._position_limit_recovery_count, 0)
        self.assertTrue(node._position_limit_recovery_pending(inside))
        self.assertTrue(node._position_limit_recovery_pending(inside))
        self.assertFalse(node._position_limit_recovery_pending(inside))

    def solver(self, side):
        cfg = build_config()
        cfg.ik.urdf_path = str(ROOT / "urdf/esrobo_waist_with_head.urdf")
        solver = IkSolver(cfg.ik)
        lo = np.array(getattr(cfg.robot, f"{side}_joint_soft_lower_limits_urdf"))
        hi = np.array(getattr(cfg.robot, f"{side}_joint_soft_upper_limits_urdf"))
        solver.configure_position_envelope(side, lo, hi)
        return cfg, solver, lo, hi

    def solve(self, solver, side, feedback, poses, terminal):
        with contextlib.redirect_stdout(io.StringIO()):
            result = solver.solve(poses["left_wrist"], poses["right_wrist"], feedback,
                                  poses["left_elbow"], poses["right_elbow"], side, terminal)
        self.assertTrue(solver.last_solution_valid, solver.solution_diagnostics)
        return result

    def test_complete_recorded_following_sequences_and_exact_failure_frame(self):
        sessions = json.loads((ROOT / "tests/fixtures/right_ik_sequences_20260920.json").read_text())
        for session in sessions:
            with self.subTest(source=session["source"]):
                cfg, solver, lo, hi = self.solver("right")
                startup = np.zeros(14)
                startup[7:] = (np.array(session["startup_physical_rad"])
                               - np.array(cfg.robot.right_joint_offsets))
                solver.initialize_arm_session("right", startup,
                                              np.deg2rad(session["human_reference_deg"]),
                                              relative_elbow_reference=True)
                frames = list(session["frames"])
                failure = session["rejection"]
                if failure:
                    frames.append(dict(feedback_deg=np.degrees(failure["current_urdf_rad"][7:]),
                                       target=failure["retarget_positions_m"]))
                for frame in frames:
                    feedback = np.zeros(14)
                    feedback[7:] = np.deg2rad(frame["feedback_deg"])
                    poses = solver.current_task_frame_poses(feedback)
                    for key, value in frame["target"].items():
                        poses[key][:3] = value
                    result = self.solve(solver, "right", feedback, poses, feedback[11:])
                    # The larger Sept 20 motion also reaches other workspace
                    # boundaries; require the real 30 mm acceptance envelope.
                    tolerance = .03 if session['source'] == '1789915667901387569' else .002
                    self.assertLess(solver.solution_diagnostics["elbow_error_m"], tolerance)
                    self.assertLess(solver.solution_diagnostics["wrist_error_m"], tolerance)
                    self.assertTrue(np.all(result[7:] >= lo - 1e-9))
                    self.assertTrue(np.all(result[7:] <= hi + 1e-9))
                    np.testing.assert_allclose(result[11:], feedback[11:], atol=0.)

    def test_both_arms_extend_bend_rotate_and_return_with_wrist_offsets(self):
        for side in ("left", "right"):
            offset = 0 if side == "left" else 7
            for wrist in ((1.5, .7, -4.1), (-5., -8., 15.)):
                with self.subTest(side=side, wrist=wrist):
                    _, solver, lo, hi = self.solver(side)
                    initial = np.zeros(14)
                    initial[offset:offset + 7] = np.deg2rad([.8, -2., .9, -1.4, *wrist])
                    solver.initialize_arm_session(side, initial, np.deg2rad(40.),
                                                  relative_elbow_reference=True)
                    feedback = initial.copy()
                    desired = initial.copy()
                    for step in range(201):
                        t = step / 200.
                        bend = np.sin(np.pi * t)**2
                        desired[offset] = initial[offset] + np.deg2rad(8. * bend)
                        desired[offset + 1] = initial[offset + 1] - np.deg2rad(15. * bend)
                        desired[offset + 2] = initial[offset + 2] + np.deg2rad(50. * bend**4)
                        desired[offset + 3] = initial[offset + 3] + np.deg2rad(70. * bend)
                        poses = solver.current_task_frame_poses(desired)
                        result = self.solve(solver, side, feedback, poses,
                                            desired[offset + 4:offset + 7])
                        self.assertLess(solver.solution_diagnostics["elbow_error_m"], .003)
                        self.assertLess(solver.solution_diagnostics["wrist_error_m"], .003)
                        self.assertTrue(np.all(result[offset:offset + 7] >= lo - 1e-9))
                        self.assertTrue(np.all(result[offset:offset + 7] <= hi + 1e-9))
                        feedback += .15 * (result - feedback)
                    self.assertLess(np.max(np.abs(result[offset:offset + 4] -
                                                   initial[offset:offset + 4])), np.deg2rad(.5))

    def test_latest_targets_through_trajectory_collision_and_lagging_feedback(self):
        session = json.loads((ROOT / "tests/fixtures/right_ik_sequences_20260920.json").read_text())[0]
        self.simulate_targets(session, collision=True)

    def test_expanded_j4_session_through_live_trajectory_and_lagging_feedback(self):
        session = json.loads((ROOT / "tests/fixtures/right_ik_sequences_20260920.json").read_text())[-1]
        self.simulate_targets(session, collision=False)

    def simulate_targets(self, session, collision):
        cfg, solver, lo, hi = self.solver("right")
        initial = np.zeros(14)
        initial[7:] = (np.array(session["startup_physical_rad"])
                       - np.array(cfg.robot.right_joint_offsets))
        solver.initialize_arm_session("right", initial, np.deg2rad(session["human_reference_deg"]),
                                      relative_elbow_reference=True)
        trajectory = CommandTrajectory(cfg.robot.right_max_joint_velocity,
                                       cfg.robot.right_max_joint_acceleration,
                                       cfg.robot.right_max_joint_step,
                                       np.deg2rad(cfg.robot.command_trajectory_lead_deg))
        trajectory.reset(initial[7:], 10.)
        guard = TorsoCollisionGuard(solver, "right", include_fingers=False) if collision else None
        fk_full = solver.make_command_fk("right")

        def full(q):
            value = np.zeros(14)
            value[7:] = q
            return value

        fk = lambda q: fk_full(full(q))
        check = (lambda a, b: guard(full(a), full(b))) if collision else None
        feedback = initial.copy()
        stamp = 10.
        # Logs sample at roughly 10 Hz. Interpolate them over five 20 ms
        # simulated cycles; this is a feedback simulation, not raw CAN replay.
        previous = solver.current_task_frame_poses(initial)
        for frame in session["frames"]:
            for part in range(1, 6):
                poses = {key: value.copy() for key, value in previous.items()}
                for key, point in frame["target"].items():
                    poses[key][:3] = previous[key][:3] + (np.array(point) - previous[key][:3]) * part / 5.
                target = self.solve(solver, "right", feedback, poses, initial[11:])
                stamp += .02
                before = trajectory.state
                proposal = trajectory.propose(target[7:], feedback[7:], stamp, lo, hi, fk, .22,
                                              path_check=check, coordinate_elbow=True)
                self.assertLessEqual(np.max(np.abs(proposal.position - feedback[7:]) /
                                             trajectory.lead), 1. + 1e-8)
                self.assertTrue(np.all(np.abs(proposal.velocity - before.velocity) <= trajectory.amax * .02 + 1e-8))
                trajectory.commit(proposal)
                feedback[7:] += .02 / .12 * (proposal.position - feedback[7:])
            previous = poses

    def test_unreachable_target_preserves_last_valid_seed_and_rejects(self):
        _, solver, _, _ = self.solver("right")
        q = np.zeros(14)
        solver.initialize_arm_session("right", q, .4, relative_elbow_reference=True)
        poses = solver.current_task_frame_poses(q)
        poses["right_wrist"][2] -= .4
        with contextlib.redirect_stdout(io.StringIO()):
            result = solver.solve(poses["left_wrist"], poses["right_wrist"], q,
                                  poses["left_elbow"], poses["right_elbow"], "right", q[11:])
        self.assertFalse(solver.last_solution_valid)
        np.testing.assert_array_equal(result, q)
        np.testing.assert_array_equal(solver._prev_targets, q)


if __name__ == "__main__":
    unittest.main()
