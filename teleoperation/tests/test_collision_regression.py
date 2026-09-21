"""Offline regressions from the 2026-09-18 incident; no SDK/CAN construction."""
import json
from pathlib import Path
from types import SimpleNamespace
import time
import unittest
from unittest import mock

import numpy as np

from esrobo_teleop.config import build_config
from esrobo_teleop.ik.solver import IkSolver
from esrobo_teleop.robot.command_trajectory import CommandTrajectory, TrajectoryStep, TrajectoryFault
from esrobo_teleop.robot.nero_driver import NeroArm, NeroSingleArmDriver
import test_command_trajectory as trajectory_tests
from test_command_trajectory import linear_fk, planner

ROOT = Path(__file__).resolve().parents[1]
FIXTURE = json.loads((ROOT / 'tests/fixtures/arm_collision_20260918.json').read_text())


class IncidentTests(unittest.TestCase):
    def test_right_recorded_reserve_conflict_brakes_inside_all_hard_limits(self):
        c = FIXTURE['right_fault']
        cfg = build_config().robot
        p = planner()
        q, v, feedback = (np.array(c[k]) for k in (
            'previous_command_physical_rad', 'previous_velocity_physical_rad_s', 'feedback_physical_rad'))
        p.state = TrajectoryStep(q, v, c['previous_command_stamp_s'], np.full(7, np.nan), {})
        ik_cfg = build_config().ik
        ik_cfg.urdf_path = str(ROOT / 'urdf/esrobo_waist_with_head.urdf')
        fk14 = IkSolver(ik_cfg).make_command_fk('right')
        def fk(physical):
            full = np.zeros(14)
            full[7:] = physical - cfg.right_joint_offsets
            return fk14(full)
        step = p.propose(c['target_physical_rad'], feedback, c['feedback_stamp_s'],
                         c['position_lower_rad'], c['position_upper_rad'], fk, .22)
        self.assertTrue(step.diagnostics['braking_recovery'])
        self.assertTrue(np.all(abs(step.velocity) <= abs(v) + 1e-9))
        self.assertTrue(np.all(abs(step.velocity-v) <= p.amax*c['feedback_dt_s'] + 1e-9))
        self.assertTrue(np.all(abs(step.position-feedback) <= p.lead + 1e-9))
        self.assertTrue(np.all(np.linalg.norm(fk(step.position)-fk(q), axis=1) <= .22*c['feedback_dt_s']))
        self.assertIs(p.state.position, q)
        # Continued large feedback reversal cannot be hidden by recovery.
        p.commit(step)
        with self.assertRaises(TrajectoryFault):
            p.propose(c['target_physical_rad'], feedback - .2, step.stamp+.02,
                      c['position_lower_rad'], c['position_upper_rad'], fk, .22)

    def test_left_recorded_outward_targets_keep_their_geometry(self):
        cfg = build_config().ik
        cfg.urdf_path = str(ROOT / 'urdf/esrobo_waist_with_head.urdf')
        s = IkSolver(cfg)
        robot_cfg = build_config().robot
        s.configure_position_envelope('left', robot_cfg.left_joint_soft_lower_limits_urdf,
                                      robot_cfg.left_joint_soft_upper_limits_urdf)
        for row in FIXTURE['left']:
            measured = np.zeros(14)
            measured[:7] = np.deg2rad(row['joints_deg']['feedback'])
            s.initialize_arm_session('left', np.zeros(14), np.deg2rad(25.42), relative_elbow_reference=True)
            poses = s.current_task_frame_poses(measured)
            for name in ('left_elbow', 'left_wrist'):
                poses[name][:3] = row['frames']['retarget'][name]['pos']
            target = s.solve(poses['left_wrist'], poses['right_wrist'], measured,
                             poses['left_elbow'], poses['right_elbow'], 'left', measured[4:7])
            self.assertTrue(s.last_solution_valid, s.solution_diagnostics)
            actual = s.current_task_frame_poses(target)
            self.assertLess(np.linalg.norm(actual['left_wrist'][:3]-poses['left_wrist'][:3]), .005)
            self.assertLess(target[2], np.deg2rad(-90))

    def test_both_bend_directions_wait_for_measured_j3_then_flex(self):
        for direction in (-1, 1):
            p = planner()
            feedback = np.zeros(7)
            target = np.array([0., 0., direction*1.0, .6, 0., 0., 0.])
            for i in range(700):
                before = p.state
                step = p.propose(target, feedback, 10+(i+1)*.02,
                                 -np.ones(7)*2, np.ones(7)*2, linear_fk, .22, coordinate_elbow=True)
                if abs(target[2]-feedback[2]) > np.deg2rad(10):
                    self.assertLess(abs(step.position[3]), 1e-8)
                self.assertTrue(np.all(abs(step.velocity-before.velocity) <= p.amax*.02+1e-8))
                p.commit(step)
                feedback += .02/.12*(step.position-feedback)
            self.assertLess(np.linalg.norm(feedback-target), .01)

    def test_obstructed_braking_path_never_commits(self):
        p = planner()
        before = p.state
        with self.assertRaises(TrajectoryFault):
            p.propose(np.ones(7)*.3, np.zeros(7), 10.02, -np.ones(7), np.ones(7),
                      linear_fk, .22, path_check=lambda a, b: False)
        self.assertIs(p.state, before)


class ControllerStatusTests(unittest.TestCase):
    def arm(self):
        arm = NeroArm.__new__(NeroArm)
        arm._cfg = build_config().robot
        arm._robot = mock.Mock()
        arm._robot.get_arm_status.return_value = SimpleNamespace(timestamp=time.time(),
            msg=SimpleNamespace(arm_status=0, err_code=0, ctrl_mode=1))
        flags = SimpleNamespace(driver_enable_status=True, voltage_too_low=False,
            motor_overheating=False, driver_overcurrent=False, driver_overheating=False,
            collision_status=False, driver_error_status=False, stall_status=False)
        arm._robot.get_driver_states.return_value = SimpleNamespace(timestamp=time.time(),
            msg=SimpleNamespace(foc_status=flags))
        return arm

    def test_collision_and_stale_states_fail_closed(self):
        arm = self.arm()
        self.assertIsNone(arm.controller_fault())
        arm._robot.get_driver_states.return_value.msg.foc_status.collision_status = True
        self.assertEqual(arm.controller_fault()['category'], 'driver_fault')
        arm._robot.get_arm_status.return_value.timestamp -= 1
        self.assertEqual(arm.controller_fault()['category'], 'controller_feedback_stale')

    def test_disabled_brake_code_only_allowed_before_enable(self):
        arm = self.arm()
        arm._robot.get_arm_status.return_value.msg.arm_status = 6
        arm._robot.get_driver_states.return_value.msg.foc_status.driver_enable_status = False
        self.assertIsNone(arm.controller_fault(allow_disabled=True))
        self.assertEqual(arm.controller_fault()['category'], 'controller_fault')

    def test_estop_can_be_ignored_only_to_inspect_disabled_driver_health(self):
        arm = self.arm()
        arm._robot.get_arm_status.return_value.msg.arm_status = 1
        arm._robot.get_driver_states.return_value.msg.foc_status.driver_enable_status = False
        self.assertEqual(arm.controller_fault(allow_disabled=True)['status_name'],
                         'EMERGENCY_STOP')
        self.assertIsNone(arm.controller_fault(allow_disabled=True, allow_estop=True))
        arm._robot.get_driver_states.return_value.msg.foc_status.stall_status = True
        fault = arm.controller_fault(allow_disabled=True, allow_estop=True)
        self.assertEqual(fault['category'], 'driver_fault')
        self.assertEqual(fault['flags'], ['stall_status'])

    def test_torque_fault_blocks_return_even_after_torque_falls(self):
        harness = trajectory_tests.DriverTrajectoryTests()
        d = harness.driver('left')
        d._torque_is_safe.return_value = False
        self.assertFalse(d.command_full_urdf(np.zeros(14)))
        d._torque_is_safe.return_value = True
        self.assertEqual(d.return_to_zero_and_disable(), (False, False))
        d._arm.move_j.assert_not_called()
        d._arm.disable.assert_not_called()

    def test_controller_collision_blocks_follow_and_exit_return(self):
        harness = trajectory_tests.DriverTrajectoryTests()
        for side in ('left', 'right'):
            d = harness.driver(side)
            d._arm.controller_fault.return_value = dict(category='controller_fault', arm_status=7)
            self.assertFalse(d.command_full_urdf(np.zeros(14)))
            self.assertEqual(d.return_to_zero_and_disable(), (False, False))
            d._arm.move_j.assert_not_called()
            d._arm.disable.assert_not_called()

    def test_missing_collision_guard_prevents_enable(self):
        d = trajectory_tests.DriverTrajectoryTests().driver('left')
        d._collision_guard = None
        self.assertFalse(d.enable())
        d._arm.enable.assert_not_called()
        d._arm.move_j.assert_not_called()


class GeometryGuardTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from esrobo_teleop.robot.torso_collision import TorsoCollisionGuard
        cfg = build_config().ik
        cfg.urdf_path = str(ROOT / 'urdf/esrobo_waist_with_head.urdf')
        cls.solver = IkSolver(cfg)
        cls.guards = {side: TorsoCollisionGuard(cls.solver, side) for side in ('left', 'right')}

    def test_recorded_collision_poses_are_rejected_before_contact(self):
        g = self.guards['left']
        for row in FIXTURE['left']:
            q = np.zeros(14)
            q[:7] = np.deg2rad(row['joints_deg']['feedback'])
            self.assertFalse(g(q, q), row['line'])
            self.assertIsNotNone(g.last_diagnostics['pair'])

    def test_all_finger_postures_require_clearance_at_startup(self):
        for side, offset in (('left', 0), ('right', 7)):
            g = self.guards[side]
            zero = np.zeros(14)
            # Zero cannot certify *every* allowed finger pose. Do not silently
            # assume open fingers merely because only the arm is controlled.
            self.assertFalse(g(zero, zero))
            separated = zero.copy()
            separated[offset+1] = np.deg2rad(-5)
            self.assertTrue(g(separated, separated), g.last_diagnostics)
            nearby = separated.copy()
            nearby[offset+2] = np.deg2rad(.05)
            self.assertTrue(g(separated, nearby), g.last_diagnostics)

    def test_arm_only_profile_keeps_arm_wrist_and_palm_checks_without_finger_feedback(self):
        from esrobo_teleop.robot.torso_collision import TorsoCollisionGuard
        for side, offset in (("left", 0), ("right", 7)):
            guard = TorsoCollisionGuard(self.solver, side, include_fingers=False)
            zero = np.zeros(14)
            self.assertTrue(guard(zero, zero), guard.last_diagnostics)
            self.assertEqual(guard.last_diagnostics["collision_profile"], "arm_wrist_palm")
            self.assertEqual(guard.fingers, [])

        # Removing finger articulation must not remove rigid wrist/hand torso
        # protection. This is a logged left-arm pose immediately before the
        # controller collision report; its palm body is inside the 30 mm
        # software clearance even without any articulated-finger geometry.
        left_guard = TorsoCollisionGuard(self.solver, "left", include_fingers=False)
        q = np.zeros(14)
        q[:7] = np.deg2rad([
            2.27, -12.645, -25.203, 28.305, 0.001, -0.092, -0.162,
        ])
        self.assertFalse(left_guard(q, q), left_guard.last_diagnostics)
        self.assertEqual(
            left_guard.last_diagnostics["pair"],
            ["leftHand_link_0", "waist_link1_0"],
        )

    def test_latest_right_measured_pose_passes_arm_only_profile(self):
        from esrobo_teleop.robot.torso_collision import TorsoCollisionGuard
        # pico_arm_probe_1789782621146427206 immediately before the refused
        # enable. Values are SDK physical radians, converted with live mapping.
        physical = np.asarray([
            0.0286932129, 1.5576190909, 0.0151669112, 0.0535990613,
            -0.0014835299, 0.0310145008, -0.1356295362,
        ])
        robot = build_config().robot
        q = np.zeros(14)
        q[7:] = ((physical - np.asarray(robot.right_joint_offsets))
                 / np.asarray(robot.right_joint_directions))
        guard = TorsoCollisionGuard(self.solver, "right", include_fingers=False)
        self.assertTrue(guard(q, q), guard.last_diagnostics)
        self.assertGreaterEqual(
            guard.last_diagnostics["clearance_m"],
            guard.last_diagnostics["required_m"],
        )

    def test_missing_mesh_never_falls_back_to_guessed_dimensions(self):
        import tempfile
        from esrobo_teleop.robot.torso_collision import TorsoCollisionGuard
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'missing.urdf'
            path.write_text((ROOT / 'urdf/esrobo_waist_with_head.urdf').read_text().replace(
                'meshes/base_link.STL', 'meshes/unavailable.STL'))
            cfg = build_config().ik
            cfg.urdf_path = str(path)
            with self.assertRaisesRegex(ValueError, 'missing collision mesh'):
                TorsoCollisionGuard(IkSolver(cfg), 'left')

    def test_interval_check_covers_unsynchronized_axes_not_only_diagonal(self):
        import threading
        from esrobo_teleop.robot.torso_collision import TorsoCollisionGuard
        g = TorsoCollisionGuard.__new__(TorsoCollisionGuard)
        g.lock = threading.Lock()
        g.margin = .01
        g.last_diagnostics = {}
        def clearance(q, halfwidth):
            distance = np.linalg.norm(q[:2]-[1., 0.]) - .15
            g._box_certified = distance > g.margin + np.linalg.norm(halfwidth[:2])
            return distance
        g._clearance = clearance
        a, b = np.zeros(14), np.zeros(14)
        b[:2] = 1
        # Every point on the diagonal is clear; the independent-axis box is not.
        self.assertFalse(g(a, b))


class ReturnTrajectoryTests(unittest.TestCase):
    def test_return_holds_last_certified_command_until_delayed_feedback_catches_up(self):
        d = trajectory_tests.DriverTrajectoryTests().driver('right')
        _, zero = d._mapping('right')
        feedback = zero.copy()
        feedback[6] -= np.deg2rad(5.0)
        stamp = 10.0
        sends_seen = 0
        hold_reads = 0
        observed_hold_without_send = False
        events = []

        def read():
            nonlocal stamp, sends_seen, hold_reads, observed_hold_without_send
            stamp += .02
            d._arm.last_feedback_timestamp = stamp
            d._return_feedback_received = time.monotonic()
            send_count = d._arm.move_j.call_count
            if send_count > 1 and send_count == sends_seen:
                hold_reads += 1
                observed_hold_without_send = True
                # Model the field log: the controller starts following only
                # after the position stream pauses at the certified target.
                feedback[:] = np.asarray(d._arm.move_j.call_args.args[0])
            else:
                hold_reads = 0
            sends_seen = send_count
            return feedback.copy()

        d.read_joints = mock.Mock(side_effect=read)
        d._emit_probe = lambda event: events.append(event)
        with mock.patch('esrobo_teleop.robot.nero_driver.time.sleep'):
            returned = d._return_group_to_zero(
                np.zeros(14), np.arange(7), zero, np.deg2rad(1.5), 2.0,
                phase_label='delayed-controller',
            )

        self.assertTrue(returned)
        self.assertTrue(observed_hold_without_send)
        self.assertTrue(any(e['event'] == 'return_lead_hold' for e in events))
        self.assertTrue(any(e['event'] == 'return_lead_resumed' for e in events))
        self.assertFalse(any(e['event'] == 'return_fault' for e in events))
        self.assertLessEqual(np.max(np.abs(feedback - zero)), np.deg2rad(1.5) + 1e-9)

    def test_return_lead_hold_times_out_without_resending_or_reanchoring(self):
        d = trajectory_tests.DriverTrajectoryTests().driver('right')
        _, zero = d._mapping('right')
        feedback = zero.copy()
        feedback[6] -= np.deg2rad(3.0)
        stamp = 20.0
        events = []
        d._cfg.command_trajectory_lead_timeout_s = .01

        def read():
            nonlocal stamp
            stamp += .02
            d._arm.last_feedback_timestamp = stamp
            d._return_feedback_received = time.monotonic()
            return feedback.copy()

        def record(event):
            events.append((event, d._arm.move_j.call_count))

        d.read_joints = mock.Mock(side_effect=read)
        d._emit_probe = record
        with mock.patch('esrobo_teleop.robot.nero_driver.time.sleep'):
            returned = d._return_group_to_zero(
                np.zeros(14), np.arange(7), zero, np.deg2rad(1.5), .5,
                phase_label='stalled-controller',
            )

        self.assertFalse(returned)
        hold = next(item for item in events if item[0]['event'] == 'return_lead_hold')
        fault = next(item for item in events if item[0]['event'] == 'return_fault')
        self.assertEqual(fault[0]['fault'], 'return feedback did not catch the held command')
        self.assertEqual(hold[1], fault[1])
        self.assertTrue(d._return_inhibited)

    def test_normal_return_brakes_existing_velocity_without_feedback_snap(self):
        d = trajectory_tests.DriverTrajectoryTests().driver('left')
        _, zero = d._mapping('left')
        position = zero.copy()
        position[3] += .1
        velocity = np.zeros(7)
        velocity[3] = .1
        d._return_start_state = TrajectoryStep(position.copy(), velocity, 10., np.full(7, np.nan), {})
        feedback = position.copy()
        stamp = 10.
        def read():
            nonlocal stamp
            stamp += .02
            d._arm.last_feedback_timestamp = stamp
            d._return_feedback_received = time.monotonic()
            return feedback.copy()
        def send(q):
            nonlocal feedback
            feedback = np.array(q)
        d.read_joints = mock.Mock(side_effect=read)
        d._arm.move_j.side_effect = send
        self.assertTrue(d._return_group_to_zero(np.zeros(14), np.arange(7), zero, .01, 2.))
        sent = np.array([c.args[0] for c in d._arm.move_j.call_args_list])
        first_velocity = (sent[0]-position)/.02
        self.assertGreater(sent[0, 3], position[3])
        amax = d._joint_rate_limit_vector('left', 'max_joint_acceleration')
        self.assertTrue(np.all(abs(first_velocity-velocity) <= amax*.02 + 1e-8))
        self.assertLess(np.max(abs(sent[-1]-zero)), .01)

    def test_return_rejects_expired_motion_state_without_send(self):
        d = trajectory_tests.DriverTrajectoryTests().driver('right')
        _, zero = d._mapping('right')
        d._return_start_state = d._trajectory.state
        d.read_joints = mock.Mock(return_value=zero)
        d._arm.last_feedback_timestamp = 11.
        self.assertFalse(d._return_group_to_zero(np.zeros(14), np.arange(7), zero, .01, 1.))
        d._arm.move_j.assert_not_called()
        self.assertTrue(d._return_inhibited)
