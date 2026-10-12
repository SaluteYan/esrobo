"""Robot adapter tests; no CAN, ROS or device initialization."""
import json
from pathlib import Path
import sys
import threading
import time
import unittest
from unittest.mock import Mock, patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'robot_link'))
from esrobo_link.backends import HardwareBackend
from esrobo_teleop.config import build_config
from esrobo_teleop.robot.linker_hand_driver import LinkerHandDriver
from esrobo_teleop.robot.nero_driver import NeroSingleArmDriver


class AdapterTests(unittest.TestCase):
    def backend(self):
        backend = HardwareBackend.__new__(HardwareBackend)
        backend.np = np
        backend.arm = Mock()
        backend.hand = None
        backend.side = 'left'
        backend.cfg = build_config()
        backend._full_slice = slice(0, 7)
        return backend

    def test_step_requires_new_cycle_feedback(self):
        backend = self.backend()
        backend.arm.read_control_cycle_joints.return_value = None
        with self.assertRaisesRegex(RuntimeError, 'fresh seven-joint'):
            backend.step(dict(arm_urdf_rad=[0.]*7), lambda: False)
        backend.arm.command_full_urdf.assert_not_called()

    def test_step_refreshes_before_command_and_honors_cancel(self):
        backend = self.backend()
        calls = []
        backend.arm.read_control_cycle_joints.side_effect = lambda: calls.append('read') or np.zeros(14)
        backend.arm.command_full_urdf.side_effect = lambda q: calls.append('send') or True
        backend.step(dict(arm_urdf_rad=[.1]*7), lambda: False)
        self.assertEqual(calls, ['read', 'send'])
        calls.clear()
        with self.assertRaisesRegex(RuntimeError, 'expired'):
            backend.step(dict(arm_urdf_rad=[.1]*7), lambda: True)
        self.assertEqual(calls, ['read'])

    def test_stale_enable_bits_are_unknown(self):
        backend = self.backend()
        backend.arm.read_control_cycle_joints.return_value = np.zeros(14)
        backend.arm._cycle_feedback = None
        backend.arm._arm._robot.get_driver_states.return_value = Mock(timestamp=time.time()-100)
        self.assertIsNone(backend.snapshot()['enable_states'])

    def test_torque_settle_is_cancellable_without_sampling(self):
        driver = NeroSingleArmDriver.__new__(NeroSingleArmDriver)
        driver._cfg = build_config().robot
        driver._cfg.torque_monitor_enabled = True
        driver._cfg.torque_baseline_settle_s = 1.
        arm = Mock()
        self.assertFalse(driver._capture_arm_torque_baseline(arm, 'left', cancelled=lambda: True))
        arm.get_joint_torques.assert_not_called()

    def test_session_preparation_holds_hand_during_arm_return(self):
        backend = self.backend()
        backend.hand_only = False
        events = []
        backend.hand = Mock()
        backend.hand.feedback_snapshot.return_value = dict(position_unit=[128] * 10, age_s=0.)
        backend.hand.set_enabled.side_effect = lambda enabled: events.append('hand_disabled')
        backend.return_zero = Mock(side_effect=lambda cancelled, **kwargs: events.append('arm_return') or {
            'returned': True, 'disabled': True, 'stage': 'disabling', 'reason': ''})
        result = backend.prepare_session(lambda: False)
        self.assertEqual(events, ['hand_disabled', 'arm_return'])
        self.assertTrue(result['hands_stationary'])
        self.assertFalse(result['hands_open'])
        self.assertTrue(backend.return_zero.call_args.kwargs['arm_only_collision'])
        self.assertTrue(callable(backend.return_zero.call_args.kwargs['stationary_hand_check']))
        backend.hand.align_and_verify_open_pose.assert_not_called()

    def test_session_preparation_rejects_missing_hand_feedback_without_motion(self):
        backend = self.backend()
        backend.hand_only = False
        backend.hand = Mock()
        backend.hand.feedback_snapshot.return_value = dict(position_unit=None, age_s=None)
        backend.return_zero = Mock()
        with self.assertRaisesRegex(RuntimeError, 'fresh hand feedback required'):
            backend.prepare_session(lambda: False)
        backend.hand.align_and_verify_open_pose.assert_not_called()
        backend.return_zero.assert_not_called()


class PhysicalHandTests(unittest.TestCase):
    def setUp(self):
        patcher = patch('esrobo_teleop.robot.linker_hand_driver.socket.socket')
        self.socket = patcher.start().return_value
        self.addCleanup(patcher.stop)
        self.socket.recvfrom.side_effect = BlockingIOError
        cfg = build_config().hand
        cfg.mode = 'udp'
        cfg.feedback_udp_port = 0
        cfg.max_step = 5
        cfg.max_velocity = 100
        cfg.publish_hz = 50
        self.driver = LinkerHandDriver(cfg, udp_port=9, active_sides=('left',))
        self.addCleanup(self.driver.close)
        self.driver._feedback['left'] = np.full(10, 120.)
        self.driver._feedback_time['left'] = time.monotonic()
        self.assertTrue(self.driver.set_enabled(True))
        self.driver._last_sent_time = time.monotonic()-1.

    def test_local_limits_mask_and_selected_side(self):
        self.assertTrue(self.driver.command_physical('left', [255]*10))
        payload = json.loads(self.socket.sendto.call_args.args[0])
        self.assertEqual(set(payload), {'left'})
        for i, value in enumerate(payload['left']):
            self.assertEqual(value, 122 if i in self.driver._cfg.enabled_physical_joints else 120)

    def test_expired_hand_feedback_sends_nothing(self):
        self.driver._feedback_time['left'] = time.monotonic()-100
        self.assertFalse(self.driver.command_physical('left', [255]*10))
        self.socket.sendto.assert_not_called()
        self.assertFalse(self.driver.is_enabled())

    def test_fifty_hz_jitter_never_silently_drops_a_successful_command(self):
        for periods in ((.02,), (.019, .021), (.001, .039)):
            with self.subTest(periods=periods):
                self.socket.sendto.reset_mock()
                self.driver._last_cmd['left'][:] = 120
                self.driver._last_sent_time = 10.
                self.driver._cfg.max_velocity = 400
                self.driver._cfg.max_step = 18
                previous = 120
                stamp = 10.
                for n in range(100):
                    dt = periods[n % len(periods)]
                    stamp += dt
                    target = 255 if n < 50 else 0
                    with patch('esrobo_teleop.robot.linker_hand_driver.time.monotonic', return_value=stamp):
                        self.driver._feedback_time['left'] = stamp
                        self.assertTrue(self.driver.command_physical('left', [target]*10))
                    command = json.loads(self.socket.sendto.call_args.args[0])['left'][0]
                    budget = int(np.floor(400*min(dt, .02)+1.e-7))
                    self.assertLessEqual(abs(command-previous), budget)
                    previous = command
                self.assertEqual(self.socket.sendto.call_count, 100)

    def test_transport_failure_preserves_published_state_and_velocity_baseline(self):
        self.driver.command_physical('left', [255]*10)
        command = self.driver._last_cmd['left'].copy()
        published = self.driver.feedback_snapshot('left')
        stamp = self.driver._last_sent_time
        self.socket.sendto.side_effect = OSError('test UDP failure')
        with self.assertRaises(OSError):
            self.driver.command_physical('left', [0]*10)
        np.testing.assert_array_equal(self.driver._last_cmd['left'], command)
        self.assertEqual(self.driver._last_sent_time, stamp)
        latest = self.driver.feedback_snapshot('left')
        self.assertEqual(latest['command_seq'], published['command_seq'])
        self.assertEqual(latest['command_unit'], published['command_unit'])

    def test_enabling_does_not_report_unsent_command_as_published(self):
        self.assertIsNone(self.driver.feedback_snapshot('left')['command_unit'])
        self.assertEqual(self.driver.feedback_snapshot('left')['command_seq'], 0)

    def test_open_return_stops_when_feedback_disappears_mid_ramp(self):
        if 'right' in self.driver._active_sides:
            self.driver._feedback['right'][0] = 160
        with patch.object(self.driver, '_feedback_is_fresh', side_effect=[True, True, False]), \
                patch('esrobo_teleop.robot.linker_hand_driver.time.sleep'):
            self.assertFalse(self.driver.return_to_open())
        self.assertEqual(self.socket.sendto.call_count, 1)
        self.assertFalse(self.driver.is_enabled())

    def test_thumb_axes_map_to_distinct_physical_channels(self):
        from esrobo_teleop.robot.linker_hand_driver import ACTIVE_HAND_JOINTS, ACTIVE_JOINT_LIMITS_RAD
        baseline = self.driver._rad_to_servo(np.zeros(10), 'right')
        for name, physical_index in [('thumb_cmc_pitch', 0), ('thumb_cmc_yaw', 1), ('thumb_cmc_roll', 9)]:
            angles = np.zeros(10)
            angles[ACTIVE_HAND_JOINTS.index(name)] = ACTIVE_JOINT_LIMITS_RAD[name][1]
            result = self.driver._rad_to_servo(angles, 'right')
            self.assertEqual(np.flatnonzero(result != baseline).tolist(), [physical_index])

    def test_feedback_is_read_only_copy(self):
        state = self.driver.feedback_snapshot('left')
        state['position_unit'][0] = 0
        self.assertEqual(self.driver._feedback['left'][0], 120)
        self.socket.sendto.assert_not_called()

    def test_feedback_reports_incomplete_collision_geometry_calibration(self):
        geometry = self.driver.feedback_snapshot('left')['geometry']
        self.assertFalse(geometry['ready'])
        self.assertEqual(geometry['calibrated_joints'], 0)
        self.assertEqual(geometry['total_joints'], 10)
        self.assertEqual(len(geometry['missing_joints']), 10)

        names = list(geometry['missing_joints'])
        self.driver._cfg.geometry_feedback_calibration = {'left': {
            name: dict(raw=[0, 255], rad=[0, 1], error_rad=.02,
                       max_velocity_rad_s=1., verified=True)
            for name in names
        }}
        geometry = self.driver.feedback_snapshot('left')['geometry']
        self.assertTrue(geometry['ready'])
        self.assertEqual(geometry['calibrated_joints'], 10)

    def test_invalid_target_sends_nothing(self):
        for target in ([0]*9, [float('nan')]*10, [256]*10):
            with self.assertRaises(ValueError):
                self.driver.command_physical('left', target)
        self.socket.sendto.assert_not_called()


class DualPhysicalHandTests(PhysicalHandTests):
    def setUp(self):
        super().setUp()
        self.driver._active_sides = ('left', 'right')
        self.driver._feedback['right'] = np.full(10, 140.)
        self.driver._feedback_time['right'] = time.monotonic()
        self.assertTrue(self.driver.set_enabled(True))

    def test_batch_sends_both_targets_once(self):
        self.assertTrue(self.driver.command_physical_batch({'left': [255]*10, 'right': [0]*10}))
        self.socket.sendto.assert_called_once()
        payload = json.loads(self.socket.sendto.call_args.args[0])
        self.assertEqual(payload['left'][0], 122)
        # A target below the commissioned thumb flexion floor moves away from
        # the measured stop rather than pushing farther into it.
        self.assertEqual(payload['right'][0], 142)
        for i in (6, 7, 8):
            self.assertEqual(payload['left'][i], 120)
            self.assertEqual(payload['right'][i], 140)

    def test_right_stale_blocks_whole_batch(self):
        self.driver._feedback_time['right'] -= 100
        self.assertFalse(self.driver.command_physical_batch({'left': [255]*10, 'right': [0]*10}))
        self.socket.sendto.assert_not_called()

    def test_partial_or_invalid_batch_sends_nothing(self):
        for target in ({'left': [255]*10}, {'left': [255]*10, 'right': [256]*10}):
            with self.assertRaises(ValueError):
                self.driver.command_physical_batch(target)
        self.socket.sendto.assert_not_called()

    # The inherited single-side test checks only-left publication, whereas this
    # fixture deliberately activates both hands. Replace it with batch coverage.
    def test_local_limits_mask_and_selected_side(self):
        self.test_batch_sends_both_targets_once()


class DualConstructionTests(unittest.TestCase):
    def test_duplicate_can_rejected_before_opening_devices(self):
        from esrobo_link.backends import DualHardwareBackend
        cfg = build_config()
        cfg.robot.right_can_channel = cfg.robot.left_can_channel
        with patch('esrobo_teleop.config.load_config', return_value=cfg), \
             patch('esrobo_teleop.robot.linker_hand_driver.LinkerHandDriver') as hand, \
             patch('esrobo_link.backends.HardwareBackend') as arm:
            with self.assertRaisesRegex(ValueError, 'distinct CAN'):
                DualHardwareBackend('unused.yaml')
            hand.assert_not_called()
            arm.assert_not_called()

    def test_one_feedback_receiver_shared_by_two_arm_adapters(self):
        from esrobo_link.backends import DualHardwareBackend, MockBackend
        config = Path(__file__).resolve().parents[1]/'config/teleop_config.yaml'
        with patch('esrobo_teleop.robot.linker_hand_driver.LinkerHandDriver') as hand, \
             patch('esrobo_link.backends.HardwareBackend') as arm:
            arm.side_effect = lambda config, side, with_hand, hand_driver: MockBackend(side, True)
            backend = DualHardwareBackend(config)
            try:
                hand.assert_called_once()
                self.assertEqual(hand.call_args.kwargs['active_sides'], ('left', 'right'))
                self.assertEqual(arm.call_count, 2)
                self.assertTrue(all(call.kwargs['hand_driver'] is hand.return_value for call in arm.call_args_list))
            finally:
                backend.close()
            hand.return_value.close.assert_called_once()


if __name__ == '__main__':
    unittest.main()
