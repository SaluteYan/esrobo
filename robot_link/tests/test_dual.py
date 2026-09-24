import copy
import importlib.util
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest.mock import Mock

from esrobo_link.backends import MockDualBackend
from esrobo_link.client import RobotClient
from esrobo_link.gateway import Gateway
from esrobo_link.protocol import ProtocolError, pack, validate_target
from esrobo_link.session import SessionGate

KEY = b'dual-offline-test-only-secret'*2
PEER = ('127.0.0.1', 12500)


class DualTests(unittest.TestCase):
    def setUp(self):
        self.backend = MockDualBackend()
        self.target = dict(targets={side: dict(arm_urdf_rad=[0.]*7, hand_unit=[128]*10)
                                   for side in ('left', 'right')})

    def test_protocol_validates_whole_frame_before_committing(self):
        gate = SessionGate(self.backend.contract)
        gate.hello(dict(client_nonce='a'*32), PEER, 10., False)
        state = gate.state(10., {})
        msg = dict(self.target, type='target', side='both', contract_id=self.backend.contract['id'],
                   session=gate.session, lease=state['lease'], seq=1)
        gate.accept(msg, PEER, 10.01)
        previous = copy.deepcopy(gate.target)
        for side in ('left', 'right'):
            for field, value in [('arm_urdf_rad', [2.]*7), ('hand_unit', [256]*10)]:
                invalid = copy.deepcopy(msg)
                invalid['seq'] = 2
                invalid['targets'][side][field] = value
                with self.subTest(side=side, field=field), self.assertRaises(ProtocolError):
                    gate.accept(invalid, PEER, 10.02)
                self.assertEqual(gate.target, previous)
                self.assertEqual(gate.sequence, 1)
        for invalid in ({'left': self.target['targets']['left']}, {}, None):
            with self.assertRaises(ProtocolError):
                validate_target(dict(msg, targets=invalid), self.backend.contract)

    def test_both_preflight_before_any_enable(self):
        left, right = (self.backend.members[s] for s in ('left', 'right'))
        right.preflight_enable = Mock(side_effect=RuntimeError('right stale'))
        left.enable = Mock(wraps=left.enable)
        with self.assertRaisesRegex(RuntimeError, 'right stale'):
            self.backend.enable(self.target, lambda: False)
        left.enable.assert_not_called()
        self.assertFalse(left.enabled)
        self.assertEqual((left.stops, right.stops), (1, 1))

    def test_second_enable_failure_stops_first(self):
        self.backend.members['right'].enable = Mock(side_effect=RuntimeError('right enable failed'))
        with self.assertRaisesRegex(RuntimeError, 'right enable failed'):
            self.backend.enable(self.target, lambda: False)
        self.assertFalse(self.backend.members['left'].enabled)
        self.assertTrue(all(m.stops == 1 for m in self.backend.members.values()))

    def test_first_arm_monitored_during_second_enable(self):
        left = self.backend.members['left']
        left.monitor_hold = Mock(side_effect=RuntimeError('left holding torque fault'))
        with self.assertRaisesRegex(RuntimeError, 'holding torque fault'):
            self.backend.enable(self.target, lambda: False)
        self.assertFalse(left.enabled)
        self.assertFalse(self.backend.members['right'].enabled)
        self.assertTrue(all(m.stops == 1 for m in self.backend.members.values()))

    def test_second_feedback_failure_prevents_first_send(self):
        self.backend.enable(self.target, lambda: False)
        self.backend.members['right'].prepare_step = Mock(side_effect=RuntimeError('right feedback stale'))
        self.backend.members['left'].send_arm = Mock()
        with self.assertRaisesRegex(RuntimeError, 'right feedback stale'):
            self.backend.step(self.target, lambda: False)
        self.backend.members['left'].send_arm.assert_not_called()
        self.assertTrue(all(not m.enabled for m in self.backend.members.values()))

    def test_stop_and_disable_always_attempt_second_side(self):
        left, right = (self.backend.members[s] for s in ('left', 'right'))
        left.stop = Mock(side_effect=OSError('left CAN failed'))
        right.stop = Mock(wraps=right.stop)
        with self.assertRaisesRegex(RuntimeError, 'left CAN failed'):
            self.backend.stop()
        right.stop.assert_called_once()
        left.disable = Mock(return_value=False)
        right.disable = Mock(return_value=True)
        self.assertFalse(self.backend.disable())
        right.disable.assert_called_once()
        left.disable.side_effect = OSError('left disable failed')
        with self.assertRaisesRegex(RuntimeError, 'left disable failed'):
            self.backend.disable()
        self.assertEqual(right.disable.call_count, 2)

    def test_shared_hands_enabled_after_both_arms_and_batched_once(self):
        hand = Mock()
        hand.set_enabled.side_effect = lambda value: all(m.enabled for m in self.backend.members.values())
        hand.feedback_ready.return_value = True
        hand.command_physical_batch.return_value = True
        self.backend.hand = hand
        self.backend.enable(self.target, lambda: False)
        hand.set_enabled.assert_called_once_with(True)
        self.backend.step(self.target, lambda: False)
        hand.command_physical_batch.assert_called_once_with({'left': [128]*10, 'right': [128]*10})

    def test_shared_hands_disable_verifies_disabled_state(self):
        hand = Mock()
        hand.is_enabled.return_value = False
        self.backend.hand = hand
        self.assertTrue(self.backend.disable_hands())
        hand.set_enabled.assert_called_once_with(False)
        hand.is_enabled.assert_called_once_with()
        self.assertEqual(self.backend.operation_results,
                         {'operation': 'disable_hands', 'hands': True})

    def test_expired_hand_feedback_prevents_arm_send(self):
        self.backend.hand = Mock()
        self.backend.hand.feedback_ready.return_value = False
        for member in self.backend.members.values():
            member.send_arm = Mock()
        with self.assertRaisesRegex(RuntimeError, 'hand feedback expired'):
            self.backend.step(self.target, lambda: False)
        for member in self.backend.members.values():
            member.send_arm.assert_not_called()

    def test_dual_z_refused_without_state_change_or_movement(self):
        gw = Gateway(self.backend, KEY, port=0)
        try:
            with self.assertRaisesRegex(RuntimeError, 'inter-arm'):
                gw.request('z')
            self.assertEqual(gw.mode, 'IDLE')
            self.assertIsNone(gw.pending)
            self.assertTrue(all(m.stops == 0 for m in self.backend.members.values()))
        finally:
            gw.socket.close()

    def test_dual_recovery_refused_without_return_checker(self):
        gw = Gateway(self.backend, KEY, port=0)
        try:
            gw.mode = 'FAULT'
            with self.assertRaisesRegex(RuntimeError, 'supervised inter-arm'):
                gw.request('r')
            self.assertEqual(gw.mode, 'FAULT')
            self.assertIsNone(gw.pending)
        finally:
            gw.socket.close()

    def test_real_udp_dual_sdk_and_timeout(self):
        gw = Gateway(self.backend, KEY, port=0)
        worker = threading.Thread(target=gw.run)
        worker.start()
        with tempfile.TemporaryDirectory() as tmp:
            keyfile = Path(tmp)/'key'
            keyfile.write_bytes(KEY)
            client = RobotClient(*gw.address, keyfile)
            try:
                self.assertEqual(client.connect()['contract']['side'], 'both')
                with self.assertRaises(ProtocolError):
                    client.send_target([0.]*7)
                client.send_dual_target(left_arm_urdf_rad=[0.]*7, right_arm_urdf_rad=[0.]*7,
                                        left_hand_unit=[128]*10, right_hand_unit=[128]*10)
                until = time.monotonic()+1
                while client.receive()['accepted_seq'] < 1:
                    self.assertLess(time.monotonic(), until)
                gw.request('e')
                for _ in range(8):
                    client.receive()
                    client.send_dual_target(left_arm_urdf_rad=[.05]*7, right_arm_urdf_rad=[-.05]*7,
                                            left_hand_unit=[140]*10, right_hand_unit=[110]*10)
                self.assertGreater(self.backend.members['left'].q[0], 0)
                self.assertLess(self.backend.members['right'].q[0], 0)
                self.assertEqual(self.backend.members['left'].hand, [140]*10)
                self.assertEqual(self.backend.members['right'].hand, [110]*10)
                self.assertLess(len(pack(gw.gate.state(time.monotonic(), {'feedback': self.backend.snapshot()}), KEY)), 8192)
                time.sleep(.26)
                self.assertEqual(gw.mode, 'FAULT')
                self.assertTrue(all(not m.enabled for m in self.backend.members.values()))
            finally:
                client.socket.close()
                gw.request('q')
                worker.join(2)
                self.assertFalse(worker.is_alive())

    def test_laptop_example_rejects_one_stale_source(self):
        path = Path(__file__).resolve().parents[1]/'examples/laptop_integration.py'
        spec = importlib.util.spec_from_file_location('dual_example', path)
        example = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(example)
        client = Mock()
        client.receive.return_value = dict(contract=self.backend.contract, mode='ACTIVE',
                                           feedback=self.backend.snapshot(), feedback_cache_age_s=0.)
        stamps = {s: time.monotonic() for s in ('left_arm', 'right_arm', 'left_hand', 'right_hand')}
        args = dict(left_arm_urdf_rad=[0.]*7, right_arm_urdf_rad=[0.]*7,
                    left_hand_unit=[128]*10, right_hand_unit=[128]*10)
        for side in stamps:
            bad = dict(stamps)
            bad[side] -= 1.
            example.send_dual_solved_frame(client, **args, input_received_monotonic=bad)
            client.send_dual_target.assert_not_called()
        example.send_dual_solved_frame(client, **args, input_received_monotonic=stamps)
        client.send_dual_target.assert_called_once()


if __name__ == '__main__':
    unittest.main()
