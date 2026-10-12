import io
import json
import socket
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

from esrobo_link.protocol import ProtocolError, pack, unpack, validate_hold, validate_target
from esrobo_link.session import SessionGate
from esrobo_link.backends import HardwareBackend, MockBackend, MockDualBackend
from esrobo_link.gateway import Gateway
from esrobo_link.client import RobotClient
from esrobo_link.camera import LatestImage, handler_for
from http.server import ThreadingHTTPServer
import http.client

KEY = b'unit-test-only-key-not-for-deployment' * 2
PEER = ('127.0.0.1', 12000)


class Fixture(unittest.TestCase):
    def setUp(self):
        self.backend = MockBackend(with_hand=True)
        self.gate = SessionGate(self.backend.contract)
        self.gate.hello(dict(client_nonce='a'*32), PEER, 10., False)
        self.state = self.gate.state(10., {})
        self.msg = dict(type='target', session=self.gate.session, seq=1,
                        lease=self.state['lease'], contract_id=self.backend.contract['id'],
                        side='left', arm_urdf_rad=[.1]*7, hand_unit=[128]*10)


class ProtocolTests(Fixture):
    def test_roundtrip_and_auth(self):
        self.assertEqual(unpack(pack(self.msg, KEY), KEY), dict(self.msg, v=1))
        with self.assertRaises(ProtocolError):
            unpack(pack(self.msg, KEY), b'wrong key')
        packet = json.loads(pack(self.msg, KEY))
        packet['body'] = packet['body'].replace('left', 'right')
        with self.assertRaises(ProtocolError):
            unpack(json.dumps(packet).encode(), KEY)

    def test_bad_packets(self):
        for raw in (b'', b'[]', b'null', b'{}', b'a'*8193, b'\xff'):
            with self.subTest(raw=raw[:8]), self.assertRaises(ProtocolError):
                unpack(raw, KEY)
        with self.assertRaises(ValueError):
            pack({'q': float('nan')}, KEY)
        with self.assertRaises(ProtocolError):
            pack({'image': 'x'*9000}, KEY)

    def test_target_validation(self):
        cases = [dict(side='right'), dict(contract_id='other'), dict(arm_urdf_rad=[0]*6),
                 dict(arm_urdf_rad=[2.]*7), dict(arm_urdf_rad=[True]*7),
                 dict(arm_urdf_rad=[float('nan')]*7), dict(hand_unit=[256]*10),
                 dict(arm_urdf_rad=[10**400]*7),
                 dict(hand_unit=[128.]*10), dict(hand_unit=None)]
        for change in cases:
            with self.subTest(change=change), self.assertRaises(ProtocolError):
                validate_target(dict(self.msg, **change), self.backend.contract)

    def test_hand_only_target_has_no_arm_field(self):
        contract = MockBackend('right', True, True).contract
        message = dict(contract_id=contract['id'], side='right', hand_unit=[128]*10)
        self.assertEqual(validate_target(message, contract),
                         {'arm_urdf_rad': None, 'hand_unit': [128]*10})
        with self.assertRaises(ProtocolError):
            validate_target(dict(message, arm_urdf_rad=None), contract)

    def test_hold_has_no_position_and_requires_new_protocol_for_hands(self):
        contract = MockBackend('right', False).contract
        hold = dict(side='right', contract_id=contract['id'])
        self.assertEqual(validate_hold(hold, contract), {'hold': True})
        for invalid in (dict(side='left'), dict(arm_urdf_rad=[0.] * 7),
                        dict(hand_unit=None)):
            with self.subTest(invalid=invalid), self.assertRaises(ProtocolError):
                validate_hold(dict(hold, **invalid), contract)
        hand_contract = MockBackend('right', True).contract
        hand_hold = dict(side='right', contract_id=hand_contract['id'])
        self.assertEqual(validate_hold(hand_hold, hand_contract), {'hold': True})
        old_contract = dict(hand_contract)
        old_contract.pop('input_hold_protocol')
        with self.assertRaises(ProtocolError, msg='old hand gateway must reject hold'):
            validate_hold(hand_hold, old_contract)
        dual_contract = MockDualBackend().contract
        self.assertEqual(validate_hold(dict(side='both', contract_id=dual_contract['id']), dual_contract),
                         {'hold': True})
        with self.assertRaises(ProtocolError):
            validate_hold(hold, MockBackend('right', True).contract)

    def test_position_hold_requires_v2_and_a_single_arm(self):
        contract = MockBackend('right', True).contract
        msg = dict(side='right', contract_id=contract['id'], reason='position')
        self.assertEqual(validate_hold(msg, contract), {'hold': True, 'hold_reason': 'position'})
        # Legacy packets retain their finite input-gap semantics.
        self.assertEqual(validate_hold(dict(msg, reason='input'), dict(contract, input_hold_protocol=1)),
                         {'hold': True})
        for invalid in (dict(contract, input_hold_protocol=1),
                        dict(contract, input_hold_protocol=None),
                        dict(contract, hand_only=True), dict(contract, side='both')):
            with self.subTest(invalid=invalid), self.assertRaises(ProtocolError):
                validate_hold(dict(msg, side=invalid['side']), invalid)
        for reason in ('unknown', None, True, [], {}):
            with self.subTest(reason=reason), self.assertRaises(ProtocolError):
                validate_hold(dict(msg, reason=reason), contract)


class SessionTests(Fixture):
    def test_fresh_hold_renews_lease_but_never_carries_a_position(self):
        gate = SessionGate(MockBackend('right', False).contract)
        gate.hello(dict(client_nonce='h'*32), PEER, 10., False)
        state = gate.state(10., {})
        hold = dict(type='hold', session=gate.session, seq=2,
                    lease=state['lease'], contract_id=gate.contract['id'], side='right')
        with self.assertRaisesRegex(ProtocolError, 'preceding fresh target'):
            gate.accept(hold, PEER, 10.005)
        target = dict(type='target', session=gate.session, seq=1,
                      lease=state['lease'], contract_id=gate.contract['id'],
                      side='right', arm_urdf_rad=[.1] * 7)
        gate.accept(target, PEER, 10.005)
        self.assertEqual(gate.accept(hold, PEER, 10.01), 'hold')
        self.assertEqual(gate.target, {'hold': True})
        self.assertEqual(gate.hold_started_at, 10.01)
        self.assertTrue(gate.fresh(10.19))
        self.assertFalse(gate.fresh(10.21))

    def test_delay_consumes_original_lease(self):
        self.gate.accept(self.msg, PEER, 10.19)
        self.assertAlmostEqual(self.gate.deadline, 10.2)
        self.assertTrue(self.gate.fresh(10.199))
        self.assertFalse(self.gate.fresh(10.201))
        with self.assertRaises(ProtocolError):
            self.gate.accept(dict(self.msg, seq=2), PEER, 10.21)

    def test_replay_wrong_session_and_invalid_do_not_renew(self):
        self.gate.accept(self.msg, PEER, 10.01)
        for change in ({}, dict(seq=0), dict(seq=True), dict(seq=2, session='old'),
                       dict(seq=2, side='right'), dict(seq=2, type='enable')):
            with self.subTest(change=change), self.assertRaises(ProtocolError):
                self.gate.accept(dict(self.msg, **change), PEER, 10.1)
        self.assertEqual(self.gate.sequence, 1)
        self.assertAlmostEqual(self.gate.deadline, 10.2)
        with self.assertRaises(ProtocolError):
            self.gate.accept(dict(self.msg, seq=2), ('other', 1), 10.1)

    def test_owner_and_restart(self):
        for now, busy in [(10.2, False), (12., True)]:
            with self.assertRaises(ProtocolError):
                self.gate.hello(dict(client_nonce='b'*32), ('other', 1), now, busy)
        self.gate.hello(dict(client_nonce='b'*32), ('other', 1), 12., False)
        self.assertIsNone(self.gate.target)
        with self.assertRaises(ProtocolError):
            self.gate.accept(self.msg, PEER, 12.)

    def test_hello_does_not_keep_motion_alive(self):
        self.gate.accept(self.msg, PEER, 10.01)
        self.gate.hello(dict(client_nonce='a'*32), PEER, 10.3, True)
        self.assertFalse(self.gate.fresh(10.3))
        for n in range(60):
            self.gate.state(10.+n*.02, {})
        self.assertEqual(len(self.gate.leases), 32)


class GatewayTests(unittest.TestCase):
    def setUp(self):
        self.backend = MockBackend()
        self.gw = Gateway(self.backend, KEY, port=0, journal=io.StringIO())
        self.addCleanup(self.gw.socket.close)
        self.now = 100.
        self.clock = patch('esrobo_link.gateway.time.monotonic', lambda: self.now)
        self.clock.start()
        self.addCleanup(self.clock.stop)
        self.gw.gate.hello(dict(client_nonce='a'*32), PEER, self.now, False)
        self.seq = 0

    def target(self):
        self.seq += 1
        state = self.gw.gate.state(self.now, {})
        self.gw.gate.accept(dict(type='target', seq=self.seq, session=state['session'],
            lease=state['lease'], side='left', contract_id=self.backend.contract['id'],
            arm_urdf_rad=[.1]*7), PEER, self.now)

    def start(self):
        self.target()
        self.gw.request('e')
        self.gw.tick()
        self.assertEqual(self.gw.mode, 'CALIBRATING')
        self.target()
        self.gw.tick()
        self.assertEqual(self.gw.mode, 'ACTIVE')

    def test_no_automatic_enable(self):
        self.target()
        self.gw.tick()
        self.assertFalse(self.backend.enabled)
        self.assertEqual(self.backend.q, [0.]*7)

    def test_active_hold_keeps_mock_pose_without_reusing_previous_target(self):
        self.start()
        before = list(self.backend.q)
        self.seq += 1
        state = self.gw.gate.state(self.now, {})
        self.gw.gate.accept(dict(type='hold', seq=self.seq, session=state['session'],
            lease=state['lease'], side='left', contract_id=self.backend.contract['id']),
            PEER, self.now)
        self.assertEqual(self.gw.gate.target, {'hold': True})
        self.gw.tick()
        self.assertEqual(self.gw.mode, 'ACTIVE')
        self.assertEqual(self.backend.q, before)

    def test_hold_limit_trips_even_with_fresh_lease(self):
        self.start()
        self.gw.gate.target = {'hold': True}
        self.gw.gate.hold_started_at = self.now - .501
        self.gw.gate.deadline = self.now + .2
        self.gw.watchdog()
        self.assertEqual(self.gw.mode, 'FAULT')
        self.assertIn('input hold exceeded 500 ms', self.gw.reason)

    def hold_packet(self, reason):
        self.seq += 1
        state = self.gw.gate.state(self.now, {})
        return dict(type='hold', seq=self.seq, session=state['session'], lease=state['lease'],
                    side='left', contract_id=self.backend.contract['id'], reason=reason)

    def test_position_pause_exceeds_500ms_with_fresh_leases_but_still_expires(self):
        self.start()
        before = list(self.backend.q)
        for _ in range(60):
            self.now += .02
            self.gw.gate.accept(self.hold_packet('position'), PEER, self.now)
            self.gw.tick()
            self.assertEqual(self.gw.mode, 'ACTIVE')
            self.assertIsNone(self.gw.gate.hold_started_at)
            self.assertEqual(self.backend.q, before)
        self.assertGreater(self.now-self.gw.gate.position_hold_started_at, 1.)
        self.gw.record()
        entry = json.loads(self.gw.journal.getvalue().splitlines()[-1])
        self.assertEqual(entry['hold_reason'], 'position')
        self.assertGreater(entry['position_hold_age_s'], 1.)
        self.now += .21
        self.gw.watchdog()
        self.assertEqual(self.gw.mode, 'FAULT')
        self.assertIn('lease expired', self.gw.reason)

    def test_switch_from_position_to_input_starts_500ms_timer_without_renewing_it(self):
        self.start()
        for _ in range(35):
            self.now += .02
            self.gw.gate.accept(self.hold_packet('position'), PEER, self.now)
            self.gw.tick()
        self.now += .02
        self.gw.gate.accept(self.hold_packet('input'), PEER, self.now)
        started = self.gw.gate.hold_started_at
        self.assertIsNone(self.gw.gate.position_hold_started_at)
        for _ in range(24):
            self.now += .02
            self.gw.gate.accept(self.hold_packet('input'), PEER, self.now)
            self.gw.watchdog()
            self.assertEqual(self.gw.mode, 'ACTIVE')
            self.assertEqual(self.gw.gate.hold_started_at, started)
        self.now += .03
        self.gw.watchdog()
        self.assertEqual(self.gw.mode, 'FAULT')
        self.assertIn('input hold exceeded 500 ms', self.gw.reason)

    def test_fresh_position_pause_clears_input_timer_and_target_clears_pause(self):
        self.start()
        self.now += .02
        self.gw.gate.accept(self.hold_packet('input'), PEER, self.now)
        self.now += .1
        self.gw.gate.accept(self.hold_packet('position'), PEER, self.now)
        self.assertIsNone(self.gw.gate.hold_started_at)
        self.assertIsNotNone(self.gw.gate.position_hold_started_at)
        self.now += .02
        self.target()
        self.assertIsNone(self.gw.gate.position_hold_started_at)

    def test_invalid_position_hold_cannot_renew_lease_or_clear_input_timer(self):
        self.start()
        self.gw.gate.accept(self.hold_packet('input'), PEER, self.now)
        started, deadline = self.gw.gate.hold_started_at, self.gw.gate.deadline
        msg = self.hold_packet('position')
        for change in (dict(lease='invalid'), dict(arm_urdf_rad=[0.]*7),
                       dict(session='invalid'), dict(seq=self.seq-1)):
            with self.subTest(change=change), self.assertRaises(ProtocolError):
                self.gw.gate.accept(dict(msg, **change), PEER, self.now)
            self.assertEqual(self.gw.gate.hold_started_at, started)
            self.assertEqual(self.gw.gate.deadline, deadline)

    def test_timeout_latches_and_requires_explicit_recovery(self):
        self.start()
        self.now += .21
        self.gw.tick()
        self.assertEqual(self.gw.mode, 'FAULT')
        self.assertEqual(self.backend.stops, 1)
        q = list(self.backend.q)
        self.target()
        self.gw.tick()
        self.assertEqual(self.backend.q, q)
        with self.assertRaises(RuntimeError):
            self.gw.request('e')
        self.gw.request('z')
        self.gw.tick()
        self.assertEqual(self.gw.mode, 'IDLE')
        self.assertFalse(self.backend.enabled)
        self.assertIsNone(self.gw.gate.target)

    def test_hand_only_recovery_verifies_disable_without_motion_or_auto_enable(self):
        backend = MockBackend('right', True, True)
        gateway = Gateway(backend, KEY, port=0)
        self.addCleanup(gateway.socket.close)
        backend.hand_enabled = True
        gateway.mode = 'ACTIVE'
        gateway.request('s')
        with self.assertRaisesRegex(RuntimeError, 'stopped FAULT'):
            gateway.request('r')
        gateway.tick()
        self.assertEqual(gateway.mode, 'FAULT')
        gateway.request('r')
        self.assertEqual(gateway.mode, 'RECOVERING')
        gateway.tick()
        self.assertEqual(gateway.mode, 'IDLE')
        self.assertFalse(backend.enabled)
        self.assertFalse(backend.hand_enabled)
        self.assertEqual(backend.q, [0.] * 7)
        self.assertIsNone(gateway.gate.target)

    def test_hand_only_recovery_failure_keeps_fault_latched(self):
        backend = MockBackend('right', True, True)
        gateway = Gateway(backend, KEY, port=0)
        self.addCleanup(gateway.socket.close)
        gateway.mode = 'FAULT'
        backend.recover_idle = Mock(side_effect=RuntimeError('disable feedback stale'))
        gateway.request('r')
        gateway.tick()
        self.assertEqual(gateway.mode, 'FAULT')
        self.assertIn('disable feedback stale', gateway.reason)

    def test_single_arm_recovery_reuses_checked_return(self):
        self.gw.mode = 'FAULT'
        self.gw.request('r')
        self.assertEqual(self.gw.mode, 'RETURNING')
        self.gw.tick()
        self.assertEqual(self.gw.mode, 'IDLE')
        self.assertEqual(self.gw.last_return['returned'], True)

    def test_restarted_idle_gateway_allows_explicit_controller_fault_recovery(self):
        self.gw.feedback = {'controller_fault': {'arm_status': 1, 'status_name': 'EMERGENCY_STOP'}}
        self.gw.request('r')
        self.assertEqual(self.gw.mode, 'RETURNING')
        self.gw.tick()
        self.assertEqual(self.gw.mode, 'IDLE')

    def test_healthy_idle_gateway_does_not_admit_fault_recovery(self):
        with self.assertRaises(RuntimeError):
            self.gw.request('r')

    def test_q_stops_without_return(self):
        self.start()
        q = list(self.backend.q)
        preparation = self.gw.last_return
        self.gw.request('q')
        self.gw.tick()
        self.assertTrue(self.gw.shutdown.is_set())
        self.assertEqual(self.backend.q, q)
        self.assertEqual(self.gw.last_return, preparation)

    def test_partial_enable_failure_stops(self):
        def fail(target, cancelled):
            self.backend.enabled = True
            raise RuntimeError('CAN failed')
        self.backend.enable = fail
        self.target()
        self.gw.request('e')
        self.gw.tick()
        self.target()
        self.gw.tick()
        self.assertEqual(self.gw.mode, 'FAULT')
        self.gw.tick()
        self.assertFalse(self.backend.enabled)
        self.assertTrue(self.gw.stop_send_returned)

    def test_cancel_during_enable(self):
        def enable(target, cancelled):
            self.now += .21
            self.assertTrue(cancelled())
        self.backend.enable = enable
        self.target()
        self.gw.request('e')
        self.gw.tick()
        self.target()
        self.gw.tick()
        self.assertEqual(self.gw.mode, 'FAULT')
        self.assertEqual(self.backend.q, [0.]*7)

    def test_stop_failure_not_reported_as_verified(self):
        self.start()
        self.backend.stop = lambda: (_ for _ in ()).throw(OSError('CAN down'))
        self.gw.request('x')
        self.gw.tick()
        self.assertFalse(self.gw.stop_send_returned)
        self.assertEqual(self.gw.mode, 'FAULT')

    def test_log_contains_diagnostics_not_credentials(self):
        self.start()
        self.gw.record()
        text = self.gw.journal.getvalue()
        entry = json.loads(text)
        self.assertEqual(entry['mode'], 'ACTIVE')
        self.assertNotIn(self.gw.gate.session, text)
        self.assertNotIn(KEY.decode(), text)
        self.assertIn('feedback', entry)

    def test_arm_and_hand_hold_brakes_without_replaying_position_and_still_expires(self):
        backend = MockBackend('right', True)
        gateway = Gateway(backend, KEY, port=0)
        self.addCleanup(gateway.socket.close)
        backend.enabled = backend.hand_enabled = True
        backend.q[0], backend.hand[0] = .3, 177
        gateway.mode = 'ACTIVE'
        gateway.gate.target = {'hold': True}
        gateway.gate.deadline = self.now + .2
        gateway.tick()
        self.assertEqual(gateway.mode, 'ACTIVE')
        self.assertEqual((backend.q[0], backend.hand[0]), (.3, 177))
        self.now += .21
        gateway.watchdog()
        self.assertEqual(gateway.mode, 'FAULT')
        self.assertIn('lease expired', gateway.reason)

    def test_hardware_arm_and_hand_hold_uses_fresh_measured_hand_position(self):
        backend = HardwareBackend.__new__(HardwareBackend)
        backend.side = 'right'
        backend.hand_only = False
        backend.arm = Mock()
        backend.arm.read_control_cycle_joints.return_value = [0.] * 14
        backend.arm.hold_command_trajectory.return_value = True
        backend.hand = Mock()
        backend.hand.feedback_snapshot.return_value = {'position_unit': [137.] * 10}
        backend.hand.feedback_ready.return_value = True
        backend.hand.command_physical.return_value = True
        backend.step({'hold': True}, lambda: False)
        backend.arm.hold_command_trajectory.assert_called_once_with()
        backend.hand.command_physical.assert_called_once_with('right', [137.] * 10)
        backend.arm.hold_command_trajectory.reset_mock()
        backend.hand.feedback_ready.return_value = False
        with self.assertRaisesRegex(RuntimeError, 'hand feedback missing'):
            backend.step({'hold': True}, lambda: False)
        backend.arm.hold_command_trajectory.assert_not_called()

    def test_first_trajectory_fault_is_preserved_after_stop(self):
        context = dict(failed_joints=[7], feedback_dt_s=.0252,
                       target_physical_rad=[0.0] * 7,
                       previous_command_physical_rad=[0.0] * 7,
                       feedback_physical_rad=[0.0] * 7,
                       feasible_velocity_lower_rad_s=[0.0] * 7,
                       feasible_velocity_upper_rad_s=[0.0] * 7,
                       lead_rad=[0.0] * 7, lead_limit_rad=[.026] * 7)
        context['target_physical_rad'][6] = -.275
        context['feasible_velocity_lower_rad_s'][6] = -.0157
        context['feasible_velocity_upper_rad_s'][6] = -.0302
        diagnostics = {'fault': 'right command trajectory', 'fault_context': context}
        self.backend.fault_diagnostics = lambda: diagnostics
        self.gw.mode = 'ACTIVE'
        reason = 'executor: RuntimeError: right command trajectory: no acceleration-consistent command'
        self.gw.trip(reason)
        context['target_physical_rad'][6] = 99.0  # The captured frame must be immutable.
        self.gw.trip('laptop requested stop')
        self.assertEqual(self.gw.reason, reason)
        self.gw.record()
        self.now += .21
        self.gw.record()
        records = [json.loads(line) for line in self.gw.journal.getvalue().splitlines()]
        events = [row for row in records if row.get('record_type') == 'first_fault']
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0]['reason'], reason)
        self.assertEqual(events[0]['diagnostics']['fault_context']['target_physical_rad'][6], -.275)
        self.assertEqual(records[-1]['first_fault_reason'], reason)
        self.assertEqual(records[-1]['first_fault_summary']['failed_joints'], [7])
        self.assertEqual(records[-1]['first_fault_summary']['feasible_velocity_upper_rad_s'], -.0302)

    def test_owned_arm_can_be_disabled_independently(self):
        self.start()
        self.gw.request('dl')
        self.gw.tick()
        self.assertEqual(self.gw.mode, 'FAULT')
        self.assertFalse(self.backend.enabled)
        self.assertIn('left_disable_verified=True', self.gw.reason)

    def test_unowned_arm_disable_fails_closed(self):
        self.gw.request('dr')
        self.gw.tick()
        self.assertEqual(self.gw.mode, 'FAULT')
        self.assertIn('does not own right arm', self.gw.reason)

    def test_dual_backend_disables_only_requested_arm_after_stop(self):
        backend = MockDualBackend()
        gateway = Gateway(backend, KEY, port=0)
        self.addCleanup(gateway.socket.close)
        backend.members['left'].enabled = True
        backend.members['right'].enabled = True
        gateway.request('dl')
        gateway.tick()
        self.assertFalse(backend.members['left'].enabled)
        self.assertTrue(backend.members['right'].enabled)
        self.assertEqual(backend.operation_results['operation'], 'disable_left')

    def test_dual_hands_disable_is_recorded(self):
        backend = MockDualBackend()
        gateway = Gateway(backend, KEY, port=0)
        self.addCleanup(gateway.socket.close)
        gateway.request('dh')
        gateway.tick()
        self.assertIn('hands_disable_verified=True', gateway.reason)
        self.assertEqual(backend.operation_results['operation'], 'disable_hands')

    def test_hand_only_gateway_never_enables_or_commands_arm(self):
        backend = MockBackend('right', True, True)
        gateway = Gateway(backend, KEY, port=0)
        self.addCleanup(gateway.socket.close)
        gateway.gate.hello(dict(client_nonce='b'*32), PEER, self.now, False)
        state = gateway.gate.state(self.now, {})
        gateway.gate.accept(dict(type='target', seq=1, session=state['session'],
            lease=state['lease'], side='right', contract_id=backend.contract['id'],
            hand_unit=[140]*10), PEER, self.now)
        gateway.request('e')
        gateway.tick()
        self.assertEqual(gateway.mode, 'CALIBRATING')
        state = gateway.gate.state(self.now, {})
        gateway.gate.accept(dict(type='target', seq=2, session=state['session'],
            lease=state['lease'], side='right', contract_id=backend.contract['id'],
            hand_unit=[140]*10), PEER, self.now)
        gateway.tick()
        self.assertEqual(gateway.mode, 'ACTIVE')
        self.assertFalse(backend.enabled)
        self.assertTrue(backend.hand_enabled)
        self.assertEqual(backend.q, [0.]*7)
        self.assertEqual(backend.hand, [140]*10)

    def test_hand_only_hardware_asserts_stop_if_arm_becomes_enabled(self):
        backend = HardwareBackend.__new__(HardwareBackend)
        backend.hand_only = True
        backend.arm = Mock()
        backend.snapshot = Mock(return_value=dict(
            enable_states=[False, False, True, False, False, False, False],
            controller_fault=None, driver_fault=None))
        with self.assertRaisesRegex(RuntimeError, 'arm enable detected'):
            backend.prepare_step(lambda: False)
        backend.arm.emergency_stop.assert_called_once_with()

    def test_arm_only_hardware_hold_uses_checked_braking_without_position_target(self):
        backend = HardwareBackend.__new__(HardwareBackend)
        backend.hand_only = False
        backend.hand = None
        backend.arm = Mock()
        backend.arm.hold_command_trajectory.return_value = True
        backend.prepare_step = Mock()
        backend.send_arm = Mock()
        backend.step({'hold': True}, lambda: False)
        backend.prepare_step.assert_called_once()
        backend.arm.hold_command_trajectory.assert_called_once_with()
        backend.send_arm.assert_not_called()


class NetworkTests(unittest.TestCase):
    def test_udp_hold_requires_active_session_and_keeps_watchdog(self):
        gw = Gateway(MockBackend('right', False), KEY, port=0)
        worker = threading.Thread(target=gw.run)
        worker.start()
        with tempfile.TemporaryDirectory() as directory:
            keyfile = Path(directory) / 'key'
            keyfile.write_bytes(KEY)
            client = RobotClient(*gw.address, keyfile)
            try:
                client.connect()
                client.send_hold()
                deadline = time.monotonic() + 1.
                while time.monotonic() < deadline and gw.rejected_packets == 0:
                    client.receive()
                self.assertGreater(gw.rejected_packets, 0)
                self.assertIsNone(gw.gate.target)
                client.receive()
                client.send_target([0.] * 7)
                deadline = time.monotonic() + 1.
                while time.monotonic() < deadline and gw.gate.target is None:
                    client.receive()
                self.assertIsNotNone(gw.gate.target)
                gw.request('e')
                deadline = time.monotonic() + 1.
                while time.monotonic() < deadline and gw.mode != 'ACTIVE':
                    client.receive()
                    client.send_target([0.] * 7)
                self.assertEqual(gw.mode, 'ACTIVE')
                client.receive()
                client.send_hold()
                deadline = time.monotonic() + 1.
                while time.monotonic() < deadline and gw.gate.target != {'hold': True}:
                    client.receive()
                self.assertEqual(gw.gate.target, {'hold': True})
                self.assertEqual(gw.mode, 'ACTIVE')
                time.sleep(.26)
                self.assertEqual(gw.mode, 'FAULT')
            finally:
                client.socket.close()
                gw.request('q')
                worker.join(2.)
                self.assertFalse(worker.is_alive())

    def test_real_udp_client_gateway_and_timeout(self):
        gw = Gateway(MockBackend(), KEY, port=0)
        worker = threading.Thread(target=gw.run)
        worker.start()
        with tempfile.TemporaryDirectory() as directory:
            keyfile = Path(directory)/'key'
            keyfile.write_bytes(KEY)
            client = RobotClient(*gw.address, keyfile)
            try:
                state = client.connect()
                self.assertEqual(state['mode'], 'IDLE')
                for _ in range(4):
                    client.receive()
                    if client.latest['state_send_gap_s'] is not None:
                        break
                self.assertIsNotNone(client.latest['state_send_gap_s'])
                self.assertGreater(client.latest['state_send_gap_max_s'], 0)
                client.send_target([0.]*7)
                until = time.monotonic()+1.
                while time.monotonic() < until:
                    state = client.receive()
                    if state['accepted_seq'] >= 1:
                        break
                gw.request('e')
                for _ in range(8):
                    client.receive()
                    client.send_target([.05]*7)
                self.assertTrue(gw.backend.enabled)
                time.sleep(.26)
                self.assertEqual(gw.mode, 'FAULT')
                self.assertFalse(gw.backend.enabled)
            finally:
                client.socket.close()
                gw.request('q')
                worker.join(2.)
                self.assertFalse(worker.is_alive())


class CameraTests(unittest.TestCase):
    def test_latest_only_and_header_sanitization(self):
        images = LatestImage()
        images.publish(b'\xff\xd8one\xff\xd9', 1, 'a\r\nb')
        images.publish(b'\xff\xd8two\xff\xd9', 2, 'a\r\nb')
        self.assertEqual(images.sequence, 2)
        self.assertEqual(images.frame[:3], (b'\xff\xd8two\xff\xd9', 2, 'a__b'))
        with self.assertRaises(ValueError):
            images.publish(b'not JPEG', 1, '')

    def test_http_jpeg_metadata(self):
        images = LatestImage()
        jpeg = b'\xff\xd8test\xff\xd9'
        images.publish(jpeg, 1234, 'color')
        server = ThreadingHTTPServer(('127.0.0.1', 0), handler_for(images))
        worker = threading.Thread(target=server.serve_forever)
        worker.start()
        connection = http.client.HTTPConnection(*server.server_address, timeout=1)
        try:
            connection.request('GET', '/stream.mjpg')
            response = connection.getresponse()
            self.assertEqual(response.status, 200)
            headers = b''
            while not headers.endswith(b'\r\n\r\n'):
                headers += response.read(1)
            self.assertIn(b'X-Capture-Unix-Ns: 1234', headers)
            self.assertEqual(response.read(len(jpeg)), jpeg)
        finally:
            connection.close()
            server.shutdown()
            server.server_close()
            worker.join(1)


if __name__ == '__main__':
    unittest.main()
