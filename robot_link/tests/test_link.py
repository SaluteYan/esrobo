import io
import json
import socket
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

from esrobo_link.protocol import ProtocolError, pack, unpack, validate_target
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


class SessionTests(Fixture):
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


class NetworkTests(unittest.TestCase):
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
