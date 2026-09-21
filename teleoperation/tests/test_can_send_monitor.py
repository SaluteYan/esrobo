import errno
import json
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock

from esrobo_teleop.debug.can_send_monitor import CanSendMonitor


class FakeComm:
    def __init__(self):
        self.send_bus = NS(send=Mock(return_value=None))

    def send(self, message):
        try:
            return self.send_bus.send(message)
        except OSError as exc:
            if exc.errno not in (errno.ENOBUFS, errno.ENETDOWN):
                raise
            # Reproduce SDK silent failure and RX clearing last_error.
            self.last_error = None


class CanSendMonitorTests(unittest.TestCase):
    def setUp(self):
        self.comm = FakeComm()
        self.native = self.comm.send_bus.send
        self.monitor = CanSendMonitor(self.comm)
        self.message = NS(arbitration_id=0x155, data=bytes(range(8)))

    def test_success_records_actual_payload_without_extra_sends(self):
        records = []
        self.monitor.sink = records.append
        self.assertIsNone(self.comm.send(self.message))
        self.native.assert_called_once_with(self.message, timeout=.02)
        self.assertEqual(records[0]['can_id'], 0x155)
        self.assertEqual(records[0]['data_hex'], '0001020304050607')
        self.assertEqual(records[0]['outcome'], 'socket_send_returned')
        self.assertEqual(self.monitor.snapshot()['attempts'], 1)
        json.dumps(self.monitor.snapshot(), allow_nan=False)

    def test_swallowed_errors_abort_multiframe_target_without_retry(self):
        for code in (errno.ENOBUFS, errno.ENETDOWN):
            with self.subTest(code=code):
                self.setUp()
                self.native.side_effect = OSError(code, 'simulated socket failure')
                with self.assertRaisesRegex(RuntimeError, 'CAN socket send failed'):
                    for _ in range(4):
                        self.comm.send(self.message)
                self.assertEqual(self.native.call_count, 1)
                self.assertEqual(self.monitor.failures, 1)
                self.assertEqual(self.monitor.last_frame['outcome'], 'socket_send_failed')
                self.assertEqual(self.monitor.snapshot()['last_control_frames']['0x155']['outcome'],
                                 'socket_send_failed')

    def test_stop_preserves_bounded_control_evidence_and_snapshot_is_independent(self):
        for cid in (0x471, 0x151, 0x155, 0x156, 0x157, 0x170,
                    0x181, 0x182, 0x183, 0x184, 0x185, 0x186, 0x187, 0x150):
            self.comm.send(NS(arbitration_id=cid, data=bytes(range(8))))
        snapshot = self.monitor.snapshot()
        self.assertEqual(snapshot['last_frame']['can_id'], 0x150)
        self.assertEqual(set(snapshot['last_control_frames']),
                         {'0x471', '0x151', '0x155', '0x156', '0x157', '0x170',
                          '0x181', '0x182', '0x183', '0x184', '0x185', '0x186', '0x187'})
        mode = snapshot['last_control_frames']['0x151']
        self.assertLessEqual(mode['monotonic_s'],
                             snapshot['last_control_frames']['0x155']['monotonic_s'])
        mode['data_hex'] = 'mutated'
        self.assertEqual(self.monitor.snapshot()['last_control_frames']['0x151']['data_hex'],
                         '0001020304050607')
        recent = snapshot['recent_control_frames']
        self.assertEqual([event['can_id'] for event in recent],
                         [0x471, 0x151, 0x155, 0x156, 0x157, 0x170,
                          0x181, 0x182, 0x183, 0x184, 0x185, 0x186, 0x187])
        recent[0]['data_hex'] = 'mutated'
        self.assertEqual(self.monitor.snapshot()['recent_control_frames'][0]['data_hex'],
                         '0001020304050607')
        self.assertEqual(self.native.call_count, 14)

    def test_recent_control_history_is_bounded_and_keeps_mode_sequence(self):
        for i in range(70):
            cid = 0x151 if i % 2 == 0 else 0x181
            self.comm.send(NS(arbitration_id=cid, data=bytes([i % 256])))
        recent = self.monitor.snapshot()['recent_control_frames']
        self.assertEqual(len(recent), 64)
        self.assertEqual(recent[0]['data_hex'], bytes([6]).hex())
        self.assertEqual(recent[-1]['data_hex'], bytes([69]).hex())

    def test_other_errors_propagate_and_logging_failure_does_not_mask(self):
        self.monitor.sink = Mock(side_effect=ValueError('bad diagnostic'))
        self.comm.send(self.message)
        self.native.side_effect = OSError(errno.EINVAL, 'bad frame')
        with self.assertRaises(OSError):
            self.comm.send(self.message)
        self.assertEqual(self.monitor.failures, 1)

    def test_reconnect_rebinds_once_and_unmonitored_bus_is_rejected(self):
        self.monitor.bind_bus()
        self.comm.send(self.message)
        self.assertEqual(self.monitor.attempts, 1)
        replacement = Mock(return_value=None)
        self.comm.send_bus = NS(send=replacement)
        with self.assertRaisesRegex(RuntimeError, 'bus changed'):
            self.comm.send(self.message)
        replacement.assert_not_called()
        self.monitor.bind_bus()
        self.assertEqual(self.monitor.snapshot()['last_control_frames'], {})
        self.monitor.bind_bus()
        self.comm.send(self.message)
        replacement.assert_called_once_with(self.message, timeout=.02)
        self.assertEqual(self.monitor.attempts, 2)

    def test_explicit_send_timeout_is_preserved(self):
        self.comm.send_bus.send(self.message, timeout=.5)
        self.native.assert_called_once_with(self.message, timeout=.5)

    def test_success_after_failure_remains_sendable_for_emergency_disable(self):
        self.native.side_effect = OSError(errno.ENOBUFS, 'full')
        with self.assertRaises(RuntimeError):
            self.comm.send(self.message)
        self.native.side_effect = None
        self.comm.send(self.message)
        self.assertEqual(self.monitor.attempts, 2)
        self.assertEqual(self.monitor.failures, 1)
