import json
import socket
import struct
import threading
import unittest
from types import SimpleNamespace as NS
from unittest import mock

from esrobo_teleop.debug.arm_probe import ArmProbe, controller_snapshot, plain


class ArmProbeTests(unittest.TestCase):
    def test_snapshot_reads_cache_only_and_preserves_individual_stamps(self):
        parser = NS(arm_status=NS(timestamp=10., msg=NS(ctrl_mode=1, motion_status=1)),
                    joint_12=NS(timestamp=11., msg=NS(joint_1=.1, joint_2=.2)),
                    joint_7=NS(timestamp=12., msg=NS(joint_7=.7)),
                    driver_state_1=NS(timestamp=13., msg=NS(foc_status=NS(driver_enable_status=True))),
                    motor_state_1=NS(timestamp=14., msg=NS(position=.1, velocity=.2, current=3., torque=4.)))
        robot = NS(_parser=parser)
        driver = NS(_arm=NS(_robot=robot), safety_fault_reason=None)
        result = controller_snapshot(driver)
        self.assertEqual(result['joint_groups']['joint_12']['stamp_s'], 11.)
        self.assertEqual(result['joint_groups']['joint_7']['stamp_s'], 12.)
        self.assertTrue(result['drivers'][0]['fields']['foc_status.driver_enable_status'])
        self.assertEqual(result['motors'][0]['fields']['velocity'], .2)
        self.assertEqual(parser.motor_state_1.msg.current, 3.)
        json.dumps(result, allow_nan=False)
        self.assertIsNone(result['motors'][1])

    def test_absent_fields_and_nonfinite_values_are_json_safe(self):
        self.assertEqual(plain([float('nan'),float('inf'),2.]), [None,None,2.])
        result = controller_snapshot(NS(_arm=NS(_robot=NS())))
        json.dumps(result, allow_nan=False)

    def test_passive_receiver_records_local_frames_and_never_sends(self):
        probe = ArmProbe.__new__(ArmProbe)
        probe.stop = threading.Event()
        probe.channel = 'can_test'
        probe.raw = mock.Mock(dropped=0)
        probe.submit = mock.Mock()
        bus = mock.MagicMock()
        def receive(*args):
            probe.stop.set()
            return (struct.pack('=IB3x8s', 0x2A1, 8, bytes(range(8))),
                    [(socket.SOL_SOCKET,40,struct.pack('=I',3))],socket.MSG_DONTROUTE,())
        bus.recvmsg.side_effect = receive
        bus.__enter__.return_value = bus
        with mock.patch('socket.socket',return_value=bus):
            probe._receive()
        bus.bind.assert_called_once_with(('can_test',))
        bus.send.assert_not_called()
        bus.sendto.assert_not_called()
        row=probe.raw.submit.call_args.args[0]
        self.assertEqual(row['can_id'],0x2A1)
        self.assertEqual(row['data_hex'],'0001020304050607')
        self.assertTrue(row['local_origin'])
        self.assertEqual(row['kernel_drop_count'],3)

    def test_receiver_failure_is_diagnostic_only(self):
        probe = ArmProbe.__new__(ArmProbe)
        probe.submit = mock.Mock()
        with mock.patch('socket.socket',side_effect=OSError('unavailable')):
            probe._receive()
        self.assertEqual(probe.submit.call_args.args[0]['event'],'passive_can_error')


if __name__ == '__main__':
    unittest.main()
