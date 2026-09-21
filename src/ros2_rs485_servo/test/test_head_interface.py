"""PTY hardware simulator: no real serial device is opened.

Run after building servo_driver and sourcing install/setup.bash with system Python.
"""
import os
import pty
import select
import subprocess
import threading
import time
import unittest

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from std_srvs.srv import SetBool
from servo_driver.srv import HeadJog


class HeadInterfaceTest(unittest.TestCase):
    def setUp(self):
        self.master, self.slave = pty.openpty()
        self.commands = []
        self.pos = {1: 1500, 2: 3450}
        self.torque = {1: 0, 2: 0}
        self.respond = True
        self.freeze = False
        self.position_error = 0
        self.drop_write_ack = False
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self.emulate, daemon=True)
        self.thread.start()
        self.node = rclpy.create_node('head_interface_test')
        self.latest = None
        self.sub = self.node.create_subscription(DiagnosticArray, '/head/state', self.state, 10)
        self.enable = self.node.create_client(SetBool, '/head/adjust_enable')
        self.jog = self.node.create_client(HeadJog, '/head/jog')
        self.process = None

    def state(self, msg):
        self.latest = msg

    def emulate(self):
        buffer = bytearray()
        while not self.stop.is_set():
            if not select.select([self.master], [], [], .02)[0]:
                continue
            buffer.extend(os.read(self.master, 256))
            while len(buffer) >= 4 and len(buffer) >= buffer[3] + 4:
                size = buffer[3] + 4
                packet = bytes(buffer[:size])
                del buffer[:size]
                assert packet[:2] == b'\xff\xff' and sum(packet[2:]) % 256 == 255
                self.commands.append(packet)
                sid, instruction, addr = packet[2], packet[4], packet[5]
                if not self.respond:
                    continue
                if instruction == 2:
                    data = ([self.pos[sid] & 255, self.pos[sid] >> 8]
                            if addr == 56 else [self.torque[sid]])
                else:
                    assert instruction == 3
                    data = []
                    if addr == 40:
                        self.torque[sid] = packet[6]
                    elif addr == 42 and not self.freeze:
                        self.pos[sid] = packet[6] + 256 * packet[7] + self.position_error
                if instruction == 3 and self.drop_write_ack:
                    continue
                reply = bytes([sid, len(data) + 2, 0] + data)
                os.write(self.master, b'\xff\xff' + reply + bytes([~sum(reply) & 255]))

    def start(self, motion):
        self.process = subprocess.Popen([
            'ros2', 'run', 'servo_driver', 'servo_node', '--ros-args',
            '-p', 'port:=' + os.ttyname(self.slave),
            '-p', 'allow_motion:=' + str(motion).lower()],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            start_new_session=True)
        self.assertTrue(self.enable.wait_for_service(timeout_sec=8))
        self.spin(.5)
        self.assertTrue(self.commands)
        self.assertTrue(all(p[4] == 2 for p in self.commands))

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=.05)

    def call(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=4)
        self.assertTrue(future.done())
        return future.result()

    def gate(self, value):
        return self.call(self.enable, SetBool.Request(data=value))

    def move(self, sid=1, delta=10, speed=5):
        return self.call(self.jog, HeadJog.Request(servo_id=sid, delta_ticks=delta, speed=speed))

    def test_read_only(self):
        self.start(False)
        self.assertFalse(self.gate(True).success)
        self.assertFalse(self.move().success)
        self.assertTrue(all(p[4] == 2 for p in self.commands))

    def test_adjust_lock_and_limits(self):
        self.start(True)
        self.assertTrue(self.gate(True).success)
        writes = [p for p in self.commands if p[4] == 3]
        self.assertEqual([p[5] for p in writes], [42, 42, 40, 40])
        self.assertEqual(self.torque, {1: 1, 2: 1})
        for _ in range(20):
            self.assertTrue(self.move().success)
        self.assertEqual(self.pos[1], 1700)
        self.assertTrue(self.move().success)
        self.assertEqual(self.pos[1], 1710)
        self.assertFalse(self.move(speed=0).success)
        self.assertTrue(self.gate(True).success)
        self.assertFalse(self.move(delta=21).success)
        self.assertTrue(self.gate(True).success)
        self.assertFalse(self.move(sid=3).success)
        self.assertTrue(self.gate(True).success)
        self.assertTrue(self.gate(False).success)
        count = len([p for p in self.commands if p[4] == 3])
        self.assertFalse(self.move().success)
        self.assertEqual(len([p for p in self.commands if p[4] == 3]), count)
        self.assertEqual(self.torque, {1: 1, 2: 1})

    def test_timeout_and_no_queue(self):
        self.start(True)
        self.assertTrue(self.gate(True).success)
        self.freeze = True
        self.assertTrue(self.move().success)
        self.assertFalse(self.move().success)
        self.assertFalse(self.gate(True).success)
        self.spin(5.2)
        self.assertIn('timeout', self.latest.status[0].message)
        self.assertEqual(self.torque, {1: 1, 2: 1})

    def test_lost_feedback(self):
        self.start(True)
        self.assertTrue(self.gate(True).success)
        self.respond = False
        self.spin(.8)
        count = len([p for p in self.commands if p[4] == 3])
        self.assertFalse(self.move().success)
        self.assertEqual(len([p for p in self.commands if p[4] == 3]), count)
        self.assertEqual(self.torque, {1: 1, 2: 1})

    def test_lost_write_ack_locks_even_if_motor_moved(self):
        self.start(True)
        self.assertTrue(self.gate(True).success)
        self.drop_write_ack = True
        self.assertFalse(self.move().success)
        self.assertEqual(self.pos[1], 1510)
        count = len([p for p in self.commands if p[4] == 3])
        self.assertFalse(self.move().success)
        self.assertEqual(len([p for p in self.commands if p[4] == 3]), count)
        self.assertEqual(self.torque, {1: 1, 2: 1})

    def test_absolute_limit(self):
        self.pos[1] = 1005
        self.start(True)
        self.assertTrue(self.gate(True).success)
        self.assertFalse(self.move(delta=-10).success)
        self.assertEqual(self.pos[1], 1005)

    def test_four_tick_residual_arrives_but_six_does_not(self):
        self.start(True)
        self.assertTrue(self.gate(True).success)
        self.position_error = 4
        self.assertTrue(self.move().success)
        self.spin(.3)
        values = {v.key: v.value for v in self.latest.status[0].values}
        self.assertEqual(values['pending'], 'false')
        self.assertEqual(values['arrival_tolerance_ticks'], '5')
        self.position_error = 6
        self.assertTrue(self.move().success)
        self.spin(.3)
        values = {v.key: v.value for v in self.latest.status[0].values}
        self.assertEqual(values['pending'], 'true')
        self.assertFalse(self.move().success)

    def test_web_continuous_adjustment(self):
        from pathlib import Path
        import json
        import socket
        import signal
        from urllib.request import Request, urlopen
        script = Path(__file__).resolve().parents[3] / 'teleoperation/scripts/head_camera_web.py'
        self.start(True)
        with socket.socket() as sock:
            sock.bind(('127.0.0.1', 0))
            port = sock.getsockname()[1]
        base = f'http://127.0.0.1:{port}'
        service = subprocess.Popen(['/usr/bin/python3', str(script), '--port', str(port)],
                                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                   start_new_session=True)
        def request(action=None, **data):
            if action is None:
                return json.load(urlopen(base + '/api/head/state', timeout=3))
            body = json.dumps(dict(session='integration-test', **data)).encode()
            return json.load(urlopen(Request(base + '/api/head/' + action, data=body, headers={
                'Origin': base, 'Content-Type': 'application/json', 'X-Head-Control': '1'}), timeout=4))
        def wait_idle():
            deadline = time.monotonic() + 6
            while time.monotonic() < deadline:
                request('heartbeat')
                state = request()
                if not state['control']['busy']:
                    return state
                time.sleep(.1)
            self.fail('web command timeout')
        try:
            deadline = time.monotonic() + 8
            while True:
                try:
                    if request()['axes']:
                        break
                except OSError:
                    pass
                self.assertLess(time.monotonic(), deadline)
                time.sleep(.1)
            self.assertTrue(all(p[4] == 2 for p in self.commands))
            request('enable')
            self.assertEqual(wait_idle()['control']['owner'], 'integration-test')
            for target in (1800, 1500):
                request('move', id=1, target=target)
                completed = wait_idle()
                self.assertEqual(completed['control']['owner'], 'integration-test', completed)
                self.assertLessEqual(abs(self.pos[1] - target), 5)
            request('lock')
            self.assertIsNone(request()['control']['owner'])
            self.assertEqual(self.torque, {1: 1, 2: 1})
        finally:
            os.killpg(service.pid, signal.SIGINT)
            service.wait(timeout=8)

    def tearDown(self):
        if self.process:
            import signal
            os.killpg(self.process.pid, signal.SIGINT)
            self.process.wait(timeout=5)
        self.node.destroy_node()
        self.stop.set()
        self.thread.join(timeout=1)
        os.close(self.master)
        os.close(self.slave)


if __name__ == '__main__':
    rclpy.init()
    try:
        unittest.main()
    finally:
        rclpy.shutdown()
