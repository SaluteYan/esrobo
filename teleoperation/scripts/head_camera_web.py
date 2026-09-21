#!/usr/bin/env python3
"""Loopback ROS bridge for the head camera page. No automatic hardware enable."""
import argparse
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import sys
import threading
import time
from urllib.parse import urlsplit

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.signals import SignalHandlerOptions
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool
from servo_driver.srv import HeadJog

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from esrobo_teleop.debug.head_control import HeadControl


class RosHead:
    def __init__(self):
        self.node = rclpy.create_node('head_camera_web')
        self.guard = threading.Lock()
        self.axes = {}
        self.received = None
        self.sequence = 0
        self.images = {}
        self.jpeg = {}
        self.bridge = CvBridge()
        self.subs = [self.node.create_subscription(DiagnosticArray, '/head/state', self.on_state, 1)]
        for kind in ('color', 'depth'):
            self.subs.append(self.node.create_subscription(
                Image, f'/camera/{kind}/image_raw', lambda m, k=kind: self.on_image(k, m), 1))
        self.enable = self.node.create_client(SetBool, '/head/adjust_enable')
        self.move = self.node.create_client(HeadJog, '/head/jog')

    def on_state(self, message):
        with self.guard:
            self.axes = {s.hardware_id: dict({v.key: v.value for v in s.values}, message=s.message)
                         for s in message.status}
            self.received = time.monotonic()
            self.sequence += 1

    def on_image(self, kind, message):
        with self.guard:
            self.images[kind] = (message, time.monotonic())

    def state(self):
        with self.guard:
            return dict(axes=self.axes.copy(), sequence=self.sequence,
                        age_s=None if self.received is None else time.monotonic() - self.received)

    def call(self, client, request):
        if not client.wait_for_service(timeout_sec=.5):
            raise RuntimeError('ROS 控制服务不可用')
        future = client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(2):
            raise RuntimeError('ROS 应答超时，已接受的小步可能仍在执行')
        result = future.result()
        if not result.success:
            raise RuntimeError(result.message)

    def gate(self, enabled):
        self.call(self.enable, SetBool.Request(data=enabled))

    def jog(self, sid, delta, speed):
        self.call(self.move, HeadJog.Request(servo_id=sid, delta_ticks=delta, speed=speed))

    def encode(self):
        with self.guard:
            images = self.images.copy()
        for kind, (message, received) in images.items():
            if kind in self.jpeg and self.jpeg[kind][1] == received:
                continue
            try:
                if kind == 'color':
                    frame = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
                else:
                    raw = self.bridge.imgmsg_to_cv2(message, desired_encoding='passthrough')
                    depth = raw.astype(np.float32) * (1000 if message.encoding == '32FC1' else 1)
                    valid = np.isfinite(depth) & (depth > 0)
                    scaled = np.clip(np.nan_to_num(depth, nan=0) / 3000 * 255, 0, 255).astype(np.uint8)
                    frame = cv2.applyColorMap(scaled, cv2.COLORMAP_TURBO)
                    frame[~valid] = 0
                ok, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                if ok:
                    with self.guard:
                        self.jpeg[kind] = (encoded.tobytes(), received)
            except Exception as exc:
                self.node.get_logger().error(f'{kind} preview: {exc}')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=8766)
    parser.add_argument('--standalone-viewer', action='store_true',
                        help='Start the shared web site on 8765 when no PICO bridge is running')
    args = parser.parse_args()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    adapter = RosHead()
    control = HeadControl(adapter)
    stop = threading.Event()
    assets = Path(__file__).resolve().parents[1] / 'web/pico_skeleton_viewer'

    class Handler(BaseHTTPRequestHandler):
        def send(self, code, body, content_type='application/json'):
            self.send_response(code)
            self.send_header('Content-Type', content_type)
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_GET(self):
            path = urlsplit(self.path).path
            if path == '/api/head/state':
                data = adapter.state()
                data['control'] = control.snapshot()
                with adapter.guard:
                    data['images'] = {k: {'age_s': time.monotonic() - v[1]} for k, v in adapter.jpeg.items()}
                self.send(200, json.dumps(data).encode())
            elif path in ('/api/head/color.jpg', '/api/head/depth.jpg'):
                kind = path.rsplit('/', 1)[-1].split('.')[0]
                with adapter.guard:
                    picture = adapter.jpeg.get(kind)
                if picture and time.monotonic() - picture[1] <= 1:
                    self.send(200, picture[0], 'image/jpeg')
                else:
                    self.send(503, b'{"error":"image unavailable or stale"}')
            elif path in ('/', '/head.html', '/head.js', '/head.css'):
                name = 'head.html' if path == '/' else path[1:]
                mime = {'html': 'text/html; charset=utf-8', 'js': 'text/javascript', 'css': 'text/css'}
                self.send(200, (assets / name).read_bytes(), mime[name.split('.')[-1]])
            else:
                self.send(404, b'{}')

        def do_POST(self):
            origin = urlsplit(self.headers.get('Origin', ''))
            if (origin.netloc != self.headers.get('Host') or origin.scheme != 'http'
                    or self.headers.get('X-Head-Control') != '1'
                    or self.headers.get('Content-Type') != 'application/json'):
                self.send(403, b'{"error":"same-origin JSON request required"}')
                return
            try:
                size = int(self.headers.get('Content-Length', '0'))
                if not 0 < size <= 1024:
                    raise ValueError('请求大小无效')
                data = json.loads(self.rfile.read(size))
                action = urlsplit(self.path).path.removeprefix('/api/head/')
                if action == 'heartbeat':
                    control.heartbeat(data.get('session'))
                elif action == 'lock':
                    control.lock()
                else:
                    control.submit(action, data.get('session'), data.get('id'), data.get('target'))
                self.send(200, b'{"ok":true}')
            except (ValueError, RuntimeError, TypeError, AttributeError) as exc:
                self.send(409, json.dumps({'error': str(exc)}).encode())

        def log_message(self, *_):
            pass

    server = ThreadingHTTPServer(('127.0.0.1', args.port), Handler)
    server.daemon_threads = True
    viewer = None
    if args.standalone_viewer:
        from esrobo_teleop.debug.skeleton_viewer import SkeletonViewerServer
        if args.port != 8766:
            raise ValueError('Standalone shared viewer requires backend port 8766')
        viewer = SkeletonViewerServer('127.0.0.1', 8765, diagnostics_port=0)
        viewer.start()
    ros_thread = threading.Thread(target=rclpy.spin, args=(adapter.node,), daemon=True)
    http_thread = threading.Thread(target=server.serve_forever, daemon=True)
    def watchdog():
        while not stop.wait(.1):
            control.expire()
    watch_thread = threading.Thread(target=watchdog, daemon=True)
    ros_thread.start()
    http_thread.start()
    watch_thread.start()
    print(f'Head camera bridge: http://127.0.0.1:{args.port}/head.html (no automatic enable)', flush=True)
    try:
        while not stop.wait(.1):
            adapter.encode()
    except KeyboardInterrupt:
        pass
    finally:
        control.close()
        stop.set()
        server.shutdown()
        server.server_close()
        if viewer:
            viewer.stop()
        rclpy.shutdown()
        ros_thread.join(timeout=2)
        adapter.node.destroy_node()


if __name__ == '__main__':
    main()
