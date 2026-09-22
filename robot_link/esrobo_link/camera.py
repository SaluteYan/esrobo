"""Separate latest-frame MJPEG capture/relay; localhost HTTP via SSH tunnel.

No image traffic or camera dependency enters the joint control process.
"""
import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import threading
import time


class LatestImage:
    def __init__(self):
        self.condition = threading.Condition()
        self.sequence = 0
        self.frame = None
        self.clients = threading.BoundedSemaphore(2)

    def publish(self, jpeg, stamp_ns, frame_id):
        if len(jpeg) > 4 * 1024 * 1024 or not jpeg.startswith(b"\xff\xd8") or not jpeg.endswith(b"\xff\xd9"):
            raise ValueError("expected JPEG of at most 4 MiB")
        # Header injection is not allowed through a ROS frame_id.
        frame_id = str(frame_id).encode("ascii", "replace").decode().replace("\r", "_").replace("\n", "_")[:100]
        with self.condition:
            self.sequence += 1
            self.frame = (bytes(jpeg), int(stamp_ns), frame_id, time.monotonic())
            self.condition.notify_all()


def handler_for(images):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def do_GET(self):
            if self.path != "/stream.mjpg":
                self.send_error(404)
                return
            if not images.clients.acquire(blocking=False):
                self.send_error(503, "two viewers maximum")
                return
            try:
                self.connection.settimeout(2.0)
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                last_seq = 0
                while True:
                    with images.condition:
                        changed = images.condition.wait_for(lambda: images.sequence != last_seq, timeout=2.0)
                        if not changed:
                            return  # stale acquisition ends stream, never loops an old image
                        last_seq, frame = images.sequence, images.frame
                    jpeg, stamp, frame_id, received = frame
                    if time.monotonic() - received > 1.0:
                        return
                    headers = (f"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: {len(jpeg)}\r\n"
                               f"X-Frame-Sequence: {last_seq}\r\nX-Capture-Unix-Ns: {stamp}\r\n"
                               f"X-Frame-Id: {frame_id}\r\n\r\n")
                    self.wfile.write(headers.encode() + jpeg + b"\r\n")
                    self.wfile.flush()
            except (OSError, TimeoutError):
                pass
            finally:
                images.clients.release()
    return Handler


def realsense_capture(images, args):
    import cv2
    import numpy as np
    import pyrealsense2 as rs
    pipeline, config = rs.pipeline(), rs.config()
    if args.serial:
        config.enable_device(args.serial)
    config.enable_stream(rs.stream.color, args.width, args.height, rs.format.bgr8, args.fps)
    pipeline.start(config)
    try:
        while True:
            frame = pipeline.wait_for_frames(timeout_ms=2000).get_color_frame()
            if not frame:
                continue
            stamp = time.time_ns()  # host arrival time; explicitly not sensor exposure time
            ok, jpeg = cv2.imencode(".jpg", np.asanyarray(frame.get_data()), [cv2.IMWRITE_JPEG_QUALITY, 75])
            if ok:
                images.publish(jpeg.tobytes(), stamp, "color_host_arrival")
    finally:
        pipeline.stop()


def ros_capture(images, args):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage
    rclpy.init()
    node = Node("esrobo_robot_link_camera")

    def on_frame(msg):
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        try:
            images.publish(bytes(msg.data), stamp, msg.header.frame_id)
        except ValueError as exc:
            node.get_logger().warning(str(exc))
    node.create_subscription(CompressedImage, args.topic, on_frame, qos_profile_sensor_data)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--source", choices=("realsense", "ros"), required=True)
    ap.add_argument("--topic", default="/camera/color/image_raw/compressed")
    ap.add_argument("--serial", default="")
    ap.add_argument("--port", type=int, default=18080)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, choices=(6, 15, 30), default=15)
    args = ap.parse_args()
    images = LatestImage()
    server = ThreadingHTTPServer(("127.0.0.1", args.port), handler_for(images))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f"Camera: http://127.0.0.1:{args.port}/stream.mjpg (SSH tunnel required)", flush=True)
    try:
        (realsense_capture if args.source == "realsense" else ros_capture)(images, args)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        server.server_close()


if __name__ == "__main__":
    main()
