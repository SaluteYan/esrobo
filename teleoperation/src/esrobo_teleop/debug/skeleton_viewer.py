"""PICO diagnostics server with an isolated loopback proxy for the head camera page."""

from __future__ import annotations

from collections import deque
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import socket
import threading
import time
from typing import Any
from urllib.parse import urlsplit
from urllib.request import Request, urlopen
from urllib.error import HTTPError, URLError


DEFAULT_SOURCE_TO_ROBOT_ROTATION = (
    0.0, 0.0, -1.0,
    -1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
)


class ReusableThreadingHTTPServer(ThreadingHTTPServer):
    """Permit an immediate viewer restart after the previous bridge exits."""

    allow_reuse_address = True
    daemon_threads = True


class SkeletonViewerState:
    """Keep only the latest body packet and source-update timing statistics."""

    def __init__(self, source_to_robot_rotation=DEFAULT_SOURCE_TO_ROBOT_ROTATION):
        self._lock = threading.Lock()
        self._packet: dict[str, Any] | None = None
        self._received_monotonic = 0.0
        self._source_updates: deque[float] = deque(maxlen=240)
        self._last_signature: tuple[Any, ...] | None = None
        self._rotation = [float(value) for value in source_to_robot_rotation]
        self._diagnostics: dict[str, Any] | None = None
        self._diagnostics_monotonic = 0.0

    @staticmethod
    def _signature(packet: dict[str, Any]) -> tuple[Any, ...]:
        frames = packet.get("frames", {})
        arm_points = []
        if isinstance(frames, dict):
            for name in (
                "left_shoulder", "left_elbow", "left_wrist",
                "right_shoulder", "right_elbow", "right_wrist",
            ):
                frame = frames.get(name, {})
                pos = frame.get("pos", []) if isinstance(frame, dict) else []
                arm_points.extend(round(float(value), 5) for value in pos[:3])
        body_timestamp = int(packet.get("xrt_body_timestamp_ns", 0))
        joint_timestamp = int(packet.get("xrt_body_joint_timestamp_ns", 0))
        timestamp = body_timestamp if body_timestamp > 0 else joint_timestamp
        return (timestamp, *arm_points)

    def update(self, packet: dict[str, Any]) -> None:
        now = time.monotonic()
        signature = self._signature(packet)
        with self._lock:
            self._packet = packet
            self._received_monotonic = now
            if signature != self._last_signature:
                self._source_updates.append(now)
                self._last_signature = signature

    def snapshot(self) -> dict[str, Any]:
        now = time.monotonic()
        with self._lock:
            while self._source_updates and now - self._source_updates[0] > 1.0:
                self._source_updates.popleft()
            packet = self._packet
            age_s = now - self._received_monotonic if packet is not None else None
            update_hz = 0.0
            if len(self._source_updates) >= 2:
                elapsed = self._source_updates[-1] - self._source_updates[0]
                update_hz = (len(self._source_updates) - 1) / max(elapsed, 1.0e-9)
            return {
                "connected": age_s is not None and age_s <= 0.5,
                "age_ms": None if age_s is None else round(age_s * 1000.0, 1),
                "source_update_hz": round(update_hz, 1),
                "source_to_robot_rotation": self._rotation,
                "packet": packet,
                "diagnostics": self._diagnostics,
                "diagnostics_age_ms": (
                    None
                    if self._diagnostics is None
                    else round((now - self._diagnostics_monotonic) * 1000.0, 1)
                ),
            }

    def update_diagnostics(self, diagnostics: dict[str, Any]) -> None:
        with self._lock:
            self._diagnostics = diagnostics
            self._diagnostics_monotonic = time.monotonic()


class SkeletonViewerServer:
    """Serve static viewer assets and the latest skeleton packet."""

    def __init__(
        self,
        host: str,
        port: int,
        source_to_robot_rotation=DEFAULT_SOURCE_TO_ROBOT_ROTATION,
        asset_root: str | Path | None = None,
        diagnostics_port: int = 15060,
    ):
        self.host = host
        self.port = int(port)
        self.state = SkeletonViewerState(source_to_robot_rotation)
        self.asset_root = (
            Path(asset_root)
            if asset_root is not None
            else Path(__file__).resolve().parents[3] / "web" / "pico_skeleton_viewer"
        )
        self._httpd: ThreadingHTTPServer | None = None
        self._thread: threading.Thread | None = None
        self._diagnostics_port = int(diagnostics_port)
        self._diagnostics_socket: socket.socket | None = None
        self._diagnostics_thread: threading.Thread | None = None
        self._diagnostics_stop = threading.Event()

    def start(self) -> None:
        asset_root = self.asset_root.resolve()
        state = self.state

        class Handler(BaseHTTPRequestHandler):
            def head_proxy(self, post=False):
                path = urlsplit(self.path).path
                allowed = ('state', 'color.jpg', 'depth.jpg') if not post else ('enable', 'move', 'lock', 'heartbeat')
                if path not in tuple('/api/head/' + name for name in allowed):
                    self.send_error(404)
                    return
                payload = None
                headers = {}
                if post:
                    origin = urlsplit(self.headers.get('Origin', ''))
                    if (origin.netloc != self.headers.get('Host') or origin.scheme != 'http'
                            or self.headers.get('X-Head-Control') != '1'
                            or self.headers.get('Content-Type') != 'application/json'):
                        self.send_error(403)
                        return
                    try:
                        size = int(self.headers.get('Content-Length', '0'))
                    except ValueError:
                        size = 0
                    if not 0 < size <= 1024:
                        self.send_error(400)
                        return
                    payload = self.rfile.read(size)
                    headers = {'Content-Type': 'application/json', 'X-Head-Control': '1',
                               'Origin': 'http://127.0.0.1:8766'}
                try:
                    request = Request('http://127.0.0.1:8766' + path, data=payload, headers=headers)
                    with urlopen(request, timeout=3) as response:
                        body = response.read(2 * 1024 * 1024)
                        code, mime = response.status, response.headers.get('Content-Type')
                except HTTPError as exc:
                    body, code, mime = exc.read(4096), exc.code, 'application/json'
                except (URLError, TimeoutError, OSError):
                    body, code, mime = b'{"error":"head camera bridge unavailable"}', 503, 'application/json'
                self.send_response(code)
                self.send_header('Content-Type', mime)
                self.send_header('Cache-Control', 'no-store')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                try:
                    self.wfile.write(body)
                except (BrokenPipeError, ConnectionResetError):
                    pass

            def do_POST(self):
                self.head_proxy(post=True)

            def do_GET(self) -> None:  # noqa: N802 - HTTP handler API
                path = urlsplit(self.path).path
                if path.startswith('/api/head/'):
                    self.head_proxy()
                    return
                if path == "/api/state":
                    payload = json.dumps(state.snapshot(), separators=(",", ":")).encode("utf-8")
                    self.send_response(HTTPStatus.OK)
                    self.send_header("Content-Type", "application/json; charset=utf-8")
                    self.send_header("Cache-Control", "no-store")
                    self.send_header("Content-Length", str(len(payload)))
                    self.end_headers()
                    self.wfile.write(payload)
                    return

                relative = "index.html" if path == "/" else path.lstrip("/")
                candidate = (asset_root / relative).resolve()
                if asset_root not in candidate.parents and candidate != asset_root:
                    self.send_error(HTTPStatus.NOT_FOUND)
                    return
                try:
                    payload = candidate.read_bytes()
                except (FileNotFoundError, IsADirectoryError):
                    self.send_error(HTTPStatus.NOT_FOUND)
                    return
                content_types = {
                    ".html": "text/html; charset=utf-8",
                    ".css": "text/css; charset=utf-8",
                    ".js": "text/javascript; charset=utf-8",
                    ".map": "application/json",
                }
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", content_types.get(candidate.suffix, "application/octet-stream"))
                self.send_header("Cache-Control", "no-cache")
                self.send_header("Content-Length", str(len(payload)))
                self.end_headers()
                self.wfile.write(payload)

            def log_message(self, _format: str, *_args: Any) -> None:
                return

        self._httpd = ReusableThreadingHTTPServer((self.host, self.port), Handler)
        self._thread = threading.Thread(
            target=self._httpd.serve_forever,
            name="pico-skeleton-viewer",
            daemon=True,
        )
        self._thread.start()
        self._diagnostics_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self._diagnostics_socket.bind(("127.0.0.1", self._diagnostics_port))
        except OSError:
            self._diagnostics_socket.close()
            self._diagnostics_socket = None
            self._httpd.shutdown()
            self._httpd.server_close()
            self._thread.join(timeout=2.0)
            self._httpd = None
            self._thread = None
            raise
        self._diagnostics_socket.settimeout(0.2)
        self._diagnostics_thread = threading.Thread(
            target=self._receive_diagnostics,
            name="pico-arm-diagnostics",
            daemon=True,
        )
        self._diagnostics_thread.start()

    def _receive_diagnostics(self) -> None:
        while not self._diagnostics_stop.is_set():
            try:
                payload, _ = self._diagnostics_socket.recvfrom(65535)
                message = json.loads(payload.decode("utf-8"))
                if isinstance(message, dict):
                    self.state.update_diagnostics(message)
            except socket.timeout:
                continue
            except (OSError, UnicodeDecodeError, json.JSONDecodeError):
                if not self._diagnostics_stop.is_set():
                    continue

    def update(self, packet: dict[str, Any]) -> None:
        self.state.update(packet)

    @property
    def bound_port(self) -> int:
        return self._httpd.server_port if self._httpd is not None else self.port

    def stop(self) -> None:
        self._diagnostics_stop.set()
        if self._diagnostics_socket is not None:
            self._diagnostics_socket.close()
        if self._httpd is not None:
            self._httpd.shutdown()
            self._httpd.server_close()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        if self._diagnostics_thread is not None:
            self._diagnostics_thread.join(timeout=1.0)
        self._httpd = None
        self._thread = None
        self._diagnostics_thread = None
        self._diagnostics_socket = None
