import json
import socket
import time
from urllib.request import urlopen

from esrobo_teleop.debug.skeleton_viewer import SkeletonViewerServer, SkeletonViewerState


def _packet(sequence=4):
    return {
        "sequence": sequence,
        "xrt_body_timestamp_ns": 123456,
        "frames": {
            "waist": {"pos": [0.0, 1.0, 0.0]},
            "left_shoulder": {"pos": [-0.2, 1.5, 0.0]},
            "left_elbow": {"pos": [-0.3, 1.2, 0.1]},
            "left_wrist": {"pos": [-0.4, 0.9, 0.15]},
            "right_shoulder": {"pos": [0.2, 1.5, 0.0]},
            "right_elbow": {"pos": [0.3, 1.2, 0.1]},
            "right_wrist": {"pos": [0.4, 0.9, 0.15]},
        },
    }


def test_viewer_state_exposes_latest_packet_and_mapping():
    mapping = (0, 0, -1, -1, 0, 0, 0, 1, 0)
    state = SkeletonViewerState(mapping)
    packet = _packet()
    state.update(packet)

    snapshot = state.snapshot()

    assert snapshot["connected"] is True
    assert snapshot["age_ms"] < 100
    assert snapshot["source_to_robot_rotation"] == list(mapping)
    assert snapshot["packet"] is packet


def test_viewer_http_serves_page_and_read_only_state(tmp_path):
    (tmp_path / "index.html").write_text("viewer-ready", encoding="utf-8")
    server = SkeletonViewerServer("127.0.0.1", 0, asset_root=tmp_path, diagnostics_port=0)
    server.start()
    server.update(_packet(sequence=9))
    try:
        base = f"http://127.0.0.1:{server.bound_port}"
        with urlopen(f"{base}/", timeout=2) as response:
            assert response.read() == b"viewer-ready"
        with urlopen(f"{base}/api/state", timeout=2) as response:
            payload = json.load(response)
        assert payload["connected"] is True
        assert payload["packet"]["sequence"] == 9
    finally:
        server.stop()


def test_viewer_marks_old_data_stale():
    state = SkeletonViewerState()
    state.update(_packet())
    state._received_monotonic = time.monotonic() - 0.6

    assert state.snapshot()["connected"] is False


def test_viewer_receives_arm_diagnostics_udp(tmp_path):
    (tmp_path / "index.html").write_text("viewer-ready", encoding="utf-8")
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    probe.bind(("127.0.0.1", 0))
    diagnostics_port = probe.getsockname()[1]
    probe.close()
    server = SkeletonViewerServer(
        "127.0.0.1", 0, asset_root=tmp_path, diagnostics_port=diagnostics_port
    )
    server.start()
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sender.sendto(
            json.dumps({"side": "left", "joints_deg": {"ik": [1, 2, 3, 4, 5, 6, 7]}}).encode(),
            ("127.0.0.1", diagnostics_port),
        )
        deadline = time.monotonic() + 1.0
        while server.state.snapshot()["diagnostics"] is None and time.monotonic() < deadline:
            time.sleep(0.01)
        assert server.state.snapshot()["diagnostics"]["side"] == "left"
    finally:
        sender.close()
        server.stop()
