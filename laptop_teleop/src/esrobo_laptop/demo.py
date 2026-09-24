"""Self-contained loopback demo with synthetic sensors and a mock robot only."""
import json
from pathlib import Path
import socket
import tempfile
import threading
import time

import numpy as np

from esrobo_link.backends import MockDualBackend
from esrobo_link.client import RobotClient
from esrobo_link.gateway import Gateway

from .acquisition import annotate
from .app import Controller
from .config import ROOT, read_robot_config, read_settings
from .contract import verify_contract
from .inputs import InputUnavailable
from .pipeline import Pipeline


class SyntheticSensors:
    """Test source; never selectable by the real robot application."""
    def __init__(self, pipeline):
        self.pipeline = pipeline
        self.stop = threading.Event()
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.address = pipeline.body._sock.getsockname()
        solver = next(iter(pipeline.solvers.values()))
        poses = solver.current_task_frame_poses(np.zeros(14))
        rotation = np.asarray(pipeline.cfg.retarget.source_to_robot_rotation).reshape(3, 3)
        self.frames = {name: dict(pos=(rotation.T @ pose[:3]).tolist(), quat_wxyz=[1, 0, 0, 0])
                       for name, pose in poses.items()}
        self.frames["waist"] = dict(pos=[0, 0, 0], quat_wxyz=[1, 0, 0, 0])
        self.thread = threading.Thread(target=self.run, daemon=True)

    def run(self):
        while not self.stop.is_set():
            now = time.monotonic()
            self.socket.sendto(annotate(dict(frames=self.frames), "pico",
                                       dict(left=now, right=now), ("left", "right")), self.address)
            hand = dict(pico_hands={s: dict(joints=[0.]*10, sequence=time.monotonic_ns())
                                    for s in ("left", "right")})
            self.socket.sendto(annotate(hand, "pico_hand", dict(left=now, right=now),
                                       ("left", "right")), self.address)
            self.stop.wait(.012)

    def close(self):
        self.stop.set()
        self.thread.join(timeout=1)
        self.socket.close()


def run_demo():
    settings = read_settings(ROOT / "config/laptop.yaml")
    settings.robot_host, settings.input_port = "127.0.0.1", 0
    cfg = read_robot_config(settings)
    # Accelerate only synthetic reference collection, never a robot configuration.
    cfg.retarget.auto_start_reference_prepare_s = 0
    cfg.retarget.auto_start_reference_delay_s = .12
    cfg.retarget.auto_start_reference_sample_start_s = 0
    cfg.retarget.auto_start_reference_min_samples = 5
    cfg.retarget.reference_min_upper_raise_deg = 0
    cfg.retarget.reference_min_elbow_flexion_deg = 0
    cfg.retarget.reference_max_elbow_flexion_deg = 180
    backend = MockDualBackend()
    key = b"offline-demo-only-key-not-for-real-robots-123456"
    gateway = Gateway(backend, key, port=0)
    worker = threading.Thread(target=gateway.run, daemon=True)
    worker.start()
    pipeline = producer = client = None
    with tempfile.TemporaryDirectory() as temp:
        path = Path(temp) / "key"
        path.write_bytes(key)
        path.chmod(0o600)
        try:
            client = RobotClient("127.0.0.1", gateway.address[1], path)
            state = client.connect()
            members = verify_contract(state["contract"], cfg, settings, mock=True)
            pipeline = Pipeline(cfg, settings, members)
            producer = SyntheticSensors(pipeline)
            producer.thread.start()
            controller = Controller(client, pipeline, settings, state["contract"])
            active_frames = 0
            enabled = False
            deadline = time.monotonic()+8
            while time.monotonic() < deadline:
                state = client.receive(.1)
                try:
                    targets = controller.step(state)
                except InputUnavailable:
                    if state["mode"] not in ("IDLE", "CALIBRATING"):
                        raise
                    continue
                if targets is not None and controller.sent >= 3 and not enabled:
                    # Only this in-process mock gateway is ever enabled by the demo.
                    if state["accepted_seq"] is not None:
                        gateway.request("e")
                        enabled = True
                if state["mode"] == "ACTIVE" and targets is not None:
                    active_frames += 1
                    if active_frames >= 10:
                        break
            if active_frames < 10:
                raise RuntimeError(f"demo not ACTIVE: {pipeline.body.last_rejection}; stamps={pipeline.body.stamps}; mode={state['mode']}")
            producer.close()
            producer = None
            # No cached target replay: let the gateway watchdog see the lost source.
            time.sleep(.25)
            state = client.receive(.2)
            if state["mode"] != "FAULT":
                raise RuntimeError("mock gateway failed to fault after input stopped")
            result = dict(active_frames=active_frames, target_packets=controller.sent,
                          final_mode=state["mode"], reason=state["reason"])
            print(json.dumps(result, indent=2))
            return result
        finally:
            if producer:
                producer.close()
            if client:
                client.close()
            if pipeline:
                pipeline.close()
            gateway.request("q")
            worker.join(timeout=2)


if __name__ == "__main__":
    run_demo()
