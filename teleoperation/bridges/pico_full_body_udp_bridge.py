#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Stream PICO full-body tracking from IsaacTeleop to IsaacLab over UDP."""

from __future__ import annotations

import argparse
import json
import os
import socket
import sys
import time
from pathlib import Path

import numpy as np

from isaacteleop.retargeting_engine.deviceio_source_nodes import FullBodySource
from isaacteleop.retargeting_engine.interface import OutputCombiner
from isaacteleop.retargeting_engine.tensor_types.indices import BodyJointPicoIndex, FullBodyInputIndex
from isaacteleop.teleop_session_manager import TeleopSession, TeleopSessionConfig


JOINT_TO_FRAME = {
    BodyJointPicoIndex.PELVIS: "waist",
    BodyJointPicoIndex.NECK: "neck",
    BodyJointPicoIndex.HEAD: "head",
    BodyJointPicoIndex.LEFT_SHOULDER: "left_shoulder",
    BodyJointPicoIndex.RIGHT_SHOULDER: "right_shoulder",
    BodyJointPicoIndex.LEFT_ELBOW: "left_elbow",
    BodyJointPicoIndex.RIGHT_ELBOW: "right_elbow",
    BodyJointPicoIndex.LEFT_WRIST: "left_wrist",
    BodyJointPicoIndex.RIGHT_WRIST: "right_wrist",
    BodyJointPicoIndex.LEFT_HAND: "left_hand",
    BodyJointPicoIndex.RIGHT_HAND: "right_hand",
    BodyJointPicoIndex.LEFT_ANKLE: "left_ankle",
    BodyJointPicoIndex.RIGHT_ANKLE: "right_ankle",
}

BODY_JOINT_NAMES = [joint.name for joint in BodyJointPicoIndex]


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1", help="IsaacLab UDP host")
    parser.add_argument("--port", type=int, default=15050, help="IsaacLab UDP port")
    parser.add_argument("--rate-hz", type=float, default=60.0, help="Send rate limit")
    parser.add_argument("--print-interval", type=float, default=1.0, help="Status print interval in seconds")
    parser.add_argument(
        "--cloudxr-env",
        default=str(Path.home() / ".cloudxr" / "run" / "cloudxr.env"),
        help="CloudXR environment file to load before creating the OpenXR session",
    )
    parser.add_argument(
        "--include-full-body-arrays",
        action="store_true",
        help="Also include raw 24-joint arrays for debugging; frames are always included.",
    )
    return parser.parse_args(argv)


def load_env_file(path: str) -> bool:
    env_path = Path(path).expanduser()
    if not env_path.exists():
        return False

    for raw_line in env_path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        if line.startswith("export "):
            line = line[len("export ") :]
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        os.environ.setdefault(key.strip(), value.strip().strip("'\""))
    return True


def check_openxr_runtime_ready() -> bool:
    runtime_json = os.environ.get("XR_RUNTIME_JSON")
    runtime_dir = os.environ.get("NV_CXR_RUNTIME_DIR")

    if not runtime_json:
        print("[pico_bridge] ERROR: XR_RUNTIME_JSON is not set.")
        print("[pico_bridge] Start CloudXR runtime first, or pass --cloudxr-env to a valid cloudxr.env file.")
        return False

    runtime_json_path = Path(runtime_json).expanduser()
    if not runtime_json_path.exists():
        print(f"[pico_bridge] ERROR: XR_RUNTIME_JSON does not exist: {runtime_json_path}")
        return False

    if "cloudxr" in runtime_json_path.name.lower() and runtime_dir:
        ipc_path = Path(runtime_dir).expanduser() / "ipc_cloudxr"
        if not ipc_path.exists():
            print(f"[pico_bridge] ERROR: CloudXR OpenXR service socket is missing: {ipc_path}")
            print("[pico_bridge] Start the CloudXR runtime in another terminal and keep it running:")
            print("  cd ~/PhdResearch/DualArmTeleopration")
            print("  conda activate env_isaaclab")
            print("  python -m isaacteleop.cloudxr --accept-eula --host-client")
            print("[pico_bridge] IsaacLab will still run locally; this runtime is only needed to receive PICO XR data.")
            return False

    return True


def build_full_body_pipeline():
    full_body = FullBodySource(name="full_body")
    return OutputCombiner({"full_body": full_body.output(FullBodySource.FULL_BODY)})


def quat_xyzw_to_wxyz(quat_xyzw: np.ndarray) -> list[float]:
    return [
        float(quat_xyzw[3]),
        float(quat_xyzw[0]),
        float(quat_xyzw[1]),
        float(quat_xyzw[2]),
    ]


def build_packet(full_body, include_full_body_arrays: bool) -> tuple[dict, int]:
    positions = np.asarray(full_body[FullBodyInputIndex.JOINT_POSITIONS], dtype=np.float32)
    orientations = np.asarray(full_body[FullBodyInputIndex.JOINT_ORIENTATIONS], dtype=np.float32)
    valid = np.asarray(full_body[FullBodyInputIndex.JOINT_VALID], dtype=np.uint8).astype(bool)

    frames = {}
    for joint_index, frame_name in JOINT_TO_FRAME.items():
        index = int(joint_index)
        if index >= len(valid) or not valid[index]:
            continue
        frames[frame_name] = {
            "joint_name": joint_index.name,
            "pos": [float(v) for v in positions[index, :3]],
            "quat_wxyz": quat_xyzw_to_wxyz(orientations[index, :4]),
            "valid": True,
        }

    packet = {
        "timestamp": time.time(),
        "source": "pico_full_body",
        "source_frame": "openxr_local",
        "quat_order": "wxyz",
        "frames": frames,
    }
    if include_full_body_arrays:
        packet.update(
            {
                "joint_names": BODY_JOINT_NAMES,
                "joint_positions": positions.tolist(),
                "joint_orientations": orientations.tolist(),
                "joint_valid": valid.tolist(),
            }
        )
    return packet, int(np.count_nonzero(valid))


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    if load_env_file(args.cloudxr_env):
        print(f"[pico_bridge] Loaded CloudXR env from {args.cloudxr_env}.")
    if not check_openxr_runtime_ready():
        return 2

    rate_hz = max(args.rate_hz, 1.0)
    step_s = 1.0 / rate_hz
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    config = TeleopSessionConfig(app_name="PicoFullBodyUdpBridge", pipeline=build_full_body_pipeline())

    print(f"[pico_bridge] Sending body-tracking UDP to {args.host}:{args.port} at up to {rate_hz:.1f} Hz.")
    print("[pico_bridge] Wear PICO trackers, enable body tracking, then keep the zero pose for IsaacLab calibration.")

    last_print = 0.0
    inactive_print = 0.0
    sent_frames = 0
    try:
        with TeleopSession(config) as session:
            while True:
                loop_start = time.perf_counter()
                result = session.step()
                full_body = result["full_body"]
                now = time.time()

                if full_body.is_none:
                    if now - inactive_print >= args.print_interval:
                        print("[pico_bridge] full_body inactive; waiting for PICO XR_BD_body_tracking data.")
                        inactive_print = now
                else:
                    packet, valid_count = build_packet(full_body, args.include_full_body_arrays)
                    encoded = json.dumps(packet, separators=(",", ":")).encode("utf-8")
                    sock.sendto(encoded, (args.host, args.port))
                    sent_frames += 1

                    if now - last_print >= args.print_interval:
                        names = ",".join(sorted(packet["frames"].keys())) or "none"
                        print(
                            f"[pico_bridge] sent={sent_frames} valid={valid_count:02d}/{len(BODY_JOINT_NAMES)} "
                            f"frames=[{names}]"
                        )
                        last_print = now

                elapsed = time.perf_counter() - loop_start
                if elapsed < step_s:
                    time.sleep(step_s - elapsed)
    except RuntimeError as exc:
        print(f"[pico_bridge] ERROR: {exc}")
        print("[pico_bridge] Check that the CloudXR/OpenXR runtime is running and the PICO client is connected.")
        return 3


if __name__ == "__main__":
    try:
        sys.exit(main(sys.argv[1:]))
    except KeyboardInterrupt:
        print("\n[pico_bridge] stopped.")
        sys.exit(0)
