#!/usr/bin/env python3
"""Stream XRoboToolkit PICO full-body poses to IsaacLab over UDP."""

from __future__ import annotations

import argparse
import errno
import json
import math
import os
from pathlib import Path
import socket
import sys
import time
from collections import deque
from typing import Any


BODY_JOINT_NAMES = [
    "Pelvis",
    "Left_Hip",
    "Right_Hip",
    "Spine1",
    "Left_Knee",
    "Right_Knee",
    "Spine2",
    "Left_Ankle",
    "Right_Ankle",
    "Spine3",
    "Left_Foot",
    "Right_Foot",
    "Neck",
    "Left_Collar",
    "Right_Collar",
    "Head",
    "Left_Shoulder",
    "Right_Shoulder",
    "Left_Elbow",
    "Right_Elbow",
    "Left_Wrist",
    "Right_Wrist",
    "Left_Hand",
    "Right_Hand",
]

JOINT_TO_FRAME = {
    0: "waist",
    7: "left_ankle",
    8: "right_ankle",
    10: "left_foot",
    11: "right_foot",
    12: "neck",
    15: "head",
    16: "left_shoulder",
    17: "right_shoulder",
    18: "left_elbow",
    19: "right_elbow",
    20: "left_wrist",
    21: "right_wrist",
    22: "left_hand",
    23: "right_hand",
}


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1", help="IsaacLab UDP host.")
    parser.add_argument("--port", type=int, default=15050, help="IsaacLab UDP port.")
    parser.add_argument("--rate-hz", type=float, default=90.0, help="Polling/sending rate limit.")
    parser.add_argument(
        "--viewer-host",
        default="",
        help="Bind address for the read-only skeleton viewer. Empty disables it.",
    )
    parser.add_argument(
        "--viewer-port",
        type=int,
        default=8765,
        help="HTTP port for the read-only skeleton viewer.",
    )
    parser.add_argument("--print-interval", type=float, default=1.0, help="Status print interval in seconds.")
    parser.add_argument("--samples", type=int, default=0, help="Number of packets to send. 0 means forever.")
    parser.add_argument(
        "--include-full-body-arrays",
        action="store_true",
        help="Also include raw 24-joint arrays for debugging; frames are always included.",
    )
    parser.add_argument(
        "--print-poses",
        action="store_true",
        help="Include shoulder/elbow/wrist positions in periodic status output.",
    )
    parser.add_argument(
        "--record-path",
        default=os.environ.get("XROBOTOOLKIT_BODY_RECORD_PATH", ""),
        help="Optional JSONL path used to record every body-tracking UDP packet for later replay.",
    )
    parser.add_argument(
        "--record-only",
        action="store_true",
        help="Record packets without sending them to IsaacLab UDP.",
    )
    parser.add_argument(
        "--record-flush-interval",
        type=int,
        default=30,
        help="Flush the recording file after this many packets.",
    )
    parser.add_argument(
        "--record-neutral-start-s",
        type=float,
        default=float(os.environ.get("XROBOTOOLKIT_BODY_RECORD_NEUTRAL_START_S", "3.0")),
        help=(
            "When recording, save this many seconds of neutral/still body data at the beginning before prompting "
            "the operator to start motion. The neutral segment is kept in the file for replay startup reference."
        ),
    )
    parser.add_argument(
        "--record-warmup-discard-s",
        type=float,
        default=float(os.environ.get("XROBOTOOLKIT_BODY_RECORD_WARMUP_DISCARD_S", "0.5")),
        help=(
            "When recording, discard this many seconds after the first valid body packet before writing JSONL. "
            "This removes the small startup jitter that often appears in the first few frames."
        ),
    )
    return parser.parse_args(argv)


def _as_float_list(values: Any, length: int) -> list[float]:
    row = [float(value) for value in values or []]
    if len(row) < length:
        return []
    row = row[:length]
    if not all(math.isfinite(value) for value in row):
        return []
    return row


def _valid_pose(pose: list[float]) -> bool:
    if len(pose) < 7:
        return False
    quat_norm = math.sqrt(sum(value * value for value in pose[3:7]))
    pos_norm = math.sqrt(sum(value * value for value in pose[:3]))
    return quat_norm > 1.0e-6 or pos_norm > 1.0e-6


def _rate(values: deque[float], now: float, window_s: float = 1.0) -> float:
    while values and now - values[0] > window_s:
        values.popleft()
    if len(values) < 2:
        return 0.0
    return (len(values) - 1) / max(values[-1] - values[0], 1.0e-9)


def _safe_int(getter, default: int = 0) -> int:
    try:
        return int(getter())
    except Exception:
        return default


def _build_packet(xrt, sequence: int, include_full_body_arrays: bool) -> tuple[dict[str, Any] | None, int]:
    if not xrt.is_body_data_available():
        return None, 0

    raw_poses = xrt.get_body_joints_pose()
    poses = [_as_float_list(row, 7) for row in list(raw_poses or [])]
    if len(poses) < len(BODY_JOINT_NAMES):
        return None, 0

    body_timestamp_ns = _safe_int(xrt.get_body_timestamp_ns)
    xr_timestamp_ns = _safe_int(xrt.get_time_stamp_ns)
    if body_timestamp_ns <= 0:
        body_timestamp_ns = xr_timestamp_ns
    frames: dict[str, Any] = {}
    valid = []
    positions = []
    orientations = []
    for index, pose in enumerate(poses[: len(BODY_JOINT_NAMES)]):
        is_valid = _valid_pose(pose)
        valid.append(is_valid)
        positions.append(pose[:3] if pose else [0.0, 0.0, 0.0])
        orientations.append(pose[3:7] if pose else [0.0, 0.0, 0.0, 1.0])
        frame_name = JOINT_TO_FRAME.get(index)
        if frame_name is None or not is_valid:
            continue
        frames[frame_name] = {
            "joint_name": BODY_JOINT_NAMES[index],
            "pos": pose[:3],
            "quat_xyzw": pose[3:7],
            "valid": True,
        }

    packet: dict[str, Any] = {
        "timestamp": time.time(),
        "sender_monotonic_ns": time.monotonic_ns(),
        "source": "xrobotoolkit_body_tracking",
        "source_frame": "pico_tracking",
        "sequence": sequence,
        "xrt_body_timestamp_ns": body_timestamp_ns,
        "xrt_timestamp_ns": xr_timestamp_ns,
        "quat_order": "xyzw",
        "frames": frames,
    }
    if include_full_body_arrays:
        packet.update(
            {
                "joint_names": BODY_JOINT_NAMES,
                "joint_positions": positions,
                "joint_orientations": orientations,
                "joint_valid": valid,
            }
        )
    return packet, sum(1 for value in valid if value)


def _packet_signature(packet: dict[str, Any]) -> tuple:
    timestamp_ns = int(packet.get("xrt_body_timestamp_ns", 0))
    frames = packet.get("frames", {})
    frame_sig = tuple(
        (name, tuple(round(float(value), 5) for value in frame.get("pos", [])[:3]))
        for name, frame in sorted(frames.items())
        if isinstance(frame, dict)
    )
    return timestamp_ns, frame_sig


def _format_pose_summary(packet: dict[str, Any]) -> str:
    frames = packet.get("frames", {})
    if not isinstance(frames, dict):
        return "none"
    names = (
        "left_shoulder",
        "left_elbow",
        "left_wrist",
        "right_shoulder",
        "right_elbow",
        "right_wrist",
    )
    chunks = []
    for name in names:
        frame = frames.get(name)
        if not isinstance(frame, dict):
            continue
        pos = frame.get("pos", [])
        try:
            pos_text = ",".join(f"{float(value):+.3f}" for value in pos[:3])
        except (TypeError, ValueError):
            continue
        chunks.append(f"{name}=[{pos_text}]")
    return " | ".join(chunks) if chunks else "none"


class BodyRecordingWriter:
    """Write timestamped body-tracking packets as line-delimited JSON."""

    def __init__(self, path: str | Path, flush_interval: int, neutral_start_s: float, warmup_discard_s: float):
        self.path = Path(path).expanduser()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.flush_interval = max(int(flush_interval), 1)
        self.neutral_start_s = max(float(neutral_start_s), 0.0)
        self.warmup_discard_s = max(float(warmup_discard_s), 0.0)
        self.started_wall_time = time.time()
        self.first_packet_monotonic_ns: int | None = None
        self.packet_count = 0
        self._stream = self.path.open("w", encoding="utf-8")
        header = {
            "type": "esrobo_body_recording_header",
            "version": 1,
            "source": "xrobotoolkit_body_tracking",
            "created_time": self.started_wall_time,
            "created_time_iso": time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime(self.started_wall_time)),
            "body_joint_names": BODY_JOINT_NAMES,
            "record_format": "jsonl",
            "packet_field": "packet",
            "time_field": "t_rel_s",
            "t_rel_zero": "first_recorded_body_packet_after_warmup",
            "neutral_start_s": self.neutral_start_s,
            "warmup_discard_s": self.warmup_discard_s,
        }
        self._write_json_line(header)
        self._stream.flush()

    def write_packet(self, packet: dict[str, Any], encoded_size: int) -> tuple[float, str]:
        now_ns = time.monotonic_ns()
        if self.first_packet_monotonic_ns is None:
            self.first_packet_monotonic_ns = now_ns
        t_rel_s = (now_ns - self.first_packet_monotonic_ns) / 1.0e9
        phase = "neutral_start" if t_rel_s < self.neutral_start_s else "motion"
        item = {
            "type": "esrobo_body_packet",
            "record_sequence": self.packet_count,
            "t_rel_s": t_rel_s,
            "record_monotonic_ns": now_ns,
            "record_phase": phase,
            "encoded_size": encoded_size,
            "packet": packet,
        }
        self._write_json_line(item)
        self.packet_count += 1
        if self.packet_count % self.flush_interval == 0:
            self._stream.flush()
        return t_rel_s, phase

    def write_event(self, event: str, **fields: Any) -> None:
        now_ns = time.monotonic_ns()
        if self.first_packet_monotonic_ns is None:
            t_rel_s = 0.0
        else:
            t_rel_s = (now_ns - self.first_packet_monotonic_ns) / 1.0e9
        item = {
            "type": "esrobo_body_recording_event",
            "event": event,
            "t_rel_s": t_rel_s,
            "record_monotonic_ns": now_ns,
        }
        item.update(fields)
        self._write_json_line(item)
        self._stream.flush()

    def close(self) -> None:
        self._stream.flush()
        self._stream.close()

    def _write_json_line(self, item: dict[str, Any]) -> None:
        self._stream.write(json.dumps(item, separators=(",", ":")) + "\n")


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    if args.record_only and not args.record_path:
        print("[xrobotoolkit_body_bridge] ERROR: --record-only requires --record-path.", file=sys.stderr)
        return 2
    rate_hz = max(args.rate_hz, 1.0)
    step_s = 1.0 / rate_hz

    try:
        import xrobotoolkit_sdk as xrt
    except ModuleNotFoundError as exc:
        missing = exc.name or "unknown"
        print(f"[xrobotoolkit_body_bridge] Missing Python module: {missing}", file=sys.stderr)
        print("[xrobotoolkit_body_bridge] Activate env_xrobotoolkit and install xrobotoolkit_sdk first.", file=sys.stderr)
        return 2

    viewer = None
    if args.viewer_host:
        from esrobo_teleop.config import load_config
        from esrobo_teleop.debug.skeleton_viewer import SkeletonViewerServer

        config_path = Path(__file__).resolve().parents[1] / "config" / "teleop_config.yaml"
        source_to_robot_rotation = load_config(str(config_path)).retarget.source_to_robot_rotation
        viewer = SkeletonViewerServer(
            args.viewer_host,
            args.viewer_port,
            source_to_robot_rotation=source_to_robot_rotation,
        )
        try:
            viewer.start()
        except OSError as exc:
            print(
                f"[xrobotoolkit_body_bridge] ERROR: skeleton viewer could not bind "
                f"{args.viewer_host}:{args.viewer_port}: {exc}",
                file=sys.stderr,
            )
            if exc.errno == errno.EADDRINUSE:
                print(
                    "[xrobotoolkit_body_bridge] The viewer port is already occupied on the robot host. "
                    "Do not run 'ssh -L' inside a robot/VS Code Remote SSH terminal; "
                    "run the forwarding command on the operator PC instead.",
                    file=sys.stderr,
                )
            return 2
        print(
            f"[xrobotoolkit_body_bridge] Read-only skeleton viewer: "
            f"http://{args.viewer_host}:{args.viewer_port}",
            flush=True,
        )

    record_writer = (
        BodyRecordingWriter(
            args.record_path,
            args.record_flush_interval,
            args.record_neutral_start_s,
            args.record_warmup_discard_s,
        )
        if args.record_path
        else None
    )
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    if args.record_only:
        print("[xrobotoolkit_body_bridge] Record-only mode enabled; UDP packets will not be sent.", flush=True)
    else:
        print(f"[xrobotoolkit_body_bridge] Sending PICO full-body UDP to {args.host}:{args.port}.", flush=True)
    if record_writer is not None:
        print(f"[xrobotoolkit_body_bridge] Recording body-tracking packets to {record_writer.path}.", flush=True)
        print(
            "[xrobotoolkit_body_bridge] Recording starts at the first valid body packet. "
            f"The first {record_writer.warmup_discard_s:.1f}s are discarded to avoid startup jitter. "
            f"Keep the neutral start pose still for {record_writer.neutral_start_s:.1f}s; "
            "that still segment is saved for deterministic replay startup reference.",
            flush=True,
        )
    print("[xrobotoolkit_body_bridge] Keep XRoboToolkit-PC-Service and the PICO app connected.", flush=True)
    print("[xrobotoolkit_body_bridge] Enable full-body/body tracking in the PICO XRoboToolkit app.", flush=True)

    xrt.init()
    sent_times: deque[float] = deque()
    source_change_times: deque[float] = deque()
    last_signature: tuple | None = None
    last_print = time.monotonic()
    last_wait_print = 0.0
    sent_total = 0
    latest_packet_size = 0
    latest_valid_count = 0
    latest_frame_names: list[str] = []
    latest_pose_summary = "none"
    record_started_printed = False
    record_motion_started_printed = False
    record_warmup_started_printed = False
    record_first_valid_monotonic: float | None = None
    latest_record_t_rel_s = 0.0
    latest_record_phase = "none"

    try:
        while args.samples <= 0 or sent_total < args.samples:
            loop_start = time.perf_counter()
            now = time.monotonic()
            packet, valid_count = _build_packet(xrt, sent_total, args.include_full_body_arrays)

            if packet is None:
                if now - last_wait_print >= args.print_interval:
                    print("[xrobotoolkit_body_bridge] body tracking unavailable; waiting for PICO body data.", flush=True)
                    last_wait_print = now
            else:
                if viewer is not None:
                    viewer.update(packet)
                signature = _packet_signature(packet)
                if signature != last_signature:
                    source_change_times.append(now)
                    last_signature = signature

                encoded = json.dumps(packet, separators=(",", ":")).encode("utf-8")
                if record_writer is not None:
                    if record_first_valid_monotonic is None:
                        record_first_valid_monotonic = now
                    warmup_elapsed_s = now - record_first_valid_monotonic
                    if warmup_elapsed_s < record_writer.warmup_discard_s:
                        latest_record_t_rel_s = warmup_elapsed_s
                        latest_record_phase = "warmup_discard"
                        if not record_warmup_started_printed:
                            print(
                                "[xrobotoolkit_body_bridge] RECORDING WARMUP: "
                                f"discarding the first {record_writer.warmup_discard_s:.1f}s of valid packets "
                                "to avoid startup jitter.",
                                flush=True,
                            )
                            record_warmup_started_printed = True
                    else:
                        latest_record_t_rel_s, latest_record_phase = record_writer.write_packet(packet, len(encoded))
                        if not record_started_printed:
                            record_writer.write_event(
                                "recording_started",
                                message="first post-warmup body packet recorded; hold neutral start pose",
                            )
                            print(
                                "[xrobotoolkit_body_bridge] RECORDING STARTED: first post-warmup body packet saved. "
                                f"Hold neutral/still for {record_writer.neutral_start_s:.1f}s.",
                                flush=True,
                            )
                            record_started_printed = True
                        if latest_record_phase == "motion" and not record_motion_started_printed:
                            record_writer.write_event(
                                "motion_started",
                                message="neutral start segment complete; operator may start teleoperation motion",
                            )
                            print(
                                "[xrobotoolkit_body_bridge] MOTION RECORDING ACTIVE: neutral start segment complete. "
                                "Start moving now.",
                                flush=True,
                            )
                            record_motion_started_printed = True
                if not args.record_only:
                    sock.sendto(encoded, (args.host, args.port))
                sent_total += 1
                sent_times.append(now)
                latest_packet_size = len(encoded)
                latest_valid_count = valid_count
                frames = packet.get("frames", {})
                latest_frame_names = sorted(frames.keys()) if isinstance(frames, dict) else []
                latest_pose_summary = _format_pose_summary(packet)

                if now - last_print >= args.print_interval:
                    names = ",".join(latest_frame_names) if latest_frame_names else "none"
                    print(
                        "[xrobotoolkit_body_bridge] "
                        f"send_hz={_rate(sent_times, now):5.1f} "
                        f"xrt_update_hz={_rate(source_change_times, now):5.1f} "
                        f"valid={latest_valid_count:02d}/{len(BODY_JOINT_NAMES)} "
                        f"frames=[{names}] "
                        f"body_timestamp_ns={packet.get('xrt_body_timestamp_ns', 0)} "
                        f"bytes={latest_packet_size} "
                        f"recorded={record_writer.packet_count if record_writer is not None else 0} "
                        f"record_phase={latest_record_phase} "
                        f"record_t={latest_record_t_rel_s:.2f}s",
                        flush=True,
                    )
                    if record_writer is not None and latest_record_phase == "warmup_discard":
                        warmup_remaining = max(record_writer.warmup_discard_s - latest_record_t_rel_s, 0.0)
                        print(
                            "[xrobotoolkit_body_bridge] recording prompt: "
                            f"discarding startup jitter for {warmup_remaining:.1f}s more; keep neutral/still.",
                            flush=True,
                        )
                    if record_writer is not None and latest_record_phase == "neutral_start":
                        neutral_remaining = max(record_writer.neutral_start_s - latest_record_t_rel_s, 0.0)
                        print(
                            "[xrobotoolkit_body_bridge] recording prompt: "
                            f"keep neutral/still for {neutral_remaining:.1f}s, then start motion after the prompt.",
                            flush=True,
                        )
                    if args.print_poses:
                        print(f"[xrobotoolkit_body_bridge] arm_poses {latest_pose_summary}", flush=True)
                    last_print = now

            elapsed = time.perf_counter() - loop_start
            if elapsed < step_s:
                time.sleep(step_s - elapsed)
    except KeyboardInterrupt:
        print("\n[xrobotoolkit_body_bridge] stopped.", flush=True)
    finally:
        xrt.close()
        sock.close()
        if viewer is not None:
            viewer.stop()
        if record_writer is not None:
            record_writer.close()
            print(
                f"[xrobotoolkit_body_bridge] recording saved: {record_writer.path} "
                f"packets={record_writer.packet_count}",
                flush=True,
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
