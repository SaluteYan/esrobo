"""Temporary passive arm diagnostics. Never calls SDK getters or CAN send."""

import math
import socket
import struct
import threading
import time

import numpy as np

from .async_json_log import AsyncJsonLog


def plain(value, depth=0):
    if depth > 6:
        return None
    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, (float, np.floating)):
        return float(value) if math.isfinite(value) else None
    if isinstance(value, np.ndarray):
        return plain(value.tolist(), depth+1)
    if isinstance(value, (list, tuple)):
        return [plain(v, depth+1) for v in value]
    if isinstance(value, dict):
        return {str(k): plain(v, depth+1) for k, v in value.items()}
    return None


def cached_message(parser, name, fields):
    message = getattr(parser, name, None)
    if message is None:
        return None
    before = getattr(message, "timestamp", None)
    payload = getattr(message, "msg", None)
    result = {}
    for field in fields:
        value = payload
        for part in field.split("."):
            value = getattr(value, part, None)
        result[field] = plain(value)
    return dict(stamp_s=plain(before), fields=result,
                stamp_changed_during_copy=before != getattr(message, "timestamp", None))


def controller_snapshot(driver):
    parser = getattr(driver._arm._robot, "_parser", None)
    groups = ["joint_12", "joint_34", "joint_56", "joint_7"]
    groups += [f"leader_joint_{i}" for i in range(1, 8)]
    return dict(
        sdk_auto_motion_mode_enabled=plain(getattr(driver._arm._robot, "_auto_set_motion_mode_enabled", None)),
        selected_motion_mode=plain(getattr(driver._arm, "_selected_motion_mode", None)),
        tracking_control_mode=plain(getattr(driver._arm, "_tracking_control_mode", None)),
        cpv_tracking_ready=plain(getattr(driver._arm, "_cpv_tracking_ready", None)),
        transport=driver._arm._send_monitor.snapshot()
        if getattr(driver._arm, "_send_monitor", None) is not None else None,
        controller=cached_message(parser, "arm_status", (
            "ctrl_mode", "arm_status", "mode_feedback", "teach_status",
            "motion_status", "trajectory_num", "err_code")),
        joint_groups={name: cached_message(parser, name, tuple(f"joint_{i}" for i in range(1, 8)))
                      for name in groups},
        drivers=[cached_message(parser, f"driver_state_{i}", (
            "foc_status.driver_enable_status", "foc_status.voltage_too_low",
            "foc_status.motor_overheating", "foc_status.driver_overcurrent",
            "foc_status.driver_overheating", "foc_status.collision_status",
            "foc_status.driver_error_status", "foc_status.stall_status",
            "vol", "bus_current",
            "driver_temp", "motor_temp")) for i in range(1, 8)],
        motors=[cached_message(parser, f"motor_state_{i}", (
            "position", "velocity", "current", "torque")) for i in range(1, 8)],
        sdk_cached_motor_warning="SDK getters may zero velocity or flip current; use raw CAN to verify",
        cached_command_mode=plain(vars(driver._arm._robot).get("_msg_mode").__dict__)
        if hasattr(vars(driver._arm._robot).get("_msg_mode"), "__dict__") else None,
        trajectory=plain(getattr(driver, "trajectory_diagnostics", None)),
        last_command_urdf_rad=plain(getattr(driver, "_last_commanded_full_urdf", None)),
        fault=plain(getattr(driver, "safety_fault_reason", None)),
    )


class ArmProbe:
    def __init__(self, path, driver, state):
        self.driver, self.state = driver, state
        self.log = AsyncJsonLog(path, capacity=2048)
        self.raw_path = path.with_name(path.name.replace("pico_arm_probe", "pico_can"))
        self.raw = AsyncJsonLog(self.raw_path, capacity=8192)
        self.send_monitor = getattr(driver._arm, "_send_monitor", None)
        if self.send_monitor is not None:
            self.send_monitor.sink = self.submit
        self.stop = threading.Event()
        self.threads = []
        self.channel = getattr(driver._cfg, f"{driver._side}_can_channel")
        self.submit(dict(event="metadata", channel=self.channel, side=driver._side,
                         configured_firmware=str(driver._cfg.firmware_enum_for(driver._side)),
                         note="20Hz non-atomic SDK cache snapshots + passive classic CAN + native send outcomes; probe sends no commands; socket return is not controller execution acknowledgement"))
        for target in (self._sample, self._receive):
            thread = threading.Thread(target=target, daemon=True, name="arm-passive-probe")
            self.threads.append(thread)
            thread.start()

    def submit(self, event):
        self.log.submit(dict(plain(event), timestamp=time.time(), monotonic_s=time.monotonic(),
                             log_dropped=self.log.dropped, raw_log_dropped=self.raw.dropped))

    def _sample(self):
        while not self.stop.is_set():
            started = time.monotonic()
            try:
                record = controller_snapshot(self.driver)
                record.update(event="state", teleop=plain(self.state()))
                record["snapshot_ms"] = (time.monotonic()-started)*1000
                self.submit(record)
            except Exception as exc:
                self.submit(dict(event="snapshot_error", error=str(exc)))
            self.stop.wait(max(.001, .05-(time.monotonic()-started)))

    def _receive(self):
        try:
            with socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as bus:
                bus.settimeout(.1)
                try:
                    bus.setsockopt(socket.SOL_SOCKET, 40, 1)  # Linux SO_RXQ_OVFL.
                except OSError:
                    self.submit(dict(event="kernel_drop_counter_unavailable"))
                bus.bind((self.channel,))
                self.submit(dict(event="passive_can_ready", channel=self.channel))
                while not self.stop.is_set():
                    try:
                        frame, ancillary, flags, _ = bus.recvmsg(16, 64)
                    except socket.timeout:
                        continue
                    if len(frame) != 16:
                        continue
                    can_id, dlc, payload = struct.unpack("=IB3x8s", frame)
                    kernel_drops = next((struct.unpack("=I", data[:4])[0]
                                         for level, kind, data in ancillary
                                         if level == socket.SOL_SOCKET and kind == 40 and len(data) >= 4), None)
                    self.raw.submit(dict(timestamp=time.time(), monotonic_s=time.monotonic(),
                                         can_id=can_id, dlc=dlc, data_hex=payload[:min(dlc, 8)].hex(),
                                         local_origin=bool(flags & socket.MSG_DONTROUTE),
                                         kernel_drop_count=kernel_drops,
                                         log_dropped=self.raw.dropped))
        except Exception as exc:
            self.submit(dict(event="passive_can_error", error=str(exc)))
            print(f"[teleop] passive CAN diagnostic unavailable: {exc}", flush=True)

    def close(self):
        if self.send_monitor is not None:
            self.send_monitor.sink = None
        self.stop.set()
        for thread in self.threads:
            thread.join(timeout=1.)
        self.submit(dict(event="probe_closed"))
        self.log.close()
        self.raw.close()
