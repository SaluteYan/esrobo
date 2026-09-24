"""Serialized BodyDevice ingestion with independent, non-replayed source clocks.

Adapter clocks are monotonic clocks on THIS host; only loopback is accepted.
Robot clocks are never compared with these sensor clocks.
"""
import json
from collections import deque
import threading
import time

import numpy as np

from esrobo_teleop.device.body_device import BodyDevice


class InputUnavailable(RuntimeError):
    pass


class FreshBodyDevice(BodyDevice):
    def __init__(self, cfg, sides, with_hand, hand_only=False):
        self.input_lock = threading.RLock()
        self.reference_capture_enabled = False
        self.sides, self.with_hand, self.hand_only = tuple(sides), with_hand, hand_only
        self.stamps = {}
        self.source_updates = {}
        self.used = {}
        self.hand_sequences = {}
        self.pico_hand_targets = {}
        self.pico_hand_status = {}
        self.pico_hand_status_at = None
        self.hand_history = {side: deque(maxlen=120) for side in self.sides}
        self.wrist_rotations = {}
        self.last_rejection = "waiting for local adapters"
        cfg.use_hand_imu_orientation = False
        cfg.require_hand_imu_for_active_arm = False
        super().__init__(cfg, active_arm_side=sides[0] if len(sides) == 1 else None)

    def _update_auto_reference(self, now):
        if self.reference_capture_enabled:
            super()._update_auto_reference(now)

    @property
    def required(self):
        if self.hand_only:
            return tuple(f"{s}_hand" for s in self.sides)
        return tuple(f"{s}_{kind}" for s in self.sides
                     for kind in (("arm", "hand", "wrist") if self.with_hand else ("arm",)))

    def _compose_wrist_rotation(self, target_side, target_arm_points):
        initial = (self._initial_left_pose_matrix if target_side == "left"
                   else self._initial_right_pose_matrix)[:3, :3]
        if not self.with_hand:
            return initial.copy()
        wrist, _ = self._current_wrist_matrix(self._source_side_for_target(target_side))
        reference = self._reference_wrist_rotations.get(target_side)
        if wrist is None or reference is None:
            raise InputUnavailable(f"{target_side} PICO wrist reference missing")
        # Both matrices are already in the robot/waist frame. This applies to
        # both arms; glove anatomical-axis conversion must not be reused here.
        delta = wrist[:3, :3] @ reference.T
        from scipy.spatial.transform import Rotation
        rotvec = Rotation.from_matrix(delta).as_rotvec()
        maximum = np.deg2rad(self._cfg.hand_imu_max_angle_deg)
        length = np.linalg.norm(rotvec)
        if length > maximum:
            rotvec *= maximum / length
        target = Rotation.from_rotvec(rotvec).as_matrix() @ initial
        previous = self.wrist_rotations.get(target_side, initial)
        step = Rotation.from_matrix(target @ previous.T).as_rotvec()
        limit = np.deg2rad(self._cfg.hand_imu_max_step_deg)
        if np.linalg.norm(step) > limit:
            step *= limit / np.linalg.norm(step)
        result = Rotation.from_rotvec(step).as_matrix() @ previous
        self.wrist_rotations[target_side] = result
        return result.astype(np.float32)

    def _record_source_updates(self, updates):
        for key, stamp in updates.items():
            samples = self.source_updates.setdefault(key, deque(maxlen=200))
            samples.append(float(stamp))

    def source_rates(self, now=None):
        """Observed source update rate over one second, using source sample clocks."""
        now = time.monotonic() if now is None else now
        rates = {}
        for key, samples in self.source_updates.items():
            while samples and now-samples[0] > 1.0:
                samples.popleft()
            rates[key] = ((len(samples)-1)/(samples[-1]-samples[0])
                          if len(samples) > 1 and samples[-1] > samples[0] else 0.)
        return rates

    def _handle_packet(self, payload):
        try:
            message = json.loads(payload)
            meta = message["laptop_input"]
            if meta.get("v") != 1:
                raise ValueError("unsupported input version")
            now = time.monotonic()
            kind = meta["kind"]
            if kind not in ("pico", "pico_hand"):
                return  # Legacy SenseGlove publishers cannot overwrite PICO targets.
            physical = meta["physical_sides"]
            if kind == "pico_hand":
                self._handle_pico_hands(message, meta, now)
                return
            if not set(self.sides).issubset(physical):
                raise ValueError("PICO body missing selected sides")
            updates = {}
            if kind == "pico":
                # The supplied XRoboToolkit adapter emits frames only for advancing
                # body samples (its upstream bridge rejects frozen SDK caches).
                if self._cfg.auto_start_reference_require_waist and self._parse_pose(message["frames"]["waist"]) is None:
                    raise ValueError("waist reference missing")
                for side in self.sides:
                    for part in ("shoulder", "elbow", "wrist"):
                        pose = self._parse_pose(message["frames"][f"{side}_{part}"])
                        if pose is None or np.linalg.norm(pose[3:]) < .5:
                            raise ValueError("incomplete body frame")
                    updates[f"{side}_arm"] = meta["sample_monotonic"][side]
                    if self.with_hand:
                        updates[f"{side}_wrist"] = meta["sample_monotonic"][side]
            else:
                return
            if any(type(t) not in (int, float) or not np.isfinite(t) or not 0 <= now-t <= .1
                   for t in updates.values()):
                raise ValueError("expired/non-local sensor timestamps")
            with self.input_lock:
                hand_status = message.get("pico_hand_status")
                if isinstance(hand_status, dict):
                    self.pico_hand_status = {
                        side: value[:120] for side, value in hand_status.items()
                        if side in self.sides and isinstance(value, str)
                    }
                    self.pico_hand_status_at = now
                if any(t <= self.stamps.get(key, -1) for key, t in updates.items()):
                    # Repeated publishes of an old ROS sample cannot renew freshness.
                    return
                # Body traffic owns only body frames, never fingers/IMU fields.
                super()._handle_packet(json.dumps({"frames": message["frames"]}).encode())
                self.stamps.update(updates)
                self._record_source_updates(updates)
                self.last_rejection = ""
        except (KeyError, TypeError, ValueError, OverflowError) as exc:
            self.last_rejection = str(exc)

    def _handle_pico_hands(self, message, meta, now):
        if not self.with_hand:
            return
        validated = {}
        for side in self.sides:
            if side not in meta["physical_sides"]:
                continue
            sample = message["pico_hands"][side]
            angles = np.asarray(sample["joints"], dtype=float)
            stamp, seq = meta["sample_monotonic"][side], sample["sequence"]
            if angles.shape != (10,) or not np.all(np.isfinite(angles)):
                raise ValueError(f"{side} invalid PICO finger targets")
            from esrobo_teleop.robot.linker_hand_driver import ACTIVE_HAND_JOINTS, ACTIVE_JOINT_LIMITS_RAD
            upper = np.array([ACTIVE_JOINT_LIMITS_RAD[n][1] for n in ACTIVE_HAND_JOINTS])
            if np.any(angles < 0) or np.any(angles > upper + 1e-6):
                raise ValueError(f"{side} PICO finger targets out of range")
            if type(seq) is not int or seq <= 0 or type(stamp) not in (int, float) or not np.isfinite(stamp) or not 0 <= now-stamp <= .1:
                raise ValueError(f"{side} invalid PICO Hand sample time/sequence")
            validated[side] = (angles, stamp, seq)
        with self.input_lock:
            changed = False
            for side, (angles, stamp, seq) in validated.items():
                if seq <= self.hand_sequences.get(side, 0) or stamp <= self.stamps.get(f"{side}_hand", -1):
                    continue
                self.pico_hand_targets[side] = angles
                self.hand_history[side].append((float(stamp), angles.copy()))
                self.hand_sequences[side] = seq
                self.stamps[f"{side}_hand"] = stamp
                self._record_source_updates({f"{side}_hand": stamp})
                changed = True
            if changed:
                # An unselected side is never transmitted to the robot.
                self._hand_joint_targets = np.concatenate([self.pico_hand_targets.get(s, np.zeros(10))
                                                          for s in ("left", "right")])
                self._last_hand_joint_packet_time_monotonic = now
                self.last_rejection = ""

    def restart_preparation(self):
        """Discard pre-enable body/hand references and wait for a new visible pose."""
        with self.input_lock:
            self.reference_capture_enabled = True
            if not self.hand_only:
                self.restart_arm_reference()
            for history in self.hand_history.values():
                history.clear()
            self.last_rejection = "等待胸前抬臂与自然张手准备姿态"

    def natural_open_hand_reference(self, window_s=.6, min_samples=12):
        """Return stable PICO hand baselines after a comfortable open-hand pose."""
        if not self.with_hand:
            return {}, ""
        now = time.monotonic()
        references = {}
        # PICO active-joint order: thumb pitch plus the four finger flexions.
        flexion_indices = np.asarray([2, 4, 5, 7, 9], dtype=int)
        from esrobo_teleop.robot.linker_hand_driver import ACTIVE_HAND_JOINTS, ACTIVE_JOINT_LIMITS_RAD
        upper = np.asarray([ACTIVE_JOINT_LIMITS_RAD[name][1] for name in ACTIVE_HAND_JOINTS])
        limits = upper[flexion_indices] * np.asarray([.65, .28, .28, .28, .28])
        for side in self.sides:
            samples = [values for stamp, values in self.hand_history[side]
                       if 0 <= now-stamp <= window_s]
            if len(samples) < min_samples:
                return None, f"等待{side}手连续自然张开（{len(samples)}/{min_samples} 帧）"
            values = np.stack(samples)
            median = np.median(values, axis=0)
            if np.any(median[flexion_indices] > limits):
                return None, f"{side}手尚未自然张开，请伸展拇指和四指"
            maximum_std = float(np.max(np.std(values[:, flexion_indices], axis=0)))
            if maximum_std > .04:
                return None, f"{side}手仍在移动，请自然张开并保持稳定"
            references[side] = median
        return references, "PICO 已检测到自然张手"

    def ticket(self, max_age, *, require_reference=True):
        """Must be called under input_lock; no cached frame is a new sample."""
        now = time.monotonic()
        missing = [k for k in self.required if k not in self.stamps or not 0 <= now-self.stamps[k] <= max_age]
        if missing:
            raise InputUnavailable("PICO 输入缺失或过期: " + ", ".join(missing))
        if require_reference and not self.hand_only and not self.is_ready():
            raise InputUnavailable("PICO reference calibration not ready")
        if any(self.stamps[k] <= self.used.get(k, -1) for k in self.required):
            return None
        return {k: self.stamps[k] for k in self.required}

    def consume(self, ticket):
        with self.input_lock:
            self.used.update(ticket)

    def close(self):
        super().close()
        self._thread.join(timeout=.2)
