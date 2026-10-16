"""Serialized BodyDevice ingestion with independent, non-replayed source clocks.

Adapter clocks are monotonic clocks on THIS host; only loopback is accepted.
Robot clocks are never compared with these sensor clocks.
"""
import json
import copy
from collections import deque
import select
import threading
import time

import numpy as np

from esrobo_teleop.device.body_device import BodyDevice
from .orientation import AbsoluteHandOrientation, ForearmFrame, checked_rotation, palm_clock, forearm_twist_deg
from esrobo_teleop import math_utils as mu


class InputUnavailable(RuntimeError):
    pass


class SourceExpired(InputUnavailable):
    """Only the source timestamp gate may enter the bounded hold state."""


class SourceTicket(dict):
    """Timestamp mapping with frozen diagnostic identities outside the clock keys."""
    def __init__(self, stamps, hand_sequences):
        super().__init__(stamps)
        self.hand_sequences = dict(hand_sequences)


class FreshBodyDevice(BodyDevice):
    def __init__(self, cfg, sides, with_hand, hand_only=False):
        self.input_lock = threading.RLock()
        self.reference_capture_enabled = False
        self.sides, self.with_hand, self.hand_only = tuple(sides), with_hand, hand_only
        self.stamps = {}
        self.source_updates = {}
        self.used = {}
        self.hand_sequences = {}
        self.hand_mapping_fingerprint = None
        self.hand_mapping_upper = None
        self.hand_retarget_diagnostics = {}
        self.pico_hand_targets = {}
        self.pico_hand_status = {}
        self.pico_hand_status_at = None
        self.hand_history = {side: deque(maxlen=120) for side in self.sides}
        self.palm_history = {side: deque(maxlen=120) for side in self.sides}
        self.incoming_body_stamps = {}
        self.wrist_rotations = {}
        self.wrist_references = {}
        self.wrist_input_diagnostics = {}
        self.wrist_observations = {}
        self.last_rejection = "waiting for local adapters"
        self.receive_counts = dict(packets=0, body_coalesced=0, hand_coalesced=0, rejected=0)
        cfg.use_hand_imu_orientation = False
        cfg.require_hand_imu_for_active_arm = False
        super().__init__(cfg, active_arm_side=sides[0] if len(sides) == 1 else None)

    def _update_auto_reference(self, now):
        """Atomically lock one valid initial pose; no stability/countdown gate."""
        if not self.reference_capture_enabled or self._reference_locked or self.hand_only:
            return
        points, wrists, palms, references = {}, {}, {}, {}
        try:
            for side in self.sides:
                stamp = self.incoming_body_stamps.get(f"{side}_arm")
                if stamp is None or not 0 <= now-stamp <= .1:
                    raise ValueError(f"{side} 缺少新鲜 Body 数据")
                source = self._source_side_for_target(side)
                current = self._current_arm_points(source)
                if current is None:
                    raise ValueError(f"{side} 缺少完整肩肘腕或腰部数据")
                points[side] = {key: value.copy() for key, value in current.items()}
                wrist, _ = self._current_wrist_matrix(source)
                if wrist is not None:
                    wrists[side] = checked_rotation(wrist[:3, :3]).copy()
                if self.with_hand:
                    if wrist is None:
                        raise ValueError(f"{side} 缺少有效手腕姿态")
                    palms[side] = self._calibration_palm(side, now)
                    references[side] = AbsoluteHandOrientation(
                        points[side], wrists[side], palm_rotation=palms[side][0])
        except ValueError as exc:
            self.last_rejection = str(exc)
            self._print_reference_prompt("data", 0, f"初始位姿采集等待：{exc}")
            return
        # Validate every selected side before publishing any part of the reference.
        for side in self.sides:
            setattr(self, f"_reference_{side}_points", points[side])
        self._reference_wrist_rotations.update(wrists)
        self.wrist_references.update(references)
        self._reference_locked = True
        for side in self.sides:
            body_stamp = self.incoming_body_stamps[f"{side}_arm"]
            self._record_reference_diagnostic(
                "calibrated", side, points[side], points[side], 1, 0., body_stamp, now)
            if side in references:
                palm, stamp, sequence = palms[side]
                reference = references[side]
                record = dict(event="palm_calibrated", side=side,
                              frame="waist_local" if self._cfg.auto_start_reference_require_waist else "robot_world",
                              body_sample_monotonic=body_stamp,
                              hand_sample_monotonic=stamp, hand_sequence=sequence,
                              pair_gap_ms=1000*abs(stamp-body_stamp), palm_basis=palm.tolist(),
                              palm_normal_forearm=(reference.forearm.rotation.T @ palm[:, 0]).tolist(),
                              palm_reference_clock_deg=(None if reference.palm_reference_clock is None else
                                                        float(np.degrees(reference.palm_reference_clock))))
                path = getattr(self, "_reference_diagnostic_path", None)
                if path is not None:
                    self._write_reference_record(path, record)
                self._diagnostic_print(f"[PICO 首帧姿态] {side}: 绝对掌面目标已采集，Body/Hand 间隔 {record['pair_gap_ms']:.1f} ms", flush=True)
        self._diagnostic_print(
            "[PICO 标定] 完成：有效初始位姿已锁定；系统将自动检查目标与接管路径。", flush=True)

    def _calibration_palm(self, side, now):
        """Use landmarks from the nearest fresh Hand frame, in Body's frame."""
        history = self.palm_history[side]
        if not history:
            raise ValueError(f"{side} 等待有效掌面数据")
        body_stamp = self.incoming_body_stamps[f"{side}_arm"]
        stamp, palm, sequence = min(history, key=lambda s: abs(s[0]-body_stamp))
        if not 0 <= now-stamp <= .1 or abs(stamp-body_stamp) > .04:
            raise ValueError(f"{side} Body/Hand 掌面采样不同步（要求 ≤40 ms）")
        # Landmark axes are world vectors: left-multiply the source basis
        # change, without quaternion's right-side conjugation.
        transformed = self._rotation @ palm
        if self._cfg.auto_start_reference_require_waist:
            waist = mu.pose_array_to_matrix(self._frames["waist"])
            transformed = waist[:3, :3].T @ transformed
        points = self._current_arm_points(self._source_side_for_target(side))
        if points is None:
            raise ValueError(f"{side} 缺少完整前臂数据")
        checked_rotation(transformed)
        return transformed, stamp, sequence

    @property
    def required(self):
        if self.hand_only:
            return tuple(f"{s}_hand" for s in self.sides)
        return tuple(f"{s}_{kind}" for s in self.sides
                     for kind in (("arm", "hand", "wrist") if self.with_hand else ("arm",)))

    def _compose_wrist_rotation(self, target_side, target_arm_points, *, full_target=False):
        initial = (self._initial_left_pose_matrix if target_side == "left"
                   else self._initial_right_pose_matrix)[:3, :3]
        if not self.with_hand:
            return initial.copy()
        wrist, _ = self._current_wrist_matrix(self._source_side_for_target(target_side))
        points = self._current_arm_points(self._source_side_for_target(target_side))
        reference = self.wrist_references.get(target_side)
        if wrist is None or points is None or reference is None:
            raise InputUnavailable(f"{target_side} PICO forearm/hand reference missing")
        now = time.monotonic()
        body_stamp = self.stamps.get(f"{target_side}_arm")
        samples = self.palm_history[target_side]
        if body_stamp is None or not samples:
            raise SourceExpired(f"{target_side} 缺少新鲜的绝对掌面姿态")
        stamp, palm, sequence = min(samples, key=lambda sample: abs(sample[0]-body_stamp))
        if not 0 <= now-stamp <= self._cfg.max_stale_time_s or abs(stamp-body_stamp) > .04:
            raise SourceExpired(f"{target_side} Body/Hand 绝对姿态采样不同步（要求 ≤40 ms）")
        source_to_local = self._rotation
        if self._cfg.auto_start_reference_require_waist:
            source_to_local = mu.pose_array_to_matrix(self._frames["waist"])[:3, :3].T @ source_to_local
        local = reference.update(
            points, wrist[:3, :3], max_step_deg=self._cfg.hand_imu_max_step_deg,
            palm_rotation=source_to_local @ palm, full_target=full_target)
        self.wrist_rotations[target_side] = local
        self.wrist_input_diagnostics[target_side] = dict(reference.diagnostics)
        self.wrist_input_diagnostics[target_side].update(
            palm_comparison_status="paired", palm_pair_gap_ms=1000*abs(stamp-body_stamp),
            palm_hand_sequence=sequence, comparison_body_sample_monotonic=body_stamp,
            comparison_hand_sample_monotonic=stamp)
        # BodyDevice owns the position carrier. Pipeline attaches the local
        # rotation to the same input ticket and composes the WORLD hand pose
        # only after this cycle's position IK, using the robot's actual axes.
        return initial.copy()

    def wrist_diagnostics(self, now=None):
        """Passive Body/Hand comparison, called only by the low-rate logger.

        Absolute-mode diagnostics are frozen with the control input frame.
        Caller holds input_lock, just as for the other diagnostic snapshots.
        """
        from scipy.spatial.transform import Rotation
        now = time.monotonic() if now is None else now
        result = {side: dict(values) for side, values in self.wrist_input_diagnostics.items()}
        for side, values in result.items():
            if values.get("orientation_mode") == "forearm_absolute":
                if (not 0 <= now-values.get("comparison_body_sample_monotonic", -1) <= .1
                        or not 0 <= now-values.get("comparison_hand_sample_monotonic", -1) <= .1):
                    values["palm_comparison_status"] = "unavailable"
                continue
            values["palm_comparison_status"] = "unavailable"
            observation = self.wrist_observations.get(side)
            if not observation or observation["body_stamp"] is None:
                continue
            body_stamp = observation["body_stamp"]
            samples = self.palm_history[side]
            if not samples or not 0 <= now-body_stamp <= .1:
                continue
            stamp, palm, sequence = min(samples, key=lambda s: abs(s[0]-body_stamp))
            gap = abs(stamp-body_stamp)
            if not 0 <= now-stamp <= .1 or gap > .04:
                continue
            palm_delta = (observation["source_to_forearm"] @ palm
                          @ observation["palm_reference_local"].T)
            values.update(
                palm_comparison_status="paired",
                hand_palm_roll_deg=forearm_twist_deg(palm_delta),
                body_palm_disagreement_deg=float(np.degrees(Rotation.from_matrix(
                    palm_delta @ observation["body_delta"].T).magnitude())),
                palm_pair_gap_ms=1000*gap, palm_hand_sequence=sequence,
                comparison_body_sample_monotonic=body_stamp,
                comparison_hand_sample_monotonic=stamp)
        return result

    def reset_command_reference(self, poses):
        super().reset_command_reference(poses)
        for side, reference in self.wrist_references.items():
            reference.reset_filter()
            self.wrist_rotations[side] = reference.filtered.copy()

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

    def _receive_loop(self):
        """Drain a bounded UDP batch, keeping the newest valid components.

        This never manufactures a sampling time. Validation is outside the
        input lock; the selected snapshots are age-checked again at commit.
        """
        while not self._stop.is_set():
            try:
                first, _ = self._sock.recvfrom(self._cfg.max_packet_bytes)
                payloads = [first]
                for _ in range(63):
                    if not select.select([self._sock], [], [], 0)[0]:
                        break
                    payloads.append(self._sock.recvfrom(self._cfg.max_packet_bytes)[0])
                self._commit_batch(payloads)
            except TimeoutError:
                continue
            except (OSError, ValueError):
                break
            except Exception as exc:
                # A malformed packet must not terminate the receiver, nor
                # trigger synchronous terminal output from this thread.
                self.receive_counts['rejected'] += 1
                self.last_rejection = f'PICO receive error: {exc}'[:200]

    def _commit_batch(self, payloads):
        body, hands = None, {}
        for payload in payloads:
            # Validate each component before selection, but transform only the
            # surviving Body. Bursty delivery must not spend control CPU on
            # coordinate conversions for snapshots that will be coalesced.
            prepared = self._decode_input(payload, transform=False)
            if prepared is None:
                continue
            kind, data = prepared
            if kind == 'pico':
                if body is not None:
                    self.receive_counts['body_coalesced'] += 1
                    if any(t <= body[2][k] for k,t in data[2].items()):
                        continue
                body = data
            else:
                for side, sample in data.items():
                    previous = hands.get(side)
                    if previous is not None:
                        self.receive_counts['hand_coalesced'] += 1
                        if sample[1] <= previous[1] or sample[2] <= previous[2]:
                            continue
                    hands[side] = sample
        # Commit paired hands before Body reference admission. Both retain
        # their independent source times, and the existing 40 ms pairing gate.
        if hands:
            self._commit_input(('pico_hand', hands))
        if body is not None:
            message, _, stamps, status = body
            body = (message, self._decode_frames(message), stamps, status)
            self._commit_input(('pico', body))

    def _handle_packet(self, payload):
        prepared = self._decode_input(payload)
        if prepared is not None:
            self._commit_input(prepared)

    def _decode_input(self, payload, *, transform=True):
        self.receive_counts['packets'] += 1
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
                return kind, self._prepare_pico_hands(message, meta, now)
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
            # Decode/transform once outside the input lock. Atomic commit and
            # reference/filter state remain under the same lock as before.
            # The raw skeleton viewer receives the full packet independently.
            # Control needs only waist + the selected arm chains. Converting
            # head/feet/the inactive arm at sensor rate needlessly competes
            # with IK on this process's Python interpreter.
            source_sides = set(self.sides) | {self._source_side_for_target(s) for s in self.sides}
            needed = {'waist'} | {f'{s}_{part}' for s in source_sides
                                  for part in ('shoulder', 'elbow', 'wrist')}
            body_message = {"frames": {k:v for k,v in message["frames"].items() if k in needed}}
            parsed_frames = self._decode_frames(body_message) if transform else None
            return kind, (body_message, parsed_frames, updates, message.get('pico_hand_status'))
        except (KeyError, TypeError, ValueError, OverflowError) as exc:
            self.receive_counts['rejected'] += 1
            self.last_rejection = str(exc)
            return None

    def _commit_input(self, prepared):
        kind, data = prepared
        try:
            if kind == 'pico_hand':
                self._commit_pico_hands(data)
                return
            body_message, parsed_frames, updates, hand_status = data
            with self.input_lock:
                now = time.monotonic()
                if any(not 0 <= now-t <= .1 for t in updates.values()):
                    raise ValueError("expired sensor timestamps before commit")
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
                self.incoming_body_stamps = updates
                try:
                    super()._handle_message(body_message, parsed_frames=parsed_frames)
                finally:
                    self.incoming_body_stamps = {}
                self.stamps.update(updates)
                self._record_source_updates(updates)
                self.last_rejection = ""
        except (KeyError, TypeError, ValueError, OverflowError) as exc:
            self.receive_counts['rejected'] += 1
            self.last_rejection = str(exc)

    def _prepare_pico_hands(self, message, meta, now):
        if not self.with_hand:
            return {}
        validated = {}
        for side in self.sides:
            if side not in meta["physical_sides"]:
                continue
            sample = message["pico_hands"][side]
            if side == 'left' and self.hand_mapping_fingerprint is not None:
                from .hand_retarget import VERSION
                if (sample.get('mapping_version') != VERSION or
                        sample.get('mapping_fingerprint') != self.hand_mapping_fingerprint):
                    raise ValueError('left hand mapping version/model mismatch; restart PICO and laptop')
            angles = np.asarray(sample["joints"], dtype=float)
            stamp, seq = meta["sample_monotonic"][side], sample["sequence"]
            if angles.shape != (10,) or not np.all(np.isfinite(angles)):
                raise ValueError(f"{side} invalid PICO finger targets")
            from esrobo_teleop.robot.linker_hand_driver import ACTIVE_HAND_JOINTS, ACTIVE_JOINT_LIMITS_RAD
            upper = np.array([ACTIVE_JOINT_LIMITS_RAD[n][1] for n in ACTIVE_HAND_JOINTS])
            if side == 'left' and self.hand_mapping_upper is not None:
                upper = self.hand_mapping_upper
            if np.any(angles < 0) or np.any(angles > upper + 1e-6):
                raise ValueError(f"{side} PICO finger targets out of range")
            if type(seq) is not int or seq <= 0 or type(stamp) not in (int, float) or not np.isfinite(stamp) or not 0 <= now-stamp <= .1:
                raise ValueError(f"{side} invalid PICO Hand sample time/sequence")
            palm = (None if self.hand_only else checked_rotation(sample["palm_basis"]))
            diagnostics = {}
            if side == 'left' and self.hand_mapping_fingerprint is not None and sample.get('diagnostics'):
                values = sample['diagnostics']
                elapsed = float(values['solve_ms'])
                tips = np.asarray(values['tip_error_mm'], dtype=float)
                pinch = np.asarray(values['pinch_error_mm'], dtype=float)
                limited = values['limited_axes']
                if (not np.isfinite(elapsed) or not 0 <= elapsed <= 20 or tips.shape != (5,) or pinch.shape != (4,)
                        or not np.isfinite(tips).all() or not np.isfinite(pinch).all()
                        or np.any(tips < 0) or np.any(pinch < 0)
                        or not isinstance(limited, list) or len(limited) > 10 or any(n not in ACTIVE_HAND_JOINTS for n in limited)):
                    raise ValueError('invalid left hand fit diagnostic summary')
                diagnostics = dict(solve_ms=elapsed, tip_error_mm=tips.tolist(), pinch_error_mm=pinch.tolist(), limited_axes=list(limited))
            validated[side] = (angles, stamp, seq, palm, diagnostics)
        return validated

    def _commit_pico_hands(self, validated):
        with self.input_lock:
            now = time.monotonic()
            changed = False
            for side, (angles, stamp, seq, palm, diagnostics) in validated.items():
                if not 0 <= now-stamp <= .1:
                    self.receive_counts['rejected'] += 1
                    self.last_rejection = f'{side} expired PICO Hand before commit'
                    continue
                if seq <= self.hand_sequences.get(side, 0) or stamp <= self.stamps.get(f"{side}_hand", -1):
                    continue
                self.pico_hand_targets[side] = angles
                self.hand_history[side].append((float(stamp), angles.copy()))
                if palm is not None:
                    self.palm_history[side].append((float(stamp), palm, seq))
                self.hand_sequences[side] = seq
                self.hand_retarget_diagnostics[side] = dict(diagnostics, source_sequence=seq)
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
            self.wrist_references.clear()
            self.wrist_rotations.clear()
            self.wrist_input_diagnostics.clear()
            self.wrist_observations.clear()
            for history in self.hand_history.values():
                history.clear()
            for history in self.palm_history.values():
                history.clear()
            self.last_rejection = "等待有效 PICO 初始位姿"

    def initial_hand_reference(self):
        """Capture valid finger angles without requiring an open or steady hand."""
        if not self.with_hand:
            return {}, ""
        now = time.monotonic()
        references = {}
        with self.input_lock:
            for side in self.sides:
                history = self.hand_history[side]
                if not history or not 0 <= now-history[-1][0] <= .1:
                    return None, f"等待{side}手有效新鲜数据"
                values = np.asarray(history[-1][1], dtype=float)
                if values.shape != (10,) or not np.isfinite(values).all():
                    return None, f"{side}手关节数据无效"
                references[side] = values.copy()
        return references, "PICO 有效初始手部姿态已采集"

    def ticket(self, max_age, *, require_reference=True):
        """Must be called under input_lock; no cached frame is a new sample."""
        now = time.monotonic()
        missing = [k for k in self.required if k not in self.stamps or not 0 <= now-self.stamps[k] <= max_age]
        if missing:
            raise SourceExpired("PICO 输入缺失或过期: " + ", ".join(missing))
        if require_reference and not self.hand_only and not self.is_ready():
            raise InputUnavailable("PICO reference calibration not ready")
        if any(self.stamps[k] <= self.used.get(k, -1) for k in self.required):
            return None
        return SourceTicket({k: self.stamps[k] for k in self.required}, self.hand_sequences)

    def consume(self, ticket):
        with self.input_lock:
            self.used.update(ticket)

    def capture_targets(self, max_age, *, full_target=False):
        """Freeze receiver-owned inputs; retarget on the control thread.

        A shallow calculation view does not initialize a device or subscription.
        Wrist filters are candidates until the controller accepts this frame.
        Rejected recovery frames must not move the committed wrist filter.
        Preparation/reset also runs on the same control thread.
        """
        with self.input_lock:
            ticket = self.ticket(max_age)
            if ticket is None:
                return None
            if self.hand_only:
                hand = self.hand_joints()
                if hand is None:
                    raise InputUnavailable('PICO finger tracking missing')
                return ticket, None, hand.copy()
            view = copy.copy(self)
            view._frames = {k:v.copy() for k,v in self._frames.items()}
            view.stamps = dict(self.stamps)
            # Pair against this frozen Body timestamp with the exact same
            # nearest-sample rule used by _compose_wrist_rotation. Only that
            # sample is consumed by retargeting; cloning up to 120 matrices
            # per side here delays receiver commits for no additional input.
            view.palm_history = {}
            for side, history in self.palm_history.items():
                stamp = view.stamps.get(f'{side}_arm')
                if stamp is None or not history:
                    view.palm_history[side] = ()
                else:
                    t, palm, sequence = min(history, key=lambda s:abs(s[0]-stamp))
                    view.palm_history[side] = ((t, palm.copy(), sequence),)
            view._hand_joint_targets = (None if self._hand_joint_targets is None
                                        else self._hand_joint_targets.copy())
        # These small objects are control-owned. Copy outside the receiver lock;
        # never deepcopy the device, socket, or receiver histories.
        view.wrist_references = copy.deepcopy(self.wrist_references)
        view.wrist_rotations = dict(self.wrist_rotations)
        view.wrist_input_diagnostics = dict(self.wrist_input_diagnostics)
        try:
            poses = view.advance(full_target=full_target)
            if poses is None:
                raise InputUnavailable('retargeting not ready')
            poses['hand_local_orientations'] = {
                s:r.copy() for s,r in view.wrist_rotations.items()}
            poses['hand_source_orientations'] = {
                s:r.source_local.copy() for s,r in view.wrist_references.items()}
            poses['_wrist_filter_candidates'] = view.wrist_references
            self.wrist_input_diagnostics = view.wrist_input_diagnostics
            # The nearest paired palm can be older than the latest finger
            # sample. Include its actual clock in the post-IK age check too.
            for side, values in view.wrist_input_diagnostics.items():
                stamp = values.get('comparison_hand_sample_monotonic')
                if stamp is not None:
                    ticket[side+'_palm'] = stamp
            # Arrival continuity must refer to the very same Body snapshot as
            # this ticket, even if the receiver commits another frame meanwhile.
            poses['entry_sources'] = {}
            for side in self.sides:
                points = view._current_arm_points(view._source_side_for_target(side))
                if points is not None:
                    poses['entry_sources'][side] = np.concatenate(
                        [points[k] for k in ('shoulder', 'elbow', 'wrist')]).tolist()
            hand = view.hand_joints() if self.with_hand else None
            if self.with_hand and hand is None:
                raise InputUnavailable('PICO finger tracking missing')
            return ticket, poses, hand
        finally:
            # Publish only control-owned scalar/pose replacements. Never copy
            # receiver frames, clocks, source stamps or histories back from view.
            with self.input_lock:
                for name in ('_last_target_update_time_monotonic', '_printed_stale',
                             '_previous_left_pose', '_previous_right_pose',
                             '_previous_left_elbow_pose', '_previous_right_elbow_pose'):
                    setattr(self, name, getattr(view, name))

    def commit_wrist_filters(self, poses, sides):
        """Called only after send/preview acceptance, never for input holds."""
        if poses is None:
            return
        for side in sides:
            reference = poses.get('_wrist_filter_candidates', {}).get(side)
            if reference is not None:
                self.wrist_references[side] = reference
                self.wrist_rotations[side] = reference.filtered.copy()

    def close(self):
        super().close()
        self._thread.join(timeout=.2)
