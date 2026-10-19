"""One SDK subscriber, notification-driven Body forwarding, bounded hand work.

SDK snapshots are owned values. No logger, mapper, or viewer thread subscribes
or calls init/deinit. Only the owner uses wait_tracking_updates.
"""
from collections import Counter, deque
import copy
import json
import signal
import socket
import sys
import threading
import time

import numpy as np


from .config import WORKSPACE
from .diagnostics import RuntimeDiagnostics
from .pico_hand import PicoHands, palm_sample


def require_notifications(sdk):
    version = getattr(sdk, 'get_tracking_updates_version', lambda: 0)()
    if version != 1 or not all(callable(getattr(sdk, name, None)) for name in ('wait_tracking_updates', 'get_tracking_diagnostics', 'close')):
        raise RuntimeError('PICO SDK needs tracking notifications v1; rebuild with laptop_teleop/scripts/install_pico.sh')


def summary_times(times, now_ns):
    """Recent transport rate; retain the true age even after the window expires."""
    times = list(times)[-200:]
    recent = [stamp for stamp in times if 0 <= now_ns-stamp <= 2_000_000_000]
    return dict(hz=(len(recent)-1)*1e9/max(now_ns-recent[0], 1) if len(recent) >= 2 else 0.,
                age_ms=(now_ns-times[-1])/1e6 if times else None)


class TransportMetrics:
    def __init__(self):
        self.lock = threading.Lock()
        self.counts = Counter()
        self.times = {k: deque(maxlen=200) for k in ('notification', 'body_forward', 'left_map', 'right_map')}
        self.hand_status = {}
        self.errors = {}

    def count(self, name, n=1):
        with self.lock:
            self.counts[name] += n

    def mark(self, name, now_ns=None):
        with self.lock:
            self.times[name].append(time.monotonic_ns() if now_ns is None else now_ns)

    def status(self, side, value):
        with self.lock:
            self.hand_status[side] = value

    def status_snapshot(self):
        with self.lock:
            return dict(self.hand_status)

    def error(self, name, error):
        with self.lock:
            self.errors[name] = str(error)

    def error_snapshot(self):
        with self.lock:
            return dict(self.errors)

    def raw(self):
        with self.lock:
            return (dict(self.counts), {k: list(v) for k, v in self.times.items()},
                    dict(self.hand_status))


class LatestHands:
    """At most one unprocessed snapshot per side; no historical frame replay."""
    def __init__(self, metrics):
        self.condition = threading.Condition()
        self.slots = {}
        self.seen = {}
        self.metrics = metrics
        self.next_side = 'left'
        self.closed = False

    def offer(self, side, sample):
        sequence = int(sample.get('sequence', 0))
        with self.condition:
            if self.closed or sequence <= self.seen.get(side, 0):
                return
            self.metrics.count(side+'_coalesced', max(0, sequence-self.seen.get(side, 0)-1))
            self.seen[side] = sequence
            if side in self.slots:
                self.metrics.count(side+'_coalesced')
            self.slots[side] = sample  # A new, owned pybind snapshot, never mutated.
            self.condition.notify()

    def take(self):
        with self.condition:
            self.condition.wait_for(lambda: self.closed or self.slots)
            if self.closed:
                return None
            side = self.next_side if self.next_side in self.slots else next(iter(self.slots))
            self.next_side = 'right' if side == 'left' else 'left'
            return side, self.slots.pop(side)

    def close(self):
        with self.condition:
            self.closed = True
            self.slots.clear()
            self.condition.notify_all()

    def latest(self, side, fallback):
        with self.condition:
            return self.slots.pop(side, fallback)


class AsyncRecording:
    """Original JSONL format with real submission times; bounded 64-packet queue."""
    def __init__(self, writer, metrics, diagnostics):
        self.writer, self.metrics, self.diagnostics = writer, metrics, diagnostics
        self.condition = threading.Condition()
        self.queue = deque()
        self.closed = False
        self.first_valid = None
        self.failed = False
        self.thread = threading.Thread(target=self._run, name='pico-recording', daemon=True)
        self.thread.start()

    def submit(self, packet, encoded_size):
        now = time.monotonic_ns()
        if self.first_valid is None:
            self.first_valid = now
        if (now-self.first_valid)/1e9 < self.writer.warmup_discard_s:
            return
        # Forwarder never mutates a submitted packet. Copy explicitly to keep
        # recording independent of the viewer/caller's subsequent ownership.
        item = (copy.deepcopy(packet), encoded_size, now)
        with self.condition:
            if len(self.queue) == 64:
                self.queue.popleft()
                self.metrics.count('record_dropped')
            self.queue.append(item)
            self.condition.notify()

    def _run(self):
        started = motion = False
        try:
            while True:
                with self.condition:
                    self.condition.wait_for(lambda: self.closed or self.queue)
                    if not self.queue:
                        break
                    packet, size, now = self.queue.popleft()
                _, phase = self.writer.write_packet(packet, size, record_monotonic_ns=now)
                if not started:
                    self.writer.write_event('recording_started', record_monotonic_ns=now,
                                            message='first post-warmup body packet recorded; hold neutral start pose')
                    self.diagnostics.message('[pico] RECORDING STARTED: hold neutral start pose')
                    started = True
                if phase == 'motion' and not motion:
                    self.writer.write_event('motion_started', record_monotonic_ns=now,
                                            message='neutral start segment complete; operator may start teleoperation motion')
                    self.diagnostics.message('[pico] MOTION RECORDING ACTIVE')
                    motion = True
        except Exception as exc:
            self.failed = True
            self.metrics.error('recording', exc)
            self.metrics.count('record_errors')
            self.diagnostics.message('[pico] recording failed:', repr(exc), file=sys.stderr)
        finally:
            try:
                self.writer.close()
            except Exception as exc:
                self.failed = True
                self.metrics.error('recording', exc)
                self.metrics.count('record_errors')
                self.diagnostics.message('[pico] recording close failed:', repr(exc), file=sys.stderr)

    def close(self, timeout=1.):
        with self.condition:
            self.closed = True
            self.condition.notify_all()
        self.thread.join(timeout)
        return not self.thread.is_alive() and not self.failed and not self.queue


def default_hand_mapper():
    from pathlib import Path
    from esrobo_teleop.config import load_config
    cfg = load_config(str(WORKSPACE/'teleoperation/config/teleop_config.yaml'))
    if not Path(cfg.ik.urdf_path).is_absolute():
        cfg.ik.urdf_path = str(WORKSPACE/'teleoperation'/cfg.ik.urdf_path)
    if cfg.hand.left_retarget_mode == 'urdf_10':
        from .hand_worker import ProcessHandMapper
        return ProcessHandMapper(cfg)
    return PicoHands()


class PicoForwarder:
    def __init__(self, sdk, bridge, args, *, viewer=None, diagnostics=None, mapper=None, body_socket=None, hand_socket=None, recording=None, trace=None):
        require_notifications(sdk)
        self.sdk, self.bridge, self.args = sdk, bridge, args
        self.viewer, self.diagnostics, self.recording = viewer, diagnostics, recording
        self.trace = trace
        self.trace_seen = {}
        self.trace_receipts = {}
        self.stop = threading.Event()
        self.metrics = TransportMetrics()
        self.hands = LatestHands(self.metrics)
        self.mapper = default_hand_mapper() if mapper is None else mapper
        self.hand_log_path = WORKSPACE/'laptop_teleop/log'/f'hand_raw_{time.time_ns()}.jsonl'
        self.body_socket = body_socket or socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.hand_socket = hand_socket or socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.body_socket.setblocking(False)
        self.hand_socket.setblocking(False)
        self.hand_thread = threading.Thread(target=self._hands, name='pico-hand-mapping', daemon=True)
        self.summary_thread = threading.Thread(target=self._summaries, name='pico-transport-summary', daemon=True)
        self.sent_total = 0
        self.latest_packet = None
        self.closed = False

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.stop.set()
        self.hands.close()
        if self.hand_thread.is_alive():
            self.hand_thread.join(.6)
        if callable(getattr(self.mapper, 'close', None)):
            self.mapper.close()
        if self.summary_thread.is_alive():
            self.summary_thread.join(.3)
        self.body_socket.close()
        self.hand_socket.close()

    def _trace_samples(self, updates, observed):
        if self.trace is None or not self.trace.enabled:
            return
        for component in ('body', 'left', 'right'):
            sample = updates[component]
            seq = int(sample.get('sequence', 0))
            previous = self.trace_seen.get(component, 0)
            if seq <= previous:
                continue
            self.trace_seen[component] = seq
            poses = sample.get('poses', [])
            if component == 'body':
                selected = {name: poses[i] for i,name in self.bridge.JOINT_TO_FRAME.items()
                            if name in ('waist', 'left_shoulder', 'left_elbow', 'left_wrist',
                                        'right_shoulder', 'right_elbow', 'right_wrist') and i < len(poses)}
            else:
                # Fixed-size numeric ownership copy avoids hundreds of Python
                # scalar visits on the SDK owner thread. Keep malformed raw
                # observations available for diagnostics as well.
                try:
                    selected = np.asarray(poses, dtype=float)
                except (ValueError, TypeError):
                    selected = poses
            self.trace.emit('raw_sample', component=component, source_sequence=seq,
                sequence_gap=max(0, seq-previous-1) if previous else None,
                observed_monotonic_ns=observed, received_monotonic_ns=sample.get('received_monotonic_ns'),
                packet_timestamp_ns=sample.get('packet_timestamp_ns'), device_timestamp_ns=sample.get('timestamp_ns'),
                valid=sample.get('valid'), active=sample.get('active'), flags=sample.get('flags'),
                poses=selected, pose_format='xyz_quat_xyzw', notification_id=updates.get('notification_id'))

    def _send(self, sock, encoded, component):
        try:
            sock.sendto(encoded, (self.args.host, self.args.port))
            return True
        except OSError as exc:
            self.metrics.count(component+'_send_failed')
            self.metrics.error(component+'_send', exc)
            return False

    def _hands(self):
        while not self.stop.is_set():
            item = self.hands.take()
            if item is None:
                break
            side, sample = item
            published = False
            try:
                if side == 'left' and hasattr(self.mapper, 'next_due'):
                    if self.stop.wait(max(0., self.mapper.next_due-time.monotonic())):
                        break
                    sample = self.hands.latest(side, sample)
                map_started = time.monotonic_ns()
                mapped = self.mapper.map_sample(side, sample)
                if mapped is None:
                    self.metrics.count(side+'_duplicate')
                    if self.trace is not None:
                        self.trace.emit('hand_rejected', side=side, source_sequence=sample.get('sequence'), reason='duplicate')
                    continue
                if side == 'left' and self.diagnostics:
                    self.diagnostics.reference(self.hand_log_path, dict(event='hand_retarget', side=side,
                        source_sequence=sample['sequence'], sample=sample, result=mapped))
                # Full geometry is retained locally, never in status/control packets.
                mapped.pop('target_points', None)
                mapped.pop('solved_points', None)
                mapped.pop('reconstructed_points', None)
                mapped.pop('candidate_id', None)
                if side == 'left' and 'diagnostics' in mapped:
                    # Full contact/solver diagnostics were frozen in the local
                    # log above; retain only the existing UI summary on wire.
                    mapped['diagnostics'] = {k: v for k, v in mapped['diagnostics'].items()
                        if k in ('solve_ms', 'tip_error_mm', 'pinch_error_mm', 'limited_axes',
                                 'structural_tip_error_mm', 'pinch_blend', 'nearest_pinch_error_mm')}
                self.metrics.mark(side+'_map')
                # Recheck after mapping and immediately before encoding/output.
                age = time.monotonic()-mapped['sample_monotonic']
                if not 0 <= age <= .1:
                    raise ValueError('PICO Hand stream stale before send')
                mapped['age_s'] = age
                packet = dict(pico_hands={side: {k:v for k,v in mapped.items() if k != 'palm_basis'}},
                    palm_publication_version=1, laptop_input=dict(
                    v=1, kind='pico_hand', sample_monotonic={side: mapped['sample_monotonic']}, physical_sides=[side]))
                encoded = json.dumps(packet, separators=(',', ':'), allow_nan=False).encode()
                if time.monotonic()-mapped['sample_monotonic'] > .1:
                    raise ValueError('PICO Hand stream stale after encoding')
                if not self.stop.is_set() and not self.args.record_only:
                    sent = self._send(self.hand_socket, encoded, side)
                    published = sent
                    if self.trace is not None:
                        self.trace.emit('hand_forward', side=side, source_sequence=sample.get('sequence'),
                            received_monotonic_ns=sample.get('received_monotonic_ns'),
                            map_started_ns=map_started, sent=sent, joints=mapped['joints'], palm_basis=mapped['palm_basis'])
                if self.args.record_only and not self.stop.is_set():
                    published = True
                self.metrics.status(side, 'tracking')
            except (ValueError, TypeError, KeyError, OverflowError) as exc:
                if side == 'left' and self.diagnostics:
                    self.diagnostics.reference(self.hand_log_path, dict(event='hand_retarget_rejected', side=side,
                        source_sequence=sample.get('sequence'), sample=sample, error=str(exc)))
                self.metrics.count(side+('_stale' if 'stale' in str(exc) else '_invalid'))
                self.metrics.status(side, str(exc))
                if self.trace is not None:
                    self.trace.emit('hand_rejected', side=side, source_sequence=sample.get('sequence'), reason=str(exc))
            finally:
                finish = getattr(self.mapper, 'finish', None)
                if finish is not None:
                    try:
                        finish(side, sample.get('sequence'), published)
                    except ValueError as exc:
                        self.metrics.count(side+'_invalid')
                        self.metrics.status(side, str(exc))

    def snapshot(self):
        counts, times, status = self.metrics.raw()
        now = time.monotonic_ns()
        sdk = self.sdk.get_tracking_diagnostics()
        if self.trace is not None:
            for component, data in sdk.items():
                receipts = data['receipts_ns']
                previous = self.trace_receipts.get(component, 0)
                fresh = [t for t in receipts if t > previous]
                if fresh:
                    self.trace_receipts[component] = fresh[-1]
                    self.trace.emit('sdk_receipts', component=component, receipts_ns=fresh,
                                    committed=data['committed'], rejected=data['rejected'], invalid=data['invalid'],
                                    retained_receipt_count=len(receipts), previous_receipt_ns=previous)

        sdk = {k: dict(summary_times(v['receipts_ns'], now),
                      committed=v['committed'], rejected=v['rejected'], invalid=v['invalid'])
               for k, v in sdk.items()}
        timing = {k: summary_times(v, now) for k, v in times.items()}
        for side in ('left', 'right'):
            if status.get(side) == 'tracking':
                ages = (sdk.get(side, {}).get('age_ms'), timing[side+'_map']['age_ms'])
                if any(age is None or not 0 <= age <= 100 for age in ages):
                    status[side] = 'stale'
        return dict(generated_monotonic_ns=now, sdk=sdk, timing=timing,
                    counts=counts, hand_status=status,
                    left_hand_backend=getattr(self.mapper, "compute_backend", None),
                    left_hand_fingerprint=getattr(self.mapper, "fingerprint", None),
                    errors=self.metrics.error_snapshot(),
                    logging=self.diagnostics.stats() if self.diagnostics else {}, web_poll_hz=20.,
                    motion_trace=self.trace.stats() if self.trace is not None else None)

    def _summaries(self):
        last_print = 0.
        while not self.stop.wait(.2):
            try:
                snapshot = self.snapshot()
                if self.viewer:
                    self.viewer.state.update_acquisition_diagnostics(snapshot)
                if self.diagnostics:
                    self.diagnostics.submit(dict(event='pico_transport', **snapshot))
                    now = time.monotonic()
                    if now-last_print >= max(2., self.args.print_interval):
                        sdk = snapshot['sdk']['body']
                        body = snapshot['timing']['body_forward']
                        self.diagnostics.message(f"[pico] sdk_body_hz={sdk['hz']:.1f} forward_hz={body['hz']:.1f} "
                            f"counts={snapshot['counts']} hands={snapshot['hand_status']}")
                        if self.args.print_poses and self.latest_packet:
                            self.diagnostics.message('[pico] arm_poses', self.bridge._format_pose_summary(self.latest_packet))
                        last_print = now
            except Exception as exc:
                self.metrics.count('summary_errors')
                if self.diagnostics:
                    self.diagnostics.message('[pico] diagnostics failed:', repr(exc), file=sys.stderr)

    def run(self):
        self.hand_thread.start()
        self.summary_thread.start()
        period = 1./max(float(self.args.rate_hz), 1.)
        after = seen = 0
        last_sent_stamp = 0
        due = 0.
        pending = None
        try:
            while not self.stop.is_set() and (self.args.samples <= 0 or self.sent_total < self.args.samples):
                now = time.monotonic()
                remaining = due-now
                if pending and 0 < remaining < .001:
                    # Native waits use integer milliseconds. Avoid rounding
                    # every 90 Hz deadline up by 1 ms (and losing input frames).
                    self.stop.wait(remaining)
                    timeout = 0
                else:
                    timeout = min(50, max(0, int(remaining*1000))) if pending else 50
                updates = self.sdk.wait_tracking_updates(after, timeout)
                notified = time.monotonic_ns()
                if self.stop.is_set():
                    break
                if updates['notification_id'] > after:
                    self.metrics.mark('notification', notified)
                    self.metrics.count('notifications_coalesced', updates['notification_id']-after-1)
                    after = updates['notification_id']
                    try:
                        self._trace_samples(updates, notified)
                    except Exception:
                        self.metrics.count('trace_snapshot_errors')
                    sample = updates['body']
                    seq = int(sample['sequence'])
                    if seq > seen:
                        self.metrics.count('body_coalesced', max(0, seq-seen-1)+(1 if pending else 0))
                        seen = seq
                        pending = sample
                now = time.monotonic()
                # Body always goes first; Hand work only sees raw owned snapshots.
                if pending and now >= due:
                    sample = pending
                    pending = None
                    try:
                        if int(sample.get('received_monotonic_ns', 0)) <= 0:
                            raise ValueError('SDK missing native CLOCK_MONOTONIC receipt')
                        if int(sample['packet_timestamp_ns']) <= last_sent_stamp:
                            self.metrics.count('body_duplicate')
                            if self.trace is not None:
                                self.trace.emit('body_rejected', source_sequence=sample['sequence'], reason='duplicate_device_timestamp')
                            for side in ('left', 'right'):
                                self.hands.offer(side, updates[side])
                            continue
                        packet, _ = self.bridge._packet_from_sample(sample, self.sent_total, self.args.include_full_body_arrays)
                        if packet is None:
                            reason = 'body_stale' if now-sample['received_monotonic_ns']/1e9 > .1 else 'body_invalid'
                            self.metrics.count(reason)
                            if self.trace is not None:
                                self.trace.emit('body_rejected', source_sequence=sample['sequence'], reason=reason)
                        else:
                            received = packet['body_sample_monotonic']
                            packet['laptop_input'] = dict(v=1, kind='pico', sample_monotonic=dict(left=received, right=received), physical_sides=['left', 'right'])
                            packet['pico_hand_status'] = self.metrics.status_snapshot()
                            # Body and raw palm share a receive transaction. Finger
                            # fitting stays on the worker and cannot delay the palm.
                            packet['palm_publication_version'] = 1
                            packet['pico_palms'] = {}
                            for side in ('left', 'right'):
                                try:
                                    packet['pico_palms'][side] = palm_sample(updates[side])
                                except (ValueError, TypeError, KeyError, OverflowError):
                                    self.metrics.count(side+'_palm_invalid')
                            encoded = json.dumps(packet, separators=(',', ':'), allow_nan=False).encode()
                            if time.monotonic()-received > .1:
                                self.metrics.count('body_stale')
                                if self.trace is not None:
                                    self.trace.emit('body_rejected', source_sequence=sample['sequence'], reason='stale_after_encoding')
                            elif self.args.record_only or self._send(self.body_socket, encoded, 'body'):
                                sent = time.monotonic_ns()
                                self.metrics.mark('body_forward', sent)
                                if self.trace is not None:
                                    self.trace.emit('body_forward', source_sequence=sample['sequence'],
                                        received_monotonic_ns=sample['received_monotonic_ns'], sent_monotonic_ns=sent,
                                        sent=not self.args.record_only,
                                        palms={s: dict(sequence=v['sequence'], sample_monotonic=v['sample_monotonic'])
                                               for s,v in packet['pico_palms'].items()})
                                last_sent_stamp = int(sample['packet_timestamp_ns'])
                                self.sent_total += 1
                                # Fixed rate-limit deadlines, rather than adding
                                # serialization/scheduler jitter to every period.
                                # A missed whole period starts a new deadline;
                                # there is no catch-up loop or historical queue.
                                sent_s = sent/1e9
                                due = (sent_s+period if due == 0. or sent_s-due >= period
                                       else due+period)
                                self.latest_packet = packet
                                if self.viewer:
                                    self.viewer.update(packet)
                                if self.recording:
                                    self.recording.submit(packet, len(encoded))
                            elif self.trace is not None:
                                self.trace.emit('body_rejected', source_sequence=sample['sequence'], reason='send_failed')
                    except (ValueError, TypeError, KeyError, OverflowError) as exc:
                        self.metrics.count('body_invalid')
                        if self.trace is not None:
                            self.trace.emit('body_rejected', source_sequence=sample.get('sequence'), reason=str(exc))
                for side in ('left', 'right'):
                    self.hands.offer(side, updates[side])
        finally:
            self.close()
        return 0


def run_pico(bridge, args):
    import xrobotoolkit_sdk as sdk
    require_notifications(sdk)  # Reject old binaries before starting a subscription.
    if args.record_only and not args.record_path:
        raise ValueError('--record-only requires --record-path')
    log_path = WORKSPACE/'laptop_teleop/log'/f'pico_transport_{time.time_ns()}.jsonl'
    diagnostics = RuntimeDiagnostics(log_path)
    from esrobo_teleop.debug.motion_trace import MotionTrace, metadata
    trace = MotionTrace(log_path.with_name(log_path.name.replace('pico_transport_', 'pico_motion_')),
                        info=metadata(WORKSPACE, role='pico', arguments=vars(args)))
    viewer = recording = forwarder = None
    initialized = False
    cleanup_errors = []
    old_handlers = {}
    try:
        if args.viewer_host:
            from esrobo_teleop.config import load_config
            from esrobo_teleop.debug.skeleton_viewer import SkeletonViewerServer
            rotation = load_config(str(WORKSPACE/'teleoperation/config/teleop_config.yaml')).retarget.source_to_robot_rotation
            viewer = SkeletonViewerServer(args.viewer_host, args.viewer_port, source_to_robot_rotation=rotation)
            viewer.start()
        forwarder = PicoForwarder(sdk, bridge, args, viewer=viewer, diagnostics=diagnostics, trace=trace)
        if args.record_path:
            writer = bridge.BodyRecordingWriter(args.record_path, args.record_flush_interval,
                                                 args.record_neutral_start_s, args.record_warmup_discard_s)
            recording = AsyncRecording(writer, forwarder.metrics, diagnostics)
            forwarder.recording = recording
        if threading.current_thread() is threading.main_thread():
            for signum in (signal.SIGTERM, signal.SIGINT):
                old_handlers[signum] = signal.signal(signum, lambda *_: forwarder.stop.set())
        sdk.init()
        initialized = True
        diagnostics.message('[pico] tracking notifications v1; one SDK subscription; diagnostics:', str(log_path))
        return forwarder.run()
    finally:
        if forwarder:
            forwarder.close()
        if not trace.close():
            diagnostics.message('[pico] detailed trace incomplete:', trace.stats())
        # Workers that depend on native snapshots have stopped before owner deinit.
        if initialized:
            try:
                sdk.close()  # The vendor's Python name for native deinit().
            except Exception as exc:
                cleanup_errors.append(('SDK close', exc))
        if recording and not recording.close():
            diagnostics.message('[pico] recording failed or shutdown timed out; persistence incomplete', file=sys.stderr)
        if viewer:
            try:
                viewer.stop()
            except Exception as exc:
                cleanup_errors.append(('viewer stop', exc))
        for signum, handler in old_handlers.items():
            signal.signal(signum, handler)
        for component, exc in cleanup_errors:
            diagnostics.message('[pico] cleanup failed:', component, repr(exc), file=sys.stderr)
        diagnostics.close()
        if cleanup_errors:
            raise RuntimeError('PICO cleanup incomplete: '+', '.join(c for c,_ in cleanup_errors))
