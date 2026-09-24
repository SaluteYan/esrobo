"""Receive feedback, compute fresh targets, and send protocol-v1 commands."""
import argparse
from collections import deque
import json
from pathlib import Path
import select
import signal
import sys
import time

import numpy as np

from esrobo_link.client import RobotClient
from esrobo_link.protocol import ProtocolError, canonical

from .config import ROOT, read_robot_config, read_settings
from .contract import verify_contract
from .inputs import InputUnavailable
from .pipeline import Pipeline


class ControlRateMonitor:
    """Fail a live session before the robot's 200 ms target lease expires."""
    def __init__(self, minimum_hz, gap_timeout_s, rate_tolerance_hz=None, rate_window_s=1.0):
        self.minimum_hz = float(minimum_hz)
        # A one-second floating window can report 39.9 Hz for a nominal 40 Hz
        # stream because of scheduler jitter. The independent gap watchdog
        # still catches every interruption longer than gap_timeout_s.
        self.rate_tolerance_hz = (max(0.5, self.minimum_hz * 0.01)
                                  if rate_tolerance_hz is None else float(rate_tolerance_hz))
        self.rate_window_s = float(rate_window_s)
        if self.rate_window_s <= 0:
            raise ValueError("control rate window must be positive")
        self.gap_timeout_s = float(gap_timeout_s)
        self.started = self.last_target = None
        self.targets = deque(maxlen=200)

    def observe(self, now, mode, has_target):
        if mode not in ("ARMING", "ACTIVE"):
            self.started = self.last_target = None
            self.targets.clear()
            return
        if self.started is None:
            self.started = now
        if has_target:
            self.last_target = now
            self.targets.append(now)
        if self.last_target is None or now-self.last_target > self.gap_timeout_s:
            raise RuntimeError(f"control target gap exceeded {self.gap_timeout_s*1000:.0f} ms")
        while self.targets and now-self.targets[0] > self.rate_window_s:
            self.targets.popleft()
        if now-self.started >= self.rate_window_s:
            hz = ((len(self.targets)-1)/(self.targets[-1]-self.targets[0])
                  if len(self.targets) > 1 else 0.)
            if hz < self.minimum_hz - self.rate_tolerance_hz:
                raise RuntimeError(f"control rate {hz:.1f} Hz below required {self.minimum_hz:.1f} Hz "
                                   f"(tolerance {self.rate_tolerance_hz:.1f} Hz; stop below "
                                   f"{self.minimum_hz-self.rate_tolerance_hz:.1f} Hz over "
                                   f"{self.rate_window_s:.0f} s)")


def feedback_for(state, settings, received_at, now=None):
    now = time.monotonic() if now is None else now
    feedback = state.get("feedback", {})
    members = feedback.get("sides", {}) if settings.side == "both" else {settings.side: feedback}
    cache = state.get("feedback_cache_age_s")
    if cache is None or not np.isfinite(cache) or cache < 0 or now < received_at:
        raise InputUnavailable("invalid feedback cache age")
    # The peer's sampling + cache age, local computation time, and a configured
    # transport budget are distinct. This is not a clock-synchronized latency measurement.
    lag = cache + now-received_at + settings.network_margin_s
    full = np.zeros(14)  # Non-controlled side is model-only in single-side mode.
    for side in settings.sides:
        item = members.get(side, {})
        ages = []
        if settings.hand_only:
            if item.get("enable_states") != [False] * 7:
                raise InputUnavailable(f"{side} arm must remain verified disabled in hand-only mode")
            if item.get("controller_fault") or item.get("driver_fault"):
                raise InputUnavailable(f"{side} arm safety fault in hand-only mode")
        else:
            q = np.asarray(item.get("arm_urdf_rad"), dtype=float)
            if q.shape != (7,) or not np.all(np.isfinite(q)):
                raise InputUnavailable(f"{side} seven-joint feedback missing")
            ages.append((item.get("arm_feedback_age_s"), settings.feedback_max_age_s))
            start = 0 if side == "left" else 7
            full[start:start+7] = q
        if settings.with_hand:
            hand = item.get("hand") or {}
            value = np.asarray(hand.get("position_unit"), dtype=float)
            if value.shape != (10,) or not np.all(np.isfinite(value)) or np.any((value < 0) | (value > 255)):
                raise InputUnavailable(f"{side} hand feedback missing/invalid")
            ages.append((hand.get("age_s"), settings.hand_feedback_max_age_s))
        if state["mode"] != "ARMING":
            if any(a is None or not np.isfinite(a) or a < 0 or a+lag > maximum
                   for a, maximum in ages):
                raise InputUnavailable(f"{side} robot feedback stale")
    return full, members


def measured_targets(members, settings):
    return {side: dict(arm_urdf_rad=(None if settings.hand_only else list(members[side]["arm_urdf_rad"])),
                       hand_unit=([round(v) for v in members[side]["hand"]["position_unit"]]
                                  if settings.with_hand else None)) for side in settings.sides}


def send_targets(client, targets, settings):
    if settings.hand_only:
        client.send_hand_target(targets[settings.side]["hand_unit"])
    elif settings.side == "both":
        client.send_dual_target(
            left_arm_urdf_rad=targets["left"]["arm_urdf_rad"],
            right_arm_urdf_rad=targets["right"]["arm_urdf_rad"],
            left_hand_unit=targets["left"]["hand_unit"],
            right_hand_unit=targets["right"]["hand_unit"])
    else:
        client.send_target(**targets[settings.side])


class Controller:
    """One session; faults terminate following and require an explicit restart."""
    def __init__(self, client, pipeline, settings, contract, preview=False):
        self.client, self.pipeline, self.settings = client, pipeline, settings
        self.contract = canonical(contract)
        self.mode = "IDLE"
        self.preview = preview
        self.sent = 0
        self.computed = 0
        self.preparing = False
        if preview:
            self.pipeline.restart_preparation()

    def step(self, state):
        if canonical(state["contract"]) != self.contract:
            raise ProtocolError("gateway configuration changed during session")
        mode = state["mode"]
        if mode not in ("IDLE", "RETURNING", "CALIBRATING", "ARMING", "ACTIVE"):
            raise RuntimeError(f"robot {mode}: {state.get('reason', '')}; restart after local recovery")
        if mode == "RETURNING":
            # Reference collection starts only after the robot has completed
            # its return and reported CALIBRATING.  This preserves the user
            # visible order: robot zero first, then operator pose capture.
            self.preparing = False
            self.pipeline.preparation_status = "机器人正在回零并张开灵巧手，请等待完成"
            self.mode = mode
            return None
        if mode == "CALIBRATING" and not self.preparing:
            # A fast/mock return can complete between two 50 Hz state packets,
            # so CALIBRATING itself must also start a fresh reference cycle.
            self.pipeline.restart_preparation()
            self.preparing = True
        measured, feedback = feedback_for(state, self.settings, self.client.received_at)
        with self.pipeline.body.input_lock:
            if mode == "CALIBRATING" and not self.settings.hand_only and not self.pipeline.body.is_ready():
                self.pipeline.preparation_status = (
                    "请把手臂抬到胸前、肘部自然弯曲，双手自然张开并保持在头显视野内"
                )
            # Before e, only prove fresh source streams and send the measured
            # robot pose back as a no-jump target.  The real session reference
            # is deliberately collected after the robot has returned to zero.
            require_reference = self.preview or mode != "IDLE" or self.preparing
            ticket = self.pipeline.body.ticket(
                self.settings.input_max_age_s, require_reference=require_reference)
            if ticket is None:
                return None
            if mode == "CALIBRATING" and not self.pipeline.preparation_ready():
                return None
            if require_reference and (not self.pipeline.initialized
                                      or (mode == "ACTIVE" and self.mode != "ACTIVE")):
                self.pipeline.initialize(measured)
            frame = (self.pipeline.capture() if require_reference else
                     (ticket, None, self.pipeline.body.hand_joints()))
        if frame is None:
            return None
        if mode in ("CALIBRATING", "ACTIVE") or self.preview:
            targets = self.pipeline.solve(frame, measured, feedback)
        else:
            targets = measured_targets(feedback, self.settings)
        # Recheck original sample/feedback ages AFTER potentially slow IK. Never
        # stamp old sensor input with the time at which computation completed.
        self.pipeline.check_ticket(frame[0])
        feedback_for(state, self.settings, self.client.received_at)
        if not self.preview:
            send_targets(self.client, targets, self.settings)
            self.sent += 1
        self.pipeline.publish_diagnostics(
            frame, targets, measured, preview=self.preview, robot_mode=mode)
        self.pipeline.body.consume(frame[0])
        self.computed += 1
        self.mode = mode
        if mode == "ACTIVE":
            self.preparing = False
        return targets


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("command", choices=("run", "inspect", "check"))
    p.add_argument("--config", default=str(ROOT / "config/laptop.yaml"))
    p.add_argument("--host")
    p.add_argument("--port", type=int)
    p.add_argument("--side", choices=("left", "right", "both"))
    p.add_argument("--arm-only", action="store_true")
    p.add_argument("--hand-only", action="store_true")
    p.add_argument("--key-file")
    p.add_argument("--input-port", type=int)
    p.add_argument("--mock", action="store_true", help="accept only explicit mock contracts on loopback")
    p.add_argument("--preview", action="store_true", help="compute targets but send no commands")
    p.add_argument("--seconds", type=float, default=0)
    p.add_argument("--log-file", help="new JSONL file; default laptop_teleop/log/session_<time>.jsonl")
    p.add_argument("--status-file", help="atomic, token-free live status for the local dashboard")
    return p


def main(argv=None):
    argument_parser = parser()
    args = argument_parser.parse_args(argv)
    settings = read_settings(args.config)
    for arg, field in (("host", "robot_host"), ("port", "robot_port"), ("side", "side"),
                       ("key_file", "key_file"), ("input_port", "input_port")):
        value = getattr(args, arg)
        if value is not None:
            setattr(settings, field, value)
    if args.arm_only:
        settings.with_hand = False
    if args.hand_only:
        if args.arm_only:
            argument_parser.error("--arm-only and --hand-only cannot be combined")
        settings.with_hand, settings.hand_only = True, True
    settings.validate()
    if args.mock and settings.robot_host not in ("127.0.0.1", "localhost"):
        raise ValueError("--mock is restricted to the local mock gateway")
    cfg = read_robot_config(settings)
    if args.command == "check":
        import importlib.util
        if settings.hand_only:
            model_dof = None
        else:
            from esrobo_teleop.ik.solver import IkSolver
            model_dof = IkSolver(cfg.ik)._model.nq
        required_sources = [
            f"{side}_{kind}"
            for side in settings.sides
            for kind in (("hand",) if settings.hand_only else
                         ("arm", "hand", "wrist") if settings.with_hand else ("arm",))
        ]
        print(json.dumps(dict(urdf=(None if settings.hand_only else cfg.ik.urdf_path), model_dof=model_dof,
                              robot=f"{settings.robot_host}:{settings.robot_port}",
                              side=settings.side, with_hand=settings.with_hand,
                              hand_only=settings.hand_only,
                              required_input_sources=required_sources,
                              required_robot_services=(
                                  ["gateway", "hands", "hand_bridge"]
                                  if settings.with_hand else ["gateway"]
                              ),
                              xrobotoolkit_sdk=importlib.util.find_spec("xrobotoolkit_sdk") is not None), indent=2))
        return 0
    client = RobotClient(settings.robot_host, settings.robot_port, Path(settings.key_file).expanduser())
    pipeline = None
    journal = None
    controlling = False
    old_sigterm = signal.getsignal(signal.SIGTERM)
    def terminate(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, terminate)
    try:
        state = client.connect()
        contracts = verify_contract(state["contract"], cfg, settings, mock=args.mock)
        client.expected_contract_id = state["contract"]["id"]
        print(f"Verified {settings.side} gateway {settings.robot_host}:{settings.robot_port}; contract={client.expected_contract_id}")
        if args.command == "inspect":
            # Omit session/lease tokens from user-visible diagnostics.
            print(json.dumps({k: state.get(k) for k in ("contract", "mode", "reason", "feedback", "feedback_cache_age_s")}, indent=2))
            return 0
        if state["mode"] != "IDLE":
            raise RuntimeError("start the laptop session only with an IDLE gateway")
        pipeline = Pipeline(cfg, settings, contracts)
        controller = Controller(client, pipeline, settings, state["contract"], args.preview)
        log_path = Path(args.log_file) if args.log_file else ROOT / "log" / f"session_{time.time_ns()}.jsonl"
        log_path.parent.mkdir(parents=True, exist_ok=True)
        journal = log_path.open("x")
        print(f"Listening for laptop adapters on 127.0.0.1:{settings.input_port}; log={log_path}")
        if settings.hand_only:
            pose_instruction = f"naturally open the selected {settings.side} hand in the headset view"
        elif settings.with_hand:
            pose_instruction = "raise the selected forearm(s) in front of the chest with bent elbows and naturally open hands"
        else:
            pose_instruction = "raise the selected forearm(s) in front of the chest with bent elbows"
        print(f"Click Enable once. The robot returns to zero first; then {pose_instruction}.")
        print("PICO stability and open-hand detection automatically start following. Laptop s/x/q or Ctrl+C stops and exits.")
        started = time.monotonic()
        next_print, next_record = 0., 0.
        loop_times, compute_times, publish_times = deque(maxlen=200), deque(maxlen=200), deque(maxlen=200)
        compute_durations_ms = deque(maxlen=200)
        # Hand-only mapping is fast, but independent optical Hand frames can
        # briefly arrive slower than the feedback loop. Average over two
        # seconds while keeping the short-gap guard unchanged.
        control_monitor = ControlRateMonitor(settings.minimum_control_rate_hz,
                                             settings.control_gap_timeout_s,
                                             rate_tolerance_hz=(settings.hand_only_rate_tolerance_hz
                                                                if settings.hand_only else None),
                                             rate_window_s=2.0 if settings.hand_only else 1.0)
        period, next_cycle, deadline_misses = 1/settings.rate_hz, time.monotonic(), 0
        def rate(samples):
            return ((len(samples)-1)/(samples[-1]-samples[0])
                    if len(samples) > 1 and samples[-1] > samples[0] else 0.)
        last_mode = None
        while not args.seconds or time.monotonic()-started < args.seconds:
            begin = time.monotonic()
            if sys.stdin.isatty() and select.select([sys.stdin], [], [], 0)[0]:
                if sys.stdin.readline().strip().lower() in ("s", "x", "q"):
                    break
            targets = None
            status = "waiting"
            try:
                state = client.receive(timeout=.08)
                controlling = not args.preview
                step_started = time.perf_counter()
                targets = controller.step(state)
                if targets is not None:
                    compute_times.append(time.monotonic())
                    compute_durations_ms.append((time.perf_counter()-step_started)*1000.)
                status = state["mode"] if targets is not None else "waiting for next complete source frame"
            except (InputUnavailable, TimeoutError) as exc:
                status = str(exc)
                if isinstance(exc, InputUnavailable) and "_hand" in status:
                    with pipeline.body.input_lock:
                        details = []
                        for side in settings.sides:
                            key = f"{side}_hand"
                            if key not in status:
                                continue
                            stamp = pipeline.body.stamps.get(key)
                            age = "never" if stamp is None else f"{(time.monotonic()-stamp)*1000:.0f} ms"
                            sdk = pipeline.body.pico_hand_status.get(side, "unknown")
                            sdk_at = pipeline.body.pico_hand_status_at
                            sdk_age = "unknown" if sdk_at is None else f"{(time.monotonic()-sdk_at)*1000:.0f} ms old"
                            details.append(f"{side} sample age={age}, PICO SDK={sdk} ({sdk_age})")
                        if details:
                            status += "; " + "; ".join(details)
                if state["mode"] in ("ACTIVE", "ARMING"):
                    raise RuntimeError(f"following stopped: {status}") from exc
            now = time.monotonic()
            loop_times.append(now)
            if not args.preview:
                control_monitor.observe(now, state["mode"], targets is not None)
            record = dict(unix_ns=time.time_ns(), monotonic_s=now, mode=state["mode"], status=status,
                          sent=controller.sent, preview=args.preview, targets=targets,
                          feedback=state.get("feedback"), feedback_cache_age_s=state.get("feedback_cache_age_s"),
                          input_rejection=pipeline.body.last_rejection)
            if state["mode"] in ("RETURNING", "CALIBRATING"):
                record["preparation_status"] = pipeline.preparation_status
            with pipeline.body.input_lock:
                record["sources"] = {k: max(0., now-v) for k, v in pipeline.body.stamps.items()}
                record["source_hz"] = pipeline.body.source_rates(now)
                record["required_sources"] = pipeline.body.required
                record["input_source"] = "pico"
                record["pico_hand_status"] = dict(pipeline.body.pico_hand_status)
                record["pico_hand_status_age_s"] = (None if pipeline.body.pico_hand_status_at is None else
                                                    now-pipeline.body.pico_hand_status_at)
                body_reference_ready = settings.hand_only or pipeline.body.is_ready()
                hand_reference_ready = (not settings.with_hand
                                        or all(side in pipeline.hand_references
                                               for side in settings.sides))
                record["body_reference_ready"] = body_reference_ready
                record["hand_reference_ready"] = hand_reference_ready
                record["reference_ready"] = body_reference_ready and hand_reference_ready
            record["initialized"] = pipeline.initialized
            record["computed"] = controller.computed
            durations = np.asarray(compute_durations_ms, dtype=float)
            record["rates"] = dict(loop_hz=rate(loop_times), compute_hz=rate(compute_times),
                                   status_hz=rate(publish_times), configured_hz=settings.rate_hz,
                                   minimum_hz=settings.minimum_control_rate_hz,
                                   rate_tolerance_hz=control_monitor.rate_tolerance_hz,
                                   rate_window_s=control_monitor.rate_window_s,
                                   stop_below_hz=(control_monitor.minimum_hz-
                                                  control_monitor.rate_tolerance_hz),
                                   deadline_misses=deadline_misses,
                                   compute_ms_median=(None if not len(durations) else float(np.median(durations))),
                                   compute_ms_p95=(None if not len(durations) else float(np.percentile(durations, 95))),
                                   compute_ms_max=(None if not len(durations) else float(np.max(durations))))
            if now >= next_record or state["mode"] != last_mode:
                publish_times.append(now)
                record["rates"]["status_hz"] = rate(publish_times)
                with pipeline.body.input_lock:
                    record["pico_hand_input_rad"] = {
                        side: angles.tolist() for side, angles in pipeline.body.pico_hand_targets.items()
                        if side in settings.sides
                    }
                    record["pico_hand_reference_rad"] = {
                        side: np.asarray(reference, dtype=float).tolist()
                        for side, reference in pipeline.hand_references.items()
                        if side in settings.sides
                    }
                journal.write(json.dumps(record, allow_nan=False)+"\n")
                journal.flush()
                if args.status_file:
                    status_path = Path(args.status_file)
                    status_path.parent.mkdir(parents=True, exist_ok=True)
                    temporary = status_path.with_suffix(".tmp")
                    temporary.write_text(json.dumps(record, allow_nan=False), encoding="utf-8")
                    temporary.replace(status_path)
                next_record = now+.2
            if now >= next_print or state["mode"] != last_mode:
                print(f"[laptop] {status}; targets sent={controller.sent}", flush=True)
                next_print = now+2
            last_mode = state["mode"]
            next_cycle += period
            finished = time.monotonic()
            if finished > next_cycle:
                deadline_misses += 1
                if finished-next_cycle >= period:
                    next_cycle = finished
            time.sleep(max(0., next_cycle-time.monotonic()))
        return 0
    except KeyboardInterrupt:
        return 0
    except (OSError, ValueError, RuntimeError, TimeoutError) as exc:
        print(f"[laptop] {exc}", file=sys.stderr, flush=True)
        return 1
    finally:
        signal.signal(signal.SIGTERM, old_sigterm)
        if controlling:
            client.close()  # Best-effort authenticated STOP, otherwise gateway lease expires.
        else:
            client.socket.close()
        if pipeline:
            pipeline.close()
        if journal:
            journal.close()


if __name__ == "__main__":
    raise SystemExit(main())
