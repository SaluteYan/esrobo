"""Independent network watchdog and serialized robot I/O worker."""
import argparse
import json
import socket
import threading
import time

from .backends import MockBackend, MockDualBackend
from .protocol import MAX_PACKET, ProtocolError, key_from_file, pack, unpack
from .session import SessionGate


class Gateway:
    def __init__(self, backend, key, bind="127.0.0.1", port=16000, max_age=0.2, journal=None):
        if not .05 <= max_age <= .5:
            raise ValueError("max target age must be between 0.05 and 0.5 s")
        self.backend, self.key = backend, key
        self.gate = SessionGate(backend.contract, max_age)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((bind, port))
        self.socket.settimeout(.005)
        self.address = self.socket.getsockname()
        self.lock = threading.RLock()
        self.cancelled = threading.Event()
        self.shutdown = threading.Event()
        self.network_done = threading.Event()
        self.mode = "IDLE"
        self.reason = "local e required; no automatic enable"
        self.pending = None
        self.stop_pending = False
        self.stop_send_returned = None
        self.disable_verified = None
        self.feedback = {}
        self.feedback_time = None
        self.rejected_packets = 0
        self.last_rejection = ""
        self.last_return = None
        self.journal = journal
        self._last_record = None
        self._record_time = 0.0

    def record(self):
        """5 Hz diagnostic snapshots plus state changes; never log key/lease."""
        if self.journal is None:
            return
        now = time.monotonic()
        with self.lock:
            identity = (self.mode, self.reason, self.stop_send_returned, self.disable_verified)
            if identity == self._last_record and now - self._record_time < .2:
                return
            entry = dict(unix_ns=time.time_ns(), monotonic_s=now, mode=self.mode,
                         reason=self.reason, contract_id=self.backend.contract["id"],
                         accepted_seq=self.gate.sequence, target=self.gate.target,
                         target_remaining_s=max(0.0, self.gate.deadline-now),
                         feedback=self.feedback, stop_send_returned=self.stop_send_returned,
                         disable_verified=self.disable_verified, recovery_supported=True,
                         feedback_cache_age_s=None if self.feedback_time is None else now-self.feedback_time,
                         rejected_packets=self.rejected_packets, last_rejection=self.last_rejection,
                         last_return=self.last_return)
        self.journal.write(json.dumps(entry, allow_nan=False) + "\n")
        self.journal.flush()
        self._last_record, self._record_time = identity, now

    def trip(self, reason):
        with self.lock:
            self.cancelled.set()
            if self.mode != "FAULT":
                self.stop_pending = self.mode in ("ACTIVE", "ARMING", "RETURNING")
                self.stop_send_returned = None
                self.disable_verified = None
            self.mode, self.reason = "FAULT", reason
            if self.pending not in ("d", "dl", "dr", "dh"):
                self.pending = None
        self.backend.cancel()  # Only sets an Event; never performs network/CAN I/O.

    def watchdog(self):
        with self.lock:
            if self.mode in ("ACTIVE", "ARMING") and not self.gate.fresh(time.monotonic()):
                self.trip("joint target lease expired; explicit local recovery required")

    def request(self, action):
        """Only the robot operator calls enable/disable/return; the wire supports STOP only."""
        if action in ("x", "s", "q"):
            self.trip("operator requested stop")
            # Even idle x must send a stop (controller may have external state).
            with self.lock:
                self.stop_pending = True
            if action == "q":
                self.shutdown.set()
            return
        with self.lock:
            if action == "d":
                self.trip("operator requested disable")
                self.disable_verified = None
                self.pending = "d"
            elif action in ("dl", "dr", "dh"):
                label = {"dl": "left arm", "dr": "right arm", "dh": "hands"}[action]
                self.trip(f"operator requested {label} disable")
                self.pending = action
            elif action == "e":
                if not getattr(self.backend, "supports_prepare", True):
                    raise RuntimeError("automatic session preparation is unavailable without an inter-arm return checker")
                if self.mode != "IDLE" or self.pending or not self.gate.fresh(time.monotonic()):
                    raise RuntimeError("e requires IDLE and a fresh input/feedback stream")
                self.cancelled.clear()
                self.pending, self.mode = "p", "RETURNING"
                self.reason = "returning robot arm and hand to verified zero"
            elif action == "z":
                if not getattr(self.backend, "supports_return", True):
                    raise RuntimeError("z unavailable in hand-only mode or without an inter-arm return checker; no motion sent")
                if self.mode not in ("IDLE", "FAULT") or self.pending or self.stop_pending:
                    raise RuntimeError("stop first and wait for stop handling before z")
                self.cancelled.clear()
                self.pending, self.mode = "z", "RETURNING"
            elif action == "r":
                if self.mode != "FAULT" or self.pending or self.stop_pending or self.stop_send_returned is False:
                    raise RuntimeError("recovery requires a stopped FAULT gateway")
                if getattr(self.backend, "hand_only", False):
                    self.cancelled.clear()
                    self.pending, self.mode = "r", "RECOVERING"
                    self.reason = "verifying disabled arm and hand feedback without motion"
                elif getattr(self.backend, "supports_return", True):
                    self.cancelled.clear()
                    self.pending, self.mode = "z", "RETURNING"
                    self.reason = "explicit single-arm recovery; verified return to zero"
                else:
                    raise RuntimeError("dual-arm recovery requires the supervised inter-arm procedure; no motion sent")
            else:
                raise ValueError("keys: e prepare+enable, s/x stop, d all, dl left, dr right, dh hands, r recover, z return, q exit")

    def network_loop(self):
        next_state = 0.0
        try:
            while not self.network_done.is_set():
                self.watchdog()
                try:
                    packet, peer = self.socket.recvfrom(MAX_PACKET + 1)
                    message = unpack(packet, self.key)
                    with self.lock:
                        self.watchdog()  # A late packet cannot revive an expired stream.
                        now = time.monotonic()
                        if message.get("type") == "hello":
                            self.gate.hello(message, peer, now,
                                            self.mode in ("ACTIVE", "ARMING", "RETURNING", "RECOVERING", "CALIBRATING"))
                        else:
                            kind = self.gate.accept(message, peer, now)
                            if kind == "stop":
                                self.trip("laptop requested stop")
                                self.stop_pending = True
                except socket.timeout:
                    pass
                except (ProtocolError, ValueError, TypeError) as exc:
                    with self.lock:
                        self.rejected_packets += 1
                        self.last_rejection = str(exc)[:180]
                self.watchdog()
                now = time.monotonic()
                if now >= next_state:
                    # Keep the 50 Hz state stream on its original clock. Reset
                    # only after a full missed period; rebasing on every send
                    # accumulates socket/scheduler latency and slows the laptop
                    # control loop, which waits for each fresh state packet.
                    next_state += .02
                    if next_state <= now:
                        next_state = now + .02
                    with self.lock:
                        if self.gate.peer is None:
                            continue
                        status = dict(mode=self.mode, reason=self.reason,
                                      feedback=self.feedback,
                                      feedback_cache_age_s=None if self.feedback_time is None else now - self.feedback_time,
                                      stop_send_returned=self.stop_send_returned,
                                      disable_verified=self.disable_verified, recovery_supported=True,
                                      rejected_packets=self.rejected_packets, last_rejection=self.last_rejection,
                                      last_return=self.last_return)
                        packet = pack(self.gate.state(now, status), self.key)
                        peer = self.gate.peer
                    try:
                        self.socket.sendto(packet, peer)
                    except OSError:
                        self.trip("feedback send failed")
        except Exception as exc:
            self.trip(f"network worker failed: {exc}")
            self.shutdown.set()

    def motion_cancelled(self):
        self.watchdog()
        return self.cancelled.is_set() or self.shutdown.is_set()

    def tick(self):
        """Single executor: all hardware operations are serialized here."""
        self.watchdog()
        with self.lock:
            if (self.mode == "CALIBRATING" and self.pending is None
                    and self.gate.target is not None and self.gate.fresh(time.monotonic())):
                self.pending, self.mode = "e", "ARMING"
                self.reason = "PICO preparation verified; enabling from matching zero target"
            stop, self.stop_pending = self.stop_pending, False
            operation, self.pending = self.pending, None
            target = self.gate.target
        if stop:
            try:
                self.backend.stop()
                self.stop_send_returned = True
            except Exception as exc:
                self.stop_send_returned = False
                self.reason = f"stop send failed: {exc}; hardware state unverified"
        if self.shutdown.is_set():
            return
        try:
            if operation == "d":
                disabled = self.backend.disable()
                with self.lock:
                    self.disable_verified = bool(disabled)
                    self.reason = f"disable_verified={disabled}; z required before following"
            elif operation in ("dl", "dr"):
                side = "left" if operation == "dl" else "right"
                disabled = self.backend.disable_side(side)
                with self.lock:
                    self.reason = f"{side}_disable_verified={disabled}; restart session before following"
            elif operation == "dh":
                disabled = self.backend.disable_hands()
                with self.lock:
                    self.reason = f"hands_disable_verified={disabled}; restart session before following"
            elif operation == "p":
                result = self.backend.prepare_session(self.motion_cancelled)
                with self.lock:
                    self.last_return = result
                    if (not self.motion_cancelled() and result.get("returned")
                            and result.get("disabled") and result.get("hands_open")):
                        self.mode = "CALIBRATING"
                        self.reason = ("robot_zero_verified=True; waiting for raised-arm "
                                       "and natural-open PICO pose")
                        self.gate.target, self.gate.deadline = None, 0.0
                    else:
                        raise RuntimeError(f"session preparation failed/cancelled: {result}")
            elif operation == "e":
                self.backend.enable(target, self.motion_cancelled)
                with self.lock:
                    if self.motion_cancelled():
                        raise RuntimeError("enable interrupted")
                    self.mode, self.reason = "ACTIVE", "following fresh laptop targets"
            elif operation == "z":
                result = self.backend.return_zero(self.cancelled.is_set)
                with self.lock:
                    self.last_return = result
                    if not self.cancelled.is_set() and result["returned"] and result["disabled"]:
                        self.mode, self.reason = "IDLE", "zero/disable verified; local e required"
                        self.gate.target, self.gate.deadline = None, 0.0
                    else:
                        raise RuntimeError(f"return failed/cancelled: {result}")
            elif operation == "r":
                result = self.backend.recover_idle()
                with self.lock:
                    self.last_return = result
                    if not self.cancelled.is_set() and result.get("disabled") and result.get("hands_ready"):
                        self.mode, self.reason = "IDLE", "hand-only disable/feedback verified; local e required"
                        self.gate.target, self.gate.deadline = None, 0.0
                    else:
                        raise RuntimeError(f"hand-only recovery failed/cancelled: {result}")
            with self.lock:
                active, target = self.mode == "ACTIVE", self.gate.target
            if active:
                if self.motion_cancelled():
                    return
                self.backend.step(target, self.motion_cancelled)
            feedback = self.backend.snapshot()
            with self.lock:
                self.feedback, self.feedback_time = feedback, time.monotonic()
        except Exception as exc:
            self.trip(f"executor: {type(exc).__name__}: {exc}")
            # enable can fail after partially enabling hardware; stop even if
            # an asynchronous watchdog has already changed the state to FAULT.
            if operation in ("e", "p"):
                with self.lock:
                    self.stop_pending = True

    def run(self):
        thread = threading.Thread(target=self.network_loop, name="robot-link-network", daemon=True)
        thread.start()
        try:
            while True:
                started = time.monotonic()
                self.tick()
                self.record()
                if self.shutdown.is_set():
                    break
                time.sleep(max(0.0, .02 - (time.monotonic() - started)))
        finally:
            # Covers exceptions/KeyboardInterrupt outside the normal operator path.
            try:
                self.backend.stop()
            finally:
                self.network_done.set()
                thread.join(timeout=1)
                self.socket.close()
                self.backend.close()


def main():
    ap = argparse.ArgumentParser(description="Robot-side joint executor; mock by default")
    ap.add_argument("--bind", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=16000)
    ap.add_argument("--key-file", required=True)
    ap.add_argument("--side", choices=("left", "right", "both"), default="left",
                    help="both selects both arms AND both hands")
    ap.add_argument("--with-hand", action="store_true")
    ap.add_argument("--hand-only", action="store_true",
                    help="single-side LinkerHand control while the arm remains disabled")
    ap.add_argument("--hardware", action="store_true")
    ap.add_argument("--config")
    ap.add_argument("--log-file", help="JSONL diagnostics; default robot_link/log/gateway_<time>.jsonl")
    args = ap.parse_args()
    if args.hand_only and (not args.with_hand or args.side == "both"):
        ap.error("--hand-only requires --with-hand and a single side")
    key = key_from_file(args.key_file)
    # Whole robot execution is exclusive; prevent two endpoints or an old local
    # teleop from silently sharing CAN. Old programs do not honor this lock, so
    # operator must also stop them (documented in deployment checklist).
    import fcntl
    import os
    lock_file = None
    if args.hardware:
        if not args.config:
            ap.error("--hardware requires --config")
        lock_file = open(f"/tmp/esrobo-robot-link-{os.getuid()}.lock", "w")
        fcntl.flock(lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
        from .backends import HardwareBackend, DualHardwareBackend
        backend = (DualHardwareBackend(args.config) if args.side == "both" else
                   HardwareBackend(args.config, args.side, args.with_hand, hand_only=args.hand_only))
    else:
        backend = (MockDualBackend() if args.side == "both" else
                   MockBackend(args.side, args.with_hand, args.hand_only))
    try:
        gateway = Gateway(backend, key, args.bind, args.port)
    except BaseException:
        backend.close()
        raise
    print(json.dumps(dict(listening=gateway.address, hardware=args.hardware,
                          contract=backend.contract), ensure_ascii=False), flush=True)
    print("Robot terminal: e zero+PICO prepare+enable, s/x stop, d all disable, dl/dr arm disable, dh hands disable, r recover, z return, q exit (Enter required)", flush=True)
    from pathlib import Path
    log_path = Path(args.log_file) if args.log_file else Path(__file__).resolve().parents[1] / "log" / f"gateway_{time.time_ns()}.jsonl"
    try:
        log_path.parent.mkdir(parents=True, exist_ok=True)
        gateway.journal = log_path.open("x", encoding="utf-8")
    except BaseException:
        gateway.socket.close()
        backend.close()
        raise
    print(f"Diagnostics: {log_path}", flush=True)

    def console():
        import sys
        for line in sys.stdin:
            try:
                gateway.request(line.strip())
            except (ValueError, RuntimeError) as exc:
                print(f"REFUSED: {exc}", flush=True)
        gateway.request("q")  # Losing the operator terminal never leaves following active.

    import signal
    signal.signal(signal.SIGTERM, lambda *_: gateway.request("q"))
    signal.signal(signal.SIGINT, lambda *_: gateway.request("q"))
    threading.Thread(target=console, name="robot-link-console", daemon=True).start()
    try:
        gateway.run()
    finally:
        gateway.journal.close()
        if lock_file:
            lock_file.close()


if __name__ == "__main__":
    main()
