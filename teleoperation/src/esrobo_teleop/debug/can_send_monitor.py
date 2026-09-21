"""Observe native socket sends without changing CAN loopback or SDK parsing."""
from collections import deque
import time
import threading


class CanSendMonitor:
    def __init__(self, comm, *, send_timeout_s=0.02):
        self.sink = None
        self.attempts = 0
        self.failures = 0
        self.last_error = None
        self.last_frame = None
        # Preserve startup/motion evidence when an emergency stop becomes the
        # last frame. Bounded to mode, seven-joint targets, and enable/disable.
        self._last_control_frames = {}
        self._recent_control_frames = deque(maxlen=64)
        self._lock = threading.RLock()
        self._comm = comm
        self._bus = None
        self._send_timeout_s = max(0.0, float(send_timeout_s))
        sdk_send = comm.send

        def checked(message, *args, **kwargs):
            with self._lock:
                if comm.send_bus is not self._bus:
                    raise RuntimeError("CAN send bus changed; reconnect to restore send monitoring")
                before = self.failures
                result = sdk_send(message, *args, **kwargs)
                # SDK may swallow a socket error and return None, like success.
                if self.failures != before:
                    raise RuntimeError("CAN socket send failed: " + str(self.last_error))
                return result

        self.bind_bus()
        comm.send = checked

    def bind_bus(self):
        """Rebind after SDK reconnect without stacking SDK send wrappers."""
        with self._lock:
            self._bind_bus()

    def _bind_bus(self):
        bus = self._comm.send_bus
        if self._bus is bus:
            return
        self._last_control_frames = {}
        self._recent_control_frames.clear()
        native_send = bus.send

        def native(message, *args, **kwargs):
            self.attempts += 1
            event = dict(event="native_can_send", timestamp=time.time(),
                         monotonic_s=time.monotonic(), can_id=int(message.arbitration_id),
                         data_hex=bytes(message.data).hex(), outcome="socket_send_returned")
            try:
                # python-can's SocketCAN default is timeout=0: it fails
                # immediately when the small kernel TX queue is momentarily
                # full.  Preserve an explicit caller timeout; otherwise wait
                # only long enough for a healthy 1-Mbit bus to drain.
                if "timeout" not in kwargs and len(args) == 0:
                    kwargs["timeout"] = self._send_timeout_s
                return native_send(message, *args, **kwargs)
            except Exception as exc:
                self.failures += 1
                self.last_error = repr(exc)
                event.update(outcome="socket_send_failed", error=self.last_error)
                raise
            finally:
                self.last_frame = event
                if event['can_id'] in (
                    0x151, 0x155, 0x156, 0x157, 0x170, 0x471,
                    0x181, 0x182, 0x183, 0x184, 0x185, 0x186, 0x187,
                ):
                    self._last_control_frames[hex(event['can_id'])] = event
                    self._recent_control_frames.append(dict(event))
                if self.sink is not None:
                    try:
                        self.sink(dict(event))
                    except Exception:
                        pass  # Diagnostic availability must not alter sending.

        bus.send = native
        self._bus = bus

    def snapshot(self):
        # Never wait for a hardware send to finish merely to collect diagnostics.
        with self._lock:
            return dict(attempts=self.attempts, failures=self.failures,
                        last_error=self.last_error, last_frame=self.last_frame,
                        last_control_frames={key: dict(event) for key, event in
                                             self._last_control_frames.copy().items()},
                        recent_control_frames=[dict(event) for event in
                                               list(self._recent_control_frames)],
                        note="Socket return is not controller execution acknowledgement")
