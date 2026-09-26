"""Portable laptop SDK. Never repeats a cached target automatically."""
import secrets
import socket
import time

from .protocol import MAX_PACKET, ProtocolError, key_from_file, pack, unpack, validate_hold, validate_target


class RobotClient:
    def __init__(self, host, port, key_file, *, expected_contract_id=None):
        self.key = key_from_file(key_file)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.connect((host, port))
        self.nonce = secrets.token_hex(16)
        self.latest = None
        self.received_at = 0.0
        self.seq = 0
        self.expected_contract_id = expected_contract_id

    def connect(self, timeout=3.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.socket.send(pack(dict(type="hello", client_nonce=self.nonce), self.key))
            try:
                state = self.receive(min(0.2, max(0.001, deadline - time.monotonic())))
                if self.expected_contract_id is not None and state["contract"]["id"] != self.expected_contract_id:
                    raise ProtocolError("robot contract differs from the commissioned laptop configuration")
                return state
            except TimeoutError:
                pass
        raise TimeoutError("no authenticated robot state")

    def receive(self, timeout=0.2):
        deadline = time.monotonic() + timeout
        newest = None
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                if newest is not None:
                    return newest
                raise TimeoutError("robot feedback timeout")
            self.socket.settimeout(remaining if newest is None else 0.0)
            try:
                raw = self.socket.recv(MAX_PACKET + 1)
            except BlockingIOError:
                return newest
            except socket.timeout:
                raise TimeoutError("robot feedback timeout") from None
            try:
                msg = unpack(raw, self.key)
                if msg.get("type") != "state" or msg.get("client_nonce") != self.nonce:
                    continue
                if self.latest and msg.get("session") != self.latest["session"]:
                    raise ProtocolError("robot session changed; create a new client")
                state_seq = msg.get("state_seq")
                if type(state_seq) is not int or (self.latest and state_seq <= self.latest["state_seq"]):
                    continue
                self.latest, self.received_at = msg, time.monotonic()
                newest = msg
            except ProtocolError:
                continue

    def _command(self, kind, **payload):
        if self.latest is None or time.monotonic() - self.received_at > self.latest["lease_ttl_s"]:
            raise TimeoutError("fresh robot state required before sending")
        self.seq += 1
        self.socket.send(pack(dict(type=kind, session=self.latest["session"], seq=self.seq,
                                   lease=self.latest["lease"], **payload), self.key))

    def send_target(self, arm_urdf_rad, hand_unit=None):
        """Call once per NEW solved input frame, never on stale PICO/glove data."""
        if self.latest is None:
            raise RuntimeError("connect first")
        c = self.latest["contract"]
        if c["side"] == "both":
            raise ProtocolError("dual endpoint requires send_dual_target")
        payload = dict(side=c["side"], contract_id=c["id"],
                       arm_urdf_rad=list(arm_urdf_rad),
                       hand_unit=None if hand_unit is None else list(hand_unit))
        validate_target(payload, c)
        self._command("target", **payload)

    def send_hold(self):
        """Renew a fresh arm-only lease and ask the robot to brake/hold."""
        if self.latest is None:
            raise RuntimeError("connect first")
        c = self.latest["contract"]
        payload = dict(side=c["side"], contract_id=c["id"])
        validate_hold(payload, c)
        self._command("hold", **payload)

    def send_hand_target(self, hand_unit):
        """Send one single-side hand target without any arm command field."""
        if self.latest is None:
            raise RuntimeError("connect first")
        c = self.latest["contract"]
        if c["side"] == "both" or not c.get("hand_only", False):
            raise ProtocolError("send_hand_target requires a single-side hand-only endpoint")
        payload = dict(side=c["side"], contract_id=c["id"], hand_unit=list(hand_unit))
        validate_target(payload, c)
        self._command("target", **payload)

    def send_dual_target(self, *, left_arm_urdf_rad, right_arm_urdf_rad,
                         left_hand_unit, right_hand_unit):
        """One frame for both arms and hands; never send a partial side update."""
        if self.latest is None:
            raise RuntimeError("connect first")
        c = self.latest["contract"]
        if c["side"] != "both":
            raise ProtocolError("send_dual_target requires a dual endpoint")
        payload = dict(side="both", contract_id=c["id"], targets={
            "left": dict(arm_urdf_rad=list(left_arm_urdf_rad), hand_unit=list(left_hand_unit)),
            "right": dict(arm_urdf_rad=list(right_arm_urdf_rad), hand_unit=list(right_hand_unit))})
        validate_target(payload, c)
        self._command("target", **payload)

    def stop(self):
        self._command("stop")

    def close(self):
        # Even when this best-effort packet is lost, robot lease expiry still applies.
        try:
            if self.latest:
                self.stop()
        except (OSError, TimeoutError):
            pass
        self.socket.close()

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *_):
        self.close()
