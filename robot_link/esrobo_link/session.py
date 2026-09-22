"""Pure session gate, independent of sockets and hardware (robot clock only)."""
from collections import OrderedDict
import secrets

from .protocol import ProtocolError, validate_target


class SessionGate:
    def __init__(self, contract, max_age=0.2):
        self.contract = contract
        self.max_age = max_age
        self.peer = None
        self.session = None
        self.client_nonce = None
        self.sequence = -1
        self.state_sequence = 0
        self.leases = OrderedDict()
        self.target = None
        self.deadline = 0.0
        self.last_seen = 0.0

    def hello(self, msg, peer, now, busy):
        nonce = msg.get("client_nonce")
        if not isinstance(nonce, str) or len(nonce) != 32:
            raise ProtocolError("invalid client nonce")
        if self.peer == peer and self.client_nonce == nonce:
            return  # idempotent; hello never renews target validity
        if busy or (self.peer is not None and now - self.last_seen < 1.0):
            raise ProtocolError("endpoint already owned; stop locally before changing client")
        self.peer, self.client_nonce = peer, nonce
        self.session = secrets.token_hex(16)
        self.sequence = -1
        self.state_sequence = 0
        self.leases.clear()
        self.target = None
        self.deadline = 0.0
        self.last_seen = now

    def state(self, now, status):
        lease = secrets.token_hex(12)
        self.leases[lease] = now
        while len(self.leases) > 32:
            self.leases.popitem(last=False)
        self.state_sequence += 1
        return dict(type="state", session=self.session, client_nonce=self.client_nonce,
                    state_seq=self.state_sequence, lease=lease,
                    lease_ttl_s=self.max_age, accepted_seq=self.sequence,
                    target_remaining_s=max(0.0, self.deadline - now),
                    contract=self.contract, **status)

    def accept(self, msg, peer, now):
        if self.session is None or peer != self.peer or msg.get("session") != self.session:
            raise ProtocolError("wrong session or peer")
        seq = msg.get("seq")
        if type(seq) is not int or not 0 <= seq < 2**53 or seq <= self.sequence:
            raise ProtocolError("old, duplicate, or invalid sequence")
        issued = self.leases.get(msg.get("lease"))
        if issued is None or not 0 <= now - issued < self.max_age:
            raise ProtocolError("expired robot-issued lease")
        kind = msg.get("type")
        if kind == "target":
            target = validate_target(msg, self.contract)
        elif kind == "stop":
            target = None
        else:
            raise ProtocolError("only target and stop accepted; enable/return are local")
        self.sequence, self.last_seen = seq, now
        self.target = target
        # A packet delayed in flight gets only its remaining lease, NOT a new TTL.
        self.deadline = issued + self.max_age if target is not None else 0.0
        return kind

    def fresh(self, now):
        return self.target is not None and now < self.deadline
