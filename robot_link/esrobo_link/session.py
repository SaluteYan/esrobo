"""Pure session gate, independent of sockets and hardware (robot clock only)."""
from collections import OrderedDict
import secrets

from .protocol import ProtocolError, validate_hold, validate_target, validate_entry_keepalive, validate_recovery


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
        self.target_deadline = 0.0
        self.last_entry_keepalive_at = None
        self.last_seen = 0.0
        self.hold_started_at = None
        self.position_hold_started_at = None
        self.recovery_started_at = None
        self.recovery_epoch = None
        self.completed_recovery_epochs = set()

    def hello(self, msg, peer, now, busy):
        nonce = msg.get("client_nonce")
        if not isinstance(nonce, str) or len(nonce) != 32:
            raise ProtocolError("invalid client nonce")
        if (self.contract.get('input_recovery_enabled') is True
                and msg.get('input_recovery_protocol') != 1):
            raise ProtocolError('client lacks input recovery protocol 1; update both endpoints')
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
        self.target_deadline = 0.0
        self.last_entry_keepalive_at = None
        self.last_seen = now
        self.hold_started_at = None
        self.position_hold_started_at = None
        self.recovery_started_at = None
        self.recovery_epoch = None
        self.completed_recovery_epochs.clear()

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

    def accept(self, msg, peer, now, *, entry_plan_token=None):
        if self.session is None or peer != self.peer or msg.get("session") != self.session:
            raise ProtocolError("wrong session or peer")
        seq = msg.get("seq")
        if type(seq) is not int or not 0 <= seq < 2**53 or seq <= self.sequence:
            raise ProtocolError("old, duplicate, or invalid sequence")
        issued = self.leases.get(msg.get("lease"))
        if issued is None or not 0 <= now - issued < self.max_age:
            raise ProtocolError("expired robot-issued lease")
        kind = msg.get("type")
        if kind == "entry_keepalive":
            token = validate_entry_keepalive(msg, self.contract)
            if not entry_plan_token or token != entry_plan_token:
                raise ProtocolError("entry keepalive requires the current plan and entry phase")
            if not self.fresh(now):
                raise ProtocolError("entry keepalive cannot revive an expired plan")
            self.sequence, self.last_seen = seq, now
            self.deadline = issued + self.max_age
            self.last_entry_keepalive_at = now
            return kind  # Do not modify target, ready validity, or input-hold state.
        if kind == "target":
            target = validate_target(msg, self.contract)
            recovery = validate_recovery(msg, self.contract)
            if self.recovery_epoch is not None:
                if recovery is None or recovery['epoch'] != self.recovery_epoch:
                    raise ProtocolError('active reacquisition requires the original epoch')
                if now - self.recovery_started_at >= 3.:
                    raise ProtocolError('reacquisition deadline expired')
            if recovery is not None:
                epoch = recovery['epoch']
                if recovery['phase'] == 'reacquiring':
                    if self.recovery_epoch is None:
                        if epoch in self.completed_recovery_epochs or self.hold_started_at is None:
                            raise ProtocolError('new reacquisition requires an input hold and unused epoch')
                        self.recovery_epoch, self.recovery_started_at = epoch, now
                elif epoch != self.recovery_epoch:
                    if epoch not in self.completed_recovery_epochs or self.recovery_epoch is not None:
                        raise ProtocolError('completion requires current reacquisition epoch')
                else:
                    if len(self.completed_recovery_epochs) >= 128:
                        raise ProtocolError('recovery epoch budget exhausted; restart session')
                    self.completed_recovery_epochs.add(epoch)
                    self.recovery_epoch = self.recovery_started_at = None
                target['recovery'] = recovery
            self.hold_started_at = None
            self.position_hold_started_at = None
        elif kind == "hold":
            target = validate_hold(msg, self.contract)
            if self.target is None or not self.fresh(now):
                raise ProtocolError("hold requires a preceding fresh target")
            if target.get("hold_reason") == "position":
                # The client has consumed a complete fresh input frame and
                # deliberately pauses its unreachable position target. The
                # ordinary robot-issued lease still expires if packets stop.
                self.hold_started_at = None
                if self.position_hold_started_at is None:
                    self.position_hold_started_at = now
            else:
                self.position_hold_started_at = None
                if self.hold_started_at is None:
                    self.hold_started_at = now
        elif kind in ("stop", "close", "input_fault"):
            if kind != 'stop' and self.contract.get('input_recovery_protocol') != 1:
                raise ProtocolError('input recovery protocol 1 required')
            target = None
            self.hold_started_at = None
            self.position_hold_started_at = None
        else:
            raise ProtocolError("only target, hold and stop accepted; enable/return are local")
        self.sequence, self.last_seen = seq, now
        self.target = target
        # A packet delayed in flight gets only its remaining lease, NOT a new TTL.
        self.deadline = issued + self.max_age if target is not None else 0.0
        self.target_deadline = self.deadline
        return kind

    def fresh(self, now):
        return self.target is not None and now < self.deadline
