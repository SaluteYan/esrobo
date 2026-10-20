"""Bounded JSON/HMAC UDP packets; joint units are part of the wire schema."""
import hashlib
import hmac
import json
import math
from pathlib import Path

VERSION = 1
MAX_PACKET = 8192


class ProtocolError(ValueError):
    pass


class PacketTooLarge(ProtocolError):
    """The encoded packet exceeds the transport bound, not invalid telemetry."""


class InputHoldTransition(RuntimeError):
    """Abort an in-flight target because the gateway switched to latched hold."""


class HoldReturnTransition(RuntimeError):
    """Yield an in-flight hold to an accepted robot-owned checked return."""


def key_from_file(path):
    key = Path(path).read_bytes().strip()
    if len(key) < 32:
        raise ProtocolError("key file must contain at least 32 random bytes/characters")
    return key


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()


def pack(message, key):
    body = canonical(dict(message, v=VERSION))
    packet = canonical({"body": body.decode(), "mac": hmac.new(key, body, hashlib.sha256).hexdigest()})
    if len(packet) > MAX_PACKET:
        raise PacketTooLarge("packet exceeds 8192 bytes; images use a separate channel")
    return packet


def unpack(packet, key):
    try:
        if len(packet) > MAX_PACKET:
            raise ProtocolError("oversized packet")
        envelope = json.loads(packet)
        if set(envelope) != {"body", "mac"} or not isinstance(envelope["body"], str):
            raise ProtocolError("invalid envelope")
        body = envelope["body"].encode()
        expected = hmac.new(key, body, hashlib.sha256).hexdigest()
        if not isinstance(envelope["mac"], str) or not hmac.compare_digest(expected, envelope["mac"]):
            raise ProtocolError("authentication failed")
        msg = json.loads(body)
        if not isinstance(msg, dict) or type(msg.get("v")) is not int or msg["v"] != VERSION:
            raise ProtocolError("unsupported protocol version")
        # Also rejects JSON NaN/Infinity, including nested fields.
        canonical(msg)
        return msg
    except (TypeError, ValueError, KeyError, UnicodeError, RecursionError) as exc:
        raise ProtocolError(str(exc)) from exc


def vector(value, size, name):
    if not isinstance(value, list) or len(value) != size:
        raise ProtocolError(f"{name} requires {size} numbers")
    try:
        invalid = any(type(x) not in (int, float) or not math.isfinite(x) for x in value)
    except OverflowError:
        invalid = True
    if invalid:
        raise ProtocolError(f"{name} requires finite numbers, not booleans")
    return list(value)


def validate_target(msg, contract):
    if msg.get("contract_id") != contract["id"] or msg.get("side") != contract["side"]:
        raise ProtocolError("side/configuration contract mismatch")
    if contract["side"] == "both":
        targets = msg.get("targets")
        if not isinstance(targets, dict) or set(targets) != {"left", "right"}:
            raise ProtocolError("dual target requires both left and right in the same packet")
        checked = {}
        for side in ("left", "right"):
            target = targets[side]
            if not isinstance(target, dict) or set(target) != {"arm_urdf_rad", "hand_unit"}:
                raise ProtocolError(f"{side} requires arm_urdf_rad and hand_unit")
            member = contract["sides"][side]
            checked[side] = validate_target(dict(target, side=side, contract_id=member["id"]), member)
        return {"targets": checked}
    if contract.get("hand_only", False):
        if not contract.get("with_hand") or "arm_urdf_rad" in msg:
            raise ProtocolError("hand-only target cannot contain an arm command")
        hand = vector(msg.get("hand_unit"), 10, "hand_unit")
        if any(type(x) is not int or not 0 <= x <= 255 for x in hand):
            raise ProtocolError("hand_unit requires ten integers in [0,255]")
        return {"arm_urdf_rad": None, "hand_unit": hand}
    arm = vector(msg.get("arm_urdf_rad"), 7, "arm_urdf_rad")
    if any(q < lo or q > hi for q, lo, hi in zip(arm, contract["lower_rad"], contract["upper_rad"])):
        raise ProtocolError("target outside commissioned arm limits")
    hand = msg.get("hand_unit")
    if contract["with_hand"]:
        hand = vector(hand, 10, "hand_unit")
        if any(type(x) is not int or not 0 <= x <= 255 for x in hand):
            raise ProtocolError("hand_unit requires ten integers in [0,255]")
    elif hand is not None:
        raise ProtocolError("hand not enabled on this endpoint")
    result = {"arm_urdf_rad": arm, "hand_unit": hand}
    if contract.get("entry_protocol") in (1, 2, 3, 4, 5, 6, 7):
        entry = msg.get("entry")
        if not isinstance(entry, dict) or entry.get("phase") not in ("idle", "candidate", "ready", "active"):
            raise ProtocolError("safe handover metadata required; update laptop and gateway together")
        token = entry.get("token", "")
        if not isinstance(token, str) or len(token) > 64:
            raise ProtocolError("invalid handover token")
        source = (vector(entry.get("source"), 9, "entry source")
                  if entry["phase"] in ("candidate", "ready") else None)
        reference_source = (vector(entry.get("reference_source"), 9, "calibrated entry source")
                            if contract.get("entry_protocol") in (2, 3, 4, 5, 6, 7)
                            and entry["phase"] in ("candidate", "ready") else None)
        result["entry"] = dict(phase=entry["phase"], token=token, source=source,
                               reference_source=reference_source)
    return result


def validate_recovery(msg, contract):
    value = msg.get('recovery')
    if value is None:
        return None
    if (contract.get('input_recovery_protocol') != 1
            or contract.get('input_recovery_enabled') is not True):
        raise ProtocolError('input recovery requires a commissioned protocol 1 endpoint')
    if (not isinstance(value, dict) or set(value) != {'phase', 'epoch'}
            or value.get('phase') not in ('reacquiring', 'complete')
            or not isinstance(value.get('epoch'), str) or len(value['epoch']) != 16
            or any(c not in '0123456789abcdef' for c in value['epoch'])):
        raise ProtocolError('invalid input recovery metadata')
    return dict(value)


def validate_entry_keepalive(msg, contract):
    """Communication liveness for one frozen plan; never a sensor sample."""
    if (contract.get("entry_keepalive_protocol") != 1 or contract.get("side") == "both"
            or contract.get("hand_only")):
        raise ProtocolError("entry keepalive protocol 1 required")
    if msg.get("contract_id") != contract["id"] or msg.get("side") != contract["side"]:
        raise ProtocolError("side/configuration contract mismatch")
    allowed = {"v", "type", "session", "seq", "lease", "side", "contract_id", "plan_token"}
    if set(msg) - allowed:
        raise ProtocolError("entry keepalive cannot contain targets or sensor metadata")
    token = msg.get("plan_token")
    if not isinstance(token, str) or len(token) != 32:
        raise ProtocolError("invalid entry plan token")
    return token


def validate_hold(msg, contract):
    """A fresh request to brake/hold; it never carries a position target."""
    if contract.get("input_hold_protocol") not in (1, 2) and (
            contract["side"] == "both" or contract.get("with_hand")
            or contract.get("hand_only")):
        raise ProtocolError("hold is commissioned only for one arm without a hand")
    if msg.get("contract_id") != contract["id"] or msg.get("side") != contract["side"]:
        raise ProtocolError("side/configuration contract mismatch")
    if any(name in msg for name in ("arm_urdf_rad", "hand_unit", "targets")):
        raise ProtocolError("hold cannot contain a position or hand command")
    reason = msg.get("reason", "input")
    if reason not in ("input", "position"):
        raise ProtocolError("hold reason must be input or position")
    if reason == "position":
        if contract.get("input_hold_protocol") != 2:
            raise ProtocolError("position hold requires input hold protocol 2")
        if contract["side"] == "both" or contract.get("hand_only"):
            raise ProtocolError("position hold requires a single arm endpoint")
        return {"hold": True, "hold_reason": "position"}
    return {"hold": True}
