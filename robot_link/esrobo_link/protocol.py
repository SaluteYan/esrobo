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
        raise ProtocolError("packet exceeds 8192 bytes; images use a separate channel")
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
    return {"arm_urdf_rad": arm, "hand_unit": hand}


def validate_hold(msg, contract):
    """A fresh single arm request to brake/hold without a position target."""
    if (contract["side"] == "both" or contract.get("with_hand")
            or contract.get("hand_only")):
        raise ProtocolError("hold is commissioned only for one arm without a hand")
    if msg.get("contract_id") != contract["id"] or msg.get("side") != contract["side"]:
        raise ProtocolError("side/configuration contract mismatch")
    if any(name in msg for name in ("arm_urdf_rad", "hand_unit", "targets")):
        raise ProtocolError("hold cannot contain a position or hand command")
    return {"hold": True}
