"""Verify the robot fingerprint against local config/model without constructing drivers."""
import dataclasses
import hashlib
from pathlib import Path

from esrobo_link.protocol import ProtocolError, canonical
from esrobo_teleop.robot.linker_hand_driver import L10_PHYSICAL_JOINT_NAMES
from esrobo_teleop.config import retarget_filter_fingerprint

REQUIRED_ENTRY_PROTOCOL = 7
REQUIRED_ENTRY_KEEPALIVE_PROTOCOL = 1
REQUIRED_INPUT_HOLD_PROTOCOL = 2


def local_id(cfg, side, with_hand, hand_only=False):
    # Keep byte-for-byte compatible with HardwareBackend's protocol-v1 fingerprint.
    fingerprint = dict(
        robot=dataclasses.asdict(cfg.robot), hand=dataclasses.asdict(cfg.hand),
        ik={k: v for k, v in dataclasses.asdict(cfg.ik).items() if k != "urdf_path"},
        endpoint_speed=cfg.retarget.max_endpoint_translation_velocity_m_s,
        retarget_filter=retarget_filter_fingerprint(cfg.retarget),
        model_sha256=hashlib.sha256(Path(cfg.ik.urdf_path).read_bytes()).hexdigest(),
        side=side, with_hand=with_hand, hand_only=hand_only,
    )
    return hashlib.sha256(canonical(fingerprint)).hexdigest()


def verify_contract(contract, cfg, settings, *, mock=False):
    if contract.get('input_recovery_protocol') != 1:
        raise ProtocolError('gateway lacks input recovery protocol 1; update and restart both endpoints')
    if contract.get("input_hold_protocol") != REQUIRED_INPUT_HOLD_PROTOCOL:
        raise ProtocolError("gateway lacks input hold protocol 2 (separate position/input pauses); "
                            "update and restart the robot gateway")
    if (contract.get("side") != settings.side
            or contract.get("with_hand") is not settings.with_hand
            or bool(contract.get("hand_only", False)) is not settings.hand_only):
        raise ProtocolError("gateway side/hand mode does not match laptop settings")
    if (not mock and settings.side != "both" and not settings.hand_only
            and contract.get("entry_protocol") != REQUIRED_ENTRY_PROTOCOL):
        raise ProtocolError(
            f"gateway entry_protocol={contract.get('entry_protocol')!r}; "
            f"laptop requires {REQUIRED_ENTRY_PROTOCOL}. Synchronize robot_link and teleoperation "
            "on the robot, then restart the gateway")
    members = contract.get("sides", {}) if settings.side == "both" else {settings.side: contract}
    if (not mock and settings.side != "both" and not settings.hand_only
            and contract.get("entry_keepalive_protocol") != REQUIRED_ENTRY_KEEPALIVE_PROTOCOL):
        raise ProtocolError("gateway lacks entry_keepalive_protocol=1; synchronize robot_link and restart gateway")
    if set(members) != set(settings.sides):
        raise ProtocolError("incomplete gateway contract")
    for side, member in members.items():
        expected = (f"mock-v1-{side}-{settings.with_hand}-{settings.hand_only}" if mock else
                    local_id(cfg, side, settings.with_hand, settings.hand_only))
        if member.get("id") != expected:
            raise ProtocolError(f"{side} config/URDF fingerprint mismatch; synchronize commissioned configuration")
        if (member.get("side") != side or member.get("with_hand") is not settings.with_hand
                or bool(member.get("hand_only", False)) is not settings.hand_only):
            raise ProtocolError("member side/hand mismatch")
        if not mock:
            if (member.get("arm_order") != list(getattr(cfg.ik, f"{side}_arm_joints"))
                    or member.get("hand_order") != L10_PHYSICAL_JOINT_NAMES
                    or member.get("arm_unit") != "URDF radians"
                    or member.get("hand_unit") != "L10 physical 0..255"):
                raise ProtocolError("joint order/unit mismatch")
    if settings.side == "both" and contract.get("id") != hashlib.sha256(canonical(members)).hexdigest():
        raise ProtocolError("dual contract digest mismatch")
    return members
