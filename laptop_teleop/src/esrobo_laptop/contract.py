"""Verify the robot fingerprint against local config/model without constructing drivers."""
import dataclasses
import hashlib
from pathlib import Path

from esrobo_link.protocol import ProtocolError, canonical
from esrobo_teleop.robot.linker_hand_driver import L10_PHYSICAL_JOINT_NAMES


def local_id(cfg, side, with_hand, hand_only=False):
    # Keep byte-for-byte compatible with HardwareBackend's protocol-v1 fingerprint.
    fingerprint = dict(
        robot=dataclasses.asdict(cfg.robot), hand=dataclasses.asdict(cfg.hand),
        ik={k: v for k, v in dataclasses.asdict(cfg.ik).items() if k != "urdf_path"},
        endpoint_speed=cfg.retarget.max_endpoint_translation_velocity_m_s,
        model_sha256=hashlib.sha256(Path(cfg.ik.urdf_path).read_bytes()).hexdigest(),
        side=side, with_hand=with_hand, hand_only=hand_only,
    )
    return hashlib.sha256(canonical(fingerprint)).hexdigest()


def verify_contract(contract, cfg, settings, *, mock=False):
    if (contract.get("side") != settings.side
            or contract.get("with_hand") is not settings.with_hand
            or bool(contract.get("hand_only", False)) is not settings.hand_only):
        raise ProtocolError("gateway side/hand mode does not match laptop settings")
    members = contract.get("sides", {}) if settings.side == "both" else {settings.side: contract}
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
