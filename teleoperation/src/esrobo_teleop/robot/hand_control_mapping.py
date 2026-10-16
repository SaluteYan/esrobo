"""Explicit command calibration, independent of feedback geometry calibration."""
import numpy as np


def enabled_for(cfg, side):
    override = getattr(cfg, f"{side}_enabled_physical_joints", None)
    return list(cfg.enabled_physical_joints if override is None else override)


def calibrated_units(angles, cfg):
    from .linker_hand_driver import ACTIVE_HAND_JOINTS, L10_PHYSICAL_JOINT_NAMES
    result = []
    for name in L10_PHYSICAL_JOINT_NAMES:
        item = cfg.left_control_calibration[name]
        q0, q1 = item['rad']; u0, u1 = item['raw']
        if not np.isfinite([q0, q1, u0, u1]).all() or q1 <= q0 or not 0 <= min(u0, u1) < max(u0, u1) <= 255:
            raise ValueError('invalid left control calibration: '+name)
        q = angles[ACTIVE_HAND_JOINTS.index(name)]
        allowed = item.get('allowed_rad', (q0, q1))
        if not q0 <= allowed[0] < allowed[1] <= q1:
            raise ValueError('invalid allowed command angle: '+name)
        bounds = sorted(u0+(np.asarray(allowed)-q0)/(q1-q0)*(u1-u0))
        value = u0+(np.clip(q, *allowed)-q0)/(q1-q0)*(u1-u0)
        # Quantization must remain inside the angular envelope, including at a stop.
        result.append(np.clip(np.rint(value), np.ceil(bounds[0]), np.floor(bounds[1])))
    return np.rint(result).astype(int)
