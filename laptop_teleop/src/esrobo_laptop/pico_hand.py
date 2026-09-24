"""PICO/OpenXR 26-joint optical hands -> ESROBO's ten active joints.

Body's 24 joints cannot describe fingers. The Hand stream is sampled separately,
with a per-hand SDK reception sequence (never the generic head/XR timestamp).
"""
import numpy as np

from esrobo_teleop.robot.linker_hand_driver import ACTIVE_HAND_JOINTS, ACTIVE_JOINT_LIMITS_RAD


def unit(vector):
    length = np.linalg.norm(vector)
    if length < 1e-5:
        raise ValueError("degenerate PICO hand bone")
    return vector / length


def angle(a, b):
    return float(np.arccos(np.clip(unit(a) @ unit(b), -1., 1.)))


def retarget_hand(poses):
    """Geometric curl/spread, invariant to world pose and left/right reflection.

    OpenXR order: palm, wrist, thumb 2..5, index 6..10, middle
    11..15, ring 16..20, little 21..25. Angles are bounded by the
    commissioned robot hand limits; this is an initial geometric mapping.
    """
    poses = np.asarray(poses, dtype=float)
    if poses.shape != (26, 7) or not np.all(np.isfinite(poses)):
        raise ValueError("PICO hand requires 26 finite xyz+xyzw poses")
    if np.any(np.linalg.norm(poses[:, 3:], axis=1) < .5):
        raise ValueError("invalid PICO hand orientation")
    p = poses[:, :3]
    forward = unit(p[12] - p[1])
    lateral = unit(p[7] - p[22])
    normal = unit(np.cross(lateral, forward))
    lateral = unit(np.cross(forward, normal))

    def curl(indices):
        bones = np.diff(p[indices], axis=0)
        return float(np.clip(np.mean([angle(a, b) for a, b in zip(bones[:-1], bones[1:])]) / (np.pi/2), 0, 1))

    def spread(mcp, pip, maximum):
        direction = unit(p[pip] - p[mcp])
        return float(np.clip(abs(np.arctan2(direction @ lateral, direction @ forward)) / maximum, 0, 1))

    thumb = unit(p[3] - p[2])
    thumb_plane = thumb - (thumb @ normal) * normal
    opposition = np.clip(1 - angle(thumb_plane, forward) / (np.pi/3), 0, 1)
    fractions = np.array([
        np.clip(abs(thumb @ normal), 0, 1), opposition, curl([2, 3, 4, 5]),
        spread(7, 8, .19), curl([6, 7, 8, 9, 10]),
        curl([11, 12, 13, 14, 15]),
        # PICO tracks these fingers separately; keep their targets independent.
        spread(17, 18, .20), curl([16, 17, 18, 19, 20]),
        spread(22, 23, .30), curl([21, 22, 23, 24, 25]),
    ])
    limits = np.array([ACTIVE_JOINT_LIMITS_RAD[n][1] for n in ACTIVE_HAND_JOINTS])
    return (fractions * limits).tolist()


class PicoHands:
    def __init__(self):
        self.last = {}
        self.signatures = {}
        self.status = {}

    def poll(self, sdk):
        """Return only new, valid sides. A missing side never gets mirrored."""
        result = {}
        for side in ("left", "right"):
            try:
                getter = getattr(sdk, f"get_{side}_hand_sample", None)
                if getter is None:
                    raise ValueError("SDK needs per-hand sample extension; run install_pico.sh")
                sample = getter()
                sequence = int(sample["sequence"])
                if not sample["active"] or sequence <= 0:
                    raise ValueError("PICO hand tracking inactive")
                age = float(sample["age_s"])
                if not np.isfinite(age) or not 0 <= age <= .1:
                    raise ValueError("PICO Hand stream stale")
                flags = np.asarray(sample["flags"], dtype=np.uint64)
                if flags.shape != (26,) or np.any((flags & 3) != 3):
                    raise ValueError("PICO hand joints invalid/occluded")
                if sequence <= self.last.get(side, 0):
                    self.status[side] = "waiting for new Hand sample"
                    continue
                self.last[side] = sequence
                timestamp = int(sample["timestamp_ns"])
                signature = (("timestamp", timestamp) if timestamp > 0 else
                             ("poses", tuple(np.asarray(sample["poses"]).ravel())))
                if signature == self.signatures.get(side):
                    self.status[side] = "Hand timestamp/pose frozen"
                    continue
                self.signatures[side] = signature
                angles = retarget_hand(sample["poses"])
                result[side] = dict(joints=angles, sequence=sequence, age_s=age)
                self.status[side] = "tracking"
            except (ValueError, TypeError, KeyError, OverflowError) as exc:
                self.status[side] = str(exc)
        return result
