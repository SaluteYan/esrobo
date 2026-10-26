"""PICO/OpenXR 26-joint optical hands -> ESROBO's ten active joints.

Body's 24 joints cannot describe fingers. The Hand stream is sampled separately,
with a per-hand SDK reception sequence (never the generic head/XR timestamp).
"""
import time
import numpy as np

from esrobo_teleop.robot.linker_hand_driver import ACTIVE_HAND_JOINTS, ACTIVE_JOINT_LIMITS_RAD


def unit(vector):
    length = float(vector @ vector)**.5
    if length < 1e-5:
        raise ValueError("degenerate PICO hand bone")
    return vector / length


def angle(a, b):
    return float(np.arccos(np.clip(unit(a) @ unit(b), -1., 1.)))


def palm_basis(poses):
    """Anatomical columns: normal, little-to-index, wrist-to-middle MCP.

    Columns are vectors in the SDK tracking space, not quaternion axes. The
    left hand's normal is mirrored, as is the URDF's index/little direction.
    """
    poses = np.asarray(poses, dtype=float)
    if poses.shape != (26, 7) or not np.all(np.isfinite(poses)):
        raise ValueError("PICO hand requires 26 finite xyz+xyzw poses")
    return _palm_basis_validated(poses)


def _palm_basis_validated(poses):
    # Three-dimensional cross products avoid generic broadcasting/axis setup
    # on the acquisition owner. Same anatomical construction as palm_basis.
    def cross(a, b):
        return np.array([a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]])
    forward = unit(poses[12, :3] - poses[1, :3])
    lateral = unit(poses[7, :3] - poses[22, :3])
    normal_raw = cross(lateral, forward)
    if float(normal_raw @ normal_raw) < .25:
        raise ValueError("degenerate PICO palm plane")
    normal = unit(normal_raw)
    return np.column_stack((normal, cross(forward, normal), forward))


def palm_sample(sample, now=None):
    """Cheap independent palm observation; never renew the SDK source clock."""
    now = time.monotonic() if now is None else now
    stamp = int(sample['received_monotonic_ns']) / 1e9
    seq = sample['sequence']
    flags = np.asarray(sample['flags'], dtype=np.uint64)
    poses = np.asarray(sample['poses'], dtype=float)
    if (not sample['active'] or type(seq) is not int or seq <= 0 or
            not 0 <= now-stamp <= .1 or flags.shape != (26,) or
            np.any((flags & 3) != 3) or poses.shape != (26, 7) or
            not np.isfinite(poses).all() or np.any(np.sum(poses[:, 3:]**2, axis=1) < .25)):
        raise ValueError('invalid/stale PICO palm observation')
    return dict(sequence=seq, sample_monotonic=stamp, palm_basis=_palm_basis_validated(poses).tolist())


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
    normal, lateral, forward = palm_basis(poses).T

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
    def __init__(self, retargeter=None):
        self.retargeter = retargeter
        self.last = {}
        self.signatures = {}
        self.status = {}

    def map_sample(self, side, sample, *, observed=None):
        """Map an independent snapshot. SDK access stays with the owner thread."""
        now = time.monotonic()
        observed = now if observed is None else observed
        sequence = int(sample["sequence"])
        if not sample["active"] or sequence <= 0:
            raise ValueError("PICO hand tracking inactive")
        received_ns = int(sample.get("received_monotonic_ns", 0))
        received = received_ns / 1e9 if received_ns else observed-float(sample["age_s"])
        age = now-received
        if not np.isfinite(age) or not 0 <= age <= .1:
            raise ValueError("PICO Hand stream stale")
        flags = np.asarray(sample["flags"], dtype=np.uint64)
        if flags.shape != (26,) or np.any((flags & 3) != 3):
            raise ValueError("PICO hand joints invalid/occluded")
        if sequence <= self.last.get(side, 0):
            self.status[side] = "waiting for new Hand sample"
            return None
        self.last[side] = sequence
        timestamp = int(sample["timestamp_ns"])
        if sample.get("freshness_source") == "component-packet":
            # The SDK commits only complete, advancing same-packet
            # component frames, including valid stationary poses.
            signature = ("component-packet", int(sample["packet_timestamp_ns"]))
        else:
            signature = (("timestamp", timestamp) if timestamp > 0 else
                         ("poses", tuple(np.asarray(sample["poses"]).ravel())))
        if signature == self.signatures.get(side):
            self.status[side] = "Hand timestamp/pose frozen"
            return None
        self.signatures[side] = signature
        poses = np.asarray(sample['poses'], dtype=float)
        if poses.shape != (26, 7) or not np.isfinite(poses).all() or np.any(np.linalg.norm(poses[:, 3:], axis=1) < .5):
            raise ValueError('invalid PICO hand poses/orientations')
        fitted = self.retargeter(side, sample['poses']) if self.retargeter else None
        angles = fitted['joints'] if fitted is not None else retarget_hand(sample["poses"])
        result = dict(joints=angles, sequence=sequence, age_s=age,
                            sample_monotonic=received,
                            palm_basis=palm_basis(sample["poses"]).tolist())
        if fitted is not None:
            result.update(fitted)
        self.status[side] = "tracking"
        # Mapping can take time: never send a result that expired during work.
        age = time.monotonic() - received
        if not 0 <= age <= .1:
            raise ValueError("PICO Hand stream stale after mapping")
        result["age_s"] = age
        return result

    def poll(self, sdk):
        """Compatibility getter path, retaining real component freshness."""
        result = {}
        for side in ("left", "right"):
            try:
                getter = getattr(sdk, f"get_{side}_hand_sample", None)
                if getter is None:
                    raise ValueError("SDK needs per-hand sample extension; run install_pico.sh")
                observed = time.monotonic()
                mapped = self.map_sample(side, getter(), observed=observed)
                if mapped is not None:
                    result[side] = mapped
            except (ValueError, TypeError, KeyError, OverflowError) as exc:
                self.status[side] = str(exc)
        return result
