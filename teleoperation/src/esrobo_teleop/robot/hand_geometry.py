"""Read-only, independently calibrated hand feedback for collision checking."""
from dataclasses import dataclass
import time
import numpy as np


@dataclass(frozen=True)
class HandGeometryState:
    stamp: float | None
    intervals: dict
    calibrated: tuple
    valid: bool
    reason: str = ""


def geometry_state(raw, stamp, side, calibration, names, horizon, timeout, now=None):
    now = time.monotonic() if now is None else now
    if (stamp is None or not np.isfinite(stamp) or not 0 <= now-stamp <= timeout
            or raw is None or len(raw) != len(names) or not np.all(np.isfinite(raw))):
        return HandGeometryState(stamp, {}, (), False, "missing/stale hand feedback")
    intervals = {}
    try:
        if not np.isfinite(horizon) or horizon < 0:
            raise ValueError("invalid hand motion horizon")
        for i, name in enumerate(names):
            item = calibration.get(side, {}).get(name)
            if item is None:
                continue  # Collision guard retains full URDF range.
            # Each record is an explicit calibration, with bounded error and
            # physical speed. No control-command endpoints are substituted.
            raw0, raw1 = item['raw']
            rad0, rad1 = item['rad']
            error, speed = item['error_rad'], item['max_velocity_rad_s']
            if (item.get('verified') is not True
                    or not np.all(np.isfinite([raw0, raw1, rad0, rad1, error, speed]))
                    or raw0 == raw1 or error <= 0 or speed <= 0
                    or not min(raw0, raw1) <= raw[i] <= max(raw0, raw1)):
                raise ValueError(f"invalid/out-of-range hand calibration: {name}")
            q = rad0 + (raw[i]-raw0)*(rad1-rad0)/(raw1-raw0)
            radius = error + speed * (now-stamp+horizon)
            intervals[side+'_'+name] = (float(q-radius), float(q+radius))
    except (KeyError, TypeError, ValueError) as exc:
        return HandGeometryState(stamp, {}, (), False, str(exc))
    return HandGeometryState(float(stamp), intervals, tuple(intervals), True)
