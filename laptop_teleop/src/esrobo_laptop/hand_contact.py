"""Source-time contact evidence from optical landmarks, not a touch sensor.

The returned state is a candidate. Only a successfully published hand result
may commit it. No FK, optimizer, hardware access or unbounded history.
"""
from dataclasses import dataclass
import numpy as np


POLICY = dict(algorithm="relative_pair_v1",
              close_m=.008, isolated_enter_m=.016, isolated_exit_m=.020,
              multi_enter_m=.012, ordinary_exit_m=.012,
              separation_m=.005, separation_ratio=.6,
              close_confirm_s=.060, relative_confirm_s=.100, exit_confirm_s=.060,
              departure_m=.002, approach_m=.002, own_approach_width=.025,
              history_s=.300, minimum_history_s=.080, history_capacity=32,
              projection_s=.080, clock_gap_s=.100)


@dataclass(frozen=True)
class State:
    stamp: float
    active: np.ndarray
    projection: np.ndarray
    entering: np.ndarray
    leaving: np.ndarray
    minimum: np.ndarray
    history: tuple


class HandContact:
    def __init__(self):
        self.state = None

    def propose(self, distances, points, width, stamp):
        d = np.asarray(distances, dtype=float)
        if (d.shape != (4,) or not np.isfinite(d).all() or np.any(d < 0)
                or not np.isfinite(stamp) or not np.isfinite(width) or width <= 0):
            raise ValueError("invalid hand contact input")
        p = np.asarray(points, dtype=float)
        if p.shape != (26, 3) or not np.isfinite(p).all():
            raise ValueError("invalid contact landmarks")
        prev = self.state
        dt = 0. if prev is None else stamp-prev.stamp
        if prev is not None and dt <= 0:
            raise ValueError("non-advancing hand source time")
        reset = dt > POLICY["clock_gap_s"]
        if reset:
            dt = 0.
        active = np.zeros(4, bool) if prev is None else prev.active.copy()
        projection = np.zeros(4) if prev is None else prev.projection.copy()
        entering = np.zeros(4) if prev is None or reset else prev.entering.copy()
        leaving = np.zeros(4) if prev is None or reset else prev.leaving.copy()
        minimum = d.copy() if prev is None else prev.minimum.copy()
        history = () if prev is None or reset else tuple(
            h for h in prev.history if stamp-h[0] <= POLICY["history_s"])
        tips = p[[10, 15, 20, 25]]
        relative = (tips-p[[7, 12, 17, 22]])/width
        toward = p[5]-tips
        toward = toward/np.maximum(np.linalg.norm(toward, axis=1)[:, None], 1e-12)
        own = np.zeros(4, bool)
        closing = np.zeros(4, bool)
        if history and stamp-history[0][0] >= POLICY["minimum_history_s"]:
            old = history[0]
            own = np.sum((relative-old[2])*old[3], axis=1) >= POLICY["own_approach_width"]
            closing = old[1]-d >= POLICY["approach_m"]
        history = (*history, (stamp, d.copy(), relative, toward))[-POLICY["history_capacity"]:]

        # A clearly isolated pair may have nonzero landmark spacing at physical
        # contact. A nearest rank alone never establishes contact on an open hand.
        order = np.argsort(d)
        first, second = int(order[0]), int(order[1])
        isolated = np.zeros(4, bool)
        isolated[first] = (d[second]-d[first] >= POLICY["separation_m"]
                           and d[first] <= POLICY["separation_ratio"]*d[second])
        dominated = np.full(4, bool(isolated[first]))
        dominated[first] = False
        approach = own & closing
        close = d < POLICY["close_m"]
        # A secondary pair needs independent approach evidence when another
        # pair clearly dominates. Non-dominant close pairs can coexist.
        entry = ((close & (~dominated | approach))
                 | (isolated & (d < POLICY["isolated_enter_m"]))
                 | (approach & (d < POLICY["multi_enter_m"])))
        duration = np.where(close, POLICY["close_confirm_s"], POLICY["relative_confirm_s"])
        reason = []
        for f in range(4):
            if active[f]:
                minimum[f] = min(minimum[f], d[f])
                competing_departure = dominated[f] and d[f]-minimum[f] >= POLICY["departure_m"] and not approach[f]
                supported = ((d[f] < POLICY["ordinary_exit_m"] and not competing_departure)
                             or (isolated[f] and d[f] < POLICY["isolated_exit_m"]))
                leaving[f] = 0. if supported else leaving[f]+dt
                if not reset and (d[f] >= POLICY["isolated_exit_m"]
                                  or (not isolated[f] and d[f] >= POLICY["ordinary_exit_m"])
                                  or leaving[f] >= POLICY["exit_confirm_s"]-1e-9):
                    active[f] = False
                    entering[f] = leaving[f] = 0.
                reason.append("confirmed" if active[f] else "separated")
            else:
                entering[f] = entering[f]+dt if entry[f] else 0.
                if entering[f] >= duration[f]-1e-9:
                    active[f] = True
                    minimum[f] = d[f]
                    leaving[f] = 0.
                reason.append("confirmed" if active[f] else "candidate" if entry[f] else "unconfirmed")
        projection += np.clip(active.astype(float)-projection,
                              -dt/POLICY["projection_s"], dt/POLICY["projection_s"])
        state = State(stamp, active, projection, entering, leaving, minimum, history)
        return state, dict(algorithm=POLICY["algorithm"], reason=reason,
                           isolated_pair=isolated.tolist(), independent_approach=approach.tolist(),
                           entry_evidence=entry.tolist(), entering_s=entering.tolist(),
                           leaving_s=leaving.tolist(), closest_gap_mm=float(d[first]*1000),
                           second_gap_mm=float(d[second]*1000), clock_reset=reset)

    def commit(self, state):
        self.state = state
