"""Certified shoulder-first return planning for an arm near the torso."""

import time

import numpy as np

from .return_planner import ReturnPlan, ReturnPlanner


class StagedReturnPlanner:
    """Try an outward shoulder corridor before a general joint-space search.

    Every returned edge is checked as a complete joint interval. In particular,
    the elbow is not folded until the measured pose has reached a certified
    shoulder-out waypoint. Failure to certify any stage leaves the robot still.
    """

    def __init__(self, lower, upper, check, fk, direction, *, timeout=5.,
                 max_nodes=5000, seed=0, cancelled=lambda: False,
                 diagnostics=lambda: {}):
        self.lower = np.asarray(lower, dtype=float)
        self.upper = np.asarray(upper, dtype=float)
        self.check = check
        self.fk = fk
        self.direction = np.asarray(direction, dtype=float)
        self.timeout = float(timeout)
        self.max_nodes = int(max_nodes)
        self.seed = int(seed)
        self.cancelled = cancelled
        self.diagnostics = diagnostics
        self.certificates = []
        ReturnPlanner(self.lower, self.upper, check, timeout=timeout,
                      max_nodes=max_nodes, seed=seed, cancelled=cancelled)
        if (self.direction.shape != (7,)
                or not np.all(np.isin(self.direction, (-1., 1.)))):
            raise ValueError("return requires seven physical mapping directions")

    def plan(self, start, goal):
        self.certificates = []
        start, goal = np.asarray(start, float), np.asarray(goal, float)
        deadline = time.monotonic() + self.timeout
        checks = 0

        def alive():
            return not self.cancelled() and time.monotonic() < deadline

        def checked(a, b):
            nonlocal checks
            if not alive():
                return False
            checks += 1
            return bool(self.check(a, b)) and alive()

        def segment(a, b):
            count = max(1, int(np.ceil(np.linalg.norm(b-a)/np.deg2rad(4))))
            if count > self.max_nodes:
                return None
            path = [a+(b-a)*i/count for i in range(count+1)]
            if all(checked(x, y) for x, y in zip(path, path[1:])):
                return path
            return None

        def certified(path):
            if len(path) > self.max_nodes:
                return None
            certificates = []
            for a, b in zip(path, path[1:]):
                if not checked(a, b):
                    return None
                certificates.append(dict(self.diagnostics(),
                    start_physical_rad=a.tolist(), end_physical_rad=b.tolist()))
            self.certificates = certificates
            return ReturnPlan(tuple(np.array(q, copy=True) for q in path),
                              self.seed, len(path))

        for name, q in (("start", start), ("goal", goal)):
            if (q.shape != (7,) or not np.all(np.isfinite(q))
                    or np.any(q < self.lower) or np.any(q > self.upper)):
                return ReturnPlan((), self.seed, checks,
                                  f"{name} outside joint limits")
            if not checked(q, q):
                return ReturnPlan((), self.seed, checks,
                                  f"{name} pose not certified safe")

        if checked(start, goal):
            direct = certified([start, goal])
            if direct is not None:
                return direct
        direct = segment(start, goal)
        if direct is not None:
            result = certified(direct)
            if result is not None:
                return result

        # As in handover, use FK to select shoulder-only moves that increase
        # both elbow and wrist distance from the centre line. Keep J3--J7 fixed.
        original = np.abs(self.fk(start)[:, 1])
        candidates = []
        for j2 in (-8, 8, -15, 15, -25, 25):
            for j1 in (0, -10, 10):
                clear = start.copy()
                clear[:2] += np.deg2rad([j1, j2]) * self.direction[:2]
                if np.any(clear < self.lower) or np.any(clear > self.upper):
                    continue
                gain = float(np.min(np.abs(self.fk(clear)[:, 1]) - original))
                if gain >= .025:
                    candidates.append((np.linalg.norm(clear-start), clear))
        candidates.sort(key=lambda item: item[0])

        for _, clear in candidates:
            if not alive():
                break
            first = segment(start, clear)
            if first is None:
                continue
            # Fold the elbow while the shoulder remains outside. Try both
            # elbow orders because the safe order depends on the current pose.
            for order in ((3, 2), (2, 3)):
                path = list(first)
                current = clear.copy()
                for joint in order:
                    next_pose = current.copy()
                    next_pose[joint] = goal[joint]
                    leg = segment(current, next_pose)
                    if leg is None:
                        break
                    path.extend(leg[1:])
                    current = next_pose
                else:
                    # Set the wrist before bringing the shoulder toward the
                    # torso. Never silently skip even a small wrist difference.
                    wrist = current.copy()
                    wrist[4:] = goal[4:]
                    leg = segment(current, wrist)
                    if leg is None:
                        continue
                    path.extend(leg[1:])
                    current = wrist
                    final = segment(current, goal)
                    if final is None:
                        continue
                    path.extend(final[1:])
                    result = certified(path)
                    if result is not None:
                        return result

        remaining = deadline - time.monotonic()
        if self.cancelled() or remaining <= 0:
            return ReturnPlan((), self.seed, checks,
                              "cancelled" if self.cancelled() else
                              "no certified staged return path within search budget")
        fallback = ReturnPlanner(self.lower, self.upper, checked,
            timeout=remaining, max_nodes=self.max_nodes,
            seed=self.seed, cancelled=self.cancelled).plan(start, goal)
        if fallback.path:
            result = certified(fallback.path)
            if result is not None:
                return result
        return ReturnPlan((), self.seed, checks,
                          "cancelled" if self.cancelled() else
                          "no certified staged or general return path within search budget")
