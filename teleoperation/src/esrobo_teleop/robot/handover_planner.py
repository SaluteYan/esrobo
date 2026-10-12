"""Offline two-leg single-arm entry planning; every MOVE_J interval is certified."""
import time
import numpy as np
from .return_planner import ReturnPlanner, ReturnPlan


class HandoverPlanner:
    def __init__(self, lower, upper, check, fk, direction, *, timeout=20.,
                 max_nodes=5000, seed=0, cancelled=lambda: False, diagnostics=lambda: {}):
        self.lower, self.upper = np.asarray(lower), np.asarray(upper)
        self.check, self.fk, self.direction = check, fk, np.asarray(direction)
        self.timeout, self.max_nodes, self.seed = timeout, max_nodes, seed
        self.cancelled = cancelled
        self.diagnostics = diagnostics
        self.certificates = []
        # Reuse the kernel's finite-limit/budget validation before any FK call.
        ReturnPlanner(self.lower, self.upper, check, timeout=timeout, max_nodes=max_nodes)
        if self.direction.shape != (7,) or not np.all(np.isin(self.direction, (-1., 1.))):
            raise ValueError("handover requires seven physical mapping directions")

    def plan(self, start, goal):
        self.certificates = []
        start, goal = np.asarray(start, float), np.asarray(goal, float)
        deadline = time.monotonic() + self.timeout
        def alive():
            return not self.cancelled() and time.monotonic() < deadline
        def checked(a, b):
            if not alive():
                return False
            certified = bool(self.check(a, b))
            # Convex collision queries can monopolize the Python thread across
            # many edges. Yield between complete checks so the gateway's state
            # sender can keep its independent 50 Hz feedback stream alive.
            time.sleep(0.001)
            return certified and alive()
        def fail(reason):
            return ReturnPlan((), self.seed, 0, reason)
        for name, q in (("start", start), ("handover goal", goal)):
            if (q.shape != (7,) or not np.all(np.isfinite(q))
                    or np.any(q < self.lower) or np.any(q > self.upper)):
                return fail(f"{name} outside joint limits")
            if not checked(q, q):
                return fail(f"{name} not certified safe")
        if np.max(np.abs(start[5:]-goal[5:])) > 1e-10:
            return fail("handover J6/J7 must equal measured start")
        goal = goal.copy()
        goal[5:] = start[5:]
        if np.max(abs(goal-start)) < np.deg2rad(.5) and checked(start, goal):
            self.certificates = [dict(self.diagnostics(),
                start_physical_rad=start.tolist(), end_physical_rad=goal.tolist())]
            return ReturnPlan((start.copy(), goal.copy()), self.seed, 2)
        # Search both shoulder directions. FK identifies outward displacement;
        # exact geometry, not a sign convention, certifies both complete legs.
        original = np.abs(self.fk(start)[:, 1])
        candidates = []
        for j2 in (-25, -15, -8, 8, 15, 25):
            for j1 in (0, -10, 10):
                q = start.copy()
                q[:2] += np.deg2rad([j1, j2]) * self.direction[:2]
                if np.any(q < self.lower) or np.any(q > self.upper):
                    continue
                gain = float(np.min(np.abs(self.fk(q)[:, 1]) - original))
                if gain >= .025:
                    candidates.append((gain, q))
        # Prefer a modest certified outward pose, before pursuing elbow bend.
        candidates.sort(key=lambda item: np.linalg.norm(item[1]-start))
        for _, clear in candidates:
            if not alive():
                break
            # First leg moves only shoulder joints; hold J3--J7. No RRT here
            # because it could introduce an elbow swing before moving outward.
            count = max(1, int(np.ceil(np.linalg.norm(clear-start)/np.deg2rad(4))))
            first = [start+(clear-start)*i/count for i in range(count+1)]
            if not all(checked(a,b) for a,b in zip(first, first[1:])):
                continue
            remaining = deadline-time.monotonic()
            if remaining <= 0:
                break
            second = ReturnPlanner(self.lower, self.upper, checked,
                timeout=min(5., remaining), max_nodes=self.max_nodes, seed=self.seed,
                cancelled=self.cancelled,
                locked_joints=(5, 6)).plan(clear, goal)
            if second.path:
                path = tuple(first) + second.path[1:]
                self.certificates = []
                for a,b in zip(path,path[1:]):
                    if not checked(a,b):
                        self.certificates = []
                        break
                    self.certificates.append(dict(self.diagnostics(),
                        start_physical_rad=a.tolist(), end_physical_rad=b.tolist()))
                if len(self.certificates) == len(path)-1:
                    return ReturnPlan(path, self.seed, len(first)+second.nodes)
        return fail("cancelled" if self.cancelled() else
                    "no certified outward shoulder pose / two-leg path within budget")
