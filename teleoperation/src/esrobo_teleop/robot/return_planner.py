"""Bounded, cancellable joint-space return planning. No hardware access."""
from dataclasses import dataclass
import time
import numpy as np


@dataclass(frozen=True)
class ReturnResult:
    returned: bool
    disabled: bool
    stage: str
    reason: str = ""


@dataclass(frozen=True)
class ReturnPlan:
    path: tuple
    seed: int
    nodes: int
    reason: str = ""


class ReturnPlanner:
    def __init__(self, lower, upper, check, *, timeout=5., max_nodes=5000,
                 seed=0, step=np.deg2rad(4), cancelled=lambda: False):
        self.lower, self.upper = np.asarray(lower, float), np.asarray(upper, float)
        if (self.lower.shape != (7,) or self.upper.shape != (7,)
                or not np.all(np.isfinite([self.lower, self.upper]))
                or np.any(self.lower >= self.upper) or not np.isfinite(timeout)
                or timeout <= 0 or max_nodes < 2 or not np.isfinite(step) or step <= 0):
            raise ValueError("invalid return planner limits")
        self.check, self.timeout, self.max_nodes = check, timeout, max_nodes
        self.seed, self.step, self.cancelled = int(seed), step, cancelled

    def plan(self, start, goal):
        deadline = time.monotonic() + self.timeout
        nodes = 2
        def alive():
            return not self.cancelled() and time.monotonic() < deadline
        def edge(a, b):
            return alive() and bool(self.check(a, b)) and alive()
        def result(path=(), reason=""):
            return ReturnPlan(tuple(np.array(p, copy=True) for p in path), self.seed, nodes, reason)
        start, goal = np.asarray(start, float), np.asarray(goal, float)
        for name, q in (("start", start), ("goal", goal)):
            if (q.shape != (7,) or not np.all(np.isfinite(q))
                    or np.any(q < self.lower) or np.any(q > self.upper)):
                return result(reason=f"{name} outside joint limits")
            if not edge(q, q):
                return result(reason=f"{name} pose not certified safe")
        if edge(start, goal):
            return result([start, goal])
        # A large independent-axis box may be unsafe while a sequence of
        # short, stop-at-waypoint boxes along the same ray is safe. Retain
        # these intermediate targets in the result; never claim the big edge
        # itself is safe merely because its diagonal is clear.
        count = max(1, int(np.ceil(np.linalg.norm(goal-start)/self.step)))
        if count+1 <= self.max_nodes:
            direct = [start + (goal-start)*(i/count) for i in range(count+1)]
            if all(edge(x, y) for x, y in zip(direct, direct[1:])):
                nodes = len(direct)
                return result(direct)
        rng = np.random.default_rng(self.seed)
        a, b = [(start, -1)], [(goal, -1)]
        swapped = False
        def extend(tree, target):
            nonlocal nodes
            if nodes >= self.max_nodes or not alive():
                return None, False
            distances = [np.linalg.norm(p-target) for p, _ in tree]
            parent = int(np.argmin(distances))
            q = tree[parent][0]
            delta = target-q
            length = np.linalg.norm(delta)
            if length < 1e-10:
                return parent, True
            reached = length <= self.step
            nxt = target.copy() if reached else q + delta * self.step/length
            if not edge(q, nxt):
                return None, False
            tree.append((nxt, parent))
            nodes += 1
            return len(tree)-1, reached
        def trace(tree, i):
            path = []
            while i >= 0:
                q, i = tree[i]
                path.append(q)
            return path[::-1]
        for tree in (a, b):
            root = tree[0][0]
            for joint in range(7):
                for sign in (-1, 1):
                    target = root.copy()
                    target[joint] = np.clip(target[joint]+sign*self.step, self.lower[joint], self.upper[joint])
                    extend(tree, target)
        while alive() and nodes < self.max_nodes:
            target = b[0][0] if rng.random() < .2 else rng.uniform(self.lower, self.upper)
            ai, _ = extend(a, target)
            if ai is not None:
                while alive() and nodes < self.max_nodes:
                    bi, reached = extend(b, a[ai][0])
                    if bi is None:
                        break
                    if reached:
                        path = trace(a, ai) + trace(b, bi)[-2::-1]
                        if swapped:
                            path.reverse()
                        # Bounded shortcutting. Every replacement is a checked
                        # interval box, never merely a diagonal sample.
                        i = 0
                        while i < len(path)-2 and alive():
                            for j in range(len(path)-1, i+1, -1):
                                if edge(path[i], path[j]):
                                    path[i+1:j] = []
                                    break
                            i += 1
                        if not alive():
                            break
                        if all(edge(x, y) for x, y in zip(path, path[1:])):
                            return result(path)
                        return result(reason="path changed or validation budget expired")
            a, b, swapped = b, a, not swapped
        return result(reason="cancelled" if self.cancelled() else "no certified path within search budget")
