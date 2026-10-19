"""Bounded live-trajectory fallback. No hardware access or committed state."""
import time
import threading
from types import SimpleNamespace

import numpy as np
from scipy.optimize import minimize


from .trajectory_version import TRAJECTORY_ALGORITHM_VERSION

_warm_lock = threading.Lock()
_warm = False


def warmup_solver():
    """Resolve SciPy's lazy initialization before any live command deadline."""
    global _warm
    with _warm_lock:
        if not _warm:
            minimize(lambda z: float(z@z), np.ones(7)*.1, jac=lambda z: 2*z,
                     bounds=[(-1., 1.)]*7, method="SLSQP",
                     constraints={"type": "ineq", "fun": lambda z: np.array([1.-z@z]),
                                  "jac": lambda z: -2*z[None, :]},
                     options={"maxiter": 3})
            _warm = True


class SolverDeadline(Exception):
    pass


class ValidatedCandidate(Exception):
    def __init__(self, z):
        self.z = z.copy()


def solve_velocity(q, previous, goal, brake, lower, upper, vmax, dt,
                   endpoint_speed, before, fk, jacobian, validate, *, brake_only=False):
    """Return an independently validated candidate, never trust SLSQP status.

    The deadline is cooperative (Python/OS scheduling cannot be hard real time).
    Check after expensive calls as well, so overruns can never publish a result.
    """
    start = time.monotonic()
    deadline = start + .005
    attempts = []
    info = dict(search_method="joint_slsqp", attempts=attempts,
                solver_deadline_monotonic=deadline, solver_started_monotonic=start)
    modes = ("brake",) if brake_only else ("tracking", "brake")
    # Aim very slightly inside the unchanged exact FK limit (0.005% in
    # speed). Without this numerical interior, SLSQP spends several updates
    # approaching the boundary from its infeasible side. This is not a relaxed
    # acceptance tolerance: validate() retains the original physical limit.
    interior = 1e-4
    for mode in modes:
        end = deadline if mode == "brake" else start + .003
        detail = dict(mode=mode, iterations=0)
        attempts.append(detail)

        def check_time():
            if time.monotonic() >= end:
                raise SolverDeadline()

        cache = {}
        def geometry(z):
            check_time()
            if "z" not in cache or not np.array_equal(z, cache["z"]):
                position = q + dt * vmax * z
                points = np.asarray(fk(position), dtype=float)
                jac = np.asarray(jacobian(position), dtype=float)
                if (points.shape != (2, 3) or jac.shape != (2, 3, 7)
                        or not np.all(np.isfinite(points)) or not np.all(np.isfinite(jac))):
                    raise ValueError("non-finite or malformed endpoint FK/Jacobian")
                # Dimensionless constraints avoid metre-scale SLSQP tolerances.
                delta = (points - before) / (endpoint_speed * dt)
                cache.update(z=z.copy(), constraint=1 - interior - np.sum(delta*delta, axis=1),
                             jac=-2*np.einsum("ij,ijk->ik", delta, jac)
                             * vmax / endpoint_speed)
            check_time()
            return cache

        reference = goal / vmax
        old = previous / vmax
        def objective(z):
            check_time()
            if mode == "brake":
                return float(z @ z)
            return float(np.sum((z-reference)**2) + .01*np.sum((z-old)**2))

        def gradient(z):
            check_time()
            return 2*z if mode == "brake" else 2*(z-reference) + .02*(z-old)

        def iteration(z):
            detail["iterations"] += 1
            check_time()
            # Feasibility, not sub-microradian objective convergence, is the
            # purpose of this fallback. SLSQP's objective still determines the
            # step. Stop at its first independently verified feasible iterate.
            # This avoids spending the live budget polishing an already safe
            # candidate (and then unnecessarily requesting a latched hold).
            v = z*vmax
            if (np.all(v >= lower-1e-10) and np.all(v <= upper+1e-10)
                    and (mode != "brake" or z@z < old@old)
                    and np.all(geometry(z)["constraint"] >= -interior)
                    and validate(np.clip(v, lower, upper)) is not None):
                check_time()
                raise ValidatedCandidate(z)

        try:
            check_time()
            try:
                result = minimize(objective, (brake if mode == "brake" else goal)/vmax,
                                  jac=gradient, bounds=list(zip(lower/vmax, upper/vmax)),
                                  constraints={"type": "ineq",
                                               "fun": lambda z: geometry(z)["constraint"],
                                               "jac": lambda z: geometry(z)["jac"]},
                                  method="SLSQP", callback=iteration,
                                  options={"maxiter": 12, "ftol": 1e-8})
            except ValidatedCandidate as candidate:
                result = SimpleNamespace(x=candidate.z, status=-1, success=False)
                detail["termination"] = "validated_feasible_iterate"
            check_time()
            detail.update(optimizer_status=int(getattr(result, "status", 0 if result.success else -2)),
                          optimizer_success=bool(result.success))
            velocity = np.asarray(result.x)*vmax
            if not np.all(np.isfinite(velocity)):
                detail["rejection"] = "nonfinite_candidate"
                continue
            if np.any(velocity < lower-1e-10) or np.any(velocity > upper+1e-10):
                detail["rejection"] = "velocity_interval"
                continue
            # Clamp only floating point roundoff, then rerun all exact checks.
            velocity = np.clip(velocity, lower, upper)
            if mode == "brake" and not np.sum((velocity/vmax)**2) < np.sum(old**2):
                detail["rejection"] = "no_braking_progress"
                continue
            distance = validate(velocity)
            check_time()
            if distance is not None:
                info.update(solver_elapsed_ms=(time.monotonic()-start)*1000,
                            requires_controlled_hold=(mode == "brake" and not brake_only),
                            solver_mode=mode)
                return (q + dt*velocity, velocity, distance, 0.), info
            detail["rejection"] = "exact_safety_check"
        except SolverDeadline:
            detail["rejection"] = "solver_timeout"
    info.update(solver_elapsed_ms=(time.monotonic()-start)*1000,
                fault_code=("solver_timeout" if any(a.get("rejection") == "solver_timeout"
                                                   for a in attempts) else "endpoint_search_exhausted"))
    return None, info
