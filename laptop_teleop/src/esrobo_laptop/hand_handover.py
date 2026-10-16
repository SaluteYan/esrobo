"""Feedback-bounded first hand transition; no sensor extrapolation."""
import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class HandStep:
    position: tuple
    at: float
    matched: int
    direction: tuple
    reversal_wait: tuple
    reference_target: tuple


class HandHandover:
    def __init__(self, measured, cfg, now, control_hz, *, live_goal=False, side=None):
        self.live_goal = live_goal
        self.velocity = float(cfg.max_velocity)
        self.step = int(cfg.max_step)
        self.period = 1. / max(float(cfg.publish_hz), float(control_hz))
        if not (math.isfinite(self.velocity) and self.velocity > 0 and self.step > 0
                and math.isfinite(self.period) and self.period > 0):
            raise ValueError("hand handover requires finite positive motion limits")
        from esrobo_teleop.robot.hand_control_mapping import enabled_for
        self.enabled = list(cfg.enabled_physical_joints) if side is None else enabled_for(cfg, side)
        self.lead = min(15, self.step)
        self.started = now
        self.command = self.checked(measured)
        self.last_at = now
        self.matched = 0
        self.done = False
        self.direction = np.zeros(10, dtype=int)
        self.reversal_wait = ()
        self.reference_target = None

    @staticmethod
    def checked(value):
        value = np.asarray(value, dtype=float)
        if value.shape != (10,) or not np.all(np.isfinite(value)) or np.any((value < 0) | (value > 255)):
            raise ValueError("hand handover requires ten valid physical positions")
        return np.rint(value).astype(int)

    def hold(self, measured, now):
        # The gateway's hold command uses measured fingers. Do not accumulate
        # motion credit or retain an ahead-of-feedback command during a pause.
        self.command = self.checked(measured)
        self.last_at = now
        self.matched = 0
        self.direction = np.zeros(10, dtype=int)
        self.reversal_wait = ()

    def propose(self, desired, measured, now):
        source, actual = self.checked(desired), self.checked(measured)
        # Complete one bounded initial transition before live following. A
        # moving human target must not keep restarting the arrival criterion.
        # Latch only in commit(), after input checks and successful transport.
        goal = source if self.live_goal or self.reference_target is None else np.asarray(self.reference_target)
        if not math.isfinite(now) or now < self.last_at:
            raise ValueError("hand handover clock moved backwards or is invalid")
        if now-self.started > 10.:
            pending = [
                f"physical[{i}] goal={goal[i]} command={self.command[i]} feedback={actual[i]}"
                for i in self.enabled
                if self.command[i] != goal[i] or abs(actual[i]-goal[i]) > 2
            ]
            detail = "; ".join(pending) or f"matched_frames={self.matched}/3"
            raise RuntimeError(
                "initial hand handover exceeded 10 s; explicit recovery required; " + detail)
        dt = max(0., min(now-self.last_at, self.period))
        budget = min(self.step, int(math.floor(self.velocity*dt+1.e-7)))
        lower = np.maximum(self.command-budget, actual-self.lead)
        upper = np.minimum(self.command+budget, actual+self.lead)
        for i in self.enabled:
            if lower[i] > upper[i]:
                raise RuntimeError(
                    f"hand handover joint {i+1} feedback diverged from command; "
                    f"physical[{i}] goal={goal[i]} command={self.command[i]} "
                    f"feedback={actual[i]} budget={budget} lead={self.lead}")
        # A fresh feedback packet can still describe a finger following older
        # commands. Do not send a reversal while it is chasing the last target:
        # otherwise the delayed original movement and the reversed command can
        # separate beyond both limits on the next feedback update. Use the
        # existing two-unit arrival tolerance, not an enlarged feedback band.
        reversing = (np.sign(goal-self.command)*self.direction < 0)
        waiting = reversing & (np.abs(actual-self.command) > 2)
        bounded_goal = np.where(waiting, self.command, goal)
        candidate = actual.copy()
        candidate[self.enabled] = np.clip(bounded_goal[self.enabled], lower[self.enabled], upper[self.enabled])
        direction = self.direction.copy()
        delta = np.sign(candidate-self.command)
        for i in self.enabled:
            if delta[i]:
                direction[i] = delta[i]
        matched = (np.all(candidate[self.enabled] == goal[self.enabled])
                   and np.all(np.abs(actual[self.enabled]-goal[self.enabled]) <= 2))
        return HandStep(tuple(int(v) for v in candidate), now,
                        self.matched+1 if matched else 0,
                        tuple(int(v) for v in direction),
                        tuple(i for i in self.enabled if waiting[i]),
                        tuple(int(v) for v in goal))

    def commit(self, candidate):
        # Only the caller that successfully sent this candidate may commit it.
        # Rejected input and transport errors must leave the sent baseline intact.
        self.command = np.asarray(candidate.position, dtype=int).copy()
        self.last_at, self.matched = candidate.at, candidate.matched
        self.direction = np.asarray(candidate.direction, dtype=int).copy()
        self.reversal_wait = candidate.reversal_wait
        self.reference_target = candidate.reference_target
        self.done = self.matched >= 3
