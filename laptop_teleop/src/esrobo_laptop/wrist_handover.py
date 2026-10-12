"""Measured-start J6/J7 target transition; robot trajectory checks remain authoritative."""
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class WristStep:
    position: np.ndarray
    at: float
    matched: int


class WristHandover:
    def __init__(self, measured, cfg, side, now, control_hz):
        def limits(name):
            value = getattr(cfg, side+'_'+name)
            if value is None:
                value = getattr(cfg, name)
            value = self.joints(value)[5:]
            if np.any(value <= 0):
                raise ValueError('wrist handover requires positive motion limits')
            return value
        self.velocity = limits('max_joint_velocity')
        self.step = limits('max_joint_step')
        self.lead = np.deg2rad(self.joints(cfg.command_trajectory_lead_deg)[5:])
        self.period = 1./float(control_hz)
        self.timeout = float(cfg.wrist_return_timeout_s)
        self.tolerance = np.deg2rad(float(cfg.wrist_return_tolerance_deg))
        if (not np.all(np.isfinite([self.period, self.timeout, self.tolerance]))
                or min(self.period, self.timeout, self.tolerance) <= 0
                or np.any(self.lead <= 0)):
            raise ValueError('invalid wrist handover timing/feedback limits')
        self.command = self.joints(measured)[5:].copy()
        self.started = self.last_at = now
        self.matched = 0
        self.done = False
        self.paused = False

    @staticmethod
    def joints(value):
        value = np.asarray(value, dtype=float)
        if value.shape != (7,) or not np.all(np.isfinite(value)):
            raise ValueError('wrist handover requires seven finite joint values')
        return value.copy()

    def check_timeout(self, now):
        if not self.done and now-self.started > self.timeout:
            raise RuntimeError('initial wrist handover exceeded '
                               f'{self.timeout:g} s; explicit recovery required')

    def pause(self, now):
        # Gateway braking can change the final stopped pose. Resume from fresh
        # measured joints, without accumulating elapsed-time movement credit.
        self.last_at, self.matched, self.paused = now, 0, True

    def propose(self, desired, measured, now):
        self.check_timeout(now)
        goal, actual = self.joints(desired)[5:], self.joints(measured)[5:]
        if now < self.last_at:
            raise ValueError('wrist handover clock moved backwards')
        base = actual if self.paused else self.command
        budget = np.minimum(self.step, self.velocity*min(now-self.last_at, self.period))
        lower = np.maximum(base-budget, actual-self.lead)
        upper = np.minimum(base+budget, actual+self.lead)
        if np.any(lower > upper):
            raise RuntimeError('wrist handover feedback diverged from command')
        candidate = np.clip(goal, lower, upper)
        matched = (np.all(np.abs(candidate-goal) <= 1.e-10)
                   and np.all(np.abs(actual-goal) <= self.tolerance))
        return WristStep(candidate.copy(), now, self.matched+1 if matched else 0)

    def commit(self, candidate):
        # A failed send or expired input cannot advance the transition.
        self.command = candidate.position.copy()
        self.last_at, self.matched = candidate.at, candidate.matched
        self.done, self.paused = self.matched >= 3, False

    def diagnostics(self):
        return dict(active=not self.done, command_deg=np.degrees(self.command).tolist(),
                    matched_frames=self.matched, paused=self.paused)
