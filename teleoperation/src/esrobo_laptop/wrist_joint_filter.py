"""Transactional, source-clock J6/J7 filtering; no hardware or trajectory state."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class WristFilterStep:
    smoothed: tuple
    output: tuple
    stamp: float
    dt: float | None
    cutoff: tuple
    alpha: tuple
    gap: tuple
    update: tuple
    clock_reset: bool


class WristJointFilter:
    """All positions are bounded joint radians, never wrapped Euler angles."""

    def __init__(self, seed, config, lower, upper):
        self.minimum = float(config.wrist_joint_filter_min_cutoff_hz)
        self.maximum = float(config.wrist_joint_filter_max_cutoff_hz)
        self.coefficient = float(config.wrist_joint_filter_speed_coefficient)
        self.deadband = math.radians(float(config.wrist_joint_filter_deadband_deg))
        if (not all(math.isfinite(x) for x in
                    (self.minimum, self.maximum, self.coefficient, self.deadband))
                or not 0 < self.minimum <= self.maximum
                or self.coefficient < 0 or not 0 <= self.deadband < math.pi):
            raise ValueError('invalid J6/J7 filter parameters')
        self.lower, self.upper = self.pair(lower), self.pair(upper)
        if any(lo >= hi for lo, hi in zip(self.lower, self.upper)):
            raise ValueError('invalid J6/J7 bounds')
        seed = self.checked(seed)
        self.smoothed = seed.copy()
        self.output = seed.copy()
        self.stamp = None

    @staticmethod
    def pair(value):
        try:
            if len(value) != 2 or getattr(value, 'ndim', 1) != 1:
                raise ValueError('two positions required')
            pair = [float(value[0]), float(value[1])]
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError('two finite J6/J7 positions required') from exc
        if not all(math.isfinite(x) for x in pair):
            raise ValueError('two finite J6/J7 positions required')
        return pair

    def checked(self, value):
        value = self.pair(value)
        if any(q < lo or q > hi for q, lo, hi in zip(value, self.lower, self.upper)):
            raise ValueError('J6/J7 filter input outside joint bounds')
        return value

    def propose(self, target, stamp):
        goal = self.checked(target)
        stamp = float(stamp)
        if not math.isfinite(stamp):
            raise ValueError('finite PICO palm source time required')
        dt = None if self.stamp is None else stamp-self.stamp
        smoothed, output = self.smoothed.copy(), self.output.copy()
        cutoff = [self.minimum, self.minimum]
        alpha, gap, update = [0., 0.], [0., 0.], [0., 0.]
        reset = dt is None or dt > .1
        next_stamp = stamp if reset else self.stamp
        if dt is not None and 0 < dt <= .1:
            # Two scalar channels avoid NumPy allocation/dispatch in the 50 Hz
            # producer. Same exponential low-pass and excess-angle formula.
            for j in range(2):
                cutoff[j] = min(self.maximum,
                    self.minimum+self.coefficient*abs(goal[j]-smoothed[j])/dt)
                alpha[j] = -math.expm1(-2*math.pi*cutoff[j]*dt)
                smoothed[j] += alpha[j]*(goal[j]-smoothed[j])
                gap[j] = smoothed[j]-output[j]
                update[j] = math.copysign(max(0., abs(gap[j])-self.deadband), gap[j])
                output[j] += update[j]
            next_stamp = stamp
        # A gap resets only the clock; duplicate/backwards clocks change nothing.
        return WristFilterStep(tuple(smoothed), tuple(output), next_stamp, dt,
                               tuple(cutoff), tuple(alpha), tuple(gap), tuple(update), reset)

    def commit(self, candidate):
        self.smoothed = list(candidate.smoothed)
        self.output = list(candidate.output)
        self.stamp = candidate.stamp

    @staticmethod
    def diagnostics(candidate):
        return dict(mode='joint_continuous_deadband_v1',
                    source_sample_monotonic=candidate.stamp, dt_s=candidate.dt,
                    lowpass_rad=candidate.smoothed, output_rad=candidate.output,
                    cutoff_hz=candidate.cutoff, alpha=candidate.alpha,
                    gap_rad=candidate.gap, update_rad=candidate.update,
                    clock_reset=candidate.clock_reset)
