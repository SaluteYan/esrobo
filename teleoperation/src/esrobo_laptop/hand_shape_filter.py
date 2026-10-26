"""Transactional smoothing of optical finger shape in the palm-local frame.

This never filters the independently published palm orientation or changes
sample timestamps. The owner commits the candidate only after publication.
"""
from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class ShapeState:
    smoothed: np.ndarray
    output: np.ndarray
    stamp: float


class HandShapeFilter:
    def __init__(self, cfg, max_gap_s=.1):
        self.enabled = cfg.left_shape_filter_enabled
        self.minimum = float(cfg.left_shape_filter_min_cutoff_hz)
        self.maximum = float(cfg.left_shape_filter_max_cutoff_hz)
        self.beta = float(cfg.left_shape_filter_beta)
        self.deadband = float(cfg.left_shape_filter_deadband_m)
        self.max_gap = float(max_gap_s)
        if (type(self.enabled) is not bool or
                not all(math.isfinite(v) for v in (self.minimum, self.maximum, self.beta,
                                                   self.deadband, self.max_gap)) or
                not 0 < self.minimum <= self.maximum or self.beta < 0 or
                not 0 <= self.deadband <= .002 or not 0 < self.max_gap <= .1):
            raise ValueError('invalid left hand shape filter settings')
        self.state = None

    def propose(self, points, stamp):
        points = np.asarray(points, dtype=float)
        if points.shape != (26, 3) or not np.isfinite(points).all() or not math.isfinite(stamp):
            raise ValueError('invalid hand shape filter input')
        previous = self.state
        dt = None if previous is None else stamp-previous.stamp
        if dt is not None and dt <= 0:
            raise ValueError('non-advancing hand shape source time')
        reset = dt is not None and dt > self.max_gap
        cutoff = np.full(26, self.minimum)
        if not self.enabled or previous is None:
            smoothed = output = points.copy()
        elif reset:
            # Rebuild the clock without banking a long-gap motion allowance.
            smoothed, output = previous.smoothed.copy(), previous.output.copy()
        else:
            speed = np.linalg.norm(points-previous.smoothed, axis=1)/dt
            # One cutoff for each chain keeps its landmarks temporally coherent.
            for start in (2, 7, 12, 17, 22):
                speed[start:start+4] = max(speed[start:start+4])
            cutoff = np.minimum(self.maximum, self.minimum+self.beta*speed)
            alpha = -np.expm1(-2*np.pi*cutoff*dt)
            smoothed = previous.smoothed+alpha[:, None]*(points-previous.smoothed)
            gap = smoothed-previous.output
            length = np.linalg.norm(gap, axis=1)
            output = previous.output+gap*np.maximum(
                0., 1.-self.deadband/np.maximum(length, 1.e-12))[:, None]
        candidate = ShapeState(smoothed.copy(), output.copy(), stamp)
        diagnostic = dict(enabled=self.enabled, algorithm='palm_local_continuous_v1',
                          initialized=previous is None, clock_reset=reset, source_dt_s=dt,
                          cutoff_hz=cutoff.tolist(), deadband_m=self.deadband,
                          input_output_max_mm=float(np.max(np.linalg.norm(points-output, axis=1))*1000),
                          update_max_mm=0. if previous is None else float(
                              np.max(np.linalg.norm(output-previous.output, axis=1))*1000))
        return candidate, diagnostic

    def commit(self, candidate):
        self.state = candidate
