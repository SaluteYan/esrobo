"""Bound a MOVE J stream when measured joints stop making progress.

This is producer flow control, not a model of firmware queues or an execution
acknowledgement. Braking still goes through CommandTrajectory on every cycle.
"""
import numpy as np

from .command_trajectory import TrajectoryFault


class CommandBackpressure:
    def __init__(self, *, timeout=2., pause_after=.15,
                 stationary_velocity=np.deg2rad(.5)):
        self.timeout = float(timeout)
        self.stationary_velocity = float(stationary_velocity)
        self.pause_after = float(pause_after)
        self.error_threshold = np.deg2rad(.1)
        self.progress_threshold = np.deg2rad(.02)
        if (not np.isfinite(self.pause_after) or self.pause_after < 0
                or not np.isfinite(self.timeout) or self.timeout <= self.pause_after):
            raise ValueError("backpressure requires 0 <= pause_after < timeout")
        self.reset()

    def reset(self):
        self.previous = None
        self.command_key = None
        self.anchor = np.zeros(7)
        self.since = np.full(7, np.nan)
        self.direction = np.zeros(7)
        self.paused = False
        self.settled = 0
        self.diagnostics = {}

    def evaluate(self, state, feedback, stamp):
        """Return brake_only; never advance the trajectory or send a command."""
        feedback = np.asarray(feedback, dtype=float)
        if feedback.shape != (7,) or not np.all(np.isfinite(feedback)) or not np.isfinite(stamp):
            raise TrajectoryFault("invalid backpressure feedback")
        if state is None:
            raise TrajectoryFault("backpressure requires initialized trajectory")
        if self.previous is not None and stamp <= self.previous[1]:
            if stamp < self.previous[1]:
                raise TrajectoryFault("backpressure feedback timestamp reversed")
            return self.paused  # Duplicate frames cannot count toward settling.
        error = state.position - feedback
        active = np.abs(error) > self.error_threshold
        direction = np.sign(error)
        command_key = joint_wire_key(state.position)
        command_changed = (
            self.command_key is not None and command_key != self.command_key
        )
        self.command_key = command_key
        progress = (feedback - self.anchor) * direction
        restart = active & (np.isnan(self.since) | (direction != self.direction)
                            | (progress >= self.progress_threshold))
        self.since = np.where(active, np.where(restart, stamp, self.since), np.nan)
        self.anchor = np.where(restart | ~active, feedback, self.anchor)
        elapsed = np.where(active, stamp - self.since, 0.)
        self.direction = direction
        stationary = (self.previous is not None
                      and np.max(np.abs(feedback - self.previous[0]) / (stamp - self.previous[1]))
                      <= self.stationary_velocity)
        self.previous = (feedback.copy(), float(stamp))
        was_paused = self.paused
        self.paused = self.paused or bool(np.any(active & (elapsed >= self.pause_after)))
        if self.paused and (not was_paused or command_changed):
            # The controller has not yet had ``timeout`` seconds to act while
            # the bounded trajectory is still braking toward a new wire
            # position.  Start the stall window after the last distinct SDK
            # millidegree payload, while preserving per-joint progress checks.
            self.since = np.where(active, stamp, self.since)
            self.anchor = np.where(active, feedback, self.anchor)
            elapsed = np.where(active, stamp - self.since, 0.)
        if self.paused:
            settled = (not np.any(active) and stationary
                       and np.max(np.abs(state.velocity)) <= 1e-9)
            self.settled = self.settled + 1 if settled else 0
            if self.settled >= 3:
                self.paused = False
                self.settled = 0
        self.diagnostics = dict(
            paused=self.paused, settled_samples=self.settled,
            no_progress_s=elapsed.tolist(), error_rad=error.tolist(),
            pause_after_s=self.pause_after, timeout_s=self.timeout,
            error_threshold_rad=self.error_threshold,
            progress_threshold_rad=self.progress_threshold,
        )
        stalled = active & (elapsed >= self.timeout)
        if np.any(stalled):
            joints = (np.flatnonzero(stalled) + 1).tolist()
            raise TrajectoryFault(f"persistent feedback lead on joints {joints} (command backpressure)", dict(
                fault_code="persistent_feedback_lead", failed_joints=joints,
                source="command_backpressure", feedback_stamp_s=float(stamp),
                previous_command_physical_rad=state.position.tolist(),
                feedback_physical_rad=feedback.tolist(), backpressure=self.diagnostics,
            ))
        return self.paused


def joint_wire_key(position):
    """Match the SDK's seven signed millidegree fields, including round ties."""
    return tuple(round(float(q) * (180. / np.pi) * 1000) for q in position)


class MoveJWaypointGate:
    """Submit one actionable MOVE_J waypoint, then wait for feedback.

    The Nero trajectory planner is a point-to-point interface.  Replacing its
    target every body-tracking cycle can keep restarting (or reject) the
    controller-side plan.  This gate lets the host trajectory accumulate a
    useful, bounded waypoint and permits the next send only after the previous
    one has measurably completed.
    """

    def __init__(self, *, minimum_step=np.deg2rad(.5), timeout=3.,
                 tolerance=np.deg2rad(.1), progress=np.deg2rad(.02)):
        self.minimum_step = float(minimum_step)
        self.timeout = float(timeout)
        self.tolerance = float(tolerance)
        self.progress = float(progress)
        if (not np.isfinite(self.minimum_step) or self.minimum_step <= 0
                or not np.isfinite(self.timeout) or self.timeout <= 0
                or not np.isfinite(self.tolerance) or self.tolerance <= 0
                or not np.isfinite(self.progress) or self.progress <= 0):
            raise ValueError("invalid MOVE_J waypoint gate configuration")
        self.reset()

    def reset(self):
        self.last_sent = None
        self.pending = None
        self.anchor = np.zeros(7)
        self.since = np.full(7, np.nan)
        self.direction = np.zeros(7)
        self.settled = 0
        self.last_stamp = None
        self.diagnostics = {}

    def should_send(self, position, feedback, *, force=False):
        position = np.asarray(position, dtype=float)
        feedback = np.asarray(feedback, dtype=float)
        if position.shape != (7,) or feedback.shape != (7,):
            raise TrajectoryFault("MOVE_J waypoint gate requires seven joints")
        if self.pending is not None:
            return False
        baseline = feedback if self.last_sent is None else self.last_sent
        return bool(force or np.max(np.abs(position - baseline)) >= self.minimum_step)

    def mark_sent(self, position, feedback, stamp):
        position = np.asarray(position, dtype=float).copy()
        feedback = np.asarray(feedback, dtype=float).copy()
        self.last_sent = position
        error = position - feedback
        active = np.abs(error) > self.tolerance
        self.pending = position if np.any(active) else None
        self.anchor = feedback
        self.since = np.where(active, float(stamp), np.nan)
        self.direction = np.sign(error)
        self.settled = 0
        self.last_stamp = float(stamp)
        self._update_diagnostics(feedback, stamp)

    def evaluate(self, feedback, stamp):
        """Return True while a transmitted waypoint remains outstanding."""
        feedback = np.asarray(feedback, dtype=float)
        stamp = float(stamp)
        if feedback.shape != (7,) or not np.all(np.isfinite(feedback)) or not np.isfinite(stamp):
            raise TrajectoryFault("invalid MOVE_J waypoint feedback")
        if self.pending is None:
            return False
        if self.last_stamp is not None and stamp <= self.last_stamp:
            if stamp < self.last_stamp:
                raise TrajectoryFault("MOVE_J waypoint feedback timestamp reversed")
            return True
        error = self.pending - feedback
        active = np.abs(error) > self.tolerance
        direction = np.sign(error)
        progress = (feedback - self.anchor) * direction
        restart = active & ((direction != self.direction) | (progress >= self.progress))
        self.since = np.where(active, np.where(restart, stamp, self.since), np.nan)
        self.anchor = np.where(restart | ~active, feedback, self.anchor)
        self.direction = direction
        self.last_stamp = stamp
        self.settled = self.settled + 1 if not np.any(active) else 0
        self._update_diagnostics(feedback, stamp)
        stalled = active & (stamp - self.since >= self.timeout)
        if np.any(stalled):
            joints = (np.flatnonzero(stalled) + 1).tolist()
            raise TrajectoryFault(
                f"persistent MOVE_J waypoint on joints {joints} (controller did not execute)",
                dict(
                    fault_code="persistent_feedback_lead",
                    failed_joints=joints,
                    source="move_j_stop_and_wait",
                    feedback_stamp_s=stamp,
                    previous_command_physical_rad=self.pending.tolist(),
                    feedback_physical_rad=feedback.tolist(),
                    waypoint_gate=self.diagnostics,
                ),
            )
        if self.settled >= 3:
            self.pending = None
            self.settled = 0
            self._update_diagnostics(feedback, stamp)
            return False
        return True

    def _update_diagnostics(self, feedback, stamp):
        error = np.zeros(7) if self.pending is None else self.pending - feedback
        elapsed = np.where(np.isnan(self.since), 0., float(stamp) - self.since)
        self.diagnostics = dict(
            pending=self.pending is not None,
            settled_samples=self.settled,
            no_progress_s=elapsed.tolist(),
            error_rad=error.tolist(),
            minimum_step_rad=self.minimum_step,
            tolerance_rad=self.tolerance,
            timeout_s=self.timeout,
        )
