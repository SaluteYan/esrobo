"""Feedback-bounded command-space trajectory; proposals have no side effects."""

from dataclasses import dataclass
import numpy as np


class TrajectoryFault(ValueError):
    def __init__(self, message, diagnostics=None):
        super().__init__(message)
        self.diagnostics = diagnostics or {}


@dataclass(frozen=True)
class TrajectoryStep:
    position: np.ndarray
    velocity: np.ndarray
    stamp: float
    lead_since: np.ndarray
    diagnostics: dict


class CommandTrajectory:
    def __init__(self, velocity, acceleration, step, lead, *, max_dt=.1, lead_timeout=2.0):
        self.vmax, self.amax, self.step, self.lead = [
            self._vector(v) for v in (velocity, acceleration, step, lead)
        ]
        if any(np.any(v <= 0) for v in (self.vmax, self.amax, self.step, self.lead)):
            raise ValueError("command trajectory requires positive seven-axis limits")
        if not np.isfinite(max_dt) or not 0 < max_dt <= .12:
            raise ValueError("trajectory max_dt must be in (0, 0.12]")
        if not np.isfinite(lead_timeout) or lead_timeout <= 0:
            raise ValueError("invalid trajectory lead timeout")
        self.max_dt, self.lead_timeout = max_dt, lead_timeout
        self.state = None

    @staticmethod
    def _vector(value):
        result = np.asarray(value, dtype=float)
        if result.shape != (7,) or not np.all(np.isfinite(result)):
            raise TrajectoryFault("expected seven finite joints")
        return result.copy()

    def reset(self, position, stamp, *, feedback=None):
        if not np.isfinite(stamp):
            raise TrajectoryFault("invalid initialization timestamp")
        position = self._vector(position)
        measured = position if feedback is None else self._vector(feedback)
        self.state = TrajectoryStep(position, np.zeros(7), float(stamp),
                                    np.full(7, np.nan), {
                                        "feedback_physical_rad": measured.tolist(),
                                        "feedback_motion_stamp_s": float(stamp),
                                        "feedback_velocity_rad_s": [0.0] * 7,
                                    })

    def invalidate(self):
        self.state = None

    def propose(self, target, feedback, stamp, lower, upper,
                *, path_check=None, coordinate_elbow=False, brake_only=False):
        state = self.state
        if state is None:
            raise TrajectoryFault("trajectory needs explicit session initialization")
        if not np.isfinite(stamp):
            raise TrajectoryFault("invalid feedback timestamp")
        dt = float(stamp - state.stamp)
        if dt == 0:
            return None
        if dt < 0 or dt > self.max_dt:
            raise TrajectoryFault(
                f"feedback interval {dt:.4f}s outside (0, {self.max_dt}]s "
                "since last committed command state",
                dict(fault_code="command_state_interval_exceeded",
                     feedback_stamp_s=float(stamp), previous_command_stamp_s=state.stamp,
                     command_state_interval_s=dt))
        target, feedback, lower, upper = [self._vector(v) for v in (target, feedback, lower, upper)]
        q, velocity = state.position, state.velocity
        # A large bend-plane rotation must precede additional elbow flexion.
        # Gate from measured J3, not the IK seed or the last command. Continue
        # to respect acceleration when braking J4; never snap its velocity.
        elbow_wait = coordinate_elbow and abs(target[2] - feedback[2]) > np.deg2rad(10)
        if elbow_wait and target[3] > q[3]:
            target[3] = q[3]
        lo = np.maximum(lower, feedback - self.lead)
        hi = np.minimum(upper, feedback + self.lead)
        if np.any(lo > hi) or np.any(q < lo - 1e-8) or np.any(q > hi + 1e-8):
            outside = np.flatnonzero((lo > hi) | (q < lo - 1e-8) | (q > hi + 1e-8))
            raise TrajectoryFault(
                "last command outside position/feedback-lead envelope",
                dict(
                    fault_code="last_command_outside_feedback_lead",
                    failed_joints=(outside + 1).tolist(),
                    feedback_stamp_s=float(stamp),
                    feedback_dt_s=dt,
                    previous_command_stamp_s=state.stamp,
                    previous_command_physical_rad=q.tolist(),
                    feedback_physical_rad=feedback.tolist(),
                    lead_rad=(q - feedback).tolist(),
                    lead_limit_rad=self.lead.tolist(),
                ),
            )

        # Reserve stopping distance even if feedback stops moving next frame.
        # v*dt + v^2/(2*a) <= remaining distance.
        def braking_bound(distance):
            return np.sqrt((self.amax * dt) ** 2 + 2 * self.amax * np.maximum(distance, 0)) - self.amax * dt

        # The feedback-centred envelope can move TOWARDS an outstanding
        # command during a reversal. Static stopping distance alone is then
        # insufficient, even when every previous command was inside its
        # instantaneous envelope. Estimate motion only from measured frames
        # carried by committed proposals; command positions are not feedback.
        # Planned return can refresh a stationary command's clock after a
        # long geometry check. Its old measurement must keep its own clock.
        measured_stamp = state.diagnostics.get("feedback_motion_stamp_s")
        motion_dt = (float(stamp - measured_stamp)
                     if isinstance(measured_stamp, (int, float)) and np.isfinite(measured_stamp)
                     else None)
        feedback_motion_valid = bool("feedback_physical_rad" in state.diagnostics
                                     and motion_dt is not None and 0 < motion_dt <= self.max_dt)
        if feedback_motion_valid:
            previous_feedback = self._vector(state.diagnostics["feedback_physical_rad"])
            feedback_velocity = (feedback - previous_feedback) / motion_dt
            previous_feedback_velocity = self._vector(
                state.diagnostics.get("feedback_velocity_rad_s", np.zeros(7)))
        else:
            # Older externally restored states have no measurement history.
            # Keep the hard envelope/acceleration checks; do not invent motion.
            feedback_velocity = previous_feedback_velocity = np.zeros(7)
        # Retain the larger directional speed across two measured intervals.
        # A single quiet interval must not instantly erase reversal reserve.
        closing_lower = np.maximum.reduce((feedback_velocity, previous_feedback_velocity, np.zeros(7)))
        closing_upper = np.maximum.reduce((-feedback_velocity, -previous_feedback_velocity, np.zeros(7)))
        uncertainty = np.minimum(np.deg2rad(.05), .05 * self.lead)

        def moving_braking_bound(distance, closing):
            # Reserve one maximum accepted feedback interval H, followed by
            # command braking time v/a. With adverse feedback speed w:
            # v*H + v^2/(2*a) + w*(H + v/a) + uncertainty <= distance.
            # This is a measured-motion forecast, not a hardware deceleration
            # guarantee. Abrupt unobserved changes still use the hard guards.
            active = closing > 1e-9
            remaining = distance - closing * self.max_dt - uncertainty
            term = self.amax * self.max_dt + closing
            root = np.sqrt(term * term + 2 * self.amax * np.maximum(remaining, 0))
            # Rationalized root avoids cancellation near a shrinking bound.
            bound = 2 * self.amax * np.maximum(remaining, 0) / (root + term)
            return np.where(active, np.minimum(bound, self.vmax), self.vmax)

        motion_lower_speed = moving_braking_bound(q - feedback + self.lead, closing_lower)
        motion_upper_speed = moving_braking_bound(feedback + self.lead - q, closing_upper)
        vmax = np.minimum(self.vmax, self.step / dt)
        vlow = np.maximum.reduce((-vmax, velocity - self.amax * dt, -braking_bound(q - lo),
                                  -motion_lower_speed))
        vhigh = np.minimum.reduce((vmax, velocity + self.amax * dt, braking_bound(hi - q),
                                   motion_upper_speed))
        reserve_conflict = bool(np.any(vlow > vhigh + 1e-9))
        recovering = bool(state.diagnostics.get("braking_recovery", False))
        # Reserve feasibility is stronger than this frame's hard envelope.
        # On shrinkage, try only maximal braking, never an accelerating goal.
        # Stay in recovery until both stopped and reserve-feasible.
        recovery = bool(brake_only or reserve_conflict or (recovering and np.any(np.abs(velocity) > 1e-9)))
        if recovery:
            vlow = np.maximum.reduce((-vmax, velocity - self.amax * dt, (lo - q) / dt))
            vhigh = np.minimum.reduce((vmax, velocity + self.amax * dt, (hi - q) / dt))
        if np.any(vlow > vhigh + 1e-9):
            joints = np.flatnonzero(vlow > vhigh + 1e-9)
            details = {
                "failed_joints": (joints + 1).tolist(),
                "feedback_dt_s": dt,
                "feedback_stamp_s": float(stamp),
                "previous_command_stamp_s": state.stamp,
                "coordinate_frame": "SDK physical joints; rad and rad/s",
                "target_physical_rad": target.tolist(),
                "previous_command_physical_rad": q.tolist(),
                "feedback_physical_rad": feedback.tolist(),
                "previous_velocity_physical_rad_s": velocity.tolist(),
                "position_lower_rad": lo.tolist(),
                "position_upper_rad": hi.tolist(),
                "acceleration_velocity_lower_rad_s": (velocity - self.amax * dt).tolist(),
                "acceleration_velocity_upper_rad_s": (velocity + self.amax * dt).tolist(),
                "step_velocity_bound_rad_s": vmax.tolist(),
                "braking_velocity_lower_rad_s": (-braking_bound(q - lo)).tolist(),
                "braking_velocity_upper_rad_s": braking_bound(hi - q).tolist(),
                "feasible_velocity_lower_rad_s": vlow.tolist(),
                "feasible_velocity_upper_rad_s": vhigh.tolist(),
                "lead_rad": (q - feedback).tolist(),
                "lead_limit_rad": self.lead.tolist(),
                "feedback_motion_valid": feedback_motion_valid,
                "feedback_motion_dt_s": motion_dt,
                "feedback_velocity_rad_s": feedback_velocity.tolist(),
                "feedback_closing_lower_rad_s": closing_lower.tolist(),
                "feedback_closing_upper_rad_s": closing_upper.tolist(),
                "feedback_motion_velocity_lower_rad_s": (-motion_lower_speed).tolist(),
                "feedback_motion_velocity_upper_rad_s": motion_upper_speed.tolist(),
            }
            summary = "; ".join(
                f"J{i+1}: v_min={np.degrees(vlow[i]):+.2f} > "
                f"v_max={np.degrees(vhigh[i]):+.2f}deg/s, "
                f"lead={np.degrees(q[i]-feedback[i]):+.2f}deg" for i in joints)
            raise TrajectoryFault(
                "no acceleration-consistent command inside safety envelope "
                f"(dt={dt*1000:.1f}ms; {summary})", details)
        # Do not let roundoff produce an inverted interval.
        vhigh = np.maximum(vhigh, vlow)
        bounded_target = np.clip(target, lower, upper)
        error = bounded_target - q
        desired = np.sign(error) * np.minimum(np.abs(error) / dt, braking_bound(np.abs(error)))
        goal_velocity = np.clip(desired, vlow, vhigh)
        brake_velocity = np.clip(np.zeros(7), vlow, vhigh)
        if recovery:
            goal_velocity = brake_velocity.copy()

        collision_context = dict(
            feedback_stamp_s=float(stamp), feedback_dt_s=dt,
            previous_command_stamp_s=float(state.stamp),
            coordinate_frame="SDK physical joints; rad and rad/s",
            target_physical_rad=target.tolist(),
            previous_command_physical_rad=q.tolist(),
            previous_velocity_physical_rad_s=velocity.tolist(),
            feedback_physical_rad=feedback.tolist(),
        )
        if path_check is not None and not path_check(feedback, q):
            raise TrajectoryFault("outstanding command/feedback motion intersects torso safety envelope",
                                  dict(collision_context, fault_code="collision_rejected", collision_stage="outstanding_motion",
                                       checked_lower_physical_rad=np.minimum(feedback, q).tolist(),
                                       checked_upper_physical_rad=np.maximum(feedback, q).tolist()))
        selected = None
        collision_rejections = 0
        def validate(candidate_velocity):
            nonlocal collision_rejections
            candidate = q + dt * candidate_velocity
            if (not np.all(np.isfinite(candidate))
                    or np.any(candidate_velocity < vlow-1e-10)
                    or np.any(candidate_velocity > vhigh+1e-10)
                    or np.any(candidate < lo-1e-8) or np.any(candidate > hi+1e-8)):
                return None
            if path_check is not None:
                # Include independently lagging axes and the braking corridor.
                stop = candidate + candidate_velocity * self.max_dt + (
                    candidate_velocity * np.abs(candidate_velocity) / (2 * self.amax))
                vertices = np.array([feedback, q, candidate, stop])
                box_lower, box_upper = vertices.min(axis=0), vertices.max(axis=0)
                if not path_check(box_lower, box_upper):
                    collision_rejections += 1
                    collision_context.update(
                        collision_stage="feedback_command_braking_box",
                        candidate_not_sent_physical_rad=candidate.tolist(),
                        candidate_velocity_rad_s=candidate_velocity.tolist(),
                        stopping_physical_rad=stop.tolist(),
                        checked_lower_physical_rad=box_lower.tolist(),
                        checked_upper_physical_rad=box_upper.tolist())
                    return None
            return True

        # Backtracking is only needed when collision corridors are enabled.
        progress_values = ([2.0 ** -i for i in range(11)] + [0.0]
                           if path_check is not None else (1.0,))
        for progress in progress_values:
            candidate_velocity = brake_velocity + progress * (goal_velocity - brake_velocity)
            if validate(candidate_velocity):
                selected = q + dt*candidate_velocity, candidate_velocity, progress
                break
        if selected is None:
            raise TrajectoryFault("no joint candidate passes the collision/safety checks", dict(
                collision_context,
                fault_code=("collision_rejected" if collision_rejections else "joint_candidate_invalid"),
                collision_rejections=collision_rejections,
                feasible_velocity_lower_rad_s=vlow.tolist(),
                feasible_velocity_upper_rad_s=vhigh.tolist()))
        candidate, candidate_velocity, progress = selected
        lead = candidate - feedback
        pushing = (np.abs(lead) >= .9 * self.lead) & ((target - candidate) * lead > 0)
        # ``lead_timeout`` measures a stalled controller, not total time spent
        # near the lead envelope.  The commissioned controller can wait almost
        # two seconds after enable before it starts following.  Once measured
        # feedback has made meaningful progress toward the held command, start
        # a new no-progress window without moving that command farther away.
        lead_abs = np.abs(lead)
        try:
            feedback_anchor = self._vector(state.diagnostics["lead_watch_feedback_rad"])
        except (KeyError, TypeError, TrajectoryFault):
            feedback_anchor = feedback.copy()
        was_pushing = ~np.isnan(state.lead_since)
        progress_threshold = np.minimum(np.deg2rad(.1), .05 * self.lead)
        feedback_progress = feedback - feedback_anchor
        meaningful_progress = (
            pushing & was_pushing
            & (feedback_progress * lead > 0)
            & (np.abs(feedback_progress) >= progress_threshold)
        )
        restart_watch = pushing & (~was_pushing | meaningful_progress)
        since = np.where(
            pushing,
            np.where(restart_watch, stamp, state.lead_since),
            np.nan,
        )
        feedback_anchor = np.where(
            pushing,
            np.where(restart_watch, feedback, feedback_anchor),
            feedback,
        )
        if np.any(pushing & (stamp - since >= self.lead_timeout)):
            joints = (np.flatnonzero(pushing & (stamp - since >= self.lead_timeout)) + 1).tolist()
            raise TrajectoryFault(f"persistent feedback lead on joints {joints}", dict(
                fault_code="persistent_feedback_lead",
                failed_joints=joints, feedback_stamp_s=float(stamp), feedback_dt_s=dt,
                target_physical_rad=target.tolist(), previous_command_physical_rad=q.tolist(),
                candidate_not_sent_physical_rad=candidate.tolist(), feedback_physical_rad=feedback.tolist(),
                lead_rad=lead.tolist(), lead_limit_rad=self.lead.tolist(),
                lead_watch_feedback_rad=feedback_anchor.tolist(),
                lead_progress_from_anchor_rad=feedback_progress.tolist(),
                lead_progress_threshold_rad=progress_threshold.tolist(),
                lead_elapsed_s=np.where(pushing, stamp-since, 0).tolist()))
        diagnostics = {
            "collision_rejections": collision_rejections,
            "braking_recovery": recovery,
            "reserve_conflict": reserve_conflict,
            "elbow_waiting_for_bend_plane": bool(elbow_wait),
            "feedback_dt_s": dt,
            "feedback_physical_rad": feedback.tolist(),
            "feedback_motion_stamp_s": float(stamp),
            "feedback_motion_valid": feedback_motion_valid,
            "feedback_motion_dt_s": motion_dt,
            "feedback_velocity_rad_s": feedback_velocity.tolist(),
            "feedback_motion_braking_active": (
                ((closing_lower > 1e-9) & (desired < 0) & (-desired > motion_lower_speed + 1e-9)) |
                ((closing_upper > 1e-9) & (desired > 0) & (desired > motion_upper_speed + 1e-9))
            ).tolist(),
            "feedback_closing_lower_rad_s": closing_lower.tolist(),
            "feedback_closing_upper_rad_s": closing_upper.tolist(),
            "feedback_motion_velocity_lower_rad_s": (-motion_lower_speed).tolist(),
            "feedback_motion_velocity_upper_rad_s": motion_upper_speed.tolist(),
            "position_clipped": (np.abs(target - bounded_target) > 1e-9).tolist(),
            "velocity_bound_active": (np.abs(desired) > vmax).tolist(),
            "acceleration_bound_active": (np.abs(desired - velocity) > self.amax * dt).tolist(),
            "lead_braking_active": ((braking_bound(hi - q) < vmax) | (braking_bound(q - lo) < vmax)).tolist(),
            "collision_search_progress": progress,
            "lead_rad": lead.tolist(),
            "velocity_rad_s": candidate_velocity.tolist(),
            "lead_watch_feedback_rad": feedback_anchor.tolist(),
            "lead_progress_from_anchor_rad": feedback_progress.tolist(),
            "lead_progress_reset": meaningful_progress.tolist(),
            "lead_progress_threshold_rad": progress_threshold.tolist(),
            "lead_saturation_elapsed_s": np.where(pushing, stamp - since, 0).tolist(),
            "configured_speed_bound_active": (np.abs(desired) > self.vmax + 1e-9).tolist(),
            "step_bound_active": (np.abs(desired) > self.step / dt + 1e-9).tolist(),
            "directional_lead_braking_active": (
                ((desired > 0) & (desired > braking_bound(feedback+self.lead-q) + 1e-9)) |
                ((desired < 0) & (-desired > braking_bound(q-feedback+self.lead) + 1e-9))
            ).tolist(),
            "position_braking_active": (
                ((desired > 0) & (desired > braking_bound(upper-q) + 1e-9)) |
                ((desired < 0) & (-desired > braking_bound(q-lower) + 1e-9))
            ).tolist(),
            "desired_velocity_rad_s": desired.tolist(),
            "feasible_velocity_lower_rad_s": vlow.tolist(),
            "feasible_velocity_upper_rad_s": vhigh.tolist(),
        }
        return TrajectoryStep(candidate, candidate_velocity, float(stamp), since, diagnostics)

    def commit(self, step):
        self.state = step
