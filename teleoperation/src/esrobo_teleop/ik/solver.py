"""URDF IK for the two NERO arms.

Partitioned arm control jointly fits elbow and hand-base positions with bounded
least squares, holding terminal joints inside the solve. Other modes retain the
Pink task-velocity solver. Only the 14 arm targets are returned; the model uses
the fixed waist/head configuration.
"""

from __future__ import annotations

import numpy as np

from ..config import IkConfig


class IkSolver:
    """Inverse-kinematics solver that returns 14 arm joint targets (rad)."""

    ARM_JOINT_NAMES = [
        "leftArm_joint", "left_nero_joint2", "left_nero_joint3", "left_nero_joint4",
        "left_nero_joint5", "left_nero_joint6", "left_nero_joint7",
        "rightArm_joint", "right_nero_joint2", "right_nero_joint3", "right_nero_joint4",
        "right_nero_joint5", "right_nero_joint6", "right_nero_joint7",
    ]

    def __init__(self, cfg: IkConfig):
        self._cfg = cfg
        import pinocchio as pin

        self._pin = pin
        self._model = pin.buildModelFromUrdf(cfg.urdf_path)
        self._data = self._model.createData()
        self._config = None

        # Resolve the 14 controlled arm joints to their pinocchio q indices.
        self._arm_q_indices: list[int] = []
        self._arm_v_indices: list[int] = []
        for joint_name in cfg.left_arm_joints + cfg.right_arm_joints:
            joint_id = self._model.getJointId(joint_name)
            self._arm_q_indices.append(self._model.idx_qs[joint_id])
            self._arm_v_indices.append(self._model.idx_vs[joint_id])
        self._arm_q_indices = np.asarray(self._arm_q_indices, dtype=int)
        self._arm_v_indices = np.asarray(self._arm_v_indices, dtype=int)
        self._all_q_indices = np.arange(self._model.nq, dtype=int)

        self._task_frames = [cfg.left_hand_frame, cfg.right_hand_frame]
        self._task_frames = [cfg.left_hand_frame, cfg.right_hand_frame]
        if cfg.enable_elbow_tasks:
            self._task_frames = [
                cfg.left_elbow_frame, cfg.left_hand_frame,
                cfg.right_elbow_frame, cfg.right_hand_frame,
            ]

        # Lower/upper limits for the 14 arm joints (rad).
        lower = []
        upper = []
        for joint_name in cfg.left_arm_joints + cfg.right_arm_joints:
            joint_id = self._model.getJointId(joint_name)
            lower.append(float(self._model.lowerPositionLimit[self._model.idx_qs[joint_id]]))
            upper.append(float(self._model.upperPositionLimit[self._model.idx_qs[joint_id]]))
        self._arm_lower = np.asarray(lower, dtype=np.float64)
        self._arm_upper = np.asarray(upper, dtype=np.float64)

        self._prev_targets: np.ndarray | None = None
        self._solve_failure_active = False
        self.last_solution_valid = True
        self.solution_diagnostics = {}
        # Elbow-flexion angle at the start of the current arming session, per
        # side. The J3 observability gate compares against this instead of an
        # absolute angle, since a human calibration pose is never perfectly
        # straight (see initialize_arm_session).
        self._j3_flexion_baseline: dict[str, float | None] = {"left": None, "right": None}
        # Freeze the legacy seed relationship between segment bend and J4 at
        # the start of each armed session.  The configured wrist task frame is
        # the hand base, so held J5-J7 offsets can make the shoulder/elbow/hand
        # angle non-monotonic around a nearly straight arm.  Re-anchoring this
        # relationship to live feedback on every cycle creates positive
        # feedback: as J4 follows, the next J4 target moves farther away.
        self._j4_flexion_reference: dict[str, tuple[float, float] | None] = {
            "left": None,
            "right": None,
        }
        self._segment_bend_baseline = {"left": None, "right": None}
        self._position_data = self._model.createData()
        self._position_lower = self._arm_lower.copy()
        self._position_upper = self._arm_upper.copy()
        margin_deg = float(cfg.position_limit_hold_feedback_margin_deg)
        if not np.isfinite(margin_deg) or not 0.0 < margin_deg <= 1.0:
            raise ValueError("position-limit hold feedback margin must be in (0, 1] deg")
        self._position_limit_hold_feedback_margin = np.deg2rad(margin_deg)
        recovery_error = float(cfg.position_limit_recovery_error_m)
        recovery_frames = int(cfg.position_limit_recovery_frames)
        if (not np.isfinite(recovery_error) or not 0.0 < recovery_error < 0.03
                or recovery_frames < 1):
            raise ValueError("position-limit recovery requires error in (0, .03) m and frames >= 1")

    def configure_position_envelope(self, side, lower, upper):
        """Use the driver's commissioned URDF envelope inside partitioned IK."""
        if side not in ("left", "right"):
            raise ValueError("invalid arm side")
        lower, upper = np.asarray(lower, dtype=float), np.asarray(upper, dtype=float)
        if (lower.shape != (7,) or upper.shape != (7,)
                or not np.all(np.isfinite([lower, upper])) or np.any(lower >= upper)):
            raise ValueError("invalid IK position envelope")
        offset = 0 if side == "left" else 7
        sl = slice(offset, offset + 7)
        lo, hi = np.maximum(self._arm_lower[sl], lower), np.minimum(self._arm_upper[sl], upper)
        if np.any(lo >= hi):
            raise ValueError("IK position envelope does not intersect URDF limits")
        self._position_lower[sl], self._position_upper[sl] = lo, hi

    def _report_solve_failure(self) -> None:
        """Report one state transition without flooding the operator terminal."""
        if self._solve_failure_active:
            return
        self._solve_failure_active = True
        print(
            "[ik] solve unavailable; retaining the last accepted command. "
            "Repeated solve messages are suppressed until recovery.",
            flush=True,
        )

    def _report_solve_recovery(self) -> None:
        if not self._solve_failure_active:
            return
        self._solve_failure_active = False
        print("[ik] solve recovered.", flush=True)

    def _make_config(self, arm_joint_pos: np.ndarray):
        from pink import Configuration

        full_q = np.zeros(self._model.nq, dtype=np.float64)
        full_q[self._arm_q_indices] = np.clip(
            arm_joint_pos, self._arm_lower, self._arm_upper
        )
        if self._config is None:
            self._config = Configuration(self._model, self._data, full_q.copy())
        self._config.update(full_q.copy())
        return self._config

    def _project_configuration_to_limits(self, config) -> None:
        """Keep every numerical IK iterate inside the URDF position limits."""
        q = np.asarray(config.q, dtype=np.float64).copy()
        lower = np.asarray(self._model.lowerPositionLimit, dtype=np.float64)
        upper = np.asarray(self._model.upperPositionLimit, dtype=np.float64)
        bounded = np.isfinite(lower) & np.isfinite(upper) & (lower <= upper)
        q[bounded] = np.clip(q[bounded], lower[bounded], upper[bounded])
        config.update(q)

    def _make_tasks(self, partitioned_side: str | None = None):
        from pink.tasks import FrameTask, PostureTask

        tasks = []
        active_hand_frame = (
            self._cfg.left_hand_frame if partitioned_side == "left"
            else self._cfg.right_hand_frame if partitioned_side == "right"
            else None
        )
        for frame in self._task_frames:
            if frame.endswith("Hand_link"):
                orientation_cost = (
                    0.0 if frame == active_hand_frame else self._cfg.orientation_cost
                )
                cost = (self._cfg.position_cost, orientation_cost)
                lm = self._cfg.lm_damping
            else:
                cost = (self._cfg.elbow_position_cost, self._cfg.elbow_orientation_cost)
                lm = self._cfg.elbow_lm_damping
            tasks.append(
                FrameTask(
                    frame,
                    position_cost=cost[0],
                    orientation_cost=cost[1],
                    lm_damping=lm,
                    gain=self._cfg.gain,
                )
            )
        # Null-space posture keeps the base arm joints from drifting.
        posture = PostureTask(
            cost=0.001,
            lm_damping=0.5,
            gain=0.001,
        )
        tasks.append(posture)
        return tasks, posture

    def _joint_velocity_lock(self, velocity_indices: np.ndarray):
        """Build a hard zero-velocity task for selected Pinocchio DoFs."""
        from pink.tasks import Task

        indices = np.unique(np.asarray(velocity_indices, dtype=int))
        nv = int(self._model.nv)

        class JointVelocityLock(Task):
            def __init__(self):
                super().__init__(cost=None, gain=1.0, lm_damping=0.0)

            def compute_error(self, _configuration):
                return np.zeros(indices.shape[0], dtype=np.float64)

            def compute_jacobian(self, _configuration):
                jacobian = np.zeros((indices.shape[0], nv), dtype=np.float64)
                jacobian[np.arange(indices.shape[0]), indices] = 1.0
                return jacobian

            def __repr__(self):
                return f"JointVelocityLock(indices={indices.tolist()})"

        return JointVelocityLock()

    def _target_se3(self, pose: np.ndarray):
        from ..math_utils import quat_wxyz_to_matrix

        rotation = quat_wxyz_to_matrix(np.asarray(pose[3:7], dtype=np.float64))
        return self._pin.SE3(rotation, np.asarray(pose[0:3], dtype=np.float64))

    def _segment_elbow_flexion(
        self,
        side: str,
        current_arm_joint_pos: np.ndarray,
        elbow_pose: np.ndarray | None,
        wrist_pose: np.ndarray | None,
    ) -> float | None:
        """Return NERO J4 flexion from the target upper/forearm segment angle."""
        if elbow_pose is None or wrist_pose is None:
            return None
        elbow = np.asarray(elbow_pose[:3], dtype=np.float64)
        wrist = np.asarray(wrist_pose[:3], dtype=np.float64)
        if elbow.shape != (3,) or wrist.shape != (3,):
            return None
        current_poses = self.current_task_frame_poses(current_arm_joint_pos)
        shoulder = np.asarray(current_poses[f"{side}_shoulder"][:3], dtype=np.float64)

        def segment_angle(a: np.ndarray, b: np.ndarray, c: np.ndarray) -> float | None:
            upper = b - a
            forearm = c - b
            denominator = float(np.linalg.norm(upper) * np.linalg.norm(forearm))
            if denominator < 1.0e-12:
                return None
            cosine = float(np.clip(np.dot(upper, forearm) / denominator, -1.0, 1.0))
            return float(np.arccos(cosine))

        target_angle = segment_angle(shoulder, elbow, wrist)
        if target_angle is None:
            return None
        j4_index = (0 if side == "left" else 7) + 3
        reference = getattr(self, "_j4_flexion_reference", {}).get(side)
        if reference is None:
            # Preview/backward-compatible path before an arm session is
            # initialized.  Live following always installs a fixed reference
            # in initialize_arm_session().
            current_elbow = np.asarray(
                current_poses[f"{side}_elbow"][:3], dtype=np.float64
            )
            current_wrist = np.asarray(
                current_poses[f"{side}_wrist"][:3], dtype=np.float64
            )
            reference_angle = segment_angle(shoulder, current_elbow, current_wrist)
            if reference_angle is None:
                return None
            reference_j4 = float(current_arm_joint_pos[j4_index])
        else:
            reference_j4, reference_angle = reference
        # Apply the Cartesian bend change to the J4 value measured once at
        # session start.  Do not re-anchor to delayed feedback: with a held
        # terminal-wrist offset, the hand-base segment angle has an absolute-
        # value branch near extension and live re-anchoring makes J4 run away.
        flexion = float(reference_j4 + target_angle - reference_angle)
        limited = float(np.clip(flexion, self._arm_lower[j4_index], self._arm_upper[j4_index]))
        if not hasattr(self, "_last_elbow_solve_diagnostics"):
            self._last_elbow_solve_diagnostics = {}
        self._last_elbow_solve_diagnostics[side] = {
            "geometric_j4_deg": float(np.degrees(flexion)),
            "soft_limited_j4_deg": float(np.degrees(limited)),
            "soft_limit_active": abs(limited - flexion) > 1e-6,
        }
        return limited

    def initialize_arm_session(
        self, side: str, measured: np.ndarray, human_reference_flexion: float,
        *, relative_elbow_reference: bool = False,
    ) -> None:
        """Anchor J3 observability and J4 flexion to one measured startup frame."""
        if side not in ("left", "right"):
            raise ValueError("side must be left or right")
        measured = np.asarray(measured, dtype=np.float64)
        if measured.shape != (14,) or not np.all(np.isfinite(measured)):
            raise ValueError("session needs fourteen finite measured joints")
        angle = float(human_reference_flexion)
        if not np.isfinite(angle) or not 0.0 <= angle <= np.pi:
            raise ValueError("invalid calibrated elbow flexion")
        poses = self.current_task_frame_poses(measured)
        upper = poses[f"{side}_elbow"][:3] - poses[f"{side}_shoulder"][:3]
        forearm = poses[f"{side}_wrist"][:3] - poses[f"{side}_elbow"][:3]
        robot_angle = np.arccos(np.clip(
            np.dot(upper, forearm) / (np.linalg.norm(upper) * np.linalg.norm(forearm)),
            -1.0, 1.0,
        ))
        offset = 0 if side == "left" else 7
        self._j3_flexion_baseline[side] = float(np.clip(
            measured[offset + 3] + (0.0 if relative_elbow_reference else angle - robot_angle),
            self._arm_lower[offset + 3], self._arm_upper[offset + 3],
        ))
        if not hasattr(self, "_j4_flexion_reference"):
            self._j4_flexion_reference = {"left": None, "right": None}
        self._j4_flexion_reference[side] = (
            float(measured[offset + 3]),
            float(robot_angle),
        )
        self._segment_bend_baseline[side] = float(
            robot_angle if relative_elbow_reference else angle
        )
        if self._prev_targets is None:
            self._prev_targets = measured.copy()
        else:
            self._prev_targets[offset:offset + 7] = measured[offset:offset + 7]

    def reset_j3_observability_baseline(self, side: str) -> None:
        """Clear a side's J3/J4 session references before live arming."""
        if side not in ("left", "right"):
            raise ValueError("side must be 'left' or 'right'")
        self._j3_flexion_baseline[side] = None
        if hasattr(self, "_j4_flexion_reference"):
            self._j4_flexion_reference[side] = None
        if hasattr(self, "_segment_bend_baseline"):
            self._segment_bend_baseline[side] = None

    def _solve_partitioned_positions(self, side, measured, elbow_pose, wrist_pose, terminal):
        """Fit both URDF endpoints together, keeping all constraints in the solve.

        Hand_link is downstream of J5-J7. Its unsigned segment bend is not a
        signed J4 angle, particularly with a bent wrist or negative startup J4.
        J4 must therefore remain a variable; editing J3 after solving would
        likewise invalidate the endpoint geometry.
        """
        from scipy.optimize import least_squares

        offset = 0 if side == "left" else 7
        terminal = np.asarray(terminal, dtype=float)
        desired = np.array([elbow_pose[:3], wrist_pose[:3]], dtype=float)
        if (measured.shape != (14,) or terminal.shape != (3,)
                or desired.shape != (2, 3)
                or not all(np.all(np.isfinite(v)) for v in (measured, terminal, desired))):
            raise ValueError("partitioned IK requires finite joints, endpoints and terminal targets")
        base = measured.copy()
        if self._prev_targets is not None:
            base[offset:offset + 4] = self._prev_targets[offset:offset + 4]
        base[offset + 4:offset + 7] = terminal
        if (np.any(terminal < self._position_lower[offset + 4:offset + 7])
                or np.any(terminal > self._position_upper[offset + 4:offset + 7])):
            self.solution_diagnostics = {"reason": "terminal target outside position envelope"}
            self._report_solve_failure()
            return measured.copy()

        poses = self.current_task_frame_poses(measured)
        upper = desired[0] - poses[f"{side}_shoulder"][:3]
        forearm = desired[1] - desired[0]
        length = np.linalg.norm(upper) * np.linalg.norm(forearm)
        if length < 1e-12:
            self.solution_diagnostics = {"reason": "degenerate target arm segments"}
            self._report_solve_failure()
            return measured.copy()
        bend = float(np.arccos(np.clip(np.dot(upper, forearm) / length, -1., 1.)))
        requested_bend = bend
        requested_positions = desired.copy()
        # Retargeted human flexion can exceed the commissioned J4 range. Bound
        # the Cartesian request BEFORE solving, using the actual held wrist
        # geometry at the positive flexion limit (not bend == J4). Preserve
        # upper-arm direction, forearm length and the requested bend plane.
        probe = base.copy()
        probe[offset + 3] = self._position_upper[offset + 3]
        boundary = self.current_task_frame_poses(probe)
        boundary_upper = boundary[f"{side}_elbow"][:3] - boundary[f"{side}_shoulder"][:3]
        boundary_forearm = boundary[f"{side}_wrist"][:3] - boundary[f"{side}_elbow"][:3]
        bend_limit = float(np.arccos(np.clip(
            np.dot(boundary_upper, boundary_forearm)
            / (np.linalg.norm(boundary_upper) * np.linalg.norm(boundary_forearm)), -1., 1.)))
        workspace_limited = bend > bend_limit + 1e-7
        if workspace_limited:
            axis = upper / np.linalg.norm(upper)
            plane = forearm - axis * np.dot(axis, forearm)
            if np.linalg.norm(plane) < 1e-9:
                self.solution_diagnostics = {"reason": "unobservable bend plane at workspace boundary"}
                self._report_solve_failure()
                return measured.copy()
            desired[1] = desired[0] + np.linalg.norm(forearm) * (
                axis * np.cos(bend_limit) + plane / np.linalg.norm(plane) * np.sin(bend_limit))
            bend = bend_limit
        baseline = self._segment_bend_baseline.get(side)
        additional = max(0., bend - (baseline or 0.))
        # The bend plane becomes observable from the actual segment angle, not
        # from motion relative to the startup pose.  Relative gating froze J3
        # for a clearly bent PICO forearm whenever calibration had a large bend.
        freeze_j3 = bend <= np.deg2rad(self._cfg.j3_observability_start_flexion_deg)
        if not freeze_j3 and self._cfg.partition_elbow_flexion_from_segments:
            # Only an initial guess for the flexed anatomical branch, never a
            # locked angle. Preserve continuity once a flexed solution exists.
            if abs(base[offset + 3]) < np.deg2rad(4.):
                guess = self._segment_elbow_flexion(side, measured, elbow_pose, wrist_pose)
                if guess is not None:
                    base[offset + 3] = guess
        indices = np.array([offset, offset + 1, offset + 3] if freeze_j3
                           else list(range(offset, offset + 4)))
        if freeze_j3 and not self._position_lower[offset + 2] <= base[offset + 2] <= self._position_upper[offset + 2]:
            self.solution_diagnostics = {"reason": "frozen J3 outside position envelope"}
            self._report_solve_failure()
            return measured.copy()
        # The retargeter already blends the Cartesian plane through 4..10 deg.
        # Solve that target directly once observable; never blend J3 again.
        frame_ids = [self._model.getFrameId(getattr(self._cfg, f"{side}_{name}"))
                     for name in ("elbow_frame", "hand_frame")]
        weights = np.array([self._cfg.elbow_position_cost, self._cfg.position_cost])[:, None]
        data = self._position_data

        def configuration(x):
            joints = base.copy()
            joints[indices] = x
            q = np.zeros(self._model.nq)
            q[self._arm_q_indices] = joints
            return q

        def residual(x):
            q = configuration(x)
            self._pin.forwardKinematics(self._model, data, q)
            self._pin.updateFramePlacements(self._model, data)
            points = np.array([data.oMf[i].translation.copy() for i in frame_ids])
            return ((points - desired) * weights).ravel()

        def jacobian(x):
            q = configuration(x)
            blocks = [self._pin.computeFrameJacobian(
                self._model, data, q, frame, self._pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
            )[:3, self._arm_v_indices[indices]] * weight
                for frame, weight in zip(frame_ids, weights[:, 0])]
            return np.vstack(blocks)

        try:
            result = least_squares(
                residual, np.clip(base[indices], self._position_lower[indices], self._position_upper[indices]),
                jac=jacobian, bounds=(self._position_lower[indices], self._position_upper[indices]),
                max_nfev=self._cfg.partition_position_max_evaluations,
                ftol=1e-9, xtol=1e-9, gtol=1e-9,
            )
            target = base.copy()
            target[indices] = result.x
            errors = np.linalg.norm(residual(result.x).reshape(2, 3) / weights, axis=1)
            attempts = 1
            evaluations = result.nfev
            if not freeze_j3 and np.max(errors) > .03:
                # Bounded local IK can settle on the opposite elbow branch.
                # Try each side of the bend plane without relaxing any bound.
                for angle in (-2., 2.):
                    seed = base[indices].copy()
                    seed[2], seed[3] = angle, max(bend, .1)
                    candidate = least_squares(
                        residual, np.clip(seed, self._position_lower[indices], self._position_upper[indices]),
                        jac=jacobian, bounds=(self._position_lower[indices], self._position_upper[indices]),
                        max_nfev=self._cfg.partition_position_max_evaluations,
                        ftol=1e-9, xtol=1e-9, gtol=1e-9,
                    )
                    attempts += 1
                    evaluations += candidate.nfev
                    candidate_errors = np.linalg.norm(residual(candidate.x).reshape(2, 3) / weights, axis=1)
                    if np.max(candidate_errors) < np.max(errors):
                        result, errors = candidate, candidate_errors
                        target[indices] = result.x
                    if np.max(errors) <= .03:
                        break
        except Exception as exc:
            self.solution_diagnostics = {"reason": "partitioned position solve failed",
                                         "exception": str(exc)}
            self._report_solve_failure()
            return measured.copy()
        active_bounds = []
        bound_tolerance = 1.0e-6
        for index in indices:
            joint = int(index - offset)
            if abs(target[index] - self._position_lower[index]) <= bound_tolerance:
                bound = "lower"
                limit = self._position_lower[index]
            elif abs(target[index] - self._position_upper[index]) <= bound_tolerance:
                bound = "upper"
                limit = self._position_upper[index]
            else:
                continue
            feedback_distance = abs(measured[index] - limit)
            active_bounds.append({
                "joint": joint + 1,
                "bound": bound,
                "limit_rad": float(limit),
                "candidate_rad": float(target[index]),
                "feedback_rad": float(measured[index]),
                "feedback_distance_deg": float(np.degrees(feedback_distance)),
                "feedback_at_bound": bool(
                    feedback_distance <= self._position_limit_hold_feedback_margin
                ),
            })
        recoverable_position_limit = bool(
            np.all(np.isfinite(errors))
            and np.max(errors) > .03
            and active_bounds
            and all(item["feedback_at_bound"] for item in active_bounds)
        )
        self.solution_diagnostics = {
            "elbow_error_m": float(errors[0]), "wrist_error_m": float(errors[1]),
            "method": "bounded_urdf_positions", "evaluations": evaluations, "attempts": attempts,
            "j3_frozen": bool(freeze_j3), "target_bend_deg": float(np.degrees(bend)),
            "additional_bend_deg": float(np.degrees(additional)),
            "candidate_urdf_rad": target[offset:offset + 7].tolist(),
            "workspace_limited": bool(workspace_limited),
            "requested_bend_deg": float(np.degrees(requested_bend)),
            "bend_limit_deg": float(np.degrees(bend_limit)),
            "j4_upper_limit_deg": float(np.degrees(self._position_upper[offset + 3])),
            "requested_positions_m": requested_positions.tolist(),
            "effective_positions_m": desired.tolist(),
            "workspace_wrist_shift_m": float(np.linalg.norm(desired[1] - requested_positions[1])),
            "active_position_bounds": active_bounds,
            "recoverable_position_limit": recoverable_position_limit,
        }
        self._last_elbow_solve_diagnostics = getattr(self, "_last_elbow_solve_diagnostics", {})
        self._last_elbow_solve_diagnostics[side] = {
            "method": "bounded_urdf_positions", "solved_j4_deg": float(np.degrees(target[offset + 3])),
            "target_segment_bend_deg": float(np.degrees(bend)),
            "workspace_limited": bool(workspace_limited),
            "requested_segment_bend_deg": float(np.degrees(requested_bend)),
            "segment_bend_limit_deg": float(np.degrees(bend_limit)),
        }
        if not np.all(np.isfinite(errors)) or np.max(errors) > .03:
            self._report_solve_failure()
            if recoverable_position_limit and self._prev_targets is not None:
                # Keep the last valid IK target available for diagnostics. The
                # driver receives an explicit brake/hold request instead of
                # this target while the human request remains unreachable.
                return self._prev_targets.copy()
            return measured.copy()
        self.last_solution_valid = True
        self._prev_targets = target.copy()
        self._report_solve_recovery()
        return target

    @staticmethod
    def _stabilized_j3_target(
        raw_target: float,
        anchor: float,
        elbow_flexion: float,
        gain: float,
        start_flexion_deg: float,
        full_flexion_deg: float,
    ) -> float:
        """Apply J3 morphology gain only where the elbow bend plane is observable."""
        start = max(0.0, float(np.deg2rad(start_flexion_deg)))
        full = max(start, float(np.deg2rad(full_flexion_deg)))
        flexion = abs(float(elbow_flexion))
        if flexion <= start:
            visibility = 0.0
        elif full <= start or flexion >= full:
            visibility = 1.0
        else:
            ratio = (flexion - start) / (full - start)
            visibility = ratio * ratio * (3.0 - 2.0 * ratio)
        compensated = float(raw_target) * max(0.0, float(gain))
        return float(anchor + visibility * (compensated - anchor))

    def current_task_frame_poses(self, arm_joint_pos: np.ndarray) -> dict[str, np.ndarray]:
        """Return measured-configuration elbow/wrist poses for startup rebasing."""
        from ..math_utils import matrix_to_quat_wxyz

        config = self._make_config(np.asarray(arm_joint_pos, dtype=np.float64))
        self._pin.forwardKinematics(self._model, self._data, config.q)
        self._pin.updateFramePlacements(self._model, self._data)
        frames = {
            "left_shoulder": self._cfg.left_shoulder_frame,
            "left_elbow": self._cfg.left_elbow_frame,
            "left_wrist": self._cfg.left_hand_frame,
            "right_shoulder": self._cfg.right_shoulder_frame,
            "right_elbow": self._cfg.right_elbow_frame,
            "right_wrist": self._cfg.right_hand_frame,
        }
        poses = {}
        for key, frame_name in frames.items():
            placement = self._data.oMf[self._model.getFrameId(frame_name)]
            poses[key] = np.concatenate(
                (placement.translation, matrix_to_quat_wxyz(placement.rotation))
            ).astype(np.float32)
        return poses

    def make_command_fk(self, side):
        """Independent FK workspace for the driver's command safety checks."""
        if side not in ("left", "right"):
            raise ValueError("invalid arm side")
        import threading
        lock = threading.Lock()
        data = self._model.createData()
        frames = [getattr(self._cfg, f"{side}_elbow_frame"), getattr(self._cfg, f"{side}_hand_frame")]
        ids = [self._model.getFrameId(name) for name in frames]

        def fk(full_urdf):
            joints = np.asarray(full_urdf, dtype=float)
            if joints.shape != (14,) or not np.all(np.isfinite(joints)):
                raise ValueError("invalid command FK joints")
            q = np.zeros(self._model.nq)
            q[self._arm_q_indices] = joints  # No clipping: inspect the actual command.
            with lock:
                self._pin.forwardKinematics(self._model, data, q)
                self._pin.updateFramePlacements(self._model, data)
                return np.array([data.oMf[index].translation.copy() for index in ids])
        return fk

    def wrist_orientation_linearization(
        self, side: str, arm_joint_pos: np.ndarray, joint_indices: tuple[int, ...]
    ) -> tuple[np.ndarray, np.ndarray]:
        """Return wrist world rotation and local angular axes for terminal arm joints."""
        if side not in ("left", "right"):
            raise ValueError("side must be 'left' or 'right'")
        joints = np.asarray(arm_joint_pos, dtype=np.float64).reshape(-1)
        indices = np.asarray(joint_indices, dtype=int).reshape(-1)
        if joints.shape != (14,) or indices.shape[0] not in (2, 3):
            raise ValueError("expected 14 arm joints and two or three local joint indices")
        if len(set(indices.tolist())) != indices.shape[0] or np.any(indices < 0) or np.any(indices >= 7):
            raise ValueError("wrist joint indices must be distinct values in 0..6")

        from ..math_utils import quat_wxyz_to_matrix, rotation_matrix_to_rotvec

        frame_key = f"{side}_wrist"
        base_pose = self.current_task_frame_poses(joints)[frame_key]
        base_rotation = quat_wxyz_to_matrix(base_pose[3:]).astype(np.float64)
        side_offset = 0 if side == "left" else 7
        epsilon = 1.0e-4
        columns = []
        for local_index in indices:
            perturbed = joints.copy()
            perturbed[side_offset + local_index] += epsilon
            pose = self.current_task_frame_poses(perturbed)[frame_key]
            rotation = quat_wxyz_to_matrix(pose[3:]).astype(np.float64)
            local_delta = base_rotation.T @ rotation
            columns.append(rotation_matrix_to_rotvec(local_delta) / epsilon)
        return base_rotation, np.column_stack(columns).astype(np.float64)

    def solve(
        self,
        left_wrist_pose: np.ndarray,
        right_wrist_pose: np.ndarray,
        current_arm_joint_pos: np.ndarray,
        left_elbow_pose: np.ndarray | None = None,
        right_elbow_pose: np.ndarray | None = None,
        partitioned_side: str | None = None,
        terminal_joint_targets: np.ndarray | None = None,
    ) -> np.ndarray:
        """Solve IK and return the 14 arm joint targets (rad).

        ``current_arm_joint_pos`` must be the current measured joint angles of
        the 14 arm joints in the same order as :attr:`ARM_JOINT_NAMES`.
        """
        from pink import solve_ik

        self.last_solution_valid = False
        self.solution_diagnostics = {}

        if partitioned_side not in (None, "left", "right"):
            raise ValueError("partitioned_side must be None, 'left', or 'right'")
        if partitioned_side is not None and not self._cfg.partition_terminal_wrist_ik:
            partitioned_side = None
        measured = np.asarray(current_arm_joint_pos, dtype=np.float64).copy()
        if partitioned_side is not None and self._cfg.enable_elbow_tasks:
            return self._solve_partitioned_positions(
                partitioned_side, measured,
                left_elbow_pose if partitioned_side == "left" else right_elbow_pose,
                left_wrist_pose if partitioned_side == "left" else right_wrist_pose,
                terminal_joint_targets,
            )
        solve_seed = measured.copy()
        max_lead = max(0.0, float(self._cfg.max_command_lead))
        # Full-arm geometry and executable trajectory are different objects.
        # Clipping J3 independently after solving can reverse the forearm's
        # direction while preserving J4 (the 2026-09-18 torso collision).
        complete_geometry = partitioned_side is not None
        if complete_geometry:
            max_lead = 0.0
            if self._prev_targets is not None:
                solve_seed = np.asarray(self._prev_targets, dtype=float).copy()
            offset = 0 if partitioned_side == "left" else 7
            opposite = 7 if offset == 0 else 0
            solve_seed[opposite:opposite + 7] = measured[opposite:opposite + 7]
            solve_seed[offset + 4:offset + 7] = terminal_joint_targets
        if self._prev_targets is not None and max_lead > 0.0:
            previous = np.asarray(self._prev_targets, dtype=np.float64)
            if previous.shape == measured.shape and np.all(np.isfinite(previous)):
                solve_seed = np.clip(previous, measured - max_lead, measured + max_lead)
        lock_partitioned_j4 = False
        if partitioned_side is not None and self._cfg.partition_elbow_flexion_from_segments:
            elbow_pose = left_elbow_pose if partitioned_side == "left" else right_elbow_pose
            wrist_pose = left_wrist_pose if partitioned_side == "left" else right_wrist_pose
            flexion = self._segment_elbow_flexion(
                partitioned_side, measured, elbow_pose, wrist_pose
            )
            if flexion is not None:
                side_offset = 0 if partitioned_side == "left" else 7
                solve_seed[side_offset + 3] = flexion
                lock_partitioned_j4 = True

        # Seed J4 with geometric elbow flexion before solving shoulder direction.
        config = self._make_config(solve_seed)
        tasks, posture = self._make_tasks(partitioned_side)
        posture.set_target(config.q)

        # Set frame targets.
        target_poses = [
            (left_elbow_pose, 0),
            (left_wrist_pose, 1),
            (right_elbow_pose, 2),
            (right_wrist_pose, 3),
        ]
        for pose, task_index in target_poses:
            if pose is None or task_index >= len(self._task_frames):
                continue
            task = tasks[task_index]
            task.set_target(self._target_se3(pose))
        # If elbow tasks disabled, only first two tasks (left/right hand).
        if not self._cfg.enable_elbow_tasks:
            for task, pose in zip(tasks[:2], (left_wrist_pose, right_wrist_pose)):
                task.set_target(self._target_se3(pose))

        try:
            # The URDF contains waist/head joints that this controller never
            # commands. Lock them in every solve. In single-arm partitioned
            # mode also lock the opposite arm and active J5..J7, leaving only
            # the anatomical shoulder/elbow chain J1..J4 for PICO positions.
            locked = np.setdiff1d(np.arange(self._model.nv), self._arm_v_indices)
            if partitioned_side is not None:
                active_offset = 0 if partitioned_side == "left" else 7
                opposite_offset = 7 if partitioned_side == "left" else 0
                terminal_lock_start = 3 if lock_partitioned_j4 else 4
                locked = np.concatenate(
                    (
                        locked,
                        self._arm_v_indices[opposite_offset:opposite_offset + 7],
                        self._arm_v_indices[
                            active_offset + terminal_lock_start:active_offset + 7
                        ],
                    )
                )
            lock_constraint = self._joint_velocity_lock(locked)
            iterations = max(1, int(self._cfg.iterations_per_cycle))
            for _ in range(iterations):
                velocity = solve_ik(
                    config,
                    tasks,
                    self._cfg.dt,
                    solver="proxqp",
                    limits=[],
                    constraints=[lock_constraint],
                )
                config.integrate_inplace(velocity, self._cfg.dt)
                # Locked joints can still drift by a few microradians through
                # numerical constraint tolerance, while active joints can
                # overshoot a boundary before the next Pink iteration. Pink
                # rejects either case at the next solve, so project each
                # internal iterate before continuing.
                self._project_configuration_to_limits(config)
        except Exception:  # noqa: BLE001
            self._report_solve_failure()
            return current_arm_joint_pos.copy()

        targets = config.q[self._arm_q_indices].copy()
        if partitioned_side is not None:
            terminal = np.asarray(terminal_joint_targets, dtype=np.float64).reshape(-1)
            if terminal.shape != (3,) or not np.all(np.isfinite(terminal)):
                raise ValueError("partitioned IK requires three finite terminal joint targets")
            offset = 0 if partitioned_side == "left" else 7
            opposite = 7 if offset == 0 else 0
            targets[opposite:opposite + 7] = measured[opposite:opposite + 7]
            targets[offset + 4:offset + 7] = terminal

            # PICO positions do not define rotation about a nearly straight arm.
            # Hold J3 in that region, then fade in the calibrated amplitude as
            # elbow flexion makes the bend plane geometrically observable.
            anchor = measured[offset + 2]
            if self._prev_targets is not None:
                previous = np.asarray(self._prev_targets, dtype=np.float64)
                if previous.shape == measured.shape and np.all(np.isfinite(previous)):
                    anchor = previous[offset + 2]
            gain = (
                self._cfg.left_j3_retarget_gain
                if partitioned_side == "left"
                else self._cfg.right_j3_retarget_gain
            )
            # Gate on flexion *relative to this session's calibration
            # baseline*, not an absolute angle: a human calibration pose is
            # never perfectly straight, so an absolute threshold is crossed
            # on frame one and the observability protection never engages.
            # Only additional bend beyond the baseline counts as evidence the
            # bend plane has become observable; straightening back toward or
            # past the baseline keeps J3 frozen at its anchor.
            baseline = self._j3_flexion_baseline.get(partitioned_side)
            gated_flexion = (
                targets[offset + 3]
                if baseline is None
                else max(0.0, targets[offset + 3] - baseline)
            )
            targets[offset + 2] = self._stabilized_j3_target(
                targets[offset + 2],
                anchor,
                gated_flexion,
                gain,
                self._cfg.j3_observability_start_flexion_deg,
                self._cfg.j3_observability_full_flexion_deg,
            )

            # With J1/J2/J4 fixed, wrist position traces a circle around J3.
            # Resolve that bend plane directly, including the +/-pi branch,
            # instead of depending on local IK convergence near straight arms.
            fk = self.make_command_fk(partitioned_side)
            probe = targets.copy()
            points = []
            for angle in (0.0, np.pi / 2, np.pi):
                probe[offset + 2] = angle
                points.append(fk(probe)[1])
            center = (points[0] + points[2]) / 2
            a, b = points[0] - center, points[1] - center
            desired_wrist = left_wrist_pose if partitioned_side == "left" else right_wrist_pose
            if (np.linalg.norm(a) > 0.025
                    and gated_flexion > np.deg2rad(self._cfg.j3_observability_start_flexion_deg)):
                delta = np.asarray(desired_wrist[:3]) - center
                angle = np.arctan2(np.dot(delta, b), np.dot(delta, a))
                candidates = [angle + k * 2 * np.pi for k in (-1, 0, 1)]
                candidates = [x for x in candidates
                              if self._arm_lower[offset + 2] <= x <= self._arm_upper[offset + 2]]
                if candidates:
                    targets[offset + 2] = min(candidates, key=lambda x: abs(x - anchor))

        # Stabilizers mirroring the reference controller.
        if not complete_geometry and self._cfg.max_command_step and self._cfg.max_command_step > 0.0:
            if self._prev_targets is not None:
                targets = np.clip(
                    targets,
                    self._prev_targets - self._cfg.max_command_step,
                    self._prev_targets + self._cfg.max_command_step,
                )
        if not complete_geometry and self._cfg.max_joint_position_delta and self._cfg.max_joint_position_delta > 0.0:
            targets = np.clip(
                targets,
                measured - self._cfg.max_joint_position_delta,
                measured + self._cfg.max_joint_position_delta,
            )
        if max_lead > 0.0:
            targets = np.clip(targets, measured - max_lead, measured + max_lead)
        if self._cfg.clamp_to_limits:
            targets = np.clip(targets, self._arm_lower, self._arm_upper)
        if complete_geometry:
            desired_elbow = left_elbow_pose if partitioned_side == "left" else right_elbow_pose
            errors = np.linalg.norm(fk(targets) - np.array(
                [desired_elbow[:3], desired_wrist[:3]]), axis=1)
            self.solution_diagnostics = {"elbow_error_m": float(errors[0]),
                                         "wrist_error_m": float(errors[1])}
            if not np.all(np.isfinite(errors)) or np.max(errors) > 0.03:
                self._report_solve_failure()
                return measured.copy()
        self.last_solution_valid = True
        self._report_solve_recovery()
        self._prev_targets = targets.copy()
        return targets.astype(np.float64)


__all__ = ["IkSolver"]
