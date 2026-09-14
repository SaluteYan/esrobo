"""Pink IK solver over the full ESROBO URDF for the two NERO arms.

Standalone port of the reference ``pink_ik.py`` / ``pink_actions.py``:
the four controlled task frames (elbow + hand on each side) are driven by
``pink.solve_ik`` and the resulting joint velocity is integrated to produce
target positions for the 14 NERO arm joints.  Only the arm joints are read out;
waist / head / hand joints are held at their current value.
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

    def _make_config(self, arm_joint_pos: np.ndarray):
        from pink import Configuration

        full_q = np.zeros(self._model.nq, dtype=np.float64)
        full_q[self._arm_q_indices] = arm_joint_pos
        if self._config is None:
            self._config = Configuration(self._model, self._data, full_q.copy())
        self._config.update(full_q.copy())
        return self._config

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
        current_elbow = np.asarray(current_poses[f"{side}_elbow"][:3], dtype=np.float64)
        current_wrist = np.asarray(current_poses[f"{side}_wrist"][:3], dtype=np.float64)

        def segment_angle(a: np.ndarray, b: np.ndarray, c: np.ndarray) -> float | None:
            upper = b - a
            forearm = c - b
            denominator = float(np.linalg.norm(upper) * np.linalg.norm(forearm))
            if denominator < 1.0e-12:
                return None
            cosine = float(np.clip(np.dot(upper, forearm) / denominator, -1.0, 1.0))
            return float(np.arccos(cosine))

        target_angle = segment_angle(shoulder, elbow, wrist)
        current_angle = segment_angle(shoulder, current_elbow, current_wrist)
        if target_angle is None or current_angle is None:
            return None
        j4_index = (0 if side == "left" else 7) + 3
        # Use an angle delta around measured J4. This cancels fixed Hand_link
        # geometry offsets and any held J5-J7 contribution to wrist position.
        flexion = float(current_arm_joint_pos[j4_index] + target_angle - current_angle)
        return float(np.clip(flexion, self._arm_lower[j4_index], self._arm_upper[j4_index]))

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

        if partitioned_side not in (None, "left", "right"):
            raise ValueError("partitioned_side must be None, 'left', or 'right'")
        if partitioned_side is not None and not self._cfg.partition_terminal_wrist_ik:
            partitioned_side = None
        measured = np.asarray(current_arm_joint_pos, dtype=np.float64).copy()
        solve_seed = measured.copy()
        max_lead = max(0.0, float(self._cfg.max_command_lead))
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
        except Exception as exc:  # noqa: BLE001
            print(f"[ik] solve failed: {exc}; holding joints.", flush=True)
            return current_arm_joint_pos.copy()

        targets = config.q[self._arm_q_indices].copy()
        if partitioned_side is not None:
            terminal = np.asarray(terminal_joint_targets, dtype=np.float64).reshape(-1)
            if terminal.shape != (3,) or not np.all(np.isfinite(terminal)):
                raise ValueError("partitioned IK requires three finite terminal joint targets")
            offset = 0 if partitioned_side == "left" else 7
            targets[offset + 4:offset + 7] = terminal

        # Stabilizers mirroring the reference controller.
        if self._cfg.max_command_step and self._cfg.max_command_step > 0.0:
            if self._prev_targets is not None:
                targets = np.clip(
                    targets,
                    self._prev_targets - self._cfg.max_command_step,
                    self._prev_targets + self._cfg.max_command_step,
                )
        if self._cfg.max_joint_position_delta and self._cfg.max_joint_position_delta > 0.0:
            targets = np.clip(
                targets,
                measured - self._cfg.max_joint_position_delta,
                measured + self._cfg.max_joint_position_delta,
            )
        if max_lead > 0.0:
            targets = np.clip(targets, measured - max_lead, measured + max_lead)
        if self._cfg.clamp_to_limits:
            targets = np.clip(targets, self._arm_lower, self._arm_upper)
        self._prev_targets = targets.copy()
        return targets.astype(np.float64)


__all__ = ["IkSolver"]
