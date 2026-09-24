"""Pure hand and wrist mappings matching the existing teleoperation implementation."""
import numpy as np

from esrobo_teleop import math_utils as mu
from esrobo_teleop.robot.linker_hand_driver import (
    ACTIVE_HAND_JOINTS, ACTIVE_JOINT_LIMITS_RAD, L10_ACTIVE_INDEX_BY_PHYSICAL,
)
from esrobo_teleop.robot.nero_driver import imu_local_rotation_to_wrist_offsets


RIGHT_FINGER_FLEXION_GAIN = 1.25
RIGHT_FINGER_FLEXION_PHYSICAL = (2, 3, 4, 5)
RIGHT_THUMB_FLEXION_GAIN = 1.25
RIGHT_THUMB_SIDE_GAIN = 0.60


def hand_units(active_rad, feedback, cfg, side, reference_rad=None):
    angles, measured = np.asarray(active_rad, dtype=float), np.asarray(feedback, dtype=float)
    if angles.shape != (10,) or measured.shape != (10,) or not np.all(np.isfinite([angles, measured])):
        raise ValueError("hand mapping requires ten finite angles and feedback values")
    upper = np.array([ACTIVE_JOINT_LIMITS_RAD[n][1] for n in ACTIVE_HAND_JOINTS])
    reference = np.zeros(10) if reference_rad is None else np.asarray(reference_rad, dtype=float)
    if reference.shape != (10,) or not np.all(np.isfinite(reference)):
        raise ValueError("invalid PICO open-hand reference")
    span = np.maximum(upper-reference, 1.0e-6)
    fraction = np.clip((angles-reference) / span, 0, 1)[L10_ACTIVE_INDEX_BY_PHYSICAL]
    if side == "right" and reference_rad is not None:
        # A full PICO fist reached about 78% of the previous four-finger
        # command range in the live right-hand sessions. Preserve the open
        # reference and commissioned endpoints; only rescale active flexion.
        fraction[list(RIGHT_FINGER_FLEXION_PHYSICAL)] = np.clip(
            fraction[list(RIGHT_FINGER_FLEXION_PHYSICAL)] * RIGHT_FINGER_FLEXION_GAIN, 0, 1)
        # PICO's thumb curl used only part of the commissioned pitch range,
        # while its opposition estimate repeatedly saturated thumb abduction.
        # Tune these two active L10 axes independently; leave thumb roll and
        # the robot's configured endpoints and velocity limits untouched.
        fraction[0] = np.clip(fraction[0] * RIGHT_THUMB_FLEXION_GAIN, 0, 1)
        fraction[1] = np.clip(fraction[1] * RIGHT_THUMB_SIDE_GAIN, 0, 1)
    opened = np.asarray(getattr(cfg, f"{side}_open"), dtype=float)
    closed = getattr(cfg, f"{side}_teleop_closed")
    closed = np.asarray(closed if len(closed) == 10 else getattr(cfg, f"{side}_closed"), dtype=float)
    if opened.shape != (10,) or closed.shape != (10,) or not np.all(np.isfinite([opened, closed])):
        raise ValueError("invalid hand endpoints")
    result = opened + fraction * (closed - opened)
    for index in range(10):
        if index not in cfg.enabled_physical_joints:
            result[index] = measured[index]
    # Robot still applies its commissioned step/velocity limits.
    return np.rint(np.clip(result, 0, 255)).astype(int).tolist()


class WristMapping:
    def __init__(self, solver, cfg, side, measured):
        self.solver, self.cfg, self.side = solver, cfg, side
        self.indices = np.asarray(cfg.right_wrist_joint_indices, dtype=int)
        if self.indices.tolist() != [4, 5, 6]:
            raise ValueError("wrist indices must be J5-J7")
        self.direction = np.asarray(getattr(cfg, f"{side}_joint_directions"))[self.indices]
        self.offset = np.asarray(getattr(cfg, f"{side}_joint_offsets"))[self.indices]
        self.neutral = np.asarray(cfg.wrist_neutral_joint_positions)
        lower, upper = solver.position_envelope(side)
        self.terminal_lower = np.asarray(lower, dtype=float)[self.indices]
        self.terminal_upper = np.asarray(upper, dtype=float)[self.indices]
        q = self.with_neutral(measured)
        _, axes = solver.wrist_orientation_linearization(side, q, tuple(self.indices))
        self.axes = axes / self.direction[np.newaxis, :]

    def with_neutral(self, full):
        full = np.asarray(full, dtype=float).copy()
        index = self.indices + (0 if self.side == "left" else 7)
        full[index] = (self.neutral - self.offset) / self.direction
        return full

    def terminal(self, rotation):
        physical = imu_local_rotation_to_wrist_offsets(
            rotation, self.cfg, self.axes, limit_total_angle=True)
        target = (self.neutral + physical - self.offset) / self.direction
        # The PICO wrist orientation can remain valid beyond the robot's
        # commissioned J5-J7 range.  Constrain it before IK so an otherwise
        # valid shoulder/elbow target is not discarded wholesale.
        return np.clip(target, self.terminal_lower, self.terminal_upper)

    def compensated(self, desired_pose, position_solution):
        baseline = self.solver.current_task_frame_poses(self.with_neutral(position_solution))[f"{self.side}_wrist"]
        rotation = mu.quat_wxyz_to_matrix(baseline[3:]).T @ mu.quat_wxyz_to_matrix(desired_pose[3:])
        return self.terminal(rotation)
