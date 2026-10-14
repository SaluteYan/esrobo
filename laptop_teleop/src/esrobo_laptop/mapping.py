"""Pure hand and wrist mappings matching the existing teleoperation implementation."""
import numpy as np

from esrobo_teleop import math_utils as mu
from esrobo_teleop.robot.linker_hand_driver import (
    ACTIVE_HAND_JOINTS, ACTIVE_JOINT_LIMITS_RAD, L10_ACTIVE_INDEX_BY_PHYSICAL,
)
from .orientation import ForearmFrame, palm_clock, checked_rotation


FINGER_FLEXION_GAIN = 1.25
FINGER_FLEXION_PHYSICAL = (2, 3, 4, 5)
THUMB_FLEXION_GAIN = 1.25
THUMB_SIDE_GAIN = 0.60


def _wrist_residual_jacobian(axes, delta, desired_rotation):
    """SO(3) log error and its exact joint derivative, in the same frame.

    E = R(q).T R_des; a joint perturbation left-multiplies E by
    Exp(-R(q).T axis_world dq). The inverse left log Jacobian converts
    this perturbation to the rotation-vector residual used by least_squares.
    """
    from scipy.spatial.transform import Rotation
    predicted = Rotation.identity()
    world_axes = []
    for axis, angle in zip(axes.T, delta):
        world_axes.append(predicted.apply(axis))
        predicted = predicted * Rotation.from_rotvec(axis * angle)
    residual = (predicted.inv() * desired_rotation).as_rotvec()
    x, y, z = residual
    cross = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    theta2 = float(residual @ residual)
    if theta2 < 1.e-8:
        coefficient = 1./12. + theta2/720. + theta2*theta2/30240.
    else:
        theta = np.sqrt(theta2)
        coefficient = (1. - .5*theta/np.tan(.5*theta))/theta2
    inverse_left = np.eye(3) - .5*cross + coefficient*(cross @ cross)
    local_axes = predicted.as_matrix().T @ np.asarray(world_axes).T
    return residual, -inverse_left @ local_axes


def hand_units(active_rad, feedback, cfg, side, reference_rad=None, *, absolute_angles=False):
    angles, measured = np.asarray(active_rad, dtype=float), np.asarray(feedback, dtype=float)
    if angles.shape != (10,) or measured.shape != (10,) or not np.all(np.isfinite([angles, measured])):
        raise ValueError("hand mapping requires ten finite angles and feedback values")
    upper = np.array([ACTIVE_JOINT_LIMITS_RAD[n][1] for n in ACTIVE_HAND_JOINTS])
    reference = np.zeros(10) if reference_rad is None else np.asarray(reference_rad, dtype=float)
    if reference.shape != (10,) or not np.all(np.isfinite(reference)):
        raise ValueError("invalid PICO open-hand reference")
    # Absolute PICO angles retain their anatomical zero across preparations.
    # The sampled open pose is only an entry readiness check in this mode.
    origin = np.zeros(10) if absolute_angles else reference
    span = np.maximum(upper-origin, 1.0e-6)
    fraction = np.clip((angles-origin) / span, 0, 1)[L10_ACTIVE_INDEX_BY_PHYSICAL]
    if side in ("left", "right") and (absolute_angles or reference_rad is not None):
        # A full PICO fist reached about 78% of the previous four-finger
        # command range in the live right-hand sessions. Share these gains
        # with the left hand while preserving each side's measured endpoints.
        # Preserve the open reference and commissioned endpoints; only
        # rescale active flexion.
        fraction[list(FINGER_FLEXION_PHYSICAL)] = np.clip(
            fraction[list(FINGER_FLEXION_PHYSICAL)] * FINGER_FLEXION_GAIN, 0, 1)
        # PICO's thumb curl used only part of the commissioned pitch range,
        # while its opposition estimate repeatedly saturated thumb abduction.
        # Tune these two active L10 axes independently.  The endpoint below
        # limits flexion before the measured mechanical stop.
        fraction[0] = np.clip(fraction[0] * THUMB_FLEXION_GAIN, 0, 1)
        fraction[1] = np.clip(fraction[1] * THUMB_SIDE_GAIN, 0, 1)
    opened = np.asarray(getattr(cfg, f"{side}_open"), dtype=float)
    closed = getattr(cfg, f"{side}_teleop_closed")
    closed = np.asarray(closed if len(closed) == 10 else getattr(cfg, f"{side}_closed"), dtype=float)
    if opened.shape != (10,) or closed.shape != (10,) or not np.all(np.isfinite([opened, closed])):
        raise ValueError("invalid hand endpoints")
    result = opened + fraction * (closed - opened)
    if side == "right":
        # Physical index 0 drives all three linked thumb flexion joints.
        result[0] = max(result[0], cfg.right_thumb_pitch_min_unit)
    for index in range(10):
        if index not in cfg.enabled_physical_joints:
            result[index] = measured[index]
    # Robot still applies its commissioned step/velocity limits.
    return np.rint(np.clip(result, 0, 255)).astype(int).tolist()


class WristMapping:
    def __init__(self, solver, cfg, side, measured, *, max_angle_deg=None,
                 source_palm_clock=None, absolute_pose=False):
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
        self.neutral_urdf = (self.neutral - self.offset) / self.direction
        if np.any(self.neutral_urdf < self.terminal_lower) or np.any(self.neutral_urdf > self.terminal_upper):
            raise ValueError("wrist neutral lies outside commissioned joint limits")
        self.max_angle_rad = np.deg2rad(
            cfg.right_wrist_max_angle_deg if max_angle_deg is None else max_angle_deg)
        if not np.isfinite(self.max_angle_rad) or self.max_angle_rad <= 0:
            raise ValueError("invalid wrist mapping angle")
        q = self.with_neutral(measured)
        _, axes = solver.wrist_orientation_linearization(side, q, tuple(self.indices))
        # The URDF joint axes are exact at neutral. Their ordered rotation
        # product describes finite J5/J6/J7 motion; a single rotvec projection
        # is only a small-angle approximation and accumulated 14+ deg error.
        norms = np.linalg.norm(axes, axis=0)
        if np.any(norms < .5) or np.any(norms > 1.5):
            raise ValueError("invalid wrist orientation axes")
        self.axes = axes / norms
        offset = 0 if side == "left" else 7
        self.absolute_pose = bool(absolute_pose)
        self.reference_terminal = (self.neutral_urdf.copy() if self.absolute_pose else
                                  np.asarray(measured, dtype=float)[offset+4:offset+7].copy())
        if (not np.all(np.isfinite(self.reference_terminal))
                or np.any(self.reference_terminal < self.terminal_lower)
                or np.any(self.reference_terminal > self.terminal_upper)):
            raise ValueError("wrist calibration pose lies outside commissioned joint limits")
        from scipy.spatial.transform import Rotation
        reference_rotation = Rotation.identity()
        for axis, angle in zip(self.axes.T, self.reference_terminal-self.neutral_urdf):
            reference_rotation = reference_rotation * Rotation.from_rotvec(axis * angle)
        self.reference_rotation_local = reference_rotation.as_matrix()
        self.forearm_frame = ForearmFrame()
        if source_palm_clock is not None and not np.isfinite(source_palm_clock):
            raise ValueError("finite PICO palm reference clock required")
        self.source_palm_clock = source_palm_clock
        self.last_diagnostics = {}
        self._last_terminal_target = None

    def with_neutral(self, full):
        full = np.asarray(full, dtype=float).copy()
        index = self.indices + (0 if self.side == "left" else 7)
        full[index] = (self.neutral - self.offset) / self.direction
        return full

    def terminal(self, rotation):
        from scipy.optimize import least_squares
        from scipy.spatial.transform import Rotation

        desired = checked_rotation(rotation)
        desired_rotation = Rotation.from_matrix(desired)
        requested_deg = float(np.degrees(desired_rotation.magnitude()))
        linear = np.linalg.lstsq(self.axes, desired_rotation.as_rotvec(), rcond=None)[0]
        lower = self.terminal_lower - self.neutral_urdf
        upper = self.terminal_upper - self.neutral_urdf
        seed = np.clip(linear, lower, upper)
        linear_seed = seed.copy()
        if np.allclose(self.axes.T @ self.axes, np.eye(3), atol=1.e-5):
            # Ordered intrinsic XYZ in the neutral joint-axis basis yields
            # the finite-angle branch directly, including large J5 twists.
            # A rotvec projection is not a valid seed near 180-degree poses.
            handedness = float(np.sign(np.linalg.det(self.axes)))
            finite = Rotation.from_matrix(self.axes.T @ desired @ self.axes).as_euler("XYZ")
            seed = np.clip(handedness * finite, lower, upper)

        # scipy asks for fun/jac at the same point. Cache only this call's
        # exact point, never a previous sensor target or a mutable solver state.
        cached_delta = cached_value = None

        def evaluate(delta):
            nonlocal cached_delta, cached_value
            if cached_delta is None or not np.array_equal(delta, cached_delta):
                cached_value = _wrist_residual_jacobian(self.axes, delta, desired_rotation)
                cached_delta = np.asarray(delta).copy()
            return cached_value

        def residual(delta):
            return evaluate(delta)[0]

        def jacobian(delta):
            return evaluate(delta)[1]

        # Within the principal middle-angle branch, the Euler seed is unique
        # (outer spans < one revolution). URDF rounded pi/2 angles introduce
        # tiny nonorthogonality: permit one small Newton correction, then
        # certify reconstruction using the actual axes and all existing bounds.
        # No approximate projection, ambiguous branch or singular pose uses
        # this shortcut; those retain the bounded warm-start solver below.
        exact = False
        certified = seed.copy()
        if (np.allclose(self.axes.T @ self.axes, np.eye(3), rtol=0., atol=1.e-7)
                and lower[1] > -np.pi/2+.001 and upper[1] < np.pi/2-.001
                and np.all(upper-lower < 2*np.pi)):
            error, derivative = evaluate(certified)
            if 1.e-10 <= np.linalg.norm(error) < 1.e-6:
                try:
                    correction = np.linalg.solve(derivative, -error)
                    if np.max(np.abs(correction)) < 1.e-4:
                        certified = certified + correction
                except np.linalg.LinAlgError:
                    pass
            exact = (np.all(certified >= lower) and np.all(certified <= upper)
                     and np.linalg.norm(residual(certified)) < 1.e-10)
        if exact:
            delta, evaluations = certified, 0
        else:
            start = seed if self._last_terminal_target is None else np.clip(
                self._last_terminal_target-self.neutral_urdf, lower, upper)
            solution = least_squares(residual, start, jac=jacobian, bounds=(lower, upper),
                                     max_nfev=20, ftol=1e-6, xtol=1e-6, gtol=1e-6)
            # Preserve branch continuity; retry the absolute estimate if needed.
            evaluations = solution.nfev
            if np.linalg.norm(residual(solution.x)) > np.deg2rad(.25) and not np.allclose(start, seed):
                alternate = least_squares(residual, seed, jac=jacobian, bounds=(lower, upper),
                                          max_nfev=20, ftol=1e-6, xtol=1e-6, gtol=1e-6)
                evaluations += alternate.nfev
                if np.linalg.norm(residual(alternate.x)) < np.linalg.norm(residual(solution.x))-1.e-6:
                    solution = alternate
            delta = solution.x
        raw_target = self.neutral_urdf + delta
        # Reachability is governed by each commissioned joint bound. Shrinking
        # the solved joint vector would distort the already projected attitude.
        target = np.clip(self.neutral_urdf + delta, self.terminal_lower, self.terminal_upper)
        self._last_terminal_target = target.copy()
        remaining = residual(target - self.neutral_urdf)
        self.last_diagnostics = dict(
            source="PICO Body wrist", requested_rotation_deg=requested_deg,
            linear_orientation_error_deg=float(np.degrees(np.linalg.norm(residual(linear_seed)))),
            finite_seed_orientation_error_deg=float(np.degrees(np.linalg.norm(residual(seed)))),
            mapped_orientation_error_deg=float(np.degrees(np.linalg.norm(remaining))),
            target_joint_deg=np.degrees(target).tolist(),
            at_joint_limit=[bool(abs(q-lo) < np.deg2rad(.2) or abs(q-hi) < np.deg2rad(.2))
                            for q, lo, hi in zip(target, self.terminal_lower, self.terminal_upper)],
            joint_norm_capped=False, max_joint_norm_deg=None,
            orientation_projected=bool(np.linalg.norm(remaining) > np.deg2rad(.25)),
            joint_lower_deg=np.degrees(self.terminal_lower).tolist(),
            joint_upper_deg=np.degrees(self.terminal_upper).tolist(),
            solver_evaluations=int(evaluations),
            solver_method='certified_exact_wrist' if exact else 'bounded_least_squares')
        self.last_diagnostics.update(
            unscaled_target_joint_deg=np.degrees(raw_target).tolist(),
            joint_norm_scale=1.,
            j5_norm_reduction_deg=float(np.degrees(raw_target[0]-target[0])))
        return target

    @property
    def anatomical_mount(self):
        # Left URDF palm normal/lateral point opposite to Hand landmark axes.
        return np.eye(3) if self.side == "right" else np.diag([-1., -1., 1.])

    def absolute_local_from_robot(self, full):
        """Seed only the rate limiter from the arrived physical attitude."""
        poses = self.solver.current_task_frame_poses(self.with_neutral(full))
        carrier = self.forearm_frame.update({k: poses[f"{self.side}_{k}"][:3]
                                            for k in ("shoulder", "elbow", "wrist")})
        actual = self.solver.current_task_frame_poses(full)
        rotation = mu.quat_wxyz_to_matrix(actual[f"{self.side}_wrist"][3:])
        return carrier.T @ rotation @ self.anatomical_mount.T

    def source_terminal(self, local_orientation, position_solution):
        """Map the unfiltered source without changing command IK history."""
        import copy
        previous = self._last_terminal_target
        diagnostics = self.last_diagnostics
        forearm = self.forearm_frame
        self.forearm_frame = copy.deepcopy(forearm)
        try:
            return self.absolute_terminal(local_orientation, position_solution)
        finally:
            self._last_terminal_target = previous
            self.last_diagnostics = diagnostics
            self.forearm_frame = forearm

    def absolute_terminal(self, local_orientation, position_solution):
        """Transfer absolute palm attitude without subtracting the entry pose."""
        poses = self.solver.current_task_frame_poses(self.with_neutral(position_solution))
        carrier = self.forearm_frame.update({k: poses[f"{self.side}_{k}"][:3]
                                            for k in ("shoulder", "elbow", "wrist")})
        neutral = mu.quat_wxyz_to_matrix(poses[f"{self.side}_wrist"][3:])
        desired = carrier @ checked_rotation(local_orientation) @ self.anatomical_mount
        target = self.terminal(neutral.T @ desired)
        self.last_diagnostics.update(
            source="PICO Hand anatomical palm", orientation_mode="forearm_absolute",
            position_task="neutral_terminal_carrier", anatomical_alignment=True,
            position_reference_joint_deg=np.degrees(self.neutral_urdf).tolist(),
            desired_world_quat_wxyz=mu.matrix_to_quat_wxyz(desired).tolist(),
            robot_forearm_plane_observable=bool(self.forearm_frame.plane_observable),
            palm_clock_alignment_deg=None,
        )
        return target

    def with_reference(self, full):
        """Arm position carrier with the calibrated terminal joints held fixed."""
        result = np.asarray(full, dtype=float).copy()
        offset = 0 if self.side == "left" else 7
        result[offset+4:offset+7] = self.reference_terminal
        return result

    def relative_terminal(self, relative_rotation, position_solution):
        """Compose arm-carried world attitude with independent local hand motion.

        The fixed-terminal FK endpoint is the arm's position task. Appending
        wrist motion moves the physical palm about the wrist as in the URDF;
        it must not make J1-J4 counter-move to pin the palm to the old point.
        """
        poses = self.solver.current_task_frame_poses(self.with_reference(position_solution))
        points = {name: poses[f"{self.side}_{name}"][:3] for name in ("shoulder", "elbow", "wrist")}
        carrier = self.forearm_frame.update(points)
        hand_reference = mu.quat_wxyz_to_matrix(poses[f"{self.side}_wrist"][3:])
        neutral = hand_reference @ self.reference_rotation_local.T
        clock_delta = 0.
        aligned = relative_rotation
        if self.source_palm_clock is not None:
            from scipy.spatial.transform import Rotation
            # URDF hand +Z points to fingers. Index-minus-little is +Y on
            # right, -Y on left, hence the corresponding normal is +/-X.
            robot_normal = hand_reference[:, 0] * (1. if self.side == "right" else -1.)
            clock_delta = palm_clock(carrier, robot_normal) - self.source_palm_clock
            alignment = Rotation.from_rotvec([clock_delta, 0., 0.]).as_matrix()
            aligned = alignment @ relative_rotation @ alignment.T
        desired = carrier @ aligned @ carrier.T @ hand_reference
        target = self.terminal(neutral.T @ desired)
        self.last_diagnostics.update(
            source="PICO Body wrist relative to forearm", orientation_mode="forearm_relative",
            position_task="calibrated_terminal_reference",
            position_reference_joint_deg=np.degrees(self.reference_terminal).tolist(),
            desired_world_quat_wxyz=mu.matrix_to_quat_wxyz(desired).tolist(),
            robot_forearm_plane_observable=bool(self.forearm_frame.plane_observable),
            anatomical_alignment=self.source_palm_clock is not None,
            palm_clock_alignment_deg=float(np.degrees(clock_delta)),
        )
        return target
