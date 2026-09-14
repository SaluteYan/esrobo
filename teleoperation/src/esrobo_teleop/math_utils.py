"""Pure-NumPy math helpers for ESROBO real-machine teleoperation.

These functions are a self-contained port of the NumPy helpers used by the
``udp_bimanual_body_device`` in the ``Dual-arm-teleoperation`` reference repo,
stripped of all IsaacLab / PyTorch dependencies so they run on the real robot
inside a plain conda environment.
"""

from __future__ import annotations

import math

import numpy as np


def normalize_quat_wxyz(quat: np.ndarray) -> np.ndarray:
    """Normalize a quaternion stored as ``[w, x, y, z]``."""
    quat = np.asarray(quat, dtype=np.float32).reshape(-1)
    norm = float(np.linalg.norm(quat))
    if norm < 1.0e-8:
        return np.asarray([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
    return (quat / norm).astype(np.float32)


def quat_wxyz_to_matrix(quat: np.ndarray) -> np.ndarray:
    """Convert a ``[w, x, y, z]`` quaternion to a 3x3 rotation matrix."""
    w, x, y, z = normalize_quat_wxyz(quat)
    return np.asarray(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float32,
    )


def matrix_to_quat_wxyz(matrix: np.ndarray) -> np.ndarray:
    """Convert a 3x3 rotation matrix to a normalized ``[w, x, y, z]`` quaternion."""
    matrix = np.asarray(matrix, dtype=np.float32).reshape(3, 3)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (matrix[2, 1] - matrix[1, 2]) / s
        y = (matrix[0, 2] - matrix[2, 0]) / s
        z = (matrix[1, 0] - matrix[0, 1]) / s
    elif matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
        s = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
        w = (matrix[2, 1] - matrix[1, 2]) / s
        x = 0.25 * s
        y = (matrix[0, 1] + matrix[1, 0]) / s
        z = (matrix[0, 2] + matrix[2, 0]) / s
    elif matrix[1, 1] > matrix[2, 2]:
        s = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
        w = (matrix[0, 2] - matrix[2, 0]) / s
        x = (matrix[0, 1] + matrix[1, 0]) / s
        y = 0.25 * s
        z = (matrix[1, 2] + matrix[2, 1]) / s
    else:
        s = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
        w = (matrix[1, 0] - matrix[0, 1]) / s
        x = (matrix[0, 2] + matrix[2, 0]) / s
        y = (matrix[1, 2] + matrix[2, 1]) / s
        z = 0.25 * s
    return normalize_quat_wxyz(np.asarray([w, x, y, z], dtype=np.float32))


def pose_array_to_matrix(pose: np.ndarray) -> np.ndarray:
    """Convert a ``[x, y, z, qw, qx, qy, qz]`` pose to a 4x4 matrix."""
    matrix = np.eye(4, dtype=np.float32)
    matrix[:3, 3] = pose[:3]
    matrix[:3, :3] = quat_wxyz_to_matrix(pose[3:])
    return matrix


def pose_matrix_to_array(matrix: np.ndarray) -> np.ndarray:
    """Convert a 4x4 matrix to a ``[x, y, z, qw, qx, qy, qz]`` pose."""
    quat = matrix_to_quat_wxyz(matrix[:3, :3])
    return np.concatenate([matrix[:3, 3], quat]).astype(np.float32)


def make_position_pose(position: np.ndarray) -> np.ndarray:
    """Build an identity-orientation pose ``[x, y, z, 1, 0, 0, 0]``."""
    return np.asarray(
        [position[0], position[1], position[2], 1.0, 0.0, 0.0, 0.0], dtype=np.float32
    )


def average_pose_matrices(samples: list[np.ndarray]) -> np.ndarray:
    """Average a list of 4x4 pose matrices (positions + quaternion mean)."""
    if not samples:
        return np.eye(4, dtype=np.float32)
    positions = np.stack([sample[:3, 3] for sample in samples], axis=0)
    quaternions = np.stack(
        [matrix_to_quat_wxyz(sample[:3, :3]) for sample in samples], axis=0
    )
    reference = quaternions[0]
    aligned = []
    for quat in quaternions:
        if float(np.dot(reference, quat)) < 0.0:
            quat = -quat
        aligned.append(quat)
    avg_quat = normalize_quat_wxyz(np.mean(np.stack(aligned, axis=0), axis=0))
    matrix = np.eye(4, dtype=np.float32)
    matrix[:3, 3] = np.mean(positions, axis=0)
    matrix[:3, :3] = quat_wxyz_to_matrix(avg_quat)
    return matrix


def average_arm_points(samples: list[dict[str, np.ndarray]]) -> dict[str, np.ndarray] | None:
    if not samples:
        return None
    return {
        key: np.mean(np.stack([sample[key] for sample in samples], axis=0), axis=0).astype(
            np.float32
        )
        for key in ("shoulder", "elbow", "wrist")
    }


def arm_points_relative_to_shoulder(
    points: dict[str, np.ndarray], position_delta_signs: np.ndarray
) -> dict[str, np.ndarray]:
    shoulder = points["shoulder"]
    return {
        "shoulder": np.zeros(3, dtype=np.float32),
        "elbow": ((points["elbow"] - shoulder) * position_delta_signs).astype(np.float32),
        "wrist": ((points["wrist"] - shoulder) * position_delta_signs).astype(np.float32),
    }


def normalize_vector(vector: np.ndarray) -> np.ndarray | None:
    norm = float(np.linalg.norm(vector))
    if norm < 1.0e-6:
        return None
    return (vector / norm).astype(np.float32)


def scale_vector_to_length(
    vector: np.ndarray, length: float, fallback: np.ndarray
) -> np.ndarray | None:
    direction = normalize_vector(vector)
    if direction is None:
        direction = normalize_vector(fallback)
    if direction is None:
        return None
    return (direction * float(length)).astype(np.float32)


def minimal_vector_alignment_rotation(
    source: np.ndarray, target: np.ndarray
) -> np.ndarray | None:
    """Return the minimum-angle rotation that aligns ``source`` to ``target``.

    A single measured direction does not constrain roll about that direction.
    The shortest rotation preserves the already-defined transverse axes.
    """
    source_direction = normalize_vector(source)
    target_direction = normalize_vector(target)
    if source_direction is None or target_direction is None:
        return None

    cosine = float(np.clip(np.dot(source_direction, target_direction), -1.0, 1.0))
    cross = np.cross(source_direction, target_direction)
    sine = float(np.linalg.norm(cross))
    if sine < 1.0e-7:
        if cosine > 0.0:
            return np.eye(3, dtype=np.float32)
        basis = np.eye(3, dtype=np.float32)[int(np.argmin(np.abs(source_direction)))]
        axis = normalize_vector(np.cross(source_direction, basis))
        if axis is None:
            return None
        return rotvec_to_rotation_matrix(axis * np.pi)

    axis = cross / sine
    return rotvec_to_rotation_matrix(axis * np.arctan2(sine, cosine))


def arm_points_to_rotation(points: dict[str, np.ndarray]) -> np.ndarray | None:
    """Build a rotation from shoulder/elbow/wrist reach + bend plane."""
    reach_axis = normalize_vector(points["wrist"] - points["shoulder"])
    upper_axis = normalize_vector(points["elbow"] - points["shoulder"])
    if reach_axis is None:
        return None

    bend_axis = None
    if upper_axis is not None:
        bend_axis = normalize_vector(
            upper_axis - float(np.dot(upper_axis, reach_axis)) * reach_axis
        )
    if bend_axis is None:
        for reference in (
            np.asarray([0.0, 0.0, 1.0], dtype=np.float32),
            np.asarray([0.0, 1.0, 0.0], dtype=np.float32),
            np.asarray([1.0, 0.0, 0.0], dtype=np.float32),
        ):
            bend_axis = normalize_vector(
                reference - float(np.dot(reference, reach_axis)) * reach_axis
            )
            if bend_axis is not None:
                break
    if bend_axis is None:
        return None

    plane_normal = normalize_vector(np.cross(bend_axis, reach_axis))
    if plane_normal is None:
        return None
    side_axis = normalize_vector(np.cross(plane_normal, reach_axis))
    if side_axis is None:
        return None
    return np.stack([reach_axis, side_axis, plane_normal], axis=1).astype(np.float32)


def invert_pose_matrix(matrix: np.ndarray) -> np.ndarray:
    inverse = np.eye(4, dtype=np.float32)
    inverse[:3, :3] = matrix[:3, :3].T
    inverse[:3, 3] = -inverse[:3, :3] @ matrix[:3, 3]
    return inverse


def rotation_matrix_to_rotvec(matrix: np.ndarray) -> np.ndarray:
    quat = matrix_to_quat_wxyz(matrix)
    if quat[0] < 0.0:
        quat = -quat
    vector = quat[1:].astype(np.float64)
    vector_norm = float(np.linalg.norm(vector))
    if vector_norm < 1.0e-8:
        return (2.0 * vector).astype(np.float32)
    angle = 2.0 * math.atan2(vector_norm, float(quat[0]))
    return (vector * (angle / vector_norm)).astype(np.float32)


def rotation_matrix_to_xyz_angles(matrix: np.ndarray) -> np.ndarray:
    """Decompose ``R = Rx(x) @ Ry(y) @ Rz(z)`` without rotvec cross-coupling."""
    rotation = np.asarray(matrix, dtype=np.float64).reshape(3, 3)
    if not np.all(np.isfinite(rotation)):
        raise ValueError("rotation matrix must be finite")
    y_angle = math.asin(float(np.clip(rotation[0, 2], -1.0, 1.0)))
    cos_y = math.cos(y_angle)
    if abs(cos_y) > 1.0e-6:
        x_angle = math.atan2(-rotation[1, 2], rotation[2, 2])
        z_angle = math.atan2(-rotation[0, 1], rotation[0, 0])
    else:
        # The configured wrist range stays well clear of this singularity.
        x_angle = math.atan2(rotation[2, 1], rotation[1, 1])
        z_angle = 0.0
    return np.asarray([x_angle, y_angle, z_angle], dtype=np.float32)


def rotation_matrix_to_x_twist_yz_swing(matrix: np.ndarray) -> np.ndarray:
    """Split a hand delta into longitudinal X twist and orthogonal Y/Z swing.

    The quaternion is factored as ``q = q_swing * q_twist_x``.  Unlike ordered
    Euler angles, palm side-sway/flexion cannot create a synthetic X component
    merely because two rotations are present at the same time.
    """
    rotation = np.asarray(matrix, dtype=np.float64).reshape(3, 3)
    if not np.all(np.isfinite(rotation)):
        raise ValueError("rotation matrix must be finite")
    quat = matrix_to_quat_wxyz(rotation).astype(np.float64)
    if quat[0] < 0.0:
        quat = -quat
    twist_norm = math.hypot(float(quat[0]), float(quat[1]))
    if twist_norm < 1.0e-8:
        # A 180-degree swing is outside the configured wrist range, but retain
        # a finite fallback for malformed/replayed packets.
        return rotation_matrix_to_rotvec(rotation)
    twist_w = float(quat[0]) / twist_norm
    twist_x = float(quat[1]) / twist_norm
    twist_angle = 2.0 * math.atan2(twist_x, twist_w)
    twist_matrix = rotvec_to_rotation_matrix(
        np.asarray([twist_angle, 0.0, 0.0], dtype=np.float64)
    )
    swing_matrix = rotation @ twist_matrix.T
    swing_rotvec = rotation_matrix_to_rotvec(swing_matrix).astype(np.float64)
    return np.asarray(
        [twist_angle, swing_rotvec[1], swing_rotvec[2]], dtype=np.float32
    )


def rotvec_to_rotation_matrix(rotvec: np.ndarray) -> np.ndarray:
    rotvec = np.asarray(rotvec, dtype=np.float64).reshape(3)
    angle = float(np.linalg.norm(rotvec))
    if angle < 1.0e-8:
        quat = np.asarray([1.0, *(0.5 * rotvec)], dtype=np.float32)
    else:
        half_angle = 0.5 * angle
        quat = np.concatenate(
            (
                np.asarray([math.cos(half_angle)]),
                rotvec * (math.sin(half_angle) / angle),
            )
        ).astype(np.float32)
    return quat_wxyz_to_matrix(quat)


__all__ = [
    "normalize_quat_wxyz",
    "quat_wxyz_to_matrix",
    "matrix_to_quat_wxyz",
    "pose_array_to_matrix",
    "pose_matrix_to_array",
    "make_position_pose",
    "average_pose_matrices",
    "average_arm_points",
    "arm_points_relative_to_shoulder",
    "normalize_vector",
    "scale_vector_to_length",
    "minimal_vector_alignment_rotation",
    "arm_points_to_rotation",
    "invert_pose_matrix",
    "rotation_matrix_to_rotvec",
    "rotation_matrix_to_xyz_angles",
    "rotvec_to_rotation_matrix",
]
