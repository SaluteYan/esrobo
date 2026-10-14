"""Hand motion relative to a transported, right-handed forearm frame.

The carrier follows elbow/wrist geometry. Rotation about its longitudinal axis
is deliberately left to the hand, so pronation is not lost in arm tracking.
"""
import numpy as np
from scipy.spatial.transform import Rotation


def checked_rotation(matrix):
    matrix = np.asarray(matrix, dtype=float)
    if (matrix.shape != (3, 3) or not np.all(np.isfinite(matrix))
            or not np.allclose(matrix.T @ matrix, np.eye(3), atol=1.e-4)
            or abs(np.linalg.det(matrix)-1.) > 1.e-4):
        raise ValueError("proper orthonormal palm rotation required")
    return matrix.copy()


def palm_clock(carrier, normal):
    """Clock around the forearm; never tilt its independent roll axis."""
    local = np.asarray(carrier).T @ np.asarray(normal)
    if local.shape != (3,) or not np.all(np.isfinite(local)) or np.linalg.norm(local[1:]) < .5:
        raise ValueError("palm normal too parallel to forearm; straighten wrist for calibration")
    return float(np.arctan2(local[2], local[1]))


def forearm_twist_deg(rotation):
    """Signed swing/twist decomposition about forearm X, independent of swing.

    A 180-degree transverse swing has no unique twist; report unavailable.
    """
    quat = Rotation.from_matrix(rotation).as_quat()
    if np.hypot(quat[0], quat[3]) < 1.e-8:
        return None
    angle = 2*np.arctan2(quat[0], quat[3])
    return float(np.degrees((angle+np.pi) % (2*np.pi)-np.pi))


class ForearmFrame:
    def __init__(self):
        self.rotation = None
        self.plane_observable = False

    def update(self, points):
        upper = np.asarray(points["elbow"], dtype=float) - points["shoulder"]
        lower = np.asarray(points["wrist"], dtype=float) - points["elbow"]
        if (upper.shape != (3,) or lower.shape != (3,)
                or not np.all(np.isfinite([upper, lower]))
                or min(np.linalg.norm(upper), np.linalg.norm(lower)) < 1.e-6):
            raise ValueError("finite, nonzero arm segments required for hand orientation")
        upper /= np.linalg.norm(upper)
        x = lower / np.linalg.norm(lower)
        cross = np.cross(upper, x)
        strength = float(np.linalg.norm(cross))
        self.plane_observable = strength >= np.sin(np.deg2rad(4.))
        if self.rotation is None:
            if self.plane_observable:
                z = cross / strength
            else:
                # A straight arm has no measurable bend plane. Choose a
                # deterministic initial gauge, then transport it continuously.
                axis = np.eye(3)[int(np.argmin(np.abs(x)))]
                z = np.cross(x, axis)
                z /= np.linalg.norm(z)
        else:
            old_x = self.rotation[:, 0]
            turn = np.cross(old_x, x)
            sine = float(np.linalg.norm(turn))
            cosine = float(np.clip(old_x @ x, -1., 1.))
            if sine > 1.e-9:
                transport = Rotation.from_rotvec(turn/sine * np.arctan2(sine, cosine)).as_matrix()
            elif cosine < 0:
                transport = Rotation.from_rotvec(self.rotation[:, 1] * np.pi).as_matrix()
            else:
                transport = np.eye(3)
            z = transport @ self.rotation[:, 2]
            if self.plane_observable:
                goal = cross / strength
                # Crossing an almost straight elbow must not flip the hand by
                # 180 degrees because an unobservable plane changed sign.
                if goal @ z < 0:
                    goal = -goal
                blend = float(np.clip((strength-np.sin(np.deg2rad(4.))) /
                                      (np.sin(np.deg2rad(10.))-np.sin(np.deg2rad(4.))), 0., 1.))
                angle = np.arctan2(x @ np.cross(z, goal), np.clip(z @ goal, -1., 1.))
                z = Rotation.from_rotvec(x * angle * blend).apply(z)
        z -= x * (x @ z)
        z /= np.linalg.norm(z)
        # y = z cross x is essential: [x, x cross z, z] is a reflection.
        self.rotation = np.column_stack((x, np.cross(z, x), z))
        return self.rotation.copy()


class RelativeHandOrientation:
    def __init__(self, points, hand_rotation, *, palm_rotation=None):
        self.forearm = ForearmFrame()
        carrier = self.forearm.update(points)
        self.reference_local = carrier.T @ np.asarray(hand_rotation, dtype=float)
        self.palm_reference = None if palm_rotation is None else checked_rotation(palm_rotation)
        self.palm_reference_clock = (None if self.palm_reference is None else
                                     palm_clock(carrier, self.palm_reference[:, 0]))
        self.palm_reference_local = (None if self.palm_reference is None else carrier.T @ self.palm_reference)
        self.raw_delta = np.eye(3)
        self.filtered = np.eye(3)
        self.diagnostics = {}

    def reset_filter(self):
        """Restart engagement from the held hand pose, preserving calibration."""
        self.filtered = np.eye(3)

    def update(self, points, hand_rotation, max_angle_deg, max_step_deg, *, full_target=False):
        carrier = self.forearm.update(points)
        local = carrier.T @ np.asarray(hand_rotation, dtype=float)
        delta = local @ self.reference_local.T
        self.raw_delta = delta.copy()
        rotvec = Rotation.from_matrix(delta).as_rotvec()
        angle = float(np.linalg.norm(rotvec))
        maximum = np.deg2rad(max_angle_deg)
        if angle > maximum:
            rotvec *= maximum / angle
        target = Rotation.from_rotvec(rotvec).as_matrix()
        limited_roll = forearm_twist_deg(target)
        step = Rotation.from_matrix(target @ self.filtered.T).as_rotvec()
        step_angle = float(np.linalg.norm(step))
        step_limit = np.deg2rad(max_step_deg)
        if not full_target:
            if step_angle > step_limit:
                step *= step_limit / step_angle
            target = Rotation.from_rotvec(step).as_matrix() @ self.filtered
            self.filtered = target.copy()
        # Entry inspection may observe a full relative target, but must not
        # pre-load it into the first ACTIVE command's rate-limit history.
        self.diagnostics = dict(
            source="PICO Body wrist relative to forearm",
            orientation_mode="forearm_relative",
            requested_delta_deg=float(np.degrees(angle)),
            limited_delta_deg=float(np.degrees(np.linalg.norm(rotvec))),
            source_angle_capped=bool(angle > maximum),
            step_limited=bool(not full_target and step_angle > step_limit),
            requested_step_deg=float(np.degrees(step_angle)),
            forearm_plane_observable=bool(self.forearm.plane_observable),
            relative_rotation=target.tolist(),
            palm_reference_clock_deg=(None if self.palm_reference_clock is None else
                                      float(np.degrees(self.palm_reference_clock))),
            requested_roll_deg=forearm_twist_deg(delta),
            limited_roll_deg=limited_roll,
            filtered_roll_deg=forearm_twist_deg(target),
        )
        return target


# Columns are palm normal, little-to-index lateral, and finger direction.
# This fixed anatomical neutral is independent of the first captured pose.
ANATOMICAL_NEUTRAL = np.array([[0., 0., 1.], [1., 0., 0.], [0., 1., 0.]])


class AbsoluteHandOrientation(RelativeHandOrientation):
    """Track the current anatomical palm in its arm-carried forearm frame.

    Calibration captures the first approach target and a Body-to-palm basis
    for passive diagnostics. It never subtracts that pose from control input.
    SO(3) angle and combined joint-norm caps do not define reachability;
    bounded wrist IK and the robot trajectory enforce the individual limits.
    """

    def __init__(self, points, hand_rotation, *, palm_rotation):
        self.forearm = ForearmFrame()
        carrier = self.forearm.update(points)
        self.reference_local = carrier.T @ checked_rotation(hand_rotation)
        self.palm_reference = checked_rotation(palm_rotation)
        self.palm_reference_local = carrier.T @ self.palm_reference
        normal = self.palm_reference_local[:, 0]
        self.palm_reference_clock = (float(np.arctan2(normal[2], normal[1]))
                                     if np.linalg.norm(normal[1:]) > 1.e-6 else None)
        self.initial_local = self.palm_reference_local.copy()
        self.source_local = self.initial_local.copy()
        self.body_to_palm = checked_rotation(hand_rotation).T @ self.palm_reference
        self.filtered = self.initial_local.copy()
        self.raw_delta = np.eye(3)
        self.diagnostics = {}

    def reset_filter(self, local=None):
        self.filtered = checked_rotation(self.initial_local if local is None else local)

    def update(self, points, hand_rotation, max_angle_deg=None, max_step_deg=2., *,
               palm_rotation, full_target=False):
        carrier = self.forearm.update(points)
        body = checked_rotation(hand_rotation)
        palm = checked_rotation(palm_rotation)
        local = checked_rotation(carrier.T @ palm)
        self.source_local = local.copy()
        self.raw_delta = carrier.T @ body @ self.reference_local.T
        body_local = carrier.T @ body @ self.body_to_palm
        step = Rotation.from_matrix(local @ self.filtered.T).as_rotvec()
        step_angle = float(np.linalg.norm(step))
        step_limit = np.deg2rad(max_step_deg)
        if not np.isfinite(step_limit) or step_limit <= 0:
            raise ValueError("positive finite wrist step limit required")
        target = local
        if not full_target:
            if step_angle > step_limit:
                step *= step_limit / step_angle
                target = Rotation.from_rotvec(step).as_matrix() @ self.filtered
            # When the full step fits, use the source matrix directly. Avoid
            # an unnecessary SO(3) round trip and preserve exact equality for
            # reusing the source/command IK result after the filter catches up.
            self.filtered = target.copy()
        requested = local @ ANATOMICAL_NEUTRAL.T
        filtered = target @ ANATOMICAL_NEUTRAL.T
        self.diagnostics = dict(
            source="PICO Hand anatomical palm", orientation_mode="forearm_absolute",
            requested_delta_deg=float(np.degrees(Rotation.from_matrix(requested).magnitude())),
            limited_delta_deg=float(np.degrees(Rotation.from_matrix(requested).magnitude())),
            source_angle_capped=False,
            step_limited=bool(not full_target and step_angle > step_limit),
            requested_step_deg=float(np.degrees(step_angle)),
            forearm_plane_observable=bool(self.forearm.plane_observable),
            absolute_local_orientation=target.tolist(),
            requested_roll_deg=forearm_twist_deg(requested),
            limited_roll_deg=forearm_twist_deg(requested),
            filtered_roll_deg=forearm_twist_deg(filtered),
            hand_palm_roll_deg=forearm_twist_deg(requested),
            body_absolute_roll_deg=forearm_twist_deg(body_local @ ANATOMICAL_NEUTRAL.T),
            body_palm_disagreement_deg=float(np.degrees(
                Rotation.from_matrix(body_local @ local.T).magnitude())),
            palm_reference_clock_deg=(None if self.palm_reference_clock is None else
                                      float(np.degrees(self.palm_reference_clock))),
        )
        return target
