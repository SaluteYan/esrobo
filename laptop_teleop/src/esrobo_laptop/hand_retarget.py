"""Left L20Lite geometry fitting. Pure computation; no SDK/ROS/CAN access."""
import hashlib
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

from esrobo_teleop.robot.linker_hand_driver import ACTIVE_HAND_JOINTS
from .hand_participation import FingerParticipation, POLICY as PARTICIPATION_POLICY
from .hand_contact import HandContact, POLICY as CONTACT_POLICY

VERSION = "left-urdf-10-v8"
POLICY = dict(pinch_start_width=1.1, pinch_ramp_width=.6,
              project_target_m=.002,
              project_transition_s=.080, clock_gap_s=.1,
              release_thumb_transition=True, participation_thumb_transition_s=.080, fixed_finger_contact="posture_soft_vector_v1",
              flex_correction_deg=15., side_correction_deg=5.,
              direction_weights=(5.3, 3., 3.), open_direction_weights=(4., 1., 1.), plane_weight=.02,
              temporal_weight=.001, finger_regularization=.01,
              huber_delta=.2, contact_weight=100., initial_contact_weight=10., opening_vector_weight=.1, contact_tolerance_m=.002,
              damping=.001, max_updates=6, seed_grid=(7, 7, 5))
FINGERS = ("thumb", "index", "middle", "ring", "pinky")
TIPS = np.array([5, 10, 15, 20, 25])
BONES = np.array([(a, a+1) for start in (2, 7, 12, 17, 22) for a in range(start, start+3)])
MANUAL_DEG = dict(thumb_cmc_pitch=27.9, thumb_mcp=38.4, thumb_ip=37.9,
                  thumb_cmc_yaw=69., thumb_cmc_roll=60., index_mcp_roll=10.7,
                  ring_mcp_roll=11.7, pinky_mcp_roll=17.2)
for _finger in FINGERS[1:]:
    MANUAL_DEG.update({f"{_finger}_mcp_pitch": 77.9, f"{_finger}_pip": 102., f"{_finger}_dip": 35.8})


def fingerprint(cfg):
    # Hash the geometry and the exact control policy, not machine-specific paths.
    from dataclasses import asdict
    model = Path(cfg.ik.urdf_path)
    value = dict(version=VERSION, policy=POLICY, participation_policy=PARTICIPATION_POLICY,
                 contact_policy=CONTACT_POLICY, contact_source=hashlib.sha256(Path(__file__).with_name("hand_contact.py").read_bytes()).hexdigest(),
                 participation_source=hashlib.sha256(Path(__file__).with_name("hand_participation.py").read_bytes()).hexdigest(), hand=asdict(cfg.hand), manual=MANUAL_DEG,
                 urdf=hashlib.sha256(model.read_bytes()).hexdigest(),
                 solver_kernel=hashlib.sha256(Path(__file__).with_name("_hand_native.cpp").read_bytes()).hexdigest())
    for finger in FINGERS:
        mesh = model.parent.parent / "assets/esrobo_waist_with_head/meshes/linkerhand_l20lite/left" / (finger+"_distal.STL")
        value[finger] = hashlib.sha256(mesh.read_bytes()).hexdigest()
    return hashlib.sha256(json.dumps(value, sort_keys=True, allow_nan=False).encode()).hexdigest()


class HandModel:
    def __init__(self, cfg):
        self.digest = fingerprint(cfg)
        root = ET.parse(cfg.ik.urdf_path).getroot()
        self.names = list(ACTIVE_HAND_JOINTS)
        self.lower, self.upper = np.zeros(10), np.full(10, np.inf)
        self.joints = []
        self.joint_index = {}
        links = {"leftHand_link": -1}
        pending = [j for j in root.findall("joint") if j.get("name", "").removeprefix("left_") in MANUAL_DEG and j.get("name", "").startswith("left_")]
        while pending:
            progressed = False
            for j in pending[:]:
                parent = j.find("parent").get("link")
                if parent not in links:
                    continue
                name = j.get("name")[5:]
                origin = j.find("origin")
                offset = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")
                rot = Rotation.from_euler("xyz", np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")).as_matrix()
                axis = np.fromstring(j.find("axis").get("xyz"), sep=" ")
                axis /= np.linalg.norm(axis)
                mimic = j.find("mimic")
                source = name if mimic is None else mimic.get("joint")[5:]
                ratio = 1. if mimic is None else float(mimic.get("multiplier", "1"))
                bias = 0. if mimic is None else float(mimic.get("offset", "0"))
                k = self.names.index(source)
                limit = j.find("limit")
                self.lower[k] = max(self.lower[k], (float(limit.get("lower"))-bias)/ratio)
                self.upper[k] = min(self.upper[k], (min(float(limit.get("upper")), np.deg2rad(MANUAL_DEG[name]))-bias)/ratio)
                index = len(self.joints)
                self.joints.append((links[parent], offset, rot, axis, k, ratio, bias))
                self.joint_index[name] = index
                links[j.find("child").get("link")] = index
                pending.remove(j)
                progressed = True
            if not progressed:
                raise ValueError("unresolved left hand URDF chain")
        if len(self.joints) != 20 or not np.isfinite(self.upper).all():
            raise ValueError("L20Lite model must have 10 active and 10 mimic joints")
        # Restrict solver to the independently configured command envelope.
        calibration = cfg.hand.left_control_calibration
        for k, name in enumerate(self.names):
            item = calibration[name]
            self.lower[k] = max(self.lower[k], min(item.get('allowed_rad', item['rad'])))
            self.upper[k] = min(self.upper[k], max(item.get('allowed_rad', item['rad'])))
        self.tip_offsets = []
        for finger in FINGERS:
            mesh = Path(cfg.ik.urdf_path).parent.parent / "assets/esrobo_waist_with_head/meshes/linkerhand_l20lite/left" / (finger+"_distal.STL")
            data = mesh.read_bytes()
            count = int.from_bytes(data[80:84], "little")
            triangles = np.frombuffer(data, dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("attr", "<u2")]), count=count, offset=84)
            vertices = np.unique(triangles["v"].reshape(-1, 3), axis=0).astype(float)
            z = vertices[:, 2].max()
            cap = vertices[vertices[:, 2] >= z-.001]
            tip = cap.mean(axis=0)
            tip[2] = z
            self.tip_offsets.append(tip)
        self.neutral = np.zeros(10)
        for k, name in enumerate(self.names):
            item = calibration[name]
            from esrobo_teleop.robot.linker_hand_driver import L10_PHYSICAL_JOINT_NAMES
            raw = cfg.hand.left_open[L10_PHYSICAL_JOINT_NAMES.index(name)]
            self.neutral[k] = np.interp(raw, sorted(item["raw"]), item["rad"] if item["raw"][0] < item["raw"][1] else item["rad"][::-1])
        self.neutral = np.clip(self.neutral, self.lower, self.upper)
        self._rot = np.array([j[2] for j in self.joints])
        axes = np.array([j[3] for j in self.joints])
        self._skew = np.zeros((len(axes), 3, 3))
        self._skew[:, 0, 1], self._skew[:, 0, 2] = -axes[:, 2], axes[:, 1]
        self._skew[:, 1, 0], self._skew[:, 1, 2] = axes[:, 2], -axes[:, 0]
        self._skew[:, 2, 0], self._skew[:, 2, 1] = -axes[:, 1], axes[:, 0]
        self._skew2 = self._skew @ self._skew
        self._indices = np.array([j[4] for j in self.joints])
        self._ratios = np.array([j[5] for j in self.joints])
        self._biases = np.array([j[6] for j in self.joints])
        self._thumb_weights = np.zeros((5, 3))
        self._thumb_weights[np.arange(5), self._indices[:5]] = self._ratios[:5]
        self._thumb_ancestry = np.arange(5)[None, :] < np.array([2, 3, 4, 5])[:, None]
        # Four independent chains, padded with an identity side-swing joint for
        # the middle finger. Evaluate all four together, with only two local
        # derivatives per finger (side swing, flexion).
        chains = []
        identity = (-1, np.zeros(3), np.eye(3), np.zeros(3), 0, 0., 0.)
        for finger in FINGERS[1:]:
            chains.append([identity if finger == 'middle' else self.joints[self.joint_index[finger+'_mcp_roll']]] +
                          [self.joints[self.joint_index[finger+name]] for name in ('_mcp_pitch', '_pip', '_dip')])
        self._finger_offsets = np.array([[j[1] for j in chain] for chain in chains])
        self._finger_rot = np.array([[j[2] for j in chain] for chain in chains])
        self._finger_axes = np.array([[j[3] for j in chain] for chain in chains])
        self._finger_ratios = np.array([[j[5] for j in chain] for chain in chains])
        self._finger_biases = np.array([[j[6] for j in chain] for chain in chains])
        a = self._finger_axes
        self._finger_skew = np.zeros((4, 4, 3, 3))
        self._finger_skew[:, :, 0, 1], self._finger_skew[:, :, 0, 2] = -a[:, :, 2], a[:, :, 1]
        self._finger_skew[:, :, 1, 0], self._finger_skew[:, :, 1, 2] = a[:, :, 2], -a[:, :, 0]
        self._finger_skew[:, :, 2, 0], self._finger_skew[:, :, 2, 1] = -a[:, :, 1], a[:, :, 0]
        self._finger_skew2 = self._finger_skew @ self._finger_skew
        points, _ = self.forward(self.neutral)
        from .pico_hand import palm_basis
        poses = np.zeros((26, 7)); poses[:, :3] = points; poses[:, 6] = 1
        self.basis = palm_basis(poses)
        self.origin = points[[7, 12, 17, 22]].mean(axis=0)
        self.width = np.linalg.norm(points[7]-points[22])

    def thumb_landmarks(self, angles):
        """Thumb CMC/MCP/IP/tip with its exact three-column Jacobian only."""
        q = np.asarray(angles)[self._indices[:5]]*self._ratios[:5]+self._biases[:5]
        local = self._rot[:5] @ (np.eye(3)+np.sin(q)[:, None, None]*self._skew[:5]+
                               (1-np.cos(q))[:, None, None]*self._skew2[:5])
        origins = np.empty((5, 3)); axes = np.empty((5, 3))
        p = np.zeros(3); r = np.eye(3)
        for i, joint in enumerate(self.joints[:5]):
            p = p+r @ joint[1]
            origins[i] = p
            axes[i] = r @ joint[2] @ joint[3]
            r = r @ local[i]
        points = np.vstack((origins[2:5], p+r @ self.tip_offsets[0]))
        d = points[:, None, :]-origins[None, :, :]
        cross = np.stack((axes[:, 1]*d[:, :, 2]-axes[:, 2]*d[:, :, 1],
                          axes[:, 2]*d[:, :, 0]-axes[:, 0]*d[:, :, 2],
                          axes[:, 0]*d[:, :, 1]-axes[:, 1]*d[:, :, 0]), axis=2)
        jac = np.einsum('pjc,jk->pck', cross*self._thumb_ancestry[:, :, None], self._thumb_weights)
        return points, jac

    def forward_fingers(self, angles, finger_ids=None):
        """MCP/PIP/DIP/tip positions and exact 2-axis local Jacobians.

        ``angles`` is (4, 2); the middle finger's first column is ignored.
        This uses the same URDF transforms/mimics as forward(), without any
        thumb or unrelated actuator derivatives.
        """
        ids = np.arange(4) if finger_ids is None else np.asarray(finger_ids, dtype=int)
        count = len(ids)
        angles = np.asarray(angles)[ids]
        q = angles[:, [0, 1, 1, 1]]*self._finger_ratios[ids]+self._finger_biases[ids]
        local = self._finger_rot[ids] @ (np.eye(3)+np.sin(q)[:, :, None, None]*self._finger_skew[ids]+
                                   (1-np.cos(q))[:, :, None, None]*self._finger_skew2[ids])
        p = np.zeros((count, 3)); r = np.broadcast_to(np.eye(3), (count, 3, 3)).copy()
        origins = np.empty((count, 4, 3)); axes = np.empty_like(origins)
        offsets, rotations = self._finger_offsets[ids], self._finger_rot[ids]
        joint_axes, ratios = self._finger_axes[ids], self._finger_ratios[ids]
        for i in range(4):
            p += (r @ offsets[:, i, :, None])[:, :, 0]
            origins[:, i] = p
            axes[:, i] = (r @ rotations[:, i] @ joint_axes[:, i, :, None])[:, :, 0]
            r = r @ local[:, i]
        tip = p+(r @ np.asarray(self.tip_offsets[1:])[ids][:, :, None])[:, :, 0]
        points = np.concatenate((origins[:, 1:], tip[:, None]), axis=1)
        delta = points[:, :, None]-origins[:, None]
        cross = np.stack((axes[:, None, :, 1]*delta[..., 2]-axes[:, None, :, 2]*delta[..., 1],
                          axes[:, None, :, 2]*delta[..., 0]-axes[:, None, :, 0]*delta[..., 2],
                          axes[:, None, :, 0]*delta[..., 1]-axes[:, None, :, 1]*delta[..., 0]), axis=-1)
        ancestry = np.arange(4)[None, :] < np.arange(1, 5)[:, None]
        weights = np.zeros((count, 4, 2)); weights[:, 0, 0] = ratios[:, 0]
        weights[:, 1:, 1] = ratios[:, 1:]
        jac = np.einsum('fpjc,fjk->fpck', cross*ancestry[None, :, :, None], weights)
        return points, jac

    def forward(self, q, *, thumb_only=False):
        """26 landmarks and their exact derivatives, including mimic chains."""
        n = 5 if thumb_only else len(self.joints)
        angles = np.asarray(q)[self._indices[:n]]*self._ratios[:n]+self._biases[:n]
        local = self._rot[:n] @ (np.eye(3) + np.sin(angles)[:, None, None]*self._skew[:n]
                                + (1-np.cos(angles))[:, None, None]*self._skew2[:n])
        def cross_columns(w, d):
            return np.array([w[1]*d[2]-w[2]*d[1], w[2]*d[0]-w[0]*d[2], w[0]*d[1]-w[1]*d[0]])
        positions = np.zeros((n, 3)); rotations = np.zeros((n, 3, 3))
        angular = np.zeros((n, 3, 10)); jac = np.zeros((n, 3, 10))
        for i, (parent, offset, rot, axis, k, ratio, bias) in enumerate(self.joints[:n]):
            R = np.eye(3) if parent < 0 else rotations[parent]
            p = np.zeros(3) if parent < 0 else positions[parent]
            w = np.zeros((3, 10)) if parent < 0 else angular[parent]
            dp = np.zeros((3, 10)) if parent < 0 else jac[parent]
            d = R @ offset
            positions[i] = p+d
            jac[i] = dp+cross_columns(w, d)
            base = R @ rot
            rotations[i] = R @ local[i]
            angular[i] = w
            angular[i, :, k] += base @ axis * ratio
        points = np.zeros((26, 3)); derivatives = np.zeros((26, 3, 10))
        def landmark(slot, name, offset=None):
            i = self.joint_index[name]
            d = np.zeros(3) if offset is None else rotations[i] @ offset
            points[slot] = positions[i]+d
            derivatives[slot] = jac[i]+cross_columns(angular[i], d)
        for f, finger in enumerate(FINGERS[:1] if thumb_only else FINGERS):
            if f == 0:
                for slot, name in ((2, "thumb_cmc_pitch"), (3, "thumb_mcp"), (4, "thumb_ip")):
                    landmark(slot, name)
                landmark(5, "thumb_ip", self.tip_offsets[f])
            else:
                start = 6+(f-1)*5
                if finger == 'middle':
                    landmark(start, finger+"_mcp_pitch")
                    points[start] -= [0, 0, .02]
                else:
                    landmark(start, finger+"_mcp_roll")
                landmark(start+1, finger+"_mcp_pitch")
                landmark(start+2, finger+"_pip")
                landmark(start+3, finger+"_dip")
                landmark(start+4, finger+"_dip", self.tip_offsets[f])
        points[0] = points[[7, 12, 17, 22]].mean(axis=0)
        derivatives[0] = derivatives[[7, 12, 17, 22]].mean(axis=0)
        return points, derivatives

    def human_target(self, poses):
        from .pico_hand import palm_basis
        poses = np.asarray(poses, dtype=float)
        basis = palm_basis(poses)
        p = poses[:, :3]
        width = np.linalg.norm(p[7]-p[22])
        if not .015 <= width <= .15:
            raise ValueError("invalid human palm width")
        origin = p[[7, 12, 17, 22]].mean(axis=0)
        return (p-origin) @ basis @ self.basis.T * (self.width/width)+self.origin


class HandRetargeter:
    def __init__(self, cfg, *, use_native=True):
        from .hand_shape_filter import HandShapeFilter
        self.shape_filter = HandShapeFilter(cfg.hand, POLICY['clock_gap_s'])
        self.model = HandModel(cfg)
        self.previous = self.model.neutral.copy()
        self.participation = FingerParticipation(self.model.lower, self.model.upper, self.model.width,
            flex_deg=POLICY["flex_correction_deg"], side_deg=POLICY["side_correction_deg"],
            max_gap_s=POLICY["clock_gap_s"])
        self.contact_detector = HandContact()
        self.contact = np.zeros(4, dtype=bool)
        self.permission_mask = np.zeros(4, dtype=bool)
        self.permission_anchor = self.previous[:3].copy()
        self.permission_progress = 1.
        self.projection = np.zeros(4)
        self.source_time = None
        self._pending = None
        self._serial = 0
        m = self.model
        zero, _ = m.forward(np.zeros(10))
        self.finger_geometry = []
        for f, name in enumerate(FINGERS[1:]):
            k = m.names.index(name+'_mcp_pitch')
            root = m.joints[m.joint_index[name+('_mcp_pitch' if name == 'middle' else '_mcp_roll')]]
            base, axis = root[2], None if name == 'middle' else root[3]
            idx = np.arange(7+f*5, 11+f*5)
            # Ratios come from the actual URDF, including its passive linkage.
            pip = m.joints[m.joint_index[name+'_pip']][5]
            dip = m.joints[m.joint_index[name+'_dip']][5]
            self.finger_geometry.append((k, base, axis, idx, np.diff(zero[idx], axis=0) @ base,
                                         np.array([1., 1.+pip, 1.+pip+dip])))

        self.rest_points, _ = m.forward(m.neutral)
        self.segment_lengths = np.array([np.linalg.norm(np.diff(self.rest_points[start:start+4], axis=0), axis=1)
                                         for start in (2, 7, 12, 17, 22)])
        self.finger_lower = self._finger_angles(m.lower)
        self.finger_upper = self._finger_angles(m.upper)
        # Deterministic URDF orientation seeds avoid a bound-local solution.
        # This table is constructed once; online selection is one vector norm.
        self.thumb_seed_angles = np.array(np.meshgrid(*[
            np.linspace(m.lower[k], m.upper[k], POLICY['seed_grid'][k])
            for k in range(3)], indexing='ij')).reshape(3, -1).T
        self.thumb_seed_directions = np.array([self._directions(m.thumb_landmarks(q)[0])
                                               for q in self.thumb_seed_angles])
        self._native = None
        if use_native:
            from .hand_native import load_model
            self._native = load_model(m, self.rest_points, self.thumb_seed_angles, POLICY)
        if use_native and self._native is None:
            raise RuntimeError('left hand native kernel is missing; run laptop_teleop/scripts/build_hand_native.sh')
        if self._native is not None:
            from .hand_native import _MODULE
            self.participation.native = _MODULE.participate
        self.backend = 'native_cpp' if self._native is not None else 'python_reference'

    def direct_fingers(self, target):
        """Signed four-finger geometric seed; URDF correction follows for pinch."""
        if self._native is not None:
            return self._native.direct_fingers(target, self.previous)
        m = self.model
        q = self.previous.copy()
        for k, base, axis, idx, neutral, coefficients in self.finger_geometry:
            bones = np.diff(target[idx], axis=0) @ base
            if axis is not None:
                sign = np.sign(axis[0])
                roll = (np.arctan2(-sign*bones[0, 1], bones[0, 2])
                        - np.arctan2(-sign*neutral[0, 1], neutral[0, 2]))
                q[k-1] = np.clip(roll, m.lower[k-1], m.upper[k-1])
                bones = bones @ Rotation.from_rotvec(axis*q[k-1]).as_matrix()
            angles = np.arctan2(bones[:, 0], bones[:, 2])-np.arctan2(neutral[:, 0], neutral[:, 2])
            # Unwrap bone rotations (not bounded actuator angles) at deep flexion.
            angles += 2*np.pi*np.rint((coefficients*angles[0]-angles)/(2*np.pi))
            q[k] = np.clip(coefficients @ angles / (coefficients @ coefficients), m.lower[k], m.upper[k])
        return q

    @staticmethod
    def _directions(points, jac=None):
        vectors = np.diff(points, axis=-2)
        lengths = np.linalg.norm(vectors, axis=-1)
        if np.any(lengths < 1e-5):
            raise ValueError('degenerate PICO finger bone')
        directions = vectors/lengths[..., None]
        if jac is None:
            return directions
        dv = np.diff(jac, axis=-3)
        derivative = (dv-directions[..., None]*np.sum(directions[..., None]*dv, axis=-2)[..., None, :])/lengths[..., None, None]
        return directions, derivative

    def reconstruct_target(self, target, q=None, points=None):
        """Diagnostic overlay only; movable roots come from the candidate FK.

        PICO 2/3/4/5 (CMC/MCP/IP/tip) correspond to model pitch-origin,
        MCP-origin, IP-origin and the distal mesh tip. No fixed-root position
        enters the solver objective. Directions, unlike roots, are transferable.
        """
        roots = points if points is not None else self.model.forward(self.previous if q is None else q)[0]
        rebuilt = target.copy()
        for f, start in enumerate((2, 7, 12, 17, 22)):
            rebuilt[start] = roots[start]
            rebuilt[start+1:start+4] = roots[start]+np.cumsum(
                self._directions(target[start:start+4])*self.segment_lengths[f, :, None], axis=0)
        return rebuilt

    @staticmethod
    def _finger_angles(q):
        return np.array([[q[3], q[4]], [0., q[5]], [q[6], q[7]], [q[8], q[9]]])

    def commit(self, candidate_id):
        """Publish acknowledgement; stale acknowledgements cannot commit state."""
        if self._pending is None or self._pending[0] != candidate_id:
            return False
        _, self.previous, self.contact, self.projection, self.source_time, shape, participation, self.permission_mask, self.permission_anchor, self.permission_progress, contact_state = self._pending
        self.contact_detector.commit(contact_state)
        self.participation.commit(participation)
        if shape is not None:
            self.shape_filter.commit(shape)
        self._pending = None
        return True

    def discard(self):
        self._pending = None

    @staticmethod
    def _plane(directions, jac):
        def cross(a, b):
            return np.array([a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]])
        cross_value = cross(directions[0], directions[2])
        dc = cross(jac[0], directions[2, :, None])+cross(directions[0, :, None], jac[2])
        norm = np.sqrt(cross_value@cross_value+np.sin(np.deg2rad(5.))**2)
        return cross_value/norm, (np.eye(3)/norm-np.outer(cross_value, cross_value)/norm**3)@dc

    def solve(self, poses, *, budget_s=.020, target=None, source_time=None, commit=True):
        started = time.monotonic()
        deadline = started+min(budget_s, 1.)
        self.discard()
        def check():
            if time.monotonic() >= deadline:
                raise TimeoutError('left hand solver exceeded 20 ms')
        check()
        stamp = started if source_time is None else float(source_time)
        if not np.isfinite(stamp):
            raise ValueError('invalid hand source time')
        m = self.model
        synthetic = target is not None
        target = m.human_target(poses) if target is None else np.asarray(target, dtype=float)
        if target.shape != (26, 3) or not np.isfinite(target).all():
            raise ValueError('invalid hand target')
        desired = self._directions(target[2:6])
        # Also validate bones used by the direct geometric mapper.
        for a in (7, 12, 17, 22):
            self._directions(target[a:a+4])
        raw_target = target
        shape = None
        shape_diagnostic = dict(enabled=False, bypass='explicit_model_fixture')
        if not synthetic:
            shape, shape_diagnostic = self.shape_filter.propose(target, stamp)
            target = shape.output
            desired = self._directions(target[2:6])
            for a in (7, 12, 17, 22):
                self._directions(target[a:a+4])
        pair = target[TIPS[1:]]-target[5]
        distance = np.linalg.norm(pair, axis=1)
        raw = target if synthetic else np.asarray(poses)[:, :3]
        human_distance = np.linalg.norm(raw[TIPS[1:]]-raw[5], axis=1)
        contact_started = time.monotonic()
        contact_state, contact_diagnostic = self.contact_detector.propose(human_distance, target, m.width, stamp)
        contact, projection = contact_state.active, contact_state.projection
        reset = contact_diagnostic["clock_reset"]
        contact_ms = (time.monotonic()-contact_started)*1000
        if synthetic:
            # Explicit model fixtures retain their exact attainable vectors.
            projection = np.zeros(4)
        # Preserve the observed contact direction. Smoothing shapes must not
        # change a fully projected 2 mm contact into a different pinch target.
        projected = raw_target[TIPS[1:]]-raw_target[5]
        projection_distance = np.linalg.norm(projected, axis=1)
        nonzero = projection_distance > 1e-8
        projected[nonzero] *= (POLICY['project_target_m']/projection_distance[nonzero])[:, None]
        if not nonzero.all():
            # Coincident human tips have no measured direction. Continue the
            # accepted model direction, rather than divide by zero or invent
            # a fresh orientation from noise at every frame.
            previous_thumb, _ = m.thumb_landmarks(self.previous[:3])
            previous_fingers, _ = m.forward_fingers(self._finger_angles(self.previous))
            fallback = previous_fingers[:, 3]-previous_thumb[3]
            norm = np.linalg.norm(fallback, axis=1)
            fallback[norm < 1e-8] = [1., 0., 0.]
            fallback /= np.linalg.norm(fallback, axis=1)[:, None]
            projected[~nonzero] = fallback[~nonzero]*POLICY['project_target_m']
        pair = pair+(projected-pair)*projection[:, None]
        gates = np.clip((POLICY['pinch_start_width']-distance/m.width)/POLICY['pinch_ramp_width'], 0., 1.)
        gates = np.maximum(gates*gates*(3-2*gates), projection)
        active = np.flatnonzero(contact | (projection > 0.))
        participation_started = time.monotonic()
        geometric_base = self.direct_fingers(target)
        raw_base = geometric_base if synthetic else self.direct_fingers(raw_target)
        permission = self.participation.propose(geometric_base, raw_base, target, stamp, contact)
        base = permission.fixed
        moving_indices = np.flatnonzero(permission.upper[3:]-permission.lower[3:] > 1e-12)+3
        moving = np.array([f for f in active if any(k in moving_indices
            for k in ([5] if f == 1 else [[3,4],[],[6,7],[8,9]][f]))], dtype=int)
        constrained_contact = contact & np.asarray(permission.diagnostic["eligible"])
        strict = constrained_contact[active]
        participation_ms = (time.monotonic()-participation_started)*1000
        target_plane, _ = self._plane(desired, np.zeros((3, 3, 3)))
        plane_confidence = float(np.linalg.norm(target_plane))
        weights = np.array(POLICY['open_direction_weights'])
        weights += max(gates)*(np.array(POLICY['direction_weights'])-weights)
        if self._native is not None:
            packed = self._native.solve(base, self.previous, desired, pair, constrained_contact,
                                        projection, gates, weights, permission.lower, permission.upper, geometric_base,
                                        deadline-time.monotonic())
            q, points, solved_directions = packed['joints'], packed['points'], packed['directions']
            n, evals = packed['dof'], packed['evaluations']
            updates, attempts = packed['updates'], packed['attempts']
            check()
        else:
            indices = np.r_[np.arange(3), moving_indices]
            lower, upper = permission.lower[indices], permission.upper[indices]
            n = len(indices)
            positions = {int(k): i for i, k in enumerate(indices)}
            local_columns = [(positions.get((3, -1, 6, 8)[f]), positions.get((4, 5, 7, 9)[f])) for f in active]
            # Non-participating four fingers are evaluated only once per frame.
            fixed_fingers, _ = m.forward_fingers(self._finger_angles(base))
            plane_scale = np.sqrt(POLICY['plane_weight'])*plane_confidence
            direction_weights = np.sqrt(weights/3)[:, None]
            previous = self.previous[indices]
            regularization = np.full(n, POLICY['temporal_weight'])
            regularization[3:] = POLICY['finger_regularization']
            reference = previous.copy(); reference[3:] = geometric_base[indices[3:]]
            evals = 0

            def evaluate(x):
                nonlocal evals
                check(); evals += 1
                thumb, jt = m.thumb_landmarks(x[:3])
                full_jt = np.zeros((4, 3, n)); full_jt[:, :, :3] = jt
                directions, jd = self._directions(thumb, full_jt)
                plane, jp = self._plane(directions, jd)
                posture = np.r_[((directions-desired)*direction_weights).ravel(),
                                (plane-target_plane)*plane_scale]
                posture_j = np.vstack(((jd*direction_weights[:, :, None]).reshape(-1, n),
                                        jp*plane_scale))
                # Huber IRLS on dimensionless orientation residuals.
                robust = np.sqrt(np.minimum(1., POLICY['huber_delta']/np.maximum(abs(posture), 1e-12)))
                residual = [posture*robust, (x-reference)*np.sqrt(regularization)]
                jac = [posture_j*robust[:, None], np.diag(np.sqrt(regularization))]
                errors = np.zeros(len(active)); excess = np.zeros(len(active))
                constraints = []
                release_cost = 0.
                pair_jacobian = np.repeat((-full_jt[3])[None], 4, axis=0)
                restoration = []; restoration_jac = []
                finger_points = fixed_fingers.copy()
                tips = finger_points[:, 3]
                if len(active):
                    angles = self._finger_angles(base)
                    for f, (side_col, flex_col) in zip(active, local_columns):
                        if flex_col is not None:
                            angles[f, 1] = x[flex_col]
                        if side_col is not None:
                            angles[f, 0] = x[side_col]
                    finger_jac = {}
                    if len(moving):
                        fp, fj = m.forward_fingers(angles, moving)
                        finger_points[moving] = fp
                        finger_jac = dict(zip(moving, fj))
                    for a, (f, cols) in enumerate(zip(active, local_columns)):
                        delta = finger_points[f, 3]-thumb[3]-pair[f]
                        length = np.linalg.norm(delta); errors[a] = length
                        dj = -full_jt[3].copy()
                        for local, col in enumerate(cols):
                            if col is not None:
                                dj[:, col] += finger_jac[f][3, :, local]
                        pair_jacobian[f] = dj
                        if not strict[a]:
                            continue
                        if strict[a] and 1e-8 < length <= POLICY['contact_tolerance_m']:
                            constraints.append((delta@dj/length,
                                max(0., POLICY['contact_tolerance_m']-length-1e-5)))
                        tol = POLICY['contact_tolerance_m']-2e-4
                        excess[a] = max(length-POLICY['contact_tolerance_m']+2e-6, 0.)
                        # Stop minimizing contact once inside the accepted band.
                        if length > POLICY['contact_tolerance_m']:
                            v = delta*(1-tol/length)
                            jv = ((1-tol/length)*np.eye(3)+tol*np.outer(delta, delta)/length**3)@dj
                        else:
                            # Zero contact gradient inside the band. Exact tip
                            # Jacobian supplies a local motion metric (damping),
                            # not a derivative of the constant zero residual.
                            v, jv = np.zeros(3), dj
                        weight = np.sqrt(POLICY['contact_weight']*gates[f])/m.width
                        restoration.append(v*weight); restoration_jac.append(jv*weight)
                        residual.append(v*weight); jac.append(jv*weight)
                soft = np.flatnonzero((~constrained_contact) & (gates > 1e-8))
                if len(soft):
                    weight = np.sqrt(POLICY['opening_vector_weight'])*gates[soft]/m.width
                    delta = tips[soft]-thumb[3]-pair[soft]
                    value = delta*weight[:, None]
                    residual.append(value.ravel())
                    jac.append((pair_jacobian[soft]*weight[:, None, None]).reshape(-1,n))
                    release_cost = float(np.sum(value**2))
                contact_residual = np.concatenate(restoration) if restoration else np.empty(0)
                contact_jacobian = np.vstack(restoration_jac) if restoration else np.empty((0, n))
                residual = np.concatenate(residual); jac = np.vstack(jac)
                pose_cost = float(np.sum(np.where(abs(posture)<=POLICY['huber_delta'], posture**2,
                               2*POLICY['huber_delta']*abs(posture)-POLICY['huber_delta']**2)))
                violation = float(np.max(excess[strict], initial=0.))
                # Micrometre numeric tolerance prevents an infeasible roundoff tail
                # outranking every orientation improvement at the contact boundary.
                key = (violation > 1e-6, violation if violation > 1e-6 else 0.,
                       pose_cost+release_cost, float(np.sum(regularization*(x-reference)**2)))
                return residual, jac, key, thumb, tips, directions, errors, constraints, contact_residual, contact_jacobian, finger_points

            # Seed selection is a merit comparison, not contact-only ranking:
            # a tiny contact advantage must not discard a nearly correct warm pose.
            # Final selection still retains the best contact-priority candidate.
            def initial_cost(candidate):
                key = candidate[2]
                return key[2]+key[3]+POLICY['initial_contact_weight']*(key[1]/m.width)**2

            x = np.clip(previous, lower, upper)
            current = evaluate(x)
            best_x, best = x.copy(), current
            neutral = np.clip(np.r_[m.neutral[:3], base[indices[3:]]], lower, upper)
            alternate = evaluate(neutral)
            if alternate[2] < best[2]: best_x, best = neutral.copy(), alternate
            if initial_cost(alternate) < initial_cost(current):
                x, current = neutral, alternate
            seed_index = np.argmin(np.sum((self.thumb_seed_directions-desired)**2*
                                          weights[None, :, None], axis=(1, 2)))
            seed = np.clip(np.r_[self.thumb_seed_angles[seed_index], base[indices[3:]]], lower, upper)
            seeded = evaluate(seed)
            if seeded[2] < best[2]: best_x, best = seed.copy(), seeded
            if initial_cost(seeded) < initial_cost(current):
                x, current = seed, seeded
            damping = POLICY['damping']
            updates = attempts = stalled = 0
            for _ in range(POLICY['max_updates']):
                check()
                residual, jac = current[:2]
                if current[2][0]:
                    # Contact-first restoration: posture cannot balance a nonzero
                    # contact violation into a stationary point outside the band.
                    residual, jac = current[8:10]
                gradient = jac.T@residual
                hessian = jac.T@jac+np.eye(n)*damping
                # Determine active bounds from the coupled Newton direction,
                # not from each raw gradient component: cross-axis terms can turn
                # a seemingly inward gradient into an outward proposed step.
                blocked = np.zeros(n, dtype=bool)
                for _ in range(n+1):
                    constrained_hessian = hessian.copy()
                    constrained_gradient = gradient.copy()
                    constrained_hessian[blocked, :] = 0.
                    constrained_hessian[:, blocked] = 0.
                    constrained_hessian[blocked, blocked] = 1.
                    constrained_gradient[blocked] = 0.
                    step = np.linalg.solve(constrained_hessian, -constrained_gradient)
                    outward = ((x <= lower+1e-9)&(step<0)) | ((x >= upper-1e-9)&(step>0))
                    if not np.any(outward & ~blocked):
                        hessian = constrained_hessian
                        break
                    blocked |= outward
                # Project the proposed posture step into already satisfied contact
                # half-spaces. This avoids repeatedly trying an outward step at the
                # tolerance boundary. FK still checks both finite trial steps.
                for _ in range(2):
                    for normal, margin in current[7]:
                        normal = normal.copy(); normal[blocked] = 0.
                        violation = normal@step-margin
                        if violation > 0:
                            direction = np.linalg.solve(hessian, normal)
                            step -= direction*violation/max(normal@direction, 1e-12)
                # Scale the whole coupled step: per-axis clipping destroys its
                # contact-preserving direction, especially for thumb yaw/roll.
                scale_bound = min(1., .3/max(np.max(abs(step)), 1e-12))
                positive, negative = step > 1e-12, step < -1e-12
                if np.any(positive):
                    scale_bound = min(scale_bound, np.min((upper[positive]-x[positive])/step[positive]))
                if np.any(negative):
                    scale_bound = min(scale_bound, np.min((lower[negative]-x[negative])/step[negative]))
                step *= max(scale_bound, 0.)
                if np.max(abs(step)) < 1e-7:
                    break
                accepted = False
                for scale in (1., .5):
                    attempts += 1
                    trial_x = np.clip(x+scale*step, lower, upper)
                    trial = evaluate(trial_x)
                    if trial[2] < current[2]:
                        x, current = trial_x, trial
                        damping = max(damping*.5, 1e-6)
                        accepted = True
                        stalled = 0
                        break
                updates += 1
                if not accepted:
                    stalled += 1
                    damping = min(damping*10, 10.)
                    if stalled >= 2:
                        break
            check()
            if best[2] < current[2]:
                x, current = best_x, best
            q = base.copy(); q[indices] = x
            # Reuse the accepted exact chain FK; a second full-hand FK would
            # duplicate every active transform and compute unused derivatives.
            points = self.rest_points.copy()
            points[2:6] = current[3]
            for f, start in enumerate((7, 12, 17, 22)):
                points[start:start+4] = current[10][f]
            points[0] = points[[7, 12, 17, 22]].mean(axis=0)
            solved_directions = current[5]
        release_blend = float(max(projection)) if not contact.any() else 0.
        release_applied = (release_blend > 0. and POLICY['release_thumb_transition']
                           and self.permission_progress >= 1.
                           and np.array_equal(constrained_contact, self.permission_mask))
        if release_applied:
            # No contact remains constrained. Fade in the unconstrained thumb
            # solution across the existing 80 ms release, rather than jumping
            # to it when the final contact bit clears. Blend all three thumb
            # axes together; four-finger coordination bounds remain unchanged.
            q = q.copy()
            q[:3] = self.previous[:3]+(0. if reset else 1.-release_blend)*(q[:3]-self.previous[:3])
            points, _ = m.forward(q)
            solved_directions = self._directions(points[2:6])
        # Permission is a discrete decision; changing its contact priority must
        # not instantly replace the accepted thumb posture. Interpolate the
        # three bounded joints together from the accepted anchor for 80 ms.
        # This intentionally permits a residual contact gap during transition.
        permission_changed = not np.array_equal(constrained_contact, self.permission_mask)
        permission_anchor = self.previous[:3].copy() if permission_changed else self.permission_anchor
        permission_progress = 0. if permission_changed else self.permission_progress
        if reset and permission_progress < 1.:
            permission_anchor, permission_progress = self.previous[:3].copy(), 0.
        permission_dt = 0. if reset or self.source_time is None else stamp-self.source_time
        permission_progress = min(1., permission_progress+permission_dt/POLICY['participation_thumb_transition_s'])
        if permission_progress < 1.:
            q = q.copy()
            q[:3] = permission_anchor+(q[:3]-permission_anchor)*permission_progress
            points, _ = self._native.landmarks(q) if self._native is not None else m.forward(q)
            solved_directions = self._directions(points[2:6])
        rebuilt = self.reconstruct_target(target, q, points)
        original_pair = target[TIPS[1:]]-target[5]
        contact_error = (points[TIPS[1:]]-points[5])-pair
        direction_error = np.degrees(np.arccos(np.clip(np.sum(solved_directions*desired, axis=1), -1., 1.)))
        errors = np.linalg.norm(contact_error, axis=1)*1000
        diagnostic = dict(solve_ms=(time.monotonic()-started)*1000,
            input_filter=shape_diagnostic,
            release_orientation_blend=release_blend, release_orientation_applied=release_applied,
            participation_transition_progress=permission_progress, participation_transition_changed=permission_changed,
            solver_method='bounded_vector_contact_v5', compute_backend=self.backend, nonlinear_dof=n, direction_weights=weights.tolist(),
            evaluations=evals, updates=updates, step_attempts=attempts,
            contact_active=contact.tolist(), projection_fraction=projection.tolist(),
            bend_plane_confidence=plane_confidence,
            coordination_fingers=moving.tolist(), contact_goal_fingers=active.tolist(), clock_reset=reset,
            participation=permission.diagnostic, participation_ms=participation_ms,
            contact_detection=contact_diagnostic, contact_detection_ms=contact_ms,
            human_gap_mm=(human_distance*1000).tolist(), projected_gap_mm=(np.linalg.norm(pair, axis=1)*1000).tolist(),
            model_gap_mm=(np.linalg.norm(points[TIPS[1:]]-points[5], axis=1)*1000).tolist(),
            projected_error_mm=errors.tolist(), contact_satisfied=bool(np.all(errors[contact]<=2.001)),
            constrained_contact=constrained_contact.tolist(),
            thumb_direction_error_deg=direction_error.tolist(),
            correction_deg=np.degrees(q-geometric_base).tolist(), geometric_seed_rad=geometric_base.tolist(),
            pinch_blend=float(max(gates)),
            nearest_pinch_error_mm=float(np.linalg.norm((points[TIPS[1:]]-points[5])-original_pair, axis=1)[int(np.argmin(distance))]*1000),
            pinch_error_mm=(np.linalg.norm((points[TIPS[1:]]-points[5])-original_pair, axis=1)*1000).tolist(),
            tip_error_mm=(np.linalg.norm(points[TIPS]-target[TIPS], axis=1)*1000).tolist(),
            structural_tip_error_mm=(np.linalg.norm(points[TIPS]-rebuilt[TIPS], axis=1)*1000).tolist(),
            limited_axes=[m.names[i] for i in range(10) if min(q[i]-m.lower[i],m.upper[i]-q[i])<1e-3])
        permission.diagnostic['actual_correction_change_deg'] = np.degrees(
            (q-geometric_base)-permission.state.correction).tolist()
        permission.diagnostic['actual_correction_change_deg'][:3] = [0., 0., 0.]
        check()
        self._serial += 1
        self._pending = (self._serial, q.copy(), contact.copy(), projection.copy(), stamp, shape, permission.solved(q, geometric_base), constrained_contact.copy(), permission_anchor.copy(), permission_progress, contact_state)
        result = dict(joints=q.tolist(), mapping_version=VERSION, mapping_fingerprint=m.digest,
                      diagnostics=diagnostic, target_points=target.tolist(), reconstructed_points=rebuilt.tolist(),
                      solved_points=points.tolist(), candidate_id=self._serial)
        if commit:
            self.commit(self._serial)
        return result
