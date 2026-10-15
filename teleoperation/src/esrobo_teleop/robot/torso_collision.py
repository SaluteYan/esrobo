"""Conservative arm/torso swept-volume checks using URDF collision geometry.

Original mesh convex hulls enclose each link; continuous angular sweeps enclose
uncertain finger motion. Convexification can still reject concave free space. No guessed
torso dimensions or network access at runtime. All non-arm joints are assumed
stationary at URDF zero.
"""
import threading
import hashlib
import json
import time
import struct
import itertools
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np


def _convex_vertices(points):
    """Retain the original convex hull, without a box or support-plane expansion."""
    from scipy.spatial import ConvexHull
    points = np.asarray(points, dtype=float)
    return points[ConvexHull(points).vertices]


def _sweep_convex_vertices(points, axis, lower, upper):
    """Enclose every rotation, including between angular subdivision endpoints.

    Each vertex traces a circular arc. For each arc of at most 15 degrees,
    its two endpoints and the intersection of their tangents enclose the
    *whole* arc. Convexity then covers every interior mesh point as well.
    Unlike a sampled hull, this does not miss an intermediate orientation;
    unlike repeated axis-aligned boxes, it retains the finger's shape.
    """
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    points = np.asarray(points, dtype=float)
    # Tiny 3-D products must not start large BLAS thread pools while planning.
    base = np.outer(np.einsum('ij,j->i', points, axis, optimize=False), axis)
    cosine = points - base
    sine = np.cross(axis, points)
    count = max(1, int(np.ceil((upper-lower) / np.deg2rad(15))))
    angles = np.linspace(lower, upper, count+1)
    values = [base + cosine*np.cos(t) + sine*np.sin(t) for t in angles]
    for a, b in zip(angles[:-1], angles[1:]):
        middle, half = (a+b)/2, (b-a)/2
        values.append(base + (cosine*np.cos(middle) + sine*np.sin(middle))/np.cos(half))
    return _convex_vertices(np.concatenate(values))


class TorsoCollisionGuard:
    _entry_certificates = None
    _entry_certificate_context = None
    _entry_certificate_provider = None
    _entry_coverage_only = False
    _entry_arm_geometry_only = False

    def __init__(self, solver, side, package_dirs=(), margin=0.005,
                 shoulder_margin=0.005, *, include_fingers=True, mesh_snapshot=None):
        import pinocchio as pin
        try:
            import coal
        except ImportError:
            import hppfcl as coal
        if side not in ("left", "right") or not np.isfinite(margin) or margin <= 0:
            raise ValueError("invalid torso collision configuration")
        self.pin, self.coal = pin, coal
        self.model = solver._model
        self.indices = solver._arm_q_indices
        self.data = self.model.createData()
        self.lock = threading.Lock()
        self.hand_state_provider = None
        self.include_fingers = bool(include_fingers)
        self.thumb_envelope = "original_mesh_convex_arc_hull"
        self.hand_motion_horizon_s = 0.5
        self._hand_interval_key = None
        self._hand_diagnostics = {}
        self.margin = float(margin)
        if not np.isfinite(shoulder_margin) or not 0 < shoulder_margin <= margin:
            raise ValueError("invalid shoulder attachment clearance")
        self.shoulder_margin = float(shoulder_margin)
        self.last_diagnostics = {}
        self._entry_certificates = None
        self._entry_certificate_context = None
        roots = [str(Path(p).expanduser().resolve()) for p in package_dirs]
        roots.append(str(Path(__file__).resolve().parents[4] / "model" / "urdf"))
        roots.append(str(Path(__file__).resolve().parents[3] / "assets"))
        # Build original STL convex hulls for every link. Temporary URDF boxes
        # carry placements into Pinocchio; collision queries use the hulls.
        urdf_bytes = Path(solver._cfg.urdf_path).read_bytes()
        self.proof_urdf_digest = hashlib.sha256(urdf_bytes).hexdigest()
        root = ET.fromstring(urdf_bytes)
        collision_hulls = {}
        finger_mesh_vertices = {}
        self.proof_mesh_snapshot = {}
        collision_links = {id(c): link.attrib["name"] for link in root.findall("link") for c in link.findall("collision")}
        for collision in root.findall(".//collision"):
            mesh = collision.find("geometry/mesh")
            if mesh is None:
                continue
            filename = mesh.attrib["filename"]
            mesh_key = filename + '|' + mesh.get('scale', '1 1 1')
            if mesh_snapshot is not None:
                cached = mesh_snapshot[mesh_key]
                center, extent, points = (np.asarray(cached[k], dtype=float)
                                          for k in ('center', 'extent', 'points'))
                if (center.shape != (3,) or extent.shape != (3,) or points.ndim != 2
                        or points.shape[1] != 3 or len(points) < 4
                        or not all(np.all(np.isfinite(v)) for v in (center, extent, points))
                        or np.any(extent <= 0)):
                    raise ValueError('invalid immutable collision mesh snapshot')
            else:
                if filename.startswith("package://"):
                    candidates = [Path(p) / filename.removeprefix("package://") for p in roots]
                else:
                    candidates = [Path(solver._cfg.urdf_path).parent / filename]
                path = next((p for p in candidates if p.is_file()), None)
                if path is None:
                    raise ValueError(f"missing collision mesh: {filename}; run scripts/fetch_collision_meshes.py")
                raw = path.read_bytes()
                count = struct.unpack_from("<I", raw, 80)[0] if len(raw) >= 84 else 0
                if len(raw) != 84 + count * 50 or not count:
                    raise ValueError(f"expected binary STL collision mesh: {path}")
                dtype = np.dtype([("normal", "<f4", 3), ("vertices", "<f4", (3, 3)), ("attr", "<u2")])
                vertices = np.frombuffer(raw, dtype=dtype, count=count, offset=84)["vertices"].reshape(-1, 3)
                scale = np.fromstring(mesh.get("scale", "1 1 1"), sep=" ")
                bounds = np.array([vertices.min(axis=0), vertices.max(axis=0)]) * scale
                lower, upper = bounds.min(axis=0), bounds.max(axis=0)
                extent, center = upper - lower, (upper + lower) / 2
                if not np.all(np.isfinite(extent)) or np.any(extent <= 0):
                    raise ValueError(f"invalid mesh bounds: {path}")
                points = _convex_vertices(vertices.astype(float) * scale - center)
            self.proof_mesh_snapshot[mesh_key] = dict(center=center.tolist(), extent=extent.tolist(),
                                                     points=points.tolist())
            link_name = collision_links[id(collision)]
            cloud = coal.StdVec_Vec3s()
            for point in points:
                cloud.append(point)
            collision_hulls[link_name] = coal.Convex.convexHull(cloud, True, "Qt")
            if link_name.startswith(tuple(f"{side}_{finger}_" for finger in
                    ("thumb", "index", "middle", "ring", "pinky"))):
                finger_mesh_vertices[link_name] = points
            origin = collision.find("origin")
            if origin is None:
                origin = ET.SubElement(collision, "origin")
            xyz = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")
            rpy = np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")
            origin.set("xyz", " ".join(map(str, xyz + pin.rpy.rpyToMatrix(rpy) @ center)))
            geometry = collision.find("geometry")
            geometry.remove(mesh)
            ET.SubElement(geometry, "box", size=" ".join(map(str, extent)))
        self.proof_mesh_digest = hashlib.sha256(json.dumps(
            self.proof_mesh_snapshot, sort_keys=True, allow_nan=False).encode()).hexdigest()
        self.geometry = pin.buildGeomFromUrdfString(
            self.model, ET.tostring(root).decode(), pin.GeometryType.COLLISION)
        self.geometry_data = pin.GeometryData(self.geometry)
        self.shapes = []
        self.motion_weights = []
        arm_index = {int(q): i for i, q in enumerate(self.indices)}
        def motion_weights(joint, translation, radius):
            weights = np.zeros(14)
            reach = np.linalg.norm(translation) + radius
            while joint:
                qi = int(self.model.idx_qs[joint])
                if qi in arm_index:
                    weights[arm_index[qi]] = reach
                reach += np.linalg.norm(self.model.jointPlacements[joint].translation)
                joint = self.model.parents[joint]
            return weights
        moving, torso = [], []
        self.reach = 0.0
        # Retain all collision links, including the force sensor and hand.
        # Finger geometries are swept below over measured or full joint ranges.
        hand_id = self.model.getFrameId(getattr(solver._cfg, f"{side}_hand_frame"))
        hand_joint = self.model.frames[hand_id].parentJoint
        finger_specs = []
        joint_axes = {j.attrib["name"]: np.fromstring(j.find("axis").get("xyz"), sep=" ")
                      for j in root.findall("joint") if j.find("axis") is not None}

        def corners(lower, upper):
            return np.array(list(itertools.product(*zip(lower, upper))))

        for i, obj in enumerate(self.geometry.geometryObjects):
            frame_name = self.model.frames[obj.parentFrame].name
            obj.geometry.computeLocalAABB()
            box = obj.geometry.aabb_local
            center = (np.asarray(box.min_) + np.asarray(box.max_)) / 2
            extent = np.asarray(box.max_) - np.asarray(box.min_)
            if not np.all(np.isfinite(extent)) or np.any(extent <= 0):
                raise ValueError(f"invalid collision geometry: {frame_name}")
            shape = collision_hulls.get(frame_name, coal.Box(*extent))
            self.shapes.append((shape, center))
            self.motion_weights.append(motion_weights(obj.parentJoint, obj.placement.translation,
                                                       np.linalg.norm(center) + np.linalg.norm(extent)/2))
            is_arm = frame_name.startswith(("left", "right"))
            if not is_arm:
                torso.append(i)
            if frame_name.startswith(side) and frame_name not in (
                    side + "Arm_link", side + "_nero_base_link", side + "_nero_link1"):
                moving.append(i)
                radius = np.linalg.norm(center) + np.linalg.norm(extent) / 2
                reach = np.linalg.norm(obj.placement.translation) + radius
                joint = obj.parentJoint
                while joint:
                    reach += np.linalg.norm(self.model.jointPlacements[joint].translation)
                    joint = self.model.parents[joint]
                self.reach = max(self.reach, reach)
                # Identify the finger chain without replacing mesh geometry
                # by a box. Primitive box links retain their exact corners.
                joint = obj.parentJoint
                if joint != hand_joint:
                    points = corners(center - extent / 2, center + extent / 2)
                    points = points @ obj.placement.rotation.T + obj.placement.translation
                    initial_points, initial_joint = points.copy(), joint
                    visited = []
                    while joint and joint != hand_joint:
                        visited.append(joint)
                        name = self.model.names[joint]
                        if name not in joint_axes or self.model.joints[joint].nq != 1:
                            raise ValueError(f"unsupported finger collision joint: {name}")
                        joint = self.model.parents[joint]
                    if joint == hand_joint and visited:
                        mesh_points = finger_mesh_vertices.get(frame_name)
                        if mesh_points is not None:
                            mesh_points = mesh_points @ obj.placement.rotation.T + obj.placement.translation
                        finger_specs.append((mesh_points if mesh_points is not None else initial_points,
                                             initial_joint, frame_name))
                        moving.remove(i)
        if not moving or not torso or (self.include_fingers and not finger_specs):
            raise ValueError("URDF lacks complete active arm, hand or torso collision geometry")
        self.pairs = [(a, b) for a in moving for b in torso]
        # Initial approach excludes the entire dexterous hand, including its
        # palm mount. Normal checks and checked return retain all these pairs.
        self._entry_arm_pairs = tuple((a, b) for a, b in self.pairs
            if not self.model.frames[self.geometry.geometryObjects[a].parentFrame].name.startswith(side + "Hand"))
        if not self._entry_arm_pairs:
            raise ValueError("URDF lacks initial approach arm collision geometry")
        self.pair_margins = {(a, b): (
            self.shoulder_margin if self.model.frames[self.geometry.geometryObjects[a].parentFrame].name == side + "_nero_link2"
            and self.model.frames[self.geometry.geometryObjects[b].parentFrame].name == "waist_link3"
            else self.margin) for a, b in self.pairs}
        self.torso = torso
        self.hand_joint = hand_joint
        self.fingers = []
        self.distance_request = coal.DistanceRequest()
        mimics = {j.attrib["name"]: (j.find("mimic").get("joint"),
                    float(j.find("mimic").get("multiplier", "1")),
                    float(j.find("mimic").get("offset", "0")))
                  for j in root.findall("joint") if j.find("mimic") is not None}
        def build_fingers(intervals):
            def limits(name, qi):
                lo, hi = self.model.lowerPositionLimit[qi], self.model.upperPositionLimit[qi]
                bound = intervals.get(name)
                if bound is None and name in mimics:
                    source, multiplier, offset = mimics[name]
                    if source in intervals:
                        bound = sorted(multiplier*x+offset for x in intervals[source])
                if bound is not None:
                    lo, hi = max(lo, bound[0]), min(hi, bound[1])
                    if lo > hi:
                        raise ValueError("hand feedback outside URDF limits: " + name)
                return lo, hi
            result = []
            for initial_points, initial_joint, name in finger_specs:
                # The selected side's fingers use enclosing mesh hulls and
                # conservative arc hulls. Missing feedback still uses the full
                # URDF interval; no joint or calibration uncertainty is omitted.
                points, joint = initial_points.copy(), initial_joint
                while joint != hand_joint:
                    joint_name = self.model.names[joint]
                    lo, hi = limits(joint_name, self.model.idx_qs[joint])
                    points = _sweep_convex_vertices(points, joint_axes[joint_name], lo, hi)
                    placement = self.model.jointPlacements[joint]
                    points = (np.einsum('ij,kj->ik', points, placement.rotation, optimize=False)
                              + placement.translation)
                    joint = self.model.parents[joint]
                lo, hi = points.min(axis=0), points.max(axis=0)
                cloud = coal.StdVec_Vec3s()
                for point in points - (hi+lo)/2:
                    cloud.append(point)
                shape = coal.Convex.convexHull(cloud, True, "Qt")
                result.append((shape, (hi+lo)/2, name,
                    motion_weights(hand_joint, (hi+lo)/2, np.linalg.norm(hi-lo)/2)))
            return result
        self._build_fingers = build_fingers
        if self.include_fingers:
            self.fingers = build_fingers({})
            self.reach += max(np.linalg.norm(center) for _, center, _, _ in self.fingers)

    def _refresh_hand(self, *, include_fingers=True, state=None):
        if not include_fingers or not getattr(self, "include_fingers", True):
            self._hand_diagnostics = {"collision_profile": (
                "entry_arm" if self._entry_arm_geometry_only else "arm_wrist_palm")}
            return True
        if state is None and getattr(self, "hand_state_provider", None) is None:
            # Do not reuse the last measured pose after losing the provider.
            if getattr(self, '_hand_interval_key', None) is not None:
                self.fingers = self._build_fingers({})
                self._hand_interval_key = None
            self._hand_diagnostics = {"thumb_envelope": getattr(
                self, 'thumb_envelope', 'original_mesh_convex_arc_hull')}
            return True  # Legacy/offline use retains all-pose convex sweeps.
        if state is None:
            state = self.hand_state_provider(self.hand_motion_horizon_s)
        if not state.valid:
            self.last_diagnostics = dict(reason=state.reason, hand_stamp=state.stamp)
            return False
        self._hand_diagnostics = dict(hand_feedback_age_s=time.monotonic()-state.stamp,
            calibrated_hand_joints=len(state.calibrated),
            thumb_envelope=self.thumb_envelope)
        # Round OUTWARDS to cache conservative envelopes across control ticks.
        intervals = dict(state.intervals)
        intervals = {name: (float(np.floor(lo/.02)*.02), float(np.ceil(hi/.02)*.02))
                     for name, (lo, hi) in intervals.items()}
        # During one planning session a fresh packet often makes the age
        # uncertainty smaller, then the next poll makes it larger again.
        # Retain a conservative envelope instead of rebuilding every finger
        # mesh and invalidating every path proof on each such oscillation.
        # Expansion changes the context below and requires NEW full proofs;
        # no old certificate is widened or combined. Outside planning, or on
        # a new session/provider/model, use the current measured envelope.
        if (self._entry_certificates is not None
                and self._entry_certificate_context is not None
                and self._entry_certificate_context == self._certificate_context(True)
                and self._entry_certificate_provider is getattr(self, 'hand_state_provider', None)):
            previous = dict(self._hand_interval_key or ())
            if previous.keys() == intervals.keys():
                intervals = {name: (min(lo, previous[name][0]), max(hi, previous[name][1]))
                             for name, (lo, hi) in intervals.items()}
        self._hand_diagnostics['thumb_joint_intervals_rad'] = {
            name: bound for name, bound in intervals.items() if '_thumb_' in name}
        key = tuple(sorted(intervals.items()))
        try:
            if key != self._hand_interval_key:
                self.fingers = self._build_fingers(intervals)
                self._hand_interval_key = key
        except ValueError as exc:
            self.last_diagnostics = dict(reason=str(exc))
            return False
        return True

    def _clearance(self, full, halfwidth=None, *, include_fingers=True,
                   active_pairs=None):
        self._box_certified = True
        halfwidth = np.zeros(14) if halfwidth is None else halfwidth
        q = np.zeros(self.model.nq)
        q[self.indices] = full
        self.pin.forwardKinematics(self.model, self.data, q)
        self.pin.updateGeometryPlacements(self.model, self.data, self.geometry, self.geometry_data)
        # Only construct transforms used by the remaining collision pairs.
        # FK and geometry placements still use this node's actual midpoint.
        transforms = {}
        def transform(index):
            if index not in transforms:
                placement = self.geometry_data.oMg[index]
                center = self.shapes[index][1]
                transforms[index] = self.coal.Transform3s(placement.rotation,
                    placement.translation + placement.rotation @ center)
            return transforms[index]
        unresolved_arm, unresolved_fingers = [], []
        arm_pairs = self.pairs if active_pairs is None else active_pairs[0]
        if self._entry_arm_geometry_only:
            retained = getattr(self, '_entry_arm_pairs', self.pairs)
            arm_pairs = tuple(pair for pair in arm_pairs if pair in retained)
        minimum, pair, required, actual = float("inf"), None, self.margin, float("inf")
        limiting_slack, split_weights = float("inf"), None
        for a, b in arm_pairs:
            result = self.coal.DistanceResult()
            d = self.coal.distance(self.shapes[a][0], transform(a), self.shapes[b][0],
                                   transform(b), self.distance_request, result)
            margin = self.pair_margins[a, b]
            slack = d - margin - self.motion_weights[a] @ halfwidth
            if not np.isfinite(d) or not np.isfinite(slack):
                self._box_certified = False
                self.last_diagnostics = dict(reason='nonfinite arm collision distance/bound',
                                             **self._hand_diagnostics)
                return float('nan')
            if slack <= 0:
                self._box_certified = False
                unresolved_arm.append((a, b))
            if slack < limiting_slack:
                limiting_slack, split_weights = slack, self.motion_weights[a]
            if d - margin < minimum:
                minimum, pair = d - margin, [self.geometry.geometryObjects[k].name for k in (a, b)]
                required, actual = margin, d
        finger_pairs = ([(i, b) for i in range(len(self.fingers)) for b in self.torso]
                        if active_pairs is None and include_fingers else
                        active_pairs[1] if active_pairs is not None and include_fingers else ())
        finger_transforms = {}
        for i, b in finger_pairs:
            shape, center, name, weights = self.fingers[i]
            hand = self.data.oMi[self.hand_joint]
            if i not in finger_transforms:
                finger_transforms[i] = self.coal.Transform3s(
                    hand.rotation, hand.translation + hand.rotation @ center)
            result = self.coal.DistanceResult()
            d = self.coal.distance(shape, finger_transforms[i], self.shapes[b][0], transform(b),
                                   self.distance_request, result)
            slack = d - self.margin - weights @ halfwidth
            if not np.isfinite(d) or not np.isfinite(slack):
                self._box_certified = False
                self.last_diagnostics = dict(reason='nonfinite finger collision distance/bound',
                                             **self._hand_diagnostics)
                return float('nan')
            if slack <= 0:
                self._box_certified = False
                unresolved_fingers.append((i, b))
            if slack < limiting_slack:
                limiting_slack, split_weights = slack, weights
            if d - self.margin < minimum:
                minimum, pair = d - self.margin, [name + "_finger_envelope", self.geometry.geometryObjects[b].name]
                required, actual = self.margin, d
        self.last_diagnostics = dict(clearance_m=float(actual), required_m=required, pair=pair,
                                     **self._hand_diagnostics)
        self._split_weights = split_weights
        self._uncertified_pairs = (tuple(unresolved_arm), tuple(unresolved_fingers))
        return minimum + self.margin

    def check_arm_only(self, start, end):
        """Certify arm, wrist and palm while the fingers receive no commands."""
        return self._check(start, end, include_fingers=False)

    def __call__(self, start, end):
        return self._check(start, end, include_fingers=True)

    def check_hand_return(self, start, end, hand_state):
        """Certify a bounded hand sweep with the stopped arm in a small box."""
        return self._check_once(start, end, include_fingers=True, hand_state=hand_state)

    def begin_entry_certificates(self, *, arm_geometry_only=False):
        with self.lock:
            self._entry_arm_geometry_only = bool(arm_geometry_only)
            self._entry_coverage_only = False
            self._entry_frozen_provider = None
            self._entry_certificates = []
            self._entry_certificate_context = None
            self._entry_certificate_provider = None

    def end_entry_certificates(self):
        with self.lock:
            self._entry_arm_geometry_only = False
            self._entry_coverage_only = False
            self._entry_frozen_provider = None
            self._entry_certificates = None
            self._entry_certificate_context = None
            self._entry_certificate_provider = None

    def invalidate_entry_certificates(self):
        with self.lock:
            if self._entry_certificates is not None:
                self._entry_certificates.clear()
            self._entry_certificate_context = None

    def freeze_entry_certificates(self, *, include_fingers=True):
        """Finish planning. Misses can no longer perform geometry until end/begin.

        Invalidating a frozen session leaves this gate CLOSED: a later fresh
        frame cannot silently rebuild certificates while the arm is moving.
        """
        with self.lock:
            self._entry_coverage_only = True
            self._entry_frozen_provider = getattr(self, 'hand_state_provider', None)
            return bool(self._entry_certificates and
                        self._entry_certificate_context == self._certificate_context(include_fingers))

    def check_entry_coverage(self, start, end, *, include_fingers=True):
        """Explicit execution API: even an ended session cannot do geometry."""
        start, end = np.asarray(start, float), np.asarray(end, float)
        if any(q.shape != (14,) or not np.all(np.isfinite(q)) for q in (start, end)):
            raise ValueError("invalid collision-check joints")
        with self.lock:
            if not self._entry_coverage_only:
                self.last_diagnostics = dict(certified=False, category='entry_certificate_invalid',
                    reason='no frozen planning certificates', entry_cache_hit=False, checked_boxes=0)
                return False
            return self._entry_coverage_locked(start, end, include_fingers)

    def check_cached_coverage(self, start, end, *, include_fingers=True):
        """Validate an existing proof at a stationary preload boundary.

        Unlike a normal planning check this cannot rebuild meshes or certify a
        cache miss. It does not freeze/unfreeze the surrounding return planner.
        """
        start, end = np.asarray(start, float), np.asarray(end, float)
        if any(q.shape != (14,) or not np.all(np.isfinite(q)) for q in (start, end)):
            raise ValueError("invalid collision-check joints")
        with self.lock:
            return self._entry_coverage_locked(start, end, include_fingers, frozen=False)

    def _entry_coverage_locked(self, start, end, include_fingers, *, frozen=True):
        """Scalar/interval checks only; never refresh meshes, run FK or distance."""
        lo, hi = np.minimum(start, end), np.maximum(start, end)
        def reject(category, reason, *, invalidate=False):
            if invalidate:
                self._entry_certificates = []
                self._entry_certificate_context = None
            self.last_diagnostics = dict(certified=False, category=category,
                reason=reason, entry_cache_hit=False, entry_coverage_only=True,
                checked_boxes=0, checked_lower_urdf_rad=lo.tolist(),
                checked_upper_urdf_rad=hi.tolist())
            return False
        if (not self._entry_certificates or
                self._entry_certificate_context != self._certificate_context(include_fingers)):
            return reject('entry_certificate_invalid', 'entry certificate context changed or invalidated',
                          invalidate=True)
        hand = {}
        if include_fingers and self.include_fingers:
            provider = getattr(self, 'hand_state_provider', None)
            expected_provider = (self._entry_frozen_provider if frozen
                                 else self._entry_certificate_provider)
            if provider is not expected_provider:
                return reject('entry_certificate_invalid', 'hand feedback provider changed', invalidate=True)
            if provider is not None:
                state = provider(self.hand_motion_horizon_s)
                if not state.valid:
                    return reject('hand_geometry_unavailable', state.reason, invalidate=True)
                bounds = dict(self._hand_interval_key or ())
                if (bounds.keys() != state.intervals.keys() or
                        any(not np.all(np.isfinite([a, b])) or a > b or
                            a < bounds[n][0] or b > bounds[n][1]
                            for n, (a, b) in state.intervals.items())):
                    return reject('entry_hand_envelope_changed',
                                  'fresh hand uncertainty exceeds the planned envelope', invalidate=True)
                hand = dict(hand_feedback_age_s=time.monotonic()-state.stamp,
                            calibrated_hand_joints=len(state.calibrated))
        for a, b, certificate in self._entry_certificates:
            if np.all(lo >= a) and np.all(hi <= b):
                self.last_diagnostics = dict(certificate, **hand)
                self.last_diagnostics.update(certified=True, category='certified_cache',
                    entry_cache_hit=True, entry_coverage_only=True, checked_boxes=0,
                    checked_lower_urdf_rad=lo.tolist(), checked_upper_urdf_rad=hi.tolist())
                return True
        # A candidate/optional lookahead miss must not discard valid stopping
        # certificates. The trajectory may select a smaller certified proposal.
        return reject('entry_certificate_miss', 'whole interval is outside every planned certificate')

    def _certificate_context(self, include_fingers):
        # Geometry/model are immutable during an execution. Identity plus the
        # concrete envelope, margins, shape offsets and motion bounds bind proofs.
        return (id(self.model), id(self.geometry), tuple(self.indices),
                include_fingers, self._entry_arm_geometry_only,
                (tuple(getattr(self, '_entry_arm_pairs', self.pairs))
                 if self._entry_arm_geometry_only else None),
                ((self.include_fingers, self.thumb_envelope, self._hand_interval_key,
                  self.hand_motion_horizon_s) if include_fingers else None),
                self.margin, self.shoulder_margin,
                tuple(sorted(self.pair_margins.items())), tuple(self.pairs),
                tuple((id(shape), np.asarray(center).tobytes()) for shape, center in self.shapes),
                tuple(np.asarray(w).tobytes() for w in self.motion_weights))

    def _check(self, start, end, *, include_fingers):
        return self._check_once(start, end, include_fingers=include_fingers)

    def _check_once(self, start, end, *, include_fingers, hand_state=None):
        start, end = np.asarray(start, float), np.asarray(end, float)
        if any(q.shape != (14,) or not np.all(np.isfinite(q)) for q in (start, end)):
            raise ValueError("invalid collision-check joints")
        def outcome(ok, category, checks):
            self.last_diagnostics.update(
                certified=bool(ok), category=category, checked_boxes=checks,
                collision_geometry="original_mesh_convex_hull",
                checked_lower_urdf_rad=np.minimum(start,end).tolist(),
                checked_upper_urdf_rad=np.maximum(start,end).tolist())
            return ok
        # Certify the whole joint interval box: a MOVE J controller need not
        # interpolate all axes synchronously. Subdivide ONE joint at a time,
        # covering every combination, rather than sampling only a diagonal.
        with self.lock:
            if hand_state is not None and (not self.include_fingers or self._entry_arm_geometry_only
                                          or self._entry_certificates is not None):
                return outcome(False, "hand_return_requires_full_geometry", 0)
            if self._entry_coverage_only:
                return self._entry_coverage_locked(start, end, include_fingers)
            self.last_diagnostics['entry_cache_hit'] = False
            refreshed = (self._refresh_hand(include_fingers=include_fingers) if hand_state is None
                         else self._refresh_hand(include_fingers=include_fingers, state=hand_state))
            if not refreshed:
                if self._entry_certificates is not None:
                    self._entry_certificates.clear()
                self._entry_certificate_context = None
                return outcome(False, "hand_geometry_unavailable", 0)
            root_lower, root_upper = np.minimum(start, end), np.maximum(start, end)
            cache = self._entry_certificates
            if cache is not None:
                context = self._certificate_context(include_fingers)
                provider = getattr(self, 'hand_state_provider', None)
                if (context != self._entry_certificate_context
                        or provider is not self._entry_certificate_provider):
                    cache.clear()
                    self._entry_certificate_context = context
                    self._entry_certificate_provider = provider
                for index, (lo, hi, certificate) in enumerate(cache):
                    if np.all(root_lower >= lo) and np.all(root_upper <= hi):
                        cache.append(cache.pop(index))
                        self.last_diagnostics = dict(certificate, **self._hand_diagnostics,
                                                     entry_cache_hit=True)
                        return outcome(True, "certified_cache", 0)
            pending = [(root_lower, root_upper, None)]
            checks = 0
            # Planning also proves seven-axis tracking/braking boxes. Their
            # independent drift dimensions can need more than 64 subdivisions
            # even with positive clearance. Keep a bounded larger proof budget
            # before motion; frozen execution never enters this calculation.
            max_checks = 256 if cache is not None else 64
            worst = None
            while pending:
                a, b, active_pairs = pending.pop()
                middle = (a + b) / 2
                clearance = self._clearance(middle, (b-a)/2,
                                            include_fingers=include_fingers,
                                            **({'active_pairs': active_pairs}
                                               if active_pairs is not None else {}))
                checks += 1
                if checks % 8 == 0:
                    # A dense offline certificate can occupy the Python thread
                    # for many state periods. Yield without changing any box,
                    # margin, or acceptance rule so the gateway can send its
                    # independent 50 Hz state stream during planning.
                    time.sleep(0)
                diag = self.last_diagnostics
                if 'clearance_m' in diag and (worst is None or
                        diag['clearance_m']-diag['required_m'] < worst['clearance_m']-worst['required_m']):
                    worst = dict(diag)
                if not np.isfinite(clearance) or clearance <= self.margin:
                    return outcome(False, "geometry_rejected", checks)
                if self._box_certified:
                    continue
                if checks >= max_checks:
                    self.last_diagnostics["reason"] = "swept path could not be certified within budget"
                    return outcome(False, "certification_budget_exhausted", checks)
                # Split the joint contributing most to the least-certifiable
                # pair's motion envelope. Subdivision still covers the exact
                # same interval box; this only chooses a more useful order.
                weights = getattr(self, "_split_weights", None)
                weighted_width = ((b-a) * weights if weights is not None
                                  and np.shape(weights) == (14,) else b-a)
                joint = int(np.argmax(weighted_width))
                if weighted_width[joint] <= 0:
                    joint = int(np.argmax(b-a))
                left_end, right_start = b.copy(), a.copy()
                left_end[joint] = right_start[joint] = middle[joint]
                # A pair with d(midpoint)-margin-motion_bound > 0 was
                # certified for this WHOLE parent box, hence for each child.
                # Recheck only unresolved pairs during entry or a hand-return
                # sweep. Proofs never escape this locked request or cross a
                # hand/model change; each root starts with all pairs.
                child_pairs = (getattr(self, '_uncertified_pairs', None)
                               if cache is not None or hand_state is not None else None)
                pending.extend(((a, left_end, child_pairs), (right_start, b, child_pairs)))
            if worst is not None:
                self.last_diagnostics = worst
            self.last_diagnostics['entry_cache_hit'] = False
            result = outcome(True, "certified", checks)
            if cache is not None:
                cache.append((root_lower.copy(), root_upper.copy(), dict(self.last_diagnostics)))
                del cache[:-32]
            return result
