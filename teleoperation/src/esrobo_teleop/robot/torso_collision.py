"""Conservative arm/torso swept-volume checks using URDF collision geometry.

Arm mesh boxes and torso convex hulls enclose meshes; this rejects some paths
that exact mesh distance would accept. No guessed torso dimensions or network
access at runtime. All non-arm joints are assumed stationary at URDF zero.
"""
import threading
import time
import struct
import itertools
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np


class TorsoCollisionGuard:
    def __init__(self, solver, side, package_dirs=(), margin=0.03,
                 shoulder_margin=0.005, *, include_fingers=True):
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
        self.hand_motion_horizon_s = 0.5
        self._hand_interval_key = None
        self._hand_diagnostics = {}
        self.margin = float(margin)
        if not np.isfinite(shoulder_margin) or not 0 < shoulder_margin <= margin:
            raise ValueError("invalid shoulder attachment clearance")
        self.shoulder_margin = float(shoulder_margin)
        self.last_diagnostics = {}
        roots = [str(Path(p).expanduser().resolve()) for p in package_dirs]
        roots.append(str(Path(__file__).resolve().parents[4] / "model" / "urdf"))
        roots.append(str(Path(__file__).resolve().parents[3] / "assets"))
        # Build enclosing boxes directly from STL vertices. Avoid building
        # several million-triangle BVHs just to read their local bounds.
        root = ET.parse(solver._cfg.urdf_path).getroot()
        torso_hulls = {}
        collision_links = {id(c): link.attrib["name"] for link in root.findall("link") for c in link.findall("collision")}
        for collision in root.findall(".//collision"):
            mesh = collision.find("geometry/mesh")
            if mesh is None:
                continue
            filename = mesh.attrib["filename"]
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
            link_name = collision_links[id(collision)]
            if not link_name.startswith(("left", "right")):
                from scipy.spatial import ConvexHull
                points = vertices.astype(float) * scale - center
                hull = ConvexHull(points)
                cloud = coal.StdVec_Vec3s()
                for point in points[hull.vertices]:
                    cloud.append(point)
                torso_hulls[link_name] = coal.Convex.convexHull(cloud, True, "Qt")
            origin = collision.find("origin")
            if origin is None:
                origin = ET.SubElement(collision, "origin")
            xyz = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")
            rpy = np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")
            origin.set("xyz", " ".join(map(str, xyz + pin.rpy.rpyToMatrix(rpy) @ center)))
            geometry = collision.find("geometry")
            geometry.remove(mesh)
            ET.SubElement(geometry, "box", size=" ".join(map(str, extent)))
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
        # Finger geometries are replaced below by a pose-independent envelope.
        hand_id = self.model.getFrameId(getattr(solver._cfg, f"{side}_hand_frame"))
        hand_joint = self.model.frames[hand_id].parentJoint
        fingers = []
        finger_specs = []
        joint_axes = {j.attrib["name"]: np.fromstring(j.find("axis").get("xyz"), sep=" ")
                      for j in root.findall("joint") if j.find("axis") is not None}

        def corners(lower, upper):
            return np.array(list(itertools.product(*zip(lower, upper))))

        def sweep_axis(points, axis, lower, upper):
            axis = axis / np.linalg.norm(axis)
            base = np.outer(points @ axis, axis)
            cosine = points - base
            sine = np.cross(axis, points)
            values = [base + cosine * np.cos(t) + sine * np.sin(t) for t in (lower, upper)]
            # Extrema of each sinusoidal coordinate; evaluating extra points
            # is conservative and avoids angular sampling assumptions.
            for phase in np.arctan2(sine, cosine).flat:
                for k in range(-3, 4):
                    t = phase + k * np.pi
                    if lower <= t <= upper:
                        values.append(base + cosine * np.cos(t) + sine * np.sin(t))
            values = np.concatenate(values)
            return corners(values.min(axis=0), values.max(axis=0))
        for i, obj in enumerate(self.geometry.geometryObjects):
            frame_name = self.model.frames[obj.parentFrame].name
            obj.geometry.computeLocalAABB()
            box = obj.geometry.aabb_local
            center = (np.asarray(box.min_) + np.asarray(box.max_)) / 2
            extent = np.asarray(box.max_) - np.asarray(box.min_)
            if not np.all(np.isfinite(extent)) or np.any(extent <= 0):
                raise ValueError(f"invalid collision geometry: {frame_name}")
            shape = torso_hulls.get(frame_name, coal.Box(*extent))
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
                # Enclose every allowed finger posture with interval boxes.
                # Sweep box corners analytically over each URDF joint range,
                # working from the mesh back to the fixed wrist parent.
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
                        qi = self.model.idx_qs[joint]
                        points = sweep_axis(points, joint_axes[name],
                            self.model.lowerPositionLimit[qi], self.model.upperPositionLimit[qi])
                        placement = self.model.jointPlacements[joint]
                        points = points @ placement.rotation.T + placement.translation
                        joint = self.model.parents[joint]
                    if joint == hand_joint and visited:
                        finger_specs.append((initial_points, initial_joint, frame_name))
                        lo, hi = points.min(axis=0), points.max(axis=0)
                        fingers.append((coal.Box(*(hi-lo)), (hi+lo)/2, frame_name,
                                        motion_weights(hand_joint, (hi+lo)/2, np.linalg.norm(hi-lo)/2)))
                        moving.remove(i)
        if not moving or not torso or (self.include_fingers and not fingers):
            raise ValueError("URDF lacks complete active arm, hand or torso collision geometry")
        self.pairs = [(a, b) for a in moving for b in torso]
        self.pair_margins = {(a, b): (
            self.shoulder_margin if self.model.frames[self.geometry.geometryObjects[a].parentFrame].name == side + "_nero_link2"
            and self.model.frames[self.geometry.geometryObjects[b].parentFrame].name == "waist_link3"
            else self.margin) for a, b in self.pairs}
        self.torso = torso
        self.hand_joint = hand_joint
        self.fingers = fingers if self.include_fingers else []
        if self.fingers:
            self.reach += max(np.linalg.norm(center) for _, center, _, _ in self.fingers)
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
                points, joint = initial_points.copy(), initial_joint
                while joint != hand_joint:
                    joint_name = self.model.names[joint]
                    lo, hi = limits(joint_name, self.model.idx_qs[joint])
                    points = sweep_axis(points, joint_axes[joint_name], lo, hi)
                    placement = self.model.jointPlacements[joint]
                    points = points @ placement.rotation.T + placement.translation
                    joint = self.model.parents[joint]
                lo, hi = points.min(axis=0), points.max(axis=0)
                result.append((coal.Box(*(hi-lo)), (hi+lo)/2, name,
                    motion_weights(hand_joint, (hi+lo)/2, np.linalg.norm(hi-lo)/2)))
            return result
        self._build_fingers = build_fingers

    def _refresh_hand(self):
        if not getattr(self, "include_fingers", True):
            self._hand_diagnostics = {"collision_profile": "arm_wrist_palm"}
            return True
        if getattr(self, "hand_state_provider", None) is None:
            return True  # Legacy/offline use retains all-pose envelopes.
        state = self.hand_state_provider(self.hand_motion_horizon_s)
        if not state.valid:
            self.last_diagnostics = dict(reason=state.reason, hand_stamp=state.stamp)
            return False
        self._hand_diagnostics = dict(hand_feedback_age_s=time.monotonic()-state.stamp,
            calibrated_hand_joints=len(state.calibrated))
        # Round OUTWARDS to cache conservative envelopes across control ticks.
        intervals = {name: (float(np.floor(lo/.02)*.02), float(np.ceil(hi/.02)*.02))
                     for name, (lo, hi) in state.intervals.items()}
        key = tuple(sorted(intervals.items()))
        try:
            if key != self._hand_interval_key:
                self.fingers = self._build_fingers(intervals)
                self._hand_interval_key = key
        except ValueError as exc:
            self.last_diagnostics = dict(reason=str(exc))
            return False
        return True

    def _clearance(self, full, halfwidth=None):
        self._box_certified = True
        halfwidth = np.zeros(14) if halfwidth is None else halfwidth
        q = np.zeros(self.model.nq)
        q[self.indices] = full
        self.pin.forwardKinematics(self.model, self.data, q)
        self.pin.updateGeometryPlacements(self.model, self.data, self.geometry, self.geometry_data)
        transforms = []
        for placement, (_, center) in zip(self.geometry_data.oMg, self.shapes):
            transforms.append(self.coal.Transform3s(placement.rotation,
                                                   placement.translation + placement.rotation @ center))
        minimum, pair, required, actual = float("inf"), None, self.margin, float("inf")
        for a, b in self.pairs:
            result = self.coal.DistanceResult()
            d = self.coal.distance(self.shapes[a][0], transforms[a], self.shapes[b][0],
                                   transforms[b], self.distance_request, result)
            margin = self.pair_margins[a, b]
            if d - margin <= self.motion_weights[a] @ halfwidth:
                self._box_certified = False
            if d - margin < minimum:
                minimum, pair = d - margin, [self.geometry.geometryObjects[k].name for k in (a, b)]
                required, actual = margin, d
        for shape, center, name, weights in self.fingers:
            hand = self.data.oMi[self.hand_joint]
            transform = self.coal.Transform3s(hand.rotation, hand.translation + hand.rotation @ center)
            for b in self.torso:
                result = self.coal.DistanceResult()
                d = self.coal.distance(shape, transform, self.shapes[b][0], transforms[b],
                                       self.distance_request, result)
                if d - self.margin <= weights @ halfwidth:
                    self._box_certified = False
                if d - self.margin < minimum:
                    minimum, pair = d - self.margin, [name + "_finger_envelope", self.geometry.geometryObjects[b].name]
                    required, actual = self.margin, d
        self.last_diagnostics = dict(clearance_m=float(actual), required_m=required, pair=pair,
                                     **self._hand_diagnostics)
        return minimum + self.margin

    def __call__(self, start, end):
        start, end = np.asarray(start, float), np.asarray(end, float)
        if any(q.shape != (14,) or not np.all(np.isfinite(q)) for q in (start, end)):
            raise ValueError("invalid collision-check joints")
        # Certify the whole joint interval box: a MOVE J controller need not
        # interpolate all axes synchronously. Subdivide ONE joint at a time,
        # covering every combination, rather than sampling only a diagonal.
        with self.lock:
            if not self._refresh_hand():
                return False
            pending = [(np.minimum(start, end), np.maximum(start, end))]
            checks = 0
            worst = None
            while pending:
                a, b = pending.pop()
                middle = (a + b) / 2
                clearance = self._clearance(middle, (b-a)/2)
                checks += 1
                diag = self.last_diagnostics
                if 'clearance_m' in diag and (worst is None or
                        diag['clearance_m']-diag['required_m'] < worst['clearance_m']-worst['required_m']):
                    worst = dict(diag)
                if not np.isfinite(clearance) or clearance <= self.margin:
                    return False
                if self._box_certified:
                    continue
                if checks >= 64:
                    self.last_diagnostics["reason"] = "swept path could not be certified within budget"
                    return False
                joint = int(np.argmax(b - a))
                left_end, right_start = b.copy(), a.copy()
                left_end[joint] = right_start[joint] = middle[joint]
                pending.extend(((a, left_end), (right_start, b)))
            if worst is not None:
                self.last_diagnostics = worst
            return True
