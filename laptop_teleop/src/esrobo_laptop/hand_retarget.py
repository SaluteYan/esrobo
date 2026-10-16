"""Left L20Lite geometry fitting. Pure computation; no SDK/ROS/CAN access."""
import hashlib
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from esrobo_teleop.robot.linker_hand_driver import ACTIVE_HAND_JOINTS

VERSION = "left-urdf-10-v1"
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
    value = dict(version=VERSION, hand=asdict(cfg.hand), manual=MANUAL_DEG,
                 urdf=hashlib.sha256(model.read_bytes()).hexdigest())
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
        points, _ = self.forward(self.neutral)
        from .pico_hand import palm_basis
        poses = np.zeros((26, 7)); poses[:, :3] = points; poses[:, 6] = 1
        self.basis = palm_basis(poses)
        self.origin = points[[7, 12, 17, 22]].mean(axis=0)
        self.width = np.linalg.norm(points[7]-points[22])

    def forward(self, q):
        """26 landmarks and their exact derivatives, including mimic chains."""
        n = len(self.joints)
        positions = np.zeros((n, 3)); rotations = np.zeros((n, 3, 3))
        angular = np.zeros((n, 3, 10)); jac = np.zeros((n, 3, 10))
        for i, (parent, offset, rot, axis, k, ratio, bias) in enumerate(self.joints):
            R = np.eye(3) if parent < 0 else rotations[parent]
            p = np.zeros(3) if parent < 0 else positions[parent]
            w = np.zeros((3, 10)) if parent < 0 else angular[parent]
            dp = np.zeros((3, 10)) if parent < 0 else jac[parent]
            d = R @ offset
            positions[i] = p+d
            jac[i] = dp+np.cross(w.T, d).T
            base = R @ rot
            rotations[i] = base @ Rotation.from_rotvec(axis*(q[k]*ratio+bias)).as_matrix()
            angular[i] = w
            angular[i, :, k] += base @ axis * ratio
        points = np.zeros((26, 3)); derivatives = np.zeros((26, 3, 10))
        def landmark(slot, name, offset=None):
            i = self.joint_index[name]
            d = np.zeros(3) if offset is None else rotations[i] @ offset
            points[slot] = positions[i]+d
            derivatives[slot] = jac[i]+np.cross(angular[i].T, d).T
        for f, finger in enumerate(FINGERS):
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
    def __init__(self, cfg):
        self.model = HandModel(cfg)
        self.previous = self.model.neutral.copy()

    def residual(self, q, target, previous, *, jacobian=False):
        m = self.model
        points, jac = m.forward(q)
        tips, jt = points[TIPS], jac[TIPS]
        desired = target[TIPS]
        pair = ((tips[1:]-tips[0])-(desired[1:]-desired[0]))/m.width
        jpair = (jt[1:]-jt[0])/m.width
        position = (tips-desired)/m.width
        jposition = jt/m.width
        vectors = points[BONES[:, 1]]-points[BONES[:, 0]]
        lengths = np.linalg.norm(vectors, axis=1)
        directions = vectors/lengths[:, None]
        dv = jac[BONES[:, 1]]-jac[BONES[:, 0]]
        jd = (dv-directions[:, :, None]*np.einsum('ni,nik->nk', directions, dv)[:, None, :])/lengths[:, None, None]
        human = target[BONES[:, 1]]-target[BONES[:, 0]]
        human_lengths = np.linalg.norm(human, axis=1)
        if np.any(human_lengths < 1e-5):
            raise ValueError("degenerate PICO finger bone")
        direction = directions-human/human_lengths[:, None]
        span = m.upper-m.lower
        weights = (4/12, 1/15, .25/45, .01/10)
        values = np.concatenate([a.ravel()*np.sqrt(w) for a, w in zip((pair, position, direction, (q-previous)/span), weights)])
        matrix = np.concatenate([a.reshape(-1, 10)*np.sqrt(w) for a, w in zip((jpair, jposition, jd, np.diag(1/span)), weights)])
        return (values, matrix) if jacobian is None else matrix if jacobian else values

    def solve(self, poses, *, budget_s=.020, target=None):
        started = time.monotonic()
        m = self.model
        target = m.human_target(poses) if target is None else np.asarray(target, dtype=float)
        previous = self.previous.copy()
        deadline = started+budget_s
        cache_q = cache = None
        class Deadline(Exception):
            pass
        def evaluate(q):
            nonlocal cache_q, cache
            if time.monotonic() >= deadline:
                raise Deadline()
            if cache_q is None or not np.array_equal(cache_q, q):
                cache_q, cache = q.copy(), self.residual(q, target, previous, jacobian=None)
            return cache
        try:
            seed = np.clip(previous, m.lower+1e-5, m.upper-1e-5)
            solution = least_squares(lambda q: evaluate(q)[0], seed, jac=lambda q: evaluate(q)[1],
                                     bounds=(m.lower, m.upper), max_nfev=10, ftol=1e-5, xtol=1e-5, gtol=1e-5)
            q = solution.x
        except Deadline:
            raise TimeoutError("left hand solver exceeded 20 ms") from None
        elapsed = time.monotonic()-started
        if elapsed > budget_s:
            raise TimeoutError("left hand solver exceeded 20 ms")
        self.previous = q.copy()
        points, _ = m.forward(q)
        error = points[TIPS]-target[TIPS]
        pair = (points[TIPS[1:]]-points[TIPS[0]])-(target[TIPS[1:]]-target[TIPS[0]])
        return dict(joints=q.tolist(), mapping_version=VERSION, mapping_fingerprint=m.digest,
                    diagnostics=dict(solve_ms=elapsed*1000, tip_error_mm=(np.linalg.norm(error, axis=1)*1000).tolist(),
                        pinch_error_mm=(np.linalg.norm(pair, axis=1)*1000).tolist(),
                        limited_axes=[m.names[i] for i in range(10) if min(q[i]-m.lower[i], m.upper[i]-q[i]) < 1e-3]),
                    target_points=target.tolist(), solved_points=points.tolist())
