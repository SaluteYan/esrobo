"""Offline finger geometry regressions; no robot SDK or motion commands."""
from pathlib import Path
import itertools
import struct
import time
import unittest
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import ConvexHull

from esrobo_teleop.config import load_config
from esrobo_teleop.ik.solver import IkSolver
from esrobo_teleop.robot.hand_geometry import HandGeometryState, geometry_state
from esrobo_teleop.robot.linker_hand_driver import L10_PHYSICAL_JOINT_NAMES
from esrobo_teleop.robot.torso_collision import (
    TorsoCollisionGuard, _sweep_convex_vertices, _convex_vertices,
)

ROOT = Path(__file__).resolve().parents[1]


class ConvexArcTests(unittest.TestCase):
    def test_compaction_keeps_original_vertices_without_outer_plane_expansion(self):
        points = np.array([[0., 0., 0.], [.04, 0., 0.], [0., .03, 0.],
                           [0., 0., .02], [.004, .003, .002]])
        compact = _convex_vertices(points)
        self.assertEqual(len(compact), 4)
        for vertex in compact:
            self.assertTrue(np.any(np.all(points == vertex, axis=1)))
        self.assertAlmostEqual(ConvexHull(compact).volume, ConvexHull(points).volume)

    def test_continuous_arc_is_inside_outer_hull_not_only_sampled_angles(self):
        rng = np.random.default_rng(42)
        points = rng.uniform(-.04, .04, (30, 3))
        for axis, lo, hi in (([0, 0, 1], -.37, .98),
                             ([1, 2, -3], -2.8, 2.7),
                             ([1, 0, 0], .2, .2)):
            axis = np.asarray(axis, float)
            axis /= np.linalg.norm(axis)
            vertices = _sweep_convex_vertices(points, axis, lo, hi)
            hull = ConvexHull(vertices)
            for angle in np.linspace(lo, hi, 313):
                base = np.outer(points @ axis, axis)
                actual = (base + (points-base)*np.cos(angle)
                          + np.cross(axis, points)*np.sin(angle))
                self.assertLessEqual(np.max(actual @ hull.equations[:, :3].T
                                            + hull.equations[:, 3]), 1e-12)

    def test_two_independent_rotation_intervals_are_both_enclosed(self):
        points = np.array([[x, y, z] for x in (-.03, .01)
                           for y in (-.008, .009) for z in (-.004, .006)])
        outer = _sweep_convex_vertices(points, [0, 0, 1], -.17, .4)
        outer = _sweep_convex_vertices(outer, [1, 0, 0], -.3, .2)
        hull = ConvexHull(outer)
        for a in np.linspace(-.17, .4, 23):
            rz = np.array([[np.cos(a), -np.sin(a), 0],
                           [np.sin(a), np.cos(a), 0], [0, 0, 1]])
            for b in np.linspace(-.3, .2, 19):
                rx = np.array([[1, 0, 0], [0, np.cos(b), -np.sin(b)],
                               [0, np.sin(b), np.cos(b)]])
                actual = points @ rz.T @ rx.T
                self.assertLessEqual(np.max(actual @ hull.equations[:, :3].T
                                            + hull.equations[:, 3]), 1e-12)


class RecordedThumbTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cfg = load_config(str(ROOT / 'config/teleop_config.yaml'))
        cls.cfg.ik.urdf_path = str(ROOT / 'urdf/esrobo_waist_with_head.urdf')
        cls.solver = IkSolver(cls.cfg.ik)
        # Preserve the historical 30 mm envelope regression independently of
        # the commissioned runtime clearance.
        cls.guard = TorsoCollisionGuard(cls.solver, 'right', margin=.03)

    def recorded_state(self, age, *, horizon=0):
        return geometry_state(
            [143, 127, 254, 254, 254, 254, 0, 0, 0, 112],
            time.monotonic()-age, 'right', self.cfg.hand.geometry_feedback_calibration,
            L10_PHYSICAL_JOINT_NAMES, horizon, self.cfg.hand.geometry_feedback_timeout_s)

    def recorded_arm(self):
        q = np.zeros(14)
        # session_1791431178868581105, before the reported 29.15 mm refusal.
        q[7:] = [.00015707963267949, .002914699850830482,
                 .00006981317007977, -.004206243497306334,
                 -.000925024503556995, -.000226892802759263,
                 -.006510078109938850]
        return q

    def test_recorded_zero_pose_keeps_30mm_clearance_with_uncertain_thumb(self):
        g, q = self.guard, self.recorded_arm()
        for age in (0, .0133, .05, .08, .099):
            state = self.recorded_state(age)
            g.hand_state_provider = lambda horizon, state=state: state
            self.assertTrue(g(q, q), g.last_diagnostics)
            self.assertEqual(g.margin, .03)
            self.assertGreater(g.last_diagnostics['clearance_m'],
                               g.last_diagnostics['required_m'])
            self.assertEqual(g.last_diagnostics['thumb_envelope'], 'original_mesh_convex_arc_hull')

    def test_stale_feedback_is_still_rejected(self):
        g, q = self.guard, self.recorded_arm()
        state = self.recorded_state(.101)
        g.hand_state_provider = lambda horizon: state
        self.assertFalse(g(q, q))
        self.assertEqual(g.last_diagnostics['category'], 'hand_geometry_unavailable')

    def test_initial_entry_arm_only_does_not_compute_finger_geometry(self):
        from unittest.mock import Mock
        g, q = self.guard, self.recorded_arm()
        g.hand_state_provider = Mock(side_effect=AssertionError('entry hand geometry'))
        g.begin_entry_certificates(arm_geometry_only=True)
        try:
            lo, hi = q.copy(), q.copy()
            lo[7:] -= np.deg2rad(.2)
            hi[7:] += np.deg2rad(.2)
            self.assertTrue(g.check_arm_only(lo, hi), g.last_diagnostics)
            self.assertTrue(g.freeze_entry_certificates(include_fingers=False))
            self.assertTrue(g.check_entry_coverage(q, q, include_fingers=False))
            g.hand_state_provider.assert_not_called()
            self.assertTrue(all(not g.model.frames[g.geometry.geometryObjects[a].parentFrame].name.startswith('rightHand')
                                for a, _ in g._entry_arm_pairs))
            self.assertEqual(g.last_diagnostics['collision_profile'], 'entry_arm')
        finally:
            g.end_entry_certificates()
            g.hand_state_provider = None

    def test_active_hand_motion_horizon_is_still_checked(self):
        g, q = self.guard, self.recorded_arm()
        state = self.recorded_state(.02, horizon=.5)
        g.hand_state_provider = lambda horizon: state
        self.assertFalse(g(q, q), g.last_diagnostics)
        self.assertEqual(g.last_diagnostics['category'], 'geometry_rejected')

    def test_zero_position_does_not_exempt_a_larger_clearance_requirement(self):
        g, q = self.guard, self.recorded_arm()
        state = self.recorded_state(0)
        g.hand_state_provider = lambda horizon: state
        original = g.margin
        try:
            g.margin = .08
            self.assertFalse(g(q, q))
            self.assertEqual(g.last_diagnostics['category'], 'geometry_rejected')
        finally:
            g.margin = original

    def test_recorded_bent_pinky_zero_target_preserves_uncertainty_and_margin(self):
        # 2026-10-10 return rejection: the old interval box measured 26.803 mm
        # at age 10 ms. The continuous outer mesh hull encloses the same input.
        g, q = self.guard, np.zeros(14)
        raw = [143, 146, 104, 107, 112, 125, 0, 0, 0, 49]
        for age in (0., .005, .01, .02, .05, .099):
            now = time.monotonic()
            state = geometry_state(raw, now-age, 'right',
                self.cfg.hand.geometry_feedback_calibration, L10_PHYSICAL_JOINT_NAMES,
                0., .1, now=now)
            g.hand_state_provider = lambda horizon, state=state: state
            self.assertTrue(g(q, q), g.last_diagnostics)
            self.assertEqual(g.last_diagnostics['required_m'], .03)
            if age == .01:
                # Removing support-plane expansion cannot reduce this clearance.
                self.assertGreaterEqual(g.last_diagnostics['clearance_m'], .04474566742215065)
        original = g.margin
        try:
            g.margin = .05
            self.assertFalse(g(q, q))
        finally:
            g.margin = original

    def test_distal_meshes_are_enclosed_through_all_joint_intervals(self):
        for finger in ('index', 'middle', 'ring', 'pinky'):
            with self.subTest(finger=finger):
                self._assert_finger_mesh_enclosed(finger)

    def _assert_finger_mesh_enclosed(self, finger):
        g = self.guard
        prefix = getattr(self, 'side', 'right') + '_' + finger
        intervals = ({prefix+'_cmc_pitch': (.15, .4), prefix+'_cmc_yaw': (.5, .8),
                      prefix+'_cmc_roll': (.3, .5)} if finger == 'thumb' else
                     {prefix+'_mcp_pitch': (.63, .76), prefix+'_mcp_roll': (0., .07)})
        shape, center, _, _ = next(item for item in g._build_fingers(intervals)
                                   if item[2] == prefix+'_distal')
        hull = ConvexHull(np.asarray(shape.points()) + center)
        root = ET.parse(self.cfg.ik.urdf_path).getroot()
        link = root.find(f"link[@name='{prefix}_distal']")
        collision = link.find('collision')
        mesh = collision.find('geometry/mesh')
        filename = mesh.get('filename').removeprefix('package://')
        raw = (ROOT / 'assets' / filename).read_bytes()
        count = struct.unpack_from('<I', raw, 80)[0]
        dtype = np.dtype([('normal', '<f4', 3), ('vertices', '<f4', (3, 3)), ('attr', '<u2')])
        vertices = np.frombuffer(raw, dtype=dtype, count=count, offset=84)['vertices'].reshape(-1, 3)
        # The mesh convex vertices are sufficient: containment of their hull
        # contains every original triangle, not just a subset of mesh samples.
        points = vertices[ConvexHull(vertices).vertices].astype(float)
        points *= np.fromstring(mesh.get('scale', '1 1 1'), sep=' ')
        origin = collision.find('origin')
        points = points @ g.pin.rpy.rpyToMatrix(np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')).T
        points += np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
        chain = []
        joint = g.model.getJointId(prefix+('_ip' if finger == 'thumb' else '_dip'))
        while joint != g.hand_joint:
            name = g.model.names[joint]
            xml = root.find(f"joint[@name='{name}']")
            mimic = xml.find('mimic')
            if mimic is not None:
                lower, upper = intervals[mimic.get('joint')]
                mul, offset = float(mimic.get('multiplier', '1')), float(mimic.get('offset', '0'))
                lower, upper = sorted((lower*mul+offset, upper*mul+offset))
            else:
                lower, upper = intervals[name]
            qi = g.model.idx_qs[joint]
            lower, upper = max(lower, g.model.lowerPositionLimit[qi]), min(upper, g.model.upperPositionLimit[qi])
            chain.append((joint, np.fromstring(xml.find('axis').get('xyz'), sep=' '), lower, upper))
            joint = g.model.parents[joint]
        # Include independent extremes as well as intermediate rotations; the
        # algorithm must enclose even the larger independent mimic intervals.
        for angles in itertools.product(*(np.linspace(lo, hi, 4) for _, _, lo, hi in chain)):
            actual = points.copy()
            for angle, (joint, axis, _, _) in zip(angles, chain):
                # URDF parsing and the production sweep normalize axes. The
                # vendor left thumb axes are rounded and not exactly unit length;
                # AngleAxis requires a unit axis rather than normalizing for us.
                rotation = g.pin.AngleAxis(float(angle), axis/np.linalg.norm(axis)).matrix()
                actual = actual @ rotation.T
                placement = g.model.jointPlacements[joint]
                actual = actual @ placement.rotation.T + placement.translation
            self.assertLessEqual(np.max(actual @ hull.equations[:, :3].T + hull.equations[:, 3]), 1e-9)


class LeftFingerEnvelopeTests(unittest.TestCase):
    side = 'left'
    _assert_finger_mesh_enclosed = RecordedThumbTests._assert_finger_mesh_enclosed

    @classmethod
    def setUpClass(cls):
        cls.cfg = load_config(str(ROOT / 'config/teleop_config.yaml'))
        cls.cfg.ik.urdf_path = str(ROOT / 'urdf/esrobo_waist_with_head.urdf')
        cls.solver = IkSolver(cls.cfg.ik)
        cls.guard = TorsoCollisionGuard(cls.solver, cls.side, margin=.03)

    def test_all_five_left_distal_meshes_are_inside_continuous_envelopes(self):
        for finger in ('thumb', 'index', 'middle', 'ring', 'pinky'):
            with self.subTest(finger=finger):
                self._assert_finger_mesh_enclosed(finger)

    def test_recorded_return_sweep_still_rejects_contact_at_10mm(self):
        # 2026-10-14: the current thumb pose alone is not a return certificate.
        guard = TorsoCollisionGuard(self.solver, 'left')
        self.assertEqual(guard.margin, .01)
        now = time.monotonic()
        state = geometry_state([254,127,254,254,254,254,15,122,121,112], now,
            'left', self.cfg.hand.geometry_feedback_calibration,
            L10_PHYSICAL_JOINT_NAMES, 0., .1, now=now)
        q = np.zeros(14)
        q[:7] = [-.000593411945678072, -.0005235987755982663,
                 -.00031415926535897936, -.0006457718232379018,
                 -.000017453292519943296, 0., -.00034906585039886593]
        guard.hand_state_provider = lambda horizon: state
        self.assertTrue(guard(q, q), guard.last_diagnostics)
        intervals = dict(state.intervals)
        intervals.update(left_thumb_cmc_pitch=(-.24, .24),
                         left_thumb_cmc_yaw=(-.34, 1.12),
                         left_thumb_cmc_roll=(.12, .78))
        sweep = HandGeometryState(now, intervals, state.calibrated, True)
        drift = np.zeros(14)
        drift[:7] = np.deg2rad(.2)
        self.assertFalse(guard.check_hand_return(q-drift, q+drift, sweep))
        self.assertLess(guard.last_diagnostics['clearance_m'], 0.)
        self.assertEqual(guard.last_diagnostics['required_m'], .01)
        self.assertEqual(guard.last_diagnostics['pair'],
                         ['left_thumb_distal_finger_envelope', 'waist_link1_0'])

    def test_left_recorded_pose_keeps_margin_and_rejects_stale_feedback(self):
        g = self.guard
        raw = [254,127,254,254,254,254,0,128,99,112]
        q = np.zeros(14)
        q[:7] = [-.0324980306721344, -.0374024058702385, .1494699971407944,
                 .0338419341961701, .0034382986264288, .0218340689424491, -.0830078592248503]
        for age in (.01, .099, .101):
            now = time.monotonic()
            state = geometry_state(raw, now-age, self.side,
                self.cfg.hand.geometry_feedback_calibration, L10_PHYSICAL_JOINT_NAMES,
                0., .1, now=now)
            g.hand_state_provider = lambda horizon, state=state: state
            valid = g(q, q)
            if age == .01:
                self.assertTrue(valid, g.last_diagnostics)
                self.assertEqual(g.last_diagnostics['thumb_envelope'], 'original_mesh_convex_arc_hull')
                self.assertEqual(g.margin, .03)
                margin = g.margin
                try:
                    g.margin = .1
                    self.assertFalse(g(q, q))
                finally:
                    g.margin = margin
            elif age > .1:
                self.assertFalse(valid)
                self.assertEqual(g.last_diagnostics['category'], 'hand_geometry_unavailable')

    def test_left_certificates_invalidated_by_hand_change(self):
        g, q = self.guard, np.zeros(14)
        q[:7] = [-.0324980306721344, -.0374024058702385, .1494699971407944,
                 .0338419341961701, .0034382986264288, .0218340689424491, -.0830078592248503]
        raw = [254,127,254,254,254,254,0,128,99,112]
        g.begin_entry_certificates()
        try:
            for i, pitch in enumerate((254, 200)):
                raw[2] = pitch
                now = time.monotonic()
                state = geometry_state(raw, now, self.side,
                    self.cfg.hand.geometry_feedback_calibration, L10_PHYSICAL_JOINT_NAMES,
                    0., .1, now=now)
                g.hand_state_provider = lambda horizon, state=state: state
                g(q, q)
                self.assertFalse(g.last_diagnostics.get('entry_cache_hit', False))
                if i == 0:
                    self.assertTrue(g(q, q), g.last_diagnostics)
                    self.assertTrue(g.last_diagnostics['entry_cache_hit'])
        finally:
            g.end_entry_certificates()

    def test_left_zero_with_recorded_thumb_posture_is_not_automatically_exempt(self):
        now = time.monotonic()
        state = geometry_state([254,127,254,254,254,254,0,128,99,112], now,
            self.side, self.cfg.hand.geometry_feedback_calibration,
            L10_PHYSICAL_JOINT_NAMES, 0., .1, now=now)
        self.guard.hand_state_provider = lambda horizon: state
        self.assertFalse(self.guard(np.zeros(14), np.zeros(14)))
        self.assertEqual(self.guard.last_diagnostics['category'], 'geometry_rejected')
        self.assertEqual(self.guard.last_diagnostics['pair'],
                         ['left_thumb_distal_finger_envelope', 'waist_link1_0'])


if __name__ == '__main__':
    unittest.main()
