"""Adapter to dex-retargeting's actual VectorOptimizer.

No hardware imports/outputs. Live selection is explicit in hand_solver and uses
the isolated worker environment; see docs/09_TECHNICAL_SCHEME.md.
"""
import copy
import hashlib
import json
from pathlib import Path
import tempfile
import time
import xml.etree.ElementTree as ET

import numpy as np

from .hand_retarget import HandModel, FINGERS, TIPS, BONES
from .hand_shape_filter import HandShapeFilter

UPSTREAM_COMMIT = '3f56141bc8bd2760d5e452e382937269554ebb21'
VERSION = 'left-dex-vector-experiment-v2'
# 0 is the fixed neutral model MCP centroid, not a moving thumb joint.
PRESETS = {
    'wrist5': [(1, int(i)) for i in TIPS],
    'wrist9': [(1, int(i)) for i in TIPS] + [(5, int(i)) for i in TIPS[1:]],
    'root5': [(1, 5), (7, 10), (12, 15), (17, 20), (22, 25)],
    'palm10': [(0, int(i)) for i in TIPS] + [(0, i) for i in (3, 8, 13, 18, 23)],
    'bones19': [tuple(map(int, pair)) for pair in BONES] + [(5, int(i)) for i in TIPS[1:]],
}


def export_hand(cfg, model, path):
    """Exact left-hand subtree plus fixed markers matching HandModel.forward.

    Pinocchio computes physical FK; its mimic adaptor applies the original
    URDF ratios. Active bounds already intersect passive, manual/control limits.
    Meshes/inertias are unnecessary for kinematic fitting and are not copied.
    """
    source = ET.parse(cfg.ik.urdf_path).getroot()
    joints = {j.get('name')[5:]: j for j in source.findall('joint')
              if j.get('name', '').startswith('left_')}
    root = ET.Element('robot', name='left_l20lite_vector_experiment')
    ET.SubElement(root, 'link', name='leftHand_link')
    children = {}
    for name in model.joint_index:
        j = copy.deepcopy(joints[name])
        child = j.find('child').get('link')
        ET.SubElement(root, 'link', name=child)
        if name in model.names:
            k = model.names.index(name)
            j.find('limit').set('lower', repr(float(model.lower[k])))
            j.find('limit').set('upper', repr(float(model.upper[k])))
        root.append(j)
        children[name] = child

    def marker(i, parent, offset):
        name = f'vector_point_{i}'
        ET.SubElement(root, 'link', name=name)
        j = ET.SubElement(root, 'joint', name=name+'_fixed', type='fixed')
        ET.SubElement(j, 'parent', link=parent)
        ET.SubElement(j, 'child', link=name)
        ET.SubElement(j, 'origin', xyz=' '.join(map(str, offset)), rpy='0 0 0')

    marker(0, 'leftHand_link', model.origin)
    marker(1, 'leftHand_link', [0, 0, 0])
    for i, name in [(2, 'thumb_cmc_pitch'), (3, 'thumb_mcp'), (4, 'thumb_ip')]:
        marker(i, children[name], [0, 0, 0])
    marker(5, children['thumb_ip'], model.tip_offsets[0])
    for f, finger in enumerate(FINGERS[1:]):
        start = 7+5*f
        for delta, suffix in enumerate(('_mcp_pitch', '_pip', '_dip')):
            marker(start+delta, children[finger+suffix], [0, 0, 0])
        marker(start+3, children[finger+'_dip'], model.tip_offsets[f+1])
    ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)


class VectorHandRetargeter:
    """Ten bounded axes jointly fitted to unprojected vectors, transactional.

    This deliberately does NOT load v13 contact/permission logic. The same
    input shape filter is retained and upstream output LP filtering is disabled
    to avoid confounding the comparison with a second smoother.
    """
    def __init__(self, cfg, *, preset='wrist5', scale_mode='palm',
                 scale_factor=1., reference_width_m=None, objective_backend='upstream'):
        import torch
        import dex_retargeting.optimizer as upstream
        from dex_retargeting.retargeting_config import RetargetingConfig
        if preset not in PRESETS:
            raise ValueError('unknown vector preset')
        if objective_backend not in ('upstream', 'analytic'):
            raise ValueError('unknown vector objective backend')
        self.objective_backend = objective_backend
        if scale_mode not in ('palm', 'fixed', 'none'):
            raise ValueError('unknown hand scale mode')
        if not np.isfinite(scale_factor) or scale_factor <= 0:
            raise ValueError('scale factor must be positive and finite')
        if scale_mode == 'fixed' and (reference_width_m is None or
                not np.isfinite(reference_width_m) or not .015 <= reference_width_m <= .15):
            raise ValueError('fixed scale requires a valid measured reference width')
        if scale_mode != 'fixed' and reference_width_m is not None:
            raise ValueError('reference width only applies to fixed scale')
        self.scale_mode = scale_mode
        self.scale_factor = float(scale_factor)
        self.reference_width_m = reference_width_m
        torch.set_num_threads(1)
        self.model = HandModel(cfg)
        self.shape_filter = HandShapeFilter(cfg.hand)
        self.preset = preset
        self.pairs = np.array(PRESETS[preset], dtype=int)
        self._temp = tempfile.TemporaryDirectory(prefix='esrobo-dex-vector-')
        path = Path(self._temp.name)/'left_hand.urdf'
        export_hand(cfg, self.model, path)
        config = RetargetingConfig.from_dict(dict(
            type='vector', urdf_path=str(path),
            target_joint_names=['left_'+n for n in self.model.names],
            target_origin_link_names=[f'vector_point_{a}' for a, b in self.pairs],
            target_task_link_names=[f'vector_point_{b}' for a, b in self.pairs],
            target_link_human_indices=self.pairs.T.tolist(),
            scaling_factor=1., normal_delta=.004, huber_delta=.02,
            low_pass_alpha=-1., has_joint_limits=True, ignore_mimic_joint=False))
        # Direct construction avoids upstream build()'s unowned temp directory.
        from dex_retargeting.robot_wrapper import RobotWrapper
        from dex_retargeting.kinematics_adaptor import MimicJointKinematicAdaptor
        robot = RobotWrapper(str(path))
        self.optimizer = upstream.VectorOptimizer(robot, config.target_joint_names,
            config.target_origin_link_names, config.target_task_link_names,
            config.target_link_human_indices, huber_delta=.02, norm_delta=.004, scaling=1.)
        js = ET.parse(path).getroot().findall('joint')
        mimics = [(j.get('name'), j.find('mimic')) for j in js if j.find('mimic') is not None]
        self.optimizer.set_kinematic_adaptor(MimicJointKinematicAdaptor(
            robot, config.target_joint_names, [m.get('joint') for _, m in mimics],
            [n for n, _ in mimics], [float(m.get('multiplier', '1')) for _, m in mimics],
            [float(m.get('offset', '0')) for _, m in mimics]))
        # Upstream default expands bounds by 1e-3 rad. Do not expand ours.
        self.optimizer.set_joint_limit(np.column_stack((self.model.lower, self.model.upper)), epsilon=0.)
        self.optimizer.opt.set_maxeval(80)
        self.optimizer.opt.set_ftol_abs(1e-8)
        self._origin_indices = self.optimizer.origin_link_indices.numpy().copy()
        self._task_indices = self.optimizer.task_link_indices.numpy().copy()
        children = {j.get('name')[5:]:j.find('child').get('link') for j in js
                    if j.get('name', '').startswith('left_')}
        landmarks = [(2,'thumb_cmc_pitch',None),(3,'thumb_mcp',None),
                     (4,'thumb_ip',None),(5,'thumb_ip',self.model.tip_offsets[0])]
        for f, finger in enumerate(FINGERS[1:]):
            start = 6+5*f
            landmarks.extend([(start,finger+('_mcp_pitch' if finger=='middle' else '_mcp_roll'),None),
                (start+1,finger+'_mcp_pitch',None),(start+2,finger+'_pip',None),
                (start+3,finger+'_dip',None),(start+4,finger+'_dip',self.model.tip_offsets[f+1])])
        self._diagnostic_links = [(slot,robot.get_link_index(children[name]),offset)
                                  for slot,name,offset in landmarks]
        self.previous = self.model.neutral.copy()
        self.source_time = None
        self._pending = None
        self._serial = 0
        self.metadata = dict(version=VERSION, upstream_commit=UPSTREAM_COMMIT,
            upstream_optimizer_sha256=hashlib.sha256(Path(upstream.__file__).read_bytes()).hexdigest(),
            model_fingerprint=self.model.digest, preset=preset, pairs=self.pairs.tolist(),
            scale_mode=scale_mode, scale_factor=self.scale_factor,
            reference_width_m=reference_width_m, robot_width_m=self.model.width,
            wrist_marker='leftHand_link origin / PICO node 1',
            huber_delta_m=.02, normal_delta=.004, max_evaluations=80,
            output_filter=False, input_filter=True, exact_bounds=True,
            ftol_abs=1e-8, consistent_temporal_objective=True,
            objective_backend=objective_backend)
        self.fingerprint = hashlib.sha256(json.dumps(self.metadata, sort_keys=True).encode()).hexdigest()

    def human_points(self, poses):
        """Mapped human geometry before the shared shape filter, in metres.

        palm: alpha = extra * robot_width / current_human_width;
        fixed: alpha = extra * robot_width / reference_human_width;
        none: alpha = extra. Synthetic solve(target=...) bypasses this mapping.
        No online size estimate/state is silently advanced on rejected frames.
        """
        points = self.model.human_target(poses)
        width = float(np.linalg.norm(np.asarray(poses)[7, :3]-np.asarray(poses)[22, :3]))
        factor = self.scale_factor
        if self.scale_mode == 'fixed':
            factor *= width/self.reference_width_m
        elif self.scale_mode == 'none':
            factor *= width/self.model.width
        return self.model.origin+(points-self.model.origin)*factor

    def close(self):
        self._temp.cleanup()

    def discard(self):
        self._pending = None

    def commit(self, candidate_id):
        if self._pending is None or self._pending[0] != candidate_id:
            return False
        _, self.previous, self.source_time, shape = self._pending
        if shape is not None:
            self.shape_filter.commit(shape)
        self.discard()
        return True

    def _diagnostic_points(self, q):
        # Reuse compiled FK; diagnostics need positions, not another full Jacobian.
        opt = self.optimizer
        full = np.zeros(opt.num_joints)
        full[opt.idx_pin2target] = q
        opt.adaptor.forward_qpos(full)
        opt.robot.compute_forward_kinematics(full)
        points = np.zeros((26,3))
        for slot, link, offset in self._diagnostic_links:
            pose = opt.robot.get_link_pose(link)
            points[slot] = pose[:3,3] if offset is None else pose[:3,3]+pose[:3,:3]@offset
        points[11] -= [0,0,.02]
        points[0] = points[[7,12,17,22]].mean(axis=0)
        return points

    def _analytic_objective(self, vectors, previous):
        """Same vector-distance SmoothL1 + temporal loss, without autograd.

        FK, exact mimic Jacobians, SLSQP and tolerances remain upstream's.
        Avoid constructing a Torch graph at every small solver evaluation.
        """
        opt = self.optimizer
        full = np.zeros(opt.num_joints)
        origins, tasks = self._origin_indices, self._task_indices
        beta = float(opt.huber_loss.beta)
        target = np.asarray(vectors)*opt.scaling
        def objective(q, grad):
            full[opt.idx_pin2target] = q
            opt.adaptor.forward_qpos(full)
            opt.robot.compute_forward_kinematics(full)
            poses = [opt.robot.get_link_pose(i) for i in opt.computed_link_indices]
            points = np.array([pose[:3, 3] for pose in poses])
            delta = points[tasks]-points[origins]-target
            distance = np.linalg.norm(delta, axis=1)
            loss = np.where(distance < beta, .5*distance**2/beta, distance-.5*beta).mean()
            change = q-previous
            if grad.size:
                jac = np.array([pose[:3, :3] @ opt.robot.compute_single_link_local_jacobian(full, i)[:3]
                                for pose, i in zip(poses, opt.computed_link_indices)])
                jac = opt.adaptor.backward_jacobian(jac)
                direction = delta/np.maximum(distance, beta)[:, None]/len(distance)
                grad[:] = np.einsum('vi,vij->j', direction, jac[tasks]-jac[origins]) + 2*opt.norm_delta*change
            return float(loss+opt.norm_delta*(change@change))
        return objective

    def build_vectors(self, points):
        """Objective geometry hook; the default keeps the original experiment."""
        return points[self.pairs[:, 1]]-points[self.pairs[:, 0]]

    def solve(self, poses, *, source_time, budget_s=.020, commit=False, target=None,
              include_diagnostics=False):
        started = time.monotonic()
        self.discard()
        stamp = float(source_time)
        if not np.isfinite(stamp) or (self.source_time is not None and stamp <= self.source_time):
            raise ValueError('non-advancing hand source time')
        if not np.isfinite(budget_s) or budget_s <= 0:
            raise ValueError('invalid compute budget')
        points = self.human_points(poses) if target is None else np.asarray(target, dtype=float)
        if points.shape != (26, 3) or not np.isfinite(points).all():
            raise ValueError('invalid hand points')
        for start in (2, 7, 12, 17, 22):
            if np.any(np.linalg.norm(np.diff(points[start:start+4], axis=0), axis=1) < 1e-5):
                raise ValueError('degenerate finger bone')
        shape = None
        if target is None:
            shape, _ = self.shape_filter.propose(points, stamp)
            points = shape.output.copy()
        # Both models use the neutral MCP centroid as the fixed palm marker.
        points = points.copy()
        points[0] = self.model.origin
        vectors = self.build_vectors(points)
        upstream_objective = self.optimizer.get_objective_function(vectors, np.array([]), self.previous.copy())
        previous = self.previous.copy()
        def objective(q, grad):
            # Pinned upstream adds this regularizer's gradient, but omits its
            # scalar loss. Restore function/gradient consistency for SLSQP.
            return upstream_objective(q, grad) + .004*float((q-previous)@(q-previous))
        if self.objective_backend == 'analytic':
            objective = self._analytic_objective(vectors, previous)
        remaining = budget_s-(time.monotonic()-started)
        if remaining <= 0:
            raise TimeoutError('vector preprocessing exceeded deadline')
        self.optimizer.opt.set_maxtime(remaining)
        self.optimizer.opt.set_min_objective(objective)
        # Do not use upstream retarget() which catches failure and returns old
        # qpos as a new result. Reject numerical failure and timed-out results.
        q = np.asarray(self.optimizer.opt.optimize(self.previous), dtype=float)
        status = int(self.optimizer.opt.last_optimize_result())
        elapsed = time.monotonic()-started
        if status == 6 or elapsed >= budget_s:
            raise TimeoutError('vector solver exceeded deadline')
        if status <= 0 or not np.isfinite(q).all():
            raise RuntimeError(f'vector optimizer failure {status}')
        if np.any(q < self.model.lower-1e-9) or np.any(q > self.model.upper+1e-9):
            raise RuntimeError('vector output outside joint bounds')
        q = np.clip(q, self.model.lower, self.model.upper)
        diagnostic = dict(solve_ms=elapsed*1000, status=status,
                          evaluations=self.optimizer.opt.get_numevals(), preset=self.preset)
        geometry = {}
        if include_diagnostics:
            solved = self._diagnostic_points(q)
            wrist_error = (solved[TIPS]-solved[1])-(points[TIPS]-points[1])
            pinch_error = (solved[TIPS[1:]]-solved[5])-(points[TIPS[1:]]-points[5])
            diagnostic.update(tip_error_mm=(1000*np.linalg.norm(wrist_error, axis=1)).tolist(),
                pinch_error_mm=(1000*np.linalg.norm(pinch_error, axis=1)).tolist(),
                limited_axes=[n for i,n in enumerate(self.model.names)
                              if min(q[i]-self.model.lower[i],self.model.upper[i]-q[i]) < 1e-5],
                tip_error_reference='wrist_relative_vectors',
                contact_projection=False)
            if 'geometry_mode' in self.metadata:
                objective_points = solved.copy()
                objective_points[0] = self.model.origin
                residual = objective_points[self.pairs[:,1]]-objective_points[self.pairs[:,0]]-vectors
                diagnostic.update(target_geometry=self.metadata['geometry_mode'],
                    lateral_geometry=self.metadata.get('lateral_geometry'),
                    objective_vectors_m=vectors.tolist(),
                    objective_error_mm=(1000*np.linalg.norm(residual,axis=1)).tolist(),
                    tip_error_reference='unmodified_human_wrist_relative_vectors')
            geometry = dict(target_points=points.tolist(), solved_points=solved.tolist())
        elapsed = time.monotonic()-started
        if elapsed >= budget_s:
            raise TimeoutError('vector diagnostics exceeded deadline')
        diagnostic['solve_ms'] = elapsed*1000
        self._serial += 1
        self._pending = (self._serial, q.copy(), stamp, shape)
        result = dict(joints=q.tolist(), candidate_id=self._serial, mapping_version=VERSION,
                      source_time=stamp, fingerprint=self.fingerprint,
                      diagnostics=diagnostic, **geometry)
        if commit:
            self.commit(self._serial)
        return result
