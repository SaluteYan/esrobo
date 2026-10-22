"""Opt-in target construction; live factory admits aligned_thumb_fingers only.

Modes distinguish four-only changes from a shared palm reference for all five.
Bounds, optimizer, source clock, filter and transactions remain inherited.
"""
import hashlib
import json
from pathlib import Path
import numpy as np
from .hand_vector_experiment import VectorHandRetargeter
from .hand_retarget import TIPS

GEOMETRY_MODES = ('offset', 'palm_length', 'finger_length', 'segment_length',
                  'offset_all', 'hand_segments', 'aligned_thumb_fingers',
                  'aligned_thumb_segments', 'root_relative')


class GeometryVectorHandRetargeter(VectorHandRetargeter):
    def __init__(self, cfg, *, geometry_mode):
        if geometry_mode not in GEOMETRY_MODES:
            raise ValueError('unknown target geometry mode')
        from .hand_solver import geometry_parameters
        parameters = geometry_parameters(cfg)
        super().__init__(cfg, preset='root5' if geometry_mode=='root_relative' else 'wrist5', objective_backend='analytic')
        self.geometry_mode = geometry_mode
        self.lateral_parameters = parameters
        self._lateral = self.model.basis[:, 1].copy()
        self._finger_offsets = np.asarray(parameters['finger_lateral_offset_mm'])[:, None]*.001*self._lateral
        self._thumb_gain = parameters['thumb_ulnar_gain']
        self.four_indices = np.array([
            [7,8,9,10], [12,13,14,15], [17,18,19,20], [22,23,24,25]])
        # Fixed root reference for the experiment, NOT a substitute for FK.
        # Candidate FK still includes movable MCP offsets caused by side-splay.
        neutral = self.model.forward(np.zeros(10))[0]
        self.four_roots = neutral[self.four_indices[:, 0]].copy()
        self.robot_lengths = np.linalg.norm(np.diff(neutral[self.four_indices], axis=1), axis=2)
        self.thumb_root = neutral[2].copy()
        self.thumb_lengths = np.linalg.norm(np.diff(neutral[2:6], axis=0), axis=1)
        self.metadata.update(target_geometry_version='anatomical-geometry-v1',
            target_geometry_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            geometry_mode=geometry_mode,
            thumb_target=('aligned_palm' if geometry_mode in ('offset_all','aligned_thumb_fingers','aligned_thumb_segments','root_relative') else
                          'segment_length' if geometry_mode=='hand_segments' else 'unchanged_wrist5'),
            fixed_reference_roots_m=self.four_roots.tolist(),
            robot_segment_lengths_m=self.robot_lengths.tolist(),
            thumb_root_m=self.thumb_root.tolist(),thumb_segment_lengths_m=self.thumb_lengths.tolist(),
            target_size_source='filtered_input_bone_lengths',
            lateral_geometry=parameters if geometry_mode=='aligned_thumb_fingers' else None,
            scope='live_opt_in' if geometry_mode=='aligned_thumb_fingers' else 'offline_only')
        self.fingerprint = hashlib.sha256(json.dumps(self.metadata, sort_keys=True).encode()).hexdigest()

    def build_vectors(self, points):
        vectors = super().build_vectors(points)
        if self.geometry_mode == 'offset_all':
            # human_target already aligns the four-MCP centroid and palm axes.
            # Treat that as a virtual robot-root frame; do NOT claim it is a
            # physically calibrated human anatomical wrist location.
            return points[TIPS].copy()
        if self.geometry_mode == 'offset':
            vectors[1:] += points[1]
        elif self.geometry_mode == 'palm_length':
            centre = points[self.four_indices[:, 0]].mean(axis=0)
            palm = centre-points[1]
            length = np.linalg.norm(palm)
            if not np.isfinite(length) or length < 1e-5:
                raise ValueError('degenerate palm length')
            vectors[1:] = palm*(np.linalg.norm(self.model.origin)/length)+(points[TIPS[1:]]-centre)
        else:
            bones = np.diff(points[self.four_indices], axis=1)
            lengths = np.linalg.norm(bones, axis=2)
            if not np.isfinite(lengths).all() or np.any(lengths < 1e-5):
                raise ValueError('degenerate finger bone')
            if self.geometry_mode in ('finger_length', 'aligned_thumb_fingers', 'root_relative'):
                scale = self.robot_lengths.sum(axis=1)/lengths.sum(axis=1)
                displacement = bones.sum(axis=1)*scale[:, None]
            else:
                displacement = (bones*(self.robot_lengths/lengths)[:, :, None]).sum(axis=1)
            vectors[1:] = self.four_roots+displacement
            if self.geometry_mode == 'root_relative':
                # Moving MCP origins are taken from each candidate's real FK,
                # and their Jacobians are subtracted by VectorOptimizer.
                vectors[1:] = displacement
            if self.geometry_mode in ('aligned_thumb_fingers', 'aligned_thumb_segments','root_relative'):
                vectors[0] = points[5]
            if self.geometry_mode == 'hand_segments':
                bones = np.diff(points[2:6], axis=0)
                lengths = np.linalg.norm(bones, axis=1)
                if not np.isfinite(lengths).all() or np.any(lengths < 1e-5):
                    raise ValueError('degenerate thumb bone')
                vectors[0] = self.thumb_root+(bones*(self.thumb_lengths/lengths)[:,None]).sum(axis=0)
        if self.geometry_mode == 'aligned_thumb_fingers':
            # Only the thumb's own filtered position drives this correction.
            # Positive lateral is toward the index; negative is toward the little
            # finger. No nearest-finger selection or contact projection is used.
            u = float((vectors[0]-self.model.origin) @ self._lateral)
            progress = min(1., max(0., -u/.005))
            blend = progress*progress*(3.-2.*progress)
            vectors[0] += (self._thumb_gain-1.)*min(u, 0.)*blend*self._lateral
            vectors[1:] += self._finger_offsets
        return vectors
