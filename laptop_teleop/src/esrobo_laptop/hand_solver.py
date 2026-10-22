"""Left-hand solver selection and lightweight input contract (no Torch import)."""
import hashlib
import json
import math
from pathlib import Path

VECTOR_VERSION = 'left-dex-wrist5-live-v1'
GEOMETRY_VERSION = 'left-dex-wrist5-anatomical-live-v2'
LATERAL_VERSION = 'left-dex-wrist5-anatomical-live-v3'
OPTIMIZER_SHA256 = '6b816d65c81f2e959ea6b76876bb03c2b768be95064a1b4b9e1563d36262cdf8'


def selection(cfg):
    name = getattr(cfg.hand, 'left_retarget_solver', 'v13')
    if name not in ('v13', 'wrist5'):
        raise ValueError('left_retarget_solver must be v13 or wrist5')
    if name == 'wrist5' and cfg.hand.left_retarget_mode != 'urdf_10':
        raise ValueError('wrist5 requires calibrated urdf_10 output mapping')
    geometry(cfg)
    geometry_parameters(cfg)
    return name


def geometry(cfg):
    mode = getattr(cfg.hand, 'left_vector_geometry', 'original')
    if mode not in ('original', 'aligned_thumb_fingers'):
        raise ValueError('left_vector_geometry must be original or aligned_thumb_fingers')
    return mode


def geometry_parameters(cfg):
    """Bounded geometric corrections, independent of contact detection."""
    gain = getattr(cfg.hand, 'left_vector_thumb_ulnar_gain', 1.)
    offsets = getattr(cfg.hand, 'left_vector_finger_lateral_offset_mm', (0.,)*4)
    numeric = lambda x: type(x) in (int, float) and math.isfinite(x)
    if not numeric(gain) or not 1. <= gain <= 1.2:
        raise ValueError('left_vector_thumb_ulnar_gain must be finite within 1..1.2')
    if (not isinstance(offsets, (list, tuple)) or len(offsets) != 4
            or any(not numeric(x) or abs(x) > 5. for x in offsets)):
        raise ValueError('left_vector_finger_lateral_offset_mm requires four finite offsets within +/-5 mm')
    return dict(thumb_ulnar_gain=float(gain), finger_lateral_offset_mm=list(map(float, offsets)),
                thumb_transition_mm=5., frame='neutral_model_palm_lateral')


def contract(cfg, model_digest=None):
    from .hand_retarget import VERSION, fingerprint
    digest = fingerprint(cfg) if model_digest is None else model_digest
    if selection(cfg) == 'v13':
        return VERSION, digest
    source = Path(__file__).with_name('hand_vector_experiment.py')
    policy = dict(version=VECTOR_VERSION, model=digest, preset='wrist5',
                  scale_mode='palm', scale_factor=1., objective_backend='analytic', optimizer_sha256=OPTIMIZER_SHA256,
                  adapter_sha256=hashlib.sha256(source.read_bytes()).hexdigest())
    if geometry(cfg) != 'original':
        policy.update(version=GEOMETRY_VERSION, target_geometry=geometry(cfg),
            geometry_sha256=hashlib.sha256(source.with_name('hand_vector_geometry.py').read_bytes()).hexdigest())
        parameters = geometry_parameters(cfg)
        policy['lateral_geometry'] = parameters
        if parameters['thumb_ulnar_gain'] != 1. or any(parameters['finger_lateral_offset_mm']):
            policy['version'] = LATERAL_VERSION
    return policy['version'], hashlib.sha256(json.dumps(policy, sort_keys=True).encode()).hexdigest()


def create_solver(cfg):
    if selection(cfg) == 'v13':
        from .hand_retarget import HandRetargeter
        return HandRetargeter(cfg)
    if geometry(cfg) == 'original':
        from .hand_vector_experiment import VectorHandRetargeter
        solver = VectorHandRetargeter(cfg, preset='wrist5', objective_backend='analytic')
    else:
        from .hand_vector_geometry import GeometryVectorHandRetargeter
        solver = GeometryVectorHandRetargeter(cfg, geometry_mode=geometry(cfg))
    if solver.metadata['upstream_optimizer_sha256'] != OPTIMIZER_SHA256:
        solver.close()
        raise RuntimeError('dex optimizer source mismatch; reinstall pinned vector environment')
    return solver
