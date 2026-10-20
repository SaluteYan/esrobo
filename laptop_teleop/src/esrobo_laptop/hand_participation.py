"""Transactional permission for four-finger pinch assistance (no hardware access).

Confirmed contact retains independently authorized axes after approach motion
stops. Thumb motion alone cannot grant another finger's participation.
"""
from dataclasses import dataclass, replace, field
import numpy as np

POLICY = dict(window_s=.600, minimum_span_s=.100, minimum_samples=3,
              flex_motion_deg=3., side_motion_deg=1., coherence=.8,
              approach_width=.01, jump_window_s=.100, jump_flex_deg=20.,
              jump_side_deg=6., cooldown_s=.300, transition_s=.080,
              history_capacity=64, evidence_lifetime_s=.600,
              contact_confirm_s=.100, release_s=.160)
FLEX = np.array([4, 5, 7, 9])
SIDE = np.array([3, 5, 6, 8])  # Middle's placeholder is masked below.
HAS_SIDE = np.array([True, False, True, True])
START = np.array([7, 12, 17, 22])


@dataclass(frozen=True)
class Sample:
    stamp: float
    raw: np.ndarray
    base: np.ndarray
    relative_tip: np.ndarray
    toward_thumb: np.ndarray


@dataclass(frozen=True)
class State:
    history: tuple
    correction: np.ndarray
    cooldown_until: np.ndarray
    evidence_after: np.ndarray
    evidence_until: np.ndarray = field(default_factory=lambda: np.zeros(10))
    evidence_origin: np.ndarray = field(default_factory=lambda: np.zeros(10))
    evidence_net: np.ndarray = field(default_factory=lambda: np.zeros(10))
    authorized_axes: np.ndarray = field(default_factory=lambda: np.zeros(10, dtype=bool))
    release_axes: np.ndarray = field(default_factory=lambda: np.zeros(10, dtype=bool))
    contact_elapsed: np.ndarray = field(default_factory=lambda: np.zeros(4))


@dataclass(frozen=True)
class Candidate:
    state: State
    fixed: np.ndarray
    lower: np.ndarray
    upper: np.ndarray
    diagnostic: dict

    def solved(self, angles, base):
        correction = np.asarray(angles)-base
        correction[:3] = 0.
        return replace(self.state, correction=correction.copy())


class FingerParticipation:
    def __init__(self, lower, upper, width, *, flex_deg=15., side_deg=5., max_gap_s=.1):
        self.lower, self.upper = np.asarray(lower), np.asarray(upper)
        self.width, self.max_gap = float(width), max_gap_s
        self.radius = np.deg2rad([0., 0., 0., side_deg, flex_deg, flex_deg,
                                 side_deg, flex_deg, side_deg, flex_deg])
        self.state = None
        self.native = None

    def propose(self, base, raw_base, points, stamp, contact):
        if self.native is not None:
            return self.native(self, base, raw_base, points, stamp, contact, POLICY)
        prev = self.state
        history = () if prev is None else prev.history
        dt = 0. if not history else stamp-history[-1].stamp
        if history and dt <= 0:
            raise ValueError('non-advancing finger participation time')
        reset = dt > self.max_gap
        correction = np.zeros(10) if prev is None else prev.correction.copy()
        cooldown = np.zeros(4) if prev is None else prev.cooldown_until.copy()
        after = np.full(4, -np.inf) if prev is None else prev.evidence_after.copy()
        if reset:
            history = ()
            after[:] = stamp
            dt = 0.
        tips = points[START+3]
        toward = points[5]-tips
        length = np.linalg.norm(toward, axis=1)
        toward = toward/np.maximum(length[:, None], 1e-12)
        sample = Sample(stamp, raw_base.copy(), base.copy(),
                        (tips-points[START]).copy(), toward.copy())
        recent = tuple(v for v in history if stamp-v.stamp <= POLICY['window_s']+1e-9)
        jump_history = tuple(v for v in recent if stamp-v.stamp <= POLICY['jump_window_s']+1e-9)
        jump = np.zeros(4, dtype=bool)
        if jump_history:
            changes = abs(np.array([v.raw for v in jump_history])-raw_base)
            jump = ((changes[:, FLEX] > np.deg2rad(POLICY['jump_flex_deg'])).any(axis=0)
                    | ((changes[:, SIDE] > np.deg2rad(POLICY['jump_side_deg'])).any(axis=0) & HAS_SIDE))
            cooldown[jump] = stamp+POLICY['cooldown_s']
            # Quarantine rapid shape changes, retaining history for a later
            # stable, confirmed contact; a jump never immediately grants motion.
        recent = (*recent, sample)[-POLICY['history_capacity']:]
        axis_own = np.zeros(10, dtype=bool)
        own = np.zeros(4, dtype=bool)
        approach = np.zeros(4)
        net_deg = np.zeros((4, 2))
        coherent = np.zeros(4)
        reasons = ['insufficient_history']*4
        for f in range(4):
            samples = [v for v in recent if v.stamp >= after[f]]
            if (len(samples) < POLICY['minimum_samples']
                    or stamp-samples[0].stamp < POLICY['minimum_span_s']-1e-9):
                continue
            cols = [FLEX[f], SIDE[f]] if HAS_SIDE[f] else [FLEX[f]]
            angles = np.array([v.base[cols] for v in samples])
            net = angles[-1]-angles[0]
            net_deg[f, :len(cols)] = np.degrees(net)
            travel = np.sum(abs(np.diff(angles, axis=0)), axis=0)
            ratio = abs(net)/np.maximum(travel, 1e-12)
            thresholds = np.deg2rad([POLICY['flex_motion_deg'], POLICY['side_motion_deg']][:len(cols)])
            qualifies = (abs(net) >= thresholds) & (ratio >= POLICY['coherence'])
            coherent[f] = float(np.max(ratio))
            axis_own[cols] = qualifies
            own[f] = bool(qualifies.any())
            approach[f] = float((sample.relative_tip[f]-samples[0].relative_tip[f])
                                @ samples[0].toward_thumb[f]/self.width)
            reasons[f] = 'own_motion' if own[f] else 'weak_or_inconsistent_motion'
        evidence = np.zeros(10) if prev is None else prev.evidence_until.copy()
        origin = np.zeros(10) if prev is None else prev.evidence_origin.copy()
        proof_net = np.zeros(10) if prev is None else prev.evidence_net.copy()
        authorized = np.zeros(10, dtype=bool) if prev is None else prev.authorized_axes.copy()
        release = np.zeros(10, dtype=bool) if prev is None else prev.release_axes.copy()
        elapsed = np.zeros(4) if prev is None else prev.contact_elapsed.copy()
        if reset:
            evidence[:] = 0.
            elapsed[:] = 0.
        correction = np.clip(correction, -self.radius, self.radius)
        fixed = np.clip(base+correction, self.lower, self.upper)
        lo, hi = fixed.copy(), fixed.copy()
        lo[:3], hi[:3] = self.lower[:3], self.upper[:3]
        budget = np.zeros(10)
        participating = np.zeros(4, dtype=bool)
        for f in range(4):
            cols = [FLEX[f], SIDE[f]] if HAS_SIDE[f] else [FLEX[f]]
            exited = not contact[f] and elapsed[f] > 0.
            if exited:
                after[f] = stamp
                evidence[cols] = 0.
                authorized[cols] = False
            elapsed[f] = elapsed[f]+dt if contact[f] else 0.
            for c, k in enumerate(cols):
                threshold = np.deg2rad(POLICY['flex_motion_deg' if c == 0 else 'side_motion_deg'])
                if (base[k]-origin[k])*np.sign(proof_net[k]) < threshold*.5:
                    evidence[k] = 0.
                if axis_own[k] and approach[f] >= POLICY['approach_width'] and not reset and not exited:
                    evidence[k] = stamp+POLICY['evidence_lifetime_s']
                    proof_net[k] = np.deg2rad(net_deg[f,c])
                    origin[k] = base[k]-proof_net[k]
            if not contact[f]:
                authorized[cols] = False
            if contact[f]:
                release[cols] = False
            blocked = reset or (contact[f] and stamp < cooldown[f])
            confirmed = contact[f] and elapsed[f] >= POLICY['contact_confirm_s']-1e-9
            if confirmed and not blocked:
                authorized[cols] |= evidence[cols] >= stamp
            if not contact[f] and not blocked:
                release[cols] |= abs(fixed[cols]-base[cols]) > 1e-12
            if blocked:
                reasons[f] = 'clock_reset' if reset else 'shape_jump_cooldown'
                continue
            # Intent outlives instantaneous movement, but each axis has its own
            # authorization and bounded source-time budget (never cross-axis).
            for k in cols:
                if confirmed and authorized[k]:
                    budget[k] = self.radius[k]*min(dt/POLICY['transition_s'], 1.)
                    lo[k] = max(self.lower[k], base[k]-self.radius[k], fixed[k]-budget[k])
                    hi[k] = min(self.upper[k], base[k]+self.radius[k], fixed[k]+budget[k])
                    participating[f] = True
                elif not contact[f] and release[k]:
                    budget[k] = self.radius[k]*min(dt/POLICY['release_s'], 1.)
                    offset = fixed[k]-base[k]
                    fixed[k] -= np.clip(offset, -budget[k], budget[k])
                    lo[k] = hi[k] = fixed[k]
                    if abs(fixed[k]-base[k]) < 1e-12:
                        release[k] = False
            if participating[f]:
                reasons[f] = 'confirmed_contact'
            elif not contact[f] and any(budget[cols]):
                reasons[f] = 'contact_exit_release'
            elif contact[f] and not confirmed:
                reasons[f] = 'contact_confirmation'
        state = State(recent, correction, cooldown, after, evidence, origin, proof_net, authorized, release, elapsed)
        diagnostic = dict(eligible=participating.tolist(), reason=reasons,
                          authorized_axes=authorized.tolist(), axis_own_motion=axis_own.tolist(),
                          contact_elapsed_s=elapsed.tolist(),
                          evidence_remaining_s=np.maximum(evidence-stamp, 0.).tolist(),
                          own_motion=own.tolist(), net_motion_deg=net_deg.tolist(),
                          coherence=coherent.tolist(), approach_width=approach.tolist(),
                          shape_jump=jump.tolist(), cooldown_remaining_s=np.maximum(cooldown-stamp, 0.).tolist(),
                          retained_correction_deg=np.degrees(correction).tolist(),
                          correction_budget_deg=np.degrees(budget).tolist(), clock_reset=reset)
        return Candidate(state, fixed, lo, hi, diagnostic)

    def commit(self, state):
        self.state = state
