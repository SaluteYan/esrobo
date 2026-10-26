"""Source-time contact evidence from optical landmarks, not a touch sensor.

The returned state is a candidate. Only a successfully published hand result
may commit it. No FK, optimizer, hardware access or unbounded history.
"""
from dataclasses import dataclass, field
import numpy as np


POLICY = dict(algorithm="episode_pair_v3",
              close_m=.008, isolated_enter_m=.016, isolated_exit_m=.020,
              multi_enter_m=.012, ordinary_exit_m=.012,
              separation_m=.005, separation_ratio=.6,
              close_confirm_s=.060, relative_confirm_s=.100, exit_confirm_s=.060,
              departure_m=.002, approach_m=.002, own_approach_width=.050,
              own_closure_share=.25, approach_coherence=.8, evidence_lifetime_s=.300,
              neighbor_motion_ratio=.5, neighbor_alignment=.85,
              history_s=.300, minimum_history_s=.080, history_capacity=64, episode_radius_m=.020, progress_window_s=.100,
              progress_width=.002, axis_history_s=.600,
              projection_s=.080, clock_gap_s=.100)


@dataclass(frozen=True)
class State:
    stamp: float
    active: np.ndarray
    projection: np.ndarray
    entering: np.ndarray
    leaving: np.ndarray
    minimum: np.ndarray
    history: tuple
    approach_until: np.ndarray
    evidence_after: np.ndarray
    episode_id: np.ndarray = field(default_factory=lambda: np.zeros(4, dtype=np.int64))
    progress_at: np.ndarray = field(default_factory=lambda: np.zeros(4))
    progress_value: np.ndarray = field(default_factory=lambda: np.zeros(4))
    episode_direction: np.ndarray = field(default_factory=lambda: np.zeros((4, 3)))
    confirmed_at: np.ndarray = field(default_factory=lambda: np.zeros(4))
    axis_until: np.ndarray = field(default_factory=lambda: np.zeros(10))
    axis_source: np.ndarray = field(default_factory=lambda: np.zeros(10))
    axis_after: np.ndarray = field(default_factory=lambda: np.full(10, np.inf))
    frozen_axes: np.ndarray = field(default_factory=lambda: np.zeros(10, bool))


class HandContact:
    def __init__(self):
        self.state = None

    def propose(self, distances, points, width, stamp, *, human_width=None, base=None, quarantine=None):
        human_width = width if human_width is None else float(human_width)
        d = np.asarray(distances, dtype=float)
        if (d.shape != (4,) or not np.isfinite(d).all() or np.any(d < 0)
                or not np.isfinite(stamp) or not np.isfinite(width) or width <= 0
                or not np.isfinite(human_width) or human_width <= 0):
            raise ValueError("invalid hand contact input")
        p = np.asarray(points, dtype=float)
        if p.shape != (26, 3) or not np.isfinite(p).all():
            raise ValueError("invalid contact landmarks")
        # Episode lifecycle is shared by geometric detection and per-axis permission.
        from .hand_participation import POLICY as AXIS_POLICY, FLEX, SIDE, HAS_SIDE
        base = np.zeros(10) if base is None else np.asarray(base, dtype=float)
        if base.shape != (10,) or not np.isfinite(base).all():
            raise ValueError("invalid contact joint baseline")
        blocked = np.zeros(4, bool) if quarantine is None else np.asarray(quarantine, bool)
        prev = self.state
        dt = 0. if prev is None else stamp-prev.stamp
        if prev is not None and dt <= 0:
            raise ValueError("non-advancing hand source time")
        reset = dt > POLICY["clock_gap_s"]
        gap_dt = dt
        if reset:
            dt = 0.
        active = np.zeros(4, bool) if prev is None else prev.active.copy()
        projection = np.zeros(4) if prev is None else prev.projection.copy()
        entering = np.zeros(4) if prev is None or reset else prev.entering.copy()
        leaving = np.zeros(4) if prev is None or reset else prev.leaving.copy()
        minimum = d.copy() if prev is None else prev.minimum.copy()
        history = () if prev is None or reset else tuple(
            h for h in prev.history if stamp-h[0] <= POLICY["axis_history_s"])
        until = np.zeros(4) if prev is None else prev.approach_until.copy()
        after = np.full(4, np.inf) if prev is None else prev.evidence_after.copy()
        ids = np.zeros(4, dtype=np.int64) if prev is None else prev.episode_id.copy()
        progress_at = np.full(4, stamp) if prev is None else prev.progress_at.copy()
        progress_value = np.zeros(4) if prev is None else prev.progress_value.copy()
        direction = np.zeros((4, 3)) if prev is None else prev.episode_direction.copy()
        confirmed_at = np.zeros(4) if prev is None else prev.confirmed_at.copy()
        axis_until = np.zeros(10) if prev is None else prev.axis_until.copy()
        axis_source = np.zeros(10) if prev is None else prev.axis_source.copy()
        axis_after = np.full(10, np.inf) if prev is None else prev.axis_after.copy()
        frozen = np.zeros(10, bool) if prev is None else prev.frozen_axes.copy()
        tips = p[[10, 15, 20, 25]]
        relative = (tips-p[[7, 12, 17, 22]])/width
        starts = np.array([7, 12, 17, 22])
        chains = p[starts[:, None]+np.arange(4)]
        bone_lengths = np.linalg.norm(np.diff(chains, axis=1), axis=2).sum(axis=1)/width
        toward = p[5]-tips
        toward /= np.maximum(np.linalg.norm(toward, axis=1)[:, None], 1e-12)
        history = (*history, (stamp, d.copy(), relative, toward, bone_lengths, base.copy()))[-POLICY["history_capacity"]:]
        own_distance, coherence, share = np.zeros(4), np.zeros(4), np.zeros(4)
        approach, coupled = np.zeros(4, bool), np.zeros(4, bool)
        episode_reason = ['tracking' if active[f] else 'candidate' if np.isfinite(after[f]) else 'outside' for f in range(4)]
        positions = np.array([h[2] for h in history])

        def invalidate(f, reason):
            cols = [FLEX[f], SIDE[f]] if HAS_SIDE[f] else [FLEX[f]]
            after[f] = np.inf
            active[f] = False
            until[f] = entering[f] = leaving[f] = confirmed_at[f] = 0.
            axis_until[cols] = axis_source[cols] = 0.
            axis_after[cols] = np.inf
            frozen[cols] = False
            episode_reason[f] = reason

        for f in range(4):
            cols = [FLEX[f], SIDE[f]] if HAS_SIDE[f] else [FLEX[f]]
            if blocked[f] or (reset and not active[f]):
                invalidate(f, 'shape_quarantine' if blocked[f] else 'clock_reset')
                continue
            if reset:
                # A verified static contact may survive an input hold. No new
                # proof or confirmation time can be earned inside the gap.
                confirmed_at[f] += gap_dt
                until[f] = 0.
                for k in cols:
                    if not frozen[k]:
                        axis_after[k] = stamp
                        axis_until[k] = axis_source[k] = 0.
                episode_reason[f] = 'confirmed_clock_hold'
            if not active[f] and np.isfinite(after[f]):
                progress = float(relative[f] @ direction[f])
                if progress-progress_value[f] >= POLICY['progress_width']-1e-12:
                    progress_value[f], progress_at[f] = progress, stamp
                reason = ('outside' if d[f] >= POLICY['episode_radius_m'] else
                          'retreated' if d[f]-minimum[f] >= POLICY['departure_m'] else
                          'progress_stalled' if stamp-progress_at[f] >= POLICY['progress_window_s']-1e-9 else None)
                if reason:
                    invalidate(f, reason)
            if not np.isfinite(after[f]) and d[f] < POLICY['episode_radius_m']:
                ids[f] += 1
                after[f] = progress_at[f] = stamp
                direction[f] = toward[f]
                progress_value[f] = float(relative[f] @ direction[f])
                minimum[f] = d[f]
                axis_after[cols] = stamp
                if episode_reason[f] == 'outside':
                    episode_reason[f] = 'started'
            if not np.isfinite(after[f]):
                continue
            minimum[f] = min(minimum[f], d[f])
            start = next((i for i, h in enumerate(history)
                          if h[0] >= after[f] and stamp-h[0] <= POLICY['history_s']), len(history))
            if start < len(history) and stamp-history[start][0] >= POLICY['minimum_history_s']-1e-9:
                old = history[start]
                delta = relative-old[2]
                own_distance[f] = float(delta[f] @ direction[f])
                progress = positions[start:, f] @ direction[f]
                travel = float(np.abs(np.diff(progress)).sum())
                coherence[f] = own_distance[f]/max(travel, 1e-12)
                closure = old[1][f]-d[f]
                share[f] = own_distance[f]/max(closure/human_width, 1e-12) if closure > 0 else 0.
                lengths = np.linalg.norm(delta, axis=1)
                fraction = lengths/np.maximum(old[4], 1e-12)
                for neighbor in (f-1, f+1):
                    if 0 <= neighbor < 4:
                        alignment = float(delta[f] @ delta[neighbor])/max(lengths[f]*lengths[neighbor], 1e-12)
                        if fraction[f] < POLICY['neighbor_motion_ratio']*fraction[neighbor] and alignment >= POLICY['neighbor_alignment']:
                            coupled[f] = True
                approach[f] = (own_distance[f] >= POLICY['own_approach_width'] and closure >= POLICY['approach_m']
                               and coherence[f] >= POLICY['approach_coherence'] and share[f] >= POLICY['own_closure_share']
                               and not coupled[f])
                if coupled[f] and not active[f]:
                    until[f] = 0.
                if approach[f] and stamp-progress_at[f] < POLICY['progress_window_s']:
                    until[f] = stamp+POLICY['evidence_lifetime_s']
            # Per-axis proof is measured inside this episode, never imported from
            # the longer standalone participation history. After confirmation,
            # ungranted axes start a new baseline and need their own new motion.
            for c, k in enumerate(cols):
                samples = [h for h in history if h[0] >= axis_after[k]]
                if len(samples) < AXIS_POLICY['minimum_samples'] or stamp-samples[0][0] < AXIS_POLICY['minimum_span_s']-1e-9:
                    continue
                angles = np.array([h[5][k] for h in samples])
                net = float(angles[-1]-angles[0]); travel = float(np.abs(np.diff(angles)).sum())
                advance = float((relative[f]-samples[0][2][f]) @ direction[f])
                threshold = np.deg2rad(AXIS_POLICY['flex_motion_deg' if c == 0 else 'side_motion_deg'])
                if abs(net) >= threshold and abs(net)/max(travel, 1e-12) >= AXIS_POLICY['coherence'] and advance >= AXIS_POLICY['approach_width']:
                    axis_until[k] = stamp+AXIS_POLICY['evidence_lifetime_s']
                    axis_source[k] = stamp
                    if active[f]:
                        frozen[k] = True
        evidence = (until >= stamp) & np.isfinite(after) & ~blocked

        # A clearly isolated pair may have nonzero landmark spacing at physical
        # contact. A nearest rank alone never establishes contact on an open hand.
        order = np.argsort(d)
        first, second = int(order[0]), int(order[1])
        isolated = np.zeros(4, bool)
        isolated[first] = (d[second]-d[first] >= POLICY["separation_m"]
                           and d[first] <= POLICY["separation_ratio"]*d[second])
        dominated = np.full(4, bool(isolated[first]))
        dominated[first] = False
        close = d < POLICY["close_m"]
        # A secondary pair needs independent approach evidence when another
        # pair clearly dominates. Non-dominant close pairs can coexist.
        proximity = ((close & (~dominated | approach))
                 | (isolated & (d < POLICY["isolated_enter_m"]))
                 | (approach & (d < POLICY["multi_enter_m"])))
        entry = proximity & evidence & ~blocked
        duration = np.where(close, POLICY["close_confirm_s"], POLICY["relative_confirm_s"])
        reason = []
        for f in range(4):
            if active[f]:
                minimum[f] = min(minimum[f], d[f])
                competing_departure = dominated[f] and d[f]-minimum[f] >= POLICY["departure_m"] and not approach[f]
                supported = ((d[f] < POLICY["ordinary_exit_m"] and not competing_departure)
                             or (isolated[f] and d[f] < POLICY["isolated_exit_m"]))
                leaving[f] = 0. if supported else leaving[f]+dt
                if not reset and (d[f] >= POLICY["isolated_exit_m"]
                                  or (not isolated[f] and d[f] >= POLICY["ordinary_exit_m"])
                                  or leaving[f] >= POLICY["exit_confirm_s"]-1e-9):
                    invalidate(f, 'separated')
                reason.append("confirmed" if active[f] else "separated")
            else:
                entering[f] = entering[f]+dt if entry[f] else 0.
                if entering[f] >= duration[f]-1e-9:
                    active[f] = True
                    confirmed_at[f] = stamp
                    minimum[f] = d[f]
                    leaving[f] = 0.
                    cols = [FLEX[f], SIDE[f]] if HAS_SIDE[f] else [FLEX[f]]
                    for k in cols:
                        frozen[k] = axis_until[k] >= stamp and axis_source[k] >= after[f]
                        if not frozen[k]:
                            axis_after[k] = stamp
                            axis_until[k] = axis_source[k] = 0.
                reason.append("confirmed" if active[f] else "candidate" if entry[f]
                              else "proximity_without_own_approach" if proximity[f] else "unconfirmed")
        projection += np.clip(active.astype(float)-projection,
                              -dt/POLICY["projection_s"], dt/POLICY["projection_s"])
        state = State(stamp, active, projection, entering, leaving, minimum, history, until, after,
                      ids, progress_at, progress_value, direction, confirmed_at, axis_until, axis_source, axis_after, frozen)
        axis_matches = np.zeros(10, bool)
        axis_episodes = np.zeros(10, dtype=np.int64)
        for f in range(4):
            cols = [FLEX[f], SIDE[f]] if HAS_SIDE[f] else [FLEX[f]]
            axis_matches[cols] = (axis_source[cols] >= after[f]) & (axis_source[cols] > 0)
            axis_episodes[cols] = np.where(axis_matches[cols], ids[f], 0)
        return state, dict(algorithm=POLICY["algorithm"], reason=reason,
                           episode_id=ids.tolist(), episode_start_s=[float(v) if np.isfinite(v) else None for v in after],
                           episode_reason=episode_reason, confirmed_at_s=confirmed_at.tolist(),
                           axis_evidence_source_s=axis_source.tolist(), frozen_axes=frozen.tolist(),
                           axis_evidence_episode_id=axis_episodes.tolist(), evidence_matches_episode=axis_matches.tolist(),
                           isolated_pair=isolated.tolist(), independent_approach=approach.tolist(),
                           proximity_evidence=proximity.tolist(),
                           own_approach_width=own_distance.tolist(), approach_coherence=coherence.tolist(),
                           own_closure_share=share.tolist(),
                           neighbor_coupled_motion=coupled.tolist(),
                           approach_evidence_remaining_s=np.maximum(until-stamp, 0.).tolist(),
                           entry_evidence=entry.tolist(), entering_s=entering.tolist(),
                           leaving_s=leaving.tolist(), closest_gap_mm=float(d[first]*1000),
                           second_gap_mm=float(d[second]*1000), clock_reset=reset)

    def commit(self, state):
        self.state = state
