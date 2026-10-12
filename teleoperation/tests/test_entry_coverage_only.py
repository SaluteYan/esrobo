"""Frozen entry proofs: no SDK, CAN, motion or runtime performance sampling."""
import numpy as np
import pytest
from unittest.mock import Mock

from test_entry_execution import guard, run_entry
from test_entry_hand_certificates import entry_guard
from test_safe_return import make_driver


def frozen_hand_guard():
    g, state, bound = entry_guard()
    assert g(np.zeros(14), np.ones(14))
    assert g.freeze_entry_certificates()
    g._clearance = Mock(side_effect=AssertionError('distance during execution'))
    g._build_fingers = Mock(side_effect=AssertionError('mesh during execution'))
    g._refresh_hand = Mock(side_effect=AssertionError('refresh during execution'))
    return g, state


def test_frozen_checks_cover_whole_interval_without_distance_or_mesh_work():
    g, state = frozen_hand_guard()
    for interval in ((-.06, .06), (-.15, .15)):
        state.intervals = {'thumb': interval}
        assert g.check_entry_coverage(np.zeros(14), np.ones(14))
        assert g.last_diagnostics['checked_boxes'] == 0
    for check in (g, g.check_entry_coverage):
        assert not check(np.zeros(14), np.ones(14)+1e-12)
        assert g.last_diagnostics['category'] == 'entry_certificate_miss'
    # A rejected large candidate does not destroy a valid braking certificate.
    assert g.check_entry_coverage(np.full(14, .2), np.full(14, .8))
    g._clearance.assert_not_called()
    g._build_fingers.assert_not_called()
    g._refresh_hand.assert_not_called()


@pytest.mark.parametrize('change', ['stale', 'hand', 'missing_joint', 'nan',
    'margin', 'shoulder', 'pair_margin', 'model', 'geometry', 'horizon', 'provider', 'mode'])
def test_changed_context_or_hand_fails_closed_and_cannot_rebuild(change):
    g, state = frozen_hand_guard()
    if change == 'stale': state.valid = False
    if change == 'hand':
        lo, hi = dict(g._hand_interval_key)['thumb']
        state.intervals = {'thumb': (lo-1e-12, hi)}
    if change == 'missing_joint': state.intervals = {}
    if change == 'nan': state.intervals = {'thumb': (float('nan'), .1)}
    if change == 'margin': g.margin += .001
    if change == 'shoulder': g.shoulder_margin += .001
    if change == 'pair_margin': g.pair_margins[(0, 1)] = .04
    if change == 'model': g.model = object()
    if change == 'geometry': g.geometry = object()
    if change == 'horizon': g.hand_motion_horizon_s += .1
    if change == 'provider': g.hand_state_provider = lambda _: state
    assert not g.check_entry_coverage(np.zeros(14), np.zeros(14), include_fingers=change != 'mode')
    assert not g._entry_certificates
    state.valid = True
    state.intervals = {'thumb': (-.05, .05)}
    assert not g(np.zeros(14), np.zeros(14))
    assert g._entry_coverage_only
    g._clearance.assert_not_called()
    g._build_fingers.assert_not_called()


def test_cannot_join_certificates_or_rebuild_after_feedback_invalidation():
    g = guard(); g.begin_entry_certificates()
    assert g(np.zeros(14), np.ones(14))
    assert g(np.ones(14), np.ones(14)*2)
    assert g.freeze_entry_certificates()
    g._clearance.side_effect = AssertionError('unexpected geometry')
    assert not g.check_entry_coverage(np.zeros(14), np.ones(14)*2)
    g.invalidate_entry_certificates()
    assert not g(np.zeros(14), np.zeros(14))
    g.end_entry_certificates()
    assert not g.check_entry_coverage(np.zeros(14), np.zeros(14))


def test_empty_freeze_never_falls_back_to_full_geometry():
    g = guard(); g.begin_entry_certificates()
    assert not g.freeze_entry_certificates()
    g._clearance.side_effect = AssertionError('unexpected geometry')
    assert not g(np.zeros(14), np.zeros(14))


@pytest.mark.parametrize('side', ['left', 'right'])
def test_prepare_corridor_and_turn_before_execution(side):
    d, _, zero = make_driver(side)
    g = guard(); d._collision_guard = g
    d._entry_arm_only_collision = True
    g.begin_entry_certificates(arm_geometry_only=True)
    middle = zero.copy(); middle[0] += .1
    goal = middle.copy(); goal[1] += .1
    lower, upper = d._effective_position_limits(side)
    assert d._prepare_entry_coverage([zero, middle, goal], lower, upper, g) is None
    g._clearance.side_effect = AssertionError('geometry after preparation')
    assert d._check_physical_path(zero, goal, entry_coverage_only=True)
    d._arm.enable.assert_not_called()
    d._arm.move_j.assert_not_called()


def test_unprovable_corridor_refuses_before_motion():
    d, _, zero = make_driver('right')
    g = guard(); d._collision_guard = g
    # Match the production initial-approach entry; ordinary hand/return
    # certificates intentionally use a different collision context.
    d._entry_arm_only_collision = True
    g.begin_entry_certificates(arm_geometry_only=True)
    lower, upper = d._effective_position_limits('right')
    g._clearance.side_effect = lambda *a, **kw: 0.
    assert 'not certified' in d._prepare_entry_coverage([zero, zero+.01], lower, upper, g)
    d._arm.enable.assert_not_called(); d._arm.move_j.assert_not_called()


@pytest.mark.parametrize('side', ['left', 'right'])
def test_long_subdivided_path_is_covered_without_waypoint_limit_or_path_change(side):
    d, _, zero = make_driver(side)
    g = guard(); d._collision_guard = g
    # Match the production initial-approach entry; ordinary hand/return
    # certificates intentionally use a different collision context.
    d._entry_arm_only_collision = True
    g.begin_entry_certificates(arm_geometry_only=True)
    goal = zero.copy(); goal[0] += .3
    path = np.array([zero+(goal-zero)*i/96 for i in range(97)])
    original = path.copy()
    lower, upper = d._effective_position_limits(side)
    assert d._prepare_entry_coverage(path, lower, upper, g) is None
    assert len(g._entry_certificates) == 1
    np.testing.assert_array_equal(path, original)
    g._clearance.side_effect = AssertionError('geometry during execution')
    for a, b in zip(path, path[1:]):
        assert d._check_physical_path(a, b, entry_coverage_only=True)
    d._arm.enable.assert_not_called(); d._arm.move_j.assert_not_called()


def test_group_proof_includes_interior_excursion_when_endpoints_coincide():
    d, _, zero = make_driver('right')
    g = guard(); d._collision_guard = g
    # Match the production initial-approach entry; ordinary hand/return
    # certificates intentionally use a different collision context.
    d._entry_arm_only_collision = True
    g.begin_entry_certificates(arm_geometry_only=True)
    excursion = zero.copy(); excursion[0] += .2
    path = [zero, excursion, zero.copy()]
    lower, upper = d._effective_position_limits('right')
    assert d._prepare_entry_coverage(path, lower, upper, g) is None
    assert d._check_physical_path(excursion, excursion, entry_coverage_only=True)
    # The certified box must contain the interior point, even though both
    # endpoints are identical. A point/endpoints-only proof would fail this.
    _, hi, _ = g._entry_certificates[0]
    assert hi[7] >= .2


def test_unsafe_group_is_subdivided_and_requires_new_complete_proofs():
    d, _, zero = make_driver('right')
    g = guard(); d._collision_guard = g
    # Match the production initial-approach entry; ordinary hand/return
    # certificates intentionally use a different collision context.
    d._entry_arm_only_collision = True
    g.begin_entry_certificates(arm_geometry_only=True)
    original = g._clearance.side_effect
    def clearance(center, width, **kwargs):
        if width[7] > .025:
            g.last_diagnostics = dict(clearance_m=0., required_m=.03)
            return 0.
        return original(center, width, **kwargs)
    g._clearance.side_effect = clearance
    goal = zero.copy(); goal[0] += .3
    path = [zero+(goal-zero)*i/96 for i in range(97)]
    lower, upper = d._effective_position_limits('right')
    assert d._prepare_entry_coverage(path, lower, upper, g) is None
    assert 1 < len(g._entry_certificates) <= 32
    assert not d._check_physical_path(zero, goal, entry_coverage_only=True)
    for a, b in zip(path, path[1:]):
        assert d._check_physical_path(a, b, entry_coverage_only=True)


def test_genuinely_more_than_32_proofs_refuses_instead_of_evicting_required_proofs():
    d, _, zero = make_driver('right')
    g = guard(); d._collision_guard = g
    # Match the production initial-approach entry; ordinary hand/return
    # certificates intentionally use a different collision context.
    d._entry_arm_only_collision = True
    g.begin_entry_certificates(arm_geometry_only=True)
    original = g._clearance.side_effect
    def clearance(center, width, **kwargs):
        if width[7] > .014:
            g.last_diagnostics = dict(clearance_m=0., required_m=.03)
            return 0.
        return original(center, width, **kwargs)
    g._clearance.side_effect = clearance
    # Only one padded edge can fit a proof. All 40 distinct edges are needed.
    path = [zero + np.array([i*.02,0,0,0,0,0,0]) for i in range(41)]
    lower, upper = d._effective_position_limits('right')
    reason = d._prepare_entry_coverage(path, lower, upper, g)
    assert '32 certified corridors' in reason
    assert len(g._entry_certificates) == 32
    assert not g._entry_coverage_only
    d._arm.enable.assert_not_called(); d._arm.move_j.assert_not_called()


def test_planning_invalidation_recertifies_required_boxes_before_freezing():
    d, _, zero = make_driver('right')
    g = guard(); d._collision_guard = g
    # Match the production initial-approach entry; ordinary hand/return
    # certificates intentionally use a different collision context.
    d._entry_arm_only_collision = True
    g.begin_entry_certificates(arm_geometry_only=True)
    original_clearance = g._clearance.side_effect
    def clearance(center, width, **kwargs):
        if width[7] > .025:
            g.last_diagnostics = dict(clearance_m=0., required_m=.03)
            return 0.
        return original_clearance(center, width, **kwargs)
    g._clearance.side_effect = clearance
    original_check = d._check_physical_path
    invalidated = []
    def check(a, b, **kwargs):
        ok = original_check(a, b, **kwargs)
        if not ok and g._entry_certificates and not invalidated:
            g.invalidate_entry_certificates()
            invalidated.append(True)
        return ok
    d._check_physical_path = check
    path = [zero+np.array([.3*i/96,0,0,0,0,0,0]) for i in range(97)]
    lower, upper = d._effective_position_limits('right')
    assert d._prepare_entry_coverage(path, lower, upper, g) is None
    assert invalidated
    g._clearance.side_effect = AssertionError('geometry after freezing')
    assert all(original_check(a,b,entry_coverage_only=True) for a,b in zip(path,path[1:]))
    d._arm.enable.assert_not_called(); d._arm.move_j.assert_not_called()


def test_unstable_planning_context_refuses_without_motion():
    d, _, zero = make_driver('right')
    g = guard(); d._collision_guard = g
    # Match the production initial-approach entry; ordinary hand/return
    # certificates intentionally use a different collision context.
    d._entry_arm_only_collision = True
    g.begin_entry_certificates(arm_geometry_only=True)
    original_check = d._check_physical_path
    def check(a, b, **kwargs):
        ok = original_check(a, b, **kwargs)
        g._entry_certificate_context = object()
        return ok
    d._check_physical_path = check
    lower, upper = d._effective_position_limits('right')
    assert 'kept changing' in d._prepare_entry_coverage([zero,zero+.01],lower,upper,g)
    assert not g._entry_coverage_only
    d._arm.enable.assert_not_called(); d._arm.move_j.assert_not_called()


def test_explicit_entry_without_planning_cannot_send_commands():
    d, _, zero = make_driver('right')
    d._collision_guard = guard()
    assert not d._return_group_to_zero(np.zeros(14), np.arange(7), zero,
                                      .03, 1., entry_approach=True)
    assert 'no frozen' in d._return_execution_failure
    d._arm.move_j.assert_not_called()


@pytest.mark.parametrize('side', ['left', 'right'])
def test_entry_search_and_final_certification_both_exclude_fingers(side, monkeypatch):
    from esrobo_teleop.robot.handover_planner import HandoverPlanner
    from esrobo_teleop.robot.return_planner import ReturnPlan
    from esrobo_teleop.robot import nero_driver
    d, _, zero = make_driver(side)
    g = guard(); d._collision_guard = g
    d._enabled = False
    d._arm.get_joint_enable_states.return_value = [False]*7
    joint = 0 if side == 'left' else 7
    def clearance(q, width, *, include_fingers, **kwargs):
        g._box_certified = True
        ok = not include_fingers or abs(q[joint]) < .005
        g.last_diagnostics = dict(clearance_m=.1 if ok else 0., required_m=.03)
        return .1 if ok else 0.
    g._clearance.side_effect = clearance
    observed = []
    def plan(planner, start, goal):
        assert planner.check(start, goal)  # Search ignores fingers only.
        observed.append(True)
        return ReturnPlan((start.copy(), goal.copy()), 0, 2)
    monkeypatch.setattr(HandoverPlanner, 'plan', plan)
    monkeypatch.setattr(nero_driver.time, 'sleep', lambda _: None)
    target = np.array([.04,0,0,0,0,0,0])
    def certified():
        assert g._entry_coverage_only
        assert g._entry_certificate_context == g._certificate_context(False)
        assert all(not c.kwargs['include_fingers'] for c in g._clearance.call_args_list)
        d._return_cancel.set()
    result = d.safe_return(recover=True, handover_goal=target,
                           stationary_hand_check=lambda: True, on_approach=certified)
    assert observed and not result.returned and result.stage == 'planning'
    assert result.reason == 'handover input changed before enable'
    d._arm.enable.assert_not_called(); d._arm.move_j.assert_not_called()


def test_search_checks_still_require_stationary_hand_and_cannot_be_execution_checks():
    d, _, zero = make_driver('right')
    g = guard(); d._collision_guard = g
    d._entry_hand_stationary_check = lambda: False
    assert not d._check_physical_path(zero,zero,entry_search_only=True)
    g._clearance.assert_not_called()
    with pytest.raises(ValueError, match='cannot authorize execution'):
        d._check_physical_path(zero,zero,entry_search_only=True,entry_coverage_only=True)


def test_execution_uses_frozen_proofs_for_full_command_and_braking_boxes():
    # run_entry replaces the distance backend with an exception AFTER planning.
    d, ok, calls, commits, _ = run_entry([.002, .008, .015])
    assert ok and len(commits) > 10
    assert all(s.get('cache_hits', 0) > 0 for s in d._entry_diagnostics.samples if s['sent'])
    assert not d._collision_guard._entry_certificates


def test_one_repeated_frame_waits_without_destroying_proof_or_advancing_time():
    d, ok, calls, commits, _ = run_entry(feedback_mode='one_duplicate')
    assert ok and commits
    assert commits[0].stamp > 10.02
    assert len(calls) == len(commits)+1


def test_missing_feedback_cannot_rebuild_certificate_on_next_fresh_frame():
    d, ok, calls, commits, _ = run_entry(feedback_mode='one_missing')
    assert not ok and len(calls) == 1 and not commits
    assert 'entry_certificate_invalid' in d._return_execution_failure


def test_disabling_collision_flag_cannot_bypass_explicit_entry_query():
    d, _, zero = make_driver('right')
    d._cfg.torso_collision_enabled = False
    assert not d._check_physical_path(zero, zero, entry_coverage_only=True)


def test_certificate_invalidated_during_deadline_wait_prevents_send(monkeypatch):
    import test_entry_execution as fixture
    from esrobo_teleop.robot.entry_execution import EntryCadence
    guards = []
    def make_guard():
        g = guard(); guards.append(g)
        return g
    original = EntryCadence.can_send
    def wait_then_invalidate(cadence):
        result = original(cadence)
        guards[0].invalidate_entry_certificates()
        return result
    monkeypatch.setattr(fixture, 'guard', make_guard)
    monkeypatch.setattr(EntryCadence, 'can_send', wait_then_invalidate)
    d, ok, calls, commits, _ = fixture.run_entry()
    assert not ok and not calls and not commits
    assert d._entry_diagnostics.samples[-1]['reason'] == 'entry_certificate_rejected_before_send'
