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


@pytest.mark.parametrize('change', ['none', 'stale', 'hand', 'provider', 'margin', 'pose', 'ended'])
def test_stationary_preload_checks_existing_full_hand_proof_without_rebuilding(change):
    g, state, _ = entry_guard()
    lo, hi = np.full(14, -.01), np.full(14, .01)
    assert g(lo, hi)
    g._clearance = Mock(side_effect=AssertionError('distance during preload'))
    g._build_fingers = Mock(side_effect=AssertionError('mesh during preload'))
    g._refresh_hand = Mock(side_effect=AssertionError('refresh during preload'))
    q = np.zeros(14)
    if change == 'stale': state.valid = False
    if change == 'hand': state.intervals = {'thumb': (-.2, .2)}
    if change == 'provider': g.hand_state_provider = lambda _: state
    if change == 'margin': g.margin += .001
    if change == 'pose': q[0] = .011
    if change == 'ended': g.end_entry_certificates()
    assert g.check_cached_coverage(q, q) == (change == 'none')
    assert not g._entry_coverage_only  # Return planning is not frozen by this check.
    g._clearance.assert_not_called()
    g._build_fingers.assert_not_called()
    g._refresh_hand.assert_not_called()


@pytest.mark.parametrize('side', ['left', 'right'])
def test_recovery_preload_uses_scalar_full_hand_proof_after_slow_geometry(side):
    import time
    d, _, _ = make_driver(side, initial=np.ones(7)*.03)
    g, _, _ = entry_guard()
    d._collision_guard = g
    d._enabled = False
    enabled = {'value': False}
    d._arm.get_joint_enable_states.side_effect = lambda: [enabled['value']]*7
    d._configure_arm_collision_safety = Mock(return_value=True)
    d._capture_arm_torque_baseline = Mock(return_value=True)

    def slow_distance(*_, **__):
        d._return_feedback_received = time.monotonic() - .362
        g._box_certified = True
        return 1.

    def enable(**_):
        enabled['value'] = True
        return True

    g._clearance.side_effect = slow_distance
    d._arm.enable.side_effect = enable
    assert d._enable_for_return()
    assert g.last_diagnostics['entry_coverage_only']
    assert g.last_diagnostics['checked_boxes'] == 0
    d._arm.move_j.assert_called_once()


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('preparation', [False, True])
@pytest.mark.parametrize('initial_offset', [.0005, .04])
def test_return_and_preparation_keep_collision_profile_through_execution(side, preparation, initial_offset):
    import time
    from types import SimpleNamespace
    from unittest import mock
    from esrobo_link.backends import HardwareBackend
    d, _, _ = make_driver(side, initial=np.ones(7)*initial_offset)
    g, _, _ = entry_guard()
    d._collision_guard = g
    d._enabled = False
    enabled = {'value': False}
    d._arm.get_joint_enable_states.side_effect = lambda: [enabled['value']]*7
    d._configure_arm_collision_safety = Mock(return_value=True)
    d._capture_arm_torque_baseline = Mock(return_value=True)

    def distance(*_, **kwargs):
        assert kwargs.get('include_fingers') is not preparation
        assert not enabled['value'], 'heavy geometry after enable'
        d._return_feedback_received = time.monotonic() - .362
        g._box_certified = True
        return 1.

    def enable(**_):
        enabled['value'] = True
        return True

    def disable():
        enabled['value'] = False
        return True

    g._clearance.side_effect = distance
    d._arm.enable.side_effect = enable
    d._arm.disable.side_effect = disable
    with mock.patch('esrobo_teleop.robot.nero_driver.time.sleep'):
        if preparation:
            b = HardwareBackend.__new__(HardwareBackend)
            b.side, b.arm, b.hand_only, b.hand = side, d, False, Mock()
            b.cfg = SimpleNamespace(hand=SimpleNamespace(max_stale_time_s=.15))
            b.hand.feedback_snapshot.return_value = dict(position_unit=[128.]*10, age_s=.01)
            result = b.prepare_session(lambda: False)
            assert result['returned'] and result['disabled'] and result['hands_stationary'], result
            b.hand.align_and_verify_open_pose.assert_not_called()
        else:
            result = d.safe_return(recover=True, recognize_inherited_stop=True)
            assert result.returned and result.disabled, result
    assert not enabled['value']
    assert g._clearance.call_count > 0
    if preparation or initial_offset > .01:
        assert d._arm.move_j.call_count > 1
    else:
        d._arm.enable.assert_not_called()


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


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('failure', [None, 'stale', 'missing', 'moving', 'drift',
                                     'candidate', 'controller', 'torque', 'disabled',
                                     'cancel', 'hand', 'provider'])
def test_ready_activation_consumes_proof_without_geometry_or_command_reset(side, failure):
    import time
    from esrobo_teleop.robot.nero_driver import JointFeedbackSnapshot
    d, feedback, zero = make_driver(side)
    d._trajectory.reset(zero, 10.)
    g, hand, _ = entry_guard()
    assert g(np.full(14, -.0035), np.full(14, .0035))
    assert g.freeze_entry_certificates()
    d._collision_guard = g
    d._activation_candidate = zero.copy()
    g._clearance = Mock(side_effect=AssertionError('geometry during ready handshake'))
    g._build_fingers = Mock(side_effect=AssertionError('mesh during ready handshake'))
    sample = JointFeedbackSnapshot(feedback, 10.02, time.monotonic(), 'test')
    def read():
        d._cycle_feedback = sample
        return None if failure == 'missing' else np.zeros(14)
    d.read_control_cycle_joints = Mock(side_effect=read)
    candidate = zero.copy()
    if failure == 'stale':
        sample = JointFeedbackSnapshot(feedback, 10.02, time.monotonic()-.2, 'test')
    if failure == 'moving': d._trajectory.state.velocity[0] = .01
    if failure == 'drift':
        feedback[0] += np.deg2rad(.3)
        sample = JointFeedbackSnapshot(feedback, 10.02, time.monotonic(), 'test')
    if failure == 'candidate': candidate[0] += .001
    if failure == 'controller': d._controller_is_safe = Mock(return_value=False)
    if failure == 'torque': d._torque_is_safe = Mock(return_value=False)
    if failure == 'disabled': d._arm.get_joint_enable_states.return_value = [False]*7
    if failure == 'cancel': d._return_cancel.set()
    if failure == 'hand': hand.intervals = {'thumb': (-.3, .3)}
    if failure == 'provider': g.hand_state_provider = lambda _: hand
    before = d._trajectory.state
    assert d.verify_tracking_activation(zero, candidate) == (failure is None)
    assert d._trajectory.state is before
    assert d._activation_candidate is None
    assert not d.verify_tracking_activation(zero, candidate)  # no replay of a consumed ticket
    g._clearance.assert_not_called()
    g._build_fingers.assert_not_called()
    d._arm.move_j.assert_not_called()
