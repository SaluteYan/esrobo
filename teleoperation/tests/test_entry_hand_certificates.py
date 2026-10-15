"""Initial approach excludes fingers; live hand safety remains unchanged."""
from types import SimpleNamespace
from unittest.mock import Mock
import numpy as np
from esrobo_teleop.robot.torso_collision import TorsoCollisionGuard
from esrobo_teleop.robot.hand_geometry import geometry_state
from test_entry_execution import guard


def entry_guard():
    # General full-hand certificate fixture for live/return safety regressions.
    g = guard()
    state = SimpleNamespace(valid=True, reason='', stamp=10., calibrated=('thumb',),
                            intervals={'thumb': (-.15, .15)})
    g.hand_state_provider = lambda horizon: state
    g._build_fingers = Mock(return_value=[])
    g._refresh_hand = TorsoCollisionGuard._refresh_hand.__get__(g)
    g.begin_entry_certificates()
    return g, state, state


def test_live_hand_changes_require_new_geometry_and_stale_clears():
    g, state, _ = entry_guard()
    q = np.zeros(14)
    assert g(q, q)
    state.intervals = {'thumb': (.3, .4)}
    assert g(q, q)
    assert not g.last_diagnostics['entry_cache_hit']
    assert g._clearance.call_count == 2
    state.valid = False
    assert not g(q, q)
    assert g.last_diagnostics['category'] == 'hand_geometry_unavailable'
    assert not g._entry_certificates


def test_hand_return_checks_full_envelope_without_replacing_feedback_provider():
    g, measured, _ = entry_guard()
    g.end_entry_certificates()
    provider = g.hand_state_provider
    planned = SimpleNamespace(valid=True, reason='', stamp=10., calibrated=('thumb',),
                              intervals={'thumb': (-.5, .5)})
    q = np.zeros(14)
    assert g.check_hand_return(q-.001, q+.001, planned)
    assert dict(g._hand_interval_key)['thumb'] == (-.5, .5)
    assert g.hand_state_provider is provider
    assert g(q, q)
    assert dict(g._hand_interval_key)['thumb'] == (-.16, .16)
    planned.valid = False
    assert not g.check_hand_return(q, q, planned)


def test_hand_return_cannot_reuse_entry_arm_only_profile():
    g, state, _ = entry_guard()
    q = np.zeros(14)
    assert not g.check_hand_return(q, q, state)
    g.end_entry_certificates()
    g._entry_arm_geometry_only = True
    assert not g.check_hand_return(q, q, state)
    g._entry_arm_geometry_only = False
    g.include_fingers = False
    assert not g.check_hand_return(q, q, state)


def test_arm_certificate_never_reads_hand_and_ignores_hand_context_changes():
    g, state, _ = entry_guard()
    g.hand_state_provider = Mock(side_effect=AssertionError('hand during arm-only planning'))
    g._build_fingers = Mock(side_effect=AssertionError('finger mesh during entry'))
    lo, hi = np.full(14, -.01), np.full(14, .01)
    assert g.check_arm_only(lo, hi)
    assert g.freeze_entry_certificates(include_fingers=False)
    g._hand_interval_key = ('different live envelope',)
    g.hand_motion_horizon_s += .1
    g.thumb_envelope = 'different live shape'
    g.hand_state_provider = Mock(side_effect=AssertionError('hand during entry execution'))
    assert g.check_entry_coverage(lo, hi, include_fingers=False)
    assert not g.check_entry_coverage(lo, hi+1e-12, include_fingers=False)
    assert g.last_diagnostics['category'] == 'entry_certificate_miss'
    g.hand_state_provider.assert_not_called()
    g._build_fingers.assert_not_called()


def test_real_age_and_future_horizon_remain_in_live_geometry():
    calibration = {'right': {'thumb': dict(raw=[0,255], rad=[0.,1.], error_rad=.05,
                                          max_velocity_rad_s=1., verified=True)}}
    for age in (.002, .03, .099):
        state = geometry_state([128],10.,'right',calibration,['thumb'],.5,.1,now=10.+age)
        assert state.valid and state.stamp == 10.
        radius = .05+age+.5
        np.testing.assert_allclose(state.intervals['right_thumb'], [128/255-radius,128/255+radius])
    assert not geometry_state([128],10.,'right',calibration,['thumb'],0.,.1,now=10.101).valid


def test_changed_entry_arm_pair_set_invalidates_frozen_certificates():
    g = guard()
    g.begin_entry_certificates(arm_geometry_only=True)
    q = np.zeros(14)
    assert g.check_arm_only(q, q)
    assert g.freeze_entry_certificates(include_fingers=False)
    g._entry_arm_pairs = ((0, 1),)
    assert not g.check_entry_coverage(q, q, include_fingers=False)
    assert g.last_diagnostics['category'] == 'entry_certificate_invalid'
    g.end_entry_certificates()
    assert not g._entry_arm_geometry_only


def test_padded_path_proof_is_checked_before_enable_and_cancel_clears(monkeypatch):
    from test_safe_return import make_driver
    from esrobo_teleop.robot.return_planner import ReturnPlan
    from esrobo_teleop.robot.handover_planner import HandoverPlanner
    from esrobo_teleop.robot import nero_driver
    d, feedback, zero = make_driver('right')
    d._enabled = False
    d._arm.get_joint_enable_states.return_value = [False]*7
    g = guard()
    d._collision_guard = g
    requests = []

    def check(a, b):
        assert not d._enabled
        x, y = np.zeros(14), np.zeros(14)
        x[7:], y[7:] = a-zero, b-zero
        requests.append((x.copy(), y.copy()))
        return g.check_arm_only(x, y)

    d._check_physical_path = check
    goal = zero.copy()
    goal[0] += .01
    monkeypatch.setattr(HandoverPlanner, 'plan', lambda *args:
        ReturnPlan(path=(zero.copy(), goal.copy()), seed=0, nodes=2, reason='test'))
    monkeypatch.setattr(nero_driver.time, 'sleep', lambda _: None)

    def planned():
        assert g._entry_certificates
        lo, hi = requests[-1]
        assert hi[7]-lo[7] > .01  # New box, not the original narrow edge.
        assert g.check_entry_coverage(lo, hi, include_fingers=False) and g.last_diagnostics['entry_cache_hit']
        d._arm.enable.assert_not_called()
        d._return_cancel.set()

    result = d.safe_return(recover=True, handover_goal=np.array([.01,0,0,0,0,0,0]),
                           on_approach=planned)
    assert not result.returned and result.reason == 'handover input changed before enable'
    d._arm.enable.assert_not_called()
    d._arm.move_j.assert_not_called()
    assert g._entry_certificates is None


def test_planning_age_jitter_keeps_conservative_mesh_but_expansion_recertifies():
    g, state, _ = entry_guard()
    q = np.zeros(14)
    state.intervals = {'thumb': (-.09, .09)}
    assert g(q, q)
    for radius in (.07, .08, .09, .07):
        state.intervals = {'thumb': (-radius, radius)}
        assert g(q, q)
        assert g.last_diagnostics['entry_cache_hit']
    assert g._build_fingers.call_count == g._clearance.call_count == 1
    assert dict(g._hand_interval_key) == {'thumb': (-.1, .1)}
    # A new larger/asymmetric uncertainty must never reuse a smaller proof.
    state.intervals = {'thumb': (-.07, .13)}
    assert g(q, q)
    assert not g.last_diagnostics['entry_cache_hit']
    assert g._build_fingers.call_count == g._clearance.call_count == 2
    assert dict(g._hand_interval_key) == {'thumb': (-.1, .14)}
    assert g.freeze_entry_certificates()
    state.intervals = {'thumb': (-.07, .15)}
    assert not g.check_entry_coverage(q, q)
    assert g._build_fingers.call_count == g._clearance.call_count == 2


def test_new_planning_session_and_live_checks_do_not_accumulate_old_envelopes():
    g, state, _ = entry_guard()
    q = np.zeros(14)
    assert g(q, q)
    g.end_entry_certificates()
    state.intervals = {'thumb': (-.07, .07)}
    g.begin_entry_certificates()
    assert g(q, q)
    assert dict(g._hand_interval_key) == {'thumb': (-.08, .08)}
    g.end_entry_certificates()
    state.intervals = {'thumb': (-.03, .03)}
    assert g(q, q)
    assert dict(g._hand_interval_key) == {'thumb': (-.04, .04)}


def test_planning_covers_whole_valid_feedback_age_without_accepting_stale_data():
    calibration = {'left': {'thumb': dict(raw=[0,255], rad=[0.,1.], error_rad=.05,
                                        max_velocity_rad_s=1., verified=True)}}
    plan = geometry_state([128], 10., 'left', calibration, ['thumb'], 0., .1,
                          now=10.001, cover_valid_age=True)
    assert plan.valid and plan.stamp == 10.
    for age in (.001, .02, .05, .099):
        actual = geometry_state([128], 10., 'left', calibration, ['thumb'], 0., .1, now=10.+age)
        assert actual.valid
        a,b = actual.intervals['left_thumb']; lo,hi = plan.intervals['left_thumb']
        assert lo <= a <= b <= hi
    stale = geometry_state([128], 10., 'left', calibration, ['thumb'], 0., .1,
                           now=10.101, cover_valid_age=True)
    assert not stale.valid
