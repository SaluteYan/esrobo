"""Offline URDF fitting, including thumb rotation at flexion boundaries."""
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from esrobo_laptop.config import ROOT, read_robot_config, read_settings
from esrobo_laptop.hand_retarget import HandRetargeter, TIPS


@pytest.fixture(params=['native_cpp', 'python_reference'])
def solver(request):
    result = HandRetargeter(read_robot_config(read_settings(ROOT/'config/laptop.yaml')), use_native=request.param=='native_cpp')
    assert result.backend == request.param
    try:
        yield result
    finally:
        # Explicit-permission fixtures install a closure referencing the solver.
        # Break it during teardown, not in a later transport timing test's GC.
        if 'propose' in vars(result.participation):
            del result.participation.propose


def test_exact_fk_jacobian_includes_passive_thumb_and_fingers(solver):
    m = solver.model
    q = (m.lower+m.upper)*.43
    _, jac = m.forward(q)
    for k in range(10):
        step = np.zeros(10); step[k] = 1e-6
        plus, _ = m.forward(q+step); minus, _ = m.forward(q-step)
        np.testing.assert_allclose((plus-minus)/2e-6, jac[:, :, k], atol=1e-8)


@pytest.mark.parametrize('flexion_boundary', [0., 1.])
@pytest.mark.parametrize('rotation', [.1, .92])
def test_thumb_can_rotate_while_flexion_is_at_either_boundary(solver, flexion_boundary, rotation):
    m = solver.model
    goal = m.neutral.copy()
    goal[:3] = [rotation, .15, flexion_boundary*m.upper[2]]
    target, _ = m.forward(goal)
    # Physical home stays raw 111; the corrected direction changes its model angle.
    assert np.degrees(m.neutral[0]) == pytest.approx(np.degrees((255-111)/255*1.03))
    for _ in range(8):
        result = solver.solve(None, target=target, budget_s=1.)
    fitted = np.array(result['joints'])
    assert abs(fitted[0]-rotation) < np.deg2rad(1.)
    assert result['diagnostics']['tip_error_mm'][0] < 1.
    assert np.all(fitted >= m.lower) and np.all(fitted <= m.upper)


@pytest.mark.parametrize('axis', range(10))
def test_each_actuator_can_follow_reachable_geometry(solver, axis):
    m = solver.model
    goal = (m.lower+m.upper)*.35
    goal[axis] = m.lower[axis]+.7*(m.upper[axis]-m.lower[axis])
    target, _ = m.forward(goal)
    for _ in range(10):
        result = solver.solve(None, target=target, budget_s=1.)
    assert max(result['diagnostics']['tip_error_mm']) < 2.
    fitted = np.array(result['joints'])
    assert np.all(fitted >= m.lower) and np.all(fitted <= m.upper)


def test_human_alignment_is_rigid_pose_invariant(solver):
    from test_pico_hand import skeleton
    poses = skeleton(.4)
    moved = poses.copy()
    moved[:, :3] = poses[:, :3] @ Rotation.from_euler('xyz', [.3, -.6, .8]).as_matrix().T+[.7, .2, -.1]
    np.testing.assert_allclose(solver.model.human_target(poses), solver.model.human_target(moved), atol=1e-12)


def test_invalid_and_timed_out_targets_do_not_commit_solution(solver):
    target, _ = solver.model.forward(solver.previous)
    before = solver.previous.copy()
    with pytest.raises(TimeoutError):
        solver.solve(None, target=target, budget_s=0.)
    np.testing.assert_array_equal(solver.previous, before)
    target[3] = target[2]
    with pytest.raises(ValueError, match='degenerate'):
        solver.solve(None, target=target)
    np.testing.assert_array_equal(solver.previous, before)


def test_unreachable_open_thumb_stays_bounded_but_rotation_can_compensate(solver):
    m = solver.model
    q = m.neutral.copy(); q[:3] = [.9, .1, -.06]  # Beyond allowed extension.
    target, _ = m.forward(q)
    initial, _ = m.forward(m.neutral)
    initial_error = np.linalg.norm(initial[5]-target[5])
    for _ in range(8):
        result = solver.solve(None, target=target, budget_s=1.)
    assert abs(result['joints'][0]-m.neutral[0]) > .1
    assert np.linalg.norm(np.array(result['solved_points'])[5]-target[5]) < initial_error*.3
    assert np.all(np.array(result['joints']) >= m.lower)
    assert np.all(np.array(result['joints']) <= m.upper)


def test_open_rotation_command_and_expected_feedback_share_approved_target():
    from esrobo_teleop.config import HandConfig
    from esrobo_teleop.robot.linker_hand_driver import LinkerHandDriver
    cfg = read_robot_config(read_settings(ROOT/'config/laptop.yaml')).hand
    expected = [255]*6+[128]*3+[111]
    assert cfg.left_open == HandConfig().left_open == expected
    assert cfg.left_open_feedback == expected
    driver = LinkerHandDriver.__new__(LinkerHandDriver)
    driver._cfg = cfg
    driver._physical_count = 10
    driver._active_sides = ('left',)
    np.testing.assert_array_equal(driver.open_feedback_target('left'), expected)


def test_left_rotation_reversal_changes_only_physical_axis_nine():
    from copy import deepcopy
    from esrobo_teleop.robot.hand_control_mapping import calibrated_units
    cfg = read_robot_config(read_settings(ROOT/'config/laptop.yaml'))
    model = HandRetargeter(cfg).model
    original = deepcopy(cfg.hand)
    original.left_control_calibration['thumb_cmc_roll']['raw'] = [0, 255]
    for fraction in (0., .1, .3, .7, .9, 1.):
        q = model.lower + fraction*(model.upper-model.lower)
        before = calibrated_units(q, original)
        after = calibrated_units(q, cfg.hand)
        np.testing.assert_array_equal(after[:9], before[:9])
        # Half-integer rounding can differ by one unit when reflecting 0..255.
        assert abs(int(after[9])-(255-int(before[9]))) <= 1
    low, high = model.neutral.copy(), model.neutral.copy()
    low[0], high[0] = 0., 1.03
    assert calibrated_units(low, cfg.hand)[9] == 255
    assert calibrated_units(high, cfg.hand)[9] == 0
    assert calibrated_units(model.neutral, cfg.hand)[9] == 111


def test_rotation_feedback_matches_left_direction_without_changing_right():
    from esrobo_teleop.robot.hand_geometry import geometry_state
    from esrobo_teleop.robot.linker_hand_driver import L10_PHYSICAL_JOINT_NAMES
    cfg = read_robot_config(read_settings(ROOT/'config/laptop.yaml')).hand
    calibration = cfg.geometry_feedback_calibration
    for value in (0., 111., 255.):
        raw = np.asarray(cfg.left_open_feedback, dtype=float)
        raw[9] = value
        left = geometry_state(raw, 100., 'left', calibration, L10_PHYSICAL_JOINT_NAMES,
                              0., .1, now=100.)
        # Right thumb flexion's calibrated open endpoint is 254, not 255.
        raw[0] = 254.
        right = geometry_state(raw, 100., 'right', calibration, L10_PHYSICAL_JOINT_NAMES,
                               0., .1, now=100.)
        assert left.valid and right.valid
        assert np.mean(left.intervals['left_thumb_cmc_roll']) == pytest.approx((255-value)/255*1.03)
        assert np.mean(right.intervals['right_thumb_cmc_roll']) == pytest.approx(value/255*1.03)
    for name in ('thumb_cmc_pitch', 'thumb_cmc_yaw'):
        assert cfg.left_control_calibration[name]['raw'] == [255, 0]
        assert calibration['left'][name]['raw'] == [255, 0]


def test_batched_four_finger_fk_and_jacobian_match_full_urdf(solver):
    m = solver.model
    for fraction in (.01, .35, .97):
        q = m.lower+(m.upper-m.lower)*fraction
        p, j = m.forward_fingers(solver._finger_angles(q))
        full, full_j = m.forward(q)
        for f, cols in enumerate(([3, 4], [None, 5], [6, 7], [8, 9])):
            np.testing.assert_allclose(p[f], full[7+5*f:11+5*f], atol=1e-12)
            for local, k in enumerate(cols):
                if k is None:
                    np.testing.assert_allclose(j[f, :, :, local], 0., atol=1e-12)
                else:
                    np.testing.assert_allclose(j[f, :, :, local], full_j[7+5*f:11+5*f, :, k], atol=1e-12)


def test_structural_overlay_uses_current_moving_root(solver):
    m = solver.model
    human, _ = m.forward(m.upper*.35)
    shifted = human.copy(); shifted[2:6] += [.01, -.02, .03]
    q = m.neutral.copy(); q[:2] = [.9, .8]
    current, _ = m.forward(q)
    rebuilt = solver.reconstruct_target(shifted, q)
    np.testing.assert_allclose(rebuilt[2], current[2])
    assert np.linalg.norm(rebuilt[2]-solver.rest_points[2]) > .005
    np.testing.assert_allclose(solver._directions(rebuilt[2:6]), solver._directions(human[2:6]))


def test_ordinary_four_fingers_are_direct_and_solver_work_is_bounded(solver):
    target, _ = solver.model.forward(solver.model.neutral)
    r = solver.solve(None, target=target, budget_s=1.)
    d = r['diagnostics']
    assert d['nonlinear_dof'] == 3
    np.testing.assert_array_equal(r['joints'][3:], d['geometric_seed_rad'][3:])
    assert d['updates'] <= 6 and d['step_attempts'] <= 12
    assert d['evaluations'] <= 15


def test_timeout_during_fk_does_not_commit_contact_or_warm_start(solver, monkeypatch):
    target, _ = solver.model.forward(solver.model.upper*.4)
    previous = solver.previous.copy()
    def fail(*args, **kwargs):
        raise TimeoutError('injected computation deadline')
    if solver._native is None:
        monkeypatch.setattr(solver.model, 'forward_fingers', fail)
    else:
        from types import SimpleNamespace
        solver._native = SimpleNamespace(solve=fail, direct_fingers=solver._native.direct_fingers)
    with pytest.raises(TimeoutError):
        solver.solve(None, target=target, budget_s=1.)
    np.testing.assert_array_equal(solver.previous, previous)
    assert not solver.contact.any() and not solver.projection.any()
    assert solver.source_time is None and solver._pending is None


def test_thumb_specialized_fk_matches_full_model_and_all_mimic_derivatives(solver):
    m = solver.model
    for fraction in (.001, .3, .7, .999):
        q = m.lower+(m.upper-m.lower)*fraction
        p, j = m.thumb_landmarks(q[:3])
        full, full_j = m.forward(q)
        np.testing.assert_allclose(p, full[2:6], atol=1e-12)
        np.testing.assert_allclose(j, full_j[2:6, :, :3], atol=1e-12)


def test_natural_open_geometry_retains_approved_rotation(solver):
    m = solver.model
    points, _ = m.forward(m.neutral)
    poses = np.zeros((26, 7)); poses[:, :3] = points; poses[:, 6] = 1.
    for _ in range(3):
        result = solver.solve(poses, budget_s=1.)
    assert abs(result['joints'][0]-m.neutral[0]) < np.deg2rad(.1)
    assert max(result['diagnostics']['structural_tip_error_mm']) < .1


@pytest.mark.parametrize('finger,thumb,flexion', [
    (0, [.459586348, .517399375, .194684846], .746995347),
    (1, [.481157635, .694469637, .260887264], .786287445),
    (2, [.705381261, .700407807, .251176431], .777403083),
    (3, [.840361861, .821692069, .265871624], .806083171),
])
def test_each_reachable_urdf_pinch_preserves_contact(solver, finger, thumb, flexion):
    # Model-generated zero-gap fixtures, not claimed human/hardware calibration.
    m = solver.model
    q = m.upper*.4
    q[:3] = thumb
    q[[4, 5, 7, 9][finger]] = flexion
    target, _ = m.forward(q)
    tip = [10, 15, 20, 25][finger]
    assert np.linalg.norm(target[tip]-target[5]) < 1e-6
    for _ in range(12):
        result = solver.solve(None, target=target, budget_s=1.)
    fitted = np.asarray(result['solved_points'])
    assert np.linalg.norm(fitted[tip]-fitted[5]) < .002
    assert np.all(np.asarray(result['joints']) >= m.lower)
    assert np.all(np.asarray(result['joints']) <= m.upper)


@pytest.mark.parametrize('ids', [[0], [1], [2, 3], [0, 1, 2, 3]])
def test_selected_finger_fk_matches_all_fingers(solver, ids):
    angles = solver._finger_angles(solver.model.upper*.47)
    p, j = solver.model.forward_fingers(angles)
    sp, sj = solver.model.forward_fingers(angles, ids)
    np.testing.assert_allclose(sp, p[ids], atol=1e-12)
    np.testing.assert_allclose(sj, j[ids], atol=1e-12)


def test_plane_jacobian_and_straight_degeneracy(solver):
    for q in ([.2, .3, .1], [.2, .3, 0.]):
        p, j = solver.model.thumb_landmarks(q)
        d, jd = solver._directions(p, j)
        normal, jac = solver._plane(d, jd)
        for k in range(3):
            step = np.eye(3)[k]*1e-6
            def plane(x):
                p, j = solver.model.thumb_landmarks(x)
                return solver._plane(*solver._directions(p, j))[0]
            np.testing.assert_allclose((plane(np.array(q)+step)-plane(np.array(q)-step))/2e-6, jac[:, k], atol=1e-7)
    p, j = solver._plane(np.tile([0., 0., 1.], (3, 1)), np.zeros((3, 3, 3)))
    np.testing.assert_array_equal(p, 0.)
    assert np.isfinite(j).all()


def test_publish_ack_only_commits_current_candidate(solver):
    target, _ = solver.model.forward(solver.model.upper*.35)
    before = solver.previous.copy()
    first = solver.solve(None, target=target, source_time=1., commit=False, budget_s=1.)
    np.testing.assert_array_equal(solver.previous, before)
    second = solver.solve(None, target=target, source_time=1.02, commit=False, budget_s=1.)
    assert not solver.commit(first['candidate_id'])
    assert solver.commit(second['candidate_id'])
    np.testing.assert_array_equal(solver.previous, second['joints'])
    assert solver.source_time == 1.02
    assert not solver.commit(second['candidate_id'])
    third = solver.solve(None, target=target, source_time=1.04, commit=False, budget_s=1.)
    solver.discard()
    assert not solver.commit(third['candidate_id'])
    assert solver.source_time == 1.02


def authorize_solver_bounds(solver):
    """Numerical-kernel fixture: explicit permission, not optical intent detection.

    Production permission and zero-motion retention are tested independently in
    test_hand_participation. These tests still exercise the coupled solver when
    all involved joints are explicitly authorized.
    """
    from dataclasses import replace
    propose = solver.participation.propose
    def authorized(base, raw, points, stamp, contact, **context):
        candidate = propose(base, raw, points, stamp, contact, **context)
        lo, hi = candidate.lower.copy(), candidate.upper.copy()
        for f in np.flatnonzero(contact):
            cols = [5] if f == 1 else [[3,4], [], [6,7], [8,9]][f]
            lo[cols] = np.maximum(solver.model.lower[cols], base[cols]-solver.participation.radius[cols])
            hi[cols] = np.minimum(solver.model.upper[cols], base[cols]+solver.participation.radius[cols])
        candidate.diagnostic['eligible'] = contact.tolist()
        # This fixture starts after the permission transition; transition timing
        # and failed-publication behavior have separate end-to-end tests.
        solver.permission_mask = contact.copy()
        solver.permission_progress = 1.
        return replace(candidate, lower=lo, upper=hi)
    solver.participation.propose = authorized


def seed_confirmed_contact(solver, points, stamp, fingers):
    """Start numerical/transition tests after independently verified approach.

    Static proximity is deliberately no longer a production contact trigger.
    Detector admission, coupling and transactional history have separate tests.
    """
    from dataclasses import replace
    distances = np.linalg.norm(points[TIPS[1:]]-points[5], axis=1)
    state,_ = solver.contact_detector.propose(distances, points, solver.model.width, stamp)
    active = np.zeros(4, bool);active[fingers] = True
    solver.contact_detector.commit(replace(state,active=active,projection=active.astype(float)))


@pytest.mark.parametrize('authorized', [False, True])
def test_all_contact_goals_are_checked_but_only_authorized_fingers_move(solver, authorized):
    if authorized: authorize_solver_bounds(solver)
    q = solver.model.upper*.6
    target, _ = solver.model.forward(q)
    # An intentionally impossible multi-finger contact must remain bounded.
    for tip in TIPS[1:]:
        target[tip] = target[5]+[.001, .001, 0.]
    seed_confirmed_contact(solver, target, .98, [0,1,2,3])
    for i in range(6):
        r = solver.solve(None, target=target, source_time=1.+i*.02, budget_s=1.)
    d = r['diagnostics'];correction = np.array(d['correction_deg'])
    assert d['contact_goal_fingers'] == [0, 1, 2, 3]
    assert d['coordination_fingers'] == ([0, 1, 2, 3] if authorized else [])
    assert d['nonlinear_dof'] == (10 if authorized else 3)
    assert d['contact_satisfied'] == bool(np.all(np.array(d['projected_error_mm']) <= 2.001))
    if not authorized: np.testing.assert_array_equal(correction[3:], np.zeros(7))
    assert max(abs(correction[[4, 5, 7, 9]])) <= 15.000001
    assert max(abs(correction[[3, 6, 8]])) <= 5.000001
    assert np.isfinite(r['joints']).all()
    assert np.all(np.asarray(r['joints']) >= solver.model.lower)
    assert np.all(np.asarray(r['joints']) <= solver.model.upper)


def test_coupled_contact_step_improves_posture_without_breaking_contact(solver):
    authorize_solver_bounds(solver)
    import json
    from pathlib import Path
    sample = json.loads((ROOT/'tests/fixtures/hand_contact/middle_boundary.json').read_text())
    solver.previous = np.array(sample['previous'])
    solver.contact = np.array(sample['contact'])
    solver.projection = np.array(sample['projection'])
    solver.source_time = sample['previous_time']
    from dataclasses import replace
    raw = np.array(sample['poses'])[:, :3]
    distances = np.linalg.norm(raw[TIPS[1:]]-raw[5], axis=1)
    state,_ = solver.contact_detector.propose(distances, raw, solver.model.width, solver.source_time)
    solver.contact_detector.commit(replace(state,active=solver.contact.copy(),projection=solver.projection.copy()))
    target = solver.model.human_target(sample['poses'])
    before, _ = solver.model.forward(solver.previous)
    desired = solver._directions(target[2:6])
    prior = np.degrees(np.arccos(np.clip(np.sum(solver._directions(before[2:6])*desired, axis=1), -1., 1.))).mean()
    result = solver.solve(sample['poses'], source_time=sample['source_time'], budget_s=1.)
    d = result['diagnostics']
    assert d['projected_error_mm'][1] <= 2.
    assert np.mean(d['thumb_direction_error_deg']) < prior-.4
    assert d['updates'] <= 6 and d['step_attempts'] <= 12


@pytest.mark.parametrize('fraction', [.05, .35, .7, .95])
def test_reused_chain_landmarks_match_full_fk(solver, fraction):
    target, _ = solver.model.forward(solver.model.upper*fraction)
    result = solver.solve(None, target=target, source_time=1., budget_s=1.)
    expected, _ = solver.model.forward(result['joints'])
    np.testing.assert_allclose(result['solved_points'], expected, atol=1e-12)


def test_coincident_human_tips_keep_finite_two_mm_projection(solver):
    authorize_solver_bounds(solver)  # Projection is only meaningful with explicit permission.
    q = solver.model.upper*.4
    points, _ = solver.model.forward(q)
    poses = np.zeros((26, 7)); poses[:, :3] = points; poses[:, 6] = 1.
    poses[10, :3] = poses[5, :3]
    seed_confirmed_contact(solver, poses[:,:3], .98, [0])
    solver.solve(poses, source_time=1., budget_s=1.)
    result = solver.solve(poses, source_time=1.08, budget_s=1.)
    assert result['diagnostics']['projected_gap_mm'][0] == pytest.approx(2.)
    assert np.isfinite(result['joints']).all()


def test_straight_human_thumb_disables_unobservable_bending_plane(solver):
    target, _ = solver.model.forward(solver.model.neutral)
    axis = target[3]-target[2]; axis /= np.linalg.norm(axis)
    target[3:6] = target[2]+np.cumsum(solver.segment_lengths[0, :, None]*axis, axis=0)
    result = solver.solve(None, target=target, source_time=1., budget_s=1.)
    assert result['diagnostics']['bend_plane_confidence'] < 1e-12
    assert np.isfinite(result['joints']).all()


@pytest.mark.parametrize('finger,thumb,flexion', [
    (0, [.459586348, .517399375, .194684846], .746995347),
    (1, [.481157635, .694469637, .260887264], .786287445),
    (2, [.705381261, .700407807, .251176431], .777403083),
    (3, [.840361861, .821692069, .265871624], .806083171),
])
@pytest.mark.parametrize('entering', [False, True])
def test_slow_contact_transition_does_not_introduce_thumb_jump(solver, finger, thumb, flexion, entering):
    m = solver.model; q = m.upper*.4; q[:3] = thumb
    k = [4,5,7,9][finger]; q[k] = flexion
    outputs=[]; blends=[]; permissions=[]
    motion = np.r_[np.zeros(20),np.linspace(0,.25,30),np.ones(10)*.25]
    if entering: motion = .25-motion
    if not entering:
        points,_=m.forward(q)
        seed_confirmed_contact(solver,points,.98,[finger])
    for i,delta in enumerate(motion):
        goal=q.copy();goal[k]-=delta
        points,_=m.forward(goal);poses=np.zeros((26,7));poses[:,:3]=points;poses[:,6]=1.
        r=solver.solve(poses,source_time=1+i*.02,budget_s=1.)
        outputs.append(r['joints']);blends.append(r['diagnostics']['projection_fraction'][finger])
        permissions.append(r['diagnostics']['constrained_contact'][finger])
    # A seeded contact bit without axis proof no longer starts projection.
    # Entering motion still exercises actual admission; opening-only input must
    # not inherit permission from proximity or the seeded contact bit.
    assert blends[19] == 0.
    assert blends[-1] == (1. if entering else 0.)
    # Projection cannot rise before a newly verified permission, even if the
    # numerical fixture begins with a contact bit. Release may outlive permission.
    assert all(blends[j] <= (blends[j-1] if j else 0.)+1e-12 or permissions[j]
               for j in range(len(blends)))
    assert np.max(abs(np.degrees(np.diff(outputs,axis=0))[20:,:3])) < 1.
    if not entering:
        # This opening-only fixture never grants participation, so corrections stay zero.
        # The geometric baseline uses
        # the filtered optical shape, with a bounded residual deadband offset.
        filtered = np.asarray(r['target_points'])
        np.testing.assert_allclose(outputs[-1][3:], solver.direct_fingers(filtered)[3:], atol=1e-12)
        assert np.max(np.linalg.norm(filtered-m.human_target(poses), axis=1)) < .0006


def test_native_chain_and_jacobian_match_python_reference():
    cfg=read_robot_config(read_settings(ROOT/'config/laptop.yaml'))
    native=HandRetargeter(cfg);rng=np.random.default_rng(412)
    for _ in range(30):
        q=native.model.lower+rng.random(10)*(native.model.upper-native.model.lower)
        p,j=native._native.landmarks(q);rp,rj=native.model.forward(q)
        np.testing.assert_allclose(p,rp,atol=1e-12)
        np.testing.assert_allclose(j,rj,atol=1e-12)


@pytest.mark.parametrize('finger', range(4))
@pytest.mark.parametrize('segment', range(3))
def test_direct_flexion_combines_each_bone_with_urdf_linkage(solver, finger, segment):
    k, basis, axis, idx, neutral, coefficients = solver.finger_geometry[finger]
    q = solver.model.upper*.3
    points, _ = solver.model.forward(q)
    original = solver.direct_fingers(points)
    roll = np.eye(3) if axis is None else Rotation.from_rotvec(axis*original[k-1]).as_matrix()
    # Alter one absolute bone direction, translating the downstream chain so
    # its other bone directions remain unchanged. Each segment must contribute.
    bone = (points[idx[segment+1]]-points[idx[segment]]) @ basis @ roll
    turned = Rotation.from_rotvec([0., .1, 0.]).apply(bone)
    delta = (turned-bone) @ roll.T @ basis.T
    changed = points.copy()
    changed[idx[segment+1:]] += delta
    actual = solver.direct_fingers(changed)
    expected_delta = .1*coefficients[segment]/(coefficients @ coefficients)
    assert actual[k]-original[k] == pytest.approx(expected_delta, abs=1e-7)
    # The URDF's rounded axes leave a sub-microradian change in this finger's
    # reconstructed side angle when its proximal bone is rotated.
    untouched = [j for j in range(10) if j != k and (axis is None or j != k-1)]
    np.testing.assert_array_equal(actual[untouched], original[untouched])
    if axis is not None:
        assert actual[k-1] == pytest.approx(original[k-1], abs=1e-6)


@pytest.mark.parametrize('finger', range(4))
def test_three_bone_seed_does_not_use_metacarpal_direction(solver, finger):
    q = solver.model.upper*.3
    points, _ = solver.model.forward(q)
    original = solver.direct_fingers(points)
    # The previous MCP-only seed used this extra point; three-bone synthesis
    # depends on MCP->PIP, PIP->DIP, DIP->tip, not this metacarpal direction.
    points[6+5*finger] += [.01, -.005, .02]
    np.testing.assert_array_equal(solver.direct_fingers(points), original)


@pytest.mark.parametrize('finger', range(4))
def test_repository_coordination_range_preserves_step_and_release_budget(solver, finger):
    from dataclasses import replace
    g = solver.participation
    k = [4, 5, 7, 9][finger]
    assert np.degrees(g.radius[k]) == pytest.approx(15.)
    assert np.degrees(g.step_radius[k]) == pytest.approx(15.)
    # Isolate the permission gate; no FK or physical motion is needed.
    points = np.zeros((26, 3))
    points[5] = [.04, 0., 0.]
    points[[10, 15, 20, 25]] = [0., 0., .08]
    base = np.ones(10)*.4
    contact = np.ones(4, dtype=bool)
    for i in range(9):
        q = base.copy()
        q[k] += np.deg2rad(i*.65)
        p = points.copy()
        p[[10, 15, 20, 25][finger]] += [i*.0003, 0., -i*.0006]
        c = g.propose(q, q, p, 1.+i*.02, contact)
        g.commit(c.solved(c.fixed, q))
    assert c.diagnostic['eligible'][finger]
    assert np.degrees(c.upper[k]-c.fixed[k]) <= 3.75+1e-9
    # A large authorized offset must still release at the original rate.
    offset = np.zeros(10)
    offset[k] = np.deg2rad(10.)
    g.state = replace(g.state, correction=offset)
    previous = g.state
    c = g.propose(q, q, p, 1.18, np.zeros(4, dtype=bool))
    assert g.state is previous  # rejected candidates cannot consume release
    assert np.degrees(c.fixed[k]-q[k]) == pytest.approx(10.-1.875)
    g.commit(c.solved(c.fixed, q))
    c = g.propose(q, q, p, 1.4, np.zeros(4, dtype=bool))
    assert c.diagnostic['clock_reset']
    assert np.degrees(c.fixed[k]-q[k]) == pytest.approx(10.-1.875)


def test_native_solver_matches_reference_and_deadline_keeps_state():
    cfg=read_robot_config(read_settings(ROOT/'config/laptop.yaml'))
    native=HandRetargeter(cfg);reference=HandRetargeter(cfg,use_native=False)
    rng=np.random.default_rng(88)
    for i in range(25):
        q=native.model.lower+rng.random(10)*(native.model.upper-native.model.lower)
        target,_=native.model.forward(q)
        a=native.solve(None,target=target,source_time=1+i*.02,budget_s=1.)
        b=reference.solve(None,target=target,source_time=1+i*.02,budget_s=1.)
        np.testing.assert_allclose(a['solved_points'],b['solved_points'],atol=1e-7)
        np.testing.assert_allclose(a['joints'],b['joints'],atol=1e-5)
    before=native.previous.copy()
    with pytest.raises(TimeoutError):
        native._native.solve(before,before,np.tile([0.,0.,1.],(3,1)),np.zeros((4,3)),
                             np.zeros(4),np.zeros(4),np.zeros(4),np.ones(3),
                             native.model.lower,native.model.upper,before,-1.)
    np.testing.assert_array_equal(before,native.previous)
