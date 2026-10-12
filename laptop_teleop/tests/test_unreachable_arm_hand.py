"""Recorded J1 boundary and arm/hand pause regressions; no hardware drivers."""
import copy
import threading
import time
from unittest.mock import Mock, patch

import numpy as np
import pytest
# Load the numeric dependency before stamping simulated input. Cold import
# latency is independent of the hold policy; expiry remains tested below.
import scipy.optimize

from esrobo_laptop.app import Controller, send_targets
from esrobo_laptop.config import ROOT, Settings, read_robot_config, read_settings
from esrobo_laptop.inputs import InputUnavailable
from esrobo_laptop.pipeline import Pipeline
from esrobo_teleop.ik.solver import IkSolver
from esrobo_link.backends import MockBackend
from esrobo_link.protocol import ProtocolError


def recorded_pipeline(*, old_j1_limit=True):
    settings = read_settings(ROOT / "config/laptop.yaml")
    settings.side, settings.with_hand = "right", True
    cfg = read_robot_config(settings)
    solver = IkSolver(copy.deepcopy(cfg.ik))
    upper = list(cfg.robot.right_joint_soft_upper_limits_urdf)
    if old_j1_limit:
        upper[0] = 1.047198
    solver.configure_position_envelope("right", cfg.robot.right_joint_soft_lower_limits_urdf, upper)
    measured = np.zeros(14)
    measured[7:] = [0.3846880204320702, -0.8767836030318713,
                   0.8892103473060711, 1.337602885435934,
                   -0.9058782416626169, -0.30839967882739805, -0.2660230845889757]
    solver._prev_targets = measured.copy()
    solver._prev_targets[7:11] = [1.047198, -0.8579723953352318,
                                0.47671148708473204, 1.3498958573435322]
    terminal = np.array([-0.0002443460952792061, -8.726646259971648e-05,
                         -0.02192133540504878])
    poses = solver.current_task_frame_poses(measured)
    poses["right_elbow"][:3] = [0.05300046503543854, -0.45821136236190796, 1.1127511262893677]
    poses["right_wrist"][:3] = [0.4183630347251892, -0.2992064952850342, 1.0840877294540405]
    poses["hand_local_orientations"] = {"right": np.eye(3)}
    wrist = Mock(reference_terminal=terminal, last_diagnostics={})
    wrist.absolute_terminal.return_value = measured[11:].copy()
    # absolute_terminal normally populates this diagnostic during the call.
    wrist.last_diagnostics["desired_world_quat_wxyz"] = [1., 0., 0., 0.]
    pipeline = Pipeline.__new__(Pipeline)
    pipeline.cfg, pipeline.settings = cfg, settings
    pipeline.solvers, pipeline.wrists = {"right": solver}, {"right": wrist}
    pipeline.hand_references = {}
    pipeline.position_limit_holds = {"right": False}
    pipeline.position_limit_approaching = {"right": False}
    pipeline.position_limit_recovery = {"right": 0}
    feedback = {"right": {"hand": {"position_unit": [143, 209, 254, 254, 254, 254, 0, 0, 0, 33]}}}
    frame = ({"right_arm": time.monotonic()}, poses, np.zeros(20))
    return pipeline, solver, wrist, measured, feedback, frame


def solve_fresh(pipeline, measured, feedback, frame, **kwargs):
    return pipeline.solve(({f"{pipeline.settings.side}_arm": time.monotonic()}, frame[1], frame[2]),
                          measured, feedback, **kwargs)


def side_pipeline(side):
    result = recorded_pipeline()
    if side == 'right':
        return result
    p, _, wrist, _, _, _ = result
    p.settings.side = side
    solver = IkSolver(copy.deepcopy(p.cfg.ik))
    solver.configure_position_envelope(side, p.cfg.robot.left_joint_soft_lower_limits_urdf,
                                       p.cfg.robot.left_joint_soft_upper_limits_urdf)
    measured = np.zeros(14)
    measured[:7] = [.1, -.4, -.7, .9, .0, .0, .0]
    solver._prev_targets = measured.copy()
    poses = solver.current_task_frame_poses(measured)
    # An unreachable translation preserves a non-degenerate forearm vector.
    poses['left_elbow'][:3] += [.8, 0., 0.]
    poses['left_wrist'][:3] += [.8, 0., 0.]
    poses['hand_local_orientations'] = {'left': np.eye(3)}
    wrist.reference_terminal = measured[4:7].copy()
    wrist.absolute_terminal.return_value = measured[4:7].copy()
    p.solvers, p.wrists = {side: solver}, {side: wrist}
    p.position_limit_holds = {side: False}
    p.position_limit_approaching = {side: False}
    p.position_limit_recovery = {side: 0}
    feedback = {side: {'hand': {'position_unit': [254,127,254,254,254,254,0,128,99,112]}}}
    return p, solver, wrist, measured, feedback, ({'left_arm': time.monotonic()}, poses, np.zeros(20))


def test_left_real_ik_unreachable_target_holds_measured_arm_and_fingers():
    p, solver, wrist, measured, feedback, frame = side_pipeline('left')
    with patch('esrobo_laptop.pipeline.hand_units') as fingers:
        target = solve_fresh(p, measured, feedback, frame, allow_limit_hold=True)
        fingers.assert_not_called()
    assert solver.solution_diagnostics['position_target_unreachable']
    assert target['left']['hold']
    np.testing.assert_array_equal(target['left']['arm_urdf_rad'], measured[:7])
    assert target['left']['hand_unit'] == feedback['left']['hand']['position_unit']
    wrist.absolute_terminal.assert_not_called()
    client = Mock()
    send_targets(client, target, p.settings)
    client.send_hold.assert_called_once_with(reason='position')
    client.send_target.assert_not_called()


def test_recorded_boundary_pauses_arm_wrist_and_fingers_before_feedback_reaches_limit():
    p, solver, wrist, measured, feedback, frame = recorded_pipeline()
    with patch("esrobo_laptop.pipeline.hand_units") as fingers:
        target = solve_fresh(p, measured, feedback, frame, allow_limit_hold=True)
        fingers.assert_not_called()
    assert solver.solution_diagnostics["position_target_unreachable"]
    assert solver.solution_diagnostics["position_limit_approach"]
    assert solver.solution_diagnostics["elbow_error_m"] == pytest.approx(.030902854655, abs=1e-8)
    assert not solver.solution_diagnostics["active_position_bounds"][0]["feedback_at_bound"]
    assert target["right"]["hold"] and not target["right"]["limit_approach"]
    np.testing.assert_array_equal(target["right"]["arm_urdf_rad"], measured[7:])
    assert target["right"]["hand_unit"] == feedback["right"]["hand"]["position_unit"]
    wrist.absolute_terminal.assert_not_called()
    assert wrist.last_diagnostics["position_target_paused"]
    client = Mock()
    send_targets(client, target, p.settings)
    client.send_hold.assert_called_once_with(reason="position")
    client.send_target.assert_not_called()


def test_forward_75deg_envelope_solves_recorded_target_and_preserves_wrist_layer():
    p, solver, wrist, measured, feedback, frame = recorded_pipeline(old_j1_limit=False)
    result = solve_fresh(p, measured, feedback, frame, allow_limit_hold=True)
    assert solver.last_solution_valid
    assert not result["right"]["hold"]
    assert np.degrees(result["right"]["arm_urdf_rad"][0]) == pytest.approx(68.238032, abs=.001)
    assert max(solver.solution_diagnostics[k] for k in ("elbow_error_m", "wrist_error_m")) < 1e-5
    wrist.absolute_terminal.assert_called_once()


@pytest.mark.parametrize('side', ['left', 'right'])
def test_pause_does_not_require_an_active_bound_and_recovery_needs_three_good_frames(side):
    p, _, wrist, measured, feedback, frame = side_pipeline(side)
    solver = Mock(_prev_targets=measured.copy(), last_solution_valid=False,
                  solution_diagnostics=dict(position_target_unreachable=True,
                      active_position_bounds=[], elbow_error_m=.05, wrist_error_m=.04))
    solver.solve.return_value = measured.copy()
    p.solvers = {side: solver}
    assert solve_fresh(p, measured, feedback, frame, allow_limit_hold=True)[side]["hold"]
    solver.last_solution_valid = True
    solver.solution_diagnostics = dict(elbow_error_m=.02, wrist_error_m=.01)
    for _ in range(2):
        assert solve_fresh(p, measured, feedback, frame, allow_limit_hold=True)[side]["hold"]
    # Valid IK at 26 mm is still outside the tighter recovery margin.
    solver.solution_diagnostics["elbow_error_m"] = .026
    assert solve_fresh(p, measured, feedback, frame, allow_limit_hold=True)[side]["hold"]
    solver.solution_diagnostics["elbow_error_m"] = .02
    for _ in range(2):
        assert solve_fresh(p, measured, feedback, frame, allow_limit_hold=True)[side]["hold"]
    wrist.absolute_terminal.assert_not_called()
    result = solve_fresh(p, measured, feedback, frame, allow_limit_hold=True)
    assert not result[side]["hold"]
    wrist.absolute_terminal.assert_called_once()
    assert not wrist.last_diagnostics["position_target_paused"]


@pytest.mark.parametrize('side', ['left', 'right'])
def test_entry_still_rejects_unreachable_target(side):
    p, _, _, measured, feedback, frame = side_pipeline(side)
    with pytest.raises(InputUnavailable, match=f"{side} IK rejected"):
        solve_fresh(p, measured, feedback, frame, allow_limit_hold=False)


@pytest.mark.parametrize("failure", ["degenerate", "terminal_outside", "no_previous_target"])
@pytest.mark.parametrize('side', ['left', 'right'])
def test_invalid_geometry_terminal_or_missing_seed_never_becomes_a_hold(failure, side):
    p, solver, wrist, measured, feedback, frame = side_pipeline(side)
    if failure == "degenerate":
        frame[1][f"{side}_wrist"][:3] = frame[1][f"{side}_elbow"][:3]
    elif failure == "terminal_outside":
        wrist.reference_terminal[0] = 10.
    else:
        solver._prev_targets = None
    with pytest.raises(InputUnavailable, match=f"{side} IK rejected"):
        solve_fresh(p, measured, feedback, frame, allow_limit_hold=True)


def test_dual_arm_mode_does_not_gain_single_arm_unreachable_hold():
    p, _, _, measured, feedback, frame = side_pipeline('left')
    p.settings.side = 'both'
    with pytest.raises(InputUnavailable, match='left IK rejected'):
        solve_fresh(p, measured, feedback, frame, allow_limit_hold=True)


def test_model_positive_j1_is_forward_and_runtime_bounds_match_defaults():
    from esrobo_teleop.config import build_config
    p, solver, _, measured, _, _ = recorded_pipeline(old_j1_limit=False)
    upper = p.cfg.robot.right_joint_soft_upper_limits_urdf
    np.testing.assert_array_equal(upper, build_config().robot.right_joint_soft_upper_limits_urdf)
    assert np.degrees(upper[0]) == pytest.approx(75., abs=.0001)
    assert upper[0] < p.cfg.robot.joint_upper_limits[0] - p.cfg.robot.joint_limit_margin
    q = measured.copy()
    q[7] = 0.
    before = solver.current_task_frame_poses(q)
    q[7] = np.deg2rad(15.)
    after = solver.current_task_frame_poses(q)
    assert after["right_elbow"][0] > before["right_elbow"][0]


@pytest.mark.parametrize('side', ['left', 'right'])
def test_fresh_unreachable_frames_clear_input_gap_without_resuming_motion(side):
    settings = Settings(side=side, with_hand=True)
    backend = MockBackend(side, True)
    backend.enabled = backend.hand_enabled = True
    client = Mock(received_at=time.monotonic())
    pipeline = Mock(initialized=True)
    pipeline.body.input_lock = threading.RLock()
    pipeline.body.ticket.return_value = {f"{side}_arm": time.monotonic()}
    pipeline.capture.return_value = ({f"{side}_arm": time.monotonic()}, {}, np.zeros(20))
    accepted = {side: dict(arm_urdf_rad=[0.] * 7, hand_unit=[128] * 10, hold=False)}
    pipeline.solve.return_value = accepted
    c = Controller(client, pipeline, settings, backend.contract)
    state = dict(mode="ACTIVE", contract=backend.contract,
                 feedback=backend.snapshot(), feedback_cache_age_s=0.)
    c.step(state)
    c.last_active_target_at -= .05
    c.hold_missing_input(state, "no new PICO frame")
    paused = copy.deepcopy(accepted)
    paused[side]["hold"] = True
    pipeline.solve.return_value = paused
    for _ in range(2):
        assert c.step(state) is None
    assert c.step(state) == paused
    assert c.input_hold_started_at is None
    # Remain in the position pause; the old input-gap deadline no longer runs.
    for _ in range(5):
        assert c.step(state) == paused
    assert client.send_target.call_count == 1
    assert client.send_hold.call_count == 9


@pytest.mark.parametrize('side', ['left', 'right'])
def test_pause_does_not_bypass_expired_input_ticket_or_missing_hand_feedback(side):
    p, _, _, measured, feedback, frame = side_pipeline(side)
    with pytest.raises(InputUnavailable, match="input expired"):
        p.solve(({f"{side}_arm": time.monotonic() - .2}, frame[1], frame[2]),
                measured, feedback, allow_limit_hold=True)
    feedback[side]["hand"]["position_unit"] = None
    with pytest.raises(InputUnavailable, match="fresh held hand feedback"):
        solve_fresh(p, measured, feedback, frame, allow_limit_hold=True)


@pytest.mark.parametrize('side', ['left', 'right'])
def test_recorded_position_pause_survives_500ms_through_real_udp_gateway(tmp_path, side):
    from esrobo_link.client import RobotClient
    from esrobo_link.gateway import Gateway
    p, _, _, measured, feedback, frame = side_pipeline(side)
    key = b'offline-position-pause-regression-only'*2
    keyfile = tmp_path/'key'
    keyfile.write_bytes(key)
    backend = MockBackend(side, True)
    backend.contract.update(lower_rad=list(getattr(p.cfg.robot, f'{side}_joint_soft_lower_limits_urdf')),
                            upper_rad=list(getattr(p.cfg.robot, f'{side}_joint_soft_upper_limits_urdf')))
    offset = 0 if side == 'left' else 7
    arm = measured[offset:offset+7]
    backend.q = arm.tolist()
    backend.hand = feedback[side]['hand']['position_unit'].copy()
    gateway = Gateway(backend, key, port=0)
    worker = threading.Thread(target=gateway.run, daemon=True)
    worker.start()
    client = RobotClient('127.0.0.1', gateway.address[1], keyfile)
    try:
        state = client.connect()
        client.send_target(arm.tolist(), backend.hand)
        while state.get('accepted_seq', -1) < 1:
            state = client.receive(.2)
        with gateway.lock:
            gateway.mode = 'ACTIVE'
            backend.enabled = backend.hand_enabled = True
        time.sleep(.03)
        deadline = time.monotonic()+.8
        while time.monotonic() < deadline:
            state = client.receive(.2)
            assert state['mode'] == 'ACTIVE', state['reason']
            targets = solve_fresh(p, measured, feedback, frame, allow_limit_hold=True)
            send_targets(client, targets, p.settings)
            time.sleep(.02)
        state = client.receive(.2)
        assert state['mode'] == 'ACTIVE', state['reason']
        assert state['hold_reason'] == 'position'
        assert state['position_hold_age_s'] > .5
        assert state['input_hold_age_s'] is None
        np.testing.assert_array_equal(backend.q, arm)
        assert backend.hand == feedback[side]['hand']['position_unit']
    finally:
        client.close()
        gateway.request('q')
        worker.join(timeout=2)


def test_laptop_refuses_gateway_with_old_hold_semantics():
    from esrobo_laptop.contract import verify_contract
    p, _, _, _, _, _ = recorded_pipeline()
    old = dict(MockBackend('right', True).contract, input_hold_protocol=1)
    with pytest.raises(ProtocolError, match='protocol 2'):
        verify_contract(old, p.cfg, p.settings, mock=True)
