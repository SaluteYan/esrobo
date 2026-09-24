import copy
import json
import threading
import time
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np
import pytest

from esrobo_link.backends import DualBackend, MockBackend, MockDualBackend, HardwareBackend
from esrobo_link.gateway import Gateway
from esrobo_link.protocol import ProtocolError, canonical
from esrobo_teleop import math_utils as mu
from esrobo_teleop.config import build_config
from esrobo_teleop.ik.solver import IkSolver
from esrobo_teleop.robot.linker_hand_driver import LinkerHandDriver, L10_PHYSICAL_JOINT_NAMES

from esrobo_laptop.acquisition import AnnotatedSocket, annotate, senseglove
from esrobo_laptop.app import Controller, ControlRateMonitor, feedback_for, main
from esrobo_laptop.config import ROOT, Settings, read_settings, read_robot_config
from esrobo_laptop.contract import local_id, verify_contract
from esrobo_laptop.demo import run_demo
from esrobo_laptop.inputs import FreshBodyDevice, InputUnavailable
from esrobo_laptop.mapping import hand_units, WristMapping
from esrobo_laptop.pipeline import Pipeline


@pytest.fixture
def config():
    settings = read_settings(ROOT / "config/laptop.yaml")
    return read_robot_config(settings)


def test_contract_rejects_config_or_model_difference(config):
    settings = Settings()
    members = {}
    for side in settings.sides:
        members[side] = SimpleNamespace(contract=dict(
            id=local_id(config, side, True), side=side, with_hand=True,
            arm_order=list(getattr(config.ik, f"{side}_arm_joints")),
            hand_order=L10_PHYSICAL_JOINT_NAMES, arm_unit="URDF radians", hand_unit="L10 physical 0..255"))
    contract = DualBackend(members).contract
    verify_contract(contract, config, settings)
    config.robot.speed_percent += 1
    with pytest.raises(ProtocolError, match="fingerprint"):
        verify_contract(contract, config, settings)


def test_fingerprint_matches_actual_robot_backend_construction(config):
    # Construct the real backend contract while replacing every hardware/mesh
    # dependency. This detects fingerprint drift against the actual gateway.
    arm = Mock()
    arm._effective_position_limits.return_value = (np.full(7, -2.), np.full(7, 2.))
    arm._mapping.return_value = (np.ones(7), np.zeros(7))
    arm._joint_rate_limit_vector.return_value = np.ones(7)
    arm.command_trajectory_enabled = True
    path = ROOT.parent / "teleoperation/config/teleop_config.yaml"
    with patch("esrobo_teleop.robot.nero_driver.NeroSingleArmDriver", return_value=arm), \
         patch("esrobo_teleop.robot.torso_collision.TorsoCollisionGuard"), \
         patch("esrobo_link.backends.ForwardModel"):
        backend = HardwareBackend(path, "left", False)
        assert backend.contract["id"] == local_id(config, "left", False)
        verify_contract(backend.contract, config, Settings(side="left", with_hand=False))
        backend.close()


def test_mock_does_not_accept_real_or_wrong_side_contract(config):
    settings = Settings()
    verify_contract(MockDualBackend().contract, config, settings, mock=True)
    with pytest.raises(ProtocolError):
        verify_contract(MockDualBackend().contract, config, settings)
    with pytest.raises(ProtocolError):
        verify_contract(MockBackend().contract, config, settings, mock=True)


@pytest.mark.parametrize("side", ["left", "right"])
def test_hand_mapping_matches_original_without_device_init(config, side):
    original = LinkerHandDriver.__new__(LinkerHandDriver)
    original._cfg, original._model, original._physical_count = config.hand, "L10", 10
    measured = np.arange(10, dtype=float)+100
    original._feedback = {side: measured}
    rng = np.random.default_rng(20)
    for angles in rng.uniform(-.1, 1.5, (30, 10)):
        expected = original._rad_to_servo(angles, side)
        assert hand_units(angles, measured, config.hand, side) == expected.tolist()
        for idx in (6, 7, 8):
            assert expected[idx] == measured[idx]


def test_feedback_age_accumulates_cache_compute_and_network():
    settings = Settings(side="left", with_hand=False)
    state = dict(mode="ACTIVE", feedback_cache_age_s=.05,
                 feedback=dict(arm_urdf_rad=[0]*7, arm_feedback_age_s=.05))
    feedback_for(state, settings, 10., now=10.01)
    with pytest.raises(InputUnavailable, match="stale"):
        feedback_for(state, settings, 10., now=10.04)
    state["mode"] = "ARMING"
    feedback_for(state, settings, 10., now=11.)  # Robot locally holds during finite arming.
    state["feedback"]["arm_urdf_rad"] = None
    with pytest.raises(InputUnavailable):
        feedback_for(state, settings, 10., now=11.)


def test_hand_feedback_uses_strict_freshness_budget():
    settings = Settings(side="right", with_hand=True)
    state = dict(mode="ACTIVE", feedback_cache_age_s=.01, feedback=dict(
        arm_urdf_rad=[0]*7, arm_feedback_age_s=.01,
        hand=dict(position_unit=[100]*10, age_s=.10)))
    feedback_for(state, settings, 10., now=10.01)
    state["feedback"]["hand"]["age_s"] = .12
    with pytest.raises(InputUnavailable, match="stale"):
        feedback_for(state, settings, 10., now=10.01)


def test_hand_only_feedback_requires_arm_verified_disabled_but_not_arm_angles():
    settings = Settings(side="right", with_hand=True, hand_only=True)
    state = dict(mode="ACTIVE", feedback_cache_age_s=.01, feedback=dict(
        arm_urdf_rad=None, arm_feedback_age_s=None, enable_states=[False]*7,
        controller_fault=None, driver_fault=None,
        hand=dict(position_unit=[100]*10, age_s=.01)))
    measured, members = feedback_for(state, settings, 10., now=10.01)
    assert measured.tolist() == [0.]*14
    assert members['right']['hand']['position_unit'] == [100]*10
    state['feedback']['enable_states'][3] = True
    with pytest.raises(InputUnavailable, match="must remain verified disabled"):
        feedback_for(state, settings, 10., now=10.01)


def test_live_control_rate_monitor_rejects_gap_and_slow_rate():
    monitor = ControlRateMonitor(40, .08)
    for index in range(51):
        monitor.observe(index*.02, "ACTIVE", True)
    with pytest.raises(RuntimeError, match="gap"):
        monitor.observe(1.1, "ACTIVE", False)
    monitor.observe(2, "IDLE", False)
    with pytest.raises(RuntimeError, match="below required"):
        for index in range(31):
            monitor.observe(3+index/30, "ACTIVE", True)


def test_live_control_rate_monitor_accepts_rounding_jitter_at_39_9_hz():
    monitor = ControlRateMonitor(40, .08)
    for index in range(81):
        monitor.observe(index / 39.9, "ACTIVE", True)


def test_hand_only_rate_tolerance_allows_boundary_dip_but_rejects_sustained_slowdown():
    monitor = ControlRateMonitor(40, .08, rate_tolerance_hz=5.0)
    for index in range(81):
        monitor.observe(index / 37.7, "ACTIVE", True)
    monitor.observe(3, "IDLE", False)
    with pytest.raises(RuntimeError, match="stop below 35.0 Hz"):
        for index in range(81):
            monitor.observe(4 + index / 34.9, "ACTIVE", True)


def test_hand_only_rate_tolerance_keeps_gap_watchdog():
    monitor = ControlRateMonitor(40, .08, rate_tolerance_hz=5.0)
    monitor.observe(0, "ACTIVE", True)
    with pytest.raises(RuntimeError, match="gap"):
        monitor.observe(.081, "ACTIVE", False)


def test_hand_only_rate_floor_cannot_be_configured_below_35_hz():
    settings = Settings(side="right", with_hand=True, hand_only=True)
    settings.validate()
    settings.hand_only_rate_tolerance_hz = 5.1
    with pytest.raises(ValueError, match="at least 35 Hz"):
        settings.validate()


def test_hand_only_two_second_rate_window_allows_short_source_slowdown():
    monitor = ControlRateMonitor(40, .08, rate_tolerance_hz=5.0, rate_window_s=2.0)
    for index in range(135):
        monitor.observe(index / 45, "ACTIVE", True)
    for index in range(1, 35):
        monitor.observe(3 + index / 34, "ACTIVE", True)
    with pytest.raises(RuntimeError, match="stop below 35.0 Hz over 2 s"):
        for index in range(35, 103):
            monitor.observe(3 + index / 34, "ACTIVE", True)


def test_control_rate_monitor_ignores_preview_idle_period():
    monitor = ControlRateMonitor(40, .08)
    monitor.observe(0, "IDLE", False)
    monitor.observe(10, "IDLE", False)


@pytest.fixture
def body(config):
    cfg = copy.deepcopy(config.retarget)
    cfg.host, cfg.port = "127.0.0.1", 0
    device = FreshBodyDevice(cfg, ("left", "right"), True)
    yield device
    device.close()


def glove_packet(left_stamp, right_stamp, physical=("left", "right")):
    return annotate(dict(hand_joints=[0.]*20,
                         hand_orientation_source_frame="senseglove_zeroed_anatomical_axes",
                         hand_orientation_deltas=dict(left=[1, 0, 0, 0], right=[1, 0, 0, 0])),
                    "senseglove", dict(left=left_stamp, right=right_stamp), physical)


def test_legacy_glove_cannot_refresh_or_overwrite_pico(body):
    stamp = time.monotonic()
    body._handle_packet(glove_packet(stamp, stamp))
    before = dict(body.stamps)
    body._handle_packet(glove_packet(time.monotonic(), stamp))
    assert body.stamps == before
    body._handle_packet(glove_packet(time.monotonic(), time.monotonic(), ("left",)))
    assert body.stamps == before
    assert not body.stamps
    assert body.hand_joints() is None


def test_missing_imu_rejects_whole_packet(body):
    stamp = time.monotonic()
    packet = json.loads(glove_packet(stamp, stamp))
    del packet["hand_orientation_deltas"]["right"]
    body._handle_packet(json.dumps(packet).encode())
    assert not body.stamps


def test_packet_without_local_metadata_and_old_timestamp_rejected(body):
    body._handle_packet(b'{"hand_joints": [0]}')
    body._handle_packet(glove_packet(time.monotonic()-1, time.monotonic()))
    assert not body.stamps


def test_independent_input_tickets_and_no_replay(body):
    now = time.monotonic()
    body.stamps = {k: now for k in body.required}
    body.is_ready = lambda: True
    ticket = body.ticket(.1)
    body.consume(ticket)
    assert body.ticket(.1) is None
    body.stamps["left_arm"] = time.monotonic()
    assert body.ticket(.1) is None
    body.stamps["right_hand"] = now-1
    with pytest.raises(InputUnavailable):
        body.ticket(.1)


def test_adapter_preserves_ros_sample_time():
    sock = Mock()
    wrapped = AnnotatedSocket(sock, lambda: ("senseglove", dict(left=10., right=11.), ("left", "right")))
    wrapped.sendto(b'{"hand_joints": []}', ("127.0.0.1", 15050))
    sent = json.loads(sock.sendto.call_args.args[0])
    assert sent["laptop_input"]["sample_monotonic"] == dict(left=10., right=11.)


@pytest.mark.parametrize("rx, expected", [(60, 10.), (0, 0.)])
def test_senseglove_entrypoint_marks_dead_sdk_and_single_glove(rx, expected):
    sock = Mock()
    source = SimpleNamespace(stamp_monotonic=10., imu_quat_wxyz=[1, 0, 0, 0],
                             packets_per_second_received=rx, has_live_sensor_payload=lambda: True)
    class Bridge:
        left = right = source
        args = SimpleNamespace(single_side="left", swap_left_right_targets=False)
        def send(self, sock, calibration):
            sock.sendto(b'{"hand_joints": []}', ("127.0.0.1", 15050))
    module = SimpleNamespace(SenseGloveUdpBridge=Bridge, main=lambda argv: Bridge().send(sock, {}))
    with patch("esrobo_laptop.acquisition.load_bridge", return_value=module):
        senseglove([])
    meta = json.loads(sock.sendto.call_args.args[0])["laptop_input"]
    assert meta["sample_monotonic"]["left"] == expected
    assert meta["physical_sides"] == ["left"]


@pytest.mark.parametrize("side", ["left", "right"])
def test_partitioned_ik_and_wrist_mapping_without_pink(config, side):
    solver = IkSolver(config.ik)
    q = np.tile(np.array([0., -np.pi/2, 0., .4, 0., 0., 0.]), 2)
    lower = np.array([-2., -2., -2., -2., -.3, -.2, -.1])
    upper = np.array([2., 2., 2., 2., .3, .2, .1])
    solver.configure_position_envelope(side, lower, upper)
    poses = solver.current_task_frame_poses(q)
    wrist = WristMapping(solver, config.robot, side, q)
    np.testing.assert_allclose(wrist.terminal(np.eye(3)), 0, atol=1e-6)
    extreme = wrist.terminal(mu.rotvec_to_rotation_matrix(np.array([2., -1.5, 1.])))
    assert np.all(extreme >= lower[4:])
    assert np.all(extreme <= upper[4:])
    solver.initialize_arm_session(side, q, .4)
    with patch.dict("sys.modules", {"pink": None}):
        result = solver.solve(poses["left_wrist"], poses["right_wrist"], q,
                              poses["left_elbow"], poses["right_elbow"], side, np.zeros(3))
    assert solver.last_solution_valid
    offset = 0 if side == "left" else 7
    actual = solver.current_task_frame_poses(result)
    np.testing.assert_allclose(actual[f"{side}_wrist"][:3], poses[f"{side}_wrist"][:3], atol=.001)
    np.testing.assert_allclose(result[offset+4:offset+7], 0, atol=1e-6)


def test_controller_hold_never_sends_solved_target_before_active():
    settings = Settings(side="left", with_hand=False)
    backend = MockBackend("left", False)
    contract = backend.contract
    client = Mock(received_at=time.monotonic())
    pipeline = Mock(initialized=True)
    pipeline.body.input_lock = threading.RLock()
    pipeline.body.ticket.return_value = {"left_arm": time.monotonic()}
    pipeline.capture.return_value = ({"left_arm": time.monotonic()}, {}, None)
    controller = Controller(client, pipeline, settings, contract)
    state = dict(mode="IDLE", contract=contract, feedback=backend.snapshot(), feedback_cache_age_s=0.)
    controller.step(state)
    pipeline.solve.assert_not_called()
    client.send_target.assert_called_once_with(arm_urdf_rad=[0.]*7, hand_unit=None)
    pipeline.publish_diagnostics.assert_called_once()
    pipeline.check_ticket.side_effect = InputUnavailable("slow computation")
    client.reset_mock()
    with pytest.raises(InputUnavailable):
        controller.step(state)
    client.send_target.assert_not_called()


def test_pico_open_reference_maps_first_hand_target_to_robot_open(config):
    side = "right"
    reference = np.asarray([.2, .4, .1, .05, .12, .1, .03, .11, .02, .1])
    measured = np.asarray(config.hand.right_open_feedback, dtype=float)
    target = hand_units(reference, measured, config.hand, side, reference)
    expected = np.asarray(config.hand.right_open, dtype=int)
    for index in config.hand.enabled_physical_joints:
        assert target[index] == expected[index]


def test_right_pico_full_fist_reaches_four_finger_closed_end_without_changing_thumb(config):
    from esrobo_teleop.robot.linker_hand_driver import ACTIVE_HAND_JOINTS, ACTIVE_JOINT_LIMITS_RAD
    upper = np.asarray([ACTIVE_JOINT_LIMITS_RAD[name][1] for name in ACTIVE_HAND_JOINTS])
    reference = np.zeros(10)
    angles = upper * .8
    angles[[0, 1, 2]] = 0  # Isolate four-finger flexion from thumb tuning.
    feedback = np.asarray(config.hand.right_open_feedback, dtype=float)
    target = hand_units(angles, feedback, config.hand, "right", reference)
    assert [target[index] for index in (2, 3, 4, 5)] == [0, 0, 0, 0]
    for index in (0, 1, 9):
        assert target[index] == config.hand.right_open[index]
    opened = hand_units(reference, feedback, config.hand, "right", reference)
    assert [opened[index] for index in (2, 3, 4, 5)] == [255]*4
    left = hand_units(angles, np.asarray(config.hand.left_open, dtype=float),
                      config.hand, "left", reference)
    assert [left[index] for index in (2, 3, 4, 5)] == [51]*4


def test_right_pico_thumb_flexes_more_and_sways_less_without_coupling(config):
    from esrobo_teleop.robot.linker_hand_driver import ACTIVE_HAND_JOINTS, ACTIVE_JOINT_LIMITS_RAD
    from esrobo_laptop.mapping import RIGHT_THUMB_FLEXION_GAIN, RIGHT_THUMB_SIDE_GAIN

    upper = np.asarray([ACTIVE_JOINT_LIMITS_RAD[name][1] for name in ACTIVE_HAND_JOINTS])
    reference = np.zeros(10)
    angles = np.zeros(10)
    angles[2] = upper[2] * .4  # Thumb curl -> physical pitch 0.
    angles[1] = upper[1] * .8  # Thumb opposition -> physical side 1.
    right_feedback = np.asarray(config.hand.right_open_feedback, dtype=float)
    right = hand_units(angles, right_feedback, config.hand, "right", reference)
    baseline_pitch = round(config.hand.right_open[0] + .4 *
                           (config.hand.right_teleop_closed[0] - config.hand.right_open[0]))
    baseline_side = round(config.hand.right_open[1] + .8 *
                          (config.hand.right_teleop_closed[1] - config.hand.right_open[1]))
    assert right[0] == round(config.hand.right_open[0] + .4 * RIGHT_THUMB_FLEXION_GAIN *
                             (config.hand.right_teleop_closed[0] - config.hand.right_open[0]))
    assert right[1] == round(config.hand.right_open[1] + .8 * RIGHT_THUMB_SIDE_GAIN *
                             (config.hand.right_teleop_closed[1] - config.hand.right_open[1]))
    assert right[0] < baseline_pitch
    assert right[1] > baseline_side
    assert right[9] == config.hand.right_open[9]
    assert right[2:6] == config.hand.right_open[2:6]

    left = hand_units(angles, np.asarray(config.hand.left_open_feedback, dtype=float),
                      config.hand, "left", reference)
    assert left[0] == round(config.hand.left_open[0] + .4 *
                            (config.hand.left_closed[0] - config.hand.left_open[0]))
    assert left[1] == round(config.hand.left_open[1] + .8 *
                            (config.hand.left_closed[1] - config.hand.left_open[1]))


def test_controller_starts_new_reference_if_returning_state_was_too_brief():
    settings = Settings(side="left", with_hand=False)
    backend = MockBackend("left", False)
    contract = backend.contract
    client = Mock(received_at=time.monotonic())
    pipeline = Mock(initialized=False, preparation_status="")
    pipeline.body.input_lock = threading.RLock()
    pipeline.body.is_ready.return_value = False
    pipeline.body.ticket.side_effect = InputUnavailable("reference not ready")
    controller = Controller(client, pipeline, settings, contract)
    state = dict(mode="CALIBRATING", contract=contract, feedback=backend.snapshot(),
                 feedback_cache_age_s=0.)
    with pytest.raises(InputUnavailable):
        controller.step(state)
    pipeline.restart_preparation.assert_called_once_with()
    assert controller.preparing


def test_controller_waits_for_robot_zero_before_starting_reference_capture():
    settings = Settings(side="left", with_hand=True)
    backend = MockBackend("left", True)
    pipeline = Mock(initialized=False, preparation_status="")
    controller = Controller(Mock(), pipeline, settings, backend.contract)
    state = dict(mode="RETURNING", contract=backend.contract)

    assert controller.step(state) is None
    pipeline.restart_preparation.assert_not_called()
    assert "机器人正在回零" in pipeline.preparation_status

    pipeline.body.input_lock = threading.RLock()
    pipeline.body.is_ready.return_value = False
    pipeline.body.ticket.side_effect = InputUnavailable("reference not ready")
    state.update(mode="CALIBRATING", feedback=backend.snapshot(), feedback_cache_age_s=0.)
    controller.client.received_at = time.monotonic()
    with pytest.raises(InputUnavailable):
        controller.step(state)
    pipeline.restart_preparation.assert_called_once_with()


def test_laptop_pipeline_builds_viewer_layers_for_selected_arm():
    class Solver:
        @staticmethod
        def current_task_frame_poses(q):
            value = float(np.asarray(q)[7])
            return {
                'right_shoulder': np.array([0., 0., 0., 1., 0., 0., 0.]),
                'right_elbow': np.array([value, 1., 0., 1., 0., 0., 0.]),
                'right_wrist': np.array([value, 2., 0., 1., 0., 0., 0.]),
            }
    pipeline = Pipeline.__new__(Pipeline)
    pipeline.settings = Settings(side='right', with_hand=True)
    pipeline.solvers = {'right': Solver()}
    poses = {
        'right_elbow': np.array([.2, 1., 0., 1., 0., 0., 0.]),
        'right_wrist': np.array([.3, 2., 0., 1., 0., 0., 0.]),
    }
    target = [.1]*7
    message = pipeline.arm_diagnostics(
        ({'right_arm': time.monotonic()}, poses, np.zeros(20)),
        {'right': {'arm_urdf_rad': target, 'hand_unit': [128]*10}},
        np.zeros(14), preview=True, robot_mode='IDLE')
    assert message['side'] == 'right' and message['preview'] is True
    assert message['frames']['retarget']['right_elbow']['pos'] == [.2, 1., 0.]
    assert message['frames']['ik']['right_elbow']['pos'][0] == pytest.approx(.1)
    assert message['frames']['feedback']['right_elbow']['pos'][0] == 0
    np.testing.assert_allclose(message['joints_deg']['ik'], np.degrees(target))


def test_contract_change_stops_before_target():
    client, pipeline = Mock(), Mock()
    controller = Controller(client, pipeline, Settings(), dict(id="first"))
    with pytest.raises(ProtocolError):
        controller.step(dict(contract=dict(id="changed")))
    pipeline.capture.assert_not_called()


def test_real_udp_acquisition_ik_dual_targets_and_watchdog():
    result = run_demo()
    assert result["active_frames"] >= 10
    assert result["final_mode"] == "FAULT"


def test_cli_without_input_never_sends_target(tmp_path):
    key = b"test-cli-local-only-key-not-production-123456"
    path = tmp_path / "key"
    path.write_bytes(key)
    gateway = Gateway(MockDualBackend(), key, port=0)
    worker = threading.Thread(target=gateway.run, daemon=True)
    worker.start()
    # The app binds this ephemeral port after the reservation socket is released.
    import socket
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reservation:
        reservation.bind(("127.0.0.1", 0))
        input_port = reservation.getsockname()[1]
    try:
        result = main(["run", "--mock", "--host", "127.0.0.1", "--port", str(gateway.address[1]),
                       "--key-file", str(path), "--input-port", str(input_port), "--seconds", ".12",
                       "--log-file", str(tmp_path / "session.jsonl")])
        assert result == 0
        assert gateway.gate.target is None
        assert all(not m.enabled for m in gateway.backend.members.values())
        records = [json.loads(line) for line in (tmp_path / "session.jsonl").read_text().splitlines()]
        assert records and all(r["sent"] == 0 for r in records)
        assert key.decode() not in (tmp_path / "session.jsonl").read_text()
    finally:
        gateway.request("q")
        worker.join(timeout=2)
