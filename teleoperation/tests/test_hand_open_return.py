"""Hand return checks run without CAN, ROS or motion hardware."""
import time
from pathlib import Path
from unittest.mock import Mock, patch
import numpy as np
import pytest
from esrobo_teleop.config import load_config
from esrobo_teleop.robot.linker_hand_driver import LinkerHandDriver
from esrobo_teleop.robot.hand_geometry import geometry_state


@pytest.fixture(params=['left', 'right'])
def hand(request):
    cfg = load_config(str(Path(__file__).resolve().parents[1]/'config/teleop_config.yaml')).hand
    cfg.feedback_udp_port = 0
    with patch('esrobo_teleop.robot.linker_hand_driver.socket.socket') as sock:
        sock.return_value.recvfrom.side_effect = BlockingIOError
        d = LinkerHandDriver(cfg, udp_port=9, active_sides=(request.param,))
        side = request.param
        d._feedback[side] = np.asarray(getattr(cfg, side+'_open_feedback'), dtype=float)
        d._feedback_time[side] = d._geometry_feedback_time[side] = time.monotonic()
        yield d, side
        d.close()


def test_open_sweep_preserves_sample_time_and_covers_measured_and_end(hand):
    d, side = hand
    d._feedback[side][1] -= 50
    source = d.geometry_state(side, 0.)
    bound = d.open_return_geometry_state(side)
    assert bound.valid and len(bound.intervals) == 10
    assert bound.stamp == source.stamp
    target = d.open_feedback_target(side)
    from esrobo_teleop.robot.linker_hand_driver import L10_PHYSICAL_JOINT_NAMES
    for fraction in np.linspace(0, 1, 21):
        raw = np.rint(d._feedback[side]*(1-fraction)+target*fraction)
        state = geometry_state(raw, source.stamp, side, d._cfg.geometry_feedback_calibration,
                               L10_PHYSICAL_JOINT_NAMES, 0., d._cfg.geometry_feedback_timeout_s)
        assert state.valid
        assert all(bound.intervals[k][0] <= lo <= hi <= bound.intervals[k][1]
                   for k, (lo, hi) in state.intervals.items())


def test_open_sweep_rejects_stale_or_missing_calibration(hand):
    d, side = hand
    d._geometry_feedback_time[side] = time.monotonic()-1
    assert not d.open_return_geometry_state(side).valid
    d._geometry_feedback_time[side] = time.monotonic()
    d._cfg.geometry_feedback_calibration[side].pop('thumb_cmc_roll')
    assert not d.open_return_geometry_state(side).valid


def test_rejected_safety_check_or_cancellation_sends_no_frame(hand):
    d, _ = hand
    d._send_physical = Mock()
    assert not d.return_to_open(safety_check=lambda: False, force=True)
    d._send_physical.assert_not_called()
    cancelled = [False]
    def check():
        cancelled[0] = True
        return True
    assert not d.return_to_open(lambda: cancelled[0], safety_check=check, force=True)
    d._send_physical.assert_not_called()


def test_explicit_return_ignores_quit_preference_but_checks_every_send(hand):
    d, side = hand
    d._cfg.return_open_on_quit = False
    d._cfg.return_open_duration_s = .1
    d._feedback[side][1] -= 20
    events = []
    d._send_physical = Mock(side_effect=lambda *args: events.append('send'))
    def check():
        events.append('check')
        return True
    with patch('esrobo_teleop.robot.linker_hand_driver.time.sleep'):
        assert d.return_to_open(safety_check=check, force=True)
    assert events and events == ['check', 'send']*(len(events)//2)
    assert not d.is_enabled()


def test_complete_arrival_still_requires_safety_during_verification(hand):
    d, _ = hand
    d.return_to_open = Mock(return_value=True)
    d.open_pose_status = Mock(return_value=(True, {}))
    assert not d.align_and_verify_open_pose(safety_check=lambda: False, force=True)[0]
    d.open_pose_status.assert_not_called()
