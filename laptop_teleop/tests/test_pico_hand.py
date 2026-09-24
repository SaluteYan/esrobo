import copy
import json
import time
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from esrobo_laptop.acquisition import annotate
from esrobo_laptop.config import ROOT, read_settings, read_robot_config
from esrobo_laptop.inputs import FreshBodyDevice, InputUnavailable
from esrobo_laptop.mapping import hand_units
from esrobo_laptop.pico_hand import PicoHands, retarget_hand


def skeleton(curl=0.):
    poses = np.zeros((26, 7))
    poses[:, 6] = 1
    poses[0, :3] = [0, .04, 0]
    for start, x in ((6, .03), (11, 0.), (16, -.02), (21, -.04)):
        poses[start, :3] = [x, .025, 0]
        for offset in range(1, 5):
            a = curl * max(0, offset-1)
            poses[start+offset, :3] = poses[start+offset-1, :3] + [0, .025*np.cos(a), .025*np.sin(a)]
    for i in range(4):
        poses[2+i, :3] = [.02+.02*i, .015+.01*i, 0]
    return poses


def test_finger_curl_is_bounded_and_pose_independent():
    opened, closed = skeleton(), skeleton(np.pi/3)
    a, b = np.array(retarget_hand(opened)), np.array(retarget_hand(closed))
    assert np.allclose(a[[4, 5, 7, 9]], 0)
    assert np.all(b[[4, 5, 7, 9]] > .5)
    rotated = closed.copy()
    rotated[:, :3] = closed[:, :3] @ Rotation.from_euler('xyz', [.2, -.8, .6]).as_matrix().T + [1, 2, 3]
    assert np.allclose(retarget_hand(rotated), b)
    closed[:, 0] *= -1
    assert np.allclose(retarget_hand(closed), b)
    with pytest.raises(ValueError):
        retarget_hand(np.zeros((26, 7)))


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('finger_start,active_index,physical_index,other_active,other_physical', [
    (16, 7, 4, 9, 5),  # ring
    (21, 9, 5, 7, 4),  # pinky
])
def test_ring_and_pinky_follow_independent_pico_samples(
    side, finger_start, active_index, physical_index, other_active, other_physical,
):
    opened = skeleton()
    curled = skeleton(np.pi / 3)
    isolated = opened.copy()
    isolated[finger_start:finger_start + 5] = curled[finger_start:finger_start + 5]
    reference = np.asarray(retarget_hand(opened))
    angles = np.asarray(retarget_hand(isolated))
    assert angles[active_index] > reference[active_index] + .5
    assert angles[other_active] == pytest.approx(reference[other_active])

    config = read_robot_config(read_settings(ROOT / 'config/laptop.yaml'))
    feedback = np.asarray(getattr(config.hand, f'{side}_open_feedback'), dtype=float)
    target = hand_units(angles, feedback, config.hand, side, reference)
    opened_target = hand_units(reference, feedback, config.hand, side, reference)
    assert target[physical_index] < opened_target[physical_index]
    assert target[other_physical] == opened_target[other_physical]


def sample():
    return dict(sequence=1, active=True, poses=skeleton(), flags=[15]*26, timestamp_ns=100, age_s=.001)


def test_hand_cache_and_unrelated_xr_updates_never_renew_sample():
    left = sample()
    sdk = SimpleNamespace(get_left_hand_sample=lambda: left,
                          get_right_hand_sample=lambda: dict(left, active=False))
    reader = PicoHands()
    assert set(reader.poll(sdk)) == {'left'}
    assert reader.poll(sdk) == {}
    left['sequence'] = 2  # New transport packet, unchanged Hand timestamp.
    assert reader.poll(sdk) == {}
    left['timestamp_ns'] = 101
    left['sequence'] = 3
    assert set(reader.poll(sdk)) == {'left'}  # Static pose, real new sample.
    left['age_s'] = .2
    left['sequence'] = 4
    left['timestamp_ns'] = 102
    assert reader.poll(sdk) == {}
    left['age_s'] = 0
    left['flags'][8] = 0
    assert reader.poll(sdk) == {}


@pytest.fixture
def device():
    cfg = copy.deepcopy(read_robot_config(read_settings(ROOT/'config/laptop.yaml')).retarget)
    cfg.port = 0
    dev = FreshBodyDevice(cfg, ('left', 'right'), True)
    yield dev
    dev.close()


def packet(side, seq=1, stamp=None):
    return annotate(dict(pico_hands={side: dict(joints=[0.]*10, sequence=seq)}),
                    'pico_hand', {side: time.monotonic() if stamp is None else stamp}, [side])


def test_independent_sides_and_missing_required_inputs(device):
    device._handle_packet(packet('left'))
    assert set(device.stamps) == {'left_hand'}
    with pytest.raises(InputUnavailable, match='right_hand'):
        device.ticket(.1)
    before = dict(device.stamps)
    device._handle_packet(packet('left'))  # Replayed seq, new envelope time.
    assert device.stamps == before
    device._handle_packet(packet('right'))
    assert set(device.stamps) == {'left_hand', 'right_hand'}
    before = dict(device.stamps)
    device._handle_packet(packet('right', 2, time.monotonic()-1))
    assert device.stamps == before


def test_hand_only_requires_one_selected_pico_hand_without_body_reference():
    cfg = copy.deepcopy(read_robot_config(read_settings(ROOT/'config/laptop.yaml')).retarget)
    cfg.port = 0
    device = FreshBodyDevice(cfg, ('right',), True, hand_only=True)
    try:
        assert device.required == ('right_hand',)
        assert not device.is_ready()
        device._handle_packet(packet('right'))
        ticket = device.ticket(.1)
        assert set(ticket) == {'right_hand'}
        assert device.hand_joints().shape == (20,)
    finally:
        device.close()


def test_natural_open_reference_requires_stable_extended_fingers(device):
    now = time.monotonic()
    for side in ("left", "right"):
        device.hand_history[side].extend(
            (now - .3 + index * .01, np.zeros(10)) for index in range(20)
        )
    references, message = device.natural_open_hand_reference()
    assert set(references) == {"left", "right"}
    assert "自然张手" in message

    curled = np.zeros(10)
    curled[4] = 1.0
    device.hand_history["right"].clear()
    device.hand_history["right"].extend(
        (now - .3 + index * .01, curled) for index in range(20)
    )
    references, message = device.natural_open_hand_reference()
    assert references is None
    assert "尚未自然张开" in message


def test_hand_packets_cannot_change_body_or_inject_imu(device):
    msg = json.loads(packet('left'))
    msg.update(frames={'waist': {'pos': [999, 0, 0]}},
               hand_orientation_deltas={'left': [0, 1, 0, 0]})
    device._handle_packet(json.dumps(msg).encode())
    assert not device._frames
    assert not device._hand_orientation_delta_matrices


@pytest.mark.parametrize('side', ['left', 'right'])
def test_pico_wrist_zero_and_rotation_apply_to_both_sides(device, side):
    initial = Rotation.from_euler('xyz', [.3, -.2, .5]).as_matrix()
    initial_pose = np.eye(4)
    initial_pose[:3, :3] = initial
    setattr(device, f'_initial_{side}_pose_matrix', initial_pose)
    ref = Rotation.from_euler('xyz', [-.1, .5, -.2]).as_matrix()
    device._reference_wrist_rotations[side] = ref
    current = np.eye(4)
    current[:3, :3] = ref
    device._current_wrist_matrix = lambda _: (current, 'wrist')
    assert np.allclose(device._compose_wrist_rotation(side, None), initial)
    delta = Rotation.from_rotvec([0, 0, .02]).as_matrix()
    current[:3, :3] = delta @ ref
    assert np.allclose(device._compose_wrist_rotation(side, None), delta @ initial)
    current[:3, :3] = Rotation.from_rotvec([0, 0, 2.]).as_matrix() @ ref
    previous = device.wrist_rotations[side].copy()
    result = device._compose_wrist_rotation(side, None)
    assert Rotation.from_matrix(result @ previous.T).magnitude() <= np.deg2rad(2.01)
