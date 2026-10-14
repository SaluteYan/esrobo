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
from esrobo_laptop.pico_hand import PicoHands, retarget_hand, palm_basis


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


@pytest.mark.parametrize("mirror", [1., -1.])
def test_palm_landmarks_define_right_handed_world_vectors(mirror):
    poses = skeleton()
    poses[:, 0] *= mirror
    basis = palm_basis(poses)
    assert np.linalg.det(basis) == pytest.approx(1.)
    np.testing.assert_allclose(basis[:, 2], [0., 1., 0.])
    np.testing.assert_allclose(basis[:, 1], [mirror, 0., 0.])
    whole = Rotation.from_euler("xyz", [40., -25., 12.], degrees=True).as_matrix()
    poses[:, :3] = poses[:, :3] @ whole.T + [3., -2., 1.]
    np.testing.assert_allclose(palm_basis(poses), whole @ basis, atol=1e-7)
    poses[22, :3] = poses[7, :3]
    with pytest.raises(ValueError):
        palm_basis(poses)


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


def test_static_complete_hand_packets_advance_without_pose_change(monkeypatch):
    current = dict(sample(), timestamp_ns=0, packet_timestamp_ns=100,
                   freshness_source='component-packet')
    sdk = SimpleNamespace(get_left_hand_sample=lambda: current,
                          get_right_hand_sample=lambda: dict(current, active=False))
    monkeypatch.setattr('esrobo_laptop.pico_hand.time.monotonic', lambda: 10.)
    reader = PicoHands()
    assert reader.poll(sdk)['left']['sample_monotonic'] == pytest.approx(9.999)
    assert reader.poll(sdk) == {}
    current.update(sequence=2, packet_timestamp_ns=101)
    assert 'left' in reader.poll(sdk)
    current.update(sequence=3, packet_timestamp_ns=102, age_s=.101)
    assert reader.poll(sdk) == {}


@pytest.fixture
def device():
    cfg = copy.deepcopy(read_robot_config(read_settings(ROOT/'config/laptop.yaml')).retarget)
    cfg.port = 0
    dev = FreshBodyDevice(cfg, ('left', 'right'), True)
    yield dev
    dev.close()


def packet(side, seq=1, stamp=None):
    return annotate(dict(pico_hands={side: dict(joints=[0.]*10, sequence=seq, palm_basis=np.eye(3).tolist())}),
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


@pytest.mark.parametrize("palm", [None, np.zeros((3, 3)).tolist(), np.diag([1., 1., -1.]).tolist()])
def test_arm_hand_packets_need_valid_anatomical_palm_without_fallback(device, palm):
    msg = json.loads(packet("right"))
    if palm is None:
        del msg["pico_hands"]["right"]["palm_basis"]
    else:
        msg["pico_hands"]["right"]["palm_basis"] = palm
    device._handle_packet(json.dumps(msg).encode())
    assert not device.stamps
    assert not device.palm_history["right"]


def test_palm_calibration_accepts_single_frame_and_checks_pair_age_transform_reset(device):
    from esrobo_teleop import math_utils as mu
    now = time.monotonic()
    # Body's transformed forearm is X, palm normal is Z.
    points = dict(shoulder=np.array([0., 1., 0.]), elbow=np.zeros(3), wrist=np.array([1., 0., 0.]))
    device._current_arm_points = lambda _: points
    device._cfg.auto_start_reference_require_waist = True
    waist = Rotation.from_euler("y", 20., degrees=True).as_matrix()
    device._frames["waist"] = np.r_[np.zeros(3), mu.matrix_to_quat_wxyz(waist)]
    desired = Rotation.from_euler("y", 90., degrees=True).as_matrix()
    raw = device._rotation.T @ waist @ desired
    device.incoming_body_stamps = {"right_arm": now-.005}
    device.palm_history["right"].append((now-.01, raw.copy(), 1))
    transformed, stamp, seq = device._calibration_palm("right", now)
    np.testing.assert_allclose(transformed, desired, atol=1e-6)
    assert abs(stamp-device.incoming_body_stamps["right_arm"]) <= .04
    device.incoming_body_stamps["right_arm"] = now+.06
    with pytest.raises(ValueError, match="不同步"):
        device._calibration_palm("right", now)
    device.incoming_body_stamps["right_arm"] = now-.005
    device.palm_history["right"].append((now-.001, raw @ Rotation.from_euler("x", 10., degrees=True).as_matrix(), 21))
    device._calibration_palm("right", now)  # Moving history is not a rejection.
    device.incoming_body_stamps["right_arm"] = now-.2
    with pytest.raises(ValueError, match="不同步"):
        device._calibration_palm("right", now)
    device.restart_preparation()
    assert not device.palm_history["right"]
    with pytest.raises(ValueError, match="有效掌面"):
        device._calibration_palm("right", now)


def test_absolute_palm_diagnostics_are_frozen_with_control_frame(device):
    from esrobo_teleop import math_utils as mu
    from esrobo_laptop.orientation import AbsoluteHandOrientation, ForearmFrame, ANATOMICAL_NEUTRAL
    from esrobo_laptop.inputs import SourceExpired
    now=time.monotonic()
    points=dict(shoulder=np.array([0., 1., 0.]), elbow=np.zeros(3), wrist=np.array([1., 0., 0.]))
    carrier=ForearmFrame().update(points)
    palm_reference=carrier @ ANATOMICAL_NEUTRAL
    device.wrist_references['right']=AbsoluteHandOrientation(points,np.eye(3),palm_rotation=palm_reference)
    device._current_arm_points=lambda _: points
    current=np.eye(4);current[:3,:3]=Rotation.from_euler('x',35.,degrees=True).as_matrix()
    device._current_wrist_matrix=lambda _: (current,'wrist')
    waist=Rotation.from_euler('xyz',[23.,-17.,6.],degrees=True).as_matrix()
    device._cfg.auto_start_reference_require_waist=True
    device._frames['waist']=np.r_[np.zeros(3),mu.matrix_to_quat_wxyz(waist)]
    device.stamps['right_arm']=now-.01
    # Hand gives 50 degrees while the diagnostic Body gives only 35.
    palm=Rotation.from_euler('x',50.,degrees=True).as_matrix() @ palm_reference
    raw=device._rotation.T @ waist @ palm
    device.palm_history['right'].append((now-.012,raw,51))
    device._compose_wrist_rotation('right',None,full_target=True)
    np.testing.assert_allclose(device.wrist_rotations['right'],carrier.T @ palm,atol=1e-6)
    held=device.wrist_rotations['right'].copy()
    device._frames['waist'][3:]=mu.matrix_to_quat_wxyz(Rotation.from_euler('z',90.,degrees=True).as_matrix())
    values=device.wrist_diagnostics(now)['right']
    assert values['orientation_mode']=='forearm_absolute'
    assert values['requested_roll_deg']==pytest.approx(50.,abs=1e-5)
    assert values['body_absolute_roll_deg']==pytest.approx(35.,abs=1e-5)
    assert values['body_palm_disagreement_deg']==pytest.approx(15.,abs=1e-5)
    assert values['palm_pair_gap_ms']==pytest.approx(2.)
    np.testing.assert_array_equal(device.wrist_rotations['right'],held)
    assert device.wrist_diagnostics(now+.2)['right']['palm_comparison_status']=='unavailable'
    device.palm_history['right'].clear()
    device.palm_history['right'].append((now-.08,raw,52))
    with pytest.raises(SourceExpired,match='不同步'):
        device._compose_wrist_rotation('right',None)


@pytest.mark.parametrize('side', ['left', 'right'])
def test_hand_only_requires_one_selected_pico_hand_without_body_reference(side):
    cfg = copy.deepcopy(read_robot_config(read_settings(ROOT/'config/laptop.yaml')).retarget)
    cfg.port = 0
    device = FreshBodyDevice(cfg, (side,), True, hand_only=True)
    try:
        assert device.required == (f'{side}_hand',)
        assert not device.is_ready()
        device.restart_preparation()
        device._handle_packet(packet(side))
        ticket = device.ticket(.1)
        assert set(ticket) == {f'{side}_hand'}
        assert device.hand_joints().shape == (20,)
        assert set(device.initial_hand_reference()[0]) == {side}
    finally:
        device.close()


@pytest.mark.parametrize('invalid', ['stale', 'future', 'nan', 'shape', 'missing'])
def test_initial_hand_reference_accepts_one_curled_sample_and_rejects_invalid(device, invalid):
    now = time.monotonic()
    curled = np.ones(10)
    for side in device.sides:
        device.hand_history[side].append((now, curled.copy()))
    references, message = device.initial_hand_reference()
    assert set(references) == {"left", "right"}
    np.testing.assert_array_equal(references['right'], curled)
    references['right'][0] = 99
    assert device.hand_history['right'][-1][1][0] == 1
    device.hand_history['right'].clear()
    cases = dict(stale=(now-.2, curled), future=(now+1, curled),
                 nan=(now, np.full(10, np.nan)), shape=(now, np.zeros(9)))
    if invalid != 'missing':
        device.hand_history['right'].append(cases[invalid])
    assert device.initial_hand_reference()[0] is None
    device.restart_preparation()
    assert device.initial_hand_reference()[0] is None


def test_hand_packets_cannot_change_body_or_inject_imu(device):
    msg = json.loads(packet('left'))
    msg.update(frames={'waist': {'pos': [999, 0, 0]}},
               hand_orientation_deltas={'left': [0, 1, 0, 0]})
    device._handle_packet(json.dumps(msg).encode())
    assert not device._frames
    assert not device._hand_orientation_delta_matrices


@pytest.mark.parametrize('side', ['left', 'right'])
def test_absolute_palm_targets_and_step_filter_apply_to_both_sides(device,side):
    from esrobo_teleop import math_utils as mu
    from esrobo_laptop.orientation import AbsoluteHandOrientation, ForearmFrame, ANATOMICAL_NEUTRAL
    points=dict(shoulder=np.array([0.,1.,0.]),elbow=np.zeros(3),wrist=np.array([1.,0.,0.]))
    carrier=ForearmFrame().update(points)
    initial=Rotation.from_euler('x',-120.,degrees=True).as_matrix() @ ANATOMICAL_NEUTRAL
    device.wrist_references[side]=AbsoluteHandOrientation(points,np.eye(3),palm_rotation=carrier @ initial)
    device._current_arm_points=lambda _:points
    device._current_wrist_matrix=lambda _:(np.eye(4),'wrist')
    device._frames['waist']=np.r_[np.zeros(3),mu.matrix_to_quat_wxyz(np.eye(3))]
    def sample(angle):
        now=time.monotonic();device.stamps[f'{side}_arm']=now
        local=Rotation.from_euler('x',angle,degrees=True).as_matrix() @ ANATOMICAL_NEUTRAL
        device.palm_history[side].append((now,device._rotation.T @ carrier @ local,1))
        return local
    local=sample(-120.)
    device._compose_wrist_rotation(side,None,full_target=True)
    np.testing.assert_allclose(device.wrist_rotations[side],local,atol=1e-6)
    local=sample(100.)
    previous=device.wrist_references[side].filtered.copy()
    device._compose_wrist_rotation(side,None)
    result=device.wrist_rotations[side]
    assert Rotation.from_matrix(result @ previous.T).magnitude() <= np.deg2rad(2.01)
    device._compose_wrist_rotation(side,None,full_target=True)
    np.testing.assert_allclose(device.wrist_rotations[side],local,atol=1e-6)

@pytest.mark.parametrize('sides', [('left',), ('right',), ('left', 'right')])
@pytest.mark.parametrize('with_hand', [False, True])
def test_initial_reference_locks_one_body_frame_atomically(device, sides, with_hand):
    from unittest.mock import Mock
    from esrobo_laptop.orientation import ForearmFrame
    now = time.monotonic()
    device.sides, device.with_hand = sides, with_hand
    device.restart_preparation()
    points = dict(shoulder=np.array([0., 1., 0.]), elbow=np.zeros(3), wrist=np.array([1., 0., 0.]))
    device._current_arm_points = lambda _: points
    device._current_wrist_matrix = lambda _: (np.eye(4), None)
    device._record_reference_diagnostic = Mock()
    carrier = ForearmFrame().update(points)
    device._calibration_palm = Mock(return_value=(carrier, now, 1))
    device.incoming_body_stamps = {f'{s}_arm': now for s in sides}
    # No elapsed countdown, body sample window, or palm history is required.
    if len(sides) == 2:
        device.incoming_body_stamps['right_arm'] = now-.2
        device._update_auto_reference(now)
        assert not device._reference_locked and not device.wrist_references
        assert device._reference_left_points is None
        device.incoming_body_stamps['right_arm'] = now
    device._update_auto_reference(now)
    assert device._reference_locked
    assert set(device.wrist_references) == (set(sides) if with_hand else set())
    for side in sides:
        np.testing.assert_array_equal(getattr(device, f'_reference_{side}_points')['wrist'], points['wrist'])
    points['wrist'][0] = 2
    device._update_auto_reference(now+.01)
    for side in sides:
        assert getattr(device, f'_reference_{side}_points')['wrist'][0] == 1
    assert device._record_reference_diagnostic.call_count == len(sides)


def test_invalid_palm_does_not_partially_lock_reference(device):
    from unittest.mock import Mock
    device.restart_preparation()
    now = time.monotonic()
    device.incoming_body_stamps = {f'{s}_arm': now for s in device.sides}
    device._current_arm_points = lambda _: dict(shoulder=np.zeros(3), elbow=np.ones(3), wrist=np.ones(3)*2)
    device._current_wrist_matrix = lambda _: (np.eye(4), None)
    device._calibration_palm = Mock(side_effect=ValueError('invalid palm'))
    device._update_auto_reference(now)
    assert not device._reference_locked and not device.wrist_references
    assert device._reference_left_points is None
