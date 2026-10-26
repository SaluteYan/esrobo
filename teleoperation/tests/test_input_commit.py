"""Single-decode ownership and real-age admission after input-lock wait."""
import json
import numpy as np
from unittest.mock import patch
import threading
import pytest

from esrobo_teleop.device.body_device import BodyDevice
from test_laptop import body, config  # Reuse socket-owning fixtures.


def test_split_palm_is_atomic_with_body_and_late_fingers_cannot_overwrite(body):
    from scipy.spatial.transform import Rotation
    message = json.loads(packet(9.98))
    palm = Rotation.from_euler('x', .3).as_matrix()
    message.update(palm_publication_version=1, pico_palms={s:dict(
        sequence=20, sample_monotonic=9.98, palm_basis=palm.tolist()) for s in body.sides})
    original = BodyDevice._handle_message
    def apply(device, msg, **kwargs):
        assert device.input_lock._is_owned()
        for side in body.sides:
            assert device.palm_history[side][-1][2] == 20
        return original(device, msg, **kwargs)
    with patch('esrobo_laptop.inputs.time.monotonic', return_value=10.), patch.object(BodyDevice, '_handle_message', apply):
        body._handle_packet(json.dumps(message))
    side = body.sides[0]
    with patch('esrobo_laptop.inputs.time.monotonic', return_value=10.):
        body._handle_packet(hand(side, 9.96, 18))
    assert body.hand_sequences[side] == 18
    assert body.palm_history[side][-1][2] == 20
    np.testing.assert_array_equal(body.palm_history[side][-1][1], palm)


def test_split_fingers_do_not_need_or_publish_palm(body):
    side = body.sides[0]
    message = json.loads(hand(side, 9.98, 10))
    message['palm_publication_version'] = 1
    message['pico_hands'][side].pop('palm_basis')
    with patch('esrobo_laptop.inputs.time.monotonic', return_value=10.):
        body._handle_packet(json.dumps(message))
    assert body.hand_sequences[side] == 10
    assert not body.palm_history[side]


@pytest.mark.parametrize('stamp,sequence', [(9.8, 21), (9.97, 19)])
def test_stale_or_regressing_split_palms_never_renew_history(body, stamp, sequence):
    side = body.sides[0]
    body.palm_history[side].append((9.98, np.eye(3), 20))
    body.palm_sequences[side] = 20
    msg = json.loads(packet(9.99))
    msg.update(palm_publication_version=1, pico_palms={side:dict(
        sequence=sequence, sample_monotonic=stamp, palm_basis=np.eye(3).tolist())})
    with patch('esrobo_laptop.inputs.time.monotonic', return_value=10.):
        body._handle_packet(json.dumps(msg))
    assert body.palm_history[side][-1][0] == 9.98
    assert body.palm_sequences[side] == 20


def packet(stamp):
    frames={'waist':[0,0,1,1,0,0,0]}
    for side,y in [('left',.2),('right',-.2)]:
        for part,x in [('shoulder',0),('elbow',.25),('wrist',.5)]:
            frames[side+'_'+part]=[x,y,1.4,1,0,0,0]
    return json.dumps(dict(frames=frames,laptop_input=dict(v=1,kind='pico',
        physical_sides=['left','right'],sample_monotonic=dict(left=stamp,right=stamp))))


def test_decode_once_outside_lock_and_commit_inside(body):
    decode=body._decode_frames
    commit=BodyDevice._handle_message
    decoded,committed=[],[]
    def parse(message):
        assert not body.input_lock._is_owned()
        decoded.append(message)
        return decode(message)
    def apply(device,message,**kwargs):
        assert body.input_lock._is_owned()
        assert kwargs['parsed_frames']
        committed.append(message)
        return commit(device,message,**kwargs)
    with patch('esrobo_laptop.inputs.time.monotonic',return_value=10.), \
         patch.object(body,'_decode_frames',side_effect=parse), \
         patch.object(BodyDevice,'_handle_message',apply):
        body._handle_packet(packet(9.98))
    assert len(decoded)==len(committed)==1
    assert body.stamps['right_arm']==9.98
    assert body.stamps['right_wrist']==9.98


@pytest.mark.parametrize('sides,swap', [(('right',),False), (('left',),False),
                                     (('right',),True), (('left','right'),False)])
def test_control_parses_only_required_chains_without_changing_poses(body,sides,swap):
    body.sides=sides
    body._cfg.swap_left_right_targets=swap
    message=json.loads(packet(9.98))
    for name in ['head','neck','left_ankle','right_ankle','left_foot','right_foot']:
        message['frames'][name]=[0,0,1,1,0,0,0]
    all_frames=body._decode_frames(message)
    with patch('esrobo_laptop.inputs.time.monotonic',return_value=10.):
        kind, (filtered,parsed,updates,_) = body._decode_input(json.dumps(message))
    expected={'waist'} | {f'{s}_{part}' for s in set(sides)|{body._source_side_for_target(s) for s in sides}
                          for part in ('shoulder','elbow','wrist')}
    assert set(parsed)==set(filtered['frames'])==expected
    for name in expected:np.testing.assert_array_equal(parsed[name],all_frames[name])
    assert all(stamp==9.98 for stamp in updates.values())
    assert 'head' in message['frames']  # Do not mutate a full skeleton snapshot.


def test_expired_after_parse_or_lock_wait_never_committed(body):
    clock=[10.]
    decode=body._decode_frames
    def delayed(message):
        result=decode(message)
        clock[0]=10.2
        return result
    with patch('esrobo_laptop.inputs.time.monotonic',side_effect=lambda:clock[0]), \
         patch.object(body,'_decode_frames',side_effect=delayed), \
         patch.object(BodyDevice,'_handle_message') as commit:
        body._handle_packet(packet(9.98))
    commit.assert_not_called()
    assert not body.stamps
    assert not body.incoming_body_stamps
    assert body.last_rejection=='expired sensor timestamps before commit'


def hand(side, stamp, sequence, value=0.):
    return json.dumps(dict(laptop_input=dict(v=1,kind='pico_hand',
        physical_sides=[side],sample_monotonic={side:stamp}),pico_hands={side:dict(
            joints=[value]*10,sequence=sequence,palm_basis=np.eye(3).tolist())}))


def test_burst_keeps_latest_valid_body_and_each_hand(body):
    invalid=json.loads(packet(9.995));del invalid['frames']['right_wrist']
    with patch('esrobo_laptop.inputs.time.monotonic',return_value=10.):
        body._commit_batch([packet(9.95),hand('left',9.96,1),hand('right',9.96,1),
            packet(9.98),hand('left',9.98,2,.02),hand('right',9.99,2,.03),
            json.dumps(invalid),packet(9.97),hand('right',9.98,1),hand('left',9.99,3,100.)])
    assert body.stamps['right_arm']==9.98
    assert body.stamps['left_hand']==9.98
    assert body.stamps['right_hand']==9.99
    assert body.hand_sequences==dict(left=2,right=2)
    assert len(body.hand_history['left'])==len(body.hand_history['right'])==1
    assert len(body._body_frame_history)==1
    assert body.receive_counts['body_coalesced']==2
    assert body.receive_counts['hand_coalesced']==3
    assert body.receive_counts['rejected']==2


def test_burst_transforms_only_the_latest_valid_body(body):
    payloads = [packet(9.92+i*.001) for i in range(60)]
    invalid = json.loads(packet(9.999))
    invalid['frames']['right_wrist'] = [float('nan')]*7
    payloads.append(json.dumps(invalid))
    with patch('esrobo_laptop.inputs.time.monotonic',return_value=10.), \
         patch.object(body,'_decode_frames',wraps=body._decode_frames) as decode:
        body._commit_batch(payloads)
    assert decode.call_count == 1
    assert body.stamps['right_arm'] == pytest.approx(9.979)
    assert body.receive_counts['body_coalesced'] == 59
    assert body.receive_counts['rejected'] == 1


def test_wrist_filter_capture_is_transactional(body):
    from esrobo_laptop.orientation import AbsoluteHandOrientation, ForearmFrame
    from scipy.spatial.transform import Rotation
    from test_absolute_hand import points
    arm=points();carrier=ForearmFrame().update(arm)
    desired=Rotation.from_euler('z',30,degrees=True).as_matrix()
    reference=AbsoluteHandOrientation(arm,carrier,palm_rotation=carrier@desired)
    reference.reset_filter(np.eye(3))
    reference.filter_stamp = 9.96
    body.wrist_references={'right':reference}
    body.wrist_rotations={'right':np.eye(3)}
    def advance(view,**kwargs):
        view.wrist_rotations['right']=view.wrist_references['right'].update(
            arm,carrier,palm_rotation=carrier@desired,sample_time=view.stamps['right_arm'])
        return {}
    with patch('esrobo_laptop.inputs.time.monotonic',return_value=10.):
        body._commit_batch([packet(9.98),hand('left',9.98,1),hand('right',9.98,1)])
        body._reference_locked=True
        with patch.object(BodyDevice,'advance',advance):
            a=body.capture_targets(.1); b=body.capture_targets(.1)
            np.testing.assert_allclose(a[1]['hand_local_orientations']['right'],
                                       b[1]['hand_local_orientations']['right'])
            np.testing.assert_array_equal(reference.filtered,np.eye(3))
            np.testing.assert_allclose(b[1]['hand_source_orientations']['right'],desired,atol=1e-12)
            body.commit_wrist_filters(b[1],[])  # Position/input hold cannot commit.
            np.testing.assert_array_equal(reference.filtered,np.eye(3))
            body.commit_wrist_filters(b[1],['right'])
            expected = 30*(1-np.exp(-2*np.pi*8*.02))-.25
            assert np.degrees(Rotation.from_matrix(body.wrist_references['right'].filtered).magnitude()) == pytest.approx(expected)
            assert reference.filter_stamp == 9.96
            np.testing.assert_array_equal(reference.smoothed,np.eye(3))
            c=body.capture_targets(.1)
            # Capturing the same source again must not filter it twice.
            assert np.degrees(Rotation.from_matrix(c[1]['hand_local_orientations']['right']).magnitude()) == pytest.approx(expected)


def test_hand_expiring_while_waiting_for_commit_is_rejected(body):
    with patch('esrobo_laptop.inputs.time.monotonic',return_value=10.):
        prepared=body._decode_input(hand('right',9.98,1))
    with patch('esrobo_laptop.inputs.time.monotonic',return_value=10.2):
        body._commit_input(prepared)
    assert not body.hand_sequences and not body.stamps
    assert 'expired PICO Hand before commit' in body.last_rejection


def test_capture_copies_only_same_nearest_palm_and_preserves_receiver_history(body):
    with patch('esrobo_laptop.inputs.time.monotonic',return_value=10.):
        body._commit_batch([packet(9.98),hand('left',9.98,1),hand('right',9.98,1)])
        body._reference_locked=True
        for side in body.sides:
            body.palm_history[side].clear()
            for i in range(120):body.palm_history[side].append((8.8+i*.01,np.eye(3),i))
        nearest={s:min(h,key=lambda item:abs(item[0]-body.stamps[f'{s}_arm']))
                 for s,h in body.palm_history.items()}
        def calculate(view,**kwargs):
            assert not body.input_lock._is_owned()
            for side in body.sides:
                assert len(view.palm_history[side])==1
                stamp,palm,seq=view.palm_history[side][0]
                expected=nearest[side]
                assert (stamp,seq)==(expected[0],expected[2])
                np.testing.assert_array_equal(palm,expected[1])
                assert palm is not expected[1]
                assert len(body.palm_history[side])==120
            return {s+'_'+p:np.zeros(7) for s in ('left','right') for p in ('elbow','wrist')}
        with patch.object(BodyDevice,'advance',calculate):body.capture_targets(.1)


def test_retarget_snapshot_runs_outside_lock_and_does_not_overwrite_new_input(body):
    with patch('esrobo_laptop.inputs.time.monotonic',return_value=10.):
        body._commit_batch([packet(9.98),hand('left',9.98,1),hand('right',9.98,1)])
        body._reference_locked=True
        def calculate(view,**kwargs):
            assert not body.input_lock._is_owned()
            assert view is not body
            assert view._frames['right_wrist'] is not body._frames['right_wrist']
            body._handle_packet(packet(9.99))
            assert view.stamps['right_arm']==9.98
            view._previous_right_pose=np.full(7,42.)
            return {s+'_'+p:np.zeros(7) for s in ('left','right') for p in ('elbow','wrist')}
        with patch.object(BodyDevice,'advance',calculate):
            ticket,poses,hands=body.capture_targets(.1)
    assert ticket['right_arm']==9.98
    assert body.stamps['right_arm']==9.99
    assert body._previous_right_pose.tolist()==[42.]*7
    assert hands.shape==(20,)


@pytest.mark.parametrize('mode', ['ACTIVE', 'WAITING_FOR_ACTIVE_TARGET'])
def test_controller_capture_allows_receiver_commit_during_retarget(body, mode):
    from test_entry_keepalive import fixed_controller
    controller, pipeline, client, state = fixed_controller(mode)
    controller.mode = mode
    pipeline.body = body
    pipeline.capture.side_effect = lambda **kw: body.capture_targets(.1, **kw)
    completed = threading.Event()
    with patch('esrobo_laptop.inputs.time.monotonic', return_value=10.):
        client.received_at = 10.
        body._commit_batch([packet(9.98), hand('left',9.98,1), hand('right',9.98,1)])
        body._reference_locked = True
        def advance(view, **kw):
            assert not body.input_lock._is_owned()
            def receive():
                body._handle_packet(packet(9.99))
                completed.set()
            thread = threading.Thread(target=receive, daemon=True)
            thread.start()
            assert completed.wait(.5), 'Controller kept an outer input lock'
            thread.join()
            assert view.stamps['right_arm'] == 9.98
            return {s+'_'+part:np.zeros(7) for s in ('left','right') for part in ('elbow','wrist')}
        with patch.object(BodyDevice, 'advance', advance):
            controller.step(state)
    assert body.stamps['right_arm'] == 9.99
    assert body.used['right_arm'] == 9.98


def test_entry_source_uses_captured_body_not_a_later_receiver_frame(body):
    from esrobo_laptop.pipeline import Pipeline
    from esrobo_laptop.config import Settings
    with patch('esrobo_laptop.inputs.time.monotonic', return_value=10.):
        body._commit_batch([packet(9.98), hand('left',9.98,1), hand('right',9.98,1)])
        body._reference_locked = True
        with patch.object(BodyDevice, 'advance', return_value={}):
            frame = body.capture_targets(.1)
        expected = list(frame[1]['entry_sources']['right'])
        newer = json.loads(packet(9.99))
        newer['frames']['right_wrist'][0] += .1
        body._handle_packet(json.dumps(newer))
    pipeline = Pipeline.__new__(Pipeline)
    pipeline.settings, pipeline.body = Settings(side='right'), body
    assert pipeline.entry_source(frame) == expected
    assert pipeline.entry_source() != expected


def test_post_compute_age_includes_the_actual_paired_palm(body):
    from esrobo_laptop.pipeline import Pipeline
    from esrobo_laptop.config import Settings
    from esrobo_laptop.inputs import SourceExpired
    with patch('esrobo_laptop.inputs.time.monotonic',return_value=10.):
        body._commit_batch([packet(9.93),hand('left',9.98,1),hand('right',9.98,1)])
        body._reference_locked=True
        def advance(view,**kwargs):
            view.wrist_input_diagnostics['right']={'comparison_hand_sample_monotonic':9.92}
            return {}
        with patch.object(BodyDevice,'advance',advance): frame=body.capture_targets(.1)
    assert frame[0]['right_hand']==9.98 and frame[0]['right_palm']==9.92
    pipeline=Pipeline.__new__(Pipeline);pipeline.settings=Settings(side='right')
    with patch('esrobo_laptop.pipeline.time.monotonic',return_value=10.025):
        with pytest.raises(SourceExpired):pipeline.check_ticket(frame[0])


@pytest.mark.parametrize('sides', [('left',), ('right',), ('left', 'right')])
def test_new_arm_reuses_fresh_fingers_without_renewing_source_identity(body, sides):
    from esrobo_laptop.inputs import SourceExpired
    body.sides = sides
    body.with_hand = True
    body.hand_only = False
    body.is_ready = lambda: True
    body.stamps = {k: 9.94 for k in body.required}
    body.hand_sequences = {s: 12 for s in sides}
    with patch('esrobo_laptop.inputs.time.monotonic', return_value=10.):
        first = body.ticket(.1)
        body.consume(first)
        for side in sides:
            body.stamps[side+'_arm'] = body.stamps[side+'_wrist'] = 9.99
        second = body.ticket(.1)
        assert second is not None
        for side in sides:
            assert second[side+'_hand'] == 9.94
            assert second.hand_sequences[side] == 12
        body.consume(second)
        assert body.ticket(.1) is None  # Reuse cannot replay an old Body frame.
    # Even continuously arriving arms cannot renew the hand's validity window.
    with patch('esrobo_laptop.inputs.time.monotonic', return_value=10.05):
        for side in sides:
            body.stamps[side+'_arm'] = body.stamps[side+'_wrist'] = 10.04
        with pytest.raises(SourceExpired, match='_hand'):
            body.ticket(.1)
    assert all(body.stamps[s+'_hand'] == 9.94 for s in sides)


def test_hand_only_still_requires_new_hand_and_failed_candidate_is_not_consumed(body):
    body.sides = ('left',)
    body.with_hand = body.hand_only = True
    body.stamps = {'left_hand': 9.98}
    body.hand_sequences = {'left': 7}
    with patch('esrobo_laptop.inputs.time.monotonic', return_value=10.):
        candidate = body.ticket(.1)
        assert body.ticket(.1) == candidate  # Failed send never called consume.
        body.consume(candidate)
        assert body.ticket(.1) is None
        body.stamps['left_hand'] = 9.99
        body.hand_sequences['left'] = 8
        assert body.ticket(.1).hand_sequences['left'] == 8


def test_reused_hand_can_expire_during_ik(body):
    from esrobo_laptop.pipeline import Pipeline
    from esrobo_laptop.config import Settings
    from esrobo_laptop.inputs import SourceExpired
    body.sides = ('left',)
    body.with_hand, body.hand_only = True, False
    body.is_ready = lambda: True
    body.stamps = dict(left_arm=9.99, left_wrist=9.99, left_hand=9.92)
    body.used = dict(left_arm=9.97, left_wrist=9.97, left_hand=9.92)
    with patch('esrobo_laptop.inputs.time.monotonic', return_value=10.):
        candidate = body.ticket(.1)
    pipeline = Pipeline.__new__(Pipeline)
    pipeline.settings = Settings(side='left')
    with patch('esrobo_laptop.pipeline.time.monotonic', return_value=10.025):
        with pytest.raises(SourceExpired):
            pipeline.check_ticket(candidate)
    assert body.used['left_arm'] == 9.97
