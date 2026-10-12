"""Single-decode ownership and real-age admission after input-lock wait."""
import json
import numpy as np
from unittest.mock import patch
import threading
import pytest

from esrobo_teleop.device.body_device import BodyDevice
from test_laptop import body, config  # Reuse socket-owning fixtures.


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
