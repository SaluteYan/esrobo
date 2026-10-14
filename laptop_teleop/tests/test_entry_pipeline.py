import threading
import time
from unittest.mock import Mock
import numpy as np
import pytest
from esrobo_laptop.app import Controller
from esrobo_laptop.config import Settings
from esrobo_laptop.inputs import InputUnavailable
from esrobo_laptop.pipeline import Pipeline
from esrobo_link.backends import MockBackend
from esrobo_teleop.config import RobotConfig


def controller_fixture():
    backend=MockBackend('right',False)
    backend.contract['entry_protocol']=6
    client=Mock(received_at=time.monotonic())
    pipeline=Mock(initialized=True)
    pipeline.cfg.robot = RobotConfig()
    pipeline.body.input_lock=threading.RLock()
    pipeline.body.ticket.return_value={'right_arm':time.monotonic()}
    pipeline.capture.return_value=({'right_arm':time.monotonic()}, {}, None)
    pipeline.capture_calibrated_entry.return_value=({'right_arm':time.monotonic()}, {}, None)
    pipeline.calibrated_entry_source.return_value=[0.]*9
    pipeline.entry_source.return_value=[0.]*9
    pipeline.solve.return_value={'right':dict(arm_urdf_rad=[.2]*7,hand_unit=None,hold=False)}
    controller=Controller(client,pipeline,Settings(side='right',with_hand=False),backend.contract)
    controller.preparing=True
    controller.entry_seed=np.zeros(14)
    state=dict(mode='CALIBRATING',contract=backend.contract,feedback=backend.snapshot(),feedback_cache_age_s=0.)
    return controller,pipeline,client,state


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('with_hand', [False, True])
def test_calibrated_entry_uses_measured_j6_j7_and_holds_hand_feedback(side, with_hand):
    pipeline = Pipeline.__new__(Pipeline)
    pipeline.settings = Settings(side=side, with_hand=with_hand)
    pipeline._entry_limits = {side: (np.full(7, -2.), np.full(7, 2.))}
    pipeline.solvers = {side: Mock(last_solution_valid=True, solution_diagnostics={})}
    offset = 0 if side == 'left' else 7
    measured = np.zeros(14)
    measured[offset+5:offset+7] = [.17, -.13]
    q = measured.copy()
    q[offset+1], q[offset+3] = -.2, .3
    pipeline.solvers[side].solve.return_value = q
    pipeline.wrists = {}
    pipeline.position_limit_holds = {side: False}
    pipeline.position_limit_approaching = {side: False}
    pipeline.position_limit_recovery = {side: 0}
    pipeline.hand_references = {}
    pipeline.check_ticket = Mock()
    held = [142, 127, 254, 254, 254, 254, 110, 109, 120, 112]
    feedback = {side: {'hand': {'position_unit': held}}}
    poses = {name: np.zeros(7) for name in ('left_wrist','right_wrist','left_elbow','right_elbow')}
    result = pipeline.solve(({}, poses, np.zeros(20)), measured, feedback,
                            hold_entry_wrist_and_hand=True)
    assert result[side]['hand_unit'] == (held if with_hand else None)
    assert result[side]['arm_urdf_rad'][5:] == [.17, -.13]
    np.testing.assert_allclose(
        pipeline.solvers[side].solve.call_args.kwargs['terminal_joint_targets'][1:],
        [.17, -.13])


@pytest.mark.parametrize('side', ['left', 'right'])
def test_absolute_entry_holds_wrist_then_live_mapping_releases_all_axes(side):
    p = Pipeline.__new__(Pipeline)
    p.settings = Settings(side=side, with_hand=True)
    p._entry_limits = {side: (np.full(7, -2.), np.full(7, 2.))}
    offset = 0 if side == 'left' else 7
    measured = np.zeros(14)
    measured[offset+5:offset+7] = [.17, -.13]
    original = measured.copy()
    solver = Mock(last_solution_valid=True, solution_diagnostics={})
    solver.solve.return_value = measured.copy()
    wrist = Mock(reference_terminal=np.array([.2, .4, -.5]), absolute_pose=True)
    wrist.absolute_terminal.return_value = np.array([.3, .4, -.5])
    wrist.last_diagnostics = {'desired_world_quat_wxyz': [1., 0., 0., 0.]}
    p.solvers, p.wrists = {side: solver}, {side: wrist}
    p.position_limit_holds = {side: False}
    p.position_limit_approaching = {side: False}
    p.position_limit_recovery = {side: 0}
    p.check_ticket = Mock()
    p.map_hand_target = Mock(return_value=[70]*10)
    poses = {name: np.zeros(7) for name in ('left_wrist','right_wrist','left_elbow','right_elbow')}
    poses['hand_local_orientations'] = {side: np.eye(3)}
    frame, feedback = ({}, poses, np.zeros(20)), {side: {'hand': {'position_unit': [128]*10}}}
    entry = p.solve(frame, measured, feedback, hold_entry_wrist_and_hand=True)[side]
    np.testing.assert_allclose(entry['arm_urdf_rad'][4:], [.3, .17, -.13])
    assert entry['hand_unit'] == [128]*10
    p.map_hand_target.assert_not_called()
    np.testing.assert_array_equal(measured, original)
    np.testing.assert_allclose(solver.solve.call_args.kwargs['terminal_joint_targets'][1:], [.17, -.13])
    live = p.solve(frame, measured, feedback)[side]
    np.testing.assert_allclose(live['arm_urdf_rad'][4:], [.3, .4, -.5])
    assert live['hand_unit'] == [70]*10
    assert not wrist.last_diagnostics['entry_wrist_held']
    from scipy.spatial.transform import Rotation
    poses['hand_source_orientations']={side:Rotation.from_euler('z',20,degrees=True).as_matrix()}
    wrist.source_terminal.return_value=np.array([.35,.6,-.7])
    live=p.solve(frame,measured,feedback)[side]
    assert live['arm_urdf_rad'][4:]==[.3,.4,-.5]
    assert live['source_arm_urdf_rad'][4:]==[.35,.6,-.7]
    wrist.source_terminal.reset_mock()
    poses['hand_source_orientations']={side:np.eye(3)}
    live=p.solve(frame,measured,feedback)[side]
    wrist.source_terminal.assert_not_called()
    assert live['source_arm_urdf_rad']==live['arm_urdf_rad']


def test_full_target_and_source_sent_until_arrival_without_rebasing():
    c,p,client,state=controller_fixture()
    for mode in ('CALIBRATING','CHECKING_ENTRY','APPROACHING'):
        state['mode']=mode
        c.step(state)
        if mode == 'CALIBRATING':
            p.capture_calibrated_entry.assert_called_once()
        else:
            p.capture.assert_called_with(full_target=True)
        assert client.send_target.call_args.kwargs['arm_urdf_rad']==[.2]*7
        assert client.send_target.call_args.kwargs['entry']['phase']=='candidate'
        assert client.send_target.call_args.kwargs['entry']['reference_source']==[0.]*9
    p.initialize.assert_not_called()
    state.update(mode='WAITING_FOR_ACTIVE_TARGET',entry_token='arrival-token')
    state['feedback']['arm_urdf_rad']=[.2]*7
    c.step(state)
    assert client.send_target.call_args.kwargs['entry']['phase']=='ready'
    state['mode']='ACTIVE'
    c.step(state)
    p.initialize.assert_not_called()
    p.reset_command_start.assert_called_once()
    assert client.send_target.call_args.kwargs['entry']==dict(phase='active',token='arrival-token',source=None,
                                                              reference_source=None)


def test_discontinuous_ready_target_never_acknowledges_or_sends():
    c,p,client,state=controller_fixture()
    c.entry_target={'right':dict(arm_urdf_rad=[.2]*7,hand_unit=None,hold=False)}
    c.entry_reference_source=[0.]*9
    state.update(mode='WAITING_FOR_ACTIVE_TARGET',entry_token='arrival-token')
    with pytest.raises(InputUnavailable,match='尚未到位'):
        c.step(state)
    client.send_target.assert_not_called()
    p.initialize.assert_not_called()


def test_current_raw_source_is_sent_even_if_ik_goal_does_not_change():
    c,p,client,state=controller_fixture()
    c.entry_target={'right':dict(arm_urdf_rad=[.2]*7,hand_unit=None,hold=False)}
    c.entry_reference_source=[0.]*9
    state['mode']='APPROACHING'
    c.step(state)
    p.entry_source.return_value=[.1]+[0.]*8
    c.step(state)
    assert client.send_target.call_args.kwargs['entry']['source'][0]==.1
    assert client.send_target.call_args.kwargs['entry']['reference_source']==[0.]*9
    p.solve.assert_not_called()


@pytest.mark.parametrize('with_hand', [False, True])
@pytest.mark.parametrize('keepalive', [False, True])
def test_udp_single_arm_calibration_frozen_entry_and_fresh_activation(tmp_path, with_hand, keepalive):
    from esrobo_link.client import RobotClient
    from esrobo_link.gateway import Gateway
    from esrobo_laptop.config import ROOT,read_settings,read_robot_config
    from esrobo_laptop.pipeline import Pipeline
    from esrobo_laptop.demo import SyntheticSensors
    class Backend(MockBackend):
        def __init__(self):
            super().__init__('right',with_hand)
            self.contract['entry_protocol']=6
            if keepalive:
                self.contract['entry_keepalive_protocol']=1
        def approach_entry(self,target,cancelled,callback):
            assert not self.enabled
            # Exercise continuing source/lease supervision during planning.
            time.sleep(.25)
            assert not cancelled()
            callback()
            self.enabled=True
            if keepalive:
                time.sleep(.15)
                assert not cancelled()
            self.q=list(target['arm_urdf_rad'])
            return dict(returned=True,disabled=False,stage='handover_arrived')
        def activate_entry(self,target,cancelled):
            assert not cancelled()
    settings=read_settings(ROOT/'config/laptop.yaml')
    settings.side,settings.with_hand,settings.input_port='right',with_hand,0
    cfg=read_robot_config(settings)
    cfg.retarget.auto_start_reference_prepare_s=0
    cfg.retarget.auto_start_reference_delay_s=.12
    cfg.retarget.auto_start_reference_sample_start_s=0
    cfg.retarget.auto_start_reference_min_samples=5
    key=b'offline-handover-integration-test-only'*2
    path=tmp_path/'key';path.write_bytes(key)
    backend=Backend()
    gateway=Gateway(backend,key,port=0)
    pipeline=Pipeline(cfg,settings,{'right':backend.contract})
    pipeline.body._reference_diagnostic_path=None
    producer=SyntheticSensors(pipeline)
    worker=threading.Thread(target=gateway.run,daemon=True);worker.start()
    producer.thread.start()
    client=RobotClient('127.0.0.1',gateway.address[1],path)
    try:
        state=client.connect()
        controller=Controller(client,pipeline,settings,state['contract'])
        prepared=selected=moved=False
        modes=set()
        deadline=time.monotonic()+8
        while time.monotonic()<deadline:
            state=client.receive(.2)
            modes.add(state['mode'])
            try:
                targets=controller.step(state)
            except InputUnavailable:
                assert state['mode'] in ('IDLE','CALIBRATING')
                continue
            if keepalive and state['mode']=='WAITING_FOR_ACTIVE_TARGET' and producer.stop.is_set():
                assert targets is None  # Old/duplicate sensor data cannot acknowledge arrival.
                frames=producer.frames
                producer=SyntheticSensors(pipeline)
                producer.frames=frames
                producer.thread.start()
            if targets is None:
                continue
            if controller.entry_target is not None and not moved:
                # Move the human source well outside the old 10 mm pose gate;
                # the locked calibration goal must still be planned and reached.
                for name in ('right_elbow', 'right_wrist'):
                    producer.frames[name]['pos'][0] += .03
                moved=True
            if not prepared and state.get('accepted_seq', -1) >= 1:
                gateway.request('e');prepared=True
            elif state.get('entry_ready') and not selected:
                assert not backend.enabled
                gateway.request('g');selected=True
                if keepalive:
                    producer.close()  # No PICO packets throughout planning and approach.
            if state['mode']=='ACTIVE':
                break
        assert state['mode']=='ACTIVE',state.get('reason')
        assert selected and moved
        if keepalive:
            assert controller.entry_keepalives_sent >= 10
            assert 'APPROACHING' in modes
        assert {'CALIBRATING','CHECKING_ENTRY','WAITING_FOR_ACTIVE_TARGET'} <= modes
        assert gateway.entry_token and controller.entry_token==gateway.entry_token
        if with_hand:
            assert 'right' in pipeline.body.wrist_references
            assert pipeline.wrists['right'].last_diagnostics['orientation_mode']=='forearm_absolute'
            assert not pipeline.wrists['right'].last_diagnostics['entry_wrist_held']
            reference = np.array(controller.entry_target['right']['arm_urdf_rad'][4:])
            first_active = np.array(targets['right']['arm_urdf_rad'][4:])
            assert np.linalg.norm(first_active-reference) < np.deg2rad(5.)
    finally:
        producer.close();client.close();pipeline.close()
        gateway.request('q');worker.join(timeout=2)
