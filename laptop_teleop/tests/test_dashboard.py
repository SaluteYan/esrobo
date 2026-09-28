"""Operator console boundary tests; no robot/hardware connections."""
from collections import deque
import http.client
import json
import threading
import time
from unittest.mock import Mock, patch

import pytest

from esrobo_laptop.dashboard import (Console, Process, REMOTE_STATUS,
                                     WaistCommandNotSent, make_handler, validate_mode)
from http.server import ThreadingHTTPServer


@pytest.fixture
def console(tmp_path, monkeypatch):
    from esrobo_laptop import dashboard
    monkeypatch.setattr(dashboard, 'WAIST_MOVED_FILE', tmp_path / 'waist_marker.json')
    c = Console.__new__(Console)
    c.ssh = None
    c.processes = {}
    c.remote = {}
    c.remote_time = time.monotonic()
    c.remote_error = ''
    c.pc_service = False
    c.lock = threading.RLock()
    c.remote_lock = threading.Lock()
    c.owner = None
    c.heartbeat = 0.
    c.closed = threading.Event()
    c.events = deque(maxlen=80)
    c.mode = dict(side='both', with_hand=True, hand_only=False, preview=False)
    c.waist_pending = None
    c.waist_pending_hits = 0
    c.waist_enable_requested = False
    c.refresh = Mock()
    c.key = Mock()
    return c


def request(c, action, **values):
    return c.action(dict(action=action, session='test-browser-session', **values))


def test_cross_origin_and_rebinding_requests_are_rejected():
    c = Mock()
    server = ThreadingHTTPServer(('127.0.0.1', 0), make_handler(c, 0))
    port = server.server_address[1]
    server.RequestHandlerClass = make_handler(c, port)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        for method, origin, host, expected in [
            ('POST', 'http://evil.test', f'127.0.0.1:{port}', 403),
            ('GET', '', 'evil.test', 403),
            ('POST', f'http://127.0.0.1:{port}', f'127.0.0.1:{port}', 200),
        ]:
            conn = http.client.HTTPConnection('127.0.0.1', port)
            c.action.return_value = {}
            conn.request(method, '/api/action', body='{}', headers={
                'Host':host, 'Origin':origin, 'Content-Type':'application/json', 'X-Head-Control':'1'})
            res = conn.getresponse()
            assert res.status == expected
            res.read()
            conn.close()
        assert c.action.call_count == 1
    finally:
        server.shutdown()
        server.server_close()
        thread.join()


def ready(c):
    c.owner = 'test-browser-session'
    c.remote = {'sessions': ['esrobo_waist']}
    c.running = Mock(return_value=True)
    c.waist_command = Mock(return_value=dict(axes={
        str(axis): dict(position_deg=position, velocity=0.0, error=0)
        for axis, position in ((31, 139.0), (32, 215.0), (33, 144.0))}))
    c.telemetry = Mock(return_value=dict(age_s=.05, targets={'left':{}}, sent=3))
    c.robot_state = Mock(return_value=dict(mode='IDLE', age_s=.05, feedback={'sides':{
        'left':dict(controller_fault=None, hand=dict(geometry=dict(
            ready=True, calibrated_joints=10, total_joints=10))),
        'right':dict(controller_fault=None, hand=dict(geometry=dict(
            ready=True, calibrated_joints=10, total_joints=10)))}}))


@pytest.mark.parametrize('bad', ['fault','stale_robot','stale_local','preview','no_targets','not_owner','active','no_process'])
def test_enable_fails_closed(console, bad):
    ready(console)
    if bad == 'fault':
        console.robot_state.return_value['feedback']['sides']['left']['controller_fault']={'status_name':'EMERGENCY_STOP'}
    if bad == 'stale_robot': console.robot_state.return_value['age_s']=10
    if bad == 'stale_local': console.telemetry.return_value['age_s']=10
    if bad == 'preview': console.mode['preview']=True
    if bad == 'no_targets': console.telemetry.return_value['targets']=None
    if bad == 'not_owner': console.owner='different-browser'
    if bad == 'active': console.robot_state.return_value['mode']='ACTIVE'
    if bad == 'no_process': console.running.return_value=False
    with pytest.raises(RuntimeError):
        request(console, 'gateway_key', key='e')
    console.key.assert_not_called()


def test_enable_uses_existing_operator_channel(console):
    ready(console)
    request(console, 'gateway_key', key='e')
    console.key.assert_called_once_with('e')
    console.waist_command.assert_called_once_with('status')


def test_enable_rejects_waist_away_from_urdf_pose(console):
    ready(console)
    console.waist_command.return_value['axes']['32']['position_deg'] = 214.7
    with pytest.raises(RuntimeError, match='腰部 32 号未停在 URDF 竖直基准'):
        request(console, 'gateway_key', key='e')
    console.key.assert_not_called()


def test_enable_rejects_missing_waist_feedback(console):
    ready(console)
    console.waist_command.return_value['axes'].pop('33')
    with pytest.raises(RuntimeError, match='腰部三轴反馈不完整'):
        request(console, 'gateway_key', key='e')
    console.key.assert_not_called()


@pytest.mark.parametrize('field,value', [('velocity', 1.1), ('error', 7),
                                         ('position_deg', float('nan'))])
def test_enable_rejects_unstable_or_faulted_waist(console, field, value):
    ready(console)
    console.waist_command.return_value['axes']['31'][field] = value
    with pytest.raises(RuntimeError, match='腰部 31 号未停在 URDF 竖直基准'):
        request(console, 'gateway_key', key='e')
    console.key.assert_not_called()


def test_enable_rejects_missing_hand_geometry_before_robot_command(console):
    ready(console)
    console.mode.update(side='right', with_hand=True, hand_only=False)
    console.robot_state.return_value['feedback'] = {'sides': {'right': dict(
        controller_fault=None,
        hand=dict(geometry=dict(ready=False, calibrated_joints=0, total_joints=10)))}}
    with pytest.raises(RuntimeError, match='右手 0/10'):
        request(console, 'gateway_key', key='e')
    console.key.assert_not_called()


def test_hand_only_enable_does_not_require_arm_return_geometry(console):
    ready(console)
    console.mode.update(side='right', with_hand=True, hand_only=True)
    console.robot_state.return_value['feedback'] = {'sides': {'right': dict(
        controller_fault=None,
        hand=dict(geometry=dict(ready=False, calibrated_joints=0, total_joints=10)))}}
    request(console, 'gateway_key', key='e')
    console.key.assert_called_once_with('e')


def test_enable_refuses_unchecked_dual_arm_automatic_return(console):
    ready(console)
    console.remote = {'gateway_args': ['--hardware', '--side', 'both', '--with-hand']}
    with pytest.raises(RuntimeError, match='双臂互碰扫掠检查'):
        request(console, 'gateway_key', key='e')
    console.key.assert_not_called()


def test_fault_recovery_uses_existing_gateway_without_restarting_it(console):
    console.running = Mock(return_value=False)
    console.remote = {'sessions': ['esrobo_gateway'],
                      'gateway_args': ['python', '-m', 'esrobo_link.gateway', '--side', 'right', '--hand-only']}
    console.robot_state = Mock(return_value=dict(mode='FAULT', age_s=.05, recovery_supported=True))
    response = request(console, 'remote_recover')
    console.key.assert_called_once_with('r')
    assert 'IDLE' in response['message']


@pytest.mark.parametrize('reason', ['running', 'stale', 'active', 'dual'])
def test_fault_recovery_rejects_unsafe_state(console, reason):
    console.running = Mock(return_value=reason == 'running')
    side = 'both' if reason == 'dual' else 'right'
    console.remote = {'sessions': ['esrobo_gateway'], 'gateway_args': ['--side', side]}
    console.robot_state = Mock(return_value=dict(
        mode='ACTIVE' if reason == 'active' else 'FAULT',
        age_s=2 if reason == 'stale' else .05, recovery_supported=True))
    with pytest.raises(RuntimeError):
        request(console, 'remote_recover')
    console.key.assert_not_called()


def test_fault_recovery_rejects_old_robot_gateway(console):
    console.running = Mock(return_value=False)
    console.remote = {'sessions': ['esrobo_gateway'], 'gateway_args': ['--side', 'right']}
    console.robot_state = Mock(return_value=dict(mode='FAULT', age_s=.05))
    with pytest.raises(RuntimeError, match='尚未加载故障恢复功能'):
        request(console, 'remote_recover')
    console.key.assert_not_called()


def test_waist_jog_requires_disabled_arm_and_blocks_next_arm_enable(console, tmp_path):
    from esrobo_laptop import dashboard
    console.remote = {'sessions': ['esrobo_gateway', 'esrobo_waist'],
                      'waist_control_available': True}
    console.robot_state = Mock(return_value=dict(
        mode='FAULT', age_s=.05, feedback={'enable_states': [False] * 7}))
    console.running = Mock(return_value=False)
    console.waist_enable_requested = True
    console.waist_command = Mock(return_value=dict(
        axis=33, from_deg=170.0, target_deg=171.0, message='sent'))
    with patch.object(dashboard, 'WAIST_MOVED_FILE', tmp_path / 'waist.json'):
        request(console, 'waist', command='jog', axis=33, direction=1)
        assert dashboard.WAIST_MOVED_FILE.exists()
        console.waist_command.assert_called_once_with('jog', 33, 1)
        with pytest.raises(RuntimeError, match='腰部曾通过页面离开竖直基准'):
            request(console, 'gateway_key', key='e')
        console.key.assert_not_called()


def test_waist_jog_rejects_enabled_arm_before_ros_command(console, tmp_path):
    from esrobo_laptop import dashboard
    console.remote = {'sessions': ['esrobo_gateway', 'esrobo_waist'],
                      'waist_control_available': True}
    console.robot_state = Mock(return_value=dict(
        mode='FAULT', age_s=.05, feedback={'enable_states': [True] * 7}))
    console.running = Mock(return_value=False)
    console.waist_command = Mock()
    with patch.object(dashboard, 'WAIST_MOVED_FILE', tmp_path / 'waist.json'):
        with pytest.raises(RuntimeError, match='七轴全部失能'):
            request(console, 'waist', command='jog', axis=33, direction=1)
        console.waist_command.assert_not_called()


def test_waist_requires_enable_request_before_motion(console):
    console.remote = {'sessions': ['esrobo_gateway', 'esrobo_waist'],
                      'waist_control_available': True}
    console.robot_state = Mock(return_value=dict(
        mode='IDLE', age_s=.05, feedback={'enable_states': [False] * 7}))
    console.running = Mock(return_value=False)
    console.waist_command = Mock(return_value=dict(message='enable sent'))
    with pytest.raises(RuntimeError, match='尚未请求使能'):
        request(console, 'waist', command='jog', axis=33, direction=1)
    console.waist_command.assert_not_called()
    request(console, 'waist', command='enable')
    assert console.waist_enable_requested is True


def test_waist_driver_restart_requires_new_enable_request(console):
    from esrobo_laptop.dashboard import Console
    console.remote = {'waist_session_id': '$2:123'}
    console.waist_enable_requested = True
    console.command = Mock(return_value=json.dumps({'sessions': ['esrobo_waist'],
                                                    'waist_session_id': '$3:456'}))
    Console.refresh(console)
    assert console.waist_enable_requested is False


def test_waist_feedback_read_does_not_wait_for_gateway_refresh(console):
    console.waist_command = Mock(return_value=dict(axes={
        str(axis): dict(position_deg=position, error=0)
        for axis, position in ((31, 120.0), (32, 260.0), (33, 170.0))}))
    result = request(console, 'waist', command='status')
    assert result['motion_pending'] is False
    console.refresh.assert_not_called()
    console.waist_command.assert_called_once_with('status', None, None)


def test_waist_jog_waits_for_measured_target_then_accepts_next_step(console, tmp_path):
    from esrobo_laptop import dashboard
    console.remote = {'sessions': ['esrobo_gateway', 'esrobo_waist'],
                      'waist_control_available': True}
    console.robot_state = Mock(return_value=dict(
        mode='IDLE', age_s=.05, feedback={'enable_states': [False] * 7}))
    console.running = Mock(return_value=False)
    console.waist_enable_requested = True
    positions = {31: 139.0, 32: 215.0, 33: 144.0}

    def command(action, axis=None, direction=None):
        if action == 'jog':
            return dict(axis=axis, from_deg=positions[axis],
                        target_deg=positions[axis] + direction, message='sent')
        return dict(axes={str(key): dict(position_deg=value, error=0)
                          for key, value in positions.items()}, message='read')

    console.waist_command = Mock(side_effect=command)
    with patch.object(dashboard, 'WAIST_MOVED_FILE', tmp_path / 'waist.json'):
        request(console, 'waist', command='jog', axis=33, direction=1)
        with pytest.raises(RuntimeError, match='尚未确认到位'):
            request(console, 'waist', command='jog', axis=33, direction=1)
        assert request(console, 'waist', command='status')['motion_pending'] is True
        positions[33] = 145.0
        assert request(console, 'waist', command='status')['motion_pending'] is True
        assert request(console, 'waist', command='status')['motion_pending'] is False
        request(console, 'waist', command='jog', axis=33, direction=1)
        assert json.loads(dashboard.WAIST_MOVED_FILE.read_text())['pending']['target'] == 146.0


def test_waist_single_turn_battery_warning_confirms_target_and_reference(console):
    from esrobo_laptop import dashboard
    console.remote = {'sessions': ['esrobo_gateway', 'esrobo_waist'],
                      'waist_control_available': True}
    console.robot_state = Mock(return_value=dict(
        mode='IDLE', age_s=.05, feedback={'enable_states': [False] * 7}))
    console.running = Mock(return_value=False)
    console.waist_enable_requested = True
    positions = {31: 139.0, 32: 215.0, 33: 144.0}

    def command(action, axis=None, direction=None):
        if action == 'jog':
            return dict(axis=axis, from_deg=positions[axis], target_deg=145.0, message='sent')
        return dict(axes={str(key): dict(position_deg=value, error=0x0C26)
                          for key, value in positions.items()})

    console.waist_command = Mock(side_effect=command)
    request(console, 'waist', command='jog', axis=33, direction=1)
    positions[33] = 145.0
    assert request(console, 'waist', command='status')['motion_pending'] is True
    assert request(console, 'waist', command='status')['motion_pending'] is False
    assert dashboard.WAIST_MOVED_FILE.exists()
    positions[33] = 144.0
    marker = json.loads(dashboard.WAIST_MOVED_FILE.read_text())
    marker['at'] -= 4
    dashboard.WAIST_MOVED_FILE.write_text(json.dumps(marker))
    request(console, 'waist', command='status')
    assert not dashboard.WAIST_MOVED_FILE.exists()


def test_waist_command_preserves_ros_python_path(console):
    console.remote = {'waist_control_available': True}
    console.command = Mock(return_value='{"ok": true, "axes": {}}\n')
    result = console.waist_command('status')
    assert result == {'axes': {}}
    argv = console.command.call_args.args[0]
    assert argv[:2] == ['bash', '-lc']
    assert 'source /opt/ros/humble/setup.bash' in argv[2]
    assert '"${PYTHONPATH:-}"' in argv[2]
    assert 'esrobo_link.waist_control status' in argv[2]


def test_waist_direct_target_validates_range_and_uses_existing_interlock(console):
    from esrobo_laptop import dashboard
    console.remote = {'sessions': ['esrobo_gateway', 'esrobo_waist'],
                      'waist_control_available': True}
    console.robot_state = Mock(return_value=dict(
        mode='IDLE', age_s=.05, feedback={'enable_states': [False] * 7}))
    console.running = Mock(return_value=False)
    console.waist_enable_requested = True
    console.waist_command = Mock(return_value=dict(
        axis=31, from_deg=120.0, target_deg=124.5, message='sent'))
    for target in (-0.1, 360.1, float('nan'), True):
        with pytest.raises(ValueError, match='目标角度无效'):
            request(console, 'waist', command='target', axis=31, target_deg=target)
    assert not dashboard.WAIST_MOVED_FILE.exists()
    request(console, 'waist', command='target', axis=31, target_deg=124.5)
    console.waist_command.assert_called_once_with('target', 31, target_deg=124.5)
    assert json.loads(dashboard.WAIST_MOVED_FILE.read_text())['pending']['target'] == 124.5
    with pytest.raises(RuntimeError, match='尚未确认到位'):
        request(console, 'waist', command='jog', axis=31, direction=-1)


def test_waist_target_accepts_next_window_outside_original_reference(console):
    console.remote = {'sessions': ['esrobo_gateway', 'esrobo_waist'],
                      'waist_control_available': True}
    console.robot_state = Mock(return_value=dict(
        mode='IDLE', age_s=.05, feedback={'enable_states': [False] * 7}))
    console.running = Mock(return_value=False)
    console.waist_enable_requested = True
    console.waist_command = Mock(return_value=dict(
        axis=32, from_deg=255.0, target_deg=250.0, message='sent'))
    request(console, 'waist', command='target', axis=32, target_deg=250.0)
    console.waist_command.assert_called_once_with('target', 32, target_deg=250.0)


def test_waist_direct_target_cli_is_fixed_and_bounded(console):
    console.remote = {'waist_control_available': True}
    console.command = Mock(return_value='{"ok": true, "axis": 31, "target_deg": 124.5}\n')
    assert console.waist_command('target', 31, target_deg=124.5)['target_deg'] == 124.5
    assert 'esrobo_link.waist_control target --axis 31 --target-deg 124.5' in console.command.call_args.args[0][2]
    with pytest.raises(ValueError, match='目标角度无效'):
        console.waist_command('target', 31, target_deg=1000)
    assert console.command.call_count == 1


def test_waist_prepublication_rejection_does_not_latch_pending(console):
    from esrobo_laptop import dashboard
    console.remote = {'sessions': ['esrobo_gateway', 'esrobo_waist'],
                      'waist_control_available': True}
    console.robot_state = Mock(return_value=dict(
        mode='IDLE', age_s=.05, feedback={'enable_states': [False] * 7}))
    console.running = Mock(return_value=False)
    console.waist_enable_requested = True
    previous = b'{"axis": 32, "at": 1}'
    dashboard.WAIST_MOVED_FILE.write_bytes(previous)
    console.waist_command = Mock(side_effect=WaistCommandNotSent('目标超出当前反馈 ±5° 操作范围'))
    with pytest.raises(WaistCommandNotSent, match='超出当前反馈'):
        request(console, 'waist', command='target', axis=33, target_deg=161.97)
    assert console.waist_pending is None
    assert dashboard.WAIST_MOVED_FILE.read_bytes() == previous


def test_uncertain_waist_reply_keeps_motion_locked(console):
    from esrobo_laptop import dashboard
    console.remote = {'sessions': ['esrobo_gateway', 'esrobo_waist'],
                      'waist_control_available': True}
    console.robot_state = Mock(return_value=dict(
        mode='IDLE', age_s=.05, feedback={'enable_states': [False] * 7}))
    console.running = Mock(return_value=False)
    console.waist_enable_requested = True
    console.waist_command = Mock(side_effect=RuntimeError('SSH reply lost'))
    with pytest.raises(RuntimeError, match='SSH reply lost'):
        request(console, 'waist', command='target', axis=33, target_deg=165.0)
    assert console.waist_pending == (33, None)
    assert json.loads(dashboard.WAIST_MOVED_FILE.read_text())['pending']['target'] is None


def test_waist_cli_reports_explicit_prepublication_rejection(console):
    console.remote = {'waist_control_available': True}
    console.command = Mock(side_effect=RuntimeError(json.dumps(dict(
        ok=False, command_sent=False, error='目标超出当前反馈 ±5° 操作范围'))))
    with pytest.raises(WaistCommandNotSent, match='超出当前反馈'):
        console.waist_command('target', 33, target_deg=161.97)


def test_waist_disable_is_not_an_api_command(console):
    console.remote = {'waist_control_available': True}
    console.command = Mock()
    with pytest.raises(ValueError, match='未知腰部命令'):
        console.waist_command('disable')
    with pytest.raises(ValueError, match='未知腰部操作'):
        request(console, 'waist', command='disable')
    console.command.assert_not_called()


def test_stopping_waist_driver_does_not_send_disable(console):
    console.remote = {'sessions': ['esrobo_waist']}
    console.running = Mock(return_value=False)
    console.waist_interlock = Mock()
    console.waist_command = Mock(return_value={'message': 'stopped'})
    console.command = Mock()
    request(console, 'remote_stop', name='waist')
    console.waist_command.assert_called_once_with('stop')
    console.command.assert_called_once()


def test_can_initialization_stops_and_verifies_all_bus_owners_first(console):
    order = []
    sessions = {'esrobo_gateway', 'esrobo_waist', 'esrobo_hand_bridge', 'esrobo_hands'}
    console.remote = {'sessions': list(sessions), 'gateway_args': ['--side', 'right']}
    console.running = Mock(return_value=True)
    console.stop_following = Mock(side_effect=lambda: order.append('local_stop'))
    robot = dict(mode='ACTIVE', age_s=.05, reason='', feedback={
        'enable_states': [True] * 7})
    console.robot_state = Mock(side_effect=lambda: robot)

    def refresh():
        console.remote['sessions'] = list(sessions)

    def key(value):
        order.append(value)
        if value == 'd':
            robot.update(mode='FAULT', disable_verified=True,
                         feedback={'enable_states': [False] * 7})
        if value == 'q':
            sessions.remove('esrobo_gateway')

    def command(argv, **kwargs):
        if argv[:3] == ['tmux', 'send-keys', '-t']:
            name = argv[3]
            order.append(name)
            sessions.remove(name)
        elif argv[:2] == ['sudo', '-S']:
            order.append('sudo')
            assert not sessions
            return 'CAN ready'
        else:
            raise AssertionError(argv)
        return ''

    console.refresh = Mock(side_effect=refresh)
    console.key = Mock(side_effect=key)
    console.command = Mock(side_effect=command)
    console.waist_command = Mock(side_effect=lambda action: order.append('waist_stop'))
    console.wait_waist_stopped = Mock(side_effect=lambda: order.append('waist_settled'))
    response = request(console, 'can', password='test-password')
    assert response['log'] == 'CAN ready'
    assert order == ['local_stop', 'd', 'waist_stop', 'waist_settled', 'esrobo_waist',
                     'q', 'esrobo_hand_bridge', 'esrobo_hands', 'sudo']
    console.waist_command.assert_called_once_with('stop')


def test_can_waist_stop_requires_stable_feedback_before_driver_exit(console):
    positions = [120.0, 120.2, 120.201, 120.202]
    readings = [dict(axes={str(axis): dict(position_deg=(position if axis == 31 else reference),
                                          velocity=0.1)
                           for axis, reference in ((31, 120.0), (32, 260.0), (33, 170.0))})
                for position in positions]
    console.waist_command = Mock(side_effect=readings)
    with patch('esrobo_laptop.dashboard.time.sleep'):
        console.wait_waist_stopped()
    assert console.waist_command.call_count == 4


def test_can_keeps_waist_driver_if_motion_cannot_be_verified_stopped(console):
    sessions = ['esrobo_gateway', 'esrobo_waist']
    console.remote = {'sessions': sessions, 'gateway_args': ['--side', 'right']}
    console.running = Mock(return_value=False)
    console.wait_gateway_disabled = Mock()
    console.waist_command = Mock()
    console.wait_waist_stopped = Mock(side_effect=RuntimeError('腰部未停稳'))
    console.command = Mock()
    with pytest.raises(RuntimeError, match='腰部未停稳'):
        request(console, 'can', password='test-password')
    console.key.assert_called_once_with('d')
    console.waist_command.assert_called_once_with('stop')
    console.command.assert_not_called()


def test_can_initialization_aborts_if_disable_cannot_be_verified(console):
    console.remote = {'sessions': ['esrobo_gateway'], 'gateway_args': ['--side', 'right']}
    console.command = Mock()
    console.wait_gateway_disabled = Mock(side_effect=RuntimeError('disable unverified'))
    with pytest.raises(RuntimeError, match='disable unverified'):
        request(console, 'can', password='test-password')
    console.key.assert_called_once_with('d')
    console.command.assert_not_called()


def test_stop_terminates_computation_before_ssh_even_if_ssh_fails(console):
    order=[]
    process=Mock()
    process.proc.poll.return_value=None
    process.stop.side_effect=lambda:order.append('local_stop')
    console.processes['teleop']=process
    console.connected=Mock(return_value=True)
    console.remote={'sessions':['esrobo_gateway']}
    def fail(_):
        order.append('ssh_stop')
        raise RuntimeError('SSH disconnected')
    console.key.side_effect=fail
    console.owner='another-browser'
    request(console, 'stop')
    assert order == ['local_stop', 'ssh_stop']
    assert console.owner is None
    assert '失败' in console.events[-1]


def test_pause_preview_stops_only_local_preview_without_robot_stop(console):
    process = Mock()
    process.proc.poll.return_value = None
    console.processes['teleop'] = process
    console.owner = 'test-browser-session'
    console.mode.update(preview=True)
    with patch('esrobo_laptop.dashboard.STATUS_FILE') as status_file:
        request(console, 'pause_preview')
    process.stop.assert_called_once_with()
    status_file.unlink.assert_called_once_with(missing_ok=True)
    console.key.assert_not_called()
    assert console.owner is None
    assert '预览计算已暂停' in console.events[-1]


def test_pause_preview_refuses_to_interrupt_formal_control(console):
    process = Mock()
    process.proc.poll.return_value = None
    console.processes['teleop'] = process
    console.mode.update(preview=False)
    with pytest.raises(RuntimeError, match='没有正在运行的预览'):
        request(console, 'pause_preview')
    process.stop.assert_not_called()


def test_other_tab_cannot_steal_heartbeat_or_enable(console):
    console.owner='another-browser'
    request(console, 'heartbeat')
    assert console.heartbeat == 0
    with pytest.raises(RuntimeError): request(console, 'gateway_key', key='e')
    request(console, 'release')
    assert console.owner == 'another-browser'


def test_mode_and_command_input_are_allowlisted(console):
    with pytest.raises(ValueError): validate_mode(dict(side='both',with_hand=False))
    with pytest.raises(ValueError): validate_mode(dict(side='left; rm -rf /'))
    with pytest.raises(ValueError): request(console, 'gateway_key', key='e;whoami')
    with pytest.raises(ValueError): request(console, 'remote_start', name='bash')
    with pytest.raises(ValueError): request(console, 'enter', name='gateway')
    with pytest.raises(ValueError): request(console, 'glove_config', left_serial='$(id)', right_serial='001')
    console.key.assert_not_called()


def test_no_typing_into_shell_after_gateway_exit(console):
    console.key=Console.key.__get__(console)
    console.command=Mock(return_value='esrobo_gateway|bash\n')
    with pytest.raises(RuntimeError): console.key('e')
    assert console.command.call_count == 1


def test_status_does_not_query_missing_waist_session_by_target():
    assert "'#{session_name}|#{session_id}:#{session_created}'" in REMOTE_STATUS
    assert 'display-message' not in REMOTE_STATUS


def test_gateway_startup_output_is_preserved_after_pane_exits(console):
    console.command = Mock(return_value='')
    console.remote = {'sessions': []}
    refresh_count = 0
    def refresh():
        nonlocal refresh_count
        refresh_count += 1
        if refresh_count == 2:
            console.remote['sessions'] = ['esrobo_gateway']
    console.refresh = Mock(side_effect=refresh)
    console.remote_start('gateway', dict(side='both', with_hand=True, hand_only=False))
    launch = console.command.call_args.args[0]
    assert launch[:5] == ['tmux', 'new-session', '-d', '-s', 'esrobo_gateway']
    assert 'pipe-pane' in launch[-1]
    assert 'esrobo_gateway_startup.log' in launch[-1]

    console.remote['gateway_startup_log'] = 'Traceback: startup failed\n'
    assert request(console, 'remote_log', name='gateway')['log'] == 'Traceback: startup failed\n'


def test_gateway_immediate_exit_reports_startup_log(console):
    console.command = Mock(return_value='')
    console.remote = {'sessions': [], 'gateway_startup_log': 'Traceback: startup failed\n'}
    with pytest.raises(RuntimeError, match='startup failed'):
        console.remote_start('gateway', dict(side='right', with_hand=False, hand_only=False))
    assert not any('会话已建立' in event for event in console.events)


def test_pty_calibration_prompt_and_enter():
    import sys
    p=Process([sys.executable,'-u','-c','print("prepare",flush=True); input(); print("calibrated",flush=True)'])
    try:
        deadline=time.monotonic()+3
        while 'prepare' not in p.snapshot()['log'] and time.monotonic()<deadline:
            time.sleep(.01)
        assert 'prepare' in p.snapshot()['log']
        p.enter()
        p.proc.wait(timeout=3)
        deadline=time.monotonic()+1
        while 'calibrated' not in p.snapshot()['log'] and time.monotonic()<deadline:
            time.sleep(.01)
        assert 'calibrated' in p.snapshot()['log']
        with pytest.raises(RuntimeError): p.enter()
    finally:
        p.stop()


def test_stale_browser_watchdog_stops_own_session(console):
    console.owner='test-browser-session'
    console.heartbeat=time.monotonic()-10
    def stop():
        console.owner=None
        console.closed.set()
    console.stop_following=Mock(side_effect=stop)
    thread=threading.Thread(target=console._watchdog)
    thread.start()
    thread.join(timeout=2)
    assert not thread.is_alive()
    console.stop_following.assert_called_once()


def test_mode_mismatch_prevents_local_session_creation(console):
    console.connected=Mock(return_value=True)
    console.remote={'gateway_args':['python','-m','esrobo_link.gateway','--side','both']}
    console.robot_state=Mock(return_value=dict(mode='IDLE',age_s=.01))
    with pytest.raises(RuntimeError,match='模式'):
        request(console,'local_start',name='teleop',side='left',with_hand=False)
    assert not console.processes


@pytest.mark.parametrize(('side','with_hand','hand_only','expected'), [
    ('both', True, False, ['run_laptop.sh', 'check', '--side', 'both']),
    ('left', True, False, ['run_laptop.sh', 'check', '--side', 'left']),
    ('right', False, False, ['run_laptop.sh', 'check', '--side', 'right', '--arm-only']),
    ('right', True, True, ['run_laptop.sh', 'check', '--side', 'right', '--hand-only']),
])
def test_environment_check_uses_selected_teleoperation_mode(console, side, with_hand, hand_only, expected):
    with patch('esrobo_laptop.dashboard.Process') as process:
        console.local_start('check', dict(side=side, with_hand=with_hand, hand_only=hand_only, preview=True),
                            'test-browser-session')
    argv = process.call_args.args[0]
    assert argv[0].endswith(expected[0])
    assert argv[1:] == expected[1:]
    assert side in console.events[-1] or {'both':'双臂','left':'左臂','right':'右臂'}[side] in console.events[-1]


def test_snapshot_restores_live_gateway_mode_after_browser_refresh(console):
    console.remote = {'gateway_args': ['python3', '-m', 'esrobo_link.gateway',
                                       '--side', 'left', '--with-hand', '--hand-only']}
    assert console.snapshot()['mode'] == dict(
        side='left', with_hand=True, hand_only=True, preview=False)


@pytest.mark.parametrize(('name','key'), [('gateway','d'),('left_arm','dl'),('right_arm','dr'),('hands','dh')])
def test_component_disable_stops_targets_before_gateway_command(console, name, key):
    order=[]
    console.stop_following=Mock(side_effect=lambda:order.append('stop'))
    console.key=Mock(side_effect=lambda value:order.append(value))
    request(console, 'remote_disable', name=name)
    assert order == ['stop', key]


def test_head_disable_uses_verified_ros_service_without_stopping_arm_session(console):
    console.command=Mock(return_value="response: std_srvs.srv.SetBool_Response(success=True, message='verified')")
    console.stop_following=Mock()
    request(console, 'remote_disable', name='head')
    command = console.command.call_args.args[0]
    assert command[:2] == ['bash', '-lc']
    assert '/head/torque_enable' in command[2]
    console.stop_following.assert_not_called()


def test_head_disable_rejects_unverified_result(console):
    console.command=Mock(return_value="response: SetBool_Response(success=False, message='ID 2 stale')")
    with pytest.raises(RuntimeError, match='未确认'):
        request(console, 'remote_disable', name='head')


def test_http10_camera_ssh_channel_remains_open_until_body_read(console):
    import socket
    a,b=socket.socketpair()
    class Channel:
        def makefile(self,*args): return a.makefile(*args)
        def sendall(self,data): return a.sendall(data)
        def settimeout(self,value): a.settimeout(value)
        def close(self):
            a.shutdown(socket.SHUT_RDWR)
            a.close()
    payload=b'camera-jpeg-payload'*2000
    def remote():
        try:
            b.recv(4096)
            b.sendall(f'HTTP/1.0 200 OK\r\nContent-Type: image/jpeg\r\nContent-Length: {len(payload)}\r\n\r\n'.encode())
            time.sleep(.1)
            b.sendall(payload)
        finally:
            b.close()
    thread=threading.Thread(target=remote)
    thread.start()
    console.ssh=Mock()
    console.ssh.get_transport.return_value.open_channel.return_value=Channel()
    result=console.head('GET','/api/head/color.jpg')
    thread.join(timeout=2)
    assert result == (200,payload,'image/jpeg')


def test_compute_status_file_is_atomic_and_omits_session_tokens(monkeypatch, tmp_path):
    from esrobo_laptop import app
    from esrobo_laptop.config import Settings
    client=Mock()
    state=dict(mode='IDLE',contract={'id':'test-contract'},feedback={},feedback_cache_age_s=0,
               session='private-session',lease='private-lease')
    client.connect.return_value=state
    client.receive.return_value=state
    pipeline=Mock()
    pipeline.body.input_lock=threading.RLock()
    pipeline.body.stamps={'left_arm':time.monotonic()}
    pipeline.body.required=('left_arm', 'left_hand', 'left_wrist')
    pipeline.body.source_rates.return_value={'left_arm':50.0}
    pipeline.body.last_rejection=''
    pipeline.body.pico_hand_status={}
    pipeline.body.pico_hand_status_at=None
    pipeline.body.pico_hand_targets={}
    pipeline.body.is_ready.return_value=True
    pipeline.hand_references={'left': [0]*10, 'right': [0]*10}
    pipeline.initialized=True
    controller=Mock()
    controller.sent=2
    controller.computed=3
    controller.step.return_value={'left':{'arm_urdf_rad':[0]*7}}
    monkeypatch.setattr(app,'read_settings',lambda _:Settings(robot_host='127.0.0.1'))
    monkeypatch.setattr(app,'read_robot_config',lambda _:Mock())
    monkeypatch.setattr(app,'verify_contract',lambda *a,**k:{})
    monkeypatch.setattr(app,'RobotClient',lambda *a:client)
    monkeypatch.setattr(app,'Pipeline',lambda *a:pipeline)
    monkeypatch.setattr(app,'Controller',lambda *a:controller)
    status=tmp_path/'status.json'
    assert app.main(['run','--mock','--seconds','.03','--log-file',str(tmp_path/'run.jsonl'),
                     '--status-file',str(status)])==0
    data=json.loads(status.read_text())
    assert data['reference_ready'] is True
    assert data['initialized'] is True
    assert data['sent']==2 and data['sources']['left_arm']>=0
    assert 'private' not in status.read_text()
    assert not status.with_suffix('.tmp').exists()
    client.close.assert_called_once()


def test_disconnected_ssh_stops_compute_without_waiting_for_browser_timeout(console, monkeypatch):
    import esrobo_laptop.dashboard as dashboard
    console.connected=Mock(return_value=False)
    console.running=Mock(return_value=True)
    console.stop_following=Mock(side_effect=console.closed.set)
    monkeypatch.setattr(dashboard.subprocess,'run',Mock(return_value=Mock(returncode=1)))
    thread=threading.Thread(target=console._monitor)
    thread.start()
    thread.join(timeout=2)
    assert not thread.is_alive()
    console.stop_following.assert_called_once()
    assert 'SSH' in console.remote_error


def test_outer_skeleton_tab_recovers_a_navigated_iframe():
    from esrobo_laptop.config import ROOT
    script = (ROOT / 'web/app.js').read_text()
    assert 'frame.contentWindow.location.pathname' in script
    assert 'if (currentPath !== expectedPath) frame.src = frame.dataset.src' in script


@pytest.mark.parametrize('service', ['current', 'legacy', 'unrelated'])
def test_repeated_start_preserves_existing_service_without_starting_workers(monkeypatch, capsys, service):
    import esrobo_laptop.dashboard as dashboard
    from http.server import BaseHTTPRequestHandler

    class ExistingService(BaseHTTPRequestHandler):
        def do_GET(self):
            status, body = 404, b'{}'
            if service == 'current' and self.path == '/api/health':
                status, body = 200, b'{"service":"esrobo-laptop-dashboard"}'
            elif service == 'legacy' and self.path == '/':
                status, body = 200, '<title>ESROBO · 遥操作控制台</title>'.encode()
            elif service == 'legacy' and self.path == '/api/state':
                status, body = 200, b'{"connected":true,"remote":{},"processes":{},"mode":{}}'
            elif service == 'unrelated':
                status, body = 200, b'Unrelated web server'
            self.send_response(status)
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *_):
            pass

    server = ThreadingHTTPServer(('127.0.0.1', 0), ExistingService)
    port = server.server_address[1]
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    factory = Mock(side_effect=AssertionError('duplicate startup must not create a Console'))
    monkeypatch.setattr(dashboard, 'Console', factory)
    try:
        result = dashboard.main(['--port', str(port)])
        assert result == (1 if service == 'unrelated' else 0)
        output = capsys.readouterr().out
        assert ('已被其他程序占用' if service == 'unrelated' else '控制台已在运行') in output
        factory.assert_not_called()
        assert thread.is_alive()
    finally:
        server.shutdown()
        server.server_close()
        thread.join()
