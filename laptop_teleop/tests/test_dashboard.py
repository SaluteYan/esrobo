"""Operator console boundary tests; no robot/hardware connections."""
from collections import deque
import http.client
import json
import threading
import time
from unittest.mock import Mock, patch

import pytest

from esrobo_laptop.dashboard import Console, Process, make_handler, validate_mode
from http.server import ThreadingHTTPServer


@pytest.fixture
def console():
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
    c.running = Mock(return_value=True)
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


def test_can_initialization_stops_and_verifies_all_bus_owners_first(console):
    order = []
    sessions = {'esrobo_gateway', 'esrobo_hand_bridge', 'esrobo_hands'}
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
    response = request(console, 'can', password='test-password')
    assert response['log'] == 'CAN ready'
    assert order == ['local_stop', 'd', 'q', 'esrobo_hand_bridge', 'esrobo_hands', 'sudo']


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
    console.command=Mock(return_value='bash\n')
    with pytest.raises(RuntimeError): console.key('e')
    assert console.command.call_count == 1


def test_gateway_startup_output_is_preserved_after_pane_exits(console):
    console.command = Mock(return_value='')
    console.remote = {'sessions': []}
    console.remote_start('gateway', dict(side='both', with_hand=True, hand_only=False))
    launch = console.command.call_args.args[0]
    assert launch[:5] == ['tmux', 'new-session', '-d', '-s', 'esrobo_gateway']
    assert 'pipe-pane' in launch[-1]
    assert 'esrobo_gateway_startup.log' in launch[-1]

    console.remote['gateway_startup_log'] = 'Traceback: startup failed\n'
    assert request(console, 'remote_log', name='gateway')['log'] == 'Traceback: startup failed\n'


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
