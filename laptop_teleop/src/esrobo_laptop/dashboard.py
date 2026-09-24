"""Loopback operator console. Fixed commands, SSH transport, no shell from HTTP."""
import argparse
import codecs
import errno
from collections import deque
import http.client
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import pty
import re
import shlex
import signal
import subprocess
import threading
import time
from urllib.parse import urlsplit

from .config import ROOT, WORKSPACE

REMOTE = '/home/esrobo/Projects/esrobo'
REMOTE_PYTHON = '/home/esrobo/miniconda3/envs/teleop_esrobo/bin/python'
ROBOT_HOST = '192.168.10.100'
GATEWAY_STARTUP_LOG = '/tmp/esrobo_gateway_startup.log'
SESSIONS = dict(hands='esrobo_hands', hand_bridge='esrobo_hand_bridge', gateway='esrobo_gateway',
                head='esrobo_head', camera='esrobo_camera', head_web='esrobo_head_web')
LABELS = dict(pc_service='PICO PC Service', pico='PICO 采集', sensecom='SenseCom',
              glove_ros='SenseGlove ROS', glove='手套输入与标定', teleop='重定向 / IK', check='环境检查')
STATUS_FILE = ROOT / 'log/dashboard_teleop.json'
# Read only the journal held open by the live gateway, never a previous run's log.
REMOTE_STATUS = r'''
import os,json,time,subprocess
sessions=subprocess.run(['tmux','list-sessions','-F','#{session_name}'],capture_output=True,text=True).stdout.splitlines()
result={'sessions':sessions,'robot':None,'can':subprocess.run(['ip','-br','link','show','type','can'],capture_output=True,text=True).stdout}
try:
 with open('/tmp/esrobo_gateway_startup.log','rb') as f:
  f.seek(0,2); f.seek(max(0,f.tell()-20000)); result['gateway_startup_log']=f.read().decode(errors='replace')
except OSError: pass
for name in os.listdir('/proc'):
 if not name.isdigit(): continue
 try:
  args=open('/proc/'+name+'/cmdline','rb').read().split(b'\0')
  if b'esrobo_link.gateway' not in args: continue
  result['gateway_args']=[x.decode() for x in args if x]
  for fd in os.listdir('/proc/'+name+'/fd'):
   link=os.readlink('/proc/'+name+'/fd/'+fd)
   if not link.endswith('.jsonl'): continue
   with open(link,'rb') as f:
    f.seek(0,2); size=f.tell(); f.seek(max(0,size-131072)); lines=f.read().splitlines()
   for line in reversed(lines):
    try:
     entry=json.loads(line)
     if 'feedback_cache_age_s' not in entry: continue
     entry['sample_age_s']=max(0,time.monotonic()-entry['monotonic_s'])
     result['robot']=entry
     break
    except (ValueError,KeyError): pass
 except (OSError,ValueError): pass
print(json.dumps(result))
'''


class Process:
    def __init__(self, argv):
        self.log = deque(maxlen=350)
        self.lock = threading.Lock()
        self.stop_lock = threading.Lock()
        self.master, slave = pty.openpty()
        self.proc = subprocess.Popen(argv, cwd=WORKSPACE, stdin=slave, stdout=slave, stderr=slave,
                                     start_new_session=True, env=dict(os.environ, PYTHONUNBUFFERED='1'))
        os.close(slave)
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        decoder = codecs.getincrementaldecoder('utf-8')(errors='replace')
        try:
            while True:
                data = os.read(self.master, 8192)
                if not data:
                    break
                with self.lock:
                    self.log.append(decoder.decode(data))
        except OSError:
            pass
        finally:
            os.close(self.master)

    def enter(self):
        if self.proc.poll() is not None:
            raise RuntimeError('程序已经退出')
        os.write(self.master, b'\n')

    def stop(self):
        with self.stop_lock:
            if self.proc.poll() is None:
                try:
                    os.killpg(self.proc.pid, signal.SIGTERM)
                    self.proc.wait(timeout=5)
                except ProcessLookupError:
                    pass
                except subprocess.TimeoutExpired:
                    os.killpg(self.proc.pid, signal.SIGKILL)
                    self.proc.wait(timeout=2)

    def snapshot(self):
        with self.lock:
            return dict(running=self.proc.poll() is None, exit_code=self.proc.poll(),
                        log=''.join(self.log)[-22000:])


class Console:
    def __init__(self):
        self.ssh = None
        self.processes = {}
        self.remote = {}
        self.remote_time = 0.
        self.remote_error = '尚未连接机器人'
        self.pc_service = False
        self.lock = threading.RLock()
        self.remote_lock = threading.Lock()
        self.owner = None
        self.heartbeat = 0.
        self.closed = threading.Event()
        self.events = deque(maxlen=80)
        self.mode = dict(side='both', with_hand=True, hand_only=False, preview=True)
        threading.Thread(target=self._monitor, daemon=True).start()
        threading.Thread(target=self._watchdog, daemon=True).start()

    def event(self, message):
        self.events.append(time.strftime('%H:%M:%S') + ' ' + message)

    def connected(self):
        return bool(self.ssh and self.ssh.get_transport() and self.ssh.get_transport().is_active())

    def connect(self, password):
        import paramiko
        if self.running('teleop'):
            raise RuntimeError('先停止计算再重新连接 SSH')
        client = paramiko.SSHClient()
        client.load_system_host_keys()
        try:
            client.connect(ROBOT_HOST, username='esrobo', password=password or None,
                           timeout=5, auth_timeout=5, banner_timeout=5)
        except Exception:
            client.close()
            raise
        with self.remote_lock:
            if self.ssh:
                self.ssh.close()
            self.ssh = client
        self.event('SSH 已连接，主机密钥验证通过')
        self.refresh()

    def command(self, argv, *, script=False, stdin=None, timeout=8):
        if not self.connected():
            raise RuntimeError('请先连接机器人 SSH')
        with self.remote_lock:
            inp, out, err = self.ssh.exec_command(argv if script else shlex.join(argv), timeout=timeout)
            if stdin is not None:
                inp.write(stdin)
                inp.flush()
            inp.channel.shutdown_write()
            output = out.read().decode(errors='replace')
            error = err.read().decode(errors='replace')
            code = out.channel.recv_exit_status()
            if code:
                raise RuntimeError((error or output or f'命令退出 {code}')[-3000:])
            return output

    def refresh(self):
        data = json.loads(self.command(['python3', '-c', REMOTE_STATUS]))
        self.remote, self.remote_time, self.remote_error = data, time.monotonic(), ''

    def robot_state(self):
        state = dict(self.remote.get('robot') or {})
        if state:
            state['age_s'] = state.get('sample_age_s', 999) + time.monotonic()-self.remote_time
        return state

    def running(self, name):
        return name in self.processes and self.processes[name].proc.poll() is None

    def telemetry(self):
        if not self.running('teleop'):
            return {}
        try:
            value = json.loads(STATUS_FILE.read_text())
            value['age_s'] = time.monotonic()-value['monotonic_s']
            if value['age_s'] < 0:
                return {}
            return value
        except (OSError, ValueError, KeyError):
            return {}

    def snapshot(self):
        mode = dict(self.mode)
        remote_args = self.remote.get('gateway_args', [])
        if '--side' in remote_args:
            side = remote_args[remote_args.index('--side')+1]
            mode.update(side=side, with_hand=(side == 'both' or '--with-hand' in remote_args),
                        hand_only='--hand-only' in remote_args)
        return dict(connected=self.connected(), remote_error=self.remote_error,
                    remote_age_s=time.monotonic()-self.remote_time,
                    remote=self.remote, robot=self.robot_state(), telemetry=self.telemetry(),
                    processes={k: dict(label=LABELS[k], **v.snapshot()) for k, v in list(self.processes.items())},
                    events=list(self.events), mode=mode, owner=self.owner, pc_service=self.pc_service,
                    capabilities=dict(automatic_can_teardown=True, fault_recovery=True))

    def remote_start(self, name, mode):
        if name not in SESSIONS:
            raise ValueError('未知机器人服务')
        self.refresh()
        if SESSIONS[name] in self.remote.get('sessions', []):
            self.event(f'{name} 已有会话，保持现有进程；切换模式需先停止网关')
            return
        ros = 'source /opt/ros/humble/setup.bash && source install/setup.bash && '
        commands = dict(hands=ros+'exec ros2 launch linker_hand_ros2_sdk linker_hand_double.launch.py',
                        hand_bridge=ros+'exec python3 teleoperation/bridges/hand_ros_bridge.py --side both',
                        head=ros+'exec ros2 launch servo_driver start_servo.py allow_motion:=true',
                        camera='exec bash teleoperation/scripts/run_head_camera_20hz.sh',
                        head_web='exec bash teleoperation/scripts/run_head_camera_web.sh')
        gateway = ['env', f'PYTHONPATH={REMOTE}/robot_link:{REMOTE}/teleoperation/src', REMOTE_PYTHON,
                   '-m', 'esrobo_link.gateway', '--hardware', '--side', mode['side'], '--bind', ROBOT_HOST,
                   '--config', REMOTE+'/teleoperation/config/teleop_config.yaml',
                   '--key-file', '/home/esrobo/.config/esrobo/robot-link.key']
        if mode['with_hand']:
            gateway.append('--with-hand')
        if mode['hand_only']:
            gateway.append('--hand-only')
        commands['gateway'] = 'exec '+shlex.join(gateway)
        command = 'cd '+shlex.quote(REMOTE)+' && '+commands[name]
        if name == 'gateway':
            # tmux destroys the pane when startup fails. Capture its output
            # before exec so the traceback remains available after exit.
            capture = (' : > '+shlex.quote(GATEWAY_STARTUP_LOG)+' && '
                       'tmux pipe-pane -t "$TMUX_PANE" '
                       + shlex.quote('cat >> '+GATEWAY_STARTUP_LOG)+' && ')
            command = 'cd '+shlex.quote(REMOTE)+' &&'+capture+commands[name]
        self.command(['tmux', 'new-session', '-d', '-s', SESSIONS[name], 'bash -lc '+shlex.quote(command)])
        self.event(f'机器人 {name} 启动已提交，查看日志确认就绪')
        self.refresh()

    def key(self, key):
        # Never type into an exited program's shell. Only the gateway Python pane accepts keys.
        command = self.command(['tmux', 'display-message', '-p', '-t', SESSIONS['gateway'], '#{pane_current_command}']).strip()
        if 'python' not in command.lower():
            raise RuntimeError('网关 Python 程序未运行，拒绝发送终端按键')
        self.command(['tmux', 'send-keys', '-t', SESSIONS['gateway'], '-l', key])
        self.command(['tmux', 'send-keys', '-t', SESSIONS['gateway'], 'Enter'])
        self.event(f'网关命令 {key} 已发送；以实时反馈确认结果')

    def wait_remote_sessions_stopped(self, names, timeout=10):
        deadline = time.monotonic() + timeout
        while True:
            self.refresh()
            remaining = [name for name in names if SESSIONS[name] in self.remote.get('sessions', [])]
            if not remaining:
                return
            if time.monotonic() >= deadline:
                raise RuntimeError('等待机器人服务退出超时：' + '、'.join(remaining) + '；未执行 CAN 初始化')
            time.sleep(.2)

    def wait_gateway_disabled(self, timeout=10):
        deadline = time.monotonic() + timeout
        while True:
            self.refresh()
            state = self.robot_state()
            feedback = state.get('feedback') or {}
            members = feedback.get('sides') if isinstance(feedback, dict) else None
            members = members if isinstance(members, dict) else {'single': feedback}
            disabled = bool(members) and all(
                isinstance(item, dict) and item.get('enable_states') == [False] * 7
                for item in members.values())
            verified = (state.get('disable_verified') is True or
                        ('disable_verified' not in state and
                         str(state.get('reason', '')).startswith('disable_verified=True')))
            if (state.get('age_s', 999) < 1 and state.get('mode') == 'FAULT'
                    and verified and disabled):
                return
            if time.monotonic() >= deadline:
                raise RuntimeError('网关未确认双臂失能；保留服务并取消 CAN 初始化，请检查网关日志和现场状态')
            time.sleep(.2)

    def initialize_can(self, password):
        if not isinstance(password, str) or '\n' in password:
            raise ValueError('无效密码')
        # Stop target generation first. Never change CAN while a gateway or
        # hand driver can still command the bus.
        if self.running('teleop'):
            self.stop_following()
        self.refresh()
        sessions = self.remote.get('sessions', [])
        if SESSIONS['gateway'] in sessions:
            if self.remote.get('gateway_args'):
                self.key('d')
                self.wait_gateway_disabled()
                self.key('q')
            else:
                raise RuntimeError('网关会话存在但未确认网关进程；未执行 CAN 初始化，请检查会话与机器人状态')
            self.wait_remote_sessions_stopped(('gateway',))
        for name in ('hand_bridge', 'hands'):
            self.refresh()
            if SESSIONS[name] in self.remote.get('sessions', []):
                self.command(['tmux', 'send-keys', '-t', SESSIONS[name], 'C-c'])
                self.wait_remote_sessions_stopped((name,))
        self.refresh()
        if any(SESSIONS[name] in self.remote.get('sessions', []) for name in ('gateway', 'hand_bridge', 'hands')):
            raise RuntimeError('仍有占用 CAN 的服务；未执行初始化')
        output = self.command(['sudo', '-S', '-p', '', 'bash', REMOTE+'/start_can.sh'],
                              stdin=password+'\n', timeout=45)
        self.event('网关、双手通信桥和驱动已停止；CAN 初始化完成')
        return dict(log=output, message='CAN 初始化完成；按需重新启动双手驱动、通信桥和网关')

    def local_start(self, name, data, session):
        if name not in LABELS:
            raise ValueError('未知本机服务')
        if self.running(name):
            return
        scripts = ROOT/'scripts'
        mode = validate_mode(data)
        argv = {'pc_service': ['xrobotoolkit_service.sh', 'start'], 'pico': ['run_pico.sh'],
                'sensecom': ['run_sensecom.sh'], 'glove_ros': ['run_senseglove_ros.sh'],
                'glove': ['run_senseglove.sh'], 'check': ['run_laptop.sh', 'check'],
                'teleop': ['run_laptop.sh', 'run', '--side', mode['side'], '--status-file', str(STATUS_FILE)]}[name]
        if name == 'check':
            argv += ['--side', mode['side']]
            if mode['hand_only']:
                argv.append('--hand-only')
            elif not mode['with_hand']:
                argv.append('--arm-only')
        if name == 'glove':
            for side in ('left', 'right'):
                serial = data.get(side+'_serial', '')
                if not re.fullmatch(r'[A-Za-z0-9._-]{1,60}', serial):
                    raise ValueError('请填写两只真实手套序列号')
                argv += ['--'+side+'-serial', serial]
            if mode['side'] != 'both':
                argv += ['--single-side', mode['side']]
            if data.get('recalibrate'):
                argv += ['--recalibrate']
        if name == 'teleop':
            if not self.connected():
                raise RuntimeError('请先连接 SSH 以便监控和停止机器人')
            self.refresh()
            state = self.robot_state()
            if state.get('mode') != 'IDLE' or state.get('age_s', 999) > 1:
                raise RuntimeError('需要新鲜的 IDLE 网关状态')
            remote_args = self.remote.get('gateway_args', [])
            remote_side = remote_args[remote_args.index('--side')+1] if '--side' in remote_args else 'left'
            remote_hand = remote_side == 'both' or '--with-hand' in remote_args
            remote_hand_only = '--hand-only' in remote_args
            if (remote_side, remote_hand, remote_hand_only) != (mode['side'], mode['with_hand'], mode['hand_only']):
                raise RuntimeError('所选模式与正在运行的机器人网关不一致；停止并按所选模式重启网关')
            if mode['preview']:
                argv.append('--preview')
            if mode['hand_only']:
                argv.append('--hand-only')
            elif not mode['with_hand']:
                argv.append('--arm-only')
            STATUS_FILE.unlink(missing_ok=True)
        self.processes[name] = Process([str(scripts/argv[0]), *argv[1:]])
        if name == 'teleop':
            self.owner, self.heartbeat, self.mode = session, time.monotonic(), mode
        if name == 'check':
            side_label = {'both': '双臂', 'left': '左臂', 'right': '右臂'}[mode['side']]
            hand_label = ('仅灵巧手' if mode['hand_only'] else
                          '含灵巧手' if mode['with_hand'] else '仅机械臂')
            self.event(f'{LABELS[name]} 已启动（{side_label} · {hand_label}）')
        else:
            self.event(f'{LABELS[name]} 已启动')

    def stop_following(self):
        error = None
        # Stop local target generation BEFORE any potentially delayed SSH call.
        if self.running('teleop'):
            self.processes['teleop'].stop()
        self.owner = None
        try:
            if self.connected() and SESSIONS['gateway'] in self.remote.get('sessions', []):
                self.key('s')
        except Exception as exc:
            error = str(exc)
        if error:
            self.event('停止 SSH 命令失败；本机计算已停止，网关 lease 应过期：'+error)

    def disable_component(self, name):
        if name == 'head':
            # Call the verified ROS service directly.  The safety action must
            # remain available even when the optional camera web bridge is not
            # running; the service itself writes both axes and checks readback.
            script = (f'cd {shlex.quote(REMOTE)} && source /opt/ros/humble/setup.bash && '
                      'source install/setup.bash && timeout 7s ros2 service call '
                      '/head/torque_enable std_srvs/srv/SetBool "{data: false}"')
            output = self.command(['bash', '-lc', script], timeout=9)
            if not re.search(r'\bsuccess=(?:True|true)\b', output):
                detail = output.strip()[-1200:] or '头部驱动未运行或没有返回失能确认'
                raise RuntimeError('头部舵机失能未确认：'+detail)
            self.event('头部两轴舵机力矩已关闭，驱动回读 torque=0 确认')
            return
        keys = {'gateway': 'd', 'left_arm': 'dl', 'right_arm': 'dr', 'hands': 'dh'}
        if name not in keys:
            raise ValueError('该组件没有失能操作')
        # Individual disable is a special-case safety action: first stop local
        # target production and the whole gateway stream, then disable only the
        # selected actuator group. No remaining side may follow stale targets.
        self.stop_following()
        self.key(keys[name])
        self.event({'gateway':'双臂与双手全部失能', 'left_arm':'左臂单独失能',
                    'right_arm':'右臂单独失能', 'hands':'双手命令失能'}[name]+'已请求；请检查反馈')

    def action(self, data):
        action, session = data.get('action'), data.get('session')
        if not isinstance(session, str) or not re.fullmatch(r'[A-Za-z0-9-]{8,80}', session):
            raise ValueError('无效浏览器会话')
        if action == 'heartbeat':
            if session == self.owner:
                self.heartbeat = time.monotonic()
            return {}
        if action == 'stop' or (action == 'release' and session == self.owner):
            self.stop_following()
            return {}
        if action == 'remote_disable':
            self.disable_component(data.get('name'))
            return {}
        with self.lock:
            if action == 'connect':
                self.connect(data.get('password', ''))
            elif action == 'release':
                return {}
            elif self.owner and session != self.owner:
                raise RuntimeError('另一个页面正在控制；本页面仍可停止')
            elif action == 'pause_preview':
                if not self.running('teleop') or not self.mode.get('preview'):
                    raise RuntimeError('当前没有正在运行的预览计算')
                self.processes['teleop'].stop()
                self.owner = None
                STATUS_FILE.unlink(missing_ok=True)
                self.event('预览计算已暂停；机器人网关保持原状态')
            elif action == 'remote_start':
                if self.running('teleop'):
                    raise RuntimeError('先停止计算再修改机器人服务')
                self.remote_start(data.get('name'), validate_mode(data))
            elif action == 'remote_stop':
                name = data.get('name')
                if name not in SESSIONS:
                    raise ValueError('未知服务')
                self.refresh()
                if name in ('gateway', 'hands', 'hand_bridge'):
                    if self.running('teleop') or self.robot_state().get('mode') in ('ACTIVE', 'ARMING', 'RETURNING', 'RECOVERING'):
                        raise RuntimeError('请先停止遥操作并确认机器人停止')
                if name == 'gateway':
                    self.key('q')
                else:
                    if name == 'head':
                        # Never remove the only torque-control service while
                        # the physical head may still be holding torque.
                        self.disable_component('head')
                    self.command(['tmux', 'send-keys', '-t', SESSIONS[name], 'C-c'])
                self.event(f'已请求停止机器人 {name}')
            elif action == 'remote_log':
                name = data.get('name')
                if name not in SESSIONS:
                    raise ValueError('未知服务')
                if name == 'gateway':
                    self.refresh()
                    saved = self.remote.get('gateway_startup_log', '')
                    if saved:
                        return dict(log=saved)
                return dict(log=self.command(['tmux', 'capture-pane', '-p', '-t', SESSIONS[name], '-S', '-180']))
            elif action == 'can':
                return self.initialize_can(data.get('password', ''))
            elif action == 'remote_recover':
                if self.running('teleop'):
                    raise RuntimeError('先停止本机计算再恢复网关')
                self.refresh()
                state = self.robot_state()
                if (SESSIONS['gateway'] not in self.remote.get('sessions', [])
                        or state.get('age_s', 999) > 1 or state.get('mode') != 'FAULT'):
                    raise RuntimeError('恢复需要新鲜的 FAULT 网关状态')
                if state.get('recovery_supported') is not True:
                    raise RuntimeError('机器人网关尚未加载故障恢复功能；请同步新代码并重启一次网关')
                if state.get('stop_send_returned') is False:
                    raise RuntimeError('机器人停止发送失败；请先检查现场状态和网关日志')
                args = self.remote.get('gateway_args', [])
                if '--side' not in args or args[args.index('--side')+1] == 'both':
                    raise RuntimeError('双臂恢复需按现场流程操作；网页不执行未经验证的双臂回零')
                self.key('r')
                return dict(message='已请求网关恢复；请等待状态变为 IDLE，再重新开始计算和使能')
            elif action == 'local_start':
                if self.running('teleop') and data.get('name') != 'teleop':
                    raise RuntimeError('先停止计算再修改采集进程')
                self.local_start(data.get('name'), data, session)
            elif action == 'local_stop':
                name = data.get('name')
                if name not in LABELS:
                    raise ValueError('未知服务')
                if self.running('teleop'):
                    self.stop_following()
                if name == 'pc_service':
                    self.processes[name] = Process([str(ROOT/'scripts/xrobotoolkit_service.sh'), 'stop'])
                elif name in self.processes:
                    self.processes[name].stop()
            elif action == 'enter':
                name = data.get('name')
                if name not in ('glove', 'glove_ros') or name not in self.processes:
                    raise ValueError('该程序不接受标定回车')
                self.processes[name].enter()
            elif action == 'glove_config':
                if any(self.running(n) for n in ('glove', 'glove_ros', 'teleop')):
                    raise RuntimeError('先停止计算、手套桥与 ROS，再修改序列号')
                serials = [data.get(s+'_serial', '') for s in ('left','right')]
                if not all(re.fullmatch(r'[A-Za-z0-9._-]{1,60}', s) for s in serials):
                    raise ValueError('序列号格式无效')
                result = subprocess.run([str(ROOT/'scripts/configure_senseglove.sh'), *serials], capture_output=True, text=True, timeout=10)
                if result.returncode:
                    raise RuntimeError(result.stderr)
                self.event('手套序列号已配置')
            elif action == 'gateway_key':
                key = data.get('key')
                if key not in ('e', 's', 'x', 'd', 'dl', 'dr', 'dh', 'z', 'q'):
                    raise ValueError('未知网关命令')
                self.refresh()
                if key == 'e':
                    remote_args = self.remote.get('gateway_args', [])
                    if ('--side' in remote_args
                            and remote_args[remote_args.index('--side')+1] == 'both'):
                        raise RuntimeError(
                            '双臂自动准备尚无双臂互碰扫掠检查，已拒绝回零和使能；'
                            '请改用单臂模式或执行现场验证过的双臂恢复流程')
                    state, local = self.robot_state(), self.telemetry()
                    feedback = state.get('feedback', {})
                    members = feedback.get('sides', {self.mode['side']: feedback})
                    if (session != self.owner or not self.running('teleop') or self.mode['preview']
                            or local.get('age_s', 999) > 1 or not local.get('targets')
                            or local.get('sent', 0) < 1 or state.get('age_s', 999) > 1
                            or state.get('mode') != 'IDLE' or not members
                            or any(v.get('controller_fault') for v in members.values())):
                        raise RuntimeError('使能需要正式计算、持续有效目标、新鲜 IDLE 反馈且控制器无故障')
                    if self.mode.get('with_hand') and not self.mode.get('hand_only'):
                        incomplete = []
                        for side, member in members.items():
                            geometry = (member.get('hand') or {}).get('geometry') or {}
                            if not geometry.get('ready'):
                                count = geometry.get('calibrated_joints', 0)
                                total = geometry.get('total_joints', 10)
                                incomplete.append(
                                    f"{'左' if side == 'left' else '右'}手 {count}/{total}"
                                )
                        if incomplete:
                            raise RuntimeError(
                                '灵巧手碰撞几何标定不完整（' + '、'.join(incomplete) +
                                '），已在机器人运动前拒绝联合使能。可先选择仅机械臂或仅灵巧手；'
                                '机械臂与灵巧手联合运动需完成反馈值到 URDF 关节角的几何标定。'
                            )
                elif key in ('s', 'x'):
                    self.stop_following()
                    return {}
                elif key in ('d', 'dl', 'dr', 'dh', 'z', 'q') and self.running('teleop'):
                    raise RuntimeError('请先停止计算')
                if key == 'z' and '--side' in self.remote.get('gateway_args', []):
                    argv = self.remote['gateway_args']
                    if argv[argv.index('--side')+1] == 'both':
                        raise RuntimeError('双臂网关不支持自动回零；请按机器人恢复流程处理')
                self.key(key)
            else:
                raise ValueError('未知操作')
        return {}

    def head(self, method, path, body=None):
        if not self.connected():
            raise RuntimeError('请先连接机器人 SSH')
        channel = self.ssh.get_transport().open_channel('direct-tcpip', ('127.0.0.1', 8766), ('127.0.0.1', 0), timeout=3)
        channel.settimeout(3)
        response = None
        try:
            payload = body or b''
            headers = (f'{method} {path} HTTP/1.0\r\nHost: 127.0.0.1:8766\r\n'
                       'Origin: http://127.0.0.1:8766\r\nContent-Type: application/json\r\n'
                       f'X-Head-Control: 1\r\nContent-Length: {len(payload)}\r\n\r\n')
            channel.sendall(headers.encode('ascii')+payload)
            # HTTPConnection closes HTTP/1.0 sockets at getresponse(). Unlike a
            # TCP socket, Paramiko Channel.close also discards its file readers.
            # Keep this channel open until the entire response body is consumed.
            response = http.client.HTTPResponse(channel)
            response.begin()
            if response.length is None or not 0 <= response.length <= 4*1024*1024:
                raise RuntimeError('相机应答长度无效')
            return response.status, response.read(), response.getheader('Content-Type', 'application/json')
        finally:
            if response:
                response.close()
            channel.close()

    def pico(self, path):
        connection = http.client.HTTPConnection('127.0.0.1', 8765, timeout=2)
        try:
            connection.request('GET', path)
            response = connection.getresponse()
            body = response.read(4*1024*1024)
            return response.status, body, response.getheader('Content-Type', 'application/json')
        finally:
            connection.close()

    def _monitor(self):
        while not self.closed.wait(.5):
            self.pc_service = subprocess.run(['pgrep', '-f', '^/opt/apps/roboticsservice/RoboticsServiceProcess$'],
                                             stdout=subprocess.DEVNULL).returncode == 0
            if self.connected():
                try:
                    self.refresh()
                except Exception as exc:
                    self.remote_error = str(exc)
                    if self.running('teleop'):
                        self.stop_following()
            elif self.running('teleop'):
                self.remote_error = 'SSH 连接中断，已停止本机计算'
                self.stop_following()

    def _watchdog(self):
        # Independent of SSH polling and process actions: a blocked SSH read must
        # never keep local target generation alive after the browser disappears.
        while not self.closed.wait(.2):
            if self.owner and time.monotonic()-self.heartbeat > 4:
                self.event('浏览器心跳丢失，停止遥操作')
                self.stop_following()
            elif self.owner and not self.running('teleop'):
                self.owner = None

    def close(self):
        self.closed.set()
        if self.running('teleop'):
            self.stop_following()
        for proc in self.processes.values():
            proc.stop()
        if self.ssh:
            self.ssh.close()


def validate_mode(data):
    side = data.get('side', 'both')
    with_hand = data.get('with_hand', True)
    hand_only, preview = data.get('hand_only', False), data.get('preview', True)
    if (side not in ('both', 'left', 'right') or type(with_hand) is not bool
            or type(hand_only) is not bool or type(preview) is not bool):
        raise ValueError('遥操作模式无效')
    if side == 'both' and (not with_hand or hand_only):
        raise ValueError('当前机器人双臂网关包含双手，请选择双臂双手或单臂模式')
    if hand_only and not with_hand:
        raise ValueError('仅灵巧手模式必须启用手部')
    return dict(side=side, with_hand=with_hand, hand_only=hand_only, preview=preview)


def make_handler(console, port):
    assets = ROOT/'web'
    head_assets = WORKSPACE/'teleoperation/web/pico_skeleton_viewer'
    class Handler(BaseHTTPRequestHandler):
        def send(self, code, body, mime='application/json; charset=utf-8'):
            if not isinstance(body, bytes):
                body = json.dumps(body, ensure_ascii=False).encode()
            self.send_response(code)
            self.send_header('Content-Type', mime)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('X-Frame-Options', 'SAMEORIGIN')
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def host_ok(self):
            return self.headers.get('Host') in (f'127.0.0.1:{port}', f'localhost:{port}')

        def do_GET(self):
            if not self.host_ok():
                return self.send(403, {'error':'仅允许本机访问'})
            path = urlsplit(self.path).path
            try:
                if path == '/api/health':
                    return self.send(200, {'service': 'esrobo-laptop-dashboard'})
                if path == '/api/state':
                    return self.send(200, console.snapshot())
                if path == '/api/pico/state':
                    return self.send(*console.pico('/api/state'))
                if path in ('/api/head/state', '/api/head/color.jpg', '/api/head/depth.jpg'):
                    return self.send(*console.head('GET', path))
                files = {'/': (assets/'index.html', 'text/html; charset=utf-8'),
                         '/app.js': (assets/'app.js', 'text/javascript'), '/style.css': (assets/'style.css', 'text/css'),
                         '/head-embed.css': (assets/'head-embed.css', 'text/css')}
                for name, mime in (('head.html','text/html; charset=utf-8'), ('head.js','text/javascript'), ('head.css','text/css')):
                    files['/'+name] = (head_assets/name, mime)
                if path in files:
                    file, mime = files[path]
                    body = file.read_bytes()
                    if path == '/head.html':
                        body = body.replace(b'./index.html', b'http://127.0.0.1:8765/')
                    if path == '/head.css':
                        body += b'\nheader{display:none}main{padding:0;gap:18px;grid-template-columns:minmax(0,1fr) 250px}.controls{padding-left:18px}@media(max-width:850px){main{grid-template-columns:1fr}.controls{padding:12px 0}.axis{padding:12px 0}.axis-detail{min-height:18px}.actions{margin-top:10px}.safety{margin-bottom:4px}}'
                    return self.send(200, body, mime)
                if path.startswith('/pico/'):
                    relative = 'index.html' if path == '/pico/' else path.removeprefix('/pico/')
                    candidate = (head_assets / relative).resolve()
                    if head_assets.resolve() not in candidate.parents:
                        return self.send(404, {'error':'不存在'})
                    body = candidate.read_bytes()
                    if relative == 'app.js':
                        body = body.replace(b'fetch("/api/state"', b'fetch("/api/pico/state"')
                    mime = {'.html':'text/html; charset=utf-8', '.js':'text/javascript',
                            '.css':'text/css', '.map':'application/json'}.get(candidate.suffix, 'application/octet-stream')
                    return self.send(200, body, mime)
                self.send(404, {'error':'不存在'})
            except Exception as exc:
                self.send(503, {'error':str(exc)})

        def do_POST(self):
            if (not self.host_ok() or self.headers.get('Origin') != 'http://'+self.headers.get('Host', '')
                    or self.headers.get('Content-Type') != 'application/json'
                    or self.headers.get('X-Head-Control') != '1'):
                return self.send(403, {'error':'需要同源 JSON 请求'})
            try:
                size = int(self.headers.get('Content-Length', '0'))
                if not 0 < size <= 4096:
                    raise ValueError('无效请求大小')
                body = self.rfile.read(size)
                data = json.loads(body)
                if not isinstance(data, dict):
                    raise ValueError('需要 JSON 对象')
                path = urlsplit(self.path).path
                if path == '/api/head/disable':
                    console.disable_component('head')
                    return self.send(200, {'ok':True})
                if path in ('/api/head/heartbeat', '/api/head/lock', '/api/head/enable',
                            '/api/head/move'):
                    return self.send(*console.head('POST', path, body))
                if path != '/api/action':
                    return self.send(404, {'error':'不存在'})
                self.send(200, dict(ok=True, **console.action(data)))
            except Exception as exc:
                self.send(409, {'error':str(exc)})

        def log_message(self, *_):
            pass
    return Handler


def dashboard_is_running(port):
    """Identify this service without relying on proxy settings or creating a session."""
    def get(path):
        connection = http.client.HTTPConnection('127.0.0.1', port, timeout=1)
        try:
            connection.request('GET', path)
            response = connection.getresponse()
            return response.status, response.read(65536)
        finally:
            connection.close()

    try:
        status, body = get('/api/health')
        if status == 200 and json.loads(body).get('service') == 'esrobo-laptop-dashboard':
            return True
        # Recognize the already-running first version without restarting it.
        status, body = get('/')
        if status != 200 or '<title>ESROBO · 遥操作控制台</title>'.encode() not in body:
            return False
        status, body = get('/api/state')
        data = json.loads(body)
        return (status == 200 and isinstance(data.get('connected'), bool)
                and all(isinstance(data.get(k), dict) for k in ('remote', 'processes', 'mode')))
    except (OSError, ValueError, AttributeError, http.client.HTTPException):
        return False


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8080)
    args = parser.parse_args(argv)
    if not 1 <= args.port <= 65535:
        parser.error('--port 必须介于 1 和 65535 之间')
    # Bind before starting any monitoring threads or subprocess management.
    try:
        server = ThreadingHTTPServer(('127.0.0.1', args.port), make_handler(None, args.port))
    except OSError as exc:
        if exc.errno != errno.EADDRINUSE:
            raise
        if dashboard_is_running(args.port):
            print(f'ESROBO 控制台已在运行，请直接打开：http://127.0.0.1:{args.port}', flush=True)
            print('本次未重复启动服务；现有连接和程序继续运行。', flush=True)
            return 0
        print(f'端口 {args.port} 已被其他程序占用或现有服务未响应。请检查占用，或使用 --port 指定空闲端口。', flush=True)
        return 1
    try:
        console = Console()
    except BaseException:
        server.server_close()
        raise
    server.RequestHandlerClass = make_handler(console, args.port)
    server.daemon_threads = True
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    print(f'ESROBO 遥操作控制台：http://127.0.0.1:{args.port}（启动页面不会使能机器人）', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        console.close()


if __name__ == '__main__':
    raise SystemExit(main())
