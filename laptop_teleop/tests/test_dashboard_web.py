"""Execute the actual web handlers/rendering with inert DOM and network stubs."""
import json
from pathlib import Path
import shutil
import subprocess

import pytest

from esrobo_laptop.config import ROOT
from esrobo_laptop.contract import REQUIRED_INPUT_HOLD_PROTOCOL


def test_recovery_never_sends_implicit_stop_or_disable():
    node = shutil.which('node') or '/usr/lib/chatgpt/resources/cua_node/bin/node'
    if not Path(node).exists():
        pytest.skip('Node required for web action regression')
    script = r'''
const fs = require('node:fs'), vm = require('node:vm');
const app = fs.readFileSync(process.argv[1], 'utf8');
(async () => {
const results = {};
for (const scenario of ['left', 'right', 'disabled', 'hand_only', 'declined',
                       'legacy', 'estop', 'torque', 'mixed', 'send_failed', 'double_click']) {
  const nodes = new Map();
  const element = id => {
    if (['handover', 'remote-diagnostics', 'remote-sync'].includes(id)) return null;
    if (!nodes.has(id)) nodes.set(id, {value: 'left-hand', options: [], dataset: {}, style: {},
      classList: {toggle() {}}, append() {}, before() {}, after() {}, replaceChildren() {},
      setAttribute() {}, addEventListener() {}});
    return nodes.get(id);
  };
  const calls = [], messages = [];
  const side = scenario === 'right' ? 'right' : 'left';
  const arm = {arm_feedback_age_s: .01, arm_urdf_rad: Array(7).fill(0),
               enable_states: Array(7).fill(scenario !== 'disabled')};
  if (scenario === 'estop') arm.controller_fault = {status_name:'EMERGENCY_STOP'};
  if (scenario === 'torque') arm.driver_fault = 'J7 delta=9.50Nm, limit=5.00Nm';
  if (scenario === 'mixed') arm.enable_states[0] = false;
  const fixture = {connected: true, mode: {side, with_hand: true, hand_only: scenario === 'hand_only'},
    robot: {contract_id:'test', mode:scenario === 'disabled' ? 'FAULT':'HOLD_LATCHED', age_s:.01,
      hold_return_protocol: scenario === 'legacy' ? undefined : 1,
      feedback: {sides: {[side]: arm}}}, processes: {}};
  const context = vm.createContext({document: {getElementById: element,
      createElement: () => element(Symbol()), querySelectorAll: () => [], addEventListener() {}},
    window: {addEventListener() {}}, crypto: {randomUUID: () => 'test'},
    performance: {now: () => 0}, AbortSignal: {timeout: () => undefined},
    setInterval() {}, setTimeout() {}, fetch: () => new Promise(() => {}),
    confirm: () => scenario !== 'declined', fixture, calls, messages, scenario});
  vm.runInContext(app + '\nstate=fixture; render=()=>{}; notice=(m)=>messages.push(m);', context);
  vm.runInContext(`api=async (action, values)=>{
    calls.push(action);
    if (scenario==='send_failed') throw new Error('request failed');
    if (scenario==='double_click') await recoverGateway();
    return {message:'submitted'};
  };`, context);
  await vm.runInContext('recoverGateway()', context);
  results[scenario] = {calls, messages, busy:vm.runInContext('recoveryBusy', context)};
  if (scenario === 'left') {
    vm.runInContext('buttonActions=[]; perform=async action=>buttonActions.push(action)', context);
    await nodes.get('motion-stop').onclick();
    await nodes.get('stop').onclick();
    results[scenario].buttonActions = vm.runInContext('buttonActions', context);
  }
}
process.stdout.write(JSON.stringify(results));
})().catch(error => {process.stderr.write(String(error)); process.exitCode=1;});
'''
    result = subprocess.run([node, '-e', script, str(ROOT / 'web/app.js')],
                            capture_output=True, text=True, check=True, timeout=20)
    results = json.loads(result.stdout)
    assert results['left']['buttonActions'] == ['motion_hold', 'stop']
    for scenario, row in results.items():
        assert row['busy'] is False
        assert 'remote_disable' not in row['calls']
        assert row['calls'] == ([] if scenario in ('declined', 'legacy', 'estop', 'torque', 'mixed')
                                else ['remote_recover']), scenario
    for scenario in ('legacy', 'estop', 'mixed', 'send_failed'):
        assert results[scenario]['messages']
    assert '9.50Nm' in results['torque']['messages'][0]
    assert '单独失能' in results['torque']['messages'][0]
    assert '回零尚未启动' in results['estop']['messages'][0]


def test_start_buttons_accept_current_hold_protocol_and_keep_other_interlocks():
    node = shutil.which('node')
    bundled = Path('/usr/lib/chatgpt/resources/cua_node/bin/node')
    if node is None and bundled.exists():
        node = str(bundled)
    if node is None:
        pytest.skip('Node required for web rendering regression')
    script = r'''
const fs = require('node:fs');
const vm = require('node:vm');
const app = fs.readFileSync(process.argv[1], 'utf8');
const required = Number(process.argv[2]);
const result = [];
for (const scenario of ['current', 'idle_estop', 'old_return', 'missing_can_feedback', 'hand_return_refused', 'hand_return_final', 'async_diagnostics', 'async_error', 'async_stale', 'entry_diagnostics', 'hand_transition', 'wrist_wait', 'wrist_transition', 'roll_diagnostics', 'absolute_diagnostics', 'old', 'missing', 'old_entry', 'missing_entry_keepalive', 'approach_stale', 'arrival_wait', 'fault', 'stale', 'missing_feedback', 'running']) {
  const nodes = new Map();
  function element(id) {
    if (!nodes.has(id)) nodes.set(id, {
      value: id === 'mode' ? 'right-hand' : '', options: [{value: 'right-hand'}],
      classList: {toggle() {}}, dataset: {}, style: {},
      append() {}, before() {}, after() {}, replaceChildren() {},
      setAttribute() {}, addEventListener() {},
    });
    return nodes.get(id);
  }
  const fixture = {
    connected: true, remote_age_s: .01, owner: null,
    remote: {sessions: ['esrobo_gateway'], gateway_args: ['--side', 'right']},
    mode: {side: 'right', with_hand: true, hand_only: false},
    robot: {mode: 'IDLE', age_s: .01, return_protocol: 2, entry_protocol: 6, entry_keepalive_protocol: 1, input_hold_protocol: required,
      feedback: {arm_urdf_rad: [0, 0, 0, 0, 0, 0, 0], arm_feedback_age_s: .01,
        enable_states: [false, false, false, false, false, false, false],
        hand: {geometry: {ready: true, calibrated_joints: 10, total_joints: 10}}}},
    telemetry: {}, processes: {}, capabilities: {}, events: [],
  };
  if (scenario === 'old') fixture.robot.input_hold_protocol = 1;
  if (scenario === 'missing') delete fixture.robot.input_hold_protocol;
  if (scenario === 'fault') fixture.robot.mode = 'FAULT';
  if (scenario.startsWith('hand_return_')) {
    fixture.robot.mode = 'FAULT';
    fixture.robot.last_return = {returned: false, disabled: true,
      arm_returned: scenario === 'hand_return_final',
      stage: scenario === 'hand_return_final' ? 'hand_open_verify' : 'hand_open_before_arm',
      reason: 'hand feedback not verified'};
  }
  if (['idle_estop', 'old_return', 'missing_can_feedback'].includes(scenario)) {
    fixture.robot.feedback.controller_fault = {status_name: 'EMERGENCY_STOP'};
    fixture.robot.recovery_supported = true;
    fixture.capabilities.fault_recovery = true;
  }
  if (scenario === 'old_return') delete fixture.robot.return_protocol;
  if (scenario === 'missing_can_feedback') {
    fixture.robot.feedback.controller_fault = {category: 'controller_feedback_stale'};
    fixture.robot.feedback.arm_urdf_rad = null;
    fixture.robot.feedback.enable_states = null;
  }
  if (scenario === 'stale') fixture.robot.age_s = 10;
  if (scenario === 'missing_feedback') fixture.robot.feedback.arm_urdf_rad = [];
  if (scenario === 'running') fixture.processes.teleop = {running: true};
  if (scenario === 'old_entry') fixture.robot.entry_protocol = 5;
  if (scenario === 'missing_entry_keepalive') delete fixture.robot.entry_keepalive_protocol;
  if (['approach_stale', 'arrival_wait'].includes(scenario)) {
    fixture.robot.mode = scenario === 'approach_stale' ? 'APPROACHING' : 'WAITING_FOR_ACTIVE_TARGET';
    fixture.robot.entry_keepalive_age_s = .02;
    fixture.telemetry = {age_s: .01, required_sources: ['right_arm'], sources: {right_arm: .35}};
  }
  if (scenario === 'roll_diagnostics') fixture.telemetry = {
    age_s: .01, pico_wrist_input: {right: {requested_delta_deg: 35., requested_roll_deg: 35.,
      filtered_roll_deg: 30., palm_comparison_status: 'paired', hand_palm_roll_deg: 35.5}},
    wrist_mapping: {right: {target_joint_deg: [29., 1., 0.], mapped_orientation_error_deg: .2}},
  };
  if (scenario === 'absolute_diagnostics') fixture.telemetry = {
    age_s: .01,
    pico_wrist_input: {right: {orientation_mode: 'forearm_absolute', requested_delta_deg: 125.,
      requested_roll_deg: -125., filtered_roll_deg: -124., palm_comparison_status: 'paired',
      hand_palm_roll_deg: -125., body_palm_disagreement_deg: 15.}},
    wrist_mapping: {right: {target_joint_deg: [-124., 20., -15.], mapped_orientation_error_deg: .01,
      joint_lower_deg: [-150., -38., -80.], joint_upper_deg: [150., 50., 80.]}},
  };
  if (scenario === 'hand_transition') {
    fixture.robot.mode = 'ACTIVE';
    fixture.telemetry = {age_s: .01, input_hold_active: true, input_hold_age_s: .2,
      input_resume_diagnostics: {reason: 'fresh matching frames 1/3'},
      hand_handover: {active: true, source_target: [239], command: [150]}};
  }
  if (scenario.startsWith('wrist_')) {
    fixture.robot.mode = 'ACTIVE';
    fixture.telemetry = {age_s: .01, input_hold_active: scenario === 'wrist_wait',
      wrist_handover: {active: true, phase: scenario === 'wrist_wait' ? 'waiting_first_input' : 'transition'}};
  }
  if (scenario.startsWith('async_')) fixture.telemetry = {
    age_s: scenario === 'async_stale' ? 2 : .01,
    rates: {log_publish_ms_previous: .3},
    timings: {arm_ik_right_ms: {p95_ms: 4., max_ms: 8.}},
    diagnostics: {write_ms_previous: 100., files: {pending: 1, coalesced: 2, dropped: 3},
      console: {pending: 4, coalesced: 5, dropped: 6}, serialization_errors: 1,
      errors: {journal: scenario === 'async_error' ? 'disk unavailable' : null}},
  };
  if (scenario === 'entry_diagnostics') fixture.robot.entry_execution = {
    active: true, send_hz: 50., send_gap_p95_ms: 20.2, send_gap_max_ms: 22.,
    geometry_p95_ms: 3., cache_hits: 100, overruns: 2, skipped_ticks: 3, dropped: 4,
    log_path: '/robot/entry.jsonl',
  };
  const context = {
    fixture, document: {getElementById: element, createElement: tag => element(Symbol(tag)),
      createTextNode: text => ({textContent: text}), querySelectorAll: () => []},
    window: {addEventListener() {}}, crypto: {randomUUID: () => 'test-web-session'},
    performance: {now: () => 0}, AbortSignal: {timeout() {}},
    setInterval() {}, setTimeout() {}, fetch: () => new Promise(() => {}),
  };
  vm.createContext(context);
  vm.runInContext(app + '\nstate = fixture; render();', context);
  result.push({scenario, runDisabled: element('run').disabled,
    robotReason: element('robot-reason').textContent,
    previewDisabled: element('preview').disabled, recoverDisabled: element('recover').disabled,
    recoveryDetail: element('recovery-detail').textContent,
    protocolReady: vm.runInContext('gatewayProtocolReady()', context),
    entryExecution: element('entry-execution-status').textContent,
    diagnostics: element('diagnostics-status').textContent, diagnosticTitle: element('diagnostics-status').title,
    runtime: element('runtime-status').textContent, hint: element('entry-flow-hint').textContent, wrist: element('wrist-diagnostics').textContent});
}
process.stdout.write(JSON.stringify(result));
'''
    completed = subprocess.run([node, '-e', script, str(ROOT / 'web/app.js'),
                                str(REQUIRED_INPUT_HOLD_PROTOCOL)],
                               text=True, capture_output=True, timeout=10)
    assert completed.returncode == 0, completed.stderr
    results = {item['scenario']: item for item in json.loads(completed.stdout)}
    current = results['current']
    assert current['runDisabled'] is False
    assert current['previewDisabled'] is False
    assert current['protocolReady'] is True
    assert results['idle_estop']['runDisabled'] is True
    assert results['idle_estop']['previewDisabled'] is False
    assert results['idle_estop']['recoverDisabled'] is False
    assert results['old_return']['recoverDisabled'] is True
    assert '旧版受检恢复' in results['old_return']['recoveryDetail']
    assert results['missing_can_feedback']['recoverDisabled'] is False
    assert 'CAN 反馈缺失' in results['missing_can_feedback']['recoveryDetail']
    assert '灵巧手先行归零' in results['hand_return_refused']['robotReason']
    assert '机械臂七轴回零尚未开始或未完成' in results['hand_return_refused']['robotReason']
    assert '七轴回零后的手部零位核验' in results['hand_return_final']['robotReason']
    assert '尚未开始或未完成' not in results['hand_return_final']['robotReason']
    assert '控制器故障尚未解除' in results['idle_estop']['recoveryDetail']
    assert results['roll_diagnostics']['runDisabled'] is False
    assert '翻掌输入 35.0°／限幅限步后 30.0°' in results['roll_diagnostics']['wrist']
    assert 'J5 目标 29.0°／实测 0.0°' in results['roll_diagnostics']['wrist']
    assert 'Hand 掌面翻掌 35.5°' in results['roll_diagnostics']['wrist']
    assert 'Hand 绝对掌面（前臂坐标）' in results['absolute_diagnostics']['wrist']
    assert '绝对翻掌 -125.0°／限步后 -124.0°' in results['absolute_diagnostics']['wrist']
    assert 'Body/Hand 姿态差 15.0°（控制采用 Hand）' in results['absolute_diagnostics']['wrist']
    assert 'J5 -150～150°' in results['absolute_diagnostics']['wrist']
    for scenario in ('old', 'missing', 'old_entry', 'missing_entry_keepalive', 'approach_stale', 'arrival_wait', 'fault', 'stale', 'missing_feedback', 'running'):
        assert results[scenario]['runDisabled'] is True, scenario
        assert results[scenario]['previewDisabled'] is True, scenario
    for scenario in ('old', 'missing'):
        assert results[scenario]['protocolReady'] is False
        assert f'需要 {REQUIRED_INPUT_HOLD_PROTOCOL}' in results[scenario]['hint']
    assert results['old_entry']['protocolReady'] is False
    assert '需要 6' in results['old_entry']['hint']

    assert results['missing_entry_keepalive']['protocolReady'] is False
    assert '保活协议 1' in results['missing_entry_keepalive']['hint']
    assert '手指首次到位过渡' in results['hand_transition']['runtime']
    assert '网关已发' in results['hand_transition']['runtime']
    assert '首次接管等待新鲜输入' in results['wrist_wait']['runtime']
    assert 'J6/J7 首次限速接管' in results['wrist_transition']['runtime']
    assert '目标 239 / 本机指令 150' in results['hand_transition']['runtime']
    assert '恢复检查：fresh matching frames 1/3' in results['hand_transition']['runtime']
    assert 'PICO 暂无新数据' in results['approach_stale']['runtime']
    assert '350 ms' in results['approach_stale']['runtime']
    assert '网关保活年龄 20 ms' in results['approach_stale']['runtime']
    assert '最长 2 秒' in results['arrival_wait']['runtime']

    assert '耗时' not in results['async_diagnostics']['diagnostics']
    assert '异步日志' in results['async_diagnostics']['diagnostics']
    assert '待处理 5 / 合并 7 / 丢弃 10' in results['async_diagnostics']['diagnostics']
    assert results['async_diagnostics']['diagnosticTitle'] == ''
    assert 'disk unavailable' in results['async_error']['diagnostics']
    assert '诊断状态已过期' in results['async_stale']['diagnostics']

    assert '发送 50.0 Hz' in results['entry_diagnostics']['entryExecution']
    assert 'P95' not in results['entry_diagnostics']['entryExecution']
    assert '缓存命中 100' in results['entry_diagnostics']['entryExecution']
    assert '超期 2 / 跳过时刻 3' in results['entry_diagnostics']['entryExecution']
