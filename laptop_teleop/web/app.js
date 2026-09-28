const $ = id => document.getElementById(id);
const session = crypto.randomUUID();
let state = null;
let selectedLog = 'events';
let pollRunning = true;
let logRequest = false;
let modeInitialized = false;
let canBusy = false;
let waistAxes = null;
let waistReadAt = 0;
let waistReference = null;
let waistLimit = null;
let waistSingleTurnMin = null;
let waistSingleTurnMax = null;
let waistMoving = false;
let waistMotionToken = 0;
let waistAutoReading = false;
let waistAutoRetryAt = 0;
let waistServerPending = false;

const remoteServices = {
  gateway: {label: '双臂遥操作网关', session: 'esrobo_gateway', disable: '双臂双手失能'},
  left_arm: {label: '左臂驱动', virtual: true, disable: '单独失能'},
  right_arm: {label: '右臂驱动', virtual: true, disable: '单独失能'},
  hands: {label: '双手 ROS 驱动', session: 'esrobo_hands', disable: '双手命令失能'},
  hand_bridge: {label: '双手通信桥', session: 'esrobo_hand_bridge'},
  waist: {label: '腰部三轴驱动', session: 'esrobo_waist'},
  head: {label: '头部两轴驱动', session: 'esrobo_head', disable: '舵机失能'},
  camera: {label: 'RGB / 深度相机', session: 'esrobo_camera'},
  head_web: {label: '相机网页服务', session: 'esrobo_head_web'},
};
const localNames = {
  pc_service: 'PICO PC Service', pico: 'PICO 全身 + 双手采集', check: '环境检查',
};

function options() {
  const [side, ...parts] = $('mode').value.split('-');
  const kind = parts.join('-');
  return {session, side, with_hand: kind !== 'arm', hand_only: kind === 'hand-only', preview: false};
}

function modeValue(mode) {
  if (!mode || !['left', 'right', 'both'].includes(mode.side)) return null;
  if (mode.hand_only) return `${mode.side}-hand-only`;
  return `${mode.side}-${mode.with_hand ? 'hand' : 'arm'}`;
}

function notice(message, error = false) {
  $('notice').textContent = message;
  $('notice').classList.toggle('error', error);
}

async function api(action, values = {}) {
  const response = await fetch('/api/action', {method: 'POST',
    headers: {'Content-Type': 'application/json', 'X-Head-Control': '1'},
    body: JSON.stringify({...options(), action, ...values}),
    signal: AbortSignal.timeout(action === 'can' ? 90000 : 18000)});
  const data = await response.json();
  if (!response.ok) throw new Error(data.error || `HTTP ${response.status}`);
  return data;
}

async function perform(action, values = {}, button = null) {
  if (button) button.disabled = true;
  try {
    const data = await api(action, values);
    notice(data.message || '操作已提交，请查看实时状态和日志确认执行结果。');
    if (data.log) $('log').textContent = data.log;
    return data;
  } catch (error) {
    notice(error.message, true);
  } finally {
    if (button) button.disabled = false;
  }
}

function element(tag, text, className) {
  const item = document.createElement(tag);
  if (text !== undefined) item.textContent = text;
  if (className) item.className = className;
  return item;
}

function selectLog(name) {
  selectedLog = name;
  $('log-source').value = name;
  updateLog();
}

function addLogOption(value, label) {
  const option = element('option', label);
  option.value = value;
  $('log-source').append(option);
}

function buildRemoteServices() {
  for (const [name, spec] of Object.entries(remoteServices)) {
    const card = element('div', undefined, 'service');
    const heading = element('div', undefined, 'service-heading');
    heading.append(element('span', spec.label));
    const status = element('span', '未启动', 'service-status');
    status.id = `status-r-${name}`;
    heading.append(status);
    card.append(heading);
    const actions = element('div', undefined, 'service-actions');
    if (!spec.virtual) {
      const start = element('button', '启动');
      start.onclick = async () => { await perform('remote_start', {name}, start); selectLog(`r:${name}`); };
      const stop = element('button', '停止');
      stop.onclick = async () => {
        if (name === 'waist' && !confirm('将先请求腰部减速停止，再退出腰部驱动；不会发送电机失能命令。确认停止驱动？')) return;
        await perform('remote_stop', {name}, stop); selectLog(`r:${name}`);
      };
      actions.append(start, stop);
    }
    if (spec.disable) {
      const disable = element('button', spec.disable, 'disable-action');
      disable.onclick = async () => {
        const detail = name === 'head' ? '关闭头部两轴力矩' : `${spec.label}失能并停止当前遥操作`;
        if (!confirm(`确认执行：${detail}？执行后必须重新检查状态才能继续遥操作。`)) return;
        await perform('remote_disable', {name}, disable);
        selectLog(`r:${name}`);
      };
      actions.append(disable);
    }
    const logs = element('button', '日志');
    logs.onclick = () => selectLog(`r:${name}`);
    actions.append(logs);
    card.append(actions);
    $('remote-services').append(card);
    addLogOption(`r:${name}`, `机器人 · ${spec.label}`);
  }
}

function buildLocalServices() {
  for (const [name, label] of Object.entries(localNames)) {
    const card = element('div', undefined, 'service');
    const heading = element('div', undefined, 'service-heading');
    heading.append(element('span', label));
    const status = element('span', '未启动', 'service-status');
    status.id = `status-l-${name}`;
    heading.append(status);
    card.append(heading);
    const actions = element('div', undefined, 'service-actions');
    for (const [title, action] of [['启动', 'local_start'], ['停止', 'local_stop']]) {
      const button = element('button', title);
      button.onclick = async () => { await perform(action, {name}, button); selectLog(`l:${name}`); };
      actions.append(button);
    }
    const logs = element('button', '日志');
    logs.onclick = () => selectLog(`l:${name}`);
    actions.append(logs);
    card.append(actions);
    $('local-list').append(card);
    addLogOption(`l:${name}`, `本机 · ${label}`);
  }
}

buildRemoteServices();
buildLocalServices();
addLogOption('l:teleop', '本机 · 重定向 / IK');
const picoLive = element('p', '等待 PICO Body / Hand 状态', 'prompt');
picoLive.id = 'pico-live-state';
$('calibration-prompt').before(picoLive);
const handPreview = element('div', undefined, 'hint');
handPreview.id = 'hand-preview';
$('source-state').after(handPreview);

function tab(local) {
  $('local-services').hidden = !local;
  $('robot-services').hidden = local;
  $('tab-local').setAttribute('aria-selected', String(local));
  $('tab-robot').setAttribute('aria-selected', String(!local));
}
$('tab-robot').onclick = () => tab(false);
$('tab-local').onclick = () => tab(true);

$('connect-form').onsubmit = async event => {
  event.preventDefault();
  const password = $('password').value;
  $('password').value = '';
  await perform('connect', {password}, event.submitter);
};
$('can').onclick = () => $('can-dialog').showModal();
$('can-dialog').addEventListener('close', async () => {
  const password = $('sudo-password').value;
  $('sudo-password').value = '';
  if ($('can-dialog').returnValue !== 'confirm' || canBusy) return;
  canBusy = true;
  try {
    await perform('can', {password}, $('can'));
  } finally {
    canBusy = false;
    $('can').disabled = !state?.connected || !state?.capabilities?.automatic_can_teardown;
  }
});
$('preview').onclick = async () => { await perform('local_start', {name: 'teleop', preview: true}); selectLog('l:teleop'); };
$('pause-preview').onclick = () => perform('pause_preview');
$('run').onclick = async () => { await perform('local_start', {name: 'teleop', preview: false}); selectLog('l:teleop'); };
$('stop').onclick = () => perform('stop');
$('motion-stop').onclick = () => perform('stop');
$('stop-compute').onclick = () => perform('local_stop', {name: 'teleop'});
const WAIST_SINGLE_TURN_BATTERY_WARNING = 0x0C26;
const waistErrorAllowsMotion = code => code === 0 || code === WAIST_SINGLE_TURN_BATTERY_WARNING;
function waistTargetRange(position) {
  if (!Number.isFinite(position) || !Number.isFinite(waistLimit)
      || !Number.isFinite(waistSingleTurnMin) || !Number.isFinite(waistSingleTurnMax)
      || position < waistSingleTurnMin || position > waistSingleTurnMax) return null;
  const displayedPosition = Math.round(position * 100) / 100;
  return {
    min: Number(Math.max(waistSingleTurnMin, displayedPosition - waistLimit).toFixed(2)),
    max: Number(Math.min(waistSingleTurnMax, displayedPosition + waistLimit).toFixed(2)),
  };
}
function showWaistFeedback(result) {
  waistAxes = result.axes;
  waistReference = result.reference_deg;
  waistLimit = result.limit_offset_deg;
  waistSingleTurnMin = result.single_turn_min_deg;
  waistSingleTurnMax = result.single_turn_max_deg;
  waistServerPending = Boolean(result.motion_pending);
  waistReadAt = performance.now();
  waistAutoRetryAt = 0;
  for (const axis of [31, 32, 33]) {
    const feedback = result.axes?.[axis];
    $(`waist-value-${axis}`).textContent = feedback
      ? `${feedback.position_deg.toFixed(2)}° · 错误码 0x${feedback.error.toString(16).toUpperCase().padStart(4, '0')}` : '反馈缺失';
  }
  const outside = [31, 32, 33].filter(axis =>
    !waistTargetRange(result.axes?.[axis]?.position_deg));
  const blocking = [31, 32, 33].filter(axis => !waistErrorAllowsMotion(result.axes?.[axis]?.error));
  const singleTurnWarning = [31, 32, 33].some(axis => result.axes?.[axis]?.error === WAIST_SINGLE_TURN_BATTERY_WARNING);
  if (outside.length || blocking.length) {
    $('waist-detail').textContent = `反馈已读取；${outside.length ? `关节 ${outside.join('、')} 反馈超出单圈范围。` : ''}${blocking.length ? `关节 ${blocking.join('、')} 有不允许运动的错误码。` : ''}已锁定使能及调整。`;
  } else if (!waistMoving) {
    $('waist-detail').textContent = (singleTurnWarning ? '0x0C26 为多圈编码器电池报码；按单圈范围使用，每次上电须核对腰部实际姿态。' : '') + (waistServerPending
      ? '上一腰部目标尚未确认到位；请等待反馈，必要时停止腰部运动。'
      : state && !state.waist_enable_requested
        ? '反馈已读取；请先点击“使能腰部三轴”，再调整角度。现有驱动不提供使能状态回读。'
        : '反馈已读取且在当前调整范围内；可连续点按小步按钮。现有腰部驱动不提供使能状态回读。');
  }
  if (state) render();
}

async function waistAction(command, values = {}, button = null) {
  const isMotion = command === 'jog' || command === 'target';
  if (command === 'stop') {
    waistMotionToken++;
    waistMoving = false;
    waistReadAt = 0;
    if (state) render();
  }
  if (isMotion && waistMoving) return;
  const token = isMotion ? ++waistMotionToken : waistMotionToken;
  if (isMotion) {
    waistMoving = true;
    if (state) render();
  }
  const result = await perform('waist', {command, ...values}, button);
  if (!result) {
    if (isMotion && token === waistMotionToken) {
      waistMoving = false;
      waistReadAt = 0;
      waistAutoRetryAt = performance.now() + 2000;
      waistServerPending = true;
      $('waist-detail').textContent = '腰部目标请求未确认；已锁定调整并将自动重读反馈，请检查现场运动，必要时停止腰部运动。';
      if (state) render();
    }
    return;
  }
  if (command === 'status') {
    showWaistFeedback(result);
  } else if (command === 'stop') {
    waistServerPending = false;
  } else if (command === 'enable') {
    if (state) {
      state.waist_enable_requested = true;
      render();
    }
  } else if (isMotion) {
    if (token !== waistMotionToken) {
      try { await api('waist', {command: 'stop'}); }
      catch (error) { notice(`腰部停止请求未确认：${error.message}`, true); }
      return;
    }
    $('waist-detail').textContent = `${result.axis} 号已发送 ${result.target_deg.toFixed(2)}° 目标，等待实测到位…`;
    waistServerPending = true;
    let confirmed = 0;
    const timeoutMs = command === 'target' ? 30000 : 20000;
    const startedAt = performance.now();
    const deadline = startedAt + timeoutMs;
    const tolerance = Math.min(0.2, Math.max(0.03, Math.abs(result.target_deg - result.from_deg) / 2));
    try {
      while (token === waistMotionToken && performance.now() < deadline) {
        await new Promise(resolve => setTimeout(resolve, 400));
        if (token !== waistMotionToken) return;
        const feedback = await api('waist', {command: 'status'});
        if (token !== waistMotionToken) return;
        showWaistFeedback(feedback);
        const position = feedback.axes?.[result.axis]?.position_deg;
        const fault = [31, 32, 33].some(axis => !waistErrorAllowsMotion(feedback.axes?.[axis]?.error));
        if (fault) throw new Error('腰部出现不允许运动的错误码；已锁定调整，请检查反馈');
        if (performance.now() - startedAt >= 4000 && Number.isFinite(position)
            && Math.abs(position - result.from_deg) < 0.05
            && Math.abs(result.target_deg - result.from_deg) >= 0.05) {
          await api('waist', {command: 'stop'});
          waistServerPending = false;
          throw new Error('腰部目标未产生可测运动，已请求停止。请核对腰部使能与实际状态，再读取反馈。');
        }
        confirmed = !feedback.motion_pending && Number.isFinite(position)
          && Math.abs(position - result.target_deg) <= tolerance ? confirmed + 1 : 0;
        if (confirmed >= 2) {
          $('waist-detail').textContent = `${result.axis} 号实测已到 ${position.toFixed(2)}°，可以继续点按调整。`;
          return;
        }
      }
      if (token === waistMotionToken) throw new Error(`${timeoutMs / 1000} 秒内未确认腰部到位；请停止运动并检查现场反馈`);
    } catch (error) {
      if (token === waistMotionToken) {
        waistReadAt = 0;
        waistAutoRetryAt = performance.now() + 2000;
        waistServerPending = true;
        for (const axis of [31, 32, 33]) $(`waist-value-${axis}`).textContent = '反馈等待自动重读';
        $('waist-detail').textContent = `${error.message}；已锁定调整，正在自动重读反馈。`;
        notice(error.message, true);
      }
    } finally {
      if (token === waistMotionToken) {
        waistMoving = false;
        if (state) render();
      }
    }
  }
}
$('waist-read').onclick = () => waistAction('status', {}, $('waist-read'));
$('waist-enable').onclick = () => {
  if (confirm('确认机械臂已失能、躯干已托稳、急停可触及，并已核对腰部实际姿态与显示角度一致？将请求同时使能腰部三轴。'))
    waistAction('enable', {}, $('waist-enable'));
};
$('waist-stop').onclick = () => waistAction('stop', {}, $('waist-stop'));
for (const button of document.querySelectorAll('[data-waist-axis]')) button.onclick = () =>
  waistAction('jog', {axis: Number(button.dataset.waistAxis),
                       direction: Number(button.dataset.direction)}, button);
for (const form of document.querySelectorAll('[data-waist-target-form]')) {
  form.noValidate = true;
  form.querySelector('input').addEventListener('input', () => { if (state) render(); });
  form.onsubmit = event => {
  event.preventDefault();
  const input = form.querySelector('input');
  const target = Number(input.value);
  if (!input.validity.valid || !Number.isFinite(target)) {
    const axis = Number(form.dataset.waistTargetForm);
    const range = waistTargetRange(waistAxes?.[axis]?.position_deg);
    notice(range ? `腰关节 ${axis} 目标须在 ${range.min.toFixed(2)}°～${range.max.toFixed(2)}° 内；当前输入未发送。`
      : '请先读取该关节反馈，再输入允许范围内的目标角度。', true);
    return;
  }
  waistAction('target', {axis: Number(form.dataset.waistTargetForm), target_deg: target},
    form.querySelector('button'));
  };
}
$('recover').onclick = () => {
  const handOnly = Boolean(state?.mode?.hand_only);
  const message = handOnly
    ? '确认停止计算且机器人周围安全？将核验机械臂七轴失能和手部反馈；不会发送回零运动。'
    : '确认现场安全且实体急停可触及？单臂故障恢复会执行受检回零，机器人将运动。';
  if (confirm(message)) perform('remote_recover', {}, $('recover'));
};
$('disable-all').onclick = () => {
  if (confirm('确认停止当前目标流并失能双臂与双手？腰部需在独立控制区处理。')) perform('remote_disable', {name: 'gateway'});
};
$('enable').onclick = () => {
  if (confirm('确认机器人周围安全且实体急停可触及？机器人将先回零并张开灵巧手；随后请按页面提示摆好 PICO 准备姿态，检测通过后自动开始跟随。'))
    perform('gateway_key', {key: 'e'});
};
for (const button of document.querySelectorAll('[data-key]')) button.onclick = () => {
  const key = button.dataset.key;
  if (key === 'z' && !confirm('单臂受检回零会产生真实运动。确认现场安全且已按机器人恢复流程检查？')) return;
  perform('gateway_key', {key});
};
$('log-source').onchange = () => { selectedLog = $('log-source').value; updateLog(); };
$('refresh-log').onclick = updateLog;

function showVision(kind) {
  const skeleton = kind === 'skeleton';
  $('camera-view').hidden = skeleton;
  $('skeleton-view').hidden = !skeleton;
  $('view-camera').setAttribute('aria-selected', String(!skeleton));
  $('view-skeleton').setAttribute('aria-selected', String(skeleton));
  if (skeleton) {
    const frame = $('skeleton');
    const expectedPath = new URL(frame.dataset.src, window.location.href).pathname;
    let currentPath = '';
    try { currentPath = frame.contentWindow.location.pathname; } catch (_) {}
    // Recover an already-open page whose former nested link navigated this
    // iframe to /head.html while the outer PICO tab remained selected.
    if (currentPath !== expectedPath) frame.src = frame.dataset.src;
  }
}
$('view-camera').onclick = () => showVision('camera');
$('view-skeleton').onclick = () => showVision('skeleton');

function setStep(id, done) { $(id).classList.toggle('done', Boolean(done)); }

function sourceLabel(source) {
  const [side, kind] = source.split('_');
  return `${side === 'left' ? '左' : '右'}·${{arm:'肩肘腕', hand:'五指', wrist:'手腕姿态'}[kind] || kind}`;
}

function renderArm(side, value, fresh) {
  const card = element('div', undefined, 'arm');
  const heading = element('div', undefined, 'arm-title');
  heading.append(element('span', {left: '左臂', right: '右臂', arm: '机械臂'}[side] || side));
  heading.append(element('span', fresh ? `使能 ${value.enable_states?.filter(Boolean).length ?? '—'} / 7` : '状态过期'));
  card.append(heading);
  if (value.controller_fault) card.append(element('div', `故障：${value.controller_fault.status_name || value.controller_fault.category || JSON.stringify(value.controller_fault)}`, 'fault'));
  if (value.driver_fault) card.append(element('div', `驱动：${value.driver_fault}`, 'fault'));
  const joints = element('div', undefined, 'joint-grid');
  for (let i = 0; i < 7; i++) {
    const valueRad = value.arm_urdf_rad?.[i];
    const cell = element('div', undefined, 'joint');
    cell.append(element('span', `J${i + 1} · °`), document.createTextNode(fresh && Number.isFinite(valueRad) ? (valueRad * 180 / Math.PI).toFixed(1) : '—'));
    joints.append(cell);
  }
  card.append(joints);
  const handAge = value.hand?.age_s;
  card.append(element('p', fresh && value.hand?.position_unit
    ? `手部底层反馈 ${Number.isFinite(handAge) ? `${Math.round(handAge * 1000)} ms` : '年龄未知'} · ${value.hand.position_unit.join(' / ')}`
    : '手部反馈不可用 / 未选择手部', 'hint'));
  return card;
}

function render() {
  const liveGateway = Boolean(state.connected && state.remote?.sessions?.includes('esrobo_gateway')
    && state.remote?.gateway_args?.includes('--side'));
  if (liveGateway || (!modeInitialized && state.processes.teleop?.running)) {
    const restored = modeValue(state.mode);
    if (restored && [...$('mode').options].some(option => option.value === restored)) $('mode').value = restored;
    modeInitialized = true;
  }
  const robot = state.robot || {};
  const telemetry = state.telemetry || {};
  const fresh = robot.age_s < 1.5 && state.connected && state.remote_age_s < 2;
  const mode = state.processes.teleop?.running ? state.mode : options();
  // A dual-arm gateway reports feedback under `sides.left/right`, while a
  // single-arm gateway reports the selected side directly in `feedback`.
  // Preserve the real side name so hand geometry and arm status use the same
  // lookup path in both cases.
  const members = robot.feedback?.sides || (robot.feedback && mode.side !== 'both'
    ? {[mode.side]: robot.feedback} : {});
  const waistRemoteFresh = state.connected && state.remote_age_s < 2;
  const waistDriverKnown = state.remote?.sessions?.includes('esrobo_waist');
  const waistDriver = waistRemoteFresh && waistDriverKnown;
  // A waist status request shares the SSH lock with remote polling. While it
  // runs, remote_age_s may exceed 2 s even though the driver is still alive.
  // Keep the measured positions and in-flight confirmation until a fresh
  // status actually reports that the session ended (or SSH disconnects).
  if (!state.connected || (waistRemoteFresh && !waistDriverKnown)) {
    waistAxes = null; waistReference = null; waistLimit = null;
    waistSingleTurnMin = null; waistSingleTurnMax = null; waistReadAt = 0;
    waistAutoRetryAt = 0;
    waistServerPending = false;
    waistMotionToken++; waistMoving = false;
    for (const axis of [31, 32, 33]) $(`waist-value-${axis}`).textContent = '反馈未读取 / 已失效';
  }
  const waistCode = state.capabilities?.waist_control && state.remote?.waist_control_available;
  const armDisabled = fresh && liveGateway && ['IDLE', 'FAULT'].includes(robot.mode)
    && Object.values(members).length > 0
    && Object.values(members).every(value => value.enable_states?.length === 7
      && value.enable_states.every(enabled => enabled === false));
  const waistFeedbackRecent = Boolean(waistReadAt > 0 && waistAxes && waistReference && Number.isFinite(waistLimit)
    && Number.isFinite(waistSingleTurnMin) && Number.isFinite(waistSingleTurnMax)
    && performance.now() - waistReadAt < 15000);
  const waistFeedbackValid = waistFeedbackRecent && [31, 32, 33].every(axis =>
    Number.isFinite(waistAxes[axis]?.position_deg) && waistErrorAllowsMotion(waistAxes[axis]?.error)
    && Number.isFinite(waistReference[axis])
    && waistTargetRange(waistAxes[axis].position_deg));
  for (const form of document.querySelectorAll('[data-waist-target-form]')) {
    const axis = Number(form.dataset.waistTargetForm);
    const input = form.querySelector('input');
    const range = waistFeedbackRecent ? waistTargetRange(waistAxes?.[axis]?.position_deg) : null;
    if (range) {
      input.min = String(range.min);
      input.max = String(range.max);
    } else {
      input.removeAttribute('min');
      input.removeAttribute('max');
    }
    const invalidTarget = input.value !== '' && range && !input.validity.valid;
    const rangeLabel = form.querySelector('[data-waist-range]');
    rangeLabel.textContent = range
      ? `目标角度（${range.min.toFixed(2)}°～${range.max.toFixed(2)}°）${invalidTarget ? ' · 输入越界' : ''}`
      : '目标角度（读取反馈后显示范围）';
    rangeLabel.classList.toggle('invalid', Boolean(invalidTarget));
  }
  const waistCanEnable = waistCode && waistDriver && armDisabled && !waistMoving && !waistServerPending
    && !state.processes.teleop?.running && waistFeedbackValid;
  const waistCanMove = waistCanEnable && state.waist_enable_requested;
  $('waist-gate').textContent = !state.connected ? '未连接机器人'
    : !waistRemoteFresh ? waistMoving ? '正在读取腰部实测反馈' : '等待网关状态刷新'
      : !waistDriver ? '腰部驱动未运行'
        : !liveGateway ? '等待网关确认机械臂失能'
          : !armDisabled ? '等待机械臂停止并失能'
            : state.processes.teleop?.running ? '请先结束遥操作计算'
              : waistMoving ? '等待腰部实测到位'
                : waistServerPending ? '上一腰部目标尚未确认到位'
                : !waistFeedbackRecent ? '请重新读取腰部反馈'
                : !waistFeedbackValid ? '位置或错误码未通过 · 控制锁定'
                  : !state.waist_enable_requested ? '请先使能腰部三轴'
                  : state.waist_moved ? '可调整腰部 · 臂使能受限' : '反馈合格 · 可单独调整';
  $('waist-gate').className = 'tag ' + (state.waist_moved ? 'fault' : waistCanMove ? 'active' : '');
  $('waist-read').disabled = !waistCode || !waistDriver || waistMoving;
  $('waist-enable').disabled = !waistCanEnable;
  $('waist-stop').disabled = !waistCode || !waistDriver;
  for (const button of document.querySelectorAll('[data-waist-axis]')) {
    const position = waistAxes?.[Number(button.dataset.waistAxis)]?.position_deg;
    const next = position + Number(button.dataset.direction);
    button.disabled = !waistCanMove || !Number.isFinite(next)
      || next < waistSingleTurnMin || next > waistSingleTurnMax;
  }
  for (const form of document.querySelectorAll('[data-waist-target-form]'))
    form.querySelector('button').disabled = !waistCanMove;
  if (!state.connected) $('waist-detail').textContent = 'SSH 已断开，旧反馈已失效。请重新连接机器人，再读取腰部反馈。';
  else if (!waistRemoteFresh) $('waist-detail').textContent = waistMoving
    ? '正在读取腰部实测反馈；到位前暂时锁定调整按钮。'
    : '网关状态暂时未更新；已锁定调整，恢复后自动重读腰部反馈。';
  else if (!waistCode) $('waist-detail').textContent = '机器人尚未同步腰部控制代码；同步 robot_link 后刷新页面。';
  else if (!waistDriver) $('waist-detail').textContent = '先在左侧启动腰部三轴驱动，再读取三轴反馈。';
  else if (waistFeedbackValid && !liveGateway) $('waist-detail').textContent = '腰部反馈正常；机械臂网关未运行，页面无法确认机械臂七轴失能，因此尚未开放调整按钮。';
  const faults = Object.values(members).filter(value => value.controller_fault || value.driver_fault);
  const sides = mode.side === 'both' ? ['left', 'right'] : [mode.side];
  const required = telemetry.required_sources || sides.flatMap(side =>
    (mode.hand_only ? ['hand'] : mode.with_hand ? ['arm', 'hand', 'wrist'] : ['arm'])
      .map(kind => `${side}_${kind}`));
  // Source ages are captured inside the control process. The status file and
  // browser update at a deliberately lower rate and must not be added to the
  // 100 ms control-input limit.
  const staleInputs = required.filter(key =>
    !Number.isFinite(telemetry.sources?.[key]) || telemetry.sources[key] > .1);
  const inputFresh = telemetry.age_s < 1.5 && required.length > 0 && staleInputs.length === 0;
  const inputDescription = key => {
    const side = key.split('_')[0];
    const handState = state.pico_input?.packet?.pico_hand_status?.[side];
    if (key.endsWith('_hand') && state.pico_input?.connected) {
      const reason = {
        'PICO hand tracking inactive': '手指追踪未激活；请在头显开启 Hand Tracking 并让裸手进入视野',
        'PICO hand joints invalid/occluded': '手指关节无效或被遮挡',
        'PICO Hand stream stale': '手指数据已过期',
        'Hand timestamp/pose frozen': '手指时间戳或位姿未更新',
      }[handState];
      if (reason) return `${sourceLabel(key)}（${reason}）`;
    }
    return sourceLabel(key);
  };
  const geometryItems = mode.with_hand && !mode.hand_only
    ? sides.map(side => ({side, geometry: (members[side]?.hand || {}).geometry || {}}))
    : [];
  const geometryReady = geometryItems.length === 0 || geometryItems.every(item => item.geometry.ready === true);

  $('connection').textContent = state.connected ? 'SSH 已连接' : '机器人未连接';
  $('connection-dot').classList.toggle('online', state.connected);
  $('robot-mode').textContent = fresh ? robot.mode : '反馈离线';
  $('robot-mode').className = 'tag ' + (fresh && robot.mode === 'ACTIVE' ? 'active' : fresh && robot.mode === 'FAULT' ? 'fault' : '');
  const returnFailure = fresh && robot.mode === 'FAULT' && robot.last_return?.returned === false
    ? robot.last_return.reason : '';
  $('robot-reason').textContent = state.remote_error
    || (returnFailure ? `回零失败：${returnFailure}` : robot.reason)
    || '等待网关反馈';
  $('feedback-age').textContent = fresh ? `${Math.round((robot.age_s + (robot.feedback_cache_age_s || 0)) * 1000)} ms` : '—';
  $('compute-rate').textContent = Number.isFinite(telemetry.rates?.compute_hz) ? `${telemetry.rates.compute_hz.toFixed(1)} Hz` : '—';
  const loopHz = telemetry.rates?.loop_hz;
  const p95 = telemetry.rates?.compute_ms_p95;
  $('compute-detail').textContent = Number.isFinite(loopHz) && Number.isFinite(p95)
    ? `主循环 ${loopHz.toFixed(1)} Hz · P95 ${p95.toFixed(1)} ms`
    : '有效目标 / 秒';
  $('sent-count').textContent = `${telemetry.computed || 0} / ${telemetry.sent || 0}`;
  $('runtime-status').textContent = telemetry.age_s < 1.5
    ? `${telemetry.preview ? '预览 · 不发送目标' : '正式计算'} · ${telemetry.preparation_status || telemetry.status}`
    : state.processes.teleop?.running ? '正在加载模型 / 等待状态…' : '计算未运行';
  $('control-owner').textContent = state.owner === session ? '本页面控制中' : state.owner ? '其他页面控制中' : '待启动';
  const running = Boolean(state.processes.teleop?.running);
  $('mode').disabled = running || liveGateway;
  for (const id of ['preview', 'run']) $(id).disabled = running || !fresh || robot.mode !== 'IDLE';
  $('pause-preview').disabled = !(running && telemetry.preview);
  const canRecover = state.capabilities?.fault_recovery && robot.recovery_supported === true
    && fresh && liveGateway && robot.mode === 'FAULT' && !running && mode.side !== 'both';
  $('recover').disabled = !canRecover;
  $('recover').textContent = mode.hand_only ? '核验失能并恢复 IDLE' : '受检回零并恢复 IDLE';
  $('recovery-panel').classList.toggle('fault', fresh && robot.mode === 'FAULT');
  $('recovery-detail').textContent = !state.capabilities?.fault_recovery ? '本机控制台尚未加载新版本；重启本机控制台后启用。'
    : !fresh ? '等待新鲜的网关状态后才能恢复。'
    : robot.recovery_supported !== true ? '机器人网关尚未加载恢复功能；同步代码并重启一次网关后启用。'
    : robot.mode === 'FAULT' && mode.side === 'both' ? '双臂需要现场验证的恢复流程，网页不执行自动回零。'
      : robot.mode === 'FAULT' && running ? '请先结束本机计算，再执行恢复。'
        : robot.mode === 'FAULT' && mode.hand_only ? '停止后核验机械臂失能和手部反馈；恢复不发送位置目标。'
          : robot.mode === 'FAULT' ? '单臂将执行现有受检回零；确认现场安全后操作。'
            : robot.mode === 'RECOVERING' || robot.mode === 'RETURNING' ? '恢复正在执行，等待网关显示 IDLE。'
              : robot.mode === 'IDLE' ? '网关已就绪；重新开始计算后才能使能。'
                : '故障停止后可在这里恢复，无需重启网关。';
  $('can').disabled = canBusy || !state.connected || !state.capabilities?.automatic_can_teardown;
  const mayEnable = fresh && robot.mode === 'IDLE' && state.owner === session && telemetry.age_s < 1
    && inputFresh && geometryReady && telemetry.targets && telemetry.sent > 0 && !telemetry.preview && Object.values(members).length && !faults.length;
  $('enable').disabled = !mayEnable;
  setStep('step-link', state.connected);
  setStep('step-drivers', fresh);
  setStep('step-reference', telemetry.reference_ready && telemetry.age_s < 1.5);
  setStep('step-compute', telemetry.initialized && telemetry.age_s < 1.5);
  setStep('step-active', fresh && robot.mode === 'ACTIVE');

  $('gate-fault').textContent = !fresh ? '○ 机器人反馈待连接' : faults.length ? `× ${faults.length} 侧控制器/驱动故障` : '✓ 控制器无故障';
  $('gate-fault').className = faults.length ? 'bad' : fresh ? 'good' : '';
  $('gate-input').textContent = !running ? '○ 计算尚未开始'
    : inputFresh ? '✓ 计算时输入新鲜'
      : telemetry.age_s >= 1.5 ? '× 计算状态更新中断'
        : `× 缺失或过期：${staleInputs.map(inputDescription).join('、')}`;
  $('gate-input').className = running && !inputFresh ? 'bad' : inputFresh ? 'good' : '';
  $('gate-geometry').textContent = !mode.with_hand || mode.hand_only
    ? '✓ 当前模式无需手臂回零手部几何'
    : geometryReady ? `✓ 灵巧手碰撞几何标定完整：${geometryItems.map(item =>
        `${item.side === 'left' ? '左' : '右'}手 ${item.geometry.calibrated_joints}/${item.geometry.total_joints}`
      ).join('、')}`
      : `× 几何标定缺失：${geometryItems.map(item =>
          `${item.side === 'left' ? '左' : '右'}手 ${item.geometry.calibrated_joints || 0}/${item.geometry.total_joints || 10}`
        ).join('、')}`;
  $('gate-geometry').className = geometryReady ? 'good' : 'bad';
  $('gate-enable').textContent = fresh && robot.mode === 'ACTIVE' ? '● 机器人正在运动跟随'
    : fresh && robot.mode === 'RETURNING' ? '○ 机器人正在受检回零'
      : fresh && robot.mode === 'CALIBRATING' ? '○ 等待 PICO 准备姿态' : '✓ 机器人未处于跟随状态';
  $('gate-enable').className = fresh && robot.mode === 'ACTIVE' ? 'active-motion' : 'good';

  $('arm-state').replaceChildren();
  for (const [side, value] of Object.entries(members)) $('arm-state').append(renderArm(side, value, fresh));
  $('source-state').replaceChildren();
  for (const source of required) {
    const age = telemetry.sources?.[source];
    const sampleAge = Number.isFinite(age) ? age : Infinity;
    const row = element('div', undefined, 'source ' + (sampleAge > .1 ? 'stale' : ''));
    const [side, kind] = source.split('_');
    const label = `${side === 'left' ? '左' : '右'} · PICO ${{arm:'肩肘腕骨架', hand:'五指追踪', wrist:'手腕整体姿态'}[kind] || kind}`;
    const hz = telemetry.source_hz?.[source];
    row.append(element('span', label), element('span', Number.isFinite(sampleAge) ? `计算采样 ${Math.round(sampleAge * 1000)} ms · ${Number.isFinite(hz) ? hz.toFixed(1) : '—'} Hz` : '未收到'));
    $('source-state').append(row);
  }
  $('can-state').textContent = state.remote.can || '等待 SSH 连接';

  for (const [name, spec] of Object.entries(remoteServices)) {
    const output = $(`status-r-${name}`);
    if (spec.virtual) {
      const side = name === 'left_arm' ? 'left' : 'right';
      const arm = members[side];
      const enabled = arm?.enable_states?.filter(Boolean).length;
      const faultName = arm?.controller_fault?.status_name || arm?.controller_fault?.category || arm?.driver_fault;
      output.textContent = !fresh || !arm ? '等待网关反馈' : faultName ? `故障 · ${faultName}` : enabled ? `已使能 ${enabled}/7` : '已失能 0/7';
      output.classList.toggle('on', fresh && Boolean(arm));
      output.classList.toggle('fault', Boolean(arm?.controller_fault));
    } else {
      const on = state.connected && state.remote_age_s < 2 && state.remote.sessions?.includes(spec.session);
      output.textContent = on ? '会话运行中' : '未运行';
      output.classList.toggle('on', on);
    }
  }
  for (const name of Object.keys(localNames)) {
    const output = $(`status-l-${name}`);
    const process = state.processes[name];
    output.textContent = process?.running ? '运行中' : process ? `已退出 (${process.exit_code})` : '未启动';
    output.classList.toggle('on', Boolean(process?.running));
    if (name === 'pc_service') {
      output.textContent = state.pc_service ? '服务运行中' : '服务未运行';
      output.classList.toggle('on', Boolean(state.pc_service));
    }
  }
  $('calibration-prompt').textContent = telemetry.preparation_status || telemetry.input_rejection || (mode.hand_only
    ? '点击使能后自然张开所选侧人手并保持稳定。'
    : telemetry.reference_ready && telemetry.age_s < 1
      ? 'PICO 胸前抬臂参考已就绪；保持五指可见。'
      : '将前臂抬到胸前、肘部自然弯曲并保持双手在头显视野内。');
  const pico = state.pico_input;
  const handStatus = pico?.packet?.pico_hand_status || {};
  const statusLabel = value => ({tracking:'正在追踪', 'PICO hand tracking inactive':'追踪未激活',
    'PICO hand joints invalid/occluded':'关节无效 / 被遮挡', 'PICO Hand stream stale':'数据过期',
    'waiting for new Hand sample':'等待新手部帧', 'Hand timestamp/pose frozen':'时间戳 / 位姿未更新'}[value] || value || '未收到');
  picoLive.textContent = pico?.connected
    ? `Body ${pico.source_update_hz} Hz · 左手：${statusLabel(handStatus.left)} · 右手：${statusLabel(handStatus.right)}`
    : 'PICO Body 尚未收到或已过期；请查看 PICO 采集日志确认各路状态。';
  handPreview.replaceChildren();
  if (telemetry.age_s < 1 && telemetry.targets) {
    for (const [side, target] of Object.entries(telemetry.targets)) {
      if (target.hand_unit) handPreview.append(element('p', `${side === 'left' ? '左' : '右'}手${telemetry.preview ? '预览' : '计算'}目标（L10）：${target.hand_unit.join(' / ')}`));
    }
  }
  updateLog();
}

async function updateLog() {
  if (!state || logRequest) return;
  const [type, selected] = selectedLog.split(':');
  let output = '';
  if (type === 'events') output = state.events.join('\n');
  else if (type === 'l') output = state.processes[selected]?.log || '该程序尚未从页面启动';
  else {
    logRequest = true;
    const name = remoteServices[selected]?.virtual ? 'gateway' : selected;
    try { output = (await api('remote_log', {name})).log; }
    catch (error) { output = error.message; }
    finally { logRequest = false; }
  }
  const nearBottom = $('log').scrollTop + $('log').clientHeight >= $('log').scrollHeight - 40;
  $('log').textContent = output || '暂无日志';
  if (nearBottom) $('log').scrollTop = $('log').scrollHeight;
}

async function poll() {
  while (pollRunning) {
    try {
      const response = await fetch('/api/state', {cache: 'no-store', signal: AbortSignal.timeout(3000)});
      if (!response.ok) throw new Error('控制台服务不可用');
      state = await response.json();
      if (state.processes.pico?.running) {
        try {
          const picoResponse = await fetch('/api/pico/state', {cache:'no-store', signal:AbortSignal.timeout(900)});
          if (picoResponse.ok) state.pico_input = await picoResponse.json();
        } catch (_) { /* Other console controls remain available when the viewer is offline. */ }
      }
      render();
      const waistReadInterval = waistServerPending ? 1000 : 10000;
      if (waistAxes && !waistMoving && !waistAutoReading
          && (!waistReadAt || performance.now() - waistReadAt > waistReadInterval)
          && performance.now() >= waistAutoRetryAt
          && state.connected && state.remote_age_s < 2
          && state.remote?.sessions?.includes('esrobo_waist')) {
        waistAutoReading = true;
        const readToken = waistMotionToken;
        api('waist', {command: 'status'}).then(feedback => {
          if (readToken === waistMotionToken && !waistMoving && state.connected
              && state.remote?.sessions?.includes('esrobo_waist'))
            showWaistFeedback(feedback);
        }).catch(error => {
          if (readToken !== waistMotionToken || waistMoving) return;
          waistReadAt = 0;
          waistAutoRetryAt = performance.now() + 2000;
          waistServerPending = true;
          for (const axis of [31, 32, 33]) $(`waist-value-${axis}`).textContent = '反馈等待自动重读';
          $('waist-detail').textContent = `自动读取腰部反馈失败，正在重试：${error.message}`;
          notice(`自动读取腰部反馈失败，正在重试：${error.message}`, true);
          render();
        }).finally(() => { waistAutoReading = false; });
      }
    } catch (error) {
      $('connection').textContent = '控制台连接中断';
      $('connection-dot').classList.remove('online');
      $('enable').disabled = true;
      $('robot-mode').textContent = '状态过期';
      notice(error.message, true);
    }
    await new Promise(resolve => setTimeout(resolve, 700));
  }
}
setInterval(() => api('heartbeat').catch(() => {}), 700);
window.addEventListener('pagehide', () => {
  pollRunning = false;
  fetch('/api/action', {method: 'POST', headers: {'Content-Type': 'application/json', 'X-Head-Control': '1'},
    body: JSON.stringify({action: 'release', session}), keepalive: true}).catch(() => {});
});

function resizeCamera() {
  const frame = $('camera');
  const doc = frame.contentDocument;
  if (!doc?.querySelector('main') || doc.documentElement.dataset.embedded) return;
  doc.documentElement.dataset.embedded = 'true';
  const style = doc.createElement('link');
  style.rel = 'stylesheet';
  style.href = '/head-embed.css';
  doc.head.append(style);
  const resize = () => { frame.style.height = `${doc.querySelector('main').getBoundingClientRect().height + 8}px`; };
  new ResizeObserver(resize).observe(doc.querySelector('main'));
  resize();
}
$('camera').addEventListener('load', resizeCamera);
resizeCamera();
poll();
