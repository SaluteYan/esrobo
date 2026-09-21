const session = crypto.randomUUID();
const $ = id => document.getElementById(id);
let state = null, stream = 'color', running = true, imageURL = null;
let errorMessage = '', errorUntil = 0;
async function post(action, data = {}) {
  const response = await fetch(`/api/head/${action}`, {method: 'POST', headers: {'Content-Type': 'application/json', 'X-Head-Control': '1'}, body: JSON.stringify({session, ...data}), signal: AbortSignal.timeout(4000)});
  const result = await response.json();
  if (!response.ok) throw new Error(result.error || `HTTP ${response.status}`);
}
function report(error) {errorMessage = error.message; errorUntil = Date.now() + 6000; $('message').textContent = errorMessage;}
function render() {
  const fresh = state && state.age_s !== null && state.age_s < .4 && ['1','2'].every(id => state.axes[id]?.valid === 'true');
  $('connection').textContent = fresh ? `反馈 ${Math.round(state.age_s * 1000)} ms` : '电机反馈不可用';
  const own = state?.control.owner === session;
  const pending = state && Object.values(state.axes).some(a => a.pending === 'true');
  const ready = fresh && own && !state.control.busy && !pending && Object.values(state.axes).every(a => a.adjustment_enabled === 'true' && a.torque_enabled === '1');
  document.querySelectorAll('.move,.jog').forEach(el => el.disabled = !ready);
  $('enable').disabled = !fresh || state.control.busy || pending || Boolean(state.control.owner) || !Object.values(state.axes).every(a => a.allow_motion === 'true');
  for (const id of ['1','2']) {
    const axis = state?.axes[id];
    $(`position-${id}`).textContent = fresh ? axis.position_ticks : '--';
    $(`torque-${id}`).textContent = fresh ? `力矩${axis.torque_enabled === '1' ? '开启' : '关闭'}` : '力矩 --';
    $(`pending-${id}`).textContent = fresh ? (axis.pending === 'true' ? '未到位' : '静候') : '--';
    if (fresh && !$(`target-${id}`).value) $(`target-${id}`).value = axis.position_ticks;
    $(`detail-${id}`).textContent = !fresh ? '等待新鲜反馈' : axis.pending === 'true' ? `目标 ${axis.pending_target_ticks ?? '--'} · 误差 ${axis.pending_error_ticks ?? '--'}` : axis.message;
  }
  $('message').textContent = Date.now() < errorUntil ? errorMessage : state?.control.message || '服务不可用';
}
async function poll() {
  while (running) {
    try {
      const response = await fetch('/api/head/state', {cache: 'no-store', signal: AbortSignal.timeout(2000)});
      if (!response.ok) throw new Error('头部相机服务不可用');
      state = await response.json(); render();
      if (state.control.owner === session) await post('heartbeat');
    } catch (error) { state = null; render(); report(error); }
    await new Promise(resolve => setTimeout(resolve, 200));
  }
}
async function images() {
  while (running) {
    const requested = stream;
    try {
      const response = await fetch(`/api/head/${requested}.jpg`, {cache:'no-store', signal:AbortSignal.timeout(2000)});
      if (!response.ok) throw new Error('等待图像或图像已过期');
      const blob = await response.blob();
      if (requested === stream) {
        const next = URL.createObjectURL(blob); $('preview').src = next;
        if (imageURL) URL.revokeObjectURL(imageURL); imageURL = next;
        $('preview').hidden = false; $('image-empty').hidden = true;
        const age = state?.images[stream]?.age_s;
        $('image-age').textContent = age === undefined ? '' : `${Math.round(age * 1000)} ms`;
      }
    } catch (_) { $('preview').hidden = true; $('image-empty').hidden = false; $('image-age').textContent = '图像不可用'; }
    await new Promise(resolve => setTimeout(resolve, 100));
  }
}
$('enable').onclick = async () => {
  if (!confirm('请托稳头部，确认相机、线缆及周围无夹点。开启调整可能接合电机力矩，是否继续？')) return;
  try {await post('enable');} catch (error) {report(error);}
};
$('lock').onclick = async () => {try {await post('lock');} catch (error) {report(error);}};
document.querySelectorAll('.move,.jog').forEach(button => button.onclick = async () => {
  const id = Number(button.dataset.id);
  const target = button.classList.contains('jog') ? Number(state.axes[id].position_ticks) + Number(button.dataset.delta) : Number($(`target-${id}`).value);
  if (!Number.isInteger(target)) return report(new Error('目标必须为整数计数'));
  try {await post('move', {id, target});} catch (error) {report(error);}
});
document.querySelectorAll('.current').forEach(button => button.onclick = () => {
  const id = button.dataset.id; if (state?.axes[id]) $(`target-${id}`).value = state.axes[id].position_ticks;
});
document.querySelectorAll('[data-stream]').forEach(button => button.onclick = () => {
  stream = button.dataset.stream;
  document.querySelectorAll('[data-stream]').forEach(el => el.setAttribute('aria-selected', String(el === button)));
  $('depth-scale').hidden = stream !== 'depth';
  $('preview').hidden = true;
});
window.addEventListener('pagehide', () => {
  running = false;
  if (state?.control.owner === session) fetch('/api/head/lock', {method:'POST', headers:{'Content-Type':'application/json','X-Head-Control':'1'}, body:JSON.stringify({session}), keepalive:true}).catch(() => {});
});
poll(); images();
