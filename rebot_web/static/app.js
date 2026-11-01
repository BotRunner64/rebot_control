'use strict';
const $ = id => document.getElementById(id);
let client = sessionStorage.getItem('rebot-client');
if (!client) { client = crypto.randomUUID(); sessionStorage.setItem('rebot-client', client); }
let state, token = '', active = false, busy = false, pollBusy = false, pulseBusy = false;
let pending = null, sending = false, sendingJoint = null, timer = null, epoch = 0, lastSend = 0, dragging = null;
const rows = new Map();
const fmt = value => value == null ? '—' : value.toFixed(1);
function showError(message) { $('error').querySelector('span').textContent = message; $('error').hidden = false; }
function cancelTargets() { epoch++; pending = null; clearTimeout(timer); timer = null; dragging = null; }
async function api(name, data = {}) {
  const response = await fetch(`/api/${name}`, {method:'POST', headers:{'Content-Type':'application/json','X-Control-Token':token}, body:JSON.stringify({client,...data}), signal:AbortSignal.timeout(10000)});
  const result = await response.json();
  if (!response.ok) throw new Error(result.error || '操作失败');
  return result.state;
}
async function manual(name, data = {}) {
  if (busy || !token) return;
  cancelTargets(); busy = true; render();
  try {
    state = await api(name, data);
    if (name === 'connect' || name === 'takeover') active = true;
    if (name === 'disconnect') active = false;
    $('error').hidden = true;
  } catch(error) { showError(error.message); }
  finally { busy = false; render(); }
}
function canDrag(joint) {
  return active && state?.connected && state.owner && !state.resetting && joint.enabled && !joint.arming && joint.online && !joint.fault && !busy;
}
function queueTarget(joint, value) {
  if (!canDrag(joint) || !Number.isFinite(value)) return;
  pending = {joint:joint.id,angle:value,speed:Number($('speed').value),revision:state.control_revision};
  rows.get(joint.id).querySelector('output').textContent = fmt(value) + '°';
  flushTarget();
}
function flushTarget() {
  if (sending || !pending || timer) return;
  const delay = 33 - (performance.now() - lastSend);
  if (delay > 0) { timer = setTimeout(() => { timer = null; flushTarget(); }, delay); return; }
  const command = pending, version = epoch;
  pending = null; sending = true; sendingJoint = command.joint; lastSend = performance.now();
  api('target', command).then(next => {
    if (version === epoch) state = next;
  }).catch(error => {
    if (version === epoch) { cancelTargets(); showError(error.message); }
  }).finally(() => { sending = false; sendingJoint = null; render(); if (version === epoch) flushTarget(); });
}
async function stop(name = 'stop') {
  cancelTargets();
  if (!active || !state?.owner) return;
  try { state = await api(name); $('error').hidden = true; }
  catch(error) { showError(error.message); }
  render();
}
async function poll() {
  if (pollBusy) return;
  pollBusy = true;
  try {
    const response = await fetch(`/api/state?client=${encodeURIComponent(client)}`, {signal:AbortSignal.timeout(2500)});
    if (!response.ok) throw new Error('无法读取本地服务');
    const next = await response.json();
    if (token && token !== next.csrf) { cancelTargets(); active = false; }
    token = next.csrf; state = next;
    if (!state.owner || state.lease_expired) { active = false; cancelTargets(); }
    render();
  } catch(error) {
    active = false; cancelTargets(); render();
    $('status').textContent = '服务离线'; showError('服务无回应，最后保持指令可能仍有效。');
  } finally { pollBusy = false; }
}
async function pulse() {
  if (!active || !state?.owner || !state.connected || pulseBusy) return;
  pulseBusy = true;
  try { await api('heartbeat'); }
  catch(error) { active = false; cancelTargets(); render(); showError(error.message); }
  finally { pulseBusy = false; }
}
function createRows() {
  for (const joint of state.joints) {
    const row = document.createElement('div'); row.className = 'joint'; row.dataset.joint = joint.id;
    row.innerHTML = '<div class="joint-name"><strong></strong><span></span></div><div class="actual"><span>—</span><small>°</small></div><div class="slider-group"><div class="slider-wrap"><input type="range" step="0.1" disabled></div><output>—</output></div><p class="row-error" hidden></p>';
    row.querySelector('strong').textContent = 'J' + joint.id;
    row.querySelector('.joint-name span').textContent = joint.name;
    const slider = row.querySelector('input'); slider.min = joint.min_deg; slider.max = joint.max_deg;
    slider.setAttribute('aria-label', `J${joint.id} ${joint.name}目标角度`);
    slider.addEventListener('pointerdown', event => { if (event.isTrusted && !slider.disabled) dragging = joint.id; });
    slider.addEventListener('input', event => {
      if (!event.isTrusted) return;
      const fresh = state.joints.find(j => j.id === joint.id);
      queueTarget(fresh, Number(slider.value));
    });
    const release = event => { if (event.isTrusted) { dragging = null; render(); } };
    slider.addEventListener('pointerup', release); slider.addEventListener('pointercancel', release);
    slider.addEventListener('change', release);
    rows.set(joint.id, row); $('joints').append(row);
  }
}
function render() {
  if (!state) return;
  if (!rows.size) createRows();
  const owned = active && state.connected && state.owner;
  const enabled = state.joints.some(j => j.enabled);
  const enabledCount = state.joints.filter(j => j.enabled).length;
  $('status').textContent = state.connected ? owned ? '已连接' : '等待接管控制' : '未连接';
  $('status').classList.toggle('connected', owned);
  $('mode').textContent = state.simulate ? 'MIT · 模拟模式' : 'MIT · 本地真机';
  $('connect').textContent = owned ? '断开' : state.connected ? '恢复控制' : '连接';
  $('connect').disabled = busy || (owned && enabled);
  $('takeover').hidden = !state.connected || state.owner || !state.can_takeover;
  $('takeover').disabled = busy;
  $('stop').disabled = !owned || !enabled;
  $('reset').disabled = !owned || busy || enabledCount !== 7 || state.resetting || state.joints.some(j => j.fault);
  $('reset').textContent = state.resetting ? '复位中…' : '复位到零位';
  if (state.reset_error) showError(state.reset_error);
  $('enable-all').disabled = !owned || busy || enabledCount === 7;
  $('enable-all').textContent = busy ? '请稍候' : enabledCount === 7 ? '已使能' : enabledCount ? `继续使能 (${enabledCount}/7)` : '全部使能';
  $('disable-all').disabled = !owned;
  $('speed').disabled = busy;
  const moving = state.joints.find(j => j.moving);
  for (const joint of state.joints) {
    const row = rows.get(joint.id), slider = row.querySelector('input');
    row.classList.toggle('enabled', joint.enabled); row.classList.toggle('fault', !!joint.fault);
    row.querySelector('.actual span').textContent = fmt(joint.actual_deg);
    slider.disabled = !canDrag(joint) || (moving && moving.id !== joint.id);
    const localPending = pending?.joint === joint.id || sendingJoint === joint.id;
    if (dragging !== joint.id && !localPending) {
      const value = joint.enabled ? joint.target_deg : joint.actual_deg;
      slider.value = Math.max(joint.min_deg, Math.min(joint.max_deg, value ?? 0));
      row.querySelector('output').textContent = value == null ? '—' : fmt(value) + '°';
    }
    const error = row.querySelector('.row-error'); error.hidden = !joint.fault; error.textContent = joint.fault;
  }
}
function clicked(id, callback) { $(id).addEventListener('click', event => { if (event.isTrusted) callback(); }); }
clicked('connect', () => manual(active && state?.owner && state.connected ? 'disconnect' : 'connect'));
clicked('takeover', () => manual('takeover'));
clicked('enable-all', () => manual('enable-all'));
clicked('reset', () => manual('reset', {speed:Number($('speed').value),revision:state.control_revision}));
clicked('stop', () => stop()); clicked('disable-all', () => stop('disable'));
clicked('dismiss', () => { $('error').hidden = true; });
document.addEventListener('keydown', event => { if (event.isTrusted && event.key === 'Escape') stop(); });
window.addEventListener('pagehide', () => {
  cancelTargets();
  if (active && state?.owner && token) fetch('/api/stop', {method:'POST', keepalive:true, headers:{'Content-Type':'application/json','X-Control-Token':token}, body:JSON.stringify({client})}).catch(()=>{});
});
poll(); setInterval(poll, 100); setInterval(pulse, 350);
