import * as THREE from 'three';
import { OrbitControls } from './vendor/OrbitControls.js';
import { TransformControls } from './vendor/TransformControls.js';

const $ = (id) => document.getElementById(id);
const viewport = $('viewport');
const fields = ['px', 'py', 'pz', 'rx', 'ry', 'rz'].map($);
const deg = THREE.MathUtils.radToDeg;
const rad = THREE.MathUtils.degToRad;
const scene = new THREE.Scene();
scene.background = new THREE.Color('#f4f6f7');
const camera = new THREE.PerspectiveCamera(40, 1, 0.005, 30);
camera.up.set(0, 0, 1);
const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.outputColorSpace = THREE.SRGBColorSpace;
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
viewport.prepend(renderer.domElement);
renderer.domElement.tabIndex = 0;
const orbit = new OrbitControls(camera, renderer.domElement);
orbit.enableDamping = true;
orbit.dampingFactor = 0.12;
orbit.minDistance = 0.12;
orbit.maxDistance = 5;
scene.add(new THREE.HemisphereLight(0xffffff, 0x79848c, 2.7));
const light = new THREE.DirectionalLight(0xffffff, 3.2);
light.position.set(1, -1, 2);
light.castShadow = true;
light.shadow.mapSize.set(2048, 2048);
light.shadow.camera.left = light.shadow.camera.bottom = -1;
light.shadow.camera.right = light.shadow.camera.top = 1;
light.shadow.normalBias = 0.001;
scene.add(light);
const fill = new THREE.DirectionalLight(0xffffff, 1.2);
fill.position.set(-1, 1, 0.7);
scene.add(fill);
const grid = new THREE.GridHelper(3, 60, 0xc5cfd4, 0xdfe5e8);
grid.rotation.x = Math.PI / 2;
grid.position.z = -0.004;
scene.add(grid);
const ground = new THREE.Mesh(new THREE.PlaneGeometry(10, 10), new THREE.ShadowMaterial({ opacity: 0.12 }));
ground.position.z = -0.005;
ground.receiveShadow = true;
scene.add(ground);
scene.add(new THREE.AxesHelper(0.09));
const target = new THREE.Object3D();
target.add(new THREE.AxesHelper(0.055));
const targetDot = new THREE.Mesh(new THREE.SphereGeometry(0.006, 16, 12), new THREE.MeshBasicMaterial({ color: 0x167b6b, depthTest: false }));
targetDot.renderOrder = 20;
target.add(targetDot);
scene.add(target);
const actual = new THREE.AxesHelper(0.045);
scene.add(actual);
const controls = new TransformControls(camera, renderer.domElement);
controls.setSize(0.85);
controls.setSpace('world');
controls.attach(target);
const helper = controls.getHelper();
scene.add(helper);
helper.visible = controls.enabled = false;
const meshes = new Map();
const jointInputs = [];
const gainInputs = [];
let socket, clientId, state, cameraFrame;
let connected = false, dragging = false, suppress = false;
let pendingPose = false, sequence = 0, sentSequence = 0, acknowledgedSequence = 0, targetVersion = -1, cameraVersion = -1;
let poseDirty = false, gainsDirty = false, jointEditing = false;
let latestTargetState = null;
let readOnly = false;

function showError(message) { $('error').textContent = message; $('error').hidden = !message; }
function send(message) {
  if (socket?.readyState !== WebSocket.OPEN) return false;
  socket.send(JSON.stringify(message));
  return true;
}
function isIK() { return connected && state?.enabled && state.mode === 'IK control' && !state.stopped; }
function updateFields() {
  const euler = new THREE.Euler().setFromQuaternion(target.quaternion, 'ZYX');
  const values = [...target.position.toArray().map((x) => x * 1000), ...[euler.x, euler.y, euler.z].map(deg)];
  fields.forEach((input, index) => { input.value = values[index].toFixed(2); });
  poseDirty = false;
}
function setPose(pose) {
  suppress = true;
  target.position.fromArray(pose.position);
  target.quaternion.fromArray(pose.quaternion).normalize();
  suppress = false;
  if (!poseDirty) updateFields();
}
function queuePose() {
  if (!isIK()) return;
  pendingPose = true;
  showError('');
  updateFields();
}
function flushPose() {
  if (!pendingPose || !isIK()) return;
  const seq = ++sequence;
  if (send({ type: 'pose', position: target.position.toArray(), quaternion: target.quaternion.toArray(), seq })) {
    sentSequence = seq;
    pendingPose = false;
  }
}
controls.addEventListener('dragging-changed', (event) => {
  dragging = event.value;
  orbit.enabled = !dragging;
  if (dragging) {
    renderer.domElement.focus({ preventScroll: true });
    poseDirty = false;
  } else {
    flushPose();
    if (latestTargetState) applyRemoteTarget(latestTargetState);
  }
});
controls.addEventListener('objectChange', () => { if (!suppress) queuePose(); });

function setMode(mode) {
  if (dragging) return;
  controls.setMode(mode);
  $('translate').classList.toggle('selected', mode === 'translate');
  $('rotate').classList.toggle('selected', mode === 'rotate');
}
function setSpace(space) {
  if (dragging) return;
  controls.setSpace(space);
  $('world').classList.toggle('selected', space === 'world');
  $('local').classList.toggle('selected', space === 'local');
}
$('translate').onclick = () => setMode('translate');
$('rotate').onclick = () => setMode('rotate');
$('world').onclick = () => setSpace('world');
$('local').onclick = () => setSpace('local');
function fit() {
  if (!cameraFrame) return;
  camera.position.fromArray(cameraFrame.position);
  orbit.target.fromArray(cameraFrame.target);
  orbit.update();
}
$('fit').onclick = fit;
window.addEventListener('keydown', (event) => {
  if (event.target.closest('input, select, textarea, button') || event.ctrlKey || event.metaKey || event.altKey) return;
  const key = event.key.toLowerCase();
  if (key === 't') setMode('translate');
  if (key === 'r') setMode('rotate');
  if (key === 'q') setSpace(controls.space === 'world' ? 'local' : 'world');
  if (key === 'f') fit();
  if (key === 'escape' && dragging) { controls.reset(); queuePose(); flushPose(); }
  if (['t', 'r', 'q', 'f', 'escape'].includes(key)) event.preventDefault();
});
window.addEventListener('keydown', (event) => {
  if (event.key === 'Shift') { controls.setTranslationSnap(0.001); controls.setRotationSnap(rad(1)); }
});
window.addEventListener('keyup', (event) => {
  if (event.key === 'Shift') { controls.setTranslationSnap(null); controls.setRotationSnap(null); }
});
window.addEventListener('blur', () => { controls.setTranslationSnap(null); controls.setRotationSnap(null); });
fields.forEach((input) => { input.oninput = () => { poseDirty = true; }; });
$('apply-pose').onclick = () => {
  if (!isIK()) return;
  const values = fields.map((input) => input.value.trim() === '' ? NaN : Number(input.value));
  if (!values.every(Number.isFinite)) return showError('Enter six finite pose values.');
  target.position.fromArray(values.slice(0, 3).map((x) => x / 1000));
  target.quaternion.setFromEuler(new THREE.Euler(...values.slice(3).map(rad), 'ZYX'));
  queuePose();
  flushPose();
};
$('jog').addEventListener('click', (event) => {
  const button = event.target.closest('button[data-axis]');
  if (!button || !isIK()) return;
  const step = Number($('jog-step').value) / 1000;
  if (!Number.isFinite(step) || step <= 0) return showError('Enter a positive jog step.');
  const delta = new THREE.Vector3();
  delta.setComponent(Number(button.dataset.axis), step * Number(button.dataset.sign));
  if (controls.space === 'local') delta.applyQuaternion(target.quaternion);
  target.position.add(delta);
  queuePose();
  flushPose();
});
$('alignment').onchange = () => send({ type: 'align', value: $('alignment').checked });
$('enable').onclick = () => { send({ type: 'enable' }); send({ type: 'mode', mode: 'IK control' }); };
$('control-mode').onchange = () => {
  pendingPose = false;
  poseDirty = false;
  send({ type: 'mode', mode: $('control-mode').value });
};
$('hold').onclick = () => { pendingPose = false; poseDirty = false; send({ type: 'hold' }); };
$('stop').onclick = () => { pendingPose = false; send({ type: 'stop' }); };
$('apply-gains').onclick = () => {
  const kp = gainInputs.map((row) => Number(row.kp.value));
  const kd = gainInputs.map((row) => Number(row.kd.value));
  if (![...kp, ...kd].every(Number.isFinite)) return showError('Enter finite gain values.');
  send({ type: 'gains', kp, kd });
  gainsDirty = false;
};

function buildPanels(model) {
  model.limits.forEach(([lower, upper], index) => {
    const row = document.createElement('div'); row.className = 'joint-row';
    const label = document.createElement('span'); label.textContent = `J${index + 1}`;
    const slider = document.createElement('input'); slider.type = 'range';
    const value = document.createElement('input'); value.type = 'number';
    for (const input of [slider, value]) { input.min = lower; input.max = upper; input.step = 0.1; input.value = 0; input.disabled = true; input.setAttribute('aria-label', `Joint ${index + 1} ${input.type === 'range' ? 'slider' : 'angle (deg)'}`); }
    function changed(input) {
      const number = input.valueAsNumber;
      if (!Number.isFinite(number)) return;
      slider.value = value.value = Math.max(lower, Math.min(upper, number));
      send({ type: 'joints', values: jointInputs.map((item) => Number(item.value.value)) });
    }
    slider.oninput = () => changed(slider); value.onchange = () => changed(value);
    for (const input of [slider, value]) { input.onfocus = () => { jointEditing = true; }; input.onblur = () => { jointEditing = false; }; }
    row.append(label, slider, value); $('joints').append(row); jointInputs.push({ slider, value });
  });
  const labels = document.createElement('div'); labels.className = 'gain-labels'; labels.textContent = 'KP                                         KD'; $('gains').append(labels);
  model.kp.forEach((kp, index) => {
    const row = document.createElement('div'); row.className = 'gain-row';
    const label = document.createElement('span'); label.textContent = `J${index + 1}`;
    const p = document.createElement('input'), d = document.createElement('input');
    for (const input of [p, d]) { input.type = 'number'; input.min = 0; input.oninput = () => { gainsDirty = true; }; }
    p.max = 300; p.step = 0.5; p.value = kp; p.setAttribute('aria-label', `Joint ${index + 1} KP`);
    d.max = model.max_kd[index]; d.step = 0.05; d.value = model.kd[index]; d.setAttribute('aria-label', `Joint ${index + 1} KD`);
    row.append(label, p, d); $('gains').append(row); gainInputs.push({ kp: p, kd: d });
  });
}
function applyRemoteTarget(message) {
  acknowledgedSequence = Math.max(acknowledgedSequence, message.ack_seq);
  if (message.target_version <= targetVersion || dragging || pendingPose) return;
  if (acknowledgedSequence < sentSequence) return;
  if (message.target_source === clientId && message.target_seq < sentSequence) return;
  targetVersion = message.target_version;
  setPose(message.target);
}
function updateState(message) {
  state = message;
  for (const transform of message.transforms) {
    const mesh = meshes.get(transform.id);
    if (mesh) { mesh.position.fromArray(transform.position); mesh.quaternion.fromArray(transform.quaternion); }
  }
  if (!readOnly) {
    actual.position.fromArray(message.actual.position); actual.quaternion.fromArray(message.actual.quaternion);
    latestTargetState = message;
    applyRemoteTarget(message);
  }
  cameraFrame = message.camera;
  if (message.camera_version !== cameraVersion) { cameraVersion = message.camera_version; fit(); }
  if (readOnly) {
    const alive = connected && !message.stopped;
    $('stop').disabled = !alive;
    $('control-status').textContent = message.stopped ? 'Stopped' : 'Live pose display';
    $('scene-message').textContent = message.stopped ? 'Viewer stopped.' : '';
    return;
  }
  const ik = isIK();
  const alive = connected && !message.stopped;
  controls.enabled = helper.visible = ik;
  $('alignment').checked = message.aligned; $('alignment').disabled = !alive || message.enabled;
  $('enable').disabled = !alive || !message.aligned || message.enabled;
  $('enable').textContent = message.enabled ? 'Control enabled' : 'Enable control';
  $('control-mode').disabled = !alive || !message.enabled;
  ['hold', 'stop', 'apply-gains'].forEach((id) => { $(id).disabled = !alive; });
  gainInputs.forEach((row) => { row.kp.disabled = row.kd.disabled = !alive; });
  if (document.activeElement !== $('control-mode')) $('control-mode').value = message.mode;
  $('apply-pose').disabled = !ik;
  fields.forEach((input) => { input.disabled = !ik; });
  $('jog').querySelectorAll('button').forEach((button) => { button.disabled = !ik; });
  jointInputs.forEach(({ slider, value }, index) => {
    slider.disabled = value.disabled = !alive || !message.enabled || message.mode !== 'Joint sliders';
    if (!jointEditing) slider.value = value.value = (message.mode === 'Joint sliders' ? message.joints[index] : message.actual_joints[index]).toFixed(2);
  });
  if (!gainsDirty && !gainInputs.some((row) => [row.kp, row.kd].includes(document.activeElement))) gainInputs.forEach((row, index) => { row.kp.value = message.kp[index]; row.kd.value = message.kd[index]; });
  $('position-error').innerHTML = `${message.position_error_mm.toFixed(2)}<small> mm</small>`;
  $('rotation-error').innerHTML = `${message.orientation_error_deg.toFixed(2)}<small> °</small>`;
  const status = message.stopped ? 'Stopped' : !message.enabled ? 'Read only · holding' : message.blocked ? 'Target blocked' : message.mode === 'IK control' ? (message.position_error_mm <= 2 && message.orientation_error_deg <= 2 ? 'At target' : 'Tracking target') : message.mode;
  $('control-status').textContent = status;
  document.querySelector('.status-panel').classList.toggle('blocked', message.blocked);
  $('mode-note').textContent = message.mode === 'VIZ (no torque)' ? 'Zero torque: KP, KD and feedforward are zero. Nothing supports the arm.' : !message.enabled ? 'Confirm the model, then enable control to drag the TCP.' : 'Target positions go directly to IK. Gravity feedforward is zero.';
  $('scene-message').textContent = message.blocked ? 'Target blocked by self-collision. Move the target away.' : !message.enabled ? 'Confirm model alignment in the panel to enable IK control.' : message.mode !== 'IK control' ? 'Select IK control to drag the end-effector target.' : '';
}
function connect() {
  socket = new WebSocket(`${location.protocol === 'https:' ? 'wss:' : 'ws:'}//${location.host}/ws`);
  socket.onmessage = (event) => {
    const message = JSON.parse(event.data);
    if (message.type === 'hello') {
      clientId = message.client;
      connected = true;
      targetVersion = -1; pendingPose = false; acknowledgedSequence = sentSequence = sequence = 0;
      $('connection').textContent = 'Connected'; $('connection').className = 'live';
      send({ type: 'ready' });
    } else if (message.type === 'state') updateState(message);
    else if (message.type === 'error') showError(message.message);
  };
  socket.onclose = () => {
    connected = false; pendingPose = false; controls.enabled = helper.visible = false;
    orbit.enabled = true;
    $('connection').textContent = state?.stopped ? 'Stopped' : 'Disconnected · retrying'; $('connection').className = '';
    $('scene-message').textContent = state?.stopped ? 'Viewer stopped. Restart the Python command to reconnect.' : readOnly ? 'Connection lost. Reconnecting to the pose display.' : 'Connection lost. The robot keeps the last accepted command.';
    document.querySelectorAll('aside button, aside select, aside input').forEach((element) => { element.disabled = true; });
    if (!state?.stopped) setTimeout(connect, 1000);
  };
  socket.onerror = () => { $('connection').textContent = 'Connection unavailable'; };
}
new ResizeObserver(() => {
  const width = viewport.clientWidth, height = viewport.clientHeight;
  renderer.setSize(width, height); camera.aspect = width / height; camera.updateProjectionMatrix();
}).observe(viewport);
let lastSend = 0;
renderer.setAnimationLoop((now) => {
  if (now - lastSend >= 20) { flushPose(); lastSend = now; }
  orbit.update(); renderer.render(scene, camera);
});
try {
  const response = await fetch('/model.json');
  if (!response.ok) throw new Error('Could not load robot model.');
  const model = await response.json();
  readOnly = model.read_only === true;
  $('environment').textContent = readOnly ? 'READ ONLY' : model.hardware ? 'HARDWARE' : 'SIMULATION';
  model.meshes.forEach((data) => {
    const geometry = data.size ? new THREE.BoxGeometry(...data.size) : new THREE.BufferGeometry();
    if (!data.size) {
      geometry.setAttribute('position', new THREE.Float32BufferAttribute(data.vertices, 3));
      geometry.setIndex(data.faces); geometry.computeVertexNormals();
    }
    const material = new THREE.MeshStandardMaterial({ color: new THREE.Color().fromArray(data.color), metalness: 0.18, roughness: 0.58 });
    const mesh = new THREE.Mesh(geometry, material); mesh.castShadow = mesh.receiveShadow = true;
    meshes.set(data.id, mesh); scene.add(mesh);
  });
  if (readOnly) {
    document.body.classList.add('read-only');
    document.title = model.title;
    document.querySelector('.brand > span:last-child').textContent = model.title;
    $('stop').textContent = 'Stop viewer';
    document.querySelector('.actions .hint').textContent = 'Stop viewer or press terminal Ctrl-C to exit. Closing the page allows reconnection.';
    document.querySelector('.shortcuts').textContent = 'Drag: orbit · Right drag: pan · Scroll: zoom · F: fit view';
    actual.visible = target.visible = false;
  } else buildPanels(model);
  connect();
} catch (error) {
  showError(error.message); $('scene-message').textContent = 'Viewer could not start. See the error in the panel.';
}
