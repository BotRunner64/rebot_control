import * as THREE from 'three';
import { OrbitControls } from './vendor/OrbitControls.js';
import { TransformControls } from './vendor/TransformControls.js';

const $ = (id) => document.getElementById(id);
const viewport = $('viewport');
const fields = ['px', 'py', 'pz', 'rx', 'ry', 'rz'].map($);
const deg = THREE.MathUtils.radToDeg;
const rad = THREE.MathUtils.degToRad;
const scene = new THREE.Scene();
scene.background = new THREE.Color('#f5f5f7');
const camera = new THREE.PerspectiveCamera(40, 1, 0.005, 30);
camera.up.set(0, 0, 1);
const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.outputColorSpace = THREE.SRGBColorSpace;
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
viewport.prepend(renderer.domElement);
renderer.domElement.hidden = true;
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
const grid = new THREE.GridHelper(3, 30, 0xd3d6dc, 0xe1e3e8);
grid.rotation.x = Math.PI / 2;
grid.position.z = -0.004;
scene.add(grid);
const ground = new THREE.Mesh(new THREE.PlaneGeometry(10, 10), new THREE.ShadowMaterial({ opacity: 0.12 }));
ground.position.z = -0.005;
ground.receiveShadow = true;
scene.add(ground);
const target = new THREE.Object3D();
const targetDot = new THREE.Mesh(new THREE.SphereGeometry(0.006, 16, 12), new THREE.MeshBasicMaterial({ color: 0x167b6b, depthTest: false }));
targetDot.renderOrder = 20;
target.add(targetDot);
scene.add(target);
const actual = new THREE.Mesh(new THREE.SphereGeometry(0.004, 12, 8), new THREE.MeshBasicMaterial({ color: 0x65686e }));
scene.add(actual);
const controls = new TransformControls(camera, renderer.domElement);
controls.setSize(0.85);
controls.setSpace('world');
controls.attach(target);
const helper = controls.getHelper();
scene.add(helper);
helper.visible = controls.enabled = false;
const meshes = new Map();
let socket, clientId, state, cameraFrame;
let connected = false, dragging = false, suppress = false;
let pendingPose = false, sequence = 0, sentSequence = 0, acknowledgedSequence = 0, targetVersion = -1, cameraVersion = -1;
let poseDirty = false;
let hardware = false, commandError = '';
let latestTargetState = null;
let readOnly = false;

function showNotice(message, error = false) {
  $('notice').textContent = message;
  $('notice').hidden = !message;
  $('notice').dataset.error = String(error);
}
function showError(message) {
  commandError = message;
  showNotice(message, Boolean(message));
}
function send(message) {
  if (socket?.readyState !== WebSocket.OPEN) return false;
  socket.send(JSON.stringify(message));
  return true;
}
function isIK() { return connected && state?.enabled && !state.stopped; }
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
  $('translate').setAttribute('aria-pressed', String(mode === 'translate'));
  $('rotate').setAttribute('aria-pressed', String(mode === 'rotate'));
}
$('translate').onclick = () => setMode('translate');
$('rotate').onclick = () => setMode('rotate');
function fit() {
  if (!cameraFrame) return;
  orbit.target.fromArray(cameraFrame.target);
  const offset = new THREE.Vector3().fromArray(cameraFrame.position).sub(orbit.target);
  camera.position.copy(orbit.target).add(offset.multiplyScalar(Math.max(1, 1 / camera.aspect)));
  orbit.update();
}
$('fit').onclick = fit;
window.addEventListener('keydown', (event) => {
  if (event.key === 'Escape' && !$('confirm-control').open) {
    if (dragging) { controls.reset(); queuePose(); flushPose(); }
    else closePose();
    event.preventDefault();
    return;
  }
  if (event.target.closest('input, select, textarea, button') || event.ctrlKey || event.metaKey || event.altKey) return;
  const key = event.key.toLowerCase();
  if (key === 't') setMode('translate');
  if (key === 'r') setMode('rotate');
  if (key === 'f') fit();
  if (['t', 'r', 'f'].includes(key)) event.preventDefault();
});
window.addEventListener('keydown', (event) => {
  if (event.key === 'Shift') { controls.setTranslationSnap(0.001); controls.setRotationSnap(rad(1)); }
});
window.addEventListener('keyup', (event) => {
  if (event.key === 'Shift') { controls.setTranslationSnap(null); controls.setRotationSnap(null); }
});
window.addEventListener('blur', () => { controls.setTranslationSnap(null); controls.setRotationSnap(null); });
fields.forEach((input) => { input.oninput = () => { poseDirty = true; }; });
$('pose-form').onsubmit = (event) => {
  event.preventDefault();
  if (!isIK()) return;
  const values = fields.map((input) => input.value.trim() === '' ? NaN : Number(input.value));
  if (!values.every(Number.isFinite)) return showError('Enter six finite pose values.');
  target.position.fromArray(values.slice(0, 3).map((x) => x / 1000));
  target.quaternion.setFromEuler(new THREE.Euler(...values.slice(3).map(rad), 'ZYX'));
  queuePose();
  flushPose();
  closePose();
};
function closePose() {
  const restoreFocus = $('pose-panel').contains(document.activeElement);
  $('pose-panel').hidden = true;
  $('edit-pose').setAttribute('aria-expanded', 'false');
  poseDirty = false;
  if (restoreFocus && !$('edit-pose').disabled) $('edit-pose').focus();
}
$('edit-pose').onclick = () => {
  const open = $('pose-panel').hidden;
  $('pose-panel').hidden = !open;
  $('edit-pose').setAttribute('aria-expanded', String(open));
  poseDirty = false;
  if (open) { updateFields(); fields[0].focus(); }
};
$('close-pose').onclick = () => { closePose(); $('edit-pose').focus(); };
function startControl() { send({ type: 'enable', aligned: true }); }
$('enable').onclick = () => {
  if (hardware) $('confirm-control').showModal();
  else startControl();
};
$('confirm-control').addEventListener('close', () => {
  if ($('confirm-control').returnValue === 'start' && connected && !state?.stopped) startControl();
});
$('hold').onclick = () => {
  pendingPose = false;
  closePose();
  showError('');
  send({ type: 'hold' });
};
$('stop').onclick = () => {
  pendingPose = false;
  closePose();
  send({ type: 'stop' });
};

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
  renderer.domElement.hidden = false;
  const alive = connected && !message.stopped;
  if (message.stopped && $('confirm-control').open) $('confirm-control').close('cancel');
  $('stop').disabled = !alive;
  if (readOnly) {
    $('control-status').textContent = message.stopped ? 'Viewer closed' : 'Live preview';
    document.body.dataset.status = alive ? 'live' : 'idle';
    showNotice(message.stopped ? 'Viewer closed. Restart the Python command to open it again.' : '');
    return;
  }
  const ik = isIK();
  controls.enabled = helper.visible = ik;
  $('enable').disabled = !alive;
  $('enable').hidden = message.enabled;
  $('active-tools').hidden = !message.enabled;
  ['translate', 'rotate', 'edit-pose', 'hold', 'apply-pose'].forEach((id) => { $(id).disabled = !ik; });
  fields.forEach((input) => { input.disabled = !ik; });
  const reached = message.position_error_mm <= 2 && message.orientation_error_deg <= 2;
  const status = message.stopped ? 'Stopped' : !message.enabled ? 'Holding position' : message.blocked ? 'Target blocked' : reached ? 'Ready' : 'Moving to target';
  $('control-status').textContent = status;
  document.body.dataset.status = !alive ? 'idle' : message.blocked ? 'blocked' : 'live';
  $('pose-error').textContent = `Target error: ${message.position_error_mm.toFixed(2)} mm · ${message.orientation_error_deg.toFixed(2)}°`;
  $('hint').textContent = message.stopped ? 'Restart the Python command to control again.' : !message.enabled ? 'Start control to move the end effector.' : controls.mode === 'rotate' ? 'Drag a ring to rotate · Shift for 1° steps' : 'Drag a handle to move · Shift for 1 mm steps';
  showNotice(commandError || (message.blocked ? 'Move the target away from the arm to avoid self-collision.' : ''), Boolean(commandError));
}
function connect() {
  socket = new WebSocket(`${location.protocol === 'https:' ? 'wss:' : 'ws:'}//${location.host}/ws`);
  socket.onmessage = (event) => {
    const message = JSON.parse(event.data);
    if (message.type === 'hello') {
      clientId = message.client;
      connected = true;
      targetVersion = -1; pendingPose = false; acknowledgedSequence = sentSequence = sequence = 0;
      send({ type: 'ready' });
    } else if (message.type === 'state') updateState(message);
    else if (message.type === 'error') showError(message.message);
  };
  socket.onclose = () => {
    connected = false; pendingPose = false; controls.enabled = helper.visible = false;
    orbit.enabled = true;
    document.body.dataset.status = 'idle';
    $('control-status').textContent = state?.stopped ? (readOnly ? 'Viewer closed' : 'Stopped') : 'Reconnecting…';
    showNotice(state?.stopped ? 'Restart the Python command to open the viewer again.' : readOnly ? 'Connection lost. Reconnecting…' : 'Connection lost. The arm keeps its last command. Reconnecting…');
    document.querySelectorAll('#control-bar button, #pose-form input, #pose-form button, #stop').forEach((element) => { element.disabled = true; });
    closePose();
    if ($('confirm-control').open) $('confirm-control').close('cancel');
    if (!state?.stopped) setTimeout(connect, 1000);
  };
  socket.onerror = () => { $('control-status').textContent = 'Connection unavailable'; };
}
new ResizeObserver(() => {
  const width = viewport.clientWidth, height = viewport.clientHeight;
  const previousScale = Math.max(1, 1 / camera.aspect);
  renderer.setSize(width, height);
  camera.aspect = width / height;
  controls.setSize(0.85 * Math.min(1, Math.max(0.6, camera.aspect)));
  camera.updateProjectionMatrix();
  if (cameraFrame) {
    const scale = Math.max(1, 1 / camera.aspect) / previousScale;
    camera.position.sub(orbit.target).multiplyScalar(scale).add(orbit.target);
    orbit.update();
  }
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
  hardware = model.hardware === true;
  document.body.classList.toggle('hardware', hardware);
  $('environment').textContent = readOnly ? 'Preview' : hardware ? 'Hardware' : 'Simulation';
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
    document.title = model.title;
    $('stop').textContent = 'Close viewer';
    actual.visible = target.visible = false;
  } else $('control-bar').hidden = false;
  connect();
} catch (error) {
  showError(error.message);
  $('control-status').textContent = 'Could not load viewer';
}
