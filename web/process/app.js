const $ = (id) => document.getElementById(id);
const clone = (value) => structuredClone(value);
let workspace, plan, session, selected = null, selectedGap = null, detecting = false, position = 0, playing = false;
let times = [], zoom = 1, history = [], drag = null;
let saveTimer, saveQueue = Promise.resolve(), revision = 0, savedRevision = 0;
let previewRevision = 0, frameBusy = false, previewNeeded = false;
let positionAnchor = 0, clockAnchor = 0, jobTimer, exporting = false;
const cameraViews = new Map();

function time(value) {
  const ms = Math.floor(value / 1e6);
  return `${String(Math.floor(ms / 60000)).padStart(2, "0")}:${String(Math.floor(ms / 1000) % 60).padStart(2, "0")}.${String(ms % 1000).padStart(3, "0")}`;
}
function element(tag, className, text) {
  const node = document.createElement(tag);
  if (className) node.className = className;
  if (text !== undefined) node.textContent = text;
  return node;
}
async function api(path, body) {
  const response = await fetch(path, body === undefined ? {} : {method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify(body)});
  const data = await response.json();
  if (!response.ok) throw new Error(data.error || `Request failed (${response.status})`);
  return data;
}
function notice(message, error = false) {
  $("notice").hidden = !message;
  $("notice").textContent = message;
  $("notice").classList.toggle("error", error);
}
function entries() {
  return plan.episodes.filter((item) => item.session === session.id).sort((a, b) => a.start_ns - b.start_ns);
}
function checkpoint() {
  const snapshot = {plan: clone(plan), session: session.id, selected: selected?.id,
    gap: selectedGap ? clone(selectedGap) : null, position};
  if (JSON.stringify(history.at(-1)) !== JSON.stringify(snapshot)) history.push(snapshot);
  if (history.length > 80) history.shift();
  $("undo").disabled = !history.length;
}
function changed() {
  revision++;
  if (!detecting && !exporting) $("save-status").textContent = "Unsaved";
  clearTimeout(saveTimer);
  saveTimer = setTimeout(() => save().catch((error) => notice(error.message, true)), 600);
}
function save() {
  clearTimeout(saveTimer);
  const snapshot = clone(plan), current = revision;
  saveQueue = saveQueue.catch(() => {}).then(async () => {
    if (!detecting && !exporting) $("save-status").textContent = "Saving…";
    await api("/api/plan", snapshot);
    savedRevision = current;
    if (!detecting && !exporting) $("save-status").textContent = current === revision ? "Saved" : "Unsaved";
  }).catch((error) => {
    $("save-status").textContent = "Save failed";
    throw error;
  });
  return saveQueue;
}
function nextId() {
  let index = plan.episodes.length + 1, name;
  do { name = `episode_${String(index++).padStart(4, "0")}`; } while (plan.episodes.some((item) => item.id === name));
  return name;
}
function makeEpisode(start, end, instruction = recordingInstruction()) {
  const item = {id: nextId(), session: session.id, start_ns: start, end_ns: end, instruction};
  plan.episodes.push(item);
  return item;
}
function recordingInstruction(sid = session.id) {
  return plan.recording_instructions[sid] || "";
}
async function suggestEpisodes() {
  if (detecting || plan.auto_split_sessions.includes(session.id)) return;
  const sid = session.id;
  detecting = true; setPlaying(false); updateStatus();
  $("save-status").textContent = "Splitting…";
  try {
    const result = await api(`/api/suggest?${new URLSearchParams({session: sid})}`);
    // Record completion before the undo snapshot. Restoring a previous timeline
    // must not trigger another automatic split when the draft is reopened.
    plan.auto_split_sessions.push(sid);
    if (result.usable) {
      checkpoint();
      plan.episodes = plan.episodes.filter((item) => item.session !== sid);
      for (const range of result.episodes) makeEpisode(range.start_ns, range.end_ns);
      plan.reset_suggestions ??= {};
      plan.reset_suggestions[sid] = result;
      selected = entries()[0] || null; selectedGap = null;
      renderEditor(); renderTimeline();
      if (selected) seek(selected.start_ns, true);
    }
    changed();
    await save();
    $("save-status").textContent = result.usable ? "Saved" : "No reset found";
  } catch (error) { notice(error.message, true); $("save-status").textContent = "Split failed"; }
  finally { detecting = false; updateStatus(); }
}
function setPlaying(value) {
  playing = value;
  positionAnchor = position;
  clockAnchor = performance.now();
  $("play").textContent = value ? "Ⅱ" : "▶";
  $("play").setAttribute("aria-label", value ? "Pause" : "Play");
}
function drawPlayhead(follow = false) {
  const fraction = position / session.duration_ns;
  $("playhead").style.left = `${fraction * 100}%`;
  $("playhead").setAttribute("aria-valuenow", position);
  $("playhead").setAttribute("aria-valuetext", time(position));
  $("clock").textContent = time(position);
  if (follow) {
    const scroll = $("timeline-scroll"), x = fraction * $("timeline-content").clientWidth;
    if (x < scroll.scrollLeft || x > scroll.scrollLeft + scroll.clientWidth - 20) scroll.scrollLeft = Math.max(0, x - scroll.clientWidth / 3);
  }
}
function seek(value, follow = false) {
  position = Math.max(0, Math.min(session.duration_ns - 1, Math.round(value)));
  positionAnchor = position;
  clockAnchor = performance.now();
  drawPlayhead(follow);
  refreshFrames();
}
function upperBound(value) {
  let low = 0, high = times.length;
  while (low < high) { const mid = (low + high) >>> 1; if (times[mid] <= value) low = mid + 1; else high = mid; }
  return low;
}
function adjacentFrame(value, direction) {
  if (!times.length) return value + direction * 1e6;
  const upper = upperBound(value);
  const index = direction > 0 ? upper : (upper > 0 && times[upper - 1] === value ? upper - 2 : upper - 1);
  return times[Math.max(0, Math.min(times.length - 1, index))];
}
function snap(value) {
  if (!times.length) return Math.round(value);
  const upper = upperBound(value), before = times[Math.max(0, upper - 1)], after = times[Math.min(times.length - 1, upper)];
  return Math.abs(value - before) <= Math.abs(value - after) ? before : after;
}
async function refreshFrames() {
  previewNeeded = true;
  if (frameBusy || !session) return;
  previewNeeded = false; frameBusy = true;
  const token = previewRevision, current = session, timestamp = position;
  await Promise.all(current.cameras.map(async (camera) => {
    const view = cameraViews.get(camera.id);
    if (!view) return;
    try {
      const query = new URLSearchParams({session: current.id, camera: camera.id, t: timestamp});
      const response = await fetch(`/api/frame?${query}`);
      if (!response.ok) throw new Error((await response.json()).error);
      const blob = await response.blob();
      if (token !== previewRevision) return;
      if (view.url) URL.revokeObjectURL(view.url);
      view.url = URL.createObjectURL(blob);
      view.image.src = view.url;
      view.error.hidden = true; view.image.hidden = false;
    } catch (error) {
      if (token !== previewRevision) return;
      view.error.textContent = error.message;
      view.error.hidden = false; view.image.hidden = true;
    }
  }));
  frameBusy = false;
  if (previewNeeded) setTimeout(refreshFrames, 30);
}
async function loadFrameTimes() {
  const camera = session.cameras[0].id;
  times = [];
  const sid = session.id;
  try {
    const result = await api(`/api/times?${new URLSearchParams({session: sid, camera})}`);
    if (sid === session.id) times = result;
  } catch (error) { notice(error.message, true); }
}
async function chooseSession(id, preferredId) {
  setPlaying(false);
  session = workspace.sessions.find((item) => item.id === id);
  $("recording").value = id;
  previewRevision++; zoom = 1;
  for (const view of cameraViews.values()) if (view.url) URL.revokeObjectURL(view.url);
  cameraViews.clear(); $("cameras").replaceChildren();
  for (const camera of session.cameras) {
    const card = element("div", "camera-view");
    const image = element("img"); image.alt = `Preview from ${camera.id}`;
    const error = element("span", "frame-error", "Loading…");
    const name = element("input", "camera-name");
    name.value = plan.camera_names[camera.id];
    name.setAttribute("aria-label", `Camera name: ${camera.id}`);
    name.title = camera.id;
    name.pattern = "[a-z0-9_]+";
    name.onfocus = checkpoint;
    name.oninput = () => { plan.camera_names[camera.id] = name.value.trim(); changed(); };
    card.append(image, error, name); $("cameras").append(card);
    cameraViews.set(camera.id, {card, image, error, name});
  }
  selectedGap = null;
  // A recording starts as one episode. Only cuts and discards change its structure.
  if (!plan.initialized_sessions.includes(id)) {
    if (!entries().length) makeEpisode(0, session.duration_ns);
    plan.initialized_sessions.push(id);
    changed();
  }
  selected = entries().find((item) => item.id === preferredId) || entries()[0] || null;
  $("duration").textContent = time(session.duration_ns);
  $("playhead").setAttribute("aria-valuemax", session.duration_ns - 1);
  renderEditor(); renderTimeline();
  await loadFrameTimes();
  const first = Math.max(...session.cameras.map((camera) => camera.first_ns || 0));
  seek(Math.min(session.duration_ns - 1, Math.max(first, selected?.start_ns || 0)));
  await suggestEpisodes();
}
function selectEpisode(item, at = null) {
  selected = item; selectedGap = null;
  setPlaying(false);
  const firstFrame = Math.max(...session.cameras.map((camera) => camera.first_ns || 0));
  const initial = Math.min(item.end_ns - 1, Math.max(item.start_ns, firstFrame));
  renderEditor(); renderSelection(); seek(at ?? initial);
}
function renderEditor() {
  $("instruction").disabled = false;
  $("instruction").value = recordingInstruction();
  $("selection-action").hidden = !selected && !selectedGap;
  $("selection-action").textContent = selected ? "Exclude" : "Include";
  updateStatus();
}
function updateStatus() {
  $("export").textContent = exporting ? "Exporting…" : "Export";
  $("export").disabled = exporting || detecting || !plan.episodes.length;
  $("undo").disabled = detecting || !history.length;
  $("recording").disabled = detecting;
  $("play").disabled = detecting;
  $("timeline-clips").inert = detecting;
  $("episode-count").textContent = `${entries().length} episode${entries().length === 1 ? "" : "s"}`;
}
function timelineValue(clientX) {
  const rect = $("timeline-content").getBoundingClientRect();
  return Math.max(0, Math.min(session.duration_ns - 1, Math.round((clientX - rect.left) / rect.width * session.duration_ns)));
}
function place(node, start, end) {
  node.style.left = `${100 * start / session.duration_ns}%`;
  node.style.width = `${100 * (end - start) / session.duration_ns}%`;
}
function renderTimeline() {
  $("timeline-content").style.width = `${zoom * 100}%`;
  $("ruler").replaceChildren();
  const ticks = Math.min(100, Math.max(4, Math.round($("timeline-content").clientWidth / 100)));
  for (let i = 0; i <= ticks; i++) {
    const tick = element("span", "", time(session.duration_ns * i / ticks));
    tick.style.left = `${i * 100 / ticks}%`; $("ruler").append(tick);
  }
  const track = $("timeline-clips"); track.replaceChildren();
  const items = entries(); let end = 0;
  // Keep old overlapping plans editable without flattening or silently changing them.
  const laneEnds = [];
  for (const [index, item] of items.entries()) {
    if (item.start_ns > end) addGap(end, item.start_ns);
    end = Math.max(end, item.end_ns);
    let lane = laneEnds.findIndex((value) => value <= item.start_ns);
    if (lane < 0) lane = laneEnds.length;
    laneEnds[lane] = item.end_ns;
    const button = element("button", `timeline-clip${item === selected ? " selected" : ""}`);
    const title = `Episode ${String(index + 1).padStart(2, "0")}`;
    button.append(element("span", "clip-title", String(index + 1).padStart(2, "0")));
    button.title = `${item.id}: ${time(item.start_ns)} – ${time(item.end_ns)}. Double-click to cut.`;
    button.setAttribute("aria-label", title);
    button.dataset.episodeId = item.id;
    button.dataset.lane = lane;
    button.style.top = `${1 + lane * 70}px`; place(button, item.start_ns, item.end_ns);
    button.onclick = (event) => {
      const value = event.detail === 0 ? item.start_ns : timelineValue(event.clientX);
      selectEpisode(item, value);
    };
    track.append(button);

  }
  if (end < session.duration_ns) addGap(end, session.duration_ns);
  track.style.height = `${Math.max(1, laneEnds.length) * 70}px`;
  renderSelection(); drawPlayhead(); updateStatus();
}
function renderSelection() {
  const track = $("timeline-clips");
  track.querySelectorAll(".trim-handle").forEach((node) => node.remove());
  const clips = [...track.querySelectorAll(".timeline-clip")];
  for (const clip of clips) clip.classList.toggle("selected", clip.dataset.episodeId === selected?.id);
  for (const gap of track.querySelectorAll(".timeline-gap")) gap.classList.toggle("selected", !!selectedGap && Number(gap.dataset.start) === selectedGap.start_ns && Number(gap.dataset.end) === selectedGap.end_ns);
  const item = selected || selectedGap;
  if (!item) return;
  const isGap = !selected;
  const clip = isGap ? [...track.querySelectorAll(".timeline-gap")].find((node) => Number(node.dataset.start) === item.start_ns && Number(node.dataset.end) === item.end_ns) : clips.find((node) => node.dataset.episodeId === item.id);
  if (!clip) { if (!isGap) renderTimeline(); return; }
  const lane = Number(clip.dataset.lane || 0);
  for (const side of ["start", "end"]) {
    if (isGap && !entries().some((entry) => side === "start" ? entry.end_ns === item.start_ns : entry.start_ns === item.end_ns)) continue;
    const handle = element("button", `trim-handle${isGap ? " excluded" : ""}`, side === "start" ? "❮" : "❯");
    handle.dataset.side = side;
    handle.setAttribute("role", "slider"); handle.setAttribute("aria-label", `${isGap ? "Excluded range" : "Episode"} ${side} boundary`);
    handle.setAttribute("aria-valuemin", "0"); handle.setAttribute("aria-valuemax", session.duration_ns);
    handle.setAttribute("aria-valuenow", item[`${side}_ns`]);
    handle.setAttribute("aria-valuetext", time(item[`${side}_ns`]));
    handle.style.left = `${100 * item[`${side}_ns`] / session.duration_ns}%`;
    handle.style.top = `${-2 + lane * 70}px`;
    handle.onpointerdown = (event) => startDrag(event, side, item, isGap);
    handle.onkeydown = (event) => {
      if (!["ArrowLeft", "ArrowRight"].includes(event.key)) return;
      event.preventDefault(); event.stopPropagation(); checkpoint();
      const value = adjacentFrame(item[`${side}_ns`], event.key === "ArrowLeft" ? -1 : 1);
      if (isGap) moveGapBoundary(item, side, value); else moveBoundary(item, side, value);
      $("timeline-clips").querySelector(`[data-side="${side}"]`)?.focus({preventScroll: true});
    };
    track.append(handle);
  }
}
function addGap(start, end) {
  const button = element("button", "timeline-gap");
  place(button, start, end);
  button.dataset.start = start; button.dataset.end = end;
  button.setAttribute("aria-label", `Excluded range ${time(start)} to ${time(end)}`);
  button.title = "Excluded from export. Click to review and drag its edges.";
  button.onclick = (event) => {
    selected = null; selectedGap = {start_ns: start, end_ns: end};
    setPlaying(false); renderEditor(); renderSelection();
    seek(event.detail === 0 ? start : timelineValue(event.clientX));
  };
  $("timeline-clips").append(button);
}
function moveGapBoundary(gap, side, value) {
  const previous = entries().find((item) => item.end_ns === gap.start_ns);
  const next = entries().find((item) => item.start_ns === gap.end_ns);
  if (side === "start" && previous) {
    value = Math.max(previous.start_ns + 1, Math.min(gap.end_ns - 1, value));
    previous.end_ns = value; gap.start_ns = value;
  } else if (side === "end" && next) {
    value = Math.min(next.end_ns - 1, Math.max(gap.start_ns + 1, value));
    next.start_ns = value; gap.end_ns = value;
  } else return;
  changed(); renderEditor(); renderTimeline(); seek(Math.min(value, session.duration_ns - 1));
}
function cutAt(value, preferred) {
  if (detecting) return;
  let boundary = snap(value);
  const item = preferred || entries().find((entry) => entry.start_ns < value && value < entry.end_ns);
  if (!item) return;
  if (!(item.start_ns < boundary && boundary < item.end_ns)) boundary = Math.round(value);
  if (!(item.start_ns < boundary && boundary < item.end_ns)) return;
  checkpoint();
  const right = makeEpisode(boundary, item.end_ns, item.instruction);
  item.end_ns = boundary;
  changed(); selectEpisode(right, boundary);
  notice("");
}
function moveBoundary(item, side, value) {
  const items = entries(), index = items.indexOf(item), previous = items[index - 1], next = items[index + 1];
  const old = item[`${side}_ns`];
  if (side === "start") {
    const linked = previous && previous.end_ns === old;
    const minimum = linked ? previous.start_ns + 1 : previous && previous.end_ns <= old ? previous.end_ns : 0;
    value = Math.max(minimum, Math.min(item.end_ns - 1, value));
    if (linked) previous.end_ns = value;
    item.start_ns = value;
  } else {
    const linked = next && next.start_ns === old;
    const maximum = linked ? next.end_ns - 1 : next && next.start_ns >= old ? next.start_ns : session.duration_ns;
    value = Math.min(maximum, Math.max(item.start_ns + 1, value));
    if (linked) next.start_ns = value;
    item.end_ns = value;
  }
  changed(); renderEditor(); renderTimeline();
  seek(Math.min(value, session.duration_ns - 1));
}
function startDrag(event, side, item = null, gap = false) {
  if (event.button !== 0 || detecting) return;
  event.preventDefault(); event.stopPropagation();
  setPlaying(false);
  drag = {side, item, gap, pointer: event.pointerId, checkpointed: false};
  $("timeline-content").setPointerCapture(event.pointerId);
}
$("timeline-content").onpointermove = (event) => {
  if (!drag || event.pointerId !== drag.pointer) return;
  if (drag.side === "playhead") { seek(timelineValue(event.clientX)); return; }
  if (!drag.checkpointed) { checkpoint(); drag.checkpointed = true; }
  const value = snap(timelineValue(event.clientX));
  if (drag.gap) moveGapBoundary(drag.item, drag.side, value); else moveBoundary(drag.item, drag.side, value);
};
function endDrag(event) {
  if (!drag || event.pointerId !== drag.pointer) return;
  const completed = drag;
  drag = null;
  if ($("timeline-content").hasPointerCapture(event.pointerId)) $("timeline-content").releasePointerCapture(event.pointerId);
  if (completed.item) $("timeline-clips").querySelector(`[data-side="${completed.side}"]`)?.focus({preventScroll: true});
}
$("timeline-content").onpointerup = endDrag;
$("timeline-content").onpointercancel = endDrag;
$("playhead").onpointerdown = (event) => startDrag(event, "playhead");
$("timeline-clips").onclick = (event) => { if (event.target === $("timeline-clips")) { setPlaying(false); seek(timelineValue(event.clientX)); } };
$("timeline-clips").ondblclick = (event) => {
  if (event.target.closest(".trim-handle, .timeline-gap")) return;
  event.preventDefault(); cutAt(timelineValue(event.clientX));
};
$("ruler").onclick = (event) => { setPlaying(false); seek(timelineValue(event.clientX)); };
$("timeline-scroll").addEventListener("wheel", (event) => {
  if (!event.ctrlKey && !event.metaKey) return;
  event.preventDefault();
  const scroll = $("timeline-scroll"), rect = scroll.getBoundingClientRect();
  const point = event.clientX - rect.left;
  const fraction = (scroll.scrollLeft + point) / $("timeline-content").clientWidth;
  zoom = Math.max(1, Math.min(128, zoom * Math.exp(-event.deltaY * .008)));
  renderTimeline();
  scroll.scrollLeft = fraction * $("timeline-content").clientWidth - point;
}, {passive: false});
async function undo() {
  if (!history.length || detecting) return;
  const snapshot = history.pop(); plan = snapshot.plan; selectedGap = null;
  if (snapshot.session !== session.id) await chooseSession(snapshot.session, snapshot.selected);
  else {
    selectedGap = snapshot.gap;
    selected = selectedGap ? null : entries().find((item) => item.id === snapshot.selected) || entries()[0] || null;
    renderEditor(); renderTimeline();
  }
  $("undo").disabled = !history.length;
  changed();
  seek(snapshot.position, true);
  for (const [name, view] of cameraViews) view.name.value = plan.camera_names[name];
  notice("");
}
async function pollJob() {
  try {
    const job = await api("/api/job"); exporting = job.state === "running";
    updateStatus();
    if (exporting) {
      $("save-status").textContent = `${job.completed} / ${job.total}`;
      jobTimer = setTimeout(pollJob, 1000);
    } else if (job.state === "done") {
      $("save-status").textContent = "Exported";
      $("export").title = job.output;
    } else if (job.state === "error") { notice(job.error, true); }
  } catch (error) { notice(error.message, true); jobTimer = setTimeout(pollJob, 3000); }
}
$("recording").onchange = () => chooseSession($("recording").value);
$("play").onclick = () => {
  if (!playing && position >= session.duration_ns - 1) seek(0);
  setPlaying(!playing);
};
$("instruction").onfocus = checkpoint;
$("instruction").oninput = () => {
  plan.recording_instructions[session.id] = $("instruction").value;
  for (const item of entries()) item.instruction = $("instruction").value;
  changed(); updateStatus();
};
$("selection-action").onclick = () => {
  if (detecting) return;
  if (selectedGap) {
    checkpoint(); const item = makeEpisode(selectedGap.start_ns, selectedGap.end_ns);
    changed(); selectEpisode(item);
  } else if (selected) {
    checkpoint(); const index = entries().indexOf(selected);
    plan.episodes.splice(plan.episodes.indexOf(selected), 1);
    selected = entries()[Math.min(index, entries().length - 1)] || null;
    setPlaying(false); changed(); renderEditor(); renderTimeline();
  }
  notice("");
};
$("undo").onclick = undo;
$("export").onclick = async () => {
  const missing = plan.episodes.find((item) => !recordingInstruction(item.session).trim() || recordingInstruction(item.session).trim() === "__pending__");
  if (missing) {
    if (missing.session !== session.id) await chooseSession(missing.session, missing.id); else selectEpisode(missing);
    $("instruction").focus(); notice("Add an instruction."); return;
  }
  $("export").disabled = true;
  try { await save(); await api("/api/export", plan); clearTimeout(jobTimer); await pollJob(); }
  catch (error) { updateStatus(); notice(error.message, true); }
};
document.addEventListener("keydown", (event) => {
  const typing = ["INPUT", "TEXTAREA", "SELECT"].includes(event.target.tagName) || event.target.isContentEditable;
  if ((event.metaKey || event.ctrlKey) && event.key.toLowerCase() === "s") { event.preventDefault(); save().catch((error) => notice(error.message, true)); return; }
  if (typing || !session) return;
  if ((event.metaKey || event.ctrlKey) && event.key.toLowerCase() === "z") { event.preventDefault(); undo(); }
  else if (event.code === "Space") { event.preventDefault(); $("play").click(); }
  else if (event.key === "ArrowLeft") { event.preventDefault(); setPlaying(false); seek(adjacentFrame(position, -1), true); }
  else if (event.key === "ArrowRight") { event.preventDefault(); setPlaying(false); seek(adjacentFrame(position, 1), true); }
  else if (!event.metaKey && !event.ctrlKey && event.key.toLowerCase() === "s") { event.preventDefault(); cutAt(position); }
});
window.addEventListener("beforeunload", (event) => { if (revision !== savedRevision) { event.preventDefault(); event.returnValue = ""; } });
window.addEventListener("resize", () => { if (session) renderTimeline(); });
setInterval(() => {
  if (!playing || !session) return;
  const end = session.duration_ns - 1;
  position = Math.min(end, Math.round(positionAnchor + (performance.now() - clockAnchor) * 1e6));
  drawPlayhead(true); refreshFrames();
  if (position >= end) setPlaying(false);
}, 100);
async function init() {
  try {
    workspace = await api("/api/workspace"); plan = workspace.plan;
    plan.recording_instructions ??= {};
    plan.auto_split_sessions ??= Object.keys(plan.reset_suggestions || {}).filter((id) => workspace.sessions.some((item) => item.id === id));
    for (const recording of workspace.sessions) {
      if (!(recording.id in plan.recording_instructions)) {
        const texts = [...new Set(plan.episodes.filter((item) => item.session === recording.id).map((item) => item.instruction).filter((text) => text && text !== "__pending__"))];
        if (texts.length <= 1) plan.recording_instructions[recording.id] = texts[0] || "";
      }
    }
    plan.initialized_sessions ??= [...new Set(plan.episodes.map((item) => item.session))];
    $("save-status").textContent = "Saved";
    $("export").title = workspace.output;
    $("recording").hidden = workspace.sessions.length < 2;
    for (const [index, item] of workspace.sessions.entries()) {
      const option = element("option", "", `Recording ${String(index + 1).padStart(2, "0")}`);
      option.value = item.id; option.title = item.id; $("recording").append(option);
    }
    await chooseSession(workspace.sessions[0].id); await pollJob();
  } catch (error) { notice(error.message, true); $("save-status").textContent = "Unavailable"; }
}
init();
