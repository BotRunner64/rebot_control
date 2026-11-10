"use strict";

const grid = document.querySelector("#cameras");
const summary = document.querySelector("#summary");
const notice = document.querySelector("#notice");
const closeButton = document.querySelector("#close-viewer");
const cards = new Map();
let focused = null;
let closing = false;
let timer = null;
let failures = 0;

function showNotice(message) {
  notice.textContent = message;
  notice.hidden = !message;
}

function applyFocus() {
  grid.classList.toggle("focused", focused !== null);
  for (const [id, card] of cards) {
    card.root.hidden = focused !== null && focused !== id;
    const expanded = focused === id;
    card.expand.setAttribute("aria-pressed", String(expanded));
    card.expand.setAttribute("aria-label", expanded ? "Show all cameras" : `Expand camera ${id + 1}`);
    card.expand.querySelector("span").textContent = expanded ? "All cameras" : "Expand view";
  }
}

function placeholder(card, title, detail = "") {
  card.placeholder.hidden = false;
  card.root.querySelector(".placeholder-title").textContent = title;
  const detailElement = card.root.querySelector(".placeholder-detail");
  detailElement.textContent = detail;
  detailElement.hidden = !detail;
}

function createCard(camera) {
  const root = document.querySelector("#camera-template").content.firstElementChild.cloneNode(true);
  const card = {root, image: root.querySelector("img"), placeholder: root.querySelector(".placeholder"),
    expand: root.querySelector(".expand"), loaded: false, streamFailed: false, state: "waiting"};
  root.querySelector("h2").textContent = `Camera ${camera.id + 1}`;
  root.querySelector(".source").textContent = camera.source;
  card.image.alt = `Live preview of ${camera.source}`;
  card.expand.setAttribute("aria-label", `Expand camera ${camera.id + 1}`);
  card.expand.addEventListener("click", () => {
    focused = focused === camera.id ? null : camera.id;
    applyFocus();
  });
  card.image.addEventListener("load", () => {
    card.loaded = true;
    card.streamFailed = false;
    if (card.state === "live" && !closing && failures === 0) card.placeholder.hidden = true;
  });
  card.image.addEventListener("error", () => {
    card.loaded = false;
    card.streamFailed = true;
    if (!closing) placeholder(card, "Preview interrupted", "Trying to reconnect...");
  });
  cards.set(camera.id, card);
  grid.append(root);
  card.image.src = `/streams/${camera.id}.mjpg`;
  return card;
}

function render(cameras) {
  const live = cameras.filter(camera => camera.state === "live").length;
  summary.textContent = `${live} of ${cameras.length} ${cameras.length === 1 ? "camera" : "cameras"} live`;
  for (const camera of cameras) {
    const card = cards.get(camera.id) || createCard(camera);
    card.state = camera.state;
    const status = card.root.querySelector(".status");
    status.dataset.state = camera.state;
    const labels = {live: "Live", error: "Unavailable", waiting: "Waiting", stopped: "Stopped"};
    card.root.querySelector(".status-text").textContent = labels[camera.state] || "Waiting";
    card.root.querySelector(".capture-mode").textContent = camera.mode;
    if (camera.state === "error") {
      placeholder(card, "Camera unavailable", camera.error || "Check the camera connection.");
      card.image.removeAttribute("src");
    } else if (camera.state === "live" && card.loaded) {
      card.placeholder.hidden = true;
    } else {
      placeholder(card, "Waiting for camera");
    }
    if (camera.state === "live" && card.streamFailed) {
      card.streamFailed = false;
      card.image.src = `/streams/${camera.id}.mjpg?retry=${Date.now()}`;
    }
  }
  applyFocus();
}

async function poll() {
  if (closing) return;
  try {
    const response = await fetch("/api/cameras", {cache: "no-store", signal: AbortSignal.timeout(3000)});
    if (!response.ok) throw new Error("Viewer unavailable");
    const data = await response.json();
    if (closing) return;
    if (failures > 0) {
      for (const card of cards.values()) {
        card.loaded = false;
        card.streamFailed = true;
      }
    }
    failures = 0;
    showNotice("");
    render(data.cameras);
  } catch {
    if (closing) return;
    failures += 1;
    summary.textContent = "Disconnected";
    showNotice("The viewer is unavailable. Reconnecting automatically...");
    for (const card of cards.values()) {
      card.root.querySelector(".status").dataset.state = "waiting";
      card.root.querySelector(".status-text").textContent = "Disconnected";
      placeholder(card, "Connection interrupted", "Waiting for the viewer to reconnect.");
    }
  }
  if (!closing) timer = setTimeout(poll, 1000);
}

async function stopViewer() {
  if (closing) return;
  closing = true;
  closeButton.disabled = true;
  closeButton.textContent = "Closing...";
  clearTimeout(timer);
  try {
    const response = await fetch("/api/stop", {method: "POST", headers: {"Content-Type": "application/json"},
      body: "{}", signal: AbortSignal.timeout(3000)});
    if (!response.ok) throw new Error("Could not close viewer");
    summary.textContent = "Viewer stopped";
    closeButton.textContent = "Closed";
    showNotice("The viewer has closed. Restart webcam_viewer.py to preview again.");
    for (const card of cards.values()) {
      card.image.removeAttribute("src");
      card.expand.disabled = true;
      card.root.querySelector(".status").dataset.state = "stopped";
      card.root.querySelector(".status-text").textContent = "Stopped";
      placeholder(card, "Preview ended", "The camera has been released.");
    }
  } catch {
    closing = false;
    closeButton.disabled = false;
    closeButton.textContent = "Close viewer";
    showNotice("Could not close the viewer. Try again, or press Ctrl-C in the terminal.");
    timer = setTimeout(poll, 1000);
  }
}

closeButton.addEventListener("click", stopViewer);
document.addEventListener("keydown", event => {
  if (event.ctrlKey || event.metaKey || event.altKey || event.repeat) return;
  if (event.key === "Escape" && focused !== null) {
    focused = null;
    applyFocus();
  } else if (event.key.toLowerCase() === "q" || event.key === "Escape") {
    event.preventDefault();
    stopViewer();
  }
});
poll();
