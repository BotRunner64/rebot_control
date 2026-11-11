// Native playback shares the recording clock; paused images use original frame indices.
function ready(video, predicate, signal) {
  if (signal.aborted) return Promise.reject(new DOMException("Cancelled", "AbortError"));
  if (video.error) return Promise.reject(new Error("Could not decode the camera video."));
  if (predicate()) return Promise.resolve();
  return new Promise((resolve, reject) => {
    const events = ["loadedmetadata", "loadeddata", "canplay", "seeked", "progress"];
    const cleanup = () => {
      clearTimeout(timer);
      for (const event of events) video.removeEventListener(event, check);
      video.removeEventListener("error", failed);
      signal.removeEventListener("abort", cancelled);
    };
    const finish = (error) => { cleanup(); error ? reject(error) : resolve(); };
    const check = () => { if (predicate()) finish(); };
    const failed = () => finish(new Error("Could not decode the camera video."));
    const cancelled = () => finish(new DOMException("Cancelled", "AbortError"));
    const timer = setTimeout(() => finish(new Error("Camera video did not become ready.")), 20000);
    for (const event of events) video.addEventListener(event, check);
    video.addEventListener("error", failed);
    signal.addEventListener("abort", cancelled, {once: true});
  });
}

export class SynchronizedPlayback {
  constructor(views, session, {onPosition, onPlaying, onError}) {
    this.views = views;
    this.session = session;
    this.onPosition = onPosition;
    this.onPlaying = onPlaying;
    this.onError = onError;
    this.position = 0;
    this.playing = false;
    this.running = false;
    this.operation = new AbortController();
    this.listeners = [];
    for (const view of views) {
      const waiting = () => {
        if (!this.running || !view.active || !this.playing) return;
        this.restart();
      };
      const error = () => {
        if (!this.playing) return;
        this.pause();
        this.onError("Could not play the camera video.");
      };
      view.video.addEventListener("waiting", waiting);
      view.video.addEventListener("error", error);
      this.listeners.push(() => {
        view.video.removeEventListener("waiting", waiting);
        view.video.removeEventListener("error", error);
      });
    }
    const visibility = () => {
      if (!this.playing) return;
      this.cancel();
      this.halt();
      if (!document.hidden) this.synchronize(this.operation.signal);
    };
    document.addEventListener("visibilitychange", visibility);
    this.listeners.push(() => document.removeEventListener("visibilitychange", visibility));
  }

  cancel() {
    this.operation.abort();
    this.operation = new AbortController();
    clearTimeout(this.precisionTimer);
    cancelAnimationFrame(this.animation);
  }

  halt() {
    this.running = false;
    for (const {video} of this.views) { video.pause(); video.playbackRate = 1; }
  }

  seek(position) {
    this.cancel(); this.halt();
    this.playing = false;
    this.position = Math.max(0, Math.min(this.session.duration_ns - 1, Math.round(position)));
    this.onPlaying(false);
    this.onPosition(this.position);
    const signal = this.operation.signal;
    this.precisionTimer = setTimeout(() => this.preciseFrames(signal), 45);
  }

  pause() {
    this.cancel(); this.halt();
    this.playing = false;
    this.onPlaying(false);
    this.onPosition(this.position);
    this.preciseFrames(this.operation.signal);
  }

  play() {
    this.cancel(); this.halt();
    this.playing = true;
    this.onPlaying(true);
    this.synchronize(this.operation.signal);
  }

  restart() {
    this.cancel(); this.halt();
    this.synchronize(this.operation.signal);
  }

  active(view) {
    const end = Number.isFinite(view.video.duration)
      ? Math.min(view.camera.last_ns, view.camera.first_ns + view.video.duration * 1e9 - 1000000)
      : view.camera.last_ns;
    return this.position >= view.camera.first_ns && this.position < end;
  }

  async synchronize(signal) {
    if (document.hidden || signal.aborted) return;
    try {
      await Promise.all(this.views.map(async (view) => {
        const {video, camera} = view;
        await ready(video, () => video.readyState >= 1, signal);
        view.active = this.active(view);
        const seconds = Math.max(0, Math.min(video.duration - .000001, (this.position - camera.first_ns) / 1e9));
        if (Math.abs(video.currentTime - seconds) > .000001) video.currentTime = seconds;
        await ready(video, () => !video.seeking && video.readyState >= (view.active ? 3 : 2), signal);
      }));
      if (signal.aborted) return;
      await Promise.all(this.views.filter((view) => view.active).map(({video}) => video.play()));
      if (signal.aborted) return;
      for (const view of this.views) {
        view.image.hidden = true;
        view.video.hidden = this.position < view.camera.first_ns;
        view.error.textContent = "Camera has not started at this time.";
        view.error.hidden = !view.video.hidden;
      }
      this.anchor = this.position;
      this.startedAt = performance.now();
      this.running = true;
      this.animation = requestAnimationFrame(() => this.tick());
    } catch (error) {
      if (signal.aborted) return;
      this.pause(); this.onError(error.message);
    }
  }

  tick() {
    if (!this.running || !this.playing) return;
    const active = this.views.filter((view) => view.active);
    const master = active[0];
    // Follow decoded media time, so buffering never lets the playhead run ahead.
    const next = master ? master.camera.first_ns + master.video.currentTime * 1e9
      : this.anchor + (performance.now() - this.startedAt) * 1e6;
    this.position = Math.max(this.position, Math.min(this.session.duration_ns - 1, Math.round(next)));
    this.anchor = this.position; this.startedAt = performance.now();
    this.onPosition(this.position);
    if (this.position >= this.session.duration_ns - 1) { this.pause(); return; }
    if (this.views.some((view) => view.active !== this.active(view)) || active.some(({video}) => video.readyState < 3 || video.paused)) {
      this.restart(); return;
    }
    for (const view of active) {
      const offset = (this.position - view.camera.first_ns) / 1e9 - view.video.currentTime;
      if (Math.abs(offset) > .05) { this.restart(); return; }
      // Hysteresis avoids rebuilding the browser's media clock on every animation frame.
      const rate = view === master || Math.abs(offset) < .005 ? 1
        : offset > .015 ? 1.02 : offset < -.015 ? .98 : view.video.playbackRate;
      if (view.video.playbackRate !== rate) view.video.playbackRate = rate;
    }
    this.animation = requestAnimationFrame(() => this.tick());
  }

  async preciseFrames(signal) {
    const timestamp = this.position;
    const urls = [];
    try {
      const frames = await Promise.all(this.views.map(async (view) => {
        try {
          const query = new URLSearchParams({session: this.session.id, camera: view.camera.id, t: timestamp});
          const response = await fetch(`/api/frame?${query}`, {signal});
          if (!response.ok) throw new Error((await response.json()).error);
          const url = URL.createObjectURL(await response.blob());
          urls.push(url);
          const decoded = new Image(); decoded.src = url;
          await decoded.decode();
          return {view, url};
        } catch (error) { return {view, error}; }
      }));
      if (signal.aborted) return;
      // Publish all cameras together; an older drag request can never replace a newer frame.
      for (const {view, url, error} of frames) {
        if (view.url) URL.revokeObjectURL(view.url);
        view.url = url;
        view.image.dataset.timestampNs = String(timestamp);
        view.video.hidden = true;
        view.image.hidden = !!error;
        view.error.hidden = !error;
        if (error) view.error.textContent = error.message;
        else view.image.src = url;
      }
    } finally {
      for (const url of urls) {
        if (signal.aborted || !this.views.some((view) => view.url === url)) URL.revokeObjectURL(url);
      }
    }
  }

  dispose() {
    this.cancel(); this.halt();
    for (const remove of this.listeners) remove();
    for (const view of this.views) {
      if (view.url) URL.revokeObjectURL(view.url);
      view.video.removeAttribute("src"); view.video.load();
    }
  }
}
