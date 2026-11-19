"""Review continuous recordings and export annotated episodes without changing raw data."""

from bisect import bisect_right
import fcntl
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import re
import shutil
import tempfile
import threading
from urllib.parse import parse_qs, urlencode, urlsplit

import numpy as np
import pyarrow.parquet as pq

from loom_data import contract, sync

from rebot.data.previews import PreviewCache
from rebot.data.recording import CAMERA_ID, identifier
from rebot.data.reset_detection import DEFAULTS, STEP_NS, suggest_ranges
from rebot.data.synchronous import stage_clip
from rebot.paths import WEB


ASSETS = WEB / "process"


def write_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    handle, temporary = tempfile.mkstemp(prefix=".draft-", dir=path.parent)
    try:
        with os.fdopen(handle, "w", encoding="utf-8") as stream:
            json.dump(value, stream, ensure_ascii=False, allow_nan=False, indent=2)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        Path(temporary).unlink(missing_ok=True)


class Workspace:
    def __init__(self, dataset, output, plan):
        self.dataset, self.output, self.plan_path = map(lambda p: Path(p).resolve(), (dataset, output, plan))
        if (self.output == self.dataset or self.dataset in self.output.parents
                or self.output in self.dataset.parents):
            raise ValueError("Raw and output dataset paths must be separate, non-nested directories.")
        if self.plan_path == self.dataset or self.dataset in self.plan_path.parents:
            raise ValueError("Keep the processing plan outside the raw dataset.")
        if self.plan_path == self.output or self.output in self.plan_path.parents:
            raise ValueError("Keep the processing plan outside the output dataset.")
        self.sessions, self.times, self.instructions = {}, {}, {}
        for manifest in sorted((self.dataset / "recordings").glob("*/recording.json")):
            name = identifier(manifest.parent.name, "recording ID")
            record = json.loads(self.path(name, "recording.json").read_text(encoding="utf-8"))
            cameras, count = record["cameras"], record["frames"]
            if not isinstance(cameras, list) or not 1 <= len(cameras) <= 3 or len(set(cameras)) != len(cameras):
                raise ValueError(f"{name}: expected one to three distinct cameras.")
            frames = pq.ParquetFile(self.path(name, contract.FRAMES))
            if (type(count) is not int or count < 2 or count != frames.metadata.num_rows
                    or frames.schema_arrow != contract.FRAME_SCHEMA):
                raise ValueError(f"{name}: invalid synchronized frame table.")
            times = contract.frame_times(count)
            views = []
            for camera in cameras:
                identifier(camera, "camera name", CAMERA_ID)
                self.path(name, camera + ".mp4")
                views.append({"id": camera, "frames": count, "first_ns": 0, "last_ns": int(times[-1])})
            self.sessions[name] = {"id": name, "duration_ns": sync.frame_ns(count), "cameras": views}
            self.times[name] = times
            instruction = record["instruction"].strip()
            if instruction and instruction != "__pending__":
                self.instructions[name] = instruction
        if not self.sessions:
            raise ValueError("No finalized raw recordings found.")
        self.guard = threading.Lock()
        self.job = {"state": "idle"}
        self.captures, self.capture_lock = {}, threading.Lock()
        self.analysis_cache = {}
        self.previews = PreviewCache(self.plan_path.parent / ".rebot-preview")

    def preview(self, session):
        if session not in self.sessions:
            raise ValueError("Unknown recording.")
        sources = {camera["id"]: self.path(session, camera["id"] + ".mp4")
                   for camera in self.sessions[session]["cameras"]}
        result = self.previews.prepare(sources.values())
        if result["state"] == "ready":
            result["videos"] = {camera: "/api/video?" + urlencode({"session": session, "camera": camera,
                                  "preview": "1", "key": self.previews.path(source).stem})
                                for camera, source in sources.items()}
        return result

    def path(self, session, filename):
        path = (self.dataset / "recordings" / session / filename).resolve()
        if self.dataset not in path.parents or not path.is_file():
            raise ValueError("Missing source file or path outside raw dataset.")
        return path

    def empty_plan(self):
        cameras = {c["id"]: c["id"] for session in self.sessions.values() for c in session["cameras"]}
        return {"version": 2, "dataset": str(self.dataset), "recording_instructions": self.instructions.copy(),
                "camera_names": cameras, "episodes": []}

    def validate(self, plan, *, exporting=False):
        if not isinstance(plan, dict) or plan.get("version") != 2 or plan.get("dataset") != str(self.dataset):
            raise ValueError("Plan version or raw dataset does not match this workspace.")
        for key in ("initialized_sessions", "auto_split_sessions"):
            initialized = plan.get(key, [])
            if (not isinstance(initialized, list)
                    or any(not isinstance(name, str) or name not in self.sessions for name in initialized)
                    or len(set(initialized)) != len(initialized)):
                raise ValueError(f"Invalid {key} recording list.")
        instructions = plan.get("recording_instructions", {})
        if (not isinstance(instructions, dict)
                or any(name not in self.sessions or not isinstance(text, str) for name, text in instructions.items())):
            raise ValueError("Invalid recording instructions.")
        references = plan.get("reset_references", {})
        if (not isinstance(references, dict) or any(
                name not in self.sessions or value is not None and (type(value) is not int or not 0 <= value < self.sessions[name]["duration_ns"])
                for name, value in references.items())):
            raise ValueError("Invalid reset reference.")
        mapping = plan.get("camera_names")
        if not isinstance(mapping, dict) or set(mapping) != set(self.empty_plan()["camera_names"]):
            raise ValueError("Name every source camera.")
        for value in mapping.values():
            identifier(value, "camera name", CAMERA_ID)
        for session in self.sessions.values():
            names = [mapping[c["id"]] for c in session["cameras"]]
            if len(set(names)) != len(names):
                raise ValueError("Camera names must be unique within each recording.")
        episodes, ids = plan.get("episodes"), set()
        if not isinstance(episodes, list) or exporting and not episodes:
            raise ValueError("Add at least one episode before exporting.")
        for item in episodes:
            if not isinstance(item, dict):
                raise ValueError("Invalid episode entry.")
            name = identifier(item.get("id"), "episode ID")
            if name in ids:
                raise ValueError("Episode IDs must be unique.")
            ids.add(name)
            session = self.sessions.get(item.get("session"))
            if session is None:
                raise ValueError("Unknown raw session.")
            start, end = item.get("start_ns"), item.get("end_ns")
            if type(start) is not int or type(end) is not int or not 0 <= start < end <= session["duration_ns"]:
                raise ValueError("Episode range must lie inside its raw session.")
            text = self.instruction(plan, item)
            if not isinstance(text, str) or exporting and (not text.strip() or text.strip() == "__pending__"):
                raise ValueError("Set an instruction for every episode before exporting.")
            if exporting:
                first, stop = np.searchsorted(self.times[session["id"]], [start, end])
                if stop - first < 2:
                    raise ValueError(f"{name}: select at least two synchronized frames.")
                if any(mapping[c["id"]] not in contract.CAMERA_IDS for c in session["cameras"]):
                    raise ValueError("Name exported cameras top, left_wrist, or right_wrist.")
        return plan

    @staticmethod
    def instruction(plan, item):
        return plan.get("recording_instructions", {}).get(item["session"], item.get("instruction"))

    def suggest(self, session, *, reference_ns=None, **settings):
        if session not in self.sessions:
            raise ValueError("Unknown recording.")
        if session not in self.analysis_cache:
            duration = self.sessions[session]["duration_ns"]
            grid = np.arange(0, duration, STEP_NS, dtype=np.int64)
            indices = np.searchsorted(self.times[session], grid, side="right") - 1
            table = pq.read_table(self.path(session, contract.FRAMES), columns=[contract.STATE_KEY])
            states = table.column(0).combine_chunks().flatten().to_numpy().reshape(-1, contract.STATE_DIM)
            poses = {side: states[indices, contract.STATE_SLICES[f"{side}_eef"]] for side in contract.SIDES}
            self.analysis_cache = {session: (grid, poses)}
        grid, poses = self.analysis_cache[session]
        return suggest_ranges(grid, poses, np.ones(len(grid), dtype=bool), self.sessions[session]["duration_ns"],
                              reference_ns=reference_ns, **settings)

    def camera_path(self, session, camera):
        if session not in self.sessions or camera not in [view["id"] for view in self.sessions[session]["cameras"]]:
            raise ValueError("Unknown camera or session.")
        return self.path(session, camera + ".mp4")

    def load_plan(self):
        return self.validate(json.loads(self.plan_path.read_text())) if self.plan_path.exists() else self.empty_plan()

    def frame(self, session, camera, timestamp):
        import cv2
        path = self.camera_path(session, camera)
        index = bisect_right(self.times[session], timestamp) - 1
        if index < 0:
            raise ValueError("Camera has not started at this time.")
        with self.capture_lock:
            key = session, camera
            if key not in self.captures:
                self.captures[key] = cv2.VideoCapture(str(path))
            capture = self.captures[key]
            if int(capture.get(cv2.CAP_PROP_POS_FRAMES)) != index:
                capture.set(cv2.CAP_PROP_POS_FRAMES, index)
            ok, image = capture.read()
            if not ok:
                raise ValueError("Could not decode the requested source frame.")
            if image.shape[1] > 960:
                image = cv2.resize(image, (960, round(image.shape[0] * 960 / image.shape[1])))
            ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if not ok:
                raise ValueError("Could not encode preview.")
            return encoded.tobytes()

    def export(self, plan, progress=lambda value: None):
        self.output.parent.mkdir(parents=True, exist_ok=True)
        with (self.output.parent / ("." + self.output.name + ".processing.lock")).open("a") as lock:
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as exc:
                raise ValueError("Another process is exporting to this output dataset.") from exc
            return self._export(plan, progress)

    def _export(self, plan, progress):
        plan = self.validate(plan, exporting=True)
        if self.output.exists():
            raise ValueError("Output already exists. Choose a new output dataset to avoid overwriting work.")
        self.output.parent.mkdir(parents=True, exist_ok=True)
        stage = Path(tempfile.mkdtemp(prefix=f".{self.output.name}-", dir=self.output.parent))
        try:
            layouts = {}
            episodes = plan["episodes"]
            with tempfile.TemporaryDirectory(dir=stage) as clips:
                for number, item in enumerate(episodes):
                    progress({"state": "running", "completed": number, "total": len(episodes), "episode": item["id"]})
                    source = item["session"]
                    first, stop = np.searchsorted(self.times[source], [item["start_ns"], item["end_ns"]])
                    cameras = {plan["camera_names"][c["id"]]: self.camera_path(source, c["id"])
                               for c in self.sessions[source]["cameras"]}
                    directory = Path(clips) / item["id"]
                    record = stage_clip(directory, self.path(source, contract.FRAMES), cameras,
                                        int(first), int(stop), self.instruction(plan, item).strip())
                    layout = contract.layout_name(record["cameras"], record["sizes"])
                    layouts.setdefault(layout, []).append((record, directory))
                for layout, members in layouts.items():
                    contract.build_dataset(stage / layout, "rebot", members)
            write_json(stage / "processing.json", plan)
            # Never publish over an existing destination, including one created during export.
            if self.output.exists():
                raise ValueError("Output appeared during export; refusing to replace it.")
            stage.rename(self.output)
            return {"state": "done", "completed": len(episodes), "total": len(episodes), "output": str(self.output)}
        finally:
            if stage.exists():
                shutil.rmtree(stage)

    def start_export(self, plan):
        self.validate(plan, exporting=True)
        with self.guard:
            if self.job["state"] == "running":
                raise ValueError("An export is already running.")
            write_json(self.plan_path, plan)
            self.job = {"state": "running", "completed": 0, "total": len(plan["episodes"])}
        def run():
            try:
                self.job = self.export(plan, lambda value: setattr(self, "job", value))
            except Exception as exc:
                self.job = {"state": "error", "error": str(exc)}
        threading.Thread(target=run, name="episode-export", daemon=False).start()


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *_):
        pass

    def respond(self, status, value, mime="application/json"):
        data = json.dumps(value).encode() if mime == "application/json" else value
        self.send_response(status)
        self.send_header("Content-Type", mime)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Content-Security-Policy", "default-src 'self'; img-src 'self' blob:; style-src 'self' 'unsafe-inline'; frame-ancestors 'none'")
        self.end_headers()
        self.wfile.write(data)

    def video(self, query, *, head=False):
        workspace = self.server.workspace
        session, camera = query["session"][0], query["camera"][0]
        path = workspace.camera_path(session, camera)
        if query.get("preview") == ["1"]:
            path = workspace.previews.path(path)
            if not path.is_file():
                raise ValueError("Preview is not ready.")
        with path.open("rb") as stream:
            size = os.fstat(stream.fileno()).st_size
            start, end, status = 0, size - 1, 200
            requested = self.headers.get("Range")
            if requested is not None:
                match = re.fullmatch(r"bytes=(\d*)-(\d*)", requested)
                if match and any(match.groups()):
                    first, last = match.groups()
                    if first:
                        start = int(first)
                        end = min(int(last), size - 1) if last else size - 1
                    else:
                        start = max(0, size - int(last))
                    status = 206
                if not match or not any(match.groups()) or not 0 <= start <= end < size:
                    self.send_response(416)
                    self.send_header("Content-Range", f"bytes */{size}")
                    self.send_header("Content-Length", "0")
                    self.end_headers()
                    return
            self.send_response(status)
            self.send_header("Content-Type", "video/mp4")
            self.send_header("Accept-Ranges", "bytes")
            self.send_header("Content-Length", str(end - start + 1))
            self.send_header("Cache-Control", "private, max-age=3600")
            self.send_header("X-Content-Type-Options", "nosniff")
            if status == 206:
                self.send_header("Content-Range", f"bytes {start}-{end}/{size}")
            self.end_headers()
            if head:
                return
            stream.seek(start)
            remaining = end - start + 1
            while remaining:
                chunk = stream.read(min(256 * 1024, remaining))
                if not chunk:
                    break
                self.wfile.write(chunk)
                remaining -= len(chunk)

    def do_HEAD(self):
        url = urlsplit(self.path)
        try:
            if url.path == "/api/video":
                self.video(parse_qs(url.query), head=True)
            else:
                self.send_error(404)
        except (ValueError, KeyError, OSError):
            self.send_error(400)

    def do_GET(self):
        workspace = self.server.workspace
        url = urlsplit(self.path)
        try:
            if url.path == "/api/workspace":
                self.respond(200, {"sessions": list(workspace.sessions.values()), "plan": workspace.load_plan(),
                                   "output": str(workspace.output), "plan_path": str(workspace.plan_path), "reset_defaults": DEFAULTS})
            elif url.path == "/api/job":
                self.respond(200, workspace.job)
            elif url.path == "/api/suggest":
                query = parse_qs(url.query)
                reference = query.get("reference_ns", [None])[0]
                self.respond(200, workspace.suggest(query["session"][0], reference_ns=int(reference) if reference is not None else None))
            elif url.path == "/api/video":
                self.video(parse_qs(url.query))
            elif url.path == "/api/preview":
                query = parse_qs(url.query)
                self.respond(200, workspace.preview(query["session"][0]))
            elif url.path in ("/api/frame", "/api/times"):
                query = parse_qs(url.query)
                session, camera = query["session"][0], query["camera"][0]
                if url.path == "/api/frame":
                    self.respond(200, workspace.frame(session, camera, int(query["t"][0])), "image/jpeg")
                else:
                    workspace.camera_path(session, camera)
                    self.respond(200, workspace.times[session].tolist())
            elif url.path in ("/", "/app.js", "/playback.js", "/style.css"):
                filename, mime = {"/": ("index.html", "text/html; charset=utf-8"),
                                  "/app.js": ("app.js", "text/javascript; charset=utf-8"),
                                  "/playback.js": ("playback.js", "text/javascript; charset=utf-8"),
                                  "/style.css": ("style.css", "text/css; charset=utf-8")}[url.path]
                self.respond(200, (ASSETS / filename).read_bytes(), mime)
            else:
                self.respond(404, {"error": "Not found"})
        except (BrokenPipeError, ConnectionResetError):
            pass
        except (ValueError, KeyError, OSError) as exc:
            self.respond(400, {"error": str(exc)})

    def do_POST(self):
        try:
            if self.headers.get("Origin") not in self.server.origins:
                self.respond(403, {"error": "Invalid origin"})
                return
            length = int(self.headers.get("Content-Length", "0"))
            if self.headers.get_content_type() != "application/json" or not 0 < length <= 4_000_000:
                raise ValueError("Expected a JSON plan under 4 MB.")
            plan = json.loads(self.rfile.read(length))
            if self.path == "/api/plan":
                write_json(self.server.workspace.plan_path, self.server.workspace.validate(plan))
                self.respond(200, {"saved": True})
            elif self.path == "/api/export":
                self.server.workspace.start_export(plan)
                self.respond(202, self.server.workspace.job)
            else:
                self.respond(404, {"error": "Not found"})
        except (ValueError, KeyError, TypeError, OSError) as exc:
            self.respond(400, {"error": str(exc)})


def serve(workspace, port=8090):
    """Run the loopback review server until interrupted."""
    server = ThreadingHTTPServer(("127.0.0.1", port), Handler)
    server.daemon_threads = True
    server.workspace = workspace
    server.origins = {f"http://127.0.0.1:{server.server_port}", f"http://localhost:{server.server_port}"}
    print(f"Episode Studio: http://127.0.0.1:{server.server_port}", flush=True)
    print(f"Draft: {workspace.plan_path}\nOutput: {workspace.output}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        workspace.previews.close()
        for capture in workspace.captures.values():
            capture.release()
