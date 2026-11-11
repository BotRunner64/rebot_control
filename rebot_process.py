#!/usr/bin/env python3
"""Review continuous recordings and export annotated episodes without changing raw data."""

import argparse
from bisect import bisect_right
import fcntl
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
import threading
from urllib.parse import parse_qs, urlsplit

import numpy as np
import pyarrow as pa
import pyarrow.compute as pc
import pyarrow.parquet as pq

from rebot_reset_detection import DEFAULTS, MAX_AGE_NS, STEP_NS, suggest_ranges


ASSETS = Path(__file__).with_name("web") / "process"
VIDEO_SCRIPT = Path(__file__).with_name("rebot_process_video.py")
STATE_NAMES = [f"{side}_{kind}" for side in ("left", "right") for kind in ("eef", "gripper")]


def identifier(value, camera=False):
    pattern = r"[a-z0-9_]+" if camera else r"[a-zA-Z0-9_][a-zA-Z0-9_-]*"
    if not isinstance(value, str) or re.fullmatch(pattern, value) is None:
        raise ValueError("Invalid camera name." if camera else "Invalid episode ID.")
    return value


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
        self.records = [json.loads(line) for line in (self.dataset / "episodes.jsonl").read_text().splitlines() if line.strip()]
        self.sessions, self.streams = {}, {}
        for record in self.records:
            name = identifier(record["episode_id"])
            if name in self.sessions:
                raise ValueError("Duplicate raw session ID.")
            cameras = record["cameras"]
            if not cameras or len(set(cameras)) != len(cameras):
                raise ValueError("Raw sessions require distinct cameras.")
            ends, views = [], []
            for camera in cameras:
                identifier(camera, camera=True)
            for group, names in (("state", STATE_NAMES), ("rgb", cameras)):
                for stream in names:
                    path = self.path(name, group, stream + ".parquet")
                    times = pq.read_table(path, columns=["timestamp_ns"])["timestamp_ns"].to_numpy()
                    if not len(times) or np.any(times < 0) or np.any(times[1:] < times[:-1]):
                        raise ValueError(f"Empty or unordered stream: {path}")
                    if int(times[-1]) >= 2**53 - 1:
                        raise ValueError("Recording is too long for exact browser nanosecond editing.")
                    self.streams[name, group, stream] = times
                    ends.append(int(times[-1]))
                    if group == "rgb":
                        frames = pq.read_table(path, columns=["frame_index"])["frame_index"].to_numpy()
                        if not np.array_equal(frames, np.arange(len(times))):
                            raise ValueError(f"Invalid source frame index: {path}")
                        self.path(name, "rgb", stream + ".mp4")
                        views.append({"id": stream, "frames": len(times),
                                      "first_ns": int(times[0]), "last_ns": int(times[-1])})
            self.sessions[name] = {"id": name, "duration_ns": max(ends) + 1, "cameras": views}
        if not self.sessions:
            raise ValueError("No finalized raw recordings found.")
        self.guard = threading.Lock()
        self.job = {"state": "idle"}
        self.captures, self.capture_lock = {}, threading.Lock()
        self.analysis_cache = {}

    def path(self, session, group, filename):
        path = (self.dataset / "episodes" / session / group / filename).resolve()
        if self.dataset not in path.parents or not path.is_file():
            raise ValueError("Missing source file or path outside raw dataset.")
        return path

    def empty_plan(self):
        instructions = {}
        for record in self.records:
            texts = {item.get("text", "").strip() for item in record.get("instructions", [])}
            texts.discard("")
            texts.discard("__pending__")
            if len(texts) == 1:
                instructions[record["episode_id"]] = next(iter(texts))
        return {"version": 1, "dataset": str(self.dataset), "recording_instructions": instructions, "camera_names": {
            name: name for session in self.sessions.values() for name in [c["id"] for c in session["cameras"]]}, "episodes": []}

    def validate(self, plan, *, exporting=False):
        if not isinstance(plan, dict) or plan.get("version") != 1 or plan.get("dataset") != str(self.dataset):
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
            identifier(value, camera=True)
        if len(set(mapping.values())) != len(mapping):
            raise ValueError("Camera names must be unique.")
        episodes, ids = plan.get("episodes"), set()
        if not isinstance(episodes, list) or exporting and not episodes:
            raise ValueError("Add at least one episode before exporting.")
        for item in episodes:
            if not isinstance(item, dict):
                raise ValueError("Invalid episode entry.")
            name = identifier(item.get("id"))
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
                for (sid, group, stream), times in self.streams.items():
                    if sid == session["id"] and not np.any((times >= start) & (times < end)):
                        raise ValueError(f"{name}: range contains no samples in {stream}.")
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
            poses, valid = {}, np.ones(len(grid), dtype=bool)
            for side in ("left", "right"):
                times = self.streams[session, "state", f"{side}_eef"]
                indices = np.searchsorted(times, grid, side="right") - 1
                safe = np.maximum(0, indices)
                valid &= (indices >= 0) & (grid - times[safe] <= MAX_AGE_NS)
                sampled, offset = np.empty((len(grid), 12)), 0
                path = self.path(session, "state", f"{side}_eef.parquet")
                for batch in pq.ParquetFile(path).iter_batches(batch_size=8192, columns=["pose"]):
                    stop = offset + batch.num_rows
                    a, b = np.searchsorted(safe, [offset, stop])
                    if b > a:
                        values = np.asarray(batch.column(0).to_pylist(), dtype=float)
                        sampled[a:b] = values[safe[a:b] - offset]
                    offset = stop
                poses[side] = sampled
            # Cache just the most recently analyzed recording.
            self.analysis_cache = {session: (grid, poses, valid)}
        grid, poses, valid = self.analysis_cache[session]
        result = suggest_ranges(grid, poses, valid, self.sessions[session]["duration_ns"],
                                reference_ns=reference_ns, **settings)
        # Every proposed episode must be exportable across all native streams.
        original_count = len(result["episodes"])
        result["episodes"] = [item for item in result["episodes"] if all(
            np.searchsorted(times, item["start_ns"], side="left") < np.searchsorted(times, item["end_ns"], side="left")
            for (sid, _, _), times in self.streams.items() if sid == session)]
        if original_count and not result["episodes"]:
            result["usable"] = False
            result["message"] = "Required streams do not overlap in the proposed episodes. Cut manually."
        return result

    def load_plan(self):
        return self.validate(json.loads(self.plan_path.read_text())) if self.plan_path.exists() else self.empty_plan()

    def frame(self, session, camera, timestamp):
        import cv2
        times = self.streams.get((session, "rgb", camera))
        if times is None:
            raise ValueError("Unknown camera or session.")
        index = bisect_right(times, timestamp) - 1
        if index < 0:
            raise ValueError("Camera has not started at this time.")
        with self.capture_lock:
            key = session, camera
            if key not in self.captures:
                self.captures[key] = cv2.VideoCapture(str(self.path(session, "rgb", camera + ".mp4")))
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
            records = []
            for number, item in enumerate(plan["episodes"]):
                progress({"state": "running", "completed": number, "total": len(plan["episodes"]), "episode": item["id"]})
                source, start, end = item["session"], item["start_ns"], item["end_ns"]
                destination = stage / "episodes" / item["id"]
                latest = 0
                for group in ("state", "rgb"):
                    (destination / group).mkdir(parents=True)
                    names = STATE_NAMES if group == "state" else [c["id"] for c in self.sessions[source]["cameras"]]
                    for name in names:
                        times = self.streams[source, group, name]
                        first, stop = np.searchsorted(times, [start, end], side="left")
                        target = plan["camera_names"][name] if group == "rgb" else name
                        # Read only overlapping row groups; keep memory bounded on long sessions.
                        table = pq.read_table(self.path(source, group, name + ".parquet"),
                                              filters=[("timestamp_ns", ">=", start), ("timestamp_ns", "<", end)])
                        column = table.schema.get_field_index("timestamp_ns")
                        table = table.set_column(column, table.schema.field(column), pc.subtract(table["timestamp_ns"], start))
                        latest = max(latest, int(times[stop - 1]) - start)
                        if group == "rgb":
                            column = table.schema.get_field_index("frame_index")
                            table = table.set_column(column, table.schema.field(column), pa.array(range(stop - first), type=pa.int64()))
                            request = {"source": str(self.path(source, group, name + ".mp4")),
                                       "output": str(destination / group / (target + ".mp4")),
                                       "first_frame": int(first), "timestamps_ns": times[first:stop].tolist()}
                            result = subprocess.run(["/usr/bin/python3", str(VIDEO_SCRIPT)], input=json.dumps(request),
                                                    text=True, capture_output=True)
                            if result.returncode:
                                raise RuntimeError(f"Video export failed for {name}: {result.stderr[-2000:]}")
                        pq.write_table(table, destination / group / (target + ".parquet"), compression="zstd")
                if latest <= 0:
                    raise ValueError("Selected episode must have a positive sampled duration.")
                records.append({"episode_id": item["id"], "cameras": [plan["camera_names"][c["id"]] for c in self.sessions[source]["cameras"]],
                                "instructions": [{"start_ns": 0, "end_ns": latest, "text": self.instruction(plan, item).strip()}]})
            (stage / "episodes.jsonl").write_text("".join(json.dumps(record, ensure_ascii=False) + "\n" for record in records))
            write_json(stage / "processing.json", plan)
            # Never publish over an existing destination, including one created during export.
            if self.output.exists():
                raise ValueError("Output appeared during export; refusing to replace it.")
            stage.rename(self.output)
            return {"state": "done", "completed": len(records), "total": len(records), "output": str(self.output)}
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
            elif url.path in ("/api/frame", "/api/times"):
                query = parse_qs(url.query)
                session, camera = query["session"][0], query["camera"][0]
                if url.path == "/api/frame":
                    self.respond(200, workspace.frame(session, camera, int(query["t"][0])), "image/jpeg")
                else:
                    self.respond(200, workspace.streams[session, "rgb", camera].tolist())
            elif url.path in ("/", "/app.js", "/style.css"):
                filename, mime = {"/": ("index.html", "text/html; charset=utf-8"),
                                  "/app.js": ("app.js", "text/javascript; charset=utf-8"),
                                  "/style.css": ("style.css", "text/css; charset=utf-8")}[url.path]
                self.respond(200, (ASSETS / filename).read_bytes(), mime)
            else:
                self.respond(404, {"error": "Not found"})
        except (ValueError, KeyError, OSError) as exc:
            self.respond(400, {"error": str(exc)})
        except (BrokenPipeError, ConnectionResetError):
            pass

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


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, required=True, help="Finalized continuous recordings")
    parser.add_argument("--output", type=Path, required=True, help="New processed dataset directory")
    parser.add_argument("--plan", type=Path, help="Editable JSON plan outside raw dataset")
    parser.add_argument("--export", action="store_true", help="Export the saved plan without opening a server")
    parser.add_argument("--port", type=int, default=8090)
    args = parser.parse_args(argv)
    try:
        plan = args.plan or args.output.with_name(args.output.name + ".plan.json")
        workspace = Workspace(args.dataset, args.output, plan)
        if args.export:
            print(json.dumps(workspace.export(workspace.load_plan()), indent=2))
            return 0
        server = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
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
            for capture in workspace.captures.values():
                capture.release()
        return 0
    except (OSError, ValueError, KeyError, RuntimeError) as exc:
        print(str(exc), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
