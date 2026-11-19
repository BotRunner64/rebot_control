"""Record measured feedback and finalize a synchronized 30 fps recording."""

from contextlib import ExitStack
from dataclasses import dataclass
import fcntl
import json
import math
import multiprocessing as mp
import os
from pathlib import Path
import queue
import re
import signal
import socket
import shutil
import subprocess
import sys
import time
import uuid
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

from rebot.media.record_preview import RecordingViewer
from rebot.paths import MODELS, system_python


MODEL_PATH = MODELS / "rebot_rs.xml"
SIDES = ("left", "right")
STATE_NAMES = tuple(f"{side}_{kind}" for side in SIDES for kind in ("eef", "gripper"))
# Naming rules of the dataset format, shared by recording, processing, and export.
EPISODE_ID = r"[a-zA-Z0-9_][a-zA-Z0-9_-]*"
CAMERA_ID = r"[a-z0-9_]+"
BATCH_SIZE = 512


def identifier(value, label, pattern=EPISODE_ID):
    if not isinstance(value, str) or re.fullmatch(pattern, value) is None:
        raise ValueError(f"Invalid {label}: {value!r}.")
    return value


@dataclass(frozen=True)
class RecordingOptions:
    dataset: Path
    instruction: str
    cameras: dict
    episode_id: str | None = None
    gripper_open_deg: float = 270.
    camera_viewer: bool = False

    def validate(self):
        if not isinstance(self.instruction, str) or not self.instruction.strip():
            raise ValueError("Recording requires a nonempty task instruction.")
        self.instruction.encode("utf-8")
        if not 1 <= len(self.cameras) <= 3:
            raise ValueError("Recording requires one to three RGB cameras.")
        devices = set()
        for name, device in self.cameras.items():
            identifier(name, "camera ID", CAMERA_ID)
            path = Path(device).resolve()
            if path in devices:
                raise ValueError("Camera devices must be distinct.")
            devices.add(path)
        if self.episode_id is not None:
            identifier(self.episode_id, "episode ID")
        if not math.isfinite(self.gripper_open_deg) or not 0 < self.gripper_open_deg <= 270:
            raise ValueError("Recorded physical gripper opening must be in (0, 270] degrees.")


class ForwardKinematics:
    """Read the model's contract EEF directly, without rendering or an axis adapter."""

    def __init__(self, path=MODEL_PATH):
        root = ET.parse(path).getroot()
        parents = {child: parent for parent in root.iter() for child in parent}
        node = root.find(".//body[@name='gripper_mount']")
        if node is None:
            raise ValueError("The model must declare the fixed gripper mounting frame.")
        chain = []
        while node.tag == "body":
            chain.append(node)
            node = parents[node]
        self.chain = []
        joints = []
        for body in reversed(chain):
            pose = np.eye(4)
            pose[:3, 3] = np.fromstring(body.get("pos", "0 0 0"), sep=" ")
            w, x, y, z = np.fromstring(body.get("quat", "1 0 0 0"), sep=" ")
            pose[:3, :3] = Rotation.from_quat([x, y, z, w]).as_matrix()
            joint = body.find("joint")
            axis = origin = None
            if joint is not None:
                if joint.get("type", "hinge") != "hinge":
                    raise ValueError("Recording FK requires hinge arm joints.")
                joints.append(joint.get("name"))
                axis = np.fromstring(joint.get("axis", "0 0 1"), sep=" ")
                axis /= np.linalg.norm(axis)
                origin = np.fromstring(joint.get("pos", "0 0 0"), sep=" ")
            self.chain.append((pose, axis, origin))
        if joints != [f"joint{i}" for i in range(1, 7)]:
            raise ValueError("Expected the six ordered RS arm joints before gripper_mount.")

    def pose(self, q):
        q = np.asarray(q, dtype=float)
        if q.shape != (6,) or not np.all(np.isfinite(q)):
            raise ValueError("FK requires six finite measured joint angles.")
        result, index = np.eye(4), 0
        for fixed, axis, origin in self.chain:
            result = result @ fixed
            if axis is not None:
                motion = np.eye(4)
                motion[:3, :3] = Rotation.from_rotvec(axis * q[index]).as_matrix()
                motion[:3, 3] = origin - motion[:3, :3] @ origin
                result = result @ motion
                index += 1
        return np.concatenate((result[:3, 3], result[:3, :3].ravel(order="F")))


def schemas():
    import pyarrow as pa

    timestamp = pa.field("timestamp_ns", pa.int64(), nullable=False)
    return {
        "eef": pa.schema([timestamp, pa.field("pose", pa.list_(pa.float64(), 12), nullable=False)]),
        "gripper": pa.schema([timestamp, pa.field("openness", pa.float32(), nullable=False)]),
    }


class TableWriter:
    def __init__(self, path, schema):
        import pyarrow.parquet as pq

        self.schema, self.rows, self.last = schema, [], None
        self.writer = pq.ParquetWriter(path, schema, compression="zstd")

    def append(self, timestamp, value):
        if type(timestamp) is not int or not 0 <= timestamp < 2**63:
            raise ValueError("Sample timestamps must be nonnegative int64 nanoseconds.")
        if self.last is not None and timestamp < self.last:
            raise ValueError("Sample timestamps must be nondecreasing within each stream.")
        self.last = timestamp
        self.rows.append((timestamp, value))
        if len(self.rows) >= BATCH_SIZE:
            self.flush()

    def flush(self):
        import pyarrow as pa

        if self.rows:
            arrays = [pa.array(values, type=field.type)
                      for values, field in zip(zip(*self.rows), self.schema)]
            self.writer.write_table(pa.Table.from_arrays(arrays, schema=self.schema))
            self.rows.clear()

    def close(self):
        try:
            self.flush()
        finally:
            self.writer.close()


def state_writer(directory, samples, status, references, open_deg):
    """Own FK and Parquet I/O in a separate spawned process."""
    # Terminal interrupts go to the whole process group. Let the supervisor
    # stop motors first, then drain this queue through its explicit sentinel.
    for sig in (signal.SIGINT, signal.SIGHUP):
        signal.signal(sig, signal.SIG_IGN)
    try:
        fk, schema = ForwardKinematics(), schemas()
        with ExitStack() as stack:
            writers = {}
            for name in STATE_NAMES:
                side, kind = name.split("_")
                writer = TableWriter(Path(directory) / "state" / f"{name}.parquet", schema[kind])
                stack.callback(writer.close)
                writers[side, kind] = writer
            status.send({"type": "ready"})
            parent = mp.parent_process()
            while True:
                try:
                    sample = samples.get(timeout=.2)
                except queue.Empty:
                    if parent is not None and not parent.is_alive():
                        raise RuntimeError("Recording supervisor exited.")
                    continue
                if sample is None:
                    break
                side, arm_ns, raw, gripper_ns = sample
                relative = np.asarray(raw, dtype=float) - references[side]
                if relative.shape != (7,) or not np.all(np.isfinite(relative)):
                    raise ValueError("Recording requires finite actual feedback for all seven motors.")
                writers[side, "eef"].append(arm_ns, fk.pose(relative[:6]))
                openness = float(np.clip(relative[6] / math.radians(open_deg), 0., 1.))
                writers[side, "gripper"].append(gripper_ns, openness)
        status.send({"type": "done"})
    except BaseException as exc:
        try:
            status.send({"type": "error", "error": f"{type(exc).__name__}: {exc}"})
        except (BrokenPipeError, OSError):
            pass
    finally:
        status.close()


def publish_feedback(samples, side, states):
    """Enqueue a complete measured batch without blocking the motor control loop."""
    times = [state["received_at_ns"] for state in states]
    sample = (side, max(times[:6]), [state["position"] for state in states], times[6])
    try:
        samples.put_nowait(sample)
    except queue.Full as exc:
        raise RuntimeError("Recording queue overflow; stopping instead of silently dropping feedback.") from exc


def discover_cameras():
    """List capture devices through the system GStreamer environment."""
    command, env = system_python("rebot.media.recorder", "--discover")
    result = subprocess.run(command, env=env, capture_output=True, text=True, check=True, timeout=15.)
    return json.loads(result.stdout)["sources"]


class CameraProcess:
    """One recorder child. With `jpeg_fd` it records frames relayed on that socket, not the device."""

    def __init__(self, source, directory, name, *, test=False, preview_socket=None, jpeg_fd=None):
        self.log = (Path(directory) / f"{name}.log").open("wb")
        self.process = None
        self.buffer, self.ready, self.done = b"", False, False
        arguments = ["--source", str(source), "--output", str(Path(directory) / f"{name}.mp4"),
                     "--timestamps", str(Path(directory) / f"{name}.timestamps")]
        if test:
            arguments.append("--test-source")
        if preview_socket is not None:
            arguments.extend(["--preview-socket", str(preview_socket)])
        if jpeg_fd is not None:
            arguments.extend(["--jpeg-fd", str(jpeg_fd)])
        command, env = system_python("rebot.media.recorder", *arguments)
        try:
            self.process = subprocess.Popen(command, env=env, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                            stderr=self.log, pass_fds=() if jpeg_fd is None else (jpeg_fd,))
            os.set_blocking(self.process.stdout.fileno(), False)
        except BaseException:
            self.log.close()
            raise

    def check(self, *, finishing=False):
        while chunk := self.process.stdout.read():
            self.buffer += chunk
        while b"\n" in self.buffer:
            line, self.buffer = self.buffer.split(b"\n", 1)
            record = json.loads(line)
            if record["type"] == "error":
                raise RuntimeError(f"Camera recording failed: {record['error']}")
            if record["type"] == "ready":
                self.ready = True
            if record["type"] == "done":
                self.done = True
        code = self.process.poll()
        if code is not None and (code != 0 or not finishing):
            raise RuntimeError(f"Camera exited unexpectedly ({code}); see {self.log.name}.")

    def send(self, command, timestamp):
        self.process.stdin.write((json.dumps({"command": command, "timestamp_ns": timestamp}) + "\n").encode())
        self.process.stdin.flush()

    def close(self):
        try:
            if self.process is not None:
                if self.process.poll() is None:
                    self.process.terminate()
                    try:
                        self.process.wait(2.)
                    except subprocess.TimeoutExpired:
                        self.process.kill()
                        self.process.wait()
                self.process.stdin.close()
                self.process.stdout.close()
        finally:
            self.log.close()


class RecordingSession:
    """Stage a recording outside recordings/ and publish only after successful finalization.

    `references` maps each side to its seven calibrated zero positions in radians.
    With `relay`, no camera device is opened: the caller already captures the cameras and
    sends every frame to `feeds[name]` as a length-prefixed `!d seconds + JPEG` packet.
    With `debug`, the episode also gets a `debug/` directory (see `rebot.data.debug`); the
    caller fills it through `publish(..., command)` and `self.debug`.
    """

    def __init__(self, options, references, *, camera_factory=CameraProcess, relay=False, debug=False):
        options.validate()
        from rebot.data import synchronous  # Check recording dependencies before hardware startup.
        self.options, self.cameras, self.feeds = options, [], {}
        self.viewer = self.debug = None
        self.process = self.status = self.samples = self.lock = None
        self.writer_ready = self.writer_done = self.started = self.closed = False
        self.cameras_stopped = False
        self.root = Path(options.dataset).resolve()
        self.episode_id = options.episode_id or f"episode_{uuid.uuid4().hex}"
        self.directory = self.root / ".pending" / self.episode_id
        self.root.mkdir(parents=True, exist_ok=True)
        try:
            self.lock = (self.root / ".recording.lock").open("a")
            try:
                fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as exc:
                raise RuntimeError("Another recording session is using this dataset.") from exc
            if (self.root / "episodes.jsonl").exists():
                raise ValueError("Legacy recording directory. Choose a new recording directory.")
            if (self.root / "recordings" / self.episode_id).exists():
                raise ValueError(f"Recording already exists: {self.episode_id}.")
            self.directory.mkdir(parents=True)
            (self.directory / "state").mkdir()
            (self.directory / "rgb").mkdir()
            context = mp.get_context("spawn")
            self.samples = context.Queue(maxsize=4096)
            self.status, child_status = context.Pipe(duplex=False)
            references = {side: np.asarray(references[side], dtype=float) for side in SIDES}
            self.process = context.Process(name="record-state", target=state_writer,
                args=(self.directory, self.samples, child_status, references, options.gripper_open_deg))
            self.process.start()
            child_status.close()
            if debug:
                from rebot.data.debug import DebugLog

                self.debug = DebugLog(self.directory / "debug", references)
            if options.camera_viewer:
                try:
                    self.viewer = RecordingViewer(options.cameras)
                except Exception as exc:
                    print(f"Camera preview unavailable; continuing recording: {exc}", file=sys.stderr)
            for name, source in options.cameras.items():
                extra = {"preview_socket": self.viewer.paths[name]} if self.viewer is not None else {}
                child = None
                if relay:
                    self.feeds[name], child = socket.socketpair()
                    # A stalled recorder fails the sender within a second instead of blocking it.
                    self.feeds[name].settimeout(1.)
                    extra["jpeg_fd"] = child.fileno()
                try:
                    self.cameras.append(camera_factory(source, self.directory / "rgb", name, **extra))
                finally:
                    if child is not None:
                        child.close()
        except BaseException:
            self.close()
            raise

    def check(self, *, finishing=False):
        while self.status.poll():
            try:
                record = self.status.recv()
            except EOFError:
                break
            if record["type"] == "error":
                raise RuntimeError(f"State recording failed: {record['error']}")
            self.writer_ready |= record["type"] == "ready"
            self.writer_done |= record["type"] == "done"
        if self.process.exitcode is not None and (not finishing or not self.writer_done):
            raise RuntimeError(f"State recorder exited unexpectedly ({self.process.exitcode}).")
        if self.debug is not None:
            self.debug.check(finishing=finishing)
        for camera in self.cameras:
            camera.check(finishing=finishing)

    def ready(self):
        self.check()
        return (self.writer_ready and (self.debug is None or self.debug.ready)
                and all(camera.ready for camera in self.cameras))

    def publish(self, side, states, command=None):
        """Feedback hook for an in-process servo loop; ignored until the recording starts.

        `command` is the servo command behind these replies; it only feeds the debug streams.
        """
        if self.started:
            publish_feedback(self.samples, side, states)
            if self.debug is not None and command is not None:
                self.debug.joints(side, states, command)

    def start(self):
        timestamp = time.monotonic_ns()
        for camera in self.cameras:
            camera.send("start", timestamp)
        self.started = True
        print(f"Recording {self.episode_id} to {self.root}", flush=True)

    def stop_cameras(self):
        if self.started and not self.cameras_stopped:
            timestamp = time.monotonic_ns()
            for camera in self.cameras:
                camera.send("stop", timestamp)
            for feed in self.feeds.values():
                feed.shutdown(socket.SHUT_WR)
            self.cameras_stopped = True

    def finish(self, *, timeout=300.):
        if not self.started:
            raise RuntimeError("Recording never started; no episode was published.")
        deadline = time.monotonic() + timeout
        print("Finalizing episode and verifying RGB frame counts...", flush=True)
        self.stop_cameras()
        self.samples.put(None, timeout=1.)
        while True:
            self.check(finishing=True)
            if self.writer_done and all(camera.done and camera.process.poll() == 0 for camera in self.cameras):
                break
            if time.monotonic() >= deadline:
                raise TimeoutError("Episode finalization timed out; pending data was retained.")
            time.sleep(.01)
        self.process.join(2.)
        extra = [] if self.debug is None else self.debug.finish(deadline)
        from rebot.data.synchronous import synchronize

        synced = self.directory / "synced"
        metadata, origin = synchronize(self.directory, synced, self.options.cameras, self.options.instruction)
        if self.debug is not None:
            normalize_debug(extra, origin)
            (self.directory / "debug").rename(synced / "debug")
        destination = self.root / "recordings" / self.episode_id
        destination.parent.mkdir(exist_ok=True)
        synced.rename(destination)
        shutil.rmtree(self.directory)
        print(f"Saved episode {self.episode_id}: {destination}", flush=True)
        return metadata

    def close(self):
        if self.closed:
            return
        self.closed = True
        for camera in self.cameras:
            camera.close()
        for feed in self.feeds.values():
            feed.close()
        if self.viewer is not None:
            self.viewer.close()
        if self.debug is not None:
            self.debug.close()
        if self.process is not None and self.process.pid is not None:
            if self.process.is_alive():
                self.process.terminate()
            self.process.join(2.)
            if self.process.is_alive():
                self.process.kill()
                self.process.join()
        if self.samples is not None:
            self.samples.cancel_join_thread()
            self.samples.close()
        if self.status is not None:
            self.status.close()
        if self.lock is not None:
            self.lock.close()


def normalize_debug(paths, origin):
    """Put native-rate diagnostics on the clock of the first synchronized frame."""
    import pyarrow as pa
    import pyarrow.compute as pc
    import pyarrow.parquet as pq

    for path in paths:
        source = pq.ParquetFile(path)
        temporary = path.with_suffix(".normalized")
        with pq.ParquetWriter(temporary, source.schema_arrow, compression="zstd") as writer:
            for batch in source.iter_batches(batch_size=BATCH_SIZE):
                columns = list(batch.columns)
                for index, name in enumerate(batch.schema.names):
                    if name.endswith("timestamp_ns"):
                        columns[index] = pc.subtract(columns[index], pa.scalar(origin, type=pa.int64()))
                writer.write_batch(pa.RecordBatch.from_arrays(columns, schema=batch.schema))
        os.replace(temporary, path)


def parse_cameras(values):
    cameras = {}
    for index, value in enumerate(values or []):
        name, separator, source = value.partition("=")
        if not separator:
            name, source = f"camera_{index}", value
            if not source.isdecimal() and not Path(source).is_absolute():
                raise ValueError("Use --camera DEVICE or --camera ID=DEVICE.")
        if not source or name in cameras:
            raise ValueError("Camera names and devices must be unique and nonempty.")
        cameras[identifier(name, "camera ID", CAMERA_ID)] = f"/dev/video{source}" if source.isdecimal() else source
    return cameras
