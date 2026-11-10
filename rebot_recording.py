"""Asynchronous feedback recording and transactional episode export."""

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
import struct
import subprocess
import time
import uuid
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation


MODEL_PATH = Path(__file__).with_name("models") / "rebot_rs.xml"
CAMERA_SCRIPT = Path(__file__).with_name("rebot_record_camera.py")
SIDES = ("left", "right")
# Columns express dataset axes in gripper_mount: +X=+Z, +Y=-Y, +Z=+X.
MOUNT_FROM_EEF = np.array([[0., 0., 1.], [0., -1., 0.], [1., 0., 0.]])
BATCH_SIZE = 512


def identifier(value, label, pattern=r"[a-zA-Z0-9_][a-zA-Z0-9_-]*"):
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

    def validate(self):
        if not isinstance(self.instruction, str) or not self.instruction.strip():
            raise ValueError("Recording requires a nonempty task instruction.")
        self.instruction.encode("utf-8")
        if not self.cameras:
            raise ValueError("Recording requires at least one named RGB camera.")
        devices = set()
        for name, device in self.cameras.items():
            identifier(name, "camera ID", r"[a-z0-9_]+")
            path = Path(device).resolve()
            if path in devices:
                raise ValueError("Camera devices must be distinct.")
            devices.add(path)
        if self.episode_id is not None:
            identifier(self.episode_id, "episode ID")
        if not math.isfinite(self.gripper_open_deg) or not 0 < self.gripper_open_deg <= 270:
            raise ValueError("Recorded physical gripper opening must be in (0, 270] degrees.")


class ForwardKinematics:
    """Compute the mounting pose from the MJCF chain, without rendering or IK."""

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
        result[:3, :3] = result[:3, :3] @ MOUNT_FROM_EEF
        return np.concatenate((result[:3, 3], result[:3, :3].ravel(order="F")))


def schemas():
    import pyarrow as pa

    timestamp = pa.field("timestamp_ns", pa.int64(), nullable=False)
    return {
        "eef": pa.schema([timestamp, pa.field("pose", pa.list_(pa.float64(), 12), nullable=False)]),
        "gripper": pa.schema([timestamp, pa.field("openness", pa.float32(), nullable=False)]),
        "rgb": pa.schema([pa.field("frame_index", pa.int64(), nullable=False), timestamp]),
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
            for side in SIDES:
                for kind in ("eef", "gripper"):
                    writer = TableWriter(Path(directory) / "state" / f"{side}_{kind}.parquet", schema[kind])
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


class CameraProcess:
    def __init__(self, source, directory, name, *, python="/usr/bin/python3", test=False):
        self.log = (Path(directory) / f"{name}.log").open("wb")
        self.process = None
        self.buffer, self.ready, self.done = b"", False, False
        command = [python, str(CAMERA_SCRIPT), "--source", str(source),
                   "--output", str(Path(directory) / f"{name}.mp4"),
                   "--timestamps", str(Path(directory) / f"{name}.timestamps")]
        if test:
            command.append("--test-source")
        try:
            self.process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                            stderr=self.log)
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
    """Stage an episode outside episodes/ and publish only after successful finalization."""

    def __init__(self, options, arms, *, camera_factory=CameraProcess):
        options.validate()
        schemas()  # Fail before connecting to or enabling hardware.
        self.options, self.cameras = options, []
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
            self.records = self._read_metadata()
            if any(record["episode_id"] == self.episode_id for record in self.records):
                raise ValueError(f"Episode already exists: {self.episode_id}.")
            self.directory.mkdir(parents=True)
            (self.directory / "state").mkdir()
            (self.directory / "rgb").mkdir()
            context = mp.get_context("spawn")
            self.samples = context.Queue(maxsize=4096)
            self.status, child_status = context.Pipe(duplex=False)
            references = {side: np.asarray(arms[side]["reference_rad"]) for side in SIDES}
            self.process = context.Process(name="record-state", target=state_writer,
                args=(self.directory, self.samples, child_status, references, options.gripper_open_deg))
            self.process.start()
            child_status.close()
            for name, source in options.cameras.items():
                self.cameras.append(camera_factory(source, self.directory / "rgb", name))
        except BaseException:
            self.close()
            raise

    def _read_metadata(self):
        path = self.root / "episodes.jsonl"
        records = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()] if path.exists() else []
        ids = [record["episode_id"] for record in records]
        for episode_id in ids:
            identifier(episode_id, "existing episode ID")
        directories = {p.name for p in (self.root / "episodes").iterdir()} if (self.root / "episodes").exists() else set()
        if len(ids) != len(set(ids)) or set(ids) != directories:
            raise ValueError("Existing episodes.jsonl and episode directories are inconsistent.")
        return records

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
        for camera in self.cameras:
            camera.check(finishing=finishing)

    def ready(self):
        self.check()
        return self.writer_ready and all(camera.ready for camera in self.cameras)

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
        metadata = finalize_episode(self.directory, self.episode_id, list(self.options.cameras),
                                    self.options.instruction)
        destination = self.root / "episodes" / self.episode_id
        destination.parent.mkdir(exist_ok=True)
        metadata_path = self.root / ".pending" / f"{self.episode_id}.jsonl.tmp"
        try:
            with metadata_path.open("x", encoding="utf-8") as stream:
                for record in [*self.records, metadata]:
                    stream.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + "\n")
                stream.flush()
                os.fsync(stream.fileno())
            self.directory.rename(destination)
            try:
                os.replace(metadata_path, self.root / "episodes.jsonl")
            except BaseException:
                destination.rename(self.directory)
                raise
        finally:
            metadata_path.unlink(missing_ok=True)
        print(f"Saved episode {self.episode_id}: {destination}", flush=True)
        return metadata

    def close(self):
        if self.closed:
            return
        self.closed = True
        for camera in self.cameras:
            camera.close()
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


def finalize_episode(directory, episode_id, cameras, instruction):
    """Normalize all streams together, using bounded batches and integer subtraction."""
    import pyarrow as pa
    import pyarrow.compute as pc
    import pyarrow.parquet as pq

    directory = Path(directory)
    schema = schemas()
    paths, first, last = [], [], []
    for side in SIDES:
        for kind in ("eef", "gripper"):
            paths.append(directory / "state" / f"{side}_{kind}.parquet")
    for camera in cameras:
        raw_path = directory / "rgb" / f"{camera}.timestamps"
        video = directory / "rgb" / f"{camera}.mp4"
        if not video.is_file() or video.stat().st_size == 0:
            raise ValueError(f"Missing RGB video for {camera}.")
        if raw_path.stat().st_size == 0 or raw_path.stat().st_size % 8:
            raise ValueError(f"Invalid RGB timestamps for {camera}.")
        path = directory / "rgb" / f"{camera}.parquet"
        with pq.ParquetWriter(path, schema["rgb"], compression="zstd") as writer, raw_path.open("rb") as stream:
            index = 0
            while block := stream.read(8 * BATCH_SIZE):
                timestamps = [item[0] for item in struct.iter_unpack("<q", block)]
                writer.write_table(pa.Table.from_arrays(
                    [pa.array(range(index, index + len(timestamps)), type=pa.int64()),
                     pa.array(timestamps, type=pa.int64())], schema=schema["rgb"]))
                index += len(timestamps)
        paths.append(path)
    for path in paths:
        previous = None
        count = 0
        for batch in pq.ParquetFile(path).iter_batches(batch_size=BATCH_SIZE):
            values = batch.column("timestamp_ns").to_numpy()
            if len(values) == 0:
                continue
            if (np.any(values < 0) or np.any(values[1:] < values[:-1])
                    or previous is not None and values[0] < previous):
                raise ValueError(f"Invalid or unordered timestamps in {path.name}.")
            if count == 0:
                first.append(int(values[0]))
            previous = int(values[-1])
            count += len(values)
        if count == 0:
            raise ValueError(f"Required stream is empty: {path.name}.")
        last.append(previous)
    origin, end = min(first), max(last) - min(first)
    if end <= 0:
        raise ValueError("An episode must have a positive sampled duration.")
    for path in paths:
        source = pq.ParquetFile(path)
        temporary = path.with_suffix(".normalized")
        with pq.ParquetWriter(temporary, source.schema_arrow, compression="zstd") as writer:
            for batch in source.iter_batches(batch_size=BATCH_SIZE):
                index = batch.schema.get_field_index("timestamp_ns")
                columns = list(batch.columns)
                columns[index] = pc.subtract(columns[index], pa.scalar(origin, type=pa.int64()))
                writer.write_batch(pa.RecordBatch.from_arrays(columns, schema=batch.schema))
        os.replace(temporary, path)
    for camera in cameras:
        (directory / "rgb" / f"{camera}.timestamps").unlink()
        (directory / "rgb" / f"{camera}.log").unlink(missing_ok=True)
    return {"episode_id": episode_id, "cameras": cameras,
            "instructions": [{"start_ns": 0, "end_ns": end, "text": instruction}]}


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
        cameras[identifier(name, "camera ID", r"[a-z0-9_]+")] = f"/dev/video{source}" if source.isdecimal() else source
    return cameras
