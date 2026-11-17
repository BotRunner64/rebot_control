"""Debug streams of a policy run, written under an episode's `debug/` directory.

They sit next to the contract streams (`state/`, `rgb/`) but are not part of the dataset
format: `rebot process` and `rebot export` never read them.

    <side>_joints.parquet   every servo cycle: measured joints, joint target, commanded torque
    targets.parquet         every control period: the absolute EEF target handed to the robot
    chunks.parquet          every accepted policy reply, in full
    run.json                parameters and outcome of the run
"""

from contextlib import ExitStack
import json
import multiprocessing as mp
from pathlib import Path
import queue
import signal
import time

import numpy as np

SIDES = ("left", "right")
STREAMS = ("left_eef", "right_eef", "left_gripper", "right_gripper")
JOINT_COLUMNS = ("position", "velocity", "torque", "target", "feedforward")
BATCH_SIZE = 512


def schemas():
    import pyarrow as pa

    def field(name, kind):
        return pa.field(name, kind, nullable=False)

    stamp = field("timestamp_ns", pa.int64())
    pose = pa.list_(pa.float64(), 12)
    joints = pa.schema([stamp, *(field(name, pa.list_(pa.float64(), 7)) for name in JOINT_COLUMNS)])
    targets = pa.schema([
        stamp,
        *(item for side in SIDES for item in (field(f"{side}_pose", pose), field(f"{side}_openness", pa.float32()))),
        field("chunk", pa.int32()), field("row", pa.int32()), field("sent", pa.bool_())])
    chunks = pa.schema([
        stamp, field("observed_timestamp_ns", pa.int64()), field("inference_ms", pa.float64()),
        field("first_row", pa.int32()),
        *(item for name in STREAMS for item in (
            field(f"{name}_offset_ns", pa.list_(pa.int64())),
            field(name, pa.list_(pose if name.endswith("_eef") else pa.float32()))))])
    return {"left_joints": joints, "right_joints": joints, "targets": targets, "chunks": chunks}


def debug_writer(directory, rows, status):
    """Own the Parquet I/O in a separate spawned process, like the state recorder."""
    for sig in (signal.SIGINT, signal.SIGHUP):
        signal.signal(sig, signal.SIG_IGN)
    try:
        import pyarrow as pa
        import pyarrow.parquet as pq

        tables = schemas()
        pending = {name: [] for name in tables}

        def flush(name):
            if pending[name]:
                schema = tables[name]
                arrays = [pa.array(values, type=column.type) for values, column in zip(zip(*pending[name]), schema)]
                writers[name].write_table(pa.Table.from_arrays(arrays, schema=schema))
                pending[name].clear()

        with ExitStack() as stack:
            writers = {name: stack.enter_context(
                pq.ParquetWriter(Path(directory) / f"{name}.parquet", schema, compression="zstd"))
                for name, schema in tables.items()}
            status.send({"type": "ready"})
            parent = mp.parent_process()
            while True:
                try:
                    item = rows.get(timeout=.2)
                except queue.Empty:
                    if parent is not None and not parent.is_alive():
                        raise RuntimeError("Recording supervisor exited.")
                    continue
                if item is None:
                    break
                name, row = item
                pending[name].append(row)
                if len(pending[name]) >= BATCH_SIZE:
                    flush(name)
            for name in tables:
                flush(name)
        status.send({"type": "done"})
    except BaseException as exc:
        try:
            status.send({"type": "error", "error": f"{type(exc).__name__}: {exc}"})
        except (BrokenPipeError, OSError):
            pass
    finally:
        status.close()


def nanoseconds(seconds):
    return round(seconds * 1e9)


class DebugLog:
    """Queue debug rows without blocking control. Times are host monotonic seconds or nanoseconds.

    `references` maps each side to its seven calibrated zero positions, so joint angles are
    stored relative to the software zero like every other joint angle in this repository.
    """

    def __init__(self, directory, references):
        self.directory = Path(directory)
        self.directory.mkdir()
        self.references = {side: np.asarray(references[side], dtype=float) for side in SIDES}
        self.paths = [self.directory / f"{name}.parquet" for name in ("left_joints", "right_joints",
                                                                      "targets", "chunks")]
        self.ready = self.done = self.closed = False
        context = mp.get_context("spawn")
        self.rows = context.Queue(maxsize=8192)
        self.status, child_status = context.Pipe(duplex=False)
        self.process = context.Process(name="record-debug", target=debug_writer,
                                       args=(self.directory, self.rows, child_status))
        self.process.start()
        child_status.close()

    def check(self, *, finishing=False):
        while self.status.poll():
            try:
                record = self.status.recv()
            except EOFError:
                break
            if record["type"] == "error":
                raise RuntimeError(f"Debug recording failed: {record['error']}")
            self.ready |= record["type"] == "ready"
            self.done |= record["type"] == "done"
        if self.process.exitcode is not None and (not finishing or not self.done):
            raise RuntimeError(f"Debug recorder exited unexpectedly ({self.process.exitcode}).")

    def _put(self, name, row):
        try:
            self.rows.put_nowait((name, row))
        except queue.Full as exc:
            raise RuntimeError("Debug recording queue overflow; stopping instead of silently dropping rows.") from exc

    def joints(self, side, states, command):
        """One servo cycle: the seven motor replies and the command that produced them.

        `command` holds the joint `target` (relative to the software zero; the gripper in motor
        radians) and the commanded `torque`: gravity feedforward on the arm joints, on top of
        which the motor adds its own PD term, and the complete host PD torque on the gripper.
        """
        position = np.array([state["position"] for state in states], dtype=float) - self.references[side]
        self._put(f"{side}_joints", (
            max(state["received_at_ns"] for state in states[:6]), position.tolist(),
            [float(state["velocity"]) for state in states], [float(state["torque"]) for state in states],
            np.asarray(command["target"], dtype=float).tolist(), np.asarray(command["torque"], dtype=float).tolist()))

    def target(self, now, values, chunk, row, sent):
        """One control period: the 26-value absolute target, its chunk and row, and whether it was sent."""
        left, right = np.asarray(values, dtype=float).reshape(2, 13)
        self._put("targets", (nanoseconds(now), left[:12].tolist(), float(left[12]), right[:12].tolist(),
                              float(right[12]), int(chunk), int(row), bool(sent)))

    def chunk(self, chunk, observed_at, accepted_at, latency, first_row):
        """One accepted policy reply as parsed: `{stream: (seconds, values)}`."""
        row = [nanoseconds(accepted_at), nanoseconds(observed_at), latency * 1000., int(first_row)]
        for name in STREAMS:
            times, values = chunk[name]
            row.extend(([nanoseconds(stamp - observed_at) for stamp in times], np.asarray(values).tolist()))
        self._put("chunks", tuple(row))

    def write_run(self, info):
        (self.directory / "run.json").write_text(
            json.dumps(info, ensure_ascii=False, indent=2, default=str) + "\n", encoding="utf-8")

    def finish(self, deadline):
        """Drain the queue and close the tables; they still carry host monotonic timestamps."""
        self.rows.put(None, timeout=1.)
        while True:
            self.check(finishing=True)
            if self.done:
                break
            if time.monotonic() >= deadline:
                raise TimeoutError("Debug recording finalization timed out; pending data was retained.")
            time.sleep(.01)
        self.process.join(2.)
        return self.paths

    def close(self):
        if self.closed:
            return
        self.closed = True
        if self.process.is_alive():
            self.process.terminate()
        self.process.join(2.)
        if self.process.is_alive():
            self.process.kill()
            self.process.join()
        self.rows.cancel_join_thread()
        self.rows.close()
        self.status.close()
