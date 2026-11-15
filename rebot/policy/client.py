"""Asynchronous dual-arm control with a remote policy server."""

from contextlib import ExitStack
import math
import threading
import time

import numpy as np

from rebot.paths import ARM_BINDINGS, CONTROL_SETTINGS, GRIPPER_SETTINGS
from rebot.runtime.console import StopKey, stop_signals
from rebot.runtime.timing import LoopTiming

CONTROL_HZ = 30
SIDES = ("left", "right")
STREAMS = ("left_eef", "right_eef", "left_gripper", "right_gripper")


def connect_policy(host, port, timeout):
    from rebot_policy_link import PolicyClient

    client = PolicyClient(host, port, timeout=timeout)
    try:
        unknown = set(client.metadata["state"]) - set(STREAMS)
        if unknown:
            raise ValueError(f"Policy requires state streams this robot does not provide: {sorted(unknown)}.")
        return client
    except BaseException:
        client.close()
        raise


def build_observation(snapshot, frames, instruction):
    """Latest rows of every stream, in the recording contract's names, units and columns."""
    def ns(seconds):
        return round(seconds * 1e9)

    state = {}
    for side, row in zip(SIDES, np.asarray(snapshot["state"], dtype=float).reshape(2, 13)):
        stamp = [ns(snapshot["acquired_at"][side])]
        state[f"{side}_eef"] = {"timestamp_ns": stamp, "pose": [row[:12].tolist()]}
        state[f"{side}_gripper"] = {"timestamp_ns": stamp, "openness": [float(row[12])]}
    return {"timestamp_ns": ns(snapshot["observed_at"]), "instruction": instruction, "state": state,
            "rgb": {name: {"timestamp_ns": [ns(stamp)], "jpeg": [jpeg]}
                    for name, (jpeg, stamp) in frames.items()}}


def parse_actions(reply):
    """Action reply -> {stream: (seconds, values)}; future rows of the recorded state streams."""
    tables = reply.get("state") if isinstance(reply, dict) else None
    if not isinstance(tables, dict) or set(tables) != set(STREAMS):
        raise ValueError(f"Policy actions must contain exactly the state streams {list(STREAMS)}.")
    chunk = {}
    for name in STREAMS:
        column = "pose" if name.endswith("_eef") else "openness"
        times = np.asarray(tables[name]["timestamp_ns"], dtype=np.int64) / 1e9
        values = np.asarray(tables[name][column], dtype=float)
        if (not len(times) or values.shape != ((len(times), 12) if column == "pose" else (len(times),))
                or not np.all(np.isfinite(values)) or np.any(np.diff(times) <= 0)):
            raise ValueError(f"Policy stream {name} needs finite {column} rows at strictly increasing times.")
        if column == "pose":
            rotation = values[:, 3:].reshape(-1, 3, 3).swapaxes(-1, -2)
            if (not np.allclose(rotation.swapaxes(-1, -2) @ rotation, np.eye(3), atol=1e-3, rtol=0)
                    or not np.allclose(np.linalg.det(rotation), 1., atol=1e-3, rtol=0)):
                raise ValueError("Policy rotations must be proper column-major rotation matrices.")
        else:
            values = np.clip(values, 0., 1.)
        chunk[name] = times, values
    return chunk


class ActionTimeline:
    """Each stream's active target is its first row stamped later than now."""

    def __init__(self):
        self.chunk = None
        self.observed_at = None

    def accept(self, chunk, observed_at, now, replan_after=0.):
        ends = min(times[-1] for times, _ in chunk.values())
        if not math.isfinite(observed_at) or any(times[0] <= observed_at for times, _ in chunk.values()):
            raise ValueError("Policy targets must be stamped later than their observation.")
        if not observed_at <= now < ends:
            raise TimeoutError("Returned action chunk is already expired or has an invalid timestamp.")
        if ends <= observed_at + replan_after:
            raise ValueError("Action chunk ends before the next replanning time.")
        if self.observed_at is not None and observed_at <= self.observed_at:
            raise ValueError("An older observation cannot replace the active action chunk.")
        self.chunk, self.observed_at = chunk, observed_at

    def sample(self, now):
        if self.chunk is None:
            return None
        target, rows = np.empty(26), {}
        for name, (times, values) in self.chunk.items():
            rows[name] = int(np.searchsorted(times, now, side="right"))
            if rows[name] >= len(times):
                raise TimeoutError("Action buffer exhausted before the next inference arrived.")
            offset = SIDES.index(name.rsplit("_", 1)[0]) * 13
            if name.endswith("_eef"):
                target[offset:offset + 12] = values[rows[name]]
            else:
                target[offset + 12] = values[rows[name]]
        return target, rows["left_eef"]


class InferenceWorker:
    """At most one request/result in flight; never build an observation backlog."""

    def __init__(self, client, observe, stop):
        self.client, self.observe, self.stop = client, observe, stop
        self.lock = threading.Lock()
        self.wake = threading.Event()
        self.done = threading.Event()
        self.pending = False
        self.requested_at = None
        self.result = self.error = None
        self.thread = threading.Thread(target=self._run, name="policy-inference", daemon=True)

    def request(self):
        self.check()
        with self.lock:
            if self.pending:
                return False
            self.pending = True
            self.requested_at = time.monotonic()
        self.wake.set()
        return True

    def check(self):
        with self.lock:
            error = self.error
            expired = (self.pending and self.result is None and self.requested_at is not None
                       and time.monotonic() - self.requested_at > self.client.timeout)
        if error is not None:
            raise RuntimeError(f"Asynchronous inference failed: {error}") from error
        if expired:
            self.stop.set()
            raise TimeoutError("Asynchronous observation/inference exceeded its request timeout.")

    def take(self):
        self.check()
        with self.lock:
            result, self.result = self.result, None
            if result is not None:
                self.pending = False
            return result

    def _run(self):
        try:
            while not self.done.is_set() and not self.stop.is_set():
                if not self.wake.wait(.02):
                    continue
                self.wake.clear()
                if self.done.is_set() or self.stop.is_set():
                    break
                observation, observed_at = self.observe()
                began = time.monotonic()
                chunk = parse_actions(self.client.infer(observation))
                with self.lock:
                    self.result = chunk, observed_at, time.monotonic() - began
        except Exception as exc:
            if self.done.is_set():
                return
            with self.lock:
                self.error = exc
            self.stop.stop_reason = f"asynchronous inference failed: {exc}"
            self.stop.set()

    def __enter__(self):
        self.thread.start()
        return self

    def __exit__(self, *_):
        self.done.set()
        self.wake.set()
        self.client.close()
        self.thread.join(timeout=self.client.timeout + 2.)
        if self.thread.is_alive():
            raise RuntimeError("Inference worker did not stop within its socket timeout.")


def control_loop(robot, client, observe, stop, *, duration=None, replan_steps=16, keys=None):
    if (type(replan_steps) is not int or replan_steps < 1
            or (duration is not None and (not math.isfinite(duration) or duration <= 0))):
        raise ValueError("Invalid duration or replanning interval.")
    timeline = ActionTimeline()
    timing = LoopTiming("policy-target", CONTROL_HZ)
    started = next_request = time.monotonic()
    requests = commands = 0
    with InferenceWorker(client, observe, stop) as inference:
        while not stop.is_set():
            began = timing.tick()
            robot.check()
            if duration is not None and began - started >= duration:
                break
            if keys is not None and keys.pressed():
                stop.set()
                break
            result = inference.take()
            if result is not None:
                chunk, observed_at, latency = result
                now = time.monotonic()
                timeline.accept(chunk, observed_at, now, replan_steps / CONTROL_HZ)
                next_request = observed_at + replan_steps / CONTROL_HZ
                print(f"Policy chunk {requests}: inference={latency * 1000:.1f} ms; "
                      f"skip={timeline.sample(now)[1]} elapsed rows.", flush=True)
            now = time.monotonic()
            if now >= next_request and inference.request():
                requests += 1
            selected = timeline.sample(now)
            if selected is not None:
                if robot.command(selected[0]) is not False:
                    commands += 1
            timing.complete(began, time.monotonic())
            timing.pace(stop)
        inference.check()
        robot.check()
    report = {**timing.report(), "requests": requests, "commands": commands}
    print(f"Policy control: {report['hz']:.1f} Hz; {requests} requests; "
          f"{commands} targets; skipped periods={report['skipped']}.", flush=True)
    return report


def run(*, host, port, instruction, hardware=False, camera=None, mock_cameras=False,
        replan_steps=16, request_timeout=1., camera_max_age=.25, camera_max_skew=.10,
        duration=None, gripper_open_deg=270., config=ARM_BINDINGS, settings=CONTROL_SETTINGS,
        gripper_settings=GRIPPER_SETTINGS):
    """Connect the policy, the cameras and the robot, then run `control_loop` until stopped."""
    # Imported here so a refused policy never loads the camera or robot stack.
    from rebot.policy.cameras import PolicyCameras
    from rebot.runtime.robot import Robot

    stop = threading.Event()
    with ExitStack() as stack:
        stack.enter_context(stop_signals(stop))
        client = connect_policy(host, port, request_timeout)
        stack.callback(client.close)
        if hardware and client.metadata["mock"]:
            raise ValueError("A mock policy cannot control physical hardware.")
        cameras = stack.enter_context(PolicyCameras(camera, client.metadata["rgb"],
                                                   mock=mock_cameras, max_age=camera_max_age))
        robot = stack.enter_context(Robot(hardware=hardware, stop=stop, config=config,
                                          settings=settings, gripper_settings=gripper_settings,
                                          gripper_open_deg=gripper_open_deg))
        keys = stack.enter_context(StopKey())

        def observe():
            snapshot = robot.state()
            observed_at = snapshot["observed_at"]
            frames = cameras.snapshot()
            if any(abs(stamp - observed_at) > camera_max_skew for _, stamp in frames.values()):
                raise TimeoutError("Camera/state acquisition timestamps differ too much.")
            return build_observation(snapshot, frames, instruction), observed_at

        print(f"Policy {'HARDWARE' if hardware else 'SIMULATION'}: "
              f"ws://{host}:{port}; {CONTROL_HZ} Hz; replan every {replan_steps} steps. "
              "Space/Esc/Q/Ctrl-C stops control and hardware exit attempts current-position hold.", flush=True)
        return control_loop(robot, client, observe, stop, duration=duration,
                            replan_steps=replan_steps, keys=keys)
