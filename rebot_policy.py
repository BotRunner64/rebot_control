#!/usr/bin/env python3
"""Run asynchronous dual-arm control with a remote policy server."""

import argparse
from contextlib import ExitStack
import math
from pathlib import Path
import sys
import threading
import time

import numpy as np

HERE = Path(__file__).resolve().parent

ACTION_HZ = 30
ACTION_HORIZON = 50
METADATA = {"action_format": "absolute_eef26", "rotation_order": "column_major",
            "control_frequency": ACTION_HZ, "action_horizon": ACTION_HORIZON}


def connect_policy(host, port, timeout):
    from openpi_client.websocket_client_policy import WebsocketClientPolicy

    policy = WebsocketClientPolicy(host, port, timeout=timeout)
    try:
        metadata = policy.get_server_metadata()
        if not isinstance(metadata, dict):
            raise ValueError("Policy metadata must be a dictionary.")
        for key, value in METADATA.items():
            if metadata.get(key) != value:
                raise ValueError(f"Policy metadata must declare {key}={value!r}; "
                                 "connect to a server using the matching robot action contract.")
        return policy
    except BaseException:
        policy.close()
        raise


def validate_actions(value):
    actions = np.asarray(value, dtype=float)
    if actions.shape != (ACTION_HORIZON, 26) or not np.all(np.isfinite(actions)):
        raise ValueError("Policy actions must be finite absolute states with shape (50, 26).")
    for offset in (0, 13):
        rotation = actions[:, offset + 3:offset + 12].reshape(-1, 3, 3).swapaxes(-1, -2)
        if (not np.allclose(rotation.swapaxes(-1, -2) @ rotation, np.eye(3), atol=1e-3, rtol=0)
                or not np.allclose(np.linalg.det(rotation), 1., atol=1e-3, rtol=0)):
            raise ValueError("Policy rotations must be proper column-major rotation matrices.")
    actions = actions.copy()
    actions[:, [12, 25]] = np.clip(actions[:, [12, 25]], 0., 1.)
    return actions


class ActionTimeline:
    """Row k is the target for observation time + (k + 1) / 30 seconds."""

    def __init__(self):
        self.actions = None
        self.observed_at = None

    def accept(self, actions, observed_at, now):
        actions = validate_actions(actions)
        if not math.isfinite(observed_at) or not 0 <= now - observed_at < ACTION_HORIZON / ACTION_HZ:
            raise TimeoutError("Returned action chunk is already expired or has an invalid timestamp.")
        if self.observed_at is not None and observed_at <= self.observed_at:
            raise ValueError("An older observation cannot replace the active action chunk.")
        self.actions, self.observed_at = actions, observed_at

    def sample(self, now):
        if self.actions is None:
            return None
        index = math.floor((now - self.observed_at) * ACTION_HZ)
        if not 0 <= index < ACTION_HORIZON:
            raise TimeoutError("Action buffer exhausted before the next inference arrived.")
        return self.actions[index].copy(), index


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
                reply = self.client.infer(observation)
                actions = validate_actions(reply["actions"])
                with self.lock:
                    self.result = actions, observed_at, time.monotonic() - began
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
    from rebot_teleop_runtime import LoopTiming

    if (type(replan_steps) is not int or not 1 <= replan_steps < ACTION_HORIZON
            or (duration is not None and (not math.isfinite(duration) or duration <= 0))):
        raise ValueError("Invalid duration or replanning interval.")
    timeline = ActionTimeline()
    timing = LoopTiming("policy-target", ACTION_HZ)
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
                actions, observed_at, latency = result
                now = time.monotonic()
                timeline.accept(actions, observed_at, now)
                next_request = observed_at + replan_steps / ACTION_HZ
                print(f"Policy chunk {requests}: inference={latency * 1000:.1f} ms; "
                      f"skip={math.floor((now - observed_at) * ACTION_HZ)} elapsed rows.", flush=True)
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


def run(args):
    from rebot_ee import stop_signals
    from rebot_hand_guiding import StopKey
    from rebot_policy_camera import PolicyCameras
    from rebot_control import Robot

    stop = threading.Event()
    with ExitStack() as stack:
        stack.enter_context(stop_signals(stop))
        client = connect_policy(args.host, args.port, args.request_timeout)
        stack.callback(client.close)
        if args.hardware and client.get_server_metadata().get("mock_policy"):
            raise ValueError("A mock policy cannot control physical hardware.")
        cameras = stack.enter_context(PolicyCameras(args.camera, mock=args.mock_cameras,
                                                   max_age=args.camera_max_age))
        robot = stack.enter_context(Robot(args, stop))
        keys = stack.enter_context(StopKey())

        def observe():
            snapshot = robot.state()
            state = np.asarray(snapshot["state"], dtype=np.float32)
            observed_at = snapshot["observed_at"]
            images, times = cameras.snapshot()
            if max(abs(value - observed_at) for value in times) > args.camera_max_skew:
                raise TimeoutError("Camera/state acquisition timestamps differ too much.")
            return {"state": state, "image": images,
                    "image_mask": {key: np.bool_(True) for key in images},
                    "prompt": args.instruction}, observed_at

        print(f"Policy {'HARDWARE' if args.hardware else 'SIMULATION'}: "
              f"ws://{args.host}:{args.port}; 30 Hz; replan every {args.replan_steps} steps. "
              "Space/Esc/Q/Ctrl-C stops control and hardware exit attempts current-position hold.", flush=True)
        return control_loop(robot, client, observe, stop, duration=args.duration,
                            replan_steps=args.replan_steps, keys=keys)


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--host", default="127.0.0.1", help="Policy server address")
    result.add_argument("--port", type=int, default=8000)
    result.add_argument("--instruction", required=True)
    result.add_argument("--hardware", action="store_true", help="Enable physical dual-arm control; default is simulation")
    cameras = result.add_mutually_exclusive_group(required=True)
    cameras.add_argument("--camera", action="append", metavar="NAME=DEVICE",
                         help="Repeat for top, left_wrist and right_wrist")
    cameras.add_argument("--mock-cameras", action="store_true", help="Synthetic images for simulation tests")
    result.add_argument("--replan-steps", type=int, default=16, help="Request a new chunk every N observation steps")
    result.add_argument("--request-timeout", type=float, default=1., help="Bound each socket operation in seconds")
    result.add_argument("--camera-max-age", type=float, default=.25)
    result.add_argument("--camera-max-skew", type=float, default=.10, help="Maximum camera/state skew in seconds")
    result.add_argument("--duration", type=float, help="Stop after this many seconds")
    result.add_argument("--gripper-open-deg", type=float, default=270., help="Physical full-open motor angle")
    result.add_argument("--config", type=Path, default=HERE / "rebot.local.yaml")
    result.add_argument("--settings", type=Path, default=HERE / "rebot_hand_guiding.yaml")
    result.add_argument("--gripper-settings", type=Path, default=HERE / "rebot_gripper.yaml")
    return result


def main(argv=None):
    cli = parser()
    args = cli.parse_args(argv)
    if not 1 <= args.port <= 65535:
        cli.error("port must be between 1 and 65535")
    if not args.instruction.strip() or not 1 <= args.replan_steps < 50:
        cli.error("provide a nonempty instruction and replan steps in [1, 49]")
    for key in ("request_timeout", "camera_max_age", "camera_max_skew", "duration", "gripper_open_deg"):
        value = getattr(args, key)
        if value is not None and (not math.isfinite(value) or value <= 0):
            cli.error(f"{key.replace('_', '-')} must be finite and positive")
    if args.gripper_open_deg > 270 or (args.hardware and args.mock_cameras):
        cli.error("gripper opening must be <=270 degrees; hardware requires real cameras")
    try:
        run(args)
        return 0
    except (KeyboardInterrupt, InterruptedError):
        return 130
    except Exception as exc:
        print(f"Policy control failed: {exc}", file=sys.stderr, flush=True)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
