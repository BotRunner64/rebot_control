"""Native MIT trajectory tracking shared by homing and gravity identification."""

import math
import json
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from rebot_gravity import vector


# Native movement gains from the reference config/rebotarm_rs.yaml.
MOTION_KP = np.array([50., 150., 150., 50., 50., 50.])
MOTION_KD = np.array([3., 10., 10., 5., 4., 4.])
HOME_SPEED = .3
SAMPLING_SPEED = .2


def motion_settings(settings):
    cfg = dict(settings)
    cfg["hold_kp"] = MOTION_KP.copy()
    cfg["damping"] = MOTION_KD.copy()
    return cfg


def minimum_jerk(start, goal, elapsed, duration):
    progress = np.clip(elapsed / duration, 0., 1.)
    blend = 10 * progress**3 - 15 * progress**4 + 6 * progress**5
    derivative = 30 * progress**2 * (1 - progress)**2 / duration
    delta = goal - start
    return start + blend * delta, derivative * delta


def movement_duration(start, goal, speed):
    if not math.isfinite(speed) or speed <= 0:
        raise ValueError("Movement speed must be finite and positive.")
    return max(.5, 1.875 * float(np.max(np.abs(goal - start))) / speed)


class MotionLog:
    """Buffered native-motion diagnostics; no per-tick file flush."""

    def __init__(self, root=None):
        root = Path(root) if root is not None else Path(__file__).with_name("motion_runs")
        root.mkdir(exist_ok=True)
        self.path = root / ("home-" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ") + ".jsonl")
        self.stream = self.path.open("x")
        self.flushed = time.monotonic()

    def write(self, record):
        self.stream.write(json.dumps(record, default=lambda value: value.tolist(), allow_nan=False) + "\n")
        now = time.monotonic()
        if now - self.flushed >= 1 or record.get("type") == "metadata":
            self.stream.flush()
            self.flushed = now

    def __enter__(self):
        print(f"Home motion log: {self.path}", flush=True)
        return self

    def __exit__(self, *args):
        self.stream.close()


class MotionController:
    def __init__(self, model, settings):
        self.model, self.cfg = model, settings
        self.previous_q = self.previous_time = None
        self.velocity = np.zeros(6)

    def update(self, q, target, now, desired_velocity=None):
        q, target = vector(q, "q"), vector(target, "target")
        desired_velocity = (np.zeros(6) if desired_velocity is None
                            else vector(desired_velocity, "desired_velocity"))
        cfg = self.cfg
        self.model.validate(q)
        raw_velocity = np.zeros(6)
        dt = 0.
        if self.previous_time is not None:
            dt = now - self.previous_time
            if not 0 < dt <= cfg["max_sample_gap_s"]:
                raise RuntimeError("Motion feedback gap exceeded the control deadline.")
            raw_velocity = (q - self.previous_q) / dt
            self.velocity += (1 - math.exp(-dt / cfg["velocity_filter_s"])) * (raw_velocity - self.velocity)
        self.previous_q, self.previous_time = q.copy(), now
        error = target - q
        gravity = self.model.gravity(q) * cfg["gravity_scale"]
        if np.any(np.abs(gravity) > cfg["torque_limit_nm"]):
            raise RuntimeError("Required gravity torque exceeds configured limits.")
        # This estimate only gates samples. Never rewrite the trajectory position
        # using feedback: its matching derivative would no longer be valid.
        pd = cfg["hold_kp"] * error + cfg["damping"] * (desired_velocity - self.velocity)
        requested = gravity + pd
        saturated = bool(np.any(np.abs(requested) >= .98 * cfg["torque_limit_nm"]))
        stable = bool(not saturated and np.max(np.abs(error)) <= .08
                      and np.max(np.abs(self.velocity)) <= .015
                      and np.max(np.abs(raw_velocity)) <= .02)
        return {"q": q, "target": target, "error": error,
                "desired_velocity": desired_velocity, "velocity": self.velocity.copy(),
                "raw_velocity": raw_velocity, "gravity": gravity, "pd": pd,
                "requested": requested, "estimated": requested.copy(), "cycle_s": dt,
                "saturated": saturated, "stable": stable}

    def command(self, bus, raw, record, deadline):
        began = time.monotonic()
        parameters = bus.command_position(
            np.asarray(raw) + record["target"] - record["q"], record["desired_velocity"],
            record["gravity"], self.cfg["hold_kp"], self.cfg["damping"], deadline)
        record["command_s"] = time.monotonic() - began
        # Like the reference PD sweep, derive quasi-static effort from the
        # native gains and tracking error; firmware velocity scaling is unused.
        record["estimated"] = np.array([
            item["gravity"] + item["kp"] * (item["position"] - q)
            + item["kd"] * (item["velocity"] - v)
            for item, q, v in zip(parameters, raw, record["velocity"])])
        record["native_parameters"] = parameters


class MotionStats:
    """Report measured timing and tracking without changing motor commands."""

    def __init__(self):
        self.cycles = self.elapsed = self.max_cycle = self.max_command = 0
        self.over_budget = 0
        self.max_error = np.zeros(6)
        self.max_step = np.zeros(6)
        self.previous_target = None

    def update(self, record):
        dt = record["cycle_s"]
        if dt > 0:
            self.cycles += 1
            self.elapsed += dt
            self.max_cycle = max(self.max_cycle, dt)
        self.max_command = max(self.max_command, record["command_s"])
        self.max_error = np.maximum(self.max_error, np.abs(record["error"]))
        if self.previous_target is not None:
            self.max_step = np.maximum(self.max_step, np.abs(record["target"] - self.previous_target))
        self.previous_target = record["target"].copy()
        self.over_budget += int(record["saturated"])

    def report(self, label="Motion"):
        if self.elapsed <= 0:
            return
        print(f"{label} timing: {self.cycles / self.elapsed:.1f} Hz; "
              f"max cycle {self.max_cycle * 1000:.1f} ms; "
              f"max command {self.max_command * 1000:.1f} ms; "
              f"sample-effort budget exceeded {self.over_budget} times.", flush=True)
        print("Max tracking error (deg): " + " ".join(f"{value:.2f}" for value in np.degrees(self.max_error)), flush=True)
        print("Max target step (deg): " + " ".join(f"{value:.3f}" for value in np.degrees(self.max_step)), flush=True)
