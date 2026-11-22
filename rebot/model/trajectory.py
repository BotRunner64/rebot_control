"""Minimum-jerk joint trajectories and their tracking controller (home, identify)."""

import json
import math
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from rebot.model.arrays import vector6
from rebot.model.limits import validate_tracking_error
from rebot.paths import RUNS

# Native position gains from the reference config/rebotarm_rs.yaml (J1-J6).
MOTION_KP = np.array([50., 150., 150., 50., 50., 50.])
MOTION_KD = np.array([3., 10., 10., 5., 4., 4.])
HOME_SPEED = .3
SAMPLING_SPEED = .8


def minimum_jerk(start, goal, elapsed, duration):
    """Position and velocity on a rest-to-rest quintic."""
    progress = np.clip(elapsed / duration, 0., 1.)
    blend = 10 * progress**3 - 15 * progress**4 + 6 * progress**5
    derivative = 30 * progress**2 * (1 - progress)**2 / duration
    delta = goal - start
    return start + blend * delta, derivative * delta


def movement_duration(start, goal, speed):
    """Duration whose peak joint speed equals `speed` (rad/s), at least 0.5 s."""
    if not math.isfinite(speed) or speed <= 0:
        raise ValueError("Movement speed must be finite and positive.")
    return max(.5, 1.875 * float(np.max(np.abs(goal - start))) / speed)


class TrajectoryTracker:
    """Turn one measured pose and one trajectory point into a checked MIT command record."""

    def __init__(self, model, settings, kp=MOTION_KP, kd=MOTION_KD):
        self.model, self.cfg = model, settings
        self.kp, self.kd = kp, kd
        self.previous_q = self.previous_time = None
        self.velocity = np.zeros(6)

    def update(self, q, target, now, desired_velocity=None):
        q, target = vector6(q, "q"), vector6(target, "target")
        desired_velocity = (np.zeros(6) if desired_velocity is None
                            else vector6(desired_velocity, "desired_velocity"))
        cfg = self.cfg
        validate_tracking_error(target, q, cfg.max_tracking_error_deg)
        self.model.validate(q)
        raw_velocity = np.zeros(6)
        dt = 0.
        if self.previous_time is not None:
            dt = now - self.previous_time
            if not 0 < dt <= cfg.max_sample_gap_s:
                raise RuntimeError("Motion feedback gap exceeded the control deadline.")
            raw_velocity = (q - self.previous_q) / dt
            self.velocity += (1 - math.exp(-dt / cfg.velocity_filter_s)) * (raw_velocity - self.velocity)
        self.previous_q, self.previous_time = q.copy(), now
        error = target - q
        gravity = self.model.gravity(q) * cfg.gravity_scale
        if np.any(np.abs(gravity) > cfg.torque_limit_nm):
            raise RuntimeError("Required gravity torque exceeds configured limits.")
        # This estimate only gates samples. Never rewrite the trajectory position
        # using feedback: its matching derivative would no longer be valid.
        pd = self.kp * error + self.kd * (desired_velocity - self.velocity)
        requested = gravity + pd
        saturated = bool(np.any(np.abs(requested) >= .98 * cfg.torque_limit_nm))
        stable = bool(not saturated and np.max(np.abs(error)) <= .08
                      and np.max(np.abs(self.velocity)) <= .015
                      and np.max(np.abs(raw_velocity)) <= .02)
        return {"q": q, "target": target, "error": error,
                "desired_velocity": desired_velocity, "velocity": self.velocity.copy(),
                "raw_velocity": raw_velocity, "gravity": gravity, "pd": pd,
                "requested": requested, "estimated": requested.copy(), "cycle_s": dt,
                "saturated": saturated, "stable": stable}


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


def plain(value):
    """Convert NumPy containers to JSON/YAML-friendly Python values."""
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, dict):
        return {key: plain(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [plain(item) for item in value]
    if isinstance(value, np.generic):
        return value.item()
    return value


class MotionLog:
    """Buffered JSON-lines diagnostics; no per-tick file flush."""

    def __init__(self, path=None):
        if path is None:
            root = RUNS / "motion"
            root.mkdir(parents=True, exist_ok=True)
            path = root / ("home-" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ") + ".jsonl")
        self.path = Path(path)
        self.stream = self.path.open("x")
        self.flushed = time.monotonic()

    def write(self, record):
        self.stream.write(json.dumps(plain(record), allow_nan=False) + "\n")
        now = time.monotonic()
        if now - self.flushed >= 1 or record.get("type") != "sample":
            self.stream.flush()
            self.flushed = now

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.stream.close()
