#!/usr/bin/env python3
"""MIT hand guiding with gravity compensation, damping, and position hold."""

import argparse
from contextlib import ExitStack
from dataclasses import dataclass
import math
import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import time

import can
import numpy as np
import pinocchio as pin
import yaml

from rebot import CONFIG, load_config, resolve_channel, verify_identity
from rebot_mit import MotorBus

MODEL_PATH = Path(__file__).with_name("models") / "rebot_rs_dynamics.urdf"
SETTINGS_PATH = Path(__file__).with_suffix(".yaml")
VECTOR_KEYS = {"hold_kp", "damping", "torque_limit_nm", "gravity_scale"}
SCALAR_KEYS = {"rate_hz", "max_sample_gap_s", "max_feedback_age_s",
               "max_temperature_c", "startup_duration_s", "hold_ramp_s", "velocity_filter_s",
               "still_speed_rad_s", "release_speed_rad_s", "release_error_rad", "settle_duration_s"}


class StopKey:
    """Read Space/Esc/Q without Enter, restoring terminal settings on exit."""

    def __enter__(self):
        self.fd = self.saved = None
        if sys.stdin.isatty():
            import termios
            import tty

            self.fd = sys.stdin.fileno()
            self.saved = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)
        return self

    def pressed(self):
        if self.fd is None or not select.select([self.fd], [], [], 0)[0]:
            return False
        data = os.read(self.fd, 64)
        return any(key in data for key in (b" ", b"\x1b", b"q", b"Q"))

    def __exit__(self, *args):
        if self.saved is not None:
            import termios

            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.saved)


def vector(value, name):
    result = np.asarray(value, dtype=float)
    if result.shape != (6,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain six finite values.")
    return result


def load_settings(path=SETTINGS_PATH):
    settings = yaml.safe_load(path.read_text())
    expected = VECTOR_KEYS | SCALAR_KEYS
    if not isinstance(settings, dict) or set(settings) != expected:
        raise ValueError(f"Settings must contain exactly these keys: {sorted(expected)}")
    for name in SCALAR_KEYS:
        value = settings[name]
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"{name} must be finite and positive.")
    for name in VECTOR_KEYS:
        settings[name] = vector(settings[name], name)
        if np.any(settings[name] <= 0):
            raise ValueError(f"{name} values must be positive.")
    if (not 20 <= settings["rate_hz"] <= 100
            or settings["max_sample_gap_s"] > 0.15
            or settings["max_feedback_age_s"] > settings["max_sample_gap_s"]
            or settings["release_speed_rad_s"] <= settings["still_speed_rad_s"]
            or np.any(settings["torque_limit_nm"] > [6, 20, 15, 5, 3, 2])
            or np.any(settings["gravity_scale"] > 1.2)
            or np.any(settings["hold_kp"] > 50)
            or np.any(settings["damping"] > 5)):
        raise ValueError("Settings exceed experimental limits or have inconsistent thresholds.")
    return settings


class GravityModel:
    def __init__(self, path=MODEL_PATH):
        full = pin.buildModelFromUrdf(str(path))
        finger_ids = [i for i, name in enumerate(full.names)
                      if i and name not in [f"joint{j}" for j in range(1, 7)]]
        self.model = pin.buildReducedModel(full, finger_ids, pin.neutral(full))
        if list(self.model.names)[1:] != [f"joint{i}" for i in range(1, 7)]:
            raise ValueError("Expected six ordered RS arm joints in dynamics model.")
        self.data = self.model.createData()
        self.lower = self.model.lowerPositionLimit.copy()
        self.upper = self.model.upperPositionLimit.copy()

    def gravity(self, q):
        return pin.computeGeneralizedGravity(self.model, self.data, vector(q, "q")).copy()

    def validate(self, q):
        q = vector(q, "q")
        if np.any(q < self.lower - 0.03) or np.any(q > self.upper + 0.03):
            raise ValueError("Joint pose is outside model limits; check software zero and physical pose.")


@dataclass
class Command:
    torque: np.ndarray
    gravity: np.ndarray
    state: str


class HandGuidingController:
    def __init__(self, model, settings):
        self.model, self.settings = model, settings
        self.started = self.previous_time = None
        self.previous_q = self.target = None
        self.velocity = np.zeros(6)
        self.quiet_since = self.hold_since = None
        self.state = "STARTUP"

    def update(self, q, now, gravity_scale=1.0):
        q = vector(q, "q")
        if not math.isfinite(now):
            raise ValueError("Invalid sample time.")
        cfg = self.settings
        self.model.validate(q)
        if self.started is None:
            self.started = self.previous_time = now
            self.previous_q = self.target = q.copy()
        else:
            dt = now - self.previous_time
            if not 0 < dt <= cfg["max_sample_gap_s"]:
                raise RuntimeError("Feedback gap exceeded the control deadline.")
            measured_velocity = (q - self.previous_q) / dt
            alpha = 1 - math.exp(-dt / cfg["velocity_filter_s"])
            self.velocity += alpha * (measured_velocity - self.velocity)
            self.previous_q, self.previous_time = q.copy(), now
        gravity = self.model.gravity(q) * cfg["gravity_scale"] * gravity_scale
        if not np.all(np.isfinite(gravity)) or np.any(np.abs(gravity) > cfg["torque_limit_nm"]):
            raise RuntimeError("Required gravity torque exceeds configured limits; support arm and stop.")
        elapsed = now - self.started
        speed = float(np.max(np.abs(self.velocity)))
        if elapsed < cfg["startup_duration_s"]:
            # Native hold from the preceding run already carries gravity.
            # Keep that support from the first command; ramping it from zero
            # would withdraw support when ownership returns to the host loop.
            stiffness = cfg["hold_kp"]
            self.state = "STARTUP"
        else:
            if self.state == "STARTUP":
                self.state = "HOLD"
                self.hold_since = now - cfg["hold_ramp_s"]
            if self.state == "HOLD" and (
                    speed >= cfg["release_speed_rad_s"]
                    or np.max(np.abs(q - self.target)) >= cfg["release_error_rad"]):
                self.state, self.quiet_since = "DRAG", None
            if self.state == "DRAG":
                self.target = q.copy()
                if speed < cfg["still_speed_rad_s"]:
                    if self.quiet_since is None:
                        self.quiet_since = now
                    elif now - self.quiet_since >= cfg["settle_duration_s"]:
                        self.state, self.hold_since = "HOLD", now
                else:
                    self.quiet_since = None
            blend = (min(1.0, (now - self.hold_since) / cfg["hold_ramp_s"])
                     if self.state == "HOLD" else 0.0)
            stiffness = cfg["hold_kp"] * blend
        torque = gravity + stiffness * (self.target - q) - cfg["damping"] * self.velocity
        # Clip TOTAL torque, including gravity, position hold, and damping.
        torque = np.clip(torque, -cfg["torque_limit_nm"], cfg["torque_limit_nm"])
        return Command(torque, gravity, self.state)


def run(config=CONFIG, settings_path=SETTINGS_PATH, side="right", enable=False,
        duration=0.0, gravity_scale=1.0):
    if not math.isfinite(duration) or duration < 0:
        raise ValueError("Duration must be finite and nonnegative; 0 runs until Ctrl-C.")
    if not math.isfinite(gravity_scale) or not 0 < gravity_scale <= 1:
        raise ValueError("Gravity scale must be greater than 0 and at most 1.")
    cfg = load_settings(settings_path)
    arms = load_config(config)
    sides = ("left", "right") if side == "both" else (side,)
    if any(name not in arms or "reference_rad" not in arms[name] for name in sides):
        raise ValueError("Bind and calibrate every selected arm first.")
    channels = {name: resolve_channel(arms[name]) for name in sides}
    if len(set(channels.values())) != len(sides):
        raise ValueError("Selected arms must use distinct CAN interfaces.")
    if len(sides) == 2 and (set(arms["left"]["motor_uids"].values())
                           & set(arms["right"]["motor_uids"].values())):
        raise ValueError("Selected arms have overlapping motor UIDs.")
    controllers = {name: HandGuidingController(GravityModel(), cfg) for name in sides}
    references = {name: np.asarray(arms[name]["reference_rad"][:6]) for name in sides}
    stop_requested = False

    def request_stop(signum, frame):
        nonlocal stop_requested
        stop_requested = True

    old_signals = {sig: signal.signal(sig, request_stop)
                   for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)}
    try:
        with ExitStack() as stack:
            stop_key = stack.enter_context(StopKey()) if enable else None
            buses, raw = {}, {}
            for name in sides:
                buses[name] = stack.enter_context(MotorBus(
                    channels[name], arms[name]["models"], max_temperature=cfg["max_temperature_c"]))
                verify_identity(buses[name], arms[name])
                raw[name] = np.asarray(buses[name].positions())
                q = raw[name] - references[name]
                model = controllers[name].model
                model.validate(q)
                gravity = model.gravity(q) * cfg["gravity_scale"] * gravity_scale
                if np.any(np.abs(gravity) > cfg["torque_limit_nm"]):
                    raise RuntimeError(f"{name}: gravity torque exceeds configured limits.")
                if enable:
                    buses[name].set_hold(raw[name], gravity, cfg["hold_kp"], cfg["damping"])
                    buses[name].hold_gravity = (
                        lambda positions, model=model, reference=references[name]: np.clip(
                            model.gravity(np.asarray(positions) - reference)
                            * cfg["gravity_scale"] * gravity_scale,
                            -cfg["torque_limit_nm"], cfg["torque_limit_nm"]))
                print(f"{name}: verified {channels[name]}; predicted gravity (N*m): "
                      + " ".join(f"{x:+.3f}" for x in gravity), flush=True)
            if enable:
                print("ACTIVE MIT: support the selected arms during startup and exit. "
                      "Space, Esc, Q or Ctrl-C: soft stop and native position hold. "
                      "Exit keeps motors enabled; no automatic homing. Gripper is unchanged.", flush=True)
                for name in sides:
                    buses[name].prepare()
                    if stop_requested:
                        return
                # Re-read after setup, before enabling. Never target zero.
                for name in sides:
                    if stop_requested:
                        return
                    raw[name] = np.asarray(buses[name].positions())
                    q = raw[name] - references[name]
                    controllers[name].model.validate(q)
                    gravity = controllers[name].model.gravity(q) * cfg["gravity_scale"] * gravity_scale
                    if np.any(np.abs(gravity) > cfg["torque_limit_nm"]):
                        raise RuntimeError(f"{name}: gravity torque exceeds limits before takeover.")
                    buses[name].set_hold(raw[name], gravity, cfg["hold_kp"], cfg["damping"])
                    buses[name].enable(raw[name])
            else:
                print("READ-ONLY CHECK: no mode changes, enable/disable, watchdog writes, "
                      "or torque commands. Add --enable only after single-arm checks.", flush=True)
            started = time.monotonic()
            next_log = started
            states = {}
            while not stop_requested and (not duration or time.monotonic() - started < duration):
                tick = time.monotonic()
                if stop_key is not None and stop_key.pressed():
                    stop_requested = True
                    break
                commands, sampled = {}, {}
                # Read and validate both arms before sending any new torque.
                for name in sides:
                    sampled[name] = time.monotonic()
                    raw[name] = np.asarray(buses[name].positions())
                    q = raw[name] - references[name]
                    try:
                        commands[name] = controllers[name].update(q, time.monotonic(), gravity_scale)
                    except (ValueError, RuntimeError) as exc:
                        raise RuntimeError(f"{name}: {exc}") from exc
                    if enable:
                        buses[name].set_hold(raw[name], commands[name].gravity,
                                             cfg["hold_kp"], cfg["damping"])
                for name in sides:
                    if time.monotonic() - sampled[name] > cfg["max_feedback_age_s"]:
                        raise RuntimeError(f"{name}: feedback too old to apply torque.")
                if enable and not stop_requested:
                    for name in sides:
                        if time.monotonic() - sampled[name] > cfg["max_feedback_age_s"]:
                            raise RuntimeError(f"{name}: feedback became stale before command.")
                        buses[name].command(raw[name], commands[name].torque,
                                            deadline=sampled[name] + cfg["max_feedback_age_s"])
                now = time.monotonic()
                if now >= next_log or any(states.get(name) != commands[name].state for name in sides):
                    for name in sides:
                        print(f"{name} {commands[name].state}: torque (N*m): "
                              + " ".join(f"{x:+.3f}" for x in commands[name].torque), flush=True)
                    next_log = now + 1
                    states = {name: commands[name].state for name in sides}
                time.sleep(max(0, 1 / cfg["rate_hz"] - (time.monotonic() - tick)))
    finally:
        for sig, handler in old_signals.items():
            signal.signal(sig, handler)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=CONFIG)
    parser.add_argument("--settings", type=Path, default=SETTINGS_PATH)
    parser.add_argument("--side", choices=("left", "right", "both"), default="right")
    parser.add_argument("--enable", action="store_true", help="Enable physical joints 1-6 and send torque")
    parser.add_argument("--duration", type=float, default=0, help="Seconds to run; 0 until Ctrl-C")
    parser.add_argument("--gravity-scale", type=float, default=1.0, help="Additional gravity scaling (0, 1]")
    args = parser.parse_args()
    try:
        run(args.config, args.settings, args.side, args.enable, args.duration, args.gravity_scale)
        return 0
    except (OSError, ValueError, RuntimeError, ImportError, can.CanError, yaml.YAMLError,
            subprocess.SubprocessError) as exc:
        print(str(exc), file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
