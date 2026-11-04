#!/usr/bin/env python3
"""Manually return selected arms to their software reference, then keep enabled."""

import argparse
from contextlib import ExitStack
import math
from pathlib import Path
import signal
import subprocess
import sys
import time

import can
import numpy as np
import yaml

from rebot import CONFIG, load_config, resolve_channel, verify_identity
from rebot_hand_guiding import GravityModel, SETTINGS_PATH, StopKey, load_settings
from rebot_mit import MotorBus
from rebot_motion import (HOME_SPEED, MotionController, MotionLog, MotionStats,
                          minimum_jerk, motion_settings, movement_duration)


def home(config=CONFIG, settings_path=SETTINGS_PATH, side="right", speed=HOME_SPEED, calibration=None):
    if not math.isfinite(speed) or speed <= 0:
        raise ValueError("Home speed must be finite and positive.")
    cfg = motion_settings(load_settings(settings_path))
    if calibration is not None:
        cfg["gravity_scale"] = np.ones(6)
    arms = load_config(config)
    sides = ("left", "right") if side == "both" else (side,)
    if any(name not in arms or "reference_rad" not in arms[name] for name in sides):
        raise ValueError("Bind and calibrate every selected arm first.")
    channels = {name: resolve_channel(arms[name]) for name in sides}
    if len(set(channels.values())) != len(sides):
        raise ValueError("Selected arms must use distinct CAN interfaces.")
    interrupted = False

    def stop(signum, frame):
        nonlocal interrupted
        interrupted = True

    old_signals = {sig: signal.signal(sig, stop)
                   for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)}
    try:
        with ExitStack() as stack:
            journal = stack.enter_context(MotionLog())
            journal.write({"type": "metadata", "version": 2, "control": "native_mit", "sides": sides,
                           "speed": speed, "kp": cfg["hold_kp"], "kd": cfg["damping"],
                           "arms": {name: {"motor_uids": arms[name]["motor_uids"],
                                            "reference_rad": arms[name]["reference_rad"][:6]} for name in sides}})
            keys = stack.enter_context(StopKey())
            buses, references, starts, controllers, stats = {}, {}, {}, {}, {}
            for name in sides:
                model = GravityModel(calibration=calibration, arm=arms[name], side=name)
                controllers[name] = MotionController(model, cfg)
                stats[name] = MotionStats()
                stack.callback(stats[name].report, name)
                reference = references[name] = np.asarray(arms[name]["reference_rad"][:6])
                bus = buses[name] = stack.enter_context(MotorBus(channels[name], arms[name]["models"],
                                                  max_temperature=cfg["max_temperature_c"]))
                verify_identity(bus, arms[name])
                raw = np.asarray(bus.positions())
                starts[name] = raw - reference
                model.validate(starts[name])
                model.validate(np.zeros(6))
                gravity = model.gravity(starts[name]) * cfg["gravity_scale"]
                if np.any(np.abs(gravity) > cfg["torque_limit_nm"]):
                    raise RuntimeError(f"{name}: initial gravity torque exceeds limits.")
                bus.set_hold(raw, gravity, cfg["hold_kp"], cfg["damping"])
                bus.hold_gravity = lambda q, model=model, reference=reference: np.clip(
                    model.gravity(np.asarray(q) - reference) * cfg["gravity_scale"],
                    -cfg["torque_limit_nm"], cfg["torque_limit_nm"])
            for name in sides:
                if interrupted:
                    return
                buses[name].prepare()
                raw = np.asarray(buses[name].positions())
                starts[name] = raw - references[name]
                gravity = controllers[name].model.gravity(starts[name]) * cfg["gravity_scale"]
                buses[name].set_hold(raw, gravity, cfg["hold_kp"], cfg["damping"])
                buses[name].enable(raw)
            duration = max(movement_duration(q, np.zeros(6), speed) for q in starts.values())
            print(f"MANUAL HOME: {', '.join(sides)}; planned duration {duration:.1f}s. "
                  "Space/Esc/Q/Ctrl-C stops and holds. Motors remain enabled after completion.", flush=True)
            print("Joint-space interpolation does not provide collision avoidance.", flush=True)
            started = time.monotonic()
            settled = None
            while not interrupted:
                tick = time.monotonic()
                if keys.pressed():
                    break
                progress = min(1.0, (tick - started) / duration)
                sampled, raw, relative, records = {}, {}, {}, {}
                for name in sides:
                    sampled[name] = time.monotonic()
                    raw[name] = np.asarray(buses[name].positions())
                    q = relative[name] = raw[name] - references[name]
                    target, desired_velocity = minimum_jerk(starts[name], np.zeros(6), tick - started, duration)
                    records[name] = controllers[name].update(q, target, time.monotonic(), desired_velocity)
                    buses[name].set_hold(raw[name], records[name]["gravity"], cfg["hold_kp"], cfg["damping"])
                for name in sides:
                    if interrupted:
                        break
                    controllers[name].command(buses[name], raw[name], records[name],
                                              sampled[name] + cfg["max_feedback_age_s"])
                    stats[name].update(records[name])
                    journal.write({"type": "sample", "side": name, "time_s": time.monotonic() - started,
                                   "phase": "MOVE" if progress < 1 else "SETTLE",
                                   "reported": [buses[name].feedback[i]["torque"] for i in range(1, 7)],
                                   "temperature": [buses[name].feedback[i]["temperature"] for i in range(1, 7)],
                                   **records[name]})
                if progress == 1:
                    if all(np.max(np.abs(q)) <= 0.03 for q in relative.values()):
                        settled = tick if settled is None else settled
                        if tick - settled >= 0.5:
                            print("Home reached. Leaving native position hold enabled.", flush=True)
                            break
                    else:
                        settled = None
                time.sleep(max(0, 1 / cfg["rate_hz"] - (time.monotonic() - tick)))
    finally:
        for sig, handler in old_signals.items():
            signal.signal(sig, handler)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=CONFIG)
    parser.add_argument("--settings", type=Path, default=SETTINGS_PATH)
    parser.add_argument("--side", choices=("left", "right", "both"), default="right")
    parser.add_argument("--speed", type=float, default=HOME_SPEED, help="Peak planned joint speed in rad/s")
    parser.add_argument("--calibration", type=Path, help="Accepted robot-bound gravity calibration YAML")
    args = parser.parse_args()
    try:
        home(args.config, args.settings, args.side, args.speed, args.calibration)
        return 0
    except (OSError, ValueError, RuntimeError, can.CanError, yaml.YAMLError,
            subprocess.SubprocessError) as exc:
        print(str(exc), file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
