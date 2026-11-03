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


def home(config=CONFIG, settings_path=SETTINGS_PATH, side="right", speed=0.1):
    if not math.isfinite(speed) or not 0 < speed <= 0.2:
        raise ValueError("Home speed must be greater than 0 and at most 0.2 rad/s.")
    cfg = load_settings(settings_path)
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
            keys = stack.enter_context(StopKey())
            buses, models, references, starts = {}, {}, {}, {}
            for name in sides:
                model = models[name] = GravityModel()
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
                buses[name].enable(np.asarray(buses[name].positions()))
            duration = max(2.0, 1.875 * max(float(np.max(np.abs(q))) for q in starts.values()) / speed)
            print(f"MANUAL HOME: {', '.join(sides)}; planned duration {duration:.1f}s. "
                  "Space/Esc/Q/Ctrl-C stops and holds. Motors remain enabled after completion.", flush=True)
            print("Joint-space interpolation does not provide collision avoidance.", flush=True)
            started = previous_time = time.monotonic()
            previous = {name: starts[name].copy() for name in sides}
            velocity = {name: np.zeros(6) for name in sides}
            settled = None
            while not interrupted:
                tick = time.monotonic()
                if keys.pressed():
                    break
                progress = min(1.0, (tick - started) / duration)
                blend = 10 * progress ** 3 - 15 * progress ** 4 + 6 * progress ** 5
                sampled, raw, relative, torques = {}, {}, {}, {}
                for name in sides:
                    sampled[name] = time.monotonic()
                    raw[name] = np.asarray(buses[name].positions())
                    q = relative[name] = raw[name] - references[name]
                    models[name].validate(q)
                    dt = time.monotonic() - previous_time
                    if not 0 < dt <= cfg["max_sample_gap_s"]:
                        raise RuntimeError("Home feedback deadline exceeded.")
                    measured = (q - previous[name]) / dt
                    alpha = 1 - math.exp(-dt / cfg["velocity_filter_s"])
                    velocity[name] += alpha * (measured - velocity[name])
                    target = starts[name] * (1 - blend)
                    gravity = models[name].gravity(q) * cfg["gravity_scale"]
                    if np.any(np.abs(gravity) > cfg["torque_limit_nm"]):
                        raise RuntimeError(f"{name}: gravity torque exceeds limits during home.")
                    buses[name].set_hold(raw[name], gravity, cfg["hold_kp"], cfg["damping"])
                    torques[name] = np.clip(gravity + cfg["hold_kp"] * (target - q)
                                           - cfg["damping"] * velocity[name],
                                           -cfg["torque_limit_nm"], cfg["torque_limit_nm"])
                previous_time = time.monotonic()
                for name in sides:
                    if interrupted:
                        break
                    buses[name].command(raw[name], torques[name],
                                        deadline=sampled[name] + cfg["max_feedback_age_s"])
                previous = {name: q.copy() for name, q in relative.items()}
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
    parser.add_argument("--speed", type=float, default=0.1, help="Peak planned joint speed, at most 0.2 rad/s")
    args = parser.parse_args()
    try:
        home(args.config, args.settings, args.side, args.speed)
        return 0
    except (OSError, ValueError, RuntimeError, can.CanError, yaml.YAMLError,
            subprocess.SubprocessError) as exc:
        print(str(exc), file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
