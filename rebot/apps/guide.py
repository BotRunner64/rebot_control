"""Hand guiding: gravity compensation, damping, and position hold when the arm is still."""

import argparse
from dataclasses import replace
import math
from pathlib import Path
import time

import numpy as np

from rebot.bindings import select_arms
from rebot.devices.robstride import MotorBus
from rebot.model.gravity import GravityModel
from rebot.model.guiding import HandGuidingController
from rebot.paths import ARM_BINDINGS, CONTROL_SETTINGS
from rebot.runtime.arm import Arm, activate, open_arms
from rebot.runtime.console import StopKey, StopSignal, stop_signals
from rebot.settings import load_settings


def guide(config=ARM_BINDINGS, settings_path=CONTROL_SETTINGS, side="right",
          duration=0.0, gravity_scale=1.0, calibration=None, *, bus_factory=MotorBus, stop=None):
    """Run until `duration` (0: forever), a stop key, or a signal; exit holds position."""
    if not math.isfinite(duration) or duration < 0:
        raise ValueError("Duration must be finite and nonnegative; 0 runs until Ctrl-C.")
    if not math.isfinite(gravity_scale) or not 0 < gravity_scale <= 1:
        raise ValueError("Gravity scale must be greater than 0 and at most 1.")
    cfg = load_settings(settings_path)
    if calibration is not None:
        # A calibrated model already carries the measured masses.
        cfg = replace(cfg, gravity_scale=np.ones(6))
    bindings = select_arms(config, side)
    stop = StopSignal() if stop is None else stop
    arms, controllers = {}, {}
    for name, binding in bindings.items():
        model = GravityModel(calibration=calibration, binding=binding)
        arms[name] = Arm(binding, cfg, gravity=model, gravity_scale=gravity_scale,
                         kp=cfg.hold_kp, kd=cfg.damping, joint_limits=False,
                         bus_factory=bus_factory)
        controllers[name] = HandGuidingController(model, cfg, gravity_scale)
    with stop_signals(stop), StopKey() as keys, open_arms(arms):
        for name, arm in arms.items():
            print(f"{name}: verified {arm.binding.channel}; predicted gravity (N*m): "
                  + " ".join(f"{x:+.3f}" for x in arm.support(arm.raw)), flush=True)
        print("ACTIVE MIT: support the selected arms during startup and exit. "
              "Space, Esc, Q or Ctrl-C: soft stop and native position hold. "
              "Exit keeps motors enabled; no automatic homing. Gripper is unchanged.", flush=True)
        if not activate(arms, stop.is_set):
            return
        started = time.monotonic()
        next_log = started
        states = {}
        while not stop.is_set() and (not duration or time.monotonic() - started < duration):
            tick = time.monotonic()
            if keys.pressed():
                stop.set("terminal Space/Esc/Q stop key")
                break
            commands = {}
            # Read and validate both arms before sending any new torque.
            for name, arm in arms.items():
                try:
                    q = arm.sample()
                    commands[name] = controllers[name].update(q, time.monotonic())
                except (ValueError, RuntimeError) as exc:
                    raise RuntimeError(f"{name}: {exc}") from exc
            max_age = cfg.max_feedback_age_s
            for name, arm in arms.items():
                if time.monotonic() - arm.sampled_at > max_age:
                    raise RuntimeError(f"{name}: feedback too old to apply torque.")
            if not stop.is_set():
                for name, arm in arms.items():
                    if time.monotonic() - arm.sampled_at > max_age:
                        raise RuntimeError(f"{name}: feedback became stale before command.")
                    arm.command_torque(commands[name].torque)
            now = time.monotonic()
            if now >= next_log or any(states.get(name) != commands[name].state for name in arms):
                for name in arms:
                    print(f"{name} {commands[name].state}: torque (N*m): "
                          + " ".join(f"{x:+.3f}" for x in commands[name].torque), flush=True)
                next_log = now + 1
                states = {name: commands[name].state for name in arms}
            time.sleep(max(0, 1 / cfg.rate_hz - (time.monotonic() - tick)))


def main(argv=None, prog=None):
    parser = argparse.ArgumentParser(prog=prog, description=__doc__)
    parser.add_argument("--config", type=Path, default=ARM_BINDINGS)
    parser.add_argument("--settings", type=Path, default=CONTROL_SETTINGS)
    parser.add_argument("--side", choices=("left", "right", "both"), default="right")
    parser.add_argument("--duration", type=float, default=0, help="Seconds to run; 0 until Ctrl-C")
    parser.add_argument("--gravity-scale", type=float, default=1.0, help="Additional gravity scaling (0, 1]")
    parser.add_argument("--calibration", type=Path, help="Accepted robot-bound gravity calibration YAML")
    args = parser.parse_args(argv)
    guide(config=args.config, settings_path=args.settings, side=args.side,
          duration=args.duration, gravity_scale=args.gravity_scale, calibration=args.calibration)
    return 0
