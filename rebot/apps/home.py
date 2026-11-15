"""Return selected arms to their software reference, then disable all seven motors.

This is the only command that disables motors.
"""

import argparse
from contextlib import ExitStack
from dataclasses import replace
import math
from pathlib import Path
import sys
import time

import numpy as np

from rebot.bindings import select_arms
from rebot.devices.robstride import DISABLE, MOTOR_IDS, STATUS, MotorBus
from rebot.model.gravity import GravityModel
from rebot.model.trajectory import (HOME_SPEED, MotionLog, MotionStats, TrajectoryTracker,
                                    minimum_jerk, movement_duration)
from rebot.paths import ARM_BINDINGS, CONTROL_SETTINGS
from rebot.runtime.arm import Arm, activate, open_arms
from rebot.runtime.console import StopKey, StopSignal, stop_signals
from rebot.settings import load_settings

HOME_TOLERANCE_RAD = 0.05
HOME_SETTLE_S = 0.5


def disable(config=ARM_BINDINGS, side="right", *, bus_factory=MotorBus):
    """Disable all seven motors per selected arm and verify their stopped state."""
    bindings = select_arms(config, side, calibrated=False)
    print("HOME DISABLE: all seven motors per selected arm, including the gripper, "
          "will lose motor support.", flush=True)
    success = True
    with ExitStack() as stack:
        buses = {}
        for name, binding in bindings.items():
            buses[name] = stack.enter_context(bus_factory(binding.channel, binding.models, motor_count=7))
            binding.verify(buses[name])
        # First tell every motor, then spend time confirming each one.
        for name, bus in buses.items():
            for motor in MOTOR_IDS:
                try:
                    bus.send(DISABLE, motor)
                except Exception as exc:
                    success = False
                    print(f"{name} motor {motor}: disable send failed: {exc}", file=sys.stderr)
        for name, bus in buses.items():
            for motor in MOTOR_IDS:
                try:
                    bus.transaction(motor, STATUS, lambda: bus.send(DISABLE, motor), expected_mode=0)
                    print(f"{name} motor {motor}: disable confirmed.", flush=True)
                except Exception as exc:
                    success = False
                    print(f"{name} motor {motor}: disable not verified: {exc}", file=sys.stderr)
    return success


def home(config=ARM_BINDINGS, settings_path=CONTROL_SETTINGS, side="right", speed=HOME_SPEED,
         calibration=None, *, bus_factory=MotorBus, stop=None):
    """Move to the reference pose; True once every selected motor is confirmed disabled."""
    if not math.isfinite(speed) or speed <= 0:
        raise ValueError("Home speed must be finite and positive.")
    cfg = load_settings(settings_path)
    if calibration is not None:
        cfg = replace(cfg, gravity_scale=np.ones(6))
    bindings = select_arms(config, side)
    sides = tuple(bindings)
    stop = StopSignal() if stop is None else stop
    arms, trackers, stats = {}, {}, {}
    for name, binding in bindings.items():
        model = GravityModel(calibration=calibration, binding=binding)
        model.validate(np.zeros(6))
        arms[name] = Arm(binding, cfg, gravity=model, joint_limits=False, bus_factory=bus_factory)
        trackers[name] = TrajectoryTracker(model, cfg)
        stats[name] = MotionStats()
    completed = False
    with stop_signals(stop), ExitStack() as stack:
        journal = stack.enter_context(MotionLog())
        print(f"Home motion log: {journal.path}", flush=True)
        journal.write({"type": "metadata", "version": 2, "control": "native_mit", "sides": sides,
                       "speed": speed, "kp": arms[sides[0]].kp, "kd": arms[sides[0]].kd,
                       "arms": {name: {"motor_uids": bindings[name].motor_uids,
                                       "reference_rad": bindings[name].reference[:6]} for name in sides}})
        keys = stack.enter_context(StopKey())
        for name in sides:
            stack.callback(stats[name].report, name)
        stack.enter_context(open_arms(arms))
        if not activate(arms, stop.is_set):
            return False
        starts = {name: arm.q.copy() for name, arm in arms.items()}
        duration = max(movement_duration(q, np.zeros(6), speed) for q in starts.values())
        print(f"MANUAL HOME: {', '.join(sides)}; planned duration {duration:.1f}s. "
              "Completion disables all seven motors, including the gripper. "
              "Provide support at the reference pose before starting. "
              "Space/Esc/Q/Ctrl-C stops and holds without disabling.", flush=True)
        print("Joint-space interpolation does not provide collision avoidance.", flush=True)
        started = time.monotonic()
        settled = None
        while not stop.is_set():
            tick = time.monotonic()
            if keys.pressed():
                break
            progress = min(1.0, (tick - started) / duration)
            records = {}
            for name, arm in arms.items():
                q = arm.sample()
                target, velocity = minimum_jerk(starts[name], np.zeros(6), tick - started, duration)
                records[name] = trackers[name].update(q, target, time.monotonic(), velocity)
            for name, arm in arms.items():
                if stop.is_set():
                    break
                arm.command_trajectory(records[name])
                stats[name].update(records[name])
                journal.write({"type": "sample", "side": name, "time_s": time.monotonic() - started,
                               "phase": "MOVE" if progress < 1 else "SETTLE",
                               **arm.telemetry(), **records[name]})
            if progress == 1:
                if all(np.max(np.abs(arm.q)) <= HOME_TOLERANCE_RAD for arm in arms.values()):
                    settled = tick if settled is None else settled
                    if tick - settled >= HOME_SETTLE_S:
                        completed = True
                        print("Home reached. Preparing to disable all seven motors.", flush=True)
                        break
                else:
                    settled = None
            time.sleep(max(0, 1 / cfg.rate_hz - (time.monotonic() - tick)))
    if not completed or stop.is_set():
        print("Home interrupted. Keeping motors enabled; disable skipped.", flush=True)
        return False
    # The motion buses are closed before disabling so their hold cleanup cannot
    # send position commands after the motors have been disabled.
    if not disable(config, side, bus_factory=bus_factory):
        raise RuntimeError("Home reached, but disabling all selected motors was not confirmed.")
    return True


def main(argv=None, prog=None):
    parser = argparse.ArgumentParser(prog=prog, description=__doc__)
    parser.add_argument("--config", type=Path, default=ARM_BINDINGS)
    parser.add_argument("--settings", type=Path, default=CONTROL_SETTINGS)
    parser.add_argument("--side", choices=("left", "right", "both"), default="right")
    parser.add_argument("--speed", type=float, default=HOME_SPEED, help="Peak planned joint speed in rad/s")
    parser.add_argument("--calibration", type=Path, help="Accepted robot-bound gravity calibration YAML")
    args = parser.parse_args(argv)
    return 0 if home(args.config, args.settings, args.side, args.speed, args.calibration) else 130
