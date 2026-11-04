#!/usr/bin/env python3
"""Explicitly disable all seven motors per selected arm, including the gripper."""

import argparse
from contextlib import ExitStack
from pathlib import Path
import subprocess
import sys

import can
import yaml

from rebot import CONFIG, load_config, resolve_channel, verify_identity
from rebot_mit import MODE, MotorBus


def disable(config=CONFIG, side="right", prepare_mit=False):
    arms = load_config(config)
    sides = ("left", "right") if side == "both" else (side,)
    if any(name not in arms for name in sides):
        raise ValueError("Bind every selected arm first.")
    channels = {name: resolve_channel(arms[name]) for name in sides}
    if len(set(channels.values())) != len(sides):
        raise ValueError("Selected arms must use distinct CAN interfaces.")
    print("MANUAL DISABLE: all seven motors per selected arm, including the gripper, "
          "will lose motor support. "
          "Place them on a support before running this command.", flush=True)
    success = True
    motors = range(1, 8)
    with ExitStack() as stack:
        buses = {}
        for name in sides:
            buses[name] = stack.enter_context(MotorBus(channels[name], arms[name]["models"], motor_count=7))
            verify_identity(buses[name], arms[name])
        # The only physical disable commands in these experimental control tools
        # live here, behind an explicit invocation of this standalone script.
        for name in sides:
            for motor in motors:
                try:
                    buses[name].send(4, motor)
                except Exception as exc:
                    success = False
                    print(f"{name} motor {motor}: disable send failed: {exc}", file=sys.stderr)
        for name in sides:
            bus = buses[name]
            for motor in motors:
                try:
                    bus.transaction(motor, 2, lambda: bus.send(4, motor), expected_mode=0)
                    if prepare_mit:
                        bus.write_parameter(motor, MODE, 0, "b")
                    print(f"{name} motor {motor}: disable confirmed.", flush=True)
                except Exception as exc:
                    success = False
                    print(f"{name} motor {motor}: disable not verified: {exc}", file=sys.stderr)
    return success


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=CONFIG)
    parser.add_argument("--side", choices=("left", "right", "both"), default="right")
    parser.add_argument("--prepare-mit", action="store_true",
                        help="After confirming manual disable, set and verify MIT mode")
    args = parser.parse_args()
    try:
        return 0 if disable(args.config, args.side, args.prepare_mit) else 2
    except (OSError, ValueError, RuntimeError, can.CanError, yaml.YAMLError,
            subprocess.SubprocessError) as exc:
        print(str(exc), file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
