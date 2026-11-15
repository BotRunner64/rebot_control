"""Scan, bind, calibrate, and read the reBot RS follower arms.

Read-only: these commands query device IDs and positions. They never enable,
disable, move, or write parameters or hardware zero.
"""

import argparse
import math
import statistics
import subprocess
import sys
import time

import can

from rebot.bindings import (SIDES, load_arms, new_arm_record, resolve_channel, save_arms,
                            select_arms)
from rebot.devices import socketcan
from rebot.devices.robstride import MOTOR_IDS, open_reader
from rebot.paths import ARM_BINDINGS

JOINT_ORDER = "joint1 joint2 joint3 joint4 joint5 joint6 gripper"


def scan(channel=None):
    """Configure each native CAN interface at 1 Mbps and list motor IDs 1-7."""
    ports = [channel] if channel else socketcan.channels()
    found = False
    for port in ports:
        try:
            socketcan.check_channel(port, configure=True)
            with open_reader(port) as bus:
                motors = bus.scan(strict=False)
            print(f"{port}: IDs {list(motors)}  USB: {socketcan.device_path(port)}")
            for motor_id, uid in motors.items():
                print(f"  ID {motor_id}: UID {uid}")
            if not motors:
                report_empty_scan(port, bus.received_frames)
            found |= bool(motors)
        except (OSError, ValueError, RuntimeError, can.CanError, subprocess.SubprocessError) as exc:
            print(f"{port}: {exc}", file=sys.stderr)
    if not ports:
        print("No native CAN interfaces found. Connect the CAN adapter first.")
    return found


def report_empty_scan(channel, received_frames):
    print(f"{channel}: no matching IDs 1-7; received {received_frames} CAN frames during scan.",
          file=sys.stderr)
    if received_frames == 0:
        print("  No motor replies received. Check arm 48V power, CAN wiring, common ground, "
              "and termination; USB power alone does not power the motors.", file=sys.stderr)
    else:
        print("  Traffic was received, but no matching device-ID replies. "
              "Check motor IDs and the RobStride private CAN protocol configuration.", file=sys.stderr)
    try:
        print("  " + socketcan.link_report(channel), file=sys.stderr)
    except (OSError, ValueError, subprocess.SubprocessError) as exc:
        print(f"  Could not read CAN statistics: {exc}", file=sys.stderr)


def bind(side, channel, replace=False, config=ARM_BINDINGS):
    """Record which adapter and which seven motor UIDs are this side's arm."""
    arms = load_arms(config)
    if side in arms and not replace:
        raise ValueError(f"{side} is already bound; use --replace to rebind it.")
    socketcan.check_channel(channel)
    with open_reader(channel) as bus:
        found = bus.scan(strict=False)
        missing = sorted(set(MOTOR_IDS) - set(found))
        if missing:
            raise ValueError(f"Missing motor IDs {missing}; binding was not saved.")
        if len(set(found.values())) != 7:
            raise ValueError("Motor UIDs are not unique; binding was not saved.")
        bus.positions()
    record = new_arm_record(channel, found)
    for name, other in arms.items():
        same_adapter = (record["device_path"] == other["device_path"]
                        if record["device_path"] and other["device_path"]
                        else channel == other["channel"])
        if name != side and (same_adapter
                             or set(found.values()) & set(other["motor_uids"].values())):
            raise ValueError(f"This adapter or arm is already bound to {name}.")
    # Rebinding deliberately drops the previous arm's software reference.
    arms[side] = record
    save_arms(arms, config)
    print(f"Saved {side} -> {channel}; configuration: {config}")


def capture(read, moved):
    """Median of ten still samples. Preserve the continuous angle; do not fold it into +/- pi."""
    samples = []
    for _ in range(10):
        samples.append(read())
        time.sleep(0.02)
    axes = list(zip(*samples))
    if any(max(axis) - min(axis) > math.radians(2) for axis in axes):
        raise ValueError(moved)
    return [statistics.median(axis) for axis in axes]


def calibrate(side, replace=False, config=ARM_BINDINGS, gripper_only=False):
    """Save the current pose (or only the closed gripper) as the software reference."""
    arm = load_arms(config).get(side)
    if arm is None:
        raise ValueError(f"Bind {side} first.")
    if gripper_only and "reference_rad" not in arm:
        raise ValueError("Calibrate the complete arm before updating only the gripper reference.")
    if "reference_rad" in arm and not replace:
        raise ValueError("A calibration already exists; use --replace to recalibrate.")
    binding = select_arms(config, side, calibrated=False)[side]
    with open_reader(binding.channel) as bus:
        binding.verify(bus)
        if gripper_only:
            reference = list(arm["reference_rad"])
            reference[6], = capture(lambda: [bus.position(7)],
                                    "The gripper moved during capture. Keep it fully closed and retry.")
        else:
            reference = capture(bus.positions, "The arm moved during capture. Hold it still and retry.")
    latest = load_arms(config)
    if latest.get(side) != arm:
        raise ValueError("The binding changed during calibration; reference was not saved.")
    latest[side]["reference_rad"] = reference
    save_arms(latest, config)
    if gripper_only:
        print(f"Saved {side} gripper closed reference: "
              f"{math.degrees(arm['reference_rad'][6]):+.2f} -> {math.degrees(reference[6]):+.2f} deg "
              f"({reference[6]:+.9f} rad). J1-J6 software references are unchanged.")
    else:
        print(f"Saved seven software reference angles (rad) for {side}: {reference}")


def read(side, watch=False, config=ARM_BINDINGS):
    """Print raw motor angles and, once calibrated, angles relative to the reference."""
    binding = select_arms(config, side, calibrated=False)[side]
    with open_reader(binding.channel) as bus:
        binding.verify(bus)
        print(f"Order: {JOINT_ORDER}; unit: deg")
        while True:
            values = bus.positions()
            print("raw: " + " ".join(f"{math.degrees(x):+.2f}" for x in values), flush=True)
            if binding.reference is not None:
                relative = [a - b for a, b in zip(values, binding.reference)]
                print("relative: " + " ".join(f"{math.degrees(x):+.2f}" for x in relative), flush=True)
            if not watch:
                break
            time.sleep(0.1)


def main(argv=None, prog=None):
    parser = argparse.ArgumentParser(prog=prog, description=__doc__)
    parser.add_argument("--config", type=type(ARM_BINDINGS), default=ARM_BINDINGS)
    commands = parser.add_subparsers(dest="command", required=True)
    command = commands.add_parser("scan", help="Configure native CAN at 1 Mbps and query IDs 1-7")
    command.add_argument("--channel")
    command = commands.add_parser("bind", help="Bind an adapter and motor UIDs to an arm")
    command.add_argument("side", choices=SIDES)
    command.add_argument("channel")
    command.add_argument("--replace", action="store_true")
    command = commands.add_parser("calibrate", help="Capture the software reference pose immediately")
    command.add_argument("side", choices=SIDES)
    command.add_argument("--replace", action="store_true")
    command.add_argument("--gripper-only", action="store_true",
                         help="Record only the fully closed gripper reference, preserving J1-J6")
    command = commands.add_parser("read", help="Read raw and relative motor angles")
    command.add_argument("side", choices=SIDES)
    command.add_argument("--watch", action="store_true")
    args = parser.parse_args(argv)
    if args.command == "scan":
        return 0 if scan(args.channel) else 1
    if args.command == "bind":
        bind(args.side, args.channel, args.replace, args.config)
    elif args.command == "calibrate":
        calibrate(args.side, args.replace, args.config, gripper_only=args.gripper_only)
    else:
        read(args.side, args.watch, args.config)
    return 0
