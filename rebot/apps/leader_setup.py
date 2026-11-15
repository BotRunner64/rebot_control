"""Scan, bind, calibrate, and view the Star Arm leaders. Hardware origins are never changed."""

import argparse
import math
import os
import sys

import serial
from serial.tools import list_ports

from rebot.bindings import SIDES, load_leaders, save_leaders
from rebot.devices.stararm import (BAUDRATE, capture_gripper, capture_pose, open_leader,
                                   scan_port, stable_port)
from rebot.paths import LEADER_BINDINGS

CH34X_VENDOR_ID = 0x1A86


def scan(port=None, baudrate=BAUDRATE):
    """Query servo IDs 0-7 on one port, or on every CH340/CH341 serial port."""
    ports = [port] if port else sorted(
        p.device for p in list_ports.comports() if p.vid == CH34X_VENDOR_ID)
    found = False
    for name in ports:
        try:
            online = scan_port(name, baudrate)
            print(f"{name}: IDs {online}  path: {stable_port(name)}")
            found |= bool(online)
        except (OSError, serial.SerialException) as exc:
            print(f"{name}: {exc}", file=sys.stderr)
    if not ports:
        print("No CH340/CH341 serial ports found; use --port to specify one.")
    return found


def bind(side, port, baudrate=BAUDRATE, config=LEADER_BINDINGS, replace=False):
    arms = load_leaders(config)
    if side in arms and not replace:
        raise ValueError(f"{side} is already bound; use --replace to rebind it.")
    path = stable_port(port)
    for name, arm in arms.items():
        if name != side and os.path.realpath(arm["port"]) == os.path.realpath(path):
            raise ValueError(f"This port is already bound to {name}.")
    missing = sorted(set(range(7)) - set(scan_port(port, baudrate)))
    if missing:
        raise ValueError(f"Missing servo IDs {missing}; binding was not saved.")
    # Rebinding deliberately drops the previous arm's software calibration.
    arms[side] = {"port": path, "baudrate": baudrate}
    save_leaders(arms, config)
    print(f"Saved {side} -> {path}; configuration: {config}")


def gripper_reference(arm):
    reference = arm.get("reference_deg")
    if (not isinstance(reference, list) or len(reference) != 7
            or any(type(value) not in (int, float) or not math.isfinite(value)
                   for value in reference)):
        raise ValueError("Gripper-only calibration requires seven existing finite reference angles.")
    return reference.copy()


def calibrate(side, replace=False, config=LEADER_BINDINGS, gripper_only=False):
    """Save the current pose (or only the closed handle) as the software reference."""
    arms = load_leaders(config)
    if side not in arms:
        raise ValueError(f"Bind {side} before calibrating it.")
    arm = arms[side]
    if gripper_only:
        gripper_reference(arm)
    if "reference_deg" in arm and not replace:
        raise ValueError("A calibration already exists; use --replace to recalibrate.")
    with open_leader(arm["port"], arm["baudrate"]) as bus:
        reference = capture_gripper(bus) if gripper_only else capture_pose(bus)
    latest = load_leaders(config)
    if any(latest.get(side, {}).get(key) != arm[key] for key in ("port", "baudrate")):
        raise ValueError("The binding changed during calibration; calibration was not saved.")
    if gripper_only:
        updated = gripper_reference(latest[side])
        previous, updated[6] = updated[6], reference
        latest[side]["reference_deg"] = updated
        save_leaders(latest, config)
        print(f"Saved {side} gripper closed reference: {previous:g} -> {reference:g} deg; "
              "J1-J6 references preserved.")
        return
    latest[side]["reference_deg"] = reference
    save_leaders(latest, config)
    print(f"Saved seven software reference angles for {side}: {reference}")


def main(argv=None, prog=None):
    parser = argparse.ArgumentParser(prog=prog, description=__doc__)
    parser.add_argument("--config", type=type(LEADER_BINDINGS), default=LEADER_BINDINGS)
    commands = parser.add_subparsers(dest="command", required=True)
    command = commands.add_parser("scan", help="Query IDs 0-7; scan CH340/CH341 serial ports by default")
    command.add_argument("--port", help="Scan only the specified serial port")
    command.add_argument("--baudrate", type=int, default=BAUDRATE)
    command = commands.add_parser("bind", help="Bind a serial port to the left or right leader arm")
    command.add_argument("side", choices=SIDES)
    command.add_argument("port")
    command.add_argument("--baudrate", type=int, default=BAUDRATE)
    command.add_argument("--replace", action="store_true")
    command = commands.add_parser("calibrate", help="Record the reference pose immediately")
    command.add_argument("side", choices=SIDES)
    command.add_argument("--replace", action="store_true")
    command.add_argument("--gripper-only", action="store_true",
                         help="Capture only the closed handle reference; preserve J1-J6")
    command = commands.add_parser(
        "view", help="Show both leaders in the 3D viewer, or drive virtual reBot arms")
    command.add_argument("--target", choices=("stararm", "rebot"), default="stararm",
                         help="stararm: display leader poses (default); rebot: drive simulated RS arms")
    command.add_argument("--mode", choices=("observe", "hover"), default="observe",
                         help="observe: torque released (default); hover: hold when still with handles closed")
    command.add_argument("--port", type=int, default=8080, help="Local 3D viewer port")
    command.add_argument("--no-browser", action="store_true", help="Print the viewer URL without opening a browser")
    args = parser.parse_args(argv)
    if getattr(args, "baudrate", 1) <= 0:
        parser.error("Baud rate must be positive.")
    if args.command == "scan":
        return 0 if scan(args.port, args.baudrate) else 1
    if args.command == "bind":
        bind(args.side, args.port, args.baudrate, args.config, args.replace)
    elif args.command == "calibrate":
        calibrate(args.side, args.replace, args.config, gripper_only=args.gripper_only)
    else:
        from rebot.apps.leader_view import view

        view(mode=args.mode, config=args.config, target=args.target, port=args.port,
             open_browser=not args.no_browser)
    return 0
