#!/usr/bin/env python3
"""Star Arm discovery, connection binding, and software reference calibration."""
import argparse
import math
import os
from pathlib import Path
import sys
import tempfile
import time

import serial
from serial.tools import list_ports
from fashionstar_uart_sdk import UartServoManager
import yaml

CONFIG = Path(__file__).with_name("stararm.local.yaml")


def load_config(config=CONFIG):
    arms = yaml.safe_load(config.read_text()) if config.exists() else {}
    if not isinstance(arms, dict) or any(
        not isinstance(arm, dict) or not isinstance(arm.get("port"), str)
        or not isinstance(arm.get("baudrate"), int) or arm["baudrate"] <= 0
        for arm in arms.values()
    ):
        raise ValueError(f"Invalid configuration format: {config}")
    return arms


def save_config(arms, config=CONFIG):
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", dir=config.parent, delete=False) as stream:
            temporary = Path(stream.name)
            yaml.safe_dump(arms, stream, sort_keys=False)
        os.replace(temporary, config)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def scan_port(port, baudrate):
    with serial.Serial(port, baudrate=baudrate, timeout=0, exclusive=True) as uart:
        bus = UartServoManager(uart)
        online = []
        for device_id in range(8):
            uart.reset_input_buffer()
            if bus.ping(device_id):
                online.append(device_id)
    return online


def stable_port(port):
    paths = [p for p in Path("/dev/serial/by-path").glob("*")
             if p.resolve() == Path(port).resolve()]
    return str(min(paths, key=lambda p: ("-usbv" in str(p), len(str(p))))) if paths else port


def bind(side, port, baudrate, config=CONFIG, replace=False):
    arms = load_config(config)
    if side in arms and not replace:
        raise ValueError(f"{side} is already bound; use --replace to rebind it.")
    path = stable_port(port)
    for name, arm in arms.items():
        if name != side and os.path.realpath(arm["port"]) == os.path.realpath(path):
            raise ValueError(f"This port is already bound to {name}.")
    online = scan_port(port, baudrate)
    missing = sorted(set(range(7)) - set(online))
    if missing:
        raise ValueError(f"Missing servo IDs {missing}; binding was not saved.")
    arms[side] = {"port": path, "baudrate": baudrate}
    # Rebinding deliberately drops the previous arm's software calibration.
    save_config(arms, config)
    print(f"Saved {side} -> {path}; configuration: {config}")


def read_angles(bus):
    """Require fresh, checksum-validated monitor data for all seven servos."""
    bus.uart.reset_input_buffer()
    bus.pkt_buffer.empty_buffer()
    for servo_id in range(7):
        bus.servos[servo_id].angle_monitor = None
    states = bus.send_sync_servo_monitor(list(range(7)), realtime=True)
    angles = [getattr(states.get(i), "angle_monitor", None) for i in range(7)]
    if any(x is None or not math.isfinite(x) or abs(x) > 368640 for x in angles):
        raise ValueError("Missing or invalid angle feedback; cached values are not fresh samples.")
    return angles


def angle_difference(current, reference):
    # All model joints are hinges: 360-degree branches describe the same pose.
    return [(a - b + 180) % 360 - 180 for a, b in zip(current, reference)]


def capture_pose(bus):
    import numpy as np

    samples = []
    for _ in range(10):
        samples.append(read_angles(bus))
        time.sleep(.02)
    changes = np.array([angle_difference(sample, samples[0]) for sample in samples])
    if np.max(np.ptp(changes, axis=0)) > 2:
        raise ValueError("The arm moved during capture. Hold it still and retry.")
    return (np.array(samples[0]) + np.median(changes, axis=0)).tolist()


def calibrate(side, replace=False, config=CONFIG):
    arms = load_config(config)
    if side not in arms:
        raise ValueError(f"Bind {side} before calibrating it.")
    arm = arms[side]
    if "reference_deg" in arm and not replace:
        raise ValueError("A calibration already exists; use --replace to recalibrate.")
    with serial.Serial(arm["port"], baudrate=arm["baudrate"], timeout=0,
                       exclusive=True) as uart:
        reference = capture_pose(UartServoManager(uart))
    latest = load_config(config)
    if latest.get(side, {}).get("port") != arm["port"]:
        raise ValueError("The binding changed during calibration; calibration was not saved.")
    latest[side]["reference_deg"] = reference
    latest[side].pop("directions", None)
    save_config(latest, config)
    print(f"Saved seven software reference angles for {side}: {reference}")


def main():
    parser = argparse.ArgumentParser(
        description="Star Arm setup and calibration. Hardware origins are never changed."
    )
    commands = parser.add_subparsers(dest="command", required=True)
    scan = commands.add_parser("scan", help="Query IDs 0-7; scan CH340/CH341 serial ports by default")
    scan.add_argument("--port", help="Scan only the specified serial port")
    binding = commands.add_parser("bind", help="Bind the specified port to the left or right leader arm")
    binding.add_argument("side", choices=("left", "right"))
    binding.add_argument("port")
    binding.add_argument("--replace", action="store_true")
    calibration = commands.add_parser("calibrate", help="Record the default reference pose immediately")
    calibration.add_argument("side", choices=("left", "right"))
    calibration.add_argument("--replace", action="store_true")
    for command in (scan, binding):
        command.add_argument("--baudrate", type=int, default=1_000_000)
    args = parser.parse_args()
    if getattr(args, "baudrate", 1) <= 0:
        parser.error("Baud rate must be positive.")
    try:
        if args.command == "calibrate":
            calibrate(args.side, args.replace)
            return 0
        if args.command == "bind":
            bind(args.side, args.port, args.baudrate, replace=args.replace)
            return 0
        ports = [args.port] if args.port else sorted(
            p.device for p in list_ports.comports() if p.vid == 0x1A86)
        found = False
        for port in ports:
            try:
                online = scan_port(port, args.baudrate)
                print(f"{port}: IDs {online}  path: {stable_port(port)}")
                found |= bool(online)
            except (OSError, serial.SerialException) as exc:
                print(f"{port}: {exc}", file=sys.stderr)
        if not ports:
            print("No CH340/CH341 serial ports found; use --port to specify one.")
        return 0 if found else 1
    except (OSError, ValueError, RuntimeError, EOFError, ImportError, yaml.YAMLError, serial.SerialException) as exc:
        print(str(exc), file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
