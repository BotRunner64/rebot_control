#!/usr/bin/env python3
"""Star Arm: scan online IDs and bind left/right USB connections."""
import argparse
import os
from pathlib import Path
import sys
import tempfile

import serial
from serial.tools import list_ports
from fashionstar_uart_sdk import UartServoManager
import yaml

CONFIG = Path(__file__).with_name("stararm.local.yaml")


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
    arms = yaml.safe_load(config.read_text()) if config.exists() else {}
    if not isinstance(arms, dict) or any(
        not isinstance(arm, dict) or not isinstance(arm.get("port"), str)
        for arm in arms.values()
    ):
        raise ValueError(f"Invalid configuration format: {config}")
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
    # Complete the write before replacing the previous configuration.
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", dir=config.parent, delete=False) as stream:
            temporary = Path(stream.name)
            yaml.safe_dump(arms, stream, sort_keys=False)
        os.replace(temporary, config)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
    print(f"Saved {side} -> {path}; configuration: {config}")


def main():
    parser = argparse.ArgumentParser(
        description="Scan and bind Star Arms. Only queries servos; sends no control commands."
    )
    commands = parser.add_subparsers(dest="command", required=True)
    scan = commands.add_parser("scan", help="Query IDs 0-7; scan CH340/CH341 serial ports by default")
    scan.add_argument("--port", help="Scan only the specified serial port")
    binding = commands.add_parser("bind", help="Bind the specified port to the left or right leader arm")
    binding.add_argument("side", choices=("left", "right"))
    binding.add_argument("port")
    binding.add_argument("--replace", action="store_true")
    for command in (scan, binding):
        command.add_argument("--baudrate", type=int, default=1_000_000)
    args = parser.parse_args()
    if args.baudrate <= 0:
        parser.error("Baud rate must be positive.")
    try:
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
    except (OSError, ValueError, yaml.YAMLError, serial.SerialException) as exc:
        print(str(exc), file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
