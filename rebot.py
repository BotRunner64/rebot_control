#!/usr/bin/env python3
"""Read-only B601-RS discovery, binding, and software reference calibration."""

import argparse
from contextlib import contextmanager
import json
import math
import os
from pathlib import Path
import statistics
import struct
import subprocess
import sys
import tempfile
import time

import can
from robstride_dynamics import RobstrideBus, CommunicationType, ParameterType
import yaml

CONFIG = Path(__file__).with_name("rebot.local.yaml")
NET = Path("/sys/class/net")
MOTOR_IDS = tuple(range(1, 8))
MODELS = ["rs-06"] * 3 + ["rs-00"] * 4
BITRATE = 1_000_000
HOST_ID = 0xFD
DEVICE_ID_REPLY_ADDRESS = 0xFE


def load_config(config=CONFIG):
    arms = yaml.safe_load(config.read_text()) if config.exists() else {}
    if not isinstance(arms, dict):
        raise ValueError(f"Invalid configuration: {config}")
    for side, arm in arms.items():
        if (side not in ("left", "right") or not isinstance(arm, dict)
                or not isinstance(arm.get("channel"), str)
                or not isinstance(arm.get("device_path"), str)
                or arm.get("bitrate") != BITRATE or arm.get("host_id") != HOST_ID
                or arm.get("motor_ids") != list(MOTOR_IDS) or arm.get("models") != MODELS
                or not isinstance(arm.get("motor_uids"), dict)
                or set(arm["motor_uids"]) != set(MOTOR_IDS)):
            raise ValueError(f"Invalid {side} binding: {config}")
        uids = list(arm["motor_uids"].values())
        if (any(not isinstance(uid, str) or len(uid) != 16
                or any(c not in "0123456789abcdef" for c in uid) for uid in uids)
                or len(set(uids)) != 7):
            raise ValueError(f"Invalid {side} motor UIDs: {config}")
        if "reference_rad" in arm:
            reference = arm["reference_rad"]
            if (not isinstance(reference, list) or len(reference) != 7
                    or any(type(x) not in (int, float) or not math.isfinite(x)
                           for x in reference)):
                raise ValueError(f"Invalid {side} software reference: {config}")
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


def channels():
    return sorted(p.name for p in NET.iterdir()
                  if (p / "type").read_text().strip() == "280")


def device_path(channel):
    path = NET / channel / "device"
    return str(path.resolve()) if path.exists() else ""


def resolve_channel(arm):
    if not arm["device_path"]:
        return arm["channel"]
    matches = [name for name in channels() if device_path(name) == arm["device_path"]]
    if len(matches) != 1:
        raise ValueError("Bound CAN adapter is missing or ambiguous; check its USB connection.")
    return matches[0]


def check_channel(channel, configure=False):
    if channel not in channels():
        raise ValueError(f"No native CAN interface named {channel}.")
    result = subprocess.run(["ip", "-details", "-json", "link", "show", "dev", channel],
                            check=True, capture_output=True, text=True)
    entry, = json.loads(result.stdout)
    info = entry.get("linkinfo", {})
    data = info.get("info_data", {})
    if ("UP" not in entry.get("flags", []) or info.get("info_kind") != "can"
            or data.get("bittiming", {}).get("bitrate") != BITRATE
            or data.get("state") in ("BUS-OFF", "STOPPED")):
        if configure:
            print(f"Configuring {channel} at {BITRATE} bit/s...", flush=True)
            prefix = [] if os.geteuid() == 0 else ["sudo"]
            try:
                for options in (["down"], ["type", "can", "bitrate", str(BITRATE)], ["up"]):
                    subprocess.run(prefix + ["ip", "link", "set", channel] + options, check=True)
            except subprocess.CalledProcessError as exc:
                raise RuntimeError(
                    f"Could not configure {channel} (exit {exc.returncode}). "
                    "Run scan in a terminal with sudo access, then retry.") from exc
            # Verify the kernel accepted the configuration before opening CAN.
            check_channel(channel)
            return
        raise ValueError(
            f"{channel} must be UP at {BITRATE} bit/s. Configure it before scanning:\n"
            f"  sudo ip link set {channel} down\n"
            f"  sudo ip link set {channel} type can bitrate {BITRATE}\n"
            f"  sudo ip link set {channel} up")


class Reader:
    """Use SDK transmit framing with bounded, correlated parameter responses.

    The installed SDK's read helper accepts the next frame without checking its
    motor, host, or parameter and has no finite read timeout. Keep reception local.
    """

    def __init__(self, bus, timeout=0.3):
        self.bus = bus
        self.timeout = timeout
        self.received_frames = 0

    def query(self, kind, motor_id, payload=bytes(8), parameter=None):
        socket = self.bus.channel_handler
        # Discard queued traffic before issuing a new transaction.
        for _ in range(2048):
            if socket.recv(timeout=0) is None:
                break
            self.received_frames += 1
        else:
            raise RuntimeError("CAN receive queue overloaded.")
        sent_at = time.time()
        self.bus.transmit(kind, HOST_ID, motor_id, payload)
        deadline = time.monotonic() + self.timeout
        while (remaining := deadline - time.monotonic()) > 0:
            frame = socket.recv(timeout=remaining)
            if frame is None:
                break
            self.received_frames += 1
            if (not frame.is_extended_id or frame.is_error_frame or frame.is_remote_frame
                    or len(frame.data) != 8 or not math.isfinite(frame.timestamp)
                    or frame.timestamp < sent_at - 0.001):
                continue
            arb = frame.arbitration_id
            response_kind, source, host = (arb >> 24) & 31, (arb >> 8) & 255, arb & 255
            # Type 0 replies use the fixed destination 0xFE, rather than the
            # requesting host ID used by parameter/status/fault responses.
            expected_host = (DEVICE_ID_REPLY_ADDRESS
                             if response_kind == CommunicationType.GET_DEVICE_ID else HOST_ID)
            if source != motor_id or host != expected_host:
                continue
            if response_kind == CommunicationType.FAULT_REPORT and any(frame.data):
                raise RuntimeError(f"Motor {motor_id}: fault/warning {bytes(frame.data).hex()}")
            if response_kind in (CommunicationType.OPERATION_STATUS, CommunicationType.ACTIVE_REPORT):
                faults = (arb >> 16) & 63
                if faults:
                    raise RuntimeError(f"Motor {motor_id}: fault bits 0x{faults:02x}")
            if response_kind != kind:
                continue
            if parameter is not None and struct.unpack_from("<H", frame.data)[0] != parameter:
                continue
            return bytes(frame.data)
        raise TimeoutError(f"Motor {motor_id}: no matching response for type {kind}.")

    def scan(self):
        found = {}
        for motor_id in MOTOR_IDS:
            try:
                found[motor_id] = self.query(CommunicationType.GET_DEVICE_ID, motor_id).hex()
            except TimeoutError:
                continue
        return found

    def positions(self):
        return [self.position(motor_id) for motor_id in MOTOR_IDS]

    def position(self, motor_id):
        if motor_id not in MOTOR_IDS:
            raise ValueError("Expected a motor ID between 1 and 7.")
        parameter = ParameterType.MECHANICAL_POSITION[0]
        data = self.query(CommunicationType.READ_PARAMETER, motor_id,
                          struct.pack("<HHI", parameter, 0, 0), parameter)
        value, = struct.unpack_from("<f", data, 4)
        if not math.isfinite(value):
            raise ValueError(f"Motor {motor_id}: invalid position feedback.")
        return value


@contextmanager
def open_reader(channel):
    check_channel(channel)
    bus = RobstrideBus(channel, {}, bitrate=BITRATE)
    bus.host_id = HOST_ID
    try:
        bus.connect(handshake=False)
        yield Reader(bus)
    finally:
        if bus.is_connected:
            bus.disconnect(disable_torque=False)


def verify_identity(reader, arm):
    if reader.scan() != arm["motor_uids"]:
        raise ValueError("Motor UIDs or online IDs changed; check the arm and binding.")


def bind(side, channel, replace=False, config=CONFIG):
    arms = load_config(config)
    if side in arms and not replace:
        raise ValueError(f"{side} is already bound; use --replace to rebind it.")
    with open_reader(channel) as reader:
        found = reader.scan()
        missing = sorted(set(MOTOR_IDS) - set(found))
        if missing:
            raise ValueError(f"Missing motor IDs {missing}; binding was not saved.")
        if len(set(found.values())) != 7:
            raise ValueError("Motor UIDs are not unique; binding was not saved.")
        reader.positions()
    path = device_path(channel)
    for name, other in arms.items():
        same_adapter = (path == other["device_path"] if path and other["device_path"]
                        else channel == other["channel"])
        if name != side and (same_adapter
                             or set(found.values()) & set(other["motor_uids"].values())):
            raise ValueError(f"This adapter or arm is already bound to {name}.")
    arms[side] = dict(channel=channel, device_path=path, bitrate=BITRATE,
                      host_id=HOST_ID, motor_ids=list(MOTOR_IDS), models=MODELS,
                      motor_uids=found)
    save_config(arms, config)
    print(f"Saved {side} -> {channel}; configuration: {config}")


def capture_pose(reader):
    samples = []
    for _ in range(10):
        samples.append(reader.positions())
        time.sleep(0.02)
    # Preserve the continuous motor angle; do not fold it into +/- pi.
    axes = list(zip(*samples))
    if any(max(axis) - min(axis) > math.radians(2) for axis in axes):
        raise ValueError("The arm moved during capture. Hold it still and retry.")
    return [statistics.median(axis) for axis in axes]


def bound_arm(side, config):
    arms = load_config(config)
    if side not in arms:
        raise ValueError(f"Bind {side} first.")
    return arms[side]


def capture_gripper(reader):
    samples = []
    for _ in range(10):
        samples.append(reader.position(7))
        time.sleep(.02)
    if max(samples) - min(samples) > math.radians(2):
        raise ValueError("The gripper moved during capture. Keep it fully closed and retry.")
    return statistics.median(samples)


def calibrate(side, replace=False, config=CONFIG, gripper_only=False):
    arm = bound_arm(side, config)
    if gripper_only and "reference_rad" not in arm:
        raise ValueError("Calibrate the complete arm before updating only the gripper reference.")
    if "reference_rad" in arm and not replace:
        raise ValueError("A calibration already exists; use --replace to recalibrate.")
    with open_reader(resolve_channel(arm)) as reader:
        verify_identity(reader, arm)
        if gripper_only:
            reference = list(arm["reference_rad"])
            reference[6] = capture_gripper(reader)
        else:
            reference = capture_pose(reader)
    latest = load_config(config)
    if latest.get(side) != arm:
        raise ValueError("The binding changed during calibration; reference was not saved.")
    latest[side]["reference_rad"] = reference
    save_config(latest, config)
    if gripper_only:
        print(f"Saved {side} gripper closed reference: "
              f"{math.degrees(arm['reference_rad'][6]):+.2f} -> {math.degrees(reference[6]):+.2f} deg "
              f"({reference[6]:+.9f} rad). J1-J6 software references are unchanged.")
    else:
        print(f"Saved seven software reference angles (rad) for {side}: {reference}")


def read(side, watch=False, config=CONFIG):
    arm = bound_arm(side, config)
    with open_reader(resolve_channel(arm)) as reader:
        verify_identity(reader, arm)
        print("Order: joint1 joint2 joint3 joint4 joint5 joint6 gripper; unit: deg")
        while True:
            values = reader.positions()
            print("raw: " + " ".join(f"{math.degrees(x):+.2f}" for x in values), flush=True)
            if "reference_rad" in arm:
                relative = [a - b for a, b in zip(values, arm["reference_rad"])]
                print("relative: " + " ".join(f"{math.degrees(x):+.2f}" for x in relative), flush=True)
            if not watch:
                break
            time.sleep(0.1)


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
        result = subprocess.run(
            ["ip", "-details", "-statistics", "-json", "link", "show", "dev", channel],
            check=True, capture_output=True, text=True)
        entry, = json.loads(result.stdout)
        data = entry.get("linkinfo", {}).get("info_data", {})
        stats = entry.get("stats64", entry.get("stats", {}))
        errors = data.get("berr_counter", {})
        print(f"  CAN state: {data.get('state', 'unknown')}; "
              f"total RX packets: {stats.get('rx', {}).get('packets', 'unknown')}; "
              f"total TX packets: {stats.get('tx', {}).get('packets', 'unknown')}; "
              f"error counters: tx={errors.get('tx', 'unknown')} rx={errors.get('rx', 'unknown')}",
              file=sys.stderr)
    except (OSError, ValueError, subprocess.SubprocessError) as exc:
        print(f"  Could not read CAN statistics: {exc}", file=sys.stderr)


def main():
    parser = argparse.ArgumentParser(
        description="B601-RS read-only setup. No enable, disable, motion, or hardware-zero writes.")
    parser.add_argument("--config", type=Path, default=CONFIG)
    commands = parser.add_subparsers(dest="command", required=True)
    scan = commands.add_parser("scan", help="Configure native CAN at 1 Mbps and query IDs 1-7")
    scan.add_argument("--channel")
    binding = commands.add_parser("bind", help="Bind an adapter and motor UIDs to an arm")
    binding.add_argument("side", choices=("left", "right"))
    binding.add_argument("channel")
    binding.add_argument("--replace", action="store_true")
    calibration = commands.add_parser("calibrate", help="Capture the software reference pose immediately")
    calibration.add_argument("side", choices=("left", "right"))
    calibration.add_argument("--replace", action="store_true")
    calibration.add_argument("--gripper-only", action="store_true",
                             help="Record only the fully closed gripper reference, preserving J1-J6")
    reading = commands.add_parser("read", help="Read raw and relative motor angles")
    reading.add_argument("side", choices=("left", "right"))
    reading.add_argument("--watch", action="store_true")
    args = parser.parse_args()
    try:
        if args.command == "bind":
            bind(args.side, args.channel, args.replace, args.config)
        elif args.command == "calibrate":
            calibrate(args.side, args.replace, args.config, gripper_only=args.gripper_only)
        elif args.command == "read":
            read(args.side, args.watch, args.config)
        else:
            ports = [args.channel] if args.channel else channels()
            found = False
            for channel in ports:
                try:
                    check_channel(channel, configure=True)
                    with open_reader(channel) as reader:
                        motors = reader.scan()
                    print(f"{channel}: IDs {list(motors)}  USB: {device_path(channel)}")
                    for motor_id, uid in motors.items():
                        print(f"  ID {motor_id}: UID {uid}")
                    if not motors:
                        report_empty_scan(channel, reader.received_frames)
                    found |= bool(motors)
                except (OSError, ValueError, RuntimeError, can.CanError, subprocess.SubprocessError) as exc:
                    print(f"{channel}: {exc}", file=sys.stderr)
            if not ports:
                print("No native CAN interfaces found. Connect the CAN adapter first.")
            return 0 if found else 1
        return 0
    except (OSError, ValueError, RuntimeError, can.CanError, yaml.YAMLError,
            subprocess.SubprocessError) as exc:
        print(str(exc), file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
