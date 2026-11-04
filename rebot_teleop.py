#!/usr/bin/env python3
"""Teleoperate physical reBot RS arms and grippers with calibrated Star Arm leaders."""

import argparse
from contextlib import ExitStack
import math
from pathlib import Path
import signal
import subprocess
import sys
import time

import can
from fashionstar_uart_sdk import UartServoManager
import numpy as np
import serial
import yaml

from rebot import CONFIG, load_config, resolve_channel, verify_identity
from rebot_hand_guiding import SETTINGS_PATH, StopKey, load_settings
from rebot_mit import LIMITS, MotorBus
from rebot_motion import MOTION_KD, MOTION_KP
from stararm import CONFIG as LEADER_CONFIG, load_config as load_leaders, read_angles
from stararm_teleop import DIRECTIONS, release_torque


# Official Seeed LeRobot RS follower limits, ordered J1-J6 and gripper.
JOINT_LIMITS_DEG = ((-145., 145.), (0., 170.), (0., 200.), (-80., 90.),
                    (-90., 90.), (-130., 130.), (0., 270.))


class TeleopStopped(Exception):
    """A keyboard or signal requested a native position hold."""


def angles(value, name):
    result = np.asarray(value, dtype=float)
    if result.shape != (7,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain seven finite angles.")
    return result


def load_bindings(config, leader_config, side):
    if side not in ("left", "right", "both"):
        raise ValueError("Select left, right, or both arms.")
    sides = ("left", "right") if side == "both" else (side,)
    arms, leaders = load_config(config), load_leaders(leader_config)
    channels, ports, uids = {}, set(), set()
    for name in sides:
        if name not in arms or "reference_rad" not in arms[name]:
            raise ValueError(f"Bind and calibrate the {name} reBot arm first.")
        if name not in leaders or "reference_deg" not in leaders[name]:
            raise ValueError(f"Bind and calibrate the {name} Star Arm first.")
        angles(arms[name]["reference_rad"], f"{name} follower reference")
        angles(leaders[name]["reference_deg"], f"{name} leader reference")
        channels[name] = resolve_channel(arms[name])
        port = Path(leaders[name]["port"]).resolve()
        current_uids = set(arms[name]["motor_uids"].values())
        if port in ports or uids & current_uids:
            raise ValueError("Selected arms must use distinct leader ports and follower motor UIDs.")
        ports.add(port)
        uids.update(current_uids)
    if len(set(channels.values())) != len(sides):
        raise ValueError("Selected followers must use distinct CAN interfaces.")
    return sides, arms, leaders, channels


def map_target(raw, reference, lower, upper):
    # Preserve SDK multi-turn angles; wrapping at +/-180 degrees would jump.
    relative = np.radians(angles(raw, "Leader feedback") - angles(reference, "Leader reference"))
    target = relative * np.array([*DIRECTIONS, 6.])
    return np.clip(target[:len(lower)], lower, upper)


def teleop(config=CONFIG, leader_config=LEADER_CONFIG, settings_path=SETTINGS_PATH,
           side="both", gripper_open_deg=270., gripper=True):
    if not math.isfinite(gripper_open_deg) or gripper_open_deg <= 0:
        raise ValueError("Gripper opening must be finite and positive.")
    if gripper_open_deg > JOINT_LIMITS_DEG[6][1]:
        raise ValueError("Gripper opening exceeds the official 270-degree limit.")
    cfg = load_settings(settings_path)
    sides, arms, leaders, channels = load_bindings(config, leader_config, side)
    count = 7 if gripper else 6
    kp = np.append(MOTION_KP, 50.)[:count]
    kd = np.append(MOTION_KD, 4.)[:count]
    torques = np.zeros(count)
    velocities = np.zeros(count)
    joint_bounds = np.radians(JOINT_LIMITS_DEG[:count])
    if gripper:
        joint_bounds[6, 1] = math.radians(gripper_open_deg)
    interrupted = False

    def stop(signum, frame):
        nonlocal interrupted
        interrupted = True

    old_signals = {sig: signal.signal(sig, stop)
                   for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)}
    try:
        with ExitStack() as stack:
            keys = stack.enter_context(StopKey())

            def check_stop():
                if interrupted or keys.pressed():
                    raise TeleopStopped()

            buses, leader_buses, references, bounds = {}, {}, {}, {}

            def validate_pose(name, raw):
                raw = np.asarray(raw, dtype=float)
                if raw.shape != (count,) or not np.all(np.isfinite(raw)):
                    raise ValueError(f"{name}: expected {count} finite follower angles.")
                q = raw - references[name]
                low, high = bounds[name]
                if (np.any(low > high) or np.any(q < low - .03) or np.any(q > high + .03)):
                    raise ValueError(f"{name}: follower pose is outside teleoperation limits; check calibration.")

            # Validate every selected leader/follower before enabling any arm.
            for name in sides:
                check_stop()
                leader = leaders[name]
                uart = stack.enter_context(serial.Serial(leader["port"], baudrate=leader["baudrate"],
                                                        timeout=0, exclusive=True))
                leader_buses[name] = UartServoManager(uart)
                read_angles(leader_buses[name])
                bus = buses[name] = stack.enter_context(MotorBus(
                    channels[name], arms[name]["models"], max_temperature=cfg["max_temperature_c"],
                    motor_count=count))
                verify_identity(bus, arms[name])
                reference = references[name] = np.asarray(arms[name]["reference_rad"][:count])
                raw = np.asarray(bus.positions())
                low, high = joint_bounds[:, 0], joint_bounds[:, 1]
                native = np.array([LIMITS[m].position for m in arms[name]["models"][:count]])
                low, high = np.maximum(low, -native - reference), np.minimum(high, native - reference)
                bounds[name] = low, high
                validate_pose(name, raw)
                bus.set_hold(raw, torques, kp, kd)
                print(f"{name}: verified leader and seven follower UIDs on {channels[name]}.", flush=True)
            check_stop()
            print("Releasing Star Arm leaders. Support the leaders before starting.", flush=True)
            for name in sides:
                release_torque(leader_buses[name])
                buses[name].prepare()
            for name in sides:
                check_stop()
                bus = buses[name]
                raw = np.asarray(bus.positions())
                validate_pose(name, raw)
                bus.set_hold(raw, torques, kp, kd)
                bus.enable(raw)
            started = time.monotonic()
            last_sample = dict.fromkeys(sides, started)
            frames = 0
            print(f"PHYSICAL TELEOP: {', '.join(sides)}; gripper {'enabled' if gripper else 'unchanged'}; "
                  f"requested {cfg['rate_hz']:g} Hz; gravity compensation off. "
                  "Space/Esc/Q/Ctrl-C stops and holds; followers stay enabled.", flush=True)
            print("Mapped leader positions are sent directly after joint-limit clipping; MIT target velocity is zero. "
                  "Joint-space following does not provide collision avoidance.", flush=True)
            try:
                while True:
                    tick = time.monotonic()
                    check_stop()
                    # Never stream old targets after a partial or slow leader read.
                    leader_started = time.monotonic()
                    leader_raw = {name: read_angles(leader_buses[name]) for name in sides}
                    if time.monotonic() - leader_started > cfg["max_feedback_age_s"]:
                        raise TimeoutError("Star Arm feedback is too old; stopping both followers.")
                    check_stop()
                    records = {}
                    for name in sides:
                        sampled = time.monotonic()
                        raw = np.asarray(buses[name].positions())
                        validate_pose(name, raw)
                        now = time.monotonic()
                        if not 0 < now - last_sample[name] <= cfg["max_sample_gap_s"]:
                            raise RuntimeError("Teleoperation feedback gap exceeded the control deadline.")
                        last_sample[name] = now
                        target = map_target(leader_raw[name], leaders[name]["reference_deg"], *bounds[name])
                        buses[name].set_hold(raw, torques, kp, kd)
                        deadline = min(sampled + cfg["max_feedback_age_s"],
                                       leader_started + cfg["max_feedback_age_s"])
                        records[name] = (references[name] + target, deadline)
                    # Check every arm before sending either arm's new target.
                    check_stop()
                    if time.monotonic() >= min(record[1] for record in records.values()):
                        raise TimeoutError("Paired teleoperation feedback deadline expired.")
                    for name in sides:
                        check_stop()
                        target, deadline = records[name]
                        buses[name].command_position(target, velocities, torques, kp, kd, deadline)
                    frames += 1
                    time.sleep(max(0., 1 / cfg["rate_hz"] - (time.monotonic() - tick)))
            finally:
                elapsed = time.monotonic() - started
                print(f"Teleoperation stopped: {frames} paired commands in {elapsed:.1f}s "
                      f"({frames / max(elapsed, 1e-9):.1f} Hz). Installing native position hold.", flush=True)
    except TeleopStopped:
        print("Stop requested. See native position hold confirmations above.", flush=True)
    finally:
        for sig, handler in old_signals.items():
            signal.signal(sig, handler)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=CONFIG, help="Follower bindings YAML")
    parser.add_argument("--leader-config", type=Path, default=LEADER_CONFIG)
    parser.add_argument("--settings", type=Path, default=SETTINGS_PATH)
    parser.add_argument("--side", choices=("left", "right", "both"), default="both")
    parser.add_argument("--gripper-open-deg", type=float, default=270.,
                        help="Gripper motor angle from closed to open (maximum: 270 degrees)")
    parser.add_argument("--no-gripper", action="store_true", help="Leave motor 7 control unchanged")
    args = parser.parse_args()
    try:
        teleop(config=args.config, leader_config=args.leader_config, settings_path=args.settings,
               side=args.side, gripper_open_deg=args.gripper_open_deg, gripper=not args.no_gripper)
        return 0
    except (OSError, ValueError, RuntimeError, ImportError, can.CanError, yaml.YAMLError,
            subprocess.SubprocessError) as exc:
        print(str(exc), file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
