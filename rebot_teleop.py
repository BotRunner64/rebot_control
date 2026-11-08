#!/usr/bin/env python3
"""Teleoperate physical reBot RS arms and grippers with calibrated Star Arm leaders."""

import argparse
import math
import multiprocessing as mp
import queue
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
from rebot_teleop_runtime import (MOTOR_HZ, TARGET_HZ, SharedFrame, LoopTiming, child_signals,
                                 fresh, stop_on_error, supervisor_alive, wait_phase)


# Official Seeed LeRobot RS follower limits, ordered J1-J6 and gripper.
JOINT_LIMITS_DEG = ((-145., 145.), (0., 170.), (0., 200.), (-80., 90.),
                    (-90., 90.), (-130., 130.), (0., 270.))
STATE_LIMIT_TOLERANCE_RAD = 0.1


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


class StarArmSource:
    """Open the serial connection only inside its owning acquisition process."""

    def __init__(self, arm):
        self.arm = arm

    def __enter__(self):
        self.uart = serial.Serial(self.arm["port"], baudrate=self.arm["baudrate"],
                                  timeout=0, exclusive=True)
        try:
            self.bus = UartServoManager(self.uart)
        except BaseException:
            self.uart.close()
            raise
        return self

    def read(self):
        return read_angles(self.bus)

    def release(self):
        release_torque(self.bus)

    def __exit__(self, *args):
        self.uart.close()


def position_bounds(arm, count, gripper_open_deg):
    reference = np.asarray(arm["reference_rad"][:count])
    limits = np.radians(JOINT_LIMITS_DEG[:count])
    if count == 7:
        limits[6, 1] = math.radians(gripper_open_deg)
    native = np.array([LIMITS[m].position for m in arm["models"][:count]])
    low = np.maximum(limits[:, 0], -native - reference)
    high = np.minimum(limits[:, 1], native - reference)
    if np.any(low > high):
        raise ValueError("Software references leave no encodable joint range.")
    return reference, low, high


def validate_pose(raw, reference, low, high):
    raw = np.asarray(raw, dtype=float)
    if raw.shape != reference.shape or not np.all(np.isfinite(raw)):
        raise ValueError("Expected finite feedback for every selected follower motor.")
    # Check arm travel only; gripper feedback still must be finite above.
    q = (raw - reference)[:6]
    invalid = np.flatnonzero((q < low[:6] - STATE_LIMIT_TOLERANCE_RAD)
                             | (q > high[:6] + STATE_LIMIT_TOLERANCE_RAD))
    if len(invalid):
        details = []
        for index in invalid:
            details.append(
                f"J{index + 1}: relative={math.degrees(q[index]):.2f} deg, "
                f"limits=[{math.degrees(low[index]):.2f}, {math.degrees(high[index]):.2f}] deg, "
                f"raw={math.degrees(raw[index]):.2f} deg, "
                f"reference={math.degrees(reference[index]):.2f} deg")
        raise ValueError("Follower pose is outside teleoperation limits: " + "; ".join(details)
                         + f"; tolerance={math.degrees(STATE_LIMIT_TOLERANCE_RAD):.2f} deg. "
                         "Check the physical pose and software reference.")
    return raw


def leader_worker(name, arm, frame, phases, ready, prepared, stop, heartbeat, reports, source_factory):
    with source_factory(arm) as source, stop_on_error(stop):
        timing = LoopTiming(name)
        released = False
        try:
            while not stop.is_set():
                supervisor_alive(heartbeat)
                if phases["prepare"].is_set() and not released:
                    source.release()
                    released = True
                    prepared.set()
                began = timing.tick()
                values = angles(source.read(), "Leader feedback")
                frame.publish(values, began)
                ready.set()
                timing.complete(began, began)
                # Acquisition follows the real serial rate, not a claimed 500 Hz.
        finally:
            reports.put(timing.report())


def target_worker(sides, leaders, arms, count, gripper_open_deg, sources, targets,
                  phases, ready, stop, heartbeat, reports, cfg):
    bounds = {name: position_bounds(arms[name], count, gripper_open_deg) for name in sides}
    ready.set()
    if not wait_phase(phases["run"], stop, heartbeat):
        return
    timing = LoopTiming("target", TARGET_HZ)
    try:
        while not stop.is_set():
            began = timing.tick()
            supervisor_alive(heartbeat)
            values, ages = [], []
            for name in sides:
                raw, source_at = fresh(sources[name], time.monotonic(), cfg["max_feedback_age_s"], name)
                reference, low, high = bounds[name]
                values.extend(reference + map_target(raw, leaders[name]["reference_deg"], low, high))
                ages.append(source_at)
            if not stop.is_set():
                # Publish the complete pair atomically and preserve acquisition age.
                targets.publish(values, min(ages))
                timing.complete(began, min(ages))
            timing.pace(stop)
    finally:
        reports.put(timing.report())


def motor_worker(name, index, arm, channel, count, gripper_open_deg, targets, phases,
                 ready, prepared, enabled, stop, heartbeat, reports, cfg, bus_factory):
    reference, low, high = position_bounds(arm, count, gripper_open_deg)
    kp, kd = np.append(MOTION_KP, 50.)[:count], np.append(MOTION_KD, 4.)[:count]
    zero = np.zeros(count)
    with bus_factory(channel, arm["models"], max_temperature=cfg["max_temperature_c"],
                     motor_count=count) as bus, stop_on_error(stop):
        verify_identity(bus, arm)
        raw = validate_pose(bus.positions(), reference, low, high)
        bus.set_hold(raw, zero, kp, kd)
        ready.set()
        if not wait_phase(phases["prepare"], stop, heartbeat):
            return
        bus.prepare()
        prepared.set()
        if not wait_phase(phases["enable"], stop, heartbeat):
            return
        raw = validate_pose(bus.positions(), reference, low, high)
        bus.set_hold(raw, zero, kp, kd)
        if stop.is_set():
            return
        bus.enable(raw)
        enabled.set()
        if not wait_phase(phases["run"], stop, heartbeat):
            return
        began = time.monotonic()
        # The mapper may not have published its first pair yet; no motion until it does.
        while not stop.is_set() and targets.read()[1] == 0:
            supervisor_alive(heartbeat)
            if time.monotonic() - began > cfg["max_feedback_age_s"]:
                raise TimeoutError("No initial teleoperation target arrived.")
            stop.wait(.001)
        timing = LoopTiming(name, MOTOR_HZ)
        previous = time.monotonic()
        try:
            while not stop.is_set():
                began = timing.tick()
                supervisor_alive(heartbeat)
                if began - previous > cfg["max_sample_gap_s"]:
                    raise TimeoutError("Motor control feedback gap exceeded the deadline.")
                previous = began
                pair, source_at = fresh(targets, time.monotonic(), cfg["max_feedback_age_s"], "Target pair")
                target = pair[index * count:(index + 1) * count]
                # Validate the consumer boundary as well as the mapper output.
                if np.any(target < reference + low) or np.any(target > reference + high):
                    raise ValueError("Shared target exceeds the configured motor range.")
                deadline = min(began + cfg["max_feedback_age_s"], source_at + cfg["max_feedback_age_s"])
                if stop.is_set():
                    break
                bus.command_position(target, zero, zero, kp, kd, deadline)
                # Each MIT command already returns position; avoid seven extra queries.
                raw = validate_pose(bus.feedback_positions(cfg["max_feedback_age_s"]), reference, low, high)
                bus.set_hold(raw, zero, kp, kd)
                timing.complete(began, source_at)
                timing.pace(stop)
        finally:
            reports.put(timing.report())


def worker_entry(name, worker, args, stop, reports):
    child_signals(stop)
    try:
        worker(*args)
    except BaseException as exc:
        stop.set()
        reports.put({"type": "error", "worker": name, "error": f"{type(exc).__name__}: {exc}"})
        raise SystemExit(1)


def teleop(config=CONFIG, leader_config=LEADER_CONFIG, settings_path=SETTINGS_PATH,
           side="both", gripper_open_deg=270., gripper=True, *,
           source_factory=StarArmSource, bus_factory=MotorBus, startup_timeout=15., shutdown_timeout=5.):
    if not math.isfinite(gripper_open_deg) or not 0 < gripper_open_deg <= 270:
        raise ValueError("Gripper opening must be positive and at most the official 270-degree limit.")
    if any(not math.isfinite(value) or value <= 0 for value in (startup_timeout, shutdown_timeout)):
        raise ValueError("Worker startup and shutdown timeouts must be finite and positive.")
    cfg = load_settings(settings_path)
    sides, arms, leaders, channels = load_bindings(config, leader_config, side)
    count = 7 if gripper else 6
    for name in sides:
        position_bounds(arms[name], count, gripper_open_deg)
    context = mp.get_context("spawn")
    stop = context.Event()
    heartbeat = SharedFrame(context, 0)
    heartbeat.publish([], time.monotonic())
    sources = {name: SharedFrame(context, 7) for name in sides}
    targets = SharedFrame(context, count * len(sides))
    phases = {name: context.Event() for name in ("prepare", "enable", "run")}
    reports = context.Queue()
    workers, ready, prepared, enabled = [], [], [], []
    messages = []

    def add(name, worker, args):
        workers.append(context.Process(name=name, target=worker_entry, args=(name, worker, args, stop, reports)))

    for index, name in enumerate(sides):
        source_ready, source_prepared = context.Event(), context.Event()
        motor_ready, motor_prepared, motor_enabled = context.Event(), context.Event(), context.Event()
        ready.extend((source_ready, motor_ready))
        prepared.extend((source_prepared, motor_prepared))
        enabled.append(motor_enabled)
        add(f"leader-{name}", leader_worker, (f"leader-{name}", leaders[name], sources[name], phases,
            source_ready, source_prepared, stop, heartbeat, reports, source_factory))
        add(f"motor-{name}", motor_worker, (f"motor-{name}", index, arms[name], channels[name], count,
            gripper_open_deg, targets, phases, motor_ready, motor_prepared, motor_enabled,
            stop, heartbeat, reports, cfg, bus_factory))
    mapper_ready = context.Event()
    ready.append(mapper_ready)
    add("target", target_worker, (sides, leaders, arms, count, gripper_open_deg, sources, targets,
                                  phases, mapper_ready, stop, heartbeat, reports, cfg))

    def collect():
        while True:
            try:
                record = reports.get_nowait()
            except queue.Empty:
                return
            messages.append(record)
            if record["type"] == "error":
                print(f"{record['worker']}: {record['error']}", file=sys.stderr, flush=True)

    def monitor(keys):
        heartbeat.publish([], time.monotonic())
        collect()
        if keys.pressed():
            stop.set()
        for process in workers:
            if process.pid is not None and process.exitcode is not None and not stop.is_set():
                messages.append({"type": "error", "worker": process.name,
                                 "error": f"Worker exited unexpectedly ({process.exitcode})."})
                stop.set()

    def wait_ready(events, keys):
        deadline = time.monotonic() + startup_timeout
        while not stop.is_set():
            monitor(keys)
            if stop.is_set():
                return False
            if all(event.is_set() for event in events):
                return True
            if time.monotonic() > deadline:
                raise TimeoutError("Teleoperation startup phase timed out.")
            stop.wait(.01)
        return False

    old_signals = {sig: signal.signal(sig, lambda signum, frame: stop.set())
                   for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)}
    started_workers = []
    try:
        with StopKey() as keys:
            try:
                print(f"PHYSICAL TELEOP multiprocessing: motor {MOTOR_HZ} Hz per arm; target {TARGET_HZ} Hz; "
                      "gravity compensation off, MIT target velocity zero. Support the Star Arm leaders. "
                      "Space/Esc/Q/Ctrl-C stops and holds.", flush=True)
                for process in workers:
                    process.start()
                    started_workers.append(process)
                if wait_ready(ready, keys):
                    phases["prepare"].set()
                    if wait_ready(prepared, keys):
                        phases["enable"].set()
                        if wait_ready(enabled, keys):
                            phases["run"].set()
                            while not stop.is_set():
                                monitor(keys)
                                stop.wait(.01)
            finally:
                stop.set()
                deadline = time.monotonic() + shutdown_timeout
                while any(process.is_alive() for process in started_workers) and time.monotonic() < deadline:
                    try:
                        heartbeat.publish([], time.monotonic())
                    except TimeoutError:
                        # A dead process may have held this lock. Still join peers.
                        pass
                    collect()
                    for process in started_workers:
                        process.join(.01)
                for process in started_workers:
                    if process.is_alive():
                        messages.append({"type": "error", "worker": process.name,
                                         "error": "Hold cleanup timed out; native hold was not confirmed."})
                        process.kill()
                    process.join()
                collect()
                for process in started_workers:
                    if process.exitcode != 0 and not any(
                            r["type"] == "error" and r["worker"] == process.name for r in messages):
                        messages.append({"type": "error", "worker": process.name,
                                         "error": f"Worker exited unexpectedly ({process.exitcode})."})
    finally:
        for sig, handler in old_signals.items():
            signal.signal(sig, handler)
        reports.close()
        reports.join_thread()
    for record in messages:
        if record["type"] == "timing":
            print(f"{record['worker']}: {record['hz']:.1f} Hz; max gap {record['max_gap_ms']:.1f} ms; "
                  f"max work {record['max_work_ms']:.1f} ms; max source age {record['max_source_age_ms']:.1f} ms; "
                  f"skipped ticks {record['skipped']}.", flush=True)
    errors = [f"{r['worker']}: {r['error']}" for r in messages if r["type"] == "error"]
    if errors:
        raise RuntimeError("Teleoperation stopped after worker failure: " + "; ".join(errors))
    return messages


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
