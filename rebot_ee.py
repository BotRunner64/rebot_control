#!/usr/bin/env python3
"""Control one reBot RS end-effector frame with Mink IK and native MIT PD."""

import argparse
from contextlib import contextmanager, nullcontext
from dataclasses import dataclass
import math
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

import can
import numpy as np
import yaml

from rebot import CONFIG, load_config, resolve_channel, verify_identity
from rebot_hand_guiding import SETTINGS_PATH, StopKey, load_settings
from rebot_gravity import GravityModel
from rebot_kinematics import CartesianIK, EE_FRAME, SIM_START, pose_matrix, vector6
from rebot_mit import MotorBus
from rebot_motion import MOTION_KD, MOTION_KP
from rebot_teleop import position_bounds, validate_pose
from rebot_teleop_runtime import LoopTiming
from rebot_ee_runtime import GravityFeedforward, MotorWorker, request_stop


@dataclass
class Result:
    position_error_m: float
    orientation_error_rad: float
    reached: bool
    interrupted: bool


@contextmanager
def stop_signals(stop):
    def received(signum, frame):
        request_stop(stop, f"received {signal.Signals(signum).name}")

    previous = {sig: signal.signal(sig, received)
                for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)}
    try:
        yield
    finally:
        for sig, handler in previous.items():
            signal.signal(sig, handler)
        if stop.is_set():
            print(f"EE stop reason: {getattr(stop, 'stop_reason', 'external stop event')}", flush=True)


def target_pose(ik, pose, offset):
    if pose is not None:
        return pose_matrix(pose)
    target = ik.pose()
    if offset is not None:
        target[:3, 3] += offset
    return target


def track(ik, target, stop, keys, cfg, *, duration=None, viewer=None, bus=None,
          reference=None, gravity=None):
    """Publish targets at 50 Hz; an independent worker continuously services CAN."""
    timing = LoopTiming("ee-target", 50)
    started = time.monotonic()
    actual = ik.q
    motor = (MotorWorker(bus, reference, ik.lower, ik.upper, cfg, stop, gravity)
             if bus is not None else None)
    try:
        with motor if motor is not None else nullcontext():
            while not stop.is_set():
                began = timing.tick()
                if duration is not None and began - started >= duration:
                    print("EE exit reason: requested duration elapsed.", flush=True)
                    break
                if keys is not None and keys.pressed():
                    request_stop(stop, "terminal Space/Esc/Q stop key")
                    break
                if viewer is not None and not viewer.is_running():
                    request_stop(stop, "viewer closed")
                    break
                if motor is not None:
                    actual = motor.read()
                if viewer is not None:
                    if viewer.consume_hold():
                        target = ik.pose(actual)
                        viewer.set_target(target)
                    else:
                        target = viewer.read_target()
                next_q = ik.solve(target, init_q=actual, stop=stop)
                next_q = np.clip(next_q, ik.lower, ik.upper)
                blocked = ik.has_self_collision(next_q)
                if stop.is_set():
                    break
                if motor is not None:
                    motor.check()
                    if not blocked:
                        motor.command(next_q)
                    actual = motor.read()
                elif not blocked:
                    actual = next_q.copy()
                if viewer is not None:
                    viewer.blocked = blocked
                    ik.update_display(actual)
                    viewer.sync()
                timing.complete(began, time.monotonic())
                timing.pace(stop)
            if motor is not None:
                motor.check()
                actual = motor.read()
    finally:
        report = timing.report()
        print(f"EE target loop: {report['hz']:.1f} Hz; max work {report['max_work_ms']:.1f} ms; "
              f"skipped periods {report['skipped']}.", flush=True)
    errors = ik.error(target, actual)
    reached = errors[0] <= .002 and errors[1] <= math.radians(2.)
    print(f"TCP error: {errors[0] * 1000:.2f} mm, {math.degrees(errors[1]):.2f} deg; "
          f"{'reached' if reached else 'not reached'}.", flush=True)
    return Result(*errors, reached, stop.is_set())


def control(*, hardware=False, side="right", config=CONFIG, settings_path=SETTINGS_PATH,
            pose=None, offset=None, start_joints=None, tcp_offset=(0., 0., 0.),
            duration=None, headless=False,
            bus_factory=MotorBus, stop=None, port=8080, open_browser=True):
    if side not in ("left", "right"):
        raise ValueError("EE control selects one arm: left or right.")
    if pose is not None and offset is not None:
        raise ValueError("Select an absolute pose or a relative translation, not both.")
    if pose is not None:
        pose_matrix(pose)
    if offset is not None:
        offset = np.asarray(offset, dtype=float)
        if offset.shape != (3,) or not np.all(np.isfinite(offset)):
            raise ValueError("Offset must contain three finite values in meters.")
    if duration is not None and (not math.isfinite(duration) or duration <= 0):
        raise ValueError("Duration must be finite and positive.")
    if headless and (duration is None or (pose is None and offset is None)):
        raise ValueError("Headless control requires --duration and --pose or --offset.")
    if hardware and start_joints is not None:
        raise ValueError("--start-joints is simulation-only; hardware always starts at measured angles.")
    cfg = load_settings(settings_path)
    arm = channel = reference = gravity = None
    lower = upper = None
    if hardware:
        arms = load_config(config)
        if side not in arms or "reference_rad" not in arms[side]:
            raise ValueError(f"Bind and calibrate the {side} reBot arm first.")
        arm = arms[side]
        reference, lower, upper = position_bounds(arm, 6, 270.)
        gravity = GravityFeedforward(GravityModel(), reference, cfg)
        channel = resolve_channel(arm)
    start = SIM_START.copy() if start_joints is None else np.radians(vector6(start_joints, "Start joints"))
    if hardware:
        start = np.clip(start, lower, upper)
    ik = CartesianIK(start, lower, upper, tcp_offset=tcp_offset)
    stop = threading.Event() if stop is None else stop
    terminal = StopKey() if headless else nullcontext(None)
    with stop_signals(stop), terminal as keys:
        if headless:
            display = nullcontext(None)
        else:
            from rebot_ee_viewer import WebViewer

            display = WebViewer(ik, stop, port=port, hardware=hardware, open_browser=open_browser)
        with display as viewer:
            if viewer is not None and not viewer.wait_for_client():
                return Result(math.inf, math.inf, False, True)
            with (bus_factory(channel, arm["models"], max_temperature=cfg["max_temperature_c"],
                              motor_count=6) if hardware else nullcontext(None)) as bus:
                if bus is not None:
                    verify_identity(bus, arm)
                    raw = validate_pose(bus.positions(), reference, ik.lower, ik.upper)
                    ik.reset(raw - reference)
                    bus.set_hold(raw, gravity(raw), MOTION_KP, MOTION_KD)
                    bus.hold_gravity = gravity
                    bus.prepare()
                    raw = validate_pose(bus.positions(), reference, ik.lower, ik.upper)
                    ik.reset(raw - reference)
                target = target_pose(ik, pose, offset)
                if viewer is not None:
                    ik.update_display()
                    viewer.set_target(target)
                    viewer.fit_camera()
                if stop.is_set() or (viewer is not None and not viewer.is_running()):
                    return Result(math.inf, math.inf, False, True)
                if bus is not None:
                    bus.set_hold(raw, gravity(raw), MOTION_KP, MOTION_KD)
                    bus.enable(raw)
                    if stop.is_set():
                        return Result(math.inf, math.inf, False, True)
                    # Refresh the entire feedback batch after the sequential enable handshake.
                    bus.command_position(raw, np.zeros(6), gravity(raw), MOTION_KP, MOTION_KD,
                                         time.monotonic() + cfg["max_feedback_age_s"])
                mode = f"HARDWARE {side}" if hardware else "KINEMATIC SIMULATION"
                print(f"EE {mode}; reference {EE_FRAME}, TCP offset {tuple(tcp_offset)} m; "
                      "base-frame target; direct IK joint targets without host speed limiting. "
                      "MIT desired velocity is zero; measured-pose gravity feedforward is enabled on hardware; "
                      "target self-collision checks enabled.", flush=True)
                stop_help = ("Use the web STOP & hold button or terminal Ctrl-C to stop; "
                             "browser disconnects and terminal Space/Esc/Q do not stop control. "
                             if not headless else
                             "Press Space/Esc/Q/Ctrl-C to stop. ")
                print(stop_help +
                      "Hardware exit attempts position hold and leaves motors enabled; gripper is unchanged.",
                      flush=True)
                if viewer is not None:
                    print("Drag the target to control the end effector. T: move; R: rotate; "
                          "F: fit view; Esc: cancel drag. Browser reconnect preserves control.", flush=True)
                return track(ik, target, stop, keys, cfg, duration=duration, viewer=viewer, bus=bus,
                             reference=reference, gravity=gravity)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--hardware", action="store_true", help="Control physical hardware; default is kinematic simulation")
    parser.add_argument("--side", choices=("left", "right"), default="right")
    parser.add_argument("--config", type=Path, default=CONFIG)
    parser.add_argument("--settings", type=Path, default=SETTINGS_PATH)
    target = parser.add_mutually_exclusive_group()
    target.add_argument("--pose", type=float, nargs=6, metavar=("X", "Y", "Z", "ROLL", "PITCH", "YAW"),
                        help="Absolute mount TCP pose in the arm base frame: meters and degrees (extrinsic XYZ)")
    target.add_argument("--offset", type=float, nargs=3, metavar=("DX", "DY", "DZ"),
                        help="Translation in base-frame meters from startup TCP; preserve startup orientation")
    parser.add_argument("--start-joints", type=float, nargs=6, help="Simulation initial J1-J6 in degrees")
    parser.add_argument("--tcp-offset", type=float, nargs=3, default=(0., 0., 0.),
                        help="TCP offset in EEF axes (+X approach, +Z palm back), in meters; default is zero")
    parser.add_argument("--duration", type=float, help="Stop after this many seconds; required with --headless")
    parser.add_argument("--headless", action="store_true", help="No viewer; requires a fixed target and duration")
    parser.add_argument("--port", type=int, default=8080, help="Local web viewer port")
    parser.add_argument("--no-browser", action="store_true", help="Print the web URL without opening a browser")
    args = parser.parse_args(argv)
    try:
        result = control(hardware=args.hardware, side=args.side, config=args.config, settings_path=args.settings,
                         pose=args.pose, offset=args.offset, start_joints=args.start_joints,
                         tcp_offset=args.tcp_offset, duration=args.duration, headless=args.headless,
                         port=args.port, open_browser=not args.no_browser)
        if result.interrupted:
            return 130
        return 0 if result.reached or (args.pose is None and args.offset is None) else 2
    except (OSError, ValueError, RuntimeError, ImportError, can.CanError, yaml.YAMLError,
            subprocess.SubprocessError) as exc:
        print(f"EE control failed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
