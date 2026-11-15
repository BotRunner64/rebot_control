"""Control one reBot RS end-effector frame with Mink IK; kinematic simulation unless --hardware."""

import argparse
from contextlib import ExitStack
from dataclasses import dataclass
import math
from pathlib import Path
import time

import numpy as np

from rebot.bindings import SIDES, select_arms
from rebot.devices.robstride import MotorBus
from rebot.model.arrays import vector6
from rebot.model.gravity import GravityModel
from rebot.model.kinematics import CartesianIK, EE_FRAME, SIM_START, pose_matrix
from rebot.paths import ARM_BINDINGS, CONTROL_SETTINGS
from rebot.runtime.arm import Arm
from rebot.runtime.console import StopKey, StopSignal, request_stop, stop_signals
from rebot.runtime.timing import LoopTiming
from rebot.runtime.worker import ArmWorker
from rebot.settings import load_settings

TARGET_HZ = 50
REACHED_M, REACHED_RAD = .002, math.radians(2.)


@dataclass
class Result:
    position_error_m: float
    orientation_error_rad: float
    reached: bool
    interrupted: bool


INTERRUPTED = Result(math.inf, math.inf, False, True)


def target_pose(ik, pose, offset):
    """Absolute pose, or the startup pose translated by `offset`, as an SE(3) matrix."""
    if pose is not None:
        return pose_matrix(pose)
    target = ik.pose()
    if offset is not None:
        target[:3, 3] += offset
    return target


def track(ik, target, stop, keys, *, duration=None, viewer=None, worker=None):
    """Solve IK for the newest target at 50 Hz; the worker thread services CAN independently."""
    timing = LoopTiming("ee-target", TARGET_HZ)
    started = time.monotonic()
    actual = ik.q if worker is None else worker.read()
    try:
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
            if worker is not None:
                actual = worker.read()
            if viewer is not None:
                if viewer.consume_hold():
                    target = ik.pose(actual)
                    viewer.set_target(target)
                else:
                    target = viewer.read_target()
            next_q = np.clip(ik.solve(target, init_q=actual, stop=stop), ik.lower, ik.upper)
            blocked = ik.has_self_collision(next_q)
            if stop.is_set():
                break
            if worker is not None:
                worker.check()
                if not blocked:
                    worker.command(next_q)
                actual = worker.read()
            elif not blocked:
                actual = next_q.copy()
            if viewer is not None:
                viewer.blocked = blocked
                ik.update_display(actual)
                viewer.sync()
            timing.complete(began, time.monotonic())
            timing.pace(stop)
    finally:
        report = timing.report()
        print(f"EE target loop: {report['hz']:.1f} Hz; max work {report['max_work_ms']:.1f} ms; "
              f"skipped periods {report['skipped']}.", flush=True)
    return target, actual


def control(*, hardware=False, side="right", config=ARM_BINDINGS, settings_path=CONTROL_SETTINGS,
            pose=None, offset=None, start_joints=None, tcp_offset=(0., 0., 0.),
            duration=None, headless=False, bus_factory=MotorBus, stop=None, port=8080,
            open_browser=True):
    if side not in SIDES:
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
    stop = StopSignal() if stop is None else stop
    arm = None
    lower = upper = None
    start = SIM_START.copy() if start_joints is None else np.radians(vector6(start_joints, "Start joints"))
    if hardware:
        arm = Arm(select_arms(config, side)[side], load_settings(settings_path),
                  gravity=GravityModel(), stop=stop, bus_factory=bus_factory)
        lower, upper = arm.range.lower, arm.range.upper
        start = np.clip(start, lower, upper)
    ik = CartesianIK(start, lower, upper, tcp_offset=tcp_offset)
    with stop_signals(stop), ExitStack() as stack:
        keys = viewer = worker = None
        if headless:
            keys = stack.enter_context(StopKey())
        else:
            from rebot.viewer.ee import WebViewer

            viewer = stack.enter_context(
                WebViewer(ik, stop, port=port, hardware=hardware, open_browser=open_browser))
            if not viewer.wait_for_client():
                return INTERRUPTED
        if arm is not None:
            stack.enter_context(arm)
            arm.prepare()
            ik.reset(arm.sample())
        target = target_pose(ik, pose, offset)
        if viewer is not None:
            ik.update_display()
            viewer.set_target(target)
            viewer.fit_camera()
        if stop.is_set() or (viewer is not None and not viewer.is_running()):
            return INTERRUPTED
        if arm is not None:
            arm.enable()
            # Refresh the entire feedback batch after the sequential enable handshake.
            if stop.is_set() or not arm.refresh():
                return INTERRUPTED
            worker = stack.enter_context(ArmWorker(arm, stop))
        mode = f"HARDWARE {side}" if hardware else "KINEMATIC SIMULATION"
        print(f"EE {mode}; reference {EE_FRAME}, TCP offset {tuple(tcp_offset)} m; "
              "base-frame target; direct IK joint targets without host speed limiting. "
              "MIT desired velocity is zero; measured-pose gravity feedforward is enabled on hardware; "
              "target self-collision checks enabled.", flush=True)
        print(("Press Space/Esc/Q/Ctrl-C to stop. " if headless else
               "Use the web STOP & hold button or terminal Ctrl-C to stop; "
               "browser disconnects and terminal Space/Esc/Q do not stop control. ")
              + "Hardware exit attempts position hold and leaves motors enabled; gripper is unchanged.",
              flush=True)
        if viewer is not None:
            print("Drag the target to control the end effector. T: move; R: rotate; "
                  "F: fit view; Esc: cancel drag. Browser reconnect preserves control.", flush=True)
        target, actual = track(ik, target, stop, keys, duration=duration, viewer=viewer, worker=worker)
        if worker is not None:
            worker.check()
            actual = worker.read()
    position, orientation = ik.error(target, actual)
    reached = position <= REACHED_M and orientation <= REACHED_RAD
    print(f"TCP error: {position * 1000:.2f} mm, {math.degrees(orientation):.2f} deg; "
          f"{'reached' if reached else 'not reached'}.", flush=True)
    return Result(position, orientation, reached, stop.is_set())


def main(argv=None, prog=None):
    parser = argparse.ArgumentParser(prog=prog, description=__doc__)
    parser.add_argument("--hardware", action="store_true", help="Control physical hardware; default is kinematic simulation")
    parser.add_argument("--side", choices=SIDES, default="right")
    parser.add_argument("--config", type=Path, default=ARM_BINDINGS)
    parser.add_argument("--settings", type=Path, default=CONTROL_SETTINGS)
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
    result = control(hardware=args.hardware, side=args.side, config=args.config, settings_path=args.settings,
                     pose=args.pose, offset=args.offset, start_joints=args.start_joints,
                     tcp_offset=args.tcp_offset, duration=args.duration, headless=args.headless,
                     port=args.port, open_browser=not args.no_browser)
    if result.interrupted:
        return 130
    return 0 if result.reached or (args.pose is None and args.offset is None) else 2
