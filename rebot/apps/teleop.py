"""Teleoperate physical reBot RS arms and grippers with calibrated Star Arm leaders.

One process per device keeps slow serial reads away from the CAN loops:

    leader-<side>  reads leader angles as fast as the serial link allows
    target         maps the newest leader frames to follower joint targets (500 Hz)
    motor-<side>   servos one follower arm on the newest target (200 Hz)

The supervisor (this process) starts them in phases, so nothing is enabled until
every device is verified, and stops everything when any worker fails.
"""

import argparse
from dataclasses import dataclass
import math
import multiprocessing as mp
from pathlib import Path
import queue
import sys
import time

import numpy as np

from rebot.bindings import select_arms, select_leaders
from rebot.devices.robstride import MotorBus
from rebot.devices.stararm import AngleFeedbackError, open_leader, read_angles, release_torque
from rebot.model.gravity import GravityModel
from rebot.model.gripper import load_gripper_settings
from rebot.model.leader_map import leader_angles, map_target
from rebot.model.limits import GRIPPER_OPEN_DEG
from rebot.paths import ARM_BINDINGS, CONTROL_SETTINGS, GRIPPER_SETTINGS, LEADER_BINDINGS
from rebot.runtime.arm import Arm, FeedbackPolicy, follower_range
from rebot.runtime.console import StopKey, StopSignal, handle_signals, stop_signals
from rebot.runtime.ipc import SharedFrame, latest, stop_on_error, supervisor_alive, wait_phase
from rebot.runtime.timing import LoopTiming
from rebot.settings import ControlSettings, load_settings

MOTOR_HZ = 200
TARGET_HZ = 500
LEADER_READ_ATTEMPTS = 15


@dataclass(frozen=True)
class Plan:
    """Everything the workers need to know; pickled into each spawned process."""
    arms: dict                           # side -> ArmBinding
    leaders: dict                        # side -> LeaderBinding
    gravity: dict[str, tuple[np.ndarray, ControlSettings]]  # mass scales and settings per arm
    gripper: object                      # GripperSettings, or None to leave motor 7 alone
    gripper_open_deg: float

    @property
    def sides(self):
        return tuple(self.arms)

    @property
    def count(self):
        return 6 if self.gripper is None else 7

    def arm(self, side, stop, bus_factory):
        scales, settings = self.gravity[side]
        model = GravityModel()
        model.apply_scales(scales)
        return Arm(self.arms[side], settings, gravity=model, gripper=self.gripper,
                   gripper_open_deg=self.gripper_open_deg, feedback=FeedbackPolicy.tolerant(),
                   stop=stop, bus_factory=bus_factory)


@dataclass(frozen=True)
class Link:
    """What every worker shares with the supervisor."""
    stop: StopSignal
    heartbeat: SharedFrame
    phases: dict                         # prepare, enable, run: set by the supervisor in order
    reports: object                      # queue of timing and error records

    def wait(self, phase):
        """Block until the supervisor opens `phase`; False when stopped first."""
        return wait_phase(self.phases[phase], self.stop, self.heartbeat)

    def alive(self):
        supervisor_alive(self.heartbeat)


class StarArmSource:
    """Open the serial connection only inside its owning acquisition process."""

    def __init__(self, leader):
        self.leader = leader

    def __enter__(self):
        self.connection = open_leader(self.leader.port, self.leader.baudrate)
        self.bus = self.connection.__enter__()
        return self

    def read(self):
        return read_angles(self.bus)

    def release(self):
        release_torque(self.bus)

    def __exit__(self, *args):
        return self.connection.__exit__(*args)


def leader_worker(name, leader, frame, link, ready, prepared, source_factory):
    with source_factory(leader) as source, stop_on_error(link.stop):
        timing = LoopTiming(name)
        released = False
        failures = 0
        try:
            while not link.stop.is_set():
                link.alive()
                if link.phases["prepare"].is_set() and not released:
                    source.release()
                    released = True
                    prepared.set()
                began = timing.tick()
                try:
                    raw = source.read()
                except AngleFeedbackError as exc:
                    if link.stop.is_set():
                        break
                    failures += 1
                    if failures >= LEADER_READ_ATTEMPTS:
                        raise AngleFeedbackError(
                            f"{name}: angle feedback failed after {failures} attempts: {exc}") from exc
                    print(f"{name}: angle feedback attempt {failures}/{LEADER_READ_ATTEMPTS} failed; "
                          f"keeping the last complete target ({exc}).", file=sys.stderr, flush=True)
                    link.stop.wait(.001)
                    continue
                if link.stop.is_set():
                    break
                if failures:
                    print(f"{name}: angle feedback recovered after {failures} failed attempt(s).", flush=True)
                failures = 0
                frame.publish(leader_angles(raw, "Leader feedback"), began)
                ready.set()
                timing.complete(began, began)
                # Acquisition follows the real serial rate, not a claimed 500 Hz.
        finally:
            link.reports.put(timing.report())


def target_worker(plan, sources, targets, link, ready):
    ranges = {side: follower_range(plan.arms[side], plan.count, plan.gripper_open_deg)
              for side in plan.sides}
    tolerance = plan.gripper.leader_close_tolerance_deg if plan.gripper is not None else 0.
    ready.set()
    if not link.wait("run"):
        return
    timing = LoopTiming("target", TARGET_HZ)
    try:
        while not link.stop.is_set():
            began = timing.tick()
            link.alive()
            values, ages = [], []
            for side in plan.sides:
                raw, source_at = latest(sources[side], side)
                values.extend(map_target(raw, plan.leaders[side].reference_deg,
                                         ranges[side].lower, ranges[side].upper, tolerance))
                ages.append(source_at)
            if not link.stop.is_set():
                # Publish the complete pair atomically and preserve acquisition age.
                targets.publish(values, min(ages))
                timing.complete(began, min(ages))
            timing.pace(link.stop)
    finally:
        link.reports.put(timing.report())


def motor_worker(name, side, plan, targets, link, ready, prepared, enabled, bus_factory,
                 recording_samples=None):
    if recording_samples is not None:
        from rebot.data.recording import publish_feedback

    stop = link.stop
    index = plan.sides.index(side)
    with plan.arm(side, stop, bus_factory) as arm, stop_on_error(stop):
        ready.set()
        if not link.wait("prepare"):
            return
        arm.prepare()
        prepared.set()
        if not link.wait("enable") or stop.is_set():
            return
        arm.enable()
        enabled.set()
        if not link.wait("run"):
            return
        # The mapper may not have published its first pair yet; no motion until it does.
        while not stop.is_set() and targets.read()[1] == 0:
            link.alive()
            stop.wait(.001)
        if stop.is_set():
            return
        if arm.gripper is not None:
            # Sequential enable and the run barrier can age the first motor's reply.
            # Refresh every axis at the hold pose before the gripper consumes
            # feedback, without moving to the leader target.
            link.alive()
            if not arm.refresh():
                return
        timing = LoopTiming(name, MOTOR_HZ)
        try:
            while not stop.is_set():
                began = timing.tick()
                link.alive()
                pair, source_at = latest(targets, "Target pair")
                target = pair[index * arm.count:(index + 1) * arm.count]
                # Validate the consumer boundary as well as the mapper output.
                if np.any(target < arm.range.lower) or np.any(target > arm.range.upper):
                    raise ValueError("Shared target exceeds the configured motor range.")
                if not arm.servo(target, began):
                    break
                if recording_samples is not None:
                    publish_feedback(recording_samples, side, arm.feedback_states())
                timing.complete(began, source_at)
                timing.pace(stop)
        finally:
            link.reports.put(timing.report())


def worker_entry(name, worker, args, link):
    # Handle termination cooperatively so active motor workers can install hold.
    handle_signals(link.stop)
    try:
        worker(*args)
    except BaseException as exc:
        link.stop.set()
        link.reports.put({"type": "error", "worker": name, "error": f"{type(exc).__name__}: {exc}"})
        raise SystemExit(1)


class Supervisor:
    """Start workers in phases, watch them, and shut everything down together."""

    STAGES = ("ready", "prepared", "enabled")

    def __init__(self, startup_timeout=15., shutdown_timeout=5., session=None):
        if any(not math.isfinite(value) or value <= 0 for value in (startup_timeout, shutdown_timeout)):
            raise ValueError("Worker startup and shutdown timeouts must be finite and positive.")
        self.startup_timeout, self.shutdown_timeout = startup_timeout, shutdown_timeout
        self.session = session
        self.context = mp.get_context("spawn")
        heartbeat = SharedFrame(self.context, 0)
        heartbeat.publish([], time.monotonic())
        self.link = Link(StopSignal(self.context.Event()), heartbeat,
                         {name: self.context.Event() for name in ("prepare", "enable", "run")},
                         self.context.Queue())
        self.stop = self.link.stop
        self.acks = {stage: [] for stage in self.STAGES}
        self.workers, self.started, self.messages = [], [], []

    def frame(self, size):
        return SharedFrame(self.context, size)

    def ack(self, stage):
        """An event the next worker sets when it has completed `stage`."""
        event = self.context.Event()
        self.acks[stage].append(event)
        return event

    def add(self, name, worker, *args):
        self.workers.append(self.context.Process(
            name=name, target=worker_entry, args=(name, worker, args, self.link)))

    def error(self, worker, message):
        self.messages.append({"type": "error", "worker": worker, "error": message})

    def collect(self):
        while True:
            try:
                record = self.link.reports.get_nowait()
            except queue.Empty:
                return
            self.messages.append(record)
            if record["type"] == "error":
                print(f"{record['worker']}: {record['error']}", file=sys.stderr, flush=True)

    def monitor(self, keys):
        self.link.heartbeat.publish([], time.monotonic())
        self.collect()
        if self.session is not None:
            self.session.check()
        if keys.pressed():
            self.stop.set("terminal Space/Esc/Q stop key")
        for process in self.workers:
            if process.pid is not None and process.exitcode is not None and not self.stop.is_set():
                self.error(process.name, f"Worker exited unexpectedly ({process.exitcode}).")
                self.stop.set()

    def wait_until(self, done, keys, what):
        deadline = time.monotonic() + self.startup_timeout
        while not self.stop.is_set():
            self.monitor(keys)
            if self.stop.is_set():
                return False
            if done():
                return True
            if time.monotonic() > deadline:
                raise TimeoutError(f"Teleoperation startup timed out: {what}.")
            self.stop.wait(.01)
        return False

    def wait_for(self, stage, keys):
        return self.wait_until(lambda: all(event.is_set() for event in self.acks[stage]),
                               keys, f"workers not {stage}")

    def run(self, keys):
        """Walk the startup phases, then supervise until something stops the session."""
        phases, session = self.link.phases, self.session
        for process in self.workers:
            process.start()
            self.started.append(process)
        if not self.wait_for("ready", keys):
            return
        phases["prepare"].set()
        if not self.wait_for("prepared", keys):
            return
        # Cameras must be recording-ready before any motor is enabled.
        if session is not None and not self.wait_until(session.ready, keys, "recording not ready"):
            return
        phases["enable"].set()
        if not self.wait_for("enabled", keys):
            return
        if session is not None:
            session.start()
        phases["run"].set()
        while not self.stop.is_set():
            self.monitor(keys)
            self.stop.wait(.01)

    def shutdown(self):
        """Stop, give workers time to install hold, and account for every exit."""
        self.stop.set()
        if self.session is not None:
            try:
                self.session.stop_cameras()
            except (OSError, RuntimeError) as exc:
                self.error("record-camera", str(exc))
        deadline = time.monotonic() + self.shutdown_timeout
        while any(process.is_alive() for process in self.started) and time.monotonic() < deadline:
            try:
                self.link.heartbeat.publish([], time.monotonic())
            except TimeoutError:
                # A dead process may have held this lock. Still join peers.
                pass
            self.collect()
            for process in self.started:
                process.join(.01)
        for process in self.started:
            if process.is_alive():
                self.error(process.name, "Hold cleanup timed out; native hold was not confirmed.")
                process.kill()
            process.join()
        self.collect()
        for process in self.started:
            if process.exitcode != 0 and not any(
                    r["type"] == "error" and r["worker"] == process.name for r in self.messages):
                self.error(process.name, f"Worker exited unexpectedly ({process.exitcode}).")

    def close(self):
        self.link.reports.close()
        self.link.reports.join_thread()

    def report(self):
        """Print worker timing; raise if any worker failed."""
        for record in self.messages:
            if record["type"] == "timing":
                print(f"{record['worker']}: {record['hz']:.1f} Hz; max gap {record['max_gap_ms']:.1f} ms; "
                      f"max work {record['max_work_ms']:.1f} ms; max source age {record['max_source_age_ms']:.1f} ms; "
                      f"skipped ticks {record['skipped']}.", flush=True)
        errors = [f"{r['worker']}: {r['error']}" for r in self.messages if r["type"] == "error"]
        if errors:
            raise RuntimeError("Teleoperation stopped after worker failure: " + "; ".join(errors))
        return self.messages


def make_plan(config=ARM_BINDINGS, leader_config=LEADER_BINDINGS, settings_path=CONTROL_SETTINGS,
              side="both", gripper=True, gripper_open_deg=GRIPPER_OPEN_DEG,
              gripper_settings_path=GRIPPER_SETTINGS, gripper_torque_nm=None):
    """Load and cross-check every binding and setting before any process starts."""
    arms = select_arms(config, side)
    settings = load_settings(settings_path)
    # Send validated parameters to workers; each motor process owns its Pinocchio model.
    gravity = {}
    for name, binding in arms.items():
        model, arm_settings = GravityModel.for_arm(binding, settings)
        gravity[name] = model.scales, arm_settings
    plan = Plan(arms, select_leaders(leader_config, side), gravity,
                load_gripper_settings(gripper_settings_path, gripper_torque_nm) if gripper else None,
                gripper_open_deg)
    for name in plan.sides:
        plan.arm(name, None, None)       # validates travel limits and gripper gains
    return plan


def teleop(plan, *, recording=None, source_factory=StarArmSource, bus_factory=MotorBus,
           startup_timeout=15., shutdown_timeout=5.):
    """Run until stopped. `recording` (RecordingOptions) also records one dataset recording."""
    session = None
    try:
        if recording is not None:
            from rebot.data.recording import RecordingSession

            if plan.sides != ("left", "right") or plan.gripper is None:
                raise ValueError("Dataset recording requires both arms and both grippers.")
            session = RecordingSession(
                recording, {side: plan.arms[side].reference for side in plan.sides})
        supervisor = Supervisor(startup_timeout, shutdown_timeout, session)
        sources = {side: supervisor.frame(7) for side in plan.sides}
        targets = supervisor.frame(plan.count * len(plan.sides))
        for side in plan.sides:
            supervisor.add(f"leader-{side}", leader_worker, f"leader-{side}", plan.leaders[side],
                           sources[side], supervisor.link, supervisor.ack("ready"),
                           supervisor.ack("prepared"), source_factory)
            supervisor.add(f"motor-{side}", motor_worker, f"motor-{side}", side, plan, targets,
                           supervisor.link, supervisor.ack("ready"), supervisor.ack("prepared"),
                           supervisor.ack("enabled"), bus_factory,
                           session.samples if session is not None else None)
        supervisor.add("target", target_worker, plan, sources, targets, supervisor.link,
                       supervisor.ack("ready"))
        try:
            with stop_signals(supervisor.stop), StopKey() as keys:
                print(f"PHYSICAL TELEOP: motor {MOTOR_HZ} Hz per arm; target {TARGET_HZ} Hz; "
                      "measured-pose gravity feedforward on, MIT target velocity zero. Support the Star Arm leaders. "
                      "Space/Esc/Q/Ctrl-C stops and holds.", flush=True)
                if plan.gripper is not None:
                    print(f"Gripper: leader closing tolerance {plan.gripper.leader_close_tolerance_deg:g} deg; "
                          f"smooth closing preload {plan.gripper.close_preload_deg:g} motor deg; total host torque limit "
                          f"+/-{plan.gripper.torque_limit_nm:g} Nm during tracking; "
                          "native position hold on exit (not torque limited).", flush=True)
                try:
                    supervisor.run(keys)
                finally:
                    supervisor.shutdown()
        finally:
            supervisor.close()
        messages = supervisor.report()
        if session is not None and session.started:
            session.finish()
        return messages
    finally:
        if session is not None:
            session.close()


def add_arguments(parser):
    """Options shared by `rebot teleop` and `rebot collect`."""
    parser.add_argument("--config", type=Path, default=ARM_BINDINGS, help="Follower bindings YAML")
    parser.add_argument("--leader-config", type=Path, default=LEADER_BINDINGS)
    parser.add_argument("--settings", type=Path, default=CONTROL_SETTINGS)
    parser.add_argument("--gripper-settings", type=Path, default=GRIPPER_SETTINGS,
                        help="RS gripper host position gains and total torque limit YAML")
    parser.add_argument("--gripper-torque-nm", type=float,
                        help="Override the maximum absolute total gripper tracking torque in Nm")
    parser.add_argument("--gripper-open-deg", type=float, default=GRIPPER_OPEN_DEG,
                        help="Gripper motor angle from closed to open (maximum: 270 degrees)")


def plan_from_arguments(args, side="both", gripper=True):
    return make_plan(args.config, args.leader_config, args.settings, side, gripper,
                     args.gripper_open_deg, args.gripper_settings, args.gripper_torque_nm)


def main(argv=None, prog=None):
    parser = argparse.ArgumentParser(prog=prog, description=__doc__.split("\n\n")[0])
    add_arguments(parser)
    parser.add_argument("--side", choices=("left", "right", "both"), default="both")
    parser.add_argument("--no-gripper", action="store_true", help="Leave motor 7 control unchanged")
    args = parser.parse_args(argv)
    teleop(plan_from_arguments(args, args.side, not args.no_gripper))
    return 0
