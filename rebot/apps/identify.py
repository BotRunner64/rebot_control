"""Automatic single-arm gravity calibration: sample, fit mass scales, validate independently."""

import argparse
from contextlib import ExitStack
from datetime import datetime, timezone
import json
from pathlib import Path
import time

import yaml

from rebot.bindings import SIDES, select_arms
from rebot.devices.robstride import MotorBus
from rebot.model.arrays import vector6
from rebot.model.gravity import GravityModel, digest
from rebot.model.gravity_fit import (SAMPLE_S, SETTLE_S, clearance_pose, fit, make_plan, path_check,
                                     positive, validate, validate_plan, write_yaml)
from rebot.model.trajectory import (SAMPLING_SPEED, MotionLog, MotionStats, TrajectoryTracker,
                                    minimum_jerk, movement_duration, plain)
from rebot.paths import ARM_BINDINGS, CONTROL_SETTINGS, RUNS
from rebot.runtime.arm import Arm
from rebot.runtime.console import StopKey, StopSignal, stop_signals
from rebot.settings import load_settings


def collect(plan_path, output, side="right", config=ARM_BINDINGS, settings_path=CONTROL_SETTINGS,
            speed=SAMPLING_SPEED, settle_s=SETTLE_S, sample_s=SAMPLE_S, *,
            bus_factory=MotorBus, stop=None):
    """Follow one plan, logging settled samples. Returns completed or interrupted."""
    positive(speed, "speed")
    positive(settle_s, "settle_s", 10)
    positive(sample_s, "sample_s", 10)
    if settle_s < .5 or sample_s < .5:
        raise ValueError("Settle/sample durations must be >= 0.5 s.")
    if side not in SIDES:
        raise ValueError("Collect one arm at a time.")
    cfg, model = load_settings(settings_path), GravityModel()
    plan = yaml.safe_load(Path(plan_path).read_text())
    poses = validate_plan(plan, model, cfg.gravity_scale, cfg.torque_limit_nm)
    binding = select_arms(config, side)[side]
    arm = Arm(binding, cfg, gravity=model, joint_limits=False, bus_factory=bus_factory)
    first = vector6(poses[0]["q"], "pose")
    stop = StopSignal() if stop is None else stop
    status = "error"
    with stop_signals(stop), ExitStack() as stack:
        journal = stack.enter_context(MotionLog(output))
        stats = MotionStats()
        stack.callback(stats.report, side)
        journal.write({"type": "metadata", "version": 3, "control": "native_mit", "side": side,
                       "motor_uids": binding.motor_uids, "reference_rad": binding.reference[:6],
                       "model_sha256": model.sha256, "plan": plan,
                       "settings": {**cfg.plain(), "hold_kp": arm.kp, "damping": arm.kd},
                       "speed": speed, "settle_s": settle_s, "sample_s": sample_s,
                       "effort_note": "Native PD effort is estimated from decoded commands and measured pose; no independent torque sensor."})
        # Register the terminal record before the bus, so freeze runs before final logging.
        stack.callback(lambda: journal.write({"type": "end", "status": status}))
        try:
            stack.enter_context(arm)
            keys = stack.enter_context(StopKey())

            def interrupted():
                return stop.is_set() or keys.pressed()

            path_check(model, arm.q, first, cfg.gravity_scale, cfg.torque_limit_nm)
            if interrupted():
                return (status := "interrupted")
            arm.prepare()
            path_check(model, arm.sample(), first, cfg.gravity_scale, cfg.torque_limit_nm)
            if interrupted():
                return (status := "interrupted")
            arm.enable()
            tracker = TrajectoryTracker(model, cfg)
            print("AUTOMATIC SAMPLING: Space/Esc/Q/Ctrl-C stops and holds. "
                  "Closed unloaded gripper; no external support. "
                  "Fixed elbow-up sweeps; no collision avoidance.", flush=True)
            print("Native MIT Kp: " + " ".join(f"{gain:g}" for gain in arm.kp), flush=True)
            session_start = time.monotonic()
            trajectory_target = arm.q.copy()
            for pose in poses:
                start = trajectory_target.copy()
                goal = vector6(pose["q"], "pose")
                duration = movement_duration(start, goal, speed)
                sampling = pose["stage"] == "sample"
                began = time.monotonic()
                quiet_since = sample_since = None
                window = 0
                label = (f"pair {pose['pair_id']} {pose['direction']:+d}, {pose['split']}"
                         if sampling else pose["stage"])
                print(f"Waypoint {pose['id']} ({label}): move {duration:.1f}s" +
                      (", then settle/sample." if sampling else "."), flush=True)
                while True:
                    tick = time.monotonic()
                    if interrupted():
                        return (status := "interrupted")
                    progress = min(1., (tick - began) / duration)
                    target, velocity = minimum_jerk(start, goal, tick - began, duration)
                    q = arm.sample()
                    now = time.monotonic()
                    record = tracker.update(q, target, now, velocity)
                    if interrupted():
                        return (status := "interrupted")
                    arm.command_trajectory(record)
                    stats.update(record)
                    # A sample window opens after the arm has been stable for settle_s.
                    if sampling and progress == 1 and record["stable"]:
                        quiet_since = now if quiet_since is None else quiet_since
                        if now - quiet_since >= settle_s and sample_since is None:
                            sample_since = now
                    else:
                        quiet_since = sample_since = None
                        window += 1
                    record.update({"type": "sample", "pose_id": pose["id"], "window": window,
                                   "time_s": now - session_start, "eligible": sample_since is not None,
                                   "phase": "MOVE" if progress < 1 or not sampling else
                                   ("SAMPLE" if sample_since is not None else "SETTLE"),
                                   **arm.telemetry()})
                    journal.write(record)
                    if progress == 1 and not sampling:
                        break
                    if sample_since is not None and now - sample_since >= sample_s:
                        journal.write({"type": "pose_complete", "pose_id": pose["id"], "window": window})
                        break
                    time.sleep(max(0, 1 / cfg.rate_hz - (time.monotonic() - tick)))
                trajectory_target = goal.copy()
            status = "completed"
        except BaseException as exc:
            journal.write({"type": "error", "message": str(exc)})
            raise
    return status


def read_current_pose(config, side, *, bus_factory=MotorBus):
    binding = select_arms(config, side)[side]
    with bus_factory(binding.channel, binding.models) as bus:
        binding.verify(bus)
        return vector6(bus.positions(), "positions") - binding.reference[:6]


def preflight(plans, side="right", config=ARM_BINDINGS, settings_path=CONTROL_SETTINGS, *,
              bus_factory=MotorBus):
    """Check identity and both paths internally before starting automatic motion."""
    cfg, model = load_settings(settings_path), GravityModel()
    current = read_current_pose(config, side, bus_factory=bus_factory)
    for path in plans:
        poses = validate_plan(yaml.safe_load(Path(path).read_text()), model,
                              cfg.gravity_scale, cfg.torque_limit_nm)
        path_check(model, current, vector6(poses[0]["q"], "pose"), cfg.gravity_scale, cfg.torque_limit_nm)
        current = vector6(poses[-1]["q"], "pose")


def nominal_duration(plan, speed):
    previous, duration = vector6(plan["start"], "start"), 0.
    for pose in plan["poses"]:
        goal = vector6(pose["q"], "pose")
        duration += movement_duration(previous, goal, speed)
        if pose["stage"] == "sample":
            duration += SETTLE_S + SAMPLE_S
        previous = goal
    return duration


def run_calibration(output_dir=None, side="right", start=None, config=ARM_BINDINGS,
                    settings_path=CONTROL_SETTINGS, speed=SAMPLING_SPEED, *, bus_factory=MotorBus):
    """Run both automatic sampling rounds, fitting, and independent validation."""
    if side not in SIDES:
        raise ValueError("Select one arm.")
    positive(speed, "speed")
    model, cfg = GravityModel(), load_settings(settings_path)
    root = (Path(output_dir) if output_dir is not None else
            RUNS / "calibration" / side / datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ"))
    plans = [root / "plan.yaml", root / "validation_plan.yaml"]
    if root.exists():
        raise ValueError("Run directory already exists; choose a new output directory.")
    start = (read_current_pose(config, side, bus_factory=bus_factory) if start is None
             else vector6(start, "start"))
    print("Planning fixed elbow-up joint sweeps...", flush=True)
    generated = [make_plan(start, cfg), make_plan(clearance_pose(start[0]), cfg, independent=True)]
    durations = [nominal_duration(plan, speed) for plan in generated]
    root.mkdir(parents=True, exist_ok=False)
    for path, plan in zip(plans, generated):
        write_yaml(path, plan)
    write_yaml(root / "session.yaml", {"version": 1, "side": side, "start": plain(start),
               "method": "paired_static", "clearance": clearance_pose(start[0]).tolist(),
               "nominal_round_duration_s": durations,
               "model_sha256": model.sha256, "settings_sha256": digest(settings_path)})
    print(f"Calibration run: {root.resolve()}", flush=True)
    print(f"Paired static sampling: 24 targets, then 12 independent targets; "
          f"nominal {durations[0]/60:.1f} + {durations[1]/60:.1f} min, "
          "plus additional settling if needed.", flush=True)
    report = {"version": 1, "accepted": False, "stage": "preflight", "status": "checking"}
    # One stop object spans every stage, so a signal between stages also ends the workflow.
    stop = StopSignal()
    options = {"side": side, "config": config, "settings_path": settings_path, "speed": speed,
               "bus_factory": bus_factory, "stop": stop}

    def sampled(stage, plan, output):
        report.update(stage=stage, status="running")
        if stop.is_set() or collect(plan, output, **options) != "completed":
            report["status"] = "interrupted"
            return False
        return True

    try:
        with stop_signals(stop):
            preflight(plans, side, config, settings_path, bus_factory=bus_factory)
            if not sampled("sampling", plans[0], root / "data.jsonl"):
                return report
            report["stage"] = "fitting"
            report["fit"] = fitted = fit(root / "data.jsonl", root / "candidate")
            if not fitted["accepted"]:
                report["status"] = "rejected"
                print("Fit rejected: " + fitted.get("reason", "See the fit report."), flush=True)
                return report
            if not sampled("independent_sampling", plans[1], root / "independent.jsonl"):
                return report
            report["stage"] = "independent_validation"
            candidate = root / "candidate" / "calibration.yaml"
            report["independent_validation"] = result = validate(root / "independent.jsonl", candidate)
            if not result["accepted"]:
                report["status"] = "rejected"
                print("Independent validation rejected: " + " ".join(result.get("reasons", [])), flush=True)
                return report
            profile = yaml.safe_load(candidate.read_text())
            profile["independent_validation"] = result
            profile["independent_dataset_sha256"] = digest(root / "independent.jsonl")
            with (root / "calibrated.urdf").open("xb") as stream:
                stream.write((root / "candidate" / "calibrated.urdf").read_bytes())
            write_yaml(root / "calibration.yaml", profile)
            report.update(stage="complete", status="completed", accepted=True)
            print(f"Calibration passed. Profile: {root / 'calibration.yaml'}", flush=True)
            return report
    except BaseException as exc:
        report.update(status="interrupted" if isinstance(exc, KeyboardInterrupt) else "error", error=str(exc))
        raise
    finally:
        with (root / "report.json").open("x") as stream:
            json.dump(plain(report), stream, indent=2, allow_nan=False)
        print(f"Workflow {report['status']} at {report['stage']}; report: {root / 'report.json'}",
              flush=True)


def main(argv=None, prog=None):
    parser = argparse.ArgumentParser(prog=prog, description=__doc__)
    parser.add_argument("--side", choices=SIDES, default="right")
    parser.add_argument("--speed", type=float, default=SAMPLING_SPEED, help="Peak planned joint speed in rad/s")
    parser.add_argument("--config", type=Path, default=ARM_BINDINGS)
    parser.add_argument("--settings", type=Path, default=CONTROL_SETTINGS)
    args = parser.parse_args(argv)
    try:
        result = run_calibration(side=args.side, speed=args.speed, config=args.config,
                                 settings_path=args.settings)
    except (KeyError, TypeError) as exc:
        # Malformed plan or dataset records.
        raise ValueError(f"Invalid calibration data: {exc!r}") from exc
    if result["status"] == "interrupted":
        return 130
    return 0 if result["accepted"] else 2
