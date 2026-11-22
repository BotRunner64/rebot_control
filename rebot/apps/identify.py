"""Automatic single-arm gravity calibration: sample, fit and save mass scales, optionally verify."""

import argparse
from contextlib import ExitStack
from dataclasses import replace
from datetime import datetime, timezone
import json
from pathlib import Path
import time

import numpy as np
import yaml

from rebot.bindings import SIDES, load_arms, select_arms
from rebot.devices.robstride import MotorBus
from rebot.model.arrays import vector6
from rebot.model.gravity import GravityModel, check_binding, digest
from rebot.model.gravity_fit import (CLEARANCE, SAMPLE_S, SETTLE_S, candidate_scales, fit, load_dataset,
                                     make_plan, path_check, planned_entry, positive, validate,
                                     validate_plan, write_yaml)
from rebot.model.gravity_trials import compare_holds, compare_payload
from rebot.model.motion_collision import MotionCollision
from rebot.model.trajectory import (SAMPLING_SPEED, MotionLog, MotionStats, TrajectoryTracker,
                                    minimum_jerk, movement_duration, plain)
from rebot.paths import ARM_BINDINGS, CONTROL_SETTINGS, RUNS
from rebot.runtime.arm import Arm
from rebot.runtime.console import StopKey, StopSignal, stop_signals
from rebot.settings import load_settings


def collect(plan_path, output, side="right", config=ARM_BINDINGS, settings_path=CONTROL_SETTINGS,
            speed=SAMPLING_SPEED, settle_s=SETTLE_S, sample_s=SAMPLE_S, *,
            bus_factory=MotorBus, stop=None, experiment="identification", candidate=None, payload=None, expected_arm=None):
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
    binding = select_arms(config, side)[side]
    if expected_arm is not None:
        check_binding(expected_arm, binding)
    if candidate is not None:
        model.apply_scales(candidate_scales(candidate, binding))
        cfg = replace(cfg, gravity_scale=np.ones(6))
    poses = validate_plan(plan, model, cfg.gravity_scale, cfg.torque_limit_nm)
    budget_model = None
    if payload is not None:
        budget_model = GravityModel()
        budget_model.add_point_payload(payload["mass_kg"], payload["com_link6_m"])
        validate_plan(plan, budget_model, np.ones(6), cfg.torque_limit_nm)

    def check_entry(collision, q):
        collision.check_plan(q, poses[:1])
        path_check(model, q, first, cfg.gravity_scale, cfg.torque_limit_nm)
        if budget_model is not None:
            path_check(budget_model, q, first, np.ones(6), cfg.torque_limit_nm)

    arm = Arm(binding, cfg, gravity=model, joint_limits=False, bus_factory=bus_factory)
    first = vector6(poses[0]["q"], "pose")
    stop = StopSignal() if stop is None else stop
    status = "error"
    with stop_signals(stop), ExitStack() as stack:
        journal = stack.enter_context(MotionLog(output))
        stats = MotionStats()
        stack.callback(stats.report, side)
        journal.write({"type": "metadata", "version": 3, "control": "native_mit", "side": side,
                       "experiment": experiment, "payload": payload, "applied_mass_scales": model.scales,
                       "motor_uids": binding.motor_uids, "reference_rad": binding.reference[:6],
                       "model_sha256": model.sha256, "plan": plan,
                       "settings": {**cfg.plain(), "hold_kp": arm.kp, "damping": arm.kd},
                       "speed": speed, "settle_s": settle_s, "sample_s": sample_s,
                       "effort_note": "Native PD effort is estimated from decoded commands and measured pose; no independent torque sensor."})
        # Register the terminal record before the bus, so freeze runs before final logging.
        stack.callback(lambda: journal.write({"type": "end", "status": status}))
        try:
            collision = workstation_check(config, side, bus_factory)
            collision.check_plan(first, poses[1:])
            stack.enter_context(arm)
            keys = stack.enter_context(StopKey())

            def interrupted():
                return stop.is_set() or keys.pressed()

            check_entry(collision, arm.q)
            if interrupted():
                return (status := "interrupted")
            arm.prepare()
            trajectory_target = arm.sample().copy()
            check_entry(collision, trajectory_target)
            if interrupted():
                return (status := "interrupted")
            arm.enable()
            if poses[0]["stage"] != "entry":
                collision.check_segment(arm.q, trajectory_target, poses[0]["id"])
            tracker = TrajectoryTracker(model, cfg)
            print("AUTOMATIC SAMPLING: Space/Esc/Q/Ctrl-C stops and holds. "
                  "Keep the documented tool/load fixed; no external support. "
                  "Keep the other arm stationary throughout sampling.", flush=True)
            print("Native MIT Kp: " + " ".join(f"{gain:g}" for gain in arm.kp), flush=True)
            session_start = time.monotonic()
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
                    if sampling and tick - began > duration + settle_s + sample_s + 15:
                        raise RuntimeError(f"Waypoint {pose['id']} did not settle; stopping and holding.")
                    q = arm.sample()
                    collision.check_pose(q, pose["id"])
                    if budget_model is not None and np.any(np.abs(budget_model.gravity(q)) > cfg.torque_limit_nm):
                        raise RuntimeError("Loaded pose exceeds gravity torque budget.")
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


def workstation_check(config, side, bus_factory):
    other = "left" if side == "right" else "right"
    parked = None
    if other in load_arms(config):
        binding = select_arms(config, "both")[other]
        with bus_factory(binding.channel, binding.models, motor_count=7) as bus:
            binding.verify(bus)
            parked = np.asarray(bus.positions()) - binding.reference
    else:
        print(f"Single-arm layout: no {other} arm is bound or included in the collision check.", flush=True)
    return MotionCollision(side, parked)


def preflight(plans, side="right", config=ARM_BINDINGS, settings_path=CONTROL_SETTINGS, *,
              bus_factory=MotorBus):
    """Check identity and both paths internally before starting automatic motion."""
    cfg, model = load_settings(settings_path), GravityModel()
    current = read_current_pose(config, side, bus_factory=bus_factory)
    collision = workstation_check(config, side, bus_factory)
    for path in plans:
        poses = validate_plan(yaml.safe_load(Path(path).read_text()), model,
                              cfg.gravity_scale, cfg.torque_limit_nm)
        path_check(model, current, vector6(poses[0]["q"], "pose"), cfg.gravity_scale, cfg.torque_limit_nm)
        collision.check_plan(current, poses)
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
                    settings_path=CONTROL_SETTINGS, speed=SAMPLING_SPEED, verify=False, max_rmse=.25, *,
                    bus_factory=MotorBus):
    """Sample and fit one round, always saving a usable fit; `verify` adds independent rounds."""
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
    print("Planning entry to the fixed workstation sweeps...", flush=True)
    generated = [make_plan(start, cfg), make_plan(CLEARANCE, cfg, independent=True)]
    durations = [nominal_duration(plan, speed) for plan in generated]
    root.mkdir(parents=True, exist_ok=False)
    for path, plan in zip(plans, generated):
        write_yaml(path, plan)
    write_yaml(root / "session.yaml", {"version": 1, "side": side, "start": plain(start),
               "method": "paired_static", "clearance": generated[0]["clearance"],
               "nominal_round_duration_s": durations,
               "model_sha256": model.sha256, "settings_sha256": digest(settings_path)})
    print(f"Calibration run: {root.resolve()}", flush=True)
    if verify:
        print(f"Paired static sampling: 24 targets, then 12 independent targets with each feedforward; "
              f"nominal {durations[0]/60:.1f} + 2 x {durations[1]/60:.1f} min, "
              "plus additional settling if needed.", flush=True)
    else:
        print(f"Paired static sampling: 24 targets, nominal {durations[0]/60:.1f} min plus settling. "
              "Add --verify for independent on-robot checks.", flush=True)
    report = {"version": 1, "accepted": False, "stage": "preflight", "status": "checking"}
    # One stop object spans every stage, so a signal between stages also ends the workflow.
    stop = StopSignal()
    options = {"side": side, "config": config, "settings_path": settings_path, "speed": speed,
               "bus_factory": bus_factory, "stop": stop}

    def sampled(stage, plan, output, **trial_options):
        report.update(stage=stage, status="running")
        if stop.is_set() or collect(plan, output, **options, **trial_options) != "completed":
            report["status"] = "interrupted"
            return False
        return True

    try:
        with stop_signals(stop):
            preflight(plans, side, config, settings_path, bus_factory=bus_factory)
            if not sampled("sampling", plans[0], root / "data.jsonl"):
                return report
            report["stage"] = "fitting"
            report["fit"] = fitted = fit(root / "data.jsonl", root / "fit", max_rmse=max_rmse)
            if not fitted["accepted"]:
                report["status"] = "rejected"
                print("Fit rejected: " + fitted.get("reason", "See the fit report."), flush=True)
                return report
            fitted_path = root / "fit" / "calibration.yaml"
            profile = yaml.safe_load(fitted_path.read_text())
            model.export_urdf(profile["mass_scales"], root / "calibrated.urdf")
            exported = GravityModel(root / "calibrated.urdf")
            if any(not np.allclose(exported.gravity(q), model.basis(q) @ profile["mass_scales"],
                                   rtol=1e-8, atol=1e-8) for q in profile["identification_targets_rad"]):
                raise RuntimeError("Exported URDF does not reproduce fitted gravity.")
            write_yaml(root / "calibration.yaml", profile)
            report["warnings"] = list(fitted["warnings"])
            for warning in fitted["warnings"]:
                print("Warning: " + warning, flush=True)
            print("Mass scales: " + " ".join(f"{x:.3f}" for x in profile["mass_scales"]), flush=True)
            print(f"Profile saved: {root / 'calibration.yaml'}", flush=True)
            if verify:
                # Optional on-robot checks: they annotate the saved profile but never remove it.
                if not sampled("independent_sampling", plans[1], root / "independent.jsonl", experiment="baseline"):
                    return report
                report["stage"] = "independent_validation"
                report["independent_validation"] = result = validate(
                    root / "independent.jsonl", fitted_path, max_rmse=max_rmse)
                if not sampled("candidate_sampling", plans[1], root / "trial.jsonl",
                               experiment="trial", candidate=profile):
                    return report
                report["stage"] = "hold_comparison"
                report["hold_comparison"] = holds = compare_holds(
                    root / "independent.jsonl", root / "trial.jsonl", profile, max_rmse=max_rmse)
                for name, stage in (("Independent validation", result), ("Hold comparison", holds)):
                    report["warnings"] += [f"{name}: {reason}" for reason in stage["reasons"]]
                    for reason in stage["reasons"]:
                        print(f"Warning: {name}: {reason}", flush=True)
                profile.update(verified=result["accepted"] and holds["accepted"],
                               warnings=report["warnings"], independent_validation=result,
                               hold_comparison=holds,
                               independent_dataset_sha256=digest(root / "independent.jsonl"),
                               trial_dataset_sha256=digest(root / "trial.jsonl"))
                (root / "calibration.yaml").unlink()
                write_yaml(root / "calibration.yaml", profile)
                print("Verification " + ("passed." if profile["verified"] else "found issues; profile kept."),
                      flush=True)
            report.update(stage="complete", status="completed", accepted=True)
            return report
    except BaseException as exc:
        report.update(status="interrupted" if isinstance(exc, KeyboardInterrupt) else "error", error=str(exc))
        raise
    finally:
        with (root / "report.json").open("x") as stream:
            json.dump(plain(report), stream, indent=2, allow_nan=False)
        print(f"Workflow {report['status']} at {report['stage']}; report: {root / 'report.json'}",
              flush=True)


def payload_session(output, source=None, mass=None, com=None, *, side="right", config=ARM_BINDINGS,
                    settings_path=CONTROL_SETTINGS, speed=SAMPLING_SPEED, bus_factory=MotorBus):
    """Collect an empty or known-load session, with identical sampling targets."""
    root = Path(output)
    if root.exists():
        raise ValueError("Choose a new output directory for this session.")
    cfg = load_settings(settings_path)
    positive(speed, "speed")
    payload, reference = None, None
    if source is not None:
        empty_path = Path(source) / "data.jsonl"
        reference, rows = load_dataset(empty_path)
        if (reference.get("experiment") != "payload_empty" or len(rows) != 12
                or reference["speed"] != speed or reference["side"] != side
                or any(reference["settings"].get(k) != v for k, v in cfg.plain().items()
                       if k not in ("hold_kp", "damping"))):
            raise ValueError("Use the completed empty payload run with the same arm, speed and settings.")
        check_binding(reference, select_arms(config, side)[side])
        model = GravityModel()
        model.add_point_payload(mass, com)
        payload = {"mass_kg": mass, "com_link6_m": list(com)}
    start = read_current_pose(config, side, bus_factory=bus_factory)
    if reference is None:
        plan = make_plan(start, cfg, independent=True)
    else:
        plan = dict(reference["plan"], start=start.tolist())
        nominal = GravityModel()

        def check_segment(a, b):
            path_check(nominal, a, b, cfg.gravity_scale, cfg.torque_limit_nm)
            path_check(model, a, b, np.ones(6), cfg.torque_limit_nm)

        clearance = vector6(plan["clearance"], "reference clearance")
        entry = planned_entry(start, clearance[0], check_segment, nominal)
        last = entry[-1] if entry else start
        if not np.array_equal(last, clearance):
            check_segment(last, clearance)
            entry.append(clearance)
        poses = [{"stage": "entry", "q": q.tolist()} for q in entry]
        poses.extend(p for p in plan["poses"] if p["stage"] != "entry")
        plan["poses"] = [dict(p, id=i) for i, p in enumerate(poses)]
        validate_plan(plan, model, np.ones(6), cfg.torque_limit_nm)
        workstation_check(config, side, bus_factory).check_plan(start, plan["poses"])
    root.mkdir(parents=True, exist_ok=False)
    write_yaml(root / "plan.yaml", plan)
    status = collect(root / "plan.yaml", root / "data.jsonl", side=side, config=config, settings_path=settings_path,
                     speed=speed, payload=payload, expected_arm=reference, bus_factory=bus_factory,
                     experiment="payload_empty" if source is None else "payload_loaded")
    if status != "completed":
        return 130
    if source is None:
        _, rows = load_dataset(root / "data.jsonl")
        if len(rows) != 12:
            raise ValueError("Empty reference requires 12 complete pairs; repeat in a new directory.")
    if source is not None:
        result = compare_payload(empty_path, root / "data.jsonl", mass, com)
        with (root / "payload_report.json").open("x") as stream:
            json.dump(result, stream, indent=2, allow_nan=False)
        print(f"Payload comparison: {root / 'payload_report.json'}")
    else:
        print(f"Empty reference collected: {root}")
    return 0


def main(argv=None, prog=None):
    common = argparse.ArgumentParser(add_help=False, argument_default=argparse.SUPPRESS)
    common.add_argument("--side", choices=SIDES)
    common.add_argument("--speed", type=float, help=f"Peak joint speed in rad/s (default: {SAMPLING_SPEED:g})")
    common.add_argument("--config", type=Path)
    common.add_argument("--settings", type=Path)
    parser = argparse.ArgumentParser(prog=prog, description=__doc__, parents=[common])
    modes = parser.add_subparsers(dest="mode")
    for mode, desc in (("payload-empty", "Collect the empty-tool reference"),
                       ("payload-loaded", "Repeat with a measured rigid payload")):
        sub = modes.add_parser(mode, help=desc, parents=[common])
        sub.add_argument("--output", type=Path, required=True)
        if mode == "payload-loaded":
            sub.add_argument("--empty", type=Path, required=True, help="Completed payload-empty run directory")
            sub.add_argument("--mass-kg", type=float, required=True)
            sub.add_argument("--com-m", type=float, nargs=3, required=True, metavar=("X", "Y", "Z"),
                             help="Added payload center of mass in the URDF link6 frame, metres")
    parser.add_argument("--verify", action="store_true",
                        help="Also run two independent rounds that check the saved fit on the robot")
    parser.add_argument("--max-rmse", type=float, default=.25,
                        help="Advisory per-joint RMSE in N*m; exceeding it only warns")
    replay = modes.add_parser("replay", help="Fit a completed identification log offline")
    replay.add_argument("data", type=Path)
    replay.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        if args.mode == "replay":
            fitted = fit(args.data, args.output, max_rmse=args.max_rmse)
            for warning in fitted.get("warnings", []):
                print("Warning: " + warning)
            print(f"Offline fit: {args.output / 'report.json'}")
            if fitted["accepted"]:
                print(f"Profile: {args.output / 'calibration.yaml'}")
            return 0 if fitted["accepted"] else 2
        options = dict(side=getattr(args, "side", "right"), speed=getattr(args, "speed", SAMPLING_SPEED),
                       config=getattr(args, "config", ARM_BINDINGS),
                       settings_path=getattr(args, "settings", CONTROL_SETTINGS))
        if args.mode == "payload-empty":
            return payload_session(args.output, **options)
        if args.mode == "payload-loaded":
            return payload_session(args.output, args.empty, args.mass_kg, args.com_m, **options)
        result = run_calibration(**options, verify=args.verify, max_rmse=args.max_rmse)
    except (KeyError, TypeError) as exc:
        # Malformed plan or dataset records.
        raise ValueError(f"Invalid calibration data: {exc!r}") from exc
    if result["status"] == "interrupted":
        return 130
    return 0 if result["accepted"] else 2
