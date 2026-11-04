#!/usr/bin/env python3
"""Run automatic single-arm gravity calibration and independent validation."""

import argparse
from contextlib import ExitStack
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import signal
import subprocess
import sys
import time

import can
import numpy as np
from scipy.optimize import least_squares
import yaml

from rebot import CONFIG, load_config, resolve_channel, verify_identity
from rebot_gravity import GravityModel, digest, vector
from rebot_hand_guiding import SETTINGS_PATH, StopKey, load_settings
from rebot_mit import MotorBus
from rebot_motion import (MotionController, MotionStats, SAMPLING_SPEED,
                          minimum_jerk, motion_settings, movement_duration)


SETTLE_S = .7
SAMPLE_S = 1.
APPROACH_RAD = .08
CLEARANCE = np.array([0., .7, 1.1, 0., 0., 0.])
# Joint numbers and absolute angles in the upright URDF frame.
SWEEPS = ((2, (.45, .65, .85, 1.05, 1.25, 1.45)),
          (3, (.65, .9, 1.15, 1.4, 1.65, 1.9)),
          (4, (-.6, -.36, -.12, .12, .36, .6)),
          (5, (-.8, -.48, -.16, .16, .48, .8)))
VALIDATION_SWEEPS = ((2, (.55, .95, 1.35)), (3, (.8, 1.3, 1.8)),
                     (4, (-.48, 0., .48)), (5, (-.64, 0., .64)))


def clearance_pose(yaw):
    q = CLEARANCE.copy()
    q[0] = yaw
    return q


def entry_path(start):
    q = vector(start, "start").copy()
    result = []
    for axis, angle in ((2, max(q[2], 1.1)), (1, .7), (3, 0.), (4, 0.), (5, 0.), (2, 1.1)):
        if abs(q[axis] - angle) > 1e-8:
            q = q.copy()
            q[axis] = angle
            result.append(q)
    return result


def plain(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, dict):
        return {key: plain(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [plain(item) for item in value]
    if isinstance(value, np.generic):
        return value.item()
    return value


def write_yaml(path, value):
    with Path(path).open("x") as stream:
        yaml.safe_dump(plain(value), stream, sort_keys=False)


def positive(value, name, upper=math.inf):
    if not math.isfinite(value) or not 0 < value <= upper:
        raise ValueError(f"{name} must be finite, positive, and at most {upper}.")
    return value


def path_check(model, start, goal, cfg):
    steps = max(2, int(np.max(np.abs(goal - start)) / .02) + 2)
    for q in np.linspace(start, goal, steps):
        if np.any(q < model.lower) or np.any(q > model.upper):
            raise ValueError("Planned path exceeds joint limits.")
        if np.any(np.abs(model.gravity(q) * cfg["gravity_scale"]) > .9 * cfg["torque_limit_nm"]):
            raise ValueError("Planned path exceeds the 90% model-gravity torque budget.")


def make_plan(start, independent=False, settings_path=SETTINGS_PATH):
    start = vector(start, "start")
    model, cfg = GravityModel(), load_settings(settings_path)
    model.validate(start)
    home = clearance_pose(start[0])
    poses = []

    def append(q, stage, **fields):
        poses.append({"id": len(poses), "q": q.tolist(), "stage": stage, **fields})

    for q in entry_path(start):
        append(q, "entry")
    pair_id = 0
    for joint, angles in VALIDATION_SWEEPS if independent else SWEEPS:
        axis = joint - 1
        append(home.copy(), "transit")
        for direction in (1, -1):
            turn = home.copy()
            turn[axis] = angles[0] - APPROACH_RAD if direction == 1 else angles[-1] + APPROACH_RAD
            append(turn, "transit")
            indices = range(len(angles)) if direction == 1 else reversed(range(len(angles)))
            for index in indices:
                goal = home.copy()
                goal[axis] = angles[index]
                split = "validation" if independent or index % 3 == 1 else "train"
                append(goal, "sample", pair_id=pair_id + index, axis=joint,
                       direction=direction, split=split)
        append(home.copy(), "transit")
        pair_id += len(angles)
    plan = {"version": 2, "method": "paired_static", "model_sha256": model.sha256,
            "units": "rad", "start": start.tolist(), "clearance": home.tolist(), "poses": poses}
    validate_plan(plan, model, cfg)
    return plan


def validate_plan(plan, model, cfg):
    if (not isinstance(plan, dict) or plan.get("version") != 2 or plan.get("units") != "rad"
            or plan.get("method") != "paired_static" or plan.get("model_sha256") != model.sha256):
        raise ValueError("Expected a version 2 paired-static plan matching the model; old random plans are unsupported.")
    poses = plan.get("poses")
    if not isinstance(poses, list) or not poses:
        raise ValueError("Plan requires waypoints.")
    previous = vector(plan["start"], "start")
    ids, pairs = set(), {}
    measuring = False
    for pose in poses:
        if type(pose.get("id")) is not int or pose["id"] in ids:
            raise ValueError("Waypoint IDs must be unique integers.")
        ids.add(pose["id"])
        stage = pose.get("stage")
        if stage not in ("entry", "transit", "sample") or (measuring and stage == "entry"):
            raise ValueError("Entry waypoints must precede all measurement waypoints.")
        measuring |= stage != "entry"
        q = vector(pose["q"], "pose")
        path_check(model, previous, q, cfg)
        if stage == "sample":
            if (type(pose.get("pair_id")) is not int or pose.get("direction") not in (-1, 1)
                    or pose.get("split") not in ("train", "validation") or pose.get("axis") not in (2, 3, 4, 5)):
                raise ValueError("Samples require pair IDs, a swept joint, approach directions, and splits.")
            axis = pose["axis"] - 1
            delta = q - previous
            if pose["direction"] * delta[axis] <= 0 or not np.allclose(np.delete(delta, axis), 0., atol=1e-8):
                raise ValueError("Each sample must follow a single-joint approach in its specified direction.")
            pairs.setdefault(pose["pair_id"], []).append(pose)
        previous = q
    for pair in pairs.values():
        if (len(pair) != 2 or {p["direction"] for p in pair} != {-1, 1}
                or pair[0]["split"] != pair[1]["split"] or pair[0]["axis"] != pair[1]["axis"]
                or not np.array_equal(pair[0]["q"], pair[1]["q"])):
            raise ValueError("Each pair needs opposite approaches to the same target and joint in one split.")
    if not pairs:
        raise ValueError("Plan requires paired measurements.")
    return poses


def collect(plan_path, output, side="right", config=CONFIG,
            settings_path=SETTINGS_PATH, speed=SAMPLING_SPEED, settle_s=SETTLE_S, sample_s=SAMPLE_S):
    positive(speed, "speed")
    positive(settle_s, "settle_s", 10)
    positive(sample_s, "sample_s", 10)
    if settle_s < .5 or sample_s < .5:
        raise ValueError("Settle/sample durations must be >= 0.5 s.")
    if side not in ("right", "left"):
        raise ValueError("Collect one arm at a time.")
    cfg, model = motion_settings(load_settings(settings_path)), GravityModel()
    plan = yaml.safe_load(Path(plan_path).read_text())
    poses = validate_plan(plan, model, cfg)
    arms = load_config(config)
    if side not in arms or "reference_rad" not in arms[side]:
        raise ValueError("Bind and calibrate the selected arm first.")
    arm = arms[side]
    reference = vector(arm["reference_rad"][:6], "reference_rad")
    stopped = False

    def stop(signum, frame):
        nonlocal stopped
        stopped = True

    old_signals = {sig: signal.signal(sig, stop)
                   for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)}
    try:
        with ExitStack() as stack:
            stream = stack.enter_context(Path(output).open("x"))
            stats = MotionStats()
            stack.callback(stats.report, side)
            last_flush = time.monotonic()

            def log(record):
                nonlocal last_flush
                stream.write(json.dumps(plain(record), allow_nan=False) + "\n")
                now = time.monotonic()
                if record["type"] != "sample" or now - last_flush >= 1:
                    stream.flush()
                    last_flush = now

            log({"type": "metadata", "version": 3, "control": "native_mit", "side": side,
                 "motor_uids": arm["motor_uids"], "reference_rad": reference,
                 "model_sha256": model.sha256, "plan": plan, "settings": cfg,
                 "speed": speed, "settle_s": settle_s, "sample_s": sample_s,
                 "effort_note": "Native PD effort is estimated from decoded commands and measured pose; no independent torque sensor."})
            status = "error"
            try:
                # Register the terminal record before the bus, so freeze runs before final logging.
                stack.callback(lambda: log({"type": "end", "status": status}))
                bus = stack.enter_context(MotorBus(resolve_channel(arm), arm["models"],
                                                  max_temperature=cfg["max_temperature_c"]))
                verify_identity(bus, arm)
                raw = np.asarray(bus.positions())
                current = raw - reference
                path_check(model, current, vector(poses[0]["q"], "pose"), cfg)
                keys = stack.enter_context(StopKey())
                gravity = model.gravity(current) * cfg["gravity_scale"]
                bus.set_hold(raw, gravity, cfg["hold_kp"], cfg["damping"])
                bus.hold_gravity = lambda positions: np.clip(
                    model.gravity(np.asarray(positions) - reference) * cfg["gravity_scale"],
                    -cfg["torque_limit_nm"], cfg["torque_limit_nm"])
                if stopped or keys.pressed():
                    status = "interrupted"
                    return status
                bus.prepare()
                raw = np.asarray(bus.positions())
                current = raw - reference
                path_check(model, current, vector(poses[0]["q"], "pose"), cfg)
                gravity = model.gravity(current) * cfg["gravity_scale"]
                bus.set_hold(raw, gravity, cfg["hold_kp"], cfg["damping"])
                if stopped or keys.pressed():
                    status = "interrupted"
                    return status
                bus.enable(raw)
                sampler = MotionController(model, cfg)
                print("AUTOMATIC SAMPLING: Space/Esc/Q/Ctrl-C stops and holds. "
                      "Closed unloaded gripper; no external support. "
                      "Fixed elbow-up sweeps; no collision avoidance.", flush=True)
                print("Native MIT Kp: " + " ".join(f"{gain:g}" for gain in cfg["hold_kp"]), flush=True)
                session_start = time.monotonic()
                trajectory_target = current.copy()
                for pose in poses:
                    start = trajectory_target.copy()
                    goal = vector(pose["q"], "pose")
                    duration = movement_duration(start, goal, speed)
                    began = time.monotonic()
                    quiet_since = sample_since = None
                    window = 0
                    label = (f"pair {pose['pair_id']} {pose['direction']:+d}, {pose['split']}"
                             if pose["stage"] == "sample" else pose["stage"])
                    print(f"Waypoint {pose['id']} ({label}): move {duration:.1f}s" +
                          (", then settle/sample." if pose["stage"] == "sample" else "."), flush=True)
                    while True:
                        tick = time.monotonic()
                        if stopped or keys.pressed():
                            status = "interrupted"
                            return status
                        progress = min(1., (tick - began) / duration)
                        target, desired_velocity = minimum_jerk(start, goal, tick - began, duration)
                        sampled = time.monotonic()
                        raw = np.asarray(bus.positions())
                        current = raw - reference
                        now = time.monotonic()
                        record = sampler.update(current, target, now, desired_velocity)
                        bus.set_hold(raw, record["gravity"], cfg["hold_kp"], cfg["damping"])
                        if stopped or keys.pressed():
                            status = "interrupted"
                            return status
                        sampler.command(bus, raw, record, sampled + cfg["max_feedback_age_s"])
                        stats.update(record)
                        ready = pose["stage"] == "sample" and progress == 1 and record["stable"]
                        if ready:
                            quiet_since = now if quiet_since is None else quiet_since
                            if now - quiet_since >= settle_s and sample_since is None:
                                sample_since = now
                        else:
                            quiet_since = sample_since = None
                            window += 1
                        record.update({"type": "sample", "pose_id": pose["id"], "window": window,
                                       "time_s": now - session_start, "eligible": sample_since is not None,
                                       "phase": "MOVE" if progress < 1 or pose["stage"] != "sample" else
                                       ("SAMPLE" if sample_since is not None else "SETTLE"),
                                       "reported": [bus.feedback[i]["torque"] for i in range(1, 7)],
                                       "temperature": [bus.feedback[i]["temperature"] for i in range(1, 7)]})
                        log(record)
                        if progress == 1 and pose["stage"] != "sample":
                            trajectory_target = goal.copy()
                            break
                        if sample_since is not None and now - sample_since >= sample_s:
                            log({"type": "pose_complete", "pose_id": pose["id"], "window": window})
                            trajectory_target = goal.copy()
                            break
                        time.sleep(max(0, 1 / cfg["rate_hz"] - (time.monotonic() - tick)))
                status = "completed"
            except BaseException as exc:
                log({"type": "error", "message": str(exc)})
                raise
    finally:
        for sig, handler in old_signals.items():
            signal.signal(sig, handler)
    return status


def load_dataset(path):
    with Path(path).open() as stream:
        rows = [json.loads(line) for line in stream if line.strip()]
    if (not rows or rows[0].get("type") != "metadata" or rows[0].get("version") != 3
            or rows[0].get("control") != "native_mit"):
        raise ValueError("Dataset requires version 3 paired-static native MIT data; recollect old unpaired data.")
    meta = rows[0]
    model = GravityModel()
    if meta.get("model_sha256") != model.sha256:
        raise ValueError("Dataset model hash does not match.")
    cfg = meta["settings"]
    cfg["gravity_scale"] = vector(cfg["gravity_scale"], "gravity_scale")
    cfg["torque_limit_nm"] = vector(cfg["torque_limit_nm"], "torque_limit_nm")
    poses = {pose["id"]: pose for pose in validate_plan(meta["plan"], model, cfg)}
    completed = {(r["pose_id"], r["window"]) for r in rows if r.get("type") == "pose_complete"}
    samples, invalid = {}, set()
    for row in rows[1:]:
        if row.get("type") != "sample" or row.get("eligible") is not True:
            continue
        key = (row["pose_id"], row["window"])
        if key not in completed or key[0] not in poses or poses[key[0]]["stage"] != "sample":
            continue
        if row.get("saturated") is not False or row.get("phase") != "SAMPLE":
            invalid.add(key)
            continue
        q, tau = vector(row["q"], "q"), vector(row["estimated"], "estimated")
        model.validate(q)
        if (np.max(np.abs(vector(row["velocity"], "velocity"))) > .015
                or np.max(np.abs(vector(row["raw_velocity"], "raw_velocity"))) > .02
                or np.max(np.abs(q - vector(poses[key[0]]["q"], "pose"))) > .08
                or np.any(np.abs(vector(row["requested"], "requested")) >= .98 * cfg["torque_limit_nm"])):
            invalid.add(key)
            continue
        samples.setdefault(key, []).append((q, tau, row["time_s"]))
    windows = {}
    for (pose_id, window), records in samples.items():
        if (pose_id, window) in invalid:
            continue
        q = np.array([r[0] for r in records])
        times = np.array([r[2] for r in records])
        if (len(records) < 5 or np.any(np.diff(times) <= 0)
                or np.max(np.diff(times)) > cfg["max_sample_gap_s"]
                or np.ptp(times) < .8 * meta["sample_s"] or np.max(np.std(q, axis=0)) > .01):
            continue
        if pose_id in windows:
            raise ValueError("Multiple completed sample windows for the same waypoint.")
        windows[pose_id] = {"q": np.median(q, axis=0),
                            "torque": np.median([r[1] for r in records], axis=0)}
    pairs = {}
    for pose_id, row in windows.items():
        pose = poses[pose_id]
        pairs.setdefault(pose["pair_id"], {})[pose["direction"]] = (pose, row)
    aggregated = []
    for pair_id, passes in pairs.items():
        if set(passes) != {-1, 1}:
            continue
        plus, minus = passes[1][1], passes[-1][1]
        bases = np.array([model.basis(row["q"]) for row in (plus, minus)])
        aggregated.append({"pair_id": pair_id, "split": passes[1][0]["split"],
                           "axis": passes[1][0]["axis"] - 1,
                           "q": (plus["q"] + minus["q"]) / 2,
                           "basis": bases.mean(axis=0),
                           "torque": (plus["torque"] + minus["torque"]) / 2,
                           "basis_difference": bases[0] - bases[1],
                           "torque_difference": plus["torque"] - minus["torque"]})
    return meta, aggregated


def metrics(rows, scales):
    if not rows:
        raise ValueError("No accepted pairs for validation.")
    measured = np.array([row["torque"][row["axis"]] for row in rows])
    matrices = np.array([row["basis"][row["axis"]] for row in rows])
    axes = np.array([row["axis"] for row in rows])
    before = matrices @ np.ones(6) - measured
    after = matrices @ scales - measured

    def per_joint(values, statistic):
        return [float(statistic(values[axes == axis])) if np.any(axes == axis) else None
                for axis in range(6)]

    hysteresis = np.array([(row["torque_difference"][row["axis"]]
                           - row["basis_difference"][row["axis"]] @ scales) / 2 for row in rows])
    return {"pairs": len(rows),
            "rmse_before_nm": per_joint(before**2, lambda x: np.sqrt(np.mean(x))),
            "rmse_after_nm": per_joint(after**2, lambda x: np.sqrt(np.mean(x))),
            "max_error_after_nm": per_joint(np.abs(after), np.max),
            "pooled_before_nm": float(np.sqrt(np.mean(before**2))),
            "pooled_after_nm": float(np.sqrt(np.mean(after**2))),
            "mean_residual_after_nm": per_joint(-after, np.mean),
            "median_approach_hysteresis_nm": per_joint(hysteresis, np.median)}


def validation_failures(result, max_rmse):
    before, after = (np.asarray(result[key], dtype=float)
                     for key in ("rmse_before_nm", "rmse_after_nm"))
    reasons = []
    missing = np.flatnonzero(~np.isfinite(after[1:5])) + 1
    if len(missing):
        reasons.append("Missing validation sweeps: " + ", ".join(f"J{i+1}" for i in missing) + ".")
    outside = np.flatnonzero(after > max_rmse)
    if len(outside):
        values = ", ".join(f"J{i+1}={after[i]:.3f}" for i in outside)
        reasons.append(f"Validation RMSE exceeds {max_rmse:g} N*m: {values}.")
    worse = np.flatnonzero(after > before + .02)
    if len(worse):
        values = ", ".join(f"J{i+1}: {before[i]:.3f} -> {after[i]:.3f}" for i in worse)
        reasons.append(f"Validation became worse: {values} N*m.")
    if result["pooled_after_nm"] > max(.05, .9 * result["pooled_before_nm"]):
        reasons.append(f"Held-out RMSE did not improve enough: {result['pooled_before_nm']:.3f} "
                       f"-> {result['pooled_after_nm']:.3f} N*m.")
    return reasons


def fit(data_path, output_dir, regularization=.01, max_rmse=.25):
    positive(regularization, "regularization")
    positive(max_rmse, "max_rmse")
    meta, rows = load_dataset(data_path)
    train = [r for r in rows if r["split"] == "train"]
    valid = [r for r in rows if r["split"] == "validation"]
    if len(train) < 12 or len(valid) < 4:
        raise ValueError("Need at least 12 complete training pairs and 4 held-out validation pairs.")
    if any(np.linalg.norm(a["q"] - b["q"]) < .02 for a in train for b in valid):
        raise ValueError("Training and validation poses overlap; collect distinct held-out poses.")
    model = GravityModel()
    basis = np.array([r["basis"][r["axis"]] for r in train])
    matrix = basis[:, 1:]
    target = np.array([r["torque"][r["axis"]] for r in train]) - basis[:, 0]
    singular = np.linalg.svd(matrix, compute_uv=False)
    rank = int(np.linalg.matrix_rank(matrix))
    condition = float(singular[0] / singular[-1]) if singular[-1] > 0 else None
    report = {"version": 1, "effort_source": "native_pd_estimate", "rank": rank, "condition": condition,
              "scope": "CAD body mass scales with fixed centers of mass; only the swept joint torque is fitted, with opposite passes averaged.",
              "accepted": False}
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=False)
    if rank != 5 or condition is None or condition > 1e4:
        report["reason"] = "Insufficient independent posture excitation for five mass scales."
    else:
        def residual(scales):
            return np.concatenate((matrix @ scales - target,
                                   math.sqrt(regularization) * (scales - 1)))

        solution = least_squares(residual, np.ones(5), bounds=(.5, 2.), loss="soft_l1", f_scale=.1)
        scales = np.r_[1., solution.x]
        report.update({"mass_scales": scales.tolist(), "solver_success": bool(solution.success),
                       "train": metrics(train, scales), "validation": metrics(valid, scales),
                       "max_rmse_nm": max_rmse})
        reasons = validation_failures(report["validation"], max_rmse)
        if not solution.success:
            reasons.insert(0, f"Solver failed: {solution.message}")
        bound_groups = [f"J{i}={value:.3f}" for i, value in enumerate(solution.x, 2)
                        if value <= .5001 or value >= 1.9999]
        if bound_groups:
            reasons.insert(0, "CAD body mass scales reached bounds: " + ", ".join(bound_groups) + ".")
        report["reasons"] = reasons
        report["accepted"] = not reasons
        if not report["accepted"]:
            report["reason"] = " ".join(reasons)
        else:
            profile = {"version": 2, "kind": "static_mass_scale", "accepted": True,
                       "model_sha256": model.sha256, "side": meta["side"],
                       "motor_uids": meta["motor_uids"], "reference_rad": meta["reference_rad"],
                       "mass_scales": scales.tolist(), "effort_source": "native_pd_estimate",
                       "method": "paired_static",
                       "identification_targets_rad": [r["q"].tolist() for r in rows],
                       "dataset_sha256": digest(data_path), "validation": report["validation"]}
            exported = output_dir / "calibrated.urdf"
            model.export_urdf(scales, exported)
            reloaded = GravityModel(exported)
            if any(not np.allclose(reloaded.gravity(r["q"]), model.basis(r["q"]) @ scales,
                                   rtol=1e-8, atol=1e-8) for r in rows):
                raise RuntimeError("Exported URDF does not reproduce the fitted gravity model.")
            write_yaml(output_dir / "calibration.yaml", profile)
    with (output_dir / "report.json").open("x") as stream:
        json.dump(plain(report), stream, indent=2, allow_nan=False)
    return report


def validate(data_path, calibration, max_rmse=.25):
    positive(max_rmse, "max_rmse")
    profile = yaml.safe_load(Path(calibration).read_text())
    # Loading through GravityModel enforces accepted status, source hash, and bounds.
    if profile.get("effort_source") != "native_pd_estimate" or profile.get("method") != "paired_static":
        raise ValueError("Calibration must use paired-static native PD effort estimates.")
    meta, rows = load_dataset(data_path)
    if profile.get("dataset_sha256") == digest(data_path):
        raise ValueError("Independent validation requires a different collection session.")
    if len(rows) < 8:
        raise ValueError("Independent validation requires at least 8 complete approach pairs.")
    prior_targets = [vector(q, "identification target") for q in profile["identification_targets_rad"]]
    if any(np.linalg.norm(row["q"] - q) < .02 for row in rows for q in prior_targets):
        raise ValueError("Independent validation requires targets distinct from the identification session.")
    arm = {"motor_uids": meta["motor_uids"], "reference_rad": meta["reference_rad"]}
    calibrated = GravityModel(calibration=calibration, arm=arm, side=meta["side"])
    result = metrics(rows, calibrated.scales)
    result["reasons"] = validation_failures(result, max_rmse)
    matrix = np.array([row["basis"][row["axis"], 1:] for row in rows])
    result["rank"] = int(np.linalg.matrix_rank(matrix))
    condition = float(np.linalg.cond(matrix))
    result["condition"] = condition if math.isfinite(condition) else None
    if result["rank"] != 5 or not math.isfinite(condition) or condition > 1e4:
        result["reasons"].append("Independent posture excitation is insufficient for five mass scales.")
    result["accepted"] = not result["reasons"]
    return result


def read_current_pose(config, side):
    arms = load_config(config)
    if side not in arms or "reference_rad" not in arms[side]:
        raise ValueError("Bind and calibrate the selected arm first.")
    arm = arms[side]
    with MotorBus(resolve_channel(arm), arm["models"]) as bus:
        verify_identity(bus, arm)
        return np.asarray(bus.positions()) - vector(arm["reference_rad"][:6], "reference_rad")


def preflight(plans, side="right", config=CONFIG, settings_path=SETTINGS_PATH):
    """Check identity and both paths internally before starting automatic motion."""
    cfg, model = load_settings(settings_path), GravityModel()
    current = read_current_pose(config, side)
    for path in plans:
        poses = validate_plan(yaml.safe_load(Path(path).read_text()), model, cfg)
        path_check(model, current, vector(poses[0]["q"], "pose"), cfg)
        current = vector(poses[-1]["q"], "pose")


def run_calibration(output_dir=None, side="right", start=None,
                    config=CONFIG, settings_path=SETTINGS_PATH, speed=SAMPLING_SPEED):
    """Run both automatic sampling rounds, fitting, and independent validation."""
    if side not in ("right", "left"):
        raise ValueError("Select one arm.")
    positive(speed, "speed")
    model = GravityModel()
    root = (Path(output_dir) if output_dir is not None else
            Path(__file__).with_name("calibration_runs") / side /
            datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ"))
    plans = [root / "plan.yaml", root / "validation_plan.yaml"]
    if root.exists():
        raise ValueError("Run directory already exists; choose a new output directory.")
    start = read_current_pose(config, side) if start is None else vector(start, "start")
    print("Planning fixed elbow-up joint sweeps...", flush=True)
    generated = [make_plan(start, settings_path=settings_path),
                 make_plan(clearance_pose(start[0]), independent=True, settings_path=settings_path)]
    durations = []
    for plan in generated:
        previous, duration = vector(plan["start"], "start"), 0.
        for pose in plan["poses"]:
            goal = vector(pose["q"], "pose")
            duration += movement_duration(previous, goal, speed)
            if pose["stage"] == "sample":
                duration += SETTLE_S + SAMPLE_S
            previous = goal
        durations.append(duration)
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

    def stop(signum, frame):
        # collect() temporarily installs its hold-preserving handler; stop between stages too.
        raise KeyboardInterrupt

    old_signals = {sig: signal.signal(sig, stop)
                   for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)}
    try:
        preflight(plans, side, config, settings_path)
        options = {"side": side, "config": config, "settings_path": settings_path, "speed": speed}
        report.update(stage="sampling", status="running")
        if collect(plans[0], root / "data.jsonl", **options) != "completed":
            report["status"] = "interrupted"
            return report
        report["stage"] = "fitting"
        fitted = fit(root / "data.jsonl", root / "candidate")
        report["fit"] = fitted
        if not fitted["accepted"]:
            report["status"] = "rejected"
            print("Fit rejected: " + fitted.get("reason", "See the fit report."), flush=True)
            return report
        report["stage"] = "independent_sampling"
        if collect(plans[1], root / "independent.jsonl", **options) != "completed":
            report["status"] = "interrupted"
            return report
        report["stage"] = "independent_validation"
        candidate = root / "candidate" / "calibration.yaml"
        result = validate(root / "independent.jsonl", candidate)
        report["independent_validation"] = result
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
        try:
            with (root / "report.json").open("x") as stream:
                json.dump(plain(report), stream, indent=2, allow_nan=False)
            print(f"Workflow {report['status']} at {report['stage']}; report: {root / 'report.json'}",
                  flush=True)
        finally:
            for sig, handler in old_signals.items():
                signal.signal(sig, handler)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--side", choices=("right", "left"), default="right")
    parser.add_argument("--speed", type=float, default=SAMPLING_SPEED, help="Peak planned joint speed in rad/s")
    args = parser.parse_args()
    try:
        result = run_calibration(side=args.side, speed=args.speed)
        if result["status"] == "interrupted":
            return 130
        return 0 if result["accepted"] else 2
    except (OSError, ValueError, RuntimeError, KeyError, TypeError, can.CanError,
            yaml.YAMLError, subprocess.SubprocessError) as exc:
        print(str(exc), file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
