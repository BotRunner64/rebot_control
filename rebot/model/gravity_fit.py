"""Paired-static gravity identification: sweep plans, dataset reduction, and mass-scale fitting.

Everything here is offline math on plans and recorded samples; the motion that
produces the samples lives in `rebot.apps.identify`.
"""

import json
import math
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
import yaml

from rebot.bindings import ArmBinding
from rebot.model.gravity import GravityModel, digest
from rebot.model.arrays import vector6 as vector
from rebot.model.trajectory import plain


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


def write_yaml(path, value):
    with Path(path).open("x") as stream:
        yaml.safe_dump(plain(value), stream, sort_keys=False)


def positive(value, name, upper=math.inf):
    if not math.isfinite(value) or not 0 < value <= upper:
        raise ValueError(f"{name} must be finite, positive, and at most {upper}.")
    return value


def path_check(model, start, goal, gravity_scale, torque_limit):
    """Reject a straight joint-space segment that leaves the limits or the torque budget."""
    steps = max(2, int(np.max(np.abs(goal - start)) / .02) + 2)
    for q in np.linspace(start, goal, steps):
        if np.any(q < model.lower) or np.any(q > model.upper):
            raise ValueError("Planned path exceeds joint limits.")
        if np.any(np.abs(model.gravity(q) * gravity_scale) > .9 * torque_limit):
            raise ValueError("Planned path exceeds the 90% model-gravity torque budget.")


def make_plan(start, settings, independent=False):
    """Fixed elbow-up single-joint sweeps, each target approached from both directions."""
    start = vector(start, "start")
    model = GravityModel()
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
    validate_plan(plan, model, settings.gravity_scale, settings.torque_limit_nm)
    return plan


def validate_plan(plan, model, gravity_scale, torque_limit):
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
        path_check(model, previous, q, gravity_scale, torque_limit)
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
    gravity_scale = vector(cfg["gravity_scale"], "gravity_scale")
    torque_limit = vector(cfg["torque_limit_nm"], "torque_limit_nm")
    poses = {pose["id"]: pose
             for pose in validate_plan(meta["plan"], model, gravity_scale, torque_limit)}
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
                or np.any(np.abs(vector(row["requested"], "requested")) >= .98 * torque_limit)):
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
    binding = ArmBinding(meta["side"], "", (), meta["motor_uids"], np.asarray(meta["reference_rad"]))
    calibrated = GravityModel(calibration=calibration, binding=binding)
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
