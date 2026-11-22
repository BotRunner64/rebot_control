"""Paired-static gravity identification: sweep plans, dataset reduction, and mass-scale fitting.

Everything here is offline math on plans and recorded samples; the motion that
produces the samples lives in `rebot.apps.identify`.
"""

import json
import math
from itertools import pairwise
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
import yaml

from rebot.bindings import ArmBinding
from rebot.model.gravity import MODEL_PATH, GravityModel, check_binding, digest, validate_scales
from rebot.model.arrays import vector6 as vector
from rebot.model.trajectory import plain


FIT_AXES = [1, 2, 3, 5]  # J1 and the weakly observed J5 body keep their CAD masses.
SETTLE_S = .7
SAMPLE_S = 1.
APPROACH_RAD = .08
CLEARANCE = np.array([0., .7, 1.1, 0., 0., 0.])
# Each sweep specifies the joint, the first target (J2..J5), and its next two angles.
SWEEPS = (
    (2, (.45, 1.7, -.35, .45), (.85, 1.25)),
    (2, (.65, 1.2, -.35, -.45), (1.05, 1.45)),
    (3, (1.15, .65, .35, -.45), (1.15, 1.65)),
    (3, (.55, .9, -.35, -.45), (1.4, 1.9)),
    (4, (.65, 1.7, -.6, .45), (-.12, .36)),
    (4, (.95, 1.55, -.36, .45), (.12, .6)),
    (5, (.95, 1.55, .35, -.8), (-.16, .48)),
    (5, (.65, 1.7, -.35, -.48), (.16, .8)),
)
VALIDATION_SWEEPS = (
    (2, (.55, 1.45, -.18, .25), (.95, 1.35)),
    (3, (.75, .8, -.22, -.3), (1.3, 1.8)),
    (4, (.6, 1.35, -.48, .3), (0., .48)),
    (5, (.6, 1.35, .22, -.64), (0., .64)),
)


def clearance_pose(yaw):
    q = CLEARANCE.copy()
    q[0] = yaw
    return q


def entry_path(start):
    q = vector(start, "start")
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


def path_check(model, start, goal, gravity_scale, torque_limit):
    """Reject a straight joint-space segment that leaves the limits or the torque budget.

    A measured start may sit slightly outside the limits (within the model's state
    tolerance, e.g. resting on a hard stop); the segment may then only move inward,
    never further out than the start.
    """
    start, goal = vector(start, "start"), vector(goal, "goal")
    model.validate(start)
    allowed = model.limit_excess(start) + 1e-9
    steps = max(2, int(np.max(np.abs(goal - start)) / .02) + 2)
    for q in np.linspace(start, goal, steps):
        if np.any(model.limit_excess(q) > allowed):
            raise ValueError("Planned path exceeds joint limits.")
        if np.any(np.abs(model.gravity(q) * gravity_scale) > .9 * torque_limit):
            raise ValueError("Planned path exceeds the 90% model-gravity torque budget.")


def entry_candidates(start, yaw):
    """Try turning at different stages of unfolding, then try a short detour."""
    unfolding = [start, *entry_path(start)]
    if yaw == start[0]:
        yield unfolding[1:]
    else:
        turned = [q.copy() for q in unfolding]
        for q in turned:
            q[0] = yaw
        for pivot in range(len(unfolding)):
            yield unfolding[1:pivot + 1] + turned[pivot:]

    # A tool on the far side of the mast may need space to unfold before returning.
    for offset in np.radians((15., -15.)):
        detour = start.copy()
        detour[0] += offset
        yield [detour, *entry_path(detour), clearance_pose(yaw)]


def planned_entry(start, yaw, check_segment, model):
    """Waypoints from a measured start to the clearance pose, all inside the model limits.

    A start resting slightly outside a limit first steps back inside. Waypoints that kept
    the measured out-of-limit angle would be rejected whenever a later reading of the same
    pose is marginally closer to the limit.
    """
    inside = np.clip(start, model.lower, model.upper)
    prefix = []
    if not np.array_equal(inside, start):
        check_segment(start, inside)
        prefix, start = [inside], inside
    for route in entry_candidates(start, yaw):
        try:
            for a, b in pairwise([start, *route]):
                check_segment(a, b)
            return prefix + route
        except ValueError as exc:
            reason = exc
    raise ValueError(f"No feasible entry to sweep posture: {reason}") from reason


def make_plan(start, settings, independent=False):
    """Join the measured start to a fixed, reproducible identification trajectory."""
    start = vector(start, "start")
    model = GravityModel()
    model.validate(start)
    poses = []

    def check_entry_segment(a, b):
        path_check(model, a, b, settings.gravity_scale, settings.torque_limit_nm)

    def append(q, stage, **fields):
        poses.append({"id": len(poses), "q": q.tolist(), "stage": stage, **fields})

    for q in planned_entry(start, CLEARANCE[0], check_entry_segment, model):
        append(q, "entry")
    for sweep, (joint, first, following) in enumerate(VALIDATION_SWEEPS if independent else SWEEPS):
        axis = joint - 1
        q = CLEARANCE.copy()
        q[1:5] = first
        angles = (q[axis], *following)
        for direction in (1, -1):
            q[axis] = (angles[0] if direction == 1 else angles[-1]) - direction * APPROACH_RAD
            append(q, "transit")
            indices = range(len(angles)) if direction == 1 else reversed(range(len(angles)))
            for i in indices:
                q[axis] = angles[i]
                split = "validation" if independent or i == 1 else "train"
                append(q, "sample", pair_id=3*sweep+i, axis=joint, direction=direction, split=split)
        append(CLEARANCE, "transit")
    plan = {"version": 2, "method": "paired_static", "model_sha256": model.sha256,
            "units": "rad", "start": start.tolist(), "clearance": CLEARANCE.tolist(), "poses": poses}
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
    if "applied_mass_scales" in meta:
        model.apply_scales(meta["applied_mass_scales"])
    poses = {pose["id"]: pose
             for pose in validate_plan(meta["plan"], model, gravity_scale, torque_limit)}
    completed = {(r["pose_id"], r["window"]) for r in rows if r.get("type") == "pose_complete"}
    if rows[-1].get("type") != "end" or rows[-1].get("status") != "completed":
        raise ValueError("Identification requires a completed collection.")
    samples, invalid, arrival_drop = {}, set(), {}
    heights = {i: model.tool_height(np.array(p["q"])) for i, p in poses.items() if p["stage"] == "sample"}
    for row in rows[1:]:
        if row.get("type") != "sample":
            continue
        pose_id = row["pose_id"]
        if pose_id in heights and row.get("phase") in ("SETTLE", "SAMPLE"):
            drop = heights[pose_id] - model.tool_height(vector(row["q"], "q"))
            arrival_drop[pose_id] = max(0., drop, arrival_drop.get(pose_id, 0.))
        if row.get("eligible") is not True:
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
        reported = vector(row["reported"], "reported")
        native = row.get("native_parameters")
        pd = None if native is None else tau - vector([p["gravity"] for p in native], "native feedforward")
        samples.setdefault(key, []).append((q, tau, row["time_s"], reported, pd))
    windows = {}
    for (pose_id, window), records in samples.items():
        if (pose_id, window) in invalid:
            continue
        q = np.array([r[0] for r in records])
        times = np.array([r[2] for r in records])
        if (len(records) < 5 or not np.all(np.isfinite(times)) or np.any(np.diff(times) <= 0)
                or np.max(np.diff(times)) > cfg["max_sample_gap_s"]
                or np.ptp(times) < .8 * meta["sample_s"] or np.max(np.std(q, axis=0)) > .01):
            continue
        if pose_id in windows:
            raise ValueError("Multiple completed sample windows for the same waypoint.")
        windows[pose_id] = {"q": np.median(q, axis=0),
                            "torque": np.median([r[1] for r in records], axis=0),
                            "reported": np.median([r[3] for r in records], axis=0),
                            "pd": (np.median([r[4] for r in records], axis=0)
                                   if all(r[4] is not None for r in records) else None)}
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
                           "target": np.array(passes[1][0]["q"]), "pass_q": [plus["q"], minus["q"]],
                           "reported": (plus["reported"] + minus["reported"]) / 2,
                           "pass_pd": [plus["pd"], minus["pd"]],
                           "arrival_drop_m": max(arrival_drop[passes[d][0]["id"]] for d in (1, -1)),
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
            "median_approach_hysteresis_nm": per_joint(hysteresis, np.median),
            "median_abs_approach_hysteresis_nm": per_joint(np.abs(hysteresis), np.median),
            "estimated_reported_rmse_nm": per_joint(
                np.square([r["torque"][r["axis"]] - r["reported"][r["axis"]] for r in rows]),
                lambda x: np.sqrt(np.mean(x)))}


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
    if any(np.linalg.norm(a["target"] - b["target"]) < .02 for a in train for b in valid):
        raise ValueError("Training and validation poses overlap; collect distinct held-out poses.")
    basis = np.array([r["basis"][r["axis"]] for r in train])
    matrix = basis[:, FIT_AXES]
    target = np.array([r["torque"][r["axis"]] for r in train]) - basis[:, [0, 4]].sum(axis=1)
    singular = np.linalg.svd(matrix, compute_uv=False)
    rank = int(np.linalg.matrix_rank(matrix))
    condition = float(singular[0] / singular[-1]) if singular[-1] > 0 else None
    report = {"version": 1, "effort_source": "native_pd_estimate", "rank": rank, "condition": condition,
              "fitted_joints": [i+1 for i in FIT_AXES], "fixed_joints": [1, 5],
              "singular_values": singular.tolist(), "scope": "CAD body mass scales with fixed centers of mass; only the swept joint torque is fitted, with opposite passes averaged.",
              "accepted": False}
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=False)
    if rank != len(FIT_AXES) or condition is None or condition > 1e4:
        report["reason"] = "Insufficient independent posture excitation for four mass scales."
    else:
        def residual(scales):
            return np.concatenate((matrix @ scales - target,
                                   math.sqrt(regularization) * (scales - 1)))

        solution = least_squares(residual, np.ones(len(FIT_AXES)), bounds=(.5, 2.), loss="soft_l1", f_scale=.1)
        scales = np.ones(6)
        scales[FIT_AXES] = solution.x
        report.update({"mass_scales": scales.tolist(), "solver_success": bool(solution.success),
                       "train": metrics(train, scales), "validation": metrics(valid, scales),
                       "max_rmse_nm": max_rmse})
        warnings = validation_failures(report["validation"], max_rmse)
        reasons = []
        if not solution.success:
            reasons.insert(0, f"Solver failed: {solution.message}")
        bound_groups = [f"J{i}={value:.3f}" for i, value in zip(report["fitted_joints"], solution.x)
                        if value <= .5001 or value >= 1.9999]
        if bound_groups:
            reasons.insert(0, "CAD body mass scales reached bounds: " + ", ".join(bound_groups) + ".")
        # Only an unusable solution blocks the profile; accuracy limits are advisory.
        report["reasons"] = reasons
        report["warnings"] = warnings
        report["accepted"] = not reasons
        if not report["accepted"]:
            report["reason"] = " ".join(reasons)
        else:
            profile = {"version": 2, "kind": "static_mass_scale", "accepted": True, "verified": False,
                       "model_sha256": meta["model_sha256"], "side": meta["side"],
                       "motor_uids": meta["motor_uids"], "reference_rad": meta["reference_rad"],
                       "mass_scales": scales.tolist(), "effort_source": "native_pd_estimate",
                       "method": "paired_static",
                       "identification_targets_rad": [r["target"].tolist() for r in rows],
                       "dataset_sha256": digest(data_path), "validation": report["validation"],
                       "warnings": warnings}
            write_yaml(output_dir / "calibration.yaml", profile)
    with (output_dir / "report.json").open("x") as stream:
        json.dump(plain(report), stream, indent=2, allow_nan=False)
    return report


def validate(data_path, calibration, max_rmse=.25):
    positive(max_rmse, "max_rmse")
    profile = yaml.safe_load(Path(calibration).read_text())
    meta, rows = load_dataset(data_path)
    if profile.get("dataset_sha256") == digest(data_path):
        raise ValueError("Independent validation requires a different collection session.")
    if len(rows) < 8:
        raise ValueError("Independent validation requires at least 8 complete approach pairs.")
    prior_targets = [vector(q, "identification target") for q in profile["identification_targets_rad"]]
    if any(np.linalg.norm(row["target"] - q) < .02 for row in rows for q in prior_targets):
        raise ValueError("Independent validation requires targets distinct from the identification session.")
    binding = ArmBinding(meta["side"], "", (), meta["motor_uids"], np.asarray(meta["reference_rad"]))
    scales = candidate_scales(profile, binding)
    result = metrics(rows, scales)
    result["reasons"] = validation_failures(result, max_rmse)
    matrix = np.array([row["basis"][row["axis"], FIT_AXES] for row in rows])
    result["rank"] = int(np.linalg.matrix_rank(matrix))
    condition = float(np.linalg.cond(matrix))
    result["condition"] = condition if math.isfinite(condition) else None
    if result["rank"] != len(FIT_AXES) or not math.isfinite(condition) or condition > 1e4:
        result["reasons"].append("Independent posture excitation is insufficient for four mass scales.")
    result["accepted"] = not result["reasons"]
    return result


def candidate_scales(profile, binding):
    kind = (profile.get("kind"), profile.get("accepted"))
    if (kind not in (("static_mass_scale", True), ("static_mass_scale_candidate", False))
            or profile.get("version") != 2 or profile.get("model_sha256") != digest(MODEL_PATH)
            or profile.get("method") != "paired_static" or profile.get("effort_source") != "native_pd_estimate"):
        raise ValueError("Expected a provisional paired-static fit matching the arm model.")
    check_binding(profile, binding)
    scales = validate_scales(profile["mass_scales"])
    if scales[4] != 1:
        raise ValueError("Candidate must preserve the CAD J5 mass.")
    return scales
