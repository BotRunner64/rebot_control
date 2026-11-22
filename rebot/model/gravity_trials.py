"""Matched hold trials and known-load torque comparisons for gravity calibration."""

import numpy as np

from rebot.bindings import ArmBinding
from rebot.model.gravity import GravityModel, check_binding
from rebot.model.gravity_fit import candidate_scales, load_dataset, validation_failures


def matched_sessions(first_path, second_path):
    """Return complete matched target pairs for controlled A/B experiments."""
    first, a = load_dataset(first_path)
    second, b = load_dataset(second_path)
    binding = ArmBinding(first["side"], "", (), first["motor_uids"], np.array(first["reference_rad"]))
    check_binding(second, binding)
    for key in ("speed", "settle_s", "sample_s"):
        if first.get(key) != second.get(key):
            raise ValueError(f"Comparison sessions differ in {key}.")
    settings_a, settings_b = (dict(m["settings"]) for m in (first, second))
    for cfg in (settings_a, settings_b):
        del cfg["gravity_scale"]
    if settings_a != settings_b:
        raise ValueError("Comparison sessions require identical gains and control settings.")

    def indexed(rows):
        return {(r["axis"], tuple(r["target"])): r for r in rows}

    a, b = indexed(a), indexed(b)
    planned = {(p["axis"]-1, tuple(p["q"])) for p in first["plan"]["poses"] if p["stage"] == "sample"}
    other = {(p["axis"]-1, tuple(p["q"])) for p in second["plan"]["poses"] if p["stage"] == "sample"}
    if a.keys() != planned or b.keys() != planned or other != planned:
        raise ValueError("Comparison requires complete paired passes at every matching target.")
    return first, second, [(a[k], b[k]) for k in a]


def compare_holds(baseline, trial, profile, max_rmse=.25):
    first, second, pairs = matched_sessions(baseline, trial)
    binding = ArmBinding(first["side"], "", (), first["motor_uids"], np.array(first["reference_rad"]))
    scales = candidate_scales(profile, binding)
    if (first.get("experiment") != "baseline" or second.get("experiment") != "trial"
            or not np.array_equal(first["applied_mass_scales"], np.ones(6))
            or not np.array_equal(second["applied_mass_scales"], scales)
            or not np.array_equal(second["settings"]["gravity_scale"], np.ones(6))):
        raise ValueError("Expected independent baseline and matching candidate hold trials.")
    result = {"pairs": len(pairs)}
    for n, label in enumerate(("before", "after")):
        by_axis = [[] for _ in range(6)]
        all_pd = []
        drops = []
        for pair in pairs:
            r = pair[n]
            for pd in r["pass_pd"]:
                if pd is None:
                    raise ValueError("Hold comparison requires decoded native feedforward.")
                by_axis[r["axis"]].append(pd[r["axis"]])
                all_pd.append(pd)
            drops.append(r["arrival_drop_m"])
        result[f"rmse_{label}_nm"] = [float(np.sqrt(np.mean(np.square(v)))) if v else None for v in by_axis]
        result[f"pooled_{label}_nm"] = float(np.sqrt(np.mean(np.square([v for axis in by_axis for v in axis]))))
        result[f"all_joint_hold_rms_{label}_nm"] = np.sqrt(np.mean(np.square(all_pd), axis=0)).tolist()
        result[f"max_arrival_drop_{label}_mm"] = 1000 * max(drops)
    result["reasons"] = validation_failures(result, max_rmse)
    result["accepted"] = not result["reasons"]
    return result


def compare_payload(empty, loaded, mass, com):
    first, second, pairs = matched_sessions(empty, loaded)
    if (first.get("experiment") != "payload_empty" or second.get("experiment") != "payload_loaded"
            or second.get("payload") != {"mass_kg": mass, "com_link6_m": list(com)}
            or first["settings"]["gravity_scale"] != second["settings"]["gravity_scale"]):
        raise ValueError("Payload comparison requires matched empty and known-load experiments.")
    model, loaded_model = GravityModel(), GravityModel()
    loaded_model.add_point_payload(mass, com)
    expected, estimated, reported = [], [], []
    for a, b in pairs:
        # Subtract the nominal arm's change due to small tracking differences.
        base_shift = (b["basis"] - a["basis"]).sum(axis=1)
        expected.append(np.mean([loaded_model.gravity(q) - model.gravity(q) for q in b["pass_q"]], axis=0))
        estimated.append(b["torque"] - a["torque"] - base_shift)
        reported.append(b["reported"] - a["reported"] - base_shift)
    expected = np.array(expected)
    result = {"pairs": len(pairs), "mass_kg": mass, "com_link6_m": list(com),
              "expected_rms_nm": np.sqrt(np.mean(expected**2, axis=0)).tolist(),
              "targets": [{"target_rad": a["target"].tolist(), "delta_expected_nm": e.tolist(),
                           "delta_estimated_nm": x.tolist(), "delta_reported_nm": y.tolist()}
                          for (a, _), e, x, y in zip(pairs, expected, estimated, reported)]}
    energy = np.sum(expected**2, axis=0)
    for label, observed in (("estimated", estimated), ("reported", reported)):
        # A gain from negligible predicted torque is uninformative.
        observed = np.asarray(observed)
        result[label] = {"rmse_nm": np.sqrt(np.mean((observed - expected)**2, axis=0)).tolist(),
                         "gain": [float(expected[:, j] @ observed[:, j] / energy[j])
                                  if energy[j] / len(pairs) >= .05**2 else None for j in range(6)]}
    return result
