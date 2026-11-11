"""Propose task ranges between repeated, stationary returns to a reference EEF pose.

The analysis grid is only for segmentation. Export always uses native samples.
"""

import math

import numpy as np


DEFAULTS = {"home_radius_cm": 10.0, "still_seconds": 0.8, "reset_lead_seconds": 1.0}
STEP_NS = 50_000_000
MAX_AGE_NS = 250_000_000
SPEED_WINDOW_NS = 250_000_000


def runs(mask):
    edges = np.diff(np.r_[False, mask, False].astype(np.int8))
    return list(zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)))


def rotation_distance(a, b):
    return np.arccos(np.clip((np.sum(a * b, axis=(-2, -1)) - 1) / 2, -1, 1))


def suggest_ranges(grid, poses, valid, duration_ns, *, reference_ns=None, **options):
    """Use both arms, translation, orientation, and sustained low speed.

    A fixed lead-in removes an initial estimate of the manual return motion;
    it is deliberately editable and is not a semantic task-boundary detector.
    """
    settings = {**DEFAULTS, **options}
    if set(settings) != set(DEFAULTS):
        raise ValueError("Unknown reset detection setting.")
    for key, lower, upper in (("home_radius_cm", .1, 50), ("still_seconds", .2, 30),
                              ("reset_lead_seconds", 0, 30)):
        value = settings[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not lower <= value <= upper:
            raise ValueError(f"Invalid reset setting: {key}.")
    if reference_ns is not None and (type(reference_ns) is not int or not 0 <= reference_ns < duration_ns):
        raise ValueError("Reset reference must lie within the recording.")
    response = {"usable": False, "episodes": [], "resets": [], "reference_ns": None, "settings": settings}
    if len(grid) < 3:
        return {**response, "message": "Recording is too short to detect stationary resets."}
    valid = valid.copy()
    still = valid.copy()
    lag = max(1, round(SPEED_WINDOW_NS / STEP_NS))
    before = np.maximum(0, np.arange(len(grid)) - lag)
    dt = np.maximum((grid - grid[before]) / 1e9, 1e-9)
    still[:lag] = False
    # Missing observations cannot establish a stationary interval.
    for offset in range(lag + 1):
        still[offset:] &= valid[:len(valid) - offset] if offset else valid
    recent = still.copy()
    moving = np.zeros(len(grid), dtype=bool)
    rotations = {}
    for side, pose in poses.items():
        if pose.shape != (len(grid), 12):
            raise ValueError("Reset detection requires 12-component EEF poses for both arms.")
        valid &= np.all(np.isfinite(pose), axis=1)
        rotation = pose[:, 3:].reshape(-1, 3, 3).transpose(0, 2, 1)
        rotations[side] = rotation
        speed = np.linalg.norm(pose[:, :3] - pose[before, :3], axis=1) / dt
        angular_speed = rotation_distance(rotation, rotation[before]) / dt
        moving |= (speed > .015) | (angular_speed > math.radians(8))
        still &= (speed <= .015) & (angular_speed <= math.radians(8))
    still &= valid
    moving &= recent & valid
    boundaries = np.r_[grid, duration_ns]
    minimum = round(settings["still_seconds"] * 1e9)
    if reference_ns is None:
        initial = next(((a, b) for a, b in runs(still)
                        if int(boundaries[b]) - int(grid[a]) >= minimum and grid[a] <= 30_000_000_000), None)
        if initial is None:
            return {**response, "message": "No initial still pose found. Pause on a reset and use that pose."}
        a, b = initial
        reference_index = (a + min(b, a + max(1, round(minimum / STEP_NS)))) // 2
    else:
        reference_index = int(np.argmin(np.abs(grid - reference_ns)))
        if not valid[reference_index]:
            raise ValueError("No recent feedback at this reset reference. Choose another frame.")
    response["reference_ns"] = int(grid[reference_index])
    near = valid.copy()
    for side, pose in poses.items():
        near &= (np.linalg.norm(pose[:, :3] - pose[reference_index, :3], axis=1) <= settings["home_radius_cm"] / 100)
        near &= rotation_distance(rotations[side], rotations[side][reference_index]) <= math.radians(20)
    stationary_home = near & still
    # Close short speed spikes only when observations stay valid and near home.
    for a, b in runs(~stationary_home):
        if a > 0 and b < len(grid) and boundaries[b] - grid[a] <= 150_000_000 and np.all(near[a:b]):
            stationary_home[a:b] = True
    lead_ns = round(settings["reset_lead_seconds"] * 1e9)
    resets = []
    for a, b in runs(stationary_home):
        if int(boundaries[b]) - int(grid[a]) < minimum:
            continue
        start = max(0, int(grid[a]) - SPEED_WINDOW_NS - lead_ns)
        # Keep a small departure margin so slow initial task motion is retained.
        end = duration_ns if b == len(grid) else max(start + 1, int(boundaries[b]) - 150_000_000)
        record = {"start_ns": start, "end_ns": end,
                  "stationary_start_ns": int(grid[a]), "stationary_end_ns": int(boundaries[b])}
        if resets and start <= resets[-1]["end_ns"]:
            resets[-1]["end_ns"] = max(resets[-1]["end_ns"], end)
            resets[-1]["stationary_end_ns"] = record["stationary_end_ns"]
        else:
            resets.append(record)
    if not resets:
        return {**response, "message": "No sustained stationary returns found. Adjust the reference pose or cut manually."}
    episodes, end = [], 0
    for reset in resets:
        if (reset["start_ns"] - end >= 750_000_000
                and np.count_nonzero(moving & (grid >= end) & (grid < reset["start_ns"])) >= 3):
            episodes.append({"start_ns": end, "end_ns": reset["start_ns"]})
        end = reset["end_ns"]
    if duration_ns - end >= 750_000_000 and np.count_nonzero(moving & (grid >= end)) >= 3:
        episodes.append({"start_ns": end, "end_ns": duration_ns})
    return {**response, "usable": True, "episodes": episodes, "resets": resets,
            "message": f"Suggested {len(episodes)} episodes from {len(resets)} stationary home intervals. Review the excluded return motion."}
