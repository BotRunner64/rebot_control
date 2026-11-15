"""Map Star Arm leader angles onto reBot RS follower joint targets."""

import math

import numpy as np

# Leader-to-follower sign per arm joint, applied in motor coordinates.
DIRECTIONS = (1, 1, -1, -1, -1, 1)
GRIPPER_RATIO = 6.                       # follower motor travel per leader handle angle
HANDLE_STROKE_DEG = 45.                  # leader handle angle from closed to open


def leader_angles(value, name):
    result = np.asarray(value, dtype=float)
    if result.shape != (7,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain seven finite angles.")
    return result


def map_target(raw_deg, reference_deg, lower, upper, gripper_close_tolerance_deg=1.):
    """Follower joint targets (rad, relative to its reference), clipped to [lower, upper].

    `lower`/`upper` have six entries (arm only) or seven (with gripper).
    """
    # Preserve SDK multi-turn angles; wrapping at +/-180 degrees would jump.
    relative = np.radians(leader_angles(raw_deg, "Leader feedback")
                          - leader_angles(reference_deg, "Leader reference"))
    target = relative * np.array([*DIRECTIONS, GRIPPER_RATIO])
    if len(lower) == 7:
        if (type(gripper_close_tolerance_deg) not in (int, float)
                or not math.isfinite(gripper_close_tolerance_deg)
                or not 0 <= gripper_close_tolerance_deg < HANDLE_STROKE_DEG):
            raise ValueError("Gripper leader closing tolerance must be finite and in [0, 45) degrees.")
        # Collapse only the closing margin; preserve the 45-degree open endpoint.
        margin = math.radians(gripper_close_tolerance_deg)
        openness = np.clip((relative[6] - margin) / (math.radians(HANDLE_STROKE_DEG) - margin), 0., 1.)
        target[6] = lower[6] + openness * (upper[6] - lower[6])
    return np.clip(target[:len(lower)], lower, upper)
