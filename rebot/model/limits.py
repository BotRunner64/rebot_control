"""Joint travel and tracking-error limits for position control."""

from dataclasses import dataclass
import math

import numpy as np

# Software travel limits in motor coordinates relative to the reference, ordered J1-J6
# and gripper. J1-J5 match LeRobot's RS profile (motor_family.py); J6 +/-130 deg follows
# Seeed's current integration (LeRobot uses +/-90). The URDF limits are not used.
JOINT_LIMITS_DEG = ((-145., 145.), (0., 170.), (0., 200.), (-80., 90.),
                    (-90., 90.), (-130., 130.), (0., 270.))
GRIPPER_OPEN_DEG = 270.
STATE_LIMIT_TOLERANCE_RAD = 0.1
DEFAULT_MAX_TRACKING_ERROR_DEG = 45.0


@dataclass(frozen=True, eq=False)
class JointRange:
    """Software reference and allowed travel (relative to it) for six or seven motors."""
    reference: np.ndarray
    lower: np.ndarray
    upper: np.ndarray

    def validate(self, raw):
        """Return finite raw feedback whose arm joints are inside the travel limits."""
        raw = np.asarray(raw, dtype=float)
        if raw.shape != self.reference.shape or not np.all(np.isfinite(raw)):
            raise ValueError("Expected finite feedback for every selected follower motor.")
        # Check arm travel only; gripper feedback still must be finite above.
        q = (raw - self.reference)[:6]
        low, high = self.lower[:6], self.upper[:6]
        invalid = np.flatnonzero((q < low - STATE_LIMIT_TOLERANCE_RAD)
                                 | (q > high + STATE_LIMIT_TOLERANCE_RAD))
        if len(invalid):
            details = "; ".join(
                f"J{i + 1}: relative={math.degrees(q[i]):.2f} deg, "
                f"limits=[{math.degrees(low[i]):.2f}, {math.degrees(high[i]):.2f}] deg, "
                f"raw={math.degrees(raw[i]):.2f} deg, "
                f"reference={math.degrees(self.reference[i]):.2f} deg" for i in invalid)
            raise ValueError(f"Follower pose is outside the joint limits: {details}; "
                             f"tolerance={math.degrees(STATE_LIMIT_TOLERANCE_RAD):.2f} deg. "
                             "Check the physical pose and software reference.")
        return raw


def joint_range(reference, native_position, gripper_open_deg=GRIPPER_OPEN_DEG):
    """Travel limits for `len(reference)` motors, kept inside the encodable MIT range."""
    reference = np.asarray(reference, dtype=float)
    count = len(reference)
    if count not in (6, 7):
        raise ValueError("Expected six arm joints, or seven including the gripper.")
    if not math.isfinite(gripper_open_deg) or not 0 < gripper_open_deg <= GRIPPER_OPEN_DEG:
        raise ValueError("Gripper opening must be positive and at most the official 270-degree limit.")
    limits = np.radians(JOINT_LIMITS_DEG[:count])
    if count == 7:
        limits[6, 1] = math.radians(gripper_open_deg)
    native = np.asarray(native_position, dtype=float)[:count]
    low = np.maximum(limits[:, 0], -native - reference)
    high = np.minimum(limits[:, 1], native - reference)
    if np.any(low > high):
        raise ValueError("Software references leave no encodable joint range.")
    return JointRange(reference, low, high)


def validate_tracking_error(target, measured, max_error_deg=DEFAULT_MAX_TRACKING_ERROR_DEG):
    """Reject an excessive error on any arm joint before sending a target.

    Compare multi-turn angles directly; wrapping would hide a large requested move.
    The optional seventh gripper joint is excluded because contact prevents tracking.
    """
    if (type(max_error_deg) not in (int, float) or not math.isfinite(max_error_deg)
            or max_error_deg <= 0):
        raise ValueError("Maximum tracking error must be finite and positive in degrees.")
    target, measured = np.asarray(target, dtype=float), np.asarray(measured, dtype=float)
    if (target.shape not in ((6,), (7,)) or measured.shape != target.shape
            or not np.all(np.isfinite(target[:6])) or not np.all(np.isfinite(measured[:6]))):
        raise ValueError("Tracking checks require six finite arm targets and measured angles.")
    errors = np.abs(target[:6] - measured[:6])
    invalid = np.flatnonzero(errors > math.radians(max_error_deg))
    if len(invalid):
        details = "; ".join(f"J{i + 1}: error={math.degrees(errors[i]):.2f} deg"
                            for i in invalid)
        raise RuntimeError(f"Tracking error limit exceeded ({max_error_deg:g} deg): {details}. "
                           "Stopping control and attempting current-position hold.")
