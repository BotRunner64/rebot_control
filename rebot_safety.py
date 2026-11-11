"""Joint tracking-error checks shared by native position controllers."""

import math

import numpy as np


DEFAULT_MAX_TRACKING_ERROR_DEG = 45.0


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
