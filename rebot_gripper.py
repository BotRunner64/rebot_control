"""Continuous RS gripper position tracking with a total motor-torque limit."""

from dataclasses import dataclass, fields, replace
import math
from pathlib import Path

import yaml


SETTINGS_PATH = Path(__file__).with_suffix(".yaml")


@dataclass(frozen=True)
class GripperSettings:
    torque_limit_nm: float = 1.0
    kp: float = 12.0
    kd: float = 0.5
    leader_close_tolerance_deg: float = 1.0
    close_preload_deg: float = 1.0
    close_preload_window_deg: float = 5.0

    def __post_init__(self):
        for field in fields(self):
            value = getattr(self, field.name)
            if (type(value) not in (int, float) or not math.isfinite(value)
                    or value < 0 or (field.name not in ("leader_close_tolerance_deg", "close_preload_deg")
                                     and value == 0)):
                raise ValueError(f"Gripper {field.name} must be finite and positive "
                                 "(leader tolerance and closing preload may be zero).")
        if self.leader_close_tolerance_deg >= 45.:
            raise ValueError("Gripper leader closing tolerance must be less than the 45-degree handle stroke.")

    def validate_motor(self, limits):
        if (not limits.torque / 65535 <= self.torque_limit_nm <= limits.torque
                or self.kp > limits.kp or self.kd > limits.kd):
            raise ValueError("Gripper torque limit or hold gains exceed the selected motor's MIT range.")


def load_settings(path=SETTINGS_PATH, torque_limit_nm=None):
    values = yaml.safe_load(Path(path).read_text())
    expected = {field.name for field in fields(GripperSettings)}
    optional = {"leader_close_tolerance_deg", "close_preload_deg", "close_preload_window_deg"}
    if (not isinstance(values, dict) or not expected - optional <= set(values)
            or not set(values) <= expected):
        raise ValueError(f"Gripper settings accept exactly these keys: {sorted(expected)}; "
                         "closing tolerance and preload settings are optional. "
                         "Integral and fixed friction settings are not supported.")
    settings = GripperSettings(**values)
    return settings if torque_limit_nm is None else replace(settings, torque_limit_nm=torque_limit_nm)


class GripperController:
    """Compute bounded PD effort with smooth closing-end position preload.

    Increasing motor position opens the RS gripper. Send the returned torque
    with motor-side MIT kp=kd=0, so no additional PD effort bypasses this limit.
    """

    def __init__(self, closed_position, open_position, settings):
        if (not all(math.isfinite(value) for value in (closed_position, open_position))
                or closed_position >= open_position):
            raise ValueError("Gripper travel must have finite, increasing closed/open positions.")
        self.closed, self.open = closed_position, open_position
        self.settings = settings

    def update(self, target, position, velocity):
        if not all(math.isfinite(value) for value in (target, position, velocity)):
            raise ValueError("Gripper target, position, and velocity must be finite.")
        target = min(self.open, max(self.closed, target))
        cfg = self.settings
        window = min(math.radians(cfg.close_preload_window_deg), self.open - self.closed)
        blend = max(0., 1. - (target - self.closed) / window)
        weight = blend * blend * (3. - 2. * blend)
        # Keep the calibrated closed pose and published target intact. Only the
        # host PD's virtual target extends past closure to maintain preload.
        effective_target = target - math.radians(cfg.close_preload_deg) * weight
        error = effective_target - position
        pd_torque = cfg.kp * error - cfg.kd * velocity
        return min(cfg.torque_limit_nm, max(-cfg.torque_limit_nm, pd_torque))
