"""Shared control settings from `config/control.yaml`."""

from dataclasses import dataclass, fields
import math
from pathlib import Path

import numpy as np
import yaml

from rebot.model.arrays import vector6
from rebot.paths import CONTROL_SETTINGS

VECTOR_KEYS = ("torque_limit_nm", "gravity_scale", "hold_kp", "damping")


@dataclass(frozen=True, eq=False)
class ControlSettings:
    rate_hz: float
    max_sample_gap_s: float
    max_feedback_age_s: float
    max_tracking_error_deg: float
    torque_limit_nm: np.ndarray
    gravity_scale: np.ndarray
    hold_kp: np.ndarray
    damping: np.ndarray
    startup_duration_s: float
    hold_ramp_s: float
    velocity_filter_s: float
    still_speed_rad_s: float
    release_speed_rad_s: float
    release_error_rad: float
    settle_duration_s: float

    def __post_init__(self):
        for field in fields(self):
            value = getattr(self, field.name)
            if field.name in VECTOR_KEYS:
                value = vector6(value, field.name)
                if np.any(value <= 0):
                    raise ValueError(f"{field.name} values must be positive.")
                object.__setattr__(self, field.name, value)
            elif type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"{field.name} must be finite and positive.")
        if (not 20 <= self.rate_hz <= 100
                or self.max_sample_gap_s > 0.15
                or self.max_feedback_age_s > self.max_sample_gap_s
                or self.release_speed_rad_s <= self.still_speed_rad_s
                or np.any(self.torque_limit_nm > [6, 20, 15, 5, 3, 2])
                or np.any(self.gravity_scale > 1.2)
                or np.any(self.hold_kp > 50)
                or np.any(self.damping > 5)):
            raise ValueError("Settings exceed experimental limits or have inconsistent thresholds.")

    def plain(self):
        """JSON/YAML-friendly copy for run logs."""
        return {field.name: (getattr(self, field.name).tolist() if field.name in VECTOR_KEYS
                             else getattr(self, field.name)) for field in fields(self)}


def load_settings(path=CONTROL_SETTINGS):
    values = yaml.safe_load(Path(path).read_text())
    expected = {field.name for field in fields(ControlSettings)}
    if not isinstance(values, dict) or set(values) != expected:
        raise ValueError(f"Settings must contain exactly these keys: {sorted(expected)}")
    return ControlSettings(**values)
