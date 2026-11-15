"""Hand-guiding control law: gravity support, damping, and hold when the arm is still."""

from dataclasses import dataclass
import math

import numpy as np

from rebot.model.arrays import vector6


@dataclass
class Command:
    torque: np.ndarray
    gravity: np.ndarray
    state: str                          # STARTUP, HOLD, or DRAG


class HandGuidingController:
    """STARTUP holds the initial pose; HOLD releases to DRAG on a push; DRAG re-holds when still."""

    def __init__(self, model, settings, gravity_scale=1.0):
        self.model, self.settings = model, settings
        self.gravity_scale = gravity_scale
        self.started = self.previous_time = None
        self.previous_q = self.target = None
        self.velocity = np.zeros(6)
        self.quiet_since = self.hold_since = None
        self.state = "STARTUP"

    def gravity(self, q):
        cfg = self.settings
        gravity = self.model.gravity(q) * cfg.gravity_scale * self.gravity_scale
        if not np.all(np.isfinite(gravity)) or np.any(np.abs(gravity) > cfg.torque_limit_nm):
            raise RuntimeError("Required gravity torque exceeds configured limits; support arm and stop.")
        return gravity

    def update(self, q, now):
        q = vector6(q, "q")
        if not math.isfinite(now):
            raise ValueError("Invalid sample time.")
        cfg = self.settings
        self.model.validate(q)
        if self.started is None:
            self.started = self.previous_time = now
            self.previous_q = self.target = q.copy()
        else:
            dt = now - self.previous_time
            if not 0 < dt <= cfg.max_sample_gap_s:
                raise RuntimeError("Feedback gap exceeded the control deadline.")
            measured_velocity = (q - self.previous_q) / dt
            alpha = 1 - math.exp(-dt / cfg.velocity_filter_s)
            self.velocity += alpha * (measured_velocity - self.velocity)
            self.previous_q, self.previous_time = q.copy(), now
        gravity = self.gravity(q)
        elapsed = now - self.started
        speed = float(np.max(np.abs(self.velocity)))
        if elapsed < cfg.startup_duration_s:
            # Native hold from the preceding run already carries gravity.
            # Keep that support from the first command; ramping it from zero
            # would withdraw support when ownership returns to the host loop.
            stiffness = cfg.hold_kp
            self.state = "STARTUP"
        else:
            if self.state == "STARTUP":
                self.state = "HOLD"
                self.hold_since = now - cfg.hold_ramp_s
            if self.state == "HOLD" and (
                    speed >= cfg.release_speed_rad_s
                    or np.max(np.abs(q - self.target)) >= cfg.release_error_rad):
                self.state, self.quiet_since = "DRAG", None
            if self.state == "DRAG":
                self.target = q.copy()
                if speed < cfg.still_speed_rad_s:
                    if self.quiet_since is None:
                        self.quiet_since = now
                    elif now - self.quiet_since >= cfg.settle_duration_s:
                        self.state, self.hold_since = "HOLD", now
                else:
                    self.quiet_since = None
            blend = (min(1.0, (now - self.hold_since) / cfg.hold_ramp_s)
                     if self.state == "HOLD" else 0.0)
            stiffness = cfg.hold_kp * blend
        torque = gravity + stiffness * (self.target - q) - cfg.damping * self.velocity
        # Clip TOTAL torque, including gravity, position hold, and damping.
        torque = np.clip(torque, -cfg.torque_limit_nm, cfg.torque_limit_nm)
        return Command(torque, gravity, self.state)
