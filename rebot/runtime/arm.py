"""One follower arm session: the startup, command, and exit sequence every controller shares.

    open -> verify identity -> validate pose -> install native hold
         -> prepare (MIT mode, watchdog off) -> re-read -> enable

Leaving the context freezes enabled motors in native position hold with gravity
support. Nothing here ever disables a motor.

Joint values at this interface are relative to the software reference (rad);
raw motor angles stay inside the class.
"""

from contextlib import ExitStack, contextmanager
from dataclasses import dataclass
import math
import time

import numpy as np

from rebot.devices.robstride import LIMITS, MotorBus
from rebot.model.gravity import GravitySupport
from rebot.model.gripper import GripperController
from rebot.model.limits import GRIPPER_OPEN_DEG, joint_range, validate_tracking_error
from rebot.model.trajectory import MOTION_KD, MOTION_KP


@dataclass(frozen=True)
class FeedbackPolicy:
    """How a servo cycle treats late or missing motor replies."""
    max_age_s: float                     # oldest cached MIT feedback a cycle may use
    cycle_budget_s: float | None         # abort a cycle that takes longer; None: unbounded
    attempts: int = 1                    # resend the same command to motors that did not reply
    reply_timeout_s: float | None = None # per attempt; None: the bus default

    @classmethod
    def strict(cls, settings):
        """Fail on the first missing reply (EE and policy control)."""
        return cls(settings.max_feedback_age_s, settings.max_feedback_age_s)

    @classmethod
    def tolerant(cls):
        """Retry briefly and keep the last feedback (teleoperation on a busy bus)."""
        return cls(math.inf, None, attempts=15, reply_timeout_s=.01)


def follower_range(binding, count, gripper_open_deg=GRIPPER_OPEN_DEG):
    """Travel limits of a bound arm's first `count` motors."""
    native = [LIMITS[model].position for model in binding.models]
    return joint_range(binding.reference[:count], native, gripper_open_deg)


class Arm:
    """Six arm motors, or seven with the gripper, of one bound follower arm.

    gravity:       GravityModel for feedforward and exit hold; None commands zero feedforward.
    gripper:       GripperSettings to control motor 7 with host-side torque-limited PD.
    joint_limits:  True checks feedback against the position-control travel limits
                   (servo controllers); False checks the gravity model's own joint
                   range instead (torque and trajectory controllers).
    """

    def __init__(self, binding, settings, *, gravity=None, gravity_scale=1., gripper=None,
                 gripper_open_deg=GRIPPER_OPEN_DEG, kp=MOTION_KP, kd=MOTION_KD,
                 joint_limits=True, feedback=None, stop=None, bus_factory=MotorBus):
        if binding.reference is None:
            raise ValueError(f"Calibrate the {binding.side} reBot arm first.")
        self.binding, self.settings, self.stop = binding, settings, stop
        self.side = binding.side
        self.count = 7 if gripper is not None else 6
        self.joint_limits = joint_limits
        self.feedback = FeedbackPolicy.strict(settings) if feedback is None else feedback
        self.bus_factory = bus_factory
        self.range = follower_range(binding, self.count, gripper_open_deg)
        self.reference = self.range.reference
        self.model = gravity
        self.gravity = None if gravity is None else GravitySupport(
            gravity, self.reference[:6], settings.gravity_scale * gravity_scale,
            settings.torque_limit_nm)
        self.kp, self.kd = np.array(kp, dtype=float), np.array(kd, dtype=float)
        self.gripper = None
        self.torque_limits = None
        if gripper is not None:
            gripper.validate_motor(LIMITS[binding.models[6]])
            self.kp, self.kd = np.append(self.kp, gripper.kp), np.append(self.kd, gripper.kd)
            self.gripper = GripperController(self.reference[6] + self.range.lower[6],
                                             self.reference[6] + self.range.upper[6], gripper)
            self.torque_limits = {7: gripper.torque_limit_nm}
        # While tracking, the host computes the gripper's total torque; native PD
        # must not add motor torque after that limit. Hold still uses native gains.
        self.servo_kp, self.servo_kd = self.kp.copy(), self.kd.copy()
        if gripper is not None:
            self.servo_kp[6] = self.servo_kd[6] = 0.
        self._zero = np.zeros(self.count)
        self.bus = self.raw = None
        self.sampled_at = self.acquired_at = None

    # --- lifecycle -----------------------------------------------------------

    def __enter__(self):
        self.bus = self.bus_factory(self.binding.channel, self.binding.models,
                                    motor_count=self.count).__enter__()
        try:
            self.binding.verify(self.bus)
            self.sample()
            if self.gravity is not None:
                self.bus.hold_gravity = lambda raw: np.append(
                    self.gravity.clipped(raw), np.zeros(self.count - 6))
        except BaseException:
            self.bus.__exit__(None, None, None)
            raise
        return self

    def __exit__(self, *args):
        self.bus.__exit__(*args)

    def prepare(self):
        """Require MIT mode and disable the watchdog; a normal host exit must not reset motors."""
        self.bus.prepare()

    def enable(self):
        """Take over at the freshly measured pose. Never target zero."""
        self.sample()
        self.bus.enable(self.raw)

    # --- measurement ---------------------------------------------------------

    @property
    def q(self):
        return self.raw - self.reference

    def _check(self, raw):
        if self.joint_limits:
            return self.range.validate(raw)
        raw = np.asarray(raw, dtype=float)
        if raw.shape != self.reference.shape or not np.all(np.isfinite(raw)):
            raise ValueError("Expected finite feedback for every selected follower motor.")
        if self.model is not None:
            self.model.validate(raw[:6] - self.reference[:6])
        return raw

    def support(self, raw):
        """Feedforward torque per controlled motor at a raw pose."""
        if self.gravity is None:
            return self._zero.copy()
        return np.append(self.gravity(raw[:6]), np.zeros(self.count - 6))

    def hold(self, torque=None):
        """Refresh what the motors hold if this process stops commanding them."""
        self.bus.set_hold(self.raw, self.support(self.raw) if torque is None else torque,
                          self.kp, self.kd)

    def sample(self):
        """Query positions, validate them, and refresh the exit hold. Returns relative angles."""
        self.sampled_at = time.monotonic()
        self.raw = self._check(self.bus.positions())
        self.hold()
        return self.q

    def telemetry(self):
        """Reported torque and temperature of the arm joints from the latest MIT replies."""
        return {"reported": [self.bus.feedback[i]["torque"] for i in range(1, 7)],
                "temperature": [self.bus.feedback[i]["temperature"] for i in range(1, 7)]}

    def feedback_states(self):
        """Position, velocity, torque, and timestamps from the latest MIT replies."""
        return self.bus.feedback_states(self.feedback.max_age_s)

    def _stopped(self):
        return self.stop is not None and self.stop.is_set()

    def _adopt_feedback(self):
        states = self.feedback_states()
        self.raw = self._check([state["position"] for state in states])
        self.acquired_at = min(state["received_at"] for state in states)
        self.hold()

    # --- commands ------------------------------------------------------------

    def command_torque(self, torque):
        """Pure torque at the sampled pose (hand guiding). Stale samples are refused."""
        self.bus.command(self.raw, torque, deadline=self.sampled_at + self.settings.max_feedback_age_s)

    def command_trajectory(self, record):
        """Send one `TrajectoryTracker` record as a native position/velocity command."""
        began = time.monotonic()
        parameters = self.bus.command_position(
            self.raw + record["target"] - record["q"], record["desired_velocity"],
            record["gravity"], self.kp, self.kd,
            self.sampled_at + self.settings.max_feedback_age_s)
        record["command_s"] = time.monotonic() - began
        # Like the reference PD sweep, derive quasi-static effort from the
        # native gains and tracking error; firmware velocity scaling is unused.
        record["estimated"] = np.array([
            item["gravity"] + item["kp"] * (item["position"] - q)
            + item["kd"] * (item["velocity"] - v)
            for item, q, v in zip(parameters, self.raw, record["velocity"])])
        record["native_parameters"] = parameters

    def _position_command(self, target, torque, began, *, host_gripper):
        """Native position command; `host_gripper` sends motor 7 as limited pure torque."""
        policy = self.feedback
        deadline = None if policy.cycle_budget_s is None else began + policy.cycle_budget_s
        kp, kd, limits = self.kp, self.kd, None
        if host_gripper and self.gripper is not None:
            kp, kd, limits = self.servo_kp, self.servo_kd, self.torque_limits
        return self.bus.command_position(
            target, self._zero, torque, kp, kd, deadline, torque_limits=limits,
            max_attempts=policy.attempts, reply_timeout_s=policy.reply_timeout_s, stop=self.stop)

    def refresh(self):
        """Command the measured pose once so every axis has fresh MIT feedback.

        The sequential enable handshake ages the first motors' replies; servo
        loops and the gripper controller must not start from them.
        Returns False when stopped first.
        """
        if self._position_command(self.raw, self.support(self.raw), time.monotonic(),
                                  host_gripper=False) is None or self._stopped():
            return False
        self._adopt_feedback()
        return True

    def servo(self, q, began=None):
        """One latest-target position cycle. Returns False when stopped before completing.

        Checks the tracking error against the last feedback, sends the target with
        gravity feedforward and the gripper's limited torque, then validates the
        replies and refreshes the exit hold.
        """
        began = time.monotonic() if began is None else began
        q = np.asarray(q, dtype=float)
        if q.shape != (self.count,) or not np.all(np.isfinite(q)):
            raise ValueError("Joint target must contain one finite value per motor.")
        target = self.reference + q
        max_error = self.settings.max_tracking_error_deg
        validate_tracking_error(target, self.raw, max_error)
        torque = self.support(self.raw)
        if self.gripper is not None:
            grip = self.feedback_states()[6]
            torque[6] = self.gripper.update(target[6], grip["position"], grip["velocity"])
        if self._stopped():
            return False
        if self._position_command(target, torque, began, host_gripper=True) is None:
            return False
        if self._stopped():
            return False
        # Each MIT command already returns position; avoid extra parameter queries.
        self._adopt_feedback()
        validate_tracking_error(target, self.raw, max_error)
        return True


@contextmanager
def open_arms(arms):
    """Open every arm (verified, validated, holding) before any of them is enabled."""
    with ExitStack() as stack:
        for arm in arms.values():
            stack.enter_context(arm)
        yield arms


def activate(arms, stopped=lambda: False):
    """Prepare, then enable, every opened arm. Returns False if stopped part-way."""
    for arm in arms.values():
        if stopped():
            return False
        arm.prepare()
    for arm in arms.values():
        if stopped():
            return False
        arm.enable()
    return not stopped()
