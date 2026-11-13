"""Independent latest-command MIT loop, following YAM's cached-command architecture."""

import threading
import time

import numpy as np

from rebot_kinematics import vector6
from rebot_motion import MOTION_KD, MOTION_KP
from rebot_safety import validate_tracking_error
from rebot_teleop import validate_pose
from rebot_teleop_runtime import LoopTiming


def request_stop(stop, reason):
    """Keep the first explicit stop source for the shutdown report."""
    if not stop.is_set():
        stop.stop_reason = reason
        stop.set()


class GravityFeedforward:
    """Compute bounded gravity support from raw feedback and software references."""

    def __init__(self, model, reference, cfg):
        self.model = model
        self.reference = vector6(reference, "Software reference")
        self.scale = vector6(cfg["gravity_scale"], "Gravity scale")
        self.limit = vector6(cfg["torque_limit_nm"], "Gravity torque limits")

    def __call__(self, raw):
        q = vector6(raw, "Motor feedback") - self.reference
        gravity = vector6(self.model.gravity(q) * self.scale, "Gravity feedforward")
        if np.any(np.abs(gravity) > self.limit):
            raise RuntimeError("Required gravity torque exceeds configured EE feedforward limits.")
        return gravity


class MotorWorker:
    """Own CAN while running; IK and rendering only exchange copied snapshots."""

    def __init__(self, bus, reference, lower, upper, cfg, stop, gravity, *, gripper=None):
        self.bus, self.cfg, self.stop = bus, cfg, stop
        self.gravity = gravity
        self.reference, self.lower, self.upper = reference, lower, upper
        self.gripper = gripper
        self.count = len(reference)
        if self.count != (7 if gripper is not None else 6):
            raise ValueError("Use six arm joints, or seven joints with a gripper controller.")
        self.kp, self.kd = MOTION_KP, MOTION_KD
        self.command_options = {}
        if gripper is not None:
            self.kp = np.append(MOTION_KP, gripper.settings.kp)
            self.kd = np.append(MOTION_KD, gripper.settings.kd)
            self.command_options = {"torque_limits": {7: gripper.settings.torque_limit_nm}, "stop": stop}
        self.control_kp, self.control_kd = self.kp.copy(), self.kd.copy()
        if gripper is not None:
            self.control_kp[6] = self.control_kd[6] = 0.
        self.lock = threading.Lock()
        self.raw = validate_pose(bus.feedback_positions(cfg["max_feedback_age_s"]),
                                 reference, lower, upper).copy()
        self.target = self.raw.copy()
        self.acquired_at = self._acquisition_time()
        self.error = None
        self.done = threading.Event()
        self.thread = threading.Thread(target=self._run, name="ee-can")

    def read(self):
        return self.snapshot()[0]

    def _acquisition_time(self):
        if self.gripper is None:
            return time.monotonic()
        return min(state["received_at"] for state in self.bus.feedback_states(self.cfg["max_feedback_age_s"]))

    def snapshot(self):
        self.check()
        with self.lock:
            if time.monotonic() - self.acquired_at > self.cfg["max_feedback_age_s"]:
                raise TimeoutError("Motor feedback snapshot is stale.")
            return self.raw.copy() - self.reference, self.acquired_at

    def check(self):
        if self.error is not None:
            raise self.error

    def command(self, q):
        q = np.asarray(q, dtype=float)
        if q.shape != (self.count,) or not np.all(np.isfinite(q)):
            raise ValueError("Joint command must contain one finite value per motor.")
        q = np.clip(q, self.lower, self.upper)
        self.check()
        with self.lock:
            self.target = self.reference + q

    def _run(self):
        timing = LoopTiming("ee-can", 250)
        zero = np.zeros(self.count)
        try:
            while not self.stop.is_set() and not self.done.is_set():
                began = timing.tick()
                with self.lock:
                    target = self.target.copy()
                    measured = self.raw.copy()
                validate_tracking_error(target, measured, self.cfg["max_tracking_error_deg"])
                gravity = self.gravity(measured)
                if self.gripper is not None:
                    grip = self.bus.feedback_states(self.cfg["max_feedback_age_s"])[6]
                    gravity[6] = self.gripper.update(target[6], grip["position"], grip["velocity"])
                if self.stop.is_set() or self.done.is_set():
                    break
                self.bus.command_position(target, zero, gravity, self.control_kp, self.control_kd,
                                          began + self.cfg["max_feedback_age_s"], **self.command_options)
                if self.gripper is not None and self.stop.is_set():
                    break
                raw = validate_pose(self.bus.feedback_positions(self.cfg["max_feedback_age_s"]),
                                    self.reference, self.lower, self.upper)
                self.bus.set_hold(raw, self.gravity(raw), self.kp, self.kd)
                validate_tracking_error(target, raw, self.cfg["max_tracking_error_deg"])
                acquired_at = self._acquisition_time()
                with self.lock:
                    self.raw = raw.copy()
                    self.acquired_at = acquired_at
                timing.complete(began, time.monotonic())
                timing.pace(self.done)
        except BaseException as exc:
            self.error = exc
            request_stop(self.stop, f"CAN worker failed: {exc}")
        finally:
            report = timing.report()
            print(f"EE CAN: {report['hz']:.1f} Hz; max work {report['max_work_ms']:.1f} ms; "
                  f"skipped periods {report['skipped']}.", flush=True)

    def __enter__(self):
        self.thread.start()
        return self

    def __exit__(self, *args):
        self.done.set()
        # Transport transactions are bounded; join before the bus installs exit hold.
        self.thread.join()
        if args[0] is None:
            self.check()
