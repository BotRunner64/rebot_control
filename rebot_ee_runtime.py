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

    def __init__(self, bus, reference, lower, upper, cfg, stop, gravity):
        self.bus, self.cfg, self.stop = bus, cfg, stop
        self.gravity = gravity
        self.reference, self.lower, self.upper = reference, lower, upper
        self.lock = threading.Lock()
        self.raw = validate_pose(bus.feedback_positions(cfg["max_feedback_age_s"]),
                                 reference, lower, upper).copy()
        self.target = self.raw.copy()
        self.error = None
        self.done = threading.Event()
        self.thread = threading.Thread(target=self._run, name="ee-can")

    def read(self):
        self.check()
        with self.lock:
            return self.raw.copy() - self.reference

    def check(self):
        if self.error is not None:
            raise self.error

    def command(self, q):
        q = np.clip(vector6(q, "Joint command"), self.lower, self.upper)
        self.check()
        with self.lock:
            self.target = self.reference + q

    def _run(self):
        timing = LoopTiming("ee-can", 250)
        zero = np.zeros(6)
        try:
            while not self.stop.is_set() and not self.done.is_set():
                began = timing.tick()
                with self.lock:
                    target = self.target.copy()
                    measured = self.raw.copy()
                validate_tracking_error(target, measured, self.cfg["max_tracking_error_deg"])
                gravity = self.gravity(measured)
                self.bus.command_position(target, zero, gravity, MOTION_KP, MOTION_KD,
                                          began + self.cfg["max_feedback_age_s"])
                raw = validate_pose(self.bus.feedback_positions(self.cfg["max_feedback_age_s"]),
                                    self.reference, self.lower, self.upper)
                self.bus.set_hold(raw, self.gravity(raw), MOTION_KP, MOTION_KD)
                validate_tracking_error(target, raw, self.cfg["max_tracking_error_deg"])
                with self.lock:
                    self.raw = raw.copy()
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
