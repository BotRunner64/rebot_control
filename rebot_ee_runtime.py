"""Independent latest-command MIT loop, following YAM's cached-command architecture."""

import threading
import time

import numpy as np

from rebot_kinematics import vector6
from rebot_motion import MOTION_KD, MOTION_KP
from rebot_teleop import validate_pose
from rebot_teleop_runtime import LoopTiming


def request_stop(stop, reason):
    """Keep the first explicit stop source for the shutdown report."""
    if not stop.is_set():
        stop.stop_reason = reason
        stop.set()


class MotorWorker:
    """Own CAN while running; IK and rendering only exchange copied snapshots."""

    def __init__(self, bus, reference, lower, upper, cfg, stop):
        self.bus, self.cfg, self.stop = bus, cfg, stop
        self.reference, self.lower, self.upper = reference, lower, upper
        self.lock = threading.Lock()
        self.raw = validate_pose(bus.feedback_positions(cfg["max_feedback_age_s"]),
                                 reference, lower, upper).copy()
        self.target = self.raw.copy()
        self.kp, self.kd = MOTION_KP.copy(), MOTION_KD.copy()
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

    def command(self, q, kp, kd):
        q = np.clip(vector6(q, "Joint command"), self.lower, self.upper)
        kp, kd = vector6(kp, "KP"), vector6(kd, "KD")
        self.check()
        with self.lock:
            self.target = self.reference + q
            self.kp, self.kd = kp, kd

    def _run(self):
        timing = LoopTiming("ee-can", 250)
        zero = np.zeros(6)
        try:
            while not self.stop.is_set() and not self.done.is_set():
                began = timing.tick()
                with self.lock:
                    target, kp, kd = self.target.copy(), self.kp.copy(), self.kd.copy()
                self.bus.command_position(target, zero, zero, kp, kd,
                                          began + self.cfg["max_feedback_age_s"])
                raw = validate_pose(self.bus.feedback_positions(self.cfg["max_feedback_age_s"]),
                                    self.reference, self.lower, self.upper)
                # Cleanup always holds with the established reBot position gains,
                # even when the selected UI mode has zero PD gains.
                self.bus.set_hold(raw, zero, MOTION_KP, MOTION_KD)
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
