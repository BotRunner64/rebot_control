"""Background servo loop: the CAN owner runs at a fixed rate on the latest joint target."""

import threading
import time

import numpy as np

from rebot.runtime.console import request_stop
from rebot.runtime.timing import LoopTiming

SERVO_HZ = 250


class ArmWorker:
    """Own one enabled `Arm` on a thread; planners only exchange copied snapshots.

    Following YAM's cached-command architecture, IK and rendering never touch CAN:
    they publish the newest target with `command()` and read `snapshot()`.
    `on_feedback(side, states, command)` receives, on the servo thread, every cycle's motor
    replies and the command they answer: the joint `target` and the commanded `torque`.
    """

    def __init__(self, arm, stop, hz=SERVO_HZ, on_feedback=None):
        self.arm, self.stop, self.hz, self.on_feedback = arm, stop, hz, on_feedback
        self.lock = threading.Lock()
        self.target = arm.q.copy()
        self.measured = arm.q.copy()
        self.acquired_at = arm.acquired_at
        self.error = None
        self.done = threading.Event()
        self.thread = threading.Thread(target=self._run, name=f"servo-{arm.side}")

    def check(self):
        if self.error is not None:
            raise self.error

    def snapshot(self):
        """Latest measured joint angles and their acquisition time; stale data raises."""
        self.check()
        with self.lock:
            if time.monotonic() - self.acquired_at > self.arm.settings.max_feedback_age_s:
                raise TimeoutError("Motor feedback snapshot is stale.")
            return self.measured.copy(), self.acquired_at

    def read(self):
        return self.snapshot()[0]

    def command(self, q):
        q = np.asarray(q, dtype=float)
        if q.shape != (self.arm.count,) or not np.all(np.isfinite(q)):
            raise ValueError("Joint command must contain one finite value per motor.")
        q = np.clip(q, self.arm.range.lower, self.arm.range.upper)
        self.check()
        with self.lock:
            self.target = q

    def _run(self):
        timing = LoopTiming(self.thread.name, self.hz)
        try:
            while not self.stop.is_set() and not self.done.is_set():
                began = timing.tick()
                with self.lock:
                    target = self.target.copy()
                if not self.arm.servo(target, began):
                    break
                with self.lock:
                    self.measured = self.arm.q.copy()
                    self.acquired_at = self.arm.acquired_at
                if self.on_feedback is not None:
                    self.on_feedback(self.arm.side, self.arm.feedback_states(), self.arm.commanded)
                timing.complete(began, time.monotonic())
                timing.pace(self.done)
        except BaseException as exc:
            self.error = exc
            request_stop(self.stop, f"CAN worker failed: {exc}")
        finally:
            report = timing.report()
            print(f"{self.thread.name}: {report['hz']:.1f} Hz; max work {report['max_work_ms']:.1f} ms; "
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
