"""Bounded shared snapshots and scheduling for spawned teleoperation workers."""

from contextlib import contextmanager
import math
import signal
import time

import numpy as np


MOTOR_HZ = 200
TARGET_HZ = 500
FEEDBACK_ATTEMPTS = 15
FEEDBACK_TIMEOUT_S = .01
SUPERVISOR_TIMEOUT_S = .5


class StopSignal:
    """Defer signal-handler requests until outside multiprocessing locks."""

    def __init__(self, event):
        self._event = event
        self._signal_requested = False

    def request(self):
        # Python signal handlers may interrupt Event.wait/is_set/set while its
        # non-reentrant condition lock is held. Do not touch IPC here.
        self._signal_requested = True

    def set(self):
        self._event.set()

    def is_set(self):
        if self._signal_requested:
            self._event.set()
            self._signal_requested = False
        return self._event.is_set()

    def wait(self, timeout=None):
        # A local signal does not notify the shared event until we poll it.
        # Bound each wait so even long waits observe signal requests promptly.
        deadline = math.inf if timeout is None else time.monotonic() + max(0., timeout)
        while not self.is_set():
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return False
            self._event.wait(min(.01, remaining))
        return True


class SharedFrame:
    """One coherent latest frame; never queue targets or wait on a dead owner forever."""

    def __init__(self, context, size):
        self.size = size
        self.data = context.RawArray("d", size + 2)
        self.lock = context.Lock()

    @contextmanager
    def locked(self):
        if not self.lock.acquire(timeout=.01):
            raise TimeoutError("Shared frame lock did not become available.")
        try:
            yield
        finally:
            self.lock.release()

    def publish(self, values, source_at):
        values = np.asarray(values, dtype=float)
        if (values.shape != (self.size,) or not np.all(np.isfinite(values))
                or not math.isfinite(source_at) or source_at <= 0):
            raise ValueError("Invalid shared frame or acquisition timestamp.")
        with self.locked():
            self.data[:] = [time.monotonic(), source_at, *values]

    def read(self):
        with self.locked():
            published, source_at, *values = self.data[:]
        return np.asarray(values), published, source_at


def latest(frame, label):
    """Read the last complete snapshot without expiring a cached setpoint."""
    values, published, source_at = frame.read()
    if published <= 0 or source_at <= 0:
        raise TimeoutError(f"{label}: latest frame is missing.")
    return values, source_at


def fresh(frame, now, max_age, label):
    values, published, source_at = frame.read()
    # A writer may publish between the caller's clock read and our snapshot.
    now = max(now, time.monotonic())
    if (published <= 0 or source_at <= 0 or not 0 <= now - published <= max_age
            or not 0 <= now - source_at <= max_age):
        raise TimeoutError(f"{label}: latest frame is missing or stale.")
    return values, source_at


def supervisor_alive(heartbeat):
    fresh(heartbeat, time.monotonic(), SUPERVISOR_TIMEOUT_S, "Supervisor heartbeat")


def wait_phase(event, stop, heartbeat):
    while not stop.is_set():
        supervisor_alive(heartbeat)
        if event.wait(.005):
            return True
    return False


def child_signals(stop):
    # Handle termination cooperatively so active motor workers can install hold.
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda signum, frame: stop.request())


@contextmanager
def stop_on_error(stop):
    try:
        yield
    except BaseException:
        # Enter this context AFTER the hardware context: notify peers before
        # potentially slow CAN hold cleanup begins.
        stop.set()
        raise


class LoopTiming:
    def __init__(self, name, hz=None):
        self.name, self.hz = name, hz
        self.started = time.monotonic()
        self.previous = None
        self.count = self.skipped = 0
        self.max_gap = self.max_work = self.max_age = 0.
        self.next_tick = self.started

    def tick(self):
        now = time.monotonic()
        if self.previous is not None:
            self.max_gap = max(self.max_gap, now - self.previous)
        self.previous = now
        return now

    def complete(self, began, source_at):
        now = time.monotonic()
        self.count += 1
        self.max_work = max(self.max_work, now - began)
        self.max_age = max(self.max_age, now - source_at)

    def pace(self, stop):
        if self.hz is None:
            return
        period = 1 / self.hz
        self.next_tick += period
        now = time.monotonic()
        if self.next_tick < now:
            missed = int((now - self.next_tick) / period) + 1
            self.skipped += missed
            self.next_tick += missed * period
        stop.wait(max(0., self.next_tick - time.monotonic()))

    def report(self):
        elapsed = time.monotonic() - self.started
        return {"type": "timing", "worker": self.name, "count": self.count,
                "hz": self.count / max(elapsed, 1e-9), "requested_hz": self.hz,
                "max_gap_ms": self.max_gap * 1000, "max_work_ms": self.max_work * 1000,
                "max_source_age_ms": self.max_age * 1000, "skipped": self.skipped}
