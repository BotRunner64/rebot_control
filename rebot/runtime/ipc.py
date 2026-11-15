"""Shared latest-value frames and supervision for spawned worker processes."""

from contextlib import contextmanager
import math
import time

import numpy as np


SUPERVISOR_TIMEOUT_S = .5


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


@contextmanager
def stop_on_error(stop):
    try:
        yield
    except BaseException:
        # Enter this context AFTER the hardware context: notify peers before
        # potentially slow CAN hold cleanup begins.
        stop.set()
        raise
