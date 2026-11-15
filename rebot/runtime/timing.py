"""Fixed-rate loop pacing with timing statistics."""

import time


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
