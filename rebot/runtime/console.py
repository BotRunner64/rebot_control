"""Stopping a controller: terminal keys, POSIX signals, and one shared stop object."""

from contextlib import contextmanager
import math
import os
import select
import signal
import sys
import threading
import time

STOP_SIGNALS = (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)


class StopSignal:
    """An event that is safe to request from a signal handler.

    Wraps a `threading.Event` (default) or a multiprocessing event shared with
    worker processes. Python signal handlers may interrupt `Event.set/wait`
    while its non-reentrant lock is held, so `request()` only raises a local
    flag; the event itself is set on the next `is_set()`/`wait()` poll.
    """

    def __init__(self, event=None):
        self._event = threading.Event() if event is None else event
        self._signal_requested = False
        self.reason = None

    def request(self, reason=None):
        if self.reason is None:
            self.reason = reason
        self._signal_requested = True

    def set(self, reason=None):
        """Stop now, keeping the first explicit reason for the shutdown report."""
        if self.reason is None and not self._event.is_set():
            self.reason = reason
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


def request_stop(stop, reason):
    """Set any stop object, recording why when it can carry a reason."""
    if isinstance(stop, StopSignal):
        stop.set(reason)
    elif not stop.is_set():
        stop.stop_reason = reason
        stop.set()


def stop_reason(stop):
    return getattr(stop, "reason", None) or getattr(stop, "stop_reason", None)


def handle_signals(stop):
    """Route SIGINT/SIGTERM/SIGHUP to `stop`; return the previous handlers."""
    def received(signum, frame):
        reason = f"received {signal.Signals(signum).name}"
        if isinstance(stop, StopSignal):
            stop.request(reason)
        else:
            request_stop(stop, reason)

    return {sig: signal.signal(sig, received) for sig in STOP_SIGNALS}


@contextmanager
def stop_signals(stop):
    """Stop cooperatively on termination signals so controllers can install hold."""
    previous = handle_signals(stop)
    try:
        yield stop
    finally:
        for sig, handler in previous.items():
            signal.signal(sig, handler)
        if stop.is_set() and stop_reason(stop):
            print(f"Stop reason: {stop_reason(stop)}", flush=True)


class StopKey:
    """Read Space/Esc/Q without Enter, restoring terminal settings on exit."""

    def __enter__(self):
        self.fd = self.saved = None
        if sys.stdin.isatty():
            import termios
            import tty

            self.fd = sys.stdin.fileno()
            self.saved = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)
        return self

    def pressed(self):
        if self.fd is None or not select.select([self.fd], [], [], 0)[0]:
            return False
        data = os.read(self.fd, 64)
        return any(key in data for key in (b" ", b"\x1b", b"q", b"Q"))

    def __exit__(self, *args):
        if self.saved is not None:
            import termios

            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.saved)
