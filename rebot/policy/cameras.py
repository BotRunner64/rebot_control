"""Latest JPEG camera snapshots; camera reads never run on the control thread."""

import math
from pathlib import Path
import socket
import struct
import subprocess
import threading
import time

from rebot.media.record_preview import packet
from rebot.paths import system_python


CAPTURE_MODULE = "rebot.media.policy_capture"


class CameraStream:
    def __init__(self, source):
        self.source = source
        self.lock = threading.Lock()
        self.frame = self.error = None
        self.acquired_at = 0.
        self.done = threading.Event()
        self.ready = threading.Event()
        self.connection, child = socket.socketpair()
        self.connection.settimeout(12.)
        try:
            command, env = system_python(CAPTURE_MODULE, "--source", source,
                                         "--socket-fd", str(child.fileno()), flags=("-B", "-u"))
            self.process = subprocess.Popen(command, env=env, pass_fds=(child.fileno(),),
                                            stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL)
        except BaseException:
            self.connection.close()
            raise
        finally:
            child.close()
        self.thread = threading.Thread(target=self._run, name=f"policy-camera-{source}", daemon=True)
        try:
            self.thread.start()
        except BaseException:
            self.process.terminate()
            self.process.wait(timeout=2.)
            self.connection.close()
            raise

    def _run(self):
        try:
            while not self.done.is_set():
                payload = packet(self.connection, 4_000_000)
                if len(payload) <= 8:
                    raise ValueError("Incomplete camera sample.")
                acquired = struct.unpack("!d", payload[:8])[0]
                if not math.isfinite(acquired) or not 0 <= time.monotonic() - acquired <= 1.:
                    raise TimeoutError("Camera acquisition timestamp is invalid or stale.")
                # Forward the capture process's JPEG untouched; the policy side decodes and resizes.
                if payload[8:10] != b"\xff\xd8":
                    raise ValueError("Camera sample is not a JPEG image.")
                self.connection.settimeout(1.)
                with self.lock:
                    self.frame, self.acquired_at = payload[8:], acquired
                self.ready.set()
        except Exception as exc:
            if self.done.is_set():
                return
            with self.lock:
                self.error = exc
            self.ready.set()

    def snapshot(self, max_age):
        with self.lock:
            if self.error is not None:
                raise RuntimeError(f"Camera {self.source} failed: {self.error}") from self.error
            if self.frame is None or not 0 <= time.monotonic() - self.acquired_at <= max_age:
                raise TimeoutError(f"Camera {self.source} has no fresh frame.")
            return self.frame, self.acquired_at

    def close(self):
        self.done.set()
        try:
            self.connection.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        if self.process.poll() is None:
            self.process.terminate()
        try:
            self.process.wait(timeout=2.)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait()
        self.thread.join(timeout=1.5)
        self.connection.close()


class PolicyCameras:
    """Exactly the named camera streams a policy requires; snapshot() -> {name: (jpeg, acquired_at)}."""

    def __init__(self, cameras, names, *, max_age=.25):
        from rebot.data.recording import parse_cameras

        self.streams = []
        self.names, self.max_age = tuple(names), max_age
        self.sources = parse_cameras(cameras or [])
        if set(self.sources) != set(self.names):
            raise ValueError(f"Name exactly the cameras this policy requires: {', '.join(self.names)}.")
        if len({str(Path(p).resolve()) for p in self.sources.values()}) != len(self.sources):
            raise ValueError("Policy camera devices must be distinct.")

    def __enter__(self):
        try:
            for name in self.names:
                self.streams.append(CameraStream(self.sources[name]))
            deadline = time.monotonic() + 10.
            for stream in self.streams:
                if not stream.ready.wait(max(0., deadline - time.monotonic())):
                    raise TimeoutError(f"No first frame from {stream.source} within 10 seconds.")
            self.snapshot()
            return self
        except BaseException:
            self.__exit__()
            raise

    def snapshot(self):
        return {name: stream.snapshot(self.max_age) for name, stream in zip(self.names, self.streams)}

    def __exit__(self, *_):
        for stream in self.streams:
            stream.close()
