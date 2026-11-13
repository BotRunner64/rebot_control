"""Latest RGB camera snapshots; camera reads never run on the control thread."""

import io
from pathlib import Path
import socket
import struct
import subprocess
import threading
import time

import numpy as np
from PIL import Image

from rebot_record_preview import packet


CAMERA_KEYS = ("base_0_rgb", "left_wrist_0_rgb", "right_wrist_0_rgb")
CAMERA_NAMES = ("top", "left_wrist", "right_wrist")
CAPTURE_SCRIPT = Path(__file__).with_name("rebot_policy_camera_capture.py")


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
            self.process = subprocess.Popen(
                ["/usr/bin/python3", "-u", str(CAPTURE_SCRIPT), "--source", source,
                 "--socket-fd", str(child.fileno())], pass_fds=(child.fileno(),),
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
                if not np.isfinite(acquired) or not 0 <= time.monotonic() - acquired <= 1.:
                    raise TimeoutError("Camera acquisition timestamp is invalid or stale.")
                with Image.open(io.BytesIO(payload[8:])) as image:
                    rgb = np.array(image.convert("RGB"))
                self.connection.settimeout(1.)
                with self.lock:
                    self.frame, self.acquired_at = rgb, acquired
                self.ready.set()
        except Exception as exc:
            if self.done.is_set():
                return
            with self.lock:
                self.error = exc
            self.ready.set()

    def snapshot(self, max_age):
        from openpi_client.image_tools import resize_with_pad

        with self.lock:
            if self.error is not None:
                raise RuntimeError(f"Camera {self.source} failed: {self.error}") from self.error
            if self.frame is None or not 0 <= time.monotonic() - self.acquired_at <= max_age:
                raise TimeoutError(f"Camera {self.source} has no fresh frame.")
            frame, stamp = self.frame.copy(), self.acquired_at
        return resize_with_pad(frame, 224, 224), stamp

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
    def __init__(self, cameras, *, mock=False, max_age=.25):
        self.streams = []
        self.mock, self.max_age = mock, max_age
        self.sources = None
        if not mock:
            from rebot_recording import parse_cameras

            self.sources = parse_cameras(cameras or [])
            if set(self.sources) != set(CAMERA_NAMES):
                raise ValueError("Name exactly three cameras: top, left_wrist, right_wrist.")
            if len({str(Path(p).resolve()) for p in self.sources.values()}) != 3:
                raise ValueError("Policy camera devices must be distinct.")

    def __enter__(self):
        try:
            if not self.mock:
                for name in CAMERA_NAMES:
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
        if self.mock:
            return ({key: np.zeros((224, 224, 3), np.uint8) for key in CAMERA_KEYS},
                    [time.monotonic()] * 3)
        frames = [stream.snapshot(self.max_age) for stream in self.streams]
        return dict(zip(CAMERA_KEYS, (f[0] for f in frames))), [f[1] for f in frames]

    def __exit__(self, *_):
        for stream in self.streams:
            stream.close()
