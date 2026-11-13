#!/usr/bin/env python3
"""Expose the existing GStreamer CameraPreview to a Conda control process."""

import argparse
from contextlib import redirect_stdout
import signal
import socket
import struct
import sys
import threading
from types import SimpleNamespace


def capture(source, socket_fd):
    import gi

    gi.require_version("Gst", "1.0")
    from gi.repository import GLib, Gst
    from webcam_viewer import CameraPreview

    Gst.init(None)
    loop = GLib.MainLoop()
    camera = CameraPreview(source, SimpleNamespace(Gst=Gst), loop)
    stop = threading.Event()
    old = {sig: signal.signal(sig, lambda *_: stop.set())
           for sig in (signal.SIGTERM, signal.SIGINT, signal.SIGHUP)}
    connection = socket.socket(fileno=socket_fd)
    connection.settimeout(.5)
    runner = threading.Thread(target=loop.run, name="camera-gst", daemon=True)
    try:
        camera.start()
        runner.start()
        sequence = 0
        while not stop.is_set():
            sample = camera.wait_sample(sequence, timeout=.25)
            if sample is None:
                camera.check_timeout()
                if camera.failed:
                    raise RuntimeError(camera.error)
                continue
            sequence, jpeg, acquired_at = sample
            # Reuse the existing length-prefixed preview transport, adding a timestamp.
            payload = struct.pack("!d", acquired_at) + jpeg
            connection.sendall(struct.pack("!I", len(payload)) + payload)
    except (BrokenPipeError, ConnectionResetError):
        pass
    finally:
        camera.close()
        loop.quit()
        if runner.is_alive():
            runner.join(timeout=1.)
        connection.close()
        for sig, handler in old.items():
            signal.signal(sig, handler)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", required=True)
    parser.add_argument("--socket-fd", required=True, type=int)
    args = parser.parse_args()
    try:
        with redirect_stdout(sys.stderr):
            capture(args.source, args.socket_fd)
    except Exception as exc:
        print(f"Camera {args.source}: {exc}", file=sys.stderr, flush=True)
        raise SystemExit(2)
