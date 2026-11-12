"""Best-effort camera previews; never open capture devices or wait in a recording callback."""

import argparse
import json
import os
from pathlib import Path
import queue
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time


class PreviewPublisher:
    """Keep at most one pending buffer; socket I/O runs outside the capture thread."""

    def __init__(self, path):
        self.path = str(path)
        self.frames = queue.Queue(maxsize=1)
        self.stopping = threading.Event()
        self.next_frame = self.retry_at = 0.
        self.thread = threading.Thread(target=self.run, name="preview-send", daemon=True)
        self.thread.start()

    def offer(self, buffer, caps, now):
        if self.stopping.is_set() or now < max(self.next_frame, self.retry_at):
            return
        self.next_frame = now + .1
        try:
            self.frames.put_nowait((buffer.copy(), caps))
        except queue.Full:
            pass
        except Exception:
            self.stopping.set()

    def run(self):
        connection = None
        try:
            while not self.stopping.is_set():
                try:
                    buffer, caps = self.frames.get(timeout=.1)
                except queue.Empty:
                    continue
                try:
                    if connection is None:
                        connection = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                        connection.settimeout(.1)
                        connection.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65536)
                        connection.connect(self.path)
                        header = caps.encode()
                        connection.sendall(struct.pack("!I", len(header)))
                        connection.sendall(header)
                    frame = buffer.extract_dup(0, buffer.get_size())
                    connection.sendall(struct.pack("!I", len(frame)))
                    connection.sendall(frame)
                except Exception:
                    if connection is not None:
                        connection.close(); connection = None
                    self.retry_at = time.monotonic() + 1.
                finally:
                    del buffer
        finally:
            if connection is not None:
                connection.close()
            while True:
                try:
                    self.frames.get_nowait()
                except queue.Empty:
                    break

    def close(self):
        self.stopping.set()
        self.thread.join(.5)


class RecordingViewer:
    def __init__(self, cameras, *, open_browser=True, port=8080):
        self.directory = tempfile.TemporaryDirectory(prefix="rebot-live-camera-")
        self.root = Path(self.directory.name)
        self.process = None
        self.paths = {name: self.root / f"{index}.sock" for index, name in enumerate(cameras)}
        config = {"cameras": [{"name": name, "source": device, "socket": str(self.paths[name])}
                              for name, device in cameras.items()],
                  "port": port, "open_browser": open_browser}
        path = self.root / "viewer.json"
        try:
            path.write_text(json.dumps(config))
            self.process = subprocess.Popen(["/usr/bin/python3", "-B", str(Path(__file__).resolve()),
                                             "--config", str(path)], stdin=subprocess.DEVNULL)
        except BaseException:
            self.close()
            raise

    def close(self):
        if self.process is not None:
            if self.process.poll() is None:
                self.process.terminate()
                try:
                    self.process.wait(2.)
                except subprocess.TimeoutExpired:
                    self.process.kill(); self.process.wait()
        self.directory.cleanup()


def packet(connection, maximum):
    def read(size):
        value = bytearray()
        while len(value) < size:
            part = connection.recv(size - len(value))
            if not part:
                raise ConnectionError("Preview sender disconnected.")
            value.extend(part)
        return bytes(value)
    size = struct.unpack("!I", read(4))[0]
    if not 0 < size <= maximum:
        raise ValueError("Invalid preview packet size.")
    return read(size)


def serve(config_path):
    # Encoding and HTTP clients have their own process and lower CPU priority.
    os.nice(10)
    from types import SimpleNamespace
    import webbrowser
    import gi
    gi.require_version("Gst", "1.0")
    from gi.repository import GLib, Gst
    from webcam_viewer import CameraPreview
    from webcam_web import WebcamServer

    Gst.init(None)
    loop = GLib.MainLoop()
    runtime = SimpleNamespace(Gst=Gst)

    class RecordingCamera(CameraPreview):
        def __init__(self, camera):
            super().__init__(f"{camera['name']} · {camera['source']}", runtime, loop)
            self.path = Path(camera["socket"])
            self.mode = "Recording preview · up to 640 px / 10 FPS"
            self.connection = None
            self.listener = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            self.listener.settimeout(.25)
            self.listener.bind(str(self.path)); self.listener.listen(1)
            self.started_at = time.monotonic()
            self.reader = threading.Thread(target=self.receive, name="preview-receive", daemon=True)
            self.reader.start()

        def encode(self, caps):
            if self.pipeline is not None:
                return self.pipeline.get_by_name("input")
            info = caps.get_structure(0)
            width, height = info.get_int("width")[1], info.get_int("height")[1]
            if info.get_string("format") != "I420" or not 0 < max(width, height) <= 960:
                raise ValueError("Unsupported recording preview format.")
            scale = min(1., 640 / max(width, height))
            width, height = max(2, int(width * scale) // 2 * 2), max(2, int(height * scale) // 2 * 2)
            self.pipeline = Gst.parse_launch(
                "appsrc name=input is-live=true format=time do-timestamp=true block=false "
                "max-buffers=1 max-bytes=0 max-time=0 leaky-type=downstream emit-signals=false "
                "! videoscale n-threads=1 "
                f"! video/x-raw,width={width},height={height},pixel-aspect-ratio=1/1 "
                "! jpegenc quality=65 "
                "! appsink name=preview emit-signals=true max-buffers=1 drop=true sync=false")
            source = self.pipeline.get_by_name("input")
            source.set_property("caps", caps)
            self.pipeline.get_by_name("preview").connect("new-sample", self.on_sample)
            self.bus = self.pipeline.get_bus()
            self.bus.add_signal_watch(); self.bus.connect("message::error", self.on_error)
            if self.pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
                raise RuntimeError("Could not start preview encoder.")
            return source

        def receive(self):
            while not self.closed and not self.failed:
                try:
                    connection, _ = self.listener.accept()
                except socket.timeout:
                    continue
                except OSError:
                    break
                try:
                    with connection:
                        self.connection = connection
                        connection.settimeout(1.)
                        caps = Gst.Caps.from_string(packet(connection, 8192).decode())
                        source = self.encode(caps)
                        while not self.closed and not self.failed:
                            data = packet(connection, 4_000_000)
                            buffer = Gst.Buffer.new_allocate(None, len(data), None)
                            buffer.fill(0, data)
                            if source.emit("push-buffer", buffer) != Gst.FlowReturn.OK:
                                raise RuntimeError("Preview encoder stopped.")
                except (OSError, ConnectionError):
                    continue
                except Exception as exc:
                    self.fail(str(exc))
                finally:
                    self.connection = None

        def close(self):
            self.closed = True
            if self.connection is not None:
                try:
                    self.connection.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
            super().close()
            self.listener.close()
            self.reader.join(1.5)

    config = json.loads(config_path.read_text())
    cameras, server, callbacks = [], None, []
    try:
        for camera in config["cameras"]:
            cameras.append(RecordingCamera(camera))
        server = WebcamServer(cameras, lambda: GLib.idle_add(loop.quit), port=config["port"], recording=True)
        (config_path.parent / "url").write_text(server.url)
        print(f"Recording camera viewer: {server.url}", flush=True)
        if config["open_browser"]:
            def open_browser():
                try:
                    webbrowser.open(server.url, new=2)
                except Exception:
                    pass
            threading.Thread(target=open_browser, daemon=True).start()
        def quit_preview():
            loop.quit()
            return False
        def check_previews():
            for camera in cameras:
                camera.check_timeout()
            return True
        callbacks.append(GLib.timeout_add(1000, check_previews))
        callbacks.extend(GLib.unix_signal_add(GLib.PRIORITY_DEFAULT, sig, quit_preview)
                         for sig in (signal.SIGTERM, signal.SIGINT, signal.SIGHUP))
        loop.run()
    finally:
        for camera in cameras:
            camera.close()
        if server is not None:
            server.close()
        for callback in callbacks:
            if GLib.MainContext.default().find_source_by_id(callback):
                GLib.source_remove(callback)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    args = parser.parse_args()
    try:
        serve(args.config)
    except Exception as exc:
        print(f"Camera preview unavailable; recording is unaffected: {exc}", file=sys.stderr)
        raise SystemExit(1)
