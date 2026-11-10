#!/usr/bin/env python3
"""Preview Linux webcams in a browser using GStreamer and automatic capture modes."""

import argparse
import os
from pathlib import Path
import signal
import sys
import time
import threading
from types import SimpleNamespace
import webbrowser

from webcam_web import WebcamServer

FIRST_FRAME_TIMEOUT = 10.0


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cameras", nargs="+",
                        help="Camera indices or device paths (default: auto-detect).")
    parser.add_argument("--port", type=int, default=8080, help="Starting loopback HTTP port (default: 8080)")
    parser.add_argument("--no-browser", action="store_true", help="Print the viewer URL without opening it")
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error("port must be between 1 and 65535")
    if args.cameras is not None:
        args.cameras = [f"/dev/video{source}" if source.isdecimal() else source
                        for source in args.cameras]
        if len(set(args.cameras)) != len(args.cameras):
            parser.error("camera sources must be unique")
    return args


def load_runtime():
    """Use system GStreamer bindings, including from a Conda shell."""
    try:
        import gi
        gi.require_version("Gst", "1.0")
        from gi.repository import GLib, Gst
        return SimpleNamespace(GLib=GLib, Gst=Gst)
    except (ImportError, ValueError) as exc:
        system_python = Path("/usr/bin/python3")
        if system_python.exists() and system_python.resolve() != Path(sys.executable).resolve():
            print("Using system Python for GStreamer bindings.", flush=True)
            os.execv(str(system_python), [str(system_python), str(Path(__file__).resolve()),
                                         *sys.argv[1:]])
        raise RuntimeError(
            "Missing GStreamer bindings. Install python3-gi and gir1.2-gstreamer-1.0."
        ) from exc


def discover_cameras(Gst):
    """Let GStreamer discover video capture devices and exclude metadata nodes."""
    print("Discovering video capture devices...", flush=True)
    monitor = Gst.DeviceMonitor.new()
    monitor.add_filter("Video/Source", None)
    try:
        if not monitor.start():
            raise RuntimeError("Could not start camera discovery.")
        sources = set()
        for device in monitor.get_devices():
            properties = device.get_properties()
            path = None
            if properties is not None:
                path = properties.get_string("device.path") or properties.get_string("api.v4l2.path")
            if path and path.startswith("/dev/video"):
                sources.add(path)
        return sorted(sources, key=lambda path: int(Path(path).name[5:]))
    finally:
        monitor.stop()


def choose_format(caps):
    """Match Cheese: highest resolution at width >= 640 and FPS >= 15, else highest."""
    modes = []
    # Normalize expands lists of discrete formats into individual structures.
    caps = caps.normalize()
    for index in range(caps.get_size()):
        candidate = caps.copy_nth(index)
        info = candidate.get_structure(0)
        if info.get_name() not in ("image/jpeg", "video/x-raw"):
            continue
        # These calls also handle devices advertising ranges instead of discrete modes.
        info.fixate_field_nearest_int("width", 2147483647)
        info.fixate_field_nearest_int("height", 2147483647)
        info.fixate_field_nearest_fraction("framerate", 2147483647, 1)
        valid_width, width = info.get_int("width")
        valid_height, height = info.get_int("height")
        valid_fps, numerator, denominator = info.get_fraction("framerate")
        if not (valid_width and valid_height and valid_fps and width > 0
                and height > 0 and numerator > 0 and denominator > 0):
            continue
        # PyGObject returns a boxed structure copy; rebuild caps from the fixed fields.
        candidate = type(caps).from_string(info.to_string())
        modes.append((width * height, width, numerator / denominator, candidate))
    if not modes:
        raise RuntimeError("Camera has no supported raw video or MJPEG capture modes.")
    preferred = [mode for mode in modes if mode[1] >= 640 and mode[2] >= 15]
    return max(preferred or modes, key=lambda mode: (mode[0], mode[2]))[3].fixate()


class CameraPreview:
    """Capture JPEG previews into a bounded latest-frame slot for browser clients."""

    def __init__(self, source, runtime, loop):
        self.source = source
        self.runtime = runtime
        self.loop = loop
        self.pipeline = None
        self.bus = None
        self.first_frame = False
        self.failed = False
        self.started_at = 0.0
        self.mode = "Selecting capture mode"
        self.error = None
        self.closed = False
        self.frame = None
        self.sequence = 0
        self.last_frame_at = 0.
        self.condition = threading.Condition()

    def start(self):
        Gst = self.runtime.Gst
        print(f"Camera {self.source}: querying supported capture modes...", flush=True)
        self.pipeline = Gst.parse_launch(
            "v4l2src name=camera ! capsfilter name=mode ! decodebin "
            "! videoconvert ! videoscale ! capsfilter name=preview_size "
            "! jpegenc quality=85 ! appsink name=preview emit-signals=true "
            "max-buffers=1 drop=true sync=false"
        )
        camera = self.pipeline.get_by_name("camera")
        camera.set_property("device", self.source)
        if self.pipeline.set_state(Gst.State.READY) == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError("Could not open camera.")
        caps = camera.get_static_pad("src").query_caps(None)
        selected = choose_format(caps)
        self.pipeline.get_by_name("mode").set_property("caps", selected)
        info = selected.get_structure(0)
        _, width = info.get_int("width")
        _, height = info.get_int("height")
        _, numerator, denominator = info.get_fraction("framerate")
        self.mode = f"{width} x {height} / {numerator / denominator:g} FPS"
        print(f"Camera {self.source}: selected {self.mode}, {info.get_name()}.", flush=True)

        # Scale only the displayed image; capture retains the full selected frame.
        scale = min(640 / width, 480 / height, 1)
        preview_width = max(2, int(width * scale) // 2 * 2)
        preview_height = max(2, int(height * scale) // 2 * 2)
        self.pipeline.get_by_name("preview_size").set_property(
            "caps", Gst.Caps.from_string(f"video/x-raw,width={preview_width},"
                                         f"height={preview_height},pixel-aspect-ratio=1/1")
        )
        self.pipeline.get_by_name("preview").connect("new-sample", self.on_sample)
        self.bus = self.pipeline.get_bus()
        self.bus.add_signal_watch()
        self.bus.connect("message::error", self.on_error)
        self.bus.connect("message::eos", lambda *args: self.fail("Camera stream ended."))
        self.started_at = time.monotonic()
        if self.pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError("Could not start camera stream.")

    def on_sample(self, sink):
        sample = sink.emit("pull-sample")
        if sample is None:
            return self.runtime.Gst.FlowReturn.EOS
        buffer = sample.get_buffer()
        frame = buffer.extract_dup(0, buffer.get_size())
        with self.condition:
            if not self.first_frame:
                print(f"Camera {self.source}: first frame received.", flush=True)
            self.first_frame = True
            self.last_frame_at = time.monotonic()
            self.frame = frame
            self.sequence += 1
            self.condition.notify_all()
        return self.runtime.Gst.FlowReturn.OK

    def wait_frame(self, sequence, timeout=1.):
        with self.condition:
            self.condition.wait_for(
                lambda: self.sequence != sequence or self.failed or self.closed, timeout)
            if self.failed or self.closed or self.sequence == sequence:
                return None
            return self.sequence, self.frame

    def snapshot(self):
        with self.condition:
            state = "error" if self.failed else "stopped" if self.closed else "live" if self.first_frame else "waiting"
            return {"source": self.source, "mode": self.mode, "state": state, "error": self.error}

    def on_error(self, bus, message):
        error, _ = message.parse_error()
        self.fail(error.message)

    def fail(self, message):
        with self.condition:
            if self.failed:
                return
            self.failed = True
            self.error = message
            self.frame = None
            self.condition.notify_all()
        print(f"Camera {self.source}: {message}", file=sys.stderr, flush=True)
        if self.pipeline is not None:
            self.pipeline.set_state(self.runtime.Gst.State.NULL)

    def check_timeout(self):
        if not self.failed and not self.first_frame and time.monotonic() - self.started_at >= FIRST_FRAME_TIMEOUT:
            self.fail("No first frame within 10 seconds.")
        elif not self.failed and self.first_frame and time.monotonic() - self.last_frame_at >= FIRST_FRAME_TIMEOUT:
            self.fail("No camera frame for 10 seconds.")

    def close(self):
        with self.condition:
            self.closed = True
            self.frame = None
            self.condition.notify_all()
        if self.bus is not None:
            self.bus.remove_signal_watch()
        if self.pipeline is not None:
            self.pipeline.set_state(self.runtime.Gst.State.NULL)


def display(sources, runtime, *, port=8080, open_browser=True):
    loop = runtime.GLib.MainLoop()
    previews = []
    callbacks = []
    server = None
    try:
        for source in sources:
            preview = CameraPreview(source, runtime, loop)
            previews.append(preview)
            try:
                preview.start()
            except Exception as exc:
                preview.fail(str(exc))
        if all(preview.failed for preview in previews):
            return 1

        server = WebcamServer(previews, lambda: runtime.GLib.idle_add(loop.quit), port=port)
        print(f"Webcam web viewer: {server.url}", flush=True)
        if open_browser:
            try:
                webbrowser.open(server.url)
            except Exception as exc:
                print(f"Could not open a browser: {exc}. Open {server.url} manually.", file=sys.stderr)

        def check_cameras():
            for preview in previews:
                preview.check_timeout()
            if all(preview.failed for preview in previews):
                loop.quit()
            return True

        def interrupt():
            loop.quit()
            return True

        callbacks.append(runtime.GLib.timeout_add(100, check_cameras))
        callbacks.append(runtime.GLib.unix_signal_add(runtime.GLib.PRIORITY_DEFAULT,
                                                     signal.SIGINT, interrupt))
        callbacks.append(runtime.GLib.unix_signal_add(runtime.GLib.PRIORITY_DEFAULT,
                                                     signal.SIGTERM, interrupt))
        print("Use Close viewer, Q or Esc in the web page, or Ctrl-C in the terminal to exit.", flush=True)
        loop.run()
        return 1 if all(preview.failed for preview in previews) else 0
    finally:
        for callback in callbacks:
            runtime.GLib.source_remove(callback)
        for preview in previews:
            preview.close()
        if server is not None:
            server.close()


def main():
    args = parse_args()
    try:
        runtime = load_runtime()
        runtime.Gst.init(None)
        sources = args.cameras if args.cameras is not None else discover_cameras(runtime.Gst)
        if not sources:
            raise RuntimeError("No video capture devices found; specify devices with --cameras.")
        return display(sources, runtime, port=args.port, open_browser=not args.no_browser)
    except KeyboardInterrupt:
        return 130
    except Exception as exc:
        print(f"Webcam preview error: {exc}", file=sys.stderr, flush=True)
        return 1


if __name__ == "__main__":
    sys.exit(main())
