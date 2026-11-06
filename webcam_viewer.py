#!/usr/bin/env python3
"""Preview Linux webcams using GStreamer and automatic capture mode selection."""

import argparse
import json
import os
from pathlib import Path
import signal
import sys
import time
from types import SimpleNamespace

FIRST_FRAME_TIMEOUT = 10.0


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cameras", nargs="+",
                        help="Camera indices or device paths (default: auto-detect).")
    args = parser.parse_args()
    if args.cameras is not None:
        args.cameras = [f"/dev/video{source}" if source.isdecimal() else source
                        for source in args.cameras]
        if len(set(args.cameras)) != len(args.cameras):
            parser.error("camera sources must be unique")
    return args


def load_runtime():
    """Use the desktop's GStreamer bindings, including from a Conda shell."""
    try:
        import gi
        gi.require_version("Gst", "1.0")
        from gi.repository import GLib, Gst
        return SimpleNamespace(GLib=GLib, Gst=Gst)
    except (ImportError, ValueError) as exc:
        system_python = Path("/usr/bin/python3")
        if system_python.exists() and system_python.resolve() != Path(sys.executable).resolve():
            print("Using system Python for desktop GStreamer bindings.", flush=True)
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
    """Let GStreamer capture, decode and display the selected camera mode."""

    def __init__(self, source, runtime, loop):
        self.source = source
        self.runtime = runtime
        self.loop = loop
        self.pipeline = None
        self.bus = None
        self.first_frame = False
        self.failed = False
        self.started_at = 0.0

    def start(self):
        Gst = self.runtime.Gst
        print(f"Camera {self.source}: querying supported capture modes...", flush=True)
        self.pipeline = Gst.parse_launch(
            "v4l2src name=camera ! capsfilter name=mode ! decodebin "
            "! videoconvert ! videoscale ! capsfilter name=preview_size "
            "! taginject name=label ! autovideosink name=preview sync=false"
        )
        camera = self.pipeline.get_by_name("camera")
        camera.set_property("device", self.source)
        if self.pipeline.set_state(Gst.State.READY) == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError("Could not open camera or desktop video output.")
        caps = camera.get_static_pad("src").query_caps(None)
        selected = choose_format(caps)
        self.pipeline.get_by_name("mode").set_property("caps", selected)
        info = selected.get_structure(0)
        _, width = info.get_int("width")
        _, height = info.get_int("height")
        _, numerator, denominator = info.get_fraction("framerate")
        label = f"{width}x{height}, {numerator / denominator:g} FPS"
        print(f"Camera {self.source}: selected {label}, {info.get_name()}.", flush=True)

        # Scale only the displayed image; capture retains the full selected frame.
        scale = min(640 / width, 480 / height, 1)
        preview_width = max(2, int(width * scale) // 2 * 2)
        preview_height = max(2, int(height * scale) // 2 * 2)
        self.pipeline.get_by_name("preview_size").set_property(
            "caps", Gst.Caps.from_string(f"video/x-raw,width={preview_width},"
                                         f"height={preview_height},pixel-aspect-ratio=1/1")
        )
        title = json.dumps(f"Camera {self.source} - {label}")
        self.pipeline.get_by_name("label").set_property("tags", f"title={title}")
        sink_pad = self.pipeline.get_by_name("preview").get_static_pad("sink")
        sink_pad.add_probe(Gst.PadProbeType.BUFFER, self.on_first_frame)
        sink_pad.add_probe(Gst.PadProbeType.EVENT_UPSTREAM, self.on_navigation)
        self.bus = self.pipeline.get_bus()
        self.bus.add_signal_watch()
        self.bus.connect("message::error", self.on_error)
        self.bus.connect("message::eos", lambda *args: self.loop.quit())
        self.started_at = time.monotonic()
        if self.pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError("Could not start camera stream.")

    def on_first_frame(self, pad, info):
        self.first_frame = True
        print(f"Camera {self.source}: first frame received.", flush=True)
        return self.runtime.Gst.PadProbeReturn.REMOVE

    def on_navigation(self, pad, info):
        event = info.get_event()
        if event.type == self.runtime.Gst.EventType.NAVIGATION:
            details = event.get_structure()
            if details.get_string("event") == "key-press":
                key = details.get_string("key") or ""
                if key.lower() in ("q", "escape"):
                    self.loop.quit()
        return self.runtime.Gst.PadProbeReturn.OK

    def on_error(self, bus, message):
        error, _ = message.parse_error()
        if "window was closed" in error.message.lower():
            self.loop.quit()
        else:
            self.fail(error.message)

    def fail(self, message):
        if self.failed:
            return
        self.failed = True
        print(f"Camera {self.source}: {message}", file=sys.stderr, flush=True)
        if self.pipeline is not None:
            self.pipeline.set_state(self.runtime.Gst.State.NULL)

    def check_timeout(self):
        if not self.failed and not self.first_frame and time.monotonic() - self.started_at >= FIRST_FRAME_TIMEOUT:
            self.fail("No first frame within 10 seconds.")

    def close(self):
        if self.bus is not None:
            self.bus.remove_signal_watch()
        if self.pipeline is not None:
            self.pipeline.set_state(self.runtime.Gst.State.NULL)


def display(sources, runtime):
    loop = runtime.GLib.MainLoop()
    previews = []
    callbacks = []
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
        print("Press Q or Esc, close a camera window, or press Ctrl-C to exit.", flush=True)
        loop.run()
        return 1 if all(preview.failed for preview in previews) else 0
    finally:
        for callback in callbacks:
            runtime.GLib.source_remove(callback)
        for preview in previews:
            preview.close()


def main():
    args = parse_args()
    try:
        runtime = load_runtime()
        runtime.Gst.init(None)
        sources = args.cameras if args.cameras is not None else discover_cameras(runtime.Gst)
        if not sources:
            raise RuntimeError("No video capture devices found; specify devices with --cameras.")
        return display(sources, runtime)
    except KeyboardInterrupt:
        return 130
    except Exception as exc:
        print(f"Webcam preview error: {exc}", file=sys.stderr, flush=True)
        return 1


if __name__ == "__main__":
    sys.exit(main())
