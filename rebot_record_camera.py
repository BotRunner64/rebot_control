#!/usr/bin/env python3
"""Isolated GStreamer H.264 recorder; controlled with newline-delimited stdin JSON."""

import argparse
from contextlib import redirect_stdout
import json
import os
from pathlib import Path
import select
import signal
import struct
import sys
import time

from webcam_viewer import choose_format, discover_cameras


def report(kind, **values):
    print(json.dumps({"type": kind, **values}), flush=True)


def recording_size(width, height):
    """Fit within 960 pixels without cropping; align I420 dimensions to even values."""
    scale = min(1., 960 / max(width, height))
    return max(2, int(width * scale) // 2 * 2), max(2, int(height * scale) // 2 * 2)


def monotonic_offset(clock):
    """Bridge the selected GStreamer clock to Python's process-shared monotonic clock."""
    candidates = []
    for _ in range(8):
        before = time.monotonic_ns()
        gst_now = clock.get_time()
        after = time.monotonic_ns()
        candidates.append((after - before, (before + after) // 2 - gst_now))
    return min(candidates)[1]


def decode_frame_count(Gst, path, *, timeout=20.):
    pipeline = Gst.parse_launch(
        "filesrc name=input ! qtdemux ! avdec_h264 "
        "! fakesink name=frames sync=false signal-handoffs=true"
    )
    pipeline.get_by_name("input").set_property("location", str(path))
    count = [0]

    def received(sink, buffer, pad):
        count[0] += 1

    pipeline.get_by_name("frames").connect("handoff", received)
    try:
        if pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError("Could not decode the recorded H.264 video.")
        deadline, previous = time.monotonic() + timeout, 0
        while True:
            message = pipeline.get_bus().timed_pop_filtered(
                100 * Gst.MSECOND, Gst.MessageType.ERROR | Gst.MessageType.EOS)
            if message is not None:
                if message.type == Gst.MessageType.ERROR:
                    raise RuntimeError(f"Recorded video decoding failed: {message.parse_error()[0]}")
                return count[0]
            if count[0] != previous:
                previous, deadline = count[0], time.monotonic() + timeout
            if time.monotonic() > deadline:
                raise TimeoutError("Recorded video verification made no progress.")
    finally:
        pipeline.set_state(Gst.State.NULL)


class CameraRecorder:
    def __init__(self, Gst, source, output, timestamps, *, test_source=False, preview_socket=None):
        self.Gst, self.output = Gst, Path(output)
        self.pipeline = None
        self.timestamps = None
        self.preview = None
        self.preview_caps = None
        self.start_ns = self.stop_ns = self.previous_ns = None
        self.ready = False
        self.error = None
        self.count = 0
        self.last_frame_at = time.monotonic()
        try:
            if preview_socket is not None:
                try:
                    from rebot_record_preview import PreviewPublisher
                    self.preview = PreviewPublisher(preview_socket)
                except Exception as exc:
                    print(f"Camera preview disabled: {exc}", file=sys.stderr)
            self.timestamps = Path(timestamps).open("xb")
            source_description = "videotestsrc is-live=true name=camera" if test_source else "v4l2src name=camera"
            self.pipeline = Gst.parse_launch(
                source_description + " ! capsfilter name=mode ! decodebin ! videoconvert ! videoscale "
                "! capsfilter name=record_size ! identity name=frames "
                "! x264enc tune=zerolatency speed-preset=ultrafast bframes=0 pass=qual quantizer=23 bitrate=50000 "
                "! video/x-h264,stream-format=avc,alignment=au ! mp4mux ! filesink name=output sync=false"
            )
            clock = Gst.SystemClock.obtain()
            self.pipeline.use_clock(clock)
            self.clock_offset = monotonic_offset(clock)
            camera = self.pipeline.get_by_name("camera")
            if not test_source:
                camera.set_property("device", source)
            self.pipeline.get_by_name("output").set_property("location", str(output))
            if self.pipeline.set_state(Gst.State.READY) == Gst.StateChangeReturn.FAILURE:
                raise RuntimeError(f"Could not open camera {source}.")
            selected = (Gst.Caps.from_string("video/x-raw,width=160,height=120,framerate=20/1")
                        if test_source else choose_format(camera.get_static_pad("src").query_caps(None)))
            self.pipeline.get_by_name("mode").set_property("caps", selected)
            info = selected.get_structure(0)
            width, height = recording_size(info.get_int("width")[1], info.get_int("height")[1])
            self.pipeline.get_by_name("record_size").set_property("caps", Gst.Caps.from_string(
                f"video/x-raw,format=I420,width={width},height={height},pixel-aspect-ratio=1/1"))
            print(f"Camera {source}: {selected.to_string()}", file=sys.stderr, flush=True)
            print(f"Recording {width}x{height}, H.264 CRF 23.", file=sys.stderr, flush=True)
            self.pipeline.get_by_name("frames").get_static_pad("src").add_probe(
                Gst.PadProbeType.BUFFER, self.on_frame)
            if self.pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
                raise RuntimeError(f"Could not start camera {source}.")
        except BaseException:
            self.close()
            raise

    def on_frame(self, pad, info):
        Gst = self.Gst
        try:
            buffer = info.get_buffer()
            event = pad.get_sticky_event(Gst.EventType.SEGMENT, 0)
            if buffer.pts == Gst.CLOCK_TIME_NONE or event is None:
                raise ValueError("Camera frame has no native timestamp or TIME segment.")
            running_ns = event.parse_segment().to_running_time(Gst.Format.TIME, buffer.pts)
            if running_ns == Gst.CLOCK_TIME_NONE:
                raise ValueError("Camera frame timestamp is outside its TIME segment.")
            timestamp = int(self.pipeline.get_base_time() + running_ns + self.clock_offset)
            self.last_frame_at = time.monotonic()
            if self.preview is not None:
                # Hand off only a buffer reference. Copying and socket I/O stay off this thread.
                try:
                    if self.preview_caps is None:
                        self.preview_caps = pad.get_current_caps().to_string()
                    self.preview.offer(buffer, self.preview_caps, self.last_frame_at)
                except Exception:
                    self.preview.stopping.set()
            if not self.ready:
                self.ready = True
                report("ready")
            if self.start_ns is None or timestamp < self.start_ns:
                return Gst.PadProbeReturn.DROP
            if self.stop_ns is not None and timestamp > self.stop_ns:
                return Gst.PadProbeReturn.DROP
            if self.previous_ns is not None and timestamp < self.previous_ns:
                raise ValueError("Camera timestamps moved backwards.")
            self.timestamps.write(struct.pack("<q", timestamp))
            self.previous_ns = timestamp
            self.count += 1
            return Gst.PadProbeReturn.OK
        except BaseException as exc:
            self.error = exc
            return Gst.PadProbeReturn.DROP

    def run(self):
        Gst = self.Gst
        bus = self.pipeline.get_bus()
        finishing = False
        commands = bytearray()
        deadline = None
        stopping = [False]
        old = {signal.SIGTERM: signal.signal(signal.SIGTERM, lambda *_: stopping.__setitem__(0, True))}
        for sig in (signal.SIGINT, signal.SIGHUP):
            old[sig] = signal.signal(sig, signal.SIG_IGN)
        try:
            while True:
                if self.error is not None:
                    raise self.error
                if stopping[0]:
                    raise RuntimeError("Camera recorder was interrupted.")
                if not finishing and select.select([sys.stdin], [], [], 0)[0]:
                    data = os.read(sys.stdin.fileno(), 65536)
                    if not data:
                        raise RuntimeError("Recording supervisor disconnected.")
                    commands.extend(data)
                # Parse every complete command already read from the pipe. TextIO
                # read-ahead must not hide a stop command from the next select().
                while not finishing and b"\n" in commands:
                    line, _, rest = commands.partition(b"\n")
                    commands = bytearray(rest)
                    command = json.loads(line)
                    if command["command"] == "start" and self.start_ns is None:
                        self.start_ns = command["timestamp_ns"]
                    elif command["command"] == "stop":
                        self.stop_ns = command["timestamp_ns"]
                        finishing = True
                        deadline = time.monotonic() + 15.
                        if not self.pipeline.send_event(Gst.Event.new_eos()):
                            raise RuntimeError("Camera pipeline refused EOS.")
                    else:
                        raise ValueError("Invalid camera recording command.")
                message = bus.timed_pop_filtered(20 * Gst.MSECOND,
                    Gst.MessageType.ERROR | Gst.MessageType.EOS)
                if message is not None:
                    if message.type == Gst.MessageType.ERROR:
                        raise RuntimeError(str(message.parse_error()[0]))
                    if not finishing:
                        raise RuntimeError("Camera stream ended unexpectedly.")
                    break
                if finishing and time.monotonic() > deadline:
                    raise TimeoutError("Camera video finalization timed out.")
                if not finishing and time.monotonic() - self.last_frame_at > 10.:
                    raise TimeoutError("Camera produced no frame for 10 seconds.")
            self.pipeline.set_state(Gst.State.NULL)
            self.timestamps.flush()
            if self.count == 0:
                raise ValueError("Camera recorded no frames.")
            decoded = decode_frame_count(Gst, self.output)
            if decoded != self.count:
                raise ValueError(f"Video/index frame count mismatch: {decoded} != {self.count}.")
            report("done", frames=self.count)
        finally:
            for sig, handler in old.items():
                signal.signal(sig, handler)

    def close(self):
        if self.pipeline is not None:
            self.pipeline.set_state(self.Gst.State.NULL)
        if self.timestamps is not None:
            self.timestamps.close()
        if self.preview is not None:
            self.preview.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--discover", action="store_true", help="List capture devices without recording")
    parser.add_argument("--source")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--timestamps", type=Path)
    parser.add_argument("--test-source", action="store_true", help="Use a synthetic camera for offline tests")
    parser.add_argument("--preview-socket", type=Path, help="Best-effort viewer socket")
    args = parser.parse_args()
    if not args.discover and any(value is None for value in (args.source, args.output, args.timestamps)):
        parser.error("recording requires --source, --output, and --timestamps")
    recorder = None
    try:
        import gi
        gi.require_version("Gst", "1.0")
        from gi.repository import Gst
        Gst.init(None)
        if args.discover:
            with redirect_stdout(sys.stderr):
                sources = discover_cameras(Gst)
            report("cameras", sources=sources)
            return 0
        recorder = CameraRecorder(Gst, args.source, args.output, args.timestamps, test_source=args.test_source,
                                  preview_socket=args.preview_socket)
        recorder.run()
        return 0
    except Exception as exc:
        report("error", error=f"{type(exc).__name__}: {exc}")
        return 2
    finally:
        if recorder is not None:
            recorder.close()


if __name__ == "__main__":
    raise SystemExit(main())
