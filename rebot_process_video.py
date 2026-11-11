#!/usr/bin/env python3
"""Frame-accurate offline H.264 slicing in the system GStreamer environment."""

import json
import os
from pathlib import Path
import sys


def cut_video(request):
    import gi
    gi.require_version("Gst", "1.0")
    from gi.repository import Gst
    from rebot_record_camera import decode_frame_count

    Gst.init(None)
    decoder = Gst.parse_launch(
        "filesrc name=input ! qtdemux ! avdec_h264 ! videoconvert "
        "! video/x-raw,format=I420 ! appsink name=frames sync=false max-buffers=2 drop=false")
    encoder = Gst.parse_launch(
        "appsrc name=frames format=time block=true max-bytes=8388608 ! x264enc tune=zerolatency speed-preset=ultrafast "
        "bframes=0 pass=qual quantizer=23 bitrate=50000 ! video/x-h264,stream-format=avc,alignment=au "
        "! mp4mux faststart=true ! filesink name=output sync=false")
    decoder.get_by_name("input").set_property("location", request["source"])
    encoder.get_by_name("output").set_property("location", request["output"])
    sink, source = decoder.get_by_name("frames"), encoder.get_by_name("frames")
    times = request["timestamps_ns"]
    start, count = request["first_frame"], 0
    try:
        decoder.set_state(Gst.State.PLAYING)
        for index in range(start + len(times)):
            sample = sink.emit("try-pull-sample", 30 * Gst.SECOND)
            if sample is None:
                message = decoder.get_bus().pop_filtered(Gst.MessageType.ERROR)
                detail = str(message.parse_error()[0]) if message else "Unexpected end or decoder timeout"
                raise RuntimeError(detail)
            if index < start:
                continue
            if count == 0:
                source.set_property("caps", sample.get_caps())
                encoder.set_state(Gst.State.PLAYING)
            buffer = sample.get_buffer().copy_deep()
            buffer.pts = times[count] - times[0]
            buffer.dts = Gst.CLOCK_TIME_NONE
            buffer.duration = (times[count + 1] - times[count] if count + 1 < len(times)
                               else max(1, times[-1] - times[-2]) if len(times) > 1 else Gst.SECOND // 30)
            if source.emit("push-buffer", buffer) != Gst.FlowReturn.OK:
                raise RuntimeError("Video encoder rejected a selected frame.")
            count += 1
        source.emit("end-of-stream")
        message = encoder.get_bus().timed_pop_filtered(
            300 * Gst.SECOND, Gst.MessageType.EOS | Gst.MessageType.ERROR)
        if message is None or message.type == Gst.MessageType.ERROR:
            raise RuntimeError(str(message.parse_error()[0]) if message else "Video encoding timed out")
    finally:
        decoder.set_state(Gst.State.NULL)
        encoder.set_state(Gst.State.NULL)
    decoded = decode_frame_count(Gst, request["output"], timeout=60.)
    if decoded != len(times):
        raise ValueError(f"Exported video/index mismatch: {decoded} != {len(times)}")


def make_preview(request):
    """Create a small, constant-rate proxy while preserving the source media clock."""
    import gi
    gi.require_version("Gst", "1.0")
    from gi.repository import Gst

    os.nice(10)
    Gst.init(None)
    probe = Gst.parse_launch(
        "filesrc name=input ! qtdemux ! avdec_h264 max-threads=2 ! fakesink name=frames sync=false")
    probe.get_by_name("input").set_property("location", request["source"])
    try:
        probe.set_state(Gst.State.PAUSED)
        result, _, _ = probe.get_state(15 * Gst.SECOND)
        if result != Gst.StateChangeReturn.SUCCESS:
            raise RuntimeError("Could not read source video dimensions.")
        info = probe.get_by_name("frames").get_static_pad("sink").get_current_caps().get_structure(0)
        width, height = info.get_int("width")[1], info.get_int("height")[1]
    finally:
        probe.set_state(Gst.State.NULL)
    scale = min(1., 640 / max(width, height))
    width, height = max(2, int(width * scale) // 2 * 2), max(2, int(height * scale) // 2 * 2)
    pipeline = Gst.parse_launch(
        "filesrc name=input ! qtdemux ! avdec_h264 max-threads=2 ! videoconvert n-threads=1 "
        "! videoscale n-threads=1 ! videorate "
        f"! video/x-raw,format=I420,width={width},height={height},pixel-aspect-ratio=1/1,framerate=15/1 "
        "! x264enc threads=2 tune=zerolatency speed-preset=veryfast key-int-max=15 "
        "bframes=0 bitrate=800 vbv-buf-capacity=500 "
        "! video/x-h264,profile=baseline,stream-format=avc,alignment=au "
        "! mp4mux faststart=true ! filesink name=output sync=false")
    pipeline.get_by_name("input").set_property("location", request["source"])
    pipeline.get_by_name("output").set_property("location", request["output"])
    try:
        if pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError("Could not start preview conversion.")
        message = pipeline.get_bus().timed_pop_filtered(
            1800 * Gst.SECOND, Gst.MessageType.EOS | Gst.MessageType.ERROR)
        if message is None or message.type == Gst.MessageType.ERROR:
            raise RuntimeError(str(message.parse_error()[0]) if message else "Preview conversion timed out.")
    finally:
        pipeline.set_state(Gst.State.NULL)
    if not Path(request["output"]).is_file() or Path(request["output"]).stat().st_size == 0:
        raise RuntimeError("Preview conversion produced no video.")


if __name__ == "__main__":
    try:
        request = json.load(sys.stdin)
        if request.get("preview"):
            make_preview(request)
        else:
            cut_video(request)
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        raise SystemExit(2)
