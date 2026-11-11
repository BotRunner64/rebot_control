#!/usr/bin/env python3
"""Frame-accurate offline H.264 slicing in the system GStreamer environment."""

import json
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


if __name__ == "__main__":
    try:
        cut_video(json.load(sys.stdin))
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        raise SystemExit(2)
