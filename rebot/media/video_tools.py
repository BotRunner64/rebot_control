"""Generate a low-resolution review proxy with the system GStreamer runtime."""

import json
import os
from pathlib import Path
import sys

from rebot.media.capture import fit_size


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
    width, height = fit_size(width, height, 640, 640)
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


def main():
    try:
        request = json.load(sys.stdin)
        make_preview(request)
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
