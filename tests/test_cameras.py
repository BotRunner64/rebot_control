"""Capture selection and cleanup checks using the real GStreamer type system."""

import contextlib
import io
import json
import os
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch
from urllib.error import HTTPError
from urllib.request import Request, urlopen

from rebot.media import capture
from rebot.media.capture import CameraPreview, choose_format, discover_cameras, parse_args
from rebot.media.mjpeg import WebcamServer
from rebot.paths import ROOT

try:
    import gi
    gi.require_version("Gst", "1.0")
    from gi.repository import Gst
    Gst.init(None)
except (ImportError, ValueError):
    Gst = None


@unittest.skipIf(Gst is None, "Run with /usr/bin/python3 for GStreamer bindings")
class FormatSelectionTests(unittest.TestCase):
    def select(self, description):
        selected = choose_format(Gst.Caps.from_string(description))
        self.assertTrue(selected.is_fixed())
        info = selected.get_structure(0)
        return (info.get_int("width")[1], info.get_int("height")[1],
                info.get_fraction("framerate")[1:])

    def test_mode_selection(self):
        for case, description, expected in (
                ("largest usable mode wins over a higher frame rate",
                 "image/jpeg,width=1280,height=720,framerate=60/1;"
                 "image/jpeg,width=2592,height=1944,framerate=23/1;"
                 "image/jpeg,width=3840,height=2160,framerate=10/1", (2592, 1944, (23, 1))),
                ("highest resolution is the fallback when no mode meets the thresholds",
                 "image/jpeg,width=320,height=240,framerate=60/1;"
                 "image/jpeg,width=1600,height=1200,framerate=10/1", (1600, 1200, (10, 1))),
                ("thresholds are inclusive",
                 "image/jpeg,width=320,height=240,framerate=60/1;"
                 "image/jpeg,width=640,height=480,framerate=15/1", (640, 480, (15, 1))),
                ("same resolution uses the highest frame rate",
                 "image/jpeg,width=1280,height=720,framerate=15/1;"
                 "image/jpeg,width=1280,height=720,framerate=30/1", (1280, 720, (30, 1))),
                ("ranges are supported",
                 "video/x-raw,format=YUY2,width=[320,1280],height=[240,720],framerate=[15/1,30/1]",
                 (1280, 720, (30, 1))),
                ("lists are supported",
                 "image/jpeg,width=1280,height=720,framerate={15/1,30/1}", (1280, 720, (30, 1)))):
            with self.subTest(case=case):
                self.assertEqual(self.select(description), expected)

    def test_empty_or_unsupported_caps_report_error(self):
        for description in ("EMPTY", "video/x-h264,width=1920,height=1080,framerate=30/1"):
            with self.subTest(description=description), self.assertRaisesRegex(RuntimeError, "no supported"):
                choose_format(Gst.Caps.from_string(description))


class ArgumentTests(unittest.TestCase):
    def test_indices_are_normalized_and_duplicate_paths_are_rejected(self):
        self.assertEqual(parse_args(["--cameras", "0", "/dev/video2"]).cameras, ["/dev/video0", "/dev/video2"])
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            parse_args(["--cameras", "0", "/dev/video0"])

    def test_missing_bindings_reexecute_the_module_under_the_system_interpreter(self):
        with patch.dict(sys.modules, {"gi": None}), patch.object(capture.sys, "executable", "/opt/env/bin/python"), \
                patch.object(capture.os, "execve", side_effect=SystemExit) as execve, \
                contextlib.redirect_stdout(io.StringIO()), self.assertRaises(SystemExit):
            capture.main(["--port", "8090", "--no-browser"])
        program, command, env = execve.call_args.args
        self.assertEqual(program, "/usr/bin/python3")
        self.assertEqual(command[-5:], ["-m", "rebot.media.capture", "--port", "8090", "--no-browser"])
        self.assertIn(str(ROOT), env["PYTHONPATH"].split(os.pathsep))


class LifecycleTests(unittest.TestCase):
    def test_discovery_releases_monitor_and_deduplicates_devices(self):
        gst = MagicMock()
        monitor = gst.DeviceMonitor.new.return_value
        properties = [SimpleNamespace(get_string=lambda key, path=path, field=field:
                                      path if key == field else None)
                      for path, field in (("/dev/video2", "device.path"),
                                          ("/dev/video0", "api.v4l2.path"),
                                          ("/dev/video2", "api.v4l2.path"),
                                          ("/dev/media0", "device.path"))]
        monitor.get_devices.return_value = [SimpleNamespace(get_properties=lambda props=props: props)
                                            for props in properties]
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(discover_cameras(gst), ["/dev/video0", "/dev/video2"])
        monitor.stop.assert_called_once()

    def test_timeout_reports_failure(self):
        for received_first_frame in (False, True):
            with self.subTest(received_first_frame=received_first_frame):
                preview = CameraPreview("/dev/video0", MagicMock(), MagicMock())
                preview.pipeline = MagicMock()
                preview.started_at = preview.last_frame_at = 1.0
                preview.first_frame = received_first_frame
                with patch("rebot.media.capture.time.monotonic", return_value=12), \
                        contextlib.redirect_stderr(io.StringIO()):
                    preview.check_timeout()
                self.assertTrue(preview.failed)

    def test_latest_frame_is_bounded_and_close_wakes_waiting_clients(self):
        preview = CameraPreview("/dev/video0", MagicMock(), MagicMock())
        sink = MagicMock()
        buffer = sink.emit.return_value.get_buffer.return_value
        buffer.extract_dup.side_effect = [b"first", b"latest"]
        with contextlib.redirect_stdout(io.StringIO()):
            preview.on_sample(sink)
            preview.on_sample(sink)
        self.assertEqual(preview.wait_frame(0), (2, b"latest"))
        self.assertIsNone(preview.wait_frame(2, timeout=.001))
        waiting = threading.Thread(target=lambda: preview.wait_frame(2, timeout=10.))
        waiting.start()
        preview.close()
        waiting.join(.5)
        self.assertFalse(waiting.is_alive())
        self.assertEqual(preview.snapshot()["state"], "stopped")


class WebTransportTests(unittest.TestCase):
    def setUp(self):
        self.camera = CameraPreview("/dev/video0", MagicMock(), MagicMock())
        self.stop = MagicMock()
        self.server = WebcamServer([self.camera], self.stop, port=0)
        self.addCleanup(self.server.close)
        self.addCleanup(self.camera.close)

    def test_browser_stop_requires_matching_origin_and_valid_json(self):
        for origin, body, expected in (("https://example.com", b"{}", 403),
                                      (self.server.url, b"[]", 400)):
            request = Request(self.server.url + "/api/stop", data=body,
                              headers={"Origin": origin, "Content-Type": "application/json"})
            with self.assertRaises(HTTPError) as caught:
                urlopen(request, timeout=2.)
            self.assertEqual(caught.exception.code, expected)
        self.stop.assert_not_called()
        request = Request(self.server.url + "/api/stop", data=b"{}",
                          headers={"Origin": self.server.url, "Content-Type": "application/json"})
        with urlopen(request, timeout=2.) as response:
            self.assertEqual(json.load(response), {"stopped": True})
        self.stop.assert_called_once()

    def test_occupied_port_uses_next_available_port(self):
        second = WebcamServer([self.camera], self.stop, port=self.server.port)
        try:
            self.assertGreater(second.port, self.server.port)
        finally:
            second.close()


@unittest.skipIf(Gst is None, "Run with /usr/bin/python3 for GStreamer bindings")
class GStreamerWebTests(unittest.TestCase):
    def test_real_jpeg_capture_is_readable_through_http(self):
        camera = CameraPreview("synthetic", SimpleNamespace(Gst=Gst), MagicMock())
        pipeline = Gst.parse_launch(
            "videotestsrc is-live=true ! video/x-raw,width=160,height=120,framerate=20/1 "
            "! jpegenc ! appsink name=preview emit-signals=true max-buffers=1 drop=true sync=false")
        camera.pipeline = pipeline
        pipeline.get_by_name("preview").connect("new-sample", camera.on_sample)
        server = WebcamServer([camera], MagicMock(), port=0)
        try:
            pipeline.set_state(Gst.State.PLAYING)
            frame = camera.wait_frame(0, timeout=3.)
            self.assertIsNotNone(frame)
            self.assertTrue(frame[1].startswith(b"\xff\xd8"))
            self.assertTrue(frame[1].endswith(b"\xff\xd9"))
            with urlopen(server.url + "/streams/0.mjpg", timeout=2.) as response:
                response.readline()
                response.readline()
                length = int(response.readline().split(b":")[1])
                response.readline()
                jpeg = response.read(length)
                self.assertTrue(jpeg.startswith(b"\xff\xd8"))
                self.assertTrue(jpeg.endswith(b"\xff\xd9"))
        finally:
            camera.close()
            server.close()


if __name__ == "__main__":
    unittest.main()
