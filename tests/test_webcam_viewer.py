"""Capture selection and cleanup checks using the real GStreamer type system."""

import contextlib
import io
import json
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch
from urllib.error import HTTPError
from urllib.request import Request, urlopen

from webcam_viewer import CameraPreview, choose_format, discover_cameras, display, parse_args
from webcam_web import WebcamServer

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

    def test_largest_usable_mode_wins_over_higher_frame_rate(self):
        self.assertEqual(self.select(
            "image/jpeg,width=1280,height=720,framerate=60/1;"
            "image/jpeg,width=2592,height=1944,framerate=23/1;"
            "image/jpeg,width=3840,height=2160,framerate=10/1"
        ), (2592, 1944, (23, 1)))

    def test_highest_resolution_is_fallback_when_no_mode_meets_thresholds(self):
        self.assertEqual(self.select(
            "image/jpeg,width=320,height=240,framerate=60/1;"
            "image/jpeg,width=1600,height=1200,framerate=10/1"
        ), (1600, 1200, (10, 1)))

    def test_thresholds_are_inclusive(self):
        self.assertEqual(self.select(
            "image/jpeg,width=320,height=240,framerate=60/1;"
            "image/jpeg,width=640,height=480,framerate=15/1"
        ), (640, 480, (15, 1)))

    def test_same_resolution_uses_highest_frame_rate(self):
        self.assertEqual(self.select(
            "image/jpeg,width=1280,height=720,framerate=15/1;"
            "image/jpeg,width=1280,height=720,framerate=30/1"
        ), (1280, 720, (30, 1)))

    def test_lists_and_ranges_are_supported(self):
        self.assertEqual(self.select(
            "video/x-raw,format=YUY2,width=[320,1280],height=[240,720],framerate=[15/1,30/1]"
        ), (1280, 720, (30, 1)))
        self.assertEqual(self.select(
            "image/jpeg,width=1280,height=720,framerate={15/1,30/1}"
        ), (1280, 720, (30, 1)))

    def test_empty_or_unsupported_caps_report_error(self):
        for description in ("EMPTY", "video/x-h264,width=1920,height=1080,framerate=30/1"):
            with self.subTest(description=description), self.assertRaisesRegex(RuntimeError, "no supported"):
                choose_format(Gst.Caps.from_string(description))


class ArgumentTests(unittest.TestCase):
    def test_defaults_defer_to_device_discovery(self):
        with patch.object(sys, "argv", ["webcam_viewer.py"]):
            self.assertEqual(vars(parse_args()), {"cameras": None, "port": 8080, "no_browser": False})

    def test_indices_are_normalized_and_duplicate_paths_are_rejected(self):
        with patch.object(sys, "argv", ["webcam_viewer.py", "--cameras", "0", "/dev/video2"]):
            self.assertEqual(parse_args().cameras, ["/dev/video0", "/dev/video2"])
        with patch.object(sys, "argv", ["webcam_viewer.py", "--cameras", "0", "/dev/video0"]), \
                contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            parse_args()

    def test_web_options_and_invalid_ports(self):
        with patch.object(sys, "argv", ["webcam_viewer.py", "--port", "8090", "--no-browser"]):
            args = parse_args()
            self.assertEqual(args.port, 8090)
            self.assertTrue(args.no_browser)
        for port in ("0", "-1", "65536"):
            with patch.object(sys, "argv", ["webcam_viewer.py", "--port", port]), \
                    contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                parse_args()


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

    def test_partial_setup_failure_still_releases_pipeline(self):
        runtime = MagicMock()
        preview = CameraPreview("/dev/video0", runtime, MagicMock())
        preview.pipeline = MagicMock()
        with contextlib.redirect_stderr(io.StringIO()):
            preview.fail("setup failed")
        preview.close()
        self.assertTrue(preview.failed)
        preview.pipeline.set_state.assert_called_with(runtime.Gst.State.NULL)

    def test_error_before_pipeline_creation_is_safe(self):
        preview = CameraPreview("/dev/video0", MagicMock(), MagicMock())
        with contextlib.redirect_stderr(io.StringIO()):
            preview.fail("missing plugin")
        preview.close()
        self.assertTrue(preview.failed)

    def test_timeout_reports_failure(self):
        for received_first_frame in (False, True):
            with self.subTest(received_first_frame=received_first_frame):
                preview = CameraPreview("/dev/video0", MagicMock(), MagicMock())
                preview.pipeline = MagicMock()
                preview.started_at = preview.last_frame_at = 1.0
                preview.first_frame = received_first_frame
                with patch("webcam_viewer.time.monotonic", return_value=12), \
                        contextlib.redirect_stderr(io.StringIO()):
                    preview.check_timeout()
                self.assertTrue(preview.failed)

    def test_pipeline_error_is_reported_to_the_web_status(self):
        preview = CameraPreview("/dev/video0", MagicMock(), MagicMock())
        message = SimpleNamespace(parse_error=lambda: (SimpleNamespace(message="Camera disconnected"), None))
        with contextlib.redirect_stderr(io.StringIO()):
            preview.on_error(None, message)
        self.assertEqual(preview.snapshot()["state"], "error")
        self.assertEqual(preview.snapshot()["error"], "Camera disconnected")

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

    def test_main_loop_error_releases_all_previews_and_signal_callbacks(self):
        runtime = MagicMock()
        runtime.GLib.MainLoop.return_value.run.side_effect = KeyboardInterrupt
        with patch("webcam_viewer.CameraPreview") as preview_type, \
                patch("webcam_viewer.WebcamServer") as server_type, \
                patch("webcam_viewer.webbrowser.open"), \
                contextlib.redirect_stdout(io.StringIO()):
            previews = [MagicMock(failed=False), MagicMock(failed=False)]
            preview_type.side_effect = previews
            with self.assertRaises(KeyboardInterrupt):
                display(["/dev/video0", "/dev/video2"], runtime)
        for preview in previews:
            preview.close.assert_called_once()
        self.assertEqual(runtime.GLib.source_remove.call_count, 3)
        server_type.return_value.close.assert_called_once()


class WebTransportTests(unittest.TestCase):
    def setUp(self):
        self.camera = CameraPreview("/dev/video0", MagicMock(), MagicMock())
        self.stop = MagicMock()
        self.server = WebcamServer([self.camera], self.stop, port=0)
        self.addCleanup(self.server.close)
        self.addCleanup(self.camera.close)

    def get(self, path):
        return urlopen(self.server.url + path, timeout=2.)

    def test_assets_status_and_path_boundaries(self):
        for path, mime in (("/", "text/html"), ("/viewer.css", "text/css"), ("/viewer.js", "text/javascript")):
            with self.get(path) as response:
                self.assertEqual(response.headers.get_content_type(), mime)
                self.assertGreater(len(response.read()), 0)
                self.assertEqual(response.headers["Cache-Control"], "no-store")
        with self.get("/api/cameras") as response:
            state = json.load(response)
        self.assertEqual(state["cameras"][0]["source"], "/dev/video0")
        self.assertEqual(state["cameras"][0]["state"], "waiting")
        for path in ("/../webcam_viewer.py", "/%2e%2e/webcam_viewer.py", "/streams/9.mjpg", "/unknown"):
            with self.assertRaises(HTTPError) as caught:
                self.get(path)
            self.assertEqual(caught.exception.code, 404)

    def test_mjpeg_bytes_and_refresh_use_the_same_capture_slot(self):
        with self.camera.condition:
            self.camera.frame, self.camera.sequence = b"\xff\xd8test\xff\xd9", 1
            self.camera.first_frame = True
            self.camera.condition.notify_all()
        for _ in range(2):
            with self.get("/streams/0.mjpg") as response:
                self.assertTrue(response.headers["Content-Type"].startswith("multipart/x-mixed-replace"))
                self.assertEqual(response.readline(), b"--frame\r\n")
                self.assertEqual(response.readline(), b"Content-Type: image/jpeg\r\n")
                self.assertEqual(response.readline(), b"Content-Length: 8\r\n")
                self.assertEqual(response.readline(), b"\r\n")
                self.assertEqual(response.read(8), b"\xff\xd8test\xff\xd9")

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
