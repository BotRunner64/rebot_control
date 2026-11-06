"""Capture selection and cleanup checks using the real GStreamer type system."""

import contextlib
import io
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch

from webcam_viewer import CameraPreview, choose_format, discover_cameras, display, parse_args

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
            self.assertEqual(vars(parse_args()), {"cameras": None})

    def test_indices_are_normalized_and_duplicate_paths_are_rejected(self):
        with patch.object(sys, "argv", ["webcam_viewer.py", "--cameras", "0", "/dev/video2"]):
            self.assertEqual(parse_args().cameras, ["/dev/video0", "/dev/video2"])
        with patch.object(sys, "argv", ["webcam_viewer.py", "--cameras", "0", "/dev/video0"]), \
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
        preview = CameraPreview("/dev/video0", MagicMock(), MagicMock())
        preview.pipeline = MagicMock()
        preview.started_at = 1.0
        with patch("webcam_viewer.time.monotonic", return_value=12), \
                contextlib.redirect_stderr(io.StringIO()):
            preview.check_timeout()
        self.assertTrue(preview.failed)

    def test_window_close_and_keyboard_events_quit_loop(self):
        gst = MagicMock()
        loop = MagicMock()
        preview = CameraPreview("/dev/video0", SimpleNamespace(Gst=gst), loop)
        message = SimpleNamespace(parse_error=lambda: (SimpleNamespace(message="Output window was closed"), None))
        preview.on_error(None, message)
        loop.quit.assert_called_once()
        loop.reset_mock()
        info = MagicMock()
        info.get_event.return_value.type = gst.EventType.NAVIGATION
        details = info.get_event.return_value.get_structure.return_value
        for key in ("q", "Escape"):
            details.get_string.side_effect = lambda field: "key-press" if field == "event" else key
            preview.on_navigation(None, info)
        self.assertEqual(loop.quit.call_count, 2)

    def test_main_loop_error_releases_all_previews_and_signal_callbacks(self):
        runtime = MagicMock()
        runtime.GLib.MainLoop.return_value.run.side_effect = KeyboardInterrupt
        with patch("webcam_viewer.CameraPreview") as preview_type, \
                contextlib.redirect_stdout(io.StringIO()):
            previews = [MagicMock(failed=False), MagicMock(failed=False)]
            preview_type.side_effect = previews
            with self.assertRaises(KeyboardInterrupt):
                display(["/dev/video0", "/dev/video2"], runtime)
        for preview in previews:
            preview.close.assert_called_once()
        self.assertEqual(runtime.GLib.source_remove.call_count, 2)


if __name__ == "__main__":
    unittest.main()
