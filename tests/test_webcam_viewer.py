"""Regression tests for independent latest-frame camera capture."""

import contextlib
import io
import queue
import sys
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch

from webcam_viewer import CameraReader, CameraWindow, configure_capture, display, parse_args


class CaptureError(Exception):
    pass


class ControlledCapture:
    def __init__(self):
        self.frames = queue.Queue()
        self.reading = threading.Event()
        self.released = threading.Event()
        self.release_thread = None

    def isOpened(self):
        return True

    def read(self):
        self.reading.set()
        item = self.frames.get(timeout=5)
        if isinstance(item, Exception):
            raise item
        return item is not None, item

    def release(self):
        self.release_thread = threading.current_thread()
        self.released.set()


class CameraReaderTests(unittest.TestCase):
    def make_reader(self, source):
        capture = ControlledCapture()
        reader = CameraReader(source, capture, SimpleNamespace(error=CaptureError))
        reader.start()
        self.addCleanup(self.close_reader, reader, capture)
        self.assertTrue(capture.reading.wait(1))
        return reader, capture

    @staticmethod
    def close_reader(reader, capture):
        reader.stop()
        capture.frames.put(None)
        reader.close()

    def wait_for(self, predicate):
        deadline = time.monotonic() + 2
        while not predicate():
            if time.monotonic() >= deadline:
                self.fail("Timed out waiting for capture state")
            threading.Event().wait(0.001)

    def test_blocked_camera_does_not_block_latest_frame_updates(self):
        slow, slow_capture = self.make_reader("slow")
        fast, fast_capture = self.make_reader("fast")
        for value in range(1, 101):
            fast_capture.frames.put(SimpleNamespace(size=1, value=value))

        self.wait_for(lambda: fast.snapshot()[0] is not None
                      and fast.snapshot()[0].value == 100)
        frame, received_at, _ = fast.snapshot()
        self.assertEqual(frame.value, 100)
        self.assertGreater(received_at, 0)
        self.assertIsNone(slow.snapshot()[0])
        self.assertFalse(slow_capture.released.is_set())

    def test_failed_read_clears_old_frame_and_recovers(self):
        reader, capture = self.make_reader("recovering")
        capture.frames.put(SimpleNamespace(size=1, value=1))
        self.wait_for(lambda: reader.snapshot()[0] is not None)
        with contextlib.redirect_stderr(io.StringIO()), contextlib.redirect_stdout(io.StringIO()):
            capture.frames.put(None)
            self.wait_for(lambda: reader.snapshot()[0] is None)
            capture.frames.put(SimpleNamespace(size=1, value=2))
            self.wait_for(lambda: reader.snapshot()[0] is not None)
        self.assertEqual(reader.snapshot()[0].value, 2)

    def test_capture_error_releases_device_on_reader_thread(self):
        reader, capture = self.make_reader("broken")
        with contextlib.redirect_stderr(io.StringIO()):
            capture.frames.put(CaptureError("Disconnected"))
            self.assertTrue(capture.released.wait(1))
        self.assertIsNone(reader.snapshot()[0])
        self.assertIs(capture.release_thread, reader.thread)


class CaptureSettingsTests(unittest.TestCase):
    def setUp(self):
        self.cv2 = SimpleNamespace(CAP_PROP_FRAME_WIDTH=3, CAP_PROP_FRAME_HEIGHT=4,
                                   CAP_PROP_FPS=5, CAP_PROP_FOURCC=6, CAP_PROP_BUFFERSIZE=38)
        self.values = {3: 640, 4: 480, 5: 60, 6: 1196444237, 38: 4}
        self.changes = []
        self.capture = SimpleNamespace(get=self.values.get, set=self.set_property)
        self.args = SimpleNamespace(width=None, height=None, fps=None,
                                    fourcc=None, buffer_size=None)

    def set_property(self, prop, value):
        self.changes.append((prop, value))
        self.values[prop] = value
        return True

    def test_defaults_do_not_renegotiate_native_stream(self):
        configure_capture(self.capture, self.args, self.cv2, "test")
        self.assertEqual(self.changes, [])
        self.assertEqual(self.values[5], 60)
        self.assertEqual(self.values[38], 4)

    def test_linux_cli_defaults_use_verified_capture_mode(self):
        with patch("webcam_viewer.sys.platform", "linux"), \
                patch("webcam_viewer.sys.argv", ["webcam_viewer.py"]), \
                patch("webcam_viewer.discover_cameras", return_value=["/dev/video5", "/dev/video7"]):
            args = parse_args()
        self.assertEqual((args.width, args.height, args.fps, args.backend),
                         (1920, 1080, 30, "v4l2"))
        self.assertIsNone(args.buffer_size)
        self.assertIsNone(args.fourcc)

    def test_buffer_changes_only_when_explicitly_requested(self):
        self.args.buffer_size = 2
        configure_capture(self.capture, self.args, self.cv2, "test")
        self.assertEqual(self.changes, [(38, 2)])

    def test_frame_size_sets_both_dimensions_even_if_height_is_unchanged(self):
        self.args.width, self.args.height = 1280, 480
        pending = {}

        def set_paired_size(prop, value):
            self.changes.append((prop, value))
            pending[prop] = value
            if 3 in pending and 4 in pending:
                self.values.update(pending)
            return True

        self.capture.set = set_paired_size
        output = io.StringIO()
        with contextlib.redirect_stderr(output):
            configure_capture(self.capture, self.args, self.cv2, "test")
        self.assertEqual(self.changes, [(3, 1280), (4, 480)])
        self.assertEqual(self.values[3], 1280)
        self.assertEqual(output.getvalue(), "")

    def test_reports_frame_rate_request_ignored_by_device(self):
        self.args.fps = 30
        self.capture.set = lambda prop, value: True
        output = io.StringIO()
        with contextlib.redirect_stderr(output):
            configure_capture(self.capture, self.args, self.cv2, "test")
        self.assertIn("requested FPS 30, got 60", output.getvalue())

class CameraWindowTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            import numpy as np
            from PIL import Image, ImageOps
        except ImportError as exc:
            raise unittest.SkipTest("Window rendering tests need NumPy and Pillow") from exc
        cls.np, cls.image, cls.image_ops = np, Image, ImageOps

    def setUp(self):
        self.tk = MagicMock()
        self.canvas = self.tk.Canvas.return_value
        self.canvas.winfo_width.return_value = 640
        self.canvas.winfo_height.return_value = 480
        self.image_tk = MagicMock()
        self.reader = MagicMock(source="test")
        frame = self.np.zeros((1080, 1920, 3), dtype=self.np.uint8)
        frame[:, :, 2] = 255
        self.reader.snapshot.return_value = (frame, 99.5, 30)
        self.window = CameraWindow(MagicMock(), self.reader, 0, self.tk,
                                   self.image, self.image_tk, self.image_ops)
        clock = patch("webcam_viewer.time.monotonic", return_value=100)
        clock.start()
        self.addCleanup(clock.stop)

    def test_bgr_frame_is_red_and_keeps_aspect_ratio(self):
        self.window.update()
        image = self.image_tk.PhotoImage.call_args.args[0]
        self.assertEqual(image.size, (640, 360))
        self.assertEqual(image.getpixel((320, 180)), (255, 0, 0))
        self.canvas.coords.assert_called_with(self.window.picture, 320, 240)
        self.assertIs(self.window.photo, self.image_tk.PhotoImage.return_value)

    def test_resize_redraws_latest_frame_without_waiting_for_capture(self):
        self.window.update()
        self.window.update()
        self.assertEqual(self.image_tk.PhotoImage.call_count, 1)
        self.canvas.winfo_width.return_value = 320
        self.canvas.winfo_height.return_value = 400
        self.window.update()
        image = self.image_tk.PhotoImage.call_args.args[0]
        self.assertEqual(image.size, (320, 180))
        self.canvas.coords.assert_called_with(self.window.picture, 160, 200)

    def test_stale_frame_clears_preview_and_fresh_frame_recovers(self):
        self.window.update()
        frame = self.reader.snapshot.return_value[0]
        self.reader.snapshot.return_value = (frame, 98, 0)
        self.window.update()
        self.canvas.itemconfigure.assert_called_with(self.window.picture, image="")
        self.assertIsNone(self.window.photo)
        self.assertIn("no signal", self.window.status.configure.call_args.kwargs["text"])
        self.reader.snapshot.return_value = (frame, 99.9, 30)
        self.window.update()
        self.assertIsNotNone(self.window.photo)
        self.assertNotIn("no signal", self.window.status.configure.call_args.kwargs["text"])


class DisplayLifecycleTests(unittest.TestCase):
    def test_no_open_devices_releases_captures_and_destroys_tk(self):
        tk, cv2 = MagicMock(), MagicMock()
        cv2.VideoCapture.return_value.isOpened.return_value = False
        args = SimpleNamespace(cameras=["0"], backend="auto")
        with contextlib.redirect_stderr(io.StringIO()):
            result = display(args, cv2, tk, None, None, None)
        self.assertEqual(result, 1)
        cv2.VideoCapture.return_value.release.assert_called_once()
        tk.Tk.return_value.destroy.assert_called_once()

    def test_gui_callback_error_stops_readers_and_destroys_tk(self):
        tk, cv2 = MagicMock(), MagicMock()
        cv2.VideoCapture.return_value.get.return_value = 0
        root = tk.Tk.return_value
        args = SimpleNamespace(cameras=["0"], backend="auto")

        def run_callback():
            callback = root.after.call_args.args[1]
            try:
                callback()
            except Exception:
                root.report_callback_exception(*sys.exc_info())

        root.mainloop.side_effect = run_callback
        with patch("webcam_viewer.CameraReader") as reader_type, \
                patch("webcam_viewer.configure_capture"), \
                patch("webcam_viewer.CameraWindow") as window_type, \
                contextlib.redirect_stdout(io.StringIO()):
            window_type.return_value.update.side_effect = RuntimeError("Render failed")
            with self.assertRaisesRegex(RuntimeError, "Render failed"):
                display(args, cv2, tk, None, None, None)
        reader_type.return_value.stop.assert_called_once()
        reader_type.return_value.close.assert_called_once()
        root.quit.assert_called_once()
        root.destroy.assert_called_once()


if __name__ == "__main__":
    unittest.main()
