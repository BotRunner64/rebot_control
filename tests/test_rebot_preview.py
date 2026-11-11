"""Preview conversion keeps duration and original data while reducing decode work."""

import hashlib
import os
from pathlib import Path
import subprocess
import tempfile
import time
import unittest
from unittest.mock import patch

import cv2

from rebot_preview import PreviewCache


class PreviewTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.cache = PreviewCache(self.root / "cache")
        self.addCleanup(self.cache.close)

    def wait(self, source):
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline:
            result = self.cache.prepare([source])
            if result["state"] != "preparing":
                return result
            time.sleep(.02)
        self.fail("Preview preparation did not complete.")

    def test_proxy_reduces_size_and_frame_rate_without_changing_duration_or_source(self):
        source = self.root / "source.mp4"
        subprocess.run(["/usr/bin/python3", "-B", "-c", '''
import sys, gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst
Gst.init(None)
p = Gst.parse_launch("videotestsrc num-buffers=60 pattern=ball ! video/x-raw,width=960,height=720,framerate=30/1 ! x264enc tune=zerolatency bframes=0 ! video/x-h264,stream-format=avc,alignment=au ! mp4mux ! filesink name=output")
p.get_by_name("output").set_property("location", sys.argv[1])
p.set_state(Gst.State.PLAYING)
m = p.get_bus().timed_pop_filtered(15 * Gst.SECOND, Gst.MessageType.EOS | Gst.MessageType.ERROR)
p.set_state(Gst.State.NULL)
assert m and m.type == Gst.MessageType.EOS
''', str(source)], check=True, capture_output=True)
        original = hashlib.sha256(source.read_bytes()).hexdigest()
        self.assertEqual(self.wait(source)["state"], "ready")
        proxy = self.cache.path(source)
        capture = cv2.VideoCapture(str(proxy))
        try:
            self.assertEqual(capture.get(cv2.CAP_PROP_FRAME_WIDTH), 640)
            self.assertEqual(capture.get(cv2.CAP_PROP_FRAME_HEIGHT), 480)
            self.assertAlmostEqual(capture.get(cv2.CAP_PROP_FPS), 15, delta=.1)
            self.assertAlmostEqual(capture.get(cv2.CAP_PROP_FRAME_COUNT) / capture.get(cv2.CAP_PROP_FPS), 2, delta=1/15)
            frames = 0
            while capture.read()[0]:
                frames += 1
            self.assertEqual(frames, 30)
        finally:
            capture.release()
        self.assertEqual(hashlib.sha256(source.read_bytes()).hexdigest(), original)
        # A new server can reuse the finished file without launching a converter.
        second = PreviewCache(self.cache.directory)
        try:
            with patch("rebot_preview.subprocess.Popen", side_effect=AssertionError("Cache was not reused")):
                self.assertEqual(second.prepare([source])["state"], "ready")
        finally:
            second.close()
        previous = self.cache.path(source)
        stat = source.stat()
        os.utime(source, ns=(stat.st_atime_ns, stat.st_mtime_ns + 1))
        self.assertNotEqual(previous, self.cache.path(source))
        # Removing a cached proxy rebuilds it, even in the same server process.
        previous.unlink()
        os.utime(source, ns=(stat.st_atime_ns, stat.st_mtime_ns))
        self.assertEqual(self.wait(source)["state"], "ready")
        self.assertTrue(previous.is_file())

    def test_invalid_video_never_publishes_a_partial_proxy(self):
        source = self.root / "broken.mp4"
        source.write_bytes(b"Not a video")
        result = self.wait(source)
        self.assertEqual(result["state"], "error")
        self.assertFalse(self.cache.path(source).exists())
        self.assertEqual(list(self.cache.directory.glob(".preview-*.mp4")), [])


if __name__ == "__main__":
    unittest.main()
