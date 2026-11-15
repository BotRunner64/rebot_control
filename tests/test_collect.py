"""Recording through the spawned teleop workers with virtual hardware."""

from contextlib import redirect_stderr, redirect_stdout
from functools import partial
import importlib.util
import io
import math
import multiprocessing as mp
import os
from pathlib import Path
import queue
import signal
import subprocess
import tempfile
import time
import unittest
from unittest.mock import MagicMock, patch

import numpy as np

from rebot.apps import teleop
from rebot.data import recording
from teleop_fakes import BusFactory, SourceFactory, virtual_bindings


ARROW_AVAILABLE = importlib.util.find_spec("pyarrow") is not None
CAMERA_FACTORY = partial(recording.CameraProcess, test=True)


def gst_available():
    result = subprocess.run(["/usr/bin/python3", "-c",
        "import gi; gi.require_version('Gst','1.0'); from gi.repository import Gst; "
        "Gst.init(None); assert all(Gst.ElementFactory.find(n) for n in "
        "('x264enc','mp4mux','qtdemux','avdec_h264','videotestsrc'))"],
        capture_output=True, timeout=10)
    return result.returncode == 0


class CollectUnitTests(unittest.TestCase):
    def test_recording_rejects_missing_sides_before_connecting_hardware(self):
        for side, gripper in (("left", True), ("both", False)):
            arms, leaders = virtual_bindings(side)
            with patch.object(teleop, "select_arms", return_value=arms), \
                    patch.object(teleop, "select_leaders", return_value=leaders):
                plan = teleop.make_plan(side=side, gripper=gripper)
            with self.subTest(side=side, gripper=gripper), \
                    patch.object(recording, "RecordingSession") as session, \
                    patch.object(teleop.mp, "get_context") as spawn, \
                    self.assertRaisesRegex(ValueError, "both"):
                teleop.teleop(plan, recording=object(), source_factory=MagicMock(), bus_factory=MagicMock())
            session.assert_not_called()
            spawn.assert_not_called()


@unittest.skipUnless(ARROW_AVAILABLE, "Install the recording dependency extra")
@unittest.skipUnless(gst_available(), "System GStreamer H.264 plugins are required")
class TeleopRecordingTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)

    def run_virtual_teleop(self, failure="", *, terminal_interrupt=False):
        context = mp.get_context("spawn")
        counts, reports = context.Array("i", 2), context.Queue()
        arms, leaders = virtual_bindings()
        options = recording.RecordingOptions(self.root, "move the arms", {"top": "synthetic"}, "teleop")
        original = recording.RecordingSession
        sessions = []

        def create_session(options, references):
            self.assertEqual(set(references), set(recording.SIDES))
            for reference in references.values():
                np.testing.assert_array_equal(reference, [.2] * 7)
            session = original(options, references, camera_factory=CAMERA_FACTORY)
            sessions.append(session)
            return session

        keys = MagicMock()
        began = time.monotonic()

        def pressed():
            done = sum(counts) >= 100 or time.monotonic() - began > 10.
            if done and terminal_interrupt:
                # Emulate a terminal delivering Ctrl-C to all owned workers.
                for child in mp.active_children():
                    os.kill(child.pid, signal.SIGINT)
                for camera in sessions[0].cameras:
                    os.kill(camera.process.pid, signal.SIGINT)
                os.kill(os.getpid(), signal.SIGINT)
                return False
            return done

        keys.__enter__.return_value.pressed.side_effect = pressed
        try:
            with patch.object(teleop, "select_arms", return_value=arms), \
                    patch.object(teleop, "select_leaders", return_value=leaders), \
                    patch.object(teleop, "StopKey", return_value=keys), \
                    patch.object(recording, "RecordingSession", side_effect=create_session), \
                    redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
                def run():
                    teleop.teleop(teleop.make_plan(), recording=options, source_factory=SourceFactory(reports),
                                  bus_factory=BusFactory(reports, counts, failure), startup_timeout=8.)

                if failure:
                    with self.assertRaisesRegex(RuntimeError, "worker failure"):
                        run()
                else:
                    run()
            hardware = []
            while True:
                try:
                    hardware.append(reports.get(timeout=.1))
                except queue.Empty:
                    break
        finally:
            reports.close()
            reports.join_thread()
        self.assertFalse([child for child in mp.active_children() if child.name == "record-state"])
        return [record for record in hardware if record["kind"] == "motor"]

    def test_spawned_teleop_records_actual_feedback_and_preserves_holds(self):
        import pyarrow.parquet as pq

        motors = self.run_virtual_teleop()
        self.assertEqual(len(motors), 2)
        for motor in motors:
            side = "left" if motor["channel"] == "can0" else "right"
            table = pq.read_table(self.root / "episodes" / "teleop" / "state" / f"{side}_eef.parquet")
            # Virtual feedback stays near the calibration pose while leader targets differ.
            bound = 4 * math.pi
            measured = int((.2 + bound) / (2 * bound) * 65535) / 65535 * 2 * bound - bound
            expected = recording.ForwardKinematics().pose(np.full(6, measured - .2))
            np.testing.assert_allclose(table["pose"][0].as_py(), expected, atol=1e-12)
            self.assertGreater(table.num_rows, 0)
            self.assertEqual(motor["position_queries"], 21)
            self.assertTrue(motor["closed"])
            self.assertEqual(motor["enabled"], list(range(1, 8)))

    def test_motor_failure_retains_pending_data_and_keeps_both_arms_held(self):
        motors = self.run_virtual_teleop("command")
        self.assertEqual(len(motors), 2)
        for motor in motors:
            self.assertTrue(motor["closed"])
            self.assertEqual(motor["enabled"], list(range(1, 8)))
            self.assertNotIn(4, motor["types"])
        self.assertFalse((self.root / "episodes.jsonl").exists())
        self.assertTrue((self.root / ".pending" / "teleop").exists())

    def test_terminal_ctrl_c_drains_recorders_and_publishes_complete_episode(self):
        motors = self.run_virtual_teleop(terminal_interrupt=True)
        self.assertEqual(len(motors), 2)
        self.assertTrue((self.root / "episodes.jsonl").exists())
        self.assertTrue((self.root / "episodes" / "teleop" / "rgb" / "top.parquet").exists())


if __name__ == "__main__":
    unittest.main()
