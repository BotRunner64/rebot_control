"""Contract export checks with actual Parquet, H.264, and spawned virtual hardware."""

from contextlib import redirect_stderr, redirect_stdout
from functools import partial
import importlib.util
import io
import json
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

import mujoco
import numpy as np

import rebot_recording as recording
import rebot_teleop as teleop
from teleop_process_fakes import BusFactory, MeasuredMotors, SourceFactory


ARROW_AVAILABLE = importlib.util.find_spec("pyarrow") is not None
CAMERA_FACTORY = partial(recording.CameraProcess, test=True)


def gst_available():
    result = subprocess.run(["/usr/bin/python3", "-c",
        "import gi; gi.require_version('Gst','1.0'); from gi.repository import Gst; "
        "Gst.init(None); assert all(Gst.ElementFactory.find(n) for n in "
        "('x264enc','mp4mux','qtdemux','avdec_h264','videotestsrc'))"],
        capture_output=True, timeout=10)
    return result.returncode == 0


class RecordingUnitTests(unittest.TestCase):
    def test_fk_agrees_with_mujoco_mount_and_contract_axes(self):
        model = mujoco.MjModel.from_xml_path(str(recording.MODEL_PATH))
        data = mujoco.MjData(model)
        fk = recording.ForwardKinematics()
        for q in (np.zeros(6), np.radians([20, 60, 90, -30, 40, 15])):
            data.qpos[:6] = q
            mujoco.mj_forward(model, data)
            mount = data.body("gripper_mount")
            pose = fk.pose(q)
            rotation = pose[3:].reshape(3, 3, order="F")
            np.testing.assert_allclose(pose[:3], mount.xpos, atol=1e-12)
            native = mount.xmat.reshape(3, 3)
            np.testing.assert_allclose(rotation[:, 0], native[:, 2], atol=1e-12)
            np.testing.assert_allclose(rotation[:, 2], native[:, 0], atol=1e-12)
            np.testing.assert_allclose(rotation.T @ rotation, np.eye(3), atol=1e-12)
            self.assertAlmostEqual(np.linalg.det(rotation), 1.)

    def test_camera_ids_devices_and_options_are_validated(self):
        self.assertEqual(recording.parse_cameras(["top=0", "left_wrist=/dev/video2"]),
                         {"top": "/dev/video0", "left_wrist": "/dev/video2"})
        for values in (["../top=0"], ["TOP=0"], ["top=0", "top=2"], ["top"]):
            with self.subTest(values=values), self.assertRaises(ValueError):
                recording.parse_cameras(values)
        for instruction, cameras, episode in (("", {"top": "0"}, "valid"),
            ("task", {}, "valid"), ("task", {"top": "0"}, "../outside"),
            ("task", {"top": "/dev/video0", "wrist": "/dev/video0"}, "valid")):
            with self.assertRaises(ValueError):
                recording.RecordingOptions(Path("unused"), instruction, cameras, episode).validate()

    def test_queue_overflow_stops_without_overwriting_or_blocking(self):
        samples = queue.Queue(maxsize=1)
        states = [{"received_at_ns": 100 + i, "position": float(i)} for i in range(7)]
        recording.publish_feedback(samples, "left", states)
        with self.assertRaisesRegex(RuntimeError, "overflow"):
            recording.publish_feedback(samples, "left", states)
        self.assertEqual(samples.get(), ("left", 105, list(map(float, range(7))), 106))

    def test_recording_rejects_missing_sides_before_connecting_hardware(self):
        for side, gripper in (("left", True), ("both", False)):
            with patch.object(teleop, "load_bindings") as bindings, self.assertRaisesRegex(ValueError, "both"):
                teleop.teleop(side=side, gripper=gripper, recording=object())
            bindings.assert_not_called()


@unittest.skipUnless(ARROW_AVAILABLE, "Install the recording dependency extra")
@unittest.skipUnless(gst_available(), "System GStreamer H.264 plugins are required")
class RecordingIntegrationTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.arms = {side: {"reference_rad": [.2] * 7} for side in recording.SIDES}

    def open_session(self, name):
        options = recording.RecordingOptions(self.root, "pick up the cup", {"top": "synthetic"}, name)
        session = recording.RecordingSession(options, self.arms, camera_factory=CAMERA_FACTORY)
        self.addCleanup(session.close)
        deadline = time.monotonic() + 8.
        while not session.ready():
            if time.monotonic() >= deadline:
                self.fail("Synthetic recording startup timed out")
            time.sleep(.01)
        return session

    def test_native_samples_shared_origin_schema_video_and_metadata_append(self):
        import pyarrow as pa
        import pyarrow.parquet as pq

        session = self.open_session("first")
        session.start()
        origin = time.monotonic_ns()
        # Cross row-group boundaries, preserve unequal rates and nonuniform intervals.
        left_times = [i * i * 100 for i in range(600)]
        for index, offset in enumerate(left_times):
            raw = [.2] * 6 + [.2 + math.radians(90)]
            session.samples.put(("left", origin + offset, raw, origin + offset + 300))
            if index < 100:
                session.samples.put(("right", origin + 1000 + index * 2000,
                                     [.2] * 6 + [20.], origin + 1100 + index * 2000))
        time.sleep(.25)
        with redirect_stdout(io.StringIO()):
            metadata = session.finish()
        session.close()
        episode = self.root / "episodes" / "first"
        left = pq.read_table(episode / "state" / "left_eef.parquet")
        self.assertEqual(left.column_names, ["timestamp_ns", "pose"])
        self.assertEqual(left.schema.field("timestamp_ns").type, pa.int64())
        self.assertEqual(left.schema.field("pose").type, pa.list_(pa.float64(), 12))
        self.assertFalse(left.schema.field("pose").nullable)
        self.assertEqual(left["timestamp_ns"].to_pylist(), left_times)
        self.assertEqual(left.num_rows, 600)
        expected_pose = recording.ForwardKinematics().pose(np.zeros(6))
        np.testing.assert_allclose(left["pose"][0].as_py(), expected_pose, atol=1e-12)
        right = pq.read_table(episode / "state" / "right_eef.parquet")
        self.assertEqual(right.num_rows, 100)
        self.assertEqual(right["timestamp_ns"][0].as_py(), 1000)
        left_gripper = pq.read_table(episode / "state" / "left_gripper.parquet")
        self.assertEqual(left_gripper.schema.field("openness").type, pa.float32())
        self.assertEqual(left_gripper["timestamp_ns"][0].as_py(), 300)
        self.assertAlmostEqual(left_gripper["openness"][0].as_py(), 1 / 3, places=6)
        right_gripper = pq.read_table(episode / "state" / "right_gripper.parquet")
        self.assertEqual(right_gripper["openness"].to_pylist(), [1.] * 100)
        rgb = pq.read_table(episode / "rgb" / "top.parquet")
        self.assertGreater(rgb.num_rows, 0)
        self.assertEqual(rgb["frame_index"].to_pylist(), list(range(rgb.num_rows)))
        self.assertGreater(rgb["timestamp_ns"][0].as_py(), 0)
        tables = (left, right, left_gripper, right_gripper, rgb)
        end = max(table["timestamp_ns"][-1].as_py() for table in tables)
        self.assertEqual(metadata["instructions"], [{"start_ns": 0, "end_ns": end, "text": "pick up the cup"}])
        self.assertEqual({p.name for p in (episode / "rgb").iterdir()}, {"top.mp4", "top.parquet"})
        self.assertEqual({p.name for p in episode.iterdir()}, {"state", "rgb"})
        second = self.open_session("second")
        second.start()
        for side in recording.SIDES:
            second.samples.put((side, time.monotonic_ns(), [.2] * 7, time.monotonic_ns()))
        time.sleep(.15)
        with redirect_stdout(io.StringIO()):
            second.finish()
        second.close()
        records = [json.loads(line) for line in (self.root / "episodes.jsonl").read_text().splitlines()]
        self.assertEqual([record["episode_id"] for record in records], ["first", "second"])
        self.assertEqual({p.name for p in (self.root / "episodes").iterdir()}, {"first", "second"})

    def test_invalid_feedback_and_recorder_death_never_publish_an_episode(self):
        session = self.open_session("invalid")
        session.start()
        session.samples.put(("left", time.monotonic_ns(), [math.nan] * 7, time.monotonic_ns()))
        deadline = time.monotonic() + 5.
        with self.assertRaisesRegex(RuntimeError, "finite|unexpectedly"):
            while time.monotonic() < deadline:
                session.check()
                time.sleep(.01)
        session.close()
        self.assertFalse((self.root / "episodes.jsonl").exists())
        self.assertTrue((self.root / ".pending" / "invalid").exists())
        crashed = self.open_session("crashed")
        crashed.process.kill()
        crashed.process.join(2.)
        with self.assertRaisesRegex(RuntimeError, "unexpectedly"):
            crashed.check()
        crashed.close()
        self.assertFalse((self.root / "episodes.jsonl").exists())

    def test_dataset_lock_and_duplicate_ids_do_not_overwrite(self):
        session = self.open_session("reserved")
        with self.assertRaisesRegex(RuntimeError, "Another recording"):
            self.open_session("concurrent")
        session.close()
        with self.assertRaises(FileExistsError):
            self.open_session("reserved")

    def run_virtual_teleop(self, failure="", *, terminal_interrupt=False):
        context = mp.get_context("spawn")
        counts, reports = context.Array("i", 2), context.Queue()
        arms = {side: {"reference_rad": [.2] * 7, "models": ["rs-06"] * 3 + ["rs-00"] * 4,
                       "motor_uids": MeasuredMotors(offset).uids}
                for side, offset in (("left", 0), ("right", 100))}
        leaders = {side: {"port": f"fake-{side}", "baudrate": 1000000, "reference_deg": [10.] * 7}
                   for side in recording.SIDES}
        channels = {"left": "can0", "right": "can1"}
        options = recording.RecordingOptions(self.root, "move the arms", {"top": "synthetic"}, "teleop")
        original = recording.RecordingSession
        sessions = []

        def create_session(options, arms):
            session = original(options, arms, camera_factory=CAMERA_FACTORY)
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
            with patch.object(teleop, "load_bindings", return_value=(recording.SIDES, arms, leaders, channels)), \
                    patch.object(teleop, "StopKey", return_value=keys), \
                    patch.object(recording, "RecordingSession", side_effect=create_session), \
                    redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
                def run():
                    teleop.teleop(recording=options, source_factory=SourceFactory(reports),
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
