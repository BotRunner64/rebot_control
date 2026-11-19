"""Contract export checks with actual Parquet and H.264 from synthetic cameras."""

from contextlib import redirect_stderr, redirect_stdout
from functools import partial
import importlib.util
import io
import json
import math
from pathlib import Path
import queue
import socket
import struct
import subprocess
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
from urllib.request import Request, urlopen

import mujoco
import numpy as np

from rebot.data import recording


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
            np.testing.assert_allclose(rotation, mount.xmat.reshape(3, 3), atol=1e-12)
            native = data.body("link6").xmat.reshape(3, 3)
            np.testing.assert_allclose(rotation[:, 0], native[:, 2], atol=1e-12)
            np.testing.assert_allclose(rotation[:, 1], -native[:, 1], atol=1e-12)
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


@unittest.skipUnless(ARROW_AVAILABLE, "Install the recording dependency extra")
@unittest.skipUnless(gst_available(), "System GStreamer H.264 plugins are required")
class RecordingIntegrationTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.references = {side: [.2] * 7 for side in recording.SIDES}

    def open_session(self, name, *, viewer=False, camera_factory=CAMERA_FACTORY, cameras=None):
        options = recording.RecordingOptions(self.root, "pick up the cup", cameras or {"top": "synthetic"}, name,
                                             camera_viewer=viewer)
        session = recording.RecordingSession(options, self.references, camera_factory=camera_factory)
        self.addCleanup(session.close)
        deadline = time.monotonic() + 8.
        while not session.ready():
            if time.monotonic() >= deadline:
                self.fail("Synthetic recording startup timed out")
            time.sleep(.01)
        self.started_at = time.monotonic_ns()
        return session

    def finish_sample(self, session):
        import pyarrow.parquet as pq
        # Supply both arms across the test camera's recorded interval.
        session.stop_cameras()
        stop = time.monotonic_ns()
        start = self.started_at
        for stamp in range(start, stop + 20_000_000, 20_000_000):
            for side in recording.SIDES:
                session.samples.put((side, stamp, [.2] * 7, stamp))
        with redirect_stdout(io.StringIO()):
            session.finish(timeout=10)
        session.close()
        return pq.read_table(self.root / "recordings" / session.episode_id / "frames.parquet")

    def test_live_viewer_uses_recorded_frames_and_closing_it_keeps_recording(self):
        viewer_factory = partial(recording.RecordingViewer, open_browser=False, port=0)
        with patch.object(recording, "RecordingViewer", side_effect=viewer_factory):
            session = self.open_session("viewer", viewer=True)
        session.start()
        started = time.monotonic()
        deadline = time.monotonic() + 8
        while not (session.viewer.root / "url").exists():
            self.assertIsNone(session.viewer.process.poll())
            if time.monotonic() >= deadline:
                self.fail("Recording viewer did not start")
            time.sleep(.02)
        url = (session.viewer.root / "url").read_text()
        while True:
            with urlopen(url + "/api/cameras", timeout=2) as response:
                state = json.load(response)
            if state["cameras"][0]["state"] == "live":
                break
            self.assertNotEqual(state["cameras"][0]["state"], "error", state)
            if time.monotonic() >= deadline:
                self.fail("No live frame arrived")
            time.sleep(.02)
        self.assertTrue(state["recording"])
        self.assertIn("top", state["cameras"][0]["source"])
        with urlopen(url + "/streams/0.mjpg", timeout=2) as response:
            response.readline(); response.readline()
            size = int(response.readline().split(b":")[1])
            response.readline()
            jpeg = response.read(size)
        self.assertTrue(jpeg.startswith(b"\xff\xd8") and jpeg.endswith(b"\xff\xd9"))
        request = Request(url + "/api/stop", data=b"{}",
                          headers={"Origin": url, "Content-Type": "application/json"})
        with urlopen(request, timeout=2) as response:
            self.assertEqual(json.load(response), {"stopped": True})
        self.assertEqual(session.viewer.process.wait(4), 0)
        closed_at = time.monotonic() - started
        time.sleep(.55)
        session.check()
        self.assertIsNone(session.cameras[0].process.poll())
        table = self.finish_sample(session)
        self.assertGreater(table.num_rows, 10)
        self.assertGreater(table.num_rows / 30, closed_at + .3)

    def test_three_camera_previews_share_capture_and_all_streams_are_saved(self):
        import av
        cameras = {"top": "synthetic0", "left_wrist": "synthetic1", "right_wrist": "synthetic2"}
        viewer_factory = partial(recording.RecordingViewer, open_browser=False, port=0)
        with patch.object(recording, "RecordingViewer", side_effect=viewer_factory):
            session = self.open_session("three_cameras", viewer=True, cameras=cameras)
        session.start()
        deadline = time.monotonic() + 8
        while not (session.viewer.root / "url").exists():
            if time.monotonic() >= deadline:
                self.fail("Recording viewer did not start")
            time.sleep(.02)
        url = (session.viewer.root / "url").read_text()
        while True:
            with urlopen(url + "/api/cameras", timeout=2) as response:
                state = json.load(response)
            if all(camera["state"] == "live" for camera in state["cameras"]):
                break
            if time.monotonic() >= deadline:
                self.fail(f"Not all previews became live: {state}")
            time.sleep(.02)
        self.assertEqual(len(state["cameras"]), 3)
        time.sleep(.2)
        for camera in state["cameras"]:
            with urlopen(url + f"/streams/{camera['id']}.mjpg", timeout=2) as response:
                response.readline(); response.readline()
                size = int(response.readline().split(b":")[1])
                response.readline()
                self.assertTrue(response.read(size).startswith(b"\xff\xd8"))
        table = self.finish_sample(session)
        for camera in cameras:
            directory = self.root / "recordings" / session.episode_id
            self.assertTrue((directory / (camera + ".mp4")).is_file())
            with av.open(str(directory / (camera + ".mp4"))) as reader:
                self.assertEqual(reader.streams.video[0].frames, table.num_rows)

    def test_batched_start_and_stop_commands_cannot_stall_camera_exit(self):
        camera = CAMERA_FACTORY("synthetic", self.root, "quick")
        self.addCleanup(camera.close)
        deadline = time.monotonic() + 5
        while not camera.ready:
            camera.check()
            if time.monotonic() >= deadline:
                self.fail("Synthetic camera did not become ready")
            time.sleep(.01)
        now = time.monotonic_ns()
        commands = [{"command": "start", "timestamp_ns": now}, {"command": "stop", "timestamp_ns": now}]
        camera.process.stdin.write(("\n".join(json.dumps(item) for item in commands) + "\n").encode())
        camera.process.stdin.flush()
        # An empty capture may be rejected, but stop must be consumed without hanging.
        camera.process.wait(5)

    def test_slow_preview_receiver_cannot_backpressure_recording(self):
        listener = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        path = self.root / "slow.sock"
        listener.bind(str(path)); listener.listen(1)
        listener.settimeout(5)
        stopped, connected = threading.Event(), threading.Event()
        def slow_receiver():
            connection = None
            try:
                connection, _ = listener.accept()
                connection.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024)
                connected.set()
                stopped.wait(5)  # Intentionally never read the preview stream.
            finally:
                if connection is not None:
                    connection.close()
        thread = threading.Thread(target=slow_receiver, daemon=True)
        thread.start()
        try:
            factory = partial(recording.CameraProcess, test=True, preview_socket=path)
            session = self.open_session("slow", camera_factory=factory)
            session.start()
            self.assertTrue(connected.wait(3))
            began = time.monotonic()
            time.sleep(1.2)
            session.check()
            table = self.finish_sample(session)
            self.assertGreaterEqual(table.num_rows, 20)
            self.assertLess(time.monotonic() - began, 5)
        finally:
            stopped.set(); listener.close(); thread.join(2)

    def test_viewer_startup_failure_does_not_stop_recording(self):
        with patch.object(recording, "RecordingViewer", side_effect=OSError("Viewer unavailable")), \
                redirect_stderr(io.StringIO()):
            session = self.open_session("no_viewer", viewer=True)
        self.assertIsNone(session.viewer)
        session.start(); time.sleep(.2)
        self.assertGreater(self.finish_sample(session).num_rows, 0)

    def test_synchronized_recordings_append_independently(self):
        import pyarrow as pa
        for name in ("first", "second"):
            session = self.open_session(name)
            session.start()
            time.sleep(.3)
            table = self.finish_sample(session)
            self.assertEqual(table["observation.state"].type, pa.list_(pa.float32(), 26))
            state = table["observation.state"][0].as_py()
            expected_pose = recording.ForwardKinematics().pose(np.zeros(6))
            np.testing.assert_allclose(state[:12], expected_pose, atol=1e-6)
            np.testing.assert_allclose(state[13:25], expected_pose, atol=1e-6)
            self.assertEqual([state[12], state[25]], [0., 0.])
            directory = self.root / "recordings" / name
            self.assertEqual({p.name for p in directory.iterdir()}, {"frames.parquet", "top.mp4", "recording.json"})
            self.assertEqual(json.loads((directory / "recording.json").read_text())["frames"], table.num_rows)
        self.assertEqual({p.name for p in (self.root / "recordings").iterdir()}, {"first", "second"})

    def test_relayed_jpeg_frames_are_recorded_with_their_own_timestamps(self):
        import pyarrow.parquet as pq
        from PIL import Image

        options = recording.RecordingOptions(self.root, "unscrew the cap", {"top": "/dev/video99"}, "relayed")
        session = recording.RecordingSession(options, self.references, relay=True)
        self.addCleanup(session.close)

        def send(index):
            # Encode the source frame number in five black/white stripes.
            stripes = [255 if index & (1 << bit) else 0 for bit in range(5)]
            pixels = np.tile(np.repeat(stripes, 32), (120, 1)).astype(np.uint8)
            jpeg = io.BytesIO()
            Image.fromarray(pixels).convert("RGB").save(jpeg, format="JPEG")
            stamp = time.monotonic()
            payload = struct.pack("!d", stamp) + jpeg.getvalue()
            session.feeds["top"].sendall(struct.pack("!I", len(payload)) + payload)
            return round(stamp * 1e9)

        deadline = time.monotonic() + 8.
        while not session.ready():          # the recorder becomes ready on its first relayed frame
            send(0)
            self.assertLess(time.monotonic(), deadline)
            time.sleep(.02)
        session.publish("left", [{"received_at_ns": 1, "position": .2}] * 7)   # ignored before start
        with redirect_stdout(io.StringIO()):
            session.start()
        time.sleep(.2)
        # No settling time after the last frame: frames in flight at stop must still be saved.
        sent = [(send(index), time.sleep(.01))[0] for index in range(20)]
        for stamp in sent:
            for side in recording.SIDES:
                session.publish(side, [{"received_at_ns": stamp, "position": .2}] * 7)
        with redirect_stdout(io.StringIO()):
            session.finish(timeout=10)
        session.close()
        import av
        from loom_data import sync
        episode = self.root / "recordings" / "relayed"
        grid = sync.frame_grid([sent])
        count = pq.ParquetFile(episode / "frames.parquet").metadata.num_rows
        self.assertEqual(count, len(grid))
        with av.open(str(episode / "top.mp4")) as reader:
            selected = []
            for frame in reader.decode(video=0):
                row = frame.to_ndarray(format="gray")[frame.height // 2]
                bits = row[frame.width // 10::frame.width // 5] > 128
                selected.append(sum(1 << bit for bit, on in enumerate(bits) if on))
        np.testing.assert_array_equal(selected, sync.nearest(sent, grid))

    def test_debug_streams_are_saved_beside_the_contract_streams_on_the_same_time_origin(self):
        import pyarrow.parquet as pq

        options = recording.RecordingOptions(self.root, "unscrew the cap", {"top": "synthetic"}, "debugged")
        session = recording.RecordingSession(options, self.references, camera_factory=CAMERA_FACTORY, debug=True)
        self.addCleanup(session.close)
        deadline = time.monotonic() + 8.
        while not session.ready():
            self.assertLess(time.monotonic(), deadline)
            time.sleep(.01)
        with redirect_stdout(io.StringIO()):
            session.start()
        time.sleep(.2)
        now = time.monotonic()
        stamp = round(now * 1e9)
        states = [{"received_at_ns": stamp, "position": .2 + .01 * i, "velocity": .1 * i, "torque": float(i)}
                  for i in range(7)]
        command = {"target": np.arange(7.) / 100, "torque": np.arange(7.) / 10}
        for side in recording.SIDES:
            session.publish(side, states, command)
        for offset in range(1, 11):
            for side in recording.SIDES:
                session.samples.put((side, stamp + offset * 20_000_000, [.2] * 7, stamp + offset * 20_000_000))
        time.sleep(.25)
        times = now + (np.arange(3) + 1) / 30
        pose = [.1, .2, .3, 1., 0., 0., 0., 1., 0., 0., 0., 1.]
        chunk = {name: (times, np.tile(pose, (3, 1)) if name.endswith("_eef") else np.array([.5, .4, .3]))
                 for name in recording.STATE_NAMES}
        session.debug.chunk(chunk, now, now + .08, .075, 2)
        session.debug.target(now + .1, [*pose, .3, *pose, .7], 0, 2, False)
        session.debug.write_run({"arguments": {"replan_steps": 16}})
        with redirect_stdout(io.StringIO()):
            session.finish(timeout=10)
        session.close()
        episode = self.root / "recordings" / "debugged"
        self.assertEqual({p.name for p in (episode / "debug").iterdir()},
                         {"left_joints.parquet", "right_joints.parquet", "targets.parquet", "chunks.parquet",
                          "run.json"})
        self.assertEqual(json.loads((episode / "debug" / "run.json").read_text())["arguments"]["replan_steps"], 16)
        # Every debug timestamp shares the origin of the contract streams.
        joints = pq.read_table(episode / "debug" / "left_joints.parquet").to_pylist()
        self.assertEqual(len(joints), 1)
        self.assertEqual(joints[0]["timestamp_ns"], 0)
        np.testing.assert_allclose(joints[0]["position"], np.arange(7) * .01, atol=1e-12)
        self.assertEqual(joints[0]["velocity"][3], .1 * 3)
        self.assertEqual(joints[0]["torque"], list(map(float, range(7))))
        np.testing.assert_allclose(joints[0]["target"], command["target"])
        np.testing.assert_allclose(joints[0]["feedforward"], command["torque"])
        row = pq.read_table(episode / "debug" / "chunks.parquet").to_pylist()[0]
        origin = stamp
        self.assertEqual(row["observed_timestamp_ns"], stamp - origin)
        self.assertEqual(row["timestamp_ns"], round((now + .08) * 1e9) - origin)
        self.assertEqual((row["inference_ms"], row["first_row"]), (75., 2))
        self.assertEqual(row["left_eef"], [pose] * 3)
        np.testing.assert_allclose(row["right_gripper"], [.5, .4, .3])
        np.testing.assert_allclose(row["left_eef_offset_ns"], (np.arange(3) + 1) * 1e9 / 30, atol=2)
        target = pq.read_table(episode / "debug" / "targets.parquet").to_pylist()[0]
        self.assertEqual(target["timestamp_ns"], round((now + .1) * 1e9) - origin)
        self.assertEqual((target["left_pose"], target["chunk"], target["row"], target["sent"]), (pose, 0, 2, False))
        self.assertAlmostEqual(target["right_openness"], .7, places=6)
        # The contract part of the episode is exactly what a recording without debug streams holds.
        self.assertEqual({p.name for p in episode.iterdir()}, {"frames.parquet", "top.mp4", "recording.json", "debug"})

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
        self.assertFalse((self.root / "recordings").exists())
        self.assertTrue((self.root / ".pending" / "invalid").exists())
        crashed = self.open_session("crashed")
        crashed.process.kill()
        crashed.process.join(2.)
        with self.assertRaisesRegex(RuntimeError, "unexpectedly"):
            crashed.check()
        crashed.close()
        self.assertFalse((self.root / "recordings").exists())

    def test_dataset_lock_and_duplicate_ids_do_not_overwrite(self):
        session = self.open_session("reserved")
        with self.assertRaisesRegex(RuntimeError, "Another recording"):
            self.open_session("concurrent")
        session.close()
        with self.assertRaises(FileExistsError):
            self.open_session("reserved")


if __name__ == "__main__":
    unittest.main()
