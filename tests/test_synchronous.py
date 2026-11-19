"""Exercise native device timing against the pinned LOOM-data export contract."""

import tempfile
import unittest
from pathlib import Path

import av
import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq
from loom_data import contract
from scipy.spatial.transform import Rotation

from rebot.data.recording import schemas
from rebot.data.synchronous import synchronize


class SynchronizationTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        (self.root / "state").mkdir()
        (self.root / "rgb").mkdir()
        self.origin = 10**16
        self.write_states([0, 100])
        self.camera_times([5, 55, 100])
        with av.open(str(self.root / "rgb/top.mp4"), "w") as writer:
            stream = writer.add_stream("libx264", rate=20)
            stream.width, stream.height, stream.pix_fmt = 160, 120, "yuv420p"
            for shade in (0, 100, 200):
                frame = av.VideoFrame.from_ndarray(np.full((120, 160, 3), shade, np.uint8), format="rgb24")
                for packet in stream.encode(frame):
                    writer.mux(packet)
            for packet in stream.encode():
                writer.mux(packet)

    def camera_times(self, milliseconds):
        (self.origin + np.array(milliseconds, dtype="<i8") * 1_000_000).tofile(self.root / "rgb/top.timestamps")

    def write_states(self, milliseconds):
        times = self.origin + np.array(milliseconds, dtype=np.int64) * 1_000_000
        fractions = np.linspace(0, 1, len(times))
        rotations = Rotation.from_euler("z", fractions[:, None] * 90, degrees=True).as_matrix()
        poses = [[f, 0, 0, *r.T.flatten()] for f, r in zip(fractions, rotations)]
        for stream in contract.STATE_NAMES:
            kind = stream.split("_")[1]
            values = poses if kind == "eef" else fractions
            schema = schemas()[kind]
            table = pa.Table.from_arrays([pa.array(times), pa.array(values, type=schema[1].type)], schema=schema)
            pq.write_table(table, self.root / "state" / (stream + ".parquet"))

    def run_sync(self):
        return synchronize(self.root, self.root / "synced", ["top"], "pick")

    def test_interpolates_state_and_selects_nearest_camera_on_common_grid(self):
        metadata, origin = self.run_sync()
        self.assertEqual(origin, self.origin + 5_000_000)
        self.assertEqual(metadata["frames"], 3)
        table = pq.read_table(self.root / "synced/frames.parquet")
        states = np.array(table[contract.STATE_KEY].to_pylist())
        fraction = .05 + np.arange(3) / 3
        for offset in (0, 13):
            np.testing.assert_allclose(states[:, offset], fraction)
            np.testing.assert_allclose(states[:, offset + 12], fraction)
            expected = Rotation.from_euler("z", fraction[:, None] * 90, degrees=True).as_matrix()
            np.testing.assert_allclose(contract.pose_rotations(states[:, offset:offset + 12]), expected, atol=1e-6)
        with av.open(str(self.root / "synced/top.mp4")) as reader:
            frames = list(reader.decode(video=0))
            np.testing.assert_allclose([f.time for f in frames], np.arange(3) / 30)
            np.testing.assert_allclose([f.to_ndarray(format="rgb24").mean() for f in frames], [0, 100, 100], atol=4)
            self.assertEqual((frames[0].width, frames[0].height), (640, 480))

    def test_rejects_a_state_gap_without_publishing(self):
        self.write_states([0, 101])
        with self.assertRaisesRegex(ValueError, "100 ms"):
            self.run_sync()
        self.assertFalse((self.root / "synced/recording.json").exists())

    def test_rejects_a_camera_gap_without_publishing(self):
        self.write_states([0, 80, 160])
        self.camera_times([0, 140, 160])
        with self.assertRaisesRegex(ValueError, "50 ms"):
            self.run_sync()
        self.assertFalse((self.root / "synced/recording.json").exists())
