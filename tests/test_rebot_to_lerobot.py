"""Real-video integration checks for asynchronous reBot to LeRobot v3 conversion."""

import fcntl
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

import av
import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.datasets.video_utils import _CameraEncoderThread

from rebot_to_lerobot import NS, STATE_NAMES, VECTOR_NAMES, convert, nearest_indices, read_records, synchronize


class ConversionTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.source, self.output = self.root / "processed", self.root / "lerobot"
        self.records = []
        self.camera_times = np.array([5, 37, 70, 101, 136, 170, 204, 237, 270, 305, 339, 373, 405], dtype=np.int64) * 1_000_000
        for number in range(2):
            name = f"pick_{number}"
            directory = self.source / "episodes" / name
            (directory / "state").mkdir(parents=True)
            (directory / "rgb").mkdir()
            for index, stream in enumerate(STATE_NAMES):
                times = np.arange(41, dtype=np.int64) * 10_000_000 + index * 1_000_000
                if stream.endswith("eef"):
                    pose = np.tile(np.r_[np.zeros(3), np.eye(3).flatten(order="F")], (len(times), 1))
                    pose[:, 0] = np.arange(len(times)) / 1000 + index / 10 + number
                    values = pa.array(pose.tolist(), type=pa.list_(pa.float64(), 12))
                    key = "pose"
                else:
                    values = pa.array(np.arange(len(times)) / 40, type=pa.float32())
                    key = "openness"
                pq.write_table(pa.table({"timestamp_ns": times, key: values}), directory / "state" / f"{stream}.parquet")
            for index, camera in enumerate(("top", "left_wrist")):
                path = directory / "rgb" / f"{camera}.mp4"
                with av.open(str(path), "w") as container:
                    stream = container.add_stream("libx264", rate=30)
                    stream.width, stream.height, stream.pix_fmt = 64, 48, "yuv420p"
                    stream.options = {"crf": "10", "bf": "0"}
                    for frame_index in range(len(self.camera_times)):
                        color = frame_index * 16 + index * 20 + number * 10
                        image = np.full((48, 64, 3), color, dtype=np.uint8)
                        frame = av.VideoFrame.from_ndarray(image, format="rgb24")
                        for packet in stream.encode(frame):
                            container.mux(packet)
                    for packet in stream.encode():
                        container.mux(packet)
                pq.write_table(pa.table({"frame_index": np.arange(len(self.camera_times), dtype=np.int64),
                                         "timestamp_ns": self.camera_times + index * 4_000_000}), path.with_suffix(".parquet"))
            self.records.append({"episode_id": name, "cameras": ["top", "left_wrist"],
                                 "instructions": [{"text": f"Pick up cup {number}", "start_ns": 0, "end_ns": 410_000_000}]})
        self.write_records()
        (self.source / "processing.json").write_text('{"version": 1}\n')

    def write_records(self):
        (self.source / "episodes.jsonl").write_text("".join(json.dumps(r) + "\n" for r in self.records))

    def digest(self):
        return {str(path.relative_to(self.source)): hashlib.sha256(path.read_bytes()).hexdigest()
                for path in self.source.rglob("*") if path.is_file()}

    def test_cli_export_round_trip_preserves_alignment_actions_and_episode_boundaries(self):
        before = self.digest()
        result = subprocess.run([sys.executable, str(Path(__file__).resolve().parents[1] / "rebot_to_lerobot.py"),
                                 "--dataset", str(self.source), "--output", str(self.output),
                                 "--repo-id", "local/test_rebot"], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual({p.name for p in self.output.iterdir()}, {"data", "meta", "videos"})
        dataset = LeRobotDataset("local/test_rebot", root=self.output, video_backend="pyav")
        self.assertEqual(dataset.meta.fps, 30)
        self.assertEqual(dataset.meta.features["observation.state"]["names"], VECTOR_NAMES)
        self.assertEqual(dataset.meta.info["codebase_version"], "v3.0")
        self.assertEqual(dataset.meta.total_episodes, 2)
        self.assertEqual(dataset.meta.total_tasks, 2)
        offset = 0
        for number, record in enumerate(self.records):
            # Common origin is the later camera start, rather than independently zeroed streams.
            targets = 9_000_000 + np.arange(12, dtype=np.int64) * NS // 30
            self.assertEqual(dataset.meta.episodes[number]["length"], len(targets) - 1)
            for tick, target in enumerate(targets[:-1]):
                expected_state, expected_action = [], []
                action_target = targets[tick + 1]
                for stream in STATE_NAMES:
                    table = pq.read_table(self.source / "episodes" / record["episode_id"] / "state" / f"{stream}.parquet")
                    times = table["timestamp_ns"].to_numpy()
                    values = table.column(1).to_pylist()
                    obs_index = int(np.argmin(np.abs(times - target)))
                    action_index = int(np.argmin(np.abs(times - action_target)))
                    expected_state.extend(np.atleast_1d(values[obs_index]))
                    expected_action.extend(np.atleast_1d(values[action_index]))
                sample = dataset[offset + tick]
                np.testing.assert_array_equal(sample["observation.state"].numpy(), np.asarray(expected_state, dtype=np.float32))
                np.testing.assert_array_equal(sample["action"].numpy(), np.asarray(expected_action, dtype=np.float32))
                self.assertAlmostEqual(sample["timestamp"].item(), tick / 30, places=6)
                self.assertEqual(sample["task"], f"Pick up cup {number}")
                for camera_number, camera in enumerate(record["cameras"]):
                    selected = int(np.argmin(np.abs(self.camera_times + camera_number * 4_000_000 - target)))
                    mean = sample[f"observation.images.{camera}"].float().mean().item() * 255
                    self.assertLess(abs(mean - (selected * 16 + camera_number * 20 + number * 10)), 5)
            offset += len(targets) - 1
        chunks = LeRobotDataset("local/test_rebot", root=self.output, video_backend="pyav",
                                delta_timestamps={"action": [0, 1 / 30, 2 / 30]})
        first_length = dataset.meta.episodes[0]["length"]
        sample = chunks[first_length - 1]
        np.testing.assert_array_equal(sample["action"].numpy(), np.tile(dataset[first_length - 1]["action"].numpy(), (3, 1)))
        np.testing.assert_array_equal(sample["action_is_pad"].numpy(), [False, True, True])
        self.assertEqual(before, self.digest())

    def test_nearest_ties_duplicates_and_exact_matches(self):
        np.testing.assert_array_equal(nearest_indices(np.array([0, 10, 10, 20]), np.array([0, 5, 10, 15, 20])), [0, 0, 1, 1, 3])

    def test_gap_aborts_without_publishing_or_changing_source(self):
        path = self.source / "episodes" / "pick_0" / "rgb" / "top.parquet"
        table = pq.read_table(path)
        times = np.r_[np.arange(6), np.arange(7) + 390].astype(np.int64) * 1_000_000
        pq.write_table(table.set_column(1, "timestamp_ns", pa.array(times)), path)
        before = self.digest()
        with self.assertRaisesRegex(ValueError, "Synchronization gap"):
            convert(self.source, self.output)
        self.assertFalse(self.output.exists())
        self.assertEqual(list(self.root.glob(".lerobot-*")), [])
        self.assertEqual(before, self.digest())

    def test_truncated_source_video_is_rejected(self):
        path = self.source / "episodes" / "pick_0" / "rgb" / "top.parquet"
        table = pq.read_table(path)
        table = pa.concat_tables([table, pa.table({"frame_index": [len(table)], "timestamp_ns": [440_000_000]})])
        pq.write_table(table, path)
        with self.assertRaisesRegex(ValueError, "Video ended before indexed frame"):
            convert(self.source, self.output)
        self.assertFalse(self.output.exists())
        self.assertEqual(list(self.root.glob(".lerobot-*")), [])

    def test_writer_failure_cleans_up_and_keeps_source(self):
        before = self.digest()
        with patch.object(LeRobotDataset, "save_episode", side_effect=RuntimeError("Encoder failed")):
            with self.assertRaisesRegex(RuntimeError, "Encoder failed"):
                convert(self.source, self.output)
        self.assertFalse(self.output.exists())
        self.assertEqual(list(self.root.glob(".lerobot-*")), [])
        self.assertEqual(before, self.digest())

    def test_slow_video_consumers_do_not_drop_frames_or_write_pngs(self):
        original_run = _CameraEncoderThread.run

        def delayed_run(thread):
            time.sleep(0.35)
            original_run(thread)

        # A one-frame queue plus a consumer slower than the recording API's drop
        # timeout forces real backpressure through the actual H.264 encoders.
        from rebot_to_lerobot import enable_blocking_encoding

        def small_queue(writer, fps):
            enable_blocking_encoding(writer, fps, queue_size=1)

        with patch("rebot_to_lerobot.enable_blocking_encoding", side_effect=small_queue), \
                patch.object(_CameraEncoderThread, "run", delayed_run), \
                patch.object(LeRobotDataset, "_save_image", side_effect=AssertionError("Unexpected PNG write")):
            result = convert(self.source, self.output)
        self.assertFalse(list(self.output.rglob("*.png")))
        dataset = LeRobotDataset("local/rebot", root=self.output, video_backend="pyav")
        self.assertEqual(len(dataset), result["total_frames"])

    def test_failed_video_consumer_aborts_without_hanging_or_publishing(self):
        def fail(thread):
            thread.result_queue.put(("error", "Injected encoder failure"))

        with patch.object(_CameraEncoderThread, "run", fail):
            with self.assertRaisesRegex(RuntimeError, "Injected encoder failure"):
                convert(self.source, self.output)
        self.assertFalse(self.output.exists())
        self.assertEqual(list(self.root.glob(".lerobot-*")), [])

    def test_existing_output_nested_paths_and_concurrent_writer_are_rejected(self):
        self.output.mkdir()
        sentinel = self.output / "keep.txt"
        sentinel.write_text("keep")
        with self.assertRaisesRegex(ValueError, "already exists"):
            convert(self.source, self.output)
        self.assertEqual(sentinel.read_text(), "keep")
        for output in (self.source, self.source / "nested", self.root):
            with self.subTest(output=output), self.assertRaisesRegex(ValueError, "non-nested"):
                convert(self.source, output)
        with (self.root / ".other.lerobot.lock").open("a") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            with self.assertRaisesRegex(ValueError, "Another converter"):
                convert(self.source, self.root / "other")

    def test_invalid_metadata_is_rejected(self):
        for value in ("", "__pending__"):
            self.records[0]["instructions"][0]["text"] = value
            self.write_records()
            with self.assertRaisesRegex(ValueError, "instruction"):
                read_records(self.source)
        self.records[0]["instructions"][0]["text"] = "Pick up cup"
        self.records[0]["episode_id"] = "../escape"
        self.write_records()
        with self.assertRaisesRegex(ValueError, "Episode IDs"):
            read_records(self.source)

    def test_invalid_state_and_short_overlap_are_rejected(self):
        path = self.source / "episodes" / "pick_0" / "state" / "left_gripper.parquet"
        original = pq.read_table(path)
        values = original["openness"].to_numpy().copy()
        values[2] = np.nan
        pq.write_table(original.set_column(1, "openness", pa.array(values)), path)
        with self.assertRaisesRegex(ValueError, "non-finite"):
            synchronize(self.source, self.records[0], 30, 50_000_000)
        pq.write_table(original.slice(1, 2), path)
        with self.assertRaisesRegex(ValueError, "at least two"):
            synchronize(self.source, self.records[0], 30, 50_000_000)

    def test_invalid_conversion_options_are_rejected(self):
        for kwargs in ({"fps": 0}, {"fps": 30.5}, {"max_delta_ms": float("nan")}, {"max_delta_ms": -1},
                       {"repo_id": "../escape"}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                convert(self.source, self.output, **kwargs)


if __name__ == "__main__":
    unittest.main()
