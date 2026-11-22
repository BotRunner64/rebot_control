"""Offline processing checks using real H.264 videos and synchronized frame tables."""

import copy
import fcntl
import hashlib
import json
from pathlib import Path
import shutil
import tempfile
import unittest
from unittest.mock import patch

import av
import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq

from rebot.data.process import Workspace, write_json
from rebot.data import contract


class ProcessingTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.raw = self.root / "raw"
        directory = self.raw / "recordings" / "continuous"
        directory.mkdir(parents=True)
        count = 40
        pose = np.tile([0., 0., 0., *np.eye(3).ravel(), .5], (count, 2))
        pose[:, 0] = np.arange(count)
        states = pa.array(pose.tolist(), type=contract.STATE)
        pq.write_table(pa.Table.from_arrays([states, pa.array(["__pending__"] * count)], schema=contract.FRAME_SCHEMA),
                       directory / "frames.parquet")
        cameras = ["camera_0", "camera_1"]
        (directory / "recording.json").write_text(json.dumps({"frames": count, "cameras": cameras,
                                                              "instruction": "__pending__"}))
        with av.open(str(directory / "camera_0.mp4"), "w") as writer:
            stream = writer.add_stream("libx264", rate=30)
            stream.width, stream.height, stream.pix_fmt = 640, 480, "yuv420p"
            for k in range(count):
                frame = av.VideoFrame.from_ndarray(np.full((480, 640, 3), k * 5, np.uint8), format="rgb24")
                for packet in stream.encode(frame):
                    writer.mux(packet)
            for packet in stream.encode():
                writer.mux(packet)
        shutil.copyfile(directory / "camera_0.mp4", directory / "camera_1.mp4")
        self.workspace = Workspace(self.raw, self.root / "processed", self.root / "draft.json")
        self.addCleanup(self.workspace.previews.close)
        self.plan = self.workspace.empty_plan()
        self.plan["camera_names"] = {"camera_0": "top", "camera_1": "left_wrist"}
        self.plan["episodes"] = [{"id": "pick", "session": "continuous", "start_ns": 125_000_000,
                                  "end_ns": 530_000_000, "instruction": "Pick up the cup"},
                                 {"id": "place", "session": "continuous", "start_ns": 660_000_000,
                                  "end_ns": 1_200_000_000, "instruction": "Place the cup on the tray"}]

    def digest(self):
        return {str(path.relative_to(self.raw)): hashlib.sha256(path.read_bytes()).hexdigest()
                for path in self.raw.rglob("*") if path.is_file()}

    def test_real_export_preserves_samples_sync_and_exact_video_frames(self):
        before = self.digest()
        result = self.workspace.export(self.plan)
        self.assertEqual(result["completed"], 2)
        dataset = contract.Dataset(self.workspace.output / "top+left_wrist_640x480")
        self.assertEqual([r["episode_id"] for r in dataset.episodes], ["pick", "place"])
        self.assertNotIn("action", dataset.info["features"])
        for item, record in zip(self.plan["episodes"], dataset.episodes):
            times = self.workspace.times[item["session"]]
            indices = np.flatnonzero((times >= item["start_ns"]) & (times < item["end_ns"]))
            states, tasks = dataset.frames(record)
            np.testing.assert_array_equal(states[:, 0], indices)
            self.assertEqual(tasks, [item["instruction"]] * len(indices))
            for camera in dataset.cameras:
                path, offset = dataset.video(record, camera)
                self.assertEqual(offset, 0)
                with av.open(str(path)) as reader:
                    frames = list(reader.decode(video=0))
                    self.assertEqual(reader.streams.video[0].average_rate, 30)
                np.testing.assert_allclose([f.time for f in frames], np.arange(len(indices)) / 30)
                np.testing.assert_allclose([f.to_ndarray(format="rgb24").mean() for f in frames], indices * 5, atol=4)
        self.assertEqual(json.loads((self.workspace.output / "processing.json").read_text()), self.plan)
        self.assertEqual(before, self.digest())

    def test_draft_round_trip_allows_unfinished_instructions(self):
        self.plan["episodes"][0]["instruction"] = ""
        self.workspace.validate(self.plan)
        write_json(self.workspace.plan_path, self.plan)
        self.assertEqual(self.workspace.load_plan(), self.plan)
        with self.assertRaisesRegex(ValueError, "instruction"):
            self.workspace.export(self.plan)
        self.assertFalse(self.workspace.output.exists())

    def test_recording_instruction_applies_to_all_exported_episodes(self):
        self.plan["recording_instructions"] = {"continuous": "Repeat the same task"}
        self.workspace.export(self.plan)
        dataset = contract.Dataset(self.workspace.output / "top+left_wrist_640x480")
        self.assertEqual(dataset.tasks, {0: "Repeat the same task"})

    def test_different_camera_sets_publish_separate_layouts(self):
        directory = self.raw / "recordings" / "second"
        shutil.copytree(self.raw / "recordings" / "continuous", directory)
        (directory / "recording.json").write_text(json.dumps({"frames": 40, "cameras": ["camera_0"],
                                                              "instruction": "single view"}))
        workspace = Workspace(self.raw, self.workspace.output, self.workspace.plan_path)
        self.addCleanup(workspace.previews.close)
        self.plan["episodes"][1]["session"] = "second"
        workspace.export(self.plan)
        for layout, episode_id in (("top_640x480", "place"), ("top+left_wrist_640x480", "pick")):
            dataset = contract.Dataset(workspace.output / layout)
            self.assertEqual([r["episode_id"] for r in dataset.episodes], [episode_id])
            self.assertEqual(dataset.episodes[0]["episode_index"], 0)

    def test_invalid_plans_and_paths_are_rejected(self):
        for key, value in (("start_ns", -1), ("start_ns", 125000000.0), ("end_ns", 9_000_000_000),
                           ("end_ns", 1), ("id", "../escape"), ("session", "missing"), ("instruction", "__pending__")):
            plan = copy.deepcopy(self.plan)
            plan["episodes"][0][key] = value
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                self.workspace.validate(plan, exporting=True)
        for value in ("TOP", "../top", "left_wrist", ""):
            plan = copy.deepcopy(self.plan)
            plan["camera_names"]["camera_0"] = value
            with self.subTest(camera=value), self.assertRaises(ValueError):
                self.workspace.validate(plan)
        for instructions in ({"missing": "task"}, {"continuous": 5}):
            plan = copy.deepcopy(self.plan)
            plan["recording_instructions"] = instructions
            with self.assertRaisesRegex(ValueError, "recording instructions"):
                self.workspace.validate(plan)
        plan = copy.deepcopy(self.plan)
        plan["recording_instructions"] = {"continuous": ""}
        with self.assertRaisesRegex(ValueError, "instruction"):
            self.workspace.validate(plan, exporting=True)
        for references in ({"missing": 0}, {"continuous": True}, {"continuous": -1}):
            plan["reset_references"] = references
            with self.assertRaisesRegex(ValueError, "reset reference"):
                self.workspace.validate(plan)
        for output in (self.raw, self.raw / "processed", self.root):
            with self.assertRaises(ValueError):
                Workspace(self.raw, output, self.root / "draft.json")
        with self.assertRaises(ValueError):
            Workspace(self.raw, self.root / "out", self.raw / "draft.json")

    def test_export_failure_never_publishes_partial_dataset(self):
        before = self.digest()
        with patch("rebot.data.synchronous.contract.extract_video_segment", side_effect=RuntimeError("Encoder failed")):
            with self.assertRaisesRegex(RuntimeError, "Encoder failed"):
                self.workspace.export(self.plan)
        self.assertFalse(self.workspace.output.exists())
        self.assertEqual(list(self.root.glob(".processed-*")), [])
        self.assertEqual(before, self.digest())

    def test_existing_output_and_empty_stream_ranges_are_rejected(self):
        self.workspace.output.mkdir()
        with self.assertRaisesRegex(ValueError, "already exists"):
            self.workspace.export(self.plan)
        self.plan["episodes"][0].update(start_ns=131_000_000, end_ns=132_000_000)
        with self.assertRaisesRegex(ValueError, "at least two"):
            self.workspace.validate(self.plan, exporting=True)

    def test_concurrent_export_to_same_destination_is_rejected(self):
        path = self.root / ".processed.processing.lock"
        with path.open("a") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            with self.assertRaisesRegex(ValueError, "Another process"):
                self.workspace.export(self.plan)
        self.assertFalse(self.workspace.output.exists())


if __name__ == "__main__":
    unittest.main()
