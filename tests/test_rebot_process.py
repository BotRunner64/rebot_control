"""Offline processing checks using real H.264 videos and asynchronous timestamps."""

import copy
import fcntl
import hashlib
from http.client import HTTPConnection
from http.server import ThreadingHTTPServer
import json
from pathlib import Path
import subprocess
import tempfile
import threading
import unittest
from unittest.mock import patch

import cv2
import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq

from rebot_process import Handler, Workspace, write_json
from rebot_recording import schemas


class ProcessingTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.raw = self.root / "raw"
        directory = self.raw / "episodes" / "continuous"
        (directory / "rgb").mkdir(parents=True)
        (directory / "state").mkdir()
        self.camera_times = np.array([20, 60, 130, 210, 300, 410, 530, 660, 800, 950, 1110, 1280], dtype=np.int64) * 1_000_000
        schema = schemas()
        for index, camera in enumerate(("camera_0", "camera_1")):
            path = directory / "rgb" / (camera + ".mp4")
            subprocess.run(["/usr/bin/python3", "-c", '''
import sys, gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst
Gst.init(None)
p = Gst.parse_launch("videotestsrc num-buffers=12 pattern=ball ! video/x-raw,width=160,height=120,framerate=10/1 ! x264enc tune=zerolatency bframes=0 ! video/x-h264,stream-format=avc,alignment=au ! mp4mux ! filesink name=output")
p.get_by_name("output").set_property("location", sys.argv[1])
p.set_state(Gst.State.PLAYING)
m = p.get_bus().timed_pop_filtered(15 * Gst.SECOND, Gst.MessageType.EOS | Gst.MessageType.ERROR)
p.set_state(Gst.State.NULL)
assert m and m.type == Gst.MessageType.EOS
''', str(path)], check=True, capture_output=True)
            pq.write_table(pa.Table.from_arrays([pa.array(range(12)), pa.array(self.camera_times + index * 5_000_000)], schema=schema["rgb"]), path.with_suffix(".parquet"))
        for side, offset in (("left", 0), ("right", 7_000_000)):
            times = np.array([0, 100, 220, 330, 440, 550, 680, 810, 970, 1120, 1290], dtype=np.int64) * 1_000_000 + offset
            for kind in ("eef", "gripper"):
                values = [[float(index)] * 12 for index in range(len(times))] if kind == "eef" else np.linspace(0, 1, len(times))
                pq.write_table(pa.Table.from_arrays([pa.array(times), pa.array(values, type=schema[kind][1].type)], schema=schema[kind]), directory / "state" / f"{side}_{kind}.parquet")
        (self.raw / "episodes.jsonl").write_text(json.dumps({"episode_id": "continuous", "cameras": ["camera_0", "camera_1"], "instructions": [{"start_ns": 0, "end_ns": 1297000000, "text": "__pending__"}]}) + "\n")
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
        records = [json.loads(line) for line in (self.workspace.output / "episodes.jsonl").read_text().splitlines()]
        self.assertEqual([r["episode_id"] for r in records], ["pick", "place"])
        for item, record in zip(self.plan["episodes"], records):
            destination = self.workspace.output / "episodes" / item["id"]
            self.assertEqual(record["cameras"], ["top", "left_wrist"])
            self.assertEqual(record["instructions"][0]["text"], item["instruction"])
            latest = 0
            for (session, group, name), times in self.workspace.streams.items():
                keep = (times >= item["start_ns"]) & (times < item["end_ns"])
                target = self.plan["camera_names"][name] if group == "rgb" else name
                table = pq.read_table(destination / group / (target + ".parquet"))
                original = pq.read_table(self.workspace.path(session, group, name + ".parquet"))
                self.assertEqual(table.schema, original.schema)
                np.testing.assert_array_equal(table["timestamp_ns"].to_numpy(), times[keep] - item["start_ns"])
                latest = max(latest, int(times[keep][-1]) - item["start_ns"])
                if group == "state":
                    self.assertEqual(table.column(1).to_pylist(), original.filter(pa.array(keep)).column(1).to_pylist())
                else:
                    np.testing.assert_array_equal(table["frame_index"].to_numpy(), np.arange(keep.sum()))
                    capture = cv2.VideoCapture(str(destination / group / (target + ".mp4")))
                    original_capture = cv2.VideoCapture(str(self.workspace.path(session, group, name + ".mp4")))
                    original_capture.set(cv2.CAP_PROP_POS_FRAMES, int(np.flatnonzero(keep)[0]))
                    count = 0
                    while True:
                        ok, frame = capture.read()
                        if not ok:
                            break
                        source_ok, source_frame = original_capture.read()
                        self.assertTrue(source_ok)
                        self.assertLess(np.mean(np.abs(frame.astype(float) - source_frame.astype(float))), 4)
                        count += 1
                    capture.release()
                    original_capture.release()
                    self.assertEqual(count, int(keep.sum()))
            self.assertEqual(record["instructions"][0]["end_ns"], latest)
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
        records = [json.loads(line) for line in (self.workspace.output / "episodes.jsonl").read_text().splitlines()]
        self.assertEqual([item["instructions"][0]["text"] for item in records], ["Repeat the same task"] * 2)

    def test_shared_instruction_and_reset_reference_are_validated(self):
        for instructions in ({"missing": "task"}, {"continuous": 5}):
            plan = copy.deepcopy(self.plan)
            plan["recording_instructions"] = instructions
            with self.assertRaisesRegex(ValueError, "recording instructions"):
                self.workspace.validate(plan)
        self.plan["recording_instructions"] = {"continuous": ""}
        with self.assertRaisesRegex(ValueError, "instruction"):
            self.workspace.validate(self.plan, exporting=True)
        for references in ({"missing": 0}, {"continuous": True}, {"continuous": -1}):
            self.plan["reset_references"] = references
            with self.assertRaisesRegex(ValueError, "reset reference"):
                self.workspace.validate(self.plan)

    def test_existing_raw_instruction_is_used_as_the_recording_default(self):
        self.workspace.records[0]["instructions"] = [{"text": "Fold the cloth"}]
        self.assertEqual(self.workspace.empty_plan()["recording_instructions"], {"continuous": "Fold the cloth"})
        self.workspace.records[0]["instructions"].append({"text": "A different task"})
        self.assertEqual(self.workspace.empty_plan()["recording_instructions"], {})

    def test_intentionally_empty_recording_remains_initialized_in_draft(self):
        plan = self.workspace.empty_plan()
        plan["initialized_sessions"] = ["continuous"]
        self.workspace.validate(plan)
        write_json(self.workspace.plan_path, plan)
        self.assertEqual(self.workspace.load_plan(), plan)
        for value in (["unknown"], ["continuous", "continuous"], {}, [["continuous"]]):
            plan["initialized_sessions"] = value
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.workspace.validate(plan)

    def test_auto_split_completion_survives_manual_edits_and_draft_reload(self):
        self.plan["auto_split_sessions"] = ["continuous"]
        self.plan["episodes"][0]["end_ns"] = 510_000_000
        write_json(self.workspace.plan_path, self.workspace.validate(self.plan))
        restored = self.workspace.load_plan()
        self.assertEqual(restored["auto_split_sessions"], ["continuous"])
        self.assertEqual(restored["episodes"][0]["end_ns"], 510_000_000)
        for value in (["unknown"], ["continuous", "continuous"], {}, [True]):
            restored["auto_split_sessions"] = value
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.workspace.validate(restored)

    def test_invalid_ranges_names_paths_and_instructions_are_rejected(self):
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
        for output in (self.raw, self.raw / "processed", self.root):
            with self.assertRaises(ValueError):
                Workspace(self.raw, output, self.root / "draft.json")
        with self.assertRaises(ValueError):
            Workspace(self.raw, self.root / "out", self.raw / "draft.json")

    def test_export_failure_never_publishes_partial_dataset(self):
        before = self.digest()
        with patch("rebot_process.subprocess.run", return_value=subprocess.CompletedProcess([], 2, "", "Encoder failed")):
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
        with self.assertRaisesRegex(ValueError, "no samples"):
            self.workspace.validate(self.plan, exporting=True)

    def test_concurrent_export_to_same_destination_is_rejected(self):
        path = self.root / ".processed.processing.lock"
        with path.open("a") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            with self.assertRaisesRegex(ValueError, "Another process"):
                self.workspace.export(self.plan)
        self.assertFalse(self.workspace.output.exists())

    def test_preview_decodes_native_timestamp_frame(self):
        image = self.workspace.frame("continuous", "camera_0", 210_000_000)
        self.assertTrue(image.startswith(b"\xff\xd8"))
        with self.assertRaisesRegex(ValueError, "not started"):
            self.workspace.frame("continuous", "camera_0", 0)
        for capture in self.workspace.captures.values():
            capture.release()

    def test_native_video_stream_supports_seeking_without_changing_source(self):
        before = self.digest()
        server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        server.workspace = self.workspace
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        connection = HTTPConnection(*server.server_address, timeout=5)
        try:
            url = "/api/video?session=continuous&camera=camera_0"
            source = self.workspace.path("continuous", "rgb", "camera_0.mp4").read_bytes()
            for method, requested, status, content in (
                    ("GET", None, 200, source),
                    ("HEAD", None, 200, b""),
                    ("GET", "bytes=0-31", 206, source[:32]),
                    ("GET", "bytes=32-", 206, source[32:]),
                    ("GET", "bytes=-32", 206, source[-32:]),
                    ("GET", f"bytes={len(source) - 4}-{len(source) + 99}", 206, source[-4:]),
                    ("GET", "bytes=999999999-", 416, b""),
                    ("GET", "bytes=3-1", 416, b""),
                    ("GET", "bytes=-0", 416, b""),
                    ("GET", "bytes=", 416, b""),
                    ("GET", "bytes=0-1,3-4", 416, b"")):
                with self.subTest(method=method, range=requested):
                    connection.request(method, url, headers={"Range": requested} if requested else {})
                    response = connection.getresponse()
                    self.assertEqual(response.status, status)
                    self.assertEqual(response.read(), content)
                    if status != 416:
                        self.assertEqual(response.getheader("Content-Type"), "video/mp4")
                        self.assertEqual(response.getheader("Accept-Ranges"), "bytes")
                    if status == 206:
                        self.assertTrue(response.getheader("Content-Range").endswith(f"/{len(source)}"))
                    if status == 416:
                        self.assertEqual(response.getheader("Content-Range"), f"bytes */{len(source)}")
            connection.request("GET", "/api/video?session=continuous&camera=../camera_0")
            self.assertEqual(connection.getresponse().status, 400)
            self.assertEqual(before, self.digest())
        finally:
            connection.close()
            server.shutdown(); server.server_close(); thread.join()


if __name__ == "__main__":
    unittest.main()
