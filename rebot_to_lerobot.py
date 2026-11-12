#!/usr/bin/env python3
"""Synchronize processed reBot episodes and export a local LeRobot v3 dataset."""

import argparse
from contextlib import ExitStack, contextmanager
from dataclasses import dataclass
import fcntl
import json
from pathlib import Path
import queue
import re
import shutil
import sys
import tempfile
import threading

import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq
from tqdm import tqdm


NS = 1_000_000_000
STATE_NAMES = ("left_eef", "left_gripper", "right_eef", "right_gripper")
POSE_NAMES = ["x", "y", "z"] + [f"R{row}{col}" for col in range(3) for row in range(3)]
VECTOR_NAMES = [f"{side}_{name}" for side in ("left", "right")
                for name in [*POSE_NAMES, "gripper_openness"]]


@contextmanager
def phase(description):
    """Show elapsed time for library operations without measurable intermediate progress."""
    with tqdm(desc=description, bar_format="{desc} [{elapsed}]", position=1,
              leave=False, dynamic_ncols=True) as status:
        stopped = threading.Event()

        def refresh():
            while not stopped.wait(0.5):
                status.refresh()

        thread = threading.Thread(target=refresh, daemon=True)
        if not status.disable:
            thread.start()
        try:
            yield
        finally:
            stopped.set()
            if thread.is_alive():
                thread.join()


def source_path(root, *parts):
    path = root.joinpath(*parts).resolve()
    if root not in path.parents or not path.is_file():
        raise ValueError(f"Missing source file or path outside dataset: {'/'.join(parts)}")
    return path


def read_records(root):
    records = [json.loads(line) for line in source_path(root, "episodes.jsonl").read_text(encoding="utf-8").splitlines()
               if line.strip()]
    if not records:
        raise ValueError("No episodes found.")
    ids, expected_cameras = set(), None
    for record in records:
        name, cameras = record.get("episode_id"), record.get("cameras")
        if not isinstance(name, str) or re.fullmatch(r"[a-zA-Z0-9_][a-zA-Z0-9_-]*", name) is None or name in ids:
            raise ValueError("Episode IDs must be valid and unique.")
        ids.add(name)
        if (not isinstance(cameras, list) or not cameras
                or any(not isinstance(c, str) or re.fullmatch(r"[a-z0-9_]+", c) is None for c in cameras)
                or len(set(cameras)) != len(cameras)):
            raise ValueError(f"Invalid camera names: {name}")
        if expected_cameras is not None and set(cameras) != set(expected_cameras):
            raise ValueError("Every episode must have the same camera names.")
        expected_cameras = cameras
        instructions = record.get("instructions", [])
        if len(instructions) != 1 or not isinstance(instructions[0], dict):
            raise ValueError(f"Expected one instruction per processed episode: {name}")
        item = instructions[0]
        text = item.get("text")
        if not isinstance(text, str) or not text.strip() or text.strip() == "__pending__":
            raise ValueError(f"Complete the instruction before converting: {name}")
        if (type(item.get("start_ns")) is not int or type(item.get("end_ns")) is not int
                or not 0 <= item["start_ns"] < item["end_ns"]):
            raise ValueError(f"Invalid instruction time range: {name}")
    return records


def read_stream(path, kind):
    table = pq.read_table(path)
    field = table.schema.field("timestamp_ns")
    if field.type != pa.int64() or table["timestamp_ns"].null_count:
        raise ValueError(f"Expected non-null int64 timestamp_ns: {path}")
    times = table["timestamp_ns"].to_numpy()
    if not len(times) or np.any(times < 0) or np.any(times[1:] < times[:-1]):
        raise ValueError(f"Empty, negative or unordered timestamps: {path}")
    key = {"eef": "pose", "gripper": "openness", "rgb": "frame_index"}[kind]
    column = table[key]
    if column.null_count:
        raise ValueError(f"Null values in {key}: {path}")
    if kind == "rgb":
        values = column.to_numpy()
        if column.type != pa.int64() or not np.array_equal(values, np.arange(len(times))):
            raise ValueError(f"Frame indices must be contiguous from zero: {path}")
    else:
        values = np.asarray(column.to_pylist(), dtype=np.float64)
        if values.shape != ((len(times), 12) if kind == "eef" else (len(times),)) or not np.all(np.isfinite(values)):
            raise ValueError(f"Invalid {key} shape or non-finite values: {path}")
        if kind == "gripper" and (np.any(values < 0) or np.any(values > 1)):
            raise ValueError(f"Gripper openness must be in [0, 1]: {path}")
        with np.errstate(over="ignore"):
            values = values.astype(np.float32)
        if not np.all(np.isfinite(values)):
            raise ValueError(f"Values cannot be represented as float32: {path}")
    return times, values


def nearest_indices(times, targets):
    """Select nearest samples; ties choose the earlier timestamp, duplicates the first row."""
    after = np.searchsorted(times, targets, side="left").clip(0, len(times) - 1)
    before = (after - 1).clip(0, len(times) - 1)
    chosen = np.where(np.abs(times[before] - targets) <= np.abs(times[after] - targets), before, after)
    return np.searchsorted(times, times[chosen], side="left")


@dataclass
class Episode:
    targets: np.ndarray
    indices: dict
    states: np.ndarray
    source_counts: dict


def synchronize(root, record, fps, max_delta_ns):
    streams = {}
    name = record["episode_id"]
    for stream in STATE_NAMES:
        streams[stream] = read_stream(source_path(root, "episodes", name, "state", stream + ".parquet"),
                                      "eef" if stream.endswith("eef") else "gripper")
    for camera in record["cameras"]:
        streams[f"rgb/{camera}"] = read_stream(source_path(root, "episodes", name, "rgb", camera + ".parquet"), "rgb")
        source_path(root, "episodes", name, "rgb", camera + ".mp4")
    start = max(int(times[0]) for times, _ in streams.values())
    end = min(int(times[-1]) for times, _ in streams.values())
    if end <= start:
        raise ValueError(f"No common stream time range: {name}")
    count = (end - start) * fps // NS + 1
    ticks = np.arange(count, dtype=np.int64)
    # Integer arithmetic avoids accumulated rounding drift at rates such as 30 Hz.
    targets = start + (ticks // fps) * NS + (ticks % fps) * NS // fps
    if len(targets) < 2:
        raise ValueError(f"Episode needs at least two synchronized ticks for next-state actions: {name}")
    instruction = record["instructions"][0]
    if instruction["start_ns"] > int(targets[0]) or instruction["end_ns"] < int(targets[-1]):
        raise ValueError(f"Instruction does not cover the synchronized episode: {name}")
    indices = {}
    for stream, (times, _) in streams.items():
        selected = nearest_indices(times, targets)
        delta = np.abs(times[selected] - targets)
        if np.any(delta > max_delta_ns):
            bad = int(np.flatnonzero(delta > max_delta_ns)[0])
            raise ValueError(f"Synchronization gap in {name}/{stream} at {targets[bad]} ns: "
                             f"{delta[bad] / 1e6:.3f} ms > {max_delta_ns / 1e6:.3f} ms")
        indices[stream] = selected
    states = np.concatenate([streams[key][1][indices[key]].reshape(len(targets), -1) for key in STATE_NAMES], axis=1)
    return Episode(targets, indices, states,
                   {key: len(times) for key, (times, _) in streams.items()})


class VideoReader:
    """Decode display-order frame indices sequentially, retaining at most one RGB frame."""

    def __init__(self, path, expected_count):
        import av
        self.path, self.expected_count = path, expected_count
        self.container = av.open(str(path))
        try:
            if len(self.container.streams.video) != 1:
                raise ValueError(f"Expected one video stream: {path}")
            self.stream = self.container.streams.video[0]
            self.stream.thread_type = "AUTO"
            self.stream.codec_context.thread_count = 2
            self.frames = iter(self.container.decode(self.stream))
            self.index, self.image, self.shape = -1, None, None
            self.get(0)
            if self.shape[0] % 2 or self.shape[1] % 2:
                raise ValueError(f"H.264 output requires even frame dimensions: {path}")
        except BaseException:
            self.container.close()
            raise

    def get(self, index):
        if index < self.index:
            raise ValueError("Video frame requests must be nondecreasing.")
        while self.index < index:
            try:
                frame = next(self.frames)
            except StopIteration as exc:
                raise ValueError(f"Video ended before indexed frame {index}: {self.path}") from exc
            self.index += 1
            self.image = frame.to_ndarray(format="rgb24")
            if self.shape is not None and self.image.shape != self.shape:
                raise ValueError(f"Video dimensions change within the episode: {self.path}")
            self.shape = self.image.shape
        return self.image

    def verify_end(self):
        self.get(self.expected_count - 1)
        if next(self.frames, None) is not None:
            raise ValueError(f"Video has more frames than its timestamp index: {self.path}")

    def close(self):
        self.container.close()


def dataset_class():
    try:
        from lerobot.datasets.lerobot_dataset import CODEBASE_VERSION, LeRobotDataset
    except ImportError as exc:
        raise RuntimeError("Activate rebot_control and install the conversion dependencies: "
                           "python -m pip install -r requirements-lerobot.txt; "
                           "python -m pip install --no-deps lerobot==0.4.4 (see docs/lerobot.md)") from exc
    if CODEBASE_VERSION != "v3.0":
        raise RuntimeError(f"Expected a LeRobot v3.0 writer, found {CODEBASE_VERSION}. Use requirements-lerobot.txt.")
    return LeRobotDataset


def enable_blocking_encoding(writer, fps, queue_size=4):
    """Use LeRobot 0.4.4's camera encoders with bounded queues and no frame dropping."""
    from lerobot.datasets.video_utils import StreamingVideoEncoder

    class BlockingVideoEncoder(StreamingVideoEncoder):
        def _enqueue(self, video_key, pending):
            # These internals belong to the pinned LeRobot 0.4.4 adapter. Unlike the
            # recording API, offline conversion waits for capacity instead of dropping frames.
            while self._threads[video_key].is_alive():
                try:
                    self._frame_queues[video_key].put(pending, timeout=0.2)
                    return
                except queue.Full:
                    continue
            try:
                _, detail = self._result_queues[video_key].get_nowait()
            except queue.Empty:
                detail = "Encoder thread stopped unexpectedly"
            raise RuntimeError(f"Video encoder failed for {video_key}: {detail}")

        def feed_frame(self, video_key, image):
            if not self._episode_active:
                raise RuntimeError("No active video encoding episode.")
            self._enqueue(video_key, image.copy())

        def finish_episode(self):
            if not self._episode_active:
                raise RuntimeError("No active video encoding episode.")
            # Sentinels also need checked, bounded waits: a dead consumer must not
            # leave save_episode() blocked forever on a full frame queue.
            for key in self._threads:
                self._enqueue(key, None)
            results = {}
            for key, thread in self._threads.items():
                thread.join(timeout=120)
                if thread.is_alive():
                    raise RuntimeError(f"Timed out finishing video encoder: {key}")
                try:
                    status, detail = self._result_queues[key].get(timeout=5)
                except queue.Empty as exc:
                    raise RuntimeError(f"Video encoder returned no result: {key}") from exc
                if status != "ok":
                    raise RuntimeError(f"Video encoder failed for {key}: {detail}")
                results[key] = (self._video_paths[key], detail)
            self._cleanup()
            self._episode_active = False
            return results

    if writer._streaming_encoder is not None:
        writer._streaming_encoder.close()
    writer._streaming_encoder = BlockingVideoEncoder(
        fps=fps, vcodec="h264", queue_maxsize=queue_size, encoder_threads=2)


def validate_export(root, repo_id, fps, lengths, cameras, cls):
    """Check actual output videos and reopen the dataset with temporal action queries."""
    import av
    with phase("Loading exported dataset"):
        dataset = cls(repo_id=repo_id, root=root, video_backend="pyav",
                      delta_timestamps={"action": [0.0, 1 / fps]})
    if dataset.meta.info["codebase_version"] != "v3.0" or len(dataset) != sum(lengths):
        raise ValueError("Exported dataset version or frame count is incorrect.")
    if dataset.meta.total_episodes != len(lengths):
        raise ValueError("Exported episode count is incorrect.")
    expected = {}
    offset = 0
    for index, length in tqdm(enumerate(lengths), total=len(lengths), desc="Checking episode boundaries",
                              unit="episode", position=1, leave=False, dynamic_ncols=True):
        for camera in cameras:
            key = f"observation.images.{camera}"
            path = root / dataset.meta.get_video_file_path(index, key)
            expected[path] = expected.get(path, 0) + length
        for row in {offset, offset + length - 1}:
            sample = dataset[row]
            if tuple(sample["observation.state"].shape) != (26,) or tuple(sample["action"].shape) != (2, 26):
                raise ValueError("Exported state or action shape is incorrect.")
            if int(sample["episode_index"].item()) != index:
                raise ValueError("Exported episode boundaries are incorrect.")
        offset += length
    for path, count in expected.items():
        with av.open(str(path)) as container:
            stream = container.streams.video[0]
            decoded = 0
            for frame in tqdm(container.decode(stream), total=count,
                              desc=f"Validating {path.parent.parent.name}/{path.stem}",
                              unit="frame", position=1, leave=False, dynamic_ncols=True):
                if frame.pts is None or abs(float(frame.pts * frame.time_base) - decoded / fps) > 1e-4:
                    raise ValueError(f"Output video is not synchronized at {fps} FPS: {path}")
                decoded += 1
            if decoded != count:
                raise ValueError(f"Output video frame count mismatch: {decoded} != {count}: {path}")


def convert(dataset, output, repo_id="local/rebot", fps=30, max_delta_ms=50.0):
    source, output = Path(dataset).resolve(), Path(output).resolve()
    if source == output or source in output.parents or output in source.parents:
        raise ValueError("Source and output must be separate, non-nested directories.")
    if type(fps) is not int or not 1 <= fps <= 1000:
        raise ValueError("FPS must be an integer between 1 and 1000.")
    if not np.isfinite(max_delta_ms) or max_delta_ms <= 0:
        raise ValueError("Maximum synchronization delta must be finite and positive.")
    if not isinstance(repo_id, str) or re.fullmatch(r"[a-zA-Z0-9_-]+/[a-zA-Z0-9_.-]+", repo_id) is None:
        raise ValueError("Repo ID must have the form owner/dataset_name.")
    if output.exists():
        raise ValueError("Output already exists; choose a new directory.")
    records = read_records(source)
    with phase("Loading LeRobot"):
        cls = dataset_class()
    output.parent.mkdir(parents=True, exist_ok=True)
    with (output.parent / ("." + output.name + ".lerobot.lock")).open("a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise ValueError("Another converter is writing to this output.") from exc
        if output.exists():
            raise ValueError("Output already exists; choose a new directory.")
        temporary = Path(tempfile.mkdtemp(prefix="." + output.name + "-", dir=output.parent))
        stage, writer = temporary / "dataset", None
        lengths, shapes = [], None
        episodes_progress = tqdm(total=len(records), desc="Episodes", unit="episode", dynamic_ncols=True, position=0)
        try:
            for number, record in enumerate(records):
                episodes_progress.set_postfix_str(record["episode_id"])
                with phase(f"Synchronizing {record['episode_id']}"):
                    episode = synchronize(source, record, fps, int(max_delta_ms * 1e6))
                with ExitStack() as stack:
                    readers = {}
                    for camera in records[0]["cameras"]:
                        path = source_path(source, "episodes", record["episode_id"], "rgb", camera + ".mp4")
                        count = episode.source_counts[f"rgb/{camera}"]
                        reader = VideoReader(path, count)
                        stack.callback(reader.close)
                        readers[camera] = reader
                    current_shapes = {camera: reader.shape for camera, reader in readers.items()}
                    if shapes is not None and current_shapes != shapes:
                        raise ValueError("Camera dimensions must be consistent across episodes.")
                    if writer is None:
                        shapes = current_shapes
                        features = {key: {"dtype": "float32", "shape": (26,), "names": VECTOR_NAMES}
                                    for key in ("observation.state", "action")}
                        features.update({f"observation.images.{camera}": {
                            "dtype": "video", "shape": shape, "names": ["height", "width", "channels"]}
                            for camera, shape in shapes.items()})
                        writer = cls.create(repo_id=repo_id, root=stage, fps=fps, features=features,
                                            robot_type="rebot_bimanual_eef", vcodec="h264", video_backend="pyav",
                                            encoder_threads=2)
                        enable_blocking_encoding(writer, fps)
                    with tqdm(range(len(episode.targets) - 1), desc=f"Frames {number + 1}/{len(records)}",
                              unit="frame", position=1, leave=False, dynamic_ncols=True) as frames_progress:
                        for tick in frames_progress:
                            frame = {"observation.state": episode.states[tick].copy(),
                                     "action": episode.states[tick + 1].copy(),
                                     "task": record["instructions"][0]["text"].strip()}
                            for camera, reader in readers.items():
                                frame[f"observation.images.{camera}"] = reader.get(int(episode.indices[f"rgb/{camera}"][tick]))
                            writer.add_frame(frame)
                    with phase(f"Checking source video tails: {record['episode_id']}"):
                        for reader in readers.values():
                            reader.verify_end()
                    with phase(f"Finishing video encoding and saving: {record['episode_id']}"):
                        writer.save_episode(parallel_encoding=False)
                length = len(episode.targets) - 1
                lengths.append(length)
                episodes_progress.update(1)
            episodes_progress.set_description("Converted episodes")
            episodes_progress.set_postfix_str("validating output")
            with phase("Finalizing dataset"):
                writer.finalize()
            writer = None
            validate_export(stage, repo_id, fps, lengths, records[0]["cameras"], cls)
            if output.exists():
                raise ValueError("Output appeared during conversion; refusing to overwrite it.")
            stage.rename(output)
            episodes_progress.set_description("Completed episodes")
            episodes_progress.set_postfix_str("validated")
            episodes_progress.close()
            print(f"Saved LeRobot v3 dataset: {output} ({sum(lengths)} frames, {fps} FPS)", flush=True)
            return {"fps": fps, "total_episodes": len(lengths), "total_frames": sum(lengths)}
        finally:
            try:
                if writer is not None:
                    writer.finalize()
            finally:
                episodes_progress.close()
                shutil.rmtree(temporary)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", required=True, type=Path, help="Processed reBot dataset directory")
    parser.add_argument("--output", required=True, type=Path, help="New LeRobot v3 output directory")
    parser.add_argument("--repo-id", default="local/rebot", help="Dataset identity; no upload is performed")
    parser.add_argument("--fps", default=30, type=int, help="Synchronized frame rate (default: 30)")
    parser.add_argument("--max-delta-ms", default=50.0, type=float, help="Maximum nearest-sample time error (default: 50 ms)")
    args = parser.parse_args()
    import av
    # Keep native encoder statistics from scrolling the progress bars off screen.
    av.logging.set_level(av.logging.WARNING)
    av.logging.set_libav_level(av.logging.WARNING)
    try:
        convert(args.dataset, args.output, args.repo_id, args.fps, args.max_delta_ms)
    except (ValueError, RuntimeError, OSError, KeyError) as exc:
        print(f"Conversion failed: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
