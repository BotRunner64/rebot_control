"""The LOOM-data export contract, implemented here: 30 fps synchronization and LeRobot v3.0 output.

The format is defined by LOOM-data's export contract (see docs/data-format.md) and this
module produces the same files as that project's `contract`, `sync` and `video` modules at
commit feba93d, without depending on the package. Recording stages one directory per
episode (`frames.parquet` plus one MP4 per camera, synchronized at 30 fps); `build_dataset`
assembles staged episodes into the published dataset, and `Dataset` reads it back.
"""

from fractions import Fraction
import json
import math
import os
import re
import shutil
import stat

import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq

FPS = 30
NS_PER_SECOND = 1_000_000_000
STATE_GAP_NS = 500_000_000   # widest pair of state samples a frame may be interpolated across
FRAME_GAP_NS = 250_000_000   # farthest RGB frame a frame time may take; nearer gaps repeat frames
VIDEO_GOP = 2
VIDEO_CRF = 18

SIDES = ("left", "right")
STATE_NAMES = ("left_eef", "right_eef", "left_gripper", "right_gripper")
EPISODE_ID = re.compile(r"[A-Za-z0-9_-]+")
CAMERA_IDS = ("left_wrist", "right_wrist", "top")
LAYOUT_ORDER = ("top", "left_wrist", "right_wrist")

STATE_KEY = "observation.state"
STATE_DIM = 26
# Where each stream lives inside observation.state.
STATE_SLICES = {"left_eef": slice(0, 12), "left_gripper": slice(12, 13),
                "right_eef": slice(13, 25), "right_gripper": slice(25, 26)}
POSE_COMPONENTS = ("x", "y", "z", "r00", "r10", "r20", "r01", "r11", "r21", "r02", "r12", "r22")
STATE_COMPONENTS = tuple(
    name for side in SIDES
    for name in (*(f"{side}_eef.{c}" for c in POSE_COMPONENTS), f"{side}_gripper.openness"))
STATE = pa.list_(pa.float32(), STATE_DIM)

# One export resolution per aspect ratio: width 640 where that gives a whole, even height;
# a ratio that does not scale to one keeps its native resolution.
RESOLUTIONS = {(4, 3): (640, 480), (16, 9): (640, 360), (8, 5): (640, 400), (16, 13): (640, 520),
               (53, 30): (848, 480)}

FRAMES = "frames.parquet"
FRAME_SCHEMA = pa.schema([pa.field(STATE_KEY, STATE, nullable=False),
                          pa.field("task", pa.string(), nullable=False)])
DATA_SCHEMA = pa.schema([pa.field(STATE_KEY, STATE, nullable=False),
                         pa.field("timestamp", pa.float32(), nullable=False),
                         *(pa.field(name, pa.int64(), nullable=False)
                           for name in ("frame_index", "episode_index", "index", "task_index"))])
DATA_PATH = "data/chunk-{chunk_index:03d}/file-{file_index:03d}.parquet"
VIDEO_PATH = "videos/{video_key}/chunk-{chunk_index:03d}/file-{file_index:03d}.mp4"
EPISODES_PATH = "meta/episodes/chunk-{chunk_index:03d}/file-{file_index:03d}.parquet"
CHUNK_SIZE = 1000              # files per chunk directory
DATA_FILE_ROWS = 500_000       # frames per data file before starting the next one
EPISODE_FILE_ROWS = 100_000    # episodes per metadata file


def is_pose(name):
    return name.endswith("_eef")


def video_key(camera):
    return f"observation.images.{camera}"


# --- 30 fps frame sequence ---------------------------------------------------

def frame_ns(index):
    """Time of frame `index` on the episode clock, rounded to whole nanoseconds."""
    return (2 * NS_PER_SECOND * index + FPS) // (2 * FPS)


def frame_times(count):
    """Frame times of an episode in ns: frame k is at k / 30 s."""
    return np.array([frame_ns(k) for k in range(count)], dtype=np.int64)


def ordered(times, label="sample"):
    times = np.asarray(times, dtype=np.int64)
    if times.ndim != 1 or not len(times) or np.any(times[1:] < times[:-1]):
        raise ValueError(f"Empty or unordered {label} timestamps")
    return times


def frame_grid(streams):
    """Frame times covering the interval every stream has data for; never extrapolates."""
    streams = [ordered(times) for times in streams]
    start, end = max(int(t[0]) for t in streams), min(int(t[-1]) for t in streams)
    count = 0
    if end >= start:
        # Frames 0..last with frame_ns(last) <= end - start. Frame times are rounded to
        # whole ns, so a stream ending exactly on a rounded-down frame time keeps that frame.
        last = (end - start) * FPS // NS_PER_SECOND
        last += frame_ns(last + 1) <= end - start
        count = last + 1
    if count < 2:
        raise ValueError("Streams share less than two 30 fps frames")
    return start + frame_times(count)


def brackets(times, grid):
    """Indices of the samples around each frame time and the weight of the later one."""
    times = ordered(times)
    if len(times) == 1:
        zeros = np.zeros(len(grid), dtype=np.int64)
        return zeros, zeros, np.zeros(len(grid)), np.zeros(len(grid), dtype=np.int64)
    high = np.clip(np.searchsorted(times, grid, side="right"), 1, len(times) - 1)
    low = high - 1
    span = times[high] - times[low]
    weight = np.where(span > 0, (grid - times[low]) / np.maximum(span, 1), 0.)
    if np.any((weight < 0) | (weight > 1)):
        raise ValueError("Frame times lie outside the sampled interval")
    return low, high, weight, span


def check_gap(weight, span, label):
    between = (weight > 0) & (weight < 1)
    if np.any(span[between] > STATE_GAP_NS):
        raise ValueError(f"{label}: state samples are more than {STATE_GAP_NS // 1_000_000} ms apart")


def resample_scalar(times, values, grid, label="state"):
    low, high, weight, span = brackets(times, grid)
    check_gap(weight, span, label)
    values = np.asarray(values, dtype=np.float64)
    return values[low] * (1 - weight) + values[high] * weight


def quaternions(rotations):
    """Rotation matrices -> unit wxyz quaternions."""
    r = np.asarray(rotations, dtype=np.float64)
    r00, r11, r22 = r[:, 0, 0], r[:, 1, 1], r[:, 2, 2]
    squared = np.column_stack((1+r00+r11+r22, 1+r00-r11-r22, 1-r00+r11-r22, 1-r00-r11+r22))
    candidates = np.stack((
        np.column_stack((squared[:, 0], r[:, 2, 1]-r[:, 1, 2], r[:, 0, 2]-r[:, 2, 0], r[:, 1, 0]-r[:, 0, 1])),
        np.column_stack((r[:, 2, 1]-r[:, 1, 2], squared[:, 1], r[:, 1, 0]+r[:, 0, 1], r[:, 0, 2]+r[:, 2, 0])),
        np.column_stack((r[:, 0, 2]-r[:, 2, 0], r[:, 1, 0]+r[:, 0, 1], squared[:, 2], r[:, 2, 1]+r[:, 1, 2])),
        np.column_stack((r[:, 1, 0]-r[:, 0, 1], r[:, 0, 2]+r[:, 2, 0], r[:, 2, 1]+r[:, 1, 2], squared[:, 3])),
    ), axis=1)
    q = candidates[np.arange(len(r)), np.argmax(squared, axis=1)]
    return q / np.linalg.norm(q, axis=1, keepdims=True)


def rotation_matrices(q):
    w, x, y, z = (q / np.linalg.norm(q, axis=1, keepdims=True)).T
    return np.stack((
        np.stack((1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)), axis=1),
        np.stack((2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)), axis=1),
        np.stack((2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)), axis=1)), axis=1)


def slerp(a, b, weight):
    b = np.where(np.sum(a * b, axis=1, keepdims=True) < 0, -b, b)
    angle = np.arccos(np.clip(np.sum(a * b, axis=1), -1., 1.))
    sine = np.sin(angle)
    small = sine < 1e-9
    safe = np.where(small, 1., sine)
    first = np.where(small, 1 - weight, np.sin((1 - weight) * angle) / safe)
    second = np.where(small, weight, np.sin(weight * angle) / safe)
    return first[:, None] * a + second[:, None] * b


def pose_rotations(poses):
    """N x 12 contract poses -> N rotation matrices."""
    return np.asarray(poses, dtype=np.float64)[:, 3:12].reshape(-1, 3, 3).transpose(0, 2, 1)


def resample_poses(times, poses, grid, label="pose"):
    """N x 12 contract poses: positions interpolate linearly, rotations by slerp."""
    low, high, weight, span = brackets(times, grid)
    check_gap(weight, span, label)
    poses = np.asarray(poses, dtype=np.float64)
    position = poses[low, :3] * (1 - weight[:, None]) + poses[high, :3] * weight[:, None]
    q = quaternions(pose_rotations(poses))
    rotation = rotation_matrices(slerp(q[low], q[high], weight))
    return np.concatenate((position, rotation.transpose(0, 2, 1).reshape(-1, 9)), axis=1)


def nearest(times, grid):
    """Index of the source frame acquired closest to each frame time."""
    times = ordered(times, "video")
    low, high, _, _ = brackets(times, grid)
    # A frame time midway between two source frames takes the earlier one, so a 15 fps
    # source repeats each frame instead of showing an image from the future. The
    # microsecond margin keeps that choice stable against timestamp rounding.
    index = np.where(grid - times[low] <= times[high] - grid + 1000, low, high)
    if np.any(np.abs(times[index] - grid) > FRAME_GAP_NS):
        raise ValueError(f"No RGB frame within {FRAME_GAP_NS // 1_000_000} ms of a frame time")
    return index


# --- staged episodes ---------------------------------------------------------

def check_rotations(rotations, tolerance=1e-6):
    rotations = np.asarray(rotations, dtype=np.float64)
    if (rotations.ndim != 3 or rotations.shape[1:] != (3, 3)
            or not np.isfinite(rotations).all()
            or not np.allclose(rotations @ np.swapaxes(rotations, 1, 2), np.eye(3), atol=tolerance, rtol=0)
            or not np.allclose(np.linalg.det(rotations), 1, atol=tolerance, rtol=0)):
        raise ValueError("Invalid rotation matrix")


def check_state(state):
    """N x 26 observation.state as stored: finite, float32-orthonormal poses, openness in [0,1]."""
    state = np.asarray(state, dtype=np.float64)
    if state.ndim != 2 or state.shape[1] != STATE_DIM or not len(state) or not np.isfinite(state).all():
        raise ValueError(f"Expected finite N x {STATE_DIM} {STATE_KEY}")
    for name in STATE_NAMES:
        values = state[:, STATE_SLICES[name]]
        if is_pose(name):
            check_rotations(pose_rotations(values), 1e-5)
        elif np.any((values < 0) | (values > 1)):
            raise ValueError("Gripper openness outside [0,1]")
    return state


def export_size(width, height):
    """Export resolution of a source frame: the one registered for its aspect ratio."""
    if type(width) is not int or type(height) is not int or width < 1 or height < 1:
        raise ValueError(f"Invalid source frame size: {width}x{height}")
    ratio = Fraction(width, height)
    size = RESOLUTIONS.get((ratio.numerator, ratio.denominator))
    if size is None:
        raise ValueError(f"Aspect ratio {ratio.numerator}:{ratio.denominator} ({width}x{height}) has no export "
                         "resolution; register it in the export contract and contract.RESOLUTIONS first")
    return size


def layout_name(cameras, sizes):
    """Deterministic dataset name of a camera set and its export resolutions."""
    cameras = [camera for camera in LAYOUT_ORDER if camera in cameras]
    found = [f"{sizes[camera][0]}x{sizes[camera][1]}" for camera in cameras]
    return "+".join(cameras) + "_" + (found[0] if len(set(found)) == 1 else "+".join(found))


def frame_tasks(instructions, times):
    """Instruction text in force at each frame time (ns on the episode clock)."""
    if not isinstance(instructions, list) or not instructions:
        raise ValueError("Missing instructions")
    previous = 0
    for instruction in instructions:
        if not isinstance(instruction, dict):
            raise ValueError("Invalid instruction")
        start, end, text = (instruction.get(k) for k in ("start_ns", "end_ns", "text"))
        if (type(start) is not int or type(end) is not int
                or not previous <= start < end < 2**63
                or not isinstance(text, str) or not text.strip()):
            raise ValueError("Invalid instruction interval/text")
        previous = end
    starts = np.array([i["start_ns"] for i in instructions], dtype=np.int64)
    index = np.clip(np.searchsorted(starts, times, side="right") - 1, 0, len(instructions) - 1)
    return [instructions[i]["text"] for i in index]


def write_frames(destination, streams, instructions, origin, grid):
    """Synchronize the four state streams and the instructions; write frames.parquet.

    `streams` maps each STATE_NAMES entry to (timestamps ns, values), `grid` holds the
    frame times from `frame_grid`, and `instructions` are intervals relative to `origin`.
    Returns the frame count.
    """
    if set(streams) != set(STATE_NAMES):
        raise ValueError("Expected the four state streams")
    grid = np.asarray(grid, dtype=np.int64)
    if len(grid) < 2 or grid[0] < origin:
        raise ValueError("An episode needs at least two frames at or after its time origin")
    state = np.empty((len(grid), STATE_DIM), dtype=np.float32)
    for name in STATE_NAMES:
        times, values = streams[name]
        times = ordered(times, name)
        data = np.asarray(values, dtype=np.float64)
        if not np.isfinite(data).all():
            raise ValueError(f"Nonfinite {name}")
        if is_pose(name):
            if data.shape != (len(times), 12):
                raise ValueError("Expected N x 12 poses")
            check_rotations(pose_rotations(data))
            data = resample_poses(times, data, grid, name)
        else:
            if data.shape != (len(times),) or np.any((data < 0) | (data > 1)):
                raise ValueError("Expected scalar gripper openness in [0,1]")
            data = np.clip(resample_scalar(times, data, grid, name), 0., 1.)
        state[:, STATE_SLICES[name]] = data.reshape(len(grid), -1)
    check_state(state)
    tasks = frame_tasks(instructions, grid - int(origin))
    destination.mkdir(parents=True, exist_ok=True)
    pq.write_table(pa.Table.from_arrays(
        [pa.FixedSizeListArray.from_arrays(pa.array(state.reshape(-1), type=pa.float32()), STATE_DIM),
         pa.array(tasks, type=pa.string())], schema=FRAME_SCHEMA), destination / FRAMES)
    return len(grid)


class Moments:
    """Running min/max/mean/std of one feature."""

    def __init__(self):
        self.count, self.low, self.high, self.total, self.squares = 0, None, None, 0., 0.

    def add(self, low, high, total, squares, count):
        low, high = np.asarray(low, dtype=np.float64), np.asarray(high, dtype=np.float64)
        self.low = low if self.low is None else np.minimum(self.low, low)
        self.high = high if self.high is None else np.maximum(self.high, high)
        self.total = self.total + np.asarray(total, dtype=np.float64)
        self.squares = self.squares + np.asarray(squares, dtype=np.float64)
        self.count += count

    def add_values(self, values):
        values = np.asarray(values, dtype=np.float64).reshape(len(values), -1)
        self.add(values.min(axis=0), values.max(axis=0), values.sum(axis=0), (values ** 2).sum(axis=0), len(values))

    def result(self, shape=None):
        mean = self.total / self.count
        std = np.sqrt(np.maximum(self.squares / self.count - mean ** 2, 0.))
        return {"min": np.reshape(self.low, shape or self.low.shape).tolist(),
                "max": np.reshape(self.high, shape or self.high.shape).tolist(),
                "mean": np.reshape(mean, shape or mean.shape).tolist(),
                "std": np.reshape(std, shape or std.shape).tolist(), "count": [self.count]}


def encode_frames(path, frames, times, grid=None):
    """Encode display-order source frames as the constant 30 fps H.264 video of an episode.

    `times[i]` is the acquisition time of source frame i. Each `grid` time takes the nearest
    source frame; `grid=None` passes the frames through one to one. Output frame k is the
    image of frame-table row k, at the export resolution of the source aspect ratio.
    Returns the exported size and image statistics for the episode record.
    """
    import av

    select = np.arange(len(times)) if grid is None else nearest(times, grid)
    moments, count, written = Moments(), 0, 0
    container = stream = source_size = size = None
    path.parent.mkdir(parents=True, exist_ok=True)
    pending = path.with_name(f".{path.name}.pending")
    try:
        for frame in frames:
            index, count = count, count + 1
            if index >= len(times):
                raise ValueError(f"Decoded more than the expected {len(times)} frames")
            if container is None:
                source_size = (frame.width, frame.height)
                size = export_size(*source_size)
                container = av.open(str(pending), "w", format="mp4")
                stream = container.add_stream("h264", FPS, options={"g": str(VIDEO_GOP), "crf": str(VIDEO_CRF)})
                stream.pix_fmt, stream.time_base = "yuv420p", Fraction(1, FPS)
                stream.width, stream.height = size
            elif (frame.width, frame.height) != source_size:
                raise ValueError("Video resolution changed within episode")
            scaled = rgb = None
            while written < len(select) and select[written] == index:
                if scaled is None:
                    scaled = frame.reformat(width=size[0], height=size[1], format="yuv420p")
                    # The encoder is fed RGB, as LOOM-data's is, so the pixels come out identical.
                    rgb = scaled.to_ndarray(format="rgb24")
                if written % FPS == 0:  # one frame per second feeds the image statistics
                    pixels = scaled.reformat(width=160, height=max(2, size[1] // 4),
                                             format="rgb24").to_ndarray().reshape(-1, 3) / 255.
                    moments.add_values(pixels)
                image = av.VideoFrame.from_ndarray(rgb, format="rgb24")
                image.pts, image.time_base = written, Fraction(1, FPS)
                container.mux(stream.encode(image))
                written += 1
        if count != len(times) or written != len(select) or not written:
            raise ValueError(f"Decoded {count} frames; expected {len(times)}")
        container.mux(stream.encode())
        container.close()
        container = None
        pending.replace(path)
    finally:
        if container is not None:
            container.close()
        pending.unlink(missing_ok=True)
    return {"size": list(size),
            "stats": {"min": moments.low.tolist(), "max": moments.high.tolist(), "sum": moments.total.tolist(),
                      "sumsq": moments.squares.tolist(), "count": moments.count}}


def file_frames(source):
    """Display-order frames of a single-video-stream file."""
    import av

    try:
        with av.open(str(source)) as reader:
            if len(reader.streams.video) != 1:
                raise ValueError(f"{source}: expected one video stream")
            yield from reader.decode(video=0)
    except av.InvalidDataError as exc:
        raise ValueError(f"{source}: {exc}") from exc


def transcode_video(source, output, times, grid=None):
    """Encode a whole source file whose frame i was acquired at times[i]."""
    if len(times) < 1:
        raise ValueError("Expected a nonempty source video")
    return encode_frames(output, file_frames(source), times, grid)


def segment_frames(video, expected_frames, fps):
    """Display frames of one episode inside a shared video file, checked for contiguity."""
    import av

    start, end = video["from_timestamp"], video["to_timestamp"]
    tolerance = min(1e-3, .01 / fps)
    with av.open(video["path"]) as reader:
        if len(reader.streams.video) != 1:
            raise ValueError(f"{video['path']}: expected one video stream")
        source = reader.streams.video[0]
        reader.seek(math.floor(start / source.time_base), stream=source, backward=True, any_frame=False)
        count = 0
        for frame in reader.decode(source):
            timestamp = frame.time
            if timestamp is None:
                raise ValueError(f"{video['path']}: video frame has no timestamp")
            if timestamp < start - tolerance:
                continue
            if timestamp >= end - tolerance:
                break
            # Reject missing/extra frames even when their total count happens to match.
            if abs(timestamp - (start + count / fps)) > tolerance:
                raise ValueError(f"{video['path']}: noncontiguous video segment at frame {count}")
            yield frame
            count += 1
        if count != expected_frames:
            raise ValueError(f"{video['path']}: expected {expected_frames} frames, decoded {count}")


def extract_video_segment(video, destination, times, fps, grid=None):
    """Encode one episode's segment of a shared video file; segment frame i was acquired at times[i]."""
    return encode_frames(destination, segment_frames(video, len(times), fps), times, grid)


def episode_record(episode_id, length, videos):
    """Record of a staged episode. `videos` maps camera -> the dict `encode_frames` returns."""
    return {"episode_id": episode_id, "cameras": sorted(videos), "length": length,
            "sizes": {camera: list(video["size"]) for camera, video in sorted(videos.items())},
            "image_stats": {camera: video["stats"] for camera, video in sorted(videos.items())}}


def validate_record(record, episode_id):
    """One staged-episode record."""
    if not isinstance(record, dict) or record.get("episode_id") != episode_id:
        raise ValueError(f"{episode_id}: invalid record identity")
    cameras = record.get("cameras")
    if (not isinstance(cameras, list) or not cameras or cameras != sorted(set(cameras))
            or any(c not in CAMERA_IDS for c in cameras)):
        raise ValueError(f"{episode_id}: invalid cameras")
    if type(record.get("length")) is not int or record["length"] < 2:
        raise ValueError(f"{episode_id}: invalid length")
    sizes, stats = record.get("sizes"), record.get("image_stats")
    if (not isinstance(sizes, dict) or set(sizes) != set(cameras)
            or any(tuple(size) not in RESOLUTIONS.values() for size in sizes.values())):
        raise ValueError(f"{episode_id}: invalid camera resolutions")
    if not isinstance(stats, dict) or set(stats) != set(cameras):
        raise ValueError(f"{episode_id}: invalid image statistics")
    for value in stats.values():
        if (not isinstance(value, dict) or type(value.get("count")) is not int or value["count"] < 1
                or any(not isinstance(value.get(k), list) or len(value[k]) != 3
                       or not all(isinstance(x, (int, float)) and math.isfinite(x) for x in value[k])
                       for k in ("min", "max", "sum", "sumsq"))):
            raise ValueError(f"{episode_id}: invalid image statistics")


def validate_episode(directory, record):
    """Check a staged episode directory against the contract."""
    import av

    validate_record(record, directory.name)
    if not stat.S_ISDIR(directory.lstat().st_mode):
        raise ValueError(f"{directory}: invalid episode directory")
    files = set()
    with os.scandir(directory) as entries:
        for entry in entries:
            info = entry.stat(follow_symlinks=False)
            if not stat.S_ISREG(info.st_mode) or info.st_size == 0:
                raise ValueError(f"{directory / entry.name}: invalid episode file")
            files.add(entry.name)
    if files != {FRAMES, *(f"{camera}.mp4" for camera in record["cameras"])}:
        raise ValueError(f"{directory}: episode files differ from record")
    table = pq.read_table(directory / FRAMES)
    if (table.schema.names != FRAME_SCHEMA.names
            or any(table.schema.field(f.name).type != f.type for f in FRAME_SCHEMA)
            or any(c.null_count for c in table.columns) or table.num_rows != record["length"]):
        raise ValueError(f"{directory}: invalid frame table schema, nulls or length")
    try:
        check_state(table[STATE_KEY].to_pylist())
    except ValueError as exc:
        raise ValueError(f"{directory}: {exc}") from exc
    if any(not text.strip() for text in table["task"].unique().to_pylist()):
        raise ValueError(f"{directory}: empty instruction")
    for camera in record["cameras"]:
        path = directory / f"{camera}.mp4"
        with av.open(str(path)) as reader:
            if len(reader.streams) != 1 or len(reader.streams.video) != 1:
                raise ValueError(f"{path}: expected a video-only file")
            video = reader.streams.video[0]
            if (video.codec_context.name != "h264" or "mp4" not in reader.format.name.split(",")
                    or video.codec_context.pix_fmt != "yuv420p" or video.average_rate != FPS
                    or video.frames != record["length"]
                    or [video.width, video.height] != list(record["sizes"][camera])):
                raise ValueError(f"{path}: video format, rate, size or frame count differs from record")


# --- published dataset -------------------------------------------------------

def link(source, target):
    """Hard-link a staged video into the dataset; copy where the filesystem cannot link."""
    target.parent.mkdir(parents=True, exist_ok=True)
    try:
        os.link(source, target)
    except OSError:
        shutil.copyfile(source, target)


def features(cameras, sizes):
    result = {STATE_KEY: {"dtype": "float32", "shape": [STATE_DIM], "names": list(STATE_COMPONENTS)}}
    for camera in sorted(cameras):
        width, height = sizes[camera]
        result[video_key(camera)] = {
            "dtype": "video", "shape": [height, width, 3], "names": ["height", "width", "channels"],
            "info": {"video.height": height, "video.width": width, "video.codec": "h264",
                     "video.pix_fmt": "yuv420p", "video.is_depth_map": False, "video.fps": FPS,
                     "video.channels": 3, "has_audio": False}}
    result["timestamp"] = {"dtype": "float32", "shape": [1], "names": None}
    for name in ("frame_index", "episode_index", "index", "task_index"):
        result[name] = {"dtype": "int64", "shape": [1], "names": None}
    return result


def write_tasks(path, tasks):
    """meta/tasks.parquet as pandas writes it: the task text is the index."""
    pandas = {"index_columns": ["task"],
              "column_indexes": [{"name": None, "field_name": None, "pandas_type": "unicode",
                                  "numpy_type": "object", "metadata": {"encoding": "UTF-8"}}],
              "columns": [{"name": "task_index", "field_name": "task_index", "pandas_type": "int64",
                           "numpy_type": "int64", "metadata": None},
                          {"name": "task", "field_name": "task", "pandas_type": "unicode",
                           "numpy_type": "object", "metadata": None}],
              "creator": {"library": "pyarrow", "version": pa.__version__}, "pandas_version": "2.0.0"}
    schema = pa.schema([pa.field("task_index", pa.int64()), pa.field("task", pa.string())],
                       metadata={"pandas": json.dumps(pandas)})
    pq.write_table(pa.Table.from_arrays([pa.array(list(tasks.values()), type=pa.int64()),
                                         pa.array(list(tasks), type=pa.string())], schema=schema), path)


def build_dataset(root, robot_type, episodes):
    """Assemble staged episodes sharing one layout into a LeRobot v3.0 dataset at `root`.

    `episodes` is the ordered list of (record, staged directory). Frame tables
    are merged into shared data files. Each episode keeps its own video file,
    so staged MP4s are linked in place without re-encoding.
    """
    record = episodes[0][0]
    cameras, sizes = record["cameras"], record["sizes"]
    if any(r["cameras"] != cameras or r["sizes"] != sizes for r, _ in episodes):
        raise ValueError("Episodes of one dataset must share their camera layout")
    root.mkdir(parents=True)
    tasks, rows, index, writer, file_index, file_rows = {}, [], 0, None, -1, 0
    moments = {name: Moments() for name in (STATE_KEY, "timestamp", "frame_index", "episode_index",
                                             "index", "task_index", *map(video_key, cameras))}
    try:
        for episode_index, (record, directory) in enumerate(episodes):
            table = pq.read_table(directory / FRAMES)
            length = table.num_rows
            if length != record["length"]:
                raise ValueError(f"{directory}: frame table length differs from record")
            texts = table["task"].to_pylist()
            task_index = [tasks.setdefault(text, len(tasks)) for text in texts]
            if writer is None or (file_rows and file_rows + length > DATA_FILE_ROWS):
                if writer is not None:
                    writer.close()
                file_index, file_rows = file_index + 1, 0
                path = root / DATA_PATH.format(chunk_index=file_index // CHUNK_SIZE,
                                               file_index=file_index % CHUNK_SIZE)
                path.parent.mkdir(parents=True, exist_ok=True)
                writer = pq.ParquetWriter(path, DATA_SCHEMA)
            frames = np.arange(length, dtype=np.int64)
            columns = {"timestamp": (frames / FPS).astype(np.float32), "frame_index": frames,
                       "episode_index": np.full(length, episode_index, dtype=np.int64),
                       "index": index + frames, "task_index": np.array(task_index, dtype=np.int64)}
            # One row group per episode, so readers can fetch an episode without the rest of the file.
            writer.write_table(pa.Table.from_arrays(
                [table[STATE_KEY].combine_chunks(), *(pa.array(columns[f.name], type=f.type)
                                                      for f in list(DATA_SCHEMA)[1:])], schema=DATA_SCHEMA))
            moments[STATE_KEY].add_values(np.asarray(table[STATE_KEY].to_pylist(), dtype=np.float64))
            for name, values in columns.items():
                moments[name].add_values(values)
            row = {"episode_index": episode_index, "episode_id": record["episode_id"],
                   "tasks": list(dict.fromkeys(texts)), "length": length,
                   "dataset_from_index": index, "dataset_to_index": index + length,
                   "data/chunk_index": file_index // CHUNK_SIZE, "data/file_index": file_index % CHUNK_SIZE,
                   "meta/episodes/chunk_index": episode_index // EPISODE_FILE_ROWS // CHUNK_SIZE,
                   "meta/episodes/file_index": episode_index // EPISODE_FILE_ROWS % CHUNK_SIZE}
            for camera in cameras:
                key = video_key(camera)
                chunk, file = episode_index // CHUNK_SIZE, episode_index % CHUNK_SIZE
                link(directory / f"{camera}.mp4", root / VIDEO_PATH.format(
                    video_key=key, chunk_index=chunk, file_index=file))
                row.update({f"videos/{key}/chunk_index": chunk, f"videos/{key}/file_index": file,
                            f"videos/{key}/from_timestamp": 0., f"videos/{key}/to_timestamp": length / FPS})
                image = record["image_stats"][camera]
                moments[key].add(image["min"], image["max"], image["sum"], image["sumsq"], image["count"])
            rows.append(row)
            index += length
            file_rows += length
    finally:
        if writer is not None:
            writer.close()
    for start in range(0, len(rows), EPISODE_FILE_ROWS):
        number = start // EPISODE_FILE_ROWS
        path = root / EPISODES_PATH.format(chunk_index=number // CHUNK_SIZE, file_index=number % CHUNK_SIZE)
        path.parent.mkdir(parents=True, exist_ok=True)
        pq.write_table(pa.Table.from_pylist(rows[start:start + EPISODE_FILE_ROWS]), path)
    write_tasks(root / "meta/tasks.parquet", tasks)
    stats = {name: value.result((3, 1, 1) if name.startswith("observation.images.") else None)
             for name, value in moments.items()}
    for camera in cameras:
        # Image statistics are sampled per pixel; LeRobot counts images. Quantiles are not computed.
        stats[video_key(camera)]["count"] = [index]
    (root / "meta/stats.json").write_text(json.dumps(stats, indent=2) + "\n", encoding="utf-8")
    info = {"codebase_version": "v3.0", "robot_type": robot_type, "total_episodes": len(rows),
            "total_frames": index, "total_tasks": len(tasks), "chunks_size": CHUNK_SIZE,
            "data_files_size_in_mb": 100, "video_files_size_in_mb": 200, "fps": FPS,
            "splits": {"train": f"0:{len(rows)}"}, "data_path": DATA_PATH, "video_path": VIDEO_PATH,
            "features": features(cameras, sizes)}
    (root / "meta/info.json").write_text(json.dumps(info, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return info


class Dataset:
    """Read access to one exported dataset."""

    def __init__(self, root):
        self.root = root
        try:
            self.info = json.loads((root / "meta/info.json").read_text(encoding="utf-8"))
        except (OSError, ValueError) as exc:
            raise ValueError(f"{root}: not an exported dataset: {exc}") from exc
        if not isinstance(self.info, dict) or not isinstance(self.info.get("features"), dict):
            raise ValueError(f"{root}: not an exported dataset: meta/info.json is not a dataset description")
        feature = self.info["features"].get(STATE_KEY, {})
        if (self.info.get("codebase_version") != "v3.0" or self.info.get("fps") != FPS
                or feature.get("shape") != [STATE_DIM] or feature.get("names") != list(STATE_COMPONENTS)):
            raise ValueError(f"{root}: not a contract dataset (LeRobot v3.0, {FPS} fps, {STATE_DIM}-d state)")
        self.cameras = [camera for camera in CAMERA_IDS if video_key(camera) in self.info["features"]]
        if not self.cameras:
            raise ValueError(f"{root}: no contract cameras")
        table = pq.read_table(root / "meta/tasks.parquet", columns=["task_index", "task"])
        self.tasks = dict(zip(table["task_index"].to_pylist(), table["task"].to_pylist()))
        episodes = []
        for path in sorted((root / "meta/episodes").rglob("*.parquet")):
            episodes += pq.read_table(path).to_pylist()
        episodes.sort(key=lambda row: row["episode_index"])
        identifiers = [row["episode_id"] for row in episodes]
        if (not episodes or [row["episode_index"] for row in episodes] != list(range(len(episodes)))
                or len(episodes) != self.info.get("total_episodes") or len(set(identifiers)) != len(identifiers)
                or any(not isinstance(i, str) or not EPISODE_ID.fullmatch(i) for i in identifiers)):
            raise ValueError(f"{root}: invalid, duplicate or missing episode metadata")
        self.episodes = episodes

    def data_path(self, episode):
        return self.root / self.info["data_path"].format(
            chunk_index=episode["data/chunk_index"], file_index=episode["data/file_index"])

    def video(self, episode, camera):
        """(path, index of the episode's first frame inside that file)."""
        key = video_key(camera)
        path = self.root / self.info["video_path"].format(
            video_key=key, chunk_index=episode[f"videos/{key}/chunk_index"],
            file_index=episode[f"videos/{key}/file_index"])
        return path, round(episode[f"videos/{key}/from_timestamp"] * FPS)

    def frames(self, episode):
        """(N x 26 float64 state, per-frame instruction text) of one episode."""
        table = pq.read_table(self.data_path(episode), columns=[STATE_KEY, "frame_index", "task_index"],
                              filters=[("episode_index", "=", episode["episode_index"])])
        if table.num_rows != episode["length"] or table["frame_index"].to_pylist() != list(range(table.num_rows)):
            raise ValueError(f"{episode['episode_id']}: frame rows differ from episode metadata")
        state = check_state(table[STATE_KEY].to_pylist())
        return state, [self.tasks[i] for i in table["task_index"].to_pylist()]
