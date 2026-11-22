"""Align native device samples and stage annotated clips in the export contract format."""

import json

import av
import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq

from rebot.data import contract


def synchronize(native, destination, cameras, instruction):
    """Write one shared frame table and a matching constant-rate video per camera."""
    streams = {}
    for name in contract.STATE_NAMES:
        table = pq.read_table(native / "state" / f"{name}.parquet")
        values = table.column(1).combine_chunks()
        if contract.is_pose(name):
            values = values.flatten().to_numpy().reshape(-1, 12)
        else:
            values = values.to_numpy()
        streams[name] = table["timestamp_ns"].to_numpy(), values

    times = {}
    for camera in cameras:
        path = native / "rgb" / f"{camera}.timestamps"
        if path.stat().st_size % 8:
            raise ValueError(f"Incomplete camera timestamp in {path}")
        times[camera] = np.fromfile(path, dtype="<i8")
    grid = contract.frame_grid([*(t for t, _ in streams.values()), *times.values()])
    origin = int(grid[0])
    tasks = [{"start_ns": 0, "end_ns": int(grid[-1]) - origin, "text": instruction}]
    count = contract.write_frames(destination, streams, tasks, origin, grid)
    for camera in cameras:
        source = native / "rgb" / f"{camera}.mp4"
        path = destination / f"{camera}.mp4"
        contract.transcode_video(source, path, times[camera], grid)
        with av.open(str(path)) as reader:
            if reader.streams.video[0].frames != count:
                raise ValueError(f"{camera}: encoded video length differs from the frame table.")
    metadata = {"frames": count, "cameras": list(cameras), "instruction": instruction}
    (destination / "recording.json").write_text(json.dumps(metadata, ensure_ascii=False), encoding="utf-8")
    return metadata, origin


def stage_clip(directory, source_frames, cameras, first, stop, instruction):
    """Copy synchronized states and cut every video at the same frame boundaries."""
    directory.mkdir(parents=True)
    count = stop - first
    states = pq.read_table(source_frames, columns=[contract.STATE_KEY]).slice(first, count)
    if states.num_rows != count:
        raise ValueError(f"{source_frames}: missing recording frames")
    table = pa.Table.from_arrays([states.column(0), pa.array([instruction] * count)], schema=contract.FRAME_SCHEMA)
    pq.write_table(table, directory / contract.FRAMES)
    videos = {}
    for camera, path in cameras.items():
        segment = {"path": str(path), "from_timestamp": first / contract.FPS, "to_timestamp": stop / contract.FPS}
        videos[camera] = contract.extract_video_segment(segment, directory / f"{camera}.mp4", range(count), contract.FPS)
    record = contract.episode_record(directory.name, count, videos)
    contract.validate_episode(directory, record)
    return record
