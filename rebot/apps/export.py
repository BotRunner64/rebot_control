"""Synchronize processed reBot episodes and export a local LeRobot v3 dataset."""

import argparse
from pathlib import Path


def main(argv=None, prog=None):
    parser = argparse.ArgumentParser(prog=prog, description=__doc__)
    parser.add_argument("--dataset", required=True, type=Path, help="Processed reBot dataset directory")
    parser.add_argument("--output", required=True, type=Path, help="New LeRobot v3 output directory")
    parser.add_argument("--repo-id", default="local/rebot", help="Dataset identity; no upload is performed")
    parser.add_argument("--fps", default=30, type=int, help="Synchronized frame rate (default: 30)")
    parser.add_argument("--max-delta-ms", default=50.0, type=float,
                        help="Maximum nearest-sample time error (default: 50 ms)")
    args = parser.parse_args(argv)

    import av

    from rebot.data.lerobot import convert

    # Keep native encoder statistics from scrolling the progress bars off screen.
    av.logging.set_level(av.logging.WARNING)
    av.logging.set_libav_level(av.logging.WARNING)
    convert(args.dataset, args.output, args.repo_id, args.fps, args.max_delta_ms)
