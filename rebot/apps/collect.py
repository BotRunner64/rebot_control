"""Teleoperate both reBot arms and record one continuous dataset recording."""

import argparse
from pathlib import Path

from rebot.apps.teleop import add_arguments, plan_from_arguments, teleop
from rebot.data.recording import RecordingOptions, discover_cameras, parse_cameras
from rebot.model.limits import GRIPPER_OPEN_DEG


def main(argv=None, prog=None):
    parser = argparse.ArgumentParser(prog=prog, description=__doc__)
    parser.add_argument("--dataset", type=Path, required=True, help="Output dataset directory")
    parser.add_argument("--instruction", default="__pending__", help="Task instruction (default: __pending__)")
    parser.add_argument("--camera", action="append", metavar="[ID=]DEVICE",
                        help="Camera device with optional name (default: auto-detect all)")
    parser.add_argument("--camera-viewer", action="store_true",
                        help="Open a best-effort camera preview without blocking recording")
    parser.add_argument("--episode-id", help="Unique episode name (default: generated UUID)")
    parser.add_argument("--record-gripper-open-deg", type=float, default=GRIPPER_OPEN_DEG,
                        help="Physical fully-open angle used to normalize recorded openness")
    add_arguments(parser)
    args = parser.parse_args(argv)
    cameras = discover_cameras() if args.camera is None else args.camera
    recording = RecordingOptions(args.dataset, args.instruction, parse_cameras(cameras),
                                 args.episode_id, args.record_gripper_open_deg,
                                 camera_viewer=args.camera_viewer)
    recording.validate()
    teleop(plan_from_arguments(args), recording=recording)
    return 0
