#!/usr/bin/env python3
"""Teleoperate both reBot arms and collect one contract-format dataset episode."""

import argparse
import json
from pathlib import Path
import subprocess
import sys

import can
import yaml

from rebot_recording import CAMERA_SCRIPT, RecordingOptions, parse_cameras
from rebot_teleop import (CONFIG, LEADER_CONFIG, SETTINGS_PATH, GRIPPER_SETTINGS_PATH,
                          teleop)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, required=True, help="Output dataset directory")
    parser.add_argument("--instruction", default="__pending__", help="Task instruction (default: __pending__)")
    parser.add_argument("--camera", action="append", metavar="[ID=]DEVICE",
                        help="Camera device with optional name (default: auto-detect all)")
    parser.add_argument("--camera-viewer", action="store_true",
                        help="Open a best-effort camera preview without blocking recording")
    parser.add_argument("--episode-id", help="Unique episode name (default: generated UUID)")
    parser.add_argument("--config", type=Path, default=CONFIG, help="Follower bindings YAML")
    parser.add_argument("--leader-config", type=Path, default=LEADER_CONFIG)
    parser.add_argument("--settings", type=Path, default=SETTINGS_PATH)
    parser.add_argument("--gripper-settings", type=Path, default=GRIPPER_SETTINGS_PATH)
    parser.add_argument("--gripper-torque-nm", type=float, help="Total gripper tracking torque limit")
    parser.add_argument("--gripper-open-deg", type=float, default=270.,
                        help="Gripper control travel in motor degrees (maximum: 270)")
    parser.add_argument("--record-gripper-open-deg", type=float, default=270.,
                        help="Physical fully-open angle used to normalize recorded openness")
    args = parser.parse_args(argv)
    try:
        cameras = args.camera
        if cameras is None:
            result = subprocess.run(["/usr/bin/python3", str(CAMERA_SCRIPT), "--discover"],
                                    capture_output=True, text=True, check=True, timeout=15.)
            cameras = json.loads(result.stdout)["sources"]
        recording = RecordingOptions(args.dataset, args.instruction, parse_cameras(cameras),
                                     args.episode_id, args.record_gripper_open_deg, camera_viewer=args.camera_viewer)
        recording.validate()
        teleop(config=args.config, leader_config=args.leader_config, settings_path=args.settings,
               side="both", gripper=True, gripper_open_deg=args.gripper_open_deg,
               gripper_settings_path=args.gripper_settings, gripper_torque_nm=args.gripper_torque_nm,
               recording=recording)
        return 0
    except (OSError, ValueError, RuntimeError, ImportError, can.CanError, yaml.YAMLError,
            subprocess.SubprocessError) as exc:
        print(str(exc), file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
