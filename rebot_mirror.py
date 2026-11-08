#!/usr/bin/env python3
"""Mirror two physical B601-RS arms in Three.js using read-only CAN queries."""

import argparse
from contextlib import ExitStack
import math
from pathlib import Path
import subprocess
import sys
import time

import can
import yaml

from rebot import CONFIG, load_config, open_reader, resolve_channel, verify_identity
from stararm_teleop import JOINTS, SIDES, load_model, open_viewer

GRIPPER_OPEN_RAD = math.radians(270)


def load_bindings(config):
    arms = load_config(config)
    references, channels = {}, {}
    for side in SIDES:
        if side not in arms:
            raise ValueError(f"Run 'python rebot.py bind {side} <channel>' first.")
        reference = arms[side].get("reference_rad")
        if reference is None:
            raise ValueError(f"Run 'python rebot.py calibrate {side}' first.")
        references[side] = reference
        channels[side] = resolve_channel(arms[side])
    if channels["left"] == channels["right"]:
        raise ValueError("The two physical arms must use different CAN interfaces.")
    if set(arms["left"]["motor_uids"].values()) & set(arms["right"]["motor_uids"].values()):
        raise ValueError("Left and right bindings contain shared motor UIDs; check the bindings.")
    return arms, references, channels


def set_model_pose(model, data, raw, references, gripper_open_rad=GRIPPER_OPEN_RAD):
    import mujoco

    if not math.isfinite(gripper_open_rad) or gripper_open_rad <= 0:
        raise ValueError("The gripper opening angle must be finite and positive.")
    relative = {}
    # Validate the entire pair before changing either model's pose.
    for side in SIDES:
        values, reference = raw[side], references[side]
        if (len(values) != 7 or len(reference) != 7
                or any(not math.isfinite(x) for x in (*values, *reference))):
            raise ValueError(f"{side}: expected seven finite angles and references.")
        relative[side] = [angle - zero for angle, zero in zip(values, reference)]
    clipped = set()
    for side in SIDES:
        # These are RS motor coordinates, not Star Arm leader coordinates.
        for name, angle in zip(JOINTS, relative[side]):
            joint = model.joint(f"{side}_{name}")
            low, high = joint.range
            position = max(low, min(high, angle))
            if position != angle:
                clipped.add(joint.name)
            data.joint(joint.name).qpos[0] = position
        # Motor 7 is rotary, while the MJCF fingers use linear slide joints.
        # No Star Arm handle multiplier belongs in this direct motor mapping.
        opening = max(0, min(1, relative[side][6] / gripper_open_rad)) * 0.05
        for name in ("joint7", "joint_left", "joint_right"):
            data.joint(f"{side}_{name}").qpos[0] = opening
    data.qvel[:] = 0
    mujoco.mj_forward(model, data)
    return clipped


def mirror(config=CONFIG, fps=30.0, gripper_open_rad=GRIPPER_OPEN_RAD, *, port=8080, open_browser=True):
    from rebot_viewer import validate_port

    validate_port(port)
    if not math.isfinite(fps) or not 1 <= fps <= 120:
        raise ValueError("Frame rate must be between 1 and 120 Hz.")
    if not math.isfinite(gripper_open_rad) or gripper_open_rad <= 0:
        raise ValueError("The gripper opening angle must be finite and positive.")
    arms, references, channels = load_bindings(config)
    model, data = load_model("rebot")
    with ExitStack() as stack:
        readers = {}
        for side in SIDES:
            readers[side] = stack.enter_context(open_reader(channels[side]))
            verify_identity(readers[side], arms[side])
            print(f"{side}: verified seven motors on {channels[side]}.", flush=True)
        raw = {side: readers[side].positions() for side in SIDES}
        clipped = set_model_pose(model, data, raw, references, gripper_open_rad)
        if clipped:
            print(f"Model limits reached: {', '.join(sorted(clipped))}. "
                  "Check the physical reference pose if the display differs.", file=sys.stderr, flush=True)
        with open_viewer(model, data, "rebot", port=port, open_browser=open_browser) as viewer:
            print("Physical reBot RS -> Three.js dual-arm mirroring started. "
                  "Read-only: motor torque state is unchanged. "
                  "Move the physical arms; use Stop viewer or press Ctrl-C to stop.", flush=True)
            started, frames, failures = time.monotonic(), 0, 0
            last_valid = started
            while viewer.is_running():
                tick = time.monotonic()
                try:
                    raw = {}
                    for side in SIDES:
                        raw[side] = readers[side].positions()
                except (OSError, ValueError, can.CanError) as exc:
                    # Timeouts are OSError subclasses. Fault reports raise
                    # RuntimeError and abort immediately instead of retrying.
                    failures += 1
                    elapsed = time.monotonic() - last_valid
                    detail = f"{side}: {exc}"
                    print(f"Feedback retry {failures}: {detail} "
                          f"({elapsed:.2f}s since last complete pair); both models frozen.",
                          file=sys.stderr, flush=True)
                    if failures >= 3 and elapsed >= 2:
                        raise RuntimeError(f"Feedback did not recover; stopping. {detail}") from exc
                else:
                    last_valid = time.monotonic()
                    if failures:
                        print(f"Feedback recovered after {failures} failed read(s).", flush=True)
                    failures = 0
                    current_clipped = set_model_pose(model, data, raw, references, gripper_open_rad)
                    if current_clipped != clipped:
                        if current_clipped:
                            print(f"Model limits reached: {', '.join(sorted(current_clipped))}.",
                                  file=sys.stderr, flush=True)
                        else:
                            print("All arm angles are within model limits again.", flush=True)
                        clipped = current_clipped
                    frames += 1
                viewer.sync()
                time.sleep(max(0, 1 / fps - (time.monotonic() - tick)))
            print(f"Displayed {frames} paired frames in {time.monotonic() - started:.1f} seconds.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=CONFIG)
    parser.add_argument("--fps", type=float, default=30, help="Requested display rate (1-120 Hz)")
    parser.add_argument("--gripper-open-deg", type=float, default=270,
                        help="Motor 7 angle relative to closed for full visual opening (default: 270)")
    parser.add_argument("--port", type=int, default=8080, help="Local Three.js viewer port")
    parser.add_argument("--no-browser", action="store_true", help="Print the viewer URL without opening a browser")
    args = parser.parse_args()
    try:
        mirror(args.config, args.fps, math.radians(args.gripper_open_deg),
               port=args.port, open_browser=not args.no_browser)
        return 0
    except (OSError, ValueError, RuntimeError, ImportError, can.CanError,
            yaml.YAMLError, subprocess.SubprocessError) as exc:
        print(str(exc), file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
