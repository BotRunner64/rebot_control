"""Mirror both physical reBot RS arms in the 3D viewer using read-only CAN queries."""

import argparse
from contextlib import ExitStack
import math
from pathlib import Path
import sys
import time

import can

from rebot.bindings import SIDES, select_arms
from rebot.devices import socketcan
from rebot.devices.robstride import open_reader
from rebot.model.limits import GRIPPER_OPEN_DEG
from rebot.paths import ARM_BINDINGS
from rebot.viewer.dual import load_dual_model, pose_from_motors
from rebot.viewer.scene import SceneViewer
from rebot.viewer.server import validate_port


def mirror(config=ARM_BINDINGS, fps=30.0, gripper_open_rad=math.radians(GRIPPER_OPEN_DEG), *,
           port=8080, open_browser=True):
    validate_port(port)
    if not math.isfinite(fps) or not 1 <= fps <= 120:
        raise ValueError("Frame rate must be between 1 and 120 Hz.")
    if not math.isfinite(gripper_open_rad) or gripper_open_rad <= 0:
        raise ValueError("The gripper opening angle must be finite and positive.")
    bindings = select_arms(config, "both")
    references = {side: bindings[side].reference for side in SIDES}
    for binding in bindings.values():
        socketcan.check_channel(binding.channel, configure=True)
    model, data = load_dual_model("rebot")
    with ExitStack() as stack:
        buses = {}
        for side, binding in bindings.items():
            buses[side] = stack.enter_context(open_reader(binding.channel))
            binding.verify(buses[side])
            print(f"{side}: verified seven motors on {binding.channel}.", flush=True)
        raw = {side: buses[side].positions() for side in SIDES}
        clipped = pose_from_motors(model, data, raw, references, gripper_open_rad)
        if clipped:
            print(f"Model limits reached: {', '.join(sorted(clipped))}. "
                  "Check the physical reference pose if the display differs.", file=sys.stderr, flush=True)
        with SceneViewer(model, data, "rebot", port=port, open_browser=open_browser) as viewer:
            print("Physical reBot RS -> 3D viewer dual-arm mirroring started. "
                  "Read-only: motor torque state is unchanged. "
                  "Move the physical arms; use Close viewer or press Ctrl-C to stop.", flush=True)
            started, frames, failures = time.monotonic(), 0, 0
            last_valid = started
            while viewer.is_running():
                tick = time.monotonic()
                try:
                    raw = {}
                    for side in SIDES:
                        raw[side] = buses[side].positions()
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
                    current = pose_from_motors(model, data, raw, references, gripper_open_rad)
                    if current != clipped:
                        if current:
                            print(f"Model limits reached: {', '.join(sorted(current))}.",
                                  file=sys.stderr, flush=True)
                        else:
                            print("All arm angles are within model limits again.", flush=True)
                        clipped = current
                    frames += 1
                viewer.sync()
                time.sleep(max(0, 1 / fps - (time.monotonic() - tick)))
            print(f"Displayed {frames} paired frames in {time.monotonic() - started:.1f} seconds.")


def main(argv=None, prog=None):
    parser = argparse.ArgumentParser(prog=prog, description=__doc__)
    parser.add_argument("--config", type=Path, default=ARM_BINDINGS)
    parser.add_argument("--fps", type=float, default=30, help="Requested display rate (1-120 Hz)")
    parser.add_argument("--gripper-open-deg", type=float, default=GRIPPER_OPEN_DEG,
                        help="Motor 7 angle relative to closed for full visual opening (default: 270)")
    parser.add_argument("--port", type=int, default=8080, help="Local 3D viewer port")
    parser.add_argument("--no-browser", action="store_true", help="Print the viewer URL without opening a browser")
    args = parser.parse_args(argv)
    mirror(args.config, args.fps, math.radians(args.gripper_open_deg),
           port=args.port, open_browser=not args.no_browser)
    return 0
