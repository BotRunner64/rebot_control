"""Control both physical arms and grippers from a remote policy server."""

import argparse
import math
from pathlib import Path

from rebot.paths import ARM_BINDINGS, CONTROL_SETTINGS, GRIPPER_SETTINGS


def parser(prog=None):
    result = argparse.ArgumentParser(prog=prog, description=__doc__)
    result.add_argument("--host", default="127.0.0.1", help="Policy server address")
    result.add_argument("--port", type=int, default=8000)
    result.add_argument("--instruction", required=True)
    result.add_argument("--camera", action="append", metavar="NAME=DEVICE", required=True,
                        help="Repeat once per camera stream the policy requires")
    result.add_argument("--replan-steps", type=int, default=16, help="Request a new chunk every N control periods (30 Hz)")
    result.add_argument("--request-timeout", type=float, default=1., help="Bound each socket operation in seconds")
    result.add_argument("--camera-max-age", type=float, default=.25)
    result.add_argument("--camera-max-skew", type=float, default=.10, help="Maximum camera/state skew in seconds")
    result.add_argument("--duration", type=float, help="Stop after this many seconds")
    result.add_argument("--gripper-open-deg", type=float, default=270., help="Physical full-open motor angle")
    result.add_argument("--record", type=Path, metavar="DATASET",
                        help="Also record this run as one episode of a `rebot collect` dataset")
    result.add_argument("--config", type=Path, default=ARM_BINDINGS)
    result.add_argument("--settings", type=Path, default=CONTROL_SETTINGS)
    result.add_argument("--gripper-settings", type=Path, default=GRIPPER_SETTINGS)
    return result


def main(argv=None, prog=None):
    cli = parser(prog)
    args = cli.parse_args(argv)
    if not 1 <= args.port <= 65535:
        cli.error("port must be between 1 and 65535")
    if not args.instruction.strip() or args.replan_steps < 1:
        cli.error("provide a nonempty instruction and positive replan steps")
    for key in ("request_timeout", "camera_max_age", "camera_max_skew", "duration", "gripper_open_deg"):
        value = getattr(args, key)
        if value is not None and (not math.isfinite(value) or value <= 0):
            cli.error(f"{key.replace('_', '-')} must be finite and positive")
    if args.gripper_open_deg > 270:
        cli.error("gripper opening must be at most 270 degrees")

    from rebot.policy.client import run

    try:
        run(**vars(args))
    except InterruptedError:
        # A stop request during robot startup or IK is an interruption, not a failure.
        return 130
    return 0
