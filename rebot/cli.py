"""The `rebot` command: dispatch to one app module and report failures uniformly."""

import importlib
import subprocess
import sys

# name -> (module, summary). Modules are imported only when their command runs.
COMMANDS = {
    "arm": ("rebot.apps.arm_setup", "Scan, bind, calibrate, and read the reBot follower arms"),
    "leader": ("rebot.apps.leader_setup", "Scan, bind, calibrate, and view the Star Arm leaders"),
    "guide": ("rebot.apps.guide", "Hand guiding with gravity compensation and position hold"),
    "home": ("rebot.apps.home", "Return to the reference pose, then disable the motors"),
    "mirror": ("rebot.apps.mirror", "Mirror the physical arms in the 3D viewer (read-only)"),
    "teleop": ("rebot.apps.teleop", "Teleoperate the followers with the Star Arm leaders"),
    "collect": ("rebot.apps.collect", "Teleoperate and record one dataset recording"),
    "ee": ("rebot.apps.ee", "End-effector pose control (simulation by default)"),
    "policy": ("rebot.apps.policy", "Dual-arm control from a remote policy server"),
    "process": ("rebot.apps.process", "Review recordings and export annotated episodes"),
    "export": ("rebot.apps.export", "Export processed episodes as a LeRobot v3 dataset"),
    "cameras": ("rebot.apps.cameras", "Preview webcams in the browser"),
    "identify": ("rebot.apps.identify", "Gravity mass-scale calibration for one arm"),
}


def usage():
    width = max(map(len, COMMANDS))
    lines = ["usage: rebot <command> [options]", "", "commands:"]
    lines += [f"  {name:<{width}}  {summary}" for name, (_, summary) in COMMANDS.items()]
    lines += ["", "Run `rebot <command> --help` for the options of one command."]
    return "\n".join(lines)


def expected_errors():
    """Failures that are reported as one line instead of a traceback."""
    errors = [OSError, ValueError, RuntimeError, ImportError, subprocess.SubprocessError]
    for module, name in (("can", "CanError"), ("yaml", "YAMLError")):
        try:
            errors.append(getattr(importlib.import_module(module), name))
        except ImportError:
            pass
    return tuple(errors)


def main(argv=None):
    argv = sys.argv[1:] if argv is None else list(argv)
    if not argv or argv[0] in ("-h", "--help"):
        print(usage())
        return 0 if argv else 2
    name, rest = argv[0], argv[1:]
    if name not in COMMANDS:
        print(f"rebot: unknown command '{name}'\n\n{usage()}", file=sys.stderr)
        return 2
    try:
        module = importlib.import_module(COMMANDS[name][0])
        return module.main(rest, prog=f"rebot {name}") or 0
    except KeyboardInterrupt:
        return 130
    except expected_errors() as exc:
        print(f"rebot {name}: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
