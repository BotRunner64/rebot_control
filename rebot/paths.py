"""Repository locations shared by every module. Standard library only."""

import os
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
MODELS = ROOT / "models"
WEB = ROOT / "web"
RUNS = ROOT / "runs"
CONFIG = ROOT / "config"

ARM_BINDINGS = CONFIG / "arm.local.yaml"
LEADER_BINDINGS = CONFIG / "leader.local.yaml"
CONTROL_SETTINGS = CONFIG / "control.yaml"
GRIPPER_SETTINGS = CONFIG / "gripper.yaml"

# GStreamer's Python bindings live in the system interpreter, not the Conda environment.
SYSTEM_PYTHON = "/usr/bin/python3"


def system_python(module, *args, flags=("-B",)):
    """Return (command, env) that runs a `rebot.media` module under the system interpreter."""
    path = os.pathsep.join(filter(None, (str(ROOT), os.environ.get("PYTHONPATH"))))
    return [SYSTEM_PYTHON, *flags, "-m", module, *args], dict(os.environ, PYTHONPATH=path)
