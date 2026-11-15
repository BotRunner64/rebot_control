"""Machine-local bindings: which adapter, motors, and software reference belong to each arm.

Both files live in `config/*.local.yaml`, are ignored by Git, and are written
only by `rebot arm` / `rebot leader`.
"""

from dataclasses import dataclass
import math
import os
from pathlib import Path
import tempfile

import numpy as np
import yaml

from rebot.devices import socketcan
from rebot.devices.robstride import HOST_ID, MODELS, MOTOR_IDS
from rebot.paths import ARM_BINDINGS, LEADER_BINDINGS

SIDES = ("left", "right")


def select_sides(side):
    """Expand a CLI side choice into the ordered tuple of arm names."""
    if side == "both":
        return SIDES
    if side not in SIDES:
        raise ValueError("Select left, right, or both arms.")
    return (side,)


def _finite_list(value, size):
    return (isinstance(value, list) and len(value) == size
            and all(type(x) in (int, float) and math.isfinite(x) for x in value))


def _load(path):
    path = Path(path)
    arms = yaml.safe_load(path.read_text()) if path.exists() else {}
    if not isinstance(arms, dict) or any(side not in SIDES or not isinstance(arm, dict)
                                         for side, arm in arms.items()):
        raise ValueError(f"Invalid configuration: {path}")
    return arms


def _save(arms, path):
    """Replace the file atomically, so an interrupted save never truncates calibration."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", dir=path.parent, delete=False) as stream:
            temporary = Path(stream.name)
            yaml.safe_dump(arms, stream, sort_keys=False)
        os.replace(temporary, path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


# --- reBot follower arms -----------------------------------------------------

def load_arms(path=ARM_BINDINGS):
    """Validated raw follower records, as stored on disk."""
    arms = _load(path)
    for side, arm in arms.items():
        if (not isinstance(arm.get("channel"), str) or not isinstance(arm.get("device_path"), str)
                or arm.get("bitrate") != socketcan.BITRATE or arm.get("host_id") != HOST_ID
                or arm.get("motor_ids") != list(MOTOR_IDS) or arm.get("models") != list(MODELS)
                or not isinstance(arm.get("motor_uids"), dict)
                or set(arm["motor_uids"]) != set(MOTOR_IDS)):
            raise ValueError(f"Invalid {side} binding: {path}")
        uids = list(arm["motor_uids"].values())
        if (any(not isinstance(uid, str) or len(uid) != 16
                or any(c not in "0123456789abcdef" for c in uid) for uid in uids)
                or len(set(uids)) != 7):
            raise ValueError(f"Invalid {side} motor UIDs: {path}")
        if "reference_rad" in arm and not _finite_list(arm["reference_rad"], 7):
            raise ValueError(f"Invalid {side} software reference: {path}")
    return arms


def save_arms(arms, path=ARM_BINDINGS):
    _save(arms, path)


def new_arm_record(channel, motor_uids):
    return dict(channel=channel, device_path=socketcan.device_path(channel),
                bitrate=socketcan.BITRATE, host_id=HOST_ID, motor_ids=list(MOTOR_IDS),
                models=list(MODELS), motor_uids=motor_uids)


def resolve_channel(arm):
    """Find the bound adapter by USB path; interface numbering is not stable."""
    if not arm["device_path"]:
        return arm["channel"]
    matches = [name for name in socketcan.channels()
               if socketcan.device_path(name) == arm["device_path"]]
    if len(matches) != 1:
        raise ValueError("Bound CAN adapter is missing or ambiguous; check its USB connection.")
    return matches[0]


@dataclass(frozen=True, eq=False)
class ArmBinding:
    """One follower arm, ready to open: resolved channel, identity, and reference."""
    side: str
    channel: str
    models: tuple
    motor_uids: dict
    reference: np.ndarray | None        # seven raw motor angles (rad) at the reference pose

    def verify(self, bus):
        """Refuse to act on an arm whose motors are not the bound ones."""
        if bus.scan(strict=False) != self.motor_uids:
            raise ValueError(f"{self.side}: motor UIDs or online IDs changed; check the arm and binding.")


def select_arms(path=ARM_BINDINGS, side="both", *, calibrated=True):
    """Bindings for the selected sides, checked to be distinct physical arms."""
    arms = load_arms(path)
    selected = {}
    for name in select_sides(side):
        if name not in arms:
            raise ValueError(f"Bind the {name} reBot arm first: rebot arm bind {name} <channel>")
        arm = arms[name]
        if calibrated and "reference_rad" not in arm:
            raise ValueError(f"Calibrate the {name} reBot arm first: rebot arm calibrate {name}")
        reference = np.asarray(arm["reference_rad"], dtype=float) if "reference_rad" in arm else None
        selected[name] = ArmBinding(name, resolve_channel(arm), tuple(arm["models"]),
                                    dict(arm["motor_uids"]), reference)
    if len({binding.channel for binding in selected.values()}) != len(selected):
        raise ValueError("Selected arms must use distinct CAN interfaces.")
    uids = [uid for binding in selected.values() for uid in binding.motor_uids.values()]
    if len(set(uids)) != len(uids):
        raise ValueError("Selected arms must use distinct motor identities.")
    return selected


# --- Star Arm leaders --------------------------------------------------------

def load_leaders(path=LEADER_BINDINGS):
    """Validated raw leader records, as stored on disk."""
    arms = _load(path)
    if any(not isinstance(arm.get("port"), str) or not isinstance(arm.get("baudrate"), int)
           or arm["baudrate"] <= 0 for arm in arms.values()):
        raise ValueError(f"Invalid configuration format: {path}")
    return arms


def save_leaders(arms, path=LEADER_BINDINGS):
    _save(arms, path)


@dataclass(frozen=True, eq=False)
class LeaderBinding:
    side: str
    port: str
    baudrate: int
    reference_deg: tuple                # seven servo angles (deg) at the reference pose


def select_leaders(path=LEADER_BINDINGS, side="both"):
    """Calibrated leader bindings for the selected sides, on distinct serial ports."""
    arms = load_leaders(path)
    selected = {}
    for name in select_sides(side):
        if name not in arms:
            raise ValueError(f"Bind the {name} Star Arm first: rebot leader bind {name} <port>")
        if not _finite_list(arms[name].get("reference_deg"), 7):
            raise ValueError(f"Calibrate the {name} Star Arm first: rebot leader calibrate {name}")
        selected[name] = LeaderBinding(name, arms[name]["port"], arms[name]["baudrate"],
                                       tuple(arms[name]["reference_deg"]))
    ports = {Path(binding.port).resolve() for binding in selected.values()}
    if len(ports) != len(selected):
        raise ValueError("Selected leaders must use distinct serial ports.")
    return selected
