#!/usr/bin/env python3
"""Teleoperate two MuJoCo Star Arms from the calibrated physical leader arms."""
import argparse
from collections import deque
from contextlib import contextmanager, ExitStack
from copy import deepcopy
import math
from pathlib import Path
import struct
import sys
import threading
import time
import xml.etree.ElementTree as ET

import serial
from fashionstar_uart_sdk import UartServoManager
import yaml

from stararm import CONFIG, angle_difference, load_config, read_angles

MODEL = Path(__file__).with_name("models") / "stararm.xml"
SIDES = ("left", "right")
JOINTS = [f"joint{i}" for i in range(1, 7)] + ["joint7_left"]


class HoverState:
    """Hold both arms when still and closed; opening either handle releases both."""

    def __init__(self, references):
        self.references = references
        self.samples = deque(maxlen=25)
        self.previous = None
        self.still_since = None
        self.holding = False
        self.closed = {side: False for side in SIDES}

    def reset_stillness(self):
        self.samples.clear()
        self.previous = None
        self.still_since = None

    def update(self, buses, raw):
        # The calibrated default pose defines each handle's closed position.
        gaps = {side: abs(angle_difference([raw[side][6]], [self.references[side][6]])[0])
                for side in SIDES}
        for side, gap in gaps.items():
            if gap <= 5:
                self.closed[side] = True
            elif gap >= 10:
                self.closed[side] = False

        if self.holding:
            if any(gap >= 10 for gap in gaps.values()):
                for side in SIDES:
                    release_torque(buses[side])
                self.holding = False
                self.reset_stillness()
                print("Handle opened; both hover unlock commands sent.", flush=True)
            return

        # Preserve the existing per-arm stillness thresholds, with one shared timer.
        self.samples.append([angle for side in SIDES for angle in raw[side][:6]])
        filtered = [sum(values) / len(self.samples) for values in zip(*self.samples)]
        changes = ([sum(abs(x) for x in angle_difference(filtered[i:i+6], self.previous[i:i+6]))
                    for i in (0, 6)] if self.previous is not None else [math.inf, math.inf])
        self.previous = filtered
        now = time.monotonic()
        if any(change > .1 for change in changes):
            self.still_since = None
        elif self.still_since is None:
            self.still_since = now
        elif now - self.still_since > 1 and all(self.closed.values()):
            for side in SIDES:
                # Hold raw hardware angles, not the calibrated model offsets.
                commands = [struct.pack("<BiIHHH", i, int(raw[side][i] * 10), 100, 50, 50, 4095)
                            for i in range(6)]
                buses[side].send_sync_multiturnanglebyinterval(14, 6, commands)
            self.holding = True
            print("Both arms still and handles closed; both hover hold commands sent.", flush=True)


def load_model():
    import mujoco

    root = ET.parse(MODEL).getroot()
    root.find("compiler").set("meshdir", str(MODEL.parent / "meshes"))
    world = root.find("worldbody")
    base = world.find("body[@name='base_link']")
    world.remove(base)
    for side, y in zip(SIDES, (.22, -.22)):
        body = deepcopy(base)
        body.set("pos", f"0 {y} 0")
        for element in body.iter():
            if "name" in element.attrib:
                element.set("name", f"{side}_{element.get('name')}")
        world.append(body)
    root.find("keyframe/key[@name='reference']").set("qpos", " ".join(["0"] * 16))
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    return model, mujoco.MjData(model)


def set_model_pose(model, data, degrees):
    import mujoco

    for side in SIDES:
        for name, angle in zip(JOINTS, degrees[side]):
            data.joint(f"{side}_{name}").qpos[0] = math.radians(angle)
        data.joint(f"{side}_joint7_right").qpos[0] = -math.radians(degrees[side][6])
    mujoco.mj_forward(model, data)


def configure_view(viewer):
    with viewer.lock():
        viewer.cam.lookat[:] = [0, 0, .15]
        viewer.cam.distance = 1.2
        viewer.cam.azimuth = 160
        viewer.cam.elevation = -25
    viewer.sync()


@contextmanager
def open_viewer(model, data):
    import mujoco.viewer

    before = set(threading.enumerate())
    viewer = mujoco.viewer.launch_passive(model, data, show_left_ui=False, show_right_ui=False)
    threads = set(threading.enumerate()) - before
    try:
        configure_view(viewer)
        yield viewer
    finally:
        viewer.close()
        # Wait for rendering to finish before Python's GLFW teardown.
        for thread in threads:
            thread.join()


def release_torque(bus):
    # Use the official broadcast stop instead of a burst of per-servo packets.
    bus.stop_on_control_mode(0xff, 0x10, 0)
    bus.uart.flush()
    time.sleep(.01)


def release_on_exit(side, bus):
    try:
        release_torque(bus)
    except Exception as exc:
        print(f"{side}: exit unlock failed: {exc}", file=sys.stderr, flush=True)
    else:
        print(f"{side}: exit unlock command sent.", flush=True)


def teleop(mode="observe", config=CONFIG):
    if mode not in ("observe", "hover"):
        raise ValueError(f"Unknown teleoperation mode: {mode}")
    arms = load_config(config)
    references = {}
    for side in SIDES:
        if side not in arms:
            raise ValueError(f"Bind {side} before starting dual-arm teleoperation.")
        reference = arms[side].get("reference_deg")
        if (not isinstance(reference, list) or len(reference) != 7
                or any(not isinstance(x, (int, float)) or not math.isfinite(x) for x in reference)):
            raise ValueError(f"Run 'python stararm.py calibrate {side}' before teleoperation.")
        references[side] = reference
    if Path(arms["left"]["port"]).resolve() == Path(arms["right"]["port"]).resolve():
        raise ValueError("The two arms must be bound to different serial ports.")
    model, data = load_model()
    with ExitStack() as stack:
        buses = {}
        for side in SIDES:
            arm = arms[side]
            uart = stack.enter_context(serial.Serial(arm["port"], baudrate=arm["baudrate"],
                                                      timeout=0, exclusive=True))
            buses[side] = UartServoManager(uart)
            if mode == "hover":
                # Registered after the serial context: unlock before closing it.
                stack.callback(release_on_exit, side, buses[side])
        raw = {side: read_angles(buses[side]) for side in SIDES}
        print("Releasing both arms before opening the viewer. Support the physical arms.", flush=True)
        for side, bus in buses.items():
            release_torque(bus)
            print(f"{side}: startup unlock command sent.", flush=True)
        # Follow the ROS driver: servo degrees map directly to URDF joint angles.
        set_model_pose(model, data, {side: angle_difference(raw[side], references[side]) for side in SIDES})
        with open_viewer(model, data) as viewer:
            hovering = HoverState(references) if mode == "hover" else None
            if mode == "hover":
                print("Hover enabled: both arms still and both handles closed -> hold both arms. "
                      "Open either handle -> release both arms. Exit unlocks both arms.", flush=True)
            print("Dual-arm teleoperation started. Move both arms; close the viewer or Ctrl-C to stop.")
            started, frames, failures = time.monotonic(), 0, 0
            last_valid = started
            while viewer.is_running():
                tick = time.monotonic()
                try:
                    raw = {}
                    for side in SIDES:
                        raw[side] = read_angles(buses[side])
                except ValueError as exc:
                    if hovering is not None:
                        hovering.reset_stillness()
                    failures += 1
                    elapsed = time.monotonic() - last_valid
                    detail = f"{side}: {exc}"
                    print(f"Feedback retry {failures}: {detail} "
                          f"({elapsed:.2f}s since last complete frame); models frozen.",
                          file=sys.stderr, flush=True)
                    # One SDK read can itself block for ~0.8 seconds. Require
                    # repeated failed reads as well as sustained loss of data.
                    if failures >= 3 and elapsed >= 2:
                        raise ValueError(f"Feedback did not recover after {failures} reads "
                                         f"and {elapsed:.2f}s; stopping. Last error: {detail}") from exc
                else:
                    last_valid = time.monotonic()
                    if failures:
                        print(f"Feedback recovered after {failures} failed read(s); resuming.", flush=True)
                    failures = 0
                    if hovering is not None:
                        hovering.update(buses, raw)
                    angles = {side: angle_difference(raw[side], references[side]) for side in SIDES}
                    with viewer.lock():
                        set_model_pose(model, data, angles)
                        data.time = time.monotonic() - started
                    frames += 1
                viewer.sync()
                time.sleep(max(0, 1 / 30 - (time.monotonic() - tick)))
            print(f"Received {frames} paired frames in {time.monotonic() - started:.1f} seconds.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("observe", "hover"), default="observe",
                        help="observe: release torque for manual movement (default); hover: paired stillness and handle-controlled pose holding")
    args = parser.parse_args()
    try:
        teleop(mode=args.mode)
        return 0
    except (OSError, ValueError, RuntimeError, EOFError, ImportError, yaml.YAMLError, serial.SerialException) as exc:
        print(str(exc), file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
