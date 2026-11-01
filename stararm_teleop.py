#!/usr/bin/env python3
"""Teleoperate two MuJoCo Star Arms from the calibrated physical leader arms."""
import argparse
from contextlib import contextmanager, ExitStack
from copy import deepcopy
import math
from pathlib import Path
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


def release_if_requested(buses, unlock):
    if unlock:
        input("Support both physical arms. Press Enter to release servo torque, or Ctrl-C to cancel: ")
        for bus in buses.values():
            for servo_id in range(7):
                bus.stop_on_control_mode(servo_id, 0x10, 0)
        print("Both arms released. Support them; torque will not be restored automatically.")


def teleop(unlock=False, config=CONFIG):
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
        raw = {side: read_angles(buses[side]) for side in SIDES}
        # Follow the ROS driver: servo degrees map directly to URDF joint angles.
        set_model_pose(model, data, {side: angle_difference(raw[side], references[side]) for side in SIDES})
        with open_viewer(model, data) as viewer:
            release_if_requested(buses, unlock)
            print("Dual-arm teleoperation started. Move both arms; close the viewer or Ctrl-C to stop.")
            started, frames, lost_since = time.monotonic(), 0, None
            while viewer.is_running():
                tick = time.monotonic()
                try:
                    raw = {side: read_angles(buses[side]) for side in SIDES}
                except ValueError:
                    if lost_since is None:
                        lost_since = tick
                        print("Feedback lost; both models frozen at the last valid poses.", file=sys.stderr)
                    if time.monotonic() - lost_since > .5:
                        raise ValueError("Feedback did not recover; teleoperation stopped.")
                else:
                    lost_since = None
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
    parser.add_argument("--unlock", action="store_true", help="Release both arms after a support-arms prompt")
    args = parser.parse_args()
    try:
        teleop(args.unlock)
        return 0
    except (OSError, ValueError, RuntimeError, EOFError, ImportError, yaml.YAMLError, serial.SerialException) as exc:
        print(str(exc), file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
