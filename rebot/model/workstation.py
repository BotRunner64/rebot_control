"""Shared MuJoCo workstation layout for dual-arm previews and motion checks."""

from copy import deepcopy
import xml.etree.ElementTree as ET

from rebot.bindings import SIDES
from rebot.paths import MODELS

SCENES = {"stararm": MODELS / "stararm.xml", "rebot": MODELS / "rebot_rs.xml"}
JOINTS = [f"joint{i}" for i in range(1, 7)]
FINGER_JOINTS = ("joint7", "joint_left", "joint_right")
FINGER_TRAVEL_M = .05


def load_dual_model(target="stararm"):
    """Duplicate the single-arm MJCF into `left_*` and `right_*` bodies."""
    import mujoco

    path = SCENES[target]
    root = ET.parse(path).getroot()
    compiler = root.find("compiler")
    compiler.set("meshdir", str(path.parent / compiler.get("meshdir")))
    world = root.find("worldbody")
    base = world.find("body[@name='base_link']")
    world.remove(base)
    mount = world
    if target == "rebot":
        mount = ET.parse(MODELS / "rebot_workstation.xml").getroot()
        world.append(mount)
        table = world.find("geom[@name='work_table']")
        world.remove(table)
        half_depth, _, half_thickness = map(float, table.get("size").split())
        # The rear edge is 70 mm behind the base centers in the layout model.
        table.set("pos", f"{half_depth - .070} 0 {-half_thickness}")
        mount.append(table)
    for side, y in zip(SIDES, (.31, -.31) if target == "rebot" else (.22, -.22)):
        body = deepcopy(base)
        body.set("pos", f"0 {y} 0")
        for element in body.iter():
            if "name" in element.attrib:
                element.set("name", f"{side}_{element.get('name')}")
        mount.append(body)
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    return model, mujoco.MjData(model)


def set_fingers(data, side, openness):
    for name in FINGER_JOINTS:
        data.joint(f"{side}_{name}").qpos[0] = max(0, min(1, openness)) * FINGER_TRAVEL_M
