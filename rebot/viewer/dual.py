"""Two-arm scenes for the read-only 3D viewer: build the model and pose it from feedback."""

from copy import deepcopy
import math
import xml.etree.ElementTree as ET

from rebot.bindings import SIDES
from rebot.model.leader_map import DIRECTIONS, GRIPPER_RATIO
from rebot.model.limits import GRIPPER_OPEN_DEG
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
    # ABC-130K workstation base frames: 620 mm apart, 750 mm above the floor.
    for side, y in zip(SIDES, (.31, -.31) if target == "rebot" else (.22, -.22)):
        body = deepcopy(base)
        body.set("pos", f"0.2525 {y} 0.75" if target == "rebot" else f"0 {y} 0")
        for element in body.iter():
            if "name" in element.attrib:
                element.set("name", f"{side}_{element.get('name')}")
        world.append(body)
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    return model, mujoco.MjData(model)


def _set_fingers(data, side, openness):
    for name in FINGER_JOINTS:
        data.joint(f"{side}_{name}").qpos[0] = max(0, min(1, openness)) * FINGER_TRAVEL_M


def pose_from_leaders(model, data, degrees, target="stararm"):
    """Pose either scene from leader angles (deg, relative to the leader reference)."""
    import mujoco

    for side in SIDES:
        if target == "stararm":
            for name, angle in zip(JOINTS, degrees[side]):
                data.joint(f"{side}_{name}").qpos[0] = math.radians(angle)
            data.joint(f"{side}_joint7_left").qpos[0] = math.radians(degrees[side][6])
            data.joint(f"{side}_joint7_right").qpos[0] = -math.radians(degrees[side][6])
            continue
        # RS follower mapping: apply signs in motor coordinates before model limits.
        for name, sign, angle in zip(JOINTS, DIRECTIONS, degrees[side]):
            joint = model.joint(f"{side}_{name}")
            low, high = joint.range
            data.joint(joint.name).qpos[0] = max(low, min(high, math.radians(sign * angle)))
        # The follower's gripper motor target spans 0..270 degrees. Visualize
        # that as the model's finger slide; this is a linear preview mapping.
        _set_fingers(data, side, degrees[side][6] * GRIPPER_RATIO / GRIPPER_OPEN_DEG)
    mujoco.mj_forward(model, data)


def pose_from_motors(model, data, raw, references, gripper_open_rad=math.radians(GRIPPER_OPEN_DEG)):
    """Pose the reBot scene from raw motor angles (rad). Returns the joints clipped at model limits."""
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
        _set_fingers(data, side, relative[side][6] / gripper_open_rad)
    data.qvel[:] = 0
    mujoco.mj_forward(model, data)
    return clipped
