"""Two-arm scenes for the read-only 3D viewer: build the model and pose it from feedback."""

import math

from rebot.bindings import SIDES
from rebot.model.leader_map import DIRECTIONS, GRIPPER_RATIO
from rebot.model.limits import GRIPPER_OPEN_DEG
from rebot.model.workstation import JOINTS, SCENES, load_dual_model, set_fingers

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
        set_fingers(data, side, degrees[side][6] * GRIPPER_RATIO / GRIPPER_OPEN_DEG)
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
        set_fingers(data, side, relative[side][6] / gripper_open_rad)
    data.qvel[:] = 0
    mujoco.mj_forward(model, data)
    return clipped
