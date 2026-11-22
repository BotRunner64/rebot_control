"""Check single-arm motions against the mounted robot and workstation geometry."""

import math

import mujoco
import numpy as np

from rebot.model.arrays import vector6
from rebot.model.limits import GRIPPER_OPEN_DEG
from rebot.model.workstation import JOINTS, load_dual_model, set_fingers


# Match the EE controller's tolerance for mesh contact at the folded pose.
PENETRATION_M = .001
PATH_STEP_RAD = math.radians(.5)


class MotionCollision:
    """Geometric preflight with a closed active gripper and a stationary other arm."""

    def __init__(self, side, parked):
        model, self.data = load_dual_model("rebot")
        self.model = model
        self.side = side
        other = "left" if side == "right" else "right"
        self.joints = [model.jnt_qposadr[model.joint(f"{side}_{j}").id] for j in JOINTS]
        if parked is not None:
            parked = np.asarray(parked, dtype=float)
            if parked.shape != (7,) or not np.isfinite(parked).all():
                raise ValueError(f"Expected seven finite measured angles for the parked {other} arm.")
            for j, angle in zip(JOINTS, parked[:6]):
                self.data.joint(f"{other}_{j}").qpos[0] = angle
            set_fingers(self.data, other, parked[6] / math.radians(GRIPPER_OPEN_DEG))

        # With the active gripper closed its fixed parts and fingers form one tool.
        # Assign that tool and each link's meshes to the nearest arm hinge.
        links = {model.body(f"{side}_base_link").id: 0}
        for i, j in enumerate(JOINTS, 1):
            links[model.jnt_bodyid[model.joint(f"{side}_{j}").id]] = i
        groups = []
        self.names = []
        for i, body in enumerate(model.geom_bodyid):
            name = model.body(body).name
            mesh = model.geom_dataid[i]
            self.names.append(f"{name}/{model.mesh(mesh).name}" if mesh >= 0 else model.geom(i).name)
            while body and body not in links:
                body = model.body_parentid[body]
            groups.append(links.get(body))
        pairs = []
        for i, link in enumerate(groups):
            if link is None:
                continue
            for j, peer in enumerate(groups):
                if peer is not None and (j <= i or abs(link - peer) <= 1):
                    continue
                if link == 0 and model.geom(j).name == "work_table":
                    continue  # The base is bolted to the tabletop.
                if parked is None and model.body(model.geom_bodyid[j]).name.startswith(other + "_"):
                    continue
                pairs.append((i, j))
        self.first, self.second = np.array(pairs).T

    def check_pose(self, q, waypoint):
        self.data.qpos[self.joints] = q
        mujoco.mj_kinematics(self.model, self.data)
        # MuJoCo distance queries work independently of the display model's contact masks.
        rotation = self.data.geom_xmat.reshape(-1, 3, 3)
        boxes = self.model.geom_aabb
        center = self.data.geom_xpos + (rotation @ boxes[:, :3, None]).squeeze(2)
        extent = (np.abs(rotation) @ boxes[:, 3:, None]).squeeze(2)
        a, b = self.first, self.second
        overlap = np.all(np.abs(center[a] - center[b]) <= extent[a] + extent[b], axis=1)
        for i, j in zip(a[overlap], b[overlap]):
            distance = mujoco.mj_geomDistance(self.model, self.data, i, j, PENETRATION_M, None)
            if distance < -PENETRATION_M:
                raise ValueError(f"{self.side} waypoint {waypoint}: collision between {self.names[i]} "
                                 f"and {self.names[j]} ({-1000 * distance:.1f} mm penetration).")

    def check_segment(self, start, goal, waypoint):
        start, goal = vector6(start, "start"), vector6(goal, "goal")
        steps = math.ceil(np.max(np.abs(goal - start)) / PATH_STEP_RAD)
        for q in np.linspace(start, goal, steps + 1):
            self.check_pose(q, waypoint)

    def check_plan(self, current, poses):
        for pose in poses:
            if pose["stage"] != "entry":
                self.check_segment(current, pose["q"], pose["id"])
            current = pose["q"]
