"""Iterative Mink IK with a MuJoCo computation backend for the six RS arm joints."""

from pathlib import Path
import math
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

from rebot_teleop import JOINT_LIMITS_DEG


MODEL_PATH = Path(__file__).with_name("models") / "rebot_rs.xml"
EE_FRAME = "gripper_mount"
JOINT_NAMES = tuple(f"joint{i}" for i in range(1, 7))
SIM_START = np.zeros(6)


def vector6(value, name):
    result = np.asarray(value, dtype=float)
    if result.shape != (6,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain six finite values.")
    return result.copy()


def pose_matrix(values):
    """XYZ in meters and extrinsic XYZ roll/pitch/yaw in degrees, in base frame."""
    values = vector6(values, "Pose")
    pose = np.eye(4)
    pose[:3, :3] = Rotation.from_euler("xyz", values[3:], degrees=True).as_matrix()
    pose[:3, 3] = values[:3]
    return pose


def validate_transform(value):
    pose = np.asarray(value, dtype=float)
    if (pose.shape != (4, 4) or not np.all(np.isfinite(pose))
            or not np.allclose(pose[3], [0, 0, 0, 1], atol=1e-8, rtol=0)
            or not np.allclose(pose[:3, :3].T @ pose[:3, :3], np.eye(3), atol=1e-7, rtol=0)
            or not np.isclose(np.linalg.det(pose[:3, :3]), 1., atol=1e-7, rtol=0)):
        raise ValueError("Target must be a finite SE(3) transform.")
    return pose.copy()


def build_model(lower, upper, tcp_offset=(0., 0., 0.)):
    """Fix visual fingers and apply a TCP offset in the fixed mounting frame."""
    import mujoco

    lower, upper = vector6(lower, "Lower limits"), vector6(upper, "Upper limits")
    offset = np.asarray(tcp_offset, dtype=float)
    if offset.shape != (3,) or not np.all(np.isfinite(offset)) or np.any(lower >= upper):
        raise ValueError("Invalid TCP offset or joint limits.")
    root = ET.parse(MODEL_PATH).getroot()
    root.find("compiler").set("meshdir", str(MODEL_PATH.parent / "rebot_rs_meshes"))
    for parent in root.iter():
        for joint in list(parent.findall("joint")):
            name = joint.get("name")
            if name not in JOINT_NAMES:
                parent.remove(joint)
            else:
                index = JOINT_NAMES.index(name)
                joint.set("range", f"{lower[index]:.17g} {upper[index]:.17g}")
    tcp = root.find(f".//body[@name='{EE_FRAME}']/site[@name='tcp']")
    tcp.set("pos", " ".join(map(str, offset)))
    # Check target mesh contacts while keeping display geometry unchanged.
    for geom in root.findall(".//geom[@type='mesh']"):
        geom.set("contype", "1")
        geom.set("conaffinity", "1")
    world = root.find("worldbody")
    world.remove(world.find("geom[@name='work_table']"))
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    if model.nq != 6 or model.nv != 6 or tuple(model.joint(i).name for i in range(6)) != JOINT_NAMES:
        raise ValueError("Expected exactly the six ordered reBot arm joints.")
    return model


class CartesianIK:
    """Solve each target in a tight numerical loop, then return the joint solution directly."""

    def __init__(self, start, lower=None, upper=None, tcp_offset=(0., 0., 0.)):
        import mink
        import mujoco
        import qpsolvers

        limits = np.radians(JOINT_LIMITS_DEG[:6])
        self.lower = vector6(limits[:, 0] if lower is None else lower, "Lower limits")
        self.upper = vector6(limits[:, 1] if upper is None else upper, "Upper limits")
        if "quadprog" not in qpsolvers.available_solvers:
            raise RuntimeError("The quadprog QP solver is required; install the ee dependency extra.")
        self.model = build_model(self.lower, self.upper, tcp_offset)
        self.configuration = mink.Configuration(self.model)
        self.data = mujoco.MjData(self.model)
        self._fk_data = mujoco.MjData(self.model)
        self._check_data = mujoco.MjData(self.model)
        self.task = mink.FrameTask("tcp", "site", position_cost=1., orientation_cost=1.,
                                   lm_damping=1.)
        self.limits = [mink.ConfigurationLimit(self.model)]
        self.converged = False
        self.iterations = 0
        self.reset(start)
        # Warm up the solver before any hardware is enabled.
        self.solve(self.pose())
        self.reset(start)
        self.update_display()

    @property
    def q(self):
        return self.configuration.q.copy()

    def validate(self, q, tolerance=1e-8):
        q = vector6(q, "Joint angles")
        if np.any(q < self.lower - tolerance) or np.any(q > self.upper + tolerance):
            raise ValueError("Joint angles are outside the EE control limits.")
        return q

    def reset(self, q):
        self.configuration.update(vector6(q, "Joint angles"))

    def pose(self, q=None):
        import mujoco

        self._fk_data.qpos[:] = self.q if q is None else self.validate(q, tolerance=.03)
        mujoco.mj_forward(self.model, self._fk_data)
        pose = np.eye(4)
        site = self._fk_data.site("tcp")
        pose[:3, :3] = site.xmat.reshape(3, 3)
        pose[:3, 3] = site.xpos
        return pose

    def update_display(self, q=None):
        import mujoco

        self.data.qpos[:] = self.q if q is None else self.validate(q, tolerance=.03)
        mujoco.mj_forward(self.model, self.data)

    def error(self, target, q=None):
        target = validate_transform(target)
        actual = self.pose(q)
        return (float(np.linalg.norm(target[:3, 3] - actual[:3, 3])),
                float(Rotation.from_matrix(target[:3, :3] @ actual[:3, :3].T).magnitude()))

    def has_self_collision(self, q):
        """Check the target, ignoring planes and adjacent rigid links."""
        import mujoco

        self._check_data.qpos[:] = vector6(q, "Joint target")
        mujoco.mj_forward(self.model, self._check_data)
        for contact in self._check_data.contact:
            if contact.dist >= -1e-3:
                continue
            first, second = contact.geom1, contact.geom2
            if (self.model.geom_type[first] == mujoco.mjtGeom.mjGEOM_PLANE
                    or self.model.geom_type[second] == mujoco.mjtGeom.mjGEOM_PLANE):
                continue
            # Fixed gripper parts belong to link6's rigid body for adjacency.
            first, second = self.model.body_weldid[self.model.geom_bodyid[[first, second]]]
            if (first == second or self.model.body_weldid[self.model.body_parentid[first]] == second
                    or self.model.body_weldid[self.model.body_parentid[second]] == first):
                continue
            return True
        return False

    def solve(self, target, *, init_q=None, stop=None, dt=.01, max_iters=200):
        """Match YAM's IK defaults; dt is numerical, not a real-time motion interval."""
        import mink

        target = validate_transform(target)
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError("IK timestep must be finite and positive.")
        if type(max_iters) is not int or max_iters <= 0:
            raise ValueError("IK iteration count must be a positive integer.")
        if init_q is not None:
            self.reset(init_q)
        previous = self.q
        self.converged = False
        self.iterations = 0
        self.task.set_target(mink.SE3.from_matrix(target))
        try:
            for iteration in range(max_iters):
                if stop is not None and stop.is_set():
                    break
                velocity = vector6(mink.solve_ik(
                    self.configuration, [self.task], dt, "quadprog", damping=1e-4,
                    limits=self.limits), "IK velocity")
                self.configuration.integrate_inplace(velocity, dt)
                self.iterations = iteration + 1
                error = self.task.compute_error(self.configuration)
                if np.linalg.norm(error[:3]) <= 1e-4 and np.linalg.norm(error[3:]) <= 1e-4:
                    self.converged = True
                    break
        except Exception as exc:
            self.configuration.update(previous)
            raise RuntimeError(f"Mink IK solve failed: {exc}") from exc
        return self.q
