"""Dual-arm robot at the end-effector level: read the 26-value state, command absolute targets.

State and target layout, per arm (left then right), 13 values each:
    position xyz (m, arm base frame), rotation matrix flattened column-major, gripper openness [0, 1].
"""

from contextlib import ExitStack
import math
import sys

import numpy as np
from scipy.spatial.transform import Rotation

from rebot.bindings import SIDES, select_arms
from rebot.devices.robstride import MotorBus
from rebot.model.gravity import GravityModel
from rebot.model.gripper import load_gripper_settings
from rebot.model.kinematics import CartesianIK
from rebot.model.limits import GRIPPER_OPEN_DEG, validate_tracking_error
from rebot.paths import ARM_BINDINGS, CONTROL_SETTINGS, GRIPPER_SETTINGS
from rebot.runtime.arm import Arm, open_arms
from rebot.runtime.worker import ArmWorker
from rebot.settings import load_settings


def decode_targets(values):
    """Split 26 values into two SE(3) targets and two gripper openness values."""
    values = np.asarray(values, dtype=float)
    if values.shape != (26,) or not np.all(np.isfinite(values)):
        raise ValueError("Target must contain 26 finite absolute EEF state values.")
    poses, openness = [], []
    for state in values.reshape(2, 13):
        rotation = state[3:12].reshape(3, 3, order="F")
        if (not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-3, rtol=0)
                or not np.isclose(np.linalg.det(rotation), 1., atol=1e-3, rtol=0)):
            raise ValueError("Target orientations must be proper column-major rotations.")
        pose = np.eye(4)
        pose[:3, 3] = state[:3]
        pose[:3, :3] = Rotation.from_matrix(rotation).as_matrix()
        poses.append(pose)
        openness.append(float(np.clip(state[12], 0., 1.)))
    return poses, openness


class Robot:
    """Both physical arms and grippers behind `state()` and `command()`.

    `on_feedback(side, states, command)` receives every servo cycle's raw motor replies and the
    joint command they answer, e.g. to record them.
    """

    def __init__(self, *, stop, config=ARM_BINDINGS, settings=CONTROL_SETTINGS,
                 gripper_settings=GRIPPER_SETTINGS, gripper_open_deg=GRIPPER_OPEN_DEG,
                 bus_factory=MotorBus, on_feedback=None):
        self.stop, self.bus_factory, self.on_feedback = stop, bus_factory, on_feedback
        self.config, self.gripper_settings = config, gripper_settings
        self.gripper_open = math.radians(gripper_open_deg)
        self.gripper_open_deg = gripper_open_deg
        self.cfg = load_settings(settings)
        self.stack = ExitStack()
        self.iks, self.workers = {}, {}

    def __enter__(self):
        try:
            bindings = select_arms(self.config, "both")
            gripper = load_gripper_settings(self.gripper_settings)
            arms = {side: Arm(bindings[side], self.cfg, gravity=GravityModel(), gripper=gripper,
                              gripper_open_deg=self.gripper_open_deg, stop=self.stop,
                              bus_factory=self.bus_factory) for side in SIDES}
            self.stack.enter_context(open_arms(arms))
            # Verify both arms and warm both IK solvers before enabling either arm.
            for side, arm in arms.items():
                arm.prepare()
                arm.sample()
                self.iks[side] = CartesianIK(arm.q[:6], arm.range.lower[:6], arm.range.upper[:6])
            for side, arm in arms.items():
                self.check()
                self._interruptible()
                arm.enable()
                self.iks[side].reset(arm.q[:6])
                self._interruptible()
                if not arm.refresh():
                    raise InterruptedError("Control startup interrupted.")
                self.workers[side] = self.stack.enter_context(
                    ArmWorker(arm, self.stop, on_feedback=self.on_feedback))
            return self
        except BaseException:
            self.__exit__(*sys.exc_info())
            raise

    def __exit__(self, *_):
        self.stop.set()
        self.stack.close()

    def _interruptible(self):
        if self.stop.is_set():
            raise InterruptedError("Control startup interrupted.")

    def check(self):
        for worker in self.workers.values():
            worker.check()

    def _measured(self, side):
        """Joint angles with normalized openness in the last slot, and their timestamp."""
        q, acquired_at = self.workers[side].snapshot()
        q[6] = np.clip(q[6] / self.gripper_open, 0., 1.)
        return q, acquired_at

    def state(self):
        values, times = [], []
        for side in SIDES:
            q, acquired_at = self._measured(side)
            pose = self.iks[side].pose(q[:6])
            values.extend((*pose[:3, 3], *pose[:3, :3].ravel(order="F"), q[6]))
            times.append(acquired_at)
        return {"state": list(map(float, values)), "observed_at": min(times),
                "acquired_at": dict(zip(SIDES, times))}

    def command(self, values):
        """Move both arms to absolute targets. Returns False if a solution self-collides."""
        poses, openness = decode_targets(values)
        pending = {}
        for side, pose, grip in zip(SIDES, poses, openness):
            measured = self._measured(side)[0]
            ik = self.iks[side]
            q = np.clip(ik.solve(pose, init_q=measured[:6], stop=self.stop), ik.lower, ik.upper)
            self.check()
            if self.stop.is_set():
                raise InterruptedError("Control interrupted during IK.")
            if ik.has_self_collision(q):
                return False
            validate_tracking_error(q, measured[:6], self.cfg.max_tracking_error_deg)
            pending[side] = np.append(q, grip * self.gripper_open)
        # Neither arm receives a target until both solutions have passed validation.
        for side in SIDES:
            self.workers[side].command(pending[side])
        return True
