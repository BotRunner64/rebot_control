#!/usr/bin/env python3
"""Dual-arm EEF state and target execution using the existing IK and CAN loop."""

from contextlib import ExitStack
import math
import sys
import threading
import time

import numpy as np
from scipy.spatial.transform import Rotation

from rebot import CONFIG, load_config, resolve_channel, verify_identity
from rebot_ee_runtime import GravityFeedforward, MotorWorker
from rebot_gravity import GravityModel
from rebot_gripper import GripperController, SETTINGS_PATH as GRIPPER_SETTINGS, load_settings as load_gripper
from rebot_hand_guiding import SETTINGS_PATH, load_settings
from rebot_kinematics import CartesianIK, SIM_START
from rebot_mit import LIMITS, MotorBus
from rebot_motion import MOTION_KD, MOTION_KP
from rebot_safety import validate_tracking_error
from rebot_teleop import position_bounds, validate_pose


SIDES = ("left", "right")


class Robot:
    def __init__(self, args, stop, *, bus_factory=MotorBus):
        self.args, self.stop, self.bus_factory = args, stop, bus_factory
        self.cfg = load_settings(args.settings)
        self.stack = ExitStack()
        self.iks, self.motors = {}, {}
        self.sim_q = {side: np.append(SIM_START, .5) for side in SIDES}
        self.sim_lock = threading.Lock()

    def __enter__(self):
        try:
            if not self.args.hardware:
                self.iks = {side: CartesianIK(SIM_START) for side in SIDES}
                return self
            arms = load_config(self.args.config)
            if any(len(arms.get(side, {}).get("reference_rad", [])) != 7 for side in SIDES):
                raise ValueError("Bind and calibrate all seven motors on both arms first.")
            channels = {side: resolve_channel(arms[side]) for side in SIDES}
            if len(set(channels.values())) != 2:
                raise ValueError("The two arms must use distinct CAN channels.")
            if set(arms["left"]["motor_uids"].values()) & set(arms["right"]["motor_uids"].values()):
                raise ValueError("The two arms must use distinct motor identities.")
            settings = load_gripper(self.args.gripper_settings)
            prepared = {}
            for side in SIDES:
                arm = arms[side]
                settings.validate_motor(LIMITS[arm["models"][6]])
                reference, low, high = position_bounds(arm, 7, self.args.gripper_open_deg)
                bus = self.stack.enter_context(self.bus_factory(
                    channels[side], arm["models"], motor_count=7,
                    max_temperature=self.cfg["max_temperature_c"]))
                gravity = GravityFeedforward(GravityModel(), reference[:6], self.cfg)
                support = lambda raw, gravity=gravity: np.append(gravity(raw[:6]), 0.)
                kp, kd = np.append(MOTION_KP, settings.kp), np.append(MOTION_KD, settings.kd)
                verify_identity(bus, arm)
                raw = validate_pose(bus.positions(), reference, low, high)
                bus.set_hold(raw, support(raw), kp, kd)
                bus.hold_gravity = support
                bus.prepare()
                raw = validate_pose(bus.positions(), reference, low, high)
                self.iks[side] = CartesianIK(raw[:6] - reference[:6], low[:6], high[:6])
                gripper = GripperController(reference[6] + low[6], reference[6] + high[6], settings)
                prepared[side] = bus, reference, low, high, support, gripper, kp, kd
            # Verify both arms and warm both IK solvers before enabling either arm.
            for side in SIDES:
                bus, reference, low, high, support, gripper, kp, kd = prepared[side]
                self.check()
                if self.stop.is_set():
                    raise InterruptedError("Control startup interrupted.")
                raw = validate_pose(bus.positions(), reference, low, high)
                self.iks[side].reset(raw[:6] - reference[:6])
                bus.set_hold(raw, support(raw), kp, kd)
                bus.enable(raw)
                if self.stop.is_set():
                    raise InterruptedError("Control startup interrupted.")
                bus.command_position(raw, np.zeros(7), support(raw), kp, kd,
                                     time.monotonic() + self.cfg["max_feedback_age_s"])
                self.motors[side] = self.stack.enter_context(MotorWorker(
                    bus, reference, low, high, self.cfg, self.stop, support, gripper=gripper))
            return self
        except BaseException:
            self.__exit__(*sys.exc_info())
            raise

    def __exit__(self, *_):
        self.stop.set()
        self.stack.close()

    def check(self):
        for motor in self.motors.values():
            motor.check()

    def state(self):
        values, times = [], []
        if not self.args.hardware:
            with self.sim_lock:
                pair = {side: q.copy() for side, q in self.sim_q.items()}
        for side in SIDES:
            if self.args.hardware:
                q, acquired_at = self.motors[side].snapshot()
                openness = np.clip(q[6] / math.radians(self.args.gripper_open_deg), 0., 1.)
            else:
                q, acquired_at = pair[side], time.monotonic()
                openness = q[6]
            pose = self.iks[side].pose(q[:6])
            values.extend((*pose[:3, 3], *pose[:3, :3].ravel(order="F"), openness))
            times.append(acquired_at)
        return {"state": list(map(float, values)), "observed_at": min(times)}

    def command(self, value):
        values = np.asarray(value, dtype=float)
        if values.shape != (26,) or not np.all(np.isfinite(values)):
            raise ValueError("Target must contain 26 finite absolute EEF state values.")
        targets = []
        for state in values.reshape(2, 13):
            rotation = state[3:12].reshape(3, 3, order="F")
            if (not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-3, rtol=0)
                    or not np.isclose(np.linalg.det(rotation), 1., atol=1e-3, rtol=0)):
                raise ValueError("Target orientations must be proper column-major rotations.")
            target = np.eye(4)
            target[:3, 3] = state[:3]
            target[:3, :3] = Rotation.from_matrix(rotation).as_matrix()
            targets.append(target)
        pending = {}
        for index, side in enumerate(SIDES):
            measured = self.motors[side].read() if self.args.hardware else self.sim_q[side]
            ik = self.iks[side]
            q = np.clip(ik.solve(targets[index], init_q=measured[:6], stop=self.stop), ik.lower, ik.upper)
            self.check()
            if self.stop.is_set():
                raise InterruptedError("Control interrupted during IK.")
            if ik.has_self_collision(q):
                return False
            validate_tracking_error(q, measured[:6], self.cfg["max_tracking_error_deg"])
            openness = np.clip(values[index * 13 + 12], 0., 1.)
            grip = openness * math.radians(self.args.gripper_open_deg) if self.args.hardware else openness
            pending[side] = np.append(q, grip)
        # Neither arm receives a target until both solutions have passed validation.
        if self.args.hardware:
            for side in SIDES:
                self.motors[side].command(pending[side])
        else:
            with self.sim_lock:
                self.sim_q.update(pending)
        return True
