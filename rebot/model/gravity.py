"""Gravity models and robot-bound static mass calibration profiles."""

import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pinocchio as pin
import yaml

from rebot.model.arrays import vector6
from rebot.paths import MODELS

MODEL_PATH = MODELS / "rebot_rs_dynamics.urdf"
ARM_JOINTS = [f"joint{i}" for i in range(1, 7)]


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def validate_scales(value):
    scales = vector6(value, "mass_scales")
    if np.any(scales < .5) or np.any(scales > 2.) or scales[0] != 1:
        raise ValueError("Mass scales must be in [0.5, 2], with unobservable J1 fixed at 1.")
    return scales


def check_binding(profile, binding):
    """A calibration profile is valid only for the arm it was measured on."""
    expected_uids = {str(key): value for key, value in binding.motor_uids.items()}
    actual_uids = {str(key): value for key, value in profile["motor_uids"].items()}
    if (profile["side"] != binding.side or actual_uids != expected_uids
            or not np.allclose(vector6(profile["reference_rad"], "reference_rad"),
                               vector6(binding.reference[:6], "reference_rad"),
                               rtol=0, atol=1e-8)):
        raise ValueError("Calibration arm, motor UIDs, or reference angles do not match.")


class GravityModel:
    """Six-joint rigid-body gravity torques, optionally with calibrated mass scales."""

    def __init__(self, path=MODEL_PATH, calibration=None, binding=None):
        self.path = Path(path)
        self.sha256 = digest(self.path)
        full = pin.buildModelFromUrdf(str(self.path))
        finger_ids = [i for i, name in enumerate(full.names)
                      if i and name not in ARM_JOINTS]
        self.model = pin.buildReducedModel(full, finger_ids, pin.neutral(full))
        if list(self.model.names)[1:] != ARM_JOINTS:
            raise ValueError("Expected six ordered RS arm joints in dynamics model.")
        self.parameters = np.concatenate([body.toDynamicParameters()
                                          for body in self.model.inertias[1:]])
        self.scales = np.ones(6)
        if calibration is not None:
            profile = yaml.safe_load(Path(calibration).read_text())
            if (not isinstance(profile, dict) or profile.get("version") != 2
                    or profile.get("kind") != "static_mass_scale"
                    or profile.get("method") != "paired_static"
                    or profile.get("accepted") is not True
                    or profile.get("model_sha256") != self.sha256):
                raise ValueError("Calibration must be accepted version 2 paired-static data matching the original model hash.")
            if binding is None:
                raise ValueError("Loading calibration requires the selected arm binding.")
            check_binding(profile, binding)
            self.apply_scales(profile["mass_scales"])
        self.data = self.model.createData()
        self.lower = self.model.lowerPositionLimit.copy()
        self.upper = self.model.upperPositionLimit.copy()

    def apply_scales(self, scales):
        self.scales = validate_scales(scales)
        for index, scale in enumerate(self.scales, 1):
            self.model.inertias[index] = pin.Inertia.FromDynamicParameters(
                self.parameters[10 * (index - 1):10 * index] * scale)

    def basis(self, q):
        zero = np.zeros(6)
        regressor = pin.computeJointTorqueRegressor(
            self.model, self.data, vector6(q, "q"), zero, zero).copy()
        return np.column_stack([regressor[:, 10*i:10*i+10] @ self.parameters[10*i:10*i+10]
                                for i in range(6)])

    def gravity(self, q):
        return pin.computeGeneralizedGravity(self.model, self.data, vector6(q, "q")).copy()

    def validate(self, q):
        q = vector6(q, "q")
        if np.any(q < self.lower - .03) or np.any(q > self.upper + .03):
            raise ValueError("Joint pose is outside model limits; check software zero and physical pose.")

    def export_urdf(self, scales, output):
        """Scale each reduced body's constituent links, retaining CAD centers of mass."""
        scales = validate_scales(scales)
        tree = ET.parse(self.path)
        parents = {joint.find("child").get("link"):
                   (joint.get("name"), joint.find("parent").get("link"))
                   for joint in tree.getroot().findall("joint")}
        for link in tree.getroot().findall("link"):
            name = link.get("name")
            while name in parents:
                joint, parent = parents[name]
                if joint in ARM_JOINTS:
                    scale = scales[int(joint[5:]) - 1]
                    inertial = link.find("inertial")
                    if inertial is not None:
                        mass = inertial.find("mass")
                        mass.set("value", str(float(mass.get("value")) * scale))
                        tensor = inertial.find("inertia")
                        for key, value in list(tensor.attrib.items()):
                            tensor.set(key, str(float(value) * scale))
                    break
                name = parent
        with Path(output).open("x") as stream:
            stream.write(ET.tostring(tree.getroot(), encoding="unicode"))


class GravitySupport:
    """Gravity feedforward torques from raw motor feedback, within the torque budget."""

    def __init__(self, model, reference, scale, limit):
        self.model = model
        self.reference = vector6(reference, "Software reference")
        self.scale = vector6(scale, "Gravity scale")
        self.limit = vector6(limit, "Gravity torque limits")

    def torque(self, q):
        """Scaled gravity at relative joint angles; refuse to undercompensate silently."""
        gravity = vector6(self.model.gravity(q) * self.scale, "Gravity feedforward")
        if np.any(np.abs(gravity) > self.limit):
            raise RuntimeError("Required gravity torque exceeds configured limits; support the arm and stop.")
        return gravity

    def __call__(self, raw):
        return self.torque(vector6(raw, "Motor feedback") - self.reference)

    def clipped(self, raw):
        """Best-effort support for the exit hold, which must not raise."""
        q = np.asarray(raw, dtype=float)[:6] - self.reference
        return np.clip(self.model.gravity(q) * self.scale, -self.limit, self.limit)
