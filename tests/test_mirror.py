"""Physical RS pose mapping, pair selection, and read-only mirroring on virtual CAN."""

from contextlib import redirect_stderr, redirect_stdout
import io
import math
from pathlib import Path
import unittest
from unittest.mock import MagicMock, patch

import mujoco
import numpy as np

from fakes import VirtualMotors, arm_bindings, patch_hardware
from rebot import bindings
from rebot.apps import mirror
from rebot.bindings import SIDES
from rebot.devices import robstride, socketcan
from rebot.devices.robstride import MODELS
from rebot.viewer.dual import JOINTS, load_dual_model, pose_from_motors


class PoseTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model, _ = load_dual_model("rebot")

    def setUp(self):
        self.data = mujoco.MjData(self.model)
        self.references = {side: [2.0, 1.0, 5.0, -0.5, 1.5, 3.0, 0.3]
                           for side in SIDES}

    def raw(self, left, right):
        return {side: [a + b for a, b in zip(self.references[side], angles)]
                for side, angles in (("left", left), ("right", right))}

    def test_both_arms_follow_rs_angles_without_leader_signs(self):
        poses = {"left": [.12, .4, .7, -.2, .15, .8, 0],
                 "right": [-.2, .3, .6, .25, -.1, -.9, 0]}
        pose_from_motors(self.model, self.data, self.raw(**poses), self.references)
        for side in SIDES:
            for index, name in enumerate(JOINTS):
                self.assertAlmostEqual(self.data.joint(f"{side}_{name}").qpos[0], poses[side][index])
        self.assertTrue(np.all(np.isfinite(self.data.xpos)))

    def test_continuous_angle_is_clamped_without_wrapping(self):
        raw = self.raw([2 * math.pi + .2, 0, 0, 0, 0, 0, 0], [0] * 7)
        clipped = pose_from_motors(self.model, self.data, raw, self.references)
        self.assertIn("left_joint1", clipped)
        self.assertAlmostEqual(self.data.joint("left_joint1").qpos[0], 2.8)

    def test_invalid_right_feedback_does_not_partially_update_left(self):
        before = self.data.qpos.copy()
        raw = self.raw([.5] * 7, [0] * 6 + [math.nan])
        with self.assertRaisesRegex(ValueError, "right"):
            pose_from_motors(self.model, self.data, raw, self.references)
        np.testing.assert_array_equal(self.data.qpos, before)


class BindingTests(unittest.TestCase):
    def arms(self):
        return {side: dict(reference_rad=[0.] * 7,
                           motor_uids={i: f"{i + offset:016x}" for i in range(1, 8)},
                           channel=channel, models=list(MODELS))
                for side, channel, offset in (("left", "can0", 0), ("right", "can1", 100))}

    def test_calibrated_pair_is_selected_with_references_and_channels(self):
        with patch.object(bindings, "load_arms", return_value=self.arms()), \
                patch.object(bindings, "resolve_channel", side_effect=lambda arm: arm["channel"]):
            selected = bindings.select_arms(Path("unused.yaml"), "both")
        self.assertEqual({side: binding.channel for side, binding in selected.items()},
                         dict(left="can0", right="can1"))
        for side, binding in selected.items():
            np.testing.assert_array_equal(binding.reference, np.zeros(7))
            self.assertEqual(binding.motor_uids, self.arms()[side]["motor_uids"])

    def test_incomplete_or_conflicting_pairs_are_rejected(self):
        def unbound(arms):
            del arms["right"]

        def uncalibrated(arms):
            del arms["right"]["reference_rad"]

        def shared_uid(arms):
            arms["right"]["motor_uids"][7] = arms["left"]["motor_uids"][7]

        own_channel = dict(side_effect=lambda arm: arm["channel"])
        for change, resolve, message in ((unbound, own_channel, "bind right <channel>"),
                                         (uncalibrated, own_channel, "calibrate right"),
                                         (shared_uid, own_channel, "distinct motor identities"),
                                         (lambda arms: None, dict(return_value="can0"), "distinct CAN")):
            arms = self.arms()
            change(arms)
            with self.subTest(message=message), \
                    patch.object(bindings, "load_arms", return_value=arms), \
                    patch.object(bindings, "resolve_channel", **resolve), \
                    self.assertRaisesRegex(ValueError, message):
                bindings.select_arms(Path("unused.yaml"), "both")


class ReadOnlyFrameTests(unittest.TestCase):
    """Mirroring on the real transport: device-ID and parameter-read frames only."""

    def setUp(self):
        self.virtual = {"can0": VirtualMotors(), "can1": VirtualMotors(100)}
        selected = arm_bindings({"left": ("can0", self.virtual["can0"]),
                                 "right": ("can1", self.virtual["can1"])})
        patch_hardware(self, self.virtual, selected, modules=(mirror,))
        self.viewer = MagicMock()
        self.viewer.__enter__.return_value = self.viewer
        self.viewer.__exit__.return_value = False
        self.viewer.is_running.side_effect = [True, True, False]
        self.model = load_dual_model("rebot")
        for target, name, value in ((socketcan, "check_channel", MagicMock()),
                                    (mirror, "load_dual_model", lambda target: self.model),
                                    (mirror, "SceneViewer", MagicMock(return_value=self.viewer)),
                                    (mirror.time, "sleep", lambda duration: None)):
            patcher = patch.object(target, name, value)
            patcher.start()
            self.addCleanup(patcher.stop)
        for context in (redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO())):
            context.__enter__()
            self.addCleanup(context.__exit__, None, None, None)

    def assert_read_only_and_closed(self):
        for motors in self.virtual.values():
            self.assertTrue(motors.closed)
            self.assertTrue(motors.sent)
            self.assertLessEqual({(m.arbitration_id >> 24) & 31 for m in motors.sent},
                                 {robstride.GET_DEVICE_ID, robstride.READ_PARAMETER})
            self.assertEqual(set(motors.modes.values()), {0})
            self.assertTrue(all(value == 4000 for (_, parameter), value in motors.params.items()
                                if parameter == robstride.WATCHDOG))

    def test_mirroring_sends_only_queries_and_displays_the_measured_pose(self):
        mirror.mirror()
        self.assert_read_only_and_closed()
        _, data = self.model
        for side in SIDES:
            for name in JOINTS[:3]:
                self.assertAlmostEqual(data.joint(f"{side}_{name}").qpos[0], .2, places=6)
        # Startup pose plus two displayed frames, seven position reads each, after the scan.
        self.assertEqual(len(self.virtual["can0"].sent), 7 + 3 * 7)

    def test_wrong_arm_exit_sends_only_queries_and_closes_both_buses(self):
        self.virtual["can1"].uids[7] = "ff" * 8
        with self.assertRaisesRegex(ValueError, "right: motor UIDs"):
            mirror.mirror()
        self.viewer.is_running.assert_not_called()
        self.assert_read_only_and_closed()


if __name__ == "__main__":
    unittest.main()
