"""Verify physical RS pose mapping and paired read-only viewer lifecycle."""

from contextlib import contextmanager, redirect_stderr, redirect_stdout
import io
import math
from pathlib import Path
import unittest
from unittest.mock import MagicMock, patch

import mujoco
import numpy as np

import rebot_mirror as mirror


class PoseTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model, _ = mirror.load_model("rebot")

    def setUp(self):
        self.data = mujoco.MjData(self.model)
        self.references = {side: [2.0, 1.0, 5.0, -0.5, 1.5, 3.0, 0.3]
                           for side in mirror.SIDES}

    def raw(self, left, right):
        return {side: [a + b for a, b in zip(self.references[side], angles)]
                for side, angles in (("left", left), ("right", right))}

    def test_reference_pose_maps_to_zero(self):
        mirror.set_model_pose(self.model, self.data, self.references, self.references)
        np.testing.assert_allclose(self.data.qpos, 0)

    def test_both_arms_follow_rs_angles_without_leader_signs(self):
        poses = {"left": [.12, .4, .7, -.2, .15, .8, 0],
                 "right": [-.2, .3, .6, .25, -.1, -.9, 0]}
        mirror.set_model_pose(self.model, self.data, self.raw(**poses), self.references)
        for side in mirror.SIDES:
            for index, name in enumerate(mirror.JOINTS):
                self.assertAlmostEqual(self.data.joint(f"{side}_{name}").qpos[0], poses[side][index])
        self.assertTrue(np.all(np.isfinite(self.data.xpos)))

    def test_continuous_angle_is_clamped_without_wrapping(self):
        raw = self.raw([2 * math.pi + .2, 0, 0, 0, 0, 0, 0], [0] * 7)
        clipped = mirror.set_model_pose(self.model, self.data, raw, self.references)
        self.assertIn("left_joint1", clipped)
        self.assertAlmostEqual(self.data.joint("left_joint1").qpos[0], 2.8)

    def test_gripper_uses_motor_angle_without_handle_multiplier(self):
        raw = self.raw([0] * 6 + [math.radians(135)], [0] * 6 + [math.radians(270)])
        mirror.set_model_pose(self.model, self.data, raw, self.references)
        for name in ("joint7", "joint_left", "joint_right"):
            self.assertAlmostEqual(self.data.joint(f"left_{name}").qpos[0], .025)
            self.assertAlmostEqual(self.data.joint(f"right_{name}").qpos[0], .05)

    def test_invalid_right_feedback_does_not_partially_update_left(self):
        before = self.data.qpos.copy()
        raw = self.raw([.5] * 7, [0] * 6 + [math.nan])
        with self.assertRaisesRegex(ValueError, "right"):
            mirror.set_model_pose(self.model, self.data, raw, self.references)
        np.testing.assert_array_equal(self.data.qpos, before)


class BindingTests(unittest.TestCase):
    def arms(self):
        return {side: dict(reference_rad=[0.] * 7,
                           motor_uids={i: f"{i + offset:016x}" for i in range(1, 8)},
                           channel=channel)
                for side, channel, offset in (("left", "can0", 0), ("right", "can1", 100))}

    def test_both_calibrations_required(self):
        arms = self.arms()
        del arms["right"]["reference_rad"]
        with patch.object(mirror, "load_config", return_value=arms), \
                patch.object(mirror, "resolve_channel", side_effect=lambda arm: arm["channel"]):
            with self.assertRaisesRegex(ValueError, "calibrate right"):
                mirror.load_bindings(Path("unused.yaml"))

    def test_same_interface_is_rejected(self):
        with patch.object(mirror, "load_config", return_value=self.arms()), \
                patch.object(mirror, "resolve_channel", return_value="can0"):
            with self.assertRaisesRegex(ValueError, "different CAN"):
                mirror.load_bindings(Path("unused.yaml"))


class LifecycleTests(unittest.TestCase):
    def setUp(self):
        self.arms = {side: {} for side in mirror.SIDES}
        self.references = {side: [0.] * 7 for side in mirror.SIDES}
        self.channels = dict(left="can0", right="can1")
        self.readers = {side: MagicMock() for side in mirror.SIDES}
        for reader in self.readers.values():
            reader.positions.return_value = [0.] * 7
        self.closed = []

        @contextmanager
        def open_reader(channel):
            side = "left" if channel == "can0" else "right"
            try:
                yield self.readers[side]
            finally:
                self.closed.append(side)

        @contextmanager
        def open_viewer(*args, **kwargs):
            try:
                yield self.viewer
            finally:
                self.viewer_closed = True

        self.viewer = MagicMock()
        self.viewer.is_running.side_effect = [True, True, False]
        self.viewer_closed = False
        self.pose_calls = []

        def pose(model, data, raw, references, gripper):
            self.pose_calls.append({side: list(angles) for side, angles in raw.items()})
            return set()

        for name, value in (
                ("load_bindings", lambda config: (self.arms, self.references, self.channels)),
                ("load_model", lambda target: (MagicMock(), MagicMock())),
                ("open_reader", open_reader), ("verify_identity", lambda *args: None),
                ("open_viewer", open_viewer), ("set_model_pose", pose)):
            patcher = patch.object(mirror, name, value)
            patcher.start()
            self.addCleanup(patcher.stop)
        for context in (redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO())):
            context.__enter__()
            self.addCleanup(context.__exit__, None, None, None)
        patcher = patch.object(mirror.time, "sleep", lambda duration: None)
        patcher.start()
        self.addCleanup(patcher.stop)

    def assert_read_only(self):
        for reader in self.readers.values():
            self.assertEqual({call[0] for call in reader.method_calls}, {"positions"})

    def test_missing_feedback_freezes_both_models_and_recovers(self):
        self.readers["left"].positions.side_effect = [[0.] * 7, [1.] * 7, [2.] * 7]
        self.readers["right"].positions.side_effect = [[0.] * 7, TimeoutError("Lost frame"), [3.] * 7]
        mirror.mirror()
        self.assertEqual(len(self.pose_calls), 2)
        self.assertEqual(self.pose_calls[-1], dict(left=[2.] * 7, right=[3.] * 7))
        self.assertTrue(self.viewer_closed)
        self.assertEqual(self.closed, ["right", "left"])
        self.assert_read_only()

    def test_fault_aborts_immediately_and_closes_both_connections(self):
        self.readers["right"].positions.side_effect = [[0.] * 7, RuntimeError("Motor fault")]
        with self.assertRaisesRegex(RuntimeError, "Motor fault"):
            mirror.mirror()
        self.assertTrue(self.viewer_closed)
        self.assertEqual(self.closed, ["right", "left"])
        self.assertEqual(len(self.pose_calls), 1)
        self.assert_read_only()

    def test_identity_failure_closes_connections_before_opening_viewer(self):
        with patch.object(mirror, "verify_identity", side_effect=[None, ValueError("Wrong arm")]):
            with self.assertRaisesRegex(ValueError, "Wrong arm"):
                mirror.mirror()
        self.assertEqual(self.closed, ["right", "left"])
        self.viewer.is_running.assert_not_called()

    def test_sustained_feedback_loss_exits(self):
        self.readers["right"].positions.side_effect = [[0.] * 7] + [TimeoutError("Lost frame")] * 3
        self.viewer.is_running.side_effect = [True] * 4
        with patch.object(mirror.time, "monotonic", side_effect=range(20)):
            with self.assertRaisesRegex(RuntimeError, "did not recover"):
                mirror.mirror()
        self.assertEqual(len(self.pose_calls), 1)
        self.assertEqual(self.closed, ["right", "left"])


if __name__ == "__main__":
    unittest.main()
