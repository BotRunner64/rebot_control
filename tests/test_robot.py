"""Dual-arm target validation and virtual CAN regressions."""

from contextlib import redirect_stderr, redirect_stdout
import io
import math
import threading
import time
import unittest
from unittest.mock import patch

import numpy as np

from fakes import arm_bindings
from rebot.bindings import SIDES, select_sides
from rebot.devices import robstride
from rebot.runtime import robot as control
from teleop_fakes import MeasuredMotors
from test_ee import EE_AVAILABLE


def frame_kinds(motor):
    return {(frame.arbitration_id >> 24) & 31 for frame in motor.sent}


class TargetTests(unittest.TestCase):
    def state(self):
        return [.3, 0., .4, 1., 0., 0., 0., 1., 0., 0., 0., 1., .25,
                .3, .1, .5, 0., 1., 0., -1., 0., 0., 0., 0., 1., 2.]

    def test_targets_are_absolute_column_major_poses_with_clipped_openness(self):
        poses, openness = control.decode_targets(self.state())
        np.testing.assert_allclose(poses[0], [[1, 0, 0, .3], [0, 1, 0, 0.], [0, 0, 1, .4], [0, 0, 0, 1]])
        np.testing.assert_allclose(poses[1], [[0, -1, 0, .3], [1, 0, 0, .1], [0, 0, 1, .5], [0, 0, 0, 1]])
        self.assertEqual(openness, [.25, 1.])

    def test_malformed_targets_are_rejected(self):
        for index, value in ((0, math.nan), (17, 0.), (3, -1.), (25, math.inf)):
            state = self.state()
            state[index] = value
            with self.subTest(index=index), self.assertRaises(ValueError):
                control.decode_targets(state)
        with self.assertRaises(ValueError):
            control.decode_targets(self.state()[:13])


@unittest.skipUnless(EE_AVAILABLE, "Install the ee dependency extra")
class RobotTests(unittest.TestCase):
    def test_invalid_right_target_and_collision_never_publish_either_arm(self):
        with control.Robot(hardware=False, stop=threading.Event()) as robot:
            previous = {side: q.copy() for side, q in robot.sim_q.items()}
            state = robot.state()["state"]
            state[12] = .1
            invalid = state.copy()
            invalid[16:25] = [0.] * 9
            with self.assertRaises(ValueError):
                robot.command(invalid)
            with patch.object(robot.iks["right"], "has_self_collision", return_value=True):
                self.assertFalse(robot.command(state))
            for side in SIDES:
                np.testing.assert_array_equal(robot.sim_q[side], previous[side])
            self.assertTrue(robot.command(state))
            self.assertFalse(robot.stop.is_set())
            self.assertEqual(robot.sim_q["left"][6], .1)

    # Relative start pose of both arms; OPEN_START keeps the gripper off its closed limit.
    START = (0., .5, 1., 0., .1, 0., 0.)
    OPEN_START = (0., .5, 1., 0., .1, 0., .1)

    def hardware(self, start=START):
        virtual = {side: MeasuredMotors(index * 100) for index, side in enumerate(SIDES)}
        start = np.array(start)
        selected = arm_bindings({side: (side, motor) for side, motor in virtual.items()},
                                reference=list(.2 - start))

        def factory(channel, models, **kwargs):
            self.factory_calls.append((channel, kwargs))
            with patch.object(robstride, "check_channel"), \
                    patch.object(robstride.can, "Bus", return_value=virtual[channel]):
                return robstride.MotorBus(channel, models, **kwargs)

        def select(path=None, side="both", *, calibrated=True):
            return {name: selected[name] for name in select_sides(side)}

        self.factory_calls = []
        self.addCleanup(patch.stopall)
        patch.object(control, "select_arms", side_effect=select).start()
        robot = control.Robot(hardware=True, stop=threading.Event(), bus_factory=factory)
        return robot, virtual, selected

    def test_both_arms_are_verified_before_either_is_enabled(self):
        for fault in ("identity", "mode"):
            with self.subTest(fault=fault):
                robot, virtual, selected = self.hardware()
                if fault == "identity":
                    selected["right"].motor_uids[7] = "ff" * 8
                else:
                    virtual["right"].params[(7, robstride.MODE)] = 1
                with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()), \
                        self.assertRaises(ValueError):
                    with robot:
                        self.fail("Mismatched hardware was enabled")
                for motor in virtual.values():
                    self.assertTrue(motor.closed)
                    self.assertFalse(frame_kinds(motor) & {1, 3, 4})
                if fault == "identity":
                    # Nothing was written either: identity and pose queries only.
                    self.assertTrue(all(frame_kinds(motor) <= {0, 17} for motor in virtual.values()))
                    self.assertTrue(robot.stop.is_set())

    def test_idle_input_keeps_can_running_and_feedback_failure_holds_both_arms(self):
        robot, virtual, _ = self.hardware()
        with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
            with self.assertRaises(TimeoutError):
                with robot:
                    state = robot.state()["state"]
                    state[12], state[25] = 1., 1.
                    self.assertTrue(robot.command(state))
                    counts = {side: len(motor.sent) for side, motor in virtual.items()}
                    time.sleep(.05)
                    for side, motor in virtual.items():
                        self.assertGreater(len(motor.sent), counts[side] + 30)
                    virtual["right"].drop_holds.add(4)
                    try:
                        self.assertTrue(robot.stop.wait(1.))
                        robot.check()
                    finally:
                        virtual["right"].drop_holds.clear()
        self.assertEqual(self.factory_calls, [(side, {"motor_count": 7}) for side in SIDES])
        for motor in virtual.values():
            self.assertTrue(motor.closed)
            self.assertEqual(set(motor.modes.values()), {2})
            self.assertNotIn(4, frame_kinds(motor))
            frames = [frame for frame in motor.sent if (frame.arbitration_id >> 24) & 31 == 1
                      and frame.arbitration_id & 255 == 7 and bytes(frame.data[4:]) == bytes(4)]
            self.assertTrue(frames)
            self.assertTrue(all(abs(robstride.decode_mit(frame, "rs-00")["gravity"]) <= 1. for frame in frames))
            # The exit hold is the last thing sent: native gains on all seven motors.
            hold = [frame for frame in motor.sent if (frame.arbitration_id >> 24) & 31 == 1][-7:]
            self.assertEqual([frame.arbitration_id & 255 for frame in hold], list(range(1, 8)))
            for frame in hold:
                model = robstride.MODELS[(frame.arbitration_id & 255) - 1]
                decoded = robstride.decode_mit(frame, model)
                self.assertGreater(decoded["kp"], 0.)
                self.assertGreater(decoded["kd"], 0.)
                self.assertAlmostEqual(decoded["position"], .2, delta=.0005)

    def test_hardware_state_reports_measured_pose_and_normalized_gripper(self):
        robot, virtual, _ = self.hardware(self.OPEN_START)
        with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
            with robot:
                state = robot.state()
                expected = robot.iks["left"].pose(np.array([0., .5, 1., 0., .1, 0.]))
        values = np.array(state["state"]).reshape(2, 13)
        for arm in values:
            np.testing.assert_allclose(arm[:3], expected[:3, 3], atol=2e-3)
            np.testing.assert_allclose(arm[3:12].reshape(3, 3, order="F"), expected[:3, :3], atol=2e-3)
            self.assertAlmostEqual(arm[12], .1 / math.radians(270.), delta=1e-3)
        self.assertEqual(set(state["acquired_at"]), set(SIDES))
        self.assertEqual(state["observed_at"], min(state["acquired_at"].values()))

    def test_hardware_rejection_of_the_right_target_never_moves_the_left_arm(self):
        robot, virtual, _ = self.hardware(self.OPEN_START)
        with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
            with robot:
                previous = {side: worker.target.copy() for side, worker in robot.workers.items()}
                state = robot.state()["state"]
                moved = list(state)
                moved[0] += .03            # a valid, reachable left target
                moved[12] = moved[25] = 1.
                invalid = list(moved)
                invalid[16:25] = [0.] * 9
                with self.assertRaises(ValueError):
                    robot.command(invalid)
                with patch.object(robot.iks["right"], "has_self_collision", return_value=True):
                    self.assertFalse(robot.command(moved))
                with patch.object(robot.iks["right"], "solve",
                                  return_value=robot.iks["right"].q + math.radians(60.)):
                    with self.assertRaisesRegex(RuntimeError, "Tracking error limit"):
                        robot.command(moved)
                for side in SIDES:
                    np.testing.assert_array_equal(robot.workers[side].target, previous[side])
                self.assertFalse(robot.stop.is_set())
                robot.check()
                self.assertTrue(robot.command(moved))
                for side in SIDES:
                    self.assertAlmostEqual(robot.workers[side].target[6], math.radians(270.))
                self.assertGreater(float(np.max(np.abs(robot.workers["left"].target[:6]
                                                       - previous["left"][:6]))), .01)
        for motor in virtual.values():
            self.assertTrue(motor.closed)
            self.assertEqual(set(motor.modes.values()), {2})
            self.assertNotIn(4, frame_kinds(motor))


if __name__ == "__main__":
    unittest.main()
