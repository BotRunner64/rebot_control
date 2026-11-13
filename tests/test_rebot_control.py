"""Dual-arm target validation and virtual CAN regressions."""

from contextlib import redirect_stderr, redirect_stdout
import io
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

import rebot_control as control
import rebot_mit as mit
from teleop_process_fakes import MeasuredMotors
from test_rebot_ee import EE_AVAILABLE


def options(hardware=False):
    return SimpleNamespace(hardware=hardware, config=control.CONFIG, settings=control.SETTINGS_PATH,
                           gripper_settings=control.GRIPPER_SETTINGS, gripper_open_deg=270.)


@unittest.skipUnless(EE_AVAILABLE, "Install the ee dependency extra")
class RobotTests(unittest.TestCase):
    def test_invalid_right_target_and_collision_never_publish_either_arm(self):
        with control.Robot(options(), threading.Event()) as robot:
            previous = {side: q.copy() for side, q in robot.sim_q.items()}
            state = robot.state()["state"]
            state[12] = .1
            invalid = state.copy()
            invalid[16:25] = [0.] * 9
            with self.assertRaises(ValueError):
                robot.command(invalid)
            with patch.object(robot.iks["right"], "has_self_collision", return_value=True):
                self.assertFalse(robot.command(state))
            for side in control.SIDES:
                np.testing.assert_array_equal(robot.sim_q[side], previous[side])
            self.assertTrue(robot.command(state))
            self.assertFalse(robot.stop.is_set())

    def hardware(self):
        virtual = {side: MeasuredMotors(index * 100) for index, side in enumerate(control.SIDES)}
        start = np.array([0., .5, 1., 0., .1, 0., 0.])
        arms = {side: {"reference_rad": list(.2 - start), "channel": side,
                       "motor_uids": dict(motor.uids), "models": ["rs-06"] * 3 + ["rs-00"] * 4}
                for side, motor in virtual.items()}

        def factory(channel, models, **kwargs):
            with patch.object(mit, "check_channel"), patch.object(mit.can, "Bus", return_value=virtual[channel]):
                return mit.MotorBus(channel, models, **kwargs)

        self.addCleanup(patch.stopall)
        patch.object(control, "load_config", return_value=arms).start()
        patch.object(control, "resolve_channel", side_effect=lambda arm: arm["channel"]).start()
        return control.Robot(options(True), threading.Event(), bus_factory=factory), virtual, arms

    def test_both_arms_are_verified_before_either_is_enabled(self):
        robot, virtual, arms = self.hardware()
        arms["right"]["motor_uids"][7] = "ff" * 8
        with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()), self.assertRaises(ValueError):
            with robot:
                self.fail("Mismatched hardware was enabled")
        for motor in virtual.values():
            self.assertTrue(motor.closed)
            self.assertNotIn(3, {(frame.arbitration_id >> 24) & 31 for frame in motor.sent})

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
        for motor in virtual.values():
            self.assertTrue(motor.closed)
            self.assertEqual(set(motor.modes.values()), {2})
            self.assertNotIn(4, {(frame.arbitration_id >> 24) & 31 for frame in motor.sent})
            frames = [frame for frame in motor.sent if (frame.arbitration_id >> 24) & 31 == 1
                      and frame.arbitration_id & 255 == 7 and bytes(frame.data[4:]) == bytes(4)]
            self.assertTrue(frames)
            self.assertTrue(all(abs(mit.decode_mit(frame, "rs-00")["gravity"]) <= 1. for frame in frames))


if __name__ == "__main__":
    unittest.main()
