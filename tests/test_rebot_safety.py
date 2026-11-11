"""Offline regressions for excessive native joint-position errors."""

from contextlib import redirect_stdout
import io
import math
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import MagicMock

import numpy as np
import yaml

from rebot_hand_guiding import SETTINGS_PATH, load_settings
from rebot_ee_runtime import MotorWorker
from rebot_motion import MotionController
from rebot_safety import validate_tracking_error


class TrackingErrorTests(unittest.TestCase):
    def test_each_joint_and_both_directions_are_checked(self):
        for joint in range(6):
            for sign in (-1, 1):
                target = np.zeros(6)
                target[joint] = sign * math.radians(46)
                with self.subTest(joint=joint, sign=sign):
                    with self.assertRaisesRegex(RuntimeError, rf"45 deg.*J{joint + 1}: error=46.00"):
                        validate_tracking_error(target, np.zeros(6))

    def test_boundary_offsets_gripper_and_custom_threshold(self):
        measured = np.zeros(7)
        target = np.full(7, math.radians(45))
        target[6] = math.radians(270)
        validate_tracking_error(target, measured)
        target[0] = math.radians(46)
        validate_tracking_error(target, measured, 60.)
        with self.assertRaises(RuntimeError):
            validate_tracking_error(target, measured)
        reference = np.full(7, .2)
        validate_tracking_error(reference + np.radians([44] * 6 + [270]), reference)

    def test_multi_turn_error_is_not_wrapped(self):
        with self.assertRaisesRegex(RuntimeError, "error=360.00"):
            validate_tracking_error(np.full(6, 2 * math.pi), np.zeros(6))

    def test_invalid_limits_and_incomplete_or_nonfinite_arm_data_fail(self):
        for limit in (0., -1., math.nan, math.inf, True):
            with self.subTest(limit=limit), self.assertRaises(ValueError):
                validate_tracking_error(np.zeros(6), np.zeros(6), limit)
        for target, measured in (([0.] * 5, [0.] * 5), ([0.] * 7, [0.] * 6),
                                 ([math.nan] * 6, [0.] * 6), ([0.] * 6, [math.inf] * 6)):
            with self.subTest(target=target), self.assertRaises(ValueError):
                validate_tracking_error(target, measured)

    def test_old_settings_inherit_guard_and_invalid_settings_fail(self):
        settings = yaml.safe_load(SETTINGS_PATH.read_text())
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "settings.yaml"
            settings.pop("max_tracking_error_deg")
            path.write_text(yaml.safe_dump(settings))
            self.assertEqual(load_settings(path)["max_tracking_error_deg"], 45.)
            for limit in (0, -1, math.nan, math.inf, True):
                settings["max_tracking_error_deg"] = limit
                path.write_text(yaml.safe_dump(settings))
                with self.subTest(limit=limit), self.assertRaises(ValueError):
                    load_settings(path)

    def test_homing_and_identification_reject_large_error_before_command(self):
        controller = MotionController(MagicMock(), load_settings())
        with self.assertRaisesRegex(RuntimeError, "Tracking error"):
            controller.update(np.zeros(6), np.full(6, math.radians(46)), 1.)


class MotorWorkerTrackingTests(unittest.TestCase):
    def worker(self):
        bus, stop = MagicMock(), threading.Event()
        bus.feedback_positions.return_value = np.zeros(6)
        worker = MotorWorker(bus, np.zeros(6), np.full(6, -3.), np.full(6, 3.),
                             load_settings(), stop, lambda raw: np.zeros(6))
        return worker, bus, stop

    def test_ee_jump_is_rejected_before_sending_any_motor_target(self):
        worker, bus, stop = self.worker()
        worker.command(np.full(6, math.radians(46)))
        with redirect_stdout(io.StringIO()):
            worker._run()
        self.assertTrue(stop.is_set())
        with self.assertRaisesRegex(RuntimeError, "Tracking error"):
            worker.check()
        bus.command_position.assert_not_called()

    def test_ee_within_limit_sends_and_custom_limit_is_used(self):
        for angle, limit in ((44., 45.), (46., 60.)):
            worker, bus, stop = self.worker()
            worker.cfg["max_tracking_error_deg"] = limit
            worker.command(np.full(6, math.radians(angle)))
            bus.command_position.side_effect = lambda *args: stop.set()
            with redirect_stdout(io.StringIO()):
                worker._run()
            worker.check()
            bus.command_position.assert_called_once()

    def test_ee_new_feedback_error_stops_after_updating_fallback_hold(self):
        worker, bus, stop = self.worker()
        measured = np.full(6, math.radians(46))
        bus.feedback_positions.return_value = measured
        with redirect_stdout(io.StringIO()):
            worker._run()
        self.assertTrue(stop.is_set())
        self.assertIsInstance(worker.error, RuntimeError)
        bus.command_position.assert_called_once()
        np.testing.assert_array_equal(bus.set_hold.call_args.args[0], measured)


if __name__ == "__main__":
    unittest.main()
