"""Offline regressions for excessive native joint-position errors."""

from contextlib import redirect_stdout
from dataclasses import replace
import io
import math
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest.mock import MagicMock

import numpy as np
import yaml

from rebot.bindings import ArmBinding
from rebot.devices.robstride import MODELS
from rebot.model.limits import DEFAULT_MAX_TRACKING_ERROR_DEG, validate_tracking_error
from rebot.model.trajectory import TrajectoryTracker
from rebot.paths import CONTROL_SETTINGS
from rebot.runtime.arm import Arm
from rebot.runtime.worker import ArmWorker
from rebot.settings import load_settings


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

    def test_settings_require_the_guard_and_invalid_settings_fail(self):
        # The guard used to be inherited by old settings files; it is now a required key.
        self.assertEqual(DEFAULT_MAX_TRACKING_ERROR_DEG, 45.)
        self.assertEqual(load_settings().max_tracking_error_deg, 45.)
        settings = yaml.safe_load(CONTROL_SETTINGS.read_text())
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "settings.yaml"
            settings.pop("max_tracking_error_deg")
            path.write_text(yaml.safe_dump(settings))
            with self.assertRaisesRegex(ValueError, "max_tracking_error_deg"):
                load_settings(path)
            for limit in (0, -1, math.nan, math.inf, True):
                settings["max_tracking_error_deg"] = limit
                path.write_text(yaml.safe_dump(settings))
                with self.subTest(limit=limit), self.assertRaises(ValueError):
                    load_settings(path)
            settings["max_tracking_error_deg"] = 60.
            path.write_text(yaml.safe_dump(settings))
            self.assertEqual(load_settings(path).max_tracking_error_deg, 60.)

    def test_homing_and_identification_reject_large_error_before_command(self):
        controller = TrajectoryTracker(MagicMock(), load_settings())
        with self.assertRaisesRegex(RuntimeError, "Tracking error"):
            controller.update(np.zeros(6), np.full(6, math.radians(46)), 1.)


def feedback(positions):
    return [{"position": float(q), "velocity": 0., "torque": 0., "temperature": 25.,
             "received_at": time.monotonic()} for q in positions]


class ArmWorkerTrackingTests(unittest.TestCase):
    def worker(self, limit=None):
        bus, stop = MagicMock(), threading.Event()
        bus.__enter__.return_value = bus
        uids = {i: i.to_bytes(8, "little").hex() for i in range(1, 8)}
        bus.scan.return_value = uids
        bus.positions.return_value = [0.] * 6
        bus.feedback_states.return_value = feedback(np.zeros(6))
        settings = load_settings()
        if limit is not None:
            settings = replace(settings, max_tracking_error_deg=limit)
        arm = Arm(ArmBinding("right", "can0", MODELS, uids, np.zeros(7)), settings,
                  stop=stop, bus_factory=lambda *args, **kwargs: bus)
        arm.__enter__()
        bus.command_position.assert_not_called()
        return ArmWorker(arm, stop), bus, stop

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
            worker, bus, stop = self.worker(limit)
            worker.command(np.full(6, math.radians(angle)))
            bus.command_position.side_effect = lambda *args, **kwargs: stop.set()
            with redirect_stdout(io.StringIO()):
                worker._run()
            worker.check()
            bus.command_position.assert_called_once()
            np.testing.assert_allclose(bus.command_position.call_args.args[0],
                                       np.full(6, math.radians(angle)))

    def test_ee_new_feedback_error_stops_after_updating_fallback_hold(self):
        worker, bus, stop = self.worker()
        measured = np.full(6, math.radians(46))
        bus.feedback_states.return_value = feedback(measured)
        with redirect_stdout(io.StringIO()):
            worker._run()
        self.assertTrue(stop.is_set())
        self.assertIsInstance(worker.error, RuntimeError)
        self.assertRegex(str(worker.error), "Tracking error")
        bus.command_position.assert_called_once()
        np.testing.assert_array_equal(bus.set_hold.call_args.args[0], measured)


if __name__ == "__main__":
    unittest.main()
