"""Verify isolated handle calibration without connecting to hardware."""

from contextlib import redirect_stdout
import copy
import io
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, call, patch

import serial

import stararm


def monitor_bus(samples):
    bus = SimpleNamespace(uart=Mock(), pkt_buffer=Mock(),
                          servos={i: SimpleNamespace(angle_monitor=999.) for i in range(7)})
    samples = iter(samples)

    def monitor(ids, realtime):
        assert realtime
        assert all(bus.servos[i].angle_monitor is None for i in ids)
        return {i: SimpleNamespace(angle_monitor=value)
                for i, value in zip(ids, next(samples))}

    bus.send_sync_servo_monitor = Mock(side_effect=monitor)
    return bus


class GripperCalibrationTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.config = Path(temporary.name) / "stararm.local.yaml"
        self.arms = {
            "left": {"port": "/fake/left", "baudrate": 1000000,
                     "reference_deg": [3.2, -.3, .4, -4., -2.1, -2.4, 0.],
                     "directions": [1] * 7},
            "right": {"port": "/fake/right", "baudrate": 1000000,
                      "reference_deg": [2.3, -.4, .4, -3.5, -2.7, -2.4, -.2]}}
        stararm.save_config(self.arms, self.config)

    def run_capture(self, samples, side="left", attempts=20):
        bus = monitor_bus([[value] for value in samples])
        with patch.object(stararm.serial, "Serial") as connection, \
                patch.object(stararm, "UartServoManager", return_value=bus), \
                patch.object(stararm.time, "sleep"), redirect_stdout(io.StringIO()):
            stararm.calibrate(side, replace=True, config=self.config, gripper_only=True)
        connection.assert_called_once_with(self.arms[side]["port"], baudrate=1000000,
                                           timeout=0, exclusive=True)
        self.assertEqual(bus.send_sync_servo_monitor.call_args_list,
                         [call([6], realtime=True)] * attempts)
        return stararm.load_config(self.config)

    def test_updates_only_selected_handle_and_preserves_binding_and_six_axes(self):
        for side, closed in (("left", 1.1), ("right", .4)):
            with self.subTest(side=side):
                stararm.save_config(self.arms, self.config)
                saved = self.run_capture([closed - .1, closed + .1] * 10, side)
                expected = copy.deepcopy(self.arms)
                expected[side]["reference_deg"][6] = closed
                self.assertEqual(saved, expected)

    def test_unstable_or_invalid_samples_leave_file_unchanged(self):
        original = self.config.read_bytes()
        for samples in ([1.1, 1.4] * 10, [float("nan")] * 30, [None] * 30):
            with self.subTest(samples=samples[:2]):
                with self.assertRaises(ValueError):
                    self.run_capture(samples)
                self.assertEqual(self.config.read_bytes(), original)

    def test_transient_missing_feedback_is_retried_with_fresh_samples(self):
        samples = [None, .4, .5, None] + [.4] * 18
        saved = self.run_capture(samples, "right", attempts=22)
        expected = copy.deepcopy(self.arms)
        expected["right"]["reference_deg"][6] = .4
        self.assertEqual(saved, expected)

    def test_partial_feedback_is_not_enough_to_save(self):
        original = self.config.read_bytes()
        with self.assertRaisesRegex(ValueError, "19/20.*30 attempts"):
            self.run_capture([.4] * 19 + [None] * 11, "right")
        self.assertEqual(self.config.read_bytes(), original)

    def test_motion_after_retry_still_rejects_calibration(self):
        original = self.config.read_bytes()
        with self.assertRaisesRegex(ValueError, "gripper moved"):
            self.run_capture([.4] * 10 + [None] + [.8] * 10, "right")
        self.assertEqual(self.config.read_bytes(), original)

    def test_requires_valid_existing_reference_and_explicit_replace_before_opening_port(self):
        with patch.object(stararm.serial, "Serial") as connection:
            with self.assertRaisesRegex(ValueError, "--replace"):
                stararm.calibrate("left", config=self.config, gripper_only=True)
            for reference in (None, [0.] * 6, [0.] * 6 + [float("nan")]):
                arms = copy.deepcopy(self.arms)
                arms["left"]["reference_deg"] = reference
                stararm.save_config(arms, self.config)
                with self.assertRaisesRegex(ValueError, "seven existing"):
                    stararm.calibrate("left", replace=True, config=self.config, gripper_only=True)
            connection.assert_not_called()

    def test_busy_serial_port_leaves_file_unchanged(self):
        original = self.config.read_bytes()
        with patch.object(stararm.serial, "Serial", side_effect=serial.SerialException("busy")):
            with self.assertRaises(serial.SerialException):
                stararm.calibrate("left", replace=True, config=self.config, gripper_only=True)
        self.assertEqual(self.config.read_bytes(), original)

    def test_binding_change_during_capture_is_not_overwritten(self):
        changed = copy.deepcopy(self.arms)
        changed["left"]["baudrate"] = 115200

        def capture(bus):
            stararm.save_config(changed, self.config)
            return 1.1

        with patch.object(stararm.serial, "Serial"), patch.object(stararm, "UartServoManager"), \
                patch.object(stararm, "capture_gripper", side_effect=capture):
            with self.assertRaisesRegex(ValueError, "binding changed"):
                stararm.calibrate("left", replace=True, config=self.config, gripper_only=True)
        self.assertEqual(stararm.load_config(self.config), changed)

    def test_default_reader_still_requires_all_seven_fresh_servo_angles(self):
        values = [1., 2., 3., 4., 5., 6., 7.]
        bus = monitor_bus([values])
        self.assertEqual(stararm.read_angles(bus), values)
        bus.send_sync_servo_monitor.assert_called_once_with(list(range(7)), realtime=True)
        bus = monitor_bus([values[:6]])
        with self.assertRaisesRegex(ValueError, "IDs \\[6\\]"):
            stararm.read_angles(bus)

    def test_full_calibration_retains_existing_behavior(self):
        reference = [1.] * 7
        with patch.object(stararm.serial, "Serial"), patch.object(stararm, "UartServoManager"), \
                patch.object(stararm, "capture_pose", return_value=reference) as capture, \
                patch.object(stararm, "capture_gripper") as gripper, redirect_stdout(io.StringIO()):
            stararm.calibrate("left", replace=True, config=self.config)
        capture.assert_called_once()
        gripper.assert_not_called()
        self.assertEqual(stararm.load_config(self.config)["left"]["reference_deg"], reference)

    def test_cli_passes_gripper_only_option(self):
        with patch.object(stararm.sys, "argv", ["stararm.py", "calibrate", "right",
                                               "--gripper-only", "--replace"]), \
                patch.object(stararm, "calibrate") as calibrate:
            self.assertEqual(stararm.main(), 0)
        calibrate.assert_called_once_with("right", True, gripper_only=True)


if __name__ == "__main__":
    unittest.main()
