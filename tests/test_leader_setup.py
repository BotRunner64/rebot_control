"""Verify isolated handle calibration without connecting to hardware."""

from contextlib import redirect_stdout
import copy
import io
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, call, patch

from rebot import bindings
from rebot.apps import leader_setup
from rebot.devices import stararm


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
        self.config = Path(temporary.name) / "leader.local.yaml"
        self.arms = {
            "left": {"port": "/fake/left", "baudrate": 1000000,
                     "reference_deg": [3.2, -.3, .4, -4., -2.1, -2.4, 0.],
                     "directions": [1] * 7},
            "right": {"port": "/fake/right", "baudrate": 1000000,
                      "reference_deg": [2.3, -.4, .4, -3.5, -2.7, -2.4, -.2]}}
        bindings.save_leaders(self.arms, self.config)

    def run_capture(self, samples, side="left", attempts=20):
        bus = monitor_bus([[value] for value in samples])
        with patch.object(stararm.serial, "Serial") as connection, \
                patch.object(stararm, "UartServoManager", return_value=bus), \
                patch.object(stararm.time, "sleep"), redirect_stdout(io.StringIO()):
            leader_setup.calibrate(side, replace=True, config=self.config, gripper_only=True)
        connection.assert_called_once_with(self.arms[side]["port"], baudrate=1000000,
                                           timeout=0, exclusive=True)
        self.assertEqual(bus.send_sync_servo_monitor.call_args_list,
                         [call([6], realtime=True)] * attempts)
        return bindings.load_leaders(self.config)

    def test_updates_only_selected_handle_and_preserves_binding_and_six_axes(self):
        for side, closed in (("left", 1.1), ("right", .4)):
            with self.subTest(side=side):
                bindings.save_leaders(self.arms, self.config)
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

    def test_requires_valid_existing_reference_and_explicit_replace_before_opening_port(self):
        # `capture_gripper` is stubbed only so that a reference wrongly accepted
        # here fails this test's assertions instead of reading a mocked serial port.
        with patch.object(stararm.serial, "Serial") as connection, \
                patch.object(leader_setup, "capture_gripper", return_value=1.1), \
                redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(ValueError, "--replace"):
                leader_setup.calibrate("left", config=self.config, gripper_only=True)
            for reference in (None, [0.] * 6, [0.] * 6 + [float("nan")]):
                with self.subTest(reference=reference):
                    arms = copy.deepcopy(self.arms)
                    arms["left"]["reference_deg"] = reference
                    bindings.save_leaders(arms, self.config)
                    original = self.config.read_bytes()
                    with self.assertRaisesRegex(ValueError, "seven existing"):
                        leader_setup.calibrate("left", replace=True, config=self.config,
                                               gripper_only=True)
                    self.assertEqual(self.config.read_bytes(), original)
                    connection.assert_not_called()

    def test_binding_change_during_capture_is_not_overwritten(self):
        changed = copy.deepcopy(self.arms)
        changed["left"]["baudrate"] = 115200

        def capture(bus):
            bindings.save_leaders(changed, self.config)
            return 1.1

        with patch.object(stararm.serial, "Serial"), patch.object(stararm, "UartServoManager"), \
                patch.object(leader_setup, "capture_gripper", side_effect=capture):
            with self.assertRaisesRegex(ValueError, "binding changed"):
                leader_setup.calibrate("left", replace=True, config=self.config, gripper_only=True)
        self.assertEqual(bindings.load_leaders(self.config), changed)

    def test_default_reader_still_requires_all_seven_fresh_servo_angles(self):
        values = [1., 2., 3., 4., 5., 6., 7.]
        bus = monitor_bus([values])
        self.assertEqual(stararm.read_angles(bus), values)
        bus.send_sync_servo_monitor.assert_called_once_with(list(range(7)), realtime=True)
        bus = monitor_bus([values[:6]])
        with self.assertRaisesRegex(ValueError, "IDs \\[6\\]"):
            stararm.read_angles(bus)
        with self.assertRaises(stararm.AngleFeedbackError):
            stararm.read_angles(monitor_bus([values[:6]]))

    def test_full_calibration_retains_existing_behavior(self):
        reference = [1.] * 7
        with patch.object(stararm.serial, "Serial") as connection, \
                patch.object(stararm, "UartServoManager"), \
                patch.object(leader_setup, "capture_pose", return_value=reference) as capture, \
                patch.object(leader_setup, "capture_gripper") as gripper, redirect_stdout(io.StringIO()):
            leader_setup.calibrate("left", replace=True, config=self.config)
        connection.assert_called_once_with("/fake/left", baudrate=1000000, timeout=0, exclusive=True)
        capture.assert_called_once()
        gripper.assert_not_called()
        saved = bindings.load_leaders(self.config)
        self.assertEqual(saved["left"]["reference_deg"], reference)
        self.assertEqual(saved["right"], self.arms["right"])

    def test_full_calibration_requires_replace_and_a_binding_before_opening_port(self):
        with patch.object(stararm.serial, "Serial") as connection:
            with self.assertRaisesRegex(ValueError, "--replace"):
                leader_setup.calibrate("left", config=self.config)
            bindings.save_leaders({"right": self.arms["right"]}, self.config)
            with self.assertRaisesRegex(ValueError, "Bind left"):
                leader_setup.calibrate("left", replace=True, config=self.config)
        connection.assert_not_called()


class SelectionTests(unittest.TestCase):
    """What every leader consumer (`rebot leader view`, teleoperation) requires of the bindings."""

    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.config = Path(temporary.name) / "leader.local.yaml"
        self.arms = {side: {"port": f"/fake/{side}", "baudrate": 1000000,
                            "reference_deg": [1., 2., 3., 4., 5., 6., 7.]}
                     for side in ("left", "right")}

    def select(self, side="both"):
        bindings.save_leaders(self.arms, self.config)
        return bindings.select_leaders(self.config, side)

    def test_calibrated_bindings_are_selected(self):
        selected = self.select()
        self.assertEqual(list(selected), ["left", "right"])
        self.assertEqual((selected["right"].port, selected["right"].baudrate, selected["right"].reference_deg),
                         ("/fake/right", 1000000, (1., 2., 3., 4., 5., 6., 7.)))
        self.assertEqual(list(self.select("left")), ["left"])

    def test_both_arms_must_be_bound_and_calibrated_on_distinct_ports(self):
        del self.arms["right"]
        with self.assertRaisesRegex(ValueError, "Bind the right Star Arm"):
            self.select()
        for reference in (None, [0.] * 6, [0.] * 6 + [float("nan")], [0.] * 6 + ["0"]):
            self.arms["right"] = {"port": "/fake/right", "baudrate": 1000000, "reference_deg": reference}
            with self.subTest(reference=reference), self.assertRaisesRegex(ValueError, "calibrate right"):
                self.select()
        self.arms["right"] = dict(self.arms["left"])
        with self.assertRaisesRegex(ValueError, "distinct serial ports"):
            self.select()

    def test_invalid_file_is_rejected(self):
        for arms in ({"left": {"port": 3, "baudrate": 1000000}},
                     {"left": {"port": "/fake/left", "baudrate": 0}},
                     {"middle": {"port": "/fake/left", "baudrate": 1000000}}):
            with self.subTest(arms=arms):
                bindings.save_leaders(arms, self.config)
                with self.assertRaisesRegex(ValueError, "Invalid configuration"):
                    bindings.load_leaders(self.config)


if __name__ == "__main__":
    unittest.main()
