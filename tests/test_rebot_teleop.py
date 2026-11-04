"""Offline leader mapping and virtual-CAN physical teleoperation lifecycle tests."""

from contextlib import redirect_stderr, redirect_stdout
import io
import math
import signal
import struct
import unittest
from unittest.mock import MagicMock, patch

import numpy as np

import rebot_disable
import rebot_gravity
import rebot_mit as mit
import rebot_teleop as teleop
from test_rebot_hand_guiding import VirtualMotors, reply


class MappingTests(unittest.TestCase):
    def test_calibrated_signs_gripper_ratio_and_no_angle_wrapping(self):
        reference = np.array([10.] * 7)
        low, high = np.array([-10.] * 6 + [0.]), np.array([10.] * 6 + [math.radians(270)])
        result = teleop.map_target(reference + [190, 20, 30, 40, 50, 60, 45], reference, low, high)
        np.testing.assert_allclose(result, np.radians([190, 20, -30, -40, -50, 60, 270]))
        clipped = teleop.map_target(reference + [800.] * 7, reference, low, high)
        np.testing.assert_allclose(clipped, [10, 10, -10, -10, -10, 10, high[6]])
        closed = teleop.map_target(reference - np.array([0.] * 6 + [10.]), reference, low, high)
        self.assertEqual(closed[6], 0)
        with self.assertRaises(ValueError):
            teleop.map_target([float("nan")] * 7, reference, low, high)


class TeleopLifecycleTests(unittest.TestCase):
    def setUp(self):
        self.virtual = {"can0": VirtualMotors(), "can1": VirtualMotors(100)}
        for bus in self.virtual.values():
            bus.params[(7, mit.WATCHDOG)] = 4000
            bus.params[(7, mit.MODE)] = 0
            bus.modes[7] = 0
        self.arms = {name: dict(channel=channel, models=["rs-06"] * 3 + ["rs-00"] * 4,
                                reference_rad=[.2] * 7, motor_uids=self.virtual[channel].uids)
                     for name, channel in (("left", "can0"), ("right", "can1"))}
        self.leaders = {name: dict(port=f"/dev/fake-{name}", baudrate=1000000, reference_deg=[10.] * 7)
                        for name in ("left", "right")}
        self.clock, self.commands, self.serials = 0., [], []
        self.stop_after = 4
        self.signal_handlers = {}

        def serial_port(port, **kwargs):
            uart = MagicMock()
            uart.port = port
            uart.__enter__.return_value = uart
            self.serials.append(uart)
            return uart

        def now():
            self.clock += .00001
            return self.clock

        def sleep(duration):
            self.clock += duration

        original_command = mit.MotorBus.command_position

        def command(bus, positions, velocities, gravity, kp, kd, deadline):
            self.commands.append(dict(channel=bus.channel, position=np.array(positions),
                                      velocity=np.array(velocities), gravity=np.array(gravity),
                                      kp=np.array(kp), kd=np.array(kd), deadline=deadline, now=self.clock))
            return original_command(bus, positions, velocities, gravity, kp, kd, deadline)

        def signals(sig, handler):
            self.signal_handlers[sig] = handler
            return signal.SIG_DFL

        self.keys = MagicMock()
        self.keys.__enter__.return_value.pressed.side_effect = lambda: len(self.commands) >= self.stop_after
        patches = [patch.object(mit, "check_channel"),
                   patch.object(rebot_gravity.GravityModel, "gravity", side_effect=AssertionError("Gravity is disabled during teleoperation")),
                   patch.object(mit.can, "Bus", side_effect=lambda **kw: self.virtual[kw["channel"]]),
                   patch.object(teleop, "load_config", return_value=self.arms),
                   patch.object(teleop, "load_leaders", return_value=self.leaders),
                   patch.object(teleop, "resolve_channel", side_effect=lambda arm: arm["channel"]),
                   patch.object(teleop.serial, "Serial", side_effect=serial_port),
                   patch.object(teleop, "UartServoManager", side_effect=lambda uart: uart),
                   patch.object(teleop, "read_angles", return_value=[30.] * 7),
                   patch.object(teleop, "release_torque"),
                   patch.object(teleop, "StopKey", return_value=self.keys),
                   patch.object(teleop.time, "monotonic", now),
                   patch.object(teleop.time, "sleep", sleep),
                   patch.object(teleop.signal, "signal", side_effect=signals),
                   patch.object(mit.MotorBus, "command_position", command)]
        for item in patches:
            item.start()
            self.addCleanup(item.stop)

    def run_teleop(self, **kwargs):
        with redirect_stdout(io.StringIO()):
            return teleop.teleop(**kwargs)

    def assert_holding(self, channels=("can0", "can1"), count=7):
        for channel in channels:
            bus = self.virtual[channel]
            self.assertTrue(bus.closed)
            self.assertTrue(all(bus.modes[i] == 2 for i in range(1, count + 1)))
            self.assertTrue(all(bus.params[(i, mit.WATCHDOG)] == 0 for i in range(1, count + 1)))
            self.assertFalse(any((m.arbitration_id >> 24) & 31 in (4, 6, 7, 22, 23, 25) for m in bus.sent))
            last = [m for m in bus.sent if (m.arbitration_id >> 24) & 31 == 1][-count:]
            self.assertEqual({m.arbitration_id & 255 for m in last}, set(range(1, count + 1)))
            for message in last:
                native = mit.decode_mit(message, self.arms["left"]["models"][(message.arbitration_id & 255)-1])
                self.assertGreater(native["kp"], 0)
                self.assertGreater(native["kd"], 0)
                self.assertAlmostEqual(native["velocity"], 0, delta=.001)

    def test_both_arms_send_mapped_positions_directly_and_hold_on_stop(self):
        changed = [70, 50, -40, -50, 20, 100, 55]
        with patch.object(teleop, "read_angles", side_effect=[[30.] * 7] * 4 + [changed] * 2):
            self.run_teleop()
        self.assertEqual(len(self.commands), 4)
        for command in self.commands:
            self.assertGreater(command["deadline"], command["now"])
            np.testing.assert_array_equal(command["velocity"], np.zeros(7))
        for channel in self.virtual:
            commands = [c for c in self.commands if c["channel"] == channel]
            # Both the first target and subsequent large changes are sent directly.
            np.testing.assert_allclose(commands[0]["position"],
                                       .2 + np.radians([20, 20, 0, -20, -20, 20, 120]))
            np.testing.assert_allclose(commands[1]["position"],
                                       .2 + np.radians([60, 40, 50, 60, -10, 90, 270]))
        self.assert_holding()
        for uart in self.serials:
            uart.__exit__.assert_called_once()

    def test_invalid_initial_gripper_pose_prevents_either_arm_enabling(self):
        self.arms["right"]["reference_rad"][6] = .4
        with self.assertRaisesRegex(ValueError, "follower pose.*limits"):
            self.run_teleop()
        self.assertFalse(any((m.arbitration_id >> 24) & 31 in (1, 3)
                             for bus in self.virtual.values() for m in bus.sent))

    def test_all_targets_are_clipped_to_official_limits(self):
        high = [170, 200, -210, -100, -100, 170, 60]
        low = [-150, -10, 30, 110, 120, -150, 0]
        with patch.object(teleop, "read_angles", side_effect=[[30.] * 7] * 2 + [high] * 2 + [low] * 2):
            self.run_teleop()
        for channel in self.virtual:
            commands = [c for c in self.commands if c["channel"] == channel]
            np.testing.assert_allclose(commands[0]["position"],
                                       .2 + np.radians([145, 170, 200, 90, 90, 130, 270]))
            np.testing.assert_allclose(commands[1]["position"],
                                       .2 + np.radians([-145, 0, 0, -80, -90, -130, 0]))
        self.assert_holding()

    def test_j3_feedback_above_old_model_limit_is_accepted(self):
        original = mit.MotorBus.positions

        def read(bus):
            positions = original(bus)
            positions[2] = .2 + math.radians(190)
            return positions

        with patch.object(mit.MotorBus, "positions", read):
            self.run_teleop()
        self.assertEqual(len(self.commands), 4)
        self.assert_holding()

    def test_initial_j6_pose_outside_official_limits_prevents_enabling(self):
        original = mit.MotorBus.positions

        def read(bus):
            positions = original(bus)
            positions[5] = .2 + math.radians(140)
            return positions

        with patch.object(mit.MotorBus, "positions", read):
            with self.assertRaisesRegex(ValueError, "follower pose.*limits"):
                self.run_teleop()
        self.assertFalse(any((m.arbitration_id >> 24) & 31 in (1, 3)
                             for bus in self.virtual.values() for m in bus.sent))

    def test_gripper_opening_cannot_exceed_official_limit(self):
        with self.assertRaisesRegex(ValueError, "official 270-degree limit"):
            self.run_teleop(gripper_open_deg=300)
        self.assertFalse(self.serials)

    def test_feedback_gap_still_stops_after_speed_limiter_removal(self):
        def delayed_tick(duration):
            self.clock += .11

        with patch.object(teleop.time, "sleep", delayed_tick):
            with self.assertRaisesRegex(RuntimeError, "feedback gap"):
                self.run_teleop()
        self.assertEqual(len(self.commands), 2)
        self.assert_holding()

    def test_teleop_and_exit_hold_use_zero_feedforward_with_existing_gains(self):
        self.run_teleop()
        kp = [50, 150, 150, 50, 50, 50, 50]
        kd = [3, 10, 10, 5, 4, 4, 4]
        for command in self.commands:
            np.testing.assert_array_equal(command["gravity"], np.zeros(7))
            np.testing.assert_array_equal(command["kp"], kp)
            np.testing.assert_array_equal(command["kd"], kd)
        # Cover startup/adoption, following, and cleanup, including quantization.
        for bus in self.virtual.values():
            frames = [m for m in bus.sent if (m.arbitration_id >> 24) & 31 == 1]
            self.assertTrue(frames)
            for frame in frames:
                index = (frame.arbitration_id & 255) - 1
                self.assertEqual((frame.arbitration_id >> 8) & 65535, 32767)
                native = mit.decode_mit(frame, self.arms["left"]["models"][index])
                self.assertAlmostEqual(native["kp"], kp[index], delta=.08)
                self.assertAlmostEqual(native["kd"], kd[index], delta=.002)
        self.assert_holding()

    def test_single_arm_no_gripper_never_writes_motor_seven(self):
        self.run_teleop(side="right", gripper=False)
        self.assert_holding(("can1",), count=6)
        self.assertFalse(self.virtual["can0"].sent)
        self.assertEqual(self.virtual["can1"].modes[7], 0)
        self.assertTrue(all(len(c["position"]) == 6 for c in self.commands))
        self.assertFalse(any((m.arbitration_id & 255) == 7 and (m.arbitration_id >> 24) & 31 != 0
                             for m in self.virtual["can1"].sent))

    def test_partial_leader_loss_stops_both_without_another_target(self):
        # Two startup reads, one valid paired tick, then a partial frame.
        with patch.object(teleop, "read_angles", side_effect=[[30.] * 7] * 5 + [ValueError("Missing servo IDs [3]")]):
            with self.assertRaisesRegex(ValueError, "Missing servo"):
                self.run_teleop()
        self.assertEqual(len(self.commands), 2)
        self.assert_holding()

    def test_slow_leader_feedback_is_not_forwarded(self):
        reads = 0

        def read(bus):
            nonlocal reads
            reads += 1
            if reads > 2:
                self.clock += .09
            return [30.] * 7

        with patch.object(teleop, "read_angles", side_effect=read):
            with self.assertRaisesRegex(TimeoutError, "Star Arm feedback is too old"):
                self.run_teleop()
        self.assertFalse(self.commands)
        self.assert_holding()

    def test_slow_follower_read_prevents_sending_either_new_target(self):
        reads = 0
        original = mit.MotorBus.positions

        def read(bus):
            nonlocal reads
            reads += 1
            if reads == 5:
                self.clock += .09
            return original(bus)

        with patch.object(mit.MotorBus, "positions", read):
            with self.assertRaisesRegex(TimeoutError, "Paired teleoperation feedback deadline"):
                self.run_teleop()
        self.assertFalse(self.commands)
        self.assert_holding()

    def test_native_feedback_loss_stops_and_holds_both(self):
        original = mit.MotorBus.command_position

        def drop_gripper(bus, *args):
            if bus.channel == "can1":
                self.virtual[bus.channel].drop_holds.add(7)
                try:
                    return original(bus, *args)
                finally:
                    self.virtual[bus.channel].drop_holds.clear()
            return original(bus, *args)

        with patch.object(mit.MotorBus, "command_position", drop_gripper):
            with self.assertRaisesRegex(TimeoutError, "missing native motion feedback.*7"):
                self.run_teleop()
        self.assert_holding()

    def test_bad_second_uid_prevents_either_arm_enabling(self):
        self.arms["right"]["motor_uids"] = dict(self.arms["right"]["motor_uids"])
        self.arms["right"]["motor_uids"][7] = "ff" * 8
        with self.assertRaisesRegex(ValueError, "UIDs"):
            self.run_teleop()
        for bus in self.virtual.values():
            self.assertFalse(any((m.arbitration_id >> 24) & 31 in (1, 3) for m in bus.sent))

    def test_invalid_binding_is_rejected_before_opening_devices(self):
        for field, value in (("reference_deg", [math.nan] * 7), ("port", self.leaders["left"]["port"])):
            original = self.leaders["right"][field]
            self.leaders["right"][field] = value
            with self.assertRaises(ValueError):
                self.run_teleop()
            self.leaders["right"][field] = original
        self.assertFalse(self.serials)

    def test_non_mit_gripper_is_rejected_before_enabling_either_arm(self):
        self.virtual["can1"].params[(7, mit.MODE)] = 1
        with self.assertRaisesRegex(ValueError, "MIT mode"):
            self.run_teleop()
        self.assertFalse(any((m.arbitration_id >> 24) & 31 in (1, 3)
                             for bus in self.virtual.values() for m in bus.sent))

    def test_partial_enable_failure_holds_every_motor_already_touched(self):
        self.virtual["can1"].fail_enable = 3
        with self.assertRaises(TimeoutError):
            self.run_teleop()
        self.assertTrue(all(bus.closed for bus in self.virtual.values()))
        self.assertTrue(all(self.virtual["can0"].modes[i] == 2 for i in range(1, 8)))
        self.assertTrue(all(self.virtual["can1"].modes[i] == 2 for i in (1, 2)))
        self.assertFalse(any((m.arbitration_id >> 24) & 31 == 4
                             for bus in self.virtual.values() for m in bus.sent))

    def test_sigterm_stops_before_forwarding_new_targets(self):
        reads = 0

        def read(bus):
            nonlocal reads
            reads += 1
            if reads == 3:
                self.signal_handlers[signal.SIGTERM](signal.SIGTERM, None)
            return [30.] * 7

        with patch.object(teleop, "read_angles", side_effect=read):
            self.run_teleop()
        self.assertFalse(self.commands)
        self.assert_holding()

    def test_manual_disable_can_prepare_gripper_explicitly(self):
        self.run_teleop(side="right")
        # A fresh virtual CAN socket stands in for a separately invoked CLI.
        self.virtual["can1"].closed = False
        with patch.object(rebot_disable, "load_config", return_value=self.arms), \
                patch.object(rebot_disable, "resolve_channel", side_effect=lambda arm: arm["channel"]), \
                redirect_stdout(io.StringIO()):
            self.assertTrue(rebot_disable.disable(side="right", prepare_mit=True, gripper=True))
        self.assertEqual(set(self.virtual["can1"].modes.values()), {0})
        self.assertEqual(self.virtual["can1"].params[(7, mit.MODE)], 0)

    def test_cli_defaults_to_both_with_grippers(self):
        with patch.object(teleop.sys, "argv", ["rebot_teleop.py"]), patch.object(teleop, "teleop") as run:
            self.assertEqual(teleop.main(), 0)
            self.assertEqual(run.call_args.kwargs["side"], "both")
            self.assertTrue(run.call_args.kwargs["gripper"])

    def test_cli_rejects_removed_speed_options(self):
        for option in ("--speed", "--gripper-speed"):
            with self.subTest(option=option), \
                    patch.object(teleop.sys, "argv", ["rebot_teleop.py", option, "0.5"]), \
                    patch.object(teleop, "teleop") as run, redirect_stderr(io.StringIO()), \
                    self.assertRaises(SystemExit) as error:
                teleop.main()
            self.assertEqual(error.exception.code, 2)
            run.assert_not_called()


class SevenMotorTransportTests(unittest.TestCase):
    def test_gripper_encoding_is_explicit_and_range_checked(self):
        with self.assertRaises(ValueError):
            mit.encode_mit(7, "rs-00", 0, 0)
        frame = mit.encode_mit(7, "rs-00", 1., 0, kp=50, kd=4, allow_gripper=True)
        self.assertEqual(frame.arbitration_id & 255, 7)
        self.assertAlmostEqual(mit.decode_mit(frame, "rs-00")["position"], 1., delta=.001)
        for position in (math.nan, 4 * math.pi + .01):
            with self.assertRaises(ValueError):
                mit.encode_mit(7, "rs-00", position, 0, allow_gripper=True)

    def test_gripper_temperature_and_faults_are_checked(self):
        with patch.object(mit, "check_channel"), patch.object(mit.can, "Bus", return_value=VirtualMotors()):
            with mit.MotorBus("can0", ["rs-06"] * 3 + ["rs-00"] * 4, motor_count=7) as bus:
                with self.assertRaisesRegex(RuntimeError, "temperature=70"):
                    bus.inspect(reply(2, 7, struct.pack(">HHHH", 32767, 32767, 32767, 700)))
                with self.assertRaisesRegex(RuntimeError, "fault"):
                    bus.inspect(reply(21, 7, b"\x01" + bytes(7)))


if __name__ == "__main__":
    unittest.main()
