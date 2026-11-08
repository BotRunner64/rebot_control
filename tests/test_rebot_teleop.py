"""Offline leader mapping and virtual-CAN physical teleoperation lifecycle tests."""

from contextlib import redirect_stderr, redirect_stdout
import io
import math
import multiprocessing as mp
import os
import queue
import time
import signal
import struct
import unittest
from unittest.mock import MagicMock, patch

import numpy as np

import rebot_home
import rebot_mit as mit
import rebot_teleop as teleop
from test_rebot_hand_guiding import VirtualMotors, reply
from teleop_process_fakes import BusFactory, SourceFactory, MeasuredMotors
from rebot_teleop_runtime import SharedFrame, fresh, LoopTiming


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


class SharedFrameTests(unittest.TestCase):
    def setUp(self):
        self.context = mp.get_context("spawn")

    def test_latest_snapshot_overwrites_without_queueing_and_preserves_source_age(self):
        frame = SharedFrame(self.context, 2)
        source_at = time.monotonic() - .04
        frame.publish([1, 2], source_at)
        frame.publish([3, 4], source_at)
        values, published, acquired = frame.read()
        np.testing.assert_array_equal(values, [3, 4])
        self.assertGreater(published, acquired)
        self.assertEqual(acquired, source_at)
        with self.assertRaisesRegex(TimeoutError, "stale"):
            fresh(frame, time.monotonic(), .02, "Source")

    def test_unpublished_invalid_and_abandoned_locked_frames_are_bounded(self):
        frame = SharedFrame(self.context, 2)
        with self.assertRaises(TimeoutError):
            fresh(frame, time.monotonic(), .1, "Missing")
        with self.assertRaises(ValueError):
            frame.publish([math.nan, 2], time.monotonic())
        frame.lock.acquire()
        try:
            began = time.monotonic()
            with self.assertRaisesRegex(TimeoutError, "lock"):
                frame.read()
            self.assertLess(time.monotonic() - began, .2)
        finally:
            frame.lock.release()

    def test_official_limits_apply_to_targets_and_feedback_without_old_model_limit(self):
        arm = {"reference_rad": [.2] * 7, "models": ["rs-06"] * 3 + ["rs-00"] * 4}
        reference, low, high = teleop.position_bounds(arm, 7, 270)
        np.testing.assert_allclose(np.degrees(low), [-145, 0, 0, -80, -90, -130, 0])
        np.testing.assert_allclose(np.degrees(high), [145, 170, 200, 90, 90, 130, 270])
        q = reference.copy()
        q[2] += math.radians(190)
        teleop.validate_pose(q, reference, low, high)
        q[5] += math.radians(140)
        with self.assertRaisesRegex(ValueError, r"J6: relative=140\.00 deg, limits=\[-130\.00, 130\.00\] deg"):
            teleop.validate_pose(q, reference, low, high)

    def test_arm_feedback_accepts_logged_pose_and_checks_wider_margin_on_both_bounds(self):
        arm = {"reference_rad": [.2] * 7, "models": ["rs-06"] * 3 + ["rs-00"] * 4}
        for count in (6, 7):
            reference, low, high = teleop.position_bounds(arm, count, 270)
            q = reference.copy()
            q[3] += math.radians(-81.73)
            np.testing.assert_array_equal(teleop.validate_pose(q, reference, low, high), q)
            for joint in range(6):
                for bound, direction in ((low[joint], -1), (high[joint], 1)):
                    with self.subTest(count=count, joint=joint, direction=direction):
                        q = reference.copy()
                        q[joint] += bound + direction * .099
                        teleop.validate_pose(q, reference, low, high)
                        q[joint] = reference[joint] + bound + direction * .101
                        with self.assertRaisesRegex(ValueError, rf"J{joint + 1}:.*tolerance=5\.73 deg"):
                            teleop.validate_pose(q, reference, low, high)

    def test_gripper_feedback_ignores_travel_limits_but_requires_finite_complete_feedback(self):
        arm = {"reference_rad": [.2] * 7, "models": ["rs-06"] * 3 + ["rs-00"] * 4}
        reference, low, high = teleop.position_bounds(arm, 7, 180)
        for relative in (-20., 200., 280.):
            with self.subTest(relative=relative):
                q = reference.copy()
                q[6] += math.radians(relative)
                np.testing.assert_array_equal(teleop.validate_pose(q, reference, low, high), q)
        for invalid in (math.nan, math.inf, -math.inf):
            q[6] = invalid
            with self.subTest(invalid=invalid), self.assertRaisesRegex(ValueError, "finite feedback"):
                teleop.validate_pose(q, reference, low, high)
        with self.assertRaisesRegex(ValueError, "finite feedback"):
            teleop.validate_pose(reference[:6], reference, low, high)

    def test_timing_skips_missed_ticks_instead_of_bursting_commands(self):
        stop = MagicMock()
        with patch("rebot_teleop_runtime.time.monotonic", return_value=1.):
            timing = LoopTiming("motor", 200)
        with patch("rebot_teleop_runtime.time.monotonic", return_value=1.022):
            timing.pace(stop)
        self.assertEqual(timing.skipped, 4)
        self.assertAlmostEqual(stop.wait.call_args.args[0], .003)


class ProcessLifecycleTests(unittest.TestCase):
    def setUp(self):
        self.context = mp.get_context("spawn")
        self.counts = self.context.Array("i", 2)
        self.hardware_reports = self.context.Queue()
        self.arms = {side: {"reference_rad": [.2] * 7, "models": ["rs-06"] * 3 + ["rs-00"] * 4,
                            "motor_uids": MeasuredMotors(offset).uids}
                     for side, offset in (("left", 0), ("right", 100))}
        self.leaders = {side: {"port": f"fake-{side}", "baudrate": 1000000, "reference_deg": [10.] * 7}
                        for side in ("left", "right")}
        self.snapshots = []
        self.addCleanup(self.hardware_reports.join_thread)
        self.addCleanup(self.hardware_reports.close)

    def run_teleop(self, failure="", stall=False, side="both", gripper=True, signal_stop=False):
        sides = ("left", "right") if side == "both" else (side,)
        channels = {name: "can0" if name == "left" else "can1" for name in sides}
        keys = MagicMock()
        began = time.monotonic()

        def pressed():
            done = (not failure and not stall and sum(self.counts) >= 12) or time.monotonic() - began > 8
            if done and signal_stop:
                os.kill(os.getpid(), signal.SIGTERM)
                return False
            return done

        keys.__enter__.return_value.pressed.side_effect = pressed
        try:
            with patch.object(teleop, "load_bindings", return_value=(sides, self.arms, self.leaders, channels)), \
                    patch.object(teleop, "StopKey", return_value=keys), \
                    redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
                return teleop.teleop(side=side, gripper=gripper, startup_timeout=8, shutdown_timeout=3,
                    source_factory=SourceFactory(self.hardware_reports, stall),
                    bus_factory=BusFactory(self.hardware_reports, self.counts, failure))
        finally:
            while True:
                try:
                    self.snapshots.append(self.hardware_reports.get(timeout=.1))
                except queue.Empty:
                    break
            self.assertFalse([p for p in mp.active_children()
                              if p.name.startswith(("leader-", "motor-")) or p.name == "target"])

    def assert_holding(self, channels=("can0", "can1"), count=7):
        motors = {record["channel"]: record for record in self.snapshots if record["kind"] == "motor"}
        for channel in channels:
            record = motors[channel]
            self.assertTrue(record["closed"])
            self.assertEqual(record["enabled"], list(range(1, count + 1)))
            self.assertTrue(all(record["modes"][i] == 2 for i in range(1, count + 1)))
            self.assertNotIn(4, record["types"])
            self.assertTrue(record["tau_zero"])
            self.assertTrue(record["velocity_zero"])
            for hold in record["hold"]:
                self.assertGreater(hold["kp"], 0)
                self.assertGreater(hold["kd"], 0)
                self.assertAlmostEqual(hold["velocity"], 0, delta=.001)
        return motors

    def test_real_spawn_runs_independent_hardware_with_existing_mit_parameters(self):
        messages = self.run_teleop()
        motors = self.assert_holding()
        pids = [r["pid"] for r in self.snapshots]
        self.assertEqual(len(set(pids)), 4)
        self.assertNotIn(os.getpid(), pids)
        for record in motors.values():
            self.assertGreater(record["calls"], 0)
            self.assertEqual(record["position_queries"], 21)
            np.testing.assert_allclose(record["first"], .2 + np.radians([20, 20, 0, -20, -20, 20, 120]))
            np.testing.assert_allclose([p["kp"] for p in record["parameters"]],
                                       [50, 150, 150, 50, 50, 50, 50], atol=.08)
            np.testing.assert_allclose([p["kd"] for p in record["parameters"]],
                                       [3, 10, 10, 5, 4, 4, 4], atol=.002)
        rates = {r["worker"]: r for r in messages if r["type"] == "timing"}
        self.assertEqual(rates["target"]["requested_hz"], 500)
        self.assertEqual(rates["motor-left"]["requested_hz"], 200)
        self.assertEqual(rates["motor-right"]["requested_hz"], 200)
        self.assertGreater(rates["target"]["count"], 0)

    def test_single_arm_without_gripper_owns_only_selected_hardware(self):
        self.run_teleop(side="right", gripper=False)
        motors = self.assert_holding(("can1",), 6)
        self.assertEqual(len(self.snapshots), 2)
        self.assertEqual(motors["can1"]["modes"][7], 0)
        self.assertEqual(motors["can1"]["position_queries"], 18)

    def test_bad_uid_on_one_side_prevents_enabling_either_arm(self):
        with self.assertRaisesRegex(RuntimeError, "UIDs"):
            self.run_teleop(failure="uid")
        motors = [r for r in self.snapshots if r["kind"] == "motor"]
        self.assertTrue(motors)
        self.assertTrue(all(not r["enabled"] for r in motors))
        self.assertTrue(all(3 not in r["types"] for r in motors))

    def test_wrong_mode_on_one_side_prevents_enabling_either_arm(self):
        with self.assertRaisesRegex(RuntimeError, "MIT mode"):
            self.run_teleop(failure="prepare")
        self.assertTrue(all(not r["enabled"] for r in self.snapshots if r["kind"] == "motor"))

    def test_missing_can_reply_stops_and_holds_both_arms(self):
        with self.assertRaisesRegex(RuntimeError, "missing native motion feedback"):
            self.run_teleop(failure="command")
        self.assert_holding()

    def test_partial_enable_failure_holds_touched_motors(self):
        with self.assertRaisesRegex(RuntimeError, "mode 2"):
            self.run_teleop(failure="enable")
        records = {r["channel"]: r for r in self.snapshots if r["kind"] == "motor"}
        self.assertTrue(all(r["closed"] for r in records.values()))
        self.assertTrue(all(4 not in r["types"] for r in records.values()))
        self.assertEqual(records["can1"]["enabled"], [1, 2, 3])

    def test_slow_source_cannot_be_made_fresh_by_500hz_republication(self):
        with self.assertRaisesRegex(RuntimeError, "stale"):
            self.run_teleop(stall=True)
        self.assert_holding()

    def test_abrupt_motor_process_death_is_detected_and_peer_holds(self):
        with self.assertRaisesRegex(RuntimeError, "exited unexpectedly"):
            self.run_teleop(failure="crash")
        self.assert_holding(("can0",))

    def test_parent_sigterm_performs_cooperative_hold_cleanup(self):
        self.run_teleop(signal_stop=True)
        self.assert_holding()

    def test_cli_and_invalid_gripper_range(self):
        with patch.object(teleop.sys, "argv", ["rebot_teleop.py"]), patch.object(teleop, "teleop") as run:
            self.assertEqual(teleop.main(), 0)
            self.assertEqual(run.call_args.kwargs["side"], "both")
            self.assertTrue(run.call_args.kwargs["gripper"])
        with self.assertRaises(ValueError):
            teleop.teleop(gripper_open_deg=300)
        for option in ("--speed", "--gripper-speed"):
            with self.subTest(option=option), \
                    patch.object(teleop.sys, "argv", ["rebot_teleop.py", option, "0.5"]), \
                    redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as error:
                teleop.main()
            self.assertEqual(error.exception.code, 2)

    def test_parent_heartbeat_loss_stops_workers_independently(self):
        original = SharedFrame.publish

        def publish(frame, values, source_at):
            # This patch runs only in the supervisor: spawn starts fresh interpreters.
            if frame.size == 0 and sum(self.counts) >= 8:
                return
            return original(frame, values, source_at)

        with patch.object(SharedFrame, "publish", publish):
            with self.assertRaisesRegex(RuntimeError, "Supervisor heartbeat"):
                self.run_teleop(failure="heartbeat")
        self.assert_holding()


class BindingAndDisableTests(unittest.TestCase):
    def test_invalid_bindings_are_rejected_before_workers_start(self):
        arms = {name: {"reference_rad": [0.] * 7,
                       "motor_uids": MeasuredMotors(offset).uids, "channel": name}
                for name, offset in (("left", 0), ("right", 100))}
        leaders = {name: {"port": f"fake-{name}", "reference_deg": [0.] * 7} for name in arms}
        with patch.object(teleop, "load_config", return_value=arms), \
                patch.object(teleop, "load_leaders", return_value=leaders), \
                patch.object(teleop, "resolve_channel", side_effect=lambda arm: arm["channel"]):
            leaders["right"]["reference_deg"][0] = math.nan
            with self.assertRaises(ValueError):
                teleop.load_bindings(None, None, "both")
            leaders["right"]["reference_deg"][0] = 0
            leaders["right"]["port"] = leaders["left"]["port"]
            with self.assertRaisesRegex(ValueError, "distinct"):
                teleop.load_bindings(None, None, "both")

    def test_home_disable_stops_all_seven_without_changing_parameters(self):
        virtual = MeasuredMotors()
        virtual.modes = dict.fromkeys(range(1, 8), 2)
        for motor in range(1, 8):
            virtual.params[(motor, mit.MODE)] = 1
        parameters = dict(virtual.params)
        arms = {"right": {"models": ["rs-06"] * 3 + ["rs-00"] * 4, "motor_uids": virtual.uids}}
        with patch.object(mit, "check_channel"), patch.object(mit.can, "Bus", return_value=virtual), \
                patch.object(rebot_home, "load_config", return_value=arms), \
                patch.object(rebot_home, "resolve_channel", return_value="can0"), redirect_stdout(io.StringIO()):
            self.assertTrue(rebot_home.disable(side="right"))
        self.assertEqual(set(virtual.modes.values()), {0})
        self.assertEqual(virtual.params, parameters)
        self.assertEqual({(m.arbitration_id >> 24) & 31 for m in virtual.sent}, {0, 4})


class SevenMotorTransportTests(unittest.TestCase):
    def test_mit_feedback_positions_are_fresh_and_do_not_query_parameters(self):
        virtual = MeasuredMotors()
        with patch.object(mit, "check_channel"), patch.object(mit.can, "Bus", return_value=virtual):
            with mit.MotorBus("can0", ["rs-06"] * 3 + ["rs-00"] * 4, motor_count=7) as bus:
                with self.assertRaises(TimeoutError):
                    bus.feedback_positions(.08)
                for motor in range(1, 8):
                    position = .1 * motor
                    code = int((position + 4 * math.pi) / (8 * math.pi) * 65535)
                    bus.inspect(reply(2, motor, struct.pack(">HHHH", code, 32767, 32767, 250), mode=2))
                np.testing.assert_allclose(bus.feedback_positions(.08), np.arange(1, 8) * .1, atol=.0004)
                self.assertFalse(virtual.sent)
                bus.feedback[7]["received_at"] -= .2
                with self.assertRaisesRegex(TimeoutError, "7.*stale"):
                    bus.feedback_positions(.08)

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
