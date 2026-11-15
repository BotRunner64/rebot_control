"""Offline leader mapping and virtual-CAN physical teleoperation lifecycle tests."""

from contextlib import redirect_stderr, redirect_stdout
import io
import math
import multiprocessing as mp
import os
from pathlib import Path
import queue
import time
import signal
import struct
import tempfile
import unittest
from unittest.mock import MagicMock, patch

import numpy as np

import yaml

from fakes import reply
from rebot.apps import home as rebot_home
from rebot.apps import teleop
from rebot.bindings import ArmBinding
from rebot.devices import robstride as mit
from rebot.devices import socketcan
from rebot.devices.robstride import MODELS
from rebot.model.leader_map import map_target
from rebot.runtime.arm import follower_range
from rebot.runtime.console import StopSignal
from rebot.runtime.ipc import SharedFrame, fresh, latest
from rebot.runtime.timing import LoopTiming
from teleop_fakes import (BusFactory, SourceFactory, MeasuredMotors, virtual_bindings,
                          signal_while_stop_lock_is_held, wait_for_signal)


def calibrated_range(count, gripper_open_deg):
    """Travel limits of a follower whose seven references are 0.2 rad."""
    return follower_range(ArmBinding("right", "can1", MODELS, {}, np.full(7, .2)),
                          count, gripper_open_deg)


class SignalStopTests(unittest.TestCase):
    def setUp(self):
        self.context = mp.get_context("spawn")

    def run_worker(self, worker, send_signal=False):
        stop = StopSignal(self.context.Event())
        reader, writer = self.context.Pipe(duplex=False)
        process = self.context.Process(target=worker, args=(stop, writer))
        process.start()
        writer.close()
        try:
            if send_signal:
                self.assertTrue(reader.poll(3.))
                self.assertEqual(reader.recv(), "ready")
                os.kill(process.pid, signal.SIGTERM)
            self.assertTrue(reader.poll(2.), "Signal handler deadlocked on the shared stop lock.")
            self.assertTrue(reader.recv())
            process.join(2.)
            self.assertFalse(process.is_alive())
            self.assertEqual(process.exitcode, 0)
            self.assertTrue(stop.is_set())
        finally:
            if process.is_alive():
                process.kill()
            process.join()
            process.close()
            reader.close()

    def test_repeated_signals_inside_stop_event_lock_exit_and_notify_peers(self):
        self.run_worker(signal_while_stop_lock_is_held)

    def test_signal_interrupts_long_wait_without_waiting_for_its_timeout(self):
        self.run_worker(wait_for_signal, send_signal=True)


class MappingTests(unittest.TestCase):
    def test_calibrated_signs_gripper_ratio_and_no_angle_wrapping(self):
        reference = np.array([10.] * 7)
        low, high = np.array([-10.] * 6 + [0.]), np.array([10.] * 6 + [math.radians(270)])
        result = map_target(reference + [190, 20, 30, 40, 50, 60, 45], reference, low, high)
        np.testing.assert_allclose(result, np.radians([190, 20, -30, -40, -50, 60, 270]))
        clipped = map_target(reference + [800.] * 7, reference, low, high)
        np.testing.assert_allclose(clipped, [10, 10, -10, -10, -10, 10, high[6]])
        closed = map_target(reference - np.array([0.] * 6 + [10.]), reference, low, high)
        self.assertEqual(closed[6], 0)
        with self.assertRaises(ValueError):
            map_target([float("nan")] * 7, reference, low, high)

    def test_closing_tolerance_is_continuous_and_does_not_change_arm_mapping(self):
        reference = np.array([10.] * 7)
        low, high = np.array([-10.] * 6 + [0.]), np.array([10.] * 6 + [math.radians(270)])
        raw = reference + [10., 20., 30., 40., 50., 60., .9]
        legacy = map_target(raw, reference, low, high, 0.)
        for handle in (0., .5, 1., 1.000001, 1.1, 23., 45.):
            raw[6] = reference[6] + handle
            target = map_target(raw, reference, low, high)
            np.testing.assert_array_equal(target[:6], legacy[:6])
            self.assertAlmostEqual(math.degrees(target[6]), max(0., (handle - 1.) / 44.) * 270.)
        raw[6] = reference[6] + 22.5
        self.assertAlmostEqual(math.degrees(map_target(raw, reference, low, high, 0.)[6]), 135.)
        for invalid in (-1., 45., math.nan, math.inf, True):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                map_target(raw, reference, low, high, invalid)


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
        cached, acquired = latest(frame, "Source")
        np.testing.assert_array_equal(cached, [3, 4])
        self.assertEqual(acquired, source_at)

    def test_unpublished_invalid_and_abandoned_locked_frames_are_bounded(self):
        frame = SharedFrame(self.context, 2)
        with self.assertRaises(TimeoutError):
            fresh(frame, time.monotonic(), .1, "Missing")
        with self.assertRaisesRegex(TimeoutError, "missing"):
            latest(frame, "Missing")
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
        limits = calibrated_range(7, 270)
        reference, low, high = limits.reference, limits.lower, limits.upper
        np.testing.assert_allclose(np.degrees(low), [-145, 0, 0, -80, -90, -130, 0])
        np.testing.assert_allclose(np.degrees(high), [145, 170, 200, 90, 90, 130, 270])
        q = reference.copy()
        q[2] += math.radians(190)
        limits.validate(q)
        q[5] += math.radians(140)
        with self.assertRaisesRegex(ValueError, r"J6: relative=140\.00 deg, limits=\[-130\.00, 130\.00\] deg"):
            limits.validate(q)

    def test_arm_feedback_accepts_logged_pose_and_checks_wider_margin_on_both_bounds(self):
        for count in (6, 7):
            limits = calibrated_range(count, 270)
            reference, low, high = limits.reference, limits.lower, limits.upper
            q = reference.copy()
            q[3] += math.radians(-81.73)
            np.testing.assert_array_equal(limits.validate(q), q)
            for joint in range(6):
                for bound, direction in ((low[joint], -1), (high[joint], 1)):
                    with self.subTest(count=count, joint=joint, direction=direction):
                        q = reference.copy()
                        q[joint] += bound + direction * .099
                        limits.validate(q)
                        q[joint] = reference[joint] + bound + direction * .101
                        with self.assertRaisesRegex(ValueError, rf"J{joint + 1}:.*tolerance=5\.73 deg"):
                            limits.validate(q)

    def test_gripper_feedback_ignores_travel_limits_but_requires_finite_complete_feedback(self):
        limits = calibrated_range(7, 180)
        reference = limits.reference
        for relative in (-20., 200., 280.):
            with self.subTest(relative=relative):
                q = reference.copy()
                q[6] += math.radians(relative)
                np.testing.assert_array_equal(limits.validate(q), q)
        for invalid in (math.nan, math.inf, -math.inf):
            q[6] = invalid
            with self.subTest(invalid=invalid), self.assertRaisesRegex(ValueError, "finite feedback"):
                limits.validate(q)
        with self.assertRaisesRegex(ValueError, "finite feedback"):
            limits.validate(reference[:6])

    def test_timing_skips_missed_ticks_instead_of_bursting_commands(self):
        stop = MagicMock()
        with patch("rebot.runtime.timing.time.monotonic", return_value=1.):
            timing = LoopTiming("motor", 200)
        with patch("rebot.runtime.timing.time.monotonic", return_value=1.022):
            timing.pace(stop)
        self.assertEqual(timing.skipped, 4)
        self.assertAlmostEqual(stop.wait.call_args.args[0], .003)


class ProcessLifecycleTests(unittest.TestCase):
    def setUp(self):
        self.context = mp.get_context("spawn")
        self.counts = self.context.Array("i", 2)
        self.hardware_reports = self.context.Queue()
        self.snapshots = []
        self.addCleanup(self.hardware_reports.join_thread)
        self.addCleanup(self.hardware_reports.close)

    def run_teleop(self, failure="", stall=False, side="both", gripper=True, signal_stop=False, grasp="",
                   gripper_open_deg=270., wait_for_timeout=False, gripper_torque_nm=None):
        arms, leaders = virtual_bindings(side)
        keys = MagicMock()
        began = time.monotonic()

        def pressed():
            recoverable = failure in ('', 'transient_command', 'leader_transient')
            minimum = 320 if grasp else 100 if stall or failure else 12
            done = (recoverable and sum(self.counts) >= minimum) or time.monotonic() - began > 8
            if done and signal_stop:
                os.kill(os.getpid(), signal.SIGTERM)
                return False
            return done

        keys.__enter__.return_value.pressed.side_effect = pressed
        try:
            with patch.object(teleop, "select_arms", return_value=arms), \
                    patch.object(teleop, "select_leaders", return_value=leaders), \
                    patch.object(teleop, "StopKey", return_value=keys), \
                    redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
                plan = teleop.make_plan(side=side, gripper=gripper, gripper_open_deg=gripper_open_deg,
                                        gripper_torque_nm=gripper_torque_nm)
                return teleop.teleop(plan, startup_timeout=8, shutdown_timeout=3,
                    source_factory=SourceFactory(self.hardware_reports, stall, grasp, self.counts, failure),
                    bus_factory=BusFactory(self.hardware_reports, self.counts, failure, grasp, wait_for_timeout))
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
            self.assertTrue(record["arm_tau_zero"])
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
            np.testing.assert_allclose(record["first"], [.2] * 7)
            np.testing.assert_allclose(record["second"],
                                       .2 + np.radians([20, 20, 0, -20, -20, 20, 19. / 44. * 270.]))
            np.testing.assert_allclose([p["kp"] for p in record["parameters"]],
                                       [50, 150, 150, 50, 50, 50, 0], atol=.08)
            np.testing.assert_allclose([p["kd"] for p in record["parameters"]],
                                       [3, 10, 10, 5, 4, 4, 0], atol=.002)
            self.assertTrue(record["gripper_torques"])
            self.assertTrue(all(abs(tau) <= 1. for tau in record["gripper_torques"]))
        rates = {r["worker"]: r for r in messages if r["type"] == "timing"}
        self.assertEqual(rates["target"]["requested_hz"], 500)
        self.assertEqual(rates["motor-left"]["requested_hz"], 200)
        self.assertEqual(rates["motor-right"]["requested_hz"], 200)
        self.assertGreater(rates["target"]["count"], 0)

    def test_initial_large_error_stops_both_arms_without_sending_leader_target(self):
        with self.assertRaisesRegex(RuntimeError, "Tracking error limit exceeded.*J1"):
            self.run_teleop(failure="tracking_jump")
        for record in self.assert_holding().values():
            self.assertLessEqual(record["calls"], 1)
            self.assertIsNone(record["second"])

    def test_single_arm_without_gripper_owns_only_selected_hardware(self):
        self.run_teleop(side="right", gripper=False)
        motors = self.assert_holding(("can1",), 6)
        self.assertEqual(len(self.snapshots), 2)
        self.assertEqual(motors["can1"]["modes"][7], 0)
        self.assertEqual(motors["can1"]["position_queries"], 18)

    def test_blocked_gripper_keeps_closing_target_and_limited_total_torque(self):
        self.run_teleop(grasp="hold")
        for record in self.assert_holding().values():
            self.assertAlmostEqual(record["parameters"][-1]["position"], .2, delta=.001)
            self.assertTrue(all(-1. <= tau < -.99 for tau in record["gripper_torques"]))
            self.assertAlmostEqual(record["hold"][-1]["position"], 1.2, delta=.001)
            self.assertEqual(record["position_queries"], 21)

    def test_custom_total_torque_limit_applies_to_closing_and_opening_on_the_wire(self):
        self.run_teleop(grasp="open", gripper_torque_nm=.3)
        for record in self.assert_holding().values():
            self.assertLess(min(record["gripper_torques"]), -.29)
            self.assertGreater(max(record["gripper_torques"]), .29)
            self.assertTrue(all(abs(tau) <= .3 for tau in record["gripper_torques"]))

    def test_identity_or_mode_mismatch_on_one_side_prevents_enabling_either_arm(self):
        for failure, message in (("uid", "UIDs"), ("prepare", "MIT mode")):
            with self.subTest(failure=failure):
                self.snapshots.clear()
                with self.assertRaisesRegex(RuntimeError, message):
                    self.run_teleop(failure=failure)
                motors = [r for r in self.snapshots if r["kind"] == "motor"]
                self.assertTrue(motors)
                self.assertTrue(all(not r["enabled"] for r in motors))
                self.assertTrue(all(3 not in r["types"] for r in motors))

    def test_lost_or_invalid_feedback_stops_and_holds_both_arms(self):
        for failure, message in (("command", "missing native motion feedback"),
                                 ("gripper_feedback", "MIT feedback must be finite")):
            with self.subTest(failure=failure):
                self.snapshots.clear()
                with self.assertRaisesRegex(RuntimeError, message):
                    self.run_teleop(failure=failure)
                self.assert_holding()

    def test_partial_enable_failure_holds_touched_motors(self):
        with self.assertRaisesRegex(RuntimeError, "mode 2"):
            self.run_teleop(failure="enable")
        records = {r["channel"]: r for r in self.snapshots if r["kind"] == "motor"}
        self.assertTrue(all(r["closed"] for r in records.values()))
        self.assertTrue(all(4 not in r["types"] for r in records.values()))
        self.assertEqual(records["can1"]["enabled"], [1, 2, 3])

    def test_slow_source_keeps_cached_targets_without_expiring_or_hiding_their_age(self):
        messages = self.run_teleop(stall=True)
        self.assert_holding()
        timings = [r for r in messages if r['type'] == 'timing']
        self.assertGreater(max(r['max_source_age_ms'] for r in timings), 100.)

    def test_transient_can_loss_recovers_on_the_fifteenth_attempt(self):
        self.run_teleop(failure='transient_command', wait_for_timeout=True)
        self.assert_holding()

    def test_transient_leader_loss_keeps_both_followers_running_and_recovers(self):
        self.run_teleop(failure='leader_transient')
        self.assert_holding()
        source = next(r for r in self.snapshots if r['kind'] == 'source' and r['port'] == 'fake-right')
        self.assertEqual(source['failures'], 14)
        self.assertGreater(source['reads'], source['failures'] + 1)

    def test_persistent_leader_loss_stops_after_fifteen_attempts(self):
        with self.assertRaisesRegex(RuntimeError, 'angle feedback failed after 15 attempts'):
            self.run_teleop(failure='leader_feedback')
        self.assert_holding()
        source = next(r for r in self.snapshots if r['kind'] == 'source' and r['port'] == 'fake-right')
        self.assertEqual(source['failures'], 15)

    def test_abrupt_motor_process_death_is_detected_and_peer_holds(self):
        with self.assertRaisesRegex(RuntimeError, "exited unexpectedly"):
            self.run_teleop(failure="crash")
        self.assert_holding(("can0",))

    def test_parent_sigterm_performs_cooperative_hold_cleanup(self):
        self.run_teleop(signal_stop=True)
        self.assert_holding()

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
    def test_invalid_gripper_torque_and_travel_are_rejected_before_spawning_workers(self):
        arms, leaders = virtual_bindings("right")
        with patch.object(teleop, "select_arms", return_value=arms), \
                patch.object(teleop, "select_leaders", return_value=leaders), \
                patch.object(teleop.mp, "get_context") as spawn:
            for torque in (0., -1., .00001, math.nan, math.inf, 15.):
                with self.subTest(torque=torque), self.assertRaises(ValueError):
                    teleop.teleop(teleop.make_plan(side="right", gripper_torque_nm=torque))
            self.assertEqual(teleop.make_plan(side="right", gripper_torque_nm=.3).gripper.torque_limit_nm, .3)
            with self.assertRaisesRegex(ValueError, "270-degree"):
                teleop.make_plan(side="right", gripper_open_deg=300)
            spawn.assert_not_called()

    def test_invalid_bindings_are_rejected_before_workers_start(self):
        arms = {name: {"channel": channel, "device_path": "", "bitrate": socketcan.BITRATE,
                       "host_id": mit.HOST_ID, "motor_ids": list(mit.MOTOR_IDS), "models": list(MODELS),
                       "motor_uids": MeasuredMotors(offset).uids, "reference_rad": [0.] * 7}
                for name, channel, offset in (("left", "can0", 0), ("right", "can1", 100))}
        leaders = {name: {"port": f"fake-{name}", "baudrate": 1000000, "reference_deg": [0.] * 7}
                   for name in arms}
        with tempfile.TemporaryDirectory() as directory, \
                patch.object(teleop.mp, "get_context") as spawn:
            config, leader_config = Path(directory) / "arm.yaml", Path(directory) / "leader.yaml"

            def plan():
                config.write_text(yaml.safe_dump(arms))
                leader_config.write_text(yaml.safe_dump(leaders))
                return teleop.make_plan(config, leader_config)

            self.assertEqual(plan().sides, ("left", "right"))
            leaders["right"]["reference_deg"][0] = math.nan
            with self.assertRaises(ValueError):
                plan()
            leaders["right"]["reference_deg"][0] = 0
            leaders["right"]["port"] = leaders["left"]["port"]
            with self.assertRaisesRegex(ValueError, "distinct"):
                plan()
            leaders["right"]["port"] = "fake-right"
            arms["right"]["reference_rad"][0] = math.nan
            with self.assertRaises(ValueError):
                plan()
            arms["right"]["reference_rad"][0] = 0.
            arms["right"]["channel"] = arms["left"]["channel"]
            with self.assertRaisesRegex(ValueError, "distinct"):
                plan()
            arms["right"]["channel"] = "can1"
            arms["right"]["motor_uids"] = arms["left"]["motor_uids"]
            with self.assertRaisesRegex(ValueError, "distinct"):
                plan()
            spawn.assert_not_called()

    def test_home_disable_stops_all_seven_without_changing_parameters(self):
        virtual = MeasuredMotors()
        virtual.modes = dict.fromkeys(range(1, 8), 2)
        for motor in range(1, 8):
            virtual.params[(motor, mit.MODE)] = 1
        parameters = dict(virtual.params)
        arms = {"right": ArmBinding("right", "can0", MODELS, dict(virtual.uids), None)}
        with patch.object(mit, "check_channel"), patch.object(mit.can, "Bus", return_value=virtual), \
                patch.object(rebot_home, "select_arms", return_value=arms) as select, \
                redirect_stdout(io.StringIO()):
            self.assertTrue(rebot_home.disable(side="right"))
        # Disabling must not require a software reference.
        self.assertIs(select.call_args.kwargs["calibrated"], False)
        self.assertEqual(set(virtual.modes.values()), {0})
        self.assertEqual(virtual.params, parameters)
        self.assertEqual({(m.arbitration_id >> 24) & 31 for m in virtual.sent}, {0, 4})


class SevenMotorTransportTests(unittest.TestCase):
    def test_feedback_retries_only_missing_motors_and_recovers_on_attempt_fifteen(self):
        virtual = MeasuredMotors()
        virtual.modes = dict.fromkeys(range(1, 8), 2)
        virtual.drop_remaining[7] = 14
        with patch.object(mit, "check_channel"), patch.object(mit.can, "Bus", return_value=virtual), \
                redirect_stderr(io.StringIO()):
            with mit.MotorBus("can0", ["rs-06"] * 3 + ["rs-00"] * 4, motor_count=7) as bus:
                bus.command_position([.2] * 7, [0.] * 7, [0.] * 7, [50.] * 7, [4.] * 7,
                                     None, max_attempts=15, reply_timeout_s=.01)
                sent = [msg.arbitration_id & 255 for msg in virtual.sent]
                self.assertEqual(sent, list(range(1, 8)) + [7] * 14)
                self.assertEqual(len(bus.feedback_positions(math.inf)), 7)

    def test_persistent_feedback_loss_exhausts_exactly_fifteen_attempts(self):
        virtual = MeasuredMotors()
        virtual.modes = dict.fromkeys(range(1, 8), 2)
        virtual.drop_holds.add(7)
        with patch.object(mit, "check_channel"), patch.object(mit.can, "Bus", return_value=virtual), \
                redirect_stderr(io.StringIO()):
            with mit.MotorBus("can0", ["rs-06"] * 3 + ["rs-00"] * 4, motor_count=7) as bus:
                with self.assertRaisesRegex(TimeoutError, r"missing native motion feedback.*\[7\]"):
                    bus.command_position([.2] * 7, [0.] * 7, [0.] * 7, [50.] * 7, [4.] * 7,
                                         None, max_attempts=15, reply_timeout_s=.01)
                self.assertEqual(sum(msg.arbitration_id & 255 == 7 for msg in virtual.sent), 15)

    def test_stop_interrupts_feedback_retries_before_another_batch(self):
        virtual = MeasuredMotors()
        virtual.modes = dict.fromkeys(range(1, 8), 2)
        virtual.drop_holds.add(7)
        stop = mp.get_context('spawn').Event()
        with patch.object(mit, "check_channel"), patch.object(mit.can, "Bus", return_value=virtual), \
                patch.object(stop, 'wait', side_effect=lambda timeout: stop.set() or True), \
                redirect_stderr(io.StringIO()):
            with mit.MotorBus("can0", ["rs-06"] * 3 + ["rs-00"] * 4, motor_count=7) as bus:
                result = bus.command_position([.2] * 7, [0.] * 7, [0.] * 7, [50.] * 7, [4.] * 7,
                                              None, max_attempts=15, reply_timeout_s=.01, stop=stop)
                self.assertIsNone(result)
                self.assertEqual(len(virtual.sent), 7)

    def test_can_send_error_recovers_without_retrying_valid_axes(self):
        virtual = MeasuredMotors()
        virtual.modes = dict.fromkeys(range(1, 8), 2)
        send = virtual.send
        failures = []

        def flaky_send(msg, timeout):
            if msg.arbitration_id & 255 == 7 and not failures:
                failures.append(True)
                raise mit.can.CanError('Transient CAN send failure')
            send(msg, timeout)

        with patch.object(mit, "check_channel"), patch.object(mit.can, "Bus", return_value=virtual), \
                patch.object(virtual, 'send', side_effect=flaky_send), redirect_stderr(io.StringIO()):
            with mit.MotorBus("can0", ["rs-06"] * 3 + ["rs-00"] * 4, motor_count=7) as bus:
                bus.command_position([.2] * 7, [0.] * 7, [0.] * 7, [50.] * 7, [4.] * 7,
                                     None, max_attempts=15, reply_timeout_s=.01)
                self.assertEqual([msg.arbitration_id & 255 for msg in virtual.sent], list(range(1, 8)))

    def test_motor_fault_is_not_hidden_by_feedback_retries(self):
        virtual = MeasuredMotors()
        recv = virtual.recv

        def faulty_recv(timeout):
            return reply(21, 7, b'\x01' + bytes(7)) if timeout else recv(timeout)

        with patch.object(mit, "check_channel"), patch.object(mit.can, "Bus", return_value=virtual), \
                patch.object(virtual, 'recv', side_effect=faulty_recv):
            with mit.MotorBus("can0", ["rs-06"] * 3 + ["rs-00"] * 4, motor_count=7) as bus:
                with self.assertRaisesRegex(RuntimeError, 'fault/warning'):
                    bus.command_position([.2] * 7, [0.] * 7, [0.] * 7, [50.] * 7, [4.] * 7,
                                         None, max_attempts=15, reply_timeout_s=.01)
                self.assertEqual(len(virtual.sent), 7)

    def test_invalid_limited_batch_sends_no_partial_motor_commands(self):
        virtual = MeasuredMotors()
        with patch.object(mit, "check_channel"), patch.object(mit.can, "Bus", return_value=virtual):
            with mit.MotorBus("can0", ["rs-06"] * 3 + ["rs-00"] * 4, motor_count=7) as bus:
                for limits in ({7: 1.}, {8: 1.}):
                    with self.subTest(limits=limits), self.assertRaises(ValueError):
                        bus.command_position([.2] * 7, [0.] * 7, [0.] * 7,
                                             [12.] * 7, [.5] * 7, time.monotonic() + .08,
                                             torque_limits=limits)
                self.assertFalse(virtual.sent)

    def test_feedback_snapshots_include_effort_without_queries_and_are_independent(self):
        virtual = MeasuredMotors()
        with patch.object(mit, "check_channel"), patch.object(mit.can, "Bus", return_value=virtual):
            with mit.MotorBus("can0", ["rs-06"] * 3 + ["rs-00"] * 4, motor_count=7) as bus:
                for motor in range(1, 8):
                    bus.inspect(reply(2, motor, struct.pack(">HHHH", 32767, 32767, 31000, 250), mode=2))
                states = bus.feedback_states(.08)
                self.assertLess(states[6]["torque"], 0.)
                self.assertIn("velocity", states[6])
                states[6]["torque"] = math.nan
                self.assertTrue(math.isfinite(bus.feedback_states(.08)[6]["torque"]))
                bus.feedback[7]["received_at"] -= .2
                with self.assertRaisesRegex(TimeoutError, "7.*stale"):
                    bus.feedback_states(.08)
                self.assertFalse(virtual.sent)

    def test_exit_position_query_failure_uses_last_validated_hold_pose(self):
        virtual = MeasuredMotors()
        with patch.object(mit, "check_channel"), patch.object(mit.can, "Bus", return_value=virtual), \
                redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
            with mit.MotorBus("can0", ["rs-06"] * 3 + ["rs-00"] * 4, motor_count=7) as bus:
                bus.set_hold([.2] * 7, [0.] * 7, [50.] * 7, [4.] * 7)
                bus.enable([.2] * 7)
                with patch.object(bus, "positions", side_effect=TimeoutError("Missing exit position")):
                    bus.freeze()
                frames = [msg for msg in virtual.sent if (msg.arbitration_id >> 24) & 31 == 1]
                holds = {msg.arbitration_id & 255: mit.decode_mit(msg, "rs-00") for msg in frames[-7:]}
                self.assertAlmostEqual(holds[7]["position"], .2, delta=.001)
                self.assertAlmostEqual(holds[1]["position"], .2, delta=.001)


if __name__ == "__main__":
    unittest.main()
