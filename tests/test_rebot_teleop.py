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
from teleop_process_fakes import (BusFactory, SourceFactory, MeasuredMotors,
                                  signal_while_stop_lock_is_held, wait_for_signal)
from rebot_teleop_runtime import SharedFrame, StopSignal, fresh, latest, LoopTiming


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

    def test_signal_request_defers_ipc_until_normal_control_flow(self):
        event = MagicMock()
        stop = StopSignal(event)
        stop.request()
        event.set.assert_not_called()
        stop.is_set()
        event.set.assert_called_once()

    def test_wait_preserves_timeout_and_normal_stop_behavior(self):
        stop = StopSignal(self.context.Event())
        self.assertFalse(stop.wait(.002))
        self.assertFalse(stop.wait(0.))
        stop.set()
        self.assertTrue(stop.wait(0.))
        self.assertTrue(stop.wait())


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

    def test_handle_openness_maps_to_configured_travel(self):
        reference = np.zeros(7)
        low, high = np.zeros(7), np.ones(7) * math.radians(180)
        for handle, motor in ((0, 0), (.5, 0), (1., 0), (23., 90), (45, 180), (90, 180), (-10, 0)):
            with self.subTest(handle=handle):
                raw = reference.copy()
                raw[6] = handle
                self.assertAlmostEqual(teleop.map_target(raw, reference, low, high)[6], math.radians(motor))

    def test_closing_tolerance_is_continuous_and_does_not_change_arm_mapping(self):
        reference = np.array([10.] * 7)
        low, high = np.array([-10.] * 6 + [0.]), np.array([10.] * 6 + [math.radians(270)])
        raw = reference + [10., 20., 30., 40., 50., 60., .9]
        legacy = teleop.map_target(raw, reference, low, high, 0.)
        for handle in (0., .5, 1., 1.000001, 1.1, 23., 45.):
            raw[6] = reference[6] + handle
            target = teleop.map_target(raw, reference, low, high)
            np.testing.assert_array_equal(target[:6], legacy[:6])
            self.assertAlmostEqual(math.degrees(target[6]), max(0., (handle - 1.) / 44.) * 270.)
        raw[6] = reference[6] + 22.5
        self.assertAlmostEqual(math.degrees(teleop.map_target(raw, reference, low, high, 0.)[6]), 135.)
        for invalid in (-1., 45., math.nan, math.inf, True):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                teleop.map_target(raw, reference, low, high, invalid)


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

    def run_teleop(self, failure="", stall=False, side="both", gripper=True, signal_stop=False, grasp="",
                   gripper_open_deg=270., wait_for_timeout=False, gripper_torque_nm=None):
        sides = ("left", "right") if side == "both" else (side,)
        channels = {name: "can0" if name == "left" else "can1" for name in sides}
        keys = MagicMock()
        began = time.monotonic()

        def pressed():
            recoverable = failure in ('', 'transient_command', 'leader_transient', 'slow_motor')
            minimum = 320 if grasp else 100 if stall or failure else 12
            done = (recoverable and sum(self.counts) >= minimum) or time.monotonic() - began > 8
            if done and signal_stop:
                os.kill(os.getpid(), signal.SIGTERM)
                return False
            return done

        keys.__enter__.return_value.pressed.side_effect = pressed
        try:
            with patch.object(teleop, "load_bindings", return_value=(sides, self.arms, self.leaders, channels)), \
                    patch.object(teleop, "StopKey", return_value=keys), \
                    redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
                return teleop.teleop(side=side, gripper=gripper, gripper_open_deg=gripper_open_deg,
                    gripper_torque_nm=gripper_torque_nm,
                    startup_timeout=8, shutdown_timeout=3,
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

    def test_running_large_error_stops_following_and_holds_both_arms(self):
        with self.assertRaisesRegex(RuntimeError, "Tracking error limit exceeded.*J1"):
            self.run_teleop(failure="tracking_jump_later")
        for record in self.assert_holding().values():
            if record["parameters"] is not None:
                self.assertLess(abs(record["parameters"][0]["position"] - .2), math.radians(45))

    def test_single_arm_without_gripper_owns_only_selected_hardware(self):
        self.run_teleop(side="right", gripper=False)
        motors = self.assert_holding(("can1",), 6)
        self.assertEqual(len(self.snapshots), 2)
        self.assertEqual(motors["can1"]["modes"][7], 0)
        self.assertEqual(motors["can1"]["position_queries"], 18)

    def test_sequential_enable_timeouts_refresh_feedback_before_gripper_control(self):
        messages = self.run_teleop(wait_for_timeout=True)
        for record in self.assert_holding().values():
            np.testing.assert_allclose(record["first"], [.2] * 7)
            np.testing.assert_allclose(record["second"],
                                       .2 + np.radians([20, 20, 0, -20, -20, 20, 19. / 44. * 270.]))
            self.assertEqual(record["position_queries"], 21)
        for record in messages:
            if record["type"] == "timing" and record["worker"].startswith("motor-"):
                self.assertGreater(record["count"], 0)

    def test_missing_startup_refresh_reply_stops_and_holds_both_arms(self):
        with self.assertRaisesRegex(RuntimeError, "missing native motion feedback"):
            self.run_teleop(failure="startup_feedback")
        self.assert_holding()

    def test_shortened_travel_keeps_zero_native_gripper_gains_and_limited_total_torque(self):
        self.run_teleop(side="right", gripper_open_deg=180.)
        record = self.assert_holding(("can1",))["can1"]
        command = record["parameters"][-1]
        self.assertAlmostEqual(record["first"][-1], .2)
        self.assertAlmostEqual(record["second"][-1], .2 + math.radians(19. / 44. * 180.))
        self.assertEqual(command["kp"], 0.)
        self.assertEqual(command["kd"], 0.)
        self.assertTrue(all(abs(tau) <= 1. for tau in record["gripper_torques"]))

    def test_blocked_gripper_keeps_closing_target_and_limited_total_torque(self):
        self.run_teleop(grasp="hold")
        for record in self.assert_holding().values():
            self.assertAlmostEqual(record["parameters"][-1]["position"], .2, delta=.001)
            self.assertTrue(all(-1. <= tau < -.99 for tau in record["gripper_torques"]))
            self.assertAlmostEqual(record["hold"][-1]["position"], 1.2, delta=.001)
            self.assertEqual(record["position_queries"], 21)

    def test_opening_after_a_stall_reverses_limited_torque_without_changing_control_modes(self):
        self.run_teleop(grasp="open")
        for record in self.assert_holding().values():
            self.assertLess(min(record["gripper_torques"]), -.99)
            self.assertGreater(max(record["gripper_torques"]), .99)
            self.assertTrue(all(abs(tau) <= 1. for tau in record["gripper_torques"]))
            self.assertGreater(max(record["gripper_commands"]), 4.)
            self.assertAlmostEqual(record["hold"][-1]["position"], 1.2, delta=.001)

    def test_invalid_gripper_effort_stops_and_holds_both_arms(self):
        with self.assertRaisesRegex(RuntimeError, "MIT feedback must be finite"):
            self.run_teleop(failure="gripper_feedback")
        self.assert_holding()

    def test_custom_total_torque_limit_applies_to_closing_and_opening_on_the_wire(self):
        self.run_teleop(grasp="open", gripper_torque_nm=.3)
        for record in self.assert_holding().values():
            self.assertLess(min(record["gripper_torques"]), -.29)
            self.assertGreater(max(record["gripper_torques"]), .29)
            self.assertTrue(all(abs(tau) <= .3 for tau in record["gripper_torques"]))

    def test_small_error_outside_preload_window_has_no_fixed_friction_feedforward(self):
        self.run_teleop(side="right", grasp="small_close")
        record = self.assert_holding(("can1",))["can1"]
        self.assertAlmostEqual(record["parameters"][-1]["position"], 1.19, delta=.001)
        self.assertTrue(record["gripper_torques"])
        self.assertTrue(all(-.125 < tau < -.115 for tau in record["gripper_torques"]))
        self.assertLess(max(record["gripper_torques"]) - min(record["gripper_torques"]), .001)

    def test_closed_handle_margin_produces_preload_at_closed_follower_pose(self):
        self.run_teleop(side="right", grasp="closed_margin")
        record = self.assert_holding(("can1",))["can1"]
        self.assertAlmostEqual(record["parameters"][-1]["position"], .2, delta=.001)
        self.assertEqual(record["parameters"][-1]["kp"], 0.)
        self.assertEqual(record["parameters"][-1]["kd"], 0.)
        self.assertTrue(record["gripper_torques"])
        self.assertTrue(all(-.215 < tau < -.20 for tau in record["gripper_torques"]))
        self.assertAlmostEqual(record["hold"][-1]["position"], .2, delta=.001)


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

    def test_motor_scheduling_gap_is_reported_without_aborting(self):
        messages = self.run_teleop(failure='slow_motor')
        self.assert_holding()
        timing = next(r for r in messages if r['type'] == 'timing' and r['worker'] == 'motor-right')
        self.assertGreater(timing['max_gap_ms'], 100.)

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
        with patch.object(teleop.sys, "argv", ["rebot_teleop.py", "--gripper-torque-nm", "0.3"]), \
                patch.object(teleop, "teleop") as run:
            self.assertEqual(teleop.main(), 0)
            self.assertEqual(run.call_args.kwargs["gripper_torque_nm"], .3)
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
    def test_invalid_gripper_torque_is_rejected_before_spawning_workers(self):
        arms = {"right": {"models": ["rs-06"] * 3 + ["rs-00"] * 4,
                          "reference_rad": [.2] * 7}}
        with patch.object(teleop, "load_bindings", return_value=(("right",), arms, {}, {})), \
                patch.object(teleop.mp, "get_context") as spawn:
            for torque in (0., -1., .00001, math.nan, math.inf, 15.):
                with self.subTest(torque=torque), self.assertRaises(ValueError):
                    teleop.teleop(side="right", gripper_torque_nm=torque)
            spawn.assert_not_called()

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

    def test_gripper_temperature_is_recorded_and_faults_are_checked(self):
        with patch.object(mit, "check_channel"), patch.object(mit.can, "Bus", return_value=VirtualMotors()):
            with mit.MotorBus("can0", ["rs-06"] * 3 + ["rs-00"] * 4, motor_count=7) as bus:
                bus.inspect(reply(2, 7, struct.pack(">HHHH", 32767, 32767, 32767, 700)))
                self.assertEqual(bus.feedback[7]["temperature"], 70.)
                with self.assertRaisesRegex(RuntimeError, "fault"):
                    bus.inspect(reply(21, 7, b"\x01" + bytes(7)))


if __name__ == "__main__":
    unittest.main()
