"""Real Mink IK and virtual-CAN EE control checks; never open a physical CAN socket."""

from contextlib import redirect_stderr, redirect_stdout
import importlib.util
import io
import math
from pathlib import Path
import signal
import struct
import threading
import time
import unittest
from unittest.mock import patch

import numpy as np
from scipy.spatial.transform import Rotation

import rebot_ee as ee
import rebot_mit as mit
from rebot_kinematics import CartesianIK, SIM_START, pose_matrix
from rebot_teleop import JOINT_LIMITS_DEG
from test_rebot_hand_guiding import VirtualMotors, reply


EE_AVAILABLE = all(importlib.util.find_spec(name) is not None for name in ("mink", "quadprog"))
TEST_START = np.radians([0., 60., 90., 0., 20., 0.])


@unittest.skipUnless(EE_AVAILABLE, "Install the ee dependency extra to test Mink control")
class KinematicsTests(unittest.TestCase):
    def test_measured_seed_just_outside_limit_is_accepted_without_a_safety_break(self):
        ik = CartesianIK(SIM_START)
        measured = SIM_START.copy()
        measured[1] = -.0001
        target = ik.pose(measured)
        ik.solve(target, init_q=measured)
        self.assertTrue(ik.converged)
        self.assertLess(ik.error(target)[0], 1e-4)

    def test_target_collision_check_uses_contacts_and_preserves_render_state(self):
        ik = CartesianIK(SIM_START)
        displayed = ik.data.qpos.copy()
        self.assertFalse(ik.has_self_collision(SIM_START))
        self.assertFalse(ik.has_self_collision(TEST_START))
        colliding = np.radians([-66.403, 146.983, 13.731, 58.845, 68.73, -19.886])
        self.assertTrue(ik.has_self_collision(colliding))
        np.testing.assert_array_equal(ik.data.qpos, displayed)

    def test_default_simulation_starts_at_model_reference(self):
        np.testing.assert_array_equal(SIM_START, np.zeros(6))
        ik = CartesianIK(SIM_START)
        np.testing.assert_array_equal(ik.q, np.zeros(6))
        target = ik.pose()
        ik.solve(target)
        np.testing.assert_allclose(ik.q, np.zeros(6), atol=1e-9)

    def test_full_pose_is_solved_in_one_call_without_real_time_speed_limiting(self):
        ik = CartesianIK(TEST_START)
        goal = TEST_START + np.radians([3., 2., -3., 2., 2., -4.])
        target = ik.pose(goal)
        previous = ik.q
        q = ik.solve(target)
        self.assertGreater(float(np.max(np.abs(q - previous))), .03)
        self.assertTrue(np.all(q >= ik.lower - 1e-8))
        self.assertTrue(np.all(q <= ik.upper + 1e-8))
        self.assertTrue(ik.converged)
        self.assertLessEqual(ik.iterations, 200)
        distance, angle = ik.error(target)
        self.assertLess(distance, 1e-4)
        self.assertLess(angle, 1e-4)

    def test_reference_pose_five_centimeter_target_is_solved_before_first_command(self):
        ik = CartesianIK(SIM_START)
        target = ik.pose()
        target[0, 3] += .05
        q = ik.solve(target)
        self.assertTrue(ik.converged)
        self.assertGreater(float(np.max(np.abs(q))), math.radians(30.))
        self.assertLess(ik.error(target)[0], 1e-4)

    def test_model_has_only_arm_joints_and_uses_official_limits_without_changing_source(self):
        source = Path(ee.__file__).parent / "models" / "rebot_rs.xml"
        original = source.read_bytes()
        ik = CartesianIK(TEST_START)
        self.assertEqual((ik.model.nq, ik.model.nv), (6, 6))
        np.testing.assert_allclose(np.degrees(ik.model.jnt_range), JOINT_LIMITS_DEG[:6])
        q = TEST_START.copy()
        q[2] = math.radians(195)
        ik.reset(q)
        ik.solve(ik.pose())
        self.assertGreater(ik.q[2], math.pi)
        self.assertEqual(source.read_bytes(), original)

    def test_unreachable_target_stays_finite_bounded_and_is_not_reported_reached(self):
        ik = CartesianIK(TEST_START)
        target = ik.pose()
        target[:3, 3] += [10., 10., 10.]
        q = ik.solve(target)
        self.assertTrue(np.all(np.isfinite(q)))
        self.assertTrue(np.all(q >= ik.lower - 1e-8))
        self.assertTrue(np.all(q <= ik.upper + 1e-8))
        self.assertFalse(ik.converged)
        self.assertGreater(ik.error(target)[0], 10)

    def test_solver_failure_rolls_back_configuration(self):
        ik = CartesianIK(TEST_START)
        previous = ik.q
        with patch("mink.solve_ik", return_value=np.full(6, np.nan)):
            with self.assertRaisesRegex(RuntimeError, "finite"):
                ik.solve(ik.pose())
        np.testing.assert_array_equal(ik.q, previous)

    def test_invalid_inputs_and_pose_conventions(self):
        transform = pose_matrix([.1, .2, .3, 30., 40., 50.])
        np.testing.assert_allclose(transform[:3, 3], [.1, .2, .3])
        np.testing.assert_allclose(transform[:3, :3], Rotation.from_euler("xyz", [30, 40, 50], degrees=True).as_matrix())
        ik = CartesianIK(TEST_START)
        for dt in (0, -.01, math.nan):
            with self.assertRaises(ValueError):
                ik.solve(transform, dt=dt)
        invalid = np.eye(4)
        invalid[0, 0] = 2
        with self.assertRaises(ValueError):
            ik.solve(invalid)
        with self.assertRaises(ValueError):
            ik.solve(transform, max_iters=0)

    def test_tcp_offset_is_applied_in_tool_frame(self):
        original = CartesianIK(TEST_START)
        offset = np.array([.01, -.02, .03])
        moved = CartesianIK(TEST_START, tcp_offset=offset)
        base = original.pose()
        np.testing.assert_allclose(moved.pose()[:3, 3], base[:3, 3] + base[:3, :3] @ offset)
        target = ee.target_pose(original, None, [.01, .02, .03])
        np.testing.assert_allclose(target[:3, :3], base[:3, :3])
        np.testing.assert_allclose(target[:3, 3], base[:3, 3] + [.01, .02, .03])


class TrackingMotors(VirtualMotors):
    """Instant joint following with the actual MIT wire encoder and decoder."""

    def __init__(self, raw):
        super().__init__()
        self.raw = np.asarray(raw).copy()
        self.follow = True

    def send(self, msg, timeout):
        kind, motor = (msg.arbitration_id >> 24) & 31, msg.arbitration_id & 255
        if kind == 17 and struct.unpack_from("<H", msg.data)[0] == mit.POSITION:
            self.sent.append(msg)
            self.queue.append(reply(17, motor, struct.pack("<HHf", mit.POSITION, 0, self.raw[motor - 1])))
            return
        queued = len(self.queue)
        super().send(msg, timeout)
        if kind in (1, 3, 4) and len(self.queue) > queued:
            model = "rs-06" if motor <= 3 else "rs-00"
            bound = mit.LIMITS[model].position
            if kind == 1 and self.modes[motor] == 2 and self.follow:
                self.raw[motor - 1] = mit.decode_mit(msg, model)["position"]
            code = int(round((self.raw[motor - 1] + bound) / (2 * bound) * 65535))
            self.queue[-1] = reply(2, motor, struct.pack(">HHHH", code, 32767, 32767, 250), mode=self.modes[motor])


@unittest.skipUnless(EE_AVAILABLE, "Install the ee dependency extra to test Mink control")
class HardwareTests(unittest.TestCase):
    def setUp(self):
        self.reference = np.array([.1, .2, -.3, .4, -.5, .6])
        self.virtual = TrackingMotors(self.reference + TEST_START)
        self.arm = {"reference_rad": [*self.reference, 0.], "motor_uids": dict(self.virtual.uids),
                    "models": ["rs-06"] * 3 + ["rs-00"] * 4}
        self.commands = []
        self.failure = None

    def factory(self, *args, **kwargs):
        with patch.object(mit, "check_channel"), patch.object(mit.can, "Bus", return_value=self.virtual):
            bus = mit.MotorBus(*args, **kwargs)
        self.bus = bus
        command = bus.command_position

        def record(*values):
            self.commands.append(values)
            if self.failure == "feedback" and len(self.commands) == 2:
                self.virtual.drop_holds.add(4)
            try:
                return command(*values)
            finally:
                self.virtual.drop_holds.clear()
        bus.command_position = record
        return bus

    def run_control(self, **kwargs):
        options = dict(hardware=True, offset=[.003, 0., 0.], headless=True, duration=.12,
                       bus_factory=self.factory)
        options.update(kwargs)
        with patch.object(ee, "load_config", return_value={"right": self.arm}), \
                patch.object(ee, "resolve_channel", return_value="virtual-can"), \
                redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
            return ee.control(**options)

    def assert_closed_and_holding(self):
        self.assertTrue(self.virtual.closed)
        frames = [msg for msg in self.virtual.sent if (msg.arbitration_id >> 24) & 31 == 1]
        self.assertEqual({msg.arbitration_id & 255 for msg in frames[-6:]}, set(range(1, 7)))
        self.assertFalse(any((msg.arbitration_id >> 24) & 31 == 4 for msg in self.virtual.sent))
        self.assertFalse(any((msg.arbitration_id & 255) == 7 and (msg.arbitration_id >> 24) & 31 != 0
                             for msg in self.virtual.sent))
        self.assertTrue(all(value == 2 for value in self.virtual.modes.values()))

    def test_real_mit_batches_preserve_offsets_gains_zero_velocity_and_exit_hold(self):
        result = self.run_control()
        self.assertTrue(result.reached)
        self.assertGreater(len(self.commands), 2)
        np.testing.assert_allclose(self.commands[0][0], self.reference + TEST_START, atol=1e-6)
        expected = CartesianIK(TEST_START)
        target = expected.pose()
        target[0, 3] += .003
        solution = expected.solve(target)
        moving = [c for c in self.commands[1:]
                  if np.max(np.abs(c[0] - self.commands[0][0])) > .002]
        self.assertTrue(moving)
        np.testing.assert_allclose(moving[0][0], self.reference + solution, atol=.0004)
        self.assertGreater(float(np.max(np.abs(moving[0][0] - self.commands[0][0]))), .3 / 200)
        for position, velocity, gravity, kp, kd, deadline in self.commands[1:]:
            np.testing.assert_array_equal(velocity, np.zeros(6))
            np.testing.assert_array_equal(gravity, np.zeros(6))
            np.testing.assert_array_equal(kp, ee.MOTION_KP)
            np.testing.assert_array_equal(kd, ee.MOTION_KD)
        self.assert_closed_and_holding()

    def test_feedback_failure_holds_and_closes_without_disabling(self):
        self.failure = "feedback"
        with self.assertRaises(TimeoutError):
            self.run_control()
        self.assert_closed_and_holding()

    def test_colliding_target_never_replaces_last_accepted_motor_command(self):
        original = CartesianIK.solve
        calls = 0
        colliding = np.radians([-66.403, 146.983, 13.731, 58.845, 68.73, -19.886])

        def solve(ik, *args, **kwargs):
            nonlocal calls
            calls += 1
            return original(ik, *args, **kwargs) if calls == 1 else colliding

        with patch.object(CartesianIK, "solve", solve):
            result = self.run_control()
        self.assertFalse(result.reached)
        for command in self.commands:
            np.testing.assert_allclose(command[0], self.reference + TEST_START, atol=.0004)
        self.assert_closed_and_holding()

    def test_tracking_lag_does_not_add_an_extra_host_stop_condition(self):
        self.virtual.follow = False
        result = self.run_control(offset=[.03, 0., 0.])
        self.assertFalse(result.reached)
        self.assertGreater(len(self.commands), 2)
        self.assert_closed_and_holding()

    def test_slow_ik_does_not_interrupt_can_feedback_or_reject_the_solution(self):
        original = CartesianIK.solve
        calls = 0

        def slow_solve(ik, *args, **kwargs):
            nonlocal calls
            calls += 1
            if calls > 1:
                time.sleep(.09)
            return original(ik, *args, **kwargs)

        with patch.object(CartesianIK, "solve", slow_solve):
            result = self.run_control(duration=.24)
        self.assertTrue(result.reached)
        self.assertGreater(len(self.commands), 30)
        self.assert_closed_and_holding()

    def test_uid_and_mode_checks_fail_before_enabling(self):
        for failure in ("uid", "mode"):
            with self.subTest(failure=failure):
                self.setUp()
                if failure == "uid":
                    self.arm["motor_uids"][7] = "ff" * 8
                else:
                    self.virtual.params[(3, mit.MODE)] = 1
                with self.assertRaises(ValueError):
                    self.run_control()
                self.assertTrue(self.virtual.closed)
                self.assertFalse(any((msg.arbitration_id >> 24) & 31 == 3 for msg in self.virtual.sent))

    def test_stop_during_ik_prevents_next_motion_batch(self):
        stop = threading.Event()
        original = CartesianIK.solve
        calls = 0

        def stop_solve(ik, *args, **kwargs):
            nonlocal calls
            calls += 1
            value = original(ik, *args, **kwargs)
            if calls > 1:
                stop.set()
            return value

        with patch.object(CartesianIK, "solve", stop_solve):
            result = self.run_control(stop=stop)
        self.assertTrue(result.interrupted)
        for command in self.commands:
            np.testing.assert_allclose(command[0], self.reference + TEST_START, atol=.0004)
        self.assert_closed_and_holding()

    def test_partial_enable_failure_cleans_up_only_attempted_axes(self):
        self.virtual.fail_enable = 3
        with self.assertRaises(TimeoutError):
            self.run_control()
        self.assertTrue(self.virtual.closed)
        self.assertEqual(self.bus.enabled, {1, 2, 3})
        self.assertEqual(self.virtual.modes[1], 2)
        self.assertEqual(self.virtual.modes[2], 2)
        self.assertTrue(all(self.virtual.modes[i] == 0 for i in (3, 4, 5, 6)))
        self.assertFalse(any((msg.arbitration_id >> 24) & 31 == 4 for msg in self.virtual.sent))

    def test_sigterm_stops_motion_restores_handler_and_installs_hold(self):
        previous = signal.getsignal(signal.SIGTERM)
        original = CartesianIK.solve
        calls = 0

        def terminate(ik, *args, **kwargs):
            nonlocal calls
            calls += 1
            value = original(ik, *args, **kwargs)
            if calls > 1:
                signal.raise_signal(signal.SIGTERM)
            return value

        with patch.object(CartesianIK, "solve", terminate):
            result = self.run_control()
        self.assertTrue(result.interrupted)
        self.assertEqual(signal.getsignal(signal.SIGTERM), previous)
        for command in self.commands:
            np.testing.assert_allclose(command[0], self.reference + TEST_START, atol=.0004)
        self.assert_closed_and_holding()

    def test_invalid_options_do_not_construct_bus(self):
        for options in ({"start_joints": [0.] * 6}, {"offset": [math.nan, 0., 0.]},
                        {"duration": -1}, {"side": "both"}, {"duration": None}):
            with self.subTest(options=options):
                with self.assertRaises(ValueError):
                    self.run_control(**options)
                self.assertEqual(self.virtual.sent, [])


if __name__ == "__main__":
    unittest.main()
