"""Real Mink IK and virtual-CAN EE control checks; never open a physical CAN socket."""

from contextlib import redirect_stderr, redirect_stdout
import importlib.util
import io
import math
import signal
import struct
import time
import unittest
from unittest.mock import patch

import numpy as np
from scipy.spatial.transform import Rotation

from fakes import VirtualMotors, isolate_config, reply
from rebot import bindings
from rebot.apps import ee
from rebot.bindings import ArmBinding
from rebot.devices import robstride
from rebot.model.kinematics import CartesianIK, SIM_START, pose_matrix
from rebot.model.limits import JOINT_LIMITS_DEG
from rebot.model.trajectory import MOTION_KD, MOTION_KP
from rebot.paths import MODELS
from rebot.settings import load_settings


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

    def test_default_tcp_is_the_mounting_face_with_contract_axes(self):
        ik = CartesianIK(TEST_START)
        for q in (SIM_START, TEST_START, TEST_START + np.radians([5., 4., -3., 2., 1., 30.])):
            ik.update_display(q)
            flange = ik.data.body("link6")
            rotation = flange.xmat.reshape(3, 3)
            pose = ik.pose(q)
            np.testing.assert_allclose(pose[:3, 3], flange.xpos + rotation @ [0., 0., .009], atol=1e-12)
            np.testing.assert_allclose(pose[:3, 0], rotation[:, 2], atol=1e-12)
            np.testing.assert_allclose(pose[:3, 1], -rotation[:, 1], atol=1e-12)
            np.testing.assert_allclose(pose[:3, 2], rotation[:, 0], atol=1e-12)

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

    def test_model_has_only_arm_joints_and_uses_official_limits_without_changing_source(self):
        source = MODELS / "rebot_rs.xml"
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

    def test_tcp_offset_is_applied_in_mount_frame(self):
        original = CartesianIK(TEST_START)
        offset = np.array([.01, -.02, .03])
        moved = CartesianIK(TEST_START, tcp_offset=offset)
        base = original.pose()
        np.testing.assert_allclose(moved.pose()[:3, 3], base[:3, 3] + base[:3, :3] @ offset)
        np.testing.assert_allclose(moved.pose()[:3, :3], base[:3, :3], atol=1e-12)
        target = ee.target_pose(original, None, [.01, .02, .03])
        np.testing.assert_allclose(target[:3, :3], base[:3, :3])
        np.testing.assert_allclose(target[:3, 3], base[:3, 3] + [.01, .02, .03])


class TrackingMotors(VirtualMotors):
    """Instant joint following with the actual MIT wire encoder and decoder."""

    def __init__(self, raw):
        super().__init__()
        self.raw = np.asarray(raw).copy()

    def send(self, msg, timeout):
        kind, motor = (msg.arbitration_id >> 24) & 31, msg.arbitration_id & 255
        if kind == 17 and struct.unpack_from("<H", msg.data)[0] == robstride.POSITION:
            self.sent.append(msg)
            self.queue.append(reply(17, motor, struct.pack("<HHf", robstride.POSITION, 0, self.raw[motor - 1])))
            return
        queued = len(self.queue)
        super().send(msg, timeout)
        if kind in (1, 3, 4) and len(self.queue) > queued:
            model = "rs-06" if motor <= 3 else "rs-00"
            bound = robstride.LIMITS[model].position
            if kind == 1 and self.modes[motor] == 2:
                self.raw[motor - 1] = robstride.decode_mit(msg, model)["position"]
            code = int(round((self.raw[motor - 1] + bound) / (2 * bound) * 65535))
            self.queue[-1] = reply(2, motor, struct.pack(">HHHH", code, 32767, 32767, 250), mode=self.modes[motor])


@unittest.skipUnless(EE_AVAILABLE, "Install the ee dependency extra to test Mink control")
class HardwareTests(unittest.TestCase):
    def setUp(self):
        isolate_config(self)
        self.reference = np.array([.1, .2, -.3, .4, -.5, .6])
        self.virtual = TrackingMotors(self.reference + TEST_START)
        self.binding = ArmBinding("right", "virtual-can", robstride.MODELS, dict(self.virtual.uids),
                                  np.array([*self.reference, 0.]))
        self.commands = []
        self.measured = []
        self.mechanical_reads = []
        self.failure = None
        self.factory_calls = []

    def factory(self, *args, **kwargs):
        self.factory_calls.append((args, kwargs))
        with patch.object(robstride, "check_channel"), \
                patch.object(robstride.can, "Bus", return_value=self.virtual):
            bus = robstride.MotorBus(*args, **kwargs)
        self.bus = bus
        positions = bus.positions

        def read_positions():
            raw = positions()
            self.mechanical_reads.append(np.asarray(raw).copy())
            return raw

        bus.positions = read_positions
        command = bus.command_position

        def record(*values, **options):
            # The enable-refresh batch uses the validated startup mechanical read.
            raw = (np.asarray(values[0]) if not self.commands else
                   np.array([bus.feedback[i]["position"] for i in range(1, 7)]))
            self.measured.append(raw - self.reference)
            self.commands.append(values)
            if self.failure == "feedback" and len(self.commands) == 2:
                self.virtual.drop_holds.add(4)
            try:
                return command(*values, **options)
            finally:
                self.virtual.drop_holds.clear()
        bus.command_position = record
        return bus

    def run_control(self, **kwargs):
        options = dict(hardware=True, offset=[.003, 0., 0.], headless=True, duration=.12,
                       bus_factory=self.factory)
        options.update(kwargs)
        # No addCleanup here: the EE viewer tests drive this fixture outside a test run.
        with patch.object(ee, "select_arms", side_effect=self.select_arms), \
                redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
            return ee.control(**options)

    def select_arms(self, path=None, side="both", *, calibrated=True):
        return {name: self.binding for name in bindings.select_sides(side)}

    @staticmethod
    def kind(message):
        return (message.arbitration_id >> 24) & 31

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
        model = ee.GravityModel()
        scale = load_settings().gravity_scale
        for measured, (position, velocity, gravity, kp, kd, deadline) in zip(self.measured, self.commands):
            np.testing.assert_array_equal(velocity, np.zeros(6))
            np.testing.assert_allclose(gravity, model.gravity(measured) * scale, atol=1e-10)
            np.testing.assert_array_equal(kp, MOTION_KP)
            np.testing.assert_array_equal(kd, MOTION_KD)
        self.assertGreater(np.linalg.norm(self.commands[0][2]), 1.)
        self.assertGreater(np.linalg.norm(self.commands[-1][2] - self.commands[0][2]), 1e-4)
        self.assert_closed_and_holding()
        frames = [msg for msg in self.virtual.sent if (msg.arbitration_id >> 24) & 31 == 1]
        for batch, q in ((frames[:12], TEST_START),
                         (frames[-6:], self.mechanical_reads[-1] - self.reference)):
            expected_gravity = model.gravity(q) * scale
            for frame in batch:
                motor = (frame.arbitration_id & 255) - 1
                decoded = robstride.decode_mit(frame, self.binding.models[motor])
                self.assertAlmostEqual(decoded["gravity"], expected_gravity[motor], delta=.002)
        # The enable preload and the exit hold are native position holds at the measured
        # pose: full gains, zero velocity, never a zero-position or zero-gain frame.
        for batch, raw in ((frames[:12], self.reference + TEST_START),
                           (frames[-6:], self.mechanical_reads[-1])):
            for frame in batch:
                motor = (frame.arbitration_id & 255) - 1
                limits = robstride.LIMITS[self.binding.models[motor]]
                decoded = robstride.decode_mit(frame, self.binding.models[motor])
                self.assertAlmostEqual(decoded["position"], raw[motor], delta=.0005)
                self.assertAlmostEqual(decoded["velocity"], 0., delta=2 * limits.velocity / 65535)
                self.assertAlmostEqual(decoded["kp"], MOTION_KP[motor], delta=limits.kp / 65535)
                self.assertAlmostEqual(decoded["kd"], MOTION_KD[motor], delta=limits.kd / 65535)
                self.assertGreater(decoded["kp"], 0.)
        self.assertEqual(set(self.bus.hold_records), set(range(1, 7)))
        for motor, (position, torque, kp, kd) in self.bus.hold_records.items():
            self.assertEqual((kp, kd), (MOTION_KP[motor - 1], MOTION_KD[motor - 1]))

    def test_startup_verifies_identity_pose_mode_and_watchdog_before_any_motor_command(self):
        self.run_control()
        self.assertEqual(self.factory_calls, [(("virtual-can", robstride.MODELS), {"motor_count": 6})])
        sent = self.virtual.sent
        first_command = next(index for index, message in enumerate(sent) if self.kind(message) in (1, 3, 4))
        startup = [(self.kind(message), message.arbitration_id & 255,
                    None if self.kind(message) == 0 else struct.unpack_from("<H", message.data)[0])
                   for message in sent[:first_command]]
        motors = range(1, 7)
        expected = ([(0, motor, None) for motor in range(1, 8)]
                    + [(17, motor, robstride.POSITION) for motor in motors]
                    + [(17, motor, robstride.MODE) for motor in motors]
                    + [step for motor in motors for step in ((18, motor, robstride.WATCHDOG),
                                                             (17, motor, robstride.WATCHDOG))])
        self.assertEqual(startup[:len(expected)], expected)
        # After the watchdog is off the pose is read again, and nothing else, before enabling.
        remaining = startup[len(expected):]
        self.assertGreaterEqual(len(remaining), 6)
        self.assertEqual(len(remaining) % 6, 0)
        self.assertEqual(remaining, [(17, motor, robstride.POSITION) for motor in motors] * (len(remaining) // 6))
        self.assertTrue(all(self.virtual.params[(motor, robstride.WATCHDOG)] == 0 for motor in motors))
        # Each motor is preloaded with its hold before it is enabled, in motor order.
        commands = [(self.kind(message), message.arbitration_id & 255) for message in sent[first_command:]]
        self.assertEqual(commands[:18], [step for motor in motors
                                         for step in ((1, motor), (3, motor), (1, motor))])
        self.assertEqual([motor for kind, motor in commands if kind == 3], list(motors))
        self.assertEqual({kind for kind, _ in commands}, {1, 3, 17})

    def test_exit_hold_follows_the_pose_reached_during_control(self):
        self.run_control(offset=[.02, 0., 0.], duration=.2)
        final = self.mechanical_reads[-1]
        self.assertGreater(float(np.max(np.abs(final - self.reference - TEST_START))), .01)
        for motor, (position, torque, kp, kd) in self.bus.hold_records.items():
            self.assertAlmostEqual(position, final[motor - 1], delta=.0005)
        frames = [msg for msg in self.virtual.sent if self.kind(msg) == 1][-12:]
        self.assertEqual([msg.arbitration_id & 255 for msg in frames], [*range(1, 7)] * 2)
        for frame in frames:
            motor = (frame.arbitration_id & 255) - 1
            decoded = robstride.decode_mit(frame, self.binding.models[motor])
            self.assertAlmostEqual(decoded["position"], final[motor], delta=.0005)
        self.assert_closed_and_holding()

    def test_invalid_gravity_prevents_motor_enable(self):
        for value in (math.nan, math.inf, 100.):
            with self.subTest(value=value):
                self.setUp()
                with patch.object(ee.GravityModel, "gravity", return_value=np.full(6, value)):
                    with self.assertRaises((ValueError, RuntimeError)):
                        self.run_control()
                self.assertTrue(self.virtual.closed)
                self.assertFalse(any((msg.arbitration_id >> 24) & 31 == 3 for msg in self.virtual.sent))

    def test_gravity_failure_during_control_preserves_last_valid_hold(self):
        original = ee.GravityModel.gravity
        calls = 0

        def fail(model, q):
            nonlocal calls
            calls += 1
            return original(model, q) if calls < 6 else np.full(6, math.nan)

        with patch.object(ee.GravityModel, "gravity", fail):
            with self.assertRaisesRegex(ValueError, "Gravity feedforward"):
                self.run_control()
        self.assert_closed_and_holding()
        self.assertTrue(all(abs(record[1]) < 15 for record in self.bus.hold_records.values()))

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
                    self.binding.motor_uids[7] = "ff" * 8
                else:
                    self.virtual.params[(3, robstride.MODE)] = 1
                with self.assertRaises(ValueError):
                    self.run_control()
                self.assertTrue(self.virtual.closed)
                self.assertFalse(any((msg.arbitration_id >> 24) & 31 == 3 for msg in self.virtual.sent))

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
                self.assertEqual(self.factory_calls, [])


if __name__ == "__main__":
    unittest.main()
