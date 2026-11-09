"""Offline dynamics, controller transitions, and simulated CAN lifecycle checks."""

from contextlib import redirect_stderr, redirect_stdout
import ast
import io
import json
import math
from pathlib import Path
import struct
import signal
import tempfile
import time
import unittest
from unittest.mock import MagicMock, patch

import can
import numpy as np
import pinocchio as pin
import yaml

import rebot_hand_guiding as drive
import rebot_mit as mit
import rebot_home
from rebot_motion import MotionLog


def reply(kind, motor, data=bytes(8), mode=0):
    host = 0xFE if kind == 0 else 0xFD
    return can.Message(arbitration_id=(kind << 24) | (mode << 22) | (motor << 8) | host,
                       is_extended_id=True, data=data, timestamp=time.time())


class VirtualMotors:
    """In-process firmware model; never opens SocketCAN or drives hardware."""

    def __init__(self, offset=0, fail_enable=None):
        self.queue, self.sent = [], []
        self.params = {(i, mit.WATCHDOG): 4000 for i in range(1, 7)}
        self.params.update({(i, mit.MODE): 0 for i in range(1, 7)})
        self.modes = dict.fromkeys(range(1, 7), 0)
        self.uids = {i: (offset + i).to_bytes(8, "little").hex() for i in range(1, 8)}
        self.fail_enable = fail_enable
        self.drop_stops = set()
        self.drop_holds = set()
        self.position = .2
        self.temperature = 250
        self.closed = False

    def recv(self, timeout):
        return self.queue.pop(0) if self.queue else None

    def send(self, msg, timeout):
        self.sent.append(msg)
        kind, motor = (msg.arbitration_id >> 24) & 31, msg.arbitration_id & 255
        if kind == 0:
            self.queue.append(reply(0, motor, bytes.fromhex(self.uids[motor])))
        elif kind == 17:
            parameter = struct.unpack_from("<H", msg.data)[0]
            if parameter == mit.POSITION:
                value = struct.pack("<f", self.position)
            else:
                fmt = "b" if parameter == mit.MODE else "I"
                value = struct.pack("<" + fmt, self.params[(motor, parameter)]).ljust(4, b"\0")
            self.queue.append(reply(17, motor, struct.pack("<HH", parameter, 0) + value))
        elif kind == 18:
            parameter = struct.unpack_from("<H", msg.data)[0]
            fmt = "b" if parameter == mit.MODE else "I"
            self.params[(motor, parameter)] = struct.unpack_from("<" + fmt, msg.data, 4)[0]
        elif kind in (1, 3, 4):
            if kind == 1 and motor in self.drop_holds:
                return
            if kind == 3:
                if motor == self.fail_enable:
                    return
                self.modes[motor] = 2
            elif kind == 4:
                if motor in self.drop_stops:
                    return
                self.modes[motor] = 0
            self.queue.append(reply(2, motor, struct.pack(">HHHH", 32767, 32767, 32767, self.temperature),
                                    mode=self.modes[motor]))
        else:
            raise AssertionError(f"Unexpected command type: {kind}")

    def shutdown(self):
        self.closed = True


class ControllerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = drive.GravityModel()

    def setUp(self):
        self.cfg = drive.load_settings()
        self.q = np.array([0, .5, 1., 0, .1, 0])
        self.ctrl = drive.HandGuidingController(self.model, self.cfg)

    def advance(self, q, start, end, step=.02):
        for now in np.arange(start, end + step / 2, step):
            result = self.ctrl.update(q, float(now))
        return result

    def test_gravity_matches_potential_energy_derivative(self):
        epsilon = 1e-6
        gradient = []
        for i in range(6):
            delta = np.zeros(6)
            delta[i] = epsilon
            upper = pin.computePotentialEnergy(self.model.model, self.model.data, self.q + delta)
            lower = pin.computePotentialEnergy(self.model.model, self.model.data, self.q - delta)
            gradient.append((upper - lower) / (2 * epsilon))
        np.testing.assert_allclose(self.model.gravity(self.q), gradient, atol=1e-6)

    def test_startup_preserves_gravity_support_from_the_first_command(self):
        result = self.ctrl.update(self.q, 0)
        gravity = self.model.gravity(self.q) * self.cfg["gravity_scale"]
        np.testing.assert_allclose(result.torque, gravity)
        np.testing.assert_allclose(self.ctrl.target, self.q)
        result = self.advance(self.q, .02, 1.0)
        np.testing.assert_allclose(result.gravity, gravity)

    def test_startup_to_hold_keeps_the_existing_target_and_gain(self):
        self.ctrl.update(self.q, 0)
        moved = self.q.copy()
        moved[0] += .002
        self.advance(moved, .02, 1.98)
        before = self.ctrl.update(moved, 1.999)
        after = self.ctrl.update(moved, 2.001)
        self.assertEqual(after.state, "HOLD")
        np.testing.assert_allclose(self.ctrl.target, self.q)
        np.testing.assert_allclose(after.torque, before.torque, atol=1e-6)

    def test_fast_motion_does_not_abort_in_startup_or_drag(self):
        for start in (0., 2.4):
            for direction in (-1, 1):
                with self.subTest(start=start, direction=direction):
                    self.ctrl = drive.HandGuidingController(self.model, self.cfg)
                    self.advance(self.q, 0, start)
                    moved = self.q.copy()
                    for i in range(1, 11):
                        moved[0] += direction * .08
                        result = self.ctrl.update(moved, start + i * .02)
                        self.assertTrue(np.all(np.isfinite(result.torque)))
                        self.assertTrue(np.all(np.abs(result.torque) <= self.cfg["torque_limit_nm"]))
                    self.assertEqual(result.state, "STARTUP" if start == 0 else "DRAG")

    def test_logged_joint6_motion_does_not_trigger_sustained_speed_stop(self):
        moved = self.q.copy()
        moved[5] = -.434270
        self.advance(moved, 0, 2.4)
        for i in range(1, 11):
            moved[5] += .042146
            result = self.ctrl.update(moved, 2.4 + i * .020041)
        self.assertEqual(result.state, "DRAG")

    def test_drag_release_stillness_hold_and_push_again(self):
        result = self.advance(self.q, 0, 2.4)
        self.assertEqual(result.state, "HOLD")
        moved = self.q.copy()
        for i in range(1, 9):
            moved[0] += .005
            result = self.ctrl.update(moved, 2.4 + i * .02)
        self.assertEqual(result.state, "DRAG")
        np.testing.assert_allclose(self.ctrl.target, moved)
        result = self.advance(moved, 2.58, 3.5)
        self.assertEqual(result.state, "HOLD")
        np.testing.assert_allclose(self.ctrl.target, moved)
        for now in (3.52, 3.54, 3.56):
            moved[1] += .01
            result = self.ctrl.update(moved, now)
        self.assertEqual(result.state, "DRAG")

    def test_error_threshold_releases_even_during_slow_push(self):
        self.advance(self.q, 0, 2.4)
        moved = self.q.copy()
        # Slow enough that velocity release does not trigger first.
        for i in range(1, 41):
            moved[0] += .001
            result = self.ctrl.update(moved, 2.4 + i * .02)
        self.assertEqual(result.state, "DRAG")

    def test_total_torque_is_clamped(self):
        cfg = {**self.cfg, "damping": np.full(6, 1000.)}
        ctrl = drive.HandGuidingController(self.model, cfg)
        ctrl.update(self.q, 0)
        result = ctrl.update(self.q + .01, .02)
        self.assertTrue(np.all(np.abs(result.torque) <= cfg["torque_limit_nm"]))
        self.assertTrue(np.any(np.abs(result.torque) == cfg["torque_limit_nm"]))

    def test_gap_and_invalid_pose_abort(self):
        self.ctrl.update(self.q, 0)
        for q, now, message in ((self.q, .2, "deadline"),
                                (np.array([0, -1, 0, 0, 0, 0]), .02, "outside")):
            with self.subTest(message=message), self.assertRaisesRegex((ValueError, RuntimeError), message):
                self.ctrl.update(q, now)

    def test_gravity_above_limit_is_rejected_instead_of_silently_undercompensated(self):
        self.ctrl.settings = {**self.cfg, "torque_limit_nm": np.full(6, .001)}
        with self.assertRaisesRegex(RuntimeError, "gravity torque"):
            self.ctrl.update(self.q, 0)


class ProtocolTests(unittest.TestCase):
    def setUp(self):
        self.virtual = VirtualMotors()
        self.patches = [patch.object(mit, "check_channel"),
                        patch.object(mit.can, "Bus", return_value=self.virtual)]
        for item in self.patches:
            item.start()
            self.addCleanup(item.stop)
        self.bus = mit.MotorBus("can0", ["rs-06"] * 3 + ["rs-00"] * 4, timeout=.001)
        self.bus.set_hold([.2] * 6, [1.] * 6, [6, 15, 12, 4, 3, 2], [.5, 1.5, 1.2, .4, .3, .2])
        self.addCleanup(self.virtual.shutdown)

    def test_encoding_matches_vendor_endpoints_and_zero_gains(self):
        for model, bound in (("rs-06", 36.), ("rs-00", 14.)):
            for torque, code in ((-bound, 0), (0, 32767), (bound, 65535)):
                frame = mit.encode_mit(1, model, 0, torque)
                self.assertEqual((frame.arbitration_id >> 8) & 65535, code)
                self.assertEqual(struct.unpack(">HHHH", frame.data), (32767, 32767, 0, 0))
        with self.assertRaises(ValueError):
            mit.encode_mit(7, "rs-00", 0, 0)

    def test_prepare_and_exit_keep_enabled_with_native_hold(self):
        with redirect_stdout(io.StringIO()), self.bus:
            self.assertEqual(self.bus.scan(), self.virtual.uids)
            self.bus.prepare()
            self.assertTrue(all(self.virtual.params[(i, mit.WATCHDOG)] == 0 for i in range(1, 7)))
            self.bus.enable([.2] * 6)
            self.bus.command([.2] * 6, [1.] * 6)
        self.assertTrue(self.virtual.closed)
        self.assertEqual(set(self.virtual.modes.values()), {2})
        self.assertTrue(all(self.virtual.params[(i, mit.WATCHDOG)] == 0 for i in range(1, 7)))
        writes = [m for m in self.virtual.sent if (m.arbitration_id >> 24) & 31 not in (0, 17)]
        self.assertTrue(all((m.arbitration_id & 255) <= 6 for m in writes))
        self.assertFalse(any((m.arbitration_id >> 24) & 31 in (4, 6, 7, 22, 23, 25) for m in writes))
        holds = [m for m in writes if (m.arbitration_id >> 24) & 31 == 1][-6:]
        self.assertTrue(all(struct.unpack(">HHHH", m.data)[2] > 0 for m in holds))

    def test_queued_fault_is_not_discarded_before_next_command(self):
        self.virtual.queue.append(reply(21, 2, b"\x01" + bytes(7)))
        with self.assertRaisesRegex(RuntimeError, "fault/warning"):
            self.bus.command([.2] * 6, [0.] * 6)
        self.assertEqual(self.virtual.sent, [])

    def test_hold_failure_does_not_disable_or_skip_other_motors(self):
        self.bus.prepare()
        self.bus.enable([.2] * 6)
        self.virtual.drop_holds.add(1)
        with redirect_stderr(io.StringIO()), redirect_stdout(io.StringIO()):
            self.bus.freeze()
        self.assertEqual(set(self.virtual.modes.values()), {2})
        self.assertFalse(any((m.arbitration_id >> 24) & 31 == 4 for m in self.virtual.sent))
        self.assertTrue(all(self.virtual.params[(i, mit.WATCHDOG)] == 0 for i in range(1, 7)))

    def test_invalid_batch_sends_no_partial_torque_commands(self):
        with self.assertRaises(ValueError):
            self.bus.command([.2] * 6, [0.] * 5 + [math.nan])
        self.assertEqual(self.virtual.sent, [])

    def test_native_motion_sends_all_axes_before_waiting_for_replies(self):
        self.virtual.modes = dict.fromkeys(range(1, 7), 2)
        original = self.virtual.recv
        counts = []

        def recv(timeout):
            count = sum((frame.arbitration_id >> 24) & 31 == 1 for frame in self.virtual.sent)
            if count:
                counts.append(count)
            return original(timeout)

        self.virtual.recv = recv
        result = self.bus.command_position([.3] * 6, [.04] * 6, [1.] * 6,
                                          [12.] * 6, [.5] * 6, time.monotonic() + .08)
        self.assertEqual(len(result), 6)
        self.assertTrue(counts)
        self.assertEqual(set(counts), {6})
        for record in result:
            self.assertAlmostEqual(record["position"], .3, delta=.001)
            self.assertAlmostEqual(record["velocity"], .04, delta=.002)
            self.assertGreater(record["kp"], 0)
            self.assertGreater(record["kd"], 0)

    def test_native_motion_rejects_invalid_batch_and_missing_feedback(self):
        with self.assertRaises(ValueError):
            self.bus.command_position([.2] * 6, [0.] * 5 + [math.nan], [0.] * 6,
                                      [12.] * 6, [.5] * 6, time.monotonic() + .08)
        self.assertEqual(self.virtual.sent, [])
        self.virtual.modes = dict.fromkeys(range(1, 7), 2)
        self.virtual.drop_holds.add(6)
        with self.assertRaisesRegex(TimeoutError, "motors \\[6\\]"):
            self.bus.command_position([.2] * 6, [0.] * 6, [0.] * 6,
                                      [12.] * 6, [.5] * 6, time.monotonic() + .08)

    def test_hot_status_is_recorded_and_fault_bits_still_abort(self):
        for kind in (2, 24):
            with self.subTest(kind=kind):
                hot = reply(kind, 2, struct.pack(">HHHH", 32767, 32767, 32767, 700))
                self.bus.inspect(hot)
                self.assertEqual(self.bus.feedback[2]["temperature"], 70.)
                hot.arbitration_id |= 1 << 16
                with self.assertRaisesRegex(RuntimeError, "faults=0x01"):
                    self.bus.inspect(hot)

    def test_deadline_is_checked_before_each_motor_command(self):
        self.virtual.modes = dict.fromkeys(range(1, 7), 2)
        original = self.virtual.send

        def delayed_send(msg, timeout):
            original(msg, timeout)
            time.sleep(.01)

        self.virtual.send = delayed_send
        with self.assertRaisesRegex(TimeoutError, "deadline expired"):
            self.bus.command([.2] * 6, [0.] * 6, deadline=time.monotonic() + .005)
        commands = [m for m in self.virtual.sent if (m.arbitration_id >> 24) & 31 == 1]
        self.assertEqual(len(commands), 1)

    def test_parameter_readback_failure_prevents_enable(self):
        original = self.virtual.send

        def ignore_watchdog_write(msg, timeout):
            kind = (msg.arbitration_id >> 24) & 31
            if kind == 18 and struct.unpack_from("<H", msg.data)[0] == mit.WATCHDOG:
                self.virtual.sent.append(msg)
            else:
                original(msg, timeout)

        self.virtual.send = ignore_watchdog_write
        with redirect_stderr(io.StringIO()), self.assertRaisesRegex(RuntimeError, "readback"):
            with self.bus:
                self.bus.prepare()
                self.bus.enable([.2] * 6)
        self.assertFalse(any((m.arbitration_id >> 24) & 31 == 3 for m in self.virtual.sent))

    def test_wrong_mode_does_not_trigger_automatic_disable_or_mode_switch(self):
        self.virtual.params[(2, mit.MODE)] = 1
        with self.assertRaisesRegex(ValueError, "already be in MIT"):
            self.bus.prepare()
        self.assertTrue(all((m.arbitration_id >> 24) & 31 == 17 for m in self.virtual.sent))

    def test_late_preload_reply_does_not_make_enable_fail(self):
        original = self.virtual.send
        pending = {}

        def delayed_preload(msg, timeout):
            kind, motor = (msg.arbitration_id >> 24) & 31, msg.arbitration_id & 255
            if kind == 1 and self.virtual.modes[motor] == 0:
                # Preload reply exists on the wire, but has not arrived during drain().
                self.virtual.sent.append(msg)
                pending[motor] = True
                return
            original(msg, timeout)
            if kind == 3 and pending.pop(motor, False):
                self.virtual.queue.insert(0, reply(2, motor, mode=0))

        self.virtual.send = delayed_preload
        with redirect_stdout(io.StringIO()):
            self.bus.enable([.2] * 6)
        self.assertEqual(set(self.virtual.modes.values()), {2})
        self.assertEqual(self.bus.enabled, set(range(1, 7)))
        self.assertFalse(any((m.arbitration_id >> 24) & 31 == 4 for m in self.virtual.sent))

    def test_enable_does_not_require_an_ack_for_the_enable_frame_itself(self):
        original = self.virtual.send

        def no_enable_ack(msg, timeout):
            kind, motor = (msg.arbitration_id >> 24) & 31, msg.arbitration_id & 255
            if kind == 3:
                self.virtual.sent.append(msg)
                self.virtual.modes[motor] = 2
                return
            original(msg, timeout)

        self.virtual.send = no_enable_ack
        with redirect_stdout(io.StringIO()):
            self.bus.enable([.2] * 6)
        self.assertEqual(self.bus.enabled, set(range(1, 7)))

    def test_persistent_reset_state_is_not_reported_as_enable_success(self):
        self.virtual.fail_enable = 1
        with self.assertRaisesRegex(TimeoutError, "mode 2; last observed mode=0"):
            self.bus.enable([.2] * 6)
        self.assertEqual(self.virtual.modes[1], 0)
        self.assertEqual(self.bus.enabled, {1})

    def test_fault_during_enable_is_not_ignored_while_waiting_for_run_mode(self):
        original = self.virtual.send

        def fault_on_enable(msg, timeout):
            original(msg, timeout)
            if (msg.arbitration_id >> 24) & 31 == 3:
                self.virtual.queue.insert(0, reply(21, 1, b"\x01" + bytes(7)))

        self.virtual.send = fault_on_enable
        with self.assertRaisesRegex(RuntimeError, "fault/warning"):
            self.bus.enable([.2] * 6)

    def test_restarting_adopts_running_motors_without_reenabling_or_zeroing_support(self):
        self.bus.prepare()
        with redirect_stdout(io.StringIO()):
            self.bus.enable([.2] * 6)
            self.bus.freeze()
        self.virtual.sent.clear()
        with redirect_stdout(io.StringIO()):
            self.bus.enable([.2] * 6)
        self.assertFalse(any((m.arbitration_id >> 24) & 31 in (3, 4) for m in self.virtual.sent))
        for msg in self.virtual.sent:
            if (msg.arbitration_id >> 24) & 31 != 1:
                continue
            motor = msg.arbitration_id & 255
            bound = mit.LIMITS[self.bus.models[motor - 1]].torque
            torque = ((msg.arbitration_id >> 8) & 65535) / 65535 * 2 * bound - bound
            self.assertAlmostEqual(torque, 1.0, delta=.002)
            self.assertGreater(struct.unpack(">HHHH", msg.data)[2], 0)


class LifecycleTests(unittest.TestCase):
    def setup_run(self, fail_right=False):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.journal_dir = Path(directory.name)
        self.virtual = {"can0": VirtualMotors(), "can1": VirtualMotors(100, 3 if fail_right else None)}
        self.arms = {side: dict(channel=channel, models=["rs-06"] * 3 + ["rs-00"] * 4,
                               reference_rad=[0.] * 7, motor_uids=self.virtual[channel].uids)
                     for side, channel in (("left", "can0"), ("right", "can1"))}
        for item in (patch.object(mit, "check_channel"),
                     patch.object(mit.can, "Bus", side_effect=lambda **kw: self.virtual[kw["channel"]]),
                     patch.object(drive, "load_config", return_value=self.arms),
                     patch.object(drive, "resolve_channel", side_effect=lambda arm: arm["channel"]),
                     patch.object(rebot_home, "load_config", return_value=self.arms),
                     patch.object(rebot_home, "resolve_channel", side_effect=lambda arm: arm["channel"]),
                     patch.object(rebot_home, "MotionLog", side_effect=lambda: MotionLog(self.journal_dir))):
            item.start()
            self.addCleanup(item.stop)

    def test_default_run_enables_motors_and_sends_control_torque(self):
        self.setup_run()
        with redirect_stdout(io.StringIO()):
            drive.run(side="both", duration=.02)
        for bus in self.virtual.values():
            self.assertTrue(bus.closed)
            self.assertEqual(set(bus.modes.values()), {2})
            commands = [m for m in bus.sent if (m.arbitration_id >> 24) & 31 == 1]
            self.assertTrue(any(struct.unpack(">HHHH", m.data)[2:] == (0, 0) for m in commands))
            self.assertTrue(all(struct.unpack(">HHHH", m.data)[2] > 0 for m in commands[-6:]))

    def test_cli_runs_without_enable_flag(self):
        with patch.object(drive.sys, "argv", ["rebot_hand_guiding.py", "--side", "both"]), \
                patch.object(drive, "run") as run:
            self.assertEqual(drive.main(), 0)
            run.assert_called_once()
            self.assertEqual(run.call_args.kwargs["side"], "both")
            self.assertNotIn("enable", run.call_args.kwargs)

    def test_removed_enable_flag_is_rejected(self):
        with patch.object(drive.sys, "argv", ["rebot_hand_guiding.py", "--enable"]), \
                redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as error:
            drive.main()
        self.assertEqual(error.exception.code, 2)

    def test_partial_dual_enable_failure_keeps_enabled_motors_held(self):
        self.setup_run(fail_right=True)
        with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
            with self.assertRaises(TimeoutError):
                drive.run(side="both", duration=.02)
        for bus in self.virtual.values():
            self.assertTrue(bus.closed)
            self.assertFalse(any((m.arbitration_id >> 24) & 31 == 4 for m in bus.sent))
        self.assertEqual(set(self.virtual["can0"].modes.values()), {2})
        self.assertEqual(self.virtual["can1"].modes[1], 2)
        self.assertEqual(self.virtual["can1"].modes[3], 0)

    def test_duration_exit_leaves_native_hold_on_both_arms(self):
        self.setup_run()
        with redirect_stdout(io.StringIO()):
            drive.run(side="both", duration=.025)
        for bus in self.virtual.values():
            commands = [m for m in bus.sent if (m.arbitration_id >> 24) & 31 == 1]
            self.assertGreater(len(commands), 6)
            for msg in commands:
                q_code, _, kp, kd = struct.unpack(">HHHH", msg.data)
                decoded_q = q_code / 65535 * (8 * math.pi) - 4 * math.pi
                self.assertAlmostEqual(decoded_q, .2, delta=.001)
            self.assertTrue(all(struct.unpack(">HHHH", m.data)[2] > 0 for m in commands[-6:]))
            self.assertTrue(bus.closed)
            self.assertEqual(set(bus.modes.values()), {2})
            self.assertFalse(any((m.arbitration_id >> 24) & 31 == 4 for m in bus.sent))

    def test_stale_pair_is_rejected_before_sending_control_torque(self):
        self.setup_run()
        original = drive.HandGuidingController.update

        def delayed_update(controller, *args):
            result = original(controller, *args)
            time.sleep(.045)
            return result

        with patch.object(drive.HandGuidingController, "update", delayed_update), \
                redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
            with self.assertRaisesRegex(RuntimeError, "too old"):
                drive.run(side="both", duration=.025)
        for bus in self.virtual.values():
            kinds = [(m.arbitration_id >> 24) & 31 for m in bus.sent]
            last_enable = max(i for i, kind in enumerate(kinds) if kind == 3)
            after = bus.sent[last_enable + 1:]
            controls = [m for m in after if (m.arbitration_id >> 24) & 31 == 1]
            self.assertTrue(all(struct.unpack(">HHHH", m.data)[2] > 0 for m in controls))
            self.assertEqual(set(bus.modes.values()), {2})
            self.assertNotIn(4, kinds)

    def test_ctrl_c_exits_to_hold_without_disable(self):
        self.setup_run()
        original = drive.HandGuidingController.update

        def request_stop(controller, *args):
            result = original(controller, *args)
            signal.raise_signal(signal.SIGINT)
            return result

        with patch.object(drive.HandGuidingController, "update", request_stop), redirect_stdout(io.StringIO()):
            drive.run(side="both")
        for bus in self.virtual.values():
            self.assertTrue(bus.closed)
            self.assertEqual(set(bus.modes.values()), {2})
            self.assertFalse(any((m.arbitration_id >> 24) & 31 == 4 for m in bus.sent))

    def test_second_run_keeps_gravity_support_and_does_not_reenable_running_arm(self):
        self.setup_run()
        with redirect_stdout(io.StringIO()):
            drive.run(side="right", duration=.025)
        bus = self.virtual["can1"]
        bus.sent.clear()
        with redirect_stdout(io.StringIO()):
            drive.run(side="right", duration=.025)
        kinds = [(m.arbitration_id >> 24) & 31 for m in bus.sent]
        self.assertNotIn(3, kinds)
        self.assertNotIn(4, kinds)
        expected = drive.GravityModel().gravity(np.full(6, .2)) * drive.load_settings()["gravity_scale"]
        frames = [m for m in bus.sent if (m.arbitration_id >> 24) & 31 == 1]
        self.assertTrue(frames)
        for msg in frames:
            motor = msg.arbitration_id & 255
            model = self.arms["right"]["models"][motor - 1]
            bound = mit.LIMITS[model].torque
            torque = ((msg.arbitration_id >> 8) & 65535) / 65535 * 2 * bound - bound
            self.assertAlmostEqual(torque, expected[motor - 1], delta=.003)

    def test_space_stop_does_not_disable(self):
        self.setup_run()
        keys = MagicMock()
        keys.__enter__.return_value.pressed.return_value = True
        with patch.object(drive, "StopKey", return_value=keys), redirect_stdout(io.StringIO()):
            drive.run(side="both")
        for bus in self.virtual.values():
            self.assertEqual(set(bus.modes.values()), {2})
            self.assertFalse(any((m.arbitration_id >> 24) & 31 == 4 for m in bus.sent))

    def test_home_disable_stops_all_seven_motors(self):
        self.setup_run()
        for bus in self.virtual.values():
            bus.modes = dict.fromkeys(range(1, 8), 2)
        with patch.object(rebot_home, "load_config", return_value=self.arms), \
                patch.object(rebot_home, "resolve_channel", side_effect=lambda arm: arm["channel"]), \
                redirect_stdout(io.StringIO()):
            self.assertTrue(rebot_home.disable(side="both"))
        for bus in self.virtual.values():
            self.assertTrue(bus.closed)
            self.assertEqual(set(bus.modes.values()), {0})
            self.assertEqual({m.arbitration_id & 255 for m in bus.sent
                              if (m.arbitration_id >> 24) & 31 == 4}, set(range(1, 8)))

    def run_completed_home(self, side="both", residual=0.):
        for bus in self.virtual.values():
            bus.position = residual
        clock = [0.]

        def now():
            clock[0] += .00001
            return clock[0]

        def sleep(duration):
            clock[0] += duration

        keys = MagicMock()
        keys.__enter__.return_value.pressed.side_effect = lambda: clock[0] >= 5
        with patch.object(rebot_home, "load_config", return_value=self.arms), \
                patch.object(rebot_home, "resolve_channel", side_effect=lambda arm: arm["channel"]), \
                patch.object(rebot_home, "StopKey", return_value=keys), \
                patch.object(rebot_home.time, "monotonic", now), \
                patch.object(rebot_home.time, "sleep", sleep), redirect_stdout(io.StringIO()):
            return rebot_home.home(side=side)

    def test_manual_home_completion_disables_all_seven_motors_on_both_arms(self):
        self.setup_run()
        self.assertTrue(self.run_completed_home())
        journals = list(self.journal_dir.glob("home-*.jsonl"))
        self.assertEqual(len(journals), 1)
        rows = [json.loads(line) for line in journals[0].read_text().splitlines()]
        self.assertEqual(rows[0]["control"], "native_mit")
        self.assertEqual(rows[0]["speed"], .3)
        samples = [row for row in rows if row["type"] == "sample"]
        self.assertEqual({row["side"] for row in samples}, {"left", "right"})
        self.assertTrue(all("native_parameters" in row and "cycle_s" in row for row in samples))
        for bus in self.virtual.values():
            self.assertTrue(bus.closed)
            self.assertEqual(set(bus.modes.values()), {0})
            self.assertEqual({m.arbitration_id & 255 for m in bus.sent
                              if (m.arbitration_id >> 24) & 31 == 4}, set(range(1, 8)))
            kinds = [(m.arbitration_id >> 24) & 31 for m in bus.sent]
            first_disable = kinds.index(4)
            self.assertNotIn(1, kinds[first_disable:])
            self.assertNotIn(3, kinds[first_disable:])

    def test_single_arm_home_does_not_disable_the_other_arm(self):
        self.setup_run()
        self.assertTrue(self.run_completed_home(side="right"))
        self.assertEqual(set(self.virtual["can1"].modes.values()), {0})
        self.assertFalse(self.virtual["can0"].sent)

    def test_hot_arm_can_complete_home_and_disable_all_seven_motors(self):
        self.setup_run()
        self.virtual["can1"].temperature = 680
        self.assertTrue(self.run_completed_home(side="right"))
        self.assertEqual(set(self.virtual["can1"].modes.values()), {0})
        self.assertFalse(self.virtual["can0"].sent)

    def test_home_residual_within_relaxed_tolerance_disables_all_motors(self):
        self.setup_run()
        self.assertTrue(self.run_completed_home(side="right", residual=.04))
        bus = self.virtual["can1"]
        self.assertEqual(set(bus.modes.values()), {0})
        self.assertEqual({m.arbitration_id & 255 for m in bus.sent
                          if (m.arbitration_id >> 24) & 31 == 4}, set(range(1, 8)))

    def test_home_residual_outside_tolerance_keeps_motors_enabled(self):
        self.setup_run()
        self.assertFalse(self.run_completed_home(side="right", residual=.06))
        bus = self.virtual["can1"]
        self.assertEqual(set(bus.modes.values()), {2})
        self.assertFalse(any((m.arbitration_id >> 24) & 31 == 4 for m in bus.sent))

    def test_home_disable_failure_is_reported_and_other_motors_are_attempted(self):
        self.setup_run()
        self.virtual["can0"].drop_stops.add(3)
        with redirect_stderr(io.StringIO()), self.assertRaisesRegex(RuntimeError, "not confirmed"):
            self.run_completed_home()
        self.assertEqual(self.virtual["can0"].modes[3], 2)
        self.assertEqual(self.virtual["can0"].modes[7], 0)
        self.assertEqual(set(self.virtual["can1"].modes.values()), {0})

    def test_home_signal_or_motion_error_skips_disable(self):
        for failure in ("signal", "error"):
            with self.subTest(failure=failure):
                self.setup_run()
                original = rebot_home.MotionController.update

                def update(controller, *args):
                    if failure == "error":
                        raise RuntimeError("Motion failed.")
                    record = original(controller, *args)
                    signal.raise_signal(signal.SIGINT)
                    return record

                with patch.object(rebot_home, "load_config", return_value=self.arms), \
                        patch.object(rebot_home, "resolve_channel", side_effect=lambda arm: arm["channel"]), \
                        patch.object(rebot_home.MotionController, "update", update), \
                        patch.object(rebot_home, "disable") as disable, redirect_stdout(io.StringIO()):
                    if failure == "error":
                        with self.assertRaisesRegex(RuntimeError, "Motion failed"):
                            rebot_home.home(side="right")
                    else:
                        self.assertFalse(rebot_home.home(side="right"))
                    disable.assert_not_called()
                bus = self.virtual["can1"]
                self.assertTrue(bus.closed)
                self.assertEqual(set(bus.modes.values()), {2})
                self.assertFalse(any((m.arbitration_id >> 24) & 31 == 4 for m in bus.sent))

    def test_home_cli_reports_completion_interruption_and_disable_failure(self):
        for result, status in ((True, 0), (False, 130), (RuntimeError("Disable failed."), 2)):
            with self.subTest(status=status), \
                    patch.object(rebot_home.sys, "argv", ["rebot_home.py", "--side", "both"]), \
                    patch.object(rebot_home, "home", return_value=result) as home, \
                    redirect_stderr(io.StringIO()):
                if isinstance(result, Exception):
                    home.side_effect = result
                self.assertEqual(rebot_home.main(), status)
                self.assertEqual(home.call_args.args[2], "both")

    def test_home_tracking_lag_continues_until_manual_stop_and_keeps_enabled(self):
        self.setup_run()
        clock = [0.]

        def now():
            clock[0] += .00001
            return clock[0]

        def sleep(duration):
            clock[0] += duration

        keys = MagicMock()
        keys.__enter__.return_value.pressed.side_effect = lambda: clock[0] >= 12
        output = io.StringIO()
        with patch.object(rebot_home, "load_config", return_value=self.arms), \
                patch.object(rebot_home, "resolve_channel", side_effect=lambda arm: arm["channel"]), \
                patch.object(rebot_home, "StopKey", return_value=keys), \
                patch.object(rebot_home.time, "monotonic", now), \
                patch.object(rebot_home.time, "sleep", sleep), redirect_stdout(output):
            self.assertFalse(rebot_home.home(side="right"))
        self.assertGreaterEqual(clock[0], 12)
        self.assertNotIn("Home reached", output.getvalue())
        bus = self.virtual["can1"]
        frames = [msg for msg in bus.sent if (msg.arbitration_id >> 24) & 31 == 1]
        decoded = [mit.decode_mit(msg, self.arms["right"]["models"][(msg.arbitration_id & 255)-1])
                   for msg in frames]
        self.assertTrue(all(item["kp"] > 0 and item["kd"] > 0 for item in decoded))
        self.assertTrue(any(item["velocity"] < -.005 for item in decoded))
        self.assertTrue(bus.closed)
        self.assertEqual(set(bus.modes.values()), {2})
        self.assertFalse(any((m.arbitration_id >> 24) & 31 == 4 for m in bus.sent))

    def test_home_starts_from_enabled_target_without_feedback_reset(self):
        self.setup_run()
        clock, reads, targets = [0.], [0], []
        original = mit.MotorBus.command_position

        def now():
            clock[0] += .00001
            return clock[0]

        def sleep(duration):
            clock[0] += duration

        def positions(bus):
            reads[0] += 1
            return [.2 if reads[0] == 1 else .23 if reads[0] == 2 else .24] * 6

        def command(bus, positions, *args):
            targets.append(np.array(positions))
            return original(bus, positions, *args)

        keys = MagicMock()
        keys.__enter__.return_value.pressed.side_effect = lambda: clock[0] > .2
        with patch.object(rebot_home, "load_config", return_value=self.arms), \
                patch.object(rebot_home, "resolve_channel", side_effect=lambda arm: arm["channel"]), \
                patch.object(rebot_home, "StopKey", return_value=keys), \
                patch.object(rebot_home.time, "monotonic", now), \
                patch.object(rebot_home.time, "sleep", sleep), \
                patch.object(mit.MotorBus, "positions", positions), \
                patch.object(mit.MotorBus, "command_position", command), redirect_stdout(io.StringIO()):
            rebot_home.home(side="right")
        self.assertTrue(targets)
        np.testing.assert_allclose(targets[0], [.23] * 6, atol=.0001)


class DisableIsolationTests(unittest.TestCase):
    def test_control_modules_cannot_call_disable_or_send_type_four(self):
        for module in (drive, mit):
            tree = ast.parse(Path(module.__file__).read_text())
            for node in ast.walk(tree):
                if isinstance(node, ast.Import):
                    self.assertFalse(any(n.name == "rebot_home" for n in node.names))
                if isinstance(node, ast.ImportFrom):
                    self.assertNotEqual(node.module, "rebot_home")
                if not isinstance(node, ast.Call):
                    continue
                if isinstance(node.func, ast.Attribute):
                    self.assertNotIn(node.func.attr, ("disable", "disable_all", "stop"))
                    if node.func.attr == "send" and node.args and isinstance(node.args[0], ast.Constant):
                        self.assertNotEqual(node.args[0].value, 4)


class SettingsTests(unittest.TestCase):
    def test_invalid_settings_are_rejected_before_can_is_opened(self):
        for key, value in (("rate_hz", 1), ("gravity_scale", [-1] * 6),
                           ("hold_kp", [math.nan] * 6), ("max_sample_gap_s", 1)):
            settings = yaml.safe_load(drive.SETTINGS_PATH.read_text())
            settings[key] = value
            with self.subTest(key=key), tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / "settings.yaml"
                path.write_text(yaml.safe_dump(settings))
                with self.assertRaises(ValueError):
                    drive.load_settings(path)


if __name__ == "__main__":
    unittest.main()
