"""RobStride private-CAN protocol and `MotorBus` checks against simulated motors."""

from contextlib import redirect_stderr, redirect_stdout
import io
import math
import struct
import time
import unittest
from unittest.mock import patch

from rebot.devices import robstride
from fakes import VirtualMotors, reply


class ProtocolTests(unittest.TestCase):
    def setUp(self):
        self.virtual = VirtualMotors()
        self.patches = [patch.object(robstride, "check_channel"),
                        patch.object(robstride.can, "Bus", return_value=self.virtual)]
        for item in self.patches:
            item.start()
            self.addCleanup(item.stop)
        self.bus = robstride.MotorBus("can0", ["rs-06"] * 3 + ["rs-00"] * 4, timeout=.001)
        self.bus.set_hold([.2] * 6, [1.] * 6, [6, 15, 12, 4, 3, 2], [.5, 1.5, 1.2, .4, .3, .2])
        self.addCleanup(self.virtual.shutdown)

    def test_encoding_matches_vendor_endpoints_and_zero_gains(self):
        for model, bound in (("rs-06", 36.), ("rs-00", 14.)):
            for torque, code in ((-bound, 0), (0, 32767), (bound, 65535)):
                frame = robstride.encode_mit(1, model, 0, torque)
                self.assertEqual((frame.arbitration_id >> 8) & 65535, code)
                self.assertEqual(struct.unpack(">HHHH", frame.data), (32767, 32767, 0, 0))
        with self.assertRaises(ValueError):
            robstride.encode_mit(7, "rs-00", 0, 0)

    def test_prepare_and_exit_keep_enabled_with_native_hold(self):
        with redirect_stdout(io.StringIO()), self.bus:
            self.assertEqual(self.bus.scan(), self.virtual.uids)
            self.bus.prepare()
            self.assertTrue(all(self.virtual.params[(i, robstride.WATCHDOG)] == 0 for i in range(1, 7)))
            self.bus.enable([.2] * 6)
            self.bus.command([.2] * 6, [1.] * 6)
        self.assertTrue(self.virtual.closed)
        self.assertEqual(set(self.virtual.modes.values()), {2})
        self.assertTrue(all(self.virtual.params[(i, robstride.WATCHDOG)] == 0 for i in range(1, 7)))
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
        self.assertTrue(all(self.virtual.params[(i, robstride.WATCHDOG)] == 0 for i in range(1, 7)))

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
            if kind == 18 and struct.unpack_from("<H", msg.data)[0] == robstride.WATCHDOG:
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
        self.virtual.params[(2, robstride.MODE)] = 1
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
            bound = robstride.LIMITS[self.bus.models[motor - 1]].torque
            torque = ((msg.arbitration_id >> 8) & 65535) / 65535 * 2 * bound - bound
            self.assertAlmostEqual(torque, 1.0, delta=.002)
            self.assertGreater(struct.unpack(">HHHH", msg.data)[2], 0)


if __name__ == "__main__":
    unittest.main()
