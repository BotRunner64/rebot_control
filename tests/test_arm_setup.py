"""Exercise read-only CAN transactions and fail-closed setup persistence."""

from contextlib import contextmanager, redirect_stderr, redirect_stdout
import io
import json
import math
from pathlib import Path
import struct
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch

import can
import yaml

from fakes import VirtualMotors, patch_hardware
from rebot import bindings
from rebot.apps import arm_setup
from rebot.devices import robstride, socketcan
from rebot.devices.robstride import HOST_ID, MOTOR_IDS

READ_ONLY_KINDS = {robstride.GET_DEVICE_ID, robstride.READ_PARAMETER}


def frame(kind, motor=1, host=HOST_ID, data=bytes(8), **kwargs):
    return can.Message(arbitration_id=(kind << 24) | (motor << 8) | host,
                       is_extended_id=True, data=data, timestamp=time.time(), **kwargs)


def sent_kinds(virtual):
    return {(message.arbitration_id >> 24) & 31 for message in virtual.sent}


class QueryBus:
    """Stands in for `can.Bus`: every transmitted frame queues `responses()`."""

    def __init__(self, responses):
        self.responses = responses
        self.queue = []
        self.sent = []
        self.closed = False

    def send(self, message, timeout):
        self.sent.append(message)
        self.queue.extend(self.responses())

    def recv(self, timeout):
        return self.queue.pop(0) if self.queue else None

    def shutdown(self):
        self.closed = True

    def motor(self):
        return self.sent[-1].arbitration_id & 255

    def requests(self):
        """(type, host, motor) of each transmitted frame."""
        return [(m.arbitration_id >> 24, (m.arbitration_id >> 8) & 0xFFFF, m.arbitration_id & 255)
                for m in self.sent]


def open_reader(bus):
    with patch.object(robstride, "check_channel"), \
            patch.object(robstride.can, "Bus", return_value=bus):
        return robstride.open_reader("can0")


def query(reader, kind, motor, payload=bytes(8), parameter=None):
    return reader.transaction(motor, kind, lambda: reader.send(kind, motor, payload), parameter)


class ReaderTests(unittest.TestCase):
    """The read-only path: `open_reader` is a seven-motor `MotorBus` with a 0.3 s timeout."""

    def test_scan_accepts_fixed_fe_device_id_reply_address(self):
        bus = QueryBus(lambda: [frame(0, motor=bus.motor(), host=0xFE,
                                      data=bus.motor().to_bytes(8, "little"))])
        reader = open_reader(bus)
        self.assertEqual(reader.scan(strict=False),
                         {i: i.to_bytes(8, "little").hex() for i in range(1, 8)})
        self.assertEqual(bus.requests(), [(0, 0xFD, i) for i in range(1, 8)])
        self.assertEqual(reader.received_frames, 7)

    def test_device_id_reply_still_requires_matching_motor_and_reply_address(self):
        uid = b"12345678"
        bus = QueryBus(lambda: [frame(0, motor=2, host=0xFE, data=uid),
                                frame(0, host=0xAA, data=uid), frame(0, host=0xFE, data=uid)])
        self.assertEqual(query(open_reader(bus), 0, 1), uid)
        # The requesting host ID is not a valid device-ID reply address.
        bus = QueryBus(lambda: [frame(0, host=HOST_ID, data=uid)])
        with self.assertRaises(TimeoutError):
            query(open_reader(bus), 0, 1)

    def test_ignores_wrong_host_motor_kind_parameter_and_old_traffic(self):
        parameter = 0x7019
        payload = struct.pack("<HHf", parameter, 0, 1.25)

        def replies():
            stale = frame(17, data=payload)
            stale.timestamp -= 10
            remote = frame(17, data=payload, is_remote_frame=True)
            standard = frame(17, data=payload)
            standard.is_extended_id = False
            return [frame(17, motor=2, data=payload), frame(17, host=0xFF, data=payload),
                    frame(2), frame(17, data=struct.pack("<HHf", 0x701B, 0, 9)),
                    stale, remote, standard, frame(17, data=bytes(4)), frame(17, data=payload)]

        bus = QueryBus(replies)
        reader = open_reader(bus)
        request = struct.pack("<HHI", parameter, 0, 0)
        self.assertEqual(query(reader, 17, 1, request, parameter), payload)
        self.assertEqual(bus.requests(), [(17, 0xFD, 1)])
        self.assertEqual(bytes(bus.sent[0].data), request)
        # The public position read correlates the same way.
        bus = QueryBus(replies)
        self.assertEqual(open_reader(bus).position(1), 1.25)
        self.assertEqual(bus.requests(), [(17, 0xFD, 1)])
        self.assertEqual(bytes(bus.sent[0].data), request)

    def test_fault_report_aborts(self):
        bus = QueryBus(lambda: [frame(21, data=b"\x01" + bytes(7))])
        with self.assertRaisesRegex(RuntimeError, "fault/warning"):
            query(open_reader(bus), 17, 1)
        with self.assertRaisesRegex(RuntimeError, "fault/warning"):
            open_reader(QueryBus(lambda: [frame(21, data=b"\x01" + bytes(7))])).positions()

    def test_overloaded_receive_queue_aborts_before_transmitting(self):
        bus = QueryBus(lambda: [])
        bus.recv = lambda timeout: frame(2, motor=3)
        with self.assertRaisesRegex(RuntimeError, "overloaded"):
            open_reader(bus).position(1)
        self.assertEqual(bus.sent, [])

    def test_nonfinite_position_is_rejected_when_every_motor_replies(self):
        for value in (math.nan, math.inf):
            bus = QueryBus(lambda: [frame(17, motor=bus.motor(),
                                          data=struct.pack("<HHf", 0x7019, 0, value))])
            with self.assertRaisesRegex(ValueError, "invalid mechanical position"):
                open_reader(bus).positions()
            bus = QueryBus(lambda: [frame(17, data=struct.pack("<HHf", 0x7019, 0, value))])
            with self.assertRaisesRegex(ValueError, "invalid mechanical position"):
                open_reader(bus).position(1)

    def test_shutdown_after_queries_and_error_sends_only_queries(self):
        virtual = VirtualMotors()
        patch_hardware(self, {"can0": virtual})
        with self.assertRaises(RuntimeError):
            with robstride.open_reader("can0") as reader:
                self.assertEqual(reader.scan(strict=False), virtual.uids)
                reader.positions()
                raise RuntimeError("Read failed")
        self.assertTrue(virtual.closed)
        self.assertEqual(sent_kinds(virtual), READ_ONLY_KINDS)
        self.assertEqual(len(virtual.sent), 14)


class SetupTests(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.config = Path(temp.name) / "arm.local.yaml"
        self.uids = {i: f"{i:016x}" for i in MOTOR_IDS}
        self.reader = MagicMock()
        self.reader.scan.return_value = self.uids
        self.reader.positions.return_value = [0.25] * 7
        self.opened = []

        @contextmanager
        def opened(channel):
            self.opened.append(channel)
            yield self.reader

        self.patch(arm_setup, "open_reader", opened)
        self.check_channel = MagicMock()
        self.patch(socketcan, "check_channel", self.check_channel)
        self.patch(socketcan, "device_path", lambda name: "/usb/" + name)
        self.patch(arm_setup.time, "sleep", lambda seconds: None)
        self.output = redirect_stdout(io.StringIO())
        self.output.__enter__()
        self.addCleanup(self.output.__exit__, None, None, None)

    def patch(self, obj, name, value):
        patcher = patch.object(obj, name, value)
        patcher.start()
        self.addCleanup(patcher.stop)

    def bind(self, side="left", channel="can0", replace=False):
        arm_setup.bind(side, channel, replace, self.config)

    def test_binding_requires_all_seven_ids_and_valid_positions(self):
        self.reader.scan.return_value = {1: self.uids[1]}
        with self.assertRaisesRegex(ValueError, "Missing motor IDs"):
            self.bind()
        self.assertFalse(self.config.exists())
        self.reader.scan.return_value = self.uids
        self.reader.positions.side_effect = TimeoutError("Missing feedback")
        with self.assertRaises(TimeoutError):
            self.bind()
        self.assertFalse(self.config.exists())

    def test_duplicate_motor_uids_cannot_bind(self):
        self.reader.scan.return_value = dict.fromkeys(MOTOR_IDS, "01" * 8)
        with self.assertRaisesRegex(ValueError, "not unique"):
            self.bind()
        self.assertFalse(self.config.exists())

    def test_same_arm_cannot_be_bound_to_both_sides(self):
        self.bind()
        original = self.config.read_bytes()
        with self.assertRaisesRegex(ValueError, "already bound to left"):
            self.bind("right", "can1")
        self.assertEqual(self.config.read_bytes(), original)

    def test_rebinding_requires_replace_and_drops_reference(self):
        self.bind()
        arms = bindings.load_arms(self.config)
        arms["left"]["reference_rad"] = [1.0] * 7
        bindings.save_arms(arms, self.config)
        with self.assertRaisesRegex(ValueError, "--replace"):
            self.bind()
        self.bind(channel="can1", replace=True)
        self.assertNotIn("reference_rad", bindings.load_arms(self.config)["left"])

    def test_moving_arm_is_not_calibrated(self):
        self.bind()
        original = self.config.read_bytes()
        self.reader.positions.side_effect = [[i * 0.1] * 7 for i in range(10)]
        with patch.object(bindings, "resolve_channel", return_value="can0"):
            with self.assertRaisesRegex(ValueError, "moved"):
                arm_setup.calibrate("left", config=self.config)
        self.assertEqual(self.config.read_bytes(), original)

    def test_calibration_preserves_continuous_angles_and_requires_replace(self):
        self.bind()
        self.reader.positions.return_value = [7.0] * 7
        with patch.object(bindings, "resolve_channel", return_value="can0"):
            arm_setup.calibrate("left", config=self.config)
            self.assertEqual(bindings.load_arms(self.config)["left"]["reference_rad"], [7.0] * 7)
            with self.assertRaisesRegex(ValueError, "--replace"):
                arm_setup.calibrate("left", config=self.config)

    def test_changed_identity_aborts_without_saving(self):
        self.bind()
        original = self.config.read_bytes()
        self.reader.scan.return_value = {**self.uids, 7: "ff" * 8}
        with patch.object(bindings, "resolve_channel", return_value="can0"):
            with self.assertRaisesRegex(ValueError, "UIDs"):
                arm_setup.calibrate("left", config=self.config)
            with self.assertRaisesRegex(ValueError, "UIDs"):
                arm_setup.read("left", config=self.config)
        self.assertEqual(self.config.read_bytes(), original)
        self.reader.positions.assert_called_once()  # only while binding

    def test_gripper_only_calibration_preserves_six_axes_and_binding(self):
        self.bind()
        arm = bindings.load_arms(self.config)["left"]
        arm["reference_rad"] = [.1, .2, .3, .4, .5, .6, math.radians(.42)]
        bindings.save_arms({"left": arm}, self.config)
        self.reader.position.return_value = math.radians(-10.17)
        self.reader.positions.reset_mock()
        with patch.object(bindings, "resolve_channel", return_value="can0"):
            arm_setup.calibrate("left", replace=True, config=self.config, gripper_only=True)
        updated = bindings.load_arms(self.config)["left"]
        expected = dict(arm, reference_rad=arm["reference_rad"][:6] + [math.radians(-10.17)])
        self.assertEqual(updated, expected)
        self.assertEqual(self.reader.position.call_count, 10)
        self.assertTrue(all(call.args == (7,) for call in self.reader.position.call_args_list))
        self.reader.positions.assert_not_called()

    def test_gripper_only_requires_existing_references_and_explicit_replace(self):
        self.bind()
        with self.assertRaisesRegex(ValueError, "complete arm"):
            arm_setup.calibrate("left", config=self.config, gripper_only=True)
        arm = bindings.load_arms(self.config)["left"]
        arm["reference_rad"] = [0.] * 7
        bindings.save_arms({"left": arm}, self.config)
        with self.assertRaisesRegex(ValueError, "--replace"):
            arm_setup.calibrate("left", config=self.config, gripper_only=True)

    def test_moving_gripper_does_not_overwrite_existing_references(self):
        self.bind()
        arm = bindings.load_arms(self.config)["left"]
        arm["reference_rad"] = [0.] * 7
        bindings.save_arms({"left": arm}, self.config)
        original = self.config.read_bytes()
        self.reader.position.side_effect = [i * .02 for i in range(10)]
        with patch.object(bindings, "resolve_channel", return_value="can0"):
            with self.assertRaisesRegex(ValueError, "gripper moved"):
                arm_setup.calibrate("left", replace=True, config=self.config, gripper_only=True)
        self.assertEqual(self.config.read_bytes(), original)

    def test_invalid_binding_file_is_rejected(self):
        for content in ("- left\n", "middle: {}\n", "left: {channel: can0}\n", "left: [unclosed\n"):
            with self.subTest(content=content):
                self.config.write_text(content)
                with self.assertRaises((ValueError, yaml.YAMLError)):
                    bindings.load_arms(self.config)

    def test_usb_path_resolves_after_can_interface_renumbering(self):
        self.bind()
        arm = bindings.load_arms(self.config)["left"]
        with patch.object(socketcan, "channels", return_value=["can1"]), \
                patch.object(socketcan, "device_path", return_value=arm["device_path"]):
            self.assertEqual(bindings.resolve_channel(arm), "can1")
            self.assertEqual(bindings.select_arms(self.config, "left", calibrated=False)["left"].channel,
                             "can1")
        with patch.object(socketcan, "channels", return_value=["can0", "can1"]), \
                patch.object(socketcan, "device_path", return_value=arm["device_path"]):
            with self.assertRaisesRegex(ValueError, "missing or ambiguous"):
                bindings.resolve_channel(arm)

    def test_binding_change_during_calibration_is_not_overwritten(self):
        self.bind()

        def capture(read, moved):
            self.bind(channel="can1", replace=True)
            return [1.0] * 7

        with patch.object(bindings, "resolve_channel", return_value="can0"), \
                patch.object(arm_setup, "capture", side_effect=capture):
            with self.assertRaisesRegex(ValueError, "binding changed"):
                arm_setup.calibrate("left", config=self.config)
        self.assertEqual(bindings.load_arms(self.config)["left"]["channel"], "can1")
        self.assertNotIn("reference_rad", bindings.load_arms(self.config)["left"])


class ReadOnlyFrameTests(unittest.TestCase):
    """Setup commands on the real transport: only device-ID and parameter-read frames."""

    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.config = Path(temp.name) / "arm.local.yaml"
        self.virtual = {"can0": VirtualMotors(), "can1": VirtualMotors(100)}
        patch_hardware(self, self.virtual)
        for target, name, value in (
                (socketcan, "check_channel", MagicMock()),
                (socketcan, "channels", lambda: ["can0", "can1"]),
                (socketcan, "device_path", lambda name: "/usb/" + name),
                (arm_setup.time, "sleep", lambda seconds: None)):
            patcher = patch.object(target, name, value)
            patcher.start()
            self.addCleanup(patcher.stop)
        for context in (redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO())):
            context.__enter__()
            self.addCleanup(context.__exit__, None, None, None)

    def assert_read_only(self, closed=True):
        for motors in self.virtual.values():
            self.assertLessEqual(sent_kinds(motors), READ_ONLY_KINDS)
            self.assertEqual(set(motors.modes.values()), {0})
            self.assertTrue(all(value == 4000 for (_, parameter), value in motors.params.items()
                                if parameter == robstride.WATCHDOG))
        self.assertEqual({motors.closed for motors in self.virtual.values() if motors.sent}, {closed})

    def test_scan_bind_calibrate_and_read_never_enable_disable_command_or_write(self):
        self.assertTrue(arm_setup.scan())
        arm_setup.bind("left", "can0", config=self.config)
        arm_setup.bind("right", "can1", config=self.config)
        arm_setup.calibrate("left", config=self.config)
        arm_setup.calibrate("left", replace=True, config=self.config, gripper_only=True)
        arm_setup.read("left", config=self.config)
        arms = bindings.load_arms(self.config)
        self.assertEqual(arms["left"]["motor_uids"], self.virtual["can0"].uids)
        self.assertEqual(arms["right"]["motor_uids"], self.virtual["can1"].uids)
        self.assertEqual(arms["left"]["reference_rad"], [struct.unpack("<f", struct.pack("<f", .2))[0]] * 7)
        self.assert_read_only()


class ChannelTests(unittest.TestCase):
    @staticmethod
    def link_result(up=True, bitrate=socketcan.BITRATE):
        entry = dict(flags=["UP"] if up else [], linkinfo=dict(info_kind="can",
                     info_data=dict(state="ERROR-ACTIVE" if up else "STOPPED",
                                    bittiming=dict(bitrate=bitrate))))
        return SimpleNamespace(stdout=json.dumps([entry]))

    def test_unready_interface_is_configured_and_verified(self):
        for initial in (self.link_result(up=False), self.link_result(bitrate=500000)):
            for uid in (0, 1000):
                with self.subTest(initial=initial, uid=uid), \
                        patch.object(socketcan, "channels", return_value=["can0"]), \
                        patch.object(socketcan.os, "geteuid", return_value=uid), \
                        patch.object(socketcan.subprocess, "run", side_effect=[
                            initial, None, None, None, self.link_result()]) as run, \
                        redirect_stdout(io.StringIO()):
                    socketcan.check_channel("can0", configure=True)
                    prefix = ["sudo"] if uid else []
                    self.assertEqual([call.args[0] for call in run.call_args_list[1:4]], [
                        prefix + ["ip", "link", "set", "can0", "down"],
                        prefix + ["ip", "link", "set", "can0", "type", "can", "bitrate", "1000000"],
                        prefix + ["ip", "link", "set", "can0", "up"]])
                    self.assertEqual(run.call_count, 5)

    def test_down_and_wrong_bitrate_rejected(self):
        for entry in [dict(flags=[], linkinfo={}), dict(flags=["UP"], linkinfo=dict(
                info_kind="can", info_data=dict(bittiming=dict(bitrate=500000))))]:
            with self.subTest(entry=entry), patch.object(socketcan, "channels", return_value=["can0"]), \
                    patch.object(socketcan.subprocess, "run", return_value=SimpleNamespace(
                        stdout=json.dumps([entry]))) as run:
                with self.assertRaisesRegex(ValueError, "1000000"):
                    socketcan.check_channel("can0")
                self.assertEqual(run.call_count, 1)

    def test_unready_channel_fails_before_the_bus_is_opened(self):
        with patch.object(robstride, "check_channel", side_effect=ValueError("can0 must be UP")), \
                patch.object(robstride.can, "Bus") as bus:
            with self.assertRaisesRegex(ValueError, "must be UP"):
                robstride.open_reader("can0")
        bus.assert_not_called()


if __name__ == "__main__":
    unittest.main()
