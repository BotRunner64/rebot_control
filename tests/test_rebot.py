"""Exercise read-only CAN transactions and fail-closed setup persistence."""

from contextlib import contextmanager, redirect_stdout
import io
import math
from pathlib import Path
import struct
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch

import can

import rebot


def frame(kind, motor=1, host=rebot.HOST_ID, data=bytes(8), **kwargs):
    return can.Message(arbitration_id=(kind << 24) | (motor << 8) | host,
                       is_extended_id=True, data=data, timestamp=time.time(), **kwargs)


class QueryBus:
    def __init__(self, responses):
        self.responses = responses
        self.queue = []
        self.sent = []
        self.channel_handler = self

    def transmit(self, *args):
        self.sent.append(args)
        self.queue.extend(self.responses())

    def recv(self, timeout):
        return self.queue.pop(0) if self.queue else None


class ReaderTests(unittest.TestCase):
    def test_scan_accepts_fixed_fe_device_id_reply_address(self):
        bus = QueryBus(lambda: [frame(0, motor=bus.sent[-1][2], host=0xFE,
                                    data=bus.sent[-1][2].to_bytes(8, "little"))])
        reader = rebot.Reader(bus)
        self.assertEqual(reader.scan(), {i: i.to_bytes(8, "little").hex() for i in range(1, 8)})
        self.assertEqual([call[:3] for call in bus.sent], [(0, 0xFD, i) for i in range(1, 8)])
        self.assertEqual(reader.received_frames, 7)

    def test_device_id_reply_still_requires_matching_motor_and_reply_address(self):
        uid = b"12345678"
        bus = QueryBus(lambda: [frame(0, motor=2, host=0xFE, data=uid),
                               frame(0, host=0xAA, data=uid), frame(0, host=0xFE, data=uid)])
        self.assertEqual(rebot.Reader(bus).query(0, 1), uid)

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
        reader = rebot.Reader(bus)
        result = reader.query(17, 1, struct.pack("<HHI", parameter, 0, 0), parameter)
        self.assertEqual(result, payload)
        self.assertEqual(bus.sent, [(17, 0xFD, 1, struct.pack("<HHI", parameter, 0, 0))])

    def test_missing_response_times_out(self):
        with self.assertRaises(TimeoutError):
            rebot.Reader(QueryBus(lambda: [])).query(0, 1)

    def test_fault_report_aborts(self):
        bus = QueryBus(lambda: [frame(21, data=b"\x01" + bytes(7))])
        with self.assertRaisesRegex(RuntimeError, "fault/warning"):
            rebot.Reader(bus).query(17, 1)

    def test_positions_only_send_read_requests_and_keep_continuous_angles(self):
        bus = QueryBus(lambda: [frame(17, motor=bus.sent[-1][2],
                                    data=struct.pack("<HHf", 0x7019, 0, 7.0))])
        self.assertEqual(rebot.Reader(bus).positions(), [7.0] * 7)
        self.assertEqual([call[:3] for call in bus.sent], [(17, 0xFD, i) for i in range(1, 8)])

    def test_nonfinite_position_is_rejected(self):
        bus = QueryBus(lambda: [frame(17, data=struct.pack("<HHf", 0x7019, 0, math.nan))])
        with self.assertRaisesRegex(ValueError, "invalid position"):
            rebot.Reader(bus).positions()

    def test_shutdown_does_not_disable_motors_even_after_error(self):
        sdk = MagicMock(is_connected=True)
        with patch.object(rebot, "check_channel"), \
                patch.object(rebot, "RobstrideBus", return_value=sdk):
            with self.assertRaises(RuntimeError):
                with rebot.open_reader("can0"):
                    raise RuntimeError("Read failed")
        sdk.connect.assert_called_once_with(handshake=False)
        sdk.disconnect.assert_called_once_with(disable_torque=False)
        sdk.transmit.assert_not_called()


class SetupTests(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.config = Path(temp.name) / "rebot.local.yaml"
        self.uids = {i: f"{i:016x}" for i in rebot.MOTOR_IDS}
        self.reader = MagicMock()
        self.reader.scan.return_value = self.uids
        self.reader.positions.return_value = [0.25] * 7

        @contextmanager
        def opened(channel):
            yield self.reader

        self.patch(rebot, "open_reader", opened)
        self.patch(rebot, "device_path", lambda name: "/usb/" + name)
        self.patch(rebot.time, "sleep", lambda seconds: None)
        self.output = redirect_stdout(io.StringIO())
        self.output.__enter__()
        self.addCleanup(self.output.__exit__, None, None, None)

    def patch(self, obj, name, value):
        patcher = patch.object(obj, name, value)
        patcher.start()
        self.addCleanup(patcher.stop)

    def bind(self, side="left", channel="can0", replace=False):
        rebot.bind(side, channel, replace, self.config)

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
        self.reader.scan.return_value = dict.fromkeys(rebot.MOTOR_IDS, "01" * 8)
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
        arms = rebot.load_config(self.config)
        arms["left"]["reference_rad"] = [1.0] * 7
        rebot.save_config(arms, self.config)
        with self.assertRaisesRegex(ValueError, "--replace"):
            self.bind()
        self.bind(channel="can1", replace=True)
        self.assertNotIn("reference_rad", rebot.load_config(self.config)["left"])

    def test_missing_other_adapter_does_not_prevent_binding(self):
        self.bind()
        self.reader.scan.return_value = {i: f"{i + 100:016x}" for i in rebot.MOTOR_IDS}
        self.bind("right", "can1")
        self.assertEqual(set(rebot.load_config(self.config)), {"left", "right"})

    def test_moving_arm_is_not_calibrated(self):
        self.bind()
        original = self.config.read_bytes()
        self.reader.positions.side_effect = [[i * 0.1] * 7 for i in range(10)]
        with patch.object(rebot, "resolve_channel", return_value="can0"):
            with self.assertRaisesRegex(ValueError, "moved"):
                rebot.calibrate("left", config=self.config)
        self.assertEqual(self.config.read_bytes(), original)

    def test_calibration_preserves_continuous_angles_and_requires_replace(self):
        self.bind()
        self.reader.positions.return_value = [7.0] * 7
        with patch.object(rebot, "resolve_channel", return_value="can0"):
            rebot.calibrate("left", config=self.config)
            self.assertEqual(rebot.load_config(self.config)["left"]["reference_rad"], [7.0] * 7)
            with self.assertRaisesRegex(ValueError, "--replace"):
                rebot.calibrate("left", config=self.config)

    def test_changed_identity_aborts_without_saving(self):
        self.bind()
        original = self.config.read_bytes()
        self.reader.scan.return_value = {**self.uids, 7: "ff" * 8}
        with patch.object(rebot, "resolve_channel", return_value="can0"):
            with self.assertRaisesRegex(ValueError, "UIDs"):
                rebot.calibrate("left", config=self.config)
        self.assertEqual(self.config.read_bytes(), original)

    def test_gripper_only_calibration_preserves_six_axes_and_binding(self):
        self.bind()
        arm = rebot.load_config(self.config)["left"]
        arm["reference_rad"] = [.1, .2, .3, .4, .5, .6, math.radians(.42)]
        rebot.save_config({"left": arm}, self.config)
        self.reader.position.return_value = math.radians(-10.17)
        self.reader.positions.reset_mock()
        with patch.object(rebot, "resolve_channel", return_value="can0"):
            rebot.calibrate("left", replace=True, config=self.config, gripper_only=True)
        updated = rebot.load_config(self.config)["left"]
        expected = dict(arm, reference_rad=arm["reference_rad"][:6] + [math.radians(-10.17)])
        self.assertEqual(updated, expected)
        self.assertEqual(self.reader.position.call_count, 10)
        self.assertTrue(all(call.args == (7,) for call in self.reader.position.call_args_list))
        self.reader.positions.assert_not_called()

    def test_gripper_only_requires_existing_references_and_explicit_replace(self):
        self.bind()
        with self.assertRaisesRegex(ValueError, "complete arm"):
            rebot.calibrate("left", config=self.config, gripper_only=True)
        arm = rebot.load_config(self.config)["left"]
        arm["reference_rad"] = [0.] * 7
        rebot.save_config({"left": arm}, self.config)
        with self.assertRaisesRegex(ValueError, "--replace"):
            rebot.calibrate("left", config=self.config, gripper_only=True)

    def test_moving_gripper_does_not_overwrite_existing_references(self):
        self.bind()
        arm = rebot.load_config(self.config)["left"]
        arm["reference_rad"] = [0.] * 7
        rebot.save_config({"left": arm}, self.config)
        original = self.config.read_bytes()
        self.reader.position.side_effect = [i * .02 for i in range(10)]
        with patch.object(rebot, "resolve_channel", return_value="can0"):
            with self.assertRaisesRegex(ValueError, "gripper moved"):
                rebot.calibrate("left", replace=True, config=self.config, gripper_only=True)
        self.assertEqual(self.config.read_bytes(), original)

    def test_usb_path_resolves_after_can_interface_renumbering(self):
        self.bind()
        arm = rebot.load_config(self.config)["left"]
        with patch.object(rebot, "channels", return_value=["can1"]), \
                patch.object(rebot, "device_path", return_value=arm["device_path"]):
            self.assertEqual(rebot.resolve_channel(arm), "can1")

    def test_binding_change_during_calibration_is_not_overwritten(self):
        self.bind()

        def capture(reader):
            self.bind(channel="can1", replace=True)
            return [1.0] * 7

        with patch.object(rebot, "resolve_channel", return_value="can0"), \
                patch.object(rebot, "capture_pose", side_effect=capture):
            with self.assertRaisesRegex(ValueError, "binding changed"):
                rebot.calibrate("left", config=self.config)
        self.assertEqual(rebot.load_config(self.config)["left"]["channel"], "can1")
        self.assertNotIn("reference_rad", rebot.load_config(self.config)["left"])


class ChannelTests(unittest.TestCase):
    @staticmethod
    def link_result(up=True, bitrate=rebot.BITRATE):
        entry = dict(flags=["UP"] if up else [], linkinfo=dict(info_kind="can",
                     info_data=dict(state="ERROR-ACTIVE" if up else "STOPPED",
                                    bittiming=dict(bitrate=bitrate))))
        return SimpleNamespace(stdout=rebot.json.dumps([entry]))

    def test_ready_interface_is_not_reconfigured(self):
        with patch.object(rebot, "channels", return_value=["can0"]), \
                patch.object(rebot.subprocess, "run", return_value=self.link_result()) as run:
            rebot.check_channel("can0", configure=True)
        self.assertEqual(run.call_count, 1)
        self.assertEqual(run.call_args.args[0],
                         ["ip", "-details", "-json", "link", "show", "dev", "can0"])

    def test_unready_interface_is_configured_and_verified(self):
        for initial in (self.link_result(up=False), self.link_result(bitrate=500000)):
            for uid in (0, 1000):
                with self.subTest(initial=initial, uid=uid), \
                        patch.object(rebot, "channels", return_value=["can0"]), \
                        patch.object(rebot.os, "geteuid", return_value=uid), \
                        patch.object(rebot.subprocess, "run", side_effect=[
                            initial, None, None, None, self.link_result()]) as run, \
                        redirect_stdout(io.StringIO()):
                    rebot.check_channel("can0", configure=True)
                    prefix = ["sudo"] if uid else []
                    self.assertEqual([call.args[0] for call in run.call_args_list[1:4]], [
                        prefix + ["ip", "link", "set", "can0", "down"],
                        prefix + ["ip", "link", "set", "can0", "type", "can", "bitrate", "1000000"],
                        prefix + ["ip", "link", "set", "can0", "up"]])
                    self.assertEqual(run.call_count, 5)

    def test_failed_sudo_stops_configuration(self):
        with patch.object(rebot, "channels", return_value=["can0"]), \
                patch.object(rebot.os, "geteuid", return_value=1000), \
                patch.object(rebot.subprocess, "run", side_effect=[self.link_result(up=False),
                    rebot.subprocess.CalledProcessError(1, ["sudo", "ip"])]) as run, \
                redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(RuntimeError, "sudo access"):
                rebot.check_channel("can0", configure=True)
        self.assertEqual(run.call_count, 2)

    def test_scan_does_not_open_can_when_configuration_fails(self):
        with patch.object(rebot.sys, "argv", ["rebot.py", "scan", "--channel", "can0"]), \
                patch.object(rebot, "check_channel", side_effect=RuntimeError("Setup failed")) as check, \
                patch.object(rebot, "open_reader") as opened, \
                patch.object(rebot.sys, "stderr", io.StringIO()):
            self.assertEqual(rebot.main(), 1)
        check.assert_called_once_with("can0", configure=True)
        opened.assert_not_called()

    def test_configuration_is_rechecked_before_scanning(self):
        with patch.object(rebot, "channels", return_value=["can0"]), \
                patch.object(rebot.subprocess, "run", side_effect=[
                    self.link_result(up=False), None, None, None, self.link_result(up=False)]), \
                redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(ValueError, "must be UP"):
                rebot.check_channel("can0", configure=True)

    def test_down_and_wrong_bitrate_rejected(self):
        for entry in [dict(flags=[], linkinfo={}), dict(flags=["UP"], linkinfo=dict(
                info_kind="can", info_data=dict(bittiming=dict(bitrate=500000))))]:
            with self.subTest(entry=entry), patch.object(rebot, "channels", return_value=["can0"]), \
                    patch.object(rebot.subprocess, "run", return_value=SimpleNamespace(
                        stdout=rebot.json.dumps([entry]))):
                with self.assertRaisesRegex(ValueError, "1000000"):
                    rebot.check_channel("can0")


class DiagnosticsTests(unittest.TestCase):
    def test_empty_scan_reports_zero_traffic_and_kernel_counters(self):
        entry = dict(linkinfo=dict(info_data=dict(state="ERROR-ACTIVE",
                                                berr_counter=dict(tx=0, rx=0))),
                     stats64=dict(rx=dict(packets=0), tx=dict(packets=7)))
        output = io.StringIO()
        with patch.object(rebot.subprocess, "run", return_value=SimpleNamespace(
                stdout=rebot.json.dumps([entry]))), patch.object(rebot.sys, "stderr", output):
            rebot.report_empty_scan("can0", 0)
        self.assertIn("No motor replies received", output.getvalue())
        self.assertIn("total RX packets: 0", output.getvalue())
        self.assertIn("total TX packets: 7", output.getvalue())

    def test_unmatched_traffic_is_distinguished_from_no_traffic(self):
        output = io.StringIO()
        with patch.object(rebot.subprocess, "run", side_effect=OSError("ip unavailable")), \
                patch.object(rebot.sys, "stderr", output):
            rebot.report_empty_scan("can0", 3)
        self.assertIn("Traffic was received", output.getvalue())
        self.assertNotIn("No motor replies received", output.getvalue())
        self.assertIn("Could not read CAN statistics", output.getvalue())


if __name__ == "__main__":
    unittest.main()
