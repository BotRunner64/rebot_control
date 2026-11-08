"""Three.js protocol, browser lifecycle, and real MIT encoding without physical CAN."""

import asyncio
from contextlib import redirect_stdout
import importlib.util
import io
import json
import os
import pty
import signal
import sys
from types import SimpleNamespace
import threading
import unittest
from unittest.mock import patch

import numpy as np
from scipy.spatial.transform import Rotation

import rebot_ee as ee
from rebot_ee_viewer import PoseCommands, WebViewer, pose_packet
from rebot_viewer import ViewerServer
from rebot_kinematics import CartesianIK, SIM_START, pose_matrix
import test_rebot_ee as hardware_tests


class MemoryServer:
    def __init__(self, viewer, port):
        self.viewer, self.port, self.stopped = viewer, port, False

    def stop(self):
        self.stopped = True


class Socket:
    def __init__(self, messages):
        self.id = "test-client"
        self.messages, self.sent = iter(messages), []

    def __aiter__(self):
        return self

    async def __anext__(self):
        try:
            return next(self.messages)
        except StopIteration:
            raise StopAsyncIteration

    async def send(self, data):
        self.sent.append(json.loads(data))


class CommandTests(unittest.TestCase):
    def test_pose_snapshots_are_copied_and_hold_requests_are_consumed_once(self):
        initial = pose_matrix([.3, .1, .5, 10, 20, 30])
        commands = PoseCommands(initial)
        initial[0, 3] = 99
        snapshot = commands.read_target()
        self.assertAlmostEqual(snapshot[0, 3], .3)
        snapshot[0, 3] = 100
        self.assertAlmostEqual(commands.read_target()[0, 3], .3)
        commands.request_hold()
        self.assertTrue(commands.consume_hold())
        self.assertFalse(commands.consume_hold())
        invalid = np.eye(4)
        invalid[1, 3] = np.nan
        with self.assertRaises(ValueError):
            commands.set_target(invalid)


@unittest.skipUnless(importlib.util.find_spec("mink") and importlib.util.find_spec("quadprog"),
                     "Install the ee dependency extra to test the web viewer")
class ViewerTests(unittest.TestCase):
    def setUp(self):
        self.ik = CartesianIK(SIM_START)
        self.stop = threading.Event()
        with redirect_stdout(io.StringIO()):
            self.viewer = WebViewer(self.ik, self.stop, open_browser=False, server_factory=MemoryServer)
        self.addCleanup(self.viewer.__exit__, None, None, None)

    def send(self, action, **values):
        self.viewer.receive({"type": action, **values}, "browser-1")

    def enable_ik(self):
        self.send("enable", aligned=True)

    def pose(self, target, seq=1):
        self.send("pose", **pose_packet(target), seq=seq)

    def test_pose_protocol_preserves_full_transform_and_acknowledges_latest_sequence(self):
        self.enable_ik()
        target = pose_matrix([.31, .08, .65, 10, 20, 30])
        self.pose(target, seq=15)
        np.testing.assert_allclose(self.viewer.read_target(), target, atol=1e-12)
        packet = self.viewer.snapshot("browser-1")
        self.assertEqual((packet["target_source"], packet["target_seq"]), ("browser-1", 15))
        self.assertEqual(packet["ack_seq"], 15)
        version = packet["target_version"]
        self.viewer.set_target(self.ik.pose())
        packet = self.viewer.snapshot("browser-1")
        self.assertGreater(packet["target_version"], version)
        self.assertEqual(packet["target_source"], "server")
        self.assertEqual(packet["ack_seq"], 15)
        np.testing.assert_allclose(packet["target"]["position"], self.ik.pose()[:3, 3])

    def test_alignment_gate_and_malformed_commands_do_not_change_the_target(self):
        initial = self.viewer.read_target()
        with self.assertRaises(ValueError):
            self.send("enable")
        with self.assertRaises(ValueError):
            self.pose(initial)
        self.enable_ik()
        for message in ({"type": "pose", "position": [1, 2, 3], "quaternion": [0, 0, 0, 0], "seq": 1},
                        {"type": "pose", "position": [float("nan"), 2, 3], "quaternion": [0, 0, 0, 1], "seq": 2},
                        {"type": "pose", **pose_packet(initial), "seq": -1},
                        {"type": "mode", "mode": "invalid"},
                        {"type": "gains", "kp": [50.] * 6, "kd": [6.] * 6},
                        {"type": "joints", "values": [1.] * 6}):
            with self.subTest(message=message), self.assertRaises(ValueError):
                self.viewer.receive(message, "browser-1")
        np.testing.assert_allclose(self.viewer.read_target(), initial)
        self.assertTrue(self.viewer.is_enabled())

    def test_hold_and_stop_are_distinct_and_report_the_stop_client(self):
        self.send("hold")
        self.assertTrue(self.viewer.consume_hold())
        self.assertFalse(self.stop.is_set())
        output = io.StringIO()
        with redirect_stdout(output), ee.stop_signals(self.stop):
            self.send("stop")
        self.assertFalse(self.viewer.is_running())
        self.assertIn("web STOP & hold button (client browser-1)", output.getvalue())

    def test_read_only_gate_holds_position_and_rejects_removed_settings(self):
        target = self.ik.pose()
        target[0, 3] += .05
        self.viewer.set_target(target)
        with redirect_stdout(io.StringIO()):
            result = ee.track(self.ik, target, self.stop, SimpleNamespace(pressed=lambda: False),
                              ee.load_settings(), duration=.025, viewer=self.viewer)
        self.assertFalse(result.reached)
        np.testing.assert_array_equal(self.ik.data.qpos, SIM_START)
        for action in ("gains", "joints", "mode", "align"):
            with self.subTest(action=action), self.assertRaises(ValueError):
                self.send(action)
        self.assertFalse(self.viewer.is_enabled())

    def test_simple_enable_and_pose_keep_native_mit_gains(self):
        hardware = hardware_tests.HardwareTests()
        hardware.setUp()
        target_offset = np.array([.003, 0., 0.])

        def connected_viewer(ik, stop, **kwargs):
            viewer = WebViewer(ik, stop, open_browser=False, server_factory=MemoryServer)
            viewer.receive({"type": "ready"}, "test")
            sync = viewer.sync
            calls = 0

            def advance():
                nonlocal calls
                sync()
                calls += 1
                if calls == 2:
                    viewer.receive({"type": "enable", "aligned": True}, "test")
                elif calls == 3:
                    target = ik.pose(ik.data.qpos)
                    target[:3, 3] += target_offset
                    viewer.receive({"type": "pose", **pose_packet(target), "seq": 1}, "test")
            viewer.sync = advance
            return viewer

        with patch("rebot_ee_viewer.WebViewer", side_effect=connected_viewer):
            hardware.run_control(headless=False, offset=None, duration=.2)
        self.assertTrue(any(np.max(np.abs(command[0] - hardware.commands[0][0])) > .002
                            for command in hardware.commands))
        for command in hardware.commands:
            np.testing.assert_array_equal(command[3], ee.MOTION_KP)
            np.testing.assert_array_equal(command[4], ee.MOTION_KD)
            self.assertGreater(np.linalg.norm(command[2]), 1.)
        hardware.assert_closed_and_holding()

    def test_disconnect_and_bad_packets_do_not_stop_or_enable_the_controller(self):
        server = ViewerServer.__new__(ViewerServer)
        server.viewer = self.viewer
        socket = Socket([json.dumps({"type": "ready"}), "not-json", json.dumps({"type": "enable"})])
        asyncio.run(server._connection(socket))
        self.assertTrue(self.viewer.ready.is_set())
        self.assertFalse(self.viewer.clients)
        self.assertFalse(self.stop.is_set())
        self.assertFalse(self.viewer.enabled)
        self.assertEqual(sum(packet["type"] == "error" for packet in socket.sent), 2)
        self.send("ready")
        self.assertTrue(self.viewer.wait_for_client())

    def test_http_assets_model_and_path_traversal(self):
        server = ViewerServer.__new__(ViewerServer)
        server.viewer = self.viewer
        for path, expected in (("/", b"End-effector control"),
                               ("/viewer.js", b"TransformControls"),
                               ("/vendor/three.module.js", b"three.core.min.js"),
                               ("/model.json", b"meshes")):
            response = server._http(None, SimpleNamespace(path=path))
            self.assertEqual(response.status_code, 200)
            self.assertIn(expected, response.body)
        self.assertEqual(server._http(None, SimpleNamespace(path="/%2e%2e/%2e%2e/rebot.local.yaml")).status_code, 404)
        self.assertIsNone(server._http(None, SimpleNamespace(path="/ws")))
        packet = json.loads(self.viewer.model_json)
        self.assertTrue(packet["meshes"])
        self.assertEqual(set(packet), {"meshes", "hardware"})

    def test_web_mode_ignores_terminal_focus_mouse_and_arrow_reports(self):
        master, slave = pty.openpty()
        self.addCleanup(os.close, master)
        reports = b"\x1b[O\x1b[I\x1b[A\x1b[<0;20;20M q\n"

        def connected_viewer(ik, stop, **kwargs):
            viewer = WebViewer(ik, stop, open_browser=False, server_factory=MemoryServer)
            viewer.receive({"type": "ready"}, "test")
            return viewer

        with os.fdopen(slave, "r") as terminal, patch.object(sys, "stdin", terminal), \
                patch.object(ee, "StopKey", side_effect=AssertionError("Web must not poll terminal keys")), \
                patch("rebot_ee_viewer.WebViewer", side_effect=connected_viewer), \
                redirect_stdout(io.StringIO()):
            os.write(master, reports)
            result = ee.control(duration=.04, open_browser=False)
            self.assertFalse(result.interrupted)
            self.assertEqual(os.read(terminal.fileno(), 128), reports)

    def test_signals_report_their_source(self):
        output = io.StringIO()
        with redirect_stdout(output), ee.stop_signals(self.stop):
            signal.raise_signal(signal.SIGINT)
        self.assertIn("EE stop reason: received SIGINT", output.getvalue())

    def test_unreachable_pose_packets_do_not_stop_the_web_loop(self):
        self.enable_ik()
        target = self.ik.pose()
        target[:3, 3] += [.5, .2, .2]
        target[:3, :3] = Rotation.from_euler("xyz", [20, 10, 30], degrees=True).as_matrix()
        self.pose(target)
        with redirect_stdout(io.StringIO()):
            result = ee.track(self.ik, target, self.stop, SimpleNamespace(pressed=lambda: False),
                              ee.load_settings(), duration=.08, viewer=self.viewer)
        self.assertFalse(result.interrupted)
        self.assertFalse(result.reached)

    def test_hold_request_discards_old_goal_in_control_thread(self):
        goal = self.ik.pose()
        goal[0, 3] += .1
        self.viewer.set_target(goal)
        self.send("hold")
        with redirect_stdout(io.StringIO()):
            result = ee.track(self.ik, goal, self.stop, SimpleNamespace(pressed=lambda: False),
                              ee.load_settings(), duration=.025, viewer=self.viewer)
        self.assertTrue(result.reached)
        np.testing.assert_allclose(self.viewer.read_target(), self.ik.pose(), atol=1e-6)

    def test_cancel_before_browser_connection_never_opens_hardware_can(self):
        hardware = hardware_tests.HardwareTests()
        hardware.setUp()

        def cancelled_viewer(ik, stop, **kwargs):
            viewer = WebViewer(ik, stop, open_browser=False, server_factory=MemoryServer)
            stop.set()
            return viewer

        with patch("rebot_ee_viewer.WebViewer", side_effect=cancelled_viewer):
            result = hardware.run_control(headless=False)
        self.assertTrue(result.interrupted)
        self.assertFalse(hardware.virtual.sent)

    def test_web_hold_uses_physical_feedback_and_real_mit_cleanup(self):
        hardware = hardware_tests.HardwareTests()
        hardware.setUp()

        def connected_viewer(ik, stop, **kwargs):
            viewer = WebViewer(ik, stop, open_browser=False, server_factory=MemoryServer)
            viewer.receive({"type": "ready"}, "test")
            viewer.receive({"type": "hold"}, "test")
            return viewer

        with patch("rebot_ee_viewer.WebViewer", side_effect=connected_viewer):
            result = hardware.run_control(headless=False, offset=[.1, 0., 0.])
        self.assertTrue(result.reached)
        for command in hardware.commands:
            np.testing.assert_allclose(command[0], hardware.reference + hardware_tests.TEST_START, atol=.0004)
        hardware.assert_closed_and_holding()

    def test_hold_discards_pending_startup_goal_before_control_is_started(self):
        hardware = hardware_tests.HardwareTests()
        hardware.setUp()

        def connected_viewer(ik, stop, **kwargs):
            viewer = WebViewer(ik, stop, open_browser=False, server_factory=MemoryServer)
            viewer.receive({"type": "ready"}, "test")
            viewer.receive({"type": "hold"}, "test")
            sync = viewer.sync

            def advance():
                sync()
                viewer.receive({"type": "enable", "aligned": True}, "test")
            viewer.sync = advance
            return viewer

        with patch("rebot_ee_viewer.WebViewer", side_effect=connected_viewer):
            result = hardware.run_control(headless=False, offset=[.1, 0., 0.])
        self.assertTrue(result.reached)
        for command in hardware.commands:
            np.testing.assert_allclose(command[0], hardware.reference + hardware_tests.TEST_START, atol=.0004)
        hardware.assert_closed_and_holding()


if __name__ == "__main__":
    unittest.main()
