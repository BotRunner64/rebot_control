"""Shared Three.js scene export, read-only commands, and preview lifecycle."""

import asyncio
from contextlib import redirect_stdout
import io
import json
import sys
import unittest
from unittest.mock import patch

import mujoco
import numpy as np

import rebot_mirror
import stararm_teleop
from rebot_viewer import SceneViewer, ViewerServer, model_meshes
from test_rebot_ee_viewer import MemoryServer, Socket


class SceneViewerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = mujoco.MjModel.from_xml_string('''
            <mujoco><asset><mesh name="tetra" vertex="0 0 0 .1 0 0 0 .1 0 0 0 .1"
                face="0 2 1 0 1 3 0 3 2 1 2 3"/></asset><worldbody>
                <geom type="box" size=".3 .65 .005" pos=".6 0 .745"/>
                <body><joint type="hinge"/><geom type="mesh" mesh="tetra"/></body>
            </worldbody></mujoco>''')

    def setUp(self):
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)
        with redirect_stdout(io.StringIO()), patch.dict(sys.modules, {"mujoco.viewer": None}):
            self.viewer = SceneViewer(self.model, self.data, "rebot", open_browser=False,
                                      server_factory=MemoryServer)
        self.addCleanup(self.viewer.close)

    def test_backend_mesh_and_table_are_exported_with_matching_render_frames(self):
        packet = json.loads(self.viewer.model_json)
        self.assertTrue(packet["read_only"])
        self.assertNotIn("limits", packet)
        self.assertNotIn("kp", packet)
        box, mesh = packet["meshes"]
        np.testing.assert_allclose(box["size"], [.6, 1.3, .01])
        self.assertGreater(len(mesh["vertices"]), 0)
        self.assertGreater(len(mesh["faces"]), 0)
        snapshot = self.viewer.snapshot()
        np.testing.assert_allclose(snapshot["transforms"][0]["position"], [.6, 0, .745])
        self.assertEqual(snapshot["transforms"][1]["id"], mesh["id"])
        self.assertEqual(len(model_meshes(self.model, include_boxes=False)), 1)

    def test_sync_publishes_copies_without_web_thread_accessing_backend(self):
        previous = self.viewer.snapshot()
        self.data.qpos[0] = .4
        mujoco.mj_forward(self.model, self.data)
        self.assertEqual(self.viewer.snapshot()["transforms"], previous["transforms"])
        self.viewer.sync()
        self.assertNotEqual(self.viewer.snapshot()["transforms"], previous["transforms"])

    def test_read_only_commands_reject_motion_and_accept_explicit_stop(self):
        self.viewer.receive({"type": "ready"}, "browser")
        self.assertIn("browser", self.viewer.clients)
        for command in (None, {"type": "enable"}, {"type": "pose"}, {"type": "joints"}, {"type": "gains"}):
            with self.subTest(command=command), self.assertRaises(ValueError):
                self.viewer.receive(command, "browser")
        self.assertTrue(self.viewer.is_running())
        self.viewer.receive({"type": "stop"}, "browser")
        self.assertTrue(self.viewer.snapshot()["stopped"])
        self.assertFalse(self.viewer.is_running())

    def test_disconnect_and_reconnect_keep_pose_display_running(self):
        server = ViewerServer.__new__(ViewerServer)
        server.viewer = self.viewer
        for _ in range(2):
            socket = Socket([json.dumps({"type": "ready"}), "bad-json", json.dumps({"type": "enable"})])
            asyncio.run(server._connection(socket))
            self.assertEqual(sum(packet["type"] == "error" for packet in socket.sent), 2)
            self.assertFalse(self.viewer.clients)
            self.assertTrue(self.viewer.is_running())

    def test_browser_open_failure_closes_the_server(self):
        servers = []

        def factory(*args):
            server = MemoryServer(*args)
            servers.append(server)
            return server

        with patch("rebot_viewer.webbrowser.open", side_effect=OSError("No browser")), \
                redirect_stdout(io.StringIO()), self.assertRaisesRegex(OSError, "No browser"):
            SceneViewer(self.model, self.data, server_factory=factory)
        self.assertTrue(servers[0].stopped)

    def test_close_stops_transport(self):
        self.viewer.close()
        self.assertTrue(self.viewer.server.stopped)
        self.assertFalse(self.viewer.is_running())


class DualArmSceneTests(unittest.TestCase):
    def test_both_model_types_use_threejs_without_importing_native_viewer(self):
        for target in ("stararm", "rebot"):
            with self.subTest(target=target), patch.dict(sys.modules, {"mujoco.viewer": None}):
                model, data = stararm_teleop.load_model(target)
                angles = {side: [0., 45., 75., 0., 20., 0., 10.] for side in stararm_teleop.SIDES}
                stararm_teleop.set_model_pose(model, data, angles, target)
                with patch("rebot_viewer.ViewerServer", MemoryServer), redirect_stdout(io.StringIO()):
                    with stararm_teleop.open_viewer(model, data, target, open_browser=False) as viewer:
                        self.assertIsInstance(viewer, SceneViewer)
                        self.assertTrue(viewer.is_running())
                        packet = json.loads(viewer.model_json)
                        self.assertTrue(packet["meshes"])
                        self.assertEqual(len(viewer.snapshot()["transforms"]), len(packet["meshes"]))
                        if target == "rebot":
                            tables = [mesh for mesh in packet["meshes"] if "size" in mesh]
                            self.assertEqual(len(tables), 1)
                            np.testing.assert_allclose(tables[0]["size"], [.595, 1.3, .01])
                    self.assertTrue(viewer.server.stopped)

    def test_invalid_ports_fail_before_hardware_bindings_are_read(self):
        for port in (0, 65536, True, 1.5):
            for module, entrypoint in ((stararm_teleop, stararm_teleop.teleop),
                                       (rebot_mirror, rebot_mirror.mirror)):
                with self.subTest(port=port, entrypoint=entrypoint), patch.object(module, "load_config") as load:
                    with self.assertRaisesRegex(ValueError, "port"):
                        entrypoint(port=port)
                    load.assert_not_called()


if __name__ == "__main__":
    unittest.main()
