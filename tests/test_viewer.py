"""Shared Three.js scene export, read-only commands, and preview lifecycle."""

from contextlib import redirect_stdout
import io
import json
import sys
import unittest
from unittest.mock import patch

import mujoco
import numpy as np

from rebot.bindings import SIDES
from rebot.viewer.dual import load_dual_model, pose_from_leaders
from rebot.viewer.scene import SceneViewer
from test_ee_viewer import MemoryServer


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


class DualArmSceneTests(unittest.TestCase):
    def test_both_model_types_use_threejs_without_importing_native_viewer(self):
        for target in ("stararm", "rebot"):
            with self.subTest(target=target), patch.dict(sys.modules, {"mujoco.viewer": None}):
                model, data = load_dual_model(target)
                angles = {side: [0., 45., 75., 0., 20., 0., 10.] for side in SIDES}
                pose_from_leaders(model, data, angles, target)
                with patch("rebot.viewer.server.ViewerServer", MemoryServer), redirect_stdout(io.StringIO()):
                    with SceneViewer(model, data, target, open_browser=False) as viewer:
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


if __name__ == "__main__":
    unittest.main()
