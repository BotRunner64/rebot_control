"""Local Three.js viewer; WebSocket handlers only exchange copied control snapshots."""

import json
import threading
import webbrowser

import numpy as np
from scipy.spatial.transform import Rotation

from rebot_ee_runtime import request_stop
from rebot_kinematics import validate_transform
from rebot_viewer import ViewerServer, model_meshes, render_transforms, validate_port


def pose_packet(target):
    return {"position": target[:3, 3].tolist(),
            "quaternion": Rotation.from_matrix(target[:3, :3]).as_quat().tolist()}


class PoseCommands:
    """Latest complete pose and one-shot hold request, shared with the control thread."""

    def __init__(self, target):
        self._lock = threading.Lock()
        self._target = validate_transform(target)
        self._hold = False

    def set_target(self, target):
        with self._lock:
            self._target = validate_transform(target)

    def read_target(self):
        with self._lock:
            return self._target.copy()

    def request_hold(self):
        with self._lock:
            self._hold = True

    def consume_hold(self):
        with self._lock:
            requested, self._hold = self._hold, False
            return requested


class WebViewer:
    """Three.js camera and target interaction for the EE control loop."""

    def __init__(self, ik, stop, *, port=8080, hardware=False, open_browser=True, server_factory=None):
        validate_port(port)
        self.ik, self.stop = ik, stop
        self.commands = PoseCommands(ik.pose())
        self.ready = threading.Event()
        self.control_lock = threading.RLock()
        self.clients = set()
        self.acknowledged = {}
        self.blocked = False
        self.target_version, self.target_source, self.target_seq = 0, "server", 0
        self.camera_version = 0
        self._state = {}
        self._build_model(hardware)
        self.fit_camera()
        self.sync()
        self.server = (ViewerServer if server_factory is None else server_factory)(self, port)
        self.url = f"http://127.0.0.1:{self.server.port}"
        print(f"Three.js web viewer: {self.url}", flush=True)
        try:
            if open_browser:
                webbrowser.open(self.url)
        except Exception:
            self.server.stop()
            raise

    def _build_model(self, hardware):
        meshes = model_meshes(self.ik.model, include_boxes=False)
        self.mesh_ids = [mesh["id"] for mesh in meshes]
        self.model_json = json.dumps({"meshes": meshes, "hardware": hardware},
                                     allow_nan=False).encode()

    def receive(self, message, client):
        """Validate complete messages before publishing; never access IK or CAN here."""
        if not isinstance(message, dict):
            raise ValueError("Expected a command object.")
        action = message.get("type")
        with self.control_lock:
            if action == "ready":
                self.clients.add(client)
                self.ready.set()
            elif action == "pose":
                position = np.asarray(message["position"], dtype=float)
                quaternion = np.asarray(message["quaternion"], dtype=float)
                if (position.shape != (3,) or quaternion.shape != (4,)
                        or not np.all(np.isfinite(position)) or not np.all(np.isfinite(quaternion))
                        or not np.isclose(np.linalg.norm(quaternion), 1., atol=1e-5)):
                    raise ValueError("Expected a finite position and unit quaternion.")
                sequence = message.get("seq")
                if type(sequence) is not int or sequence < 0:
                    raise ValueError("Expected a nonnegative target sequence.")
                target = np.eye(4)
                target[:3, 3] = position
                target[:3, :3] = Rotation.from_quat(quaternion).as_matrix()
                self._set_target(target, client, sequence)
                self.acknowledged[client] = sequence
            elif action == "hold":
                self.commands.request_hold()
            elif action == "stop":
                request_stop(self.stop, f"web STOP & hold button (client {client})")
            else:
                raise ValueError("Unknown viewer command.")

    def _set_target(self, target, source, sequence):
        self.commands.set_target(target)
        self.target_version += 1
        self.target_source, self.target_seq = source, sequence

    def set_target(self, target):
        with self.control_lock:
            self._set_target(validate_transform(target), "server", 0)

    def fit_camera(self):
        points = np.vstack([self.ik.data.geom_xpos[index] for index in self.mesh_ids]
                           + [self.ik.pose(self.ik.data.qpos)[:3, 3], np.zeros(3)])
        center = (points.min(axis=0) + points.max(axis=0)) / 2
        distance = max(.6, 1.5 * float(np.linalg.norm(np.ptp(points, axis=0))))
        direction = np.array([1., -1., .7])
        with self.control_lock:
            self.camera = {"position": (center + direction / np.linalg.norm(direction) * distance).tolist(),
                           "target": center.tolist()}
            self.camera_version += 1

    def sync(self):
        """Called only by the control thread; publish copied render state."""
        data = self.ik.data
        actual = self.ik.pose(data.qpos)
        transforms = render_transforms(data, self.mesh_ids)
        with self.control_lock:
            distance, angle = self.ik.error(self.commands.read_target(), data.qpos)
            self._state = {"transforms": transforms, "actual": pose_packet(actual),
                           "position_error_mm": distance * 1000.,
                           "orientation_error_deg": float(np.degrees(angle)), "blocked": bool(self.blocked)}

    def snapshot(self, client=None):
        with self.control_lock:
            return {"type": "state", **self._state, "target": pose_packet(self.commands.read_target()),
                    "target_version": self.target_version, "target_source": self.target_source,
                    "target_seq": self.target_seq,
                    "ack_seq": self.acknowledged.get(client, 0),
                    "camera": self.camera, "camera_version": self.camera_version,
                    "stopped": self.stop.is_set()}

    def wait_for_client(self):
        while not self.ready.wait(.05):
            if self.stop.is_set():
                return False
        return not self.stop.is_set()

    def read_target(self):
        return self.commands.read_target()

    def consume_hold(self):
        return self.commands.consume_hold()

    def is_running(self):
        return not self.stop.is_set()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.server.stop()
