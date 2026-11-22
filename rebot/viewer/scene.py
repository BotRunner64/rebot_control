"""Read-only robot scene viewer and the MuJoCo geometry export shared with the EE viewer."""

import json
import threading

import numpy as np
from scipy.spatial.transform import Rotation

from rebot.runtime.console import request_stop
from rebot.viewer.server import start_server, validate_port


def model_meshes(model, *, include_boxes=True):
    """Copy backend geometry once; Three.js owns all rendering and interaction."""
    import mujoco

    meshes = []
    for index in range(model.ngeom):
        kind = model.geom_type[index]
        primitive = kind == mujoco.mjtGeom.mjGEOM_BOX or kind == mujoco.mjtGeom.mjGEOM_CYLINDER
        if kind != mujoco.mjtGeom.mjGEOM_MESH and not (include_boxes and primitive):
            continue
        material = model.geom_matid[index]
        rgba = model.mat_rgba[material] if material >= 0 else model.geom_rgba[index]
        packet = {"id": index, "color": rgba[:3].tolist()}
        if kind == mujoco.mjtGeom.mjGEOM_MESH:
            mesh = model.geom_dataid[index]
            start, count = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
            face_start, face_count = model.mesh_faceadr[mesh], model.mesh_facenum[mesh]
            packet.update(vertices=model.mesh_vert[start:start + count].ravel().tolist(),
                          faces=model.mesh_face[face_start:face_start + face_count].ravel().tolist())
        elif kind == mujoco.mjtGeom.mjGEOM_CYLINDER:
            radius, half_length = model.geom_size[index, :2]
            packet["cylinder"] = [float(radius), float(2 * half_length)]
        else:
            packet["size"] = (model.geom_size[index] * 2).tolist()
        meshes.append(packet)
    return meshes


def render_transforms(data, geom_ids):
    return [{"id": index, "position": data.geom_xpos[index].tolist(),
             "quaternion": Rotation.from_matrix(data.geom_xmat[index].reshape(3, 3)).as_quat().tolist()}
            for index in geom_ids]


class SceneViewer:
    """Read-only dual-arm poses, with browser stop and reconnection support."""

    def __init__(self, model, data, target="stararm", *, port=8080, open_browser=True,
                 server_factory=None):
        validate_port(port)
        self.data = data
        self.stop = threading.Event()
        self.control_lock = threading.RLock()
        self.clients, self.acknowledged = set(), {}
        self.title = "reBot RS dual-arm mirror" if target == "rebot" else "Star Arm dual-arm preview"
        meshes = model_meshes(model)
        self.geom_ids = [mesh["id"] for mesh in meshes]
        self.model_json = json.dumps({"meshes": meshes, "read_only": True,
                                     "title": self.title}, allow_nan=False).encode()
        center = np.array([.4, 0., 1.]) if target == "rebot" else np.array([0., 0., .15])
        distance = 1.7 if target == "rebot" else 1.2
        direction = np.array([1., -.9, .53]) if target == "rebot" else np.array([-1., -.25, .5])
        self.camera = {"position": (center + direction / np.linalg.norm(direction) * distance).tolist(),
                       "target": center.tolist()}
        self.sync()
        self.server, self.url = start_server(self, port, open_browser=open_browser,
                                             server_factory=server_factory)

    def receive(self, message, client):
        if not isinstance(message, dict):
            raise ValueError("Expected a command object.")
        with self.control_lock:
            if message.get("type") == "ready":
                self.clients.add(client)
            elif message.get("type") == "stop":
                request_stop(self.stop, f"web stop viewer button (client {client})")
            else:
                raise ValueError("This viewer only displays poses; robot control is unavailable.")

    def sync(self):
        transforms = render_transforms(self.data, self.geom_ids)
        with self.control_lock:
            self._transforms = transforms

    def snapshot(self, client=None):
        with self.control_lock:
            return {"type": "state", "read_only": True, "transforms": self._transforms,
                    "camera": self.camera, "camera_version": 1, "stopped": self.stop.is_set()}

    def is_running(self):
        return not self.stop.is_set()

    def close(self):
        self.stop.set()
        self.server.stop()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
