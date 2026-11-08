"""Shared local Three.js transport and read-only robot scene viewer."""

import asyncio
import errno
from http import HTTPStatus
import json
import mimetypes
from pathlib import Path
import socket
import threading
from urllib.parse import unquote, urlsplit
import webbrowser

import numpy as np
from scipy.spatial.transform import Rotation

from rebot_ee_runtime import request_stop


ASSET_ROOT = Path(__file__).with_name("web") / "ee"


def validate_port(port):
    if type(port) is not int or not 1 <= port <= 65535:
        raise ValueError("Web viewer port must be between 1 and 65535.")


def model_meshes(model, *, include_boxes=True):
    """Copy backend geometry once; Three.js owns all rendering and interaction."""
    import mujoco

    meshes = []
    for index in range(model.ngeom):
        kind = model.geom_type[index]
        if kind != mujoco.mjtGeom.mjGEOM_MESH and not (
                include_boxes and kind == mujoco.mjtGeom.mjGEOM_BOX):
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
        else:
            packet["size"] = (model.geom_size[index] * 2).tolist()
        meshes.append(packet)
    return meshes


def render_transforms(data, geom_ids):
    return [{"id": index, "position": data.geom_xpos[index].tolist(),
             "quaternion": Rotation.from_matrix(data.geom_xmat[index].reshape(3, 3)).as_quat().tolist()}
            for index in geom_ids]


class ViewerServer:
    """Serve local assets and latest-only state on one loopback HTTP/WebSocket port."""

    def __init__(self, viewer, port):
        validate_port(port)
        self.viewer = viewer
        self.error = None
        self.started = threading.Event()
        self.loop = self.finished = None
        for candidate in range(port, 65536):
            try:
                self.socket = socket.create_server(("127.0.0.1", candidate))
                break
            except OSError as exc:
                if exc.errno != errno.EADDRINUSE:
                    raise
        else:
            raise OSError("No available local viewer port.")
        self.port = self.socket.getsockname()[1]
        self.socket.setblocking(False)
        self.thread = threading.Thread(target=self._run, name="threejs-web", daemon=True)
        self.thread.start()
        self.started.wait()
        if self.error is not None:
            self.socket.close()
            self.thread.join()
            raise self.error

    def _run(self):
        try:
            asyncio.run(self._serve())
        except BaseException as exc:
            self.error = exc
            if self.started.is_set():
                request_stop(self.viewer.stop, f"web server failed: {exc}")
        finally:
            self.started.set()

    async def _serve(self):
        from websockets.asyncio.server import serve

        self.loop = asyncio.get_running_loop()
        self.finished = asyncio.Event()
        origins = [f"http://{host}:{self.port}" for host in ("127.0.0.1", "localhost")]
        async with serve(self._connection, sock=self.socket, origins=origins,
                         process_request=self._http, max_size=16384, close_timeout=1):
            self.started.set()
            await self.finished.wait()

    def _http(self, connection, request):
        from websockets.datastructures import Headers
        from websockets.http11 import Response

        path = unquote(urlsplit(request.path).path)
        if path == "/ws":
            return None
        status = HTTPStatus.OK
        if path == "/model.json":
            content, mime = self.viewer.model_json, "application/json"
        else:
            relative = "index.html" if path == "/" else path.lstrip("/")
            file = (ASSET_ROOT / relative).resolve()
            if not file.is_relative_to(ASSET_ROOT.resolve()) or not file.is_file():
                status, content, mime = HTTPStatus.NOT_FOUND, b"Not found", "text/plain"
            else:
                content = file.read_bytes()
                mime = "text/javascript" if file.suffix == ".js" else mimetypes.guess_type(file.name)[0]
        return Response(status.value, status.phrase, Headers({
            "Content-Type": f"{mime or 'application/octet-stream'}; charset=utf-8",
            "Content-Length": str(len(content)), "Cache-Control": "no-store",
            "X-Content-Type-Options": "nosniff"}), content)

    async def _connection(self, websocket):
        from websockets.exceptions import ConnectionClosed

        client = str(websocket.id)
        await websocket.send(json.dumps({"type": "hello", "client": client}))

        async def send_states():
            while True:
                await websocket.send(json.dumps(self.viewer.snapshot(client), allow_nan=False))
                await asyncio.sleep(.02)

        sender = asyncio.create_task(send_states())
        try:
            async for raw in websocket:
                try:
                    message = json.loads(raw)
                    self.viewer.receive(message, client)
                    if self.viewer.stop.is_set():
                        await websocket.send(json.dumps(self.viewer.snapshot(client), allow_nan=False))
                except (ValueError, TypeError, KeyError) as exc:
                    await websocket.send(json.dumps({"type": "error", "message": str(exc)}))
        except ConnectionClosed:
            pass
        finally:
            sender.cancel()
            await asyncio.gather(sender, return_exceptions=True)
            with self.viewer.control_lock:
                self.viewer.clients.discard(client)
                self.viewer.acknowledged.pop(client, None)

    def stop(self):
        if self.thread.is_alive():
            self.loop.call_soon_threadsafe(self.finished.set)
            self.thread.join()


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
        direction = np.array([-1., -.25, .5])
        self.camera = {"position": (center + direction / np.linalg.norm(direction) * distance).tolist(),
                       "target": center.tolist()}
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


