"""Loopback HTTP/WebSocket transport shared by the Three.js viewers."""

import asyncio
import errno
from http import HTTPStatus
import json
import mimetypes
import socket
import threading
from urllib.parse import unquote, urlsplit
import webbrowser

from rebot.paths import WEB
from rebot.runtime.console import request_stop


ASSET_ROOT = WEB / "viewer"


def validate_port(port):
    if type(port) is not int or not 1 <= port <= 65535:
        raise ValueError("Web viewer port must be between 1 and 65535.")


class ViewerServer:
    """Serve local assets and latest-only state on one loopback HTTP/WebSocket port.

    The viewer supplies `model_json`, `snapshot(client)`, `receive(message, client)`, the `stop`
    event, and `control_lock` guarding its `clients` set and `acknowledged` map.
    """

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


def start_server(viewer, port, *, open_browser=True, server_factory=None):
    """Serve `viewer`, announce its URL, and optionally open it; returns (server, url)."""
    server = (ViewerServer if server_factory is None else server_factory)(viewer, port)
    url = f"http://127.0.0.1:{server.port}"
    print(f"Three.js web viewer: {url}", flush=True)
    try:
        if open_browser:
            webbrowser.open(url)
    except Exception:
        server.stop()
        raise
    return server, url
