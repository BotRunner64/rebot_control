"""Loopback HTTP and MJPEG transport for the webcam preview page."""

import errno
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import re
import threading
from urllib.parse import urlsplit


ASSET_ROOT = Path(__file__).with_name("web") / "webcam"
ASSETS = {"/": ("index.html", "text/html; charset=utf-8"),
          "/viewer.css": ("viewer.css", "text/css; charset=utf-8"),
          "/viewer.js": ("viewer.js", "text/javascript; charset=utf-8")}


class PreviewHTTPServer(ThreadingHTTPServer):
    daemon_threads = True

    def get_request(self):
        connection, address = super().get_request()
        connection.settimeout(2.)
        return connection, address


class PreviewHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass

    def respond(self, status, body, mime="application/json"):
        self.send_response(status)
        self.send_header("Content-Type", mime)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Content-Security-Policy", "default-src 'self'; frame-ancestors 'none'")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = urlsplit(self.path).path
        viewer = self.server.viewer
        try:
            if path in ASSETS:
                file, mime = ASSETS[path]
                self.respond(200, (ASSET_ROOT / file).read_bytes(), mime)
            elif path == "/api/cameras":
                cameras = [{"id": index, **camera.snapshot()} for index, camera in enumerate(viewer.cameras)]
                self.respond(200, json.dumps({"cameras": cameras}).encode())
            elif match := re.fullmatch(r"/streams/(\d+)\.mjpg", path):
                index = int(match[1])
                if index >= len(viewer.cameras):
                    self.respond(404, b'{"error":"Camera not found"}')
                    return
                self.stream(viewer.cameras[index], viewer)
            else:
                self.respond(404, b'{"error":"Not found"}')
        except (BrokenPipeError, ConnectionResetError, TimeoutError):
            pass

    def stream(self, camera, viewer):
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        sequence = 0
        while not viewer.stopping.is_set():
            latest = camera.wait_frame(sequence)
            if latest is None:
                if camera.snapshot()["state"] in ("error", "stopped"):
                    break
                continue
            sequence, frame = latest
            self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                             + str(len(frame)).encode() + b"\r\n\r\n" + frame + b"\r\n")

    def do_POST(self):
        viewer = self.server.viewer
        if urlsplit(self.path).path != "/api/stop":
            self.respond(404, b'{"error":"Not found"}')
            return
        if self.headers.get("Origin") not in viewer.origins:
            self.respond(403, b'{"error":"Invalid origin"}')
            return
        if self.headers.get("Content-Type") != "application/json" or self.headers.get("Content-Length") != "2":
            self.respond(400, b'{"error":"Expected an empty JSON object"}')
            return
        if self.rfile.read(2) != b"{}":
            self.respond(400, b'{"error":"Expected an empty JSON object"}')
            return
        self.respond(200, b'{"stopped":true}')
        viewer.request_stop()


class WebcamServer:
    def __init__(self, cameras, request_stop, *, port=8080):
        if type(port) is not int or not 0 <= port <= 65535:
            raise ValueError("Webcam viewer port must be between 0 and 65535.")
        self.cameras, self.request_stop = cameras, request_stop
        self.stopping = threading.Event()
        for candidate in range(port, 65536):
            try:
                self.server = PreviewHTTPServer(("127.0.0.1", candidate), PreviewHandler)
                break
            except OSError as exc:
                if exc.errno != errno.EADDRINUSE:
                    raise
        else:
            raise OSError("No available local webcam viewer port.")
        self.port = self.server.server_port
        self.url = f"http://127.0.0.1:{self.port}"
        self.origins = {self.url, f"http://localhost:{self.port}"}
        self.server.viewer = self
        self.thread = threading.Thread(target=self.server.serve_forever, name="webcam-http", daemon=True)
        try:
            self.thread.start()
        except BaseException:
            self.server.server_close()
            raise

    def close(self):
        if not self.stopping.is_set():
            self.stopping.set()
            self.server.shutdown()
            self.server.server_close()
            self.thread.join()
