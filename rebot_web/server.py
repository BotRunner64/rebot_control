"""Loopback HTTP server. Opening a page never connects to a motor."""
import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import mimetypes
from pathlib import Path
import secrets
from urllib.parse import parse_qs, urlsplit

from .controller import Controller, load_config


STATIC = Path(__file__).with_name("static")


def make_server(controller, port=8765):
    csrf = secrets.token_urlsafe(32)

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            if args and not str(args[0]).startswith(("GET /api/state", "POST /api/heartbeat", "POST /api/target")):
                super().log_message(fmt, *args)

        def valid_host(self):
            port = self.server.server_port
            return self.headers.get("Host") in (f"127.0.0.1:{port}", f"localhost:{port}")

        def reply(self, data, code=200, content_type="application/json; charset=utf-8"):
            body = json.dumps(data, ensure_ascii=False, allow_nan=False).encode() if isinstance(data, dict) else data
            self.send_response(code)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header("Content-Security-Policy", "default-src 'self'; script-src 'self'; style-src 'self'; connect-src 'self'; img-src 'self' data:; frame-ancestors 'none'; base-uri 'none'; form-action 'none'")
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_GET(self):
            if not self.valid_host():
                return self.reply({"error": "仅允许本机访问。"}, 403)
            parsed = urlsplit(self.path)
            if parsed.path == "/api/state":
                client = parse_qs(parsed.query).get("client", [""])[0]
                return self.reply({**controller.snapshot(client), "csrf": csrf})
            files = {"/": "index.html", "/index.html": "index.html", "/app.js": "app.js", "/style.css": "style.css"}
            name = files.get(parsed.path)
            if name:
                mime = mimetypes.guess_type(name)[0] or "text/plain"
                return self.reply((STATIC / name).read_bytes(), content_type=mime + "; charset=utf-8")
            self.reply({"error": "Not found"}, 404)

        def do_POST(self):
            origin = self.headers.get("Origin")
            allowed = {f"http://127.0.0.1:{self.server.server_port}", f"http://localhost:{self.server.server_port}"}
            if (not self.valid_host() or origin not in allowed or
                not secrets.compare_digest(self.headers.get("X-Control-Token", ""), csrf)):
                return self.reply({"error": "请从本机控制页面操作。"}, 403)
            if self.headers.get_content_type() != "application/json":
                return self.reply({"error": "需要 JSON 请求。"}, 415)
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if not 0 < length <= 4096:
                    raise ValueError("请求大小不正确。")
                body = json.loads(self.rfile.read(length))
                if not isinstance(body, dict):
                    raise ValueError("请求格式不正确。")
                client = body.get("client")
                route = urlsplit(self.path).path
                if route == "/api/connect":
                    controller.connect(client)
                elif route == "/api/heartbeat":
                    controller.heartbeat(client)
                elif route == "/api/takeover":
                    controller.takeover(client)
                elif route == "/api/enable":
                    controller.enable(client, body.get("joint"))
                elif route == "/api/enable-all":
                    controller.enable_all(client)
                elif route == "/api/execute":
                    controller.execute(client, body.get("joint"), body.get("angle"), body.get("speed"))
                elif route == "/api/target":
                    controller.execute(client, body.get("joint"), body.get("angle"), body.get("speed"),
                                       direct=True, revision=body.get("revision"))
                elif route == "/api/stop":
                    controller.stop(client)
                elif route == "/api/reset":
                    controller.reset_all(client, body.get("speed"), body.get("revision"))
                elif route == "/api/disable":
                    controller.disable(client, body.get("joint"))
                elif route == "/api/disconnect":
                    controller.disconnect(client)
                else:
                    return self.reply({"error": "Unknown action"}, 404)
                self.reply({"ok": True, "state": controller.snapshot(client)})
            except (ValueError, RuntimeError, OSError, KeyError, TypeError) as exc:
                self.reply({"error": str(exc)}, 400)

    return ThreadingHTTPServer(("127.0.0.1", port), Handler)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--config")
    parser.add_argument("--simulate", action="store_true", help="只使用软件电机，不访问 CAN")
    args = parser.parse_args()
    controller = Controller(load_config(args.config), simulate=args.simulate)
    server = make_server(controller, args.port)
    print(f"reBot WebUI: http://127.0.0.1:{server.server_port}", flush=True)
    print("模拟模式" if args.simulate else "真机模式；等待页面手动连接，尚未打开 CAN。", flush=True)
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        print("停止服务；不会自动回零或失能。电机最后保持指令可能仍有效。", flush=True)
    finally:
        server.server_close()
        controller.close()


if __name__ == "__main__":
    main()
