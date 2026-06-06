#!/usr/bin/env python3
"""Tiny zero-dependency UI server for previewing painter output.

Serves index.html and exposes one endpoint:

    POST /paint   body: {svg, width, height, fit, flipY}
                  -> runs ../painter and returns its polyline JSON

Run:  python3 ui/server.py   then open http://localhost:8000
"""

import json
import os
import subprocess
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
PAINTER = os.path.join(ROOT, "painter")
PORT = int(os.environ.get("PORT", "8000"))


def run_painter(svg, width, height, fit=True, flip_y=False):
    """Feed the SVG to ./painter via stdin, return (polylines, log)."""
    args = [PAINTER, "--width", str(width), "--height", str(height)]
    if not fit:
        args.append("--no-fit")
    if flip_y:
        args.append("--flip-y")
    proc = subprocess.run(
        args,
        input=svg.encode("utf-8"),
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    log = proc.stderr.decode("utf-8", "replace")
    if proc.returncode != 0:
        raise RuntimeError(log or f"painter exited {proc.returncode}")
    return json.loads(proc.stdout.decode("utf-8")), log


class Handler(BaseHTTPRequestHandler):
    def _send(self, code, body, ctype="application/json"):
        if isinstance(body, str):
            body = body.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            with open(os.path.join(HERE, "index.html"), "rb") as f:
                self._send(200, f.read(), "text/html; charset=utf-8")
        else:
            self._send(404, "not found", "text/plain")

    def do_POST(self):
        if self.path != "/paint":
            self._send(404, json.dumps({"error": "not found"}))
            return
        length = int(self.headers.get("Content-Length", "0"))
        try:
            req = json.loads(self.rfile.read(length) or b"{}")
            polylines, log = run_painter(
                req.get("svg", ""),
                req.get("width", 600),
                req.get("height", 400),
                fit=req.get("fit", True),
                flip_y=req.get("flipY", False),
            )
            self._send(200, json.dumps({"polylines": polylines, "log": log}))
        except Exception as e:  # noqa: BLE001 - surface any failure to the UI
            self._send(200, json.dumps({"error": str(e)}))

    def log_message(self, *_):  # quiet the default per-request logging
        pass


def main():
    if not os.path.exists(PAINTER):
        sys.exit(f"painter binary not found at {PAINTER}\n"
                 f"Build it:  g++ -std=c++17 -O2 controller/painter.cpp -o painter")
    srv = ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
    print(f"painter UI running -> http://localhost:{PORT}  (Ctrl-C to stop)")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\nbye")


if __name__ == "__main__":
    main()
