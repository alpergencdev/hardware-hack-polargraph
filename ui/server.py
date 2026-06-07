#!/usr/bin/env python3
"""Zero-dependency UI + plotter server for painter output.

Two audiences talk to this server:

  • The BROWSER (humans uploading pictures):
        GET  /              -> the upload/preview website (index.html)
        POST /paint         -> run painter, return polyline JSON for the preview
        POST /publish       -> run painter --single-line, store it as THE path the
                               plotter should draw next; returns the point count
        GET  /status        -> small JSON: is a path queued, how far has the
                               plotter got (for the live progress bar)

  • The ESP32 PLOTTER (fetching the path over WiFi):
        GET  /path.ndjson   -> the current single-line path, STREAMED one point
                               per line as "x y\\n", terminated by a line "END".
                               Long-polls (waits) until a path is published, so the
                               firmware can just block on this GET after boot.
        POST /progress       -> firmware reports points drawn so far (optional;
                               drives the website progress bar)

Why NDJSON streaming for the ESP32: the firmware never holds the whole array or
runs a JSON parser — it reads a line, parses two floats, drives the motors, then
reads the next. Tiny constant memory regardless of path length.

Run:  python3 ui/server.py     then open http://localhost:8000
The ESP32 connects to  http://<this-machine-LAN-IP>:8000/path.ndjson
"""

import json
import os
import subprocess
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
PAINTER = os.path.join(ROOT, "painter")
PORT = int(os.environ.get("PORT", "8000"))

# Default target box for the published path, in millimetres of the drawing area.
# The firmware maps these straight into its anchor frame, so keep them in the
# machine's coordinate range. Overridable per-publish from the website.
DEFAULT_W = float(os.environ.get("PLOT_WIDTH", "600"))
DEFAULT_H = float(os.environ.get("PLOT_HEIGHT", "400"))


class PathStore:
    """Holds THE current path the plotter should draw, plus live progress.

    A path is a list of (x, y) points (one continuous pen-down line). Access is
    guarded by a lock + condition so /path.ndjson can block until one exists and
    wake up the instant the browser publishes a new one.
    """

    def __init__(self):
        self._cond = threading.Condition()
        self.points = []        # [(x, y), ...] the single line, or [] if none
        self.version = 0        # bumps each publish; lets the firmware detect "new"
        self.drawn = 0          # points the plotter reports it has completed
        self.total = 0          # len(points) of the active path

    def publish(self, points):
        with self._cond:
            self.points = points
            self.total = len(points)
            self.drawn = 0
            self.version += 1
            self._cond.notify_all()
            return self.version

    def wait_for_path(self, known_version, timeout=25.0):
        """Block until a path newer than known_version exists. Returns
        (version, points) or (known_version, None) if it timed out."""
        with self._cond:
            if self.version == known_version or not self.points:
                self._cond.wait(timeout)
            if self.version != known_version and self.points:
                return self.version, list(self.points)
            return self.version, None

    def set_progress(self, drawn):
        with self._cond:
            self.drawn = drawn

    def snapshot(self):
        with self._cond:
            return {
                "version": self.version,
                "total": self.total,
                "drawn": self.drawn,
                "has_path": bool(self.points),
            }


STORE = PathStore()


def run_painter(svg, width, height, fit=True, flip_y=False, single_line=False):
    """Feed the SVG to ./painter via stdin, return (polylines, log)."""
    args = [PAINTER, "--width", str(width), "--height", str(height)]
    if not fit:
        args.append("--no-fit")
    if flip_y:
        args.append("--flip-y")
    if single_line:
        args.append("--single-line")
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


def flatten_single_line(polylines):
    """painter --single-line emits one polyline. Flatten to [(x,y), ...].

    If painter ever returns more than one polyline (e.g. it fell back), we
    concatenate them in order — the pen stays down across the joins, which is
    the single-line contract the firmware expects.
    """
    pts = []
    for pl in polylines:
        for xy in pl:
            if len(xy) >= 2:
                pts.append((float(xy[0]), float(xy[1])))
    return pts


class Handler(BaseHTTPRequestHandler):
    def _send(self, code, body, ctype="application/json"):
        if isinstance(body, str):
            body = body.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    # ── browser GETs ────────────────────────────────────────────────────────
    def do_GET(self):
        if self.path in ("/", "/index.html"):
            with open(os.path.join(HERE, "index.html"), "rb") as f:
                self._send(200, f.read(), "text/html; charset=utf-8")
        elif self.path == "/status":
            self._send(200, json.dumps(STORE.snapshot()))
        elif self.path.startswith("/path.ndjson"):
            self._stream_path()
        else:
            self._send(404, "not found", "text/plain")

    # ── the ESP32's blocking fetch ──────────────────────────────────────────
    def _stream_path(self):
        """Stream the current path as 'x y\\n' lines, ending with 'END'.

        ?since=N long-polls: the firmware passes the version it last drew so a
        fresh boot (since=0) gets the current path immediately, while a plotter
        that already drew version N waits here until the browser publishes N+1.
        """
        since = 0
        if "?" in self.path:
            q = self.path.split("?", 1)[1]
            for kv in q.split("&"):
                if kv.startswith("since="):
                    try:
                        since = int(kv[6:])
                    except ValueError:
                        since = 0

        version, points = STORE.wait_for_path(since)
        if points is None:
            # Timed out with nothing new — tell the firmware to retry (204).
            self.send_response(204)
            self.send_header("X-Path-Version", str(version))
            self.end_headers()
            return

        self.send_response(200)
        self.send_header("Content-Type", "text/plain; charset=utf-8")
        self.send_header("X-Path-Version", str(version))
        self.send_header("X-Path-Points", str(len(points)))
        # No Content-Length: we stream and close, so the firmware reads until END.
        self.end_headers()
        try:
            for (x, y) in points:
                self.wfile.write(b"%.3f %.3f\n" % (x, y))
            self.wfile.write(b"END\n")
        except (BrokenPipeError, ConnectionResetError):
            pass  # ESP32 dropped mid-stream; nothing to clean up

    # ── browser + plotter POSTs ─────────────────────────────────────────────
    def do_POST(self):
        length = int(self.headers.get("Content-Length", "0"))
        raw = self.rfile.read(length) if length else b"{}"

        if self.path == "/paint":
            self._handle_paint(raw)
        elif self.path == "/publish":
            self._handle_publish(raw)
        elif self.path == "/progress":
            self._handle_progress(raw)
        else:
            self._send(404, json.dumps({"error": "not found"}))

    def _handle_paint(self, raw):
        try:
            req = json.loads(raw or b"{}")
            polylines, log = run_painter(
                req.get("svg", ""),
                req.get("width", DEFAULT_W),
                req.get("height", DEFAULT_H),
                fit=req.get("fit", True),
                flip_y=req.get("flipY", False),
                single_line=req.get("singleLine", False),
            )
            self._send(200, json.dumps({"polylines": polylines, "log": log}))
        except Exception as e:  # noqa: BLE001 - surface failures to the UI
            self._send(200, json.dumps({"error": str(e)}))

    def _handle_publish(self, raw):
        """Run painter in single-line mode and make it the plotter's next path."""
        try:
            req = json.loads(raw or b"{}")
            polylines, log = run_painter(
                req.get("svg", ""),
                req.get("width", DEFAULT_W),
                req.get("height", DEFAULT_H),
                fit=req.get("fit", True),
                flip_y=req.get("flipY", False),
                single_line=True,  # the plotter always draws ONE continuous line
            )
            points = flatten_single_line(polylines)
            if len(points) < 2:
                self._send(200, json.dumps(
                    {"error": "painter produced no drawable path", "log": log}))
                return
            version = STORE.publish(points)
            # Return the polylines too so the website can preview what was sent.
            self._send(200, json.dumps({
                "ok": True, "version": version, "points": len(points),
                "polylines": polylines, "log": log,
            }))
        except Exception as e:  # noqa: BLE001
            self._send(200, json.dumps({"error": str(e)}))

    def _handle_progress(self, raw):
        try:
            req = json.loads(raw or b"{}")
            STORE.set_progress(int(req.get("drawn", 0)))
            self._send(200, json.dumps({"ok": True}))
        except Exception as e:  # noqa: BLE001
            self._send(200, json.dumps({"error": str(e)}))

    def log_message(self, *_):  # quiet the default per-request logging
        pass


def lan_ip():
    """Best-effort LAN IP so we can print the URL the ESP32 should use."""
    import socket
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))  # no packets sent; just picks the route's iface
        return s.getsockname()[0]
    except Exception:  # noqa: BLE001
        return "127.0.0.1"
    finally:
        s.close()


def main():
    if not os.path.exists(PAINTER):
        sys.exit(f"painter binary not found at {PAINTER}\n"
                 f"Build it:  g++ -std=c++17 -O2 controller/painter.cpp -o painter")
    srv = ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
    ip = lan_ip()
    print(f"painter UI    -> http://localhost:{PORT}  (open this to upload)")
    print(f"ESP32 fetches -> http://{ip}:{PORT}/path.ndjson")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\nbye")


if __name__ == "__main__":
    main()
