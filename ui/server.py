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
DEFAULT_W = float(os.environ.get("PLOT_WIDTH", "40"))
DEFAULT_H = float(os.environ.get("PLOT_HEIGHT", "40"))


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
        self.kind = "draw"      # "draw" (full drawing) or "jog" (reposition only)
        # Per-run config sent to the firmware in the path-stream header. The
        # firmware latches whatever config is current WHEN IT STARTS a run, so
        # editing config here only affects the next run that begins (firmware-side
        # locking). start_mode is "xy" or "cord".
        self.config = {
            "start_mode": "xy",   # "xy" -> use x/y; "cord" -> use left/right
            "start_x": 250.0,
            "start_y": 150.0,
            "start_left": 291.5,
            "start_right": 291.5,
            "speed": 10,          # stepper RPM
        }

    def publish(self, points, kind="draw"):
        """Make `points` the next thing the plotter does. kind is "draw" (a
        normal drawing, re-seeded to START first) or "jog" (move the pen from
        its CURRENT position to the single target, then adopt it as the new
        position/START)."""
        with self._cond:
            self.points = points
            self.total = len(points)
            self.drawn = 0
            self.kind = kind
            self.version += 1
            self._cond.notify_all()
            return self.version

    def set_config(self, updates):
        """Merge config updates. Always accepted; the firmware decides when to
        apply them (at the start of each run). Returns the merged config."""
        with self._cond:
            for k, v in updates.items():
                if k in self.config:
                    self.config[k] = v
            return dict(self.config)

    def wait_for_path(self, known_version, timeout=25.0, resume_from=0):
        """Block until a path newer than known_version exists. Returns
        (version, points, config) or (known_version, None, None) if it timed out.
        The config is snapshotted at the moment the path is handed out, so the
        firmware draws with exactly the config that was current at run start.

        resume_from > 0 is the firmware re-fetching the SAME version it was
        mid-way through after a connection drop: it asks for the current path
        starting at point `resume_from`. We hand back that version immediately
        (no long-poll) with the remaining points and mark the run kind "resume"
        so the firmware continues from where the pen physically is instead of
        re-seeding to START. Out-of-range indices fall through to a normal wait."""
        with self._cond:
            # Resume path: same version still loaded, and the requested index is
            # within it -> replay the tail without waiting or re-seeding.
            if (resume_from > 0 and self.version == known_version
                    and self.points and resume_from < len(self.points)):
                cfg = dict(self.config)
                cfg["_kind"] = "resume"
                return self.version, list(self.points[resume_from:]), cfg
            if self.version == known_version or not self.points:
                self._cond.wait(timeout)
            if self.version != known_version and self.points:
                cfg = dict(self.config)
                cfg["_kind"] = self.kind  # piggyback the run kind into the header
                return self.version, list(self.points), cfg
            return self.version, None, None

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
                "config": dict(self.config),
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
        resume_from = 0
        if "?" in self.path:
            q = self.path.split("?", 1)[1]
            for kv in q.split("&"):
                if kv.startswith("since="):
                    try:
                        since = int(kv[6:])
                    except ValueError:
                        since = 0
                elif kv.startswith("from="):
                    # Resume point: how many points of the SAME version the
                    # firmware already drew before it lost connection.
                    try:
                        resume_from = max(0, int(kv[5:]))
                    except ValueError:
                        resume_from = 0

        version, points, config = STORE.wait_for_path(since, resume_from=resume_from)
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
        # Body format the firmware parses:
        #   CFG <key> <value>      (zero or more config lines, FIRST)
        #   POINTS                 (marks the end of config / start of points)
        #   <x> <y>                (one per point)
        #   END
        # Config comes first so the firmware can latch START + speed before it
        # begins drawing this run.
        try:
            # kind=jog -> move from the pen's CURRENT position to the target and
            # don't re-seed to START first; kind=draw -> normal drawing.
            self.wfile.write(b"CFG kind %s\n" % config.get("_kind", "draw").encode())
            self.wfile.write(b"CFG start_mode %s\n" % config["start_mode"].encode())
            self.wfile.write(b"CFG start_x %.3f\n" % config["start_x"])
            self.wfile.write(b"CFG start_y %.3f\n" % config["start_y"])
            self.wfile.write(b"CFG start_left %.3f\n" % config["start_left"])
            self.wfile.write(b"CFG start_right %.3f\n" % config["start_right"])
            self.wfile.write(b"CFG speed %d\n" % int(config["speed"]))
            self.wfile.write(b"POINTS\n")
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
        elif self.path == "/config":
            self._handle_config(raw)
        elif self.path == "/jog":
            self._handle_jog(raw)
        else:
            self._send(404, json.dumps({"error": "not found"}))

    def _handle_jog(self, raw):
        """Physically move the pen to (x,y): publish a one-point JOG path. The
        firmware moves from its CURRENT position to the target (no re-seed), then
        adopts (x,y) as its position. We also set START=(x,y) so the next drawing
        begins there (jog sets the new START)."""
        try:
            req = json.loads(raw or b"{}")
            x = float(req["x"])
            y = float(req["y"])
            # Update START in config so the next DRAW run also begins here.
            STORE.set_config({"start_mode": "xy", "start_x": x, "start_y": y})
            version = STORE.publish([(x, y)], kind="jog")
            self._send(200, json.dumps({"ok": True, "version": version,
                                        "target": [x, y]}))
        except (KeyError, ValueError, TypeError):
            self._send(200, json.dumps({"error": "jog needs numeric x and y"}))
        except Exception as e:  # noqa: BLE001
            self._send(200, json.dumps({"error": str(e)}))

    def _handle_config(self, raw):
        """Update the per-run config (start position + motor speed). Always
        accepted; the firmware applies it at the start of the next run."""
        try:
            req = json.loads(raw or b"{}")
            updates = {}
            mode = req.get("start_mode")
            if mode in ("xy", "cord"):
                updates["start_mode"] = mode
            for k in ("start_x", "start_y", "start_left", "start_right"):
                if k in req:
                    updates[k] = float(req[k])
            if "speed" in req:
                # Clamp to a sane stepper RPM range so a typo can't stall/overspeed.
                updates["speed"] = max(1, min(30, int(req["speed"])))
            merged = STORE.set_config(updates)
            self._send(200, json.dumps({"ok": True, "config": merged}))
        except Exception as e:  # noqa: BLE001
            self._send(200, json.dumps({"error": str(e)}))

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
            # The painter emits points in a box from (0,0) to (W,H). Fed straight
            # to the firmware those land up against the anchor line (small y, near
            # the (0,0) anchor), so the pen yanks into a degenerate corner — it
            # looks like "a straight line upward". Center the box on the pen's
            # configured START instead, so a small (e.g. 80x80) drawing sits around
            # where the pen actually hangs. The box center is (W/2, H/2); shifting
            # it to (start_x, start_y) means adding (start_x - W/2, start_y - H/2).
            w = float(req.get("width", DEFAULT_W))
            h = float(req.get("height", DEFAULT_H))
            cfg = STORE.snapshot()["config"]
            off_x = float(cfg["start_x"]) - w / 2.0
            off_y = float(cfg["start_y"]) - h / 2.0
            points = [(x + off_x, y + off_y) for (x, y) in points]
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
