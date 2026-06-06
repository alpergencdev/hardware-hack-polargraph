#!/usr/bin/env python3
"""Zero-dependency server for the polargraph SIMULATOR UI.

Unlike ui/server.py (which only runs painter and previews its polylines), this
one runs the full controller pipeline:

    [image] --(rasterize)--> SVG --(painter)--> polylines --(sim)--> frame trace
    driving the REAL PolargraphState / EncoderState controller logic.

The rasterize stage is optional: paste/drop an SVG and it is used directly; drop
a RASTER image (PNG/JPG/...) and rasterize.py first converts it to a plottable,
detail-limited SVG (so a busy photo like a selfie becomes a clean line drawing).

The browser then animates the two encoder motors (anchors), their cords, and the
pen doing pen-up (travel) / pen-down (draw) movements.

Endpoint:
    POST /simulate   body: {svg, width, height, fit, flipY,
                            spacing, spoolD, ticksRev, initLen,
                            image, mode, detail, maxStrokes}
        image      base64-encoded raster image; when present it is rasterized
                   to SVG (overriding any svg field).
        mode       rasterize line-art mode: edge|threshold|centerline
        detail     0..100 detail dial for rasterize
        maxStrokes hard cap on strokes (0 = no cap)
        -> {anchorSpacing, left, right, box, frames, svg, log}  (sim output;
            `svg` is the rasterized SVG so the UI can show/edit it)
        -> or {error: "..."}

Run:  python3 simulator/server.py    then open http://localhost:8001
Build the binaries first:
    g++ -std=c++17 -O2 controller/painter.cpp -o painter
    g++ -std=c++17 -O2 controller/sim.cpp     -o sim
"""

import base64
import binascii
import json
import os
import subprocess
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
PAINTER = os.path.join(ROOT, "painter")
SIM = os.path.join(ROOT, "sim")
RASTERIZE = os.path.join(ROOT, "rasterize.py")
PORT = int(os.environ.get("PORT", "8001"))


def run_rasterize(image_bytes, mode="edge", detail=50, max_strokes=0):
    """Raster image bytes -> plottable SVG (str) via rasterize.py.

    The image is fed on stdin and the SVG comes back on stdout; rasterize.py
    reduces detail per `detail`/`max_strokes` so busy photos become plottable.
    """
    args = [
        sys.executable, RASTERIZE,
        "--mode", str(mode),
        "--detail", str(detail),
        "--max-strokes", str(int(max_strokes)),
    ]
    proc = subprocess.run(
        args, input=image_bytes,
        stdout=subprocess.PIPE, stderr=subprocess.PIPE,
    )
    log = proc.stderr.decode("utf-8", "replace")
    if proc.returncode != 0:
        raise RuntimeError(log or f"rasterize exited {proc.returncode}")
    return proc.stdout.decode("utf-8"), log


def run_painter(svg, width, height, fit=True, flip_y=False):
    """SVG -> polyline JSON (bytes) via ./painter."""
    args = [PAINTER, "--width", str(width), "--height", str(height)]
    if not fit:
        args.append("--no-fit")
    if flip_y:
        args.append("--flip-y")
    proc = subprocess.run(
        args, input=svg.encode("utf-8"),
        stdout=subprocess.PIPE, stderr=subprocess.PIPE,
    )
    log = proc.stderr.decode("utf-8", "replace")
    if proc.returncode != 0:
        raise RuntimeError(log or f"painter exited {proc.returncode}")
    return proc.stdout, log


def run_sim(polylines_json, spacing, spool_d, ticks_rev, init_len):
    """polyline JSON -> frame trace driving the real controller, via ./sim."""
    args = [
        SIM,
        "--spacing", str(int(spacing)),
        "--spool-d", str(int(spool_d)),
        "--ticks-rev", str(int(ticks_rev)),
        "--init-len", str(int(init_len)),
    ]
    proc = subprocess.run(
        args, input=polylines_json,
        stdout=subprocess.PIPE, stderr=subprocess.PIPE,
    )
    log = proc.stderr.decode("utf-8", "replace")
    if proc.returncode != 0:
        raise RuntimeError(log or f"sim exited {proc.returncode}")
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
        if self.path != "/simulate":
            self._send(404, json.dumps({"error": "not found"}))
            return
        length = int(self.headers.get("Content-Length", "0"))
        try:
            req = json.loads(self.rfile.read(length) or b"{}")

            # If a raster image was uploaded, convert it to an SVG first. The
            # data URL prefix (data:image/png;base64,) is stripped if present.
            svg = req.get("svg", "")
            rlog = ""
            image_b64 = req.get("image", "")
            if image_b64:
                if "," in image_b64[:64]:
                    image_b64 = image_b64.split(",", 1)[1]
                try:
                    image_bytes = base64.b64decode(image_b64, validate=False)
                except (binascii.Error, ValueError) as e:
                    raise RuntimeError(f"bad image data: {e}")
                svg, rlog = run_rasterize(
                    image_bytes,
                    mode=req.get("mode", "edge"),
                    detail=req.get("detail", 50),
                    max_strokes=req.get("maxStrokes", 0),
                )

            polys, plog = run_painter(
                svg,
                req.get("width", 300),
                req.get("height", 300),
                fit=req.get("fit", True),
                flip_y=req.get("flipY", False),
            )
            result, slog = run_sim(
                polys,
                req.get("spacing", 800),
                req.get("spoolD", 12),
                req.get("ticksRev", 360),
                req.get("initLen", 500),
            )
            result["log"] = (rlog + plog + slog).strip()
            if image_b64:
                result["svg"] = svg  # let the UI show/edit the rasterized SVG
            self._send(200, json.dumps(result))
        except Exception as e:  # noqa: BLE001 - surface any failure to the UI
            self._send(200, json.dumps({"error": str(e)}))

    def log_message(self, *_):  # quiet per-request logging
        pass


def main():
    missing = [p for p in (PAINTER, SIM) if not os.path.exists(p)]
    if missing:
        sys.exit(
            "missing binaries: " + ", ".join(missing) + "\nBuild them:\n"
            "  g++ -std=c++17 -O2 controller/painter.cpp -o painter\n"
            "  g++ -std=c++17 -O2 controller/sim.cpp     -o sim"
        )
    # rasterize (image -> SVG) is optional; warn but don't block the SVG flow.
    if not os.path.exists(RASTERIZE):
        print(f"note: {RASTERIZE} not found; image upload disabled (SVG still works).")
    else:
        try:
            import cv2  # noqa: F401
        except ImportError:
            print("note: rasterize needs OpenCV+NumPy for image upload "
                  "(pip install opencv-python numpy); SVG flow still works.")
    srv = ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
    print(f"polargraph simulator -> http://localhost:{PORT}  (Ctrl-C to stop)")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\nbye")


if __name__ == "__main__":
    main()
