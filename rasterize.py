#!/usr/bin/env python3
"""rasterize.py — turn ANY image into a plottable SVG (with detail control).

This is the missing front-end for the polargraph pipeline. The existing tools
expect an SVG:

    image.svg --> ./painter --> polylines --> ./sim --> frame trace --> UI

But two things break that:

  1. The input is often a RASTER image (PNG/JPG/HEIC/...), not an SVG.
  2. Even once vectorized, a photo (e.g. a selfie) has FAR too much detail to
     plot — a pen plotter wants a few hundred clean strokes, not ten thousand
     pixel-edge wiggles.

This tool solves both at once. It loads any raster image, reduces it to line
art with a tunable amount of detail, traces those lines into vector contours,
simplifies them, and writes an SVG made of <path> elements — exactly the subset
painter.cpp already understands (M/L/Z commands, fill="none" so nothing
occludes). The result drops straight into the existing pipeline.

If the input is ALREADY an SVG, it is passed through unchanged (optionally still
detail-limited by dropping the smallest paths) so this tool is safe to put in
front of everything.

Modes (how the line art is produced):

  edge      Canny edge detection. Best general-purpose "sketch from a photo".
  threshold Adaptive threshold -> bold black/white regions. Good for high
            contrast subjects, logos, lettering.
  centerline Skeletonize the thresholded shape into 1px center lines. Good for
            line drawings / handwriting you want as single strokes, not outlines.

Detail control (the knob the user asked for):

  --detail 0..100   One dial. 0 = drastically simplified (few bold strokes),
                    100 = keep almost everything. Default 50. Internally this
                    scales blur, edge thresholds, the minimum contour length to
                    keep, and how aggressively contours are simplified.
  --max-strokes N   Hard cap: keep only the N longest contours. The single best
                    lever for "this is too complex" — a selfie at --max-strokes
                    300 becomes a clean recognizable line portrait.

Examples:

    # Photo -> sketch SVG, then straight into the existing pipeline:
    python3 rasterize.py selfie.jpg -o selfie.svg
    ./painter selfie.svg --width 600 --height 400 | ./sim --spacing 800

    # Same thing in one shot, no temp file:
    python3 rasterize.py selfie.jpg | ./painter --width 600 --height 400 | ./sim

    # Way fewer strokes for a busy image:
    python3 rasterize.py selfie.jpg --detail 30 --max-strokes 250 -o out.svg

Dependencies: OpenCV (cv2) and NumPy. No svg library needed — we emit SVG text.
"""

import argparse
import os
import sys

import cv2
import numpy as np


# ──────────────────────────────── loading ───────────────────────────────────

def load_gray(path):
    """Load an image (any OpenCV-supported format) as a grayscale uint8 array.

    Reads from `path`, or from stdin if path is '-' / empty. HEIC/other formats
    OpenCV cannot decode will raise a clear error.
    """
    if path and path != "-":
        img = cv2.imread(path, cv2.IMREAD_COLOR)
        if img is None:
            raise RuntimeError(
                f"cannot decode image: {path} "
                f"(unsupported format? try converting to PNG/JPG first)"
            )
    else:
        data = sys.stdin.buffer.read()
        if not data:
            raise RuntimeError("empty image on stdin")
        arr = np.frombuffer(data, dtype=np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if img is None:
            raise RuntimeError("could not decode image from stdin")
    return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


def looks_like_svg(path):
    """Cheap sniff: is this file already an SVG we should pass through?"""
    if not path or path == "-":
        return False
    if path.lower().endswith(".svg"):
        return True
    try:
        with open(path, "rb") as f:
            head = f.read(512).lstrip().lower()
        return head.startswith(b"<?xml") and b"<svg" in head or head.startswith(b"<svg")
    except OSError:
        return False


# ─────────────────────────── detail → parameters ────────────────────────────
# One user-facing dial (--detail 0..100) fans out to the several knobs that
# actually control how much survives. Keeping the mapping in one place makes the
# behavior predictable: higher detail => less blur, lower edge thresholds (more
# edges), shorter minimum contour, gentler simplification.

def detail_params(detail, longest_side):
    """Map --detail (0..100) to concrete processing parameters."""
    d = max(0.0, min(100.0, float(detail))) / 100.0  # 0..1

    # Pre-blur kernel: more blur at low detail wipes out fine texture (pores,
    # hair, noise) so only major features survive. Always odd.
    blur = int(round(7 - 6 * d))          # 7 (low detail) .. 1 (high detail)
    blur = max(1, blur | 1)

    # Canny hysteresis thresholds: low detail => high thresholds => fewer edges.
    canny_lo = int(round(120 - 90 * d))   # 120 .. 30
    canny_hi = int(round(canny_lo * 2.5))

    # Minimum contour length to keep, as a fraction of the image's longest side.
    # Low detail discards short squiggles aggressively.
    min_len = (0.08 - 0.075 * d) * longest_side  # ~8% .. ~0.5% of the side

    # Douglas–Peucker simplification tolerance (px). Low detail => coarse.
    epsilon = 3.5 - 3.0 * d               # 3.5px .. 0.5px

    return {
        "blur": blur,
        "canny_lo": canny_lo,
        "canny_hi": canny_hi,
        "min_len": max(2.0, min_len),
        "epsilon": max(0.4, epsilon),
    }


# ─────────────────────────────── line art ───────────────────────────────────

def to_edges(gray, p):
    """edge mode: Canny edges → binary line image (255 = ink)."""
    g = cv2.GaussianBlur(gray, (p["blur"], p["blur"]), 0)
    return cv2.Canny(g, p["canny_lo"], p["canny_hi"])


def to_threshold(gray, p):
    """threshold mode: adaptive threshold → bold black-on-white regions."""
    g = cv2.GaussianBlur(gray, (p["blur"], p["blur"]), 0)
    # Block size scales with blur so smoother (low-detail) runs use larger
    # neighborhoods and fewer, bolder regions.
    block = max(3, (p["blur"] * 4 + 3) | 1)
    binv = cv2.adaptiveThreshold(
        g, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, block, 5
    )
    return binv


def to_centerline(gray, p):
    """centerline mode: threshold then morphologically thin to 1px skeleton.

    Uses cv2.ximgproc.thinning if available (opencv-contrib), otherwise a
    classic morphological thinning fallback so this works on plain opencv.
    """
    binv = to_threshold(gray, p)
    thin = getattr(getattr(cv2, "ximgproc", None), "thinning", None)
    if thin is not None:
        return thin(binv)
    return _morph_skeleton(binv)


def _morph_skeleton(binv):
    """Zhang-Suen-ish skeleton via iterated morphological thinning (fallback)."""
    img = (binv > 0).astype(np.uint8) * 255
    skel = np.zeros_like(img)
    elem = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
    while True:
        opened = cv2.morphologyEx(img, cv2.MORPH_OPEN, elem)
        temp = cv2.subtract(img, opened)
        eroded = cv2.erode(img, elem)
        skel = cv2.bitwise_or(skel, temp)
        img = eroded
        if cv2.countNonZero(img) == 0:
            break
    return skel


MODES = {"edge": to_edges, "threshold": to_threshold, "centerline": to_centerline}


# ───────────────────────── contours → simplified paths ──────────────────────

def polyline_length(pts):
    """Total path length of an Nx2 contour."""
    if len(pts) < 2:
        return 0.0
    d = np.diff(pts, axis=0)
    return float(np.sum(np.sqrt((d * d).sum(axis=1))))


def trace_paths(line_img, p, max_strokes):
    """Trace a binary line image into simplified vector contours.

    Returns a list of Nx2 float arrays (polylines), ordered longest-first and
    truncated to `max_strokes` if given. Each contour is simplified with
    Douglas–Peucker and short contours are dropped per the detail params.
    """
    # RETR_LIST: every contour, no hierarchy — we want all the lines, not just
    # outer boundaries. NONE: keep all points so simplification controls density.
    contours, _ = cv2.findContours(
        line_img, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE
    )

    paths = []
    for c in contours:
        pts = c.reshape(-1, 2).astype(np.float32)
        if polyline_length(pts) < p["min_len"]:
            continue
        approx = cv2.approxPolyDP(pts, p["epsilon"], False).reshape(-1, 2)
        if len(approx) >= 2:
            paths.append(approx)

    # Longest strokes carry the most visual meaning; sort so --max-strokes keeps
    # the important ones.
    paths.sort(key=polyline_length, reverse=True)
    if max_strokes and max_strokes > 0:
        paths = paths[:max_strokes]
    return paths


# ────────────────────────────── SVG output ──────────────────────────────────

def paths_to_svg(paths, width, height):
    """Render polylines as an SVG of fill="none" <path> elements.

    Exactly the subset painter.cpp parses: M to the first point, L through the
    rest. fill="none" guarantees nothing occludes anything downstream.
    """
    out = []
    out.append(
        f'<?xml version="1.0" encoding="UTF-8"?>\n'
        f'<svg xmlns="http://www.w3.org/2000/svg" '
        f'width="{width}" height="{height}" '
        f'viewBox="0 0 {width} {height}">\n'
    )
    for pts in paths:
        d = "M " + " L ".join(f"{x:.2f},{y:.2f}" for x, y in pts)
        out.append(
            f'  <path d="{d}" fill="none" stroke="black" stroke-width="1"/>\n'
        )
    out.append("</svg>\n")
    return "".join(out)


# ──────────────────────────────── driver ────────────────────────────────────

def convert(path, mode, detail, max_strokes, invert):
    """Full pipeline: load -> line art -> trace -> SVG string. Returns (svg, log)."""
    gray = load_gray(path)
    if invert:
        gray = cv2.bitwise_not(gray)
    h, w = gray.shape
    p = detail_params(detail, max(h, w))

    line_img = MODES[mode](gray, p)
    paths = trace_paths(line_img, p, max_strokes)

    if not paths:
        raise RuntimeError(
            "no strokes survived — try a higher --detail, a different --mode, "
            "or a higher --max-strokes."
        )

    total_pts = sum(len(pl) for pl in paths)
    svg = paths_to_svg(paths, w, h)
    log = (
        f"rasterize: {w}x{h} image, mode={mode}, detail={detail}"
        + (f", max-strokes={max_strokes}" if max_strokes else "")
        + f" -> {len(paths)} paths, {total_pts} points "
        f"(blur={p['blur']}, canny={p['canny_lo']}/{p['canny_hi']}, "
        f"min_len={p['min_len']:.1f}px, eps={p['epsilon']:.1f}px)."
    )
    return svg, log


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="Convert any image to a plottable SVG with tunable detail."
    )
    ap.add_argument("input", nargs="?", default="-",
                    help="input image (PNG/JPG/...); '-' or omitted = stdin")
    ap.add_argument("-o", "--output", default="-",
                    help="output SVG path; '-' or omitted = stdout")
    ap.add_argument("--mode", choices=list(MODES), default="edge",
                    help="line-art method (default: edge)")
    ap.add_argument("--detail", type=float, default=50,
                    help="0..100 detail dial (default 50); lower = simpler")
    ap.add_argument("--max-strokes", type=int, default=0,
                    help="hard cap on number of strokes (0 = no cap); the best "
                         "lever for over-complex images like selfies")
    ap.add_argument("--invert", action="store_true",
                    help="invert before processing (for light lines on dark)")
    args = ap.parse_args(argv)

    # Already an SVG? Pass it through untouched so this tool is safe to front
    # the whole pipeline. (Detail limiting of existing SVGs is left to painter.)
    if looks_like_svg(args.input):
        with open(args.input, "r", encoding="utf-8", errors="replace") as f:
            svg = f.read()
        log = f"rasterize: input is already SVG ({args.input}); passed through."
        write_output(args.output, svg)
        print(log, file=sys.stderr)
        return 0

    try:
        svg, log = convert(
            args.input, args.mode, args.detail, args.max_strokes, args.invert
        )
    except Exception as e:  # surface a clean message, not a traceback
        print(f"rasterize: error: {e}", file=sys.stderr)
        return 1

    write_output(args.output, svg)
    print(log, file=sys.stderr)
    return 0


def write_output(out_path, svg):
    if out_path and out_path != "-":
        with open(out_path, "w", encoding="utf-8") as f:
            f.write(svg)
    else:
        sys.stdout.write(svg)


if __name__ == "__main__":
    sys.exit(main())
