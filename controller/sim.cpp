// sim.cpp — host-side polargraph simulator driving the REAL controller logic.
//
// This is the firmware's twin on your computer. Instead of re-implementing the
// kinematics, it #includes the very same headers the firmware uses
// (EncoderState.h, PolargraphState.h, Geometry.h) and drives them exactly the
// way main.cpp would: it builds two EncoderState "motors", wraps them in a
// PolargraphState, and feeds it a sequence of pen moves via moveTo(). The only
// thing that differs from hardware is the drive() callback — on a real machine
// it would spin motors and block until the encoders arrive; here it just
// advances the simulated encoders by the requested tick delta and records a
// frame.
//
// INPUT  (stdin): polyline JSON, exactly what painter.cpp emits —
//   [ [[x,y],[x,y],...],   <- stroke 0: pen-up travel to first pt, pen down,
//     [[x,y],...], ... ]      draw through the rest, pen up.
//
// OUTPUT (stdout): a frame trace JSON describing the whole run, ready for a UI:
//   {
//     "anchorSpacing": 800,
//     "left":  {"x":0,   "y":0, "spool":..., "ticksPerRev":..., "initLen":...},
//     "right": {"x":800, "y":0, ...},
//     "box":   {"w":..., "h":...},          // source drawing box (from input)
//     "frames": [
//       {"x":..,"y":..,"L":..,"R":..,"lp":..,"rp":..,"pen":0|1}, ...
//     ]
//   }
//   where x,y    = pen position the controller THINKS it is at (forward
//                  kinematics from the encoders — i.e. what the machine
//                  actually achieves, including tick-quantization error),
//         L,R    = left/right cord lengths (mm),
//         lp,rp  = left/right encoder tick counts,
//         pen    = 1 while drawing (pen down), 0 while traveling (pen up).
//
// Build & run on your computer (NOT firmware — no Arduino dependency):
//   g++ -std=c++17 -O2 controller/sim.cpp -o sim
//   ./painter drawing.svg --width 600 --height 400 | ./sim --spacing 800
//
// Anchor geometry: the left motor sits at (0,0) and the right motor at
// (spacing, 0); the pen hangs below at y>0. Change --spacing to move the
// anchors apart/together (the UI exposes this as draggable anchors).

#include <cmath>
#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "EncoderState.h"
#include "Geometry.h"
#include "PolargraphState.h"

namespace {

struct Pt {
  double x = 0, y = 0;
};
using Polyline = std::vector<Pt>;

// ── minimal JSON-number scanner for painter's array-of-arrays-of-pairs ──
// painter emits a strict shape ( [ [ [x,y],... ], ... ] ) with plain decimals,
// so we don't need a real JSON parser: pull every number out, and use the
// bracket structure to know where each polyline starts/ends.
std::vector<Polyline> parsePolylines(const std::string &s) {
  std::vector<Polyline> out;
  Polyline cur;
  std::vector<double> pair;  // accumulates up to 2 numbers (one [x,y])
  int depth = 0;             // bracket nesting: 1=outer, 2=polyline, 3=point

  auto flushPair = [&]() {
    if (pair.size() >= 2) cur.push_back({pair[0], pair[1]});
    pair.clear();
  };
  auto flushPolyline = [&]() {
    if (!cur.empty()) out.push_back(cur);
    cur.clear();
  };

  size_t i = 0, n = s.size();
  while (i < n) {
    char c = s[i];
    if (c == '[') {
      ++depth;
      ++i;
    } else if (c == ']') {
      if (depth == 3) flushPair();        // close a point
      else if (depth == 2) flushPolyline();  // close a polyline
      --depth;
      ++i;
    } else if (c == '-' || c == '+' || c == '.' || std::isdigit((unsigned char)c)) {
      size_t j = i;
      if (s[j] == '+' || s[j] == '-') ++j;
      bool dot = false;
      while (j < n) {
        char d = s[j];
        if (std::isdigit((unsigned char)d)) ++j;
        else if (d == '.' && !dot) { dot = true; ++j; }
        else if (d == 'e' || d == 'E') { ++j; if (j < n && (s[j]=='+'||s[j]=='-')) ++j; }
        else break;
      }
      try { pair.push_back(std::stod(s.substr(i, j - i))); } catch (...) {}
      i = j;
    } else {
      ++i;
    }
  }
  return out;
}

// ── shared sim state the (capture-less) drive callback reaches through ──
// PolargraphState::moveTo wants a plain function pointer, which cannot capture,
// so we route through file scope — exactly the kind of global the firmware's ISR
// world uses too.
//
// IMPORTANT: PolargraphState owns its EncoderState members BY VALUE. The
// firmware feedback loop is: drive the motors, the encoder ISR counts ticks,
// then the caller pushes the fresh tick counts back into the controller via
// setLeftPosition()/setRightPosition(). We reproduce exactly that: we keep the
// running tick counts here, advance them by each segment's delta, and push them
// into the machine. We then read cord lengths / pen position back OUT of the
// machine, so every reported value comes from the real controller.
struct SimState {
  PolargraphState *machine = nullptr;
  int leftPos = 0;   // running left encoder tick count (mirrors the machine's)
  int rightPos = 0;  // running right encoder tick count
  // Calibration (needed only to convert ticks -> cord length for the report;
  // the controller does its own conversion internally and we cross-check).
  int spoolD = 0, ticksRev = 0, initLen = 0;
  int penDown = 0;
  std::string frames;  // accumulated JSON frame objects, comma-separated
  bool firstFrame = true;

  float cordLen(int pos) const {
    return initLen + pos * (MY_PI * spoolD / ticksRev);
  }

  void record() {
    Coord p = machine->getCoordinates();   // forward kinematics from the machine
    char buf[256];
    std::snprintf(buf, sizeof buf,
                  "%s{\"x\":%.3f,\"y\":%.3f,\"L\":%.3f,\"R\":%.3f,"
                  "\"lp\":%d,\"rp\":%d,\"pen\":%d}",
                  firstFrame ? "" : ",", p.x, p.y,
                  cordLen(leftPos), cordLen(rightPos), leftPos, rightPos,
                  penDown);
    frames += buf;
    firstFrame = false;
  }
};

SimState g;

// The simulator's stand-in for the motor driver. On hardware this would command
// the L293D and spin until the encoders reach the target, then the ISR-updated
// tick counts would be pushed back into the controller. Here we ADVANCE the tick
// counts by the requested delta (a perfect, instantaneous move — the only
// inaccuracy left is the same integer tick quantization the firmware has), push
// them into the machine via its real setLeftPosition/setRightPosition API, and
// snapshot a frame so the UI can animate the cords + pen.
void drive(PositionDelta d) {
  g.leftPos += d.left;
  g.rightPos += d.right;
  g.machine->setLeftPosition(g.leftPos);
  g.machine->setRightPosition(g.rightPos);
  g.record();
}

void usage(const char *p) {
  std::cerr
      << "Usage: " << p << " [--spacing MM]\n"
      << "                 [--spool-d MM] [--ticks-rev N] [--init-len MM]\n"
      << "  Reads painter polyline JSON from stdin, drives the real\n"
      << "  PolargraphState controller, writes a frame-trace JSON to stdout.\n"
      << "  --spacing   anchor (motor) spacing in mm           (default 800)\n"
      << "  --spool-d   spool diameter mm, both motors         (default 12)\n"
      << "  --ticks-rev encoder ticks per revolution           (default 360)\n"
      << "  --init-len  initial cord length mm at tick 0        (default 500)\n";
}

}  // namespace

int main(int argc, char **argv) {
  int spacing = 800;
  int spoolD = 12;
  int ticksRev = 360;
  int initLen = 500;

  for (int a = 1; a < argc; ++a) {
    std::string arg = argv[a];
    auto next = [&](int def) { return a + 1 < argc ? std::stoi(argv[++a]) : def; };
    if (arg == "--spacing") spacing = next(spacing);
    else if (arg == "--spool-d") spoolD = next(spoolD);
    else if (arg == "--ticks-rev") ticksRev = next(ticksRev);
    else if (arg == "--init-len") initLen = next(initLen);
    else if (arg == "-h" || arg == "--help") { usage(argv[0]); return 0; }
    else { std::cerr << "Unknown arg: " << arg << "\n"; usage(argv[0]); return 1; }
  }

  std::stringstream ss;
  ss << std::cin.rdbuf();
  std::string in = ss.str();
  std::vector<Polyline> polys = parsePolylines(in);

  // Source drawing box: the bounding box of the incoming polylines. This is the
  // coordinate space painter scaled into (0..W, 0..H), and the UI uses it to
  // place the drawing under the anchors.
  double minX = 1e300, minY = 1e300, maxX = -1e300, maxY = -1e300;
  for (const auto &pl : polys)
    for (const auto &p : pl) {
      minX = std::min(minX, p.x); minY = std::min(minY, p.y);
      maxX = std::max(maxX, p.x); maxY = std::max(maxY, p.y);
    }
  double boxW = (maxX > minX) ? maxX : 0;
  double boxH = (maxY > minY) ? maxY : 0;

  // The pen plots in machine coordinates: x to the right, y DOWNWARD below the
  // anchors. painter's drawing box starts at the top, so we drop it below the
  // motors by an offset that keeps cords taut (y must be > 0). Center the
  // drawing horizontally between the two anchors and hang it below.
  double drawW = (maxX > minX) ? (maxX - minX) : 0;
  double offX = (spacing - drawW) / 2.0 - (minX > 0 ? 0 : minX);
  if (offX < 0) offX = 0;
  // Hang the top of the drawing a comfortable distance below the anchor line.
  const double topMargin = std::max(150.0, boxH * 0.4);
  auto toMachine = [&](const Pt &p) -> Pt {
    return {p.x + offX, (p.y - minY) + topMargin};
  };

  // Build the two encoder "motors" and the controller, then home the pen to the
  // very first stroke's first point so initial cord lengths are sane. We give
  // both encoders the same calibration; their differing cord lengths come from
  // geometry, not calibration.
  EncoderState left(spoolD, ticksRev, initLen, 0);
  EncoderState right(spoolD, ticksRev, initLen, 0);

  // Seed the encoders so the pen actually starts at the first point. We compute
  // the cord lengths for that point and convert to tick positions, the same way
  // a homing routine would on the machine.
  PolargraphState machine(left, right, spacing);

  g.machine = &machine;
  g.leftPos = 0;
  g.rightPos = 0;
  g.spoolD = spoolD;
  g.ticksRev = ticksRev;
  g.initLen = initLen;

  // Home onto the first point before drawing so the run starts at the true
  // start with the pen up. moveTo interpolates from the machine's current
  // logical pen position (which getCoordinates() set at construction) to here,
  // recording a frame per segment — exactly how a real homing move would look.
  if (!polys.empty() && polys.front().size() >= 2) {
    Pt start = toMachine(polys.front().front());
    g.penDown = 0;
    machine.moveTo(start.x, start.y, drive);
  }

  // Plot: for each polyline, travel (pen up) to its first point, then draw
  // (pen down) through the rest. This mirrors painter's contract exactly.
  for (const auto &pl : polys) {
    if (pl.size() < 2) continue;
    Pt p0 = toMachine(pl[0]);
    g.penDown = 0;                 // travel
    machine.moveTo(p0.x, p0.y, drive);
    g.penDown = 1;                 // draw
    for (size_t i = 1; i < pl.size(); ++i) {
      Pt p = toMachine(pl[i]);
      machine.moveTo(p.x, p.y, drive);
    }
  }
  g.penDown = 0;

  // Emit the trace. Anchors: left at (0,0), right at (spacing,0).
  std::ostream &os = std::cout;
  os << "{\"anchorSpacing\":" << spacing
     << ",\"left\":{\"x\":0,\"y\":0,\"spool\":" << spoolD
     << ",\"ticksPerRev\":" << ticksRev << ",\"initLen\":" << initLen << "}"
     << ",\"right\":{\"x\":" << spacing << ",\"y\":0,\"spool\":" << spoolD
     << ",\"ticksPerRev\":" << ticksRev << ",\"initLen\":" << initLen << "}"
     << ",\"box\":{\"w\":" << boxW << ",\"h\":" << boxH << "}"
     << ",\"frames\":[" << g.frames << "]}";

  std::cerr << "sim: " << polys.size() << " polylines -> frames emitted, "
            << "spacing " << spacing << "mm, spool " << spoolD << "mm, "
            << ticksRev << " ticks/rev.\n";
  return 0;
}
