// painter.cpp — host-side SVG → drawing-coordinate generator.
//
// Reads an SVG, flattens every shape into straight-line polylines, scales them
// into a target width × height box, and prints the result as JSON:
//
//   [ [ [x,y], [x,y], ... ],   <- polyline 0: move to first point, pen down,
//     [ [x,y], [x,y], ... ],      draw through the rest, pen up
//     ... ]                     <- and so on for each polyline
//
// Each inner array is one contiguous stroke: travel to its first point with the
// pen UP, lower the pen, draw through the remaining points, then raise the pen.
//
// This is a HOST tool, not firmware. It is excluded from the PlatformIO/Arduino
// build via the ARDUINO guard below. Build & run on your computer:
//
//   g++ -std=c++17 -O2 src/painter.cpp -o painter
//   ./painter drawing.svg --width 600 --height 400 > coords.json
//   cat drawing.svg | ./painter --width 600 --height 400      # or via stdin
//
// Supported SVG: <line> <rect> <circle> <ellipse> <polyline> <polygon> and
// <path> with commands M m L l H h V v C c S s Q q T t A a Z z. Curves and arcs
// are flattened into line segments. Transforms and CSS styling are ignored.

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;

// Number of straight segments used when flattening a curve span. Higher = smoother.
constexpr int kCurveSteps = 24;

struct Point {
  double x = 0.0;
  double y = 0.0;
};

using Polyline = std::vector<Point>;

// One SVG element after flattening. `strokes` are the visible outline paths to
// plot. `fillRegions` are the closed polygons this element paints when it has a
// real fill; a later element's fillRegions occlude (clip) earlier strokes, just
// as the browser paints later shapes over earlier ones (z-order / painter's
// algorithm). An element with fill="none" (or no fill) has no fillRegions and
// therefore hides nothing.
struct Shape {
  std::vector<Polyline> strokes;
  std::vector<Polyline> fillRegions;
};

// ─────────────────────────── small parsing helpers ──────────────────────────

// Pull every signed/decimal/exponent number out of an SVG attribute value.
// Handles separators of commas, whitespace, and implicit signs (e.g. "1-2").
std::vector<double> parseNumbers(const std::string &s) {
  std::vector<double> out;
  size_t i = 0;
  const size_t n = s.size();
  while (i < n) {
    char c = s[i];
    bool start = (c == '+' || c == '-' || c == '.' || std::isdigit((unsigned char)c));
    if (!start) {
      ++i;
      continue;
    }
    size_t j = i;
    if (s[j] == '+' || s[j] == '-') ++j;
    bool sawDot = false;
    while (j < n) {
      char d = s[j];
      if (std::isdigit((unsigned char)d)) {
        ++j;
      } else if (d == '.' && !sawDot) {
        sawDot = true;
        ++j;
      } else if ((d == 'e' || d == 'E')) {
        ++j;
        if (j < n && (s[j] == '+' || s[j] == '-')) ++j;
      } else {
        break;
      }
    }
    try {
      out.push_back(std::stod(s.substr(i, j - i)));
    } catch (...) {
      // ignore malformed token
    }
    i = j;
  }
  return out;
}

// Find attribute value: name="..." within a single SVG element string.
bool getAttr(const std::string &el, const std::string &name, std::string &out) {
  std::string key = name + "=";
  size_t p = el.find(key);
  while (p != std::string::npos) {
    // ensure the char before key is a delimiter so "x=" doesn't match "rx="
    if (p == 0 || std::isspace((unsigned char)el[p - 1])) break;
    p = el.find(key, p + 1);
  }
  if (p == std::string::npos) return false;
  p += key.size();
  if (p >= el.size()) return false;
  char quote = el[p];
  if (quote != '"' && quote != '\'') return false;
  size_t end = el.find(quote, p + 1);
  if (end == std::string::npos) return false;
  out = el.substr(p + 1, end - p - 1);
  return true;
}

double attrNum(const std::string &el, const std::string &name, double def = 0.0) {
  std::string v;
  if (!getAttr(el, name, v)) return def;
  auto nums = parseNumbers(v);
  return nums.empty() ? def : nums[0];
}

// ───────────────────────────── shape flatteners ─────────────────────────────

void addEllipse(double cx, double cy, double rx, double ry,
                std::vector<Polyline> &out) {
  if (rx <= 0.0 || ry <= 0.0) return;
  Polyline pl;
  const int steps = std::max(kCurveSteps, 32);
  for (int i = 0; i <= steps; ++i) {
    double t = (2.0 * kPi * i) / steps;
    pl.push_back({cx + rx * std::cos(t), cy + ry * std::sin(t)});
  }
  out.push_back(std::move(pl));
}

// Cubic Bézier from p0 (current) through control points p1,p2 to p3.
void flattenCubic(const Point &p0, const Point &p1, const Point &p2,
                  const Point &p3, Polyline &pl) {
  for (int i = 1; i <= kCurveSteps; ++i) {
    double t = (double)i / kCurveSteps;
    double u = 1.0 - t;
    double a = u * u * u, b = 3 * u * u * t, c = 3 * u * t * t, d = t * t * t;
    pl.push_back({a * p0.x + b * p1.x + c * p2.x + d * p3.x,
                  a * p0.y + b * p1.y + c * p2.y + d * p3.y});
  }
}

// Quadratic Bézier from p0 through control p1 to p2.
void flattenQuad(const Point &p0, const Point &p1, const Point &p2,
                 Polyline &pl) {
  for (int i = 1; i <= kCurveSteps; ++i) {
    double t = (double)i / kCurveSteps;
    double u = 1.0 - t;
    double a = u * u, b = 2 * u * t, c = t * t;
    pl.push_back({a * p0.x + b * p1.x + c * p2.x,
                  a * p0.y + b * p1.y + c * p2.y});
  }
}

// SVG elliptical-arc (A/a) → line segments, appended to pl starting after p0.
void flattenArc(const Point &p0, double rx, double ry, double xAxisRotDeg,
                bool largeArc, bool sweep, const Point &p1, Polyline &pl) {
  if (rx == 0.0 || ry == 0.0 || (p0.x == p1.x && p0.y == p1.y)) {
    pl.push_back(p1);
    return;
  }
  rx = std::fabs(rx);
  ry = std::fabs(ry);
  double phi = xAxisRotDeg * kPi / 180.0;
  double cosP = std::cos(phi), sinP = std::sin(phi);

  double dx = (p0.x - p1.x) / 2.0, dy = (p0.y - p1.y) / 2.0;
  double x1p = cosP * dx + sinP * dy;
  double y1p = -sinP * dx + cosP * dy;

  // Correct out-of-range radii.
  double lambda = (x1p * x1p) / (rx * rx) + (y1p * y1p) / (ry * ry);
  if (lambda > 1.0) {
    double s = std::sqrt(lambda);
    rx *= s;
    ry *= s;
  }

  double num = rx * rx * ry * ry - rx * rx * y1p * y1p - ry * ry * x1p * x1p;
  double den = rx * rx * y1p * y1p + ry * ry * x1p * x1p;
  double co = den == 0.0 ? 0.0 : std::sqrt(std::max(0.0, num / den));
  if (largeArc == sweep) co = -co;
  double cxp = co * (rx * y1p) / ry;
  double cyp = co * -(ry * x1p) / rx;

  double cx = cosP * cxp - sinP * cyp + (p0.x + p1.x) / 2.0;
  double cy = sinP * cxp + cosP * cyp + (p0.y + p1.y) / 2.0;

  auto angle = [](double ux, double uy, double vx, double vy) {
    double dot = ux * vx + uy * vy;
    double len = std::sqrt((ux * ux + uy * uy) * (vx * vx + vy * vy));
    double a = std::acos(std::max(-1.0, std::min(1.0, len == 0 ? 1.0 : dot / len)));
    if (ux * vy - uy * vx < 0) a = -a;
    return a;
  };

  double theta1 = angle(1, 0, (x1p - cxp) / rx, (y1p - cyp) / ry);
  double dtheta = angle((x1p - cxp) / rx, (y1p - cyp) / ry,
                        (-x1p - cxp) / rx, (-y1p - cyp) / ry);
  if (!sweep && dtheta > 0) dtheta -= 2 * kPi;
  if (sweep && dtheta < 0) dtheta += 2 * kPi;

  int steps = std::max(2, (int)std::ceil(std::fabs(dtheta) / (kPi / 12)));
  for (int i = 1; i <= steps; ++i) {
    double t = theta1 + dtheta * ((double)i / steps);
    double ex = cosP * rx * std::cos(t) - sinP * ry * std::sin(t) + cx;
    double ey = sinP * rx * std::cos(t) + cosP * ry * std::sin(t) + cy;
    pl.push_back({ex, ey});
  }
}

// ───────────────────────────── path "d" parser ──────────────────────────────

// Tokenize a path data string into commands and numbers.
void parsePath(const std::string &d, std::vector<Polyline> &out) {
  std::vector<double> nums;  // number buffer for the current command
  size_t i = 0;
  const size_t n = d.size();

  Point cur{0, 0};       // current point
  Point start{0, 0};     // subpath start (for Z)
  Point prevCtrl{0, 0};  // last control point (for S/T reflection)
  char prevCmd = 0;
  Polyline pl;

  auto flushPolyline = [&]() {
    if (pl.size() >= 2) out.push_back(pl);
    pl.clear();
  };

  // Read one command letter and its following numbers into `nums`.
  auto readNumbersUntilCmd = [&]() {
    nums.clear();
    while (i < n) {
      char c = d[i];
      if (std::isalpha((unsigned char)c)) break;
      if (c == ',' || std::isspace((unsigned char)c)) {
        ++i;
        continue;
      }
      // parse a number starting here
      size_t j = i;
      if (d[j] == '+' || d[j] == '-') ++j;
      bool sawDot = false;
      while (j < n) {
        char e = d[j];
        if (std::isdigit((unsigned char)e)) {
          ++j;
        } else if (e == '.' && !sawDot) {
          sawDot = true;
          ++j;
        } else if (e == 'e' || e == 'E') {
          ++j;
          if (j < n && (d[j] == '+' || d[j] == '-')) ++j;
        } else {
          break;
        }
      }
      if (j == i) {
        ++i;
        continue;
      }
      try {
        nums.push_back(std::stod(d.substr(i, j - i)));
      } catch (...) {
      }
      i = j;
    }
  };

  while (i < n) {
    char c = d[i];
    if (std::isspace((unsigned char)c) || c == ',') {
      ++i;
      continue;
    }
    if (!std::isalpha((unsigned char)c)) {
      ++i;
      continue;
    }
    char cmd = c;
    ++i;
    readNumbersUntilCmd();
    bool rel = std::islower((unsigned char)cmd);
    char up = std::toupper((unsigned char)cmd);

    size_t k = 0;
    auto take = [&](int count) -> bool { return k + count <= nums.size(); };

    switch (up) {
      case 'M': {
        // first pair = moveto, subsequent pairs = implicit lineto
        if (!take(2)) break;
        flushPolyline();
        double x = nums[k++], y = nums[k++];
        if (rel) {
          x += cur.x;
          y += cur.y;
        }
        cur = {x, y};
        start = cur;
        pl.push_back(cur);
        while (take(2)) {
          double lx = nums[k++], ly = nums[k++];
          if (rel) {
            lx += cur.x;
            ly += cur.y;
          }
          cur = {lx, ly};
          pl.push_back(cur);
        }
        break;
      }
      case 'L': {
        while (take(2)) {
          double x = nums[k++], y = nums[k++];
          if (rel) {
            x += cur.x;
            y += cur.y;
          }
          cur = {x, y};
          pl.push_back(cur);
        }
        break;
      }
      case 'H': {
        while (take(1)) {
          double x = nums[k++];
          if (rel) x += cur.x;
          cur = {x, cur.y};
          pl.push_back(cur);
        }
        break;
      }
      case 'V': {
        while (take(1)) {
          double y = nums[k++];
          if (rel) y += cur.y;
          cur = {cur.x, y};
          pl.push_back(cur);
        }
        break;
      }
      case 'C': {
        while (take(6)) {
          Point c1{nums[k], nums[k + 1]}, c2{nums[k + 2], nums[k + 3]},
              end{nums[k + 4], nums[k + 5]};
          k += 6;
          if (rel) {
            c1 = {c1.x + cur.x, c1.y + cur.y};
            c2 = {c2.x + cur.x, c2.y + cur.y};
            end = {end.x + cur.x, end.y + cur.y};
          }
          flattenCubic(cur, c1, c2, end, pl);
          prevCtrl = c2;
          cur = end;
        }
        break;
      }
      case 'S': {
        while (take(4)) {
          Point c2{nums[k], nums[k + 1]}, end{nums[k + 2], nums[k + 3]};
          k += 4;
          if (rel) {
            c2 = {c2.x + cur.x, c2.y + cur.y};
            end = {end.x + cur.x, end.y + cur.y};
          }
          Point c1 = cur;
          if (prevCmd == 'C' || prevCmd == 'S')
            c1 = {2 * cur.x - prevCtrl.x, 2 * cur.y - prevCtrl.y};
          flattenCubic(cur, c1, c2, end, pl);
          prevCtrl = c2;
          cur = end;
        }
        break;
      }
      case 'Q': {
        while (take(4)) {
          Point c1{nums[k], nums[k + 1]}, end{nums[k + 2], nums[k + 3]};
          k += 4;
          if (rel) {
            c1 = {c1.x + cur.x, c1.y + cur.y};
            end = {end.x + cur.x, end.y + cur.y};
          }
          flattenQuad(cur, c1, end, pl);
          prevCtrl = c1;
          cur = end;
        }
        break;
      }
      case 'T': {
        while (take(2)) {
          Point end{nums[k], nums[k + 1]};
          k += 2;
          if (rel) end = {end.x + cur.x, end.y + cur.y};
          Point c1 = cur;
          if (prevCmd == 'Q' || prevCmd == 'T')
            c1 = {2 * cur.x - prevCtrl.x, 2 * cur.y - prevCtrl.y};
          flattenQuad(cur, c1, end, pl);
          prevCtrl = c1;
          cur = end;
        }
        break;
      }
      case 'A': {
        while (take(7)) {
          double rx = nums[k], ry = nums[k + 1], rot = nums[k + 2];
          bool large = nums[k + 3] != 0.0, sweep = nums[k + 4] != 0.0;
          Point end{nums[k + 5], nums[k + 6]};
          k += 7;
          if (rel) end = {end.x + cur.x, end.y + cur.y};
          flattenArc(cur, rx, ry, rot, large, sweep, end, pl);
          cur = end;
        }
        break;
      }
      case 'Z': {
        if (!pl.empty()) {
          pl.push_back(start);
          cur = start;
        }
        flushPolyline();
        break;
      }
      default:
        break;
    }
    prevCmd = up;
  }
  flushPolyline();
}

// ───────────────────────── element extraction ───────────────────────────────

// Does this element have a real fill (i.e. paints a region that occludes
// whatever is drawn before it)? SVG fill defaults to black, so a missing fill
// attribute still fills; only fill="none" (or a fully transparent fill) does
// not. We treat fill-opacity/opacity == 0 as "no fill" too.
bool elementFills(const std::string &el) {
  std::string fill;
  if (getAttr(el, "fill", fill)) {
    // trim and lowercase for the "none" / "transparent" check
    std::string f;
    for (char c : fill)
      if (!std::isspace((unsigned char)c)) f += (char)std::tolower((unsigned char)c);
    if (f == "none" || f == "transparent") return false;
  }
  // A zero fill-opacity or element opacity makes the fill invisible.
  std::string v;
  if (getAttr(el, "fill-opacity", v) && parseNumbers(v).size() &&
      parseNumbers(v)[0] == 0.0)
    return false;
  if (getAttr(el, "opacity", v) && parseNumbers(v).size() &&
      parseNumbers(v)[0] == 0.0)
    return false;
  return true;
}

// True if a polyline is (approximately) closed — first point == last point.
bool isClosed(const Polyline &pl) {
  if (pl.size() < 3) return false;
  const Point &a = pl.front(), &b = pl.back();
  double dx = a.x - b.x, dy = a.y - b.y;
  return dx * dx + dy * dy < 1e-9;
}

// Walk the raw SVG text element by element, flattening each shape into a Shape
// (its visible strokes plus any closed fill regions) in document order.
void extractShapes(const std::string &svg, std::vector<Shape> &out) {
  size_t i = 0;
  while ((i = svg.find('<', i)) != std::string::npos) {
    size_t end = svg.find('>', i);
    if (end == std::string::npos) break;
    std::string el = svg.substr(i, end - i + 1);
    i = end + 1;
    if (el.size() < 2 || el[1] == '/' || el[1] == '!' || el[1] == '?') continue;

    // element tag name
    size_t t = 1;
    while (t < el.size() && (std::isalnum((unsigned char)el[t]) || el[t] == ':'))
      ++t;
    std::string tag = el.substr(1, t - 1);

    std::vector<Polyline> polys;
    if (tag == "line") {
      polys.push_back({{attrNum(el, "x1"), attrNum(el, "y1")},
                       {attrNum(el, "x2"), attrNum(el, "y2")}});
    } else if (tag == "rect") {
      double x = attrNum(el, "x"), y = attrNum(el, "y");
      double w = attrNum(el, "width"), h = attrNum(el, "height");
      if (w > 0 && h > 0)
        polys.push_back(
            {{x, y}, {x + w, y}, {x + w, y + h}, {x, y + h}, {x, y}});
    } else if (tag == "circle") {
      double r = attrNum(el, "r");
      addEllipse(attrNum(el, "cx"), attrNum(el, "cy"), r, r, polys);
    } else if (tag == "ellipse") {
      addEllipse(attrNum(el, "cx"), attrNum(el, "cy"), attrNum(el, "rx"),
                 attrNum(el, "ry"), polys);
    } else if (tag == "polyline" || tag == "polygon") {
      std::string pts;
      if (getAttr(el, "points", pts)) {
        auto nums = parseNumbers(pts);
        Polyline pl;
        for (size_t j = 0; j + 1 < nums.size(); j += 2)
          pl.push_back({nums[j], nums[j + 1]});
        if (tag == "polygon" && pl.size() >= 2) pl.push_back(pl.front());
        if (pl.size() >= 2) polys.push_back(std::move(pl));
      }
    } else if (tag == "path") {
      std::string d;
      if (getAttr(el, "d", d)) parsePath(d, polys);
    } else {
      continue;
    }
    if (polys.empty()) continue;

    Shape sh;
    sh.strokes = polys;
    if (elementFills(el))
      for (const auto &pl : polys)
        if (isClosed(pl)) sh.fillRegions.push_back(pl);
    out.push_back(std::move(sh));
  }
}

// ─────────────────────────── occlusion clipping ─────────────────────────────

// Winding/crossing-number point-in-polygon test. `poly` is assumed closed.
bool pointInPolygon(const Point &p, const Polyline &poly) {
  bool inside = false;
  size_t n = poly.size();
  for (size_t a = 0, b = n - 1; a < n; b = a++) {
    const Point &pa = poly[a], &pb = poly[b];
    bool straddles = (pa.y > p.y) != (pb.y > p.y);
    if (straddles) {
      double xCross =
          (pb.x - pa.x) * (p.y - pa.y) / (pb.y - pa.y) + pa.x;
      if (p.x < xCross) inside = !inside;
    }
  }
  return inside;
}

// Is point p covered by ANY of the occluder fill regions?
bool covered(const Point &p, const std::vector<const Polyline *> &occluders) {
  for (const Polyline *poly : occluders)
    if (pointInPolygon(p, *poly)) return true;
  return false;
}

// Clip a single stroke against the occluders, splitting it into visible
// sub-strokes. Each segment is sampled at fine sub-steps so that the entry and
// exit of a fill region are caught even when endpoints straddle a boundary.
void clipStroke(const Polyline &stroke,
                const std::vector<const Polyline *> &occluders,
                std::vector<Polyline> &out) {
  if (occluders.empty()) {
    if (stroke.size() >= 2) out.push_back(stroke);
    return;
  }
  // Densify: emit visible points, breaking the run wherever we cross into a
  // covered region.
  constexpr int kSub = 8;  // sub-samples per source segment
  Polyline run;
  auto flush = [&]() {
    if (run.size() >= 2) out.push_back(run);
    run.clear();
  };
  auto consider = [&](const Point &p) {
    if (covered(p, occluders)) {
      flush();
    } else {
      run.push_back(p);
    }
  };

  consider(stroke.front());
  for (size_t s = 1; s < stroke.size(); ++s) {
    const Point &a = stroke[s - 1], &b = stroke[s];
    for (int k = 1; k <= kSub; ++k) {
      double t = (double)k / kSub;
      consider({a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t});
    }
  }
  flush();
}

// Apply z-order occlusion: for each shape, clip its strokes against the union
// of all *later* shapes' fill regions, since those are painted on top.
std::vector<Polyline> resolveOcclusion(const std::vector<Shape> &shapes) {
  std::vector<Polyline> out;
  for (size_t i = 0; i < shapes.size(); ++i) {
    std::vector<const Polyline *> occluders;
    for (size_t j = i + 1; j < shapes.size(); ++j)
      for (const auto &fr : shapes[j].fillRegions) occluders.push_back(&fr);
    for (const auto &stroke : shapes[i].strokes)
      clipStroke(stroke, occluders, out);
  }
  return out;
}

// ───────────────────────── pen-travel minimization ──────────────────────────
// painter emits polylines in document/contour order, which on a detailed image
// makes the pen leap all over the page between strokes (pen-up travel). That
// wasted motion is slow and, on a polargraph, accumulates positioning error.
//
// Reducing it is an open-loop TSP over the strokes, where each stroke is a
// "city" that can be visited from EITHER endpoint (we may draw it head→tail or
// tail→head). We solve it in two stages, both designed to stay fast as the
// stroke count climbs into the tens of thousands:
//
//   1. Construction: greedy nearest-neighbor, but accelerated with a uniform
//      spatial grid over all stroke endpoints. Instead of scanning every
//      remaining stroke (O(n) per pick, O(n^2) total), each pick searches only
//      the grid cells in expanding rings around the pen — O(1) average per pick,
//      so ~O(n) overall on evenly spread art.
//
//   2. Refinement: Or-opt + 2-opt passes over the resulting sequence to undo the
//      long "return" hops greedy leaves behind. Both are time-boxed so they
//      never dominate runtime; on big inputs they take whatever budget is left
//      and stop. They only reorder/reverse strokes — never change WHAT is drawn.
double sqDist(const Point &a, const Point &b) {
  double dx = a.x - b.x, dy = a.y - b.y;
  return dx * dx + dy * dy;
}

double travelDistance(const std::vector<Polyline> &polys) {
  // Sum of pen-up hops: from the origin to stroke 0's start, then each stroke's
  // end to the next stroke's start. (Drawing length within a stroke is fixed.)
  double total = 0.0;
  Point pen{0, 0};
  for (const auto &pl : polys) {
    if (pl.empty()) continue;
    total += std::sqrt(sqDist(pen, pl.front()));
    pen = pl.back();
  }
  return total;
}

using Clock = std::chrono::steady_clock;
double elapsedMs(Clock::time_point t0) {
  return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}

// A uniform grid of stroke endpoints for fast nearest-endpoint queries. Each
// stroke contributes its two endpoints (head = slot 0, tail = slot 1). A query
// walks outward in square rings of cells and stops once no closer endpoint can
// possibly lie in a farther ring.
struct EndpointGrid {
  struct Entry { int stroke; int slot; };  // slot: 0=head, 1=tail
  double minX = 0, minY = 0, cell = 1;
  int cols = 1, rows = 1;
  std::unordered_map<int64_t, std::vector<Entry>> cells;

  int64_t key(int cx, int cy) const { return (int64_t)cy * cols + cx; }
  int cx(double x) const {
    int c = (int)((x - minX) / cell);
    return c < 0 ? 0 : (c >= cols ? cols - 1 : c);
  }
  int cy(double y) const {
    int r = (int)((y - minY) / cell);
    return r < 0 ? 0 : (r >= rows ? rows - 1 : r);
  }

  void build(const std::vector<Polyline> &polys) {
    double maxX = -1e300, maxY = -1e300;
    minX = minY = 1e300;
    size_t pts = 0;
    for (const auto &pl : polys) {
      if (pl.empty()) continue;
      for (const Point &p : {pl.front(), pl.back()}) {
        minX = std::min(minX, p.x); minY = std::min(minY, p.y);
        maxX = std::max(maxX, p.x); maxY = std::max(maxY, p.y);
      }
      ++pts;
    }
    if (pts == 0) { minX = minY = 0; }
    double w = std::max(1e-6, maxX - minX), h = std::max(1e-6, maxY - minY);
    // Target ~1 endpoint per cell on average: cell ≈ sqrt(area / count).
    cell = std::max(1e-6, std::sqrt((w * h) / std::max<size_t>(1, pts)));
    cols = std::max(1, (int)(w / cell) + 1);
    rows = std::max(1, (int)(h / cell) + 1);
    cells.reserve(pts * 2);
    for (size_t i = 0; i < polys.size(); ++i) {
      if (polys[i].empty()) continue;
      cells[key(cx(polys[i].front().x), cy(polys[i].front().y))]
          .push_back({(int)i, 0});
      cells[key(cx(polys[i].back().x), cy(polys[i].back().y))]
          .push_back({(int)i, 1});
    }
  }

  void erase(int stroke, int slot, const Point &p) {
    auto it = cells.find(key(cx(p.x), cy(p.y)));
    if (it == cells.end()) return;
    auto &v = it->second;
    for (size_t k = 0; k < v.size(); ++k)
      if (v[k].stroke == stroke && v[k].slot == slot) {
        v[k] = v.back();
        v.pop_back();
        return;
      }
  }

  // Nearest live endpoint to `from` whose stroke is not `used`. Returns stroke
  // index (or -1) and writes the winning slot.
  int nearest(const Point &from, const std::vector<bool> &used,
              const std::vector<Polyline> &polys, int &outSlot) const {
    int gx = cx(from.x), gy = cy(from.y);
    int best = -1;
    double bestD = 1e300;
    int maxRing = std::max(cols, rows);
    for (int ring = 0; ring <= maxRing; ++ring) {
      // Once we have a hit, a ring whose nearest possible cell edge is farther
      // than the current best cannot improve it — stop.
      if (best >= 0) {
        double ringDist = (double)(ring - 1) * cell;
        if (ringDist > 0 && ringDist * ringDist > bestD) break;
      }
      int x0 = gx - ring, x1 = gx + ring, y0 = gy - ring, y1 = gy + ring;
      for (int yy = y0; yy <= y1; ++yy) {
        if (yy < 0 || yy >= rows) continue;
        bool edgeRow = (yy == y0 || yy == y1);
        for (int xx = x0; xx <= x1; ++xx) {
          if (xx < 0 || xx >= cols) continue;
          // Only the ring's perimeter is new; skip the interior we already saw.
          if (ring > 0 && !edgeRow && xx != x0 && xx != x1) continue;
          auto it = cells.find(key(xx, yy));
          if (it == cells.end()) continue;
          for (const Entry &e : it->second) {
            if (used[e.stroke]) continue;
            const Point &p = e.slot == 0 ? polys[e.stroke].front()
                                         : polys[e.stroke].back();
            double d = sqDist(from, p);
            if (d < bestD) { bestD = d; best = e.stroke; outSlot = e.slot; }
          }
        }
      }
    }
    return best;
  }
};

// Stage 1: grid-accelerated greedy nearest-neighbor. Produces an order plus a
// per-stroke "reversed" flag (true = draw tail→head). Strokes are NOT physically
// reversed yet; we keep that lazy so refinement can flip orientation for free.
void greedyOrder(const std::vector<Polyline> &polys,
                 std::vector<int> &order, std::vector<char> &rev) {
  const size_t n = polys.size();
  EndpointGrid grid;
  grid.build(polys);

  std::vector<bool> used(n, false);
  order.clear();
  order.reserve(n);
  rev.assign(n, 0);

  Point pen{0, 0};  // pen starts at the origin, same assumption sim/firmware use
  for (size_t step = 0; step < n; ++step) {
    int slot = 0;
    int s = grid.nearest(pen, used, polys, slot);
    if (s < 0) break;
    used[s] = true;
    grid.erase(s, 0, polys[s].front());
    grid.erase(s, 1, polys[s].back());
    rev[s] = (slot == 1) ? 1 : 0;       // entered from the tail → reverse
    order.push_back(s);
    pen = rev[s] ? polys[s].front() : polys[s].back();  // exit endpoint
  }
  // Any leftovers (shouldn't happen unless n==0) appended in original order.
  for (size_t i = 0; i < n; ++i)
    if (!used[i]) order.push_back((int)i);
}

// Endpoints of stroke `idx` as drawn, honoring its reversed flag.
inline const Point &entryPt(const std::vector<Polyline> &p,
                            const std::vector<char> &rev, int idx) {
  return rev[idx] ? p[idx].back() : p[idx].front();
}
inline const Point &exitPt(const std::vector<Polyline> &p,
                           const std::vector<char> &rev, int idx) {
  return rev[idx] ? p[idx].front() : p[idx].back();
}

// Pen-up hop from the exit of `a` (or the origin) to the entry of `b`.
inline double hop(const std::vector<Polyline> &p, const std::vector<char> &rev,
                  int a, int b) {
  Point from = (a < 0) ? Point{0, 0} : exitPt(p, rev, a);
  return std::sqrt(sqDist(from, entryPt(p, rev, b)));
}

// Minimum relative gain we bother applying — guards against float oscillation
// (a move and its near-equal inverse flip-flopping forever).
constexpr double kEps = 1e-7;

// Returns true if it made at least one improving move this sweep.
//
// Stage 2a: 2-opt. Reversing order[i..j] flips both the segment order AND each
// stroke's drawing direction. The internal hops are PRESERVED: a reversed
// adjacency traverses the same two physical endpoints in the opposite direction,
// and Euclidean distance is symmetric. So only the two boundary hops change —
// a→b becomes a→c', and c→d becomes b'→d (primes = reversed) — which is what we
// evaluate. Window-limited so one sweep is O(n·window).
bool twoOptSweep(const std::vector<Polyline> &polys, std::vector<int> &order,
                 std::vector<char> &rev, Clock::time_point t0, double budgetMs,
                 int window) {
  const int n = (int)order.size();
  if (n < 3) return false;
  bool any = false;
  for (int i = 0; i < n - 1; ++i) {
    if ((i & 1023) == 0 && elapsedMs(t0) > budgetMs) return any;
    int a = (i == 0) ? -1 : order[i - 1];
    int jMax = std::min(n - 1, i + window);
    for (int j = i + 1; j <= jMax; ++j) {
      int b = order[i];
      int c = order[j];
      int d = (j + 1 < n) ? order[j + 1] : -2;  // -2 = nothing after
      double before = hop(polys, rev, a, b);
      if (d != -2) before += hop(polys, rev, c, d);
      // Evaluate with b and c reversed (their entry/exit swap after the flip).
      rev[b] = !rev[b]; rev[c] = !rev[c];
      double after = hop(polys, rev, a, c);
      if (d != -2) after += hop(polys, rev, b, d);
      rev[b] = !rev[b]; rev[c] = !rev[c];  // restore
      if (before - after > kEps) {
        std::reverse(order.begin() + i, order.begin() + j + 1);
        for (int k = i; k <= j; ++k) rev[order[k]] = !rev[order[k]];
        any = true;
        break;  // order[i] changed; move on to the next i
      }
    }
  }
  return any;
}

// Stage 2b: Or-opt. Lift a short run of 1–3 strokes and reinsert it (optionally
// reversed) at the cheapest nearby gap — catches a stroke stranded far from its
// neighbors, which 2-opt's contiguous reversal can't fix. Window-limited.
bool orOptSweep(const std::vector<Polyline> &polys, std::vector<int> &order,
                std::vector<char> &rev, Clock::time_point t0, double budgetMs,
                int window) {
  const int n = (int)order.size();
  if (n < 4) return false;
  bool any = false;
  for (int segLen = 1; segLen <= 3; ++segLen) {
    for (int i = 1; i + segLen <= n; ++i) {
      if ((i & 1023) == 0 && elapsedMs(t0) > budgetMs) return any;
      int p = order[i - 1];
      int s0 = order[i], s1 = order[i + segLen - 1];
      int q = (i + segLen < n) ? order[i + segLen] : -2;
      // Gain from removing the segment: p→s0 (+ s1→q) collapses to p→q.
      double removed = hop(polys, rev, p, s0);
      if (q != -2) removed += hop(polys, rev, s1, q) - hop(polys, rev, p, q);
      int jMax = std::min(n, i + window);
      for (int j = i + segLen + 1; j <= jMax; ++j) {  // skip the no-op gap
        int u = order[j - 1];
        int v = (j < n) ? order[j] : -2;
        double base = (v == -2) ? 0.0 : hop(polys, rev, u, v);
        double fwd = hop(polys, rev, u, s0)
                   + ((v == -2) ? 0.0 : hop(polys, rev, s1, v)) - base;
        // reversed insertion: flip the segment's directions to measure
        for (int k = i; k < i + segLen; ++k) rev[order[k]] = !rev[order[k]];
        double bwd = hop(polys, rev, u, s1)
                   + ((v == -2) ? 0.0 : hop(polys, rev, s0, v)) - base;
        for (int k = i; k < i + segLen; ++k) rev[order[k]] = !rev[order[k]];
        double bestIns = std::min(fwd, bwd);
        if (removed - bestIns > kEps) {
          std::vector<int> seg(order.begin() + i, order.begin() + i + segLen);
          if (bwd < fwd) {
            std::reverse(seg.begin(), seg.end());
            for (int s : seg) rev[s] = !rev[s];
          }
          order.erase(order.begin() + i, order.begin() + i + segLen);
          int dst = j - segLen;  // j > i always here, so the removal shifts it
          order.insert(order.begin() + dst, seg.begin(), seg.end());
          any = true;
          break;  // segment moved; advance i
        }
      }
    }
  }
  return any;
}

// Top-level: build the order, refine it, and materialize the reordered (and, as
// needed, reversed) polylines back into `polys`. `budgetMs` caps refinement.
void orderForMinTravel(std::vector<Polyline> &polys, double budgetMs = 750.0) {
  const size_t n = polys.size();
  if (n < 2) return;

  Clock::time_point t0 = Clock::now();
  std::vector<int> order;
  std::vector<char> rev;
  greedyOrder(polys, order, rev);

  // Alternate 2-opt and Or-opt sweeps. Each sweep returns whether it improved;
  // we stop when a full round (both passes) makes no change, when the time box
  // is hit, or at a hard sweep cap — three independent guarantees that this
  // terminates regardless of float edge cases. The neighbor window scales down
  // with size so a sweep stays affordable on huge inputs.
  int window = (n > 20000) ? 30 : (n > 4000 ? 60 : 200);
  const int kMaxSweeps = 60;
  for (int sweep = 0; sweep < kMaxSweeps && elapsedMs(t0) < budgetMs; ++sweep) {
    bool a = twoOptSweep(polys, order, rev, t0, budgetMs, window);
    bool b = orOptSweep(polys, order, rev, t0, budgetMs, window);
    if (!a && !b) break;  // converged
  }

  // Apply: emit strokes in the chosen order, reversing flagged ones in place.
  std::vector<Polyline> out;
  out.reserve(n);
  for (int idx : order) {
    Polyline pl = std::move(polys[idx]);
    if (rev[idx]) std::reverse(pl.begin(), pl.end());
    out.push_back(std::move(pl));
  }
  polys.swap(out);
}

// Determine the SVG source coordinate box from viewBox or width/height.
struct ViewBox {
  double minX = 0, minY = 0, w = 0, h = 0;
  bool valid = false;
};

ViewBox findViewBox(const std::string &svg, const std::vector<Polyline> &shapes) {
  ViewBox vb;
  size_t s = svg.find("<svg");
  if (s != std::string::npos) {
    size_t e = svg.find('>', s);
    std::string el = svg.substr(s, e == std::string::npos ? std::string::npos : e - s + 1);
    std::string vbStr;
    if (getAttr(el, "viewBox", vbStr)) {
      auto n = parseNumbers(vbStr);
      if (n.size() == 4 && n[2] > 0 && n[3] > 0) {
        vb = {n[0], n[1], n[2], n[3], true};
        return vb;
      }
    }
    double w = attrNum(el, "width"), h = attrNum(el, "height");
    if (w > 0 && h > 0) {
      vb = {0, 0, w, h, true};
      return vb;
    }
  }
  // Fall back to the bounding box of the geometry itself.
  double minX = 1e300, minY = 1e300, maxX = -1e300, maxY = -1e300;
  for (const auto &pl : shapes)
    for (const auto &p : pl) {
      minX = std::min(minX, p.x);
      minY = std::min(minY, p.y);
      maxX = std::max(maxX, p.x);
      maxY = std::max(maxY, p.y);
    }
  if (maxX > minX && maxY > minY) {
    vb = {minX, minY, maxX - minX, maxY - minY, true};
  }
  return vb;
}

void printUsage(const char *prog) {
  std::cerr << "Usage: " << prog
            << " [file.svg] --width W --height H [--no-fit] [--flip-y]\n"
            << "  Reads SVG from FILE or stdin, writes polyline JSON to stdout.\n"
            << "  --width / --height : target drawing box (required)\n"
            << "  --no-fit  : scale X and Y independently to fill the box\n"
            << "              (default preserves aspect ratio and centers)\n"
            << "  --flip-y  : origin at bottom-left (y grows up)\n"
            << "  --no-optimize : keep document stroke order; do NOT reorder\n"
            << "              strokes to minimize pen-up travel (default: reorder)\n";
}

}  // namespace

int main(int argc, char **argv) {
  std::string path;
  double targetW = 0, targetH = 0;
  bool fit = true;   // preserve aspect ratio
  bool flipY = false;
  bool optimize = true;  // reorder strokes to minimize pen-up travel

  for (int a = 1; a < argc; ++a) {
    std::string arg = argv[a];
    if (arg == "--width" && a + 1 < argc) {
      targetW = std::stod(argv[++a]);
    } else if (arg == "--height" && a + 1 < argc) {
      targetH = std::stod(argv[++a]);
    } else if (arg == "--no-fit") {
      fit = false;
    } else if (arg == "--flip-y") {
      flipY = true;
    } else if (arg == "--no-optimize") {
      optimize = false;
    } else if (arg == "-h" || arg == "--help") {
      printUsage(argv[0]);
      return 0;
    } else if (!arg.empty() && arg[0] != '-') {
      path = arg;
    } else {
      std::cerr << "Unknown argument: " << arg << "\n";
      printUsage(argv[0]);
      return 1;
    }
  }

  if (targetW <= 0 || targetH <= 0) {
    std::cerr << "Error: --width and --height are required and must be > 0.\n";
    printUsage(argv[0]);
    return 1;
  }

  // Read the SVG (file or stdin).
  std::string svg;
  if (!path.empty()) {
    std::ifstream f(path, std::ios::binary);
    if (!f) {
      std::cerr << "Error: cannot open " << path << "\n";
      return 1;
    }
    std::stringstream ss;
    ss << f.rdbuf();
    svg = ss.str();
  } else {
    std::stringstream ss;
    ss << std::cin.rdbuf();
    svg = ss.str();
  }
  if (svg.empty()) {
    std::cerr << "Error: empty SVG input.\n";
    return 1;
  }

  std::vector<Shape> elements;
  extractShapes(svg, elements);

  // Honor z-order: clip each shape's strokes against the fill regions of the
  // shapes painted after it, so hidden lines (e.g. the tops of the feet behind
  // the body) are not drawn.
  std::vector<Polyline> shapes = resolveOcclusion(elements);

  ViewBox vb = findViewBox(svg, shapes);
  if (!vb.valid) {
    std::cerr << "Error: could not determine SVG dimensions (no viewBox, "
                 "width/height, or drawable geometry).\n";
    return 1;
  }

  // Minimize pen-up travel by reordering (and possibly reversing) strokes via a
  // greedy nearest-neighbor pass. Done in SOURCE coordinates — the transform is
  // affine and order-independent, so the same ordering is optimal after scaling.
  double travelBefore = 0, travelAfter = 0;
  if (optimize && shapes.size() > 1) {
    travelBefore = travelDistance(shapes);
    orderForMinTravel(shapes);
    travelAfter = travelDistance(shapes);
  }

  // Build the source→target transform.
  double sx = targetW / vb.w;
  double sy = targetH / vb.h;
  double offX = 0, offY = 0;
  if (fit) {
    double s = std::min(sx, sy);
    sx = sy = s;
    offX = (targetW - vb.w * s) / 2.0;  // center within the box
    offY = (targetH - vb.h * s) / 2.0;
  }

  auto mapPoint = [&](const Point &p) -> Point {
    double x = (p.x - vb.minX) * sx + offX;
    double y = (p.y - vb.minY) * sy + offY;
    if (flipY) y = targetH - y;
    return {x, y};
  };

  // Emit JSON: array of polylines, each an array of [x, y] pairs.
  std::ostream &os = std::cout;
  os.setf(std::ios::fixed);
  os.precision(3);
  os << "[";
  bool firstPl = true;
  for (const auto &pl : shapes) {
    if (pl.size() < 2) continue;  // skip degenerate strokes
    if (!firstPl) os << ",";
    firstPl = false;
    os << "\n  [";
    for (size_t j = 0; j < pl.size(); ++j) {
      Point m = mapPoint(pl[j]);
      if (j) os << ",";
      os << "[" << m.x << "," << m.y << "]";
    }
    os << "]";
  }
  os << "\n]\n";

  std::cerr << "painter: emitted " << shapes.size() << " visible polylines ("
            << elements.size() << " elements parsed, z-order clipped) from a "
            << vb.w << "x" << vb.h << " source box into " << targetW << "x"
            << targetH << ".\n";
  if (optimize && travelBefore > 0) {
    double pct = 100.0 * (travelBefore - travelAfter) / travelBefore;
    std::cerr << "painter: pen-up travel reordered " << travelBefore
              << " -> " << travelAfter << " src-units (" << pct
              << "% less travel).\n";
  }
  return 0;
}

