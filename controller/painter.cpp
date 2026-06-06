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

// Total DRAWN length: the origin→first-point hop plus every segment within and
// between polylines (i.e. the whole path the pen traces). For the single-line
// pipeline this is the quantity being minimized, since the pen is always down.
double drawnLength(const std::vector<Polyline> &polys) {
  double total = 0.0;
  Point pen{0, 0};
  for (const auto &pl : polys) {
    if (pl.empty()) continue;
    total += std::sqrt(sqDist(pen, pl.front()));  // move/connector to its start
    for (size_t i = 1; i < pl.size(); ++i)
      total += std::sqrt(sqDist(pl[i - 1], pl[i]));  // drawn segments
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
//
// `penDown`: in single-line mode the origin→first hop is itself a DRAWN segment,
// so the first stroke is worth relocating too. We then let i start at 0 with the
// origin (-1) as its predecessor; in pen-up mode we keep i≥1 (the origin hop is
// short homing travel not worth churning the route over) so that mode is
// unchanged.
bool orOptSweep(const std::vector<Polyline> &polys, std::vector<int> &order,
                std::vector<char> &rev, Clock::time_point t0, double budgetMs,
                int window, bool penDown) {
  const int n = (int)order.size();
  if (n < 4) return false;
  bool any = false;
  const int iStart = penDown ? 0 : 1;
  for (int segLen = 1; segLen <= 3; ++segLen) {
    for (int i = iStart; i + segLen <= n; ++i) {
      if ((i & 1023) == 0 && elapsedMs(t0) > budgetMs) return any;
      int p = (i == 0) ? -1 : order[i - 1];   // -1 = origin
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
//
// `penDown` = single-line mode: the inter-stroke connectors are DRAWN, not
// pen-up travel, so shortening them improves the visible output. The objective
// (sum of connectors + the origin→first hop) is the same quantity either way —
// minimizing pen-up travel and minimizing the drawn connector length are the
// same TSP — but in pen-down mode it also pays to optimize the first stroke's
// placement, which we enable in Or-opt.
void orderForMinTravel(std::vector<Polyline> &polys, double budgetMs = 750.0,
                       bool penDown = false) {
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
    bool b = orOptSweep(polys, order, rev, t0, budgetMs, window, penDown);
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

// ───────────────── single-line pipeline B: shortest continuous path ─────────
// Pipeline A (mergeIntoSingleLine) keeps each stroke rigid and just orders them,
// so every gap between strokes becomes a drawn connector. Pipeline B does much
// better when strokes share geometry: it treats the whole drawing as a GRAPH
// (vertices = points, edges = segments) and finds a single continuous pen-down
// path that covers every edge with the least extra drawing — the classic Route
// Inspection / Chinese-Postman problem.
//
// A connected graph has an Eulerian path (each edge traversed exactly once, pen
// never lifts, zero waste) iff it has 0 or 2 odd-degree vertices. Real drawings
// break two assumptions, which we repair with the minimum extra length we can
// find cheaply:
//   1. Multiple disconnected shapes  → bridge components together (greedy
//      nearest-endpoint, an MST-style join). Each bridge is drawn twice (out and
//      back) in the worst case, but is the shortest connection available.
//   2. Too many odd-degree vertices  → pair them up and duplicate the connecting
//      segment so both become even (greedy nearest-pair matching — the standard
//      CPP heuristic, typically within a few % of the optimal blossom matching).
// Then Hierholzer's algorithm walks the now-Eulerian multigraph into one path.
//
// The result is a single polyline whose drawn length is at or near the minimum
// possible for "one stroke, pen never lifts" — strictly shorter than pipeline A
// whenever the drawing has shared/branching geometry.
namespace euler {

struct Vertex {
  Point p;
};

// Weld points that are within `eps` into shared vertices so touching strokes
// actually connect in the graph. Returns vertex list; fills each input point's
// vertex id.
struct Graph {
  std::vector<Point> verts;
  // adjacency: for each vertex, list of (neighbor vertex, edge id)
  std::vector<std::vector<std::pair<int, int>>> adj;
  int edgeCount = 0;
};

// Quantized spatial hash for welding coincident endpoints.
int weld(std::unordered_map<int64_t, int> &lookup, std::vector<Point> &verts,
         const Point &p, double inv) {
  int64_t gx = (int64_t)std::llround(p.x * inv);
  int64_t gy = (int64_t)std::llround(p.y * inv);
  int64_t key = gx * 73856093LL ^ gy * 19349663LL;
  auto it = lookup.find(key);
  if (it != lookup.end()) return it->second;
  int id = (int)verts.size();
  verts.push_back(p);
  lookup[key] = id;
  return id;
}

// Add an undirected edge between two vertices (used both for real segments and
// for the duplicate/bridge edges we insert to make the graph Eulerian).
void addEdge(Graph &g, int u, int v) {
  int e = g.edgeCount++;
  g.adj[u].push_back({v, e});
  g.adj[v].push_back({u, e});
}

// Union-find for component bridging.
struct DSU {
  std::vector<int> par;
  void init(int n) { par.resize(n); for (int i = 0; i < n; ++i) par[i] = i; }
  int find(int x) { while (par[x] != x) { par[x] = par[par[x]]; x = par[x]; } return x; }
  void uni(int a, int b) { par[find(a)] = find(b); }
};

}  // namespace euler

// Build the shortest single continuous pen-down path covering every segment.
// Returns a 1-element vector (or empty if nothing to draw). `weldEps` is the
// distance under which endpoints are treated as the same point.
//
// `hasStart` / `startPref` (in SOURCE coordinates): bias where the single line
// begins. The pen physically starts here on the machine, so the firmware can
// home to a known point. We cannot start at an arbitrary vertex — an Eulerian
// PATH must begin at one of its two odd-degree ends — so we honor the request as
// closely as the topology allows:
//   * all-even graph (Eulerian circuit): can start ANYWHERE, so we begin at the
//     vertex nearest startPref;
//   * graph with two odd ends (Eulerian path): we begin at whichever of those
//     two ends is nearer startPref.
std::vector<Polyline> buildEulerianSingleLine(const std::vector<Polyline> &polys,
                                              double weldEps,
                                              bool hasStart = false,
                                              Point startPref = {0, 0}) {
  using namespace euler;
  Graph g;
  std::unordered_map<int64_t, int> lookup;
  double inv = (weldEps > 0) ? 1.0 / weldEps : 1e6;

  // 1. Weld vertices and add every polyline segment as a graph edge.
  for (const auto &pl : polys) {
    if (pl.size() < 2) continue;
    int prev = weld(lookup, g.verts, pl[0], inv);
    for (size_t i = 1; i < pl.size(); ++i) {
      int cur = weld(lookup, g.verts, pl[i], inv);
      if (cur != prev) {  // skip zero-length segments from welding
        if ((int)g.adj.size() < (int)g.verts.size()) g.adj.resize(g.verts.size());
        addEdge(g, prev, cur);
      }
      prev = cur;
    }
  }
  const int V = (int)g.verts.size();
  g.adj.resize(V);
  if (g.edgeCount == 0) return {};

  // 2. Bridge disconnected components. Find components over the real edges, then
  // repeatedly connect the two nearest-but-different components by their closest
  // vertices, adding a (drawable) bridge edge. Greedy = MST-like; cheap and good.
  DSU dsu; dsu.init(V);
  for (int u = 0; u < V; ++u)
    for (auto [v, e] : g.adj[u]) dsu.uni(u, v);

  // Join components with a Borůvka-style MST using a spatial grid for the
  // nearest-neighbor queries, so this is ~O(V log V) instead of O(C^2 * V) —
  // critical when a drawing has thousands of separate strokes (each its own
  // component). Each round, every component finds its nearest vertex in ANY
  // other component via the grid; we add those bridges and union, halving the
  // component count per round.
  int nComp = 0;
  {
    std::unordered_map<int, int> seen;
    for (int u = 0; u < V; ++u)
      if (!g.adj[u].empty()) { int r = dsu.find(u); if (!seen[r]++) ++nComp; }
  }
  if (nComp > 1) {
    // Spatial grid over all edged vertices.
    double mnx = 1e300, mny = 1e300, mxx = -1e300, mxy = -1e300;
    for (int u = 0; u < V; ++u)
      if (!g.adj[u].empty()) {
        mnx = std::min(mnx, g.verts[u].x); mny = std::min(mny, g.verts[u].y);
        mxx = std::max(mxx, g.verts[u].x); mxy = std::max(mxy, g.verts[u].y);
      }
    double gw = std::max(1e-6, mxx - mnx), gh = std::max(1e-6, mxy - mny);
    double cell = std::max(1e-6, std::sqrt((gw * gh) / std::max(1, V)));
    int cols = std::max(1, (int)(gw / cell) + 1);
    int rows = std::max(1, (int)(gh / cell) + 1);
    auto cellX = [&](double x) { int c=(int)((x-mnx)/cell); return c<0?0:(c>=cols?cols-1:c); };
    auto cellY = [&](double y) { int r=(int)((y-mny)/cell); return r<0?0:(r>=rows?rows-1:r); };
    std::unordered_map<int64_t, std::vector<int>> grid;
    grid.reserve(V * 2);
    for (int u = 0; u < V; ++u)
      if (!g.adj[u].empty())
        grid[(int64_t)cellY(g.verts[u].y) * cols + cellX(g.verts[u].x)].push_back(u);

    auto nearestOtherComp = [&](int u) -> int {
      int gx = cellX(g.verts[u].x), gy = cellY(g.verts[u].y);
      int ru = dsu.find(u);
      int best = -1; double bestD = 1e300;
      int maxRing = std::max(cols, rows);
      for (int ring = 0; ring <= maxRing; ++ring) {
        if (best >= 0) {
          double rd = (double)(ring - 1) * cell;
          if (rd > 0 && rd * rd > bestD) break;
        }
        int x0=gx-ring,x1=gx+ring,y0=gy-ring,y1=gy+ring;
        for (int yy=y0; yy<=y1; ++yy) {
          if (yy<0||yy>=rows) continue;
          bool edge=(yy==y0||yy==y1);
          for (int xx=x0; xx<=x1; ++xx) {
            if (xx<0||xx>=cols) continue;
            if (ring>0 && !edge && xx!=x0 && xx!=x1) continue;
            auto it=grid.find((int64_t)yy*cols+xx);
            if (it==grid.end()) continue;
            for (int w : it->second) {
              if (dsu.find(w)==ru) continue;
              double d=sqDist(g.verts[u],g.verts[w]);
              if (d<bestD){bestD=d;best=w;}
            }
          }
        }
      }
      return best;
    };

    Clock::time_point tb = Clock::now();
    const double bridgeBudgetMs = 1500.0;
    bool budgetTripped = false;
    while (nComp > 1 && !budgetTripped) {
      // Borůvka round: cheapest outgoing bridge per component. We only need ONE
      // outgoing candidate per component, so iterate over component reps, not all
      // V vertices — and check the time budget inside the loop so a single huge
      // round can still be interrupted on pathological inputs.
      std::unordered_map<int, std::pair<double,std::pair<int,int>>> bestEdge;
      for (int u = 0; u < V; ++u) {
        if (g.adj[u].empty()) continue;
        int ru = dsu.find(u);
        if (bestEdge.count(ru)) continue;  // already have a candidate for this comp
        if ((u & 255) == 0 && elapsedMs(tb) > bridgeBudgetMs) { budgetTripped = true; break; }
        int w = nearestOtherComp(u);
        if (w < 0) continue;
        double d = sqDist(g.verts[u], g.verts[w]);
        bestEdge[ru] = {d, {u, w}};
      }
      if (bestEdge.empty()) break;
      int before = nComp;
      for (auto &kv : bestEdge) {
        int u = kv.second.second.first, w = kv.second.second.second;
        if (dsu.find(u) != dsu.find(w)) { addEdge(g, u, w); dsu.uni(u, w); --nComp; }
      }
      if (nComp == before) break;  // safety: no progress
    }
    // Fallback if the budget tripped on a pathological point-cloud input: chain
    // the remaining components by one representative each, sorted spatially, so
    // the result is still a valid single line (just with longer bridges).
    if (nComp > 1) {
      std::unordered_map<int,int> repOf;
      for (int u = 0; u < V; ++u)
        if (!g.adj[u].empty()) repOf.emplace(dsu.find(u), u);
      std::vector<int> reps;
      reps.reserve(repOf.size());
      for (auto &kv : repOf) reps.push_back(kv.second);
      std::sort(reps.begin(), reps.end(), [&](int a, int b) {
        if (g.verts[a].x != g.verts[b].x) return g.verts[a].x < g.verts[b].x;
        return g.verts[a].y < g.verts[b].y;
      });
      for (size_t i = 1; i < reps.size(); ++i)
        if (dsu.find(reps[i-1]) != dsu.find(reps[i])) {
          addEdge(g, reps[i-1], reps[i]); dsu.uni(reps[i-1], reps[i]);
        }
    }
  }

  // 3. Make it Eulerian: pair up odd-degree vertices and duplicate the segment
  // between each pair (greedy nearest matching). Each duplicated edge means that
  // bit of line gets drawn twice — the unavoidable retracing cost.
  //
  // An Eulerian PATH (not circuit) may keep exactly TWO odd vertices as its free
  // start/end. So we greedily match all odd vertices into nearest pairs, then
  // DROP the single most expensive pair — leaving those two odd. That turns the
  // costliest retrace into free path endpoints (e.g. a lone open stroke is then
  // drawn once, not twice).
  std::vector<int> odd;
  for (int u = 0; u < V; ++u)
    if (g.adj[u].size() % 2) odd.push_back(u);

  struct OddPair { int u, v; double d; };
  std::vector<OddPair> pairs;
  if (!odd.empty()) {
    // Grid over odd vertices so nearest-unmatched lookup is ~O(1), not O(odd) —
    // pathological inputs can have tens of thousands of odd vertices.
    double mnx=1e300,mny=1e300,mxx=-1e300,mxy=-1e300;
    for (int u : odd){ mnx=std::min(mnx,g.verts[u].x); mny=std::min(mny,g.verts[u].y);
                       mxx=std::max(mxx,g.verts[u].x); mxy=std::max(mxy,g.verts[u].y); }
    double gw=std::max(1e-6,mxx-mnx), gh=std::max(1e-6,mxy-mny);
    double cell=std::max(1e-6,std::sqrt((gw*gh)/std::max<size_t>(1,odd.size())));
    int cols=std::max(1,(int)(gw/cell)+1), rows=std::max(1,(int)(gh/cell)+1);
    auto cX=[&](double x){int c=(int)((x-mnx)/cell);return c<0?0:(c>=cols?cols-1:c);};
    auto cY=[&](double y){int r=(int)((y-mny)/cell);return r<0?0:(r>=rows?rows-1:r);};
    std::unordered_map<int64_t,std::vector<int>> grid;  // cell -> odd-vertex ids
    grid.reserve(odd.size()*2);
    for (int u : odd) grid[(int64_t)cY(g.verts[u].y)*cols+cX(g.verts[u].x)].push_back(u);
    std::vector<char> matched(V, 0);

    auto nearestUnmatched=[&](int u)->int{
      int gx=cX(g.verts[u].x), gy=cY(g.verts[u].y);
      int best=-1; double bestD=1e300; int maxRing=std::max(cols,rows);
      for(int ring=0;ring<=maxRing;++ring){
        if(best>=0){double rd=(double)(ring-1)*cell; if(rd>0&&rd*rd>bestD)break;}
        int x0=gx-ring,x1=gx+ring,y0=gy-ring,y1=gy+ring;
        for(int yy=y0;yy<=y1;++yy){ if(yy<0||yy>=rows)continue;
          bool e=(yy==y0||yy==y1);
          for(int xx=x0;xx<=x1;++xx){ if(xx<0||xx>=cols)continue;
            if(ring>0&&!e&&xx!=x0&&xx!=x1)continue;
            auto it=grid.find((int64_t)yy*cols+xx); if(it==grid.end())continue;
            for(int w:it->second){ if(w==u||matched[w])continue;
              double d=sqDist(g.verts[u],g.verts[w]); if(d<bestD){bestD=d;best=w;} }
          } }
      }
      return best;
    };

    for (int u : odd) {
      if (matched[u]) continue;
      int w = nearestUnmatched(u);
      if (w < 0) break;
      matched[u] = matched[w] = 1;
      pairs.push_back({u, w, std::sqrt(sqDist(g.verts[u], g.verts[w]))});
    }
  }
  // Find the most expensive pair to leave unmatched (its two ends become the
  // path's start/end). Add a duplicate edge for every OTHER pair.
  int drop = -1;
  double dropD = -1;
  for (size_t i = 0; i < pairs.size(); ++i)
    if (pairs[i].d > dropD) { dropD = pairs[i].d; drop = (int)i; }
  for (size_t i = 0; i < pairs.size(); ++i) {
    if ((int)i == drop) continue;  // keep these two odd as path endpoints
    addEdge(g, pairs[i].u, pairs[i].v);  // duplicate connector (drawn twice)
  }

  // 4. Hierholzer's algorithm — extract one Eulerian trail covering all edges.
  // Choose the start vertex honoring any requested start position (see header):
  // an Eulerian path must start at an odd end; an Eulerian circuit may start
  // anywhere.
  std::vector<int> oddEnds;
  for (int u = 0; u < V; ++u)
    if (g.adj[u].size() % 2) oddEnds.push_back(u);

  auto pickNearest = [&](const std::vector<int> &cands) -> int {
    int best = -1; double bestD = 1e300;
    for (int u : cands) {
      if (g.adj[u].empty()) continue;
      double d = sqDist(g.verts[u], startPref);
      if (d < bestD) { bestD = d; best = u; }
    }
    return best;
  };

  int start = -1;
  if (!oddEnds.empty()) {
    // Eulerian path: must begin at an odd end. If a start was requested, take the
    // odd end nearest it; otherwise just the first.
    start = hasStart ? pickNearest(oddEnds) : oddEnds.front();
  } else {
    // Eulerian circuit: may begin anywhere. Honor the requested start by picking
    // the nearest vertex; otherwise the first edged vertex.
    if (hasStart) {
      std::vector<int> all;
      all.reserve(V);
      for (int u = 0; u < V; ++u) if (!g.adj[u].empty()) all.push_back(u);
      start = pickNearest(all);
    } else {
      for (int u = 0; u < V; ++u)
        if (!g.adj[u].empty()) { start = u; break; }
    }
  }
  if (start < 0) return {};

  std::vector<char> usedEdge(g.edgeCount, 0);
  std::vector<int> iter(V, 0);  // next-adjacency cursor per vertex
  std::vector<int> stack{start};
  std::vector<int> circuit;
  while (!stack.empty()) {
    int u = stack.back();
    bool advanced = false;
    while (iter[u] < (int)g.adj[u].size()) {
      auto [v, e] = g.adj[u][iter[u]++];
      if (usedEdge[e]) continue;
      usedEdge[e] = 1;
      stack.push_back(v);
      advanced = true;
      break;
    }
    if (!advanced) { circuit.push_back(u); stack.pop_back(); }
  }
  std::reverse(circuit.begin(), circuit.end());

  // 5. Emit the vertex circuit as a single polyline.
  Polyline line;
  line.reserve(circuit.size());
  for (int vid : circuit) line.push_back(g.verts[vid]);
  std::vector<Polyline> out;
  if (line.size() >= 2) out.push_back(std::move(line));
  return out;
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
            << "              strokes to minimize pen-up travel (default: reorder)\n"
            << "  --single-line : draw everything as ONE continuous line, pen\n"
            << "              never lifts. Uses a graph/Eulerian pipeline that\n"
            << "              splits & merges strokes at shared points for the\n"
            << "              shortest possible pen-down path (retracing only\n"
            << "              where topology requires it).\n"
            << "  --start X Y : (single-line only) bias where the line begins,\n"
            << "              in SOURCE/SVG coordinates. The path starts at the\n"
            << "              reachable vertex nearest this point — useful so the\n"
            << "              machine homes the pen to a known spot. Default when\n"
            << "              omitted: the top-left-most point of the drawing.\n";
}

}  // namespace

int main(int argc, char **argv) {
  std::string path;
  double targetW = 0, targetH = 0;
  bool fit = true;   // preserve aspect ratio
  bool flipY = false;
  bool optimize = true;  // reorder strokes to minimize pen-up travel
  bool singleLine = false;  // join all strokes into one continuous, pen-down line
  bool hasStart = false;        // explicit single-line start requested?
  double startX = 0, startY = 0; // requested start, in SOURCE coordinates

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
    } else if (arg == "--single-line") {
      singleLine = true;
    } else if (arg == "--start" && a + 2 < argc) {
      startX = std::stod(argv[++a]);
      startY = std::stod(argv[++a]);
      hasStart = true;
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

  // Two distinct pipelines, selected by --single-line:
  double travelBefore = 0, travelAfter = 0;
  if (singleLine) {
    // PIPELINE B (single line): build the graph-based Eulerian path — the
    // genuinely shortest continuous pen-down line, splitting/merging strokes at
    // shared points and only retracing where topology forces it. weldEps scales
    // with the drawing so coincident endpoints actually connect.
    double diag = std::sqrt(vb.w * vb.w + vb.h * vb.h);
    double weldEps = std::max(1e-6, diag * 1e-4);  // ~0.01% of the diagonal
    travelBefore = drawnLength(shapes);            // total drawn before
    // Start position (SOURCE coords): explicit --start if given, else the
    // drawing's top-left corner (nearest vertex to it is chosen inside).
    Point startPref = hasStart ? Point{startX, startY}
                              : Point{vb.minX, vb.minY};
    shapes = buildEulerianSingleLine(shapes, weldEps, /*hasStart=*/true, startPref);
    travelAfter = drawnLength(shapes);             // total drawn after
  } else if (optimize && shapes.size() > 1) {
    // PIPELINE A (default): minimize pen-up travel by reordering / reversing the
    // rigid strokes. Done in SOURCE coordinates — the transform is affine and
    // order-independent, so the ordering stays optimal after scaling.
    travelBefore = travelDistance(shapes);
    orderForMinTravel(shapes, 750.0, /*penDown=*/false);
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
  if (singleLine && travelBefore > 0) {
    double pct = 100.0 * (travelBefore - travelAfter) / travelBefore;
    std::cerr << "painter: single-line (Eulerian) — total drawn path "
              << travelBefore << " -> " << travelAfter << " src-units (" << pct
              << "% shorter) as one continuous pen-down line.\n";
  } else if (optimize && travelBefore > 0) {
    double pct = 100.0 * (travelBefore - travelAfter) / travelBefore;
    std::cerr << "painter: pen-up travel reordered " << travelBefore
              << " -> " << travelAfter << " src-units (" << pct
              << "% less travel).\n";
  }
  return 0;
}

