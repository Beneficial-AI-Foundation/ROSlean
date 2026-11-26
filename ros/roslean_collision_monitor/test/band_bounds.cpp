// Point by point: where the Lean core's zones differ from upstream's.
//
// The Lean core tests scan points in integer millimetres against zones that
// are a little larger than configured (docs/collision_monitor/SPEC.md, D3 and
// D4, "Safe direction"). This program measures the difference and asserts
// the two bounds the SPEC states:
//
//   S  a point that upstream's test puts inside a zone is inside for the core;
//   G  the core puts no point inside that lies farther outside upstream's
//      zone than 3.5 mm (circle) or 4.4 mm (polygon).
//
// For each of many zones it draws points within 6 mm of the boundary, asks
// upstream's test (the expressions of circle.cpp and polygon.cpp, in double,
// compiled here) and the Lean core through its C interface, exactly as the
// node does (one reading per scan, min_points 1, then a command), and prints
// how often they disagree by distance from the boundary.
//
//   band_bounds [points per zone, default 20000] [seed]
//
// Exit code 0 iff neither bound is violated.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <string>
#include <vector>

#include "roslean_collision_monitor/conversions.hpp"
#include "roslean_core/collision_monitor.hpp"
#include "roslean_core/scan_points_monitor.hpp"

namespace
{

// The scan is handed over with the base-frame coordinates of its readings;
// ScanPointsMonitor passes them with every command, as the node does.
using CollisionMonitor = roslean::ScanPointsMonitor;
using roslean_collision_monitor::put_le;
using P = std::array<double, 2>;

// nav2_collision_monitor/src/circle.cpp, Circle::getPointsInside.
bool upstream_circle(double radius, P p)
{
  const double radius_squared = radius * radius;
  return p[0] * p[0] + p[1] * p[1] < radius_squared;
}

// nav2_collision_monitor/src/polygon.cpp, Polygon::isPointInside.
bool upstream_polygon(const std::vector<P> & poly, P point)
{
  const int poly_size = static_cast<int>(poly.size());
  int i, j;
  bool res = false;
  i = poly_size - 1;
  for (j = 0; j < poly_size; j++) {
    if ((point[1] <= poly[i][1]) == (point[1] > poly[j][1])) {
      const double x_inter = poly[i][0] +
        (point[1] - poly[i][1]) * (poly[j][0] - poly[i][0]) /
        (poly[j][1] - poly[i][1]);
      if (x_inter > point[0]) {
        res = !res;
      }
    }
    i = j;
  }
  return res;
}

double segment_distance(P p, P a, P b)
{
  const double dx = b[0] - a[0], dy = b[1] - a[1];
  const double len2 = dx * dx + dy * dy;
  double t = len2 > 0 ? ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / len2 : 0.0;
  t = std::max(0.0, std::min(1.0, t));
  return std::hypot(p[0] - (a[0] + t * dx), p[1] - (a[1] + t * dy));
}

struct Zone
{
  bool circle;
  double radius;
  std::vector<std::array<float, 2>> vertices;
  std::string name;
};

// Distance bins of 0.25 mm from -6 mm (inside) to +6 mm (outside).
constexpr int kBins = 48;
struct Tally
{
  uint64_t total[kBins] = {}, lean[kBins] = {}, up[kBins] = {};
  uint64_t points = 0, differ = 0, missed = 0, too_far = 0;
  double farthest = 0;  // metres outside upstream's zone, of a point the core counts
  double nearest_rejected = 1;  // metres outside, of a point the core does not count
};

// Is the point inside the zone for the Lean core? One reading with this
// point, then a command: a stop means the zone (min_points 1) triggered.
bool lean_inside(CollisionMonitor & m, P p)
{
  uint8_t rec[20];
  put_le(rec, roslean::bits_of(1.0f), 4);
  put_le(rec + 4, roslean::bits_of(p[0]), 8);
  put_le(rec + 12, roslean::bits_of(p[1]), 8);
  m.on_scan(0, 0, true, roslean::bits_of(0.0f), roslean::bits_of(10.0f), rec, sizeof rec);
  const auto out = m.on_cmd(0, roslean::bits_of(0.5), 0, 0, 0, 0, 0);
  return out.kind == CollisionMonitor::OutputKind::StopZone;
}

void run_zone(const Zone & z, size_t n, std::mt19937_64 & rng, Tally & t, double bound)
{
  CollisionMonitor::Builder b;
  b.set_source(true, 0.0, std::nullopt);       // no age check
  b.set_stop_pub_timeout(1.0);                 // the clock stands at 0: zero is always published
  std::vector<P> poly;
  if (z.circle) {
    b.add_circle(z.radius, CollisionMonitor::ZoneAction::Stop, 0.5, 1, true);
  } else {
    b.add_polygon(CollisionMonitor::ZoneAction::Stop, 0.5, 1, true);
    for (const auto & v : z.vertices) {
      b.add_vertex(v[0], v[1]);
      poly.push_back({static_cast<double>(v[0]), static_cast<double>(v[1])});
    }
  }
  CollisionMonitor m = b.build();
  if (!m.ok()) {
    std::printf("FAIL  the core refuses zone %s\n", z.name.c_str());
    ++t.too_far;
    return;
  }
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  const double pi = 3.14159265358979323846;
  for (size_t k = 0; k < n; ++k) {
    // A point of the boundary, moved up to 6 mm in a random direction. One
    // point in eight is then snapped to a whole millimetre on both axes.
    P q;
    if (z.circle) {
      const double a = 2 * pi * unit(rng);
      q = {std::fabs(z.radius) * std::cos(a), std::fabs(z.radius) * std::sin(a)};
    } else {
      const size_t i = rng() % poly.size();
      const P a = poly[(i + poly.size() - 1) % poly.size()], c = poly[i];
      const double s = rng() % 8 == 0 ? 0.0 : unit(rng);
      q = {a[0] + s * (c[0] - a[0]), a[1] + s * (c[1] - a[1])};
    }
    const double off = rng() % 16 == 0 ? 0.0 : 0.006 * unit(rng);
    const double phi = 2 * pi * unit(rng);
    P p = {q[0] + off * std::cos(phi), q[1] + off * std::sin(phi)};
    if (rng() % 8 == 0) {
      p = {std::round(p[0] * 1000) / 1000, std::round(p[1] * 1000) / 1000};
    }
    bool up;
    double d;  // metres outside upstream's zone; negative inside
    if (z.circle) {
      up = upstream_circle(z.radius, p);
      d = std::hypot(p[0], p[1]) - std::fabs(z.radius);
    } else {
      up = upstream_polygon(poly, p);
      d = 1e9;
      for (size_t i = 0; i < poly.size(); ++i) {
        d = std::min(d, segment_distance(p, poly[(i + poly.size() - 1) % poly.size()], poly[i]));
      }
    }
    // Upstream's own verdict defines the side; the distance is its amount.
    d = up ? -std::fabs(d) : std::fabs(d);
    const bool lean = lean_inside(m, p);
    const int bin = std::max(0, std::min(kBins - 1, static_cast<int>(std::floor(d / 0.00025)) + 24));
    ++t.total[bin];
    t.lean[bin] += lean;
    t.up[bin] += up;
    ++t.points;
    t.differ += lean != up;
    if (up && !lean) {
      if (t.missed++ < 5) {
        std::printf("FAIL  S: zone %s, point (%.17g, %.17g): inside for upstream, outside for "
          "the core\n", z.name.c_str(), p[0], p[1]);
      }
    }
    if (lean && !up) {
      t.farthest = std::max(t.farthest, d);
      if (d > bound) {
        if (t.too_far++ < 5) {
          std::printf("FAIL  G: zone %s, point (%.17g, %.17g): %.4f mm outside, inside for the "
            "core\n", z.name.c_str(), p[0], p[1], d * 1000);
        }
      }
    }
    if (!lean) {
      t.nearest_rejected = std::min(t.nearest_rejected, d);
    }
  }
}

void report(const char * kind, const Tally & t, double bound)
{
  std::printf(
    "\n%s zones: %llu points within 6 mm of a boundary; the two tests disagree on %llu "
    "(%.2f %%)\n", kind, static_cast<unsigned long long>(t.points),
    static_cast<unsigned long long>(t.differ),
    100.0 * static_cast<double>(t.differ) / static_cast<double>(t.points));
  std::printf("  distance outside upstream's zone   points   inside for the core\n");
  for (int b = 20; b < 44; ++b) {
    if (t.total[b] == 0) {continue;}
    std::printf(
      "  %+5.2f to %+5.2f mm %18llu   %6.2f %%%s\n", (b - 24) * 0.25, (b - 23) * 0.25,
      static_cast<unsigned long long>(t.total[b]),
      100.0 * static_cast<double>(t.lean[b]) / static_cast<double>(t.total[b]),
      b < 24 ? "   (inside for upstream)" : "");
  }
  std::printf(
    "  farthest point outside upstream's zone that the core counts: %.3f mm (bound %.1f mm)\n"
    "  nearest point outside upstream's zone that the core does not count: %.3f mm\n",
    t.farthest * 1000, bound * 1000, t.nearest_rejected * 1000);
  std::printf(
    "%s  S (%s): %llu points inside for upstream and outside for the core\n",
    t.missed ? "FAIL" : "ok  ", kind, static_cast<unsigned long long>(t.missed));
  std::printf(
    "%s  G (%s): %llu points counted by the core beyond %.1f mm\n",
    t.too_far ? "FAIL" : "ok  ", kind, static_cast<unsigned long long>(t.too_far), bound * 1000);
}

}  // namespace

int main(int argc, char ** argv)
{
  const size_t per_zone = argc > 1 ? std::strtoull(argv[1], nullptr, 10) : 20000;
  std::mt19937_64 rng(argc > 2 ? std::strtoull(argv[2], nullptr, 10) : 1);
  std::uniform_real_distribution<double> unit(0.0, 1.0);

  std::vector<Zone> circles, polygons;
  for (double r : {0.3, 0.5, 0.7, 0.75, 1.0, 1.5, 2.0, -0.5, 0.001, 0.2995, 0.3005, 999.0}) {
    circles.push_back({true, r, {}, "circle " + std::to_string(r)});
  }
  for (int k = 0; k < 150; ++k) {  // random radii, many of them close to whole millimetres
    double r = 0.05 + 4.0 * unit(rng);
    if (k % 3 == 0) {r = std::round(r * 1000) / 1000 + (unit(rng) - 0.5) * 1e-9;}
    circles.push_back({true, r, {}, "circle " + std::to_string(r)});
  }
  auto box = [&](float x0, float x1, float y0, float y1, const char * name) {
      polygons.push_back({false, 0, {{{x1, y1}, {x1, y0}, {x0, y0}, {x0, y1}}}, name});
    };
  box(-0.3f, 0.3f, -0.3f, 0.3f, "square 0.3");
  box(-0.5f, 0.5f, -0.5f, 0.5f, "square 0.5");
  box(-0.7f, 0.7f, -0.7f, 0.7f, "square 0.7");
  box(-1.0f, 1.0f, -1.0f, 1.0f, "square 1.0");
  box(0.0f, 0.3f, -0.3f, 0.3f, "nav2 example stop");
  box(0.125f, 0.75f, -0.25f, 0.25f, "front");
  box(-1.0f, 1.0f, 0.0f, 1.0f, "upper half");
  polygons.push_back(
    {false, 0, {{{1.0f, 0.0f}, {0.25f, -1.0f}, {-1.0f, -1.0f}, {-1.0f, 1.0f}, {0.25f, 1.0f}}},
      "pentagon"});
  for (int k = 0; k < 150; ++k) {
    // Star-shaped about a centre up to 50 m away: 4 to 8 vertices at sorted
    // angles, 0.05 to 3 m from the centre, so also concave and thin shapes.
    const double cx = k % 4 == 0 ? 100 * (unit(rng) - 0.5) : unit(rng) - 0.5;
    const double cy = k % 4 == 0 ? 100 * (unit(rng) - 0.5) : unit(rng) - 0.5;
    const int nv = 4 + static_cast<int>(rng() % 5);
    std::vector<double> angles;
    for (int i = 0; i < nv; ++i) {angles.push_back(6.283185307179586 * unit(rng));}
    // One in five keeps the angles unsorted: a polygon that crosses itself.
    if (k % 5 != 4) {std::sort(angles.begin(), angles.end());}
    Zone z{false, 0, {}, "random polygon " + std::to_string(k)};
    for (double a : angles) {
      const double rad = 0.05 + 2.95 * unit(rng);
      float x = static_cast<float>(cx + rad * std::cos(a));
      float y = static_cast<float>(cy + rad * std::sin(a));
      if (k % 3 == 0) {  // vertices on whole millimetres
        x = static_cast<float>(std::round(static_cast<double>(x) * 1000) / 1000);
        y = static_cast<float>(std::round(static_cast<double>(y) * 1000) / 1000);
      }
      z.vertices.push_back({x, y});
    }
    if (k % 7 == 0) {z.vertices.push_back(z.vertices.front());}  // closing vertex repeated
    polygons.push_back(z);
  }
  for (int k = 0; k < 20; ++k) {  // slivers 0.5 to 10 mm wide, at any angle
    const double a = 6.283185307179586 * unit(rng), w = 0.0005 + 0.0095 * unit(rng);
    const double ux = std::cos(a), uy = std::sin(a), len = 0.2 + unit(rng);
    Zone z{false, 0, {}, "sliver " + std::to_string(k)};
    for (auto c : {std::pair{len, w}, {len, -w}, {-len, -w}, {-len, w}}) {
      z.vertices.push_back({static_cast<float>(c.first * ux - c.second * uy),
          static_cast<float>(c.first * uy + c.second * ux)});
    }
    polygons.push_back(z);
  }

  Tally tc, tp;
  for (const auto & z : circles) {run_zone(z, per_zone, rng, tc, 0.0035);}
  for (const auto & z : polygons) {run_zone(z, per_zone, rng, tp, 0.0044);}
  std::printf("%zu circles and %zu polygons, %zu points each\n", circles.size(), polygons.size(),
    per_zone);
  report("circle", tc, 0.0035);
  report("polygon", tp, 0.0044);
  return (tc.missed || tc.too_far || tp.missed || tp.too_far) ? EXIT_FAILURE : EXIT_SUCCESS;
}
