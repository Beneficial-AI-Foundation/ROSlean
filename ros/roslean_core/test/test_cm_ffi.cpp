// Exercises the C++ wrapper of the collision monitor against the Lean core,
// without ROS. A mismatch between collision_monitor.hpp, ffi.h and the Lean
// exports (argument order, widths, signedness, byte layout of a scan,
// ownership) shows up as a wrong decision or a crash here. Expectations follow
// docs/collision_monitor/SPEC.md and ROSlean/CollisionMonitor/Examples.lean.
//
// The second half compares the conversions that Lean performs in integer
// arithmetic (millimetres, nanoseconds) with reference computations in
// hardware floating point.

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <optional>
#include <random>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "roslean_core/collision_monitor.hpp"

namespace
{

using CM = roslean::CollisionMonitor;
using Kind = CM::OutputKind;
using Err = CM::ZoneError;
using CErr = CM::ConfigError;
using roslean::bits_of;

int failures = 0;

void check(bool cond, const std::string & what)
{
  std::printf("%s  %s\n", cond ? "ok  " : "FAIL", what.c_str());
  if (!cond) {
    ++failures;
  }
}

uint64_t d(double v) {return bits_of(v);}
uint32_t f(float v) {return bits_of(v);}

const double kInf = std::numeric_limits<double>::infinity();
const double kNaN = std::numeric_limits<double>::quiet_NaN();
const float kInfF = std::numeric_limits<float>::infinity();
const float kNaNF = std::numeric_limits<float>::quiet_NaN();

const uint64_t kMs = 1000000;
const uint64_t kS = 1000000000;

// One reading: the range and the point it denotes, in metres.
struct Rec
{
  float range;
  double x;
  double y;
};

void put(std::vector<uint8_t> & out, uint64_t v, int bytes)
{
  for (int k = 0; k < bytes; ++k) {
    out.push_back(static_cast<uint8_t>(v >> (8 * k)));
  }
}

std::vector<uint8_t> encode(const std::vector<Rec> & recs)
{
  std::vector<uint8_t> out;
  for (const auto & r : recs) {
    put(out, f(r.range), 4);
    put(out, d(r.x), 8);
    put(out, d(r.y), 8);
  }
  return out;
}

// A scan with range limits 0.1 m to 10 m.
void scan(CM & m, uint64_t now, int64_t stamp, const std::vector<Rec> & recs, bool usable = true)
{
  const auto bytes = encode(recs);
  m.on_scan(now, stamp, usable, f(0.1f), f(10.0f), bytes.data(), bytes.size());
}

// A command: forward at 0.5 m/s, turning at 0.2 rad/s.
CM::Output fwd(CM & m, uint64_t now)
{
  return m.on_cmd(now, d(0.5), d(0.0), d(0.0), d(0.0), d(0.0), d(0.2));
}

CM::Output halt(CM & m, uint64_t now)
{
  return m.on_cmd(now, d(0.0), d(-0.0), d(0.0), d(0.0), d(0.0), d(0.0));
}

bool is(const CM::Output & o, Kind kind, uint32_t zone = 0, uint64_t ratio = 0)
{
  return o.kind == kind && o.zone == zone && o.ratio_bits == ratio;
}

// A coordinate in metres that lies in the middle of millimetre `mm`: it
// truncates to `mm` whatever the rounding of the decimal fraction.
double mid(int mm)
{
  return mm == 0 ? 0.0 : (mm + (mm > 0 ? 0.5 : -0.5)) / 1000.0;
}

// n readings at range 0.2 m, at the point that truncates to (x, y) mm.
std::vector<Rec> at(int x, int y, size_t n = 4)
{
  return std::vector<Rec>(n, Rec{0.2f, mid(x), mid(y)});
}

// A radius in metres that converts to exactly `mm` millimetres (rounded up,
// plus 1).
double radius(int mm) {return (mm - 1.5) / 1000.0;}

// Zone 0: stop circle of 300 mm, 2 points. Zone 1: slowdown circle of 600 mm
// at 0.5, 2 points. Source timeout 2 s, stop_pub_timeout 1 s.
CM standard()
{
  CM::Builder b;
  b.set_source(true, 2.0, std::nullopt);
  b.set_stop_pub_timeout(1.0);
  b.add_circle(radius(300), CM::ZoneAction::Stop, 0.0, 2, true);
  b.add_circle(radius(600), CM::ZoneAction::Slowdown, 0.5, 2, true);
  return b.build();
}

// A monitor with one stop circle, both timeouts 0.
CM circle_stop(double radius_m, int64_t min_points = 1)
{
  CM::Builder b;
  b.add_circle(radius_m, CM::ZoneAction::Stop, 0.0, min_points, true);
  return b.build();
}

// Exact floor of |v| * 1000 for a finite v with |v| < 1000, from the rounded
// product and its exact error. (An integer can lie between the rounded and
// the exact product only if the rounded product is that integer.)
double exact_floor_mm(double v)
{
  const double a = std::fabs(v);
  const double t = a * 1000.0;
  const double e = std::fma(a, 1000.0, -t);
  double fl = std::floor(t);
  if (fl == t && e < 0.0) {
    fl -= 1.0;
  }
  return fl;
}

double exact_ceil_mm(double v)
{
  const double a = std::fabs(v);
  const double t = a * 1000.0;
  const double e = std::fma(a, 1000.0, -t);
  double c = std::ceil(t);
  if (c == t && e > 0.0) {
    c += 1.0;
  }
  return c;
}

// Exact round-to-nearest of |v| * 1000, a half away from zero.
double exact_round_mm(double v)
{
  const double a = std::fabs(v);
  const double t = a * 1000.0;
  const double e = std::fma(a, 1000.0, -t);
  const double fl = std::floor(t);
  if (t - fl == 0.5) {
    return e < 0.0 ? fl : fl + 1.0;
  }
  return t - fl > 0.5 ? fl + 1.0 : fl;
}

int64_t expected_coord(double v)
{
  if (std::isnan(v)) {
    return 1000000;
  }
  if (std::fabs(v) >= 1000.0) {
    return v > 0 ? 1000000 : -1000000;
  }
  const double fl = exact_floor_mm(v);
  return static_cast<int64_t>(std::signbit(v) ? -fl : fl);
}

}  // namespace

int main()
{
  roslean::initialize();
  roslean::initialize();  // idempotent

  {
    // The three decisions, and the source check (R8, R7).
    auto m = standard();
    check(m.ok() && m.error() == CErr::None, "standard configuration is accepted");
    check(m.zone_error(0) == Err::None && m.zone_error(1) == Err::None &&
      m.zone_error(2) == Err::NoSuchZone && m.zone_error(UINT32_MAX) == Err::NoSuchZone,
      "zone_error: none for both zones, no such zone beyond the last");
    check(is(fwd(m, 100 * kMs), Kind::StopSource), "no scan yet: stop for an invalid source");
    scan(m, 200 * kMs, 200 * kMs, at(2000, 0));
    check(is(fwd(m, 300 * kMs), Kind::Pass), "points outside every zone: pass");
    scan(m, 400 * kMs, 400 * kMs, at(500, 0));
    check(is(fwd(m, 500 * kMs), Kind::Slowdown, 1, d(0.5)), "points in the slowdown zone: zone 1, ratio 0.5");
    scan(m, 600 * kMs, 600 * kMs, at(200, 0));
    check(is(fwd(m, 700 * kMs), Kind::StopZone, 0), "points in the stop zone: stop, zone 0");
    scan(m, 800 * kMs, 800 * kMs, at(2000, 0));
    check(is(fwd(m, 2800 * kMs), Kind::Pass), "scan not too old exactly at the timeout");
    check(is(fwd(m, 2800 * kMs + 1), Kind::StopSource), "scan too old 1 ns after the timeout");
    scan(m, 2900 * kMs, 5000 * kMs, at(2000, 0));
    check(is(fwd(m, 3000 * kMs), Kind::Pass), "a scan stamped in the future is valid");
  }
  {
    // Zone boundaries and counting (R9, R11); coordinates truncate toward 0.
    auto m = standard();
    scan(m, 0, 0, at(300, 0));
    check(is(fwd(m, 1), Kind::Slowdown, 1, d(0.5)), "a point on the stop circle is outside it");
    scan(m, 0, 0, at(299, 0));
    check(is(fwd(m, 1), Kind::StopZone, 0), "a point 1 mm inside the stop circle");
    scan(m, 0, 0, at(299, 0, 1));
    check(is(fwd(m, 1), Kind::Pass), "one point is below min_points 2");
    scan(m, 0, 0, {{0.2f, mid(299), 0.0}, {0.2f, 0.0, mid(-299)}});
    check(is(fwd(m, 1), Kind::StopZone, 0), "two points reach min_points 2");
    scan(m, 0, 0, at(180, 240));
    check(is(fwd(m, 1), Kind::Slowdown, 1, d(0.5)), "(180, 240) is on the 300 mm circle: outside");
    scan(m, 0, 0, at(-179, -240));
    check(is(fwd(m, 1), Kind::StopZone, 0), "negative coordinates keep their sign");
    scan(m, 0, 0, at(0, -599));
    check(is(fwd(m, 1), Kind::Slowdown, 1, d(0.5)), "negative y alone");
    scan(m, 0, 0, at(0, 599));
    check(is(fwd(m, 1), Kind::Slowdown, 1, d(0.5)), "positive y alone");
    scan(m, 0, 0, {{0.2f, 0.5999999, 0.0}, {0.2f, -0.5999999, 0.0}});
    check(is(fwd(m, 1), Kind::Slowdown, 1, d(0.5)), "599.9999 mm truncates to 599, either sign: inside");
    scan(m, 0, 0, {{0.2f, 0.6, 0.0}, {0.2f, -0.6, 0.0}});
    check(is(fwd(m, 1), Kind::Slowdown, 1, d(0.5)),
      "0.6 as a double is just below 600 mm: truncated to 599, inside");
    scan(m, 0, 0, {{0.2f, 0.6000001, 0.0}, {0.2f, -0.6000001, 0.0}});
    check(is(fwd(m, 1), Kind::Pass), "600.0001 mm is 600: on the circle, outside");
  }
  {
    // Validity of readings (R4): range_min <= range <= range_max on floats.
    auto m = standard();
    const double x = 0.1;
    scan(m, 0, 0, {{0.05f, x, 0}, {0.05f, x, 0}, {20.0f, x, 0}, {kNaNF, x, 0}, {kInfF, x, 0},
        {-1.0f, x, 0}});
    check(is(fwd(m, 1), Kind::Pass), "readings outside the limits, NaN and inf are skipped");
    scan(m, 0, 0, {{0.1f, x, 0}, {10.0f, x, 0}});
    check(is(fwd(m, 1), Kind::StopZone, 0), "readings equal to the limits are valid");
    auto bytes = encode({{kInfF, x, 0}, {kInfF, x, 0}});
    m.on_scan(0, 0, true, f(0.1f), f(kInfF), bytes.data(), bytes.size());
    check(is(fwd(m, 1), Kind::StopZone, 0), "an infinite reading is valid if range_max is infinite");
    bytes = encode(at(100, 0));
    m.on_scan(0, 0, true, f(kNaNF), f(10.0f), bytes.data(), bytes.size());
    check(is(fwd(m, 1), Kind::Pass), "NaN range_min: no reading is valid");
    m.on_scan(0, 0, true, f(0.1f), f(kNaNF), bytes.data(), bytes.size());
    check(is(fwd(m, 1), Kind::Pass), "NaN range_max: no reading is valid");
    // Limits in the right order: min then max.
    m.on_scan(0, 0, true, f(10.0f), f(0.1f), bytes.data(), bytes.size());
    check(is(fwd(m, 1), Kind::Pass), "range_min above range_max: no reading is valid");
    bytes = encode({{0.0f, x, 0}, {-0.0f, x, 0}});
    m.on_scan(0, 0, true, f(-0.0f), f(0.0f), bytes.data(), bytes.size());
    check(is(fwd(m, 1), Kind::StopZone, 0), "0 and -0 are equal");
  }
  {
    // A scan of zero length, partial records, and replacement (R5).
    auto m = standard();
    m.on_scan(0, 0, true, f(0.1f), f(10.0f), nullptr, 0);
    check(is(fwd(m, 1), Kind::Pass), "an empty scan is free space");
    scan(m, 0, 0, at(100, 0));
    check(is(fwd(m, 1), Kind::StopZone, 0), "an obstacle");
    const uint8_t none = 0;
    m.on_scan(0, 0, true, f(0.1f), f(10.0f), &none, 0);
    check(is(fwd(m, 1), Kind::Pass), "an empty scan replaces the stored one");
    auto bytes = encode(at(100, 0, 2));
    m.on_scan(0, 0, true, f(0.1f), f(10.0f), bytes.data(), 39);
    check(is(fwd(m, 1), Kind::Pass), "39 bytes are one record: the partial record is ignored");
    m.on_scan(0, 0, true, f(0.1f), f(10.0f), bytes.data(), 40);
    check(is(fwd(m, 1), Kind::StopZone, 0), "40 bytes are two records");
    bytes.push_back(0xff);
    m.on_scan(0, 0, true, f(0.1f), f(10.0f), bytes.data(), 41);
    check(is(fwd(m, 1), Kind::StopZone, 0), "41 bytes are two records");
    m.on_scan(0, 0, true, f(0.1f), f(10.0f), bytes.data(), 19);
    check(is(fwd(m, 1), Kind::Pass), "19 bytes are no record");
  }
  {
    // A scan that is not usable (no transform for its frame, D11) discards
    // the stored scan: the source has no data.
    auto m = standard();
    scan(m, 0, 0, at(2000, 0));
    check(is(fwd(m, 1), Kind::Pass), "usable scan: pass");
    scan(m, 2, 2, at(2000, 0), false);
    check(is(fwd(m, 3), Kind::StopSource), "unusable scan: stop for an invalid source");
    check(is(fwd(m, 4), Kind::StopSource), "and it stays so");
    scan(m, 5, 5, at(2000, 0));
    check(is(fwd(m, 6), Kind::Pass), "the next usable scan restores the source");
    // With source_timeout 0 a missing scan is "no points" (R8).
    auto m0 = circle_stop(radius(300));
    scan(m0, 0, 0, at(100, 0));
    check(is(fwd(m0, 1), Kind::StopZone, 0), "timeout 0: obstacle stops");
    scan(m0, 2, 2, at(100, 0), false);
    check(is(fwd(m0, 3), Kind::Pass), "timeout 0: an unusable scan contributes no points, as upstream");
  }
  {
    // Zero is published for stop_pub_timeout, then nothing (R15); a
    // non-finite command is ignored (R2).
    auto m = standard();
    scan(m, 0, 0, at(100, 0));
    check(is(fwd(m, 1000 * kMs), Kind::StopZone, 0), "stop published");
    check(is(fwd(m, 2000 * kMs), Kind::StopZone, 0), "still published exactly at stop_pub_timeout");
    check(is(fwd(m, 2000 * kMs + 1), Kind::Silent), "silent 1 ns after stop_pub_timeout");
    check(
      is(m.on_cmd(2001 * kMs, d(0.5), d(0.0), d(kNaN), d(0.0), d(0.0), d(0.2)), Kind::Silent),
      "NaN in linear.z: ignored");
    scan(m, 2001 * kMs, 2001 * kMs, at(2000, 0));
    check(is(halt(m, 2002 * kMs), Kind::Silent), "a zero command continues the silence");
    check(is(fwd(m, 2003 * kMs), Kind::Pass), "a non-zero command is published");
    check(is(halt(m, 2004 * kMs), Kind::Pass), "zero command: published as pass");
    check(is(halt(m, 3004 * kMs), Kind::Pass), "zero command still published at the timeout");
    check(is(halt(m, 3004 * kMs + 1), Kind::Silent), "and dropped after it");
    // Each of the six components is checked, in its own position.
    const double bad[] = {kInf, -kInf, kNaN};
    bool all = true;
    for (int i = 0; i < 6; ++i) {
      uint64_t v[6] = {d(0.5), d(0.0), d(0.0), d(0.0), d(0.0), d(0.2)};
      v[i] = d(bad[i % 3]);
      all = is(m.on_cmd(4000 * kMs, v[0], v[1], v[2], v[3], v[4], v[5]), Kind::Silent) && all;
    }
    check(all, "a non-finite value in any of the six components: silent");
    // Only linear.x, linear.y and angular.z count for "zero".
    scan(m, 4000 * kMs, static_cast<int64_t>(100 * kS), at(2000, 0));
    check(is(fwd(m, 4500 * kMs), Kind::Pass), "a non-zero command ends the zero phase");
    check(is(m.on_cmd(5000 * kMs, d(0.0), d(0.0), d(1.0), d(1.0), d(1.0), d(0.0)), Kind::Pass),
      "planar-zero command starts a zero phase");
    check(is(m.on_cmd(6000 * kMs + 1, d(0.0), d(0.0), d(1.0), d(1.0), d(1.0), d(0.0)), Kind::Silent),
      "linear.z, angular.x, angular.y do not make it non-zero");
    check(is(m.on_cmd(6000 * kMs + 2, d(0.0), d(1e-300), d(0.0), d(0.0), d(0.0), d(0.0)), Kind::Pass),
      "linear.y counts");
    check(is(halt(m, 6000 * kMs + 3), Kind::Pass), "zero again: published");
    check(is(m.on_cmd(8000 * kMs, d(0.0), d(0.0), d(0.0), d(0.0), d(0.0), d(-5e-324)), Kind::Pass),
      "angular.z counts");
  }
  {
    // Argument order and meaning of add_circle and set_source.
    CM::Builder b;
    b.set_source(true, 0.0, std::nullopt);  // timeout 0: no data is "no points"
    // Disabled stop zone, then an always-triggering slowdown (min_points -3).
    b.add_circle(0.5, CM::ZoneAction::Stop, 0.25, 1, false);
    b.add_circle(0.0, CM::ZoneAction::Slowdown, 0.25, -3, true);
    auto m = b.build();
    check(m.ok(), "accepted");
    check(is(fwd(m, 5), Kind::Slowdown, 1, d(0.25)), "negative min_points: triggers without points");
    scan(m, 0, 0, at(100, 0));
    check(is(fwd(m, 5), Kind::Slowdown, 1, d(0.25)), "a disabled stop zone does not stop");
    check(is(fwd(m, UINT64_MAX), Kind::Slowdown, 1, d(0.25)), "timeout 0: never too old");
  }
  {
    // The source's own timeout replaces the node's (argument order).
    CM::Builder b;
    b.set_source(true, 5.0, 1.0);
    auto m = b.build();
    scan(m, 0, 0, {});
    check(is(fwd(m, kS), Kind::Pass) && is(fwd(m, kS + 1), Kind::StopSource),
      "own timeout 1 s is used, not the node's 5 s");
    b.set_source(true, 5.0, std::nullopt);
    auto m2 = b.build();
    scan(m2, 0, 0, {});
    check(is(fwd(m2, 5 * kS), Kind::Pass) && is(fwd(m2, 5 * kS + 1), Kind::StopSource),
      "without an own timeout the node's 5 s is used");
  }
  {
    // A disabled source contributes nothing and is never invalid (R6).
    CM::Builder b;
    b.set_source(false, 1e-9, std::nullopt);
    b.add_circle(0.5, CM::ZoneAction::Stop, 0.0, 1, true);
    auto m = b.build();
    check(is(fwd(m, 5 * kS), Kind::Pass), "disabled source, no scan: pass");
    scan(m, 0, 0, at(100, 0));
    check(is(fwd(m, 5 * kS), Kind::Pass), "disabled source: its points are not used");
  }
  {
    // Action codes at the C level: 1 is slowdown, everything else is stop.
    lean_object * b = roslean_cm_builder_new(roslean_core_unit());
    b = roslean_cm_builder_add_circle(b, d(0.5), 7, d(0.5), 0, 9 /* any non-zero flag */);
    lean_object * m = roslean_cm_new(b);
    m = roslean_cm_on_cmd(m, 5, d(0.5), 0, 0, 0, 0, 0);
    roslean_core_inc(m);
    check(roslean_cm_out_kind(m) == 3, "action code 7 is stop; flag 9 is enabled");
    roslean_core_dec(m);
  }
  {
    // Slowdown selection (R13): the smallest ratio below 1; the earlier of
    // two equal ones; a stop zone after them wins (R12).
    CM::Builder b;
    b.add_circle(1.0, CM::ZoneAction::Slowdown, 0.7, 1, true);
    b.add_circle(1.0, CM::ZoneAction::Slowdown, 0.3, 1, true);
    b.add_circle(1.0, CM::ZoneAction::Slowdown, 0.3, 1, true);
    b.add_circle(1.0, CM::ZoneAction::Slowdown, 1.0, 1, true);
    b.add_circle(1.0, CM::ZoneAction::Slowdown, kNaN, 1, true);
    b.add_circle(1.0, CM::ZoneAction::Slowdown, kInf, 1, true);
    check(b.add_circle(0.2, CM::ZoneAction::Stop, 0.0, 1, true) == 6, "seventh zone is zone 6");
    auto m = b.build();
    check(m.ok(), "ratios of 1, NaN and infinity are accepted");
    scan(m, 0, 0, at(500, 0));
    check(is(fwd(m, 1), Kind::Slowdown, 1, d(0.3)), "smallest ratio, earlier of two equal zones");
    scan(m, 0, 0, at(100, 0));
    check(is(fwd(m, 1), Kind::StopZone, 6), "a later stop zone wins over slowdown zones");
  }
  {
    CM::Builder b;
    b.add_circle(1.0, CM::ZoneAction::Slowdown, 1.0, 0, true);
    b.add_circle(1.0, CM::ZoneAction::Slowdown, 2.0, 0, true);
    auto m = b.build();
    check(is(fwd(m, 1), Kind::Pass), "ratios of 1 and above never take effect");
  }
  {
    // A ratio of 0 yields zero: published for stop_pub_timeout only (D6).
    CM::Builder b;
    b.set_stop_pub_timeout(1e-8);
    b.add_circle(1.0, CM::ZoneAction::Slowdown, 0.0, 0, true);
    auto m = b.build();
    check(m.ok() && CM::timeout_ns(1e-8) == 10, "ratio 0 is accepted; 1e-8 s is 10 ns");
    check(is(fwd(m, 100), Kind::Slowdown, 0, 0), "ratio 0: slowdown with pattern 0");
    check(is(fwd(m, 110), Kind::Slowdown, 0, 0), "published at the timeout (10 ns)");
    check(is(fwd(m, 111), Kind::Silent), "silent after it");
  }
  {
    // Polygon: vertices in order, signed coordinates, the 4-vertex rule.
    // The box of upstream's example: x in [0, 300], y in [-300, 300] mm.
    CM::Builder b;
    b.add_polygon(CM::ZoneAction::Stop, 0.0, 1, true);
    b.add_vertex(0.3f, 0.3f);
    b.add_vertex(0.3f, -0.3f);
    b.add_vertex(0.0f, -0.3f);
    b.add_vertex(0.0f, 0.3f);
    auto m = b.build();
    check(m.ok(), "box accepted");
    scan(m, 0, 0, at(150, -200, 1));
    check(is(fwd(m, 1), Kind::StopZone, 0), "inside the box, negative y");
    scan(m, 0, 0, at(-150, 200, 1));
    check(is(fwd(m, 1), Kind::Pass), "x and y are not swapped or negated");
    scan(m, 0, 0, at(302, 0, 1));
    check(is(fwd(m, 1), Kind::StopZone, 0), "2 mm outside an edge: within the margin (D4)");
    scan(m, 0, 0, at(303, 0, 1));
    check(is(fwd(m, 1), Kind::Pass), "3 mm outside an edge");
    scan(m, 0, 0, at(-3, 0, 1));
    check(is(fwd(m, 1), Kind::Pass), "3 mm outside the edge at x = 0");
    scan(m, 0, 0, {{0.2f, kInf, kNaN}, {0.2f, -kInf, 0.1}, {0.2f, 1e300, -1e300}});
    check(is(fwd(m, 1), Kind::Pass), "saturated points are in no zone");
  }
  {
    // A box entirely at negative coordinates, far out.
    CM::Builder b;
    b.add_polygon(CM::ZoneAction::Slowdown, 0.5, 1, true);
    b.add_vertex(-900.0f, -900.0f);
    b.add_vertex(-900.0f, -800.0f);
    b.add_vertex(-800.0f, -800.0f);
    b.add_vertex(-800.0f, -900.0f);
    auto m = b.build();
    check(m.ok(), "far negative box accepted");
    scan(m, 0, 0, {{0.2f, -850.0, -850.0}});
    check(is(fwd(m, 1), Kind::Slowdown, 0, d(0.5)), "inside the far negative box");
    scan(m, 0, 0, {{0.2f, 850.0, 850.0}});
    check(is(fwd(m, 1), Kind::Pass), "the mirrored point is outside");
  }
  {
    // add_vertex applies to the zone added last, and only to a polygon.
    CM::Builder b;
    b.add_vertex(1.0f, 1.0f);  // no zone yet: no effect
    b.add_polygon(CM::ZoneAction::Stop, 0.0, 1, true);
    b.add_vertex(0.1f, 0.1f);
    b.add_vertex(0.1f, -0.1f);
    b.add_vertex(-0.1f, -0.1f);
    b.add_circle(0.05, CM::ZoneAction::Stop, 0.0, 1, true);
    b.add_vertex(-0.1f, 0.1f);  // last zone is a circle: no effect
    auto m = b.build();
    check(!m.ok() && m.error() == CErr::Zone, "a polygon left with three vertices is refused");
    check(m.zone_error(0) == Err::Points && m.zone_error(1) == Err::None, "zone_error names the triangle");

    b.add_polygon(CM::ZoneAction::Stop, 0.0, 1, true);
    b.add_vertex(0.1f, 0.1f);
    b.add_vertex(0.1f, -0.1f);
    b.add_polygon(CM::ZoneAction::Stop, 0.0, 1, true);
    b.add_vertex(0.1f, 0.1f);
    b.add_vertex(0.1f, -0.1f);
    b.add_vertex(-0.1f, -0.1f);
    b.add_vertex(-0.1f, 0.1f);
    auto m2 = b.build();
    check(m2.zone_error(0) == Err::Points && m2.zone_error(1) == Err::None,
      "vertices go to the polygon added last");
  }
  {
    // Review finding 1: a negative radius is its magnitude, as upstream (R9).
    auto m = circle_stop(-0.3);
    check(m.ok(), "radius -0.3 is accepted");
    scan(m, 0, 0, {{0.2f, 0.2, 0.0}});
    check(is(fwd(m, 1), Kind::StopZone, 0), "radius -0.3: a point at 0.2 m stops, as with +0.3");
    scan(m, 0, 0, {{0.2f, 0.31, 0.0}});
    check(is(fwd(m, 1), Kind::Pass), "radius -0.3: a point at 0.31 m is outside");
  }
  {
    // What is refused, and how it is reported.
    auto circle = [](double r) {return circle_stop(r).zone_error(0);};
    check(circle(0.3) == Err::None && circle(0.0) == Err::None && circle(-0.0) == Err::None &&
      circle(999.9989) == Err::None && circle(-999.9989) == Err::None, "radii below 1000 m are accepted");
    check(circle(999.9999) == Err::Radius && circle(1000.0) == Err::Radius &&
      circle(-1000.0) == Err::Radius && circle(1e300) == Err::Radius,
      "radii whose zone would exceed 1000 m are refused, either sign");
    check(circle(kInf) == Err::Radius && circle(-kInf) == Err::Radius && circle(kNaN) == Err::Radius &&
      circle(std::numeric_limits<double>::max()) == Err::Radius, "non-finite and huge radii are refused");
    check(!circle_stop(kNaN).ok() && circle_stop(kNaN).error() == CErr::Zone, "and the monitor is not ok");

    auto ratio = [](double r) {
        CM::Builder b;
        b.add_circle(0.1, CM::ZoneAction::Slowdown, r, 1, true);
        return b.build().zone_error(0);
      };
    check(ratio(0.5) == Err::None && ratio(7.0) == Err::None && ratio(kInf) == Err::None &&
      ratio(kNaN) == Err::None && ratio(0.0) == Err::None, "sign-clear ratios accepted");
    check(ratio(-0.5) == Err::Ratio && ratio(-0.0) == Err::Ratio && ratio(-kInf) == Err::Ratio,
      "ratios with the sign bit refused (D5)");
    {
      CM::Builder b;
      b.add_circle(0.1, CM::ZoneAction::Stop, -0.5, 1, true);
      check(b.build().ok(), "the ratio of a stop zone is ignored");
    }

    // Review finding 13: min_points outside the range of a 32-bit int.
    auto mp = [](int64_t n) {return circle_stop(0.3, n).zone_error(0);};
    check(mp(0) == Err::None && mp(INT32_MAX) == Err::None && mp(-1) == Err::None &&
      mp(INT32_MIN) == Err::None, "min_points within the int range is accepted");
    check(mp(static_cast<int64_t>(INT32_MAX) + 1) == Err::MinPoints && mp(4294967297LL) == Err::MinPoints &&
      mp(INT64_MAX) == Err::MinPoints && mp(static_cast<int64_t>(INT32_MIN) - 1) == Err::MinPoints &&
      mp(INT64_MIN) == Err::MinPoints, "min_points beyond the int range is refused");

    auto box = [](float lim) {
        CM::Builder b;
        b.add_polygon(CM::ZoneAction::Stop, 0.0, 1, true);
        b.add_vertex(lim, 1.0f);
        b.add_vertex(1.0f, -1.0f);
        b.add_vertex(-1.0f, -1.0f);
        b.add_vertex(-1.0f, lim);
        return b.build().zone_error(0);
      };
    check(box(999.99f) == Err::None && box(-999.99f) == Err::None, "vertices within 999.997 m");
    check(box(999.999f) == Err::Points && box(-999.999f) == Err::Points && box(1e30f) == Err::Points,
      "vertices beyond 999.997 m are refused");
    check(box(kInfF) == Err::Points && box(-kInfF) == Err::Points && box(kNaNF) == Err::Points,
      "non-finite vertices are refused");
    {
      // A refused vertex is not silently dropped: five vertices, one NaN.
      CM::Builder b;
      b.add_polygon(CM::ZoneAction::Stop, 0.0, 1, true);
      b.add_vertex(1.0f, 1.0f);
      b.add_vertex(1.0f, -1.0f);
      b.add_vertex(kNaNF, 0.0f);
      b.add_vertex(-1.0f, -1.0f);
      b.add_vertex(-1.0f, 1.0f);
      check(b.build().zone_error(0) == Err::Points, "a NaN vertex among four good ones refuses the zone");
    }

    auto timeouts = [](double node, std::optional<double> own, double stop_pub) {
        CM::Builder b;
        b.set_source(true, node, own);
        b.set_stop_pub_timeout(stop_pub);
        return b.build().error();
      };
    check(timeouts(2.0, std::nullopt, 1.0) == CErr::None && timeouts(0.0, 0.0, 0.0) == CErr::None &&
      timeouts(-0.0, -0.0, -0.0) == CErr::None && timeouts(9.1e9, 9.1e9, 9.1e9) == CErr::None,
      "timeouts from 0 to below 9.2e9 s are accepted, -0.0 included");
    check(timeouts(-1.0, std::nullopt, 1.0) == CErr::SourceTimeout &&
      timeouts(-5e-324, std::nullopt, 1.0) == CErr::SourceTimeout &&
      timeouts(kNaN, std::nullopt, 1.0) == CErr::SourceTimeout &&
      timeouts(kInf, std::nullopt, 1.0) == CErr::SourceTimeout &&
      timeouts(9.2e9, std::nullopt, 1.0) == CErr::SourceTimeout,
      "negative, non-finite and huge source_timeout refused (D9)");
    check(timeouts(-1.0, 2.0, 1.0) == CErr::SourceTimeout,
      "the node's source_timeout is checked even if the source has its own");
    check(timeouts(2.0, -1.0, 1.0) == CErr::OwnSourceTimeout && timeouts(2.0, kNaN, 1.0) == CErr::OwnSourceTimeout,
      "the source's own timeout");
    check(timeouts(2.0, std::nullopt, -1.0) == CErr::StopPubTimeout &&
      timeouts(2.0, std::nullopt, kInf) == CErr::StopPubTimeout &&
      timeouts(2.0, std::nullopt, 1e10) == CErr::StopPubTimeout, "stop_pub_timeout");
    {
      // Which part is reported when several are wrong.
      CM::Builder b;
      b.add_circle(0.1, CM::ZoneAction::Slowdown, -0.5, 1, true);
      b.add_circle(2000.0, CM::ZoneAction::Slowdown, -0.5, INT64_MAX, true);
      b.add_circle(0.1, CM::ZoneAction::Stop, 0.0, INT64_MAX, true);
      auto m = b.build();
      check(m.zone_error(0) == Err::Ratio && m.zone_error(1) == Err::Radius &&
        m.zone_error(2) == Err::MinPoints && m.error() == CErr::Zone, "one code per zone");
    }
    CM::Builder b;
    check(b.build().ok(), "no zones: accepted");
  }
  {
    // Review finding 7: a monitor that is not ok is not advisory. It answers
    // every command with a stop, whatever the command and the scans.
    const auto never_passes = [](CM m, const std::string & what) {
        bool all = !m.ok();
        all = is(fwd(m, 1), Kind::StopSource) && all;
        scan(m, 2, 2, at(5000, 0));
        all = is(fwd(m, 3), Kind::StopSource) && all;
        all = is(fwd(m, 100 * kS), Kind::StopSource) && all;  // no silence after stop_pub_timeout
        all = is(halt(m, 200 * kS), Kind::StopSource) && all;
        all = is(m.on_cmd(300 * kS, d(kNaN), 0, 0, 0, 0, 0), Kind::StopSource) && all;
        check(all, "not ok, every command is stopped: " + what);
      };
    {
      // Were a negative source_timeout taken as 0, this would pass.
      CM::Builder b;
      b.set_source(true, -1.0, std::nullopt);
      never_passes(b.build(), "negative source_timeout");
    }
    {
      CM::Builder b;
      b.set_stop_pub_timeout(kNaN);
      never_passes(b.build(), "NaN stop_pub_timeout");
    }
    never_passes(circle_stop(kNaN), "NaN radius");
    never_passes(circle_stop(0.3, 4294967297LL), "min_points 4294967297");
    {
      CM::Builder b;
      b.add_circle(0.3, CM::ZoneAction::Slowdown, -0.5, 1, true);
      never_passes(b.build(), "negative slowdown ratio");
    }
    {
      CM::Builder b;
      b.add_polygon(CM::ZoneAction::Stop, 0.0, 1, true);
      never_passes(b.build(), "polygon without vertices");
    }
  }
  {
    // Extreme times: sums at and above 2^63 take Lean's big-number path.
    CM::Builder b;
    b.set_source(true, 9.1e9, std::nullopt);
    b.set_stop_pub_timeout(9.1e9);
    b.add_circle(0.3, CM::ZoneAction::Stop, 0.0, INT32_MAX, true);
    auto m = b.build();
    const uint64_t timeout = 9100000000000000000ULL;
    check(m.ok() && CM::source_timeout_ns(9.1e9, std::nullopt) == static_cast<int64_t>(timeout),
      "extreme timeouts and min_points accepted");
    check(is(fwd(m, UINT64_MAX), Kind::StopSource), "no scan at t = 2^64 - 1");
    scan(m, 5, 5, at(100, 0));
    check(is(fwd(m, UINT64_MAX), Kind::StopSource), "5 + 9.1e18 < 2^64 - 1: too old");
    scan(m, 5, INT64_MAX, at(100, 0));
    const uint64_t limit = static_cast<uint64_t>(INT64_MAX) + timeout;
    check(is(fwd(m, limit), Kind::Pass), "stamp 2^63 - 1: not too old exactly at the timeout");
    check(is(fwd(m, limit + 1), Kind::StopSource), "and too old 1 ns after it");
  }
  {
    // A negative stamp is taken as 0 (D8).
    auto m = standard();
    scan(m, 0, -1, at(2000, 0));
    check(is(fwd(m, 2 * kS), Kind::Pass), "stamp -1 counts as 0: valid at the timeout");
    check(is(fwd(m, 2 * kS + 1), Kind::StopSource), "and too old after it");
    scan(m, 0, INT64_MIN, at(2000, 0));
    check(is(fwd(m, 2 * kS + 1), Kind::StopSource), "stamp INT64_MIN counts as 0");
  }
  {
    // Byte layout of a record: range 0.2f is 0x3E4CCCCD; x = 0.25 is
    // 0x3FD0000000000000; y = -2.0 is 0xC000000000000000.
    auto m = standard();
    const uint8_t rec[20] = {0xCD, 0xCC, 0x4C, 0x3E, 0, 0, 0, 0, 0, 0, 0xD0, 0x3F, 0, 0, 0, 0, 0, 0, 0, 0xC0};
    std::vector<uint8_t> two(rec, rec + 20);
    two.insert(two.end(), rec, rec + 20);
    m.on_scan(0, 0, true, f(0.1f), f(10.0f), two.data(), two.size());
    check(is(fwd(m, 1), Kind::Pass), "little-endian record: (0.25, -2.0) m is outside");
    two[19] = 0x80;  // y of the first record becomes -0.0
    two[39] = 0x00;  // y of the second record becomes +0.0
    m.on_scan(0, 0, true, f(0.1f), f(10.0f), two.data(), two.size());
    check(is(fwd(m, 1), Kind::StopZone, 0), "(0.25, -0.0) and (0.25, 0.0) m are inside 300 mm");
    two[10] = 0xE0;  // x of the first record becomes 0.5
    m.on_scan(0, 0, true, f(0.1f), f(10.0f), two.data(), two.size());
    check(is(fwd(m, 1), Kind::Slowdown, 1, d(0.5)), "byte 6 of x is in the exponent: 0.5 m");
    std::vector<uint8_t> ones(40, 0xff);  // range and coordinates are NaN patterns
    m.on_scan(0, 0, true, f(0.1f), f(10.0f), ones.data(), ones.size());
    check(is(fwd(m, 1), Kind::Pass), "all-ones records: NaN ranges, skipped");
    m.on_scan(0, 0, true, 0xffffffffu, 0xffffffffu, ones.data(), ones.size());
    check(is(fwd(m, 1), Kind::Pass), "all-ones limits");
  }
  {
    // A large scan: 1,000,000 readings.
    auto m = standard();
    std::vector<Rec> recs(1000000, Rec{0.2f, 2.0, -1.5});
    auto bytes = encode(recs);
    const auto t0 = std::chrono::steady_clock::now();
    m.on_scan(0, 0, true, f(0.1f), f(10.0f), bytes.data(), bytes.size());
    const auto t1 = std::chrono::steady_clock::now();
    const bool pass = is(fwd(m, 1), Kind::Pass);
    const auto t2 = std::chrono::steady_clock::now();
    check(pass, "1,000,000 readings outside");
    std::printf(
      "      1,000,000 readings: scan event %.1f ns per reading (decoding and conversion to mm), "
      "command event %.1f ns per reading (two zones)\n",
      std::chrono::duration<double, std::nano>(t1 - t0).count() / 1e6,
      std::chrono::duration<double, std::nano>(t2 - t1).count() / 1e6);
    recs[999998] = Rec{0.2f, 0.1, 0.0};
    recs[999999] = Rec{0.2f, 0.0, -0.1};
    scan(m, 0, 0, recs);
    check(is(fwd(m, 1), Kind::StopZone, 0), "the last two of 1,000,000 readings are inside");
    recs.assign(1000000, Rec{0.2f, -0.1, 0.0});
    scan(m, 0, 0, recs);
    check(is(fwd(m, 1), Kind::StopZone, 0), "1,000,000 readings inside");
  }
  {
    // A build empties the builder; moves; a moved-from monitor throws.
    CM::Builder b;
    b.set_source(true, 7.5e-9, std::nullopt);
    b.set_stop_pub_timeout(1e-7);
    b.add_circle(0.3, CM::ZoneAction::Stop, 0.0, 0, true);
    auto m1 = b.build();
    check(b.add_circle(0.3, CM::ZoneAction::Slowdown, 0.5, 0, true) == 0, "builder is empty after a build");
    auto m2 = b.build();
    check(is(fwd(m1, 1), Kind::StopSource), "first monitor has the source timeout: no scan, stop");
    check(is(fwd(m2, 1), Kind::Slowdown, 0, d(0.5)),
      "second monitor is independent, and the source settings were reset");
    scan(m1, 0, 0, {});

    CM m3(std::move(m1));
    check(is(fwd(m3, 2), Kind::StopZone, 0), "move-constructed monitor works");
    m2 = std::move(m3);
    check(is(fwd(m2, 3), Kind::StopZone, 0), "move-assigned monitor works");
    int threw = 0;
    try {
      fwd(m3, 4);
    } catch (const std::logic_error &) {
      ++threw;
    }
    try {
      m3.on_scan(0, 0, true, 0, 0, nullptr, 0);
    } catch (const std::logic_error &) {
      ++threw;
    }
    try {
      m3.ok();
    } catch (const std::logic_error &) {
      ++threw;
    }
    try {
      m3.zone_error(0);
    } catch (const std::logic_error &) {
      ++threw;
    }
    try {
      m3.error();
    } catch (const std::logic_error &) {
      ++threw;
    }
    check(threw == 5, "a moved-from monitor throws instead of crashing");
    // An exception that crosses a frame which has called into Lean is caught
    // (PLAN, lesson 14: Lean's unwinder must not replace the system's).
    bool caught = false;
    try {
      auto m = standard();
      scan(m, 0, 0, at(100, 0));
      if (is(fwd(m, 1), Kind::StopZone, 0)) {
        throw std::runtime_error("thrown after calls into Lean");
      }
    } catch (const std::runtime_error & e) {
      caught = std::string(e.what()) == "thrown after calls into Lean";
    }
    check(caught, "an exception thrown after calls into Lean is caught");
  }
  {
    // Repeated build cycles: reference counting on every path.
    bool all = true;
    for (int n = 0; n < 20000; ++n) {
      CM::Builder b;
      b.set_source(true, 0.0, n % 3 == 0 ? std::optional<double>(0.0) : std::nullopt);
      b.set_stop_pub_timeout(5e-9);
      b.add_polygon(CM::ZoneAction::Stop, 0.0, 1, true);
      b.add_vertex(0.1f, 0.1f);
      b.add_vertex(0.1f, -0.1f);
      b.add_vertex(-0.1f, -0.1f);
      b.add_vertex(-0.1f, 0.1f);
      b.add_circle(0.5, CM::ZoneAction::Slowdown, 0.5, 1, n % 2 == 0);
      auto m = b.build();
      all = m.ok() && m.zone_error(1) == Err::None && m.error() == CErr::None && all;
      scan(m, 0, 0, at(300, 0, 1));
      all = (fwd(m, 1).kind == (n % 2 == 0 ? Kind::Slowdown : Kind::Pass)) && all;
      if (n % 5 == 0) {
        scan(m, 0, 0, at(300, 0, 1), false);
      }
    }
    check(all, "20000 build cycles");
  }
  {
    // Long run: catches reference-count mistakes (leak or double free).
    auto m = standard();
    bool all = true;
    const auto near = encode(at(100, 0));
    const auto far = encode(at(2000, 0));
    for (uint64_t t = 0; t < 300000; ++t) {
      const bool stop = t % 3 == 0;
      const auto & bytes = stop ? near : far;
      m.on_scan(t, static_cast<int64_t>(t), true, f(0.1f), f(10.0f), bytes.data(), bytes.size());
      all = (fwd(m, t).kind == (stop ? Kind::StopZone : Kind::Pass)) && all;
      all = m.ok() && all;
    }
    check(all, "300000 scan and command events");
  }

  // ---- The conversions Lean performs, against floating-point references ----

  std::mt19937_64 gen(1);
  {
    // Scan coordinates: truncated toward zero, saturated, NaN to +limit.
    check(CM::coord_mm(0.0) == 0 && CM::coord_mm(-0.0) == 0, "coord_mm(0)");
    check(CM::coord_mm(0.25) == 250 && CM::coord_mm(-0.25) == -250, "exact millimetre values are kept");
    check(CM::coord_mm(std::nextafter(0.25, 0.0)) == 249, "just below +250 mm truncates to 249");
    check(CM::coord_mm(std::nextafter(0.25, 1.0)) == 250, "just above +250 mm stays 250");
    check(CM::coord_mm(std::nextafter(-0.25, 0.0)) == -249, "just inside -250 mm truncates to -249");
    check(CM::coord_mm(std::nextafter(-0.25, -1.0)) == -250, "just beyond -250 mm stays -250");
    check(CM::coord_mm(0.0009999) == 0 && CM::coord_mm(-0.0009999) == 0, "less than 1 mm is 0, either sign");
    check(CM::coord_mm(0.0019999) == 1 && CM::coord_mm(-0.0019999) == -1, "1.9999 mm is 1, either sign");
    // 0.3 as a double is below 0.3. Its floating-point product with 1000
    // rounds up to 300.0; the exact value is 299.99999999999998…
    check(0.3 * 1000.0 == 300.0 && CM::coord_mm(0.3) == 299 && CM::coord_mm(-0.3) == -299,
      "0.3 (just below 300 mm) truncates to 299: exact, where the rounded product gives 300");
    check(CM::coord_mm(5e-324) == 0 && CM::coord_mm(-5e-324) == 0 && CM::coord_mm(2.3e-308) == 0,
      "denormals and tiny values are 0");
    check(CM::coord_mm(999.9999) == 999999 && CM::coord_mm(-999.9999) == -999999, "just inside the limit");
    check(CM::coord_mm(1000.0) == 1000000 && CM::coord_mm(-1000.0) == -1000000, "at the limit");
    check(CM::coord_mm(1023.9) == 1000000 && CM::coord_mm(1024.0) == 1000000 &&
      CM::coord_mm(-1024.5) == -1000000, "beyond the limit");
    check(CM::coord_mm(1e300) == 1000000 && CM::coord_mm(-1e300) == -1000000 &&
      CM::coord_mm(std::numeric_limits<double>::max()) == 1000000, "huge values");
    check(CM::coord_mm(kInf) == 1000000 && CM::coord_mm(-kInf) == -1000000, "infinities");
    check(CM::coord_mm(kNaN) == 1000000 && CM::coord_mm(-kNaN) == 1000000,
      "NaN of either sign saturates to the positive limit");
    std::uniform_real_distribution<double> dist(-1100.0, 1100.0);
    std::uniform_int_distribution<uint64_t> any;
    bool exact = true, patterns = true;
    for (int n = 0; n < 300000; ++n) {
      double v = dist(gen);
      if (n % 3 == 1) {
        v /= 1000.0;
      } else if (n % 3 == 2) {
        // Next to a whole millimetre.
        v = std::nextafter(std::round(v * 1000.0) / 1000.0, n % 2 ? kInf : -kInf);
      }
      const int64_t mm = CM::coord_mm(v);
      exact = mm == expected_coord(v) && exact;
      double w;
      const uint64_t bits = any(gen);
      std::memcpy(&w, &bits, sizeof w);
      patterns = CM::coord_mm(w) == expected_coord(w) && patterns;
    }
    // The exact truncation is never away from the origin and less than 1 mm
    // from the value (proved in Lean: coordMm_abs_le, coordMm_abs_gt). A
    // check of that against the rounded product v * 1000.0 would be wrong:
    // for 0.3 the rounded product is 300.0, a full millimetre above 299.
    check(exact, "300000 random coordinates: equal to the exact truncation");
    check(patterns, "300000 random bit patterns: equal to the exact truncation");
  }
  {
    // Radius: magnitude, rounded up, plus 1.
    check(CM::radius_mm(0.25) == 251 && CM::radius_mm(-0.25) == 251, "0.25 m: 250 + 1, either sign");
    check(CM::radius_mm(std::nextafter(0.25, 1.0)) == 252, "just above 0.25 m: rounded up to 251, + 1");
    check(CM::radius_mm(std::nextafter(0.25, 0.0)) == 251, "just below 0.25 m: rounded up to 250, + 1");
    check(CM::radius_mm(0.0) == 1 && CM::radius_mm(-0.0) == 1 && CM::radius_mm(5e-324) == 2,
      "0 m: 0 + 1; the smallest positive value: 1 + 1");
    check(CM::radius_mm(0.3) == 301 && CM::radius_mm(0.1) == 102,
      "0.3 as a double is below 300 mm: 301. 0.1 is above 100 mm: 102");
    check(CM::radius_mm(999.9989) == 1000000 && !CM::radius_mm(999.9991).has_value(), "the limit");
    check(!CM::radius_mm(kNaN) && !CM::radius_mm(kInf) && !CM::radius_mm(-kInf) && !CM::radius_mm(1e300),
      "NaN, infinite and huge radii are refused");
    std::uniform_real_distribution<double> dist(-999.9, 999.9);
    bool exact = true, up = true;
    for (int n = 0; n < 200000; ++n) {
      const double r = n % 2 ? dist(gen) : dist(gen) / 1000.0;
      const auto mm = CM::radius_mm(r);
      exact = mm.has_value() && static_cast<double>(*mm) == exact_ceil_mm(r) + 1.0 && exact;
      up = mm.has_value() && static_cast<double>(*mm) >= std::fabs(r) * 1000.0 + 1.0 && up;
    }
    check(exact, "200000 random radii: equal to the exact ceiling plus 1");
    check(up, "200000 random radii: at least 1 mm above the magnitude");
  }
  {
    // Vertices: rounded to nearest, a half away from zero.
    check(CM::vertex_mm(0.25f) == 250 && CM::vertex_mm(-0.25f) == -250, "exact values");
    check(CM::vertex_mm(0.3f) == 300 && CM::vertex_mm(-0.3f) == -300, "0.3f");
    check(CM::vertex_mm(0.0004f) == 0 && CM::vertex_mm(-0.0004f) == 0, "0.4 mm rounds to 0");
    check(CM::vertex_mm(0.0006f) == 1 && CM::vertex_mm(-0.0006f) == -1, "0.6 mm rounds to 1, either sign");
    check(CM::vertex_mm(0.2996f) == 300 && CM::vertex_mm(-0.2994f) == -299, "299.6 and -299.4 mm");
    check(CM::vertex_mm(0.0625f) == 63 && CM::vertex_mm(-0.0625f) == -63,
      "62.5 mm (exact in binary) rounds away from zero");
    check(CM::vertex_mm(0.0f) == 0 && CM::vertex_mm(-0.0f) == 0 && CM::vertex_mm(1e-45f) == 0, "zeros");
    check(!CM::vertex_mm(kNaNF) && !CM::vertex_mm(kInfF) && !CM::vertex_mm(-kInfF), "non-finite refused");
    check(CM::vertex_mm(3e38f) == (1LL << 62) && CM::vertex_mm(-3e38f) == -(1LL << 62),
      "huge values are reported at the query's limit");
    std::uniform_real_distribution<float> dist(-1000.0f, 1000.0f);
    bool exact = true;
    for (int n = 0; n < 200000; ++n) {
      const float v = n % 2 ? dist(gen) : dist(gen) / 1000.0f;
      const double want = exact_round_mm(static_cast<double>(v));
      exact = CM::vertex_mm(v) == static_cast<int64_t>(std::signbit(v) ? -want : want) && exact;
    }
    check(exact, "200000 random vertices: equal to the exact rounding");
  }
  {
    // Timeouts: (int64_t)(seconds * 1e9), the expression of
    // rclcpp::Duration::from_seconds, reproduced bit for bit.
    check(CM::timeout_ns(2.0) == 2000000000 && CM::timeout_ns(0.0) == 0 && CM::timeout_ns(-0.0) == 0,
      "whole seconds");
    check(CM::timeout_ns(0.1) == 100000000 && CM::timeout_ns(1.9999999999) == 1999999999,
      "fractions; truncated, not rounded");
    check(CM::timeout_ns(9.99e-10) == 0 && CM::timeout_ns(5e-324) == 0, "below 1 ns is 0 (R7)");
    check(CM::timeout_ns(2.094) == 2093999999, "2.094 s is 2,093,999,999 ns, as upstream");
    check(CM::timeout_ns(std::nextafter(9.2e9, 0.0)).has_value() && !CM::timeout_ns(9.2e9), "the limit");
    check(!CM::timeout_ns(-1e-9) && !CM::timeout_ns(-5e-324) && !CM::timeout_ns(kNaN) &&
      !CM::timeout_ns(kInf) && !CM::timeout_ns(-kInf) && !CM::timeout_ns(1e300),
      "negative, non-finite and huge timeouts are refused");
    std::uniform_real_distribution<double> exponent(-12.0, 9.9);
    std::uniform_int_distribution<int> thousandths(0, 30000);
    bool same = true, same_default = true;
    int lost = 0;
    for (int n = 0; n < 300000; ++n) {
      const double s = n % 2 ? std::pow(10.0, exponent(gen)) : thousandths(gen) / 1000.0;
      const int64_t ns = static_cast<int64_t>(s * 1e9);
      same = CM::timeout_ns(s) == ns && same;
      // The default of the source's timeout: through seconds and back.
      const int64_t again = static_cast<int64_t>((static_cast<double>(ns) / 1e9) * 1e9);
      same_default = CM::source_timeout_ns(s, std::nullopt) == again && same_default;
      lost += again != ns;
    }
    check(same, "300000 random timeouts: equal to (int64_t)(seconds * 1e9)");
    check(same_default, "and the default of the source's timeout equals the double conversion");
    check(lost > 0, "which differs from the single conversion for some of them");
    check(CM::source_timeout_ns(2.094, std::nullopt) == 2093999998,
      "node timeout 2.094 s as the source's default: 2,093,999,998 ns (1 ns lost, as upstream)");
    check(CM::source_timeout_ns(2.094, 2.094) == 2093999999, "set on the source itself: 2,093,999,999 ns");
    check(CM::source_timeout_ns(5.0, 0.25) == 250000000 && !CM::source_timeout_ns(-1.0, 0.25) &&
      !CM::source_timeout_ns(5.0, kNaN), "own value used; either value can be refused");
  }

  std::printf("%d failure(s)\n", failures);
  return failures == 0 ? 0 : 1;
}
