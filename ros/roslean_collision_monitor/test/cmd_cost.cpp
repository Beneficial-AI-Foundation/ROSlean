// What a command costs in the Lean core, per reading and zone.
//
// EXPECTED TO FAIL at present for polygon zones
// (docs/collision_monitor/COMPARISON.md, defect 2), and registered with
// WILL_FAIL. A command tests every stored point against every zone in exact
// integer arithmetic. For a circle that is a few nanoseconds per point; for
// a four-vertex polygon it is two orders of magnitude more than upstream
// spends on a reading altogether (3 to 7 ns, measured by bench_cm).
//
//   cmd_cost [limit in ns per reading and zone, default 25]
//
// The program stores a scan of 1080 readings with nothing near (so every
// zone is evaluated in full), then times 400 commands and takes the median,
// which is steady on a busy machine. It does this for two circles, where it
// reports only, and for the two boxes of Nav2's example configuration,
// where the median must be below the limit. 25 ns is three times what the
// circle path needs and several times upstream's whole cost per reading.
//
// Exit code 0 iff the polygon path is within the limit.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <optional>
#include <utility>
#include <vector>

#include "roslean_collision_monitor/conversions.hpp"
#include "roslean_core/collision_monitor.hpp"

namespace
{

using roslean::CollisionMonitor;
using roslean_collision_monitor::put_le;

constexpr size_t kReadings = 1080;
constexpr int kCommands = 400;

// Nanoseconds per reading and zone for a command, median of kCommands.
double measure(bool polygons)
{
  CollisionMonitor::Builder b;
  b.set_source(true, 0.0, std::nullopt);
  b.set_stop_pub_timeout(1.0);
  if (polygons) {
    b.add_polygon(CollisionMonitor::ZoneAction::Stop, 0.5, 4, true);
    for (auto v : {std::pair{0.3f, 0.3f}, {0.3f, -0.3f}, {0.0f, -0.3f}, {0.0f, 0.3f}}) {
      b.add_vertex(v.first, v.second);
    }
    b.add_polygon(CollisionMonitor::ZoneAction::Slowdown, 0.3, 4, true);
    for (auto v : {std::pair{0.4f, 0.4f}, {0.4f, -0.4f}, {-0.4f, -0.4f}, {-0.4f, 0.4f}}) {
      b.add_vertex(v.first, v.second);
    }
  } else {
    b.add_circle(0.3, CollisionMonitor::ZoneAction::Stop, 0.5, 4, true);
    b.add_circle(0.6, CollisionMonitor::ZoneAction::Slowdown, 0.3, 4, true);
  }
  CollisionMonitor m = b.build();
  if (!m.ok()) {
    std::printf("FAIL  the core refuses the configuration\n");
    std::exit(EXIT_FAILURE);
  }
  // 1080 points on a circle of 3.5 m: nothing is in a zone.
  std::vector<uint8_t> rec(20 * kReadings);
  for (size_t i = 0; i < kReadings; ++i) {
    const double a = 6.283185307179586 * static_cast<double>(i) / kReadings;
    put_le(&rec[20 * i], roslean::bits_of(3.5f), 4);
    put_le(&rec[20 * i + 4], roslean::bits_of(3.5 * std::cos(a)), 8);
    put_le(&rec[20 * i + 12], roslean::bits_of(3.5 * std::sin(a)), 8);
  }
  m.on_scan(0, 0, true, roslean::bits_of(0.05f), roslean::bits_of(30.0f), rec.data(), rec.size());
  std::vector<double> ns;
  bool passed = true;
  for (int k = 0; k < kCommands + 40; ++k) {
    const auto t0 = std::chrono::steady_clock::now();
    const auto out = m.on_cmd(0, roslean::bits_of(0.5), 0, 0, 0, 0, 0);
    const auto t1 = std::chrono::steady_clock::now();
    passed = passed && out.kind == CollisionMonitor::OutputKind::Pass;
    if (k >= 40) {  // the first calls warm the caches
      ns.push_back(std::chrono::duration<double, std::nano>(t1 - t0).count());
    }
  }
  if (!passed) {
    std::printf("FAIL  a command was not passed through: the scan is not the one intended\n");
    std::exit(EXIT_FAILURE);
  }
  std::sort(ns.begin(), ns.end());
  return ns[ns.size() / 2] / (kReadings * 2.0);
}

}  // namespace

int main(int argc, char ** argv)
{
  const double limit = argc > 1 ? std::strtod(argv[1], nullptr) : 25.0;
  const double circles = measure(false);
  const double polygons = measure(true);
  std::printf(
    "a command with %zu stored readings, median of %d, per reading and zone:\n"
    "      two circles          %7.1f ns  (%.1f us per command)\n"
    "      two 4-vertex polygons %6.1f ns  (%.1f us per command)\n",
    kReadings, kCommands, circles, circles * kReadings * 2 / 1000,
    polygons, polygons * kReadings * 2 / 1000);
  const bool ok = polygons <= limit;
  std::printf(
    "%s  polygon zones cost at most %.0f ns per reading and zone\n", ok ? "ok  " : "FAIL", limit);
  return ok ? EXIT_SUCCESS : EXIT_FAILURE;
}
