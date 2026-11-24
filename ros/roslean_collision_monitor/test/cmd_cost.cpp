// What a command costs in the Lean core, against the budget.
//
// A command tests every stored reading against every zone in exact integer
// arithmetic (docs/collision_monitor/COMPARISON.md, "Cost of a command").
// The budget per command, back to back, for two zones of either kind:
//
//     360 readings     50 us
//    1080 readings    150 us
//  10,000 readings   1500 us
//
// that is about 140 ns per reading; upstream needs 3 to 7 ns per reading
// (bench_cm). Measured on the machine the budget was drawn on (Apple M5,
// macOS, after task T03): 10 to 20 / 17 to 27 us, 29 to 39 / 44 to 60 us
// and 0.24 / 0.39 ms for two circles / two 4-vertex polygons, so the budget
// leaves a factor of two or more for a busy machine. Before T03 the same
// commands took 208 us and 640 us at 1080 readings, and the limit this test
// asserted then (25 ns per reading and zone, a line drawn by T02) was not
// met.
//
//   cmd_cost [factor, default 1: the budget is multiplied by it]
//
// For each size the program stores a scan with nothing near (so every zone
// is evaluated in full), then times 400 commands and takes the median; it
// does that three times and takes the lowest, because other work on the
// machine only adds time. It does this for two circles and for the two
// boxes of Nav2's example configuration.
//
// The commands are timed back to back, which is the most favourable case.
// A node gets one command every 20 ms or so and finds caches cold and the
// processor idle; the program also reports the median of 100 commands sent
// 20 ms apart at 1080 readings (reported only: it depends on the machine's
// power management; on the Apple M5 it is four to five times the figure
// back to back).
//
// Exit code 0 iff every configuration is within the budget.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <optional>
#include <thread>
#include <utility>
#include <vector>

#include "roslean_collision_monitor/conversions.hpp"
#include "roslean_core/collision_monitor.hpp"

namespace
{

using roslean::CollisionMonitor;
using roslean_collision_monitor::put_le;

constexpr int kCommands = 400;

// Microseconds for a command with `kReadings` stored readings: the median
// of kCommands sent back to back, or of 100 sent `pace_ms` apart.
double measure(bool polygons, size_t kReadings, int pace_ms = 0)
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
  const int commands = pace_ms ? 100 : kCommands;
  for (int k = 0; k < commands + 40; ++k) {
    if (pace_ms && k >= 40) {
      std::this_thread::sleep_for(std::chrono::milliseconds(pace_ms));
    }
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
  return ns[ns.size() / 2] / 1000.0;
}

}  // namespace

int main(int argc, char ** argv)
{
  const double factor = argc > 1 ? std::strtod(argv[1], nullptr) : 1.0;
  struct Row {size_t readings; double budget_us;};
  const Row rows[] = {{360, 50.0}, {1080, 150.0}, {10000, 1500.0}};
  bool ok = true;
  std::printf(
    "a command, back to back, the lowest of three medians of %d, in microseconds:\n", kCommands);
  std::printf("  readings   two circles   two 4-vertex polygons   budget\n");
  for (const auto & row : rows) {
    // The lowest of three medians: other work on the machine only adds time.
    double circles = measure(false, row.readings);
    double polygons = measure(true, row.readings);
    for (int again = 0; again < 2; ++again) {
      circles = std::min(circles, measure(false, row.readings));
      polygons = std::min(polygons, measure(true, row.readings));
    }
    const double budget = row.budget_us * factor;
    const bool within = circles <= budget && polygons <= budget;
    ok = ok && within;
    std::printf(
      "  %8zu   %11.1f   %21.1f   %6.0f   %s\n", row.readings, circles, polygons, budget,
      within ? "ok" : "FAIL");
  }
  const double circles_paced = measure(false, 1080, 20);
  const double polygons_paced = measure(true, 1080, 20);
  std::printf(
    "one command every 20 ms, 1080 readings, median of 100 (reported only):\n"
    "  two circles %.1f us, two 4-vertex polygons %.1f us\n", circles_paced, polygons_paced);
  std::printf("%s  every command is within the budget\n", ok ? "ok  " : "FAIL");
  return ok ? EXIT_SUCCESS : EXIT_FAILURE;
}
