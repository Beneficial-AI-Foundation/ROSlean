// CM3/CM4 review: memory use of the C++ wrapper over many events.
//
// test_cm_ffi runs 20,000 build cycles and 300,000 events "to catch
// reference-count mistakes (leak or double free)", but it measures nothing: a
// leak would pass. This program prints the peak resident set size after N and
// after 5 N rounds of every wrapper call; a leaked object per round shows as
// growth.
//
//   c++ -std=c++17 -I<roslean_core include> leak_probe.cpp -L<lib> -lroslean_core \
//       -Wl,-rpath,<lib> -o leak_probe && ./leak_probe

#include <sys/resource.h>

#include <cstdio>
#include <cstring>
#include <optional>
#include <vector>

#include "roslean_core/collision_monitor.hpp"

using CM = roslean::CollisionMonitor;
using roslean::bits_of;

static long peak_kb()
{
  rusage u;
  getrusage(RUSAGE_SELF, &u);
#ifdef __APPLE__
  return u.ru_maxrss / 1024;
#else
  return u.ru_maxrss;
#endif
}

static std::vector<uint8_t> scan_bytes(size_t n, double x)
{
  std::vector<uint8_t> out(20 * n);
  const float r = 1.0f;
  for (size_t i = 0; i < n; ++i) {
    std::memcpy(&out[20 * i], &r, 4);
    std::memcpy(&out[20 * i + 4], &x, 8);
    std::memcpy(&out[20 * i + 12], &x, 8);
  }
  return out;
}

static void round_trip(const std::vector<uint8_t> & near, const std::vector<uint8_t> & far, int n)
{
  CM::Builder b;
  b.set_source(true, 2.0, n % 2 ? std::optional<double>(1.0) : std::nullopt);
  b.set_stop_pub_timeout(1.0);
  b.add_circle(n % 7 ? 0.3 : -1.0e9, CM::ZoneAction::Stop, 0.0, 1, true);  // sometimes refused
  b.add_polygon(CM::ZoneAction::Slowdown, 0.5, 1, true);
  for (float v : {0.5f, -0.5f}) {
    b.add_vertex(v, v);
    b.add_vertex(v, -v);
  }
  CM m = b.build();
  (void)m.ok();
  (void)m.error();
  (void)m.zone_error(0);
  for (int k = 0; k < 20; ++k) {
    const auto & s = k % 2 ? near : far;
    m.on_scan(k, k, k % 5 != 0, 0, bits_of(100.0f), s.data(), s.size());
    (void)m.on_cmd(k, bits_of(0.5), 0, 0, 0, 0, 0);
  }
  CM m2 = std::move(m);
  m = std::move(m2);
  (void)CM::coord_mm(1.5);
  (void)CM::radius_mm(1.5);
  (void)CM::vertex_mm(1.5f);
  (void)CM::timeout_ns(1.5);
  (void)CM::source_timeout_ns(1.5, std::nullopt);
}

int main()
{
  const auto near = scan_bytes(360, 0.1);
  const auto far = scan_bytes(360, 5.0);
  const int n = 20000;
  for (int k = 0; k < n; ++k) {round_trip(near, far, k);}
  const long first = peak_kb();
  for (int k = 0; k < 4 * n; ++k) {round_trip(near, far, k);}
  const long second = peak_kb();
  std::printf(
    "peak RSS after %d rounds: %ld kB; after %d rounds: %ld kB (growth %ld kB; each round is "
    "one build, 20 scans of 360 readings and 20 commands)\n", n, first, 5 * n, second,
    second - first);
  return 0;
}
