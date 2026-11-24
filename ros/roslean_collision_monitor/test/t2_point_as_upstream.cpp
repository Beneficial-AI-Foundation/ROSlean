// T2 on macOS: is the point of a reading the one upstream computes?
//
// Registered on macOS only. It failed while `pixi run build` compiled the
// node without optimisation; since the package builds as Release by default
// (review CM3/CM4, finding 5) it passes, and it fails again in a Debug
// build. smoke_conversions makes the same comparison on 200 scans against a
// copy of upstream's loop; this test holds one angle and upstream's bits.
//
// docs/collision_monitor/SPEC.md, "Scan event": the point of a reading is
// computed as upstream's Scan::getData computes it, range * cos(angle) and
// range * sin(angle) in single precision. Upstream's binary (RoboStack,
// nav2_collision_monitor 1.3.12, arm64) obtains sine and cosine from one
// call of Apple's __sincosf_stret. A build of the node without optimisation
// calls cosf and sinf, and for 2.6 % of angles Apple's library returns a
// cosine or a sine from those that is one float32 step away.
//
// The angle below is the one of the random case that showed it
// (random_diff.py --roslean --yaw, seed 1057): five readings of
// 0.8880624175071716 m at this angle, seen through a frame at
// (0.25, 0.125) turned by a quarter turn, lie 15 nm inside a circle of
// 0.75 m with cosf and outside it with __sincosf_stret, and upstream's
// binary puts them outside.
//
// This test compiles conversions.hpp with the node's own flags and asks for
// the point of a reading of 1 m at that angle. It passes when x is
// upstream's value.

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

#include "roslean_collision_monitor/conversions.hpp"
#include "roslean_collision_monitor/scan_records.hpp"

int main()
{
  using namespace roslean_collision_monitor;  // NOLINT
  // volatile: the compiler must call the library, not fold the constant.
  volatile float angle_min = 1.1233999729156494f;
  const float angle = angle_min;
  const auto rec = encode_scan(angle, 0.0f, std::vector<float>{1.0f});
  uint64_t xb = 0, yb = 0;
  for (int k = 0; k < 8; ++k) {
    xb |= static_cast<uint64_t>(rec[4 + k]) << (8 * k);
    yb |= static_cast<uint64_t>(rec[12 + k]) << (8 * k);
  }
  const float x = static_cast<float>(double_of_bits(xb));
  const float y = static_cast<float>(double_of_bits(yb));
  const uint32_t upstream_x = 0x3edd8052;  // 0.4326196312904358, __sincosf_stret
  const uint32_t separate_x = 0x3edd8051;  // 0.4326196014881134, cosf
  const uint32_t upstream_y = 0x3f66cdb8;  // 0.9015765190124512, both routines
  std::printf(
    "x = %.17g (bits %08x); upstream computes %08x; cosf gives %08x\n",
    static_cast<double>(x), bits_of(x), upstream_x, separate_x);
  const bool ok = bits_of(x) == upstream_x && bits_of(y) == upstream_y;
  std::printf("%s  the point of the reading is upstream's, bit for bit\n", ok ? "ok  " : "FAIL");
  return ok ? 0 : 1;
}
