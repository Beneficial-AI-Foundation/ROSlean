// Reference for smoke_conversions.cpp: the points of a scan as upstream's
// Scan::getData computes them (scan.cpp of nav2_collision_monitor, R3), for
// a scan that is already in the base frame.
//
// This file is deliberately a translation unit of its own:
//
//   - it does not include conversions.hpp, so it shares no code with the
//     shim's encode_scan;
//   - CMake compiles it with -O2 whatever the build type of the package, as
//     an optimised build of upstream is compiled. With optimisation a
//     compiler replaces the pair cos(angle), sin(angle) by one call that
//     returns both (`__sincosf_stret` on macOS, `sincosf` with glibc), and
//     that call need not round as the two separate calls do.
//
// smoke_conversions compares encode_scan, compiled with the flags of the
// node, with this, bit for bit. The comparison fails if the node is built in
// a way that computes other points than an optimised upstream on the same
// machine (review CM3/CM4, finding 5: 1.1 % of the coordinates differ on
// macOS arm64 when the node is built without optimisation).
//
// Upstream then multiplies each point by the transform to the base frame,
// which for a scan in the base frame is the identity. That product changes
// no value; it turns a coordinate of -0.0 into +0.0, which is the same
// millimetre. It is left out here.

#include <array>
#include <cmath>
#include <cstddef>
#include <vector>

std::vector<std::array<double, 2>> upstream_points(
  float angle_min, float angle_increment, const std::vector<float> & ranges)
{
  std::vector<std::array<double, 2>> data;
  // Upstream's loop, without its test of the range (the shim produces a
  // point for every reading; validity is decided in Lean, R4).
  float angle = angle_min;
  for (size_t i = 0; i < ranges.size(); i++) {
    const double x = ranges[i] * std::cos(angle);
    const double y = ranges[i] * std::sin(angle);
    data.push_back({x, y});
    angle += angle_increment;
  }
  return data;
}
