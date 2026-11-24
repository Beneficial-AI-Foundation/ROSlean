// For tests and benchmarks, not for the node: a scan as 20-byte records, the
// form roslean::ScanPointsMonitor takes (range pattern, 4 bytes; x and y in
// metres as binary64 patterns, 8 bytes each; little-endian). The point of a
// reading is computed by the node's own code (transforms.hpp): the
// scan-frame point of R3, and its image under `tf` if one is given. Without
// one the scan-frame point is written as it is (a scan in the base frame;
// the node applies the identity there, which changes no value but the sign
// of a zero).

#ifndef ROSLEAN_COLLISION_MONITOR__SCAN_RECORDS_HPP_
#define ROSLEAN_COLLISION_MONITOR__SCAN_RECORDS_HPP_

#include <cstdint>
#include <vector>

#include "roslean_collision_monitor/conversions.hpp"
#include "roslean_collision_monitor/transforms.hpp"

namespace roslean_collision_monitor
{

constexpr size_t kRecordSize = 20;

inline std::vector<uint8_t> encode_scan(
  float angle_min, float angle_increment, const std::vector<float> & ranges,
  const tf2::Transform * tf = nullptr)
{
  std::vector<uint8_t> out(kRecordSize * ranges.size());
  float angle = angle_min;
  for (size_t i = 0; i < ranges.size(); ++i) {
    auto p = scan_frame_point(ranges[i], angle);
    if (tf) {
      p = image(*tf, p[0], p[1]);
    }
    uint8_t * rec = out.data() + kRecordSize * i;
    put_le(rec, bits_of(ranges[i]), 4);
    put_pair(rec + 4, p[0], p[1]);
    angle += angle_increment;
  }
  return out;
}

}  // namespace roslean_collision_monitor

#endif  // ROSLEAN_COLLISION_MONITOR__SCAN_RECORDS_HPP_
