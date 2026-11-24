// Transforms: the only file of the shim that calls tf2, and the only one
// that computes with a transform.
//
// Everything here is *trusted* (docs/collision_monitor/SPEC_NAV2.md, section
// 7: T2 and T6). The Lean core receives points in the base frame and does
// not see the transform; that the points handed over are the images of the
// stored readings under the right transform is this file's work, and no
// theorem covers it. It is kept free of the node, so that it can be read in
// one sitting and tested on its own (test/smoke_transforms.cpp).
//
//   T2 (a)  scan_frame_point     the point of a reading in the scan frame (R3)
//   T2 (b)  to_tf2, image        the transform of N25 and the projection
//           scan_points, polygon_vertices   the byte arrays for the core
//   T6 (b)  scan_request, shape_request     which transform N23, N24, N33 ask for
//           try_lookup           one attempt, without waiting; ND7
//
// No function here decides anything about a command or a zone, tests the
// range of a reading, or skips one: which readings count is decided in Lean
// from the stored ranges. No value is converted to millimetres here.
//
// Compiled with -ffp-contract=off like the rest of the package (decided by
// the user: the same results on every platform). tf2's own code is used for
// N25 (`tf2::fromMsg`, `tf2::Transform::operator*`), so that the operations
// and their order are upstream's by construction; since those are inline
// functions, the flag applies to them here.

#ifndef ROSLEAN_COLLISION_MONITOR__TRANSFORMS_HPP_
#define ROSLEAN_COLLISION_MONITOR__TRANSFORMS_HPP_

#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Transform.hpp"
#include "tf2/LinearMath/Vector3.hpp"
#include "tf2/buffer_core.hpp"
#include "tf2/exceptions.hpp"
#include "tf2/time.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace roslean_collision_monitor
{

/// Bytes per coordinate pair handed to the core: x and y in metres in the
/// base frame, as binary64 bit patterns, little-endian.
constexpr size_t kPairSize = 16;

inline void put_pair(uint8_t * out, double x, double y)
{
  // The bytes of the two doubles, least significant first (the patterns are
  // copied, not converted).
  uint64_t bx, by;
  std::memcpy(&bx, &x, sizeof bx);
  std::memcpy(&by, &y, sizeof by);
  for (int k = 0; k < 8; ++k) {
    out[k] = static_cast<uint8_t>(bx >> (8 * k));
    out[8 + k] = static_cast<uint8_t>(by >> (8 * k));
  }
}

/// T2 (a), R3. The point of a reading in the scan frame, as upstream's
/// Scan::getData computes it: range * cos(angle) and range * sin(angle) are
/// single-precision products, widened to double. (The angle is accumulated
/// in single precision by the caller, scan_points.)
///
/// The bits are upstream's when this is compiled with optimisation (the
/// package's default build type is Release): without it the compiler calls
/// another library entry for cos and sin, and about 1 % of the coordinates
/// differ by one unit in the last place of a float (smoke_conversions).
inline std::array<double, 2> scan_frame_point(float range, float angle)
{
  const float xf = range * std::cos(angle);
  const float yf = range * std::sin(angle);
  return {static_cast<double>(xf), static_cast<double>(yf)};
}

/// ND7. The seven numbers of a transform are finite and the quaternion is
/// not zero. tf2 returns no other transform, by its source; were it to, the
/// transform must not be applied: the points would be NaN, and a NaN becomes
/// a point that lies in no zone.
inline bool transform_usable(const geometry_msgs::msg::Transform & t)
{
  const bool finite =
    std::isfinite(t.translation.x) && std::isfinite(t.translation.y) &&
    std::isfinite(t.translation.z) && std::isfinite(t.rotation.x) &&
    std::isfinite(t.rotation.y) && std::isfinite(t.rotation.z) && std::isfinite(t.rotation.w);
  const bool zero =
    t.rotation.x == 0.0 && t.rotation.y == 0.0 && t.rotation.z == 0.0 && t.rotation.w == 0.0;
  return finite && !zero;
}

/// T2 (b), N25. The looked-up message as a matrix and an origin, by tf2's own
/// conversion; nothing if the transform is not usable (ND7).
inline std::optional<tf2::Transform> to_tf2(const geometry_msgs::msg::Transform & t)
{
  if (!transform_usable(t)) {
    return std::nullopt;
  }
  tf2::Transform out;
  tf2::fromMsg(t, out);
  return out;
}

/// The identity, which upstream applies when the two frames have the same
/// name (N23, N33).
inline tf2::Transform identity_transform()
{
  tf2::Transform t;
  t.setIdentity();
  return t;
}

/// T2 (b), N25. The image of the point (x, y, 0) by tf2's own product, the
/// third coordinate dropped: the projection on the base frame's x-y plane.
inline std::array<double, 2> image(const tf2::Transform & tf, double x, double y)
{
  const tf2::Vector3 p_s(x, y, 0.0);
  const tf2::Vector3 p_b = tf * p_s;
  return {p_b.x(), p_b.y()};
}

/// The array of a command's points: 16 bytes for every reading, valid or
/// not, in scan order. The angle accumulates in single precision, also over
/// readings that will not count, as upstream's does (R3).
inline std::vector<uint8_t> scan_points(
  const tf2::Transform & tf, float angle_min, float angle_increment,
  const std::vector<float> & ranges)
{
  std::vector<uint8_t> out(kPairSize * ranges.size());
  float angle = angle_min;
  for (size_t i = 0; i < ranges.size(); ++i) {
    const auto s = scan_frame_point(ranges[i], angle);
    const auto b = image(tf, s[0], s[1]);
    put_pair(out.data() + kPairSize * i, b[0], b[1]);
    angle += angle_increment;
  }
  return out;
}

/// The vertices of a polygon message in the base frame (N33, step 3): 16
/// bytes per point, from the `float` coordinates of the message, `z` taken
/// as 0.
inline std::vector<uint8_t> polygon_vertices(
  const tf2::Transform & tf, const std::vector<geometry_msgs::msg::Point32> & points)
{
  std::vector<uint8_t> out(kPairSize * points.size());
  for (size_t i = 0; i < points.size(); ++i) {
    const auto b = image(tf, static_cast<double>(points[i].x), static_cast<double>(points[i].y));
    put_pair(out.data() + kPairSize * i, b[0], b[1]);
  }
  return out;
}

/// Which transform is asked for.
struct Request
{
  /// The two frame names are equal and no times are involved: the identity,
  /// nothing is looked up.
  bool identity{false};
  /// N24: the lookup through a fixed frame at two times. Otherwise N23: the
  /// newest transform available.
  bool through_fixed{false};
  std::string target;       ///< the base frame
  std::string source;       ///< the frame of the scan or of the polygon message
  std::string fixed;        ///< the odom frame (N24 only)
  int64_t target_ns{0};     ///< the node clock at the command (N24 only)
  int64_t source_ns{0};     ///< the stamp of the scan (N24 only)
};

/// T6 (b). The transform for a scan (Source::getTransform). Without base
/// shift correction (N23): the identity if the scan's frame is the base
/// frame by name, else the newest transform from the scan's frame to the
/// base frame. With it (N24): from the scan's frame at the scan's stamp to
/// the base frame at `now_ns`, through the odom frame; this lookup is made
/// also when the two names are equal.
inline Request scan_request(
  const std::string & base_frame, const std::string & scan_frame, int64_t scan_stamp_ns,
  int64_t now_ns, const std::string & odom_frame, bool base_shift_correction)
{
  Request r;
  r.target = base_frame;
  r.source = scan_frame;
  if (base_shift_correction) {
    r.through_fixed = true;
    r.fixed = odom_frame;
    r.target_ns = now_ns;
    r.source_ns = scan_stamp_ns;
  } else {
    r.identity = scan_frame == base_frame;
  }
  return r;
}

/// T6 (b). The transform for a polygon message (N33, N34): the identity if
/// its frame is the base frame by name, else the newest transform.
inline Request shape_request(const std::string & base_frame, const std::string & frame)
{
  Request r;
  r.target = base_frame;
  r.source = frame;
  r.identity = frame == base_frame;
  return r;
}

/// T6 (b). One attempt at the lookup, without waiting. Nothing if tf2
/// refuses (unknown frame, no connection, extrapolation, an empty name), or
/// if the transform it returns is not usable (ND7); `why` then holds the
/// reason. The wait of N26 is the caller's loop around this.
inline std::optional<tf2::Transform> try_lookup(
  const tf2::BufferCore & buffer, const Request & r, std::string * why = nullptr)
{
  if (r.identity) {
    return identity_transform();
  }
  try {
    geometry_msgs::msg::TransformStamped t;
    if (r.through_fixed) {
      t = buffer.lookupTransform(
        r.target, tf2::TimePoint(std::chrono::nanoseconds(r.target_ns)), r.source,
        tf2::TimePoint(std::chrono::nanoseconds(r.source_ns)), r.fixed);
    } else {
      t = buffer.lookupTransform(r.target, r.source, tf2::TimePointZero);
    }
    auto out = to_tf2(t.transform);
    if (!out && why) {
      *why = "the transform is not finite or its quaternion is zero";
    }
    return out;
  } catch (const tf2::TransformException & e) {
    if (why) {
      *why = e.what();
    }
    return std::nullopt;
  }
}

}  // namespace roslean_collision_monitor

#endif  // ROSLEAN_COLLISION_MONITOR__TRANSFORMS_HPP_
