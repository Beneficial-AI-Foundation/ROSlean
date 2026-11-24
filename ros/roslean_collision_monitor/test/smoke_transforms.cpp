// Unit tests of transforms.hpp: the trusted code that looks transforms up
// and computes the points of a scan and the vertices of a polygon message in
// the base frame (docs/collision_monitor/SPEC_NAV2.md: N23 to N25, N33, ND7;
// trusted items T2 and T6). No node and no ROS graph: the lookups run
// against a tf2::BufferCore that the test fills.
//
// What is pinned here:
//   - which transform is asked for (source and target frame, the two times
//     and the fixed frame, in the right order), by results that a swapped
//     argument would change;
//   - that every kind of failure gives "none", and so does a transform with
//     a number that is not finite or with a zero quaternion (ND7);
//   - the arithmetic of N25: bit for bit a reference written out operation
//     by operation, on exact and on general quaternions; no multiplication
//     is fused with an addition (-ffp-contract=off);
//   - that a point is produced for every reading, valid or not.

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <optional>
#include <random>
#include <string>
#include <vector>

#include "roslean_collision_monitor/conversions.hpp"
#include "roslean_collision_monitor/transforms.hpp"

namespace
{

using namespace roslean_collision_monitor;  // NOLINT

int failures = 0;

void check(bool cond, const std::string & what)
{
  std::printf("%s  %s\n", cond ? "ok  " : "FAIL", what.c_str());
  if (!cond) {
    ++failures;
  }
}

const double kNaN = std::numeric_limits<double>::quiet_NaN();
const double kInf = std::numeric_limits<double>::infinity();
const int64_t kS = 1000000000;

geometry_msgs::msg::Transform tfm(
  double ox, double oy, double oz, double qx, double qy, double qz, double qw)
{
  geometry_msgs::msg::Transform t;
  t.translation.x = ox;
  t.translation.y = oy;
  t.translation.z = oz;
  t.rotation.x = qx;
  t.rotation.y = qy;
  t.rotation.z = qz;
  t.rotation.w = qw;
  return t;
}

void set(
  tf2::BufferCore & b, const std::string & parent, const std::string & child, int64_t stamp_ns,
  const geometry_msgs::msg::Transform & t, bool is_static = false)
{
  geometry_msgs::msg::TransformStamped s;
  s.header.frame_id = parent;
  s.header.stamp.sec = static_cast<int32_t>(stamp_ns / kS);
  s.header.stamp.nanosec = static_cast<uint32_t>(stamp_ns % kS);
  s.child_frame_id = child;
  s.transform = t;
  b.setTransform(s, "test", is_static);
}

bool same_bits(double a, double b)
{
  return bits_of(a) == bits_of(b) || (std::isnan(a) && std::isnan(b));
}

double le_double(const std::vector<uint8_t> & b, size_t o)
{
  uint64_t v = 0;
  for (int k = 0; k < 8; ++k) {
    v |= static_cast<uint64_t>(b[o + k]) << (8 * k);
  }
  return double_of_bits(v);
}

// N25 written out operation by operation, as tf2 computes it
// (tf2/LinearMath: Quaternion::length2, Matrix3x3::setRotation,
// Transform::operator*, Vector3::dot). Every product is stored in a volatile
// double before it is used, so that no compiler can fuse it with the
// addition that follows.
std::array<double, 2> reference(const geometry_msgs::msg::Transform & t, double px, double py)
{
  const double qx = t.rotation.x, qy = t.rotation.y, qz = t.rotation.z, qw = t.rotation.w;
  volatile double x2 = qx * qx, y2 = qy * qy, z2 = qz * qz, w2 = qw * qw;
  volatile double d = ((x2 + y2) + z2) + w2;
  volatile double s = 2.0 / d;
  volatile double xs = qx * s, ys = qy * s, zs = qz * s;
  volatile double wx = qw * xs, wy = qw * ys, wz = qw * zs;
  volatile double xx = qx * xs, xy = qx * ys, xz = qx * zs;
  volatile double yy = qy * ys, yz = qy * zs, zz = qz * zs;
  volatile double m00 = 1.0 - (yy + zz), m01 = xy - wz, m02 = xz + wy;
  volatile double m10 = xy + wz, m11 = 1.0 - (xx + zz), m12 = yz - wx;
  volatile double a0 = m00 * px, a1 = m01 * py, a2 = m02 * 0.0;
  volatile double b0 = m10 * px, b1 = m11 * py, b2 = m12 * 0.0;
  volatile double x = ((a0 + a1) + a2) + t.translation.x;
  volatile double y = ((b0 + b1) + b2) + t.translation.y;
  return {x, y};
}

// The same with the two sums fused, as upstream's arm64 binary computes them
// (SPEC_NAV2, N25): to show that this test can tell the two apart.
std::array<double, 2> reference_fused(const geometry_msgs::msg::Transform & t, double px, double py)
{
  tf2::Transform m;
  tf2::fromMsg(t, m);
  const auto & r0 = m.getBasis()[0];
  const auto & r1 = m.getBasis()[1];
  const double x = std::fma(r0.z(), 0.0, std::fma(r0.y(), py, r0.x() * px)) + t.translation.x;
  const double y = std::fma(r1.z(), 0.0, std::fma(r1.y(), py, r1.x() * px)) + t.translation.y;
  return {x, y};
}

std::array<double, 2> img(const geometry_msgs::msg::Transform & t, double x, double y)
{
  return image(*to_tf2(t), x, y);
}

bool near(const std::array<double, 2> & p, double x, double y)
{
  return std::fabs(p[0] - x) < 1e-9 && std::fabs(p[1] - y) < 1e-9;
}

}  // namespace

int main()
{
  const double h = std::sqrt(0.5);
  // ---- which transform is asked for (T6 b) --------------------------------
  {
    const Request a = scan_request("base", "base", 5 * kS, 7 * kS, "odom", false);
    check(a.identity && !a.through_fixed, "N23: a scan in the base frame, no correction: the identity");
    const Request b = scan_request("base", "laser", 5 * kS, 7 * kS, "odom", false);
    check(!b.identity && !b.through_fixed && b.target == "base" && b.source == "laser",
      "N23: another frame: the newest transform from the scan's frame to the base frame");
    const Request c = scan_request("base", "base", 5 * kS, 7 * kS, "odom", true);
    check(!c.identity && c.through_fixed && c.target == "base" && c.source == "base" &&
      c.fixed == "odom" && c.target_ns == 7 * kS && c.source_ns == 5 * kS,
      "N24: with base shift correction the lookup is made also for a scan in the base frame: "
      "target time the clock, source time the scan's stamp, through odom");
    const Request d = shape_request("base", "base");
    const Request e = shape_request("base", "odom");
    const Request f = shape_request("base", "");
    check(d.identity && !e.identity && !e.through_fixed && e.target == "base" &&
      e.source == "odom" && !f.identity,
      "N33: a polygon message: the identity for the base frame by name, else the newest transform");
  }
  // ---- the lookups against a buffer ----------------------------------------
  {
    tf2::BufferCore buf;
    // The laser is 1 m ahead of the base, turned a quarter turn to the left.
    set(buf, "base", "laser", 0, tfm(1.0, 0.0, 0.0, 0.0, 0.0, h, h), true);
    std::string why;
    const auto tf = try_lookup(buf, scan_request("base", "laser", 0, 0, "odom", false), &why);
    check(tf.has_value(), "N23: a static transform is found");
    // A point 0.5 m along the laser's x axis is 0.5 m to the left of a spot
    // 1 m ahead of the base. With source and target swapped it would be at
    // (0, 0.5) - (0,1)... not there.
    check(tf && near(image(*tf, 0.5, 0.0), 1.0, 0.5),
      "N23: the transform takes a point of the scan's frame to the base frame (not the reverse)");
    check(try_lookup(buf, scan_request("base", "base", 0, 0, "odom", false)).has_value() &&
      near(image(*try_lookup(buf, scan_request("base", "base", 0, 0, "odom", false)), 0.25, -2.0),
      0.25, -2.0), "N23: the identity needs no buffer entry and changes nothing");
    check(!try_lookup(buf, scan_request("base", "nowhere", 0, 0, "odom", false), &why) && !why.empty(),
      "an unknown frame: none, with tf2's reason");
    check(!try_lookup(buf, scan_request("base", "", 0, 0, "odom", false), &why),
      "an empty frame name: none");
    set(buf, "world", "island", 0, tfm(0, 0, 0, 0, 0, 0, 1), true);
    check(!try_lookup(buf, shape_request("base", "island"), &why),
      "two frames that are not connected: none");
    // The newest transform, whatever its stamp (N23).
    set(buf, "base", "arm", 1 * kS, tfm(1.0, 0.0, 0.0, 0, 0, 0, 1));
    set(buf, "base", "arm", 3 * kS, tfm(3.0, 0.0, 0.0, 0, 0, 0, 1));
    set(buf, "base", "arm", 2 * kS, tfm(2.0, 0.0, 0.0, 0, 0, 0, 1));
    const auto newest = try_lookup(buf, shape_request("base", "arm"));
    check(newest && near(image(*newest, 0.0, 0.0), 3.0, 0.0),
      "N23, N33: the newest transform is used, also when an older one arrived later");
  }
  {
    // N24. The base drives along odom's x: at 0 m at t = 10 s, at 1 m at
    // t = 12 s. The scan is in the base frame.
    tf2::BufferCore buf;
    set(buf, "odom", "base", 10 * kS, tfm(0.0, 0.0, 0.0, 0, 0, 0, 1));
    set(buf, "odom", "base", 12 * kS, tfm(1.0, 0.0, 0.0, 0, 0, 0, 1));
    std::string why;
    const auto tf = try_lookup(buf, scan_request("base", "base", 10 * kS, 12 * kS, "odom", true), &why);
    // A point 1.2 m ahead when the scan was taken is 0.2 m ahead now. With
    // the two times swapped it would be 2.2 m ahead.
    check(tf && near(image(*tf, 1.2, 0.0), 0.2, 0.0),
      "N24: a point seen at the scan's stamp, in the base frame at the command's time");
    const auto half = try_lookup(buf, scan_request("base", "base", 10 * kS, 11 * kS, "odom", true));
    check(half && near(image(*half, 1.2, 0.0), 0.7, 0.0), "N24: tf2 interpolates between two samples");
    check(!try_lookup(buf, scan_request("base", "base", 10 * kS, 12 * kS + 1, "odom", true), &why) &&
      why.find("future") != std::string::npos,
      "N24: a command later than the newest odom sample: none (extrapolation into the future)");
    check(!try_lookup(buf, scan_request("base", "base", 10 * kS - 1, 12 * kS, "odom", true), &why) &&
      why.find("past") != std::string::npos,
      "N24: a scan older than the oldest sample: none (extrapolation into the past)");
    check(!try_lookup(buf, scan_request("base", "base", 10 * kS, 12 * kS, "map", true), &why),
      "N24: no such fixed frame: none");
    // With the scan in a sensor frame both factors apply.
    set(buf, "base", "laser", 0, tfm(0.5, 0.0, 0.0, 0, 0, 0, 1), true);
    const auto both = try_lookup(buf, scan_request("base", "laser", 10 * kS, 12 * kS, "odom", true));
    check(both && near(image(*both, 1.2, 0.0), 0.7, 0.0), "N24: sensor frame and base shift together");
  }
  // ---- ND7 ------------------------------------------------------------------
  {
    bool refused = true;
    for (int k = 0; k < 7; ++k) {
      for (double bad : {kNaN, kInf, -kInf}) {
        double v[7] = {0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0};
        v[k] = bad;
        const auto t = tfm(v[0], v[1], v[2], v[3], v[4], v[5], v[6]);
        refused = !transform_usable(t) && !to_tf2(t).has_value() && refused;
      }
    }
    check(refused, "ND7: a transform with a number that is not finite is not used (each of the seven)");
    check(!transform_usable(tfm(1, 2, 3, 0, 0, 0, 0)) && !to_tf2(tfm(1, 2, 3, 0.0, -0.0, 0.0, 0.0)),
      "ND7: a zero quaternion is not used");
    check(transform_usable(tfm(0, 0, 0, 0, 0, 0, 2)) && near(img(tfm(0, 0, 0, 0, 0, 0, 2), 0.3, -0.4), 0.3, -0.4),
      "N25: a quaternion that is not of unit length acts as its normalisation");
    check(transform_usable(tfm(0, 0, 0, 1e-200, 0, 0, 0)),
      "a tiny quaternion is not zero (what tf2 makes of it is tf2's)");
  }
  // ---- N25: geometry ---------------------------------------------------------
  {
    check(near(img(tfm(1.0, 2.0, 5.0, 0, 0, 0, 1), 0.5, 0.25), 1.5, 2.25),
      "translation; the height of the sensor is dropped (projection)");
    check(near(img(tfm(0, 0, 0, 0, 0, h, h), 1.0, 0.0), 0.0, 1.0), "yaw by a quarter turn");
    check(near(img(tfm(0, 0, 0, 1, 0, 0, 0), 1.0, 0.5), 1.0, -0.5),
      "a sensor mounted upside down mirrors left and right");
    check(near(img(tfm(3.0, 0, 0, 0, h, 0, h), 0.0, 0.25), 3.0, 0.25) &&
      near(img(tfm(0, 0, 0, 0, -h, 0, h), 3.0, 0.0), 0.0, 0.0),
      "a sensor pitched by a quarter turn: a point 3 m along its x axis lands on its own origin");
    const auto id = identity_transform();
    const auto z = image(id, -0.0, -0.0);
    check(bits_of(z[0]) == bits_of(0.0) && bits_of(z[1]) == bits_of(0.0) &&
      same_bits(image(id, 1.25, -7.5)[0], 1.25) && same_bits(image(id, 1.25, -7.5)[1], -7.5),
      "the identity changes no value but the sign of a zero");
    check(std::isnan(image(id, kNaN, 1.0)[0]) && std::isinf(image(id, kInf, 1.0)[0]),
      "NaN and infinity pass through (the core saturates them)");
  }
  // ---- N25: the arithmetic, bit for bit -------------------------------------
  {
    std::mt19937_64 gen(23);
    std::uniform_real_distribution<double> q(-1.0, 1.0), o(-20.0, 20.0), p(-30.0, 30.0);
    size_t differ = 0, total = 0, fused_differs = 0;
    const geometry_msgs::msg::Transform exact[] = {
      tfm(0, 0, 0, 0, 0, 0, 1), tfm(0.5, -0.25, 1.0, 0, 0, 1, 0), tfm(1, 2, 3, 1, 0, 0, 0),
      tfm(-1, 0, 0, 0, 1, 0, 0), tfm(0, 0, 0, 0.5, 0.5, 0.5, 0.5), tfm(0, 0, 0, 0, 0, 0, 2),
      tfm(0.125, 0, 0, 0, 0, 0.6, 0.8)};
    for (int n = 0; n < 200000; ++n) {
      geometry_msgs::msg::Transform t;
      if (n % 10 == 0) {
        t = exact[(n / 10) % (sizeof exact / sizeof exact[0])];
      } else {
        t = tfm(o(gen), o(gen), o(gen), q(gen), q(gen), q(gen), q(gen));
        if (n % 3 == 0) {
          // A unit quaternion, as tf2 returns: normalised in double.
          const double len = std::sqrt(
            t.rotation.x * t.rotation.x + t.rotation.y * t.rotation.y +
            t.rotation.z * t.rotation.z + t.rotation.w * t.rotation.w);
          t.rotation.x /= len;
          t.rotation.y /= len;
          t.rotation.z /= len;
          t.rotation.w /= len;
        }
      }
      // Scan-frame points are floats widened to double.
      const double px = static_cast<double>(static_cast<float>(p(gen)));
      const double py = static_cast<double>(static_cast<float>(p(gen)));
      const auto got = img(t, px, py);
      const auto want = reference(t, px, py);
      const auto fused = reference_fused(t, px, py);
      differ += !same_bits(got[0], want[0]) + !same_bits(got[1], want[1]);
      fused_differs += !same_bits(fused[0], want[0]) + !same_bits(fused[1], want[1]);
      total += 2;
    }
    std::printf(
      "      %zu of %zu coordinates differ from the unfused reference; a fused evaluation differs "
      "in %zu\n", differ, total, fused_differs);
    check(differ == 0,
      "200000 transforms, exact and general: the image is the reference computation, bit for bit "
      "(tf2's operations in tf2's order, no multiplication fused with an addition)");
    check(fused_differs > 0,
      "and the test can tell: with the two sums fused, some coordinates differ in the last bits");
  }
  // ---- the byte arrays for the core ------------------------------------------
  {
    const float kNaNF = std::numeric_limits<float>::quiet_NaN();
    const float kInfF = std::numeric_limits<float>::infinity();
    const std::vector<float> ranges = {1.0f, kNaNF, 2.0f, kInfF, -1.0f, 0.0f, 3.5f};
    const auto t = tfm(0.5, -0.25, 2.0, 0.1, -0.2, 0.6, 0.7);
    const auto tf = *to_tf2(t);
    const auto b = scan_points(tf, -1.0f, 0.3f, ranges);
    check(b.size() == 16 * ranges.size() && scan_points(tf, 0.0f, 0.1f, {}).empty(),
      "scan_points: 16 bytes for every reading, valid or not; none for an empty scan");
    bool all = true;
    float angle = -1.0f;
    for (size_t i = 0; i < ranges.size(); ++i) {
      const auto s = scan_frame_point(ranges[i], angle);
      const auto want = reference(t, s[0], s[1]);
      all = same_bits(le_double(b, 16 * i), want[0]) && same_bits(le_double(b, 16 * i + 8), want[1]) &&
        all;
      angle += 0.3f;  // also over readings that are not valid
    }
    check(all, "scan_points: reading i is the image of its scan-frame point, the angle accumulated "
      "in single precision over every reading; x then y, little-endian");
    check(std::isnan(le_double(b, 16)) && !std::isfinite(le_double(b, 48)),
      "scan_points: a NaN or infinite reading gives a pair that is not finite, not a skipped one");
    const auto s = scan_frame_point(2.0f, 0.5f);
    check(same_bits(s[0], static_cast<double>(2.0f * std::cos(0.5f))) &&
      same_bits(s[1], static_cast<double>(2.0f * std::sin(0.5f))),
      "scan_frame_point: single-precision products, widened (R3)");
    std::vector<geometry_msgs::msg::Point32> pts(3);
    pts[0].x = 0.1f; pts[0].y = 0.2f; pts[0].z = 9.0f;
    pts[1].x = -0.3f; pts[1].y = 0.4f;
    pts[2].x = kNaNF; pts[2].y = 1.0f;
    const auto v = polygon_vertices(tf, pts);
    const auto w0 = reference(t, static_cast<double>(0.1f), static_cast<double>(0.2f));
    const auto w1 = reference(t, static_cast<double>(-0.3f), static_cast<double>(0.4f));
    check(v.size() == 48 && same_bits(le_double(v, 0), w0[0]) && same_bits(le_double(v, 8), w0[1]) &&
      same_bits(le_double(v, 16), w1[0]) && same_bits(le_double(v, 24), w1[1]) &&
      std::isnan(le_double(v, 32)),
      "polygon_vertices: the float coordinates widened, z ignored, every point (N33)");
    const auto idv = polygon_vertices(identity_transform(), pts);
    check(same_bits(le_double(idv, 0), static_cast<double>(0.1f)) &&
      same_bits(le_double(idv, 24), static_cast<double>(0.4f)),
      "polygon_vertices under the identity: the widened floats");
  }

  std::printf("%d failure(s)\n", failures);
  return failures == 0 ? 0 : 1;
}
