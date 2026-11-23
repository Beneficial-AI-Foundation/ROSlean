// Unit tests of the trusted floating-point work of the shim (conversions.hpp),
// together with the conversion to millimetres that the Lean core applies to
// its results. They pin what docs/collision_monitor/SPEC.md demands of a scan
// ("Scan event", "Safe direction"): the point of a reading is computed as
// upstream computes it, a record is produced for every reading, and the
// millimetre point the core sees is truncated toward the origin, also at
// exact millimetre boundaries and for negative coordinates, and saturates at
// 1000 m.

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <random>
#include <string>
#include <vector>

#include "roslean_collision_monitor/conversions.hpp"
#include "roslean_core/collision_monitor.hpp"

namespace
{

using namespace roslean_collision_monitor;  // NOLINT
using roslean::CollisionMonitor;

int failures = 0;

void check(bool cond, const std::string & what)
{
  std::printf("%s  %s\n", cond ? "ok  " : "FAIL", what.c_str());
  if (!cond) {
    ++failures;
  }
}

const float kInfF = std::numeric_limits<float>::infinity();
const float kNaNF = std::numeric_limits<float>::quiet_NaN();

uint64_t le_at(const std::vector<uint8_t> & b, size_t o, int bytes)
{
  uint64_t v = 0;
  for (int k = 0; k < bytes; ++k) {
    v |= static_cast<uint64_t>(b[o + k]) << (8 * k);
  }
  return v;
}

// One decoded record: the range's bits, the point in metres as written by the
// shim, and the point in millimetres as the Lean core converts it.
struct Rec
{
  uint32_t range_bits;
  double x;
  double y;
  int64_t x_mm;
  int64_t y_mm;
};

Rec rec(const std::vector<uint8_t> & b, size_t i)
{
  Rec r;
  r.range_bits = static_cast<uint32_t>(le_at(b, 20 * i, 4));
  r.x = double_of_bits(le_at(b, 20 * i + 4, 8));
  r.y = double_of_bits(le_at(b, 20 * i + 12, 8));
  r.x_mm = CollisionMonitor::coord_mm(r.x);
  r.y_mm = CollisionMonitor::coord_mm(r.y);
  return r;
}

// A single reading at the given angle, with the given pose.
Rec one(float range, float angle, const SensorPose & pose = SensorPose{})
{
  return rec(encode_scan(angle, 0.0f, {range}, pose), 0);
}

bool mm(const Rec & r, int64_t x, int64_t y) {return r.x_mm == x && r.y_mm == y;}

}  // namespace

int main()
{
  // ---- bit patterns ------------------------------------------------------
  check(bits_of(1.0) == 0x3FF0000000000000ULL && bits_of(-0.0) == 0x8000000000000000ULL,
    "bits_of(double)");
  check(bits_of(1.0f) == 0x3F800000u && bits_of(0.2f) == 0x3E4CCCCDu, "bits_of(float)");
  check(double_of_bits(0x3FE0000000000000ULL) == 0.5, "double_of_bits");
  check(bits_of(double_of_bits(0x7FF8000000000001ULL)) == 0x7FF8000000000001ULL,
    "a NaN pattern survives the round trip");

  // ---- encode_scan: the bytes --------------------------------------------
  check(encode_scan(0.0f, 0.1f, {}, SensorPose{}).empty(), "a scan of zero length gives no bytes");
  {
    const auto b = encode_scan(0.0f, 0.0f, {1.0f}, SensorPose{});
    const std::vector<uint8_t> expect = {
      0x00, 0x00, 0x80, 0x3F,                           // 1.0f
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x3F,   // x = 1.0
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // y = 0.0
    check(b == expect, "range 1 at angle 0: 20 bytes, little-endian");
    const auto n = encode_scan(3.1415927f, 0.0f, {1.0f}, SensorPose{});
    check(le_at(n, 4, 8) == 0xBFF0000000000000ULL, "range 1 at angle pi: x = -1.0");
  }

  // ---- the point of a reading, in metres and as the core's millimetres ----
  check(one(1.0f, 0.0f).x == 1.0 && one(1.0f, 0.0f).y == 0.0, "angle 0: (1, 0) m");
  check(mm(one(1.0f, 1.5707964f), 0, 1000), "angle pi/2: (0, 1000) mm; the tiny negative x truncates to 0");
  check(mm(one(1.0f, -1.5707964f), 0, -1000), "angle -pi/2: (0, -1000) mm");
  check(one(0.2f, 0.0f).x == static_cast<double>(0.2f), "the product is single precision, widened");
  check(mm(one(0.2f, 0.0f), 200, 0), "0.2f is 0.2000000030 m: 200 mm");
  check(mm(one(0.7f, 0.0f), 699, 0), "0.7f is 0.6999999881 m: truncated to 699 mm");
  check(mm(one(0.7f, 3.1415927f), -699, 0), "the same toward -x: -699 mm, toward the origin");
  check(mm(one(0.7f, -1.5707964f), 0, -699), "and toward -y");
  check(mm(one(0.25f, 0.0f), 250, 0) && mm(one(0.25f, 3.1415927f), -250, 0),
    "exactly 250 mm stays 250, either sign");
  check(mm(one(std::nextafter(0.25f, 0.0f), 0.0f), 249, 0) &&
    mm(one(std::nextafter(0.25f, 0.0f), 3.1415927f), -249, 0),
    "the float just below 250 mm truncates to 249, either sign");
  check(mm(one(std::nextafter(0.25f, 1.0f), 0.0f), 250, 0) &&
    mm(one(std::nextafter(0.25f, 1.0f), 3.1415927f), -250, 0),
    "the float just above 250 mm stays 250, either sign");
  check(mm(one(999.99f, 0.0f), 999989, 0), "999.99f is 999.98999 m: 999989 mm");
  check(mm(one(1000.0f, 0.0f), 1000000, 0), "a reading of 1000 m saturates");
  check(mm(one(2000.0f, 3.1415927f), -1000000, 0), "saturation toward -x");
  {
    // A record is produced for every reading, valid or not, with the range's
    // bits unchanged.
    const auto b = encode_scan(0.0f, 0.0f, {kNaNF, kInfF, -kInfF, -1.0f, 0.0f}, SensorPose{});
    check(b.size() == 100, "five readings, five records");
    check(rec(b, 0).range_bits == bits_of(kNaNF) && mm(rec(b, 0), 1000000, 1000000),
      "NaN reading: bits kept, point saturated");
    // inf * cos(0) = inf, inf * sin(0) = NaN.
    check(rec(b, 1).range_bits == 0x7F800000u && mm(rec(b, 1), 1000000, 1000000), "+inf reading");
    check(rec(b, 2).range_bits == 0xFF800000u && mm(rec(b, 2), -1000000, 1000000), "-inf reading");
    check(rec(b, 3).range_bits == 0xBF800000u && mm(rec(b, 3), -1000, 0), "negative reading");
    check(rec(b, 4).range_bits == 0u && mm(rec(b, 4), 0, 0), "zero reading");
  }
  {
    // The angle accumulates in single precision, one addition per reading,
    // valid or not (R3).
    const float inc = 0.0061f;
    std::vector<float> ranges(1000, 5.0f);
    ranges[10] = kNaNF;
    const auto b = encode_scan(-3.05f, inc, ranges, SensorPose{});
    bool same = true;
    int differs_from_double = 0;
    float angle = -3.05f;
    for (size_t i = 0; i < ranges.size(); ++i) {
      const float xf = ranges[i] * std::cos(angle);
      const float yf = ranges[i] * std::sin(angle);
      const Rec r = rec(b, i);
      same = bits_of(r.x) == bits_of(static_cast<double>(xf)) &&
        bits_of(r.y) == bits_of(static_cast<double>(yf)) && same;
      const double a = -3.05 + static_cast<double>(i) * 0.0061;
      differs_from_double += r.x != 5.0 * std::cos(a) || r.y != 5.0 * std::sin(a);
      angle += inc;
    }
    check(same, "1000 readings: single-precision angle accumulation and products");
    check(differs_from_double > 0, "which is distinguishable from a double-precision computation");
  }
  {
    // Whatever the reading and the angle, the millimetre point never lies
    // farther from the origin than the metre point, on either axis.
    std::mt19937_64 gen(4);
    std::uniform_real_distribution<float> range(0.0f, 30.0f), angle(-7.0f, 7.0f);
    bool toward = true, close = true;
    for (int n = 0; n < 100000; ++n) {
      const Rec r = one(range(gen), angle(gen));
      for (const auto & c : {std::array<double, 2>{r.x, static_cast<double>(r.x_mm)},
          std::array<double, 2>{r.y, static_cast<double>(r.y_mm)}})
      {
        toward = std::fabs(c[1]) <= std::fabs(c[0]) * 1000.0 && toward;
        close = std::fabs(c[0]) * 1000.0 - std::fabs(c[1]) < 1.0 && close;
      }
    }
    check(toward, "100000 random readings: the point never moves away from the origin");
    check(close, "100000 random readings: it moves by less than 1 mm per axis");
  }
  {
    // The fixed sensor pose (D11).
    check(mm(one(1.0f, 0.0f, SensorPose{1.0, 2.0, 0.0}), 2000, 2000), "pose (1, 2, 0)");
    check(mm(one(1.0f, 0.0f, SensorPose{0.0, 0.0, 1.5707963267948966}), 0, 1000), "yaw pi/2");
    check(mm(one(1.0f, 0.0f, SensorPose{0.0, 0.0, -1.5707963267948966}), 0, -1000), "yaw -pi/2");
    check(mm(one(1.0f, 0.0f, SensorPose{0.0, 0.0, 3.141592653589793}), -1000, 0),
      "yaw pi: a sensor mounted backwards sees ahead what is behind");
    check(mm(one(0.2f, 0.0f, SensorPose{-0.5, 0.0, 0.0}), -299, 0),
      "-0.5 + 0.2000000030 = -0.2999999970 m: truncated toward zero to -299 mm");
    check(mm(one(0.2f, 0.0f, SensorPose{0.5, -1.0, 0.0}), 700, -1000), "0.5 + 0.2000000030: 700 mm");
    check(mm(one(1.0f, 0.0f, SensorPose{999.5, 0.0, 0.0}), 1000000, 0), "pose pushes beyond the limit");
    check(mm(one(kNaNF, 0.0f, SensorPose{1.0, 0.0, 0.0}), 1000000, 1000000), "NaN with a pose");
    {
      // inf through the rotation gives inf or NaN; either way a saturated
      // coordinate, which lies in no zone.
      const Rec r = one(kInfF, 3.1415927f, SensorPose{1.0, 0.0, 0.5});
      check(std::llabs(r.x_mm) == 1000000 && std::llabs(r.y_mm) == 1000000,
        "infinite reading with a pose: both coordinates saturated");
    }
    const SensorPose zero{};
    const auto a = encode_scan(0.3f, 0.01f, {1.0f, 2.0f, 3.0f}, zero);
    const auto b = encode_scan(0.3f, 0.01f, {1.0f, 2.0f, 3.0f}, SensorPose{0.0, 0.0, 0.0});
    check(a == b, "a zero pose changes nothing");
  }

  // ---- the `points` string ------------------------------------------------
  {
    std::vector<std::array<float, 2>> v;
    std::string error;
    check(parse_points("[[0.3, 0.3], [0.3, -0.3], [0.0, -0.3], [0.0, 0.3]]", v, error) &&
      v.size() == 4 && error.empty(), "upstream's example box");
    check(v[1][0] == 0.3f && v[1][1] == -0.3f && v[2][0] == 0.0f, "values are floats, in order");
    check(parse_points("[[1,2],[3,4],[-5,6],[7,-8]]", v, error) && v.size() == 4 && v[2][0] == -5.0f &&
      v[3][1] == -8.0f, "no spaces");
    check(parse_points(" [\t[1e3, -2.5E-1] ,[0,0]]", v, error) && v.size() == 2 && v[0][0] == 1000.0f &&
      v[0][1] == -0.25f, "exponents, tabs and stray separators");
    check(parse_points("", v, error) && v.empty(), "empty string: no vertices (the core refuses it)");
    check(parse_points("[]", v, error) && v.empty(), "empty list");
    check(parse_points("[[1, 2], [3, 4], [5, 6]]", v, error) && v.size() == 3,
      "three vertices parse; refusing them is the core's decision");
    check(!parse_points("[[1, 2], [3, 4], [5]]", v, error) && v.empty() && !error.empty(), "a single number");
    check(!parse_points("[[1, 2, 3], [3, 4]]", v, error), "a triple");
    check(!parse_points("[[1, 2], []]", v, error), "an empty point");
    check(!parse_points("[[1, 2], [3, 4]", v, error) && error == "Unterminated vector string.",
      "unterminated");
    check(!parse_points("[[1, 2]]]", v, error) && error == "More close ] than open [", "extra ]");
    check(!parse_points("[[[1, 2]]]", v, error) && error == "Array depth greater than 2", "too deep");
    check(!parse_points("[1, 2]", v, error) && !parse_points("1", v, error), "numbers at depth 1 or 0");
    check(!parse_points("[[a, b]]", v, error), "not a number");
  }

  // ---- slowdown multiplication (T1) --------------------------------------
  check(scale(0.5, bits_of(0.5)) == 0.25 && scale(-0.5, bits_of(0.5)) == -0.25, "scale by 0.5");
  check(scale(0.3, bits_of(0.7)) == 0.3 * 0.7, "scale is the IEEE product");
  check(scale(1.25, bits_of(0.0)) == 0.0 && bits_of(scale(-1.25, bits_of(0.0))) == bits_of(-0.0),
    "ratio 0 gives zero, with the sign of the component");
  {
    std::mt19937_64 gen(3);
    std::uniform_real_distribution<double> ratio(0.0, 1.0);
    std::uniform_real_distribution<double> exponent(-300.0, 300.0);
    bool bounded = true, sign = true;
    for (int n = 0; n < 200000; ++n) {
      const double r = n % 100 == 0 ? std::nextafter(1.0, 0.0) : ratio(gen);
      const double x = (n % 2 ? 1.0 : -1.0) * std::pow(10.0, exponent(gen));
      const double y = scale(x, bits_of(r));
      bounded = std::fabs(y) <= std::fabs(x) && bounded;
      sign = (y == 0.0 || std::signbit(y) == std::signbit(x)) && sign;
    }
    check(bounded, "200000 random products with a ratio in [0, 1): no amplification");
    check(sign, "200000 random products: the sign does not flip");
  }

  std::printf("%d failure(s)\n", failures);
  return failures == 0 ? 0 : 1;
}
