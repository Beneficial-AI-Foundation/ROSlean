// Unit tests of the trusted work of the shim (conversions.hpp: the
// floating-point code, and what is published for the core's answer),
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
#include "roslean_collision_monitor/scan_records.hpp"
#include "roslean_core/collision_monitor.hpp"

// test/upstream_points.cpp: upstream's computation of the points of a scan,
// in a translation unit of its own that is always compiled with -O2.
std::vector<std::array<double, 2>> upstream_points(
  float angle_min, float angle_increment, const std::vector<float> & ranges);

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

// A single reading at the given angle, in the scan frame.
Rec one(float range, float angle)
{
  return rec(encode_scan(angle, 0.0f, {range}), 0);
}

bool mm(const Rec & r, int64_t x, int64_t y) {return r.x_mm == x && r.y_mm == y;}

// The same double, bit for bit; two NaNs count as the same.
bool same_bits(double a, double b)
{
  return bits_of(a) == bits_of(b) || (std::isnan(a) && std::isnan(b));
}

// The number of coordinates of the encoded scan that are not, bit for bit,
// those of the reference points.
size_t differing(const std::vector<uint8_t> & b, const std::vector<std::array<double, 2>> & ref)
{
  if (b.size() != 20 * ref.size()) {
    return 2 * ref.size() + 1;
  }
  size_t n = 0;
  for (size_t i = 0; i < ref.size(); ++i) {
    n += !same_bits(rec(b, i).x, ref[i][0]);
    n += !same_bits(rec(b, i).y, ref[i][1]);
  }
  return n;
}

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
  check(encode_scan(0.0f, 0.1f, {}).empty(), "a scan of zero length gives no bytes");
  {
    const auto b = encode_scan(0.0f, 0.0f, {1.0f});
    const std::vector<uint8_t> expect = {
      0x00, 0x00, 0x80, 0x3F,                           // 1.0f
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x3F,   // x = 1.0
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // y = 0.0
    check(b == expect, "range 1 at angle 0: 20 bytes, little-endian");
    const auto n = encode_scan(3.1415927f, 0.0f, {1.0f});
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
    const auto b = encode_scan(0.0f, 0.0f, {kNaNF, kInfF, -kInfF, -1.0f, 0.0f});
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
    // The points of a scan in the base frame are, bit for bit, the points an
    // optimised build of upstream's loop computes on this machine (R3; review
    // CM3/CM4, finding 5). The reference is upstream_points.cpp, which shares
    // no code with encode_scan and is always compiled with -O2; encode_scan
    // is compiled here with the flags of the node. On macOS arm64 this fails
    // when the package is built without optimisation.
    std::mt19937_64 gen(11);
    std::uniform_real_distribution<float> range(0.05f, 12.0f), start(-3.2f, 0.0f),
    step(0.001f, 0.013f);
    size_t differ = 0, total = 0, differs_from_double = 0;
    for (int n = 0; n < 200; ++n) {
      std::vector<float> ranges(1000);
      for (auto & r : ranges) {r = range(gen);}
      ranges[10] = kNaNF;  // the angle advances for an invalid reading too
      ranges[20] = kInfF;
      const float angle_min = start(gen), inc = step(gen);
      const auto b = encode_scan(angle_min, inc, ranges);
      differ += differing(b, upstream_points(angle_min, inc, ranges));
      total += 2 * ranges.size();
      for (size_t i = 0; i < ranges.size(); ++i) {
        const double a = static_cast<double>(angle_min) + static_cast<double>(i) * inc;
        differs_from_double += rec(b, i).x != ranges[i] * std::cos(a);
      }
    }
    std::printf("      %zu of %zu coordinates differ from the reference\n", differ, total);
    check(differ == 0,
      "200 scans of 1000 readings: bit for bit the points of an optimised build of upstream's loop");
    check(differs_from_double > total / 4,
      "which is distinguishable from a double-precision computation of the angle");
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
    // The ranges of a scan for the core: four bytes per reading, the bit
    // pattern, for every reading.
    const auto b = encode_ranges({1.0f, kNaNF, -kInfF, 0.0f});
    check(b.size() == 16 && le_at(b, 0, 4) == 0x3F800000u && le_at(b, 4, 4) == bits_of(kNaNF) &&
      le_at(b, 8, 4) == 0xFF800000u && le_at(b, 12, 4) == 0u && encode_ranges({}).empty(),
      "encode_ranges: the bit pattern of every reading, little-endian");
  }
  {
    // T8: the polygon published for a circle zone, as upstream's.
    const auto c = circle_polygon(0.5);
    check(c.size() == 16 && c[0][0] == 0.5f && c[0][1] == 0.0f &&
      c[4][1] == 0.5f && std::fabs(c[4][0]) < 1e-7f && c[8][0] == -0.5f,
      "circle_polygon: 16 points on the radius, starting at angle 0");
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

  // ---- what is published for the core's answer (T5) -----------------------
  {
    // The table of the SPEC, "Where the velocity is computed", bit for bit.
    using Vec = std::array<double, 3>;
    const Vec cmd = {-0.0, 0.25, -1.5};
    const uint64_t half = bits_of(0.5);
    const auto same = [](const Vec & a, const Vec & b) {
        return bits_of(a[0]) == bits_of(b[0]) && bits_of(a[1]) == bits_of(b[1]) &&
               bits_of(a[2]) == bits_of(b[2]);
      };
    const Vec zero = {0.0, 0.0, 0.0};
    const Vec junk = {9.0, 9.0, 9.0};  // what `out` holds before each call
    Vec out = junk;
    check(apply_answer(0, half, cmd, out) == Answer::Nothing, "kind 0: nothing is published");
    out = junk;
    check(apply_answer(1, half, cmd, out) == Answer::Publish && same(out, cmd),
      "kind 1: the command, bit for bit");
    out = junk;
    check(apply_answer(2, half, cmd, out) == Answer::Publish && same(out, Vec{-0.0, 0.125, -0.75}),
      "kind 2: the three products");
    bool stops = true;
    for (int kind : {3, 4}) {
      out = junk;
      stops = apply_answer(static_cast<uint8_t>(kind), half, cmd, out) == Answer::Publish &&
        same(out, zero) && stops;
    }
    check(stops, "kinds 3 and 4: zero is published");
    // Review CM3/CM4, finding 8: a kind the shim does not know is answered
    // with zero velocity, not with silence and not with the command.
    out = junk;
    check(apply_answer(5, half, cmd, out) == Answer::Nothing,
      "kind 5 (processed, velocity withheld): no velocity is published");
    bool unknown = true;
    for (int kind = 6; kind <= 255; ++kind) {
      out = junk;
      unknown = apply_answer(static_cast<uint8_t>(kind), half, cmd, out) ==
        Answer::PublishUnknownKind && same(out, zero) && unknown;
    }
    check(unknown, "kinds 6 to 255, which the core cannot return: zero is published");
  }
  {
    // T9 (D15): a /clock message is usable iff rclcpp accepts its time.
    using roslean_collision_monitor::clock_message_usable;
    check(
      clock_message_usable(0) && clock_message_usable(1) &&
      clock_message_usable(std::numeric_limits<int32_t>::max()),
      "T9: a /clock message with sec >= 0 is usable");
    check(
      !clock_message_usable(-1) && !clock_message_usable(-5) &&
      !clock_message_usable(std::numeric_limits<int32_t>::min()),
      "T9: a /clock message with sec < 0 is not");
  }

  std::printf("%d failure(s)\n", failures);
  return failures == 0 ? 0 : 1;
}
