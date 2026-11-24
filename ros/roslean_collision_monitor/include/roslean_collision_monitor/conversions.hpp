// The floating-point work of the collision monitor shim, and what it
// publishes for the core's answer to a command. Nothing else.
//
// Everything here is *trusted*: it is not covered by a theorem about the Lean
// core (docs/collision_monitor/SPEC.md, T1 to T3, T5 and T9). It is kept in this one
// header, free of ROS and of Lean, so that it can be read in one sitting and
// tested on its own (test/smoke_conversions.cpp).
//
//   T1  scale          the slowdown multiplication
//   T2  encode_scan    scan readings to points in the base frame, in metres:
//                      the angle, cos, sin, the product and the sensor pose
//   T3  parse_points   the text of the `points` parameter to floats
//   T5  apply_answer   what is published for the core's answer to a command
//   T9  clock_message_usable   whether a /clock message is one rclcpp accepts
//
// What is *not* here, because Lean does it in integer arithmetic
// (core/ROSleanCore/CollisionMonitorConvert.lean): metres to millimetres with
// its rounding direction and saturation, seconds to nanoseconds, the radius,
// and every refusal of a value. This file contains no cast from a
// floating-point value to an integer.
//
// No function here decides anything about a command or a zone. apply_answer
// carries out what the core decided; the one thing it settles itself is what
// to do with an answer it does not know, and that is a stop.

#ifndef ROSLEAN_COLLISION_MONITOR__CONVERSIONS_HPP_
#define ROSLEAN_COLLISION_MONITOR__CONVERSIONS_HPP_

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <sstream>
#include <string>
#include <vector>

namespace roslean_collision_monitor
{

// Bit patterns: bytes copied, no conversion.

inline uint64_t bits_of(double v)
{
  uint64_t b;
  std::memcpy(&b, &v, sizeof b);
  return b;
}

inline uint32_t bits_of(float v)
{
  uint32_t b;
  std::memcpy(&b, &v, sizeof b);
  return b;
}

inline double double_of_bits(uint64_t b)
{
  double v;
  std::memcpy(&v, &b, sizeof v);
  return v;
}

/// T1. One component of a slowed-down command: the IEEE product of the
/// component and the ratio whose bit pattern the core returned.
inline double scale(double component, uint64_t ratio_bits)
{
  return component * double_of_bits(ratio_bits);
}

/// What the shim does with the core's answer to a command.
enum class Answer
{
  Nothing,             ///< publish nothing
  Publish,             ///< publish the three components
  PublishUnknownKind   ///< publish zero: the kind is none of the core's
};

/// T5. The table of the SPEC, "Where the velocity is computed". `kind` and
/// `ratio_bits` are what `roslean_cm_out_kind` and `roslean_cm_out_ratio`
/// returned (0 silent, 1 pass, 2 slowdown, 3 stop by a zone, 4 stop by the
/// source); `cmd` holds linear.x, linear.y and angular.z of the command, and
/// `out` receives the same three components of the message to publish.
///
/// A kind outside 0 to 4 cannot be returned by the core (`kindOf`). Were a
/// core and a shim of different versions ever linked together, the answer
/// the shim does not understand is a stop, not silence and not the command:
/// `out` is zero and the caller publishes it (review CM3/CM4, finding 8).
inline Answer apply_answer(
  uint8_t kind, uint64_t ratio_bits, const std::array<double, 3> & cmd,
  std::array<double, 3> & out)
{
  out = {0.0, 0.0, 0.0};
  switch (kind) {
    case 0:
      return Answer::Nothing;
    case 1:
      out = cmd;
      return Answer::Publish;
    case 2:
      out = {scale(cmd[0], ratio_bits), scale(cmd[1], ratio_bits), scale(cmd[2], ratio_bits)};
      return Answer::Publish;
    case 3:
    case 4:
      return Answer::Publish;
    default:
      return Answer::PublishUnknownKind;
  }
}

/// T9 (D15). A message on /clock is usable iff its `sec` field is not
/// negative: rclcpp refuses to build a time from any other ("cannot store a
/// negative time point in rclcpp::Time"), throws, and keeps the time it had.
/// While the message received last is not usable the node hands commands to
/// the core as "without a clock reading", and the core answers with a stop.
/// The condition is rclcpp's (rclcpp/src/rclcpp/time.cpp, Jazzy); it is
/// repeated here because the node cannot ask rclcpp.
inline bool clock_message_usable(int32_t sec)
{
  return sec >= 0;
}

/// The fixed pose of the sensor in the base frame: metres and radians (D11).
struct SensorPose
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

/// Bytes per reading in the array handed to the core.
constexpr size_t kRecordSize = 20;

inline void put_le(uint8_t * out, uint64_t v, int bytes)
{
  for (int k = 0; k < bytes; ++k) {
    out[k] = static_cast<uint8_t>(v >> (8 * k));
  }
}

/// T2. One 20-byte record per reading, valid or not: the range's bit pattern
/// (4 bytes), then x and y of its point in the base frame in metres, as
/// binary64 bit patterns (8 bytes each), all little-endian. The point is
/// computed as upstream's Scan::getData does (R3): the angle accumulates in
/// single precision, range * cos(angle) and range * sin(angle) are
/// single-precision products, widened to double. The core converts the
/// coordinates to millimetres.
///
/// "As upstream" is about the operations. The bits of the result are
/// upstream's when this file is compiled with optimisation (the package's
/// default build type is Release) and with -ffp-contract=off, for a scan
/// without a pose; without optimisation the compiler calls another library
/// entry for cos and sin, and about 1 % of the coordinates differ by one unit
/// in the last place of a float (SPEC, "Scan event"; smoke_conversions).
inline std::vector<uint8_t> encode_scan(
  float angle_min, float angle_increment, const std::vector<float> & ranges,
  const SensorPose & pose)
{
  std::vector<uint8_t> out(kRecordSize * ranges.size());
  const bool shift = pose.x != 0.0 || pose.y != 0.0 || pose.yaw != 0.0;
  const double cos_yaw = std::cos(pose.yaw);
  const double sin_yaw = std::sin(pose.yaw);
  float angle = angle_min;
  for (size_t i = 0; i < ranges.size(); ++i) {
    const float xf = ranges[i] * std::cos(angle);
    const float yf = ranges[i] * std::sin(angle);
    double x = xf;
    double y = yf;
    if (shift) {
      const double xb = pose.x + cos_yaw * x - sin_yaw * y;
      const double yb = pose.y + sin_yaw * x + cos_yaw * y;
      x = xb;
      y = yb;
    }
    uint8_t * rec = out.data() + kRecordSize * i;
    put_le(rec, bits_of(ranges[i]), 4);
    put_le(rec + 4, bits_of(x), 8);
    put_le(rec + 12, bits_of(y), 8);
    angle += angle_increment;
  }
  return out;
}

/// Upstream's parser for a list of lists of numbers, "[[x, y], ...]"
/// (nav2_util/src/array_parser.cpp, parseVVF), statement for statement. On
/// success `error` is empty.
inline std::vector<std::vector<float>> parse_vvf(const std::string & input, std::string & error)
{
  std::vector<std::vector<float>> result;
  std::stringstream input_ss(input);
  int depth = 0;
  std::vector<float> current_vector;
  while (!!input_ss && !input_ss.eof()) {
    switch (input_ss.peek()) {
      case EOF:
        break;
      case '[':
        depth++;
        if (depth > 2) {
          error = "Array depth greater than 2";
          return result;
        }
        input_ss.get();
        current_vector.clear();
        break;
      case ']':
        depth--;
        if (depth < 0) {
          error = "More close ] than open [";
          return result;
        }
        input_ss.get();
        if (depth == 1) {
          result.push_back(current_vector);
        }
        break;
      case ',':
      case ' ':
      case '\t':
        input_ss.get();
        break;
      default:  // All other characters should be part of the numbers.
        if (depth != 2) {
          std::stringstream err_ss;
          err_ss << "Numbers at depth other than 2. Char was '" << char(input_ss.peek()) << "'.";
          error = err_ss.str();
          return result;
        }
        float value;
        input_ss >> value;
        if (!!input_ss) {
          current_vector.push_back(value);
        }
        break;
    }
  }
  if (depth != 0) {
    error = "Unterminated vector string.";
  } else {
    error = "";
  }
  return result;
}

/// T3. The vertices of the `points` parameter in metres, as the `float`
/// values upstream parses (R10). Every element must be a pair of numbers.
/// How many vertices a polygon needs, and which values are acceptable, is the
/// core's decision.
/// @return false, with a message in `error`, if the string is not accepted.
inline bool parse_points(
  const std::string & input, std::vector<std::array<float, 2>> & vertices, std::string & error)
{
  vertices.clear();
  const auto vvf = parse_vvf(input, error);
  if (!error.empty()) {
    return false;
  }
  for (const auto & v : vvf) {
    if (v.size() != 2) {
      error = "every point must be a pair of numbers; found one with " +
        std::to_string(v.size());
      vertices.clear();
      return false;
    }
    vertices.push_back({v[0], v[1]});
  }
  return true;
}

}  // namespace roslean_collision_monitor

#endif  // ROSLEAN_COLLISION_MONITOR__CONVERSIONS_HPP_
