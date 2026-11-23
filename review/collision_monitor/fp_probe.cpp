// CM3/CM4 review: how the shim's trusted floating-point code (conversions.hpp,
// T1 and T2) depends on the compiler flags and on the machine.
//
// Writes, for a fixed pseudo-random set of scans and commands, the bytes that
// encode_scan produces without and with a sensor pose, and the products that
// scale computes. fp_probe.sh compiles this file several ways and compares
// the outputs.
//
//   fp_probe <output file>

#include <cstdint>
#include <cstdio>
#include <vector>

#include "roslean_collision_monitor/conversions.hpp"

using namespace roslean_collision_monitor;

static uint64_t state = 0x9E3779B97F4A7C15ull;
static uint64_t next()
{
  state ^= state << 13; state ^= state >> 7; state ^= state << 17;
  return state;
}
static double unit() {return static_cast<double>(next() >> 11) / 9007199254740992.0;}

int main(int argc, char ** argv)
{
  if (argc != 2) {return 2;}
  FILE * f = std::fopen(argv[1], "wb");
  if (!f) {return 2;}
  // Section 1: 200 scans of 500 readings, no pose. Section 2: the same scans
  // with a pose. Section 3: 100000 slowdown products.
  std::vector<std::vector<float>> scans;
  std::vector<float> mins, incs;
  for (int s = 0; s < 200; ++s) {
    std::vector<float> r(500);
    for (auto & v : r) {v = static_cast<float>(0.05 + 12.0 * unit());}
    scans.push_back(r);
    mins.push_back(static_cast<float>(-3.14159 + unit()));
    incs.push_back(static_cast<float>(0.001 + 0.012 * unit()));
  }
  for (int pass = 0; pass < 2; ++pass) {
    for (size_t s = 0; s < scans.size(); ++s) {
      SensorPose pose;
      if (pass == 1) {pose = SensorPose{0.1 + unit(), -0.3 + unit(), 6.3 * unit() - 3.15};}
      const auto bytes = encode_scan(mins[s], incs[s], scans[s], pose);
      std::fwrite(bytes.data(), 1, bytes.size(), f);
    }
  }
  for (int k = 0; k < 100000; ++k) {
    const double x = (unit() - 0.5) * 4.0;
    const double r = unit();
    const uint64_t b = bits_of(scale(x, bits_of(r)));
    std::fwrite(&b, 8, 1, f);
  }
  std::fclose(f);
  // Not written to the file: arbitrary bit patterns and arbitrary text, for a
  // build with -fsanitize=undefined,address (fp_probe.sh, last step).
  size_t total = 0;
  for (int k = 0; k < 20000; ++k) {
    std::vector<float> r(1 + next() % 40);
    for (auto & v : r) {
      const uint32_t b = static_cast<uint32_t>(next());
      std::memcpy(&v, &b, 4);
    }
    float a, i;
    const uint32_t ab = static_cast<uint32_t>(next()), ib = static_cast<uint32_t>(next());
    std::memcpy(&a, &ab, 4);
    std::memcpy(&i, &ib, 4);
    const SensorPose pose{k % 2 ? 1e300 * unit() : 0.0, -1e300 * unit(), 1e308 * unit()};
    total += encode_scan(a, i, r, pose).size();
    const uint64_t xb = next(), rb = next();
    total += bits_of(scale(double_of_bits(xb), rb)) & 1;
  }
  const char alphabet[] = "[[]],, \t-+.0123456789eEnaif x";
  for (int k = 0; k < 200000; ++k) {
    std::string text;
    for (size_t n = next() % 40; n > 0; --n) {text += alphabet[next() % (sizeof alphabet - 1)];}
    std::vector<std::array<float, 2>> v;
    std::string error;
    total += parse_points(text, v, error) ? v.size() : 0;
  }
  std::printf("arbitrary inputs done (%zu)\n", total);
  return 0;
}
