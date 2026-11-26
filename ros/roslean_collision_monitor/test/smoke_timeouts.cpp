// The timeouts the Lean core computes, against rclcpp::Duration.
//
// Upstream converts `source_timeout` and `stop_pub_timeout` with
// rclcpp::Duration::from_seconds, and takes as default of a source's own
// timeout the node's timeout converted to seconds and back
// (Source::getCommonParameters; docs/collision_monitor/SPEC.md, R7 and R17).
// The core reproduces both in integer arithmetic. This test compares it with
// the library itself, so that the node's timeouts are upstream's to the
// nanosecond.

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <optional>
#include <random>
#include <string>

#include "rclcpp/duration.hpp"
#include "roslean_core/collision_monitor.hpp"

namespace
{

using roslean::CollisionMonitor;

int failures = 0;

void check(bool cond, const std::string & what)
{
  std::printf("%s  %s\n", cond ? "ok  " : "FAIL", what.c_str());
  if (!cond) {
    ++failures;
  }
}

int64_t upstream_ns(double seconds)
{
  return rclcpp::Duration::from_seconds(seconds).nanoseconds();
}

// The default of `<source>.source_timeout`, as upstream arrives at it.
int64_t upstream_default_ns(double node_seconds)
{
  const rclcpp::Duration node = rclcpp::Duration::from_seconds(node_seconds);
  return rclcpp::Duration::from_seconds(node.seconds()).nanoseconds();
}

}  // namespace

int main()
{
  check(upstream_ns(2.094) == 2093999999 && CollisionMonitor::timeout_ns(2.094) == 2093999999,
    "2.094 s is 2,093,999,999 ns in rclcpp and in the core");
  check(upstream_default_ns(2.094) == 2093999998 &&
    CollisionMonitor::source_timeout_ns(2.094, std::nullopt) == 2093999998,
    "as the default of the source's timeout it is 2,093,999,998 ns in both");

  bool same = true, same_default = true;
  int lost = 0;
  for (int k = 0; k <= 30000; ++k) {
    const double s = k / 1000.0;
    same = CollisionMonitor::timeout_ns(s) == upstream_ns(s) && same;
    same_default =
      CollisionMonitor::source_timeout_ns(s, std::nullopt) == upstream_default_ns(s) && same_default;
    lost += upstream_default_ns(s) != upstream_ns(s);
  }
  check(same, "every k/1000 s up to 30 s: equal to Duration::from_seconds");
  check(same_default, "every k/1000 s up to 30 s: the source's default equals upstream's");
  std::printf("      the default loses 1 ns for %d of 30001 values\n", lost);

  std::mt19937_64 gen(5);
  std::uniform_real_distribution<double> exponent(-12.0, 9.9);
  same = same_default = true;
  for (int n = 0; n < 200000; ++n) {
    const double s = std::pow(10.0, exponent(gen));
    same = CollisionMonitor::timeout_ns(s) == upstream_ns(s) && same;
    same_default =
      CollisionMonitor::source_timeout_ns(s, std::nullopt) == upstream_default_ns(s) && same_default;
  }
  check(same, "200000 random timeouts from 1e-12 s to 8e9 s: equal to Duration::from_seconds");
  check(same_default, "200000 random timeouts: the source's default equals upstream's");

  std::printf("%d failure(s)\n", failures);
  return failures == 0 ? 0 : 1;
}
