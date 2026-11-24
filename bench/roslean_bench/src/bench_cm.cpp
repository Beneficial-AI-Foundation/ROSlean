// Per-call cost of the collision monitor's two callbacks, in process:
//
//   Lean      the two callback bodies of ros/roslean_collision_monitor
//             (collision_monitor_node.cpp, on_scan and on_cmd): clock read,
//             encode_scan, the Lean core, the slowdown product, publish.
//   upstream  nav2_collision_monitor's own library (the version in the pixi
//             environment), configured and activated in this process, its
//             scan callback (Scan::dataCallback) and its command callback
//             (CollisionMonitor::cmdVelInCallbackUnstamped) called directly.
//
// Both get the same LaserScan and Twist messages. Upstream stores a scan and
// converts it to points when a command arrives; the Lean node converts when
// the scan arrives. So the two paths are reported separately and as a sum.
//
// Output: one JSON object per run on stdout.
//
//   bench_cm --readings 1080 --iters 20000 --zones polygons --scan clear
//   bench_cm --which lean --iters 1000000 --soak 50000    memory every 50,000 iterations

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <memory>
#include <optional>
#include <random>
#include <string>
#include <vector>

#include <unistd.h>

#include "geometry_msgs/msg/twist.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "nav2_collision_monitor/collision_monitor_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "roslean_collision_monitor/conversions.hpp"
#include "roslean_collision_monitor/scan_records.hpp"
#include "roslean_core/collision_monitor.hpp"
#include "roslean_core/scan_points_monitor.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace
{

using Clock = std::chrono::steady_clock;
using CollisionMonitor = roslean::ScanPointsMonitor;

uint64_t mono_ns()
{
  return static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count());
}

// Scan::dataCallback is protected. A pointer to it, named through a derived
// class, has type void (Scan::*)(...), and may be applied to any Scan.
struct ScanAccess : nav2_collision_monitor::Scan
{
  static auto callback() {return &ScanAccess::dataCallback;}
};

class Upstream : public nav2_collision_monitor::CollisionMonitor
{
public:
  using CollisionMonitor::CollisionMonitor;
  using CollisionMonitor::cmdVelInCallbackUnstamped;

  void scan(sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
  {
    auto & s = static_cast<nav2_collision_monitor::Scan &>(*sources_.at(0));
    (s.*ScanAccess::callback())(msg);
  }
  int last_action() const {return static_cast<int>(robot_action_prev_.action_type);}
};

// The Lean node's callback bodies, statement for statement.
class Lean : public rclcpp::Node
{
public:
  explicit Lean(const std::string & zones)
  : Node("roslean_cm_bench")
  {
    CollisionMonitor::Builder b;
    b.set_stop_pub_timeout(1.0);
    b.set_source(true, 2.0, std::nullopt);
    if (zones == "circles") {
      b.add_circle(0.3, CollisionMonitor::ZoneAction::Stop, 0.5, 4, true);
      b.add_circle(0.6, CollisionMonitor::ZoneAction::Slowdown, 0.3, 4, true);
    } else {
      b.add_polygon(CollisionMonitor::ZoneAction::Stop, 0.5, 4, true);
      for (auto v : {std::pair{0.3f, 0.3f}, {0.3f, -0.3f}, {0.0f, -0.3f}, {0.0f, 0.3f}}) {
        b.add_vertex(v.first, v.second);
      }
      b.add_polygon(CollisionMonitor::ZoneAction::Slowdown, 0.3, 4, true);
      for (auto v : {std::pair{0.4f, 0.4f}, {0.4f, -0.4f}, {-0.4f, -0.4f}, {-0.4f, 0.4f}}) {
        b.add_vertex(v.first, v.second);
      }
    }
    monitor_.emplace(b.build());
    if (!monitor_->ok()) {
      throw std::runtime_error("the Lean core refuses the benchmark configuration");
    }
    pub_ = create_publisher<geometry_msgs::msg::Twist>(
      "bench_lean_out", rclcpp::QoS(rclcpp::KeepLast(1)));
  }

  uint64_t now_ns()
  {
    return static_cast<uint64_t>(std::max<rcl_time_point_value_t>(now().nanoseconds(), 0));
  }

  void scan(const sensor_msgs::msg::LaserScan & msg)
  {
    using namespace roslean_collision_monitor;
    const uint64_t t = now_ns();
    const int64_t stamp_ns =
      static_cast<int64_t>(msg.header.stamp.sec) * 1000000000 + msg.header.stamp.nanosec;
    const bool usable = msg.header.frame_id == base_frame_id_;
    const auto beams = encode_scan(msg.angle_min, msg.angle_increment, msg.ranges);
    monitor_->on_scan(
      t, stamp_ns, usable, bits_of(msg.range_min), bits_of(msg.range_max), beams.data(),
      beams.size());
  }

  void cmd(const geometry_msgs::msg::Twist & c)
  {
    using namespace roslean_collision_monitor;
    const uint64_t t = now_ns();
    const auto d = monitor_->on_cmd(
      t, bits_of(c.linear.x), bits_of(c.linear.y), bits_of(c.linear.z),
      bits_of(c.angular.x), bits_of(c.angular.y), bits_of(c.angular.z));
    geometry_msgs::msg::Twist out;
    last_ = static_cast<int>(d.kind);
    switch (d.kind) {
      case CollisionMonitor::OutputKind::Pass:
        out.linear.x = c.linear.x;
        out.linear.y = c.linear.y;
        out.angular.z = c.angular.z;
        break;
      case CollisionMonitor::OutputKind::Slowdown:
        out.linear.x = scale(c.linear.x, d.ratio_bits);
        out.linear.y = scale(c.linear.y, d.ratio_bits);
        out.angular.z = scale(c.angular.z, d.ratio_bits);
        break;
      case CollisionMonitor::OutputKind::StopZone:
      case CollisionMonitor::OutputKind::StopSource:
        break;
      case CollisionMonitor::OutputKind::Silent:
        return;
    }
    pub_->publish(out);
  }
  int last() const {return last_;}

private:
  std::optional<CollisionMonitor> monitor_;
  std::string base_frame_id_{"base_footprint"};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  int last_{-1};
};

std::shared_ptr<Upstream> make_upstream(const std::string & zones)
{
  std::vector<rclcpp::Parameter> p = {
    {"base_frame_id", "base_footprint"}, {"odom_frame_id", "odom"},
    {"cmd_vel_in_topic", "bench_up_in"}, {"cmd_vel_out_topic", "bench_up_out"},
    {"state_topic", ""}, {"transform_tolerance", 0.0}, {"source_timeout", 2.0},
    {"stop_pub_timeout", 1.0}, {"base_shift_correction", false},
    {"bond_heartbeat_period", 0.0},
    {"observation_sources", std::vector<std::string>{"scan"}},
    {"scan.type", "scan"}, {"scan.topic", "bench_scan"},
    {"polygons", std::vector<std::string>{"Stop", "Slow"}},
    {"Stop.action_type", "stop"}, {"Stop.min_points", 4}, {"Stop.visualize", false},
    {"Slow.action_type", "slowdown"}, {"Slow.slowdown_ratio", 0.3}, {"Slow.min_points", 4},
    {"Slow.visualize", false},
  };
  if (zones == "circles") {
    p.insert(p.end(), {{"Stop.type", "circle"}, {"Stop.radius", 0.3},
        {"Slow.type", "circle"}, {"Slow.radius", 0.6}});
  } else {
    p.insert(p.end(), {{"Stop.type", "polygon"},
        {"Stop.points", "[[0.3, 0.3], [0.3, -0.3], [0.0, -0.3], [0.0, 0.3]]"},
        {"Slow.type", "polygon"},
        {"Slow.points", "[[0.4, 0.4], [0.4, -0.4], [-0.4, -0.4], [-0.4, 0.4]]"}});
  }
  auto node = std::make_shared<Upstream>(rclcpp::NodeOptions().parameter_overrides(p));
  if (node->configure().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE ||
    node->activate().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    throw std::runtime_error("upstream did not reach the active state");
  }
  return node;
}

struct Stats
{
  std::vector<uint64_t> v;
  void print(const char * name)
  {
    std::sort(v.begin(), v.end());
    auto q = [&](double p) {
        return v[std::min(v.size() - 1, static_cast<size_t>(p * static_cast<double>(v.size())))];
      };
    double mean = 0;
    for (uint64_t x : v) {mean += static_cast<double>(x);}
    mean /= static_cast<double>(v.size());
    std::printf(
      "\"%s\":{\"mean\":%.0f,\"min\":%" PRIu64 ",\"p50\":%" PRIu64 ",\"p90\":%" PRIu64
      ",\"p99\":%" PRIu64 ",\"p999\":%" PRIu64 ",\"max\":%" PRIu64 "}",
      name, mean, v.front(), q(0.5), q(0.9), q(0.99), q(0.999), v.back());
  }
};

long rss_kb()
{
  // Resident set size of this process, by `ps` (portable between macOS and Linux).
  char cmd[64];
  std::snprintf(cmd, sizeof cmd, "ps -o rss= -p %d", static_cast<int>(getpid()));
  FILE * f = popen(cmd, "r");
  long kb = -1;
  if (f) {
    if (std::fscanf(f, "%ld", &kb) != 1) {kb = -1;}
    pclose(f);
  }
  return kb;
}

}  // namespace

int main(int argc, char ** argv)
{
  std::map<std::string, std::string> kv;
  for (int i = 1; i + 1 < argc && std::strncmp(argv[i], "--", 2) == 0; i += 2) {
    kv[argv[i] + 2] = argv[i + 1];
  }
  auto num = [&](const char * k, double d) {
      return kv.count(k) ? std::strtod(kv[k].c_str(), nullptr) : d;
    };
  const size_t readings = static_cast<size_t>(num("readings", 1080));
  const size_t iters = static_cast<size_t>(num("iters", 20000));
  const size_t warmup = static_cast<size_t>(num("warmup", 2000));
  const std::string zones = kv.count("zones") ? kv["zones"] : "polygons";
  // clear: nothing within 3 m. near: one reading in twenty at 0.2 m (stop).
  // slow: one reading in twenty at 0.35 m behind the robot (slowdown only).
  const std::string scan_kind = kv.count("scan") ? kv["scan"] : "clear";
  // which: both | lean | upstream. soak: report memory every `soak` iterations.
  const std::string which = kv.count("which") ? kv["which"] : "both";
  const size_t soak = static_cast<size_t>(num("soak", 0));

  rclcpp::init(0, nullptr);
  std::shared_ptr<Lean> lean;
  std::shared_ptr<Upstream> up;
  if (which != "upstream") {lean = std::make_shared<Lean>(zones);}
  if (which != "lean") {up = make_upstream(zones);}

  // A pool of scans, so that the data is not the same array every time.
  std::mt19937 rng(12345);
  std::uniform_real_distribution<float> noise(-0.05f, 0.05f);
  std::vector<sensor_msgs::msg::LaserScan> pool(16);
  for (auto & m : pool) {
    m.header.frame_id = "base_footprint";
    m.angle_min = -3.14159f;
    m.angle_increment = 6.28318f / static_cast<float>(readings);
    m.angle_max = 3.14159f;
    m.range_min = 0.05f;
    m.range_max = 30.0f;
    m.ranges.resize(readings);
    for (size_t i = 0; i < readings; ++i) {
      float r = 3.5f + noise(rng);
      const bool ahead = i > readings * 15 / 32 && i < readings * 17 / 32;  // around angle 0
      const bool behind = i < readings / 32;                               // around -pi
      if (scan_kind == "near" && ahead && i % 2 == 0) {r = 0.2f + 0.1f * noise(rng);}
      if (scan_kind == "slow" && behind && i % 2 == 0) {r = 0.35f + 0.1f * noise(rng);}
      if (i % 97 == 0) {r = std::numeric_limits<float>::infinity();}  // no return
      m.ranges[i] = r;
    }
  }
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 0.5;
  cmd.angular.z = 0.25;

  Stats ls, lc, us, uc;
  int lean_kind = -1, up_action = -1;
  rclcpp::Clock ros_clock(RCL_SYSTEM_TIME);
  const long rss0 = rss_kb();
  if (soak) {std::printf("{\"soak\":[");}
  for (size_t k = 0; k < warmup + iters; ++k) {
    // A new message per event, as a subscription delivers one.
    auto scan = std::make_shared<sensor_msgs::msg::LaserScan>(pool[k % pool.size()]);
    scan->header.stamp = ros_clock.now();
    auto c = std::make_shared<geometry_msgs::msg::Twist>(cmd);
    // In a soak no samples are kept: the memory curve must be the node's, not this list's.
    const bool rec = k >= warmup && !soak;
    uint64_t t0, t1;
    if (lean) {
      t0 = mono_ns(); lean->scan(*scan); t1 = mono_ns();
      if (rec) {ls.v.push_back(t1 - t0);}
      t0 = mono_ns(); lean->cmd(*c); t1 = mono_ns();
      if (rec) {lc.v.push_back(t1 - t0);}
      lean_kind = lean->last();
    }
    if (up) {
      t0 = mono_ns(); up->scan(scan); t1 = mono_ns();
      if (rec) {us.v.push_back(t1 - t0);}
      t0 = mono_ns(); up->cmdVelInCallbackUnstamped(c); t1 = mono_ns();
      if (rec) {uc.v.push_back(t1 - t0);}
      up_action = up->last_action();
    }
    if (soak && k % soak == 0) {
      std::printf("%s[%zu,%ld]", k ? "," : "", 2 * k, rss_kb());
      std::fflush(stdout);
    }
  }
  if (soak) {std::printf("],");} else {std::printf("{");}
  std::printf(
    "\"readings\":%zu,\"zones\":\"%s\",\"scan\":\"%s\",\"iters\":%zu,"
    "\"lean_out_kind\":%d,\"upstream_action\":%d,\"rss_start_kb\":%ld,\"rss_end_kb\":%ld",
    readings, zones.c_str(), scan_kind.c_str(), iters, lean_kind, up_action, rss0, rss_kb());
  if (lean && !soak) {
    std::printf(","); ls.print("lean_scan_ns");
    std::printf(","); lc.print("lean_cmd_ns");
  }
  if (up && !soak) {
    std::printf(","); us.print("upstream_scan_ns");
    std::printf(","); uc.print("upstream_cmd_ns");
  }
  std::printf("}\n");
  std::fflush(stdout);
  up.reset();
  lean.reset();
  rclcpp::shutdown();
  return 0;
}
