// Load generator and latency probe for running collision monitor nodes
// (ROSlean's, upstream's, or both side by side on the same messages).
//
// It publishes LaserScan messages of `--readings` readings at `--scan-hz`
// and TwistStamped commands at `--cmd-hz` (the nodes must run with
// enable_stamped_cmd_vel, so that the header comes back), listens on each
// output topic given by `--outs a,b`, and prints one JSON line per
// `--window` seconds and one at the end: commands sent, answers received
// per node and the latency from `publish` to the subscriber's callback.
//
// Each command carries its send time (steady clock) in header.stamp and its
// number in header.frame_id; both nodes copy the header to their answer.
//
// The scene changes every `--phase` seconds: nothing near, an obstacle in
// the slowdown zone, an obstacle in the stop zone. The obstacle ranges are
// far from the zone boundaries, so both nodes must answer alike; answers to
// the same command that differ between the first two outputs are counted,
// and in which direction: `first_less_restrictive` is the number of commands
// for which the first output's linear.x is larger in magnitude than the
// second's (the first node lets through what the second slows or stops),
// `second_less_restrictive` the reverse. Two nodes that decide alike can
// still differ here when one of them has not yet taken in the scan that the
// other has: the scene changes every `--phase` seconds.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace
{

using Clock = std::chrono::steady_clock;

uint64_t mono_ns()
{
  return static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count());
}

struct Out
{
  std::string topic;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub;
  std::vector<uint64_t> lat;       // this window
  uint64_t received{0};            // total
  std::map<uint64_t, double> recent;  // command number -> linear.x, for the comparison
};

void print_stats(const char * name, std::vector<uint64_t> & v)
{
  std::sort(v.begin(), v.end());
  auto q = [&](double p) -> uint64_t {
      if (v.empty()) {return 0;}
      return v[std::min(v.size() - 1, static_cast<size_t>(p * static_cast<double>(v.size())))];
    };
  double mean = 0;
  for (uint64_t x : v) {mean += static_cast<double>(x);}
  mean = v.empty() ? 0 : mean / static_cast<double>(v.size());
  std::printf(
    "\"%s\":{\"n\":%zu,\"mean_ns\":%.0f,\"p50_ns\":%" PRIu64 ",\"p90_ns\":%" PRIu64
    ",\"p99_ns\":%" PRIu64 ",\"p999_ns\":%" PRIu64 ",\"max_ns\":%" PRIu64 "}",
    name, v.size(), mean, q(0.5), q(0.9), q(0.99), q(0.999), v.empty() ? 0 : v.back());
}

}  // namespace

int main(int argc, char ** argv)
{
  std::map<std::string, std::string> kv;
  for (int i = 1; i + 1 < argc && std::strncmp(argv[i], "--", 2) == 0 &&
    std::strcmp(argv[i], "--ros-args") != 0; i += 2)
  {
    kv[argv[i] + 2] = argv[i + 1];
  }
  auto num = [&](const char * k, double d) {
      return kv.count(k) ? std::strtod(kv[k].c_str(), nullptr) : d;
    };
  auto str = [&](const char * k, const char * d) {
      return kv.count(k) ? kv[k] : std::string(d);
    };
  const size_t readings = static_cast<size_t>(num("readings", 1080));
  const double scan_hz = num("scan-hz", 25);
  const double cmd_hz = num("cmd-hz", 50);
  const double duration = num("duration", 10);
  const double window = num("window", 0);
  const double phase = num("phase", 2.5);
  const double ready_timeout = num("ready-timeout", 120);

  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("cm_load");
  auto scan_pub = node->create_publisher<sensor_msgs::msg::LaserScan>(
    str("scan-topic", "scan"), rclcpp::SensorDataQoS());
  auto cmd_pub = node->create_publisher<geometry_msgs::msg::TwistStamped>(
    str("cmd-topic", "cmd_vel_in"), rclcpp::QoS(rclcpp::KeepLast(10)));

  std::mutex mutex;
  std::vector<std::unique_ptr<Out>> outs;
  uint64_t mismatches = 0, compared = 0, first_less = 0, second_less = 0;
  {
    std::string list = str("outs", "cmd_vel_out");
    size_t pos = 0;
    while (pos <= list.size()) {
      const size_t comma = std::min(list.find(',', pos), list.size());
      outs.push_back(std::make_unique<Out>());
      outs.back()->topic = list.substr(pos, comma - pos);
      pos = comma + 1;
    }
  }
  for (size_t k = 0; k < outs.size(); ++k) {
    Out * o = outs[k].get();
    o->sub = node->create_subscription<geometry_msgs::msg::TwistStamped>(
      o->topic, rclcpp::QoS(rclcpp::KeepLast(1000)),
      [&, o, k](geometry_msgs::msg::TwistStamped::ConstSharedPtr msg) {
        const uint64_t now = mono_ns();
        const uint64_t sent = static_cast<uint64_t>(msg->header.stamp.sec) * 1000000000ull +
        msg->header.stamp.nanosec;
        const uint64_t seq = std::strtoull(msg->header.frame_id.c_str(), nullptr, 10);
        std::lock_guard<std::mutex> lock(mutex);
        o->lat.push_back(now - sent);
        ++o->received;
        if (outs.size() >= 2 && k < 2) {
          auto & other = outs[1 - k]->recent;
          auto it = other.find(seq);
          if (it != other.end()) {
            ++compared;
            mismatches += it->second != msg->twist.linear.x;
            const double mine = std::fabs(msg->twist.linear.x), theirs = std::fabs(it->second);
            const double v0 = k == 0 ? mine : theirs, v1 = k == 0 ? theirs : mine;
            first_less += v0 > v1;
            second_less += v1 > v0;
            other.erase(it);
          } else {
            o->recent[seq] = msg->twist.linear.x;
            if (o->recent.size() > 4096) {o->recent.erase(o->recent.begin());}
          }
        }
      });
  }

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  std::thread spinner([&] {exec.spin();});

  const uint64_t wait_start = mono_ns();
  while (rclcpp::ok()) {
    bool ready = scan_pub->get_subscription_count() >= outs.size() &&
      cmd_pub->get_subscription_count() >= outs.size();
    for (const auto & o : outs) {ready = ready && o->sub->get_publisher_count() >= 1;}
    if (ready) {break;}
    if (static_cast<double>(mono_ns() - wait_start) * 1e-9 > ready_timeout) {
      std::fprintf(stderr, "cm_load: the nodes did not come up\n");
      exec.cancel();
      spinner.join();
      rclcpp::shutdown();
      return 1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // Three scenes. Zones of the benchmark configuration: stop within 0.3 m
  // ahead, slowdown within 0.4 m (config written by run_cm_bench.py).
  std::vector<sensor_msgs::msg::LaserScan> scenes(3);
  for (size_t s = 0; s < scenes.size(); ++s) {
    auto & m = scenes[s];
    m.header.frame_id = str("frame", "base_footprint");
    m.angle_min = -3.14159f;
    m.angle_increment = 6.28318f / static_cast<float>(readings);
    m.angle_max = 3.14159f;
    m.range_min = 0.05f;
    m.range_max = 30.0f;
    m.ranges.resize(readings);
    for (size_t i = 0; i < readings; ++i) {
      float r = 3.5f + 0.001f * static_cast<float>(i % 50);
      const bool ahead = i > readings * 15 / 32 && i < readings * 17 / 32;
      const bool behind = i < readings / 32;
      if (s == 1 && behind) {r = 0.35f;}   // slowdown zone only
      if (s == 2 && ahead) {r = 0.2f;}     // stop zone
      if (i % 97 == 0) {r = std::numeric_limits<float>::infinity();}
      m.ranges[i] = r;
    }
  }

  geometry_msgs::msg::TwistStamped cmd;
  cmd.twist.linear.x = 0.5;
  cmd.twist.angular.z = 0.25;
  const uint64_t start = mono_ns() + 1000000;
  const uint64_t end = start + static_cast<uint64_t>(duration * 1e9);
  const uint64_t scan_period = static_cast<uint64_t>(1e9 / scan_hz);
  const uint64_t cmd_period = static_cast<uint64_t>(1e9 / cmd_hz);
  uint64_t next_scan = start, next_cmd = start + cmd_period / 2;
  uint64_t next_window = window > 0 ? start + static_cast<uint64_t>(window * 1e9) : UINT64_MAX;
  uint64_t sent_cmd = 0, sent_scan = 0, win_cmd = 0;

  auto report = [&](const char * kind, uint64_t t, uint64_t cmds) {
      std::lock_guard<std::mutex> lock(mutex);
      std::printf(
        "{\"%s\":%.1f,\"commands\":%" PRIu64 ",\"scans_total\":%" PRIu64
        ",\"compared\":%" PRIu64 ",\"mismatches\":%" PRIu64
        ",\"first_less_restrictive\":%" PRIu64 ",\"second_less_restrictive\":%" PRIu64,
        kind, static_cast<double>(t - start) * 1e-9, cmds, sent_scan, compared, mismatches,
        first_less, second_less);
      for (auto & o : outs) {
        std::printf(",");
        print_stats(o->topic.c_str(), o->lat);
        o->lat.clear();
      }
      std::printf("}\n");
      std::fflush(stdout);
    };

  while (rclcpp::ok()) {
    const uint64_t target = std::min(next_scan, next_cmd);
    if (target >= end) {break;}
    std::this_thread::sleep_until(Clock::time_point(std::chrono::nanoseconds(target)));
    if (next_scan <= next_cmd) {
      const size_t scene =
        static_cast<size_t>(static_cast<double>(target - start) * 1e-9 / phase) % scenes.size();
      auto & m = scenes[scene];
      m.header.stamp = node->now();
      scan_pub->publish(m);
      ++sent_scan;
      next_scan += scan_period;
    } else {
      const uint64_t send = mono_ns();
      cmd.header.stamp.sec = static_cast<int32_t>(send / 1000000000ull);
      cmd.header.stamp.nanosec = static_cast<uint32_t>(send % 1000000000ull);
      cmd.header.frame_id = std::to_string(sent_cmd);
      cmd_pub->publish(cmd);
      ++sent_cmd;
      ++win_cmd;
      next_cmd += cmd_period;
    }
    if (mono_ns() >= next_window) {
      report("window_end_s", next_window, win_cmd);
      win_cmd = 0;
      next_window += static_cast<uint64_t>(window * 1e9);
    }
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  exec.cancel();
  spinner.join();
  if (window <= 0) {report("duration_s", end, sent_cmd);}
  {
    std::lock_guard<std::mutex> lock(mutex);
    std::printf(
      "{\"done\":true,\"sent_commands\":%" PRIu64 ",\"sent_scans\":%" PRIu64
      ",\"compared\":%" PRIu64 ",\"mismatches\":%" PRIu64
      ",\"first_less_restrictive\":%" PRIu64 ",\"second_less_restrictive\":%" PRIu64,
      sent_cmd, sent_scan, compared, mismatches, first_less, second_less);
    for (auto & o : outs) {
      std::printf(",\"received_%s\":%" PRIu64, o->topic.c_str(), o->received);
    }
    std::printf("}\n");
  }
  std::fflush(stdout);
  rclcpp::shutdown();
  return 0;
}
