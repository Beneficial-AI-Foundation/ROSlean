// Load generator and latency probe for a running twist_mux node (either
// implementation). It publishes TwistStamped on the mux's input topics at a
// fixed rate, listens on cmd_vel_out and reports, as one JSON object, how many
// messages came through and how long each took.
//
// Topics are in_00000.. and lock_00000.., matching the parameter file that
// scripts/run_bench.py writes. Priorities there rise with the index, so the
// highest active index is the one the mux forwards.
//
// Each message carries its send time (steady clock) in header.stamp and its
// input index in header.frame_id. The mux forwards messages unchanged, so the
// receiver reads both back.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cinttypes>
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
#include "std_msgs/msg/bool.hpp"

namespace
{

using Clock = std::chrono::steady_clock;

uint64_t mono_ns()
{
  return static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count());
}

std::string name(const char * prefix, uint32_t i)
{
  char buf[32];
  std::snprintf(buf, sizeof buf, "%s_%05u", prefix, i);
  return buf;
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
  const uint32_t inputs = static_cast<uint32_t>(num("inputs", 3));
  const uint32_t locks = static_cast<uint32_t>(num("locks", 0));
  // The `active` highest inputs publish; with --lowest 1 the lowest ones do.
  const uint32_t active = std::min(inputs, static_cast<uint32_t>(num("active", inputs)));
  const bool lowest = num("lowest", 0) != 0;
  const double rate = num("rate", 50);          // Hz per active input
  const double duration = num("duration", 10);  // seconds measured
  const double warmup = num("warmup", 1.5);     // seconds before measuring
  const double lock_hz = num("lock-hz", 5);     // each lock is published `false` at this rate
  const bool burst = kv.count("pattern") && kv["pattern"] == "burst";
  const double ready_timeout = num("ready-timeout", 120);

  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("load_probe");

  std::vector<uint32_t> act;
  for (uint32_t a = 0; a < active; ++a) {
    act.push_back(lowest ? a : inputs - active + a);
  }
  const uint32_t winner = act.back();

  std::vector<rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr> pubs;
  for (uint32_t a : act) {
    pubs.push_back(
      node->create_publisher<geometry_msgs::msg::TwistStamped>(
        name("in", a), rclcpp::SystemDefaultsQoS()));
  }
  std::vector<rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr> lock_pubs;
  for (uint32_t j = 0; j < locks; ++j) {
    lock_pubs.push_back(
      node->create_publisher<std_msgs::msg::Bool>(name("lock", j), rclcpp::SystemDefaultsQoS()));
  }

  std::mutex mutex;
  std::vector<uint64_t> latencies;
  latencies.reserve(static_cast<size_t>(duration * rate * active) + 1024);
  std::atomic<uint64_t> measure_from{UINT64_MAX}, measure_to{UINT64_MAX};
  uint64_t received_winner = 0, received_other = 0;

  auto sub = node->create_subscription<geometry_msgs::msg::TwistStamped>(
    "cmd_vel_out", rclcpp::QoS(rclcpp::KeepLast(1000)),
    [&](geometry_msgs::msg::TwistStamped::ConstSharedPtr msg) {
      const uint64_t now = mono_ns();
      const uint64_t sent = static_cast<uint64_t>(msg->header.stamp.sec) * 1000000000ull +
      msg->header.stamp.nanosec;
      if (sent < measure_from.load() || sent >= measure_to.load()) {
        return;
      }
      std::lock_guard<std::mutex> lock(mutex);
      if (std::strtoul(msg->header.frame_id.c_str(), nullptr, 10) == winner) {
        ++received_winner;
        latencies.push_back(now - sent);
      } else {
        ++received_other;
      }
    });

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  std::thread spinner([&] {exec.spin();});

  std::printf("PROBE_UP\n");
  std::fflush(stdout);

  // Ready: the mux subscribes to every topic we publish and publishes the output.
  const uint64_t wait_start = mono_ns();
  uint64_t ready_ns = 0;
  while (rclcpp::ok()) {
    bool ready = sub->get_publisher_count() >= 1;
    for (const auto & p : pubs) {ready = ready && p->get_subscription_count() >= 1;}
    for (const auto & p : lock_pubs) {ready = ready && p->get_subscription_count() >= 1;}
    if (ready) {
      ready_ns = mono_ns();
      break;
    }
    if (static_cast<double>(mono_ns() - wait_start) * 1e-9 > ready_timeout) {
      std::fprintf(stderr, "load_probe: the mux did not come up\n");
      exec.cancel();
      spinner.join();
      rclcpp::shutdown();
      return 1;
    }
    std::this_thread::sleep_for(std::chrono::microseconds(500));
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(500));  // let discovery settle

  const uint64_t period = static_cast<uint64_t>(1e9 / rate);
  const uint64_t start = mono_ns() + 1000000;
  const uint64_t from = start + static_cast<uint64_t>(warmup * 1e9);
  const uint64_t to = from + static_cast<uint64_t>(duration * 1e9);
  measure_from = from;
  measure_to = to;

  geometry_msgs::msg::TwistStamped msg;
  msg.twist.linear.x = 0.25;
  msg.twist.angular.z = -0.5;
  std_msgs::msg::Bool unlocked;
  std::vector<std::string> ids;
  for (uint32_t a : act) {ids.push_back(std::to_string(a));}

  uint64_t sent_winner = 0, sent_total = 0, max_lag = 0;
  const uint64_t lock_period = lock_hz > 0 ? static_cast<uint64_t>(1e9 / lock_hz) : UINT64_MAX;
  uint64_t next_lock = start;
  for (uint64_t k = 0;; ++k) {
    const uint64_t tick_time = start + k * period;
    if (tick_time >= to) {
      break;
    }
    for (size_t a = 0; a < pubs.size(); ++a) {
      // stagger: spread the inputs evenly over the period. burst: all at once,
      // the winner last, so that it queues behind everything else.
      const uint64_t target = tick_time + (burst ? 0 : a * period / pubs.size());
      std::this_thread::sleep_until(Clock::time_point(std::chrono::nanoseconds(target)));
      const uint64_t now = mono_ns();
      if (now >= next_lock) {
        for (auto & p : lock_pubs) {p->publish(unlocked);}
        next_lock = now + lock_period;
      }
      const uint64_t send = mono_ns();
      msg.header.stamp.sec = static_cast<int32_t>(send / 1000000000ull);
      msg.header.stamp.nanosec = static_cast<uint32_t>(send % 1000000000ull);
      msg.header.frame_id = ids[a];
      pubs[a]->publish(msg);
      if (send >= from && send < to) {
        ++sent_total;
        sent_winner += act[a] == winner;
        max_lag = std::max(max_lag, send - target);
      }
    }
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));  // stragglers
  exec.cancel();
  spinner.join();

  std::lock_guard<std::mutex> lock(mutex);
  std::sort(latencies.begin(), latencies.end());
  auto q = [&](double p) -> uint64_t {
      if (latencies.empty()) {return 0;}
      return latencies[std::min(latencies.size() - 1, static_cast<size_t>(p * latencies.size()))];
    };
  double mean = 0;
  for (uint64_t l : latencies) {mean += static_cast<double>(l);}
  mean = latencies.empty() ? 0 : mean / static_cast<double>(latencies.size());
  std::printf(
    "{\"ready_mono_ns\":%" PRIu64 ",\"sent_total\":%" PRIu64 ",\"sent_winner\":%" PRIu64
    ",\"received_winner\":%" PRIu64 ",\"received_other\":%" PRIu64
    ",\"lat_mean_ns\":%.0f,\"lat_p50_ns\":%" PRIu64 ",\"lat_p90_ns\":%" PRIu64
    ",\"lat_p99_ns\":%" PRIu64 ",\"lat_p999_ns\":%" PRIu64 ",\"lat_max_ns\":%" PRIu64
    ",\"generator_max_lag_ns\":%" PRIu64 "}\n",
    ready_ns, sent_total, sent_winner, received_winner, received_other, mean,
    q(0.5), q(0.9), q(0.99), q(0.999), latencies.empty() ? 0 : latencies.back(), max_lag);
  std::fflush(stdout);
  rclcpp::shutdown();
  return 0;
}
