// In-process benchmark of the twist_mux arbitration decision.
//
// One process measures one implementation on one workload and prints one JSON
// object. scripts/run_bench.py runs the matrix.
//
// Implementations (--impl):
//   lean      the Lean core through roslean::Mux, exactly as the node calls it
//   port      a plain C++ port of the model: integer nanoseconds, one array
//             scan. Not upstream's code; it shows what the algorithm costs
//             without any runtime around it, and it is the oracle that every
//             Lean decision is checked against.
//   upstream  upstream twist_mux's own sources (compiled unchanged), called
//             in process: TwistMux::hasPriorityStamped and the handle
//             callbacks.
//
// Clocks (--clock):
//   synthetic the event times are generated (lean and port only). The two
//             are run on the same events and must decide identically.
//   ros       times are read from the node clock, as the nodes do: once per
//             event for lean, once per handle for upstream.

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <memory>
#include <optional>
#include <random>
#include <string>
#include <vector>

#include <unistd.h>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "roslean_core/mux.hpp"
#include "std_msgs/msg/bool.hpp"
#include "twist_mux/topic_handle.hpp"
#include "twist_mux/twist_mux.hpp"

namespace
{

using Clock = std::chrono::steady_clock;

inline uint64_t tick()
{
  return static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count());
}

long rss_kb()
{
  std::ifstream f("/proc/self/status");
  std::string line;
  while (std::getline(f, line)) {
    if (line.rfind("VmRSS:", 0) == 0) {
      return std::strtol(line.c_str() + 6, nullptr, 10);
    }
  }
  return -1;
}

struct Options
{
  std::string impl = "lean";
  std::string clock = "synthetic";
  std::string what = "decide";       // ros clock only: decide | callback
  std::string timeouts = "live";     // none | live | expired | mixed | huge
  std::string prio = "asc";          // asc | desc | equal | random
  std::string speaker = "rr";        // first | last | rr | random
  std::string label;
  uint32_t inputs = 3;
  uint32_t locks = 1;
  uint64_t events = 1000000;
  double lock_rate = 0.0;            // fraction of events that are lock messages
  double lock_true = 0.3;            // fraction of lock messages that are `true`
  uint64_t t0 = 1800000000000000000ull;  // about the year 2027, in ns
  uint64_t mean_dt = 1000000;        // mean time between events, ns
  uint64_t seed = 1;
};

Options parse(int argc, char ** argv)
{
  Options o;
  std::map<std::string, std::string> kv;
  for (int i = 1; i + 1 < argc; i += 2) {
    if (std::strncmp(argv[i], "--", 2) != 0) {
      std::fprintf(stderr, "bad argument %s\n", argv[i]);
      std::exit(2);
    }
    kv[argv[i] + 2] = argv[i + 1];
  }
  auto s = [&](const char * k, std::string & v) {if (kv.count(k)) {v = kv[k]; kv.erase(k);}};
  auto u = [&](const char * k, uint64_t & v) {
      if (kv.count(k)) {v = std::strtoull(kv[k].c_str(), nullptr, 10); kv.erase(k);}
    };
  auto d = [&](const char * k, double & v) {
      if (kv.count(k)) {v = std::strtod(kv[k].c_str(), nullptr); kv.erase(k);}
    };
  uint64_t inputs = o.inputs, locks = o.locks;
  s("impl", o.impl); s("clock", o.clock); s("what", o.what); s("timeouts", o.timeouts);
  s("prio", o.prio); s("speaker", o.speaker); s("label", o.label);
  u("inputs", inputs); u("locks", locks); u("events", o.events); u("t0", o.t0);
  u("mean-dt", o.mean_dt); u("seed", o.seed);
  d("lock-rate", o.lock_rate); d("lock-true", o.lock_true);
  o.inputs = static_cast<uint32_t>(inputs);
  o.locks = static_cast<uint32_t>(locks);
  if (!kv.empty()) {
    std::fprintf(stderr, "unknown option --%s\n", kv.begin()->first.c_str());
    std::exit(2);
  }
  return o;
}

// ---------------------------------------------------------------- workload

struct HandleCfg
{
  int64_t priority;    // raw, may be outside 0..255
  int64_t timeout_ns;  // raw
};

struct Config
{
  std::vector<HandleCfg> inputs, locks;
};

struct Event
{
  uint64_t t;
  uint32_t index;
  bool is_lock;
  bool value;
};

int64_t pick_timeout(const Options & o, std::mt19937_64 & rng, uint32_t count)
{
  if (o.timeouts == "none") {return 0;}
  if (o.timeouts == "live") {return 1000000000000000ll;}  // 11 days: never reached
  if (o.timeouts == "expired") {return 1;}
  // Above 2^63 - t0: `stamp + timeout` no longer fits a machine word, which
  // Lean's Nat handles with a heap-allocated big number.
  if (o.timeouts == "huge") {return 9000000000000000000ll;}
  // mixed: disabled, short enough to expire between two visits, or long.
  switch (rng() % 3) {
    case 0: return 0;
    case 1: return static_cast<int64_t>(o.mean_dt * std::max<uint32_t>(count, 1) / 2 + 1);
    default: return 1000000000000000ll;
  }
}

Config make_config(const Options & o, std::mt19937_64 & rng)
{
  Config c;
  for (uint32_t i = 0; i < o.inputs; ++i) {
    int64_t p;
    const uint32_t span = std::max<uint32_t>(o.inputs - 1, 1);
    if (o.prio == "asc") {
      p = 1 + static_cast<int64_t>(i) * 254 / span;
    } else if (o.prio == "desc") {
      p = 255 - static_cast<int64_t>(i) * 254 / span;
    } else if (o.prio == "equal") {
      p = 100;
    } else {
      p = static_cast<int64_t>(rng() % 300) - 20;  // includes values to be clamped
    }
    c.inputs.push_back({p, pick_timeout(o, rng, o.inputs)});
  }
  for (uint32_t j = 0; j < o.locks; ++j) {
    // Below most inputs, so that a locked lock masks some inputs, not all.
    c.locks.push_back({static_cast<int64_t>(rng() % 128), pick_timeout(o, rng, o.inputs)});
  }
  return c;
}

std::vector<Event> make_events(const Options & o, std::mt19937_64 & rng)
{
  std::vector<Event> ev;
  ev.reserve(o.events);
  uint64_t t = o.t0;
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  uint32_t rr = 0;
  for (uint64_t k = 0; k < o.events; ++k) {
    t += rng() % (2 * o.mean_dt + 1);
    if (o.locks > 0 && unit(rng) < o.lock_rate) {
      ev.push_back({t, static_cast<uint32_t>(rng() % o.locks), true, unit(rng) < o.lock_true});
      continue;
    }
    uint32_t i;
    if (o.speaker == "first") {
      i = 0;
    } else if (o.speaker == "last") {
      i = o.inputs - 1;
    } else if (o.speaker == "rr") {
      i = rr++ % o.inputs;
    } else {
      i = static_cast<uint32_t>(rng() % o.inputs);
    }
    ev.push_back({t, i, false, false});
  }
  return ev;
}

// -------------------------------------------------------------------- port

// The model of core/ROSleanCore/Mux.lean in plain C++ (integer nanoseconds,
// R1-R12 of docs/SPEC.md). 128-bit sums, since Lean's are unbounded.
class PortMux
{
public:
  explicit PortMux(const Config & c)
  {
    for (const auto & h : c.inputs) {inputs_.push_back(make(h));}
    for (const auto & h : c.locks) {locks_.push_back(make(h));}
  }

  bool on_input(uint32_t i, uint64_t t)
  {
    if (i >= inputs_.size()) {return false;}
    inputs_[i].stamp = t;
    uint32_t lock_priority = 0;
    for (const auto & l : locks_) {
      if ((expired(l, t) || l.value) && lock_priority < l.priority) {
        lock_priority = l.priority;
      }
    }
    uint32_t best = 0;
    int64_t selected = -1;
    for (size_t k = 0; k < inputs_.size(); ++k) {
      const auto & h = inputs_[k];
      if (!expired(h, t) && lock_priority <= h.priority && best < h.priority) {
        best = h.priority;
        selected = static_cast<int64_t>(k);
      }
    }
    return selected == static_cast<int64_t>(i);
  }

  void on_lock(uint32_t j, bool value, uint64_t t)
  {
    if (j >= locks_.size()) {return;}
    locks_[j].stamp = t;
    locks_[j].value = value;
  }

private:
  struct H
  {
    uint32_t priority;
    uint64_t timeout;
    uint64_t stamp;
    bool value;
  };
  static H make(const HandleCfg & h)
  {
    return {static_cast<uint32_t>(std::clamp<int64_t>(h.priority, 0, 255)),
      static_cast<uint64_t>(std::max<int64_t>(h.timeout_ns, 0)), 0, false};
  }
  static bool expired(const H & h, uint64_t now)
  {
    return h.timeout > 0 &&
           static_cast<unsigned __int128>(h.stamp) + h.timeout < static_cast<unsigned __int128>(now);
  }
  std::vector<H> inputs_, locks_;
};

roslean::Mux make_lean(const Config & c)
{
  roslean::Mux::Builder b;
  for (const auto & h : c.inputs) {b.add_input(h.priority, h.timeout_ns);}
  for (const auto & h : c.locks) {b.add_lock(h.priority, h.timeout_ns);}
  return b.build();
}

// ---------------------------------------------------------------- upstream

// Upstream's node, with its handle lists made reachable. Nothing is changed.
class UpstreamMux : public twist_mux::TwistMux
{
public:
  using twist_mux::TwistMux::velocity_stamped_hs_;
  using twist_mux::TwistMux::lock_hs_;
};

std::string write_params(const Options & o, const Config & c)
{
  const std::string path = "/tmp/roslean_bench_" + std::to_string(getpid()) + ".yaml";
  std::ofstream f(path);
  f << "/**:\n  ros__parameters:\n    use_stamped: true\n";
  auto group = [&](const char * name, const char * prefix, const std::vector<HandleCfg> & hs) {
      if (hs.empty()) {return;}
      f << "    " << name << ":\n";
      for (size_t i = 0; i < hs.size(); ++i) {
        char id[32];
        std::snprintf(id, sizeof id, "%s_%05zu", prefix, i);  // ASCII order = index order
        char timeout[64];
        std::snprintf(timeout, sizeof timeout, "%.9f", hs[i].timeout_ns * 1e-9);
        f << "      " << id << ":\n        topic: bench/" << id << "\n        timeout: " <<
          timeout << "\n        priority: " << hs[i].priority << "\n";
      }
    };
  group("topics", "in", c.inputs);
  group("locks", "lock", c.locks);
  (void)o;
  return path;
}

// ------------------------------------------------------------------- stats

struct Stats
{
  double mean_ns = 0;
  uint64_t p50 = 0, p90 = 0, p99 = 0, p999 = 0, max = 0;
};

Stats summarise(std::vector<uint64_t> & v, double mean_ns)
{
  Stats s;
  s.mean_ns = mean_ns;
  if (v.empty()) {return s;}
  std::sort(v.begin(), v.end());
  auto q = [&](double p) {return v[std::min(v.size() - 1, static_cast<size_t>(p * v.size()))];};
  s.p50 = q(0.50); s.p90 = q(0.90); s.p99 = q(0.99); s.p999 = q(0.999); s.max = v.back();
  return s;
}

// Runs `fn(k)` for every event twice: once untimed per event (total time gives
// the mean, without the timer in the loop), once with a timer around each
// call (percentiles). `reset` restores the initial state in between.
template<typename Reset, typename Fn>
Stats measure(uint64_t n, Reset reset, Fn fn)
{
  const uint64_t warm = std::min<uint64_t>(n / 10, 20000);
  reset();
  for (uint64_t k = 0; k < warm; ++k) {fn(k);}
  const uint64_t a = tick();
  for (uint64_t k = warm; k < n; ++k) {fn(k);}
  const uint64_t b = tick();
  const double mean = static_cast<double>(b - a) / static_cast<double>(std::max<uint64_t>(n - warm, 1));

  reset();
  std::vector<uint64_t> samples;
  samples.reserve(n - warm);
  for (uint64_t k = 0; k < warm; ++k) {fn(k);}
  for (uint64_t k = warm; k < n; ++k) {
    const uint64_t x = tick();
    fn(k);
    samples.push_back(tick() - x);
  }
  return summarise(samples, mean);
}

// Results of the decisions are added here, so that no pass is dead code.
volatile uint64_t g_sink = 0;

uint64_t timer_overhead()
{
  std::vector<uint64_t> v;
  for (int i = 0; i < 20000; ++i) {
    const uint64_t x = tick();
    v.push_back(tick() - x);
  }
  std::sort(v.begin(), v.end());
  return v[v.size() / 2];
}

}  // namespace

int main(int argc, char ** argv)
{
  const Options o = parse(argc, argv);
  std::mt19937_64 rng(o.seed);
  const Config cfg = make_config(o, rng);
  const std::vector<Event> events = make_events(o, rng);
  const long rss_start = rss_kb();

  Stats st;
  uint64_t forwarded = 0, mismatches = 0;
  long rss_built = 0;
  double build_ms = 0;

  if (o.clock == "synthetic") {
    if (o.impl == "lean") {
      std::optional<roslean::Mux> mux;
      const uint64_t b0 = tick();
      mux.emplace(make_lean(cfg));
      build_ms = static_cast<double>(tick() - b0) * 1e-6;
      rss_built = rss_kb();
      std::vector<uint8_t> out(events.size(), 0);
      st = measure(
        events.size(),
        [&] {mux.emplace(make_lean(cfg));},
        [&](uint64_t k) {
          const Event & e = events[k];
          if (e.is_lock) {
            mux->on_lock(e.index, e.value, e.t);
          } else {
            out[k] = mux->on_input(e.index, e.t);
          }
        });
      // Every decision against the port, on the same events.
      PortMux port(cfg);
      for (size_t k = 0; k < events.size(); ++k) {
        const Event & e = events[k];
        bool expect = false;
        if (e.is_lock) {
          port.on_lock(e.index, e.value, e.t);
        } else {
          expect = port.on_input(e.index, e.t);
        }
        forwarded += out[k];
        mismatches += (out[k] != 0) != expect;
      }
    } else if (o.impl == "port") {
      std::optional<PortMux> mux;
      const uint64_t b0 = tick();
      mux.emplace(cfg);
      build_ms = static_cast<double>(tick() - b0) * 1e-6;
      rss_built = rss_kb();
      uint64_t count = 0;
      st = measure(
        events.size(),
        [&] {mux.emplace(cfg); g_sink = g_sink + count; count = 0;},
        [&](uint64_t k) {
          const Event & e = events[k];
          if (e.is_lock) {
            mux->on_lock(e.index, e.value, e.t);
          } else {
            count += mux->on_input(e.index, e.t);
          }
        });
      forwarded = count;
    } else {
      std::fprintf(stderr, "--clock synthetic supports --impl lean|port\n");
      return 2;
    }
  } else {
    // ROS clock: both sides as deployed, inside a real rclcpp process.
    const std::string params = write_params(o, cfg);
    const char * args[] = {"bench_core", "--ros-args", "--params-file", params.c_str(),
      "--log-level", "error"};
    rclcpp::init(6, args);
    // A matched subscriber, so that publishing does its real work.
    auto sink = std::make_shared<rclcpp::Node>(
      "bench_sink", rclcpp::NodeOptions().use_global_arguments(false));
    auto sink_sub = sink->create_subscription<geometry_msgs::msg::TwistStamped>(
      "cmd_vel_out", rclcpp::QoS(rclcpp::KeepLast(1)),
      [](geometry_msgs::msg::TwistStamped::ConstSharedPtr) {});
    auto msg = std::make_shared<const geometry_msgs::msg::TwistStamped>();
    auto lock_true = std::make_shared<std_msgs::msg::Bool>();
    lock_true->data = true;
    auto lock_false = std::make_shared<std_msgs::msg::Bool>();
    const bool callback = o.what == "callback";

    if (o.impl == "upstream") {
      const uint64_t b0 = tick();
      auto node = std::make_shared<UpstreamMux>();
      node->init();
      build_ms = static_cast<double>(tick() - b0) * 1e-6;
      rss_built = rss_kb();
      std::vector<twist_mux::VelocityStampedTopicHandle *> in;
      for (auto & h : *node->velocity_stamped_hs_) {in.push_back(&h);}
      std::vector<twist_mux::LockTopicHandle *> lk;
      for (auto & h : *node->lock_hs_) {lk.push_back(&h);}
      if (in.size() != o.inputs || lk.size() != o.locks) {
        std::fprintf(stderr, "upstream built %zu inputs, %zu locks\n", in.size(), lk.size());
        return 1;
      }
      uint64_t count = 0;
      st = measure(
        events.size(),
        [&] {
          // Give every handle a stamp and a value, as after one round of messages.
          for (auto * h : lk) {h->callback(lock_false);}
          for (auto * h : in) {h->callback(msg);}
          g_sink = g_sink + count;
          count = 0;
        },
        [&](uint64_t k) {
          const Event & e = events[k];
          if (e.is_lock) {
            lk[e.index]->callback(e.value ? lock_true : lock_false);
          } else if (callback) {
            in[e.index]->callback(msg);  // stamp, copy, decide, publish
          } else {
            count += node->hasPriorityStamped(*in[e.index]);  // decide only
          }
        });
      forwarded = count;
    } else if (o.impl == "lean") {
      const uint64_t b0 = tick();
      auto node = std::make_shared<rclcpp::Node>(
        "twist_mux", rclcpp::NodeOptions().use_global_arguments(false));
      auto pub = node->create_publisher<geometry_msgs::msg::TwistStamped>(
        "cmd_vel_out", rclcpp::QoS(rclcpp::KeepLast(1)));
      std::optional<roslean::Mux> mux;
      mux.emplace(make_lean(cfg));
      build_ms = static_cast<double>(tick() - b0) * 1e-6;
      rss_built = rss_kb();
      auto now_ns = [&] {
          return static_cast<uint64_t>(std::max<rcl_time_point_value_t>(node->now().nanoseconds(), 0));
        };
      uint64_t count = 0;
      st = measure(
        events.size(),
        [&] {
          mux.emplace(make_lean(cfg));
          for (uint32_t j = 0; j < o.locks; ++j) {mux->on_lock(j, false, now_ns());}
          for (uint32_t i = 0; i < o.inputs; ++i) {mux->on_input(i, now_ns());}
          g_sink = g_sink + count;
          count = 0;
        },
        [&](uint64_t k) {
          // The bodies of the two callbacks in twist_mux_node.cpp.
          const Event & e = events[k];
          if (e.is_lock) {
            mux->on_lock(e.index, e.value, now_ns());
          } else if (mux->on_input(e.index, now_ns())) {
            ++count;
            if (callback) {pub->publish(*msg);}
          }
        });
      forwarded = count;
    } else {
      std::fprintf(stderr, "--clock ros supports --impl lean|upstream\n");
      return 2;
    }
    rclcpp::shutdown();
    std::remove(params.c_str());
  }

  std::printf(
    "{\"label\":\"%s\",\"impl\":\"%s\",\"clock\":\"%s\",\"what\":\"%s\",\"inputs\":%u,\"locks\":%u,"
    "\"timeouts\":\"%s\",\"prio\":\"%s\",\"speaker\":\"%s\",\"lock_rate\":%g,\"events\":%" PRIu64 ","
    "\"mean_ns\":%.1f,\"p50_ns\":%" PRIu64 ",\"p90_ns\":%" PRIu64 ",\"p99_ns\":%" PRIu64
    ",\"p999_ns\":%" PRIu64 ",\"max_ns\":%" PRIu64 ",\"timer_ns\":%" PRIu64
    ",\"forwarded\":%" PRIu64 ",\"mismatches\":%" PRIu64
    ",\"build_ms\":%.3f,\"rss_start_kb\":%ld,\"rss_built_kb\":%ld,\"rss_end_kb\":%ld}\n",
    o.label.c_str(), o.impl.c_str(), o.clock.c_str(), o.what.c_str(), o.inputs, o.locks,
    o.timeouts.c_str(), o.prio.c_str(), o.speaker.c_str(), o.lock_rate, o.events,
    st.mean_ns, st.p50, st.p90, st.p99, st.p999, st.max, timer_overhead(),
    forwarded, mismatches, build_ms, rss_start, rss_built, rss_kb());
  return mismatches == 0 ? 0 : 1;
}
