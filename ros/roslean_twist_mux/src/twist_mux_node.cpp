// twist_mux with the arbitration logic in Lean.
//
// This file is the shim: it reads parameters, creates subscriptions, reads the
// clock and publishes. It contains no arbitration logic. Each callback passes
// (handle index, node-clock time in ns) to the Lean core and forwards the
// message iff the core says so. See docs/SPEC.md for the rules (R…) and the
// deliberate differences from upstream (D…).

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "roslean_core/mux.hpp"
#include "std_msgs/msg/bool.hpp"

namespace roslean_twist_mux
{

struct HandleParams
{
  std::string name;   // full parameter prefix, e.g. "topics.navigation"
  std::string topic;
  uint64_t timeout_ns;  // 0 = never expires
  uint64_t priority;    // clamped to 0..255
};

class TwistMux : public rclcpp::Node
{
public:
  TwistMux()
  : Node(
      "twist_mux", "",
      rclcpp::NodeOptions().allow_undeclared_parameters(true)
      .automatically_declare_parameters_from_overrides(true))
  {
    bool use_stamped = true;
    rclcpp::Parameter param;
    if (get_parameter("use_stamped", param)) {
      use_stamped = param.get_value<bool>();
    } else {
      RCLCPP_INFO(get_logger(), "\"use_stamped\" is not set, defaulting to \"true\".");
    }

    const auto inputs = read_handles("topics");
    const auto locks = read_handles("locks");

    // Indices are assigned in list order, separately for inputs and locks.
    roslean::Mux::Builder builder;
    for (const auto & h : inputs) {
      builder.add_input(h.priority, h.timeout_ns);
    }
    for (const auto & h : locks) {
      builder.add_lock(h.priority, h.timeout_ns);
    }
    mux_.emplace(builder.build());  // throws roslean::ConfigError

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
    if (use_stamped) {
      pub_stamped_ = create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel_out", qos);
    } else {
      pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel_out", qos);
    }

    for (uint32_t i = 0; i < inputs.size(); ++i) {
      log_handle("Topic", inputs[i]);
      if (inputs[i].priority == 0) {
        RCLCPP_WARN(
          get_logger(), "Topic handler '%s' with priority 0 will never be selected. "
          "Increase the priority to at least 1.", inputs[i].name.c_str());
      }
      if (use_stamped) {
        subscribe_input<geometry_msgs::msg::TwistStamped>(i, inputs[i].topic, pub_stamped_);
      } else {
        subscribe_input<geometry_msgs::msg::Twist>(i, inputs[i].topic, pub_);
      }
    }
    for (uint32_t j = 0; j < locks.size(); ++j) {
      log_handle("Lock", locks[j]);
      if (locks[j].priority == 0) {
        RCLCPP_WARN(
          get_logger(), "Lock '%s' with priority 0 is redundant.", locks[j].name.c_str());
      }
      subs_.push_back(
        create_subscription<std_msgs::msg::Bool>(
          locks[j].topic, rclcpp::SystemDefaultsQoS(),
          [this, j](std_msgs::msg::Bool::ConstSharedPtr msg) {
            mux_->on_lock(j, msg->data, now_ns());
          }));
    }
  }

private:
  // One clock reading per event (D4). The core takes unsigned nanoseconds.
  uint64_t now_ns()
  {
    return static_cast<uint64_t>(std::max<rcl_time_point_value_t>(now().nanoseconds(), 0));
  }

  template<typename MsgT>
  void subscribe_input(
    uint32_t i, const std::string & topic,
    typename rclcpp::Publisher<MsgT>::SharedPtr pub)
  {
    subs_.push_back(
      create_subscription<MsgT>(
        topic, rclcpp::SystemDefaultsQoS(),
        [this, i, pub](typename MsgT::ConstSharedPtr msg) {
          if (mux_->on_input(i, now_ns())) {
            pub->publish(*msg);
          }
        }));
  }

  template<typename T>
  T fetch(const std::string & name)
  {
    rclcpp::Parameter param;
    if (!get_parameter(name, param)) {
      throw std::runtime_error("could not load parameter '" + name + "'");
    }
    return param.get_value<T>();  // throws on a wrong type: fatal, as upstream (R14)
  }

  // Same discovery as upstream: one handle per parameter prefix under `group`.
  std::vector<HandleParams> read_handles(const std::string & group)
  {
    std::vector<HandleParams> handles;
    for (const auto & prefix : list_parameters({group}, 10).prefixes) {
      HandleParams h;
      h.name = prefix;
      h.topic = fetch<std::string>(prefix + ".topic");
      h.timeout_ns = timeout_to_ns(prefix, fetch<double>(prefix + ".timeout"));
      h.priority = clamp_priority(prefix, fetch<int64_t>(prefix + ".priority"));
      handles.push_back(h);
    }
    return handles;
  }

  // R1, R2: truncate to integer ns as upstream; <= 0 disables expiry.
  // D5: non-finite and out-of-range values are rejected.
  static uint64_t timeout_to_ns(const std::string & name, double seconds)
  {
    if (!std::isfinite(seconds) || seconds >= 9.2e9) {
      throw std::runtime_error(
              "timeout of '" + name + "' must be finite and below 9.2e9 s");
    }
    if (seconds <= 0.0) {
      return 0;
    }
    const auto ns = rclcpp::Duration(std::chrono::duration<double>(seconds)).nanoseconds();
    return ns > 0 ? static_cast<uint64_t>(ns) : 0;
  }

  // R1: clamp to 0..255 with a warning.
  uint64_t clamp_priority(const std::string & name, int64_t priority)
  {
    const int64_t clamped = std::clamp<int64_t>(priority, 0, 255);
    if (clamped != priority) {
      RCLCPP_WARN(
        get_logger(), "The priority of '%s' has been clamped from %lld to %lld.",
        name.c_str(), static_cast<long long>(priority), static_cast<long long>(clamped));
    }
    return static_cast<uint64_t>(clamped);
  }

  void log_handle(const char * kind, const HandleParams & h)
  {
    RCLCPP_INFO(
      get_logger(), "%s handler '%s' subscribed to topic '%s': timeout = %s, priority = %llu.",
      kind, h.name.c_str(), h.topic.c_str(),
      h.timeout_ns > 0 ? (std::to_string(h.timeout_ns * 1e-9) + "s").c_str() : "None",
      static_cast<unsigned long long>(h.priority));
  }

  std::optional<roslean::Mux> mux_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_stamped_;
};

}  // namespace roslean_twist_mux

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  int result = EXIT_SUCCESS;
  try {
    roslean::initialize();
    // Single-threaded: the core processes one event at a time (R16), and the
    // Lean runtime is initialised for this thread only.
    rclcpp::spin(std::make_shared<roslean_twist_mux::TwistMux>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("twist_mux"), "%s", e.what());
    result = EXIT_FAILURE;
  }
  rclcpp::shutdown();
  return result;
}
