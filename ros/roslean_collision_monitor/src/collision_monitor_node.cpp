// Collision monitor with the decision logic in Lean.
//
// This file is the shim: it reads parameters, creates subscriptions, reads the
// clock and publishes. It contains no decision logic and converts no
// configuration value: parameters go to the Lean core as they were read
// (doubles and floats as bit patterns), and the core converts them, refuses
// what the node does not accept, and says which parameter it refused. A scan
// is handed over as bit patterns of its ranges and of the points they denote;
// a command as six bit patterns. The core answers a command with one of:
// publish nothing, publish the command, publish zero, publish the command
// scaled by a ratio. The shim performs no floating-point test on a command
// and keeps no state between events.
//
// The contract is docs/collision_monitor/SPEC.md, "Shim boundary"; rule (R…)
// and difference (D…) numbers refer to that file. The floating-point work
// that remains here is in conversions.hpp.

#include <algorithm>
#include <array>
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
#include "roslean_collision_monitor/conversions.hpp"
#include "roslean_core/collision_monitor.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace roslean_collision_monitor
{

using roslean::CollisionMonitor;

/// A configuration the node refuses to start with (D14).
struct ConfigError : std::runtime_error
{
  using std::runtime_error::runtime_error;
};

struct ZoneParams
{
  std::string name;
};

class CollisionMonitorNode : public rclcpp::Node
{
public:
  CollisionMonitorNode()
  : Node(
      "collision_monitor", "",
      rclcpp::NodeOptions().allow_undeclared_parameters(true)
      .automatically_declare_parameters_from_overrides(true))
  {
    // Everything is read, checked and handed to the core before any
    // subscription exists.
    const auto cmd_vel_in_topic = get_or<std::string>("cmd_vel_in_topic", "cmd_vel_smoothed");
    const auto cmd_vel_out_topic = get_or<std::string>("cmd_vel_out_topic", "cmd_vel");
    const bool stamped = get_or<bool>("enable_stamped_cmd_vel", false);
    refuse_unsupported_node_parameters();

    base_frame_id_ = get_or<std::string>("base_frame_id", "base_footprint");

    CollisionMonitor::Builder builder;
    builder.set_stop_pub_timeout(get_or<double>("stop_pub_timeout", 1.0));
    const std::string scan_topic = read_source(builder, get_or<double>("source_timeout", 2.0));
    read_zones(builder);

    monitor_.emplace(builder.build());
    if (!monitor_->ok()) {
      // The core refuses the configuration. Ask it why, name the parameters,
      // and do not run. (Were the node to run all the same, the core would
      // answer every command with a stop.)
      throw ConfigError(explain_refusal());
    }

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
    if (stamped) {
      pub_stamped_ = create_publisher<geometry_msgs::msg::TwistStamped>(cmd_vel_out_topic, qos);
      cmd_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        cmd_vel_in_topic, qos,
        [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr msg) {
          geometry_msgs::msg::TwistStamped out;
          if (on_cmd(msg->twist, out.twist)) {
            out.header = msg->header;
            pub_stamped_->publish(out);
          }
        });
    } else {
      pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_out_topic, qos);
      cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_in_topic, qos,
        [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
          geometry_msgs::msg::Twist out;
          if (on_cmd(*msg, out)) {
            pub_->publish(out);
          }
        });
    }
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {on_scan(*msg);});

    RCLCPP_INFO(
      get_logger(), "Monitoring '%s' -> '%s' (%s) with scans from '%s' and %zu zone(s).",
      cmd_vel_in_topic.c_str(), cmd_vel_out_topic.c_str(),
      stamped ? "TwistStamped" : "Twist", scan_topic.c_str(), zones_.size());
  }

private:
  // ---- events ------------------------------------------------------------

  // One clock reading per event (D7). The core takes unsigned nanoseconds.
  uint64_t now_ns()
  {
    return static_cast<uint64_t>(std::max<rcl_time_point_value_t>(now().nanoseconds(), 0));
  }

  void on_scan(const sensor_msgs::msg::LaserScan & msg)
  {
    const uint64_t t = now_ns();
    const int64_t stamp_ns =
      static_cast<int64_t>(msg.header.stamp.sec) * 1000000000 + msg.header.stamp.nanosec;
    // D11: there is no transform lookup. The points of a scan can be placed
    // in the base frame only if the scan is in that frame already, or if the
    // sensor's pose was configured. The comparison is made here; what an
    // unusable scan means is decided by the core (the source has no data).
    const bool usable = sensor_pose_set_ || msg.header.frame_id == base_frame_id_;
    if (!usable) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Scan is in frame '%s', not in the base frame '%s', and '%s.roslean_sensor_pose' is not "
        "set: the scan is not used, and the source counts as having no data.",
        msg.header.frame_id.c_str(), base_frame_id_.c_str(), source_name_.c_str());
    }
    const auto beams = encode_scan(msg.angle_min, msg.angle_increment, msg.ranges, sensor_pose_);
    monitor_->on_scan(
      t, stamp_ns, usable, bits_of(msg.range_min), bits_of(msg.range_max), beams.data(),
      beams.size());
  }

  /// Hands the command to the core and fills `out` with what it says to
  /// publish. Returns false if nothing is to be published.
  bool on_cmd(const geometry_msgs::msg::Twist & cmd, geometry_msgs::msg::Twist & out)
  {
    const uint64_t t = now_ns();
    const auto decision = monitor_->on_cmd(
      t, bits_of(cmd.linear.x), bits_of(cmd.linear.y), bits_of(cmd.linear.z),
      bits_of(cmd.angular.x), bits_of(cmd.angular.y), bits_of(cmd.angular.z));
    // `out` is a new message: linear.z, angular.x and angular.y stay 0 (R16).
    switch (decision.kind) {
      case CollisionMonitor::OutputKind::Pass:
        out.linear.x = cmd.linear.x;
        out.linear.y = cmd.linear.y;
        out.angular.z = cmd.angular.z;
        return true;
      case CollisionMonitor::OutputKind::Slowdown:
        out.linear.x = scale(cmd.linear.x, decision.ratio_bits);
        out.linear.y = scale(cmd.linear.y, decision.ratio_bits);
        out.angular.z = scale(cmd.angular.z, decision.ratio_bits);
        RCLCPP_DEBUG(
          get_logger(), "Robot to slow down due to %s", zone_name(decision.zone).c_str());
        return true;
      case CollisionMonitor::OutputKind::StopZone:
        RCLCPP_DEBUG(get_logger(), "Robot to stop due to %s", zone_name(decision.zone).c_str());
        return true;
      case CollisionMonitor::OutputKind::StopSource:
        RCLCPP_DEBUG(get_logger(), "Robot to stop due to invalid source");
        return true;
      case CollisionMonitor::OutputKind::Silent:
        return false;
    }
    // Not a kind the core can return. Publish nothing.
    RCLCPP_ERROR(get_logger(), "unknown output kind from the core");
    return false;
  }

  std::string zone_name(uint32_t k) const
  {
    return k < zones_.size() ? zones_[k].name : "zone " + std::to_string(k);
  }

  // ---- parameters --------------------------------------------------------

  /// The value of a parameter, or nothing if it is not set. A value of the
  /// wrong type is fatal, as upstream (R17).
  template<typename T>
  std::optional<T> get_opt(const std::string & name)
  {
    rclcpp::Parameter param;
    if (!get_parameter(name, param) || param.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
      return std::nullopt;
    }
    try {
      return param.get_value<T>();
    } catch (const std::exception & e) {
      throw ConfigError("parameter '" + name + "' has the wrong type: " + e.what());
    }
  }

  template<typename T>
  T get_or(const std::string & name, const T & fallback)
  {
    return get_opt<T>(name).value_or(fallback);
  }

  bool is_set(const std::string & name)
  {
    rclcpp::Parameter param;
    return get_parameter(name, param) && param.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET;
  }

  [[noreturn]] void refuse(const std::string & name, const std::string & why)
  {
    throw ConfigError("parameter '" + name + "': " + why);
  }

  // SPEC, "Excluded features", node level.
  void refuse_unsupported_node_parameters()
  {
    if (!get_or<std::string>("state_topic", "").empty()) {
      refuse("state_topic", "the state topic is not provided; leave it empty or unset (D13)");
    }
    const auto shift = get_opt<bool>("base_shift_correction");
    if (!shift.has_value() || *shift) {
      refuse(
        "base_shift_correction", "must be set to false explicitly: scan points are not "
        "corrected for the robot's motion, and upstream's default is true (D11)");
    }
    if (get_or<bool>("use_realtime_priority", false)) {
      refuse("use_realtime_priority", "real-time priority is not supported");
    }
    for (const char * name : {"odom_frame_id", "transform_tolerance"}) {
      if (is_set(name)) {
        RCLCPP_WARN(
          get_logger(), "Parameter '%s' is ignored: this node performs no transform lookup. "
          "A scan must be in the frame 'base_frame_id', or the sensor's pose must be given by "
          "'<source>.roslean_sensor_pose' (D11).", name);
      }
    }
  }

  /// Reads the one scan source and returns its topic.
  std::string read_source(CollisionMonitor::Builder & builder, double node_source_timeout)
  {
    const auto sources = get_opt<std::vector<std::string>>("observation_sources");
    if (!sources.has_value()) {
      refuse("observation_sources", "required");
    }
    if (sources->size() != 1) {
      refuse(
        "observation_sources", "exactly one source is supported; " +
        std::to_string(sources->size()) + " given");
    }
    source_name_ = sources->front();
    const std::string & s = source_name_;
    const auto type = get_or<std::string>(s + ".type", "scan");
    if (type != "scan") {
      refuse(s + ".type", "only 'scan' sources are supported; got '" + type + "'");
    }
    const auto topic = get_or<std::string>(s + ".topic", "scan");
    const bool enabled = get_or<bool>(s + ".enabled", true);
    // The source's own timeout, if set; otherwise the core derives it from
    // the node's, as upstream's default does (R17).
    builder.set_source(enabled, node_source_timeout, get_opt<double>(s + ".source_timeout"));

    const auto pose = get_opt<std::vector<double>>(s + ".roslean_sensor_pose");
    if (pose.has_value()) {
      if (pose->size() != 3 ||
        !std::all_of(pose->begin(), pose->end(), [](double v) {return std::isfinite(v);}))
      {
        refuse(s + ".roslean_sensor_pose", "must be three finite numbers [x, y, yaw]");
      }
      sensor_pose_ = SensorPose{(*pose)[0], (*pose)[1], (*pose)[2]};
      sensor_pose_set_ = true;
    }
    return topic;
  }

  void read_zones(CollisionMonitor::Builder & builder)
  {
    const auto names = get_opt<std::vector<std::string>>("polygons");
    if (!names.has_value()) {
      refuse("polygons", "required");
    }
    for (const auto & z : *names) {
      read_zone(builder, z);
    }
  }

  // Zones are added in the order of `polygons`; that order is the zone index
  // in the core's answers.
  void read_zone(CollisionMonitor::Builder & builder, const std::string & z)
  {
    const auto type = get_opt<std::string>(z + ".type");
    if (!type.has_value()) {
      refuse(z + ".type", "required ('circle' or 'polygon')");
    }
    if (*type != "circle" && *type != "polygon") {
      refuse(z + ".type", "only 'circle' and 'polygon' are supported; got '" + *type + "'");
    }
    const auto action_type = get_opt<std::string>(z + ".action_type");
    if (!action_type.has_value()) {
      refuse(z + ".action_type", "required ('stop' or 'slowdown')");
    }
    if (*action_type != "stop" && *action_type != "slowdown") {
      refuse(
        z + ".action_type", "only 'stop' and 'slowdown' are supported; got '" + *action_type + "'");
    }
    const auto action = *action_type == "stop" ?
      CollisionMonitor::ZoneAction::Stop : CollisionMonitor::ZoneAction::Slowdown;
    if (is_set(z + ".max_points")) {
      refuse(z + ".max_points", "deprecated and not supported; use min_points (D10)");
    }
    const auto sources_names = get_opt<std::vector<std::string>>(z + ".sources_names");
    if (sources_names.has_value() && *sources_names != std::vector<std::string>{source_name_}) {
      refuse(z + ".sources_names", "must be the one source '" + source_name_ + "' or unset");
    }
    if (get_or<bool>(z + ".visualize", false) || is_set(z + ".polygon_pub_topic")) {
      RCLCPP_WARN(
        get_logger(), "Zone '%s': 'visualize' and 'polygon_pub_topic' are ignored; the zone's "
        "shape is not published (D13).", z.c_str());
    }
    const int64_t min_points = get_or<int64_t>(z + ".min_points", 4);
    const double ratio = get_or<double>(z + ".slowdown_ratio", 0.5);
    const bool enabled = get_or<bool>(z + ".enabled", true);

    if (*type == "circle") {
      const auto radius = get_opt<double>(z + ".radius");
      if (!radius.has_value()) {
        refuse(z + ".radius", "required: a circle whose radius comes from a topic is not supported");
      }
      builder.add_circle(*radius, action, ratio, min_points, enabled);
    } else {
      const auto points = get_opt<std::string>(z + ".points");
      if (!points.has_value()) {
        refuse(z + ".points", "required: a polygon that comes from a topic is not supported");
      }
      std::vector<std::array<float, 2>> vertices;
      std::string error;
      if (!parse_points(*points, vertices, error)) {
        refuse(z + ".points", error);
      }
      builder.add_polygon(action, ratio, min_points, enabled);
      for (const auto & v : vertices) {
        builder.add_vertex(v[0], v[1]);
      }
    }
    zones_.push_back(ZoneParams{z});
  }

  /// Why the core refuses the configuration, naming every refused
  /// parameter. The reasons are the core's; this only words them.
  std::string explain_refusal()
  {
    std::string why;
    const auto add = [&why](const std::string & name, const std::string & reason) {
        why += (why.empty() ? "" : "; ") + ("parameter '" + name + "': " + reason);
      };
    const char * timeout = "a timeout must be finite, not negative and below 9.2e9 s (D9)";
    switch (monitor_->error()) {
      case CollisionMonitor::ConfigError::SourceTimeout:
        add("source_timeout", timeout);
        break;
      case CollisionMonitor::ConfigError::OwnSourceTimeout:
        add(source_name_ + ".source_timeout", timeout);
        break;
      case CollisionMonitor::ConfigError::StopPubTimeout:
        add("stop_pub_timeout", timeout);
        break;
      case CollisionMonitor::ConfigError::Zone:
      case CollisionMonitor::ConfigError::None:
        break;
    }
    for (uint32_t k = 0; k < zones_.size(); ++k) {
      const std::string & z = zones_[k].name;
      switch (monitor_->zone_error(k)) {
        case CollisionMonitor::ZoneError::None:
          break;
        case CollisionMonitor::ZoneError::Radius:
          add(z + ".radius", "must be finite and below 1000 m in magnitude (D12)");
          break;
        case CollisionMonitor::ZoneError::Points:
          add(
            z + ".points", "a polygon needs at least four vertices, all finite and within "
            "999.997 m of both axes (R10, D12)");
          break;
        case CollisionMonitor::ZoneError::Ratio:
          add(z + ".slowdown_ratio", "must not be negative (D5)");
          break;
        case CollisionMonitor::ZoneError::MinPoints:
          add(z + ".min_points", "must be in the range of a 32-bit integer (D10)");
          break;
        case CollisionMonitor::ZoneError::NoSuchZone:
          add(z, "zone unknown to the core");
          break;
      }
    }
    return why.empty() ? "the Lean core does not accept the configuration" : why;
  }

  std::optional<CollisionMonitor> monitor_;
  std::vector<ZoneParams> zones_;  // names, for messages only
  std::string source_name_;
  std::string base_frame_id_;
  SensorPose sensor_pose_;
  bool sensor_pose_set_{false};  // the pose parameter was given explicitly
  rclcpp::SubscriptionBase::SharedPtr cmd_sub_;
  rclcpp::SubscriptionBase::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_stamped_;
};

}  // namespace roslean_collision_monitor

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  int result = EXIT_SUCCESS;
  try {
    roslean::initialize();
    // Single-threaded: the core processes one event at a time (A1), and the
    // Lean runtime is initialised for this thread only.
    rclcpp::spin(std::make_shared<roslean_collision_monitor::CollisionMonitorNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("collision_monitor"), "%s", e.what());
    result = EXIT_FAILURE;
  }
  rclcpp::shutdown();
  return result;
}
