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
// and keeps no state that enters a decision. (It remembers the last answer
// it logged, so that a change of the action is logged once, as upstream
// does; that memory is read by the logging and by nothing else.)
//
// Parameters are read once, here, at start-up (D2). Every later change is
// rejected through the parameter service, and a name that was not given at
// start-up cannot be set at all, so that nobody is told "accepted" for a
// change the node does not act on.
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
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "roslean_collision_monitor/conversions.hpp"
#include "roslean_core/collision_monitor.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace roslean_collision_monitor
{

using roslean::CollisionMonitor;

/// A configuration the node refuses to start with (D14).
struct ConfigError : std::runtime_error
{
  using std::runtime_error::runtime_error;
};

/// What was read for a zone: for messages and warnings only. The core has
/// its own copy, and decides with that.
struct ZoneParams
{
  std::string name;
  bool circle{false};
  bool slowdown{false};
  bool enabled{true};
  double radius{0.0};
  double ratio{0.0};
  int64_t min_points{0};
  std::vector<std::array<float, 2>> vertices;
};

/// Twice the signed area of a polygon whose vertices are whole millimetres
/// below 2^20 in magnitude: exact in 64-bit integers. For a warning only.
inline int64_t twice_area_mm2(const std::vector<std::array<int64_t, 2>> & v)
{
  int64_t sum = 0;
  for (size_t i = 0; i < v.size(); ++i) {
    const auto & a = v[i];
    const auto & b = v[(i + 1) % v.size()];
    sum += a[0] * b[1] - b[0] * a[1];
  }
  return sum;
}

class CollisionMonitorNode : public rclcpp::Node
{
public:
  CollisionMonitorNode()
  : Node(
      "collision_monitor", "",
      // Every parameter given at start-up is declared; no other name can be
      // set later (finding 2 of the review CM3/CM4).
      rclcpp::NodeOptions().allow_undeclared_parameters(false)
      .automatically_declare_parameters_from_overrides(true)
      // D15: rclcpp takes /clock messages in a thread of its own by default,
      // and a message with a negative time makes it throw there, where
      // nothing can catch it: the process ends. Without that thread the
      // message is taken in by the executor that spins this node, and the
      // exception arrives in main(), which goes on (see there).
      .use_clock_thread(false))
  {
    // Everything is read, checked and handed to the core before any
    // subscription exists. A refusal does not end the reading: all of them
    // are collected and reported together.
    const auto cmd_vel_in_topic = get_or<std::string>("cmd_vel_in_topic", "cmd_vel_smoothed");
    const auto cmd_vel_out_topic = get_or<std::string>("cmd_vel_out_topic", "cmd_vel");
    const bool stamped = get_or<bool>("enable_stamped_cmd_vel", false);
    refuse_unsupported_node_parameters();

    base_frame_id_ = get_or<std::string>("base_frame_id", "base_footprint");

    CollisionMonitor::Builder builder;
    stop_pub_timeout_ = get_or<double>("stop_pub_timeout", 1.0);
    builder.set_stop_pub_timeout(stop_pub_timeout_);
    node_source_timeout_ = get_or<double>("source_timeout", 2.0);
    const std::string scan_topic = read_source(builder);
    read_zones(builder);

    monitor_.emplace(builder.build());
    if (!monitor_->ok()) {
      // The core refuses the configuration. Ask it why and name the
      // parameters. (Were the node to run all the same, the core would
      // answer every command with a stop.)
      explain_refusal();
    }
    if (!refusals_.empty() || !monitor_->ok()) {
      throw ConfigError(refusal_message());
    }
    warn_about_what_protects_nothing();

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
    // D15: under simulated time the node looks at /clock itself, beside
    // rclcpp's own subscription, to know whether the message received last
    // is one that rclcpp can use. It reads nothing else from the message;
    // the time still comes from now().
    sim_time_ = get_or<bool>("use_sim_time", false);
    if (sim_time_) {
      clock_sub_ = create_subscription<rosgraph_msgs::msg::Clock>(
        "/clock", rclcpp::ClockQoS(),
        [this](rosgraph_msgs::msg::Clock::ConstSharedPtr msg) {
          set_clock_valid(clock_message_usable(msg->clock.sec));
        });
    }

    // D2: parameters were read once, above. From here on every change is
    // rejected, with the reason, instead of being accepted and ignored.
    on_set_parameters_ = add_on_set_parameters_callback(
      [](const std::vector<rclcpp::Parameter> & changes) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = false;
        result.reason = "parameters of this node are read once, at start-up, and cannot be "
        "changed while it runs (docs/collision_monitor/SPEC.md, D2)";
        if (!changes.empty()) {
          result.reason += "; to change '" + changes.front().get_name() +
          "', restart the node with the new value";
        }
        return result;
      });

    RCLCPP_INFO(
      get_logger(), "Monitoring '%s' -> '%s' (%s) with scans from '%s' and %zu zone(s).",
      cmd_vel_in_topic.c_str(), cmd_vel_out_topic.c_str(),
      stamped ? "TwistStamped" : "Twist", scan_topic.c_str(), zones_.size());
  }

  /// An exception left a callback and was caught in main(). Under simulated
  /// time the one known cause is a /clock message that rclcpp refuses (D15),
  /// so the clock counts as not valid until a usable message arrives; the
  /// node's own look at the message says the same, and whichever of the two
  /// comes first counts.
  void on_callback_exception(const char * what)
  {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "An exception left a callback and was caught; the node goes on: %s", what);
    if (sim_time_) {
      set_clock_valid(false);
    }
  }

private:
  // ---- events ------------------------------------------------------------

  void set_clock_valid(bool valid)
  {
    if (valid == clock_valid_) {
      return;
    }
    clock_valid_ = valid;
    if (valid) {
      RCLCPP_INFO(get_logger(), "The clock is valid again: commands are evaluated as usual.");
    } else {
      RCLCPP_ERROR(
        get_logger(), "A message on /clock carries a negative time, which cannot be used. Until "
        "a usable one arrives every command is answered with zero velocity (D15).");
    }
  }

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
    const bool same_frame = msg.header.frame_id == base_frame_id_;
    const bool usable = sensor_pose_set_ || same_frame;
    if (sensor_pose_set_ && !same_frame) {
      // The scan is used: the configured pose stands in for the transform,
      // and nothing checks that it is the pose of this scan's frame.
      // Once a minute: with a pose this is the normal state of a correctly
      // configured robot, and the point is that it is not forgotten.
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 60000,
        "Scan is in frame '%s', not in the base frame '%s'. It is used all the same, placed by "
        "'%s.roslean_sensor_pose' = [%g, %g, %g]; no transform is looked up and the frame name "
        "is not checked (D11).",
        msg.header.frame_id.c_str(), base_frame_id_.c_str(), source_name_.c_str(),
        sensor_pose_.x, sensor_pose_.y, sensor_pose_.yaw);
    }
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
    // D15: without a valid clock reading there is no time to hand over, and
    // the core is told so; it answers with a stop.
    const auto decision = clock_valid_ ?
      monitor_->on_cmd(
      now_ns(), bits_of(cmd.linear.x), bits_of(cmd.linear.y), bits_of(cmd.linear.z),
      bits_of(cmd.angular.x), bits_of(cmd.angular.y), bits_of(cmd.angular.z)) :
      monitor_->on_cmd_no_clock(
      bits_of(cmd.linear.x), bits_of(cmd.linear.y), bits_of(cmd.linear.z),
      bits_of(cmd.angular.x), bits_of(cmd.angular.y), bits_of(cmd.angular.z));
    log_change(decision);
    // What to publish for the answer is apply_answer (conversions.hpp, with
    // its own test), which takes the kind as the number the core returned.
    static_assert(
      static_cast<uint8_t>(CollisionMonitor::OutputKind::Silent) == 0 &&
      static_cast<uint8_t>(CollisionMonitor::OutputKind::Pass) == 1 &&
      static_cast<uint8_t>(CollisionMonitor::OutputKind::Slowdown) == 2 &&
      static_cast<uint8_t>(CollisionMonitor::OutputKind::StopZone) == 3 &&
      static_cast<uint8_t>(CollisionMonitor::OutputKind::StopSource) == 4,
      "OutputKind must have the codes of roslean_cm_out_kind");
    std::array<double, 3> planar{};
    const Answer answer = apply_answer(
      static_cast<uint8_t>(decision.kind), decision.ratio_bits,
      {cmd.linear.x, cmd.linear.y, cmd.angular.z}, planar);
    if (answer == Answer::Nothing) {
      // R2 (a component is not finite) or R15 (zero was published for
      // stop_pub_timeout already).
      RCLCPP_DEBUG(get_logger(), "Command not answered");
      return false;
    }
    if (answer == Answer::PublishUnknownKind) {
      // Unreachable: the kind is `kindOf` of the core's output
      // (core/ROSleanCore/CollisionMonitorFFI.lean), a total function whose
      // values are 0 to 4. Were a core and a shim of different versions ever
      // linked together, the answer the shim does not understand is treated
      // as a stop, not as silence: `planar` is all zeros.
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "unknown output kind %u from the core: publishing zero velocity",
        static_cast<unsigned>(decision.kind));
    }
    // `out` is a new message: linear.z, angular.x and angular.y stay 0 (R16).
    out.linear.x = planar[0];
    out.linear.y = planar[1];
    out.angular.z = planar[2];
    return true;
  }

  /// Logs the action the core demands when it differs from the one logged
  /// last, as upstream does on each change of its action (and in upstream's
  /// words). The text is built from the three numbers the core returned:
  /// the kind, the zone index and the ratio's bit pattern; they are compared
  /// as integers with those of the last logged answer. A command the core
  /// leaves unanswered is not a change. Nothing here feeds back into what is
  /// published.
  void log_change(const CollisionMonitor::Output & d)
  {
    if (d.kind == CollisionMonitor::OutputKind::Silent) {
      return;
    }
    if (logged_ && logged_kind_ == d.kind && logged_zone_ == d.zone &&
      logged_ratio_bits_ == d.ratio_bits)
    {
      return;
    }
    logged_ = true;
    logged_kind_ = d.kind;
    logged_zone_ = d.zone;
    logged_ratio_bits_ = d.ratio_bits;
    switch (d.kind) {
      case CollisionMonitor::OutputKind::Pass:
        RCLCPP_INFO(get_logger(), "Robot to continue normal operation");
        break;
      case CollisionMonitor::OutputKind::Slowdown:
        RCLCPP_INFO(
          get_logger(), "Robot to slowdown for %f percents due to %s polygon",
          double_of_bits(d.ratio_bits) * 100, zone_name(d.zone).c_str());
        break;
      case CollisionMonitor::OutputKind::StopZone:
        RCLCPP_INFO(get_logger(), "Robot to stop due to %s polygon", zone_name(d.zone).c_str());
        break;
      case CollisionMonitor::OutputKind::StopSource:
        RCLCPP_WARN(
          get_logger(), "Robot to stop due to invalid source: no scan was received yet, or the "
          "last one is older than the source timeout, or it was in a frame that cannot be used");
        break;
      case CollisionMonitor::OutputKind::Silent:
        break;
    }
  }

  std::string zone_name(uint32_t k) const
  {
    return k < zones_.size() ? zones_[k].name : "zone " + std::to_string(k);
  }

  // ---- parameters --------------------------------------------------------

  /// The value of a parameter, or nothing if it is not set. A value of the
  /// wrong type is refused, as upstream (R17).
  template<typename T>
  std::optional<T> get_opt(const std::string & name)
  {
    rclcpp::Parameter param;
    if (!is_set(name, &param)) {
      return std::nullopt;
    }
    try {
      return param.get_value<T>();
    } catch (const std::exception & e) {
      refuse(name, std::string("has the wrong type: ") + e.what());
      return std::nullopt;
    }
  }

  template<typename T>
  T get_or(const std::string & name, const T & fallback)
  {
    return get_opt<T>(name).value_or(fallback);
  }

  bool is_set(const std::string & name, rclcpp::Parameter * out = nullptr)
  {
    rclcpp::Parameter param;
    if (!has_parameter(name) || !get_parameter(name, param) ||
      param.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET)
    {
      return false;
    }
    if (out) {
      *out = param;
    }
    return true;
  }

  /// Records a refusal. The node does not start if there is one; all of them
  /// are reported together (one per parameter: the first reason is kept).
  void refuse(const std::string & name, const std::string & why)
  {
    if (refused_names_.insert(name).second) {
      refusals_.push_back("parameter '" + name + "': " + why);
    }
  }

  bool refused(const std::string & name) const {return refused_names_.count(name) != 0;}

  std::string refusal_message() const
  {
    if (refusals_.empty()) {
      return "the Lean core does not accept the configuration";
    }
    std::string all = std::to_string(refusals_.size()) +
      (refusals_.size() == 1 ? " parameter refused: " : " parameters refused: ");
    for (size_t i = 0; i < refusals_.size(); ++i) {
      all += (i == 0 ? "" : "; ") + refusals_[i];
    }
    return all;
  }

  // SPEC, "Excluded features", node level.
  void refuse_unsupported_node_parameters()
  {
    if (!get_or<std::string>("state_topic", "").empty()) {
      refuse("state_topic", "the state topic is not provided; leave it empty or unset (D13)");
    }
    const auto shift = get_opt<bool>("base_shift_correction");
    if ((!shift.has_value() || *shift) && !refused("base_shift_correction")) {
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
  std::string read_source(CollisionMonitor::Builder & builder)
  {
    const auto sources = get_opt<std::vector<std::string>>("observation_sources");
    if (!sources.has_value()) {
      refuse("observation_sources", "required");
      return "scan";
    }
    if (sources->size() != 1) {
      refuse(
        "observation_sources", "exactly one source is supported; " +
        std::to_string(sources->size()) + " given");
      return "scan";
    }
    source_name_ = sources->front();
    const std::string & s = source_name_;
    const auto type = get_or<std::string>(s + ".type", "scan");
    if (type != "scan") {
      refuse(s + ".type", "only 'scan' sources are supported; got '" + type + "'");
    }
    const auto topic = get_or<std::string>(s + ".topic", "scan");
    source_enabled_ = get_or<bool>(s + ".enabled", true);
    // The source's own timeout, if set; otherwise the core derives it from
    // the node's, as upstream's default does (R17).
    own_source_timeout_ = get_opt<double>(s + ".source_timeout");
    builder.set_source(source_enabled_, node_source_timeout_, own_source_timeout_);

    const auto pose = get_opt<std::vector<double>>(s + ".roslean_sensor_pose");
    if (pose.has_value()) {
      if (pose->size() != 3 ||
        !std::all_of(pose->begin(), pose->end(), [](double v) {return std::isfinite(v);}))
      {
        refuse(s + ".roslean_sensor_pose", "must be three finite numbers [x, y, yaw]");
      } else {
        sensor_pose_ = SensorPose{(*pose)[0], (*pose)[1], (*pose)[2]};
        sensor_pose_set_ = true;
        // Finding 1 of the review CM3/CM4: this switches the frame check
        // off, so it is said once, loudly, with the pose in use.
        RCLCPP_WARN(
          get_logger(), "'%s.roslean_sensor_pose' is set: scans are used whatever their "
          "frame_id, placed in '%s' by the fixed pose [x, y, yaw] = [%g, %g, %g] (metres, "
          "radians). No transform is looked up and the frame of a scan is not checked (D11); a "
          "wrong pose puts obstacles in the wrong place.",
          s.c_str(), base_frame_id_.c_str(), sensor_pose_.x, sensor_pose_.y, sensor_pose_.yaw);
      }
    }
    return topic;
  }

  void read_zones(CollisionMonitor::Builder & builder)
  {
    const auto names = get_opt<std::vector<std::string>>("polygons");
    if (!names.has_value()) {
      refuse("polygons", "required");
      return;
    }
    for (const auto & z : *names) {
      read_zone(builder, z);
    }
  }

  // Zones are added in the order of `polygons`; that order is the zone index
  // in the core's answers. A zone that the shim refuses is not added (the
  // node does not start in that case).
  void read_zone(CollisionMonitor::Builder & builder, const std::string & z)
  {
    bool complete = true;
    const auto type = get_opt<std::string>(z + ".type");
    if (!type.has_value()) {
      refuse(z + ".type", "required ('circle' or 'polygon')");
      complete = false;
    } else if (*type != "circle" && *type != "polygon") {
      refuse(z + ".type", "only 'circle' and 'polygon' are supported; got '" + *type + "'");
      complete = false;
    }
    const auto action_type = get_opt<std::string>(z + ".action_type");
    if (!action_type.has_value()) {
      refuse(z + ".action_type", "required ('stop' or 'slowdown')");
      complete = false;
    } else if (*action_type != "stop" && *action_type != "slowdown") {
      refuse(
        z + ".action_type", "only 'stop' and 'slowdown' are supported; got '" + *action_type + "'");
      complete = false;
    }
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
    ZoneParams zone;
    zone.name = z;
    zone.min_points = get_or<int64_t>(z + ".min_points", 4);
    zone.ratio = get_or<double>(z + ".slowdown_ratio", 0.5);
    zone.enabled = get_or<bool>(z + ".enabled", true);
    zone.slowdown = action_type.has_value() && *action_type == "slowdown";
    const auto action = zone.slowdown ?
      CollisionMonitor::ZoneAction::Slowdown : CollisionMonitor::ZoneAction::Stop;

    if (type.has_value() && *type == "circle") {
      const auto radius = get_opt<double>(z + ".radius");
      if (!radius.has_value()) {
        refuse(z + ".radius", "required: a circle whose radius comes from a topic is not supported");
        return;
      }
      if (!complete) {
        return;
      }
      zone.circle = true;
      zone.radius = *radius;
      builder.add_circle(*radius, action, zone.ratio, zone.min_points, zone.enabled);
    } else if (type.has_value() && *type == "polygon") {
      const auto points = get_opt<std::string>(z + ".points");
      if (!points.has_value()) {
        refuse(z + ".points", "required: a polygon that comes from a topic is not supported");
        return;
      }
      std::string error;
      if (!parse_points(*points, zone.vertices, error)) {
        refuse(z + ".points", error);
        return;
      }
      if (!complete) {
        return;
      }
      builder.add_polygon(action, zone.ratio, zone.min_points, zone.enabled);
      for (const auto & v : zone.vertices) {
        builder.add_vertex(v[0], v[1]);
      }
    } else {
      return;
    }
    zones_.push_back(zone);
  }

  /// Why the core refuses the configuration: records a refusal for every
  /// parameter the core does not accept. The reasons are the core's (its
  /// conversions and its error codes); this only words them.
  void explain_refusal()
  {
    const char * timeout = "a timeout must be finite, not negative and below 9.2e9 s (D9)";
    // Each timeout is asked about on its own, so that all of them are named
    // and not only the first.
    if (!CollisionMonitor::timeout_ns(node_source_timeout_).has_value()) {
      refuse("source_timeout", timeout);
    }
    if (own_source_timeout_.has_value() &&
      !CollisionMonitor::timeout_ns(*own_source_timeout_).has_value())
    {
      refuse(source_name_ + ".source_timeout", timeout);
    }
    if (!CollisionMonitor::timeout_ns(stop_pub_timeout_).has_value()) {
      refuse("stop_pub_timeout", timeout);
    }
    for (uint32_t k = 0; k < zones_.size(); ++k) {
      const std::string & z = zones_[k].name;
      switch (monitor_->zone_error(k)) {
        case CollisionMonitor::ZoneError::None:
          break;
        case CollisionMonitor::ZoneError::Radius:
          refuse(
            z + ".radius", "must be finite and at most 999.999 m in magnitude: the zone is the "
            "radius rounded up to a whole millimetre plus 1 mm, and that must not exceed 1000 m "
            "(the double written 999.999 lies just above the bound and is refused; 999.9989 is "
            "accepted) (D4, D12)");
          break;
        case CollisionMonitor::ZoneError::Points:
          refuse(
            z + ".points", "a polygon needs at least four vertices, all finite and within "
            "999.997 m of both axes (R10, D12)");
          break;
        case CollisionMonitor::ZoneError::Ratio:
          refuse(z + ".slowdown_ratio", "must not be negative (D5)");
          break;
        case CollisionMonitor::ZoneError::MinPoints:
          refuse(z + ".min_points", "must be in the range of a 32-bit integer (D10)");
          break;
        case CollisionMonitor::ZoneError::NoSuchZone:
          refuse(z, "zone unknown to the core");
          break;
      }
    }
    if (refusals_.empty()) {
      // Not reachable through this file: the source is always set before the
      // zones, and vertices are added only after their polygon.
      refusals_.push_back(
        "the Lean core does not accept the configuration (code " +
        std::to_string(static_cast<unsigned>(monitor_->error())) + ")");
    }
  }

  /// Finding 7c of the review CM3/CM4: a configuration that is accepted but
  /// protects nothing, or less than it seems to, is said so at start-up.
  /// These are warnings: whether to refuse such configurations is an open
  /// decision (SPEC, "Open decisions"). Nothing here changes what the node
  /// does.
  void warn_about_what_protects_nothing()
  {
    const auto warn = [this](const std::string & what) {
        RCLCPP_WARN(get_logger(), "This configuration protects nothing here: %s", what.c_str());
      };
    if (zones_.empty()) {
      warn("'polygons' is empty: there is no zone, so no obstacle stops or slows the robot.");
    }
    if (!source_enabled_) {
      warn(
        "'" + source_name_ + ".enabled' is false: scans are not used, no zone can be triggered "
        "by an obstacle, and missing data does not stop the robot (R6).");
    } else if (CollisionMonitor::source_timeout_ns(node_source_timeout_, own_source_timeout_) ==
      std::optional<int64_t>(0))
    {
      warn(
        "the source timeout is 0: a scan is used however old it is, and with no scan (or after "
        "a scan in an unusable frame) commands pass as if nothing were near (R7, R8).");
    }
    for (const auto & zone : zones_) {
      const std::string z = "zone '" + zone.name + "': ";
      if (zone.min_points <= 0) {
        warn(
          z + "'min_points' is " + std::to_string(zone.min_points) + ": the zone triggers on " +
          "every command while it is enabled, with or without an obstacle (R11).");
      }
      if (zone.slowdown && !(zone.ratio < 1.0)) {
        warn(
          z + "'slowdown_ratio' is " + std::to_string(zone.ratio) + ", not below 1: the zone "
          "never slows the robot down (R13).");
      }
      if (zone.circle && zone.radius == 0.0) {
        warn(z + "'radius' is 0: the zone is a disc of 1 mm around the origin of the base frame.");
      }
      if (!zone.circle) {
        // The vertices as the core sees them, in whole millimetres.
        std::vector<std::array<int64_t, 2>> mm;
        for (const auto & v : zone.vertices) {
          const auto x = CollisionMonitor::vertex_mm(v[0]);
          const auto y = CollisionMonitor::vertex_mm(v[1]);
          if (x.has_value() && y.has_value() && std::llabs(*x) < (1 << 20) &&
            std::llabs(*y) < (1 << 20))
          {
            mm.push_back({*x, *y});
          }
        }
        if (mm.size() == zone.vertices.size() && twice_area_mm2(mm) == 0) {
          warn(
            z + "the polygon of 'points' has no area (its vertices lie on one line, or repeat): "
            "only points within a few millimetres of that line count as inside.");
        }
      }
    }
  }

  std::optional<CollisionMonitor> monitor_;
  std::vector<ZoneParams> zones_;  // as read, for messages only
  std::vector<std::string> refusals_;  // every reason not to start
  std::set<std::string> refused_names_;
  // Raw values as read, for messages only.
  double node_source_timeout_{0.0};
  std::optional<double> own_source_timeout_;
  double stop_pub_timeout_{0.0};
  bool source_enabled_{true};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_;
  // The answer logged last (log_change). Not read by anything else.
  bool logged_{false};
  CollisionMonitor::OutputKind logged_kind_{CollisionMonitor::OutputKind::Silent};
  uint32_t logged_zone_{0};
  uint64_t logged_ratio_bits_{0};
  // D15. `clock_valid_` selects which of the core's two command calls is
  // made; it is false only while the /clock message received last carried a
  // negative time. Always true without simulated time.
  bool sim_time_{false};
  bool clock_valid_{true};
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub_;
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
    auto node = std::make_shared<roslean_collision_monitor::CollisionMonitorNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    // D15: no topic content may end the node. An exception that leaves a
    // callback (rclcpp's own /clock callback throws on a negative time) ends
    // spin(); the message that caused it has been taken, so spinning again
    // goes on with the next one. A refused configuration is thrown by the
    // constructor above, outside this loop, and still ends the process.
    while (rclcpp::ok()) {
      try {
        executor.spin();
      } catch (const std::exception & e) {
        node->on_callback_exception(e.what());
      }
    }
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("collision_monitor"), "%s", e.what());
    result = EXIT_FAILURE;
  }
  rclcpp::shutdown();
  return result;
}
